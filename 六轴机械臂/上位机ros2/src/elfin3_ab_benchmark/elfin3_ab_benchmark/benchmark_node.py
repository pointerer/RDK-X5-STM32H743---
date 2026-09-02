from __future__ import annotations

import math
import threading
import time
from pathlib import Path
from typing import Dict, Optional, Sequence, Tuple

import rclpy
from action_msgs.msg import GoalStatus
from elfin3_interfaces.action import MoveJ, MovePose
from elfin3_interfaces.msg import PlanningMetrics
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener

from .dataset import load_dataset
from .models import AttemptRecord, PlanningMetricsSnapshot, PoseSpec
from .reporting import write_reports


JOINT_NAMES = (
    "elfin_joint1",
    "elfin_joint2",
    "elfin_joint3",
    "elfin_joint4",
    "elfin_joint5",
    "elfin_joint6",
)


def _finite_or_none(value: float) -> Optional[float]:
    value = float(value)
    return value if math.isfinite(value) else None


class Q0ReferenceMismatch(RuntimeError):
    """A fresh joint-state window is stable but remains outside the q0 limit."""

    def __init__(
        self, reference_error: float, motion_span: float, limiting_joint: str
    ) -> None:
        self.reference_error = reference_error
        self.motion_span = motion_span
        self.limiting_joint = limiting_joint
        super().__init__(
            "stable joint state remains outside q0; "
            f"reference_error={reference_error:.9f} rad "
            f"motion_span={motion_span:.9f} rad "
            f"limiting_joint={limiting_joint}"
        )


class Elfin3AbBenchmark(Node):
    def __init__(self) -> None:
        super().__init__("elfin3_ab_benchmark")
        dataset_path = str(self.declare_parameter("dataset", "").value)
        execution_mode = str(self.declare_parameter("execution_mode", "").value)
        self.output_root = str(self.declare_parameter("output_root", "results").value)
        if not dataset_path:
            raise ValueError("parameter 'dataset' must name a benchmark YAML file")
        self.config = load_dataset(dataset_path, execution_mode)

        self.callback_group = ReentrantCallbackGroup()
        self.move_pose_clients = {
            "move_pose": ActionClient(
                self,
                MovePose,
                "/elfin3_motion/move_pose",
                callback_group=self.callback_group,
            ),
            "move_pose_ptp": ActionClient(
                self,
                MovePose,
                "/elfin3_motion/move_pose_ptp",
                callback_group=self.callback_group,
            ),
        }
        self.move_j_client = ActionClient(
            self,
            MoveJ,
            "/elfin3_motion/move_j",
            callback_group=self.callback_group,
        )
        self.supervisor_client = self.create_client(
            Trigger,
            "/elfin3_supervisor/is_ready",
            callback_group=self.callback_group,
        )
        self.stop_client = self.create_client(
            Trigger,
            "/elfin3_motion/stop",
            callback_group=self.callback_group,
        )

        metrics_qos = QoSProfile(depth=10)
        metrics_qos.reliability = ReliabilityPolicy.RELIABLE
        self.metrics_subscription = self.create_subscription(
            PlanningMetrics,
            "/elfin3_motion/planning_metrics",
            self._metrics_callback,
            metrics_qos,
            callback_group=self.callback_group,
        )
        self.joint_state_subscription = self.create_subscription(
            JointState,
            "/joint_states",
            self._joint_state_callback,
            qos_profile_sensor_data,
            callback_group=self.callback_group,
        )
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(
            self.tf_buffer, self, spin_thread=False
        )

        self.metrics_condition = threading.Condition()
        self.metrics_by_goal_id: Dict[str, PlanningMetricsSnapshot] = {}
        self.last_metrics_snapshot: Optional[PlanningMetricsSnapshot] = None
        self.joint_state_lock = threading.Lock()
        self.latest_joint_positions: Optional[Tuple[float, ...]] = None
        self.latest_joint_state_time = 0.0
        self.active_goal_lock = threading.Lock()
        self.active_goal_handle = None
        self.active_goal_monitored = False
        self.last_tf_success_time = time.monotonic()
        self.abort_event = threading.Event()
        self.runtime_safety_reason = ""
        self.records = []
        self.baseline_q0: Optional[Tuple[float, ...]] = None

        self.safety_timer = self.create_timer(
            self.config.safety_poll_period_sec,
            self._runtime_safety_monitor,
            callback_group=self.callback_group,
        )

    def _joint_state_callback(self, message: JointState) -> None:
        if len(message.name) != len(message.position):
            return
        values = dict(zip(message.name, message.position))
        if any(name not in values for name in JOINT_NAMES):
            return
        positions = tuple(float(values[name]) for name in JOINT_NAMES)
        if not all(math.isfinite(value) for value in positions):
            return
        with self.joint_state_lock:
            self.latest_joint_positions = positions
            self.latest_joint_state_time = time.monotonic()

    def _metrics_callback(self, message: PlanningMetrics) -> None:
        snapshot = PlanningMetricsSnapshot(
            goal_id=message.goal_id,
            algorithm=message.algorithm,
            planning_eligible=bool(message.planning_eligible),
            planning_success=bool(message.planning_success),
            failure_stage=message.failure_stage,
            failure_reason=message.failure_reason,
            total_planning_time_ms=float(message.total_planning_time_ms),
            ompl_planning_time_ms=float(message.ompl_planning_time_ms),
            trajectory_points=int(message.trajectory_points),
            max_adjacent_delta_rad=float(message.max_adjacent_delta_rad),
            ik_method=message.ik_method,
            ik_segments=int(message.ik_segments),
            minimum_tcp_y_m=float(message.minimum_tcp_y_m),
            workspace_safe=bool(message.workspace_safe),
            workspace_y_constraint_enabled=bool(message.workspace_y_constraint_enabled),
            workspace_min_tcp_y_m=float(message.workspace_min_tcp_y_m),
            workspace_y_margin_m=float(message.workspace_y_margin_m),
            fk_samples_evaluated=int(message.fk_samples_evaluated),
            workspace_fk_sample_max_joint_step_rad=float(
                message.workspace_fk_sample_max_joint_step_rad
            ),
        )
        with self.metrics_condition:
            self.metrics_by_goal_id[message.goal_id] = snapshot
            self.metrics_condition.notify_all()

    @staticmethod
    def _goal_id_string(goal_handle) -> str:
        return bytes(goal_handle.goal_id.uuid).hex()

    @staticmethod
    def _wait_future(future, timeout_sec: float, description: str):
        completed = threading.Event()
        future.add_done_callback(lambda _: completed.set())
        if not completed.wait(timeout_sec):
            raise TimeoutError(f"timeout waiting for {description}")
        exception = future.exception()
        if exception is not None:
            raise RuntimeError(f"{description} failed: {exception}")
        return future.result()

    def _wait_metric(self, goal_id: str) -> Optional[PlanningMetricsSnapshot]:
        deadline = time.monotonic() + self.config.metrics_timeout_sec
        with self.metrics_condition:
            while goal_id not in self.metrics_by_goal_id:
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    return None
                self.metrics_condition.wait(remaining)
            return self.metrics_by_goal_id.pop(goal_id)

    def _lookup_tcp_y(self) -> float:
        transform = self.tf_buffer.lookup_transform(
            self.config.base_frame,
            self.config.tool_frame,
            Time(),
            timeout=Duration(seconds=self.config.tf_lookup_timeout_sec),
        )
        tcp_y = float(transform.transform.translation.y)
        if not math.isfinite(tcp_y):
            raise RuntimeError("TCP transform contains a non-finite Y value")
        return tcp_y

    def _wait_for_safe_tcp(self, timeout_sec: float) -> float:
        deadline = time.monotonic() + timeout_sec
        last_error = "TF unavailable"
        while time.monotonic() < deadline and rclpy.ok():
            try:
                tcp_y = self._lookup_tcp_y()
                if tcp_y < self.config.effective_y_min_m:
                    raise RuntimeError(
                        f"TCP y={tcp_y:.6f} m is below configured boundary="
                        f"{self.config.effective_y_min_m:.6f} m"
                    )
                self.last_tf_success_time = time.monotonic()
                return tcp_y
            except TransformException as exception:
                last_error = str(exception)
            time.sleep(0.05)
        raise RuntimeError(f"cannot confirm safe TCP workspace: {last_error}")

    def _runtime_safety_monitor(self) -> None:
        with self.active_goal_lock:
            goal_handle = self.active_goal_handle
            monitored = self.active_goal_monitored
        if goal_handle is None or not monitored or self.abort_event.is_set():
            return
        try:
            tcp_y = self._lookup_tcp_y()
            self.last_tf_success_time = time.monotonic()
            if tcp_y < self.config.effective_y_min_m:
                self._request_safety_stop(
                    f"RUNTIME_Y_UNSAFE: TCP y={tcp_y:.6f} m below "
                    f"{self.config.effective_y_min_m:.6f} m"
                )
        except (TransformException, RuntimeError) as exception:
            if (
                time.monotonic() - self.last_tf_success_time
                > self.config.joint_state_timeout_sec
            ):
                self._request_safety_stop(f"RUNTIME_TF_UNAVAILABLE: {exception}")

    def _request_safety_stop(self, reason: str) -> None:
        if self.abort_event.is_set():
            return
        self.runtime_safety_reason = reason
        self.abort_event.set()
        self.get_logger().error(reason)
        with self.active_goal_lock:
            goal_handle = self.active_goal_handle
        if goal_handle is not None:
            goal_handle.cancel_goal_async()
        if self.stop_client.service_is_ready():
            self.stop_client.call_async(Trigger.Request())

    def _wait_supervisor_ready(self) -> None:
        deadline = time.monotonic() + self.config.supervisor_wait_timeout_sec
        if not self.supervisor_client.wait_for_service(
            timeout_sec=self.config.supervisor_wait_timeout_sec
        ):
            raise RuntimeError(
                "/elfin3_supervisor/is_ready remained unavailable for "
                f"{self.config.supervisor_wait_timeout_sec:.1f} s"
            )
        last_message = ""
        while time.monotonic() < deadline and rclpy.ok():
            request_future = self.supervisor_client.call_async(Trigger.Request())
            try:
                response = self._wait_future(
                    request_future,
                    min(3.0, max(0.1, deadline - time.monotonic())),
                    "supervisor readiness response",
                )
            except (TimeoutError, RuntimeError) as exception:
                request_future.cancel()
                last_message = str(exception)
                remaining = max(0.0, deadline - time.monotonic())
                if remaining > 0.0:
                    self.get_logger().warning(
                        f"{exception}; retrying supervisor readiness query "
                        f"for up to {remaining:.1f} s"
                    )
                    time.sleep(0.1)
                continue
            last_message = response.message
            if response.success:
                return
            time.sleep(0.1)
        raise RuntimeError(
            "supervisor did not become READY within "
            f"{self.config.supervisor_wait_timeout_sec:.1f} s: {last_message}"
        )

    def _make_goal(self, pose: PoseSpec) -> MovePose.Goal:
        goal = MovePose.Goal()
        goal.target_pose = PoseStamped()
        goal.target_pose.header.frame_id = pose.frame_id
        goal.target_pose.pose.position.x = pose.position[0]
        goal.target_pose.pose.position.y = pose.position[1]
        goal.target_pose.pose.position.z = pose.position[2]
        goal.target_pose.pose.orientation.x = pose.orientation[0]
        goal.target_pose.pose.orientation.y = pose.orientation[1]
        goal.target_pose.pose.orientation.z = pose.orientation[2]
        goal.target_pose.pose.orientation.w = pose.orientation[3]
        goal.velocity_scaling = self.config.velocity_scaling
        goal.acceleration_scaling = self.config.acceleration_scaling
        return goal

    def _make_joint_goal(self, joint_positions: Sequence[float]) -> MoveJ.Goal:
        goal = MoveJ.Goal()
        goal.joint_positions = list(joint_positions)
        goal.velocity_scaling = self.config.velocity_scaling
        goal.acceleration_scaling = self.config.acceleration_scaling
        return goal

    def _apply_metrics(
        self, record: AttemptRecord, metrics: Optional[PlanningMetricsSnapshot]
    ) -> None:
        if metrics is None:
            return
        self.last_metrics_snapshot = metrics
        record.planning_metrics_received = True
        record.planning_eligible = metrics.planning_eligible
        record.planning_success = metrics.planning_success
        record.failure_stage = metrics.failure_stage
        record.total_planning_time_ms = _finite_or_none(metrics.total_planning_time_ms)
        record.ompl_planning_time_ms = _finite_or_none(metrics.ompl_planning_time_ms)
        record.trajectory_points = metrics.trajectory_points
        record.max_adjacent_delta_rad = _finite_or_none(
            metrics.max_adjacent_delta_rad
        )
        record.ik_method = metrics.ik_method
        record.ik_segments = metrics.ik_segments
        record.minimum_tcp_y_m = _finite_or_none(metrics.minimum_tcp_y_m)
        record.workspace_safe = metrics.workspace_safe
        record.fk_samples_evaluated = metrics.fk_samples_evaluated
        if not record.detail and metrics.failure_reason:
            record.detail = metrics.failure_reason

    def _classify(self, record: AttemptRecord) -> str:
        if record.safety_stop_requested:
            return self.runtime_safety_reason.split(":", 1)[0]
        if not record.goal_accepted:
            return "GOAL_REJECTED"
        if not record.planning_metrics_received:
            return "METRICS_MISSING"
        if record.planning_eligible is not True:
            return record.failure_stage or "PRECONDITION_FAILED"
        if record.planning_success is not True:
            return record.failure_stage or "PLANNING_FAILED"
        if record.action_success:
            return "SUCCESS"
        if record.result_code == MovePose.Result.NOT_READY:
            return "PRE_EXECUTION_NOT_READY"
        if record.result_code == MovePose.Result.CANCELED:
            return "CANCELED"
        return "EXECUTION_FAILED"

    def _execute_pose(
        self,
        algorithm: str,
        pose: PoseSpec,
        pair_id: str,
        target_id: str,
        category: str,
        repetition: int,
        order_in_pair: int,
        monitor_motion: bool = True,
    ) -> AttemptRecord:
        record = AttemptRecord(
            pair_id=pair_id,
            target_id=target_id,
            category=category,
            repetition=repetition,
            order_in_pair=order_in_pair,
            algorithm=algorithm,
            classification="PENDING",
            target_tcp_y_m=pose.position[1],
        )
        client = self.move_pose_clients[algorithm]
        feedback_stages = []

        def feedback_callback(feedback_message) -> None:
            feedback_stages.append(str(feedback_message.feedback.stage))

        started_at = time.monotonic()
        send_future = client.send_goal_async(
            self._make_goal(pose), feedback_callback=feedback_callback
        )
        try:
            goal_handle = self._wait_future(
                send_future, self.config.action_timeout_sec, f"{algorithm} goal response"
            )
        except (TimeoutError, RuntimeError) as exception:
            record.classification = "GOAL_RESPONSE_TIMEOUT"
            record.detail = str(exception)
            return record

        record.goal_accepted = bool(goal_handle.accepted)
        if not goal_handle.accepted:
            record.classification = "GOAL_REJECTED"
            return record
        record.goal_id = self._goal_id_string(goal_handle)
        self.last_tf_success_time = time.monotonic()
        with self.active_goal_lock:
            self.active_goal_handle = goal_handle
            self.active_goal_monitored = monitor_motion

        try:
            wrapped_result = self._wait_future(
                goal_handle.get_result_async(),
                self.config.action_timeout_sec,
                f"{algorithm} result",
            )
            record.action_status = int(wrapped_result.status)
            result = wrapped_result.result
            record.result_code = int(result.result_code)
            record.moveit_error_code = int(result.moveit_error_code)
            record.detail = str(result.message)
            record.action_success = (
                wrapped_result.status == GoalStatus.STATUS_SUCCEEDED
                and result.result_code == MovePose.Result.SUCCESS
            )
        except (TimeoutError, RuntimeError) as exception:
            goal_handle.cancel_goal_async()
            record.detail = str(exception)
            record.classification = "ACTION_TIMEOUT"
            self.abort_event.set()
        finally:
            record.end_to_end_time_ms = (time.monotonic() - started_at) * 1000.0
            record.feedback_stages = list(feedback_stages)
            with self.active_goal_lock:
                self.active_goal_handle = None
                self.active_goal_monitored = False

        metrics = self._wait_metric(record.goal_id)
        self._apply_metrics(record, metrics)
        record.safety_stop_requested = bool(self.runtime_safety_reason)
        if record.classification == "PENDING":
            record.classification = self._classify(record)
        return record

    def _execute_joint_reset(self, joint_positions: Sequence[float]) -> None:
        send_future = self.move_j_client.send_goal_async(
            self._make_joint_goal(joint_positions)
        )
        goal_handle = self._wait_future(
            send_future,
            self.config.action_timeout_sec,
            "move_j reset goal response",
        )
        if not goal_handle.accepted:
            raise RuntimeError("MoveJ reset goal was rejected")

        self.last_tf_success_time = time.monotonic()
        with self.active_goal_lock:
            self.active_goal_handle = goal_handle
            self.active_goal_monitored = True
        try:
            wrapped_result = self._wait_future(
                goal_handle.get_result_async(),
                self.config.action_timeout_sec,
                "move_j reset result",
            )
        except (TimeoutError, RuntimeError):
            goal_handle.cancel_goal_async()
            self.abort_event.set()
            raise
        finally:
            with self.active_goal_lock:
                self.active_goal_handle = None
                self.active_goal_monitored = False

        if self.abort_event.is_set():
            raise RuntimeError(
                self.runtime_safety_reason or "MoveJ reset aborted by safety monitor"
            )
        result = wrapped_result.result
        if not (
            wrapped_result.status == GoalStatus.STATUS_SUCCEEDED
            and result.result_code == MoveJ.Result.SUCCESS
        ):
            raise RuntimeError(
                "MoveJ reset failed: "
                f"status={wrapped_result.status} result_code={result.result_code} "
                f"message={result.message}"
            )

    def _probe_motion_server_configuration(self) -> None:
        invalid_probe = PoseSpec(
            frame_id="__elfin3_benchmark_invalid_probe__",
            position=self.config.start_pose.position,
            orientation=self.config.start_pose.orientation,
        )
        probe = self._execute_pose(
            "move_pose",
            invalid_probe,
            "CONFIG_PROBE",
            "CONFIG_PROBE",
            "probe",
            0,
            0,
            monitor_motion=False,
        )
        if not probe.planning_metrics_received:
            raise RuntimeError(
                "motion server did not publish PlanningMetrics for the safe invalid-goal probe"
            )
        metrics = self.last_metrics_snapshot
        if metrics is None:
            raise RuntimeError("workspace configuration probe snapshot is unavailable")
        if not metrics.workspace_y_constraint_enabled:
            raise RuntimeError(
                "MotionCommand workspace_y_constraint_enabled is false; refusing motion"
            )
        expected = self.config.effective_y_min_m
        actual = metrics.workspace_min_tcp_y_m + metrics.workspace_y_margin_m
        if not math.isclose(expected, actual, rel_tol=0.0, abs_tol=1.0e-9):
            raise RuntimeError(
                f"workspace boundary mismatch: dataset={expected:.9f} m "
                f"motion_server={actual:.9f} m"
            )
        if not math.isclose(
            self.config.workspace_fk_sample_max_joint_step_rad,
            metrics.workspace_fk_sample_max_joint_step_rad,
            rel_tol=0.0,
            abs_tol=1.0e-12,
        ):
            raise RuntimeError(
                "workspace FK sampling step differs between dataset and MotionCommand"
            )

    def _wait_joint_state_at_q0(
        self, expected_q0: Optional[Sequence[float]]
    ) -> Tuple[Tuple[float, ...], float]:
        deadline = time.monotonic() + self.config.q0_wait_timeout_sec
        window_started_at: Optional[float] = None
        window_min: Optional[list[float]] = None
        window_max: Optional[list[float]] = None
        last_observed_sample_time = 0.0
        last_reference_error = math.inf if expected_q0 is not None else 0.0
        last_motion_span = math.inf
        limiting_joint = "unknown"
        expected = tuple(expected_q0) if expected_q0 is not None else None

        def reset_window(
            positions: Tuple[float, ...], sample_time: float
        ) -> None:
            nonlocal window_started_at, window_min, window_max
            window_started_at = sample_time
            window_min = list(positions)
            window_max = list(positions)

        while time.monotonic() < deadline and rclpy.ok():
            now = time.monotonic()
            with self.joint_state_lock:
                positions = self.latest_joint_positions
                sample_time = self.latest_joint_state_time
                age = now - sample_time
            if positions is None or age > self.config.joint_state_timeout_sec:
                window_started_at = None
                window_min = None
                window_max = None
                time.sleep(0.02)
                continue
            if sample_time <= last_observed_sample_time:
                time.sleep(0.02)
                continue
            last_observed_sample_time = sample_time

            if window_started_at is None or window_min is None or window_max is None:
                reset_window(positions, sample_time)

            assert window_min is not None
            assert window_max is not None
            for index, value in enumerate(positions):
                window_min[index] = min(window_min[index], value)
                window_max[index] = max(window_max[index], value)

            spans = [
                maximum - minimum
                for minimum, maximum in zip(window_min, window_max)
            ]
            span_index = max(range(len(spans)), key=spans.__getitem__)
            last_motion_span = spans[span_index]
            limiting_joint = JOINT_NAMES[span_index]

            if expected is not None:
                errors = [
                    abs(actual - target)
                    for actual, target in zip(positions, expected)
                ]
                error_index = max(range(len(errors)), key=errors.__getitem__)
                last_reference_error = errors[error_index]
                if last_reference_error >= last_motion_span:
                    limiting_joint = JOINT_NAMES[error_index]

            if last_motion_span > self.config.q0_motion_span_tolerance_rad:
                reset_window(positions, sample_time)
                time.sleep(0.02)
                continue

            if sample_time - window_started_at >= self.config.q0_settle_sec:
                if (
                    expected is not None
                    and last_reference_error > self.config.q0_tolerance_rad
                ):
                    raise Q0ReferenceMismatch(
                        last_reference_error, last_motion_span, limiting_joint
                    )
                return positions, max(last_reference_error, last_motion_span)
            time.sleep(0.02)

        raise RuntimeError(
            "joint state did not settle at q0; "
            f"reference_error={last_reference_error:.9f} rad "
            f"motion_span={last_motion_span:.9f} rad "
            f"limiting_joint={limiting_joint}"
        )

    def _reset_and_confirm_q0(self) -> Tuple[float, float]:
        self._wait_supervisor_ready()
        used_joint_reset = False
        if self.config.auto_reset_to_start:
            if self.config.reset_algorithm == "move_j" and self.baseline_q0 is not None:
                self._execute_joint_reset(self.baseline_q0)
                used_joint_reset = True
            else:
                pose_reset_algorithm = (
                    "move_pose"
                    if self.config.reset_algorithm == "move_j"
                    else self.config.reset_algorithm
                )
                reset = self._execute_pose(
                    pose_reset_algorithm,
                    self.config.start_pose,
                    "RESET",
                    "RESET",
                    "reset",
                    0,
                    0,
                )
                if not reset.action_success:
                    raise RuntimeError(
                        "failed to return to benchmark start pose: "
                        f"{reset.classification}: {reset.detail}"
                    )
        try:
            positions, delta = self._wait_joint_state_at_q0(self.baseline_q0)
        except Q0ReferenceMismatch as exception:
            if not used_joint_reset or self.baseline_q0 is None:
                raise
            self.get_logger().warning(
                f"{exception}; retrying MoveJ reset once without widening q0 tolerance"
            )
            self._wait_supervisor_ready()
            self._execute_joint_reset(self.baseline_q0)
            positions, delta = self._wait_joint_state_at_q0(self.baseline_q0)
        if self.baseline_q0 is None:
            self.baseline_q0 = positions
            delta = 0.0
            self.get_logger().info(
                "Captured deterministic benchmark q0 after the initial MovePose reset"
            )
        tcp_y = self._wait_for_safe_tcp(self.config.tf_lookup_timeout_sec + 1.0)
        return delta, tcp_y

    def _wait_interfaces(self) -> None:
        for name, client in self.move_pose_clients.items():
            if not client.wait_for_server(timeout_sec=10.0):
                raise RuntimeError(f"Action server for {name} is unavailable")
        if not self.move_j_client.wait_for_server(timeout_sec=10.0):
            raise RuntimeError("Action server for move_j is unavailable")
        if not self.stop_client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError("/elfin3_motion/stop is unavailable")

    def run(self) -> None:
        output_directory: Optional[Path] = None
        try:
            self.get_logger().info(
                "Starting paired benchmark: "
                f"mode={self.config.execution_mode} "
                f"targets={len(self.config.targets)} "
                f"repetitions={self.config.repetitions} "
                f"allowed TCP y>={self.config.effective_y_min_m:.6f} m"
            )
            self._wait_interfaces()
            self._probe_motion_server_configuration()
            self._wait_for_safe_tcp(5.0)
            self._reset_and_confirm_q0()

            for target_index, target in enumerate(self.config.targets):
                for repetition in range(self.config.repetitions):
                    pair_id = f"{target.target_id}:r{repetition + 1}"
                    order = ("move_pose", "move_pose_ptp")
                    if (target_index + repetition) % 2 == 1:
                        order = tuple(reversed(order))
                    for order_index, algorithm in enumerate(order):
                        if self.abort_event.is_set():
                            raise RuntimeError(
                                self.runtime_safety_reason or "benchmark abort requested"
                            )
                        q0_delta, start_tcp_y = self._reset_and_confirm_q0()
                        self._wait_supervisor_ready()
                        record = self._execute_pose(
                            algorithm,
                            target.pose,
                            pair_id,
                            target.target_id,
                            target.category,
                            repetition + 1,
                            order_index + 1,
                        )
                        record.q0_max_delta_rad = q0_delta
                        record.start_tcp_y_m = start_tcp_y
                        self.records.append(record)
                        self.get_logger().info(
                            f"{pair_id} {algorithm}: {record.classification} "
                            f"planning_success={record.planning_success} "
                            f"total_ms={record.total_planning_time_ms} "
                            f"max_jump={record.max_adjacent_delta_rad}"
                        )
                        if record.safety_stop_requested:
                            raise RuntimeError(self.runtime_safety_reason)
                        if record.planning_success is True and not record.action_success:
                            raise RuntimeError(
                                "execution failed after a valid plan; aborting to avoid an "
                                "unknown robot start state"
                            )
                        if not record.planning_metrics_received:
                            raise RuntimeError("planning metrics missing; aborting benchmark")

            output_directory = write_reports(
                self.output_root, self.config, self.records
            )
            self.get_logger().info(f"Benchmark complete: {output_directory}")
        except Exception as exception:  # noqa: BLE001 - top-level safety boundary
            self.get_logger().error(f"Benchmark aborted: {exception}")
            try:
                output_directory = write_reports(
                    self.output_root, self.config, self.records
                )
                self.get_logger().error(f"Partial report written to {output_directory}")
            except Exception as report_exception:  # noqa: BLE001
                self.get_logger().error(
                    f"Failed to write partial report: {report_exception}"
                )
        finally:
            if rclpy.ok():
                rclpy.shutdown()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    executor = MultiThreadedExecutor(num_threads=4)
    runner = None
    try:
        node = Elfin3AbBenchmark()
        executor.add_node(node)
        runner = threading.Thread(target=node.run, name="elfin3-ab-benchmark", daemon=True)
        runner.start()
        executor.spin()
    finally:
        if runner is not None:
            runner.join(timeout=2.0)
        if node is not None:
            executor.remove_node(node)
            node.destroy_node()
        executor.shutdown()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
