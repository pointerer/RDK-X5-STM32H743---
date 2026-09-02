from __future__ import annotations

from dataclasses import asdict, dataclass, field
from typing import Any, Dict, List, Optional, Tuple


@dataclass(frozen=True)
class PoseSpec:
    frame_id: str
    position: Tuple[float, float, float]
    orientation: Tuple[float, float, float, float]


@dataclass(frozen=True)
class TargetSpec:
    target_id: str
    category: str
    pose: PoseSpec


@dataclass(frozen=True)
class BenchmarkConfig:
    base_frame: str
    tool_frame: str
    y_safe_min_m: float
    y_safety_margin_m: float
    effective_y_min_m: float
    workspace_fk_sample_max_joint_step_rad: float
    repetitions: int
    execution_mode: str
    allow_real_motion: bool
    velocity_scaling: float
    acceleration_scaling: float
    q0_tolerance_rad: float
    q0_motion_span_tolerance_rad: float
    q0_settle_sec: float
    q0_wait_timeout_sec: float
    supervisor_wait_timeout_sec: float
    action_timeout_sec: float
    metrics_timeout_sec: float
    joint_state_timeout_sec: float
    tf_lookup_timeout_sec: float
    safety_poll_period_sec: float
    auto_reset_to_start: bool
    reset_algorithm: str
    start_pose: PoseSpec
    targets: Tuple[TargetSpec, ...]


@dataclass(frozen=True)
class PlanningMetricsSnapshot:
    goal_id: str
    algorithm: str
    planning_eligible: bool
    planning_success: bool
    failure_stage: str
    failure_reason: str
    total_planning_time_ms: float
    ompl_planning_time_ms: float
    trajectory_points: int
    max_adjacent_delta_rad: float
    ik_method: str
    ik_segments: int
    minimum_tcp_y_m: float
    workspace_safe: bool
    workspace_y_constraint_enabled: bool
    workspace_min_tcp_y_m: float
    workspace_y_margin_m: float
    fk_samples_evaluated: int
    workspace_fk_sample_max_joint_step_rad: float


@dataclass
class AttemptRecord:
    pair_id: str
    target_id: str
    category: str
    repetition: int
    order_in_pair: int
    algorithm: str
    classification: str
    detail: str = ""
    goal_id: str = ""
    goal_accepted: bool = False
    planning_metrics_received: bool = False
    planning_eligible: Optional[bool] = None
    planning_success: Optional[bool] = None
    failure_stage: str = ""
    action_status: Optional[int] = None
    result_code: Optional[int] = None
    moveit_error_code: Optional[int] = None
    action_success: bool = False
    total_planning_time_ms: Optional[float] = None
    ompl_planning_time_ms: Optional[float] = None
    end_to_end_time_ms: Optional[float] = None
    trajectory_points: Optional[int] = None
    max_adjacent_delta_rad: Optional[float] = None
    ik_method: str = ""
    ik_segments: Optional[int] = None
    minimum_tcp_y_m: Optional[float] = None
    start_tcp_y_m: Optional[float] = None
    target_tcp_y_m: Optional[float] = None
    q0_max_delta_rad: Optional[float] = None
    workspace_safe: Optional[bool] = None
    fk_samples_evaluated: Optional[int] = None
    safety_stop_requested: bool = False
    feedback_stages: List[str] = field(default_factory=list)

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)
