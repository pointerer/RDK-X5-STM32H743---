from __future__ import annotations

import math
from pathlib import Path
from typing import Any, Mapping, Sequence, Tuple

import yaml

from .models import BenchmarkConfig, PoseSpec, TargetSpec


REAL_MODE_MAX_SCALING = 0.2


def _finite_number(value: Any, field_name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{field_name} must be a finite number")
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"{field_name} must be a finite number")
    return result


def _positive_number(value: Any, field_name: str) -> float:
    result = _finite_number(value, field_name)
    if result <= 0.0:
        raise ValueError(f"{field_name} must be greater than zero")
    return result


def _nonnegative_number(value: Any, field_name: str) -> float:
    result = _finite_number(value, field_name)
    if result < 0.0:
        raise ValueError(f"{field_name} must be non-negative")
    return result


def _boolean(value: Any, field_name: str) -> bool:
    if not isinstance(value, bool):
        raise ValueError(f"{field_name} must be true or false")
    return value


def _integer(value: Any, field_name: str, minimum: int = 1) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value < minimum:
        raise ValueError(f"{field_name} must be an integer >= {minimum}")
    return value


def _vector(value: Any, length: int, field_name: str) -> Tuple[float, ...]:
    if (
        not isinstance(value, Sequence)
        or isinstance(value, (str, bytes))
        or len(value) != length
    ):
        raise ValueError(f"{field_name} must contain exactly {length} numbers")
    return tuple(
        _finite_number(component, f"{field_name}[{index}]")
        for index, component in enumerate(value)
    )


def _parse_pose(value: Any, field_name: str, base_frame: str) -> PoseSpec:
    if not isinstance(value, Mapping):
        raise ValueError(f"{field_name} must be a mapping")
    frame_id = str(value.get("frame_id", base_frame))
    if frame_id != base_frame or frame_id != "elfin_base":
        raise ValueError(f"{field_name}.frame_id must be exactly elfin_base")
    position = _vector(value.get("position"), 3, f"{field_name}.position")
    orientation = _vector(value.get("orientation"), 4, f"{field_name}.orientation")
    norm = math.sqrt(sum(component * component for component in orientation))
    if norm <= 1.0e-6:
        raise ValueError(f"{field_name}.orientation quaternion norm must be > 1e-6")
    normalized = tuple(component / norm for component in orientation)
    return PoseSpec(
        frame_id=frame_id,
        position=(position[0], position[1], position[2]),
        orientation=(normalized[0], normalized[1], normalized[2], normalized[3]),
    )


def _validate_pose_y(name: str, pose: PoseSpec, effective_y_min_m: float) -> None:
    if pose.position[1] < effective_y_min_m:
        raise ValueError(
            f"{name}.position.y={pose.position[1]:.6f} m is below "
            f"effective_y_min_m={effective_y_min_m:.6f} m"
        )


def load_dataset(path: str, execution_mode_override: str = "") -> BenchmarkConfig:
    dataset_path = Path(path).expanduser().resolve()
    if not dataset_path.is_file():
        raise ValueError(f"dataset_path does not exist: {dataset_path}")
    with dataset_path.open("r", encoding="utf-8") as stream:
        document = yaml.safe_load(stream)
    if not isinstance(document, Mapping):
        raise ValueError("dataset root must be a mapping")

    base_frame = str(document.get("base_frame", "elfin_base"))
    if base_frame != "elfin_base":
        raise ValueError("base_frame must be exactly elfin_base")
    tool_frame = str(document.get("tool_frame", "elfin_end_link")).strip()
    if not tool_frame:
        raise ValueError("tool_frame must not be empty")

    # No safety default is permitted: the operator must enter measured values.
    y_safe_min_m = _positive_number(document.get("y_safe_min_m"), "y_safe_min_m")
    y_safety_margin_m = _nonnegative_number(
        document.get("y_safety_margin_m", 0.0), "y_safety_margin_m"
    )
    effective_y_min_m = y_safe_min_m + y_safety_margin_m
    fk_step = _positive_number(
        document.get("workspace_fk_sample_max_joint_step_rad"),
        "workspace_fk_sample_max_joint_step_rad",
    )

    execution_mode = execution_mode_override or str(
        document.get("execution_mode", "simulation")
    )
    if execution_mode not in ("simulation", "real"):
        raise ValueError("execution_mode must be simulation or real")
    allow_real_motion = _boolean(
        document.get("allow_real_motion", False), "allow_real_motion"
    )
    if execution_mode == "real" and not allow_real_motion:
        raise ValueError(
            "real mode requires allow_real_motion: true after operator safety review"
        )

    velocity_scaling = _positive_number(
        document.get("velocity_scaling", 0.2), "velocity_scaling"
    )
    acceleration_scaling = _positive_number(
        document.get("acceleration_scaling", 0.2), "acceleration_scaling"
    )
    if velocity_scaling > 1.0 or acceleration_scaling > 1.0:
        raise ValueError("velocity_scaling and acceleration_scaling must be <= 1.0")
    if execution_mode == "real" and (
        velocity_scaling > REAL_MODE_MAX_SCALING
        or acceleration_scaling > REAL_MODE_MAX_SCALING
    ):
        raise ValueError(
            "real mode requires velocity_scaling and acceleration_scaling "
            f"<= {REAL_MODE_MAX_SCALING}"
        )

    start_pose = _parse_pose(document.get("start_pose"), "start_pose", base_frame)
    _validate_pose_y("start_pose", start_pose, effective_y_min_m)

    raw_targets = document.get("targets")
    if (
        not isinstance(raw_targets, Sequence)
        or isinstance(raw_targets, (str, bytes))
        or not raw_targets
    ):
        raise ValueError("targets must be a non-empty sequence")
    targets = []
    seen_ids = set()
    for index, item in enumerate(raw_targets):
        if not isinstance(item, Mapping):
            raise ValueError(f"targets[{index}] must be a mapping")
        target_id = str(item.get("id", "")).strip()
        if not target_id or target_id in seen_ids:
            raise ValueError(f"targets[{index}].id must be non-empty and unique")
        seen_ids.add(target_id)
        category = str(item.get("category", "unspecified")).strip() or "unspecified"
        pose = _parse_pose(item.get("pose"), f"targets[{index}].pose", base_frame)
        _validate_pose_y(f"targets[{index}].pose", pose, effective_y_min_m)
        targets.append(TargetSpec(target_id=target_id, category=category, pose=pose))

    reset_algorithm = str(document.get("reset_algorithm", "move_pose"))
    if reset_algorithm not in ("move_j", "move_pose", "move_pose_ptp"):
        raise ValueError(
            "reset_algorithm must be move_j, move_pose, or move_pose_ptp"
        )

    q0_tolerance_rad = _positive_number(
        document.get("q0_tolerance_rad", 0.01), "q0_tolerance_rad"
    )
    q0_motion_span_tolerance_rad = _positive_number(
        document.get("q0_motion_span_tolerance_rad", 0.001),
        "q0_motion_span_tolerance_rad",
    )
    if q0_motion_span_tolerance_rad >= q0_tolerance_rad:
        raise ValueError(
            "q0_motion_span_tolerance_rad must be smaller than q0_tolerance_rad"
        )

    return BenchmarkConfig(
        base_frame=base_frame,
        tool_frame=tool_frame,
        y_safe_min_m=y_safe_min_m,
        y_safety_margin_m=y_safety_margin_m,
        effective_y_min_m=effective_y_min_m,
        workspace_fk_sample_max_joint_step_rad=fk_step,
        repetitions=_integer(document.get("repetitions", 3), "repetitions"),
        execution_mode=execution_mode,
        allow_real_motion=allow_real_motion,
        velocity_scaling=velocity_scaling,
        acceleration_scaling=acceleration_scaling,
        q0_tolerance_rad=q0_tolerance_rad,
        q0_motion_span_tolerance_rad=q0_motion_span_tolerance_rad,
        q0_settle_sec=_positive_number(
            document.get("q0_settle_sec", 0.5), "q0_settle_sec"
        ),
        q0_wait_timeout_sec=_positive_number(
            document.get("q0_wait_timeout_sec", 15.0), "q0_wait_timeout_sec"
        ),
        supervisor_wait_timeout_sec=_positive_number(
            document.get("supervisor_wait_timeout_sec", 15.0),
            "supervisor_wait_timeout_sec",
        ),
        action_timeout_sec=_positive_number(
            document.get("action_timeout_sec", 60.0), "action_timeout_sec"
        ),
        metrics_timeout_sec=_positive_number(
            document.get("metrics_timeout_sec", 2.0), "metrics_timeout_sec"
        ),
        joint_state_timeout_sec=_positive_number(
            document.get("joint_state_timeout_sec", 0.5),
            "joint_state_timeout_sec",
        ),
        tf_lookup_timeout_sec=_positive_number(
            document.get("tf_lookup_timeout_sec", 0.05),
            "tf_lookup_timeout_sec",
        ),
        safety_poll_period_sec=_positive_number(
            document.get("safety_poll_period_sec", 0.02),
            "safety_poll_period_sec",
        ),
        auto_reset_to_start=_boolean(
            document.get("auto_reset_to_start", True), "auto_reset_to_start"
        ),
        reset_algorithm=reset_algorithm,
        start_pose=start_pose,
        targets=tuple(targets),
    )
