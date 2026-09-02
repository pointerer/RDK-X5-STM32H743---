import json

import pytest

from elfin3_ab_benchmark.models import (
    AttemptRecord,
    BenchmarkConfig,
    PoseSpec,
    TargetSpec,
)
from elfin3_ab_benchmark.reporting import (
    aggregate_records,
    percentile_nearest_rank,
    write_reports,
)


def make_config():
    pose = PoseSpec("elfin_base", (0.0, 0.4, 0.3), (0.0, 0.0, 0.0, 1.0))
    return BenchmarkConfig(
        base_frame="elfin_base",
        tool_frame="elfin_end_link",
        y_safe_min_m=0.2,
        y_safety_margin_m=0.05,
        effective_y_min_m=0.25,
        workspace_fk_sample_max_joint_step_rad=0.02,
        repetitions=1,
        execution_mode="simulation",
        allow_real_motion=False,
        velocity_scaling=0.1,
        acceleration_scaling=0.1,
        q0_tolerance_rad=0.005,
        q0_motion_span_tolerance_rad=0.001,
        q0_settle_sec=0.5,
        q0_wait_timeout_sec=5.0,
        supervisor_wait_timeout_sec=5.0,
        action_timeout_sec=10.0,
        metrics_timeout_sec=1.0,
        joint_state_timeout_sec=0.5,
        tf_lookup_timeout_sec=0.05,
        safety_poll_period_sec=0.02,
        auto_reset_to_start=True,
        reset_algorithm="move_pose",
        start_pose=pose,
        targets=(TargetSpec("t1", "ordinary", pose),),
    )


def record(pair_id, algorithm, eligible, success, total_ms=None, jump=None):
    return AttemptRecord(
        pair_id=pair_id,
        target_id="t1",
        category="ordinary",
        repetition=1,
        order_in_pair=1,
        algorithm=algorithm,
        classification="SUCCESS" if success else "PLANNING_FAILED",
        goal_accepted=True,
        planning_metrics_received=True,
        planning_eligible=eligible,
        planning_success=success,
        action_success=success,
        total_planning_time_ms=total_ms,
        ompl_planning_time_ms=total_ms,
        max_adjacent_delta_rad=jump,
    )


def test_percentile_uses_nearest_rank():
    assert percentile_nearest_rank(list(range(1, 101)), 95.0) == 95.0
    assert percentile_nearest_rank([], 95.0) is None


def test_success_rate_excludes_precondition_rows():
    records = [
        record("p1", "move_pose", True, True, 10.0, 0.2),
        record("p2", "move_pose", True, False),
        record("p3", "move_pose", False, False),
        record("p1", "move_pose_ptp", True, True, 20.0, 0.1),
    ]
    summary = aggregate_records(records)
    move_pose = summary["algorithms"]["move_pose"]
    assert move_pose["planning_eligible"] == 2
    assert move_pose["planning_success_rate"] == pytest.approx(0.5)
    assert summary["paired"]["both_succeeded"] == 1


def test_write_reports_creates_machine_and_human_readable_outputs(tmp_path):
    records = [record("p1", "move_pose", True, True, 10.0, 0.2)]
    output = write_reports(str(tmp_path), make_config(), records)
    assert (output / "records.csv").is_file()
    assert (output / "summary.md").is_file()
    summary = json.loads((output / "summary.json").read_text(encoding="utf-8"))
    assert summary["algorithms"]["move_pose"]["planning_successes"] == 1
