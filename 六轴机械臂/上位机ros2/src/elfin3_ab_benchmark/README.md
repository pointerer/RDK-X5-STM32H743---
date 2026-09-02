# Elfin3 MovePose / MovePosePTP A/B Benchmark

This package runs paired MovePose and MovePosePTP trials from the same measured
joint start, alternates AB/BA order, and writes raw CSV plus JSON/Markdown
summaries for planning success, P95 planning time, and maximum adjacent planned
joint change.

## Workspace restriction

The installation covered by this project may only operate in the positive-Y
half-space and must not approach the X axis. The implemented test condition is:

```text
elfin_base TCP_y >= y_safe_min_m + y_safety_margin_m
```

`y_safe_min_m` has no default and must be measured for the real installation.
The runner refuses to start when it is missing, non-finite, or non-positive.

MotionCommand checks target Y, start Y, and FK samples along every planned
MoveJ/MovePose/MovePosePTP trajectory before execution. The runner additionally
monitors `elfin_base -> elfin_end_link` during an active action and cancels/stops
the batch if Y crosses the boundary or TF becomes unavailable.

This is a sampled software TCP guard. It is not a continuous mathematical proof,
does not constrain every robot link, and is not functional-safety certification.
If every link must remain outside the forbidden half-space, add a reviewed
PlanningScene collision region and a safety-rated external mechanism.

## Configuration

Review `config/pose_ab.example.yaml`, copy it to a run-specific absolute path,
and freeze the target list. The launch file deliberately has no default dataset,
so every run must name the reviewed file explicitly. Configure MotionCommand
with exactly the same boundary and sampling step:

```yaml
workspace_y_constraint_enabled: true
workspace_min_tcp_y_m: <measured-positive-value>
workspace_y_margin_m: <reviewed-margin>
workspace_fk_sample_max_joint_step_rad: <reviewed-positive-step>
```

The benchmark first sends a non-executable invalid-frame probe and refuses any
motion if the server reports a disabled or mismatched workspace guard.

With `reset_algorithm: move_j`, the first reset uses MovePose to reach the
configured safe `start_pose` and captures the measured six-joint q0. All later
resets plan a MoveJ back to that exact q0. This prevents IK from selecting a
different joint solution for the same Cartesian pose while retaining the
MotionCommand planned-trajectory Y check and the runner's runtime TCP monitor.
The example's q0 acceptance limit is 0.011 rad: the real controller's 0.010 rad
terminal tolerance plus a bounded 0.001 rad observation guard band. The raw
start-state delta is still stored for every attempt and should be reviewed when
interpreting paired results.

Stationarity is checked independently with `q0_motion_span_tolerance_rad` over
the full settle window. If the state is stationary but just outside the q0
limit, the runner performs exactly one additional MoveJ reset. A second q0
mismatch, continuously moving joints, stale JointState, or any workspace safety
failure aborts the run instead of widening the acceptance limit.

Supervisor readiness uses the configured overall wait timeout. A single
service-response timeout is canceled and retried within that fixed window; no
motion goal is submitted until a fresh response explicitly reports READY.

For a fair comparison:

- use the same `start_pose`, target PoseStamped, scaling and planning settings;
- use at least 20 fixed targets and three repetitions when practical;
- include ordinary, large translation/rotation, and safe multi-solution targets;
- do not include invalid targets in the planning-success denominator;
- run the complete set in simulation before selecting a small real-hardware set.

## Build and run

```bash
colcon build --symlink-install --packages-up-to elfin3_ab_benchmark
source install/setup.bash
ros2 launch elfin3_ab_benchmark ab_benchmark.launch.py \
  dataset:=/absolute/path/to/pose_ab.yaml \
  output_root:=/absolute/path/to/results
```

The robot bringup must already be running with MotionCommand and Supervisor.
Real mode additionally requires `execution_mode: real`, `allow_real_motion: true`,
and velocity/acceleration scaling no greater than 0.2.

Each run creates:

```text
ab_benchmark_<UTC timestamp>/
  records.csv
  records.json
  failed_cases.csv
  summary.json
  summary.md
  config.json
```

`planning_success_rate` uses only metrics marked `planning_eligible`; rejected,
invalid, not-ready, canceled, and workspace-precondition cases remain visible but
do not dilute the algorithm denominator. A safe endpoint whose planned path
violates Y is reported as a trajectory-validation planning failure.
