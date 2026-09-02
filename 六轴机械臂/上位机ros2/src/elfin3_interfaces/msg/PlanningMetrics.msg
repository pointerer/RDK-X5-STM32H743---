# Correlation identifier copied from the ROS 2 action goal UUID.
string goal_id
# move_pose or move_pose_ptp.
string algorithm
# True only after input, mode, supervisor, current-state, and target-workspace checks pass.
bool planning_eligible
# True when OMPL succeeds and the resulting trajectory passes all validation.
bool planning_success
string failure_stage
string failure_reason

# Server-side elapsed time from command processing start through planning validation.
float64 total_planning_time_ms
float64 ompl_planning_time_ms
uint32 trajectory_points
float64 max_adjacent_delta_rad
string ik_method
uint32 ik_segments

# TCP workspace sampling results in elfin_base. NaN means unavailable/not evaluated.
float64 minimum_tcp_y_m
bool workspace_safe
bool workspace_y_constraint_enabled
float64 workspace_min_tcp_y_m
float64 workspace_y_margin_m
uint32 fk_samples_evaluated
float64 workspace_fk_sample_max_joint_step_rad
