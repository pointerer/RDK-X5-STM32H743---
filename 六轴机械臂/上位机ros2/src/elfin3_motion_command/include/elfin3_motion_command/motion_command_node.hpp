#ifndef ELFIN3_MOTION_COMMAND__MOTION_COMMAND_NODE_HPP_
#define ELFIN3_MOTION_COMMAND__MOTION_COMMAND_NODE_HPP_

#include <cstdint>
#include <condition_variable>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "elfin3_interfaces/action/move_j.hpp"
#include "elfin3_interfaces/action/move_pose.hpp"
#include "elfin3_interfaces/msg/planning_metrics.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "moveit/move_group_interface/move_group_interface.h"
#include "moveit/robot_state/robot_state.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

namespace elfin3_motion_command
{

class MotionCommandNode final : public rclcpp::Node
{
public:
  explicit MotionCommandNode(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~MotionCommandNode() override;

  void initializeMoveGroup();
  void checkSupervisorAtStartup();

private:
  using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;
  using MoveJ = elfin3_interfaces::action::MoveJ;
  using MoveJGoalHandle = rclcpp_action::ServerGoalHandle<MoveJ>;
  using MovePose = elfin3_interfaces::action::MovePose;
  using MovePoseGoalHandle = rclcpp_action::ServerGoalHandle<MovePose>;
  using PlanningMetrics = elfin3_interfaces::msg::PlanningMetrics;
  using SupervisorTrigger = std_srvs::srv::Trigger;

  enum class SupervisorCheckStatus
  {
    kReady,
    kNotReady,
    kServiceUnavailable,
    kTimedOut,
    kError,
  };

  enum class CommandState
  {
    kIdle,
    kValidating,
    kPlanning,
    kExecuting,
    kStopping,
  };

  enum class CommandType
  {
    kNone,
    kMoveJ,
    kMovePose,
    kMovePosePtp,
  };

  struct SupervisorCheckResult
  {
    SupervisorCheckStatus status;
    std::string message;
  };

  struct TrajectoryAnalysisResult
  {
    bool valid{false};
    std::string reason;
    double maximum_adjacent_delta_rad{0.0};
    double minimum_tcp_y_m{std::numeric_limits<double>::quiet_NaN()};
    std::uint32_t fk_samples_evaluated{0};
  };

  SupervisorCheckResult querySupervisorReadiness();
  bool canFdHoldAvailable() const;
  bool requestCanFdHold(const char * reason);
  rclcpp_action::GoalResponse handleMoveJGoal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const MoveJ::Goal> goal);
  rclcpp_action::CancelResponse handleMoveJCancel(
    const std::shared_ptr<MoveJGoalHandle> goal_handle);
  void handleMoveJAccepted(const std::shared_ptr<MoveJGoalHandle> goal_handle);
  rclcpp_action::GoalResponse handleMovePoseGoal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const MovePose::Goal> goal);
  rclcpp_action::CancelResponse handleMovePoseCancel(
    const std::shared_ptr<MovePoseGoalHandle> goal_handle);
  void handleMovePoseAccepted(
    const std::shared_ptr<MovePoseGoalHandle> goal_handle);
  rclcpp_action::GoalResponse handleMovePosePtpGoal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const MovePose::Goal> goal);
  rclcpp_action::CancelResponse handleMovePosePtpCancel(
    const std::shared_ptr<MovePoseGoalHandle> goal_handle);
  void handleMovePosePtpAccepted(
    const std::shared_ptr<MovePoseGoalHandle> goal_handle);
  void handleStop(
    const std::shared_ptr<SupervisorTrigger::Request> request,
    std::shared_ptr<SupervisorTrigger::Response> response);
  bool validateMoveJGoal(
    const MoveJ::Goal & goal,
    double & velocity_scaling,
    double & acceleration_scaling,
    std::string & reason) const;
  bool validateMovePoseGoal(
    const MovePose::Goal & goal,
    geometry_msgs::msg::PoseStamped & normalized_target,
    double & velocity_scaling,
    double & acceleration_scaling,
    std::string & reason) const;
  TrajectoryAnalysisResult analyzeTrajectory(
    const moveit::core::RobotState & reference_state,
    const trajectory_msgs::msg::JointTrajectory & trajectory) const;
  bool cancelRequested();
  void finishMoveJAsCanceled(
    const std::shared_ptr<MoveJGoalHandle> goal_handle,
    std::int32_t moveit_error_code,
    const std::string & message);
  void finishMovePoseAsCanceled(
    const std::shared_ptr<MovePoseGoalHandle> goal_handle,
    std::int32_t moveit_error_code,
    const std::string & message);
  void commandWorker();
  void processMoveJGoal(const std::shared_ptr<MoveJGoalHandle> goal_handle);
  void processMovePoseGoal(
    const std::shared_ptr<MovePoseGoalHandle> goal_handle);
  void processMovePosePtpGoal(
    const std::shared_ptr<MovePoseGoalHandle> goal_handle);

  std::string planning_group_;
  std::string base_frame_;
  std::string tool_frame_;
  std::string planning_pipeline_;
  double planning_time_sec_{5.0};
  std::int64_t planning_attempts_{10};
  double default_velocity_scaling_{0.2};
  double default_acceleration_scaling_{0.2};
  double joint_tolerance_rad_{0.01};
  double position_tolerance_m_{0.005};
  double orientation_tolerance_rad_{0.02};
  double move_pose_max_joint_delta_rad_{1.0};
  double move_pose_ptp_ik_translation_step_m_{0.05};
  double move_pose_ptp_ik_rotation_step_rad_{0.17453292519943295};
  std::int64_t move_pose_ptp_max_ik_segments_{100};
  double supervisor_timeout_sec_{0.5};
  bool canfd_hold_on_cancel_{false};
  std::string canfd_hold_service_;
  bool workspace_y_constraint_enabled_{false};
  double workspace_min_tcp_y_m_{0.0};
  double workspace_y_margin_m_{0.0};
  double workspace_fk_sample_max_joint_step_rad_{0.0};
  rclcpp::Client<SupervisorTrigger>::SharedPtr supervisor_client_;
  rclcpp::Client<SupervisorTrigger>::SharedPtr canfd_hold_client_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr control_mode_subscription_;
  rclcpp_action::Server<MoveJ>::SharedPtr move_j_action_server_;
  rclcpp_action::Server<MovePose>::SharedPtr move_pose_action_server_;
  rclcpp_action::Server<MovePose>::SharedPtr move_pose_ptp_action_server_;
  rclcpp_action::Client<MoveJ>::SharedPtr move_j_cancel_client_;
  rclcpp_action::Client<MovePose>::SharedPtr move_pose_cancel_client_;
  rclcpp_action::Client<MovePose>::SharedPtr move_pose_ptp_cancel_client_;
  rclcpp::Service<SupervisorTrigger>::SharedPtr stop_service_;
  rclcpp::Publisher<PlanningMetrics>::SharedPtr planning_metrics_publisher_;
  std::unique_ptr<MoveGroupInterface> move_group_;

  std::mutex command_mutex_;
  std::condition_variable command_cv_;
  CommandState command_state_{CommandState::kIdle};
  CommandType command_type_{CommandType::kNone};
  std::string control_mode_{"MOVEIT"};
  std::shared_ptr<MoveJGoalHandle> pending_move_j_goal_;
  std::shared_ptr<MovePoseGoalHandle> pending_move_pose_goal_;
  std::shared_ptr<MovePoseGoalHandle> pending_move_pose_ptp_goal_;
  bool cancel_requested_{false};
  bool shutting_down_{false};
  std::thread command_worker_;
};

}  // namespace elfin3_motion_command

#endif  // ELFIN3_MOTION_COMMAND__MOTION_COMMAND_NODE_HPP_
