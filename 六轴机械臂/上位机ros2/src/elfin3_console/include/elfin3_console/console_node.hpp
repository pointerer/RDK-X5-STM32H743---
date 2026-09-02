#ifndef ELFIN3_CONSOLE__CONSOLE_NODE_HPP_
#define ELFIN3_CONSOLE__CONSOLE_NODE_HPP_

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "elfin3_interfaces/action/move_j.hpp"
#include "elfin3_interfaces/action/move_pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace elfin3_console
{

class ConsoleNode final : public rclcpp::Node
{
public:
  explicit ConsoleNode(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~ConsoleNode() override;

  void start();
  void prepareForShutdown();

private:
  static constexpr std::size_t kJointCount = 6;
  using MoveJ = elfin3_interfaces::action::MoveJ;
  using MoveJGoalHandle = rclcpp_action::ClientGoalHandle<MoveJ>;
  using MovePose = elfin3_interfaces::action::MovePose;
  using MovePoseGoalHandle = rclcpp_action::ClientGoalHandle<MovePose>;
  using SupervisorTrigger = std_srvs::srv::Trigger;

  enum class JointStateHealth
  {
    kNoData,
    kValid,
    kIncomplete,
    kInvalid,
  };

  enum class CommandState
  {
    kIdle,
    kWaiting,
    kSending,
    kActive,
    kCanceling,
  };

  enum class CommandType
  {
    kNone,
    kMoveJ,
    kMovePose,
  };

  struct DiagnosticLine
  {
    std::string level{"UNKNOWN"};
    std::string message{"no data"};
  };

  void handleJointState(const sensor_msgs::msg::JointState & message);
  void handleDiagnostics(const diagnostic_msgs::msg::DiagnosticArray & message);
  void updateToolPose();
  void displayStatus();
  void displayJointState();
  void displayToolPose();
  void rejectJointState(JointStateHealth health, const std::string & reason);
  void inputLoop();
  void handleCommand(const std::string & line);
  void handleMoveJCommand(const std::vector<std::string> & arguments);
  bool handleMovePoseCommand(
    const std::vector<std::string> & arguments,
    bool point_to_point,
    bool sequence_step = false);
  bool handleMovePoseRelativeCommand(
    const std::vector<std::string> & arguments,
    bool sequence_step = false,
    bool downward_orientation = false);
  void handleMovePoseRelativeSequenceCommand(
    const std::vector<std::string> & arguments,
    bool positive_xy);
  void startMovePoseRelativeSequenceStep();
  void scheduleMovePoseRelativeSequenceStep();
  void handleCancelCommand();
  void handleStopCommand();
  bool querySupervisorReadiness(std::string & reason);
  void requestMoveJCancellation(const MoveJGoalHandle::SharedPtr & goal_handle);
  void requestMovePoseCancellation(const MovePoseGoalHandle::SharedPtr & goal_handle);
  void cancelActiveGoalForShutdown();
  void handleMoveJGoalResponse(const MoveJGoalHandle::SharedPtr & goal_handle);
  void handleMoveJFeedback(
    MoveJGoalHandle::SharedPtr goal_handle,
    const std::shared_ptr<const MoveJ::Feedback> feedback);
  void handleMoveJResult(const MoveJGoalHandle::WrappedResult & wrapped_result);
  void handleMovePoseGoalResponse(const MovePoseGoalHandle::SharedPtr & goal_handle);
  void handleMovePoseFeedback(
    MovePoseGoalHandle::SharedPtr goal_handle,
    const std::shared_ptr<const MovePose::Feedback> feedback);
  void handleMovePoseResult(const MovePoseGoalHandle::WrappedResult & wrapped_result);
  void displayHelp();
  void printOutput(const std::string & text, bool show_prompt = true);
  void printPrompt();

  std::string base_frame_;
  std::string tool_frame_;
  double joint_state_timeout_sec_{0.5};
  double tf_timeout_sec_{0.5};
  double tf_lookup_timeout_sec_{0.05};
  double diagnostics_timeout_sec_{2.0};
  double supervisor_timeout_sec_{0.5};
  double action_server_timeout_sec_{0.5};
  double stop_service_timeout_sec_{0.5};
  std::int64_t input_poll_timeout_ms_{100};
  bool terminal_output_{true};

  std::mutex joint_state_mutex_;
  std::array<double, kJointCount> latest_joint_positions_{};
  std::chrono::steady_clock::time_point last_valid_joint_state_time_{};
  JointStateHealth joint_state_health_{JointStateHealth::kNoData};
  std::string joint_state_issue_;
  bool has_valid_joint_state_{false};

  std::mutex tf_mutex_;
  std::array<double, 3> latest_tool_position_{};
  std::array<double, 4> latest_tool_orientation_{};
  std::chrono::steady_clock::time_point last_tf_update_time_{};
  std::int64_t last_tf_stamp_ns_{0};
  std::string tf_issue_;
  bool tf_available_{false};
  bool tf_values_valid_{false};
  bool tf_time_reset_{false};

  std::mutex diagnostics_mutex_;
  std::unordered_map<std::string, DiagnosticLine> supervisor_diagnostics_;
  std::chrono::steady_clock::time_point last_diagnostics_time_{};
  std::string supervisor_state_{"UNKNOWN"};
  std::string supervisor_reason_{"no diagnostics received"};
  bool has_supervisor_diagnostics_{false};

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscription_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr
    diagnostics_subscription_;
  rclcpp::TimerBase::SharedPtr tf_update_timer_;

  std::mutex output_mutex_;
  std::atomic<bool> input_stop_requested_{false};
  bool input_started_{false};
  std::thread input_thread_;

  std::mutex command_mutex_;
  CommandState command_state_{CommandState::kIdle};
  CommandType command_type_{CommandType::kNone};
  bool cancel_when_accepted_{false};
  std::atomic<bool> shutdown_cleanup_started_{false};
  MoveJGoalHandle::SharedPtr active_move_j_goal_;
  MovePoseGoalHandle::SharedPtr active_move_pose_goal_;
  bool move_pose_relative_sequence_active_{false};
  bool move_pose_relative_sequence_positive_xy_{false};
  std::size_t move_pose_relative_sequence_step_{0};
  double move_pose_relative_sequence_velocity_scaling_{0.0};
  double move_pose_relative_sequence_acceleration_scaling_{0.0};
  rclcpp::CallbackGroup::SharedPtr move_pose_relative_sequence_callback_group_;
  rclcpp::TimerBase::SharedPtr move_pose_relative_sequence_timer_;
  rclcpp::Client<SupervisorTrigger>::SharedPtr supervisor_client_;
  rclcpp::Client<SupervisorTrigger>::SharedPtr stop_client_;
  rclcpp_action::Client<MoveJ>::SharedPtr move_j_client_;
  rclcpp_action::Client<MovePose>::SharedPtr move_pose_client_;
  rclcpp_action::Client<MovePose>::SharedPtr move_pose_ptp_client_;
  rclcpp_action::Client<MovePose>::SharedPtr active_move_pose_client_;
  std::string active_move_pose_name_{"MovePose"};
};

}  // namespace elfin3_console

#endif  // ELFIN3_CONSOLE__CONSOLE_NODE_HPP_
