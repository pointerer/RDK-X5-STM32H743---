#include "elfin3_motion_command/motion_command_node.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <exception>
#include <future>
#include <iomanip>
#include <iterator>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Geometry>
#include "moveit/robot_model/joint_model.h"
#include "moveit_msgs/msg/move_it_error_codes.hpp"

namespace elfin3_motion_command
{
namespace
{

constexpr std::array<const char *, 6> kExpectedActiveJoints = {
  "elfin_joint1",
  "elfin_joint2",
  "elfin_joint3",
  "elfin_joint4",
  "elfin_joint5",
  "elfin_joint6",
};

constexpr double kMoveItInterfaceWaitSeconds = 5.0;
constexpr std::size_t kMaximumWorkspaceFkSamples = 100000U;

std::string goalUuidToString(const rclcpp_action::GoalUUID & uuid)
{
  std::ostringstream stream;
  stream << std::hex << std::setfill('0');
  for (const auto byte : uuid) {
    stream << std::setw(2) << static_cast<unsigned int>(byte);
  }
  return stream.str();
}

class PlanningMetricsScope
{
public:
  using Message = elfin3_interfaces::msg::PlanningMetrics;

  PlanningMetricsScope(
    rclcpp::Publisher<Message>::SharedPtr publisher,
    const rclcpp_action::GoalUUID & goal_id,
    std::string algorithm,
    bool workspace_constraint_enabled,
    double workspace_min_tcp_y_m,
    double workspace_y_margin_m,
    double workspace_fk_sample_max_joint_step_rad)
  : publisher_(std::move(publisher)),
    started_at_(std::chrono::steady_clock::now())
  {
    message_.goal_id = goalUuidToString(goal_id);
    message_.algorithm = std::move(algorithm);
    message_.planning_eligible = false;
    message_.planning_success = false;
    message_.failure_stage = "PRECONDITION";
    message_.total_planning_time_ms = 0.0;
    message_.ompl_planning_time_ms = std::numeric_limits<double>::quiet_NaN();
    message_.trajectory_points = 0U;
    message_.max_adjacent_delta_rad = std::numeric_limits<double>::quiet_NaN();
    message_.ik_method.clear();
    message_.ik_segments = 0U;
    message_.minimum_tcp_y_m = std::numeric_limits<double>::quiet_NaN();
    message_.workspace_safe = !workspace_constraint_enabled;
    message_.workspace_y_constraint_enabled = workspace_constraint_enabled;
    message_.workspace_min_tcp_y_m = workspace_min_tcp_y_m;
    message_.workspace_y_margin_m = workspace_y_margin_m;
    message_.fk_samples_evaluated = 0U;
    message_.workspace_fk_sample_max_joint_step_rad =
      workspace_fk_sample_max_joint_step_rad;
  }

  ~PlanningMetricsScope()
  {
    publish();
  }

  PlanningMetricsScope(const PlanningMetricsScope &) = delete;
  PlanningMetricsScope & operator=(const PlanningMetricsScope &) = delete;

  void setEligible()
  {
    message_.planning_eligible = true;
  }

  void setFailureStage(const std::string & stage, const std::string & reason = {})
  {
    message_.failure_stage = stage;
    message_.failure_reason = reason;
  }

  void setIkResult(const std::string & method, std::uint32_t segments)
  {
    message_.ik_method = method;
    message_.ik_segments = segments;
  }

  void setOmplResult(double planning_time_sec, std::size_t trajectory_points)
  {
    message_.ompl_planning_time_ms = planning_time_sec * 1000.0;
    message_.trajectory_points = static_cast<std::uint32_t>(
      std::min<std::size_t>(
        trajectory_points, std::numeric_limits<std::uint32_t>::max()));
  }

  void setTrajectoryAnalysis(
    double maximum_adjacent_delta_rad,
    double minimum_tcp_y_m,
    std::uint32_t fk_samples_evaluated,
    bool workspace_safe)
  {
    message_.max_adjacent_delta_rad = maximum_adjacent_delta_rad;
    message_.minimum_tcp_y_m = minimum_tcp_y_m;
    message_.fk_samples_evaluated = fk_samples_evaluated;
    message_.workspace_safe = workspace_safe;
  }

  void markPlanningSuccess()
  {
    message_.planning_success = true;
    message_.failure_stage.clear();
    message_.failure_reason.clear();
    publish();
  }

private:
  void publish() noexcept
  {
    if (published_ || !publisher_) {
      return;
    }
    const auto elapsed = std::chrono::steady_clock::now() - started_at_;
    message_.total_planning_time_ms =
      std::chrono::duration<double, std::milli>(elapsed).count();
    try {
      publisher_->publish(message_);
    } catch (...) {
      // Metrics must never alter the motion command result.
    }
    published_ = true;
  }

  rclcpp::Publisher<Message>::SharedPtr publisher_;
  std::chrono::steady_clock::time_point started_at_;
  Message message_;
  bool published_{false};
};

void requireNonEmpty(const std::string & value, const char * parameter_name)
{
  if (value.empty()) {
    throw std::invalid_argument(std::string(parameter_name) + " must not be empty");
  }
}

void requirePositiveFinite(double value, const char * parameter_name)
{
  if (!std::isfinite(value) || value <= 0.0) {
    throw std::invalid_argument(
            std::string(parameter_name) + " must be finite and greater than zero");
  }
}

void requireScaling(double value, const char * parameter_name)
{
  if (!std::isfinite(value) || value <= 0.0 || value > 1.0) {
    throw std::invalid_argument(
            std::string(parameter_name) + " must be within (0.0, 1.0]");
  }
}

bool resolveScaling(
  double requested_value,
  double default_value,
  const char * field_name,
  double & resolved_value,
  std::string & reason)
{
  if (!std::isfinite(requested_value) || requested_value < 0.0 || requested_value > 1.0) {
    reason = std::string(field_name) + " must be finite and within [0.0, 1.0]";
    return false;
  }

  resolved_value = requested_value == 0.0 ? default_value : requested_value;
  return true;
}

bool validateJointDeltaLimit(
  const std::vector<double> & start,
  const std::vector<double> & target,
  double maximum_delta,
  const std::string & context,
  std::string & reason)
{
  if (start.size() != kExpectedActiveJoints.size() || target.size() != start.size()) {
    reason = context + " has an unexpected joint vector size";
    return false;
  }

  for (std::size_t index = 0; index < start.size(); ++index) {
    if (!std::isfinite(start[index]) || !std::isfinite(target[index])) {
      reason = context + " contains a non-finite value for " + kExpectedActiveJoints[index];
      return false;
    }

    const double delta = std::abs(target[index] - start[index]);
    if (delta > maximum_delta) {
      std::ostringstream message;
      message << context << " rejected: " << kExpectedActiveJoints[index]
              << " delta=" << delta << " rad exceeds limit=" << maximum_delta << " rad";
      reason = message.str();
      return false;
    }
  }

  reason.clear();
  return true;
}

bool validateTrajectoryPointField(
  const std::vector<double> & values,
  std::size_t expected_size,
  bool required,
  const std::string & point_context,
  const char * field_name,
  std::string & reason)
{
  if (!required && values.empty()) {
    return true;
  }
  if (values.size() != expected_size) {
    std::ostringstream message;
    message << point_context << " has " << values.size() << ' ' << field_name
            << " values, expected " << expected_size;
    reason = message.str();
    return false;
  }
  for (double value : values) {
    if (!std::isfinite(value)) {
      reason = point_context + " contains a non-finite " + field_name + " value";
      return false;
    }
  }
  return true;
}

}  // namespace

MotionCommandNode::MotionCommandNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("elfin3_motion_command", options)
{
  planning_group_ = declare_parameter<std::string>("planning_group", "elfin_arm");
  base_frame_ = declare_parameter<std::string>("base_frame", "elfin_base");
  tool_frame_ = declare_parameter<std::string>("tool_frame", "elfin_end_link");
  planning_pipeline_ = declare_parameter<std::string>("planning_pipeline", "ompl");
  planning_time_sec_ = declare_parameter<double>("planning_time_sec", 5.0);
  planning_attempts_ = declare_parameter<std::int64_t>("planning_attempts", 10);
  default_velocity_scaling_ =
    declare_parameter<double>("default_velocity_scaling", 0.2);
  default_acceleration_scaling_ =
    declare_parameter<double>("default_acceleration_scaling", 0.2);
  joint_tolerance_rad_ = declare_parameter<double>("joint_tolerance_rad", 0.01);
  position_tolerance_m_ = declare_parameter<double>("position_tolerance_m", 0.005);
  orientation_tolerance_rad_ =
    declare_parameter<double>("orientation_tolerance_rad", 0.02);
  move_pose_max_joint_delta_rad_ =
    declare_parameter<double>("move_pose_max_joint_delta_rad", 1.0);
  move_pose_ptp_ik_translation_step_m_ =
    declare_parameter<double>("move_pose_ptp_ik_translation_step_m", 0.05);
  move_pose_ptp_ik_rotation_step_rad_ =
    declare_parameter<double>(
    "move_pose_ptp_ik_rotation_step_rad", 0.17453292519943295);
  move_pose_ptp_max_ik_segments_ =
    declare_parameter<std::int64_t>("move_pose_ptp_max_ik_segments", 100);
  supervisor_timeout_sec_ = declare_parameter<double>("supervisor_timeout_sec", 0.5);
  canfd_hold_on_cancel_ = declare_parameter<bool>("canfd_hold_on_cancel", false);
  canfd_hold_service_ =
    declare_parameter<std::string>("canfd_hold_service", "/elfin3_canfd/hold");
  workspace_y_constraint_enabled_ =
    declare_parameter<bool>("workspace_y_constraint_enabled", false);
  workspace_min_tcp_y_m_ =
    declare_parameter<double>("workspace_min_tcp_y_m", 0.0);
  workspace_y_margin_m_ =
    declare_parameter<double>("workspace_y_margin_m", 0.0);
  workspace_fk_sample_max_joint_step_rad_ =
    declare_parameter<double>("workspace_fk_sample_max_joint_step_rad", 0.0);

  requireNonEmpty(planning_group_, "planning_group");
  requireNonEmpty(base_frame_, "base_frame");
  requireNonEmpty(tool_frame_, "tool_frame");
  requireNonEmpty(planning_pipeline_, "planning_pipeline");
  requirePositiveFinite(planning_time_sec_, "planning_time_sec");
  if (planning_attempts_ <= 0 ||
    planning_attempts_ >
    static_cast<std::int64_t>(std::numeric_limits<unsigned int>::max()))
  {
    throw std::invalid_argument("planning_attempts is outside the supported range");
  }
  requireScaling(default_velocity_scaling_, "default_velocity_scaling");
  requireScaling(default_acceleration_scaling_, "default_acceleration_scaling");
  requirePositiveFinite(joint_tolerance_rad_, "joint_tolerance_rad");
  requirePositiveFinite(position_tolerance_m_, "position_tolerance_m");
  requirePositiveFinite(orientation_tolerance_rad_, "orientation_tolerance_rad");
  requirePositiveFinite(move_pose_max_joint_delta_rad_, "move_pose_max_joint_delta_rad");
  requirePositiveFinite(
    move_pose_ptp_ik_translation_step_m_, "move_pose_ptp_ik_translation_step_m");
  requirePositiveFinite(
    move_pose_ptp_ik_rotation_step_rad_, "move_pose_ptp_ik_rotation_step_rad");
  if (move_pose_ptp_max_ik_segments_ <= 0 || move_pose_ptp_max_ik_segments_ > 10000) {
    throw std::invalid_argument("move_pose_ptp_max_ik_segments must be within [1, 10000]");
  }
  requirePositiveFinite(supervisor_timeout_sec_, "supervisor_timeout_sec");
  if (canfd_hold_on_cancel_) {
    requireNonEmpty(canfd_hold_service_, "canfd_hold_service");
  }
  if (!std::isfinite(workspace_min_tcp_y_m_) || workspace_min_tcp_y_m_ < 0.0) {
    throw std::invalid_argument(
            "workspace_min_tcp_y_m must be finite and non-negative");
  }
  if (!std::isfinite(workspace_y_margin_m_) || workspace_y_margin_m_ < 0.0) {
    throw std::invalid_argument(
            "workspace_y_margin_m must be finite and non-negative");
  }
  if (!std::isfinite(workspace_fk_sample_max_joint_step_rad_) ||
    workspace_fk_sample_max_joint_step_rad_ < 0.0)
  {
    throw std::invalid_argument(
            "workspace_fk_sample_max_joint_step_rad must be finite and non-negative");
  }
  if (workspace_y_constraint_enabled_) {
    requirePositiveFinite(workspace_min_tcp_y_m_, "workspace_min_tcp_y_m");
    requirePositiveFinite(
      workspace_fk_sample_max_joint_step_rad_,
      "workspace_fk_sample_max_joint_step_rad");
  }

  supervisor_client_ =
    create_client<SupervisorTrigger>("/elfin3_supervisor/is_ready");
  if (canfd_hold_on_cancel_) {
    canfd_hold_client_ = create_client<SupervisorTrigger>(canfd_hold_service_);
  }
  planning_metrics_publisher_ = create_publisher<PlanningMetrics>(
    "/elfin3_motion/planning_metrics", rclcpp::QoS(10).reliable());
  control_mode_subscription_ = create_subscription<std_msgs::msg::String>(
    "/elfin3/control_mode",
    rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local(),
    [this](std_msgs::msg::String::ConstSharedPtr message) {
      std::lock_guard<std::mutex> lock(command_mutex_);
      control_mode_ = message->data;
    });

  RCLCPP_INFO(
    get_logger(),
    "Motion command parameters validated: group=%s base=%s tool=%s pipeline=%s "
    "move_pose_max_joint_delta_rad=%.3f ptp_translation_step=%.3f "
    "ptp_rotation_step=%.3f ptp_max_segments=%ld canfd_hold_on_cancel=%s",
    planning_group_.c_str(), base_frame_.c_str(), tool_frame_.c_str(),
    planning_pipeline_.c_str(), move_pose_max_joint_delta_rad_,
    move_pose_ptp_ik_translation_step_m_, move_pose_ptp_ik_rotation_step_rad_,
    static_cast<long>(move_pose_ptp_max_ik_segments_),
    canfd_hold_on_cancel_ ? "true" : "false");
}

MotionCommandNode::~MotionCommandNode()
{
  {
    std::lock_guard<std::mutex> lock(command_mutex_);
    shutting_down_ = true;
  }
  command_cv_.notify_all();
  if (command_worker_.joinable()) {
    command_worker_.join();
  }
}

void MotionCommandNode::initializeMoveGroup()
{
  if (move_group_) {
    throw std::logic_error("MoveGroupInterface is already initialized");
  }

  move_group_ = std::make_unique<MoveGroupInterface>(
    shared_from_this(), planning_group_, nullptr,
    rclcpp::Duration::from_seconds(kMoveItInterfaceWaitSeconds));

  const auto robot_model = move_group_->getRobotModel();
  if (!robot_model) {
    throw std::runtime_error("MoveIt robot model is unavailable");
  }
  if (!robot_model->hasLinkModel(base_frame_)) {
    throw std::runtime_error("base_frame is not present in the robot model: " + base_frame_);
  }
  if (!robot_model->hasLinkModel(tool_frame_)) {
    throw std::runtime_error("tool_frame is not present in the robot model: " + tool_frame_);
  }

  const auto & active_joints = move_group_->getActiveJoints();
  bool joint_order_matches = active_joints.size() == kExpectedActiveJoints.size();
  if (joint_order_matches) {
    for (std::size_t index = 0; index < kExpectedActiveJoints.size(); ++index) {
      if (active_joints[index] != kExpectedActiveJoints[index]) {
        joint_order_matches = false;
        break;
      }
    }
  }
  if (!joint_order_matches) {
    std::ostringstream message;
    message << "unexpected active joint order:";
    for (const auto & joint_name : active_joints) {
      message << ' ' << joint_name;
    }
    throw std::runtime_error(message.str());
  }

  move_group_->setPlanningPipelineId(planning_pipeline_);
  move_group_->setPlanningTime(planning_time_sec_);
  move_group_->setNumPlanningAttempts(
    static_cast<unsigned int>(planning_attempts_));
  move_group_->setMaxVelocityScalingFactor(default_velocity_scaling_);
  move_group_->setMaxAccelerationScalingFactor(default_acceleration_scaling_);
  move_group_->setGoalJointTolerance(joint_tolerance_rad_);
  move_group_->setGoalPositionTolerance(position_tolerance_m_);
  move_group_->setGoalOrientationTolerance(orientation_tolerance_rad_);
  move_group_->setPoseReferenceFrame(base_frame_);
  if (!move_group_->setEndEffectorLink(tool_frame_)) {
    throw std::runtime_error("tool_frame cannot be used as the end-effector link: " + tool_frame_);
  }

  const auto current_state = move_group_->getCurrentState(kMoveItInterfaceWaitSeconds);
  if (!current_state) {
    throw std::runtime_error("current robot state is unavailable");
  }

  RCLCPP_INFO(
    get_logger(),
    "MoveIt interface ready: group=%s active_joints=%zu planning_frame=%s "
    "pose_reference=%s tool=%s current_state=OK",
    move_group_->getName().c_str(), active_joints.size(),
    move_group_->getPlanningFrame().c_str(),
    move_group_->getPoseReferenceFrame().c_str(),
    move_group_->getEndEffectorLink().c_str());

  command_worker_ = std::thread([this]() {commandWorker();});
  move_j_action_server_ = rclcpp_action::create_server<MoveJ>(
    this,
    "/elfin3_motion/move_j",
    [this](
      const rclcpp_action::GoalUUID & uuid,
      std::shared_ptr<const MoveJ::Goal> goal)
    {
      return handleMoveJGoal(uuid, goal);
    },
    [this](const std::shared_ptr<MoveJGoalHandle> goal_handle)
    {
      return handleMoveJCancel(goal_handle);
    },
    [this](const std::shared_ptr<MoveJGoalHandle> goal_handle)
    {
      handleMoveJAccepted(goal_handle);
    });

  move_pose_action_server_ = rclcpp_action::create_server<MovePose>(
    this,
    "/elfin3_motion/move_pose",
    [this](
      const rclcpp_action::GoalUUID & uuid,
      std::shared_ptr<const MovePose::Goal> goal)
    {
      return handleMovePoseGoal(uuid, goal);
    },
    [this](const std::shared_ptr<MovePoseGoalHandle> goal_handle)
    {
      return handleMovePoseCancel(goal_handle);
    },
    [this](const std::shared_ptr<MovePoseGoalHandle> goal_handle)
    {
      handleMovePoseAccepted(goal_handle);
    });

  move_pose_ptp_action_server_ = rclcpp_action::create_server<MovePose>(
    this,
    "/elfin3_motion/move_pose_ptp",
    [this](
      const rclcpp_action::GoalUUID & uuid,
      std::shared_ptr<const MovePose::Goal> goal)
    {
      return handleMovePosePtpGoal(uuid, goal);
    },
    [this](const std::shared_ptr<MovePoseGoalHandle> goal_handle)
    {
      return handleMovePosePtpCancel(goal_handle);
    },
    [this](const std::shared_ptr<MovePoseGoalHandle> goal_handle)
    {
      handleMovePosePtpAccepted(goal_handle);
    });

  move_j_cancel_client_ = rclcpp_action::create_client<MoveJ>(
    this, "/elfin3_motion/move_j");
  move_pose_cancel_client_ = rclcpp_action::create_client<MovePose>(
    this, "/elfin3_motion/move_pose");
  move_pose_ptp_cancel_client_ = rclcpp_action::create_client<MovePose>(
    this, "/elfin3_motion/move_pose_ptp");
  stop_service_ = create_service<SupervisorTrigger>(
    "/elfin3_motion/stop",
    [this](
      const std::shared_ptr<SupervisorTrigger::Request> request,
      std::shared_ptr<SupervisorTrigger::Response> response)
    {
      handleStop(request, response);
    });

  RCLCPP_INFO(
    get_logger(),
    "Motion endpoints ready: MoveJ=/elfin3_motion/move_j "
    "MovePose=/elfin3_motion/move_pose "
    "MovePosePTP=/elfin3_motion/move_pose_ptp Stop=/elfin3_motion/stop");
  if (workspace_y_constraint_enabled_) {
    RCLCPP_WARN(
      get_logger(),
      "TCP workspace Y guard enabled: frame=%s allowed_y>=%.6f m "
      "(min=%.6f margin=%.6f) fk_joint_step<=%.6f rad; "
      "sampled software guard only, not functional safety",
      base_frame_.c_str(), workspace_min_tcp_y_m_ + workspace_y_margin_m_,
      workspace_min_tcp_y_m_, workspace_y_margin_m_,
      workspace_fk_sample_max_joint_step_rad_);
  } else {
    RCLCPP_WARN(
      get_logger(),
      "TCP workspace Y guard is disabled; enable it and configure an explicitly "
      "validated positive workspace_min_tcp_y_m before automated real testing");
  }
}

MotionCommandNode::SupervisorCheckResult
MotionCommandNode::querySupervisorReadiness()
{
  if (!rclcpp::ok()) {
    return {
      SupervisorCheckStatus::kError,
      "ROS context is shutting down",
    };
  }

  if (!supervisor_client_->service_is_ready()) {
    return {
      SupervisorCheckStatus::kServiceUnavailable,
      "/elfin3_supervisor/is_ready is unavailable",
    };
  }

  try {
    const auto request = std::make_shared<SupervisorTrigger::Request>();
    auto future = supervisor_client_->async_send_request(request);
    const auto timeout = std::chrono::duration<double>(supervisor_timeout_sec_);

    if (future.wait_for(timeout) != std::future_status::ready) {
      supervisor_client_->remove_pending_request(future);
      return {
        SupervisorCheckStatus::kTimedOut,
        "/elfin3_supervisor/is_ready timed out",
      };
    }

    const auto response = future.get();
    const std::string message = response->message.empty() ?
      "supervisor returned an empty message" : response->message;
    return {
      response->success ?
      SupervisorCheckStatus::kReady : SupervisorCheckStatus::kNotReady,
      message,
    };
  } catch (const std::exception & exception) {
    return {
      SupervisorCheckStatus::kError,
      std::string("supervisor readiness request failed: ") + exception.what(),
    };
  }
}

void MotionCommandNode::checkSupervisorAtStartup()
{
  const auto result = querySupervisorReadiness();

  switch (result.status) {
    case SupervisorCheckStatus::kReady:
      RCLCPP_INFO(get_logger(), "Supervisor gate open: %s", result.message.c_str());
      break;
    case SupervisorCheckStatus::kNotReady:
      RCLCPP_WARN(get_logger(), "Supervisor gate closed: %s", result.message.c_str());
      break;
    case SupervisorCheckStatus::kServiceUnavailable:
      RCLCPP_WARN(get_logger(), "Supervisor unavailable: %s", result.message.c_str());
      break;
    case SupervisorCheckStatus::kTimedOut:
      RCLCPP_WARN(get_logger(), "Supervisor timeout: %s", result.message.c_str());
      break;
    case SupervisorCheckStatus::kError:
      RCLCPP_ERROR(get_logger(), "Supervisor check error: %s", result.message.c_str());
      break;
  }
}

bool MotionCommandNode::canFdHoldAvailable() const
{
  return !canfd_hold_on_cancel_ ||
         (canfd_hold_client_ && canfd_hold_client_->service_is_ready());
}

bool MotionCommandNode::requestCanFdHold(const char * reason)
{
  if (!canfd_hold_on_cancel_) {
    return true;
  }
  if (!canfd_hold_client_ || !canfd_hold_client_->service_is_ready()) {
    RCLCPP_ERROR(
      get_logger(), "CAN FD HOLD request failed for %s: service %s is unavailable",
      reason, canfd_hold_service_.c_str());
    return false;
  }

  const auto logger = get_logger();
  const auto service_name = canfd_hold_service_;
  const auto request_reason = std::string(reason);
  try {
    canfd_hold_client_->async_send_request(
      std::make_shared<SupervisorTrigger::Request>(),
      [logger, service_name, request_reason](
        rclcpp::Client<SupervisorTrigger>::SharedFuture future)
      {
        try {
          const auto response = future.get();
          if (response->success) {
            RCLCPP_INFO(
              logger, "CAN FD HOLD accepted for %s: %s", request_reason.c_str(),
              response->message.c_str());
          } else {
            RCLCPP_ERROR(
              logger, "CAN FD HOLD rejected for %s by %s: %s", request_reason.c_str(),
              service_name.c_str(), response->message.c_str());
          }
        } catch (const std::exception & exception) {
          RCLCPP_ERROR(
            logger, "CAN FD HOLD response failed for %s: %s", request_reason.c_str(),
            exception.what());
        }
      });
  } catch (const std::exception & exception) {
    RCLCPP_ERROR(
      get_logger(), "CAN FD HOLD request failed for %s: %s", reason, exception.what());
    return false;
  }

  RCLCPP_WARN(
    get_logger(), "CAN FD HOLD requested for %s through %s", reason,
    canfd_hold_service_.c_str());
  return true;
}

rclcpp_action::GoalResponse MotionCommandNode::handleMoveJGoal(
  const rclcpp_action::GoalUUID &,
  std::shared_ptr<const MoveJ::Goal>)
{
  if (!canFdHoldAvailable()) {
    RCLCPP_ERROR(
      get_logger(), "MoveJ goal rejected: required CAN FD HOLD service %s is unavailable",
      canfd_hold_service_.c_str());
    return rclcpp_action::GoalResponse::REJECT;
  }
  {
    std::lock_guard<std::mutex> lock(command_mutex_);
    if (control_mode_ != "MOVEIT") {
      RCLCPP_WARN(
        get_logger(), "MoveJ goal rejected: control mode is %s", control_mode_.c_str());
      return rclcpp_action::GoalResponse::REJECT;
    }
    if (shutting_down_) {
      RCLCPP_WARN(get_logger(), "MoveJ goal rejected: node is shutting down");
      return rclcpp_action::GoalResponse::REJECT;
    }
    if (command_state_ != CommandState::kIdle) {
      RCLCPP_WARN(get_logger(), "MoveJ goal rejected: another command is active");
      return rclcpp_action::GoalResponse::REJECT;
    }
    cancel_requested_ = false;
    command_state_ = CommandState::kValidating;
    command_type_ = CommandType::kMoveJ;
  }

  RCLCPP_INFO(get_logger(), "MoveJ goal accepted: state=VALIDATING");
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse MotionCommandNode::handleMoveJCancel(
  const std::shared_ptr<MoveJGoalHandle>)
{
  bool stop_execution = false;
  {
    std::lock_guard<std::mutex> lock(command_mutex_);
    if (shutting_down_ || command_state_ == CommandState::kIdle ||
      command_type_ != CommandType::kMoveJ)
    {
      RCLCPP_WARN(get_logger(), "MoveJ cancel rejected: no active command");
      return rclcpp_action::CancelResponse::REJECT;
    }
    cancel_requested_ = true;
    if (command_state_ == CommandState::kExecuting ||
      command_state_ == CommandState::kStopping)
    {
      command_state_ = CommandState::kStopping;
      stop_execution = true;
    }
  }

  if (stop_execution) {
    requestCanFdHold("MoveJ cancellation");
    move_group_->stop();
  }
  RCLCPP_INFO(get_logger(), "MoveJ cancel accepted");
  return rclcpp_action::CancelResponse::ACCEPT;
}

void MotionCommandNode::handleMoveJAccepted(
  const std::shared_ptr<MoveJGoalHandle> goal_handle)
{
  bool queued = false;
  {
    std::lock_guard<std::mutex> lock(command_mutex_);
    if (!shutting_down_ && command_state_ == CommandState::kValidating &&
      command_type_ == CommandType::kMoveJ && !pending_move_j_goal_ &&
      !pending_move_pose_goal_ && !pending_move_pose_ptp_goal_)
    {
      pending_move_j_goal_ = goal_handle;
      queued = true;
    }
  }

  if (queued) {
    command_cv_.notify_one();
    return;
  }

  auto result = std::make_shared<MoveJ::Result>();
  result->result_code = MoveJ::Result::INTERNAL_ERROR;
  result->moveit_error_code = 0;
  result->message = "MoveJ goal could not be queued";
  RCLCPP_ERROR(get_logger(), "%s", result->message.c_str());
  goal_handle->abort(result);
}

rclcpp_action::GoalResponse MotionCommandNode::handleMovePoseGoal(
  const rclcpp_action::GoalUUID &,
  std::shared_ptr<const MovePose::Goal>)
{
  if (!canFdHoldAvailable()) {
    RCLCPP_ERROR(
      get_logger(), "MovePose goal rejected: required CAN FD HOLD service %s is unavailable",
      canfd_hold_service_.c_str());
    return rclcpp_action::GoalResponse::REJECT;
  }
  {
    std::lock_guard<std::mutex> lock(command_mutex_);
    if (control_mode_ != "MOVEIT") {
      RCLCPP_WARN(
        get_logger(), "MovePose goal rejected: control mode is %s", control_mode_.c_str());
      return rclcpp_action::GoalResponse::REJECT;
    }
    if (shutting_down_) {
      RCLCPP_WARN(get_logger(), "MovePose goal rejected: node is shutting down");
      return rclcpp_action::GoalResponse::REJECT;
    }
    if (command_state_ != CommandState::kIdle) {
      RCLCPP_WARN(get_logger(), "MovePose goal rejected: another command is active");
      return rclcpp_action::GoalResponse::REJECT;
    }
    cancel_requested_ = false;
    command_state_ = CommandState::kValidating;
    command_type_ = CommandType::kMovePose;
  }

  RCLCPP_INFO(get_logger(), "MovePose goal accepted: state=VALIDATING");
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse MotionCommandNode::handleMovePoseCancel(
  const std::shared_ptr<MovePoseGoalHandle>)
{
  bool stop_execution = false;
  {
    std::lock_guard<std::mutex> lock(command_mutex_);
    if (shutting_down_ || command_state_ == CommandState::kIdle ||
      command_type_ != CommandType::kMovePose)
    {
      RCLCPP_WARN(get_logger(), "MovePose cancel rejected: no active MovePose command");
      return rclcpp_action::CancelResponse::REJECT;
    }
    cancel_requested_ = true;
    if (command_state_ == CommandState::kExecuting ||
      command_state_ == CommandState::kStopping)
    {
      command_state_ = CommandState::kStopping;
      stop_execution = true;
    }
  }

  if (stop_execution) {
    requestCanFdHold("MovePose cancellation");
    move_group_->stop();
  }
  RCLCPP_INFO(get_logger(), "MovePose cancel accepted");
  return rclcpp_action::CancelResponse::ACCEPT;
}

void MotionCommandNode::handleMovePoseAccepted(
  const std::shared_ptr<MovePoseGoalHandle> goal_handle)
{
  bool queued = false;
  {
    std::lock_guard<std::mutex> lock(command_mutex_);
    if (!shutting_down_ && command_state_ == CommandState::kValidating &&
      command_type_ == CommandType::kMovePose && !pending_move_j_goal_ &&
      !pending_move_pose_goal_ && !pending_move_pose_ptp_goal_)
    {
      pending_move_pose_goal_ = goal_handle;
      queued = true;
    }
  }

  if (queued) {
    command_cv_.notify_one();
    return;
  }

  auto result = std::make_shared<MovePose::Result>();
  result->result_code = MovePose::Result::INTERNAL_ERROR;
  result->moveit_error_code = 0;
  result->message = "MovePose goal could not be queued";
  RCLCPP_ERROR(get_logger(), "%s", result->message.c_str());
  goal_handle->abort(result);
}

rclcpp_action::GoalResponse MotionCommandNode::handleMovePosePtpGoal(
  const rclcpp_action::GoalUUID &,
  std::shared_ptr<const MovePose::Goal>)
{
  if (!canFdHoldAvailable()) {
    RCLCPP_ERROR(
      get_logger(),
      "MovePosePTP goal rejected: required CAN FD HOLD service %s is unavailable",
      canfd_hold_service_.c_str());
    return rclcpp_action::GoalResponse::REJECT;
  }

  {
    std::lock_guard<std::mutex> lock(command_mutex_);
    if (control_mode_ != "MOVEIT") {
      RCLCPP_WARN(
        get_logger(), "MovePosePTP goal rejected: control mode is %s",
        control_mode_.c_str());
      return rclcpp_action::GoalResponse::REJECT;
    }
    if (shutting_down_) {
      RCLCPP_WARN(get_logger(), "MovePosePTP goal rejected: node is shutting down");
      return rclcpp_action::GoalResponse::REJECT;
    }
    if (command_state_ != CommandState::kIdle) {
      RCLCPP_WARN(get_logger(), "MovePosePTP goal rejected: another command is active");
      return rclcpp_action::GoalResponse::REJECT;
    }
    cancel_requested_ = false;
    command_state_ = CommandState::kValidating;
    command_type_ = CommandType::kMovePosePtp;
  }

  RCLCPP_INFO(get_logger(), "MovePosePTP goal accepted: state=VALIDATING");
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse MotionCommandNode::handleMovePosePtpCancel(
  const std::shared_ptr<MovePoseGoalHandle>)
{
  bool stop_execution = false;
  {
    std::lock_guard<std::mutex> lock(command_mutex_);
    if (shutting_down_ || command_state_ == CommandState::kIdle ||
      command_type_ != CommandType::kMovePosePtp)
    {
      RCLCPP_WARN(
        get_logger(), "MovePosePTP cancel rejected: no active MovePosePTP command");
      return rclcpp_action::CancelResponse::REJECT;
    }
    cancel_requested_ = true;
    if (command_state_ == CommandState::kExecuting ||
      command_state_ == CommandState::kStopping)
    {
      command_state_ = CommandState::kStopping;
      stop_execution = true;
    }
  }

  if (stop_execution) {
    requestCanFdHold("MovePosePTP cancellation");
    move_group_->stop();
  }
  RCLCPP_INFO(get_logger(), "MovePosePTP cancel accepted");
  return rclcpp_action::CancelResponse::ACCEPT;
}

void MotionCommandNode::handleMovePosePtpAccepted(
  const std::shared_ptr<MovePoseGoalHandle> goal_handle)
{
  bool queued = false;
  {
    std::lock_guard<std::mutex> lock(command_mutex_);
    if (!shutting_down_ && command_state_ == CommandState::kValidating &&
      command_type_ == CommandType::kMovePosePtp && !pending_move_j_goal_ &&
      !pending_move_pose_goal_ && !pending_move_pose_ptp_goal_)
    {
      pending_move_pose_ptp_goal_ = goal_handle;
      queued = true;
    }
  }

  if (queued) {
    command_cv_.notify_one();
    return;
  }

  auto result = std::make_shared<MovePose::Result>();
  result->result_code = MovePose::Result::INTERNAL_ERROR;
  result->moveit_error_code = 0;
  result->message = "MovePosePTP goal could not be queued";
  RCLCPP_ERROR(get_logger(), "%s", result->message.c_str());
  goal_handle->abort(result);
}

void MotionCommandNode::handleStop(
  const std::shared_ptr<SupervisorTrigger::Request>,
  std::shared_ptr<SupervisorTrigger::Response> response)
{
  bool command_active = false;
  CommandType active_type = CommandType::kNone;
  {
    std::lock_guard<std::mutex> lock(command_mutex_);
    if (shutting_down_) {
      response->success = false;
      response->message = "Motion command node is shutting down";
      return;
    }
    command_active = command_state_ != CommandState::kIdle;
    active_type = command_type_;
  }

  if (!command_active) {
    response->success = true;
    response->message = "No active motion command; already stopped";
    return;
  }

  try {
    const auto logger = get_logger();
    if (active_type == CommandType::kMoveJ) {
      if (!move_j_cancel_client_->action_server_is_ready()) {
        throw std::runtime_error("MoveJ cancel route is unavailable");
      }
      move_j_cancel_client_->async_cancel_all_goals(
        [logger](const auto & cancel_response)
        {
          RCLCPP_INFO(
            logger, "MoveJ stop response: goals_canceling=%zu",
            cancel_response->goals_canceling.size());
        });
    } else if (active_type == CommandType::kMovePose) {
      if (!move_pose_cancel_client_->action_server_is_ready()) {
        throw std::runtime_error("MovePose cancel route is unavailable");
      }
      move_pose_cancel_client_->async_cancel_all_goals(
        [logger](const auto & cancel_response)
        {
          RCLCPP_INFO(
            logger, "MovePose stop response: goals_canceling=%zu",
            cancel_response->goals_canceling.size());
        });
    } else if (active_type == CommandType::kMovePosePtp) {
      if (!move_pose_ptp_cancel_client_->action_server_is_ready()) {
        throw std::runtime_error("MovePosePTP cancel route is unavailable");
      }
      move_pose_ptp_cancel_client_->async_cancel_all_goals(
        [logger](const auto & cancel_response)
        {
          RCLCPP_INFO(
            logger, "MovePosePTP stop response: goals_canceling=%zu",
            cancel_response->goals_canceling.size());
        });
    } else {
      throw std::logic_error("active command has no command type");
    }
    response->success = true;
    response->message = "Stop request accepted for the active motion command";
    RCLCPP_INFO(get_logger(), "%s", response->message.c_str());
  } catch (const std::exception & exception) {
    response->success = false;
    response->message =
      std::string("Failed to request motion cancellation: ") + exception.what();
    RCLCPP_ERROR(get_logger(), "%s", response->message.c_str());
  }
}

void MotionCommandNode::commandWorker()
{
  while (true) {
    std::shared_ptr<MoveJGoalHandle> move_j_goal_handle;
    std::shared_ptr<MovePoseGoalHandle> move_pose_goal_handle;
    std::shared_ptr<MovePoseGoalHandle> move_pose_ptp_goal_handle;
    bool stop_worker = false;
    {
      std::unique_lock<std::mutex> lock(command_mutex_);
      command_cv_.wait(
        lock,
        [this]()
        {
          return shutting_down_ || pending_move_j_goal_ || pending_move_pose_goal_ ||
                 pending_move_pose_ptp_goal_;
        });
      move_j_goal_handle = std::move(pending_move_j_goal_);
      move_pose_goal_handle = std::move(pending_move_pose_goal_);
      move_pose_ptp_goal_handle = std::move(pending_move_pose_ptp_goal_);
      stop_worker = shutting_down_;
    }

    if (stop_worker) {
      if (move_j_goal_handle && move_j_goal_handle->is_active()) {
        auto result = std::make_shared<MoveJ::Result>();
        result->result_code = MoveJ::Result::INTERNAL_ERROR;
        result->moveit_error_code = 0;
        result->message = "MoveJ command aborted because the node is shutting down";
        move_j_goal_handle->abort(result);
      }
      if (move_pose_goal_handle && move_pose_goal_handle->is_active()) {
        auto result = std::make_shared<MovePose::Result>();
        result->result_code = MovePose::Result::INTERNAL_ERROR;
        result->moveit_error_code = 0;
        result->message = "MovePose command aborted because the node is shutting down";
        move_pose_goal_handle->abort(result);
      }
      if (move_pose_ptp_goal_handle && move_pose_ptp_goal_handle->is_active()) {
        auto result = std::make_shared<MovePose::Result>();
        result->result_code = MovePose::Result::INTERNAL_ERROR;
        result->moveit_error_code = 0;
        result->message = "MovePosePTP command aborted because the node is shutting down";
        move_pose_ptp_goal_handle->abort(result);
      }
      return;
    }

    try {
      if (move_j_goal_handle) {
        processMoveJGoal(move_j_goal_handle);
      } else if (move_pose_goal_handle) {
        processMovePoseGoal(move_pose_goal_handle);
      } else if (move_pose_ptp_goal_handle) {
        processMovePosePtpGoal(move_pose_ptp_goal_handle);
      } else {
        throw std::logic_error("command worker woke without a pending goal");
      }
    } catch (const std::exception & exception) {
      if (move_j_goal_handle) {
        auto result = std::make_shared<MoveJ::Result>();
        result->result_code = MoveJ::Result::INTERNAL_ERROR;
        result->moveit_error_code = 0;
        result->message = std::string("MoveJ worker failed: ") + exception.what();
        RCLCPP_ERROR(get_logger(), "%s", result->message.c_str());
        if (move_j_goal_handle->is_active()) {
          move_j_goal_handle->abort(result);
        }
      } else if (move_pose_goal_handle) {
        auto result = std::make_shared<MovePose::Result>();
        result->result_code = MovePose::Result::INTERNAL_ERROR;
        result->moveit_error_code = 0;
        result->message = std::string("MovePose worker failed: ") + exception.what();
        RCLCPP_ERROR(get_logger(), "%s", result->message.c_str());
        if (move_pose_goal_handle->is_active()) {
          move_pose_goal_handle->abort(result);
        }
      } else if (move_pose_ptp_goal_handle) {
        auto result = std::make_shared<MovePose::Result>();
        result->result_code = MovePose::Result::INTERNAL_ERROR;
        result->moveit_error_code = 0;
        result->message = std::string("MovePosePTP worker failed: ") + exception.what();
        RCLCPP_ERROR(get_logger(), "%s", result->message.c_str());
        if (move_pose_ptp_goal_handle->is_active()) {
          move_pose_ptp_goal_handle->abort(result);
        }
      }
    }

    {
      std::lock_guard<std::mutex> lock(command_mutex_);
      command_state_ = CommandState::kIdle;
      command_type_ = CommandType::kNone;
      cancel_requested_ = false;
    }
    RCLCPP_INFO(get_logger(), "Motion command finished: state=IDLE");
  }
}

void MotionCommandNode::processMoveJGoal(
  const std::shared_ptr<MoveJGoalHandle> goal_handle)
{
  auto result = std::make_shared<MoveJ::Result>();
  result->moveit_error_code = 0;

  auto validating_feedback = std::make_shared<MoveJ::Feedback>();
  validating_feedback->stage = "VALIDATING";
  goal_handle->publish_feedback(validating_feedback);

  double velocity_scaling = 0.0;
  double acceleration_scaling = 0.0;
  std::string reason;
  if (!validateMoveJGoal(
      *goal_handle->get_goal(), velocity_scaling, acceleration_scaling, reason))
  {
    result->result_code = MoveJ::Result::INVALID_GOAL;
    result->message = reason;
    RCLCPP_WARN(get_logger(), "MoveJ validation failed: %s", reason.c_str());
    goal_handle->abort(result);
    return;
  }

  RCLCPP_INFO(
    get_logger(),
    "MoveJ goal validated: velocity_scaling=%.3f acceleration_scaling=%.3f",
    velocity_scaling, acceleration_scaling);

  if (cancelRequested()) {
    finishMoveJAsCanceled(goal_handle, 0, "MoveJ canceled during validation");
    return;
  }

  const auto supervisor = querySupervisorReadiness();
  if (cancelRequested()) {
    finishMoveJAsCanceled(goal_handle, 0, "MoveJ canceled before planning");
    return;
  }
  if (supervisor.status != SupervisorCheckStatus::kReady) {
    result->result_code = MoveJ::Result::NOT_READY;
    result->message = "MoveJ blocked by supervisor: " + supervisor.message;
    RCLCPP_WARN(get_logger(), "%s", result->message.c_str());
    goal_handle->abort(result);
    return;
  }

  const auto current_state = move_group_->getCurrentState(kMoveItInterfaceWaitSeconds);
  if (cancelRequested()) {
    finishMoveJAsCanceled(goal_handle, 0, "MoveJ canceled before planning");
    return;
  }
  if (!current_state) {
    result->result_code = MoveJ::Result::NOT_READY;
    result->message = "MoveJ current robot state is unavailable";
    RCLCPP_WARN(get_logger(), "%s", result->message.c_str());
    goal_handle->abort(result);
    return;
  }

  bool stop_before_planning = false;
  bool cancel_before_planning = false;
  {
    std::lock_guard<std::mutex> lock(command_mutex_);
    if (cancel_requested_) {
      cancel_before_planning = true;
    } else if (shutting_down_ || !rclcpp::ok()) {
      stop_before_planning = true;
    } else {
      command_state_ = CommandState::kPlanning;
    }
  }
  if (stop_before_planning) {
    result->result_code = MoveJ::Result::INTERNAL_ERROR;
    result->message = "MoveJ planning canceled because the node is shutting down";
    goal_handle->abort(result);
    return;
  }
  if (cancel_before_planning) {
    finishMoveJAsCanceled(goal_handle, 0, "MoveJ canceled before planning");
    return;
  }

  auto planning_feedback = std::make_shared<MoveJ::Feedback>();
  planning_feedback->stage = "PLANNING";
  goal_handle->publish_feedback(planning_feedback);

  move_group_->setStartState(*current_state);
  move_group_->setMaxVelocityScalingFactor(velocity_scaling);
  move_group_->setMaxAccelerationScalingFactor(acceleration_scaling);
  const std::vector<double> joint_target(
    goal_handle->get_goal()->joint_positions.begin(),
    goal_handle->get_goal()->joint_positions.end());
  if (!move_group_->setJointValueTarget(joint_target)) {
    result->result_code = MoveJ::Result::INVALID_GOAL;
    result->message = "MoveIt rejected the validated MoveJ joint target";
    RCLCPP_WARN(get_logger(), "%s", result->message.c_str());
    goal_handle->abort(result);
    return;
  }

  MoveGroupInterface::Plan plan;
  RCLCPP_INFO(get_logger(), "MoveJ planning started: state=PLANNING");
  const auto planning_result = move_group_->plan(plan);
  result->moveit_error_code = planning_result.val;
  if (cancelRequested()) {
    finishMoveJAsCanceled(
      goal_handle, planning_result.val, "MoveJ canceled during planning");
    return;
  }
  if (!static_cast<bool>(planning_result)) {
    result->result_code = MoveJ::Result::PLANNING_FAILED;
    std::ostringstream message;
    message << "MoveJ planning failed with MoveIt error code " << planning_result.val;
    result->message = message.str();
    RCLCPP_WARN(get_logger(), "%s", result->message.c_str());
    goal_handle->abort(result);
    return;
  }

  const auto trajectory_points = plan.trajectory_.joint_trajectory.points.size();
  const auto trajectory_analysis =
    analyzeTrajectory(*current_state, plan.trajectory_.joint_trajectory);
  if (!trajectory_analysis.valid) {
    result->result_code = MoveJ::Result::PLANNING_FAILED;
    result->moveit_error_code =
      moveit_msgs::msg::MoveItErrorCodes::INVALID_MOTION_PLAN;
    result->message = "MoveJ trajectory validation failed: " +
      trajectory_analysis.reason;
    RCLCPP_WARN(get_logger(), "%s", result->message.c_str());
    goal_handle->abort(result);
    return;
  }
  RCLCPP_INFO(
    get_logger(),
    "MoveJ planning succeeded: planning_time=%.3f trajectory_points=%zu "
    "max_adjacent_delta=%.9f minimum_tcp_y=%.9f fk_samples=%u",
    plan.planning_time_, trajectory_points,
    trajectory_analysis.maximum_adjacent_delta_rad,
    trajectory_analysis.minimum_tcp_y_m,
    trajectory_analysis.fk_samples_evaluated);

  const auto pre_execution_supervisor = querySupervisorReadiness();
  if (cancelRequested()) {
    finishMoveJAsCanceled(
      goal_handle, planning_result.val, "MoveJ canceled before execution");
    return;
  }
  if (pre_execution_supervisor.status != SupervisorCheckStatus::kReady) {
    result->result_code = MoveJ::Result::NOT_READY;
    result->message =
      "MoveJ execution blocked by supervisor: " + pre_execution_supervisor.message;
    RCLCPP_WARN(get_logger(), "%s", result->message.c_str());
    goal_handle->abort(result);
    return;
  }

  bool stop_before_execution = false;
  bool cancel_before_execution = false;
  {
    std::lock_guard<std::mutex> lock(command_mutex_);
    if (cancel_requested_) {
      cancel_before_execution = true;
    } else if (shutting_down_ || !rclcpp::ok()) {
      stop_before_execution = true;
    } else {
      command_state_ = CommandState::kExecuting;
    }
  }
  if (stop_before_execution) {
    result->result_code = MoveJ::Result::INTERNAL_ERROR;
    result->message = "MoveJ execution canceled because the node is shutting down";
    goal_handle->abort(result);
    return;
  }
  if (cancel_before_execution) {
    finishMoveJAsCanceled(
      goal_handle, planning_result.val, "MoveJ canceled before execution");
    return;
  }

  auto executing_feedback = std::make_shared<MoveJ::Feedback>();
  executing_feedback->stage = "EXECUTING";
  goal_handle->publish_feedback(executing_feedback);

  if (cancelRequested()) {
    finishMoveJAsCanceled(
      goal_handle, planning_result.val, "MoveJ canceled before execution");
    return;
  }

  RCLCPP_INFO(get_logger(), "MoveJ execution started: state=EXECUTING");
  const auto execution_result = move_group_->execute(plan);
  result->moveit_error_code = execution_result.val;
  if (cancelRequested()) {
    finishMoveJAsCanceled(
      goal_handle, execution_result.val, "MoveJ canceled during execution");
    return;
  }
  if (!static_cast<bool>(execution_result)) {
    requestCanFdHold("MoveJ execution failure");
    result->result_code = MoveJ::Result::EXECUTION_FAILED;
    std::ostringstream message;
    message << "MoveJ execution failed with MoveIt error code " << execution_result.val;
    result->message = message.str();
    RCLCPP_WARN(get_logger(), "%s", result->message.c_str());
    goal_handle->abort(result);
    return;
  }

  result->result_code = MoveJ::Result::SUCCESS;
  result->message = "MoveJ planning and execution succeeded";
  RCLCPP_INFO(get_logger(), "%s", result->message.c_str());
  goal_handle->succeed(result);
}

void MotionCommandNode::processMovePosePtpGoal(
  const std::shared_ptr<MovePoseGoalHandle> goal_handle)
{
  PlanningMetricsScope metrics(
    planning_metrics_publisher_, goal_handle->get_goal_id(), "move_pose_ptp",
    workspace_y_constraint_enabled_, workspace_min_tcp_y_m_,
    workspace_y_margin_m_, workspace_fk_sample_max_joint_step_rad_);
  metrics.setFailureStage("INPUT_VALIDATION");
  auto result = std::make_shared<MovePose::Result>();
  result->moveit_error_code = 0;

  auto validating_feedback = std::make_shared<MovePose::Feedback>();
  validating_feedback->stage = "VALIDATING";
  goal_handle->publish_feedback(validating_feedback);

  geometry_msgs::msg::PoseStamped normalized_target;
  double velocity_scaling = 0.0;
  double acceleration_scaling = 0.0;
  std::string reason;
  if (!validateMovePoseGoal(
      *goal_handle->get_goal(), normalized_target, velocity_scaling,
      acceleration_scaling, reason))
  {
    result->result_code = MovePose::Result::INVALID_GOAL;
    result->message = reason;
    RCLCPP_WARN(get_logger(), "MovePosePTP validation failed: %s", reason.c_str());
    goal_handle->abort(result);
    return;
  }

  RCLCPP_INFO(
    get_logger(),
    "MovePosePTP goal validated: velocity_scaling=%.3f acceleration_scaling=%.3f",
    velocity_scaling, acceleration_scaling);
  metrics.setFailureStage("SUPERVISOR");

  if (cancelRequested()) {
    finishMovePoseAsCanceled(goal_handle, 0, "MovePosePTP canceled during validation");
    return;
  }

  const auto supervisor = querySupervisorReadiness();
  if (cancelRequested()) {
    finishMovePoseAsCanceled(goal_handle, 0, "MovePosePTP canceled before start capture");
    return;
  }
  if (supervisor.status != SupervisorCheckStatus::kReady) {
    result->result_code = MovePose::Result::NOT_READY;
    result->message = "MovePosePTP blocked by supervisor: " + supervisor.message;
    RCLCPP_WARN(get_logger(), "%s", result->message.c_str());
    goal_handle->abort(result);
    return;
  }

  const auto current_state = move_group_->getCurrentState(kMoveItInterfaceWaitSeconds);
  if (cancelRequested()) {
    finishMovePoseAsCanceled(goal_handle, 0, "MovePosePTP canceled during start capture");
    return;
  }
  if (!current_state) {
    result->result_code = MovePose::Result::NOT_READY;
    result->message = "MovePosePTP current robot state is unavailable";
    RCLCPP_WARN(get_logger(), "%s", result->message.c_str());
    goal_handle->abort(result);
    return;
  }

  const auto robot_model = move_group_->getRobotModel();
  const auto * joint_model_group =
    robot_model ? robot_model->getJointModelGroup(planning_group_) : nullptr;
  if (!joint_model_group) {
    result->result_code = MovePose::Result::INTERNAL_ERROR;
    result->message = "MovePosePTP planning group is unavailable in the robot model";
    RCLCPP_ERROR(get_logger(), "%s", result->message.c_str());
    goal_handle->abort(result);
    return;
  }

  // This copy is the immutable reference for every later PTP stage. IK probes
  // and planning must use copies derived from it instead of re-reading a newer
  // state and silently changing the start of the command.
  moveit::core::RobotState captured_start_state(*current_state);
  captured_start_state.update();

  std::vector<double> captured_start_joints;
  captured_start_state.copyJointGroupPositions(
    joint_model_group, captured_start_joints);
  if (captured_start_joints.size() != kExpectedActiveJoints.size()) {
    result->result_code = MovePose::Result::INTERNAL_ERROR;
    result->message = "MovePosePTP captured start state has an unexpected joint count";
    RCLCPP_ERROR(get_logger(), "%s", result->message.c_str());
    goal_handle->abort(result);
    return;
  }
  for (std::size_t index = 0; index < captured_start_joints.size(); ++index) {
    if (!std::isfinite(captured_start_joints[index])) {
      result->result_code = MovePose::Result::NOT_READY;
      result->message = "MovePosePTP captured non-finite start position for " +
        std::string(kExpectedActiveJoints[index]);
      RCLCPP_WARN(get_logger(), "%s", result->message.c_str());
      goal_handle->abort(result);
      return;
    }
  }
  if (!captured_start_state.satisfiesBounds(joint_model_group)) {
    result->result_code = MovePose::Result::NOT_READY;
    result->message = "MovePosePTP captured start state violates joint bounds";
    RCLCPP_WARN(get_logger(), "%s", result->message.c_str());
    goal_handle->abort(result);
    return;
  }

  const Eigen::Isometry3d captured_start_transform =
    captured_start_state.getGlobalLinkTransform(tool_frame_);
  const Eigen::Isometry3d captured_base_transform =
    captured_start_state.getGlobalLinkTransform(base_frame_);
  const double captured_start_tcp_y =
    (captured_base_transform.inverse() * captured_start_transform).translation().y();
  if (workspace_y_constraint_enabled_ &&
    captured_start_tcp_y < workspace_min_tcp_y_m_ + workspace_y_margin_m_)
  {
    std::ostringstream message;
    message << "MovePosePTP start TCP y=" << captured_start_tcp_y
            << " m is below the configured workspace boundary="
            << workspace_min_tcp_y_m_ + workspace_y_margin_m_ << " m";
    result->result_code = MovePose::Result::NOT_READY;
    result->message = message.str();
    metrics.setFailureStage("START_WORKSPACE", result->message);
    RCLCPP_WARN(get_logger(), "%s", result->message.c_str());
    goal_handle->abort(result);
    return;
  }
  metrics.setEligible();
  metrics.setFailureStage("IK");
  Eigen::Quaterniond captured_start_orientation(captured_start_transform.rotation());
  captured_start_orientation.normalize();

  std::ostringstream capture_message;
  capture_message << "MovePosePTP captured start: joints=";
  for (std::size_t index = 0; index < captured_start_joints.size(); ++index) {
    if (index != 0U) {
      capture_message << ',';
    }
    capture_message << kExpectedActiveJoints[index] << '=' << captured_start_joints[index];
  }
  capture_message << " pose_xyz=[" << captured_start_transform.translation().x() << ','
                  << captured_start_transform.translation().y() << ','
                  << captured_start_transform.translation().z() << "] pose_xyzw=["
                  << captured_start_orientation.x() << ',' << captured_start_orientation.y()
                  << ',' << captured_start_orientation.z() << ','
                  << captured_start_orientation.w() << "] target_xyz=["
                  << normalized_target.pose.position.x << ','
                  << normalized_target.pose.position.y << ','
                  << normalized_target.pose.position.z << ']';
  RCLCPP_INFO(get_logger(), "%s", capture_message.str().c_str());

  auto captured_feedback = std::make_shared<MovePose::Feedback>();
  captured_feedback->stage = "START_CAPTURED";
  goal_handle->publish_feedback(captured_feedback);

  if (cancelRequested()) {
    finishMovePoseAsCanceled(goal_handle, 0, "MovePosePTP canceled after start capture");
    return;
  }

  // normalized_target is expressed in base_frame_. RobotState::setFromIK()
  // expects a pose in the robot model frame, so perform the conversion
  // explicitly instead of assuming that world and elfin_base are identical.
  Eigen::Isometry3d base_target_transform = Eigen::Isometry3d::Identity();
  base_target_transform.translation() = Eigen::Vector3d(
    normalized_target.pose.position.x,
    normalized_target.pose.position.y,
    normalized_target.pose.position.z);
  const Eigen::Quaterniond base_target_orientation(
    normalized_target.pose.orientation.w,
    normalized_target.pose.orientation.x,
    normalized_target.pose.orientation.y,
    normalized_target.pose.orientation.z);
  base_target_transform.linear() = base_target_orientation.toRotationMatrix();
  const Eigen::Isometry3d model_target_transform =
    captured_start_state.getGlobalLinkTransform(base_frame_) * base_target_transform;

  Eigen::Quaterniond model_target_orientation(model_target_transform.rotation());
  model_target_orientation.normalize();

  auto validate_ik_candidate = [this, joint_model_group](
    moveit::core::RobotState & candidate_state,
    const Eigen::Isometry3d & expected_tool_transform,
    const std::vector<double> & seed_joints,
    const std::string & context,
    std::vector<double> & candidate_joints,
    double & position_error,
    double & orientation_error,
    std::string & validation_reason) -> bool
    {
      candidate_state.update();
      candidate_state.copyJointGroupPositions(joint_model_group, candidate_joints);
      if (candidate_joints.size() != kExpectedActiveJoints.size()) {
        validation_reason = context + " returned an unexpected joint count";
        return false;
      }
      for (std::size_t index = 0; index < candidate_joints.size(); ++index) {
        if (!std::isfinite(candidate_joints[index])) {
          validation_reason = context + " returned a non-finite position for " +
            std::string(kExpectedActiveJoints[index]);
          return false;
        }
      }
      if (!candidate_state.satisfiesBounds(joint_model_group)) {
        validation_reason = context + " violates joint bounds";
        return false;
      }

      const Eigen::Isometry3d solved_tool_transform =
        candidate_state.getGlobalLinkTransform(tool_frame_);
      position_error =
        (solved_tool_transform.translation() - expected_tool_transform.translation()).norm();
      Eigen::Quaterniond solved_orientation(solved_tool_transform.rotation());
      Eigen::Quaterniond expected_orientation(expected_tool_transform.rotation());
      solved_orientation.normalize();
      expected_orientation.normalize();
      orientation_error = solved_orientation.angularDistance(expected_orientation);
      if (!std::isfinite(position_error) || !std::isfinite(orientation_error) ||
        position_error > position_tolerance_m_ ||
        orientation_error > orientation_tolerance_rad_)
      {
        std::ostringstream message;
        message << context << " residual exceeds tolerance: position_error="
                << position_error << " m orientation_error=" << orientation_error << " rad";
        validation_reason = message.str();
        return false;
      }

      return validateJointDeltaLimit(
        seed_joints, candidate_joints, move_pose_max_joint_delta_rad_,
        context, validation_reason);
    };

  // Try the target once using exactly the captured q0 as the solver seed.
  // A zero timeout selects the non-searching solver path.
  moveit::core::RobotState direct_ik_state(captured_start_state);
  std::vector<double> direct_ik_joints;
  double direct_position_error = std::numeric_limits<double>::infinity();
  double direct_orientation_error = std::numeric_limits<double>::infinity();
  std::string direct_ik_reason;
  const bool direct_ik_returned = direct_ik_state.setFromIK(
    joint_model_group, model_target_transform, tool_frame_, 0.0);
  const bool direct_ik_accepted = direct_ik_returned && validate_ik_candidate(
    direct_ik_state, model_target_transform, captured_start_joints,
    "MovePosePTP direct IK target", direct_ik_joints,
    direct_position_error, direct_orientation_error, direct_ik_reason);
  if (!direct_ik_returned) {
    direct_ik_reason = "solver returned no solution from captured q0";
  }

  std::vector<double> resolved_ik_joints;
  std::string ik_method;
  std::size_t segmented_ik_count = 0U;
  double maximum_segment_joint_delta = 0.0;

  if (direct_ik_accepted) {
    resolved_ik_joints = direct_ik_joints;
    ik_method = "direct";

    auto direct_ik_feedback = std::make_shared<MovePose::Feedback>();
    direct_ik_feedback->stage = "DIRECT_IK_SOLVED";
    goal_handle->publish_feedback(direct_ik_feedback);
  } else {
    RCLCPP_INFO(
      get_logger(), "MovePosePTP direct IK requires segmented fallback: %s",
      direct_ik_reason.c_str());

    const double translation_distance =
      (model_target_transform.translation() - captured_start_transform.translation()).norm();
    const double rotation_distance =
      captured_start_orientation.angularDistance(model_target_orientation);
    const double required_translation_segments =
      std::ceil(translation_distance / move_pose_ptp_ik_translation_step_m_);
    const double required_rotation_segments =
      std::ceil(rotation_distance / move_pose_ptp_ik_rotation_step_rad_);
    const double required_segments = std::max(
      {1.0, required_translation_segments, required_rotation_segments});
    if (!std::isfinite(required_segments) ||
      required_segments > static_cast<double>(move_pose_ptp_max_ik_segments_))
    {
      std::ostringstream message;
      message << "MovePosePTP segmented IK requires " << required_segments
              << " segments, exceeding limit=" << move_pose_ptp_max_ik_segments_;
      result->result_code = MovePose::Result::INVALID_GOAL;
      result->moveit_error_code = moveit_msgs::msg::MoveItErrorCodes::INVALID_GOAL_CONSTRAINTS;
      result->message = message.str();
      RCLCPP_WARN(get_logger(), "%s", result->message.c_str());
      goal_handle->abort(result);
      return;
    }
    segmented_ik_count = static_cast<std::size_t>(required_segments);

    RCLCPP_INFO(
      get_logger(),
      "MovePosePTP segmented IK started: segments=%zu translation=%.6f m rotation=%.6f rad",
      segmented_ik_count, translation_distance, rotation_distance);
    auto segmented_feedback = std::make_shared<MovePose::Feedback>();
    segmented_feedback->stage = "SEGMENTED_IK";
    goal_handle->publish_feedback(segmented_feedback);

    moveit::core::RobotState segmented_ik_state(captured_start_state);
    std::vector<double> previous_joints = captured_start_joints;
    const Eigen::Vector3d start_translation = captured_start_transform.translation();
    const Eigen::Vector3d target_translation = model_target_transform.translation();

    for (std::size_t segment_index = 1U;
      segment_index <= segmented_ik_count; ++segment_index)
    {
      if (cancelRequested()) {
        finishMovePoseAsCanceled(
          goal_handle, 0, "MovePosePTP canceled during segmented IK");
        return;
      }

      const double ratio =
        static_cast<double>(segment_index) / static_cast<double>(segmented_ik_count);
      Eigen::Isometry3d waypoint_transform = Eigen::Isometry3d::Identity();
      waypoint_transform.translation() =
        start_translation + ratio * (target_translation - start_translation);
      Eigen::Quaterniond waypoint_orientation =
        captured_start_orientation.slerp(ratio, model_target_orientation);
      waypoint_orientation.normalize();
      waypoint_transform.linear() = waypoint_orientation.toRotationMatrix();

      if (!segmented_ik_state.setFromIK(
          joint_model_group, waypoint_transform, tool_frame_, 0.0))
      {
        std::ostringstream message;
        message << "MovePosePTP segmented IK failed at segment " << segment_index
                << '/' << segmented_ik_count << ": solver returned no solution";
        result->result_code = MovePose::Result::PLANNING_FAILED;
        result->moveit_error_code = moveit_msgs::msg::MoveItErrorCodes::NO_IK_SOLUTION;
        result->message = message.str();
        RCLCPP_WARN(get_logger(), "%s", result->message.c_str());
        goal_handle->abort(result);
        return;
      }

      std::vector<double> waypoint_joints;
      double waypoint_position_error = 0.0;
      double waypoint_orientation_error = 0.0;
      std::string waypoint_reason;
      const std::string waypoint_context =
        "MovePosePTP segmented IK point " + std::to_string(segment_index);
      if (!validate_ik_candidate(
          segmented_ik_state, waypoint_transform, previous_joints,
          waypoint_context, waypoint_joints, waypoint_position_error,
          waypoint_orientation_error, waypoint_reason))
      {
        result->result_code = MovePose::Result::PLANNING_FAILED;
        result->moveit_error_code = moveit_msgs::msg::MoveItErrorCodes::PLANNING_FAILED;
        result->message = waypoint_reason;
        RCLCPP_WARN(get_logger(), "%s", result->message.c_str());
        goal_handle->abort(result);
        return;
      }

      for (std::size_t index = 0; index < waypoint_joints.size(); ++index) {
        maximum_segment_joint_delta = std::max(
          maximum_segment_joint_delta,
          std::abs(waypoint_joints[index] - previous_joints[index]));
      }
      previous_joints = std::move(waypoint_joints);
    }

    resolved_ik_joints = previous_joints;
    ik_method = "segmented";
    auto segmented_solved_feedback = std::make_shared<MovePose::Feedback>();
    segmented_solved_feedback->stage = "SEGMENTED_IK_SOLVED";
    goal_handle->publish_feedback(segmented_solved_feedback);
  }

  metrics.setIkResult(
    ik_method, static_cast<std::uint32_t>(
      std::min<std::size_t>(
        segmented_ik_count, std::numeric_limits<std::uint32_t>::max())));
  metrics.setFailureStage("TARGET_LOCK");
  std::ostringstream resolved_ik_message;
  resolved_ik_message << "MovePosePTP IK target resolved: method=" << ik_method
                      << " segments=" << segmented_ik_count;
  for (std::size_t index = 0; index < resolved_ik_joints.size(); ++index) {
    resolved_ik_message << ' ' << kExpectedActiveJoints[index] << '='
                        << resolved_ik_joints[index] << "(total_delta="
                        << resolved_ik_joints[index] - captured_start_joints[index] << ')';
  }
  resolved_ik_message << " max_segment_delta=" << maximum_segment_joint_delta;
  RCLCPP_INFO(get_logger(), "%s", resolved_ik_message.str().c_str());

  if (cancelRequested()) {
    finishMovePoseAsCanceled(goal_handle, 0, "MovePosePTP canceled after IK resolution");
    return;
  }

  // Remove any pose constraints left by an earlier command. From this point
  // onward MoveGroup receives only the captured q0 and the resolved six-axis
  // qN, so a later planner cannot invoke IK again or select another branch.
  move_group_->clearPoseTargets();
  move_group_->setStartState(captured_start_state);
  move_group_->setMaxVelocityScalingFactor(velocity_scaling);
  move_group_->setMaxAccelerationScalingFactor(acceleration_scaling);
  if (!move_group_->setJointValueTarget(resolved_ik_joints)) {
    result->result_code = MovePose::Result::INVALID_GOAL;
    result->moveit_error_code = moveit_msgs::msg::MoveItErrorCodes::INVALID_GOAL_CONSTRAINTS;
    result->message = "MovePosePTP failed to lock the resolved IK joint target";
    RCLCPP_WARN(get_logger(), "%s", result->message.c_str());
    goal_handle->abort(result);
    return;
  }

  std::vector<double> locked_joint_target;
  move_group_->getJointValueTarget(locked_joint_target);
  if (locked_joint_target.size() != resolved_ik_joints.size()) {
    result->result_code = MovePose::Result::INTERNAL_ERROR;
    result->moveit_error_code = moveit_msgs::msg::MoveItErrorCodes::FAILURE;
    result->message = "MovePosePTP locked joint target has an unexpected joint count";
    RCLCPP_ERROR(get_logger(), "%s", result->message.c_str());
    goal_handle->abort(result);
    return;
  }

  constexpr double kLockedTargetToleranceRad = 1.0e-12;
  for (std::size_t index = 0; index < locked_joint_target.size(); ++index) {
    const double lock_error =
      std::abs(locked_joint_target[index] - resolved_ik_joints[index]);
    if (!std::isfinite(locked_joint_target[index]) ||
      lock_error > kLockedTargetToleranceRad)
    {
      std::ostringstream message;
      message << "MovePosePTP locked target mismatch for " << kExpectedActiveJoints[index]
              << ": requested=" << resolved_ik_joints[index]
              << " stored=" << locked_joint_target[index]
              << " error=" << lock_error << " rad";
      result->result_code = MovePose::Result::INTERNAL_ERROR;
      result->moveit_error_code = moveit_msgs::msg::MoveItErrorCodes::FAILURE;
      result->message = message.str();
      RCLCPP_ERROR(get_logger(), "%s", result->message.c_str());
      goal_handle->abort(result);
      return;
    }
  }

  std::ostringstream locked_target_message;
  locked_target_message << "MovePosePTP locked joint target: method=" << ik_method;
  for (std::size_t index = 0; index < locked_joint_target.size(); ++index) {
    locked_target_message << ' ' << kExpectedActiveJoints[index] << '='
                          << locked_joint_target[index];
  }
  RCLCPP_INFO(get_logger(), "%s", locked_target_message.str().c_str());

  auto target_locked_feedback = std::make_shared<MovePose::Feedback>();
  target_locked_feedback->stage = "JOINT_TARGET_LOCKED";
  goal_handle->publish_feedback(target_locked_feedback);

  if (cancelRequested()) {
    finishMovePoseAsCanceled(goal_handle, 0, "MovePosePTP canceled after target locking");
    return;
  }

  bool stop_before_planning = false;
  bool cancel_before_planning = false;
  {
    std::lock_guard<std::mutex> lock(command_mutex_);
    if (cancel_requested_) {
      cancel_before_planning = true;
    } else if (shutting_down_ || !rclcpp::ok()) {
      stop_before_planning = true;
    } else {
      command_state_ = CommandState::kPlanning;
    }
  }
  if (stop_before_planning) {
    result->result_code = MovePose::Result::INTERNAL_ERROR;
    result->moveit_error_code = moveit_msgs::msg::MoveItErrorCodes::FAILURE;
    result->message = "MovePosePTP planning canceled because the node is shutting down";
    RCLCPP_WARN(get_logger(), "%s", result->message.c_str());
    goal_handle->abort(result);
    return;
  }
  if (cancel_before_planning) {
    finishMovePoseAsCanceled(goal_handle, 0, "MovePosePTP canceled before planning");
    return;
  }

  auto planning_feedback = std::make_shared<MovePose::Feedback>();
  planning_feedback->stage = "PLANNING";
  goal_handle->publish_feedback(planning_feedback);
  metrics.setFailureStage("OMPL");

  // Both IK paths converge here. OMPL receives one fixed q0-to-qN request;
  // segmented IK waypoints are deliberately not added as path constraints or
  // separate goals, so this produces one complete joint-space trajectory.
  MoveGroupInterface::Plan ptp_plan;
  RCLCPP_INFO(
    get_logger(), "MovePosePTP planning started: method=%s state=PLANNING",
    ik_method.c_str());
  const auto planning_result = move_group_->plan(ptp_plan);
  result->moveit_error_code = planning_result.val;
  if (cancelRequested()) {
    finishMovePoseAsCanceled(
      goal_handle, planning_result.val, "MovePosePTP canceled during planning");
    return;
  }
  if (!static_cast<bool>(planning_result)) {
    result->result_code = MovePose::Result::PLANNING_FAILED;
    std::ostringstream message;
    message << "MovePosePTP planning failed with MoveIt error code " << planning_result.val;
    result->message = message.str();
    RCLCPP_WARN(get_logger(), "%s", result->message.c_str());
    goal_handle->abort(result);
    return;
  }

  const std::size_t trajectory_points =
    ptp_plan.trajectory_.joint_trajectory.points.size();
  metrics.setOmplResult(ptp_plan.planning_time_, trajectory_points);
  RCLCPP_INFO(
    get_logger(),
    "MovePosePTP planning succeeded: planning_time=%.3f trajectory_points=%zu",
    ptp_plan.planning_time_, trajectory_points);

  auto planned_feedback = std::make_shared<MovePose::Feedback>();
  planned_feedback->stage = "PLANNED";
  goal_handle->publish_feedback(planned_feedback);

  if (cancelRequested()) {
    finishMovePoseAsCanceled(
      goal_handle, planning_result.val, "MovePosePTP canceled after planning");
    return;
  }

  auto trajectory_validation_feedback = std::make_shared<MovePose::Feedback>();
  trajectory_validation_feedback->stage = "VALIDATING_TRAJECTORY";
  goal_handle->publish_feedback(trajectory_validation_feedback);
  metrics.setFailureStage("TRAJECTORY_VALIDATION");

  const auto & joint_trajectory = ptp_plan.trajectory_.joint_trajectory;
  const auto & multi_dof_trajectory = ptp_plan.trajectory_.multi_dof_joint_trajectory;
  std::array<std::size_t, kExpectedActiveJoints.size()> trajectory_joint_indices{};
  bool trajectory_is_safe = true;
  if (joint_trajectory.points.empty()) {
    reason = "MovePosePTP planned trajectory contains no points";
    trajectory_is_safe = false;
  } else if (joint_trajectory.joint_names.size() != kExpectedActiveJoints.size()) {
    reason = "MovePosePTP planned trajectory has an unexpected joint count";
    trajectory_is_safe = false;
  } else if (!multi_dof_trajectory.points.empty()) {
    reason = "MovePosePTP planned trajectory unexpectedly contains multi-DOF points";
    trajectory_is_safe = false;
  }

  for (std::size_t index = 0;
    trajectory_is_safe && index < kExpectedActiveJoints.size(); ++index)
  {
    const auto joint_iterator = std::find(
      joint_trajectory.joint_names.begin(), joint_trajectory.joint_names.end(),
      kExpectedActiveJoints[index]);
    if (joint_iterator == joint_trajectory.joint_names.end()) {
      reason = "MovePosePTP planned trajectory is missing " +
        std::string(kExpectedActiveJoints[index]);
      trajectory_is_safe = false;
      break;
    }
    if (std::count(
        joint_trajectory.joint_names.begin(), joint_trajectory.joint_names.end(),
        kExpectedActiveJoints[index]) != 1)
    {
      reason = "MovePosePTP planned trajectory contains duplicate joint " +
        std::string(kExpectedActiveJoints[index]);
      trajectory_is_safe = false;
      break;
    }
    trajectory_joint_indices[index] = static_cast<std::size_t>(
      std::distance(joint_trajectory.joint_names.begin(), joint_iterator));
  }

  const double endpoint_tolerance = joint_tolerance_rad_ + 1.0e-6;
  std::vector<double> previous_waypoint;
  std::int64_t previous_time_ns = -1;
  double maximum_start_error = 0.0;
  double maximum_endpoint_error = 0.0;
  double maximum_adjacent_delta = 0.0;
  for (std::size_t point_index = 0;
    trajectory_is_safe && point_index < joint_trajectory.points.size(); ++point_index)
  {
    if (cancelRequested()) {
      finishMovePoseAsCanceled(
        goal_handle, planning_result.val,
        "MovePosePTP canceled during trajectory validation");
      return;
    }

    const auto & point = joint_trajectory.points[point_index];
    const std::string point_context =
      "MovePosePTP planned point " + std::to_string(point_index);
    trajectory_is_safe = validateTrajectoryPointField(
      point.positions, joint_trajectory.joint_names.size(), true,
      point_context, "position", reason);
    if (trajectory_is_safe) {
      trajectory_is_safe = validateTrajectoryPointField(
        point.velocities, joint_trajectory.joint_names.size(), false,
        point_context, "velocity", reason);
    }
    if (trajectory_is_safe) {
      trajectory_is_safe = validateTrajectoryPointField(
        point.accelerations, joint_trajectory.joint_names.size(), false,
        point_context, "acceleration", reason);
    }
    if (trajectory_is_safe) {
      trajectory_is_safe = validateTrajectoryPointField(
        point.effort, joint_trajectory.joint_names.size(), false,
        point_context, "effort", reason);
    }
    if (!trajectory_is_safe) {
      break;
    }

    if (point.time_from_start.sec < 0 || point.time_from_start.nanosec >= 1000000000U) {
      reason = point_context + " has an invalid time_from_start";
      trajectory_is_safe = false;
      break;
    }
    const std::int64_t point_time_ns =
      static_cast<std::int64_t>(point.time_from_start.sec) * 1000000000LL +
      static_cast<std::int64_t>(point.time_from_start.nanosec);
    if (point_index > 0U && point_time_ns <= previous_time_ns) {
      reason = point_context + " does not have a strictly increasing time_from_start";
      trajectory_is_safe = false;
      break;
    }
    previous_time_ns = point_time_ns;

    std::vector<double> waypoint(kExpectedActiveJoints.size());
    for (std::size_t joint_index = 0; joint_index < waypoint.size(); ++joint_index) {
      waypoint[joint_index] = point.positions[trajectory_joint_indices[joint_index]];
    }

    moveit::core::RobotState waypoint_state(captured_start_state);
    for (std::size_t joint_index = 0; joint_index < waypoint.size(); ++joint_index) {
      waypoint_state.setVariablePosition(
        kExpectedActiveJoints[joint_index], waypoint[joint_index]);
    }
    waypoint_state.update();
    if (!waypoint_state.satisfiesBounds(joint_model_group)) {
      reason = point_context + " violates joint bounds";
      trajectory_is_safe = false;
      break;
    }

    if (point_index == 0U) {
      for (std::size_t joint_index = 0; joint_index < waypoint.size(); ++joint_index) {
        maximum_start_error = std::max(
          maximum_start_error,
          std::abs(waypoint[joint_index] - captured_start_joints[joint_index]));
      }
      if (maximum_start_error > endpoint_tolerance) {
        std::ostringstream message;
        message << "MovePosePTP planned start differs from captured q0: max_error="
                << maximum_start_error << " rad tolerance=" << endpoint_tolerance << " rad";
        reason = message.str();
        trajectory_is_safe = false;
        break;
      }
    } else {
      if (!validateJointDeltaLimit(
          previous_waypoint, waypoint, move_pose_max_joint_delta_rad_,
          point_context + " adjacent transition", reason))
      {
        trajectory_is_safe = false;
        break;
      }
      for (std::size_t joint_index = 0; joint_index < waypoint.size(); ++joint_index) {
        maximum_adjacent_delta = std::max(
          maximum_adjacent_delta,
          std::abs(waypoint[joint_index] - previous_waypoint[joint_index]));
      }
    }
    previous_waypoint = std::move(waypoint);
  }

  if (trajectory_is_safe) {
    for (std::size_t joint_index = 0;
      joint_index < locked_joint_target.size(); ++joint_index)
    {
      maximum_endpoint_error = std::max(
        maximum_endpoint_error,
        std::abs(previous_waypoint[joint_index] - locked_joint_target[joint_index]));
    }
    if (maximum_endpoint_error > endpoint_tolerance) {
      std::ostringstream message;
      message << "MovePosePTP planned endpoint differs from locked qN: max_error="
              << maximum_endpoint_error << " rad tolerance=" << endpoint_tolerance << " rad";
      reason = message.str();
      trajectory_is_safe = false;
    }
  }

  TrajectoryAnalysisResult trajectory_analysis;
  if (trajectory_is_safe) {
    trajectory_analysis = analyzeTrajectory(captured_start_state, joint_trajectory);
    maximum_adjacent_delta = std::max(
      maximum_adjacent_delta,
      trajectory_analysis.maximum_adjacent_delta_rad);
    metrics.setTrajectoryAnalysis(
      maximum_adjacent_delta, trajectory_analysis.minimum_tcp_y_m,
      trajectory_analysis.fk_samples_evaluated, trajectory_analysis.valid);
    if (!trajectory_analysis.valid) {
      reason = trajectory_analysis.reason;
      trajectory_is_safe = false;
    }
  }

  if (!trajectory_is_safe) {
    metrics.setFailureStage("TRAJECTORY_VALIDATION", reason);
    result->result_code = MovePose::Result::PLANNING_FAILED;
    result->moveit_error_code = moveit_msgs::msg::MoveItErrorCodes::INVALID_MOTION_PLAN;
    result->message = reason;
    RCLCPP_WARN(
      get_logger(), "MovePosePTP trajectory validation failed: %s", reason.c_str());
    goal_handle->abort(result);
    return;
  }

  RCLCPP_INFO(
    get_logger(),
    "MovePosePTP trajectory validated: points=%zu duration=%.6f s "
    "start_error=%.9f endpoint_error=%.9f max_adjacent_delta=%.9f rad "
    "minimum_tcp_y=%.9f m fk_samples=%u",
    trajectory_points, static_cast<double>(previous_time_ns) / 1.0e9,
    maximum_start_error, maximum_endpoint_error, maximum_adjacent_delta,
    trajectory_analysis.minimum_tcp_y_m,
    trajectory_analysis.fk_samples_evaluated);
  metrics.markPlanningSuccess();

  auto trajectory_validated_feedback = std::make_shared<MovePose::Feedback>();
  trajectory_validated_feedback->stage = "TRAJECTORY_VALIDATED";
  goal_handle->publish_feedback(trajectory_validated_feedback);

  if (cancelRequested()) {
    finishMovePoseAsCanceled(
      goal_handle, planning_result.val,
      "MovePosePTP canceled after trajectory validation");
    return;
  }

  auto pre_execution_feedback = std::make_shared<MovePose::Feedback>();
  pre_execution_feedback->stage = "PRE_EXECUTION_CHECK";
  goal_handle->publish_feedback(pre_execution_feedback);

  const auto pre_execution_supervisor = querySupervisorReadiness();
  if (cancelRequested()) {
    finishMovePoseAsCanceled(
      goal_handle, planning_result.val,
      "MovePosePTP canceled during pre-execution readiness check");
    return;
  }
  if (pre_execution_supervisor.status != SupervisorCheckStatus::kReady) {
    result->result_code = MovePose::Result::NOT_READY;
    result->message =
      "MovePosePTP execution blocked by supervisor: " +
      pre_execution_supervisor.message;
    RCLCPP_WARN(get_logger(), "%s", result->message.c_str());
    goal_handle->abort(result);
    return;
  }

  const auto pre_execution_state =
    move_group_->getCurrentState(kMoveItInterfaceWaitSeconds);
  if (cancelRequested()) {
    finishMovePoseAsCanceled(
      goal_handle, planning_result.val,
      "MovePosePTP canceled during pre-execution state check");
    return;
  }
  if (!pre_execution_state) {
    result->result_code = MovePose::Result::NOT_READY;
    result->message = "MovePosePTP current state is unavailable before execution";
    RCLCPP_WARN(get_logger(), "%s", result->message.c_str());
    goal_handle->abort(result);
    return;
  }

  std::vector<double> pre_execution_joints;
  pre_execution_state->copyJointGroupPositions(
    joint_model_group, pre_execution_joints);
  if (pre_execution_joints.size() != captured_start_joints.size()) {
    result->result_code = MovePose::Result::INTERNAL_ERROR;
    result->moveit_error_code = moveit_msgs::msg::MoveItErrorCodes::FAILURE;
    result->message = "MovePosePTP pre-execution state has an unexpected joint count";
    RCLCPP_ERROR(get_logger(), "%s", result->message.c_str());
    goal_handle->abort(result);
    return;
  }
  if (!pre_execution_state->satisfiesBounds(joint_model_group)) {
    result->result_code = MovePose::Result::NOT_READY;
    result->moveit_error_code = moveit_msgs::msg::MoveItErrorCodes::INVALID_ROBOT_STATE;
    result->message = "MovePosePTP pre-execution state violates joint bounds";
    RCLCPP_WARN(get_logger(), "%s", result->message.c_str());
    goal_handle->abort(result);
    return;
  }

  double maximum_pre_execution_drift = 0.0;
  for (std::size_t index = 0; index < pre_execution_joints.size(); ++index) {
    if (!std::isfinite(pre_execution_joints[index])) {
      result->result_code = MovePose::Result::NOT_READY;
      result->moveit_error_code = moveit_msgs::msg::MoveItErrorCodes::INVALID_ROBOT_STATE;
      result->message = "MovePosePTP pre-execution state contains a non-finite value for " +
        std::string(kExpectedActiveJoints[index]);
      RCLCPP_WARN(get_logger(), "%s", result->message.c_str());
      goal_handle->abort(result);
      return;
    }
    const double drift =
      std::abs(pre_execution_joints[index] - captured_start_joints[index]);
    maximum_pre_execution_drift = std::max(maximum_pre_execution_drift, drift);
    if (drift > endpoint_tolerance) {
      std::ostringstream message;
      message << "MovePosePTP start state changed before execution for "
              << kExpectedActiveJoints[index] << ": drift=" << drift
              << " rad tolerance=" << endpoint_tolerance << " rad; replan required";
      result->result_code = MovePose::Result::NOT_READY;


      result->message = message.str();
      RCLCPP_WARN(get_logger(), "%s", result->message.c_str());
      goal_handle->abort(result);
      return;
    }
  }

  RCLCPP_INFO(
    get_logger(),
    "MovePosePTP pre-execution checks passed: supervisor=READY "
    "max_start_drift=%.9f rad",
    maximum_pre_execution_drift);

  bool stop_before_execution = false;
  bool cancel_before_execution = false;
  {
    std::lock_guard<std::mutex> lock(command_mutex_);
    if (cancel_requested_) {
      cancel_before_execution = true;
    } else if (shutting_down_ || !rclcpp::ok()) {
      stop_before_execution = true;
    } else {
      command_state_ = CommandState::kExecuting;
    }
  }
  if (stop_before_execution) {
    result->result_code = MovePose::Result::INTERNAL_ERROR;
    result->moveit_error_code = moveit_msgs::msg::MoveItErrorCodes::FAILURE;
    result->message = "MovePosePTP execution canceled because the node is shutting down";
    RCLCPP_WARN(get_logger(), "%s", result->message.c_str());
    goal_handle->abort(result);
    return;
  }
  if (cancel_before_execution) {
    finishMovePoseAsCanceled(
      goal_handle, planning_result.val,
      "MovePosePTP canceled before execution");
    return;
  }

  auto executing_feedback = std::make_shared<MovePose::Feedback>();
  executing_feedback->stage = "EXECUTING";
  goal_handle->publish_feedback(executing_feedback);

  if (cancelRequested()) {
    finishMovePoseAsCanceled(
      goal_handle, planning_result.val,
      "MovePosePTP canceled before execution");
    return;
  }

  RCLCPP_INFO(
    get_logger(),
    "MovePosePTP execution started: state=EXECUTING method=%s trajectory_points=%zu",
    ik_method.c_str(), trajectory_points);
  const auto execution_result = move_group_->execute(ptp_plan);
  result->moveit_error_code = execution_result.val;
  if (cancelRequested()) {
    finishMovePoseAsCanceled(
      goal_handle, execution_result.val,
      "MovePosePTP canceled during execution");
    return;
  }
  if (!static_cast<bool>(execution_result)) {
    requestCanFdHold("MovePosePTP execution failure");
    result->result_code = MovePose::Result::EXECUTION_FAILED;
    std::ostringstream message;
    message << "MovePosePTP execution failed with MoveIt error code "
            << execution_result.val;
    result->message = message.str();
    RCLCPP_WARN(get_logger(), "%s", result->message.c_str());
    goal_handle->abort(result);
    return;
  }

  result->result_code = MovePose::Result::SUCCESS;
  result->message = "MovePosePTP planning and execution succeeded";
  RCLCPP_INFO(get_logger(), "%s", result->message.c_str());
  goal_handle->succeed(result);
}

void MotionCommandNode::processMovePoseGoal(
  const std::shared_ptr<MovePoseGoalHandle> goal_handle)
{
  PlanningMetricsScope metrics(
    planning_metrics_publisher_, goal_handle->get_goal_id(), "move_pose",
    workspace_y_constraint_enabled_, workspace_min_tcp_y_m_,
    workspace_y_margin_m_, workspace_fk_sample_max_joint_step_rad_);
  metrics.setFailureStage("INPUT_VALIDATION");
  auto result = std::make_shared<MovePose::Result>();
  result->moveit_error_code = 0;

  auto validating_feedback = std::make_shared<MovePose::Feedback>();
  validating_feedback->stage = "VALIDATING";
  goal_handle->publish_feedback(validating_feedback);

  geometry_msgs::msg::PoseStamped normalized_target;
  double velocity_scaling = 0.0;
  double acceleration_scaling = 0.0;
  std::string reason;
  if (!validateMovePoseGoal(
      *goal_handle->get_goal(), normalized_target, velocity_scaling,
      acceleration_scaling, reason))
  {
    result->result_code = MovePose::Result::INVALID_GOAL;
    result->message = reason;
    RCLCPP_WARN(get_logger(), "MovePose validation failed: %s", reason.c_str());
    goal_handle->abort(result);
    return;
  }

  RCLCPP_INFO(
    get_logger(),
    "MovePose goal validated: velocity_scaling=%.3f acceleration_scaling=%.3f",
    velocity_scaling, acceleration_scaling);
  metrics.setFailureStage("SUPERVISOR");

  if (cancelRequested()) {
    finishMovePoseAsCanceled(goal_handle, 0, "MovePose canceled during validation");
    return;
  }

  const auto supervisor = querySupervisorReadiness();
  if (cancelRequested()) {
    finishMovePoseAsCanceled(goal_handle, 0, "MovePose canceled before planning");
    return;
  }
  if (supervisor.status != SupervisorCheckStatus::kReady) {
    result->result_code = MovePose::Result::NOT_READY;
    result->message = "MovePose blocked by supervisor: " + supervisor.message;
    RCLCPP_WARN(get_logger(), "%s", result->message.c_str());
    goal_handle->abort(result);
    return;
  }

  const auto current_state = move_group_->getCurrentState(kMoveItInterfaceWaitSeconds);
  if (cancelRequested()) {
    finishMovePoseAsCanceled(goal_handle, 0, "MovePose canceled before planning");
    return;
  }
  if (!current_state) {
    result->result_code = MovePose::Result::NOT_READY;
    result->message = "MovePose current robot state is unavailable";
    RCLCPP_WARN(get_logger(), "%s", result->message.c_str());
    goal_handle->abort(result);
    return;
  }

  const Eigen::Isometry3d current_base_transform =
    current_state->getGlobalLinkTransform(base_frame_);
  const Eigen::Isometry3d current_tool_transform =
    current_state->getGlobalLinkTransform(tool_frame_);
  const double current_tcp_y =
    (current_base_transform.inverse() * current_tool_transform).translation().y();
  if (workspace_y_constraint_enabled_ &&
    current_tcp_y < workspace_min_tcp_y_m_ + workspace_y_margin_m_)
  {
    std::ostringstream message;
    message << "MovePose start TCP y=" << current_tcp_y
            << " m is below the configured workspace boundary="
            << workspace_min_tcp_y_m_ + workspace_y_margin_m_ << " m";
    result->result_code = MovePose::Result::NOT_READY;
    result->message = message.str();
    metrics.setFailureStage("START_WORKSPACE", result->message);
    RCLCPP_WARN(get_logger(), "%s", result->message.c_str());
    goal_handle->abort(result);
    return;
  }
  metrics.setEligible();
  metrics.setFailureStage("IK");

  bool stop_before_planning = false;
  bool cancel_before_planning = false;
  {
    std::lock_guard<std::mutex> lock(command_mutex_);
    if (cancel_requested_) {
      cancel_before_planning = true;
    } else if (shutting_down_ || !rclcpp::ok()) {
      stop_before_planning = true;
    } else {
      command_state_ = CommandState::kPlanning;
    }
  }
  if (stop_before_planning) {
    result->result_code = MovePose::Result::INTERNAL_ERROR;
    result->message = "MovePose planning canceled because the node is shutting down";
    goal_handle->abort(result);
    return;
  }
  if (cancel_before_planning) {
    finishMovePoseAsCanceled(goal_handle, 0, "MovePose canceled before planning");
    return;
  }

  auto planning_feedback = std::make_shared<MovePose::Feedback>();
  planning_feedback->stage = "PLANNING";
  goal_handle->publish_feedback(planning_feedback);

  move_group_->setStartState(*current_state);
  move_group_->setMaxVelocityScalingFactor(velocity_scaling);
  move_group_->setMaxAccelerationScalingFactor(acceleration_scaling);

  // Compute IK once from the same captured state that will be used as the
  // planning start state. setJointValueTarget(PoseStamped) stores the single
  // IK solution as a joint-space goal instead of leaving a pose constraint for
  // the planner to solve again.
  if (!move_group_->setJointValueTarget(normalized_target, tool_frame_)) {
    result->result_code = MovePose::Result::PLANNING_FAILED;
    result->moveit_error_code = moveit_msgs::msg::MoveItErrorCodes::NO_IK_SOLUTION;
    result->message = "MovePose IK failed for the captured current-state seed";
    RCLCPP_WARN(get_logger(), "%s", result->message.c_str());
    goal_handle->abort(result);
    return;
  }

  const auto robot_model = move_group_->getRobotModel();
  const auto * joint_model_group =
    robot_model ? robot_model->getJointModelGroup(planning_group_) : nullptr;
  if (!joint_model_group) {
    result->result_code = MovePose::Result::INTERNAL_ERROR;
    result->message = "MovePose planning group is unavailable in the robot model";
    RCLCPP_ERROR(get_logger(), "%s", result->message.c_str());
    goal_handle->abort(result);
    return;
  }

  std::vector<double> start_joint_values;
  std::vector<double> ik_joint_target;
  current_state->copyJointGroupPositions(joint_model_group, start_joint_values);
  move_group_->getJointValueTarget(ik_joint_target);
  if (!validateJointDeltaLimit(
      start_joint_values, ik_joint_target, move_pose_max_joint_delta_rad_,
      "MovePose IK target", reason))
  {
    result->result_code = MovePose::Result::INVALID_GOAL;
    result->message = reason;
    RCLCPP_WARN(get_logger(), "%s", result->message.c_str());
    goal_handle->abort(result);
    return;
  }

  // Re-apply the extracted vector explicitly. From this point onward the
  // planner receives only this fixed six-axis joint target and cannot choose a
  // different IK branch for the original Cartesian pose.
  if (!move_group_->setJointValueTarget(ik_joint_target)) {
    result->result_code = MovePose::Result::INVALID_GOAL;
    result->message = "MoveIt rejected the fixed MovePose IK joint target";
    RCLCPP_WARN(get_logger(), "%s", result->message.c_str());
    goal_handle->abort(result);
    return;
  }
  metrics.setIkResult("direct", 0U);
  metrics.setFailureStage("OMPL");

  std::ostringstream ik_message;
  ik_message << "MovePose locked IK target:";
  for (std::size_t index = 0; index < ik_joint_target.size(); ++index) {
    ik_message << ' ' << kExpectedActiveJoints[index] << '=' << ik_joint_target[index]
               << "(delta=" << ik_joint_target[index] - start_joint_values[index] << ')';
  }
  RCLCPP_INFO(get_logger(), "%s", ik_message.str().c_str());

  MoveGroupInterface::Plan plan;
  RCLCPP_INFO(get_logger(), "MovePose planning started: state=PLANNING");
  const auto planning_result = move_group_->plan(plan);
  result->moveit_error_code = planning_result.val;
  if (cancelRequested()) {
    finishMovePoseAsCanceled(
      goal_handle, planning_result.val, "MovePose canceled during planning");
    return;
  }
  if (!static_cast<bool>(planning_result)) {
    result->result_code = MovePose::Result::PLANNING_FAILED;
    std::ostringstream message;
    message << "MovePose planning failed with MoveIt error code " << planning_result.val;
    result->message = message.str();
    RCLCPP_WARN(get_logger(), "%s", result->message.c_str());
    goal_handle->abort(result);
    return;
  }

  const auto trajectory_points = plan.trajectory_.joint_trajectory.points.size();
  metrics.setOmplResult(plan.planning_time_, trajectory_points);
  const auto & joint_trajectory = plan.trajectory_.joint_trajectory;
  metrics.setFailureStage("TRAJECTORY_VALIDATION");
  std::array<std::size_t, kExpectedActiveJoints.size()> trajectory_joint_indices{};
  bool trajectory_is_safe = !joint_trajectory.points.empty();
  if (!trajectory_is_safe) {
    reason = "MovePose planned trajectory contains no points";
  }

  for (std::size_t index = 0;
    trajectory_is_safe && index < kExpectedActiveJoints.size(); ++index)
  {
    const auto joint_iterator = std::find(
      joint_trajectory.joint_names.begin(), joint_trajectory.joint_names.end(),
      kExpectedActiveJoints[index]);
    if (joint_iterator == joint_trajectory.joint_names.end()) {
      reason = "MovePose planned trajectory is missing " +
        std::string(kExpectedActiveJoints[index]);
      trajectory_is_safe = false;
      break;
    }
    trajectory_joint_indices[index] = static_cast<std::size_t>(
      std::distance(joint_trajectory.joint_names.begin(), joint_iterator));
  }

  for (std::size_t point_index = 0;
    trajectory_is_safe && point_index < joint_trajectory.points.size(); ++point_index)
  {
    std::vector<double> waypoint(kExpectedActiveJoints.size());
    const auto & positions = joint_trajectory.points[point_index].positions;
    for (std::size_t joint_index = 0; joint_index < waypoint.size(); ++joint_index) {
      const auto trajectory_index = trajectory_joint_indices[joint_index];
      if (trajectory_index >= positions.size()) {
        reason = "MovePose planned trajectory point has incomplete joint positions";
        trajectory_is_safe = false;
        break;
      }
      waypoint[joint_index] = positions[trajectory_index];
    }

    if (trajectory_is_safe && !validateJointDeltaLimit(
        start_joint_values, waypoint, move_pose_max_joint_delta_rad_,
        "MovePose planned waypoint " + std::to_string(point_index), reason))
    {
      trajectory_is_safe = false;
    }
  }

  if (trajectory_is_safe) {
    const auto & final_positions = joint_trajectory.points.back().positions;
    for (std::size_t joint_index = 0;
      joint_index < kExpectedActiveJoints.size(); ++joint_index)
    {
      const double final_error = std::abs(
        final_positions[trajectory_joint_indices[joint_index]] - ik_joint_target[joint_index]);
      if (final_error > joint_tolerance_rad_ + 1.0e-6) {
        std::ostringstream message;
        message << "MovePose planned endpoint differs from locked IK target for "
                << kExpectedActiveJoints[joint_index] << ": error=" << final_error
                << " rad tolerance=" << joint_tolerance_rad_ << " rad";
        reason = message.str();
        trajectory_is_safe = false;
        break;
      }
    }
  }

  TrajectoryAnalysisResult trajectory_analysis;
  if (trajectory_is_safe) {
    trajectory_analysis = analyzeTrajectory(*current_state, joint_trajectory);
    metrics.setTrajectoryAnalysis(
      trajectory_analysis.maximum_adjacent_delta_rad,
      trajectory_analysis.minimum_tcp_y_m,
      trajectory_analysis.fk_samples_evaluated,
      trajectory_analysis.valid);
    if (!trajectory_analysis.valid) {
      reason = trajectory_analysis.reason;
      trajectory_is_safe = false;
    }
  }

  if (!trajectory_is_safe) {
    metrics.setFailureStage("TRAJECTORY_VALIDATION", reason);
    result->result_code = MovePose::Result::PLANNING_FAILED;
    result->moveit_error_code = moveit_msgs::msg::MoveItErrorCodes::INVALID_MOTION_PLAN;
    result->message = reason;
    RCLCPP_WARN(get_logger(), "%s", result->message.c_str());
    goal_handle->abort(result);
    return;
  }

  RCLCPP_INFO(
    get_logger(),
    "MovePose planning succeeded: planning_time=%.3f trajectory_points=%zu "
    "max_adjacent_delta=%.9f minimum_tcp_y=%.9f fk_samples=%u",
    plan.planning_time_, trajectory_points,
    trajectory_analysis.maximum_adjacent_delta_rad,
    trajectory_analysis.minimum_tcp_y_m,
    trajectory_analysis.fk_samples_evaluated);
  metrics.markPlanningSuccess();

  const auto pre_execution_supervisor = querySupervisorReadiness();
  if (cancelRequested()) {
    finishMovePoseAsCanceled(
      goal_handle, planning_result.val, "MovePose canceled before execution");
    return;
  }
  if (pre_execution_supervisor.status != SupervisorCheckStatus::kReady) {
    result->result_code = MovePose::Result::NOT_READY;
    result->message =
      "MovePose execution blocked by supervisor: " + pre_execution_supervisor.message;
    RCLCPP_WARN(get_logger(), "%s", result->message.c_str());
    goal_handle->abort(result);
    return;
  }

  bool stop_before_execution = false;
  bool cancel_before_execution = false;
  {
    std::lock_guard<std::mutex> lock(command_mutex_);
    if (cancel_requested_) {
      cancel_before_execution = true;
    } else if (shutting_down_ || !rclcpp::ok()) {
      stop_before_execution = true;
    } else {
      command_state_ = CommandState::kExecuting;
    }
  }
  if (stop_before_execution) {
    result->result_code = MovePose::Result::INTERNAL_ERROR;
    result->message = "MovePose execution canceled because the node is shutting down";
    goal_handle->abort(result);
    return;
  }
  if (cancel_before_execution) {
    finishMovePoseAsCanceled(
      goal_handle, planning_result.val, "MovePose canceled before execution");
    return;
  }

  auto executing_feedback = std::make_shared<MovePose::Feedback>();
  executing_feedback->stage = "EXECUTING";
  goal_handle->publish_feedback(executing_feedback);

  if (cancelRequested()) {
    finishMovePoseAsCanceled(
      goal_handle, planning_result.val, "MovePose canceled before execution");
    return;
  }

  RCLCPP_INFO(get_logger(), "MovePose execution started: state=EXECUTING");
  const auto execution_result = move_group_->execute(plan);
  result->moveit_error_code = execution_result.val;
  if (cancelRequested()) {
    finishMovePoseAsCanceled(
      goal_handle, execution_result.val, "MovePose canceled during execution");
    return;
  }
  if (!static_cast<bool>(execution_result)) {
    requestCanFdHold("MovePose execution failure");
    result->result_code = MovePose::Result::EXECUTION_FAILED;
    std::ostringstream message;
    message << "MovePose execution failed with MoveIt error code " << execution_result.val;
    result->message = message.str();
    RCLCPP_WARN(get_logger(), "%s", result->message.c_str());
    goal_handle->abort(result);
    return;
  }

  result->result_code = MovePose::Result::SUCCESS;
  result->message = "MovePose planning and execution succeeded";
  RCLCPP_INFO(get_logger(), "%s", result->message.c_str());
  goal_handle->succeed(result);
}

bool MotionCommandNode::cancelRequested()
{
  std::lock_guard<std::mutex> lock(command_mutex_);
  return cancel_requested_;
}

void MotionCommandNode::finishMoveJAsCanceled(
  const std::shared_ptr<MoveJGoalHandle> goal_handle,
  std::int32_t moveit_error_code,
  const std::string & message)
{
  {
    std::lock_guard<std::mutex> lock(command_mutex_);
    if (command_state_ != CommandState::kIdle) {
      command_state_ = CommandState::kStopping;
    }
  }

  auto stopping_feedback = std::make_shared<MoveJ::Feedback>();
  stopping_feedback->stage = "STOPPING";
  goal_handle->publish_feedback(stopping_feedback);

  const auto cancel_state_deadline =
    std::chrono::steady_clock::now() + std::chrono::milliseconds(100);
  while (!goal_handle->is_canceling() && rclcpp::ok() &&
    std::chrono::steady_clock::now() < cancel_state_deadline)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

  auto result = std::make_shared<MoveJ::Result>();
  result->moveit_error_code = moveit_error_code;
  if (!goal_handle->is_canceling()) {
    result->result_code = MoveJ::Result::INTERNAL_ERROR;
    result->message = "MoveJ cancel state did not become active";
    RCLCPP_ERROR(get_logger(), "%s", result->message.c_str());
    goal_handle->abort(result);
    return;
  }

  result->result_code = MoveJ::Result::CANCELED;
  result->message = message;
  RCLCPP_INFO(get_logger(), "%s", message.c_str());
  goal_handle->canceled(result);
}

void MotionCommandNode::finishMovePoseAsCanceled(
  const std::shared_ptr<MovePoseGoalHandle> goal_handle,
  std::int32_t moveit_error_code,
  const std::string & message)
{
  {
    std::lock_guard<std::mutex> lock(command_mutex_);
    if (command_state_ != CommandState::kIdle) {
      command_state_ = CommandState::kStopping;
    }
  }

  auto stopping_feedback = std::make_shared<MovePose::Feedback>();
  stopping_feedback->stage = "STOPPING";
  goal_handle->publish_feedback(stopping_feedback);

  const auto cancel_state_deadline =
    std::chrono::steady_clock::now() + std::chrono::milliseconds(100);
  while (!goal_handle->is_canceling() && rclcpp::ok() &&
    std::chrono::steady_clock::now() < cancel_state_deadline)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

  auto result = std::make_shared<MovePose::Result>();
  result->moveit_error_code = moveit_error_code;
  if (!goal_handle->is_canceling()) {
    result->result_code = MovePose::Result::INTERNAL_ERROR;
    result->message = "MovePose cancel state did not become active";
    RCLCPP_ERROR(get_logger(), "%s", result->message.c_str());
    goal_handle->abort(result);
    return;
  }

  result->result_code = MovePose::Result::CANCELED;
  result->message = message;
  RCLCPP_INFO(get_logger(), "%s", message.c_str());
  goal_handle->canceled(result);
}

bool MotionCommandNode::validateMoveJGoal(
  const MoveJ::Goal & goal,
  double & velocity_scaling,
  double & acceleration_scaling,
  std::string & reason) const
{
  for (std::size_t index = 0; index < goal.joint_positions.size(); ++index) {
    if (!std::isfinite(goal.joint_positions[index])) {
      reason = std::string(kExpectedActiveJoints[index]) + " position must be finite";
      return false;
    }
  }

  if (!resolveScaling(
      goal.velocity_scaling, default_velocity_scaling_, "velocity_scaling",
      velocity_scaling, reason))
  {
    return false;
  }
  if (!resolveScaling(
      goal.acceleration_scaling, default_acceleration_scaling_,
      "acceleration_scaling", acceleration_scaling, reason))
  {
    return false;
  }
  const auto robot_model = move_group_->getRobotModel();
  for (std::size_t index = 0; index < kExpectedActiveJoints.size(); ++index) {
    const auto * joint_model = robot_model->getJointModel(kExpectedActiveJoints[index]);
    const double position = goal.joint_positions[index];
    if (!joint_model->satisfiesPositionBounds(&position)) {
      std::ostringstream message;
      message << kExpectedActiveJoints[index] << " target " << position <<
        " rad violates the MoveIt robot-model position bounds";
      reason = message.str();
      return false;
    }
  }

  reason.clear();
  return true;
}

bool MotionCommandNode::validateMovePoseGoal(
  const MovePose::Goal & goal,
  geometry_msgs::msg::PoseStamped & normalized_target,
  double & velocity_scaling,
  double & acceleration_scaling,
  std::string & reason) const
{
  if (goal.target_pose.header.frame_id != base_frame_) {
    reason = "target_pose.header.frame_id must be " + base_frame_;
    return false;
  }

  const auto & position = goal.target_pose.pose.position;
  if (!std::isfinite(position.x) || !std::isfinite(position.y) ||
    !std::isfinite(position.z))
  {
    reason = "target_pose position components must be finite";
    return false;
  }

  const auto & orientation = goal.target_pose.pose.orientation;
  if (!std::isfinite(orientation.x) || !std::isfinite(orientation.y) ||
    !std::isfinite(orientation.z) || !std::isfinite(orientation.w))
  {
    reason = "target_pose orientation components must be finite";
    return false;
  }

  const double orientation_norm = std::sqrt(
    orientation.x * orientation.x + orientation.y * orientation.y +
    orientation.z * orientation.z + orientation.w * orientation.w);
  if (!std::isfinite(orientation_norm) || orientation_norm <= 1e-6) {
    reason = "target_pose quaternion norm must be greater than 1e-6";
    return false;
  }

  if (!resolveScaling(
      goal.velocity_scaling, default_velocity_scaling_, "velocity_scaling",
      velocity_scaling, reason))
  {
    return false;
  }
  if (!resolveScaling(
      goal.acceleration_scaling, default_acceleration_scaling_,
      "acceleration_scaling", acceleration_scaling, reason))
  {
    return false;
  }
  if (workspace_y_constraint_enabled_) {
    const double effective_minimum_y =
      workspace_min_tcp_y_m_ + workspace_y_margin_m_;
    if (position.y < effective_minimum_y) {
      std::ostringstream message;
      message << "target_pose TCP y=" << position.y
              << " m is below the configured safe boundary="
              << effective_minimum_y << " m in " << base_frame_;
      reason = message.str();
      return false;
    }
  }

  normalized_target = goal.target_pose;
  normalized_target.pose.orientation.x /= orientation_norm;
  normalized_target.pose.orientation.y /= orientation_norm;
  normalized_target.pose.orientation.z /= orientation_norm;
  normalized_target.pose.orientation.w /= orientation_norm;
  reason.clear();
  return true;
}

MotionCommandNode::TrajectoryAnalysisResult
MotionCommandNode::analyzeTrajectory(
  const moveit::core::RobotState & reference_state,
  const trajectory_msgs::msg::JointTrajectory & trajectory) const
{
  TrajectoryAnalysisResult result;
  if (trajectory.points.empty()) {
    result.reason = "planned trajectory contains no points";
    return result;
  }

  std::array<std::size_t, kExpectedActiveJoints.size()> joint_indices{};
  for (std::size_t index = 0; index < kExpectedActiveJoints.size(); ++index) {
    const auto iterator = std::find(
      trajectory.joint_names.begin(), trajectory.joint_names.end(),
      kExpectedActiveJoints[index]);
    if (iterator == trajectory.joint_names.end()) {
      result.reason =
        "planned trajectory is missing " + std::string(kExpectedActiveJoints[index]);
      return result;
    }
    if (std::count(
        trajectory.joint_names.begin(), trajectory.joint_names.end(),
        kExpectedActiveJoints[index]) != 1)
    {
      result.reason =
        "planned trajectory contains duplicate joint " +
        std::string(kExpectedActiveJoints[index]);
      return result;
    }
    joint_indices[index] = static_cast<std::size_t>(
      std::distance(trajectory.joint_names.begin(), iterator));
  }

  moveit::core::RobotState sample_state(reference_state);
  const double effective_minimum_y =
    workspace_min_tcp_y_m_ + workspace_y_margin_m_;
  std::size_t samples_evaluated = 0U;
  double minimum_tcp_y = std::numeric_limits<double>::infinity();

  auto evaluate_tcp_y =
    [this, &sample_state, &samples_evaluated, &minimum_tcp_y,
      effective_minimum_y, &result](const std::vector<double> & joints) -> bool
    {
      if (samples_evaluated >= kMaximumWorkspaceFkSamples) {
        result.reason =
          "workspace FK sampling exceeded the hard limit of " +
          std::to_string(kMaximumWorkspaceFkSamples) + " samples";
        return false;
      }
      for (std::size_t index = 0; index < joints.size(); ++index) {
        sample_state.setVariablePosition(kExpectedActiveJoints[index], joints[index]);
      }
      sample_state.update();
      try {
        const Eigen::Isometry3d model_to_base =
          sample_state.getGlobalLinkTransform(base_frame_);
        const Eigen::Isometry3d model_to_tool =
          sample_state.getGlobalLinkTransform(tool_frame_);
        const double tcp_y =
          (model_to_base.inverse() * model_to_tool).translation().y();
        ++samples_evaluated;
        if (!std::isfinite(tcp_y)) {
          result.reason = "workspace FK produced a non-finite TCP y value";
          return false;
        }
        minimum_tcp_y = std::min(minimum_tcp_y, tcp_y);
        if (tcp_y < effective_minimum_y) {
          std::ostringstream message;
          message << "planned TCP path violates workspace Y boundary: y="
                  << tcp_y << " m boundary=" << effective_minimum_y
                  << " m in " << base_frame_;
          result.reason = message.str();
          return false;
        }
      } catch (const std::exception & exception) {
        result.reason =
          std::string("workspace FK evaluation failed: ") + exception.what();
        return false;
      }
      return true;
    };

  std::vector<double> previous_waypoint;
  previous_waypoint.reserve(kExpectedActiveJoints.size());
  for (const auto * joint_name : kExpectedActiveJoints) {
    const double reference_position =
      reference_state.getVariablePosition(joint_name);
    if (!std::isfinite(reference_position)) {
      result.reason =
        "reference state contains a non-finite position for " +
        std::string(joint_name);
      return result;
    }
    previous_waypoint.push_back(reference_position);
  }
  if (workspace_y_constraint_enabled_ && !evaluate_tcp_y(previous_waypoint)) {
    result.fk_samples_evaluated = static_cast<std::uint32_t>(samples_evaluated);
    if (samples_evaluated > 0U) {
      result.minimum_tcp_y_m = minimum_tcp_y;
    }
    return result;
  }

  for (std::size_t point_index = 0;
    point_index < trajectory.points.size(); ++point_index)
  {
    const auto & positions = trajectory.points[point_index].positions;
    std::vector<double> waypoint(kExpectedActiveJoints.size());
    for (std::size_t joint_index = 0; joint_index < waypoint.size(); ++joint_index) {
      const auto trajectory_index = joint_indices[joint_index];
      if (trajectory_index >= positions.size()) {
        result.reason =
          "planned trajectory point " + std::to_string(point_index) +
          " has incomplete joint positions";
        return result;
      }
      waypoint[joint_index] = positions[trajectory_index];
      if (!std::isfinite(waypoint[joint_index])) {
        result.reason =
          "planned trajectory point " + std::to_string(point_index) +
          " contains a non-finite joint position";
        return result;
      }
    }

    double segment_maximum_delta = 0.0;
    for (std::size_t joint_index = 0; joint_index < waypoint.size(); ++joint_index) {
      segment_maximum_delta = std::max(
        segment_maximum_delta,
        std::abs(waypoint[joint_index] - previous_waypoint[joint_index]));
    }
    result.maximum_adjacent_delta_rad = std::max(
      result.maximum_adjacent_delta_rad, segment_maximum_delta);

    if (workspace_y_constraint_enabled_) {
      const auto interpolation_steps = std::max<std::size_t>(
        1U, static_cast<std::size_t>(
          std::ceil(
            segment_maximum_delta /
            workspace_fk_sample_max_joint_step_rad_)));
      if (samples_evaluated + interpolation_steps > kMaximumWorkspaceFkSamples) {
        result.reason =
          "workspace FK sampling would exceed the hard limit of " +
          std::to_string(kMaximumWorkspaceFkSamples) + " samples";
        result.fk_samples_evaluated = static_cast<std::uint32_t>(samples_evaluated);
        if (samples_evaluated > 0U) {
          result.minimum_tcp_y_m = minimum_tcp_y;
        }
        return result;
      }
      std::vector<double> interpolated(waypoint.size());
      for (std::size_t step = 1U; step <= interpolation_steps; ++step) {
        const double alpha =
          static_cast<double>(step) / static_cast<double>(interpolation_steps);
        for (std::size_t joint_index = 0; joint_index < waypoint.size(); ++joint_index) {
          interpolated[joint_index] =
            previous_waypoint[joint_index] +
            alpha * (waypoint[joint_index] - previous_waypoint[joint_index]);
        }
        if (!evaluate_tcp_y(interpolated)) {
          result.fk_samples_evaluated = static_cast<std::uint32_t>(samples_evaluated);
          if (samples_evaluated > 0U) {
            result.minimum_tcp_y_m = minimum_tcp_y;
          }
          return result;
        }
      }
    }
    previous_waypoint = std::move(waypoint);
  }

  result.valid = true;
  result.fk_samples_evaluated = static_cast<std::uint32_t>(samples_evaluated);
  if (samples_evaluated > 0U) {
    result.minimum_tcp_y_m = minimum_tcp_y;
  }
  result.reason.clear();
  return result;
}

}  // namespace elfin3_motion_command
