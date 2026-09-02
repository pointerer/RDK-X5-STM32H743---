#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <exception>
#include <iomanip>
#include <iterator>
#include <iostream>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_set>

#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "controller_manager_msgs/srv/list_controllers.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "moveit_msgs/srv/get_motion_plan.hpp"
#include "moveit_msgs/srv/get_position_ik.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rosgraph_msgs/msg/clock.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/exceptions.h"
#include "tf2/time.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace elfin3_supervisor
{

class Elfin3SupervisorNode final : public rclcpp::Node
{
public:
  Elfin3SupervisorNode()
  : rclcpp::Node("elfin3_supervisor")
  {
    node_start_time_ = std::chrono::steady_clock::now();

    const double display_rate_hz = declare_parameter<double>("display_rate_hz", 2.0);
    const double diagnostics_rate_hz =
      declare_parameter<double>("diagnostics_rate_hz", 1.0);
    const double joint_state_timeout_sec =
      declare_parameter<double>("joint_state_timeout_sec", 0.5);
    const double tf_timeout_sec = declare_parameter<double>("tf_timeout_sec", 0.5);
    const double controller_poll_rate_hz =
      declare_parameter<double>("controller_poll_rate_hz", 1.0);
    const double controller_response_timeout_sec =
      declare_parameter<double>("controller_response_timeout_sec", 0.5);
    const double controller_state_timeout_sec =
      declare_parameter<double>("controller_state_timeout_sec", 2.5);
    const double clock_timeout_sec = declare_parameter<double>("clock_timeout_sec", 1.0);
    const double startup_grace_period_sec =
      declare_parameter<double>("startup_grace_period_sec", 15.0);
    const double hardware_control_loop_timeout_sec =
      declare_parameter<double>("hardware_control_loop_timeout_sec", 3.0);
    const double motion_window_sec = declare_parameter<double>("motion_window_sec", 0.2);
    motion_position_threshold_rad_ =
      declare_parameter<double>("motion_position_threshold_rad", 0.001);
    terminal_output_ = declare_parameter<bool>("terminal_output", true);
    hardware_control_loop_required_ =
      declare_parameter<bool>("hardware_control_loop_required", true);
    base_frame_ = declare_parameter<std::string>("base_frame", "elfin_base");
    tool_frame_ = declare_parameter<std::string>("tool_frame", "elfin_end_link");

    requirePositiveFinite(display_rate_hz, "display_rate_hz");
    requirePositiveFinite(diagnostics_rate_hz, "diagnostics_rate_hz");
    requirePositiveFinite(joint_state_timeout_sec, "joint_state_timeout_sec");
    requirePositiveFinite(tf_timeout_sec, "tf_timeout_sec");
    requirePositiveFinite(controller_poll_rate_hz, "controller_poll_rate_hz");
    requirePositiveFinite(controller_response_timeout_sec, "controller_response_timeout_sec");
    requirePositiveFinite(controller_state_timeout_sec, "controller_state_timeout_sec");
    requirePositiveFinite(clock_timeout_sec, "clock_timeout_sec");
    requirePositiveFinite(startup_grace_period_sec, "startup_grace_period_sec");
    requirePositiveFinite(
      hardware_control_loop_timeout_sec, "hardware_control_loop_timeout_sec");
    requirePositiveFinite(motion_window_sec, "motion_window_sec");
    requirePositiveFinite(
      motion_position_threshold_rad_, "motion_position_threshold_rad");
    if (base_frame_.empty() || tool_frame_.empty()) {
      throw std::invalid_argument("base_frame and tool_frame must not be empty");
    }

    joint_state_timeout_ = std::chrono::duration<double>(joint_state_timeout_sec);
    tf_timeout_ = std::chrono::duration<double>(tf_timeout_sec);
    controller_response_timeout_ =
      std::chrono::duration<double>(controller_response_timeout_sec);
    controller_state_timeout_ = std::chrono::duration<double>(controller_state_timeout_sec);
    clock_timeout_ = std::chrono::duration<double>(clock_timeout_sec);
    startup_grace_period_ = std::chrono::duration<double>(startup_grace_period_sec);
    hardware_control_loop_timeout_ =
      std::chrono::duration<double>(hardware_control_loop_timeout_sec);
    motion_window_ = std::chrono::duration<double>(motion_window_sec);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    controller_client_ = create_client<ListControllers>(
      "/controller_manager/list_controllers");
    compute_ik_client_ = create_client<moveit_msgs::srv::GetPositionIK>("/compute_ik");
    motion_plan_client_ =
      create_client<moveit_msgs::srv::GetMotionPlan>("/plan_kinematic_path");
    trajectory_action_client_ = rclcpp_action::create_client<FollowJointTrajectory>(
      this, "/elfin_arm_controller/follow_joint_trajectory");

    diagnostics_publisher_ =
      create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 10);
    /*
     * 注册 Supervisor 就绪查询服务；私有名称 ~/is_ready 展开为
     * /elfin3_supervisor/is_ready，回调汇总运行状态并仅在 READY 时返回成功。
     */
    is_ready_service_ = create_service<std_srvs::srv::Trigger>(
      "~/is_ready",
      [this](
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
      {
        handleIsReady(*response);
      });

    clock_subscription_ = create_subscription<rosgraph_msgs::msg::Clock>(
      "/clock",
      rclcpp::ClockQoS(),
      [this](rosgraph_msgs::msg::Clock::ConstSharedPtr message) {
        handleClock(*message);
      });

    joint_state_subscription_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states",
      rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::JointState::ConstSharedPtr message) {
        handleJointState(*message);
      });

    hardware_diagnostics_subscription_ =
      create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
      "/diagnostics", rclcpp::QoS(10).reliable(),
      [this](diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr message) {
        handleHardwareDiagnostics(*message);
      });

    control_mode_subscription_ = create_subscription<std_msgs::msg::String>(
      "/elfin3/control_mode",
      rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local(),
      [this](std_msgs::msg::String::ConstSharedPtr message) {
        control_mode_ = message->data;
      });

    const auto display_period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / display_rate_hz));
    display_timer_ = create_wall_timer(
      display_period,
      [this]() {
        updateToolPose();
        if (terminal_output_) {
          renderTerminal();
        }
      });

    const auto controller_poll_period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / controller_poll_rate_hz));
    controller_poll_timer_ = create_wall_timer(
      controller_poll_period, [this]() {pollControllers();});

    const auto diagnostics_period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / diagnostics_rate_hz));
    diagnostics_timer_ = create_wall_timer(
      diagnostics_period, [this]() {publishDiagnostics();});

    RCLCPP_INFO(
      get_logger(), "Monitoring /joint_states at %.1f Hz", display_rate_hz);
  }

private:
  using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
  using ListControllers = controller_manager_msgs::srv::ListControllers;

  enum class JointStateHealth
  {
    kNoData,
    kValid,
    kIncomplete,
    kInvalid,
  };

  enum class SystemState
  {
    kStarting,
    kNotReady,
    kReady,
    kMoving,
    kFault,
  };

  struct JointPositionSample
  {
    std::chrono::steady_clock::time_point receive_time;
    std::array<double, 6> positions;
  };

  struct RuntimeChecks
  {
    bool hardware_control_loop_ready{false};
    bool hardware_control_loop_required{true};
    bool joint_state_ready{false};
    bool transform_ready{false};
    bool controllers_ready{false};
    bool compute_ik_ready{false};
    bool motion_plan_ready{false};
    bool trajectory_action_ready{false};
    bool clock_ready{false};
    bool use_sim_time{false};
    bool moveit_required{true};
  };

  struct SystemEvaluation
  {
    SystemState state{SystemState::kStarting};
    std::string reason;
    bool motion_detected{false};
  };

  static constexpr std::size_t kJointCount = 6;
  static constexpr std::array<const char *, kJointCount> kJointNames = {
    "elfin_joint1",
    "elfin_joint2",
    "elfin_joint3",
    "elfin_joint4",
    "elfin_joint5",
    "elfin_joint6",
  };

  static void requirePositiveFinite(double value, const char * parameter_name)
  {
    if (!std::isfinite(value) || value <= 0.0) {
      const std::string message =
        std::string(parameter_name) + " must be finite and greater than zero";
      throw std::invalid_argument(message);
    }
  }

  void handleJointState(const sensor_msgs::msg::JointState & message)
  {
    if (message.name.size() != message.position.size()) {
      joint_state_health_ = JointStateHealth::kInvalid;
      joint_state_issue_ = "name and position arrays have different sizes";
      RCLCPP_ERROR_ONCE(
        get_logger(),
        "Rejected JointState: name size (%zu) differs from position size (%zu)",
        message.name.size(), message.position.size());
      return;
    }

    std::unordered_set<std::string> unique_names;
    unique_names.reserve(message.name.size());

    for (std::size_t index = 0; index < message.name.size(); ++index) {
      if (!unique_names.insert(message.name[index]).second) {
        joint_state_health_ = JointStateHealth::kInvalid;
        joint_state_issue_ = "duplicate joint name: " + message.name[index];
        RCLCPP_ERROR_ONCE(
          get_logger(), "Rejected JointState: duplicate joint name '%s'",
          message.name[index].c_str());
        return;
      }
    }

    std::array<double, kJointCount> ordered_positions{};
    for (std::size_t joint_index = 0; joint_index < kJointCount; ++joint_index) {
      const auto position = std::find(
        message.name.begin(), message.name.end(), kJointNames[joint_index]);

      if (position == message.name.end()) {
        joint_state_health_ = JointStateHealth::kIncomplete;
        joint_state_issue_ = "missing expected joint: " + std::string(kJointNames[joint_index]);
        RCLCPP_WARN_ONCE(
          get_logger(), "Incomplete JointState: missing expected joint '%s'",
          kJointNames[joint_index]);
        return;
      }

      const auto message_index = static_cast<std::size_t>(
        std::distance(message.name.begin(), position));
      const double joint_position = message.position[message_index];
      if (!std::isfinite(joint_position)) {
        joint_state_health_ = JointStateHealth::kInvalid;
        joint_state_issue_ = "non-finite position: " + std::string(kJointNames[joint_index]);
        RCLCPP_ERROR_ONCE(
          get_logger(), "Rejected JointState: non-finite position for '%s'",
          kJointNames[joint_index]);
        return;
      }

      ordered_positions[joint_index] = joint_position;
    }

    const auto receive_time = std::chrono::steady_clock::now();
    latest_joint_positions_ = ordered_positions;
    last_valid_joint_state_time_ = receive_time;
    joint_position_history_.push_back({receive_time, ordered_positions});
    pruneMotionHistory(receive_time);
    joint_state_health_ = JointStateHealth::kValid;
    joint_state_issue_.clear();

    if (!has_valid_joint_state_) {
      has_valid_joint_state_ = true;
      RCLCPP_INFO(
        get_logger(),
        "First complete ordered joint state J1..J6 [rad]: "
        "J1=%.6f J2=%.6f J3=%.6f J4=%.6f J5=%.6f J6=%.6f",
        latest_joint_positions_[0], latest_joint_positions_[1], latest_joint_positions_[2],
        latest_joint_positions_[3], latest_joint_positions_[4], latest_joint_positions_[5]);
    }
  }

  void handleClock(const rosgraph_msgs::msg::Clock & message)
  {
    const std::int64_t clock_time_ns =
      static_cast<std::int64_t>(message.clock.sec) * 1000000000LL +
      static_cast<std::int64_t>(message.clock.nanosec);
    const auto now = std::chrono::steady_clock::now();

    if (!has_clock_sample_) {
      has_clock_sample_ = true;
      clock_has_advanced_ = false;
      last_clock_time_ns_ = clock_time_ns;
      last_clock_progress_time_ = now;
      return;
    }

    if (clock_time_ns > last_clock_time_ns_) {
      clock_has_advanced_ = true;
      last_clock_progress_time_ = now;
    } else if (clock_time_ns < last_clock_time_ns_) {
      clock_has_advanced_ = false;
      last_clock_progress_time_ = now;
    }

    last_clock_time_ns_ = clock_time_ns;
  }

  void handleHardwareDiagnostics(const diagnostic_msgs::msg::DiagnosticArray & message)
  {
    for (const auto & status : message.status) {
      if (status.name != "elfin3_canfd_system/communication") {
        continue;
      }

      const auto now = std::chrono::steady_clock::now();
      hardware_diagnostic_received_ = true;
      last_hardware_diagnostic_time_ = now;

      for (const auto & value : status.values) {
        if (value.key != "hardware_write_cycles") {
          continue;
        }

        try {
          std::size_t parsed_characters = 0;
          if (value.value.empty() || value.value.front() == '-') {
            throw std::invalid_argument("negative or empty cycle count");
          }
          const auto write_cycles = static_cast<std::uint64_t>(
            std::stoull(value.value, &parsed_characters, 10));
          if (parsed_characters != value.value.size()) {
            throw std::invalid_argument("trailing characters in cycle count");
          }

          if (!hardware_write_counter_initialized_) {
            hardware_write_counter_initialized_ = true;
            hardware_write_cycles_ = write_cycles;
            last_hardware_write_progress_time_ = now;
          } else if (write_cycles != hardware_write_cycles_) {
            hardware_write_cycles_ = write_cycles;
            hardware_write_progress_observed_ = true;
            last_hardware_write_progress_time_ = now;
          }
          hardware_write_cycles_valid_ = true;
        } catch (const std::exception &) {
          hardware_write_cycles_valid_ = false;
        }
        return;
      }

      hardware_write_cycles_valid_ = false;
      return;
    }
  }

  void pollControllers()
  {
    const auto now = std::chrono::steady_clock::now();

    if (controller_request_pending_) {
      if ((now - controller_request_sent_time_) <= controller_response_timeout_) {
        return;
      }

      if (controller_request_id_ >= 0) {
        controller_client_->remove_pending_request(controller_request_id_);
      }
      controller_request_id_ = -1;
      controller_request_pending_ = false;
      controllers_response_received_ = false;
      ++controller_request_generation_;
    }

    if (!controller_client_->service_is_ready()) {
      controllers_response_received_ = false;
      joint_state_broadcaster_active_ = false;
      arm_controller_active_ = false;
      jog_controller_active_ = false;
      return;
    }

    const auto request = std::make_shared<ListControllers::Request>();
    const std::uint64_t request_generation = ++controller_request_generation_;
    controller_request_sent_time_ = now;

    try {
      const auto pending_request = controller_client_->async_send_request(
        request,
        [this, request_generation](rclcpp::Client<ListControllers>::SharedFuture future) {
          if (request_generation != controller_request_generation_) {
            return;
          }

          controller_request_pending_ = false;
          controller_request_id_ = -1;

          try {
            const auto response = future.get();
            joint_state_broadcaster_active_ = false;
            arm_controller_active_ = false;
            jog_controller_active_ = false;

            for (const auto & controller : response->controller) {
              if (controller.name == "joint_state_broadcaster") {
                joint_state_broadcaster_active_ = controller.state == "active";
              } else if (controller.name == "elfin_arm_controller") {
                arm_controller_active_ = controller.state == "active";
              } else if (controller.name == "elfin_jog_controller") {
                jog_controller_active_ = controller.state == "active";
              }
            }

            controllers_response_received_ = true;
            last_controller_response_time_ = std::chrono::steady_clock::now();
          } catch (const std::exception & exception) {
            controllers_response_received_ = false;
            joint_state_broadcaster_active_ = false;
            arm_controller_active_ = false;
            jog_controller_active_ = false;
            RCLCPP_ERROR(
              get_logger(), "ListControllers request failed: %s", exception.what());
          }
        });
      controller_request_id_ = pending_request.request_id;
      controller_request_pending_ = true;
    } catch (const std::exception & exception) {
      controllers_response_received_ = false;
      controller_request_pending_ = false;
      controller_request_id_ = -1;
      RCLCPP_ERROR(
        get_logger(), "Failed to send ListControllers request: %s", exception.what());
    }
  }

  void updateToolPose()
  {
    try {
      const auto transform = tf_buffer_->lookupTransform(
        base_frame_, tool_frame_, tf2::TimePointZero);
      const auto & translation = transform.transform.translation;
      const auto & rotation = transform.transform.rotation;

      const bool transform_is_finite =
        std::isfinite(translation.x) && std::isfinite(translation.y) &&
        std::isfinite(translation.z) && std::isfinite(rotation.x) &&
        std::isfinite(rotation.y) && std::isfinite(rotation.z) &&
        std::isfinite(rotation.w);
      const double quaternion_norm_squared =
        rotation.x * rotation.x + rotation.y * rotation.y +
        rotation.z * rotation.z + rotation.w * rotation.w;

      if (!transform_is_finite || quaternion_norm_squared <= 1.0e-12) {
        tf_available_ = true;
        tf_values_valid_ = false;
        tf_issue_ = "transform contains invalid values";
        return;
      }

      const std::int64_t transform_stamp_ns =
        static_cast<std::int64_t>(transform.header.stamp.sec) * 1000000000LL +
        static_cast<std::int64_t>(transform.header.stamp.nanosec);
      const auto now = std::chrono::steady_clock::now();

      if (!tf_available_) {
        last_tf_stamp_ns_ = transform_stamp_ns;
        last_tf_update_time_ = now;
        tf_time_reset_ = false;
      } else if (transform_stamp_ns > last_tf_stamp_ns_) {
        last_tf_stamp_ns_ = transform_stamp_ns;
        last_tf_update_time_ = now;
        tf_time_reset_ = false;
      } else if (transform_stamp_ns < last_tf_stamp_ns_) {
        last_tf_stamp_ns_ = transform_stamp_ns;
        last_tf_update_time_ = now;
        tf_time_reset_ = true;
      }

      tf_available_ = true;
      tf_values_valid_ = true;

      tf2::Quaternion quaternion(rotation.x, rotation.y, rotation.z, rotation.w);
      quaternion.normalize();

      double roll = 0.0;
      double pitch = 0.0;
      double yaw = 0.0;
      tf2::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);
      latest_tool_position_ = {translation.x, translation.y, translation.z};
      latest_tool_rpy_ = {roll, pitch, yaw};
      tf_issue_.clear();
    } catch (const tf2::TransformException & exception) {
      tf_available_ = false;
      tf_values_valid_ = false;
      tf_issue_ = exception.what();
    }
  }

  bool jointStateReady(const std::chrono::steady_clock::time_point & now) const
  {
    return joint_state_health_ == JointStateHealth::kValid &&
           (now - last_valid_joint_state_time_) <= joint_state_timeout_;
  }

  bool transformReady(const std::chrono::steady_clock::time_point & now) const
  {
    return tf_available_ && tf_values_valid_ && !tf_time_reset_ &&
           (now - last_tf_update_time_) <= tf_timeout_;
  }

  bool controllersReady(const std::chrono::steady_clock::time_point & now) const
  {
    if (!controller_client_->service_is_ready() || !controllers_response_received_ ||
      !joint_state_broadcaster_active_)
    {
      return false;
    }

    if ((now - last_controller_response_time_) > controller_state_timeout_) {
      return false;
    }

    if (controller_request_pending_ &&
      (now - controller_request_sent_time_) > controller_response_timeout_)
    {
      return false;
    }

    if (control_mode_ == "MOVEIT") {
      return arm_controller_active_ && !jog_controller_active_;
    }
    if (control_mode_ == "JOG_IDLE" || control_mode_ == "JOG_ACTIVE") {
      return jog_controller_active_ && !arm_controller_active_;
    }
    return false;
  }

  bool hardwareControlLoopReady(const std::chrono::steady_clock::time_point & now) const
  {
    if (!hardware_control_loop_required_) {
      return true;
    }
    return hardware_diagnostic_received_ && hardware_write_cycles_valid_ &&
           hardware_write_progress_observed_ &&
           (now - last_hardware_diagnostic_time_) <= hardware_control_loop_timeout_ &&
           (now - last_hardware_write_progress_time_) <= hardware_control_loop_timeout_;
  }

  std::string hardwareControlLoopIssue(
    const std::chrono::steady_clock::time_point & now) const
  {
    if (!hardware_control_loop_required_) {
      return "hardware control-loop check is disabled";
    }
    if (!hardware_diagnostic_received_) {
      return "hardware control-loop diagnostics have not been received";
    }
    if (!hardware_write_cycles_valid_) {
      return "hardware_write_cycles is missing or invalid";
    }
    if (!hardware_write_progress_observed_) {
      return "waiting for hardware_write_cycles to advance";
    }
    if ((now - last_hardware_diagnostic_time_) > hardware_control_loop_timeout_) {
      return "hardware control-loop diagnostics are stale";
    }
    if ((now - last_hardware_write_progress_time_) > hardware_control_loop_timeout_) {
      const auto stalled_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        now - last_hardware_write_progress_time_).count();
      return "hardware control loop is stalled: hardware_write_cycles has not advanced for " +
             std::to_string(stalled_ms) + " ms";
    }
    return "hardware control loop is alive";
  }

  bool clockReady(
    const std::chrono::steady_clock::time_point & now, bool use_sim_time) const
  {
    if (!use_sim_time) {
      return true;
    }

    return has_clock_sample_ && clock_has_advanced_ &&
           (now - last_clock_progress_time_) <= clock_timeout_;
  }

  void pruneMotionHistory(const std::chrono::steady_clock::time_point & now)
  {
    while (!joint_position_history_.empty() &&
      (now - joint_position_history_.front().receive_time) > motion_window_)
    {
      joint_position_history_.pop_front();
    }
  }

  bool motionDetected(const std::chrono::steady_clock::time_point & now)
  {
    pruneMotionHistory(now);
    if (joint_position_history_.size() < 2) {
      return false;
    }

    auto minimum = joint_position_history_.front().positions;
    auto maximum = minimum;

    for (const auto & sample : joint_position_history_) {
      for (std::size_t joint_index = 0; joint_index < kJointCount; ++joint_index) {
        minimum[joint_index] = std::min(minimum[joint_index], sample.positions[joint_index]);
        maximum[joint_index] = std::max(maximum[joint_index], sample.positions[joint_index]);
      }
    }

    for (std::size_t joint_index = 0; joint_index < kJointCount; ++joint_index) {
      if ((maximum[joint_index] - minimum[joint_index]) >
        motion_position_threshold_rad_)
      {
        return true;
      }
    }

    return false;
  }

  RuntimeChecks collectRuntimeChecks(
    const std::chrono::steady_clock::time_point & now)
  {
    RuntimeChecks checks;
    get_parameter_or("use_sim_time", checks.use_sim_time, false);
    checks.hardware_control_loop_required = hardware_control_loop_required_;
    checks.hardware_control_loop_ready = hardwareControlLoopReady(now);
    checks.joint_state_ready = jointStateReady(now);
    checks.transform_ready = transformReady(now);
    checks.controllers_ready = controllersReady(now);
    checks.compute_ik_ready = compute_ik_client_->service_is_ready();
    checks.motion_plan_ready = motion_plan_client_->service_is_ready();
    checks.trajectory_action_ready = trajectory_action_client_->action_server_is_ready();
    checks.clock_ready = clockReady(now, checks.use_sim_time);
    checks.moveit_required = control_mode_ == "MOVEIT";
    return checks;
  }

  std::string firstNotReadyReason(
    const std::chrono::steady_clock::time_point & now,
    const RuntimeChecks & checks) const
  {
    if (!checks.hardware_control_loop_ready) {
      return hardwareControlLoopIssue(now);
    }
    if (!checks.joint_state_ready) {
      if (joint_state_health_ == JointStateHealth::kNoData) {
        return "joint states have not been received";
      }
      if (joint_state_health_ == JointStateHealth::kIncomplete) {
        return "joint state incomplete: " + joint_state_issue_;
      }
      return "joint state data is stale";
    }
    if (!checks.transform_ready) {
      if (!tf_available_) {
        return "tool transform is unavailable";
      }
      if (tf_time_reset_) {
        return "tool transform timestamp moved backwards";
      }
      return "tool transform data is stale";
    }
    if (!checks.controllers_ready) {
      return "controllers are unavailable, inactive, or stale";
    }
    if (checks.moveit_required && !checks.compute_ik_ready) {
      return "/compute_ik service is unavailable";
    }
    if (checks.moveit_required && !checks.motion_plan_ready) {
      return "/plan_kinematic_path service is unavailable";
    }
    if (checks.moveit_required && !checks.trajectory_action_ready) {
      return "FollowJointTrajectory action server is unavailable";
    }
    if (!checks.clock_ready) {
      return "simulation clock is not advancing";
    }
    return "runtime prerequisites are not ready";
  }

  SystemEvaluation evaluateSystemState(
    const std::chrono::steady_clock::time_point & now,
    const RuntimeChecks & checks)
  {
    SystemEvaluation evaluation;
    evaluation.motion_detected = motionDetected(now) || control_mode_ == "JOG_ACTIVE";

    if (control_mode_ == "FAULT") {
      evaluation.state = SystemState::kFault;
      evaluation.reason = "jog mode manager reported FAULT";
      return evaluation;
    }
    if (control_mode_ != "MOVEIT" && control_mode_ != "SWITCHING_TO_JOG" &&
      control_mode_ != "JOG_IDLE" && control_mode_ != "JOG_ACTIVE" &&
      control_mode_ != "STOPPING")
    {
      evaluation.state = SystemState::kFault;
      evaluation.reason = "unknown control mode: " + control_mode_;
      return evaluation;
    }

    if (joint_state_health_ == JointStateHealth::kInvalid) {
      evaluation.state = SystemState::kFault;
      evaluation.reason = "invalid joint state: " + joint_state_issue_;
      return evaluation;
    }
    if (tf_available_ && !tf_values_valid_) {
      evaluation.state = SystemState::kFault;
      evaluation.reason = "tool transform contains invalid values";
      return evaluation;
    }

    const bool all_runtime_checks_ready =
      checks.hardware_control_loop_ready && checks.joint_state_ready && checks.transform_ready &&
      checks.controllers_ready &&
      (!checks.moveit_required ||
      (checks.compute_ik_ready && checks.motion_plan_ready && checks.trajectory_action_ready)) &&
      checks.clock_ready;

    if (all_runtime_checks_ready) {
      has_ever_had_healthy_snapshot_ = true;
      if (evaluation.motion_detected) {
        evaluation.state = SystemState::kMoving;
        evaluation.reason = "joint position window indicates motion";
      } else {
        evaluation.state = SystemState::kReady;
        evaluation.reason = "all runtime prerequisites are healthy";
      }
      return evaluation;
    }

    evaluation.reason = firstNotReadyReason(now, checks);
    if (!has_ever_had_healthy_snapshot_ && (now - node_start_time_) < startup_grace_period_) {
      evaluation.state = SystemState::kStarting;
    } else {
      evaluation.state = SystemState::kNotReady;
    }
    return evaluation;
  }

  static const char * systemStateName(SystemState state)
  {
    switch (state) {
      case SystemState::kStarting:
        return "STARTING";
      case SystemState::kNotReady:
        return "NOT_READY";
      case SystemState::kReady:
        return "READY";
      case SystemState::kMoving:
        return "MOVING";
      case SystemState::kFault:
        return "FAULT";
    }
    return "FAULT";
  }

  static std::uint8_t diagnosticLevel(SystemState state)
  {
    if (state == SystemState::kFault) {
      return diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    }
    if (state == SystemState::kStarting || state == SystemState::kNotReady) {
      return diagnostic_msgs::msg::DiagnosticStatus::WARN;
    }
    return diagnostic_msgs::msg::DiagnosticStatus::OK;
  }

  static diagnostic_msgs::msg::DiagnosticStatus makeDiagnosticStatus(
    std::uint8_t level, const std::string & name, const std::string & message)
  {
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.level = level;
    status.name = name;
    status.message = message;
    status.hardware_id = "elfin3";
    return status;
  }

  static void addDiagnosticValue(
    diagnostic_msgs::msg::DiagnosticStatus & status,
    const std::string & key,
    const std::string & value)
  {
    diagnostic_msgs::msg::KeyValue key_value;
    key_value.key = key;
    key_value.value = value;
    status.values.push_back(key_value);
  }

  static const char * booleanText(bool value)
  {
    return value ? "true" : "false";
  }

  void handleIsReady(std_srvs::srv::Trigger::Response & response)
  {
    const auto now = std::chrono::steady_clock::now();
    const auto checks = collectRuntimeChecks(now);
    const auto evaluation = evaluateSystemState(now, checks);
    response.success = evaluation.state == SystemState::kReady;
    response.message =
      std::string(systemStateName(evaluation.state)) + ": " + evaluation.reason;
  }

  void publishDiagnostics()
  {
    const auto steady_now = std::chrono::steady_clock::now();
    const auto checks = collectRuntimeChecks(steady_now);
    const auto evaluation = evaluateSystemState(steady_now, checks);

    diagnostic_msgs::msg::DiagnosticArray diagnostics;
    diagnostics.header.stamp = this->now();

    auto system_status = makeDiagnosticStatus(
      diagnosticLevel(evaluation.state),
      "elfin3_supervisor/system",
      evaluation.reason);
    addDiagnosticValue(system_status, "state", systemStateName(evaluation.state));
    addDiagnosticValue(
      system_status, "motion_detected", booleanText(evaluation.motion_detected));
    addDiagnosticValue(
      system_status, "is_ready", booleanText(evaluation.state == SystemState::kReady));
    addDiagnosticValue(system_status, "control_mode", control_mode_);
    addDiagnosticValue(
      system_status, "hardware_control_loop_ready",
      booleanText(checks.hardware_control_loop_ready));
    diagnostics.status.push_back(system_status);

    const auto hardware_message = hardwareControlLoopIssue(steady_now);
    auto hardware_status = makeDiagnosticStatus(
      checks.hardware_control_loop_ready ? diagnostic_msgs::msg::DiagnosticStatus::OK :
      diagnostic_msgs::msg::DiagnosticStatus::WARN,
      "elfin3_supervisor/hardware_control_loop", hardware_message);
    addDiagnosticValue(
      hardware_status, "required", booleanText(checks.hardware_control_loop_required));
    addDiagnosticValue(
      hardware_status, "diagnostic_received", booleanText(hardware_diagnostic_received_));
    addDiagnosticValue(
      hardware_status, "counter_valid", booleanText(hardware_write_cycles_valid_));
    addDiagnosticValue(
      hardware_status, "progress_observed", booleanText(hardware_write_progress_observed_));
    addDiagnosticValue(
      hardware_status, "hardware_write_cycles",
      hardware_write_cycles_valid_ ? std::to_string(hardware_write_cycles_) : "unavailable");
    diagnostics.status.push_back(hardware_status);

    std::uint8_t joint_level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    std::string joint_message = "six-axis joint state is valid and fresh";
    if (joint_state_health_ == JointStateHealth::kInvalid) {
      joint_level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      joint_message = joint_state_issue_;
    } else if (joint_state_health_ == JointStateHealth::kIncomplete) {
      joint_level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      joint_message = joint_state_issue_;
    } else if (!checks.joint_state_ready) {
      joint_level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      joint_message = "joint state is unavailable or stale";
    }
    auto joint_status = makeDiagnosticStatus(
      joint_level, "elfin3_supervisor/joint_states", joint_message);
    addDiagnosticValue(joint_status, "healthy", booleanText(checks.joint_state_ready));
    diagnostics.status.push_back(joint_status);

    std::uint8_t tf_level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    std::string tf_message = "tool transform is valid and fresh";
    if (tf_available_ && !tf_values_valid_) {
      tf_level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      tf_message = "tool transform contains invalid values";
    } else if (!checks.transform_ready) {
      tf_level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      tf_message = "tool transform is unavailable, stale, or reset";
    }
    auto tf_status = makeDiagnosticStatus(
      tf_level, "elfin3_supervisor/tool_transform", tf_message);
    addDiagnosticValue(tf_status, "base_frame", base_frame_);
    addDiagnosticValue(tf_status, "tool_frame", tool_frame_);
    diagnostics.status.push_back(tf_status);

    auto controller_status = makeDiagnosticStatus(
      checks.controllers_ready ? diagnostic_msgs::msg::DiagnosticStatus::OK :
      diagnostic_msgs::msg::DiagnosticStatus::WARN,
      "elfin3_supervisor/controllers",
      checks.controllers_ready ? "required controllers are active" :
      "required controllers are unavailable, inactive, or stale");
    addDiagnosticValue(
      controller_status, "joint_state_broadcaster_active",
      booleanText(joint_state_broadcaster_active_));
    addDiagnosticValue(
      controller_status, "elfin_arm_controller_active",
      booleanText(arm_controller_active_));
    addDiagnosticValue(
      controller_status, "elfin_jog_controller_active",
      booleanText(jog_controller_active_));
    diagnostics.status.push_back(controller_status);

    const bool moveit_ready =
      checks.compute_ik_ready && checks.motion_plan_ready && checks.trajectory_action_ready;
    auto moveit_status = makeDiagnosticStatus(
      (moveit_ready || !checks.moveit_required) ? diagnostic_msgs::msg::DiagnosticStatus::OK :
      diagnostic_msgs::msg::DiagnosticStatus::WARN,
      "elfin3_supervisor/moveit",
      !checks.moveit_required ? "MoveIt interfaces are not required in Jog mode" :
      moveit_ready ? "MoveIt and trajectory interfaces are available" :
      "one or more MoveIt or trajectory interfaces are unavailable");
    addDiagnosticValue(moveit_status, "required", booleanText(checks.moveit_required));
    addDiagnosticValue(moveit_status, "compute_ik", booleanText(checks.compute_ik_ready));
    addDiagnosticValue(
      moveit_status, "plan_kinematic_path", booleanText(checks.motion_plan_ready));
    addDiagnosticValue(
      moveit_status, "follow_joint_trajectory",
      booleanText(checks.trajectory_action_ready));
    diagnostics.status.push_back(moveit_status);

    auto clock_status = makeDiagnosticStatus(
      checks.clock_ready ? diagnostic_msgs::msg::DiagnosticStatus::OK :
      diagnostic_msgs::msg::DiagnosticStatus::WARN,
      "elfin3_supervisor/clock",
      checks.use_sim_time ?
      (checks.clock_ready ? "simulation clock is advancing" :
      "simulation clock is unavailable or stopped") :
      "system time is selected");
    addDiagnosticValue(clock_status, "use_sim_time", booleanText(checks.use_sim_time));
    diagnostics.status.push_back(clock_status);

    diagnostics_publisher_->publish(diagnostics);
  }

  void renderTerminal()
  {
    const auto now = std::chrono::steady_clock::now();
    const auto checks = collectRuntimeChecks(now);
    const auto evaluation = evaluateSystemState(now, checks);
    const auto interface_status = [](bool ready) {return ready ? "OK" : "WAIT";};
    const char * hardware_status = checks.hardware_control_loop_required ?
      interface_status(checks.hardware_control_loop_ready) : "N/A";
    const char * clock_status = checks.use_sim_time ?
      interface_status(checks.clock_ready) : "N/A";
    constexpr double kRadiansToDegrees =
      180.0 / 3.14159265358979323846;

    std::ostringstream output;
    output << "\033[2J\033[H";
    output << "============================================================\n";
    output << " Elfin3 Supervisor\n";
    output << " System : " << systemStateName(evaluation.state) << '\n';
    output << " Mode   : " << control_mode_ << '\n';
    output << " Reason : " << evaluation.reason << '\n';
    output << " Motion : " << (evaluation.motion_detected ? "MOVING" : "STOPPED") << '\n';
    output << "------------------------------------------------------------\n";
    output << " Runtime interfaces\n";
    output << " Hardware   : " << std::setw(4) << hardware_status << '\n';
    output << " JointState : " << std::setw(4) << interface_status(checks.joint_state_ready)
           << "   Tool TF     : " << std::setw(4) << interface_status(checks.transform_ready)
           << "   Controllers : " << interface_status(checks.controllers_ready) << '\n';
    output << " IK         : " << std::setw(4) << interface_status(checks.compute_ik_ready)
           << "   Planning    : " << std::setw(4) << interface_status(checks.motion_plan_ready)
           << "   Trajectory  : " << interface_status(checks.trajectory_action_ready) << '\n';
    output << " Clock      : " << clock_status
           << "   Time source : " << (checks.use_sim_time ? "SIM" : "SYSTEM") << '\n';
    output << "------------------------------------------------------------\n";
    output << " Joints J1..J6 (latest valid sample)\n";

    if (has_valid_joint_state_) {
      output << " Joint       rad          deg\n";
      output << std::fixed << std::setprecision(6);
      for (std::size_t joint_index = 0; joint_index < kJointCount; ++joint_index) {
        output << " J" << (joint_index + 1)
               << "     " << std::setw(10) << latest_joint_positions_[joint_index]
               << "   " << std::setw(10)
               << latest_joint_positions_[joint_index] * kRadiansToDegrees << '\n';
      }
    } else {
      output << " UNAVAILABLE: no complete valid JointState received\n";
    }

    output << "------------------------------------------------------------\n";
    output << " 末端位姿 [" << base_frame_ << "] -> " << tool_frame_ << '\n';
    if (tf_available_ && tf_values_valid_) {
      output << std::fixed << std::setprecision(6);
      output << " XYZ [m]   x=" << latest_tool_position_[0]
             << "  y=" << latest_tool_position_[1]
             << "  z=" << latest_tool_position_[2] << '\n';
      output << " RPY [rad] r=" << latest_tool_rpy_[0]
             << "  p=" << latest_tool_rpy_[1]
             << "  y=" << latest_tool_rpy_[2] << '\n';
    } else {
      output << " UNAVAILABLE";
      if (!tf_issue_.empty()) {
        output << ": " << tf_issue_;
      }
      output << '\n';
    }
    output << "============================================================\n";

    std::cout << output.str() << std::flush;
  }

  std::array<double, kJointCount> latest_joint_positions_{};
  std::array<double, 3> latest_tool_position_{};
  std::array<double, 3> latest_tool_rpy_{};
  std::deque<JointPositionSample> joint_position_history_;
  std::chrono::steady_clock::time_point node_start_time_{};
  std::chrono::steady_clock::time_point last_valid_joint_state_time_{};
  std::chrono::steady_clock::time_point last_tf_update_time_{};
  std::chrono::steady_clock::time_point controller_request_sent_time_{};
  std::chrono::steady_clock::time_point last_controller_response_time_{};
  std::chrono::steady_clock::time_point last_clock_progress_time_{};
  std::chrono::steady_clock::time_point last_hardware_diagnostic_time_{};
  std::chrono::steady_clock::time_point last_hardware_write_progress_time_{};
  std::chrono::duration<double> joint_state_timeout_{0.5};
  std::chrono::duration<double> tf_timeout_{0.5};
  std::chrono::duration<double> controller_response_timeout_{0.5};
  std::chrono::duration<double> controller_state_timeout_{2.5};
  std::chrono::duration<double> clock_timeout_{1.0};
  std::chrono::duration<double> startup_grace_period_{15.0};
  std::chrono::duration<double> hardware_control_loop_timeout_{3.0};
  std::chrono::duration<double> motion_window_{0.2};
  double motion_position_threshold_rad_{0.001};
  JointStateHealth joint_state_health_{JointStateHealth::kNoData};
  std::string joint_state_issue_;
  std::string tf_issue_;
  std::string base_frame_;
  std::string tool_frame_;
  std::string control_mode_{"MOVEIT"};
  std::int64_t last_tf_stamp_ns_{0};
  std::int64_t last_clock_time_ns_{0};
  std::int64_t controller_request_id_{-1};
  std::uint64_t controller_request_generation_{0};
  std::uint64_t hardware_write_cycles_{0};
  bool has_valid_joint_state_{false};
  bool tf_available_{false};
  bool tf_values_valid_{false};
  bool tf_time_reset_{false};
  bool controller_request_pending_{false};
  bool controllers_response_received_{false};
  bool joint_state_broadcaster_active_{false};
  bool arm_controller_active_{false};
  bool jog_controller_active_{false};
  bool has_clock_sample_{false};
  bool clock_has_advanced_{false};
  bool has_ever_had_healthy_snapshot_{false};
  bool hardware_control_loop_required_{true};
  bool hardware_diagnostic_received_{false};
  bool hardware_write_counter_initialized_{false};
  bool hardware_write_cycles_valid_{false};
  bool hardware_write_progress_observed_{false};
  bool terminal_output_{true};
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscription_;
  rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr clock_subscription_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr control_mode_subscription_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr
    hardware_diagnostics_subscription_;
  rclcpp::Client<ListControllers>::SharedPtr controller_client_;
  rclcpp::Client<moveit_msgs::srv::GetPositionIK>::SharedPtr compute_ik_client_;
  rclcpp::Client<moveit_msgs::srv::GetMotionPlan>::SharedPtr motion_plan_client_;
  rclcpp_action::Client<FollowJointTrajectory>::SharedPtr trajectory_action_client_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr is_ready_service_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::TimerBase::SharedPtr display_timer_;
  rclcpp::TimerBase::SharedPtr controller_poll_timer_;
  rclcpp::TimerBase::SharedPtr diagnostics_timer_;
};

}  // namespace elfin3_supervisor

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<elfin3_supervisor::Elfin3SupervisorNode>());
  rclcpp::shutdown();
  return 0;
}
