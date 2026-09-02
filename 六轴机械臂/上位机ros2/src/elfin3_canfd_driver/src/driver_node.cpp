#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <limits>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <linux/can/error.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "elfin3_canfd_driver/can_fd_transport.hpp"
#include "elfin3_canfd_driver/csp_scheduler.hpp"
#include "elfin3_canfd_driver/decode.hpp"
#include "elfin3_canfd_driver/encode.hpp"
#include "elfin3_canfd_driver/position_conversion.hpp"
#include "elfin3_canfd_driver/protocol.hpp"
#include "elfin3_canfd_driver/sequence_tracker.hpp"

namespace elfin3_canfd
{

class DriverNode : public rclcpp::Node
{
public:
  DriverNode()
  : Node("elfin3_canfd_driver")
  {
    const auto interface_name = declare_parameter<std::string>("interface_name", "can0");
    const auto poll_period_ms = declare_parameter<int>("receive_poll_period_ms", 1);
    position_timeout_ms_ = declare_parameter<int>("position_timeout_ms", 20);
    detailed_timeout_ms_ = declare_parameter<int>("detailed_timeout_ms", 100);
    heartbeat_timeout_ms_ = declare_parameter<int>("heartbeat_timeout_ms", 300);
    trajectory_status_timeout_ms_ = declare_parameter<int>(
      "trajectory_status_timeout_ms", 100);
    const auto diagnostics_period_ms = declare_parameter<int>("diagnostics_period_ms", 1000);
    const auto heartbeat_period_ms = declare_parameter<int>("host_heartbeat_period_ms", 100);
    const auto csp_period_us = declare_parameter<int>(
      "csp_period_us", static_cast<int>(kCspPeriodUs));
    const auto csp_target_timeout_ms = declare_parameter<int>("csp_target_timeout_ms", 100);
    const auto csp_validity_ms = declare_parameter<int>(
      "csp_validity_ms", static_cast<int>(kCspValidityMs));
    const auto csp_refill_period_us = declare_parameter<int>(
      "csp_refill_period_us", static_cast<int>(kDefaultCspRefillPeriodUs));
    const auto csp_remote_low_watermark = declare_parameter<int>(
      "csp_remote_low_watermark", static_cast<int>(kDefaultRemoteLowWatermark));
    const auto csp_remote_high_watermark = declare_parameter<int>(
      "csp_remote_high_watermark", static_cast<int>(kDefaultRemoteHighWatermark));
    csp_thread_priority_ = declare_parameter<int>(
      "csp_thread_priority", kDefaultCspThreadPriority);
    const auto zero_offsets = declare_parameter<std::vector<std::int64_t>>(
      "joint_zero_offset_counts", std::vector<std::int64_t>{});
    const auto direction_signs = declare_parameter<std::vector<std::int64_t>>(
      "joint_direction_signs", std::vector<std::int64_t>{});
    if (poll_period_ms <= 0) {
      throw std::invalid_argument("receive_poll_period_ms must be positive");
    }
    if (position_timeout_ms_ <= 0 || detailed_timeout_ms_ <= 0 ||
      heartbeat_timeout_ms_ <= 0 || trajectory_status_timeout_ms_ <= 0 ||
      diagnostics_period_ms <= 0 ||
      heartbeat_period_ms <= 0 || heartbeat_period_ms >= 300 ||
      csp_period_us != static_cast<int>(kCspPeriodUs) ||
      csp_target_timeout_ms <= 0 ||
      csp_refill_period_us <= 0 || csp_refill_period_us >= csp_period_us ||
      csp_remote_low_watermark < 0 ||
      csp_remote_low_watermark >= csp_remote_high_watermark ||
      csp_remote_high_watermark >
      static_cast<int>(kExpectedRemoteQueueCapacity - kRemoteQueueReserve) ||
      csp_thread_priority_ < 0 || csp_thread_priority_ > 99 ||
      csp_validity_ms != static_cast<int>(kCspValidityMs))
    {
      throw std::invalid_argument(
              "CAN FD timeouts must be positive and protocol timing must match constants");
    }
    load_calibration(zero_offsets, direction_signs);

    std::string error;
    if (!transport_.open(interface_name, error)) {
      throw std::runtime_error("Failed to open " + interface_name + ": " + error);
    }
    csp_scheduler_ = std::make_unique<CspScheduler>(
      [this](
        const std::uint32_t id, const std::uint8_t * data, const std::size_t size,
        std::string & send_error)
      {
        return send_frame(id, data, size, send_error);
      },
      std::chrono::microseconds(csp_period_us),
      std::chrono::milliseconds(csp_target_timeout_ms),
      static_cast<std::uint16_t>(csp_validity_ms), kDefaultCspTargetQueueCapacity,
      static_cast<std::size_t>(csp_remote_low_watermark),
      static_cast<std::size_t>(csp_remote_high_watermark),
      std::chrono::microseconds(csp_refill_period_us), csp_thread_priority_);
    csp_scheduler_->start();
    receive_timer_ = create_wall_timer(
      std::chrono::milliseconds(poll_period_ms), [this]() {poll_receive();});
    diagnostics_publisher_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/diagnostics", rclcpp::QoS(10));
    joint_state_publisher_ = create_publisher<sensor_msgs::msg::JointState>(
      "/joint_states", rclcpp::SensorDataQoS());
    diagnostics_timer_ = create_wall_timer(
      std::chrono::milliseconds(diagnostics_period_ms), [this]() {publish_diagnostics();});
    heartbeat_timer_ = create_wall_timer(
      std::chrono::milliseconds(heartbeat_period_ms), [this]() {send_host_heartbeat();});
    disable_service_ = create_control_service("/elfin3_canfd/disable", MotionControl::kDisable);
    enable_service_ = create_control_service("/elfin3_canfd/enable", MotionControl::kEnable);
    hold_service_ = create_control_service("/elfin3_canfd/hold", MotionControl::kHold);
    quick_stop_service_ = create_control_service(
      "/elfin3_canfd/quick_stop", MotionControl::kQuickStop);
    fault_reset_service_ = create_control_service(
      "/elfin3_canfd/fault_reset", MotionControl::kFaultReset);
    RCLCPP_INFO(get_logger(), "CAN FD driver opened on %s", interface_name.c_str());
  }

  ~DriverNode() override
  {
    if (csp_scheduler_) {
      csp_scheduler_->stop();
    }
    std::string error;
    std::uint32_t token = 0;
    transmit_control(MotionControl::kHold, error, token);
    send_host_heartbeat(HostState::kShuttingDown);
  }

private:
  enum class LinkState
  {
    kDown,
    kLinkUp,
    kHandshaking,
    kReady,
    kEnabled,
    kOperational,
    kDegraded,
    kBusOff,
    kFault,
  };

  static const char * state_name(const LinkState state)
  {
    switch (state) {
      case LinkState::kDown: return "DOWN";
      case LinkState::kLinkUp: return "LINK_UP";
      case LinkState::kHandshaking: return "HANDSHAKING";
      case LinkState::kReady: return "READY";
      case LinkState::kEnabled: return "ENABLED";
      case LinkState::kOperational: return "OPERATIONAL";
      case LinkState::kDegraded: return "DEGRADED";
      case LinkState::kBusOff: return "BUS_OFF";
      case LinkState::kFault: return "FAULT";
    }
    return "UNKNOWN";
  }

  HostState heartbeat_state() const
  {
    const auto state = evaluate_state(std::chrono::steady_clock::now());
    switch (state) {
      case LinkState::kLinkUp:
      case LinkState::kHandshaking:
        return HostState::kStarting;
      case LinkState::kReady:
      case LinkState::kEnabled:
        return HostState::kReady;
      case LinkState::kOperational:
        return HostState::kMoving;
      case LinkState::kFault:
      case LinkState::kBusOff:
        return HostState::kFault;
      case LinkState::kDown:
      case LinkState::kDegraded:
        return HostState::kNotReady;
    }
    return HostState::kFault;
  }

  static bool accepted_sequence(const SequenceResult result)
  {
    return result == SequenceResult::kFirst || result == SequenceResult::kAccepted ||
           result == SequenceResult::kAcceptedWithGap;
  }

  void load_calibration(
    const std::vector<std::int64_t> & zero_offsets,
    const std::vector<std::int64_t> & direction_signs)
  {
    if (zero_offsets.empty() && direction_signs.empty()) {
      RCLCPP_WARN(
        get_logger(),
        "Joint calibration is not loaded; monitoring is allowed but Enable is blocked");
      return;
    }
    if (zero_offsets.size() != kAxisCount || direction_signs.size() != kAxisCount) {
      throw std::invalid_argument(
              "joint_zero_offset_counts and joint_direction_signs must each contain 6 values");
    }

    for (std::size_t joint = 0; joint < kAxisCount; ++joint) {
      if (zero_offsets[joint] < std::numeric_limits<std::int32_t>::min() ||
        zero_offsets[joint] > std::numeric_limits<std::int32_t>::max())
      {
        throw std::invalid_argument("joint_zero_offset_counts value is outside int32 range");
      }
      if (direction_signs[joint] != -1 && direction_signs[joint] != 1) {
        throw std::invalid_argument("joint_direction_signs values must be either -1 or 1");
      }
      joint_zero_offset_counts_[joint] = static_cast<std::int32_t>(zero_offsets[joint]);
      joint_direction_signs_[joint] = static_cast<std::int8_t>(direction_signs[joint]);
    }
    calibration_ready_ = true;
    RCLCPP_INFO(
      get_logger(),
      "Joint calibration loaded in ROS joint order: offsets=[%d, %d, %d, %d, %d, %d], "
      "directions=[%d, %d, %d, %d, %d, %d]",
      joint_zero_offset_counts_[0], joint_zero_offset_counts_[1],
      joint_zero_offset_counts_[2], joint_zero_offset_counts_[3],
      joint_zero_offset_counts_[4], joint_zero_offset_counts_[5],
      static_cast<int>(joint_direction_signs_[0]),
      static_cast<int>(joint_direction_signs_[1]),
      static_cast<int>(joint_direction_signs_[2]),
      static_cast<int>(joint_direction_signs_[3]),
      static_cast<int>(joint_direction_signs_[4]),
      static_cast<int>(joint_direction_signs_[5]));
  }

  template<typename FrameType>
  static FrameType copy_payload(const CanFdFrame & frame)
  {
    FrameType payload{};
    std::copy_n(frame.data.begin(), payload.size(), payload.begin());
    return payload;
  }

  void poll_receive()
  {
    for (std::size_t count = 0; count < 128U; ++count) {
      CanFdFrame frame;
      std::uint32_t can_error = 0;
      std::string error;
      const auto result = transport_.receive(frame, can_error, error);
      if (result == ReceiveResult::kNoData) {
        return;
      }
      if (result == ReceiveResult::kCanError) {
        if ((can_error & CAN_ERR_BUSOFF) != 0U) {
          bus_off_ = true;
        }
        RCLCPP_ERROR(
          get_logger(), "SocketCAN error frame: mask=0x%08x",
          static_cast<unsigned>(can_error));
        continue;
      }
      if (result == ReceiveResult::kError) {
        ++receive_rejection_count_;
        if (receive_rejection_count_ == 1U || receive_rejection_count_ % 100U == 0U) {
          RCLCPP_WARN(get_logger(), "Rejected CAN frame: %s", error.c_str());
        }
        continue;
      }
      bus_off_ = false;
      dispatch(frame);
    }
  }

  static std::int64_t age_ms(
    const bool available, const std::chrono::steady_clock::time_point received,
    const std::chrono::steady_clock::time_point now)
  {
    if (!available) {
      return -1;
    }
    return std::chrono::duration_cast<std::chrono::milliseconds>(now - received).count();
  }

  bool periodic_fault_present() const
  {
    if (!has_position_ || !has_detailed_) {
      return false;
    }
    if ((latest_position_.status_flags & 0x20U) != 0U) {
      return true;
    }
    for (std::size_t axis = 0; axis < kAxisCount; ++axis) {
      if ((latest_detailed_.valid_axes_mask & (1U << axis)) != 0U &&
        latest_detailed_.drive_error_code[axis] != 0U)
      {
        return true;
      }
    }
    return false;
  }

  LinkState evaluate_state(const std::chrono::steady_clock::time_point now) const
  {
    if (!transport_.is_open()) {
      return LinkState::kDown;
    }
    if (bus_off_) {
      return LinkState::kBusOff;
    }
    if (periodic_fault_present()) {
      return LinkState::kFault;
    }
    if (!has_heartbeat_) {
      return LinkState::kLinkUp;
    }
    if (!has_position_ || !has_detailed_ || !has_trajectory_status_) {
      return LinkState::kHandshaking;
    }

    const bool stale =
      age_ms(true, last_position_time_, now) > position_timeout_ms_ ||
      age_ms(true, last_detailed_time_, now) > detailed_timeout_ms_ ||
      age_ms(true, last_heartbeat_time_, now) > heartbeat_timeout_ms_ ||
      age_ms(true, last_trajectory_status_time_, now) > trajectory_status_timeout_ms_;
    const bool complete_masks =
      latest_position_.valid_axes_mask == kAllAxesMask &&
      latest_detailed_.valid_axes_mask == kAllAxesMask;
    if (stale || !complete_masks) {
      return LinkState::kDegraded;
    }
    const bool ethercat_operational = (latest_position_.status_flags & 0x01U) != 0U;
    const bool all_axes_enabled = (latest_position_.status_flags & 0x02U) != 0U;
    const bool safety_latched = (latest_position_.status_flags & 0x0cU) != 0U;
    const bool trajectory_safety_latched =
      (latest_trajectory_status_.flags &
      static_cast<std::uint8_t>(kSafetyHoldLatched | kQuickStopLatched)) != 0U;
    if (!ethercat_operational || safety_latched || trajectory_safety_latched) {
      return LinkState::kReady;
    }
    if (!all_axes_enabled) {
      return LinkState::kReady;
    }
    return csp_scheduler_ && csp_scheduler_->is_applying() ?
           LinkState::kOperational : LinkState::kEnabled;
  }

  static void add_value(
    diagnostic_msgs::msg::DiagnosticStatus & status, const std::string & key,
    const std::string & value)
  {
    diagnostic_msgs::msg::KeyValue item;
    item.key = key;
    item.value = value;
    status.values.push_back(item);
  }

  void publish_joint_state(const std::array<double, kAxisCount> & positions)
  {
    sensor_msgs::msg::JointState message;
    message.header.stamp = now();
    message.name = {
      "elfin_joint1", "elfin_joint2", "elfin_joint3",
      "elfin_joint4", "elfin_joint5", "elfin_joint6"};
    message.position.assign(positions.begin(), positions.end());
    joint_state_publisher_->publish(message);
    ++joint_state_publications_;
  }

  void publish_diagnostics()
  {
    const auto steady_now = std::chrono::steady_clock::now();
    const auto state = evaluate_state(steady_now);
    if (!state_initialized_ || state != current_state_) {
      RCLCPP_INFO(get_logger(), "CAN FD state: %s", state_name(state));
      current_state_ = state;
      state_initialized_ = true;
    }

    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "elfin3_canfd_driver/communication";
    status.hardware_id = "elfin3";
    status.message = state_name(state);
    if (state == LinkState::kBusOff || state == LinkState::kFault) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    } else if (state == LinkState::kReady || state == LinkState::kEnabled ||
      state == LinkState::kOperational)
    {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    } else {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    }

    add_value(status, "position_age_ms", std::to_string(
        age_ms(has_position_, last_position_time_, steady_now)));
    add_value(status, "detailed_age_ms", std::to_string(
        age_ms(has_detailed_, last_detailed_time_, steady_now)));
    add_value(status, "heartbeat_age_ms", std::to_string(
        age_ms(has_heartbeat_, last_heartbeat_time_, steady_now)));
    add_value(status, "trajectory_status_age_ms", std::to_string(
        age_ms(has_trajectory_status_, last_trajectory_status_time_, steady_now)));
    add_value(status, "trajectory_status_received", has_trajectory_status_ ? "true" : "false");
    if (has_trajectory_status_) {
      add_value(status, "trajectory_state", std::to_string(
          static_cast<unsigned>(latest_trajectory_status_.state)));
      add_value(status, "trajectory_queue_depth", std::to_string(
          latest_trajectory_status_.queue_depth));
      add_value(status, "trajectory_queue_capacity", std::to_string(
          latest_trajectory_status_.queue_capacity));
      add_value(status, "trajectory_prefill_target", std::to_string(
          latest_trajectory_status_.prefill_target));
      add_value(status, "trajectory_queue_contract_ok",
        latest_trajectory_status_.queue_capacity == kExpectedRemoteQueueCapacity &&
        latest_trajectory_status_.prefill_target == kExpectedRemotePrefillTarget ?
        "true" : "false");
      add_value(status, "trajectory_flags", std::to_string(
          latest_trajectory_status_.flags));
      add_value(status, "trajectory_reset_required",
        (latest_trajectory_status_.flags & kTrajectoryResetRequired) != 0U ?
        "true" : "false");
      add_value(status, "trajectory_safety_hold_latched",
        (latest_trajectory_status_.flags & kSafetyHoldLatched) != 0U ?
        "true" : "false");
      add_value(status, "trajectory_quick_stop_latched",
        (latest_trajectory_status_.flags & kQuickStopLatched) != 0U ?
        "true" : "false");
      add_value(status, "trajectory_generation", std::to_string(
          latest_trajectory_status_.generation));
      add_value(status, "trajectory_last_executed_ecat_cycle", std::to_string(
          latest_trajectory_status_.last_executed_ecat_cycle));
      add_value(status, "trajectory_last_received_sequence", std::to_string(
          latest_trajectory_status_.last_received_sequence));
      add_value(status, "trajectory_last_accepted_sequence", std::to_string(
          latest_trajectory_status_.last_accepted_sequence));
      add_value(status, "trajectory_last_executed_sequence", std::to_string(
          latest_trajectory_status_.last_executed_sequence));
      add_value(status, "trajectory_expected_sequence", std::to_string(
          latest_trajectory_status_.expected_sequence));
      add_value(status, "trajectory_last_rejected_sequence", std::to_string(
          latest_trajectory_status_.last_rejected_sequence));
      add_value(status, "trajectory_reject_reason", std::to_string(
          static_cast<unsigned>(latest_trajectory_status_.reject_reason)));
      add_value(status, "trajectory_hold_reason", std::to_string(
          static_cast<unsigned>(latest_trajectory_status_.hold_reason)));
      add_value(status, "trajectory_accepted_count", std::to_string(
          latest_trajectory_status_.accepted_count));
      add_value(status, "trajectory_executed_count", std::to_string(
          latest_trajectory_status_.executed_count));
      add_value(status, "trajectory_rejected_count", std::to_string(
          latest_trajectory_status_.rejected_count));
      add_value(status, "trajectory_underrun_count", std::to_string(
          latest_trajectory_status_.underrun_count));
      add_value(status, "trajectory_overflow_count", std::to_string(
          latest_trajectory_status_.overflow_count));
      add_value(status, "trajectory_expired_count", std::to_string(
          latest_trajectory_status_.expired_count));
    }
    add_value(status, "protocol_rejections", std::to_string(protocol_rejection_count_));
    add_value(status, "calibration_ready", calibration_ready_ ? "true" : "false");
    add_value(status, "joint_state_publications", std::to_string(joint_state_publications_));
    TransportStats transport_stats;
    {
      std::lock_guard<std::mutex> lock(transmit_mutex_);
      transport_stats = transport_.stats();
    }
    add_value(status, "transport_rx", std::to_string(transport_stats.received));
    add_value(status, "transport_tx", std::to_string(transport_stats.transmitted));
    add_value(status, "transport_malformed", std::to_string(transport_stats.malformed));
    add_value(status, "can_error_frames", std::to_string(transport_stats.can_error_frames));
    add_value(status, "heartbeat_tx_errors", std::to_string(heartbeat_tx_errors_));
    add_value(status, "last_control_token", std::to_string(last_control_token_));
    add_value(status, "last_control_command", has_sent_control_ ?
      std::to_string(static_cast<unsigned>(last_control_command_)) : "none");
    add_value(status, "last_control_age_ms", std::to_string(
        age_ms(has_sent_control_, last_control_sent_at_, steady_now)));
    if (csp_scheduler_) {
      const auto csp_stats = csp_scheduler_->stats();
      add_value(status, "csp_stream_state", std::to_string(
          static_cast<unsigned>(csp_scheduler_->stream_state())));
      add_value(status, "csp_apply_frames", std::to_string(csp_stats.apply_frames));
      add_value(status, "csp_hold_frames", std::to_string(csp_stats.hold_frames));
      add_value(status, "csp_tx_errors", std::to_string(csp_stats.transmit_errors));
      add_value(status, "csp_overruns", std::to_string(csp_stats.overruns));
      add_value(status, "csp_queue_overflows", std::to_string(csp_stats.queue_overflows));
      add_value(status, "csp_queue_underruns", std::to_string(csp_stats.queue_underruns));
      add_value(status, "csp_credit_stalls", std::to_string(csp_stats.credit_stalls));
      add_value(status, "csp_high_watermark_stalls", std::to_string(
          csp_stats.high_watermark_stalls));
      add_value(status, "csp_refill_cycles", std::to_string(csp_stats.refill_cycles));
      add_value(status, "csp_refill_transmissions", std::to_string(
          csp_stats.refill_transmissions));
      add_value(status, "csp_filler_frames", std::to_string(csp_stats.filler_frames));
      add_value(status, "csp_local_point_wait_cycles", std::to_string(
          csp_stats.local_point_wait_cycles));
      add_value(status, "csp_remote_faults", std::to_string(csp_stats.remote_faults));
      add_value(status, "csp_remote_queue_depth", std::to_string(
          csp_stats.remote_queue_depth));
      add_value(status, "csp_minimum_remote_queue_depth",
        csp_stats.minimum_remote_queue_depth == 0xffU ?
        "unavailable" : std::to_string(csp_stats.minimum_remote_queue_depth));
      add_value(status, "csp_unacknowledged_points", std::to_string(
          csp_stats.unacknowledged_points));
      add_value(status, "csp_cycles", std::to_string(csp_stats.cycles));
      add_value(status, "csp_last_tx_gap_us", std::to_string(csp_stats.last_tx_gap_us));
      add_value(status, "csp_max_tx_gap_us", std::to_string(csp_stats.max_tx_gap_us));
      add_value(status, "csp_realtime_priority", std::to_string(csp_thread_priority_));
      add_value(status, "csp_realtime_scheduling_active",
        csp_stats.realtime_scheduling_active ? "true" : "false");
      add_value(status, "csp_realtime_scheduling_error", std::to_string(
          csp_stats.realtime_scheduling_error));
      add_value(
        status, "csp_last_cycle_interval_us",
        std::to_string(csp_stats.last_cycle_interval_us));
      add_value(
        status, "csp_last_cycle_jitter_us",
        std::to_string(csp_stats.last_cycle_jitter_us));
      add_value(
        status, "csp_max_cycle_jitter_us",
        std::to_string(csp_stats.max_cycle_jitter_us));
      add_value(
        status, "csp_last_wakeup_lateness_us",
        std::to_string(csp_stats.last_wakeup_lateness_us));
      add_value(
        status, "csp_max_wakeup_lateness_us",
        std::to_string(csp_stats.max_wakeup_lateness_us));
    }

    diagnostic_msgs::msg::DiagnosticArray message;
    message.header.stamp = now();
    message.status.push_back(status);
    diagnostics_publisher_->publish(message);
  }

  void send_host_heartbeat()
  {
    send_host_heartbeat(heartbeat_state());
  }

  void send_host_heartbeat(const HostState state)
  {
    if (!transport_.is_open()) {
      return;
    }
    const auto uptime = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::steady_clock::now() - started_at_).count() / 100;
    HostHeartbeat heartbeat;
    heartbeat.sequence = heartbeat_tx_sequence_++;
    heartbeat.state = state;
    heartbeat.flags = kHostFlagTrajectoryStatusSupported;
    heartbeat.uptime_100ms = static_cast<std::uint16_t>(uptime);
    HostHeartbeatFrame frame{};
    std::string error;
    if (!encode_host_heartbeat(heartbeat, frame) ||
      !send_frame(kHostHeartbeatId, frame.data(), frame.size(), error))
    {
      ++heartbeat_tx_errors_;
      if (heartbeat_tx_errors_ == 1U || heartbeat_tx_errors_ % 10U == 0U) {
        RCLCPP_ERROR(get_logger(), "Failed to send 0x183 heartbeat: %s", error.c_str());
      }
    }
  }

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr create_control_service(
    const std::string & name, const MotionControl command)
  {
    return create_service<std_srvs::srv::Trigger>(
      name,
      [this, command](
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
      {
        handle_control_request(command, *response);
      });
  }

  bool recent_device_heartbeat() const
  {
    return has_heartbeat_ && age_ms(
      true, last_heartbeat_time_, std::chrono::steady_clock::now()) <= heartbeat_timeout_ms_;
  }

  void handle_control_request(
    const MotionControl command, std_srvs::srv::Trigger::Response & response)
  {
    const auto state = evaluate_state(std::chrono::steady_clock::now());
    if (!transport_.is_open() || bus_off_) {
      response.success = false;
      response.message = "CAN FD link is unavailable";
      return;
    }
    if (command == MotionControl::kEnable) {
      if (!calibration_ready_) {
        response.success = false;
        response.message = "Enable rejected: joint calibration is not loaded";
        return;
      }
      if (state == LinkState::kEnabled || state == LinkState::kOperational) {
        response.success = true;
        response.message = "Axes are already enabled";
        return;
      }
      if (state != LinkState::kReady || !recent_device_heartbeat() ||
        !has_position_ || !has_detailed_ ||
        latest_position_.valid_axes_mask != kAllAxesMask ||
        latest_detailed_.valid_axes_mask != kAllAxesMask ||
        (latest_position_.status_flags & 0x01U) == 0U ||
        (latest_position_.status_flags & 0x0cU) != 0U)
      {
        response.success = false;
        response.message = "Enable rejected: communication or safety state is not READY";
        return;
      }
    }
    if (command == MotionControl::kFaultReset && !recent_device_heartbeat()) {
      response.success = false;
      response.message = "Fault Reset rejected: STM32 heartbeat is not fresh";
      return;
    }

    std::string error;
    std::uint32_t token = 0;
    if (!transmit_control(command, error, token)) {
      response.success = false;
      response.message = "0x181 transmit failed: " + error;
      return;
    }
    response.success = true;
    response.message = "0x181 sent with token " + std::to_string(token) +
      "; execution must be confirmed from 0x200/0x201 feedback";
  }

  bool transmit_control(
    const MotionControl command, std::string & error, std::uint32_t & token)
  {
    if (!transport_.is_open()) {
      error = "CAN socket is not open";
      return false;
    }
    const auto uptime_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::steady_clock::now() - started_at_).count();
    MotionControlCommand request;
    request.sequence = control_tx_sequence_++;
    request.command = command;
    request.axes_mask = kAllAxesMask;
    request.request_token = next_control_token_++;
    request.host_time_ms = static_cast<std::uint32_t>(uptime_ms);
    MotionControlFrame frame{};
    if (!encode_motion_control(request, frame)) {
      error = "motion control encoder rejected a locally generated command";
      return false;
    }
    if (!send_frame(kMotionControlId, frame.data(), frame.size(), error)) {
      return false;
    }
    token = request.request_token;
    last_control_token_ = token;
    last_control_command_ = command;
    last_control_sent_at_ = std::chrono::steady_clock::now();
    has_sent_control_ = true;
    RCLCPP_INFO(
      get_logger(), "0x181 sent: command=%u token=%u",
      static_cast<unsigned>(command), static_cast<unsigned>(token));
    return true;
  }

  bool send_frame(
    const std::uint32_t id, const std::uint8_t * data, const std::size_t size,
    std::string & error)
  {
    std::lock_guard<std::mutex> lock(transmit_mutex_);
    return transport_.send(id, data, size, error);
  }

  void dispatch(const CanFdFrame & frame)
  {
    switch (frame.id) {
      case kPositionFeedbackId: {
        PositionFeedback decoded;
        if (!decode_position_feedback(copy_payload<PositionFeedbackFrame>(frame), decoded)) {
          ++protocol_rejection_count_;
          return;
        }
        const bool complete_position = decoded.valid_axes_mask == kAllAxesMask;
        std::array<double, kAxisCount> converted_positions{};
        if (calibration_ready_ && complete_position && !protocol_counts_to_joint_radians(
            decoded.actual_position_counts, joint_zero_offset_counts_,
            joint_direction_signs_, converted_positions))
        {
          ++protocol_rejection_count_;
          RCLCPP_ERROR_ONCE(get_logger(), "Failed to convert 0x200 positions to radians");
          return;
        }
        if (!accepted_sequence(position_sequence_.accept(decoded.sequence))) {
          ++protocol_rejection_count_;
          return;
        }
        const bool first = !has_position_;
        const bool first_calibrated = !has_joint_positions_rad_;
        latest_position_ = decoded;
        if (calibration_ready_ && complete_position) {
          latest_joint_positions_rad_ = converted_positions;
          has_joint_positions_rad_ = true;
          publish_joint_state(latest_joint_positions_rad_);
        }
        last_position_time_ = frame.received_at;
        has_position_ = true;
        if (csp_scheduler_) {
          const bool enabled = decoded.valid_axes_mask == kAllAxesMask &&
            (decoded.status_flags & 0x03U) == 0x03U &&
            (decoded.status_flags & 0x2cU) == 0U;
          csp_scheduler_->set_enabled(enabled);
        }
        if (first) {
          RCLCPP_INFO(
            get_logger(), "First 0x200: mask=0x%02x raw=[%d, %d, %d, %d, %d, %d]",
            static_cast<unsigned>(decoded.valid_axes_mask), decoded.actual_position_counts[0],
            decoded.actual_position_counts[1], decoded.actual_position_counts[2],
            decoded.actual_position_counts[3], decoded.actual_position_counts[4],
            decoded.actual_position_counts[5]);
        }
        if (first_calibrated && has_joint_positions_rad_) {
          RCLCPP_INFO(
            get_logger(),
            "First calibrated joint position [rad]=[%.6f, %.6f, %.6f, %.6f, %.6f, %.6f]",
            latest_joint_positions_rad_[0], latest_joint_positions_rad_[1],
            latest_joint_positions_rad_[2], latest_joint_positions_rad_[3],
            latest_joint_positions_rad_[4], latest_joint_positions_rad_[5]);
        }
        break;
      }
      case kDetailedStatusId: {
        DetailedStatus decoded;
        if (!decode_detailed_status(copy_payload<DetailedStatusFrame>(frame), decoded) ||
          !accepted_sequence(detailed_sequence_.accept(decoded.sequence)))
        {
          ++protocol_rejection_count_;
          return;
        }
        latest_detailed_ = decoded;
        last_detailed_time_ = frame.received_at;
        if (!has_detailed_) {
          RCLCPP_INFO(get_logger(), "First valid 0x201 detailed status received");
        }
        has_detailed_ = true;
        break;
      }
      case kDiagnosticEventId: {
        DiagnosticEvent decoded;
        if (!decode_diagnostic_event(copy_payload<DiagnosticEventFrame>(frame), decoded) ||
          !accepted_sequence(diagnostic_sequence_.accept(decoded.sequence)))
        {
          ++protocol_rejection_count_;
          return;
        }
        latest_diagnostic_ = decoded;
        last_diagnostic_time_ = frame.received_at;
        has_diagnostic_ = true;
        RCLCPP_WARN(
          get_logger(), "0x202 diagnostic: severity=%u axis=%u primary=0x%04x detail=0x%04x",
          static_cast<unsigned>(decoded.severity), static_cast<unsigned>(decoded.source_axis),
          static_cast<unsigned>(decoded.primary_error_code),
          static_cast<unsigned>(decoded.detail_error_code));
        break;
      }
      case kDeviceHeartbeatId: {
        DeviceHeartbeat decoded;
        if (!decode_device_heartbeat(copy_payload<DeviceHeartbeatFrame>(frame), decoded)) {
          ++protocol_rejection_count_;
          return;
        }
        const bool new_device_session = has_heartbeat_ &&
          age_ms(true, last_heartbeat_time_, frame.received_at) > heartbeat_timeout_ms_;
        if (new_device_session) {
          position_sequence_.reset();
          detailed_sequence_.reset();
          diagnostic_sequence_.reset();
          heartbeat_sequence_.reset();
          trajectory_status_sequence_.reset();
          has_position_ = false;
          has_detailed_ = false;
          has_diagnostic_ = false;
          has_trajectory_status_ = false;
        }
        if (!accepted_sequence(heartbeat_sequence_.accept(decoded.sequence))) {
          ++protocol_rejection_count_;
          return;
        }
        latest_heartbeat_ = decoded;
        last_heartbeat_time_ = frame.received_at;
        if (!has_heartbeat_) {
          RCLCPP_INFO(get_logger(), "First valid 0x203 device heartbeat received");
        }
        has_heartbeat_ = true;
        break;
      }
      case kTrajectoryStatusId: {
        TrajectoryStatus decoded;
        if (!decode_trajectory_status(copy_payload<TrajectoryStatusFrame>(frame), decoded) ||
          !accepted_sequence(trajectory_status_sequence_.accept(decoded.sequence)))
        {
          ++protocol_rejection_count_;
          return;
        }
        latest_trajectory_status_ = decoded;
        last_trajectory_status_time_ = frame.received_at;
        has_trajectory_status_ = true;
        if (csp_scheduler_) {
          csp_scheduler_->update_remote_status(decoded, frame.received_at);
        }
        break;
      }
      default:
        ++protocol_rejection_count_;
        break;
    }
  }

  CanFdTransport transport_;
  mutable std::mutex transmit_mutex_;
  std::unique_ptr<CspScheduler> csp_scheduler_;
  rclcpp::TimerBase::SharedPtr receive_timer_;
  rclcpp::TimerBase::SharedPtr diagnostics_timer_;
  rclcpp::TimerBase::SharedPtr heartbeat_timer_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr disable_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr enable_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr hold_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr quick_stop_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr fault_reset_service_;

  SequenceTracker position_sequence_;
  SequenceTracker detailed_sequence_;
  SequenceTracker diagnostic_sequence_;
  SequenceTracker heartbeat_sequence_;
  SequenceTracker trajectory_status_sequence_;

  PositionFeedback latest_position_{};
  DetailedStatus latest_detailed_{};
  DiagnosticEvent latest_diagnostic_{};
  DeviceHeartbeat latest_heartbeat_{};
  TrajectoryStatus latest_trajectory_status_{};
  std::array<double, kAxisCount> latest_joint_positions_rad_{};
  std::chrono::steady_clock::time_point last_position_time_{};
  std::chrono::steady_clock::time_point last_detailed_time_{};
  std::chrono::steady_clock::time_point last_diagnostic_time_{};
  std::chrono::steady_clock::time_point last_heartbeat_time_{};
  std::chrono::steady_clock::time_point last_trajectory_status_time_{};
  bool has_position_{false};
  bool has_detailed_{false};
  bool has_diagnostic_{false};
  bool has_heartbeat_{false};
  bool has_trajectory_status_{false};
  bool has_joint_positions_rad_{false};
  bool bus_off_{false};
  std::array<std::int32_t, kAxisCount> joint_zero_offset_counts_{};
  std::array<std::int8_t, kAxisCount> joint_direction_signs_{};
  bool calibration_ready_{false};
  bool state_initialized_{false};
  LinkState current_state_{LinkState::kDown};
  int position_timeout_ms_{20};
  int detailed_timeout_ms_{100};
  int heartbeat_timeout_ms_{300};
  int trajectory_status_timeout_ms_{100};
  int csp_thread_priority_{kDefaultCspThreadPriority};
  std::chrono::steady_clock::time_point started_at_{std::chrono::steady_clock::now()};
  std::uint8_t heartbeat_tx_sequence_{0};
  std::uint64_t heartbeat_tx_errors_{0};
  std::uint8_t control_tx_sequence_{0};
  std::uint32_t next_control_token_{1};
  std::uint32_t last_control_token_{0};
  MotionControl last_control_command_{MotionControl::kDisable};
  std::chrono::steady_clock::time_point last_control_sent_at_{};
  bool has_sent_control_{false};
  std::uint64_t receive_rejection_count_{0};
  std::uint64_t protocol_rejection_count_{0};
  std::uint64_t joint_state_publications_{0};
};

}  // namespace elfin3_canfd

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<elfin3_canfd::DriverNode>());
  } catch (const std::exception & exception) {
    RCLCPP_FATAL(rclcpp::get_logger("elfin3_canfd_driver"), "%s", exception.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
