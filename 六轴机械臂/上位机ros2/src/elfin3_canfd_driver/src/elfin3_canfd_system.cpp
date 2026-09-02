#include "elfin3_canfd_driver/elfin3_canfd_system.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <utility>

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <linux/can/error.h>
#include <pluginlib/class_list_macros.hpp>

#include "elfin3_canfd_driver/decode.hpp"
#include "elfin3_canfd_driver/encode.hpp"
#include "elfin3_canfd_driver/position_conversion.hpp"

namespace elfin3_canfd
{
namespace
{

const auto kLogger = rclcpp::get_logger("elfin3_canfd_system");

template<typename ValueType>
bool parse_integer_list(
  const std::string & text, std::array<ValueType, kAxisCount> & output)
{
  std::array<ValueType, kAxisCount> parsed{};
  std::stringstream stream(text);
  std::string item;
  std::size_t index = 0;
  try {
    while (std::getline(stream, item, ',')) {
      if (index >= kAxisCount || item.empty()) {
        return false;
      }
      const auto value = std::stoll(item);
      if (value < static_cast<long long>(std::numeric_limits<ValueType>::min()) ||
        value > static_cast<long long>(std::numeric_limits<ValueType>::max()))
      {
        return false;
      }
      parsed[index++] = static_cast<ValueType>(value);
    }
  } catch (const std::exception &) {
    return false;
  }
  if (index != kAxisCount) {
    return false;
  }
  output = parsed;
  return true;
}

bool accepted_sequence(const SequenceResult result)
{
  return result == SequenceResult::kFirst || result == SequenceResult::kAccepted ||
         result == SequenceResult::kAcceptedWithGap;
}

template<typename FrameType>
FrameType copy_payload(const CanFdFrame & frame)
{
  FrameType payload{};
  std::copy_n(frame.data.begin(), payload.size(), payload.begin());
  return payload;
}

std::int64_t age_ms(
  const std::chrono::steady_clock::time_point received,
  const std::chrono::steady_clock::time_point now)
{
  return std::chrono::duration_cast<std::chrono::milliseconds>(now - received).count();
}

const char * trajectory_stream_state_name(const TrajectoryStreamState state)
{
  switch (state) {
    case TrajectoryStreamState::kIdle:
      return "IDLE";
    case TrajectoryStreamState::kStartPending:
      return "START_PENDING";
    case TrajectoryStreamState::kResetAwaitingAck:
      return "RESET_AWAITING_ACK";
    case TrajectoryStreamState::kStreaming:
      return "STREAMING";
    case TrajectoryStreamState::kFinishing:
      return "FINISHING";
    case TrajectoryStreamState::kHolding:
      return "HOLDING";
    case TrajectoryStreamState::kResetRequired:
      return "RESET_REQUIRED";
    case TrajectoryStreamState::kFault:
      return "FAULT";
  }
  return "UNKNOWN";
}

}  // namespace

Elfin3CanFdSystem::~Elfin3CanFdSystem()
{
  stop_backend();
}

Elfin3CanFdSystem::CallbackReturn Elfin3CanFdSystem::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }
  if (info_.joints.size() != kAxisCount) {
    RCLCPP_ERROR(kLogger, "Expected 6 joints, got %zu", info_.joints.size());
    return CallbackReturn::ERROR;
  }
  for (std::size_t joint = 0; joint < kAxisCount; ++joint) {
    const auto & joint_info = info_.joints[joint];
    if (joint_info.name != kJointNames[joint] || joint_info.command_interfaces.size() != 1U ||
      joint_info.command_interfaces[0].name != hardware_interface::HW_IF_POSITION ||
      joint_info.state_interfaces.size() != 1U ||
      joint_info.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_ERROR(kLogger, "Joint %zu has an unexpected name or interface layout", joint);
      return CallbackReturn::ERROR;
    }
    try {
      lower_limits_[joint] = std::stod(joint_info.command_interfaces[0].min);
      upper_limits_[joint] = std::stod(joint_info.command_interfaces[0].max);
    } catch (const std::exception &) {
      RCLCPP_ERROR(kLogger, "Joint %s has invalid position limits", joint_info.name.c_str());
      return CallbackReturn::ERROR;
    }
    if (!std::isfinite(lower_limits_[joint]) || !std::isfinite(upper_limits_[joint]) ||
      lower_limits_[joint] >= upper_limits_[joint])
    {
      RCLCPP_ERROR(kLogger, "Joint %s position limits are invalid", joint_info.name.c_str());
      return CallbackReturn::ERROR;
    }
  }
  if (!parse_hardware_parameters()) {
    return CallbackReturn::ERROR;
  }
  state_positions_.fill(std::numeric_limits<double>::quiet_NaN());
  command_positions_.fill(std::numeric_limits<double>::quiet_NaN());
  return CallbackReturn::SUCCESS;
}

bool Elfin3CanFdSystem::parse_hardware_parameters()
{
  const auto parameter = [this](const std::string & name, const std::string & fallback) {
      const auto found = info_.hardware_parameters.find(name);
      return found == info_.hardware_parameters.end() ? fallback : found->second;
    };
  interface_name_ = parameter("interface_name", "can0");
  try {
    position_warning_timeout_ms_ = std::stoi(
      parameter("position_warning_timeout_ms", "20"));
    position_timeout_ms_ = std::stoi(parameter("position_timeout_ms", "50"));
    communication_error_grace_ms_ = std::stoi(
      parameter("communication_error_grace_ms", "300"));
    detailed_timeout_ms_ = std::stoi(parameter("detailed_timeout_ms", "100"));
    heartbeat_timeout_ms_ = std::stoi(parameter("heartbeat_timeout_ms", "300"));
    trajectory_status_timeout_ms_ = std::stoi(
      parameter("trajectory_status_timeout_ms", "100"));
    activation_timeout_ms_ = std::stoi(parameter("activation_timeout_ms", "3000"));
    csp_period_us_ = std::stoi(
      parameter("csp_period_us", std::to_string(kCspPeriodUs)));
    csp_target_timeout_ms_ = std::stoi(parameter("csp_target_timeout_ms", "100"));
    csp_validity_ms_ = std::stoi(
      parameter("csp_validity_ms", std::to_string(kCspValidityMs)));
    csp_refill_period_us_ = std::stoi(parameter("csp_refill_period_us", "1000"));
    csp_remote_low_watermark_ = std::stoi(
      parameter(
        "csp_remote_low_watermark", std::to_string(kDefaultRemoteLowWatermark)));
    csp_remote_high_watermark_ = std::stoi(
      parameter(
        "csp_remote_high_watermark", std::to_string(kDefaultRemoteHighWatermark)));
    csp_thread_priority_ = std::stoi(
      parameter("csp_thread_priority", std::to_string(kDefaultCspThreadPriority)));
  } catch (const std::exception &) {
    RCLCPP_ERROR(kLogger, "CAN FD hardware timing parameter is not an integer");
    return false;
  }
  if (interface_name_.empty() || position_warning_timeout_ms_ <= 0 ||
    position_warning_timeout_ms_ >= position_timeout_ms_ || position_timeout_ms_ <= 0 ||
    communication_error_grace_ms_ <= 0 || detailed_timeout_ms_ <= 0 ||
    heartbeat_timeout_ms_ <= 0 || trajectory_status_timeout_ms_ <= 0 ||
    activation_timeout_ms_ <= 0 ||
    csp_period_us_ != static_cast<int>(kCspPeriodUs) ||
    csp_target_timeout_ms_ <= 0 ||
    csp_refill_period_us_ <= 0 || csp_refill_period_us_ >= csp_period_us_ ||
    csp_remote_low_watermark_ < 0 ||
    csp_remote_low_watermark_ >= csp_remote_high_watermark_ ||
    csp_remote_high_watermark_ >
    static_cast<int>(kExpectedRemoteQueueCapacity - kRemoteQueueReserve) ||
    csp_thread_priority_ < 0 || csp_thread_priority_ > 99 ||
    csp_validity_ms_ != static_cast<int>(kCspValidityMs))
  {
    RCLCPP_ERROR(
      kLogger, "CAN FD timeouts must be positive, the position warning threshold must be "
      "lower than the motion threshold, and protocol timing must match constants: "
      "csp_period_us=%ld, csp_validity_ms=%u",
      static_cast<long>(kCspPeriodUs), static_cast<unsigned>(kCspValidityMs));
    return false;
  }
  if (!parse_integer_list(parameter("joint_zero_offset_counts", ""), zero_offsets_) ||
    !parse_integer_list(parameter("joint_direction_signs", ""), direction_signs_) ||
    !valid_direction_signs(direction_signs_))
  {
    RCLCPP_ERROR(kLogger, "Six valid joint zero offsets and direction signs are required");
    return false;
  }
  return true;
}

std::vector<hardware_interface::StateInterface>
Elfin3CanFdSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> interfaces;
  interfaces.reserve(kAxisCount);
  for (std::size_t joint = 0; joint < kAxisCount; ++joint) {
    interfaces.emplace_back(
      kJointNames[joint], hardware_interface::HW_IF_POSITION, &state_positions_[joint]);
  }
  return interfaces;
}

/**
 * @brief 导出各关节的位置命令接口
 * @return 命令接口列表
 */
std::vector<hardware_interface::CommandInterface>
Elfin3CanFdSystem::export_command_interfaces()
{
  // 为每个关节创建位置命令接口
  std::vector<hardware_interface::CommandInterface> interfaces;
  interfaces.reserve(kAxisCount);
  for (std::size_t joint = 0; joint < kAxisCount; ++joint) {
    interfaces.emplace_back(
      kJointNames[joint], hardware_interface::HW_IF_POSITION, &command_positions_[joint]);
  }
  return interfaces;
}

Elfin3CanFdSystem::CallbackReturn Elfin3CanFdSystem::on_configure(
  const rclcpp_lifecycle::State &)
{
  return start_backend() ? CallbackReturn::SUCCESS : CallbackReturn::ERROR;
}

Elfin3CanFdSystem::CallbackReturn Elfin3CanFdSystem::on_cleanup(
  const rclcpp_lifecycle::State &)
{
  stop_backend();
  return CallbackReturn::SUCCESS;
}

Elfin3CanFdSystem::CallbackReturn Elfin3CanFdSystem::on_shutdown(
  const rclcpp_lifecycle::State &)
{
  stop_backend();
  return CallbackReturn::SUCCESS;
}

Elfin3CanFdSystem::CallbackReturn Elfin3CanFdSystem::on_activate(
  const rclcpp_lifecycle::State &)
{
  const auto deadline = std::chrono::steady_clock::now() +
    std::chrono::milliseconds(activation_timeout_ms_);
  while (std::chrono::steady_clock::now() < deadline) {
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (communication_ready_locked(std::chrono::steady_clock::now())) {
        state_positions_ = latest_feedback_positions_;
        command_positions_ = state_positions_;
        last_submitted_command_positions_ = command_positions_;
        has_last_submitted_command_ = true;
        stable_command_cycles_ = 0U;
        hardware_write_cycles_.store(0U, std::memory_order_relaxed);
        hardware_command_change_count_.store(0U, std::memory_order_relaxed);
        hardware_last_command_delta_rad_.store(0.0, std::memory_order_relaxed);
        hardware_max_command_delta_rad_.store(0.0, std::memory_order_relaxed);
        hardware_last_command_change_ms_.store(-1, std::memory_order_relaxed);
        hardware_start_attempts_.store(0U, std::memory_order_relaxed);
        hardware_start_successes_.store(0U, std::memory_order_relaxed);
        hardware_start_rejections_.store(0U, std::memory_order_relaxed);
        position_delay_warning_active_.store(false, std::memory_order_relaxed);
        communication_hold_latched_.store(false, std::memory_order_relaxed);
        position_delay_warning_count_.store(0U, std::memory_order_relaxed);
        communication_hold_count_.store(0U, std::memory_order_relaxed);
        communication_fatal_count_.store(0U, std::memory_order_relaxed);
        communication_loss_active_ = false;
        communication_fatal_reported_ = false;
        hardware_active_.store(true);
        RCLCPP_INFO(
          kLogger,
          "CAN FD hardware active in monitoring mode; command initialized from feedback");
        return CallbackReturn::SUCCESS;
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  RCLCPP_ERROR(
    kLogger, "CAN FD hardware activation timed out waiting for fresh 0x200/0x201/0x203");
  return CallbackReturn::ERROR;
}

Elfin3CanFdSystem::CallbackReturn Elfin3CanFdSystem::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  hardware_active_.store(false);
  position_delay_warning_active_.store(false, std::memory_order_relaxed);
  communication_hold_latched_.store(false, std::memory_order_relaxed);
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    communication_loss_active_ = false;
    communication_fatal_reported_ = false;
  }
  has_last_submitted_command_ = false;
  stable_command_cycles_ = 0U;
  if (csp_scheduler_) {
    csp_scheduler_->abort_trajectory();
    csp_scheduler_->set_enabled(false);
    if (!wait_for_scheduler_terminal(std::chrono::milliseconds(20))) {
      RCLCPP_WARN(kLogger, "Timed out waiting for the CSP HOLD while deactivating");
    }
  }
  std::string error;
  std::uint32_t token = 0;
  if (!transmit_control(MotionControl::kHold, error, token)) {
    RCLCPP_ERROR(kLogger, "Failed to send Hold while deactivating: %s", error.c_str());
  }
  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type Elfin3CanFdSystem::read(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  if (!hardware_active_.load()) {
    return hardware_interface::return_type::OK;
  }
  std::lock_guard<std::mutex> lock(state_mutex_);
  const auto now = std::chrono::steady_clock::now();
  const auto position_age = has_position_ ? age_ms(last_position_time_, now) : -1;
  const bool position_delay_warning = has_position_ &&
    position_age > position_warning_timeout_ms_ && position_age <= position_timeout_ms_;
  const bool previous_position_delay_warning =
    position_delay_warning_active_.exchange(position_delay_warning, std::memory_order_relaxed);
  if (position_delay_warning && !previous_position_delay_warning) {
    position_delay_warning_count_.fetch_add(1U, std::memory_order_relaxed);
    RCLCPP_WARN(
      kLogger,
      "Position feedback delay crossed the warning threshold: age=%lld ms, motion remains enabled",
      static_cast<long long>(position_age));
  }

  const bool communication_ready = communication_ready_locked(now);
  const bool motion_ready = motion_ready_locked(now);
  const bool motion_enabled = motion_ready && axes_enabled_locked();
  const bool bus_off = bus_off_.load();
  const bool device_fault = bus_off ||
    (has_position_ && has_detailed_ && fault_present_locked());
  bool communication_loss_started = false;
  std::int64_t communication_loss_age = -1;
  if (!communication_ready) {
    if (!communication_loss_active_) {
      communication_loss_active_ = true;
      communication_fatal_reported_ = false;
      communication_loss_started_at_ = now;
      communication_hold_latched_.store(true, std::memory_order_relaxed);
      communication_hold_count_.fetch_add(1U, std::memory_order_relaxed);
      communication_loss_started = true;
      RCLCPP_WARN(
        kLogger,
        "Communication motion gate closed; CSP HOLD latched and trajectory RESET required "
        "after recovery (position_age=%lld ms)",
        static_cast<long long>(position_age));
    }
    communication_loss_age = age_ms(communication_loss_started_at_, now);
  } else if (communication_loss_active_) {
    communication_loss_age = age_ms(communication_loss_started_at_, now);
    communication_loss_active_ = false;
    communication_fatal_reported_ = false;
    RCLCPP_INFO(
      kLogger,
      "Communication recovered after %lld ms; trajectory RESET remains required",
      static_cast<long long>(communication_loss_age));
  }

  const bool communication_fatal = bus_off ||
    (communication_loss_active_ && communication_loss_age >= communication_error_grace_ms_);
  if (communication_fatal && !communication_fatal_reported_) {
    communication_fatal_reported_ = true;
    communication_fatal_count_.fetch_add(1U, std::memory_order_relaxed);
    if (bus_off) {
      RCLCPP_ERROR(kLogger, "CAN bus-off requires immediate hardware deactivation");
    } else {
      RCLCPP_ERROR(
        kLogger,
        "Communication remained unavailable for %lld ms; deactivating the hardware component",
        static_cast<long long>(communication_loss_age));
    }
  }
  if (!position_delay_warning && previous_position_delay_warning && communication_ready) {
    RCLCPP_INFO(kLogger, "Position feedback delay returned below the warning threshold");
  }
  if (communication_ready) {
    state_positions_ = latest_feedback_positions_;
  }
  if (csp_scheduler_) {
    csp_scheduler_->set_enabled(motion_enabled);
    const auto stream_state = csp_scheduler_->stream_state();
    const bool stream_active = stream_state == TrajectoryStreamState::kStartPending ||
      stream_state == TrajectoryStreamState::kResetAwaitingAck ||
      stream_state == TrajectoryStreamState::kStreaming ||
      stream_state == TrajectoryStreamState::kFinishing;
    if (bus_off) {
      csp_scheduler_->mark_fault();
      stable_command_cycles_ = 0U;
    } else if (communication_loss_started) {
      csp_scheduler_->abort_trajectory();
      stable_command_cycles_ = 0U;
    } else if (!motion_enabled && stream_active) {
      if (device_fault) {
        csp_scheduler_->mark_fault();
      } else {
        csp_scheduler_->abort_trajectory();
      }
      stable_command_cycles_ = 0U;
    }
  }
  return communication_fatal ?
         hardware_interface::return_type::ERROR : hardware_interface::return_type::OK;
}

/**
 * @brief 将关节位置指令写入 CAN FD CSP 调度器
 * @return 指令写入结果
 */
hardware_interface::return_type Elfin3CanFdSystem::write(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  // controller_manager 在控制器 update() 之后调用本函数；此时 command_positions_
  // 已由各关节的 position command interface 写入最新目标弧度值。
  if (!hardware_active_.load()) {
    return hardware_interface::return_type::OK;
  }
  hardware_write_cycles_.fetch_add(1U, std::memory_order_relaxed);

  // 在目标进入 CSP 调度器前，逐轴检查数值有效性和软件关节限位。
  for (std::size_t joint = 0; joint < kAxisCount; ++joint) {
    if (!std::isfinite(command_positions_[joint]) ||
      command_positions_[joint] < lower_limits_[joint] ||
      command_positions_[joint] > upper_limits_[joint])
    {
      RCLCPP_ERROR(kLogger, "Rejected out-of-range command for %s", kJointNames[joint]);
      if (csp_scheduler_) {
        csp_scheduler_->mark_fault();
      }
      stable_command_cycles_ = 0U;
      return hardware_interface::return_type::ERROR;
    }
  }

  if (!csp_scheduler_) {
    RCLCPP_ERROR(kLogger, "CSP scheduler is unavailable");
    return hardware_interface::return_type::ERROR;
  }

  // 通信 HOLD 锁存期间保持 ros2_control 生命周期存活，但不再接收变化中的目标。
  // 只有反馈恢复且轨迹 RESET 成功后，才允许新的 command 重新进入 CSP 调度器。
  if (communication_hold_latched_.load(std::memory_order_relaxed)) {
    stable_command_cycles_ = 0U;
    return hardware_interface::return_type::OK;
  }

  // 比较本周期目标与上次已提交目标；超过启动阈值表示有新的轨迹运动。
  std::array<std::int32_t, kAxisCount> protocol_counts{};
  bool command_changed = false;
  double command_delta_rad = 0.0;
  if (has_last_submitted_command_) {
    for (std::size_t joint = 0; joint < kAxisCount; ++joint) {
      command_delta_rad = std::max(
        command_delta_rad,
        std::abs(command_positions_[joint] - last_submitted_command_positions_[joint]));
    }
    command_changed = command_delta_rad > kTrajectoryStartDeltaRad;
  }
  hardware_last_command_delta_rad_.store(command_delta_rad, std::memory_order_relaxed);
  if (command_delta_rad > hardware_max_command_delta_rad_.load(std::memory_order_relaxed)) {
    hardware_max_command_delta_rad_.store(command_delta_rad, std::memory_order_relaxed);
  }

  // 根据目标变化和当前流状态启动轨迹。异常状态下必须先完成 CSP RESET，
  // 不允许仅凭新的位置指令直接恢复运动。
  const auto stream_state = csp_scheduler_->stream_state();
  const bool stream_blocks_writes = stream_state == TrajectoryStreamState::kFinishing ||
    stream_state == TrajectoryStreamState::kResetRequired ||
    stream_state == TrajectoryStreamState::kFault;
  if (stream_blocks_writes) {
    // 这些状态由 CSP HOLD/RESET 流程恢复，不表示 ros2_control 硬件接口失效。
    // 保持组件 active，同时不接收新目标，也不更新 last_submitted_command_positions_，
    // 确保 RESET 完成后当前 command 仍能被识别为新的轨迹目标。
    stable_command_cycles_ = 0U;
    return hardware_interface::return_type::OK;
  }
  if (command_changed) {
    hardware_command_change_count_.fetch_add(1U, std::memory_order_relaxed);
    hardware_last_command_change_ms_.store(
      std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count(),
      std::memory_order_relaxed);
    stable_command_cycles_ = 0U;
    const bool may_start = stream_state == TrajectoryStreamState::kIdle ||
      stream_state == TrajectoryStreamState::kHolding;
    if (may_start) {
      hardware_start_attempts_.fetch_add(1U, std::memory_order_relaxed);
    }
    if (may_start && !csp_scheduler_->start_trajectory()) {
      hardware_start_rejections_.fetch_add(1U, std::memory_order_relaxed);
      csp_scheduler_->abort_trajectory();
      stable_command_cycles_ = 0U;
      RCLCPP_WARN(
        kLogger,
        "Trajectory start was rejected by the CAN FD motion gate; "
        "CSP RESET is required before motion can resume");
      return hardware_interface::return_type::OK;
    }
    if (may_start) {
      hardware_start_successes_.fetch_add(1U, std::memory_order_relaxed);
    }
    if (!may_start && stream_state != TrajectoryStreamState::kStartPending &&
      stream_state != TrajectoryStreamState::kResetAwaitingAck &&
      stream_state != TrajectoryStreamState::kStreaming)
    {
      stable_command_cycles_ = 0U;
      RCLCPP_WARN(kLogger, "A new trajectory RESET is required before motion can resume");
      return hardware_interface::return_type::OK;
    }
  }

  // 将 ros2_control 使用的关节弧度转换为协议编码器计数，同时应用零偏和方向符号。
  if (!joint_radians_to_protocol_counts(
      command_positions_, zero_offsets_, direction_signs_, protocol_counts))
  {
    csp_scheduler_->mark_fault();
    stable_command_cycles_ = 0U;
    RCLCPP_ERROR(kLogger, "Failed to convert the ros2_control command to encoder counts");
    return hardware_interface::return_type::ERROR;
  }

  // 把六轴目标写入 CSP 本地队列；独立调度线程随后按队列水位和远端状态发送 0x180。
  if (!csp_scheduler_->submit_target(protocol_counts)) {
    csp_scheduler_->abort_trajectory();
    stable_command_cycles_ = 0U;
    RCLCPP_WARN(
      kLogger,
      "Failed to queue the ros2_control command for CSP transmission; "
      "CSP RESET is required before motion can resume");
    return hardware_interface::return_type::OK;
  }

  // 记录成功提交的弧度目标，供下一控制周期判断目标是否发生变化。
  last_submitted_command_positions_ = command_positions_;
  has_last_submitted_command_ = true;

  // 目标连续稳定且六轴实际位置均进入容差后，通知调度器结束 APPLY 流并转入 HOLD。
  if (csp_scheduler_->stream_state() == TrajectoryStreamState::kStreaming) {
    if (!command_changed &&
      stable_command_cycles_.load() < kTrajectoryFinishStableCycles)
    {
      stable_command_cycles_.fetch_add(1U);
    }
    bool goal_reached = true;
    for (std::size_t joint = 0; joint < kAxisCount; ++joint) {
      if (!std::isfinite(state_positions_[joint]) ||
        std::abs(state_positions_[joint] - command_positions_[joint]) >
        kTrajectoryGoalToleranceRad)
      {
        goal_reached = false;
        break;
      }
    }
    if (stable_command_cycles_.load() >= kTrajectoryFinishStableCycles && goal_reached) {
      if (!csp_scheduler_->finish_trajectory()) {
        stable_command_cycles_ = 0U;
        RCLCPP_WARN(
          kLogger,
          "The CSP trajectory stream changed state before finish; "
          "keeping the hardware component active for recovery");
        return hardware_interface::return_type::OK;
      }
      stable_command_cycles_ = 0U;
    }
  } else {
    stable_command_cycles_ = 0U;
  }
  return hardware_interface::return_type::OK;
}

bool Elfin3CanFdSystem::start_backend()
{
  if (transport_.is_open()) {
    return true;
  }
  std::string error;
  if (!transport_.open(interface_name_, error)) {
    RCLCPP_ERROR(kLogger, "Failed to open %s: %s", interface_name_.c_str(), error.c_str());
    return false;
  }
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    has_position_ = false;
    has_detailed_ = false;
    has_heartbeat_ = false;
    has_trajectory_status_ = false;
    communication_loss_active_ = false;
    communication_fatal_reported_ = false;
  }
  position_delay_warning_active_.store(false, std::memory_order_relaxed);
  communication_hold_latched_.store(false, std::memory_order_relaxed);
  position_sequence_ = SequenceTracker{};
  detailed_sequence_ = SequenceTracker{};
  diagnostic_sequence_ = SequenceTracker{};
  heartbeat_sequence_ = SequenceTracker{};
  trajectory_status_sequence_ = SequenceTracker{};
  bus_off_.store(false);
  csp_scheduler_ = std::make_unique<CspScheduler>(
    [this](
      const std::uint32_t id, const std::uint8_t * data, const std::size_t size,
      std::string & send_error)
    {
      return send_frame(id, data, size, send_error);
    },
    std::chrono::microseconds(csp_period_us_),
    std::chrono::milliseconds(csp_target_timeout_ms_),
    static_cast<std::uint16_t>(csp_validity_ms_), kDefaultCspTargetQueueCapacity,
    static_cast<std::size_t>(csp_remote_low_watermark_),
    static_cast<std::size_t>(csp_remote_high_watermark_),
    std::chrono::microseconds(csp_refill_period_us_), csp_thread_priority_);
  csp_scheduler_->start();
  receive_running_.store(true);
  receive_thread_ = std::thread([this]() {receive_loop();});
  create_management_node();
  RCLCPP_INFO(kLogger, "CAN FD hardware configured on %s", interface_name_.c_str());
  return true;
}

void Elfin3CanFdSystem::stop_backend()
{
  hardware_active_.store(false);
  if (csp_scheduler_) {
    csp_scheduler_->abort_trajectory();
    csp_scheduler_->set_enabled(false);
    if (transport_.is_open() &&
      !wait_for_scheduler_terminal(std::chrono::milliseconds(20)))
    {
      RCLCPP_WARN(kLogger, "Timed out waiting for the CSP HOLD before backend shutdown");
    }
    csp_scheduler_->stop();
  }
  destroy_management_node();
  if (transport_.is_open()) {
    std::string error;
    std::uint32_t token = 0;
    transmit_control(MotionControl::kHold, error, token);
    send_host_heartbeat(HostState::kShuttingDown);
  }
  receive_running_.store(false);
  if (receive_thread_.joinable()) {
    receive_thread_.join();
  }
  transport_.close();
  csp_scheduler_.reset();
}

bool Elfin3CanFdSystem::wait_for_scheduler_terminal(
  const std::chrono::milliseconds timeout) const
{
  if (!csp_scheduler_) {
    return true;
  }
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    const auto state = csp_scheduler_->stream_state();
    if (state == TrajectoryStreamState::kIdle ||
      state == TrajectoryStreamState::kHolding ||
      state == TrajectoryStreamState::kResetRequired ||
      state == TrajectoryStreamState::kFault)
    {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  return false;
}

void Elfin3CanFdSystem::receive_loop()
{
  while (receive_running_.load()) {
    bool received_any = false;
    for (std::size_t count = 0; count < 128U; ++count) {
      CanFdFrame frame;
      std::uint32_t can_error = 0;
      std::string error;
      const auto result = transport_.receive(frame, can_error, error);
      if (result == ReceiveResult::kNoData) {
        break;
      }
      received_any = true;
      if (result == ReceiveResult::kCanError) {
        if ((can_error & CAN_ERR_BUSOFF) != 0U) {
          bus_off_.store(true);
        }
        continue;
      }
      if (result == ReceiveResult::kFrame) {
        bus_off_.store(false);
        dispatch(frame);
      }
    }
    if (!received_any) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }
}

void Elfin3CanFdSystem::dispatch(const CanFdFrame & frame)
{
  if (frame.id == kPositionFeedbackId) {
    PositionFeedback decoded;
    std::array<double, kAxisCount> positions{};
    if (!decode_position_feedback(copy_payload<PositionFeedbackFrame>(frame), decoded) ||
      decoded.valid_axes_mask != kAllAxesMask ||
      !protocol_counts_to_joint_radians(
        decoded.actual_position_counts, zero_offsets_, direction_signs_, positions) ||
      !accepted_sequence(position_sequence_.accept(decoded.sequence)))
    {
      return;
    }
    std::lock_guard<std::mutex> lock(state_mutex_);
    latest_position_ = decoded;
    latest_feedback_positions_ = positions;
    last_position_time_ = frame.received_at;
    has_position_ = true;
    return;
  }
  if (frame.id == kDetailedStatusId) {
    DetailedStatus decoded;
    if (!decode_detailed_status(copy_payload<DetailedStatusFrame>(frame), decoded) ||
      decoded.valid_axes_mask != kAllAxesMask ||
      !accepted_sequence(detailed_sequence_.accept(decoded.sequence)))
    {
      return;
    }
    std::lock_guard<std::mutex> lock(state_mutex_);
    latest_detailed_ = decoded;
    last_detailed_time_ = frame.received_at;
    has_detailed_ = true;
    return;
  }
  if (frame.id == kDiagnosticEventId) {
    DiagnosticEvent decoded;
    if (decode_diagnostic_event(copy_payload<DiagnosticEventFrame>(frame), decoded) &&
      accepted_sequence(diagnostic_sequence_.accept(decoded.sequence)))
    {
      RCLCPP_WARN(
        kLogger, "STM32 diagnostic severity=%u axis=%u primary=0x%04x detail=0x%04x",
        static_cast<unsigned>(decoded.severity), static_cast<unsigned>(decoded.source_axis),
        static_cast<unsigned>(decoded.primary_error_code),
        static_cast<unsigned>(decoded.detail_error_code));
    }
    return;
  }
  if (frame.id == kDeviceHeartbeatId) {
    DeviceHeartbeat decoded;
    if (!decode_device_heartbeat(copy_payload<DeviceHeartbeatFrame>(frame), decoded)) {
      return;
    }
    bool new_device_session = false;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      new_device_session = has_heartbeat_ &&
        age_ms(last_heartbeat_time_, frame.received_at) > heartbeat_timeout_ms_;
      if (new_device_session) {
        has_position_ = false;
        has_detailed_ = false;
        has_trajectory_status_ = false;
      }
    }
    if (new_device_session) {
      position_sequence_.reset();
      detailed_sequence_.reset();
      diagnostic_sequence_.reset();
      heartbeat_sequence_.reset();
      trajectory_status_sequence_.reset();
    }
    if (!accepted_sequence(heartbeat_sequence_.accept(decoded.sequence))) {
      return;
    }
    std::lock_guard<std::mutex> lock(state_mutex_);
    latest_heartbeat_ = decoded;
    last_heartbeat_time_ = frame.received_at;
    has_heartbeat_ = true;
    return;
  }
  if (frame.id == kTrajectoryStatusId) {
    TrajectoryStatus decoded;
    if (!decode_trajectory_status(copy_payload<TrajectoryStatusFrame>(frame), decoded) ||
      !accepted_sequence(trajectory_status_sequence_.accept(decoded.sequence)))
    {
      return;
    }
    std::lock_guard<std::mutex> lock(state_mutex_);
    latest_trajectory_status_ = decoded;
    last_trajectory_status_time_ = frame.received_at;
    has_trajectory_status_ = true;
    if (csp_scheduler_) {
      csp_scheduler_->update_remote_status(decoded, frame.received_at);
    }
  }
}

bool Elfin3CanFdSystem::fault_present_locked() const
{
  if ((latest_position_.status_flags & 0x20U) != 0U) {
    return true;
  }
  for (std::size_t axis = 0; axis < kAxisCount; ++axis) {
    if (latest_detailed_.drive_error_code[axis] != 0U) {
      return true;
    }
  }
  return false;
}

bool Elfin3CanFdSystem::communication_ready_locked(
  const std::chrono::steady_clock::time_point now) const
{
  return !bus_off_.load() && 
          has_position_ && has_detailed_ && 
          has_heartbeat_ &&
          age_ms(last_position_time_, now) <= position_timeout_ms_ &&
          age_ms(last_detailed_time_, now) <= detailed_timeout_ms_ &&
          age_ms(last_heartbeat_time_, now) <= heartbeat_timeout_ms_;
}

bool Elfin3CanFdSystem::motion_ready_locked(
  const std::chrono::steady_clock::time_point now) const
{
  return communication_ready_locked(now) &&
         has_trajectory_status_ &&
         age_ms(last_trajectory_status_time_, now) <= trajectory_status_timeout_ms_ &&
         (latest_trajectory_status_.flags &
         static_cast<std::uint8_t>(kSafetyHoldLatched | kQuickStopLatched)) == 0U &&
         latest_position_.valid_axes_mask == kAllAxesMask &&
         latest_detailed_.valid_axes_mask == kAllAxesMask &&
         latest_heartbeat_.slave_op_mask == 0x07U &&
         (latest_position_.status_flags & 0x01U) != 0U &&
         (latest_position_.status_flags & 0x0cU) == 0U && !fault_present_locked();
}

bool Elfin3CanFdSystem::axes_enabled_locked() const
{
  return (latest_position_.status_flags & 0x02U) != 0U;
}

bool Elfin3CanFdSystem::send_frame(
  const std::uint32_t id, const std::uint8_t * data, const std::size_t size,
  std::string & error)
{
  std::lock_guard<std::mutex> lock(transmit_mutex_);
  return transport_.send(id, data, size, error);
}

bool Elfin3CanFdSystem::transmit_control(
  const MotionControl command, std::string & error, std::uint32_t & token)
{
  MotionControlCommand request;
  request.sequence = control_sequence_.fetch_add(1);
  request.command = command;
  request.axes_mask = kAllAxesMask;
  request.request_token = next_control_token_.fetch_add(1);
  request.host_time_ms = static_cast<std::uint32_t>(
    std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::steady_clock::now() - started_at_).count());
  MotionControlFrame frame{};
  if (!encode_motion_control(request, frame) ||
    !send_frame(kMotionControlId, frame.data(), frame.size(), error))
  {
    return false;
  }
  token = request.request_token;
  return true;
}

HostState Elfin3CanFdSystem::heartbeat_state() const
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  if (bus_off_.load() || (has_position_ && has_detailed_ && fault_present_locked())) {
    return HostState::kFault;
  }
  if (!motion_ready_locked(std::chrono::steady_clock::now())) {
    return HostState::kNotReady;
  }
  return csp_scheduler_ && csp_scheduler_->is_applying() ?
         HostState::kMoving : HostState::kReady;
}

void Elfin3CanFdSystem::send_host_heartbeat(const HostState state)
{
  HostHeartbeat heartbeat;
  heartbeat.sequence = heartbeat_tx_sequence_++;
  heartbeat.state = state;
  heartbeat.flags = kHostFlagTrajectoryStatusSupported;
  heartbeat.uptime_100ms = static_cast<std::uint16_t>(
    std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::steady_clock::now() - started_at_).count() / 100);
  HostHeartbeatFrame frame{};
  std::string error;
  if (!encode_host_heartbeat(heartbeat, frame) ||
    !send_frame(kHostHeartbeatId, frame.data(), frame.size(), error))
  {
    RCLCPP_ERROR(kLogger, "Failed to send host heartbeat: %s", error.c_str());
  }
}

void Elfin3CanFdSystem::handle_control_request(
  const MotionControl command, std_srvs::srv::Trigger::Response & response)
{
  if (csp_scheduler_ &&
    (command == MotionControl::kDisable || command == MotionControl::kHold ||
    command == MotionControl::kQuickStop))
  {
    csp_scheduler_->abort_trajectory();
    stable_command_cycles_ = 0U;
  }
  if (command == MotionControl::kEnable) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (!motion_ready_locked(std::chrono::steady_clock::now())) {
      response.success = false;
      response.message =
        "Enable rejected: EtherCAT is not OP or a timeout, Quick Stop, or fault is active";
      return;
    }
  }
  if (command == MotionControl::kFaultReset) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (!has_heartbeat_ ||
      age_ms(last_heartbeat_time_, std::chrono::steady_clock::now()) > heartbeat_timeout_ms_)
    {
      response.success = false;
      response.message = "Fault Reset rejected: STM32 heartbeat is stale";
      return;
    }
  }
  std::string error;
  std::uint32_t token = 0;
  response.success = transmit_control(command, error, token);
  response.message = response.success ?
    "0x181 sent with token " + std::to_string(token) +
    "; confirm execution from feedback" : "0x181 transmit failed: " + error;
}

void Elfin3CanFdSystem::handle_trajectory_reset(
  std_srvs::srv::Trigger::Response & response)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  const auto reject = [&response](const std::string & reason) {
      response.success = false;
      response.message = "Trajectory RESET rejected: " + reason;
    };
  const auto now = std::chrono::steady_clock::now();
  if (!hardware_active_.load()) {
    reject("hardware component is not active");
    return;
  }
  if (!csp_scheduler_) {
    reject("CSP scheduler is unavailable");
    return;
  }
  if (bus_off_.load()) {
    reject("CAN bus is BUS_OFF");
    return;
  }
  if (!has_position_) {
    reject("position feedback has not been received");
    return;
  }
  const auto position_age = age_ms(last_position_time_, now);
  if (position_age > position_timeout_ms_) {
    reject(
      "position feedback is stale (age=" + std::to_string(position_age) +
      " ms, limit=" + std::to_string(position_timeout_ms_) + " ms)");
    return;
  }
  if (!has_detailed_) {
    reject("detailed drive feedback has not been received");
    return;
  }
  const auto detailed_age = age_ms(last_detailed_time_, now);
  if (detailed_age > detailed_timeout_ms_) {
    reject(
      "detailed drive feedback is stale (age=" + std::to_string(detailed_age) +
      " ms, limit=" + std::to_string(detailed_timeout_ms_) + " ms)");
    return;
  }
  if (!has_heartbeat_) {
    reject("STM32 heartbeat has not been received");
    return;
  }
  const auto heartbeat_age = age_ms(last_heartbeat_time_, now);
  if (heartbeat_age > heartbeat_timeout_ms_) {
    reject(
      "STM32 heartbeat is stale (age=" + std::to_string(heartbeat_age) +
      " ms, limit=" + std::to_string(heartbeat_timeout_ms_) + " ms)");
    return;
  }
  if (!has_trajectory_status_) {
    reject("trajectory status has not been received");
    return;
  }
  const auto trajectory_status_age = age_ms(last_trajectory_status_time_, now);
  if (trajectory_status_age > trajectory_status_timeout_ms_) {
    reject(
      "trajectory status is stale (age=" + std::to_string(trajectory_status_age) +
      " ms, limit=" + std::to_string(trajectory_status_timeout_ms_) + " ms)");
    return;
  }
  if ((latest_trajectory_status_.flags & kSafetyHoldLatched) != 0U) {
    reject("remote trajectory safety HOLD is latched");
    return;
  }
  if ((latest_trajectory_status_.flags & kQuickStopLatched) != 0U) {
    reject("remote trajectory Quick Stop is latched");
    return;
  }
  if (latest_position_.valid_axes_mask != kAllAxesMask) {
    reject(
      "position feedback does not contain all six axes (mask=" +
      std::to_string(latest_position_.valid_axes_mask) + ")");
    return;
  }
  if (latest_detailed_.valid_axes_mask != kAllAxesMask) {
    reject(
      "detailed drive feedback does not contain all six axes (mask=" +
      std::to_string(latest_detailed_.valid_axes_mask) + ")");
    return;
  }
  if (latest_heartbeat_.slave_op_mask != 0x07U) {
    reject(
      "not all EtherCAT slaves are OP (slave_op_mask=" +
      std::to_string(latest_heartbeat_.slave_op_mask) + ")");
    return;
  }
  if ((latest_position_.status_flags & 0x01U) == 0U) {
    reject("EtherCAT operational flag is not set in position feedback");
    return;
  }
  if ((latest_position_.status_flags & 0x0cU) != 0U) {
    reject(
      "position feedback reports a safety or Quick Stop condition (status_flags=" +
      std::to_string(latest_position_.status_flags) + ")");
    return;
  }
  if (fault_present_locked()) {
    reject("a drive or controller fault is present");
    return;
  }
  if (!axes_enabled_locked()) {
    reject("one or more axes are not enabled");
    return;
  }
  if (!motion_ready_locked(now)) {
    reject("motion readiness gate is closed by an unclassified condition");
    return;
  }

  const auto recovery_result = csp_scheduler_->recover_trajectory();
  if (recovery_result == TrajectoryRecoveryResult::kMotionGateClosed) {
    reject("CSP scheduler motion gate is closed; wait for the next hardware read cycle");
    return;
  }
  if (recovery_result == TrajectoryRecoveryResult::kStateNotRecoverable) {
    const auto state = csp_scheduler_->stream_state();
    reject(
      "CSP stream state is " + std::string(trajectory_stream_state_name(state)) +
      " (" + std::to_string(static_cast<unsigned>(state)) +
      "); expected RESET_REQUIRED or FAULT");
    return;
  }
  if (recovery_result != TrajectoryRecoveryResult::kSuccess) {
    reject("CSP scheduler returned an unknown recovery result");
    return;
  }

  communication_hold_latched_.store(false, std::memory_order_relaxed);
  response.success = true;
  response.message = "New CSP trajectory RESET armed";
}

void Elfin3CanFdSystem::create_management_node()
{
  rclcpp::NodeOptions options;
  options.use_global_arguments(false);
  management_node_ = std::make_shared<rclcpp::Node>("elfin3_canfd_hardware", options);
  const auto service = [this](const char * name, const MotionControl command) {
      return management_node_->create_service<std_srvs::srv::Trigger>(
        name,
        [this, command](
          const std::shared_ptr<std_srvs::srv::Trigger::Request>,
          std::shared_ptr<std_srvs::srv::Trigger::Response> response)
        {
          handle_control_request(command, *response);
        });
    };
  disable_service_ = service("/elfin3_canfd/disable", MotionControl::kDisable);
  enable_service_ = service("/elfin3_canfd/enable", MotionControl::kEnable);
  hold_service_ = service("/elfin3_canfd/hold", MotionControl::kHold);
  quick_stop_service_ = service("/elfin3_canfd/quick_stop", MotionControl::kQuickStop);
  fault_reset_service_ = service("/elfin3_canfd/fault_reset", MotionControl::kFaultReset);
  trajectory_reset_service_ =
    management_node_->create_service<std_srvs::srv::Trigger>(
    "/elfin3_canfd/reset_trajectory",
    [this](
      const std::shared_ptr<std_srvs::srv::Trigger::Request>,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
      handle_trajectory_reset(*response);
    });
  diagnostics_publisher_ =
    management_node_->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
    "/diagnostics", rclcpp::QoS(10));
  trajectory_fault_publisher_ =
    management_node_->create_publisher<std_msgs::msg::String>(
    "/elfin3_canfd/trajectory_fault", rclcpp::QoS(1).reliable().transient_local());
  heartbeat_timer_ = management_node_->create_wall_timer(
    std::chrono::milliseconds(100), [this]() {send_host_heartbeat(heartbeat_state());});
  diagnostics_timer_ = management_node_->create_wall_timer(
    std::chrono::seconds(1), [this]() {publish_diagnostics();});
  trajectory_fault_timer_ = management_node_->create_wall_timer(
    std::chrono::milliseconds(10), [this]() {publish_trajectory_fault_if_needed();});
  management_executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  management_executor_->add_node(management_node_);
  management_thread_ = std::thread([this]() {management_executor_->spin();});
}

void Elfin3CanFdSystem::destroy_management_node()
{
  if (management_executor_) {
    management_executor_->cancel();
  }
  if (management_thread_.joinable()) {
    management_thread_.join();
  }
  management_executor_.reset();
  heartbeat_timer_.reset();
  diagnostics_timer_.reset();
  trajectory_fault_timer_.reset();
  diagnostics_publisher_.reset();
  trajectory_fault_publisher_.reset();
  disable_service_.reset();
  enable_service_.reset();
  hold_service_.reset();
  quick_stop_service_.reset();
  fault_reset_service_.reset();
  trajectory_reset_service_.reset();
  management_node_.reset();
  trajectory_fault_published_ = false;
}

void Elfin3CanFdSystem::publish_trajectory_fault_if_needed()
{
  if (!csp_scheduler_ || !trajectory_fault_publisher_) {
    return;
  }
  const auto stream_state = csp_scheduler_->stream_state();
  const bool faulted = stream_state == TrajectoryStreamState::kResetRequired ||
    stream_state == TrajectoryStreamState::kFault;
  if (!faulted) {
    if (trajectory_fault_published_) {
      std_msgs::msg::String clear;
      clear.data = "CLEAR";
      trajectory_fault_publisher_->publish(clear);
    }
    trajectory_fault_published_ = false;
    return;
  }
  if (trajectory_fault_published_) {
    return;
  }

  TrajectoryStatus remote_status;
  std::chrono::steady_clock::time_point received_at;
  std_msgs::msg::String message;
  message.data = "CAN FD trajectory stream state=" +
    std::to_string(static_cast<unsigned>(stream_state));
  if (csp_scheduler_->get_remote_status(remote_status, received_at)) {
    message.data +=
      " [remote_state=" +
      std::to_string(static_cast<unsigned>(remote_status.state)) +
      ", flags=" + std::to_string(static_cast<unsigned>(remote_status.flags)) +
      ", hold_reason=" +
      std::to_string(static_cast<unsigned>(remote_status.hold_reason)) +
      ", reject_reason=" +
      std::to_string(static_cast<unsigned>(remote_status.reject_reason)) +
      ", underrun_count=" + std::to_string(remote_status.underrun_count) + "]";
  }
  trajectory_fault_publisher_->publish(message);
  trajectory_fault_published_ = true;
}

void Elfin3CanFdSystem::publish_diagnostics()
{
  const auto now = std::chrono::steady_clock::now();
  bool communication_ready = false;
  bool motion_ready = false;
  bool enabled = false;
  bool fault = false;
  std::int64_t position_age = -1;
  std::int64_t detailed_age = -1;
  std::int64_t heartbeat_age = -1;
  std::int64_t trajectory_status_age = -1;
  bool has_trajectory_status = false;
  bool communication_loss_active = false;
  std::int64_t communication_loss_age = -1;
  TrajectoryStatus trajectory_status;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    communication_ready = communication_ready_locked(now);
    motion_ready = motion_ready_locked(now);
    enabled = motion_ready && axes_enabled_locked();
    fault = bus_off_.load() || (has_position_ && has_detailed_ && fault_present_locked());
    if (has_position_) {
      position_age = age_ms(last_position_time_, now);
    }
    if (has_detailed_) {
      detailed_age = age_ms(last_detailed_time_, now);
    }
    if (has_heartbeat_) {
      heartbeat_age = age_ms(last_heartbeat_time_, now);
    }
    if (has_trajectory_status_) {
      trajectory_status_age = age_ms(last_trajectory_status_time_, now);
      trajectory_status = latest_trajectory_status_;
      has_trajectory_status = true;
    }
    communication_loss_active = communication_loss_active_;
    if (communication_loss_active_) {
      communication_loss_age = age_ms(communication_loss_started_at_, now);
    }
  }

  const bool applying = csp_scheduler_ && csp_scheduler_->is_applying();
  const bool position_delay_warning =
    position_delay_warning_active_.load(std::memory_order_relaxed);
  const bool communication_hold_latched =
    communication_hold_latched_.load(std::memory_order_relaxed);
  const bool communication_fatal = communication_loss_active &&
    communication_loss_age >= communication_error_grace_ms_;
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = "elfin3_canfd_system/communication";
  status.hardware_id = "elfin3";
  if (fault) {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    status.message = bus_off_.load() ? "BUS_OFF" : "FAULT";
  } else if (communication_fatal) {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    status.message = "COMMUNICATION_FATAL";
  } else if (communication_loss_active) {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    status.message = "COMMUNICATION_HOLD";
  } else if (communication_hold_latched) {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    status.message = "COMMUNICATION_RESET_REQUIRED";
  } else if (position_delay_warning) {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    status.message = "POSITION_FEEDBACK_DELAYED";
  } else if (!communication_ready) {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    status.message = "COMMUNICATION_NOT_READY";
  } else if (!motion_ready) {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    status.message = "MOTION_INHIBITED";
  } else {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    status.message = applying ? "OPERATIONAL" : (enabled ? "ENABLED" : "READY");
  }
  const auto add = [&status](const std::string & key, const std::string & value) {
      diagnostic_msgs::msg::KeyValue item;
      item.key = key;
      item.value = value;
      status.values.push_back(std::move(item));
    };
  const auto format_double = [](const double value) {
    std::ostringstream stream;
    stream.precision(12);
    stream << value;
    return stream.str();
  };
  add("position_age_ms", std::to_string(position_age));
  add("position_warning_timeout_ms", std::to_string(position_warning_timeout_ms_));
  add("position_motion_timeout_ms", std::to_string(position_timeout_ms_));
  add("communication_error_grace_ms", std::to_string(communication_error_grace_ms_));
  add("position_delay_warning_active", position_delay_warning ? "true" : "false");
  add("communication_loss_active", communication_loss_active ? "true" : "false");
  add("communication_loss_age_ms",
    communication_loss_active ? std::to_string(communication_loss_age) : "unavailable");
  add("communication_hold_latched", communication_hold_latched ? "true" : "false");
  add("position_delay_warning_count", std::to_string(
      position_delay_warning_count_.load(std::memory_order_relaxed)));
  add("communication_hold_count", std::to_string(
      communication_hold_count_.load(std::memory_order_relaxed)));
  add("communication_fatal_count", std::to_string(
      communication_fatal_count_.load(std::memory_order_relaxed)));
  add("detailed_age_ms", std::to_string(detailed_age));
  add("heartbeat_age_ms", std::to_string(heartbeat_age));
  add("trajectory_status_age_ms", std::to_string(trajectory_status_age));
  add("trajectory_status_received", has_trajectory_status ? "true" : "false");
  if (has_trajectory_status) {
    add("trajectory_state", std::to_string(static_cast<unsigned>(trajectory_status.state)));
    add("trajectory_queue_depth", std::to_string(trajectory_status.queue_depth));
    add("trajectory_queue_capacity", std::to_string(trajectory_status.queue_capacity));
    add("trajectory_prefill_target", std::to_string(trajectory_status.prefill_target));
    add("trajectory_queue_contract_ok",
      trajectory_status.queue_capacity == kExpectedRemoteQueueCapacity &&
      trajectory_status.prefill_target == kExpectedRemotePrefillTarget ? "true" : "false");
    add("trajectory_flags", std::to_string(trajectory_status.flags));
    add("trajectory_reset_required",
      (trajectory_status.flags & kTrajectoryResetRequired) != 0U ? "true" : "false");
    add("trajectory_safety_hold_latched",
      (trajectory_status.flags & kSafetyHoldLatched) != 0U ? "true" : "false");
    add("trajectory_quick_stop_latched",
      (trajectory_status.flags & kQuickStopLatched) != 0U ? "true" : "false");
    add("trajectory_generation", std::to_string(trajectory_status.generation));
    add("trajectory_last_executed_ecat_cycle", std::to_string(
        trajectory_status.last_executed_ecat_cycle));
    add("trajectory_last_received_sequence", std::to_string(
        trajectory_status.last_received_sequence));
    add("trajectory_last_accepted_sequence", std::to_string(
        trajectory_status.last_accepted_sequence));
    add("trajectory_last_executed_sequence", std::to_string(
        trajectory_status.last_executed_sequence));
    add("trajectory_expected_sequence", std::to_string(
        trajectory_status.expected_sequence));
    add("trajectory_last_rejected_sequence", std::to_string(
        trajectory_status.last_rejected_sequence));
    add("trajectory_reject_reason", std::to_string(
        static_cast<unsigned>(trajectory_status.reject_reason)));
    add("trajectory_hold_reason", std::to_string(
        static_cast<unsigned>(trajectory_status.hold_reason)));
    add("trajectory_accepted_count", std::to_string(trajectory_status.accepted_count));
    add("trajectory_executed_count", std::to_string(trajectory_status.executed_count));
    add("trajectory_rejected_count", std::to_string(trajectory_status.rejected_count));
    add("trajectory_underrun_count", std::to_string(trajectory_status.underrun_count));
    add("trajectory_overflow_count", std::to_string(trajectory_status.overflow_count));
    add("trajectory_expired_count", std::to_string(trajectory_status.expired_count));
  }
  add("hardware_active", hardware_active_.load() ? "true" : "false");
  add("communication_ready", communication_ready ? "true" : "false");
  add("motion_ready", motion_ready ? "true" : "false");
  add("axes_enabled", enabled ? "true" : "false");
  add("hardware_write_cycles", std::to_string(
      hardware_write_cycles_.load(std::memory_order_relaxed)));
  add("hardware_command_change_count", std::to_string(
      hardware_command_change_count_.load(std::memory_order_relaxed)));
  add("hardware_last_command_delta_rad", format_double(
      hardware_last_command_delta_rad_.load(std::memory_order_relaxed)));
  add("hardware_max_command_delta_rad", format_double(
      hardware_max_command_delta_rad_.load(std::memory_order_relaxed)));
  const auto last_command_change_ms =
    hardware_last_command_change_ms_.load(std::memory_order_relaxed);
  const auto now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
    now.time_since_epoch()).count();
  add("hardware_last_command_change_age_ms",
    last_command_change_ms < 0 ? "unavailable" :
    std::to_string(std::max<std::int64_t>(0, now_ms - last_command_change_ms)));
  add("hardware_start_attempts", std::to_string(
      hardware_start_attempts_.load(std::memory_order_relaxed)));
  add("hardware_start_successes", std::to_string(
      hardware_start_successes_.load(std::memory_order_relaxed)));
  add("hardware_start_rejections", std::to_string(
      hardware_start_rejections_.load(std::memory_order_relaxed)));
  if (csp_scheduler_) {
    const auto stats = csp_scheduler_->stats();
    add("csp_stream_state", std::to_string(
        static_cast<unsigned>(csp_scheduler_->stream_state())));
    add("csp_remote_low_watermark", std::to_string(csp_remote_low_watermark_));
    add("csp_remote_high_watermark", std::to_string(csp_remote_high_watermark_));
    add("csp_remote_queue_reserve", std::to_string(kRemoteQueueReserve));
    add("csp_validity_ms", std::to_string(csp_validity_ms_));
    add("csp_local_queue_depth", std::to_string(csp_scheduler_->queued_target_count()));
    add("csp_start_requests", std::to_string(stats.start_requests));
    add("csp_start_successes", std::to_string(stats.start_successes));
    add("csp_start_rejections", std::to_string(stats.start_rejections));
    add("csp_start_pending_cycles", std::to_string(stats.start_pending_cycles));
    add("csp_start_wait_enabled_cycles", std::to_string(
        stats.start_wait_enabled_cycles));
    add("csp_start_wait_prefill_cycles", std::to_string(
        stats.start_wait_prefill_cycles));
    add("csp_start_wait_fresh_point_cycles", std::to_string(
        stats.start_wait_fresh_point_cycles));
    add("csp_start_wait_remote_cycles", std::to_string(
        stats.start_wait_remote_cycles));
    add("csp_reset_apply_frames", std::to_string(stats.reset_apply_frames));
    add(
      "csp_start_gate_sample_valid",
      stats.start_gate_sample_valid ? "true" : "false");
    add("csp_start_last_enabled", stats.start_last_enabled ? "true" : "false");
    add(
      "csp_start_last_prefill_ready",
      stats.start_last_prefill_ready ? "true" : "false");
    add(
      "csp_start_last_point_available",
      stats.start_last_point_available ? "true" : "false");
    add(
      "csp_start_last_point_fresh",
      stats.start_last_point_fresh ? "true" : "false");
    add(
      "csp_start_last_remote_ready",
      stats.start_last_remote_ready ? "true" : "false");
    add(
      "csp_start_last_local_queue_depth",
      std::to_string(stats.start_last_local_queue_depth));
    add("csp_apply_frames", std::to_string(stats.apply_frames));
    add("csp_hold_frames", std::to_string(stats.hold_frames));
    add("csp_tx_errors", std::to_string(stats.transmit_errors));
    add("csp_overruns", std::to_string(stats.overruns));
    add("csp_queue_overflows", std::to_string(stats.queue_overflows));
    add("csp_queue_underruns", std::to_string(stats.queue_underruns));
    add("csp_credit_stalls", std::to_string(stats.credit_stalls));
    add("csp_high_watermark_stalls", std::to_string(stats.high_watermark_stalls));
    add("csp_refill_cycles", std::to_string(stats.refill_cycles));
    add("csp_refill_transmissions", std::to_string(stats.refill_transmissions));
    add("csp_filler_frames", std::to_string(stats.filler_frames));
    add("csp_local_point_wait_cycles", std::to_string(stats.local_point_wait_cycles));
    add("csp_remote_faults", std::to_string(stats.remote_faults));
    add("csp_remote_queue_depth", std::to_string(stats.remote_queue_depth));
    add("csp_minimum_remote_queue_depth",
      stats.minimum_remote_queue_depth == 0xffU ?
      "unavailable" : std::to_string(stats.minimum_remote_queue_depth));
    add("csp_unacknowledged_points", std::to_string(stats.unacknowledged_points));
    add("csp_cycles", std::to_string(stats.cycles));
    add("csp_last_tx_gap_us", std::to_string(stats.last_tx_gap_us));
    add("csp_max_tx_gap_us", std::to_string(stats.max_tx_gap_us));
    add("csp_realtime_priority", std::to_string(csp_thread_priority_));
    add("csp_realtime_scheduling_active",
      stats.realtime_scheduling_active ? "true" : "false");
    add("csp_realtime_scheduling_error",
      std::to_string(stats.realtime_scheduling_error));
    add("csp_last_cycle_interval_us", std::to_string(stats.last_cycle_interval_us));
    add("csp_last_cycle_jitter_us", std::to_string(stats.last_cycle_jitter_us));
    add("csp_max_cycle_jitter_us", std::to_string(stats.max_cycle_jitter_us));
    add("csp_last_wakeup_lateness_us", std::to_string(stats.last_wakeup_lateness_us));
    add("csp_max_wakeup_lateness_us", std::to_string(stats.max_wakeup_lateness_us));
  }

  diagnostic_msgs::msg::DiagnosticArray message;
  message.header.stamp = management_node_->now();
  message.status.push_back(std::move(status));
  diagnostics_publisher_->publish(message);
}

}  // namespace elfin3_canfd

PLUGINLIB_EXPORT_CLASS(
  elfin3_canfd::Elfin3CanFdSystem, hardware_interface::SystemInterface)
