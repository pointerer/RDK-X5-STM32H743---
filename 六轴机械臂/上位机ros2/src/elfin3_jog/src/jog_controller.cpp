#include "elfin3_jog/jog_controller.hpp"

#include <cmath>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/logging.hpp"

namespace elfin3_jog
{
std::int64_t Elfin3JogController::steadyNowNanoseconds()
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}

void Elfin3JogController::latchSafetyFault(const FaultCode fault_code)
{
  if (safety_fault_.load()) {
    return;
  }

  const std::int64_t now_ns = steadyNowNanoseconds();
  const std::int64_t callback_ns = last_callback_ns_.load();
  const std::int64_t command_ns = last_command_ns_.load();
  fault_callback_age_ms_.store(callback_ns > 0 ? (now_ns - callback_ns) / 1000000 : -1);
  fault_command_age_ms_.store(command_ns > 0 ? (now_ns - command_ns) / 1000000 : -1);
  fault_max_callback_gap_ms_.store(max_callback_gap_ns_.load() / 1000000);
  fault_callback_count_.store(callback_count_.load());
  fault_accepted_command_count_.store(accepted_command_count_.load());
  fault_ignored_sequence_count_.store(ignored_sequence_count_.load());
  fault_last_received_sequence_.store(last_received_sequence_.load());
  fault_last_sequence_.store(last_sequence_.load());
  fault_code_.store(fault_code);
  safety_fault_.store(true);
}

void Elfin3JogController::publishSafetyFault()
{
  if (!safety_fault_.load() || !fault_publisher_ || fault_published_.exchange(true)) {
    return;
  }
  std_msgs::msg::String message;
  switch (fault_code_.load()) {
    case FaultCode::kCommandNotReceived:
      message.data = "Jog command stream was not received";
      break;
    case FaultCode::kDeadmanInactive:
      message.data = "Jog deadman inactive or keyboard input failed";
      break;
    case FaultCode::kCommandTimeout:
      message.data = "Jog command stream timed out";
      break;
    case FaultCode::kInvalidDirection:
      message.data = "Jog command contains an invalid direction";
      break;
    case FaultCode::kMultipleAxesActive:
      message.data = "Jog command requests multiple axes";
      break;
    case FaultCode::kFollowingError:
      message.data = "Jog following error or invalid joint state";
      break;
    default:
      message.data = "Jog controller fault with no reason code";
      break;
  }
  message.data +=
    " [callback_age_ms=" + std::to_string(fault_callback_age_ms_.load()) +
    ", command_age_ms=" + std::to_string(fault_command_age_ms_.load()) +
    ", max_callback_gap_ms=" + std::to_string(fault_max_callback_gap_ms_.load()) +
    ", callbacks=" + std::to_string(fault_callback_count_.load()) +
    ", accepted=" + std::to_string(fault_accepted_command_count_.load()) +
    ", ignored_sequence=" + std::to_string(fault_ignored_sequence_count_.load()) +
    ", last_received_sequence=" +
    std::to_string(fault_last_received_sequence_.load()) +
    ", last_accepted_sequence=" + std::to_string(fault_last_sequence_.load()) + "]";
  RCLCPP_ERROR(get_node()->get_logger(), "%s", message.data.c_str());
  fault_publisher_->publish(message);
}

/**
 * @brief 接收并校验 Jog 指令，将有效指令写入实时缓冲区供控制循环使用。
 * @param command 待处理的 Jog 指令，包含各关节运动方向、deadman 状态和序列号。
 * @return 无。
 */
void Elfin3JogController::handleJogCommand(
  const elfin3_interfaces::msg::JogCommand & command)
{
  // 记录本次回调的到达时间和接收统计，供命令超时检测及故障诊断使用。
  const std::int64_t now_ns = steadyNowNanoseconds();
  const std::int64_t previous_callback_ns = last_callback_ns_.exchange(now_ns);
  callback_count_.fetch_add(1U);
  last_received_sequence_.store(command.sequence);

  // 原子更新相邻两次回调的最大时间间隔，便于定位指令流抖动或阻塞。
  if (previous_callback_ns > 0 && now_ns >= previous_callback_ns) {
    const std::int64_t gap_ns = now_ns - previous_callback_ns;
    std::int64_t previous_max = max_callback_gap_ns_.load();
    while (gap_ns > previous_max &&
      !max_callback_gap_ns_.compare_exchange_weak(previous_max, gap_ns))
    {
    }
  }

  // 序列号必须向前递增。无符号差值同时兼容 uint32_t 序列号自然回绕。
  if (has_sequence_.load()) {
    const std::uint32_t last_sequence = last_sequence_.load();
    const std::uint32_t forward_distance = command.sequence - last_sequence;
    if (forward_distance == 0U || forward_distance > 0x7fffffffU) {
      // 对重复或乱序指令先写入零方向保持命令，使机械臂立即停止，但不立刻触发
      // deadman 故障；后续正常递增的指令可以恢复 Jog，持续异常则由看门狗处理。
      CommandSnapshot hold_command;
      hold_command.received = last_command_ns_.load() > 0;
      hold_command.deadman = true;
      hold_command.sequence = last_sequence;
      hold_command.received_ns = last_command_ns_.load();
      command_buffer_.writeFromNonRT(hold_command);
      ignored_sequence_count_.fetch_add(1U);
      RCLCPP_WARN_THROTTLE(
        get_node()->get_logger(), *get_node()->get_clock(), 1000,
        "Ignoring non-forward jog sequence: last=%u received=%u",
        last_sequence, command.sequence);
      return;
    }
  }

  // 每个方向只能为 -1、0 或 1，并且一次只允许 Jog 一个关节。
  std::size_t active_axes = 0;
  for (const auto direction : command.directions) {
    if (direction < -1 || direction > 1) {
      latchSafetyFault(FaultCode::kInvalidDirection);
      return;
    }
    if (direction != 0) {
      ++active_axes;
    }
  }
  if (active_axes > 1U) {
    latchSafetyFault(FaultCode::kMultipleAxesActive);
    return;
  }

  // 指令通过校验后更新已接收序列号；deadman 释放时强制清零所有运动方向。
  last_sequence_.store(command.sequence);
  has_sequence_.store(true);
  CommandSnapshot snapshot;
  for (std::size_t joint = 0; joint < kJointNames.size(); ++joint) {
    snapshot.directions[joint] = command.deadman ? command.directions[joint] : 0;
  }
  snapshot.received = true;
  snapshot.deadman = command.deadman;
  snapshot.sequence = command.sequence;
  snapshot.received_ns = now_ns;

  // ROS 回调属于非实时线程，通过实时缓冲区将快照安全地传递给 update()。
  // RealtimeBuffer 保证 update() 读取的是一份完整快照，同时尽量避免阻塞实时控制循环。
  command_buffer_.writeFromNonRT(snapshot);
  accepted_command_count_.fetch_add(1U);
  last_command_ns_.store(now_ns);
}

void Elfin3JogController::writeReleasedCommandSnapshot()
{
  const std::int64_t now_ns = steadyNowNanoseconds();
  CommandSnapshot snapshot;
  snapshot.received = true;
  snapshot.deadman = true;
  snapshot.sequence = last_sequence_.load();
  snapshot.received_ns = now_ns;
  command_buffer_.writeFromNonRT(snapshot);
  last_callback_ns_.store(now_ns);
  last_command_ns_.store(now_ns);
  has_sequence_.store(false);
}

controller_interface::CallbackReturn Elfin3JogController::on_init()
{
  auto_declare<std::vector<double>>(
    "lower_limits", {-3.14, -2.35, -2.61, -3.14, -2.56, -3.14});
  auto_declare<std::vector<double>>(
    "upper_limits", {3.14, 2.35, 2.61, 3.14, 2.56, 3.14});
  auto_declare<double>("soft_limit_margin_rad", 0.01);
  auto_declare<double>("following_error_limit_rad", 0.05);
  auto_declare<double>("command_timeout_sec", 0.50);
  command_callback_group_ = get_node()->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);
  auxiliary_callback_group_ = get_node()->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
Elfin3JogController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration configuration;
  configuration.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto * joint : kJointNames) {
    configuration.names.emplace_back(
      std::string(joint) + "/" + hardware_interface::HW_IF_POSITION);
  }
  return configuration;
}

controller_interface::InterfaceConfiguration
Elfin3JogController::state_interface_configuration() const
{
  return command_interface_configuration();
}

/**
 * @brief 更新各关节的 Jog 位置指令
 * @return 控制器更新结果
 */
controller_interface::return_type Elfin3JogController::update(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  // 检查目标状态和命令接口是否有效
  if (!target_initialized_ || command_interfaces_.size() != kJointNames.size()) {
    return controller_interface::return_type::ERROR;
  }

  // 复位时以六轴实际位置重新建立目标，并清除已锁存的安全故障。
  if (reset_requested_.exchange(false)) {
    bool reset_valid = state_interfaces_.size() == kJointNames.size();
    std::array<std::int64_t, 6> reset_targets{};
    for (std::size_t joint = 0; reset_valid && joint < kJointNames.size(); ++joint) {
      const double actual_position = state_interfaces_[joint].get_value();
      const std::int64_t actual_counts = radians_to_counts(actual_position);
      reset_valid = std::isfinite(actual_position) &&
        actual_counts >= lower_counts_[joint] && actual_counts <= upper_counts_[joint];
      reset_targets[joint] = actual_counts;
    }
    if (reset_valid) {
      target_counts_ = reset_targets;
      safety_fault_.store(false);
      fault_code_.store(FaultCode::kNone);
      fault_published_.store(false);
    } else {
      fault_code_.store(FaultCode::kFollowingError);
      safety_fault_.store(true);
      fault_published_.store(false);
    }
  }

  // 故障期间保持最后一个有效的六轴位置目标。
  if (safety_fault_.load()) {
    for (std::size_t joint = 0; joint < kJointNames.size(); ++joint) {
      command_interfaces_[joint].set_value(counts_to_radians(target_counts_[joint]));
    }
    return controller_interface::return_type::OK;
  }

  // 校验实时指令快照、deadman 状态和命令时效性。
  const CommandSnapshot command = *command_buffer_.readFromRT();
  const std::int64_t command_age_ns = steadyNowNanoseconds() - command.received_ns;
  if (!command.received) {
    latchSafetyFault(FaultCode::kCommandNotReceived);
    return controller_interface::return_type::OK;
  }
  if (!command.deadman) {
    latchSafetyFault(FaultCode::kDeadmanInactive);
    return controller_interface::return_type::OK;
  }
  if (command_age_ns < 0 || command_age_ns > command_timeout_ns_) {
    latchSafetyFault(FaultCode::kCommandTimeout);
    return controller_interface::return_type::OK;
  }

  // 任意一轴反馈无效或跟随误差超限时，锁存安全故障。
  for (std::size_t joint = 0; joint < kJointNames.size(); ++joint) {
    const double actual_position = state_interfaces_[joint].get_value();
    if (!std::isfinite(actual_position) ||
      std::abs(counts_to_radians(target_counts_[joint]) - actual_position) >
      following_error_limit_rad_)
    {
      latchSafetyFault(FaultCode::kFollowingError);
      return controller_interface::return_type::OK;
    }
  }

  // 核心目标生成：六轴每个 500 Hz 周期按方向增减 100 脉冲，并检查软限位。
  for (std::size_t joint = 0; joint < kJointNames.size(); ++joint) {
    const std::int64_t direction = command.directions[joint];
    std::int64_t candidate = target_counts_[joint];
    if (!bounded_jog_step(
        target_counts_[joint], static_cast<std::int8_t>(direction),
        lower_counts_[joint], upper_counts_[joint], candidate))
    {
      candidate = target_counts_[joint];
    } 
    else {
      target_counts_[joint] = candidate;
    }
    command_interfaces_[joint].set_value(counts_to_radians(target_counts_[joint]));
  }
  return controller_interface::return_type::OK;
}

controller_interface::CallbackReturn Elfin3JogController::on_configure(
  const rclcpp_lifecycle::State &)
{
  const auto lower_limits = get_node()->get_parameter("lower_limits").as_double_array();
  const auto upper_limits = get_node()->get_parameter("upper_limits").as_double_array();
  const double margin = get_node()->get_parameter("soft_limit_margin_rad").as_double();
  following_error_limit_rad_ =
    get_node()->get_parameter("following_error_limit_rad").as_double();
  const double command_timeout_sec =
    get_node()->get_parameter("command_timeout_sec").as_double();
  if (lower_limits.size() != kJointNames.size() ||
    upper_limits.size() != kJointNames.size() || !std::isfinite(margin) || margin < 0.0 ||
    !std::isfinite(following_error_limit_rad_) || following_error_limit_rad_ <= 0.0 ||
    !std::isfinite(command_timeout_sec) || command_timeout_sec <= 0.0)
  {
    return controller_interface::CallbackReturn::ERROR;
  }

  for (std::size_t joint = 0; joint < kJointNames.size(); ++joint) {
    if (!std::isfinite(lower_limits[joint]) || !std::isfinite(upper_limits[joint]) ||
      lower_limits[joint] + margin >= upper_limits[joint] - margin)
    {
      return controller_interface::CallbackReturn::ERROR;
    }
    lower_counts_[joint] = radians_to_counts(lower_limits[joint] + margin);
    upper_counts_[joint] = radians_to_counts(upper_limits[joint] - margin);
  }
  command_timeout_ns_ = static_cast<std::int64_t>(command_timeout_sec * 1.0e9);

  // 将 Jog 指令放入独立的互斥回调组，避免同一时刻并发处理多条控制命令。
  rclcpp::SubscriptionOptions command_options;
  command_options.callback_group = command_callback_group_;

  // 只保留最新的一条可靠指令；收到消息后交由 handleJogCommand() 校验并缓存。
  command_subscription_ = get_node()->create_subscription<elfin3_interfaces::msg::JogCommand>(
    "/elfin3_jog/command",
    rclcpp::QoS(rclcpp::KeepLast(1)).reliable(),
    [this](const elfin3_interfaces::msg::JogCommand & command) {
      handleJogCommand(command);
    },
    command_options);

  fault_publisher_ = get_node()->create_publisher<std_msgs::msg::String>(
    "/elfin3_jog/fault", rclcpp::QoS(rclcpp::KeepLast(1)).reliable());

  // 创建 Jog 控制器复位服务，将复位请求交给下一个 update() 周期处理。
  reset_service_ = get_node()->create_service<std_srvs::srv::Trigger>(
    "/elfin3_jog/controller_reset",
    [this](
      const std::shared_ptr<std_srvs::srv::Trigger::Request>,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
      writeReleasedCommandSnapshot();
      reset_requested_.store(true);
      response->success = true;
      response->message = "Jog controller reset requested";
    }, rmw_qos_profile_services_default, auxiliary_callback_group_);
    
  fault_timer_ = get_node()->create_wall_timer(
    std::chrono::milliseconds(50), [this]() {publishSafetyFault();},
    auxiliary_callback_group_);
  limits_configured_ = true;
  safety_fault_.store(false);
  reset_requested_.store(false);
  fault_published_.store(false);
  fault_code_.store(FaultCode::kNone);
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn Elfin3JogController::on_activate(
  const rclcpp_lifecycle::State &)
{
  if (!limits_configured_ || state_interfaces_.size() != kJointNames.size() ||
    command_interfaces_.size() != kJointNames.size())
  {
    return controller_interface::CallbackReturn::ERROR;
  }
  for (std::size_t joint = 0; joint < kJointNames.size(); ++joint) {
    const double position = state_interfaces_[joint].get_value();
    if (!std::isfinite(position)) {
      return controller_interface::CallbackReturn::ERROR;
    }
    target_counts_[joint] = radians_to_counts(position);
    if (target_counts_[joint] < lower_counts_[joint] ||
      target_counts_[joint] > upper_counts_[joint])
    {
      return controller_interface::CallbackReturn::ERROR;
    }
    command_interfaces_[joint].set_value(counts_to_radians(target_counts_[joint]));
  }
  writeReleasedCommandSnapshot();
  safety_fault_.store(false);
  fault_code_.store(FaultCode::kNone);
  fault_published_.store(false);
  if (fault_publisher_) {
    fault_publisher_->on_activate();
  }
  target_initialized_ = true;
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn Elfin3JogController::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  CommandSnapshot stopped_command;
  command_buffer_.writeFromNonRT(stopped_command);
  reset_requested_.store(false);
  target_initialized_ = false;
  if (fault_publisher_) {
    fault_publisher_->on_deactivate();
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

}  // namespace elfin3_jog

PLUGINLIB_EXPORT_CLASS(
  elfin3_jog::Elfin3JogController,
  controller_interface::ControllerInterface)
