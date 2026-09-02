#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <future>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>

#include "controller_manager_msgs/srv/switch_controller.hpp"
#include "elfin3_interfaces/msg/jog_command.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace elfin3_jog
{

class JogModeManager final : public rclcpp::Node
{
public:
  JogModeManager()
  : rclcpp::Node("elfin3_jog_mode_manager")
  {
    moveit_controller_ =
      declare_parameter<std::string>("moveit_controller", "elfin_arm_controller");
    jog_controller_ =
      declare_parameter<std::string>("jog_controller", "elfin_jog_controller");
    service_timeout_sec_ = declare_parameter<double>("service_timeout_sec", 2.0);
    command_timeout_sec_ = declare_parameter<double>("command_timeout_sec", 0.50);
    canfd_safety_enabled_ = declare_parameter<bool>("canfd_safety_enabled", true);
    if (moveit_controller_.empty() || jog_controller_.empty() ||
      !std::isfinite(service_timeout_sec_) || service_timeout_sec_ <= 0.0 ||
      !std::isfinite(command_timeout_sec_) || command_timeout_sec_ <= 0.0)
    {
      throw std::invalid_argument("controller names and a positive service timeout are required");
    }
    command_timeout_ns_ = static_cast<std::int64_t>(command_timeout_sec_ * 1.0e9);

    client_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    /*
     * 创建控制器切换服务客户端并加入可重入回调组；此处仅建立客户端，
     * 实际的切换请求由 switchControllers() 中的 async_send_request() 发送。
     */
    switch_client_ = create_client<controller_manager_msgs::srv::SwitchController>(
      "/controller_manager/switch_controller", rmw_qos_profile_services_default,
      client_group_);
      
    /*
     * 创建 Supervisor 就绪查询服务客户端；进入或退出 Jog 前调用
     * /elfin3_supervisor/is_ready，确认系统允许开始新的运动或控制器切换。
     */
    supervisor_client_ = create_client<std_srvs::srv::Trigger>(
      "/elfin3_supervisor/is_ready", rmw_qos_profile_services_default, client_group_);
    hold_client_ = create_client<std_srvs::srv::Trigger>(
      "/elfin3_canfd/hold", rmw_qos_profile_services_default, client_group_);
    trajectory_reset_client_ = create_client<std_srvs::srv::Trigger>(
      "/elfin3_canfd/reset_trajectory", rmw_qos_profile_services_default, client_group_);
    controller_reset_client_ = create_client<std_srvs::srv::Trigger>(
      "/elfin3_jog/controller_reset", rmw_qos_profile_services_default, client_group_);

    mode_publisher_ = create_publisher<std_msgs::msg::String>(
      "/elfin3/control_mode", rclcpp::QoS(1).reliable().transient_local());

    // 监听 Jog 指令流，仅用于更新活动状态和看门狗时间，不直接执行关节运动。
    // 采用可靠传输并只保留最新指令，使模式管理器始终处理最新的操作状态。
    command_subscription_ = create_subscription<elfin3_interfaces::msg::JogCommand>(
      "/elfin3_jog/command",
      rclcpp::QoS(rclcpp::KeepLast(1)).reliable(),
      [this](const elfin3_interfaces::msg::JogCommand & command) {
        updateJogActivity(command);
      });

    fault_subscription_ = create_subscription<std_msgs::msg::String>(
      "/elfin3_jog/fault", rclcpp::QoS(rclcpp::KeepLast(1)).reliable(),
      [this](const std_msgs::msg::String & fault) {triggerFault(fault.data);});
    trajectory_fault_subscription_ = create_subscription<std_msgs::msg::String>(
      "/elfin3_canfd/trajectory_fault",
      rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local(),
      [this](const std_msgs::msg::String & fault) {updateTrajectoryFault(fault.data);});
      
    /*注册进入 Jog 模式的 Trigger 服务；收到 /elfin3_jog/enter 请求后，由
      enterJog() 执行安全条件检查，并将 MoveIt 控制器切换为 Jog 控制器。*/
    enter_service_ = create_service<std_srvs::srv::Trigger>(
      "/elfin3_jog/enter",
      [this](
        const std::shared_ptr<Trigger::Request>,
        std::shared_ptr<Trigger::Response> response) {enterJog(*response);});

    exit_service_ = create_service<std_srvs::srv::Trigger>(
      "/elfin3_jog/exit",
      [this](
        const std::shared_ptr<Trigger::Request>,
        std::shared_ptr<Trigger::Response> response) {exitJog(*response);});
    reset_service_ = create_service<std_srvs::srv::Trigger>(
      "/elfin3_jog/reset",
      [this](
        const std::shared_ptr<Trigger::Request>,
        std::shared_ptr<Trigger::Response> response) {resetJog(*response);});
    watchdog_timer_ = create_wall_timer(
      std::chrono::milliseconds(10), [this]() {checkCommandWatchdog();});
    setMode("MOVEIT");
  }

private:
  using SwitchController = controller_manager_msgs::srv::SwitchController;
  using Trigger = std_srvs::srv::Trigger;

  static std::int64_t steadyNowNanoseconds()
  {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
  }

  void setMode(const std::string & mode)
  {
    {
      std::lock_guard<std::mutex> lock(mode_mutex_);
      mode_ = mode;
    }
    std_msgs::msg::String message;
    message.data = mode;
    mode_publisher_->publish(message);
    RCLCPP_INFO(get_logger(), "Control mode: %s", mode.c_str());
  }

  std::string mode() const
  {
    std::lock_guard<std::mutex> lock(mode_mutex_);
    return mode_;
  }

  /**
   * @brief 检查 Supervisor 是否就绪
   * @param reason 检查结果说明
   * @return 是否已就绪
   */
  bool supervisorReady(std::string & reason)
  {
    const auto timeout = std::chrono::duration<double>(service_timeout_sec_);
    if (!supervisor_client_->wait_for_service(timeout)) {
      reason = "supervisor service unavailable";
      return false;
    }
    auto future = supervisor_client_->async_send_request(std::make_shared<Trigger::Request>());
    if (future.wait_for(timeout) != std::future_status::ready) {
      supervisor_client_->remove_pending_request(future);
      reason = "supervisor request timed out";
      return false;
    }
    const auto response = future.get();
    reason = response->message;
    return response->success;
  }

  bool callTrigger(
    const rclcpp::Client<Trigger>::SharedPtr & client,
    const char * service_name, std::string & reason)
  {
    const auto timeout = std::chrono::duration<double>(service_timeout_sec_);
    if (!client->wait_for_service(timeout)) {
      reason = std::string(service_name) + " is unavailable";
      return false;
    }
    auto future = client->async_send_request(std::make_shared<Trigger::Request>());
    if (future.wait_for(timeout) != std::future_status::ready) {
      client->remove_pending_request(future);
      reason = std::string(service_name) + " timed out";
      return false;
    }
    const auto response = future.get();
    reason = response->message;
    return response->success;
  }

  bool freshReleasedCommand() const
  {
    if (!jog_command_received_.load() || !jog_keys_released_.load()) {
      return false;
    }
    const std::int64_t age = steadyNowNanoseconds() - last_jog_command_ns_.load();
    return age >= 0 && age <= command_timeout_ns_;
  }

  bool switchControllers(
    const std::string & activate, const std::string & deactivate, std::string & reason)
  {
    const auto timeout = std::chrono::duration<double>(service_timeout_sec_);
    if (!switch_client_->wait_for_service(timeout)) {
      reason = "controller switch service unavailable";
      return false;
    }
    auto request = std::make_shared<SwitchController::Request>();
    request->activate_controllers = {activate};
    request->deactivate_controllers = {deactivate};
    request->strictness = SwitchController::Request::STRICT;
    request->start_asap = true;
    const auto timeout_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(timeout);
    request->timeout.sec = static_cast<std::int32_t>(timeout_ns.count() / 1000000000LL);
    request->timeout.nanosec = static_cast<std::uint32_t>(
      timeout_ns.count() % 1000000000LL);
    auto future = switch_client_->async_send_request(request);
    if (future.wait_for(timeout) != std::future_status::ready) {
      switch_client_->remove_pending_request(future);
      reason = "controller switch request timed out";
      return false;
    }
    if (!future.get()->ok) {
      reason = "controller manager rejected STRICT switch";
      return false;
    }
    return true;
  }

  /**
   * @brief 进入 Jog 控制模式
   * @param response 模式切换结果
   */
  void enterJog(Trigger::Response & response)
  {
    /*
     * 进入 Jog 前必须处于 MOVEIT 模式，且不存在待复位的 CAN FD 轨迹故障。
     */
    if (mode() != "MOVEIT") {
      response.success = false;
      response.message = "enter rejected: control mode is not MOVEIT";
      return;
    }
    if (trajectory_fault_active_.load()) {
      response.success = false;
      response.message = "enter rejected: CAN FD trajectory requires reset";
      return;
    }
    /*
     * 要求在命令超时时间内收到“所有 Jog 按键均已释放”的新鲜命令，
     * 防止切换完成后因按键仍处于按下状态而立即产生运动。
     */
    if (!freshReleasedCommand()) {
      response.success = false;
      response.message = "enter rejected: no fresh all-keys-released JogCommand";
      return;
    }
    /*
     * 切换控制器前确认 supervisor 已就绪，机械臂当前满足允许运动的安全条件。
     */
    std::string reason;
    if (!supervisorReady(reason)) {
      response.success = false;
      response.message = "enter rejected: " + reason;
      return;
    }
    /*
     * 发布切换中状态，并以 STRICT 模式停用 MoveIt 控制器、激活 Jog 控制器。
     */
    setMode("SWITCHING_TO_JOG");
    response.success = switchControllers(jog_controller_, moveit_controller_, reason);
    /*
     * 切换成功后进入等待 Jog 输入的 JOG_IDLE；切换失败则进入 FAULT，
     * 避免在控制器实际状态不确定时继续接收运动命令。
     */
    if (response.success) {
      hold_requested_.store(false);
      setMode("JOG_IDLE");
      response.message = "Jog controller active; MoveIt controller inactive";
    } 
    else {
      setMode("FAULT");
      response.message = "enter failed: " + reason;
    }
  }

  void exitJog(Trigger::Response & response)
  {
    const auto current_mode = mode();
    if (current_mode != "JOG_IDLE") {
      response.success = false;
      response.message = current_mode == "FAULT" ?
        "exit rejected: call /elfin3_jog/reset before leaving Jog mode" :
        "exit rejected: release all jog keys first";
      return;
    }
    std::string reason;
    if (!supervisorReady(reason)) {
      response.success = false;
      response.message = "exit rejected: " + reason;
      return;
    }
    setMode("STOPPING");
    response.success = switchControllers(moveit_controller_, jog_controller_, reason);
    if (response.success) {
      setMode("MOVEIT");
      response.message = "MoveIt controller active; Jog controller inactive";
    } else {
      setMode("FAULT");
      response.message = "exit failed: " + reason;
    }
  }

  /**
   * @brief 根据最新 Jog 指令更新按键状态、命令活性时间和 Jog 控制模式。
   * @param command 最新收到的 Jog 指令，包含六轴方向、deadman 状态和序列号。
   * @return 无。
   */
  void updateJogActivity(const elfin3_interfaces::msg::JogCommand & command)
  {
    // 无论当前处于何种控制模式，都记录命令已到达并刷新最后接收时间。
    jog_command_received_.store(true);
    last_jog_command_ns_.store(steadyNowNanoseconds());

    // 任一关节方向非零即表示当前存在有效的 Jog 按键动作。
    bool active = false;
    for (const auto direction : command.directions) {
      active = active || direction != 0;
    }

    // deadman 有效且所有方向为零，表示操作者已释放全部 Jog 按键。
    // 该状态是进入 Jog 模式及故障复位前的安全条件之一。
    jog_keys_released_.store(command.deadman && !active);

    // 非 Jog 模式下只维护命令新鲜度和按键状态，不执行 Jog 模式切换。
    const auto current_mode = mode();
    if (current_mode != "JOG_IDLE" && current_mode != "JOG_ACTIVE") {
      return;
    }

    // Jog 模式下 deadman 失效表示安全使能被释放或键盘输入异常，立即进入故障状态。
    if (!command.deadman) {
      triggerFault("Jog deadman released or keyboard input failed");
      return;
    }

    // 有活动方向时进入 JOG_ACTIVE，否则保持或切换到 JOG_IDLE。
    const std::string next_mode = command.deadman && active ? "JOG_ACTIVE" : "JOG_IDLE";
    if (next_mode != current_mode) {
      setMode(next_mode);
    }
  }

  void checkCommandWatchdog()
  {
    const auto current_mode = mode();
    if (current_mode != "JOG_IDLE" && current_mode != "JOG_ACTIVE") {
      return;
    }
    const std::int64_t age = steadyNowNanoseconds() - last_jog_command_ns_.load();
    if (!jog_command_received_.load() || age < 0 || age > command_timeout_ns_) {
      triggerFault("Jog command stream timed out");
    }
  }

  void updateTrajectoryFault(const std::string & reason)
  {
    if (reason == "CLEAR") {
      trajectory_fault_active_.store(false);
      return;
    }
    trajectory_fault_active_.store(true);
    triggerFault(reason);
  }

  void triggerFault(const std::string & reason)
  {
    const auto current_mode = mode();
    if (current_mode != "JOG_IDLE" && current_mode != "JOG_ACTIVE") {
      return;
    }
    if (hold_requested_.exchange(true)) {
      return;
    }
    setMode("FAULT");
    RCLCPP_ERROR(get_logger(), "Jog fault: %s", reason.c_str());
    if (!canfd_safety_enabled_) {
      RCLCPP_WARN(get_logger(), "CAN FD HOLD skipped because canfd_safety_enabled=false");
      return;
    }
    if (!hold_client_->service_is_ready()) {
      RCLCPP_ERROR(get_logger(), "CAN FD HOLD service is unavailable");
      return;
    }
    hold_client_->async_send_request(
      std::make_shared<Trigger::Request>(),
      [logger = get_logger()](rclcpp::Client<Trigger>::SharedFuture future) {
        try {
          const auto response = future.get();
          if (!response->success) {
            RCLCPP_ERROR(logger, "CAN FD HOLD rejected: %s", response->message.c_str());
          }
        } catch (const std::exception & exception) {
          RCLCPP_ERROR(logger, "CAN FD HOLD request failed: %s", exception.what());
        }
      });
  }

  void resetJog(Trigger::Response & response)
  {
    if (mode() != "FAULT") {
      response.success = false;
      response.message = "reset rejected: control mode is not FAULT";
      return;
    }
    if (!freshReleasedCommand()) {
      response.success = false;
      response.message = "reset rejected: a fresh all-keys-released command is required";
      return;
    }
    std::string reason;
    if (canfd_safety_enabled_ &&
      !callTrigger(trajectory_reset_client_, "/elfin3_canfd/reset_trajectory", reason))
    {
      response.success = false;
      response.message = "reset rejected: " + reason;
      return;
    }
    if (!callTrigger(controller_reset_client_, "/elfin3_jog/controller_reset", reason)) {
      response.success = false;
      response.message = "reset rejected: " + reason;
      return;
    }
    trajectory_fault_active_.store(false);
    hold_requested_.store(false);
    setMode("JOG_IDLE");
    response.success = true;
    response.message = "Jog fault cleared; next motion starts from a new RESET trajectory";
  }

  std::string moveit_controller_;
  std::string jog_controller_;
  double service_timeout_sec_{2.0};
  double command_timeout_sec_{0.50};
  std::int64_t command_timeout_ns_{500000000};
  bool canfd_safety_enabled_{true};
  mutable std::mutex mode_mutex_;
  std::string mode_{"MOVEIT"};
  std::atomic<bool> jog_keys_released_{false};
  std::atomic<bool> jog_command_received_{false};
  std::atomic<bool> hold_requested_{false};
  std::atomic<bool> trajectory_fault_active_{false};
  std::atomic<std::int64_t> last_jog_command_ns_{0};
  rclcpp::CallbackGroup::SharedPtr client_group_;
  /*
   * /controller_manager/switch_controller 服务客户端，用于请求激活和停用控制器。
   */
  rclcpp::Client<SwitchController>::SharedPtr switch_client_;
  rclcpp::Client<Trigger>::SharedPtr supervisor_client_;
  rclcpp::Client<Trigger>::SharedPtr hold_client_;
  rclcpp::Client<Trigger>::SharedPtr trajectory_reset_client_;
  rclcpp::Client<Trigger>::SharedPtr controller_reset_client_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mode_publisher_;
  rclcpp::Subscription<elfin3_interfaces::msg::JogCommand>::SharedPtr command_subscription_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr fault_subscription_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr trajectory_fault_subscription_;
  rclcpp::Service<Trigger>::SharedPtr enter_service_;
  rclcpp::Service<Trigger>::SharedPtr exit_service_;
  rclcpp::Service<Trigger>::SharedPtr reset_service_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;
};

}  // namespace elfin3_jog

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<elfin3_jog::JogModeManager>();
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2U);
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
