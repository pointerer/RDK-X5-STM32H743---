#pragma once

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "elfin3_canfd_driver/can_fd_transport.hpp"
#include "elfin3_canfd_driver/csp_scheduler.hpp"
#include "elfin3_canfd_driver/protocol.hpp"
#include "elfin3_canfd_driver/sequence_tracker.hpp"

namespace elfin3_canfd
{

class Elfin3CanFdSystem : public hardware_interface::SystemInterface
{
public:
  using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  ~Elfin3CanFdSystem() override;

  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  bool parse_hardware_parameters();
  bool start_backend();
  void stop_backend();
  bool wait_for_scheduler_terminal(std::chrono::milliseconds timeout) const;
  void receive_loop();
  void dispatch(const CanFdFrame & frame);
  bool send_frame(
    std::uint32_t id, const std::uint8_t * data, std::size_t size, std::string & error);
  bool transmit_control(MotionControl command, std::string & error, std::uint32_t & token);
  void send_host_heartbeat(HostState state);
  HostState heartbeat_state() const;
  bool communication_ready_locked(std::chrono::steady_clock::time_point now) const;
  bool motion_ready_locked(std::chrono::steady_clock::time_point now) const;
  bool axes_enabled_locked() const;
  bool fault_present_locked() const;
  void create_management_node();
  void destroy_management_node();
  void publish_diagnostics();
  void publish_trajectory_fault_if_needed();
  void handle_control_request(
    MotionControl command, std_srvs::srv::Trigger::Response & response);
  void handle_trajectory_reset(std_srvs::srv::Trigger::Response & response);

  static constexpr std::array<const char *, kAxisCount> kJointNames{
    "elfin_joint1", "elfin_joint2", "elfin_joint3",
    "elfin_joint4", "elfin_joint5", "elfin_joint6"};
  static constexpr double kTrajectoryStartDeltaRad = 1.0e-9;
  static constexpr double kTrajectoryGoalToleranceRad = 0.01;
  static constexpr std::size_t kTrajectoryFinishStableCycles = 50U;

  CanFdTransport transport_;
  std::unique_ptr<CspScheduler> csp_scheduler_;
  mutable std::mutex transmit_mutex_;
  mutable std::mutex state_mutex_;

  std::array<double, kAxisCount> state_positions_{};
  std::array<double, kAxisCount> command_positions_{};
  std::array<double, kAxisCount> last_submitted_command_positions_{};
  std::array<double, kAxisCount> latest_feedback_positions_{};
  std::array<double, kAxisCount> lower_limits_{};
  std::array<double, kAxisCount> upper_limits_{};
  std::array<std::int32_t, kAxisCount> zero_offsets_{};
  std::array<std::int8_t, kAxisCount> direction_signs_{};

  PositionFeedback latest_position_{};
  DetailedStatus latest_detailed_{};
  DeviceHeartbeat latest_heartbeat_{};
  TrajectoryStatus latest_trajectory_status_{};
  std::chrono::steady_clock::time_point last_position_time_{};
  std::chrono::steady_clock::time_point last_detailed_time_{};
  std::chrono::steady_clock::time_point last_heartbeat_time_{};
  std::chrono::steady_clock::time_point last_trajectory_status_time_{};
  std::chrono::steady_clock::time_point communication_loss_started_at_{};
  bool has_position_{false};
  bool has_detailed_{false};
  bool has_heartbeat_{false};
  bool has_trajectory_status_{false};
  bool communication_loss_active_{false};
  bool communication_fatal_reported_{false};
  bool has_last_submitted_command_{false};
  std::atomic<std::size_t> stable_command_cycles_{0U};
  std::atomic_bool position_delay_warning_active_{false};
  std::atomic_bool communication_hold_latched_{false};
  std::atomic<std::uint64_t> position_delay_warning_count_{0U};
  std::atomic<std::uint64_t> communication_hold_count_{0U};
  std::atomic<std::uint64_t> communication_fatal_count_{0U};
  std::atomic<std::uint64_t> hardware_write_cycles_{0U};
  std::atomic<std::uint64_t> hardware_command_change_count_{0U};
  std::atomic<double> hardware_last_command_delta_rad_{0.0};
  std::atomic<double> hardware_max_command_delta_rad_{0.0};
  std::atomic<std::int64_t> hardware_last_command_change_ms_{-1};
  std::atomic<std::uint64_t> hardware_start_attempts_{0U};
  std::atomic<std::uint64_t> hardware_start_successes_{0U};
  std::atomic<std::uint64_t> hardware_start_rejections_{0U};

  SequenceTracker position_sequence_;
  SequenceTracker detailed_sequence_;
  SequenceTracker diagnostic_sequence_;
  SequenceTracker heartbeat_sequence_;
  SequenceTracker trajectory_status_sequence_;

  std::string interface_name_{"can0"};
  int position_warning_timeout_ms_{20};
  int position_timeout_ms_{50};
  int communication_error_grace_ms_{300};
  int detailed_timeout_ms_{100};
  int heartbeat_timeout_ms_{300};
  int trajectory_status_timeout_ms_{100};
  int activation_timeout_ms_{3000};
  int csp_period_us_{static_cast<int>(kCspPeriodUs)};
  int csp_target_timeout_ms_{100};
  int csp_validity_ms_{static_cast<int>(kCspValidityMs)};
  int csp_refill_period_us_{static_cast<int>(kDefaultCspRefillPeriodUs)};
  int csp_remote_low_watermark_{static_cast<int>(kDefaultRemoteLowWatermark)};
  int csp_remote_high_watermark_{static_cast<int>(kDefaultRemoteHighWatermark)};
  int csp_thread_priority_{kDefaultCspThreadPriority};

  std::atomic_bool receive_running_{false};
  std::atomic_bool hardware_active_{false};
  std::atomic_bool bus_off_{false};
  std::thread receive_thread_;

  rclcpp::Node::SharedPtr management_node_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> management_executor_;
  std::thread management_thread_;
  rclcpp::TimerBase::SharedPtr heartbeat_timer_;
  rclcpp::TimerBase::SharedPtr diagnostics_timer_;
  rclcpp::TimerBase::SharedPtr trajectory_fault_timer_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr trajectory_fault_publisher_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr disable_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr enable_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr hold_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr quick_stop_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr fault_reset_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr trajectory_reset_service_;

  std::chrono::steady_clock::time_point started_at_{std::chrono::steady_clock::now()};
  std::uint8_t heartbeat_tx_sequence_{0};
  std::atomic<std::uint8_t> control_sequence_{0};
  std::atomic<std::uint32_t> next_control_token_{1};
  bool trajectory_fault_published_{false};
};

}  // namespace elfin3_canfd
