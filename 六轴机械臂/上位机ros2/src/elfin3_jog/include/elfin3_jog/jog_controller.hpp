#ifndef ELFIN3_JOG__JOG_CONTROLLER_HPP_
#define ELFIN3_JOG__JOG_CONTROLLER_HPP_

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "elfin3_interfaces/msg/jog_command.hpp"
#include "elfin3_jog/jog_math.hpp"
#include "rclcpp/callback_group.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

namespace elfin3_jog
{

class Elfin3JogController final : public controller_interface::ControllerInterface
{
public:
  controller_interface::CallbackReturn on_init() override;
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;
  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

private:
  enum class FaultCode : std::uint8_t
  {
    kNone = 0,
    kCommandNotReceived = 1,
    kDeadmanInactive = 2,
    kCommandTimeout = 3,
    kInvalidDirection = 4,
    kMultipleAxesActive = 5,
    kFollowingError = 6,
  };

  struct CommandSnapshot
  {
    std::array<std::int8_t, 6> directions{};
    bool received{false};
    bool deadman{false};
    std::uint32_t sequence{0};
    std::int64_t received_ns{0};
  };

  static constexpr std::array<const char *, 6> kJointNames = {
    "elfin_joint1", "elfin_joint2", "elfin_joint3",
    "elfin_joint4", "elfin_joint5", "elfin_joint6",
  };
  static std::int64_t steadyNowNanoseconds();
  void handleJogCommand(const elfin3_interfaces::msg::JogCommand & command);
  void writeReleasedCommandSnapshot();
  void latchSafetyFault(FaultCode fault_code);
  void publishSafetyFault();

  std::array<std::int64_t, 6> target_counts_{};
  std::array<std::int64_t, 6> lower_counts_{};
  std::array<std::int64_t, 6> upper_counts_{};
  realtime_tools::RealtimeBuffer<CommandSnapshot> command_buffer_{};
  double following_error_limit_rad_{0.05};
  std::int64_t command_timeout_ns_{500000000};
  bool limits_configured_{false};
  bool target_initialized_{false};
  std::atomic<bool> safety_fault_{false};
  std::atomic<bool> reset_requested_{false};
  std::atomic<bool> fault_published_{false};
  std::atomic<FaultCode> fault_code_{FaultCode::kNone};
  std::atomic<std::int64_t> last_callback_ns_{0};
  std::atomic<std::int64_t> last_command_ns_{0};
  std::atomic<std::int64_t> max_callback_gap_ns_{0};
  std::atomic<std::int64_t> fault_callback_age_ms_{-1};
  std::atomic<std::int64_t> fault_command_age_ms_{-1};
  std::atomic<std::int64_t> fault_max_callback_gap_ms_{0};
  std::atomic<std::uint64_t> callback_count_{0};
  std::atomic<std::uint64_t> accepted_command_count_{0};
  std::atomic<std::uint64_t> ignored_sequence_count_{0};
  std::atomic<std::uint64_t> fault_callback_count_{0};
  std::atomic<std::uint64_t> fault_accepted_command_count_{0};
  std::atomic<std::uint64_t> fault_ignored_sequence_count_{0};
  std::atomic<std::uint32_t> last_received_sequence_{0};
  std::atomic<std::uint32_t> last_sequence_{0};
  std::atomic<std::uint32_t> fault_last_received_sequence_{0};
  std::atomic<std::uint32_t> fault_last_sequence_{0};
  std::atomic<bool> has_sequence_{false};
  rclcpp::CallbackGroup::SharedPtr command_callback_group_;
  rclcpp::CallbackGroup::SharedPtr auxiliary_callback_group_;
  rclcpp::Subscription<elfin3_interfaces::msg::JogCommand>::SharedPtr command_subscription_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr fault_publisher_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_service_;
  rclcpp::TimerBase::SharedPtr fault_timer_;
};

}  // namespace elfin3_jog

#endif  // ELFIN3_JOG__JOG_CONTROLLER_HPP_
