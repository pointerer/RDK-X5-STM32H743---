#include <fcntl.h>
#include <linux/input.h>
#include <poll.h>
#include <unistd.h>

#include <array>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <system_error>
#include <thread>

#include "elfin3_interfaces/msg/jog_command.hpp"
#include "rclcpp/rclcpp.hpp"

namespace elfin3_jog
{

class KeyboardTeleopNode final : public rclcpp::Node
{
public:
  KeyboardTeleopNode()
  : rclcpp::Node("elfin3_keyboard_teleop")
  {
    input_device_ = declare_parameter<std::string>("input_device", "/dev/input/event0");
    poll_timeout_ms_ = declare_parameter<int>("poll_timeout_ms", 50);
    const double publish_rate_hz = declare_parameter<double>("publish_rate_hz", 100.0);
    if (input_device_.empty() || poll_timeout_ms_ <= 0 || publish_rate_hz <= 0.0) {
      throw std::invalid_argument("input_device and positive polling/publish rates are required");
    }

    clearDirections();
    input_fd_ = ::open(input_device_.c_str(), O_RDONLY | O_NONBLOCK | O_CLOEXEC);
    if (input_fd_ < 0) {
      throw std::system_error(errno, std::generic_category(), "open " + input_device_);
    }
    command_publisher_ = create_publisher<elfin3_interfaces::msg::JogCommand>(
      "/elfin3_jog/command", rclcpp::QoS(rclcpp::KeepLast(1)).reliable());
    const auto publish_period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / publish_rate_hz));
    publish_timer_ = create_wall_timer(publish_period, [this]() {publishCommand();});
    input_thread_ = std::thread([this]() {inputLoop();});
    RCLCPP_INFO(get_logger(), "Reading jog keys from %s", input_device_.c_str());
  }

  ~KeyboardTeleopNode() override
  {
    stop_requested_.store(true);
    if (input_thread_.joinable()) {
      input_thread_.join();
    }
    if (input_fd_ >= 0) {
      ::close(input_fd_);
    }
  }

private:
  struct KeyBinding
  {
    std::size_t joint;
    std::int8_t direction;
  };

  static std::optional<KeyBinding> bindingFor(const std::uint16_t code)
  {
    switch (code) {
      case KEY_Q: return KeyBinding{0, 1};
      case KEY_A: return KeyBinding{0, -1};
      case KEY_W: return KeyBinding{1, 1};
      case KEY_S: return KeyBinding{1, -1};
      case KEY_E: return KeyBinding{2, 1};
      case KEY_D: return KeyBinding{2, -1};
      case KEY_R: return KeyBinding{3, 1};
      case KEY_F: return KeyBinding{3, -1};
      case KEY_T: return KeyBinding{4, 1};
      case KEY_G: return KeyBinding{4, -1};
      case KEY_Y: return KeyBinding{5, 1};
      case KEY_H: return KeyBinding{5, -1};
      default: return std::nullopt;
    }
  }

  void clearDirections()
  {
    for (auto & direction : directions_) {
      direction.store(0);
    }
    active_key_.store(-1);
  }

  /**
   * @brief 发布当前 Jog 控制指令
   */
  void publishCommand()
  {
    // 发布器未初始化时跳过
    if (!command_publisher_) {
      return;
    }

    // 加锁并写入各关节的当前运动方向
    std::lock_guard<std::mutex> lock(publish_mutex_);
    elfin3_interfaces::msg::JogCommand command;
    for (std::size_t joint = 0; joint < directions_.size(); ++joint) {
      command.directions[joint] = directions_[joint].load();
    }

    // 设置安全使能状态和指令序列号
    command.deadman = !emergency_stop_requested_.load() && !input_fault_.load();
    command.sequence = sequence_.fetch_add(1U);

    // 发布组装完成的 Jog 指令
    command_publisher_->publish(command);
  }

  /**
   * @brief 处理键盘输入事件
   * @param event 键盘事件
   */
  void handleKey(const input_event & event)
  {
    // 处理急停键的按下与释放
    if (event.code == KEY_ESC) {
      if (event.value == 1) {
        clearDirections();
        emergency_stop_requested_.store(true);
        publishCommand();
        RCLCPP_WARN(get_logger(), "Emergency stop key pressed");
      } else if (event.value == 0) {
        clearDirections();
        emergency_stop_requested_.store(false);
        publishCommand();
        RCLCPP_INFO(
          get_logger(), "Emergency key released; explicit /elfin3_jog/reset is still required");
      }
      return;
    }

    // 查找按键对应的关节运动配置
    const auto binding = bindingFor(event.code);
    if (!binding) {
      return;
    }

    // 处理 Jog 键释放事件
    if (event.value == 0) {
      // 仅当释放的是当前活动键时，才停止对应关节并清除活动键状态。
      if (active_key_.load() == static_cast<int>(event.code)) {
        directions_[binding->joint].store(0);
        active_key_.store(-1);
      }
      publishCommand();
      return;
    }
    // 仅处理按下和长按事件
    if (event.value != 1 && event.value != 2) {
      return;
    }

    // 更新活动按键和关节运动方向
    const int active_key = active_key_.load();
    // 当前没有活动键：将本次按键设为活动键，并启动对应关节。
    if (active_key < 0) {
      directions_[binding->joint].store(binding->direction);
      active_key_.store(event.code);
    } 
    // 当前按键仍是活动键：处理长按产生的重复事件，维持原运动方向。
    else if (active_key == static_cast<int>(event.code)) {
      directions_[binding->joint].store(binding->direction);
    } 
    // 已有其他活动键：检查新按键是否与它控制同一关节的相反方向。
    else if (active_key != static_cast<int>(event.code)) {
      const auto active_binding = bindingFor(static_cast<std::uint16_t>(active_key));
      if (active_binding && active_binding->joint == binding->joint &&
        active_binding->direction != binding->direction)
      {
        // 同一关节的正反方向键同时按下时停止该关节；其他关节的新按键被忽略。
        directions_[binding->joint].store(0);
        RCLCPP_WARN(get_logger(), "Opposite jog keys pressed for joint %zu", binding->joint + 1U);
      }
    }
    // 发布最新 Jog 指令
    publishCommand();
  }

  /**
   * @brief 循环读取并处理 Jog 键盘事件
   */
  void inputLoop()
  {
    pollfd descriptor{};
    descriptor.fd = input_fd_;
    descriptor.events = POLLIN;
    while (rclcpp::ok() && !stop_requested_.load()) {
      descriptor.revents = 0;
      const int poll_result = ::poll(&descriptor, 1, poll_timeout_ms_);
      if (poll_result < 0) {
        if (errno == EINTR) {
          continue;
        }
        input_fault_.store(true);
        break;
      }
      if (poll_result == 0) {
        continue;
      }
      if ((descriptor.revents & POLLIN) != 0) {
        input_event event{};
        while (::read(input_fd_, &event, sizeof(event)) == sizeof(event)) {
          if (event.type == EV_KEY) {
            handleKey(event);
          }
        }
      }
      if ((descriptor.revents & (POLLERR | POLLHUP | POLLNVAL)) != 0) {
        input_fault_.store(true);
        break;
      }
    }
    clearDirections();
    publishCommand();
    if (input_fault_.load()) {
      RCLCPP_ERROR(get_logger(), "Keyboard input device failed or disconnected");
    }
  }

  std::string input_device_;
  int poll_timeout_ms_{50};
  int input_fd_{-1};
  std::array<std::atomic<std::int8_t>, 6> directions_{};
  std::atomic<int> active_key_{-1};
  std::atomic<bool> emergency_stop_requested_{false};
  std::atomic<bool> input_fault_{false};
  std::atomic<bool> stop_requested_{false};
  std::atomic<std::uint32_t> sequence_{0};
  std::mutex publish_mutex_;
  rclcpp::Publisher<elfin3_interfaces::msg::JogCommand>::SharedPtr command_publisher_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
  std::thread input_thread_;
};

}  // namespace elfin3_jog

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<elfin3_jog::KeyboardTeleopNode>());
  } catch (const std::exception & exception) {
    RCLCPP_FATAL(rclcpp::get_logger("elfin3_keyboard_teleop"), "%s", exception.what());
  }
  rclcpp::shutdown();
  return 0;
}
