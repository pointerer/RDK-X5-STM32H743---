#include <chrono>
#include <csignal>
#include <exception>
#include <iostream>
#include <memory>
#include <thread>

#include "elfin3_console/console_node.hpp"
#include "rclcpp/rclcpp.hpp"

namespace
{

volatile std::sig_atomic_t shutdown_signal = 0;

void handleSignal(int signal_number)
{
  shutdown_signal = signal_number;
}

}  // namespace

int main(int argc, char * argv[])
{
  rclcpp::init(
    argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
  std::signal(SIGINT, handleSignal);
  std::signal(SIGTERM, handleSignal);

  try {
    const auto node = std::make_shared<elfin3_console::ConsoleNode>();
    node->start();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);

    std::exception_ptr executor_exception;
    std::thread executor_thread(
      [&executor, &executor_exception]()
      {
        try {
          executor.spin();
        } catch (...) {
          executor_exception = std::current_exception();
          rclcpp::shutdown();
        }
      });

    while (rclcpp::ok() && shutdown_signal == 0) {
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    if (shutdown_signal != 0 && rclcpp::ok()) {
      node->prepareForShutdown();
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      rclcpp::shutdown();
    }

    if (executor_thread.joinable()) {
      executor_thread.join();
    }
    if (executor_exception) {
      std::rethrow_exception(executor_exception);
    }
  } catch (const std::exception & exception) {
    std::cerr << "Failed to run elfin3_console: " << exception.what() << '\n';
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
    return 1;
  }

  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  return 0;
}
