#include <exception>
#include <memory>
#include <thread>

#include "elfin3_motion_command/motion_command_node.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char * argv[])
{
  // 初始化 ROS 2，并创建机械臂运动命令节点。
  rclcpp::init(argc, argv);
  const auto node =
    std::make_shared<elfin3_motion_command::MotionCommandNode>();

  // MoveIt 初始化期间需要处理服务、Action 和机器人状态回调，因此先让多线程
  // 执行器在后台运行，避免主线程等待 MoveIt 响应时无人处理 ROS 回调。
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  std::thread executor_thread([&executor]() {executor.spin();});

  try {
    // 建立 MoveGroupInterface、运动 Action/Stop 服务，并检查启动时的系统就绪状态。
    // supervisor 的启动检查主要用于记录状态；每条运动命令执行前仍会再次检查。
    node->initializeMoveGroup();
    node->checkSupervisorAtStartup();
  } catch (const std::exception & exception) {
    // 初始化失败时停止执行器并回收后台线程，确保 ROS 资源有序释放。
    RCLCPP_FATAL(
      node->get_logger(), "MoveIt initialization failed: %s", exception.what());
    executor.cancel();
    if (executor_thread.joinable()) {
      executor_thread.join();
    }
    rclcpp::shutdown();
    return 1;
  }

  // 正常运行时在此等待执行器退出；通常由 Ctrl+C 触发 ROS 2 关闭并使 spin() 返回。
  if (executor_thread.joinable()) {
    executor_thread.join();
  }

  // 完成 ROS 2 上下文和通信资源的最终清理。
  rclcpp::shutdown();
  return 0;
}
