from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


# 根据功能包名和文件名构造子 Launch 文件的描述源。
# 实际路径会在 Launch 运行时解析为：<package_share>/launch/<file_name>。
def _launch_file(package_name, file_name):
    return PythonLaunchDescriptionSource(
        PathJoinSubstitution([FindPackageShare(package_name), "launch", file_name])
    )


def generate_launch_description():
    # ==================== Launch 启动参数 ====================
    # LaunchConfiguration 是运行时参数引用，实际值由文件末尾的
    # DeclareLaunchArgument 默认值或 ros2 launch 命令行参数提供。
    can_interface = LaunchConfiguration("can_interface")
    start_rviz = LaunchConfiguration("start_rviz")
    start_keyboard_jog = LaunchConfiguration("start_keyboard_jog")
    jog_input_device = LaunchConfiguration("jog_input_device")
    rviz_config = LaunchConfiguration("rviz_config")
    supervisor_terminal_output = LaunchConfiguration("supervisor_terminal_output")

    # ==================== 真机机器人描述与控制器配置 ====================
    # 加载真机 Xacro，并将 SocketCAN 接口名传给 CAN FD 硬件插件。
    # xacro 命令的输出是完整 URDF XML，作为 robot_description 参数使用。
    real_xacro = PathJoinSubstitution(
        [FindPackageShare("elfin3_canfd_driver"), "urdf", "elfin3_real.urdf.xacro"]
    )
    robot_description = {
        "robot_description": ParameterValue(
            Command(["xacro ", real_xacro, " can_interface:=", can_interface]),
            value_type=str,
        )
    }

    # ros2_control 的真机控制器配置，包括状态广播、轨迹和 Jog 控制器。
    controllers = PathJoinSubstitution(
        [
            FindPackageShare("elfin3_canfd_driver"),
            "config",
            "elfin_arm_controller_real.yaml",
        ]
    )

    # ==================== ros2_control 与机器人状态 ====================
    # 启动 Controller Manager，加载真机硬件接口和控制器配置。
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="screen",
        parameters=[robot_description, controllers, {"use_sim_time": False}],
    )

    # 根据 URDF 和 /joint_states 发布机械臂各连杆的 TF。
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description, {"use_sim_time": False}],
    )

    # ==================== 控制器加载 ====================
    # 发布关节位置、速度等状态，供 TF、MoveIt 和 RViz 使用。
    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    # 加载并激活机械臂轨迹控制器，执行 MoveIt 下发的关节轨迹。
    arm_controller = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=[
            "elfin_arm_controller",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    # 预加载 Jog 控制器但保持未激活，避免与轨迹控制器同时占用关节接口。
    jog_controller = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=[
            "elfin_jog_controller",
            "--controller-manager",
            "/controller_manager",
            "--inactive",
        ],
    )

    # ==================== Jog 手动控制 ====================
    # 管理轨迹控制器与 Jog 控制器的模式切换；真机启用 CAN FD 安全处理。
    jog_mode_manager = Node(
        package="elfin3_jog",
        executable="elfin3_jog_mode_manager",
        output="screen",
        parameters=[
            {
                "use_sim_time": False,
                "canfd_safety_enabled": True,
                "command_timeout_sec": 0.50,
            }
        ],
    )

    # 可选的 evdev 键盘输入节点，仅在 start_keyboard_jog=true 时启动。
    keyboard_jog = Node(
        package="elfin3_jog",
        executable="elfin3_keyboard_teleop",
        output="screen",
        condition=IfCondition(start_keyboard_jog),
        parameters=[{"use_sim_time": False, "input_device": jog_input_device}],
    )

    # ==================== MoveIt 与 RViz ====================
    # 启动 move_group，负责运动规划、碰撞检测和轨迹执行。
    move_group = IncludeLaunchDescription(
        _launch_file("elfin3_moveit_config", "move_group.launch.py"),
        launch_arguments={"use_sim_time": "false"}.items(),
    )

    # 可选启动 MoveIt RViz 客户端，并允许从命令行指定 RViz 配置文件。
    rviz = IncludeLaunchDescription(
        _launch_file("elfin3_moveit_config", "moveit_rviz.launch.py"),
        condition=IfCondition(start_rviz),
        launch_arguments={
            "use_sim_time": "false",
            "rviz_config": rviz_config,
        }.items(),
    )

    # ==================== 系统监督节点 ====================
    # 启动项目级监督节点，协调和监控机械臂系统状态。
    supervisor = Node(
        package="elfin3_supervisor",
        executable="elfin3_supervisor_node",
        output="screen",
        parameters=[
            {
                "use_sim_time": False,
                "hardware_control_loop_required": True,
                "hardware_control_loop_timeout_sec": 3.0,
                "terminal_output": ParameterValue(
                    supervisor_terminal_output, value_type=bool
                ),
            }
        ],
    )

    # ==================== 上层运动命令节点 ====================
    # 为 Motion Command 节点装配机器人语义描述、运动学、关节限制和
    # OMPL 规划管线参数，使其能够创建 MoveIt RobotModel 和规划接口。
    motion_moveit_config = (
        MoveItConfigsBuilder("elfin3", package_name="elfin3_moveit_config")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .planning_pipelines(
            default_planning_pipeline="ompl",
            pipelines=["ompl"],
            load_all=False,
        )
        .to_moveit_configs()
    )

    # 加载运动命令业务参数；取消运动时通过 CAN FD hold 服务保持真机。
    motion_command = Node(
        package="elfin3_motion_command",
        executable="elfin3_motion_command_node",
        output="screen",
        parameters=[
            PathJoinSubstitution(
                [
                    FindPackageShare("elfin3_motion_command"),
                    "config",
                    "motion_command.yaml",
                ]
            ),
            motion_moveit_config.robot_description,
            motion_moveit_config.robot_description_semantic,
            motion_moveit_config.robot_description_kinematics,
            motion_moveit_config.joint_limits,
            {
                "use_sim_time": False,
                "canfd_hold_on_cancel": True,
                "canfd_hold_service": "/elfin3_canfd/hold",
            },
        ],
    )

    # ==================== 汇总启动动作 ====================
    # 除 motion_command 外，各节点/子 Launch 会异步启动；控制器 spawner
    # 会等待 Controller Manager。Motion Command 延迟 5 秒启动，为硬件、
    # 控制器、关节状态和 move_group 的初始化预留时间。
    return LaunchDescription(
        [
            # 对外暴露的 Launch 参数及其默认值。
            DeclareLaunchArgument(
                "can_interface",
                default_value="can0",
                description="SocketCAN interface owned by the real hardware plugin.",
            ),
            DeclareLaunchArgument(
                "start_rviz",
                default_value="true",
                description="Start the MoveIt RViz client.",
            ),
            DeclareLaunchArgument(
                "start_keyboard_jog",
                default_value="false",
                description="Start the evdev keyboard Jog input node.",
            ),
            DeclareLaunchArgument(
                "supervisor_terminal_output",
                default_value="true",
                description=(
                    "Render the Supervisor dashboard. Disable it for long automated "
                    "runs to prevent terminal-output backpressure from blocking callbacks."
                ),
            ),
            DeclareLaunchArgument(
                "jog_input_device",
                default_value="/dev/input/event0",
                description="Linux evdev device used by keyboard Jog.",
            ),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("elfin3_moveit_config"),
                        "config",
                        "moveit.rviz",
                    ]
                ),
                description="Path to the MoveIt RViz configuration.",
            ),
            control_node,
            robot_state_publisher,
            joint_state_broadcaster,
            arm_controller,
            jog_controller,
            jog_mode_manager,
            keyboard_jog,
            move_group,
            rviz,
            supervisor,
            # 上层命令节点依赖前述基础组件，因此最后延迟启动。
            TimerAction(period=5.0, actions=[motion_command]),
        ]
    )
