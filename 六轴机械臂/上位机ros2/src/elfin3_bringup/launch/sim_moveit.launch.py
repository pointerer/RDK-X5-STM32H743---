from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def _launch_file(package_name, file_name):
    return PythonLaunchDescriptionSource(
        PathJoinSubstitution(
            [FindPackageShare(package_name), "launch", file_name]
        )
    )


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    start_rviz = LaunchConfiguration("start_rviz")
    start_keyboard_jog = LaunchConfiguration("start_keyboard_jog")
    jog_input_device = LaunchConfiguration("jog_input_device")
    rviz_config = LaunchConfiguration("rviz_config")

    motion_moveit_config = (
        MoveItConfigsBuilder(
            "elfin3", package_name="elfin3_moveit_config"
        )
        .robot_description_kinematics(
            file_path="config/kinematics.yaml"
        )
        .joint_limits(file_path="config/joint_limits.yaml")
        .planning_pipelines(
            default_planning_pipeline="ompl",
            pipelines=["ompl"],
            load_all=False,
        )
        .to_moveit_configs()
    )

    gazebo = IncludeLaunchDescription(
        _launch_file(
            "elfin3_ros2_gazebo", "elfin3_gazebo.launch.py"
        ),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

    move_group = IncludeLaunchDescription(
        _launch_file(
            "elfin3_moveit_config", "move_group.launch.py"
        ),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

    rviz = IncludeLaunchDescription(
        _launch_file(
            "elfin3_moveit_config", "moveit_rviz.launch.py"
        ),
        condition=IfCondition(start_rviz),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "rviz_config": rviz_config,
        }.items(),
    )

    supervisor = Node(
        package="elfin3_supervisor",
        executable="elfin3_supervisor_node",
        output="screen",
        parameters=[
            {
                "use_sim_time": ParameterValue(
                    use_sim_time, value_type=bool
                ),
                "hardware_control_loop_required": False,
            }
        ],
    )

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
                "use_sim_time": ParameterValue(
                    use_sim_time, value_type=bool
                )
            },
        ],
    )

    jog_mode_manager = Node(
        package="elfin3_jog",
        executable="elfin3_jog_mode_manager",
        output="screen",
        parameters=[
            {
                "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                "canfd_safety_enabled": False,
            }
        ],
    )

    keyboard_jog = Node(
        package="elfin3_jog",
        executable="elfin3_keyboard_teleop",
        output="screen",
        condition=IfCondition(start_keyboard_jog),
        parameters=[
            {
                "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                "input_device": jog_input_device,
            }
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use the Gazebo simulation clock.",
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
            # Keep the includes in logical order. Their child processes start
            # asynchronously and use ROS graph discovery while Gazebo loads.
            gazebo,
            move_group,
            rviz,
            supervisor,
            jog_mode_manager,
            keyboard_jog,
            # Give Gazebo, controllers, move_group, and joint states a short
            # startup window before MotionCommandNode performs its checks.
            TimerAction(period=5.0, actions=[motion_command]),
        ]
    )
