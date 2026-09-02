#!/usr/bin/python3


# elfin3_gazebo.launch.py:
# Launch file for the elfin3 Robot GAZEBO SIMULATION in ROS2 Humble:

# Import libraries:
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.parameter_descriptions import ParameterValue
import xacro
import yaml

# LOAD FILE:
def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return file.read()
    except EnvironmentError:
        # parent of IOError, OSError *and* WindowsError where available.
        return None
# LOAD YAML:
def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        # parent of IOError, OSError *and* WindowsError where available.
        return None

# ========== **GENERATE LAUNCH DESCRIPTION** ========== #
def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # ***** GAZEBO ***** #   
    # DECLARE Gazebo WORLD file:
    elfin3_ros2_gazebo = os.path.join(
        get_package_share_directory('elfin3_ros2_gazebo'),
        'worlds',
        'elfin3.world')
    # DECLARE Gazebo LAUNCH file:
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
                launch_arguments={'world': elfin3_ros2_gazebo}.items(),
             )

    # ***** ROBOT DESCRIPTION ***** #
    # elfin3 Description file package:
    elfin3_description_path = os.path.join(
        get_package_share_directory('elfin3_ros2_gazebo'))
    # elfin3 ROBOT urdf file path:
    xacro_file = os.path.join(elfin3_description_path,
                              'urdf',
                              'elfin3_sim.urdf.xacro')
    # Generate ROBOT_DESCRIPTION for elfin3:
    robot_description_config = Command(
        [FindExecutable(name='xacro'), ' ', xacro_file,
        ' use_fake_hardware:=false',])
        
    robot_description = {'robot_description': robot_description_config}

    # ROBOT STATE PUBLISHER NODE:
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[
            robot_description,
            {
                'use_sim_time': ParameterValue(
                    use_sim_time, value_type=bool
                )
            },
        ]
    )

    # SPAWN ROBOT TO GAZEBO:
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'elfin3',"-x", "0.0", "-y", "0.0", "-z", "0.1"],
                        output='screen')

    # ***** CONTROLLERS ***** #
    # Joint STATE Controller:
    # load_joint_state_broadcaster = ExecuteProcess(
    #     cmd=['ros2', 'control', 'load_start_controller', 'joint_state_broadcaster'],
    #     output='screen'
    # )
    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )

    # Joint TRAJECTORY Controller:
    # load_joint_trajectory_controller = ExecuteProcess(
    #     cmd=['ros2', 'control', 'load_start_controller', 'joint_trajectory_controller'],
    #     output='screen'
    # )
    load_joint_trajectory_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'elfin_arm_controller'],
        output='screen'
    )

    load_jog_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'inactive',
             'elfin_jog_controller'],
        output='screen'
    )

    close_evt1 =  RegisterEventHandler( 
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[load_joint_state_broadcaster],
            )
    )
    close_evt2 = RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_broadcaster,
                on_exit=[load_joint_trajectory_controller],
            )
    )
    close_evt3 = RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_trajectory_controller,
                on_exit=[load_jog_controller],
            )
    )

    # ***** RETURN LAUNCH DESCRIPTION ***** #
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use the Gazebo simulation clock.',
        ),
        gazebo, 
        spawn_entity,
        close_evt1,
        close_evt2,
        close_evt3,
        node_robot_state_publisher
    ])
