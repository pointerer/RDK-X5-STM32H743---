from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    dataset = LaunchConfiguration("dataset")
    execution_mode = LaunchConfiguration("execution_mode")
    output_root = LaunchConfiguration("output_root")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "dataset",
                description=(
                    "Required explicit A/B dataset YAML path. No default is "
                    "provided to prevent accidental real-hardware execution."
                ),
            ),
            DeclareLaunchArgument(
                "execution_mode",
                default_value="",
                description="Optional simulation or real override; empty uses the dataset.",
            ),
            DeclareLaunchArgument(
                "output_root",
                default_value="results",
                description="Directory under which timestamped reports are created.",
            ),
            Node(
                package="elfin3_ab_benchmark",
                executable="ab_benchmark",
                name="elfin3_ab_benchmark",
                output="screen",
                parameters=[
                    {
                        "dataset": dataset,
                        "execution_mode": execution_mode,
                        "output_root": output_root,
                    }
                ],
            ),
        ]
    )
