
import os

from ament_index_python.packages import (
    get_package_share_directory
)

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    package_dir = get_package_share_directory(
        "robot_state_demo"
    )

    config_file = os.path.join(
        package_dir,
        "config",
        "robot_params.yaml"
    )

    rate_argument = DeclareLaunchArgument(
        "rate",
        default_value="2.0",
        description="JointState publishing frequency"
    )

    publisher_node = Node(
        package="robot_state_demo",
        executable="joint_state_publisher",
        name="robot_state_publisher_demo",
        output="screen",
        parameters=[
            config_file,
            {
                "publish_rate": ParameterValue(
                    LaunchConfiguration("rate"),
                    value_type=float
                )
            }
        ]
    )

    monitor_node = Node(
        package="robot_state_demo",
        executable="joint_state_monitor",
        name="joint_state_monitor",
        output="screen"
    )

    return LaunchDescription([
        rate_argument,
        publisher_node,
        monitor_node
    ])

