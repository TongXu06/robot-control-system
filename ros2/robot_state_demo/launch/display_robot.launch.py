
import os

from ament_index_python.packages import (
    get_package_share_directory
)

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    package_dir = get_package_share_directory(
        "robot_state_demo"
    )

    urdf_file = os.path.join(
        package_dir,
        "urdf",
        "two_link_arm.urdf"
    )

    config_file = os.path.join(
        package_dir,
        "config",
        "robot_params.yaml"
    )

    # Read the URDF description
    with open(urdf_file, "r", encoding="utf-8") as file:
        robot_description = file.read()

    # Official ROS 2 robot state publisher
    robot_state_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {"robot_description": robot_description}
        ]
    )

    # Our Day 33 joint angle publisher
    joint_state_node = Node(
        package="robot_state_demo",
        executable="joint_state_publisher",
        name="robot_state_publisher_demo",
        output="screen",
        parameters=[config_file]
    )

    # Open RViz
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen"
    )

    return LaunchDescription([
        robot_state_node,
        joint_state_node,
        rviz_node
    ])
