
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

    urdf_path = os.path.join(
        package_dir,
        "urdf",
        "two_link_arm.urdf"
    )

    with open(urdf_path, "r", encoding="utf-8") as f:
        robot_description = f.read()

    # Day 32 Action Server
    action_server = Node(
        package="robot_state_demo",
        executable="move_action_server",
        name="move_action_server",
        output="screen"
    )

    # Convert /joint_angles to /joint_states
    joint_bridge = Node(
        package="robot_state_demo",
        executable="joint_state_bridge",
        name="joint_state_bridge",
        output="screen"
    )

    # Convert URDF + JointState to TF
    robot_state = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{
            "robot_description": robot_description
        }]
    )

    # Visualize the robot
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen"
    )

    return LaunchDescription([
        action_server,
        joint_bridge,
        robot_state,
        rviz
    ])

