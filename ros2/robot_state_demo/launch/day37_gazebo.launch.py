import os

from ament_index_python.packages import (
    get_package_share_directory
)

from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    TimerAction
)
from launch.launch_description_sources import (
    PythonLaunchDescriptionSource
)
from launch_ros.actions import Node


def generate_launch_description():

    package_dir = get_package_share_directory(
        "robot_state_demo"
    )

    gazebo_pkg = get_package_share_directory(
        "ros_gz_sim"
    )

    world_file = os.path.join(
        package_dir, "worlds", "day36_world.sdf"
    )

    robot_file = os.path.join(
        package_dir, "urdf", "day37_controlled_arm.urdf"
    )

    with open(robot_file, "r", encoding="utf-8") as f:
        robot_description = f.read()

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                gazebo_pkg, "launch", "gz_sim.launch.py"
            )
        ),
        launch_arguments={
            "gz_args": f"-r {world_file} -v 4"
        }.items()
    )

    # Publish robot_description for controller_manager
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{
            "robot_description": robot_description,
            "use_sim_time": True
        }]
    )

    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "--world", "day36_world",
            "--file", robot_file,
            "--name", "day37_robot_arm",
            "--x", "0.7",
            "--y", "0.0",
            "--z", "0.0"
        ],
        output="screen"
    )

    delayed_spawn = TimerAction(
        period=5.0,
        actions=[spawn_robot]
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        delayed_spawn
    ])

