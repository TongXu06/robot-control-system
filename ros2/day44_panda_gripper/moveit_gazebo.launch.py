from pathlib import Path

from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    root = Path.home() / "ros2_ws/day44_panda_gripper"

    urdf = root / "panda_gripper.urdf"
    controllers = root / "moveit_gazebo_controllers.yaml"

    panda_share = Path(
        get_package_share_directory(
            "moveit_resources_panda_moveit_config"
        )
    )

    rviz_config = panda_share / "launch" / "moveit.rviz"

    config = (
        MoveItConfigsBuilder(
            "moveit_resources_panda",
            package_name="moveit_resources_panda_moveit_config"
        )
        .robot_description(file_path=str(urdf))
        .robot_description_semantic(
            file_path="config/panda.srdf"
        )
        .robot_description_kinematics(
            file_path="config/kinematics.yaml"
        )
        .joint_limits(
            file_path="config/joint_limits.yaml"
        )
        .trajectory_execution(
            file_path=str(controllers)
        )
        .planning_pipelines(
            pipelines=["ompl"],
            default_planning_pipeline="ompl"
        )
        .to_moveit_configs()
    )

    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            config.to_dict(),
            {"use_sim_time": True}
        ],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=[
            "-d",
            str(rviz_config)
        ],
        parameters=[
            config.robot_description,
            config.robot_description_semantic,
            config.robot_description_kinematics,
            config.planning_pipelines,
            config.joint_limits,
            {"use_sim_time": True}
        ],
    )

    return LaunchDescription([
        move_group,
        rviz
    ])

