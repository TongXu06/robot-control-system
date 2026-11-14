import os
from glob import glob
from setuptools import find_packages, setup

package_name = "robot_state_demo"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        (
            "share/" + package_name,
            ["package.xml"],
        ),
        (
            os.path.join("share", package_name, "launch"),
            glob("launch/*.launch.py"),
        ),
        (
            os.path.join("share", package_name, "config"),
            glob("config/*.yaml"),
        ),
        (
            os.path.join("share", package_name, "urdf"),
            glob("urdf/*.urdf"),
        ),
        (
            os.path.join("share", package_name, "worlds"),
            glob("worlds/*.sdf"),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="robotdev",
    maintainer_email="robotdev@example.com",
    description="ROS 2 robot arm control and simulation",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "day38_trajectory_publisher = robot_state_demo.day38_trajectory_publisher:main",
            "joint_publisher = robot_state_demo.joint_publisher:main",
            "joint_subscriber = robot_state_demo.joint_subscriber:main",
            "joint_service = robot_state_demo.joint_service:main",
            "joint_client = robot_state_demo.joint_client:main",
            "move_action_server = robot_state_demo.move_action_server:main",
            "move_action_client = robot_state_demo.move_action_client:main",
            "joint_state_publisher = robot_state_demo.joint_state_publisher:main",
            "joint_state_monitor = robot_state_demo.joint_state_monitor:main",
            "joint_state_bridge = robot_state_demo.joint_state_bridge:main",
        ],
    },
)

