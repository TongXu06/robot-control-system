
from setuptools import find_packages, setup

package_name = 'robot_state_demo'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        (
            'share/' + package_name,
            ['package.xml'],
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robotdev',
    maintainer_email='robotdev@example.com',
    description='ROS 2 robot joint state communication demo',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'joint_publisher = robot_state_demo.joint_publisher:main',
            'joint_subscriber = robot_state_demo.joint_subscriber:main',
        ],
    },
)

