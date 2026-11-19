
from setuptools import find_packages, setup

package_name = 'day42_panda_obstacles'

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
    description='Panda collision object management for MoveIt 2',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'obstacle_manager = day42_panda_obstacles.obstacle_manager:main',
        ],
    },
)
