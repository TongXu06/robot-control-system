from setuptools import find_packages, setup

package_name = 'day43_panda_gazebo'

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
        (
            'share/' + package_name + '/config',
            ['config/controllers.yaml'],
        ),
        (
            'share/' + package_name + '/urdf',
            ['urdf/panda_gazebo.urdf'],
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robotdev',
    maintainer_email='robotdev@example.com',
    description='Day 43 Panda Gazebo simulation',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'move_arm = day43_panda_gazebo.move_arm:main',
        ],
    },
)

