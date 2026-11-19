Robot Control System — ROS 2, MoveIt 2 & Panda Manipulation

A 45-day robotics development project | Python · ROS 2 · Gazebo · MoveIt 2

A personal robotics project exploring robot kinematics, trajectory planning, control algorithms, ROS 2 communication, motion planning, and robotic manipulation.

Project Highlights

* Implemented forward and inverse kinematics for robotic arm control.
* Developed cubic and quintic trajectory planning methods.
* Explored Jacobian-based velocity control and singularity handling.
* Built ROS 2 applications using topics, services, and actions.
* Integrated a 7-DOF Franka Emika Panda robot with Gazebo and MoveIt 2.
* Successfully demonstrated simulated cube grasping and lifting with a two-finger gripper.

Demo — Panda Robot Grasping

Final result: The Panda robot successfully approached a simulated cube, closed its gripper around the object, and lifted it from the ground in Gazebo.

View the demonstration video and screenshots

The demonstration uses manually selected target poses in RViz, with motion trajectories planned by MoveIt 2 and executed through ROS 2 controllers.

Scope: This is a motion-planned, manually guided grasping demonstration, not a fully autonomous pick-and-place system.

Technology Stack

Category	Technologies
Programming	Python 3, NumPy
Robot Framework	ROS 2 Jazzy
Motion Planning	MoveIt 2
Simulation	Gazebo Harmonic
Visualization	RViz2
Robot Control	ros2_control
Robot Description	URDF, SDF
Development Environment	Ubuntu 24.04, WSL2
Version Control	Git, GitHub

Technical Modules

1. Robot Kinematics

Implemented fundamental robot kinematics, including forward kinematics, inverse kinematics, Jacobian matrices, and end-effector velocity control.

2. Trajectory Planning

Explored joint-space and Cartesian-space trajectory generation, cubic and quintic interpolation, and trajectory visualization.

3. Robot Control and Safety

Developed joint position control, motion history recording, joint-limit checks, and velocity-control experiments.

4. ROS 2 Communication

Built ROS 2 applications using publisher/subscriber communication, services, actions, custom interfaces, and robot joint-state monitoring.

5. Gazebo and MoveIt 2 Integration

Configured the Panda robot model, controllers, gripper, and simulated environment. Used MoveIt 2 to generate and execute collision-checked robot trajectories.

6. Grasping Demonstration

Completed a simulated grasp-and-lift sequence involving:

1. Moving the robotic arm above the target cube.
2. Aligning the two-finger gripper.
3. Planning and executing the approach trajectory.
4. Closing the gripper around the cube.
5. Lifting the grasped object in Gazebo.

Repository Structure

robot-control-system/
├── README.md
├── day18_safety.py
├── day19_trajectory.py
├── day20_smooth_trajectory.py
├── day21_quintic_trajectory.py
├── day22_forward_kinenatics.py
├── day23_inverse_kinematics.py
├── day24_cartesian_trajectory.py
├── day25_jacobian.py
├── day26_velocity_control.py
├── day27_singularity.py
├── day28_robot_control_v1.py
├── main.py
├── visualize.py
├── ros2/
│   ├── robot_state_demo/
│   ├── robot_interfaces/
│   ├── day40_arm_moveit_config/
│   ├── day42_panda_obstacles/
│   ├── day43_panda_gazebo/
│   ├── day44_panda_gripper/
│   └── day45_panda_auto/
├── ros2_projects/
└── media/

The day45_panda_auto folder contains experimental motion-planning scripts. It does not represent a verified end-to-end autonomous grasping pipeline.

Running the Project

Python Control Simulation

Install Python 3 and run:

python main.py

ROS 2 Simulation Environment

The ROS 2 portion was developed using Ubuntu 24.04, ROS 2 Jazzy, Gazebo Harmonic, and MoveIt 2.

For the ROS 2 packages, copy the relevant packages into a ROS 2 workspace, install their dependencies, and build using colcon.

The Panda demonstration also uses the URDF, controller configuration, Gazebo world, and MoveIt configuration provided in this repository.

Some launch and simulation scripts contain workspace-specific paths and may need adjustment before running on another computer.

Project Outcomes

This project provided practical experience in:

* Mathematical foundations of robot manipulation.
* Python-based robotics algorithm implementation.
* ROS 2 software architecture and communication.
* Robot motion planning and simulation.
* Integration and debugging of robotic controllers.
* Simulated object grasping and lifting.

Limitations and Future Improvements

Current limitations include manual grasp-target selection, experimental automation scripts, and environment-specific configuration.

Potential future work includes vision-based object localization, automated grasp pose generation, complete pick-and-place execution, and more reproducible launch configurations.

Author

Tong Xu

Robotics and Embodied AI Enthusiast

GitHub: https://github.com/TongXu06

Project Repository: https://github.com/TongXu06/robot-control-system
