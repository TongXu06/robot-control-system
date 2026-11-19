import argparse
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from sensor_msgs.msg import JointState
from trajectory_msgs.msg import (
    JointTrajectory,
    JointTrajectoryPoint
)


JOINT_NAMES = [
    "shoulder_joint",
    "elbow_joint"
]

JOINT_LIMITS = [
    (0.0, math.pi / 2),
    (-math.pi / 2, math.pi / 2)
]

MAX_SPEED = 1.0  # rad/s, demo planning limit


def generate_quintic(start, goal, duration, steps):
    """
    Generate position, velocity and acceleration.
    """

    trajectory = []

    for i in range(steps + 1):
        t = duration * i / steps
        u = t / duration

        s = 10*u**3 - 15*u**4 + 6*u**5

        ds = (
            30*u**2 - 60*u**3 + 30*u**4
        ) / duration

        dds = (
            60*u - 180*u**2 + 120*u**3
        ) / (duration**2)

        positions = []
        velocities = []
        accelerations = []

        for q0, qf in zip(start, goal):
            delta = qf - q0

            positions.append(q0 + delta*s)
            velocities.append(delta*ds)
            accelerations.append(delta*dds)

        trajectory.append((
            t,
            positions,
            velocities,
            accelerations
        ))

    return trajectory


class TrajectoryPublisher(Node):

    def __init__(self):
        super().__init__("day38_trajectory_publisher")

        self.current_positions = None

        self.create_subscription(
            JointState,
            "/joint_states",
            self.joint_callback,
            10
        )

        self.publisher = self.create_publisher(
            JointTrajectory,
            "/arm_trajectory_controller/joint_trajectory",
            10
        )

    def joint_callback(self, msg):

        if not all(
            name in msg.name for name in JOINT_NAMES
        ):
            return

        values = []

        for name in JOINT_NAMES:
            index = msg.name.index(name)
            if index >= len(msg.position):
                return
            values.append(float(msg.position[index]))

        self.current_positions = values

    def wait_for_joint_state(self, timeout=10.0):

        start_time = time.monotonic()

        while rclpy.ok():
            if self.current_positions is not None:
                return True

            if time.monotonic() - start_time > timeout:
                return False

            rclpy.spin_once(self, timeout_sec=0.1)

        return False

    def wait_for_controller(self, timeout=10.0):

        start_time = time.monotonic()

        while rclpy.ok():
            if self.publisher.get_subscription_count() > 0:
                return True

            if time.monotonic() - start_time > timeout:
                return False

            rclpy.spin_once(self, timeout_sec=0.1)

        return False

    def send_trajectory(self, trajectory):

        msg = JointTrajectory()

        msg.joint_names = JOINT_NAMES

        # Zero stamp: start as soon as accepted.
        msg.header.stamp.sec = 0
        msg.header.stamp.nanosec = 0

        for t, q, dq, ddq in trajectory:

            point = JointTrajectoryPoint()

            point.positions = q
            point.velocities = dq
            point.accelerations = ddq

            point.time_from_start = Duration(
                seconds=t
            ).to_msg()

            msg.points.append(point)

        self.publisher.publish(msg)

        self.get_logger().info(
            f"Published {len(msg.points)} trajectory points"
        )


def main():

    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--shoulder",
        type=float,
        default=60.0
    )

    parser.add_argument(
        "--elbow",
        type=float,
        default=30.0
    )

    parser.add_argument(
        "--duration",
        type=float,
        default=6.0
    )

    args = parser.parse_args()

    if args.duration <= 0:
        parser.error("Duration must be positive")

    rclpy.init()

    node = TrajectoryPublisher()

    try:
        print("\nDay 38: Joint Trajectory Control")

        if not node.wait_for_joint_state():
            raise RuntimeError(
                "No joint states received"
            )

        if not node.wait_for_controller():
            raise RuntimeError(
                "Trajectory controller not available"
            )

        start = node.current_positions.copy()

        goal = [
            math.radians(args.shoulder),
            math.radians(args.elbow)
        ]

        for i, (low, high) in enumerate(JOINT_LIMITS):
            if not low <= goal[i] <= high:
                raise ValueError(
                    f"{JOINT_NAMES[i]} target out of range"
                )

        # Approximate peak quintic velocity.
        peak_speeds = [
            1.875 * abs(g - s) / args.duration
            for s, g in zip(start, goal)
        ]

        if max(peak_speeds) > MAX_SPEED:
            raise ValueError(
                "Trajectory is too fast. "
                "Increase --duration."
            )

        steps = max(
            2,
            math.ceil(args.duration / 0.2)
        )

        trajectory = generate_quintic(
            start,
            goal,
            args.duration,
            steps
        )

        print("Start (deg):", [
            round(math.degrees(x), 2)
            for x in start
        ])

        print("Goal (deg):", [
            round(math.degrees(x), 2)
            for x in goal
        ])

        print(
            "Peak planned speed (deg/s):",
            [
                round(math.degrees(x), 2)
                for x in peak_speeds
            ]
        )

        node.send_trajectory(trajectory)
        motion_start = time.monotonic()
        # Allow time for Gazebo to execute and
        # for /joint_states feedback to arrive.
        deadline = (
            time.monotonic()
            + args.duration * 4
            + 5.0
        )

        final_error = float("inf")

        while rclpy.ok() and time.monotonic() < deadline:

            rclpy.spin_once(
                node,
                timeout_sec=0.1
            )

            if node.current_positions is None:
                continue

            errors = [
                abs(actual - target)
                for actual, target in zip(
                    node.current_positions, goal
                )
            ]

            final_error = max(errors)

            if (
                time.monotonic() - motion_start >= args.duration
                and final_error < math.radians(1.0)
            ):
                break

        print(
            "Measured final angles (deg):",
            [
                round(math.degrees(x), 2)
                for x in node.current_positions
            ]
        )

        print(
            "Final position error (deg):",
            round(math.degrees(final_error), 3)
        )

        print(
            "Within 1-degree tolerance:",
            final_error < math.radians(1.0)
        )

        print(
            "Check Gazebo and controller_state "
            "to verify full trajectory tracking."
        )

    except (RuntimeError, ValueError) as error:
        node.get_logger().error(str(error))

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
