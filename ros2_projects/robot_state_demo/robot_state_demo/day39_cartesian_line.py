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


# Day 37 robot geometry, metres
L1 = 0.12
L2 = 0.10

JOINTS = ["shoulder_joint", "elbow_joint"]

# Joint limits, radians
LIMITS = [
    (0.0, math.pi / 2),
    (-math.pi / 2, math.pi / 2),
]

MAX_JOINT_SPEED = 1.0  # rad/s
MAX_JOINT_ACCEL = 2.0  # rad/s^2
MIN_ELBOW_ANGLE = math.radians(3.0)


def fk(q):
    """Forward kinematics, relative to shoulder joint."""
    q1, q2 = q

    x = L1 * math.cos(q1) + L2 * math.cos(q1 + q2)
    y = L1 * math.sin(q1) + L2 * math.sin(q1 + q2)

    return (x, y)


def ik(x, y, elbow_sign=1):
    """Inverse kinematics for a planar two-link arm."""

    r2 = x*x + y*y

    c2 = (r2 - L1*L1 - L2*L2) / (2*L1*L2)

    if abs(c2) > 1.0 + 1e-9:
        raise ValueError(
            f"Unreachable point: ({x:.4f}, {y:.4f})"
        )

    c2 = max(-1.0, min(1.0, c2))

    s2 = elbow_sign * math.sqrt(max(0.0, 1.0-c2*c2))

    q2 = math.atan2(s2, c2)

    q1 = (
        math.atan2(y, x)
        - math.atan2(
            L2*math.sin(q2),
            L1+L2*math.cos(q2)
        )
    )

    return [q1, q2]


def quintic(u):
    """Smooth time scaling with zero endpoint derivatives."""
    return 10*u**3 - 15*u**4 + 6*u**5


def plan_line(start_q, target, duration, steps):
    """
    Plan a Cartesian straight line, then solve IK
    for each waypoint.
    """

    start_xy = fk(start_q)

    # Choose the elbow branch from the current pose.
    elbow_sign = 1 if start_q[1] >= 0 else -1

    times = []
    positions = []

    for i in range(steps + 1):
        t = duration*i/steps
        u = i/steps
        s = quintic(u)

        x = start_xy[0] + s*(target[0]-start_xy[0])
        y = start_xy[1] + s*(target[1]-start_xy[1])

        if i == 0:
            q = list(start_q)
        else:
            q = ik(x, y, elbow_sign)

        for j, angle in enumerate(q):
            low, high = LIMITS[j]

            if angle < low-1e-7 or angle > high+1e-7:
                raise ValueError(
                    f"{JOINTS[j]} limit exceeded "
                    f"at point {i}: "
                    f"{math.degrees(angle):.2f} deg"
                )

        if abs(math.sin(q[1])) < math.sin(MIN_ELBOW_ANGLE):
            raise ValueError(
                f"Near singularity at point {i}. "
                "Choose another path."
            )

        times.append(t)
        positions.append(q)

    dt = duration/steps

    # Numerically estimate joint velocities.
    velocities = [[0.0, 0.0] for _ in positions]

    for i in range(1, steps):
        for j in range(2):
            velocities[i][j] = (
                positions[i+1][j] - positions[i-1][j]
            ) / (2*dt)

    # Numerically estimate accelerations.
    accelerations = [[0.0, 0.0] for _ in positions]

    for i in range(1, steps):
        for j in range(2):
            accelerations[i][j] = (
                positions[i+1][j]
                - 2*positions[i][j]
                + positions[i-1][j]
            ) / (dt*dt)

    max_speed = max(
        abs(v)
        for pair in velocities
        for v in pair
    )

    max_accel = max(
        abs(a)
        for pair in accelerations
        for a in pair
    )

    if max_speed > MAX_JOINT_SPEED:
        raise ValueError(
            f"Joint speed too high: {max_speed:.3f} rad/s. "
            "Increase duration."
        )

    if max_accel > MAX_JOINT_ACCEL:
        raise ValueError(
            f"Joint acceleration too high: "
            f"{max_accel:.3f} rad/s^2. "
            "Increase duration."
        )

    return (
        times,
        positions,
        velocities,
        accelerations,
        max_speed,
        max_accel
    )


class CartesianLineNode(Node):

    def __init__(self):
        super().__init__("day39_cartesian_line")

        self.current_q = None

        self.create_subscription(
            JointState,
            "/joint_states",
            self.state_callback,
            10
        )

        self.pub = self.create_publisher(
            JointTrajectory,
            "/arm_trajectory_controller/joint_trajectory",
            10
        )

    def state_callback(self, msg):
        if not all(n in msg.name for n in JOINTS):
            return

        indices = [msg.name.index(n) for n in JOINTS]

        if any(i >= len(msg.position) for i in indices):
            return

        self.current_q = [
            float(msg.position[i]) for i in indices
        ]

    def wait_for_ready(self, timeout=15.0):
        start = time.monotonic()

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)

            if (
                self.current_q is not None
                and self.pub.get_subscription_count() > 0
            ):
                return

            if time.monotonic() - start > timeout:
                raise RuntimeError(
                    "No joint states or trajectory subscriber. "
                    "Check active controllers."
                )

    def send(self, times, positions, velocities, accelerations):

        msg = JointTrajectory()
        msg.joint_names = JOINTS

        # Zero stamp means start on receipt.
        for t, q, dq, ddq in zip(
            times, positions, velocities, accelerations
        ):
            point = JointTrajectoryPoint()

            point.positions = q
            point.velocities = dq
            point.accelerations = ddq
            point.time_from_start = Duration(
                seconds=t
            ).to_msg()

            msg.points.append(point)

        self.pub.publish(msg)

        self.get_logger().info(
            f"Sent {len(msg.points)} Cartesian line waypoints"
        )


def main():
    parser = argparse.ArgumentParser()

    parser.add_argument("--x", type=float, default=0.14)
    parser.add_argument("--y", type=float, default=0.12)
    parser.add_argument("--duration", type=float, default=8.0)
    parser.add_argument("--dt", type=float, default=0.1)

    args = parser.parse_args()

    if args.duration <= 0 or args.dt <= 0:
        parser.error("Duration and dt must be positive")

    steps = max(2, math.ceil(args.duration/args.dt))

    rclpy.init()
    node = CartesianLineNode()

    try:
        node.wait_for_ready()

        start_q = list(node.current_q)
        start_xy = fk(start_q)
        target = (args.x, args.y)

        print("\n=== Day 39 Cartesian Straight Line ===")
        print("Start XY (m):", start_xy)
        print("Target XY (m):", target)
        print(
            "Start joints (deg):",
            [round(math.degrees(q), 2) for q in start_q]
        )

        result = plan_line(
            start_q, target, args.duration, steps
        )

        times, qs, dqs, ddqs, vmax, amax = result

        print("Waypoints:", len(times))
        print("Max planned speed:", round(vmax, 4), "rad/s")
        print(
            "Max planned acceleration:",
            round(amax, 4),
            "rad/s^2"
        )

        # Check FK/IK agreement for planned waypoints.
        max_plan_error = 0.0

        for i, q in enumerate(qs):
            u = i/steps
            s = quintic(u)

            desired = (
                start_xy[0] + s*(target[0]-start_xy[0]),
                start_xy[1] + s*(target[1]-start_xy[1])
            )

            actual = fk(q)

            error = math.hypot(
                actual[0]-desired[0],
                actual[1]-desired[1]
            )
            max_plan_error = max(max_plan_error, error)

        print(
            "Maximum planned waypoint error (mm):",
            round(max_plan_error*1000, 6)
        )

        node.send(times, qs, dqs, ddqs)

        # Topic interface has no execution result.
        # Observe measured joint states during motion.
        wall_start = time.monotonic()
        deadline = wall_start + args.duration*4 + 5

        final_error = float("inf")

        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)

            if node.current_q is None:
                continue

            xy = fk(node.current_q)

            final_error = math.hypot(
                xy[0]-target[0],
                xy[1]-target[1]
            )

            if (
                time.monotonic()-wall_start >= args.duration
                and final_error < 0.002
            ):
                break

        print(
            "Measured final XY (m):",
            tuple(round(v, 5) for v in fk(node.current_q))
        )

        print(
            "Measured endpoint error (mm):",
            round(final_error*1000, 3)
        )

        print(
            "Endpoint within 2 mm:",
            final_error < 0.002
        )

        print("Day 39 test finished")

    except (RuntimeError, ValueError) as exc:
        node.get_logger().error(str(exc))

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
