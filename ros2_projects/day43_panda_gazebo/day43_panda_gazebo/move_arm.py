import argparse
import math

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from action_msgs.msg import GoalStatus


JOINTS = [f"panda_joint{i}" for i in range(1, 8)]

READY = [
    0.0, -0.785, 0.0, -2.356,
    0.0, 1.571, 0.785
]

TARGET = [
    0.25, -0.55, 0.20, -2.0,
    0.15, 1.75, 0.65
]


class PandaMover(Node):

    def __init__(self):
        super().__init__("day43_panda_mover")

        self.client = ActionClient(
            self,
            FollowJointTrajectory,
            "/panda_arm_controller/follow_joint_trajectory"
        )

    def move(self, positions, duration):

        if not self.client.wait_for_server(
            timeout_sec=10.0
        ):
            raise RuntimeError(
                "Trajectory Action server unavailable"
            )

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = JOINTS

        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start = Duration(
            seconds=duration
        ).to_msg()

        goal.trajectory.points.append(point)

        self.get_logger().info(
            f"Sending seven-joint target: {positions}"
        )

        future = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)

        handle = future.result()

        if handle is None or not handle.accepted:
            raise RuntimeError("Trajectory goal rejected")

        self.get_logger().info("Goal accepted")

        result_future = handle.get_result_async()

        rclpy.spin_until_future_complete(
            self,
            result_future
        )

        response = result_future.result()

        if response.status != GoalStatus.STATUS_SUCCEEDED:
            raise RuntimeError(
                f"Goal status: {response.status}"
            )

        if response.result.error_code != 0:
            raise RuntimeError(
                f"Controller error: "
                f"{response.result.error_code}"
            )

        self.get_logger().info(
            "Seven-joint motion completed!"
        )


def main():

    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--pose",
        choices=["ready", "target"],
        default="target"
    )

    parser.add_argument(
        "--duration",
        type=float,
        default=8.0
    )

    args = parser.parse_args()

    if args.duration <= 0:
        parser.error("Duration must be positive")

    pose = READY if args.pose == "ready" else TARGET

    rclpy.init()
    node = PandaMover()

    try:
        node.move(pose, args.duration)

    except Exception as exc:
        node.get_logger().error(str(exc))
        raise SystemExit(1)

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
