
import argparse
import sys

import rclpy
from rclpy.utilities import remove_ros_args
from rclpy.node import Node
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus

from robot_interfaces.action import MoveRobot


class MoveActionClient(Node):

    def __init__(self, cancel_after=0.0):
        super().__init__("move_action_client")

        self.action_client = ActionClient(
            self, MoveRobot, "/move_robot"
        )

        self.cancel_after = cancel_after
        self.goal_handle = None
        self.cancel_timer = None
        self.done = False

    def send_goal(self, shoulder, elbow, duration):

        goal = MoveRobot.Goal()
        goal.shoulder = shoulder
        goal.elbow = elbow
        goal.duration = duration

        self.get_logger().info(
            f"Requesting target: {shoulder}, {elbow}"
        )

        self.action_client.wait_for_server()

        future = self.action_client.send_goal_async(
            goal,
            feedback_callback=self.feedback_callback
        )
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):

        try:
            self.goal_handle = future.result()
        except Exception as exc:
            self.get_logger().error(f"Goal error: {exc}")
            self.done = True
            return

        if not self.goal_handle.accepted:
            self.get_logger().warning("Goal rejected!")
            self.done = True
            return

        self.get_logger().info("Goal accepted!")

        result_future = self.goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

        if self.cancel_after > 0:
            self.cancel_timer = self.create_timer(
                self.cancel_after,
                self.cancel_goal
            )

    def feedback_callback(self, feedback_msg):

        fb = feedback_msg.feedback
        self.get_logger().info(
            f"Progress={fb.progress:.0f}% | "
            f"Shoulder={fb.current_shoulder:.2f} | "
            f"Elbow={fb.current_elbow:.2f}"
        )

    def cancel_goal(self):

        if self.cancel_timer is not None:
            self.cancel_timer.cancel()

        if self.done or self.goal_handle is None:
            return

        self.get_logger().info("Sending cancel request...")

        future = self.goal_handle.cancel_goal_async()
        future.add_done_callback(self.cancel_response_callback)

    def cancel_response_callback(self, future):

        try:
            response = future.result()
            self.get_logger().info(
                f"Cancel goals accepted: "
                f"{len(response.goals_canceling)}"
            )
        except Exception as exc:
            self.get_logger().error(f"Cancel error: {exc}")

    def result_callback(self, future):

        try:
            wrapped = future.result()
            result = wrapped.result

            status_name = {
                GoalStatus.STATUS_SUCCEEDED: "SUCCEEDED",
                GoalStatus.STATUS_CANCELED: "CANCELED",
                GoalStatus.STATUS_ABORTED: "ABORTED",
            }.get(wrapped.status, str(wrapped.status))

            print("\n========== RESULT ==========")
            print("Status:", status_name)
            print("Success:", result.success)
            print("Final shoulder:", round(
                result.final_shoulder, 2
            ))
            print("Final elbow:", round(
                result.final_elbow, 2
            ))
            print("Message:", result.message)

        except Exception as exc:
            self.get_logger().error(f"Result error: {exc}")

        finally:
            self.done = True


def main(args=None):

    parser = argparse.ArgumentParser()
    parser.add_argument("shoulder", type=float)
    parser.add_argument("elbow", type=float)
    parser.add_argument("duration", type=float)
    parser.add_argument(
        "--cancel-after", type=float, default=0.0
    )

    cli_args = remove_ros_args(args=sys.argv)[1:]

    parsed = parser.parse_args(cli_args)

    rclpy.init(args=args)
    node = MoveActionClient(parsed.cancel_after)

    try:
        node.send_goal(
            parsed.shoulder,
            parsed.elbow,
            parsed.duration
        )

        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)

    except KeyboardInterrupt:
        print("Client interrupted.")

    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
