
import math
import time
import threading

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from std_msgs.msg import Float64MultiArray
from robot_interfaces.action import MoveRobot


class MoveActionServer(Node):

    def __init__(self):
        super().__init__("move_action_server")

        self.angles = [30.0, 45.0]
        self.busy = False
        self.lock = threading.Lock()

        self.publisher = self.create_publisher(
            Float64MultiArray, "/joint_angles", 10
        )

        self.action_server = ActionServer(
            self,
            MoveRobot,
            "/move_robot",
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=ReentrantCallbackGroup()
        )

        self.get_logger().info("Move Action Server ready!")

    def goal_callback(self, goal_request):
        shoulder = goal_request.shoulder
        elbow = goal_request.elbow
        duration = goal_request.duration

        valid = (
            math.isfinite(shoulder)
            and math.isfinite(elbow)
            and math.isfinite(duration)
            and 0.0 <= shoulder <= 90.0
            and -90.0 <= elbow <= 90.0
            and 1.0 <= duration <= 30.0
        )

        if not valid:
            self.get_logger().warning("Invalid goal rejected")
            return GoalResponse.REJECT

        with self.lock:
            if self.busy:
                self.get_logger().warning("Robot is busy")
                return GoalResponse.REJECT

            self.busy = True

        self.get_logger().info("Goal accepted")
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info("Cancel request received")
        return CancelResponse.ACCEPT

    def publish_angles(self):
        msg = Float64MultiArray()
        msg.data = list(self.angles)
        self.publisher.publish(msg)

    def make_result(self, success, message):
        result = MoveRobot.Result()
        result.success = success
        result.final_shoulder = self.angles[0]
        result.final_elbow = self.angles[1]
        result.message = message
        return result

    def execute_callback(self, goal_handle):

        try:
            target = [
                goal_handle.request.shoulder,
                goal_handle.request.elbow
            ]
            duration = goal_handle.request.duration

            start = list(self.angles)
            steps = max(1, round(duration / 0.1))
            dt = duration / steps

            self.get_logger().info(
                f"Moving from {start} to {target}"
            )

            for i in range(steps + 1):

                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()

                    self.get_logger().info("Motion canceled")
                    return self.make_result(
                        False, "Motion canceled"
                    )

                u = i / steps

                # Quintic smoothstep
                s = 10*u**3 - 15*u**4 + 6*u**5

                self.angles = [
                    start[j] + (target[j] - start[j]) * s
                    for j in range(2)
                ]

                self.publish_angles()

                feedback = MoveRobot.Feedback()
                feedback.current_shoulder = self.angles[0]
                feedback.current_elbow = self.angles[1]
                feedback.progress = u * 100.0

                goal_handle.publish_feedback(feedback)

                self.get_logger().info(
                    f"Progress: {feedback.progress:.0f}% "
                    f"Angles: {self.angles[0]:.2f}, "
                    f"{self.angles[1]:.2f}"
                )

                if i < steps:
                    time.sleep(dt)

            goal_handle.succeed()

            self.get_logger().info("Motion completed!")
            return self.make_result(
                True, "Target reached in simulation"
            )

        except Exception as exc:
            self.get_logger().error(f"Motion error: {exc}")
            goal_handle.abort()
            return self.make_result(
                False, f"Motion failed: {exc}"
            )

        finally:
            with self.lock:
                self.busy = False


def main(args=None):
    rclpy.init(args=args)
    node = MoveActionServer()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
