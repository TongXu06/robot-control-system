
import sys

import rclpy
from rclpy.node import Node

from robot_interfaces.srv import SetJointAngles


class JointClient(Node):

    def __init__(self):
        super().__init__("robot_joint_client")

        self.client = self.create_client(
            SetJointAngles,
            "/set_joint_angles"
        )

    def send_request(self, shoulder, elbow):

        while not self.client.wait_for_service(
            timeout_sec=1.0
        ):
            self.get_logger().info(
                "Waiting for joint service..."
            )

        request = SetJointAngles.Request()
        request.shoulder = shoulder
        request.elbow = elbow

        return self.client.call_async(request)


def main(args=None):

    if len(sys.argv) != 3:
        print(
            "Usage: joint_client "
            "<shoulder_deg> <elbow_deg>"
        )
        return

    try:
        shoulder = float(sys.argv[1])
        elbow = float(sys.argv[2])
    except ValueError:
        print("Please enter numeric angles.")
        return

    rclpy.init(args=args)
    node = JointClient()

    try:
        future = node.send_request(
            shoulder,
            elbow
        )

        rclpy.spin_until_future_complete(
            node,
            future
        )

        result = future.result()

        if result is None:
            node.get_logger().error(
                "Service request failed"
            )
        else:
            print("Accepted:", result.accepted)
            print("Message:", result.message)

    except KeyboardInterrupt:
        print("Client interrupted.")

    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
