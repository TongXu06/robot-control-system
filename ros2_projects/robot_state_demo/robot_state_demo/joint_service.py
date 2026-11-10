
import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

from robot_interfaces.srv import SetJointAngles


class JointService(Node):

    def __init__(self):
        super().__init__("robot_joint_service")

        # Create a service server
        self.service = self.create_service(
            SetJointAngles,
            "/set_joint_angles",
            self.handle_request
        )

        # Reuse Day 30 topic
        self.publisher = self.create_publisher(
            Float64MultiArray,
            "/joint_angles",
            10
        )

        self.get_logger().info(
            "Joint angle service is ready!"
        )

    def handle_request(self, request, response):

        shoulder = request.shoulder
        elbow = request.elbow

        self.get_logger().info(
            f"Request: shoulder={shoulder:.2f}, "
            f"elbow={elbow:.2f}"
        )

        if not (
            math.isfinite(shoulder)
            and math.isfinite(elbow)
        ):
            response.accepted = False
            response.message = "Angles must be finite"
            return response

        if not (0.0 <= shoulder <= 90.0):
            response.accepted = False
            response.message = "Shoulder out of range"
            return response

        if not (-90.0 <= elbow <= 90.0):
            response.accepted = False
            response.message = "Elbow out of range"
            return response

        # Accept the request
        response.accepted = True
        response.message = "Target accepted"

        # Publish the accepted command
        msg = Float64MultiArray()
        msg.data = [shoulder, elbow]

        self.publisher.publish(msg)

        self.get_logger().info(
            "Accepted target published!"
        )

        return response


def main(args=None):
    rclpy.init(args=args)
    node = JointService()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
