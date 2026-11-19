
import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


class JointPublisher(Node):

    def __init__(self):
        super().__init__("robot_joint_publisher")

        self.publisher = self.create_publisher(
            Float64MultiArray,
            "/joint_angles",
            10
        )

        self.time_elapsed = 0.0

        self.timer = self.create_timer(
            1.0,
            self.publish_angles
        )

    def publish_angles(self):

        shoulder = 30.0 + 10.0 * math.sin(
            self.time_elapsed
        )

        elbow = 45.0 + 15.0 * math.sin(
            self.time_elapsed
        )

        msg = Float64MultiArray()
        msg.data = [shoulder, elbow]

        self.publisher.publish(msg)

        self.get_logger().info(
            f"Publishing shoulder={shoulder:.2f}, "
            f"elbow={elbow:.2f}"
        )

        self.time_elapsed += 0.2


def main(args=None):
    rclpy.init(args=args)
    node = JointPublisher()

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
