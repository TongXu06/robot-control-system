
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class RobotJointStatePublisher(Node):

    def __init__(self):
        super().__init__("robot_state_publisher_demo")

        # Declare ROS 2 parameters
        self.declare_parameter("shoulder_deg", 30.0)
        self.declare_parameter("elbow_deg", 45.0)
        self.declare_parameter("publish_rate", 2.0)

        rate = self.get_parameter(
            "publish_rate"
        ).value

        if not math.isfinite(rate) or rate <= 0:
            raise ValueError("publish_rate must be positive")

        self.publisher = self.create_publisher(
            JointState,
            "/joint_states",
            10
        )

        # Timer interval in seconds
        self.timer = self.create_timer(
            1.0 / rate,
            self.publish_state
        )

        self.get_logger().info(
            f"JointState Publisher started at {rate} Hz"
        )

    def publish_state(self):

        shoulder_deg = self.get_parameter(
            "shoulder_deg"
        ).value

        elbow_deg = self.get_parameter(
            "elbow_deg"
        ).value

        if not all(math.isfinite(x) for x in
                   [shoulder_deg, elbow_deg]):
            self.get_logger().warning(
                "Invalid joint parameter"
            )
            return

        if not (0 <= shoulder_deg <= 90):
            self.get_logger().warning(
                "Shoulder out of range"
            )
            return

        if not (-90 <= elbow_deg <= 90):
            self.get_logger().warning(
                "Elbow out of range"
            )
            return

        msg = JointState()

        msg.header.stamp = (
            self.get_clock().now().to_msg()
        )

        msg.name = [
            "shoulder_joint",
            "elbow_joint"
        ]

        msg.position = [
            math.radians(shoulder_deg),
            math.radians(elbow_deg)
        ]

        # No velocity or effort measurements
        msg.velocity = []
        msg.effort = []

        self.publisher.publish(msg)

        self.get_logger().info(
            f"Published shoulder={shoulder_deg:.1f} deg, "
            f"elbow={elbow_deg:.1f} deg"
        )


def main(args=None):
    rclpy.init(args=args)
    node = RobotJointStatePublisher()

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
