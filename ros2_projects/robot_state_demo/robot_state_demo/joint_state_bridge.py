
import math

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState


class JointStateBridge(Node):

    def __init__(self):
        super().__init__("joint_state_bridge")

        # Initial simulated joint angles (degrees)
        self.angles = [30.0, 45.0]

        # Receive Day 32 Action Server output
        self.subscription = self.create_subscription(
            Float64MultiArray,
            "/joint_angles",
            self.receive_angles,
            10
        )

        # Publish standard ROS 2 JointState
        self.publisher = self.create_publisher(
            JointState,
            "/joint_states",
            10
        )

        # Keep the latest state available at 10 Hz
        self.timer = self.create_timer(
            0.1,
            self.publish_joint_state
        )

        self.get_logger().info(
            "JointState Bridge started!"
        )

    def receive_angles(self, msg):

        if len(msg.data) != 2:
            self.get_logger().warning(
                "Expected exactly two joint angles"
            )
            return

        shoulder = float(msg.data[0])
        elbow = float(msg.data[1])

        if not all(math.isfinite(x) for x in
                   [shoulder, elbow]):
            self.get_logger().warning(
                "Received non-finite angles"
            )
            return

        if not (0.0 <= shoulder <= 90.0):
            self.get_logger().warning(
                "Shoulder angle out of range"
            )
            return

        if not (-90.0 <= elbow <= 90.0):
            self.get_logger().warning(
                "Elbow angle out of range"
            )
            return

        self.angles = [shoulder, elbow]

    def publish_joint_state(self):

        msg = JointState()

        msg.header.stamp = (
            self.get_clock().now().to_msg()
        )

        msg.name = [
            "shoulder_joint",
            "elbow_joint"
        ]

        msg.position = [
            math.radians(self.angles[0]),
            math.radians(self.angles[1])
        ]

        # Unknown in this simple simulation
        msg.velocity = []
        msg.effort = []

        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    node = JointStateBridge()

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

