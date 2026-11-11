
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class RobotJointStateMonitor(Node):

    def __init__(self):
        super().__init__("joint_state_monitor")

        self.subscription = self.create_subscription(
            JointState,
            "/joint_states",
            self.receive_state,
            10
        )

        self.get_logger().info(
            "JointState Monitor started!"
        )

    def receive_state(self, msg):

        if len(msg.name) != len(msg.position):
            self.get_logger().warning(
                "Joint names and positions do not match"
            )
            return

        for name, angle_rad in zip(
            msg.name,
            msg.position
        ):

            angle_deg = math.degrees(angle_rad)

            self.get_logger().info(
                f"{name}: {angle_rad:.4f} rad "
                f"({angle_deg:.2f} deg)"
            )


def main(args=None):
    rclpy.init(args=args)
    node = RobotJointStateMonitor()

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
