
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


class JointSubscriber(Node):

    def __init__(self):
        super().__init__("robot_joint_subscriber")

        self.subscription = self.create_subscription(
            Float64MultiArray,
            "/joint_angles",
            self.receive_angles,
            10
        )

        self.get_logger().info(
            "Joint subscriber started!"
        )

    def receive_angles(self, msg):

        if len(msg.data) != 2:
            self.get_logger().warning(
                "Expected two joint angles!"
            )
            return

        shoulder = msg.data[0]
        elbow = msg.data[1]

        self.get_logger().info(
            f"Received shoulder={shoulder:.2f}, "
            f"elbow={elbow:.2f}"
        )

        if not (0 <= shoulder <= 90):
            self.get_logger().warning(
                "Shoulder angle out of range!"
            )

        if not (-90 <= elbow <= 90):
            self.get_logger().warning(
                "Elbow angle out of range!"
            )


def main(args=None):
    rclpy.init(args=args)
    node = JointSubscriber()

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
