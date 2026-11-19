
import argparse

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose
from shape_msgs.msg import SolidPrimitive
from moveit_msgs.msg import CollisionObject
from moveit_msgs.msg import PlanningScene
from moveit_msgs.srv import ApplyPlanningScene


class ObstacleManager(Node):

    def __init__(self):
        super().__init__("day42_obstacle_manager")

        self.client = self.create_client(
            ApplyPlanningScene,
            "/apply_planning_scene"
        )

    def send_object(
        self,
        object_id,
        frame,
        position,
        dimensions,
        remove=False
    ):
        # Wait for MoveIt's planning scene service.
        if not self.client.wait_for_service(
            timeout_sec=10.0
        ):
            raise RuntimeError(
                "MoveIt service not available: "
                "/apply_planning_scene"
            )

        collision = CollisionObject()
        collision.id = object_id
        collision.header.frame_id = frame

        if remove:
            collision.operation = CollisionObject.REMOVE
        else:
            box = SolidPrimitive()
            box.type = SolidPrimitive.BOX
            box.dimensions = [
                float(v) for v in dimensions
            ]

            pose = Pose()
            pose.position.x = float(position[0])
            pose.position.y = float(position[1])
            pose.position.z = float(position[2])
            pose.orientation.w = 1.0

            collision.primitives.append(box)
            collision.primitive_poses.append(pose)
            collision.operation = CollisionObject.ADD

        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects.append(collision)

        request = ApplyPlanningScene.Request()
        request.scene = scene

        self.get_logger().info(
            f"Sending object: {object_id}, remove={remove}"
        )

        future = self.client.call_async(request)

        rclpy.spin_until_future_complete(
            self,
            future,
            timeout_sec=10.0
        )

        if not future.done():
            raise RuntimeError(
                "Timed out applying planning scene"
            )

        if future.exception() is not None:
            raise RuntimeError(
                str(future.exception())
            )

        response = future.result()

        if response is None or not response.success:
            raise RuntimeError(
                "MoveIt rejected planning scene update"
            )

        self.get_logger().info(
            "Planning scene update successful!"
        )


def main():
    parser = argparse.ArgumentParser(
        description="Day 42 Panda collision objects"
    )

    parser.add_argument(
        "--remove",
        action="store_true",
        help="Remove the obstacle"
    )

    parser.add_argument(
        "--id",
        default="day42_box"
    )

    parser.add_argument(
        "--frame",
        default="panda_link0"
    )

    parser.add_argument(
        "--x",
        type=float,
        default=0.40
    )

    parser.add_argument(
        "--y",
        type=float,
        default=0.25
    )

    parser.add_argument(
        "--z",
        type=float,
        default=0.35
    )

    parser.add_argument(
        "--sx",
        type=float,
        default=0.15
    )

    parser.add_argument(
        "--sy",
        type=float,
        default=0.15
    )

    parser.add_argument(
        "--sz",
        type=float,
        default=0.25
    )

    args = parser.parse_args()

    if not args.remove:
        if min(args.sx, args.sy, args.sz) <= 0:
            parser.error(
                "Obstacle dimensions must be positive"
            )

    rclpy.init()
    node = ObstacleManager()

    try:
        node.send_object(
            object_id=args.id,
            frame=args.frame,
            position=(args.x, args.y, args.z),
            dimensions=(args.sx, args.sy, args.sz),
            remove=args.remove
        )

    except Exception as exc:
        node.get_logger().error(str(exc))
        raise SystemExit(1)

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
