
import rclpy
from rclpy.node import Node

from moveit_msgs.srv import ApplyPlanningScene
from moveit_msgs.msg import PlanningScene, CollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose


def make_box(name, size, position):
    obj = CollisionObject()
    obj.header.frame_id = "panda_link0"
    obj.id = name

    box = SolidPrimitive()
    box.type = SolidPrimitive.BOX
    box.dimensions = list(size)

    pose = Pose()
    pose.position.x = position[0]
    pose.position.y = position[1]
    pose.position.z = position[2]
    pose.orientation.w = 1.0

    obj.primitives.append(box)
    obj.primitive_poses.append(pose)
    obj.operation = CollisionObject.ADD

    return obj


def main():
    rclpy.init()
    node = Node("day44_floor_manager")

    client = node.create_client(
        ApplyPlanningScene,
        "/apply_planning_scene"
    )

    if not client.wait_for_service(timeout_sec=10.0):
        node.get_logger().error("MoveIt service unavailable")
        node.destroy_node()
        rclpy.shutdown()
        return

    scene = PlanningScene()
    scene.is_diff = True
    scene.robot_state.is_diff = True

    # Four floor sections with clearance around the base.
    sections = [
        ("floor_front", (0.85, 2.0, 0.02), (0.575, 0.0, -0.01)),
        ("floor_back", (0.85, 2.0, 0.02), (-0.575, 0.0, -0.01)),
        ("floor_left", (0.30, 0.85, 0.02), (0.0, 0.575, -0.01)),
        ("floor_right", (0.30, 0.85, 0.02), (0.0, -0.575, -0.01)),
    ]

    for name, size, position in sections:
        scene.world.collision_objects.append(
            make_box(name, size, position)
        )

    request = ApplyPlanningScene.Request()
    request.scene = scene

    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future)

    response = future.result()

    if response is not None and response.success:
        print("SUCCESS: Four floor sections added.")
    else:
        print("FAILED: Could not add floor.")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
