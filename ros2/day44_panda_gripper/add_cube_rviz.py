import rclpy
from rclpy.node import Node
from moveit_msgs.srv import ApplyPlanningScene
from moveit_msgs.msg import PlanningScene, CollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose

rclpy.init()
node = Node("add_cube_rviz")

client = node.create_client(
    ApplyPlanningScene, "/apply_planning_scene"
)

if not client.wait_for_service(timeout_sec=10.0):
    raise RuntimeError("MoveIt planning scene service unavailable")

cube = CollisionObject()
cube.header.frame_id = "panda_link0"
cube.id = "day44_cube_collision"
cube.operation = CollisionObject.ADD

shape = SolidPrimitive()
shape.type = SolidPrimitive.BOX
shape.dimensions = [0.05, 0.05, 0.05]

pose = Pose()
pose.position.x = 0.45
pose.position.y = 0.0
pose.position.z = 0.025
pose.orientation.w = 1.0

cube.primitives.append(shape)
cube.primitive_poses.append(pose)

scene = PlanningScene()
scene.is_diff = True
scene.world.collision_objects.append(cube)

request = ApplyPlanningScene.Request()
request.scene = scene

future = client.call_async(request)
rclpy.spin_until_future_complete(node, future)

if future.result() and future.result().success:
    print("SUCCESS: Cube added to MoveIt planning scene")
else:
    print("FAILED: Cube not added")

node.destroy_node()
rclpy.shutdown()
