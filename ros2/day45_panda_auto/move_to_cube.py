import math
import rclpy
from rclpy.node import Node

from moveit_msgs.srv import GetMotionPlan
from moveit_msgs.msg import (
    Constraints,
    PositionConstraint,
    OrientationConstraint,
    MoveItErrorCodes,
)
from geometry_msgs.msg import Pose
from shape_msgs.msg import SolidPrimitive


# Target for panda_hand, in panda_link0 coordinates
TARGET_X = 0.450
TARGET_Y = 0.000
TARGET_Z = 0.300

# Orientation measured from the working Panda TF
Q = [1.000, 0.028, -0.003, 0.004]
norm = math.sqrt(sum(v * v for v in Q))
Q = [v / norm for v in Q]


class CubePlanner(Node):
    def __init__(self):
        super().__init__("day45_cube_pose_planner")
        self.client = self.create_client(
            GetMotionPlan,
            "/plan_kinematic_path"
        )


def main():
    rclpy.init()
    node = CubePlanner()

    try:
        print("DAY 45 - PANDA AUTO POSITIONING")
        print(
            f"Target panda_hand: "
            f"({TARGET_X}, {TARGET_Y}, {TARGET_Z})"
        )

        if not node.client.wait_for_service(
            timeout_sec=10.0
        ):
            raise RuntimeError("MoveIt planning service unavailable")

        req = GetMotionPlan.Request()
        motion = req.motion_plan_request

        motion.group_name = "panda_arm"
        motion.pipeline_id = "ompl"
        motion.allowed_planning_time = 10.0
        motion.num_planning_attempts = 15
        motion.max_velocity_scaling_factor = 0.1
        motion.max_acceleration_scaling_factor = 0.1
        motion.start_state.is_diff = True

        goal = Constraints()
        goal.name = "pregrasp_above_cube"

        # Position target: sphere around desired hand position
        pc = PositionConstraint()
        pc.header.frame_id = "panda_link0"
        pc.link_name = "panda_hand"
        pc.weight = 1.0

        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [0.015]

        pose = Pose()
        pose.position.x = TARGET_X
        pose.position.y = TARGET_Y
        pose.position.z = TARGET_Z
        pose.orientation.w = 1.0

        pc.constraint_region.primitives.append(sphere)
        pc.constraint_region.primitive_poses.append(pose)

        goal.position_constraints.append(pc)

        # Maintain a downward-facing gripper
        oc = OrientationConstraint()
        oc.header.frame_id = "panda_link0"
        oc.link_name = "panda_hand"

        oc.orientation.x = Q[0]
        oc.orientation.y = Q[1]
        oc.orientation.z = Q[2]
        oc.orientation.w = Q[3]

        oc.absolute_x_axis_tolerance = 0.15
        oc.absolute_y_axis_tolerance = 0.15
        oc.absolute_z_axis_tolerance = 0.15
        oc.weight = 1.0

        goal.orientation_constraints.append(oc)
        motion.goal_constraints.append(goal)

        print("Requesting collision-aware MoveIt plan...")

        future = node.client.call_async(req)
        rclpy.spin_until_future_complete(
            node, future, timeout_sec=25.0
        )

        if not future.done():
            raise RuntimeError("Planning timed out")

        response = future.result()
        if response is None:
            raise RuntimeError("No MoveIt response")

        result = response.motion_plan_response
        count = len(
            result.trajectory.joint_trajectory.points
        )

        if result.error_code.val == MoveItErrorCodes.SUCCESS:
            print("SUCCESS: Pregrasp planning completed!")
            print("Trajectory points:", count)
        else:
            print("PLANNING FAILED")
            print("MoveIt error code:", result.error_code.val)

        print("PLAN ONLY - NO ROBOT MOTION")

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
