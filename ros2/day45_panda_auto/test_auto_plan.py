import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from moveit_msgs.srv import GetMotionPlan
from moveit_msgs.msg import Constraints, JointConstraint
from moveit_msgs.msg import MoveItErrorCodes

JOINTS = [f"panda_joint{i}" for i in range(1, 8)]


class AutoPlanner(Node):
    def __init__(self):
        super().__init__("day45_auto_planner")
        self.positions = None

        self.create_subscription(
            JointState,
            "/joint_states",
            self.on_joint_state,
            10
        )

        self.client = self.create_client(
            GetMotionPlan,
            "/plan_kinematic_path"
        )

    def on_joint_state(self, msg):
        joint_map = dict(zip(msg.name, msg.position))
        if all(j in joint_map for j in JOINTS):
            self.positions = {
                j: float(joint_map[j]) for j in JOINTS
            }


def main():
    rclpy.init()
    node = AutoPlanner()

    try:
        print("Waiting for robot joint states...")

        for _ in range(100):
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.positions is not None:
                break

        if node.positions is None:
            raise RuntimeError("No joint states received")

        print("Current joints:", node.positions)

        target = node.positions.copy()
        target["panda_joint1"] += 0.05

        print("Target joints:", target)

        if not node.client.wait_for_service(
            timeout_sec=10.0
        ):
            raise RuntimeError("MoveIt service unavailable")

        request = GetMotionPlan.Request()
        motion = request.motion_plan_request

        motion.group_name = "panda_arm"
        motion.pipeline_id = "ompl"
        motion.num_planning_attempts = 10
        motion.allowed_planning_time = 5.0
        motion.max_velocity_scaling_factor = 0.1
        motion.max_acceleration_scaling_factor = 0.1
        motion.start_state.is_diff = True

        goal = Constraints()

        for joint, angle in target.items():
            constraint = JointConstraint()
            constraint.joint_name = joint
            constraint.position = angle
            constraint.tolerance_above = 0.01
            constraint.tolerance_below = 0.01
            constraint.weight = 1.0
            goal.joint_constraints.append(constraint)

        motion.goal_constraints.append(goal)

        print("Requesting MoveIt collision-aware plan...")

        future = node.client.call_async(request)
        rclpy.spin_until_future_complete(
            node, future, timeout_sec=20.0
        )

        if not future.done():
            raise RuntimeError("Planning request timed out")

        response = future.result()
        if response is None:
            raise RuntimeError("No planning response received")

        result = response.motion_plan_response
        if result.error_code.val == MoveItErrorCodes.SUCCESS:
            print("SUCCESS: Automatic planning completed!")
            print(
                "Trajectory points:",
                len(result.trajectory.joint_trajectory.points)
            )
        else:
            print(
                "PLANNING FAILED:",
                result.error_code.val
            )
            print("Check collision scene and start state.")

        print("No robot motion was executed.")

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
