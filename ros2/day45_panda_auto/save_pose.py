#!/usr/bin/env python3

import json
import sys
from pathlib import Path

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

JOINTS = [f"panda_joint{i}" for i in range(1, 8)]
OUTPUT = Path(__file__).parent / "poses.json"


class PoseRecorder(Node):
    def __init__(self):
        super().__init__("day45_pose_recorder")
        self.positions = None
        self.create_subscription(
            JointState,
            "/joint_states",
            self.callback,
            10
        )

    def callback(self, msg):
        values = dict(zip(msg.name, msg.position))
        if all(j in values for j in JOINTS):
            self.positions = {
                j: round(float(values[j]), 6)
                for j in JOINTS
            }


def main():
    if len(sys.argv) != 2:
        print("Usage: python3 save_pose.py POSE_NAME")
        sys.exit(1)

    name = sys.argv[1]

    rclpy.init()
    node = PoseRecorder()

    print("Waiting for Panda joint states...")

    try:
        for _ in range(100):
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.positions is not None:
                break

        if node.positions is None:
            raise RuntimeError(
                "No joint states received. Start Gazebo first."
            )

        data = {}
        if OUTPUT.exists():
            data = json.loads(OUTPUT.read_text())

        data[name] = node.positions

        OUTPUT.write_text(
            json.dumps(data, indent=2)
        )

        print(f"\nSaved pose: {name}")
        for joint, angle in node.positions.items():
            print(f"{joint}: {angle:.6f} rad")

        print(f"\nFile: {OUTPUT}")

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
