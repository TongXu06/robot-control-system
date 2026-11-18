from pathlib import Path
import xml.etree.ElementTree as ET
import yaml

home = Path.home()
src = home / "ros2_ws/src/day43_panda_gazebo"
dst = home / "ros2_ws/day44_panda_gripper"

source_urdf = src / "urdf/panda_gazebo.urdf"
source_yaml = src / "config/controllers.yaml"

out_urdf = dst / "panda_gripper.urdf"
out_yaml = dst / "controllers.yaml"

tree = ET.parse(source_urdf)
robot = tree.getroot()

# Verify the fingers exist.
joints = {
    j.get("name"): j
    for j in robot.findall("joint")
}

for name in ("panda_finger_joint1",
             "panda_finger_joint2"):
    if name not in joints:
        raise RuntimeError(f"Missing joint: {name}")

# Ensure finger 2 follows finger 1.
follower = joints["panda_finger_joint2"]
mimic = follower.find("mimic")

if mimic is None:
    mimic = ET.SubElement(follower, "mimic")

mimic.set("joint", "panda_finger_joint1")
mimic.set("multiplier", "1.0")
mimic.set("offset", "0.0")

control = robot.find("ros2_control")

if control is None:
    raise RuntimeError("ros2_control section not found")

controlled = {
    j.get("name"): j
    for j in control.findall("joint")
}

# First finger is the commanded joint.
if "panda_finger_joint1" not in controlled:
    finger1 = ET.SubElement(
        control, "joint",
        {"name": "panda_finger_joint1"}
    )

    ET.SubElement(
        finger1,
        "command_interface",
        {"name": "position"}
    )

    state = ET.SubElement(
        finger1,
        "state_interface",
        {"name": "position"}
    )
    ET.SubElement(
        state, "param",
        {"name": "initial_value"}
    ).text = "0.04"

    ET.SubElement(
        finger1,
        "state_interface",
        {"name": "velocity"}
    )

# Second finger follows through mimic.
if "panda_finger_joint2" not in controlled:
    finger2 = ET.SubElement(
        control, "joint",
        {"name": "panda_finger_joint2"}
    )

    state = ET.SubElement(
        finger2,
        "state_interface",
        {"name": "position"}
    )
    ET.SubElement(
        state, "param",
        {"name": "initial_value"}
    ).text = "0.04"

    ET.SubElement(
        finger2,
        "state_interface",
        {"name": "velocity"}
    )
else:
    finger2 = controlled["panda_finger_joint2"]
    if finger2.findall("command_interface"):
        raise RuntimeError(
            "Mimic finger must not have command interface"
        )

# Update the Gazebo plugin YAML path.
plugin = robot.find(
    ".//gazebo/plugin"
    "[@name='gz_ros2_control::GazeboSimROS2ControlPlugin']"
)

if plugin is None:
    raise RuntimeError("Gazebo control plugin not found")

parameters = plugin.find("parameters")
if parameters is None:
    parameters = ET.SubElement(plugin, "parameters")

parameters.text = str(out_yaml)

# Preserve Day 43 controller settings.
with source_yaml.open() as f:
    data = yaml.safe_load(f)

if not isinstance(data, dict):
    raise RuntimeError("Invalid controller YAML")

manager = data["controller_manager"]["ros__parameters"]

manager["panda_gripper_controller"] = {
    "type": "forward_command_controller/ForwardCommandController"
}

data["panda_gripper_controller"] = {
    "ros__parameters": {
        "joints": ["panda_finger_joint1"],
        "interface_name": "position"
    }
}

with out_yaml.open("w") as f:
    yaml.safe_dump(data, f, sort_keys=False)

ET.indent(tree, space="  ")
tree.write(
    out_urdf,
    encoding="utf-8",
    xml_declaration=True
)

# Prepare matching robot_state_publisher parameters.
description = out_urdf.read_text()

params = {
    "robot_state_publisher": {
        "ros__parameters": {
            "robot_description": description,
            "use_sim_time": True
        }
    }
}

with (dst / "robot_description.yaml").open("w") as f:
    yaml.safe_dump(params, f)

print("Day 44 model generated!")
print("URDF:", out_urdf)
print("YAML:", out_yaml)
print("Controlled finger: panda_finger_joint1")
print("Mimic finger: panda_finger_joint2")
