from pathlib import Path
import xml.etree.ElementTree as ET

from ament_index_python.packages import (
    get_package_share_directory,
)


ROOT = Path(__file__).resolve().parent

PANDA_SHARE = Path(
    get_package_share_directory(
        "moveit_resources_panda_description"
    )
)

# Locate the installed Panda URDF.
source = PANDA_SHARE / "urdf" / "panda.urdf"

if not source.exists():
    raise FileNotFoundError(
        f"Panda URDF not found: {source}"
    )

tree = ET.parse(source)
robot = tree.getroot()

if robot.tag != "robot":
    raise ValueError("Invalid robot model")

joints = [
    f"panda_joint{i}" for i in range(1, 8)
]

existing_joints = {
    item.get("name")
    for item in robot.findall("joint")
}

missing = set(joints) - existing_joints

if missing:
    raise ValueError(
        f"Missing robot joints: {sorted(missing)}"
    )

links = {
    item.get("name")
    for item in robot.findall("link")
}

if "panda_link0" not in links:
    raise ValueError("panda_link0 not found")

# Convert ROS package mesh URIs into Gazebo-readable
# absolute file URIs, while retaining original meshes.
for element in robot.iter():
    filename = element.get("filename")

    if not filename:
        continue

    prefix = (
        "package://moveit_resources_panda_description/"
    )

    if filename.startswith(prefix):
        relative = filename[len(prefix):]
        absolute = (PANDA_SHARE / relative).resolve()

        if not absolute.exists():
            raise FileNotFoundError(absolute)

        element.set("filename", absolute.as_uri())

# Attach the robot base to the Gazebo world.
ET.SubElement(robot, "link", {"name": "world"})

fixed = ET.SubElement(
    robot,
    "joint",
    {"name": "world_to_panda", "type": "fixed"}
)

ET.SubElement(fixed, "parent", {"link": "world"})
ET.SubElement(fixed, "child", {"link": "panda_link0"})
ET.SubElement(
    fixed, "origin",
    {"xyz": "0 0 0", "rpy": "0 0 0"}
)

control = ET.SubElement(
    robot,
    "ros2_control",
    {
        "name": "GazeboSimSystem",
        "type": "system"
    }
)

hardware = ET.SubElement(control, "hardware")
ET.SubElement(
    hardware,
    "plugin"
).text = "gz_ros2_control/GazeboSimSystem"

for name in joints:
    item = ET.SubElement(
        control,
        "joint",
        {"name": name}
    )

    ET.SubElement(
        item, "command_interface",
        {"name": "position"}
    )

    state = ET.SubElement(
        item,
        "state_interface",
        {"name": "position"}
    )

    # Start near a normal non-singular pose.
    initial = {
        "panda_joint1": 0.0,
        "panda_joint2": -0.785,
        "panda_joint3": 0.0,
        "panda_joint4": -2.356,
        "panda_joint5": 0.0,
        "panda_joint6": 1.571,
        "panda_joint7": 0.785,
    }

    ET.SubElement(
        state,
        "param",
        {"name": "initial_value"}
    ).text = str(initial[name])

    ET.SubElement(
        item,
        "state_interface",
        {"name": "velocity"}
    )

gazebo = ET.SubElement(robot, "gazebo")

plugin = ET.SubElement(
    gazebo,
    "plugin",
    {
        "filename": "libgz_ros2_control-system.so",
        "name": (
            "gz_ros2_control::"
            "GazeboSimROS2ControlPlugin"
        )
    }
)

config = (
    ROOT / "config" / "controllers.yaml"
).resolve()

ET.SubElement(
    plugin,
    "parameters"
).text = str(config)

output = ROOT / "urdf" / "panda_gazebo.urdf"
output.parent.mkdir(parents=True, exist_ok=True)

ET.indent(tree, space="  ")
tree.write(
    output,
    encoding="utf-8",
    xml_declaration=True
)

print("Generated:", output)
print("Robot:", robot.get("name"))
print("Controlled joints:", len(joints))
print("Controller config:", config)
