import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation


# Robot parameters
L1 = 100
L2 = 80

start_angles = np.array([30.0, 20.0])
target_angles = np.array([70.0, 60.0])

T = 5.0
steps = 100
time = np.linspace(0, T, steps + 1)

# Quintic trajectory
joint_trajectory = []

for t in time:
    u = t / T

    s = 10*u**3 - 15*u**4 + 6*u**5

    angles = start_angles + (
        target_angles - start_angles
    ) * s

    joint_trajectory.append(angles)

joint_trajectory = np.array(joint_trajectory)


def forward_kinematics(angles):

    theta1 = np.radians(angles[0])
    theta2 = np.radians(angles[1])

    x1 = L1 * np.cos(theta1)
    y1 = L1 * np.sin(theta1)

    x2 = x1 + L2 * np.cos(theta1 + theta2)
    y2 = y1 + L2 * np.sin(theta1 + theta2)

    return x1, y1, x2, y2


fig, ax = plt.subplots(figsize=(7, 7))

ax.set_xlim(-200, 200)
ax.set_ylim(-200, 200)

ax.set_aspect("equal")
ax.set_xlabel("X Position (mm)")
ax.set_ylabel("Y Position (mm)")
ax.set_title("Day 22: Robot Arm Animation")
ax.grid(True)

# Robot arm line
arm_line, = ax.plot(
    [],
    [],
    "o-",
    linewidth=4,
    markersize=8,
    label="Robot Arm"
)

# End effector path
path_line, = ax.plot(
    [],
    [],
    "--",
    linewidth=2,
    label="End Effector Path"
)

ax.legend()


def update(frame):

    # Get joint angles at current frame
    angles = joint_trajectory[frame]

    # Calculate robot positions
    x1, y1, x2, y2 = forward_kinematics(angles)

    # Update robot arm
    arm_line.set_data(
        [0, x1, x2],
        [0, y1, y2]
    )

    # Calculate end effector history
    x_history = []
    y_history = []

    for i in range(frame + 1):
        _, _, x, y = forward_kinematics(
            joint_trajectory[i]
        )

        x_history.append(x)
        y_history.append(y)

    # Update end effector path
    path_line.set_data(x_history, y_history)

    return arm_line, path_line


ani = FuncAnimation(
    fig,
    update,
    frames=len(joint_trajectory),
    interval=50,
    blit=True,
    repeat=False
)

plt.show()
