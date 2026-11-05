
import numpy as np
import matplotlib.pyplot as plt

# 1. Start and target positions (mm)
start_pos = np.array([120.0, 100.0])
target_pos = np.array([80.0, 140.0])

# 2. Time settings
T = 5.0
steps = 100
time = np.linspace(0, T, steps + 1)

# 3. Generate Cartesian trajectory
trajectory = []

for t in time:
    u = t / T

    # Quintic smoothstep
    s = 10*u**3 - 15*u**4 + 6*u**5

    # Cartesian interpolation
    pos = start_pos + (target_pos - start_pos) * s

    trajectory.append(pos)

trajectory = np.array(trajectory)

# 4. Extract X and Y
x_history = trajectory[:, 0]
y_history = trajectory[:, 1]

# 5. Visualize trajectory
plt.figure(figsize=(7, 7))

plt.plot(
    x_history,
    y_history,
    linewidth=2,
    label="Cartesian Straight-Line Path"
)

plt.scatter(
    start_pos[0], start_pos[1],
    s=80, label="Start"
)

plt.scatter(
    target_pos[0], target_pos[1],
    s=80, label="Target"
)

plt.xlabel("X Position (mm)")
plt.ylabel("Y Position (mm)")
plt.title("Day 24: Cartesian Trajectory Planning")
plt.axis("equal")
plt.grid(True)
plt.legend()

plt.show()

print("Start:", trajectory[0])
print("Target:", trajectory[-1])


L1 = 100.0
L2 = 80.0

def inverse_kinematics(x, y):
    r = np.hypot(x, y)

    if r > L1 + L2 or r < abs(L1 - L2):
        raise ValueError("Target is unreachable!")

    cos_theta2 = (
        x**2 + y**2 - L1**2 - L2**2
    ) / (2 * L1 * L2)

    cos_theta2 = np.clip(cos_theta2, -1.0, 1.0)

    theta2 = np.arccos(cos_theta2)

    theta1 = (
        np.arctan2(y, x)
        - np.arctan2(
            L2 * np.sin(theta2),
            L1 + L2 * np.cos(theta2)
        )
    )

    return np.degrees([theta1, theta2])


joint_trajectory = []

for pos in trajectory:
    angles = inverse_kinematics(pos[0], pos[1])
    joint_trajectory.append(angles)

joint_trajectory = np.array(joint_trajectory)

print("Start joint angles:", np.round(joint_trajectory[0], 2))
print("Target joint angles:", np.round(joint_trajectory[-1], 2))


# Joint limits (degrees)
min_angles = np.array([0.0, -90.0])
max_angles = np.array([90.0, 90.0])

max_velocity = np.array([20.0, 20.0])
max_acceleration = np.array([15.0, 15.0])

# Position safety
position_safe = np.all(
    (joint_trajectory >= min_angles) &
    (joint_trajectory <= max_angles)
)

# Numerical velocity and acceleration
dt = time[1] - time[0]

velocity = np.gradient(
    joint_trajectory, dt, axis=0,
    edge_order=2
)

acceleration = np.gradient(
    velocity, dt, axis=0,
    edge_order=2
)

velocity_safe = np.all(
    np.abs(velocity) <= max_velocity
)

acceleration_safe = np.all(
    np.abs(acceleration) <= max_acceleration
)

print("Position safe:", position_safe)
print("Velocity safe:", velocity_safe)
print("Acceleration safe:", acceleration_safe)

if not (position_safe and velocity_safe and acceleration_safe):
    raise ValueError("Unsafe Cartesian trajectory!")

print("Cartesian trajectory safety check passed!")


from matplotlib.animation import FuncAnimation

def forward_kinematics(angles):
    theta1, theta2 = np.radians(angles)

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
ax.set_title("Day 24: Cartesian Straight-Line Motion")
ax.grid(True)

# Desired straight-line path
ax.plot(
    x_history, y_history,
    "k--", linewidth=1.5,
    label="Desired Path"
)

arm_line, = ax.plot(
    [], [], "o-",
    linewidth=4, markersize=8,
    label="Robot Arm"
)

path_line, = ax.plot(
    [], [], linewidth=2,
    label="Actual End Effector Path"
)

ax.legend()

actual_x = []
actual_y = []

for angles in joint_trajectory:
    _, _, x, y = forward_kinematics(angles)
    actual_x.append(x)
    actual_y.append(y)

# Verify FK against desired Cartesian path
error = np.hypot(
    np.array(actual_x) - x_history,
    np.array(actual_y) - y_history
)

print("Maximum path error:", np.max(error), "mm")


def update(frame):
    angles = joint_trajectory[frame]
    x1, y1, x2, y2 = forward_kinematics(angles)

    arm_line.set_data(
        [0, x1, x2],
        [0, y1, y2]
    )

    path_line.set_data(
        actual_x[:frame + 1],
        actual_y[:frame + 1]
    )

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
