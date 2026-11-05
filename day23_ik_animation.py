
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# Robot parameters
L1 = 100.0
L2 = 80.0

start_angles = np.array([30.0, 20.0])
target_x = 120.0
target_y = 100.0

T = 5.0
steps = 100
time = np.linspace(0, T, steps + 1)

min_angles = np.array([0.0, -90.0])
max_angles = np.array([90.0, 90.0])
max_velocity = np.array([20.0, 20.0])
max_acceleration = np.array([15.0, 15.0])


def forward_kinematics(angles):
    t1, t2 = np.radians(angles)

    x1 = L1 * np.cos(t1)
    y1 = L1 * np.sin(t1)

    x2 = x1 + L2 * np.cos(t1 + t2)
    y2 = y1 + L2 * np.sin(t1 + t2)

    return x1, y1, x2, y2


def inverse_kinematics(x, y):
    r = np.hypot(x, y)

    if r > L1 + L2 or r < abs(L1 - L2):
        raise ValueError("Target is unreachable!")

    c2 = (x*x + y*y - L1*L1 - L2*L2) / (2*L1*L2)
    c2 = np.clip(c2, -1.0, 1.0)

    # Positive elbow-angle solution
    t2 = np.arccos(c2)

    t1 = (
        np.arctan2(y, x)
        - np.arctan2(
            L2 * np.sin(t2),
            L1 + L2 * np.cos(t2)
        )
    )

    return np.degrees([t1, t2])


# 1. Solve IK
target_angles = inverse_kinematics(target_x, target_y)
print("Target joint angles:", np.round(target_angles, 2))

# 2. Plan quintic trajectory
u = time / T
s = 10*u**3 - 15*u**4 + 6*u**5

delta = target_angles - start_angles

joint_trajectory = (
    start_angles[None, :] + s[:, None] * delta[None, :]
)

velocity = (
    (30*u**2 - 60*u**3 + 30*u**4)[:, None]
    * delta[None, :] / T
)

acceleration = (
    (60*u - 180*u**2 + 120*u**3)[:, None]
    * delta[None, :] / T**2
)

# 3. Safety checks
position_safe = np.all(
    (joint_trajectory >= min_angles) &
    (joint_trajectory <= max_angles)
)

velocity_safe = np.all(
    np.abs(velocity) <= max_velocity
)

acceleration_safe = np.all(
    np.abs(acceleration) <= max_acceleration
)

if not (position_safe and velocity_safe and acceleration_safe):
    raise ValueError("Unsafe joint trajectory!")

print("Trajectory safety check passed!")

# 4. Verify target position
_, _, check_x, check_y = forward_kinematics(target_angles)

error = np.hypot(check_x - target_x, check_y - target_y)
print("Final position error:", round(error, 8), "mm")

# 5. Precompute end-effector path
x_history = []
y_history = []

for angles in joint_trajectory:
    _, _, x, y = forward_kinematics(angles)
    x_history.append(x)
    y_history.append(y)

# 6. Create animation
fig, ax = plt.subplots(figsize=(7, 7))

ax.set_xlim(-200, 200)
ax.set_ylim(-200, 200)
ax.set_aspect("equal")
ax.set_xlabel("X Position (mm)")
ax.set_ylabel("Y Position (mm)")
ax.set_title("Day 23: Inverse Kinematics")
ax.grid(True)

arm_line, = ax.plot(
    [], [], "o-", linewidth=4,
    markersize=8, label="Robot Arm"
)

path_line, = ax.plot(
    [], [], "--", linewidth=2,
    label="End Effector Path"
)

ax.plot(
    target_x, target_y, "rx",
    markersize=12, markeredgewidth=3,
    label="Target"
)

ax.legend()


def update(frame):
    angles = joint_trajectory[frame]
    x1, y1, x2, y2 = forward_kinematics(angles)

    arm_line.set_data(
        [0, x1, x2],
        [0, y1, y2]
    )

    path_line.set_data(
        x_history[:frame + 1],
        y_history[:frame + 1]
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
