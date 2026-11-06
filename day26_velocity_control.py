
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# =====================================
# Day 26: Jacobian Velocity Control
# =====================================

# 1. Robot parameters
L1 = 100.0
L2 = 80.0

# Cartesian start and target (mm)
start_pos = np.array([120.0, 100.0])
target_pos = np.array([80.0, 140.0])

# Simulation parameters
T = 8.0
steps = 200

dt = T / steps
time = np.linspace(0, T, steps + 1)

# Controller parameters
Kp = 2.0
damping = 8.0

# Joint limits
min_angles = np.array([0.0, -90.0])
max_angles = np.array([90.0, 90.0])

# Joint velocity limits (deg/s)
max_joint_velocity = np.array([25.0, 25.0])


# =====================================
# 2. Forward Kinematics
# =====================================

def forward_kinematics(angles):
    t1, t2 = np.radians(angles)

    x1 = L1 * np.cos(t1)
    y1 = L1 * np.sin(t1)

    x2 = x1 + L2 * np.cos(t1 + t2)
    y2 = y1 + L2 * np.sin(t1 + t2)

    return x1, y1, x2, y2


# =====================================
# 3. Inverse Kinematics
# =====================================

def inverse_kinematics(x, y):
    r = np.hypot(x, y)

    if r > L1 + L2 or r < abs(L1 - L2):
        raise ValueError("Target is unreachable!")

    c2 = (
        x*x + y*y - L1*L1 - L2*L2
    ) / (2 * L1 * L2)

    c2 = np.clip(c2, -1.0, 1.0)

    t2 = np.arccos(c2)

    t1 = (
        np.arctan2(y, x)
        - np.arctan2(
            L2 * np.sin(t2),
            L1 + L2 * np.cos(t2)
        )
    )

    return np.degrees([t1, t2])


# =====================================
# 4. Jacobian Matrix
# =====================================

def calculate_jacobian(angles):
    t1, t2 = np.radians(angles)

    J = np.array([
        [
            -L1*np.sin(t1) - L2*np.sin(t1+t2),
            -L2*np.sin(t1+t2)
        ],
        [
            L1*np.cos(t1) + L2*np.cos(t1+t2),
            L2*np.cos(t1+t2)
        ]
    ])

    return J


# =====================================
# 5. Damped Least Squares
# =====================================

def damped_least_squares(J, velocity, lam):
    # J: mm/rad
    # velocity: mm/s
    # lam: damping value on the same scale as J

    I = np.eye(2)

    A = J @ J.T + (lam**2) * I

    dq = J.T @ np.linalg.solve(A, velocity)

    return dq  # rad/s


# =====================================
# 6. Plan Cartesian Reference Path
# =====================================

desired_positions = []
desired_velocities = []

delta_pos = target_pos - start_pos

for t in time:
    u = t / T

    # Quintic position interpolation
    s = 10*u**3 - 15*u**4 + 6*u**5

    # Derivative of quintic polynomial
    ds_dt = (
        30*u**2 - 60*u**3 + 30*u**4
    ) / T

    pos = start_pos + delta_pos * s
    vel = delta_pos * ds_dt

    desired_positions.append(pos)
    desired_velocities.append(vel)

desired_positions = np.array(desired_positions)
desired_velocities = np.array(desired_velocities)


# =====================================
# 7. Initialize Robot
# =====================================

angles = inverse_kinematics(
    start_pos[0],
    start_pos[1]
)

print("Start angles:", np.round(angles, 2))

angle_history = [angles.copy()]
actual_positions = []
joint_velocity_history = []
position_error_history = []

_, _, x, y = forward_kinematics(angles)
actual_positions.append([x, y])


# =====================================
# 8. Velocity Control Loop
# =====================================

for i in range(steps):

    # Current end-effector position
    _, _, x, y = forward_kinematics(angles)
    actual_pos = np.array([x, y])

    # Desired position and velocity
    desired_pos = desired_positions[i]
    desired_vel = desired_velocities[i]

    # Position feedback
    error = desired_pos - actual_pos

    # Command Cartesian velocity
    command_vel = desired_vel + Kp * error

    # Current Jacobian
    J = calculate_jacobian(angles)

    # DLS velocity solution (rad/s)
    dq = damped_least_squares(
        J,
        command_vel,
        damping
    )

    # Convert to degrees/s
    dq_deg = np.degrees(dq)

    # Velocity limiting
    dq_deg = np.clip(
        dq_deg,
        -max_joint_velocity,
        max_joint_velocity
    )

    # Euler integration
    next_angles = angles + dq_deg * dt

    # Joint position safety
    if np.any(next_angles < min_angles) or \
       np.any(next_angles > max_angles):
        raise ValueError(
            f"Joint limit exceeded at step {i}!"
        )

    # Update joint angles
    angles = next_angles

    # Store simulation data
    angle_history.append(angles.copy())
    joint_velocity_history.append(dq_deg.copy())
    position_error_history.append(
        np.linalg.norm(error)
    )

    _, _, x, y = forward_kinematics(angles)
    actual_positions.append([x, y])


# =====================================
# 9. Convert Results to Arrays
# =====================================

angle_history = np.array(angle_history)
actual_positions = np.array(actual_positions)
joint_velocity_history = np.array(
    joint_velocity_history
)

position_error_history = np.array(
    position_error_history
)

final_error = np.linalg.norm(
    actual_positions[-1] - target_pos
)

print("Target position:", target_pos)
print("Final position:", np.round(actual_positions[-1], 2))
print("Final position error:", round(final_error, 4), "mm")

print(
    "Maximum tracking error:",
    round(np.max(position_error_history), 4),
    "mm"
)

print(
    "Maximum joint speed:",
    round(np.max(np.abs(joint_velocity_history)), 2),
    "deg/s"
)

if final_error < 1.0:
    print("Target reached within 1 mm tolerance!")
else:
    print("Warning: Target accuracy needs improvement.")


# =====================================
# 10. Robot Arm Animation
# =====================================

fig, ax = plt.subplots(figsize=(7, 7))

ax.set_xlim(-200, 200)
ax.set_ylim(-200, 200)
ax.set_aspect("equal")

ax.set_xlabel("X Position (mm)")
ax.set_ylabel("Y Position (mm)")
ax.set_title("Day 26: Jacobian Velocity Control")
ax.grid(True)

ax.plot(
    desired_positions[:, 0],
    desired_positions[:, 1],
    "k--",
    label="Desired Path"
)

ax.plot(
    target_pos[0],
    target_pos[1],
    "rx",
    markersize=12,
    label="Target"
)

arm_line, = ax.plot(
    [], [],
    "o-",
    linewidth=4,
    markersize=8,
    label="Robot Arm"
)

path_line, = ax.plot(
    [], [],
    color="orange",
    linewidth=2,
    label="Actual Path"
)

ax.legend()


def update(frame):
    q = angle_history[frame]

    x1, y1, x2, y2 = forward_kinematics(q)

    arm_line.set_data(
        [0, x1, x2],
        [0, y1, y2]
    )

    path_line.set_data(
        actual_positions[:frame+1, 0],
        actual_positions[:frame+1, 1]
    )

    return arm_line, path_line


ani = FuncAnimation(
    fig,
    update,
    frames=len(angle_history),
    interval=40,
    blit=False,
    repeat=False
)

plt.show()


# =====================================
# 11. Plot Joint Angular Velocities
# =====================================

plt.figure(figsize=(9, 5))

plt.plot(
    time[:-1],
    joint_velocity_history[:, 0],
    label="Shoulder Velocity"
)

plt.plot(
    time[:-1],
    joint_velocity_history[:, 1],
    label="Elbow Velocity"
)

plt.axhline(
    25, color="red",
    linestyle="--", alpha=0.5
)
plt.axhline(
    -25, color="red",
    linestyle="--", alpha=0.5
)

plt.xlabel("Time (s)")
plt.ylabel("Joint Velocity (deg/s)")
plt.title("Day 26: Joint Velocity Control")
plt.grid(True)
plt.legend()

plt.show()


# =====================================
# 12. Plot Tracking Error
# =====================================

tracking_error = np.linalg.norm(
    desired_positions - actual_positions,
    axis=1
)

plt.figure(figsize=(9, 5))

plt.plot(
    time,
    tracking_error,
    label="Position Tracking Error"
)

plt.xlabel("Time (s)")
plt.ylabel("Position Error (mm)")
plt.title("Day 26: End Effector Tracking Error")
plt.grid(True)
plt.legend()

plt.show()

print("\nDay 26 completed!")

