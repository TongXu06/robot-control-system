
import numpy as np
import matplotlib.pyplot as plt

# ==========================================
# Day 25: Robot Jacobian Matrix
# ==========================================

# 1. Robot parameters
L1 = 100.0
L2 = 80.0

# Current joint angles (degrees)
theta1 = 30.0
theta2 = 45.0

# Joint angular velocities (degrees/s)
omega1 = 10.0
omega2 = 20.0


# ==========================================
# 2. Forward Kinematics
# ==========================================

def forward_kinematics(angles):

    t1, t2 = np.radians(angles)

    x1 = L1 * np.cos(t1)
    y1 = L1 * np.sin(t1)

    x2 = x1 + L2 * np.cos(t1 + t2)
    y2 = y1 + L2 * np.sin(t1 + t2)

    return x1, y1, x2, y2


# ==========================================
# 3. Jacobian Matrix
# ==========================================

def calculate_jacobian(angles):

    t1, t2 = np.radians(angles)

    J = np.array([
        [
            -L1 * np.sin(t1) - L2 * np.sin(t1 + t2),
            -L2 * np.sin(t1 + t2)
        ],
        [
            L1 * np.cos(t1) + L2 * np.cos(t1 + t2),
            L2 * np.cos(t1 + t2)
        ]
    ])

    return J


# ==========================================
# 4. Current End Effector Velocity
# ==========================================

current_angles = np.array([theta1, theta2])

joint_velocity = np.radians(
    [omega1, omega2]
)

J = calculate_jacobian(current_angles)

end_velocity = J @ joint_velocity

vx = end_velocity[0]
vy = end_velocity[1]

speed = np.hypot(vx, vy)

print("========== Day 25 ==========")
print("Jacobian Matrix:")
print(np.round(J, 2))

print("\nEnd Effector Velocity:")
print("Vx:", round(vx, 2), "mm/s")
print("Vy:", round(vy, 2), "mm/s")
print("Speed:", round(speed, 2), "mm/s")


# ==========================================
# 5. Singularity Detection
# ==========================================

det_J = np.linalg.det(J)

print("\nJacobian determinant:", round(det_J, 6))

if abs(det_J) < 1e-6:
    print("Warning: Singular configuration!")
else:
    print("Jacobian is non-singular.")


# ==========================================
# 6. Visualize Robot and Velocity
# ==========================================

x1, y1, x2, y2 = forward_kinematics(
    current_angles
)

fig, ax = plt.subplots(figsize=(7, 7))

ax.plot(
    [0, x1, x2],
    [0, y1, y2],
    "o-",
    linewidth=4,
    markersize=8,
    label="Robot Arm"
)

ax.quiver(
    x2, y2,
    vx, vy,
    angles="xy",
    scale_units="xy",
    scale=1,
    color="red",
    label="End Effector Velocity"
)

ax.set_xlim(-200, 200)
ax.set_ylim(-200, 200)
ax.set_aspect("equal")

ax.set_xlabel("X Position (mm)")
ax.set_ylabel("Y Position (mm)")
ax.set_title("Day 25: Jacobian Velocity Visualization")

ax.grid(True)
ax.legend()

plt.show()


# ==========================================
# 7. Day 24 Cartesian Trajectory
# ==========================================

start_pos = np.array([120.0, 100.0])
target_pos = np.array([80.0, 140.0])

T = 5.0
steps = 100

time = np.linspace(0, T, steps + 1)

trajectory = []

for t in time:

    u = t / T

    # Quintic smoothstep
    s = 10*u**3 - 15*u**4 + 6*u**5

    pos = start_pos + (
        target_pos - start_pos
    ) * s

    trajectory.append(pos)

trajectory = np.array(trajectory)


# ==========================================
# 8. Inverse Kinematics
# ==========================================

def inverse_kinematics(x, y):

    r = np.hypot(x, y)

    if r > L1 + L2 or r < abs(L1 - L2):
        raise ValueError("Target is unreachable!")

    c2 = (
        x**2 + y**2 - L1**2 - L2**2
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


# Calculate joint angles along Cartesian path
joint_trajectory = []

for pos in trajectory:

    angles = inverse_kinematics(
        pos[0], pos[1]
    )

    joint_trajectory.append(angles)

joint_trajectory = np.array(joint_trajectory)


# ==========================================
# 9. Joint Velocity and Acceleration
# ==========================================

dt = time[1] - time[0]

joint_velocity_history = np.gradient(
    joint_trajectory,
    dt,
    axis=0,
    edge_order=2
)

joint_acceleration_history = np.gradient(
    joint_velocity_history,
    dt,
    axis=0,
    edge_order=2
)

# Joint safety limits
min_angles = np.array([0.0, -90.0])
max_angles = np.array([90.0, 90.0])

max_velocity = np.array([20.0, 20.0])
max_acceleration = np.array([15.0, 15.0])

position_safe = np.all(
    (joint_trajectory >= min_angles) &
    (joint_trajectory <= max_angles)
)

velocity_safe = np.all(
    np.abs(joint_velocity_history) <= max_velocity
)

acceleration_safe = np.all(
    np.abs(joint_acceleration_history) <= max_acceleration
)

print("\n========== Trajectory Safety ==========")
print("Position safe:", position_safe)
print("Velocity safe:", velocity_safe)
print("Acceleration safe:", acceleration_safe)

if not (position_safe and velocity_safe and acceleration_safe):
    raise ValueError("Unsafe trajectory!")

print("Trajectory safety check passed!")


# ==========================================
# 10. Jacobian Velocity Along Trajectory
# ==========================================

vx_history = []
vy_history = []

for i in range(len(time)):

    angles = joint_trajectory[i]

    J = calculate_jacobian(angles)

    # degrees/s -> radians/s
    dq = np.radians(
        joint_velocity_history[i]
    )

    v = J @ dq

    vx_history.append(v[0])
    vy_history.append(v[1])

vx_history = np.array(vx_history)
vy_history = np.array(vy_history)

cartesian_speed = np.hypot(
    vx_history, vy_history
)

print("\nMaximum Cartesian speed:",
      round(np.max(cartesian_speed), 2),
      "mm/s")


# ==========================================
# 11. Plot Joint Velocities
# ==========================================

plt.figure(figsize=(9, 5))

plt.plot(
    time,
    joint_velocity_history[:, 0],
    label="Shoulder Velocity (deg/s)"
)

plt.plot(
    time,
    joint_velocity_history[:, 1],
    label="Elbow Velocity (deg/s)"
)

plt.xlabel("Time (s)")
plt.ylabel("Joint Velocity (deg/s)")
plt.title("Day 25: Joint Velocities")
plt.grid(True)
plt.legend()

plt.show()


# ==========================================
# 12. Plot Cartesian Velocities
# ==========================================

plt.figure(figsize=(9, 5))

plt.plot(
    time,
    vx_history,
    label="Vx (mm/s)"
)

plt.plot(
    time,
    vy_history,
    label="Vy (mm/s)"
)

plt.xlabel("Time (s)")
plt.ylabel("End Effector Velocity (mm/s)")
plt.title("Day 25: Cartesian Velocity via Jacobian")
plt.grid(True)
plt.legend()

plt.show()


# ==========================================
# 13. Verify Jacobian Result
# ==========================================

desired_delta = target_pos - start_pos

u = time / T

smooth_derivative = (
    30*u**2 - 60*u**3 + 30*u**4
) / T

expected_velocity = (
    smooth_derivative[:, None]
    * desired_delta[None, :]
)

velocity_error = np.hypot(
    vx_history - expected_velocity[:, 0],
    vy_history - expected_velocity[:, 1]
)

print("\nMaximum velocity error:",
      round(np.max(velocity_error), 6),
      "mm/s")

print("\nDay 25 completed!")
