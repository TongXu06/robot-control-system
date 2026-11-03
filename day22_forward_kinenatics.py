
import numpy as np
import matplotlib.pyplot as plt

# 1. Robot parameters
L1 = 100
L2 = 80

start_angles = np.array([30.0, 20.0])
target_angles = np.array([70.0, 60.0])

T = 5.0
steps = 100
time = np.linspace(0, T, steps + 1)

# 2. Forward kinematics function
def forward_kinematics(angles):
    theta1 = np.radians(angles[0])
    theta2 = np.radians(angles[1])

    x1 = L1 * np.cos(theta1)
    y1 = L1 * np.sin(theta1)

    x2 = x1 + L2 * np.cos(theta1 + theta2)
    y2 = y1 + L2 * np.sin(theta1 + theta2)

    return x1, y1, x2, y2

# 3. Quintic trajectory planning
joint_trajectory = []

for t in time:
    u = t / T

    s = 10*u**3 - 15*u**4 + 6*u**5

    angles = start_angles + (
        target_angles - start_angles
    ) * s

    joint_trajectory.append(angles)

joint_trajectory = np.array(joint_trajectory)

# 4. Joint angle safety check
min_angles = np.array([0.0, -90.0])
max_angles = np.array([90.0, 90.0])

position_safe = np.all(
    (joint_trajectory >= min_angles) &
    (joint_trajectory <= max_angles)
)

# 5. Joint velocity and acceleration safety
delta = target_angles - start_angles
u = time / T

velocity = (
    delta[None, :] *
    (30*u**2 - 60*u**3 + 30*u**4)[:, None] / T
)

acceleration = (
    delta[None, :] *
    (60*u - 180*u**2 + 120*u**3)[:, None] / T**2
)

max_velocity = np.array([20.0, 20.0])
max_acceleration = np.array([15.0, 15.0])

velocity_safe = np.all(
    np.abs(velocity) <= max_velocity
)

acceleration_safe = np.all(
    np.abs(acceleration) <= max_acceleration
)

if not (position_safe and velocity_safe and acceleration_safe):
    print("Danger! Joint trajectory rejected!")
    raise SystemExit

print("Joint trajectory safety check passed!")

# 6. Calculate end effector trajectory
x_history = []
y_history = []

for angles in joint_trajectory:
    x1, y1, x2, y2 = forward_kinematics(angles)

    x_history.append(x2)
    y_history.append(y2)

# 7. Draw end effector path
plt.figure(figsize=(7, 7))

plt.plot(
    x_history,
    y_history,
    label="End Effector Path",
    linewidth=2
)

plt.scatter(
    x_history[0],
    y_history[0],
    label="Start",
    s=80
)

plt.scatter(
    x_history[-1],
    y_history[-1],
    label="Target",
    s=80
)

plt.xlabel("X Position (mm)")
plt.ylabel("Y Position (mm)")
plt.title("Day 22: Forward Kinematics")
plt.axis("equal")
plt.grid(True)
plt.legend()

plt.show()

print("Start position:",
      round(x_history[0], 2),
      round(y_history[0], 2))

print("Target position:",
      round(x_history[-1], 2),
      round(y_history[-1], 2))
