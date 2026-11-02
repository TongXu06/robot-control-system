
import numpy as np
import matplotlib.pyplot as plt

# Three robot joints
joint_names = ["Shoulder", "Elbow", "Wrist"]

# Initial and target angles
start_angles = np.array([30.0, 60.0, 90.0])
target_angles = np.array([80.0, 100.0, 120.0])

# Joint position limits
min_angles = np.array([0.0, 0.0, 0.0])
max_angles = np.array([90.0, 120.0, 150.0])

# Maximum velocity and acceleration
max_velocity = np.array([20.0, 20.0, 20.0])
max_acceleration = np.array([15.0, 15.0, 15.0])

# Motion duration and sampling
T = 3.0
steps = 100
time = np.linspace(0, T, steps + 1)

# Quintic polynomial trajectory
trajectory = []
velocity = []
acceleration = []

for t in time:
    s = t / T

    # Position
    q = start_angles + (target_angles - start_angles) * (
        10 * s**3 - 15 * s**4 + 6 * s**5
    )

    # Velocity
    v = (target_angles - start_angles) / T * (
        30 * s**2 - 60 * s**3 + 30 * s**4
    )

    # Acceleration
    a = (target_angles - start_angles) / T**2 * (
        60 * s - 180 * s**2 + 120 * s**3
    )

    trajectory.append(q)
    velocity.append(v)
    acceleration.append(a)

trajectory = np.array(trajectory)
velocity = np.array(velocity)
acceleration = np.array(acceleration)


# Safety checks
position_safe = np.all(
    (trajectory >= min_angles) &
    (trajectory <= max_angles)
)

velocity_safe = np.all(
    np.abs(velocity) <= max_velocity
)

acceleration_safe = np.all(
    np.abs(acceleration) <= max_acceleration
)

# Report maximum values
peak_velocity = np.max(np.abs(velocity), axis=0)
peak_acceleration = np.max(np.abs(acceleration), axis=0)

for j, name in enumerate(joint_names):
    print(
        name,
        "Peak velocity:", round(peak_velocity[j], 2),
        "Peak acceleration:", round(peak_acceleration[j], 2)
    )

if position_safe and velocity_safe and acceleration_safe:
    print("Quintic trajectory safety check passed!")
else:
    print("Danger! Trajectory rejected!")

    if not position_safe:
        print("Position limit exceeded!")

    if not velocity_safe:
        print("Velocity limit exceeded!")

    if not acceleration_safe:
        print("Acceleration limit exceeded!")


# Plot three curves
fig, axes = plt.subplots(3, 1, figsize=(9, 10))

for j, name in enumerate(joint_names):
    axes[0].plot(time, trajectory[:, j], label=name)
    axes[1].plot(time, velocity[:, j], label=name)
    axes[2].plot(time, acceleration[:, j], label=name)

axes[0].set_title("Day 21: Quintic Joint Position")
axes[0].set_ylabel("Position (degrees)")

axes[1].set_title("Joint Velocity")
axes[1].set_ylabel("Velocity (degrees/s)")

axes[2].set_title("Joint Acceleration")
axes[2].set_ylabel("Acceleration (degrees/s^2)")
axes[2].set_xlabel("Time (seconds)")

for ax in axes:
    ax.legend()
    ax.grid(True)

plt.tight_layout()
plt.show()

