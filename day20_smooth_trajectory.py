import numpy as np
import matplotlib.pyplot as plt

# Robot joint configuration
start_angles = np.array([30, 60, 90])
target_angles = np.array([80, 100, 120])

min_angles = np.array([0, 0, 0])
max_angles = np.array([90, 120, 150])

joint_names = ["Shoulder", "Elbow", "Wrist"]

steps = 50
duration = 5.0
time = np.linspace(0, duration, steps + 1)

t = time / duration
smooth_t = 3 * t**2 - 2 * t**3
trajectory = []

for s in smooth_t:
    angles = start_angles + (target_angles - start_angles) * s
    trajectory.append(angles)

trajectory = np.array(trajectory)
velocity = []

for x in t:
    v = (target_angles - start_angles) / duration * (6*x - 6*x**2)
    velocity.append(v)

velocity = np.array(velocity)
def check_safety(angles):
    return np.all(
        (angles >= min_angles) &
        (angles <= max_angles)
    )

safe = True
# Maximum allowed joint velocities (degrees/s)
max_velocity = np.array([10, 10, 10])

for i, v in enumerate(velocity):
    if np.any(np.abs(v) > max_velocity):
        print("Danger! Velocity limit exceeded!")
        print("Unsafe step:", i)
        print("Velocity:", v)
        safe = False
        break

for i, angles in enumerate(trajectory):
    if not check_safety(angles):
        print("Danger! Unsafe trajectory!")
        print("Unsafe step:", i)
        print("Unsafe angles:", angles)
        safe = False
        break

for i, v in enumerate(velocity):
    if np.any(np.abs(v) > max_velocity):
        print("Danger! Velocity limit exceeded!")
        print("Unsafe step:", i)
        print("Velocity:", v)
        safe = False
        break

if safe:
    print("Smooth trajectory safety check passed!")

    plt.figure(figsize=(10, 5))

    for j in range(3):
        plt.plot(
            time,
            trajectory[:, j],
            label=joint_names[j]
        )

    plt.xlabel("Time (seconds)")
    plt.ylabel("Joint Angle (degrees)")
    plt.title("Day 20: Smooth Joint Trajectory")
    plt.legend()
    plt.grid(True)

    plt.figure(figsize=(10, 5))

    for j in range(3):
        plt.plot(
            time,
            velocity[:, j],
            label=joint_names[j]
        )

    plt.xlabel("Time (seconds)")
    plt.ylabel("Angular Velocity (degrees/s)")
    plt.title("Day 20: Joint Velocity")
    plt.legend()
    plt.grid(True)

    plt.show()