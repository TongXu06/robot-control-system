
import numpy as np
import matplotlib.pyplot as plt

# 1. Robot configuration
start_angles = np.array([30, 60, 90])
target_angles = np.array([80, 100, 120])

min_angles = np.array([0, 0, 0])
max_angles = np.array([90, 120, 150])

joint_names = ["Shoulder", "Elbow", "Wrist"]
steps = 10

# 2. Safety check function
def check_safety(angles):
    return np.all(
        (angles >= min_angles) &
        (angles <= max_angles)
    )

# 3. Generate linear trajectory
trajectory = []

for t in np.linspace(0, 1, steps + 1):
    angles = start_angles + (target_angles - start_angles) * t
    trajectory.append(angles)

trajectory = np.array(trajectory)

# 4. Check entire trajectory before execution
for i, angles in enumerate(trajectory):
    if not check_safety(angles):
        print("Danger! Trajectory rejected!")
        print("Unsafe step:", i)

        for j in range(len(joint_names)):
            if angles[j] > max_angles[j]:
                print(
                    joint_names[j],
                    "exceeds maximum limit:",
                    angles[j], ">", max_angles[j]
                )
            elif angles[j] < min_angles[j]:
                print(
                    joint_names[j],
                    "below minimum limit:",
                    angles[j], "<", min_angles[j]
                )
        break
else:
    print("Trajectory safety check passed!")

    # 5. Simulate execution
    for i, angles in enumerate(trajectory):
        print("Step", i, ":", angles)

    # 6. Visualize trajectory
    for j in range(len(joint_names)):
        plt.plot(
            range(steps + 1),
            trajectory[:, j],
            marker="o",
            label=joint_names[j]
        )

    plt.xlabel("Movement Step")
    plt.ylabel("Joint Angle (degrees)")
    plt.title("Day 19: Robot Trajectory Planning")
    plt.legend()
    plt.grid(True)
    plt.show()
