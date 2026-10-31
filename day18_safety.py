
import numpy as np
import matplotlib.pyplot as plt

# 1. Initial joint angles
angles = np.array([30, 60, 90])

# 2. Safety limits
min_angles = np.array([0, 0, 0])
max_angles = np.array([90, 120, 180])

# 3. Movement settings
steps = 10
movement = np.array([10, 5, 3])

# 4. Store motion history
history = [angles.copy()]

# 5. Robot movement simulation
for i in range(steps):

    new_angles = angles + movement

    # Safety check
    is_safe = np.all(
        (new_angles >= min_angles) &
        (new_angles <= max_angles)
    )

    if is_safe:
        angles = new_angles
        history.append(angles.copy())
        print("Step", i + 1, "Safe:", angles)
    
    else:
        print("Danger! Movement rejected!")

        joint_names = ["Shoulder", "Elbow", "Wrist"]

        for j in range(3):
            if new_angles[j] > max_angles[j]:
                print(
                    joint_names[j],
                    "exceeds maximum limit:",
                    new_angles[j],
                    ">",
                    max_angles[j]
                )

            elif new_angles[j] < min_angles[j]:
                print(
                    joint_names[j],
                    "below minimum limit:",
                    new_angles[j],
                    "<",
                    min_angles[j]
                )

        break


# 6. Convert history to NumPy array
history = np.array(history)

# 7. Draw safe trajectories
plt.plot(history[:, 0], label="Shoulder")
plt.plot(history[:, 1], label="Elbow")
plt.plot(history[:, 2], label="Wrist")

plt.xlabel("Movement Step")
plt.ylabel("Joint Angle (degrees)")
plt.title("Robot Motion with Safety Check")

plt.legend()
plt.grid(True)
plt.show()
