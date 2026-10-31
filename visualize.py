
import matplotlib.pyplot as plt

# Initial joint angles
shoulder = [30]
elbow = [60]
wrist = [90]

# Number of movement steps
steps = list(range(6))

# Automatically calculate joint motion
for i in range(5):
    shoulder.append(shoulder[-1] + 5)
    elbow.append(elbow[-1] + 3)
    wrist.append(wrist[-1] + 2)

print("Shoulder:", shoulder)
print("Elbow:", elbow)
print("Wrist:", wrist)


# 3. Draw three trajectories
plt.plot(steps, shoulder, label="Shoulder")
plt.plot(steps, elbow, label="Elbow")
plt.plot(steps, wrist, label="Wrist")

# 4. Add labels and title
plt.xlabel("Movement Step")
plt.ylabel("Joint Angle (degrees)")
plt.title("Robot Joint Motion")

# 5. Show legend and grid
plt.legend()
plt.grid(True)

# 6. Display the figure
plt.show()
if 0 <= angle <= 180:
    print("Safe movement")
else:
    print("Danger! Stop robot")