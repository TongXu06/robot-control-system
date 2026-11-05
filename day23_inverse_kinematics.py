
import numpy as np

# Robot arm lengths (mm)
L1 = 100
L2 = 80

# Target position
target_x = 120
target_y = 100

# Calculate distance from base to target
r = np.sqrt(target_x**2 + target_y**2)

print("Target distance:", round(r, 2), "mm")

# Check reachability
if r > L1 + L2 or r < abs(L1 - L2):
    print("Target is unreachable!")

else:
    # Calculate elbow angle
    cos_theta2 = (
        target_x**2 + target_y**2 - L1**2 - L2**2
    ) / (2 * L1 * L2)

    cos_theta2 = np.clip(cos_theta2, -1.0, 1.0)

    theta2 = np.arccos(cos_theta2)

    # Calculate shoulder angle
    theta1 = (
        np.arctan2(target_y, target_x)
        - np.arctan2(
            L2 * np.sin(theta2),
            L1 + L2 * np.cos(theta2)
        )
    )

    # Convert radians to degrees
    theta1_deg = np.degrees(theta1)
    theta2_deg = np.degrees(theta2)

    print("Shoulder angle:", round(theta1_deg, 2))
    print("Elbow angle:", round(theta2_deg, 2))

    # Verify IK using forward kinematics
    check_x = (
        L1 * np.cos(theta1)
        + L2 * np.cos(theta1 + theta2)
    )

    check_y = (
        L1 * np.sin(theta1)
        + L2 * np.sin(theta1 + theta2)
    )

    error = np.sqrt(
        (check_x - target_x)**2
        + (check_y - target_y)**2
    )

    print("FK check X:", round(check_x, 2))
    print("FK check Y:", round(check_y, 2))
    print("Position error:", round(error, 8), "mm")