
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# ============================================
# Day 27: Singularity Handling
# ============================================

# Robot parameters (mm)
L1 = 100.0
L2 = 80.0

# Desired Cartesian velocity (mm/s)
desired_velocity = np.array([10.0, 0.0])

# DLS damping parameter (mm/rad)
damping = 8.0

# Joint velocity limit (deg/s)
max_joint_speed = 30.0


# ============================================
# 1. Forward Kinematics
# ============================================

def forward_kinematics(angles):
    t1, t2 = np.radians(angles)

    x1 = L1 * np.cos(t1)
    y1 = L1 * np.sin(t1)

    x2 = x1 + L2 * np.cos(t1 + t2)
    y2 = y1 + L2 * np.sin(t1 + t2)

    return x1, y1, x2, y2


# ============================================
# 2. Jacobian Matrix
# ============================================

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


# ============================================
# 3. Damped Least Squares
# ============================================

def damped_least_squares(J, velocity, lam):
    I = np.eye(2)

    A = J @ J.T + lam**2 * I

    dq = J.T @ np.linalg.solve(A, velocity)

    return dq  # rad/s


# ============================================
# 4. Compare Three Methods
# ============================================

def calculate_methods(angles):
    J = calculate_jacobian(angles)

    # Pseudoinverse
    dq_pinv = np.linalg.pinv(J) @ desired_velocity

    # Damped least squares
    dq_dls = damped_least_squares(
        J, desired_velocity, damping
    )

    # Ordinary inverse: only if well conditioned
    condition = np.linalg.cond(J)

    if np.isfinite(condition) and condition < 1e10:
        dq_inverse = np.linalg.solve(
            J, desired_velocity
        )
    else:
        dq_inverse = np.array([np.nan, np.nan])

    return J, dq_inverse, dq_pinv, dq_dls


# ============================================
# 5. Singularity Experiment
# ============================================

# Elbow angles from 90 degrees to 0 degrees
elbow_angles = np.linspace(90.0, 0.0, 181)

inverse_speeds = []
pinv_speeds = []
dls_speeds = []

dls_actual_vx = []
condition_numbers = []

for theta2 in elbow_angles:
    angles = np.array([0.0, theta2])

    J, dq_inv, dq_pinv, dq_dls = calculate_methods(
        angles
    )

    # Norm of joint angular velocity (deg/s)
    inverse_speeds.append(
        np.linalg.norm(np.degrees(dq_inv))
    )

    pinv_speeds.append(
        np.linalg.norm(np.degrees(dq_pinv))
    )

    dls_speeds.append(
        np.linalg.norm(np.degrees(dq_dls))
    )

    # Actual Cartesian velocity from DLS
    actual_velocity = J @ dq_dls
    dls_actual_vx.append(actual_velocity[0])

    condition_numbers.append(
        np.linalg.cond(J)
    )

inverse_speeds = np.array(inverse_speeds)
pinv_speeds = np.array(pinv_speeds)
dls_speeds = np.array(dls_speeds)
dls_actual_vx = np.array(dls_actual_vx)
condition_numbers = np.array(condition_numbers)


# ============================================
# 6. Print Important Results
# ============================================

print("========== Day 27 ==========")

for theta2 in [90.0, 10.0, 1.0, 0.0]:
    angles = np.array([0.0, theta2])

    J, dq_inv, dq_pinv, dq_dls = calculate_methods(
        angles
    )

    print(f"\nElbow angle: {theta2:.1f} deg")

    print(
        "Jacobian determinant:",
        round(np.linalg.det(J), 6)
    )

    print(
        "Condition number:",
        round(np.linalg.cond(J), 2)
    )

    print(
        "Inverse joint speed:",
        round(np.linalg.norm(np.degrees(dq_inv)), 2),
        "deg/s"
    )

    print(
        "Pseudoinverse joint speed:",
        round(np.linalg.norm(np.degrees(dq_pinv)), 2),
        "deg/s"
    )

    print(
        "DLS joint speed:",
        round(np.linalg.norm(np.degrees(dq_dls)), 2),
        "deg/s"
    )

    print(
        "DLS actual Cartesian velocity:",
        np.round(J @ dq_dls, 3),
        "mm/s"
    )


# ============================================
# 7. Plot Joint Speed Comparison
# ============================================

fig1, ax1 = plt.subplots(figsize=(9, 5))

ax1.plot(
    elbow_angles,
    inverse_speeds,
    label="Ordinary Inverse"
)

ax1.plot(
    elbow_angles,
    pinv_speeds,
    label="Pseudoinverse"
)

ax1.plot(
    elbow_angles,
    dls_speeds,
    label="DLS"
)

ax1.axhline(
    max_joint_speed,
    color="red",
    linestyle="--",
    label="Reference Speed Limit"
)

ax1.set_xlabel("Elbow Angle (deg)")
ax1.set_ylabel("Joint Velocity Norm (deg/s)")
ax1.set_title("Day 27: Singularity and Joint Speed")

# Log scale makes the large speed differences visible
ax1.set_yscale("symlog", linthresh=1)
ax1.invert_xaxis()

ax1.grid(True)
ax1.legend()

plt.show()


# ============================================
# 8. Plot Cartesian Velocity Tracking
# ============================================

fig2, ax2 = plt.subplots(figsize=(9, 5))

ax2.plot(
    elbow_angles,
    dls_actual_vx,
    label="DLS Actual Vx"
)

ax2.axhline(
    desired_velocity[0],
    color="red",
    linestyle="--",
    label="Desired Vx"
)

ax2.set_xlabel("Elbow Angle (deg)")
ax2.set_ylabel("Velocity (mm/s)")
ax2.set_title("Day 27: DLS Velocity Tracking")

ax2.invert_xaxis()
ax2.grid(True)
ax2.legend()

plt.show()


# ============================================
# 9. Robot Arm Animation
# ============================================

fig3, ax3 = plt.subplots(figsize=(7, 7))

ax3.set_xlim(-200, 220)
ax3.set_ylim(-200, 200)
ax3.set_aspect("equal")
ax3.grid(True)

ax3.set_xlabel("X Position (mm)")
ax3.set_ylabel("Y Position (mm)")
ax3.set_title("Day 27: Approaching Singularity")

arm_line, = ax3.plot(
    [], [], "o-",
    linewidth=4,
    markersize=8,
    label="Robot Arm"
)

velocity_arrow = ax3.quiver(
    [0], [0], [0], [0],
    color="red",
    angles="xy",
    scale_units="xy",
    scale=1
)

status_text = ax3.text(
    0.03, 0.95, "",
    transform=ax3.transAxes,
    verticalalignment="top"
)

ax3.legend()


def update(frame):
    theta2 = elbow_angles[frame]
    angles = np.array([0.0, theta2])

    x1, y1, x2, y2 = forward_kinematics(angles)

    arm_line.set_data(
        [0, x1, x2],
        [0, y1, y2]
    )

    J = calculate_jacobian(angles)

    dq = damped_least_squares(
        J, desired_velocity, damping
    )

    actual_velocity = J @ dq

    # Scale velocity arrow for visibility
    arrow_scale = 5.0

    velocity_arrow.set_offsets([[x2, y2]])
    velocity_arrow.set_UVC(
        actual_velocity[0] * arrow_scale,
        actual_velocity[1] * arrow_scale
    )

    status_text.set_text(
        f"Elbow: {theta2:.1f} deg\n"
        f"Actual Vx: {actual_velocity[0]:.2f} mm/s\n"
        f"DLS speed: {np.linalg.norm(np.degrees(dq)):.2f} deg/s"
    )

    return arm_line, velocity_arrow, status_text


ani = FuncAnimation(
    fig3,
    update,
    frames=len(elbow_angles),
    interval=40,
    blit=False,
    repeat=False
)

plt.show()

print("\nDay 27 completed!")
