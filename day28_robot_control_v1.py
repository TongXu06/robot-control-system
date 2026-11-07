"""Day 28 — 2-link planar robot arm control demo (simulation only)."""
import argparse
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter

L1, L2 = 100.0, 80.0                   # mm
JOINT_MIN = np.array([0.0, -90.0])      # deg
JOINT_MAX = np.array([90.0, 90.0])      # deg
MAX_SPEED = np.array([25.0, 25.0])      # deg/s
MAX_ACCEL = np.array([30.0, 30.0])      # deg/s^2 (discrete check)


def fk(q_deg):
    t1, t2 = np.radians(q_deg)
    elbow = np.array([L1 * np.cos(t1), L1 * np.sin(t1)])
    tip = elbow + L2 * np.array([np.cos(t1 + t2), np.sin(t1 + t2)])
    return elbow, tip


def ik(xy, near=None):
    x, y = map(float, xy)
    r = np.hypot(x, y)
    if r > L1 + L2 + 1e-9 or r < abs(L1 - L2) - 1e-9:
        raise ValueError(f"Unreachable target {xy}; distance = {r:.2f} mm")
    c2 = np.clip((x*x + y*y - L1*L1 - L2*L2) / (2*L1*L2), -1.0, 1.0)
    candidates = []
    for t2 in (np.arccos(c2), -np.arccos(c2)):
        t1 = np.arctan2(y, x) - np.arctan2(L2*np.sin(t2), L1 + L2*np.cos(t2))
        q = np.degrees([t1, t2])
        if np.all(q >= JOINT_MIN - 1e-8) and np.all(q <= JOINT_MAX + 1e-8):
            candidates.append(q)
    if not candidates:
        raise ValueError(f"Target {xy} is geometrically reachable but violates joint limits")
    if near is None:
        return candidates[0]
    return min(candidates, key=lambda q: np.linalg.norm(q - near))


def jacobian(q_deg):
    t1, t2 = np.radians(q_deg)
    return np.array([
        [-L1*np.sin(t1)-L2*np.sin(t1+t2), -L2*np.sin(t1+t2)],
        [ L1*np.cos(t1)+L2*np.cos(t1+t2),  L2*np.cos(t1+t2)],
    ])


def dls(J, v, damping=8.0):
    return J.T @ np.linalg.solve(J @ J.T + damping**2*np.eye(2), v)  # rad/s


def reference_path(start, target, times, total):
    u = times / total
    s = 10*u**3 - 15*u**4 + 6*u**5
    sd = (30*u**2 - 60*u**3 + 30*u**4)/total
    delta = target - start
    return start + s[:, None]*delta, sd[:, None]*delta


def simulate(start, target, T=8.0, steps=240, kp=2.0, damping=8.0):
    if T <= 0 or steps < 10 or kp < 0 or damping <= 0:
        raise ValueError("T>0, steps>=10, Kp>=0, damping>0 are required")
    # IK checks both ends; every reference point is also checked for geometric/joint reachability.
    q0 = ik(start)
    ik(target, near=q0)
    times = np.linspace(0, T, steps+1)
    dt = T/steps
    desired, desired_vel = reference_path(start, target, times, T)
    q_near = q0
    for index, point in enumerate(desired):
        try:
            q_near = ik(point, near=q_near)
        except ValueError as exc:
            raise ValueError(f"Reference path invalid at sample {index}: {exc}") from exc

    q_history = np.empty((steps+1, 2))
    actual = np.empty((steps+1, 2))
    command_qdot = np.empty((steps, 2))
    q_history[0] = q0
    actual[0] = fk(q0)[1]
    for i in range(steps):
        position_error = desired[i] - actual[i]
        v_cmd = desired_vel[i] + kp*position_error
        J = jacobian(q_history[i])
        dq_deg = np.degrees(dls(J, v_cmd, damping))
        dq_deg = np.clip(dq_deg, -MAX_SPEED, MAX_SPEED)
        next_q = q_history[i] + dq_deg*dt
        if np.any(next_q < JOINT_MIN - 1e-8) or np.any(next_q > JOINT_MAX + 1e-8):
            raise ValueError(f"Joint position limit exceeded at step {i}")
        q_history[i+1] = next_q
        command_qdot[i] = dq_deg
        actual[i+1] = fk(next_q)[1]

    # Measured finite-difference acceleration is a post-run check, not a motor-level guarantee.
    accel = np.diff(command_qdot, axis=0)/dt
    max_acc = np.max(np.abs(accel), axis=0)
    if np.any(max_acc > MAX_ACCEL):
        raise ValueError(f"Acceleration limit exceeded: {np.round(max_acc, 3)} deg/s^2")
    errors = np.linalg.norm(desired-actual, axis=1)
    return dict(time=times, desired=desired, actual=actual, q=q_history,
                qdot=command_qdot, acceleration=accel, error=errors,
                max_acc=max_acc, desired_vel=desired_vel)


def plot_result(result, save_gif=False, no_show=False):
    time = result['time']
    fig, ax = plt.subplots(figsize=(7,7))
    ax.set(xlim=(-200,200), ylim=(-200,200), xlabel='X (mm)', ylabel='Y (mm)',
           title='Day 28 | Robot Arm Control System V1.0')
    ax.set_aspect('equal')
    ax.grid(True)
    ax.plot(result['desired'][:,0], result['desired'][:,1], 'k--', label='Reference')
    ax.plot(result['desired'][-1,0], result['desired'][-1,1], 'rx', markersize=10, label='Target')
    robot_line, = ax.plot([],[], 'o-', linewidth=4, markersize=8, label='Robot')
    trace, = ax.plot([],[], color='tab:orange', linewidth=2, label='Actual path')
    label = ax.text(.02,.98,'',transform=ax.transAxes,va='top')
    ax.legend(loc='lower left')

    def update(i):
        elbow, end = fk(result['q'][i])
        robot_line.set_data([0,elbow[0],end[0]], [0,elbow[1],end[1]])
        trace.set_data(result['actual'][:i+1,0], result['actual'][:i+1,1])
        label.set_text(f"t={time[i]:.2f}s | error={result['error'][i]:.3f} mm")
        return robot_line, trace, label

    if save_gif or not no_show:
        ani = FuncAnimation(fig, update, frames=len(time), interval=1000*(time[1]-time[0]),
                            blit=False, repeat=False)
        if save_gif:
            ani.save('day28_robot_demo.gif', writer=PillowWriter(fps=20))
            print('Saved day28_robot_demo.gif')
        if not no_show:
            plt.show()
    plt.close(fig)
    fig2, axes = plt.subplots(2,1,figsize=(9,7),sharex=True)
    axes[0].plot(time[:-1],result['qdot'][:,0],label='Shoulder')
    axes[0].plot(time[:-1],result['qdot'][:,1],label='Elbow')
    axes[0].axhline(25,color='r',ls='--',lw=1)
    axes[0].axhline(-25,color='r',ls='--',lw=1)
    axes[0].set_ylabel('Joint speed (deg/s)')
    axes[0].legend()
    axes[1].plot(time,result['error'],color='tab:orange')
    axes[1].set(xlabel='Time (s)',ylabel='Tracking error (mm)')
    for a in axes: a.grid(True)
    fig2.tight_layout()
    fig2.savefig('day28_metrics.png',dpi=150)
    print('Saved day28_metrics.png')
    if no_show:
        plt.close(fig2)
    else:
        plt.show()


def tests():
    _, point = fk(ik(np.array([120.,100.])))
    assert np.allclose(point,[120,100],atol=1e-8), 'FK/IK consistency failed'
    try:
        ik(np.array([200.,100.]))
        raise AssertionError('Unreachable target was accepted')
    except ValueError:
        pass
    J = jacobian(np.array([0.,0.]))
    assert abs(np.linalg.det(J)) < 1e-8, 'Singularity test failed'
    dq = dls(J,np.array([10.,0.]),8.0)
    assert np.all(np.isfinite(dq)), 'DLS returned non-finite speeds'
    print('4 validation checks passed.')


def main():
    parser=argparse.ArgumentParser(description='Day28 simulated robot controller')
    parser.add_argument('--test',action='store_true')
    parser.add_argument('--save-gif',action='store_true')
    parser.add_argument('--no-show',action='store_true')
    parser.add_argument('--target',nargs=2,type=float,default=[80.,140.],metavar=('X','Y'))
    args=parser.parse_args()
    if args.test:
        tests()
        return
    result=simulate(np.array([120.,100.]),np.array(args.target))
    print('Final error: %.4f mm' % result['error'][-1])
    print('Maximum tracking error: %.4f mm' % np.max(result['error']))
    print('Peak joint speeds (deg/s):', np.round(np.max(np.abs(result['qdot']),axis=0),2))
    print('Peak sampled acceleration (deg/s^2):',np.round(result['max_acc'],2))
    print('End effector target within 1 mm:', bool(result['error'][-1]<1.0))
    plot_result(result,save_gif=args.save_gif,no_show=args.no_show)
    print('Day 28 simulation completed.')


if __name__ == '__main__':
    main()
