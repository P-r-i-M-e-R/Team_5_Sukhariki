import os
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation


# ---------- style defaults ----------
plt.rcParams.update({
    "figure.dpi": 150,
    "font.size": 10,
    "axes.grid": True,
    "grid.alpha": 0.3,
})


# =====================================================================
# 1.  Comparison plots (Lyapunov vs PID)
# =====================================================================

def plot_comparison(
    times, states_lyap, torques_lyap, states_pid, torques_pid,
    theta_d, save_dir="figures",
):
    """6-panel figure: angles, errors, torques for both controllers."""
    os.makedirs(save_dir, exist_ok=True)

    target = np.asarray(theta_d)
    err_lyap = states_lyap[:, :2] - target
    err_pid = states_pid[:, :2] - target

    fig, axs = plt.subplots(3, 2, figsize=(14, 9), sharex=True)

    # --- row 0: angles ---
    axs[0, 0].plot(times, states_lyap[:, 0], label=r"$\theta_1$")
    axs[0, 0].plot(times, states_lyap[:, 1], label=r"$\theta_2$")
    axs[0, 0].axhline(target[0], color="r", ls="--", lw=0.8, label=r"$\theta_{1d}$")
    axs[0, 0].axhline(target[1], color="g", ls="--", lw=0.8, label=r"$\theta_{2d}$")
    axs[0, 0].set_ylabel("Angle [rad]")
    axs[0, 0].set_title("Joint angles - Lyapunov")
    axs[0, 0].legend(fontsize=8)

    axs[0, 1].plot(times, states_pid[:, 0], label=r"$\theta_1$")
    axs[0, 1].plot(times, states_pid[:, 1], label=r"$\theta_2$")
    axs[0, 1].axhline(target[0], color="r", ls="--", lw=0.8)
    axs[0, 1].axhline(target[1], color="g", ls="--", lw=0.8)
    axs[0, 1].set_ylabel("Angle [rad]")
    axs[0, 1].set_title("Joint angles - PID")
    axs[0, 1].legend(fontsize=8)

    # --- row 1: tracking errors ---
    axs[1, 0].plot(times, err_lyap[:, 0], label=r"$e_1$")
    axs[1, 0].plot(times, err_lyap[:, 1], label=r"$e_2$")
    axs[1, 0].set_ylabel("Error [rad]")
    axs[1, 0].set_title("Tracking error - Lyapunov")
    axs[1, 0].legend(fontsize=8)

    axs[1, 1].plot(times, err_pid[:, 0], label=r"$e_1$")
    axs[1, 1].plot(times, err_pid[:, 1], label=r"$e_2$")
    axs[1, 1].set_ylabel("Error [rad]")
    axs[1, 1].set_title("Tracking error - PID")
    axs[1, 1].legend(fontsize=8)

    # --- row 2: control torques ---
    axs[2, 0].plot(times, torques_lyap[:, 0], label=r"$\tau_1$")
    axs[2, 0].plot(times, torques_lyap[:, 1], label=r"$\tau_2$")
    axs[2, 0].set_ylabel("Torque [N·m]")
    axs[2, 0].set_xlabel("Time [s]")
    axs[2, 0].set_title("Control action - Lyapunov")
    axs[2, 0].legend(fontsize=8)

    axs[2, 1].plot(times, torques_pid[:, 0], label=r"$\tau_1$")
    axs[2, 1].plot(times, torques_pid[:, 1], label=r"$\tau_2$")
    axs[2, 1].set_ylabel("Torque [N·m]")
    axs[2, 1].set_xlabel("Time [s]")
    axs[2, 1].set_title("Control action - PID")
    axs[2, 1].legend(fontsize=8)

    fig.suptitle(
        "Comparison: Lyapunov vs PID",
        fontsize=13, fontweight="bold",
    )
    fig.tight_layout(rect=[0, 0, 1, 0.96])
    path = os.path.join(save_dir, "comparison_plots.png")
    fig.savefig(path)
    plt.close(fig)
    print(f"  Saved {path}")


# =====================================================================
# 2.  Lyapunov function plot
# =====================================================================

def plot_lyapunov(times, L_vals, dL_vals, save_dir="figures"):
    """Plot L(t) and dL/dt over time to verify the stability guarantee."""
    os.makedirs(save_dir, exist_ok=True)

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 6), sharex=True)

    ax1.plot(times, L_vals, color="tab:blue")
    ax1.set_ylabel(r"$L(t)$")
    ax1.set_title(r"Lyapunov function $L(t)$")

    ax2.plot(times, dL_vals, color="tab:red")
    ax2.axhline(0, color="k", ls="--", lw=0.5)
    ax2.set_ylabel(r"$\dot{L}(t)$")
    ax2.set_xlabel("Time [s]")
    ax2.set_title(r"$\dot{L}(t)$")

    fig.tight_layout()
    path = os.path.join(save_dir, "lyapunov_function.png")
    fig.savefig(path)
    plt.close(fig)
    print(f"  Saved {path}")


# =====================================================================
# 3.  Phase portrait
# =====================================================================

def plot_phase_portrait(times, states, theta_d, save_dir="figures"):
    """Phase portrait (theta_i, dtheta_i) for each joint."""
    os.makedirs(save_dir, exist_ok=True)

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 5))

    # Joint 1
    ax1.plot(states[:, 0], states[:, 2], "b-", lw=0.7)
    ax1.plot(states[0, 0], states[0, 2], "go", ms=8, label="start")
    ax1.plot(theta_d[0], 0, "r*", ms=12, label="target")
    ax1.set_xlabel(r"$\theta_1$ [rad]")
    ax1.set_ylabel(r"$\dot{\theta}_1$ [rad/s]")
    ax1.set_title("Phase portrait - Joint 1")
    ax1.legend()

    # Joint 2
    ax2.plot(states[:, 1], states[:, 3], "b-", lw=0.7)
    ax2.plot(states[0, 1], states[0, 3], "go", ms=8, label="start")
    ax2.plot(theta_d[1], 0, "r*", ms=12, label="target")
    ax2.set_xlabel(r"$\theta_2$ [rad]")
    ax2.set_ylabel(r"$\dot{\theta}_2$ [rad/s]")
    ax2.set_title("Phase portrait - Joint 2")
    ax2.legend()

    fig.tight_layout()
    path = os.path.join(save_dir, "phase_portrait.png")
    fig.savefig(path)
    plt.close(fig)
    print(f"  Saved {path}")


# =====================================================================
# 4.  Animation
# =====================================================================

def create_animation(times, states, l1, l2, theta_d, save_path="animations/robot_motion.gif"):
    """Animated two-link arm with target configuration overlay."""
    os.makedirs(os.path.dirname(save_path), exist_ok=True)

    fig, ax = plt.subplots(figsize=(6, 6))
    lim = (l1 + l2) * 1.3
    ax.set_xlim(-lim, lim)
    ax.set_ylim(-lim, lim)
    ax.set_aspect("equal")
    ax.grid(True, alpha=0.3)
    ax.set_title("Two-Link Manipulator - Lyapunov Control")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")

    # target configuration (static)
    th1d, th2d = theta_d
    x1d = l1 * np.cos(th1d)
    y1d = l1 * np.sin(th1d)
    x2d = x1d + l2 * np.cos(th1d + th2d)
    y2d = y1d + l2 * np.sin(th1d + th2d)
    ax.plot([0, x1d, x2d], [0, y1d, y2d], "r--o", lw=1.5, ms=6, alpha=0.4, label="target")

    (arm_line,) = ax.plot([], [], "o-", color="tab:blue", lw=3, ms=8)
    time_text = ax.text(0.02, 0.95, "", transform=ax.transAxes, fontsize=10)
    ax.legend(loc="upper right", fontsize=9)

    # sub-sample for smoother file size
    skip = max(1, len(times) // 500)
    idx = np.arange(0, len(times), skip)

    def _update(frame):
        i = idx[frame]
        th1, th2 = states[i, 0], states[i, 1]
        x1 = l1 * np.cos(th1)
        y1 = l1 * np.sin(th1)
        x2 = x1 + l2 * np.cos(th1 + th2)
        y2 = y1 + l2 * np.sin(th1 + th2)
        arm_line.set_data([0, x1, x2], [0, y1, y2])
        time_text.set_text(f"t = {times[i]:.2f} s")
        return arm_line, time_text

    ani = animation.FuncAnimation(fig, _update, frames=len(idx), blit=True, interval=30)
    ani.save(save_path, writer="pillow", fps=30)
    plt.close(fig)
    print(f"  Saved {save_path}")
