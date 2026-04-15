import os
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

plt.rcParams.update({
    "figure.dpi": 150,
    "font.size": 10,
    "axes.grid": True,
    "grid.alpha": 0.3,
})


# ------------------------------------------------------------------
#     Comparison plots (Lyapunov vs PID)
# ------------------------------------------------------------------

def plot_comparison(times, states_lyap, torques_lyap, states_pid, torques_pid, theta_d, save_dir="figures",):
    """6-panel figure: angles, errors, torques for both controllers."""

    os.makedirs(save_dir, exist_ok=True)

    target = np.asarray(theta_d)
    err_lyap = states_lyap[:, :2] - target
    err_pid = states_pid[:, :2] - target

    fig, axs = plt.subplots(3, 2, figsize=(14, 9), sharex=True)

    # row 0: angles
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

    # row 1: tracking errors
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

    # row 2: control torques
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


# ------------------------------------------------------------------
#     Lyapunov function plot
# ------------------------------------------------------------------

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


# ------------------------------------------------------------------
#     Phase portrait
# ------------------------------------------------------------------

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


# ------------------------------------------------------------------
#     Animation
# ------------------------------------------------------------------

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


# ------------------------------------------------------------------
#     Gain sensitivity analysis
# ------------------------------------------------------------------

def plot_k_sensitivity_comparison(results: dict, save_dir: str = "figures") -> None:
    """
    Compare performance across 5 different (k1, k2) pairs.
    
    Parameters:
    results: dict with keys 'k_configs' (list of dicts) and 'simulations' (list of tuples)
    """
    os.makedirs(save_dir, exist_ok=True)
    
    simulations = results["simulations"]
    descriptions = [sim[5] for sim in simulations]
    colors = plt.cm.viridis(np.linspace(0, 1, len(simulations)))
    
    # Figure 1: Joint angles comparison
    fig, axes = plt.subplots(2, 1, figsize=(12, 8), sharex=True)
    
    for idx, (times, states, torques, k1, k2, desc) in enumerate(simulations):
        theta_d = results["theta_d"]
        color = colors[idx]
        
        # Joint 1 angles
        axes[0].plot(times, states[:, 0], color=color, label=desc, linewidth=2)
        # Joint 2 angles
        axes[1].plot(times, states[:, 1], color=color, linewidth=2)
    
    # Target lines
    axes[0].axhline(results["theta_d"][0], color="r", ls="--", lw=1.5, alpha=0.7, label=r"$\theta_{1d}$")
    axes[1].axhline(results["theta_d"][1], color="r", ls="--", lw=1.5, alpha=0.7, label=r"$\theta_{2d}$")
    
    axes[0].set_ylabel(r"$\theta_1$ [rad]", fontsize=11)
    axes[1].set_ylabel(r"$\theta_2$ [rad]", fontsize=11)
    axes[1].set_xlabel("Time [s]")
    axes[0].set_title("Joint Angles - Sensitivity to Gain Values")
    axes[0].legend(loc="best", fontsize=9)
    axes[1].legend(loc="best", fontsize=9)
    axes[0].grid(True, alpha=0.3)
    axes[1].grid(True, alpha=0.3)
    
    fig.tight_layout()
    path = os.path.join(save_dir, "k_sensitivity_angles.png")
    fig.savefig(path, dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f"  Saved {path}")
    
    # Figure 2: Tracking errors comparison
    fig, axes = plt.subplots(2, 1, figsize=(12, 8), sharex=True)
    
    for idx, (times, states, torques, k1, k2, desc) in enumerate(simulations):
        theta_d = results["theta_d"]
        error = states[:, :2] - theta_d
        color = colors[idx]
        
        # Error in joint 1
        axes[0].plot(times, error[:, 0], color=color, label=desc, linewidth=2)
        # Error in joint 2
        axes[1].plot(times, error[:, 1], color=color, linewidth=2)
    
    axes[0].axhline(0, color="k", ls="--", lw=0.8, alpha=0.5)
    axes[1].axhline(0, color="k", ls="--", lw=0.8, alpha=0.5)
    
    axes[0].set_ylabel(r"$e_1 = \theta_1 - \theta_{1d}$ [rad]", fontsize=11)
    axes[1].set_ylabel(r"$e_2 = \theta_2 - \theta_{2d}$ [rad]", fontsize=11)
    axes[1].set_xlabel("Time [s]")
    axes[0].set_title("Tracking Errors - Sensitivity to Gain Values")
    axes[0].legend(loc="best", fontsize=9)
    axes[0].grid(True, alpha=0.3)
    axes[1].grid(True, alpha=0.3)
    
    fig.tight_layout()
    path = os.path.join(save_dir, "k_sensitivity_errors.png")
    fig.savefig(path, dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f"  Saved {path}")
    
    # Figure 3: Control torques comparison
    fig, axes = plt.subplots(2, 1, figsize=(12, 8), sharex=True)
    
    # Plot "Very Large" (index 4) first with low zorder so others appear on top
    if len(simulations) > 4:
        times, states, torques, k1, k2, desc = simulations[4]
        color = colors[4]
        # Torque on joint 1
        axes[0].plot(times, torques[:, 0], color=color, label=desc, linewidth=2, zorder=1)
        # Torque on joint 2
        axes[1].plot(times, torques[:, 1], color=color, linewidth=2, zorder=1)
    
    # Plot all others with higher zorder
    for idx, (times, states, torques, k1, k2, desc) in enumerate(simulations):
        if idx == 4:  # Skip "Very Large" since we already plotted it
            continue
        color = colors[idx]
        
        # Torque on joint 1
        axes[0].plot(times, torques[:, 0], color=color, label=desc, linewidth=2, zorder=2)
        # Torque on joint 2
        axes[1].plot(times, torques[:, 1], color=color, linewidth=2, zorder=2)
    
    axes[0].set_ylabel(r"$\tau_1$ [N·m]", fontsize=11)
    axes[1].set_ylabel(r"$\tau_2$ [N·m]", fontsize=11)
    axes[1].set_xlabel("Time [s]")
    axes[0].set_title("Control Torques - Sensitivity to Gain Values")
    axes[0].legend(loc="best", fontsize=9)
    axes[0].grid(True, alpha=0.3)
    axes[1].grid(True, alpha=0.3)
    
    fig.tight_layout()
    path = os.path.join(save_dir, "k_sensitivity_torques.png")
    fig.savefig(path, dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f"  Saved {path}")
    
    # Figure 4: Performance metrics table
    fig, ax = plt.subplots(figsize=(12, 5))
    ax.axis('tight')
    ax.axis('off')
    
    # Configuration names list (independent of gain_analysis)
    config_names = ["Very Small", "Small", "Current", "Large", "Very Large"]
    
    # Compute metrics
    metrics_data = []
    for idx, (times, states, torques, k1, k2, desc) in enumerate(simulations):
        error = states[:, :2] - results["theta_d"]
        error_norm = np.linalg.norm(error, axis=1)
        
        settling_threshold = 0.02
        settling_idx = np.where(error_norm < settling_threshold)[0]
        settling_time = times[settling_idx[0]] if len(settling_idx) > 0 else times[-1]
        
        max_error = np.max(error_norm)
        final_error = error_norm[-1]
        max_torque = np.max(np.abs(torques))
        control_energy = np.sum(torques**2) * (times[1] - times[0])
        
        config_name = config_names[idx] if idx < len(config_names) else desc
        
        metrics_data.append([
            config_name,
            f"{k1:.0f}",
            f"{k2:.0f}",
            f"{settling_time:.2f}",
            f"{max_error:.4f}",
            f"{final_error:.6f}",
            f"{max_torque:.1f}",
            f"{control_energy:.0f}",
        ])
    
    columns = [r"$Configuration$", r"$k_1$", r"$k_2$", 
               "Settling\nTime [s]", "Max Error\n[rad]", 
               "Final Error\n[rad]", "Max Torque\n[N·m]", 
               "Energy\n[N·m·s]"]
    
    table = ax.table(cellText=metrics_data, colLabels=columns, cellLoc='center', 
                     loc='center', bbox=[0, 0, 1, 1])
    table.auto_set_font_size(False)
    table.set_fontsize(9)
    table.scale(1, 2)
    
    # Style header
    for i in range(len(columns)):
        table[(0, i)].set_facecolor('#4472C4')
        table[(0, i)].set_text_props(weight='bold', color='white')
    
    # Alternate row colors
    for i in range(1, len(metrics_data) + 1):
        for j in range(len(columns)):
            if i % 2 == 0:
                table[(i, j)].set_facecolor('#E7E6E6')
            else:
                table[(i, j)].set_facecolor('#F2F2F2')
    
    path = os.path.join(save_dir, "k_sensitivity_metrics_table.png")
    fig.savefig(path, dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f"  Saved {path}")
