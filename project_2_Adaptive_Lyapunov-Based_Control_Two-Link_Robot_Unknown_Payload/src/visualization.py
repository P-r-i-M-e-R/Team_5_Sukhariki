"""
Visualization utilities for Project 2.

The plots focus on the adaptive payload-mass estimator described in the README:
closed-loop trajectories,  Lyapunov decay, parameter estimates,
payload compensation, phase portraits, and robot animation.
"""

import os

import matplotlib.animation as animation
import matplotlib.pyplot as plt
import numpy as np


plt.rcParams.update({
    "figure.dpi": 150,
    "font.size": 10,
    "axes.grid": True,
    "grid.alpha": 0.3,
})


def _ensure_dir(path: str) -> None:
    os.makedirs(path, exist_ok=True)


def _joint_positions(theta, l1, l2):
    th1, th2 = theta
    x1 = l1 * np.cos(th1)
    y1 = l1 * np.sin(th1)
    x2 = x1 + l2 * np.cos(th1 + th2)
    y2 = y1 + l2 * np.sin(th1 + th2)
    return np.array([0.0, x1, x2]), np.array([0.0, y1, y2])


def plot_comparison(
    times,
    adaptive_states,
    adaptive_torques,
    baseline_states,
    baseline_torques,
    theta_d,
    save_dir="figures",
):
    """Compare adaptive and non-adaptive PD+gravity controllers."""
    _ensure_dir(save_dir)

    target = np.asarray(theta_d)
    adaptive_error = adaptive_states[:, :2] - target
    baseline_error = baseline_states[:, :2] - target

    fig, axs = plt.subplots(3, 2, figsize=(14, 9), sharex=True)
    colors = {"adaptive": "tab:blue", "baseline": "tab:orange"}

    for joint in range(2):
        ax = axs[0, joint]
        ax.plot(times, adaptive_states[:, joint], color=colors["adaptive"], label="adaptive")
        ax.plot(times, baseline_states[:, joint], color=colors["baseline"], ls="--", label="non-adaptive")
        ax.axhline(target[joint], color="k", ls=":", lw=1.0, label="target")
        ax.set_ylabel("Angle [rad]")
        ax.set_title(rf"Joint {joint + 1}: $\theta_{joint + 1}(t)$")
        ax.legend(fontsize=8)

    for joint in range(2):
        ax = axs[1, joint]
        ax.plot(times, adaptive_error[:, joint], color=colors["adaptive"], label="adaptive")
        ax.plot(times, baseline_error[:, joint], color=colors["baseline"], ls="--", label="non-adaptive")
        ax.axhline(0.0, color="k", ls=":", lw=1.0)
        ax.set_ylabel("Error [rad]")
        ax.set_title(rf"Joint {joint + 1}: tracking error")
        ax.legend(fontsize=8)

    for joint in range(2):
        ax = axs[2, joint]
        ax.plot(times, adaptive_torques[:, joint], color=colors["adaptive"], label="adaptive")
        ax.plot(times, baseline_torques[:, joint], color=colors["baseline"], ls="--", label="non-adaptive")
        ax.set_ylabel("Torque [N m]")
        ax.set_xlabel("Time [s]")
        ax.set_title(rf"Joint {joint + 1}: control torque")
        ax.legend(fontsize=8)

    fig.suptitle("Adaptive payload-mass compensation vs non-adaptive PD + gravity", fontsize=13)
    fig.tight_layout(rect=[0, 0, 1, 0.96])
    path = os.path.join(save_dir, "comparison_plots.png")
    fig.savefig(path)
    plt.close(fig)
    print(f"  Saved {path}")


def plot_lyapunov(times, L_vals, dL_vals, save_dir="figures"):
    """Plot the Lyapunov function and its derivative."""
    _ensure_dir(save_dir)

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 6), sharex=True)

    ax1.plot(times, L_vals, color="tab:blue")
    ax1.set_ylabel(r"$L_c(t)$")
    ax1.set_title("Lyapunov function")

    ax2.plot(times, dL_vals, color="tab:red")
    ax2.axhline(0.0, color="k", ls="--", lw=0.8)
    ax2.set_ylabel(r"$\dot{L}_c(t)$")
    ax2.set_xlabel("Time [s]")
    ax2.set_title(r"Ideal derivative: $\dot{L}_c = -k_d ||s||^2$")

    fig.tight_layout()
    path = os.path.join(save_dir, "lyapunov_function.png")
    fig.savefig(path)
    plt.close(fig)
    print(f"  Saved {path}")


def plot_parameter_estimation(times, estimates, true_payload_mass, save_dir="figures"):
    """Plot the online payload-mass estimate next to the true mass."""
    _ensure_dir(save_dir)

    estimates = np.asarray(estimates)
    mass_estimates = estimates[:, 0] if estimates.ndim == 2 else estimates

    fig, ax = plt.subplots(figsize=(10, 5))
    ax.plot(times, mass_estimates, color="tab:blue", label=r"$\hat{m}_p$")
    ax.axhline(true_payload_mass, color="tab:green", ls="--", lw=1.1, label=r"$m_p$ true")

    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Payload mass [kg]")
    ax.set_title("Online estimate of unknown payload mass")
    ax.legend()

    fig.tight_layout()
    path = os.path.join(save_dir, "parameter_estimation.png")
    fig.savefig(path)
    plt.close(fig)
    print(f"  Saved {path}")


def plot_payload_compensation(times, states, estimates, true_payload_mass, robot, save_dir="figures"):
    """Compare estimated and true payload torques along the motion."""
    _ensure_dir(save_dir)

    estimates = np.asarray(estimates)
    mass_estimates = estimates[:, 0] if estimates.ndim == 2 else estimates
    payload_gravity = np.array([robot.payload_gravity_regressor(theta) for theta in states[:, :2]])
    true_tau_payload = true_payload_mass * payload_gravity
    estimated_tau_payload = mass_estimates[:, None] * payload_gravity

    fig, axs = plt.subplots(2, 1, figsize=(10, 7), sharex=True)
    for joint, ax in enumerate(axs):
        ax.plot(times, true_tau_payload[:, joint], color="tab:gray", label="true payload torque")
        ax.plot(times, estimated_tau_payload[:, joint], color="tab:purple", ls="--", label="estimated compensation")
        ax.axhline(0.0, color="k", ls=":", lw=0.8)
        ax.set_ylabel("Torque [N m]")
        ax.set_title(rf"Joint {joint + 1}: $m_p g_i(\theta)$ vs $\hat{{m}}_p g_i(\theta)$")
        ax.legend(fontsize=8)

    axs[-1].set_xlabel("Time [s]")
    fig.tight_layout()
    path = os.path.join(save_dir, "payload_compensation.png")
    fig.savefig(path)
    plt.close(fig)
    print(f"  Saved {path}")


def plot_phase_portrait(
    times,
    adaptive_states,
    theta_d,
    baseline_states=None,
    save_dir="figures",
):
    """Phase portrait (theta_i, dtheta_i) for each joint."""
    _ensure_dir(save_dir)

    fig, axs = plt.subplots(1, 2, figsize=(12, 5))

    for joint, ax in enumerate(axs):
        if baseline_states is not None:
            ax.plot(
                baseline_states[:, joint],
                baseline_states[:, joint + 2],
                color="tab:orange",
                ls="--",
                lw=0.9,
                label="non-adaptive",
            )
        ax.plot(
            adaptive_states[:, joint],
            adaptive_states[:, joint + 2],
            color="tab:blue",
            lw=1.2,
            label="adaptive",
        )
        ax.plot(adaptive_states[0, joint], adaptive_states[0, joint + 2], "go", ms=7, label="start")
        ax.plot(theta_d[joint], 0.0, "r*", ms=12, label="target")
        ax.set_xlabel(rf"$\theta_{joint + 1}$ [rad]")
        ax.set_ylabel(rf"$\dot{{\theta}}_{joint + 1}$ [rad/s]")
        ax.set_title(f"Phase portrait: joint {joint + 1}")
        ax.legend(fontsize=8)

    fig.tight_layout()
    path = os.path.join(save_dir, "phase_portrait.png")
    fig.savefig(path)
    plt.close(fig)
    print(f"  Saved {path}")

def create_animation(
    times,
    states,
    l1,
    l2,
    theta_d,
    estimates=None,
    true_payload_mass=None,
    baseline_states=None,
    save_path="animations/robot_motion.gif",
):
    """Animated adaptive arm with target and optional non-adaptive baseline."""
    os.makedirs(os.path.dirname(save_path), exist_ok=True)

    fig, ax = plt.subplots(figsize=(6, 6))
    lim = (l1 + l2) * 1.3
    ax.set_xlim(-lim, lim)
    ax.set_ylim(-lim, lim)
    ax.set_aspect("equal")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_title("Adaptive payload-mass compensation")

    xd, yd = _joint_positions(theta_d, l1, l2)
    ax.plot(xd, yd, "o--", color="tab:red", lw=1.6, ms=6, alpha=0.55, label="target")

    baseline_line = None
    if baseline_states is not None:
        (baseline_line,) = ax.plot([], [], "o--", color="tab:orange", lw=2, ms=5, alpha=0.65, label="non-adaptive")

    (arm_line,) = ax.plot([], [], "o-", color="tab:blue", lw=3, ms=8, label="adaptive")
    time_text = ax.text(0.02, 0.95, "", transform=ax.transAxes, fontsize=10)
    estimate_text = ax.text(0.02, 0.89, "", transform=ax.transAxes, fontsize=9)
    ax.legend(loc="upper right", fontsize=8)

    skip = max(1, len(times) // 220)
    idx = np.arange(0, len(times), skip)

    def _update(frame):
        i = idx[frame]
        x, y = _joint_positions(states[i, :2], l1, l2)
        arm_line.set_data(x, y)

        artists = [arm_line, time_text, estimate_text]
        if baseline_line is not None:
            xb, yb = _joint_positions(baseline_states[i, :2], l1, l2)
            baseline_line.set_data(xb, yb)
            artists.append(baseline_line)

        time_text.set_text(f"t = {times[i]:.2f} s")
        if estimates is not None:
            mass_estimate = estimates[i, 0] if np.asarray(estimates).ndim == 2 else estimates[i]
            estimate_text.set_text(
                f"hat_m_p = {mass_estimate:.2f} kg"
            )
        elif true_payload_mass is not None:
            estimate_text.set_text(f"m_p = {true_payload_mass:.2f} kg")
        else:
            estimate_text.set_text("")
        return artists

    ani = animation.FuncAnimation(fig, _update, frames=len(idx), blit=True, interval=30)
    writer = animation.PillowWriter(fps=30)
    ani.save(save_path, writer=writer)
    plt.close(fig)
    print(f"  Saved {save_path}")
