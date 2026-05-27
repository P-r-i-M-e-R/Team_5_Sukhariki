"""Visualization and animation utilities."""

from __future__ import annotations

import os
import tempfile
from pathlib import Path

os.environ.setdefault("MPLCONFIGDIR", str(Path(tempfile.gettempdir()) / "project_4_mpc_matplotlib"))

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter
from matplotlib.patches import Arc, FancyArrowPatch, FancyBboxPatch
from matplotlib.transforms import Affine2D
import numpy as np

from environment import Scenario, default_config, default_obstacles, default_start_state, default_target


def ensure_dirs() -> tuple[Path, Path]:
    figures = Path("figures")
    animations = Path("animations")
    figures.mkdir(exist_ok=True)
    animations.mkdir(exist_ok=True)
    return figures, animations


def obstacle_path(obstacle, times: np.ndarray) -> np.ndarray:
    return obstacle.positions_at(times)


def draw_map(ax, times: np.ndarray | None = None, scenario: Scenario | None = None) -> None:
    obstacles = default_obstacles(scenario)
    target = default_target(scenario)
    start = default_start_state(scenario)
    config = default_config(scenario)
    x_min, x_max, y_min, y_max = config.map_bounds
    if times is None:
        times = np.linspace(0.0, config.max_steps * config.dt, 100)

    ax.add_patch(
        plt.Rectangle(
            (x_min, y_min),
            x_max - x_min,
            y_max - y_min,
            fill=False,
            edgecolor="#111111",
            linewidth=1.4,
            linestyle="-",
            label="Map boundary",
        )
    )
    ax.scatter(start[0], start[1], s=75, color="tab:green", marker="o", label="Start")
    ax.scatter(target[0], target[1], s=180, color="tab:red", marker="*", label="Goal")

    for obstacle in obstacles:
        positions = obstacle_path(obstacle, times)
        if obstacle.is_dynamic:
            ax.plot(positions[:, 0], positions[:, 1], color="tab:orange", linestyle=":", linewidth=2)
            ax.add_patch(plt.Circle(positions[0], obstacle.radius, color="tab:orange", alpha=0.22))
            ax.add_patch(plt.Circle(positions[-1], obstacle.radius, color="tab:red", alpha=0.16))
        else:
            ax.add_patch(plt.Circle(obstacle.initial_center, obstacle.radius, color="tab:gray", alpha=0.35))

    ax.set_aspect("equal", adjustable="box")
    ax.set_xlim(x_min - 0.25, x_max + 0.25)
    ax.set_ylim(y_min - 0.25, y_max + 0.25)
    ax.set_xlabel("p_x [m]")
    ax.set_ylabel("p_y [m]")
    ax.grid(True, alpha=0.3)


def plot_robot_schematic(filename: str | Path) -> None:
    fig, ax = plt.subplots(figsize=(7.2, 4.8))
    heading = 0.43
    transform = Affine2D().rotate_around(0.0, 0.0, heading) + ax.transData

    body = FancyBboxPatch(
        (-0.78, -0.42),
        1.56,
        0.84,
        boxstyle="round,pad=0.03,rounding_size=0.18",
        facecolor="#d7e9ff",
        edgecolor="#1f77b4",
        linewidth=2.4,
        transform=transform,
    )
    ax.add_patch(body)
    ax.add_patch(plt.Rectangle((-0.52, 0.47), 1.04, 0.18, color="#2f2f2f", zorder=3, transform=transform))
    ax.add_patch(plt.Rectangle((-0.52, -0.65), 1.04, 0.18, color="#2f2f2f", zorder=3, transform=transform))

    def rotate(point: tuple[float, float]) -> np.ndarray:
        x, y = point
        return np.array([x * np.cos(heading) - y * np.sin(heading), x * np.sin(heading) + y * np.cos(heading)])

    nose = rotate((0.92, 0.0))
    heading_tip = rotate((1.55, 0.0))
    theta_ray = rotate((1.18, 0.0))
    left_wheel = rotate((0.0, 0.82))
    right_wheel = rotate((0.0, -0.82))

    ax.plot([0, nose[0]], [0, nose[1]], color="#1f77b4", linewidth=1.5, alpha=0.7)
    ax.scatter(0, 0, color="#1f77b4", s=80, zorder=5)

    ax.arrow(0, 0, heading_tip[0], heading_tip[1], head_width=0.12, head_length=0.20, color="#1f77b4", lw=2.3, length_includes_head=True)
    ax.annotate(r"$v$", heading_tip + np.array([-0.02, 0.16]), color="#1f77b4", fontsize=13, weight="bold")

    ax.plot([0, 1.15], [0, 0], color="#777777", linestyle="--", linewidth=1.1)
    ax.plot([0, theta_ray[0]], [0, theta_ray[1]], color="#777777", linestyle="--", linewidth=1.1)
    ax.add_patch(Arc((0, 0), 1.05, 1.05, theta1=0, theta2=np.degrees(heading), color="#7b3294", linewidth=2.0))
    ax.annotate(r"$\theta$", (0.58, 0.16), color="#7b3294", fontsize=13, weight="bold")

    omega_center = np.array([0.58, 1.05])
    omega_arrow = FancyArrowPatch(
        omega_center + np.array([0.52, 0.12]),
        omega_center + np.array([-0.42, 0.08]),
        connectionstyle="arc3,rad=0.55",
        arrowstyle="-|>",
        mutation_scale=16,
        linewidth=2.2,
        color="#2ca02c",
    )
    ax.add_patch(omega_arrow)
    ax.annotate(r"$\omega$", omega_center + np.array([0.16, 0.46]), color="#2ca02c", fontsize=13, weight="bold")

    left_arrow_start = rotate((-0.34, 0.92))
    left_arrow_delta = rotate((0.52, 0.92)) - left_arrow_start
    right_arrow_start = rotate((-0.34, -0.92))
    right_arrow_delta = rotate((0.52, -0.92)) - right_arrow_start
    ax.arrow(left_arrow_start[0], left_arrow_start[1], left_arrow_delta[0], left_arrow_delta[1], head_width=0.08, head_length=0.10, color="#444444", lw=2.0, length_includes_head=True)
    ax.arrow(right_arrow_start[0], right_arrow_start[1], right_arrow_delta[0], right_arrow_delta[1], head_width=0.08, head_length=0.10, color="#444444", lw=2.0, length_includes_head=True)
    ax.annotate(r"$\tau_L$", left_wheel + np.array([-0.16, 0.22]), fontsize=13, weight="bold")
    ax.annotate(r"$\tau_R$", right_wheel + np.array([-0.18, -0.38]), fontsize=13, weight="bold")

    ax.annotate(r"$(p_x,p_y)$", (0.08, -0.22), fontsize=12)
    ax.set_xlim(-1.55, 2.05)
    ax.set_ylim(-1.35, 1.75)
    ax.set_aspect("equal", adjustable="box")
    ax.axis("off")
    ax.set_title("Differential-drive robot state and wheel-torque inputs")
    fig.tight_layout()
    fig.savefig(filename, dpi=180)
    plt.close(fig)


def plot_comparison_trajectories(results: list, filename: str | Path, scenario: Scenario | None = None) -> None:
    fig, ax = plt.subplots(figsize=(8.3, 6.1))
    draw_map(ax, results[0].times, scenario)
    colors = ["#1f77b4", "#d627a4"]
    for result, color in zip(results, colors):
        ax.plot(result.states[:, 0], result.states[:, 1], linewidth=2.4, color=color, label=result.name)
    ax.set_title("Trajectory comparison")
    ax.legend(loc="best", fontsize=8)
    fig.tight_layout()
    fig.savefig(filename, dpi=180)
    plt.close(fig)


def plot_time_comparison(
    results: list,
    attr: str,
    ylabel: str,
    title: str,
    filename: str | Path,
    scenario: Scenario | None = None,
) -> None:
    fig, ax = plt.subplots(figsize=(8, 4.3))
    for result in results:
        ax.plot(result.times[: len(getattr(result, attr))], getattr(result, attr), linewidth=2.1, label=result.name)
    if "clearance" in attr or "margin" in attr:
        ax.axhline(0.0, color="black", linestyle="--", linewidth=1.0)
    if "goal" in attr:
        ax.axhline(default_config(scenario).goal_tolerance, color="black", linestyle="--", linewidth=1.0)
    ax.set_xlabel("time [s]")
    ax.set_ylabel(ylabel)
    ax.set_title(title)
    ax.grid(True, alpha=0.3)
    ax.legend()
    fig.tight_layout()
    fig.savefig(filename, dpi=180)
    plt.close(fig)


def plot_heading_change(results: list, filename: str | Path) -> None:
    fig, axes = plt.subplots(2, 1, figsize=(8, 6.2), sharex=True)
    for result in results:
        times = result.times[1 : len(result.cumulative_heading_change) + 1]
        axes[0].plot(times, result.cumulative_heading_change, linewidth=2.1, label=result.name)
        smooth_times = result.times[2 : len(result.cumulative_motion_smoothness) + 2]
        axes[1].plot(smooth_times, result.cumulative_motion_smoothness, linewidth=2.1, label=result.name)
    axes[0].set_ylabel("cumulative |heading change| [rad]")
    axes[0].set_title("Motion smoothness comparison")
    axes[0].grid(True, alpha=0.3)
    axes[0].legend()
    axes[1].set_xlabel("time [s]")
    axes[1].set_ylabel("second-difference smoothness [m]")
    axes[1].grid(True, alpha=0.3)
    axes[1].legend()
    fig.tight_layout()
    fig.savefig(filename, dpi=180)
    plt.close(fig)


def plot_control_effort(results: list, filename: str | Path, scenario: Scenario | None = None) -> None:
    fig, ax = plt.subplots(figsize=(8, 4.3))
    dt = default_config(scenario).dt
    for result in results:
        effort = np.cumsum(np.sum(result.controls**2, axis=1)) * dt
        ax.plot(result.times[:-1], effort, linewidth=2.1, label=result.name)
    ax.set_xlabel("time [s]")
    ax.set_ylabel("cumulative effort")
    ax.set_title("Control effort comparison")
    ax.grid(True, alpha=0.3)
    ax.legend()
    fig.tight_layout()
    fig.savefig(filename, dpi=180)
    plt.close(fig)


def plot_energy_weight_study(records: list[dict], filename: str | Path) -> None:
    weights = np.array([record["weight"] for record in records], dtype=float)
    energy = np.array([record["torque_energy"] for record in records], dtype=float)
    time_to_goal = np.array([record["time_to_goal"] for record in records], dtype=float)
    path_length = np.array([record["path_length"] for record in records], dtype=float)
    final_distance = np.array([record["final_distance"] for record in records], dtype=float)

    fig, axes = plt.subplots(2, 2, figsize=(9.2, 6.6))
    axes[0, 0].plot(weights, energy, marker="o", linewidth=2.0)
    axes[0, 0].set_ylabel("motor energy proxy")
    axes[0, 0].set_title("Energy use")

    axes[0, 1].plot(weights, time_to_goal, marker="o", linewidth=2.0)
    axes[0, 1].set_ylabel("time to goal [s]")
    axes[0, 1].set_title("Travel time")

    axes[1, 0].plot(weights, path_length, marker="o", linewidth=2.0)
    axes[1, 0].set_ylabel("path length [m]")
    axes[1, 0].set_xlabel("energy weight")
    axes[1, 0].set_title("Path length")

    axes[1, 1].plot(weights, final_distance, marker="o", linewidth=2.0)
    axes[1, 1].set_ylabel("final distance [m]")
    axes[1, 1].set_xlabel("energy weight")
    axes[1, 1].set_title("Goal accuracy")

    for ax in axes.flat:
        ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(filename, dpi=180)
    plt.close(fig)


def create_comparison_gif(results: list, filename: str | Path, scenario: Scenario | None = None) -> None:
    obstacles = default_obstacles(scenario)
    target = default_target(scenario)
    start = default_start_state(scenario)
    config = default_config(scenario)
    x_min, x_max, y_min, y_max = config.map_bounds
    colors = ["#1f77b4", "#d627a4"]
    labels = [result.name for result in results]
    max_time = max(result.times[-1] for result in results)
    frame_times = np.linspace(0.0, max_time, 105)
    fig, ax = plt.subplots(figsize=(7.6, 5.6))

    def state_index(result, time: float) -> int:
        return min(int(np.searchsorted(result.times, time, side="right") - 1), len(result.states) - 1)

    def update(frame_time: float) -> None:
        ax.clear()
        ax.add_patch(
            plt.Rectangle(
                (x_min, y_min),
                x_max - x_min,
                y_max - y_min,
                fill=False,
                edgecolor="#111111",
                linewidth=1.3,
                label="Map boundary",
            )
        )
        ax.scatter(start[0], start[1], s=70, color="tab:green", marker="o", label="Start")
        ax.scatter(target[0], target[1], s=160, color="tab:red", marker="*", label="Goal")
        for obstacle in obstacles:
            center = obstacle.position_at(frame_time)
            color = "tab:orange" if obstacle.is_dynamic else "tab:gray"
            ax.add_patch(plt.Circle(center, obstacle.radius, color=color, alpha=0.38))
            if obstacle.is_dynamic:
                path_times = np.linspace(0.0, frame_time, max(2, int(frame_time / config.dt)))
                path = obstacle.positions_at(path_times)
                ax.plot(path[:, 0], path[:, 1], color=color, linestyle=":", linewidth=1.6)

        for result, color, label in zip(results, colors, labels):
            idx = state_index(result, frame_time)
            ax.plot(result.states[: idx + 1, 0], result.states[: idx + 1, 1], color=color, linewidth=2.2)
            px, py, theta = result.states[idx, :3]
            ax.scatter(px, py, s=78, color=color, label=label)
            ax.arrow(px, py, 0.26 * np.cos(theta), 0.26 * np.sin(theta), color=color, head_width=0.10)

        ax.set_xlim(x_min - 0.25, x_max + 0.25)
        ax.set_ylim(y_min - 0.25, y_max + 0.25)
        ax.set_aspect("equal", adjustable="box")
        ax.set_xlabel("p_x [m]")
        ax.set_ylabel("p_y [m]")
        ax.set_title(f"MPC vs APF dynamic obstacle comparison, t={frame_time:.1f}s")
        ax.grid(True, alpha=0.25)
        ax.legend(loc="upper left", fontsize=8)

    animation = FuncAnimation(fig, update, frames=frame_times, interval=90)
    animation.save(filename, writer=PillowWriter(fps=12))
    plt.close(fig)
