from __future__ import annotations

from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from matplotlib import animation, patches

from .simulation import ProjectConfig


def prepare_output_dirs(config: ProjectConfig) -> tuple[Path, Path]:
    figures_dir = Path(config.output.figures_dir)
    animations_dir = Path(config.output.animations_dir)
    figures_dir.mkdir(parents=True, exist_ok=True)
    animations_dir.mkdir(parents=True, exist_ok=True)
    return figures_dir, animations_dir


def _setup_style() -> None:
    plt.rcParams.update(
        {
            "figure.dpi": 130,
            "savefig.dpi": 180,
            "axes.grid": True,
            "grid.alpha": 0.25,
            "axes.spines.top": False,
            "axes.spines.right": False,
            "font.size": 10,
            "axes.titlesize": 12,
            "axes.labelsize": 10,
            "legend.fontsize": 9,
        }
    )


def _fault_line(ax: plt.Axes, config: ProjectConfig) -> None:
    _, labels = ax.get_legend_handles_labels()
    ax.axvline(
        config.scenario.fault_time,
        color="0.25",
        linestyle=":",
        linewidth=1.2,
        label="fault onset" if "fault onset" not in labels else None,
    )


def _save_figure(fig: plt.Figure, path: Path) -> None:
    fig.tight_layout()
    fig.savefig(path, bbox_inches="tight")
    plt.close(fig)


def plot_altitude_tracking(
    results: dict[str, dict[str, np.ndarray | str]],
    config: ProjectConfig,
    figures_dir: Path,
) -> None:
    fig, ax = plt.subplots(figsize=(8.8, 4.8))
    nominal = results["nominal"]
    adaptive = results["adaptive"]
    t = nominal["t"]
    ax.plot(t, nominal["z_d"], "k--", linewidth=1.5, label="reference")
    ax.plot(t, nominal["z"], color="#d55e00", linewidth=1.7, label=nominal["label"])
    ax.plot(t, adaptive["z"], color="#0072b2", linewidth=1.7, label=adaptive["label"])
    _fault_line(ax, config)
    ax.set_title("Altitude tracking")
    ax.set_xlabel("time [s]")
    ax.set_ylabel("altitude z [m]")
    ax.legend(loc="best")
    _save_figure(fig, figures_dir / "altitude_tracking.png")


def plot_altitude_error(
    results: dict[str, dict[str, np.ndarray | str]],
    config: ProjectConfig,
    figures_dir: Path,
) -> None:
    fig, ax = plt.subplots(figsize=(8.8, 4.8))
    nominal = results["nominal"]
    adaptive = results["adaptive"]
    t = nominal["t"]
    e_star = -config.physical.gravity / (config.gains.K * config.gains.k1 + 1.0)
    ax.plot(t, nominal["e"], color="#d55e00", linewidth=1.7, label=nominal["label"])
    ax.plot(t, adaptive["e"], color="#0072b2", linewidth=1.7, label=adaptive["label"])
    ax.axhline(0.0, color="0.2", linewidth=1.0)
    ax.axhline(e_star, color="#d55e00", linestyle="--", linewidth=1.0, label=f"nominal e* = {e_star:.2f} m")
    _fault_line(ax, config)
    ax.set_title("Altitude error")
    ax.set_xlabel("time [s]")
    ax.set_ylabel("error e = z - z_d [m]")
    ax.legend(loc="best")
    _save_figure(fig, figures_dir / "altitude_error.png")


def plot_rho_estimation(
    results: dict[str, dict[str, np.ndarray | str]],
    config: ProjectConfig,
    figures_dir: Path,
) -> None:
    fig, ax = plt.subplots(figsize=(8.8, 4.8))
    adaptive = results["adaptive"]
    t = adaptive["t"]
    ax.plot(t, adaptive["rho_true"], "k--", linewidth=1.5, label="true rho")
    ax.plot(t, adaptive["rho_hat"], color="#0072b2", linewidth=1.8, label="adaptive rho_hat")
    ax.axhline(config.scenario.rho_fault, color="0.35", linestyle=":", linewidth=1.0, label="fault level")
    _fault_line(ax, config)
    ax.set_title("Thruster effectiveness estimate")
    ax.set_xlabel("time [s]")
    ax.set_ylabel("rho")
    ax.margins(y=0.08)
    ax.legend(loc="best")
    _save_figure(fig, figures_dir / "rho_estimation.png")


def plot_lyapunov(
    results: dict[str, dict[str, np.ndarray | str]],
    config: ProjectConfig,
    figures_dir: Path,
) -> None:
    fig, ax = plt.subplots(figsize=(8.8, 4.8))
    nominal = results["nominal"]
    adaptive = results["adaptive"]
    t = adaptive["t"]
    ax.plot(
        t,
        nominal["lyapunov"],
        color="#d55e00",
        linewidth=1.8,
        label="V_nom = 0.5e^2 + 0.5delta^2",
    )
    ax.plot(
        t,
        adaptive["lyapunov"],
        color="#0072b2",
        linestyle="--",
        linewidth=2.0,
        label="L_aug adaptive",
    )
    ax.axvline(
        config.scenario.reference_step_time,
        color="0.45",
        linestyle=":",
        linewidth=1.0,
        label="reference step",
    )
    _fault_line(ax, config)
    ax.set_title("Lyapunov functions")
    ax.set_xlabel("time [s]")
    ax.set_ylabel("V, L_aug")
    ax.set_xlim(0.0, config.simulation.final_time)
    ax.set_yscale("symlog", linthresh=1e-4)
    ax.margins(y=0.08)
    ax.legend(loc="best")

    _save_figure(fig, figures_dir / "lyapunov.png")


def plot_lyapunov_derivative(
    results: dict[str, dict[str, np.ndarray | str]],
    config: ProjectConfig,
    figures_dir: Path,
) -> None:
    fig, ax = plt.subplots(figsize=(8.8, 4.8))
    nominal = results["nominal"]
    adaptive = results["adaptive"]
    t = adaptive["t"]

    ax.plot(
        t,
        nominal["lyapunov_dot"],
        color="#d55e00",
        linewidth=1.4,
        label="dV_nom/dt, actual",
    )
    ax.plot(
        t,
        adaptive["lyapunov_dot"],
        color="#0072b2",
        linestyle="--",
        linewidth=1.7,
        label="dL_aug/dt, actual",
    )
    ax.plot(
        t,
        adaptive["lyapunov_design_bound"],
        color="0.05",
        linestyle="-.",
        linewidth=1.2,
        label="-k1 e^2 - K delta^2",
    )
    ax.axhline(0.0, color="0.2", linewidth=1.0)
    ax.axvline(
        config.scenario.reference_step_time,
        color="0.35",
        linestyle=":",
        linewidth=1.1,
        label="reference step",
    )
    _fault_line(ax, config)
    ax.set_title("Lyapunov derivatives")
    ax.set_xlabel("time [s]")
    ax.set_ylabel("dV/dt")
    ax.set_xlim(0.0, config.simulation.final_time)
    ax.set_yscale("symlog", linthresh=1e-2)
    ax.margins(y=0.08)
    ax.legend(loc="best")

    _save_figure(fig, figures_dir / "lyapunov_derivative.png")


def plot_thrust(
    results: dict[str, dict[str, np.ndarray | str]],
    config: ProjectConfig,
    figures_dir: Path,
) -> None:
    fig, ax = plt.subplots(figsize=(8.8, 4.8))
    nominal = results["nominal"]
    adaptive = results["adaptive"]
    t = nominal["t"]
    hover_nominal = config.physical.mass * config.physical.gravity
    hover_fault = hover_nominal / config.scenario.rho_fault
    ax.plot(t, nominal["thrust"], color="#d55e00", linewidth=1.5, label=nominal["label"])
    ax.plot(t, adaptive["thrust"], color="#0072b2", linewidth=1.5, label=adaptive["label"])
    ax.axhline(hover_nominal, color="0.35", linestyle="--", linewidth=1.0, label="hover, rho=1")
    ax.axhline(hover_fault, color="0.35", linestyle="-.", linewidth=1.0, label="hover, faulted")
    _fault_line(ax, config)
    ax.set_title("Commanded thrust")
    ax.set_xlabel("time [s]")
    ax.set_ylabel("T_c [N]")
    ax.legend(loc="best")
    _save_figure(fig, figures_dir / "thrust.png")


def plot_phase_portrait(
    results: dict[str, dict[str, np.ndarray | str]],
    config: ProjectConfig,
    figures_dir: Path,
) -> None:
    fig, ax = plt.subplots(figsize=(6.6, 5.8))
    nominal = results["nominal"]
    adaptive = results["adaptive"]
    nominal_mask = nominal["t"] >= config.scenario.fault_time
    adaptive_mask = adaptive["t"] >= config.scenario.fault_time
    ax.plot(
        nominal["e"][nominal_mask],
        nominal["delta"][nominal_mask],
        color="#d55e00",
        linewidth=1.6,
        label=nominal["label"],
    )
    ax.plot(
        adaptive["e"][adaptive_mask],
        adaptive["delta"][adaptive_mask],
        color="#0072b2",
        linewidth=1.6,
        label=adaptive["label"],
    )
    ax.scatter([0.0], [0.0], color="0.1", s=28, label="origin")
    e_star = -config.physical.gravity / (config.gains.K * config.gains.k1 + 1.0)
    ax.scatter([e_star], [config.gains.k1 * e_star], color="#d55e00", s=32, label="nominal fault equilibrium")
    ax.set_title("Phase portrait")
    ax.set_xlabel("altitude error e [m]")
    ax.set_ylabel("backstepping error delta [m/s]")
    ax.legend(loc="best")
    ax.axis("equal")
    _save_figure(fig, figures_dir / "phase_portrait.png")


def create_rocket_animation(
    results: dict[str, dict[str, np.ndarray | str]],
    config: ProjectConfig,
    animations_dir: Path,
) -> None:
    nominal = results["nominal"]
    adaptive = results["adaptive"]
    dt = float(adaptive["t"][1] - adaptive["t"][0])
    stride = max(1, int(round(config.output.animation_frame_dt / dt)))
    frame_idx = np.arange(0, len(adaptive["t"]), stride)
    if frame_idx[-1] != len(adaptive["t"]) - 1:
        frame_idx = np.append(frame_idx, len(adaptive["t"]) - 1)

    all_z = np.concatenate([nominal["z"], adaptive["z"], adaptive["z_d"]])
    z_min = min(-1.0, float(np.min(all_z)) - 1.0)
    z_max = max(config.scenario.reference_target + 3.5, float(np.max(all_z)) + 2.0)

    fig, ax = plt.subplots(figsize=(5.4, 7.2))
    ax.set_xlim(-1.8, 1.8)
    ax.set_ylim(z_min, z_max)
    ax.set_xlabel("controller")
    ax.set_ylabel("altitude z [m]")
    ax.set_xticks([-0.75, 0.75])
    ax.set_xticklabels(["nominal", "adaptive"])
    ax.grid(True, axis="y", alpha=0.25)

    ref_line = ax.axhline(0.0, color="0.1", linestyle="--", linewidth=1.2, label="reference")
    status_text = ax.text(
        0.02,
        0.98,
        "",
        transform=ax.transAxes,
        va="top",
        bbox={"boxstyle": "round,pad=0.25", "facecolor": "white", "edgecolor": "0.8", "alpha": 0.9},
        zorder=10,
    )

    def make_rocket(x_center: float, color: str) -> dict[str, patches.Patch]:
        nose = patches.Polygon(
            [[x_center - 0.18, 0.28], [x_center + 0.18, 0.28], [x_center, 0.64]],
            facecolor="#f4f4f0",
            edgecolor="0.15",
            linewidth=1.0,
            zorder=5,
        )
        body = patches.FancyBboxPatch(
            (x_center - 0.18, -0.32),
            0.36,
            0.62,
            boxstyle="round,pad=0.012,rounding_size=0.08",
            facecolor=color,
            edgecolor="0.15",
            linewidth=1.0,
            zorder=4,
        )
        window = patches.Circle(
            (x_center, 0.05),
            radius=0.075,
            facecolor="#b9e7ff",
            edgecolor="white",
            linewidth=1.2,
            zorder=6,
        )
        left_fin = patches.Polygon(
            [[x_center - 0.17, -0.18], [x_center - 0.34, -0.42], [x_center - 0.16, -0.39]],
            facecolor="#5f6368",
            edgecolor="0.15",
            linewidth=0.9,
            zorder=3,
        )
        right_fin = patches.Polygon(
            [[x_center + 0.17, -0.18], [x_center + 0.34, -0.42], [x_center + 0.16, -0.39]],
            facecolor="#5f6368",
            edgecolor="0.15",
            linewidth=0.9,
            zorder=3,
        )
        engine = patches.Polygon(
            [[x_center - 0.11, -0.33], [x_center + 0.11, -0.33], [x_center + 0.07, -0.44], [x_center - 0.07, -0.44]],
            facecolor="#2f3337",
            edgecolor="0.15",
            linewidth=0.8,
            zorder=4,
        )
        flame_outer = patches.Polygon(
            [[x_center - 0.13, -0.44], [x_center + 0.13, -0.44], [x_center, -0.95]],
            facecolor="#f28e2b",
            edgecolor="none",
            alpha=0.75,
            zorder=2,
        )
        flame_inner = patches.Polygon(
            [[x_center - 0.07, -0.44], [x_center + 0.07, -0.44], [x_center, -0.75]],
            facecolor="#ffdf5d",
            edgecolor="none",
            alpha=0.9,
            zorder=3,
        )
        parts = {
            "nose": nose,
            "body": body,
            "window": window,
            "left_fin": left_fin,
            "right_fin": right_fin,
            "engine": engine,
            "flame_outer": flame_outer,
            "flame_inner": flame_inner,
        }
        for part in parts.values():
            ax.add_patch(part)
        return parts

    nominal_rocket = make_rocket(-0.75, "#d55e00")
    adaptive_rocket = make_rocket(0.75, "#0072b2")
    ax.legend(loc="lower right")

    def move_rocket(
        rocket: dict[str, patches.Patch],
        x_center: float,
        z: float,
        thrust: float,
        frame_number: int,
    ) -> None:
        rocket["nose"].set_xy(
            [[x_center - 0.18, z + 0.28], [x_center + 0.18, z + 0.28], [x_center, z + 0.64]]
        )
        rocket["body"].set_x(x_center - 0.18)
        rocket["body"].set_y(z - 0.32)
        rocket["window"].center = (x_center, z + 0.05)
        rocket["left_fin"].set_xy(
            [[x_center - 0.17, z - 0.18], [x_center - 0.34, z - 0.42], [x_center - 0.16, z - 0.39]]
        )
        rocket["right_fin"].set_xy(
            [[x_center + 0.17, z - 0.18], [x_center + 0.34, z - 0.42], [x_center + 0.16, z - 0.39]]
        )
        rocket["engine"].set_xy(
            [
                [x_center - 0.11, z - 0.33],
                [x_center + 0.11, z - 0.33],
                [x_center + 0.07, z - 0.44],
                [x_center - 0.07, z - 0.44],
            ]
        )
        thrust_level = min(max(thrust, 0.0) / 60.0, 1.0)
        flicker = 0.05 * np.sin(0.7 * frame_number + 6.0 * x_center)
        outer_length = 0.20 + 0.72 * thrust_level + flicker
        inner_length = 0.12 + 0.45 * thrust_level - 0.5 * flicker
        rocket["flame_outer"].set_xy(
            [
                [x_center - 0.13, z - 0.44],
                [x_center + 0.13, z - 0.44],
                [x_center, z - 0.44 - outer_length],
            ]
        )
        rocket["flame_inner"].set_xy(
            [
                [x_center - 0.07, z - 0.44],
                [x_center + 0.07, z - 0.44],
                [x_center, z - 0.44 - inner_length],
            ]
        )
        rocket["flame_outer"].set_alpha(0.15 + 0.65 * thrust_level)
        rocket["flame_inner"].set_alpha(0.20 + 0.70 * thrust_level)

    def update(frame_number: int):
        idx = frame_idx[frame_number]
        z_nom = float(nominal["z"][idx])
        z_ad = float(adaptive["z"][idx])
        z_ref = float(adaptive["z_d"][idx])
        t_i = float(adaptive["t"][idx])
        move_rocket(nominal_rocket, -0.75, z_nom, float(nominal["thrust"][idx]), frame_number)
        move_rocket(adaptive_rocket, 0.75, z_ad, float(adaptive["thrust"][idx]), frame_number)
        ref_line.set_ydata([z_ref, z_ref])
        status_text.set_text(
            f"time: {t_i:4.1f} s\n"
            f"rho true: {float(adaptive['rho_true'][idx]):.2f}\n"
            f"rho_hat adaptive: {float(adaptive['rho_hat'][idx]):.2f}"
        )
        return (
            *nominal_rocket.values(),
            *adaptive_rocket.values(),
            ref_line,
            status_text,
        )

    anim = animation.FuncAnimation(
        fig,
        update,
        frames=len(frame_idx),
        interval=1000 / config.output.animation_fps,
        blit=True,
    )
    writer = animation.PillowWriter(fps=config.output.animation_fps)
    anim.save(animations_dir / "rocket_flight.gif", writer=writer)
    plt.close(fig)


def create_all_outputs(
    results: dict[str, dict[str, np.ndarray | str]],
    config: ProjectConfig,
) -> tuple[Path, Path]:
    _setup_style()
    figures_dir, animations_dir = prepare_output_dirs(config)
    plot_altitude_tracking(results, config, figures_dir)
    plot_altitude_error(results, config, figures_dir)
    plot_rho_estimation(results, config, figures_dir)
    plot_lyapunov(results, config, figures_dir)
    plot_lyapunov_derivative(results, config, figures_dir)
    plot_thrust(results, config, figures_dir)
    plot_phase_portrait(results, config, figures_dir)
    create_rocket_animation(results, config, animations_dir)
    return figures_dir, animations_dir
