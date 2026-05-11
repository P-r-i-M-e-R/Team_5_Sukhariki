from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np
import yaml

from .controller import (
    AdaptiveFTCController,
    ControllerGains,
    ControllerLimits,
    NominalBacksteppingController,
    clip_rho_hat,
)
from .system import PhysicalParams, ScenarioParams, reference_trajectory, true_rho


@dataclass(frozen=True)
class SimulationParams:
    final_time: float
    dt: float
    initial_altitude: float
    initial_velocity: float
    initial_rho_hat: float
    adaptation_start_time: float


@dataclass(frozen=True)
class OutputParams:
    figures_dir: str
    animations_dir: str
    animation_frame_dt: float
    animation_fps: int


@dataclass(frozen=True)
class ProjectConfig:
    physical: PhysicalParams
    scenario: ScenarioParams
    simulation: SimulationParams
    gains: ControllerGains
    limits: ControllerLimits
    output: OutputParams


def load_config(path: str | Path) -> ProjectConfig:
    with Path(path).open("r", encoding="utf-8") as handle:
        raw: dict[str, Any] = yaml.safe_load(handle)

    return ProjectConfig(
        physical=PhysicalParams(
            mass=float(raw["physical"]["mass"]),
            gravity=float(raw["physical"]["gravity"]),
        ),
        scenario=ScenarioParams(
            reference_step_time=float(raw["scenario"]["reference_step_time"]),
            reference_initial=float(raw["scenario"]["reference_initial"]),
            reference_target=float(raw["scenario"]["reference_target"]),
            fault_time=float(raw["scenario"]["fault_time"]),
            rho_nominal=float(raw["scenario"]["rho_nominal"]),
            rho_fault=float(raw["scenario"]["rho_fault"]),
        ),
        simulation=SimulationParams(
            final_time=float(raw["simulation"]["final_time"]),
            dt=float(raw["simulation"]["dt"]),
            initial_altitude=float(raw["simulation"]["initial_altitude"]),
            initial_velocity=float(raw["simulation"]["initial_velocity"]),
            initial_rho_hat=float(raw["simulation"]["initial_rho_hat"]),
            adaptation_start_time=float(raw["simulation"]["adaptation_start_time"]),
        ),
        gains=ControllerGains(
            k1=float(raw["gains"]["k1"]),
            K=float(raw["gains"]["K"]),
            gamma=float(raw["gains"]["gamma"]),
        ),
        limits=ControllerLimits(
            rho_hat_min=float(raw["limits"]["rho_hat_min"]),
            rho_hat_max=float(raw["limits"]["rho_hat_max"]),
            thrust_min=float(raw["limits"]["thrust_min"]),
            thrust_max=(
                None
                if raw["limits"]["thrust_max"] is None
                else float(raw["limits"]["thrust_max"])
            ),
        ),
        output=OutputParams(
            figures_dir=str(raw["output"]["figures_dir"]),
            animations_dir=str(raw["output"]["animations_dir"]),
            animation_frame_dt=float(raw["output"]["animation_frame_dt"]),
            animation_fps=int(raw["output"]["animation_fps"]),
        ),
    )


def _controller_by_name(name: str):
    if name == "nominal":
        return NominalBacksteppingController()
    if name == "adaptive":
        return AdaptiveFTCController()
    raise ValueError(f"Unknown controller '{name}'")


def _lyapunov_value(
    controller_name: str,
    e: float,
    delta: float,
    rho_hat: float,
    rho: float,
    gains: ControllerGains,
) -> float:
    base = 0.5 * e**2 + 0.5 * delta**2
    if controller_name == "adaptive":
        return base + 0.5 * (rho_hat - rho) ** 2 / gains.gamma
    return base


def _lyapunov_derivative(
    controller_name: str,
    e: float,
    v_e: float,
    delta: float,
    rho: float,
    rho_hat: float,
    rho_hat_dot: float,
    thrust: float,
    z_d_ddot: float,
    physical: PhysicalParams,
    gains: ControllerGains,
) -> float:
    e_dot = v_e
    v_e_dot = rho * thrust / physical.mass - physical.gravity - z_d_ddot
    delta_dot = v_e_dot + gains.k1 * e_dot
    derivative = e * e_dot + delta * delta_dot
    if controller_name == "adaptive":
        derivative += (rho_hat - rho) * rho_hat_dot / gains.gamma
    return float(derivative)


def _lyapunov_design_bound(e: float, delta: float, gains: ControllerGains) -> float:
    return float(-gains.k1 * e**2 - gains.K * delta**2)


def simulate_controller(name: str, config: ProjectConfig) -> dict[str, np.ndarray | str]:
    controller = _controller_by_name(name)
    sim = config.simulation
    physical = config.physical
    scenario = config.scenario
    gains = config.gains
    limits = config.limits

    steps = int(round(sim.final_time / sim.dt))
    t = np.linspace(0.0, sim.final_time, steps + 1)
    y = np.array(
        [sim.initial_altitude, sim.initial_velocity, sim.initial_rho_hat],
        dtype=float,
    )

    data: dict[str, np.ndarray | str] = {
        "name": name,
        "label": controller.label,
        "t": t,
        "z": np.zeros_like(t),
        "z_dot": np.zeros_like(t),
        "z_d": np.zeros_like(t),
        "rho_true": np.zeros_like(t),
        "rho_hat": np.zeros_like(t),
        "thrust": np.zeros_like(t),
        "nominal_thrust": np.zeros_like(t),
        "rho_hat_dot": np.zeros_like(t),
        "e": np.zeros_like(t),
        "v_e": np.zeros_like(t),
        "delta": np.zeros_like(t),
        "lyapunov": np.zeros_like(t),
        "lyapunov_dot": np.zeros_like(t),
        "lyapunov_design_bound": np.zeros_like(t),
    }

    def evaluate(t_i: float, y_i: np.ndarray):
        state = y_i[:2]
        rho_hat = 1.0 if name == "nominal" else clip_rho_hat(y_i[2], limits)
        ref = reference_trajectory(t_i, scenario)
        output = controller.evaluate(state, ref, rho_hat, physical, gains, limits)
        return ref, rho_hat, output

    def ode(t_i: float, y_i: np.ndarray) -> np.ndarray:
        _, rho_hat, output = evaluate(t_i, y_i)
        rho = true_rho(t_i, scenario)
        z_dot = y_i[1]
        z_ddot = rho * output.thrust_command / physical.mass - physical.gravity
        adaptation_active = t_i >= sim.adaptation_start_time
        rho_hat_dot = (
            output.rho_hat_dot
            if name == "adaptive" and adaptation_active
            else 0.0
        )
        return np.array([z_dot, z_ddot, rho_hat_dot], dtype=float)

    for idx, t_i in enumerate(t):
        ref, rho_hat, output = evaluate(float(t_i), y)
        rho = true_rho(float(t_i), scenario)
        adaptation_active = float(t_i) >= sim.adaptation_start_time
        rho_hat_dot = (
            output.rho_hat_dot
            if name == "adaptive" and adaptation_active
            else 0.0
        )

        data["z"][idx] = y[0]  # type: ignore[index]
        data["z_dot"][idx] = y[1]  # type: ignore[index]
        data["z_d"][idx] = ref[0]  # type: ignore[index]
        data["rho_true"][idx] = rho  # type: ignore[index]
        data["rho_hat"][idx] = rho_hat  # type: ignore[index]
        data["thrust"][idx] = output.thrust_command  # type: ignore[index]
        data["nominal_thrust"][idx] = output.nominal_thrust  # type: ignore[index]
        data["rho_hat_dot"][idx] = rho_hat_dot  # type: ignore[index]
        data["e"][idx] = output.e  # type: ignore[index]
        data["v_e"][idx] = output.v_e  # type: ignore[index]
        data["delta"][idx] = output.delta  # type: ignore[index]
        data["lyapunov"][idx] = _lyapunov_value(  # type: ignore[index]
            name, output.e, output.delta, rho_hat, rho, gains
        )
        data["lyapunov_dot"][idx] = _lyapunov_derivative(  # type: ignore[index]
            name,
            output.e,
            output.v_e,
            output.delta,
            rho,
            rho_hat,
            rho_hat_dot,
            output.thrust_command,
            ref[2],
            physical,
            gains,
        )
        data["lyapunov_design_bound"][idx] = _lyapunov_design_bound(  # type: ignore[index]
            output.e, output.delta, gains
        )

        if idx == steps:
            break

        dt = sim.dt
        k1 = ode(float(t_i), y)
        k2 = ode(float(t_i + 0.5 * dt), y + 0.5 * dt * k1)
        k3 = ode(float(t_i + 0.5 * dt), y + 0.5 * dt * k2)
        k4 = ode(float(t_i + dt), y + dt * k3)
        y = y + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)

        if name == "nominal":
            y[2] = 1.0
        else:
            y[2] = clip_rho_hat(y[2], limits)

    return data


def run_all(config: ProjectConfig) -> dict[str, dict[str, np.ndarray | str]]:
    return {
        "nominal": simulate_controller("nominal", config),
        "adaptive": simulate_controller("adaptive", config),
    }
