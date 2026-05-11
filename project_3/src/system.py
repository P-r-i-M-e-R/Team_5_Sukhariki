from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class PhysicalParams:
    mass: float
    gravity: float


@dataclass(frozen=True)
class ScenarioParams:
    reference_step_time: float
    reference_initial: float
    reference_target: float
    fault_time: float
    rho_nominal: float
    rho_fault: float


def reference_trajectory(t: float, scenario: ScenarioParams) -> tuple[float, float, float]:
    """Piecewise-constant altitude command from the project statement."""
    z_d = (
        scenario.reference_target
        if t >= scenario.reference_step_time
        else scenario.reference_initial
    )
    return z_d, 0.0, 0.0


def true_rho(t: float, scenario: ScenarioParams) -> float:
    """Thruster effectiveness used by the plant, unknown to controllers."""
    return scenario.rho_fault if t >= scenario.fault_time else scenario.rho_nominal


def rocket_derivative(
    t: float,
    state: np.ndarray,
    thrust_command: float,
    physical: PhysicalParams,
    scenario: ScenarioParams,
) -> np.ndarray:
    z, z_dot = state
    rho = true_rho(t, scenario)
    z_ddot = rho * thrust_command / physical.mass - physical.gravity
    return np.array([z_dot, z_ddot], dtype=float)
