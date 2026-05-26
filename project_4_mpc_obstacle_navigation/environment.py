"""Environment, robot parameters, and obstacle validation."""

from __future__ import annotations

from dataclasses import dataclass
from itertools import combinations

import numpy as np


@dataclass(frozen=True)
class RobotParams:
    k_v: float = 0.85
    c_v: float = 0.55
    k_omega: float = 1.25
    c_omega: float = 0.65
    tau_max: float = 1.35
    v_max: float = 1.35
    omega_max: float = 1.9


@dataclass(frozen=True)
class SimulationConfig:
    dt: float = 0.15
    max_steps: int = 180
    goal_tolerance: float = 0.35
    robot_radius: float = 0.18
    safety_margin: float = 0.12
    map_bounds: tuple[float, float, float, float] = (-0.25, 9.75, -0.25, 6.55)
    horizon: int = 14
    random_seed: int = 11
    obstacle_validation_margin: float = 0.08


@dataclass(frozen=True)
class CircularObstacle:
    name: str
    initial_center: np.ndarray
    radius: float
    velocity: np.ndarray

    def position_at(self, time: float) -> np.ndarray:
        return self.initial_center + self.velocity * time

    def positions_at(self, times: np.ndarray) -> np.ndarray:
        return self.initial_center.reshape(1, 2) + times.reshape(-1, 1) * self.velocity.reshape(1, 2)

    @property
    def is_dynamic(self) -> bool:
        return bool(np.linalg.norm(self.velocity) > 1e-12)


def default_robot_params() -> RobotParams:
    return RobotParams()


def default_config() -> SimulationConfig:
    return SimulationConfig()


def default_start_state() -> np.ndarray:
    return np.array([0.0, 0.0, 0.08, 0.0, 0.0], dtype=float)


def default_target() -> np.ndarray:
    return np.array([9.0, 6.0], dtype=float)


def default_obstacles() -> list[CircularObstacle]:
    zero = np.zeros(2, dtype=float)
    obstacles = [
        CircularObstacle("static_lower_gate", np.array([3.7, 1.20]), 0.65, zero),
        CircularObstacle("static_upper_gate", np.array([5.25, 5.35]), 0.68, zero),
        CircularObstacle("moving_east_crossing", np.array([1.15, 2.65]), 0.35, np.array([0.27, 0.00])),
        CircularObstacle("moving_west_crossing", np.array([8.75, 3.85]), 0.36, np.array([-0.23, 0.00])),
    ]
    validate_obstacle_layout(obstacles, default_config())
    return obstacles


def validate_obstacle_layout(
    obstacles: list[CircularObstacle],
    config: SimulationConfig,
    samples: int | None = None,
) -> tuple[float, tuple[str, str], float]:
    """Validate obstacle-obstacle disk separation over the whole simulation."""
    times = np.linspace(0.0, config.max_steps * config.dt, samples or config.max_steps + 1)
    min_margin = float("inf")
    min_pair = ("", "")
    min_time = 0.0

    for first, second in combinations(obstacles, 2):
        first_pos = first.positions_at(times)
        second_pos = second.positions_at(times)
        margins = np.linalg.norm(first_pos - second_pos, axis=1) - first.radius - second.radius
        idx = int(np.argmin(margins))
        if float(margins[idx]) < min_margin:
            min_margin = float(margins[idx])
            min_pair = (first.name, second.name)
            min_time = float(times[idx])

    if min_margin < config.obstacle_validation_margin:
        raise ValueError(
            "Invalid obstacle layout: "
            f"{min_pair[0]} and {min_pair[1]} have margin {min_margin:.3f} m "
            f"at t={min_time:.2f} s, below {config.obstacle_validation_margin:.3f} m."
        )
    return min_margin, min_pair, min_time
