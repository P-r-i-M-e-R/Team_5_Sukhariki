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
    horizon: int = 50
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


@dataclass(frozen=True)
class Scenario:
    name: str
    title: str
    start_state: np.ndarray
    target: np.ndarray
    obstacles: tuple[CircularObstacle, ...]
    config: SimulationConfig = SimulationConfig()


def default_robot_params() -> RobotParams:
    return RobotParams()


def scenarios() -> list[Scenario]:
    zero = np.zeros(2, dtype=float)
    base_config = SimulationConfig()
    maps = [
        Scenario(
            "map_1",
            "central crossing obstacle corridor",
            np.array([0.0, 0.0, 0.08, 0.0, 0.0], dtype=float),
            np.array([9.0, 6.0], dtype=float),
            (
                CircularObstacle("static_lower_gate", np.array([3.7, 1.20]), 0.65, zero),
                CircularObstacle("static_upper_gate", np.array([5.25, 5.35]), 0.68, zero),
                CircularObstacle("moving_east_crossing", np.array([1.15, 2.65]), 0.35, np.array([0.27, 0.00])),
                CircularObstacle("moving_west_crossing", np.array([8.75, 3.85]), 0.36, np.array([-0.23, 0.00])),
            ),
            base_config,
        ),
        Scenario(
            "map_2",
            "offset bottleneck with horizontal crossing",
            np.array([0.0, 0.0, 0.08, 0.0, 0.0], dtype=float),
            np.array([9.0, 6.0], dtype=float),
            (
                CircularObstacle("static_low_offset", np.array([3.60, 1.25]), 0.60, zero),
                CircularObstacle("static_high_offset", np.array([5.35, 5.30]), 0.62, zero),
                CircularObstacle("moving_mid_horizontal", np.array([1.20, 2.70]), 0.33, np.array([0.26, 0.00])),
                CircularObstacle("moving_goal_horizontal", np.array([8.80, 3.90]), 0.33, np.array([-0.22, 0.00])),
            ),
            base_config,
        ),
        Scenario(
            "map_3",
            "upper corridor with diagonal motion",
            np.array([0.0, 0.0, 0.08, 0.0, 0.0], dtype=float),
            np.array([9.0, 6.0], dtype=float),
            (
                CircularObstacle("static_lower_wall", np.array([3.9, 1.05]), 0.57, zero),
                CircularObstacle("static_upper_wall", np.array([6.1, 5.45]), 0.58, zero),
                CircularObstacle("moving_diagonal_entry", np.array([1.10, 2.20]), 0.32, np.array([0.22, 0.07])),
                CircularObstacle("moving_goal_vertical", np.array([7.90, 3.50]), 0.34, np.array([0.00, 0.04])),
            ),
            base_config,
        ),
        Scenario(
            "map_4",
            "lower corridor with delayed crossing",
            np.array([0.0, 0.0, 0.08, 0.0, 0.0], dtype=float),
            np.array([9.0, 6.0], dtype=float),
            (
                CircularObstacle("static_lower_mid", np.array([3.95, 1.10]), 0.56, zero),
                CircularObstacle("static_upper_mid", np.array([5.10, 5.45]), 0.58, zero),
                CircularObstacle("moving_delayed_mid", np.array([1.00, 2.85]), 0.33, np.array([0.24, 0.00])),
                CircularObstacle("moving_delayed_goal", np.array([8.85, 3.65]), 0.33, np.array([-0.20, 0.02])),
            ),
            base_config,
        ),
        Scenario(
            "map_5",
            "wider two-crossing passage",
            np.array([0.0, 0.0, 0.08, 0.0, 0.0], dtype=float),
            np.array([9.0, 6.0], dtype=float),
            (
                CircularObstacle("static_low_wide", np.array([3.25, 1.05]), 0.52, zero),
                CircularObstacle("static_high_wide", np.array([6.25, 5.25]), 0.56, zero),
                CircularObstacle("moving_wide_entry", np.array([1.20, 2.55]), 0.31, np.array([0.23, 0.03])),
                CircularObstacle("moving_wide_exit", np.array([8.00, 3.55]), 0.32, np.array([0.00, 0.04])),
            ),
            base_config,
        ),
    ]
    for scenario in maps:
        validate_obstacle_layout(list(scenario.obstacles), scenario.config)
    return maps


def get_scenario(name: str = "map_1") -> Scenario:
    for scenario in scenarios():
        if scenario.name == name:
            return scenario
    raise ValueError(f"Unknown scenario: {name}")


def default_scenario() -> Scenario:
    return get_scenario("map_1")


def default_config(scenario: Scenario | None = None) -> SimulationConfig:
    return (scenario or default_scenario()).config


def default_start_state(scenario: Scenario | None = None) -> np.ndarray:
    return np.array((scenario or default_scenario()).start_state, dtype=float)


def default_target(scenario: Scenario | None = None) -> np.ndarray:
    return np.array((scenario or default_scenario()).target, dtype=float)


def default_obstacles(scenario: Scenario | None = None) -> list[CircularObstacle]:
    active = scenario or default_scenario()
    obstacles = list(active.obstacles)
    validate_obstacle_layout(obstacles, active.config)
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
