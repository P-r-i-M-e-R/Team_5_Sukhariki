"""Metrics for Project 4."""

from __future__ import annotations

import math
import numpy as np

from robot_model import wrap_angle


def goal_distance(state: np.ndarray, target: np.ndarray) -> float:
    return float(np.linalg.norm(state[:2] - target))


def target_heading(state: np.ndarray, target: np.ndarray) -> float:
    return math.atan2(target[1] - state[1], target[0] - state[0])


def heading_error(state: np.ndarray, target: np.ndarray) -> float:
    return wrap_angle(target_heading(state, target) - state[2])


def min_obstacle_clearance(state: np.ndarray, time: float, obstacles, robot_radius: float) -> float:
    return min(
        float(np.linalg.norm(state[:2] - obstacle.position_at(time)) - obstacle.radius - robot_radius)
        for obstacle in obstacles
    )


def boundary_margins(states: np.ndarray, map_bounds, robot_radius: float) -> np.ndarray:
    x_min, x_max, y_min, y_max = map_bounds
    margins = np.column_stack(
        [
            states[:, 0] - (x_min + robot_radius),
            (x_max - robot_radius) - states[:, 0],
            states[:, 1] - (y_min + robot_radius),
            (y_max - robot_radius) - states[:, 1],
        ]
    )
    return np.min(margins, axis=1)


def boundary_violation_count(states: np.ndarray, map_bounds, robot_radius: float) -> int:
    return int(np.sum(boundary_margins(states, map_bounds, robot_radius) < -1e-9))


def path_length(states: np.ndarray) -> float:
    if len(states) < 2:
        return 0.0
    return float(np.sum(np.linalg.norm(np.diff(states[:, :2], axis=0), axis=1)))


def cumulative_heading_change(states: np.ndarray) -> np.ndarray:
    if len(states) < 2:
        return np.zeros(0)
    headings = np.unwrap(states[:, 2])
    return np.cumsum(np.abs(np.diff(headings)))


def path_smoothness(states: np.ndarray) -> float:
    changes = cumulative_heading_change(states)
    return float(changes[-1]) if len(changes) else 0.0


def cumulative_motion_smoothness(states: np.ndarray) -> np.ndarray:
    if len(states) < 3:
        return np.zeros(0)
    second_difference = states[2:, :2] - 2.0 * states[1:-1, :2] + states[:-2, :2]
    return np.cumsum(np.linalg.norm(second_difference, axis=1))


def motion_smoothness(states: np.ndarray) -> float:
    values = cumulative_motion_smoothness(states)
    return float(values[-1]) if len(values) else 0.0


def heading_aggressiveness(states: np.ndarray) -> float:
    if len(states) < 3:
        return 0.0
    return float(np.sum(np.abs(np.diff(states[:, 4]))))


def progress_efficiency(states: np.ndarray, target: np.ndarray, total_path_length: float) -> float:
    if total_path_length <= 1e-12:
        return 0.0
    straight_distance = float(np.linalg.norm(target - states[0, :2]))
    return straight_distance / total_path_length


def average_progress_rate(initial_distance: float, final_distance: float, duration: float) -> float:
    if duration <= 1e-12:
        return 0.0
    return (initial_distance - final_distance) / duration


def control_effort(controls: np.ndarray, dt: float) -> float:
    if len(controls) == 0:
        return 0.0
    return float(np.sum(controls**2) * dt)


def torque_energy(controls: np.ndarray, dt: float) -> float:
    if len(controls) == 0:
        return 0.0
    return float(np.sum(controls[:, 0] ** 2 + controls[:, 1] ** 2) * dt)
