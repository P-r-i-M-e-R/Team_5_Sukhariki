"""Artificial Potential Field baseline."""

from __future__ import annotations

import math

import numpy as np

from robot_model import wrap_angle


class ArtificialPotentialFieldController:
    def __init__(self, config, robot_params, obstacles, target: np.ndarray) -> None:
        self.config = config
        self.robot_params = robot_params
        self.obstacles = obstacles
        self.target = target
        self.goal_bias = 1.05
        self.repulsion_radius = 2.2
        self.repulsion_gain = 0.9
        self.boundary_radius = 0.8
        self.boundary_gain = 1.2
        self.command_smoothing = 1.0
        self.omega_gain = 5.2
        self.slowdown_radius = 3.0
        self.previous = np.zeros(2)

    def control(self, state: np.ndarray, time: float) -> np.ndarray:
        attraction = self.target - state[:2]
        attraction_norm = np.linalg.norm(attraction) + 1e-9
        direction = self.goal_bias * attraction / attraction_norm

        repulsion = np.zeros(2)
        for obstacle in self.obstacles:
            center = obstacle.position_at(time)
            vector = state[:2] - center
            distance = np.linalg.norm(vector) + 1e-9
            clearance = distance - obstacle.radius - self.config.robot_radius
            if clearance < self.repulsion_radius:
                scale = (1.0 / max(clearance, 0.08) - 1.0 / self.repulsion_radius) / (distance**2)
                repulsion += self.repulsion_gain * scale * (vector / distance)

        x_min, x_max, y_min, y_max = self.config.map_bounds
        x_low = x_min + self.config.robot_radius
        x_high = x_max - self.config.robot_radius
        y_low = y_min + self.config.robot_radius
        y_high = y_max - self.config.robot_radius
        distances = np.array([state[0] - x_low, x_high - state[0], state[1] - y_low, y_high - state[1]])
        directions = np.array([[1.0, 0.0], [-1.0, 0.0], [0.0, 1.0], [0.0, -1.0]])
        for distance_to_boundary, direction_to_inside in zip(distances, directions):
            if distance_to_boundary < self.boundary_radius:
                scale = 1.0 / max(distance_to_boundary, 0.08) - 1.0 / self.boundary_radius
                repulsion += self.boundary_gain * scale * direction_to_inside

        desired = direction + repulsion
        desired_heading = math.atan2(desired[1], desired[0])
        heading_error = wrap_angle(desired_heading - state[2])
        v = self.robot_params.v_max * max(0.0, math.cos(heading_error)) * min(1.0, attraction_norm / self.slowdown_radius)
        omega = np.clip(self.omega_gain * heading_error, -self.robot_params.omega_max, self.robot_params.omega_max)
        raw = np.array([v, omega])
        self.previous = self.command_smoothing * raw + (1.0 - self.command_smoothing) * self.previous
        return self.previous.copy()
