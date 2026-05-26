"""Lyapunov-based sampling MPC with wheel torque inputs."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from metrics import LyapunovWeights, heading_error, lyapunov_value
from robot_model import rollout_torque, rollout_torque_batch


@dataclass(frozen=True)
class MPCWeights:
    goal: float = 1.4
    terminal_goal: float = 12.0
    lyapunov: float = 1.0
    lyapunov_increase: float = 18.0
    obstacle: float = 60.0
    collision: float = 16000.0
    control: float = 0.05
    smoothness: float = 0.38
    energy: float = 0.18
    path_length: float = 0.55
    boundary: float = 20000.0


@dataclass
class MPCResult:
    control: np.ndarray
    predicted_trajectory: np.ndarray
    cost: float


class LyapunovMPCController:
    def __init__(
        self,
        config,
        robot_params,
        obstacles,
        target: np.ndarray,
        weights: MPCWeights | None = None,
    ) -> None:
        self.config = config
        self.robot_params = robot_params
        self.obstacles = obstacles
        self.target = np.array(target, dtype=float)
        self.weights = weights or MPCWeights()
        self.lyapunov_weights = LyapunovWeights()
        self.rng = np.random.default_rng(config.random_seed)
        self.previous_solution = np.zeros((config.horizon, 2), dtype=float)
        self.previous_control = np.zeros(2, dtype=float)
        self.samples = 180
        self.iterations = 4

    def optimize(self, state: np.ndarray, time: float) -> MPCResult:
        mean = self._warm_start(state)
        std = np.tile(np.array([0.55, 0.55]), (self.config.horizon, 1))
        best_controls = mean.copy()
        best_cost = float(self.evaluate_cost_batch(state, mean.reshape(1, self.config.horizon, 2), time)[0])

        for _ in range(self.iterations):
            candidates = self.rng.normal(mean, std, size=(self.samples, self.config.horizon, 2))
            candidates[0] = mean
            candidates[1] = self._nominal_torque_guess(state)
            candidates = self._clip(candidates)
            costs = self.evaluate_cost_batch(state, candidates, time)

            elite_count = max(10, self.samples // 8)
            elite_idx = np.argsort(costs)[:elite_count]
            elites = candidates[elite_idx]
            mean = 0.72 * elites.mean(axis=0) + 0.28 * mean
            std = np.maximum(0.55 * elites.std(axis=0), 0.05)

            if costs[elite_idx[0]] < best_cost:
                best_cost = float(costs[elite_idx[0]])
                best_controls = candidates[elite_idx[0]].copy()

        trajectory = rollout_torque(
            state,
            best_controls,
            self.robot_params,
            self.config.dt,
            self.config.map_bounds,
            self.config.robot_radius,
        )
        control = best_controls[0].copy()
        self.previous_control = control.copy()
        self.previous_solution = self._shift(best_controls)
        return MPCResult(control=control, predicted_trajectory=trajectory, cost=best_cost)

    def evaluate_cost_batch(self, state: np.ndarray, controls: np.ndarray, time: float) -> np.ndarray:
        controls = self._clip(controls)
        trajectories = rollout_torque_batch(
            state,
            controls,
            self.robot_params,
            self.config.dt,
            self.config.map_bounds,
            self.config.robot_radius,
        )
        positions = trajectories[:, 1:, :2]

        errors = positions - self.target.reshape(1, 1, 2)
        goal_cost = self.weights.goal * np.sum(errors**2, axis=(1, 2))
        terminal_cost = self.weights.terminal_goal * np.sum(errors[:, -1, :] ** 2, axis=1)

        lyap = self._lyapunov_batch(trajectories)
        lyap_cost = self.weights.lyapunov * np.sum(lyap[:, 1:], axis=1)
        lyap_delta = np.diff(lyap, axis=1)
        lyap_increase_cost = self.weights.lyapunov_increase * np.sum(np.maximum(0.0, lyap_delta) ** 2, axis=1)

        obstacle_cost = self._obstacle_cost_batch(positions, time)
        step_lengths = np.linalg.norm(np.diff(trajectories[:, :, :2], axis=1), axis=2)
        path_length_cost = self.weights.path_length * np.sum(step_lengths, axis=1)
        boundary_cost = self._boundary_cost_batch(positions)
        control_cost = self.weights.control * np.sum(controls**2, axis=(1, 2))
        energy_cost = self.weights.energy * np.sum(controls[:, :, 0] ** 2 + controls[:, :, 1] ** 2, axis=1) * self.config.dt

        previous = np.repeat(self.previous_control.reshape(1, 1, 2), controls.shape[0], axis=0)
        diffs = np.diff(np.concatenate([previous, controls], axis=1), axis=1)
        smoothness_cost = self.weights.smoothness * np.sum(diffs**2, axis=(1, 2))

        return (
            goal_cost
            + terminal_cost
            + lyap_cost
            + lyap_increase_cost
            + obstacle_cost
            + path_length_cost
            + boundary_cost
            + control_cost
            + smoothness_cost
            + energy_cost
        )

    def _lyapunov_batch(self, trajectories: np.ndarray) -> np.ndarray:
        values = np.zeros(trajectories.shape[:2], dtype=float)
        for i in range(trajectories.shape[0]):
            for k in range(trajectories.shape[1]):
                values[i, k] = lyapunov_value(trajectories[i, k], self.target, self.lyapunov_weights)
        return values

    def _obstacle_cost_batch(self, positions: np.ndarray, time: float) -> np.ndarray:
        future_times = time + self.config.dt * np.arange(1, self.config.horizon + 1)
        costs = np.zeros(positions.shape[0], dtype=float)
        influence = 2.35
        for obstacle in self.obstacles:
            centers = obstacle.positions_at(future_times).reshape(1, self.config.horizon, 2)
            clearance = np.linalg.norm(positions - centers, axis=2) - obstacle.radius - self.config.robot_radius
            collision = np.maximum(0.0, -clearance)
            near = np.maximum(0.0, influence - clearance)
            costs += self.weights.collision * np.sum(collision**2, axis=1)
            costs += self.weights.obstacle * np.sum((near / influence) ** 4, axis=1)
        return costs

    def _boundary_cost_batch(self, positions: np.ndarray) -> np.ndarray:
        x_min, x_max, y_min, y_max = self.config.map_bounds
        margin = self.config.robot_radius
        soft = 0.22
        left = np.maximum(0.0, x_min + margin + soft - positions[:, :, 0])
        right = np.maximum(0.0, positions[:, :, 0] - (x_max - margin - soft))
        bottom = np.maximum(0.0, y_min + margin + soft - positions[:, :, 1])
        top = np.maximum(0.0, positions[:, :, 1] - (y_max - margin - soft))
        return self.weights.boundary * np.sum(left**2 + right**2 + bottom**2 + top**2, axis=1)

    def _nominal_torque_guess(self, state: np.ndarray) -> np.ndarray:
        controls = np.zeros((self.config.horizon, 2), dtype=float)
        current = state.copy()
        for k in range(self.config.horizon):
            dist = np.linalg.norm(current[:2] - self.target)
            e_theta = heading_error(current, self.target)
            desired_v = np.clip(0.75 * dist * max(0.0, np.cos(e_theta)), 0.0, self.robot_params.v_max)
            desired_omega = np.clip(2.2 * e_theta, -self.robot_params.omega_max, self.robot_params.omega_max)
            a_v = 2.0 * (desired_v - current[3])
            a_omega = 2.0 * (desired_omega - current[4])
            tau_sum = (a_v + self.robot_params.c_v * current[3]) / self.robot_params.k_v
            tau_diff = (a_omega + self.robot_params.c_omega * current[4]) / self.robot_params.k_omega
            controls[k] = [0.5 * (tau_sum + tau_diff), 0.5 * (tau_sum - tau_diff)]
            controls[k] = np.clip(controls[k], -self.robot_params.tau_max, self.robot_params.tau_max)
            current = rollout_torque(
                current,
                controls[k : k + 1],
                self.robot_params,
                self.config.dt,
                self.config.map_bounds,
                self.config.robot_radius,
            )[-1]
        return controls

    def _warm_start(self, state: np.ndarray) -> np.ndarray:
        if np.allclose(self.previous_solution, 0.0):
            return self._nominal_torque_guess(state)
        return self.previous_solution.copy()

    def _clip(self, controls: np.ndarray) -> np.ndarray:
        return np.clip(controls, -self.robot_params.tau_max, self.robot_params.tau_max)

    @staticmethod
    def _shift(controls: np.ndarray) -> np.ndarray:
        shifted = np.zeros_like(controls)
        shifted[:-1] = controls[1:]
        shifted[-1] = controls[-1]
        return shifted
