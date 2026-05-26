"""Lyapunov-based sampling MPC with wheel torque inputs."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from metrics import LyapunovWeights, heading_error, lyapunov_value
from robot_model import rollout_torque, rollout_torque_batch, step_torque


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
    feasible: bool = True
    used_fallback: bool = False
    used_emergency: bool = False


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
        self.previous_solution: np.ndarray | None = None
        self.previous_terminal_state: np.ndarray | None = None
        self.previous_control = np.zeros(2, dtype=float)
        self.samples = 180
        self.iterations = 4
        self.rho_term = 2.0
        self.shifted_fallback_count = 0
        self.emergency_count = 0
        self.infeasible_iteration_count = 0
        self.last_feasible_count = 0

    def optimize(self, state: np.ndarray, time: float) -> MPCResult:
        shifted_candidate = self.shifted_feasible_candidate()
        mean = self._warm_start(state, shifted_candidate)
        std = np.tile(np.array([0.55, 0.55]), (self.config.horizon, 1))
        best_controls = mean.copy()
        best_cost = float(self.evaluate_cost_batch(state, mean.reshape(1, self.config.horizon, 2), time)[0])
        if not np.isfinite(best_cost):
            best_controls = shifted_candidate if shifted_candidate is not None else self._nominal_torque_guess(state)
            best_cost = float(self.evaluate_cost_batch(state, best_controls.reshape(1, self.config.horizon, 2), time)[0])

        for _ in range(self.iterations):
            candidates = self.rng.normal(mean, std, size=(self.samples, self.config.horizon, 2))
            candidates[0] = mean
            candidates[1] = self._nominal_torque_guess(state)
            if shifted_candidate is not None:
                candidates[2] = shifted_candidate
            candidates = self._clip(candidates)
            costs = self.evaluate_cost_batch(state, candidates, time)
            feasible_mask = np.isfinite(costs)
            self.last_feasible_count = int(np.sum(feasible_mask))
            if not np.any(feasible_mask):
                self.infeasible_iteration_count += 1
                continue

            elite_count = max(10, self.samples // 8)
            finite_idx = np.flatnonzero(feasible_mask)
            elite_idx = finite_idx[np.argsort(costs[finite_idx])[: min(elite_count, len(finite_idx))]]
            elites = candidates[elite_idx]
            mean = 0.72 * elites.mean(axis=0) + 0.28 * mean
            std = np.maximum(0.55 * elites.std(axis=0), 0.05)

            if costs[elite_idx[0]] < best_cost:
                best_cost = float(costs[elite_idx[0]])
                best_controls = candidates[elite_idx[0]].copy()

        used_fallback = False
        used_emergency = False
        if not np.isfinite(best_cost):
            if shifted_candidate is not None:
                fallback_cost = float(
                    self.evaluate_cost_batch(state, shifted_candidate.reshape(1, self.config.horizon, 2), time)[0]
                )
                if np.isfinite(fallback_cost):
                    best_controls = shifted_candidate.copy()
                    best_cost = fallback_cost
                    used_fallback = True
                    self.shifted_fallback_count += 1

        if not np.isfinite(best_cost):
            best_controls = self._emergency_braking_sequence(state)
            best_cost = float(self.evaluate_unconstrained_cost_batch(state, best_controls.reshape(1, self.config.horizon, 2), time)[0])
            used_emergency = True
            self.emergency_count += 1

        trajectory = rollout_torque(
            state,
            best_controls,
            self.robot_params,
            self.config.dt,
            None,
            0.0,
        )
        control = best_controls[0].copy()
        self.previous_control = control.copy()
        self.previous_solution = best_controls.copy()
        self.previous_terminal_state = trajectory[-1].copy()
        return MPCResult(
            control=control,
            predicted_trajectory=trajectory,
            cost=best_cost,
            feasible=not used_emergency,
            used_fallback=used_fallback,
            used_emergency=used_emergency,
        )

    def evaluate_cost_batch(self, state: np.ndarray, controls: np.ndarray, time: float) -> np.ndarray:
        costs = self.evaluate_unconstrained_cost_batch(state, controls, time)
        trajectories = rollout_torque_batch(
            state,
            self._clip(controls),
            self.robot_params,
            self.config.dt,
            None,
            0.0,
        )
        feasible = self.feasible_trajectories(trajectories, self._clip(controls), time)
        return np.where(feasible, costs, np.inf)

    def evaluate_unconstrained_cost_batch(self, state: np.ndarray, controls: np.ndarray, time: float) -> np.ndarray:
        controls = self._clip(controls)
        trajectories = rollout_torque_batch(
            state,
            controls,
            self.robot_params,
            self.config.dt,
            None,
            0.0,
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

    def feasible_trajectories(self, trajectories: np.ndarray, controls: np.ndarray, time: float) -> np.ndarray:
        positions = trajectories[:, 1:, :2]
        x_min, x_max, y_min, y_max = self.config.map_bounds
        margin = self.config.robot_radius
        in_bounds = (
            (positions[:, :, 0] >= x_min + margin - 1e-9)
            & (positions[:, :, 0] <= x_max - margin + 1e-9)
            & (positions[:, :, 1] >= y_min + margin - 1e-9)
            & (positions[:, :, 1] <= y_max - margin + 1e-9)
        )
        boundary_ok = np.all(in_bounds, axis=1)

        input_ok = np.all(np.abs(controls) <= self.robot_params.tau_max + 1e-9, axis=(1, 2))
        velocity_ok = np.all(
            (np.abs(trajectories[:, 1:, 3]) <= self.robot_params.v_max + 1e-9)
            & (np.abs(trajectories[:, 1:, 4]) <= self.robot_params.omega_max + 1e-9),
            axis=1,
        )

        future_times = time + self.config.dt * np.arange(1, self.config.horizon + 1)
        obstacle_ok = np.ones(trajectories.shape[0], dtype=bool)
        for obstacle in self.obstacles:
            centers = obstacle.positions_at(future_times).reshape(1, self.config.horizon, 2)
            clearance = np.linalg.norm(positions - centers, axis=2) - obstacle.radius - self.config.robot_radius
            obstacle_ok &= np.all(clearance >= self.config.safety_margin - 1e-9, axis=1)

        terminal_values = np.array(
            [lyapunov_value(trajectory[-1], self.target, self.lyapunov_weights) for trajectory in trajectories]
        )
        terminal_ok = terminal_values <= self.rho_term + 1e-9
        return boundary_ok & input_ok & velocity_ok & obstacle_ok & terminal_ok

    def in_terminal_set(self, state: np.ndarray) -> bool:
        return lyapunov_value(state, self.target, self.lyapunov_weights) <= self.rho_term + 1e-9

    def terminal_policy(self, state: np.ndarray) -> np.ndarray:
        best_control = self.stabilizing_torque_guess(state)
        best_state = step_torque(state, best_control, self.robot_params, self.config.dt, None, 0.0)
        best_value = lyapunov_value(best_state, self.target, self.lyapunov_weights)
        grid = np.linspace(-self.robot_params.tau_max, self.robot_params.tau_max, 11)
        for tau_r in grid:
            for tau_l in grid:
                control = np.array([tau_r, tau_l], dtype=float)
                next_state = step_torque(state, control, self.robot_params, self.config.dt, None, 0.0)
                value = lyapunov_value(next_state, self.target, self.lyapunov_weights)
                if value < best_value:
                    best_value = value
                    best_control = control
        return best_control

    def stabilizing_torque_guess(self, state: np.ndarray) -> np.ndarray:
        distance = np.linalg.norm(state[:2] - self.target)
        e_theta = heading_error(state, self.target)
        desired_v = np.clip(0.85 * distance * max(0.0, np.cos(e_theta)), 0.0, 0.75 * self.robot_params.v_max)
        desired_omega = np.clip(2.5 * e_theta - 0.8 * state[4], -self.robot_params.omega_max, self.robot_params.omega_max)
        a_v = 2.8 * (desired_v - state[3])
        a_omega = 3.0 * (desired_omega - state[4])
        tau_sum = (a_v + self.robot_params.c_v * state[3]) / self.robot_params.k_v
        tau_diff = (a_omega + self.robot_params.c_omega * state[4]) / self.robot_params.k_omega
        return np.clip(
            np.array([0.5 * (tau_sum + tau_diff), 0.5 * (tau_sum - tau_diff)]),
            -self.robot_params.tau_max,
            self.robot_params.tau_max,
        )

    def terminal_decrease_holds(self, state: np.ndarray) -> bool:
        control = self.terminal_policy(state)
        next_state = step_torque(state, control, self.robot_params, self.config.dt, None, 0.0)
        return lyapunov_value(next_state, self.target, self.lyapunov_weights) <= (
            lyapunov_value(state, self.target, self.lyapunov_weights) + 1e-9
        )

    def verify_terminal_decrease(self, samples: int = 240) -> dict[str, float]:
        rng = np.random.default_rng(self.config.random_seed + 101)
        tested = 0
        satisfied = 0
        max_delta = -np.inf
        attempts = 0
        while tested < samples and attempts < 20 * samples:
            attempts += 1
            radius = 1.0 * np.sqrt(rng.random())
            angle = rng.uniform(-np.pi, np.pi)
            position = self.target + radius * np.array([np.cos(angle), np.sin(angle)])
            heading = np.arctan2(self.target[1] - position[1], self.target[0] - position[0]) + rng.uniform(-0.25, 0.25)
            state = np.array(
                [
                    position[0],
                    position[1],
                    heading,
                    rng.uniform(0.0, 0.2),
                    rng.uniform(-0.15, 0.15),
                ],
                dtype=float,
            )
            if not self.in_terminal_set(state):
                continue
            control = self.terminal_policy(state)
            next_state = step_torque(state, control, self.robot_params, self.config.dt, None, 0.0)
            delta = lyapunov_value(next_state, self.target, self.lyapunov_weights) - lyapunov_value(
                state, self.target, self.lyapunov_weights
            )
            max_delta = max(max_delta, float(delta))
            satisfied += int(delta <= 1e-9)
            tested += 1
        ratio = satisfied / tested if tested else 0.0
        return {"tested": tested, "satisfied": satisfied, "ratio": ratio, "max_delta": float(max_delta)}

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
                None,
                0.0,
            )[-1]
        return controls

    def shifted_feasible_candidate(self) -> np.ndarray | None:
        if self.previous_solution is None or self.previous_terminal_state is None:
            return None
        return self._shift_with_terminal_policy(self.previous_solution, self.previous_terminal_state)

    def _warm_start(self, state: np.ndarray, shifted_candidate: np.ndarray | None) -> np.ndarray:
        if shifted_candidate is not None:
            return shifted_candidate.copy()
        if self.previous_solution is None:
            return self._nominal_torque_guess(state)
        return self.previous_solution.copy()

    def _clip(self, controls: np.ndarray) -> np.ndarray:
        return np.clip(controls, -self.robot_params.tau_max, self.robot_params.tau_max)

    def _shift_with_terminal_policy(self, controls: np.ndarray, terminal_state: np.ndarray) -> np.ndarray:
        shifted = np.zeros_like(controls)
        shifted[:-1] = controls[1:]
        shifted[-1] = self.terminal_policy(terminal_state)
        return shifted

    def _emergency_braking_sequence(self, state: np.ndarray) -> np.ndarray:
        controls = np.zeros((self.config.horizon, 2), dtype=float)
        current = state.copy()
        for k in range(self.config.horizon):
            a_v = -3.0 * current[3]
            a_omega = -3.0 * current[4]
            tau_sum = (a_v + self.robot_params.c_v * current[3]) / self.robot_params.k_v
            tau_diff = (a_omega + self.robot_params.c_omega * current[4]) / self.robot_params.k_omega
            controls[k] = np.clip(
                np.array([0.5 * (tau_sum + tau_diff), 0.5 * (tau_sum - tau_diff)]),
                -self.robot_params.tau_max,
                self.robot_params.tau_max,
            )
            current = step_torque(current, controls[k], self.robot_params, self.config.dt, None, 0.0)
        return controls
