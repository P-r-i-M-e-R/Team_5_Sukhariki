"""Simulation experiments for Project 4."""

from __future__ import annotations

import time
from dataclasses import dataclass

import numpy as np

from environment import default_config, default_obstacles, default_robot_params, default_start_state, default_target
from metrics import (
    average_progress_rate,
    boundary_violation_count,
    control_effort,
    cumulative_heading_change,
    goal_distance,
    heading_aggressiveness,
    lyapunov_value,
    min_obstacle_clearance,
    path_length,
    path_smoothness,
    progress_efficiency,
    torque_energy,
)
from mpc_controller import LyapunovMPCController
from potential_field_controller import ArtificialPotentialFieldController
from robot_model import step_torque, step_velocity


@dataclass
class SimulationResult:
    name: str
    states: np.ndarray
    controls: np.ndarray
    times: np.ndarray
    costs: np.ndarray
    predicted: list[np.ndarray]
    goal_distances: np.ndarray
    obstacle_clearances: np.ndarray
    safety_margins: np.ndarray
    cumulative_heading_change: np.ndarray
    lyapunov_values: np.ndarray
    lyapunov_delta: np.ndarray
    final_distance: float
    min_clearance: float
    min_safety_margin: float
    avg_safety_margin: float
    boundary_violations: int
    path_length: float
    path_smoothness: float
    heading_aggressiveness: float
    near_obstacle_events: int
    progress_efficiency: float
    average_progress_rate: float
    control_effort: float
    torque_energy: float
    time_to_goal: float
    collision: bool
    reached: bool
    avg_compute_time: float


def run_mpc() -> SimulationResult:
    config = default_config()
    robot_params = default_robot_params()
    obstacles = default_obstacles()
    target = default_target()
    controller = LyapunovMPCController(config, robot_params, obstacles, target)
    state = default_start_state()
    states = [state.copy()]
    controls = []
    costs = []
    predicted = []
    compute_times = []

    for step in range(config.max_steps):
        current_time = step * config.dt
        tic = time.perf_counter()
        result = controller.optimize(state, current_time)
        compute_times.append(time.perf_counter() - tic)
        state = step_torque(state, result.control, robot_params, config.dt, config.map_bounds, config.robot_radius)
        states.append(state.copy())
        controls.append(result.control)
        costs.append(result.cost)
        predicted.append(result.predicted_trajectory)
        if goal_distance(state, target) <= config.goal_tolerance:
            break
    return build_result("Lyapunov MPC", states, controls, costs, predicted, compute_times, torque_mode=True)


def run_apf() -> SimulationResult:
    config = default_config()
    robot_params = default_robot_params()
    obstacles = default_obstacles()
    target = default_target()
    controller = ArtificialPotentialFieldController(config, robot_params, obstacles, target)
    return run_velocity_controller("APF baseline", controller, torque_mode=False)


def run_velocity_controller(name: str, controller, torque_mode: bool) -> SimulationResult:
    config = default_config()
    robot_params = default_robot_params()
    target = default_target()
    state = default_start_state()
    states = [state.copy()]
    controls = []
    costs = []
    compute_times = []

    for step in range(config.max_steps):
        current_time = step * config.dt
        tic = time.perf_counter()
        command = controller.control(state, current_time)
        compute_times.append(time.perf_counter() - tic)
        state = step_velocity(state, command, robot_params, config.dt, config.map_bounds, config.robot_radius)
        states.append(state.copy())
        controls.append(command)
        costs.append(goal_distance(state, target) ** 2)
        if goal_distance(state, target) <= config.goal_tolerance:
            break
    return build_result(name, states, controls, costs, [], compute_times, torque_mode=torque_mode)


def build_result(
    name: str,
    states,
    controls,
    costs,
    predicted,
    compute_times,
    torque_mode: bool,
) -> SimulationResult:
    config = default_config()
    target = default_target()
    obstacles = default_obstacles()
    states = np.array(states, dtype=float)
    controls = np.array(controls, dtype=float)
    costs = np.array(costs, dtype=float)
    times = config.dt * np.arange(len(states))
    distances = np.array([goal_distance(state, target) for state in states])
    clearances = np.array(
        [min_obstacle_clearance(state, t, obstacles, config.robot_radius) for state, t in zip(states, times)]
    )
    safety_margins = clearances - config.safety_margin
    lyapunov = np.array([lyapunov_value(state, target) for state in states])
    delta = np.diff(lyapunov)
    total_path_length = path_length(states)
    initial_distance = float(distances[0])
    final_distance = float(distances[-1])
    duration = float(times[-1])
    reached = bool(distances[-1] <= config.goal_tolerance)
    return SimulationResult(
        name=name,
        states=states,
        controls=controls,
        times=times,
        costs=costs,
        predicted=predicted,
        goal_distances=distances,
        obstacle_clearances=clearances,
        safety_margins=safety_margins,
        cumulative_heading_change=cumulative_heading_change(states),
        lyapunov_values=lyapunov,
        lyapunov_delta=delta,
        final_distance=final_distance,
        min_clearance=float(np.min(clearances)),
        min_safety_margin=float(np.min(safety_margins)),
        avg_safety_margin=float(np.mean(safety_margins)),
        boundary_violations=boundary_violation_count(states, config.map_bounds, config.robot_radius),
        path_length=total_path_length,
        path_smoothness=path_smoothness(states),
        heading_aggressiveness=heading_aggressiveness(states),
        near_obstacle_events=int(np.sum(clearances < 0.45)),
        progress_efficiency=progress_efficiency(states, target, total_path_length),
        average_progress_rate=average_progress_rate(initial_distance, final_distance, duration),
        control_effort=control_effort(controls, config.dt),
        torque_energy=torque_energy(controls, config.dt) if torque_mode else 0.0,
        time_to_goal=float(times[-1]) if reached else float("nan"),
        collision=bool(np.min(clearances) < 0.0),
        reached=reached,
        avg_compute_time=float(np.mean(compute_times)) if compute_times else 0.0,
    )


def run_all() -> dict[str, SimulationResult]:
    mpc = run_mpc()
    apf = run_apf()
    return {"mpc": mpc, "apf": apf}
