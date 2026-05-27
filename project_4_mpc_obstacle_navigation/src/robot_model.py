"""Differential-drive robot model with wheel torque inputs."""

from __future__ import annotations

import math

import numpy as np


def wrap_angle(angle: float) -> float:
    """Wrap an angle to [-pi, pi]."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def clip_position(position: np.ndarray, map_bounds=None, margin: float = 0.0) -> np.ndarray:
    """Clip a position so a disk with the given margin remains inside map bounds."""
    if map_bounds is None:
        return position
    x_min, x_max, y_min, y_max = map_bounds
    return np.array(
        [
            np.clip(position[0], x_min + margin, x_max - margin),
            np.clip(position[1], y_min + margin, y_max - margin),
        ],
        dtype=float,
    )


def step_torque(
    state: np.ndarray,
    control: np.ndarray,
    params,
    dt: float,
    map_bounds=None,
    margin: float = 0.0,
) -> np.ndarray:
    """Euler step for the torque-driven differential-drive model."""
    px, py, theta, v, omega = state
    tau_r, tau_l = np.clip(control, -params.tau_max, params.tau_max)

    v_dot = params.k_v * (tau_r + tau_l) - params.c_v * v
    omega_dot = params.k_omega * (tau_r - tau_l) - params.c_omega * omega

    next_position = clip_position(
        np.array([px + v * math.cos(theta) * dt, py + v * math.sin(theta) * dt]),
        map_bounds,
        margin,
    )
    next_state = np.array(
        [
            next_position[0],
            next_position[1],
            wrap_angle(theta + omega * dt),
            np.clip(v + v_dot * dt, 0.0, params.v_max),
            np.clip(omega + omega_dot * dt, -params.omega_max, params.omega_max),
        ],
        dtype=float,
    )
    return next_state


def step_velocity(
    state: np.ndarray,
    command: np.ndarray,
    params,
    dt: float,
    map_bounds=None,
    margin: float = 0.0,
) -> np.ndarray:
    """Euler step for baselines that command linear and angular velocity directly."""
    px, py, theta, _, _ = state
    v, omega = command
    v = float(np.clip(v, 0.0, params.v_max))
    omega = float(np.clip(omega, -params.omega_max, params.omega_max))
    next_position = clip_position(
        np.array([px + v * math.cos(theta) * dt, py + v * math.sin(theta) * dt]),
        map_bounds,
        margin,
    )
    return np.array(
        [
            next_position[0],
            next_position[1],
            wrap_angle(theta + omega * dt),
            v,
            omega,
        ],
        dtype=float,
    )


def rollout_torque(
    state: np.ndarray,
    controls: np.ndarray,
    params,
    dt: float,
    map_bounds=None,
    margin: float = 0.0,
) -> np.ndarray:
    """Roll out a torque sequence and return N + 1 states."""
    trajectory = np.zeros((len(controls) + 1, 5), dtype=float)
    trajectory[0] = state
    current = np.array(state, dtype=float)
    for k, control in enumerate(controls):
        current = step_torque(current, control, params, dt, map_bounds, margin)
        trajectory[k + 1] = current
    return trajectory


def rollout_torque_batch(
    state: np.ndarray,
    controls: np.ndarray,
    params,
    dt: float,
    map_bounds=None,
    margin: float = 0.0,
) -> np.ndarray:
    """Vectorized rollout for sampled MPC candidates."""
    trajectories = np.zeros((controls.shape[0], controls.shape[1] + 1, 5), dtype=float)
    current = np.repeat(np.array(state, dtype=float).reshape(1, 5), controls.shape[0], axis=0)
    trajectories[:, 0, :] = current

    clipped = np.array(controls, dtype=float, copy=True)
    clipped[..., 0] = np.clip(clipped[..., 0], -params.tau_max, params.tau_max)
    clipped[..., 1] = np.clip(clipped[..., 1], -params.tau_max, params.tau_max)

    for k in range(controls.shape[1]):
        tau_r = clipped[:, k, 0]
        tau_l = clipped[:, k, 1]
        theta = current[:, 2]
        v = current[:, 3]
        omega = current[:, 4]

        v_dot = params.k_v * (tau_r + tau_l) - params.c_v * v
        omega_dot = params.k_omega * (tau_r - tau_l) - params.c_omega * omega

        next_x = current[:, 0] + v * np.cos(theta) * dt
        next_y = current[:, 1] + v * np.sin(theta) * dt
        if map_bounds is not None:
            x_min, x_max, y_min, y_max = map_bounds
            next_x = np.clip(next_x, x_min + margin, x_max - margin)
            next_y = np.clip(next_y, y_min + margin, y_max - margin)

        current = np.column_stack(
            [
                next_x,
                next_y,
                (theta + omega * dt + np.pi) % (2.0 * np.pi) - np.pi,
                np.clip(v + v_dot * dt, 0.0, params.v_max),
                np.clip(omega + omega_dot * dt, -params.omega_max, params.omega_max),
            ]
        )
        trajectories[:, k + 1, :] = current
    return trajectories
