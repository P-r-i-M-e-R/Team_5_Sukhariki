"""
Adaptive Lyapunov controller with online payload-mass estimation.

The controller uses a filtered error
    s = dtheta + lambda * (theta - theta_d)
and a certainty-equivalence model of the object held at the end effector.
Only the scalar payload mass is estimated online.
"""

import numpy as np

from src.system import TwoLinkManipulator


class AdaptivePayloadMassController:
    """Certainty-equivalence adaptive controller for an unknown payload mass."""

    def __init__(
        self,
        robot: TwoLinkManipulator,
        lambda_gain: float,
        kd: float,
        alpha: float,
        initial_estimate: float,
    ):
        if lambda_gain <= 0:
            raise ValueError("lambda_gain must be positive")
        if kd <= 0:
            raise ValueError("kd must be positive")
        if alpha <= 0:
            raise ValueError("alpha must be positive")

        self.robot = robot
        self.lambda_gain = float(lambda_gain)
        self.kd = float(kd)
        self.alpha = float(alpha)
        self.initial_estimate = float(initial_estimate)

    def initial_extended_state(self, physical_state: np.ndarray) -> np.ndarray:
        """Append the initial payload-mass estimate to the plant state."""
        return np.concatenate([np.asarray(physical_state, dtype=float), [self.initial_estimate]])

    def extract_estimate(self, state: np.ndarray) -> float:
        """Return hat_m_p from an extended state, or the configured initial value."""
        state = np.asarray(state, dtype=float)
        if state.shape[0] >= 5:
            return float(state[4])
        return self.initial_estimate

    def filtered_error(self, state: np.ndarray, theta_d: np.ndarray) -> np.ndarray:
        """Return s = dtheta + lambda * e."""
        state = np.asarray(state, dtype=float)
        theta = state[:2]
        dtheta = state[2:4]
        e = theta - theta_d
        return dtheta + self.lambda_gain * e

    def payload_regressor(
        self,
        theta: np.ndarray,
        dtheta: np.ndarray,
        reference_velocity: np.ndarray,
        reference_acceleration: np.ndarray,
    ) -> np.ndarray:
        """Generalized torque contribution per 1 kg payload."""
        return (
            self.robot.payload_inertia_regressor(theta) @ reference_acceleration
            + self.robot.payload_coriolis_regressor(theta, dtheta) @ reference_velocity
            + self.robot.payload_gravity_regressor(theta)
        )

    def compute_control(self, state: np.ndarray, theta_d: np.ndarray) -> np.ndarray:
        """Compute adaptive CE torque without using the true payload mass."""
        state = np.asarray(state, dtype=float)
        theta = state[:2]
        dtheta = state[2:4]
        e = theta - theta_d
        s = self.filtered_error(state, theta_d)
        hat_m = self.extract_estimate(state)

        reference_velocity = -self.lambda_gain * e
        reference_acceleration = -self.lambda_gain * dtheta

        known_torque = (
            self.robot.link_inertia_matrix(theta) @ reference_acceleration
            + self.robot.link_coriolis_matrix(theta, dtheta) @ reference_velocity
            + self.robot.link_gravity_vector(theta)
        )
        payload_torque_per_kg = self.payload_regressor(
            theta,
            dtheta,
            reference_velocity,
            reference_acceleration,
        )

        return known_torque + hat_m * payload_torque_per_kg - self.kd * s

    def extra_state_derivative(self, state: np.ndarray, theta_d: np.ndarray) -> np.ndarray:
        """Derivative of hat_m_p from the adaptation law."""
        state = np.asarray(state, dtype=float)
        theta = state[:2]
        dtheta = state[2:4]
        e = theta - theta_d
        s = self.filtered_error(state, theta_d)
        reference_velocity = -self.lambda_gain * e
        reference_acceleration = -self.lambda_gain * dtheta
        y_payload = self.payload_regressor(theta, dtheta, reference_velocity, reference_acceleration)
        return np.array([-self.alpha * float(y_payload @ s)])

    def compute_lyapunov(self, state: np.ndarray, theta_d: np.ndarray):
        """Evaluate the composite Lyapunov function and its ideal derivative."""
        state = np.asarray(state, dtype=float)
        theta = state[:2]
        hat_m = self.extract_estimate(state)
        s = self.filtered_error(state, theta_d)
        tilde = hat_m - self.robot.payload_mass

        M = self.robot.inertia_matrix(theta)
        filtered_energy = 0.5 * s @ M @ s
        estimation = 0.5 / self.alpha * tilde**2

        L = filtered_energy + estimation
        dL = -self.kd * (s @ s)
        return float(L), float(dL)
