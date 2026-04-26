"""
Non-adaptive Lyapunov controller used as the Project 2 baseline.

Control law:
    tau = -k1 * (theta - theta_d) - k2 * dtheta + G_links(theta)

This controller is the Project 1 PD + gravity compensation law applied to the
loaded plant. It compensates the known links but does not know or estimate the
payload mass.
"""

import numpy as np
from src.system import TwoLinkManipulator


class LyapunovController:
    """PD controller with known-link gravity compensation and no mass adaptation."""

    def __init__(self, robot: TwoLinkManipulator, k1: float, k2: float):
        self.robot = robot
        self.k1 = k1
        self.k2 = k2

    def compute_control(self, state: np.ndarray, theta_d: np.ndarray) -> np.ndarray:
        """Compute the control action a.

        Parameters
        ----------
        state   : [theta1, theta2, dtheta1, dtheta2]
        theta_d : [theta1_d, theta2_d]  desired joint angles

        Returns
        -------
        a : [tau1, tau2]  applied torques
        """
        physical_state = np.asarray(state, dtype=float)[:4]
        theta = physical_state[:2]
        dtheta = physical_state[2:]
        e = theta - theta_d

        G = self.robot.link_gravity_vector(theta)

        return -self.k1 * e - self.k2 * dtheta + G

    def compute_lyapunov(self, state: np.ndarray, theta_d: np.ndarray):
        """Evaluate the Lyapunov function L and its derivative dL/dt.

        Useful for plotting and verifying the stability guarantee.

        Returns
        -------
        L  : float   Lyapunov function value  (>= 0)
        dL : float   derivative on the loaded plant
        """
        physical_state = np.asarray(state, dtype=float)[:4]
        theta = physical_state[:2]
        dtheta = physical_state[2:]
        e = theta - theta_d

        M = self.robot.inertia_matrix(theta)

        L = 0.5 * dtheta @ M @ dtheta + 0.5 * self.k1 * (e @ e)
        payload_disturbance = self.robot.payload_mass * self.robot.payload_gravity_regressor(theta)
        dL = -self.k2 * (dtheta @ dtheta) - dtheta @ payload_disturbance

        return L, dL
