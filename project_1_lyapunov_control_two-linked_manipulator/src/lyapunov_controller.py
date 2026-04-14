import numpy as np
from src.system import TwoLinkManipulator


class LyapunovController:
    """PD controller with gravity compensation, proven stable via Lyapunov."""

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
        theta = state[:2]
        dtheta = state[2:]
        e = theta - theta_d

        G = self.robot.gravity_vector(theta)

        # a = -k1 * e - k2 * dtheta + G(theta)
        a = -self.k1 * e - self.k2 * dtheta + G

        return a

    def compute_lyapunov(self, state: np.ndarray, theta_d: np.ndarray):
        """Evaluate the Lyapunov function L and its derivative dL/dt.

        Useful for plotting and verifying the stability guarantee.

        Returns
        -------
        L  : float   Lyapunov function value  (>= 0)
        dL : float   time derivative of L     (<= 0)
        """
        theta = state[:2]
        dtheta = state[2:]
        e = theta - theta_d

        M = self.robot.inertia_matrix(theta)

        L = 0.5 * dtheta @ M @ dtheta + 0.5 * self.k1 * (e @ e)
        dL = -self.k2 * (dtheta @ dtheta)

        return L, dL
