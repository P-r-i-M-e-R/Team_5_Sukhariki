import numpy as np
from .system import TwoLinkManipulator


class LyapunovController:
    """PD controller with gravity compensation, proven stable via Lyapunov."""

    def __init__(self, robot: TwoLinkManipulator, k1: float, k2: float):
        self.robot = robot
        self.k1 = k1
        self.k2 = k2

    def compute_control(self, state: np.ndarray, theta_d: np.ndarray) -> np.ndarray:
        """
        Compute the control action a at the current step.
        
        We use such formula:
        a = -k1 * e - k2 * dtheta + G(theta)
        
        Where:
          - e = theta - theta_d is a position error
          - dtheta is a joint velocity
          - G(theta) is a gravity compensation
          - k1 and k2 are gains that define how hard controller reacts to errors

        Parameters:
        state: [theta1, theta2, dtheta1, dtheta2] - current state
        theta_d: [theta1_d, theta2_d] - desired joint angles

        Returns:
        a: [tau1, tau2] - applied torques
        """

        theta = state[:2]
        dtheta = state[2:]
        e = theta - theta_d

        G = self.robot.gravity_vector(theta)

        a = -self.k1 * e - self.k2 * dtheta + G

        return a

    def compute_lyapunov(self, state: np.ndarray, theta_d: np.ndarray):
        """
        Evaluate the Lyapunov function L(t) and its time derivative dL/dt for verifying stability.
        According to Lyapunov theory, if:
        1) L(t) > 0 for all (e, dtheta) != (0, 0) (positive definite)
        2) dL/dt <= 0 for all trajectories (negative semidefinite)
        
        Then the system is guaranteed to converge to the equilibrium (e=0, dtheta=0).
        
        We use such the energy function:
        L = 0.5 * dtheta^T @ M(theta) @ dtheta  +  0.5 * k1 * e^T @ e
        
        This represents the total "energy" of the system where first part represents 
        kinetic energy from joint velocities, weighted by inertia and second part represents
        potential energy from the spring-like PD control
        
        Returns
        L: float - Lyapunov function value > 0
        dL: float - time derivative, which <= 0
        """

        theta = state[:2]
        dtheta = state[2:]
        e = theta - theta_d

        M = self.robot.inertia_matrix(theta)

        L = 0.5 * dtheta @ M @ dtheta + 0.5 * self.k1 * (e @ e)
        dL = -self.k2 * (dtheta @ dtheta)

        return L, dL
