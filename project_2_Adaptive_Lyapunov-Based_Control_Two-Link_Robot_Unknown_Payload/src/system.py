"""
Two-link planar robot manipulator carrying a point payload.

Dynamics:
    M(theta, m_p) ddtheta + C(theta, dtheta, m_p) dtheta + G(theta, m_p) = tau

The payload mass is a physical plant parameter. Controllers may ignore it or
estimate it, but the plant always applies the true payload dynamics.
"""

import numpy as np


class TwoLinkManipulator:
    """Rigid two-link arm rotating in a vertical plane."""

    def __init__(
        self,
        m1: float,
        m2: float,
        l1: float,
        l2: float,
        g: float = 9.81,
        payload_mass: float = 0.0,
    ):
        self.m1 = m1
        self.m2 = m2
        self.l1 = l1
        self.l2 = l2
        self.g = g
        self.payload_mass = float(payload_mass)
        if self.payload_mass < 0.0:
            raise ValueError("payload_mass must be non-negative")

    # ------------------------------------------------------------------
    # Dynamic matrices
    # ------------------------------------------------------------------

    def link_inertia_matrix(self, theta: np.ndarray) -> np.ndarray:
        """Known link-only inertia matrix."""
        _, th2 = theta
        cos_th2 = np.cos(th2)

        M11 = (self.m1 + self.m2) * self.l1**2 + self.m2 * self.l2**2 + 2 * self.m2 * self.l1 * self.l2 * cos_th2
        M12 = self.m2 * self.l2**2 + self.m2 * self.l1 * self.l2 * cos_th2
        M22 = self.m2 * self.l2**2

        return np.array([[M11, M12],
                         [M12, M22]])

    def payload_inertia_regressor(self, theta: np.ndarray) -> np.ndarray:
        """Inertia contribution per 1 kg point payload at the end effector."""
        _, th2 = theta
        cos_th2 = np.cos(th2)

        P11 = self.l1**2 + self.l2**2 + 2 * self.l1 * self.l2 * cos_th2
        P12 = self.l2**2 + self.l1 * self.l2 * cos_th2
        P22 = self.l2**2

        return np.array([[P11, P12],
                         [P12, P22]])

    def inertia_matrix(self, theta: np.ndarray) -> np.ndarray:
        """Full plant inertia matrix M(theta, m_p)."""
        return self.link_inertia_matrix(theta) + self.payload_mass * self.payload_inertia_regressor(theta)

    def link_coriolis_matrix(self, theta: np.ndarray, dtheta: np.ndarray) -> np.ndarray:
        """Known link-only Coriolis / centrifugal matrix.

        Constructed via Christoffel symbols so that (dM - 2C) is skew-symmetric.
        """
        _, th2 = theta
        dth1, dth2 = dtheta

        h = -self.m2 * self.l1 * self.l2 * np.sin(th2)

        C11 = h * dth2
        C12 = h * (dth1 + dth2)
        C21 = -h * dth1
        C22 = 0.0

        return np.array([[C11, C12],
                         [C21, C22]])

    def payload_coriolis_regressor(self, theta: np.ndarray, dtheta: np.ndarray) -> np.ndarray:
        """Coriolis contribution per 1 kg point payload at the end effector."""
        _, th2 = theta
        dth1, dth2 = dtheta

        h = -self.l1 * self.l2 * np.sin(th2)

        C11 = h * dth2
        C12 = h * (dth1 + dth2)
        C21 = -h * dth1
        C22 = 0.0

        return np.array([[C11, C12],
                         [C21, C22]])

    def coriolis_matrix(self, theta: np.ndarray, dtheta: np.ndarray) -> np.ndarray:
        """Full plant Coriolis / centrifugal matrix C(theta, dtheta, m_p)."""
        return self.link_coriolis_matrix(theta, dtheta) + self.payload_mass * self.payload_coriolis_regressor(theta, dtheta)

    def link_gravity_vector(self, theta: np.ndarray) -> np.ndarray:
        """Known link-only gravity vector."""
        th1, th2 = theta

        G1 = (self.m1 + self.m2) * self.g * self.l1 * np.cos(th1) \
            + self.m2 * self.g * self.l2 * np.cos(th1 + th2)
        G2 = self.m2 * self.g * self.l2 * np.cos(th1 + th2)

        return np.array([G1, G2])

    def payload_gravity_regressor(self, theta: np.ndarray) -> np.ndarray:
        """Gravity torque contribution per 1 kg point payload."""
        th1, th2 = theta
        return np.array([
            self.g * (self.l1 * np.cos(th1) + self.l2 * np.cos(th1 + th2)),
            self.g * self.l2 * np.cos(th1 + th2),
        ])

    def gravity_vector(self, theta: np.ndarray) -> np.ndarray:
        """Full plant gravity vector G(theta, m_p)."""
        return self.link_gravity_vector(theta) + self.payload_mass * self.payload_gravity_regressor(theta)

    # ------------------------------------------------------------------
    # State derivative (for ODE solvers)
    # ------------------------------------------------------------------

    def dynamics(self, t: float, state: np.ndarray, a: np.ndarray) -> np.ndarray:
        """Compute dx/dt = f(x, a).

        Parameters
        ----------
        t     : current time (unused, required by solve_ivp signature)
        state : [theta1, theta2, dtheta1, dtheta2]
        a     : [tau1, tau2]  control action / applied torques

        Returns
        -------
        dstate : [dtheta1, dtheta2, ddtheta1, ddtheta2]
        """
        theta = state[:2]
        dtheta = state[2:]

        M = self.inertia_matrix(theta)
        C = self.coriolis_matrix(theta, dtheta)
        G = self.gravity_vector(theta)

        # M * ddtheta = tau - C * dtheta - G
        ddtheta = np.linalg.solve(M, a - C @ dtheta - G)

        return np.array([dtheta[0], dtheta[1], ddtheta[0], ddtheta[1]])
