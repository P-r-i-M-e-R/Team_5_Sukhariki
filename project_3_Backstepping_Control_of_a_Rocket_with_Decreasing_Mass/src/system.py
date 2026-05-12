import numpy as np


class RocketSystem:
    """Vertical rocket with exponentially decreasing mass (eq. 1-4 in README)."""

    def __init__(self, m0, mf, beta, g):
        self.m0 = m0
        self.mf = mf
        self.beta = beta
        self.g = g

    def mass(self, t):
        """m(t) = mf + (m0 - mf) * exp(-beta * t)  [eq. 1]"""
        return self.mf + (self.m0 - self.mf) * np.exp(-self.beta * t)

    def mass_dot(self, t):
        """dm/dt = -beta * (m0 - mf) * exp(-beta * t)  [eq. 2]"""
        return -self.beta * (self.m0 - self.mf) * np.exp(-self.beta * t)

    def dynamics(self, state, T, t):
        """
        State: [z, zdot]
        Equation of motion: z_ddot = T/m(t) - g  [eq. 4]
        Returns state derivative [zdot, z_ddot].
        """
        z, zdot = state
        m = self.mass(t)
        z_ddot = T / m - self.g
        return np.array([zdot, z_ddot])

    def reference(self, t, z_step, t_step):
        """Step reference: z_d = 0 for t < t_step, z_d = z_step for t >= t_step."""
        zd = z_step if t >= t_step else 0.0
        zd_dot = 0.0
        zd_ddot = 0.0
        return zd, zd_dot, zd_ddot
