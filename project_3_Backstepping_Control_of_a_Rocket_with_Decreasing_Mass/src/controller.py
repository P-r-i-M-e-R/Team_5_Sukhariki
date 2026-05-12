import numpy as np


class BacksteppingController:
    """
    Backstepping controller for vertical rocket with time-varying mass.

    Algorithm 1 (README Section 5):
      delta = v_e + k1 * e                              [eq. 8]
      T = m(t) * (g + zd_ddot - k1*v_e - K*delta - e)  [eq. 19]
    """

    def __init__(self, k1, K, g):
        self.k1 = k1
        self.K = K
        self.g = g

    def compute(self, z, zdot, zd, zd_dot, zd_ddot, m):
        e = z - zd
        v_e = zdot - zd_dot
        pi0 = -self.k1 * e
        delta = v_e - pi0                                                 # [eq. 8]
        T = m * (self.g + zd_ddot - self.k1 * v_e - self.K * delta - e)  # [eq. 19]
        T = max(T, 0.0)
        return T, e, v_e, delta

    def lyapunov(self, e, delta):
        """L1 = 0.5*e^2 + 0.5*delta^2  [eq. 12]"""
        return 0.5 * e ** 2 + 0.5 * delta ** 2

    def lyapunov_dot(self, e, delta):
        """dL1/dt = -k1*e^2 - K*delta^2  [eq. 20]"""
        return -self.k1 * e ** 2 - self.K * delta ** 2


class PDController:
    """
    PD baseline controller designed for nominal mass m0.
    T_PD = m0 * (g - kp*e - kd*v_e)  [eq. 23]
    """

    def __init__(self, kp, kd, g, m0):
        self.kp = kp
        self.kd = kd
        self.g = g
        self.m0 = m0

    def compute(self, z, zdot, zd, zd_dot):
        e = z - zd
        v_e = zdot - zd_dot
        T = self.m0 * (self.g - self.kp * e - self.kd * v_e)  # [eq. 23]
        T = max(T, 0.0)
        return T, e, v_e

    def lyapunov_pd(self, e, v_e):
        """L_PD = 0.5*v_e^2 + 0.5*kp*e^2  [eq. 26]"""
        return 0.5 * v_e ** 2 + 0.5 * self.kp * e ** 2

    def lyapunov_pd_dot(self, e, v_e, mu):
        """dL_PD/dt actual under m(t) != m0  [eq. 30]"""
        return (-mu * self.kd * v_e ** 2
                + self.kp * e * v_e * (1.0 - mu)
                + (mu - 1.0) * self.g * v_e)
