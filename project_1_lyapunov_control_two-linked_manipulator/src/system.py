import numpy as np

class TwoLinkManipulator:
    """Rigid two-link arm rotating in a vertical plane."""

    def __init__(self, m1: float, m2: float, l1: float, l2: float, g: float = 9.81):
        self.m1 = m1
        self.m2 = m2
        self.l1 = l1
        self.l2 = l2
        self.g = g

    # ------------------------------------------------------------------
    # Dynamic matrices
    # ------------------------------------------------------------------

    def inertia_matrix(self, theta: np.ndarray) -> np.ndarray:
        """Inertia matrix M(theta), symmetric positive-definite 2x2."""
        _, th2 = theta
        c2 = np.cos(th2)

        M11 = (self.m1 + self.m2) * self.l1**2 + self.m2 * self.l2**2 + 2 * self.m2 * self.l1 * self.l2 * c2
        M12 = self.m2 * self.l2**2 + self.m2 * self.l1 * self.l2 * c2
        M22 = self.m2 * self.l2**2

        return np.array([[M11, M12],
                         [M12, M22]])

    def coriolis_matrix(self, theta: np.ndarray, dtheta: np.ndarray) -> np.ndarray:
        """Coriolis matrix C(theta, dtheta), 2x2.
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

    def gravity_vector(self, theta: np.ndarray) -> np.ndarray:
        """Gravity vector G(theta), 2x1."""

        th1, th2 = theta

        G1 = (self.m1 + self.m2) * self.g * self.l1 * np.cos(th1) \
            + self.m2 * self.g * self.l2 * np.cos(th1 + th2)
        G2 = self.m2 * self.g * self.l2 * np.cos(th1 + th2)

        return np.array([G1, G2])

    # ------------------------------------------------------------------
    # State derivative 
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

        ddtheta = np.linalg.solve(M, a - C @ dtheta - G)

        return np.array([dtheta[0], dtheta[1], ddtheta[0], ddtheta[1]])
