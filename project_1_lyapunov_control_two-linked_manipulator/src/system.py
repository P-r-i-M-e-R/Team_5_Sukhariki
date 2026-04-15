import numpy as np

class TwoLinkManipulator:
    """Class for two-link arm rotating in a vertical plane."""

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
        """
        Inertia matrix M(theta), symmetric positive-definite 2x2. 
        This matrix captures the mass distribution and coupling between the two links
        """
        _, th2 = theta
        c2 = np.cos(th2)

        # M11 is inertia of link 1 + reflected inertia from link 2
        # M12 = M21 is off-diagonal coupling term (depends on theta_2)
        # M22 is inertia of link 2 only (constant)
        M11 = (self.m1 + self.m2) * self.l1**2 + self.m2 * self.l2**2 + 2 * self.m2 * self.l1 * self.l2 * c2
        M12 = self.m2 * self.l2**2 + self.m2 * self.l1 * self.l2 * c2
        M22 = self.m2 * self.l2**2

        return np.array([[M11, M12],
                         [M12, M22]])

    def coriolis_matrix(self, theta: np.ndarray, dtheta: np.ndarray) -> np.ndarray:
        """
        Coriolis and centrifugal matrix C(theta, dtheta), 2x2.
        
        This matrix handles velocity-dependent forces that arise from the
        rotation of the links:
        - Centrifugal forces which appear when a link is already rotating
        - Coriolis forces which appear due to interaction between different joint velocities
        """
        _, th2 = theta
        dth1, dth2 = dtheta

        # Auxiliary term proportional to sin(theta_2), derived from Lagrangian
        h = -self.m2 * self.l1 * self.l2 * np.sin(th2)

        # When link 2 accelerates or link 1 moves, it creates forces on link 2
        C11 = h * dth2
        # Coupling effect where both joints' velocities affect this term
        C12 = h * (dth1 + dth2)
        # Link 1's motion causes Coriolis effect on link 2
        C21 = -h * dth1
        # Link 2's acceleration doesn't directly affect itself
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
        """
        Compute the system's state derivative: dx/dt = f(x, a, t).
        
        This implements the Lagrangian equation of motion:
        M(theta) @ ddtheta + C(theta, dtheta) @ dtheta + G(theta) = a
        
        We doing this to get solution for accelerations:
        ddtheta = M^{-1} * (a - C @ dtheta - G)
        
        Parameters:
        t: current time [s] (needed for solve_ivp)
        state: [theta1, theta2, dtheta1, dtheta2] where theta is joint angles[rad] and dtheta is joint angular velocities[rad/s]
        a: [tau1, tau2] is applied control torques[N·m]

        Returns:
        dstate: rate of change vector [dtheta1, dtheta2, ddtheta1, ddtheta2]
        """
        theta = state[:2]
        dtheta = state[2:]

        # Compute dynamic matrices M, C, G from current state
        M = self.inertia_matrix(theta)
        C = self.coriolis_matrix(theta, dtheta)
        G = self.gravity_vector(theta)

        # Solve M @ ddtheta as "a - C @ dtheta - G" for accelerations
        ddtheta = np.linalg.solve(M, a - C @ dtheta - G)

        # Return state derivative for ODE integrator
        return np.array([dtheta[0], dtheta[1], ddtheta[0], ddtheta[1]])
