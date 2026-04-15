import numpy as np

class PIDController:
    """Independent PID for each joint"""

    def __init__(self, kp: np.ndarray, ki: np.ndarray, kd: np.ndarray):
        """
        Parameters:
        kp, ki, kd: array-like of shape (2,) - gains for joint 1 and joint 2.
        """

        self.kp = np.asarray(kp, dtype=float)
        self.ki = np.asarray(ki, dtype=float)
        self.kd = np.asarray(kd, dtype=float)

        self.integral_error = np.zeros(2)
        self.prev_error = np.zeros(2)

    def reset(self):
        """Clear integrator and derivative memory"""

        self.integral_error = np.zeros(2)
        self.prev_error = np.zeros(2)

    def compute_control(self, state: np.ndarray, theta_d: np.ndarray, dt: float) -> np.ndarray:
        """Compute PID torques.

        Parameters:
        state: [theta1, theta2, dtheta1, dtheta2]
        theta_d: [theta1_d, theta2_d]
        dt: time step for numerical derivative and integration

        Returns:
        a: [tau1, tau2]
        """
        
        theta = state[:2]
        error = theta_d - theta   

        self.integral_error += error * dt
        d_error = (error - self.prev_error) / dt if dt > 0 else np.zeros(2)
        self.prev_error = error.copy()

        a = self.kp * error + self.ki * self.integral_error + self.kd * d_error
        return a
