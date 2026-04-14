import numpy as np
from scipy.integrate import solve_ivp
from src.system import TwoLinkManipulator


def run_simulation(
    robot: TwoLinkManipulator,
    controller,
    initial_state: np.ndarray,
    theta_d: np.ndarray,
    t_span: tuple,
    dt: float,
    is_pid: bool = False,
):
    """Simulate the closed-loop system.

    Parameters
    ----------
    robot         : TwoLinkManipulator plant
    controller    : LyapunovController or PIDController
    initial_state : [theta1, theta2, dtheta1, dtheta2]
    theta_d       : [theta1_d, theta2_d]
    t_span        : (t_start, t_end)
    dt            : fixed output time step
    is_pid        : if True, pass dt to controller.compute_control

    Returns
    -------
    times   : (N,)    time stamps
    states  : (N, 4)  state trajectories
    torques : (N, 2)  applied control actions
    """
    t_eval = np.arange(t_span[0], t_span[1] + dt, dt)
    n_steps = len(t_eval)

    states = np.zeros((n_steps, 4))
    torques = np.zeros((n_steps, 2))

    current_state = np.array(initial_state, dtype=float)

    if is_pid and hasattr(controller, "reset"):
        controller.reset()

    for i in range(n_steps):
        states[i] = current_state

        # --- control action ---
        if is_pid:
            a = controller.compute_control(current_state, theta_d, dt)
        else:
            a = controller.compute_control(current_state, theta_d)

        torques[i] = a

        # --- integrate one step ---
        if i < n_steps - 1:
            sol = solve_ivp(
                fun=lambda t, y: robot.dynamics(t, y, a),
                t_span=[t_eval[i], t_eval[i] + dt],
                y0=current_state,
                method="RK45",
                rtol=1e-6,
                atol=1e-9,
            )
            if sol.success:
                current_state = sol.y[:, -1]
            else:
                print(f"[WARNING] Integration failed at t = {t_eval[i]:.3f}")
                break

    return t_eval, states, torques
