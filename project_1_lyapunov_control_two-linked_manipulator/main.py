import numpy as np
import yaml

from src.system import TwoLinkManipulator
from src.lyapunov_controller import LyapunovController
from src.pid_controller import PIDController
from src.simulation import run_simulation
from src.visualization import (
    plot_comparison,
    plot_lyapunov,
    plot_phase_portrait,
    create_animation,
)


def load_config(path: str = "configs/params.yaml") -> dict:
    with open(path, "r") as f:
        return yaml.safe_load(f)


def main():
    cfg = load_config()

    # Plant
    rp = cfg["robot"]
    robot = TwoLinkManipulator(
        m1=rp["m1"], m2=rp["m2"], l1=rp["l1"], l2=rp["l2"], g=rp["g"],
    )

    # Simulation settings
    sim = cfg["simulation"]
    t_span = (sim["t_start"], sim["t_end"])
    dt = sim["dt"]
    x0 = np.array(sim["initial_state"], dtype=float)

    ctrl = cfg["control"]
    theta_d = np.array([ctrl["target_theta1"], ctrl["target_theta2"]])

    # Controllers
    lp = cfg["lyapunov"]
    lyap_ctrl = LyapunovController(robot, k1=lp["k1"], k2=lp["k2"])

    pp = cfg["pid"]
    pid_ctrl = PIDController(
        kp=[pp["kp1"], pp["kp2"]],
        ki=[pp["ki1"], pp["ki2"]],
        kd=[pp["kd1"], pp["kd2"]],
    )

    # Run simulations
    print("Running Lyapunov (PD + gravity comp.) simulation ...")
    t_lyap, states_lyap, torques_lyap = run_simulation(
        robot, lyap_ctrl, x0, theta_d, t_span, dt, is_pid=False,
    )

    print("Running PID baseline simulation ...")
    t_pid, states_pid, torques_pid = run_simulation(
        robot, pid_ctrl, x0, theta_d, t_span, dt, is_pid=True,
    )

    # Compute Lyapunov function values
    n = len(t_lyap)
    L_vals = np.zeros(n)
    dL_vals = np.zeros(n)
    for i in range(n):
        L_vals[i], dL_vals[i] = lyap_ctrl.compute_lyapunov(states_lyap[i], theta_d)

    # Generate outputs
    print("Generating plots ...")
    plot_comparison(
        t_lyap, states_lyap, torques_lyap, states_pid, torques_pid, theta_d,
    )
    plot_lyapunov(t_lyap, L_vals, dL_vals)
    plot_phase_portrait(t_lyap, states_lyap, theta_d)

    print("Creating animation ...")
    create_animation(t_lyap, states_lyap, rp["l1"], rp["l2"], theta_d)

    # Gain sensitivity analysis
    print("Analyzing gain sensitivity with 5 k-value configurations ...")
    from src.gain_analysis import analyze_k_sensitivity
    from src.visualization import plot_k_sensitivity_comparison
    
    k_sensitivity = analyze_k_sensitivity(robot, x0, theta_d, t_span, dt)
    plot_k_sensitivity_comparison(k_sensitivity)

    # Summary
    final_err_lyap = np.linalg.norm(states_lyap[-1, :2] - theta_d)
    final_err_pid = np.linalg.norm(states_pid[-1, :2] - theta_d)
    print("\n===== Results Summary =====")
    print(f"  Lyapunov — final position error: {final_err_lyap:.6f} rad")
    print(f"  PID      — final position error: {final_err_pid:.6f} rad")
    print(f"  Lyapunov — max |torque|:         {np.max(np.abs(torques_lyap)):.2f} N·m")
    print(f"  PID      — max |torque|:         {np.max(np.abs(torques_pid)):.2f} N·m")
    print("\nDone. Check figures/ and animations/ folders.")


if __name__ == "__main__":
    main()
