import numpy as np
import yaml
import matplotlib.pyplot as plt
from .system import TwoLinkManipulator
from .lyapunov_controller import LyapunovController
from .simulation import run_simulation


def load_config(path: str = "configs/params.yaml") -> dict:
    """Load experimental parameters from YAML configuration file."""

    with open(path, "r") as f:
        return yaml.safe_load(f)


def compute_performance_metrics(t: np.ndarray, states: np.ndarray, torques: np.ndarray, theta_d: np.ndarray) -> dict:
    """
    Compute performance metrics for a single simulation run.
    
    Parameters:
    t: np.ndarray which is time vector [s]
    states: np.ndarray is trajectory of system states, shape (n_steps, 4)
    torques: np.ndarray is applied control torques, shape (n_steps, 2)
    theta_d: np.ndarray is target joint configuration[rad]
    
    Returns:
    dict - Dictionary with performance metrics including settling time, overshoot, max torque, etc.
    """
    theta = states[:, :2]
    error = theta - theta_d[np.newaxis, :]
    error_norm = np.linalg.norm(error, axis=1)
    
    # Settling time: time to reach within 2% of target
    settling_threshold = 0.02
    settling_idx = np.where(error_norm < settling_threshold)[0]
    settling_time = t[settling_idx[0]] if len(settling_idx) > 0 else t[-1]
    
    # Maximum position error
    max_error = np.max(error_norm)
    
    # Final steady-state error
    final_error = error_norm[-1]
    
    # Maximum control effort (torque magnitude)
    max_torque = np.max(np.abs(torques))
    
    # Control effort integral (sum of squared torques) as energy metric
    control_energy = np.sum(torques**2) * (t[1] - t[0])
    
    return {
        "settling_time": settling_time,
        "max_error": max_error,
        "final_error": final_error,
        "max_torque": max_torque,
        "control_energy": control_energy,
    }


def analysis_k1_k2_grid(robot: TwoLinkManipulator, 
                        x0: np.ndarray, 
                        theta_d: np.ndarray,
                        t_span: tuple,
                        dt: float,
                        k1_range: np.ndarray,
                        k2_range: np.ndarray) -> dict:
    """
    Perform a grid search over (k1, k2) parameter space.
    
    Parameters:
    robot: TwoLinkManipulator
    x0: np.ndarray is initial state
    theta_d: np.ndarray is target joint configuration[rad]
    t_span: tuple consisting of(t_start, t_end) simulation interval and dt is time step for integration
    k1_range: np.ndarray which is proportional gain values to test
    k2_range : np.ndarray which is derivative gain values to test
    
    Returns:
    dict - With grid of performance metrics with shape (len(k1_range), len(k2_range))
    """

    n_k1 = len(k1_range)
    n_k2 = len(k2_range)
    
    # Initialize result tensors
    settling_times = np.zeros((n_k1, n_k2))
    max_errors = np.zeros((n_k1, n_k2))
    final_errors = np.zeros((n_k1, n_k2))
    max_torques = np.zeros((n_k1, n_k2))
    control_energies = np.zeros((n_k1, n_k2))
    
    print("\nPerforming sensitivity analysis over (k1, k2) grid ...")
    print(f"  k1 range: {k1_range}")
    print(f"  k2 range: {k2_range}")
    
    for i, k1 in enumerate(k1_range):
        for j, k2 in enumerate(k2_range):
            # Create controller with current gains
            ctrl = LyapunovController(robot, k1=k1, k2=k2)
            
            try:
                t, states, torques = run_simulation(
                    robot, ctrl, x0, theta_d, t_span, dt, is_pid=False
                )
                
                metrics = compute_performance_metrics(t, states, torques, theta_d)
                
                settling_times[i, j] = metrics["settling_time"]
                max_errors[i, j] = metrics["max_error"]
                final_errors[i, j] = metrics["final_error"]
                max_torques[i, j] = metrics["max_torque"]
                control_energies[i, j] = metrics["control_energy"]
                
            except Exception as e:
                print(f"    Warning: k1={k1}, k2={k2} failed: {e}")
                settling_times[i, j] = np.nan
                max_errors[i, j] = np.nan
                final_errors[i, j] = np.nan
                max_torques[i, j] = np.nan
                control_energies[i, j] = np.nan
            
            if (i * n_k2 + j) % max(1, n_k1 * n_k2 // 10) == 0:
                progress = (i * n_k2 + j) / (n_k1 * n_k2)
                print(f"    Progress: {progress*100:.1f}%")
    
    return {
        "k1_range": k1_range,
        "k2_range": k2_range,
        "settling_times": settling_times,
        "max_errors": max_errors,
        "final_errors": final_errors,
        "max_torques": max_torques,
        "control_energies": control_energies,
    }


def plot_gain_analysis(results: dict, output_dir: str = "figures") -> None:
    """
    Create heatmaps and contour plots of performance metrics over (k1, k2) grid.
    
    Parameters:
    results : dict which are output from analysis_k1_k2_grid containing gain ranges and metric grids.
    """
    k1_range = results["k1_range"]
    k2_range = results["k2_range"]
    
    metrics = [
        ("settling_times", "Settling Time [s]"),
        ("max_errors", "Max Error [rad]"),
        ("max_torques", "Max Torque [N·m]"),
        ("control_energies", "Control Energy [N·m·s]"),
    ]
    
    for metric_key, metric_label in metrics:
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 5))
        
        data = results[metric_key]
        
        # Heatmap
        im = ax1.imshow(data, aspect="auto", origin="lower",
                         extent=[k2_range.min(), k2_range.max(),
                                k1_range.min(), k1_range.max()],
                         cmap="viridis")
        ax1.set_xlabel("$k_2$ (derivative gain)")
        ax1.set_ylabel("$k_1$ (proportional gain)")
        ax1.set_title(f"{metric_label} (Heatmap)")
        plt.colorbar(im, ax=ax1)
        
        # Contour plot
        K1, K2 = np.meshgrid(k2_range, k1_range)
        cs = ax2.contour(K2, K1, data, levels=10)
        ax2.clabel(cs, inline=True, fontsize=8)
        ax2.set_xlabel("$k_2$ (derivative gain)")
        ax2.set_ylabel("$k_1$ (proportional gain)")
        ax2.set_title(f"{metric_label} (Contours)")
        
        plt.tight_layout()
        filename = f"{output_dir}/gain_analysis_{metric_key}.png"
        plt.savefig(filename, dpi=150)
        print(f"Saved: {filename}")
        plt.close()


def print_best_gains(results: dict) -> None:
    """Print the gain pair that achieves best performance in each metric."""

    k1_range = results["k1_range"]
    k2_range = results["k2_range"]
    
    metrics = [
        ("settling_times", "Fastest Settling"),
        ("max_errors", "Smallest Max Error"),
        ("max_torques", "Minimum Peak Torque"),
        ("control_energies", "Least Control Energy"),
    ]
    
    print("\n" + "="*70)
    print("Optimal Gains by Performance Metric")
    print("="*70)
    
    for metric_key, description in metrics:
        data = results[metric_key]
        idx = np.nanargmin(data)
        i, j = np.unravel_index(idx, data.shape)
        best_k1 = k1_range[i]
        best_k2 = k2_range[j]
        best_val = data[i, j]
        
        print(f"\n{description}:")
        print(f"  Optimal k1 = {best_k1:.1f}, k2 = {best_k2:.1f}")
        print(f"  Value: {best_val:.4f}")


def analyze_k_sensitivity(robot: TwoLinkManipulator,
                         x0: np.ndarray,
                         theta_d: np.ndarray,
                         t_span: tuple,
                         dt: float) -> dict:
    """
    Analyze system performance with 5 selected k-value pairs.
    """
    print("\n" + "="*70)
    print("Gain Sensitivity Analysis - 5 Configuration Points")
    print("="*70)
    
    # Define 5 k-pairs
    k_configs = [
        {"k1": 10,  "k2": 2,  "label": "Very small (k1=10, k2=2)"},
        {"k1": 30,  "k2": 8,  "label": "Small (k1=30, k2=8)"},
        {"k1": 100, "k2": 20, "label": "Current (k1=100, k2=20)"},
        {"k1": 150, "k2": 30, "label": "Large (k1=150, k2=30)"},
        {"k1": 200, "k2": 40, "label": "Very Large (k1=200, k2=40)"},
    ]
    
    simulations = []
    
    for config in k_configs:
        k1, k2 = config["k1"], config["k2"]
        label = config["label"]
        
        print(f"\nTesting: {label} (k1={k1}, k2={k2})")
        
        try:
            ctrl = LyapunovController(robot, k1=k1, k2=k2)
            t, states, torques = run_simulation(
                robot, ctrl, x0, theta_d, t_span, dt, is_pid=False
            )
            simulations.append((t, states, torques, k1, k2, label))
            print(f"  OK")
        except Exception as e:
            print(f"  FAILED: {e}")
    
    return {
        "theta_d": theta_d,
        "simulations": simulations,
        "k_configs": k_configs,
    }


def main(mode: str = "normal"):
    """Execute the gain sensitivity analysis."""
    cfg = load_config()
    
    # Initialize robot
    rp = cfg["robot"]
    robot = TwoLinkManipulator(
        m1=rp["m1"], m2=rp["m2"], l1=rp["l1"], l2=rp["l2"], g=rp["g"]
    )
    
    sim = cfg["simulation"]
    t_span = (sim["t_start"], sim["t_end"])
    dt = sim["dt"]
    x0 = np.array(sim["initial_state"], dtype=float)
    
    ctrl = cfg["control"]
    theta_d = np.array([ctrl["target_theta1"], ctrl["target_theta2"]])
    
    if mode == "sensitivity":
        # Analyze 5 k-value points
        results = analyze_k_sensitivity(robot, x0, theta_d, t_span, dt)
        
        # Import visualization function and plot
        from visualization import plot_k_sensitivity_comparison
        plot_k_sensitivity_comparison(results)
        
    else:
        # Full grid analysis (original mode)
        k1_range = np.linspace(10, 200, 10)   # Proportional gain sweep
        k2_range = np.linspace(5, 50, 10)     # Derivative gain sweep
        
        results = analysis_k1_k2_grid(robot, x0, theta_d, t_span, dt, k1_range, k2_range)
        
        plot_gain_analysis(results)
        
        print_best_gains(results)
    
    print("\nResults saved to figures/")


if __name__ == "__main__":
    main()
