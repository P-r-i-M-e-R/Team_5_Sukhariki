"""Run Project 4 simulations and generate all figures."""

from __future__ import annotations

from environment import default_config, default_obstacles, validate_obstacle_layout
from experiments import run_all
from visualization import (
    create_comparison_gif,
    ensure_dirs,
    plot_comparison_trajectories,
    plot_control_effort,
    plot_heading_change,
    plot_robot_schematic,
    plot_time_comparison,
)


def clean_outputs(results_dir) -> None:
    for pattern in ("*.png", "*.gif"):
        for path in results_dir.glob(pattern):
            path.unlink()


def print_summary(results: list) -> None:
    for result in results:
        print(f"Experiment: {result.name}")
        print(f"  reached goal: {result.reached}")
        print(f"  final distance: {result.final_distance:.3f} m")
        print(f"  min clearance: {result.min_clearance:.3f} m")
        print(f"  min safety margin: {result.min_safety_margin:.3f} m")
        print(f"  avg safety margin: {result.avg_safety_margin:.3f} m")
        print(f"  boundary violations: {result.boundary_violations}")
        print(f"  path length: {result.path_length:.3f} m")
        print(f"  path smoothness: {result.path_smoothness:.3f} rad")
        print(f"  heading aggressiveness: {result.heading_aggressiveness:.3f} rad/s")
        print(f"  near-obstacle events: {result.near_obstacle_events}")
        print(f"  progress efficiency: {result.progress_efficiency:.3f}")
        print(f"  average progress rate: {result.average_progress_rate:.3f} m/s")
        print(f"  control effort: {result.control_effort:.3f}")
        print(f"  torque energy: {result.torque_energy:.3f}")
        print(f"  time to goal: {result.time_to_goal:.3f} s")
        print(f"  collision: {result.collision}")
        print(f"  avg compute time: {1000.0 * result.avg_compute_time:.2f} ms")


def main() -> None:
    results_dir = ensure_dirs()
    clean_outputs(results_dir)
    margin, pair, time = validate_obstacle_layout(default_obstacles(), default_config())
    print(f"Obstacle layout valid: min margin {margin:.3f} m for {pair[0]}/{pair[1]} at t={time:.2f}s")

    result_map = run_all()
    results = [result_map["mpc"], result_map["apf"]]

    plot_robot_schematic(results_dir / "robot_model_schematic.png")
    plot_comparison_trajectories(results, results_dir / "comparison_trajectories.png")
    plot_time_comparison(results, "goal_distances", "distance [m]", "Distance to goal", results_dir / "comparison_distance_to_goal.png")
    plot_time_comparison(
        results,
        "safety_margins",
        "safety margin [m]",
        "Safety margin to obstacles",
        results_dir / "comparison_safety_margin.png",
    )
    plot_heading_change(results, results_dir / "comparison_heading_change.png")
    plot_control_effort(results, results_dir / "comparison_control_effort.png")
    create_comparison_gif(results, results_dir / "comparison_animation.gif")
    print_summary(results)
    print(f"Results saved to {results_dir.resolve()}")


if __name__ == "__main__":
    main()
