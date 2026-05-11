# Project 3: Fault-Tolerant Backstepping Control of a Vertical Rocket

This repository implements the project described in `README(6).md`: a comparison between nominal backstepping control and adaptive fault-tolerant backstepping control for a vertical rocket with a partial thruster failure.

## Run

```bash
pip install -r requirements.txt
python main.py
```

The script saves plots to `figures/` and the rocket animation to `animations/rocket_flight.gif`.

## Outputs

- `figures/altitude_tracking.png`
- `figures/altitude_error.png`
- `figures/rho_estimation.png`
- `figures/lyapunov.png`
- `figures/lyapunov_derivative.png`
- `figures/thrust.png`
- `figures/phase_portrait.png`
- `animations/rocket_flight.gif`

Simulation parameters are in `configs/params.yaml`; implementation modules are in `src/`.
