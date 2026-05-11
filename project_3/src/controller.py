from __future__ import annotations

from dataclasses import dataclass
from math import inf

import numpy as np

from .system import PhysicalParams


@dataclass(frozen=True)
class ControllerGains:
    k1: float
    K: float
    gamma: float


@dataclass(frozen=True)
class ControllerLimits:
    rho_hat_min: float
    rho_hat_max: float
    thrust_min: float = 0.0
    thrust_max: float | None = None


@dataclass(frozen=True)
class ControllerOutput:
    thrust_command: float
    nominal_thrust: float
    rho_hat_dot: float
    e: float
    v_e: float
    delta: float


def clip_rho_hat(rho_hat: float, limits: ControllerLimits) -> float:
    return float(np.clip(rho_hat, limits.rho_hat_min, limits.rho_hat_max))


def _clip_thrust(thrust: float, limits: ControllerLimits) -> float:
    upper = inf if limits.thrust_max is None else limits.thrust_max
    return float(np.clip(thrust, limits.thrust_min, upper))


def tracking_errors(
    state: np.ndarray,
    reference: tuple[float, float, float],
    gains: ControllerGains,
) -> tuple[float, float, float]:
    z, z_dot = state
    z_d, z_d_dot, _ = reference
    e = z - z_d
    v_e = z_dot - z_d_dot
    delta = v_e + gains.k1 * e
    return float(e), float(v_e), float(delta)


def nominal_thrust_term(
    e: float,
    v_e: float,
    delta: float,
    z_d_ddot: float,
    physical: PhysicalParams,
    gains: ControllerGains,
) -> float:
    return physical.mass * (
        physical.gravity
        + z_d_ddot
        - gains.k1 * v_e
        - gains.K * delta
        - e
    )


class NominalBacksteppingController:
    name = "nominal"
    label = "Nominal backstepping"

    def evaluate(
        self,
        state: np.ndarray,
        reference: tuple[float, float, float],
        rho_hat: float,
        physical: PhysicalParams,
        gains: ControllerGains,
        limits: ControllerLimits,
    ) -> ControllerOutput:
        del rho_hat
        e, v_e, delta = tracking_errors(state, reference, gains)
        _, _, z_d_ddot = reference
        t_nom = nominal_thrust_term(e, v_e, delta, z_d_ddot, physical, gains)
        thrust = _clip_thrust(t_nom, limits)
        return ControllerOutput(thrust, t_nom, 0.0, e, v_e, delta)


class AdaptiveFTCController:
    name = "adaptive"
    label = "Adaptive FTC"

    def evaluate(
        self,
        state: np.ndarray,
        reference: tuple[float, float, float],
        rho_hat: float,
        physical: PhysicalParams,
        gains: ControllerGains,
        limits: ControllerLimits,
    ) -> ControllerOutput:
        rho_hat_safe = clip_rho_hat(rho_hat, limits)
        e, v_e, delta = tracking_errors(state, reference, gains)
        _, _, z_d_ddot = reference
        t_nom = nominal_thrust_term(e, v_e, delta, z_d_ddot, physical, gains)
        thrust = _clip_thrust(t_nom / rho_hat_safe, limits)
        rho_hat_dot = gains.gamma * delta * t_nom / (physical.mass * rho_hat_safe)
        return ControllerOutput(thrust, t_nom, rho_hat_dot, e, v_e, delta)
