"""Loop tuning from the motor parameters.

Current loop — modulus optimum (Betragsoptimum)
    Plant seen by the PI: 1/(R + sL) behind a delay T_Σ (one control period for the computation
    plus half a period for the PWM update: T_Σ = 1.5·T_s). The PI zero cancels the electrical
    pole (T_i = L/R) and kp = L/(2·T_Σ): open loop kp/(sL)·e^(-s·T_Σ), crossover 1/(2·T_Σ),
    phase margin ≈ 90° − 28.6° and about 4 % overshoot on a step.

Speed loop — symmetrical optimum
    Plant k_t/(J·s) behind the small lags T_σ (closed current loop ≈ 2·T_Σ, speed measurement
    averaged over one speed period, zero-order hold of the reference, speed filter). With the
    spacing factor a: kp = J/(a·k_t·T_σ), T_i = a²·T_σ, crossover 1/(a·T_σ), phase margin
    atan((a² − 1)/(2a)). Optimal for load disturbances; a ramp on the reference keeps the
    overshoot of setpoint changes small.
"""

from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class LoopGains:
    kp: float
    ki: float                 # integral gain (per second): ki = kp / T_i
    omega_c: float            # crossover (rad/s)
    phase_margin_deg: float
    t_sigma: float            # small time constants lumped in the design (s)


def current_loop(r: float, l: float, ts: float, delay_periods: float = 1.5) -> LoopGains:
    """PI gains of a current loop (V/A and V/(A·s)) by the modulus optimum."""
    t_sigma = delay_periods * ts
    kp = l / (2.0 * t_sigma)
    ki = r / (2.0 * t_sigma)
    omega_c = kp / l
    return LoopGains(kp, ki, omega_c, 90.0 - math.degrees(omega_c * t_sigma), t_sigma)


def speed_t_sigma(ts: float, speed_div: int, speed_alpha: float, current_delay: float = 1.5) -> float:
    """Small lags of the speed loop: closed current loop, speed averaging and hold, IIR filter."""
    tw = speed_div * ts
    t_filter = tw * (1.0 - speed_alpha) / speed_alpha   # y += alpha (x - y) sampled every tw
    return 2.0 * current_delay * ts + tw + t_filter


def speed_loop(j: float, k_t: float, t_sigma: float, a: float = 3.0) -> LoopGains:
    """PI gains of a speed loop (A per rad/s and A/rad) by the symmetrical optimum."""
    if a <= 1.0:
        raise ValueError("the spacing factor a must be above 1")
    kp = j / (a * k_t * t_sigma)
    ki = kp / (a * a * t_sigma)
    pm = math.degrees(math.atan((a * a - 1.0) / (2.0 * a)))
    return LoopGains(kp, ki, 1.0 / (a * t_sigma), pm, t_sigma)
