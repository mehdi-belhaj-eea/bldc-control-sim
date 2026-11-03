"""The drive used in the examples and tests: invented, generic values.

A 48 V, 4-pole-pair permanent-magnet motor of about 400 W at 3000 rpm, 20 kHz PWM, 0.5 µs dead
time. The same windings exist with a sinusoidal or a trapezoidal back-EMF, to compare the two
control methods on both. These are not the data of a commercial motor.
"""

from __future__ import annotations

import math

from .control import tuning
from .control.foc import FocConfig
from .control.six_step import SixStepConfig
from .motor import Motor

V_DC = 48.0
F_PWM = 20e3
TS = 1.0 / F_PWM
DEAD_TIME = 0.5e-6
SPEED_DIV = 20          # speed loop at 1 kHz
SPEED_ALPHA = 0.5


def rpm(x: float) -> float:
    """rpm -> mechanical rad/s."""
    return x * math.pi / 30.0


def motor(emf: str = "sinusoidal") -> Motor:
    return Motor(r=0.12, l=0.25e-3, psi_m=0.016, pole_pairs=4, j=5e-4, b=1e-4, emf=emf)


def foc_gains(m: Motor):
    cur = tuning.current_loop(m.r, m.l, TS)
    spd = tuning.speed_loop(m.j, m.k_t, tuning.speed_t_sigma(TS, SPEED_DIV, SPEED_ALPHA), a=3.0)
    return cur, spd


def foc_config(m: Motor, **overrides) -> FocConfig:
    cur, spd = foc_gains(m)
    cfg = dict(ts=TS, pole_pairs=m.pole_pairs, kp_i=cur.kp, ki_i=cur.ki, kp_w=spd.kp, ki_w=spd.ki,
               l=m.l, psi_m=m.psi_fundamental, i_max=20.0, accel_max=rpm(1500.0) / 0.1,
               speed_div=SPEED_DIV, speed_alpha=SPEED_ALPHA, dt_comp=DEAD_TIME / TS)
    cfg.update(overrides)
    return FocConfig(**cfg)


def six_step_k_t(m: Motor) -> float:
    """Mean torque per ampere with 120-degree block currents."""
    if m.emf == "trapezoidal":
        return m.k_t_six_step
    return 3.0 * math.sqrt(3.0) / math.pi * m.pole_pairs * m.psi_m


def six_step_t_sigma(m: Motor, speed_rpm: float) -> float:
    """Small lags of the six-step speed loop at a given speed.

    The speed is averaged over one electrical turn (six Hall edges): half a turn of delay,
    which grows as the speed falls.
    """
    t_turn = 2.0 * math.pi / (m.pole_pairs * rpm(speed_rpm))
    return 2.0 * 1.5 * TS + SPEED_DIV * TS + 0.5 * t_turn


SIX_STEP_DESIGN_RPM = 600.0     # range 300-3000 rpm: 52° of margin here, 34° at 300 rpm (methods §7)


def six_step_gains(m: Motor, design_rpm: float = SIX_STEP_DESIGN_RPM):
    # current loop: two phases in series, 2R and 2L
    cur = tuning.current_loop(2.0 * m.r, 2.0 * m.l, TS)
    # speed loop designed at a low speed, where the Hall-based measurement is slow; faster
    # speeds only shorten its delay and add margin
    spd = tuning.speed_loop(m.j, six_step_k_t(m), six_step_t_sigma(m, design_rpm), a=3.0)
    return cur, spd


def six_step_config(m: Motor, **overrides) -> SixStepConfig:
    cur, spd = six_step_gains(m)
    cfg = dict(ts=TS, pole_pairs=m.pole_pairs, kp_i=cur.kp, ki_i=cur.ki, kp_w=spd.kp, ki_w=spd.ki,
               i_max=20.0, accel_max=rpm(1500.0) / 0.1, speed_div=SPEED_DIV)
    cfg.update(overrides)
    return SixStepConfig(**cfg)
