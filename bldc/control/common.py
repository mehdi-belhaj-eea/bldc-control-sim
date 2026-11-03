"""Inputs and outputs exchanged between the simulation and a controller, once per PWM period."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass
class Inputs:
    i_abc: tuple[float, float, float]   # measured phase currents (A)
    v_dc: float                         # measured DC-link voltage (V)
    theta_e: float = 0.0                # measured electrical angle of the d axis (rad)
    hall: int = 0                       # Hall code 4·H_a + 2·H_b + H_c
    reference: float = 0.0              # speed (mechanical rad/s), or current (A) in torque mode


@dataclass
class Outputs:
    duty: tuple[float, float, float]    # high-side duty cycle of each leg (0..1)
    enable: tuple[int, int, int]        # 1: leg switching (complementary), 0: both switches off


OFF = Outputs((0.0, 0.0, 0.0), (0, 0, 0))
UV_RESUME = 1.1     # after an undervoltage lockout, restart 10 % above the threshold (hysteresis)


def clamp(x: float, lo: float, hi: float) -> float:
    return lo if x < lo else hi if x > hi else x


def check_config(cfg, positive=(), non_negative=(), uint16=(), at_least_one=()) -> None:
    """Range checks shared by the controller configurations (the C init applies the same).

    Written as `not x > 0` so that NaN fails too. uint16 fields must fit their C type.
    """
    for name in positive:
        if not getattr(cfg, name) > 0.0:
            raise ValueError(f"{name} must be positive")
    for name in non_negative:
        if not getattr(cfg, name) >= 0.0:
            raise ValueError(f"{name} must not be negative")
    for name in uint16:
        v = getattr(cfg, name)
        if int(v) != v or not 0 <= v <= 65535:
            raise ValueError(f"{name} must be an integer in 0..65535 (uint16 in C)")
    for name in at_least_one:
        if getattr(cfg, name) < 1:
            raise ValueError(f"{name} must be at least 1")
