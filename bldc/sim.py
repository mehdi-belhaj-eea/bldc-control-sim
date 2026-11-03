"""Closed-loop simulation: sample, compute, apply one period later.

At the start of each PWM period the sensors are sampled (center-aligned PWM: all low sides on,
where the phase shunts read the mean current). The controller computes the next duty cycles
during that period; they are applied at the start of the following one. This one-period
computation delay, plus the half period of the PWM update, is the 1.5·T_s delay used by the
tuning.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Callable

import numpy as np

from .control.common import OFF, Inputs
from .plant import Plant
from .sensors import Sensors


@dataclass
class Result:
    t: np.ndarray                 # sample instants (s)
    speed: np.ndarray             # mechanical speed (rad/s)
    reference: np.ndarray         # reference given to the controller
    theta_e: np.ndarray           # true electrical angle (rad)
    i_abc: np.ndarray             # true phase currents at the samples (A), shape (n, 3)
    torque: np.ndarray            # mean electromagnetic torque over the period after the sample (N·m)
    load: np.ndarray              # mean load torque over the same period (N·m)
    duty: np.ndarray              # duty cycles applied over that period, shape (n, 3)
    i_peak: np.ndarray            # largest phase current (absolute) over that period (A)
    telemetry: dict = field(default_factory=dict)

    @property
    def rpm(self) -> np.ndarray:
        return self.speed * 30.0 / math.pi

    @property
    def i_dq(self) -> tuple[np.ndarray, np.ndarray]:
        """True d and q currents (amplitude-invariant Park with the true angle)."""
        a, b, c = self.i_abc[:, 0], self.i_abc[:, 1], self.i_abc[:, 2]
        alpha = (2.0 * a - b - c) / 3.0
        beta = (b - c) / math.sqrt(3.0)
        cs, sn = np.cos(self.theta_e), np.sin(self.theta_e)
        return alpha * cs + beta * sn, -alpha * sn + beta * cs

    def window(self, t0: float, t1: float) -> np.ndarray:
        return (self.t >= t0) & (self.t < t1)


def simulate(plant: Plant, controller, t_end: float, reference: Callable[[float], float],
             sensors: Sensors | None = None, telemetry: tuple[str, ...] = ()) -> Result:
    """Run plant and controller for t_end seconds. reference(t) is given to the controller."""
    sensors = sensors or Sensors()
    n = int(round(t_end / plant.t_pwm))
    t = np.empty(n)
    speed = np.empty(n)
    ref = np.empty(n)
    theta = np.empty(n)
    i_abc = np.empty((n, 3))
    torque = np.empty(n)
    load = np.empty(n)
    duty = np.empty((n, 3))
    i_peak = np.empty(n)
    tm = {key: np.empty(n) for key in telemetry}
    pending = OFF
    for k in range(n):
        t_k = k * plant.t_pwm
        i_meas, v_dc, angle, hall = sensors.sample(plant)
        r = reference(t_k)
        out = controller.step(Inputs(i_meas, v_dc, angle, hall, r))
        t[k], speed[k], ref[k], theta[k] = t_k, plant.omega_m, r, plant.theta_e
        i_abc[k] = plant.i
        if telemetry:
            values = controller.telemetry()
            for key in telemetry:
                tm[key][k] = values[key]
        plant.step_period(pending.duty, pending.enable)
        torque[k], load[k], i_peak[k] = plant.torque_avg, plant.load_avg, plant.i_peak
        duty[k] = [d if e else math.nan for d, e in zip(pending.duty, pending.enable)]
        pending = out
    return Result(t, speed, ref, theta, i_abc, torque, load, duty, i_peak, tm)
