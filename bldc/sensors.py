"""Sensors seen by the controller: phase currents through an ADC, encoder, Hall sensors, DC link.

The default current chain is the one of hardware-design-kit, example 4: 1 mΩ shunt, gain-50
amplifier referred to mid-scale, 12-bit ADC on 3.3 V. That is 50 mV/A, ±33 A, 16.1 mA per LSB.
"""

from __future__ import annotations

import math
import random
from dataclasses import dataclass, field

from .motor import TWO_PI, hall_code


@dataclass
class CurrentSensor:
    """Phase-current measurement: offset (A) and gain error per phase, ADC quantisation, noise."""

    offset: tuple[float, float, float] = (0.0, 0.0, 0.0)
    gain_error: tuple[float, float, float] = (0.0, 0.0, 0.0)
    bits: int = 12
    full_scale: float = 33.0       # measurable range ±full_scale (A)
    noise_rms: float = 0.0         # white noise referred to the current (A rms)
    seed: int = 1

    def __post_init__(self) -> None:
        self._rng = random.Random(self.seed)

    @property
    def lsb(self) -> float:
        return 2.0 * self.full_scale / 2 ** self.bits

    def measure(self, i_abc) -> tuple[float, float, float]:
        lsb = self.lsb
        top = 2 ** (self.bits - 1)
        out = []
        for k in range(3):
            x = i_abc[k] * (1.0 + self.gain_error[k]) + self.offset[k]
            if self.noise_rms:
                x += self._rng.gauss(0.0, self.noise_rms)
            code = min(max(round(x / lsb), -top), top - 1)
            out.append(code * lsb)
        return (out[0], out[1], out[2])


@dataclass
class Encoder:
    """Incremental encoder read in quadrature (4 × lines counts per turn), aligned on the d axis.

    4096 lines = 16384 counts per turn, the resolution of a common 14-bit magnetic encoder.
    """

    lines: int = 4096
    offset: float = 0.0            # alignment error (electrical rad)

    def angle(self, theta_m: float, pole_pairs: int) -> float:
        res = TWO_PI / (4 * self.lines)
        return (pole_pairs * math.floor(theta_m / res) * res + self.offset) % TWO_PI


@dataclass
class HallSensors:
    """Three Hall sensors; offsets model a placement error of each sensor (electrical rad)."""

    offsets: tuple[float, float, float] = (0.0, 0.0, 0.0)

    def code(self, theta_e: float) -> int:
        return hall_code(theta_e, self.offsets)


@dataclass
class Sensors:
    current: CurrentSensor = field(default_factory=CurrentSensor)
    encoder: Encoder = field(default_factory=Encoder)
    hall: HallSensors = field(default_factory=HallSensors)
    v_dc_gain_error: float = 0.0

    def sample(self, plant) -> tuple[tuple[float, float, float], float, float, int]:
        """Measurements at the start of the PWM period: currents, DC link, angle, Hall code."""
        p = plant.motor.pole_pairs
        return (self.current.measure(plant.i),
                plant.v_dc * (1.0 + self.v_dc_gain_error),
                self.encoder.angle(plant.theta_m, p),
                self.hall.code(plant.theta_e))
