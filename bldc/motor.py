"""Permanent-magnet motor: parameters, back-EMF shapes, torque and Hall sensors.

Conventions used throughout the package
---------------------------------------
* ``theta_e`` is the electrical angle of the d axis (the magnet flux) from the phase-a axis.
* Sinusoidal machine: the magnet flux linkage of phase k (k = 0, 1, 2 for a, b, c) is
  ``psi_m cos(theta_e - k 2pi/3)``, so its back-EMF is ``e_k = -omega_e psi_m sin(theta_e - k 2pi/3)``.
* Trapezoidal (BLDC) machine: the sine is replaced by a trapezoid with the same zero crossings,
  60-degree linear ramps and a 120-degree flat top of amplitude 1.
* Written with the "EMF angle" ``theta_emf = theta_e + pi``: ``e_k = omega_e psi_m s(theta_emf - k 2pi/3)``
  with s = sin or the trapezoid.
* Torque from the power balance: ``T = sum(e_k i_k) / omega_m = p psi_m sum(s_k i_k)``. This form
  holds at standstill too, and for any current shape.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

TWO_PI = 2.0 * math.pi
THIRD = TWO_PI / 3.0
EMF_SHAPES = ("sinusoidal", "trapezoidal")


def trapezoid(x: float) -> float:
    """Trapezoid in phase with sin(x): zero at 0 and pi, flat top of 1 over [pi/6, 5pi/6]."""
    x %= TWO_PI
    if x < math.pi / 6.0:
        return 6.0 * x / math.pi
    if x < 5.0 * math.pi / 6.0:
        return 1.0
    if x < 7.0 * math.pi / 6.0:
        return 1.0 - 6.0 * (x - 5.0 * math.pi / 6.0) / math.pi
    if x < 11.0 * math.pi / 6.0:
        return -1.0
    return -1.0 + 6.0 * (x - 11.0 * math.pi / 6.0) / math.pi


# Fundamental of the trapezoid above (Fourier coefficient b1 = 4 sin(a) / (pi a) with a = pi/6).
TRAPEZOID_FUNDAMENTAL = 12.0 / math.pi ** 2


def hall_code(theta_e: float, offsets: tuple[float, float, float] = (0.0, 0.0, 0.0)) -> int:
    """Hall code 4·H_a + 2·H_b + H_c.

    The sensors are placed so that their edges fall on the ideal six-step commutation instants
    (30 electrical degrees after each back-EMF zero crossing). ``offsets`` shifts each sensor
    (electrical rad) to model a placement error.
    """
    theta_emf = theta_e + math.pi
    code = 0
    for k in range(3):
        x = (theta_emf - k * THIRD - math.pi / 6.0 - offsets[k]) % TWO_PI
        code = (code << 1) | (1 if x < math.pi else 0)
    return code


# Hall code -> (phase driven high, phase driven low) for forward rotation.
COMMUTATION = {5: (0, 1), 4: (0, 2), 6: (1, 2), 2: (1, 0), 3: (2, 0), 1: (2, 1)}


@dataclass(frozen=True)
class Motor:
    """Star-connected permanent-magnet motor with an isolated neutral.

    r: phase resistance (ohm); l: synchronous inductance per phase (H), equal on d and q
    (surface magnets); psi_m: magnet flux linkage, peak per phase (Wb) — for the trapezoidal
    machine, the flat-top value; pole_pairs; j: total inertia (kg·m²); b: viscous friction
    (N·m·s/rad); emf: "sinusoidal" or "trapezoidal".
    """

    r: float
    l: float
    psi_m: float
    pole_pairs: int
    j: float
    b: float = 0.0
    emf: str = "sinusoidal"

    def __post_init__(self) -> None:
        if min(self.r, self.l, self.psi_m, self.j) <= 0.0 or self.b < 0.0 or self.pole_pairs < 1:
            raise ValueError("motor parameters must be positive")
        if self.emf not in EMF_SHAPES:
            raise ValueError(f"emf must be one of {EMF_SHAPES}")

    @property
    def tau(self) -> float:
        """Electrical time constant L/R (s)."""
        return self.l / self.r

    @property
    def k_e(self) -> float:
        """Back-EMF constant: phase peak (flat top) per mechanical rad/s (V·s/rad)."""
        return self.pole_pairs * self.psi_m

    @property
    def psi_fundamental(self) -> float:
        """Flux linkage of the fundamental of the back-EMF (Wb)."""
        return self.psi_m * (TRAPEZOID_FUNDAMENTAL if self.emf == "trapezoidal" else 1.0)

    @property
    def k_t(self) -> float:
        """Mean torque per ampere of q current with sinusoidal currents (N·m/A, peak amperes)."""
        return 1.5 * self.pole_pairs * self.psi_fundamental

    @property
    def k_t_six_step(self) -> float:
        """Torque per ampere with 120-degree block currents on the flat tops (trapezoidal machine)."""
        return 2.0 * self.pole_pairs * self.psi_m

    def shape(self, theta_e: float) -> tuple[float, float, float]:
        """Normalised back-EMF s_k of the three phases (e_k = omega_e psi_m s_k)."""
        theta_emf = theta_e + math.pi
        f = math.sin if self.emf == "sinusoidal" else trapezoid
        return (f(theta_emf), f(theta_emf - THIRD), f(theta_emf + THIRD))

    def back_emf(self, theta_e: float, omega_m: float) -> tuple[float, float, float]:
        """Phase back-EMFs (V) at the electrical angle theta_e and mechanical speed omega_m."""
        k = self.pole_pairs * omega_m * self.psi_m
        s = self.shape(theta_e)
        return (k * s[0], k * s[1], k * s[2])

    def torque(self, theta_e: float, i_abc: tuple[float, float, float]) -> float:
        """Electromagnetic torque (N·m) for the phase currents i_abc."""
        s = self.shape(theta_e)
        return self.pole_pairs * self.psi_m * (s[0] * i_abc[0] + s[1] * i_abc[1] + s[2] * i_abc[2])
