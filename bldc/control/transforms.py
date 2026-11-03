"""Clarke and Park transforms (amplitude invariant) and space-vector PWM."""

from __future__ import annotations

import math

TWO_PI = 2.0 * math.pi
SQRT3 = math.sqrt(3.0)


def wrap_pi(x: float) -> float:
    """Angle wrapped to (-pi, pi]."""
    x = math.fmod(x + math.pi, TWO_PI)
    if x <= 0.0:
        x += TWO_PI
    return x - math.pi


def clarke(a: float, b: float, c: float) -> tuple[float, float]:
    """abc -> alpha-beta, amplitude invariant (the zero-sequence part is rejected)."""
    return (2.0 * a - b - c) / 3.0, (b - c) / SQRT3


def inverse_clarke(alpha: float, beta: float) -> tuple[float, float, float]:
    return alpha, -0.5 * alpha + 0.5 * SQRT3 * beta, -0.5 * alpha - 0.5 * SQRT3 * beta


def park(alpha: float, beta: float, cos_t: float, sin_t: float) -> tuple[float, float]:
    """alpha-beta -> d-q, the d axis at angle theta (given by its cosine and sine)."""
    return alpha * cos_t + beta * sin_t, -alpha * sin_t + beta * cos_t


def inverse_park(d: float, q: float, cos_t: float, sin_t: float) -> tuple[float, float]:
    return d * cos_t - q * sin_t, d * sin_t + q * cos_t


def svpwm(v_alpha: float, v_beta: float, v_dc: float) -> tuple[float, float, float]:
    """Duty cycles of the three legs for the voltage vector (v_alpha, v_beta).

    Min-max zero-sequence injection: the phase references are shifted by -(max + min)/2, which
    centres the active vectors in the period exactly like space-vector modulation with equal
    zero vectors. Linear up to a phase-voltage amplitude of V_dc/√3; beyond, the duties clip.
    """
    a, b, c = inverse_clarke(v_alpha, v_beta)
    shift = -0.5 * (max(a, b, c) + min(a, b, c))
    out = []
    for v in (a, b, c):
        d = 0.5 + (v + shift) / v_dc
        out.append(min(max(d, 0.0), 1.0))
    return (out[0], out[1], out[2])
