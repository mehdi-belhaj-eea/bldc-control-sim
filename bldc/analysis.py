"""Figures of merit computed from simulation results."""

from __future__ import annotations

import numpy as np


def ripple(x: np.ndarray) -> float:
    """Peak-to-peak value divided by the mean."""
    return float((np.max(x) - np.min(x)) / np.mean(x))


def harmonics(x: np.ndarray, samples_per_cycle: int, orders) -> dict[int, float]:
    """Amplitudes (peak) of the given harmonic orders of x.

    x must hold a whole number of fundamental cycles of samples_per_cycle samples each, so that
    every harmonic falls exactly on an FFT bin (no window needed).
    """
    n = len(x)
    cycles = n // samples_per_cycle
    if cycles < 1 or cycles * samples_per_cycle != n:
        raise ValueError("x must hold a whole number of cycles")
    spec = np.abs(np.fft.rfft(x)) * 2.0 / n
    return {h: float(spec[h * cycles]) if h * cycles < len(spec) else 0.0 for h in orders}


def thd(x: np.ndarray, samples_per_cycle: int, max_order: int = 50) -> float:
    """Total harmonic distortion of a periodic waveform (rms of harmonics 2..max / fundamental)."""
    h = harmonics(x - np.mean(x), samples_per_cycle, range(1, max_order + 1))
    return float(np.sqrt(sum(v * v for k, v in h.items() if k > 1)) / h[1])


def _crossing(t: np.ndarray, z: np.ndarray, level: float) -> float:
    """First instant z reaches level, interpolated linearly between samples; NaN if never."""
    above = np.nonzero(z >= level)[0]
    if len(above) == 0:
        return float("nan")
    k = int(above[0])
    if k == 0:
        return float(t[0])
    return float(t[k - 1] + (level - z[k - 1]) / (z[k] - z[k - 1]) * (t[k] - t[k - 1]))


def step_metrics(t: np.ndarray, y: np.ndarray, y0: float, y1: float, band: float = 0.02) -> dict[str, float]:
    """Overshoot, 10–90 % rise time, time to 90 % and settling time (±band) of a step response.

    t is counted from the step. Crossing instants are interpolated between samples. A level
    never reached, or a response still outside the band at the end, gives NaN.
    """
    z = (y - y0) / (y1 - y0)
    t10, t90 = _crossing(t, z, 0.1), _crossing(t, z, 0.9)
    outside = np.nonzero(np.abs(z - 1.0) > band)[0]
    if len(outside) == 0:
        settle = float(t[0])
    elif outside[-1] + 1 < len(t):
        settle = float(t[outside[-1] + 1])
    else:
        settle = float("nan")
    return {"overshoot": float(max(np.max(z) - 1.0, 0.0)), "rise_time": t90 - t10, "t90": t90,
            "settling_time": settle}
