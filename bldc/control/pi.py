"""Discrete PI controller with feed-forward and anti-windup (conditional integration)."""

from __future__ import annotations


class PI:
    """u = kp·e + x + ff, limited to [out_min, out_max], with x the integral of ki·e.

    The integrator takes the current error before the output is computed (backward Euler):
    x ← x + ki·T_s·e, then u = kp·e + x + ff. Anti-windup: while the output is saturated the
    integrator only moves if the error drives the output back into range, and it is kept within
    the range left by the feed-forward. The limits are passed at every step because they often
    vary (voltage limit of the q axis, for instance).
    """

    def __init__(self, kp: float, ki: float, ts: float) -> None:
        self.kp, self.ki, self.ts = kp, ki, ts
        self.integ = 0.0

    def reset(self, value: float = 0.0) -> None:
        self.integ = value

    def step(self, error: float, out_min: float, out_max: float, ff: float = 0.0) -> float:
        i_new = self.integ + self.ki * self.ts * error
        u = self.kp * error + i_new + ff
        if u > out_max:
            u = out_max
            if error < 0.0:
                self.integ = i_new
        elif u < out_min:
            u = out_min
            if error > 0.0:
                self.integ = i_new
        else:
            self.integ = i_new
        self.integ = min(max(self.integ, out_min - ff), out_max - ff)
        return u
