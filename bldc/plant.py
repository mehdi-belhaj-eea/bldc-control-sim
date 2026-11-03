"""Three-phase inverter and motor, simulated one PWM period at a time.

Switched model
--------------
Each period is cut at the switching instants of center-aligned PWM, with the dead time inserted
before every turn-on. Within one sub-interval the pole voltages are constant and the back-EMF
is held at its value at the start of the sub-interval (the rotor turns by 1.8 electrical
degrees per period at 1500 rpm), so the RL phase currents are exponentials, solved in closed
form. The only approximation is that frozen EMF; max_step splits the sub-intervals further
when more accuracy is wanted.

A leg with both switches off conducts through a diode as long as its current flows: the pole
voltage is 0 for a positive current (lower diode) and V_dc for a negative one (upper diode).
The instant that current reaches zero is solved in closed form, and the leg opens. An open leg
starts conducting again if its terminal voltage leaves the rails, checked at the start of each
sub-interval. This covers the dead time, the floating phase of six-step drive and the diode
bridge of a spinning motor whose inverter is disabled.

Averaged model
--------------
One interval per period with the pole voltage d·V_dc of each enabled leg, minus the mean
dead-time error sign(i)·(t_d/T)·V_dc. Disabled legs follow the same diode logic. It is faster
and used for control design checks; the switched model is the reference.

Sign convention: a phase current is positive when it flows from the inverter into the motor.
"""

from __future__ import annotations

import math
from typing import Callable, Sequence

from .motor import TWO_PI, Motor

HIGH, LOW, OPEN = 1, 0, -1
_EPS_V = 1e-9


def _no_load(t: float, omega: float) -> float:
    return 0.0


def _zero_leg(i: list, hit: int, two_legs: bool) -> list:
    """Set the current of leg `hit` to zero, keeping the three currents summing to zero."""
    if two_legs:
        return [0.0, 0.0, 0.0]
    i[hit] = 0.0
    a, b = [k for k in range(3) if k != hit]
    half = 0.5 * (i[a] - i[b])
    i[a], i[b] = half, -half
    return i


class Plant:
    """Inverter + motor + mechanical load.

    v_dc: DC-link voltage (V); f_pwm: PWM and control frequency (Hz); dead_time (s);
    model: "switched" or "average"; load(t, omega_m) -> load torque (N·m);
    max_step: longest sub-interval (s), None to cut only at the switching events.
    """

    def __init__(self, motor: Motor, v_dc: float = 48.0, f_pwm: float = 20e3, dead_time: float = 0.0,
                 model: str = "switched", load: Callable[[float, float], float] | None = None,
                 theta_m0: float = 0.0, omega_m0: float = 0.0, max_step: float | None = None) -> None:
        if model not in ("switched", "average"):
            raise ValueError("model must be 'switched' or 'average'")
        if v_dc <= 0.0 or f_pwm <= 0.0 or not 0.0 <= dead_time < 0.25 / f_pwm:
            raise ValueError("invalid inverter parameters")
        if max_step is not None and max_step <= 0.0:
            raise ValueError("max_step must be positive")
        self.motor = motor
        self.v_dc = v_dc
        self.t_pwm = 1.0 / f_pwm
        self.dead_time = dead_time
        self.model = model
        self.load = load or _no_load
        self.max_step = max_step
        self.t = 0.0
        self.i = [0.0, 0.0, 0.0]
        self.omega_m = omega_m0
        self.theta_m = theta_m0
        self.torque_avg = 0.0    # mean electromagnetic torque over the last period (N·m)
        self.load_avg = 0.0      # mean load torque over the last period (N·m)
        self.i_peak = 0.0        # largest phase current (absolute) over the last period (A)
        self._t_int = 0.0
        self._l_int = 0.0
        self._i_max = 0.0

    # ------------------------------------------------------------------ state
    @property
    def theta_e(self) -> float:
        return (self.motor.pole_pairs * self.theta_m) % TWO_PI

    @property
    def omega_e(self) -> float:
        return self.motor.pole_pairs * self.omega_m

    # ------------------------------------------------------------------ PWM pattern
    def leg_pattern(self, duty: float, enabled: bool) -> list[tuple[float, int]]:
        """Switching sequence of one leg over a period: [(start time, HIGH/LOW/OPEN), ...]."""
        if not enabled:
            return [(0.0, OPEN)]
        if duty <= 0.0:
            return [(0.0, LOW)]
        if duty >= 1.0:
            return [(0.0, HIGH)]
        t, td = self.t_pwm, self.dead_time
        t1, t2 = 0.5 * (1.0 - duty) * t, 0.5 * (1.0 + duty) * t
        if td == 0.0:
            return [(0.0, LOW), (t1, HIGH), (t2, LOW)]
        if t1 + td < t2:
            seq = [(0.0, LOW), (t1, OPEN), (t1 + td, HIGH), (t2, OPEN)]
        else:  # pulse shorter than the dead time: the high side never turns on
            seq = [(0.0, LOW), (t1, OPEN)]
        if t2 + td < t:
            seq.append((t2 + td, LOW))
        return seq

    def _segments(self, duty: Sequence[float], enable: Sequence[int]) -> list[tuple[float, tuple]]:
        vdc, t = self.v_dc, self.t_pwm
        if self.model == "average":
            cond = []
            for k in range(3):
                if not enable[k]:
                    cond.append(None)
                    continue
                d = min(max(duty[k], 0.0), 1.0)
                u = d * vdc
                if 0.0 < d < 1.0 and self.i[k] != 0.0:
                    u -= math.copysign(self.dead_time / t * vdc, self.i[k])
                cond.append(min(max(u, 0.0), vdc))
            return self._split([(t, tuple(cond))])
        patterns = [self.leg_pattern(duty[k], bool(enable[k])) for k in range(3)]
        times = sorted({ts for p in patterns for ts, _ in p} | {t})
        segs = []
        for a, b in zip(times[:-1], times[1:]):
            if b - a <= 0.0:
                continue
            cond = []
            for p in patterns:
                state = p[0][1]
                for ts, st in p:
                    if ts <= a:
                        state = st
                cond.append(vdc if state == HIGH else 0.0 if state == LOW else None)
            segs.append((b - a, tuple(cond)))
        return self._split(segs)

    def _split(self, segs: list[tuple[float, tuple]]) -> list[tuple[float, tuple]]:
        if self.max_step is None:
            return segs
        out = []
        for dt, cond in segs:
            n = max(1, math.ceil(dt / self.max_step - 1e-9))
            out.extend([(dt / n, cond)] * n)
        return out

    # ------------------------------------------------------------------ simulation
    def step_period(self, duty: Sequence[float], enable: Sequence[int]) -> None:
        """Advance one PWM period with the duty cycles (0..1) and enables of the three legs."""
        self._t_int = 0.0
        self._l_int = 0.0
        self._i_max = max(abs(x) for x in self.i)
        for dt, cond in self._segments(duty, enable):
            self._advance(dt, cond)
        self.torque_avg = self._t_int / self.t_pwm
        self.load_avg = self._l_int / self.t_pwm
        self.i_peak = self._i_max

    def _start_conduction(self, u: list, expect: list, e: tuple, blocked: set) -> None:
        """Turn open legs into conducting diodes where the circuit forces a current."""
        vdc = self.v_dc
        driven = [k for k in range(3) if u[k] is not None]
        if len(driven) == 3:
            return
        if len(driven) == 2:
            j, m = driven
            o = 3 - j - m
            if o in blocked:
                return
            v_o = 0.5 * (u[j] - e[j] + u[m] - e[m]) + e[o]
            if v_o > vdc + _EPS_V:
                u[o], expect[o] = vdc, -1
            elif v_o < -_EPS_V:
                u[o], expect[o] = 0.0, 1
            return
        # At most one leg imposes a voltage: look for a pair of legs that can conduct, current
        # flowing into the motor at a and out at b (open legs through their diodes).
        best, best_drive = None, _EPS_V
        for a in range(3):
            for b in range(3):
                if a == b or a in blocked or b in blocked:
                    continue
                if u[a] is not None and u[b] is not None:
                    continue
                ua = u[a] if u[a] is not None else 0.0
                ub = u[b] if u[b] is not None else vdc
                drive = ua - e[a] - ub + e[b]
                if drive > best_drive:
                    best, best_drive = (a, b), drive
        if best is not None:
            a, b = best
            if u[a] is None:
                u[a], expect[a] = 0.0, 1
            if u[b] is None:
                u[b], expect[b] = vdc, -1

    def _advance(self, dt: float, cond: tuple) -> None:
        m = self.motor
        r, tau, vdc = m.r, m.tau, self.v_dc
        kp = m.pole_pairs * m.psi_m
        t_left = dt
        blocked: set = set()
        for _ in range(64):
            if t_left <= 1e-15:
                return
            s = m.shape(self.theta_e)
            we = m.pole_pairs * self.omega_m
            e = (we * m.psi_m * s[0], we * m.psi_m * s[1], we * m.psi_m * s[2])
            i = self.i
            u = [None, None, None]
            expect = [0, 0, 0]
            for k in range(3):
                if cond[k] is not None:
                    u[k] = cond[k]
                elif i[k] > 0.0:
                    u[k], expect[k] = 0.0, 1
                elif i[k] < 0.0:
                    u[k], expect[k] = vdc, -1
            self._start_conduction(u, expect, e, blocked)
            driven = [k for k in range(3) if u[k] is not None]
            if len(driven) == 3:
                vn = (u[0] + u[1] + u[2] - e[0] - e[1] - e[2]) / 3.0
                iss = [(u[k] - e[k] - vn) / r for k in range(3)]
            elif len(driven) == 2:
                j, q = driven
                a = 0.5 * (u[j] - e[j] - u[q] + e[q]) / r
                iss = [0.0, 0.0, 0.0]
                iss[j], iss[q] = a, -a
            else:
                iss = [0.0, 0.0, 0.0]
            # earliest instant at which a diode current reaches zero
            h, hit = t_left, None
            for k in range(3):
                if expect[k] == 0 or expect[k] * iss[k] >= 0.0:
                    continue
                if i[k] == 0.0:   # the leg cannot start in that direction: keep it open
                    h, hit = 0.0, k
                    break
                tz = tau * math.log((i[k] - iss[k]) / -iss[k])
                if tz < h:
                    h, hit = max(tz, 0.0), k
            if hit is not None and h <= 0.0:
                if i[hit] == 0.0:
                    blocked.add(hit)
                else:
                    self.i = _zero_leg(list(i), hit, len(driven) == 2)
                continue
            x = math.exp(-h / tau)
            g = tau / h * (1.0 - x)
            if len(driven) < 2:
                i_new = [0.0, 0.0, 0.0]
                i_avg = [0.0, 0.0, 0.0]
            else:
                i_new = [iss[k] + (i[k] - iss[k]) * x for k in range(3)]
                i_avg = [iss[k] + (i[k] - iss[k]) * g for k in range(3)]
            if hit is not None:
                i_new = _zero_leg(i_new, hit, len(driven) == 2)
                blocked = set()
            torque = kp * (s[0] * i_avg[0] + s[1] * i_avg[1] + s[2] * i_avg[2])
            self._mechanics(h, torque)
            self.i = i_new
            # each current is monotonic within a sub-interval: its extremes are at the ends
            self._i_max = max(self._i_max, abs(i_new[0]), abs(i_new[1]), abs(i_new[2]))
            t_left -= h
        raise RuntimeError("inverter solver: too many events in one interval")

    def _mechanics(self, h: float, torque: float) -> None:
        m = self.motor
        t_load = self.load(self.t, self.omega_m)
        w0 = self.omega_m
        w1 = w0 + h * (torque - t_load - m.b * w0) / m.j
        self.theta_m += 0.5 * (w0 + w1) * h
        self.omega_m = w1
        self.t += h
        self._t_int += torque * h
        self._l_int += t_load * h
