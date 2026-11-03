"""Six-step (block commutation) control with Hall sensors — Python reference of c/src/six_step.c.

Every period (T_s):
  1. Offset calibration at start-up, as in the FOC controller.
  2. Speed from the Hall edges: the number of periods between consecutive edges is kept for
     the last six edges (one electrical turn, which averages out the placement errors of the
     sensors). No edge for hall_timeout: speed zero.
  3. Every speed_div periods: reference ramp, speed PI -> current reference in [0, i_max]
     (motoring only). In torque mode the reference is the current itself.
  4. Commutation table: the Hall code selects the leg switched with PWM (complementary, duty d)
     and the leg held low; the third leg is off and its current decays through the diodes.
  5. Current PI on ½·(i_hi − i_lo), the mean of the two conducting currents. At a commutation
     one of them is the incoming current, which starts from zero: regulating it alone would
     saturate the PI at half of the commutations and make the torque jump. The output is the
     line voltage d·V_dc applied to the two conducting phases, divided by the measured V_dc.

Below v_dc_min all legs are off until the voltage is back above 1.1·v_dc_min; the current
integrator restarts from zero. An invalid configuration is refused.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from ..motor import COMMUTATION
from .common import OFF, UV_RESUME, Inputs, Outputs, check_config, clamp
from .pi import PI


@dataclass(frozen=True)
class SixStepConfig:
    ts: float
    pole_pairs: int
    kp_i: float                # current PI (V/A)
    ki_i: float                # (V/(A·s))
    kp_w: float                # speed PI (A per rad/s)
    ki_w: float                # (A/rad)
    i_max: float               # current limit (A)
    accel_max: float           # reference ramp (rad/s²)
    speed_div: int = 20
    calib_periods: int = 200
    hall_timeout: float = 0.05     # s without edge -> standstill
    v_dc_min: float = 10.0         # undervoltage lockout (V)
    speed_loop: bool = True

    def __post_init__(self) -> None:
        check_config(self, positive=("ts", "i_max", "accel_max", "hall_timeout", "v_dc_min"),
                     non_negative=("kp_i", "ki_i", "kp_w", "ki_w"),
                     uint16=("pole_pairs", "speed_div", "calib_periods"), at_least_one=("pole_pairs", "speed_div"))
        if not self.hall_timeout < 65535 * self.ts:
            raise ValueError("hall_timeout must be shorter than 65535 periods (16-bit edge counter)")


class SixStepController:
    EDGES = 6

    def __init__(self, cfg: SixStepConfig) -> None:
        self.cfg = cfg
        self.pi_i = PI(cfg.kp_i, cfg.ki_i, cfg.ts)
        self.pi_w = PI(cfg.kp_w, cfg.ki_w, cfg.speed_div * cfg.ts)
        self.calib_count = 0
        self.offset_acc = [0.0, 0.0, 0.0]
        self.offset = [0.0, 0.0, 0.0]
        self.undervoltage = False
        self.hall_prev = -1
        self.have_edge = False
        self.edge_count = 0
        self.intervals = [0] * self.EDGES
        self.n_intervals = 0
        self.idx = 0
        self.div_count = 0
        self.speed_est = 0.0
        self.ref_ramped = 0.0
        self.i_ref = 0.0
        self.i_meas = 0.0
        self.v_cmd = 0.0
        self.duty = 0.0

    @property
    def running(self) -> bool:
        return self.calib_count >= self.cfg.calib_periods

    def telemetry(self) -> dict:
        return {"i_ref": self.i_ref, "i_meas": self.i_meas, "v_cmd": self.v_cmd, "duty": self.duty,
                "speed_est": self.speed_est, "ref_ramped": self.ref_ramped, "running": float(self.running)}

    def _speed(self) -> float:
        n = self.n_intervals
        if n == 0:
            return 0.0
        total = sum(self.intervals[:n]) if n < self.EDGES else sum(self.intervals)
        return n * (math.pi / 3.0) / (self.cfg.pole_pairs * total * self.cfg.ts)

    def step(self, inp: Inputs) -> Outputs:
        c = self.cfg
        if self.calib_count < c.calib_periods:
            for k in range(3):
                self.offset_acc[k] += inp.i_abc[k]
            self.calib_count += 1
            if self.calib_count == c.calib_periods:
                self.offset = [a / c.calib_periods for a in self.offset_acc]
            return OFF
        i_corr = [inp.i_abc[k] - self.offset[k] for k in range(3)]

        # speed from the Hall edges (the counter saturates like its 16-bit C counterpart)
        self.edge_count = min(self.edge_count + 1, 65535)
        if inp.hall != self.hall_prev:
            if self.hall_prev >= 0:
                if self.have_edge:
                    self.intervals[self.idx] = self.edge_count
                    self.idx = (self.idx + 1) % self.EDGES
                    self.n_intervals = min(self.n_intervals + 1, self.EDGES)
                self.have_edge = True
            self.edge_count = 0
            self.hall_prev = inp.hall
        if self.edge_count * c.ts > c.hall_timeout:
            self.n_intervals = 0
            self.idx = 0
            self.have_edge = False

        self.div_count += 1
        if self.div_count >= c.speed_div:
            self.div_count = 0
            self.speed_est = self._speed()
            if c.speed_loop:
                step_max = c.accel_max * c.speed_div * c.ts
                self.ref_ramped += clamp(inp.reference - self.ref_ramped, -step_max, step_max)
                self.i_ref = self.pi_w.step(self.ref_ramped - self.speed_est, 0.0, c.i_max)
        if not c.speed_loop:
            self.i_ref = clamp(inp.reference, 0.0, c.i_max)

        legs = COMMUTATION.get(inp.hall)
        if legs is None:      # invalid code (0 or 7): sensor fault, all legs off
            return OFF
        if self.undervoltage:
            if not inp.v_dc >= UV_RESUME * c.v_dc_min:
                return OFF
            self.undervoltage = False
        elif not inp.v_dc >= c.v_dc_min:        # undervoltage, or a NaN measurement
            self.undervoltage = True
            self.pi_i.reset()
            return OFF
        hi, lo = legs
        self.i_meas = 0.5 * (i_corr[hi] - i_corr[lo])
        self.v_cmd = self.pi_i.step(self.i_ref - self.i_meas, 0.0, inp.v_dc)
        self.duty = self.v_cmd / inp.v_dc
        duty = [0.0, 0.0, 0.0]
        enable = [0, 0, 0]
        duty[hi] = self.duty
        enable[hi] = enable[lo] = 1
        return Outputs((duty[0], duty[1], duty[2]), (enable[0], enable[1], enable[2]))
