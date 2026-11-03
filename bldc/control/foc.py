"""Field-oriented control — Python reference of the C implementation (c/src/foc.c).

Every period (T_s):
  1. Offset calibration first: during calib_periods the legs are off and the measured currents,
     which should be zero, are averaged; the averages are then subtracted from every sample.
  2. Clarke and Park transforms of the measured currents with the encoder angle.
  3. Every speed_div periods: speed from the angle difference, first-order filter, reference
     ramp (accel_max), speed PI -> q-current reference (limited to ±i_max). In torque mode the
     input reference is the q current itself. The d-current reference is zero.
  4. d and q current PIs with decoupling feed-forward (−ω_e·L·i_q on d, ω_e·(L·i_d + ψ_m) on q).
     The voltage vector is limited to modulation_limit·V_dc, d axis first.
  5. Inverse Park at the angle advanced by angle_comp·ω_e·T_s (the rotor turns during the
     computation and PWM delay), then space-vector PWM.
  6. Optional dead-time compensation: each duty is raised by dt_comp·sign(i_ref), with a
     linear transition over ±dt_comp_band. The sign comes from the current reference turned
     into phase currents at the output angle, not from the measured current: the measured
     current ripples across zero at light load, and feeding it back would act as a negative
     resistance.

Below v_dc_min (undervoltage, or a broken DC-link measurement) all legs are off until the
voltage is back above 1.1·v_dc_min. During the lockout the motor current decays and the rotor
keeps turning: on recovery the speed estimate restarts from a fresh angle reading and the
current integrators from zero. An invalid configuration is refused (ValueError here, an error
code and legs off in C).

The structure, the order of the operations and the variable names follow foc.c, so that the
two can be compared step by step (tests/test_sil.py).
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from .common import OFF, UV_RESUME, Inputs, Outputs, check_config, clamp
from .pi import PI
from .transforms import SQRT3, clarke, inverse_clarke, inverse_park, park, svpwm, wrap_pi


@dataclass(frozen=True)
class FocConfig:
    ts: float                     # control period = PWM period (s)
    pole_pairs: int
    kp_i: float                   # current PI (V/A)
    ki_i: float                   # (V/(A·s))
    kp_w: float                   # speed PI (A per rad/s)
    ki_w: float                   # (A/rad)
    l: float                      # inductance used by the decoupling (H)
    psi_m: float                  # flux linkage used by the feed-forward (Wb)
    i_max: float                  # q-current limit (A, peak)
    accel_max: float              # reference ramp (rad/s²)
    speed_div: int = 20           # speed loop every speed_div periods
    speed_alpha: float = 0.5      # speed filter: y += alpha (x − y)
    modulation_limit: float = 1.0 / SQRT3   # max phase-voltage amplitude / V_dc (linear SVPWM)
    angle_comp: float = 1.5       # angle advance, in periods
    calib_periods: int = 200      # offset calibration at start-up (0: none)
    dt_comp: float = 0.0          # dead-time compensation, as a duty (t_d/T_s); 0: off
    dt_comp_band: float = 0.3     # about half the PWM current ripple (A)
    v_dc_min: float = 10.0        # undervoltage lockout (V)
    speed_loop: bool = True       # False: torque mode, the reference is the q current (A)

    def __post_init__(self) -> None:
        check_config(self, positive=("ts", "i_max", "accel_max", "v_dc_min"),
                     non_negative=("kp_i", "ki_i", "kp_w", "ki_w", "l", "psi_m", "angle_comp", "dt_comp"),
                     uint16=("pole_pairs", "speed_div", "calib_periods"), at_least_one=("pole_pairs", "speed_div"))
        if not 0.0 < self.speed_alpha <= 1.0:
            raise ValueError("speed_alpha must be in (0, 1]")
        if not 0.0 < self.modulation_limit <= 1.0 / SQRT3 + 1e-6:
            raise ValueError("modulation_limit must be in (0, 1/√3]: the linear range of SVPWM")
        if self.dt_comp > 0.0 and not self.dt_comp_band > 0.0:
            raise ValueError("dt_comp_band must be positive when dt_comp is used")


class FocController:
    def __init__(self, cfg: FocConfig) -> None:
        self.cfg = cfg
        self.pi_d = PI(cfg.kp_i, cfg.ki_i, cfg.ts)
        self.pi_q = PI(cfg.kp_i, cfg.ki_i, cfg.ts)
        self.pi_w = PI(cfg.kp_w, cfg.ki_w, cfg.speed_div * cfg.ts)
        self.calib_count = 0
        self.offset_acc = [0.0, 0.0, 0.0]
        self.offset = [0.0, 0.0, 0.0]
        self.started = False
        self.undervoltage = False
        self.theta_prev = 0.0
        self.div_count = 0
        self.speed_est = 0.0
        self.ref_ramped = 0.0
        self.i_d = self.i_q = 0.0
        self.i_d_ref = self.i_q_ref = 0.0
        self.v_d = self.v_q = 0.0

    @property
    def running(self) -> bool:
        return self.calib_count >= self.cfg.calib_periods

    def telemetry(self) -> dict:
        return {"i_d": self.i_d, "i_q": self.i_q, "i_d_ref": self.i_d_ref, "i_q_ref": self.i_q_ref,
                "v_d": self.v_d, "v_q": self.v_q, "speed_est": self.speed_est,
                "ref_ramped": self.ref_ramped, "running": float(self.running)}

    def step(self, inp: Inputs) -> Outputs:
        c = self.cfg
        if self.calib_count < c.calib_periods:
            for k in range(3):
                self.offset_acc[k] += inp.i_abc[k]
            self.calib_count += 1
            if self.calib_count == c.calib_periods:
                self.offset = [a / c.calib_periods for a in self.offset_acc]
            return OFF
        if self.undervoltage:
            if not inp.v_dc >= UV_RESUME * c.v_dc_min:
                return OFF
            self.undervoltage = False
        elif not inp.v_dc >= c.v_dc_min:        # also catches a NaN measurement
            self.undervoltage = True
            self.started = False                # the angle will have moved on: re-read it
            self.div_count = 0
            self.pi_d.reset()
            self.pi_q.reset()
            return OFF

        ia = inp.i_abc[0] - self.offset[0]
        ib = inp.i_abc[1] - self.offset[1]
        ic = inp.i_abc[2] - self.offset[2]
        th = inp.theta_e
        cs, sn = math.cos(th), math.sin(th)
        alpha, beta = clarke(ia, ib, ic)
        self.i_d, self.i_q = park(alpha, beta, cs, sn)

        if not self.started:
            self.started = True
            self.theta_prev = th
        self.div_count += 1
        if self.div_count >= c.speed_div:
            self.div_count = 0
            tw = c.speed_div * c.ts
            w_raw = wrap_pi(th - self.theta_prev) / (tw * c.pole_pairs)
            self.theta_prev = th
            self.speed_est += c.speed_alpha * (w_raw - self.speed_est)
            if c.speed_loop:
                step_max = c.accel_max * tw
                self.ref_ramped += clamp(inp.reference - self.ref_ramped, -step_max, step_max)
                self.i_q_ref = self.pi_w.step(self.ref_ramped - self.speed_est, -c.i_max, c.i_max)
        if not c.speed_loop:
            self.i_q_ref = clamp(inp.reference, -c.i_max, c.i_max)
        self.i_d_ref = 0.0

        we = c.pole_pairs * self.speed_est
        v_max = c.modulation_limit * inp.v_dc
        ff_d = -we * c.l * self.i_q
        ff_q = we * (c.l * self.i_d + c.psi_m)
        self.v_d = self.pi_d.step(self.i_d_ref - self.i_d, -v_max, v_max, ff_d)
        vq_lim = math.sqrt(max(v_max * v_max - self.v_d * self.v_d, 0.0))
        self.v_q = self.pi_q.step(self.i_q_ref - self.i_q, -vq_lim, vq_lim, ff_q)

        th_out = th + c.angle_comp * we * c.ts
        co, so = math.cos(th_out), math.sin(th_out)
        v_alpha, v_beta = inverse_park(self.v_d, self.v_q, co, so)
        duty = list(svpwm(v_alpha, v_beta, inp.v_dc))
        if c.dt_comp > 0.0:
            ref = inverse_clarke(*inverse_park(self.i_d_ref, self.i_q_ref, co, so))
            for k in range(3):
                duty[k] = clamp(duty[k] + c.dt_comp * clamp(ref[k] / c.dt_comp_band, -1.0, 1.0), 0.0, 1.0)
        return Outputs((duty[0], duty[1], duty[2]), (1, 1, 1))
