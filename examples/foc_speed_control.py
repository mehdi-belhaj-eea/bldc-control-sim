"""Example 1 — FOC speed control of the 48 V motor, with the C controller in the loop.

Switched inverter with 0.5 µs dead time (compensated), current sensors with offsets (removed by
the start-up calibration), 4096-line encoder. Scenario: ramp to 1500 rpm, 0.6 N·m load step,
then 3000 rpm. A second run checks the current loop alone against its design, on a rotor
turning at 1500 rpm, with and without the decoupling.

Run:  python examples/foc_speed_control.py
Writes docs/example_foc.md, docs/img/foc_speed_response.png and docs/img/foc_current_step.png.
"""

from __future__ import annotations

import math
import pathlib

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from scipy import signal  # noqa: E402

import plotstyle as st  # noqa: E402
from bldc import analysis as an  # noqa: E402
from bldc import csil  # noqa: E402
from bldc import presets as ps  # noqa: E402
from bldc.motor import Motor  # noqa: E402
from bldc.plant import Plant  # noqa: E402
from bldc.sensors import CurrentSensor, Encoder, Sensors  # noqa: E402
from bldc.sim import simulate  # noqa: E402

ROOT = pathlib.Path(__file__).resolve().parents[1]
IMG = ROOT / "docs" / "img"
IMG.mkdir(parents=True, exist_ok=True)
st.apply()

m = ps.motor("sinusoidal")
cur, spd = ps.foc_gains(m)
cfg = ps.foc_config(m)
OFFSETS = (0.10, -0.05, 0.02)
T_LOAD, T_STEP_LOAD, T_STEP_SPEED, T_END = 0.6, 0.25, 0.40, 0.60


def reference(t: float) -> float:
    return ps.rpm(1500.0) if t < T_STEP_SPEED else ps.rpm(3000.0)


def load(t: float, w: float) -> float:
    return T_LOAD if t >= T_STEP_LOAD else 0.0


# ----------------------------------------------------------------- speed control run
plant = Plant(m, v_dc=ps.V_DC, f_pwm=ps.F_PWM, dead_time=ps.DEAD_TIME, model="switched", load=load)
sensors = Sensors(current=CurrentSensor(offset=OFFSETS))
ctrl = csil.CFocController(cfg)
res = simulate(plant, ctrl, T_END, reference, sensors=sensors,
               telemetry=("i_q_ref", "ref_ramped", "v_d", "v_q"))
i_d, i_q = res.i_dq
rpm = res.rpm

offset_err = max(abs(a - b) for a, b in zip(ctrl.offset, OFFSETS))
w_before = res.window(0.20, T_STEP_LOAD)
w_load = res.window(T_STEP_LOAD, T_STEP_SPEED)
w_steady = res.window(0.35, T_STEP_SPEED)            # 5 electrical periods at 100 Hz
w_end = res.window(0.55, T_END)
overshoot = np.max(rpm[res.window(0.05, T_STEP_LOAD)]) - 1500.0
dip = 1500.0 - np.min(rpm[w_load])
k_min = np.nonzero(w_load)[0][np.argmin(rpm[w_load])]
back = np.nonzero(np.abs(rpm[k_min:np.nonzero(w_load)[0][-1]] - 1500.0) < 15.0)[0]
recovery = res.t[k_min + back[0]] - T_STEP_LOAD if len(back) else float("nan")
tq = res.torque[w_steady]
ripple = an.ripple(tq)
h = an.harmonics(tq, 200, (1, 2, 6))
thd = an.thd(res.i_abc[w_steady, 0], 200)
step_w = 2.0 * math.pi / (4 * sensors.encoder.lines) / (cfg.speed_div * cfg.ts)   # rad/s per count
step_iq = cfg.kp_w * cfg.speed_alpha * step_w
v_mag = np.hypot(res.telemetry["v_d"][w_end], res.telemetry["v_q"][w_end])
modulation = np.mean(v_mag) / (ps.V_DC / math.sqrt(3.0))
i_rms = math.sqrt(np.mean(res.i_abc[w_steady] ** 2))

# ----------------------------------------------------------------- current-loop step at 1500 rpm
# The rotor speed is held by a large inertia; ideal sensors, so that only the loop is seen.
spin = Motor(r=m.r, l=m.l, psi_m=m.psi_m, pole_pairs=m.pole_pairs, j=1e9)
I_STEP, T0, STEP_RPM = 5.0, 20e-3, 1500.0


def current_step(**overrides):
    plant2 = Plant(spin, v_dc=ps.V_DC, f_pwm=ps.F_PWM, model="switched", omega_m0=ps.rpm(STEP_RPM))
    ctrl2 = csil.CFocController(ps.foc_config(spin, speed_loop=False, calib_periods=0, dt_comp=0.0,
                                              **overrides))
    r = simulate(plant2, ctrl2, T0 + 3e-3, lambda t: I_STEP if t >= T0 else 0.0,
                 sensors=Sensors(current=CurrentSensor(bits=24), encoder=Encoder(lines=1 << 20)))
    return r, r.t >= T0


res2, w2 = current_step()
id2, iq2 = res2.i_dq
res3, w3 = current_step(l=0.0)                  # the same without the d-q decoupling
id3, _ = res3.i_dq
met = an.step_metrics(res2.t[w2] - T0, iq2[w2], 0.0, I_STEP)
t_sig = cur.t_sigma
design = signal.TransferFunction([1.0], [2.0 * t_sig ** 2, 2.0 * t_sig, 1.0])
t_design = np.linspace(0.0, 2e-3, 2001)
_, y_design = signal.step(design, T=t_design)
met_design = an.step_metrics(t_design, y_design, 0.0, 1.0)

# ----------------------------------------------------------------- report
report = [
    "# Example 1 — FOC speed control with the C controller in the loop", "",
    "Generic 48 V motor (invented values): R = 0.12 Ω, L = 0.25 mH, ψ_m = 16 mWb, 4 pole pairs, "
    "J = 5·10⁻⁴ kg·m² with its load. Inverter at 20 kHz, 0.5 µs dead time. The controller is the "
    "C code of `c/src/foc.c`, compiled and called once per PWM period.", "",
    "## Loop design", "",
    "| Loop | Method | kp | ki | Crossover | Phase margin |",
    "| --- | --- | --- | --- | --- | --- |",
    f"| Current (d and q) | Modulus optimum, T_Σ = 1.5·T_s = {cur.t_sigma * 1e6:.0f} µs | {cur.kp:.3f} V/A | "
    f"{cur.ki:.0f} V/(A·s) | {cur.omega_c:.0f} rad/s ({cur.omega_c / (2 * math.pi):.0f} Hz) | "
    f"{cur.phase_margin_deg:.1f}° |",
    f"| Speed | Symmetrical optimum, a = 3, T_σ = {spd.t_sigma * 1e3:.2f} ms | {spd.kp:.3f} A·s/rad | "
    f"{spd.ki:.1f} A/rad | {spd.omega_c:.0f} rad/s ({spd.omega_c / (2 * math.pi):.1f} Hz) | "
    f"{spd.phase_margin_deg:.1f}° |", "",
    "T_σ of the speed loop: closed current loop (2·T_Σ), speed averaged over the 1 ms speed period "
    "and held for the next one (T_ω), first-order speed filter (T_ω again).", "",
    f"## Current loop alone (5 A step on i_q, rotor at {STEP_RPM:.0f} rpm)", "",
    "| | Overshoot | Time to 90 % | Settling (±2 %) |", "| --- | --- | --- | --- |",
    f"| Design (continuous model) | {100 * met_design['overshoot']:.1f} % | {met_design['t90'] * 1e6:.0f} µs | "
    f"{met_design['settling_time'] * 1e6:.0f} µs |",
    f"| Simulation (C code, switched inverter) | {100 * met['overshoot']:.1f} % | {met['t90'] * 1e6:.0f} µs | "
    f"{met['settling_time'] * 1e6:.0f} µs |", "",
    "The overshoot matches. The simulated current starts later and then rises faster: the real "
    "delay is a pure delay of one period plus the PWM update, which the design approximates by a "
    "first-order lag of 1.5·T_s.", "",
    f"At {STEP_RPM:.0f} rpm the axes are coupled by ω_e·L (a q current induces a d voltage): during the "
    f"step, i_d moves by {1e3 * np.max(np.abs(id2[w2])):.0f} mA with the decoupling feed-forward, "
    f"{1e3 * np.max(np.abs(id3[w3])):.0f} mA without it.", "",
    "![Current step](img/foc_current_step.png)", "",
    "## Speed control", "",
    "| Check | Result |", "| --- | --- |",
    f"| Current offsets {' / '.join(f'{x * 1e3:+.0f}'.replace('-', '−') for x in OFFSETS)} mA, "
    f"start-up calibration (10 ms, legs off) | largest residual {offset_err * 1e3:.1f} mA "
    f"(ADC step: {sensors.current.lsb * 1e3:.1f} mA) |",
    f"| Overshoot at the end of the ramp to 1500 rpm | {overshoot:.1f} rpm |",
    f"| Mean speed before the load step | {np.mean(rpm[w_before]):.1f} rpm |",
    f"| Dip after the {T_LOAD:.1f} N·m load step | {dip:.0f} rpm, back within 1 % in {recovery * 1e3:.0f} ms |",
    f"| Torque ripple at 1500 rpm, {T_LOAD:.1f} N·m (peak-to-peak / mean) | {100 * ripple:.1f} % |",
    f"| Torque harmonics at f_e, 2·f_e, 6·f_e | {h[1] * 1e3:.1f}, {h[2] * 1e3:.1f}, {h[6] * 1e3:.1f} mN·m |",
    f"| Phase-current THD at 1500 rpm | {100 * thd:.1f} % ({i_rms:.2f} A rms) |",
    f"| Mean speed at 3000 rpm, {T_LOAD:.1f} N·m | {np.mean(rpm[w_end]):.1f} rpm |",
    f"| Voltage used at 3000 rpm | {100 * modulation:.0f} % of the linear limit V_dc/√3 |", "",
    f"The {overshoot:.0f} rpm overshoot at the end of the ramp is the price of the symmetrical "
    "optimum, tuned for load steps: its integrator keeps the speed on the ramp and carries on when "
    "the ramp stops. A filter on the speed reference would remove it.", "",
    "The remaining torque ripple comes mostly from the speed loop: over the 1 ms speed period the "
    f"16384-count encoder gives the speed in steps of {step_w * 30 / math.pi:.1f} rpm "
    f"({100 * step_w / ps.rpm(1500.0):.2f} % at 1500 rpm), and the filter and the speed PI turn "
    f"each step into a step of about {step_iq:.2f} A of q current "
    f"({step_iq * m.k_t * 1e3:.0f} mN·m). Example 3 shows how it grows with a coarser encoder.", "",
    "![Speed control](img/foc_speed_response.png)", "",
]

# ----------------------------------------------------------------- figures
fig, axes = plt.subplots(3, 1, figsize=(8.0, 6.6), sharex=True)
ax = axes[0]
ax.plot(res.t, res.telemetry["ref_ramped"] * 30.0 / math.pi, **st.REF, label="reference (ramped)")
ax.plot(res.t, rpm, color=st.BLUE, label="speed")
ax.set_ylabel("speed (rpm)")
ax.legend(loc="lower right")
ax.annotate(f"{T_LOAD:.1f} N·m load step\n−{dip:.0f} rpm", xy=(res.t[k_min], rpm[k_min]),
            xytext=(T_STEP_LOAD + 0.03, 700), fontsize=8, color=st.INK2,
            arrowprops=dict(arrowstyle="-", color=st.MUTED, lw=0.8))
ax = axes[1]
ax.plot(res.t, res.torque, color=st.BLUE, lw=1.0, label="motor torque (mean per PWM period)")
ax.plot(res.t, res.load + m.b * res.speed, color=st.ORANGE, label="load + friction")
ax.set_ylabel("torque (N·m)")
ax.legend(loc="upper left")
ax = axes[2]
ax.plot(res.t, i_q, color=st.BLUE, lw=1.0, label="i_q")
ax.plot(res.t, i_d, color=st.ORANGE, lw=1.0, label="i_d")
ax.set_ylabel("current (A)")
ax.set_xlabel("time (s)")
ax.legend(loc="upper left")
axes[0].set_title("FOC speed control, C controller in the loop: 1500 rpm, load step, 3000 rpm")
fig.tight_layout()
fig.savefig(IMG / "foc_speed_response.png", dpi=150)
plt.close(fig)

fig, ax = plt.subplots(figsize=(7.0, 3.6))
ax.plot(t_design * 1e6, I_STEP * y_design, **st.REF, label="design (modulus optimum)")
ax.step((res2.t[w2] - T0) * 1e6, iq2[w2], where="post", color=st.BLUE, label="i_q")
ax.step((res2.t[w2] - T0) * 1e6, id2[w2], where="post", color=st.ORANGE, label="i_d, with decoupling")
ax.step((res3.t[w3] - T0) * 1e6, id3[w3], where="post", color=st.AQUA, label="i_d, without decoupling")
ax.set_xlim(0, 1500)
ax.set_xlabel("time after the step (µs)")
ax.set_ylabel("current (A)")
ax.set_title(f"Current loop at {STEP_RPM:.0f} rpm: {I_STEP:.0f} A step, overshoot "
             f"{100 * met['overshoot']:.1f} % (design {100 * met_design['overshoot']:.1f} %)")
ax.legend(loc="center right")
fig.tight_layout()
fig.savefig(IMG / "foc_current_step.png", dpi=150)
plt.close(fig)

(ROOT / "docs" / "example_foc.md").write_text("\n".join(report), encoding="utf-8")
print("\n".join(report))
