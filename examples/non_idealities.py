"""Example 3 — what the real hardware does to FOC: dead time, sensor errors, encoder resolution.

FOC (C controller) on the sinusoidal motor at 300 rpm (20 Hz electrical), 0.3 N·m. One
non-ideality at a time, the others removed (24-bit ADC, 2^20-line encoder, no dead time), and
the torque ripple split into its harmonics of the electrical frequency.

Run:  python examples/non_idealities.py
Writes docs/example_non_idealities.md, docs/img/non_idealities.png and docs/img/dead_time_current.png.
"""

from __future__ import annotations

import pathlib

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

import plotstyle as st  # noqa: E402
from bldc import analysis as an  # noqa: E402
from bldc import csil  # noqa: E402
from bldc import presets as ps  # noqa: E402
from bldc.plant import Plant  # noqa: E402
from bldc.sensors import CurrentSensor, Encoder, Sensors  # noqa: E402
from bldc.sim import simulate  # noqa: E402

ROOT = pathlib.Path(__file__).resolve().parents[1]
IMG = ROOT / "docs" / "img"
IMG.mkdir(parents=True, exist_ok=True)
st.apply()

SPEED, LOAD = 300.0, 0.3
PER_CYCLE = int(round(ps.F_PWM / (SPEED / 60.0 * 4)))        # 1000 samples per electrical period
T_END = 0.5
WINDOW = (T_END - 2 * PER_CYCLE * ps.TS, T_END)              # two electrical periods
m = ps.motor("sinusoidal")


def run(dead_time=0.0, dt_comp=False, offset=(0.0, 0.0, 0.0), calibrate=True, gain=(0.0, 0.0, 0.0),
        bits=24, lines=1 << 20):
    cfg = ps.foc_config(m, dt_comp=(ps.DEAD_TIME / ps.TS if dt_comp else 0.0),
                        calib_periods=200 if calibrate else 0)
    plant = Plant(m, v_dc=ps.V_DC, f_pwm=ps.F_PWM, dead_time=dead_time, model="switched",
                  load=lambda t, w: LOAD if t >= 0.1 else 0.0)
    sensors = Sensors(current=CurrentSensor(offset=offset, gain_error=gain, bits=bits),
                      encoder=Encoder(lines=lines))
    res = simulate(plant, csil.CFocController(cfg), T_END, lambda t: ps.rpm(SPEED), sensors=sensors)
    w = res.window(*WINDOW)
    tq = res.torque[w]
    h = an.harmonics(tq, PER_CYCLE, (1, 2, 6))
    mean = float(np.mean(tq))
    return dict(res=res, w=w, mean=mean, ripple=an.ripple(tq), h={k: v / mean for k, v in h.items()},
                thd=an.thd(res.i_abc[w, 0], PER_CYCLE))


OFFSETS = (0.10, -0.05, 0.0)
cases = [
    ("Reference: no dead time, ideal sensors", {}),
    ("Dead time 0.5 µs, not compensated", dict(dead_time=ps.DEAD_TIME)),
    ("Dead time 0.5 µs, compensated", dict(dead_time=ps.DEAD_TIME, dt_comp=True)),
    ("Offsets +100 / −50 mA, no calibration", dict(offset=OFFSETS, calibrate=False)),
    ("Offsets +100 / −50 mA, calibrated at start-up", dict(offset=OFFSETS)),
    ("Gain error +1 % on phase a", dict(gain=(0.01, 0.0, 0.0))),
    ("12-bit ADC (16 mA steps)", dict(bits=12)),
    ("Encoder 4096 lines", dict(lines=4096)),
    ("Encoder 1024 lines", dict(lines=1024)),
]
results = [(name, run(**kw)) for name, kw in cases]


def no_load(dead_time, comp):
    """Torque ripple at 1500 rpm without load, default sensors (12-bit ADC, 4096-line encoder)."""
    cfg = ps.foc_config(m, dt_comp=(ps.DEAD_TIME / ps.TS if comp else 0.0))
    plant = Plant(m, v_dc=ps.V_DC, f_pwm=ps.F_PWM, dead_time=dead_time, model="switched")
    res = simulate(plant, csil.CFocController(cfg), 0.3, lambda t: ps.rpm(1500.0))
    w = res.window(0.2, 0.3)
    return float(np.ptp(res.torque[w])), float(np.std(res.i_dq[1][w]))


light = [("No dead time", no_load(0.0, False)),
         ("Dead time 0.5 µs, not compensated", no_load(ps.DEAD_TIME, False)),
         ("Dead time 0.5 µs, compensated", no_load(ps.DEAD_TIME, True))]

report = [
    "# Example 3 — Non-idealities and their signature in the torque", "",
    f"FOC with the C controller on the sinusoidal motor at {SPEED:.0f} rpm (electrical frequency "
    f"f_e = {SPEED / 60 * 4:.0f} Hz), {LOAD:.1f} N·m. Each case adds one imperfection to an ideal "
    "drive. Torque harmonics in % of the mean torque, over two electrical periods.", "",
    "| Case | Ripple (peak-to-peak) | at f_e | at 2·f_e | at 6·f_e | Current THD |",
    "| --- | --- | --- | --- | --- | --- |",
]
for name, r in results:
    report.append(f"| {name} | {100 * r['ripple']:.2f} % | {100 * r['h'][1]:.2f} % | "
                  f"{100 * r['h'][2]:.2f} % | {100 * r['h'][6]:.2f} % | {100 * r['thd']:.2f} % |")

R = dict(results)
dt_off, dt_on = R["Dead time 0.5 µs, not compensated"], R["Dead time 0.5 µs, compensated"]
off_raw, off_cal = R["Offsets +100 / −50 mA, no calibration"], R["Offsets +100 / −50 mA, calibrated at start-up"]
gain = R["Gain error +1 % on phase a"]
enc4, enc1 = R["Encoder 4096 lines"], R["Encoder 1024 lines"]
report += [
    "",
    "Each defect leaves its own signature:", "",
    "- **Dead time**: while both switches of a leg are off, the diode sets the voltage, so each leg "
    f"loses t_d/T_s·V_dc = {ps.DEAD_TIME / ps.TS * ps.V_DC:.2f} V against its current: a square-wave "
    "error that changes sign with the current. At low speed the motor needs only a few volts, so "
    "the error is large in relative terms: 5th and 7th current harmonics, a 6·f_e torque ripple of "
    f"{100 * dt_off['h'][6]:.2f} %. "
    f"The compensation adds back the lost duty according to the sign of the current: "
    f"{100 * dt_on['h'][6]:.2f} %. The sign comes from the current reference turned into phase "
    "currents, not from the measured current (see below).",
    f"- **Current offsets** are a DC error in the stator frame, hence an error at f_e in the rotor "
    f"frame: {100 * off_raw['h'][1]:.2f} % of ripple at f_e. Measuring them with the legs off at "
    f"start-up brings it below {max(100 * off_cal['h'][1], 0.01):.2f} %; drives do it at every power-up.",
    f"- **A gain mismatch** between phases makes the measured current vector elliptical: ripple at "
    f"2·f_e ({100 * gain['h'][2]:.2f} % for 1 %). It is removed by calibrating the gains, not the offsets.",
    f"- **Encoder resolution**: the speed is the angle difference over 1 ms, so it moves in steps of "
    f"one count; the speed PI turns each step into a step of current. {100 * enc4['ripple']:.0f} % of "
    f"peak-to-peak ripple with 4096 lines, {100 * enc1['ripple']:.0f} % with 1024 lines, spread over "
    "all frequencies. The cures: more counts, a longer speed window (more delay), or an observer.", "",
    "![Torque ripple by harmonic](img/non_idealities.png)", "",
    "## Dead-time compensation at no load", "",
    "At 1500 rpm without load the current is small and its PWM ripple (about ±0.3 A) crosses zero "
    "every period, so the dead-time error mostly averages out. A compensation driven by the "
    "measured current would then act as a negative resistance and amplify the noise; driven by "
    "the reference, it only follows the speed-loop noise. Torque ripple with the default sensors:", "",
    "| Case | Torque peak-to-peak | i_q standard deviation |", "| --- | --- | --- |",
    *[f"| {name} | {1e3 * ptp:.0f} mN·m | {1e3 * sd:.0f} mA |" for name, (ptp, sd) in light],
    "",
    "The compensation still costs a little at no load: about 1 % of the rated torque, against the "
    "large gain under load at low speed. Drives often fade it out at very light load; the band of "
    "±0.3 A, about half the PWM current ripple, already does part of that.", "",
    "![Dead time and phase current](img/dead_time_current.png)", "",
]

# ----------------------------------------------------------------- figures
labels = [name for name, _ in results]
y = np.arange(len(results))[::-1]
height = 0.26
fig, ax = plt.subplots(figsize=(9.0, 5.2))
for k, (order, color, lab) in enumerate(((1, st.BLUE, "at f_e"), (2, st.ORANGE, "at 2·f_e"),
                                         (6, st.AQUA, "at 6·f_e"))):
    vals = [100 * r["h"][order] for _, r in results]
    ax.barh(y + (1 - k) * height, vals, height * 0.9, color=color, label=lab)
ax.set_yticks(y)
ax.set_yticklabels(labels, fontsize=8)
ax.set_xlabel("torque ripple (% of the mean torque)")
ax.set_title(f"FOC at {SPEED:.0f} rpm, {LOAD:.1f} N·m: torque harmonics caused by each imperfection")
ax.legend(loc="lower right")
ax.grid(axis="y", visible=False)
fig.tight_layout()
fig.savefig(IMG / "non_idealities.png", dpi=150)
plt.close(fig)

fig, ax = plt.subplots(figsize=(7.5, 3.6))
for name, style in (("Reference: no dead time, ideal sensors", dict(**st.REF)),
                    ("Dead time 0.5 µs, not compensated", dict(color=st.ORANGE)),
                    ("Dead time 0.5 µs, compensated", dict(color=st.BLUE))):
    r = R[name]
    res, w = r["res"], r["w"]
    t_ms = (res.t[w] - res.t[w][0]) * 1e3
    ax.plot(t_ms[:PER_CYCLE], res.i_abc[w, 0][:PER_CYCLE], label=name.split(":")[0].lower()
            if name.startswith("Reference") else name.replace("Dead time 0.5 µs, ", "dead time, "), **style)
ax.set_xlabel("time (ms)")
ax.set_ylabel("phase-a current (A)")
ax.set_title(f"Dead time at {SPEED:.0f} rpm: a kink each time a phase current crosses zero")
ax.legend(loc="upper right")
fig.tight_layout()
fig.savefig(IMG / "dead_time_current.png", dpi=150)
plt.close(fig)

(ROOT / "docs" / "example_non_idealities.md").write_text("\n".join(report), encoding="utf-8")
print("\n".join(report))
