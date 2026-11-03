"""Example 2 — six-step versus FOC, on a sinusoidal and on a trapezoidal motor.

The same windings with two back-EMF shapes, each driven by the two C controllers at 1500 rpm
with a fan-type load of 0.6 N·m. Switched inverter with 0.5 µs dead time.

Run:  python examples/six_step_vs_foc.py
Writes docs/example_six_step_vs_foc.md and docs/img/six_step_vs_foc.png.
"""

from __future__ import annotations

import math
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
from bldc.sim import simulate  # noqa: E402

ROOT = pathlib.Path(__file__).resolve().parents[1]
IMG = ROOT / "docs" / "img"
IMG.mkdir(parents=True, exist_ok=True)
st.apply()

SPEED, T_FAN = 1500.0, 0.6
K_FAN = T_FAN / ps.rpm(SPEED) ** 2          # fan law T = k·ω², passive at standstill
T_END = 0.8
PER_CYCLE = int(round(ps.F_PWM / (SPEED / 60.0 * 4)))   # 200 samples per electrical period


def fan(t: float, w: float) -> float:
    return K_FAN * w * abs(w)


def excursions(tq: np.ndarray, band: float = 0.1) -> list[int]:
    """Lengths (in PWM periods) of the runs where the torque leaves ±band of its mean."""
    out, n = [], 0
    for x in np.abs(tq / np.mean(tq) - 1.0) > band:
        if x:
            n += 1
        elif n:
            out.append(n)
            n = 0
    return out


runs = {}
for emf in ("sinusoidal", "trapezoidal"):
    m = ps.motor(emf)
    for kind in ("FOC", "six-step"):
        ctrl = (csil.CFocController(ps.foc_config(m)) if kind == "FOC"
                else csil.CSixStepController(ps.six_step_config(m)))
        plant = Plant(m, v_dc=ps.V_DC, f_pwm=ps.F_PWM, dead_time=ps.DEAD_TIME, model="switched", load=fan)
        res = simulate(plant, ctrl, T_END, lambda t: ps.rpm(SPEED))
        w = res.window(T_END - 0.1, T_END)                # the last 10 electrical periods
        tq = res.torque[w]
        mean = np.mean(tq)
        h = an.harmonics(tq, PER_CYCLE, (3, 6))
        i_rms = math.sqrt(np.mean(res.i_abc[w] ** 2))
        runs[(emf, kind)] = dict(
            res=res, m=m, speed=np.mean(res.rpm[w]), torque=mean, ripple=an.ripple(tq),
            h3=h[3] / mean, h6=h[6] / mean, thd=an.thd(res.i_abc[w, 0], PER_CYCLE),
            i_rms=i_rms, p_cu=3.0 * m.r * i_rms ** 2, i_peak=float(np.max(res.i_peak[w])),
            i_typ=float(np.median(np.max(np.abs(res.i_abc[w]), axis=1))),
            i_amp=float(np.max(np.abs(res.i_abc[w]))), exc=excursions(tq))

report = [
    "# Example 2 — Six-step versus FOC", "",
    f"Both C controllers at {SPEED:.0f} rpm, fan load {T_FAN:.1f} N·m (plus friction), switched "
    "inverter with 0.5 µs dead time, on the same windings with a sinusoidal or a trapezoidal "
    "back-EMF (120° flat top). Figures over the last 10 electrical periods; the torque is the mean "
    "over each PWM period, its harmonics are in % of the mean torque; the peak current is the "
    "instantaneous peak, PWM ripple included.", "",
    "| Motor | Control | Ripple (peak-to-peak) | at 3·f_e | at 6·f_e | Phase current | Peak current | THD | Copper loss |",
    "| --- | --- | --- | --- | --- | --- | --- | --- | --- |",
]
for (emf, kind), r in runs.items():
    report.append(f"| {emf} | {kind} | {100 * r['ripple']:.0f} % | {100 * r['h3']:.1f} % | "
                  f"{100 * r['h6']:.1f} % | {r['i_rms']:.2f} A rms | {r['i_peak']:.1f} A | "
                  f"{100 * r['thd']:.0f} % | {r['p_cu']:.1f} W |")

trap_six, trap_foc = runs[("trapezoidal", "six-step")], runs[("trapezoidal", "FOC")]
sin_six, sin_foc = runs[("sinusoidal", "six-step")], runs[("sinusoidal", "FOC")]
m_t = trap_six["m"]
exc = trap_six["exc"]
report += [
    "",
    f"All runs deliver the same {trap_foc['torque']:.3f} N·m. What the table shows:", "",
    f"- **FOC on the sinusoidal motor** is the reference: {100 * sin_foc['ripple']:.0f} % of ripple, "
    "almost all of it from the speed loop (see example 1).",
    f"- **Six-step** draws block currents ({100 * trap_six['thd']:.0f} % THD). At each commutation "
    "the outgoing current decays through a diode while the incoming one rises; for "
    f"{int(np.median(exc))} to {max(exc)} PWM periods the torque leaves ±10 % of its mean. Six "
    f"commutations per electrical period give the {100 * trap_six['h6']:.1f} % line at 6·f_e "
    "(600 Hz here, audible). The current loop regulates the mean of the two conducting currents: "
    "regulating the switched one alone would saturate the loop at every other commutation, when "
    "the switched phase is the incoming one.",
    f"- **On the trapezoidal motor**, block currents need a lower level for the same torque: "
    f"{trap_six['i_typ']:.1f} A on the flat tops against a {trap_foc['i_amp']:.1f} A sinusoidal "
    f"amplitude with FOC ({m_t.k_t_six_step:.3f} against {m_t.k_t:.3f} N·m per ampere of peak). That "
    "is the classic argument for six-step: the inverter current rating is used best. But the "
    "commutation spikes and the PWM ripple bring the instantaneous peaks level "
    f"({trap_six['i_peak']:.1f} A against {trap_foc['i_peak']:.1f} A), and the rms current, so the "
    f"copper loss, is still lower with FOC ({trap_foc['p_cu']:.1f} W against {trap_six['p_cu']:.1f} W).",
    "- **Six-step on the sinusoidal motor** adds the ripple of the EMF itself: the torque "
    "follows the line EMF, which varies by 13 % over each 60° sector, and it costs the most "
    "copper loss.",
    f"- **FOC on the trapezoidal motor** regulates sinusoidal currents, but the 5th and 7th "
    "harmonics of the EMF push harmonic currents through the loop "
    f"({100 * trap_foc['thd']:.0f} % THD). With perfectly sinusoidal currents this EMF would give "
    f"6.0 % of ripple at 6·f_e; the loop's harmonic currents cancel part of it, "
    f"{100 * trap_foc['h6']:.1f} % remains.", "",
    "In short: six-step when cost and simplicity matter (three Hall sensors, no angle sensor, "
    "little computation), ideally on a trapezoidal motor; FOC for smooth torque, low noise and "
    "the lowest copper loss, ideally on a sinusoidal motor.", "",
    "During each six-step sector the phase left off is not always idle: with complementary PWM on "
    "the leg that switches, both conducting phases sit at the negative rail during the off-time, "
    "so the off phase sees its own EMF at its terminal. While that EMF is negative, its lower diode "
    "conducts a small current.", "",
    "The six-step speed loop is designed at a low speed, 600 rpm, because its speed comes from the "
    "Hall edges averaged over one electrical turn: half a turn of delay, 12.5 ms at 600 rpm. It "
    "keeps 34° of phase margin at 300 rpm, the bottom of its range, and stays stable up to 3000 rpm "
    "(`tests/test_control.py`).", "",
    "![Six-step versus FOC](img/six_step_vs_foc.png)", "",
]

fig, axes = plt.subplots(2, 2, figsize=(9.0, 5.6), sharex=True)
for col, emf in enumerate(("sinusoidal", "trapezoidal")):
    for kind, color in (("FOC", st.BLUE), ("six-step", st.ORANGE)):
        r = runs[(emf, kind)]
        res = r["res"]
        w = res.window(T_END - 0.02, T_END)               # two electrical periods
        t_ms = (res.t[w] - res.t[w][0]) * 1e3
        axes[0, col].plot(t_ms, res.torque[w], color=color, lw=1.0, label=kind)
        axes[1, col].plot(t_ms, res.i_abc[w, 0], color=color, lw=1.0, label=kind)
    axes[0, col].set_title(f"{emf} back-EMF")
    axes[1, col].set_xlabel("time (ms)")
axes[0, 0].set_ylabel("torque (N·m)")
axes[1, 0].set_ylabel("phase-a current (A)")
for ax in axes[0]:
    ax.set_ylim(0.0, 1.0)
axes[0, 0].legend(loc="lower right", ncol=2)
fig.suptitle(f"Six-step versus FOC at {SPEED:.0f} rpm, {T_FAN:.1f} N·m: torque and current over two electrical periods",
             fontsize=10)
fig.tight_layout()
fig.savefig(IMG / "six_step_vs_foc.png", dpi=150)
plt.close(fig)

(ROOT / "docs" / "example_six_step_vs_foc.md").write_text("\n".join(report), encoding="utf-8")
print("\n".join(report))
