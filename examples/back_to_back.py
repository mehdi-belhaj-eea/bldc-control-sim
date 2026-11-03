"""Example 4 — the C code against its Python reference (MIL vs SIL), and its size on a Cortex-M4.

MIL (model in the loop): the Python controller drives the simulated motor.
SIL (software in the loop): the C code, compiled for this computer, drives the same simulation.
Both run the scenario of example 1 and a six-step run; the C unit tests run, and the C sources
are compiled for a Cortex-M4F to report their code size.

Run:  python examples/back_to_back.py
Writes docs/example_back_to_back.md and docs/img/back_to_back.png.
"""

from __future__ import annotations

import pathlib
import re
import shutil
import subprocess

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

import plotstyle as st  # noqa: E402
from bldc import csil  # noqa: E402
from bldc import presets as ps  # noqa: E402
from bldc.control import FocController, Inputs, SixStepController  # noqa: E402
from bldc.plant import Plant  # noqa: E402
from bldc.sensors import CurrentSensor, Sensors  # noqa: E402
from bldc.sim import simulate  # noqa: E402

ROOT = pathlib.Path(__file__).resolve().parents[1]
IMG = ROOT / "docs" / "img"
IMG.mkdir(parents=True, exist_ok=True)
st.apply()


def foc_run(ctrl):
    m = ps.motor("sinusoidal")
    plant = Plant(m, v_dc=ps.V_DC, f_pwm=ps.F_PWM, dead_time=ps.DEAD_TIME, model="switched",
                  load=lambda t, w: 0.6 if t >= 0.25 else 0.0)
    sensors = Sensors(current=CurrentSensor(offset=(0.10, -0.05, 0.02)))
    return simulate(plant, ctrl, 0.6, lambda t: ps.rpm(1500.0) if t < 0.4 else ps.rpm(3000.0), sensors=sensors)


class Recorder:
    def __init__(self, ctrl):
        self.ctrl, self.inputs = ctrl, []

    def step(self, inp):
        self.inputs.append(inp)
        return self.ctrl.step(inp)

    def telemetry(self):
        return self.ctrl.telemetry()


m = ps.motor("sinusoidal")
cfg = ps.foc_config(m)
rec = Recorder(FocController(cfg))
mil = foc_run(rec)
sil = foc_run(csil.CFocController(cfg))
d_speed = sil.rpm - mil.rpm
d_iq = sil.i_dq[1] - mil.i_dq[1]
k_iq = int(np.argmax(np.abs(d_iq)))
w_end = mil.window(0.55, 0.6)
d_torque = abs(np.mean(sil.torque[w_end]) - np.mean(mil.torque[w_end])) / np.mean(mil.torque[w_end])

# open loop: the same recorded inputs, in single precision, to both controllers
f32 = lambda x: float(np.float32(x))   # noqa: E731
py_c, c_c = FocController(cfg), csil.CFocController(cfg)
dd = []
for inp in rec.inputs:
    x = Inputs(tuple(f32(i) for i in inp.i_abc), f32(inp.v_dc), f32(inp.theta_e), inp.hall, f32(inp.reference))
    a, b = py_c.step(x), c_c.step(x)
    dd.append(max(abs(p - q) for p, q in zip(a.duty, b.duty)))
dd = np.array(dd)
n50 = int(0.05 / ps.TS)

mt = ps.motor("trapezoidal")
fan = 0.6 / ps.rpm(1500.0) ** 2
six = {}
for name, ctrl in (("python", SixStepController(ps.six_step_config(mt))),
                   ("c", csil.CSixStepController(ps.six_step_config(mt)))):
    plant = Plant(mt, v_dc=ps.V_DC, dead_time=ps.DEAD_TIME, load=lambda t, w: fan * w * abs(w))
    six[name] = simulate(plant, ctrl, 0.6, lambda t: ps.rpm(1500.0))
d_six = np.max(np.abs(six["c"].rpm - six["python"].rpm))

# C unit tests and target build
unit, sizes, ram, libc = "not run (make not found)", None, None, []
if shutil.which("make"):
    out = subprocess.run(["make", "-s", "-C", str(csil.C_DIR), "test"], capture_output=True, text=True).stdout
    mm = re.search(r"(\d+) checks, (\d+) failures; state: FOC (\d+) bytes, six-step (\d+) bytes", out)
    if mm:
        unit = f"{mm.group(1)} checks, {mm.group(2)} failures"
        ram = (int(mm.group(3)), int(mm.group(4)))
    if shutil.which("arm-none-eabi-gcc"):
        out = subprocess.run(["make", "-s", "-C", str(csil.C_DIR), "arm"], capture_output=True, text=True).stdout
        sizes = {pathlib.Path(p).stem: int(t) for t, p in re.findall(r"^\s*(\d+)\s+\d+\s+\d+\s+\d+\s+\w+\s+(\S+\.o)$",
                                                                    out, flags=re.M)}
        if shutil.which("arm-none-eabi-nm"):         # external symbols: the C library calls
            objs = sorted((csil.C_DIR / "build" / "arm").glob("*.o"))
            nm = subprocess.run(["arm-none-eabi-nm", "-u", *map(str, objs)], capture_output=True, text=True).stdout
            libc = sorted({ln.split()[-1] for ln in nm.splitlines()
                           if ln.strip().startswith("U ") and not ln.split()[-1].startswith("bldc_")})

report = [
    "# Example 4 — The C code against its Python reference", "",
    "The control laws exist twice: in Python (`bldc/control/`), readable and used as the "
    "reference, and in C (`c/src/`), as it would be flashed into a microcontroller. The same "
    "simulation runs one or the other:", "",
    "- **MIL**, model in the loop: the Python controller drives the motor model;",
    "- **SIL**, software in the loop: the C code, compiled for this computer, drives the same model.", "",
    "The next rungs, PIL (the code running on the target processor) and HIL (the real controller "
    "board against a real-time motor model), need hardware.", "",
    "## Back-to-back results", "",
    "| Comparison | Result |", "| --- | --- |",
    f"| FOC, scenario of example 1, closed loop: largest speed gap | {np.max(np.abs(d_speed)):.2f} rpm "
    f"(up to 3000 rpm) |",
    f"| Same run: gap on the q current, rms (largest) | {np.sqrt(np.mean(d_iq ** 2)) * 1e3:.0f} mA "
    f"({np.max(np.abs(d_iq)) * 1e3:.0f} mA at t = {mil.t[k_iq] * 1e3:.0f} ms) |",
    f"| Same run: mean torque at 3000 rpm | {100 * d_torque:.3f} % apart |",
    f"| FOC, same inputs replayed open loop: duty gap over the first 50 ms | {np.max(dd[:n50]):.1e} |",
    f"| Same, over the whole 0.6 s | {np.max(dd):.1e} |",
    f"| Six-step, 1500 rpm, closed loop: largest speed gap | {d_six:.2f} rpm |",
    f"| C unit tests (`make test`) | {unit} |", "",
    "The two are not bit-identical: the C code computes in single precision, as a Cortex-M4F "
    "does, and the Python reference in double. On the same inputs the duty cycles agree to "
    f"{np.max(dd[:n50]):.0e}; replayed without the motor, the small rounding differences on the "
    "speed estimate accumulate in the integrators, which a real loop corrects. In closed loop the "
    "quantised sensors (ADC codes, encoder counts) turn these differences into slightly different "
    "trajectories, a fraction of an rpm apart. The largest current gap falls in the no-load part "
    "of the run, where i_q itself fluctuates by about 0.1 A rms (example 3).", "",
]
if sizes:
    total = sum(sizes.values())
    report += [
        "## Size on a Cortex-M4F", "",
        "`arm-none-eabi-gcc -Os -mcpu=cortex-m4 -mfloat-abi=hard -mfpu=fpv4-sp-d16`, objects only "
        "and without the layout table used by the Python bindings; the C library functions they "
        f"call ({', '.join(libc) if libc else 'see arm-none-eabi-nm -u'}) come on top.", "",
        "| Object | Code (bytes) |", "| --- | --- |",
        *[f"| {name}.o | {size} |" for name, size in sizes.items()],
        f"| **Total** | **{total}** |", "",
    ]
    if ram:
        report += [f"State in RAM: {ram[0]} bytes for the FOC controller, {ram[1]} bytes for "
                   "the six-step controller (no heap, no mutable global data).", ""]
report += ["![MIL versus SIL](img/back_to_back.png)", ""]

fig, axes = plt.subplots(2, 1, figsize=(8.0, 4.8), sharex=True, gridspec_kw={"height_ratios": [2, 1]})
axes[0].plot(mil.t, mil.rpm, color=st.BLUE, lw=2.2, label="Python reference (MIL)")
axes[0].plot(sil.t, sil.rpm, color=st.ORANGE, lw=1.0, label="C code (SIL)")
axes[0].set_ylabel("speed (rpm)")
axes[0].legend(loc="upper left")
axes[0].set_title("FOC scenario of example 1, run by the Python reference and by the C code")
axes[1].plot(mil.t, d_speed, color=st.INK2, lw=0.8)
axes[1].set_ylabel("C − Python (rpm)")
axes[1].set_xlabel("time (s)")
fig.tight_layout()
fig.savefig(IMG / "back_to_back.png", dpi=150)
plt.close(fig)

(ROOT / "docs" / "example_back_to_back.md").write_text("\n".join(report), encoding="utf-8")
print("\n".join(report))
