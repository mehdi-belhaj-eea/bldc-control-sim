"""Back-to-back tests of the C controller against the Python reference (MIL vs SIL)."""

import ctypes
import shutil
import subprocess

import numpy as np
import pytest

from bldc import csil
from bldc import presets as ps
from bldc.control import FocController, Inputs, SixStepController
from bldc.control.transforms import svpwm, wrap_pi
from bldc.motor import COMMUTATION
from bldc.plant import Plant
from bldc.sensors import CurrentSensor, Sensors
from bldc.sim import simulate


@pytest.mark.skipif(shutil.which("make") is None, reason="make not available")
def test_c_unit_tests_pass():
    r = subprocess.run(["make", "-s", "-C", str(csil.C_DIR), "test"], capture_output=True, text=True)
    assert r.returncode == 0, r.stdout + r.stderr
    assert "0 failures" in r.stdout


def test_building_blocks_match():
    lib = csil.library()                  # also checks the structure layouts
    rng = np.random.default_rng(7)
    duty = (ctypes.c_float * 3)()
    for _ in range(500):
        va, vb = rng.uniform(-35.0, 35.0, 2)
        lib.bldc_svpwm(va, vb, 48.0, duty)
        assert list(duty) == pytest.approx(svpwm(va, vb, 48.0), abs=2e-6)
        x = rng.uniform(-20.0, 20.0)
        assert lib.bldc_wrap_pi(x) == pytest.approx(wrap_pi(x), abs=1e-5)
    hi, lo = ctypes.c_int(), ctypes.c_int()
    for code in range(8):
        ok = lib.bldc_six_legs(code, ctypes.byref(hi), ctypes.byref(lo))
        assert bool(ok) == (code in COMMUTATION)
        if ok:
            assert (hi.value, lo.value) == COMMUTATION[code]


class Recorder:
    """Wraps a controller and records every input it receives."""

    def __init__(self, ctrl):
        self.ctrl, self.inputs = ctrl, []

    def step(self, inp):
        self.inputs.append(inp)
        return self.ctrl.step(inp)

    def telemetry(self):
        return self.ctrl.telemetry()


def as_float32(inp):
    """The inputs exactly as the C code receives them (single precision)."""
    f = lambda x: float(np.float32(x))   # noqa: E731
    return Inputs(tuple(f(i) for i in inp.i_abc), f(inp.v_dc), f(inp.theta_e), inp.hall, f(inp.reference))


def replay(ctrl, inputs, key):
    # Open-loop replay: a constant rounding difference on a reference is integrated by the next
    # PI without the plant to correct it, so the inputs are rounded to single precision first.
    duties, values = [], []
    for inp in map(as_float32, inputs):
        out = ctrl.step(inp)
        duties.append([d if e else 0.0 for d, e in zip(out.duty, out.enable)])
        values.append(ctrl.telemetry()[key])
    return np.array(duties), np.array(values)


def test_foc_same_outputs_on_the_same_inputs():
    m = ps.motor("sinusoidal")
    cfg = ps.foc_config(m)
    rec = Recorder(FocController(cfg))
    plant = Plant(m, v_dc=ps.V_DC, dead_time=ps.DEAD_TIME, load=lambda t, w: 0.6 if t > 0.12 else 0.0)
    sensors = Sensors(current=CurrentSensor(offset=(0.08, -0.04, 0.0)))
    simulate(plant, rec, 0.2, lambda t: ps.rpm(1500), sensors=sensors)
    d_py, iq_py = replay(FocController(cfg), rec.inputs, "i_q_ref")
    d_c, iq_c = replay(csil.CFocController(cfg), rec.inputs, "i_q_ref")
    # Single-precision rounding of the angle difference (≈2e-7 rad) makes the speed estimates
    # differ by ~1e-4 rad/s; without the plant, the speed and current integrators accumulate it
    # over the 4000 steps. A real discrepancy (sign, gain, order of operations) is 100× larger.
    assert np.max(np.abs(d_py - d_c)) < 1e-3
    assert np.max(np.abs(iq_py - iq_c)) < 2e-3
    first = slice(0, 1000)                      # before the drift builds up: within 1e-5
    assert np.max(np.abs(d_py[first] - d_c[first])) < 1e-5


def test_six_step_same_outputs_on_the_same_inputs():
    m = ps.motor("trapezoidal")
    cfg = ps.six_step_config(m)
    rec = Recorder(SixStepController(cfg))
    plant = Plant(m, v_dc=ps.V_DC, dead_time=ps.DEAD_TIME, load=lambda t, w: 0.4)
    simulate(plant, rec, 0.2, lambda t: ps.rpm(1500), sensors=Sensors())
    d_py, w_py = replay(SixStepController(cfg), rec.inputs, "speed_est")
    d_c, w_c = replay(csil.CSixStepController(cfg), rec.inputs, "speed_est")
    assert np.max(np.abs(d_py - d_c)) < 1e-4
    assert np.max(np.abs(w_py - w_c)) < 1e-3


@pytest.mark.parametrize("emf, kind", [("sinusoidal", "foc"), ("trapezoidal", "six")])
def test_closed_loop_python_and_c_agree(emf, kind):
    m = ps.motor(emf)
    runs = {}
    for impl in ("python", "c"):
        if kind == "foc":
            cfg = ps.foc_config(m)
            ctrl = FocController(cfg) if impl == "python" else csil.CFocController(cfg)
        else:
            cfg = ps.six_step_config(m)
            ctrl = SixStepController(cfg) if impl == "python" else csil.CSixStepController(cfg)
        plant = Plant(m, v_dc=ps.V_DC, dead_time=ps.DEAD_TIME, load=lambda t, w: 0.5 if t > 0.15 else 0.0)
        runs[impl] = simulate(plant, ctrl, 0.25, lambda t: ps.rpm(1200))
    a, b = runs["python"], runs["c"]
    # Not bit-identical in closed loop: single-precision rounding meets the quantised sensors
    # (ADC codes, encoder counts, Hall edges) and the trajectories drift apart slightly.
    assert np.max(np.abs(a.rpm - b.rpm)) < 0.0025 * 1200.0
    w = a.window(0.2, 0.25)
    assert np.mean(a.torque[w]) == pytest.approx(np.mean(b.torque[w]), rel=5e-3)


def test_c_refuses_an_invalid_configuration_and_a_missing_dc_link():
    lib = csil.library()
    n = lib.bldc_interface_layout(None, 0)
    c_layout = (ctypes.c_size_t * n)()
    lib.bldc_interface_layout(c_layout, n)
    assert list(c_layout) == csil.layout()          # sizes and field offsets
    cfg = csil._FocConfig(ts=0.0, speed_div=20, pole_pairs=4)          # bypasses the Python checks
    state = ctypes.create_string_buffer(lib.bldc_foc_size())
    assert lib.bldc_foc_init(state, ctypes.byref(cfg)) == -1
    out, tm = csil._Outputs(), csil._FocTelemetry()
    lib.bldc_foc_step(state, ctypes.byref(csil._Inputs(v_dc=48.0, reference=5.0)), ctypes.byref(out))
    lib.bldc_foc_telemetry(state, ctypes.byref(tm))
    assert list(out.enable) == [0, 0, 0] and tm.fault == 1
    m = ps.motor("sinusoidal")
    cfg = ps.foc_config(m, calib_periods=0, speed_loop=False)
    for ctrl in (FocController(cfg), csil.CFocController(cfg)):
        assert ctrl.step(Inputs((0.0, 0.0, 0.0), float("nan"), 0.0, 0, 2.0)).enable == (0, 0, 0)
        assert ctrl.step(Inputs((0.0, 0.0, 0.0), 48.0, 0.0, 0, 2.0)).enable == (1, 1, 1)


class Dropout:
    """Gives the controller a DC-link reading of zero between t0 and t1; the real bus stays."""

    def __init__(self, ctrl, t0, t1):
        self.ctrl, self.t0, self.t1, self.k = ctrl, t0, t1, 0

    def step(self, inp):
        t = self.k * ps.TS
        self.k += 1
        if self.t0 <= t < self.t1:
            inp = Inputs(inp.i_abc, 0.0, inp.theta_e, inp.hall, inp.reference)
        return self.ctrl.step(inp)

    def telemetry(self):
        return self.ctrl.telemetry()


@pytest.mark.parametrize("impl", ["python", "c"])
def test_foc_recovers_cleanly_from_a_dc_link_dropout(impl):
    # 5.3 ms without a DC-link reading at 1500 rpm under load: on recovery the speed estimate
    # restarts from a fresh angle instead of a difference spanning the whole lockout.
    m = ps.motor("sinusoidal")
    cfg = ps.foc_config(m)
    ctrl = FocController(cfg) if impl == "python" else csil.CFocController(cfg)
    plant = Plant(m, v_dc=ps.V_DC, dead_time=ps.DEAD_TIME, load=lambda t, w: 0.6 if t >= 0.25 else 0.0)
    res = simulate(plant, Dropout(ctrl, 0.30, 0.3053), 0.5, lambda t: ps.rpm(1500), telemetry=("speed_est",))
    after = res.t >= 0.3053
    assert np.min(res.telemetry["speed_est"][after]) > ps.rpm(1300.0)
    assert np.max(res.i_peak[after]) < 15.0
    assert res.rpm[res.window(0.45, 0.5)].mean() == pytest.approx(1500.0, abs=5.0)
