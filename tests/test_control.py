import math

import numpy as np
import pytest

from bldc import analysis as an
from bldc import presets as ps
from bldc.control import FocController, SixStepController, tuning
from bldc.control.pi import PI
from bldc.control.transforms import SQRT3, clarke, inverse_clarke, inverse_park, park, svpwm, wrap_pi
from bldc.motor import Motor
from bldc.plant import Plant
from bldc.sensors import CurrentSensor, Sensors
from bldc.sim import simulate


def test_transforms_round_trip_and_amplitude():
    rng = np.random.default_rng(3)
    for _ in range(50):
        d, q, th = rng.normal(), rng.normal(), rng.uniform(0, 2 * math.pi)
        a, b, c = inverse_clarke(*inverse_park(d, q, math.cos(th), math.sin(th)))
        assert a + b + c == pytest.approx(0.0, abs=1e-12)
        assert max(abs(a), abs(b), abs(c)) <= math.hypot(d, q) + 1e-12      # amplitude invariant
        d2, q2 = park(*clarke(a, b, c), math.cos(th), math.sin(th))
        assert (d2, q2) == pytest.approx((d, q))
    assert wrap_pi(3.5 * math.pi) == pytest.approx(-0.5 * math.pi)
    assert wrap_pi(math.pi) == pytest.approx(math.pi)


def test_svpwm_linear_range():
    # The circle of radius V_dc/√3 is inscribed in the hexagon of the inverter voltages: the
    # duty spread reaches 1 where it touches a side, and stays below elsewhere.
    v_dc = 48.0
    assert svpwm(0.0, 0.0, v_dc) == pytest.approx((0.5, 0.5, 0.5))
    spreads = []
    for th in np.linspace(0.0, 2 * math.pi, 721):
        amp = v_dc / SQRT3
        va, vb = amp * math.cos(th), amp * math.sin(th)
        d = svpwm(va, vb, v_dc)
        spreads.append(max(d) - min(d))
        assert 0.5 * (max(d) + min(d)) == pytest.approx(0.5)      # centred: equal zero vectors
        a, b, c = inverse_clarke(va, vb)
        assert (d[0] - d[1]) * v_dc == pytest.approx(a - b)        # line voltages are exact
        assert (d[1] - d[2]) * v_dc == pytest.approx(b - c)
        over = svpwm(1.2 * va, 1.2 * vb, v_dc)
        assert all(0.0 <= x <= 1.0 for x in over)
    assert max(spreads) == pytest.approx(1.0) and min(spreads) == pytest.approx(math.sqrt(3) / 2)
    assert all(s <= 1.0 + 1e-12 for s in spreads)


def test_pi_anti_windup():
    pi = PI(kp=1.0, ki=100.0, ts=1e-3)
    for _ in range(1000):                       # long saturation with a positive error
        u = pi.step(5.0, -1.0, 1.0)
    assert u == 1.0 and pi.integ <= 1.0
    u = pi.step(-0.5, -1.0, 1.0)                # the error reverses: out of saturation at once
    assert u < 1.0
    pi2 = PI(kp=2.0, ki=10.0, ts=1e-3)          # unsaturated: plain PI
    assert pi2.step(1.0, -10.0, 10.0, ff=0.5) == pytest.approx(2.0 + 0.01 + 0.5)


def test_tuning_formulas():
    g = tuning.current_loop(r=0.12, l=0.25e-3, ts=50e-6)
    assert g.kp == pytest.approx(0.25e-3 / (2 * 75e-6))
    assert g.ki == pytest.approx(0.12 / (2 * 75e-6))
    assert g.phase_margin_deg == pytest.approx(90.0 - math.degrees(0.5))
    s = tuning.speed_loop(j=5e-4, k_t=0.096, t_sigma=2e-3, a=3.0)
    assert s.phase_margin_deg == pytest.approx(math.degrees(math.atan(8.0 / 6.0)))
    assert s.ki == pytest.approx(s.kp / (9.0 * 2e-3))
    with pytest.raises(ValueError):
        tuning.speed_loop(5e-4, 0.096, 2e-3, a=1.0)


def test_foc_current_step_follows_the_modulus_optimum():
    # Torque mode on a locked rotor: the q current answers a 5 A step as designed. The modulus
    # optimum predicts 4.3 % overshoot and a settling time (2 %) of 8.4·T_Σ = 0.63 ms.
    m = Motor(r=0.12, l=0.25e-3, psi_m=0.016, pole_pairs=4, j=1e9)
    cfg = ps.foc_config(m, speed_loop=False, calib_periods=0, dt_comp=0.0)
    plant = Plant(m, v_dc=48.0, model="switched", theta_m0=0.3)
    sensors = Sensors(current=CurrentSensor(bits=24))
    res = simulate(plant, FocController(cfg), 0.004, lambda t: 5.0 if t >= 0.001 else 0.0, sensors=sensors)
    w = res.t >= 0.001
    i_d, i_q = res.i_dq
    met = an.step_metrics(res.t[w] - 0.001, i_q[w], 0.0, 5.0)
    assert met["overshoot"] == pytest.approx(0.043, abs=0.015)
    assert met["t90"] < 0.30e-3
    assert met["settling_time"] < 0.65e-3
    assert np.max(np.abs(i_d[w])) < 0.2
    assert np.mean(i_q[res.t > 0.003]) == pytest.approx(5.0, rel=2e-3)


def test_foc_speed_loop_with_load_step_and_offset_calibration():
    m = ps.motor("sinusoidal")
    plant = Plant(m, v_dc=ps.V_DC, dead_time=ps.DEAD_TIME, model="average",
                  load=lambda t, w: 0.6 if t >= 0.25 else 0.0)
    sensors = Sensors(current=CurrentSensor(offset=(0.10, -0.05, 0.02)))
    ctrl = FocController(ps.foc_config(m))
    res = simulate(plant, ctrl, 0.4, lambda t: ps.rpm(1500), sensors=sensors)
    lsb = sensors.current.lsb
    assert ctrl.offset == pytest.approx([0.10, -0.05, 0.02], abs=0.5 * lsb + 1e-12)
    w = res.window(0.35, 0.4)
    assert res.rpm[w].mean() == pytest.approx(1500.0, abs=1.0)
    assert res.torque[w].mean() == pytest.approx(0.6 + m.b * ps.rpm(1500), rel=0.01)
    dip = 1500.0 - res.rpm[res.t >= 0.25].min()
    assert 55.0 < dip < 69.0          # linear symmetrical-optimum model: 62 rpm


def test_six_step_speed_control():
    m = ps.motor("trapezoidal")
    plant = Plant(m, v_dc=ps.V_DC, dead_time=ps.DEAD_TIME, model="switched", load=lambda t, w: 0.4)
    ctrl = SixStepController(ps.six_step_config(m))
    # the speed loop is designed at 600 rpm, the slow end of its range: it settles in ~0.5 s here
    res = simulate(plant, ctrl, 0.9, lambda t: ps.rpm(1500), telemetry=("i_ref",))
    w = res.window(0.8, 0.9)
    assert res.rpm[w].mean() == pytest.approx(1500.0, rel=5e-3)
    assert res.torque[w].mean() == pytest.approx(0.4 + m.b * ps.rpm(1500), rel=0.02)
    # regulating the mean of the two conducting currents keeps the commutations smooth
    # (regulating the switched leg alone gives about 0.9 here)
    assert an.ripple(res.torque[w]) < 0.6
    # one leg is off at any time: its current is small except during commutation
    i = np.abs(res.i_abc[w])
    smallest = np.sort(i, axis=1)[:, 0]
    assert np.median(smallest) < 0.1 * np.median(i.max(axis=1))


def test_six_step_stops_on_an_invalid_hall_code():
    m = ps.motor("trapezoidal")
    ctrl = SixStepController(ps.six_step_config(m, calib_periods=0))
    from bldc.control.common import Inputs
    out = ctrl.step(Inputs((0.0, 0.0, 0.0), 48.0, 0.0, 7, 100.0))
    assert out.enable == (0, 0, 0)
    out = ctrl.step(Inputs((0.0, 0.0, 0.0), 48.0, 0.0, 5, 100.0))
    assert out.enable == (1, 1, 0)


SPIN = Motor(r=0.12, l=0.25e-3, psi_m=0.016, pole_pairs=4, j=1e9)     # speed held by the inertia


def q_step_at_speed(rpm, **overrides):
    """Peak |i_d| after a 5 A step on i_q, rotor turning at a constant speed."""
    from bldc.sensors import Encoder
    cfg = ps.foc_config(SPIN, speed_loop=False, calib_periods=0, dt_comp=0.0, **overrides)
    plant = Plant(SPIN, v_dc=48.0, omega_m0=ps.rpm(rpm))
    sensors = Sensors(current=CurrentSensor(bits=24), encoder=Encoder(lines=1 << 20))
    res = simulate(plant, FocController(cfg), 0.03, lambda t: 5.0 if t >= 0.02 else 0.0, sensors=sensors)
    i_d, _ = res.i_dq
    return float(np.max(np.abs(i_d[res.t >= 0.02])))


def test_decoupling_and_angle_compensation_at_speed():
    # At 3000 rpm the d and q axes are coupled by ω·L and the rotor turns 5.4° during the delay.
    nominal = q_step_at_speed(3000)
    assert nominal < 0.4
    assert q_step_at_speed(3000, l=0.0) > 2.0 * nominal            # no L decoupling
    assert q_step_at_speed(3000, angle_comp=0.0) > 1.5 * nominal   # no angle advance


def _foc_low_speed(dt_comp):
    from bldc.sensors import Encoder
    m = ps.motor("sinusoidal")
    cfg = ps.foc_config(m, dt_comp=dt_comp)
    plant = Plant(m, v_dc=ps.V_DC, dead_time=ps.DEAD_TIME, load=lambda t, w: 0.3 if t >= 0.1 else 0.0)
    sensors = Sensors(current=CurrentSensor(bits=24), encoder=Encoder(lines=1 << 20))
    res = simulate(plant, FocController(cfg), 0.5, lambda t: ps.rpm(300), sensors=sensors)
    tq = res.torque[res.window(0.4, 0.5)]
    return an.harmonics(tq, 1000, (6,))[6] / np.mean(tq)


def test_dead_time_compensation_removes_the_sixth_harmonic():
    without = _foc_low_speed(0.0)
    with_comp = _foc_low_speed(ps.DEAD_TIME / ps.TS)
    assert without > 0.003
    assert with_comp < 0.2 * without


def test_dead_time_compensation_at_no_load():
    # The sign comes from the current reference: at no load it must not turn into a negative
    # resistance (with the measured current it multiplied the torque ripple by four).
    m = ps.motor("sinusoidal")
    ptp = {}
    for comp in (0.0, ps.DEAD_TIME / ps.TS):
        plant = Plant(m, v_dc=ps.V_DC, dead_time=ps.DEAD_TIME)
        res = simulate(plant, FocController(ps.foc_config(m, dt_comp=comp)), 0.3, lambda t: ps.rpm(1500))
        ptp[comp] = np.ptp(res.torque[res.window(0.2, 0.3)])
    assert ptp[ps.DEAD_TIME / ps.TS] < 2.2 * ptp[0.0]


def test_voltage_limit():
    m = ps.motor("sinusoidal")
    plant = Plant(m, v_dc=ps.V_DC, model="average", load=lambda t, w: 0.3)
    res = simulate(plant, FocController(ps.foc_config(m)), 0.5, lambda t: ps.rpm(4500),
                   telemetry=("v_d", "v_q"))
    v = np.hypot(res.telemetry["v_d"], res.telemetry["v_q"])
    assert np.max(v) <= ps.V_DC / math.sqrt(3.0) * (1.0 + 1e-9)
    assert np.max(v) > 0.999 * ps.V_DC / math.sqrt(3.0)          # the limit is reached
    assert res.rpm[res.window(0.4, 0.5)].mean() < 4200.0          # so the speed stays below


@pytest.mark.parametrize("rpm", [300.0, 3000.0])
def test_six_step_speed_loop_is_stable_over_its_range(rpm):
    m = ps.motor("trapezoidal")
    plant = Plant(m, v_dc=ps.V_DC, dead_time=ps.DEAD_TIME, model="average",
                  load=lambda t, w: 0.2 if t > 0.3 else 0.0)
    res = simulate(plant, SixStepController(ps.six_step_config(m)), 1.5, lambda t: ps.rpm(rpm))
    w = res.window(1.0, 1.5)
    assert res.rpm[w].mean() == pytest.approx(rpm, rel=5e-3)
    assert np.ptp(res.rpm[w]) < 0.02 * rpm


def test_step_metrics_flags_a_response_that_never_arrives():
    t = np.linspace(0.0, 1.0, 101)
    met = an.step_metrics(t, 0.5 * (1.0 - np.exp(-t / 0.1)), 0.0, 1.0)
    assert math.isnan(met["t90"]) and math.isnan(met["settling_time"])


def test_configurations_are_checked():
    from bldc.control import FocConfig, SixStepConfig
    m = ps.motor("sinusoidal")
    good = ps.foc_config(m)
    for change in (dict(ts=0.0), dict(speed_div=0), dict(speed_div=70000), dict(pole_pairs=0),
                   dict(modulation_limit=2.0 / math.sqrt(3.0)), dict(speed_alpha=float("nan")),
                   dict(dt_comp=0.01, dt_comp_band=0.0), dict(v_dc_min=0.0)):
        with pytest.raises(ValueError):
            FocConfig(**{**good.__dict__, **change})
    six = ps.six_step_config(ps.motor("trapezoidal"))
    with pytest.raises(ValueError):
        SixStepConfig(**{**six.__dict__, "hall_timeout": 70000 * six.ts})


def test_undervoltage_turns_the_legs_off():
    from bldc.control.common import Inputs
    foc = FocController(ps.foc_config(ps.motor("sinusoidal"), calib_periods=0, speed_loop=False))
    six = SixStepController(ps.six_step_config(ps.motor("trapezoidal"), calib_periods=0, speed_loop=False))
    for v in (5.0, float("nan")):
        assert foc.step(Inputs((0.0, 0.0, 0.0), v, 0.0, 5, 2.0)).enable == (0, 0, 0)
        assert six.step(Inputs((0.0, 0.0, 0.0), v, 0.0, 5, 2.0)).enable == (0, 0, 0)
    # hysteresis: off below 10 V, back on only above 11 V
    for v, on in ((48.0, 1), (10.5, 1), (9.5, 0), (10.5, 0), (10.9, 0), (11.5, 1)):
        assert foc.step(Inputs((0.0, 0.0, 0.0), v, 0.0, 5, 2.0)).enable == (on,) * 3
        assert six.step(Inputs((0.0, 0.0, 0.0), v, 0.0, 5, 2.0)).enable == (on, on, 0)   # code 5: a high, b low
