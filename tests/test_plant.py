import math

import numpy as np
import pytest

from bldc import presets as ps
from bldc.motor import Motor
from bldc.plant import HIGH, LOW, OPEN, Plant

LOCKED = Motor(r=0.12, l=0.25e-3, psi_m=0.016, pole_pairs=4, j=1e9)   # rotor that cannot turn


def run(plant, duty, enable, periods):
    for _ in range(periods):
        plant.step_period(duty, enable)
        assert sum(plant.i) == pytest.approx(0.0, abs=1e-9)


@pytest.mark.parametrize("model", ["switched", "average"])
def test_locked_rotor_step_matches_rl_solution(model):
    p = Plant(LOCKED, v_dc=1.0, model=model)
    run(p, (1.0, 0.0, 0.0), (1, 1, 0), 40)
    t = 40 * p.t_pwm
    expected = 1.0 / (2 * LOCKED.r) * (1.0 - math.exp(-t / LOCKED.tau))
    assert p.i[0] == pytest.approx(expected, rel=1e-9)
    assert p.i[2] == 0.0


def test_pwm_pattern_with_dead_time():
    p = Plant(LOCKED, dead_time=1e-6)
    t = p.t_pwm
    seq = p.leg_pattern(0.5, True)
    assert seq == [(0.0, LOW), (0.25 * t, OPEN), (0.25 * t + 1e-6, HIGH), (0.75 * t, OPEN), (0.75 * t + 1e-6, LOW)]
    assert p.leg_pattern(0.01, True) == [(0.0, LOW), (0.495 * t, OPEN), (0.505 * t + 1e-6, LOW)]   # pulse dropped
    assert p.leg_pattern(0.3, False) == [(0.0, OPEN)]


@pytest.mark.parametrize("model", ["switched", "average"])
def test_steady_state_of_three_phase_duties(model):
    p = Plant(LOCKED, v_dc=48.0, model=model)
    duty = (0.53, 0.49, 0.48)
    run(p, duty, (1, 1, 1), 600)
    mean = sum(duty) / 3.0
    for k in range(3):
        assert p.i[k] == pytest.approx(48.0 * (duty[k] - mean) / LOCKED.r, rel=2e-3)


@pytest.mark.parametrize("model", ["switched", "average"])
def test_dead_time_voltage_error(model):
    # Legs a and b switching, c off. With i_a > 0 and i_b < 0 each leg loses t_d/T of V_dc in the
    # direction of its current: the line voltage drops by 2·t_d/T·V_dc.
    td = 0.5e-6
    p = Plant(LOCKED, v_dc=48.0, dead_time=td, model=model)
    run(p, (0.55, 0.45, 0.5), (1, 1, 0), 600)
    expected = 48.0 * (0.10 - 2.0 * td / p.t_pwm) / (2.0 * LOCKED.r)
    assert p.i[0] == pytest.approx(expected, rel=5e-3)
    assert p.i[2] == 0.0


def test_spinning_motor_with_inverter_off_below_rectification():
    m = ps.motor("sinusoidal")
    p = Plant(m, v_dc=48.0, omega_m0=200.0)      # line EMF peak 22 V < 48 V: diodes stay blocked
    run(p, (0.0, 0.0, 0.0), (0, 0, 0), 200)
    assert p.i == [0.0, 0.0, 0.0]
    t = 200 * p.t_pwm
    assert p.omega_m == pytest.approx(200.0 * math.exp(-m.b * t / m.j), rel=1e-6)


def test_diode_bridge_brakes_above_rectification():
    m = ps.motor("sinusoidal")
    p = Plant(m, v_dc=10.0, omega_m0=200.0)      # line EMF peak 22 V > 10 V: generator mode
    torque = []
    for _ in range(400):
        p.step_period((0.0, 0.0, 0.0), (0, 0, 0))
        torque.append(p.torque_avg)
        assert sum(p.i) == pytest.approx(0.0, abs=1e-9)
    assert np.mean(torque[100:]) < -0.05
    assert max(abs(x) for x in p.i) < 200.0


def test_switched_and_average_models_agree_in_closed_loop():
    from bldc.control import FocController
    from bldc.sim import simulate
    m = ps.motor("sinusoidal")
    out = {}
    for model in ("switched", "average"):
        plant = Plant(m, v_dc=ps.V_DC, dead_time=ps.DEAD_TIME, model=model, load=lambda t, w: 0.6)
        res = simulate(plant, FocController(ps.foc_config(m)), 0.2, lambda t: ps.rpm(1000))
        w = res.window(0.15, 0.2)
        out[model] = (res.rpm[w].mean(), res.torque[w].mean(), res.i_dq[1][w].mean())
    for a, b in zip(out["switched"], out["average"]):
        assert a == pytest.approx(b, rel=5e-3)


# ----------------------------------------------------------------------------------------------
# Independent reference for the hybrid solver: fixed 50 ns steps, explicit diode logic.
from bldc.motor import COMMUTATION, hall_code  # noqa: E402


def brute_force(m, v_dc, dead_time, t_pwm, pattern, n_periods, theta_m0, omega_m, dt):
    """Phase currents at the start of each period, rotor speed held constant."""
    pat_plant = Plant(m, v_dc=v_dc, f_pwm=1.0 / t_pwm, dead_time=dead_time)   # PWM timing only
    p, steps = m.pole_pairs, round(t_pwm / dt)
    i = [0.0, 0.0, 0.0]
    out = []
    for n in range(n_periods):
        out.append(list(i))
        duty, enable = pattern(p * (theta_m0 + omega_m * n * t_pwm))
        pats = [pat_plant.leg_pattern(duty[k], bool(enable[k])) for k in range(3)]
        for s in range(steps):
            tl = (s + 0.5) * dt
            states = []
            for pat in pats:
                st = pat[0][1]
                for ts, x in pat:
                    if ts <= tl:
                        st = x
                states.append(st)
            e = m.back_emf(p * (theta_m0 + omega_m * (n * t_pwm + tl)), omega_m)
            u = [v_dc if st == HIGH else 0.0 if st == LOW else None for st in states]
            for k in range(3):
                if u[k] is None and i[k] != 0.0:
                    u[k] = 0.0 if i[k] > 0.0 else v_dc
            on = [k for k in range(3) if u[k] is not None]
            if len(on) == 2:                       # open leg: its terminal voltage v_n + e
                o = 3 - on[0] - on[1]
                v_o = 0.5 * (u[on[0]] - e[on[0]] + u[on[1]] - e[on[1]]) + e[o]
                if v_o > v_dc:
                    u[o] = v_dc
                elif v_o < 0.0:
                    u[o] = 0.0
                on = [k for k in range(3) if u[k] is not None]
            di = [0.0, 0.0, 0.0]
            if len(on) == 3:
                vn = sum(u[k] - e[k] for k in range(3)) / 3.0
                di = [(u[k] - e[k] - vn - m.r * i[k]) / m.l for k in range(3)]
            elif len(on) == 2:
                j, q = on
                a = (0.5 * (u[j] - e[j] - u[q] + e[q]) - m.r * i[j]) / m.l
                di[j], di[q] = a, -a
            new = [i[k] + dt * di[k] for k in range(3)]
            for k in range(3):                     # a diode cannot carry a reverse current
                direction = i[k] if i[k] != 0.0 else (1.0 if u[k] == 0.0 else -1.0)
                if states[k] == OPEN and new[k] * direction < 0.0:
                    new[k] = 0.0
            if len(on) == 2 and 0.0 in (new[on[0]], new[on[1]]):
                new = [0.0, 0.0, 0.0]
            elif len(on) == 3:
                z = [k for k in range(3) if new[k] == 0.0 and i[k] != 0.0]
                if z:
                    a, b = [k for k in range(3) if k != z[0]]
                    half = 0.5 * (new[a] - new[b])
                    new[a], new[b] = half, -half
            i = new
    return np.array(out)


def test_switched_model_matches_a_fine_step_reference():
    # Six-step pattern at a fixed 0.6 duty, 1500 rpm, 0.5 µs dead time: commutations, diode
    # decay of the outgoing phase, conduction of the floating phase during the off-time.
    base = ps.motor("trapezoidal")
    m = Motor(r=base.r, l=base.l, psi_m=base.psi_m, pole_pairs=4, j=1e9, emf="trapezoidal")
    t_pwm, td, w, th0, n = 50e-6, 0.5e-6, ps.rpm(1500.0), 0.05, 80

    def pattern(theta_e):
        hi, lo = COMMUTATION[hall_code(theta_e)]
        d, en = [0.0, 0.0, 0.0], [0, 0, 0]
        d[hi], en[hi], en[lo] = 0.6, 1, 1
        return d, en

    ref = brute_force(m, 48.0, td, t_pwm, pattern, n, th0, w, 50e-9)
    for max_step, tol in ((None, 0.15), (1e-6, 0.01)):
        plant = Plant(m, v_dc=48.0, dead_time=td, theta_m0=th0, omega_m0=w, max_step=max_step)
        got = []
        for _ in range(n):
            got.append(list(plant.i))
            plant.step_period(*pattern(plant.theta_e))
        # the only approximation of the solver, the EMF held over each sub-interval, shrinks
        # with the sub-interval length
        assert np.max(np.abs(np.array(got) - ref)) < tol
    assert np.max(np.abs(ref)) > 15.0


def test_average_model_keeps_the_pole_voltage_within_the_rails():
    p = Plant(LOCKED, v_dc=48.0, dead_time=0.5e-6, model="average")
    p.i = [1.0, -1.0, 0.0]
    (dt, cond), = p._segments((0.004, 0.996, 0.5), (1, 1, 1))
    assert cond[0] == 0.0 and cond[1] == 48.0 and cond[2] == pytest.approx(24.0)


def test_peak_current_includes_the_pwm_ripple():
    # Two phases in series under a 50 % duty: triangular ripple of V_dc·D·(1 − D)·T/(2L) peak to
    # peak around the mean; the sample, in the middle of the low-side interval, is the mean.
    p = Plant(LOCKED, v_dc=4.8)
    run(p, (0.5, 0.0, 0.0), (1, 1, 0), 600)
    ripple = 4.8 * 0.5 * 0.5 * p.t_pwm / (2.0 * LOCKED.l)
    sample = p.i[0]
    p.step_period((0.5, 0.0, 0.0), (1, 1, 0))
    assert sample == pytest.approx(0.5 * 4.8 / (2.0 * LOCKED.r), rel=2e-3)
    assert p.i_peak - sample == pytest.approx(0.5 * ripple, rel=0.03)
