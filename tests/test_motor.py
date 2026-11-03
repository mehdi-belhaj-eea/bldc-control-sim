import math

import numpy as np
import pytest

from bldc import presets as ps
from bldc.control.transforms import clarke, inverse_clarke, inverse_park, park
from bldc.motor import COMMUTATION, TRAPEZOID_FUNDAMENTAL, Motor, hall_code, trapezoid


def test_trapezoid_shape():
    assert trapezoid(0.0) == 0.0
    assert trapezoid(math.pi / 6.0) == pytest.approx(1.0)
    assert trapezoid(math.pi / 2.0) == 1.0
    assert trapezoid(3.0 * math.pi / 2.0) == -1.0
    x = np.linspace(0.0, 2.0 * math.pi, 3601)
    y = np.array([trapezoid(v) for v in x])
    assert np.max(np.abs(np.diff(y))) < 6.0 / math.pi * (x[1] - x[0]) + 1e-12   # continuous
    assert all(trapezoid(v + math.pi) == pytest.approx(-trapezoid(v)) for v in x[:100])
    n = 3600
    s = np.array([trapezoid(2.0 * math.pi * k / n) for k in range(n)])
    b1 = 2.0 / n * np.sum(s * np.sin(2.0 * math.pi * np.arange(n) / n))
    assert b1 == pytest.approx(TRAPEZOID_FUNDAMENTAL, rel=1e-5)


def test_back_emf_lies_on_the_q_axis():
    m = ps.motor("sinusoidal")
    for th in np.linspace(0.0, 2.0 * math.pi, 37):
        e = m.back_emf(th, 100.0)
        assert sum(e) == pytest.approx(0.0, abs=1e-12)
        d, q = park(*clarke(*e), math.cos(th), math.sin(th))
        assert d == pytest.approx(0.0, abs=1e-12)
        assert q == pytest.approx(m.pole_pairs * 100.0 * m.psi_m)


def test_torque_constant_with_sinusoidal_currents():
    m = ps.motor("sinusoidal")
    for th in np.linspace(0.0, 2.0 * math.pi, 37):
        i = inverse_clarke(*inverse_park(0.0, 5.0, math.cos(th), math.sin(th)))
        assert m.torque(th, i) == pytest.approx(m.k_t * 5.0)
    assert m.k_t == pytest.approx(1.5 * 4 * 0.016)


def test_trapezoidal_torque_with_block_currents():
    m = ps.motor("trapezoidal")
    for th in np.linspace(0.0, 2.0 * math.pi, 361):
        hi, lo = COMMUTATION[hall_code(th)]
        i = [0.0, 0.0, 0.0]
        i[hi], i[lo] = 3.0, -3.0
        s = m.shape(th)
        if abs(s[hi]) == 1.0 and abs(s[lo]) == 1.0:     # both conducting phases on their flat tops
            assert m.torque(th, i) == pytest.approx(m.k_t_six_step * 3.0)


def test_hall_sequence_and_commutation_table():
    m = ps.motor("trapezoidal")
    codes = []
    for k in range(3600):
        th = 2.0 * math.pi * k / 3600
        c = hall_code(th)
        if not codes or codes[-1] != c:
            codes.append(c)
        hi, lo = COMMUTATION[c]
        s = m.shape(th)
        assert s[hi] == pytest.approx(max(s)) and s[lo] == pytest.approx(min(s))
    seq = [5, 4, 6, 2, 3, 1]
    start = seq.index(codes[0])
    assert codes[:6] == seq[start:] + seq[:start]
    assert set(COMMUTATION) == {1, 2, 3, 4, 5, 6}


def test_invalid_parameters():
    with pytest.raises(ValueError):
        Motor(r=0.1, l=0.0, psi_m=0.01, pole_pairs=4, j=1e-4)
    with pytest.raises(ValueError):
        Motor(r=0.1, l=1e-4, psi_m=0.01, pole_pairs=4, j=1e-4, emf="square")
