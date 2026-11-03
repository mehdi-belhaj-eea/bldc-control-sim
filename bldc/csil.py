"""Software-in-the-loop: the embedded C controller (c/) driven by the Python simulation.

The C sources are compiled into a shared library (on first use, or when a source is newer than
the library) and called through ctypes once per PWM period. CFocController and
CSixStepController have the same interface as the Python reference controllers, so the same
simulation runs either one: that is the back-to-back test of the C code (tests/test_sil.py).
"""

from __future__ import annotations

import ctypes
import os
import pathlib
import subprocess
import sys
from ctypes import POINTER, c_char, c_float, c_int, c_size_t, c_uint8, c_uint16

from .control.common import Inputs, Outputs
from .control.foc import FocConfig
from .control.six_step import SixStepConfig

C_DIR = pathlib.Path(__file__).resolve().parents[1] / "c"
SOURCES = [C_DIR / "src" / name for name in ("bldc_math.c", "foc.c", "six_step.c")]
HEADER = C_DIR / "include" / "bldc_ctrl.h"
_EXT = ".dll" if sys.platform == "win32" else ".dylib" if sys.platform == "darwin" else ".so"
LIBRARY = C_DIR / "build" / f"libbldcctrl{_EXT}"


def build(force: bool = False) -> pathlib.Path:
    """Compile the shared library if it is missing or older than its sources."""
    newest = max(p.stat().st_mtime for p in SOURCES + [HEADER])
    if force or not LIBRARY.exists() or LIBRARY.stat().st_mtime < newest:
        LIBRARY.parent.mkdir(parents=True, exist_ok=True)
        cc = os.environ.get("CC", "cc")
        cmd = [cc, "-std=c99", "-O2", "-fPIC", "-shared", f"-I{HEADER.parent}",
               *map(str, SOURCES), "-o", str(LIBRARY), "-lm"]
        subprocess.run(cmd, check=True)
    return LIBRARY


class _Inputs(ctypes.Structure):
    _fields_ = [("i_abc", c_float * 3), ("v_dc", c_float), ("theta_e", c_float),
                ("hall", c_uint8), ("reference", c_float)]


class _Outputs(ctypes.Structure):
    _fields_ = [("duty", c_float * 3), ("enable", c_uint8 * 3)]


class _FocConfig(ctypes.Structure):
    _fields_ = [("ts", c_float), ("kp_i", c_float), ("ki_i", c_float), ("kp_w", c_float),
                ("ki_w", c_float), ("l", c_float), ("psi_m", c_float), ("i_max", c_float),
                ("accel_max", c_float), ("speed_alpha", c_float), ("modulation_limit", c_float),
                ("angle_comp", c_float), ("dt_comp", c_float), ("dt_comp_band", c_float),
                ("v_dc_min", c_float), ("pole_pairs", c_uint16), ("speed_div", c_uint16),
                ("calib_periods", c_uint16), ("speed_loop", c_uint8)]


class _FocTelemetry(ctypes.Structure):
    _fields_ = [("i_d", c_float), ("i_q", c_float), ("i_d_ref", c_float), ("i_q_ref", c_float),
                ("v_d", c_float), ("v_q", c_float), ("speed_est", c_float), ("ref_ramped", c_float),
                ("offset", c_float * 3), ("running", c_uint8), ("fault", c_uint8)]


class _SixConfig(ctypes.Structure):
    _fields_ = [("ts", c_float), ("kp_i", c_float), ("ki_i", c_float), ("kp_w", c_float),
                ("ki_w", c_float), ("i_max", c_float), ("accel_max", c_float), ("hall_timeout", c_float),
                ("v_dc_min", c_float), ("pole_pairs", c_uint16), ("speed_div", c_uint16),
                ("calib_periods", c_uint16), ("speed_loop", c_uint8)]


class _SixTelemetry(ctypes.Structure):
    _fields_ = [("i_ref", c_float), ("i_meas", c_float), ("v_cmd", c_float), ("duty", c_float),
                ("speed_est", c_float), ("ref_ramped", c_float), ("offset", c_float * 3),
                ("running", c_uint8), ("fault", c_uint8)]


MIRRORS = (_Inputs, _Outputs, _FocConfig, _FocTelemetry, _SixConfig, _SixTelemetry)


def layout() -> list[int]:
    """Size, then offset of each field, of every mirror: what bldc_interface_layout returns."""
    out = []
    for mirror in MIRRORS:
        out.append(ctypes.sizeof(mirror))
        out.extend(getattr(mirror, name).offset for name, _ in mirror._fields_)
    return out


_lib = None


def library() -> ctypes.CDLL:
    global _lib
    if _lib is None:
        lib = ctypes.CDLL(str(build()))
        state = POINTER(c_char)
        for prefix, cfg_t, tm_t in (("bldc_foc", _FocConfig, _FocTelemetry), ("bldc_six", _SixConfig, _SixTelemetry)):
            getattr(lib, f"{prefix}_size").restype = c_size_t
            getattr(lib, f"{prefix}_size").argtypes = []
            getattr(lib, f"{prefix}_init").argtypes = [state, POINTER(cfg_t)]
            getattr(lib, f"{prefix}_init").restype = c_int
            getattr(lib, f"{prefix}_step").argtypes = [state, POINTER(_Inputs), POINTER(_Outputs)]
            getattr(lib, f"{prefix}_step").restype = None
            getattr(lib, f"{prefix}_telemetry").argtypes = [state, POINTER(tm_t)]
            getattr(lib, f"{prefix}_telemetry").restype = None
        lib.bldc_svpwm.argtypes = [c_float, c_float, c_float, POINTER(c_float)]
        lib.bldc_svpwm.restype = None
        lib.bldc_wrap_pi.argtypes = [c_float]
        lib.bldc_wrap_pi.restype = c_float
        lib.bldc_six_legs.argtypes = [c_uint8, POINTER(c_int), POINTER(c_int)]
        lib.bldc_six_legs.restype = c_int
        lib.bldc_interface_layout.argtypes = [POINTER(c_size_t), c_size_t]
        lib.bldc_interface_layout.restype = c_size_t
        expected = layout()
        n = lib.bldc_interface_layout(None, 0)
        got = (c_size_t * n)()
        lib.bldc_interface_layout(got, n)
        if list(got) != expected:
            raise RuntimeError("the ctypes mirrors do not match the C structure layout "
                               f"(C: {list(got)}, Python: {expected})")
        _lib = lib
    return _lib


class _CController:
    _prefix = ""
    _tm_type: type = ctypes.Structure

    def _setup(self, cfg_struct: ctypes.Structure) -> None:
        lib = library()
        self._state = ctypes.create_string_buffer(getattr(lib, f"{self._prefix}_size")())
        if getattr(lib, f"{self._prefix}_init")(self._state, ctypes.byref(cfg_struct)) != 0:
            raise ValueError("the C controller refused the configuration")
        self._step = getattr(lib, f"{self._prefix}_step")
        self._telemetry = getattr(lib, f"{self._prefix}_telemetry")
        self._in = _Inputs()
        self._out = _Outputs()
        self._tm = self._tm_type()

    def step(self, inp: Inputs) -> Outputs:
        x = self._in
        x.i_abc[0], x.i_abc[1], x.i_abc[2] = inp.i_abc
        x.v_dc, x.theta_e, x.hall, x.reference = inp.v_dc, inp.theta_e, inp.hall, inp.reference
        self._step(self._state, ctypes.byref(x), ctypes.byref(self._out))
        o = self._out
        return Outputs((o.duty[0], o.duty[1], o.duty[2]), (o.enable[0], o.enable[1], o.enable[2]))

    def telemetry(self) -> dict:
        self._telemetry(self._state, ctypes.byref(self._tm))
        out = {}
        for name, _ in self._tm._fields_:
            value = getattr(self._tm, name)
            out[name] = list(value) if name == "offset" else float(value)
        return out

    @property
    def offset(self) -> list[float]:
        return self.telemetry()["offset"]

    @property
    def running(self) -> bool:
        return bool(self.telemetry()["running"])


class CFocController(_CController):
    """The C field-oriented controller, configured from a FocConfig."""

    _prefix = "bldc_foc"
    _tm_type = _FocTelemetry

    def __init__(self, cfg: FocConfig) -> None:
        self.cfg = cfg
        self._setup(_FocConfig(
            ts=cfg.ts, kp_i=cfg.kp_i, ki_i=cfg.ki_i, kp_w=cfg.kp_w, ki_w=cfg.ki_w, l=cfg.l,
            psi_m=cfg.psi_m, i_max=cfg.i_max, accel_max=cfg.accel_max, speed_alpha=cfg.speed_alpha,
            modulation_limit=cfg.modulation_limit, angle_comp=cfg.angle_comp, dt_comp=cfg.dt_comp,
            dt_comp_band=cfg.dt_comp_band, v_dc_min=cfg.v_dc_min, pole_pairs=cfg.pole_pairs,
            speed_div=cfg.speed_div, calib_periods=cfg.calib_periods, speed_loop=int(cfg.speed_loop)))


class CSixStepController(_CController):
    """The C six-step controller, configured from a SixStepConfig."""

    _prefix = "bldc_six"
    _tm_type = _SixTelemetry

    def __init__(self, cfg: SixStepConfig) -> None:
        self.cfg = cfg
        self._setup(_SixConfig(
            ts=cfg.ts, kp_i=cfg.kp_i, ki_i=cfg.ki_i, kp_w=cfg.kp_w, ki_w=cfg.ki_w, i_max=cfg.i_max,
            accel_max=cfg.accel_max, hall_timeout=cfg.hall_timeout, v_dc_min=cfg.v_dc_min,
            pole_pairs=cfg.pole_pairs, speed_div=cfg.speed_div, calib_periods=cfg.calib_periods,
            speed_loop=int(cfg.speed_loop)))
