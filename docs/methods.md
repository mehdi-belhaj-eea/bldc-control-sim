# Methods

The equations behind the simulation and the control laws, with their assumptions. Symbols
follow the code.

## 1. Motor (`bldc.motor`)

Star-connected permanent-magnet motor, isolated neutral n, phase resistance R and synchronous
inductance L (equal on d and q: surface magnets). With the pole voltages u_k measured from the
negative DC rail and the back-EMFs e_k:

```math
u_k = R\,i_k + L\,\frac{di_k}{dt} + e_k + v_n, \qquad
\sum_k i_k = 0 \;\Rightarrow\; v_n = \tfrac{1}{3}\sum_k (u_k - e_k)
```

The neutral voltage keeps the triplen harmonics of a trapezoidal EMF out of the currents.

Back-EMF. θ_e is the electrical angle of the d axis (magnet flux) from the phase-a axis. For a
sinusoidal machine the magnet flux linkage of phase k is ψ_m·cos(θ_e − k·2π/3):

```math
e_k = \omega_e \psi_m \, s\!\left(\theta_e + \pi - k\tfrac{2\pi}{3}\right), \qquad
s = \sin \text{ (sinusoidal) or a trapezoid with a } 120^\circ \text{ flat top}
```

The trapezoid has the zero crossings of the sine, 60° linear ramps and a flat top of 1. Its
fundamental is 12/π² ≈ 1.216: the fundamental flux linkage is ψ_1 = 1.216·ψ_m.

Torque, from the power balance (valid at standstill and for any current shape), and mechanics:

```math
T_e = \frac{\sum_k e_k i_k}{\omega_m} = p\,\psi_m \sum_k s_k\, i_k, \qquad
J\frac{d\omega_m}{dt} = T_e - T_L(t, \omega_m) - B\,\omega_m
```

- Sinusoidal currents of amplitude i_q: T_e = 1.5·p·ψ_1·i_q (k_t = 0.096 N·m/A for the example
  motor, 0.117 N·m/A for its trapezoidal twin).
- 120° block currents ±I on the flat tops: T_e = 2·p·ψ_m·I (0.128 N·m/A). On a sinusoidal EMF
  the line EMF varies as √3·cos x over each ±30° sector, so the mean is (3√3/π)·p·ψ_m·I and the
  torque ripples by 1 − cos 30° = 13 %.

Hall sensors are placed so that their edges fall on the ideal commutation instants, 30° after
the zero crossings of the EMF. Code 4·H_a + 2·H_b + H_c runs 5, 4, 6, 2, 3, 1 in forward
rotation; each code selects the phase with the highest EMF (switched with PWM) and the one
with the lowest (held low).

## 2. Inverter and dead time (`bldc.plant`)

Center-aligned PWM of period T_s. Leg k with duty d_k is high from (1 − d_k)·T_s/2 to
(1 + d_k)·T_s/2. A dead time t_d is inserted before each turn-on: both switches are off and
the diode of the leg conducts the current:

```math
u_k = 0 \text{ if } i_k > 0, \qquad u_k = V_{dc} \text{ if } i_k < 0
```

When i_k reaches zero the leg opens (i_k stays 0). An open leg starts conducting again if its
terminal voltage v_n + e_k leaves [0, V_dc]. With one leg open, the two others carry ±i and

```math
L\frac{di}{dt} = \tfrac{1}{2}\left(u_j - e_j - u_m + e_m\right) - R\,i
```

Solution. The period is cut at every switching instant. Within a sub-interval the pole
voltages are constant and the EMF is held at its value at the start (the rotor turns by
1.8 electrical degrees per period at 1500 rpm), so each current is an exponential, solved in
closed form:

```math
i_k(t) = i_{k,\infty} + \left(i_k(0) - i_{k,\infty}\right) e^{-t/\tau}, \qquad \tau = L/R
```

The instant a diode current would cross zero is solved in closed form,
t = τ·ln((i(0) − i_∞)/(−i_∞)), and the interval split there; an open leg can start conducting
again only at the start of a sub-interval. The torque uses the mean current over each
sub-interval; the speed and angle follow by Euler steps of at most one sub-interval.

The approximation is the EMF held over each sub-interval. Against an independent fine-step
reference (50 ns Euler steps with explicit diode logic, `tests/test_plant.py`) on a six-step
pattern at 1500 rpm, the currents differ by 0.1 A out of 23 A after 80 periods; with
`max_step = 1 µs` the gap falls to 5 mA. With 1 µs sub-intervals the six-step torque ripple of
example 2 changes by 0.1 percentage point.

Mean effect of the dead time. Over a period, a leg loses t_d/T_s·V_dc of voltage in the
direction of its current (0.48 V for 0.5 µs at 20 kHz and 48 V). The averaged model
(`model="average"`) applies u_k = d_k·V_dc − sign(i_k)·(t_d/T_s)·V_dc, kept within [0, V_dc],
over one interval per period; the tests check both models against each other.

Limit: a pulse shorter than the dead time is dropped; at a duty above 1 − 2·t_d/T_s the low-side
pulse that spans the period boundary is not shortened. Switching transients, diode drops,
MOSFET resistance and the DC-link ripple are not modelled.

## 3. Sampling and delay (`bldc.sim`)

The currents are sampled at the start of each period, in the middle of the low-side on-time,
where the shunt currents equal their mean over the period. The controller computes during that
period and its duties apply from the next one. The voltage therefore lags the measurement by
one period of computation plus, on average, half a period of PWM hold:

```math
T_\Sigma = 1.5\,T_s = 75~\mu\text{s at } 20~\text{kHz}
```

## 4. Transforms and space-vector PWM (`bldc.control.transforms`)

Amplitude-invariant Clarke and Park transforms, with the three measured currents (the
zero-sequence part, such as a common offset, is rejected):

```math
i_\alpha = \tfrac{1}{3}(2i_a - i_b - i_c), \quad i_\beta = \tfrac{1}{\sqrt3}(i_b - i_c), \quad
\begin{bmatrix} i_d \\ i_q \end{bmatrix} =
\begin{bmatrix} \cos\theta & \sin\theta \\ -\sin\theta & \cos\theta \end{bmatrix}
\begin{bmatrix} i_\alpha \\ i_\beta \end{bmatrix}
```

Space-vector PWM by min-max injection: the phase references are shifted by −(max + min)/2,
then d_k = 1/2 + (v_k + shift)/V_dc. This centres the active vectors exactly as conventional
SVPWM with equal zero vectors, and is linear up to a phase-voltage amplitude of V_dc/√3, the
circle inscribed in the hexagon (15 % more than sine PWM, V_dc/2).

## 5. PI controller with anti-windup (`bldc.control.pi`)

```math
u = k_p e + x + f, \qquad x \leftarrow x + k_i T_s e
```

f is a feed-forward. The integrator takes the current error before the output is computed
(backward Euler). When u leaves [u_min, u_max] it is clamped, and the integrator x only moves
if the error drives u back into range (conditional integration). x is also kept within
[u_min − f, u_max − f], because the limits change at every step (voltage left to the q axis).

## 6. Current loop: modulus optimum (`bldc.control.tuning.current_loop`)

With the back-EMF and the d-q cross-coupling compensated by feed-forward (section 8), each axis
is 1/(R + sL) behind the delay T_Σ. The PI zero cancels the electrical pole and the gain sets
the open loop to the modulus (Betragsoptimum) form:

```math
T_i = \frac{L}{R}, \quad k_p = \frac{L}{2T_\Sigma}, \quad k_i = \frac{k_p}{T_i} = \frac{R}{2T_\Sigma}
\quad\Rightarrow\quad
L(s) \approx \frac{1}{2T_\Sigma s\,(1 + sT_\Sigma)}, \qquad
\frac{i}{i^*} = \frac{1}{2T_\Sigma^2 s^2 + 2T_\Sigma s + 1}
```

Damping 1/√2: 4.3 % overshoot, settling to ±2 % in about 8.4·T_Σ, crossover 1/(2·T_Σ), phase
margin 90° − 28.6° = 61.4° with the delay taken as e^(−sT_Σ). Example motor: k_p = 1.67 V/A,
k_i = 800 V/(A·s), crossover 6667 rad/s (1.06 kHz). The closed current loop behaves as a first
order lag of 2·T_Σ for the speed loop.

## 7. Speed loop: symmetrical optimum (`bldc.control.tuning.speed_loop`)

Plant k_t/(J·s) behind the small lags T_σ:

```math
T_\sigma = \underbrace{2T_\Sigma}_{\text{current loop}} + \underbrace{T_\omega}_{\text{speed averaged over } T_\omega \text{, then held}} + \underbrace{\tfrac{1-\alpha}{\alpha}T_\omega}_{\text{speed filter}}
```

With the spacing factor a:

```math
k_p = \frac{J}{a\,k_t\,T_\sigma}, \quad T_i = a^2 T_\sigma, \quad
\omega_c = \frac{1}{a\,T_\sigma}, \quad
\varphi_m = \arctan\frac{a^2 - 1}{2a}
```

The crossover sits at the geometric mean of 1/T_i and 1/T_σ, where the phase lead of the PI
zero and the lag of T_σ leave the most margin. a = 3: 53° of margin. Example motor (T_ω = 1 ms,
α = 0.5): T_σ = 2.15 ms, crossover 155 rad/s (25 Hz). The symmetrical optimum is optimal against
load steps; setpoint changes go through a ramp (accel_max) to keep the overshoot small.

Six-step: the speed comes from the Hall edges averaged over one electrical turn, half a turn of
delay, which grows as the speed falls. The loop is therefore designed at a low speed, 600 rpm
(T_σ = 13.7 ms, crossover 24 rad/s): it stays stable down to 300 rpm, the bottom of its
range (300 to 3000 rpm), and faster speeds only shorten the delay. Phase margin with the lags
taken as a pure delay T_σ:

| Speed | 300 rpm | 600 rpm | 1500 rpm | 3000 rpm |
| --- | --- | --- | --- | --- |
| T_σ | 26.2 ms | 13.7 ms | 6.2 ms | 3.7 ms |
| Designed at 600 rpm | 34° | 52° | 63° | 67° |
| Designed at 1500 rpm | −13° (unstable) | 28° | 52° | 61° |

## 8. Field-oriented control (`bldc.control.foc`, `c/src/foc.c`)

Every period: offset correction, Clarke and Park with the encoder angle, then the d and q PIs
(i_d* = 0) with decoupling and EMF feed-forward:

```math
v_d = \mathrm{PI}_d(i_d^* - i_d) - \omega_e L\, i_q, \qquad
v_q = \mathrm{PI}_q(i_q^* - i_q) + \omega_e (L\, i_d + \psi_1)
```

- Voltage limit: |v| ≤ V_dc/√3, d axis first, the q axis gets √(V_max² − v_d²).
- The rotor turns during the 1.5·T_s delay: the inverse Park uses θ_e + 1.5·ω_e·T_s.
- Speed: every T_ω = 20 periods, the angle difference over T_ω (wrapped to ±π), then
  y ← y + α(x − y). The 14-bit encoder (16384 counts) gives steps of 0.38 rad/s (3.7 rpm) over
  1 ms; through the filter and k_p of the speed loop, each step moves i_q* by about 0.15 A.
- Offset calibration: during the first 200 periods (10 ms) all legs are off; the mean of each
  measured current is its offset.
- Dead-time compensation: d_k ← d_k + (t_d/T_s)·sat(i*_k/0.3 A), with i*_k the reference
  currents (i_d*, i_q*) turned into phase currents at the output angle. The band, about half the
  PWM current ripple, follows the way the real error fades when the current crosses zero within
  the period. The sign is taken from the reference, not from the measured current: at light
  load the measured current ripples across zero, and feeding it back acts as a negative
  resistance. At no load and 1500 rpm the torque ripple is 16 mN·m peak-to-peak without
  compensation, 27 mN·m with the reference sign, 44 mN·m with the measured sign.
- Below v_dc_min (10 V) all legs are off: undervoltage, or a broken DC-link measurement (NaN
  included). They come back above 1.1·v_dc_min (hysteresis, so that a bus hovering at the
  threshold does not toggle the legs every period). During the lockout the current decays and
  the rotor keeps turning: on recovery the angle is read afresh before the next speed estimate,
  and the current integrators restart from zero. After a 5.3 ms dropout at 1500 rpm under load,
  the speed estimate stays above 1400 rpm and the current peaks at 12.8 A (6.8 A before).
- An invalid configuration is refused at initialisation.

## 9. Six-step control (`bldc.control.six_step`, `c/src/six_step.c`)

The Hall code selects the conducting pair. The leg of the phase with the highest EMF switches
with complementary PWM (H_PWM–L_ON: the high side is modulated, the low side of the other phase
stays on); the third leg is off. The current PI acts on ½·(i_hi − i_lo), the mean of the two
conducting currents, with the plant 2R + 2L·s (two phases in series) tuned by the modulus
optimum; its output is the line voltage d·V_dc. Regulating the switched leg alone would be
wrong at every other commutation, when that leg carries the incoming current, which starts from
zero: the PI saturates and the torque jumps. The speed is n·(π/3)/(p·Σ intervals) over the last
n ≤ 6 Hall intervals; no edge for 50 ms reads as standstill.

Two effects appear in the simulation:

- Commutation: the outgoing phase current decays through a diode while the incoming one rises;
  the torque leaves ±10 % of its mean for one to three PWM periods with this motor.
- Floating-phase conduction: during the PWM off-time both conducting phases sit at the negative
  rail, so the off phase sees its own EMF at its terminal; while that EMF is negative, its lower
  diode conducts a small current.

## 10. Signatures of the non-idealities (example 3)

| Cause | In the rotor frame | Torque ripple |
| --- | --- | --- |
| Current offsets o_abc | constant vector in α-β, rotating at −ω_e in d-q | at f_e, about k_t·\|o_αβ\| |
| Gain mismatch g on one phase | elliptical current vector: a component at −ω_e in α-β | at 2·f_e, about k_t·g·I/3 |
| Dead time | square-wave voltage error per phase: 5th, 7th harmonics in α-β | at 6·f_e |
| Encoder quantisation | speed estimate in steps of one count per T_ω, amplified by k_p of the speed loop | broadband |

The current loop removes errors it can see; it cannot remove errors in what it measures. The
offsets and gains must therefore be calibrated, and the speed estimate must be clean enough for
the speed gain.

## 11. C implementation and software in the loop (`c/`, `bldc.csil`)

- C99, single-precision float only (checked by `-Wdouble-promotion`), no dynamic allocation,
  no mutable global data: the caller owns each controller structure. Built with
  `-Wall -Wextra -Wpedantic -Wshadow -Wconversion -Werror` by gcc, clang and arm-none-eabi-gcc.
- The step functions run once per PWM period (ADC end-of-conversion interrupt on a target).
- The initialisation checks the configuration (positive period, gains not negative, at least
  one pole pair, a modulation limit within the linear range, a Hall timeout the 16-bit counter
  can reach...) and returns an error; the controller then keeps every leg off. The step
  functions also turn every leg off below v_dc_min.
- `bldc.csil` compiles the sources into a shared library and calls them through ctypes; the
  size and the offset of every field of the interface structures are checked against the C
  layout at load time. The Cortex-M4F build leaves that layout table out (`-DBLDC_NO_LAYOUT`).
- Back-to-back tests (`tests/test_sil.py`): the same inputs, rounded to single precision, give
  the same duties to within 1e-5 over the first 50 ms; the Python and C controllers in closed
  loop give speeds within 0.25 %.

## 12. Limits of the models

- Ideal switches and diodes: no conduction drops, no switching transients or losses.
- Linear magnetic circuit, L_d = L_q, no saturation, no iron loss, no cogging torque.
- Constant DC link (an ideal source): no DC-link ripple, no regenerative overvoltage.
- Sensors: offset, gain, ADC quantisation and optional white noise; no anti-aliasing filter
  dynamics, no encoder interpolation errors.
- Mechanics: rigid shaft, viscous friction plus the load function.
- The controllers have no field weakening, no fault handling beyond invalid Hall codes, and no
  sensorless observer.

## 13. How the models are checked (`tests/`, `c/tests/`)

| Model | Checked against |
| --- | --- |
| Back-EMF, torque | Park transform of the EMF on the q axis; torque constant for any angle; flat-top torque of block currents |
| Trapezoid | its Fourier fundamental 12/π² |
| Hall sensors and table | sequence 5-4-6-2-3-1; selected phases are the highest and lowest EMF at every angle |
| Inverter solver | locked-rotor RL step (exact to 1e-9); steady state of three duties; mean dead-time error; inverter off on a spinning motor, below (no current) and above (diode-bridge braking) the rectification threshold; an independent fine-step reference with explicit diode logic (six-step pattern, dead time, floating phase) |
| Switched vs averaged model | closed-loop speed, torque and current within 0.5 % |
| SVPWM | line voltages exact; linear limit V_dc/√3 at every angle |
| Current loop | 4.3 % overshoot of the modulus optimum (simulated 4.1 %) |
| Decoupling, angle advance | 5 A step at 3000 rpm: i_d moves less than without each of them |
| Dead-time compensation | removes the 6·f_e torque line at 300 rpm; bounded at no load |
| Voltage limit | \|v\| never exceeds V_dc/√3 when the speed cannot be reached |
| Speed loops | zero steady-state error; FOC load-step dip within 10 % of the linear prediction (62 rpm); six-step stable from 300 to 3000 rpm |
| Configuration, undervoltage | invalid values refused (Python and C); legs off below v_dc_min or with a NaN DC link |
| Undervoltage | 5.3 ms DC-link dropout at 1500 rpm under load: clean restart (Python and C); hysteresis between 10 and 11 V |
| C code | 971 unit checks; layout of the interface structures; back-to-back with the Python reference (open and closed loop) |

Deliberate errors each make at least one test fail: sign of the d-axis decoupling (−ω_e·L·i_q),
of the angle advance, of the dead-time compensation, of the open-leg voltage in the solver;
compensation from the measured current; doubled voltage limit; six-step regulating the switched
leg alone; peak current not tracked within the period; undervoltage restart without a fresh
angle. Not covered: the q-axis term ω_e·L·i_d, which stays negligible while i_d* = 0 and will
need a test with field weakening.

## References

- R. Krishnan, *Permanent Magnet Synchronous and Brushless DC Motor Drives*, CRC Press, 2010.
- D. G. Holmes, T. A. Lipo, *Pulse Width Modulation for Power Converters: Principles and Practice*, IEEE Press / Wiley, 2003.
- W. Leonhard, *Control of Electrical Drives*, 3rd ed., Springer, 2001 (modulus and symmetrical optimum).
- C. Kessler, "Das symmetrische Optimum", *Regelungstechnik*, vol. 6, 1958.
- K. J. Åström, T. Hägglund, *Advanced PID Control*, ISA, 2006 (anti-windup).
- R. Carlson, M. Lajoie-Mazenc, J. C. dos S. Fagundes, "Analysis of torque ripple due to phase commutation in brushless DC machines", *IEEE Trans. Industry Applications*, vol. 28, no. 3, 1992.
- Y. Murai, T. Watanabe, H. Iwasaki, "Waveform distortion and correction circuit for PWM inverters with switching lag-times", *IEEE Trans. Industry Applications*, vol. IA-23, no. 5, 1987.
