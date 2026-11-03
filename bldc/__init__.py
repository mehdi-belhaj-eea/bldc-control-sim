"""bldc-control-sim: BLDC / PMSM drive simulation with Python reference control laws and the
embedded C implementation run in the loop.

Modules
-------
motor      motor parameters, back-EMF shapes (sinusoidal, trapezoidal), torque, Hall sensors
plant      inverter (switched with dead time, or averaged) + motor + load, one PWM period at a time
sensors    current chain with offset, gain error and ADC; encoder; Hall sensors
control    Clarke/Park, SVPWM, PI with anti-windup, loop tuning, FOC and six-step controllers
csil       ctypes bridge to the C controller (software-in-the-loop)
sim        closed-loop simulation and recorded results
analysis   torque ripple, harmonics, THD, step-response figures of merit
"""

__version__ = "0.1.0"
