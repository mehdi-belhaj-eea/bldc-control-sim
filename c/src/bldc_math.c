/* Building blocks: transforms, space-vector PWM, PI with anti-windup. */
#include <math.h>
#include <stddef.h>

#include "bldc_ctrl.h"

float bldc_clamp(float x, float lo, float hi)
{
    return (x < lo) ? lo : ((x > hi) ? hi : x);
}

float bldc_wrap_pi(float x)
{
    x = fmodf(x + BLDC_PI, BLDC_TWO_PI);
    if (x <= 0.0f) {
        x += BLDC_TWO_PI;
    }
    return x - BLDC_PI;
}

/* Amplitude-invariant Clarke transform; the zero-sequence part is rejected. */
void bldc_clarke(float a, float b, float c, float *alpha, float *beta)
{
    *alpha = (2.0f * a - b - c) * (1.0f / 3.0f);
    *beta = (b - c) * BLDC_INV_SQRT3;
}

void bldc_park(float alpha, float beta, float cos_t, float sin_t, float *d, float *q)
{
    *d = alpha * cos_t + beta * sin_t;
    *q = -alpha * sin_t + beta * cos_t;
}

void bldc_inverse_park(float d, float q, float cos_t, float sin_t, float *alpha, float *beta)
{
    *alpha = d * cos_t - q * sin_t;
    *beta = d * sin_t + q * cos_t;
}

void bldc_inverse_clarke(float alpha, float beta, float abc[3])
{
    abc[0] = alpha;
    abc[1] = -0.5f * alpha + 0.5f * BLDC_SQRT3 * beta;
    abc[2] = -0.5f * alpha - 0.5f * BLDC_SQRT3 * beta;
}

/* Min-max zero-sequence injection: equivalent to space-vector PWM with equal zero vectors.
 * Linear up to a phase-voltage amplitude of V_dc/sqrt(3); the duties clip beyond. */
void bldc_svpwm(float alpha, float beta, float v_dc, float duty[3])
{
    float v[3];
    bldc_inverse_clarke(alpha, beta, v);
    const float shift = -0.5f * (fmaxf(v[0], fmaxf(v[1], v[2])) + fminf(v[0], fminf(v[1], v[2])));
    for (int k = 0; k < 3; k++) {
        duty[k] = bldc_clamp(0.5f + (v[k] + shift) / v_dc, 0.0f, 1.0f);
    }
}

/* Interface layout for bindings (left out of the target build with -DBLDC_NO_LAYOUT). */
#ifndef BLDC_NO_LAYOUT
#define LAYOUT_FIELD(type, field) offsetof(type, field)

static const size_t LAYOUT[] = {
    sizeof(bldc_inputs_t),
    LAYOUT_FIELD(bldc_inputs_t, i_abc), LAYOUT_FIELD(bldc_inputs_t, v_dc),
    LAYOUT_FIELD(bldc_inputs_t, theta_e), LAYOUT_FIELD(bldc_inputs_t, hall),
    LAYOUT_FIELD(bldc_inputs_t, reference),
    sizeof(bldc_outputs_t),
    LAYOUT_FIELD(bldc_outputs_t, duty), LAYOUT_FIELD(bldc_outputs_t, enable),
    sizeof(bldc_foc_config_t),
    LAYOUT_FIELD(bldc_foc_config_t, ts), LAYOUT_FIELD(bldc_foc_config_t, kp_i),
    LAYOUT_FIELD(bldc_foc_config_t, ki_i), LAYOUT_FIELD(bldc_foc_config_t, kp_w),
    LAYOUT_FIELD(bldc_foc_config_t, ki_w), LAYOUT_FIELD(bldc_foc_config_t, l),
    LAYOUT_FIELD(bldc_foc_config_t, psi_m), LAYOUT_FIELD(bldc_foc_config_t, i_max),
    LAYOUT_FIELD(bldc_foc_config_t, accel_max), LAYOUT_FIELD(bldc_foc_config_t, speed_alpha),
    LAYOUT_FIELD(bldc_foc_config_t, modulation_limit), LAYOUT_FIELD(bldc_foc_config_t, angle_comp),
    LAYOUT_FIELD(bldc_foc_config_t, dt_comp), LAYOUT_FIELD(bldc_foc_config_t, dt_comp_band),
    LAYOUT_FIELD(bldc_foc_config_t, v_dc_min), LAYOUT_FIELD(bldc_foc_config_t, pole_pairs),
    LAYOUT_FIELD(bldc_foc_config_t, speed_div), LAYOUT_FIELD(bldc_foc_config_t, calib_periods),
    LAYOUT_FIELD(bldc_foc_config_t, speed_loop),
    sizeof(bldc_foc_telemetry_t),
    LAYOUT_FIELD(bldc_foc_telemetry_t, i_d), LAYOUT_FIELD(bldc_foc_telemetry_t, i_q),
    LAYOUT_FIELD(bldc_foc_telemetry_t, i_d_ref), LAYOUT_FIELD(bldc_foc_telemetry_t, i_q_ref),
    LAYOUT_FIELD(bldc_foc_telemetry_t, v_d), LAYOUT_FIELD(bldc_foc_telemetry_t, v_q),
    LAYOUT_FIELD(bldc_foc_telemetry_t, speed_est), LAYOUT_FIELD(bldc_foc_telemetry_t, ref_ramped),
    LAYOUT_FIELD(bldc_foc_telemetry_t, offset), LAYOUT_FIELD(bldc_foc_telemetry_t, running),
    LAYOUT_FIELD(bldc_foc_telemetry_t, fault),
    sizeof(bldc_six_config_t),
    LAYOUT_FIELD(bldc_six_config_t, ts), LAYOUT_FIELD(bldc_six_config_t, kp_i),
    LAYOUT_FIELD(bldc_six_config_t, ki_i), LAYOUT_FIELD(bldc_six_config_t, kp_w),
    LAYOUT_FIELD(bldc_six_config_t, ki_w), LAYOUT_FIELD(bldc_six_config_t, i_max),
    LAYOUT_FIELD(bldc_six_config_t, accel_max), LAYOUT_FIELD(bldc_six_config_t, hall_timeout),
    LAYOUT_FIELD(bldc_six_config_t, v_dc_min), LAYOUT_FIELD(bldc_six_config_t, pole_pairs),
    LAYOUT_FIELD(bldc_six_config_t, speed_div), LAYOUT_FIELD(bldc_six_config_t, calib_periods),
    LAYOUT_FIELD(bldc_six_config_t, speed_loop),
    sizeof(bldc_six_telemetry_t),
    LAYOUT_FIELD(bldc_six_telemetry_t, i_ref), LAYOUT_FIELD(bldc_six_telemetry_t, i_meas),
    LAYOUT_FIELD(bldc_six_telemetry_t, v_cmd), LAYOUT_FIELD(bldc_six_telemetry_t, duty),
    LAYOUT_FIELD(bldc_six_telemetry_t, speed_est), LAYOUT_FIELD(bldc_six_telemetry_t, ref_ramped),
    LAYOUT_FIELD(bldc_six_telemetry_t, offset), LAYOUT_FIELD(bldc_six_telemetry_t, running),
    LAYOUT_FIELD(bldc_six_telemetry_t, fault),
};

size_t bldc_interface_layout(size_t *out, size_t n)
{
    const size_t count = sizeof(LAYOUT) / sizeof(LAYOUT[0]);
    if (out != NULL) {
        for (size_t k = 0u; k < n && k < count; k++) {
            out[k] = LAYOUT[k];
        }
    }
    return count;
}
#endif /* BLDC_NO_LAYOUT */

void bldc_pi_init(bldc_pi_t *pi, float kp, float ki, float ts)
{
    pi->kp = kp;
    pi->ki = ki;
    pi->ts = ts;
    pi->integ = 0.0f;
}

/* u = kp e + integral(ki e) + ff, limited to [out_min, out_max]. While saturated, the
 * integrator only moves if the error drives the output back into range, and it is kept within
 * the range left by the feed-forward. */
float bldc_pi_step(bldc_pi_t *pi, float error, float out_min, float out_max, float ff)
{
    const float i_new = pi->integ + pi->ki * pi->ts * error;
    float u = pi->kp * error + i_new + ff;
    if (u > out_max) {
        u = out_max;
        if (error < 0.0f) {
            pi->integ = i_new;
        }
    } else if (u < out_min) {
        u = out_min;
        if (error > 0.0f) {
            pi->integ = i_new;
        }
    } else {
        pi->integ = i_new;
    }
    pi->integ = bldc_clamp(pi->integ, out_min - ff, out_max - ff);
    return u;
}
