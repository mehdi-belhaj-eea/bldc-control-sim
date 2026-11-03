/* Field-oriented control with an encoder. Reference: bldc/control/foc.py. */
#include <math.h>
#include <string.h>

#include "bldc_ctrl.h"

static void outputs_off(bldc_outputs_t *out)
{
    for (int k = 0; k < 3; k++) {
        out->duty[k] = 0.0f;
        out->enable[k] = 0u;
    }
}

/* Same checks as FocConfig in Python. Written as !(x > 0) so that NaN fails too. */
static int config_valid(const bldc_foc_config_t *c)
{
    if (!(c->ts > 0.0f) || !(c->i_max > 0.0f) || !(c->accel_max > 0.0f) || !(c->v_dc_min > 0.0f)) {
        return 0;
    }
    if (!(c->kp_i >= 0.0f) || !(c->ki_i >= 0.0f) || !(c->kp_w >= 0.0f) || !(c->ki_w >= 0.0f)
        || !(c->l >= 0.0f) || !(c->psi_m >= 0.0f) || !(c->angle_comp >= 0.0f) || !(c->dt_comp >= 0.0f)) {
        return 0;
    }
    if (c->pole_pairs < 1u || c->speed_div < 1u) {
        return 0;
    }
    if (!(c->speed_alpha > 0.0f && c->speed_alpha <= 1.0f)) {
        return 0;
    }
    if (!(c->modulation_limit > 0.0f && c->modulation_limit <= BLDC_INV_SQRT3 + 1e-6f)) {
        return 0;
    }
    if (c->dt_comp > 0.0f && !(c->dt_comp_band > 0.0f)) {
        return 0;
    }
    return 1;
}

int bldc_foc_init(bldc_foc_t *foc, const bldc_foc_config_t *cfg)
{
    memset(foc, 0, sizeof(*foc));
    foc->cfg = *cfg;
    if (!config_valid(cfg)) {
        foc->fault = 1u;
        return -1;
    }
    bldc_pi_init(&foc->pi_d, cfg->kp_i, cfg->ki_i, cfg->ts);
    bldc_pi_init(&foc->pi_q, cfg->kp_i, cfg->ki_i, cfg->ts);
    bldc_pi_init(&foc->pi_w, cfg->kp_w, cfg->ki_w, (float)cfg->speed_div * cfg->ts);
    return 0;
}

void bldc_foc_step(bldc_foc_t *foc, const bldc_inputs_t *in, bldc_outputs_t *out)
{
    const bldc_foc_config_t *c = &foc->cfg;

    if (foc->fault) {
        outputs_off(out);
        return;
    }

    /* 1. offset calibration: legs off, the measured currents should be zero */
    if (foc->calib_count < c->calib_periods) {
        for (int k = 0; k < 3; k++) {
            foc->offset_acc[k] += in->i_abc[k];
        }
        foc->calib_count++;
        if (foc->calib_count == c->calib_periods) {
            for (int k = 0; k < 3; k++) {
                foc->offset[k] = foc->offset_acc[k] / (float)c->calib_periods;
            }
        }
        outputs_off(out);
        return;
    }
    /* undervoltage lockout with hysteresis; on recovery the angle is re-read and the current
     * integrators restart from zero (the motor current has decayed meanwhile) */
    if (foc->undervoltage) {
        if (!(in->v_dc >= BLDC_UV_RESUME * c->v_dc_min)) {
            outputs_off(out);
            return;
        }
        foc->undervoltage = 0u;
    } else if (!(in->v_dc >= c->v_dc_min)) {    /* also catches a NaN measurement */
        foc->undervoltage = 1u;
        foc->started = 0u;
        foc->div_count = 0u;
        foc->pi_d.integ = 0.0f;
        foc->pi_q.integ = 0.0f;
        outputs_off(out);
        return;
    }

    /* 2. currents in the rotor frame */
    const float ia = in->i_abc[0] - foc->offset[0];
    const float ib = in->i_abc[1] - foc->offset[1];
    const float ic = in->i_abc[2] - foc->offset[2];
    const float th = in->theta_e;
    float alpha, beta;
    bldc_clarke(ia, ib, ic, &alpha, &beta);
    bldc_park(alpha, beta, cosf(th), sinf(th), &foc->i_d, &foc->i_q);

    /* 3. speed estimate and speed loop, every speed_div periods */
    if (!foc->started) {
        foc->started = 1u;
        foc->theta_prev = th;
    }
    foc->div_count++;
    if (foc->div_count >= c->speed_div) {
        foc->div_count = 0u;
        const float tw = (float)c->speed_div * c->ts;
        const float w_raw = bldc_wrap_pi(th - foc->theta_prev) / (tw * (float)c->pole_pairs);
        foc->theta_prev = th;
        foc->speed_est += c->speed_alpha * (w_raw - foc->speed_est);
        if (c->speed_loop) {
            const float step_max = c->accel_max * tw;
            foc->ref_ramped += bldc_clamp(in->reference - foc->ref_ramped, -step_max, step_max);
            foc->i_q_ref = bldc_pi_step(&foc->pi_w, foc->ref_ramped - foc->speed_est,
                                        -c->i_max, c->i_max, 0.0f);
        }
    }
    if (!c->speed_loop) {
        foc->i_q_ref = bldc_clamp(in->reference, -c->i_max, c->i_max);
    }
    foc->i_d_ref = 0.0f;

    /* 4. current loops with decoupling, voltage vector limited d axis first */
    const float we = (float)c->pole_pairs * foc->speed_est;
    const float v_max = c->modulation_limit * in->v_dc;
    const float ff_d = -we * c->l * foc->i_q;
    const float ff_q = we * (c->l * foc->i_d + c->psi_m);
    foc->v_d = bldc_pi_step(&foc->pi_d, foc->i_d_ref - foc->i_d, -v_max, v_max, ff_d);
    const float vq_lim = sqrtf(fmaxf(v_max * v_max - foc->v_d * foc->v_d, 0.0f));
    foc->v_q = bldc_pi_step(&foc->pi_q, foc->i_q_ref - foc->i_q, -vq_lim, vq_lim, ff_q);

    /* 5. back to the stator frame at the angle the rotor will have, then SVPWM */
    const float th_out = th + c->angle_comp * we * c->ts;
    const float co = cosf(th_out);
    const float so = sinf(th_out);
    float v_alpha, v_beta;
    bldc_inverse_park(foc->v_d, foc->v_q, co, so, &v_alpha, &v_beta);
    bldc_svpwm(v_alpha, v_beta, in->v_dc, out->duty);

    /* 6. dead-time compensation, with the sign of the reference phase currents: the measured
     *    current ripples across zero at light load and would act as a negative resistance */
    if (c->dt_comp > 0.0f) {
        float r_alpha, r_beta, ref[3];
        bldc_inverse_park(foc->i_d_ref, foc->i_q_ref, co, so, &r_alpha, &r_beta);
        bldc_inverse_clarke(r_alpha, r_beta, ref);
        for (int k = 0; k < 3; k++) {
            const float s = bldc_clamp(ref[k] / c->dt_comp_band, -1.0f, 1.0f);
            out->duty[k] = bldc_clamp(out->duty[k] + c->dt_comp * s, 0.0f, 1.0f);
        }
    }
    for (int k = 0; k < 3; k++) {
        out->enable[k] = 1u;
    }
}

void bldc_foc_telemetry(const bldc_foc_t *foc, bldc_foc_telemetry_t *tm)
{
    tm->i_d = foc->i_d;
    tm->i_q = foc->i_q;
    tm->i_d_ref = foc->i_d_ref;
    tm->i_q_ref = foc->i_q_ref;
    tm->v_d = foc->v_d;
    tm->v_q = foc->v_q;
    tm->speed_est = foc->speed_est;
    tm->ref_ramped = foc->ref_ramped;
    for (int k = 0; k < 3; k++) {
        tm->offset[k] = foc->offset[k];
    }
    tm->running = (!foc->fault && foc->calib_count >= foc->cfg.calib_periods) ? 1u : 0u;
    tm->fault = foc->fault;
}

size_t bldc_foc_size(void)
{
    return sizeof(bldc_foc_t);
}
