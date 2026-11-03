/* Six-step control with Hall sensors. Reference: bldc/control/six_step.py. */
#include <string.h>

#include "bldc_ctrl.h"

#define HALL_NONE 0xFFu

/* Hall code -> leg switched with PWM (hi) and leg held low (lo), forward rotation. */
static const int8_t HI_LEG[8] = {-1, 2, 1, 2, 0, 0, 1, -1};
static const int8_t LO_LEG[8] = {-1, 1, 0, 0, 2, 1, 2, -1};

static void outputs_off(bldc_outputs_t *out)
{
    for (int k = 0; k < 3; k++) {
        out->duty[k] = 0.0f;
        out->enable[k] = 0u;
    }
}

int bldc_six_legs(uint8_t hall, int *hi, int *lo)
{
    if (hall > 7u || HI_LEG[hall] < 0) {
        *hi = -1;
        *lo = -1;
        return 0;
    }
    *hi = HI_LEG[hall];
    *lo = LO_LEG[hall];
    return 1;
}

/* Same checks as SixStepConfig in Python. Written as !(x > 0) so that NaN fails too. */
static int config_valid(const bldc_six_config_t *c)
{
    if (!(c->ts > 0.0f) || !(c->i_max > 0.0f) || !(c->accel_max > 0.0f)
        || !(c->hall_timeout > 0.0f) || !(c->v_dc_min > 0.0f)) {
        return 0;
    }
    if (!(c->kp_i >= 0.0f) || !(c->ki_i >= 0.0f) || !(c->kp_w >= 0.0f) || !(c->ki_w >= 0.0f)) {
        return 0;
    }
    if (c->pole_pairs < 1u || c->speed_div < 1u) {
        return 0;
    }
    /* the 16-bit edge counter saturates at 65535 periods: the timeout must come before */
    if (!(c->hall_timeout < 65535.0f * c->ts)) {
        return 0;
    }
    return 1;
}

int bldc_six_init(bldc_six_t *six, const bldc_six_config_t *cfg)
{
    memset(six, 0, sizeof(*six));
    six->cfg = *cfg;
    six->hall_prev = HALL_NONE;
    if (!config_valid(cfg)) {
        six->fault = 1u;
        return -1;
    }
    bldc_pi_init(&six->pi_i, cfg->kp_i, cfg->ki_i, cfg->ts);
    bldc_pi_init(&six->pi_w, cfg->kp_w, cfg->ki_w, (float)cfg->speed_div * cfg->ts);
    return 0;
}

/* Mean speed over the stored Hall intervals (one electrical turn when all six are known). */
static float hall_speed(const bldc_six_t *six)
{
    if (six->n_intervals == 0u) {
        return 0.0f;
    }
    uint32_t total = 0u;
    for (unsigned k = 0u; k < six->n_intervals; k++) {
        total += six->intervals[k];
    }
    return (float)six->n_intervals * (BLDC_PI / 3.0f)
           / ((float)six->cfg.pole_pairs * (float)total * six->cfg.ts);
}

void bldc_six_step(bldc_six_t *six, const bldc_inputs_t *in, bldc_outputs_t *out)
{
    const bldc_six_config_t *c = &six->cfg;

    if (six->fault) {
        outputs_off(out);
        return;
    }
    if (six->calib_count < c->calib_periods) {
        for (int k = 0; k < 3; k++) {
            six->offset_acc[k] += in->i_abc[k];
        }
        six->calib_count++;
        if (six->calib_count == c->calib_periods) {
            for (int k = 0; k < 3; k++) {
                six->offset[k] = six->offset_acc[k] / (float)c->calib_periods;
            }
        }
        outputs_off(out);
        return;
    }

    /* speed from the Hall edges */
    if (six->edge_count < 0xFFFFu) {
        six->edge_count++;
    }
    if (in->hall != six->hall_prev) {
        if (six->hall_prev != HALL_NONE) {
            if (six->have_edge) {
                six->intervals[six->idx] = six->edge_count;
                six->idx = (uint8_t)((six->idx + 1u) % BLDC_SIX_EDGES);
                if (six->n_intervals < BLDC_SIX_EDGES) {
                    six->n_intervals++;
                }
            }
            six->have_edge = 1u;
        }
        six->edge_count = 0u;
        six->hall_prev = in->hall;
    }
    if ((float)six->edge_count * c->ts > c->hall_timeout) {
        six->n_intervals = 0u;
        six->idx = 0u;
        six->have_edge = 0u;
    }

    six->div_count++;
    if (six->div_count >= c->speed_div) {
        six->div_count = 0u;
        six->speed_est = hall_speed(six);
        if (c->speed_loop) {
            const float step_max = c->accel_max * (float)c->speed_div * c->ts;
            six->ref_ramped += bldc_clamp(in->reference - six->ref_ramped, -step_max, step_max);
            six->i_ref = bldc_pi_step(&six->pi_w, six->ref_ramped - six->speed_est, 0.0f, c->i_max, 0.0f);
        }
    }
    if (!c->speed_loop) {
        six->i_ref = bldc_clamp(in->reference, 0.0f, c->i_max);
    }

    int hi, lo;
    if (!bldc_six_legs(in->hall, &hi, &lo)) {   /* sensor fault: all legs off */
        outputs_off(out);
        return;
    }
    if (six->undervoltage) {                    /* lockout with hysteresis */
        if (!(in->v_dc >= BLDC_UV_RESUME * c->v_dc_min)) {
            outputs_off(out);
            return;
        }
        six->undervoltage = 0u;
    } else if (!(in->v_dc >= c->v_dc_min)) {    /* undervoltage, or a NaN measurement */
        six->undervoltage = 1u;
        six->pi_i.integ = 0.0f;
        outputs_off(out);
        return;
    }
    /* mean of the two conducting currents: at a commutation one of them is the incoming
     * current, which starts from zero */
    six->i_meas = 0.5f * ((in->i_abc[hi] - six->offset[hi]) - (in->i_abc[lo] - six->offset[lo]));
    six->v_cmd = bldc_pi_step(&six->pi_i, six->i_ref - six->i_meas, 0.0f, in->v_dc, 0.0f);
    six->duty = six->v_cmd / in->v_dc;
    outputs_off(out);
    out->duty[hi] = six->duty;
    out->enable[hi] = 1u;
    out->enable[lo] = 1u;
}

void bldc_six_telemetry(const bldc_six_t *six, bldc_six_telemetry_t *tm)
{
    tm->i_ref = six->i_ref;
    tm->i_meas = six->i_meas;
    tm->v_cmd = six->v_cmd;
    tm->duty = six->duty;
    tm->speed_est = six->speed_est;
    tm->ref_ramped = six->ref_ramped;
    for (int k = 0; k < 3; k++) {
        tm->offset[k] = six->offset[k];
    }
    tm->running = (!six->fault && six->calib_count >= six->cfg.calib_periods) ? 1u : 0u;
    tm->fault = six->fault;
}

size_t bldc_six_size(void)
{
    return sizeof(bldc_six_t);
}
