/* Unit tests of the embedded controller. Build and run: make test */
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "bldc_ctrl.h"

static int checks = 0;
static int failures = 0;

#define CHECK(cond)                                                              \
    do {                                                                         \
        checks++;                                                                \
        if (!(cond)) {                                                           \
            failures++;                                                          \
            printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);               \
        }                                                                        \
    } while (0)

#define CHECK_CLOSE(x, expected, tol)                                            \
    do {                                                                         \
        const float x_ = (x);                                                    \
        const float e_ = (expected);                                             \
        checks++;                                                                \
        if (!(fabsf(x_ - e_) <= (tol))) {                                        \
            failures++;                                                          \
            printf("FAIL %s:%d: %s = %.7g, expected %.7g\n", __FILE__, __LINE__, \
                   #x, (double)x_, (double)e_);                                  \
        }                                                                        \
    } while (0)

static bldc_inputs_t inputs(float ia, float ib, float ic, float theta, uint8_t hall, float ref)
{
    bldc_inputs_t in;
    memset(&in, 0, sizeof(in));
    in.i_abc[0] = ia;
    in.i_abc[1] = ib;
    in.i_abc[2] = ic;
    in.v_dc = 48.0f;
    in.theta_e = theta;
    in.hall = hall;
    in.reference = ref;
    return in;
}

static void test_wrap_and_clamp(void)
{
    CHECK_CLOSE(bldc_wrap_pi(3.5f * BLDC_PI), -0.5f * BLDC_PI, 1e-5f);
    CHECK_CLOSE(bldc_wrap_pi(-0.25f), -0.25f, 1e-7f);
    CHECK_CLOSE(bldc_wrap_pi(BLDC_PI), BLDC_PI, 1e-6f);
    CHECK(bldc_clamp(2.0f, -1.0f, 1.0f) == 1.0f);
    CHECK(bldc_clamp(-2.0f, -1.0f, 1.0f) == -1.0f);
}

static void test_transforms(void)
{
    for (int k = 0; k < 36; k++) {
        const float th = (float)k * BLDC_TWO_PI / 36.0f;
        float alpha, beta, a, b, c, d, q;
        /* d = 1.5, q = -0.7 -> abc -> back */
        bldc_inverse_park(1.5f, -0.7f, cosf(th), sinf(th), &alpha, &beta);
        a = alpha;
        b = -0.5f * alpha + 0.5f * BLDC_SQRT3 * beta;
        c = -0.5f * alpha - 0.5f * BLDC_SQRT3 * beta;
        bldc_clarke(a, b, c, &alpha, &beta);
        bldc_park(alpha, beta, cosf(th), sinf(th), &d, &q);
        CHECK_CLOSE(d, 1.5f, 1e-5f);
        CHECK_CLOSE(q, -0.7f, 1e-5f);
    }
}

static void test_svpwm(void)
{
    float duty[3];
    bldc_svpwm(0.0f, 0.0f, 48.0f, duty);
    CHECK_CLOSE(duty[0], 0.5f, 1e-7f);
    CHECK_CLOSE(duty[1], 0.5f, 1e-7f);
    CHECK_CLOSE(duty[2], 0.5f, 1e-7f);
    float spread_max = 0.0f;
    for (int k = 0; k < 360; k++) {
        const float th = (float)k * BLDC_TWO_PI / 360.0f;
        const float amp = 48.0f * BLDC_INV_SQRT3;
        const float va = amp * cosf(th);
        const float vb = amp * sinf(th);
        bldc_svpwm(va, vb, 48.0f, duty);
        const float hi = fmaxf(duty[0], fmaxf(duty[1], duty[2]));
        const float lo = fminf(duty[0], fminf(duty[1], duty[2]));
        CHECK(hi - lo <= 1.0f + 1e-5f);
        spread_max = fmaxf(spread_max, hi - lo);
        /* line voltage a-b is exact */
        const float vab = va - (-0.5f * va + 0.5f * BLDC_SQRT3 * vb);
        CHECK_CLOSE((duty[0] - duty[1]) * 48.0f, vab, 1e-4f);
    }
    CHECK_CLOSE(spread_max, 1.0f, 1e-4f);
    bldc_svpwm(60.0f, 0.0f, 48.0f, duty);           /* overmodulation: duties clip */
    CHECK(duty[0] == 1.0f && duty[1] >= 0.0f && duty[2] >= 0.0f);
}

static void test_pi(void)
{
    bldc_pi_t pi;
    bldc_pi_init(&pi, 2.0f, 10.0f, 1e-3f);
    CHECK_CLOSE(bldc_pi_step(&pi, 1.0f, -10.0f, 10.0f, 0.5f), 2.0f + 0.01f + 0.5f, 1e-6f);
    bldc_pi_init(&pi, 1.0f, 100.0f, 1e-3f);
    float u = 0.0f;
    for (int k = 0; k < 1000; k++) {
        u = bldc_pi_step(&pi, 5.0f, -1.0f, 1.0f, 0.0f);
    }
    CHECK(u == 1.0f);
    CHECK(pi.integ <= 1.0f);                         /* no wind-up */
    u = bldc_pi_step(&pi, -0.5f, -1.0f, 1.0f, 0.0f);
    CHECK(u < 1.0f);                                 /* leaves saturation at once */
}

static bldc_foc_config_t foc_config(void)
{
    bldc_foc_config_t c;
    memset(&c, 0, sizeof(c));
    c.ts = 50e-6f;
    c.kp_i = 1.6667f;
    c.ki_i = 800.0f;
    c.kp_w = 0.8f;
    c.ki_w = 20.0f;
    c.l = 0.25e-3f;
    c.psi_m = 0.016f;
    c.i_max = 20.0f;
    c.accel_max = 1000.0f;
    c.speed_alpha = 0.5f;
    c.modulation_limit = BLDC_INV_SQRT3;
    c.angle_comp = 1.5f;
    c.dt_comp = 0.0f;
    c.dt_comp_band = 0.3f;
    c.v_dc_min = 10.0f;
    c.pole_pairs = 4u;
    c.speed_div = 20u;
    c.calib_periods = 100u;
    c.speed_loop = 1u;
    return c;
}

static void test_foc_calibration_and_outputs(void)
{
    bldc_foc_config_t cfg = foc_config();
    bldc_foc_t foc;
    bldc_outputs_t out;
    bldc_foc_telemetry_t tm;
    CHECK(bldc_foc_init(&foc, &cfg) == 0);
    for (int k = 0; k < 100; k++) {
        bldc_inputs_t in = inputs(0.10f, -0.05f, 0.02f, 0.0f, 0u, 0.0f);
        bldc_foc_step(&foc, &in, &out);
        CHECK(out.enable[0] == 0u && out.enable[1] == 0u && out.enable[2] == 0u);
    }
    bldc_foc_telemetry(&foc, &tm);
    CHECK(tm.running == 1u);
    CHECK_CLOSE(tm.offset[0], 0.10f, 1e-6f);
    CHECK_CLOSE(tm.offset[1], -0.05f, 1e-6f);
    CHECK_CLOSE(tm.offset[2], 0.02f, 1e-6f);
    /* the offsets alone read as zero current: no reference, no voltage, duties at 0.5 */
    bldc_inputs_t in = inputs(0.10f, -0.05f, 0.02f, 0.3f, 0u, 0.0f);
    bldc_foc_step(&foc, &in, &out);
    CHECK(out.enable[0] == 1u && out.enable[1] == 1u && out.enable[2] == 1u);
    for (int k = 0; k < 3; k++) {
        CHECK_CLOSE(out.duty[k], 0.5f, 1e-5f);
    }
}

static void test_foc_torque_mode(void)
{
    bldc_foc_config_t cfg = foc_config();
    cfg.speed_loop = 0u;
    cfg.calib_periods = 0u;
    bldc_foc_t foc;
    bldc_outputs_t out;
    bldc_foc_telemetry_t tm;
    CHECK(bldc_foc_init(&foc, &cfg) == 0);
    bldc_inputs_t in = inputs(0.0f, 0.0f, 0.0f, 0.0f, 0u, 5.0f);   /* 5 A asked, 0 A measured */
    bldc_foc_step(&foc, &in, &out);
    bldc_foc_telemetry(&foc, &tm);
    CHECK_CLOSE(tm.i_q_ref, 5.0f, 1e-6f);
    CHECK_CLOSE(tm.v_q, 5.0f * (1.6667f + 800.0f * 50e-6f), 1e-4f);
    CHECK_CLOSE(tm.v_d, 0.0f, 1e-6f);
    /* theta = 0: the q axis is the beta axis, so v_beta > 0 raises phase b over phase c */
    CHECK(out.duty[1] > out.duty[2]);
    in.reference = 100.0f;                                       /* limited to i_max */
    bldc_foc_step(&foc, &in, &out);
    bldc_foc_telemetry(&foc, &tm);
    CHECK_CLOSE(tm.i_q_ref, 20.0f, 1e-6f);
}

static void test_foc_speed_estimate(void)
{
    bldc_foc_config_t cfg = foc_config();
    cfg.calib_periods = 0u;
    cfg.speed_alpha = 1.0f;          /* no filter: the raw estimate */
    bldc_foc_t foc;
    bldc_outputs_t out;
    bldc_foc_telemetry_t tm;
    CHECK(bldc_foc_init(&foc, &cfg) == 0);
    const float w_m = 100.0f;        /* mechanical rad/s */
    float th = 0.0f;
    for (int k = 0; k < 40; k++) {
        bldc_inputs_t in = inputs(0.0f, 0.0f, 0.0f, bldc_wrap_pi(th), 0u, 0.0f);
        bldc_foc_step(&foc, &in, &out);
        th += 4.0f * w_m * cfg.ts;
    }
    bldc_foc_telemetry(&foc, &tm);
    CHECK_CLOSE(tm.speed_est, w_m, 0.05f);
}

static bldc_six_config_t six_config(void)
{
    bldc_six_config_t c;
    memset(&c, 0, sizeof(c));
    c.ts = 50e-6f;
    c.kp_i = 3.333f;
    c.ki_i = 1600.0f;
    c.kp_w = 0.2f;
    c.ki_w = 2.0f;
    c.i_max = 20.0f;
    c.accel_max = 1000.0f;
    c.hall_timeout = 0.05f;
    c.v_dc_min = 10.0f;
    c.pole_pairs = 4u;
    c.speed_div = 20u;
    c.calib_periods = 0u;
    c.speed_loop = 0u;
    return c;
}

static void test_six_step_table(void)
{
    const uint8_t codes[6] = {5u, 4u, 6u, 2u, 3u, 1u};
    const int hi_expected[6] = {0, 0, 1, 1, 2, 2};
    const int lo_expected[6] = {1, 2, 2, 0, 0, 1};
    int hi, lo;
    for (int k = 0; k < 6; k++) {
        CHECK(bldc_six_legs(codes[k], &hi, &lo) == 1);
        CHECK(hi == hi_expected[k] && lo == lo_expected[k]);
    }
    CHECK(bldc_six_legs(0u, &hi, &lo) == 0);
    CHECK(bldc_six_legs(7u, &hi, &lo) == 0);

    bldc_six_config_t cfg = six_config();
    bldc_six_t six;
    bldc_outputs_t out;
    CHECK(bldc_six_init(&six, &cfg) == 0);
    bldc_inputs_t in = inputs(0.0f, 0.0f, 0.0f, 0.0f, 6u, 3.0f);
    bldc_six_step(&six, &in, &out);
    CHECK(out.enable[0] == 0u && out.enable[1] == 1u && out.enable[2] == 1u);
    CHECK(out.duty[1] > 0.0f && out.duty[2] == 0.0f);
    in.hall = 7u;
    bldc_six_step(&six, &in, &out);
    CHECK(out.enable[0] == 0u && out.enable[1] == 0u && out.enable[2] == 0u);
}

static void test_six_step_speed(void)
{
    bldc_six_config_t cfg = six_config();
    bldc_six_t six;
    bldc_outputs_t out;
    bldc_six_telemetry_t tm;
    CHECK(bldc_six_init(&six, &cfg) == 0);
    const uint8_t codes[6] = {5u, 4u, 6u, 2u, 3u, 1u};
    /* an edge every 25 periods: 60 electrical degrees in 1.25 ms */
    for (int k = 0; k < 25 * 30; k++) {
        bldc_inputs_t in = inputs(0.0f, 0.0f, 0.0f, 0.0f, codes[(k / 25) % 6], 0.0f);
        bldc_six_step(&six, &in, &out);
    }
    bldc_six_telemetry(&six, &tm);
    const float expected = (BLDC_PI / 3.0f) / (4.0f * 25.0f * cfg.ts);
    CHECK_CLOSE(tm.speed_est, expected, 1e-3f * expected);
    /* no edge for longer than the timeout: standstill */
    for (int k = 0; k < 2000; k++) {
        bldc_inputs_t in = inputs(0.0f, 0.0f, 0.0f, 0.0f, 5u, 0.0f);
        bldc_six_step(&six, &in, &out);
    }
    bldc_six_telemetry(&six, &tm);
    CHECK(tm.speed_est == 0.0f);
}

static void test_foc_invalid_config_and_undervoltage(void)
{
    bldc_foc_t foc;
    bldc_outputs_t out;
    bldc_foc_telemetry_t tm;
    bldc_inputs_t in = inputs(0.0f, 0.0f, 0.0f, 0.0f, 0u, 5.0f);
    bldc_foc_config_t cfg = foc_config();
    cfg.ts = 0.0f;
    cfg.calib_periods = 0u;          /* so that only the fault flag can keep the legs off */
    CHECK(bldc_foc_init(&foc, &cfg) == -1);
    bldc_foc_step(&foc, &in, &out);
    bldc_foc_telemetry(&foc, &tm);
    CHECK(tm.fault == 1u && tm.running == 0u);
    CHECK(out.enable[0] == 0u && out.enable[1] == 0u && out.enable[2] == 0u);
    cfg = foc_config();
    cfg.speed_div = 0u;
    CHECK(bldc_foc_init(&foc, &cfg) == -1);
    cfg = foc_config();
    cfg.dt_comp = 0.01f;
    cfg.dt_comp_band = 0.0f;
    CHECK(bldc_foc_init(&foc, &cfg) == -1);
    cfg = foc_config();
    cfg.modulation_limit = 2.0f * BLDC_INV_SQRT3;      /* beyond the linear range */
    CHECK(bldc_foc_init(&foc, &cfg) == -1);
    cfg = foc_config();
    cfg.speed_alpha = NAN;
    CHECK(bldc_foc_init(&foc, &cfg) == -1);

    /* undervoltage: legs off, also for a NaN measurement */
    cfg = foc_config();
    cfg.calib_periods = 0u;
    cfg.speed_loop = 0u;
    CHECK(bldc_foc_init(&foc, &cfg) == 0);
    in.v_dc = 5.0f;
    bldc_foc_step(&foc, &in, &out);
    CHECK(out.enable[0] == 0u && out.enable[1] == 0u && out.enable[2] == 0u);
    in.v_dc = NAN;
    bldc_foc_step(&foc, &in, &out);
    CHECK(out.enable[0] == 0u && out.enable[1] == 0u && out.enable[2] == 0u);
    in.v_dc = 48.0f;
    bldc_foc_step(&foc, &in, &out);
    CHECK(out.enable[0] == 1u && out.enable[1] == 1u && out.enable[2] == 1u);
    /* hysteresis: off below 10 V, back on only above 11 V */
    const float levels[5] = {10.5f, 9.5f, 10.5f, 10.9f, 11.5f};
    const uint8_t on[5] = {1u, 0u, 0u, 0u, 1u};
    for (int k = 0; k < 5; k++) {
        in.v_dc = levels[k];
        bldc_foc_step(&foc, &in, &out);
        CHECK(out.enable[0] == on[k]);
    }
}

static void test_foc_dead_time_compensation_sign(void)
{
    /* torque mode, 5 A asked on q at theta = 0 and standstill: the reference phase currents are
     * (0, +4.33, -4.33) A, beyond the band: legs b and c move by +/- dt_comp, leg a not at all */
    bldc_foc_config_t cfg = foc_config();
    cfg.calib_periods = 0u;
    cfg.speed_loop = 0u;
    bldc_foc_t plain, comp;
    bldc_outputs_t o1, o2;
    CHECK(bldc_foc_init(&plain, &cfg) == 0);
    cfg.dt_comp = 0.01f;
    CHECK(bldc_foc_init(&comp, &cfg) == 0);
    bldc_inputs_t in = inputs(0.0f, 0.0f, 0.0f, 0.0f, 0u, 5.0f);
    bldc_foc_step(&plain, &in, &o1);
    bldc_foc_step(&comp, &in, &o2);
    CHECK_CLOSE(o2.duty[0] - o1.duty[0], 0.0f, 1e-6f);
    CHECK_CLOSE(o2.duty[1] - o1.duty[1], 0.01f, 1e-6f);
    CHECK_CLOSE(o2.duty[2] - o1.duty[2], -0.01f, 1e-6f);
}

static void test_six_step_current_and_faults(void)
{
    bldc_six_config_t cfg = six_config();
    bldc_six_t six;
    bldc_outputs_t out;
    bldc_six_telemetry_t tm;
    CHECK(bldc_six_init(&six, &cfg) == 0);
    /* code 6: b switched, c low; the regulated current is the mean of the two, 3 A */
    bldc_inputs_t in = inputs(0.0f, 4.0f, -2.0f, 0.0f, 6u, 3.0f);
    bldc_six_step(&six, &in, &out);
    bldc_six_telemetry(&six, &tm);
    CHECK_CLOSE(tm.i_meas, 3.0f, 1e-6f);
    in.v_dc = 3.0f;                                   /* undervoltage */
    bldc_six_step(&six, &in, &out);
    CHECK(out.enable[0] == 0u && out.enable[1] == 0u && out.enable[2] == 0u);
    cfg.hall_timeout = 65535.0f * cfg.ts;            /* would never fire: refused */
    CHECK(bldc_six_init(&six, &cfg) == -1);
    in.v_dc = 48.0f;
    bldc_six_step(&six, &in, &out);
    bldc_six_telemetry(&six, &tm);
    CHECK(tm.fault == 1u);
    CHECK(out.enable[0] == 0u && out.enable[1] == 0u && out.enable[2] == 0u);
}

int main(void)
{
    test_wrap_and_clamp();
    test_transforms();
    test_svpwm();
    test_pi();
    test_foc_calibration_and_outputs();
    test_foc_torque_mode();
    test_foc_speed_estimate();
    test_six_step_table();
    test_six_step_speed();
    test_foc_invalid_config_and_undervoltage();
    test_foc_dead_time_compensation_sign();
    test_six_step_current_and_faults();
    printf("%d checks, %d failures; state: FOC %u bytes, six-step %u bytes\n", checks, failures,
           (unsigned)bldc_foc_size(), (unsigned)bldc_six_size());
    return failures == 0 ? 0 : 1;
}
