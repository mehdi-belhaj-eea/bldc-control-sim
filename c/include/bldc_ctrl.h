/*
 * bldc_ctrl — embedded control of a BLDC / PMSM drive.
 *
 * Field-oriented control (FOC) with an encoder, and six-step control with Hall sensors.
 * C99, single-precision float only, no dynamic allocation, no global state: every controller
 * lives in a structure owned by the caller. The step functions are meant to run once per PWM
 * period, from the ADC end-of-conversion interrupt.
 *
 * The Python modules bldc/control/foc.py and six_step.py are the reference of this code; the
 * test suite runs both on the same inputs and in closed loop with the motor model.
 */
#ifndef BLDC_CTRL_H
#define BLDC_CTRL_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define BLDC_PI        3.14159265358979f
#define BLDC_TWO_PI    6.28318530717959f
#define BLDC_SQRT3     1.73205080756888f
#define BLDC_INV_SQRT3 0.577350269189626f
#define BLDC_UV_RESUME 1.1f   /* after an undervoltage lockout, restart 10 % above the threshold */

/* ------------------------------------------------------------------ building blocks */

float bldc_clamp(float x, float lo, float hi);
float bldc_wrap_pi(float x);                       /* angle to (-pi, pi] */
void  bldc_clarke(float a, float b, float c, float *alpha, float *beta);
void  bldc_park(float alpha, float beta, float cos_t, float sin_t, float *d, float *q);
void  bldc_inverse_park(float d, float q, float cos_t, float sin_t, float *alpha, float *beta);
void  bldc_inverse_clarke(float alpha, float beta, float abc[3]);
void  bldc_svpwm(float alpha, float beta, float v_dc, float duty[3]);

typedef struct {
    float kp;
    float ki;      /* integral gain (1/s) */
    float ts;      /* sample period (s) */
    float integ;   /* integrator state */
} bldc_pi_t;

void  bldc_pi_init(bldc_pi_t *pi, float kp, float ki, float ts);
float bldc_pi_step(bldc_pi_t *pi, float error, float out_min, float out_max, float ff);

/* ------------------------------------------------------------------ interface */

typedef struct {
    float   i_abc[3];    /* measured phase currents (A) */
    float   v_dc;        /* measured DC-link voltage (V) */
    float   theta_e;     /* encoder: electrical angle of the d axis (rad) — FOC */
    uint8_t hall;        /* Hall code 4*Ha + 2*Hb + Hc — six-step */
    float   reference;   /* speed (mechanical rad/s), or current (A) in torque mode */
} bldc_inputs_t;

typedef struct {
    float   duty[3];     /* high-side duty cycle of each leg, 0..1 */
    uint8_t enable[3];   /* 1: leg switching (complementary), 0: both switches off */
} bldc_outputs_t;

/* ------------------------------------------------------------------ FOC */

typedef struct {
    float    ts;                /* control period (s) */
    float    kp_i, ki_i;        /* current PI (V/A, V/(A.s)) */
    float    kp_w, ki_w;        /* speed PI (A per rad/s, A/rad) */
    float    l;                 /* inductance for the decoupling (H) */
    float    psi_m;             /* magnet flux linkage for the feed-forward (Wb) */
    float    i_max;             /* q-current limit (A) */
    float    accel_max;         /* reference ramp (rad/s^2) */
    float    speed_alpha;       /* speed filter, y += alpha (x - y) */
    float    modulation_limit;  /* max phase-voltage amplitude / V_dc */
    float    angle_comp;        /* angle advance (periods) */
    float    dt_comp;           /* dead-time compensation (duty), 0: off */
    float    dt_comp_band;      /* current band of the compensation (A) */
    float    v_dc_min;          /* undervoltage lockout (V) */
    uint16_t pole_pairs;
    uint16_t speed_div;         /* speed loop every speed_div periods */
    uint16_t calib_periods;     /* offset calibration at start-up */
    uint8_t  speed_loop;        /* 0: torque mode */
} bldc_foc_config_t;

typedef struct {
    float   i_d, i_q, i_d_ref, i_q_ref, v_d, v_q;
    float   speed_est, ref_ramped;
    float   offset[3];
    uint8_t running;
    uint8_t fault;              /* 1: invalid configuration, legs kept off */
} bldc_foc_telemetry_t;

typedef struct {
    bldc_foc_config_t cfg;
    bldc_pi_t pi_d, pi_q, pi_w;
    float     offset_acc[3];
    float     offset[3];
    float     theta_prev, speed_est, ref_ramped;
    float     i_d, i_q, i_d_ref, i_q_ref, v_d, v_q;
    uint16_t  calib_count, div_count;
    uint8_t   started;
    uint8_t   undervoltage;
    uint8_t   fault;
} bldc_foc_t;

/* Returns 0, or -1 if the configuration is invalid: the controller then keeps every leg off. */
int    bldc_foc_init(bldc_foc_t *foc, const bldc_foc_config_t *cfg);
void   bldc_foc_step(bldc_foc_t *foc, const bldc_inputs_t *in, bldc_outputs_t *out);
void   bldc_foc_telemetry(const bldc_foc_t *foc, bldc_foc_telemetry_t *tm);
size_t bldc_foc_size(void);

/* ------------------------------------------------------------------ six-step */

#define BLDC_SIX_EDGES 6u

typedef struct {
    float    ts;
    float    kp_i, ki_i;        /* current PI (V/A, V/(A.s)) */
    float    kp_w, ki_w;        /* speed PI */
    float    i_max;             /* current limit (A) */
    float    accel_max;         /* reference ramp (rad/s^2) */
    float    hall_timeout;      /* time without Hall edge read as standstill (s) */
    float    v_dc_min;          /* undervoltage lockout (V) */
    uint16_t pole_pairs;
    uint16_t speed_div;
    uint16_t calib_periods;
    uint8_t  speed_loop;
} bldc_six_config_t;

typedef struct {
    float   i_ref, i_meas, v_cmd, duty, speed_est, ref_ramped;
    float   offset[3];
    uint8_t running;
    uint8_t fault;              /* 1: invalid configuration, legs kept off */
} bldc_six_telemetry_t;

typedef struct {
    bldc_six_config_t cfg;
    bldc_pi_t pi_i, pi_w;
    float     offset_acc[3];
    float     offset[3];
    float     speed_est, ref_ramped, i_ref, i_meas, v_cmd, duty;
    uint16_t  intervals[BLDC_SIX_EDGES];
    uint16_t  edge_count, calib_count, div_count;
    uint8_t   n_intervals, idx, have_edge, hall_prev;
    uint8_t   undervoltage;
    uint8_t   fault;
} bldc_six_t;

/* Returns 0, or -1 if the configuration is invalid: the controller then keeps every leg off. */
int    bldc_six_init(bldc_six_t *six, const bldc_six_config_t *cfg);
void   bldc_six_step(bldc_six_t *six, const bldc_inputs_t *in, bldc_outputs_t *out);
void   bldc_six_telemetry(const bldc_six_t *six, bldc_six_telemetry_t *tm);
size_t bldc_six_size(void);

/* Legs driven by a Hall code: high (PWM) and low; -1 for the invalid codes 0 and 7. */
int bldc_six_legs(uint8_t hall, int *hi, int *lo);

/* Layout of the interface structures, so that a binding from another language can check its
 * mirror: for inputs, outputs, FOC config and telemetry, six-step config and telemetry in turn,
 * the size of the structure then the offset of each field in declaration order. Copies up to
 * n values into out (out may be NULL) and returns the total count. */
size_t bldc_interface_layout(size_t *out, size_t n);

#ifdef __cplusplus
}
#endif

#endif /* BLDC_CTRL_H */
