/*
 * PC test for subsystem4/imu_hump.c (hump height, FR7) and the IF-11
 * word packing. Runs the production imu_hump.c + imu_calib.c.
 *
 * A simulated car (rear axle and ball caster IMU_HUMP_BASE_MM apart)
 * drives over ground profiles. For each position the true pitch comes
 * from the geometry (front contact found on the ground, L from the rear
 * contact); the wheel distance is the arc length of the rear contact's
 * path. Every 10 ms (2 mm at 200 mm/s) the pitch, after the same IIR as
 * the task (y += (x - y) / 4) plus optional noise and bias, and the
 * distance go to imu_hump_update(), as in imu_terrain.c.
 *
 * Build and run: ./run_tests.sh
 */
#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include "../../../app_program/subsystem4/imu_hump.h"

#define PI          3.14159265358979
#define L_MM        ((double)IMU_HUMP_BASE_MM)
#define SPEED       200.0       /* mm/s */
#define CYCLE_S     0.010

typedef struct {
    double start, up, top, down, height;   /* trapezoid, mm */
    int    cosine;                         /* 1: smooth bump instead */
} profile_t;

static double ground(profile_t const *p, double x)
{
    double u = x - p->start;
    double len = p->up + p->top + p->down;

    if (u <= 0.0 || u >= len) {
        return 0.0;
    }
    if (p->cosine) {
        return p->height * 0.5 * (1.0 - cos(2.0 * PI * u / len));
    }
    if (u < p->up) {
        return p->height * u / p->up;
    }
    if (u < p->up + p->top) {
        return p->height;
    }
    return p->height * (len - u) / p->down;
}

/* Front contact: the point on the ground L away from the rear contact */
static double front_x(profile_t const *p, double xr)
{
    double lo = xr, hi = xr + L_MM, zr = ground(p, xr);
    int i;

    for (i = 0; i < 60; i++) {
        double mid = 0.5 * (lo + hi);
        double d = hypot(mid - xr, ground(p, mid) - zr);
        if (d < L_MM) lo = mid; else hi = mid;
    }
    return 0.5 * (lo + hi);
}

typedef struct {
    double noise_ddeg;  /* uniform +- noise on pitch */
    double bias_ddeg;   /* level error */
    int    stationary;  /* 1: never move, tilt instead */
} sim_opts_t;

/* Drive length mm over profile p; returns the hump word fields */
static imu_hump_t run(profile_t const *p, double length, sim_opts_t o,
                      int *p_starts, int *p_ends, double *p_true_max)
{
    imu_hump_t hump;
    double xr = 0.0, s = 0.0, next_cycle = 0.0, filt = 0.0;
    double step = 0.05;
    imu_event_t ev;

    imu_hump_init(&hump);
    *p_starts = 0;
    *p_ends = 0;
    *p_true_max = 0.0;

    while (xr < length) {
        double xf = front_x(p, xr);
        double zr = ground(p, xr), zf = ground(p, xf);
        double pitch = atan2(zf - zr, xf - xr) * 1800.0 / PI;

        if (zf > *p_true_max) *p_true_max = zf;

        if (s >= next_cycle) {
            double meas = pitch + o.bias_ddeg
                + o.noise_ddeg * (2.0 * rand() / (double)RAND_MAX - 1.0);
            filt += (meas - filt) / 4.0;
            ev = imu_hump_update(&hump, (int32_t)lround(filt),
                                 o.stationary ? 0 : (int32_t)floor(s));
            if (ev == IMU_EVT_HUMP_START) (*p_starts)++;
            if (ev == IMU_EVT_HUMP_END) (*p_ends)++;
            next_cycle += SPEED * CYCLE_S;
        }

        if (o.stationary) {
            s += step;              /* time passes, distance does not */
            xr += step;             /* (pitch from the profile only) */
        } else {
            double nx = xr + step;
            s += hypot(step, ground(p, nx) - zr);
            xr = nx;
        }
    }
    return hump;
}

static int check(char const *name, profile_t p, double length,
                 sim_opts_t o, int want_humps, double tol_mm)
{
    int starts, ends;
    double true_max;
    imu_hump_t h = run(&p, length, o, &starts, &ends, &true_max);
    double err = (double)h.last_mm - true_max;
    int ok = (ends == want_humps) && (starts == want_humps)
             && (want_humps == 0 ? h.max_mm == 0 : fabs(err) <= tol_mm);

    printf("%-34s true %6.1f mm  measured %3u mm  humps %d/%d  %s\n",
           name, true_max, (unsigned)h.last_mm, starts, ends,
           ok ? "PASS" : "FAIL");
    return ok;
}

static int check_pack(void)
{
    imu_hump_t h;
    uint32_t w;
    int ok = 1;

    imu_hump_init(&h);
    h.max_mm = 255U; h.last_mm = 23U; h.b_valid = true;
    w = imu_hump_pack(&h, 4U, IMU_EVT_HUMP_END, 200U);
    ok &= IMU_HUMP_W_MAX(w) == 255U && IMU_HUMP_W_LAST(w) == 23U
          && IMU_HUMP_W_STATE(w) == 4U && IMU_HUMP_W_EVENT(w) == 2U
          && IMU_HUMP_W_VALID(w) == 1U && IMU_HUMP_W_SEQ(w) == 200U;

    /* out-of-range inputs are masked, never spill into other fields */
    h.b_valid = false;
    w = imu_hump_pack(&h, 0x1FU, 0x0FU, 0x1FFU);
    ok &= IMU_HUMP_W_STATE(w) == 0x0FU && IMU_HUMP_W_EVENT(w) == 0x07U
          && IMU_HUMP_W_VALID(w) == 0U && IMU_HUMP_W_SEQ(w) == 0xFFU
          && IMU_HUMP_W_LAST(w) == 23U && IMU_HUMP_W_MAX(w) == 255U;

    printf("%-34s %s\n", "IF-11 word pack/unpack round trip",
           ok ? "PASS" : "FAIL");
    return ok;
}

int main(void)
{
    sim_opts_t clean = {0.0, 0.0, 0};
    sim_opts_t noisy = {3.0, 2.0, 0};       /* +-0.3 deg noise, 0.2 bias */
    int ok = 1;

    srand(1);
    /* 23 mm block with 30 mm ramps (the bench block's height) */
    ok &= check("23 mm hump, short ramps", (profile_t){300, 30, 40, 30,
                23, 0}, 800, clean, 1, 1.5);
    ok &= check("23 mm hump, noise + bias", (profile_t){300, 30, 40, 30,
                23, 0}, 800, noisy, 1, 2.0);
    /* longer than the wheelbase: the recurrence must add up */
    ok &= check("30 mm long hump (200 mm ramps)", (profile_t){300, 200,
                150, 200, 30, 0}, 1200, clean, 1, 1.5);
    ok &= check("40 mm smooth bump (120 mm)", (profile_t){300, 60, 0, 60,
                40, 1}, 900, clean, 1, 2.0);
    /* small: max pitch asin(8/85) = 5.4 deg, above the 3 deg start */
    ok &= check("8 mm hump", (profile_t){300, 25, 30, 25, 8, 0}, 800,
                clean, 1, 1.0);
    /* flat, noisy, level error: no hump at all over 2 m */
    ok &= check("flat 2 m, noise + bias", (profile_t){300, 30, 40, 30, 0,
                0}, 2000, noisy, 0, 0.0);
    /* car tilted by hand without moving (pitch from a 23 mm hump) */
    {
        sim_opts_t still = {0.0, 0.0, 1};
        ok &= check("tilted, not moving", (profile_t){300, 30, 40, 30, 23,
                    0}, 800, still, 0, 0.0);
    }
    /* 600 mm ramps up to a 300 mm plateau: clamp at 255, never wrap.
       The 300 mm top is longer than IMU_HUMP_END_MM, so the way down is
       a second event; it has no height and must not be reported. */
    {
        int st, en;
        double t;
        profile_t big = {300, 600, 300, 600, 300, 0};
        imu_hump_t h = run(&big, 2200, clean, &st, &en, &t);
        int c = (en == 1) && (h.last_mm == 255U) && (h.max_mm == 255U);
        printf("%-34s true %6.1f mm  measured %3u mm  %s\n",
               "300 mm ramp: clamp at 255", t, (unsigned)h.last_mm,
               c ? "PASS" : "FAIL");
        ok &= c;
    }
    ok &= check_pack();

    puts(ok ? "ALL PASS" : "SOME FAILED");
    return ok ? 0 : 1;
}
