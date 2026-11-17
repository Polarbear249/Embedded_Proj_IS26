/*
 * PC test for subsystem4/imu_lift.c (lift / overturn -> motor stop,
 * prof's change 2026-11-13). Runs the production imu_lift.c.
 *
 * Each scenario feeds one sample per 10 ms, as imu_terrain.c does:
 * dev_mg = |a| - g_ref, and the filtered pitch/roll against level.
 * Vertical motion z(t) of the sensor is turned into acceleration by a
 * second difference (mm/s^2 -> mg: / 9.81). Noise is Gaussian.
 * Everything here is DERIVED, not measured: it shows the thresholds are
 * consistent, the car runs (S4-U38..U41) set the real margins.
 *
 * Build and run: ./run_tests.sh
 */
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include "../../../app_program/subsystem4/imu_lift.h"

#define PI          3.14159265358979
#define DT          0.010
#define MM_S2_PER_MG 9.81

static int g_fail = 0;

typedef struct {
    int lifted, overturned, put_down;
    int first_ms;               /* first LIFTED/OVERTURNED, -1 none */
    double z_at_first;          /* sensor height then, mm */
    int peak_vel;               /* highest estimate, mm/s */
} result_t;

static double gauss(double sd)
{
    double u1 = (rand() + 1.0) / (RAND_MAX + 2.0);
    double u2 = (rand() + 1.0) / (RAND_MAX + 2.0);
    return sd * sqrt(-2.0 * log(u1)) * cos(2.0 * PI * u2);
}

/* vertical position (mm) and tilt (0.1 deg) as functions of time */
typedef double (*fz_t)(double t);
typedef double (*ft_t)(double t);

static result_t run(fz_t fz, ft_t ft, double secs, double acc_sd,
                    double tilt_sd, double bias_mg)
{
    imu_lift_t lift;
    result_t   r = {0, 0, 0, -1, 0.0, 0};
    int        n = (int)(secs / DT);
    int        i;

    imu_lift_init(&lift);
    for (i = 0; i < n; i++) {
        double t = i * DT;
        double a = (fz(t + DT) - 2.0 * fz(t) + fz(t - DT)) / (DT * DT);
        int32_t dev = (int32_t)lround(a / MM_S2_PER_MG + bias_mg
                                      + gauss(acc_sd));
        int32_t tilt = (int32_t)lround(ft(t) + gauss(tilt_sd));
        imu_lift_result_t res = imu_lift_update(&lift, dev, tilt, 0);
        int vel = imu_lift_vel_mm_s(lift.vel_sum);

        if (vel > r.peak_vel) { r.peak_vel = vel; }
        if (IMU_LIFT_LIFTED == res) { r.lifted++; }
        if (IMU_LIFT_OVERTURNED == res) { r.overturned++; }
        if (IMU_LIFT_PUT_DOWN == res) { r.put_down++; }
        if (((IMU_LIFT_LIFTED == res) || (IMU_LIFT_OVERTURNED == res))
            && (r.first_ms < 0)) {
            r.first_ms = (int)(t * 1000.0 + 0.5);
            r.z_at_first = fz(t);
        }
    }
    return r;
}

static void check(char const *id, char const *name, int ok, result_t r)
{
    printf("%-7s %-44s lift %d over %d down %d  t %5d ms  z %5.1f mm"
           "  peak %4d mm/s  %s\n", id, name, r.lifted, r.overturned,
           r.put_down, r.first_ms, r.z_at_first, r.peak_vel,
           ok ? "PASS" : "FAIL");
    if (!ok) { g_fail++; }
}

/* ---- scenarios ------------------------------------------------------ */

static double z_flat(double t) { (void)t; return 0.0; }
static double t_level(double t) { (void)t; return 0.0; }

/* hand lift: smooth (cosine) rise of H mm over T s, starting at 0.5 s */
static double g_h, g_t;
static double z_lift(double t)
{
    double u = t - 0.5;
    if (u <= 0.0) { return 0.0; }
    if (u >= g_t) { return g_h; }
    return g_h * 0.5 * (1.0 - cos(PI * u / g_t));
}

/* hump: car at V mm/s over a trapezoid (ramp R mm, top 60 mm, height H) */
static double g_v, g_r;
static double z_hump(double t)
{
    double x = g_v * t - 100.0;
    double top = 60.0;
    if (x <= 0.0) { return 0.0; }
    if (x < g_r) { return g_h * x / g_r; }
    if (x < g_r + top) { return g_h; }
    if (x < 2.0 * g_r + top) { return g_h * (2.0 * g_r + top - x) / g_r; }
    return 0.0;
}

/* ramp-edge bangs: a short spike every 0.5 s (run 40: 421-1215 mg) */
static double z_bangs(double t) { (void)t; return 0.0; }

/* overturn: tilt from 0 to 90 deg over 1 s, from 0.5 s */
static double t_over(double t)
{
    double u = t - 0.5;
    if (u <= 0.0) { return 0.0; }
    if (u >= 1.0) { return 900.0; }
    return 900.0 * u;
}
static double t_30(double t) { return (t > 0.5) ? 300.0 : 0.0; }

/* lift 100 mm, hold 1 s in the air tilted 15 deg, put down, still */
static double z_lift_hold(double t)
{
    if (t < 3.0) { g_h = 100.0; g_t = 0.4; return z_lift(t); }
    if (t < 3.4) { return 100.0 * 0.5 * (1.0 + cos(PI * (t - 3.0) / 0.4)); }
    return 0.0;
}
static double t_hold(double t) { return ((t > 1.0) && (t < 3.4)) ? 150.0 : 0.0; }

int main(void)
{
    result_t r;
    int      i;

    srand(4242);
    printf("IMU_LIFT_VEL_MM_S %d, LEAK_DIV %d, SUM %d\n",
           IMU_LIFT_VEL_MM_S, IMU_LIFT_LEAK_DIV, IMU_LIFT_SUM_THRESH);

    /* must NOT stop the car */
    r = run(z_flat, t_level, 60.0, 60.0, 20.0, 0.0);
    check("LIFT-01", "flat driving 60 s, noise 60 mg / 2 deg", r.lifted + r.overturned == 0, r);
    r = run(z_flat, t_level, 60.0, 60.0, 20.0, 40.0);
    check("LIFT-02", "same with a 40 mg uncorrected bias", r.lifted + r.overturned == 0, r);

    {   /* ramp-edge bangs: +1215 mg for 1 sample, -600 mg for 2 */
        imu_lift_t lift;
        int bad = 0, peak = 0;
        imu_lift_init(&lift);
        for (i = 0; i < 6000; i++) {
            int k = i % 50;
            int32_t dev = (k == 0) ? 1215 : ((k <= 2) ? -600 : 0);
            dev += (int32_t)lround(gauss(60.0));
            if (IMU_LIFT_NONE != imu_lift_update(&lift, dev, 0, 0)) { bad++; }
            if (imu_lift_vel_mm_s(lift.vel_sum) > peak) { peak = imu_lift_vel_mm_s(lift.vel_sum); }
        }
        r.lifted = bad; r.overturned = 0; r.put_down = 0; r.first_ms = -1;
        r.z_at_first = 0.0; r.peak_vel = peak;
        check("LIFT-03", "ramp-edge bangs every 0.5 s for 60 s", bad == 0, r);
    }
    (void)z_bangs;

    g_h = 10.0; g_r = 40.0; g_v = 200.0;
    r = run(z_hump, t_level, 3.0, 60.0, 20.0, 0.0);
    check("LIFT-04", "10 mm ramp at 200 mm/s (climb 50 mm/s)", r.lifted == 0, r);
    g_h = 25.0; g_r = 50.0; g_v = 300.0;
    r = run(z_hump, t_level, 3.0, 60.0, 20.0, 0.0);
    check("LIFT-05", "25 mm hump, 50 mm ramp, 300 mm/s (150 mm/s)", r.lifted == 0, r);
    r = run(z_flat, t_30, 3.0, 60.0, 20.0, 0.0);
    check("LIFT-06", "held at 30 deg tilt (steep hump) 2.5 s", r.overturned == 0, r);

    /* MUST stop the car */
    g_h = 100.0; g_t = 0.5;
    r = run(z_lift, t_level, 2.0, 60.0, 20.0, 0.0);
    check("LIFT-10", "hand lift 100 mm in 0.5 s", r.lifted == 1 && r.first_ms < 900, r);
    g_h = 80.0; g_t = 0.4;
    r = run(z_lift, t_level, 2.0, 60.0, 20.0, 0.0);
    check("LIFT-11", "brisk lift 80 mm in 0.4 s", r.lifted == 1, r);
    g_h = 50.0; g_t = 0.3;
    r = run(z_lift, t_level, 2.0, 60.0, 20.0, 0.0);
    check("LIFT-15", "info: short lift 50 mm in 0.3 s", 1, r);
    g_h = 200.0; g_t = 0.6;
    r = run(z_lift, t_level, 2.0, 60.0, 20.0, 0.0);
    check("LIFT-12", "big lift 200 mm in 0.6 s", r.lifted == 1, r);
    g_h = 100.0; g_t = 1.5;
    r = run(z_lift, t_level, 3.0, 60.0, 20.0, 0.0);
    check("LIFT-13", "info: slow lift 100 mm in 1.5 s", 1, r);
    r = run(z_flat, t_over, 3.0, 60.0, 20.0, 0.0);
    check("LIFT-14", "tipped over to 90 deg in 1 s", r.overturned == 1 && r.lifted == 0, r);

    /* latch and put down */
    r = run(z_lift_hold, t_hold, 6.0, 60.0, 20.0, 0.0);
    check("LIFT-20", "lift, hold 2 s tilted, put down, still 2.6 s", r.lifted == 1 && r.put_down == 1, r);
    {
        imu_lift_t lift;
        int down = 0;
        imu_lift_init(&lift);
        for (i = 0; i < 40; i++) { (void)imu_lift_update(&lift, 0, 900, 0); }
        for (i = 0; i < 300; i++) {   /* still but tilted: stays latched */
            if (IMU_LIFT_PUT_DOWN == imu_lift_update(&lift, 0, 300, 0)) { down++; }
        }
        r.lifted = 0; r.overturned = 1; r.put_down = down; r.first_ms = -1;
        r.z_at_first = 0.0; r.peak_vel = 0;
        check("LIFT-21", "overturned, then still at 30 deg: stays latched", down == 0 && lift.b_latched, r);
    }

    /* conversions */
    {
        int ok = imu_lift_vel_mm_s(IMU_LIFT_SUM_THRESH) <= IMU_LIFT_VEL_MM_S
                 && imu_lift_vel_mm_s(IMU_LIFT_SUM_THRESH + 11) >= IMU_LIFT_VEL_MM_S
                 && imu_lift_update(NULL, 5000, 900, 0) == IMU_LIFT_NONE;
        printf("LIFT-30 threshold <-> mm/s conversion, NULL safe       %s\n", ok ? "PASS" : "FAIL");
        if (!ok) { g_fail++; }
    }

    printf(g_fail ? "LIFT FAILED (%d)\n" : "LIFT ALL PASS\n", g_fail);
    return g_fail ? 1 : 0;
}
