/*
 * imu_lift.h
 * Subsystem 4 (Buddy 4): lift / overturn detection (prof's change,
 * 2026-11-13: collision detection removed; lifting the car ends the
 * demo run, so a lifted or overturned car must stop its motors).
 *
 * Pure functions, no kernel calls: also built by the PC tests in
 * testing/subsystem4/host/. Called once per IMU sample (10 ms) by the
 * imu_terrain task, which owns the state (no lock).
 *
 * LIFTED: the accelerometer cannot see height, but it sees the upward
 * acceleration of a lift. dev = upward acceleration (g_ref = |a| at
 * level zero). Since 2026-11-16 the caller passes the acceleration
 * ALONG GRAVITY, (a . f) / |f| - g_ref (imu_terrain.c imu_vert_mg()),
 * not |a| - g_ref: the line follower's jolts made |a| as large as a
 * hand lift (run 10), but cancel along gravity (run 11, replay with vz).
 * A slow baseline of dev (time constant about 2.6 s) is
 * subtracted first, so a bias that remains or drifts cancels; then the
 * rest is integrated with a leak:
 *     sum += (dev - base) - sum / IMU_LIFT_LEAK_DIV   (mg x samples)
 * sum x 0.0981 is an estimate of the upward speed in mm/s (1 mg x 10 ms
 * = 0.0981 mm/s). LIFTED when the estimate reaches IMU_LIFT_VEL_MM_S.
 * A short spike (ramp edge, bump) adds little and is followed by an
 * opposite spike, so it does not reach the threshold; a hand lift keeps
 * accelerating upwards for 100-300 ms.
 * CLAMP (2026-11-16, measured): every dev sample is limited to
 * +-IMU_LIFT_CLAMP_MG before it is used. Car traces (logs
 * 2026-11-16_lift_run03/04): false lifts were single jolts / knocks of
 * 1000-1500 mg lasting 20-40 ms (clipping the +-2 g range, and each
 * 50 Hz accelerometer reading is seen twice by the 100 Hz task), while a
 * real hand lift was 443-885 mg sustained for about 100 ms. With the
 * clamp one spike can add at most 2 x IMU_LIFT_CLAMP_MG to the sum;
 * a sustained lift still adds plenty. Replay: replay_lift.c.
 * IMPACT VETO (2026-11-16, measured): the clamp alone let through a
 * longer bang (run 06 block 3: 2451 / 1191 / 1052 / 479 mg, ~80 ms over
 * 1 g, after the nose dipped). Every false event so far peaked at
 * >= 1028 mg with clipping; every hand lift at <= 885 mg without. So a
 * sample with |dev| >= IMU_LIFT_IMPACT_MG zeroes the speed estimate and
 * holds it at zero for IMU_LIFT_IMPACT_HOLD samples (200 ms).
 * LIMIT: a lift that starts with a jerk of 1 g or more is ignored for
 * those 200 ms; lift smoothly (it is a hand lift, not a throw).
 * LIMIT (documented, derived): a hump climbed faster than about
 * IMU_LIFT_VEL_MM_S vertically would read as a lift, and a lift slower
 * than that is not seen. Demo instruction: lift the car briskly, at
 * least 10 cm in about half a second.
 *
 * OVERTURNED: pitch or roll against level beyond IMU_OVERTURN_DDEG for
 * IMU_OVERTURN_SAMPLES samples in a row (humps tilt the car < 20 deg).
 *
 * Both LATCH. They clear only when the car is level (both tilts within
 * IMU_SETTLE_DDEG) and still (|dev| averaged over about 8 samples
 * <= IMU_SETTLE_MG) for
 * IMU_SETTLE_SAMPLES samples in a row: the "put down" result. The motor
 * stop itself is latched by the Vehicle Controller and released only
 * with the GP20 button, so putting the car down does not restart it.
 *
 * All thresholds are DERIVED from the PC simulation
 * (testing/subsystem4/host/test_lift.c); tune them on the car (S4-U38..
 * U41) and record the measured values.
 */

#ifndef IMU_LIFT_H
#define IMU_LIFT_H

#include <stdbool.h>
#include <stdint.h>

#ifndef IMU_LIFT_LEAK_DIV
#define IMU_LIFT_LEAK_DIV       (32)        // leak time constant 320 ms
#endif
#ifndef IMU_LIFT_VEL_MM_S
#define IMU_LIFT_VEL_MM_S       (200)       // estimated upward speed
#endif
#ifndef IMU_LIFT_CLAMP_MG
#define IMU_LIFT_CLAMP_MG       (300)       // per-sample limit, measured
#endif
#ifndef IMU_LIFT_IMPACT_MG
#define IMU_LIFT_IMPACT_MG      (1000)      // |dev| at or above: impact
#endif
#define IMU_LIFT_IMPACT_HOLD    (20U)       // 200 ms vetoed after it
// sum threshold = speed / 0.0981 = speed x 10000 / 981
#define IMU_LIFT_SUM_THRESH     ((IMU_LIFT_VEL_MM_S * 10000) / 981)
#define IMU_LIFT_BASE_DIV       (256)       // baseline: about 2.6 s
#define IMU_LIFT_BASE_Q         (16)        // baseline kept in mg x 16
#define IMU_SETTLE_AVG_DIV      (8)         // stillness: about 80 ms
#define IMU_OVERTURN_DDEG       (450)       // 45 deg
#define IMU_OVERTURN_SAMPLES    (20U)       // 200 ms
#define IMU_SETTLE_DDEG         (100)       // 10 deg
#define IMU_SETTLE_MG           (60)
#define IMU_SETTLE_SAMPLES      (100U)      // 1 s

typedef enum
{
    IMU_LIFT_NONE = 0,          // nothing new
    IMU_LIFT_LIFTED,            // just detected: lifted
    IMU_LIFT_OVERTURNED,        // just detected: overturned
    IMU_LIFT_PUT_DOWN           // latched condition just cleared
} imu_lift_result_t;

typedef struct
{
    int32_t  vel_sum;           // leaky sum of dev, mg x samples
    int32_t  base_q;            // slow baseline of dev, mg x 16
    int32_t  calm_avg;          // short average of dev - base, mg
    int32_t  peak_sum;          // highest vel_sum since last read
    uint32_t hold_count;        // samples left in an impact veto
    uint32_t tilt_count;        // samples in a row beyond OVERTURN
    uint32_t calm_count;        // samples in a row level and still
    bool     b_latched;         // LIFTED or OVERTURNED in force
} imu_lift_t;

void              imu_lift_init(imu_lift_t *p_lift);
imu_lift_result_t imu_lift_update(imu_lift_t *p_lift, int32_t dev_mg,
                                  int32_t pitch_ddeg, int32_t roll_ddeg);
int32_t           imu_lift_vel_mm_s(int32_t sum);
int32_t           imu_lift_take_peak(imu_lift_t *p_lift);

#endif // IMU_LIFT_H
