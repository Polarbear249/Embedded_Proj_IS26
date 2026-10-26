/*
 * imu_unit_test.c
 * Subsystem 4 (Buddy 4): on-target IMU tests, called from
 * subsystem4_test(). Built only with: make TEST=1. Record results in
 * testing/subsystem4/TEST_LOG.md.
 *
 * GY-511 = LSM303D at 0x1D on I2C1 (SDA GP2 / SCL GP3). Every line
 * starts with [IMU] or [TEST].
 *
 * Part 1 (bench): bus, identity, configuration, one sample at rest.
 * Part 2 (S4-U17..U19, guided): finds which sensor axis points forward,
 * left and up on the car. Needs a book (or any flat block). The console
 * asks for four poses in turn:
 *   1 FLAT     car on the flat floor
 *   2 NOSE UP  front wheels on the book, rear wheels on the floor
 *   3 FLAT     car back on the floor (checks the module did not shift)
 *   4 LEFT UP  left wheels on the book, right wheels on the floor
 * At rest an accelerometer axis reads +1000 mg x cos(angle to straight
 * up), so the axis that points uphill reads MORE. Lifting the nose
 * raises the forward axis, lifting the left side raises the left axis.
 */

#include "imu_unit_test.h"
#include "drv_imu.h"
#include "../test_report.h"

#if APP_TEST

#define IMU_WAIT_TERM_MS        (120000)    // give up after 2 min
#define IMU_POLL_MS             (100)
#define IMU_SETTLE_MS           (1000)
#define IMU_FIRST_SAMPLE_MS     (100)
#define IMU_SCAN_FIRST_ADDR     (0x08U)     // 7-bit range that is not
#define IMU_SCAN_LAST_ADDR      (0x77U)     //   reserved by the I2C spec
#define IMU_1G_MIN_MG           (800)       // 1 g -20 %
#define IMU_1G_MAX_MG           (1200)      // 1 g +20 %
#define IMU_MAG_SAT_POS         (32767)     // full-scale reading
#define IMU_MAG_SAT_NEG         (-32768)
#define IMU_AXES                (3U)

// S4-U17..U19 guided axis test. Thresholds are test choices, not
// datasheet values: noise at rest measured about +-10 mg (run 5).
#define IMU_POSE_MOVE_MS        (5000)      // time to place the car
#define IMU_POSE_HANDS_OFF_MS   (3000)      // then hands off
#define IMU_POSE_SAMPLES        (50)        // 50 x 20 ms = 1 s average
#define IMU_POSE_MS             (20)        //   (one 50 Hz sample each)
#define IMU_POSE_TRIES          (3U)        // retries if the car moved
#define IMU_STILL_SPREAD_MG     (60)        // max - min on every axis
#define IMU_STILL_MIN_MG        (900)       // |a| 1 g -10 %
#define IMU_STILL_MAX_MG        (1100)      // |a| 1 g +10 %
#define IMU_REPEAT_MG           (30)        // flat 1 vs flat 3, per axis
#define IMU_MIN_DELTA_MG        (50)        // about 3 deg of tilt
#define IMU_UP_MIN_MG           (500)       // up axis: at least 0.5 g
#define IMU_DOMINANT_RATIO      (2)         // main axis >= 2 x the next
#define IMU_1G_MG               (1000)

#define IMU_POSE_FLAT           (0U)
#define IMU_POSE_NOSE_UP        (1U)
#define IMU_POSE_FLAT_AGAIN     (2U)
#define IMU_POSE_LEFT_UP        (3U)
#define IMU_POSES               (4U)

// Integer atan from a tan(k deg) x 10000 table, k = 0..45
#define IMU_TAN_SCALE           (10000U)
#define IMU_TAN_STEPS           (45U)
#define IMU_DDEG_PER_DEG        (10U)       // results in 0.1 deg units
#define IMU_DDEG_90             (900U)
#define IMU_DDEG_180            (1800U)

static uint16_t const g_tan_x10000[IMU_TAN_STEPS + 1U] =
{
       0U,  175U,  349U,  524U,  699U,  875U, 1051U, 1228U, 1405U, 1584U,
    1763U, 1944U, 2126U, 2309U, 2493U, 2679U, 2867U, 3057U, 3249U, 3443U,
    3640U, 3839U, 4040U, 4245U, 4452U, 4663U, 4877U, 5095U, 5317U, 5543U,
    5774U, 6009U, 6249U, 6494U, 6745U, 7002U, 7265U, 7536U, 7813U, 8098U,
    8391U, 8693U, 9004U, 9325U, 9657U, 10000U
};

static char const * const g_pose_prompt[IMU_POSES] =
{
    "FLAT: car on the flat floor",
    "NOSE UP: front wheels on the book, rear wheels on the floor",
    "FLAT again: car back on the floor, same spot",
    "LEFT UP: left wheels on the book, right wheels on the floor"
};

// [0] = negative direction, [1] = positive direction
static char const * const g_axis_name[2][IMU_AXES] =
{
    {"-X", "-Y", "-Z"},
    {"+X", "+Y", "+Z"}
};

#if TM_CONSOLE_USB_CDC
// libtm USB glue: true once a terminal has opened the port (DTR)
extern int tm_tusb_connected(void);
#endif

static void     imu_wait_for_terminal(void);
static bool     imu_scan_bus(void);
static bool     imu_mag_axis_ok(int16_t value);
static void     imu_bench_tests(void);
static void     imu_axis_tests(void);
static bool     imu_capture_pose(uint32_t pose, int32_t p_avg[]);
static bool     imu_measure_still(int32_t p_avg[]);
static uint32_t imu_main_axis(int32_t const p_vec[], int32_t min_mg,
                              bool *p_b_clear);
static bool     imu_right_handed(uint32_t fwd, bool b_fwd_pos,
                                 uint32_t left, bool b_left_pos,
                                 uint32_t up, bool b_up_pos);
static void     imu_print_expected_left(uint32_t fwd, bool b_fwd_pos,
                                        uint32_t up, bool b_up_pos);
static void     imu_print_ddeg(char const *p_label, uint32_t ddeg);
static int32_t  imu_abs(int32_t value);
static uint32_t imu_tilt_ddeg(int32_t const p_ref[], int32_t const p_now[]);
static uint32_t imu_angle_ddeg(uint64_t opposite, uint64_t adjacent);
static uint32_t imu_atan_ddeg(uint32_t tan_x10000);
static uint32_t imu_isqrt(uint64_t value);

void
imu_unit_test (void)
{
    imu_wait_for_terminal();
    tm_printf((UB*)"\n[IMU] ===== subsystem 4 IMU test (GY-511 / LSM303D)"
              " =====\n[IMU] keep the car flat and still\n");

    imu_bench_tests();
    imu_axis_tests();
}

/*
 * Part 1: bus, identity, configuration and one sample at rest.
 */
static void
imu_bench_tests (void)
{
    imu_sample_t  sample = {0};
    i2c1_result_t result;
    uint8_t       value = 0U;
    int32_t       mag2;
    bool          b_idle;
    bool          b_found;

    TEST_INIT();

    // S4-U09: both lines idle high before we start (pull-ups present)
    b_idle = i2c1_bus_is_idle();
    TEST_CHECK("S4-U09", b_idle, "I2C1 SDA/SCL idle high (pull-ups ok)");

    if (false == b_idle)
    {
        tm_printf((UB*)"[IMU] a line is low: check SDA->GP2, SCL->GP3,"
                  " VCC->3V3, GND\n");
    }

    // S4-U11: I2C1 starts and the sensor ACKs its configuration
    result = imu_init();
    TEST_CHECK("S4-U11", I2C1_OK == result,
               "I2C1 init + sensor configuration writes ACKed");
    tm_printf((UB*)"[IMU] init: %s (abort src 0x%x)\n",
              i2c1_result_name(result), (unsigned)i2c1_last_abort());

    // S4-U14: bus scan finds the LSM303D
    b_found = imu_scan_bus();
    TEST_CHECK("S4-U14", b_found, "bus scan finds the LSM303D at 0x1D");

    // S4-U12: identification
    result = imu_read_reg(IMU_WHO_AM_I, &value);
    tm_printf((UB*)"[IMU] WHO_AM_I: %s, 0x%02x (expect 0x%02x)\n",
              i2c1_result_name(result), value, IMU_WHO_AM_I_VAL);
    TEST_CHECK("S4-U12",
               (I2C1_OK == result) && (IMU_WHO_AM_I_VAL == value),
               "LSM303D WHO_AM_I reads 0x49");

    // S4-U13: configuration was stored
    value = 0U;
    result = imu_read_reg(IMU_CTRL1, &value);
    tm_printf((UB*)"[IMU] CTRL1: %s, 0x%02x (expect 0x%02x)\n",
              i2c1_result_name(result), value, IMU_CTRL1_VAL);
    TEST_CHECK("S4-U13", (I2C1_OK == result) && (IMU_CTRL1_VAL == value),
               "LSM303D configured (CTRL1 read back)");

    // S4-U06 / S4-U07: one sample at rest
    tk_dly_tsk(IMU_FIRST_SAMPLE_MS);
    result = imu_read(&sample);
    mag2 = ((int32_t)sample.acc_x_mg * sample.acc_x_mg)
           + ((int32_t)sample.acc_y_mg * sample.acc_y_mg)
           + ((int32_t)sample.acc_z_mg * sample.acc_z_mg);
    tm_printf((UB*)"[IMU] at rest: %s  acc mg x=%d y=%d z=%d"
              "  mag x=%d y=%d z=%d\n", i2c1_result_name(result),
              sample.acc_x_mg, sample.acc_y_mg, sample.acc_z_mg,
              sample.mag_x, sample.mag_y, sample.mag_z);

    // Lying still, the only acceleration is gravity: |a| = 1000 mg
    TEST_CHECK("S4-U06", (I2C1_OK == result)
               && (mag2 >= (IMU_1G_MIN_MG * IMU_1G_MIN_MG))
               && (mag2 <= (IMU_1G_MAX_MG * IMU_1G_MAX_MG)),
               "accel at rest reads 1 g +-20 %");

    TEST_CHECK("S4-U07", (I2C1_OK == result)
               && imu_mag_axis_ok(sample.mag_x)
               && imu_mag_axis_ok(sample.mag_y)
               && imu_mag_axis_ok(sample.mag_z)
               && ((0 != sample.mag_x) || (0 != sample.mag_y)
                   || (0 != sample.mag_z)),
               "magnetometer reads a field, no axis saturated");

    TEST_SUMMARY("subsystem4 IMU bench");
}

/*
 * Part 2: guided poses -> which sensor axis is forward, left and up.
 * Works from differences to the flat pose, so a crooked mount does not
 * change the answer, only the mount tilt printed at the end.
 */
static void
imu_axis_tests (void)
{
    int32_t  pose[IMU_POSES][IMU_AXES] = {{0}};
    int32_t  fwd_delta[IMU_AXES] = {0, 0, 0};
    int32_t  left_delta[IMU_AXES] = {0, 0, 0};
    int32_t  up_ref[IMU_AXES] = {0, 0, 0};
    uint32_t idx;
    uint32_t axis;
    uint32_t fwd;
    uint32_t left;
    uint32_t up;
    bool     b_all_still = true;
    bool     b_repeat = true;
    bool     b_fwd_clear = false;
    bool     b_left_clear = false;
    bool     b_up_clear = false;
    bool     b_fwd_pos;
    bool     b_left_pos;
    bool     b_up_pos;
    bool     b_frame_ok;

    TEST_INIT();

    tm_printf((UB*)"[IMU] ----- axis test: you need a book ~2-4 cm thick"
              " -----\n[IMU] for each pose: place the car, then hands off"
              " when told\n");

    for (idx = 0U; idx < IMU_POSES; idx++)
    {
        b_all_still = imu_capture_pose(idx, pose[idx]) && b_all_still;
    }

    TEST_CHECK("S4-U17", b_all_still,
               "all 4 poses held still (|a| 1 g +-10 %, spread <= 60 mg)");

    // S4-U18: flat -> nose up -> flat gives the same vector again
    for (axis = 0U; axis < IMU_AXES; axis++)
    {
        fwd_delta[axis] = pose[IMU_POSE_NOSE_UP][axis]
                          - pose[IMU_POSE_FLAT][axis];
        left_delta[axis] = pose[IMU_POSE_LEFT_UP][axis]
                           - pose[IMU_POSE_FLAT][axis];
        b_repeat = b_repeat
                   && (imu_abs(pose[IMU_POSE_FLAT_AGAIN][axis]
                               - pose[IMU_POSE_FLAT][axis])
                       <= IMU_REPEAT_MG);
    }

    tm_printf((UB*)"[IMU] flat 1 vs flat 3 (mg): dx=%d dy=%d dz=%d\n",
              (int)(pose[IMU_POSE_FLAT_AGAIN][0] - pose[IMU_POSE_FLAT][0]),
              (int)(pose[IMU_POSE_FLAT_AGAIN][1] - pose[IMU_POSE_FLAT][1]),
              (int)(pose[IMU_POSE_FLAT_AGAIN][2] - pose[IMU_POSE_FLAT][2]));
    TEST_CHECK("S4-U18", b_repeat,
               "flat pose repeats within 30 mg (module fixed on mount)");

    tm_printf((UB*)"[IMU] nose up - flat (mg): dx=%d dy=%d dz=%d\n",
              (int)fwd_delta[0], (int)fwd_delta[1], (int)fwd_delta[2]);
    tm_printf((UB*)"[IMU] left up - flat (mg): dx=%d dy=%d dz=%d\n",
              (int)left_delta[0], (int)left_delta[1], (int)left_delta[2]);

    // The axis that rose is the one pointing that way
    up = imu_main_axis(pose[IMU_POSE_FLAT], IMU_UP_MIN_MG, &b_up_clear);
    fwd = imu_main_axis(fwd_delta, IMU_MIN_DELTA_MG, &b_fwd_clear);
    left = imu_main_axis(left_delta, IMU_MIN_DELTA_MG, &b_left_clear);
    b_up_pos = (pose[IMU_POSE_FLAT][up] > 0);
    b_fwd_pos = (fwd_delta[fwd] > 0);
    b_left_pos = (left_delta[left] > 0);
    b_frame_ok = imu_right_handed(fwd, b_fwd_pos, left, b_left_pos,
                                  up, b_up_pos);

    tm_printf((UB*)"[IMU] AXES  forward = %s%s   left = %s%s   up = %s%s\n",
              g_axis_name[b_fwd_pos][fwd], b_fwd_clear ? "" : " (unclear)",
              g_axis_name[b_left_pos][left],
              b_left_clear ? "" : " (unclear)",
              g_axis_name[b_up_pos][up], b_up_clear ? "" : " (unclear)");

    if (false == b_frame_ok)
    {
        tm_printf((UB*)"[IMU] forward x left != up: a pose was wrong or"
                  " the module is turned on the car\n");
        imu_print_expected_left(fwd, b_fwd_pos, up, b_up_pos);
    }

    TEST_CHECK("S4-U19",
               b_fwd_clear && b_left_clear && b_up_clear && b_frame_ok,
               "forward/left/up axes found, one each, right-handed");

    // Tilt of each pose from flat; compare with asin(book / wheelbase)
    imu_print_ddeg("nose up tilt",
                   imu_tilt_ddeg(pose[IMU_POSE_FLAT],
                                 pose[IMU_POSE_NOSE_UP]));
    imu_print_ddeg("left up tilt",
                   imu_tilt_ddeg(pose[IMU_POSE_FLAT],
                                 pose[IMU_POSE_LEFT_UP]));

    // How far the module itself is off level when the car is flat
    up_ref[up] = b_up_pos ? IMU_1G_MG : -IMU_1G_MG;
    imu_print_ddeg("mount tilt (module vs level)",
                   imu_tilt_ddeg(up_ref, pose[IMU_POSE_FLAT]));
    tm_printf((UB*)"[IMU] expected: nose up = asin(h / front-rear axle"
              " distance)\n[IMU]           left up = asin(h / left-right"
              " wheel track)\n");

    TEST_SUMMARY("subsystem4 IMU axes");
}

/*
 * Prompt for one pose, give time to place the car, then average 1 s.
 * Retries if the car was moving. Returns true if a still average was
 * captured; p_avg[] holds the last average either way.
 */
static bool
imu_capture_pose (uint32_t pose, int32_t p_avg[])
{
    uint32_t tries = 0U;
    bool     b_still = false;

    while ((false == b_still) && (tries < IMU_POSE_TRIES))
    {
        tm_printf((UB*)"\n[IMU] >>> pose %u/%u  %s  (%u s)\n",
                  (unsigned)(pose + 1U), (unsigned)IMU_POSES,
                  g_pose_prompt[pose], (unsigned)(IMU_POSE_MOVE_MS / 1000));
        tk_dly_tsk(IMU_POSE_MOVE_MS);
        tm_printf((UB*)"[IMU] hands off, measuring in %u s\n",
                  (unsigned)(IMU_POSE_HANDS_OFF_MS / 1000));
        tk_dly_tsk(IMU_POSE_HANDS_OFF_MS);

        b_still = imu_measure_still(p_avg);
        tries++;

        if (false == b_still)
        {
            tm_printf((UB*)"[IMU] car moved or read failed, again\n");
        }
    }

    return (b_still);
}

/*
 * Average IMU_POSE_SAMPLES readings. Still = every read ok, every axis
 * spread within IMU_STILL_SPREAD_MG and |a| within 1 g +-10 %.
 */
static bool
imu_measure_still (int32_t p_avg[])
{
    imu_sample_t sample = {0};
    int32_t      sum[IMU_AXES] = {0, 0, 0};
    int32_t      min[IMU_AXES] = {INT16_MAX, INT16_MAX, INT16_MAX};
    int32_t      max[IMU_AXES] = {INT16_MIN, INT16_MIN, INT16_MIN};
    int32_t      now[IMU_AXES] = {0, 0, 0};
    int32_t      spread = 0;
    int32_t      good = 0;
    int32_t      mag2;
    uint32_t     idx;
    uint32_t     axis;

    for (idx = 0U; idx < (uint32_t)IMU_POSE_SAMPLES; idx++)
    {
        if (I2C1_OK == imu_read(&sample))
        {
            now[0] = sample.acc_x_mg;
            now[1] = sample.acc_y_mg;
            now[2] = sample.acc_z_mg;
            good++;

            for (axis = 0U; axis < IMU_AXES; axis++)
            {
                sum[axis] += now[axis];
                min[axis] = (now[axis] < min[axis]) ? now[axis] : min[axis];
                max[axis] = (now[axis] > max[axis]) ? now[axis] : max[axis];
            }
        }

        tk_dly_tsk(IMU_POSE_MS);
    }

    for (axis = 0U; axis < IMU_AXES; axis++)
    {
        p_avg[axis] = (good > 0) ? (sum[axis] / good) : 0;

        if ((good > 0) && ((max[axis] - min[axis]) > spread))
        {
            spread = max[axis] - min[axis];
        }
    }

    mag2 = (p_avg[0] * p_avg[0]) + (p_avg[1] * p_avg[1])
           + (p_avg[2] * p_avg[2]);
    tm_printf((UB*)"[IMU] avg mg x=%d y=%d z=%d  |a|=%u  spread %d mg"
              "  (%d/%d reads ok)\n", (int)p_avg[0], (int)p_avg[1],
              (int)p_avg[2], (unsigned)imu_isqrt((uint64_t)mag2),
              (int)spread, (int)good, IMU_POSE_SAMPLES);

    return ((IMU_POSE_SAMPLES == good)
            && (spread <= IMU_STILL_SPREAD_MG)
            && (mag2 >= (IMU_STILL_MIN_MG * IMU_STILL_MIN_MG))
            && (mag2 <= (IMU_STILL_MAX_MG * IMU_STILL_MAX_MG)));
}

/*
 * Index of the axis with the largest |value|. *p_b_clear: that value is
 * at least min_mg and IMU_DOMINANT_RATIO times the next largest, so the
 * axis is really aligned with the car and not a diagonal.
 */
static uint32_t
imu_main_axis (int32_t const p_vec[], int32_t min_mg, bool *p_b_clear)
{
    uint32_t best = 0U;
    uint32_t axis;
    int32_t  best_abs = 0;
    int32_t  next_abs = 0;
    int32_t  mag;

    for (axis = 0U; axis < IMU_AXES; axis++)
    {
        mag = imu_abs(p_vec[axis]);

        if (mag > best_abs)
        {
            next_abs = best_abs;
            best_abs = mag;
            best = axis;
        }
        else if (mag > next_abs)
        {
            next_abs = mag;
        }
        else
        {
            // smaller than both so far: nothing to keep
        }
    }

    *p_b_clear = (best_abs >= min_mg)
                 && (best_abs >= (IMU_DOMINANT_RATIO * next_abs));

    return (best);
}

/*
 * The LSM303D axes form a right-handed frame, so forward x left must
 * equal up. If it does not, a pose was done wrong (or the module is
 * rotated on the car). e_i x e_j = +e_k when (i, j, k) is cyclic.
 */
static bool
imu_right_handed (uint32_t fwd, bool b_fwd_pos, uint32_t left,
                  bool b_left_pos, uint32_t up, bool b_up_pos)
{
    bool     b_ok = false;
    bool     b_cyclic;
    bool     b_cross_pos;
    uint32_t third;

    if (fwd != left)
    {
        third = (IMU_AXES - fwd) - left;    // 0 + 1 + 2 = 3
        b_cyclic = (1U == (((left + IMU_AXES) - fwd) % IMU_AXES));
        b_cross_pos = ((b_fwd_pos == b_left_pos) == b_cyclic);
        b_ok = (third == up) && (b_cross_pos == b_up_pos);
    }

    return (b_ok);
}

/*
 * Print the left axis that makes forward x left = up. If the LEFT UP
 * pose gave the opposite sign, the right side was lifted instead.
 */
static void
imu_print_expected_left (uint32_t fwd, bool b_fwd_pos, uint32_t up,
                         bool b_up_pos)
{
    uint32_t axis;
    uint32_t sign;

    for (axis = 0U; axis < IMU_AXES; axis++)
    {
        for (sign = 0U; sign < 2U; sign++)
        {
            if (imu_right_handed(fwd, b_fwd_pos, axis, (1U == sign),
                                 up, b_up_pos))
            {
                tm_printf((UB*)"[IMU] with this forward and up, left must"
                          " be %s: lift the car's own LEFT wheels (as seen"
                          " from the driver's seat) and repeat\n",
                          g_axis_name[sign][axis]);
            }
        }
    }
}

static void
imu_print_ddeg (char const *p_label, uint32_t ddeg)
{
    tm_printf((UB*)"[IMU] %s %u.%u deg\n", p_label,
              (unsigned)(ddeg / IMU_DDEG_PER_DEG),
              (unsigned)(ddeg % IMU_DDEG_PER_DEG));
}

static int32_t
imu_abs (int32_t value)
{
    return ((value < 0) ? -value : value);
}

/*
 * Angle between two gravity vectors, 0.1 deg units, from
 * atan2(|ref x now|, ref . now). Exact integer inputs, so small angles
 * keep their resolution (an acos of the dot product would not).
 */
static uint32_t
imu_tilt_ddeg (int32_t const p_ref[], int32_t const p_now[])
{
    int64_t  ref_x = p_ref[0];
    int64_t  ref_y = p_ref[1];
    int64_t  ref_z = p_ref[2];
    int64_t  now_x = p_now[0];
    int64_t  now_y = p_now[1];
    int64_t  now_z = p_now[2];
    int64_t  cross_x = (ref_y * now_z) - (ref_z * now_y);
    int64_t  cross_y = (ref_z * now_x) - (ref_x * now_z);
    int64_t  cross_z = (ref_x * now_y) - (ref_y * now_x);
    int64_t  dot = (ref_x * now_x) + (ref_y * now_y) + (ref_z * now_z);
    uint64_t cross2 = (uint64_t)((cross_x * cross_x) + (cross_y * cross_y)
                                 + (cross_z * cross_z));
    uint64_t opposite = imu_isqrt(cross2);
    uint32_t angle;

    if (dot >= 0)
    {
        angle = imu_angle_ddeg(opposite, (uint64_t)dot);
    }
    else
    {
        angle = IMU_DDEG_180 - imu_angle_ddeg(opposite, (uint64_t)(-dot));
    }

    return (angle);
}

// atan(opposite / adjacent) for 0..90 deg, 0.1 deg units
static uint32_t
imu_angle_ddeg (uint64_t opposite, uint64_t adjacent)
{
    uint32_t angle = 0U;

    if (0U == adjacent)
    {
        angle = (0U == opposite) ? 0U : IMU_DDEG_90;
    }
    else if (opposite <= adjacent)
    {
        angle = imu_atan_ddeg((uint32_t)((opposite * IMU_TAN_SCALE)
                                         / adjacent));
    }
    else
    {
        angle = IMU_DDEG_90 - imu_atan_ddeg((uint32_t)((adjacent
                                            * IMU_TAN_SCALE) / opposite));
    }

    return (angle);
}

// tan x 10000 (0..10000) -> 0..450 (0.1 deg), linear between table steps
static uint32_t
imu_atan_ddeg (uint32_t tan_x10000)
{
    uint32_t deg = 0U;
    uint32_t span;

    while ((deg < (IMU_TAN_STEPS - 1U))
           && (tan_x10000 >= g_tan_x10000[deg + 1U]))
    {
        deg++;
    }

    span = (uint32_t)g_tan_x10000[deg + 1U] - g_tan_x10000[deg];

    return ((deg * IMU_DDEG_PER_DEG)
            + ((((tan_x10000 - g_tan_x10000[deg]) * IMU_DDEG_PER_DEG)
                + (span / 2U)) / span));
}

// Integer square root (bit by bit)
static uint32_t
imu_isqrt (uint64_t value)
{
    uint64_t rem = value;
    uint64_t root = 0U;
    uint64_t bit = (uint64_t)1U << 62;

    while (bit > rem)
    {
        bit >>= 2;
    }

    while (0U != bit)
    {
        if (rem >= (root + bit))
        {
            rem -= root + bit;
            root = (root >> 1) + bit;
        }
        else
        {
            root >>= 1;
        }

        bit >>= 2;
    }

    return ((uint32_t)root);
}

static bool
imu_mag_axis_ok (int16_t value)
{
    return ((IMU_MAG_SAT_POS != value) && (IMU_MAG_SAT_NEG != value));
}

/*
 * Probe every address with a 1-byte read and list who answers.
 * Returns true if the LSM303D answered.
 */
static bool
imu_scan_bus (void)
{
    uint8_t  addr;
    uint8_t  dummy = 0U;
    uint32_t count = 0U;
    bool     b_imu = false;

    tm_printf((UB*)"[IMU] I2C1 scan:");

    for (addr = IMU_SCAN_FIRST_ADDR; addr <= IMU_SCAN_LAST_ADDR; addr++)
    {
        if (I2C1_OK == i2c1_read(addr, &dummy, 1U))
        {
            tm_printf((UB*)" 0x%02x", addr);
            count++;
            b_imu = (b_imu || (IMU_ADDR == addr));
        }
    }

    tm_printf((UB*)"  (%u found)\n", (unsigned)count);

    return (b_imu);
}

/*
 * USB output printed before a terminal opens the port is lost, so wait
 * until one is connected (PuTTY / VS Code Serial Monitor set DTR).
 */
static void
imu_wait_for_terminal (void)
{
#if TM_CONSOLE_USB_CDC
    int32_t waited_ms = 0;

    while ((0 == tm_tusb_connected()) && (waited_ms < IMU_WAIT_TERM_MS))
    {
        tk_dly_tsk(IMU_POLL_MS);
        waited_ms += IMU_POLL_MS;
    }

    tk_dly_tsk(IMU_SETTLE_MS);      // let the terminal settle
#endif
}

#endif // APP_TEST
