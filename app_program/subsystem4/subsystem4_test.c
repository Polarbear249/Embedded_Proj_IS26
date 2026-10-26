/*
 * subsystem4_test.c
 * Subsystem 4 (Buddy 4): on-target tests. Built only with: make TEST=1
 * Record unit results in testing/subsystem4/TEST_LOG.md and the
 * INT- result in testing/integration/TEST_LOG.md.
 *
 * IMU tests (GY-511 = LSM303D at 0x1D on I2C1, SDA GP2 / SCL GP3) print
 * [IMU] lines; keep the board flat and still until the S4-U08 prompt.
 */

#include "subsystem4.h"
#include "drv_imu.h"
#include "../test_report.h"

#if APP_TEST

#define IMU_STREAM_SAMPLES      (25U)       // S4-U08: 25 x 200 ms = 5 s
#define IMU_STREAM_MS           (200)
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

// S4-U15 / S4-U16 ramp test
#define IMU_RAMP_PREP_MS        (5000)      // time to put the car down
#define IMU_RAMP_CAL_SAMPLES    (50)        // 50 x 20 ms = 1 s at rest
#define IMU_RAMP_CAL_MS         (20)        //   (one 50 Hz sample each)
#define IMU_RAMP_SAMPLES        (300U)      // 300 x 100 ms = 30 s
#define IMU_RAMP_MS             (100)
#define IMU_AXES                (3U)

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

#if TM_CONSOLE_USB_CDC
// libtm USB glue: true once a terminal has opened the port (DTR)
extern int tm_tusb_connected(void);
#endif

static void     imu_wait_for_terminal(void);
static bool     imu_scan_bus(void);
static bool     imu_mag_axis_ok(int16_t value);
static void     imu_tests(void);
static void     imu_ramp_test(bool *p_b_rest_ok, bool *p_b_stream_ok);
static uint32_t imu_tilt_ddeg(int32_t const p_ref[], imu_sample_t const *p_now);
static uint32_t imu_angle_ddeg(uint64_t opposite, uint64_t adjacent);
static uint32_t imu_atan_ddeg(uint32_t tan_x10000);
static uint32_t imu_isqrt(uint64_t value);

void
subsystem4_test (void)
{
    T_RMBF   rmbf;
    uint32_t before;

    TEST_INIT();

    // S4-U01: the message buffer accepts the documented line length
    TEST_CHECK("S4-U01",
               (E_OK == tk_ref_mbf(mbf_summary, &rmbf))
               && (S4_SUMMARY_MAX == rmbf.maxmsz),
               "message buffer max line = S4_SUMMARY_MAX");

    imu_tests();

    // INT-02 (integration, IF-03): summary lines from subsystem 3 reach
    // the monitor. One line per 5 samples = one per 1.5 s.
    before = s4_lines_received;
    tk_dly_tsk(3500);
    TEST_CHECK("INT-02", s4_lines_received >= (before + 2U),
               "summary lines flow from subsystem 3 to subsystem 4");

    TEST_SUMMARY("subsystem4 (S4-U01, INT-02)");
}

/*
 * IMU tests. Uses its own TEST_INIT/TEST_SUMMARY so the IMU result line
 * is easy to find in the log.
 */
static void
imu_tests (void)
{
    imu_sample_t  sample = {0};
    i2c1_result_t result;
    uint8_t       value = 0U;
    int32_t       mag2;
    uint32_t      idx;
    bool          b_idle;
    bool          b_found;
    bool          b_rest_ok = false;
    bool          b_stream_ok = false;

    TEST_INIT();

    imu_wait_for_terminal();
    tm_printf((UB*)"\n[IMU] ===== subsystem 4 IMU test (GY-511 / LSM303D)"
              " =====\n[IMU] keep the board flat and still\n");

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

    // S4-U08 (manual): tilt and turn the board while this prints
    tm_printf((UB*)"[IMU] S4-U08 manual: tilt/rotate the board for 5 s\n");

    for (idx = 0U; idx < IMU_STREAM_SAMPLES; idx++)
    {
        result = imu_read(&sample);

        if (I2C1_OK == result)
        {
            tm_printf((UB*)"[IMU] %2u acc mg %5d %5d %5d"
                      "  mag %6d %6d %6d\n", (unsigned)idx,
                      sample.acc_x_mg, sample.acc_y_mg, sample.acc_z_mg,
                      sample.mag_x, sample.mag_y, sample.mag_z);
        }
        else
        {
            tm_printf((UB*)"[IMU] %2u read error: %s\n", (unsigned)idx,
                      i2c1_result_name(result));
        }

        tk_dly_tsk(IMU_STREAM_MS);
    }

    // S4-U15 / S4-U16 (manual): car on the flat floor, then over a ramp
    imu_ramp_test(&b_rest_ok, &b_stream_ok);
    TEST_CHECK("S4-U15", b_rest_ok,
               "rest vector on flat floor is 1 g +-20 %");
    TEST_CHECK("S4-U16", b_stream_ok,
               "30 s ramp stream with no I2C1 read errors");

    TEST_SUMMARY("subsystem4 IMU");
}

/*
 * S4-U15 / S4-U16: ramp. Average gravity on the flat floor, then print
 * the tilt away from that vector while the car is pushed over a ramp.
 * Tilt from the rest vector does not depend on how the module is
 * mounted, but it has no sign (up and down slopes both read positive).
 */
static void
imu_ramp_test (bool *p_b_rest_ok, bool *p_b_stream_ok)
{
    imu_sample_t  sample = {0};
    i2c1_result_t result;
    int32_t       sum[IMU_AXES] = {0, 0, 0};
    int32_t       ref[IMU_AXES] = {0, 0, 0};
    int32_t       ref2;
    int32_t       good = 0;
    uint32_t      errors = 0U;
    uint32_t      tilt;
    uint32_t      max_tilt = 0U;
    uint32_t      idx;
    uint32_t      axis;
    SYSTIM        start;
    SYSTIM        now;

    tm_printf((UB*)"[IMU] S4-U15 ramp: put the car on the flat floor"
              " before the ramp, hands off, %d s\n", IMU_RAMP_PREP_MS / 1000);
    tk_dly_tsk(IMU_RAMP_PREP_MS);

    for (idx = 0U; idx < (uint32_t)IMU_RAMP_CAL_SAMPLES; idx++)
    {
        if (I2C1_OK == imu_read(&sample))
        {
            sum[0] += sample.acc_x_mg;
            sum[1] += sample.acc_y_mg;
            sum[2] += sample.acc_z_mg;
            good++;
        }

        tk_dly_tsk(IMU_RAMP_CAL_MS);
    }

    for (axis = 0U; (axis < IMU_AXES) && (good > 0); axis++)
    {
        ref[axis] = sum[axis] / good;
    }

    ref2 = (ref[0] * ref[0]) + (ref[1] * ref[1]) + (ref[2] * ref[2]);
    tm_printf((UB*)"[IMU] rest vector mg x=%d y=%d z=%d  |a|=%u"
              "  (%d of %d reads ok)\n", (int)ref[0], (int)ref[1],
              (int)ref[2], (unsigned)imu_isqrt((uint64_t)ref2), (int)good,
              IMU_RAMP_CAL_SAMPLES);
    *p_b_rest_ok = (IMU_RAMP_CAL_SAMPLES == good)
                   && (ref2 >= (IMU_1G_MIN_MG * IMU_1G_MIN_MG))
                   && (ref2 <= (IMU_1G_MAX_MG * IMU_1G_MAX_MG));

    tm_printf((UB*)"[IMU] S4-U16 now push the car slowly up and over the"
              " ramp (%u s)\n",
              (unsigned)((IMU_RAMP_SAMPLES * IMU_RAMP_MS) / 1000U));
    tk_get_tim(&start);

    for (idx = 0U; idx < IMU_RAMP_SAMPLES; idx++)
    {
        result = imu_read(&sample);
        tk_get_tim(&now);

        if (I2C1_OK == result)
        {
            tilt = imu_tilt_ddeg(ref, &sample);

            if (tilt > max_tilt)
            {
                max_tilt = tilt;
            }

            tm_printf((UB*)"[IMU] ramp t=%5u ms  acc mg %5d %5d %5d"
                      "  tilt %3u.%u deg\n", (unsigned)(now.lo - start.lo),
                      sample.acc_x_mg, sample.acc_y_mg, sample.acc_z_mg,
                      (unsigned)(tilt / IMU_DDEG_PER_DEG),
                      (unsigned)(tilt % IMU_DDEG_PER_DEG));
        }
        else
        {
            errors++;
            tm_printf((UB*)"[IMU] ramp read error: %s\n",
                      i2c1_result_name(result));
        }

        tk_dly_tsk(IMU_RAMP_MS);
    }

    tm_printf((UB*)"[IMU] ramp max tilt %u.%u deg, %u read errors\n",
              (unsigned)(max_tilt / IMU_DDEG_PER_DEG),
              (unsigned)(max_tilt % IMU_DDEG_PER_DEG), (unsigned)errors);
    *p_b_stream_ok = (0U == errors);
}

/*
 * Angle between the rest vector and the current reading, 0.1 deg units,
 * from atan2(|ref x now|, ref . now). Exact integer inputs, so small
 * angles keep their resolution (an acos of the dot product would not).
 */
static uint32_t
imu_tilt_ddeg (int32_t const p_ref[], imu_sample_t const *p_now)
{
    int64_t  now_x = p_now->acc_x_mg;
    int64_t  now_y = p_now->acc_y_mg;
    int64_t  now_z = p_now->acc_z_mg;
    int64_t  cross_x = (p_ref[1] * now_z) - (p_ref[2] * now_y);
    int64_t  cross_y = (p_ref[2] * now_x) - (p_ref[0] * now_z);
    int64_t  cross_z = (p_ref[0] * now_y) - (p_ref[1] * now_x);
    int64_t  dot = (p_ref[0] * now_x) + (p_ref[1] * now_y)
                   + (p_ref[2] * now_z);
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
