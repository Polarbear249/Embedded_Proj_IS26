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

#if TM_CONSOLE_USB_CDC
// libtm USB glue: true once a terminal has opened the port (DTR)
extern int tm_tusb_connected(void);
#endif

static void imu_wait_for_terminal(void);
static bool imu_scan_bus(void);
static bool imu_mag_axis_ok(int16_t value);
static void imu_tests(void);

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

    TEST_SUMMARY("subsystem4 IMU");
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
