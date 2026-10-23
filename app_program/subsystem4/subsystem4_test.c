/*
 * subsystem4_test.c
 * Subsystem 4 (Buddy 4): on-target tests. Built only with: make TEST=1
 * Record unit results in testing/subsystem4/TEST_LOG.md and the
 * INT- result in testing/integration/TEST_LOG.md.
 *
 * IMU tests (GY-511 on I2C1, SDA GP2 / SCL GP3) print [IMU] lines; keep
 * the board flat and still until the S4-U08 prompt.
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

#if TM_CONSOLE_USB_CDC
// libtm USB glue: true once a terminal has opened the port (DTR)
extern int tm_tusb_connected(void);
#endif

// Diagnostic: chips that answer at 0x1D instead of the LSM303DLHC.
// ID values from each chip's datasheet; confirm against the board.
#define IMU_ALT_ADDR            (0x1DU)
#define IMU_ID_REG_ST           (0x0FU)     // ST WHO_AM_I
#define IMU_ID_REG_ADXL         (0x00U)     // ADXL345 DEVID
#define IMU_ID_REG_MMA          (0x0DU)     // NXP MMA845x WHO_AM_I
#define IMU_ID_LSM303D          (0x49U)     // also LSM9DS0 accel/mag
#define IMU_ID_LSM303C_A        (0x41U)
#define IMU_ID_ADXL345          (0xE5U)
#define IMU_ID_MMA8451          (0x1AU)
#define IMU_ID_MMA8452          (0x2AU)
#define IMU_ID_MMA8453          (0x3AU)

static void imu_wait_for_terminal(void);
static bool imu_scan_bus(void);
static void imu_identify_0x1d(void);
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
 * S4-U04..U11: GY-511 IMU. Uses its own TEST_INIT/TEST_SUMMARY so the
 * IMU result line is easy to find in the log.
 */
static void
imu_tests (void)
{
    imu_sample_t  sample = {0};
    i2c1_result_t result;
    uint8_t       id[IMU_MAG_ID_LEN] = {0U};
    uint8_t       ctrl1 = 0U;
    int32_t       mag2;
    uint32_t      idx;
    bool          b_idle;
    bool          b_found;

    TEST_INIT();

    imu_wait_for_terminal();
    tm_printf((UB*)"\n[IMU] ===== subsystem 4 IMU test (GY-511) =====\n"
              "[IMU] keep the board flat and still\n");

    // S4-U09: both lines idle high before we start (pull-ups present)
    b_idle = i2c1_bus_is_idle();
    TEST_CHECK("S4-U09", b_idle, "I2C1 SDA/SCL idle high (pull-ups ok)");

    if (false == b_idle)
    {
        tm_printf((UB*)"[IMU] a line is low: check SDA->GP2, SCL->GP3,"
                  " VCC->3V3, GND\n");
    }

    // S4-U11: I2C1 starts and both sensors ACK their configuration
    result = imu_init();
    TEST_CHECK("S4-U11", I2C1_OK == result,
               "I2C1 init + sensor configuration writes ACKed");
    tm_printf((UB*)"[IMU] init: %s (abort src 0x%x)\n",
              i2c1_result_name(result), (unsigned)i2c1_last_abort());

    // S4-U10: bus scan finds the accelerometer and the magnetometer
    b_found = imu_scan_bus();
    TEST_CHECK("S4-U10", b_found, "bus scan finds 0x19 and 0x1E");
    imu_identify_0x1d();

    // S4-U04: magnetometer identification
    result = imu_read_mag_id(id);
    tm_printf((UB*)"[IMU] magnetometer ID: %s, 0x%02x 0x%02x 0x%02x"
              " (expect 0x48 0x34 0x33)\n",
              i2c1_result_name(result), id[0], id[1], id[2]);
    TEST_CHECK("S4-U04", (I2C1_OK == result) && ('H' == id[0])
               && ('4' == id[1]) && ('3' == id[2]),
               "magnetometer at 0x1E answers \"H43\"");

    // S4-U05: accelerometer kept its configuration
    result = imu_read_reg(IMU_ACC_ADDR, IMU_CTRL_REG1_A, &ctrl1);
    tm_printf((UB*)"[IMU] CTRL_REG1_A: %s, 0x%02x (expect 0x%02x)\n",
              i2c1_result_name(result), ctrl1, IMU_CTRL_REG1_A_VAL);
    TEST_CHECK("S4-U05",
               (I2C1_OK == result) && (IMU_CTRL_REG1_A_VAL == ctrl1),
               "accelerometer at 0x19 configured (CTRL_REG1_A read back)");

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
               && (IMU_MAG_OVERFLOW != sample.mag_x)
               && (IMU_MAG_OVERFLOW != sample.mag_y)
               && (IMU_MAG_OVERFLOW != sample.mag_z)
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
                      "  mag %5d %5d %5d\n", (unsigned)idx,
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

/*
 * Probe every address with a 1-byte read and list who answers.
 * Returns true if both GY-511 addresses answered.
 */
static bool
imu_scan_bus (void)
{
    uint8_t  addr;
    uint8_t  dummy = 0U;
    uint32_t count = 0U;
    bool     b_acc = false;
    bool     b_mag = false;

    tm_printf((UB*)"[IMU] I2C1 scan:");

    for (addr = IMU_SCAN_FIRST_ADDR; addr <= IMU_SCAN_LAST_ADDR; addr++)
    {
        if (I2C1_OK == i2c1_read(addr, &dummy, 1U))
        {
            tm_printf((UB*)" 0x%02x", addr);
            count++;
            b_acc = (b_acc || (IMU_ACC_ADDR == addr));
            b_mag = (b_mag || (IMU_MAG_ADDR == addr));
        }
    }

    tm_printf((UB*)"  (%u found)\n", (unsigned)count);

    return (b_acc && b_mag);
}

/*
 * Read the usual ID registers of a device at 0x1D and name the chip.
 */
static void
imu_identify_0x1d (void)
{
    uint8_t     id_st = 0U;
    uint8_t     id_adxl = 0U;
    uint8_t     id_mma = 0U;
    char const *p_guess = "unknown: read the chip marking";

    if (I2C1_OK == i2c1_read(IMU_ALT_ADDR, &id_st, 1U))
    {
        (void)imu_read_reg(IMU_ALT_ADDR, IMU_ID_REG_ST, &id_st);
        (void)imu_read_reg(IMU_ALT_ADDR, IMU_ID_REG_ADXL, &id_adxl);
        (void)imu_read_reg(IMU_ALT_ADDR, IMU_ID_REG_MMA, &id_mma);

        if (IMU_ID_LSM303D == id_st)
        {
            p_guess = "LSM303D or LSM9DS0 (accel+mag at 0x1D)";
        }
        else if (IMU_ID_LSM303C_A == id_st)
        {
            p_guess = "LSM303C accelerometer";
        }
        else if (IMU_ID_ADXL345 == id_adxl)
        {
            p_guess = "ADXL345 accelerometer";
        }
        else if ((IMU_ID_MMA8451 == id_mma) || (IMU_ID_MMA8452 == id_mma)
                 || (IMU_ID_MMA8453 == id_mma))
        {
            p_guess = "NXP MMA845x accelerometer";
        }
        else
        {
            // keep the "unknown" text
        }

        tm_printf((UB*)"[IMU] device at 0x1D: reg0x0F=0x%02x reg0x00=0x%02x"
                  " reg0x0D=0x%02x -> %s\n", id_st, id_adxl, id_mma,
                  p_guess);
    }
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
