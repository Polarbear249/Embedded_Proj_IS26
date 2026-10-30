/*
 * imu_terrain.c
 * Subsystem 4 (Buddy 4): imu_terrain task, see imu_terrain.h.
 *
 * Kernel objects (both created in imu_terrain_init, owner Buddy 4):
 *  - g_flg_tick: event flag, TA_WSGL. Writer: the cyclic handler (sets
 *    IMU_TICK_BIT every 10 ms). Reader: imu_terrain_task only, waits
 *    TWF_ORW | TWF_BITCLR. A flag (not a semaphore) on purpose: if the
 *    task overruns, ticks merge instead of queueing up, and the overrun
 *    shows as a long period in the timing line.
 *  - g_cyc_tick: cyclic handler, 10 ms, never blocks.
 * Worst-case blocking in the task: the I2C1 driver busy-waits up to
 * I2C1_TIMEOUT_MS (10 ms, plus one tick) per byte when the bus is
 * stuck, so a failed cycle can take about 20 ms (derived, not
 * measured). Fault recovery (i2c1_init bus recovery) sleeps a few ms.
 */

#include "imu_terrain.h"
#include "drv_imu.h"
#include "imu_calib.h"
#include "../subsystem2/motion_control.h"     // IF-10 distance (reader)

#define IMU_TICK_BIT            (0x1U)
#define IMU_WAIT_TMO_MS         (100)   // 10 missed ticks: cyclic dead
#define IMU_TERRAIN_MAG_DIV     (5U)    // mag every 5th cycle = 20 Hz
#define IMU_TERRAIN_FAULT_COUNT (5U)    // 5 bad reads in a row = 50 ms
#define IMU_TERRAIN_RETRY_CYC   (100U)  // fault: retry init every 1 s
#define IMU_TERRAIN_REPORT_CYC  (100U)  // timing line every 1 s

// Level zero: 50 samples (0.5 s), each axis may not move more than
// 50 mg peak to peak (measured noise at rest about +-10 mg, run 5).
#define IMU_LEVEL_SAMPLES       (50)
#define IMU_LEVEL_STILL_MG      (50)

// IIR low-pass y += (x - y) / 4, state in mg x 16. At 100 Hz the -3 dB
// point is about 4.6 Hz (derived). Note: CTRL1 is still 50 Hz, so every
// second sample repeats (handoff step 6).
#define IMU_IIR_Q               (16)
#define IMU_IIR_DIV             (4)

#define IMU_LATE_US             (15000U)    // period > 1.5 x 10 ms
#define IMU_LINE_LEN            (192U)

typedef struct
{
    uint32_t acc_min_us;
    uint32_t acc_max_us;
    uint32_t mag_max_us;
    uint32_t loop_min_us;
    uint32_t loop_max_us;
    uint32_t per_min_us;
    uint32_t per_max_us;
    uint32_t late;
    uint32_t errors;
} imu_timing_t;

static ID                   g_flg_tick = 0;
static ID                   g_cyc_tick = 0;

// Written only by imu_terrain_task. A single aligned 32-bit store is
// atomic on the M0+, so imu_terrain_get_state() needs no lock.
static volatile uint32_t    g_state = (uint32_t)IMU_TERRAIN_INIT;

// Task-private (only imu_terrain_task touches these)
static imu_calib_t          g_cal;
static imu_calib_mag_t      g_mag_cal;
static imu_calib_level_t    g_level = {0, 0};
static bool                 g_b_level_valid = false;
static int32_t              g_filt_q[IMU_CALIB_AXES] = {0, 0, 0};
static int32_t              g_mag[IMU_CALIB_AXES] = {0, 0, 0};
static int32_t              g_lvl_sum[IMU_CALIB_AXES] = {0, 0, 0};
static int32_t              g_lvl_min[IMU_CALIB_AXES] = {0, 0, 0};
static int32_t              g_lvl_max[IMU_CALIB_AXES] = {0, 0, 0};
static int32_t              g_lvl_count = 0;
static uint32_t             g_bad_reads = 0U;
static uint32_t             g_wcet_us = 0U;     // whole run, not reset
static imu_timing_t         g_tim;

static char const * const   g_state_name[] =
{
    "INIT", "CALIBRATING", "ACTIVE", "FAULT"
};

static void     imu_terrain_cyc_handler(void *p_exinf);
static void     imu_terrain_task(INT stacd, void *p_exinf);
static void     imu_set_state(imu_terrain_state_t state);
static void     imu_start_device(void);
static void     imu_cycle(uint32_t cycle, uint32_t *p_acc_us,
                          uint32_t *p_mag_us);
static bool     imu_sample(uint32_t cycle, int32_t p_acc[],
                           uint32_t *p_acc_us, uint32_t *p_mag_us);
static void     imu_level_add(int32_t const p_acc[]);
static void     imu_filter(int32_t const p_acc[]);
static void     imu_read_failed(i2c1_result_t result);
static void     imu_timing_reset(void);
static void     imu_report(void);
static uint32_t imu_now_us(void);

/*!
 * Create the tick flag and the 10 ms cyclic handler (not started).
 */
ER
imu_terrain_init (void)
{
    T_CFLG cflg = {0};
    T_CCYC ccyc = {0};
    ER     er = E_OK;

    cflg.flgatr = TA_TFIFO | TA_WSGL;
    cflg.iflgptn = 0U;
    g_flg_tick = tk_cre_flg(&cflg);

    if (g_flg_tick < E_OK)
    {
        er = (ER)g_flg_tick;
    }
    else
    {
        ccyc.cycatr = TA_HLNG;
        ccyc.cychdr = (FP)imu_terrain_cyc_handler;
        ccyc.cyctim = IMU_TERRAIN_PERIOD_MS;
        ccyc.cycphs = IMU_TERRAIN_PERIOD_MS;
        g_cyc_tick = tk_cre_cyc(&ccyc);
        er = (g_cyc_tick < E_OK) ? (ER)g_cyc_tick : E_OK;
    }

    return (er);
}

/*!
 * Start the task, then the cyclic handler that paces it.
 */
ER
imu_terrain_start (void)
{
    ID tid = app_start_task((FP)imu_terrain_task, IMU_TERRAIN_PRI,
                            IMU_TERRAIN_PRC, "imu_terrain");
    ER er = E_OK;

    if (tid < E_OK)
    {
        er = (ER)tid;
    }
    else
    {
        er = tk_sta_cyc(g_cyc_tick);
    }

    return (er);
}

imu_terrain_state_t
imu_terrain_get_state (void)
{
    return ((imu_terrain_state_t)g_state);
}

// Cyclic handler: never blocks, only wakes the task.
static void
imu_terrain_cyc_handler (void *p_exinf)
{
    (void)p_exinf;
    (void)tk_set_flg(g_flg_tick, IMU_TICK_BIT);
}

static void
imu_terrain_task (INT stacd, void *p_exinf)
{
    UINT     flg_ptn = 0U;
    ER       er = E_OK;
    uint32_t cycle = 0U;
    uint32_t wake_us = 0U;
    uint32_t last_wake_us = 0U;
    uint32_t period_us = 0U;
    uint32_t loop_us = 0U;
    uint32_t acc_us = 0U;
    uint32_t mag_us = 0U;

    (void)stacd;
    (void)p_exinf;

    imu_calib_default(&g_cal);
    imu_calib_mag_default(&g_mag_cal);
    imu_timing_reset();
    imu_start_device();
    last_wake_us = imu_now_us();

    for (;;)
    {
        er = tk_wai_flg(g_flg_tick, IMU_TICK_BIT, TWF_ORW | TWF_BITCLR,
                        &flg_ptn, IMU_WAIT_TMO_MS);
        wake_us = imu_now_us();
        CHECK(er, "imu_terrain tk_wai_flg");

        period_us = wake_us - last_wake_us;    // unsigned: wraps safely
        last_wake_us = wake_us;

        acc_us = 0U;
        mag_us = 0U;
        imu_cycle(cycle, &acc_us, &mag_us);
        loop_us = imu_now_us() - wake_us;

        // Timing of this cycle (the report print is not included)
        if (0U != cycle)
        {
            g_tim.per_min_us = (period_us < g_tim.per_min_us)
                               ? period_us : g_tim.per_min_us;
            g_tim.per_max_us = (period_us > g_tim.per_max_us)
                               ? period_us : g_tim.per_max_us;
            g_tim.late += (period_us > IMU_LATE_US) ? 1U : 0U;
        }

        g_tim.loop_min_us = (loop_us < g_tim.loop_min_us)
                            ? loop_us : g_tim.loop_min_us;
        g_tim.loop_max_us = (loop_us > g_tim.loop_max_us)
                            ? loop_us : g_tim.loop_max_us;
        g_wcet_us = (loop_us > g_wcet_us) ? loop_us : g_wcet_us;

        if (0U != acc_us)
        {
            g_tim.acc_min_us = (acc_us < g_tim.acc_min_us)
                               ? acc_us : g_tim.acc_min_us;
            g_tim.acc_max_us = (acc_us > g_tim.acc_max_us)
                               ? acc_us : g_tim.acc_max_us;
        }

        g_tim.mag_max_us = (mag_us > g_tim.mag_max_us)
                           ? mag_us : g_tim.mag_max_us;

        cycle++;

        if (0U == (cycle % IMU_TERRAIN_REPORT_CYC))
        {
            imu_report();
            imu_timing_reset();
        }
    }
}

static void
imu_set_state (imu_terrain_state_t state)
{
    if ((uint32_t)state != g_state)
    {
        g_state = (uint32_t)state;
        tm_printf((UB*)"[IMU] state %s\n", g_state_name[state]);
    }
}

/*
 * Configure the LSM303D and check WHO_AM_I. Keeps the level reference
 * if there is one (recovery after a fault), else starts level zero.
 */
static void
imu_start_device (void)
{
    i2c1_result_t result = imu_init();
    uint8_t       who = 0U;

    if (I2C1_OK == result)
    {
        result = imu_read_reg(IMU_WHO_AM_I, &who);
    }

    if ((I2C1_OK == result) && (IMU_WHO_AM_I_VAL == who))
    {
        g_bad_reads = 0U;
        g_lvl_count = 0;
        imu_set_state(g_b_level_valid ? IMU_TERRAIN_ACTIVE
                                      : IMU_TERRAIN_CALIBRATING);
    }
    else
    {
        tm_printf((UB*)"[IMU] start failed: %s, WHO_AM_I 0x%x\n",
                  i2c1_result_name(result), (UINT)who);
        imu_set_state(IMU_TERRAIN_FAULT);
    }
}

/*
 * One 10 ms cycle for the current state.
 */
static void
imu_cycle (uint32_t cycle, uint32_t *p_acc_us, uint32_t *p_mag_us)
{
    int32_t acc[IMU_CALIB_AXES] = {0, 0, 0};

    switch ((imu_terrain_state_t)g_state)
    {
        case IMU_TERRAIN_CALIBRATING:
            if (imu_sample(cycle, acc, p_acc_us, p_mag_us))
            {
                imu_level_add(acc);
            }
            break;

        case IMU_TERRAIN_ACTIVE:
            if (imu_sample(cycle, acc, p_acc_us, p_mag_us))
            {
                imu_filter(acc);
            }
            break;

        case IMU_TERRAIN_FAULT:
            if (0U == (cycle % IMU_TERRAIN_RETRY_CYC))
            {
                imu_start_device();
            }
            break;

        case IMU_TERRAIN_INIT:
        default:
            imu_start_device();
            break;
    }
}

/*
 * Read and correct one sample. Returns true when p_acc is valid.
 */
static bool
imu_sample (uint32_t cycle, int32_t p_acc[], uint32_t *p_acc_us,
            uint32_t *p_mag_us)
{
    imu_sample_t  raw = {0, 0, 0, 0, 0, 0};
    int32_t       acc_raw[IMU_CALIB_AXES] = {0, 0, 0};
    int32_t       mag_raw[IMU_CALIB_AXES] = {0, 0, 0};
    uint32_t      t_start = imu_now_us();
    i2c1_result_t result = imu_read_acc(&raw);

    *p_acc_us = imu_now_us() - t_start;

    if ((I2C1_OK == result) && (0U == (cycle % IMU_TERRAIN_MAG_DIV)))
    {
        t_start = imu_now_us();
        result = imu_read_mag(&raw);
        *p_mag_us = imu_now_us() - t_start;

        mag_raw[0] = raw.mag_x;
        mag_raw[1] = raw.mag_y;
        mag_raw[2] = raw.mag_z;
        imu_calib_mag_apply(&g_mag_cal, mag_raw, g_mag);
    }

    if (I2C1_OK == result)
    {
        g_bad_reads = 0U;
        acc_raw[0] = raw.acc_x_mg;
        acc_raw[1] = raw.acc_y_mg;
        acc_raw[2] = raw.acc_z_mg;
        imu_calib_apply(&g_cal, acc_raw, p_acc);
    }
    else
    {
        imu_read_failed(result);
    }

    return (I2C1_OK == result);
}

/*
 * Level zero: average IMU_LEVEL_SAMPLES samples. If any axis moved more
 * than IMU_LEVEL_STILL_MG the car was not still: start again.
 */
static void
imu_level_add (int32_t const p_acc[])
{
    int32_t avg[IMU_CALIB_AXES] = {0, 0, 0};
    bool    b_still = true;
    uint32_t axis;

    for (axis = 0U; axis < IMU_CALIB_AXES; axis++)
    {
        if (0 == g_lvl_count)
        {
            g_lvl_sum[axis] = 0;
            g_lvl_min[axis] = p_acc[axis];
            g_lvl_max[axis] = p_acc[axis];
        }

        g_lvl_sum[axis] += p_acc[axis];
        g_lvl_min[axis] = (p_acc[axis] < g_lvl_min[axis])
                          ? p_acc[axis] : g_lvl_min[axis];
        g_lvl_max[axis] = (p_acc[axis] > g_lvl_max[axis])
                          ? p_acc[axis] : g_lvl_max[axis];
        b_still = b_still && ((g_lvl_max[axis] - g_lvl_min[axis])
                              <= IMU_LEVEL_STILL_MG);
    }

    g_lvl_count++;

    if (false == b_still)
    {
        tm_printf((UB*)"[IMU] level zero: car moved, restarting\n");
        g_lvl_count = 0;
    }
    else if (IMU_LEVEL_SAMPLES <= g_lvl_count)
    {
        for (axis = 0U; axis < IMU_CALIB_AXES; axis++)
        {
            avg[axis] = g_lvl_sum[axis] / IMU_LEVEL_SAMPLES;
            g_filt_q[axis] = avg[axis] * IMU_IIR_Q;
        }

        imu_calib_level_set(&g_level, avg);
        g_b_level_valid = true;
        tm_printf((UB*)"[IMU] level zero: pitch %d roll %d (0.1 deg)\n",
                  (INT)g_level.pitch_ddeg, (INT)g_level.roll_ddeg);
        imu_set_state(IMU_TERRAIN_ACTIVE);
    }
    else
    {
        // still collecting
    }
}

static void
imu_filter (int32_t const p_acc[])
{
    uint32_t axis;

    for (axis = 0U; axis < IMU_CALIB_AXES; axis++)
    {
        g_filt_q[axis] += ((p_acc[axis] * IMU_IIR_Q) - g_filt_q[axis])
                          / IMU_IIR_DIV;
    }
}

static void
imu_read_failed (i2c1_result_t result)
{
    g_tim.errors++;
    g_bad_reads++;

    if (IMU_TERRAIN_FAULT_COUNT == g_bad_reads)
    {
        tm_printf((UB*)"[IMU] %u bad reads in a row, last %s: IMU_FAULT\n",
                  (UINT)g_bad_reads, i2c1_result_name(result));
        imu_set_state(IMU_TERRAIN_FAULT);
    }
}

static void
imu_timing_reset (void)
{
    g_tim.acc_min_us = UINT32_MAX;
    g_tim.acc_max_us = 0U;
    g_tim.mag_max_us = 0U;
    g_tim.loop_min_us = UINT32_MAX;
    g_tim.loop_max_us = 0U;
    g_tim.per_min_us = UINT32_MAX;
    g_tim.per_max_us = 0U;
    g_tim.late = 0U;
    g_tim.errors = 0U;
}

/*
 * One line per second, built first and printed once (tm_printf is
 * atomic per call). Times in us; pitch/roll in 0.1 deg against level;
 * hdg = magnetometer heading, 0.1 deg, cross-check only.
 * d = IF-10 cumulative distance from S2, mm (hump height input).
 */
static void
imu_report (void)
{
    UB      line[IMU_LINE_LEN];
    int32_t filt_mg[IMU_CALIB_AXES] = {0, 0, 0};
    int32_t pitch = 0;
    int32_t roll = 0;
    int32_t heading = imu_calib_atan2_ddeg(g_mag[1], g_mag[0]);
    uint32_t axis;

    for (axis = 0U; axis < IMU_CALIB_AXES; axis++)
    {
        filt_mg[axis] = g_filt_q[axis] / IMU_IIR_Q;
    }

    imu_calib_level_tilt(&g_level, filt_mg, &pitch, &roll);

    if (UINT32_MAX == g_tim.acc_min_us)
    {
        g_tim.acc_min_us = 0U;      // no read in this window
    }

    (void)tm_sprintf(line, (UB*)"[IMU] t %s acc %u-%u mag %u loop %u-%u"
                     " wcet %u per %u-%u late %u err %u | p %d r %d"
                     " hdg %d d %d\n",
                     g_state_name[g_state],
                     (UINT)g_tim.acc_min_us, (UINT)g_tim.acc_max_us,
                     (UINT)g_tim.mag_max_us,
                     (UINT)g_tim.loop_min_us, (UINT)g_tim.loop_max_us,
                     (UINT)g_wcet_us,
                     (UINT)g_tim.per_min_us, (UINT)g_tim.per_max_us,
                     (UINT)g_tim.late, (UINT)g_tim.errors,
                     (INT)pitch, (INT)roll, (INT)heading,
                     (INT)motion_get_distance_mm());
    tm_printf(line);
}

// RP2040 TIMERAWL: free-running 1 us counter (wraps every 71 min).
static uint32_t
imu_now_us (void)
{
    return ((uint32_t)in_w(TIMER_TIMERAWL));
}
