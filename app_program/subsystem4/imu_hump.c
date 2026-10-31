/*
 * imu_hump.c
 * Subsystem 4 (Buddy 4): hump detection and height, see imu_hump.h.
 * Integer only (no FPU). Worst case per update: IMU_HUMP_JUMP_MM steps
 * of one table lookup and a few adds (at 0.5 m/s about 5 steps per
 * 10 ms cycle, derived).
 */

#include "imu_hump.h"
#include "imu_calib.h"
#include <stddef.h>

#define IMU_HUMP_UM_PER_MM      (1000)
#define IMU_HUMP_SIN_PER_UM     (10)    // L(mm) x sin x10000 / 10 = um

static void        imu_hump_clear_event(imu_hump_t *p_hump);
static imu_event_t imu_hump_step(imu_hump_t *p_hump, int32_t pitch_ddeg,
                                 int32_t s_mm);
static imu_event_t imu_hump_finish(imu_hump_t *p_hump);
static uint32_t    imu_hump_height_mm(int32_t peak_um);

/*!
 * Start of run: no hump measured yet.
 */
void
imu_hump_init (imu_hump_t *p_hump)
{
    if (NULL != p_hump)
    {
        p_hump->last_mm = 0U;
        p_hump->max_mm = 0U;
        p_hump->b_valid = false;
        imu_hump_resync(p_hump);
    }
}

/*!
 * Forget any hump in progress and the distance reference, keep the
 * results. Use after a fault (pitch was unknown for a while).
 */
void
imu_hump_resync (imu_hump_t *p_hump)
{
    if (NULL != p_hump)
    {
        p_hump->s_last_mm = 0;
        p_hump->b_synced = false;
        imu_hump_clear_event(p_hump);
    }
}

/*!
 * One sample: pitch (0.1 deg, against level) and the cumulative wheel
 * distance (mm, IF-10). Returns the event this sample produced.
 */
imu_event_t
imu_hump_update (imu_hump_t *p_hump, int32_t pitch_ddeg, int32_t dist_mm)
{
    imu_event_t event = IMU_EVT_NONE;
    imu_event_t step_event = IMU_EVT_NONE;
    int32_t     s_mm;

    if (NULL == p_hump)
    {
        event = IMU_EVT_NONE;
    }
    else if ((false == p_hump->b_synced)
             || ((dist_mm - p_hump->s_last_mm) > IMU_HUMP_JUMP_MM))
    {
        // First sample, or a jump the car cannot make in one cycle
        imu_hump_clear_event(p_hump);
        p_hump->s_last_mm = dist_mm;
        p_hump->b_synced = true;
    }
    else
    {
        // Each new forward mm; standing still or reversing adds nothing
        for (s_mm = p_hump->s_last_mm + 1; s_mm <= dist_mm; s_mm++)
        {
            step_event = imu_hump_step(p_hump, pitch_ddeg, s_mm);
            event = (IMU_EVT_NONE != step_event) ? step_event : event;
        }

        p_hump->s_last_mm = (dist_mm > p_hump->s_last_mm)
                            ? dist_mm : p_hump->s_last_mm;
    }

    return (event);
}

/*!
 * Ground height under the front contact right now, mm (0 off a hump).
 */
int32_t
imu_hump_now_mm (imu_hump_t const *p_hump)
{
    int32_t now = 0;

    if (NULL != p_hump)
    {
        now = p_hump->z_now_um / IMU_HUMP_UM_PER_MM;
    }

    return (now);
}

/*!
 * IF-11 word: [7:0] max mm, [15:8] last mm, [19:16] state, [22:20]
 * event, [23] valid, [31:24] seq. Inputs are masked to their widths.
 */
uint32_t
imu_hump_pack (imu_hump_t const *p_hump, uint32_t state, uint32_t event,
               uint32_t seq)
{
    uint32_t word = 0U;

    if (NULL != p_hump)
    {
        word = (p_hump->max_mm & 0xFFU)
               | ((p_hump->last_mm & 0xFFU) << 8)
               | ((state & 0x0FU) << 16)
               | ((event & 0x07U) << 20)
               | ((p_hump->b_valid ? 1U : 0U) << 23)
               | ((seq & 0xFFU) << 24);
    }

    return (word);
}

static void
imu_hump_clear_event (imu_hump_t *p_hump)
{
    uint32_t idx;

    for (idx = 0U; idx < (uint32_t)IMU_HUMP_BASE_MM; idx++)
    {
        p_hump->z_um[idx] = 0;
    }

    p_hump->cand_mm = 0;
    p_hump->quiet_mm = 0;
    p_hump->z_now_um = 0;
    p_hump->peak_um = 0;
    p_hump->b_on_hump = false;
}

/*
 * One mm of forward travel at distance s_mm.
 */
static imu_event_t
imu_hump_step (imu_hump_t *p_hump, int32_t pitch_ddeg, int32_t s_mm)
{
    imu_event_t event = IMU_EVT_NONE;
    int32_t     mag = (pitch_ddeg < 0) ? -pitch_ddeg : pitch_ddeg;
    int32_t     slot = s_mm % IMU_HUMP_BASE_MM;
    bool        b_above = (mag >= IMU_HUMP_ON_DDEG);

    slot = (slot < 0) ? (slot + IMU_HUMP_BASE_MM) : slot;

    if ((false == p_hump->b_on_hump) && (false == b_above))
    {
        // Level and no candidate: keep the profile at 0
        if (0 != p_hump->cand_mm)
        {
            imu_hump_clear_event(p_hump);
        }
    }
    else
    {
        // z(s) = z(s - L) + L sin(pitch); slot still holds z(s - L)
        p_hump->z_now_um = p_hump->z_um[slot]
                           + ((IMU_HUMP_BASE_MM
                               * imu_calib_sin_x10000(pitch_ddeg))
                              / IMU_HUMP_SIN_PER_UM);
        p_hump->z_um[slot] = p_hump->z_now_um;
        p_hump->peak_um = (p_hump->z_now_um > p_hump->peak_um)
                          ? p_hump->z_now_um : p_hump->peak_um;

        if (false == p_hump->b_on_hump)
        {
            p_hump->cand_mm++;

            if (p_hump->cand_mm >= IMU_HUMP_START_MM)
            {
                p_hump->b_on_hump = true;
                p_hump->quiet_mm = 0;
                event = IMU_EVT_HUMP_START;
            }
        }
        else
        {
            p_hump->quiet_mm = (mag < IMU_HUMP_OFF_DDEG)
                               ? (p_hump->quiet_mm + 1) : 0;

            if (p_hump->quiet_mm >= IMU_HUMP_END_MM)
            {
                event = imu_hump_finish(p_hump);
            }
        }
    }

    return (event);
}

/*
 * End of an event. One that never rose above its start (the way down a
 * ramp, or the far side of a plateau longer than IMU_HUMP_END_MM) is
 * not a hump: it ends quietly and the results are left alone.
 */
static imu_event_t
imu_hump_finish (imu_hump_t *p_hump)
{
    imu_event_t event = IMU_EVT_NONE;
    uint32_t    height = imu_hump_height_mm(p_hump->peak_um);

    if (0U != height)
    {
        p_hump->last_mm = height;
        p_hump->max_mm = (height > p_hump->max_mm) ? height
                                                   : p_hump->max_mm;
        p_hump->b_valid = true;
        event = IMU_EVT_HUMP_END;
    }

    imu_hump_clear_event(p_hump);

    return (event);
}

// Peak in um -> mm, rounded, clamped to the 8-bit field (never wraps)
static uint32_t
imu_hump_height_mm (int32_t peak_um)
{
    uint32_t height = 0U;

    if (peak_um > 0)
    {
        height = ((uint32_t)peak_um
                  + (uint32_t)(IMU_HUMP_UM_PER_MM / 2))
                 / (uint32_t)IMU_HUMP_UM_PER_MM;
    }

    return ((height > IMU_HUMP_MAX_MM) ? IMU_HUMP_MAX_MM : height);
}
