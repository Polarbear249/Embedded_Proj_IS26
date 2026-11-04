/*
 * imu_hump.c
 * Subsystem 4 (Buddy 4): hump detection and height, see imu_hump.h.
 * Integer only (no FPU). Worst case per update: IMU_HUMP_JUMP_MM steps,
 * each one ring push (two adds, two subtracts), two divides, two sines
 * and a few adds (at 0.5 m/s about 5 steps per 10 ms cycle, derived).
 */

#include "imu_hump.h"
#include "imu_calib.h"
#include <stddef.h>

#define IMU_HUMP_UM_PER_MM      (1000)
#define IMU_HUMP_SIN_PER_UM     (10)    // L(mm) x sin x10000 / 10 = um

static void        imu_hump_clear_event(imu_hump_t *p_hump);
static imu_event_t imu_hump_step(imu_hump_t *p_hump, int32_t pitch_ddeg,
                                 int32_t tilt_ddeg, int32_t side_um,
                                 bool b_may_start, int32_t s_mm);
static void        imu_hump_avg_clear(imu_hump_t *p_hump);
static void        imu_hump_avg_push(imu_hump_t *p_hump, int32_t s_mm,
                                     int32_t pitch_ddeg, int32_t roll_ddeg,
                                     int32_t *p_pitch_avg,
                                     int32_t *p_roll_avg);
static imu_event_t imu_hump_finish(imu_hump_t *p_hump);
static imu_event_t imu_hump_still(imu_hump_t *p_hump);
static uint32_t    imu_hump_height_mm(int32_t peak_um);
static int32_t     imu_hump_abs(int32_t value);

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
        p_hump->b_from_still = true;    // next travel is a drive-off
        p_hump->driveoff_mm = 0;
        imu_hump_avg_clear(p_hump);
        imu_hump_clear_event(p_hump);
    }
}

/*!
 * One sample: pitch and roll (0.1 deg, against level) and the cumulative
 * wheel distance (mm, IF-10). Returns the event this sample produced.
 */
imu_event_t
imu_hump_update (imu_hump_t *p_hump, int32_t pitch_ddeg, int32_t roll_ddeg,
                 int32_t dist_mm)
{
    imu_event_t event = IMU_EVT_NONE;
    imu_event_t step_event = IMU_EVT_NONE;
    int32_t     s_mm;
    int32_t     pitch_avg = 0;
    int32_t     roll_avg = 0;
    int32_t     roll_mag;
    int32_t     pitch_mag;
    int32_t     tilt;
    int32_t     side_um;
    bool        b_may_start;

    if (NULL == p_hump)
    {
        event = IMU_EVT_NONE;
    }
    else if ((false == p_hump->b_synced)
             || ((dist_mm - p_hump->s_last_mm) > IMU_HUMP_JUMP_MM))
    {
        // First sample, or a jump the car cannot make in one cycle
        imu_hump_clear_event(p_hump);
        imu_hump_avg_clear(p_hump);
        p_hump->b_from_still = true;
        p_hump->s_last_mm = dist_mm;
        p_hump->b_synced = true;
    }
    else if (dist_mm <= p_hump->s_last_mm)
    {
        // Standing still or reversing: no profile, maybe close the hump
        event = imu_hump_still(p_hump);
    }
    else
    {
        // Each new forward mm
        p_hump->still_calls = 0U;

        if (p_hump->b_from_still)
        {
            p_hump->driveoff_mm = IMU_HUMP_DRIVEOFF_MM;
            p_hump->b_from_still = false;
        }

        for (s_mm = p_hump->s_last_mm + 1; s_mm <= dist_mm; s_mm++)
        {
            // Both models see the mean over the last IMU_HUMP_AVG_MM mm
            imu_hump_avg_push(p_hump, s_mm, pitch_ddeg, roll_ddeg,
                              &pitch_avg, &roll_avg);
            roll_mag = imu_hump_abs(roll_avg);
            pitch_mag = imu_hump_abs(pitch_avg);
            tilt = (roll_mag > pitch_mag) ? roll_mag : pitch_mag;
            // Side model: height of one rear wheel over the other, um
            side_um = (IMU_HUMP_TRACK_MM * imu_calib_sin_x10000(roll_mag))
                      / IMU_HUMP_SIN_PER_UM;
            b_may_start = (p_hump->driveoff_mm <= 0);
            p_hump->driveoff_mm = b_may_start ? 0
                                              : (p_hump->driveoff_mm - 1);
            step_event = imu_hump_step(p_hump, pitch_avg, tilt, side_um,
                                       b_may_start, s_mm);
            event = (IMU_EVT_NONE != step_event) ? step_event : event;
        }

        p_hump->s_last_mm = dist_mm;
    }

    return (event);
}

/*!
 * Current hump profile, mm (0 off a hump): the larger of the ground
 * height under the front contact and the side (one-wheel) height.
 */
int32_t
imu_hump_now_mm (imu_hump_t const *p_hump)
{
    int32_t now = 0;

    if (NULL != p_hump)
    {
        now = (p_hump->side_now_um > p_hump->z_now_um)
              ? p_hump->side_now_um : p_hump->z_now_um;
        now = now / IMU_HUMP_UM_PER_MM;
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
    p_hump->len_mm = 0;
    p_hump->still_calls = 0U;
    p_hump->z_now_um = 0;
    p_hump->peak_um = 0;
    p_hump->side_now_um = 0;
    p_hump->side_peak_um = 0;
    p_hump->b_on_hump = false;
}

/*
 * One mm of forward travel at distance s_mm. tilt_ddeg = larger of
 * |pitch| and |roll|, side_um = side-model height for this sample (both
 * from the smoothed values); b_may_start false during a drive-off.
 */
static imu_event_t
imu_hump_step (imu_hump_t *p_hump, int32_t pitch_ddeg, int32_t tilt_ddeg,
               int32_t side_um, bool b_may_start, int32_t s_mm)
{
    imu_event_t event = IMU_EVT_NONE;
    int32_t     mag = tilt_ddeg;
    int32_t     slot = s_mm % IMU_HUMP_BASE_MM;
    // only read while no event is open: a drive-off cannot start one
    bool        b_above = b_may_start && (mag >= IMU_HUMP_ON_DDEG);

    slot = (slot < 0) ? (slot + IMU_HUMP_BASE_MM) : slot;

    if ((false == p_hump->b_on_hump) && (false == b_above))
    {
        // Level and no candidate: keep the profile at 0
        if (0 != p_hump->cand_mm)
        {
            imu_hump_clear_event(p_hump);
        }
    }
    else if (p_hump->len_mm >= IMU_HUMP_MAX_LEN_MM)
    {
        // Far too long for a hump: a slope or a level error. Drop it.
        imu_hump_clear_event(p_hump);
    }
    else
    {
        p_hump->len_mm++;

        // z(s) = z(s - L) + L sin(pitch); slot still holds z(s - L)
        p_hump->z_now_um = p_hump->z_um[slot]
                           + ((IMU_HUMP_BASE_MM
                               * imu_calib_sin_x10000(pitch_ddeg))
                              / IMU_HUMP_SIN_PER_UM);
        p_hump->z_um[slot] = p_hump->z_now_um;
        p_hump->peak_um = (p_hump->z_now_um > p_hump->peak_um)
                          ? p_hump->z_now_um : p_hump->peak_um;
        p_hump->side_now_um = side_um;
        p_hump->side_peak_um = (side_um > p_hump->side_peak_um)
                               ? side_um : p_hump->side_peak_um;

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
 * No forward travel this update. After IMU_HUMP_STILL_CALLS of them in
 * a row, a hump in progress is closed with its peak so far (the car
 * stopped on it or just after it), and a candidate is dropped.
 */
static imu_event_t
imu_hump_still (imu_hump_t *p_hump)
{
    imu_event_t event = IMU_EVT_NONE;

    if (p_hump->still_calls < IMU_HUMP_STILL_CALLS)
    {
        p_hump->still_calls++;
    }

    if (IMU_HUMP_STILL_CALLS != p_hump->still_calls)
    {
        // not still for long enough yet
    }
    else if (false == p_hump->b_from_still)
    {
        // Still for 1 s: the next travel is a drive-off. A hump in
        // progress is closed on the next call (still_calls stays here).
        p_hump->b_from_still = true;
    }
    else if (p_hump->b_on_hump)
    {
        event = imu_hump_finish(p_hump);   // clears the event as well
    }
    else if (0 != p_hump->cand_mm)
    {
        imu_hump_clear_event(p_hump);       // unconfirmed: just drop it
    }
    else
    {
        // nothing in progress
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
    // Front model for a full-width hump, side model for a one-wheel one
    int32_t     peak = (p_hump->side_peak_um > p_hump->peak_um)
                       ? p_hump->side_peak_um : p_hump->peak_um;
    uint32_t    height = imu_hump_height_mm(peak);

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

// Empty smoothing rings (start, resync, jump)
static void
imu_hump_avg_clear (imu_hump_t *p_hump)
{
    uint32_t idx;

    for (idx = 0U; idx < (uint32_t)IMU_HUMP_AVG_MM; idx++)
    {
        p_hump->avg_p[idx] = 0;
        p_hump->avg_r[idx] = 0;
    }

    p_hump->sum_p = 0;
    p_hump->sum_r = 0;
    p_hump->avg_n = 0U;
}

/*
 * Moving mean over the last IMU_HUMP_AVG_MM mm (fewer while the rings
 * fill): this mm's sample replaces the one from IMU_HUMP_AVG_MM ago.
 */
static void
imu_hump_avg_push (imu_hump_t *p_hump, int32_t s_mm, int32_t pitch_ddeg,
                   int32_t roll_ddeg, int32_t *p_pitch_avg,
                   int32_t *p_roll_avg)
{
    int32_t slot = s_mm % IMU_HUMP_AVG_MM;
    int32_t div;

    slot = (slot < 0) ? (slot + IMU_HUMP_AVG_MM) : slot;
    p_hump->sum_p += pitch_ddeg - (int32_t)p_hump->avg_p[slot];
    p_hump->sum_r += roll_ddeg - (int32_t)p_hump->avg_r[slot];
    p_hump->avg_p[slot] = (int16_t)pitch_ddeg;  // |angle| <= 1800
    p_hump->avg_r[slot] = (int16_t)roll_ddeg;

    if (p_hump->avg_n < (uint32_t)IMU_HUMP_AVG_MM)
    {
        p_hump->avg_n++;
    }

    div = (int32_t)p_hump->avg_n;
    *p_pitch_avg = p_hump->sum_p / div;
    *p_roll_avg = p_hump->sum_r / div;
}

static int32_t
imu_hump_abs (int32_t value)
{
    return ((value < 0) ? -value : value);
}
