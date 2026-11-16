/*
 * imu_lift.c
 * Subsystem 4 (Buddy 4): lift / overturn detection, see imu_lift.h.
 * Pure functions (no kernel calls), also built by the PC tests.
 */

#include "imu_lift.h"
#include <stddef.h>

static int32_t imu_lift_abs(int32_t value);

void
imu_lift_init (imu_lift_t *p_lift)
{
    if (NULL != p_lift)
    {
        p_lift->vel_sum    = 0;
        p_lift->base_q     = 0;
        p_lift->calm_avg   = 0;
        p_lift->peak_sum   = 0;
        p_lift->tilt_count = 0U;
        p_lift->calm_count = 0U;
        p_lift->b_latched  = false;
    }
    else
    {
        // nothing to initialise
    }
}

/*!
 * One sample: dev_mg = |a| - g_ref, tilt against level in 0.1 deg.
 * Returns a result only on a change (detected, or put down).
 */
imu_lift_result_t
imu_lift_update (imu_lift_t *p_lift, int32_t dev_mg, int32_t pitch_ddeg,
                 int32_t roll_ddeg)
{
    imu_lift_result_t result = IMU_LIFT_NONE;
    int32_t           tilt = 0;
    int32_t           dev_hp = 0;

    if (NULL != p_lift)
    {
        tilt = imu_lift_abs(pitch_ddeg);
        tilt = (imu_lift_abs(roll_ddeg) > tilt)
               ? imu_lift_abs(roll_ddeg) : tilt;

        // remove the slow baseline (bias, drift), then leaky integral
        dev_hp = dev_mg - (p_lift->base_q / IMU_LIFT_BASE_Q);
        p_lift->base_q += ((dev_mg * IMU_LIFT_BASE_Q) - p_lift->base_q)
                          / IMU_LIFT_BASE_DIV;
        p_lift->vel_sum += dev_hp - (p_lift->vel_sum / IMU_LIFT_LEAK_DIV);
        p_lift->calm_avg += (dev_hp - p_lift->calm_avg)
                            / IMU_SETTLE_AVG_DIV;
        p_lift->peak_sum = (p_lift->vel_sum > p_lift->peak_sum)
                           ? p_lift->vel_sum : p_lift->peak_sum;
        p_lift->tilt_count = (tilt >= IMU_OVERTURN_DDEG)
                             ? (p_lift->tilt_count + 1U) : 0U;
        p_lift->calm_count = ((tilt <= IMU_SETTLE_DDEG)
                              && (imu_lift_abs(p_lift->calm_avg)
                                  <= IMU_SETTLE_MG))
                             ? (p_lift->calm_count + 1U) : 0U;

        if (p_lift->b_latched)
        {
            if (p_lift->calm_count >= IMU_SETTLE_SAMPLES)
            {
                // level and still for 1 s: put down; start afresh
                p_lift->b_latched  = false;
                p_lift->vel_sum    = 0;
                p_lift->tilt_count = 0U;
                result = IMU_LIFT_PUT_DOWN;
            }
            else
            {
                // still lifted / overturned
            }
        }
        else if (p_lift->tilt_count >= IMU_OVERTURN_SAMPLES)
        {
            p_lift->b_latched  = true;
            p_lift->calm_count = 0U;
            result = IMU_LIFT_OVERTURNED;
        }
        else if (p_lift->vel_sum >= IMU_LIFT_SUM_THRESH)
        {
            p_lift->b_latched  = true;
            p_lift->calm_count = 0U;
            result = IMU_LIFT_LIFTED;
        }
        else
        {
            // on its wheels
        }
    }
    else
    {
        // no state
    }

    return (result);
}

/*!
 * Estimated upward speed in mm/s for a leaky sum (1 mg x 10 ms =
 * 0.0981 mm/s).
 */
int32_t
imu_lift_vel_mm_s (int32_t sum)
{
    return ((sum * 981) / 10000);
}

/*!
 * Highest sum since the previous call (for the 1 s report line), then
 * restart the peak from the current value.
 */
int32_t
imu_lift_take_peak (imu_lift_t *p_lift)
{
    int32_t peak = 0;

    if (NULL != p_lift)
    {
        peak = p_lift->peak_sum;
        p_lift->peak_sum = p_lift->vel_sum;
    }
    else
    {
        // no state
    }

    return (peak);
}

static int32_t
imu_lift_abs (int32_t value)
{
    return ((value < 0) ? -value : value);
}
