/*
 * line_drive.c
 * Subsystem 3 (Buddy 3): line position -> wheel duties, see
 * line_drive.h.
 */

#include <stddef.h>
#include "line_drive.h"
#include "line_if.h"

_Static_assert(DRIVE_ARM_TICKS <= 255U, "arm_count is 8 bits");
_Static_assert((DRIVE_BASE_DUTY > 0) && (DRIVE_BASE_DUTY <= DRIVE_DUTY_MAX),
               "base duty range");
_Static_assert(DRIVE_KP_DEN > 0, "gain denominator");

static int16_t line_drive_clamp(int32_t duty);
static void    line_drive_arm(line_drive_t *p_ctx, uint8_t mode,
                              bool b_centre_black);

/* Straight-ahead duty, DRIVE_BASE_DUTY until MQTT SET_SPEED moves it.
   One aligned 16-bit store from the VC, read by the Line task: no lock
   needed, and a half-applied value is not possible. */
static int16_t g_base_duty = DRIVE_BASE_DUTY;

void
line_drive_set_base_duty (int16_t duty)
{
    if (duty < 0)
    {
        duty = 0;
    }
    else if (duty > DRIVE_DUTY_MAX)
    {
        duty = DRIVE_DUTY_MAX;
    }
    else
    {
        // in range
    }

    g_base_duty = duty;
}

void
line_drive_init (line_drive_t *p_ctx)
{
    if (NULL != p_ctx)
    {
        p_ctx->arm_count = 0U;
        p_ctx->b_armed   = 0U;
    }
    else
    {
        // nothing to initialise
    }
}

bool
line_drive_step (line_drive_t *p_ctx, uint8_t error, uint8_t mode,
                 bool b_centre_black, int16_t *p_left, int16_t *p_right)
{
    bool    b_drive = false;
    int32_t err_signed;
    int32_t gain;
    int32_t turn;

    if ((NULL != p_ctx) && (NULL != p_left) && (NULL != p_right))
    {
        *p_left  = 0;
        *p_right = 0;

        line_drive_arm(p_ctx, mode, b_centre_black);

        if (0U != p_ctx->b_armed)
        {
            // Cast: error is 0..127, widened before the subtraction
            err_signed = (int32_t)error - (int32_t)LINE_IF_ERROR_CENTRE;

            if ((LINE_MODE_SEARCH_NEAR == mode)
                || (LINE_MODE_SEARCH_FAR == mode))
            {
                gain = DRIVE_KP_SEARCH_NUM;     // turn on the spot
            }
            else
            {
                gain = DRIVE_KP_NUM;            // gentle corrections
            }

            turn = (err_signed * gain) / DRIVE_KP_DEN;

            // JUNCTION reports error 64, so it simply drives straight
            *p_left  = line_drive_clamp(g_base_duty + turn);
            *p_right = line_drive_clamp(g_base_duty - turn);
            b_drive  = true;
        }
        else
        {
            // not armed: stop
        }
    }
    else
    {
        // NULL argument: stop
    }

    return b_drive;
}

/*!
 * Arm after DRIVE_ARM_TICKS of centre black; disarm on FAILED / INIT.
 */
static void
line_drive_arm (line_drive_t *p_ctx, uint8_t mode, bool b_centre_black)
{
    if ((LINE_MODE_FAILED == mode) || (LINE_MODE_INIT == mode))
    {
        p_ctx->b_armed   = 0U;
        p_ctx->arm_count = 0U;
    }
    else if (0U != p_ctx->b_armed)
    {
        // already driving
    }
    else if (b_centre_black)
    {
        p_ctx->arm_count++;

        if (p_ctx->arm_count >= DRIVE_ARM_TICKS)
        {
            p_ctx->b_armed   = 1U;
            p_ctx->arm_count = 0U;
        }
        else
        {
            // keep counting
        }
    }
    else
    {
        // must be on the line for the whole arm time
        p_ctx->arm_count = 0U;
    }
}

static int16_t
line_drive_clamp (int32_t duty)
{
    int16_t clamped;

    if (duty > DRIVE_DUTY_MAX)
    {
        clamped = DRIVE_DUTY_MAX;
    }
    else if (duty < -DRIVE_DUTY_MAX)
    {
        clamped = -DRIVE_DUTY_MAX;
    }
    else
    {
        // Cast: inside +-200 here
        clamped = (int16_t)duty;
    }

    return clamped;
}
