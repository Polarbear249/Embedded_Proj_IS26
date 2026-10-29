
#include "motion_control.h"

#include "motor.h"
#include "encoder.h"
#include "pid.h"

#include <stdint.h>

/*
 * Buddy 2 - Motion Control Coordinator
 *
 * Intended execution model:
 * - One task owns motion commands and motion_update().
 * - encoder_sample() runs separately at a faster rate.
 * - motion_update() runs every 100 ms.
 *
 * Current implementation supports forward wheel-speed
 * control only. Distance and turning are not implemented.
 */

#define MOTION_UPDATE_MS        100U
#define MOTION_DT_SECONDS       0.1f

/* Preliminary bench-test settings */
#define MOTION_KP               2.0f
#define MOTION_KI               2.0f

#define MOTION_BASE_PWM         85.0f
#define MOTION_MAX_PWM          200.0f

#define MOTION_START_PWM        150U
#define MOTION_START_CYCLES     2U

#define MOTION_MAX_TARGET_RPM   30.0f

static int motion_ready = 0;

static motion_state_t motion_state =
    MOTION_STATE_STOPPED;

static float target_left_rpm = 0.0f;
static float target_right_rpm = 0.0f;

static float actual_left_rpm = 0.0f;
static float actual_right_rpm = 0.0f;

static int32_t previous_left_count = 0;
static int32_t previous_right_count = 0;

static uint32_t startup_cycles = 0U;

static pi_controller_t left_pi;
static pi_controller_t right_pi;

/*
 * Convert a floating-point PWM command into an
 * integer motor-driver command.
 */
static uint16_t motion_pwm_value(float value)
{
    if (value <= 0.0f)
    {
        return 0U;
    }

    if (value >= MOTION_MAX_PWM)
    {
        return (uint16_t)MOTION_MAX_PWM;
    }

    return (uint16_t)value;
}

/*
 * Enter fault state and stop both motors.
 */
static void motion_enter_fault(void)
{
    (void)motor_stop();

    target_left_rpm = 0.0f;
    target_right_rpm = 0.0f;

    motion_state = MOTION_STATE_FAULT;
}

/*
 * Initialise the motor driver, encoder inputs
 * and two independent PI controllers.
 */
motion_result_t motion_init(void)
{
    ER er;

    if (motion_ready)
    {
        return MOTION_OK;
    }

    er = motor_init();

    if (er != E_OK)
    {
        return MOTION_ERROR;
    }

    er = encoder_init();

    if (er != E_OK)
    {
        (void)motor_stop();
        return MOTION_ERROR;
    }

    pi_init(
        &left_pi,
        MOTION_KP,
        MOTION_KI,
        -MOTION_BASE_PWM,
        MOTION_MAX_PWM - MOTION_BASE_PWM
    );

    pi_init(
        &right_pi,
        MOTION_KP,
        MOTION_KI,
        -MOTION_BASE_PWM,
        MOTION_MAX_PWM - MOTION_BASE_PWM
    );

    encoder_get_counts(
        &previous_left_count,
        &previous_right_count
    );

    target_left_rpm = 0.0f;
    target_right_rpm = 0.0f;

    actual_left_rpm = 0.0f;
    actual_right_rpm = 0.0f;

    startup_cycles = 0U;
    motion_state = MOTION_STATE_STOPPED;
    motion_ready = 1;

    return MOTION_OK;
}

/*
 * Request forward wheel speeds.
 *
 * Negative or excessive speeds are rejected.
 * Actual PWM changes occur in motion_update().
 */
motion_result_t motion_set_speed(
    float left_rpm,
    float right_rpm)
{
    if (!motion_ready)
    {
        return MOTION_NOT_READY;
    }

    if (motion_state == MOTION_STATE_FAULT)
    {
        return MOTION_ERROR;
    }

    /*
     * These comparisons also reject NaN.
     */
    if (!(left_rpm >= 0.0f &&
          left_rpm <= MOTION_MAX_TARGET_RPM &&
          right_rpm >= 0.0f &&
          right_rpm <= MOTION_MAX_TARGET_RPM))
    {
        return MOTION_ERROR;
    }

    if (left_rpm == 0.0f && right_rpm == 0.0f)
    {
        return motion_stop();
    }

    if (motion_state == MOTION_STATE_STOPPED)
    {
        pi_reset(&left_pi);
        pi_reset(&right_pi);

        encoder_get_counts(
            &previous_left_count,
            &previous_right_count
        );

        startup_cycles = MOTION_START_CYCLES;
    }

    target_left_rpm = left_rpm;
    target_right_rpm = right_rpm;

    motion_state = MOTION_STATE_RUNNING;

    return MOTION_OK;
}

/*
 * Stop both motors and reset controller state.
 *
 * A zero-PWM stop does not guarantee that the
 * car stops physically without coasting.
 */
motion_result_t motion_stop(void)
{
    if (!motion_ready)
    {
        return MOTION_NOT_READY;
    }

    if (motor_stop() != E_OK)
    {
        motion_enter_fault();
        return MOTION_ERROR;
    }

    target_left_rpm = 0.0f;
    target_right_rpm = 0.0f;

    pi_reset(&left_pi);
    pi_reset(&right_pi);

    startup_cycles = 0U;

    if (motion_state != MOTION_STATE_FAULT)
    {
        motion_state = MOTION_STATE_STOPPED;
    }

    return MOTION_OK;
}

/*
 * Return current wheel status.
 */
motion_result_t motion_get_status(
    motion_status_t *status)
{
    if (status == 0)
    {
        return MOTION_ERROR;
    }

    if (!motion_ready)
    {
        return MOTION_NOT_READY;
    }

    status->left_rpm = actual_left_rpm;
    status->right_rpm = actual_right_rpm;

    encoder_get_counts(
        &status->left_encoder_count,
        &status->right_encoder_count
    );

    status->state = motion_state;

    return MOTION_OK;
}

/*
 * Periodic 100 ms speed-control update.
 *
 * IMPORTANT:
 * Encoder sampling must happen independently
 * and much more frequently than this function.
 */
void motion_update(void)
{
    int32_t left_count;
    int32_t right_count;

    int32_t delta_left;
    int32_t delta_right;

    float left_pwm;
    float right_pwm;

    ER er;

    if (!motion_ready)
    {
        return;
    }

    encoder_get_counts(&left_count, &right_count);

    delta_left = left_count - previous_left_count;
    delta_right = right_count - previous_right_count;

    previous_left_count = left_count;
    previous_right_count = right_count;

    actual_left_rpm = encoder_calculate_left_rpm(
        delta_left,
        MOTION_UPDATE_MS
    );

    actual_right_rpm = encoder_calculate_right_rpm(
        delta_right,
        MOTION_UPDATE_MS
    );

    if (motion_state != MOTION_STATE_RUNNING)
    {
        return;
    }

    /*
     * Preliminary startup boost.
     * Applies for two control updates.
     */
    if (startup_cycles > 0U)
    {
        er = motor_set_left_pwm(
            target_left_rpm > 0.0f ?
            MOTION_START_PWM : 0U
        );

        if (er != E_OK)
        {
            motion_enter_fault();
            return;
        }

        er = motor_set_right_pwm(
            target_right_rpm > 0.0f ?
            MOTION_START_PWM : 0U
        );

        if (er != E_OK)
        {
            motion_enter_fault();
            return;
        }

        startup_cycles--;
        return;
    }

    left_pwm = 0.0f;
    right_pwm = 0.0f;

    if (target_left_rpm > 0.0f)
    {
        left_pwm = MOTION_BASE_PWM + pi_update(
            &left_pi,
            target_left_rpm,
            actual_left_rpm,
            MOTION_DT_SECONDS
        );
    }
    else
    {
        pi_reset(&left_pi);
    }

    if (target_right_rpm > 0.0f)
    {
        right_pwm = MOTION_BASE_PWM + pi_update(
            &right_pi,
            target_right_rpm,
            actual_right_rpm,
            MOTION_DT_SECONDS
        );
    }
    else
    {
        pi_reset(&right_pi);
    }

    er = motor_set_left_pwm(
        motion_pwm_value(left_pwm)
    );

    if (er != E_OK)
    {
        motion_enter_fault();
        return;
    }

    er = motor_set_right_pwm(
        motion_pwm_value(right_pwm)
    );

    if (er != E_OK)
    {
        motion_enter_fault();
    }
}

/*
 * Navigation commands reserved for future work.
 */
motion_result_t motion_move_forward(float distance_cm)
{
    (void)distance_cm;
    return MOTION_NOT_IMPLEMENTED;
}

motion_result_t motion_move_backward(float distance_cm)
{
    (void)distance_cm;
    return MOTION_NOT_IMPLEMENTED;
}

motion_result_t motion_turn_left(float angle_deg)
{
    (void)angle_deg;
    return MOTION_NOT_IMPLEMENTED;
}

motion_result_t motion_turn_right(float angle_deg)
{
    (void)angle_deg;
    return MOTION_NOT_IMPLEMENTED;
}
