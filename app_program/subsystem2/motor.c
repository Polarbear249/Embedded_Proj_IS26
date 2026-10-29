
#include <bsp/libbsp.h>
#include <tk/tkernel.h>

#include <stdint.h>

/*
 * Buddy 2 - Motor Driver
 * Raspberry Pi Pico W / Robo Pico
 *
 * Uses micro T-Kernel RP2040 BSP PWM functions.
 *
 * Physical motor mapping verified during testing:
 * LEFT  : GP10 forward PWM, GP11 inactive
 * RIGHT : GP9 forward PWM, GP8 inactive
 *
 * Bench-test PWM limits are preliminary.
 */

#define LEFT_MOTOR_A     10U
#define LEFT_MOTOR_B     11U

#define RIGHT_MOTOR_A     8U
#define RIGHT_MOTOR_B     9U

#define MOTOR_PWM_WRAP  999U
#define MOTOR_PWM_MAX   200U

static int motor_ready = 0;

/*
 * Initialise both motor PWM channels.
 *
 * Motors remain stopped after initialisation.
 */
ER motor_init(void)
{
    ER er;

    if (motor_ready)
    {
        return E_OK;
    }

    /* Configure motor pins for PWM */
    er = pwm_set_pin(LEFT_MOTOR_A);
    if (er < E_OK) return er;

    er = pwm_set_pin(LEFT_MOTOR_B);
    if (er < E_OK) return er;

    er = pwm_set_pin(RIGHT_MOTOR_A);
    if (er < E_OK) return er;

    er = pwm_set_pin(RIGHT_MOTOR_B);
    if (er < E_OK) return er;

    /* Configure each motor's PWM slice */
    er = pwm_set_wrap(LEFT_MOTOR_A, MOTOR_PWM_WRAP);
    if (er < E_OK) return er;

    er = pwm_set_wrap(RIGHT_MOTOR_A, MOTOR_PWM_WRAP);
    if (er < E_OK) return er;

    /* Set all motor channels to zero duty */
    pwm_set_cc(LEFT_MOTOR_A, 0U);
    pwm_set_cc(LEFT_MOTOR_B, 0U);

    pwm_set_cc(RIGHT_MOTOR_A, 0U);
    pwm_set_cc(RIGHT_MOTOR_B, 0U);

    /* Enable the two PWM slices */
    er = pwm_set_enabled(LEFT_MOTOR_A, TRUE);
    if (er < E_OK) return er;

    er = pwm_set_enabled(RIGHT_MOTOR_A, TRUE);
    if (er < E_OK) return er;

    motor_ready = 1;

    return E_OK;
}

/*
 * Apply forward PWM to the left motor.
 */
ER motor_set_left_pwm(uint16_t level)
{
    if (!motor_ready)
    {
        return E_OBJ;
    }

    if (level > MOTOR_PWM_MAX)
    {
        level = MOTOR_PWM_MAX;
    }

    pwm_set_cc(LEFT_MOTOR_B, 0U);

    return pwm_set_cc(LEFT_MOTOR_A, level);
}

/*
 * Apply forward PWM to the right motor.
 */
ER motor_set_right_pwm(uint16_t level)
{
    if (!motor_ready)
    {
        return E_OBJ;
    }

    if (level > MOTOR_PWM_MAX)
    {
        level = MOTOR_PWM_MAX;
    }

    pwm_set_cc(RIGHT_MOTOR_A, 0U);

    return pwm_set_cc(RIGHT_MOTOR_B, level);
}

/*
 * Stop both motors by setting PWM duty to zero.
 */
ER motor_stop(void)
{
    if (!motor_ready)
    {
        return E_OBJ;
    }

    pwm_set_cc(LEFT_MOTOR_A, 0U);
    pwm_set_cc(LEFT_MOTOR_B, 0U);

    pwm_set_cc(RIGHT_MOTOR_A, 0U);
    pwm_set_cc(RIGHT_MOTOR_B, 0U);

    return E_OK;
}
