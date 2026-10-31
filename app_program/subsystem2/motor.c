
#include <sys/machine.h>
#include <tk/tkernel.h>
#include <bsp/libbsp.h>
#include <stdint.h>
#include "motor.h"

#ifndef MOTOR_TEST_HIGH_POWER
#error "MOTOR_TEST_HIGH_POWER must come from motor.h"
#endif
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
#if MOTOR_TEST_HIGH_POWER
#define MOTOR_PWM_MAX   500U    /* test only, see motor.h */
#else
#define MOTOR_PWM_MAX   200U
#endif

/*
 * PWM clock divider = SYSCLK in MHz (125): the counter runs at 1 MHz,
 * so with WRAP 999 the motor PWM is 1 kHz. That is the frequency the
 * bench PI tuning used (testing/subsystem2/dual_motor_pid_test,
 * PWM_CLKDIV 125), so BASE 85 / START 150 / MAX 200 keep their meaning.
 * Without it the slices ran at 125 kHz, above the Robo Pico driver's
 * 20 kHz limit: the motors did not move or even hum (2026-10-31).
 * libbsp has no divider call, so the DIV register is written directly
 * (integer part in bits 11:4).
 */
#define MOTOR_PWM_DIV_INT    ((UW)SYSCLK)
#define MOTOR_PWM_DIV_SHIFT  4U
#define MOTOR_PWM_SLICE(pin) (((UW)(pin) >> 1) & 0x07U)
#define MOTOR_PWM_CH_STRIDE  0x14U
#define MOTOR_PWM_DIV_REG(pin) \
    (PWM_BASE + PWM_CHx_DIV + (MOTOR_PWM_SLICE(pin) * MOTOR_PWM_CH_STRIDE))

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

    /*
     * Take the PWM block out of reset. The kernel releases it only when
     * its physical timer uses PWM (config/config.h USE_PTMR, 0 here), so
     * until now every PWM register write was ignored and the motor driver
     * inputs got no signal (2026-10-31: the Robo Pico motor test buttons
     * turned the wheels, the software did not). Release only, no reset
     * pulse: safe to repeat, so S5's servo code can do the same without
     * wiping this set-up.
     */
    clr_w(RESETS_RESET, RESETS_RESET_PWM);

    while (0U == (in_w(RESETS_RESET_DONE) & RESETS_RESET_PWM))
    {
        /* wait for the PWM block to leave reset */
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

    /* 1 kHz PWM (see MOTOR_PWM_DIV_INT) */
    out_w(MOTOR_PWM_DIV_REG(LEFT_MOTOR_A),
          MOTOR_PWM_DIV_INT << MOTOR_PWM_DIV_SHIFT);
    out_w(MOTOR_PWM_DIV_REG(RIGHT_MOTOR_A),
          MOTOR_PWM_DIV_INT << MOTOR_PWM_DIV_SHIFT);

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
