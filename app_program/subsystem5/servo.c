/*
 * servo.c
 * Subsystem 5 (Buddy 5): scan servo, see servo.h. Single owner of PWM
 * slice 6 (TEAM.md section 3): only the obstacle task calls
 * servo_set_angle().
 *
 * libbsp has no divider call, so DIV/TOP/CC are written directly, the
 * same way as subsystem2/motor.c. Depends on motor_init() (called from
 * subsystem2_init(), before subsystem5_init()) having taken the PWM
 * block out of reset.
 */

#include "servo.h"
#include <sys/machine.h>

// Divider = SYSCLK (MHz): 1 tick = 1 us. DIV.INT is bits 11:4 (RP2040
// datasheet 4.5.3), as in subsystem2/motor.c; the earlier 125 << 24 put
// it in reserved bits. SYSCLK 125 MHz is configured, not yet measured.
#define SERVO_DIV               (((UW)SYSCLK) << 4)
#define SERVO_TOP               (19999U)    // 20 000 us = 50 Hz
#define SERVO_MIN_US            (1000U)
#define SERVO_SPAN_US           (1000U)     // 1000..2000 us
#define SERVO_CENTRE_US         (1500U)

ER
servo_init (void)
{
    ER er = pwm_set_pin(SERVO_PIN);     // route GP12 to the PWM block

    if (E_OK <= er)
    {
        out_w(PWM_CH6_BASE + PWM_CHx_CSR, 0U);      // off while set up
        out_w(PWM_CH6_BASE + PWM_CHx_DIV, SERVO_DIV);
        out_w(PWM_CH6_BASE + PWM_CHx_TOP, SERVO_TOP);
        out_w(PWM_CH6_BASE + PWM_CHx_CC, SERVO_CENTRE_US);
        out_w(PWM_CH6_BASE + PWM_CHx_CSR, PWM_CH_CSR_EN);
    }

    return (er);
}

void
servo_set_angle (uint32_t angle_deg)
{
    uint32_t angle = (angle_deg > SERVO_MAX_DEG) ? SERVO_MAX_DEG : angle_deg;
    uint32_t pulse_us = SERVO_MIN_US + ((SERVO_SPAN_US * angle)
                                        / SERVO_MAX_DEG);

    // Channel A (low half of CC) = GP12; B (GP13) stays 0
    out_w(PWM_CH6_BASE + PWM_CHx_CC, pulse_us);
}
