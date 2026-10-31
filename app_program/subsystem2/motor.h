
#ifndef MOTOR_H
#define MOTOR_H

#include <tk/tkernel.h>
#include <stdint.h>

/*
 * Buddy 2 - Motor Driver Interface
 * Raspberry Pi Pico W / Robo Pico
 *
 * Internal interface for the motion-control subsystem.
 * Other buddies should use motion_control.h instead.
 */

/*
 * TEST ONLY (added by Buddy 4, 2026-10-31, Buddy 2 to review): raise the
 * power limits so the wheels start on laptop USB power (about 4.6 V).
 * At Buddy 2's limits (20 % max, 15 % start kick) the PI reached its
 * maximum and the encoder watchdog stopped the motors after 1 s without
 * a single encoder count. 1 = 50 % max, 35 % start; 0 = Buddy 2's
 * tuned values. The motor driver allows 1 A per channel; a stalled
 * wheel is still stopped by the 1 s encoder watchdog.
 */
#define MOTOR_TEST_HIGH_POWER   (1)

/* Initialise both motor PWM channels */
ER motor_init(void);

/* Set left motor forward PWM (0-200) */
ER motor_set_left_pwm(uint16_t level);

/* Set right motor forward PWM (0-200) */
ER motor_set_right_pwm(uint16_t level);

/* Stop both motors */
ER motor_stop(void);

#endif /* MOTOR_H */
