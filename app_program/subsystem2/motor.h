
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

/* Initialise both motor PWM channels */
ER motor_init(void);

/* Set left motor forward PWM (0-200) */
ER motor_set_left_pwm(uint16_t level);

/* Set right motor forward PWM (0-200) */
ER motor_set_right_pwm(uint16_t level);

/* Stop both motors */
ER motor_stop(void);

#endif /* MOTOR_H */
