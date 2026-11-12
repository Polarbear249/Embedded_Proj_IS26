/*
 * servo.h
 * Subsystem 5 (Buddy 5): scan servo on GP12 (servo port 1, PWM slice 6
 * channel A; TEAM.md section 2). 50 Hz, 1000..2000 us pulse for
 * 0..180 deg.
 */

#ifndef S5_SERVO_H
#define S5_SERVO_H

#include <stdint.h>
#include "../app.h"

#define SERVO_PIN               (12U)
#define SERVO_MAX_DEG           (180U)

ER   servo_init(void);
void servo_set_angle(uint32_t angle_deg);

#endif // S5_SERVO_H
