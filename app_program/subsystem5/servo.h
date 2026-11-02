#ifndef S5_SERVO_H
#define S5_SERVO_H
#include "../app.h"

// Initialize GP12 for 50Hz PWM
ER servo_init(void);

// Set the servo angles
void servo_set_angle(UINT8 angle);

#endif