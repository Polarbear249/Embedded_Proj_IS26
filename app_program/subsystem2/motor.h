#ifndef S2_MOTOR_H
#define S2_MOTOR_H
#include "../app.h"
/* Private driver: only the motion task may call these after startup. */
ER motor_init(void);
ER motor_apply(INT left, INT right);
void motor_stop(void);
#endif
