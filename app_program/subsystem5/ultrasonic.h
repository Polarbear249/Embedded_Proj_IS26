#ifndef S5_ULTRASONIC_H
#define S5_ULTRASONIC_H
#include "../app.h"

// Initialize GP16 (TRIG) and GP17 (ECHO)
ER ultrasonic_init(void);

// Trigger the sensor and return the distance in cm. 
// Returns -1.0 if no echo is received (timeout).
float ultrasonic_measure(void);

#endif