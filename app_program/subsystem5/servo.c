#include "servo.h"
#include <sys/machine.h> // For PWM registers

#define SERVO_PIN 12

ER servo_init(void) {
    ER er;
    // Tell GPIO to route PWM to this pin (NOT SIO)
    er = pwm_set_pin(SERVO_PIN); 
    if (er < E_OK) return er;

    // Find which of the 8 PWM slices this pin belongs to (GPIO 12 is slice 6)
    // Configure the frequency to 50Hz (1us tick, 20ms period)
    // BSP function to set clock divider and wrap value:
    out_w(PWM_CH6_BASE + PWM_CHx_CSR, 0U);         // Disable while configuring
    out_w(PWM_CH6_BASE + PWM_CHx_DIV, 125U << 24); // 1MHz clock (1 tick = 1us)
    out_w(PWM_CH6_BASE + PWM_CHx_TOP, 19999U);     // 20,000us period (50Hz)
    
    // Initialise the PWM slice with the configuration and start it
    out_w(PWM_CH6_BASE + PWM_CHx_CC, 1500U);       // Center (1.5ms)
    out_w(PWM_CH6_BASE + PWM_CHx_CSR, PWM_CH_CSR_EN); // Enable
    
    return E_OK;
}

void servo_set_angle(UINT8 angle) {
    if (angle > 180) angle = 180; // Clamp angle to 180 degrees
    
    // Now, 1 tick = 1 microsecond. 
    // So the pulse width in microseconds is the tick level.
    UW pulse_us = 1000 + ((2000 - 1000) * angle / 180);
    
    // Write the pulse width directly (e.g., 1500 ticks = 1500us = 1.5ms)
    out_w(PWM_CH6_BASE + PWM_CHx_CC, pulse_us);
}