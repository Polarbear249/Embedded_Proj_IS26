#include "ultrasonic.h"

#define TRIG_PIN 16
#define ECHO_PIN 17

ER ultrasonic_init(void) {
    // Trigger pin (Output)
    gpio_set_pin(TRIG_PIN);     // T-Kernel BSP function to set pin mode
    gpio_set_dir(TRIG_PIN, 1);  // 1 = Output
    gpio_set_val(TRIG_PIN, 0);  // Start LOW

    // Echo pin (Input)
    gpio_set_pin(ECHO_PIN);
    gpio_set_dir(ECHO_PIN, 0);  // 0 = Input

    
    return E_OK;
}

/*
    Measures distance in centimeters.
    Returns -1.0 if no echo is received (timeout).
    WARNING: This function blocks the CPU using busy-waiting loops.
*/
float ultrasonic_measure(void) {
    SYSTIM start_time, current_time;
    UW pulse_width_us;
    
    // Send a 10 microsecond HIGH pulse to trigger the sensor
    gpio_set_val(TRIG_PIN, 1);
    tk_dly_tsk(1); // T-Kernel delay is in ms. 1ms is plenty for a 10us trigger
    gpio_set_val(TRIG_PIN, 0);

    // Wait for the ECHO pin to go HIGH
    tk_get_tim(&start_time);
    while (gpio_get_val(ECHO_PIN) == 0) {
        tk_get_tim(&current_time);
        // 5ms timeout
        if ((current_time.lo - start_time.lo) > 5) return -1.0; 
    }

    // Wait for the ECHO pin to go LOW
    tk_get_tim(&start_time);
    while (gpio_get_val(ECHO_PIN) == 1) {
        tk_get_tim(&current_time);
        // 30ms timeout
        if ((current_time.lo - start_time.lo) > 30) return -1.0; 
    }
    
    // Calculate pulse width in milliseconds (since tk_get_tim is ms)
    pulse_width_us = (current_time.lo - start_time.lo) * 1000; 
    
    // Return distance in cm
    return (float)pulse_width_us / 58.0;
}