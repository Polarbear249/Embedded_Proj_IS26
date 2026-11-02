#include "ultrasonic.h"
#include <sys/machine.h> // For in_w() and hardware registers

#define TRIG_PIN 16
#define ECHO_PIN 17

// TIMERAWL is not exposed in the µT-Kernel BSP headers.
// We hardcode the RP2040 system timer register address.
#define TIMERAWL_REG (0x40054000 + 0x28)

ER ultrasonic_init(void) {
    ER er;
    // Use GPIO_MODE_OUT, not GPIO_OUT (which is a register address in the BSP)
    er = gpio_set_pin(TRIG_PIN, GPIO_MODE_OUT);
    if (er < E_OK) return er;
    gpio_set_val(TRIG_PIN, 0);  // Start LOW

    // Use GPIO_MODE_IN
    er = gpio_set_pin(ECHO_PIN, GPIO_MODE_IN);
    return er;
}

/*
    Measures distance in centimeters.
    Returns -1.0 if no echo is received (timeout).
    WARNING: This function blocks the CPU using busy-waiting loops.
*/
float ultrasonic_measure(void) {
    UW start_time, end_time;
    
    // Send a 10 microsecond HIGH pulse to trigger the sensor
    // We use the hardware timer (TIMERAWL_REG) instead of tk_dly_tsk because the RTOS tick is 10ms
    UW t0 = in_w(TIMERAWL_REG);
    gpio_set_val(TRIG_PIN, 1);
    while ((UW)(in_w(TIMERAWL_REG) - t0) < 10U) { }
    gpio_set_val(TRIG_PIN, 0);

    // Wait for the ECHO pin to go HIGH (sensor is sending sound)
    t0 = in_w(TIMERAWL_REG);
    while (gpio_get_val(ECHO_PIN) == 0) {
        // 5ms timeout
        if ((UW)(in_w(TIMERAWL_REG) - t0) > 5000) return -1.0; 
    }

    // Wait for the ECHO pin to go LOW (sound has returned)
    start_time = in_w(TIMERAWL_REG);
    while (gpio_get_val(ECHO_PIN) == 1) {
        // 30ms timeout
        if ((UW)(in_w(TIMERAWL_REG) - start_time) > 30000) return -1.0; 
    }
    end_time = in_w(TIMERAWL_REG);
    
    // Calculate the distance: Time it took for the sound to travel there and back.
    // Speed of sound is 343m/s / 29.1 microseconds per cm
    // Since the sound travels there and back, 1cm of distance = ~58 microseconds.
    UW pulse_width = end_time - start_time;
    return (float)pulse_width / 58.0;
}