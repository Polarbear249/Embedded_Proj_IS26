#include "servo.h"
#include "hardware/pwm.h"
#include "hardware/gpio.h"

#define SERVO_PIN 12
static uint slice_num;

// RP2040 default sys_clk is 125MHz
// Setting clkdiv to 125 gives us a 1MHz PWM clock (1 tick = 1 microsecond) theoretically
#define SERVO_CLKDIV 125.0f
// 20ms period = 20,000 microseconds
#define SERVO_WRAP   20000

ER servo_init(void) {
    // Tell GPIO to route PWM to this pin
    gpio_set_function(SERVO_PIN, GPIO_FUNC_PWM);

    // Find which of the 8 PWM slices this pin belongs to (GPIO 12 is slice 6)
    slice_num = pwm_gpio_to_slice_num(SERVO_PIN);
    
    pwm_config config = pwm_get_default_config();
    
    // Configure the frequency to 50Hz (1us tick, 20ms period)
    pwm_config_set_clkdiv(&config, SERVO_CLKDIV);
    pwm_config_set_wrap(&config, SERVO_WRAP);
    
    // Initialise the PWM slice with the configuration and start it
    pwm_init(slice_num, &config, true);
    return E_OK;
}

void servo_set_angle(UINT8 angle) {
    if (angle > 180) angle = 180; // Clamp angle to 180 degrees
    
    // Now, 1 tick = 1 microsecond. 
    // So the pulse width in microseconds is the tick level.
    uint32_t pulse_us = 1000 + ((2000 - 1000) * angle / 180);
    
    // Write the pulse width directly (e.g., 1500 ticks = 1500us = 1.5ms)
    // Note: PWM_CHAN_A is hardcoded cuz GPIO 12 maps to channel A, if change, then most likely need to change this too
    pwm_set_chan_level(slice_num, PWM_CHAN_A, pulse_us);
}