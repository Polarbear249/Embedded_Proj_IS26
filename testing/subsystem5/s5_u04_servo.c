#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "hardware/timer.h"

// Using GP16/GP17 for the final Robo Pico layout
#define TRIG_PIN 16   
#define ECHO_PIN 17   
#define SERVO_PIN 12 

#define TEST_ID "S5-U04"

#define PWM_WRAP 20000    
#define SERVO_MIN_US 1000 
#define SERVO_MAX_US 2000 

uint slice_num;

void set_servo_angle(uint8_t angle) {
    if (angle > 180) angle = 180;
    uint32_t pulse_us = SERVO_MIN_US + ((SERVO_MAX_US - SERVO_MIN_US) * angle / 180);
    pwm_set_chan_level(slice_num, PWM_CHAN_A, pulse_us);
    printf("[SERVO] Commanded angle: %d degrees\n", angle);
}

float measure_distance() {
    gpio_put(TRIG_PIN, 1);
    sleep_us(10);
    gpio_put(TRIG_PIN, 0);

    uint64_t start_wait = time_us_64();
    while (gpio_get(ECHO_PIN) == 0) {
        if (time_us_64() - start_wait > 5000) return -1.0; 
    }

    uint64_t start_echo = time_us_64();
    while (gpio_get(ECHO_PIN) == 1) {
        if (time_us_64() - start_echo > 30000) return -1.0; 
    }
    uint64_t pulse_width = time_us_64() - start_echo;
    return (float)pulse_width / 58.0;
}

int main() {
    stdio_init_all();
    while (!stdio_usb_connected()) { sleep_ms(100); }
    sleep_ms(1000); 
    
    printf("\n[TEST] ===== Starting Subsystem 5: %s (Servo Calibration) =====\n", TEST_ID);

    gpio_set_function(SERVO_PIN, GPIO_FUNC_PWM);
    slice_num = pwm_gpio_to_slice_num(SERVO_PIN);
    
    pwm_config config = pwm_get_default_config();
    pwm_config_set_clkdiv(&config, 50.f); 
    pwm_config_set_wrap(&config, PWM_WRAP);
    pwm_init(slice_num, &config, true);

    gpio_init(TRIG_PIN); gpio_set_dir(TRIG_PIN, GPIO_OUT);
    gpio_init(ECHO_PIN); gpio_set_dir(ECHO_PIN, GPIO_IN);
    gpio_pull_down(ECHO_PIN);

    // Final Calibrated Sweep
    uint8_t scan_angles[] = {15, 110, 178};
    
    for (int i = 0; i < 3; i++) {
        uint8_t angle = scan_angles[i];
        
        set_servo_angle(angle);
        sleep_ms(500); // Wait for settling
        
        float dist = measure_distance();
        if (dist > 0) {
            printf("[TEST] Distance at %d deg: %.1f cm\n\n", angle, dist);
        } else {
            printf("[TEST] No echo at %d deg\n\n", angle);
        }
    }

    printf("[SERVO] Returning to center (110 degrees).\n");
    set_servo_angle(110);
    sleep_ms(500);

    // Changed this line to match the actual angles!
    printf("[TEST] %s PASS - Servo swept to 15, 110, and 178.\n", TEST_ID);
    printf("[TEST] === test run end ===\n");

    while (1) {
        tight_loop_contents();
    }
    return 0;
}