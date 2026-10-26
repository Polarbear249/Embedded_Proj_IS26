#include <stdio.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "hardware/timer.h"

// Keep ultrasonic pins on GP6/GP7
#define TRIG_PIN 6   
#define ECHO_PIN 7   
#define SERVO_PIN 12 // Changed to GP12 for standard PWM Servo

#define TEST_ID "S5-U04"

// Servo PWM settings
#define PWM_FREQ 50       // 50 Hz for standard servos
#define PWM_WRAP 20000    // 20ms period (1,000,000 us / 50 Hz)
#define SERVO_MIN_US 1000 // 1.0ms pulse = 0 degrees
#define SERVO_MAX_US 2000 // 2.0ms pulse = 180 degrees
#define SERVO_CENTER_US 1500 // 1.5ms pulse = 90 degrees

uint slice_num;

// Function to set the servo angle (0 to 180)
void set_servo_angle(uint8_t angle) {
    if (angle > 180) angle = 180;
    uint32_t pulse_us = SERVO_MIN_US + ((SERVO_MAX_US - SERVO_MIN_US) * angle / 180);
    pwm_set_chan_level(slice_num, PWM_CHAN_A, pulse_us);
    printf("[SERVO] Commanded angle: %d degrees\n", angle);
}

// REAL Ultrasonic reading function
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
    
    printf("\n[TEST] ===== Starting Subsystem 5: %s (Servo Pan) =====\n", TEST_ID);

    // Initialize Servo PWM
    gpio_set_function(SERVO_PIN, GPIO_FUNC_PWM);
    slice_num = pwm_gpio_to_slice_num(SERVO_PIN);
    
    pwm_config config = pwm_get_default_config();
    pwm_config_set_clkdiv(&config, 50.f); 
    pwm_config_set_wrap(&config, PWM_WRAP);
    pwm_init(slice_num, &config, true);

    // Initialize Ultrasonic pins
    gpio_init(TRIG_PIN); gpio_set_dir(TRIG_PIN, GPIO_OUT);
    gpio_init(ECHO_PIN); gpio_set_dir(ECHO_PIN, GPIO_IN);
    gpio_pull_down(ECHO_PIN);

    printf("[TEST] Servo initialized on GP%d\n", SERVO_PIN);
    printf("[TEST] Sweeping to 30, 90, and 150 degrees...\n\n");

    // The COARSE_SCAN simulation
    uint8_t scan_angles[] = {30, 90, 150};
    
    for (int i = 0; i < 3; i++) {
        uint8_t angle = scan_angles[i];
        
        // Command the servo
        set_servo_angle(angle);
        
        // WAIT FOR SETTLING
        printf("[SERVO] Waiting 500ms for settling...\n");
        sleep_ms(500); 
        
        // Take a REAL reading
        float dist = measure_distance();
        if (dist > 0) {
            printf("[TEST] Real Distance at %d deg: %.1f cm\n\n", angle, dist);
        } else {
            printf("[TEST] No echo at %d deg (out of range)\n\n", angle);
        }
    }

    // Return to center
    printf("[SERVO] Returning to center (90 degrees).\n");
    set_servo_angle(90);
    sleep_ms(500);

    printf("[TEST] %s PASS - Servo swept to 30, 90, and 150. Distances acquired after settling.\n", TEST_ID);
    printf("[TEST] === test run end ===\n");

    while (1) {
        tight_loop_contents();
    }
    return 0;
}