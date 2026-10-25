#include <stdio.h>
#include <stdint.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/timer.h"

// Keeping the same Grove pins used for test 1
#define TRIG_PIN 6   
#define ECHO_PIN 7   
#define TEST_ID_2 "S5-U02"
#define TEST_ID_3 "S5-U03"

// Function to trigger the sensor and get the distance
float measure_distance() {
    // Send 10us trigger pulse
    gpio_put(TRIG_PIN, 1);
    sleep_us(10);
    gpio_put(TRIG_PIN, 0);

    // Wait for ECHO to go HIGH (with timeout)
    uint64_t start_wait = time_us_64();
    while (gpio_get(ECHO_PIN) == 0) {
        if (time_us_64() - start_wait > 5000) return -1.0; // 5ms timeout
    }

    // Measure how long ECHO stays HIGH
    uint64_t start_echo = time_us_64();
    while (gpio_get(ECHO_PIN) == 1) {
        if (time_us_64() - start_echo > 30000) return -1.0; // 30ms timeout
    }
    uint64_t pulse_width = time_us_64() - start_echo;

    // Calculate distance (cm)
    float distance_cm = (float)pulse_width / 58.0;
    
    // Print the raw pulse width (for U02) and the calculated distance (for U03)
    printf("[TEST] Raw Pulse: %llu us  |  Distance: %.1f cm\n", pulse_width, distance_cm);
    
    return distance_cm;
}

int main() {
    stdio_init_all();

    // Wait for serial monitor
    while (!stdio_usb_connected()) {
        sleep_ms(100);
    }
    sleep_ms(1000); 
    
    printf("\n[TEST] ===== Starting Subsystem 5: %s & %s =====\n", TEST_ID_2, TEST_ID_3);

    // Configure Pins
    gpio_init(TRIG_PIN);
    gpio_set_dir(TRIG_PIN, GPIO_OUT);
    gpio_put(TRIG_PIN, 0);

    gpio_init(ECHO_PIN);
    gpio_set_dir(ECHO_PIN, GPIO_IN);
    gpio_pull_down(ECHO_PIN);

    printf("[TEST] Pins configured. Reading distance 5 times...\n");
    printf("[TEST] Place an object at 10cm, 20cm, and 30cm.\n\n");

    int valid_reads = 0;

    // Take 5 readings to buy time to move the object
    for (int i = 0; i < 5; i++) {
        float dist = measure_distance();
        
        // Check if reading is valid (between 2cm and 400cm)
        if (dist >= 2.0 && dist <= 400.0) {
            valid_reads++;
        }
        sleep_ms(8000); // Wait 8 seconds between readings
    }

    printf("\n[TEST] Loop finished.\n");

    // Evaluate S5-U02
    if (valid_reads > 0) {
        printf("[TEST] %s PASS - Raw pulse width values captured successfully.\n", TEST_ID_2);
    } else {
        printf("[TEST] %s FAIL - Could not capture valid pulse widths.\n", TEST_ID_2);
    }

    // Evaluate S5-U03
    if (valid_reads > 0) {
        printf("[TEST] %s PASS - Distance formula executed. (Tester must confirm prints match physical ruler).\n", TEST_ID_3);
    } else {
        printf("[TEST] %s FAIL - Distance calculation returned no valid data.\n", TEST_ID_3);
    }

    printf("[TEST] === test run end ===\n");

    while (1) {
        tight_loop_contents();
    }
    return 0;
}