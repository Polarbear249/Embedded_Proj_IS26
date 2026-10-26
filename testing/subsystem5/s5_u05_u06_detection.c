#include <stdio.h>
#include <stdint.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/timer.h"

// Keeping the same Grove pins used for test 1
#define TRIG_PIN 6   
#define ECHO_PIN 7   
#define TEST_ID_5 "S5-U05"
#define TEST_ID_6 "S5-U06"

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
    
    // Print the raw pulse width and the calculated distance
    printf("Raw Pulse: %llu us  |  Distance: %.1f cm\n", (unsigned long long)pulse_width, distance_cm);
    
    return distance_cm;
}

int main() {
    stdio_init_all();
    while (!stdio_usb_connected()) { sleep_ms(100); }
    sleep_ms(1000); 
    
    printf("\n[TEST] ===== Starting Subsystem 5: S5-U05, U06, U07 =====\n");

    gpio_init(TRIG_PIN);
    gpio_set_dir(TRIG_PIN, GPIO_OUT);
    gpio_put(TRIG_PIN, 0);
    gpio_init(ECHO_PIN);
    gpio_set_dir(ECHO_PIN, GPIO_IN);
    gpio_pull_down(ECHO_PIN);

    printf("[TEST] S5-U06: Waiting for obstacle < 15cm...\n");
    
    bool obstacle_detected = false;
    while (!obstacle_detected) {
        float dist = measure_distance(); // Troubleshoot JIC
        if (dist > 0 && dist < 15.0) {
            obstacle_detected = true;
            printf("[TEST] S5-U06 PASS - Obstacle detected at %.1f cm. State -> STOP_AND_PROFILE.\n", dist); // See if it managed to send the simulated "stop message"
        }
        sleep_ms(200);
    }

    // === S5-U05 BRAKE HANDSHAKE ===
    printf("\n[TEST] S5-U05: STOP_REQUEST sent to Q_motion. Waiting for BRAKE_ACK...\n");
    printf("[TEST] Input 'B' to simulate BRAKE_ACK.\n");
    
    char c;
    while (true) {
        c = getchar(); // Wait for keyboard
        if (c == 'B' || c == 'b') {
            printf("[TEST] S5-U05 PASS - BRAKE_ACK received. Proceeding to scan (simulated).\n");
            break; // Exit the loop and move to the next test
        } else if (c != '\n' && c != '\r') {
            printf("[TEST] Invalid input. Please type 'B'.\n");
        }
    }

    // === S5-U07 IDLE RECOVERY ===
    printf("\n[TEST] S5-U07: Scan complete. State -> REACQUIRE_LINE_AFTER_BYPASS.\n");
    printf("[TEST] Input 'L' to simulate line reacquired.\n");

    while (true) {
        c = getchar(); // Wait for keyboard
        if (c == 'L' || c == 'l') {
            printf("[TEST] S5-U07 PASS - Line reacquired. State -> SCAN_IDLE.\n");
            break;
        } else if (c != '\n' && c != '\r') {
            printf("[TEST] Invalid input. Please type 'L'.\n");
        }
    }

    printf("\n[TEST] === test run end ===\n");

    while (1) { tight_loop_contents(); }
    return 0;
}