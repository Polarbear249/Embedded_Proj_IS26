#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/timer.h"

// CHANGED FOR HARDWARE TESTING ON THE GROVE PORT
#define TRIG_PIN 6   
#define ECHO_PIN 7   
#define TEST_ID "S5-U01"

int main() {
    // Initialize stdio
    stdio_init_all();

    // Wait for the USB Serial Monitor to connect
    while (!stdio_usb_connected()) {
        sleep_ms(100);
    }

    sleep_ms(1000); 
    printf("\n[TEST] ===== Starting Subsystem 5: %s =====\n", TEST_ID);

    // Configure Pins
    gpio_init(TRIG_PIN);
    gpio_set_dir(TRIG_PIN, GPIO_OUT);
    gpio_put(TRIG_PIN, 0);

    gpio_init(ECHO_PIN);
    gpio_set_dir(ECHO_PIN, GPIO_IN);
    
    // Force the pin low when nothing is connected to prevent floating noise
    gpio_pull_down(ECHO_PIN);

    printf("[TEST] Pins configured (TRIG: GP6, ECHO: GP7). Firing 100 pings...\n");
    int successful_reads = 0;

    // Fire pings
    for (int i = 0; i < 100; i++) {
        // Send 10us trigger pulse
        gpio_put(TRIG_PIN, 1);
        sleep_us(10);
        gpio_put(TRIG_PIN, 0);

        // Wait for ECHO to go HIGH
        uint64_t start_wait = time_us_64();
        while (gpio_get(ECHO_PIN) == 0) {
            if (time_us_64() - start_wait > 5000) break; // 5ms timeout
        }

        // If ECHO went high, measure how long it stays high
        if (gpio_get(ECHO_PIN) == 1) {
            successful_reads++;
            
            uint64_t start_echo = time_us_64();
            while (gpio_get(ECHO_PIN) == 1) {
                if (time_us_64() - start_echo > 30000) break; // 30ms timeout
            }
        }
        
        // Wait 60ms before next ping to avoid echo overlap
        sleep_ms(60); 
    }

    // Print Results
    printf("[TEST] Loop finished. Successful ECHO reads: %d / 100\n", successful_reads);

    if (successful_reads > 90) { 
        printf("[TEST] %s PASS - 100 pings fired without hardware damage. ECHO pin handled 3.3V logic.\n", TEST_ID);
    } else {
        printf("[TEST] %s FAIL - ECHO pin did not register sufficient pulses.\n", TEST_ID);
    }

    printf("[TEST] === test run end ===\n");

    // Stay alive
    while (1) {
        tight_loop_contents();
    }
    return 0;
}