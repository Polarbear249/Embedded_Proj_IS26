
#include <stdio.h>          // Allows us to use printf()
#include "pico/stdlib.h"    // Provides Pico GPIO and timing functions

// GPIO pins connected to the RIGHT motor encoder
#define ENC_R_A 0U          // Right encoder signal A uses GP0
#define ENC_R_B 1U          // Right encoder signal B uses GP1

// GPIO pins connected to the LEFT motor encoder
#define ENC_L_A 4U          // Left encoder signal A uses GP4
#define ENC_L_B 5U          // Left encoder signal B uses GP5

int main(void)
{
    // Initialise USB serial communication for printing results
    stdio_init_all();

    // Store all four encoder GPIO pins in an array
    const uint encoder_pins[] = {
        ENC_R_A,
        ENC_R_B,
        ENC_L_A,
        ENC_L_B
    };

    // Configure all four encoder pins as inputs
    for (uint i = 0U; i < 4U; i++)
    {
        gpio_init(encoder_pins[i]);             // Initialise GPIO pin
        gpio_set_dir(encoder_pins[i], GPIO_IN); // Set pin as input
    }

    // Wait 2 seconds for USB serial communication to start
    sleep_ms(2000U);

    // Print a message to show the test has started
    printf("Encoder GPIO Test Started\n");

    // Keep checking encoder signals continuously
    while (true)
    {
        // Read and print the A and B signals from both encoders
        // 0 = LOW signal, 1 = HIGH signal
        printf("RIGHT A=%d B=%d | LEFT A=%d B=%d\n",
               gpio_get(ENC_R_A),  // Read right encoder A
               gpio_get(ENC_R_B),  // Read right encoder B
               gpio_get(ENC_L_A),  // Read left encoder A
               gpio_get(ENC_L_B)); // Read left encoder B

        // Wait 100 ms before checking again
        sleep_ms(100U);
    }
}
