
#include "pico/stdlib.h"
#include "hardware/sync.h"
#include <stdint.h>
#include <stdio.h>

/* Official IS26 encoder pin mapping */

/* Physical RIGHT wheel - Grove 1 */
#define RIGHT_ENC_A  0U
#define RIGHT_ENC_B  1U

/* Physical LEFT wheel - Grove 3 */
#define LEFT_ENC_A   4U
#define LEFT_ENC_B   5U

/* Preliminary encoder counts per wheel revolution */
#define LEFT_CPR     1271.0f
#define RIGHT_CPR    1266.0f

/* Calculate RPM every 1000 milliseconds */
#define SAMPLE_TIME_MS 1000U

/* Encoder counts updated by GPIO interrupts */
static volatile int32_t left_count = 0;
static volatile int32_t right_count = 0;

/* Diagnostic: total right encoder A-channel edges */
static volatile uint32_t right_raw_edges = 0;

/*
 * GPIO interrupt callback.
 * Channel A generates interrupts.
 * Channel B is used to determine direction.
 */
static void encoder_callback(uint gpio, uint32_t events)
{
    if (gpio == LEFT_ENC_A)
    {
        /* Read left encoder B and identify A's triggering edge */
        bool a = (events & GPIO_IRQ_EDGE_RISE) != 0U;
        bool b = gpio_get(LEFT_ENC_B);

        /* Preliminary left wheel direction convention */
        left_count += (a == b) ? -1 : 1;
    }
    else if (gpio == RIGHT_ENC_A)
    {
        /* Count each right encoder A-channel transition */
        right_raw_edges++;

        /* Determine direction using channel A event and channel B */
        bool a = (events & GPIO_IRQ_EDGE_RISE) != 0U;
        bool b = gpio_get(RIGHT_ENC_B);

        /* Preliminary right wheel direction convention */
        right_count += (a == b) ? 1 : -1;
    }
}

int main(void)
{
    /* Initialise USB serial output */
    stdio_init_all();
    sleep_ms(2000U);

    /* Encoder GPIO pins */
    const uint encoder_pins[] = {
        RIGHT_ENC_A,
        RIGHT_ENC_B,
        LEFT_ENC_A,
        LEFT_ENC_B
    };

    /* Configure all encoder inputs with pull-up resistors */
    for (uint i = 0U; i < 4U; i++)
    {
        gpio_init(encoder_pins[i]);
        gpio_set_dir(encoder_pins[i], GPIO_IN);
        gpio_pull_up(encoder_pins[i]);
    }

    /* Register shared callback on left encoder A */
    gpio_set_irq_enabled_with_callback(
        LEFT_ENC_A,
        GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL,
        true,
        &encoder_callback
    );

    /* Enable interrupts on right encoder A */
    gpio_set_irq_enabled(
        RIGHT_ENC_A,
        GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL,
        true
    );

    /* Previous values for RPM calculations */
    int32_t previous_left = 0;
    int32_t previous_right = 0;

    uint32_t previous_time =
        to_ms_since_boot(get_absolute_time());

    /* Firmware identification */
    printf("RPM DEBUG VERSION 4 - IS26 OFFICIAL PIN MAPPING\n");
    printf("RIGHT: GP0/GP1 | LEFT: GP4/GP5\n");
    printf("Rotate each wheel manually.\n");

    while (true)
    {
        /* Wait for next speed measurement */
        sleep_ms(SAMPLE_TIME_MS);

        uint32_t current_time =
            to_ms_since_boot(get_absolute_time());

        /* Safely copy encoder counts */
        uint32_t irq_state = save_and_disable_interrupts();

        int32_t current_left = left_count;
        int32_t current_right = right_count;
        uint32_t current_right_edges = right_raw_edges;

        restore_interrupts(irq_state);

        /* Time elapsed since previous reading */
        uint32_t elapsed_ms = current_time - previous_time;

        /* Encoder count changes during this interval */
        int32_t delta_left = current_left - previous_left;
        int32_t delta_right = current_right - previous_right;

        /* Default to zero speed */
        float left_rpm = 0.0f;
        float right_rpm = 0.0f;

        /* Calculate RPM only if time has elapsed */
        if (elapsed_ms > 0U)
        {
            left_rpm =
                ((float)delta_left * 60000.0f) /
                (LEFT_CPR * (float)elapsed_ms);

            right_rpm =
                ((float)delta_right * 60000.0f) /
                (RIGHT_CPR * (float)elapsed_ms);
        }

        /* Identify the current firmware */
        printf("Firmware V4 active\n");

        /* Display right encoder GPIO input levels */
        printf(
            "RIGHT A (GP0) = %d | RIGHT B (GP1) = %d\n",
            gpio_get(RIGHT_ENC_A),
            gpio_get(RIGHT_ENC_B)
        );

        /* Display counts and calculated RPM */
        printf(
            "LEFT COUNT = %ld | RIGHT COUNT = %ld | "
            "RIGHT EDGES = %lu | LEFT RPM = %.2f | RIGHT RPM = %.2f\n",
            (long)current_left,
            (long)current_right,
            (unsigned long)current_right_edges,
            (double)left_rpm,
            (double)right_rpm
        );

        /* Save readings for the next measurement */
        previous_left = current_left;
        previous_right = current_right;
        previous_time = current_time;
    }
} 
