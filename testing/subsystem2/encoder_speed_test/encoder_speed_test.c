
#include "pico/stdlib.h"
#include "hardware/sync.h"
#include <stdint.h>
#include <stdio.h>

/* Encoder GPIO pins — verified physical wheel mapping */
#define LEFT_ENC_A   0U
#define LEFT_ENC_B   1U
#define RIGHT_ENC_A  4U
#define RIGHT_ENC_B  5U

/* Preliminary counts per wheel revolution */
#define LEFT_CPR     1271.0f
#define RIGHT_CPR    1266.0f

/* Measure RPM every 1000 milliseconds */
#define SAMPLE_TIME_MS 1000U

/* Updated by GPIO interrupts */
static volatile int32_t left_count = 0;
static volatile int32_t right_count = 0;

/*
 * Interrupt callback:
 * Called when channel A rises or falls.
 * Channel B determines rotation direction.
 */
static void encoder_callback(uint gpio, uint32_t events)
{
    (void)events;

    if (gpio == LEFT_ENC_A)
    {
        bool a = gpio_get(LEFT_ENC_A);
        bool b = gpio_get(LEFT_ENC_B);

        /* Forward = positive for the physical left wheel */
        left_count += (a == b) ? -1 : 1;
    }
    else if (gpio == RIGHT_ENC_A)
    {
        bool a = gpio_get(RIGHT_ENC_A);
        bool b = gpio_get(RIGHT_ENC_B);

        /* Forward = positive for the physical right wheel */
        right_count += (a == b) ? 1 : -1;
    }
}

int main(void)
{
    stdio_init_all();
    sleep_ms(2000U);

    const uint encoder_pins[] = {
        LEFT_ENC_A,
        LEFT_ENC_B,
        RIGHT_ENC_A,
        RIGHT_ENC_B
    };

    /* Configure encoder inputs */
    for (uint i = 0U; i < 4U; i++)
    {
        gpio_init(encoder_pins[i]);
        gpio_set_dir(encoder_pins[i], GPIO_IN);
        gpio_pull_up(encoder_pins[i]);
    }

    /* Register the single shared GPIO interrupt callback */
    gpio_set_irq_enabled_with_callback(
        LEFT_ENC_A,
        GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL,
        true,
        &encoder_callback
    );

    gpio_set_irq_enabled(
        RIGHT_ENC_A,
        GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL,
        true
    );

    int32_t previous_left = 0;
    int32_t previous_right = 0;

    uint32_t previous_time = to_ms_since_boot(get_absolute_time());

    printf("Encoder RPM test started!\n");
    printf("Rotate the wheels manually.\n");

    while (true)
    {
        sleep_ms(SAMPLE_TIME_MS);

        uint32_t current_time = to_ms_since_boot(get_absolute_time());

        /* Snapshot both counters without ISR interference */
        uint32_t irq_state = save_and_disable_interrupts();
        int32_t current_left = left_count;
        int32_t current_right = right_count;
        restore_interrupts(irq_state);

        uint32_t elapsed_ms = current_time - previous_time;

        int32_t delta_left = current_left - previous_left;
        int32_t delta_right = current_right - previous_right;

        /* RPM = (counts / CPR) * (60000 / elapsed_ms) */
        float left_rpm =
            ((float)delta_left * 60000.0f) /
            (LEFT_CPR * (float)elapsed_ms);

        float right_rpm =
            ((float)delta_right * 60000.0f) /
            (RIGHT_CPR * (float)elapsed_ms);

        printf(
            "LEFT RPM = %.2f | RIGHT RPM = %.2f\n",
            (double)left_rpm,
            (double)right_rpm
        );

        previous_left = current_left;
        previous_right = current_right;
        previous_time = current_time;
    }
}
