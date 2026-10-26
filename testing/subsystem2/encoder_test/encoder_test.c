#include "pico/stdlib.h"
#include <stdio.h>

// /* Right wheel encoder */
// #define RIGHT_ENC_A 0U
// #define RIGHT_ENC_B 1U

// /* Left wheel encoder */
// #define LEFT_ENC_A 4U
// #define LEFT_ENC_B 5U

/* Right physical wheel encoder */
#define RIGHT_ENC_A 4U
#define RIGHT_ENC_B 5U

/* Left physical wheel encoder */
#define LEFT_ENC_A 0U
#define LEFT_ENC_B 1U

/* Signed counts: positive/negative indicate opposite directions */
static volatile int32_t right_count = 0;
static volatile int32_t left_count = 0;

/*
 * Interrupt callback:
 * Called when encoder channel A changes.
 * Read channel B to determine rotation direction.
 */
static void encoder_callback(uint gpio, uint32_t events)
{
    (void)events;

    if (gpio == RIGHT_ENC_A)
    {
        bool a = gpio_get(RIGHT_ENC_A);
        bool b = gpio_get(RIGHT_ENC_B);

        right_count += (a == b) ? 1 : -1;
    }
    else if (gpio == LEFT_ENC_A)
    {
        bool a = gpio_get(LEFT_ENC_A);
        bool b = gpio_get(LEFT_ENC_B);

        /* Forward rotation = positive count */
        left_count += (a == b) ? -1 : 1;
    }
}

int main(void)
{
    stdio_init_all();
    sleep_ms(2000U);

    /* Configure both encoder channels as inputs */
    const uint encoder_pins[] = {
        RIGHT_ENC_A,
        RIGHT_ENC_B,
        LEFT_ENC_A,
        LEFT_ENC_B
    };

    for (uint i = 0U; i < 4U; i++)
    {
        gpio_init(encoder_pins[i]);
        gpio_set_dir(encoder_pins[i], GPIO_IN);
        gpio_pull_up(encoder_pins[i]);
    }

    /* Enable interrupts on both edges of channel A */
    gpio_set_irq_enabled_with_callback(
        RIGHT_ENC_A,
        GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL,
        true,
        &encoder_callback
    );

    gpio_set_irq_enabled(
        LEFT_ENC_A,
        GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL,
        true
    );

    printf("Encoder pulse test started\n");
    printf("Rotate each wheel manually.\n");

    while (true)
    {
        printf(
            "RIGHT COUNT = %ld | LEFT COUNT = %ld\n",
            (long)right_count,
            (long)left_count
        );

        sleep_ms(500U);
    }
}