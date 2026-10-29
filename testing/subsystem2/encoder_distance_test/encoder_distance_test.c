
#include "pico/stdlib.h"
#include "hardware/sync.h"
#include <stdint.h>
#include <stdio.h>

/* Encoder pins based on verified physical wheel mapping */
#define LEFT_ENC_A    0U
#define LEFT_ENC_B    1U
#define RIGHT_ENC_A   4U
#define RIGHT_ENC_B   5U

/* Preliminary encoder calibration values */
#define LEFT_CPR      1271.0f
#define RIGHT_CPR     1266.0f

/* Estimated wheel diameter in centimetres */
#define WHEEL_DIAMETER_CM  6.6f

/* Pi for circumference calculation */
#define PI_VALUE 3.14159265f

/* Signed encoder counters updated by interrupts */
static volatile int32_t left_count = 0;
static volatile int32_t right_count = 0;

/* GPIO interrupt callback */
static void encoder_callback(uint gpio, uint32_t events)
{
    (void)events;

    if (gpio == LEFT_ENC_A)
    {
        bool a = gpio_get(LEFT_ENC_A);
        bool b = gpio_get(LEFT_ENC_B);

        /* Forward rotation gives positive counts */
        left_count += (a == b) ? -1 : 1;
    }
    else if (gpio == RIGHT_ENC_A)
    {
        bool a = gpio_get(RIGHT_ENC_A);
        bool b = gpio_get(RIGHT_ENC_B);

        /* Forward rotation gives positive counts */
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

    /* Set up GPIO inputs and pull-up resistors */
    for (uint i = 0U; i < 4U; i++)
    {
        gpio_init(encoder_pins[i]);
        gpio_set_dir(encoder_pins[i], GPIO_IN);
        gpio_pull_up(encoder_pins[i]);
    }

    /* Register shared GPIO interrupt callback */
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

    /* Calculate wheel circumference */
    const float circumference_cm =
        PI_VALUE * WHEEL_DIAMETER_CM;

    printf("Encoder distance test started!\n");
    printf("Wheel diameter: %.2f cm\n",
           (double)WHEEL_DIAMETER_CM);
    printf("Rotate each wheel manually.\n");

    while (true)
    {
        /* Take a consistent snapshot of encoder counters */
        uint32_t irq_state = save_and_disable_interrupts();

        int32_t current_left = left_count;
        int32_t current_right = right_count;

        restore_interrupts(irq_state);

        /* Convert encoder counts to wheel travel distance */
        float left_distance_cm =
            ((float)current_left / LEFT_CPR) *
            circumference_cm;

        float right_distance_cm =
            ((float)current_right / RIGHT_CPR) *
            circumference_cm;

        printf(
            "LEFT = %.2f cm | RIGHT = %.2f cm\n",
            (double)left_distance_cm,
            (double)right_distance_cm
        );

        sleep_ms(500U);
    }
}
