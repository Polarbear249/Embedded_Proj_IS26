
#include "pico/stdlib.h"
#include <stdint.h>
#include <stdio.h>

#define RIGHT_ENC_A 0U
#define RIGHT_ENC_B 1U

/* Raw transition counts */
static volatile uint32_t a_edges = 0;
static volatile uint32_t b_edges = 0;

/* Signed count using channel A and B */
static volatile int32_t signed_count = 0;

/* Interrupt callback */
static void encoder_callback(uint gpio, uint32_t events)
{
    if (gpio == RIGHT_ENC_A)
    {
        /* Count all A-channel transitions */
        a_edges++;

        /* Determine direction from A and B */
        bool a = (events & GPIO_IRQ_EDGE_RISE) != 0U;
        bool b = gpio_get(RIGHT_ENC_B);

        signed_count += (a == b) ? 1 : -1;
    }
    else if (gpio == RIGHT_ENC_B)
    {
        /* Count all B-channel transitions */
        b_edges++;
    }
}

int main(void)
{
    stdio_init_all();
    sleep_ms(2000U);

    /* Configure encoder pins */
    gpio_init(RIGHT_ENC_A);
    gpio_set_dir(RIGHT_ENC_A, GPIO_IN);
    gpio_pull_up(RIGHT_ENC_A);

    gpio_init(RIGHT_ENC_B);
    gpio_set_dir(RIGHT_ENC_B, GPIO_IN);
    gpio_pull_up(RIGHT_ENC_B);

    /* Enable both-edge interrupts */
    gpio_set_irq_enabled_with_callback(
        RIGHT_ENC_A,
        GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL,
        true,
        &encoder_callback
    );

    gpio_set_irq_enabled(
        RIGHT_ENC_B,
        GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL,
        true
    );

    printf("Right encoder direction diagnostic started\n");

    while (true)
    {
        printf(
            "A EDGES = %lu | B EDGES = %lu | SIGNED = %ld\n",
            (unsigned long)a_edges,
            (unsigned long)b_edges,
            (long)signed_count
        );

        sleep_ms(500U);
    }
}
