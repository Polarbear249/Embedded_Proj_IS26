
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include <stdio.h>

/* Robo Pico motor driver GPIO pins */
#define MOTOR1_A 8U
#define MOTOR1_B 9U
#define MOTOR2_A 10U
#define MOTOR2_B 11U

/* Test one motor-driver input at a time */
static void motor_test(uint pin)
{
    printf("Testing GP%u\n", pin);

    /* Apply approximately 50% PWM duty cycle */
    pwm_set_gpio_level(pin, 500U);
    sleep_ms(700U);

    /* Remove PWM drive from this input */
    pwm_set_gpio_level(pin, 0U);
    sleep_ms(700U);
}

int main(void)
{
    const uint pins[] = {
        MOTOR1_A,
        MOTOR1_B,
        MOTOR2_A,
        MOTOR2_B
    };

    /* Initialise USB serial output */
    stdio_init_all();
    sleep_ms(2000U);

    /* Configure GPIO pins for PWM */
    for (uint i = 0U; i < 4U; i++)
    {
        gpio_set_function(pins[i], GPIO_FUNC_PWM);

        uint slice = pwm_gpio_to_slice_num(pins[i]);

        /* Approximately 1 kHz PWM at 125 MHz */
        pwm_set_clkdiv(slice, 125.0f);
        pwm_set_wrap(slice, 999U);

        /* Start with motor input inactive */
        pwm_set_gpio_level(pins[i], 0U);
        pwm_set_enabled(slice, true);
    }

    /* Test each motor-driver input once */
for (uint i = 0U; i < 4U; i++)
{
    motor_test(pins[i]);
}

/* Stop all motor-driver inputs */
for (uint i = 0U; i < 4U; i++)
{
    pwm_set_gpio_level(pins[i], 0U);
}

printf("Motor test finished. Motors stopped.\n");

/* Stay idle without repeating the test */
while (true)
{
    sleep_ms(1000U);
}
}
