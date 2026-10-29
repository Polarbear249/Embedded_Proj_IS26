
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/sync.h"
#include <stdint.h>
#include <stdio.h>

/* Right motor: GP8 / GP9 */
#define RIGHT_MOTOR_A 8U
#define RIGHT_MOTOR_B 9U

/* Right encoder: Grove 1 */
#define RIGHT_ENC_A 0U
#define RIGHT_ENC_B 1U

#define RIGHT_CPR 1266.0f
#define PWM_WRAP 999U

static volatile int32_t right_count = 0;

static void encoder_callback(uint gpio, uint32_t events)
{
    (void)events;

    if (gpio == RIGHT_ENC_A)
    {
        bool a = gpio_get(RIGHT_ENC_A);
        bool b = gpio_get(RIGHT_ENC_B);

        right_count += (a == b) ? 1 : -1;
    }
}

static void motor_init(void)
{
    gpio_set_function(RIGHT_MOTOR_A, GPIO_FUNC_PWM);
    gpio_set_function(RIGHT_MOTOR_B, GPIO_FUNC_PWM);

    uint slice = pwm_gpio_to_slice_num(RIGHT_MOTOR_A);

    pwm_set_clkdiv(slice, 125.0f);
    pwm_set_wrap(slice, PWM_WRAP);

    pwm_set_gpio_level(RIGHT_MOTOR_A, 0U);
    pwm_set_gpio_level(RIGHT_MOTOR_B, 0U);

    pwm_set_enabled(slice, true);
}

static void motor_stop(void)
{
    pwm_set_gpio_level(RIGHT_MOTOR_A, 0U);
    pwm_set_gpio_level(RIGHT_MOTOR_B, 0U);
}

static void motor_test_forward(uint16_t level)
{
    if (level > PWM_WRAP)
    {
        level = PWM_WRAP;
    }

    /* Reverse the previously tested GP8 direction */
    pwm_set_gpio_level(RIGHT_MOTOR_A, 0U);
    pwm_set_gpio_level(RIGHT_MOTOR_B, level);
}

static void encoder_init(void)
{
    gpio_init(RIGHT_ENC_A);
    gpio_set_dir(RIGHT_ENC_A, GPIO_IN);
    gpio_pull_up(RIGHT_ENC_A);

    gpio_init(RIGHT_ENC_B);
    gpio_set_dir(RIGHT_ENC_B, GPIO_IN);
    gpio_pull_up(RIGHT_ENC_B);

    gpio_set_irq_enabled_with_callback(
        RIGHT_ENC_A,
        GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL,
        true,
        &encoder_callback
    );
}

int main(void)
{
    stdio_init_all();
    sleep_ms(2000U);

    motor_init();
    encoder_init();
    motor_stop();

    printf("\nRIGHT MOTOR DIRECTION TEST\n");
    printf("Motor: GP8/GP9 | Encoder: GP0/GP1\n");

    sleep_ms(500U);

    uint32_t irq_state = save_and_disable_interrupts();
    right_count = 0;
    restore_interrupts(irq_state);

    int32_t previous_count = 0;
    absolute_time_t previous_time = get_absolute_time();

    /* Low-power direction test */
    motor_test_forward(150U);

    for (uint i = 0U; i < 10U; i++)
    {
        sleep_ms(100U);

        absolute_time_t now = get_absolute_time();

        irq_state = save_and_disable_interrupts();
        int32_t count = right_count;
        restore_interrupts(irq_state);

        int64_t elapsed_us =
            absolute_time_diff_us(previous_time, now);

        float rpm = 0.0f;

        if (elapsed_us > 0)
        {
            rpm = ((float)(count - previous_count) * 60000000.0f) /
                  (RIGHT_CPR * (float)elapsed_us);
        }

        printf(
            "Count = %ld | RPM = %.2f\n",
            (long)count,
            (double)rpm
        );

        previous_count = count;
        previous_time = now;
    }

    motor_stop();

    printf("TEST COMPLETE - Motor stopped.\n");

    while (true)
    {
        sleep_ms(1000U);
    }
}
