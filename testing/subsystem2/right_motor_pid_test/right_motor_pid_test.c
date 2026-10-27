
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/sync.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

/* Buddy 2 - Right-wheel PI speed-control bench test */

/* Verified motor and encoder connections */
#define RIGHT_MOTOR_A          8U
#define RIGHT_MOTOR_B          9U

#define RIGHT_ENC_A            0U
#define RIGHT_ENC_B            1U

/* Preliminary encoder calibration */
#define RIGHT_CPR              1266.0f

/* PWM configuration */
#define PWM_WRAP               999U
#define PWM_CLKDIV             125.0f

/* Initial PI settings - right wheel requires tuning */
#define TARGET_RPM             10.0f
#define KP                     2.0f
#define KI                     2.0f

#define CONTROL_TIME_MS        100U
#define TEST_ITERATIONS        30U

#define BASELINE_PWM           85.0f
#define MAX_PWM_LEVEL          200.0f

/* Startup boost */
#define STARTUP_PWM            150U
#define STARTUP_TIME_MS        150U

/* Stop after five consecutive intervals without pulses */
#define MAX_NO_PULSE_CYCLES    5U

static volatile int32_t right_count = 0;

/* Right encoder interrupt */
static void encoder_callback(uint gpio, uint32_t events)
{
    (void)events;

    if (gpio == RIGHT_ENC_A)
    {
        bool a = gpio_get(RIGHT_ENC_A);
        bool b = gpio_get(RIGHT_ENC_B);

        /* Verified right-wheel forward count convention */
        right_count += (a == b) ? 1 : -1;
    }
}

/* Initialise right motor PWM */
static void motor_init(void)
{
    gpio_set_function(RIGHT_MOTOR_A, GPIO_FUNC_PWM);
    gpio_set_function(RIGHT_MOTOR_B, GPIO_FUNC_PWM);

    uint slice = pwm_gpio_to_slice_num(RIGHT_MOTOR_A);

    pwm_set_clkdiv(slice, PWM_CLKDIV);
    pwm_set_wrap(slice, PWM_WRAP);

    pwm_set_gpio_level(RIGHT_MOTOR_A, 0U);
    pwm_set_gpio_level(RIGHT_MOTOR_B, 0U);

    pwm_set_enabled(slice, true);
}

/* Command the verified forward motor direction */
static void motor_set_forward(uint16_t level)
{
    if (level > PWM_WRAP)
    {
        level = PWM_WRAP;
    }

    /* GP9 active; GP8 inactive */
    pwm_set_gpio_level(RIGHT_MOTOR_A, 0U);
    pwm_set_gpio_level(RIGHT_MOTOR_B, level);
}

static void motor_stop(void)
{
    pwm_set_gpio_level(RIGHT_MOTOR_A, 0U);
    pwm_set_gpio_level(RIGHT_MOTOR_B, 0U);
}

/* Initialise right encoder */
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

    printf("\nBUDDY 2 - RIGHT MOTOR PI CONTROL TEST\n");
    printf("Motor GP8/GP9 | Encoder GP0/GP1\n");
    printf("Target: %.1f RPM\n", (double)TARGET_RPM);
    printf("Kp: %.1f | Ki: %.1f\n", (double)KP, (double)KI);

    sleep_ms(500U);

    uint32_t irq_state = save_and_disable_interrupts();
    right_count = 0;
    restore_interrupts(irq_state);

    int32_t previous_count = 0;
    float integral_error = 0.0f;
    uint32_t no_pulse_intervals = 0U;

    absolute_time_t previous_time = get_absolute_time();

    /* Brief startup boost */
    printf("Applying startup boost...\n");
    motor_set_forward(STARTUP_PWM);
    sleep_ms(STARTUP_TIME_MS);

    /* Reset measurements after startup boost */
    irq_state = save_and_disable_interrupts();
    right_count = 0;
    restore_interrupts(irq_state);

    previous_count = 0;
    previous_time = get_absolute_time();

    /* Begin closed-loop control */
    motor_set_forward((uint16_t)BASELINE_PWM);
    printf("Starting PI control...\n");

    for (uint i = 0U; i < TEST_ITERATIONS; i++)
    {
        sleep_ms(CONTROL_TIME_MS);

        absolute_time_t now = get_absolute_time();

        irq_state = save_and_disable_interrupts();
        int32_t current_count = right_count;
        restore_interrupts(irq_state);

        int32_t delta_count = current_count - previous_count;

        int64_t elapsed_us =
            absolute_time_diff_us(previous_time, now);

        if (elapsed_us <= 0)
        {
            motor_stop();
            printf("FAULT: Invalid measurement interval.\n");
            break;
        }

        float actual_rpm =
            ((float)delta_count * 60000000.0f) /
            (RIGHT_CPR * (float)elapsed_us);

        /* Stop if feedback indicates reverse motion */
        if (delta_count < 0)
        {
            motor_stop();
            printf("FAULT: Reverse encoder direction.\n");
            break;
        }

        /* Detect missing encoder pulses */
        if (delta_count == 0)
        {
            no_pulse_intervals++;
        }
        else
        {
            no_pulse_intervals = 0U;
        }

        if (no_pulse_intervals >= MAX_NO_PULSE_CYCLES)
        {
            motor_stop();
            printf("FAULT: No encoder pulses for 500 ms.\n");
            break;
        }

        /* PI controller */
        float error = TARGET_RPM - actual_rpm;
        float dt = (float)elapsed_us / 1000000.0f;

        integral_error += error * dt;

        /* Basic integral clamp */
        if (integral_error > 50.0f)
        {
            integral_error = 50.0f;
        }
        else if (integral_error < -50.0f)
        {
            integral_error = -50.0f;
        }

        float pwm_command =
            BASELINE_PWM +
            (KP * error) +
            (KI * integral_error);

        if (pwm_command > MAX_PWM_LEVEL)
        {
            pwm_command = MAX_PWM_LEVEL;
        }
        else if (pwm_command < 0.0f)
        {
            pwm_command = 0.0f;
        }

        motor_set_forward((uint16_t)pwm_command);

        printf(
            "Target = %.1f | Actual = %.2f | "
            "Error = %.2f | PWM = %.0f\n",
            (double)TARGET_RPM,
            (double)actual_rpm,
            (double)error,
            (double)pwm_command
        );

        previous_count = current_count;
        previous_time = now;
    }

    motor_stop();
    printf("TEST COMPLETE - Motor stopped.\n");

    while (true)
    {
        sleep_ms(1000U);
    }
}
