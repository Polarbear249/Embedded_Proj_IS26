
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/sync.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

/* =========================================================
 * BUDDY 2 - DUAL MOTOR PI SPEED CONTROL
 * Raspberry Pi Pico W + Robo Pico
 *
 * Bench test only: both wheels must be raised.
 * ========================================================= */

/* Verified physical motor and encoder connections */
#define LEFT_MOTOR_A          10U
#define LEFT_MOTOR_B          11U
#define LEFT_ENC_A             4U
#define LEFT_ENC_B             5U

#define RIGHT_MOTOR_A          8U
#define RIGHT_MOTOR_B          9U
#define RIGHT_ENC_A            0U
#define RIGHT_ENC_B            1U

/* Preliminary encoder counts per wheel revolution */
#define LEFT_CPR            1271.0f
#define RIGHT_CPR           1266.0f

/* PWM setup */
#define PWM_WRAP             999U
#define PWM_CLKDIV           125.0f

/* Initial independent PI gains */
#define TARGET_RPM           10.0f
#define KP                    2.0f
#define KI                    2.0f

/* Test timing */
#define CONTROL_TIME_MS     100U
#define TEST_ITERATIONS      30U

/* Left and right baseline PWM */
#define LEFT_BASELINE_PWM    85.0f
#define RIGHT_BASELINE_PWM   85.0f
#define MAX_PWM_LEVEL       200.0f

/* Startup boost */
#define STARTUP_PWM         150U
#define STARTUP_TIME_MS     150U

/* Fault detection */
#define MAX_NO_PULSE_CYCLES  5U

/* Encoder counts updated by GPIO interrupts */
static volatile int32_t left_count = 0;
static volatile int32_t right_count = 0;

/* =========================================================
 * SHARED ENCODER INTERRUPT CALLBACK
 * ========================================================= */

static void encoder_callback(uint gpio, uint32_t events)
{
    (void)events;

    if (gpio == LEFT_ENC_A)
    {
        bool a = gpio_get(LEFT_ENC_A);
        bool b = gpio_get(LEFT_ENC_B);

        /* Verified left-forward convention */
        left_count += (a == b) ? -1 : 1;
    }
    else if (gpio == RIGHT_ENC_A)
    {
        bool a = gpio_get(RIGHT_ENC_A);
        bool b = gpio_get(RIGHT_ENC_B);

        /* Verified right-forward convention */
        right_count += (a == b) ? 1 : -1;
    }
}

/* =========================================================
 * MOTOR INITIALISATION
 * ========================================================= */

static void motor_init(void)
{
    const uint pins[] = {
        LEFT_MOTOR_A,
        LEFT_MOTOR_B,
        RIGHT_MOTOR_A,
        RIGHT_MOTOR_B
    };

    for (uint i = 0U; i < 4U; i++)
    {
        gpio_set_function(pins[i], GPIO_FUNC_PWM);

        pwm_set_gpio_level(pins[i], 0U);
    }

    /* GP8/GP9 share a PWM slice; GP10/GP11 share another */
    uint left_slice = pwm_gpio_to_slice_num(LEFT_MOTOR_A);
    uint right_slice = pwm_gpio_to_slice_num(RIGHT_MOTOR_A);

    pwm_set_clkdiv(left_slice, PWM_CLKDIV);
    pwm_set_wrap(left_slice, PWM_WRAP);

    pwm_set_clkdiv(right_slice, PWM_CLKDIV);
    pwm_set_wrap(right_slice, PWM_WRAP);

    pwm_set_enabled(left_slice, true);
    pwm_set_enabled(right_slice, true);
}

/* =========================================================
 * MOTOR OUTPUT FUNCTIONS
 * ========================================================= */

static uint16_t clamp_pwm(float level)
{
    if (level < 0.0f)
    {
        level = 0.0f;
    }
    else if (level > MAX_PWM_LEVEL)
    {
        level = MAX_PWM_LEVEL;
    }

    return (uint16_t)level;
}

static void left_motor_forward(uint16_t level)
{
    /* Verified forward: GP10 PWM, GP11 inactive */
    pwm_set_gpio_level(LEFT_MOTOR_B, 0U);
    pwm_set_gpio_level(LEFT_MOTOR_A, level);
}

static void right_motor_forward(uint16_t level)
{
    /* Verified forward: GP9 PWM, GP8 inactive */
    pwm_set_gpio_level(RIGHT_MOTOR_A, 0U);
    pwm_set_gpio_level(RIGHT_MOTOR_B, level);
}

static void motors_stop(void)
{
    pwm_set_gpio_level(LEFT_MOTOR_A, 0U);
    pwm_set_gpio_level(LEFT_MOTOR_B, 0U);
    pwm_set_gpio_level(RIGHT_MOTOR_A, 0U);
    pwm_set_gpio_level(RIGHT_MOTOR_B, 0U);
}

/* =========================================================
 * ENCODER INITIALISATION
 * ========================================================= */

static void encoder_init(void)
{
    const uint pins[] = {
        LEFT_ENC_A,
        LEFT_ENC_B,
        RIGHT_ENC_A,
        RIGHT_ENC_B
    };

    for (uint i = 0U; i < 4U; i++)
    {
        gpio_init(pins[i]);
        gpio_set_dir(pins[i], GPIO_IN);
        gpio_pull_up(pins[i]);
    }

    /* Install one shared callback for both encoders */
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
}

/* =========================================================
 * PI CONTROLLER HELPER
 * ========================================================= */

static float update_pi(
    float target_rpm,
    float actual_rpm,
    float baseline_pwm,
    float *integral_error,
    float dt)
{
    float error = target_rpm - actual_rpm;

    *integral_error += error * dt;

    /* Basic integral clamp */
    if (*integral_error > 50.0f)
    {
        *integral_error = 50.0f;
    }
    else if (*integral_error < -50.0f)
    {
        *integral_error = -50.0f;
    }

    float command =
        baseline_pwm +
        KP * error +
        KI * (*integral_error);

    if (command > MAX_PWM_LEVEL)
    {
        command = MAX_PWM_LEVEL;
    }
    else if (command < 0.0f)
    {
        command = 0.0f;
    }

    return command;
}

/* =========================================================
 * MAIN PROGRAM
 * ========================================================= */

int main(void)
{
    stdio_init_all();
    sleep_ms(2000U);

    motor_init();
    encoder_init();
    motors_stop();

    printf("\nBUDDY 2 - DUAL MOTOR PI TEST\n");
    printf("Left: GP10/GP11, encoder GP4/GP5\n");
    printf("Right: GP8/GP9, encoder GP0/GP1\n");
    printf("Both targets: %.1f RPM\n", (double)TARGET_RPM);

    sleep_ms(500U);

    /* Start both wheels with a short boost */
    printf("Applying startup boost...\n");

    left_motor_forward(STARTUP_PWM);
    right_motor_forward(STARTUP_PWM);

    sleep_ms(STARTUP_TIME_MS);

    /* Reset both encoder counters after boost */
    uint32_t irq_state = save_and_disable_interrupts();
    left_count = 0;
    right_count = 0;
    restore_interrupts(irq_state);

    int32_t previous_left = 0;
    int32_t previous_right = 0;

    float left_integral = 0.0f;
    float right_integral = 0.0f;

    uint32_t left_no_pulse = 0U;
    uint32_t right_no_pulse = 0U;

    absolute_time_t previous_time = get_absolute_time();

    left_motor_forward((uint16_t)LEFT_BASELINE_PWM);
    right_motor_forward((uint16_t)RIGHT_BASELINE_PWM);

    printf("Starting dual PI control...\n");

    for (uint i = 0U; i < TEST_ITERATIONS; i++)
    {
        sleep_ms(CONTROL_TIME_MS);

        absolute_time_t now = get_absolute_time();

        irq_state = save_and_disable_interrupts();
        int32_t current_left = left_count;
        int32_t current_right = right_count;
        restore_interrupts(irq_state);

        int32_t delta_left = current_left - previous_left;
        int32_t delta_right = current_right - previous_right;

        int64_t elapsed_us =
            absolute_time_diff_us(previous_time, now);

        if (elapsed_us <= 0)
        {
            motors_stop();
            printf("FAULT: Invalid timing interval.\n");
            break;
        }

        float left_rpm =
            ((float)delta_left * 60000000.0f) /
            (LEFT_CPR * (float)elapsed_us);

        float right_rpm =
            ((float)delta_right * 60000000.0f) /
            (RIGHT_CPR * (float)elapsed_us);

        /* Stop both motors on reverse encoder feedback */
        if (delta_left < 0 || delta_right < 0)
        {
            motors_stop();
            printf("FAULT: Reverse encoder feedback.\n");
            break;
        }

        /* Track missing encoder pulses separately */
        left_no_pulse = (delta_left == 0)
            ? left_no_pulse + 1U : 0U;

        right_no_pulse = (delta_right == 0)
            ? right_no_pulse + 1U : 0U;

        /* Stop both if either encoder times out */
        if (left_no_pulse >= MAX_NO_PULSE_CYCLES ||
            right_no_pulse >= MAX_NO_PULSE_CYCLES)
        {
            motors_stop();
            printf("FAULT: Encoder timeout. Both motors stopped.\n");
            break;
        }

        float dt = (float)elapsed_us / 1000000.0f;

        float left_pwm = update_pi(
            TARGET_RPM,
            left_rpm,
            LEFT_BASELINE_PWM,
            &left_integral,
            dt
        );

        float right_pwm = update_pi(
            TARGET_RPM,
            right_rpm,
            RIGHT_BASELINE_PWM,
            &right_integral,
            dt
        );

        left_motor_forward(clamp_pwm(left_pwm));
        right_motor_forward(clamp_pwm(right_pwm));

        printf(
            "L: %.2f RPM PWM %.0f | R: %.2f RPM PWM %.0f\n",
            (double)left_rpm,
            (double)left_pwm,
            (double)right_rpm,
            (double)right_pwm
        );

        previous_left = current_left;
        previous_right = current_right;
        previous_time = now;
    }

    motors_stop();

    printf("TEST COMPLETE - Both motors stopped.\n");

    while (true)
    {
        sleep_ms(1000U);
    }
}
