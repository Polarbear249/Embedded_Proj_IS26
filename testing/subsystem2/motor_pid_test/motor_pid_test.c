
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/sync.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

/* =========================================================
 * BUDDY 2 - LEFT MOTOR PI SPEED CONTROL TEST
 * Raspberry Pi Pico W + Robo Pico
 *
 * Verified physical connections:
 * Left motor   : GP10 / GP11
 * Left encoder : GP4 / GP5
 *
 * The car must be raised during this test.
 * ========================================================= */

/* Motor GPIO pins */
#define LEFT_MOTOR_A        10U
#define LEFT_MOTOR_B        11U

/* Encoder GPIO pins */
#define LEFT_ENC_A          4U
#define LEFT_ENC_B          5U

/* Preliminary encoder calibration */
#define LEFT_CPR            1271.0f

/* PWM configuration */
#define PWM_WRAP            999U
#define PWM_CLKDIV          125.0f

/* PI control settings */
#define TARGET_RPM          10.0f
#define KP                  2.0f
#define KI                  2.0f

/* Control loop settings */
#define CONTROL_TIME_MS     100U
#define TEST_ITERATIONS     30U

/* PWM limits */
#define BASELINE_PWM        85.0f
#define MAX_PWM_LEVEL       200.0f

/* Startup boost */
#define STARTUP_PWM         150U
#define STARTUP_TIME_MS     150U

/* Encoder fault detection */
#define MAX_NO_PULSE_CYCLES 5U

/* Encoder counter updated by interrupts */
static volatile int32_t left_count = 0;


/* =========================================================
 * ENCODER INTERRUPT CALLBACK
 * ========================================================= */

static void encoder_callback(uint gpio, uint32_t events)
{
    (void)events;

    if (gpio == LEFT_ENC_A)
    {
        bool a = gpio_get(LEFT_ENC_A);
        bool b = gpio_get(LEFT_ENC_B);

        /* Forward rotation should produce positive counts */
        left_count += (a == b) ? -1 : 1;
    }
}


/* =========================================================
 * MOTOR INITIALISATION
 * ========================================================= */

static void motor_init(void)
{
    gpio_set_function(LEFT_MOTOR_A, GPIO_FUNC_PWM);
    gpio_set_function(LEFT_MOTOR_B, GPIO_FUNC_PWM);

    uint slice_a = pwm_gpio_to_slice_num(LEFT_MOTOR_A);
    uint slice_b = pwm_gpio_to_slice_num(LEFT_MOTOR_B);

    pwm_set_clkdiv(slice_a, PWM_CLKDIV);
    pwm_set_clkdiv(slice_b, PWM_CLKDIV);

    pwm_set_wrap(slice_a, PWM_WRAP);
    pwm_set_wrap(slice_b, PWM_WRAP);

    /* Ensure motor is initially stopped */
    pwm_set_gpio_level(LEFT_MOTOR_A, 0U);
    pwm_set_gpio_level(LEFT_MOTOR_B, 0U);

    pwm_set_enabled(slice_a, true);
    pwm_set_enabled(slice_b, true);
}


/* =========================================================
 * SET LEFT MOTOR FORWARD PWM
 * ========================================================= */

static void motor_set_forward(uint16_t pwm_level)
{
    if (pwm_level > PWM_WRAP)
    {
        pwm_level = PWM_WRAP;
    }

    /* Keep the reverse channel inactive */
    pwm_set_gpio_level(LEFT_MOTOR_B, 0U);

    /* Drive the forward channel */
    pwm_set_gpio_level(LEFT_MOTOR_A, pwm_level);
}


/* =========================================================
 * ENCODER INITIALISATION
 * ========================================================= */

static void encoder_init(void)
{
    gpio_init(LEFT_ENC_A);
    gpio_set_dir(LEFT_ENC_A, GPIO_IN);
    gpio_pull_up(LEFT_ENC_A);

    gpio_init(LEFT_ENC_B);
    gpio_set_dir(LEFT_ENC_B, GPIO_IN);
    gpio_pull_up(LEFT_ENC_B);

    gpio_set_irq_enabled_with_callback(
        LEFT_ENC_A,
        GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL,
        true,
        &encoder_callback
    );
}


/* =========================================================
 * MAIN PROGRAM
 * ========================================================= */

int main(void)
{
    stdio_init_all();

    /* Allow time for USB serial connection */
    sleep_ms(2000U);

    motor_init();
    encoder_init();

    motor_set_forward(0U);

    printf("\n========================================\n");
    printf("BUDDY 2 - LEFT MOTOR PI CONTROL TEST\n");
    printf("========================================\n");

    printf("Motor pins   : GP10 / GP11\n");
    printf("Encoder pins : GP4 / GP5\n");
    printf("Target RPM   : %.1f\n", (double)TARGET_RPM);
    printf("Kp           : %.1f\n", (double)KP);
    printf("Ki           : %.1f\n", (double)KI);
    printf("Baseline PWM : %.0f\n", (double)BASELINE_PWM);
    printf("Startup PWM  : %u\n", STARTUP_PWM);

    /* Keep motor stopped during preparation */
    sleep_ms(500U);

    /* Reset encoder count */
    uint32_t irq_state = save_and_disable_interrupts();
    left_count = 0;
    restore_interrupts(irq_state);

    /* Control variables */
    int32_t previous_count = 0;
    float integral_error = 0.0f;
    uint32_t no_pulse_intervals = 0U;

    absolute_time_t previous_time = get_absolute_time();

    /* =====================================================
     * STARTUP BOOST
     * ===================================================== */

    printf("\nApplying startup boost...\n");

    motor_set_forward(STARTUP_PWM);
    sleep_ms(STARTUP_TIME_MS);

    /* Reset measurements after startup boost */
    irq_state = save_and_disable_interrupts();
    left_count = 0;
    restore_interrupts(irq_state);

    previous_count = 0;
    integral_error = 0.0f;
    no_pulse_intervals = 0U;

    /* Begin measuring from this point */
    previous_time = get_absolute_time();

    /* Transition to baseline PWM */
    motor_set_forward((uint16_t)BASELINE_PWM);

    printf("Starting PI control...\n\n");

    /* =====================================================
     * PI CONTROL LOOP
     * ===================================================== */

    for (uint i = 0U; i < TEST_ITERATIONS; i++)
    {
        sleep_ms(CONTROL_TIME_MS);

        absolute_time_t current_time = get_absolute_time();

        /* Read encoder counter atomically */
        irq_state = save_and_disable_interrupts();
        int32_t current_count = left_count;
        restore_interrupts(irq_state);

        int32_t delta_count = current_count - previous_count;

        int64_t elapsed_us =
            absolute_time_diff_us(previous_time, current_time);

        float actual_rpm = 0.0f;

        if (elapsed_us > 0)
        {
            actual_rpm =
                ((float)delta_count * 60000000.0f) /
                (LEFT_CPR * (float)elapsed_us);
        }

        /* =============================================
         * SAFETY: REVERSE ENCODER DETECTION
         * ============================================= */

        if (delta_count < 0)
        {
            motor_set_forward(0U);

            printf(
                "FAULT: Reverse encoder direction detected.\n"
            );

            break;
        }

        /* =============================================
         * SAFETY: MISSING ENCODER PULSES
         * ============================================= */

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
            motor_set_forward(0U);

            printf(
                "FAULT: No encoder pulses for 500 ms.\n"
            );

            printf("Motor stopped.\n");

            break;
        }

        /* =============================================
         * PI CONTROL CALCULATION
         * ============================================= */

        float error = TARGET_RPM - actual_rpm;

        /* Convert elapsed time to seconds */
        float dt = (float)elapsed_us / 1000000.0f;

        /* Integral control */
        integral_error += error * dt;

        /* Limit integral windup */
        if (integral_error > 50.0f)
        {
            integral_error = 50.0f;
        }
        else if (integral_error < -50.0f)
        {
            integral_error = -50.0f;
        }

        /* PI controller equation */
        float pwm_command =
            BASELINE_PWM +
            (KP * error) +
            (KI * integral_error);

        /* =============================================
         * PWM OUTPUT LIMITS
         * ============================================= */

        if (pwm_command > MAX_PWM_LEVEL)
        {
            pwm_command = MAX_PWM_LEVEL;
        }
        else if (pwm_command < 0.0f)
        {
            pwm_command = 0.0f;
        }

        /* Apply calculated PWM */
        motor_set_forward((uint16_t)pwm_command);

        /* Display results */
        printf(
            "Target = %.1f | Actual = %.2f | "
            "Error = %.2f | PWM = %.0f\n",
            (double)TARGET_RPM,
            (double)actual_rpm,
            (double)error,
            (double)pwm_command
        );

        /* Update values for next iteration */
        previous_count = current_count;
        previous_time = current_time;
    }

    /* =====================================================
     * AUTOMATIC MOTOR STOP
     * ===================================================== */

    motor_set_forward(0U);

    printf("\n========================================\n");
    printf("TEST COMPLETE - Motor stopped.\n");
    printf("========================================\n");

    while (true)
    {
        sleep_ms(1000U);
    }
}
