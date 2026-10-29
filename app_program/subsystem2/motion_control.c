
#include "motion_control.h"

#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/sync.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/*
 * Buddy 2 - Motion Control Foundation
 *
 * Raspberry Pi Pico W / Robo Pico
 *
 * Supported:
 * - Forward motor control
 * - Independent wheel speed targets
 * - Encoder counting and RPM feedback
 * - PI speed control
 * - Startup PWM boost
 * - Encoder timeout detection
 * - Zero-PWM stopping
 *
 * Not yet supported:
 * - Reverse motion
 * - Distance-based movement
 * - Angle-based turning
 * - Full task-safe command arbitration
 *
 * This version is intended for initial integration.
 */

/* Motor GPIO */
#define LEFT_MOTOR_A       10U
#define LEFT_MOTOR_B       11U

#define RIGHT_MOTOR_A       8U
#define RIGHT_MOTOR_B       9U

/* Encoder GPIO */
#define LEFT_ENC_A          4U
#define LEFT_ENC_B          5U

#define RIGHT_ENC_A         0U
#define RIGHT_ENC_B         1U

/* Preliminary encoder calibration */
#define LEFT_CPR         1271.0f
#define RIGHT_CPR        1266.0f

/* PWM configuration */
#define PWM_WRAP          999U
#define PWM_CLKDIV        125.0f

#define MAX_PWM           200.0f

/* Controller settings */
#define KP                  2.0f
#define KI                  2.0f

#define LEFT_BASELINE      85.0f
#define RIGHT_BASELINE     85.0f

/* Speed command limits */
#define MAX_TARGET_RPM     30.0f

/* Timing */
#define CONTROL_PERIOD_MS 100U
#define ENCODER_TIMEOUT_MS 500U

/* Startup boost */
#define STARTUP_PWM        150U
#define STARTUP_TIME_MS    150U

/* Encoder counters */
static volatile int32_t left_count = 0;
static volatile int32_t right_count = 0;

/* Controller state */
static bool initialised = false;

static motion_state_t motion_state =
    MOTION_STATE_STOPPED;

/* Desired speeds */
static float left_target_rpm = 0.0f;
static float right_target_rpm = 0.0f;

/* Measured speeds */
static float left_actual_rpm = 0.0f;
static float right_actual_rpm = 0.0f;

/* Independent PI accumulators */
static float left_integral = 0.0f;
static float right_integral = 0.0f;

/* Previous encoder measurements */
static int32_t previous_left_count = 0;
static int32_t previous_right_count = 0;

/* Timing information */
static absolute_time_t previous_time;

/* Startup handling */
static bool startup_active = false;
static absolute_time_t startup_start_time;

/* Encoder activity */
static uint32_t left_no_pulse_ms = 0U;
static uint32_t right_no_pulse_ms = 0U;


/* =====================================================
 * Encoder interrupt callback
 * ===================================================== */

static void encoder_callback(
    uint gpio,
    uint32_t events)
{
    (void)events;

    if (gpio == LEFT_ENC_A)
    {
        bool a = gpio_get(LEFT_ENC_A);
        bool b = gpio_get(LEFT_ENC_B);

        /* Positive count for forward rotation */
        left_count += (a == b) ? -1 : 1;
    }
    else if (gpio == RIGHT_ENC_A)
    {
        bool a = gpio_get(RIGHT_ENC_A);
        bool b = gpio_get(RIGHT_ENC_B);

        /* Positive count for forward rotation */
        right_count += (a == b) ? 1 : -1;
    }
}


/* =====================================================
 * PWM helpers
 * ===================================================== */

static uint16_t clamp_pwm(float pwm)
{
    if (!(pwm >= 0.0f))
    {
        return 0U;
    }

    if (pwm > MAX_PWM)
    {
        pwm = MAX_PWM;
    }

    return (uint16_t)pwm;
}


static void set_left_pwm(uint16_t level)
{
    /*
     * Verified forward configuration:
     * GP10 active, GP11 inactive.
     */

    pwm_set_gpio_level(LEFT_MOTOR_B, 0U);
    pwm_set_gpio_level(LEFT_MOTOR_A, level);
}


static void set_right_pwm(uint16_t level)
{
    /*
     * Verified forward configuration:
     * GP9 active, GP8 inactive.
     */

    pwm_set_gpio_level(RIGHT_MOTOR_A, 0U);
    pwm_set_gpio_level(RIGHT_MOTOR_B, level);
}


static void stop_hardware(void)
{
    pwm_set_gpio_level(LEFT_MOTOR_A, 0U);
    pwm_set_gpio_level(LEFT_MOTOR_B, 0U);

    pwm_set_gpio_level(RIGHT_MOTOR_A, 0U);
    pwm_set_gpio_level(RIGHT_MOTOR_B, 0U);
}


/* =====================================================
 * Encoder counter snapshot
 * ===================================================== */

static void read_encoder_counts(
    int32_t *left,
    int32_t *right)
{
    uint32_t irq_state =
        save_and_disable_interrupts();

    *left = left_count;
    *right = right_count;

    restore_interrupts(irq_state);
}


/* =====================================================
 * PI controller
 * ===================================================== */

static float calculate_pi(
    float target,
    float actual,
    float baseline,
    float *integral,
    float dt)
{
    if (target <= 0.0f)
    {
        *integral = 0.0f;
        return 0.0f;
    }

    float error = target - actual;

    *integral += error * dt;

    /* Limit integral accumulation */
    if (*integral > 50.0f)
    {
        *integral = 50.0f;
    }
    else if (*integral < -50.0f)
    {
        *integral = -50.0f;
    }

    float output =
        baseline +
        (KP * error) +
        (KI * (*integral));

    if (output > MAX_PWM)
    {
        output = MAX_PWM;
    }
    else if (output < 0.0f)
    {
        output = 0.0f;
    }

    return output;
}


/* =====================================================
 * Initialise hardware
 * ===================================================== */

motion_result_t motion_init(void)
{
    if (initialised)
    {
        return MOTION_OK;
    }

    const uint motor_pins[] = {
        LEFT_MOTOR_A,
        LEFT_MOTOR_B,
        RIGHT_MOTOR_A,
        RIGHT_MOTOR_B
    };

    /* Configure PWM output pins */
    for (uint i = 0U; i < 4U; i++)
    {
        gpio_set_function(
            motor_pins[i],
            GPIO_FUNC_PWM
        );
    }

    uint left_slice =
        pwm_gpio_to_slice_num(LEFT_MOTOR_A);

    uint right_slice =
        pwm_gpio_to_slice_num(RIGHT_MOTOR_A);

    pwm_set_clkdiv(
        left_slice,
        PWM_CLKDIV
    );

    pwm_set_wrap(
        left_slice,
        PWM_WRAP
    );

    pwm_set_clkdiv(
        right_slice,
        PWM_CLKDIV
    );

    pwm_set_wrap(
        right_slice,
        PWM_WRAP
    );

    /* Start with zero PWM */
    stop_hardware();

    pwm_set_enabled(left_slice, true);
    pwm_set_enabled(right_slice, true);

    /* Configure encoder inputs */
    const uint encoder_pins[] = {
        LEFT_ENC_A,
        LEFT_ENC_B,
        RIGHT_ENC_A,
        RIGHT_ENC_B
    };

    for (uint i = 0U; i < 4U; i++)
    {
        gpio_init(encoder_pins[i]);

        gpio_set_dir(
            encoder_pins[i],
            GPIO_IN
        );

        gpio_pull_up(encoder_pins[i]);
    }

    /* Shared encoder interrupt callback */
    gpio_set_irq_enabled_with_callback(
        LEFT_ENC_A,
        GPIO_IRQ_EDGE_RISE |
        GPIO_IRQ_EDGE_FALL,
        true,
        &encoder_callback
    );

    gpio_set_irq_enabled(
        RIGHT_ENC_A,
        GPIO_IRQ_EDGE_RISE |
        GPIO_IRQ_EDGE_FALL,
        true
    );

    left_count = 0;
    right_count = 0;

    left_target_rpm = 0.0f;
    right_target_rpm = 0.0f;

    left_actual_rpm = 0.0f;
    right_actual_rpm = 0.0f;

    left_integral = 0.0f;
    right_integral = 0.0f;

    left_no_pulse_ms = 0U;
    right_no_pulse_ms = 0U;

    startup_active = false;

    previous_time = get_absolute_time();

    motion_state = MOTION_STATE_STOPPED;

    initialised = true;

    return MOTION_OK;
}


/* =====================================================
 * Set independent wheel speed targets
 * ===================================================== */

motion_result_t motion_set_speed(
    float left_rpm,
    float right_rpm)
{
    if (!initialised)
    {
        return MOTION_NOT_READY;
    }

    if (motion_state == MOTION_STATE_FAULT)
    {
        return MOTION_ERROR;
    }

    /*
     * Forward speeds only.
     * The comparisons also reject NaN.
     */

    if (!(left_rpm >= 0.0f &&
          left_rpm <= MAX_TARGET_RPM) ||
        !(right_rpm >= 0.0f &&
          right_rpm <= MAX_TARGET_RPM))
    {
        return MOTION_ERROR;
    }

    if (left_rpm == 0.0f &&
        right_rpm == 0.0f)
    {
        return motion_stop();
    }

    /* Avoid unnecessary controller resets */
    if (motion_state == MOTION_STATE_RUNNING &&
        left_target_rpm == left_rpm &&
        right_target_rpm == right_rpm)
    {
        return MOTION_OK;
    }

    bool starting_from_stop =
        (motion_state == MOTION_STATE_STOPPED);

    left_target_rpm = left_rpm;
    right_target_rpm = right_rpm;

    if (starting_from_stop)
    {
        /* Prepare both controllers */
        left_integral = 0.0f;
        right_integral = 0.0f;

        left_no_pulse_ms = 0U;
        right_no_pulse_ms = 0U;

        read_encoder_counts(
            &previous_left_count,
            &previous_right_count
        );

        previous_time = get_absolute_time();

        /* Startup boost handled by motion_update() */
        startup_active = true;
        startup_start_time = previous_time;
    }

    motion_state = MOTION_STATE_RUNNING;

    return MOTION_OK;
}


/* =====================================================
 * Stop all motion
 * ===================================================== */

motion_result_t motion_stop(void)
{
    if (!initialised)
    {
        return MOTION_NOT_READY;
    }

    left_target_rpm = 0.0f;
    right_target_rpm = 0.0f;

    left_integral = 0.0f;
    right_integral = 0.0f;

    startup_active = false;

    left_no_pulse_ms = 0U;
    right_no_pulse_ms = 0U;

    stop_hardware();

    if (motion_state != MOTION_STATE_FAULT)
    {
        motion_state = MOTION_STATE_STOPPED;
    }

    /*
     * Physical wheels may continue rotating
     * briefly after PWM is removed.
     */
    return MOTION_OK;
}


/* =====================================================
 * Read current motion status
 * ===================================================== */

motion_result_t motion_get_status(
    motion_status_t *status)
{
    if (status == NULL)
    {
        return MOTION_ERROR;
    }

    if (!initialised)
    {
        return MOTION_NOT_READY;
    }

    read_encoder_counts(
        &status->left_encoder_count,
        &status->right_encoder_count
    );

    status->left_rpm = left_actual_rpm;
    status->right_rpm = right_actual_rpm;

    status->state = motion_state;

    return MOTION_OK;
}


/* =====================================================
 * Periodic Motion Control Task Update
 *
 * Intended to be called every 100 ms.
 * This function must not run in an ISR.
 * ===================================================== */

void motion_update(void)
{
    if (!initialised)
    {
        return;
    }

    if (motion_state != MOTION_STATE_RUNNING)
    {
        stop_hardware();
        return;
    }

    absolute_time_t now = get_absolute_time();

    int64_t elapsed_us =
        absolute_time_diff_us(previous_time, now);

    if (elapsed_us <= 0)
    {
        return;
    }

    /*
     * Handle startup boost without blocking
     * the motion-control task.
     */
    if (startup_active)
    {
        int64_t startup_elapsed_us =
            absolute_time_diff_us(
                startup_start_time,
                now
            );

        if (startup_elapsed_us <
            ((int64_t)STARTUP_TIME_MS * 1000))
        {
            set_left_pwm(
                left_target_rpm > 0.0f
                ? STARTUP_PWM : 0U
            );

            set_right_pwm(
                right_target_rpm > 0.0f
                ? STARTUP_PWM : 0U
            );

            return;
        }

        startup_active = false;

        /* Begin feedback measurement after boost */
        read_encoder_counts(
            &previous_left_count,
            &previous_right_count
        );

        previous_time = now;

        left_no_pulse_ms = 0U;
        right_no_pulse_ms = 0U;

        return;
    }

    if (elapsed_us <
        ((int64_t)CONTROL_PERIOD_MS * 1000))
    {
        return;
    }

    int32_t current_left;
    int32_t current_right;

    read_encoder_counts(
        &current_left,
        &current_right
    );

    int32_t delta_left =
        current_left - previous_left_count;

    int32_t delta_right =
        current_right - previous_right_count;

    float dt =
        (float)elapsed_us / 1000000.0f;

    left_actual_rpm =
        ((float)delta_left * 60000000.0f) /
        (LEFT_CPR * (float)elapsed_us);

    right_actual_rpm =
        ((float)delta_right * 60000000.0f) /
        (RIGHT_CPR * (float)elapsed_us);

    /* Reject reverse encoder feedback */
    if ((left_target_rpm > 0.0f &&
         delta_left < 0) ||
        (right_target_rpm > 0.0f &&
         delta_right < 0))
    {
        stop_hardware();

        motion_state = MOTION_STATE_FAULT;

        return;
    }

    /*
     * Accumulate time with no encoder pulses.
     * Only monitor a wheel with a nonzero target.
     */

    uint32_t elapsed_ms =
        (uint32_t)(elapsed_us / 1000);

    if (left_target_rpm > 0.0f &&
        delta_left == 0)
    {
        if (left_no_pulse_ms <
            ENCODER_TIMEOUT_MS)
        {
            left_no_pulse_ms += elapsed_ms;
        }
    }
    else
    {
        left_no_pulse_ms = 0U;
    }

    if (right_target_rpm > 0.0f &&
        delta_right == 0)
    {
        if (right_no_pulse_ms <
            ENCODER_TIMEOUT_MS)
        {
            right_no_pulse_ms += elapsed_ms;
        }
    }
    else
    {
        right_no_pulse_ms = 0U;
    }

    if (left_no_pulse_ms >= ENCODER_TIMEOUT_MS ||
        right_no_pulse_ms >= ENCODER_TIMEOUT_MS)
    {
        stop_hardware();

        motion_state = MOTION_STATE_FAULT;

        return;
    }

    /* Calculate independent PI outputs */
    float left_pwm = calculate_pi(
        left_target_rpm,
        left_actual_rpm,
        LEFT_BASELINE,
        &left_integral,
        dt
    );

    float right_pwm = calculate_pi(
        right_target_rpm,
        right_actual_rpm,
        RIGHT_BASELINE,
        &right_integral,
        dt
    );

    /* Apply forward motor commands */
    set_left_pwm(clamp_pwm(left_pwm));
    set_right_pwm(clamp_pwm(right_pwm));

    /* Save measurements for next update */
    previous_left_count = current_left;
    previous_right_count = current_right;

    previous_time = now;
}


/* =====================================================
 * Future Navigation Commands
 *
 * Not implemented in the base version.
 * ===================================================== */

motion_result_t motion_move_forward(
    float distance_cm)
{
    (void)distance_cm;

    return MOTION_NOT_IMPLEMENTED;
}


motion_result_t motion_move_backward(
    float distance_cm)
{
    (void)distance_cm;

    return MOTION_NOT_IMPLEMENTED;
}


motion_result_t motion_turn_left(
    float angle_deg)
{
    (void)angle_deg;

    return MOTION_NOT_IMPLEMENTED;
}


motion_result_t motion_turn_right(
    float angle_deg)
{
    (void)angle_deg;

    return MOTION_NOT_IMPLEMENTED;
}
