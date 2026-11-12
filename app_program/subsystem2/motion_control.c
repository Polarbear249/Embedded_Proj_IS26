
#include "motion_control.h"

#include "motor.h"
#include "encoder.h"
#include "pid.h"

#include <stdint.h>

/*
 * Buddy 2 - Motion Control Coordinator
 *
 * Execution model:
 * - One task (motion_control_task) owns the motors: motion_tick()
 *   every MOTION_TICK_MS applies duty commands and runs
 *   motion_update() (PI speed control) about every 100 ms.
 * - Encoder counts come from GPIO edge interrupts.
 * - Commands from other tasks (motion_set_speed, motion_set_duty,
 *   motion_stop) only store the request under command_mutex; the
 *   motion task reads it under the same mutex. Two modes, latest
 *   command wins: SPEED (PI) and DUTY (open loop, with a lease).
 *
 * Current implementation:
 * - Forward wheel-speed control
 * - PI speed regulation
 * - Encoder RPM feedback
 * - Motor stopping
 * - Encoder feedback timeout detection
 *
 * Distance and turning are not yet implemented.
 */

#define MOTION_UPDATE_MS          100U
#define MOTION_DT_SECONDS         0.1f

/* Preliminary bench-test settings */
#define MOTION_KP                 2.0f
#define MOTION_KI                 2.0f

#define MOTION_BASE_PWM           85.0f
#if MOTOR_TEST_HIGH_POWER
#define MOTION_MAX_PWM            500.0f    /* test only, see motor.h */
#else
#define MOTION_MAX_PWM            200.0f
#endif

#if MOTOR_TEST_HIGH_POWER
#define MOTION_START_PWM          350U      /* test only, see motor.h */
#else
#define MOTION_START_PWM          150U
#endif
#define MOTION_START_CYCLES       2U

#define MOTION_MAX_TARGET_RPM     30.0f

/*
 * Encoder feedback watchdog.
 *
 * If a wheel is commanded to move but no encoder
 * count changes are observed for approximately
 * 1000 ms, stop both motors and enter FAULT.
 *
 * This threshold is preliminary.
 */
#define MOTION_ENCODER_TIMEOUT_MS 1000U

/* Command lock: callers wait at most this long (the motion task holds
   it only for one tick's work, tens of us). TMO_POL for duty commands,
   which S3 renews every 10 ms anyway. */
#define MOTION_LOCK_WAIT_MS       (10)

typedef enum
{
    MOTION_MODE_IDLE = 0,
    MOTION_MODE_SPEED,
    MOTION_MODE_DUTY
} motion_mode_t;

static ID            command_mutex = 0;
static motion_mode_t motion_mode = MOTION_MODE_IDLE;
static int16_t       duty_left = 0;
static int16_t       duty_right = 0;
static int16_t       applied_left = 0;      /* motion task only */
static int16_t       applied_right = 0;
static UW            duty_issued_ms = 0U;
static UW            duty_lease_ms = 0U;
static UW            last_pi_ms = 0U;       /* motion task only */

/* Emergency-stop latch: written under command_mutex; read without the
   lock by motion_is_estopped() (one byte, a whole value either way). */
static volatile bool estop_latched = false;

static uint32_t left_no_pulse_ms = 0U;
static uint32_t right_no_pulse_ms = 0U;

static int motion_ready = 0;

static motion_state_t motion_state =
    MOTION_STATE_STOPPED;

static float target_left_rpm = 0.0f;
static float target_right_rpm = 0.0f;

static float actual_left_rpm = 0.0f;
static float actual_right_rpm = 0.0f;

static int32_t previous_left_count = 0;
static int32_t previous_right_count = 0;

static uint32_t startup_cycles = 0U;

static pi_controller_t left_pi;
static pi_controller_t right_pi;

/*
 * Convert floating-point PWM to an integer
 * motor-driver command.
 */
static uint16_t motion_pwm_value(float value)
{
    if (value <= 0.0f)
    {
        return 0U;
    }

    if (value >= MOTION_MAX_PWM)
    {
        return (uint16_t)MOTION_MAX_PWM;
    }

    return (uint16_t)value;
}

static void motion_stop_locked(void);
static void motion_duty_tick(UW now_ms);

/*
 * Enter fault state and stop both motors.
 */
static void motion_enter_fault(void)
{
    (void)motor_stop();

    target_left_rpm = 0.0f;
    target_right_rpm = 0.0f;

    startup_cycles = 0U;

    motion_state = MOTION_STATE_FAULT;
}

/*
 * Initialise motors, encoders and PI controllers.
 */
motion_result_t motion_init(void)
{
    ER er;
    T_CMTX cmtx = {0};

    if (motion_ready)
    {
        return MOTION_OK;
    }

    cmtx.mtxatr = TA_INHERIT;
    command_mutex = tk_cre_mtx(&cmtx);

    if (command_mutex < E_OK)
    {
        return MOTION_ERROR;
    }

    er = motor_init();

    if (er != E_OK)
    {
        return MOTION_ERROR;
    }

    er = encoder_init();

    if (er != E_OK)
    {
        (void)motor_stop();
        return MOTION_ERROR;
    }

    pi_init(
        &left_pi,
        MOTION_KP,
        MOTION_KI,
        -MOTION_BASE_PWM,
        MOTION_MAX_PWM - MOTION_BASE_PWM
    );

    pi_init(
        &right_pi,
        MOTION_KP,
        MOTION_KI,
        -MOTION_BASE_PWM,
        MOTION_MAX_PWM - MOTION_BASE_PWM
    );

    encoder_get_counts(
        &previous_left_count,
        &previous_right_count
    );

    target_left_rpm = 0.0f;
    target_right_rpm = 0.0f;

    actual_left_rpm = 0.0f;
    actual_right_rpm = 0.0f;

    left_no_pulse_ms = 0U;
    right_no_pulse_ms = 0U;

    startup_cycles = 0U;

    motion_state = MOTION_STATE_STOPPED;
    motion_ready = 1;

    return MOTION_OK;
}

/*
 * Request independent forward wheel speeds.
 *
 * Valid range: 0 to 30 RPM per wheel.
 *
 * Commands are intended to be serialised by
 * the motion-control task.
 */
motion_result_t motion_set_speed(
    float left_rpm,
    float right_rpm)
{
    if (!motion_ready)
    {
        return MOTION_NOT_READY;
    }

    /*
     * Reject negative, excessive and NaN inputs.
     */
    if (!(left_rpm >= 0.0f &&
          left_rpm <= MOTION_MAX_TARGET_RPM &&
          right_rpm >= 0.0f &&
          right_rpm <= MOTION_MAX_TARGET_RPM))
    {
        return MOTION_ERROR;
    }

    if (tk_loc_mtx(command_mutex, MOTION_LOCK_WAIT_MS) != E_OK)
    {
        return MOTION_BUSY;
    }

    if (motion_state == MOTION_STATE_FAULT)
    {
        (void)tk_unl_mtx(command_mutex);
        return MOTION_ERROR;
    }

    if (left_rpm == 0.0f && right_rpm == 0.0f)
    {
        motion_stop_locked();
        (void)tk_unl_mtx(command_mutex);
        return MOTION_OK;
    }

    if (estop_latched)
    {
        (void)tk_unl_mtx(command_mutex);
        return MOTION_ERROR;
    }

    if ((motion_state == MOTION_STATE_STOPPED) ||
        (motion_mode != MOTION_MODE_SPEED))
    {
        pi_reset(&left_pi);
        pi_reset(&right_pi);

        left_no_pulse_ms = 0U;
        right_no_pulse_ms = 0U;

        encoder_get_counts(
            &previous_left_count,
            &previous_right_count
        );

        startup_cycles = MOTION_START_CYCLES;
    }

    /*
     * Reset the watchdog for any wheel that has
     * been commanded to stop.
     */
    if (left_rpm == 0.0f)
    {
        left_no_pulse_ms = 0U;
        pi_reset(&left_pi);
    }

    if (right_rpm == 0.0f)
    {
        right_no_pulse_ms = 0U;
        pi_reset(&right_pi);
    }

    target_left_rpm = left_rpm;
    target_right_rpm = right_rpm;

    motion_mode = MOTION_MODE_SPEED;
    motion_state = MOTION_STATE_RUNNING;

    (void)tk_unl_mtx(command_mutex);

    return MOTION_OK;
}

/*
 * Open-loop signed duty with a lease, see motion_control.h.
 */
ER motion_set_duty(INT left, INT right, UINT lease_ms)
{
    SYSTIM now;
    ER     er;

    if ((left < -MOTION_MAX_DUTY) || (left > MOTION_MAX_DUTY) ||
        (right < -MOTION_MAX_DUTY) || (right > MOTION_MAX_DUTY) ||
        (lease_ms == 0U) || (lease_ms > MOTION_MAX_LEASE_MS))
    {
        return E_PAR;
    }

    if (!motion_ready)
    {
        return E_OBJ;
    }

    er = tk_loc_mtx(command_mutex, TMO_POL);

    if (er != E_OK)
    {
        return E_TMOUT;
    }

    if ((motion_state == MOTION_STATE_FAULT) || estop_latched)
    {
        er = E_OBJ;
    }
    else
    {
        er = tk_get_tim(&now);
    }

    if (er == E_OK)
    {
        if (motion_mode != MOTION_MODE_DUTY)
        {
            /* leaving SPEED mode: drop the PI state */
            target_left_rpm = 0.0f;
            target_right_rpm = 0.0f;
            pi_reset(&left_pi);
            pi_reset(&right_pi);
            startup_cycles = 0U;
            left_no_pulse_ms = 0U;
            right_no_pulse_ms = 0U;
        }

        duty_left = (int16_t)left;
        duty_right = (int16_t)right;
        duty_issued_ms = now.lo;
        duty_lease_ms = lease_ms;
        motion_mode = MOTION_MODE_DUTY;
        motion_state = MOTION_STATE_RUNNING;
    }

    (void)tk_unl_mtx(command_mutex);

    return er;
}

/*
 * Stop both motors.
 *
 * Zero PWM does not guarantee an immediate
 * physical stop because the wheels can coast.
 */
motion_result_t motion_stop(void)
{
    if (!motion_ready)
    {
        return MOTION_NOT_READY;
    }

    if (tk_loc_mtx(command_mutex, MOTION_LOCK_WAIT_MS) != E_OK)
    {
        return MOTION_BUSY;
    }

    motion_stop_locked();
    (void)tk_unl_mtx(command_mutex);

    return MOTION_OK;
}

/*
 * Emergency stop: stop now and latch, see motion_control.h.
 */
motion_result_t motion_estop(void)
{
    if (!motion_ready)
    {
        return MOTION_NOT_READY;
    }

    if (tk_loc_mtx(command_mutex, MOTION_LOCK_WAIT_MS) != E_OK)
    {
        /* Never leave the motors running on a refused emergency stop:
           zero the PWM directly (motor_stop writes only the compare
           registers), then report it. */
        (void)motor_stop();
        return MOTION_BUSY;
    }

    motion_stop_locked();
    estop_latched = true;
    (void)tk_unl_mtx(command_mutex);

    return MOTION_OK;
}

/*
 * Release the latch. Motors stay stopped until the next accepted command.
 */
motion_result_t motion_clear_estop(void)
{
    if (!motion_ready)
    {
        return MOTION_NOT_READY;
    }

    if (tk_loc_mtx(command_mutex, MOTION_LOCK_WAIT_MS) != E_OK)
    {
        return MOTION_BUSY;
    }

    estop_latched = false;
    (void)tk_unl_mtx(command_mutex);

    return MOTION_OK;
}

bool motion_is_estopped(void)
{
    return estop_latched;
}

/*
 * Stop both motors and clear every command. Caller holds command_mutex.
 */
static void motion_stop_locked(void)
{
    if (motor_stop() != E_OK)
    {
        motion_enter_fault();
        return;
    }

    duty_left = 0;
    duty_right = 0;
    duty_lease_ms = 0U;
    motion_mode = MOTION_MODE_IDLE;

    target_left_rpm = 0.0f;
    target_right_rpm = 0.0f;

    pi_reset(&left_pi);
    pi_reset(&right_pi);

    startup_cycles = 0U;

    left_no_pulse_ms = 0U;
    right_no_pulse_ms = 0U;

    if (motion_state != MOTION_STATE_FAULT)
    {
        motion_state = MOTION_STATE_STOPPED;
    }
}

/*
 * Motion task, every MOTION_TICK_MS. If a command call holds the lock
 * at this instant, this tick is skipped (outputs unchanged).
 */
void motion_tick(void)
{
    SYSTIM now;

    if (!motion_ready)
    {
        return;
    }

    if (tk_loc_mtx(command_mutex, TMO_POL) != E_OK)
    {
        return;
    }

    if (tk_get_tim(&now) != E_OK)
    {
        motion_stop_locked();
    }
    else if (motion_mode == MOTION_MODE_DUTY)
    {
        motion_duty_tick(now.lo);
    }
    else if ((UW)(now.lo - last_pi_ms) >= MOTION_UPDATE_MS)
    {
        /* SPEED or IDLE: PI step (or just RPM) about every 100 ms */
        last_pi_ms = now.lo;
        motion_update();
    }
    else
    {
        /* between PI steps */
    }

    (void)tk_unl_mtx(command_mutex);
}

/*
 * One duty tick: lease check, one zero tick before any reversal, apply.
 */
static void motion_duty_tick(UW now_ms)
{
    int16_t left = duty_left;
    int16_t right = duty_right;

    if ((UW)(now_ms - duty_issued_ms) >= duty_lease_ms)
    {
        /* lease ran out: the caller stopped renewing */
        motion_stop_locked();
        applied_left = 0;
        applied_right = 0;
        return;
    }

    if (((left > 0) && (applied_left < 0)) ||
        ((left < 0) && (applied_left > 0)) ||
        ((right > 0) && (applied_right < 0)) ||
        ((right < 0) && (applied_right > 0)))
    {
        left = 0;
        right = 0;
    }

    if (motor_apply(left, right) != E_OK)
    {
        motion_enter_fault();
        return;
    }

    applied_left = left;
    applied_right = right;
}

/*
 * Return latest wheel-speed and encoder status.
 */
motion_result_t motion_get_status(
    motion_status_t *status)
{
    if (status == 0)
    {
        return MOTION_ERROR;
    }

    if (!motion_ready)
    {
        return MOTION_NOT_READY;
    }

    status->left_rpm = actual_left_rpm;
    status->right_rpm = actual_right_rpm;

    encoder_get_counts(
        &status->left_encoder_count,
        &status->right_encoder_count
    );

    status->state = motion_state;

    return MOTION_OK;
}

/*
 * Periodic speed-control update.
 *
 * Intended call interval: 100 ms.
 *
 * Encoder sampling must occur independently
 * at a much faster rate.
 */
void motion_update(void)
{
    int32_t left_count;
    int32_t right_count;

    int32_t delta_left;
    int32_t delta_right;

    float left_pwm = 0.0f;
    float right_pwm = 0.0f;

    ER er;

    if (!motion_ready)
    {
        return;
    }

    /*
     * Obtain encoder counts and calculate the
     * number of counts observed since last update.
     */
    encoder_get_counts(
        &left_count,
        &right_count
    );

    delta_left = left_count - previous_left_count;
    delta_right = right_count - previous_right_count;

    previous_left_count = left_count;
    previous_right_count = right_count;

    /*
     * Convert count changes into wheel RPM.
     */
    actual_left_rpm = encoder_calculate_left_rpm(
        delta_left,
        MOTION_UPDATE_MS
    );

    actual_right_rpm = encoder_calculate_right_rpm(
        delta_right,
        MOTION_UPDATE_MS
    );

    if (motion_state != MOTION_STATE_RUNNING)
    {
        return;
    }

    /*
     * Encoder feedback watchdog.
     *
     * Only monitor wheels with a positive
     * target speed.
     */
    if (target_left_rpm > 0.0f)
    {
        if (delta_left == 0)
        {
            if (left_no_pulse_ms < MOTION_ENCODER_TIMEOUT_MS)
            {
                left_no_pulse_ms += MOTION_UPDATE_MS;
            }
        }
        else
        {
            left_no_pulse_ms = 0U;
        }
    }
    else
    {
        left_no_pulse_ms = 0U;
    }

    if (target_right_rpm > 0.0f)
    {
        if (delta_right == 0)
        {
            if (right_no_pulse_ms < MOTION_ENCODER_TIMEOUT_MS)
            {
                right_no_pulse_ms += MOTION_UPDATE_MS;
            }
        }
        else
        {
            right_no_pulse_ms = 0U;
        }
    }
    else
    {
        right_no_pulse_ms = 0U;
    }

    /*
     * Enter FAULT if either moving wheel
     * has stopped providing feedback.
     */
    if ((left_no_pulse_ms >= MOTION_ENCODER_TIMEOUT_MS) ||
        (right_no_pulse_ms >= MOTION_ENCODER_TIMEOUT_MS))
    {
        motion_enter_fault();
        return;
    }

    /*
     * Preliminary startup PWM boost.
     *
     * Runs for two 100 ms update cycles.
     */
    if (startup_cycles > 0U)
    {
        er = motor_set_left_pwm(
            target_left_rpm > 0.0f ?
            MOTION_START_PWM : 0U
        );

        if (er != E_OK)
        {
            motion_enter_fault();
            return;
        }

        er = motor_set_right_pwm(
            target_right_rpm > 0.0f ?
            MOTION_START_PWM : 0U
        );

        if (er != E_OK)
        {
            motion_enter_fault();
            return;
        }

        startup_cycles--;

        return;
    }

    /*
     * Left-wheel PI speed control.
     */
    if (target_left_rpm > 0.0f)
    {
        left_pwm = MOTION_BASE_PWM + pi_update(
            &left_pi,
            target_left_rpm,
            actual_left_rpm,
            MOTION_DT_SECONDS
        );
    }
    else
    {
        pi_reset(&left_pi);
    }

    /*
     * Right-wheel PI speed control.
     */
    if (target_right_rpm > 0.0f)
    {
        right_pwm = MOTION_BASE_PWM + pi_update(
            &right_pi,
            target_right_rpm,
            actual_right_rpm,
            MOTION_DT_SECONDS
        );
    }
    else
    {
        pi_reset(&right_pi);
    }

    /*
     * Apply PWM outputs.
     */
    er = motor_set_left_pwm(
        motion_pwm_value(left_pwm)
    );

    if (er != E_OK)
    {
        motion_enter_fault();
        return;
    }

    er = motor_set_right_pwm(
        motion_pwm_value(right_pwm)
    );

    if (er != E_OK)
    {
        motion_enter_fault();
    }
}

/*
 * Navigation APIs not implemented yet.
 */
motion_result_t motion_move_forward(float distance_cm)
{
    (void)distance_cm;
    return MOTION_NOT_IMPLEMENTED;
}

motion_result_t motion_move_backward(float distance_cm)
{
    (void)distance_cm;
    return MOTION_NOT_IMPLEMENTED;
}

motion_result_t motion_turn_left(float angle_deg)
{
    (void)angle_deg;
    return MOTION_NOT_IMPLEMENTED;
}

motion_result_t motion_turn_right(float angle_deg)
{
    (void)angle_deg;
    return MOTION_NOT_IMPLEMENTED;
}

/*
 * IF-10: see motion_control.h.
 */
int32_t motion_get_distance_mm(void)
{
    return encoder_get_distance_mm();
}
