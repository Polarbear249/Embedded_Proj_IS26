
#include "motion_control.h"

#include <sys/machine.h>

#include "motor.h"
#include "encoder.h"
#include "pid.h"
#include "motion_move.h"
#include "../subsystem1/telemetry_frame.h"  /* TELEM_FAULT_* (IF-05 bits) */
#include "../subsystem3/subsystem3.h"       /* IF-06 s3_line_state_read() */

#include <stdint.h>

/*
 * Buddy 2 - Motion Control Coordinator
 *
 * Execution model:
 * - One task (motion_control_task) owns the motors. motion_tick() runs
 *   every MOTION_TICK_MS: it obeys EVG_stop, reads Q_motion, checks the
 *   lease and line_state, runs one move or duty step, and calls
 *   motion_update() about every 100 ms (RPM, watchdogs, PI speed control).
 * - Encoder counts come from GPIO edge interrupts.
 * - Commands from other tasks only store the request under command_mutex;
 *   the motion task reads it under the same mutex.
 *
 * Modes: SPEED (PI), DUTY (open loop, with a lease), MOVE (encoder
 * distance or turn). Between SPEED and DUTY the latest command wins. A
 * MOVE refuses other drive commands until it ends; motion_stop(),
 * motion_estop() and EVG_stop always cancel it.
 */

#define MOTION_UPDATE_MS          100U

/* Preliminary bench-test settings */
#define MOTION_KP                 6.0f      /* was 2: steering took seconds to show, line follow run1 */
#define MOTION_KI                 6.0f      /* was 2 */

#define MOTION_BASE_PWM           85.0f
#if MOTOR_TEST_HIGH_POWER
#define MOTION_MAX_PWM            500.0f    /* test only, see motor.h */
#else
#define MOTION_MAX_PWM            200.0f
#endif

#if MOTOR_TEST_HIGH_POWER
#define MOTION_START_PWM          250       /* test only, see motor.h; lowered from 350 for a start jerk, not yet tried */
#else
#define MOTION_START_PWM          150
#endif
#define MOTION_START_CYCLES       2U

#define MOTION_MAX_TARGET_RPM     30.0f

/* Line drive only: a wheel asked for less than this is stopped, because
   the PI cannot hold it and the watchdog would call it a stall. */
#define MOTION_MIN_RPM            5.0f

/* Distance and turn duties: STARTING VALUES, tune on the car. */
#if MOTOR_TEST_HIGH_POWER
#define MOTION_MOVE_KICK          300
#define MOTION_MOVE_CRUISE        200
#define MOTION_MOVE_CREEP         140
#else
#define MOTION_MOVE_KICK          200
#define MOTION_MOVE_CRUISE        160
#define MOTION_MOVE_CREEP         110
#endif

#if MOTOR_TEST_HIGH_POWER
#define MOTION_TURN_KICK          400
#define MOTION_TURN_CRUISE        300
#define MOTION_TURN_CREEP         260
#else
#define MOTION_TURN_KICK          300
#define MOTION_TURN_CRUISE        240
#define MOTION_TURN_CREEP         200
#endif

#define MOTION_MAX_MOVE_CM        1000.0f
#define MOTION_MAX_TURN_DEG       360.0f
#define MOTION_MM_PER_CM          10.0f
#define MOTION_MM_S_TO_RPM        (60.0f / 207.345f)
#define MOTION_MS_PER_MIN         60000.0f

/*
 * Encoder feedback watchdogs (speed mode; preliminary thresholds).
 * No count for 1000 ms on a commanded wheel: stall. Counts against the
 * commanded direction for 500 ms: encoder fault.
 */
#define MOTION_ENCODER_TIMEOUT_MS 1000U
#define MOTION_WRONG_DIR_MS       500U
#define MOTION_WRONG_DIR_COUNTS   2

/* Command lock: callers wait at most this long (the motion task holds
   it only for one tick's work, tens of us). TMO_POL for duty commands,
   which S3 renews every 10 ms anyway. */
#define MOTION_LOCK_WAIT_MS       (10)

#define MOTION_NO_LEASE           0U

typedef enum
{
    MOTION_MODE_IDLE = 0,
    MOTION_MODE_SPEED,
    MOTION_MODE_DUTY,
    MOTION_MODE_MOVE
} motion_mode_t;

static ID            command_mutex = 0;
static motion_mode_t motion_mode = MOTION_MODE_IDLE;
static int16_t       duty_left = 0;
static int16_t       duty_right = 0;
static int16_t       applied_left = 0;      /* motion task only */
static int16_t       applied_right = 0;
static UW            lease_issued_ms = 0U;  /* DUTY and leased SPEED */
static UW            lease_ms = MOTION_NO_LEASE;
static UW            last_pi_ms = 0U;       /* motion task only */
static UW            last_tick_ms = 0U;     /* motion task only */
static UW            update_interval_ms = MOTION_UPDATE_MS;

/* IF-06: stop if S3 stops publishing while it is steering the car. */
static bool             line_guarded = false;
static line_if_reader_t line_reader;

/* Encoder move in progress (MOTION_MODE_MOVE) */
static move_t  active_move;
static int32_t move_base_left = 0;
static int32_t move_base_right = 0;

/* Emergency-stop latch: read without the lock by motion_is_estopped()
   (one byte, a whole value either way). */
static volatile bool estop_latched = false;

/* IF-05 bits for the last fault, see motion_get_fault_bits(). */
static volatile uint32_t fault_bits = 0U;

static uint32_t left_no_pulse_ms = 0U;
static uint32_t right_no_pulse_ms = 0U;
static uint32_t left_wrong_ms = 0U;
static uint32_t right_wrong_ms = 0U;

static int motion_ready = 0;

static motion_state_t motion_state =
    MOTION_STATE_STOPPED;

static float target_left_rpm = 0.0f;
static float target_right_rpm = 0.0f;

static float actual_left_rpm = 0.0f;
static float actual_right_rpm = 0.0f;

static int32_t previous_left_count = 0;
static int32_t previous_right_count = 0;

static uint32_t left_kick_cycles = 0U;
static uint32_t right_kick_cycles = 0U;

static pi_controller_t left_pi;
static pi_controller_t right_pi;

static motion_diag_t diag = {0U, 0U, 0U, 0U, 0U};
static uint32_t      tick_start_us = 0U;

static void motion_stop_locked(void);
static void motion_duty_tick(void);
static void motion_move_tick(UW dt_ms);
static void motion_process_queue(void);
static void motion_apply_command(UW cmd);
static bool motion_line_is_stale(void);

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

static float motion_abs(float value)
{
    return (value < 0.0f) ? -value : value;
}

/* Both zero tests are exact: targets are stored, never computed. */
static bool motion_opposite(float a, float b)
{
    return ((a > 0.0f) && (b < 0.0f)) || ((a < 0.0f) && (b > 0.0f));
}

static void motion_clear_targets(void)
{
    target_left_rpm = 0.0f;
    target_right_rpm = 0.0f;

    pi_reset(&left_pi);
    pi_reset(&right_pi);

    left_kick_cycles = 0U;
    right_kick_cycles = 0U;

    left_no_pulse_ms = 0U;
    right_no_pulse_ms = 0U;
    left_wrong_ms = 0U;
    right_wrong_ms = 0U;
}

/*
 * Enter fault state and stop both motors. The mode goes to IDLE so that
 * no later tick can drive again. Cleared by motion_clear_estop().
 */
static void motion_enter_fault(uint32_t bits)
{
    (void)motor_stop();

    applied_left = 0;
    applied_right = 0;
    lease_ms = MOTION_NO_LEASE;
    line_guarded = false;
    motion_mode = MOTION_MODE_IDLE;

    motion_clear_targets();

    fault_bits |= bits;
    motion_state = MOTION_STATE_FAULT;
}

/*
 * Initialise motors, encoders, PI controllers and the IF-01 / IF-02
 * kernel objects.
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

    if (motion_if_init() != E_OK)
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

    motion_clear_targets();

    actual_left_rpm = 0.0f;
    actual_right_rpm = 0.0f;

    motion_state = MOTION_STATE_STOPPED;
    motion_ready = 1;

    return MOTION_OK;
}

/*
 * Give one wheel a new speed target. A wheel starting from rest or
 * changing direction gets the start kick and a fresh integral.
 */
static void motion_retarget_wheel(float target, float *p_target,
                                  uint32_t *p_kick, pi_controller_t *p_pi,
                                  uint32_t *p_no_pulse, uint32_t *p_wrong)
{
    if (target == 0.0f)
    {
        *p_kick = 0U;
        *p_no_pulse = 0U;
        *p_wrong = 0U;
        pi_reset(p_pi);
    }
    else if ((*p_target == 0.0f) || motion_opposite(target, *p_target))
    {
        *p_kick = MOTION_START_CYCLES;
        *p_no_pulse = 0U;
        *p_wrong = 0U;
        pi_reset(p_pi);
    }
    else
    {
        /* same direction: keep the PI state */
    }

    *p_target = target;
}

static void motion_guard_line(bool b_guard)
{
    if (b_guard && !line_guarded)
    {
        line_if_reader_init(&line_reader, s3_line_state_read());
    }

    line_guarded = b_guard;
}

/*
 * Common part of every speed command. Ranges are already checked.
 * Stores the request; the motion task acts on it.
 */
static motion_result_t motion_speed_request(float left_rpm, float right_rpm,
                                            UINT lease, bool b_guard)
{
    SYSTIM now = {0, 0U};

    if (tk_loc_mtx(command_mutex, MOTION_LOCK_WAIT_MS) != E_OK)
    {
        return MOTION_BUSY;
    }

    if (motion_state == MOTION_STATE_FAULT)
    {
        (void)tk_unl_mtx(command_mutex);
        return MOTION_ERROR;
    }

    if ((left_rpm == 0.0f) && (right_rpm == 0.0f))
    {
        motion_stop_locked();
        (void)tk_unl_mtx(command_mutex);
        return MOTION_OK;
    }

    if (estop_latched || (tk_get_tim(&now) != E_OK))
    {
        (void)tk_unl_mtx(command_mutex);
        return MOTION_ERROR;
    }

    if (motion_mode == MOTION_MODE_MOVE)
    {
        (void)tk_unl_mtx(command_mutex);
        return MOTION_BUSY;
    }

    if ((motion_state == MOTION_STATE_STOPPED) ||
        (motion_mode != MOTION_MODE_SPEED))
    {
        motion_clear_targets();

        encoder_get_counts(
            &previous_left_count,
            &previous_right_count
        );
    }

    motion_retarget_wheel(left_rpm, &target_left_rpm, &left_kick_cycles,
                          &left_pi, &left_no_pulse_ms, &left_wrong_ms);
    motion_retarget_wheel(right_rpm, &target_right_rpm, &right_kick_cycles,
                          &right_pi, &right_no_pulse_ms, &right_wrong_ms);

    lease_issued_ms = now.lo;
    lease_ms = lease;
    motion_guard_line(b_guard);

    motion_mode = MOTION_MODE_SPEED;
    motion_state = MOTION_STATE_RUNNING;

    (void)tk_unl_mtx(command_mutex);

    return MOTION_OK;
}

/*
 * Request independent forward wheel speeds, valid until replaced.
 *
 * Valid range: 0 to 30 RPM per wheel.
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

    return motion_speed_request(left_rpm, right_rpm, MOTION_NO_LEASE, false);
}

static bool motion_rpm_in_range(float rpm)
{
    return (rpm >= -MOTION_MAX_TARGET_RPM) && (rpm <= MOTION_MAX_TARGET_RPM);
}

/*
 * Line following in PI speed mode, see motion_control.h.
 */
motion_result_t motion_line_drive(float left_rpm, float right_rpm,
                                  UINT lease)
{
    if (!motion_ready)
    {
        return MOTION_NOT_READY;
    }

    /* The range test also rejects NaN. */
    if (!motion_rpm_in_range(left_rpm) || !motion_rpm_in_range(right_rpm) ||
        (lease == 0U) || (lease > MOTION_MAX_LEASE_MS))
    {
        return MOTION_ERROR;
    }

    if (motion_abs(left_rpm) < MOTION_MIN_RPM)
    {
        left_rpm = 0.0f;
    }

    if (motion_abs(right_rpm) < MOTION_MIN_RPM)
    {
        right_rpm = 0.0f;
    }

    return motion_speed_request(left_rpm, right_rpm, lease, true);
}

/*
 * Open-loop signed duty with a lease. b_guard: also stop on stale
 * line_state.
 */
static ER motion_duty_request(INT left, INT right, UINT lease, bool b_guard)
{
    SYSTIM now;
    ER     er;

    if ((left < -MOTION_MAX_DUTY) || (left > MOTION_MAX_DUTY) ||
        (right < -MOTION_MAX_DUTY) || (right > MOTION_MAX_DUTY) ||
        (lease == 0U) || (lease > MOTION_MAX_LEASE_MS))
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

    if ((motion_state == MOTION_STATE_FAULT) || estop_latched ||
        (motion_mode == MOTION_MODE_MOVE))
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
            motion_clear_targets();
        }

        duty_left = (int16_t)left;
        duty_right = (int16_t)right;
        lease_issued_ms = now.lo;
        lease_ms = lease;
        motion_guard_line(b_guard);
        motion_mode = MOTION_MODE_DUTY;
        motion_state = MOTION_STATE_RUNNING;
    }

    (void)tk_unl_mtx(command_mutex);

    return er;
}

/*
 * Duty mode for S3's line follower, see motion_control.h.
 */
ER motion_set_duty(INT left, INT right, UINT lease)
{
    return motion_duty_request(left, right, lease, true);
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
 * Emergency stop: latch first, then stop, see motion_control.h. The latch
 * and EVG_stop are set before asking for the lock, so a busy lock cannot
 * lose the stop: the next motion_tick() acts on the flag.
 */
motion_result_t motion_estop(void)
{
    if (!motion_ready)
    {
        return MOTION_NOT_READY;
    }

    estop_latched = true;
    (void)motion_request_stop();

    if (tk_loc_mtx(command_mutex, MOTION_LOCK_WAIT_MS) != E_OK)
    {
        (void)motor_stop();
        return MOTION_OK;
    }

    motion_stop_locked();
    (void)tk_unl_mtx(command_mutex);

    return MOTION_OK;
}

/*
 * Release the latch and any fault. Motors stay stopped until the next
 * accepted command.
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

    (void)motion_if_clear_stop();
    estop_latched = false;
    fault_bits = 0U;

    if (motion_state == MOTION_STATE_FAULT)
    {
        motion_state = MOTION_STATE_STOPPED;
    }

    (void)tk_unl_mtx(command_mutex);

    return MOTION_OK;
}

bool motion_is_estopped(void)
{
    return estop_latched;
}

uint32_t motion_get_fault_bits(void)
{
    return fault_bits;
}

void motion_get_diag(motion_diag_t *p_diag)
{
    if (p_diag != 0)
    {
        *p_diag = diag;
    }
}

/*
 * Stop both motors and clear every command. Caller holds command_mutex.
 */
static void motion_stop_locked(void)
{
    if (motor_stop() != E_OK)
    {
        motion_enter_fault(TELEM_FAULT_MOTOR_STALL);
        return;
    }

    applied_left = 0;
    applied_right = 0;

    duty_left = 0;
    duty_right = 0;
    lease_ms = MOTION_NO_LEASE;
    line_guarded = false;
    motion_mode = MOTION_MODE_IDLE;

    motion_clear_targets();

    if (motion_state != MOTION_STATE_FAULT)
    {
        motion_state = MOTION_STATE_STOPPED;
    }
}

/*
 * Timing evidence for the Week 13 table, from the 1 MHz hardware timer.
 */
static void motion_timing_begin(void)
{
    uint32_t now_us = in_w(TIMER_TIMERAWL);
    uint32_t period_us = now_us - tick_start_us;

    if ((tick_start_us != 0U) && (period_us > diag.tick_period_max_us))
    {
        diag.tick_period_max_us = period_us;
    }

    tick_start_us = now_us;
}

static void motion_timing_end(void)
{
    uint32_t exec_us = in_w(TIMER_TIMERAWL) - tick_start_us;

    if (exec_us > diag.tick_exec_max_us)
    {
        diag.tick_exec_max_us = exec_us;
    }
}

/*
 * Motion task, every MOTION_TICK_MS. EVG_stop outranks everything and
 * does not depend on the lock. If a command call holds the lock at this
 * instant, the rest of this tick is skipped (outputs unchanged).
 */
void motion_tick(void)
{
    SYSTIM now;
    BOOL   b_stop;

    if (!motion_ready)
    {
        return;
    }

    motion_timing_begin();

    b_stop = motion_if_stop_requested();

    if (b_stop)
    {
        (void)motor_stop();
    }

    motion_process_queue();

    if (tk_loc_mtx(command_mutex, TMO_POL) != E_OK)
    {
        motion_timing_end();
        return;
    }

    /* Read again under the lock: motion_clear_estop() may have just run,
       and a stale reading here would latch the stop a second time. */
    if (b_stop && motion_if_stop_requested())
    {
        estop_latched = true;

        if (motion_mode != MOTION_MODE_IDLE)
        {
            motion_stop_locked();
        }
    }

    if (tk_get_tim(&now) != E_OK)
    {
        motion_stop_locked();
    }
    else
    {
        UW dt_ms = (UW)(now.lo - last_tick_ms);

        last_tick_ms = now.lo;

        if ((lease_ms != MOTION_NO_LEASE) &&
            ((UW)(now.lo - lease_issued_ms) >= lease_ms))
        {
            /* the caller stopped renewing */
            motion_stop_locked();
            diag.lease_stops++;
        }
        else if (line_guarded && motion_line_is_stale())
        {
            motion_stop_locked();
            diag.line_stale_stops++;
        }
        else
        {
            /* command still valid */
        }

        if ((UW)(now.lo - last_pi_ms) >= MOTION_UPDATE_MS)
        {
            update_interval_ms = (UW)(now.lo - last_pi_ms);
            last_pi_ms = now.lo;
            motion_update();
        }

        if (motion_mode == MOTION_MODE_DUTY)
        {
            motion_duty_tick();
        }
        else if (motion_mode == MOTION_MODE_MOVE)
        {
            motion_move_tick(dt_ms);
        }
        else
        {
            /* SPEED was handled by motion_update(); IDLE does nothing */
        }
    }

    (void)tk_unl_mtx(command_mutex);

    motion_timing_end();
}

/*
 * IF-06 guard (TEAM.md section 5: stale data -> safe stop). Not a fault:
 * if S3 recovers, its next command drives again. FAILED means S3 itself
 * gave up, so that stops the car as well.
 */
static bool motion_line_is_stale(void)
{
    uint32_t word = s3_line_state_read();

    return (!line_if_is_fresh(&line_reader, word)) ||
           (line_if_mode(word) == LINE_MODE_FAILED);
}

/*
 * Read Q_motion. Runs without the lock: each command takes it itself.
 */
static void motion_process_queue(void)
{
    UW cmd;

    while (motion_if_receive(&cmd))
    {
        motion_apply_command(cmd);
    }
}

static void motion_count_if_refused(bool b_accepted)
{
    if (!b_accepted)
    {
        diag.queue_rejected++;
    }
}

static void motion_apply_command(UW cmd)
{
    int32_t left = motion_cmd_left(cmd);
    int32_t right = motion_cmd_right(cmd);
    float   left_rpm = (float)left * MOTION_MM_S_TO_RPM;
    float   right_rpm = (float)right * MOTION_MM_S_TO_RPM;

    switch (motion_cmd_op(cmd))
    {
        case MOTION_OP_STOP:
            motion_count_if_refused(motion_stop() == MOTION_OK);
            break;

        case MOTION_OP_DUTY:
            motion_count_if_refused(
                motion_duty_request(left, right, MOTION_Q_LEASE_MS,
                                    false) == E_OK);
            break;

        case MOTION_OP_SPEED:
            motion_count_if_refused(
                motion_rpm_in_range(left_rpm) &&
                motion_rpm_in_range(right_rpm) &&
                (motion_speed_request(left_rpm, right_rpm, MOTION_Q_LEASE_MS,
                                      false) == MOTION_OK));
            break;

        case MOTION_OP_DISTANCE:
            motion_count_if_refused(
                (left != 0) &&
                (((left > 0) ? motion_move_forward((float)left / MOTION_MM_PER_CM)
                             : motion_move_backward((float)-left / MOTION_MM_PER_CM))
                 == MOTION_OK));
            break;

        case MOTION_OP_TURN:
            motion_count_if_refused(
                (left != 0) &&
                (((left > 0) ? motion_turn_right((float)left)
                             : motion_turn_left((float)-left)) == MOTION_OK));
            break;

        default:
            diag.queue_rejected++;
            break;
    }
}

/*
 * Apply duties, with one zero step before any wheel reverses.
 */
static void motion_apply_outputs(int16_t left, int16_t right)
{
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
        motion_enter_fault(TELEM_FAULT_MOTOR_STALL);
        return;
    }

    applied_left = left;
    applied_right = right;
}

static void motion_duty_tick(void)
{
    motion_apply_outputs(duty_left, duty_right);
}

/*
 * One step of an encoder distance move or turn.
 */
static void motion_move_tick(UW dt_ms)
{
    int32_t left;
    int32_t right;
    int16_t left_duty;
    int16_t right_duty;

    encoder_get_counts(&left, &right);

    switch (move_step(&active_move, left - move_base_left,
                      right - move_base_right, dt_ms, &left_duty, &right_duty))
    {
        case MOVE_DONE:
            motion_stop_locked();
            break;

        case MOVE_STALLED:
            motion_enter_fault(TELEM_FAULT_MOTOR_STALL);
            break;

        case MOVE_WRONG_WAY:
            motion_enter_fault(TELEM_FAULT_ENCODER);
            break;

        default:
            motion_apply_outputs(left_duty, right_duty);
            break;
    }
}

/*
 * Start an encoder move. Goals are signed wheel counts.
 */
static motion_result_t motion_move_request(int32_t goal_left,
                                           int32_t goal_right)
{
    bool b_turn = ((goal_left > 0) != (goal_right > 0));
    move_duty_t duty = b_turn
                        ? (move_duty_t){MOTION_TURN_KICK, MOTION_TURN_CRUISE,
                                        MOTION_TURN_CREEP}
                        : (move_duty_t){MOTION_MOVE_KICK, MOTION_MOVE_CRUISE,
                                        MOTION_MOVE_CREEP};

    if (!motion_ready)
    {
        return MOTION_NOT_READY;
    }

    if (tk_loc_mtx(command_mutex, MOTION_LOCK_WAIT_MS) != E_OK)
    {
        return MOTION_BUSY;
    }

    if ((motion_state == MOTION_STATE_FAULT) || estop_latched)
    {
        (void)tk_unl_mtx(command_mutex);
        return MOTION_ERROR;
    }

    if (motion_mode == MOTION_MODE_MOVE)
    {
        (void)tk_unl_mtx(command_mutex);
        return MOTION_BUSY;
    }

    motion_clear_targets();
    lease_ms = MOTION_NO_LEASE;
    line_guarded = false;

    encoder_get_counts(&move_base_left, &move_base_right);
    move_start(&active_move, goal_left, goal_right, duty);

    motion_mode = MOTION_MODE_MOVE;
    motion_state = MOTION_STATE_RUNNING;

    (void)tk_unl_mtx(command_mutex);

    return MOTION_OK;
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
 * Update one wheel's watchdog timers.
 */
static void motion_watch_wheel(float target, int32_t delta,
                               uint32_t *p_no_pulse, uint32_t *p_wrong)
{
    bool b_wrong = ((target > 0.0f) && (delta <= -MOTION_WRONG_DIR_COUNTS)) ||
                   ((target < 0.0f) && (delta >= MOTION_WRONG_DIR_COUNTS));

    if ((target != 0.0f) && (delta == 0))
    {
        if (*p_no_pulse < MOTION_ENCODER_TIMEOUT_MS)
        {
            *p_no_pulse += update_interval_ms;
        }
    }
    else
    {
        *p_no_pulse = 0U;
    }

    *p_wrong = b_wrong ? (*p_wrong + update_interval_ms) : 0U;
}

/*
 * One wheel's signed output in speed mode: zero for one cycle before a
 * reversal, then the start kick, then base + PI.
 */
static int16_t motion_wheel_output(float target, float actual,
                                   int16_t applied, uint32_t *p_kick,
                                   pi_controller_t *p_pi, float dt_seconds)
{
    bool    b_forward = target > 0.0f;
    int32_t size;

    if (target == 0.0f)
    {
        pi_reset(p_pi);
        return 0;
    }

    if ((applied != 0) && ((applied > 0) != b_forward))
    {
        return 0;
    }

    if (*p_kick > 0U)
    {
        (*p_kick)--;
        size = MOTION_START_PWM;
    }
    else
    {
        size = (int32_t)motion_pwm_value(
            MOTION_BASE_PWM + pi_update(
                p_pi,
                motion_abs(target),
                b_forward ? actual : -actual,
                dt_seconds));
    }

    /* Cast: size is at most MOTION_MAX_PWM (500) */
    return (int16_t)(b_forward ? size : -size);
}

/*
 * Periodic update, about every MOTION_UPDATE_MS, in every mode: wheel
 * RPM (duty and move modes need it too, for telemetry), the EVG_STOPPED
 * confirmation and, in speed mode, the watchdogs and PI control.
 *
 * Uses the interval actually measured, because a late tick would
 * otherwise read as a slower wheel.
 */
void motion_update(void)
{
    int32_t left_count;
    int32_t right_count;

    int32_t delta_left;
    int32_t delta_right;

    int16_t left_out;
    int16_t right_out;

    float dt_seconds;

    if (!motion_ready)
    {
        return;
    }

    encoder_get_counts(
        &left_count,
        &right_count
    );

    delta_left = left_count - previous_left_count;
    delta_right = right_count - previous_right_count;

    previous_left_count = left_count;
    previous_right_count = right_count;

    actual_left_rpm = encoder_calculate_left_rpm(
        delta_left,
        update_interval_ms
    );

    actual_right_rpm = encoder_calculate_right_rpm(
        delta_right,
        update_interval_ms
    );

    motion_if_report_stopped(
        ((motion_mode == MOTION_MODE_IDLE) &&
         (delta_left == 0) && (delta_right == 0)) ? TRUE : FALSE);

    if ((motion_mode != MOTION_MODE_SPEED) ||
        (motion_state != MOTION_STATE_RUNNING))
    {
        return;
    }

    motion_watch_wheel(target_left_rpm, delta_left,
                       &left_no_pulse_ms, &left_wrong_ms);
    motion_watch_wheel(target_right_rpm, delta_right,
                       &right_no_pulse_ms, &right_wrong_ms);

    if ((left_no_pulse_ms >= MOTION_ENCODER_TIMEOUT_MS) ||
        (right_no_pulse_ms >= MOTION_ENCODER_TIMEOUT_MS))
    {
        motion_enter_fault(TELEM_FAULT_MOTOR_STALL);
        return;
    }

    if ((left_wrong_ms >= MOTION_WRONG_DIR_MS) ||
        (right_wrong_ms >= MOTION_WRONG_DIR_MS))
    {
        motion_enter_fault(TELEM_FAULT_ENCODER);
        return;
    }

    dt_seconds = (float)update_interval_ms / 1000.0f;

    left_out = motion_wheel_output(target_left_rpm, actual_left_rpm,
                                   applied_left, &left_kick_cycles,
                                   &left_pi, dt_seconds);
    right_out = motion_wheel_output(target_right_rpm, actual_right_rpm,
                                    applied_right, &right_kick_cycles,
                                    &right_pi, dt_seconds);

    if (motor_apply(left_out, right_out) != E_OK)
    {
        motion_enter_fault(TELEM_FAULT_MOTOR_STALL);
        return;
    }

    applied_left = left_out;
    applied_right = right_out;
}

/*
 * Navigation. Each returns as soon as the move is accepted; poll
 * motion_get_status(): state goes RUNNING -> STOPPED when it ends, or
 * FAULT if a wheel stalled. The range tests also reject NaN.
 */
static bool motion_cm_in_range(float distance_cm)
{
    return (distance_cm > 0.0f) && (distance_cm <= MOTION_MAX_MOVE_CM);
}

static bool motion_deg_in_range(float angle_deg)
{
    return (angle_deg > 0.0f) && (angle_deg <= MOTION_MAX_TURN_DEG);
}

/* Casts: the range checks above bound these to well inside int32_t. */
static int32_t motion_cm_to_counts(float distance_cm, float cpr)
{
    return move_mm_to_counts((int32_t)((distance_cm * MOTION_MM_PER_CM *
                                        (float)MOVE_DIST_SCALE_PCT) / 100.0f),
                             (int32_t)cpr);
}

/* The left wheel's goal for a straight move, see MOVE_STRAIGHT_LEFT_PCT. */
static int32_t motion_cm_to_left_counts(float distance_cm)
{
    return (motion_cm_to_counts(distance_cm, ENCODER_LEFT_CPR) *
            MOVE_STRAIGHT_LEFT_PCT) / 100;
}

motion_result_t motion_move_forward(float distance_cm)
{
    if (!motion_cm_in_range(distance_cm))
    {
        return MOTION_ERROR;
    }

    return motion_move_request(
        motion_cm_to_left_counts(distance_cm),
        motion_cm_to_counts(distance_cm, ENCODER_RIGHT_CPR));
}

motion_result_t motion_move_backward(float distance_cm)
{
    if (!motion_cm_in_range(distance_cm))
    {
        return MOTION_ERROR;
    }

    return motion_move_request(
        -motion_cm_to_left_counts(distance_cm),
        -motion_cm_to_counts(distance_cm, ENCODER_RIGHT_CPR));
}

motion_result_t motion_turn_left(float angle_deg)
{
    if (!motion_deg_in_range(angle_deg))
    {
        return MOTION_ERROR;
    }

    return motion_move_request(
        -move_deg_to_counts((int32_t)angle_deg, (int32_t)ENCODER_LEFT_CPR),
        move_deg_to_counts((int32_t)angle_deg, (int32_t)ENCODER_RIGHT_CPR));
}

motion_result_t motion_turn_right(float angle_deg)
{
    if (!motion_deg_in_range(angle_deg))
    {
        return MOTION_ERROR;
    }

    return motion_move_request(
        move_deg_to_counts((int32_t)angle_deg, (int32_t)ENCODER_LEFT_CPR),
        -move_deg_to_counts((int32_t)angle_deg, (int32_t)ENCODER_RIGHT_CPR));
}

motion_result_t motion_u_turn(void)
{
    return motion_turn_left(MOTION_U_TURN_DEG);
}

/*
 * IF-10: see motion_control.h.
 */
int32_t motion_get_distance_mm(void)
{
    return encoder_get_distance_mm();
}
