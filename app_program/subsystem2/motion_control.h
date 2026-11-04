
#ifndef MOTION_CONTROL_H
#define MOTION_CONTROL_H

#include <stdint.h>
#include <tk/tkernel.h>
#include <stdbool.h>

/*
 * Buddy 2 - Shared Motion Control API
 *
 * Other subsystems use this interface to request movement.
 * Only Buddy 2 controls the motor PWM and encoder hardware.
 *
 * Provides wheel speed requests, line-drive speed and duty modes, encoder
 * distance moves and turns, stopping, encoder and speed feedback, and the
 * IF-01 Q_motion / IF-02 EVG_stop objects (motion_if.c).
 *
 * Only one task should command the motors at a time.
 */

/* Motion command results */
typedef enum
{
    MOTION_OK = 0,
    MOTION_ERROR,
    MOTION_NOT_READY,
    MOTION_NOT_IMPLEMENTED,
    MOTION_BUSY
} motion_result_t;

/* Current motion state */
typedef enum
{
    MOTION_STATE_STOPPED = 0,
    MOTION_STATE_RUNNING,
    MOTION_STATE_FAULT
} motion_state_t;

/* Current motor and encoder information */
typedef struct
{
    float left_rpm;
    float right_rpm;

    int32_t left_encoder_count;
    int32_t right_encoder_count;

    motion_state_t state;
} motion_status_t;

/*
 * Initialise the motion-control hardware.
 *
 * Called once by Buddy 2 during subsystem initialisation.
 * Other buddies must not call this function.
 */
motion_result_t motion_init(void);

/*
 * Request independent left and right wheel speeds.
 *
 * Units: RPM. Forward only:
 *   left_rpm  >= 0
 *   right_rpm >= 0
 *
 * Stays in force until replaced: no lease, so the caller must send the
 * stop itself. Returns MOTION_BUSY while a distance move or turn runs.
 */
motion_result_t motion_set_speed(float left_rpm,
                                 float right_rpm);

/*
 * Line following in PI speed mode (the chosen drive API for S3).
 *
 * left_rpm / right_rpm: -30..+30, negative = reverse, so the car can spin.
 * A wheel under 5 RPM in size is stopped. lease_ms: 1..MOTION_MAX_LEASE_MS,
 * renewed by the caller; the motors stop when it runs out, or when
 * line_state goes stale or reports FAILED (IF-06). Encoder stall and
 * wrong-direction watchdogs are active. PI runs about every 100 ms, so
 * steering changes take effect at that rate.
 */
motion_result_t motion_line_drive(float left_rpm, float right_rpm,
                                  UINT lease_ms);

/*
 * Request both motors to stop.
 *
 * A zero-PWM stop is not the same as a guaranteed
 * immediate physical stop.
 */
motion_result_t motion_stop(void);

/*
 * Emergency stop (added 2026-11-02 for the GP20 stop button, same idea
 * as main's 59da21b motion_estop). Stops both motors and LATCHES: while
 * latched every drive command is refused (motion_set_duty -> E_OBJ,
 * motion_set_speed -> MOTION_ERROR), so a task that keeps renewing
 * commands (S3's line drive, every 10 ms) cannot restart the motors.
 * Also sets EVG_stop, so a busy command lock cannot lose the stop.
 * motion_clear_estop() releases the latch AND any FAULT, but does not
 * restart anything; the next accepted command does. Any task, any core.
 */
motion_result_t motion_estop(void);
motion_result_t motion_clear_estop(void);
bool            motion_is_estopped(void);

/*
 * Copy the latest available motor and encoder status.
 *
 * Status values are snapshots and may change as the
 * motion-control task executes.
 */
motion_result_t motion_get_status(motion_status_t *status);

/*
 * IF-10: cumulative distance travelled in mm since power-up
 * (mean of both wheels, forward positive). Updated about
 * every 10 ms by the encoder task. Never blocks; safe from
 * any task on either core. Used by S4 for hump height.
 */
int32_t motion_get_distance_mm(void);

/*
 * Encoder distance moves and turns (spin on the spot).
 *
 * Each returns at once. Poll motion_get_status(): state goes RUNNING ->
 * STOPPED when the move ends, or FAULT if a wheel stalled. While one
 * runs, other drive commands get MOTION_BUSY / E_OBJ; motion_stop() and
 * motion_estop() cancel it. MOTION_ERROR: bad range (distance 0 < d <=
 * 1000 cm, angle 0 < a <= 360 deg), FAULT or e-stop latched.
 * Duties and turn scale are untuned starting values (motion_move.h).
 */
/* Floor-tuned by eye, 2026-11-04: 180 stopped short of a real half turn.
   Goes with MOVE_TURN_SCALE_PCT, so change them together. */
#define MOTION_U_TURN_DEG       (182.0f)

motion_result_t motion_move_forward(float distance_cm);
motion_result_t motion_move_backward(float distance_cm);
motion_result_t motion_turn_left(float angle_deg);
motion_result_t motion_turn_right(float angle_deg);
motion_result_t motion_u_turn(void);

/*
 * IF-05 fault bits (TELEM_FAULT_MOTOR_STALL, TELEM_FAULT_ENCODER) for the
 * current FAULT, 0 if none. Any task. g_fault_mask's owner reads this;
 * S2 does not write g_fault_mask itself until the team settles the owner.
 */
uint32_t motion_get_fault_bits(void);

/*
 * Diagnostics for the Week 13 evidence: worst-case motion_tick() time and
 * period (us), and counts of each safe stop and of refused queue commands.
 */
typedef struct
{
    uint32_t tick_exec_max_us;
    uint32_t tick_period_max_us;
    uint32_t lease_stops;
    uint32_t line_stale_stops;
    uint32_t queue_rejected;
} motion_diag_t;

void motion_get_diag(motion_diag_t *p_diag);

/*
 * IF-01 Q_motion: non-blocking send of a motion_cmd_pack() word, see
 * motion_cmd.h. E_TMOUT = queue full. IF-02 EVG_stop: motion_request_stop()
 * latches the same stop as motion_estop(), acted on within one tick and
 * outranking Q_motion; EVG_STOPPED is set once the encoders confirm the
 * wheels are still. Release with motion_clear_estop().
 */
#include "motion_cmd.h"

#define MOTION_QUEUE_DEPTH      (4)
#define MOTION_Q_LEASE_MS       (500U)      /* DUTY and SPEED over Q_motion */
#define EVG_STOP_REQ            (0x01U)
#define EVG_STOPPED             (0x02U)

IMPORT ID Q_motion;
IMPORT ID EVG_stop;

ER   motion_send(UW cmd);
ER   motion_request_stop(void);

/* Buddy 2's motion code only */
ER   motion_if_init(void);
BOOL motion_if_stop_requested(void);
BOOL motion_if_receive(UW *p_cmd);
void motion_if_report_stopped(BOOL stopped);
ER   motion_if_clear_stop(void);

/*
 * Open-loop duty mode (added 2026-11-02 for S3's line follower, same
 * contract as main's 59da21b motion_set_duty). Kept as the fallback:
 * motion_line_drive() is the chosen API, because duty mode stalled on the
 * 10 mm ramp (INTEGRATION.md problem 16). Also stops on stale line_state.
 *
 * left / right: -MOTION_MAX_DUTY..+MOTION_MAX_DUTY, 1000 = 100 %,
 * positive = forward, negative = reverse (so the car can spin).
 * lease_ms: 1..MOTION_MAX_LEASE_MS. The motion task stops the motors
 * when the lease runs out, so the caller renews it (e.g. every 10 ms
 * with a 100 ms lease) and a stalled caller cannot leave them running.
 * Latest command wins over motion_set_speed() and vice versa.
 * Returns E_OK, E_PAR (bad range), E_OBJ (not ready or FAULT) or
 * E_TMOUT (command lock busy for an instant: retry next cycle).
 * No encoder stall watchdog in this mode.
 */
#define MOTION_TICK_MS          (10)        /* motion task period */
#define MOTION_MAX_DUTY         (200)
#define MOTION_MAX_LEASE_MS     (1000U)

ER motion_set_duty(INT left, INT right, UINT lease_ms);

/*
 * Called every MOTION_TICK_MS by Buddy 2's motion-control task only:
 * obeys EVG_stop and Q_motion, runs one duty or move step, and runs
 * motion_update() (RPM, watchdogs, PI) about every 100 ms.
 */
void motion_tick(void);

/*
 * RPM, watchdog and PI step, called by motion_tick(). Not for other
 * subsystems.
 */
void motion_update(void);
#endif /* MOTION_CONTROL_H */
