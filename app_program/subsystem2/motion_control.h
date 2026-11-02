
#ifndef MOTION_CONTROL_H
#define MOTION_CONTROL_H

#include <stdint.h>
#include <tk/tkernel.h>

/*
 * Buddy 2 - Shared Motion Control API
 *
 * Other subsystems use this interface to request movement.
 * Only Buddy 2 controls the motor PWM and encoder hardware.
 *
 * Initial implementation:
 *   - Motor initialisation
 *   - Independent wheel speed requests
 *   - Motor stopping
 *   - Encoder and speed feedback
 *
 * Distance and turning functions will be implemented later.
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
 * Units: RPM
 * Initial version supports forward motion only:
 *   left_rpm  >= 0
 *   right_rpm >= 0
 *
 * The final implementation will process commands through
 * Buddy 2's µT-Kernel motion-control task.
 */
motion_result_t motion_set_speed(float left_rpm,
                                 float right_rpm);

/*
 * Request both motors to stop.
 *
 * A zero-PWM stop is not the same as a guaranteed
 * immediate physical stop.
 */
motion_result_t motion_stop(void);

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
 * Future navigation functions.
 *
 * These are part of the planned interface.
 * Until implemented, they must return
 * MOTION_NOT_IMPLEMENTED.
 */

/* Move forward by a requested distance in centimetres */
motion_result_t motion_move_forward(float distance_cm);

/* Move backward by a requested distance in centimetres */
motion_result_t motion_move_backward(float distance_cm);

/* Turn left by a requested angle in degrees */
motion_result_t motion_turn_left(float angle_deg);

/* Turn right by a requested angle in degrees */
motion_result_t motion_turn_right(float angle_deg);

/*
 * Open-loop duty mode (added 2026-11-02 for S3's line follower, same
 * contract as main's 59da21b motion_set_duty).
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
 * applies duty commands each tick and runs motion_update() (PI speed
 * control) about every 100 ms.
 */
void motion_tick(void);

/*
 * PI speed-control step, called by motion_tick(). Not for other
 * subsystems.
 */
void motion_update(void);
#endif /* MOTION_CONTROL_H */
