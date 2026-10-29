
#ifndef MOTION_CONTROL_H
#define MOTION_CONTROL_H

#include <stdint.h>

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
 * Called periodically by Buddy 2's motion-control task.
 * Not intended for other subsystems to call directly.
 */
void motion_update(void);
#endif /* MOTION_CONTROL_H */
