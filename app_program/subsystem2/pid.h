
#ifndef PID_H
#define PID_H

/*
 * Buddy 2 - PI Wheel Speed Controller
 *
 * Uses a proportional-integral controller.
 * Derivative control is not currently used.
 */

typedef struct
{
    float kp;
    float ki;

    float integral;
    float output_min;
    float output_max;
} pi_controller_t;

/* Configure controller gains and output limits. */
void pi_init(
    pi_controller_t *controller,
    float kp,
    float ki,
    float output_min,
    float output_max);

/* Reset accumulated integral error. */
void pi_reset(pi_controller_t *controller);

/* Calculate the next controller output. */
float pi_update(
    pi_controller_t *controller,
    float target_rpm,
    float actual_rpm,
    float dt_seconds);

#endif /* PID_H */
