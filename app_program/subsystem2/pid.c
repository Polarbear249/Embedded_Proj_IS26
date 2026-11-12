
#include "pid.h"

/*
 * Buddy 2 - PI Speed Controller
 *
 * Controller output is a bounded PWM command.
 */

static float clamp_float(
    float value,
    float minimum,
    float maximum)
{
    if (value < minimum)
    {
        return minimum;
    }

    if (value > maximum)
    {
        return maximum;
    }

    return value;
}

void pi_init(
    pi_controller_t *controller,
    float kp,
    float ki,
    float output_min,
    float output_max)
{
    if (controller == 0)
    {
        return;
    }

    controller->kp = kp;
    controller->ki = ki;
    controller->integral = 0.0f;
    controller->output_min = output_min;
    controller->output_max = output_max;
}

void pi_reset(pi_controller_t *controller)
{
    if (controller == 0)
    {
        return;
    }

    controller->integral = 0.0f;
}

float pi_update(
    pi_controller_t *controller,
    float target_rpm,
    float actual_rpm,
    float dt_seconds)
{
    float error;
    float proposed_integral;
    float output;

    if ((controller == 0) || (dt_seconds <= 0.0f))
    {
        return 0.0f;
    }

    /* Difference between requested and measured RPM */
    error = target_rpm - actual_rpm;

    /* Calculate the integral contribution */
    proposed_integral =
        controller->integral + error * dt_seconds;

    output =
        controller->kp * error +
        controller->ki * proposed_integral;

    /*
     * Basic anti-windup:
     * Only accept an integral update when it does not
     * push an already saturated output further away
     * from the permitted range.
     */
    if (!((output > controller->output_max && error > 0.0f) ||
          (output < controller->output_min && error < 0.0f)))
    {
        controller->integral = proposed_integral;
    }

    output =
        controller->kp * error +
        controller->ki * controller->integral;

    return clamp_float(
        output,
        controller->output_min,
        controller->output_max);
}
