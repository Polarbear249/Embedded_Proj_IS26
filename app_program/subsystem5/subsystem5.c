/*
 * subsystem5.c
 * Subsystem 5 (Buddy 5, Daryl): servo ultrasonic scanning and obstacle
 * profiling, see subsystem5.h.
 *
 * subsystem5_init() runs after subsystem2_init() (PWM block out of
 * reset) and on core 0 (gpio_irq_attach() rule). Kernel objects: none of
 * its own; the profile goes to the VC's message buffer vc_mbf_profile
 * (the receiver owns the object, TEAM.md section 5).
 */

#include "subsystem5.h"
#include "obstacle.h"
#include "ultrasonic.h"
#include "servo.h"

ER
subsystem5_init (void)
{
    ER er = ultrasonic_init();

    if (E_OK <= er)
    {
        er = servo_init();
    }

    return (er);
}

// Pinned to core 0, where the ECHO callback runs (ultrasonic.c)
ER
subsystem5_start (void)
{
    ID tid = app_start_task(obstacle_task, S5_PRI_MAIN, S5_PRC_MAIN,
                            "obstacle_task");

    return ((tid < E_OK) ? (ER)tid : E_OK);
}
