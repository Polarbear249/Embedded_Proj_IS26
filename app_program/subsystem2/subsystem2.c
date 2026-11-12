
/*
 * subsystem2.c
 * Buddy 2 - Motion Control Subsystem
 *
 * Initial micro T-Kernel integration.
 *
 * Responsibilities:
 * - Initialise motor and encoder hardware
 * - Start encoder sampling task
 * - Start motion control task
 * - Run PI speed control periodically
 */

#include "subsystem2.h"
#include "motion_control.h"
#include "encoder.h"

/*
 * Encoder Publish Task
 *
 * The encoder counts now come from GPIO edge interrupts
 * (encoder.c), so this task no longer samples pins. It only
 * publishes the IF-10 distance word every S2_DIST_PERIOD_MS.
 */
static void encoder_publish_task(INT stacd, void *exinf)
{
    (void)stacd;
    (void)exinf;

    for (;;)
    {
        encoder_publish_distance();

        tk_dly_tsk(S2_DIST_PERIOD_MS);
    }
}

/*
 * Motion Control Task
 *
 * Every MOTION_TICK_MS (10 ms): motion_tick() applies
 * duty commands (S3 line follower) and runs the PI
 * speed controller about every 100 ms.
 *
 * Hardware is initialised in subsystem2_init()
 * before either task starts.
 */
static void motion_control_task(INT stacd, void *exinf)
{
    (void)stacd;
    (void)exinf;

    for (;;)
    {
        motion_tick();

        tk_dly_tsk(MOTION_TICK_MS);
    }
}

/*
 * Initialise Subsystem 2.
 *
 * Motor and encoder hardware are initialised
 * before the two tasks are started.
 */
EXPORT ER subsystem2_init(void)
{
    /* Initialise motor and encoder hardware */
    if (motion_init() != MOTION_OK)
    {
        return E_SYS;
    }

    return E_OK;
}

/*
 * Start Subsystem 2 tasks.
 *
 * Both tasks are pinned to core 1 (TP_PRC2).
 *
 * Encoder publish task priority: S2_PRI_ENCODER (5)
 * Motion task priority: S2_PRI_MOTION (5), TEAM.md section 4
 *
 * Smaller priority numbers indicate higher
 * scheduling priority in micro T-Kernel.
 */
EXPORT ER subsystem2_start(void)
{
    ID encoder_tid;
    ID motion_tid;

    /* Start the IF-10 distance publish task */
    encoder_tid = app_start_task(
        encoder_publish_task,
        S2_PRI_ENCODER,
        TP_PRC2,
        "encoder_publish_task"
    );

    if (encoder_tid < E_OK)
    {
        return encoder_tid;
    }

    /* Start the motion-control task */
    motion_tid = app_start_task(
        motion_control_task,
        S2_PRI_MOTION,
        TP_PRC2,
        "motion_control_task"
    );

    if (motion_tid < E_OK)
    {
        return motion_tid;
    }

    return E_OK;
}
