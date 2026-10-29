
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
 * Retain the original demonstration resources
 * temporarily for compatibility with other subsystems.
 */
EXPORT ID sem_slots;
EXPORT ID mpf_samples;

/*
 * Encoder Sampling Task
 *
 * Reads wheel encoder signals approximately every 1 ms.
 *
 * IMPORTANT:
 * Polling is provisional and may miss encoder edges.
 * An interrupt-based implementation is preferred
 * for reliable high-speed operation.
 */
static void encoder_sampling_task(INT stacd, void *exinf)
{
    (void)stacd;
    (void)exinf;

    for (;;)
    {
        encoder_sample();

        tk_dly_tsk(1);
    }
}

/*
 * Motion Control Task
 *
 * Calls the PI speed controller approximately
 * every 100 ms.
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
        motion_update();

        tk_dly_tsk(100);
    }
}

/*
 * Initialise Subsystem 2.
 *
 * Retains the original semaphore and memory pool
 * until the demonstration subsystem dependencies
 * have been removed by the team.
 *
 * Motor and encoder hardware are initialised
 * before the two tasks are started.
 */
EXPORT ER subsystem2_init(void)
{
    T_CSEM csem = {0};
    T_CMPF cmpf = {0};

    ER er;

    /* Create semaphore */
    csem.sematr = TA_TFIFO;
    csem.isemcnt = S2_POOL_COUNT;
    csem.maxsem = S2_POOL_COUNT;

    sem_slots = tk_cre_sem(&csem);

    if (sem_slots < E_OK)
    {
        return sem_slots;
    }

    /* Create fixed-size memory pool */
    cmpf.mpfatr = TA_TFIFO | TA_RNG3;
    cmpf.mpfcnt = S2_POOL_COUNT;
    cmpf.blfsz = sizeof(SAMPLE_MSG);

    mpf_samples = tk_cre_mpf(&cmpf);

    if (mpf_samples < E_OK)
    {
        return mpf_samples;
    }

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
 * Encoder task priority: 12
 * Motion task priority: S2_PRI_PRODUCER (14)
 *
 * Smaller priority numbers indicate higher
 * scheduling priority in micro T-Kernel.
 */
EXPORT ER subsystem2_start(void)
{
    ID encoder_tid;
    ID motion_tid;

    /* Start the higher-priority encoder sampling task */
    encoder_tid = app_start_task(
        encoder_sampling_task,
        12,
        TP_PRC2,
        "encoder_sampling_task"
    );

    if (encoder_tid < E_OK)
    {
        return encoder_tid;
    }

    /* Start the motion-control task */
    motion_tid = app_start_task(
        motion_control_task,
        S2_PRI_PRODUCER,
        TP_PRC2,
        "motion_control_task"
    );

    if (motion_tid < E_OK)
    {
        return motion_tid;
    }

    return E_OK;
}
