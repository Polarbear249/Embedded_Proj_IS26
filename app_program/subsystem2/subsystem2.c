
/*
 * subsystem2.c
 * Buddy 2 - Motion Control Subsystem
 *
 * Initial µT-Kernel integration.
 */

#include "subsystem2.h"
#include "motion_control.h"

/*
 * Retain existing shared sample objects temporarily
 * for compatibility with the demonstration subsystem.
 */
EXPORT ID sem_slots;
EXPORT ID mpf_samples;

/*
 * Motion Control Task
 *
 * Updates PI control every 100 ms.
 *
 * Note: Encoder sampling requires a separate
 * high-frequency mechanism before hardware use.
 */
static void motion_control_task(INT stacd, void *exinf)
{
    (void)stacd;
    (void)exinf;

    if (motion_init() != MOTION_OK)
    {
        for (;;)
        {
            tk_dly_tsk(1000);
        }
    }

    for (;;)
    {
        motion_update();
        tk_dly_tsk(100);
    }
}

/*
 * Initialise Subsystem 2 resources.
 */
EXPORT ER subsystem2_init(void)
{
    T_CSEM csem = {0};
    T_CMPF cmpf = {0};

    csem.sematr = TA_TFIFO;
    csem.isemcnt = S2_POOL_COUNT;
    csem.maxsem = S2_POOL_COUNT;

    sem_slots = tk_cre_sem(&csem);

    if (sem_slots < E_OK)
    {
        return sem_slots;
    }

    cmpf.mpfatr = TA_TFIFO | TA_RNG3;
    cmpf.mpfcnt = S2_POOL_COUNT;
    cmpf.blfsz = sizeof(SAMPLE_MSG);

    mpf_samples = tk_cre_mpf(&cmpf);

    return (mpf_samples < E_OK) ?
           mpf_samples : E_OK;
}

/*
 * Start Buddy 2 motion-control task on core 1.
 */
EXPORT ER subsystem2_start(void)
{
    ID tid;

    tid = app_start_task(
        motion_control_task,
        S2_PRI_PRODUCER,
        TP_PRC2,
        "motion_control_task"
    );

    return (tid < E_OK) ? tid : E_OK;
}
