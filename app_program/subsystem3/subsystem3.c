/*
 * subsystem3.c
 * Subsystem 3 (Buddy 3): creates its kernel objects, starts its task,
 * and owns the IF-06 line_state word.
 */

#include "subsystem3.h"

EXPORT ID        tid_line  = 0;
EXPORT ID        cyc_line  = 0;
EXPORT s3_diag_t g_s3_diag = {0U, 0U, 0U, 0U, 0U, 0U, '\0'};

// LEGACY template IF-04, see subsystem3.h
EXPORT ID        mtx_stats = 0;
EXPORT APP_STATS app_stats = {0U, 0, 0, 0};

/*
 * IF-06 line_state word. Single writer (the Line task), any number of
 * readers. Aligned 32-bit, accessed only through the two functions
 * below with atomic store / load, so readers never see half a word.
 * Starts at 0: seq 0, mode LINE_MODE_INIT.
 */
static uint32_t g_line_state __attribute__((aligned(4))) = 0U;

uint32_t
s3_line_state_read (void)
{
    // Acquire pairs with the writer's release store
    return __atomic_load_n(&g_line_state, __ATOMIC_ACQUIRE);
}

void
s3_line_state_write (uint32_t word)
{
    __atomic_store_n(&g_line_state, word, __ATOMIC_RELEASE);
}

EXPORT ER
subsystem3_init (void)
{
    T_CCYC ccyc = {0};
    T_CMTX cmtx = {0};
    ER     result;

    // Cyclic handler: wakes the Line task every 10 ms. Created stopped
    // (no TA_STA); the task starts it once its interrupt is set up.
    ccyc.cycatr = TA_HLNG;
    ccyc.cychdr = (FP)s3_cyc_handler;
    ccyc.cyctim = S3_PERIOD_MS;
    ccyc.cycphs = S3_PERIOD_MS;
    cyc_line = tk_cre_cyc(&ccyc);

    if (cyc_line < E_OK)
    {
        result = cyc_line;
    }
    else
    {
        // LEGACY: mutex for subsystem4/imu.c only
        cmtx.mtxatr = TA_INHERIT;
        mtx_stats = tk_cre_mtx(&cmtx);
        result = (mtx_stats < E_OK) ? mtx_stats : E_OK;
    }

    return result;
}

EXPORT ER
subsystem3_start (void)
{
    // Pinned to core 0 (TP_PRC1): the barcode interrupt is enabled on
    // the core the task runs on. TEAM.md has not fixed cores yet.
    tid_line = app_start_task((FP)line_task, S3_PRI_LINE, TP_PRC1,
                              "line_task");

    return (tid_line < E_OK) ? tid_line : E_OK;
}
