/*
 * subsystem3.c
 * Subsystem 3 (Buddy 3): creates its kernel objects, starts its task,
 * and owns the IF-06 line_state word.
 */

#include "subsystem3.h"
#include "barcode_nav.h"

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

/*
 * Barcode mapping word (barcode_nav.h). Change Requirement 1: never
 * read-modify-write the shared word. The single writer builds each new
 * value from its own private copy and publishes it with one atomic
 * store; the Line task only ever loads it.
 */
static uint32_t g_nav_map __attribute__((aligned(4))) = NAV_MAP_EMPTY;
static uint32_t g_nav_map_private = NAV_MAP_EMPTY;  // writer's copy

EXPORT ER
s3_nav_map_set (uint8_t movement, char letter)
{
    ER result = E_PAR;

    if (nav_map_with(g_nav_map_private, movement, letter,
                     &g_nav_map_private))
    {
        __atomic_store_n(&g_nav_map, g_nav_map_private, __ATOMIC_RELEASE);
        result = E_OK;
    }
    else
    {
        // bad letter or movement: the mapping is unchanged
    }

    return result;
}

EXPORT uint32_t
s3_nav_map_read (void)
{
    // Acquire pairs with the writer's release store
    return __atomic_load_n(&g_nav_map, __ATOMIC_ACQUIRE);
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

#if S3_NAV_ENABLE && S3_NAV_BENCH_MAP
    // Temporary test mapping, see subsystem3.h. Runs before any task
    // starts, so the single-writer rule still holds.
    (void)s3_nav_map_set(LINE_CMD_LEFT, 'A');
    (void)s3_nav_map_set(LINE_CMD_RIGHT, 'B');
    (void)s3_nav_map_set(LINE_CMD_STRAIGHT, 'C');
    (void)s3_nav_map_set(LINE_CMD_UTURN, 'D');
#endif

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
    // Must stay on core 0 (TP_PRC1): the shared GPIO dispatcher only
    // enables its interrupt on core 0 (gpio_irq.c). On core 1 the attach
    // would still succeed but the barcode edges would never arrive.
    tid_line = app_start_task((FP)line_task, S3_PRI_LINE, TP_PRC1,
                              "line_task");

    return (tid_line < E_OK) ? tid_line : E_OK;
}
