/*
 * task_line.c
 * Subsystem 3 (Buddy 3): the Line / Barcode task.
 *
 * Priority 6, every 10 ms (TEAM.md section 4, Tribunal 1), woken by the
 * cyclic handler cyc_line so the period does not drift with execution
 * time. One cycle:
 *   1. read the left / right line sensors
 *   2. move barcode edges from the interrupt's ring into the capture
 *   3. finish any barcode run that went quiet; on a decode, update cmd
 *   4. step the line logic (position, junction, line-lost search)
 *   5. publish the IF-06 line_state word with one atomic store
 *   6. update the diagnostics (execution time, overruns)
 */

#include "subsystem3.h"
#include "drv_ir.h"
#include "line_logic.h"
#include "barcode_capture.h"

#define S3_EXEC_MAX_US          (0xFFFFU)   // diag field is 16 bits
#define S3_COUNT_MAX            (0xFFFFU)   // diag counters saturate
#define S3_CMD_SEQ_MASK         (0x03U)     // cmd_seq is 2 bits

/*
 * Task-private state (only this task touches it): bit-fields allowed.
 */
static line_logic_t g_line;
static bc_capture_t g_capture;

static struct
{
    unsigned cmd     : 2;   // LINE_CMD_*
    unsigned cmd_seq : 2;   // +1 per decoded command
    unsigned seq     : 8;   // publish counter
} g_out;

static uint8_t g_seen_overflows = 0U;   // drv_ir_overflows() last seen

static void    s3_drain_edges(void);
static void    s3_poll_barcode(uint32_t now_us);
static bool    s3_cmd_of(char data_char, uint8_t *p_cmd);
static void    s3_publish(uint8_t error, uint8_t mode);
static void    s3_update_diag(uint32_t start_us);
static void    s3_count(uint16_t *p_counter);

/*!
 * Cyclic handler (task-independent part): wake the Line task.
 */
void
s3_cyc_handler (void *exinf)
{
    (void)exinf;
    (void)tk_wup_tsk(tid_line);
}

void
line_task (INT stacd, void *exinf)
{
    uint32_t start_us;
    uint8_t  error = LINE_IF_ERROR_CENTRE;
    uint8_t  mode  = LINE_MODE_INIT;
    ER       err;

    (void)stacd;
    (void)exinf;

    // Interrupt is enabled on the core running this task (drv_ir.h)
    err = drv_ir_init();

    if (E_OK != err)
    {
        tm_printf((UB *)"S3: drv_ir_init failed %d, barcode off\n",
                  (INT)err);
    }
    else
    {
        // barcode interrupt running
    }

    line_logic_init(&g_line);
    bc_capture_init(&g_capture, drv_ir_barcode_black(), drv_ir_now_us());
    g_out.cmd     = LINE_CMD_STRAIGHT;
    g_out.cmd_seq = 0U;
    g_out.seq     = 0U;
    g_seen_overflows = drv_ir_overflows();

    // On the dual-core build this task may run before app_start_task()
    // has returned our ID to subsystem3_start(), so store it ourselves
    // before the cyclic handler can need it.
    tid_line = tk_get_tid();
    (void)tk_sta_cyc(cyc_line);

    for (;;)
    {
        (void)tk_slp_tsk(TMO_FEVR);
        start_us = drv_ir_now_us();

        s3_drain_edges();
        s3_poll_barcode(start_us);
        line_logic_step(&g_line, drv_ir_left_black(), drv_ir_right_black(),
                        &error, &mode);
        s3_publish(error, mode);
        s3_update_diag(start_us);
    }
}

/*!
 * Move every queued barcode edge into the capture. After a ring
 * overflow an edge is missing, so the current run cannot be trusted.
 */
static void
s3_drain_edges (void)
{
    uint32_t edge;
    uint8_t  overflows = drv_ir_overflows();

    if (overflows != g_seen_overflows)
    {
        g_seen_overflows = overflows;
        bc_capture_abort(&g_capture, drv_ir_now_us());
        s3_count(&g_s3_diag.edge_overflows);
    }
    else
    {
        // no edges lost
    }

    while (drv_ir_pop_edge(&edge))
    {
        bc_capture_edge(&g_capture,
                        (0U != (edge & DRV_IR_EDGE_BLACK_BIT)),
                        edge & DRV_IR_EDGE_TIME_MASK);
    }
}

static void
s3_poll_barcode (uint32_t now_us)
{
    char            data_char = '\0';
    uint8_t         cmd;
    bc_cap_result_t result;

    result = bc_capture_poll(&g_capture, now_us, &data_char);

    if (BC_CAP_DECODED == result)
    {
        if (s3_cmd_of(data_char, &cmd))
        {
            // New command: Subsystem 2 acts once when cmd_seq changes
            g_out.cmd     = cmd & LINE_IF_CMD_MASK;
            g_out.cmd_seq = (g_out.cmd_seq + 1U) & S3_CMD_SEQ_MASK;
            s3_count(&g_s3_diag.decode_ok);
        }
        else
        {
            // valid Code 39 but not A-D (e.g. 'Z'): no command
            s3_count(&g_s3_diag.decode_other);
        }

        g_s3_diag.last_char = data_char;
    }
    else if (BC_CAP_FAILED == result)
    {
        s3_count(&g_s3_diag.decode_fail);
    }
    else
    {
        // nothing new
    }
}

/*!
 * Barcode character -> command (Project Write-up table).
 */
static bool
s3_cmd_of (char data_char, uint8_t *p_cmd)
{
    bool b_is_cmd = true;

    switch (data_char)
    {
        case 'A':
            *p_cmd = LINE_CMD_LEFT;
            break;

        case 'B':
            *p_cmd = LINE_CMD_RIGHT;
            break;

        case 'C':
            *p_cmd = LINE_CMD_STRAIGHT;
            break;

        case 'D':
            *p_cmd = LINE_CMD_UTURN;
            break;

        default:
            b_is_cmd = false;
            break;
    }

    return b_is_cmd;
}

/*!
 * Build the whole word in a local, then store it once (never a
 * read-modify-write on the shared word).
 */
static void
s3_publish (uint8_t error, uint8_t mode)
{
    line_if_state_t state;

    g_out.seq = (g_out.seq + 1U) & LINE_IF_SEQ_MASK;

    // Casts: each bit-field is at most 8 bits wide
    state.seq             = (uint8_t)g_out.seq;
    state.mode            = mode;
    state.error           = error;
    state.cmd             = (uint8_t)g_out.cmd;
    state.cmd_seq         = (uint8_t)g_out.cmd_seq;
    state.b_left_black    = drv_ir_left_black();
    state.b_right_black   = drv_ir_right_black();
    state.b_barcode_black = drv_ir_barcode_black();

    s3_line_state_write(line_if_pack(&state));
}

static void
s3_update_diag (uint32_t start_us)
{
    // Unsigned subtraction is correct across the 32-bit timer wrap
    uint32_t exec_us = drv_ir_now_us() - start_us;
    INT      pending;

    if (exec_us > S3_EXEC_MAX_US)
    {
        exec_us = S3_EXEC_MAX_US;
    }
    else
    {
        // fits 16 bits
    }

    if (exec_us > g_s3_diag.max_exec_us)
    {
        // Cast: clamped to 0xFFFF just above
        g_s3_diag.max_exec_us = (uint16_t)exec_us;
    }
    else
    {
        // not a new maximum
    }

    // A wake-up already queued means the next period started before
    // this cycle finished: an overrun (NFR11).
    pending = tk_can_wup(TSK_SELF);

    if (pending > 0)
    {
        s3_count(&g_s3_diag.overruns);
    }
    else
    {
        // on time
    }
}

static void
s3_count (uint16_t *p_counter)
{
    if (*p_counter < S3_COUNT_MAX)
    {
        (*p_counter)++;
    }
    else
    {
        // saturated
    }
}
