/*
 * task_line.c
 * Subsystem 3 (Buddy 3): the Line / Barcode task.
 *
 * Priority 6, every 10 ms (TEAM.md section 4, Tribunal 1), woken by the
 * cyclic handler cyc_line so the period does not drift with execution
 * time. One cycle:
 *   1. read the centre / right line sensors
 *   2. move barcode edges from the interrupt's ring into the capture
 *   3. finish any barcode run that went quiet; on a decode, update cmd
 *   4. step the line logic (position, junction, line-lost search)
 *   5. publish the IF-06 line_state word with one atomic store
 *   6. if S3_DRIVE_ENABLE: steer through Buddy 2's motion API
 *   7. update the diagnostics (execution time, overruns)
 */

#include "subsystem3.h"
#include "drv_ir.h"
#include "line_logic.h"
#include "barcode_capture.h"
#if S3_DRIVE_ENABLE
#include "line_drive.h"
#include "../subsystem2/motion_control.h"

_Static_assert(DRIVE_DUTY_MAX <= MOTION_MAX_DUTY, "S2 duty limit");
_Static_assert(DRIVE_LEASE_MS <= MOTION_MAX_LEASE_MS, "S2 lease limit");
#endif

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

#if S3_DRIVE_ENABLE
static line_drive_t g_drive;
static bool         g_b_was_driving = false;    // last cycle drove

static void    s3_drive(uint8_t error, uint8_t mode);

// 1 = print a drive status line every second (bring-up aid)
#define S3_DRIVE_DEBUG          (1)
#define S3_DRIVE_DEBUG_TICKS    (100U)  // 100 x 10 ms = 1 s

#if S3_DRIVE_DEBUG
static void    s3_drive_debug(uint8_t error, uint8_t mode, int16_t left,
                              int16_t right, ER err);
#endif
#endif

#if S3_BARCODE_ENABLE
static void    s3_drain_edges(void);
static void    s3_poll_barcode(uint32_t now_us);
static bool    s3_cmd_of(char data_char, uint8_t *p_cmd);
#endif
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
#if S3_BARCODE_ENABLE
    ER       err;
#endif

    (void)stacd;
    (void)exinf;

    drv_ir_init();

#if S3_BARCODE_ENABLE
    // Interrupt is enabled on the core running this task (drv_ir.h)
    err = drv_ir_barcode_start();

    if (E_OK != err)
    {
        tm_printf((UB *)"S3: barcode interrupt failed %d, barcode off\n",
                  (INT)err);
    }
    else
    {
        // barcode interrupt running
    }
#else
    tm_printf((UB *)"S3: barcode disabled (S3_BARCODE_ENABLE 0)\n");
#endif

    line_logic_init(&g_line);
    bc_capture_init(&g_capture, drv_ir_barcode_black(), drv_ir_now_us());
    g_out.cmd     = LINE_CMD_STRAIGHT;
    g_out.cmd_seq = 0U;
    g_out.seq     = 0U;
    g_seen_overflows = drv_ir_overflows();
#if S3_DRIVE_ENABLE
    line_drive_init(&g_drive);
    tm_printf((UB *)"S3: drive ON: place the car on the line, it starts "
                    "after 2 s\n");
#endif

    // On the dual-core build this task may run before app_start_task()
    // has returned our ID to subsystem3_start(), so store it ourselves
    // before the cyclic handler can need it.
    tid_line = tk_get_tid();
    (void)tk_sta_cyc(cyc_line);

    for (;;)
    {
        (void)tk_slp_tsk(TMO_FEVR);
        start_us = drv_ir_now_us();

#if S3_BARCODE_ENABLE
        s3_drain_edges();
        s3_poll_barcode(start_us);
#endif
        line_logic_step(&g_line, drv_ir_centre_black(),
                        drv_ir_right_black(), drv_ir_barcode_black(),
                        &error, &mode);
        s3_publish(error, mode);
#if S3_DRIVE_ENABLE
        s3_drive(error, mode);
#endif
        s3_update_diag(start_us);
    }
}

#if S3_DRIVE_ENABLE
/*!
 * Steer through Buddy 2's motion API. Each command is renewed every
 * 10 ms with a DRIVE_LEASE_MS lease, so the motors stop by themselves
 * if this task ever stops. A failed call (e.g. S2's command mutex busy
 * for an instant) is simply retried on the next cycle.
 */
static void
s3_drive (uint8_t error, uint8_t mode)
{
    int16_t left;
    int16_t right;
    bool    b_drive;
    ER      err = E_OK;

    // Debounced centre (as the steering uses), so a one-sample flicker
    // does not restart the 2 s arm time
    b_drive = line_drive_step(&g_drive, error, mode,
                              (0U != g_line.b_centre), &left, &right);

    if (b_drive)
    {
        // Casts: duties are within +-DRIVE_DUTY_MAX (200)
        err = motion_set_duty((INT)left, (INT)right, (UINT)DRIVE_LEASE_MS);

        if (!g_b_was_driving)
        {
            tm_printf((UB *)"S3: drive armed, following the line\n");
        }
        else
        {
            // still driving
        }
    }
    else if (g_b_was_driving)
    {
        (void)motion_stop();
        tm_printf((UB *)"S3: drive stopped (mode %u), put the car back "
                        "on the line\n", (UINT)mode);
    }
    else
    {
        // stopped and staying stopped; S2's lease has already run out
    }

#if S3_DRIVE_DEBUG
    s3_drive_debug(error, mode, left, right, err);
#else
    (void)err;
#endif
    g_b_was_driving = b_drive;
}

#if S3_DRIVE_DEBUG
/*!
 * Once a second: sensors, line logic, arm progress, wheel duties and
 * the result of the last motion_set_duty() (0 = E_OK).
 */
static void
s3_drive_debug (uint8_t error, uint8_t mode, int16_t left, int16_t right,
                ER err)
{
    static uint8_t ticks = 0U;
    static ER      worst = E_OK;    // first failure since the last line

    if ((E_OK == worst) && (E_OK != err))
    {
        worst = err;
    }
    else
    {
        // keep the first failure
    }

    ticks++;

    if (ticks >= S3_DRIVE_DEBUG_TICKS)
    {
        ticks = 0U;
        tm_printf((UB *)"S3 drive: C=%u R=%u mode=%u err=%u arm=%u/%u "
                        "armed=%u wheels=%d/%d motion=%d\n",
                  (UINT)drv_ir_centre_black(), (UINT)drv_ir_right_black(),
                  (UINT)mode, (UINT)error, (UINT)g_drive.arm_count,
                  (UINT)DRIVE_ARM_TICKS, (UINT)g_drive.b_armed,
                  (INT)left, (INT)right, (INT)worst);
        worst = E_OK;
    }
    else
    {
        // not yet
    }
}
#endif
#endif

#if S3_BARCODE_ENABLE
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

#endif /* S3_BARCODE_ENABLE */

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
    state.b_centre_black  = drv_ir_centre_black();
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
