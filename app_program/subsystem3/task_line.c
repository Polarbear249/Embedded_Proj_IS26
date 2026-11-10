/*
 * task_line.c
 * Subsystem 3 (Buddy 3): the Line / Barcode task.
 *
 * Priority 6, every 10 ms (TEAM.md section 4, Tribunal 1), woken by the
 * cyclic handler cyc_line so the period does not drift with execution
 * time. One cycle:
 *   1. read the centre / right line sensors
 *   2. move barcode edges from the interrupt's ring into the capture
 *   3. finish any barcode run that went quiet; on a decode, store the
 *      letter
 *   4. step the line logic (position, junction, line-lost search)
 *   5. publish the IF-06 line_state word with one atomic store
 *   6. at a cross-junction, count it in cmd_seq; if S3_NAV_ENABLE also
 *      start the stored letter's movement and publish it in cmd
 *   7. if S3_DRIVE_ENABLE: steer through Buddy 2's motion API
 *   8. update the diagnostics (execution time, overruns)
 */

#include "subsystem3.h"
#include "drv_ir.h"
#include "line_logic.h"
#include "barcode_capture.h"
#include "barcode_nav.h"
#if S3_DRIVE_ENABLE
#include "line_drive.h"
#include "../subsystem2/motion_control.h"

_Static_assert(DRIVE_DUTY_MAX <= MOTION_MAX_DUTY, "S2 duty limit");
_Static_assert(DRIVE_LEASE_MS <= MOTION_MAX_LEASE_MS, "S2 lease limit");
_Static_assert(NAV_SPIN_DUTY <= MOTION_MAX_DUTY, "S2 duty limit");
_Static_assert(NAV_PREFWD_DUTY <= MOTION_MAX_DUTY, "S2 duty limit");
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
    unsigned cmd_seq : 2;   // +1 per cross-junction
    unsigned bc_char : 5;   // latest letter, 0..25 = 'A'..'Z'
    unsigned char_seq: 2;   // +1 per decoded letter
    unsigned seq     : 8;   // publish counter
} g_out;

#if S3_NAV_ENABLE
static nav_t g_nav;
#else
static nav_watch_t g_junction_watch;
#endif

static uint8_t g_seen_overflows = 0U;   // drv_ir_overflows() last seen

#if S3_BARCODE_ENABLE
// Barcode sensor (GP7) level changes received since power-up,
// saturating. 0 while driving over barcodes = DOUT never switched.
static uint16_t g_bc_edges = 0U;
static uint32_t g_bc_last_edge_us = 0U;     // timestamp of the last edge

// Bring-up aid, off by default: record the raw GP7 pulses of one burst
// and print them once the sensor has been still for 0.3 s, to tell clean
// bars from comparator chatter. Up to 64 prints from this 10 ms task, so
// leave it off for line-following runs.
#define S3_BC_RAW_DEBUG         (0)
#endif

#if S3_BARCODE_ENABLE && S3_BC_RAW_DEBUG
#define S3_RAW_MAX              (64U)       // pulses kept per burst
#define S3_RAW_IDLE_US          (300000U)   // burst over after 0.3 s
#define S3_RAW_LEVEL_BIT        (0x8000U)   // 1 = the pulse was black
#define S3_RAW_UNITS_MAX        (0x7FFFU)   // 8 us units: up to 262 ms

static uint16_t g_raw[S3_RAW_MAX];          // level bit + width
static uint8_t  g_raw_n = 0U;               // pulses stored
static uint32_t g_raw_last_us = 0U;         // time of the last edge
static bool     g_b_raw_started = false;    // g_raw_last_us is valid

static void    s3_raw_record(uint32_t edge);
static void    s3_raw_flush(uint32_t now_us);
#endif

#if S3_DRIVE_ENABLE
static line_drive_t g_drive;
static bool         g_b_was_driving = false;    // last cycle drove

static volatile bool g_b_drive_enabled = true;   // VC stands S3 down in a bypass

static void    s3_drive(uint8_t error, uint8_t mode);

#if S3_DRIVE_USE_SPEED
#define S3_DRIVE_RPM_MAX        (30.0f)     // motion_line_drive() limit

// Duty units to RPM, clamped: a full search duty (200) would be 33 RPM
static float
s3_duty_to_rpm (int16_t duty)
{
    float rpm = (float)duty / (float)S3_DRIVE_DUTY_PER_RPM;

    if (rpm > S3_DRIVE_RPM_MAX)
    {
        rpm = S3_DRIVE_RPM_MAX;
    }
    else if (rpm < -S3_DRIVE_RPM_MAX)
    {
        rpm = -S3_DRIVE_RPM_MAX;
    }
    else
    {
        // in range
    }

    return rpm;
}
#endif

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
static void    s3_print_run(void);
#endif
#if S3_NAV_ENABLE
static void    s3_nav(uint8_t mode, bool b_active, nav_out_t *p_nav_out);
static char const *s3_cmd_name(uint8_t cmd);
#else
static void    s3_junction_watch(uint8_t mode);
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
s3_drive_set_enabled (bool b_on)
{
#if S3_DRIVE_ENABLE
    g_b_drive_enabled = b_on;

    if (false == b_on)
    {
        g_b_was_driving = false;   // Stand-down must not motion_stop()
    }
#else
        (void)b_on;
#endif
}

void
line_task (INT stacd, void *exinf)
{
    uint32_t start_us;
    uint8_t  error = LINE_IF_ERROR_CENTRE;
    uint8_t  mode  = LINE_MODE_INIT;
#if S3_BARCODE_ENABLE
    ER       err;
    bool     b_barcode_black;
#if S3_NAV_ENABLE && !S3_DRIVE_ENABLE
    nav_out_t nav_out;
#endif
#endif

    (void)stacd;
    (void)exinf;

    drv_ir_init();

#if S3_BARCODE_ENABLE
    // Through the shared dispatcher, core 0 only (subsystem3_start)
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
    g_out.bc_char = 0U;
    g_out.char_seq = 0U;
    g_out.seq     = 0U;
    g_seen_overflows = drv_ir_overflows();
#if S3_NAV_ENABLE
    nav_init(&g_nav);
#if S3_NAV_BENCH_MAP
    tm_printf((UB *)"S3 nav: TEST mapping A=left B=right C=forward "
                    "D=u-turn (not for the demo)\n");
#endif
#else
    nav_watch_init(&g_junction_watch);
#endif
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

        // Once a barcode run is under way, black under the barcode
        // sensor is a bar, not a crossing line
        b_barcode_black = drv_ir_barcode_black()
                          && !bc_capture_busy(&g_capture);
        line_logic_step(&g_line, drv_ir_centre_black(),
                        drv_ir_right_black(), b_barcode_black,
                        &error, &mode);
#else
        line_logic_step(&g_line, drv_ir_centre_black(),
                        drv_ir_right_black(), drv_ir_barcode_black(),
                        &error, &mode);
#endif
        s3_publish(error, mode);
#if S3_DRIVE_ENABLE
        s3_drive(error, mode);
#elif S3_NAV_ENABLE
        // No drive here: nothing to arm, so the movement is only
        // published for other subsystems
        s3_nav(mode, true, &nav_out);
#endif
#if !S3_NAV_ENABLE
        s3_junction_watch(mode);
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
#if S3_NAV_ENABLE
    nav_out_t nav_out;
#endif

    // Debounced centre (as the steering uses), so a one-sample flicker
    // does not restart the 2 s arm time
    b_drive = line_drive_step(&g_drive, error, mode,
                              (0U != g_line.b_centre), &left, &right);

#if S3_NAV_ENABLE
    // A movement at a junction replaces the line steering
    s3_nav(mode, b_drive, &nav_out);

    if (nav_out.b_override)
    {
        left  = nav_out.left;
        right = nav_out.right;
    }
    else
    {
        // normal line following
    }
#endif

    if (b_drive && g_b_drive_enabled)
    {
#if S3_DRIVE_USE_SPEED
        // Same steering law, as wheel speeds through Buddy 2's PI. Any
        // refusal shows as E_OBJ in the debug line.
        err = (MOTION_OK == motion_line_drive(s3_duty_to_rpm(left),
                                              s3_duty_to_rpm(right),
                                              (UINT)DRIVE_LEASE_MS))
              ? E_OK : E_OBJ;
#else
        // Casts: duties are within +-DRIVE_DUTY_MAX (200)
        err = motion_set_duty((INT)left, (INT)right, (UINT)DRIVE_LEASE_MS);
#endif

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
    g_b_was_driving = (b_drive && g_b_drive_enabled); //d
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
    motion_diag_t  diag;
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
        motion_get_diag(&diag);
        tm_printf((UB *)"S3 drive: C=%u R=%u mode=%u err=%u arm=%u/%u "
                        "armed=%u wheels=%d/%d motion=%d stale=%u lease=%u\n",
                  (UINT)drv_ir_centre_black(), (UINT)drv_ir_right_black(),
                  (UINT)mode, (UINT)error, (UINT)g_drive.arm_count,
                  (UINT)DRIVE_ARM_TICKS, (UINT)g_drive.b_armed,
                  (INT)left, (INT)right, (INT)worst,
                  (UINT)diag.line_stale_stops, (UINT)diag.lease_stops);
#if S3_BARCODE_ENABLE
        // cap: 0 = waiting for a start bar, 1 = reading a barcode
        // (n = elements so far), 2 = waiting for white after a run
        tm_printf((UB *)"S3 barcode sensor: BC=%u edges=%u cap=%u n=%u "
                        "(ok %u, failed %u) fw %s\n",
                  (UINT)drv_ir_barcode_black(), (UINT)g_bc_edges,
                  (UINT)g_capture.state, (UINT)g_capture.count,
                  (UINT)g_s3_diag.decode_ok, (UINT)g_s3_diag.decode_fail,
                  __TIME__);
        // Timing check: "now" must advance ~1000 ms per line, and
        // "last edge" must be close to "now" after a fresh edge.
        tm_printf((UB *)"S3 timing: now=%u ms last edge=%u ms "
                        "lost=%u\n",
                  (UINT)((drv_ir_now_us() & DRV_IR_EDGE_TIME_MASK)
                         / 1000U),
                  (UINT)(g_bc_last_edge_us / 1000U),
                  (UINT)g_s3_diag.edge_overflows);
#endif
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
        s3_count(&g_bc_edges);
        g_bc_last_edge_us = edge & DRV_IR_EDGE_TIME_MASK;
#if S3_BC_RAW_DEBUG
        s3_raw_record(edge);
#endif
        bc_capture_edge(&g_capture,
                        (0U != (edge & DRV_IR_EDGE_BLACK_BIT)),
                        edge & DRV_IR_EDGE_TIME_MASK);
    }
}

static void
s3_poll_barcode (uint32_t now_us)
{
    char            data_char = '\0';
    uint8_t         code;
    bc_cap_result_t result;

    result = bc_capture_poll(&g_capture, now_us, &data_char);
#if S3_BC_RAW_DEBUG
    s3_raw_flush(now_us);
#endif

    if (BC_CAP_DECODED == result)
    {
        if (line_if_char_code(data_char, &code))
        {
            // Published for the VC; acted on at the next cross-junction
            g_out.bc_char  = code & LINE_IF_CHAR_MASK;
            g_out.char_seq = (g_out.char_seq + 1U) & LINE_IF_CHAR_SEQ_MASK;
#if S3_NAV_ENABLE
            (void)nav_store(&g_nav, data_char);
#endif
            s3_count(&g_s3_diag.decode_ok);
            tm_printf((UB *)"S3 barcode: '%c' stored (ok %u, failed %u)\n",
                      (INT)data_char, (UINT)g_s3_diag.decode_ok,
                      (UINT)g_s3_diag.decode_fail);
        }
        else
        {
            // valid Code 39 but not a letter (e.g. '5'): ignored
            s3_count(&g_s3_diag.decode_other);
            tm_printf((UB *)"S3 barcode: '%c' is not a letter, ignored\n",
                      (INT)data_char);
        }

        g_s3_diag.last_char = data_char;
    }
    else if (BC_CAP_FAILED == result)
    {
        s3_count(&g_s3_diag.decode_fail);
        tm_printf((UB *)"S3 barcode: bars seen but not decoded "
                        "(failed %u)\n", (UINT)g_s3_diag.decode_fail);
        s3_print_run();
    }
    else if (BC_CAP_SHORT == result)
    {
        tm_printf((UB *)"S3 barcode: short run (a barcode has 29)\n");
        s3_print_run();
    }
    else
    {
        // nothing new
    }
}

#if S3_BC_RAW_DEBUG
/*!
 * Store the pulse that this edge ended: its level (the opposite of the
 * new level) and its width in 8 us units.
 */
static void
s3_raw_record (uint32_t edge)
{
    uint32_t time_us = edge & DRV_IR_EDGE_TIME_MASK;
    uint32_t units;

    if (g_b_raw_started && (g_raw_n < S3_RAW_MAX))
    {
        units = ((time_us - g_raw_last_us) & DRV_IR_EDGE_TIME_MASK)
                >> BC_CAP_UNIT_SHIFT;

        if (units > S3_RAW_UNITS_MAX)
        {
            units = S3_RAW_UNITS_MAX;
        }
        else
        {
            // fits 15 bits
        }

        // The pulse that just ended was black if the new level is white
        // Cast: units <= 0x7FFF, the level bit is bit 15
        g_raw[g_raw_n] = (uint16_t)(units
                         | ((0U == (edge & DRV_IR_EDGE_BLACK_BIT))
                            ? S3_RAW_LEVEL_BIT : 0U));
        g_raw_n++;
    }
    else
    {
        // first edge of a burst (no width yet) or buffer full
    }

    g_raw_last_us   = time_us;
    g_b_raw_started = true;
}

/*!
 * Once the sensor has been still for S3_RAW_IDLE_US, print the burst:
 * B = black pulse, w = white pulse, widths in microseconds.
 */
static void
s3_raw_flush (uint32_t now_us)
{
    uint8_t  idx;
    uint32_t idle_us;

    idle_us = ((now_us & DRV_IR_EDGE_TIME_MASK) - g_raw_last_us)
              & DRV_IR_EDGE_TIME_MASK;

    if (g_b_raw_started && (idle_us > S3_RAW_IDLE_US))
    {
        if (0U != g_raw_n)
        {
            tm_printf((UB *)"S3 raw: %u pulses (us):", (UINT)g_raw_n);

            for (idx = 0U; idx < g_raw_n; idx++)
            {
                tm_printf((UB *)"%s%c%u",
                          (0U == (idx % 10U)) ? "\n  " : " ",
                          (0U != (g_raw[idx] & S3_RAW_LEVEL_BIT))
                          ? 'B' : 'w',
                          (UINT)((uint32_t)(g_raw[idx]
                                            & S3_RAW_UNITS_MAX)
                                 << BC_CAP_UNIT_SHIFT));
            }

            tm_printf((UB *)"\n");
        }
        else
        {
            // a single edge: nothing to show
        }

        g_raw_n         = 0U;
        g_b_raw_started = false;
    }
    else
    {
        // burst still running, or nothing recorded
    }
}
#endif

/*!
 * Bring-up aid: print the element widths of the run that just ended,
 * in 0.1 ms, bar first (B = black bar, w = white gap). A clean Code 39
 * symbol shows narrow and wide (3x) elements only.
 */
static void
s3_print_run (void)
{
    uint8_t  idx;
    uint32_t tenth_ms;

    tm_printf((UB *)"S3 barcode: %u elements, widths in 0.1 ms:",
              (UINT)g_capture.last_count);

    for (idx = 0U; (idx < g_capture.last_count)
                   && (idx < BC39_SYMBOL_ELEMENTS); idx++)
    {
        // 1 unit = 8 us, so units x 8 / 100 = 0.1 ms
        tenth_ms = ((uint32_t)g_capture.width[idx]
                    << BC_CAP_UNIT_SHIFT) / 100U;
        tm_printf((UB *)" %c%u", (0U == (idx & 0x1U)) ? 'B' : 'w',
                  (UINT)tenth_ms);
    }

    tm_printf((UB *)"\n");
}

#if S3_NAV_ENABLE
/*!
 * Step the junction logic. At a cross-junction: publish the movement
 * (Subsystem 2 acts once when cmd_seq changes) and drop any barcode run,
 * because the crossing line itself started one. Dropping it again after
 * a turn clears whatever the spin swept the sensor over.
 */
static void
s3_nav (uint8_t mode, bool b_active, nav_out_t *p_nav_out)
{
    nav_event_t event;

    event = nav_step(&g_nav, s3_nav_map_read(), mode,
                     (0U != g_line.b_centre), b_active, p_nav_out);

    if (NAV_EV_JUNCTION == event)
    {
        g_out.cmd     = p_nav_out->movement & LINE_IF_CMD_MASK;
        g_out.cmd_seq = (g_out.cmd_seq + 1U) & S3_CMD_SEQ_MASK;
        bc_capture_abort(&g_capture, drv_ir_now_us());

        if ('\0' == p_nav_out->letter)
        {
            tm_printf((UB *)"S3 junction: no stored letter, %s\n",
                      s3_cmd_name(p_nav_out->movement));
        }
        else if (p_nav_out->b_mapped)
        {
            tm_printf((UB *)"S3 junction: '%c' = %s\n",
                      (INT)p_nav_out->letter,
                      s3_cmd_name(p_nav_out->movement));
        }
        else
        {
            tm_printf((UB *)"S3 junction: '%c' has no mapping, %s\n",
                      (INT)p_nav_out->letter,
                      s3_cmd_name(p_nav_out->movement));
        }
    }
    else if (NAV_EV_TURN_DONE == event)
    {
        bc_capture_abort(&g_capture, drv_ir_now_us());
        tm_printf((UB *)"S3 junction: turn done, following the line\n");
    }
    else if (NAV_EV_TURN_TIMEOUT == event)
    {
        bc_capture_abort(&g_capture, drv_ir_now_us());
        tm_printf((UB *)"S3 junction: turn found no line in 5 s\n");
    }
    else
    {
        // nothing to report
    }
}

/*!
 * Printable name of a LINE_CMD_* (console only).
 */
static char const *
s3_cmd_name (uint8_t cmd)
{
    char const *p_name;

    switch (cmd)
    {
        case LINE_CMD_LEFT:
            p_name = "TURN LEFT";
            break;

        case LINE_CMD_RIGHT:
            p_name = "TURN RIGHT";
            break;

        case LINE_CMD_STRAIGHT:
            p_name = "GO STRAIGHT";
            break;

        default:
            p_name = "U-TURN";
            break;
    }

    return p_name;
}
#endif /* S3_NAV_ENABLE */

#endif /* S3_BARCODE_ENABLE */

#if !S3_NAV_ENABLE
/*!
 * The VC does the movement: only count each cross-junction in cmd_seq,
 * and drop the barcode run the crossing line itself started.
 */
static void
s3_junction_watch (uint8_t mode)
{
    if (nav_watch_step(&g_junction_watch, mode))
    {
        g_out.cmd_seq = (g_out.cmd_seq + 1U) & S3_CMD_SEQ_MASK;
        bc_capture_abort(&g_capture, drv_ir_now_us());
        tm_printf((UB *)"S3 junction: cross-junction reached\n");
    }
    else
    {
        // no new crossing
    }
}
#endif

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
    state.bc_char         = (uint8_t)g_out.bc_char;
    state.char_seq        = (uint8_t)g_out.char_seq;
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
