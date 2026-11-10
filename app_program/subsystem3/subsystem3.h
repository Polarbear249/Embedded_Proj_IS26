/*
 * subsystem3.h
 * Subsystem 3 (Buddy 3) -- IR line following and barcode decoding.
 * Student ID: <student ID>.
 *
 * Public interface. Other subsystems include it as
 * "../subsystem3/subsystem3.h".
 *
 * IF-06 (S3 -> S2, TEAM.md section 5): the line_state word.
 *   uint32_t word = s3_line_state_read();      // one atomic load
 * then decode it with line_if.h:
 *   line_if_error_signed(word)  steering error, < 0 = line to the left
 *   line_if_mode(word)          LINE_MODE_*  (FAILED: stop the car)
 *   line_if_cmd_seq(word)       changes once per cross-junction ...
 *   line_if_cmd(word)           ... whose movement is LINE_CMD_*
 *   line_if_char_seq(word)      changes once per decoded letter ...
 *   line_if_char(word)          ... which is 0..25, 'A'..'Z'
 *   line_if_is_fresh(&reader, word)   false = S3 stalled: safe stop
 *
 * Barcode mapping: s3_nav_map_set(), see below.
 */

#ifndef SUBSYSTEM3_H
#define SUBSYSTEM3_H

#include <stdbool.h>
#include <stdint.h>
#include "../app.h"
#include "line_if.h"

#define S3_PRI_LINE             (6)     // TEAM.md section 4 (Tribunal 1)
#define S3_PERIOD_MS            (10)    // TEAM.md section 4

/*
 * Barcode on/off. 1 = GP7 edge interrupt + Code 39 decoding while the
 * line is followed; every result is printed on the console ("S3
 * barcode: ...") and published in bc_char / char_seq. 0 = line following
 * only: no GP7 interrupt, no decoding, no movements, and char_seq never
 * changes. raw_bc shows the GP7 level either way.
 */
#define S3_BARCODE_ENABLE       (1)

/*
 * TEMPORARY stand-in for the Vehicle Controller's mission logic, which
 * is not written yet (barcode_nav.h). 1 = S3 keeps the mapping, and at a
 * cross-junction performs the stored letter's movement itself. 0 = S3
 * only publishes the letter and counts cross-junctions in cmd_seq, for
 * the VC to act on; cmd stays STRAIGHT. Set to 0 once the VC does this:
 * only one module may command the motors. Needs S3_BARCODE_ENABLE.
 */
#define S3_NAV_SWITCH           (1)

#if S3_NAV_SWITCH && S3_BARCODE_ENABLE
#define S3_NAV_ENABLE           (1)
#else
#define S3_NAV_ENABLE           (0)
#endif

/*
 * TEMPORARY, for testing before the MQTT mapping exists. 1 = start with
 * A = left, B = right, C = forward, D = U-turn. The demo mapping MUST
 * come from MQTT, so set this to 0 once Buddy 1's commands work.
 * s3_nav_map_set() replaces these entries either way.
 */
#define S3_NAV_BENCH_MAP        (1)

/*
 * Line-following drive (line_drive.h), a temporary stand-in until
 * Buddy 2's motion task reads line_state itself. 1 = the Line task
 * steers the car through Buddy 2's motion API; 0 = it never
 * calls the motion API (the code before 2026-11-01). Never active in
 * TEST=1 builds, so the bench tests cannot spin the wheels.
 */
#define S3_DRIVE_SWITCH         (1)

/*
 * Drive API (Buddy 2, 2026-11-04, untested on the car). 1 = steer through
 * PI speed mode, motion_line_drive(), which climbed the 10 mm ramp where
 * duty mode stalled; 0 = the old motion_set_duty(). line_drive.c still
 * works in duty units; S3_DRIVE_DUTY_PER_RPM converts them: base duty
 * 120 / 6 = 20 RPM, the speed of the passing ramp runs.
 */
#define S3_DRIVE_USE_SPEED      (1)
#define S3_DRIVE_DUTY_PER_RPM   (6)

#if S3_DRIVE_SWITCH && !APP_TEST
#define S3_DRIVE_ENABLE         (1)
#else
#define S3_DRIVE_ENABLE         (0)
#endif

/*!
 * IF-06: latest line_state word (bit layout in line_if.h). Any task.
 */
IMPORT uint32_t s3_line_state_read(void);

/*!
 * Barcode mapping: letter ('A'..'Z') now means movement (LINE_CMD_*).
 * Returns E_OK, or E_PAR for anything else. Call it from ONE task only,
 * the one that receives the MQTT mapping commands (single writer). Takes
 * effect at the next cross-junction.
 */
IMPORT ER s3_nav_map_set(uint8_t movement, char letter);

/*!
 * The current mapping word (layout in barcode_nav.h). Any task.
 */
IMPORT uint32_t s3_nav_map_read(void);

/*!
 * Runtime stand-down for a task that owns the motors (The VC's obstacle bypass). While disabled the Line Task neither drives nor stops them
 * so its motion_stop() cannot cancel a VC move. 1 = normal line following, the power-on default.
 */
IMPORT void s3_drive_set_enabled(bool b_on);

/*
 * Diagnostics for tests and NFR11 evidence. Written only by the Line
 * task; each field is one aligned 16-bit store, so a reader never sees
 * a torn value (fields may be from different cycles). Never used for
 * control.
 */
typedef struct
{
    uint16_t max_exec_us;       // longest cycle so far, us
    uint16_t overruns;          // cycles that missed the 10 ms slot
    uint16_t decode_ok;         // barcodes decoded to A..Z
    uint16_t decode_other;      // valid Code 39, not a letter
    uint16_t decode_fail;       // barcode-like runs that did not decode
    uint16_t edge_overflows;    // times the edge ring was full
    char     last_char;         // last decoded character, '\0' = none
} s3_diag_t;

IMPORT s3_diag_t g_s3_diag;

/* ---------------------------------------------------------------------
 *  LEGACY (template example IF-04): subsystem4/imu.c still locks
 *  mtx_stats and reads app_stats. Subsystem 3 no longer produces these
 *  statistics (they stay 0); they exist only so S4 still builds.
 *  Remove once S4 drops that code.
 * --------------------------------------------------------------------- */
typedef struct {
    UW  received;
    INT min;
    INT max;
    W   sum;
} APP_STATS;

IMPORT ID        mtx_stats;
IMPORT APP_STATS app_stats;

/* Private to subsystem 3 */
IMPORT ID   tid_line;
IMPORT ID   cyc_line;
IMPORT void line_task(INT stacd, void *exinf);
IMPORT void s3_cyc_handler(void *exinf);
IMPORT void s3_line_state_write(uint32_t word);

#endif /* SUBSYSTEM3_H */
