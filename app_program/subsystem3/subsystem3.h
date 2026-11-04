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
 *   line_if_cmd_seq(word)       changes once per new barcode ...
 *   line_if_cmd(word)           ... whose command is LINE_CMD_*
 *   line_if_is_fresh(&reader, word)   false = S3 stalled: safe stop
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
 * Barcode on/off. 0 = line following only: no GP7 interrupt, no
 * decoding, cmd stays STRAIGHT and cmd_seq never changes, so Subsystem 2
 * never sees a barcode command. raw_bc still shows the GP7 level.
 * TODO: set back to 1 once the barcode sensor works (2026-11-01: off
 * while the barcode sensor has hardware issues).
 */
#define S3_BARCODE_ENABLE       (0)

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
    uint16_t decode_ok;         // barcodes decoded to A..D
    uint16_t decode_other;      // valid Code 39, not a command
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
