/*
 * line_logic.h
 * Subsystem 3 (Buddy 3): line position from two on/off IR sensors.
 *
 * Sensor layout (owner, 2026-11-01): the centre sensor (GP27) sits OVER
 * the 18 mm line, the right sensor (GP6) just OUTSIDE its right edge.
 * There is no left line sensor: the left side carries the barcode
 * sensor (GP7), which looks at the barcode lane, not the line.
 *
 *   centre right   meaning                          error (64 = centre)
 *   ------ -----   -------------------------------  -------------------
 *   B      W       on the line                      64
 *   B      B       line slightly right of centre    64 + NEAR
 *   W      B       line under the right sensor      64 + SIDE
 *   W      W       line beyond the right sensor     64 + SIDE
 *                   (right sensor saw it last)
 *   W      W       line went left (a GUESS: there   64 - GUESS
 *                   is no left sensor)
 *
 * Which side for white/white: the line can only reach the right sensor
 * by passing it, so if the centre sensor loses the line and the right
 * sensor did not pick it up, the line went LEFT. If the right sensor
 * saw it last, the line is to the RIGHT. "Left" is only a guess (the
 * car may have drifted left, the line passing through the gap before
 * the right sensor), so it steers more gently (GUESS < SIDE): a wrong
 * guess costs a small wobble, not a sharp turn. Mount the right sensor
 * so its spot is just outside the line edge when the car is centred (about
 * 12-15 mm centre to centre): a gap there would briefly steer the
 * wrong way.
 *
 * Junction / crossing line: centre, right AND barcode sensor (left
 * side) all black for LINE_JUNCTION_TICKS. A barcode bar under the left
 * sensor while the car drifts (centre + right black) looks the same
 * (seen on the car), so the task passes the barcode sensor as white once
 * a barcode run is under way (bc_capture_busy), and the time limit
 * rejects a run's first bar, which is always narrow.
 *
 * If the centre and right sensors both stay white for LINE_LOST_TICKS,
 * the line is lost:
 *   SEARCH_NEAR  full turn toward the last side seen, LINE_SEARCH1_TICKS
 *   SEARCH_FAR   full turn the other way,             LINE_SEARCH2_TICKS
 *   FAILED       error 64; Subsystem 2 must stop the car
 * Any black on the centre or right sensor returns to TRACKING at once.
 *
 * Timing constants are in 10 ms task cycles and MUST be tuned on the
 * track together with Buddy 2's steering gains (see TODO_TUNE below).
 *
 * Pure C, no kernel calls: also built by the PC tests.
 */

#ifndef LINE_LOGIC_H
#define LINE_LOGIC_H

#include <stdbool.h>
#include <stdint.h>

// Error offsets from the centre value 64 (7-bit field, 0..127)
#define LINE_ERR_NEAR           (10U)   // centre + right black TODO_TUNE
#define LINE_ERR_SIDE           (32U)   // right sensor saw it  TODO_TUNE
#define LINE_ERR_GUESS          (16U)   // guessed left         TODO_TUNE
#define LINE_ERR_FULL           (63U)   // search: 1 or 127

// Debounce: a sensor must read the same this many cycles in a row
#define LINE_DEBOUNCE_TICKS     (2U)    // 20 ms

// Junction: all three sensors black for this many debounced cycles
// Longer than a barcode's narrow first bar, shorter than the 18 mm
// crossing line, at driving speed. Limits the car to about 180 mm/s.
#define LINE_JUNCTION_TICKS     (8U)    // 80 ms               TODO_TUNE

// Line lost and search phases (10 ms cycles)                 TODO_TUNE
#define LINE_LOST_TICKS         (150U)  // 1.5 s with no black reading
#define LINE_SEARCH1_TICKS      (150U)  // 1.5 s toward the last side
#define LINE_SEARCH2_TICKS      (300U)  // 3.0 s the other way

// Which side the line goes to if the centre sensor loses it
#define LINE_SIDE_NONE          (0U)
#define LINE_SIDE_LEFT          (1U)
#define LINE_SIDE_RIGHT         (2U)

/*
 * Line state. Owned by the Line task only (never shared), so C
 * bit-fields are allowed (Change Requirement 1, task-private data).
 * Widths: ticks up to 300 need 9 bits; debounce counters need 2 bits.
 * 27 bits in all.
 */
typedef struct
{
    unsigned timer        : 9;  // cycles in the current phase, 0..511
    unsigned mode         : 3;  // LINE_MODE_* (line_if.h)
    unsigned last_side    : 2;  // LINE_SIDE_*
    unsigned b_centre     : 1;  // debounced centre sensor, 1 = black
    unsigned b_right      : 1;  // debounced right sensor, 1 = black
    unsigned b_barcode    : 1;  // debounced barcode sensor, 1 = black
    unsigned centre_count : 2;  // cycles the raw centre reading differed
    unsigned right_count  : 2;  // cycles the raw right reading differed
    unsigned bc_count     : 2;  // cycles the raw barcode reading differed
    unsigned all_count    : 4;  // cycles all three have been black
} line_logic_t;

void line_logic_init(line_logic_t *p_ctx);

/*!
 * One 10 ms step. b_centre_black / b_right_black / b_barcode_black are
 * the raw sensor readings (true = black); the barcode sensor is used
 * only to recognise a crossing line. Writes the 7-bit error (64 =
 * centre) and the LINE_MODE_* to *p_error and *p_mode.
 */
void line_logic_step(line_logic_t *p_ctx, bool b_centre_black,
                     bool b_right_black, bool b_barcode_black,
                     uint8_t *p_error, uint8_t *p_mode);

#endif /* LINE_LOGIC_H */
