/*
 * line_logic.h
 * Subsystem 3 (Buddy 3): line position from two on/off IR sensors.
 *
 * Sensor layout (agreed 2026-11-01): the left (GP26) and right (GP6)
 * TCRT5000 sensors sit OUTSIDE the 18 mm line, one on each side.
 *
 *   left right   meaning                          error (64 = centre)
 *   ---- -----   -------------------------------  -------------------
 *   W    W       line between the sensors         64 -/+ weave bias
 *   B    W       line under left sensor           64 - SIDE (steer L)
 *   W    B       line under right sensor          64 + SIDE (steer R)
 *   B    B       junction / crossing line         64, mode JUNCTION
 *
 * "Centred" and "lost" both read white/white. To tell them apart the
 * white/white output is biased a little toward the side where the line
 * was last seen. The car then weaves gently and touches the line every
 * so often. If neither sensor sees black for LINE_LOST_TICKS, the line
 * is lost, and the bias already points the right way to search:
 *
 *   SEARCH_NEAR  full turn toward the last side seen, LINE_SEARCH1_TICKS
 *   SEARCH_FAR   full turn the other way,             LINE_SEARCH2_TICKS
 *   FAILED       error 64; Subsystem 2 must stop the car
 *
 * Any black reading during a search returns to TRACKING at once.
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
#define LINE_ERR_BIAS           (6U)    // white/white weave   TODO_TUNE
#define LINE_ERR_SIDE           (32U)   // one sensor on line  TODO_TUNE
#define LINE_ERR_FULL           (63U)   // search: 1 or 127

// Debounce: a sensor must read the same this many cycles in a row
#define LINE_DEBOUNCE_TICKS     (2U)    // 20 ms

// Junction: both black for this many debounced cycles
#define LINE_JUNCTION_TICKS     (2U)    // 20 ms

// Line lost and search phases (10 ms cycles)                 TODO_TUNE
#define LINE_LOST_TICKS         (150U)  // 1.5 s with no black reading
#define LINE_SEARCH1_TICKS      (150U)  // 1.5 s toward the last side
#define LINE_SEARCH2_TICKS      (300U)  // 3.0 s the other way

// Which side the line was last seen on
#define LINE_SIDE_NONE          (0U)
#define LINE_SIDE_LEFT          (1U)
#define LINE_SIDE_RIGHT         (2U)

/*
 * Line state. Owned by the Line task only (never shared), so C
 * bit-fields are allowed (Change Requirement 1, task-private data).
 * Widths: ticks up to 300 need 9 bits; debounce counters need 2 bits.
 */
typedef struct
{
    unsigned timer        : 9;  // cycles in the current phase, 0..511
    unsigned mode         : 3;  // LINE_MODE_* (line_if.h)
    unsigned last_side    : 2;  // LINE_SIDE_*
    unsigned b_left       : 1;  // debounced left sensor, 1 = black
    unsigned b_right      : 1;  // debounced right sensor, 1 = black
    unsigned left_count   : 2;  // cycles the raw left reading differed
    unsigned right_count  : 2;  // cycles the raw right reading differed
    unsigned both_count   : 2;  // cycles both have been black
} line_logic_t;

void line_logic_init(line_logic_t *p_ctx);

/*!
 * One 10 ms step. b_left_black / b_right_black are the raw sensor
 * readings (true = black). Writes the 7-bit error (64 = centre) and the
 * LINE_MODE_* to *p_error and *p_mode.
 */
void line_logic_step(line_logic_t *p_ctx, bool b_left_black,
                     bool b_right_black, uint8_t *p_error,
                     uint8_t *p_mode);

#endif /* LINE_LOGIC_H */
