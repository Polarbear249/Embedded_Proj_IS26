/*
 * line_if.h
 * Subsystem 3 (Buddy 3): IF-06 line_state word, shared with Subsystem 2
 * (Motion Control) and Subsystem 1 (telemetry).
 *
 * One aligned 32-bit word, written ONLY by the Line / Barcode task every
 * 10 ms with one atomic store, read by anyone with one atomic load.
 * No mutex: one writer, readers never write (TEAM.md IF-06).
 *
 * Change Requirement 1 (minimum-bit data): every field uses only the
 * bits its range needs, packed with explicit masks and shifts (never C
 * bit-fields) so the bit order is the same in every module and in the
 * PC-side MQTT decoder.
 *
 *   bit 31..24  seq       8  publish counter, +1 every 10 ms, wraps
 *   bit 23..22  reserved  2  always 0
 *   bit 21..17  bc_char   5  latest decoded character, 0..25 = 'A'..'Z',
 *                            LINE_CHAR_NONE (31) until the first decode
 *   bit 16      raw_bc    1  barcode sensor DOUT now (1 = black)
 *   bit 15      raw_right 1  right line sensor now (1 = black)
 *   bit 14      raw_c     1  centre line sensor now (1 = black)
 *   bit 13..12  char_seq  2  +1 for every newly decoded barcode, wraps
 *   bit 11..10  reserved  2  always 0
 *   bit  9..7   mode      3  LINE_MODE_*
 *   bit  6..0   error     7  line position, offset binary, 64 = centre
 *
 * error: where the line is relative to the car's centre line.
 *   < 64  line is to the LEFT  -> steer left
 *   = 64  centred              -> go straight
 *   > 64  line is to the RIGHT -> steer right
 * Same 7-bit offset-binary format as line_err in Buddy 1's telemetry
 * frame (subsystem1/telemetry_frame.h), so it is copied as-is.
 *
 * Barcode (CLAUDE.md 1b): S3 reports the decoded CHARACTER only, never a
 * movement. The character-to-movement mapping arrives over MQTT at run
 * time and the Vehicle Controller applies it, so no mapping is compiled
 * in here. A reader remembers char_seq; when it changes, bc_char holds
 * a new character. Nothing has to be acknowledged, so no reader ever
 * writes shared data.
 *
 * This header has no kernel dependency so it also builds on a PC.
 */

#ifndef LINE_IF_H
#define LINE_IF_H

#include <stdbool.h>
#include <stdint.h>

// Field positions and widths (masks are applied after the shift)
#define LINE_IF_ERROR_SHIFT     (0U)
#define LINE_IF_ERROR_MASK      (0x7FU)     // 7 bits
#define LINE_IF_MODE_SHIFT      (7U)
#define LINE_IF_MODE_MASK       (0x07U)     // 3 bits
#define LINE_IF_CHAR_SEQ_SHIFT  (12U)
#define LINE_IF_CHAR_SEQ_MASK   (0x03U)     // 2 bits
#define LINE_IF_CHAR_SHIFT      (17U)
#define LINE_IF_CHAR_MASK       (0x1FU)     // 5 bits
#define LINE_IF_RAW_C_SHIFT     (14U)
#define LINE_IF_RAW_R_SHIFT     (15U)
#define LINE_IF_RAW_BC_SHIFT    (16U)
#define LINE_IF_RAW_MASK        (0x01U)     // 1 bit each
#define LINE_IF_SEQ_SHIFT       (24U)
#define LINE_IF_SEQ_MASK        (0xFFU)     // 8 bits

// error field values
#define LINE_IF_ERROR_CENTRE    (64U)
#define LINE_IF_ERROR_MAX       (127U)

// mode field values (6 used of 8)
#define LINE_MODE_INIT          (0U)    // task not yet running
#define LINE_MODE_TRACKING      (1U)    // line between or under sensors
#define LINE_MODE_JUNCTION      (2U)    // both sensors on black
#define LINE_MODE_SEARCH_NEAR   (3U)    // lost: turning to last side seen
#define LINE_MODE_SEARCH_FAR    (4U)    // lost: turning the other way
#define LINE_MODE_FAILED        (5U)    // search gave up: stop the car

// bc_char field values: 0..25 = 'A'..'Z' (character - 'A')
#define LINE_CHAR_MAX           (25U)   // 'Z'
#define LINE_CHAR_NONE          (31U)   // nothing decoded yet

// Reads without a change in seq before the data counts as stale (NFR18)
#define LINE_IF_STALE_READS     (3U)

// Range guards: each value must fit its field
_Static_assert(LINE_IF_ERROR_MAX <= LINE_IF_ERROR_MASK, "error width");
_Static_assert(LINE_MODE_FAILED <= LINE_IF_MODE_MASK, "mode width");
_Static_assert(LINE_CHAR_NONE <= LINE_IF_CHAR_MASK, "char width");
_Static_assert(LINE_CHAR_MAX < LINE_CHAR_NONE, "char none");

// Unpacked copy of the word, for code that prefers named fields
typedef struct
{
    uint8_t seq;            // 0..255
    uint8_t mode;           // LINE_MODE_*
    uint8_t error;          // 0..127, 64 = centre
    uint8_t bc_char;        // 0..25 = 'A'..'Z', or LINE_CHAR_NONE
    uint8_t char_seq;       // 0..3
    bool    b_centre_black;
    bool    b_right_black;
    bool    b_barcode_black;
} line_if_state_t;

// Reader-side staleness tracker: one per reading task, never shared
typedef struct
{
    uint8_t last_seq;
    uint8_t same_count;     // reads in a row with the same seq
} line_if_reader_t;

uint32_t line_if_pack(line_if_state_t const *p_state);
void     line_if_unpack(uint32_t word, line_if_state_t *p_state);

void     line_if_reader_init(line_if_reader_t *p_reader, uint32_t word);
bool     line_if_is_fresh(line_if_reader_t *p_reader, uint32_t word);

// Signed steering error, -64..+63: negative = line is to the left
int8_t   line_if_error_signed(uint32_t word);

// One-field accessors (no struct needed)
uint8_t  line_if_mode(uint32_t word);
uint8_t  line_if_char(uint32_t word);
uint8_t  line_if_char_seq(uint32_t word);

#endif /* LINE_IF_H */
