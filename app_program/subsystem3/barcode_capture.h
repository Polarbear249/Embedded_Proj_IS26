/*
 * barcode_capture.h
 * Subsystem 3 (Buddy 3): turns barcode-sensor edges into element widths
 * and hands complete symbols to the Code 39 decoder.
 *
 * Input: level changes of the barcode sensor (GP7), each with a
 * microsecond timestamp, captured by the GPIO interrupt (drv_ir.c).
 *
 * How a symbol is found:
 *   - A symbol starts with a white -> black change after a "quiet"
 *     white (longer than the quiet limit below).
 *   - Every following element width is stored. When the 29th element
 *     (the last bar of "*X*") ends, the symbol is decoded at once.
 *   - Quiet limit = BC_CAP_QUIET_FACTOR x the widest element seen so
 *     far, never less than BC_CAP_QUIET_MIN_US. Inside a symbol no
 *     element is more than 3x the narrowest, so a longer white (or
 *     black) means the run has ended or was not a barcode (e.g. the
 *     sensor crossed the 18 mm line on a curve).
 *   - Comparator chatter: the TCRT5000 module's comparator flickers for
 *     tens of microseconds at every bar edge (measured on the car
 *     2026-11-01: bursts of 5-10 pulses of 0-64 us). A change only
 *     counts once the level has stayed put for BC_CAP_SETTLE_US; it is
 *     then dated at the FIRST edge of its burst, so widths stay exact.
 *     A burst that ends on the old level (a spike) changes nothing.
 *
 * Pure C, no kernel calls: also built by the PC tests.
 */

#ifndef BARCODE_CAPTURE_H
#define BARCODE_CAPTURE_H

#include <stdbool.h>
#include <stdint.h>
#include "barcode_code39.h"

// A level must hold this long to count. At 1 m/s (CMD_SPEED_MAX, Buddy
// 1) a 3 mm narrow bar lasts 3000 us, so 300 us cannot remove a real
// bar; measured chatter pulses are <= 64 us.
#define BC_CAP_SETTLE_US        (300U)
#define BC_CAP_QUIET_FACTOR     (5U)
#define BC_CAP_QUIET_MIN_US     (10000U)

// Longest element that can be stored: widths are kept in 8 us units in
// 16 bits (Change Requirement 1), so 65535 x 8 us = 524 ms. That is a
// 9 mm wide bar at 17 mm/s, slower than the car will ever cross one.
#define BC_CAP_UNIT_SHIFT       (3U)            // 1 unit = 8 us
#define BC_CAP_WIDTH_MAX        (0xFFFFU)

// A run that ends early with at least this many elements is reported
// as a failed read. Runs of BC_CAP_SHORT_MIN.. up to that are reported
// as BC_CAP_SHORT (bring-up diagnostics); shorter ones are ignored.
#define BC_CAP_MIN_REPORT       (BC39_ELEMENTS_PER_CHAR)
#define BC_CAP_SHORT_MIN        (3U)

// Timestamps from the interrupt are 31 bits (bit 31 holds the level)
#define BC_CAP_TIME_MASK        (0x7FFFFFFFU)

typedef enum
{
    BC_CAP_NONE = 0,        // nothing new
    BC_CAP_DECODED,         // *p_char holds the data character
    BC_CAP_FAILED,          // a barcode-like run did not decode
    BC_CAP_SHORT            // a run of 3..8 elements ended (diagnostic)
} bc_cap_result_t;

/*
 * Capture state. Owned by the Line task only (never shared), so C
 * bit-fields are allowed here (Change Requirement 1, task-private data).
 */
typedef struct
{
    uint16_t width[BC39_SYMBOL_ELEMENTS];   // element widths, 8 us units
    uint32_t level_start_us;    // when the settled level began
    uint32_t raw_last_us;       // time of the latest raw edge
    uint32_t burst_start_us;    // first raw edge since the raw level
                                // last held for BC_CAP_SETTLE_US
    uint32_t widest_us;         // widest element of the current run
    char     decoded_char;      // result waiting for bc_capture_poll()
    uint8_t  count;             // elements stored, 0..29
    uint8_t  last_count;        // elements in the run that just ended;
                                // their widths stay in width[] until
                                // the next run overwrites them
    unsigned b_black      : 1;  // settled sensor level
    unsigned b_raw_black  : 1;  // latest raw level (may be chattering)
    unsigned state        : 2;  // idle / run / wait-quiet
    unsigned pending      : 2;  // bc_cap_result_t waiting to be polled
    unsigned last_status  : 3;  // bc39_status_t of the last decode
} bc_capture_t;

void bc_capture_init(bc_capture_t *p_cap, bool b_black_now,
                     uint32_t now_us);

/*!
 * Feed one level change. b_black is the NEW level. Calls with the same
 * level as the current one are ignored.
 */
void bc_capture_edge(bc_capture_t *p_cap, bool b_black, uint32_t time_us);

/*!
 * Call every task cycle: ends runs that went quiet, and returns any
 * result. On BC_CAP_DECODED, *p_char is the data character.
 */
bc_cap_result_t bc_capture_poll(bc_capture_t *p_cap, uint32_t now_us,
                                char *p_char);

/*!
 * Drop the current run (used after the edge ring overflowed).
 */
void bc_capture_abort(bc_capture_t *p_cap, uint32_t now_us);

/*!
 * True once a run has stored at least one element, i.e. the barcode
 * sensor's black is a barcode bar, not a crossing line. A run's FIRST
 * bar returns false: a crossing line starts a run too, and it must
 * still count as a junction.
 */
bool bc_capture_busy(bc_capture_t const *p_cap);

#endif /* BARCODE_CAPTURE_H */
