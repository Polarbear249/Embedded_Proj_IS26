/*
 * barcode_code39.h
 * Subsystem 3 (Buddy 3): Code 39 decoder for the track barcodes.
 *
 * Track symbols are one data character between start/stop '*', e.g.
 * "*A*" (docs/Barcode Sample.pdf). Every Code 39 character is 9
 * elements (5 bars, 4 spaces), exactly 3 of them wide. Characters are
 * separated by one narrow gap, so a whole symbol is
 *   3 chars x 9 elements + 2 gaps = 29 elements (15 bars, 14 spaces).
 *
 * Wide vs narrow is decided by RANK inside each character (the 3 widest
 * of 9 are wide), never by an absolute width, so decoding does not
 * depend on car speed. The symbol is tried forwards, then backwards,
 * so the car may cross a barcode in either direction.
 *
 * Pure C, no kernel calls: also built by the PC tests.
 */

#ifndef BARCODE_CODE39_H
#define BARCODE_CODE39_H

#include <stdbool.h>
#include <stdint.h>

#define BC39_ELEMENTS_PER_CHAR  (9U)
#define BC39_WIDE_PER_CHAR      (3U)
#define BC39_SYMBOL_CHARS       (3U)    // '*', data, '*'
#define BC39_SYMBOL_ELEMENTS    (29U)   // 3 x 9 + 2 gaps

// Narrowest wide element must be at least this many times the widest
// narrow one (as NUM/DEN, integer maths). Printed ratio is 3.0; 1.5
// leaves room for sensor spot blur and speed changes inside a symbol.
#define BC39_MIN_RATIO_NUM      (3U)
#define BC39_MIN_RATIO_DEN      (2U)

typedef enum
{
    BC39_OK = 0,
    BC39_ERR_NULL,          // NULL pointer argument
    BC39_ERR_LENGTH,        // not exactly BC39_SYMBOL_ELEMENTS elements
    BC39_ERR_RATIO,         // wide and narrow too close to tell apart
    BC39_ERR_PATTERN,       // 9-element pattern is not a Code 39 char
    BC39_ERR_FRAME          // not '*' data '*' in either direction
} bc39_status_t;

/*!
 * Decode one symbol from its element widths (any unit, e.g. us).
 * p_width[0] must be the first bar; elements alternate bar / space.
 * On BC39_OK, *p_char is the data character (e.g. 'A') and
 * *p_b_reversed tells whether it was read backwards.
 */
bc39_status_t bc39_decode(uint16_t const p_width[], uint8_t count,
                          char *p_char, bool *p_b_reversed);

/*!
 * The 9-bit wide/narrow pattern of a character (bit 8 = first element,
 * 1 = wide), or 0 if the character is not in Code 39. Used by tests.
 */
uint16_t bc39_pattern_of(char character);

#endif /* BARCODE_CODE39_H */
