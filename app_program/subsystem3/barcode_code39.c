/*
 * barcode_code39.c
 * Subsystem 3 (Buddy 3): Code 39 decoder, see barcode_code39.h.
 *
 * Pattern table: 9 bits per character, bit 8 = first element (a bar),
 * 1 = wide. Checked against:
 *   - docs/Barcode Sample.pdf: '*', 'A' and 'Z' measured from the bars;
 *   - the Code 39 module strings of 'A'..'D' (1 = black module):
 *     A 110101001011, B 101101001011, C 110110100101, D 101011001011.
 * The PC tests (testing/subsystem3/pc) also check that all 44 patterns
 * are distinct and have exactly 3 wide elements.
 */

#include <stddef.h>
#include "barcode_code39.h"

#define BC39_TABLE_SIZE         (44U)
#define BC39_GAP_ELEMENTS       (1U)    // narrow space between chars
#define BC39_CHAR_STRIDE        (BC39_ELEMENTS_PER_CHAR + BC39_GAP_ELEMENTS)
#define BC39_NO_INDEX           (0xFFU)
#define BC39_START_STOP         ('*')

typedef struct
{
    char     character;
    uint16_t pattern;       // 9 bits used
} bc39_entry_t;

static bc39_entry_t const g_bc39_table[BC39_TABLE_SIZE] =
{
    {'0', 0x034U}, {'1', 0x121U}, {'2', 0x061U}, {'3', 0x160U},
    {'4', 0x031U}, {'5', 0x130U}, {'6', 0x070U}, {'7', 0x025U},
    {'8', 0x124U}, {'9', 0x064U}, {'A', 0x109U}, {'B', 0x049U},
    {'C', 0x148U}, {'D', 0x019U}, {'E', 0x118U}, {'F', 0x058U},
    {'G', 0x00DU}, {'H', 0x10CU}, {'I', 0x04CU}, {'J', 0x01CU},
    {'K', 0x103U}, {'L', 0x043U}, {'M', 0x142U}, {'N', 0x013U},
    {'O', 0x112U}, {'P', 0x052U}, {'Q', 0x007U}, {'R', 0x106U},
    {'S', 0x046U}, {'T', 0x016U}, {'U', 0x181U}, {'V', 0x0C1U},
    {'W', 0x1C0U}, {'X', 0x091U}, {'Y', 0x190U}, {'Z', 0x0D0U},
    {'-', 0x085U}, {'.', 0x184U}, {' ', 0x0C4U}, {'$', 0x0A8U},
    {'/', 0x0A2U}, {'+', 0x08AU}, {'%', 0x02AU}, {'*', 0x094U}
};

static uint16_t      bc39_width_at(uint16_t const p_width[], uint8_t count,
                                   uint8_t index, bool b_reversed);
static bc39_status_t bc39_char_pattern(uint16_t const p_width[],
                                       uint8_t count, uint8_t first,
                                       bool b_reversed,
                                       uint16_t *p_pattern);
static char          bc39_char_of(uint16_t pattern);
static bc39_status_t bc39_decode_dir(uint16_t const p_width[],
                                     uint8_t count, bool b_reversed,
                                     char *p_char);

bc39_status_t
bc39_decode (uint16_t const p_width[], uint8_t count, char *p_char,
             bool *p_b_reversed)
{
    bc39_status_t status;
    char          data_char = '\0';

    if ((NULL == p_width) || (NULL == p_char) || (NULL == p_b_reversed))
    {
        status = BC39_ERR_NULL;
    }
    else if (BC39_SYMBOL_ELEMENTS != count)
    {
        status = BC39_ERR_LENGTH;
    }
    else
    {
        // Forwards first; a backwards read starts with the reverse of
        // '*' (which is 'P'), so it can never pass as a forward symbol.
        *p_b_reversed = false;
        status = bc39_decode_dir(p_width, count, false, &data_char);

        if (BC39_OK != status)
        {
            if (BC39_OK == bc39_decode_dir(p_width, count, true,
                                           &data_char))
            {
                *p_b_reversed = true;
                status = BC39_OK;
            }
            else
            {
                // keep the forward error: it says what went wrong first
            }
        }
        else
        {
            // forward read succeeded
        }

        *p_char = data_char;
    }

    return status;
}

uint16_t
bc39_pattern_of (char character)
{
    uint16_t pattern = 0U;
    uint8_t  idx;

    for (idx = 0U; (idx < BC39_TABLE_SIZE) && (0U == pattern); idx++)
    {
        if (character == g_bc39_table[idx].character)
        {
            pattern = g_bc39_table[idx].pattern;
        }
        else
        {
            // keep looking
        }
    }

    return pattern;
}

/*!
 * Width of element 'index', counted from the end when reading backwards.
 */
static uint16_t
bc39_width_at (uint16_t const p_width[], uint8_t count, uint8_t index,
               bool b_reversed)
{
    uint8_t pos = index;

    if (b_reversed)
    {
        // index < count is guaranteed by the callers, so this is >= 0
        pos = (uint8_t)((count - 1U) - index);
    }
    else
    {
        // forward: element order as captured
    }

    return p_width[pos];
}

/*!
 * Classify the 9 elements starting at 'first': the 3 widest are wide.
 * Fails if the narrowest wide element is not clearly wider than the
 * widest narrow one.
 */
static bc39_status_t
bc39_char_pattern (uint16_t const p_width[], uint8_t count, uint8_t first,
                   bool b_reversed, uint16_t *p_pattern)
{
    bc39_status_t status = BC39_OK;
    uint16_t      width[BC39_ELEMENTS_PER_CHAR];
    uint16_t      wide_flags = 0U;      // bit i set = element i is wide
    uint16_t      min_wide   = 0U;
    uint16_t      max_narrow = 0U;
    uint16_t      pattern    = 0U;
    uint8_t       pick;
    uint8_t       idx;
    uint8_t       best;

    for (idx = 0U; idx < BC39_ELEMENTS_PER_CHAR; idx++)
    {
        // first + idx < 29 for every caller, so it fits uint8_t
        width[idx] = bc39_width_at(p_width, count,
                                   (uint8_t)(first + idx), b_reversed);
    }

    // Pick the 3 widest, one at a time
    for (pick = 0U; pick < BC39_WIDE_PER_CHAR; pick++)
    {
        best = BC39_NO_INDEX;

        for (idx = 0U; idx < BC39_ELEMENTS_PER_CHAR; idx++)
        {
            if (0U != (wide_flags & ((uint16_t)1U << idx)))
            {
                // already picked
            }
            else if ((BC39_NO_INDEX == best) || (width[idx] > width[best]))
            {
                best = idx;
            }
            else
            {
                // narrower than the best so far
            }
        }

        wide_flags |= (uint16_t)((uint16_t)1U << best);
        min_wide    = width[best];      // last pick is the narrowest wide
    }

    // Widest of the 6 narrow elements
    for (idx = 0U; idx < BC39_ELEMENTS_PER_CHAR; idx++)
    {
        if ((0U == (wide_flags & ((uint16_t)1U << idx)))
            && (width[idx] > max_narrow))
        {
            max_narrow = width[idx];
        }
        else
        {
            // wide element, or not the widest narrow one
        }
    }

    // Ratio check in 32 bits: 65535 x 3 cannot overflow
    if ((0U == min_wide)
        || (((uint32_t)min_wide * BC39_MIN_RATIO_DEN)
            < ((uint32_t)max_narrow * BC39_MIN_RATIO_NUM)))
    {
        status = BC39_ERR_RATIO;
    }
    else
    {
        // Build the pattern, first element in bit 8
        for (idx = 0U; idx < BC39_ELEMENTS_PER_CHAR; idx++)
        {
            pattern = (uint16_t)(pattern << 1U);

            if (0U != (wide_flags & ((uint16_t)1U << idx)))
            {
                pattern |= 1U;
            }
            else
            {
                // narrow: bit stays 0
            }
        }
    }

    *p_pattern = pattern;

    return status;
}

/*!
 * Character for a pattern, or '\0' if no Code 39 character matches.
 */
static char
bc39_char_of (uint16_t pattern)
{
    char    character = '\0';
    uint8_t idx;

    for (idx = 0U; (idx < BC39_TABLE_SIZE) && ('\0' == character); idx++)
    {
        if (pattern == g_bc39_table[idx].pattern)
        {
            character = g_bc39_table[idx].character;
        }
        else
        {
            // keep looking
        }
    }

    return character;
}

/*!
 * Decode the 3 characters in one direction and check the '*' frame.
 */
static bc39_status_t
bc39_decode_dir (uint16_t const p_width[], uint8_t count, bool b_reversed,
                 char *p_char)
{
    bc39_status_t status = BC39_OK;
    char          chars[BC39_SYMBOL_CHARS] = {'\0', '\0', '\0'};
    uint16_t      pattern = 0U;
    uint8_t       pos;

    for (pos = 0U; (pos < BC39_SYMBOL_CHARS) && (BC39_OK == status); pos++)
    {
        // pos * 10 <= 20, fits uint8_t
        status = bc39_char_pattern(p_width, count,
                                   (uint8_t)(pos * BC39_CHAR_STRIDE),
                                   b_reversed, &pattern);

        if (BC39_OK == status)
        {
            chars[pos] = bc39_char_of(pattern);

            if ('\0' == chars[pos])
            {
                status = BC39_ERR_PATTERN;
            }
            else
            {
                // valid character
            }
        }
        else
        {
            // ratio error ends the loop
        }
    }

    if (BC39_OK == status)
    {
        if ((BC39_START_STOP == chars[0])
            && (BC39_START_STOP == chars[2])
            && (BC39_START_STOP != chars[1]))
        {
            *p_char = chars[1];
        }
        else
        {
            status = BC39_ERR_FRAME;
        }
    }
    else
    {
        // keep the error
    }

    return status;
}
