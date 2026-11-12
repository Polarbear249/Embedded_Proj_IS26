/*
 * line_if.c
 * Subsystem 3 (Buddy 3): pack / unpack the IF-06 line_state word.
 * Pure functions, no kernel calls: also built by the PC tests.
 * Bit layout: see line_if.h.
 */

#include <stddef.h>
#include "line_if.h"

// Bit position of a 1-bit flag as a mask
#define LINE_IF_BIT(shift)      ((uint32_t)1U << (shift))

/*!
 * Build the 32-bit word from named fields. Each field is masked to its
 * width, so an out-of-range value can never spill into its neighbour.
 */
uint32_t
line_if_pack (line_if_state_t const *p_state)
{
    uint32_t word = 0U;

    if (NULL != p_state)
    {
        word |= ((uint32_t)p_state->error & LINE_IF_ERROR_MASK)
                << LINE_IF_ERROR_SHIFT;
        word |= ((uint32_t)p_state->mode & LINE_IF_MODE_MASK)
                << LINE_IF_MODE_SHIFT;
        word |= ((uint32_t)p_state->bc_char & LINE_IF_CHAR_MASK)
                << LINE_IF_CHAR_SHIFT;
        word |= ((uint32_t)p_state->char_seq & LINE_IF_CHAR_SEQ_MASK)
                << LINE_IF_CHAR_SEQ_SHIFT;
        word |= ((uint32_t)p_state->seq & LINE_IF_SEQ_MASK)
                << LINE_IF_SEQ_SHIFT;

        if (p_state->b_centre_black)
        {
            word |= LINE_IF_BIT(LINE_IF_RAW_C_SHIFT);
        }
        else
        {
            // bit stays 0: white
        }

        if (p_state->b_right_black)
        {
            word |= LINE_IF_BIT(LINE_IF_RAW_R_SHIFT);
        }
        else
        {
            // bit stays 0: white
        }

        if (p_state->b_barcode_black)
        {
            word |= LINE_IF_BIT(LINE_IF_RAW_BC_SHIFT);
        }
        else
        {
            // bit stays 0: white
        }
    }
    else
    {
        // NULL input: publish an all-zero word (mode INIT)
    }

    return word;
}

/*!
 * Split the word back into named fields.
 */
void
line_if_unpack (uint32_t word, line_if_state_t *p_state)
{
    if (NULL != p_state)
    {
        // Each cast keeps the low 8 bits; the mask already limits the
        // value to its field width (at most 8 bits), so nothing is lost.
        p_state->error   = (uint8_t)((word >> LINE_IF_ERROR_SHIFT)
                                     & LINE_IF_ERROR_MASK);
        p_state->mode    = (uint8_t)((word >> LINE_IF_MODE_SHIFT)
                                     & LINE_IF_MODE_MASK);
        p_state->bc_char  = (uint8_t)((word >> LINE_IF_CHAR_SHIFT)
                                      & LINE_IF_CHAR_MASK);
        p_state->char_seq = (uint8_t)((word >> LINE_IF_CHAR_SEQ_SHIFT)
                                      & LINE_IF_CHAR_SEQ_MASK);
        p_state->seq     = (uint8_t)((word >> LINE_IF_SEQ_SHIFT)
                                     & LINE_IF_SEQ_MASK);

        p_state->b_centre_black =
            (0U != (word & LINE_IF_BIT(LINE_IF_RAW_C_SHIFT)));
        p_state->b_right_black =
            (0U != (word & LINE_IF_BIT(LINE_IF_RAW_R_SHIFT)));
        p_state->b_barcode_black =
            (0U != (word & LINE_IF_BIT(LINE_IF_RAW_BC_SHIFT)));
    }
    else
    {
        // nothing to fill
    }
}

/*!
 * Start tracking staleness from the word read right now.
 */
void
line_if_reader_init (line_if_reader_t *p_reader, uint32_t word)
{
    if (NULL != p_reader)
    {
        // Cast keeps 8 bits; the mask limits seq to 8 bits
        p_reader->last_seq   = (uint8_t)((word >> LINE_IF_SEQ_SHIFT)
                                         & LINE_IF_SEQ_MASK);
        p_reader->same_count = 0U;
    }
    else
    {
        // nothing to initialise
    }
}

/*!
 * Call once per read. Returns false once seq has not changed for
 * LINE_IF_STALE_READS reads in a row: the Line task has stalled and the
 * reader must stop the car safely (NFR18).
 */
bool
line_if_is_fresh (line_if_reader_t *p_reader, uint32_t word)
{
    bool    b_fresh = false;
    uint8_t seq;

    if (NULL != p_reader)
    {
        // Cast keeps 8 bits; the mask limits seq to 8 bits
        seq = (uint8_t)((word >> LINE_IF_SEQ_SHIFT) & LINE_IF_SEQ_MASK);

        if (seq != p_reader->last_seq)
        {
            p_reader->last_seq   = seq;
            p_reader->same_count = 0U;
        }
        else if (p_reader->same_count < LINE_IF_STALE_READS)
        {
            p_reader->same_count++;
        }
        else
        {
            // already stale: hold the count so it cannot wrap
        }

        b_fresh = (p_reader->same_count < LINE_IF_STALE_READS);
    }
    else
    {
        // no tracker: report stale so the caller fails safe
    }

    return b_fresh;
}

/*!
 * Steering error as a signed number, -64..+63.
 */
int8_t
line_if_error_signed (uint32_t word)
{
    uint32_t field = (word >> LINE_IF_ERROR_SHIFT) & LINE_IF_ERROR_MASK;

    // field is 0..127, so field - 64 is -64..+63: fits int8_t. The
    // subtraction is done in int16_t, after the bitwise work, so no
    // bitwise operator ever touches a signed value.
    return (int8_t)((int16_t)field - (int16_t)LINE_IF_ERROR_CENTRE);
}

uint8_t
line_if_mode (uint32_t word)
{
    // Cast keeps 3 bits: the mask limits the value to 0..7
    return (uint8_t)((word >> LINE_IF_MODE_SHIFT) & LINE_IF_MODE_MASK);
}

uint8_t
line_if_char (uint32_t word)
{
    // Cast keeps 5 bits: the mask limits the value to 0..31
    return (uint8_t)((word >> LINE_IF_CHAR_SHIFT) & LINE_IF_CHAR_MASK);
}

uint8_t
line_if_char_seq (uint32_t word)
{
    // Cast keeps 2 bits: the mask limits the value to 0..3
    return (uint8_t)((word >> LINE_IF_CHAR_SEQ_SHIFT)
                     & LINE_IF_CHAR_SEQ_MASK);
}
