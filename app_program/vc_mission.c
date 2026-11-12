/*
 * vc_mission.c
 * Vehicle Controller mission logic, see vc_mission.h. Owner: Buddy 4.
 * Pure functions (no kernel calls), also built by the PC tests in
 * testing/vehicle_controller/pc/.
 */

#include "vc_mission.h"
#include <stddef.h>

// Bit position of the character for one movement
#define VC_MAP_SHIFT(move)      ((uint32_t)(move) * VC_CHAR_BITS)

static bool vc_chars_ok(uint8_t const p_chars[VC_MOVE_COUNT]);

/*!
 * Pack four characters (index = VC_MOVE_*) into a mapping word. Returns
 * false, and leaves *p_word alone, if a character is out of range or two
 * movements share a character.
 */
bool
vc_map_pack (uint8_t const p_chars[VC_MOVE_COUNT], uint32_t *p_word)
{
    bool     b_ok = false;
    uint32_t word = VC_MAP_VALID_BIT;
    uint8_t  move;

    if ((NULL != p_chars) && (NULL != p_word) && vc_chars_ok(p_chars))
    {
        for (move = 0U; move < VC_MOVE_COUNT; move++)
        {
            word |= ((uint32_t)p_chars[move] & VC_CHAR_MASK)
                    << VC_MAP_SHIFT(move);
        }

        *p_word = word;
        b_ok = true;
    }
    else
    {
        // invalid input: nothing written
    }

    return (b_ok);
}

/*!
 * True for a complete, checked mapping: valid bit set, unused bits 0,
 * four distinct characters 0..25.
 */
bool
vc_map_is_valid (uint32_t word)
{
    bool    b_valid = false;
    uint8_t chars[VC_MOVE_COUNT];
    uint8_t move;

    if ((0U != (word & VC_MAP_VALID_BIT))
        && (0U == (word & ~VC_MAP_USED_MASK)))
    {
        for (move = 0U; move < VC_MOVE_COUNT; move++)
        {
            chars[move] = vc_map_char(word, move);
        }

        b_valid = vc_chars_ok(chars);
    }
    else
    {
        // no valid bit, or stray high bits
    }

    return (b_valid);
}

/*!
 * Character stored for one movement (no validity check).
 */
uint8_t
vc_map_char (uint32_t word, uint8_t move)
{
    // Cast keeps 5 bits: the mask limits the value to 0..31
    return ((uint8_t)((word >> VC_MAP_SHIFT(move & VC_MOVE_MASK))
                      & VC_CHAR_MASK));
}

/*!
 * Movement mapped to bc_char. False if the word is not valid or the
 * character is not one of the four.
 */
bool
vc_map_lookup (uint32_t word, uint8_t bc_char, uint8_t *p_move)
{
    bool    b_found = false;
    uint8_t move;

    if ((NULL != p_move) && vc_map_is_valid(word)
        && (bc_char <= VC_CHAR_MAX))
    {
        for (move = 0U; move < VC_MOVE_COUNT; move++)
        {
            if (vc_map_char(word, move) == bc_char)
            {
                *p_move = move;
                b_found = true;
            }
            else
            {
                // not this movement
            }
        }
    }
    else
    {
        // nothing to look up
    }

    return (b_found);
}

void
vc_mission_init (vc_mission_t *p_state)
{
    if (NULL != p_state)
    {
        p_state->map_word = VC_MAP_NONE;
        p_state->pending  = VC_CHAR_NONE;
    }
    else
    {
        // nothing to initialise
    }
}

/*!
 * Replace the whole mapping in one store. An invalid word is rejected
 * and the previous mapping stays in force.
 */
bool
vc_mission_set_mapping (vc_mission_t *p_state, uint32_t word)
{
    bool b_ok = false;

    if ((NULL != p_state) && vc_map_is_valid(word))
    {
        p_state->map_word = word;
        b_ok = true;
    }
    else
    {
        // rejected: keep the old mapping
    }

    return (b_ok);
}

/*!
 * A barcode was decoded: store it (overwrites an unused one).
 * Values above 'Z' are ignored.
 */
void
vc_mission_on_char (vc_mission_t *p_state, uint8_t bc_char)
{
    if ((NULL != p_state) && (bc_char <= VC_CHAR_MAX))
    {
        p_state->pending = bc_char;
    }
    else
    {
        // not a character: keep the slot as it is
    }
}

/*!
 * A cross-junction was reached: return the movement to execute and set
 * *p_flags (VC_FLAG_*). The pending character is used up.
 */
uint8_t
vc_mission_on_junction (vc_mission_t *p_state, uint8_t *p_flags)
{
    uint8_t move  = VC_MOVE_FORWARD;
    uint8_t flags = 0U;

    if (NULL == p_state)
    {
        flags = VC_FLAG_NO_PENDING;
    }
    else if (VC_CHAR_NONE == p_state->pending)
    {
        flags = VC_FLAG_NO_PENDING;
    }
    else if (!vc_map_is_valid(p_state->map_word))
    {
        flags = VC_FLAG_NO_MAPPING;
    }
    else if (!vc_map_lookup(p_state->map_word, p_state->pending, &move))
    {
        move  = VC_MOVE_FORWARD;
        flags = VC_FLAG_UNMAPPED;
    }
    else
    {
        // mapped: move holds the movement
    }

    if (NULL != p_state)
    {
        p_state->pending = VC_CHAR_NONE;
    }
    else
    {
        // nothing to clear
    }

    if (NULL != p_flags)
    {
        *p_flags = flags;
    }
    else
    {
        // caller does not want the flags
    }

    return (move);
}

char const *
vc_move_name (uint8_t move)
{
    static char const * const names[VC_MOVE_COUNT] =
    {
        "FORWARD", "U-TURN", "LEFT", "RIGHT"
    };
    char const *p_name = "?";

    if (move < VC_MOVE_COUNT)
    {
        p_name = names[move];
    }
    else
    {
        // out of range
    }

    return (p_name);
}

/*!
 * Four characters, each 0..25, no two the same.
 */
static bool
vc_chars_ok (uint8_t const p_chars[VC_MOVE_COUNT])
{
    bool    b_ok = true;
    uint8_t idx;
    uint8_t other;

    for (idx = 0U; idx < VC_MOVE_COUNT; idx++)
    {
        if (p_chars[idx] > VC_CHAR_MAX)
        {
            b_ok = false;
        }
        else
        {
            // in range
        }

        for (other = idx + 1U; other < VC_MOVE_COUNT; other++)
        {
            if (p_chars[idx] == p_chars[other])
            {
                b_ok = false;
            }
            else
            {
                // distinct
            }
        }
    }

    return (b_ok);
}
