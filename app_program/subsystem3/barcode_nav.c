/*
 * barcode_nav.c
 * Subsystem 3 (Buddy 3): stored barcode letter, letter -> movement map
 * and the movement at a cross-junction, see barcode_nav.h.
 */

#include <stddef.h>
#include "barcode_nav.h"
#include "line_if.h"

#define NAV_SLOT_BITS           (5U)
#define NAV_SLOT_MASK           (0x1FU)
#define NAV_LETTER_COUNT        (26U)   // 'A'..'Z', codes 1..26

// state values
#define NAV_STATE_FOLLOW        (0U)    // line following, watch for a cross
#define NAV_STATE_PREFWD        (1U)    // straight on, over the cross
#define NAV_STATE_SPIN          (2U)    // turning on the spot
#define NAV_STATE_HOLDOFF       (3U)    // following, ignore this cross

_Static_assert(NAV_LETTER_COUNT <= NAV_SLOT_MASK, "letter code width");
_Static_assert(NAV_MOVEMENTS == (LINE_CMD_UTURN + 1U), "one slot per cmd");
_Static_assert(NAV_PREFWD_TICKS <= 511U, "timer width");
_Static_assert(NAV_SPIN_MIN_TICKS < NAV_SPIN_MAX_TICKS, "spin times");
_Static_assert(NAV_SPIN_MAX_TICKS <= 511U, "timer width");
_Static_assert(NAV_HOLDOFF_TICKS <= 511U, "timer width");
_Static_assert(NAV_HOLDOFF_TICKS <= 127U, "watch timer width");

static uint8_t nav_code_of(char letter);
static char    nav_letter_of(uint8_t code);
static void    nav_start_movement(nav_t *p_ctx, uint32_t map,
                                  nav_out_t *p_out);
static nav_event_t nav_step_spin(nav_t *p_ctx, bool b_centre_black,
                                 nav_out_t *p_out);

bool
nav_map_with (uint32_t map, uint8_t movement, char letter,
              uint32_t *p_new_map)
{
    bool     b_ok = false;
    uint8_t  code = nav_code_of(letter);
    uint8_t  slot;
    uint32_t work = map;

    if ((NULL != p_new_map) && (0U != code) && (movement < NAV_MOVEMENTS))
    {
        for (slot = 0U; slot < NAV_MOVEMENTS; slot++)
        {
            // Clear this letter, and whatever the target slot held
            if ((movement == slot)
                || (code == ((work >> (slot * NAV_SLOT_BITS))
                             & NAV_SLOT_MASK)))
            {
                work &= ~((uint32_t)NAV_SLOT_MASK << (slot * NAV_SLOT_BITS));
            }
            else
            {
                // another letter in another slot: keep it
            }
        }

        *p_new_map = work | ((uint32_t)code << (movement * NAV_SLOT_BITS));
        b_ok       = true;
    }
    else
    {
        // bad letter, bad movement or nowhere to write: no change
    }

    return b_ok;
}

bool
nav_map_lookup (uint32_t map, char letter, uint8_t *p_movement)
{
    bool    b_found = false;
    uint8_t code    = nav_code_of(letter);
    uint8_t slot;

    if ((NULL != p_movement) && (0U != code))
    {
        for (slot = 0U; slot < NAV_MOVEMENTS; slot++)
        {
            if (code == ((map >> (slot * NAV_SLOT_BITS)) & NAV_SLOT_MASK))
            {
                *p_movement = slot;
                b_found     = true;
            }
            else
            {
                // not this movement
            }
        }
    }
    else
    {
        // not a letter, so it cannot be mapped
    }

    return b_found;
}

void
nav_init (nav_t *p_ctx)
{
    if (NULL != p_ctx)
    {
        p_ctx->timer          = 0U;
        p_ctx->state          = NAV_STATE_FOLLOW;
        p_ctx->stored         = 0U;
        p_ctx->lines_left     = 0U;
        p_ctx->b_spin_right   = 0U;
        p_ctx->b_was_junction = 0U;
        p_ctx->b_was_centre   = 0U;
    }
    else
    {
        // nothing to initialise
    }
}

bool
nav_store (nav_t *p_ctx, char letter)
{
    bool    b_ok = false;
    uint8_t code = nav_code_of(letter);

    if ((NULL != p_ctx) && (0U != code))
    {
        p_ctx->stored = code & NAV_SLOT_MASK;   // 5-bit field
        b_ok          = true;
    }
    else
    {
        // not A..Z: keep what is stored
    }

    return b_ok;
}

nav_event_t
nav_step (nav_t *p_ctx, uint32_t map, uint8_t mode, bool b_centre_black,
          bool b_active, nav_out_t *p_out)
{
    nav_event_t event = NAV_EV_NONE;
    bool        b_junction = (LINE_MODE_JUNCTION == mode);

    if ((NULL != p_ctx) && (NULL != p_out))
    {
        p_out->b_override = false;
        p_out->left       = 0;
        p_out->right      = 0;
        p_out->movement   = LINE_CMD_STRAIGHT;
        p_out->letter     = '\0';
        p_out->b_mapped   = false;

        if (!b_active)
        {
            p_ctx->state = NAV_STATE_FOLLOW;
            p_ctx->timer = 0U;
        }
        else if (NAV_STATE_FOLLOW == p_ctx->state)
        {
            // Only the moment a junction appears counts
            if (b_junction && (0U == p_ctx->b_was_junction))
            {
                nav_start_movement(p_ctx, map, p_out);
                event = NAV_EV_JUNCTION;
            }
            else
            {
                // no new junction
            }
        }
        else if (NAV_STATE_PREFWD == p_ctx->state)
        {
            p_out->b_override = true;
            p_out->left       = NAV_PREFWD_DUTY;
            p_out->right      = NAV_PREFWD_DUTY;
            p_ctx->timer++;

            if (p_ctx->timer >= NAV_PREFWD_TICKS)
            {
                p_ctx->state = NAV_STATE_SPIN;
                p_ctx->timer = 0U;
            }
            else
            {
                // still driving onto the cross
            }
        }
        else if (NAV_STATE_SPIN == p_ctx->state)
        {
            event = nav_step_spin(p_ctx, b_centre_black, p_out);
        }
        else
        {
            // HOLDOFF: normal steering, but this cross must not count
            // again. Leaves only once the junction reading has gone.
            if (p_ctx->timer < NAV_HOLDOFF_TICKS)
            {
                p_ctx->timer++;
            }
            else if (!b_junction)
            {
                p_ctx->state = NAV_STATE_FOLLOW;
                p_ctx->timer = 0U;
            }
            else
            {
                // time is up but still on black: wait
            }
        }

        p_ctx->b_was_junction = b_junction ? 1U : 0U;
        p_ctx->b_was_centre   = b_centre_black ? 1U : 0U;
    }
    else
    {
        // NULL argument: nothing to do
    }

    return event;
}

void
nav_watch_init (nav_watch_t *p_watch)
{
    if (NULL != p_watch)
    {
        p_watch->timer          = 0U;
        p_watch->b_was_junction = 0U;
    }
    else
    {
        // nothing to initialise
    }
}

bool
nav_watch_step (nav_watch_t *p_watch, uint8_t mode)
{
    bool b_new      = false;
    bool b_junction = (LINE_MODE_JUNCTION == mode);

    if (NULL != p_watch)
    {
        if (0U != p_watch->timer)
        {
            p_watch->timer--;
        }
        else if (b_junction && (0U == p_watch->b_was_junction))
        {
            b_new          = true;
            p_watch->timer = NAV_HOLDOFF_TICKS;
        }
        else
        {
            // no new crossing
        }

        p_watch->b_was_junction = b_junction ? 1U : 0U;
    }
    else
    {
        // no tracker: report nothing
    }

    return b_new;
}

/*!
 * A junction was reached: use up the stored letter and choose what to
 * do. No letter, or a letter with no mapping, means forward.
 */
static void
nav_start_movement (nav_t *p_ctx, uint32_t map, nav_out_t *p_out)
{
    uint8_t movement = LINE_CMD_STRAIGHT;

    // Cast: stored is a 5-bit field, 0..31
    p_out->letter   = nav_letter_of((uint8_t)p_ctx->stored);
    p_out->b_mapped = nav_map_lookup(map, p_out->letter, &movement);
    p_out->movement = movement;
    p_ctx->stored   = 0U;
    p_ctx->timer    = 0U;

    if (LINE_CMD_STRAIGHT == movement)
    {
        p_ctx->state = NAV_STATE_HOLDOFF;
    }
    else
    {
        p_ctx->state        = NAV_STATE_PREFWD;
        p_ctx->lines_left   = (LINE_CMD_UTURN == movement) ? 2U : 1U;
        p_ctx->b_spin_right = (LINE_CMD_RIGHT == movement) ? 1U : 0U;
    }
}

/*!
 * Spin on the spot. For NAV_SPIN_MIN_TICKS the sensors are ignored, so
 * the line the car started on is not taken for the next one. After
 * that every white -> black change of the centre sensor is one line.
 */
static nav_event_t
nav_step_spin (nav_t *p_ctx, bool b_centre_black, nav_out_t *p_out)
{
    nav_event_t event = NAV_EV_NONE;

    p_out->b_override = true;
    p_out->left  = (0U != p_ctx->b_spin_right) ? NAV_SPIN_DUTY
                                               : -NAV_SPIN_DUTY;
    p_out->right = (0U != p_ctx->b_spin_right) ? -NAV_SPIN_DUTY
                                               : NAV_SPIN_DUTY;
    p_ctx->timer++;

    if ((p_ctx->timer >= NAV_SPIN_MIN_TICKS) && b_centre_black
        && (0U == p_ctx->b_was_centre))
    {
        p_ctx->lines_left--;

        if (0U == p_ctx->lines_left)
        {
            p_ctx->state = NAV_STATE_HOLDOFF;
            p_ctx->timer = 0U;
            event        = NAV_EV_TURN_DONE;
        }
        else
        {
            // U-turn: that was the side branch, keep spinning
        }
    }
    else if (p_ctx->timer >= NAV_SPIN_MAX_TICKS)
    {
        // No line found: hand back to the line logic, whose search and
        // FAILED stop then take over
        p_ctx->state = NAV_STATE_HOLDOFF;
        p_ctx->timer = 0U;
        event        = NAV_EV_TURN_TIMEOUT;
    }
    else
    {
        // keep spinning
    }

    return event;
}

/*!
 * 'A'..'Z' -> 1..26, anything else -> 0.
 */
static uint8_t
nav_code_of (char letter)
{
    uint8_t code = 0U;

    if ((letter >= 'A') && (letter <= 'Z'))
    {
        // Cast: the range check above keeps the result in 1..26
        code = (uint8_t)((letter - 'A') + 1);
    }
    else
    {
        // not a letter
    }

    return code;
}

/*!
 * 1..26 -> 'A'..'Z', anything else -> '\0'.
 */
static char
nav_letter_of (uint8_t code)
{
    char letter = '\0';

    if ((code >= 1U) && (code <= NAV_LETTER_COUNT))
    {
        // Cast: the range check above keeps the result in 'A'..'Z'
        letter = (char)('A' + (code - 1U));
    }
    else
    {
        // no letter stored
    }

    return letter;
}
