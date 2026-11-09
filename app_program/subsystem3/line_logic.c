/*
 * line_logic.c
 * Subsystem 3 (Buddy 3): line position and line-lost search, see
 * line_logic.h.
 */

#include <stddef.h>
#include "line_logic.h"
#include "line_if.h"

#define LINE_TIMER_MAX          (511U)      // 9-bit timer field

_Static_assert(LINE_LOST_TICKS <= LINE_TIMER_MAX, "timer width");
_Static_assert(LINE_SEARCH1_TICKS <= LINE_TIMER_MAX, "timer width");
_Static_assert(LINE_SEARCH2_TICKS <= LINE_TIMER_MAX, "timer width");
_Static_assert(LINE_DEBOUNCE_TICKS <= 3U, "debounce counter width");
_Static_assert(LINE_JUNCTION_TICKS <= 15U, "junction counter width");
_Static_assert(LINE_ERR_FULL < LINE_IF_ERROR_CENTRE, "error range");
_Static_assert(LINE_ERR_NEAR < LINE_ERR_SIDE, "near is the smaller");
_Static_assert(LINE_ERR_GUESS <= LINE_ERR_SIDE, "a guess is gentler");

static void    line_debounce(bool b_raw, unsigned *p_b_state,
                             unsigned *p_count);
static uint8_t line_toward(uint8_t side, uint8_t offset);
static uint8_t line_other_side(uint8_t side);
static void    line_update_mode(line_logic_t *p_ctx, bool b_contact);
static uint8_t line_error(line_logic_t const *p_ctx);

void
line_logic_init (line_logic_t *p_ctx)
{
    if (NULL != p_ctx)
    {
        p_ctx->timer        = 0U;
        p_ctx->mode         = LINE_MODE_INIT;
        p_ctx->last_side    = LINE_SIDE_NONE;
        p_ctx->b_centre     = 0U;
        p_ctx->b_right      = 0U;
        p_ctx->b_barcode    = 0U;
        p_ctx->centre_count = 0U;
        p_ctx->right_count  = 0U;
        p_ctx->bc_count     = 0U;
        p_ctx->all_count    = 0U;
    }
    else
    {
        // nothing to initialise
    }
}

void
line_logic_step (line_logic_t *p_ctx, bool b_centre_black,
                 bool b_right_black, bool b_barcode_black,
                 uint8_t *p_error, uint8_t *p_mode)
{
    unsigned b_state;
    unsigned count;
    bool     b_centre;
    bool     b_right;
    bool     b_barcode;

    if ((NULL != p_ctx) && (NULL != p_error) && (NULL != p_mode))
    {
        // 1. Debounce each sensor (bit-fields cannot be passed by
        //    address, so copy out and back)
        b_state = p_ctx->b_centre;
        count   = p_ctx->centre_count;
        line_debounce(b_centre_black, &b_state, &count);
        p_ctx->b_centre     = b_state & 0x1U;  // 1-bit field
        p_ctx->centre_count = count & 0x3U;    // 2-bit field

        b_state = p_ctx->b_right;
        count   = p_ctx->right_count;
        line_debounce(b_right_black, &b_state, &count);
        p_ctx->b_right     = b_state & 0x1U;   // 1-bit field
        p_ctx->right_count = count & 0x3U;     // 2-bit field

        b_state = p_ctx->b_barcode;
        count   = p_ctx->bc_count;
        line_debounce(b_barcode_black, &b_state, &count);
        p_ctx->b_barcode = b_state & 0x1U;     // 1-bit field
        p_ctx->bc_count  = count & 0x3U;       // 2-bit field

        b_centre  = (0U != p_ctx->b_centre);
        b_right   = (0U != p_ctx->b_right);
        b_barcode = (0U != p_ctx->b_barcode);

        // 2. Junction confirmation counter: all three black
        if (b_centre && b_right && b_barcode)
        {
            if (p_ctx->all_count < LINE_JUNCTION_TICKS)
            {
                p_ctx->all_count++;
            }
            else
            {
                // confirmed: hold
            }
        }
        else
        {
            p_ctx->all_count = 0U;
        }

        // 3. Where the line goes if the centre sensor loses it: right
        //    if the right sensor sees it, otherwise left (it can only
        //    reach the right sensor by passing it). Both white says
        //    nothing new, so keep the old side.
        if (b_right)
        {
            p_ctx->last_side = LINE_SIDE_RIGHT;
        }
        else if (b_centre)
        {
            p_ctx->last_side = LINE_SIDE_LEFT;
        }
        else
        {
            // both white: unchanged
        }

        // 4. Mode, then the error that goes with it
        line_update_mode(p_ctx, b_centre || b_right);

        *p_error = line_error(p_ctx);
        // Cast: mode is a 3-bit field, 0..7
        *p_mode  = (uint8_t)p_ctx->mode;
    }
    else
    {
        // NULL argument: nothing to do
    }
}

/*!
 * Accept a new raw level only after it has been read
 * LINE_DEBOUNCE_TICKS cycles in a row.
 */
static void
line_debounce (bool b_raw, unsigned *p_b_state, unsigned *p_count)
{
    if ((b_raw ? 1U : 0U) == *p_b_state)
    {
        *p_count = 0U;
    }
    else
    {
        (*p_count)++;

        if (*p_count >= LINE_DEBOUNCE_TICKS)
        {
            *p_b_state = b_raw ? 1U : 0U;
            *p_count   = 0U;
        }
        else
        {
            // not stable yet
        }
    }
}

/*!
 * 64 moved by 'offset' toward 'side'. Left = smaller, right = larger.
 * With no known side the search turns left first (documented default).
 */
static uint8_t
line_toward (uint8_t side, uint8_t offset)
{
    uint8_t error;

    // offset <= 63 (static assert above), so 64 - 63 = 1 and
    // 64 + 63 = 127: always inside the 7-bit field
    if (LINE_SIDE_RIGHT == side)
    {
        error = (uint8_t)(LINE_IF_ERROR_CENTRE + offset);
    }
    else
    {
        error = (uint8_t)(LINE_IF_ERROR_CENTRE - offset);
    }

    return error;
}

static uint8_t
line_other_side (uint8_t side)
{
    uint8_t other;

    if (LINE_SIDE_RIGHT == side)
    {
        other = LINE_SIDE_LEFT;
    }
    else
    {
        // left, or none (the first search went left)
        other = LINE_SIDE_RIGHT;
    }

    return other;
}

static void
line_update_mode (line_logic_t *p_ctx, bool b_contact)
{
    if (p_ctx->all_count >= LINE_JUNCTION_TICKS)
    {
        p_ctx->mode  = LINE_MODE_JUNCTION;
        p_ctx->timer = 0U;
    }
    else if (b_contact)
    {
        // Centre or right sees black: on the line (also ends a search
        // or FAILED)
        p_ctx->mode  = LINE_MODE_TRACKING;
        p_ctx->timer = 0U;
    }
    else if ((LINE_MODE_INIT == p_ctx->mode)
             || (LINE_MODE_JUNCTION == p_ctx->mode))
    {
        // Start of run, or junction just passed, with no contact:
        // track, and let the lost timer decide below.
        p_ctx->mode  = LINE_MODE_TRACKING;
        p_ctx->timer = 0U;
    }
    else if (LINE_MODE_TRACKING == p_ctx->mode)
    {
        p_ctx->timer++;

        if (p_ctx->timer >= LINE_LOST_TICKS)
        {
            p_ctx->mode  = LINE_MODE_SEARCH_NEAR;
            p_ctx->timer = 0U;
        }
        else
        {
            // still steering back toward the last side
        }
    }
    else if (LINE_MODE_SEARCH_NEAR == p_ctx->mode)
    {
        p_ctx->timer++;

        if (p_ctx->timer >= LINE_SEARCH1_TICKS)
        {
            p_ctx->mode  = LINE_MODE_SEARCH_FAR;
            p_ctx->timer = 0U;
        }
        else
        {
            // keep turning toward the last side
        }
    }
    else if (LINE_MODE_SEARCH_FAR == p_ctx->mode)
    {
        p_ctx->timer++;

        if (p_ctx->timer >= LINE_SEARCH2_TICKS)
        {
            p_ctx->mode  = LINE_MODE_FAILED;
            p_ctx->timer = 0U;
        }
        else
        {
            // keep turning the other way
        }
    }
    else
    {
        // FAILED: stay until the centre or right sensor sees black
    }
}

static uint8_t
line_error (line_logic_t const *p_ctx)
{
    uint8_t error    = LINE_IF_ERROR_CENTRE;
    // Cast: last_side is a 2-bit field, 0..3
    uint8_t side     = (uint8_t)p_ctx->last_side;
    bool    b_centre = (0U != p_ctx->b_centre);
    bool    b_right  = (0U != p_ctx->b_right);

    if (LINE_MODE_TRACKING == p_ctx->mode)
    {
        if (b_centre && b_right)
        {
            error = line_toward(LINE_SIDE_RIGHT, LINE_ERR_NEAR);
        }
        else if (b_right)
        {
            error = line_toward(LINE_SIDE_RIGHT, LINE_ERR_SIDE);
        }
        else if (b_centre)
        {
            // on the line: straight ahead
        }
        else if (LINE_SIDE_RIGHT == side)
        {
            // the right sensor saw it last: certain, steer firmly
            error = line_toward(LINE_SIDE_RIGHT, LINE_ERR_SIDE);
        }
        else if (LINE_SIDE_LEFT == side)
        {
            // no left sensor: only a guess, steer gently
            error = line_toward(LINE_SIDE_LEFT, LINE_ERR_GUESS);
        }
        else
        {
            // never seen the line yet: straight ahead
        }
    }
    else if (LINE_MODE_SEARCH_NEAR == p_ctx->mode)
    {
        error = line_toward(side, LINE_ERR_FULL);
    }
    else if (LINE_MODE_SEARCH_FAR == p_ctx->mode)
    {
        error = line_toward(line_other_side(side), LINE_ERR_FULL);
    }
    else
    {
        // INIT, JUNCTION, FAILED: straight / stopped
    }

    return error;
}
