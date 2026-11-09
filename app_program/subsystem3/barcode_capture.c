/*
 * barcode_capture.c
 * Subsystem 3 (Buddy 3): edge -> element -> symbol, see
 * barcode_capture.h.
 */

#include <stddef.h>
#include "barcode_capture.h"

// state values
#define BC_STATE_IDLE           (0U)    // armed: next black after quiet
#define BC_STATE_RUN            (1U)    // collecting elements
#define BC_STATE_WAIT_QUIET     (2U)    // done or aborted: wait for quiet

// The run limit applies once this many elements are stored (a bar and
// a space), see bc_run_limit_us()
#define BC_CAP_LIMIT_FROM       (2U)

static uint32_t bc_elapsed_us(uint32_t from_us, uint32_t to_us);
static uint32_t bc_quiet_limit_us(bc_capture_t const *p_cap);
static uint32_t bc_run_limit_us(bc_capture_t const *p_cap);
static void     bc_end_run(bc_capture_t *p_cap, uint8_t next_state);
static void     bc_store(bc_capture_t *p_cap, uint32_t width_us);
static void     bc_commit(bc_capture_t *p_cap, bool b_black,
                          uint32_t time_us);
static void     bc_settle(bc_capture_t *p_cap, uint32_t now_us);

void
bc_capture_init (bc_capture_t *p_cap, bool b_black_now, uint32_t now_us)
{
    uint8_t idx;

    if (NULL != p_cap)
    {
        for (idx = 0U; idx < BC39_SYMBOL_ELEMENTS; idx++)
        {
            p_cap->width[idx] = 0U;
        }

        p_cap->level_start_us = now_us & BC_CAP_TIME_MASK;
        p_cap->raw_last_us    = p_cap->level_start_us;
        p_cap->burst_start_us = p_cap->level_start_us;
        p_cap->widest_us      = 0U;
        p_cap->decoded_char   = '\0';
        p_cap->count          = 0U;
        p_cap->last_count     = 0U;
        p_cap->b_black        = b_black_now ? 1U : 0U;
        p_cap->b_raw_black    = p_cap->b_black;
        p_cap->pending        = BC_CAP_NONE;
        p_cap->last_status    = BC39_OK;

        // Starting on black: we may be in the middle of a symbol, so
        // wait for a quiet white before arming.
        p_cap->state = b_black_now ? BC_STATE_WAIT_QUIET : BC_STATE_IDLE;
    }
    else
    {
        // nothing to initialise
    }
}

void
bc_capture_edge (bc_capture_t *p_cap, bool b_black, uint32_t time_us)
{
    uint32_t now_us;

    if (NULL == p_cap)
    {
        // nothing to update
    }
    else if ((b_black ? 1U : 0U) == p_cap->b_raw_black)
    {
        // not a change (ring overflow can cause this): ignore
    }
    else
    {
        now_us = time_us & BC_CAP_TIME_MASK;

        // If the raw level held long enough before this edge, its burst
        // is over: commit it (if it differs from the settled level) and
        // start a new burst with this edge.
        bc_settle(p_cap, now_us);

        if (bc_elapsed_us(p_cap->raw_last_us, now_us) >= BC_CAP_SETTLE_US)
        {
            p_cap->burst_start_us = now_us;
        }
        else
        {
            // still inside the same burst: keep its first edge time
        }

        p_cap->b_raw_black = b_black ? 1U : 0U;
        p_cap->raw_last_us = now_us;
    }
}

/*!
 * Commit the raw level as the settled level once it has held for
 * BC_CAP_SETTLE_US, dated at the first edge of its burst.
 */
static void
bc_settle (bc_capture_t *p_cap, uint32_t now_us)
{
    if ((p_cap->b_raw_black != p_cap->b_black)
        && (bc_elapsed_us(p_cap->raw_last_us, now_us) >= BC_CAP_SETTLE_US))
    {
        bc_commit(p_cap, (0U != p_cap->b_raw_black),
                  p_cap->burst_start_us);
    }
    else
    {
        // nothing new, still chattering, or a spike that came back
    }
}

/*!
 * One settled level change at time_us: element and symbol logic.
 */
static void
bc_commit (bc_capture_t *p_cap, bool b_black, uint32_t time_us)
{
    uint32_t width_us    = bc_elapsed_us(p_cap->level_start_us, time_us);
    bool     b_was_black = (0U != p_cap->b_black);

    if (BC_STATE_RUN == p_cap->state)
    {
        if (width_us > bc_run_limit_us(p_cap))
        {
            // Element far too long for a barcode: run is over.
            bc_end_run(p_cap, BC_STATE_IDLE);

            if (!b_was_black)
            {
                // a long white is also a valid quiet zone, so this
                // black may start the next symbol
                p_cap->state     = BC_STATE_RUN;
                p_cap->count     = 0U;
                p_cap->widest_us = 0U;
            }
            else
            {
                p_cap->state = BC_STATE_WAIT_QUIET;
            }
        }
        else
        {
            bc_store(p_cap, width_us);
        }
    }
    else if (!b_was_black && b_black
             && (width_us >= bc_quiet_limit_us(p_cap)))
    {
        // Quiet white, then black: a new symbol starts
        p_cap->state     = BC_STATE_RUN;
        p_cap->count     = 0U;
        p_cap->widest_us = 0U;
    }
    else
    {
        // idle / waiting and not a valid start: ignore
    }

    p_cap->level_start_us = time_us;
    p_cap->b_black        = b_black ? 1U : 0U;
}

bc_cap_result_t
bc_capture_poll (bc_capture_t *p_cap, uint32_t now_us, char *p_char)
{
    bc_cap_result_t result = BC_CAP_NONE;
    uint32_t        held_us;

    if ((NULL != p_cap) && (NULL != p_char))
    {
        // A burst that has gone quiet is committed first
        bc_settle(p_cap, now_us & BC_CAP_TIME_MASK);

        held_us = bc_elapsed_us(p_cap->level_start_us,
                                now_us & BC_CAP_TIME_MASK);

        if (BC_STATE_RUN == p_cap->state)
        {
            if (held_us > bc_run_limit_us(p_cap))
            {
                // No change for too long inside a run: it has ended
                // without 29 elements.
                bc_end_run(p_cap, (0U != p_cap->b_black)
                                  ? BC_STATE_WAIT_QUIET : BC_STATE_IDLE);
            }
            else
            {
                // element still within the limit
            }
        }
        else if ((BC_STATE_WAIT_QUIET == p_cap->state)
                 && (0U == p_cap->b_black)
                 && (held_us > bc_quiet_limit_us(p_cap)))
        {
            p_cap->state = BC_STATE_IDLE;
        }
        else
        {
            // idle, waiting on black, or level still young
        }

        // Cast: pending only ever holds a bc_cap_result_t value (0..3)
        result = (bc_cap_result_t)p_cap->pending;

        if (BC_CAP_DECODED == result)
        {
            *p_char = p_cap->decoded_char;
        }
        else
        {
            // nothing decoded: *p_char untouched
        }

        p_cap->pending = BC_CAP_NONE;
    }
    else
    {
        // NULL argument: report nothing
    }

    return result;
}

void
bc_capture_abort (bc_capture_t *p_cap, uint32_t now_us)
{
    if (NULL != p_cap)
    {
        // Drop any burst in progress: take the raw level as settled
        p_cap->state          = BC_STATE_WAIT_QUIET;
        p_cap->count          = 0U;
        p_cap->level_start_us = now_us & BC_CAP_TIME_MASK;
        p_cap->raw_last_us    = p_cap->level_start_us;
        p_cap->burst_start_us = p_cap->level_start_us;
        p_cap->b_black        = p_cap->b_raw_black;
        p_cap->widest_us      = 0U;     // the dropped run is not trusted
    }
    else
    {
        // nothing to abort
    }
}

bool
bc_capture_busy (bc_capture_t const *p_cap)
{
    bool b_busy = false;

    if (NULL != p_cap)
    {
        b_busy = (BC_STATE_RUN == p_cap->state) && (0U != p_cap->count);
    }
    else
    {
        // no capture: nothing in progress
    }

    return b_busy;
}

/*!
 * Time between two 31-bit timestamps. The mask makes the subtraction
 * correct across the 31-bit wrap (every 35 minutes).
 */
static uint32_t
bc_elapsed_us (uint32_t from_us, uint32_t to_us)
{
    return (to_us - from_us) & BC_CAP_TIME_MASK;
}

static uint32_t
bc_quiet_limit_us (bc_capture_t const *p_cap)
{
    uint32_t limit_us = BC_CAP_QUIET_MIN_US;

    // widest_us <= 524 ms (it comes from a stored width), so x5 fits
    if ((p_cap->widest_us * BC_CAP_QUIET_FACTOR) > limit_us)
    {
        limit_us = p_cap->widest_us * BC_CAP_QUIET_FACTOR;
    }
    else
    {
        // keep the minimum
    }

    return limit_us;
}

/*!
 * Longest element allowed inside a run. The first two elements may
 * last up to the longest storable width; from the third on, the limit
 * is BC_CAP_QUIET_FACTOR x the widest element so far.
 *
 * Why two, not one: a single narrow bar is no reference. A forward
 * '*' starts narrow bar, WIDE space, and the sensor reads bars thin
 * and spaces wide (car, 2026-11-09: 30 ms bar, 174 ms space = 5.8x),
 * so judging the space by the first bar alone ended the run, and the
 * read restarted at the second bar with 27 elements (S3-P31). By the
 * third element both a bar and a space have been seen.
 */
static uint32_t
bc_run_limit_us (bc_capture_t const *p_cap)
{
    uint32_t limit_us = (uint32_t)BC_CAP_WIDTH_MAX << BC_CAP_UNIT_SHIFT;

    if (p_cap->count >= BC_CAP_LIMIT_FROM)
    {
        limit_us = bc_quiet_limit_us(p_cap);
    }
    else
    {
        // first or second element of the run: no reference yet
    }

    return limit_us;
}

/*!
 * A run ended before its 29th element: report it if it was long enough
 * to have been a barcode, then move to next_state.
 */
static void
bc_end_run (bc_capture_t *p_cap, uint8_t next_state)
{
    p_cap->last_count = p_cap->count;

    if (p_cap->count >= BC_CAP_MIN_REPORT)
    {
        p_cap->pending     = BC_CAP_FAILED;
        p_cap->last_status = BC39_ERR_LENGTH;
    }
    else if (p_cap->count >= BC_CAP_SHORT_MIN)
    {
        p_cap->pending = BC_CAP_SHORT;
    }
    else
    {
        // too short to be a barcode: ignore silently
    }

    // The run did not decode, so its widths cannot be trusted: forget
    // its widest element, or one stalled/merged element (seen on the car
    // 2026-11-01) makes the quiet limit so long that the next barcode is
    // skipped. The quiet limit falls back to BC_CAP_QUIET_MIN_US.
    p_cap->widest_us = 0U;
    p_cap->count     = 0U;
    // Mask: state is a 2-bit field and next_state is 0..2
    p_cap->state = (unsigned)next_state & 0x3U;
}

/*!
 * Store one element. The 29th one completes the symbol: decode it.
 */
static void
bc_store (bc_capture_t *p_cap, uint32_t width_us)
{
    uint32_t      units = width_us >> BC_CAP_UNIT_SHIFT;
    bc39_status_t status;
    bool          b_reversed = false;
    char          data_char  = '\0';

    if (units > BC_CAP_WIDTH_MAX)
    {
        units = BC_CAP_WIDTH_MAX;
    }
    else
    {
        // fits 16 bits
    }

    if (width_us > p_cap->widest_us)
    {
        // clamp to the storable width so quiet_limit cannot overflow
        p_cap->widest_us = (uint32_t)units << BC_CAP_UNIT_SHIFT;
    }
    else
    {
        // not the widest
    }

    // Cast: units was clamped to 0xFFFF just above
    p_cap->width[p_cap->count] = (uint16_t)units;
    p_cap->count++;

    if (BC39_SYMBOL_ELEMENTS == p_cap->count)
    {
        status = bc39_decode(p_cap->width, p_cap->count, &data_char,
                             &b_reversed);
        // Cast and mask: bc39_status_t values are 0..5, which fit the
        // 3-bit field
        p_cap->last_status = (unsigned)status & 0x7U;

        if (BC39_OK == status)
        {
            p_cap->decoded_char = data_char;
            p_cap->pending      = BC_CAP_DECODED;
        }
        else
        {
            p_cap->pending   = BC_CAP_FAILED;
            p_cap->widest_us = 0U;      // untrusted widths, see bc_end_run
        }

        p_cap->last_count  = p_cap->count;
        p_cap->count       = 0U;
        p_cap->state       = BC_STATE_WAIT_QUIET;
    }
    else
    {
        // symbol not complete yet
    }
}
