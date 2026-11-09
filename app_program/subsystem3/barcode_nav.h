/*
 * barcode_nav.h
 * Subsystem 3 (Buddy 3): remembers the last decoded barcode letter and
 * performs its movement at the next cross-junction.
 *
 * TEMPORARY stand-in, switched by S3_NAV_SWITCH in subsystem3.h. The
 * team plan (INTEGRATION.md) gives this job to the Vehicle Controller,
 * which is not written yet: S3 publishes the letter in line_state either
 * way, so this file can go once the VC does the mapping and the turns.
 *
 * Demo rules (owner, 2026-11-02):
 *   - a barcode's letter (A..Z) is decoded and STORED;
 *   - the car keeps following the line until a cross-junction;
 *   - there it performs the movement mapped to the stored letter;
 *   - the letter -> movement mapping comes from the PC over MQTT just
 *     before the demo, so it is data, never a fixed table.
 *
 * Mapping word (Change Requirement 1): shared between tasks, so a plain
 * 32-bit word with masks and shifts, published with one atomic store.
 * One 5-bit slot per movement, indexed by LINE_CMD_*:
 *   bit 19..15  LINE_CMD_UTURN     0 = no letter, 1..26 = 'A'..'Z'
 *   bit 14..10  LINE_CMD_STRAIGHT
 *   bit  9..5   LINE_CMD_RIGHT
 *   bit  4..0   LINE_CMD_LEFT
 *
 * A movement at a junction, using only the centre sensor (no encoders):
 *   forward     keep following the line straight through
 *   left/right  drive a little further, then spin until the centre
 *               sensor reaches the next line (the side branch)
 *   U-turn      same, but stop on the SECOND line: at a cross the first
 *               line reached is the side branch, the second the way back
 * A letter with no mapping, or no stored letter, means forward.
 *
 * Pure C, no kernel calls: also built by the PC tests.
 */

#ifndef BARCODE_NAV_H
#define BARCODE_NAV_H

#include <stdbool.h>
#include <stdint.h>

#define NAV_MAP_EMPTY           (0U)        // no letter mapped at all
#define NAV_MOVEMENTS           (4U)        // LINE_CMD_* values 0..3

// All of these are 10 ms cycles and need tuning on the track.
#define NAV_PREFWD_TICKS        (40U)   // 0.4 s: over the cross  TODO_TUNE
#define NAV_SPIN_MIN_TICKS      (40U)   // 0.4 s: leave the line  TODO_TUNE
#define NAV_SPIN_MAX_TICKS      (500U)  // 5 s: give up the turn  TODO_TUNE
#define NAV_HOLDOFF_TICKS       (100U)  // 1 s: same cross again  TODO_TUNE

// Wheel duties, same units as motion_set_duty()
#define NAV_PREFWD_DUTY         (170)                         // TODO_TUNE
#define NAV_SPIN_DUTY           (180)                         // TODO_TUNE

typedef enum
{
    NAV_EV_NONE = 0,        // nothing to report this cycle
    NAV_EV_JUNCTION,        // reached a cross: see letter and movement
    NAV_EV_TURN_DONE,       // spin ended on the wanted line
    NAV_EV_TURN_TIMEOUT     // spin found no line: back to line following
} nav_event_t;

/*
 * Task-private (Line task only): bit-fields allowed (Change Requirement
 * 1). 500 ticks need 9 bits; a letter code needs 5.
 */
typedef struct
{
    unsigned timer         : 9;     // cycles in the current state
    unsigned state         : 2;     // follow / forward / spin / hold off
    unsigned stored        : 5;     // stored letter, 0 = none
    unsigned lines_left    : 2;     // lines still to reach in this spin
    unsigned b_spin_right  : 1;     // 1 = clockwise spin
    unsigned b_was_junction: 1;     // junction seen last cycle
    unsigned b_was_centre  : 1;     // centre black last cycle
} nav_t;

/*
 * Task-private: reports each cross-junction once when nav_t is switched
 * off. 100 ticks need 7 bits.
 */
typedef struct
{
    unsigned timer         : 7;     // cycles left of the hold-off
    unsigned b_was_junction: 1;     // junction seen last cycle
} nav_watch_t;

typedef struct
{
    bool    b_override;     // true: use left/right, not line steering
    int16_t left;           // wheel duties when b_override
    int16_t right;
    uint8_t movement;       // NAV_EV_JUNCTION: LINE_CMD_* to perform
    char    letter;         // NAV_EV_JUNCTION: stored letter, '\0' = none
    bool    b_mapped;       // NAV_EV_JUNCTION: the letter had a mapping
} nav_out_t;

/*!
 * Put letter (A..Z) in slot movement (LINE_CMD_*) of map and write the
 * result to *p_new_map. A letter may only drive one movement, so it is
 * removed from any other slot first. Returns false, leaving *p_new_map
 * untouched, for any other letter or movement.
 */
bool nav_map_with(uint32_t map, uint8_t movement, char letter,
                  uint32_t *p_new_map);

/*!
 * Find the movement mapped to letter. Returns false if there is none.
 */
bool nav_map_lookup(uint32_t map, char letter, uint8_t *p_movement);

void nav_init(nav_t *p_ctx);

/*!
 * Remember a decoded letter, replacing any earlier one. Returns false
 * and changes nothing for anything but A..Z.
 */
bool nav_store(nav_t *p_ctx, char letter);

/*!
 * One 10 ms step. mode is this cycle's LINE_MODE_*, b_centre_black the
 * debounced centre sensor. b_active false (car not allowed to move)
 * cancels any movement, reports no junction, and keeps the stored
 * letter. A junction uses up the stored letter.
 */
nav_event_t nav_step(nav_t *p_ctx, uint32_t map, uint8_t mode,
                     bool b_centre_black, bool b_active,
                     nav_out_t *p_out);

void nav_watch_init(nav_watch_t *p_watch);

/*!
 * One 10 ms step. True once per crossing: when mode first becomes
 * LINE_MODE_JUNCTION, not again for NAV_HOLDOFF_TICKS or until the
 * junction reading has gone, whichever is later.
 */
bool nav_watch_step(nav_watch_t *p_watch, uint8_t mode);

#endif /* BARCODE_NAV_H */
