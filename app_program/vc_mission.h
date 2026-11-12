/*
 * vc_mission.h
 * Vehicle Controller mission logic (shared file, written by Buddy 4).
 * CLAUDE.md section 1b: barcode -> stored character -> movement at the
 * next cross-junction, using a mapping that arrives over MQTT at run
 * time. Nothing here is compiled in as a default mapping.
 *
 * Pure functions, no kernel calls: the same file builds on the PC
 * (testing/vehicle_controller/pc/). Single owner: only the Vehicle
 * Controller task calls these functions, so the state needs no lock.
 *
 * Packed data (Change Requirement 1), explicit masks and shifts, never C
 * bit-fields:
 *
 *   character  5 bits  0..25 = 'A'..'Z', VC_CHAR_NONE (31) = empty
 *   movement   2 bits  VC_MOVE_FORWARD / UTURN / LEFT / RIGHT
 *
 *   MAPPING WORD (uint32_t, 21 of 32 bits used, PROPOSED in CLAUDE.md 6):
 *     bit  4..0   character that means FORWARD
 *     bit  9..5   character that means U-TURN
 *     bit 14..10  character that means LEFT
 *     bit 19..15  character that means RIGHT
 *     bit 20      valid: 1 = complete, checked mapping
 *     bit 31..21  0
 *   A valid word has four DISTINCT characters, each 0..25. It is
 *   replaced as one whole word (never field by field), so a half-received
 *   mapping can never be applied.
 *
 * Junction rules (CLAUDE.md 1b PROPOSED defaults, team to CONFIRM):
 *   - one pending character; a new barcode before the junction
 *     overwrites it; it is cleared once the junction is handled
 *   - no pending character      -> FORWARD, flag VC_FLAG_NO_PENDING
 *   - no mapping received yet   -> FORWARD, flag VC_FLAG_NO_MAPPING
 *   - character not in mapping  -> FORWARD, flag VC_FLAG_UNMAPPED
 *   The car never stops because of a missing mapping (NFR12).
 */

#ifndef VC_MISSION_H
#define VC_MISSION_H

#include <stdbool.h>
#include <stdint.h>

// Character field
#define VC_CHAR_BITS            (5U)
#define VC_CHAR_MASK            (0x1FU)
#define VC_CHAR_MAX             (25U)       // 'Z'
#define VC_CHAR_NONE            (31U)       // empty slot

// Movement field (2 bits)
#define VC_MOVE_FORWARD         (0U)
#define VC_MOVE_UTURN           (1U)
#define VC_MOVE_LEFT            (2U)
#define VC_MOVE_RIGHT           (3U)
#define VC_MOVE_COUNT           (4U)
#define VC_MOVE_MASK            (0x03U)

// Mapping word
#define VC_MAP_VALID_SHIFT      (20U)
#define VC_MAP_VALID_BIT        ((uint32_t)1U << VC_MAP_VALID_SHIFT)
#define VC_MAP_USED_MASK        (0x001FFFFFU)   // bits 20..0
#define VC_MAP_NONE             (0U)            // no mapping (not valid)

// Junction result flags (one bit each, for telemetry)
#define VC_FLAG_NO_PENDING      (0x01U)
#define VC_FLAG_NO_MAPPING      (0x02U)
#define VC_FLAG_UNMAPPED        (0x04U)

// Owned by the VC task only; never shared between tasks.
typedef struct
{
    uint32_t map_word;      // VC_MAP_NONE until MQTT supplies one
    uint8_t  pending;       // 0..25, or VC_CHAR_NONE
} vc_mission_t;

// Mapping word
bool     vc_map_pack(uint8_t const p_chars[VC_MOVE_COUNT],
                     uint32_t *p_word);
bool     vc_map_is_valid(uint32_t word);
uint8_t  vc_map_char(uint32_t word, uint8_t move);
bool     vc_map_lookup(uint32_t word, uint8_t bc_char, uint8_t *p_move);

// Mission state
void     vc_mission_init(vc_mission_t *p_state);
bool     vc_mission_set_mapping(vc_mission_t *p_state, uint32_t word);
void     vc_mission_on_char(vc_mission_t *p_state, uint8_t bc_char);
uint8_t  vc_mission_on_junction(vc_mission_t *p_state, uint8_t *p_flags);

// Text for log lines ("FORWARD", ...); "?" for an invalid value
char const *vc_move_name(uint8_t move);

#endif /* VC_MISSION_H */
