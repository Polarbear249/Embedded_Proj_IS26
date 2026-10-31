/*
 * imu_hump.h
 * Subsystem 4 (Buddy 4): hump detection and height (FR7). Pure code: no
 * hardware, no kernel calls, so it also runs in the PC test
 * (testing/subsystem4/host/).
 *
 * Car model: a rigid body on two ground contacts, the rear axle and the
 * front ball caster, IMU_HUMP_BASE_MM apart. Pitch (nose up > 0) tells
 * how much higher the front contact is than the rear one:
 *     z_front - z_rear = L x sin(pitch)
 * The rear wheel reaches each ground point L mm (of travel) after the
 * front did, so the ground height under the front contact is
 *     z(s) = z(s - L) + L x sin(pitch(s))          s = distance, mm
 * For a hump shorter than L this is simply L x sin(pitch) while the rear
 * is still on the flat; on a long ramp the L-long steps add up. Inputs
 * are pitch and wheel distance only: no double integration of
 * acceleration, which drifts within seconds.
 *
 * Events are measured in distance, not time (CLAUDE.md section 8):
 *  - start: |pitch| >= IMU_HUMP_ON_DDEG for IMU_HUMP_START_MM of travel
 *  - end:   |pitch| <  IMU_HUMP_OFF_DDEG for IMU_HUMP_END_MM of travel
 *           (longer than L, so the rear wheel has come down too)
 * Outside an event the profile is held at 0, so pitch noise and a small
 * level error cannot build up over a run. Tilting the car without
 * moving it never counts. Reverse travel is ignored: the profile is only
 * built while the car moves past the furthest point it has reached.
 *
 * Height = highest z during the event, rounded to mm, clamped to 255
 * (8-bit field, CLAUDE.md section 6), never wrapped. An event that never
 * rises (a down ramp, or the far side of a flat top longer than
 * IMU_HUMP_END_MM) ends without a HUMP_END and leaves the results alone;
 * HUMP_START means "terrain event started", before its direction is
 * known.
 *
 * Checked in the PC test against simulated humps (23 mm, 8 mm, 30 mm
 * long, 40 mm smooth, flat with noise and level error, tilted without
 * moving, clamp): within 1 mm (derived, not measured on the car).
 */

#ifndef IMU_HUMP_H
#define IMU_HUMP_H

#include <stdbool.h>
#include <stdint.h>

// Rear axle to ball contact. Measured: the 23 mm block under the ball
// read pitch 15.7 deg (runs 12 and 35): 23 / sin(15.7 deg) = 85 mm.
#define IMU_HUMP_BASE_MM        (85)
#define IMU_HUMP_ON_DDEG        (30)    // 3.0 deg (noise at rest +-0.2)
#define IMU_HUMP_OFF_DDEG       (15)    // 1.5 deg
#define IMU_HUMP_START_MM       (5)
#define IMU_HUMP_END_MM         (IMU_HUMP_BASE_MM + 20)
#define IMU_HUMP_MAX_MM         (255U)  // 8-bit field: clamp
#define IMU_HUMP_JUMP_MM        (50)    // more per update = glitch: resync

// IMU events, 3 bits (CLAUDE.md section 6)
typedef enum
{
    IMU_EVT_NONE = 0,
    IMU_EVT_HUMP_START,
    IMU_EVT_HUMP_END,
    IMU_EVT_IMPACT,                     // reserved: impact detection
    IMU_EVT_FAULT
} imu_event_t;

typedef struct
{
    int32_t  z_um[IMU_HUMP_BASE_MM];    // profile ring, index s mod L
    int32_t  s_last_mm;                 // furthest distance processed
    int32_t  cand_mm;                   // travel with |pitch| >= ON
    int32_t  quiet_mm;                  // travel with |pitch| < OFF
    int32_t  z_now_um;                  // last profile value
    int32_t  peak_um;                   // highest z in this event
    uint32_t last_mm;                   // last hump height
    uint32_t max_mm;                    // highest hump of the run
    bool     b_synced;                  // s_last_mm valid
    bool     b_on_hump;                 // event in progress
    bool     b_valid;                   // at least one hump measured
} imu_hump_t;

// IF-11 hump word (TEAM.md section 5), one aligned uint32_t
#define IMU_HUMP_W_MAX(w)       ((uint32_t)(w) & 0xFFU)
#define IMU_HUMP_W_LAST(w)      (((uint32_t)(w) >> 8) & 0xFFU)
#define IMU_HUMP_W_STATE(w)     (((uint32_t)(w) >> 16) & 0x0FU)
#define IMU_HUMP_W_EVENT(w)     (((uint32_t)(w) >> 20) & 0x07U)
#define IMU_HUMP_W_VALID(w)     (((uint32_t)(w) >> 23) & 0x01U)
#define IMU_HUMP_W_SEQ(w)       (((uint32_t)(w) >> 24) & 0xFFU)

void        imu_hump_init(imu_hump_t *p_hump);
void        imu_hump_resync(imu_hump_t *p_hump);
imu_event_t imu_hump_update(imu_hump_t *p_hump, int32_t pitch_ddeg,
                            int32_t dist_mm);
int32_t     imu_hump_now_mm(imu_hump_t const *p_hump);
uint32_t    imu_hump_pack(imu_hump_t const *p_hump, uint32_t state,
                          uint32_t event, uint32_t seq);

#endif // IMU_HUMP_H
