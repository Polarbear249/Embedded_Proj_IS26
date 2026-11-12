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
 *  (tilt = the larger of |pitch| and |roll|)
 *  - start: tilt >= IMU_HUMP_ON_DDEG for IMU_HUMP_START_MM of travel
 *           (20 mm: a drive-off kick tilts the gravity vector for a few
 *           mm, run 37 started a false hump at 5 mm)
 *  - end:   tilt <  IMU_HUMP_OFF_DDEG for IMU_HUMP_END_MM of travel
 *           (longer than L, so the rear wheel has come down too), or
 *           the car has not moved forward for IMU_HUMP_STILL_CALLS
 *           updates (stopped on or right after the hump, run 37)
 *  - drop:  an event longer than IMU_HUMP_MAX_LEN_MM is a slope or a
 *           level error, not a hump: cleared without a result
 * Outside an event the profile is held at 0, so pitch noise and a small
 * level error cannot build up over a run. Tilting the car without
 * moving it never counts. Reverse travel is ignored: the profile is only
 * built while the car moves past the furthest point it has reached.
 *
 * Height = highest z (front or side model, below) during the event,
 * rounded to mm, clamped to 255
 * (8-bit field, CLAUDE.md section 6), never wrapped. An event that never
 * rises (a down ramp, or the far side of a flat top longer than
 * IMU_HUMP_END_MM) ends without a HUMP_END and leaves the results alone;
 * HUMP_START means "terrain event started", before its direction is
 * known.
 *
 * Side hump (demo track: hump on the RIGHT, CLAUDE.md 1b). A bump under
 * one rear wheel only, with the other wheel and the ball on the flat,
 * tips the car about the line from the other wheel to the ball:
 *     z_wheel = W x sin(|roll|)        W = IMU_HUMP_TRACK_MM
 * Checked on the car (run 38, 23 mm block under one rear wheel): roll
 * 11.2 / 10.4 deg -> 22.0 / 20.4 mm (-4 % / -11 %). The same lift gave
 * only -3.8 / -2.4 deg of pitch where the rigid 3-point model predicts
 * -7.8, so pitch is NOT used to confirm a side hump (cause unknown:
 * TODO find out why before relying on pitch for one-wheel cases).
 * Both models run during every event: |pitch| or |roll| above the
 * thresholds starts and holds it, and the height is the larger of the
 * front (pitch) and side (roll) peaks. A full-width hump gives roll
 * about 0, a one-side hump gives a front profile about 0 (nose down),
 * so the larger peak is the right model in both cases. |roll| means a
 * left or a right side hump measures the same; a dip under one wheel
 * also reads as a side hump (cannot be told apart from roll alone).
 * Risk (not yet tested on the car): lateral acceleration in a curve
 * tilts the gravity vector sideways, i.e. fake roll; at 0.3 m/s on a
 * 0.3 m radius about 1.7 deg (derived), below the 4 deg start.
 * Smallest side hump that starts: 113 x sin(4 deg) = 7.9 mm (derived).
 *
 * Checked in the PC test against simulated humps (23 mm, 8 mm, 30 mm
 * long, 40 mm smooth, flat with noise and level error, tilted without
 * moving, clamp, left- and right-wheel side ramps): within 1-2 mm
 * (derived, not measured on the car).
 */

#ifndef IMU_HUMP_H
#define IMU_HUMP_H

#include <stdbool.h>
#include <stdint.h>

// Rear axle to ball contact. Measured: the 23 mm block under the ball
// read pitch 15.7 deg (runs 12 and 35): 23 / sin(15.7 deg) = 85 mm.
#define IMU_HUMP_BASE_MM        (85)
// Rear tyre centre to centre (middle of each tread), ruler, 2026-11-03.
// Run 38 roll on the 23 mm block agrees (asin(23/113) = 11.7 deg).
#define IMU_HUMP_TRACK_MM       (113)
// Above the tilt seen while DRIVING on the flat (runs 40-42 test drive,
// 2026-11-03: pitch +-2.5 deg; 3.0 / 1.5 started a false hump at drive-
// off and never ended a real one). Still detected: 10 mm side ramp
// (roll 5.1 deg), 8 mm full-width hump (pitch 5.4 deg).
#define IMU_HUMP_ON_DDEG        (40)    // 4.0 deg (was 3.0)
#define IMU_HUMP_OFF_DDEG       (25)    // 2.5 deg (was 1.5)
#define IMU_HUMP_START_MM       (20)
#define IMU_HUMP_END_MM         (IMU_HUMP_BASE_MM + 20)
#define IMU_HUMP_MAX_MM         (255U)  // 8-bit field: clamp
#define IMU_HUMP_JUMP_MM        (50)    // more per update = glitch: resync
#define IMU_HUMP_STILL_CALLS    (100U)  // updates (10 ms each) = 1 s still
#define IMU_HUMP_MAX_LEN_MM     (1000)  // longer = not a hump

// Distance smoothing (run 45 trace, 2026-11-04): while driving, pitch and
// roll carry fast vibration (sd 1.9 / 1.2 deg, uncorrelated beyond 6 mm,
// no repeat at the wheelbase or one wheel turn). Both models use the
// mean over the last IMU_HUMP_AVG_MM mm of travel. Replay of run 45:
// see testing/subsystem4/host/replay_trace.c. #ifndef so the replay can
// try other windows; keep the default in the firmware.
#ifndef IMU_HUMP_AVG_MM
#define IMU_HUMP_AVG_MM         (16)
#endif
// Drive-off: starting from standstill the acceleration tilts the
// gravity vector (run 45: pitch +14, roll -20 deg over the first ~40
// mm). No event may START in the first IMU_HUMP_DRIVEOFF_MM after the car
// was still for IMU_HUMP_STILL_CALLS updates (or after a resync). A hump
// that close to a standing start is missed (documented limit).
#ifndef IMU_HUMP_DRIVEOFF_MM
#define IMU_HUMP_DRIVEOFF_MM    (60)
#endif

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
    int32_t  len_mm;                    // travel since the event began
    uint32_t still_calls;               // updates without forward travel
    int32_t  z_now_um;                  // last profile value
    int32_t  peak_um;                   // highest z in this event
    int32_t  side_now_um;               // W x sin(|roll|), last value
    int32_t  side_peak_um;              // highest side value, this event
    int16_t  avg_p[IMU_HUMP_AVG_MM];    // last pitch per mm (0.1 deg)
    int16_t  avg_r[IMU_HUMP_AVG_MM];    // last roll per mm
    int32_t  sum_p;                     // sums of the two rings
    int32_t  sum_r;
    uint32_t avg_n;                     // ring entries filled (<= AVG)
    int32_t  driveoff_mm;               // travel left with no new start
    uint32_t last_mm;                   // last hump height
    uint32_t max_mm;                    // highest hump of the run
    bool     b_synced;                  // s_last_mm valid
    bool     b_on_hump;                 // event in progress
    bool     b_valid;                   // at least one hump measured
    bool     b_from_still;              // next forward travel = drive-off
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
                            int32_t roll_ddeg, int32_t dist_mm);
int32_t     imu_hump_now_mm(imu_hump_t const *p_hump);
uint32_t    imu_hump_pack(imu_hump_t const *p_hump, uint32_t state,
                          uint32_t event, uint32_t seq);

#endif // IMU_HUMP_H
