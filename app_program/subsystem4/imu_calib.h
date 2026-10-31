/*
 * imu_calib.h
 * Subsystem 4 (Buddy 4): accelerometer calibration and tilt maths.
 *
 * Two steps, both integer only (RP2040 has no FPU):
 *  1. 5-position calibration (bench, once per module). X and Y are
 *     read pointing straight up (+1 g) and straight down (-1 g):
 *     offset = (up + down) / 2, span = up - down (ideal 2000 mg).
 *     Z is never pointed down (the car cannot go upside down: parts
 *     on top), so its offset is its mean reading in the four side
 *     poses, where Z is level and should read 0, and its span is
 *     2 x (up - offset). This Z offset includes any tilt of those
 *     poses, so it is less certain than X and Y. Then
 *     corrected = (raw - offset) x 2000 / span. Cross-axis and
 *     misalignment terms are not modelled. The measured values are
 *     stored as the IMU_CALIB_* constants below: no flash writes on
 *     this platform (CLAUDE.md section 5), so they live in the source.
 *  2. Level zero (every start-up, car flat and still): the pitch and
 *     roll of the corrected gravity vector are stored as the 0 deg
 *     reference, so the mount tilt drops out of every later reading.
 *
 * Axes on the car (S4-U19 run 12): +X forward, +Y left, +Z up.
 * Pitch > 0 = nose up, roll > 0 = left side up. Angles in 0.1 deg.
 * Pitch and roll relative to level are differences of angles, which is
 * a small-angle approximation (mount tilt measured about 1.3 deg).
 */

#ifndef IMU_CALIB_H
#define IMU_CALIB_H

#include <stdbool.h>
#include <stdint.h>

#define IMU_CALIB_AXES          (3U)
#define IMU_CALIB_Z             (2U)        // index of the Z axis
#define IMU_CALIB_1G_MG         (1000)
#define IMU_CALIB_SPAN_MG       (2000)      // +1 g to -1 g, ideal

// Measured 2026-10-30, bare module off the car, 6 faces with tilt
// compensation (S4-U27..U29 PASS, run 17; check pose 995 mg corrected).
// Run 16 replayed gave -1/-12/68, 1998/1955/1956: repeatable to 3 mg.
// Re-measure if the GY-511 module is replaced.
#define IMU_CALIB_OFF_X_MG      (-3)
#define IMU_CALIB_OFF_Y_MG      (-12)
#define IMU_CALIB_OFF_Z_MG      (66)
#define IMU_CALIB_SPAN_X_MG     (1994)
#define IMU_CALIB_SPAN_Y_MG     (1956)
#define IMU_CALIB_SPAN_Z_MG     (1954)

// Magnetometer hard-iron offset, raw counts (0.080 mgauss each), from a
// flat spin (S4-U25): offset = (max + min) / 2 on X and Y. A flat spin
// does not turn Z, so Z keeps its stored value (0: needs a tilted spin,
// not done). Only the car's own magnets (motors, steel) are removed;
// fields from motor current while driving are not.
// X/Y measured 2026-10-31 on the car, module in its original (run 12)
// orientation, 2 full turns on a chair (S4-U25/U26 PASS, run 36): span
// ratio 98 %, check spin RMS 1 %, min/max 8 %, so heading error about
// +-2.3 deg there (derived). Run 28's -1176/790 was for a different
// mounting (runs 26-29); runs 14-27 were partial turns. Valid for this
// mount and roughly this place only: spin again at the venue.
#define IMU_CALIB_MAG_OFF_X     (-1141)
#define IMU_CALIB_MAG_OFF_Y     (1597)
#define IMU_CALIB_MAG_OFF_Z     (0)

// Magnetometer soft-iron scale, x1000 (1000 = no change). Steel and
// motor magnets near the sensor squash the flat-spin circle into an
// ellipse (run 26 on the car: X/Y span ratio 77 %; bare chip run 25:
// 104 %), which an offset alone cannot correct. scale = mean span /
// axis span stretches it back into a circle. Runs 28 and 36 (full
// turns, car): 989/1010 and 1008/991, within 2 % of 1000 and no better
// than offset alone (run 36: RMS 1 % vs 2 %), so kept at 1000. Run
// 26/27's 77 % "ellipse" came from partial turns, not soft iron.
#define IMU_CALIB_MAG_SCALE_ONE (1000)
#define IMU_CALIB_MAG_SCALE_X   (IMU_CALIB_MAG_SCALE_ONE)
#define IMU_CALIB_MAG_SCALE_Y   (IMU_CALIB_MAG_SCALE_ONE)
#define IMU_CALIB_MAG_SCALE_Z   (IMU_CALIB_MAG_SCALE_ONE)

typedef struct
{
    int32_t offset[IMU_CALIB_AXES];         // raw counts
    int32_t scale_x1000[IMU_CALIB_AXES];    // soft iron, always > 0
} imu_calib_mag_t;

typedef struct
{
    int32_t offset_mg[IMU_CALIB_AXES];
    int32_t span_mg[IMU_CALIB_AXES];        // always > 0
} imu_calib_t;

typedef struct
{
    int32_t pitch_ddeg;                     // car flat, 0.1 deg
    int32_t roll_ddeg;
} imu_calib_level_t;

void     imu_calib_default(imu_calib_t *p_cal);
bool     imu_calib_from_poses(int32_t const p_up_mg[],
                              int32_t const p_down_mg[],
                              int32_t z_zero_mg, imu_calib_t *p_cal);
void     imu_calib_apply(imu_calib_t const *p_cal, int32_t const p_raw[],
                         int32_t p_out[]);
void     imu_calib_tilt(int32_t const p_acc[], int32_t *p_pitch_ddeg,
                        int32_t *p_roll_ddeg);
void     imu_calib_level_set(imu_calib_level_t *p_level,
                             int32_t const p_acc[]);
void     imu_calib_level_tilt(imu_calib_level_t const *p_level,
                              int32_t const p_acc[], int32_t *p_pitch_ddeg,
                              int32_t *p_roll_ddeg);
void     imu_calib_mag_default(imu_calib_mag_t *p_mag);
void     imu_calib_mag_from_spin(int32_t const p_min[],
                                 int32_t const p_max[],
                                 imu_calib_mag_t *p_mag);
void     imu_calib_mag_scale_from_spin(int32_t const p_min[],
                                       int32_t const p_max[],
                                       uint32_t h_one, uint32_t h_two,
                                       imu_calib_mag_t *p_mag);
void     imu_calib_mag_apply(imu_calib_mag_t const *p_mag,
                             int32_t const p_raw[], int32_t p_out[]);
int32_t  imu_calib_atan2_ddeg(int32_t y_val, int32_t x_val);
int32_t  imu_calib_sin_x10000(int32_t angle_ddeg);
uint32_t imu_calib_isqrt(uint64_t value);

#endif // IMU_CALIB_H
