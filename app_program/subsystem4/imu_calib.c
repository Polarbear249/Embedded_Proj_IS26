/*
 * imu_calib.c
 * Subsystem 4 (Buddy 4): accelerometer calibration and tilt maths,
 * see imu_calib.h. Pure functions: no hardware, no kernel calls, so any
 * task may call them.
 */

#include "imu_calib.h"
#include <stddef.h>     // NULL (no kernel headers: host-testable)

// Integer atan from a tan(k deg) x 10000 table, k = 0..45
#define IMU_TAN_SCALE           (10000U)
#define IMU_TAN_STEPS           (45U)
#define IMU_DDEG_PER_DEG        (10U)       // results in 0.1 deg units
#define IMU_DDEG_90             (900)
#define IMU_DDEG_180            (1800)

static uint16_t const g_tan_x10000[IMU_TAN_STEPS + 1U] =
{
       0U,  175U,  349U,  524U,  699U,  875U, 1051U, 1228U, 1405U, 1584U,
    1763U, 1944U, 2126U, 2309U, 2493U, 2679U, 2867U, 3057U, 3249U, 3443U,
    3640U, 3839U, 4040U, 4245U, 4452U, 4663U, 4877U, 5095U, 5317U, 5543U,
    5774U, 6009U, 6249U, 6494U, 6745U, 7002U, 7265U, 7536U, 7813U, 8098U,
    8391U, 8693U, 9004U, 9325U, 9657U, 10000U
};

// sin(k deg) x 10000, k = 0..90, for imu_calib_sin_x10000()
#define IMU_SIN_STEPS           (90U)

static uint16_t const g_sin_x10000[IMU_SIN_STEPS + 1U] =
{
        0U,   175U,   349U,   523U,   698U,   872U,  1045U,  1219U,  1392U,
     1564U,  1736U,  1908U,  2079U,  2250U,  2419U,  2588U,  2756U,  2924U,
     3090U,  3256U,  3420U,  3584U,  3746U,  3907U,  4067U,  4226U,  4384U,
     4540U,  4695U,  4848U,  5000U,  5150U,  5299U,  5446U,  5592U,  5736U,
     5878U,  6018U,  6157U,  6293U,  6428U,  6561U,  6691U,  6820U,  6947U,
     7071U,  7193U,  7314U,  7431U,  7547U,  7660U,  7771U,  7880U,  7986U,
     8090U,  8192U,  8290U,  8387U,  8480U,  8572U,  8660U,  8746U,  8829U,
     8910U,  8988U,  9063U,  9135U,  9205U,  9272U,  9336U,  9397U,  9455U,
     9511U,  9563U,  9613U,  9659U,  9703U,  9744U,  9781U,  9816U,  9848U,
     9877U,  9903U,  9925U,  9945U,  9962U,  9976U,  9986U,  9994U,  9998U,
    10000U
};

static uint32_t imu_angle_ddeg(uint64_t opposite, uint64_t adjacent);
static uint32_t imu_atan_ddeg(uint32_t tan_x10000);
static uint64_t imu_abs64(int64_t value);

/*!
 * Calibration stored in imu_calib.h (measured by S4-U21).
 */
void
imu_calib_default (imu_calib_t *p_cal)
{
    if (NULL != p_cal)
    {
        p_cal->offset_mg[0] = IMU_CALIB_OFF_X_MG;
        p_cal->offset_mg[1] = IMU_CALIB_OFF_Y_MG;
        p_cal->offset_mg[2] = IMU_CALIB_OFF_Z_MG;
        p_cal->span_mg[0] = IMU_CALIB_SPAN_X_MG;
        p_cal->span_mg[1] = IMU_CALIB_SPAN_Y_MG;
        p_cal->span_mg[2] = IMU_CALIB_SPAN_Z_MG;
    }
}

/*!
 * 5-position result. p_up_mg[i] = axis i reading with that axis
 * pointing up, p_down_mg[i] = pointing down (X and Y only; Z is
 * ignored). z_zero_mg = mean Z reading with Z level (side poses).
 * Returns false (and leaves *p_cal unchanged) if any span would be <= 0.
 */
bool
imu_calib_from_poses (int32_t const p_up_mg[], int32_t const p_down_mg[],
                      int32_t z_zero_mg, imu_calib_t *p_cal)
{
    bool     b_ok = (NULL != p_up_mg) && (NULL != p_down_mg)
                    && (NULL != p_cal);
    uint32_t axis;

    for (axis = 0U; b_ok && (axis < IMU_CALIB_Z); axis++)
    {
        b_ok = (p_up_mg[axis] > p_down_mg[axis]);
    }

    b_ok = b_ok && (p_up_mg[IMU_CALIB_Z] > z_zero_mg);

    for (axis = 0U; b_ok && (axis < IMU_CALIB_Z); axis++)
    {
        p_cal->offset_mg[axis] = (p_up_mg[axis] + p_down_mg[axis]) / 2;
        p_cal->span_mg[axis] = p_up_mg[axis] - p_down_mg[axis];
    }

    if (b_ok)
    {
        p_cal->offset_mg[IMU_CALIB_Z] = z_zero_mg;
        p_cal->span_mg[IMU_CALIB_Z] = 2 * (p_up_mg[IMU_CALIB_Z] - z_zero_mg);
    }

    return (b_ok);
}

/*!
 * corrected = (raw - offset) x 2000 / span, per axis, in mg.
 */
void
imu_calib_apply (imu_calib_t const *p_cal, int32_t const p_raw[],
                 int32_t p_out[])
{
    uint32_t axis;

    if ((NULL != p_cal) && (NULL != p_raw) && (NULL != p_out))
    {
        for (axis = 0U; axis < IMU_CALIB_AXES; axis++)
        {
            p_out[axis] = ((p_raw[axis] - p_cal->offset_mg[axis])
                           * IMU_CALIB_SPAN_MG) / p_cal->span_mg[axis];
        }
    }
}

/*!
 * Pitch and roll of the gravity vector in the car frame (+X forward,
 * +Y left, +Z up): pitch = atan2(x, sqrt(y^2 + z^2)), roll = atan2(y, z).
 */
void
imu_calib_tilt (int32_t const p_acc[], int32_t *p_pitch_ddeg,
                int32_t *p_roll_ddeg)
{
    int64_t  acc_y;
    int64_t  acc_z;
    uint32_t yz_len;

    if ((NULL != p_acc) && (NULL != p_pitch_ddeg) && (NULL != p_roll_ddeg))
    {
        acc_y = p_acc[1];
        acc_z = p_acc[2];
        yz_len = imu_calib_isqrt((uint64_t)((acc_y * acc_y)
                                            + (acc_z * acc_z)));
        *p_pitch_ddeg = imu_calib_atan2_ddeg(p_acc[0], (int32_t)yz_len);
        *p_roll_ddeg = imu_calib_atan2_ddeg(p_acc[1], p_acc[2]);
    }
}

/*!
 * Store the current (corrected, car flat and still) tilt as level.
 */
void
imu_calib_level_set (imu_calib_level_t *p_level, int32_t const p_acc[])
{
    if (NULL != p_level)
    {
        imu_calib_tilt(p_acc, &p_level->pitch_ddeg, &p_level->roll_ddeg);
    }
}

/*!
 * Pitch and roll relative to the level reference.
 */
void
imu_calib_level_tilt (imu_calib_level_t const *p_level,
                      int32_t const p_acc[], int32_t *p_pitch_ddeg,
                      int32_t *p_roll_ddeg)
{
    int32_t pitch = 0;
    int32_t roll = 0;

    if ((NULL != p_level) && (NULL != p_pitch_ddeg)
        && (NULL != p_roll_ddeg))
    {
        imu_calib_tilt(p_acc, &pitch, &roll);
        *p_pitch_ddeg = pitch - p_level->pitch_ddeg;
        *p_roll_ddeg = roll - p_level->roll_ddeg;
    }
}

/*!
 * Magnetometer offset stored in imu_calib.h (measured by S4-U25).
 */
void
imu_calib_mag_default (imu_calib_mag_t *p_mag)
{
    if (NULL != p_mag)
    {
        p_mag->offset[0] = IMU_CALIB_MAG_OFF_X;
        p_mag->offset[1] = IMU_CALIB_MAG_OFF_Y;
        p_mag->offset[2] = IMU_CALIB_MAG_OFF_Z;
        p_mag->scale_x1000[0] = IMU_CALIB_MAG_SCALE_X;
        p_mag->scale_x1000[1] = IMU_CALIB_MAG_SCALE_Y;
        p_mag->scale_x1000[2] = IMU_CALIB_MAG_SCALE_Z;
    }
}

/*!
 * Hard-iron offset from a flat spin: X and Y = (max + min) / 2. Z is
 * not turned by a flat spin, so it keeps the stored value.
 */
void
imu_calib_mag_from_spin (int32_t const p_min[], int32_t const p_max[],
                         imu_calib_mag_t *p_mag)
{
    if ((NULL != p_min) && (NULL != p_max) && (NULL != p_mag))
    {
        imu_calib_mag_default(p_mag);
        p_mag->offset[0] = (p_max[0] + p_min[0]) / 2;
        p_mag->offset[1] = (p_max[1] + p_min[1]) / 2;
    }
}

/*!
 * Soft-iron scale from a flat spin: the two horizontal axes h_one and
 * h_two (the ones that turn) are scaled to their mean span, so the
 * ellipse becomes a circle. Assumes the ellipse is aligned with the
 * axes (not rotated). Leaves the offsets and the vertical axis alone.
 */
void
imu_calib_mag_scale_from_spin (int32_t const p_min[], int32_t const p_max[],
                               uint32_t h_one, uint32_t h_two,
                               imu_calib_mag_t *p_mag)
{
    int32_t span_one = 0;
    int32_t span_two = 0;
    int32_t mean = 0;

    if ((NULL != p_min) && (NULL != p_max) && (NULL != p_mag)
        && (h_one < IMU_CALIB_AXES) && (h_two < IMU_CALIB_AXES))
    {
        span_one = p_max[h_one] - p_min[h_one];
        span_two = p_max[h_two] - p_min[h_two];
        mean = (span_one + span_two) / 2;
    }

    if ((span_one > 0) && (span_two > 0))
    {
        p_mag->scale_x1000[h_one] = (mean * IMU_CALIB_MAG_SCALE_ONE)
                                    / span_one;
        p_mag->scale_x1000[h_two] = (mean * IMU_CALIB_MAG_SCALE_ONE)
                                    / span_two;
    }
}

/*!
 * corrected = (raw - offset) x scale / 1000, per axis, raw counts.
 */
void
imu_calib_mag_apply (imu_calib_mag_t const *p_mag, int32_t const p_raw[],
                     int32_t p_out[])
{
    uint32_t axis;

    if ((NULL != p_mag) && (NULL != p_raw) && (NULL != p_out))
    {
        for (axis = 0U; axis < IMU_CALIB_AXES; axis++)
        {
            p_out[axis] = ((p_raw[axis] - p_mag->offset[axis])
                           * p_mag->scale_x1000[axis])
                          / IMU_CALIB_MAG_SCALE_ONE;
        }
    }
}

/*!
 * atan2(y, x) in 0.1 deg, -1800..1800. Inputs up to a few million
 * (products of mg values) are fine.
 */
int32_t
imu_calib_atan2_ddeg (int32_t y_val, int32_t x_val)
{
    int32_t angle = (int32_t)imu_angle_ddeg(imu_abs64(y_val),
                                            imu_abs64(x_val));

    if (x_val < 0)
    {
        angle = IMU_DDEG_180 - angle;
    }

    if (y_val < 0)
    {
        angle = -angle;
    }

    return (angle);
}

// Integer square root (bit by bit)
uint32_t
imu_calib_isqrt (uint64_t value)
{
    uint64_t rem = value;
    uint64_t root = 0U;
    uint64_t bit = (uint64_t)1U << 62;

    while (bit > rem)
    {
        bit >>= 2;
    }

    while (0U != bit)
    {
        if (rem >= (root + bit))
        {
            rem -= root + bit;
            root = (root >> 1) + bit;
        }
        else
        {
            root >>= 1;
        }

        bit >>= 2;
    }

    return ((uint32_t)root);
}

// atan(opposite / adjacent) for 0..90 deg, 0.1 deg units
static uint32_t
imu_angle_ddeg (uint64_t opposite, uint64_t adjacent)
{
    uint32_t angle = 0U;

    if (0U == adjacent)
    {
        angle = (0U == opposite) ? 0U : (uint32_t)IMU_DDEG_90;
    }
    else if (opposite <= adjacent)
    {
        angle = imu_atan_ddeg((uint32_t)((opposite * IMU_TAN_SCALE)
                                         / adjacent));
    }
    else
    {
        angle = (uint32_t)IMU_DDEG_90
                - imu_atan_ddeg((uint32_t)((adjacent * IMU_TAN_SCALE)
                                           / opposite));
    }

    return (angle);
}

// tan x 10000 (0..10000) -> 0..450 (0.1 deg), linear between table steps
static uint32_t
imu_atan_ddeg (uint32_t tan_x10000)
{
    uint32_t deg = 0U;
    uint32_t span;

    while ((deg < (IMU_TAN_STEPS - 1U))
           && (tan_x10000 >= g_tan_x10000[deg + 1U]))
    {
        deg++;
    }

    span = (uint32_t)g_tan_x10000[deg + 1U] - g_tan_x10000[deg];

    return ((deg * IMU_DDEG_PER_DEG)
            + ((((tan_x10000 - g_tan_x10000[deg]) * IMU_DDEG_PER_DEG)
                + (span / 2U)) / span));
}

static uint64_t
imu_abs64 (int64_t value)
{
    return ((value < 0) ? (uint64_t)(-value) : (uint64_t)value);
}

/*!
 * sin(angle) x 10000 for an angle in 0.1 deg, clamped to -90..90 deg.
 * 1-deg table plus linear interpolation: error below 0.0002 (derived).
 */
int32_t
imu_calib_sin_x10000 (int32_t angle_ddeg)
{
    uint32_t mag = (uint32_t)((angle_ddeg < 0) ? -angle_ddeg : angle_ddeg);
    uint32_t idx = 0U;
    uint32_t frac = 0U;
    uint32_t value = 0U;
    int32_t  result = 0;

    if (mag >= (IMU_SIN_STEPS * IMU_DDEG_PER_DEG))
    {
        value = IMU_TAN_SCALE;              // sin 90 deg = 1.0000
    }
    else
    {
        idx = mag / IMU_DDEG_PER_DEG;
        frac = mag % IMU_DDEG_PER_DEG;
        value = g_sin_x10000[idx]
                + (((g_sin_x10000[idx + 1U] - g_sin_x10000[idx]) * frac)
                   / IMU_DDEG_PER_DEG);
    }

    result = (int32_t)value;

    return ((angle_ddeg < 0) ? -result : result);
}
