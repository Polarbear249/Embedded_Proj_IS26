/*
 * imu_unit_test.c
 * Subsystem 4 (Buddy 4): on-target IMU tests, called from
 * subsystem4_test(). Built only with: make TEST=1. Record results in
 * testing/subsystem4/TEST_LOG.md.
 *
 * GY-511 = LSM303D at 0x1D on I2C1 (SDA GP2 / SCL GP3). Every line
 * starts with [IMU] or [TEST].
 *
 * Part 1 (bench): bus, identity, configuration, one sample at rest.
 * Part 2 (S4-U17..U19, guided): finds which sensor axis points forward,
 * left and up on the car. Needs a flat block about 2 cm high. The car
 * has two rear wheels and a front ball caster. The console asks for
 * four poses in turn:
 *   1 FLAT     car on the flat floor
 *   2 NOSE UP  ball caster on the block, both wheels on the floor
 *   3 FLAT     car back on the floor (checks the module did not shift)
 *   4 LEFT UP  left wheel on the block, right wheel and ball on floor
 * At rest an accelerometer axis reads +1000 mg x cos(angle to straight
 * up), so the axis that points uphill reads MORE. Lifting the nose
 * raises the forward axis, lifting the left side raises the left axis.
 *
 * Three contact points: lifting one wheel tips the car about the line
 * from the other wheel to the ball, a diagonal, so pose 4 also changes
 * the forward axis by (track / 2) / (axle-to-ball) of the side change.
 * The left axis is therefore picked with the forward axis left out.
 * Part 2 already passed (run 12): it only runs with IMU_TEST_AXES 1.
 *
 * Part 3 (S4-U20..U22, guided, only with IMU_TEST_CAR_CAL 1; replaced
 * by the bare-module calibration S4-U27..U29): 5-position calibration.
 * Car on its wheels, on its tail, on its nose, on each side; never
 * upside down (parts on top). X and Y use their up/down pair, Z its
 * level reading in the four side poses, see imu_calib.h. Prints the
 * offsets and spans to paste into imu_calib.h, then checks |a| = 1 g
 * on a fresh flat pose with the correction applied.
 *
 * Part 4 (S4-U23..U24, guided): level zero. Car flat (tape marks the
 * wheel and ball positions) = 0 deg pitch and roll; then turned 180 deg
 * on the same spot, which splits the reading into mount tilt and floor
 * slope; then back on the marks, which must read 0 deg again.
 *
 * Part 5 (S4-U25..U26, guided): magnetometer hard-iron offset. The car
 * is turned slowly by hand, flat, through two full turns: the X/Y
 * readings trace a circle whose centre is the car's own magnetic offset
 * (motor magnets, steel). A second spin with the offset removed must
 * give a circle centred on zero, i.e. a near-constant radius.
 */

#include "imu_unit_test.h"
#include "drv_imu.h"
#include "imu_calib.h"
#include "../test_report.h"

#if APP_TEST

#define IMU_WAIT_TERM_MS        (120000)    // give up after 2 min
#define IMU_POLL_MS             (100)
#define IMU_SETTLE_MS           (1000)
#define IMU_FIRST_SAMPLE_MS     (100)
#define IMU_SCAN_FIRST_ADDR     (0x08U)     // 7-bit range that is not
#define IMU_SCAN_LAST_ADDR      (0x77U)     //   reserved by the I2C spec
#define IMU_1G_MIN_MG           (800)       // 1 g -20 %
#define IMU_1G_MAX_MG           (1200)      // 1 g +20 %
#define IMU_MAG_SAT_POS         (32767)     // full-scale reading
#define IMU_MAG_SAT_NEG         (-32768)
#define IMU_AXES                (3U)

// S4-U17..U19 guided axis test. Thresholds are test choices, not
// datasheet values: noise at rest measured about +-10 mg (run 5).
#define IMU_POSE_MOVE_MS        (5000)      // time to place the car
#define IMU_POSE_HANDS_OFF_MS   (3000)      // then hands off
#define IMU_POSE_SAMPLES        (50)        // 50 x 20 ms = 1 s average
#define IMU_POSE_MS             (20)        //   (one 50 Hz sample each)
#define IMU_POSE_TRIES          (3U)        // retries if the car moved
#define IMU_STILL_SPREAD_MG     (60)        // max - min on every axis
// Raw |a| sanity check only, so as wide as S4-U06: the raw reading is
// what calibration corrects (run 13 flat: 1120 mg). Movement is caught
// by IMU_STILL_SPREAD_MG.
#define IMU_STILL_MIN_MG        (IMU_1G_MIN_MG)     // |a| 1 g -20 %
#define IMU_STILL_MAX_MG        (IMU_1G_MAX_MG)     // |a| 1 g +20 %
#define IMU_REPEAT_MG           (30)        // flat 1 vs flat 3, per axis
#define IMU_MIN_DELTA_MG        (50)        // about 3 deg of tilt
#define IMU_UP_MIN_MG           (500)       // up axis: at least 0.5 g
#define IMU_DOMINANT_RATIO      (2)         // main axis >= 2 x the next
#define IMU_1G_MG               (1000)

#define IMU_POSE_FLAT           (0U)
#define IMU_POSE_NOSE_UP        (1U)
#define IMU_POSE_FLAT_AGAIN     (2U)
#define IMU_POSE_LEFT_UP        (3U)
#define IMU_POSES               (4U)

#define IMU_DDEG_PER_DEG        (10U)       // angles in 0.1 deg units

// The axis map is measured (S4-U19 run 12); 1 = run S4-U17..U19 again
#define IMU_TEST_AXES           (0)

// 1 = module off the car (S4-U27..U29): bench tests, then a 6-face
// calibration of the bare GY-511 in any order; the car tests (level,
// magnetometer) are skipped. Offset and gain belong to the chip, so
// they still hold after remounting; level zero removes the mount tilt.
#define IMU_TEST_BARE           (0)         // run 17 PASS: values stored

// 1 = also run the 5-position calibration on the car (S4-U20..U22).
// Off: the car cannot be stood on its tail or sides (parts on top, runs
// 14 and 18), so those poses are only tilted and the fit is nonsense.
// The stored bare-module values (S4-U27..U29) are used instead.
#define IMU_TEST_CAR_CAL        (0)
#define IMU_BARE_FACES          (6U)
#define IMU_BARE_MAX_POSES      (12U)       // tries to collect 6 faces
#define IMU_BARE_MOVE_MS        (8000)      // time to turn the module
// While collecting, the other axes still carry their unknown offset
// (run 15: Z about +73 mg), so the raw limit allows square + offset.
// Squareness proper (IMU_CAL_SQUARE_MG) is checked after the fit, with
// the offsets removed.
#define IMU_BARE_RAW_SQUARE_MG  (IMU_CAL_SQUARE_MG + IMU_CAL_MAX_OFFSET_MG)
// Tilt compensation (run 16: +Z face 8.8 deg tilted by the pin header,
// Z span 1.2 % low). After a fit, the other two axes give each face's
// tilt r (mg); its main reading is divided by cos = sqrt(1 g^2 - r^2)
// and the fit repeated. Above IMU_BARE_TILT_MAX_MG (11.5 deg) the face
// is rejected: the gain error of the small axes is then no longer small.
#define IMU_BARE_TILT_MAX_MG    (200)
#define IMU_BARE_FIT_PASSES     (2U)        // 2nd pass changes < 1 mg
#define IMU_BARE_1G_SQ          (1000000)   // (1000 mg)^2

// S4-U20..U22 5-position calibration. Limits are test choices, not
// datasheet values; confirm them after the first run.
#define IMU_CAL_POSES           (5U)
#define IMU_CAL_SIDE_POSES      (4U)        // poses 2..5: Z level
#define IMU_CAL_MOVE_MS         (10000)     // turning the car over
#define IMU_CAL_SQUARE_MG       (100)       // other axes: <= 5.7 deg off,
                                            //   cos error <= 0.5 %
#define IMU_CAL_Z_AGREE_MG      (60)        // side-pose Z readings, max -
                                            //   min (about 3.4 deg)
#define IMU_CAL_MAX_OFFSET_MG   (150)
#define IMU_CAL_SPAN_MIN_MG     (1800)      // 2000 mg -10 %
#define IMU_CAL_SPAN_MAX_MG     (2200)      // 2000 mg +10 %
#define IMU_CAL_CHECK_MG        (15)        // |a| - 1 g after correction

// S4-U23..U24 level zero
#define IMU_LEVEL_POSES         (3U)
#define IMU_LEVEL_FLAT          (0U)
#define IMU_LEVEL_TURNED        (1U)
#define IMU_LEVEL_AGAIN         (2U)
#define IMU_LEVEL_REPEAT_DDEG   (5)         // 0.5 deg

// S4-U25..U26 magnetometer flat spin. Limits are test choices, not
// datasheet values; confirm them after the first run.
#define IMU_MAG_SPINS           (2U)
#define IMU_MAG_SPIN_MS         (20000)     // two slow turns
#define IMU_MAG_SAMPLE_MS       (20)        // one 50 Hz mag sample
#define IMU_MAG_PROGRESS_MS     (5000)
#define IMU_MAG_MIN_SPAN        (2000)      // counts = 0.16 gauss: the
                                            //   car really turned
#define IMU_MAG_ROUND_PCT       (25)        // radius spread, % of mean
#define IMU_PERCENT             (100)

typedef struct
{
    int32_t  min[IMU_AXES];     // raw counts
    int32_t  max[IMU_AXES];
    uint32_t raw_r_min;         // X/Y radius without the offset
    uint32_t raw_r_max;
    uint32_t cal_r_min;         // X/Y radius with the offset removed
    uint32_t cal_r_max;
    uint32_t errors;            // failed reads
    bool     b_saturated;
} imu_mag_spin_t;

static char const * const g_spin_prompt[IMU_MAG_SPINS] =
{
    "SPIN: car flat on the floor, away from steel, laptop and phone",
    "SPIN again (check): same place, same way"
};

typedef struct
{
    char const *p_prompt;
    uint32_t    axis;           // 0 = X, 1 = Y, 2 = Z
    bool        b_up;           // that axis points up (reads +1 g)
} imu_cal_pose_t;

#if IMU_TEST_CAR_CAL
// Car axes: +X forward (ball caster), +Y left, +Z up. No -Z (upside
// down) pose: the parts on top would be crushed. Pose 1 must be +Z.
static imu_cal_pose_t const g_cal_pose[IMU_CAL_POSES] =
{
    {"+Z UP: car on its wheels, flat on the floor", 2U, true},
    {"+X UP: car standing on its rear end, ball caster at the ceiling",
     0U, true},
    {"-X UP: car standing on its nose, rear end at the ceiling",
     0U, false},
    {"+Y UP: car lying on its RIGHT side (left side at the ceiling)",
     1U, true},
    {"-Y UP: car lying on its LEFT side (right side at the ceiling)",
     1U, false}
};
#endif // IMU_TEST_CAR_CAL

static char const * const g_level_prompt[IMU_LEVEL_POSES] =
{
    "FLAT: car on the floor, mark both wheels and the ball with tape",
    "TURNED: same spot, car turned 180 deg (nose where the tail was)",
    "FLAT again: original direction, wheels and ball on the tape marks"
};

#if IMU_TEST_AXES
static char const * const g_pose_prompt[IMU_POSES] =
{
    "FLAT: car on the flat floor",
    "NOSE UP: ball caster on the block, both wheels on the floor",
    "FLAT again: car back on the floor, same spot",
    "LEFT UP: car's own LEFT wheel on the block, right wheel and ball"
    " on the floor"
};
#endif // IMU_TEST_AXES

#if IMU_TEST_AXES || IMU_TEST_BARE
// [0] = negative direction, [1] = positive direction
static char const * const g_axis_name[2][IMU_AXES] =
{
    {"-X", "-Y", "-Z"},
    {"+X", "+Y", "+Z"}
};
#endif

#if TM_CONSOLE_USB_CDC
// libtm USB glue: true once a terminal has opened the port (DTR)
extern int tm_tusb_connected(void);
#endif

static void     imu_wait_for_terminal(void);
static bool     imu_scan_bus(void);
static bool     imu_mag_axis_ok(int16_t value);
static void     imu_bench_tests(void);
#if IMU_TEST_AXES
static void     imu_axis_tests(void);
static uint32_t imu_main_axis(int32_t const p_vec[], int32_t min_mg,
                              bool *p_b_clear);
static bool     imu_right_handed(uint32_t fwd, bool b_fwd_pos,
                                 uint32_t left, bool b_left_pos,
                                 uint32_t up, bool b_up_pos);
static void     imu_print_expected_left(uint32_t fwd, bool b_fwd_pos,
                                        uint32_t up, bool b_up_pos);
static void     imu_print_ddeg(char const *p_label, uint32_t ddeg);
static uint32_t imu_tilt_ddeg(int32_t const p_ref[], int32_t const p_now[]);
#endif
#if IMU_TEST_BARE
static void     imu_bare_tests(void);
static int32_t  imu_bare_tilt(int32_t p_face[IMU_AXES], uint32_t main_axis,
                              imu_calib_t const *p_cal);
static void     imu_bare_untilt(int32_t p_face[2][IMU_AXES][IMU_AXES],
                                imu_calib_t const *p_cal, int32_t p_up[],
                                int32_t p_down[]);
#else
#if IMU_TEST_CAR_CAL
static void     imu_calib_tests(imu_calib_t *p_cal);
static bool     imu_capture_square(uint32_t idx, int32_t p_avg[]);
#endif
static void     imu_level_tests(imu_calib_t const *p_cal);
static void     imu_mag_tests(void);
#endif
static void     imu_mag_spin(uint32_t num, imu_calib_mag_t const *p_mag,
                             imu_mag_spin_t *p_spin);
static uint32_t imu_round_pct(uint32_t r_min, uint32_t r_max);
static bool     imu_capture_pose(char const *p_prompt, uint32_t num,
                                 uint32_t total, int32_t move_ms,
                                 int32_t p_avg[]);
static bool     imu_measure_still(int32_t p_avg[]);
static void     imu_print_tilt(char const *p_label, int32_t pitch_ddeg,
                               int32_t roll_ddeg);
static int32_t  imu_abs(int32_t value);
static uint32_t imu_len(int32_t const p_vec[]);

void
imu_unit_test (void)
{
#if IMU_TEST_BARE
    imu_wait_for_terminal();
    tm_printf((UB*)"\n[IMU] ===== subsystem 4 IMU test (GY-511 / LSM303D)"
              " =====\n[IMU] BARE MODULE: lay it flat on the table, still\n");

    imu_bench_tests();
    imu_bare_tests();
#else
    imu_calib_t cal;

    imu_calib_default(&cal);
    imu_wait_for_terminal();
    tm_printf((UB*)"\n[IMU] ===== subsystem 4 IMU test (GY-511 / LSM303D)"
              " =====\n[IMU] keep the car flat and still\n");

    imu_bench_tests();
#if IMU_TEST_AXES
    imu_axis_tests();
#endif
#if IMU_TEST_CAR_CAL
    imu_calib_tests(&cal);
#endif
    imu_level_tests(&cal);
    imu_mag_tests();
#endif // IMU_TEST_BARE
}

#if IMU_TEST_BARE
/*
 * Module off the car: collect all 6 faces (each axis pointing straight
 * up and straight down) in any order. Each pose is classified by the
 * axis that carries gravity, so the board's silkscreen arrows (drawn
 * for the LSM303DLHC, wrong for our LSM303D) are not needed.
 */
static void
imu_bare_tests (void)
{
    int32_t     avg[IMU_AXES] = {0, 0, 0};
    int32_t     fixed[IMU_AXES] = {0, 0, 0};
    int32_t     up_mg[IMU_AXES] = {0, 0, 0};
    int32_t     down_mg[IMU_AXES] = {0, 0, 0};
    int32_t     face[2][IMU_AXES][IMU_AXES] = {{{0}}};  // [sign][up axis]
    int32_t     tilt_mg;
    bool        b_have[2][IMU_AXES] = {{false, false, false},
                                       {false, false, false}};
    imu_calib_t measured;
    uint32_t    found = 0U;
    uint32_t    tries = 0U;
    uint32_t    main_axis;
    uint32_t    axis;
    uint32_t    pass;
    uint32_t    sign;
    uint32_t    raw_len;
    uint32_t    fixed_len;
    bool        b_square;
    bool        b_fit_square = false;
    bool        b_cal_ok = false;
    bool        b_sane = true;
    bool        b_check;

    TEST_INIT();
    imu_calib_default(&measured);

    tm_printf((UB*)"[IMU] ----- bare module: 6 faces, any order -----\n"
              "[IMU] rest each face flat on the table (or press an edge"
              " against a book); hold it still, do not wave it\n");

    while ((found < IMU_BARE_FACES) && (tries < IMU_BARE_MAX_POSES))
    {
        tm_printf((UB*)"[IMU] still needed:");

        for (sign = 0U; sign < 2U; sign++)
        {
            for (axis = 0U; axis < IMU_AXES; axis++)
            {
                if (false == b_have[sign][axis])
                {
                    tm_printf((UB*)" %s up", g_axis_name[sign][axis]);
                }
            }
        }

        tm_printf((UB*)"\n");
        tries++;

        if (imu_capture_pose("turn the module so a NEW face points up",
                             found + 1U, IMU_BARE_FACES, IMU_BARE_MOVE_MS,
                             avg))
        {
            // The axis carrying gravity, the other two near 0
            main_axis = 0U;

            for (axis = 1U; axis < IMU_AXES; axis++)
            {
                main_axis = (imu_abs(avg[axis]) > imu_abs(avg[main_axis]))
                            ? axis : main_axis;
            }

            b_square = true;

            for (axis = 0U; axis < IMU_AXES; axis++)
            {
                b_square = b_square && ((axis == main_axis)
                                        || (imu_abs(avg[axis])
                                            <= IMU_BARE_RAW_SQUARE_MG));
            }

            sign = (avg[main_axis] > 0) ? 1U : 0U;

            if (false == b_square)
            {
                tm_printf((UB*)"[IMU] badly tilted (another axis > %d mg):"
                          " rest the face flat on the table, again\n",
                          IMU_BARE_RAW_SQUARE_MG);
            }
            else if (b_have[sign][main_axis])
            {
                tm_printf((UB*)"[IMU] already have %s up: pick another"
                          " face\n", g_axis_name[sign][main_axis]);
            }
            else
            {
                b_have[sign][main_axis] = true;
                found++;

                for (axis = 0U; axis < IMU_AXES; axis++)
                {
                    face[sign][main_axis][axis] = avg[axis];
                }

                if (1U == sign)
                {
                    up_mg[main_axis] = avg[main_axis];
                }
                else
                {
                    down_mg[main_axis] = avg[main_axis];
                }

                tm_printf((UB*)"[IMU] got %s up (%d mg)  %u/%u faces\n",
                          g_axis_name[sign][main_axis],
                          (int)avg[main_axis], (unsigned)found,
                          (unsigned)IMU_BARE_FACES);
            }
        }
    }

    // Every axis has a real up/down pair, so Z uses it too. Then
    // compensate each face's tilt and fit again.
    if (IMU_BARE_FACES == found)
    {
        b_cal_ok = imu_calib_from_poses(up_mg, down_mg,
                                        (up_mg[IMU_CALIB_Z]
                                         + down_mg[IMU_CALIB_Z]) / 2,
                                        &measured);

        for (pass = 0U; b_cal_ok && (pass < IMU_BARE_FIT_PASSES); pass++)
        {
            imu_bare_untilt(face, &measured, up_mg, down_mg);
            b_cal_ok = imu_calib_from_poses(up_mg, down_mg,
                                            (up_mg[IMU_CALIB_Z]
                                             + down_mg[IMU_CALIB_Z]) / 2,
                                            &measured);
        }
    }

    // S4-U27: tilt of each face, from its other two axes with the
    // fitted offsets removed
    b_fit_square = b_cal_ok;

    for (sign = 0U; b_cal_ok && (sign < 2U); sign++)
    {
        for (main_axis = 0U; main_axis < IMU_AXES; main_axis++)
        {
            tilt_mg = imu_bare_tilt(face[sign][main_axis], main_axis,
                                    &measured);
            tm_printf((UB*)"[IMU] face %s up: tilt %d mg (compensated,"
                      " limit %d)\n", g_axis_name[sign][main_axis],
                      (int)tilt_mg, IMU_BARE_TILT_MAX_MG);
            b_fit_square = b_fit_square
                           && (tilt_mg <= IMU_BARE_TILT_MAX_MG);
        }
    }

    TEST_CHECK("S4-U27", (IMU_BARE_FACES == found) && b_fit_square,
               "bare module: 6 faces still, each tilted <= 11.5 deg");

    for (axis = 0U; axis < IMU_AXES; axis++)
    {
        b_sane = b_sane
                 && (imu_abs(measured.offset_mg[axis])
                     <= IMU_CAL_MAX_OFFSET_MG)
                 && (measured.span_mg[axis] >= IMU_CAL_SPAN_MIN_MG)
                 && (measured.span_mg[axis] <= IMU_CAL_SPAN_MAX_MG);
    }

    tm_printf((UB*)"[IMU] up   mg x=%d y=%d z=%d\n[IMU] down mg x=%d y=%d"
              " z=%d\n", (int)up_mg[0], (int)up_mg[1], (int)up_mg[2],
              (int)down_mg[0], (int)down_mg[1], (int)down_mg[2]);

    if (b_cal_ok)
    {
        tm_printf((UB*)"[IMU] paste into imu_calib.h: OFF x=%d y=%d z=%d"
                  "  SPAN x=%d y=%d z=%d\n", (int)measured.offset_mg[0],
                  (int)measured.offset_mg[1], (int)measured.offset_mg[2],
                  (int)measured.span_mg[0], (int)measured.span_mg[1],
                  (int)measured.span_mg[2]);
    }
    else
    {
        tm_printf((UB*)"[IMU] no fit: not all 6 faces were captured\n");
    }

    TEST_CHECK("S4-U28", b_cal_ok && b_sane,
               "offset <= 150 mg and span 2000 mg +-10 % on every axis");

    // S4-U29: a fresh pose, not one used for the fit
    b_check = imu_capture_pose("CHECK: any face up, flat on the table",
                               1U, 1U, IMU_BARE_MOVE_MS, avg);
    imu_calib_apply(&measured, avg, fixed);
    raw_len = imu_len(avg);
    fixed_len = imu_len(fixed);
    tm_printf((UB*)"[IMU] corrected mg x=%d y=%d z=%d  |a| raw=%u"
              " corrected=%u (1 g = 1000)\n", (int)fixed[0], (int)fixed[1],
              (int)fixed[2], (unsigned)raw_len, (unsigned)fixed_len);
    TEST_CHECK("S4-U29", b_check && b_cal_ok
               && (imu_abs((int32_t)fixed_len - IMU_1G_MG)
                   <= IMU_CAL_CHECK_MG),
               "corrected |a| on a new pose = 1000 mg +-15 mg");

    TEST_SUMMARY("subsystem4 IMU bare-module calibration");
    tm_printf((UB*)"[IMU] done. Remount the module, set IMU_TEST_BARE 0"
              " and run the car tests (level zero, magnetometer)\n");
}

/*
 * Tilt of one face in mg: length of the two non-main axes with the
 * fitted offsets removed (= 1 g x sin(tilt)).
 */
static int32_t
imu_bare_tilt (int32_t p_face[IMU_AXES], uint32_t main_axis,
               imu_calib_t const *p_cal)
{
    int32_t  side[IMU_AXES] = {0, 0, 0};
    uint32_t axis;

    for (axis = 0U; axis < IMU_AXES; axis++)
    {
        side[axis] = (axis == main_axis)
                     ? 0 : (p_face[axis] - p_cal->offset_mg[axis]);
    }

    return ((int32_t)imu_len(side));
}

/*
 * Main-axis reading of each face as if it had been level:
 * offset + (reading - offset) / cos(tilt), cos x 1000 = sqrt(1e6 - r^2).
 * Faces tilted beyond IMU_BARE_TILT_MAX_MG are left as read (and fail
 * S4-U27).
 */
static void
imu_bare_untilt (int32_t p_face[2][IMU_AXES][IMU_AXES],
                 imu_calib_t const *p_cal, int32_t p_up[], int32_t p_down[])
{
    int32_t  tilt_mg;
    int32_t  grav_mg;
    int32_t  level_mg;
    uint32_t cos_x1000;
    uint32_t sign;
    uint32_t axis;

    for (sign = 0U; sign < 2U; sign++)
    {
        for (axis = 0U; axis < IMU_AXES; axis++)
        {
            tilt_mg = imu_bare_tilt(p_face[sign][axis], axis, p_cal);
            grav_mg = p_face[sign][axis][axis] - p_cal->offset_mg[axis];
            level_mg = p_face[sign][axis][axis];

            if (tilt_mg <= IMU_BARE_TILT_MAX_MG)
            {
                cos_x1000 = imu_calib_isqrt((uint64_t)(IMU_BARE_1G_SQ
                                            - (tilt_mg * tilt_mg)));
                level_mg = p_cal->offset_mg[axis]
                           + ((grav_mg * IMU_1G_MG) / (int32_t)cos_x1000);
            }

            if (1U == sign)
            {
                p_up[axis] = level_mg;
            }
            else
            {
                p_down[axis] = level_mg;
            }
        }
    }
}
#endif // IMU_TEST_BARE

/*
 * Part 1: bus, identity, configuration and one sample at rest.
 */
static void
imu_bench_tests (void)
{
    imu_sample_t  sample = {0};
    i2c1_result_t result;
    uint8_t       value = 0U;
    int32_t       mag2;
    bool          b_idle;
    bool          b_found;

    TEST_INIT();

    // S4-U09: both lines idle high before we start (pull-ups present)
    b_idle = i2c1_bus_is_idle();
    TEST_CHECK("S4-U09", b_idle, "I2C1 SDA/SCL idle high (pull-ups ok)");

    if (false == b_idle)
    {
        tm_printf((UB*)"[IMU] a line is low: check SDA->GP2, SCL->GP3,"
                  " VCC->3V3, GND\n");
    }

    // S4-U11: I2C1 starts and the sensor ACKs its configuration
    result = imu_init();
    TEST_CHECK("S4-U11", I2C1_OK == result,
               "I2C1 init + sensor configuration writes ACKed");
    tm_printf((UB*)"[IMU] init: %s (abort src 0x%x)\n",
              i2c1_result_name(result), (unsigned)i2c1_last_abort());

    // S4-U14: bus scan finds the LSM303D
    b_found = imu_scan_bus();
    TEST_CHECK("S4-U14", b_found, "bus scan finds the LSM303D at 0x1D");

    // S4-U12: identification
    result = imu_read_reg(IMU_WHO_AM_I, &value);
    tm_printf((UB*)"[IMU] WHO_AM_I: %s, 0x%02x (expect 0x%02x)\n",
              i2c1_result_name(result), value, IMU_WHO_AM_I_VAL);
    TEST_CHECK("S4-U12",
               (I2C1_OK == result) && (IMU_WHO_AM_I_VAL == value),
               "LSM303D WHO_AM_I reads 0x49");

    // S4-U13: configuration was stored
    value = 0U;
    result = imu_read_reg(IMU_CTRL1, &value);
    tm_printf((UB*)"[IMU] CTRL1: %s, 0x%02x (expect 0x%02x)\n",
              i2c1_result_name(result), value, IMU_CTRL1_VAL);
    TEST_CHECK("S4-U13", (I2C1_OK == result) && (IMU_CTRL1_VAL == value),
               "LSM303D configured (CTRL1 read back)");

    // S4-U06 / S4-U07: one sample at rest
    tk_dly_tsk(IMU_FIRST_SAMPLE_MS);
    result = imu_read(&sample);
    mag2 = ((int32_t)sample.acc_x_mg * sample.acc_x_mg)
           + ((int32_t)sample.acc_y_mg * sample.acc_y_mg)
           + ((int32_t)sample.acc_z_mg * sample.acc_z_mg);
    tm_printf((UB*)"[IMU] at rest: %s  acc mg x=%d y=%d z=%d"
              "  mag x=%d y=%d z=%d\n", i2c1_result_name(result),
              sample.acc_x_mg, sample.acc_y_mg, sample.acc_z_mg,
              sample.mag_x, sample.mag_y, sample.mag_z);

    // Lying still, the only acceleration is gravity: |a| = 1000 mg
    TEST_CHECK("S4-U06", (I2C1_OK == result)
               && (mag2 >= (IMU_1G_MIN_MG * IMU_1G_MIN_MG))
               && (mag2 <= (IMU_1G_MAX_MG * IMU_1G_MAX_MG)),
               "accel at rest reads 1 g +-20 %");

    TEST_CHECK("S4-U07", (I2C1_OK == result)
               && imu_mag_axis_ok(sample.mag_x)
               && imu_mag_axis_ok(sample.mag_y)
               && imu_mag_axis_ok(sample.mag_z)
               && ((0 != sample.mag_x) || (0 != sample.mag_y)
                   || (0 != sample.mag_z)),
               "magnetometer reads a field, no axis saturated");

    TEST_SUMMARY("subsystem4 IMU bench");
}

#if IMU_TEST_AXES
/*
 * Part 2: guided poses -> which sensor axis is forward, left and up.
 * Works from differences to the flat pose, so a crooked mount does not
 * change the answer, only the mount tilt printed at the end.
 */
static void
imu_axis_tests (void)
{
    int32_t  pose[IMU_POSES][IMU_AXES] = {{0}};
    int32_t  fwd_delta[IMU_AXES] = {0, 0, 0};
    int32_t  left_delta[IMU_AXES] = {0, 0, 0};
    int32_t  side_delta[IMU_AXES] = {0, 0, 0};
    int32_t  up_ref[IMU_AXES] = {0, 0, 0};
    uint32_t idx;
    uint32_t axis;
    uint32_t fwd;
    uint32_t left;
    uint32_t up;
    bool     b_all_still = true;
    bool     b_repeat = true;
    bool     b_fwd_clear = false;
    bool     b_left_clear = false;
    bool     b_up_clear = false;
    bool     b_fwd_pos;
    bool     b_left_pos;
    bool     b_up_pos;
    bool     b_frame_ok;

    TEST_INIT();

    tm_printf((UB*)"[IMU] ----- axis test: you need a flat block ~2 cm"
              " high -----\n[IMU] for each pose: place the car, then hands off"
              " when told\n");

    for (idx = 0U; idx < IMU_POSES; idx++)
    {
        b_all_still = imu_capture_pose(g_pose_prompt[idx], idx + 1U,
                                       IMU_POSES, IMU_POSE_MOVE_MS,
                                       pose[idx])
                      && b_all_still;
    }

    TEST_CHECK("S4-U17", b_all_still,
               "all 4 poses held still (|a| 1 g +-20 %, spread <= 60 mg)");

    // S4-U18: flat -> nose up -> flat gives the same vector again
    for (axis = 0U; axis < IMU_AXES; axis++)
    {
        fwd_delta[axis] = pose[IMU_POSE_NOSE_UP][axis]
                          - pose[IMU_POSE_FLAT][axis];
        left_delta[axis] = pose[IMU_POSE_LEFT_UP][axis]
                           - pose[IMU_POSE_FLAT][axis];
        b_repeat = b_repeat
                   && (imu_abs(pose[IMU_POSE_FLAT_AGAIN][axis]
                               - pose[IMU_POSE_FLAT][axis])
                       <= IMU_REPEAT_MG);
    }

    tm_printf((UB*)"[IMU] flat 1 vs flat 3 (mg): dx=%d dy=%d dz=%d\n",
              (int)(pose[IMU_POSE_FLAT_AGAIN][0] - pose[IMU_POSE_FLAT][0]),
              (int)(pose[IMU_POSE_FLAT_AGAIN][1] - pose[IMU_POSE_FLAT][1]),
              (int)(pose[IMU_POSE_FLAT_AGAIN][2] - pose[IMU_POSE_FLAT][2]));
    TEST_CHECK("S4-U18", b_repeat,
               "flat pose repeats within 30 mg (module fixed on mount)");

    tm_printf((UB*)"[IMU] nose up - flat (mg): dx=%d dy=%d dz=%d\n",
              (int)fwd_delta[0], (int)fwd_delta[1], (int)fwd_delta[2]);
    tm_printf((UB*)"[IMU] left up - flat (mg): dx=%d dy=%d dz=%d\n",
              (int)left_delta[0], (int)left_delta[1], (int)left_delta[2]);

    // The axis that rose is the one pointing that way
    up = imu_main_axis(pose[IMU_POSE_FLAT], IMU_UP_MIN_MG, &b_up_clear);
    fwd = imu_main_axis(fwd_delta, IMU_MIN_DELTA_MG, &b_fwd_clear);

    // Pose 4 tips about a diagonal: drop the forward part before
    // choosing the side axis (see the file header)
    for (axis = 0U; axis < IMU_AXES; axis++)
    {
        side_delta[axis] = (axis == fwd) ? 0 : left_delta[axis];
    }

    left = imu_main_axis(side_delta, IMU_MIN_DELTA_MG, &b_left_clear);
    b_up_pos = (pose[IMU_POSE_FLAT][up] > 0);
    b_fwd_pos = (fwd_delta[fwd] > 0);
    b_left_pos = (left_delta[left] > 0);
    b_frame_ok = imu_right_handed(fwd, b_fwd_pos, left, b_left_pos,
                                  up, b_up_pos);

    tm_printf((UB*)"[IMU] AXES  forward = %s%s   left = %s%s   up = %s%s\n",
              g_axis_name[b_fwd_pos][fwd], b_fwd_clear ? "" : " (unclear)",
              g_axis_name[b_left_pos][left],
              b_left_clear ? "" : " (unclear)",
              g_axis_name[b_up_pos][up], b_up_clear ? "" : " (unclear)");

    if (false == b_frame_ok)
    {
        tm_printf((UB*)"[IMU] forward x left != up: a pose was wrong or"
                  " the module is turned on the car\n");
        imu_print_expected_left(fwd, b_fwd_pos, up, b_up_pos);
    }

    TEST_CHECK("S4-U19",
               b_fwd_clear && b_left_clear && b_up_clear && b_frame_ok,
               "forward/left/up axes found, one each, right-handed");

    // Tilt of each pose from flat; expected values in the hint below
    imu_print_ddeg("nose up tilt",
                   imu_tilt_ddeg(pose[IMU_POSE_FLAT],
                                 pose[IMU_POSE_NOSE_UP]));
    imu_print_ddeg("left up tilt",
                   imu_tilt_ddeg(pose[IMU_POSE_FLAT],
                                 pose[IMU_POSE_LEFT_UP]));

    // How far the module itself is off level when the car is flat
    up_ref[up] = b_up_pos ? IMU_1G_MG : -IMU_1G_MG;
    imu_print_ddeg("mount tilt (module vs level)",
                   imu_tilt_ddeg(up_ref, pose[IMU_POSE_FLAT]));
    tm_printf((UB*)"[IMU] expected: nose up = asin(h / L), L = rear axle"
              " to ball contact\n[IMU]           left up = asin(h / d),"
              " d = L x W / sqrt(L^2 + (W/2)^2), W = wheel track\n");

    TEST_SUMMARY("subsystem4 IMU axes");
}
#endif // IMU_TEST_AXES

#if IMU_TEST_CAR_CAL
/*
 * Part 3: 5-position calibration. On return *p_cal holds the measured
 * calibration if S4-U21 passed, else the one stored in imu_calib.h.
 */
static void
imu_calib_tests (imu_calib_t *p_cal)
{
    int32_t     avg[IMU_AXES] = {0, 0, 0};
    int32_t     fixed[IMU_AXES] = {0, 0, 0};
    int32_t     up_mg[IMU_AXES] = {0, 0, 0};
    int32_t     down_mg[IMU_AXES] = {0, 0, 0};
    int32_t     z_sum = 0;
    int32_t     z_min = INT16_MAX;
    int32_t     z_max = INT16_MIN;
    int32_t     z_zero;
    imu_calib_t measured;
    uint32_t    idx;
    uint32_t    axis;
    uint32_t    raw_len;
    uint32_t    fixed_len;
    bool        b_all_ok = true;
    bool        b_cal_ok;
    bool        b_sane = true;
    bool        b_z_agree;
    bool        b_check;

    TEST_INIT();
    imu_calib_default(&measured);

    tm_printf((UB*)"[IMU] ----- 5-position calibration: each pose puts one"
              " axis straight up or down -----\n[IMU] square the car"
              " against a wall or box edge; prop it, do not hold it\n");

    for (idx = 0U; idx < IMU_CAL_POSES; idx++)
    {
        b_all_ok = imu_capture_square(idx, avg) && b_all_ok;
        axis = g_cal_pose[idx].axis;

        if (g_cal_pose[idx].b_up)
        {
            up_mg[axis] = avg[axis];
        }
        else
        {
            down_mg[axis] = avg[axis];
        }

        // Side poses: Z is level, so its reading is its offset
        if (IMU_CALIB_Z != axis)
        {
            z_sum += avg[IMU_CALIB_Z];
            z_min = (avg[IMU_CALIB_Z] < z_min) ? avg[IMU_CALIB_Z] : z_min;
            z_max = (avg[IMU_CALIB_Z] > z_max) ? avg[IMU_CALIB_Z] : z_max;
        }
    }

    TEST_CHECK("S4-U20", b_all_ok,
               "all 5 poses still and square (other axes <= 100 mg)");

    // S4-U21: X/Y from the up/down pairs, Z from its level readings
    z_zero = z_sum / (int32_t)IMU_CAL_SIDE_POSES;
    b_z_agree = ((z_max - z_min) <= IMU_CAL_Z_AGREE_MG);
    tm_printf((UB*)"[IMU] Z level in side poses: %d..%d mg, mean %d"
              " (agree within %d)\n", (int)z_min, (int)z_max, (int)z_zero,
              IMU_CAL_Z_AGREE_MG);
    b_cal_ok = imu_calib_from_poses(up_mg, down_mg, z_zero, &measured);

    for (axis = 0U; axis < IMU_AXES; axis++)
    {
        b_sane = b_sane
                 && (imu_abs(measured.offset_mg[axis])
                     <= IMU_CAL_MAX_OFFSET_MG)
                 && (measured.span_mg[axis] >= IMU_CAL_SPAN_MIN_MG)
                 && (measured.span_mg[axis] <= IMU_CAL_SPAN_MAX_MG);
    }

    tm_printf((UB*)"[IMU] up   mg x=%d y=%d z=%d\n[IMU] down mg x=%d y=%d"
              " (z not measured)\n", (int)up_mg[0], (int)up_mg[1],
              (int)up_mg[2], (int)down_mg[0], (int)down_mg[1]);
    if (b_cal_ok)
    {
        tm_printf((UB*)"[IMU] paste into imu_calib.h: OFF x=%d y=%d z=%d"
                  "  SPAN x=%d y=%d z=%d\n", (int)measured.offset_mg[0],
                  (int)measured.offset_mg[1], (int)measured.offset_mg[2],
                  (int)measured.span_mg[0], (int)measured.span_mg[1],
                  (int)measured.span_mg[2]);
    }
    else
    {
        tm_printf((UB*)"[IMU] no fit: an axis read less pointing up than"
                  " down (pose turned the other way?)\n");
    }
    TEST_CHECK("S4-U21", b_cal_ok && b_sane && b_z_agree,
               "offset <= 150 mg, span 2000 mg +-10 %, side Z within 60");

    // S4-U22: a fresh pose, not one used for the fit
    b_check = imu_capture_pose("CHECK: car back on its wheels, flat on the"
                               " floor", 1U, 1U, IMU_POSE_MOVE_MS, avg);
    imu_calib_apply(&measured, avg, fixed);
    raw_len = imu_len(avg);
    fixed_len = imu_len(fixed);
    tm_printf((UB*)"[IMU] corrected mg x=%d y=%d z=%d  |a| raw=%u"
              " corrected=%u (1 g = 1000)\n", (int)fixed[0], (int)fixed[1],
              (int)fixed[2], (unsigned)raw_len, (unsigned)fixed_len);
    TEST_CHECK("S4-U22", b_check && b_cal_ok
               && (imu_abs((int32_t)fixed_len - IMU_1G_MG)
                   <= IMU_CAL_CHECK_MG),
               "corrected |a| on a new flat pose = 1000 mg +-15 mg");

    TEST_SUMMARY("subsystem4 IMU calibration");

    if (b_cal_ok && b_sane && b_z_agree)
    {
        *p_cal = measured;
    }
    else
    {
        tm_printf((UB*)"[IMU] calibration rejected: level test uses the"
                  " values stored in imu_calib.h\n");
    }
}
#endif // IMU_TEST_CAR_CAL

/*
 * Part 4: level zero with the stored calibration (or part 3's, if run).
 */
static void
imu_level_tests (imu_calib_t const *p_cal)
{
    int32_t           avg[IMU_AXES] = {0, 0, 0};
    int32_t           fixed[IMU_AXES] = {0, 0, 0};
    imu_calib_level_t level = {0, 0};
    int32_t           pitch = 0;
    int32_t           roll = 0;
    bool              b_level;
    bool              b_turned;
    bool              b_again;

    TEST_INIT();

    tm_printf((UB*)"[IMU] ----- level zero: you need tape to mark where the"
              " car stands -----\n");

    // S4-U23: reference pose
    b_level = imu_capture_pose(g_level_prompt[IMU_LEVEL_FLAT], 1U,
                               IMU_LEVEL_POSES, IMU_POSE_MOVE_MS, avg);
    imu_calib_apply(p_cal, avg, fixed);
    imu_calib_level_set(&level, fixed);
    imu_print_tilt("level reference (mount + floor)", level.pitch_ddeg,
                   level.roll_ddeg);
    TEST_CHECK("S4-U23", b_level, "level zero captured (car flat, still)");

    // Turned 180 deg: the floor slope changes sign, the mount does not
    b_turned = imu_capture_pose(g_level_prompt[IMU_LEVEL_TURNED], 2U,
                                IMU_LEVEL_POSES, IMU_POSE_MOVE_MS, avg);
    imu_calib_apply(p_cal, avg, fixed);
    imu_calib_tilt(fixed, &pitch, &roll);
    imu_print_tilt("turned 180", pitch, roll);
    imu_print_tilt("mount tilt  (ref + turned) / 2",
                   (level.pitch_ddeg + pitch) / 2,
                   (level.roll_ddeg + roll) / 2);
    imu_print_tilt("floor slope (ref - turned) / 2",
                   (level.pitch_ddeg - pitch) / 2,
                   (level.roll_ddeg - roll) / 2);

    if (false == b_turned)
    {
        tm_printf((UB*)"[IMU] turned pose was not still: ignore the mount"
                  " / floor split above\n");
    }

    // S4-U24: back on the marks must read 0 deg
    b_again = imu_capture_pose(g_level_prompt[IMU_LEVEL_AGAIN], 3U,
                               IMU_LEVEL_POSES, IMU_POSE_MOVE_MS, avg);
    imu_calib_apply(p_cal, avg, fixed);
    imu_calib_level_tilt(&level, fixed, &pitch, &roll);
    imu_print_tilt("flat again vs level", pitch, roll);
    TEST_CHECK("S4-U24", b_again
               && (imu_abs(pitch) <= IMU_LEVEL_REPEAT_DDEG)
               && (imu_abs(roll) <= IMU_LEVEL_REPEAT_DDEG),
               "flat again reads pitch and roll 0 +-0.5 deg");

    TEST_SUMMARY("subsystem4 IMU level zero");
}

/*
 * Part 5: magnetometer hard-iron offset from a flat spin, then a second
 * spin to check that the corrected X/Y circle is centred (round).
 */
static void
imu_mag_tests (void)
{
    imu_mag_spin_t  spin;
    imu_calib_mag_t stored;
    imu_calib_mag_t measured;
    int32_t         span_x;
    int32_t         span_y;
    uint32_t        raw_pct;
    uint32_t        cal_pct;
    bool            b_cal_ok;

    TEST_INIT();
    imu_calib_mag_default(&stored);

    tm_printf((UB*)"[IMU] ----- magnetometer: turn the car slowly by hand,"
              " flat, 2 full turns -----\n[IMU] hold it by the edges, no"
              " phone or tools in that hand, motors off\n");

    // S4-U25: offset = centre of the X/Y circle
    imu_mag_spin(1U, &stored, &spin);
    span_x = spin.max[0] - spin.min[0];
    span_y = spin.max[1] - spin.min[1];
    imu_calib_mag_from_spin(spin.min, spin.max, &measured);
    b_cal_ok = (0U == spin.errors) && (false == spin.b_saturated)
               && (span_x >= IMU_MAG_MIN_SPAN) && (span_y >= IMU_MAG_MIN_SPAN);

    tm_printf((UB*)"[IMU] mag range x=%d..%d y=%d..%d z=%d..%d"
              "  (%u read errors%s)\n", (int)spin.min[0], (int)spin.max[0],
              (int)spin.min[1], (int)spin.max[1], (int)spin.min[2],
              (int)spin.max[2], (unsigned)spin.errors,
              spin.b_saturated ? ", SATURATED" : "");
    tm_printf((UB*)"[IMU] span x=%d y=%d counts (x/y = %d %%, 100 = no"
              " soft-iron)\n", (int)span_x, (int)span_y,
              (0 != span_y) ? (int)((span_x * IMU_PERCENT) / span_y) : 0);
    tm_printf((UB*)"[IMU] paste into imu_calib.h: MAG_OFF x=%d y=%d\n",
              (int)measured.offset[0], (int)measured.offset[1]);
    TEST_CHECK("S4-U25", b_cal_ok,
               "flat spin: 0 errors, no saturation, X and Y span >= 2000");

    // S4-U26: a fresh spin, corrected with the new offset
    imu_mag_spin(2U, b_cal_ok ? &measured : &stored, &spin);
    raw_pct = imu_round_pct(spin.raw_r_min, spin.raw_r_max);
    cal_pct = imu_round_pct(spin.cal_r_min, spin.cal_r_max);
    tm_printf((UB*)"[IMU] X/Y radius raw %u..%u (spread %u %%)  corrected"
              " %u..%u (spread %u %%)\n", (unsigned)spin.raw_r_min,
              (unsigned)spin.raw_r_max, (unsigned)raw_pct,
              (unsigned)spin.cal_r_min, (unsigned)spin.cal_r_max,
              (unsigned)cal_pct);
    TEST_CHECK("S4-U26", b_cal_ok && (0U == spin.errors)
               && (false == spin.b_saturated)
               && (cal_pct <= IMU_MAG_ROUND_PCT),
               "corrected X/Y radius spread <= 25 % of its mean");

    TEST_SUMMARY("subsystem4 IMU magnetometer");
}

/*
 * One flat spin: prompt, then sample for IMU_MAG_SPIN_MS while the car
 * is turned. Records the raw min/max per axis and the X/Y radius with
 * and without the offset *p_mag, in *p_spin.
 */
static void
imu_mag_spin (uint32_t num, imu_calib_mag_t const *p_mag,
              imu_mag_spin_t *p_spin)
{
    imu_sample_t sample = {0};
    int32_t      raw[IMU_AXES] = {0, 0, 0};
    int32_t      fixed[IMU_AXES] = {0, 0, 0};
    uint32_t     raw_r;
    uint32_t     cal_r;
    uint32_t     idx;
    uint32_t     axis;
    uint32_t     good = 0U;

    p_spin->errors = 0U;
    p_spin->b_saturated = false;
    p_spin->raw_r_min = UINT32_MAX;
    p_spin->raw_r_max = 0U;
    p_spin->cal_r_min = UINT32_MAX;
    p_spin->cal_r_max = 0U;

    for (axis = 0U; axis < IMU_AXES; axis++)
    {
        p_spin->min[axis] = INT16_MAX;
        p_spin->max[axis] = INT16_MIN;
    }

    tm_printf((UB*)"\n[IMU] >>> spin %u/%u  %s  (%u s)\n", (unsigned)num,
              (unsigned)IMU_MAG_SPINS, g_spin_prompt[num - 1U],
              (unsigned)(IMU_POSE_MOVE_MS / 1000));
    tk_dly_tsk(IMU_POSE_MOVE_MS);
    tm_printf((UB*)"[IMU] TURN NOW: slowly, 2 full turns, keep it flat"
              " (%u s)\n", (unsigned)(IMU_MAG_SPIN_MS / 1000));

    for (idx = 0U; idx < (uint32_t)(IMU_MAG_SPIN_MS / IMU_MAG_SAMPLE_MS);
         idx++)
    {
        if (I2C1_OK == imu_read(&sample))
        {
            p_spin->b_saturated = p_spin->b_saturated
                                  || (false == imu_mag_axis_ok(sample.mag_x))
                                  || (false == imu_mag_axis_ok(sample.mag_y))
                                  || (false == imu_mag_axis_ok(sample.mag_z));
            raw[0] = sample.mag_x;
            raw[1] = sample.mag_y;
            raw[2] = sample.mag_z;

            for (axis = 0U; axis < IMU_AXES; axis++)
            {
                p_spin->min[axis] = (raw[axis] < p_spin->min[axis])
                                    ? raw[axis] : p_spin->min[axis];
                p_spin->max[axis] = (raw[axis] > p_spin->max[axis])
                                    ? raw[axis] : p_spin->max[axis];
            }

            // Radius in the X/Y plane only: Z does not turn
            imu_calib_mag_apply(p_mag, raw, fixed);
            raw[2] = 0;
            fixed[2] = 0;
            raw_r = imu_len(raw);
            cal_r = imu_len(fixed);
            p_spin->raw_r_min = (raw_r < p_spin->raw_r_min)
                                ? raw_r : p_spin->raw_r_min;
            p_spin->raw_r_max = (raw_r > p_spin->raw_r_max)
                                ? raw_r : p_spin->raw_r_max;
            p_spin->cal_r_min = (cal_r < p_spin->cal_r_min)
                                ? cal_r : p_spin->cal_r_min;
            p_spin->cal_r_max = (cal_r > p_spin->cal_r_max)
                                ? cal_r : p_spin->cal_r_max;
            good++;
        }
        else
        {
            p_spin->errors++;
        }

        if (0U == (((idx + 1U) * (uint32_t)IMU_MAG_SAMPLE_MS)
                   % (uint32_t)IMU_MAG_PROGRESS_MS))
        {
            tm_printf((UB*)"[IMU] %u s\n", (unsigned)(((idx + 1U)
                      * (uint32_t)IMU_MAG_SAMPLE_MS) / 1000U));
        }

        tk_dly_tsk(IMU_MAG_SAMPLE_MS);
    }

    if (0U == good)
    {
        p_spin->raw_r_min = 0U;
        p_spin->cal_r_min = 0U;
    }

    tm_printf((UB*)"[IMU] stop, put the car down\n");
}

// (max - min) as a percentage of the mean radius
static uint32_t
imu_round_pct (uint32_t r_min, uint32_t r_max)
{
    uint32_t sum = r_min + r_max;
    uint32_t pct = IMU_PERCENT;

    if ((0U != sum) && (r_max >= r_min))
    {
        pct = ((r_max - r_min) * 2U * IMU_PERCENT) / sum;
    }

    return (pct);
}

#if IMU_TEST_CAR_CAL
/*
 * One calibration pose: still (imu_capture_pose) and square (the named
 * axis has the right sign, the other two within IMU_CAL_SQUARE_MG).
 * In the side poses Z is not checked here: its raw reading is the
 * unknown Z offset. Those poses are checked by agreeing with each other
 * (IMU_CAL_Z_AGREE_MG, in imu_calib_tests).
 */
static bool
imu_capture_square (uint32_t idx, int32_t p_avg[])
{
    uint32_t const main_axis = g_cal_pose[idx].axis;
    uint32_t       tries = 0U;
    uint32_t       axis;
    bool           b_ok = false;
    bool           b_still;
    bool           b_square;

    while ((false == b_ok) && (tries < IMU_POSE_TRIES))
    {
        b_still = imu_capture_pose(g_cal_pose[idx].p_prompt, idx + 1U,
                                   IMU_CAL_POSES, IMU_CAL_MOVE_MS, p_avg);
        b_square = g_cal_pose[idx].b_up ? (p_avg[main_axis] > 0)
                                        : (p_avg[main_axis] < 0);

        for (axis = 0U; axis < IMU_AXES; axis++)
        {
            b_square = b_square && ((axis == main_axis)
                                    || ((IMU_CALIB_Z == axis)
                                        && (IMU_CALIB_Z != main_axis))
                                    || (imu_abs(p_avg[axis])
                                        <= IMU_CAL_SQUARE_MG));
        }

        b_ok = b_still && b_square;
        tries++;

        if (b_still && (false == b_square))
        {
            tm_printf((UB*)"[IMU] not square (or wrong side up): line the"
                      " car up against a wall or box edge, again\n");
        }
    }

    return (b_ok);
}
#endif // IMU_TEST_CAR_CAL

/*
 * Prompt for one pose, give time to place the car, then average 1 s.
 * Retries if the car was moving. Returns true if a still average was
 * captured; p_avg[] holds the last average either way.
 */
static bool
imu_capture_pose (char const *p_prompt, uint32_t num, uint32_t total,
                  int32_t move_ms, int32_t p_avg[])
{
    uint32_t tries = 0U;
    bool     b_still = false;

    while ((false == b_still) && (tries < IMU_POSE_TRIES))
    {
        tm_printf((UB*)"\n[IMU] >>> pose %u/%u  %s  (%u s)\n",
                  (unsigned)num, (unsigned)total, p_prompt,
                  (unsigned)(move_ms / 1000));
        tk_dly_tsk(move_ms);
        tm_printf((UB*)"[IMU] hands off, measuring in %u s\n",
                  (unsigned)(IMU_POSE_HANDS_OFF_MS / 1000));
        tk_dly_tsk(IMU_POSE_HANDS_OFF_MS);

        b_still = imu_measure_still(p_avg);
        tries++;

        if (false == b_still)
        {
            tm_printf((UB*)"[IMU] pose rejected (reason above), again\n");
        }
    }

    return (b_still);
}

/*
 * Average IMU_POSE_SAMPLES readings. Still = every read ok, every axis
 * spread within IMU_STILL_SPREAD_MG and |a| within 1 g +-20 %.
 */
static bool
imu_measure_still (int32_t p_avg[])
{
    imu_sample_t  sample = {0};
    i2c1_result_t result;
    i2c1_result_t first_err = I2C1_OK;
    int32_t       sum[IMU_AXES] = {0, 0, 0};
    int32_t       min[IMU_AXES] = {INT16_MAX, INT16_MAX, INT16_MAX};
    int32_t       max[IMU_AXES] = {INT16_MIN, INT16_MIN, INT16_MIN};
    int32_t       now[IMU_AXES] = {0, 0, 0};
    int32_t       spread = 0;
    int32_t       good = 0;
    int32_t       mag2;
    uint32_t      first_err_idx = 0U;
    uint32_t      spread_axis = 0U;
    uint32_t      idx;
    uint32_t      axis;
    bool          b_reads_ok;
    bool          b_spread_ok;
    bool          b_len_ok;

    for (idx = 0U; idx < (uint32_t)IMU_POSE_SAMPLES; idx++)
    {
        result = imu_read(&sample);

        if ((I2C1_OK != result) && (I2C1_OK == first_err))
        {
            first_err = result;
            first_err_idx = idx;
        }

        if (I2C1_OK == result)
        {
            now[0] = sample.acc_x_mg;
            now[1] = sample.acc_y_mg;
            now[2] = sample.acc_z_mg;
            good++;

            for (axis = 0U; axis < IMU_AXES; axis++)
            {
                sum[axis] += now[axis];
                min[axis] = (now[axis] < min[axis]) ? now[axis] : min[axis];
                max[axis] = (now[axis] > max[axis]) ? now[axis] : max[axis];
            }
        }

        tk_dly_tsk(IMU_POSE_MS);
    }

    for (axis = 0U; axis < IMU_AXES; axis++)
    {
        p_avg[axis] = (good > 0) ? (sum[axis] / good) : 0;

        if ((good > 0) && ((max[axis] - min[axis]) > spread))
        {
            spread = max[axis] - min[axis];
            spread_axis = axis;
        }
    }

    mag2 = (p_avg[0] * p_avg[0]) + (p_avg[1] * p_avg[1])
           + (p_avg[2] * p_avg[2]);
    tm_printf((UB*)"[IMU] avg mg x=%d y=%d z=%d  |a|=%u  spread %d mg"
              "  (%d/%d reads ok)\n", (int)p_avg[0], (int)p_avg[1],
              (int)p_avg[2], (unsigned)imu_calib_isqrt((uint64_t)mag2),
              (int)spread, (int)good, IMU_POSE_SAMPLES);

    // Say which check failed: a bad read is wiring, not movement
    b_reads_ok = (IMU_POSE_SAMPLES == good);
    b_spread_ok = (spread <= IMU_STILL_SPREAD_MG);
    b_len_ok = (mag2 >= (IMU_STILL_MIN_MG * IMU_STILL_MIN_MG))
               && (mag2 <= (IMU_STILL_MAX_MG * IMU_STILL_MAX_MG));

    if (false == b_reads_ok)
    {
        tm_printf((UB*)"[IMU] FAIL reads: %d of %d failed, first at sample"
                  " %u: %s (abort src 0x%x)\n",
                  (int)(IMU_POSE_SAMPLES - good), IMU_POSE_SAMPLES,
                  (unsigned)first_err_idx, i2c1_result_name(first_err),
                  (unsigned)i2c1_last_abort());
    }

    if (false == b_spread_ok)
    {
        tm_printf((UB*)"[IMU] FAIL still: axis %c moved %d mg (limit %d)"
                  " min %d max %d\n", (char)('X' + spread_axis),
                  (int)spread, IMU_STILL_SPREAD_MG,
                  (int)min[spread_axis], (int)max[spread_axis]);
    }

    if (false == b_len_ok)
    {
        tm_printf((UB*)"[IMU] FAIL |a|: outside %d..%d mg\n",
                  IMU_STILL_MIN_MG, IMU_STILL_MAX_MG);
    }

    return (b_reads_ok && b_spread_ok && b_len_ok);
}

#if IMU_TEST_AXES
/*
 * Index of the axis with the largest |value|. *p_b_clear: that value is
 * at least min_mg and IMU_DOMINANT_RATIO times the next largest, so the
 * axis is really aligned with the car and not a diagonal.
 */
static uint32_t
imu_main_axis (int32_t const p_vec[], int32_t min_mg, bool *p_b_clear)
{
    uint32_t best = 0U;
    uint32_t axis;
    int32_t  best_abs = 0;
    int32_t  next_abs = 0;
    int32_t  mag;

    for (axis = 0U; axis < IMU_AXES; axis++)
    {
        mag = imu_abs(p_vec[axis]);

        if (mag > best_abs)
        {
            next_abs = best_abs;
            best_abs = mag;
            best = axis;
        }
        else if (mag > next_abs)
        {
            next_abs = mag;
        }
        else
        {
            // smaller than both so far: nothing to keep
        }
    }

    *p_b_clear = (best_abs >= min_mg)
                 && (best_abs >= (IMU_DOMINANT_RATIO * next_abs));

    return (best);
}

/*
 * The LSM303D axes form a right-handed frame, so forward x left must
 * equal up. If it does not, a pose was done wrong (or the module is
 * rotated on the car). e_i x e_j = +e_k when (i, j, k) is cyclic.
 */
static bool
imu_right_handed (uint32_t fwd, bool b_fwd_pos, uint32_t left,
                  bool b_left_pos, uint32_t up, bool b_up_pos)
{
    bool     b_ok = false;
    bool     b_cyclic;
    bool     b_cross_pos;
    uint32_t third;

    if (fwd != left)
    {
        third = (IMU_AXES - fwd) - left;    // 0 + 1 + 2 = 3
        b_cyclic = (1U == (((left + IMU_AXES) - fwd) % IMU_AXES));
        b_cross_pos = ((b_fwd_pos == b_left_pos) == b_cyclic);
        b_ok = (third == up) && (b_cross_pos == b_up_pos);
    }

    return (b_ok);
}

/*
 * Print the left axis that makes forward x left = up. If the LEFT UP
 * pose gave the opposite sign, the right side was lifted instead.
 */
static void
imu_print_expected_left (uint32_t fwd, bool b_fwd_pos, uint32_t up,
                         bool b_up_pos)
{
    uint32_t axis;
    uint32_t sign;

    for (axis = 0U; axis < IMU_AXES; axis++)
    {
        for (sign = 0U; sign < 2U; sign++)
        {
            if (imu_right_handed(fwd, b_fwd_pos, axis, (1U == sign),
                                 up, b_up_pos))
            {
                tm_printf((UB*)"[IMU] with this forward and up, left must"
                          " be %s: lift the car's own LEFT wheels (as seen"
                          " from the driver's seat) and repeat\n",
                          g_axis_name[sign][axis]);
            }
        }
    }
}

static void
imu_print_ddeg (char const *p_label, uint32_t ddeg)
{
    tm_printf((UB*)"[IMU] %s %u.%u deg\n", p_label,
              (unsigned)(ddeg / IMU_DDEG_PER_DEG),
              (unsigned)(ddeg % IMU_DDEG_PER_DEG));
}

/*
 * Angle between two gravity vectors, 0.1 deg units, from
 * atan2(|ref x now|, ref . now). Exact integer inputs, so small angles
 * keep their resolution (an acos of the dot product would not).
 */
static uint32_t
imu_tilt_ddeg (int32_t const p_ref[], int32_t const p_now[])
{
    int64_t  ref_x = p_ref[0];
    int64_t  ref_y = p_ref[1];
    int64_t  ref_z = p_ref[2];
    int64_t  now_x = p_now[0];
    int64_t  now_y = p_now[1];
    int64_t  now_z = p_now[2];
    int64_t  cross_x = (ref_y * now_z) - (ref_z * now_y);
    int64_t  cross_y = (ref_z * now_x) - (ref_x * now_z);
    int64_t  cross_z = (ref_x * now_y) - (ref_y * now_x);
    int64_t  dot = (ref_x * now_x) + (ref_y * now_y) + (ref_z * now_z);
    uint64_t cross2 = (uint64_t)((cross_x * cross_x) + (cross_y * cross_y)
                                 + (cross_z * cross_z));
    uint32_t opposite = imu_calib_isqrt(cross2);

    // |cross| and dot are below about 2e6 for mg inputs: fit int32_t;
    // opposite >= 0, so the result is 0..1800
    return ((uint32_t)imu_calib_atan2_ddeg((int32_t)opposite,
                                           (int32_t)dot));
}
#endif // IMU_TEST_AXES

static int32_t
imu_abs (int32_t value)
{
    return ((value < 0) ? -value : value);
}

// |v| in mg
static uint32_t
imu_len (int32_t const p_vec[])
{
    int64_t vec_x = p_vec[0];
    int64_t vec_y = p_vec[1];
    int64_t vec_z = p_vec[2];

    return (imu_calib_isqrt((uint64_t)((vec_x * vec_x) + (vec_y * vec_y)
                                       + (vec_z * vec_z))));
}

// One line: "<label> pitch -1.3 deg  roll 0.4 deg"
static void
imu_print_tilt (char const *p_label, int32_t pitch_ddeg, int32_t roll_ddeg)
{
    uint32_t pitch_abs = (uint32_t)imu_abs(pitch_ddeg);
    uint32_t roll_abs = (uint32_t)imu_abs(roll_ddeg);

    tm_printf((UB*)"[IMU] %s  pitch %s%u.%u deg  roll %s%u.%u deg\n",
              p_label, (pitch_ddeg < 0) ? "-" : "",
              (unsigned)(pitch_abs / IMU_DDEG_PER_DEG),
              (unsigned)(pitch_abs % IMU_DDEG_PER_DEG),
              (roll_ddeg < 0) ? "-" : "",
              (unsigned)(roll_abs / IMU_DDEG_PER_DEG),
              (unsigned)(roll_abs % IMU_DDEG_PER_DEG));
}

static bool
imu_mag_axis_ok (int16_t value)
{
    return ((IMU_MAG_SAT_POS != value) && (IMU_MAG_SAT_NEG != value));
}

/*
 * Probe every address with a 1-byte read and list who answers.
 * Returns true if the LSM303D answered.
 */
static bool
imu_scan_bus (void)
{
    uint8_t  addr;
    uint8_t  dummy = 0U;
    uint32_t count = 0U;
    bool     b_imu = false;

    tm_printf((UB*)"[IMU] I2C1 scan:");

    for (addr = IMU_SCAN_FIRST_ADDR; addr <= IMU_SCAN_LAST_ADDR; addr++)
    {
        if (I2C1_OK == i2c1_read(addr, &dummy, 1U))
        {
            tm_printf((UB*)" 0x%02x", addr);
            count++;
            b_imu = (b_imu || (IMU_ADDR == addr));
        }
    }

    tm_printf((UB*)"  (%u found)\n", (unsigned)count);

    return (b_imu);
}

/*
 * USB output printed before a terminal opens the port is lost, so wait
 * until one is connected (PuTTY / VS Code Serial Monitor set DTR).
 */
static void
imu_wait_for_terminal (void)
{
#if TM_CONSOLE_USB_CDC
    int32_t waited_ms = 0;

    while ((0 == tm_tusb_connected()) && (waited_ms < IMU_WAIT_TERM_MS))
    {
        tk_dly_tsk(IMU_POLL_MS);
        waited_ms += IMU_POLL_MS;
    }

    tk_dly_tsk(IMU_SETTLE_MS);      // let the terminal settle
#endif
}

#endif // APP_TEST
