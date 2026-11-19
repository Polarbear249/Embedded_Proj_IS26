/*
 * vehicle_controller.h
 * Vehicle Controller (VC): mission state and coordination. Shared file,
 * written by Buddy 4 (CLAUDE.md section 2). The VC authorises movement;
 * only Motion Control (S2) drives the motors.
 *
 * So far only a TEST DRIVE mode (VC_TEST_DRIVE 1) for the hump tests:
 * after the IMU is ACTIVE, each press of the Robo Pico button GP20 makes
 * the car drive straight at VC_TEST_RPM for VC_TEST_DIST_MM, then stop.
 * It also stops on: a second button press, a time limit (twice the
 * expected time + 3 s), an IMU fault, or a Motion fault (S2's encoder
 * watchdog: a wheel stalled for 1 s). S2 has no command timeout, so this
 * task must always send the stop itself.
 *
 * Not started in TEST=1 builds (the IMU task, and so hump detection,
 * does not run there).
 *
 * EMERGENCY STOP (every normal build without the test drive): the
 * vc_estop task watches GP20. First press: motion_estop() (motors stop
 * and every drive command, e.g. S3's line drive, is refused). Second
 * press: motion_clear_estop(); S3's line drive then resumes at once if
 * it is still armed on the line. Worst-case reaction about 70 ms
 * (derived: 2 polls of 20-30 ms at the 10 ms tick + lock wait).
 * In test-drive builds GP20 starts and stops the test run instead.
 */

#ifndef VEHICLE_CONTROLLER_H
#define VEHICLE_CONTROLLER_H

#include "app.h"

// Off by default since S3's line follower (S3_DRIVE_ENABLE) drives the
// motors in normal builds. For hump runs: set this to 1 AND
// S3_DRIVE_SWITCH to 0 in subsystem3/subsystem3.h. Both on = build
// error (two tasks commanding the motors fight: latest command wins).
#define VC_TEST_DRIVE           (0)         // 0 = no test-drive task

// 20 RPM x 207.3 mm per wheel turn = about 69 mm/s (derived). S2 accepts
// 0..30 RPM (MOTION_MAX_TARGET_RPM).
#define VC_TEST_RPM             (20)
#define VC_TEST_DIST_MM         (1500)

#define VC_BUTTON_PIN           (20U)       // Robo Pico button (TEAM.md 2)
#define VC_POLL_MS              (100)

// Emergency-stop task: TEAM.md section 4 VC priority, any core
#define VC_ESTOP_PRI            (7)
#define VC_ESTOP_POLL_MS        (20)
#define VC_ESTOP_DEBOUNCE       (2U)        // polls low in a row = press

// Test only: below motion_control_task (14) on the same core, so a
// command never interrupts motion_update() halfway (S2 commands are not
// yet queued through Q_motion). The agreed VC priority is P7.
#define VC_TEST_PRI             (15)
#define VC_TEST_PRC             (TP_PRC2)

// IF-09 (S5 -> VC): message buffer of uint32_t profile words (layout in
// subsystem5/subsystem5.h), created by vc_init(). S5 sends with TMO_POL;
// the vc_estop task drains it every poll. On an ALERT the VC latches
// motion_estop() (GP20 releases it): the interim stop path until
// EVG_stop exists. PROFILE and FAULT words are printed only; a dead
// ultrasonic does NOT stop the car. Worst-case alert-to-stop latency
// about 160 ms (derived: 2 pings of 60-70 ms + one 20-30 ms VC poll).
#define VC_PROFILE_DEPTH        (8U)
IMPORT ID vc_mbf_profile;

/*
 * Obstacle bypass (S5 profile -> VC manoeuvre). The VC owns the motors
 * for the whole sequence: S3 and S5 are stood down, the VC clears the
 * e-stop, the car turns out of the lane, drives past and turns back.
 * S5 is then re-armed and re-alerts if anything is still inside
 * S5_ALERT_CM, which is the repeat condition.
 *
 * 45 deg and 30 cm are starting values: lateral offset is
 * 30 x sin(45) = 21 cm, and the front passes the obstacle's plane at
 * 30 x cos(45) = 21 cm beyond the 14 cm it had at the stop.
 */
#define VC_BYPASS_PRI           (7)         // TEAM.md 4: VC band
#define VC_BYPASS_POLL_MS       (20)
#define VC_BYPASS_TIMEOUT_MS    (8000U)     // wait for a fresh PROFILE
/* [Hotfix 2] The 30cm leg was fixed, so after a reverse the car turned
   back while still short of the object and drove at it. Cap the longer
   computed leg. */
#define VC_BYPASS_FWD_CM        (30.0f)     // minimum leg
#define VC_BYPASS_FWD_MAX_CM    (80.0f)     // cap on the computed leg
#define VC_BYPASS_BACK_CM       (12.0f)     // reverse before a pivot, and to re-plan
#define VC_BYPASS_RETRIES       (3U)        // boxed-in reverses before giving up
#define VC_BYPASS_ATTEMPTS      (3U)        // full manoeuvres before giving up

/* Per-move wait, derived from the distance. The encoder move is the real
   limit; this only catches a move that never ends. 5 cm/s is below the
   measured 8 to 10 cm/s, so it cannot cut a good move short. */
#define VC_BYPASS_SPEED_CMS     (5U)
#define VC_BYPASS_MOVE_EXTRA_MS (3000U)

/* Re-joining the line after a manoeuvre or a re-plan. S3 arms itself
   after 200 cycles of 10 ms with the centre sensor black, and its arm
   counter keeps running while the VC has it stood down. Holding the car
   still on the line for 2.6 s therefore hands back an armed follower,
   which is what makes the hand-back reliable. */
#define VC_BYPASS_ARM_MS        (2600U)
#define VC_BYPASS_ARM_WAIT_MS   (5000U)     // give up if the line is not held
#define VC_BYPASS_ARMED_CHECK_MS (300U)     // line must hold this long when S3 is armed
#define VC_BYPASS_RESUME_CM     (60.0f)     // seek cap when re-joining after a re-plan
#define VC_BYPASS_RESUME_MS     (1500U)     // S3 should be driving by then
/* After the turn back, the straight return to the lane is about the leg
   length again (the outbound and return angles are the same), but a
   turn that under-rotates makes the return longer. Give the seek 60 %
   more plus 20 cm so it crosses the line instead of running out short
   and leaving the search a few cm of pivot to work with. */
#define VC_BYPASS_SEEK_FACTOR   (1.6f)
#define VC_BYPASS_SEEK_MARGIN_CM (20.0f)
/* Fraction of the return leg driven with the line sensor ignored. The
   turn back swings the sensor across the floor and a black caught there
   stops the seek on the spot, before the car has driven back. */
#define VC_BYPASS_SEEK_GRACE    (0.5f)
/* Centre sensor is this far ahead of the axle. After the seek stops on
   the line, the AXLE is still 7 cm x sin(turn) short of it: drive this
   much more so the axle reaches the lane, then the straighten pivot
   puts the sensor back on the line. Without it the counter-back looks
   shorter than the way out, which is exactly what it is. */
#define VC_SENSOR_AHEAD_CM      (7.0f)
#define VC_BYPASS_SWEEP_DEG     (60.0f)     // pivot search range, each way:
                                            // 7 cm nose x sin60 is 6 cm of
                                            // lateral reach, enough to find
                                            // the line after an overshoot
#define VC_BYPASS_MIN_STOP_CM   (18U)       // closer: back off before the 60 deg pivot
#define VC_BYPASS_SAME_CM       (3U)        // a rescan within this is the same view
/* Half-hexagon bypass: two short slants at 60 deg off the lane either
   side of a long parallel pass, then the turn at the line. Most of the
   travel is parallel to the object; only the slants are at an angle. */
#define VC_BYPASS_TURN_DEG      (60.0f)
#define VC_BYPASS_SIN_TURN      (0.866f)    // sin 60
#define VC_BYPASS_COS_TURN      (0.500f)    // cos 60
#define VC_BYPASS_SIDE_CM       (16.5f)     // sideways shift: 7.5 object half,
                                            // 5 car half, 4 clear. The barcode
                                            // sensor sits proud of the flank,
                                            // so this is 5 cm more than 2/3
#define VC_BYPASS_OBJ_DEPTH_CM  (10.0f)     // object front to back
#define VC_CAR_LEN_CM           (15.0f)     // front sensor to tail
#define VC_BYPASS_PASS_MARGIN_CM (5.0f)

ER vc_init(void);
ER vc_start(void);

#endif // VEHICLE_CONTROLLER_H
