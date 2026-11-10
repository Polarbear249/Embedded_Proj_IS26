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
#define VC_BYPASS_TIMEOUT_MS    (8000U)     // per move
//#define VC_BYPASS_TURN_DEG      (45.0f)     // TUNE on the floor
#define VC_BYPASS_FWD_CM        (30.0f)     // TUNE
//#define VC_BYPASS_RETURN_DEG    (90.0f)     // 2x TURN_DEG to face the line again
#define VC_BYPASS_SEEK_CM       (40.0f)     // It will drive looking for the line
#define VC_BYPASS_SEEK_MS       (10000U)     // timeout
#define VC_BYPASS_BACK_CM       (12.0f)     // if the line is not found, back up and try again
#define VC_BYPASS_RETRIES       (3U)        // how many times to try the bypass manoeuvre

ER vc_init(void);
ER vc_start(void);

#endif // VEHICLE_CONTROLLER_H
