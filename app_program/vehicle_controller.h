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
 */

#ifndef VEHICLE_CONTROLLER_H
#define VEHICLE_CONTROLLER_H

#include "app.h"

#define VC_TEST_DRIVE           (1)         // 0 = no test-drive task

// 20 RPM x 207.3 mm per wheel turn = about 69 mm/s (derived). S2 accepts
// 0..30 RPM (MOTION_MAX_TARGET_RPM).
#define VC_TEST_RPM             (20)
#define VC_TEST_DIST_MM         (1500)

#define VC_BUTTON_PIN           (20U)       // Robo Pico button (TEAM.md 2)
#define VC_POLL_MS              (100)

// Test only: below motion_control_task (14) on the same core, so a
// command never interrupts motion_update() halfway (S2 commands are not
// yet queued through Q_motion). The agreed VC priority is P7.
#define VC_TEST_PRI             (15)
#define VC_TEST_PRC             (TP_PRC2)

ER vc_init(void);
ER vc_start(void);

#endif // VEHICLE_CONTROLLER_H
