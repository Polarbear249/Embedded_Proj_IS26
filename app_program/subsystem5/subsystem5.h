/*
 * subsystem5.h
 * Subsystem 5 (Buddy 5): servo ultrasonic scanning and obstacle
 * profiling. Public part: task priority and the IF-09 profile word.
 *
 * Flow (Tribunal 1 safety path, CLAUDE.md section 5b):
 *  1. The obstacle task pings straight ahead every cycle (60-70 ms).
 *  2. S5_CONFIRM_PINGS readings below S5_ALERT_CM in a row: it sends an
 *     ALERT word to the Vehicle Controller. S5 never drives or stops
 *     the motors itself; the VC decides and stops the car.
 *  3. When the car is stopped and stationary (encoder distance steady),
 *     it scans left / centre / right, refines the best direction with
 *     S5_FINE_SAMPLES pings, re-centres and sends a PROFILE word.
 *  4. It waits until the stop is released, then pings again.
 *  No echo for S5_NO_ECHO_PINGS pings in a row: one FAULT word (the
 *  sensor is missing or dead), repeated only after a good reading.
 *
 * IF-09 profile word (one uint32_t, sent by value through the VC's
 * message buffer vc_mbf_profile; CLAUDE.md section 6):
 *   [8:0]   left distance cm     (9 bits, ULTRA_NO_ECHO = 0 = invalid,
 *   [17:9]  centre distance cm    400 = nothing in range)
 *   [26:18] right distance cm
 *   [28:27] kind: S5_KIND_ALERT (centre only), _PROFILE, _FAULT
 *   [30:29] best direction: S5_BEST_LEFT, _CENTRE, _RIGHT, _NONE
 *   [31]    0
 */

#ifndef SUBSYSTEM5_H
#define SUBSYSTEM5_H

#include <stdint.h>
#include "../app.h"
#include <stdbool.h>

#define S5_BYPASS_TURN_L        ((float)(S5_ANGLE_LEFT - S5_ANGLE_CENTRE))
#define S5_BYPASS_TURN_R        ((float)(S5_ANGLE_CENTRE - S5_ANGLE_RIGHT))

#define S5_PRI_MAIN             (8)         // TEAM.md section 4
#define S5_PRC_MAIN             (TP_PRC1)   // core 0: ECHO callback core

#define S5_SENSOR_TO_FRONT_CM   (8U)       // sensor to front bumper, B5's value
#define S5_ALERT_CM             (25U)       // B5's choice, TODO_TUNE
#define S5_MIN_CLEAR_CM         (34U)       // Below this, no direction is passable
#define S5_CENTRE_CLEAR_CM      (50U)       // Centre counts as "clear" only this far out
#define S5_CONFIRM_PINGS        (3U)        // in a row
#define S5_NO_ECHO_PINGS        (5U)        // in a row = sensor fault
#define S5_FINE_SAMPLES         (5U)

// Servo angles (deg). 110 = straight ahead on this mount (B5, not yet
// fine-tuned). Which end is left: CONFIRM on the car (assumed: a larger
// angle turns the sensor counter-clockwise seen from above = left).
#define S5_ANGLE_LEFT           (150U)
#define S5_ANGLE_CENTRE         (110U)
#define S5_ANGLE_RIGHT          (78U)

// 5-point course scan, outer points decide the direction, all five give the obstacle's angular extent for width estimate
#define S5_SCAN_POINTS          (5U)
#define S5_ANGLE_HALF_L         (S5_ANGLE_LEFT  - S5_ANGLE_CENTRE)  // 40
#define S5_ANGLE_HALF_R         (S5_ANGLE_CENTRE - S5_ANGLE_RIGHT)  // 32
#define S5_ANGLE_LEFT_MID       (S5_ANGLE_CENTRE + (S5_ANGLE_HALF_L / 2U))  // 130
#define S5_ANGLE_RIGHT_MID      (S5_ANGLE_CENTRE - (S5_ANGLE_HALF_R / 2U))  // 94

#define S5_TAN_LEFT_MID_MILLI   (364U)      // tan(HALF_L / 2) = tan(20)
#define S5_TAN_LEFT_MILLI       (839U)      // tan(HALF_L)     = tan(40)
#define S5_TAN_RIGHT_MID_MILLI  (287U)      // tan(HALF_R / 2) = tan(16)
#define S5_TAN_RIGHT_MILLI      (625U)      // tan(HALF_R)     = tan(32)

_Static_assert((S5_ANGLE_HALF_L >= 20U) && (S5_ANGLE_HALF_L <= 60U), "left half-angle has no tan value in the table");
_Static_assert((S5_ANGLE_HALF_R >= 20U) && (S5_ANGLE_HALF_R <= 60U), "right half-angle has no tan value in the table");
_Static_assert(0U == (S5_ANGLE_HALF_L % 2U), "left mid angle not whole");
_Static_assert(0U == (S5_ANGLE_HALF_R % 2U), "right mid angle not whole");

#define S5_KIND_ALERT           (0U)
#define S5_KIND_PROFILE         (1U)
#define S5_KIND_FAULT           (2U)

#define S5_BEST_LEFT            (0U)
#define S5_BEST_CENTRE          (1U)
#define S5_BEST_RIGHT           (2U)
#define S5_BEST_NONE            (3U)

#define S5_DIST_MASK            (0x1FFU)    // 9 bits
#define S5_P_LEFT(w)            ((uint32_t)(w) & S5_DIST_MASK)
#define S5_P_CENTRE(w)          (((uint32_t)(w) >> 9) & S5_DIST_MASK)
#define S5_P_RIGHT(w)           (((uint32_t)(w) >> 18) & S5_DIST_MASK)
#define S5_P_KIND(w)            (((uint32_t)(w) >> 27) & 0x3U)
#define S5_P_BEST(w)            (((uint32_t)(w) >> 29) & 0x3U)

ER       subsystem5_init(void);
ER       subsystem5_start(void);
void     subsystem5_test(void);
uint32_t s5_profile_pack(uint32_t left_cm, uint32_t centre_cm,
                         uint32_t right_cm, uint32_t kind, uint32_t best);
ER       s5_get_obstacle(UB *p_found, UH *p_left_cm, UH *p_right_cm,
                         UH *p_width_cm);
void s5_alert_set_enabled(bool b_on);
void s5_scan_request(void);

#endif // SUBSYSTEM5_H
