/*
 * imu_unit_test.h
 * Subsystem 4 (Buddy 4): on-target IMU tests (make TEST=1 only).
 * Same file on IMU_Branch and master; each branch's subsystem4_test()
 * calls imu_unit_test().
 */

#ifndef IMU_UNIT_TEST_H
#define IMU_UNIT_TEST_H

#include "../app.h"

#if APP_TEST
void imu_unit_test(void);
#endif

#endif // IMU_UNIT_TEST_H
