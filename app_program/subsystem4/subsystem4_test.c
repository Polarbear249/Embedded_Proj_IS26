/*
 * subsystem4_test.c
 * Subsystem 4 (Buddy 4): on-target tests. Built only with: make TEST=1
 * Record results in testing/subsystem4/TEST_LOG.md.
 *
 * IMU_Branch: the IMU tests are the only subsystem 4 tests here.
 */

#include "subsystem4.h"
#include "imu_unit_test.h"

#if APP_TEST

void
subsystem4_test (void)
{
    imu_unit_test();
}

#endif // APP_TEST
