/*
 * subsystem4.c
 * Subsystem 4 (Buddy 4): IMU and terrain monitoring.
 *
 * IMU_Branch: no kernel objects or tasks yet. The tests own I2C1 while
 * they run (subsystem4_test.c); the imu_terrain task will take over
 * that ownership when it is written.
 */

#include "subsystem4.h"

ER
subsystem4_init (void)
{
    return (E_OK);
}

ER
subsystem4_start (void)
{
    return (E_OK);
}
