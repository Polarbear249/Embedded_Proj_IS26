/*
 * subsystem4.c
 * Subsystem 4 (Buddy 4): IMU and terrain monitoring.
 *
 * The cyclic handler and flag are created in both builds; only the
 * normal build starts the task (see subsystem4.h).
 */

#include "subsystem4.h"

ER
subsystem4_init (void)
{
    return (imu_terrain_init());
}

ER
subsystem4_start (void)
{
    ER er = E_OK;

#if APP_TEST
    tm_printf((UB*)"[IMU] test build: imu_terrain not started,"
              " unit tests own I2C1\n");
#else
    er = imu_terrain_start();
#endif

    return (er);
}
