/*
 * subsystem4.h
 * Subsystem 4 (Buddy 4): IMU and terrain monitoring (GY-511 / LSM303D).
 *
 * Normal build: subsystem4_start() runs the imu_terrain task (P9,
 * 10 ms, TEAM.md section 4), the only owner of I2C1.
 * Test build (make TEST=1): the IMU unit tests own I2C1 instead, so the
 * task is not started (two users of I2C1 would corrupt transfers).
 */

#ifndef SUBSYSTEM4_H
#define SUBSYSTEM4_H

#include "../app.h"
#include "imu_terrain.h"

#endif // SUBSYSTEM4_H
