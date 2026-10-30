/*
 * imu_terrain.h
 * Subsystem 4 (Buddy 4): imu_terrain task, the only owner of I2C1.
 *
 * Period 10 ms, set by a cyclic handler that sets a bit in an event
 * flag; the task waits on that bit, so the period does not drift with
 * the work time (tk_dly_tsk would, see run 8).
 *
 * Each cycle: accelerometer burst (every cycle), magnetometer burst
 * (every IMU_TERRAIN_MAG_DIV cycles), stored calibration, fixed-point
 * IIR low-pass, pitch/roll against the start-up level reference.
 * Hump / impact / turn detection is not here yet (handoff step 5).
 *
 * Timing (A7 evidence): burst time, loop time (WCET over the window)
 * and period jitter are measured on the RP2040 1 us hardware timer and
 * printed once a second as one "[IMU] t ..." line.
 *
 * Fault: WHO_AM_I mismatch at start, or IMU_TERRAIN_FAULT_COUNT
 * consecutive failed reads, puts the task in IMU_TERRAIN_FAULT. It then
 * tries i2c1_init() + imu_init() once a second; the level reference is
 * kept (the mount did not move), so it goes straight back to ACTIVE.
 *
 * Priority: team table P9 (TEAM.md section 4). Pinned to core 0: the
 * motion tasks (S2) are pinned to core 1 and still use the template
 * priorities 12/14, so P9 on core 1 would preempt them.
 */

#ifndef IMU_TERRAIN_H
#define IMU_TERRAIN_H

#include <stdbool.h>
#include <stdint.h>
#include "../app.h"

#define IMU_TERRAIN_PRI         (9)         // TEAM.md section 4
#define IMU_TERRAIN_PRC         (TP_PRC1)   // core 0, see above
#define IMU_TERRAIN_PERIOD_MS   (10U)

// Task state, 4 bits (CLAUDE.md section 6). The hump states are added
// with the statechart.
typedef enum
{
    IMU_TERRAIN_INIT = 0,
    IMU_TERRAIN_CALIBRATING,    // level zero, car must be still
    IMU_TERRAIN_ACTIVE,
    IMU_TERRAIN_FAULT
} imu_terrain_state_t;

ER                  imu_terrain_init(void);
ER                  imu_terrain_start(void);
imu_terrain_state_t imu_terrain_get_state(void);

#endif // IMU_TERRAIN_H
