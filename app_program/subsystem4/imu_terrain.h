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
 * Then hump detection and height (imu_hump.c) from the pitch and the
 * IF-10 wheel distance; the result is the IF-11 hump word. Samples
 * that are not mostly gravity (|a| far from 1 g) are kept out of the
 * tilt filter. Turn detection is not here yet.
 *
 * LIFT / OVERTURN (prof's change 2026-11-13, replaces collision /
 * impact detection): imu_lift.c watches every sample. A lifted car
 * (upward acceleration) or an overturned one (tilt > 45 deg for 200 ms)
 * puts the task in IMU_TERRAIN_LIFTED / IMU_TERRAIN_OVERTURNED. The IMU
 * never drives the motors: the Vehicle Controller polls
 * imu_terrain_get_state() every 20 ms and latches motion_estop(); in the
 * demo, lifting the car ends the run. Back on its wheels, level and still
 * for 1 s, the task returns to ACTIVE, but the motors stay stopped until
 * GP20 is pressed. Hump detection pauses while lifted.
 *
 * Timing (A7 evidence): burst time, loop time (WCET over the window)
 * and period jitter are measured on the RP2040 1 us hardware timer and
 * printed once a second as one "[IMU] t ..." line.
 *
 * Fault: WHO_AM_I mismatch at start, or IMU_TERRAIN_FAULT_COUNT
 * consecutive failed reads, puts the task in IMU_TERRAIN_FAULT. Once a
 * second it then probes WHO_AM_I on the existing I2C1 set-up and, if the
 * sensor answers, rewrites its configuration; it re-initialises I2C1
 * (bus recovery + imu_init) only when SDA is held low. The level
 * reference is kept (the mount did not move), so it goes straight back
 * to ACTIVE.
 *
 * Priority: team table P9 (TEAM.md section 4). Pinned to core 0; the
 * motion tasks (S2, P5) are pinned to core 1.
 */

#ifndef IMU_TERRAIN_H
#define IMU_TERRAIN_H

#include <stdbool.h>
#include <stdint.h>
#include "../app.h"

#define IMU_TERRAIN_PRI         (9)         // TEAM.md section 4
#define IMU_TERRAIN_PRC         (TP_PRC1)   // core 0, see above
#define IMU_TERRAIN_PERIOD_MS   (10U)

// State, 4 bits (CLAUDE.md section 6). The task runs INIT, CALIBRATING,
// ACTIVE, FAULT, LIFTED and OVERTURNED; HUMP_UP / HUMP_DOWN are what the
// IF-11 word reports while ACTIVE and on a hump (FR8: climbing /
// descending). LIFTED / OVERTURNED = stop the motors (see above).
typedef enum
{
    IMU_TERRAIN_INIT = 0,
    IMU_TERRAIN_CALIBRATING,    // level zero, car must be still
    IMU_TERRAIN_ACTIVE,
    IMU_TERRAIN_FAULT,
    IMU_TERRAIN_HUMP_UP,
    IMU_TERRAIN_HUMP_DOWN,
    IMU_TERRAIN_LIFTED,         // picked up: end of run, motors stop
    IMU_TERRAIN_OVERTURNED      // tipped over: motors stop
} imu_terrain_state_t;

// True in LIFTED or OVERTURNED: the Vehicle Controller must stop
bool                imu_terrain_is_lifted(void);

ER                  imu_terrain_init(void);
ER                  imu_terrain_start(void);
imu_terrain_state_t imu_terrain_get_state(void);
uint32_t            imu_terrain_get_hump_word(void);

#endif // IMU_TERRAIN_H
