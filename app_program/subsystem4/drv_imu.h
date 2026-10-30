/*
 * drv_imu.h
 * Subsystem 4 (Buddy 4): GY-511 IMU driver.
 *
 * Our GY-511 board carries an ST LSM303D (WHO_AM_I = 0x49, measured
 * 2026-10-23), not the LSM303DLHC named in CLAUDE.md: 3-axis
 * accelerometer + 3-axis magnetometer behind ONE I2C address, no gyro.
 *
 * Wiring (TEAM.md section 2): SDA = GP2, SCL = GP3, VCC = 3V3, GND.
 *
 * Axes on the car (measured, S4-U19 run 12, 2026-10-26): +X forward
 * (toward the ball caster), +Y left, +Z up. The GY-511 silkscreen
 * arrows are for the LSM303DLHC and do not match: ignore them.
 * Re-run S4-U17..U19 after any remount.
 * Bus: I2C1 through drv_i2c1. Call only from the I2C1 owning task.
 * Registers and values: ST LSM303D datasheet (DocID023312).
 */

#ifndef DRV_IMU_H
#define DRV_IMU_H

#include <stdint.h>
#include "drv_i2c1.h"

#define IMU_ADDR                (0x1DU)     // 7-bit, SA0 pulled high
#define IMU_WHO_AM_I_VAL        (0x49U)

// Registers
#define IMU_WHO_AM_I            (0x0FU)
#define IMU_OUT_X_L_M           (0x08U)     // mag X,Y,Z, low byte first
#define IMU_CTRL1               (0x20U)
#define IMU_CTRL2               (0x21U)
#define IMU_CTRL5               (0x24U)
#define IMU_CTRL6               (0x25U)
#define IMU_CTRL7               (0x26U)
#define IMU_OUT_X_L_A           (0x28U)     // acc X,Y,Z, low byte first
#define IMU_AUTO_INC            (0x80U)     // OR into reg: multi-byte read

// Settings written by imu_init()
#define IMU_CTRL1_VAL           (0x5FU)     // acc 50 Hz, block update,
                                            //   X/Y/Z on
#define IMU_CTRL2_VAL           (0x00U)     // acc +-2 g: 0.061 mg/LSB
#define IMU_CTRL5_VAL           (0x70U)     // mag high res, 50 Hz,
                                            //   temperature off
#define IMU_CTRL6_VAL           (0x00U)     // mag +-2 gauss:
                                            //   0.080 mgauss/LSB
#define IMU_CTRL7_VAL           (0x00U)     // mag continuous conversion

// Accelerometer at +-2 g: 61 ug per LSB
#define IMU_ACC_UG_PER_LSB      (61)
#define IMU_UG_PER_MG           (1000)

typedef struct
{
    int16_t acc_x_mg;
    int16_t acc_y_mg;
    int16_t acc_z_mg;
    int16_t mag_x;      // raw counts, 0.080 mgauss each
    int16_t mag_y;
    int16_t mag_z;
} imu_sample_t;

i2c1_result_t imu_init(void);
i2c1_result_t imu_configure(void);
i2c1_result_t imu_read_reg(uint8_t reg, uint8_t *p_value);
i2c1_result_t imu_read(imu_sample_t *p_sample);
i2c1_result_t imu_read_acc(imu_sample_t *p_sample);
i2c1_result_t imu_read_mag(imu_sample_t *p_sample);

#endif // DRV_IMU_H
