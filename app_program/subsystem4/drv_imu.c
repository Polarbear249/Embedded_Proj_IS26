/*
 * drv_imu.c
 * Subsystem 4 (Buddy 4): GY-511 (LSM303D) driver, see drv_imu.h.
 */

#include "drv_imu.h"
#include "../app.h"     // kernel types and NULL

#define IMU_AXES_BYTES          (6U)    // 3 axes x 16 bits
#define IMU_INIT_STEPS          (5U)

static i2c1_result_t imu_write_reg(uint8_t reg, uint8_t value);
static int16_t       imu_le16(uint8_t const *p_bytes);

/*!
 * Start I2C1 and configure the accelerometer and magnetometer.
 */
i2c1_result_t
imu_init (void)
{
    i2c1_init();

    return (imu_configure());
}

/*!
 * Write the sensor configuration only, on the I2C1 set-up as it is (no
 * block reset). Used to recover after the sensor dropped off the bus.
 */
i2c1_result_t
imu_configure (void)
{
    static uint8_t const init_regs[IMU_INIT_STEPS][2] =
    {
        {IMU_CTRL1, IMU_CTRL1_VAL},
        {IMU_CTRL2, IMU_CTRL2_VAL},
        {IMU_CTRL5, IMU_CTRL5_VAL},
        {IMU_CTRL6, IMU_CTRL6_VAL},
        {IMU_CTRL7, IMU_CTRL7_VAL}
    };
    i2c1_result_t result = I2C1_OK;
    uint32_t      step;

    for (step = 0U; (step < IMU_INIT_STEPS) && (I2C1_OK == result); step++)
    {
        result = imu_write_reg(init_regs[step][0], init_regs[step][1]);
    }

    return (result);
}

i2c1_result_t
imu_read_reg (uint8_t reg, uint8_t *p_value)
{
    return (i2c1_write_read(IMU_ADDR, reg, p_value, 1U));
}

/*!
 * One accelerometer + magnetometer sample (two bursts).
 */
i2c1_result_t
imu_read (imu_sample_t *p_sample)
{
    i2c1_result_t result = imu_read_acc(p_sample);

    if (I2C1_OK == result)
    {
        result = imu_read_mag(p_sample);
    }

    return (result);
}

/*!
 * Accelerometer only: one 6-byte burst. Fills acc_*_mg, leaves mag_*.
 */
i2c1_result_t
imu_read_acc (imu_sample_t *p_sample)
{
    i2c1_result_t result = I2C1_BAD_PARAM;
    uint8_t       raw[IMU_AXES_BYTES] = {0U};

    if (NULL != p_sample)
    {
        result = i2c1_write_read(IMU_ADDR, IMU_OUT_X_L_A | IMU_AUTO_INC,
                                 raw, IMU_AXES_BYTES);
    }

    if (I2C1_OK == result)
    {
        // 16-bit counts -> mg; fits int16_t at +-2 g
        p_sample->acc_x_mg = (int16_t)(((int32_t)imu_le16(&raw[0])
                             * IMU_ACC_UG_PER_LSB) / IMU_UG_PER_MG);
        p_sample->acc_y_mg = (int16_t)(((int32_t)imu_le16(&raw[2])
                             * IMU_ACC_UG_PER_LSB) / IMU_UG_PER_MG);
        p_sample->acc_z_mg = (int16_t)(((int32_t)imu_le16(&raw[4])
                             * IMU_ACC_UG_PER_LSB) / IMU_UG_PER_MG);
    }

    return (result);
}

/*!
 * Magnetometer only: one 6-byte burst. Fills mag_*, leaves acc_*_mg.
 */
i2c1_result_t
imu_read_mag (imu_sample_t *p_sample)
{
    i2c1_result_t result = I2C1_BAD_PARAM;
    uint8_t       raw[IMU_AXES_BYTES] = {0U};

    if (NULL != p_sample)
    {
        result = i2c1_write_read(IMU_ADDR, IMU_OUT_X_L_M | IMU_AUTO_INC,
                                 raw, IMU_AXES_BYTES);
    }

    if (I2C1_OK == result)
    {
        p_sample->mag_x = imu_le16(&raw[0]);
        p_sample->mag_y = imu_le16(&raw[2]);
        p_sample->mag_z = imu_le16(&raw[4]);
    }

    return (result);
}

static i2c1_result_t
imu_write_reg (uint8_t reg, uint8_t value)
{
    uint8_t const buf[2] = {reg, value};

    return (i2c1_write(IMU_ADDR, buf, sizeof(buf)));
}

// Little-endian byte pair -> signed 16-bit
static int16_t
imu_le16 (uint8_t const *p_bytes)
{
    return ((int16_t)((uint16_t)p_bytes[0] | ((uint16_t)p_bytes[1] << 8)));
}
