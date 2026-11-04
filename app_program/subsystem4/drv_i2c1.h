/*
 * drv_i2c1.h
 * Subsystem 4 (Buddy 4): polled I2C1 master for the GY-511 IMU.
 *
 * Pins: SDA = GP2, SCL = GP3 (Robo Pico Maker / Grove 2 port), 100 kHz.
 * Owner: the subsystem 4 IMU task is the only user of I2C1 (TEAM.md
 * section 3). Not thread safe: never call from two tasks.
 *
 * Written instead of using the kernel's device/i2c driver, which does
 * not send STOP after a multi-byte read and disables the block before
 * the last written byte has left the shift register.
 */

#ifndef DRV_I2C1_H
#define DRV_I2C1_H

#include <stdint.h>
#include <stdbool.h>

#define I2C1_SDA_PIN    (2U)
#define I2C1_SCL_PIN    (3U)

// I2C1 block state captured when a transfer timed out (diagnostic)
typedef struct
{
    uint32_t status;        // IC_STATUS (bit 5 MST_ACTIVITY, 2 TFE)
    uint32_t txflr;         // IC_TXFLR: commands still in the TX FIFO
    uint32_t enable_status; // IC_ENABLE_STATUS bit 0 = enabled
    uint32_t raw_intr;      // IC_RAW_INTR_STAT
    uint32_t sda;           // line levels at that moment, 1 = high
    uint32_t scl;
    uint32_t count;         // timeouts since power-up
} i2c1_diag_t;

typedef enum
{
    I2C1_OK = 0,
    I2C1_NACK_ADDR,         // no device answered at this address
    I2C1_NACK_DATA,         // device refused a data byte
    I2C1_ARB_LOST,          // another master or noise on the bus
    I2C1_ABORT_OTHER,       // any other abort, see i2c1_last_abort()
    I2C1_TIMEOUT,           // bus stuck (SCL held low?)
    I2C1_BAD_PARAM
} i2c1_result_t;

void          i2c1_init(void);
bool          i2c1_bus_is_idle(void);
i2c1_result_t i2c1_write(uint8_t addr, uint8_t const *p_data,
                         uint32_t len);
i2c1_result_t i2c1_read(uint8_t addr, uint8_t *p_data, uint32_t len);
i2c1_result_t i2c1_write_read(uint8_t addr, uint8_t reg,
                              uint8_t *p_data, uint32_t len);
uint32_t      i2c1_last_abort(void);
void          i2c1_timeout_diag(i2c1_diag_t *p_diag);
char const   *i2c1_result_name(i2c1_result_t result);

#endif // DRV_I2C1_H
