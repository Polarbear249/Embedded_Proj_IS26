/*
 * drv_i2c1.c
 * Subsystem 4 (Buddy 4): polled I2C1 master on GP2/GP3, see drv_i2c1.h.
 *
 * Register offsets and bits come from the kernel's RP2040 headers
 * (include/sys/sysdepend/cpu/rp2040/sysdef.h and
 * device/i2c/sysdepend/rp2040/i2c_sysdep.h); the TX_ABRT_SOURCE bits are
 * from the RP2040 datasheet (Pico SDK hardware/regs/i2c.h).
 */

#include "drv_i2c1.h"
#include "../app.h"
#include "../../device/i2c/sysdepend/rp2040/i2c_sysdep.h"

// I2C1 register address from an I2Cx_ offset
#define I2C1_REG(off)           ((UW)I2C1_BASE + (UW)(off))

// 100 kHz from clk_sys (SYSCLK MHz), same split as the Pico SDK
#define I2C1_BAUD_HZ            (100000U)
#define I2C1_PERIOD_CNT         ((SYSCLK * 1000000U) / I2C1_BAUD_HZ)
#define I2C1_LCNT               ((I2C1_PERIOD_CNT * 3U) / 5U)
#define I2C1_HCNT               (I2C1_PERIOD_CNT - I2C1_LCNT)
#define I2C1_SPKLEN             (I2C1_LCNT / 16U)
#define I2C1_SDA_HOLD_CNT       (((SYSCLK * 300U) / 1000U) + 1U) // 300 ns

#define I2C1_TIMEOUT_MS         (10U)   // per byte; 1 byte = 0.1 ms
#define I2C1_RECOVERY_CLOCKS    (9U)    // frees a slave stuck mid-byte

// IC_RAW_INTR_STAT / IC_TX_ABRT_SOURCE bits (RP2040 datasheet 4.3.17)
#define I2C1_ABRT_7B_ADDR_NOACK (0x00000001U)
#define I2C1_ABRT_TXDATA_NOACK  (0x00000008U)
#define I2C1_ABRT_ARB_LOST      (0x00001000U)

#define I2C1_PAD_CFG            (GPIO_IE | GPIO_PUE | GPIO_SHEMITT \
                                 | GPIO_DRIVE_4MA)

static uint32_t g_last_abort = 0U;

static void          i2c1_bus_recover(void);
static i2c1_result_t i2c1_wait_raw(uint32_t bits);
static i2c1_result_t i2c1_transfer(uint8_t addr, uint8_t const *p_wr,
                                   uint32_t wr_len, uint8_t *p_rd,
                                   uint32_t rd_len);

/*!
 * Reset the I2C1 block, free the bus if a slave holds SDA low, then
 * configure 100 kHz master mode on GP2/GP3. Call from task context.
 */
void
i2c1_init (void)
{
    set_w(RESETS_RESET, RESETS_RESET_I2C1);
    clr_w(RESETS_RESET, RESETS_RESET_I2C1);

    while (0U == (in_w(RESETS_RESET_DONE) & RESETS_RESET_I2C1))
    {
        // wait for the block to leave reset
    }

    i2c1_bus_recover();

    out_w(I2C1_REG(I2Cx_ENABLE), 0U);
    out_w(I2C1_REG(I2Cx_CON), I2C_CON_SLAVE_DISABLE | I2C_CON_MASTER_MODE
          | I2C_CON_RESTART_EN | I2C_CON_TX_EMPTY_CTRL
          | I2C_CON_7BIT_ADR | I2C_CON_SPEED_STANDARD);
    out_w(I2C1_REG(I2Cx_SS_SCL_HCNT), I2C1_HCNT);
    out_w(I2C1_REG(I2Cx_SS_SCL_LCNT), I2C1_LCNT);
    out_w(I2C1_REG(I2Cx_FS_SPKLEN), I2C1_SPKLEN);
    out_w(I2C1_REG(I2Cx_SDA_HOLD), I2C1_SDA_HOLD_CNT);
    out_w(I2C1_REG(I2Cx_RX_TL), 0U);
    out_w(I2C1_REG(I2Cx_TX_TL), 0U);
    out_w(I2C1_REG(I2Cx_DMA_CR), 0U);
    out_w(I2C1_REG(I2Cx_INTR_MASK), 0U);     // polled: no interrupts

    out_w(GPIO(I2C1_SDA_PIN), I2C1_PAD_CFG);
    out_w(GPIO(I2C1_SCL_PIN), I2C1_PAD_CFG);
    out_w(GPIO_CTRL(I2C1_SDA_PIN), GPIO_CTRL_FUNCSEL_I2C);
    out_w(GPIO_CTRL(I2C1_SCL_PIN), GPIO_CTRL_FUNCSEL_I2C);
}

/*!
 * True if both lines read high (pull-ups working, nobody holding them).
 * Only meaningful before i2c1_init() hands the pins to the I2C block.
 */
bool
i2c1_bus_is_idle (void)
{
    bool b_idle;

    out_w(GPIO(I2C1_SDA_PIN), I2C1_PAD_CFG);
    out_w(GPIO(I2C1_SCL_PIN), I2C1_PAD_CFG);
    (void)gpio_set_pin(I2C1_SDA_PIN, GPIO_MODE_IN);
    (void)gpio_set_pin(I2C1_SCL_PIN, GPIO_MODE_IN);
    tk_dly_tsk(1);

    b_idle = ((0U != gpio_get_val(I2C1_SDA_PIN))
              && (0U != gpio_get_val(I2C1_SCL_PIN)));

    return (b_idle);
}

/*!
 * Write len bytes (normally register address then value) with STOP.
 */
i2c1_result_t
i2c1_write (uint8_t addr, uint8_t const *p_data, uint32_t len)
{
    i2c1_result_t result = I2C1_BAD_PARAM;

    if ((NULL != p_data) && (0U != len))
    {
        result = i2c1_transfer(addr, p_data, len, NULL, 0U);
    }

    return (result);
}

/*!
 * Plain read of len bytes (used by the bus scan).
 */
i2c1_result_t
i2c1_read (uint8_t addr, uint8_t *p_data, uint32_t len)
{
    i2c1_result_t result = I2C1_BAD_PARAM;

    if ((NULL != p_data) && (0U != len))
    {
        result = i2c1_transfer(addr, NULL, 0U, p_data, len);
    }

    return (result);
}

/*!
 * Write the register address, repeated START, read len bytes, STOP.
 */
i2c1_result_t
i2c1_write_read (uint8_t addr, uint8_t reg, uint8_t *p_data, uint32_t len)
{
    i2c1_result_t result = I2C1_BAD_PARAM;

    if ((NULL != p_data) && (0U != len))
    {
        result = i2c1_transfer(addr, &reg, 1U, p_data, len);
    }

    return (result);
}

/*!
 * IC_TX_ABRT_SOURCE value of the last failed transfer (0 if none).
 */
uint32_t
i2c1_last_abort (void)
{
    return (g_last_abort);
}

char const *
i2c1_result_name (i2c1_result_t result)
{
    char const *p_name = "?";

    switch (result)
    {
        case I2C1_OK:
            p_name = "OK";
            break;
        case I2C1_NACK_ADDR:
            p_name = "NACK_ADDR (no device at address)";
            break;
        case I2C1_NACK_DATA:
            p_name = "NACK_DATA";
            break;
        case I2C1_ARB_LOST:
            p_name = "ARB_LOST";
            break;
        case I2C1_ABORT_OTHER:
            p_name = "ABORT_OTHER";
            break;
        case I2C1_TIMEOUT:
            p_name = "TIMEOUT (bus stuck?)";
            break;
        case I2C1_BAD_PARAM:
            p_name = "BAD_PARAM";
            break;
        default:
            p_name = "?";
            break;
    }

    return (p_name);
}

/*
 * If SDA is held low (a slave was cut off mid-byte, e.g. by a reset),
 * clock SCL until it lets go, then make a STOP. Lines are driven
 * open-drain style: "high" = input with pull-up, "low" = output 0.
 */
static void
i2c1_bus_recover (void)
{
    uint32_t clocks = 0U;

    (void)i2c1_bus_is_idle();   // pins to SIO inputs with pull-ups

    while ((0U == gpio_get_val(I2C1_SDA_PIN))
           && (clocks < I2C1_RECOVERY_CLOCKS))
    {
        (void)gpio_set_val(I2C1_SCL_PIN, 0U);
        (void)gpio_set_pin(I2C1_SCL_PIN, GPIO_MODE_OUT);
        tk_dly_tsk(1);
        (void)gpio_set_pin(I2C1_SCL_PIN, GPIO_MODE_IN);
        tk_dly_tsk(1);
        clocks++;
    }

    // STOP: SDA low -> high while SCL is high
    (void)gpio_set_val(I2C1_SDA_PIN, 0U);
    (void)gpio_set_pin(I2C1_SDA_PIN, GPIO_MODE_OUT);
    tk_dly_tsk(1);
    (void)gpio_set_pin(I2C1_SDA_PIN, GPIO_MODE_IN);
    tk_dly_tsk(1);
}

/*
 * Busy-wait until one of bits is set in IC_RAW_INTR_STAT. A TX_ABRT
 * always ends the wait and is decoded into the result.
 */
static i2c1_result_t
i2c1_wait_raw (uint32_t bits)
{
    i2c1_result_t result = I2C1_TIMEOUT;
    SYSTIM        start;
    SYSTIM        now;
    uint32_t      raw;
    uint32_t      src;
    bool          b_done = false;

    (void)tk_get_tim(&start);

    while (false == b_done)
    {
        raw = in_w(I2C1_REG(I2Cx_RAW_INTR_STAT));

        if (0U != (raw & I2C_INT_TX_ABRT))
        {
            src = in_w(I2C1_REG(I2Cx_TX_ABRT_SOURCE));
            (void)in_w(I2C1_REG(I2Cx_CLR_TX_ABRT));
            g_last_abort = src;
            b_done = true;

            if (0U != (src & I2C1_ABRT_7B_ADDR_NOACK))
            {
                result = I2C1_NACK_ADDR;
            }
            else if (0U != (src & I2C1_ABRT_TXDATA_NOACK))
            {
                result = I2C1_NACK_DATA;
            }
            else if (0U != (src & I2C1_ABRT_ARB_LOST))
            {
                result = I2C1_ARB_LOST;
            }
            else
            {
                result = I2C1_ABORT_OTHER;
            }
        }
        else if (0U != (raw & bits))
        {
            result = I2C1_OK;
            b_done = true;
        }
        else
        {
            (void)tk_get_tim(&now);
            b_done = ((uint32_t)(now.lo - start.lo) > I2C1_TIMEOUT_MS);
        }
    }

    return (result);
}

/*
 * One transaction: START, wr_len bytes, [repeated START, rd_len bytes],
 * STOP. On any abort the block sends STOP itself; we still wait for it
 * so the bus is idle before the next call.
 */
static i2c1_result_t
i2c1_transfer (uint8_t addr, uint8_t const *p_wr, uint32_t wr_len,
               uint8_t *p_rd, uint32_t rd_len)
{
    i2c1_result_t result = I2C1_OK;
    uint32_t      idx;
    uint32_t      cmd;

    out_w(I2C1_REG(I2Cx_ENABLE), 0U);
    out_w(I2C1_REG(I2Cx_TAR), (uint32_t)addr & I2C_TAR_7BIT_ADR);
    out_w(I2C1_REG(I2Cx_ENABLE), I2C_ENABLE_ENABLE);
    (void)in_w(I2C1_REG(I2Cx_CLR_INTR));

    for (idx = 0U; (idx < wr_len) && (I2C1_OK == result); idx++)
    {
        cmd = (uint32_t)p_wr[idx] | I2C_DATA_CMD_CMD_WRITE;

        if (((wr_len - 1U) == idx) && (0U == rd_len))
        {
            cmd |= I2C_DATA_CMD_STOP;
        }

        out_w(I2C1_REG(I2Cx_DATA_CMD), cmd);
        // TX_EMPTY_CTRL: set once the byte has really left the shifter
        result = i2c1_wait_raw(I2C_INT_TX_EMPTY);
    }

    for (idx = 0U; (idx < rd_len) && (I2C1_OK == result); idx++)
    {
        cmd = I2C_DATA_CMD_CMD_READ;

        if ((0U == idx) && (0U != wr_len))
        {
            cmd |= I2C_DATA_CMD_RESTART;    // turn the bus round
        }

        if ((rd_len - 1U) == idx)
        {
            cmd |= I2C_DATA_CMD_STOP;   // last byte: NACK + STOP
        }

        out_w(I2C1_REG(I2Cx_DATA_CMD), cmd);
        result = i2c1_wait_raw(I2C_INT_RX_FULL);

        if (I2C1_OK == result)
        {
            p_rd[idx] = (uint8_t)(in_w(I2C1_REG(I2Cx_DATA_CMD))
                                  & I2C_DATA_CMD_DAT);
        }
    }

    if (I2C1_TIMEOUT != result)
    {
        (void)i2c1_wait_raw(I2C_INT_STOP_DET);
    }

    (void)in_w(I2C1_REG(I2Cx_CLR_STOP_DET));
    out_w(I2C1_REG(I2Cx_ENABLE), 0U);

    return (result);
}
