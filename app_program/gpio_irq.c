/*
 * gpio_irq.c
 * Shared GPIO edge-interrupt dispatcher, see gpio_irq.h.
 *
 * IO_BANK0 registers (RP2040 datasheet 2.19.6.1): four 32-bit registers
 * per kind, 8 GPIOs each, 4 bits per GPIO (level low, level high, edge
 * low, edge high). INTR = raw edges (write 1 to clear), PROC0_INTE =
 * enable for core 0, PROC0_INTS = enabled and pending for core 0.
 *
 * Worst case in the ISR: 4 register reads, one write per register with
 * a pending edge, and one callback per pending pin.
 */

#include "gpio_irq.h"
#include <stdbool.h>
#include <tk/syslib.h>

#define GPIO_IRQ_PINS           (30U)       // GP0..GP29
#define GPIO_IRQ_REGS           (4U)
#define GPIO_IRQ_PINS_PER_REG   (8U)
#define GPIO_IRQ_BITS_PER_PIN   (4U)
#define GPIO_IRQ_EDGES_ALL      (0xCCCCCCCCU)   // edge bits of 8 pins

#define IO_BANK0_ADDR           (0x40014000U)
#define IO_BANK0_INTR(reg)      (IO_BANK0_ADDR + 0x0F0U + (4U * (reg)))
#define IO_BANK0_P0_INTE(reg)   (IO_BANK0_ADDR + 0x100U + (4U * (reg)))
#define IO_BANK0_P0_INTS(reg)   (IO_BANK0_ADDR + 0x120U + (4U * (reg)))

// Written only during the init phase (usermain, core 0), read by the ISR
static gpio_irq_cb_t g_p_cb[GPIO_IRQ_PINS];
static bool          g_b_defined = false;

static void gpio_irq_isr(UINT intno);

/*!
 * Route the given edges of pin to p_cb. Defines and enables the bank
 * interrupt on the first call. Call only from subsystemN_init().
 */
ER
gpio_irq_attach (uint32_t pin, uint32_t events, gpio_irq_cb_t p_cb)
{
    T_DINT   dint;
    ER       er = E_OK;
    uint32_t reg = 0U;
    uint32_t shift = 0U;

    if ((pin >= GPIO_IRQ_PINS) || (NULL == p_cb)
        || (0U == (events & GPIO_IRQ_EDGE_BOTH))
        || (0U != (events & ~GPIO_IRQ_EDGE_BOTH)))
    {
        er = E_PAR;
    }
    else if (NULL != g_p_cb[pin])
    {
        er = E_OBJ;                         // one owner per pin
    }
    else if (false == g_b_defined)
    {
        dint.intatr = TA_HLNG;
        dint.inthdr = (FP)gpio_irq_isr;
        er = tk_def_int(GPIO_IRQ_INTNO, &dint);
        g_b_defined = (E_OK == er);
    }
    else
    {
        // interrupt already defined by an earlier attach
    }

    if (E_OK == er)
    {
        reg = pin / GPIO_IRQ_PINS_PER_REG;
        shift = (pin % GPIO_IRQ_PINS_PER_REG) * GPIO_IRQ_BITS_PER_PIN;
        g_p_cb[pin] = p_cb;
        out_w(IO_BANK0_INTR(reg), GPIO_IRQ_EDGE_BOTH << shift); // stale
        set_w(IO_BANK0_P0_INTE(reg), events << shift);
        ClearInt(GPIO_IRQ_INTNO);
        EnableInt(GPIO_IRQ_INTNO, GPIO_IRQ_PRI);
    }

    return (er);
}

/*
 * Bank interrupt (core 0): clear each register's pending edges first, so
 * an edge that arrives during a callback raises the interrupt again
 * instead of being lost, then call the callback of each pending pin.
 */
static void
gpio_irq_isr (UINT intno)
{
    uint32_t reg;
    uint32_t idx;
    uint32_t ints;
    uint32_t events;
    uint32_t pin;

    for (reg = 0U; reg < GPIO_IRQ_REGS; reg++)
    {
        ints = in_w(IO_BANK0_P0_INTS(reg)) & GPIO_IRQ_EDGES_ALL;

        if (0U != ints)
        {
            out_w(IO_BANK0_INTR(reg), ints);

            for (idx = 0U; idx < GPIO_IRQ_PINS_PER_REG; idx++)
            {
                events = (ints >> (idx * GPIO_IRQ_BITS_PER_PIN))
                         & GPIO_IRQ_EDGE_BOTH;
                pin = (reg * GPIO_IRQ_PINS_PER_REG) + idx;

                if ((0U != events) && (pin < GPIO_IRQ_PINS)
                    && (NULL != g_p_cb[pin]))
                {
                    g_p_cb[pin](pin, events);
                }
            }
        }
    }

    ClearInt(intno);
}
