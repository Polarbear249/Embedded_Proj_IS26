/*
 * drv_ir.c
 * Subsystem 3 (Buddy 3): IR sensor inputs and barcode edge interrupt,
 * see drv_ir.h.
 *
 * Why tk_def_int() and not the Pico SDK's gpio_set_irq_enabled_with_
 * callback(): in this kernel build the SDK's IRQ registration functions
 * are empty stubs (lib/libwifi/sysdepend/pico_rp2040/cyw43_utk_compat.c),
 * so an SDK callback would never run. The kernel owns the vector table.
 *
 * IO_BANK0 interrupt registers (RP2040 datasheet 2.19.6.1, IO_BANK0):
 *   INTR0..3        0x0F0  raw status, edge bits write-1-to-clear
 *   PROC0_INTE0..3  0x100  enables for core 0's IO_IRQ_BANK0
 *   PROC1_INTE0..3  0x130  enables for core 1's IO_IRQ_BANK0
 * 4 bits per GPIO, 8 GPIOs per register:
 *   bit 4n+0 LEVEL_LOW, 4n+1 LEVEL_HIGH, 4n+2 EDGE_LOW, 4n+3 EDGE_HIGH
 * IO_IRQ_BANK0 is interrupt 13 (RP2040 datasheet 2.3.2).
 *
 * NOTE (TEAM.md section 3): this module is the only user of
 * IO_IRQ_BANK0 today. If another subsystem needs GPIO interrupts
 * (e.g. S5 ECHO on GP17), the handler must become one shared dispatcher;
 * a second tk_def_int() on interrupt 13 would replace this one.
 */

#include "drv_ir.h"

#define DRV_IR_INTNO_IO_BANK0   (13U)
#define DRV_IR_INT_LEVEL        (2)         // same level as the kernel's
                                            //   physical timer (INTPRI_PTIM)

#define DRV_IR_IO_INTR0         (IO_BANK0_BASE + 0x0F0U)
#define DRV_IR_IO_PROC0_INTE0   (IO_BANK0_BASE + 0x100U)
#define DRV_IR_IO_PROC1_INTE0   (IO_BANK0_BASE + 0x130U)
#define DRV_IR_IO_REG(base, pin) ((base) + (((pin) / 8U) * 4U))
#define DRV_IR_EDGE_LOW(pin)    ((uint32_t)1U << ((((pin) % 8U) * 4U) + 2U))
#define DRV_IR_EDGE_HIGH(pin)   ((uint32_t)1U << ((((pin) % 8U) * 4U) + 3U))
#define DRV_IR_EDGE_BOTH(pin)   (DRV_IR_EDGE_LOW(pin) | DRV_IR_EDGE_HIGH(pin))

// SIO CPUID: 0 on core 0, 1 on core 1 (sysdef.h SIO_CPUID_ADDR)
#define DRV_IR_CPUID            (SIO_CPUID_ADDR)

// Pad: input on, output off, Schmitt trigger on, pull-DOWN on. The
// module drives DOUT itself; the pull-down only matters if a sensor
// cable falls off: the pin then reads LOW = white, which the line logic
// treats as "no line" and the car searches and then stops (fail safe).
#define DRV_IR_PAD_CFG          (GPIO_IE | GPIO_OD | GPIO_SHEMITT \
                                 | GPIO_PDE)

// Ring of barcode edges. 32 entries = 1.1 symbols (29 elements) of
// slack, emptied every 10 ms; at 1 m/s a symbol takes ~140 ms, i.e.
// about 2 edges per 10 ms. Size must be a power of 2.
#define DRV_IR_RING_SIZE        (32U)
#define DRV_IR_RING_MASK        (DRV_IR_RING_SIZE - 1U)
#define DRV_IR_OVERFLOW_MAX     (255U)

_Static_assert(0U == (DRV_IR_RING_SIZE & DRV_IR_RING_MASK),
               "ring size must be a power of 2");

/*
 * ISR -> task data (Change Requirement 1: shared, so plain words with
 * masks, no bit-fields). One writer per variable:
 *   ISR writes g_ring[], g_ring_head, g_overflows, g_isr_black
 *   task writes g_ring_tail
 * Each slot is one aligned 32-bit word, written in one store.
 */
static uint32_t         g_ring[DRV_IR_RING_SIZE];
static volatile uint8_t g_ring_head = 0U;   // next slot the ISR fills
static volatile uint8_t g_ring_tail = 0U;   // next slot the task reads
static volatile uint8_t g_overflows = 0U;   // saturating drop counter
static volatile uint8_t g_isr_black = 0U;   // last level the ISR queued

static bool drv_ir_is_black(uint32_t pin);
static void drv_ir_setup_pin(uint32_t pin);
static void drv_ir_isr(UINT intno);

void
drv_ir_init (void)
{
    drv_ir_setup_pin(DRV_IR_PIN_CENTRE);
    drv_ir_setup_pin(DRV_IR_PIN_RIGHT);
    drv_ir_setup_pin(DRV_IR_PIN_BARCODE);

    g_ring_head = 0U;
    g_ring_tail = 0U;
    g_overflows = 0U;
}

ER
drv_ir_barcode_start (void)
{
    T_DINT   dint;
    ER       err;
    uint32_t inte_base;

    g_ring_head = 0U;
    g_ring_tail = 0U;
    g_overflows = 0U;
    g_isr_black = drv_ir_is_black(DRV_IR_PIN_BARCODE) ? 1U : 0U;

    // Clear stale edge flags, then install the handler
    out_w(DRV_IR_IO_REG(DRV_IR_IO_INTR0, DRV_IR_PIN_BARCODE),
          DRV_IR_EDGE_BOTH(DRV_IR_PIN_BARCODE));

    dint.intatr = TA_HLNG;
    dint.inthdr = (FP)drv_ir_isr;
    err = tk_def_int(DRV_IR_INTNO_IO_BANK0, &dint);

    if (E_OK == err)
    {
        // Enable both edges on THIS core's IO_IRQ_BANK0 only, so the
        // handler always runs on one core.
        if (0U == in_w(DRV_IR_CPUID))
        {
            inte_base = DRV_IR_IO_PROC0_INTE0;
        }
        else
        {
            inte_base = DRV_IR_IO_PROC1_INTE0;
        }

        set_w(DRV_IR_IO_REG(inte_base, DRV_IR_PIN_BARCODE),
              DRV_IR_EDGE_BOTH(DRV_IR_PIN_BARCODE));
        EnableInt(DRV_IR_INTNO_IO_BANK0, DRV_IR_INT_LEVEL);
    }
    else
    {
        // handler not installed: leave the interrupt disabled
    }

    return err;
}

bool
drv_ir_centre_black (void)
{
    return drv_ir_is_black(DRV_IR_PIN_CENTRE);
}

bool
drv_ir_right_black (void)
{
    return drv_ir_is_black(DRV_IR_PIN_RIGHT);
}

bool
drv_ir_barcode_black (void)
{
    return drv_ir_is_black(DRV_IR_PIN_BARCODE);
}

bool
drv_ir_pop_edge (uint32_t *p_edge)
{
    bool    b_got = false;
    uint8_t tail;
    uint8_t head;

    if (NULL != p_edge)
    {
        tail = g_ring_tail;
        // Acquire: the slot contents written before the ISR published
        // head are visible once head is read
        head = __atomic_load_n(&g_ring_head, __ATOMIC_ACQUIRE);

        if (head != tail)
        {
            *p_edge = g_ring[tail];
            // Release: the slot is read before the ISR may reuse it.
            // Cast: the mask keeps the index in 0..31
            __atomic_store_n(&g_ring_tail,
                             (uint8_t)((tail + 1U) & DRV_IR_RING_MASK),
                             __ATOMIC_RELEASE);
            b_got = true;
        }
        else
        {
            // ring empty
        }
    }
    else
    {
        // nowhere to store the edge
    }

    return b_got;
}

uint8_t
drv_ir_overflows (void)
{
    return g_overflows;
}

uint32_t
drv_ir_now_us (void)
{
    return in_w(TIMER_TIMERAWL);
}

static bool
drv_ir_is_black (uint32_t pin)
{
    uint32_t level = (in_w(GPIO_IN) >> pin) & 1U;

    return (DRV_IR_BLACK_LEVEL == level);
}

static void
drv_ir_setup_pin (uint32_t pin)
{
    // Cast: pin is 6, 7 or 26, well inside UINT
    (void)gpio_set_pin((UINT)pin, GPIO_MODE_IN);
    out_w(GPIO(pin), DRV_IR_PAD_CFG);
}

/*!
 * IO_IRQ_BANK0 handler: timestamp one barcode-sensor change.
 * Minimal work, no kernel calls, no blocking (TEAM.md section 3).
 */
static void
drv_ir_isr (UINT intno)
{
    uint32_t now_us;
    uint8_t  black;
    uint8_t  head;
    uint8_t  next;

    (void)intno;

    // Clear first, so a change after the level read raises a new IRQ
    out_w(DRV_IR_IO_REG(DRV_IR_IO_INTR0, DRV_IR_PIN_BARCODE),
          DRV_IR_EDGE_BOTH(DRV_IR_PIN_BARCODE));

    now_us = in_w(TIMER_TIMERAWL);
    black  = drv_ir_is_black(DRV_IR_PIN_BARCODE) ? 1U : 0U;

    // Two edges closer than the IRQ latency read back the old level:
    // a spike shorter than a few microseconds, so drop it.
    if (black != g_isr_black)
    {
        g_isr_black = black;
        head = g_ring_head;
        // Cast: the mask keeps the index in 0..31
        next = (uint8_t)((head + 1U) & DRV_IR_RING_MASK);

        if (next == __atomic_load_n(&g_ring_tail, __ATOMIC_ACQUIRE))
        {
            if (g_overflows < DRV_IR_OVERFLOW_MAX)
            {
                g_overflows++;
            }
            else
            {
                // saturated
            }
        }
        else
        {
            g_ring[head] = (now_us & DRV_IR_EDGE_TIME_MASK)
                           | ((0U != black) ? DRV_IR_EDGE_BLACK_BIT : 0U);
            // Release: the slot is written before head moves past it
            __atomic_store_n(&g_ring_head, next, __ATOMIC_RELEASE);
        }
    }
    else
    {
        // same level as the last queued edge: spike, ignore
    }
}
