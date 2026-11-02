/*
 * drv_ir.c
 * Subsystem 3 (Buddy 3): IR sensor inputs and barcode edge interrupt,
 * see drv_ir.h.
 *
 * The barcode edges come through the team's shared GPIO interrupt
 * dispatcher (app_program/gpio_irq.c, TEAM.md section 3): it owns
 * IO_IRQ_BANK0 (interrupt 13), which the S2 wheel encoders use too. A
 * second tk_def_int(13) would replace the dispatcher and silence the
 * encoders, so this module only attaches a callback for its pin.
 * (Until 2026-11-02 it installed its own handler; ported on the
 * integration branch by Buddy 4, Buddy 3 to review.)
 * The Pico SDK's IRQ registration functions are empty stubs in this
 * kernel build (lib/libwifi/sysdepend/pico_rp2040/cyw43_utk_compat.c),
 * so they cannot be used either.
 */

#include "drv_ir.h"
#include "../gpio_irq.h"

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
static void drv_ir_barcode_edge(uint32_t pin, uint32_t events);

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
    g_ring_head = 0U;
    g_ring_tail = 0U;
    g_overflows = 0U;
    g_isr_black = drv_ir_is_black(DRV_IR_PIN_BARCODE) ? 1U : 0U;

    // Both edges of the barcode pin to drv_ir_barcode_edge(), run by the
    // shared dispatcher on core 0 (stale edge flags are cleared there)
    return gpio_irq_attach(DRV_IR_PIN_BARCODE, GPIO_IRQ_EDGE_BOTH,
                           drv_ir_barcode_edge);
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
 * Dispatcher callback (interrupt context, core 0): timestamp one
 * barcode-sensor change. The dispatcher has already cleared the edge
 * flag, so a change after the level read raises a new interrupt.
 * Minimal work, no kernel calls, no blocking (TEAM.md section 3).
 */
static void
drv_ir_barcode_edge (uint32_t pin, uint32_t events)
{
    uint32_t now_us;
    uint8_t  black;
    uint8_t  head;
    uint8_t  next;

    (void)pin;
    (void)events;   // the level read below is what counts

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
