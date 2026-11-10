/*
 * ultrasonic.c
 * Subsystem 5 (Buddy 5): HC-SR04+ ranging, see ultrasonic.h.
 *
 * Shared data: g_echo_state, g_rise_us, g_fall_us are written by the
 * ECHO callback (interrupt context, core 0: gpio_irq.h) and read by the
 * obstacle task, which is pinned to core 0 as well (subsystem5.c), so
 * the interrupt simply preempts the task and volatile word access is
 * enough (no cross-core ordering involved). The task writes only
 * g_echo_state, and only while no pulse is expected (before TRIG).
 * Worst-case blocking: none; ultrasonic_measure_cm() sleeps one cycle.
 */

#include "ultrasonic.h"
#include <sys/machine.h>
#include "../gpio_irq.h"

#define ULTRA_ARMED             (0U)    // waiting for the rising edge
#define ULTRA_ROSE              (1U)    // rising edge seen
#define ULTRA_DONE              (2U)    // both edges seen
#define ULTRA_IDLE              (3U)    // not measuring: ignore edges

static volatile uint32_t g_echo_state = ULTRA_IDLE;
static volatile uint32_t g_rise_us = 0U;
static volatile uint32_t g_fall_us = 0U;

static void ultrasonic_echo_isr(uint32_t pin, uint32_t events);

ER
ultrasonic_init (void)
{
    ER er = gpio_set_pin(ULTRA_TRIG_PIN, GPIO_MODE_OUT);

    if (E_OK <= er)
    {
        gpio_set_val(ULTRA_TRIG_PIN, 0);
        er = gpio_set_pin(ULTRA_ECHO_PIN, GPIO_MODE_IN);
    }

    if (E_OK <= er)
    {
        // Unplugged sensor must read low, not float
        // Fakes edges and defeats the 5-ping fault detection, but the VC will stop the car anyway
        set_w(GPIO(ULTRA_ECHO_PIN), GPIO_PDE);
        er = gpio_irq_attach(ULTRA_ECHO_PIN, GPIO_IRQ_EDGE_BOTH, ultrasonic_echo_isr);
    }

    return (er);
}

/*!
 * One measurement: TRIG pulse, then sleep one cycle (60-70 ms at the
 * 10 ms tick) while the callback timestamps ECHO. Task context only.
 */
uint32_t
ultrasonic_measure_cm (void)
{
    uint32_t t0_us;
    uint32_t width_us;
    uint32_t state;
    uint32_t dist_cm = ULTRA_NO_ECHO;

    g_echo_state = ULTRA_ARMED;

    t0_us = in_w(TIMER_TIMERAWL);
    gpio_set_val(ULTRA_TRIG_PIN, 1);

    while ((in_w(TIMER_TIMERAWL) - t0_us) < ULTRA_TRIG_US)
    {
        // 10 us: the only busy wait
    }

    gpio_set_val(ULTRA_TRIG_PIN, 0);
    (void)tk_dly_tsk(ULTRA_CYCLE_MS);

    state = g_echo_state;
    g_echo_state = ULTRA_IDLE;

    if (ULTRA_DONE == state)
    {
        width_us = g_fall_us - g_rise_us;       // wraps correctly
        dist_cm = width_us / ULTRA_US_PER_CM;
        dist_cm = (dist_cm > ULTRA_MAX_CM) ? ULTRA_MAX_CM : dist_cm;
        dist_cm = (0U == dist_cm) ? 1U : dist_cm;   // 0 means no echo
    }
    else
    {
        // ARMED: no rising edge (sensor missing or dead). ROSE: ECHO
        // still high a full cycle later. Both: no valid reading.
        dist_cm = ULTRA_NO_ECHO;
    }

    return (dist_cm);
}

/*
 * ECHO edge (interrupt context): store the time, nothing else.
 */
static void
ultrasonic_echo_isr (uint32_t pin, uint32_t events)
{
    uint32_t now_us = in_w(TIMER_TIMERAWL);

    (void)pin;

    if ((ULTRA_ARMED == g_echo_state)
        && (0U != (events & GPIO_IRQ_EDGE_RISE)))
    {
        g_rise_us = now_us;
        g_echo_state = ULTRA_ROSE;
    }

    if ((ULTRA_ROSE == g_echo_state)
        && (0U != (events & GPIO_IRQ_EDGE_FALL)))
    {
        g_fall_us = now_us;
        g_echo_state = ULTRA_DONE;
    }
}
