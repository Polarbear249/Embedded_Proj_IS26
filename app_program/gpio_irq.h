/*
 * gpio_irq.h
 * Shared GPIO edge-interrupt dispatcher (one handler for the RP2040 GPIO
 * bank, IO_IRQ_BANK0). Shared file: change only by team agreement.
 * Users: S2 wheel encoders (GP0, GP4); planned: S5 HC-SR04 ECHO (GP17).
 *
 * Team decision (Tribunal 1, CLAUDE.md section 4): one dispatcher, minimal
 * ISR work, no interrupt disabling around it. Each pin gets one callback,
 * called in interrupt context: store a count or a timestamp and return.
 * Never block, wait, print or allocate in a callback.
 *
 * Attach ONLY from subsystemN_init(). Those run one after another in
 * usermain() on core 0, so the callback table needs no lock and the
 * interrupt is enabled in core 0's NVIC (each RP2040 core has its own):
 * every GPIO callback runs on core 0.
 */

#ifndef GPIO_IRQ_H
#define GPIO_IRQ_H

#include <stdint.h>
#include <tk/tkernel.h>

#define GPIO_IRQ_EDGE_FALL      (0x4U)      // IO_BANK0 INTR bit per pin
#define GPIO_IRQ_EDGE_RISE      (0x8U)
#define GPIO_IRQ_EDGE_BOTH      (GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE)

#define GPIO_IRQ_INTNO          (13U)       // IO_IRQ_BANK0 (RP2040 2.3.2)
#define GPIO_IRQ_PRI            (2)         // as the kernel's ADC driver

// events: the edge bits seen since the last call (both = two edges)
typedef void (*gpio_irq_cb_t)(uint32_t pin, uint32_t events);

ER gpio_irq_attach(uint32_t pin, uint32_t events, gpio_irq_cb_t p_cb);

#endif // GPIO_IRQ_H
