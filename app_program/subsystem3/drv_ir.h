/*
 * drv_ir.h
 * Subsystem 3 (Buddy 3): three TCRT5000 IR sensor modules, DOUT only.
 *
 * Wiring (TEAM.md section 2, confirmed by the owner 2026-11-01):
 *   GP27  centre line sensor DOUT   (Grove 6, over the line; AOUT not
 *                                    connected, Grove 6's GP26 wire
 *                                    unused)
 *   GP6   right line sensor DOUT    (Grove 5, just right of the line;
 *                                    Grove 5's GP26 wire NOT connected)
 *   GP7   barcode sensor DOUT       (Grove 7; left side of the car;
 *                                    AOUT not connected: Grove 7's
 *                                    second pin is GP28 = VBAT sense)
 * DOUT is HIGH over black (measured by the owner on all three modules).
 * No ADC is used.
 *
 * The barcode sensor's edges are timestamped by the GPIO interrupt
 * (IO_IRQ_BANK0) and queued in a lock-free ring, because a 3 mm bar
 * passes in ~3 ms at 1 m/s: too fast for 10 ms polling.
 *
 * Call drv_ir_init() and then drv_ir_barcode_start() from the Line
 * task (pinned to core 0): the edge callback is attached to the shared
 * GPIO dispatcher (app_program/gpio_irq.c), which runs on core 0. Every other
 * function is for the Line task only (TEAM.md section 3: one owning
 * task per peripheral). Without drv_ir_barcode_start() the line inputs
 * still work and the ring stays empty.
 */

#ifndef DRV_IR_H
#define DRV_IR_H

#include <stdbool.h>
#include <stdint.h>
#include "../app.h"

#define DRV_IR_PIN_CENTRE       (27U)
#define DRV_IR_PIN_RIGHT        (6U)
#define DRV_IR_PIN_BARCODE      (7U)

// DOUT level that means black (owner's measurement: HIGH over black)
#define DRV_IR_BLACK_LEVEL      (1U)

// Ring entry: bit 31 = level (1 = black), bits 30..0 = time in us
#define DRV_IR_EDGE_BLACK_BIT   (0x80000000U)
#define DRV_IR_EDGE_TIME_MASK   (0x7FFFFFFFU)

/*!
 * Configure the three inputs (no interrupt yet).
 */
void     drv_ir_init(void);

/*!
 * Attach the barcode edge callback to the shared GPIO dispatcher.
 * Returns E_OK or the error from gpio_irq_attach() (E_OBJ if called
 * twice, or a kernel error from tk_def_int() on the first attach).
 */
ER       drv_ir_barcode_start(void);

bool     drv_ir_centre_black(void);
bool     drv_ir_right_black(void);
bool     drv_ir_barcode_black(void);

/*!
 * Take the oldest barcode edge from the ring. Returns false when the
 * ring is empty.
 */
bool     drv_ir_pop_edge(uint32_t *p_edge);

/*!
 * Number of edges the interrupt had to drop because the ring was full
 * (saturates at 255).
 */
uint8_t  drv_ir_overflows(void);

/*!
 * Free-running 1 MHz hardware timer (RP2040 TIMERAWL), wraps every
 * 71 minutes.
 */
uint32_t drv_ir_now_us(void);

#endif /* DRV_IR_H */
