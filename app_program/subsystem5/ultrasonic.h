/*
 * ultrasonic.h
 * Subsystem 5 (Buddy 5): HC-SR04+ ultrasonic ranging, TRIG GP16, ECHO
 * GP17 (TEAM.md section 2; the "+" variant runs at 3.3 V, so ECHO goes
 * straight to the pin).
 *
 * ECHO edges are timestamped by the shared GPIO dispatcher
 * (app_program/gpio_irq.c, team decision Tribunal 1): the callback only
 * stores the microsecond timer at the rising and the falling edge. The
 * caller sleeps through one measurement cycle (>= 60 ms, datasheet) and
 * then reads the pair, so no CPU time is burnt waiting for the echo
 * (the earlier version busy-waited up to about 35 ms at P8, which
 * starved the P9 IMU task on the same core).
 *
 * Distances are 9-bit codes (CLAUDE.md section 6, IF-09):
 *   ULTRA_NO_ECHO (0)  no rising edge, or ECHO stuck high: sensor fault
 *   1..ULTRA_MAX_CM    distance in cm (round trip us / 58)
 *   ULTRA_MAX_CM       also "nothing in range" (the + variant holds ECHO
 *                      high for about 38 ms when nothing returns)
 */

#ifndef S5_ULTRASONIC_H
#define S5_ULTRASONIC_H

#include <stdint.h>
#include "../app.h"

#define ULTRA_TRIG_PIN          (16U)
#define ULTRA_ECHO_PIN          (17U)
#define ULTRA_CYCLE_MS          (60)        // datasheet: >= 60 ms cycle
#define ULTRA_TRIG_US           (10U)       // datasheet: >= 10 us high
#define ULTRA_US_PER_CM         (58U)       // round trip, 340 m/s
#define ULTRA_MAX_CM            (400U)      // datasheet range 2..400 cm
#define ULTRA_NO_ECHO           (0U)

ER       ultrasonic_init(void);
uint32_t ultrasonic_measure_cm(void);

#endif // S5_ULTRASONIC_H
