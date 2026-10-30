
#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>
#include <tk/tkernel.h>

/*
 * Buddy 2 - Wheel Encoder Interface
 *
 * Left encoder:  GP4 (A), GP5 (B)
 * Right encoder: GP0 (A), GP1 (B)
 *
 * Counts per revolution are preliminary calibration
 * values obtained from standalone hardware tests.
 */

#define ENCODER_LEFT_A_PIN       4U
#define ENCODER_LEFT_B_PIN       5U

#define ENCODER_RIGHT_A_PIN      0U
#define ENCODER_RIGHT_B_PIN      1U

#define ENCODER_LEFT_CPR         1271.0f
#define ENCODER_RIGHT_CPR        1266.0f

#define ENCODER_WHEEL_DIAMETER_CM 6.6f

/* Initialise the encoder inputs and event detection. */
ER encoder_init(void);

/* Obtain a snapshot of both encoder counts. */
void encoder_get_counts(int32_t *left, int32_t *right);

/* Reset both encoder counts. */
void encoder_reset_counts(void);

/*
 * Calculate wheel RPM from the change in encoder counts.
 * interval_ms must be greater than zero.
 */
float encoder_calculate_left_rpm(int32_t delta, uint32_t interval_ms);
float encoder_calculate_right_rpm(int32_t delta, uint32_t interval_ms);

/* Convert encoder counts into wheel travel distance. */
float encoder_left_distance_cm(int32_t counts);
float encoder_right_distance_cm(int32_t counts);

/* Sample both encoder inputs and update counts. */
void encoder_sample(void);

/*
 * IF-10: cumulative distance travelled, in mm.
 *
 * Mean of the left and right wheel travel since power-up.
 * Forward adds, reverse subtracts. Not cleared by
 * encoder_reset_counts(), so readers can take differences.
 *
 * encoder_publish_distance(): called only by the encoder
 * sampling task (single writer).
 * encoder_get_distance_mm(): any task, any core. One
 * aligned 32-bit load, so no lock is needed.
 */
void    encoder_publish_distance(void);
int32_t encoder_get_distance_mm(void);
#endif /* ENCODER_H */
