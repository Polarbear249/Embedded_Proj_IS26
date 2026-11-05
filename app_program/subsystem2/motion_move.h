#ifndef MOTION_MOVE_H
#define MOTION_MOVE_H

/*
 * Buddy 2 - Encoder distance moves and turns: the pure control law.
 *
 * No kernel calls and no floats, so testing/subsystem2/host can run it on
 * a PC. motion_control.c owns the motors and calls move_step() each tick.
 *
 * All duties and MOVE_COAST_COUNTS are STARTING VALUES, not measured:
 * tune them on the car and record the results in
 * testing/subsystem2/TEST_LOG.md.
 */

#include <stdbool.h>
#include <stdint.h>

#define MOVE_WHEEL_CIRC_UM      (207345)    /* pi x 66 mm, as encoder.c */

/* Rear tyre centre to centre: ruler, 2026-11-03 (IMU_HUMP_TRACK_MM). */
#define MOVE_TRACK_MM           (113)
/* Spin-on-the-spot correction for tyre scrub. Raise it if a 90 degree
   command turns less than 90 degrees on the floor, lower it if it turns
   more. Tuned by eye on the floor, 2026-11-04: 99 turned about 5 degrees
   too little. */
#define MOVE_TURN_SCALE_PCT     (105)
/* Straight-move correction. Ruler on the floor, 2026-11-05: at 93 a 30 cm
   move went about 28 cm, at 100 about 29 cm. */
#define MOVE_DIST_SCALE_PCT     (100)
/* Left wheel's share of a straight move. Lower it if the car veers right,
   raise it if it veers left. Ruler on the floor, 2026-11-05: 98 veered about
   5 cm left over 28 cm, 99 about 0.5 cm left (USB cable also held up). */
#define MOVE_STRAIGHT_LEFT_PCT  (99)

#define MOVE_SLOW_COUNTS        (150)   /* creep over the last ~24 mm */
#define MOVE_COAST_COUNTS       (20)    /* cut power this early, ~3 mm */
#define MOVE_KICK_MS            (200U)  /* start kick, as speed mode */
#define MOVE_BAL_MAX            (40)    /* most duty taken off the leader */
#define MOVE_STALL_MS           (1000U) /* as the speed-mode watchdog */
#define MOVE_WRONG_COUNTS       (200)   /* ~33 mm the wrong way = fault */

typedef enum
{
    MOVE_RUNNING = 0,
    MOVE_DONE,
    MOVE_STALLED,       /* a wheel gave no encoder count for MOVE_STALL_MS */
    MOVE_WRONG_WAY      /* a wheel ran against its command: encoder fault */
} move_result_t;

typedef struct
{
    int16_t kick;
    int16_t cruise;
    int16_t creep;
} move_duty_t;

typedef struct
{
    int32_t     goal_left;      /* signed counts, forward positive */
    int32_t     goal_right;
    int32_t     last_left;
    int32_t     last_right;
    uint32_t    idle_left_ms;
    uint32_t    idle_right_ms;
    uint32_t    kick_left_ms;
    move_duty_t duty;
} move_t;

/* Wheel counts for a straight travel of mm (mm >= 0). */
int32_t move_mm_to_counts(int32_t mm, int32_t cpr);

/* Counts EACH wheel travels for a spin of deg on the spot (deg >= 0). */
int32_t move_deg_to_counts(int32_t deg, int32_t cpr);

/* Start a move. Goals are counted from zero. */
void move_start(move_t *p_move, int32_t goal_left, int32_t goal_right,
                move_duty_t duty);

/*
 * One step. left / right: counts travelled since the move started.
 * dt_ms: time since the previous step. Always writes both duties, 0 once
 * a wheel has reached its goal.
 */
move_result_t move_step(move_t *p_move, int32_t left, int32_t right,
                        uint32_t dt_ms, int16_t *p_left, int16_t *p_right);

#endif /* MOTION_MOVE_H */
