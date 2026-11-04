/*
 * line_drive.h
 * Subsystem 3 (Buddy 3): turns the line position into left / right
 * wheel duties for Buddy 2's motion API (motion_set_duty).
 *
 * TEMPORARY stand-in for Subsystem 2's line follower (IF-06 says S2
 * steers). Buddy 2's code is NOT changed: this only calls the public
 * API in subsystem2/motion_control.h, so S2's motion task stays the only
 * owner of the motors. Switched by S3_DRIVE_ENABLE in subsystem3.h.
 *
 * Steering (proportional):
 *   e     = error - 64           -64..+63, > 0 = line to the right
 *   left  = BASE + e * KP        line right -> left wheel faster
 *   right = BASE - e * KP
 * both clamped to +-DRIVE_DUTY_MAX. While tracking the gain is small
 * (DRIVE_KP_NUM), so every correction is a gentle one (owner asked
 * 2026-11-01: no sharp turns while following). Only a search for a lost
 * line uses DRIVE_KP_SEARCH_NUM, which makes the inner wheel run
 * backwards, i.e. the car turns on the spot.
 *
 *   situation              error  wheels L/R (BASE 120)
 *   centre on the line       64   120 / 120
 *   centre + right           74   130 / 110
 *   right only               96   152 /  88
 *   guessed left             48   104 / 136
 *   search (lost 1.5 s)    1/127  spin on the spot
 *
 * Safety:
 *   - not ARMED until the centre sensor has seen black for
 *     DRIVE_ARM_TICKS in a row (put the car on the line, it starts
 *     2 s later)
 *   - FAILED or INIT -> stop and disarm (place it on the line again)
 *   - every command carries a short lease: if this task stops, Buddy
 *     2's motion task stops the motors by itself
 *
 * Barcode commands (cmd / cmd_seq) are NOT acted on yet.
 *
 * Pure C, no kernel calls: also built by the PC tests.
 */

#ifndef LINE_DRIVE_H
#define LINE_DRIVE_H

#include <stdbool.h>
#include <stdint.h>

// Duty units as motion_set_duty: 1000 = 100 %. Buddy 2 limits it to
// +-200 for bring-up (MOTION_MAX_DUTY, checked in task_line.c).
#define DRIVE_DUTY_MAX          (200)
#define DRIVE_BASE_DUTY         (120)   // straight-ahead speed TODO_TUNE
#define DRIVE_KP_NUM            (1)     // tracking gain        TODO_TUNE
#define DRIVE_KP_SEARCH_NUM     (3)     // search gain (spin)   TODO_TUNE
#define DRIVE_KP_DEN            (1)     // gain = NUM / DEN

#define DRIVE_ARM_TICKS         (200U)  // 2 s on the line before moving
#define DRIVE_LEASE_MS          (100U)  // motors stop 100 ms after the
                                        // last command

/*
 * Task-private (Line task only): bit-fields allowed (Change
 * Requirement 1). 200 ticks need 8 bits.
 */
typedef struct
{
    unsigned arm_count : 8;     // cycles the centre has been black
    unsigned b_armed   : 1;     // 1 = allowed to drive
} line_drive_t;

void line_drive_init(line_drive_t *p_ctx);

/*!
 * One 10 ms step. Inputs are this cycle's IF-06 fields. Returns true
 * with the wheel duties in *p_left / *p_right when the car may drive,
 * false when it must stop (duties are then 0).
 */
bool line_drive_step(line_drive_t *p_ctx, uint8_t error, uint8_t mode,
                     bool b_centre_black, int16_t *p_left,
                     int16_t *p_right);

#endif /* LINE_DRIVE_H */
