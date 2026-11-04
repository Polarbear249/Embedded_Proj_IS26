#ifndef MOTION_CMD_H
#define MOTION_CMD_H

/*
 * Buddy 2 - IF-01 Q_motion command word. Pure helpers, also built by the
 * PC tests. Explicit shifts and masks, never C bit-fields.
 *
 *   [31:29] op 3 | [28:18] left 11 | [17:7] right 11 | [6:3] seq 4 | [2:0] 0
 *
 * op                  left                      right
 * MOTION_OP_STOP      -                         -
 * MOTION_OP_DUTY      duty -200..200            duty -200..200
 * MOTION_OP_SPEED     wheel mm/s, -103..103     wheel mm/s, -103..103
 * MOTION_OP_DISTANCE  mm, negative = backward   -
 * MOTION_OP_TURN      deg, negative = left      -
 *
 * The receiver rejects out-of-range values instead of clamping them.
 * DUTY and SPEED carry a MOTION_Q_LEASE_MS lease, so the motors stop if
 * the sender goes silent (TEAM.md section 5, command timeout).
 */

#include <stdint.h>

#define MOTION_OP_STOP      (0U)
#define MOTION_OP_DUTY      (1U)
#define MOTION_OP_SPEED     (2U)
#define MOTION_OP_DISTANCE  (3U)
#define MOTION_OP_TURN      (4U)

#define MOTION_CMD_VALUE_MAX    (1023)  /* 11-bit signed field */
#define MOTION_CMD_OP_SHIFT     (29U)
#define MOTION_CMD_LEFT_SHIFT   (18U)
#define MOTION_CMD_RIGHT_SHIFT  (7U)
#define MOTION_CMD_SEQ_SHIFT    (3U)
#define MOTION_CMD_OP_MASK      (0x7U)
#define MOTION_CMD_VALUE_MASK   (0x7FFU)
#define MOTION_CMD_VALUE_SIGN   (0x400U)
#define MOTION_CMD_SEQ_MASK     (0xFU)

static inline uint32_t motion_cmd_pack(uint32_t op, int32_t left,
                                       int32_t right, uint32_t seq)
{
    return ((op & MOTION_CMD_OP_MASK) << MOTION_CMD_OP_SHIFT) |
           (((uint32_t)left & MOTION_CMD_VALUE_MASK) << MOTION_CMD_LEFT_SHIFT) |
           (((uint32_t)right & MOTION_CMD_VALUE_MASK) << MOTION_CMD_RIGHT_SHIFT) |
           ((seq & MOTION_CMD_SEQ_MASK) << MOTION_CMD_SEQ_SHIFT);
}

/* Sign-extend one 11-bit field. */
static inline int32_t motion_cmd_value(uint32_t field)
{
    uint32_t value = field & MOTION_CMD_VALUE_MASK;

    return (int32_t)(value ^ MOTION_CMD_VALUE_SIGN) -
           (int32_t)MOTION_CMD_VALUE_SIGN;
}

static inline uint32_t motion_cmd_op(uint32_t word)
{
    return word >> MOTION_CMD_OP_SHIFT;
}

static inline int32_t motion_cmd_left(uint32_t word)
{
    return motion_cmd_value(word >> MOTION_CMD_LEFT_SHIFT);
}

static inline int32_t motion_cmd_right(uint32_t word)
{
    return motion_cmd_value(word >> MOTION_CMD_RIGHT_SHIFT);
}

static inline uint32_t motion_cmd_seq(uint32_t word)
{
    return (word >> MOTION_CMD_SEQ_SHIFT) & MOTION_CMD_SEQ_MASK;
}

#endif /* MOTION_CMD_H */
