#include "motion_move.h"

/*
 * Buddy 2 - Encoder distance moves and turns, see motion_move.h.
 */

/* pi / 360 x 1000000: wheel arc in um = deg x track_mm x this / 1000. */
#define MOVE_ARC_UM_PER_DEG_MM  (8727)
#define MOVE_UM_PER_MM          (1000)
#define MOVE_PCT                (100)

static int32_t move_abs(int32_t value)
{
    return (value < 0) ? -value : value;
}

/* Progress toward the goal: negative = the wrong way. */
static int32_t move_progress(int32_t goal, int32_t now)
{
    return (goal < 0) ? -now : now;
}

static bool move_wheel_done(int32_t goal, int32_t now)
{
    return (move_abs(goal) - move_progress(goal, now)) <= MOVE_COAST_COUNTS;
}

/* Duty size for one wheel that has not reached its goal. */
static int32_t move_wheel_duty(move_t const *p_move, int32_t goal, int32_t now)
{
    int32_t remaining = move_abs(goal) - move_progress(goal, now);

    if (remaining <= MOVE_SLOW_COUNTS)
    {
        return p_move->duty.creep;
    }

    return (p_move->kick_left_ms > 0U) ? p_move->duty.kick
                                       : p_move->duty.cruise;
}

/* Take duty off the wheel that is ahead, never below creep. */
static int32_t move_trim(int32_t duty, int32_t lead, int32_t creep)
{
    int32_t cut = (lead > MOVE_BAL_MAX) ? MOVE_BAL_MAX : lead;
    int32_t trimmed = duty - ((cut > 0) ? cut : 0);

    return (trimmed < creep) ? creep : trimmed;
}

/* How far the left wheel is ahead of its share, in left wheel counts. The
   wheels keep the ratio of their goals, so unequal goals stay unequal. */
static int32_t move_left_lead(move_t const *p_move, int32_t left_prog,
                              int32_t right_prog)
{
    return left_prog - (int32_t)(((int64_t)right_prog *
                                  move_abs(p_move->goal_left)) /
                                 move_abs(p_move->goal_right));
}

static uint32_t move_idle(uint32_t idle_ms, bool b_done, bool b_moved,
                          uint32_t dt_ms)
{
    return (b_done || b_moved) ? 0U : (idle_ms + dt_ms);
}

int32_t move_mm_to_counts(int32_t mm, int32_t cpr)
{
    return (int32_t)(((int64_t)mm * MOVE_UM_PER_MM * cpr) /
                     MOVE_WHEEL_CIRC_UM);
}

int32_t move_deg_to_counts(int32_t deg, int32_t cpr)
{
    int64_t arc_um = ((int64_t)deg * MOVE_TRACK_MM * MOVE_ARC_UM_PER_DEG_MM *
                      MOVE_TURN_SCALE_PCT) / (MOVE_UM_PER_MM * MOVE_PCT);

    return (int32_t)((arc_um * cpr) / MOVE_WHEEL_CIRC_UM);
}

void move_start(move_t *p_move, int32_t goal_left, int32_t goal_right,
                move_duty_t duty)
{
    p_move->goal_left = goal_left;
    p_move->goal_right = goal_right;
    p_move->last_left = 0;
    p_move->last_right = 0;
    p_move->idle_left_ms = 0U;
    p_move->idle_right_ms = 0U;
    p_move->kick_left_ms = MOVE_KICK_MS;
    p_move->duty = duty;
}

move_result_t move_step(move_t *p_move, int32_t left, int32_t right,
                        uint32_t dt_ms, int16_t *p_left, int16_t *p_right)
{
    bool    b_left_done = move_wheel_done(p_move->goal_left, left);
    bool    b_right_done = move_wheel_done(p_move->goal_right, right);
    int32_t left_prog = move_progress(p_move->goal_left, left);
    int32_t right_prog = move_progress(p_move->goal_right, right);
    int32_t left_duty = 0;
    int32_t right_duty = 0;

    *p_left = 0;
    *p_right = 0;

    if ((left_prog < -MOVE_WRONG_COUNTS) || (right_prog < -MOVE_WRONG_COUNTS))
    {
        return MOVE_WRONG_WAY;
    }

    p_move->idle_left_ms = move_idle(p_move->idle_left_ms, b_left_done,
                                     left != p_move->last_left, dt_ms);
    p_move->idle_right_ms = move_idle(p_move->idle_right_ms, b_right_done,
                                      right != p_move->last_right, dt_ms);
    p_move->last_left = left;
    p_move->last_right = right;

    if ((p_move->idle_left_ms >= MOVE_STALL_MS) ||
        (p_move->idle_right_ms >= MOVE_STALL_MS))
    {
        return MOVE_STALLED;
    }

    if (b_left_done && b_right_done)
    {
        return MOVE_DONE;
    }

    if (!b_left_done)
    {
        left_duty = move_wheel_duty(p_move, p_move->goal_left, left);
    }

    if (!b_right_done)
    {
        right_duty = move_wheel_duty(p_move, p_move->goal_right, right);
    }

    /* Keep the wheels level once both are running at their own pace. Neither
       goal is 0 here: a wheel with a goal of 0 is already done. */
    if ((p_move->kick_left_ms == 0U) && !b_left_done && !b_right_done)
    {
        int32_t lead = move_left_lead(p_move, left_prog, right_prog);

        left_duty = move_trim(left_duty, lead, p_move->duty.creep);
        right_duty = move_trim(right_duty, -lead, p_move->duty.creep);
    }

    p_move->kick_left_ms = (p_move->kick_left_ms > dt_ms)
                           ? (p_move->kick_left_ms - dt_ms) : 0U;

    /* Casts: duties come from move_duty_t, so they fit int16_t. */
    *p_left = (int16_t)((p_move->goal_left < 0) ? -left_duty : left_duty);
    *p_right = (int16_t)((p_move->goal_right < 0) ? -right_duty : right_duty);

    return MOVE_RUNNING;
}
