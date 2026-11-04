/*
 * Host test for the encoder move law (motion_move.c) and the IF-01 command
 * word (motion_cmd.h). A simple wheel model turns duty into counts, so a
 * whole move runs from start to finish. It shows the law is correct, not
 * that the car turns accurately: that needs the floor tests in TEST_LOG.md.
 * Build and run: ./run_tests.sh
 */
#include <assert.h>
#include <stdint.h>
#include <stdio.h>

#include "../../../app_program/subsystem2/motion_cmd.h"
#include "../../../app_program/subsystem2/motion_move.c"

#define CPR_L   1271
#define CPR_R   1266
#define DT_MS   10U
#define STEPS   20000

static const move_duty_t DUTY = {300, 200, 140};

static int32_t iabs(int32_t v)
{
    return (v < 0) ? -v : v;
}

/* Runs one move on the wheel model. Counts per step = duty / divisor. */
static move_result_t run(int32_t goal_l, int32_t goal_r, int32_t div_l,
                         int32_t div_r, int32_t *p_l, int32_t *p_r)
{
    move_t m;
    int32_t l = 0;
    int32_t r = 0;
    int16_t dl = 0;
    int16_t dr = 0;
    int i;

    move_start(&m, goal_l, goal_r, DUTY);

    for (i = 0; i < STEPS; i++)
    {
        move_result_t res = move_step(&m, l, r, DT_MS, &dl, &dr);

        if (res != MOVE_RUNNING)
        {
            assert((dl == 0) && (dr == 0));
            *p_l = l;
            *p_r = r;
            return res;
        }

        l += (div_l != 0) ? (dl / div_l) : 0;
        r += (div_r != 0) ? (dr / div_r) : 0;
    }

    assert(0 && "move never ended");
    return MOVE_RUNNING;
}

static void test_conversions(void)
{
    /* One wheel turn is one CPR. */
    assert(move_mm_to_counts(207, CPR_L) == (207000LL * CPR_L) / 207345);
    assert(iabs(move_mm_to_counts(1000, CPR_L) - 6130) <= 1);

    /* 180 deg: wheel arc = pi x 113 / 2 = 177.5 mm, about 1088 counts,
       before the turn scale. */
    assert(iabs(move_deg_to_counts(180, CPR_L) -
                (1088 * MOVE_TURN_SCALE_PCT) / 100) <= 2);
    /* A U-turn is twice a 90 degree turn, within rounding. */
    assert(iabs(move_deg_to_counts(180, CPR_L) -
                2 * move_deg_to_counts(90, CPR_L)) <= 1);
    puts("PASS conversions");
}

static void test_straight_reaches_goal(void)
{
    int32_t goal = move_mm_to_counts(500, CPR_L);
    int32_t l;
    int32_t r;

    assert(run(goal, goal, 50, 50, &l, &r) == MOVE_DONE);
    assert((l >= goal - MOVE_COAST_COUNTS) && (l <= goal + 10));
    assert((r >= goal - MOVE_COAST_COUNTS) && (r <= goal + 10));
    puts("PASS straight move stops at its goal");
}

static void test_spin_signs(void)
{
    int32_t goal = move_deg_to_counts(90, CPR_L);
    int32_t l;
    int32_t r;
    move_t m;
    int16_t dl;
    int16_t dr;

    /* Left turn: left wheel back, right wheel forward. */
    move_start(&m, -goal, goal, DUTY);
    assert(move_step(&m, 0, 0, DT_MS, &dl, &dr) == MOVE_RUNNING);
    assert((dl < 0) && (dr > 0));

    assert(run(-goal, goal, 50, 50, &l, &r) == MOVE_DONE);
    assert((l <= -goal + MOVE_COAST_COUNTS) && (r >= goal - MOVE_COAST_COUNTS));

    /* Right turn is the mirror image. */
    move_start(&m, goal, -goal, DUTY);
    assert(move_step(&m, 0, 0, DT_MS, &dl, &dr) == MOVE_RUNNING);
    assert((dl > 0) && (dr < 0));
    puts("PASS turn wheel directions");
}

static void test_unequal_wheels_finish_level(void)
{
    int32_t goal = move_mm_to_counts(500, CPR_L);
    int32_t l;
    int32_t r;

    /* The right wheel is stronger; the trim must slow it. */
    assert(run(goal, goal, 50, 40, &l, &r) == MOVE_DONE);
    assert(iabs(l - r) <= 20);
    puts("PASS balance holds an unequal pair level");
}

static void test_unequal_goals_keep_their_ratio(void)
{
    move_t m;
    int32_t l = 0;
    int32_t r = 0;
    int16_t dl = 0;
    int16_t dr = 0;

    /* The left wheel's share is 90 percent: it must not run level. */
    move_start(&m, 1800, 2000, DUTY);

    while (r < 1500)
    {
        assert(move_step(&m, l, r, DT_MS, &dl, &dr) == MOVE_RUNNING);
        l += dl / 50;
        r += dr / 50;
    }

    assert(iabs(l - (r * 9) / 10) <= 20);
    puts("PASS unequal goals keep their ratio during the move");
}

static void test_duty_profile(void)
{
    move_t m;
    int16_t dl;
    int16_t dr;
    int32_t goal = 1000;

    move_start(&m, goal, goal, DUTY);
    assert(move_step(&m, 0, 0, DT_MS, &dl, &dr) == MOVE_RUNNING);
    assert((dl == DUTY.kick) && (dr == DUTY.kick));

    /* Kick ends after MOVE_KICK_MS. */
    assert(move_step(&m, 1, 1, MOVE_KICK_MS, &dl, &dr) == MOVE_RUNNING);
    assert(move_step(&m, 2, 2, DT_MS, &dl, &dr) == MOVE_RUNNING);
    assert((dl == DUTY.cruise) && (dr == DUTY.cruise));

    /* Creep near the goal. */
    assert(move_step(&m, goal - MOVE_SLOW_COUNTS + 1, goal - MOVE_SLOW_COUNTS + 1,
                     DT_MS, &dl, &dr) == MOVE_RUNNING);
    assert((dl == DUTY.creep) && (dr == DUTY.creep));
    puts("PASS kick, cruise and creep");
}

static void test_stall_fault(void)
{
    int32_t goal = move_mm_to_counts(500, CPR_L);
    int32_t l;
    int32_t r;

    /* Right wheel never moves. */
    assert(run(goal, goal, 50, 0, &l, &r) == MOVE_STALLED);
    assert(r == 0);
    puts("PASS stalled wheel ends the move as a fault");
}

static void test_wrong_way_fault(void)
{
    move_t m;
    int16_t dl;
    int16_t dr;

    move_start(&m, 1000, 1000, DUTY);
    assert(move_step(&m, -MOVE_WRONG_COUNTS, 10, DT_MS, &dl, &dr)
           == MOVE_RUNNING);
    assert(move_step(&m, -MOVE_WRONG_COUNTS - 1, 20, DT_MS, &dl, &dr)
           == MOVE_WRONG_WAY);
    assert((dl == 0) && (dr == 0));
    puts("PASS wheel running against its command is an encoder fault");
}

static void test_finished_wheel_is_not_a_stall(void)
{
    move_t m;
    int16_t dl = 1;
    int16_t dr = 1;
    int i;

    move_start(&m, 100, 1000, DUTY);

    /* Left is at its goal and idle for over MOVE_STALL_MS; right moves. */
    for (i = 1; i <= 200; i++)
    {
        assert(move_step(&m, 100, i, DT_MS, &dl, &dr) == MOVE_RUNNING);
        assert(dl == 0);
        assert(dr > 0);
    }
    puts("PASS a finished wheel does not count as stalled");
}

static void test_cmd_word(void)
{
    static const int32_t values[] = {-1024, -1023, -200, -1, 0, 1, 200, 1023};
    size_t i;

    for (i = 0U; i < (sizeof(values) / sizeof(values[0])); i++)
    {
        int32_t  right = values[(sizeof(values) / sizeof(values[0])) - 1U - i];
        uint32_t word = motion_cmd_pack(MOTION_OP_TURN, values[i], right, 9U);

        assert(motion_cmd_op(word) == MOTION_OP_TURN);
        assert(motion_cmd_left(word) == values[i]);
        assert(motion_cmd_right(word) == right);
        assert(motion_cmd_seq(word) == 9U);
        assert((word & 0x7U) == 0U);
    }

    /* Highest op, and a fixed pattern pinning the bit layout. */
    assert(motion_cmd_op(motion_cmd_pack(7U, 0, 0, 0U)) == 7U);
    assert(motion_cmd_pack(MOTION_OP_DUTY, 1, -1, 1U) ==
           ((1U << 29) | (1U << 18) | (0x7FFU << 7) | (1U << 3)));
    puts("PASS Q_motion command word round trip");
}

int main(void)
{
    test_conversions();
    test_straight_reaches_goal();
    test_spin_signs();
    test_unequal_wheels_finish_level();
    test_unequal_goals_keep_their_ratio();
    test_duty_profile();
    test_stall_fault();
    test_wrong_way_fault();
    test_finished_wheel_is_not_a_stall();
    test_cmd_word();
    puts("test_motion_move: all passed");
    return 0;
}
