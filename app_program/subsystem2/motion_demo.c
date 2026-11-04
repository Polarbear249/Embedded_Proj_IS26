/*
 * motion_demo.c
 * Buddy 2 - bench and floor try-out of the encoder moves and turns, for
 * tuning motion_move.h. S2_MOVE_DEMO in subsystem2.h turns it on (needs
 * S3_DRIVE_SWITCH 0: one commander at a time). Runs once, S2_DEMO_DELAY_MS
 * after boot; reset the board to run it again. GP20 still stops the car.
 */

#include "subsystem2.h"
#include "../subsystem3/subsystem3.h"   /* S3_DRIVE_ENABLE */

#if S2_MOVE_DEMO && !APP_TEST

#if S3_DRIVE_ENABLE
#error "S2_MOVE_DEMO needs S3_DRIVE_SWITCH 0: two tasks would command the motors"
#endif

#define S2_DEMO_DELAY_MS    (5000)
#define S2_DEMO_PAUSE_MS    (1500)
#define S2_DEMO_POLL_MS     (50)
#define S2_DEMO_LIMIT_MS    (15000)
#define S2_DEMO_DIST_CM     (30.0f)
#define S2_DEMO_TURN_DEG    (90.0f)

/* Waits for the move to end, stops it if it never does, prints the result. */
static void s2_demo_finish(char const *p_name, motion_result_t accepted)
{
    motion_status_t before;
    motion_status_t now;
    INT             elapsed_ms = 0;

    (void)motion_get_status(&before);
    now = before;

    while ((MOTION_OK == accepted) && (MOTION_STATE_RUNNING == now.state) &&
           (elapsed_ms < S2_DEMO_LIMIT_MS))
    {
        (void)tk_dly_tsk(S2_DEMO_POLL_MS);
        elapsed_ms += S2_DEMO_POLL_MS;
        (void)motion_get_status(&now);
    }

    if (S2_DEMO_LIMIT_MS <= elapsed_ms)
    {
        (void)motion_stop();
    }

    tm_printf((UB*)"[S2 demo] %s: accepted %d, state %d, faults 0x%x, "
              "%d ms, L %d R %d counts, odometer %d mm\n",
              (UB*)p_name, (INT)accepted, (INT)now.state,
              (UINT)motion_get_fault_bits(), elapsed_ms,
              (INT)now.left_encoder_count, (INT)now.right_encoder_count,
              (INT)motion_get_distance_mm());

    (void)tk_dly_tsk(S2_DEMO_PAUSE_MS);
}

void motion_demo_task(INT stacd, void *exinf)
{
    (void)stacd;
    (void)exinf;

    tm_printf((UB*)"[S2 demo] moves start in %d ms: clear the floor\n",
              S2_DEMO_DELAY_MS);
    (void)tk_dly_tsk(S2_DEMO_DELAY_MS);

    s2_demo_finish("forward 30 cm", motion_move_forward(S2_DEMO_DIST_CM));
    s2_demo_finish("backward 30 cm", motion_move_backward(S2_DEMO_DIST_CM));
    s2_demo_finish("left 90 deg", motion_turn_left(S2_DEMO_TURN_DEG));
    s2_demo_finish("right 90 deg", motion_turn_right(S2_DEMO_TURN_DEG));
    s2_demo_finish("U-turn", motion_u_turn());

    tm_printf((UB*)"[S2 demo] done: reset the board to run it again\n");

    (void)tk_exd_tsk();
}

#endif /* S2_MOVE_DEMO && !APP_TEST */
