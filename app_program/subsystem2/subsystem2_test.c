/*
 *	subsystem2_test.c
 *	On-target tests for subsystem 2.  Built only with: make TEST=1
 *	Record results in testing/subsystem2/TEST_LOG.md.
 *
 *	No test here moves a wheel: S3's drive is off in TEST=1 builds, and
 *	these check only rejection, the kernel objects and timing. The moves
 *	and turns themselves are the manual floor tests in TEST_LOG.md.
 *	S2-U03 is retired ("distance moves not implemented"): they now exist.
 */

#include "subsystem2.h"
#include "../test_report.h"

#if APP_TEST

#define S2_TEST_NAN         (0.0f / 0.0f)
#define S2_TEST_BAD_OP      (7U)
#define S2_TEST_SETTLE_MS   (200)   /* several motion ticks */
#define S2_TEST_TICK_MAX_US (5000U) /* half the 10 ms period */

EXPORT void subsystem2_test(void)
{
    motion_diag_t   before;
    motion_diag_t   after;
    motion_status_t status;
    T_RFLG          flag = {0U, 0U};

    TEST_INIT();

    TEST_CHECK("S2-U01", MOTION_ERROR == motion_get_status(NULL),
               "reject NULL status pointer");
    TEST_CHECK("S2-U02", MOTION_OK != motion_set_speed(-1.0f, 0.0f),
               "reject negative speed without moving wheels");

    TEST_CHECK("S2-U04", (MOTION_ERROR == motion_move_forward(-1.0f)) &&
               (MOTION_ERROR == motion_move_backward(0.0f)) &&
               (MOTION_ERROR == motion_move_forward(S2_TEST_NAN)) &&
               (MOTION_ERROR == motion_move_forward(1001.0f)),
               "reject bad distances");
    TEST_CHECK("S2-U05", (MOTION_ERROR == motion_turn_left(0.0f)) &&
               (MOTION_ERROR == motion_turn_right(361.0f)) &&
               (MOTION_ERROR == motion_turn_left(S2_TEST_NAN)),
               "reject bad angles");
    TEST_CHECK("S2-U06", (MOTION_ERROR == motion_line_drive(31.0f, 0.0f, 100U)) &&
               (MOTION_ERROR == motion_line_drive(10.0f, 10.0f, 0U)) &&
               (MOTION_ERROR == motion_line_drive(10.0f, 10.0f, 1001U)) &&
               (MOTION_ERROR == motion_line_drive(S2_TEST_NAN, 0.0f, 100U)),
               "line drive rejects bad speed and lease");

    /* A reserved op through Q_motion is refused and moves nothing. */
    motion_get_diag(&before);
    TEST_CHECK("S2-U07", E_OK == motion_send(
                   motion_cmd_pack(S2_TEST_BAD_OP, 0, 0, 1U)),
               "Q_motion accepts a word");
    (void)tk_dly_tsk(S2_TEST_SETTLE_MS);
    motion_get_diag(&after);
    TEST_CHECK("S2-U08", after.queue_rejected == (before.queue_rejected + 1U),
               "motion task reads Q_motion and refuses a reserved op");

    /* EVG_stop latches, refuses movement, confirms still, then releases. */
    TEST_CHECK("S2-U09", E_OK == motion_request_stop(),
               "EVG_stop can be set");
    (void)tk_dly_tsk(S2_TEST_SETTLE_MS);
    TEST_CHECK("S2-U10", motion_is_estopped(),
               "motion task latches on EVG_stop");
    TEST_CHECK("S2-U11", MOTION_ERROR == motion_move_forward(1.0f),
               "EVG_stop refuses a move");
    (void)tk_ref_flg(EVG_stop, &flag);
    TEST_CHECK("S2-U12", 0U != (flag.flgptn & EVG_STOPPED),
               "EVG_STOPPED confirms the wheels are still");
    TEST_CHECK("S2-U13", MOTION_OK == motion_clear_estop(),
               "stop releases");
    (void)tk_dly_tsk(S2_TEST_SETTLE_MS);
    TEST_CHECK("S2-U14", (!motion_is_estopped()) &&
               (MOTION_OK == motion_get_status(&status)) &&
               (MOTION_STATE_STOPPED == status.state),
               "stays released and does not restart the motors");

    /* Timing evidence: compare with the 10 ms period (TEAM.md section 4). */
    motion_get_diag(&after);
    tm_printf((UB*)"[TEST] S2 motion_tick max exec %u us, max period %u us\n",
              (UINT)after.tick_exec_max_us, (UINT)after.tick_period_max_us);
    TEST_CHECK("S2-U15", (0U < after.tick_exec_max_us) &&
               (after.tick_exec_max_us < S2_TEST_TICK_MAX_US),
               "motion_tick worst-case time well under its period");

    TEST_SUMMARY("subsystem2");
}

#endif /* APP_TEST */
