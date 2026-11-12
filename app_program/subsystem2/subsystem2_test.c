/*
 *	subsystem2_test.c
 *	On-target tests for subsystem 2.  Built only with: make TEST=1
 *	Record results in testing/subsystem2/TEST_LOG.md.
 *
 *	Integration placeholder: checks only input rejection, so no wheel
 *	moves.  Buddy 2 to replace with the real motion tests.
 */

#include "subsystem2.h"
#include "../test_report.h"

#if APP_TEST

EXPORT void subsystem2_test(void)
{
    TEST_INIT();

    TEST_CHECK("S2-U01", MOTION_ERROR == motion_get_status(NULL),
               "reject NULL status pointer");
    TEST_CHECK("S2-U02", MOTION_OK != motion_set_speed(-1.0f, 0.0f),
               "reject negative speed without moving wheels");
    TEST_CHECK("S2-U03", MOTION_NOT_IMPLEMENTED ==
               motion_move_forward(10.0f),
               "distance moves not implemented yet");

    TEST_SUMMARY("subsystem2");
}

#endif /* APP_TEST */
