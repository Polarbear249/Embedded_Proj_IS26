/*
 *	subsystem3_test.c
 *	On-target tests for subsystem 3.  Built only with: make TEST=1
 *	Record unit results in testing/subsystem3/TEST_LOG.md.
 *
 *	The template tests INT-01 and S3-U01 checked the removed template
 *	producer/consumer and were removed with it.
 */

#include "subsystem3.h"
#include "../test_report.h"

#if APP_TEST

EXPORT void subsystem3_test(void)
{
}

#endif /* APP_TEST */
