/*
 *	subsystem1_test.c
 *	On-target tests for subsystem 1.  Built only with: make TEST=1
 *
 *	Each TEST_CHECK prints a PASS/FAIL line on the console.  Save the
 *	console output to testing/subsystem1/logs/ and record the result
 *	in testing/subsystem1/TEST_LOG.md.  Test IDs: S1-U01, S1-U02, ...
 */

#include "subsystem1.h"
#include "../test_report.h"

#if APP_TEST

EXPORT void subsystem1_test(void)
{
	T_RCYC	rcyc;
	T_RTSK	rtsk;
	ER	er;

	
	TEST_INIT();

	/* S1-U01: the cyclic handler exists and is running */
	er = tk_ref_cyc(cyc_led, &rcyc);
	TEST_CHECK("S1-U01", er == E_OK && (rcyc.cycstat & TCYC_STA),
		   "cyclic handler is running");

	/* S1-U02: between ticks the LED task is asleep, not busy-looping.
	   Sample it 10 times over ~1 s; it should almost always be waiting. */
	{
		INT	i, waiting = 0;
		for (i = 0; i < 10; i++) {
			tk_dly_tsk(97);	/* not a multiple of the tick */
			if (tk_ref_tsk(tid_led, &rtsk) == E_OK &&
			    (rtsk.tskstat & TTS_WAI)) {
				waiting++;
			}
		}
		TEST_CHECK("S1-U02", waiting >= 9, "LED task sleeps between ticks");
	}
		/* S1-U07: the WiFi task sleeps between polls (A1: it must never
	   busy-loop and starve the motion or sensing tasks). */
	{
		INT	i, waiting = 0;
		for (i = 0; i < 10; i++) {
			tk_dly_tsk(97);
			if (tk_ref_tsk(tid_wifi, &rtsk) == E_OK &&
			    (rtsk.tskstat & TTS_WAI)) {
				waiting++;
			}
		}
		TEST_CHECK("S1-U07", waiting >= 9, "WiFi task sleeps between polls");
	}

	/* S1-U08: the WiFi task is alive: about 5 polls in 1.1 s. */
	{
		UW	before = s1_wifi_poll_count();
		tk_dly_tsk(1100);
		TEST_CHECK("S1-U08", (s1_wifi_poll_count() - before) >= 4U,
			   "WiFi task polls about every 200 ms");
	}

		/* S1-U09: after joining the hotspot the WiFi task reports link UP
	   with a non-zero IP. Needs the hotspot on and in range. */
	{
		INT	i;
		for (i = 0; i < 30 && !s1_wifi_link_up(); i++) {
			(void)tk_dly_tsk(500);
		}
		TEST_CHECK("S1-U09", s1_wifi_link_up() && (s1_wifi_ip() != 0U),
			   "WiFi task reports link UP with an IP");
	}

	TEST_SUMMARY("subsystem1");
}

#endif /* APP_TEST */
