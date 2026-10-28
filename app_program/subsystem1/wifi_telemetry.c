/*
 *	wifi_telemetry.c
 *	Subsystem 1 -- WiFi state task.
 *
 *	The kernel's cyw43_utk_task owns the radio, WiFi and MQTT
 *	(TEAM.md section 3). This task only READS the connection state, so
 *	it cannot block the radio or the motion tasks (assumption A1).
 */

#include "subsystem1.h"
#include "cyw43_utk.h"

static volatile UW	s_link_up;	/* 1 = joined and has an IP */
static volatile UW	s_ip_addr;	/* lwIP u32, first octet in low byte */
static volatile UW	s_poll_count;

EXPORT BOOL s1_wifi_link_up(void)
{
	return (s_link_up != 0U) ? TRUE : FALSE;
}

EXPORT UW s1_wifi_ip(void)
{
	return s_ip_addr;
}

EXPORT UW s1_wifi_poll_count(void)
{
	return s_poll_count;
}

EXPORT void wifi_task(INT stacd, void *exinf)
{
	T_CYW43_UTK_STATUS	status;
	UW			link_up;
	UW			prev_up = 0U;

	(void)stacd;
	(void)exinf;

	for (;;) {
		cyw43_utk_get_status(&status);
		link_up = (status.link_status == S1_LINK_HAS_IP) ? 1U : 0U;
		s_ip_addr = status.ip_addr;
		s_link_up = link_up;
		s_poll_count = s_poll_count + 1U;

		if (link_up != prev_up) {
			prev_up = link_up;
			tm_printf((UB*)"s1: WiFi %s\n",
				  (link_up != 0U) ? "UP" : "DOWN");
		}
		(void)tk_dly_tsk(S1_POLL_MS);
	}
}