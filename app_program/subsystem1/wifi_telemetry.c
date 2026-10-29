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
static volatile UW	s_telem_published;	/* frames handed to MQTT */
static volatile UW	s_telem_bad;		/* frames dropped: failed decode */
static volatile UW	s_cmd_accepted;		/* commands forwarded to VC */
static volatile UW	s_cmd_rejected;		/* commands refused: bad frame */
static volatile UW	s_cmd_dropped;		/* valid but Q_cmd was full */

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

EXPORT UW s1_telem_published(void)
{
	return s_telem_published;
}

EXPORT UW s1_cmd_accepted(void)
{
	return s_cmd_accepted;
}

EXPORT UW s1_cmd_rejected(void)
{
	return s_cmd_rejected;
}

/* ---- IF-07: Vehicle Controller -> S1 (telemetry) ---- */

EXPORT ER s1_telem_send(const T_TELEM *snap)
{
	uint8_t		frame[TELEM_LEN];

	if (telem_encode(snap, frame) != FRAME_OK) {
		return E_PAR;
	}
	return tk_snd_mbf(mbf_telem, frame, (INT)TELEM_LEN, TMO_POL);
}

/* Called by mqtt_utk.c every 200 ms inside cyw43_utk_task. Never blocks:
   TMO_POL. Several frames may be waiting; only the newest is worth sending,
   so older ones are dropped rather than queued up behind a slow link. */
EXPORT uint32_t mqtt_app_tx(uint8_t *buf, uint32_t max)
{
	uint8_t		frame[TELEM_LEN];
	T_TELEM		check;
	uint32_t	have = 0U;
	uint32_t	i;
	INT		n;

	if (max < TELEM_LEN) {
		return 0U;
	}
	for (;;) {
		n = tk_rcv_mbf(mbf_telem, frame, TMO_POL);
		if (n != (INT)TELEM_LEN) {
			break;
		}
		for (i = 0U; i < TELEM_LEN; i++) {
			buf[i] = frame[i];
		}
		have = TELEM_LEN;
	}
	if (have == 0U) {
		return 0U;
	}
	if (telem_decode(buf, have, &check) != FRAME_OK) {
		s_telem_bad = s_telem_bad + 1U;
		return 0U;
	}
	s_telem_published = s_telem_published + 1U;
	return have;
}

/* ---- IF-08: S1 -> Vehicle Controller (commands) ---- */

/* Called by mqtt_utk.c for each complete message on the command topic. The
   frame is checked here; nothing unchecked reaches the Vehicle Controller. */
EXPORT void mqtt_app_rx(const char *topic, const uint8_t *data, uint32_t len)
{
	T_CMD		cmd;
	T_FRAME_STATUS	st;

	(void)topic;
	st = cmd_decode(data, len, &cmd);
	if (st != FRAME_OK) {
		s_cmd_rejected = s_cmd_rejected + 1U;
		tm_printf((UB*)"s1: command rejected (code %d)\n", (int)st);
		return;
	}
	if (tk_snd_mbf(mbf_cmd, data, (INT)CMD_LEN, TMO_POL) != E_OK) {
		s_cmd_dropped = s_cmd_dropped + 1U;
		tm_printf((UB*)"s1: command queue full, dropped\n");
		return;
	}
	s_cmd_accepted = s_cmd_accepted + 1U;
	tm_printf((UB*)"s1: command %u arg %u seq %u accepted\n",
		  (unsigned int)cmd.cmd, (unsigned int)cmd.arg,
		  (unsigned int)cmd.seq);
}

EXPORT ER s1_cmd_recv(T_CMD *out, TMO tmout)
{
	uint8_t		frame[CMD_LEN];
	INT		n;

	n = tk_rcv_mbf(mbf_cmd, frame, tmout);
	if (n < 0) {
		return (ER)n;
	}
	if (n != (INT)CMD_LEN) {
		return E_SYS;
	}
	return (cmd_decode(frame, CMD_LEN, out) == FRAME_OK) ? E_OK : E_PAR;
}

EXPORT void wifi_task(INT stacd, void *exinf)
{
	T_CYW43_UTK_STATUS	status;
	UW			link_up;
	UW			prev_up = 0U;
	UW			ticks = 0U;

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
		/* Diagnostic: while the link is down, report every 5 s so a
		   silent WiFi stall shows up in the log. kpolls is the kernel
		   WiFi task's own loop counter: if it stops rising, that task
		   is stuck; if it rises but the link never returns, the
		   re-join is not happening. */
		ticks = ticks + 1U;
		if ((link_up == 0U) && ((ticks % 25U) == 0U)) {
			tm_printf((UB*)"s1: link down, status=%d kpolls=%u\n",
				  (int)status.link_status,
				  (unsigned int)status.poll_count);
		}
		(void)tk_dly_tsk(S1_POLL_MS);
	}
}