/*
 *	subsystem1.h
 *	Subsystem 1 -- WiFi communication.  Student ID: <student ID>.
 *
 *	PUBLIC interface only; everything else stays local to the .c files.
 */

#ifndef SUBSYSTEM1_H
#define SUBSYSTEM1_H

#include "../app.h"
#include "telemetry_frame.h"

#define S1_PRI_WIFI	10		/* WiFi task, TEAM.md section 4 */
#define S1_POLL_MS	200U		/* WiFi state poll period, ms */
#define S1_LINK_HAS_IP	3		/* cyw43 link_status 3 = UP (joined + IP); 2 = no IP yet */

/* IF-05 (legacy): subsystem 4 still pauses/resumes this idle stub task.
   It drives no pin. Remove together with imu.c once the team decides
   who owns the liveness LED (TEAM.md section 4). */
#define S1_PRI_LED	12
#define S1_TICK_MS	250
IMPORT ID	tid_led;
IMPORT ID	cyc_led;
IMPORT void	led_task(INT stacd, void *exinf);
IMPORT void	led_cyclic_handler(void *exinf);

IMPORT ID	tid_wifi;
IMPORT void	wifi_task(INT stacd, void *exinf);

/* IF-07: Vehicle Controller -> S1. Message buffer owned by S1; each message
   is one 13-byte frame (TELEM_LEN), copied by value. */
IMPORT ID	mbf_telem;
/* IF-08: S1 -> Vehicle Controller. Message buffer holding validated 3-byte
   command frames (CMD_LEN). Created by S1 until the Vehicle Controller exists
   (TEAM.md lists the VC as owner: move it there when it is written). */
IMPORT ID	mbf_cmd;

#define S1_MBF_TELEM_BYTES	160	/* about 8 frames of backlog */
#define S1_MBF_CMD_BYTES	128

/* Vehicle Controller calls this: encodes the snapshot and queues it without
   ever blocking. E_OK, E_PAR (field out of range) or E_TMOUT (buffer full). */
IMPORT ER	s1_telem_send(const T_TELEM *snap);

/* Vehicle Controller calls this: the next validated command. Returns E_OK,
   or a negative error (E_TMOUT when none is waiting with tmout = TMO_POL). */
IMPORT ER	s1_cmd_recv(T_CMD *out, TMO tmout);

/* Counters, for tests and diagnostics. */
IMPORT UW	s1_telem_published(void);
IMPORT UW	s1_cmd_accepted(void);
IMPORT UW	s1_cmd_rejected(void);

/* Hooks called by lib/liblwip/.../mqtt_utk.c inside the WiFi service task. */
IMPORT uint32_t	mqtt_app_tx(uint8_t *buf, uint32_t max);
IMPORT void	mqtt_app_rx(const char *topic, const uint8_t *data, uint32_t len);

/* Read-only WiFi state, safe to call from any task. */
IMPORT BOOL	s1_wifi_link_up(void);
IMPORT UW	s1_wifi_ip(void);
IMPORT UW	s1_wifi_poll_count(void);

#endif /* SUBSYSTEM1_H */