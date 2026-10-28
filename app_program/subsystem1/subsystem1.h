/*
 *	subsystem1.h
 *	Subsystem 1 -- WiFi communication.  Student ID: <student ID>.
 *
 *	PUBLIC interface only; everything else stays local to the .c files.
 */

#ifndef SUBSYSTEM1_H
#define SUBSYSTEM1_H

#include "../app.h"

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

/* Read-only WiFi state, safe to call from any task. */
IMPORT BOOL	s1_wifi_link_up(void);
IMPORT UW	s1_wifi_ip(void);
IMPORT UW	s1_wifi_poll_count(void);

#endif /* SUBSYSTEM1_H */