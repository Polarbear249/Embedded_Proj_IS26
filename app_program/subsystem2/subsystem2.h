/*
 *	subsystem2.h
 *	Subsystem 2 -- Producer (sample source).  Student ID: <student ID>.
 *
 *	Public interface of subsystem 2.  Other subsystems include it as
 *	"../subsystem2/subsystem2.h".
 */

#ifndef SUBSYSTEM2_H
#define SUBSYSTEM2_H

#include "../app.h"
#include "motion_control.h"

#define S2_PRI_PRODUCER		14	/* priority band 14-15 (TEAM.md) */
#define S2_PERIOD_MS		300	/* one sample every 300 ms */

/* ---------------------------------------------------------------------
 *  IF-01 data format: one sample, sent to subsystem 3's mailbox.
 *  A mailbox message MUST start with a T_MSG header: the kernel links
 *  messages together through it, so the data itself is never copied.
 * --------------------------------------------------------------------- */
typedef struct {
	T_MSG	hdr;		/* used by the kernel, do not touch */
	UW	seq;		/* sample sequence number */
	UW	time_ms;	/* system time when the sample was taken */
	INT	value;		/* the "measurement", 0..100 */
	INT	prc;		/* processor (core) that produced it: 1 or 2 */
} SAMPLE_MSG;

#define S2_POOL_COUNT	4	/* at most 4 samples in flight */

/* ---------------------------------------------------------------------
 *  Interface between Vehicle Controller (Buddy 2) and Obstacle Scan (Buddy 5)
 * --------------------------------------------------------------------- */

// Message from Vehicle Controller to Obstacle Scan
typedef struct {
    T_MSG hdr;      // Required by RTOS
    UINT reason;    // 1 = Time, 2 = Turn, 3 = Barcode, 4 = Line Lost
} SCAN_REQUEST;

// Message from Obstacle Scan back to Vehicle Controller
typedef struct {
    T_MSG hdr;
    UINT8 best_angle;
    float max_dist;
} OBSTACLE_PROFILE;

/* IF-02: the receiver of a sample gives its memory block back to
   mpf_samples (tk_rel_mpf) and then a slot back to sem_slots
   (tk_sig_sem, count 1).  Nothing else may use these two objects. */
IMPORT ID	sem_slots;
IMPORT ID	mpf_samples;

/* The mailboxes themselves (defined in subsystem5.c) */
IMPORT ID mbx_scan_req;
IMPORT ID mbx_obstacle_rpt;

#define S2_PRI_VEHICLE  7
void vehicle_controller_task(INT stacd, void *exinf);
/* Private to subsystem 2 */



#endif /* SUBSYSTEM2_H */
