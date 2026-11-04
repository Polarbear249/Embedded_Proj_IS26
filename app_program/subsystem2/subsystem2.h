/*
 *	subsystem2.h
 *	Subsystem 2 -- Motion control.  Student ID: <student ID>.
 *
 *	Public interface of subsystem 2.  Other subsystems include it as
 *	"../subsystem2/subsystem2.h".
 */

#ifndef SUBSYSTEM2_H
#define SUBSYSTEM2_H

#include "../app.h"
#include "motion_control.h"

#define S2_PRI_MOTION		5	/* Motion Control, TEAM.md section 4 */
#define S2_DIST_PERIOD_MS	10	/* IF-10 publish period, 1 tick */

/* Private to subsystem 2 */

/*
 * 1 = run the move and turn try-out once after boot (motion_demo.c). The
 * car drives by itself, so keep it 0 in commits. Needs S3_DRIVE_SWITCH 0.
 */
#define S2_MOVE_DEMO		0
#define S2_PRI_DEMO		15	/* below Motion, as the VC test drive */
void motion_demo_task(INT stacd, void *exinf);


#endif /* SUBSYSTEM2_H */
