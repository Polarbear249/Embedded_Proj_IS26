/*
 *	subsystem2.c
 *	Subsystem 2 -- Motion control: creates its objects and starts its task.
 *
 *	The template producer objects (sem_slots, mpf_samples) were removed
 *	in integration step 0; motion_task never used them.
 */

#include "subsystem2.h"
#include "motion_control.h"

EXPORT ER subsystem2_init(void)
{
	return motion_init();
}

EXPORT ER subsystem2_start(void)
{
	/* Core 1 (TP_PRC2), as before; core assignment still open (TEAM.md 7) */
	ID tid = app_start_task(motion_task, S2_PRI_PRODUCER, TP_PRC2,
				"motion_task");
	return (tid < E_OK) ? tid : E_OK;
}
