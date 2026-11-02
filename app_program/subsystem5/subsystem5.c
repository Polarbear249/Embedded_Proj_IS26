/*
 *	subsystem5.c
 *	Subsystem 5 -- <Daryl>.
 *
 *	subsystem5_init():  create this subsystem's kernel objects
 *	                    (semaphores, mailboxes, ...).  Do not start
 *	                    tasks here.
 *	subsystem5_start(): create and start this subsystem's tasks, e.g.
 *	    ID tid = app_start_task(my_task, S5_PRI_MAIN, 0, "my_task");
 *	    return (tid < E_OK) ? tid : E_OK;
 *
 *	Drivers for this subsystem's hardware go in this folder as
 *	drv_<name>.c / drv_<name>.h, called only from the task that owns
 *	the peripheral (TEAM.md section 3).
 *
 *	Record every pin, task, object and interface in TEAM.md.
 */

#include "subsystem5.h"
#include "obstacle.h"
#include "ultrasonic.h"
#include "servo.h"

// Initialise the subsystem's hardware and kernel objects
EXPORT ER subsystem5_init(void)
{
    ER er;
	// Init ultrasonic sensor and servo motor
    er = ultrasonic_init();
    if (er < E_OK) return er;
    
    er = servo_init();
    if (er < E_OK) return er;

	return E_OK;
}

EXPORT ER subsystem5_start(void)
{
    // Priority 8, any core, obstacle_task for debug
	ID tid = app_start_task(obstacle_task, 8, 0, "obstacle_task");
	return (tid < E_OK) ? tid : E_OK; // Sunglass_thumbsup_emoji
}
