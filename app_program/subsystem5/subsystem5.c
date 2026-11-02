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

#include "../subsystem2/subsystem2.h" // SCAN_REQUEST, OBSTACLE_PROFILE
#include "subsystem5.h"
#include "obstacle.h"
#include "ultrasonic.h"
#include "servo.h"

// Define the mailboxes declared in subsystem2.h
EXPORT ID mbx_scan_req;
EXPORT ID mbx_obstacle_rpt;

// Initialise the subsystem's hardware and kernel objects
EXPORT ER subsystem5_init(void)
{
    ER er;
    T_CMBX cmbx = {0};
    T_CMBF cmbf = {0};

    // Create Mailbox for Buddy 2 to wake subsystem5
    cmbx.mbxatr = TA_TFIFO;
    mbx_scan_req = tk_cre_mbx(&cmbx);
    if (mbx_scan_req < E_OK) return mbx_scan_req;

    // Create Message Buffer to send reports back to Buddy 2
    cmbf.maxmsz = sizeof(OBSTACLE_PROFILE);
    cmbf.bufsz = (sizeof(OBSTACLE_PROFILE) + 4) * 4; // Hold up to 4 reports, account for 4 byte header
    mbx_obstacle_rpt = tk_cre_mbf(&cmbf);
    if (mbx_obstacle_rpt < E_OK) return mbx_obstacle_rpt;

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
    ID tid = app_start_task(obstacle_task, S5_PRI_MAIN, 0, "obstacle_task");
    return (tid < E_OK) ? tid : E_OK;
}