/*
 *	subsystem2.c
 *	Subsystem 2 -- Producer: creates its objects and starts its task.
 *
 *	APIs shown: tk_cre_sem, tk_cre_mpf
 */

#include "subsystem2.h"
#include "motion_control.h"

EXPORT ID	sem_slots;	/* IF-02, see subsystem2.h */
EXPORT ID	mpf_samples;

EXPORT ER subsystem2_init(void)
{
	T_CSEM	csem = {0};
	T_CMPF	cmpf = {0};

	/* Semaphore: a counter of free "slots".  The producer takes one
	   (tk_wai_sem) before each sample; the consumer gives one back
	   (tk_sig_sem).  It never exceeds the number of memory blocks, so
	   the producer can never run out of memory. */
	csem.sematr  = TA_TFIFO;	/* waiting tasks queue in FIFO order */
	csem.isemcnt = S2_POOL_COUNT;	/* initial count */
	csem.maxsem  = S2_POOL_COUNT;	/* maximum count */
	sem_slots = tk_cre_sem(&csem);
	if (sem_slots < E_OK) {
		return sem_slots;
	}

	/* Fixed-size memory pool: S2_POOL_COUNT blocks, each exactly one
	   SAMPLE_MSG.  Faster and more predictable than malloc(). */
	cmpf.mpfatr = TA_TFIFO | TA_RNG3;
	cmpf.mpfcnt = S2_POOL_COUNT;		/* number of blocks */
	cmpf.blfsz  = sizeof(SAMPLE_MSG);	/* size of one block */
	mpf_samples = tk_cre_mpf(&cmpf);
	return (mpf_samples < E_OK) ? mpf_samples : motion_init();
}

EXPORT ER subsystem2_start(void)
{
	/* Pinned to core 1 (TP_PRC2): samples cross to subsystem 3 on core 0 */
	ID tid1 = app_start_task(motion_task, S2_PRI_PRODUCER, TP_PRC2,
				"motion_task");
	if (tid1 < E_OK) return tid1;

    /* Start the Vehicle Controller task on any core */
    ID tid2 = app_start_task(vehicle_controller_task, S2_PRI_VEHICLE, 0, "vehicle_controller");
	return (tid2 < E_OK) ? tid2 : E_OK;
}

void vehicle_controller_task(INT stacd, void *exinf)
{
    (void)stacd; (void)exinf;
    SCAN_REQUEST req;
    OBSTACLE_PROFILE report;
    
    for (;;) {
        // Wake up Buddy 5 (Obstacle Scan)
        req.reason = 1; // 1 = Time (Routine scan)
        tm_printf((UB*)"[VC] Requesting forward scan...\n");
        tk_snd_mbx(mbx_scan_req, (T_MSG*)&req);
        
        // Wait for Buddy 5's report
        ER er = tk_rcv_mbf(mbx_obstacle_rpt, (T_MSG*)&report, 3000); // 3 sec timeout
        
        if (er == E_OK) {
            tm_printf((UB*)"[VC] Received report: Best Angle = %d, Max Dist = %.1f\n", 
                      report.best_angle, report.max_dist);
            
            // If the obstacle is too close, stop the car
            if (report.max_dist < 15.0) {
                tm_printf((UB*)"[VC] DANGER: Obstacle too close. Stopping.\n");
                motion_stop();
            } 
            // If the path is clear, drive forward
            else {
                tm_printf((UB*)"[VC] Path clear. Driving forward.\n");
                motion_set_duty(100, 100, 3000); // Move forward, 3000ms lease
            }
        } else {
            tm_printf((UB*)"[VC] Timeout waiting for Buddy 5.\n");
            // If can't hear from the sensor, stop the car
            motion_stop();
        }
        
        // Wait before next scan
        tk_dly_tsk(200);
    }
}