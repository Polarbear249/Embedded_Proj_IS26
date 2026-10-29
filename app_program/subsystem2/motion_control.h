#ifndef S2_MOTION_CONTROL_H
#define S2_MOTION_CONTROL_H
#include "../app.h"
/* Task-context API, safe across cores. Duty units: 1000 = 100%,
 * deliberately restricted to +/-200 (20%) for initial bench bring-up.
 * A successful command means accepted; hardware updates on the next tick.
 * Renew commands every 100 ms, with e.g. a 300 ms lease. */
#define MOTION_PERIOD_MS 10
#define MOTION_MAX_DUTY 200
#define MOTION_MAX_LEASE_MS 1000
ER motion_set_duty(INT left, INT right, UINT lease_ms);
ER motion_stop(void); /* cancels current command; another command may follow */
ER motion_estop(void); /* latched; new movement is rejected */
ER motion_clear_estop(void); /* clears latch but never restarts movement */
typedef struct {
    INT left_duty, right_duty; /* requested, not measured wheel speed */
    BOOL estopped;
    BOOL expired;
} MOTION_STATUS;
ER motion_get_status(MOTION_STATUS *status);
/* Internal lifecycle, called by subsystem2 only. */
ER motion_init(void);
void motion_task(INT stacd, void *exinf);
#endif
