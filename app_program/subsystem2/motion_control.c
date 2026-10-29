#include "motion_control.h"
#include "motor.h"

static ID command_mutex;
static INT requested_left, requested_right;
static UINT lease;
static UW issued;
static BOOL latched, expired;
static BOOL ready;

static ER lock_command(void)
{
    if (!ready) return E_OBJ;
    return tk_loc_mtx(command_mutex, TMO_POL);
}

ER motion_init(void)
{
    T_CMTX cfg = {0};
    ER er;
    if (ready) return E_OBJ;
    er = motor_init();
    if (er < E_OK) return er;
    cfg.mtxatr = TA_INHERIT;
    command_mutex = tk_cre_mtx(&cfg);
    if (command_mutex < E_OK) return command_mutex;
    ready = TRUE;
    return E_OK;
}

ER motion_set_duty(INT left, INT right, UINT lease_ms)
{
    SYSTIM now;
    ER er;
    if (left < -MOTION_MAX_DUTY || left > MOTION_MAX_DUTY ||
        right < -MOTION_MAX_DUTY || right > MOTION_MAX_DUTY ||
        lease_ms == 0 || lease_ms > MOTION_MAX_LEASE_MS) return E_PAR;
    er = lock_command();
    if (er < E_OK) return er;
    if (latched) er = E_OBJ;
    else {
        er = tk_get_tim(&now);
        if (er == E_OK) {
            requested_left = left; requested_right = right;
            issued = now.lo; lease = lease_ms; expired = FALSE;
        }
    }
    tk_unl_mtx(command_mutex);
    return er;
}

static ER cancel(BOOL latch, BOOL clear)
{
    ER er = lock_command();
    if (er < E_OK) return er;
    requested_left = requested_right = 0; lease = 0;
    if (latch) latched = TRUE;
    if (clear) latched = FALSE;
    tk_unl_mtx(command_mutex);
    return E_OK;
}
ER motion_stop(void) { return cancel(FALSE, FALSE); }
ER motion_estop(void) { return cancel(TRUE, FALSE); }
ER motion_clear_estop(void) { return cancel(FALSE, TRUE); }
ER motion_get_status(MOTION_STATUS *status)
{
    ER er;
    if (!status) return E_PAR;
    er = lock_command();
    if (er < E_OK) return er;
    status->left_duty = requested_left;
    status->right_duty = requested_right;
    status->estopped = latched; status->expired = expired;
    tk_unl_mtx(command_mutex);
    return E_OK;
}

/* Kept separate for host tests. Runs only in the hardware-owning task. */
static void motion_tick(void)
{
    static INT applied_left, applied_right;
    SYSTIM now;
    INT left, right;
    ER er = lock_command();
    if (er < E_OK) {
        motor_stop(); applied_left = applied_right = 0; return;
    }
    er = tk_get_tim(&now);
    if (er < E_OK || latched || lease == 0 ||
        (UW)(now.lo - issued) >= lease) {
        if (lease != 0 && !latched) expired = TRUE;
        requested_left = requested_right = 0; lease = 0;
    }
    left = requested_left; right = requested_right;
    /* One full task interval at zero before reversing either wheel. */
    if ((left > 0 && applied_left < 0) || (left < 0 && applied_left > 0) ||
        (right > 0 && applied_right < 0) || (right < 0 && applied_right > 0)) {
        left = right = 0;
    }
    motor_apply(left, right);
    applied_left = left; applied_right = right;
    tk_unl_mtx(command_mutex);
}

void motion_task(INT stacd, void *exinf)
{
    (void)stacd; (void)exinf;
    for (;;) { motion_tick(); tk_dly_tsk(MOTION_PERIOD_MS); }
}
