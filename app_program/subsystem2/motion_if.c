#include "motion_control.h"

/*
 * Buddy 2 - IF-01 Q_motion and IF-02 EVG_stop: kernel objects owned by S2
 * (TEAM.md section 5). Created once in motion_init().
 */

EXPORT ID Q_motion;
EXPORT ID EVG_stop;

ER motion_if_init(void)
{
    T_CFLG cflg = {0};
    T_CMBF cmbf = {0};

    cflg.flgatr = TA_TFIFO | TA_WMUL;   /* several tasks may wait */
    cflg.iflgptn = EVG_STOPPED;         /* wheels start at zero */
    EVG_stop = tk_cre_flg(&cflg);

    if (EVG_stop < E_OK)
    {
        return EVG_stop;
    }

    cmbf.mbfatr = TA_TFIFO;
    /* The kernel stores a length header with each message. */
    cmbf.bufsz = MOTION_QUEUE_DEPTH * (sizeof(UW) + sizeof(INT));
    cmbf.maxmsz = sizeof(UW);
    Q_motion = tk_cre_mbf(&cmbf);

    return (Q_motion < E_OK) ? Q_motion : E_OK;
}

ER motion_send(UW cmd)
{
    return tk_snd_mbf(Q_motion, &cmd, sizeof(cmd), TMO_POL);
}

ER motion_request_stop(void)
{
    return tk_set_flg(EVG_stop, EVG_STOP_REQ);
}

BOOL motion_if_stop_requested(void)
{
    T_RFLG ref;

    /* Fail safe: if the flag cannot be read, behave as if stop is set. */
    if (tk_ref_flg(EVG_stop, &ref) < E_OK)
    {
        return TRUE;
    }

    return (ref.flgptn & EVG_STOP_REQ) ? TRUE : FALSE;
}

BOOL motion_if_receive(UW *p_cmd)
{
    return tk_rcv_mbf(Q_motion, p_cmd, TMO_POL) == (INT)sizeof(*p_cmd);
}

void motion_if_report_stopped(BOOL stopped)
{
    static BOOL reported = TRUE;    /* matches iflgptn */

    if (stopped == reported)
    {
        return;
    }

    if ((stopped ? tk_set_flg(EVG_stop, EVG_STOPPED)
                 : tk_clr_flg(EVG_stop, ~(UINT)EVG_STOPPED)) == E_OK)
    {
        reported = stopped;
    }
}

ER motion_if_clear_stop(void)
{
    return tk_clr_flg(EVG_stop, ~(UINT)EVG_STOP_REQ);
}
