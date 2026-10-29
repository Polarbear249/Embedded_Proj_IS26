/*
 *	subsystem1.c
 *	Subsystem 1 -- creates its objects and starts its tasks.
 */

#include "subsystem1.h"

EXPORT ID	tid_led;
EXPORT ID	cyc_led;
EXPORT ID	tid_wifi;
EXPORT ID	mbf_telem;
EXPORT ID	mbf_cmd;

/* Legacy IF-05 stub: idles forever, drives no pin. */
EXPORT void led_task(INT stacd, void *exinf)
{
	(void)stacd;
	(void)exinf;
	for (;;) {
		(void)tk_slp_tsk(TMO_FEVR);
	}
}

EXPORT void led_cyclic_handler(void *exinf)
{
	(void)exinf;
}

EXPORT ER subsystem1_init(void)
{
	T_CCYC	ccyc = {0};
	T_CMBF	cmbf = {0};

	/* Message buffers copy each message by value, so no task shares memory
	   with another and no lock is needed (design review: no mutex in the
	   Vehicle Controller's path). */
	cmbf.mbfatr = TA_TFIFO;
	cmbf.bufsz  = S1_MBF_TELEM_BYTES;
	cmbf.maxmsz = TELEM_LEN;
	mbf_telem = tk_cre_mbf(&cmbf);
	if (mbf_telem < E_OK) {
		return mbf_telem;
	}
	cmbf.bufsz  = S1_MBF_CMD_BYTES;
	cmbf.maxmsz = CMD_LEN;
	mbf_cmd = tk_cre_mbf(&cmbf);
	if (mbf_cmd < E_OK) {
		return mbf_cmd;
	}

	ccyc.cycatr = TA_HLNG;
	ccyc.cychdr = (FP)led_cyclic_handler;
	ccyc.cyctim = S1_TICK_MS;
	ccyc.cycphs = S1_TICK_MS;
	cyc_led = tk_cre_cyc(&ccyc);
	return (cyc_led < E_OK) ? cyc_led : E_OK;
}

EXPORT ER subsystem1_start(void)
{
	tid_led = app_start_task(led_task, S1_PRI_LED, 0, "led_task");
	if (tid_led < E_OK) {
		return tid_led;
	}
	tid_wifi = app_start_task(wifi_task, S1_PRI_WIFI, 0, "wifi_task");
	if (tid_wifi < E_OK) {
		return tid_wifi;
	}
	return tk_sta_cyc(cyc_led);
}