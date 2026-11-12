/*
 *	telem_monitor.c
 *	Subsystem 1 -- the vehicle snapshot, in words, on the serial monitor.
 *
 *	Why: the MQTT telemetry frame is packed binary (14 bytes, minimum-bit
 *	requirement), so nobody can read it by eye while calibrating. This
 *	prints the SAME snapshot as text, once a second, so what the team
 *	reads here is exactly what a remote listener receives.
 *
 *	  s1_telem_print()  prints a snapshot. Callable from any task.
 *	  telem_task()      gathers the snapshot from the other subsystems,
 *	                    sends it through IF-07 and prints it.
 *
 *	WHERE THIS BELONGS: collecting the snapshot is the Vehicle
 *	Controller's job (TEAM.md section 5: VC -> S1 over IF-07). It sits in
 *	S1 only because vc_init()/vc_start() do not do it yet. Move
 *	collect_snapshot() and telem_task() into vehicle_controller.c
 *	unchanged when the VC takes it over, and delete them here.
 *
 *	Reading other subsystems' data is safe: every source below is a
 *	single aligned load or a copy-out getter, written by one task and
 *	never written by a reader (TEAM.md IF-06, IF-10, IF-11). No locks,
 *	and no hardware is touched here, so the owning tasks keep their
 *	peripherals to themselves.
 */

#include "subsystem1.h"
#include "../subsystem2/motion_control.h"	/* IF-10: RPM, distance */
#include "../subsystem3/subsystem3.h"		/* IF-06: line_state word */
#include "../subsystem4/subsystem4.h"		/* IF-11: hump word, state */
#include "../subsystem4/imu_hump.h"		/* IMU_HUMP_W_* field macros */

/*
 *	Subsystem 5 has no getter yet, so the obstacle fields stay empty.
 *	What S5 needs to add (one copy-out getter, no hardware access; the
 *	obstacle task stores its last scan result and this copies it):
 *
 *	    ER s5_get_obstacle(UB *found, UH *left_cm, UH *right_cm,
 *	                       UH *width_cm);
 *
 *	Then set S1_HAVE_S5_OBSTACLE to 1 and fill the four fields in
 *	collect_snapshot(). Field widths: obs_ok 1 bit, the three distances
 *	9 bits each (0..511 cm), so clamp before storing.
 */
#define S1_HAVE_S5_OBSTACLE	0

/*
 *	Wheel circumference, from subsystem2/encoder.c (pi x 66 mm).
 *	RPM -> cm/s: rpm x circumference_cm / 60.
 */
#define S1_WHEEL_CIRC_CM	(20.73f)

EXPORT ID	tid_telem;

static UW	s_telem_seq;
static UB	s_last_char_seq;	/* IF-06 char_seq, to spot a new barcode */
static UB	s_char_seq_valid;

/* ---------------------------------------------------------------------
 *	Names, for the printed lines only. The numbers themselves are what
 *	go into the frame.
 * ------------------------------------------------------------------ */

static const char *mstate_name(UB mstate)
{
	switch (mstate) {
	case 0U:	return "stopped";
	case 1U:	return "moving";
	case 2U:	return "turning";
	case 3U:	return "reversing";
	case 4U:	return "scanning";
	case 5U:	return "bypassing";
	case 6U:	return "estop/fault";
	default:	return "?";
	}
}

static const char *line_mode_name(UB mode)
{
	switch (mode) {
	case LINE_MODE_INIT:		return "init";
	case LINE_MODE_TRACKING:	return "tracking";
	case LINE_MODE_JUNCTION:	return "junction";
	case LINE_MODE_SEARCH_NEAR:	return "search-near";
	case LINE_MODE_SEARCH_FAR:	return "search-far";
	case LINE_MODE_FAILED:		return "failed";
	default:			return "?";
	}
}

static const char *imu_state_name(imu_terrain_state_t st)
{
	switch (st) {
	case IMU_TERRAIN_INIT:		return "init";
	case IMU_TERRAIN_CALIBRATING:	return "calibrating";
	case IMU_TERRAIN_ACTIVE:	return "active";
	case IMU_TERRAIN_FAULT:		return "FAULT";
	case IMU_TERRAIN_HUMP_UP:	return "hump-up";
	case IMU_TERRAIN_HUMP_DOWN:	return "hump-down";
	default:			return "?";
	}
}

/* ---------------------------------------------------------------------
 *	The printed snapshot.
 * ------------------------------------------------------------------ */

EXPORT void s1_telem_print(const T_TELEM *snap)
{
	if (snap == NULL) {
		return;
	}

	tm_printf((UB*)"\n[TELEM] seq %u  vstate %u  motion %u (%s)\n",
		  (unsigned int)snap->seq, (unsigned int)snap->vstate,
		  (unsigned int)snap->mstate, mstate_name(snap->mstate));

	tm_printf((UB*)"  motion   speed L %u  R %u cm/s   distance %u cm\n",
		  (unsigned int)snap->speed_l, (unsigned int)snap->speed_r,
		  (unsigned int)snap->dist_cm);

	/* line_err is offset binary: 64 = centred, below = line to the left. */
	tm_printf((UB*)"  line     err %d  on_line %u  junction %u\n",
		  (int)snap->line_err - (int)LINE_IF_ERROR_CENTRE,
		  (unsigned int)snap->line_ok, (unsigned int)snap->junction);

	if ((snap->bc_ok != 0U) && (snap->bc_char <= LINE_CHAR_MAX)) {
		tm_printf((UB*)"  barcode  '%c' (%u)\n",
			  (int)('A' + (int)snap->bc_char),
			  (unsigned int)snap->bc_char);
	} else {
		tm_printf((UB*)"  barcode  none yet\n");
	}

	tm_printf((UB*)"  terrain  hump %u mm\n", (unsigned int)snap->hump_mm);

#if S1_HAVE_S5_OBSTACLE
	tm_printf((UB*)"  obstacle found %u  left %u  right %u  width %u cm\n",
		  (unsigned int)snap->obs_ok, (unsigned int)snap->obs_left,
		  (unsigned int)snap->obs_right,
		  (unsigned int)snap->obs_width);
#else
	tm_printf((UB*)"  obstacle --   (S5: no getter yet,"
		  " see s5_get_obstacle in telem_monitor.c)\n");
#endif

	tm_printf((UB*)"  faults   0x%02x%s%s%s%s%s\n",
		  (unsigned int)snap->faults,
		  ((snap->faults & TELEM_FAULT_MOTOR_STALL) != 0U)
			? " motor_stall" : "",
		  ((snap->faults & TELEM_FAULT_ENCODER) != 0U)
			? " encoder" : "",
		  ((snap->faults & TELEM_FAULT_IMU) != 0U) ? " imu" : "",
		  ((snap->faults & TELEM_FAULT_NO_ECHO) != 0U)
			? " no_echo" : "",
		  ((snap->faults & TELEM_FAULT_WIFI_LOSS) != 0U)
			? " wifi_loss" : "");
}

/* ---------------------------------------------------------------------
 *	Gathering the snapshot.
 * ------------------------------------------------------------------ */

/* RPM -> cm/s, clamped to the field's 8 bits. Negative (reverse) reports
   as speed 0: the frame has no sign bit, and mstate carries direction. */
static UB rpm_to_cm_s(float rpm)
{
	float	cm_s;

	if (rpm < 0.0f) {
		rpm = -rpm;
	}
	cm_s = (rpm * S1_WHEEL_CIRC_CM) / 60.0f;
	if (cm_s > 255.0f) {
		return 255U;
	}
	return (UB)cm_s;
}

static void collect_motion(T_TELEM *snap)
{
	motion_status_t	ms;
	int32_t		dist_mm;
	int32_t		dist_cm;

	if (motion_get_status(&ms) != MOTION_OK) {
		/* No status at all: say so rather than report zeros as fact. */
		snap->faults |= TELEM_FAULT_ENCODER;
		return;
	}

	snap->speed_l = rpm_to_cm_s(ms.left_rpm);
	snap->speed_r = rpm_to_cm_s(ms.right_rpm);

	if (motion_is_estopped()) {
		snap->mstate = 6U;			/* estop */
	} else if (ms.state == MOTION_STATE_FAULT) {
		snap->mstate = 6U;
		snap->faults |= TELEM_FAULT_MOTOR_STALL;
	} else if (ms.state == MOTION_STATE_RUNNING) {
		/* One wheel forward and the other back is a spin on the spot;
		   both the same way is ordinary travel. */
		if (((ms.left_rpm < 0.0f) && (ms.right_rpm > 0.0f))
		 || ((ms.left_rpm > 0.0f) && (ms.right_rpm < 0.0f))) {
			snap->mstate = 2U;		/* turning */
		} else if ((ms.left_rpm < 0.0f) || (ms.right_rpm < 0.0f)) {
			snap->mstate = 3U;		/* reversing */
		} else {
			snap->mstate = 1U;		/* moving */
		}
	} else {
		snap->mstate = 0U;			/* stopped */
	}

	/* IF-10: mm since power-up, mean of both wheels. dist_cm is 16 bits,
	   so about 655 m before it would wrap; clamp rather than wrap, and
	   report backwards travel as 0 (the field is unsigned). */
	dist_mm = motion_get_distance_mm();
	dist_cm = dist_mm / 10;
	if (dist_cm < 0) {
		dist_cm = 0;
	} else if (dist_cm > 65535) {
		dist_cm = 65535;
	}
	snap->dist_cm = (uint16_t)dist_cm;
}

static void collect_line(T_TELEM *snap)
{
	uint32_t	word;
	line_if_state_t	ls;

	word = s3_line_state_read();		/* IF-06, one atomic load */
	line_if_unpack(word, &ls);

	/* Buddy 3 packs the same 7-bit offset-binary error as line_err
	   (line_if.h), so it copies straight across. */
	snap->line_err  = ls.error;
	snap->line_ok   = ((ls.mode == LINE_MODE_TRACKING)
			|| (ls.mode == LINE_MODE_JUNCTION)) ? 1U : 0U;
	snap->junction  = (ls.mode == LINE_MODE_JUNCTION) ? 1U : 0U;

	/* bc_char is only meaningful once char_seq has moved at least once:
	   before any barcode is decoded it is LINE_CHAR_NONE (CLAUDE.md 1b:
	   S3 reports the character; the VC maps it to a movement). */
	if (s_char_seq_valid == 0U) {
		s_last_char_seq = ls.char_seq;
		s_char_seq_valid = 1U;
	} else if (ls.char_seq != s_last_char_seq) {
		s_last_char_seq = ls.char_seq;
		snap->bc_ok = 1U;
	}
	if (g_s3_diag.decode_ok > 0U) {
		snap->bc_ok = 1U;		/* at least one real decode */
	}
	snap->bc_char = ls.bc_char;
}

static void collect_terrain(T_TELEM *snap)
{
	imu_terrain_state_t	st;
	uint32_t		hump;

	st = imu_terrain_get_state();
	if (st == IMU_TERRAIN_FAULT) {
		snap->faults |= TELEM_FAULT_IMU;
		return;
	}

	hump = imu_terrain_get_hump_word();	/* IF-11, one atomic load */
	if (IMU_HUMP_W_VALID(hump) != 0U) {
		snap->hump_mm = (UB)IMU_HUMP_W_LAST(hump);
	}
}

static void collect_snapshot(T_TELEM *snap)
{
	UW	i;
	UB	*p = (UB*)snap;

	for (i = 0U; i < sizeof(T_TELEM); i++) {
		p[i] = 0U;
	}

	snap->seq = (UB)(s_telem_seq & 0x1FU);		/* 5 bits, wraps */
	s_telem_seq = s_telem_seq + 1U;

	collect_motion(snap);
	collect_line(snap);
	collect_terrain(snap);

#if S1_HAVE_S5_OBSTACLE
	(void)s5_get_obstacle(&snap->obs_ok, &snap->obs_left,
			      &snap->obs_right, &snap->obs_width);
#endif

	if (!s1_wifi_link_up()) {
		snap->faults |= TELEM_FAULT_WIFI_LOSS;
	}

	/* vstate is the Vehicle Controller's mission state. vehicle_controller.c
	   publishes no getter yet, so it stays 0 rather than carrying a guess. */
	snap->vstate = 0U;
}

/* ---------------------------------------------------------------------
 *	The task.
 * ------------------------------------------------------------------ */

EXPORT void telem_task(INT stacd, void *exinf)
{
	T_TELEM			snap;
	motion_status_t		ms;
	uint32_t		word;
	ER			er;

	(void)stacd;
	(void)exinf;

	tm_printf((UB*)"s1: telemetry monitor every %u ms\n",
		  (unsigned int)S1_TELEM_MS);

	for (;;) {
		collect_snapshot(&snap);

		/* Both paths carry the same snapshot: MQTT in binary, the
		   serial monitor in words. */
		er = s1_telem_send(&snap);
		s1_telem_print(&snap);
		if (er != E_OK) {
			tm_printf((UB*)"  (telem not queued: %d)\n", (int)er);
		}

		/* Raw source values, for calibration. These are NOT in the
		   frame: they are what the frame's fields were derived from,
		   so a wrong conversion here shows up as a mismatch. */
		if (motion_get_status(&ms) == MOTION_OK) {
			tm_printf((UB*)"  raw s2   enc L %d R %d"
				  "  dist %d mm  state %d\n",
				  (int)ms.left_encoder_count,
				  (int)ms.right_encoder_count,
				  (int)motion_get_distance_mm(),
				  (int)ms.state);
		}

		word = s3_line_state_read();
		tm_printf((UB*)"  raw s3   mode %s  err %d  word 0x%08x"
			  "  decode ok %u fail %u\n",
			  line_mode_name(line_if_mode(word)),
			  (int)line_if_error_signed(word),
			  (unsigned int)word,
			  (unsigned int)g_s3_diag.decode_ok,
			  (unsigned int)g_s3_diag.decode_fail);

		tm_printf((UB*)"  raw s4   imu %s  hump word 0x%08x"
			  "  max %u mm\n",
			  imu_state_name(imu_terrain_get_state()),
			  (unsigned int)imu_terrain_get_hump_word(),
			  (unsigned int)IMU_HUMP_W_MAX(
					imu_terrain_get_hump_word()));

		tm_printf((UB*)"  wifi     %s  published %u"
			  "  cmd ok %u rejected %u\n",
			  s1_wifi_link_up() ? "UP" : "DOWN",
			  (unsigned int)s1_telem_published(),
			  (unsigned int)s1_cmd_accepted(),
			  (unsigned int)s1_cmd_rejected());

		(void)tk_dly_tsk(S1_TELEM_MS);
	}
}
