/*
 *	telemetry_frame.h
 *	Subsystem 1 -- compact binary frames for MQTT (design review: minimum-bit
 *	data, explicit shifts and masks, fixed byte order, version byte).
 *
 *	No kernel or network calls here: pure functions that can be unit-tested
 *	on a PC (testing/subsystem1/pc/).
 *
 *	TELEMETRY FRAME v4: 27 bytes, fields packed MSB-first, in this order:
 *	  version 8 | seq 5 | vstate 3 | mstate 3 | faults 5 | speed_l 8 |
 *	  speed_r 8 | dist_cm 16 | line_err 7 | line_ok 1 | junction 1 |
 *	  bc_ok 1 | bc_cmd 2 | bc_char 5 | nav_map 20 | hump_mm 8 | obs_ok 1 |
 *	  obs_left 9 | obs_right 9 | obs_width 9 | hump_max 8 | imu_state 2 |
 *	  enc_l 20 | enc_r 20 | bc_dec_ok 8 | bc_dec_fail 8 | cmd_ok 8 |
 *	  cmd_rej 8                                           = 211 bits
 *	  (padded to 216 bits; the last 5 bits are zero and ignored)
 *
 *	v4 appends what the serial monitor showed but the dashboard did not:
 *	  hump_max   highest hump since boot, mm (Mission Requirement 5:
 *	             "report the highest hump peak experienced during the
 *	             run"); hump_mm stays the most recent hump.
 *	  imu_state  IMU_TERRAIN_* (init, calibrating, active, fault).
 *	  enc_l/r    raw encoder counts, 20-bit two's complement. At about
 *	             61 counts/cm that is +-85 m; a 16-bit field would wrap
 *	             after about 5 m. Clamped, never wrapped.
 *	  bc_dec_*   S3 barcode decodes that succeeded / failed.
 *	  cmd_*      MQTT commands the car accepted / rejected.
 *	  The four counters stop at 255 rather than wrapping to small,
 *	  plausible-looking numbers.
 *	New fields go at the END so every earlier field keeps its bit
 *	position.
 *
 *	SIZE LIMIT: TELEM_LEN must stay <= MQTT_UTK_BUF_MAX (32 bytes, in
 *	lib/liblwip/sysdepend/pico_rp2040/mqtt_utk.h). Past that, mqtt_app_tx()
 *	is handed a buffer too small for the frame and nothing is published
 *	-- no error, the dashboard just stalls. v4 uses 27 of the 32.
 *
 *	v2 added bc_char so the dashboard can show the decoded letter, not
 *	only the movement it maps to. v3 adds nav_map, the barcode mapping
 *	the car actually holds, so the dashboard shows the car's state rather
 *	than what it believes it sent (a SET_MAP lost over WiFi would
 *	otherwise leave the two disagreeing). The version byte and the length
 *	change together each time, so an older reader rejects a newer frame
 *	rather than mis-decoding it -- the frame format is an interface.
 *
 *	nav_map is S3's mapping word copied as-is (barcode_nav.h): four 5-bit
 *	slots indexed by LINE_CMD_*, bits 4..0 LEFT, 9..5 RIGHT, 14..10
 *	STRAIGHT, 19..15 U-TURN; each 0 = no letter, 1..26 = 'A'..'Z'.
 *
 *	Two letter encodings appear in this file, each matching its own
 *	neighbour, so keep them apart:
 *	  bc_char (telemetry)  0 = none yet, 1..26 = 'A'..'Z'. Telemetry
 *	    starts before any barcode is read, so it needs a "nothing yet"
 *	    value; this is also barcode_nav.h's mapping-slot convention.
 *	  CMD_SET_MAP letter   0 = 'A' .. 25 = 'Z', with no spare value.
 *	    It matches line_if_char_code(), whose result it becomes, and a
 *	    command to map "no letter" would mean nothing.
 *
 *	COMMAND FRAME v1: 3 bytes, MSB-first:
 *	  version 8 | cmd 4 | arg 8 | seq 4                   = 24 bits
 */

#ifndef TELEMETRY_FRAME_H
#define TELEMETRY_FRAME_H

#include <stdint.h>

#define TELEM_VERSION		4U
#define TELEM_LEN		27U
#define TELEM_MSTATE_MAX	6U	/* seven motion states: 0..6 */
#define TELEM_MAP_SLOTS		4U	/* one per LINE_CMD_* movement */
#define TELEM_ENC_BITS		20U
#define TELEM_ENC_MIN		(-524288L)	/* -(2^19) */
#define TELEM_ENC_MAX		(524287L)	/* 2^19 - 1 */
#define TELEM_COUNT_MAX		255U	/* 8-bit counters saturate here */
#define TELEM_MAP_SLOT_BITS	5U
#define TELEM_MAP_LETTER_MAX	26U	/* slot value for 'Z' */
#define TELEM_MAP_SLOT(map, mv)	((uint8_t)(((map) >> ((mv) * 5U)) & 0x1FU))

/* Fault bits (individual bits of one mask, not separate integers). */
#define TELEM_FAULT_MOTOR_STALL	0x01U
#define TELEM_FAULT_ENCODER	0x02U
#define TELEM_FAULT_IMU		0x04U
#define TELEM_FAULT_NO_ECHO	0x08U
#define TELEM_FAULT_WIFI_LOSS	0x10U

#define CMD_VERSION		1U
#define CMD_LEN			3U
#define CMD_STOP		1U	/* arg must be 0 */
#define CMD_START		2U	/* arg must be 0 */
#define CMD_SET_SPEED		3U	/* arg = cm/s, 0..CMD_SPEED_MAX */
#define CMD_PING		4U	/* arg must be 0 */
#define CMD_SET_MAP		5U	/* arg = movement<<5 | letter index */
#define CMD_RESET		6U	/* arg must be 0: reboot the car */
#define CMD_SPEED_MAX		100U

/*
 *	CMD_SET_MAP carries the barcode letter -> movement mapping that
 *	subsystem3.h says MUST come from the PC before the demo (its built-in
 *	A/B/C/D table is only a test stand-in). Movement is a LINE_CMD_* value
 *	(0..3) and the letter is an index, 0 = 'A' .. 25 = 'Z', so the pair
 *	needs 7 bits and still fits the existing 8-bit arg: the command frame
 *	format does not change. One command sets one movement's letter, so a
 *	full mapping is four commands.
 */
#define CMD_MAP_LETTER_MAX	25U	/* 'Z' */
#define CMD_MAP_MOVEMENT_MAX	3U	/* LINE_CMD_UTURN */
#define CMD_MAP_ARG(mv, li)	((uint8_t)(((mv) << 5) | (li)))
#define CMD_MAP_MOVEMENT(arg)	((uint8_t)(((arg) >> 5) & 0x03U))
#define CMD_MAP_LETTER(arg)	((uint8_t)((arg) & 0x1FU))

typedef enum {
	FRAME_OK = 0,
	FRAME_ERR_NULL,
	FRAME_ERR_LEN,
	FRAME_ERR_VERSION,
	FRAME_ERR_RANGE
} T_FRAME_STATUS;

/* Unpacked snapshot. Each field is limited to the width noted. */
typedef struct {
	uint8_t		seq;		/* 5 bits, wraps */
	uint8_t		vstate;		/* 3 bits */
	uint8_t		mstate;		/* 3 bits, 0..TELEM_MSTATE_MAX */
	uint8_t		faults;		/* 5 bits, TELEM_FAULT_* */
	uint8_t		speed_l;	/* 8 bits, cm/s */
	uint8_t		speed_r;	/* 8 bits, cm/s */
	uint16_t	dist_cm;	/* 16 bits */
	uint8_t		line_err;	/* 7 bits, offset binary, 64 = centre */
	uint8_t		line_ok;	/* 1 bit */
	uint8_t		junction;	/* 1 bit */
	uint8_t		bc_ok;		/* 1 bit */
	uint8_t		bc_cmd;		/* 2 bits */
	uint8_t		bc_char;	/* 5 bits, 0 = none yet, 1..26 = 'A'..'Z' */
	uint8_t		hump_mm;	/* 8 bits, clamped by the sender */
	uint8_t		obs_ok;		/* 1 bit */
	uint16_t	obs_left;	/* 9 bits, cm */
	uint16_t	obs_right;	/* 9 bits, cm */
	uint16_t	obs_width;	/* 9 bits, cm */
	uint32_t	nav_map;	/* 20 bits, S3 mapping word, see above */
	uint8_t		hump_max;	/* 8 bits, mm, highest since boot */
	uint8_t		imu_state;	/* 2 bits, IMU_TERRAIN_* */
	int32_t		enc_l;		/* 20 bits signed, TELEM_ENC_MIN..MAX */
	int32_t		enc_r;		/* 20 bits signed */
	uint8_t		bc_dec_ok;	/* 8 bits, saturating */
	uint8_t		bc_dec_fail;	/* 8 bits, saturating */
	uint8_t		cmd_ok;		/* 8 bits, saturating */
	uint8_t		cmd_rej;	/* 8 bits, saturating */
} T_TELEM;

typedef struct {
	uint8_t		cmd;		/* 4 bits, CMD_* */
	uint8_t		arg;		/* 8 bits */
	uint8_t		seq;		/* 4 bits, wraps */
} T_CMD;

/* Fails with FRAME_ERR_RANGE instead of silently truncating a field. */
T_FRAME_STATUS telem_encode(const T_TELEM *in, uint8_t out[TELEM_LEN]);

/* Rejects a wrong length, unknown version or out-of-range field. */
T_FRAME_STATUS telem_decode(const uint8_t *buf, uint32_t len, T_TELEM *out);

T_FRAME_STATUS cmd_encode(const T_CMD *in, uint8_t out[CMD_LEN]);
T_FRAME_STATUS cmd_decode(const uint8_t *buf, uint32_t len, T_CMD *out);

#endif /* TELEMETRY_FRAME_H */
