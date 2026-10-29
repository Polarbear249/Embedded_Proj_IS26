/*
 *	telemetry_frame.h
 *	Subsystem 1 -- compact binary frames for MQTT (design review: minimum-bit
 *	data, explicit shifts and masks, fixed byte order, version byte).
 *
 *	No kernel or network calls here: pure functions that can be unit-tested
 *	on a PC (testing/subsystem1/pc/).
 *
 *	TELEMETRY FRAME v1: 13 bytes, fields packed MSB-first, in this order:
 *	  version 8 | seq 5 | vstate 3 | mstate 3 | faults 5 | speed_l 8 |
 *	  speed_r 8 | dist_cm 16 | line_err 7 | line_ok 1 | junction 1 |
 *	  bc_ok 1 | bc_cmd 2 | hump_mm 8 | obs_ok 1 | obs_left 9 | obs_right 9 |
 *	  obs_width 9                                         = 104 bits
 *
 *	COMMAND FRAME v1: 3 bytes, MSB-first:
 *	  version 8 | cmd 4 | arg 8 | seq 4                   = 24 bits
 */

#ifndef TELEMETRY_FRAME_H
#define TELEMETRY_FRAME_H

#include <stdint.h>

#define TELEM_VERSION		1U
#define TELEM_LEN		13U
#define TELEM_MSTATE_MAX	6U	/* seven motion states: 0..6 */

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
#define CMD_SPEED_MAX		100U

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
	uint8_t		hump_mm;	/* 8 bits, clamped by the sender */
	uint8_t		obs_ok;		/* 1 bit */
	uint16_t	obs_left;	/* 9 bits, cm */
	uint16_t	obs_right;	/* 9 bits, cm */
	uint16_t	obs_width;	/* 9 bits, cm */
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
