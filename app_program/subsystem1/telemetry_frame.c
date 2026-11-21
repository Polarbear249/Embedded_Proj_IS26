/*
 *	telemetry_frame.c
 *	Subsystem 1 -- pack and unpack the MQTT frames with explicit shifts and
 *	masks. Bits are written MSB-first, so the byte order on the wire is fixed
 *	and no byte swapping is needed. Never cast a buffer to a struct.
 */

#include <string.h>
#include "telemetry_frame.h"

#define CMD_CMD_MAX	CMD_SCAN

static void put_bits(uint8_t *buf, uint32_t *pos, uint32_t width, uint32_t value)
{
	uint32_t	i;
	uint32_t	bit;

	for (i = 0U; i < width; i++) {
		bit = (value >> ((width - 1U) - i)) & 1U;
		buf[*pos >> 3] |= (uint8_t)(bit << (7U - (*pos & 7U)));
		*pos += 1U;
	}
}

static uint32_t get_bits(const uint8_t *buf, uint32_t *pos, uint32_t width)
{
	uint32_t	i;
	uint32_t	value = 0U;
	uint32_t	bit;

	for (i = 0U; i < width; i++) {
		bit = ((uint32_t)buf[*pos >> 3] >> (7U - (*pos & 7U))) & 1U;
		value = (value << 1U) | bit;
		*pos += 1U;
	}
	return value;
}

static int fits(uint32_t value, uint32_t width)
{
	return (value >> width) == 0U;
}

/* Encoder counts: signed, sent as 20-bit two's complement. */
static int enc_fits(int32_t value)
{
	return (value >= TELEM_ENC_MIN) && (value <= TELEM_ENC_MAX);
}

static uint32_t enc_to_bits(int32_t value)
{
	return (uint32_t)value & ((1UL << TELEM_ENC_BITS) - 1UL);
}

static int32_t enc_from_bits(uint32_t bits)
{
	/* Sign-extend bit 19: subtract 2^20 when it is set. */
	if ((bits & (1UL << (TELEM_ENC_BITS - 1U))) != 0U) {
		return (int32_t)bits - (int32_t)(1L << TELEM_ENC_BITS);
	}
	return (int32_t)bits;
}

/* Every slot of the mapping word must be "no letter" (0) or 'A'..'Z'
   (1..26). 27..31 fit in 5 bits but name no letter. */
static int map_valid(uint32_t map)
{
	uint32_t	mv;

	if (!fits(map, TELEM_MAP_SLOTS * TELEM_MAP_SLOT_BITS)) {
		return 0;
	}
	for (mv = 0U; mv < TELEM_MAP_SLOTS; mv++) {
		if (TELEM_MAP_SLOT(map, mv) > TELEM_MAP_LETTER_MAX) {
			return 0;
		}
	}
	return 1;
}

static int telem_in_range(const T_TELEM *t)
{
	return fits(t->seq, 5U) && fits(t->vstate, 3U) &&
	       (t->mstate <= TELEM_MSTATE_MAX) && fits(t->faults, 5U) &&
	       fits(t->line_err, 7U) && fits(t->line_ok, 1U) &&
	       fits(t->junction, 1U) && fits(t->bc_ok, 1U) &&
	       fits(t->bc_cmd, 2U) && fits(t->bc_char, 5U) &&
	       map_valid(t->nav_map) &&
	       fits(t->obs_ok, 1U) &&
	       fits(t->obs_left, 9U) && fits(t->obs_right, 9U) &&
	       fits(t->obs_width, 9U) &&
	       fits(t->imu_state, 2U) &&
	       enc_fits(t->enc_l) && enc_fits(t->enc_r);
}

T_FRAME_STATUS telem_encode(const T_TELEM *in, uint8_t out[TELEM_LEN])
{
	uint32_t	pos = 0U;
	uint32_t	i;

	if ((in == 0) || (out == 0)) {
		return FRAME_ERR_NULL;
	}
	if (!telem_in_range(in)) {
		return FRAME_ERR_RANGE;
	}
	for (i = 0U; i < TELEM_LEN; i++) {
		out[i] = 0U;
	}
	put_bits(out, &pos, 8U, TELEM_VERSION);
	put_bits(out, &pos, 5U, in->seq);
	put_bits(out, &pos, 3U, in->vstate);
	put_bits(out, &pos, 3U, in->mstate);
	put_bits(out, &pos, 5U, in->faults);
	put_bits(out, &pos, 8U, in->speed_l);
	put_bits(out, &pos, 8U, in->speed_r);
	put_bits(out, &pos, 16U, in->dist_cm);
	put_bits(out, &pos, 7U, in->line_err);
	put_bits(out, &pos, 1U, in->line_ok);
	put_bits(out, &pos, 1U, in->junction);
	put_bits(out, &pos, 1U, in->bc_ok);
	put_bits(out, &pos, 2U, in->bc_cmd);
	put_bits(out, &pos, 5U, in->bc_char);
	put_bits(out, &pos, 20U, in->nav_map);
	put_bits(out, &pos, 8U, in->hump_mm);
	put_bits(out, &pos, 1U, in->obs_ok);
	put_bits(out, &pos, 9U, in->obs_left);
	put_bits(out, &pos, 9U, in->obs_right);
	put_bits(out, &pos, 9U, in->obs_width);
	put_bits(out, &pos, 8U, in->hump_max);
	put_bits(out, &pos, 2U, in->imu_state);
	put_bits(out, &pos, TELEM_ENC_BITS, enc_to_bits(in->enc_l));
	put_bits(out, &pos, TELEM_ENC_BITS, enc_to_bits(in->enc_r));
	put_bits(out, &pos, 8U, in->bc_dec_ok);
	put_bits(out, &pos, 8U, in->bc_dec_fail);
	put_bits(out, &pos, 8U, in->cmd_ok);
	put_bits(out, &pos, 8U, in->cmd_rej);
	return FRAME_OK;
}

T_FRAME_STATUS telem_decode(const uint8_t *buf, uint32_t len, T_TELEM *out)
{
	uint32_t	pos = 0U;
	T_TELEM		t;

	if ((buf == 0) || (out == 0)) {
		return FRAME_ERR_NULL;
	}
	if (len != TELEM_LEN) {
		return FRAME_ERR_LEN;
	}
	if (get_bits(buf, &pos, 8U) != TELEM_VERSION) {
		return FRAME_ERR_VERSION;
	}
	memset(&t, 0, sizeof(t));
	t.seq = (uint8_t)get_bits(buf, &pos, 5U);
	t.vstate = (uint8_t)get_bits(buf, &pos, 3U);
	t.mstate = (uint8_t)get_bits(buf, &pos, 3U);
	t.faults = (uint8_t)get_bits(buf, &pos, 5U);
	t.speed_l = (uint8_t)get_bits(buf, &pos, 8U);
	t.speed_r = (uint8_t)get_bits(buf, &pos, 8U);
	t.dist_cm = (uint16_t)get_bits(buf, &pos, 16U);
	t.line_err = (uint8_t)get_bits(buf, &pos, 7U);
	t.line_ok = (uint8_t)get_bits(buf, &pos, 1U);
	t.junction = (uint8_t)get_bits(buf, &pos, 1U);
	t.bc_ok = (uint8_t)get_bits(buf, &pos, 1U);
	t.bc_cmd = (uint8_t)get_bits(buf, &pos, 2U);
	t.bc_char = (uint8_t)get_bits(buf, &pos, 5U);
	t.nav_map = get_bits(buf, &pos, 20U);
	t.hump_mm = (uint8_t)get_bits(buf, &pos, 8U);
	t.obs_ok = (uint8_t)get_bits(buf, &pos, 1U);
	t.obs_left = (uint16_t)get_bits(buf, &pos, 9U);
	t.obs_right = (uint16_t)get_bits(buf, &pos, 9U);
	t.obs_width = (uint16_t)get_bits(buf, &pos, 9U);
	t.hump_max = (uint8_t)get_bits(buf, &pos, 8U);
	t.imu_state = (uint8_t)get_bits(buf, &pos, 2U);
	t.enc_l = enc_from_bits(get_bits(buf, &pos, TELEM_ENC_BITS));
	t.enc_r = enc_from_bits(get_bits(buf, &pos, TELEM_ENC_BITS));
	t.bc_dec_ok = (uint8_t)get_bits(buf, &pos, 8U);
	t.bc_dec_fail = (uint8_t)get_bits(buf, &pos, 8U);
	t.cmd_ok = (uint8_t)get_bits(buf, &pos, 8U);
	t.cmd_rej = (uint8_t)get_bits(buf, &pos, 8U);
	if ((t.mstate > TELEM_MSTATE_MAX) || !map_valid(t.nav_map)) {
		return FRAME_ERR_RANGE;
	}
	*out = t;
	return FRAME_OK;
}

static int cmd_in_range(const T_CMD *c)
{
	if ((c->cmd < CMD_STOP) || (c->cmd > CMD_CMD_MAX)) {
		return 0;
	}
	if (!fits(c->seq, 4U)) {
		return 0;
	}
	if (c->cmd == CMD_SET_SPEED) {
		return c->arg <= CMD_SPEED_MAX;
	}
	if (c->cmd == CMD_MODE) {
		return c->arg <= CMD_MODE_MANUAL;
	}
	if (c->cmd == CMD_DRIVE) {
		return c->arg <= CMD_DRIVE_RIGHT;
	}
	if (c->cmd == CMD_SET_MAP) {
		/* Both halves are checked: a letter index above 'Z' would
		   otherwise reach s3_nav_map_set() as a junk character. */
		return (CMD_MAP_MOVEMENT(c->arg) <= CMD_MAP_MOVEMENT_MAX) &&
		       (CMD_MAP_LETTER(c->arg) <= CMD_MAP_LETTER_MAX);
	}
	return c->arg == 0U;	/* STOP, START, PING, RESET, SCAN: no argument */
}

T_FRAME_STATUS cmd_encode(const T_CMD *in, uint8_t out[CMD_LEN])
{
	uint32_t	pos = 0U;
	uint32_t	i;

	if ((in == 0) || (out == 0)) {
		return FRAME_ERR_NULL;
	}
	if (!cmd_in_range(in)) {
		return FRAME_ERR_RANGE;
	}
	for (i = 0U; i < CMD_LEN; i++) {
		out[i] = 0U;
	}
	put_bits(out, &pos, 8U, CMD_VERSION);
	put_bits(out, &pos, 4U, in->cmd);
	put_bits(out, &pos, 8U, in->arg);
	put_bits(out, &pos, 4U, in->seq);
	return FRAME_OK;
}

T_FRAME_STATUS cmd_decode(const uint8_t *buf, uint32_t len, T_CMD *out)
{
	uint32_t	pos = 0U;
	T_CMD		c;

	if ((buf == 0) || (out == 0)) {
		return FRAME_ERR_NULL;
	}
	if (len != CMD_LEN) {
		return FRAME_ERR_LEN;
	}
	if (get_bits(buf, &pos, 8U) != CMD_VERSION) {
		return FRAME_ERR_VERSION;
	}
	c.cmd = (uint8_t)get_bits(buf, &pos, 4U);
	c.arg = (uint8_t)get_bits(buf, &pos, 8U);
	c.seq = (uint8_t)get_bits(buf, &pos, 4U);
	if (!cmd_in_range(&c)) {
		return FRAME_ERR_RANGE;
	}
	*out = c;
	return FRAME_OK;
}
