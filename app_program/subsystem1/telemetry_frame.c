/*
 *	telemetry_frame.c
 *	Subsystem 1 -- pack and unpack the MQTT frames with explicit shifts and
 *	masks. Bits are written MSB-first, so the byte order on the wire is fixed
 *	and no byte swapping is needed. Never cast a buffer to a struct.
 */

#include "telemetry_frame.h"

#define CMD_CMD_MAX	CMD_PING

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

static int telem_in_range(const T_TELEM *t)
{
	return fits(t->seq, 5U) && fits(t->vstate, 3U) &&
	       (t->mstate <= TELEM_MSTATE_MAX) && fits(t->faults, 5U) &&
	       fits(t->line_err, 7U) && fits(t->line_ok, 1U) &&
	       fits(t->junction, 1U) && fits(t->bc_ok, 1U) &&
	       fits(t->bc_cmd, 2U) && fits(t->obs_ok, 1U) &&
	       fits(t->obs_left, 9U) && fits(t->obs_right, 9U) &&
	       fits(t->obs_width, 9U);
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
	put_bits(out, &pos, 8U, in->hump_mm);
	put_bits(out, &pos, 1U, in->obs_ok);
	put_bits(out, &pos, 9U, in->obs_left);
	put_bits(out, &pos, 9U, in->obs_right);
	put_bits(out, &pos, 9U, in->obs_width);
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
	t.hump_mm = (uint8_t)get_bits(buf, &pos, 8U);
	t.obs_ok = (uint8_t)get_bits(buf, &pos, 1U);
	t.obs_left = (uint16_t)get_bits(buf, &pos, 9U);
	t.obs_right = (uint16_t)get_bits(buf, &pos, 9U);
	t.obs_width = (uint16_t)get_bits(buf, &pos, 9U);
	if (t.mstate > TELEM_MSTATE_MAX) {
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
	return c->arg == 0U;	/* STOP, START, PING carry no argument */
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
