/*
 *	test_frame.c  --  PC unit test for telemetry_frame.c (tests S1-P01..)
 *	Build and run:  sh testing/subsystem1/pc/run_tests.sh
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "../../../app_program/subsystem1/telemetry_frame.h"

static int failures;

#define CHECK(id, cond, desc) do { \
	if (cond) { printf("[PC] %s PASS  %s\n", (id), (desc)); } \
	else { printf("[PC] %s FAIL  %s\n", (id), (desc)); failures++; } \
} while (0)

static int same(const T_TELEM *a, const T_TELEM *b)
{
	return memcmp(a, b, sizeof(*a)) == 0;
}

static void random_telem(T_TELEM *t)
{
	memset(t, 0, sizeof(*t));
	t->seq = (uint8_t)(rand() & 0x1F);
	t->vstate = (uint8_t)(rand() & 0x07);
	t->mstate = (uint8_t)(rand() % 7);
	t->faults = (uint8_t)(rand() & 0x1F);
	t->speed_l = (uint8_t)rand();
	t->speed_r = (uint8_t)rand();
	t->dist_cm = (uint16_t)rand();
	t->line_err = (uint8_t)(rand() & 0x7F);
	t->line_ok = (uint8_t)(rand() & 1);
	t->junction = (uint8_t)(rand() & 1);
	t->bc_ok = (uint8_t)(rand() & 1);
	t->bc_cmd = (uint8_t)(rand() & 3);
	t->hump_mm = (uint8_t)rand();
	t->obs_ok = (uint8_t)(rand() & 1);
	t->obs_left = (uint16_t)(rand() & 0x1FF);
	t->obs_right = (uint16_t)(rand() & 0x1FF);
	t->obs_width = (uint16_t)(rand() & 0x1FF);
}

int main(void)
{
	T_TELEM		t, u, big;
	T_CMD		c, d;
	uint8_t		f[TELEM_LEN], g[TELEM_LEN + 1], k[CMD_LEN];
	uint32_t	i;
	int		ok;

	/* S1-P01: size and version byte */
	memset(&t, 0, sizeof(t));
	CHECK("S1-P01", telem_encode(&t, f) == FRAME_OK && TELEM_LEN == 13U &&
	      f[0] == 0x01U && f[11] == 0U && f[12] == 0U,
	      "all-zero frame is 13 bytes, version byte 0x01");

	/* S1-P02: hand-computed golden vector fixes the bit order */
	memset(&t, 0, sizeof(t));
	t.seq = 0x15U; t.vstate = 0x5U; t.mstate = 0x3U; t.faults = 0x11U;
	t.speed_l = 0x12U; t.speed_r = 0x34U; t.dist_cm = 0xABCDU;
	ok = (telem_encode(&t, f) == FRAME_OK) &&
	     f[0] == 0x01U && f[1] == 0xADU && f[2] == 0x71U && f[3] == 0x12U &&
	     f[4] == 0x34U && f[5] == 0xABU && f[6] == 0xCDU;
	CHECK("S1-P02", ok, "golden bytes 01 AD 71 12 34 AB CD (MSB-first, no swapping)");

	/* S1-P03: last field sits in the last bits */
	memset(&t, 0, sizeof(t));
	t.obs_width = 0x1FFU;
	ok = (telem_encode(&t, f) == FRAME_OK) && f[11] == 0x01U && f[12] == 0xFFU;
	CHECK("S1-P03", ok, "obs_width 0x1FF fills the final 9 bits");

	/* S1-P04: maximum value of every field round-trips */
	memset(&big, 0, sizeof(big));
	big.seq = 31; big.vstate = 7; big.mstate = 6; big.faults = 31;
	big.speed_l = 255; big.speed_r = 255; big.dist_cm = 65535;
	big.line_err = 127; big.line_ok = 1; big.junction = 1; big.bc_ok = 1;
	big.bc_cmd = 3; big.hump_mm = 255; big.obs_ok = 1;
	big.obs_left = 511; big.obs_right = 511; big.obs_width = 511;
	ok = telem_encode(&big, f) == FRAME_OK &&
	     telem_decode(f, TELEM_LEN, &u) == FRAME_OK && same(&big, &u);
	CHECK("S1-P04", ok, "maximum of every field round-trips");

	/* S1-P05: 100000 random valid snapshots round-trip */
	srand(12345);
	ok = 1;
	for (i = 0; i < 100000U; i++) {
		random_telem(&t);
		if (telem_encode(&t, f) != FRAME_OK ||
		    telem_decode(f, TELEM_LEN, &u) != FRAME_OK || !same(&t, &u)) {
			ok = 0;
			break;
		}
	}
	CHECK("S1-P05", ok, "100000 random snapshots round-trip");

	/* S1-P06: wrong length rejected */
	telem_encode(&big, f);
	memcpy(g, f, TELEM_LEN); g[TELEM_LEN] = 0;
	ok = telem_decode(f, 0, &u) == FRAME_ERR_LEN &&
	     telem_decode(f, TELEM_LEN - 1U, &u) == FRAME_ERR_LEN &&
	     telem_decode(g, TELEM_LEN + 1U, &u) == FRAME_ERR_LEN;
	CHECK("S1-P06", ok, "length 0, 12 and 14 rejected");

	/* S1-P07: unknown version rejected */
	memcpy(g, f, TELEM_LEN);
	g[0] = 0x00U; ok = telem_decode(g, TELEM_LEN, &u) == FRAME_ERR_VERSION;
	g[0] = 0x02U; ok = ok && telem_decode(g, TELEM_LEN, &u) == FRAME_ERR_VERSION;
	g[0] = 0xFFU; ok = ok && telem_decode(g, TELEM_LEN, &u) == FRAME_ERR_VERSION;
	CHECK("S1-P07", ok, "version 0, 2 and 255 rejected");

	/* S1-P08: out-of-range motion state rejected on decode and encode */
	memcpy(g, f, TELEM_LEN);
	g[2] = (uint8_t)((g[2] & 0x1FU) | (7U << 5));	/* mstate = 7 */
	ok = telem_decode(g, TELEM_LEN, &u) == FRAME_ERR_RANGE;
	t = big; t.mstate = 7;
	ok = ok && telem_encode(&t, f) == FRAME_ERR_RANGE;
	CHECK("S1-P08", ok, "motion state 7 rejected (valid 0..6)");

	/* S1-P09: encoder refuses values wider than their field */
	ok = 1;
	t = big; t.seq = 32;        ok = ok && telem_encode(&t, f) == FRAME_ERR_RANGE;
	t = big; t.vstate = 8;      ok = ok && telem_encode(&t, f) == FRAME_ERR_RANGE;
	t = big; t.faults = 32;     ok = ok && telem_encode(&t, f) == FRAME_ERR_RANGE;
	t = big; t.line_err = 128;  ok = ok && telem_encode(&t, f) == FRAME_ERR_RANGE;
	t = big; t.bc_cmd = 4;      ok = ok && telem_encode(&t, f) == FRAME_ERR_RANGE;
	t = big; t.obs_left = 512;  ok = ok && telem_encode(&t, f) == FRAME_ERR_RANGE;
	t = big; t.obs_width = 512; ok = ok && telem_encode(&t, f) == FRAME_ERR_RANGE;
	CHECK("S1-P09", ok, "too-wide fields are refused, never truncated");

	/* S1-P10: null pointers */
	ok = telem_encode(0, f) == FRAME_ERR_NULL && telem_encode(&t, 0) == FRAME_ERR_NULL &&
	     telem_decode(0, TELEM_LEN, &u) == FRAME_ERR_NULL &&
	     telem_decode(f, TELEM_LEN, 0) == FRAME_ERR_NULL;
	CHECK("S1-P10", ok, "null pointers rejected");

	/* S1-P11: command golden bytes and round trip */
	c.cmd = CMD_SET_SPEED; c.arg = 0x2AU; c.seq = 0x9U;
	ok = cmd_encode(&c, k) == FRAME_OK && k[0] == 0x01U && k[1] == 0x32U &&
	     k[2] == 0xA9U && cmd_decode(k, CMD_LEN, &d) == FRAME_OK &&
	     memcmp(&c, &d, sizeof(c)) == 0;
	CHECK("S1-P11", ok, "SET_SPEED 42 seq 9 encodes to 01 32 A9 and round-trips");

	/* S1-P12: command validation */
	ok = 1;
	c.cmd = CMD_STOP; c.arg = 0; c.seq = 0;
	ok = ok && cmd_encode(&c, k) == FRAME_OK && cmd_decode(k, CMD_LEN, &d) == FRAME_OK;
	c.arg = 1;          ok = ok && cmd_encode(&c, k) == FRAME_ERR_RANGE;   /* STOP takes no arg */
	c.cmd = 0; c.arg = 0; ok = ok && cmd_encode(&c, k) == FRAME_ERR_RANGE; /* unknown cmd */
	c.cmd = 5;          ok = ok && cmd_encode(&c, k) == FRAME_ERR_RANGE;
	c.cmd = CMD_SET_SPEED; c.arg = 101; ok = ok && cmd_encode(&c, k) == FRAME_ERR_RANGE;
	c.arg = 100; c.seq = 16; ok = ok && cmd_encode(&c, k) == FRAME_ERR_RANGE;
	k[0] = 0x01U; k[1] = 0x3FU; k[2] = 0xFFU;                              /* arg 255 */
	ok = ok && cmd_decode(k, CMD_LEN, &d) == FRAME_ERR_RANGE;
	k[0] = 0x02U; ok = ok && cmd_decode(k, CMD_LEN, &d) == FRAME_ERR_VERSION;
	ok = ok && cmd_decode(k, 2U, &d) == FRAME_ERR_LEN && cmd_decode(k, 4U, &d) == FRAME_ERR_LEN;
	CHECK("S1-P12", ok, "bad command, argument, seq, version and length rejected");

	/* S1-P13: random bytes never crash the decoder and never yield a bad value */
	srand(777);
	ok = 1;
	for (i = 0; i < 200000U; i++) {
		uint32_t j;
		for (j = 0; j < TELEM_LEN; j++) { f[j] = (uint8_t)rand(); }
		f[0] = (i & 1U) ? 0x01U : f[0];
		if (telem_decode(f, TELEM_LEN, &u) == FRAME_OK && u.mstate > TELEM_MSTATE_MAX) {
			ok = 0;
			break;
		}
		for (j = 0; j < CMD_LEN; j++) { k[j] = (uint8_t)rand(); }
		(void)cmd_decode(k, CMD_LEN, &d);
	}
	CHECK("S1-P13", ok, "200000 random byte strings decoded without a bad value");

	printf("[PC] SUMMARY %d failed\n", failures);
	return failures == 0 ? 0 : 1;
}
