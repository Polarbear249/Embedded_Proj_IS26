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
	t->bc_char = (uint8_t)(rand() % 27);	/* 0 = none, 1..26 = A..Z */
	{
		uint32_t mv;
		for (mv = 0U; mv < TELEM_MAP_SLOTS; mv++) {
			t->nav_map |= (uint32_t)(rand() % 27) << (mv * 5U);
		}
	}
	t->hump_max = (uint8_t)rand();
	t->imu_state = (uint8_t)(rand() & 3);
	t->enc_l = (int32_t)(rand() % 1048576) - 524288;
	t->enc_r = (int32_t)(rand() % 1048576) - 524288;
	t->bc_dec_ok = (uint8_t)rand();
	t->bc_dec_fail = (uint8_t)rand();
	t->cmd_ok = (uint8_t)rand();
	t->cmd_rej = (uint8_t)rand();
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
	CHECK("S1-P01", telem_encode(&t, f) == FRAME_OK && TELEM_LEN == 27U &&
	      f[0] == 0x04U && f[25] == 0U && f[26] == 0U,
	      "all-zero frame is 27 bytes, version byte 0x04");

	/* S1-P02: hand-computed golden vector fixes the bit order */
	memset(&t, 0, sizeof(t));
	t.seq = 0x15U; t.vstate = 0x5U; t.mstate = 0x3U; t.faults = 0x11U;
	t.speed_l = 0x12U; t.speed_r = 0x34U; t.dist_cm = 0xABCDU;
	ok = (telem_encode(&t, f) == FRAME_OK) &&
	     f[0] == 0x04U && f[1] == 0xADU && f[2] == 0x71U && f[3] == 0x12U &&
	     f[4] == 0x34U && f[5] == 0xABU && f[6] == 0xCDU;
	CHECK("S1-P02", ok, "golden bytes 04 AD 71 12 34 AB CD (MSB-first, no swapping)");

	/* S1-P03: last field sits at the end of the payload. In v4 the
	   payload is 211 bits inside 27 bytes: the last field, cmd_rej,
	   occupies bits 203..210 and the last 5 bits are zero padding. */
	memset(&t, 0, sizeof(t));
	t.cmd_rej = 0xFFU;
	ok = (telem_encode(&t, f) == FRAME_OK) && f[25] == 0x1FU && f[26] == 0xE0U;
	CHECK("S1-P03", ok, "cmd_rej 0xFF fills bits 203..210, 5 pad bits zero");

	/* S1-P04: maximum value of every field round-trips */
	memset(&big, 0, sizeof(big));
	big.seq = 31; big.vstate = 7; big.mstate = 6; big.faults = 31;
	big.speed_l = 255; big.speed_r = 255; big.dist_cm = 65535;
	big.line_err = 127; big.line_ok = 1; big.junction = 1; big.bc_ok = 1;
	big.bc_cmd = 3; big.bc_char = 26; big.hump_mm = 255; big.obs_ok = 1;
	big.nav_map = 0xD6B5AUL;	/* 'Z' in all four slots */
	big.hump_max = 255; big.imu_state = 3;
	big.enc_l = TELEM_ENC_MAX; big.enc_r = TELEM_ENC_MIN;
	big.bc_dec_ok = 255; big.bc_dec_fail = 255; big.cmd_ok = 255; big.cmd_rej = 255;
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
	CHECK("S1-P06", ok, "length 0, 26 and 28 rejected");

	/* S1-P07: unknown version rejected */
	memcpy(g, f, TELEM_LEN);
	g[0] = 0x00U; ok = telem_decode(g, TELEM_LEN, &u) == FRAME_ERR_VERSION;
	g[0] = 0x01U; ok = ok && telem_decode(g, TELEM_LEN, &u) == FRAME_ERR_VERSION;
	g[0] = 0x02U; ok = ok && telem_decode(g, TELEM_LEN, &u) == FRAME_ERR_VERSION;
	g[0] = 0x03U; ok = ok && telem_decode(g, TELEM_LEN, &u) == FRAME_ERR_VERSION;
	g[0] = 0xFFU; ok = ok && telem_decode(g, TELEM_LEN, &u) == FRAME_ERR_VERSION;
	CHECK("S1-P07", ok, "version 0..3 (older frames) and 255 rejected");

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
	c.cmd = 10;         ok = ok && cmd_encode(&c, k) == FRAME_ERR_RANGE;
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
		f[0] = (i & 1U) ? TELEM_VERSION : f[0];
		if (telem_decode(f, TELEM_LEN, &u) == FRAME_OK &&
		    ((u.mstate > TELEM_MSTATE_MAX) ||
		     (TELEM_MAP_SLOT(u.nav_map, 0U) > TELEM_MAP_LETTER_MAX) ||
		     (TELEM_MAP_SLOT(u.nav_map, 1U) > TELEM_MAP_LETTER_MAX) ||
		     (TELEM_MAP_SLOT(u.nav_map, 2U) > TELEM_MAP_LETTER_MAX) ||
		     (TELEM_MAP_SLOT(u.nav_map, 3U) > TELEM_MAP_LETTER_MAX))) {
			ok = 0;
			break;
		}
		for (j = 0; j < CMD_LEN; j++) { k[j] = (uint8_t)rand(); }
		(void)cmd_decode(k, CMD_LEN, &d);
	}
	CHECK("S1-P13", ok, "200000 random byte strings decoded without a bad value");

	/* S1-P14: bc_char (v2) carries every letter plus the "none yet"
	   value, and sits in its own bits without disturbing bc_cmd. */
	ok = 1;
	for (i = 0; i <= 26U; i++) {
		memset(&t, 0, sizeof(t));
		t.bc_ok = 1U; t.bc_cmd = 2U; t.bc_char = (uint8_t)i;
		if (telem_encode(&t, f) != FRAME_OK ||
		    telem_decode(f, TELEM_LEN, &u) != FRAME_OK ||
		    u.bc_char != (uint8_t)i || u.bc_cmd != 2U) {
			ok = 0;
			break;
		}
	}
	memset(&t, 0, sizeof(t));
	t.bc_char = 32U;	/* 6 bits: must be refused, never truncated */
	ok = ok && telem_encode(&t, f) == FRAME_ERR_RANGE;
	CHECK("S1-P14", ok, "bc_char 0..26 round-trips, 32 refused not truncated");

	/* S1-P15: CMD_SET_MAP packs movement and letter into the 8-bit arg,
	   so the command frame stays 3 bytes, and junk halves are refused. */
	ok = 1;
	{
		uint8_t mv, li;
		for (mv = 0; mv <= CMD_MAP_MOVEMENT_MAX; mv++) {
			for (li = 0; li <= CMD_MAP_LETTER_MAX; li++) {
				c.cmd = CMD_SET_MAP;
				c.arg = CMD_MAP_ARG(mv, li);
				c.seq = 3U;
				if (cmd_encode(&c, k) != FRAME_OK ||
				    cmd_decode(k, CMD_LEN, &d) != FRAME_OK ||
				    d.cmd != CMD_SET_MAP ||
				    CMD_MAP_MOVEMENT(d.arg) != mv ||
				    CMD_MAP_LETTER(d.arg) != li) {
					ok = 0;
				}
			}
		}
	}
	c.cmd = CMD_SET_MAP; c.seq = 0U;
	c.arg = CMD_MAP_ARG(0U, 26U);	/* letter index past 'Z' */
	ok = ok && cmd_encode(&c, k) == FRAME_ERR_RANGE;
	c.arg = 0xFFU;			/* both halves out of range */
	ok = ok && cmd_encode(&c, k) == FRAME_ERR_RANGE;
	CHECK("S1-P15", ok, "SET_MAP packs 4x26 pairs in 3 bytes, bad letter refused");

	/* S1-P16: CMD_RESET carries no argument. A reset is the one command
	   that cannot be undone from the dashboard, so a non-zero arg must be
	   refused rather than treated as "reset anyway". */
	c.cmd = CMD_RESET; c.arg = 0U; c.seq = 5U;
	ok = cmd_encode(&c, k) == FRAME_OK &&
	     cmd_decode(k, CMD_LEN, &d) == FRAME_OK &&
	     d.cmd == CMD_RESET && d.arg == 0U && d.seq == 5U;
	c.arg = 1U;   ok = ok && cmd_encode(&c, k) == FRAME_ERR_RANGE;
	c.arg = 0xFFU; ok = ok && cmd_encode(&c, k) == FRAME_ERR_RANGE;
	CHECK("S1-P16", ok, "RESET with arg 0 round-trips, any other arg refused");

	/* S1-P17: nav_map (v3) carries the car's real mapping. The stand-in
	   A/B/C/D word (0x20C41) has a hand-computed frame; every slot value
	   1..26 round-trips in every slot; a slot of 27 (no such letter) and
	   a bit above the 20-bit field are refused on encode, and a frame
	   carrying slot 27 is refused on decode. */
	memset(&t, 0, sizeof(t));
	t.nav_map = 0x20C41UL;
	ok = telem_encode(&t, f) == FRAME_OK &&
	     f[0] == TELEM_VERSION && f[9] == 0x10U && f[10] == 0x62U && f[11] == 0x08U;
	{
		uint32_t mv, li;
		for (mv = 0U; mv < TELEM_MAP_SLOTS && ok; mv++) {
			for (li = 0U; li <= TELEM_MAP_LETTER_MAX; li++) {
				memset(&t, 0, sizeof(t));
				t.nav_map = li << (mv * 5U);
				if (telem_encode(&t, f) != FRAME_OK ||
				    telem_decode(f, TELEM_LEN, &u) != FRAME_OK ||
				    TELEM_MAP_SLOT(u.nav_map, mv) != li) {
					ok = 0;
					break;
				}
			}
		}
	}
	memset(&t, 0, sizeof(t));
	t.nav_map = 27UL << 10;		/* STRAIGHT slot = 27 */
	ok = ok && telem_encode(&t, f) == FRAME_ERR_RANGE;
	t.nav_map = 1UL << 20;			/* outside the 20-bit field */
	ok = ok && telem_encode(&t, f) == FRAME_ERR_RANGE;
	memset(&t, 0, sizeof(t));
	(void)telem_encode(&t, f);
	f[10] |= 0x1BU;			/* RIGHT slot (bits 83..87) = 27 */
	ok = ok && telem_decode(f, TELEM_LEN, &u) == FRAME_ERR_RANGE;
	CHECK("S1-P17", ok, "nav_map stand-in golden, 1..26 in every slot, 27 refused");

	/* S1-P18: encoder counts are 20-bit two's complement. -1 is twenty
	   ones (hand-computed frame), the minimum is a single 1 followed by
	   zeros, both ends round-trip with their sign, and one past either
	   end is refused rather than wrapped to the other sign. */
	memset(&t, 0, sizeof(t));
	t.enc_l = -1;
	ok = telem_encode(&t, f) == FRAME_OK &&
	     f[17] == 0x1FU && f[18] == 0xFFU && f[19] == 0xFEU && f[20] == 0x00U &&
	     telem_decode(f, TELEM_LEN, &u) == FRAME_OK && u.enc_l == -1;
	memset(&t, 0, sizeof(t));
	t.enc_r = TELEM_ENC_MIN;
	ok = ok && telem_encode(&t, f) == FRAME_OK && f[19] == 0x01U && f[20] == 0x00U &&
	     telem_decode(f, TELEM_LEN, &u) == FRAME_OK && u.enc_r == TELEM_ENC_MIN;
	memset(&t, 0, sizeof(t));
	t.enc_l = TELEM_ENC_MAX; t.enc_r = TELEM_ENC_MIN;
	ok = ok && telem_encode(&t, f) == FRAME_OK &&
	     telem_decode(f, TELEM_LEN, &u) == FRAME_OK &&
	     u.enc_l == TELEM_ENC_MAX && u.enc_r == TELEM_ENC_MIN;
	t.enc_l = TELEM_ENC_MAX + 1;
	ok = ok && telem_encode(&t, f) == FRAME_ERR_RANGE;
	t.enc_l = 0; t.enc_r = TELEM_ENC_MIN - 1;
	ok = ok && telem_encode(&t, f) == FRAME_ERR_RANGE;
	CHECK("S1-P18", ok, "encoders -1 and min golden, +-limits keep sign, past either end refused");

	/* S1-P19: every v4 field at once, against a whole hand-computed frame:
	   highest hump 30, IMU active (2), encoders 1234 / -56, barcode reads
	   7 ok / 2 failed, commands 9 accepted / 1 rejected. */
	{
		static const uint8_t want[TELEM_LEN] = {
			0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0FU, 0x40U, 0x09U, 0xA5U,
			0xFFU, 0xF9U, 0x00U, 0xE0U, 0x41U, 0x20U, 0x20U
		};
		memset(&t, 0, sizeof(t));
		t.hump_max = 30U; t.imu_state = 2U; t.enc_l = 1234; t.enc_r = -56;
		t.bc_dec_ok = 7U; t.bc_dec_fail = 2U; t.cmd_ok = 9U; t.cmd_rej = 1U;
		ok = telem_encode(&t, f) == FRAME_OK && memcmp(f, want, TELEM_LEN) == 0 &&
		     telem_decode(f, TELEM_LEN, &u) == FRAME_OK && same(&t, &u);
	}
	CHECK("S1-P19", ok, "all v4 fields together match a hand-computed frame");

	/* S1-P20: MODE takes only AUTO (0) or MANUAL (1); DRIVE only STOP,
	   FWD, BACK, LEFT, RIGHT (0..4). Both round-trip with their seq, and
	   one past the end is refused, so a garbled arg cannot become a
	   movement the car was never asked for. Golden: MODE MANUAL seq 2
	   is 01 70 12, DRIVE FWD seq 3 is 01 80 13. */
	ok = 1;
	{
		uint8_t a;
		for (a = 0U; a <= CMD_MODE_MANUAL; a++) {
			c.cmd = CMD_MODE; c.arg = a; c.seq = 4U;
			ok = ok && cmd_encode(&c, k) == FRAME_OK &&
			     cmd_decode(k, CMD_LEN, &d) == FRAME_OK &&
			     d.cmd == CMD_MODE && d.arg == a && d.seq == 4U;
		}
		for (a = 0U; a <= CMD_DRIVE_RIGHT; a++) {
			c.cmd = CMD_DRIVE; c.arg = a; c.seq = 11U;
			ok = ok && cmd_encode(&c, k) == FRAME_OK &&
			     cmd_decode(k, CMD_LEN, &d) == FRAME_OK &&
			     d.cmd == CMD_DRIVE && d.arg == a && d.seq == 11U;
		}
	}
	c.cmd = CMD_MODE;  c.arg = 2U; ok = ok && cmd_encode(&c, k) == FRAME_ERR_RANGE;
	c.cmd = CMD_DRIVE; c.arg = 5U; ok = ok && cmd_encode(&c, k) == FRAME_ERR_RANGE;
	c.cmd = CMD_MODE;  c.arg = CMD_MODE_MANUAL; c.seq = 2U;
	ok = ok && cmd_encode(&c, k) == FRAME_OK &&
	     k[0] == 0x01U && k[1] == 0x70U && k[2] == 0x12U;
	c.cmd = CMD_DRIVE; c.arg = CMD_DRIVE_FWD; c.seq = 3U;
	ok = ok && cmd_encode(&c, k) == FRAME_OK &&
	     k[0] == 0x01U && k[1] == 0x80U && k[2] == 0x13U;
	CHECK("S1-P20", ok, "MODE 0..1 and DRIVE 0..4 round-trip, one past refused, golden bytes");

	/* S1-P21: SCAN carries no argument; golden SCAN seq 6 is 01 90 06. */
	c.cmd = CMD_SCAN; c.arg = 0U; c.seq = 6U;
	ok = cmd_encode(&c, k) == FRAME_OK &&
	     k[0] == 0x01U && k[1] == 0x90U && k[2] == 0x06U &&
	     cmd_decode(k, CMD_LEN, &d) == FRAME_OK && d.cmd == CMD_SCAN;
	c.arg = 1U; ok = ok && cmd_encode(&c, k) == FRAME_ERR_RANGE;
	CHECK("S1-P21", ok, "SCAN with arg 0 encodes to 01 90 06, any other arg refused");

	printf("[PC] SUMMARY %d failed\n", failures);
	return failures == 0 ? 0 : 1;
}
