/*
 * test_vc_mission.c
 * PC tests for app_program/vc_mission.c (Vehicle Controller mission
 * logic, CLAUDE.md 1b). Derived tests only: nothing here ran on the car.
 * The mapping used below is a TEST FIXTURE, never a firmware default.
 *
 * Run: sh run_tests.sh
 */

#include <stdio.h>
#include <string.h>
#include "vc_mission.h"

static int g_failed = 0;
static int g_checks = 0;

#define CHECK(id, cond, text) \
    do { \
        g_checks++; \
        if (cond) { printf("[PC] %-8s PASS  %s\n", id, text); } \
        else { g_failed++; printf("[PC] %-8s FAIL  %s\n", id, text); } \
    } while (0)

#define CH(c)   ((uint8_t)((c) - 'A'))

int main(void)
{
    /* fixture: FORWARD = C, U-TURN = D, LEFT = A, RIGHT = B */
    uint8_t      chars[VC_MOVE_COUNT] = { CH('C'), CH('D'), CH('A'), CH('B') };
    uint8_t      dup[VC_MOVE_COUNT]   = { CH('C'), CH('C'), CH('A'), CH('B') };
    uint8_t      big[VC_MOVE_COUNT]   = { 26U, CH('D'), CH('A'), CH('B') };
    uint8_t      edge[VC_MOVE_COUNT]  = { 0U, 25U, 1U, 24U };
    uint8_t      move = 0xFFU;
    uint8_t      flags = 0xFFU;
    uint32_t     word = 0U;
    uint32_t     word2 = 0U;
    vc_mission_t st;
    int          ok;
    unsigned     a, b, c, d;

    /* VC-P01: pack and read back every field, golden value */
    ok = vc_map_pack(chars, &word);
    CHECK("VC-P01", ok && word == (0x100000U | (1U << 15) | (0U << 10)
                                   | (3U << 5) | 2U),
          "pack C,D,A,B -> 0x00108062 (bit-exact)");
    CHECK("VC-P01", vc_map_char(word, VC_MOVE_FORWARD) == CH('C') &&
                    vc_map_char(word, VC_MOVE_UTURN) == CH('D') &&
                    vc_map_char(word, VC_MOVE_LEFT) == CH('A') &&
                    vc_map_char(word, VC_MOVE_RIGHT) == CH('B'),
          "each movement reads back its character");

    /* VC-P02: round trip over every set of 4 distinct chars (step 3) */
    ok = 1;
    for (a = 0U; a <= 25U; a += 3U) {
        for (b = 1U; b <= 25U; b += 3U) {
            for (c = 2U; c <= 25U; c += 3U) {
                for (d = 0U; d <= 25U; d++) {
                    uint8_t in[VC_MOVE_COUNT] = { (uint8_t)a, (uint8_t)b,
                                                  (uint8_t)c, (uint8_t)d };
                    int distinct = a != b && a != c && a != d && b != c &&
                                   b != d && c != d;
                    int packed = vc_map_pack(in, &word2);
                    if (packed != distinct) { ok = 0; }
                    if (packed && (!vc_map_is_valid(word2) ||
                        vc_map_char(word2, 0U) != a ||
                        vc_map_char(word2, 1U) != b ||
                        vc_map_char(word2, 2U) != c ||
                        vc_map_char(word2, 3U) != d)) { ok = 0; }
                }
            }
        }
    }
    CHECK("VC-P02", ok, "pack/unpack round trip; refused exactly when not "
          "distinct");

    /* VC-P03: invalid mappings refused */
    word2 = 0x12345U;
    CHECK("VC-P03", !vc_map_pack(dup, &word2) && word2 == 0x12345U,
          "repeated character refused, output untouched");
    CHECK("VC-P03", !vc_map_pack(big, &word2), "character 26 refused");
    CHECK("VC-P03", vc_map_pack(edge, &word2) && vc_map_is_valid(word2),
          "characters 0 and 25 accepted");
    CHECK("VC-P03", !vc_map_is_valid(word & ~VC_MAP_VALID_BIT),
          "word without the valid bit is not valid");
    CHECK("VC-P03", !vc_map_is_valid(word | 0x80000000U),
          "stray high bit makes the word invalid");
    CHECK("VC-P03", !vc_map_is_valid((word & ~0x1FU) | 30U),
          "field value 30 makes the word invalid");
    CHECK("VC-P03", !vc_map_is_valid(VC_MAP_NONE), "VC_MAP_NONE is invalid");

    /* VC-P04: lookup */
    ok = vc_map_lookup(word, CH('A'), &move) && move == VC_MOVE_LEFT;
    ok = ok && vc_map_lookup(word, CH('D'), &move) && move == VC_MOVE_UTURN;
    ok = ok && !vc_map_lookup(word, CH('Z'), &move);
    ok = ok && !vc_map_lookup(word, VC_CHAR_NONE, &move);
    CHECK("VC-P04", ok, "lookup: mapped found, unmapped / NONE not found");

    /* VC-P05: demo flow (barcode -> junction -> movement) */
    vc_mission_init(&st);
    CHECK("VC-P05", st.pending == VC_CHAR_NONE && st.map_word == VC_MAP_NONE,
          "init: no pending character, no mapping");
    ok = vc_mission_set_mapping(&st, word);
    vc_mission_on_char(&st, CH('B'));
    move = vc_mission_on_junction(&st, &flags);
    CHECK("VC-P05", ok && move == VC_MOVE_RIGHT && flags == 0U &&
                    st.pending == VC_CHAR_NONE,
          "B then junction -> RIGHT, slot cleared");
    move = vc_mission_on_junction(&st, &flags);
    CHECK("VC-P05", move == VC_MOVE_FORWARD && flags == VC_FLAG_NO_PENDING,
          "junction without a barcode -> FORWARD, NO_PENDING");

    /* VC-P06: newer barcode before the junction overwrites */
    vc_mission_on_char(&st, CH('A'));
    vc_mission_on_char(&st, CH('D'));
    move = vc_mission_on_junction(&st, &flags);
    CHECK("VC-P06", move == VC_MOVE_UTURN && flags == 0U,
          "A then D then junction -> U-TURN (latest wins)");
    vc_mission_on_char(&st, 26U);
    CHECK("VC-P06", st.pending == VC_CHAR_NONE, "value 26 ignored");

    /* VC-P07: no mapping yet -> FORWARD, flagged, car keeps going */
    vc_mission_init(&st);
    vc_mission_on_char(&st, CH('A'));
    move = vc_mission_on_junction(&st, &flags);
    CHECK("VC-P07", move == VC_MOVE_FORWARD && flags == VC_FLAG_NO_MAPPING &&
                    st.pending == VC_CHAR_NONE,
          "no mapping -> FORWARD, NO_MAPPING, slot cleared");

    /* VC-P08: unmapped character -> FORWARD, flagged */
    (void)vc_mission_set_mapping(&st, word);
    vc_mission_on_char(&st, CH('Q'));
    move = vc_mission_on_junction(&st, &flags);
    CHECK("VC-P08", move == VC_MOVE_FORWARD && flags == VC_FLAG_UNMAPPED,
          "unmapped Q -> FORWARD, UNMAPPED");

    /* VC-P09: mid-run update is whole-word; a bad update is rejected */
    vc_map_pack(edge, &word2);              /* FORWARD=A U=Z L=B R=Y */
    ok = vc_mission_set_mapping(&st, word2);
    vc_mission_on_char(&st, CH('B'));
    move = vc_mission_on_junction(&st, &flags);
    CHECK("VC-P09", ok && move == VC_MOVE_LEFT && st.map_word == word2,
          "new mapping replaces the old one (B now LEFT)");
    ok = !vc_mission_set_mapping(&st, word2 | 0x80000000U);
    ok = ok && !vc_mission_set_mapping(&st, VC_MAP_NONE);
    CHECK("VC-P09", ok && st.map_word == word2,
          "invalid / empty update rejected, old mapping kept");

    /* VC-P10: NULL safety and names */
    move = vc_mission_on_junction(NULL, &flags);
    CHECK("VC-P10", move == VC_MOVE_FORWARD && flags == VC_FLAG_NO_PENDING &&
                    !vc_map_pack(NULL, &word2) &&
                    !vc_map_lookup(word, CH('A'), NULL),
          "NULL inputs never crash, default FORWARD");
    CHECK("VC-P10", strcmp(vc_move_name(VC_MOVE_UTURN), "U-TURN") == 0 &&
                    strcmp(vc_move_name(4U), "?") == 0,
          "movement names");

    printf("[PC] sizeof vc_mission_t = %u B, mapping word = %u B "
           "(21 bits used, derived)\n",
           (unsigned)sizeof(vc_mission_t), (unsigned)sizeof(uint32_t));
    printf("[PC] %d checks, %d failed\n", g_checks, g_failed);
    return (g_failed == 0) ? 0 : 1;
}
