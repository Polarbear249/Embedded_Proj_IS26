/*
 * subsystem5_test.c
 * On-target tests for subsystem 5. Built only with: make TEST=1
 * Record results in testing/subsystem5/TEST_LOG.md. S5-U01..U07 are
 * B5's standalone tests in testing/subsystem5/.
 */

#include "subsystem5.h"
#include <stdbool.h>
#include "ultrasonic.h"
#include "../test_report.h"

#if APP_TEST

void
subsystem5_test (void)
{
    uint32_t word;
    bool     b_ok;

    TEST_INIT();

    // S5-U08: IF-09 word round trip (CLAUDE.md section 6, risk R3)
    word = s5_profile_pack(123U, ULTRA_MAX_CM, 1U, S5_KIND_PROFILE,
                           S5_BEST_CENTRE);
    b_ok = (123U == S5_P_LEFT(word)) && (ULTRA_MAX_CM == S5_P_CENTRE(word))
           && (1U == S5_P_RIGHT(word))
           && (S5_KIND_PROFILE == S5_P_KIND(word))
           && (S5_BEST_CENTRE == S5_P_BEST(word))
           && (0U == (word >> 31));
    TEST_CHECK("S5-U08", b_ok, "IF-09 profile word packs and unpacks");

    // S5-U09: out-of-range inputs clamp/mask, never spill into neighbours
    word = s5_profile_pack(0xFFFFU, 0U, 0xFFFFU, 0xFFU, 0xFFU);
    b_ok = (S5_DIST_MASK == S5_P_LEFT(word)) && (0U == S5_P_CENTRE(word))
           && (S5_DIST_MASK == S5_P_RIGHT(word)) && (3U == S5_P_KIND(word))
           && (3U == S5_P_BEST(word)) && (0U == (word >> 31));
    TEST_CHECK("S5-U09", b_ok, "IF-09 fields clamp to 9 bits, no spill");

    TEST_SUMMARY("subsystem5");
}

#endif // APP_TEST
