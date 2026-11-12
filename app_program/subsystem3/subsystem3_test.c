/*
 * subsystem3_test.c
 * On-target tests for Subsystem 3. Built only with: make TEST=1
 * Record results in testing/subsystem3/TEST_LOG.md.
 *
 * S3-U01..U05 are automatic. S3-U06 / S3-U07 are bench tests that need
 * a person at the car; they run only when S3_BENCH_TESTS is 1, so the
 * team's normal TEST=1 run is not held up. S3-U07 is skipped while
 * S3_BARCODE_ENABLE is 0.
 */

#include "subsystem3.h"
#include "../test_report.h"

#if APP_TEST

#define S3_BENCH_TESTS          (1)     // 1 = run S3-U06 / S3-U07

#define S3_T_WINDOW_MS          (200)
#define S3_T_US_MIN             (195000U)   // 200 ms window, -2.5 %
#define S3_T_US_MAX             (215000U)   // +7.5 %: tick rounding
#define S3_T_SEQ_MIN            (18U)       // 20 cycles in 200 ms, +-2
#define S3_T_SEQ_MAX            (22U)
#define S3_T_EXEC_MAX_US        (1000U)     // 10 % of the 10 ms slot

#if S3_BENCH_TESTS
#define S3_BENCH_SENSOR_MS      (15000)
#define S3_BENCH_BARCODE_MS     (30000)
#define S3_BENCH_POLL_MS        (20)
#define S3_BENCH_ALIVE_POLLS    (50)    // status line every 1 s
#endif

EXPORT void
subsystem3_test (void)
{
    uint32_t  t0_us;
    uint32_t  dt_us;
    uint32_t  word0;
    uint32_t  word1;
    uint8_t   seq_diff;
    uint16_t  overruns_boot;
    uint16_t  overruns_new;
    s3_diag_t diag;

    TEST_INIT();

    // Overruns before this point happened during boot (WiFi and USB
    // start-up hold the CPU); S3-U04 judges only the test window.
    overruns_boot = g_s3_diag.overruns;

    // S3-U01: the hardware timer used for barcode widths runs at 1 MHz
    t0_us = in_w(TIMER_TIMERAWL);
    word0 = s3_line_state_read();
    (void)tk_dly_tsk(S3_T_WINDOW_MS);
    dt_us = in_w(TIMER_TIMERAWL) - t0_us;
    word1 = s3_line_state_read();
    tm_printf((UB *)"[TEST] S3 info: 200 ms = %u us\n", (UINT)dt_us);
    TEST_CHECK("S3-U01", (dt_us >= S3_T_US_MIN) && (dt_us <= S3_T_US_MAX),
               "TIMERAWL counts 1 us per tick");

    // S3-U02: line_state is republished every 10 ms (seq wraps at 256)
    // Cast: the mask keeps the 8-bit difference
    seq_diff = (uint8_t)(((word1 >> LINE_IF_SEQ_SHIFT)
                          - (word0 >> LINE_IF_SEQ_SHIFT))
                         & LINE_IF_SEQ_MASK);
    tm_printf((UB *)"[TEST] S3 info: seq advanced %u in 200 ms\n",
              (UINT)seq_diff);
    TEST_CHECK("S3-U02", (seq_diff >= S3_T_SEQ_MIN)
                         && (seq_diff <= S3_T_SEQ_MAX),
               "line_state seq advances once per 10 ms");

    // S3-U03: the task has left INIT
    TEST_CHECK("S3-U03", LINE_MODE_INIT != line_if_mode(word1),
               "line task running (mode is not INIT)");

    // S3-U04 / S3-U05: timing and capture health (NFR11)
    diag = g_s3_diag;
    // Cast: unsigned 16-bit difference, correct across a wrap
    overruns_new = (uint16_t)(diag.overruns - overruns_boot);
    tm_printf((UB *)"[TEST] S3 info: max exec %u us, overruns %u at "
                    "boot, %u in test window\n",
              (UINT)diag.max_exec_us, (UINT)overruns_boot,
              (UINT)overruns_new);
    TEST_CHECK("S3-U04", (diag.max_exec_us < S3_T_EXEC_MAX_US)
                         && (0U == overruns_new),
               "cycle < 1 ms and no missed 10 ms slot");
    TEST_CHECK("S3-U05", 0U == diag.edge_overflows,
               "barcode edge ring never overflowed");

#if S3_BENCH_TESTS
    {
        line_if_state_t now;
        line_if_state_t was;
#if S3_BARCODE_ENABLE
        uint16_t        ok_before;
#endif
        INT             waited;
        INT             polls;

        // S3-U06 (manual): print the start state, every sensor change
        // and a status line each second, for 15 s. Move a black strip
        // under each sensor: 1 = black, 0 = white.
        tm_printf((UB *)"[TEST] S3-U06 move black under C, R, BC "
                        "for 15 s\n");
        line_if_unpack(s3_line_state_read(), &was);
        tm_printf((UB *)"[TEST] S3 start C=%u R=%u BC=%u mode=%u "
                        "err=%u\n",
                  (UINT)was.b_centre_black, (UINT)was.b_right_black,
                  (UINT)was.b_barcode_black, (UINT)was.mode,
                  (UINT)was.error);
        polls = 0;

        for (waited = 0; waited < S3_BENCH_SENSOR_MS;
             waited += S3_BENCH_POLL_MS)
        {
            (void)tk_dly_tsk(S3_BENCH_POLL_MS);
            line_if_unpack(s3_line_state_read(), &now);
            polls++;

            if ((now.b_centre_black != was.b_centre_black)
                || (now.b_right_black != was.b_right_black)
                || (now.b_barcode_black != was.b_barcode_black)
                || (now.mode != was.mode)
                || (polls >= S3_BENCH_ALIVE_POLLS))
            {
                polls = 0;
                tm_printf((UB *)"[TEST] S3 C=%u R=%u BC=%u mode=%u "
                                "err=%u\n",
                          (UINT)now.b_centre_black, (UINT)now.b_right_black,
                          (UINT)now.b_barcode_black, (UINT)now.mode,
                          (UINT)now.error);
            }
            else
            {
                // no change
            }

            was = now;
        }

        tm_printf((UB *)"[TEST] S3-U06 done\n");

#if S3_BARCODE_ENABLE
        // S3-U07 (manual): pass a printed A/B/C/D barcode under the
        // barcode sensor within 30 s; each decode is printed.
        tm_printf((UB *)"[TEST] S3-U07 pass a barcode under BC (30 s)\n");
        ok_before = g_s3_diag.decode_ok;
        line_if_unpack(s3_line_state_read(), &was);

        for (waited = 0; waited < S3_BENCH_BARCODE_MS;
             waited += S3_BENCH_POLL_MS)
        {
            (void)tk_dly_tsk(S3_BENCH_POLL_MS);
            line_if_unpack(s3_line_state_read(), &now);

            if (now.char_seq != was.char_seq)
            {
                tm_printf((UB *)"[TEST] S3 barcode '%c' char=%u "
                                "(ok %u fail %u other %u)\n",
                          (INT)g_s3_diag.last_char, (UINT)now.bc_char,
                          (UINT)g_s3_diag.decode_ok,
                          (UINT)g_s3_diag.decode_fail,
                          (UINT)g_s3_diag.decode_other);
            }
            else
            {
                // no new command
            }

            was = now;
        }

        TEST_CHECK("S3-U07", g_s3_diag.decode_ok != ok_before,
                   "at least one barcode decoded on the bench");
#else
        tm_printf((UB *)"[TEST] S3-U07 SKIPPED: S3_BARCODE_ENABLE "
                        "is 0\n");
#endif
    }
#endif

    TEST_SUMMARY("subsystem3");
}

#endif /* APP_TEST */
