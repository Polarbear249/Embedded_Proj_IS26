/*
 * test_s3.c
 * PC unit tests for Subsystem 3's hardware-independent modules:
 *   barcode_code39.c   Code 39 decoder
 *   barcode_capture.c  edges -> elements -> symbol
 *   line_logic.c       line position, junction, line-lost search
 *   line_if.c          IF-06 line_state word packing
 * Run with ./run_tests.sh (see testing/subsystem3/TEST_LOG.md).
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "barcode_code39.h"
#include "barcode_capture.h"
#include "line_logic.h"
#include "line_drive.h"
#include "barcode_nav.h"
#include "line_if.h"

static int g_checks = 0;
static int g_failed = 0;

#define CHECK(id, cond, text)                                          \
    do {                                                               \
        g_checks++;                                                    \
        if (cond) {                                                    \
            printf("[PC] %-8s PASS  %s\n", (id), (text));              \
        } else {                                                       \
            g_failed++;                                                \
            printf("[PC] %-8s FAIL  %s  (line %d)\n", (id), (text),   \
                   __LINE__);                                          \
        }                                                              \
    } while (0)

/* ------------------------------------------------------------------ */
/* Helpers                                                             */
/* ------------------------------------------------------------------ */

static const char g_all_data[] =
    "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ-. $/+%";

/* Deterministic PRNG so every run is identical */
static uint32_t g_rng = 12345U;
static uint32_t rng_next(void)
{
    g_rng = g_rng * 1103515245U + 12345U;
    return (g_rng >> 8) & 0xFFFFFFU;
}
/* uniform in [-pct, +pct] percent, as a factor x1000 */
static int32_t rng_pct_milli(int32_t pct)
{
    return 1000 + (int32_t)(rng_next() % (uint32_t)(2 * pct * 10 + 1))
           - pct * 10;
}

/* Element widths (in modules: 1 = narrow, 3 = wide) for "*c*" */
static int symbol_modules(char c, uint8_t out[BC39_SYMBOL_ELEMENTS])
{
    const char seq[3] = {'*', c, '*'};
    int pos = 0;
    for (int k = 0; k < 3; k++) {
        uint16_t pat = bc39_pattern_of(seq[k]);
        if (pat == 0U) {
            return 0;
        }
        for (int i = 8; i >= 0; i--) {
            out[pos++] = ((pat >> i) & 1U) ? 3U : 1U;
        }
        if (k < 2) {
            out[pos++] = 1U;            /* inter-character gap */
        }
    }
    return pos == (int)BC39_SYMBOL_ELEMENTS;
}

/* ------------------------------------------------------------------ */
/* Code 39 decoder                                                     */
/* ------------------------------------------------------------------ */

static void test_table(void)
{
    int ok_bits = 1, ok_unique = 1, ok_split = 1;
    const char *all = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ-. $/+%*";
    size_t n = strlen(all);

    for (size_t i = 0; i < n; i++) {
        uint16_t p = bc39_pattern_of(all[i]);
        int wide = 0, wide_bars = 0;
        for (int b = 0; b < 9; b++) {
            if ((p >> b) & 1U) {
                wide++;
                if ((b % 2) == 0) {     /* bits 8,6,4,2,0 are bars */
                    wide_bars++;
                }
            }
        }
        if (wide != 3) ok_bits = 0;
        /* Code 39: 2 wide bars + 1 wide space, or ($ / + %) 3 spaces */
        if (!((wide_bars == 2) || (wide_bars == 0))) ok_split = 0;
        for (size_t j = 0; j < i; j++) {
            if (bc39_pattern_of(all[j]) == p) ok_unique = 0;
        }
    }
    CHECK("S3-P01", ok_bits, "all 44 patterns have exactly 3 wide");
    CHECK("S3-P01", ok_unique, "all 44 patterns are distinct");
    CHECK("S3-P01", ok_split, "wide elements split 2 bars+1 space or "
                              "3 spaces");
    CHECK("S3-P01", bc39_pattern_of('a') == 0U,
          "lower case is not Code 39");
}

/* Module strings (1 = black module) -> 9-bit wide pattern */
static uint16_t modules_to_pattern(const char *mod)
{
    uint16_t pat = 0U;
    size_t i = 0, n = strlen(mod);
    while (i < n) {
        size_t run = 1;
        while (i + run < n && mod[i + run] == mod[i]) run++;
        pat = (uint16_t)((pat << 1) | (run >= 2U ? 1U : 0U));
        i += run;
    }
    return pat;
}

static void test_reference_patterns(void)
{
    CHECK("S3-P02", modules_to_pattern("110101001011") ==
          bc39_pattern_of('A'), "A matches module string");
    CHECK("S3-P02", modules_to_pattern("101101001011") ==
          bc39_pattern_of('B'), "B matches module string");
    CHECK("S3-P02", modules_to_pattern("110110100101") ==
          bc39_pattern_of('C'), "C matches module string");
    CHECK("S3-P02", modules_to_pattern("101011001011") ==
          bc39_pattern_of('D'), "D matches module string");
}

/* Bars of docs/Barcode Sample.pdf page 1 (x, width, PDF units x100) */
static const uint32_t g_pdf_a[15][2] = {
    {6581, 9058}, {42813, 9058}, {60929, 27174}, {97161, 27174},
    {133394, 9058}, {151510, 27174}, {187742, 9058}, {205858, 9058},
    {242090, 9058}, {260206, 27174}, {296439, 9058}, {332671, 9058},
    {350787, 27174}, {387019, 27174}, {423252, 9058}
};
/* page 3, "*Z*" */
static const uint32_t g_pdf_z[15][2] = {
    {6581, 9058}, {42813, 9058}, {60929, 27174}, {97161, 27174},
    {133394, 9058}, {151510, 9058}, {187742, 27174}, {223974, 27174},
    {260206, 9058}, {278323, 9058}, {296439, 9058}, {332671, 9058},
    {350787, 27174}, {387019, 27174}, {423252, 9058}
};

static void pdf_widths(const uint32_t bars[15][2],
                       uint16_t out[BC39_SYMBOL_ELEMENTS], int reverse)
{
    uint16_t tmp[BC39_SYMBOL_ELEMENTS];
    for (int i = 0; i < 15; i++) {
        tmp[2 * i] = (uint16_t)bars[i][1];
        if (i < 14) {
            tmp[2 * i + 1] = (uint16_t)(bars[i + 1][0]
                                        - (bars[i][0] + bars[i][1]));
        }
    }
    for (int i = 0; i < (int)BC39_SYMBOL_ELEMENTS; i++) {
        out[i] = reverse ? tmp[BC39_SYMBOL_ELEMENTS - 1 - i] : tmp[i];
    }
}

static void test_pdf_sample(void)
{
    uint16_t w[BC39_SYMBOL_ELEMENTS];
    char c = 0;
    bool rev = true;

    pdf_widths(g_pdf_a, w, 0);
    CHECK("S3-P03", bc39_decode(w, 29, &c, &rev) == BC39_OK && c == 'A'
          && !rev, "PDF sample *A* forwards -> 'A'");
    pdf_widths(g_pdf_a, w, 1);
    CHECK("S3-P03", bc39_decode(w, 29, &c, &rev) == BC39_OK && c == 'A'
          && rev, "PDF sample *A* backwards -> 'A', reversed");
    pdf_widths(g_pdf_z, w, 0);
    CHECK("S3-P03", bc39_decode(w, 29, &c, &rev) == BC39_OK && c == 'Z'
          && !rev, "PDF sample *Z* forwards -> 'Z'");
    pdf_widths(g_pdf_z, w, 1);
    CHECK("S3-P03", bc39_decode(w, 29, &c, &rev) == BC39_OK && c == 'Z'
          && rev, "PDF sample *Z* backwards -> 'Z', reversed");
}

static void test_random_decode(void)
{
    uint8_t mods[BC39_SYMBOL_ELEMENTS];
    uint16_t w[BC39_SYMBOL_ELEMENTS];
    int trials = 0, good = 0;
    const int ratios_x10[3] = {30, 25, 22};

    for (int r = 0; r < 3; r++) {
        for (size_t k = 0; k < strlen(g_all_data); k++) {
            for (int t = 0; t < 200; t++) {
                char c = g_all_data[k], got = 0;
                bool rev = false;
                int reverse = t & 1;
                /* narrow width 20..2000 units, speed drift +-20 % */
                int32_t narrow = 20 + (int32_t)(rng_next() % 1981U);
                int32_t drift = rng_pct_milli(20);
                symbol_modules(c, mods);
                for (int i = 0; i < 29; i++) {
                    int32_t base = (mods[i] == 3U)
                                   ? narrow * ratios_x10[r] / 10 : narrow;
                    /* speed changes linearly across the symbol */
                    int32_t spd = 1000 + (drift - 1000) * i / 28;
                    int32_t v = base * spd / 1000 * rng_pct_milli(15)
                                / 1000;
                    int idx = reverse ? 28 - i : i;
                    w[idx] = (uint16_t)(v < 1 ? 1 : v);
                }
                trials++;
                if (bc39_decode(w, 29, &got, &rev) == BC39_OK && got == c
                    && rev == (reverse != 0)) {
                    good++;
                }
            }
        }
    }
    printf("[PC]          random decodes: %d / %d correct\n", good,
           trials);
    CHECK("S3-P04", good == trials, "every char, ratio 3.0/2.5/2.2, "
          "+-15 % jitter, +-20 % speed drift, both directions");
}

static void test_decode_errors(void)
{
    uint16_t w[BC39_SYMBOL_ELEMENTS];
    char c = 0;
    bool rev = false;
    uint8_t mods[BC39_SYMBOL_ELEMENTS];

    for (int i = 0; i < 29; i++) w[i] = 100U;
    CHECK("S3-P05", bc39_decode(w, 29, &c, &rev) == BC39_ERR_RATIO,
          "all elements equal -> ERR_RATIO");
    CHECK("S3-P05", bc39_decode(w, 28, &c, &rev) == BC39_ERR_LENGTH,
          "28 elements -> ERR_LENGTH");
    CHECK("S3-P05", bc39_decode(NULL, 29, &c, &rev) == BC39_ERR_NULL,
          "NULL widths -> ERR_NULL");

    /* "*A*" with the data char replaced by '*' -> frame error */
    symbol_modules('*', mods);
    for (int i = 0; i < 29; i++) w[i] = (uint16_t)(mods[i] * 100U);
    CHECK("S3-P05", bc39_decode(w, 29, &c, &rev) == BC39_ERR_FRAME,
          "*** -> ERR_FRAME");

    /* 4 wide elements in the first char: not a Code 39 pattern */
    symbol_modules('A', mods);
    for (int i = 0; i < 29; i++) w[i] = (uint16_t)(mods[i] * 100U);
    w[0] = 300U; w[1] = 300U; w[2] = 300U; w[3] = 300U;
    CHECK("S3-P05", bc39_decode(w, 29, &c, &rev) != BC39_OK,
          "corrupted first char is rejected");
}

/* ------------------------------------------------------------------ */
/* Capture: edges -> symbol                                            */
/* ------------------------------------------------------------------ */

typedef struct {
    bc_capture_t cap;
    uint32_t     now;           /* simulated microsecond clock */
    uint32_t     next_poll;
    int          decoded, failed;
    char         last;
} sim_t;

static void sim_init(sim_t *s, uint32_t start)
{
    s->now = start;
    s->next_poll = start + 10000U;
    s->decoded = s->failed = 0;
    s->last = 0;
    bc_capture_init(&s->cap, false, s->now);
}

/* Advance time to 'until', polling every 10 ms like the task */
static void sim_run_to(sim_t *s, uint32_t until)
{
    while ((int32_t)(until - s->next_poll) >= 0) {
        char c = 0;
        bc_cap_result_t r = bc_capture_poll(&s->cap, s->next_poll, &c);
        if (r == BC_CAP_DECODED) { s->decoded++; s->last = c; }
        if (r == BC_CAP_FAILED)  { s->failed++; }
        s->next_poll += 10000U;
    }
    s->now = until;
}

static void sim_edge(sim_t *s, uint32_t dt, bool black)
{
    sim_run_to(s, s->now + dt);
    bc_capture_edge(&s->cap, black, s->now & BC_CAP_TIME_MASK);
}

/* Comparator burst measured on the car (2026-11-01): after the real
   edge, the output flips several more times, 0..64 us apart. */
static const uint32_t burst_gaps[] = {8U, 0U, 16U, 0U, 24U, 8U, 0U, 64U};
#define BURST_N (sizeof(burst_gaps) / sizeof(burst_gaps[0]))

/* Drive a symbol past the sensor at speed_mm_s, narrow = 3 mm.
   chatter 1: insert a 40 us spike inside every element.
   chatter 2: a burst of 8 flips (0..64 us) at every edge, as measured. */
static void sim_symbol(sim_t *s, char c, uint32_t speed_mm_s,
                       int reverse, int chatter)
{
    uint8_t mods[BC39_SYMBOL_ELEMENTS];
    uint32_t narrow_us = 3U * 1000000U / speed_mm_s;
    symbol_modules(c, mods);
    for (int i = 0; i < 29; i++) {
        int idx = reverse ? 28 - i : i;
        bool black = ((i % 2) == 0);
        uint32_t dur = mods[idx] * narrow_us;
        uint32_t jit = (uint32_t)((int64_t)dur * rng_pct_milli(10) / 1000);
        sim_edge(s, 0U, black);                  /* element starts */
        if (chatter == 2) {
            uint32_t used = 0U;
            for (size_t b = 0; b < BURST_N; b++) {
                /* odd flips go back to the old level, even ones to the
                   new: the burst ends on the new level */
                sim_edge(s, burst_gaps[b], (b % 2U == 0U) ? !black : black);
                used += burst_gaps[b];
            }
            sim_run_to(s, s->now + (jit - used));
        } else if (chatter && jit > 200U) {
            sim_edge(s, jit / 2U, !black);       /* spike ... */
            sim_edge(s, 40U, black);             /* ... 40 us later */
            sim_run_to(s, s->now + (jit - jit / 2U - 40U));
        } else {
            sim_run_to(s, s->now + jit);
        }
    }
    sim_edge(s, 0U, false);                      /* back to white */
    sim_run_to(s, s->now + 300000U);             /* quiet 300 ms */
}

static void test_capture(void)
{
    sim_t s;
    const uint32_t speeds[] = {50U, 100U, 200U, 400U, 700U, 1000U};
    int ok = 1;
    char cmds[] = "ABCD";

    /* Start close to the 31-bit wrap so the wrap is crossed */
    for (size_t v = 0; v < sizeof(speeds) / sizeof(speeds[0]); v++) {
        for (int k = 0; k < 4; k++) {
            for (int dir = 0; dir < 2; dir++) {
                sim_init(&s, 0x7FFFFFFFU - 100000U);
                sim_run_to(&s, s.now + 50000U);
                sim_symbol(&s, cmds[k], speeds[v], dir, 0);
                if (s.decoded != 1 || s.last != cmds[k] || s.failed) {
                    ok = 0;
                    printf("[PC]   clean fail %c %u mm/s dir %d "
                           "(dec %d fail %d)\n", cmds[k],
                           (unsigned)speeds[v], dir, s.decoded, s.failed);
                }
            }
        }
    }
    CHECK("S3-P06", ok, "A-D at 50..1000 mm/s, both directions, across "
          "the timer wrap");

    ok = 1;
    for (size_t v = 0; v < sizeof(speeds) / sizeof(speeds[0]); v++) {
        for (int k = 0; k < 4; k++) {
            for (int dir = 0; dir < 2; dir++) {
                sim_init(&s, 1000U);
                sim_run_to(&s, s.now + 50000U);
                sim_symbol(&s, cmds[k], speeds[v], dir, 1);
                if (s.decoded != 1 || s.last != cmds[k] || s.failed) {
                    ok = 0;
                    printf("[PC]   chatter fail %c %u mm/s dir %d\n",
                           cmds[k], (unsigned)speeds[v], dir);
                }
            }
        }
    }
    CHECK("S3-P07", ok, "same, with a 40 us comparator spike inside "
          "every element");

    ok = 1;
    for (size_t v = 0; v < sizeof(speeds) / sizeof(speeds[0]); v++) {
        for (int k = 0; k < 4; k++) {
            for (int dir = 0; dir < 2; dir++) {
                sim_init(&s, 7000U);
                sim_run_to(&s, s.now + 50000U);
                sim_symbol(&s, cmds[k], speeds[v], dir, 2);
                if (s.decoded != 1 || s.last != cmds[k] || s.failed) {
                    ok = 0;
                    printf("[PC]   burst fail %c %u mm/s dir %d "
                           "(dec %d fail %d)\n", cmds[k],
                           (unsigned)speeds[v], dir, s.decoded, s.failed);
                }
            }
        }
    }
    CHECK("S3-P07", ok, "same, with the measured 8-flip chatter burst "
          "at every edge");

    /* Two symbols in a row */
    sim_init(&s, 5000U);
    sim_run_to(&s, s.now + 50000U);
    sim_symbol(&s, 'B', 200U, 0, 0);
    sim_symbol(&s, 'C', 200U, 1, 0);
    CHECK("S3-P08", s.decoded == 2 && s.last == 'C' && s.failed == 0,
          "two barcodes in a row both decode");

    /* Sensor crosses the 18 mm line at 200 mm/s: one long bar */
    sim_init(&s, 5000U);
    sim_run_to(&s, s.now + 50000U);
    sim_edge(&s, 0U, true);
    sim_edge(&s, 90000U, false);
    sim_run_to(&s, s.now + 500000U);
    CHECK("S3-P09", s.decoded == 0 && s.failed == 0,
          "crossing the line is ignored (no decode, no failure)");

    /* Car stops half way through a barcode */
    {
        uint8_t mods[BC39_SYMBOL_ELEMENTS];
        sim_init(&s, 5000U);
        sim_run_to(&s, s.now + 50000U);
        symbol_modules('A', mods);
        for (int i = 0; i < 14; i++) {
            sim_edge(&s, 0U, (i % 2) == 0);
            sim_run_to(&s, s.now + mods[i] * 15000U);
        }
        sim_run_to(&s, s.now + 2000000U);
        CHECK("S3-P10", s.decoded == 0 && s.failed == 1,
              "half a barcode is reported once as a failed read");
    }

    /* Seen on the car: the car stalls on a bar, the run breaks off,
       and a real barcode follows after only a short white. The broken
       run must not make the decoder wait for a long quiet zone. */
    sim_init(&s, 9000U);
    sim_run_to(&s, s.now + 50000U);
    sim_edge(&s, 0U, true);                 /* bar 100 ms */
    sim_edge(&s, 100000U, false);           /* gap 100 ms */
    sim_edge(&s, 100000U, true);            /* stalled bar 1.2 s */
    sim_edge(&s, 1200000U, false);          /* then 150 ms white */
    sim_run_to(&s, s.now + 150000U);
    sim_symbol(&s, 'B', 50U, 0, 2);
    CHECK("S3-P11", s.decoded == 1 && s.last == 'B' && s.failed == 0,
          "a broken run does not hide the next barcode");

    /* Seen on the car 2026-11-09 (3 of 4 forward reads of *E* failed
       with 27 elements): the sensor reads bars thin and spaces wide,
       so '*' starts with a ~30 ms bar and a ~175 ms space, more than
       5 x the only element seen so far. The space must not end the
       run. Widths (us) are the logged run with its two lost elements
       put back: 30 ms first bar, then the 174.4 ms space. */
    {
        static const uint32_t car_e[29] = {
            30000U, 174400U,
            36100U, 60300U, 166000U, 56800U, 160700U, 62100U, 37000U,
            65800U, 154500U, 61600U, 34000U, 66500U, 146900U, 174400U,
            26000U, 68100U, 29300U, 64400U, 25600U, 191500U, 17700U,
            66300U, 149100U, 56700U, 151800U, 59300U, 35300U };
        sim_init(&s, 3000U);
        sim_run_to(&s, s.now + 50000U);
        for (int i = 0; i < 29; i++) {
            sim_edge(&s, 0U, (i % 2) == 0);
            sim_run_to(&s, s.now + car_e[i]);
        }
        sim_edge(&s, 0U, false);
        sim_run_to(&s, s.now + 2000000U);
        CHECK("S3-P31", s.decoded == 1 && s.last == 'E' && s.failed == 0,
              "car widths: thin first bar + wide space still decode *E*");
    }

    /* Power-up on top of a bar: first (partial) symbol is skipped */
    sim_init(&s, 5000U);
    bc_capture_init(&s.cap, true, s.now);
    sim_run_to(&s, s.now + 4000U);
    sim_edge(&s, 0U, false);
    sim_run_to(&s, s.now + 300000U);
    sim_symbol(&s, 'D', 300U, 0, 0);
    CHECK("S3-P11", s.decoded == 1 && s.last == 'D',
          "power-up on black waits for quiet, then decodes");
}

/* ------------------------------------------------------------------ */
/* Line logic                                                          */
/* ------------------------------------------------------------------ */

static void steps(line_logic_t *p, int n, bool c, bool r, bool bc,
                  uint8_t *e, uint8_t *m)
{
    for (int i = 0; i < n; i++) {
        line_logic_step(p, c, r, bc, e, m);
    }
}

static void test_line_logic(void)
{
    line_logic_t ctx;
    uint8_t e = 0, m = 0;

    /* Start: no line seen yet */
    line_logic_init(&ctx);
    steps(&ctx, 1, false, false, false, &e, &m);
    CHECK("S3-P12", m == LINE_MODE_TRACKING && e == 64U,
          "start on white/white: TRACKING, straight");

    /* Centre over the line */
    steps(&ctx, 1, true, false, false, &e, &m);
    CHECK("S3-P13", e == 64U, "one-cycle black is debounced away");
    steps(&ctx, 1, true, false, false, &e, &m);
    CHECK("S3-P13", m == LINE_MODE_TRACKING && e == 64U,
          "centre black: on the line, straight");

    /* Centre loses the line, right never saw it: line went left */
    steps(&ctx, 2, false, false, false, &e, &m);
    CHECK("S3-P13", m == LINE_MODE_TRACKING && e == 64U - LINE_ERR_GUESS,
          "centre lost, right white: guess left, gently");
    steps(&ctx, 2, true, false, false, &e, &m);
    CHECK("S3-P13", e == 64U, "centre back on the line: straight");

    /* Line drifts right: centre+right, then right only */
    steps(&ctx, 2, true, true, false, &e, &m);
    CHECK("S3-P13", m == LINE_MODE_TRACKING && e == 64U + LINE_ERR_NEAR,
          "centre and right black: steer a little right");
    steps(&ctx, 2, false, true, false, &e, &m);
    CHECK("S3-P13", e == 64U + LINE_ERR_SIDE, "right black: steer right");
    steps(&ctx, 2, false, false, false, &e, &m);
    CHECK("S3-P13", e == 64U + LINE_ERR_SIDE,
          "past the right sensor: keep steering right");

    /* Back over the line from the right, then off to the left */
    steps(&ctx, 2, false, true, false, &e, &m);
    steps(&ctx, 2, true, false, false, &e, &m);
    steps(&ctx, 2, false, false, false, &e, &m);
    CHECK("S3-P13", e == 64U - LINE_ERR_GUESS,
          "line crossed the centre leftward: gentle left");
    CHECK("S3-P13", LINE_ERR_GUESS < LINE_ERR_SIDE,
          "a guessed turn is gentler than a seen one");

    /* The barcode sensor alone never steers */
    steps(&ctx, 2, true, false, true, &e, &m);
    CHECK("S3-P13", m == LINE_MODE_TRACKING && e == 64U,
          "barcode black with centre: still straight");

    /* Junction: all three black */
    steps(&ctx, 2, true, true, false, &e, &m);
    CHECK("S3-P14", m == LINE_MODE_TRACKING,
          "centre+right without barcode: not a junction");
    steps(&ctx, 2, true, true, true, &e, &m);
    steps(&ctx, LINE_JUNCTION_TICKS, true, true, true, &e, &m);
    CHECK("S3-P14", m == LINE_MODE_JUNCTION && e == 64U,
          "all three black: JUNCTION, straight");
    steps(&ctx, 2, true, false, false, &e, &m);
    CHECK("S3-P14", m == LINE_MODE_TRACKING && e == 64U,
          "junction passed: TRACKING");

    /* Lost: last seen on the right */
    line_logic_init(&ctx);
    steps(&ctx, 2, false, true, false, &e, &m);
    steps(&ctx, 2, false, false, false, &e, &m);
    steps(&ctx, LINE_LOST_TICKS - 3, false, false, false, &e, &m);
    CHECK("S3-P15", m == LINE_MODE_TRACKING,
          "just under the lost time: still TRACKING");
    steps(&ctx, 2, false, false, false, &e, &m);
    CHECK("S3-P15", m == LINE_MODE_SEARCH_NEAR && e == 127U,
          "lost: full turn toward the last side (right)");
    steps(&ctx, LINE_SEARCH1_TICKS, false, false, false, &e, &m);
    CHECK("S3-P15", m == LINE_MODE_SEARCH_FAR && e == 1U,
          "then full turn the other way (left)");
    steps(&ctx, LINE_SEARCH2_TICKS, false, false, false, &e, &m);
    CHECK("S3-P15", m == LINE_MODE_FAILED && e == 64U,
          "then FAILED, error centred");
    steps(&ctx, 2, false, false, true, &e, &m);
    CHECK("S3-P15", m == LINE_MODE_FAILED,
          "barcode sensor alone does not end FAILED");
    steps(&ctx, 2, true, false, false, &e, &m);
    CHECK("S3-P15", m == LINE_MODE_TRACKING && e == 64U,
          "centre black again: back to TRACKING, straight");

    /* Lost: last seen by the centre only -> search left first */
    line_logic_init(&ctx);
    steps(&ctx, 2, true, false, false, &e, &m);
    steps(&ctx, LINE_LOST_TICKS + 2, false, false, false, &e, &m);
    CHECK("S3-P15", m == LINE_MODE_SEARCH_NEAR && e == 1U,
          "lost after centre: full turn left first");
}

/* ------------------------------------------------------------------ */
/* Line drive (wheel duties)                                           */
/* ------------------------------------------------------------------ */

static bool drive_n(line_drive_t *p, int n, uint8_t err, uint8_t mode,
                    bool c, int16_t *l, int16_t *r)
{
    bool b = false;
    for (int i = 0; i < n; i++) {
        b = line_drive_step(p, err, mode, c, l, r);
    }
    return b;
}

static void test_line_drive(void)
{
    line_drive_t d;
    int16_t l = 99, r = 99;
    bool b;

    line_drive_init(&d);
    b = drive_n(&d, (int)DRIVE_ARM_TICKS - 1, 64U, LINE_MODE_TRACKING,
                true, &l, &r);
    CHECK("S3-P19", !b && l == 0 && r == 0,
          "not armed before 2 s on the line: stopped");
    b = drive_n(&d, 1, 64U, LINE_MODE_TRACKING, true, &l, &r);
    CHECK("S3-P19", b && l == DRIVE_BASE_DUTY && r == DRIVE_BASE_DUTY,
          "armed: centred -> both wheels at base speed");

    line_drive_init(&d);
    drive_n(&d, 150, 64U, LINE_MODE_TRACKING, true, &l, &r);
    drive_n(&d, 1, 64U, LINE_MODE_TRACKING, false, &l, &r);
    b = drive_n(&d, 150, 64U, LINE_MODE_TRACKING, true, &l, &r);
    CHECK("S3-P19", !b, "leaving the line resets the arm timer");

    line_drive_init(&d);
    drive_n(&d, (int)DRIVE_ARM_TICKS, 64U, LINE_MODE_TRACKING, true,
            &l, &r);
    b = drive_n(&d, 1, 74U, LINE_MODE_TRACKING, true, &l, &r);
    CHECK("S3-P20", b && l == DRIVE_BASE_DUTY + 10 * DRIVE_KP_NUM
                    && r == DRIVE_BASE_DUTY - 10 * DRIVE_KP_NUM,
          "line a little right: left wheel faster");
    b = drive_n(&d, 1, 32U, LINE_MODE_TRACKING, false, &l, &r);
    CHECK("S3-P20", b && l < r, "line left: right wheel faster");
    b = drive_n(&d, 1, 96U, LINE_MODE_TRACKING, false, &l, &r);
    CHECK("S3-P20", b && l > r && r > 0 && (l - r) <= 64,
          "right sensor only: gentle right, both wheels forward");
    b = drive_n(&d, 1, 64U - LINE_ERR_GUESS, LINE_MODE_TRACKING, false,
                &l, &r);
    CHECK("S3-P20", b && r > l && l > 0 && (r - l) <= 64,
          "guessed left: gentle left, both wheels forward");
    b = drive_n(&d, 1, 127U, LINE_MODE_SEARCH_NEAR, false, &l, &r);
    CHECK("S3-P20", b && l == DRIVE_DUTY_MAX && r < 0,
          "search right: clamped, spins on the spot");
    b = drive_n(&d, 1, 1U, LINE_MODE_SEARCH_FAR, false, &l, &r);
    CHECK("S3-P20", b && r == DRIVE_DUTY_MAX && l < 0
                    && l >= -DRIVE_DUTY_MAX,
          "search left: clamped, spins the other way");
    b = drive_n(&d, 1, 64U, LINE_MODE_JUNCTION, true, &l, &r);
    CHECK("S3-P20", b && l == r, "junction: straight");

    b = drive_n(&d, 1, 64U, LINE_MODE_FAILED, false, &l, &r);
    CHECK("S3-P21", !b && l == 0 && r == 0, "FAILED: stop");
    b = drive_n(&d, 5, 64U, LINE_MODE_TRACKING, true, &l, &r);
    CHECK("S3-P21", !b, "after FAILED: must re-arm (2 s on the line)");
    b = line_drive_step(NULL, 64U, LINE_MODE_TRACKING, true, &l, &r);
    CHECK("S3-P21", !b, "NULL context: stop");
}

/* ------------------------------------------------------------------ */
/* IF-06 word                                                          */
/* ------------------------------------------------------------------ */

static void test_line_if(void)
{
    line_if_state_t in, out;
    line_if_reader_t rd;
    int ok = 1;

    for (unsigned err = 0; err <= 127U; err += 7U) {
        for (unsigned mode = 0; mode <= LINE_MODE_FAILED; mode++) {
            for (unsigned cmd = 0; cmd < 4U; cmd++) {
                memset(&in, 0, sizeof(in));
                in.error = (uint8_t)err;
                in.mode = (uint8_t)mode;
                in.cmd = (uint8_t)cmd;
                in.cmd_seq = (uint8_t)((cmd + 1U) & 3U);
                in.bc_char = (uint8_t)(err % 26U);
                in.char_seq = (uint8_t)(mode & 3U);
                in.seq = (uint8_t)(err * 2U + mode);
                in.b_centre_black = (mode & 1U) != 0U;
                in.b_right_black = (cmd & 1U) != 0U;
                in.b_barcode_black = (err & 1U) != 0U;
                line_if_unpack(line_if_pack(&in), &out);
                if (memcmp(&in, &out, sizeof(in)) != 0) ok = 0;
            }
        }
    }
    CHECK("S3-P16", ok, "pack/unpack round trip, all fields");

    memset(&in, 0, sizeof(in));
    in.error = 200U;                    /* out of range: masked */
    CHECK("S3-P16", (line_if_pack(&in) & ~0x7FU) == 0U,
          "out-of-range error cannot spill into other fields");
    in.error = 0U;
    in.bc_char = 0xFFU;                 /* out of range: masked */
    CHECK("S3-P16", line_if_pack(&in) == (0x1FUL << 17),
          "out-of-range letter cannot spill into other fields");
    in.bc_char = 0U;

    in.error = 0U;
    CHECK("S3-P17", line_if_error_signed(line_if_pack(&in)) == -64,
          "error 0 -> -64");
    in.error = 127U;
    CHECK("S3-P17", line_if_error_signed(line_if_pack(&in)) == 63,
          "error 127 -> +63");
    in.error = 64U;
    CHECK("S3-P17", line_if_error_signed(line_if_pack(&in)) == 0,
          "error 64 -> 0");

    memset(&in, 0, sizeof(in));
    in.seq = 10U;
    line_if_reader_init(&rd, line_if_pack(&in));
    in.seq = 11U;
    CHECK("S3-P18", line_if_is_fresh(&rd, line_if_pack(&in)),
          "new seq: fresh");
    CHECK("S3-P18", line_if_is_fresh(&rd, line_if_pack(&in)),
          "same seq once: still fresh");
    CHECK("S3-P18", line_if_is_fresh(&rd, line_if_pack(&in)),
          "same seq twice: still fresh");
    CHECK("S3-P18", !line_if_is_fresh(&rd, line_if_pack(&in)),
          "same seq 3 times: STALE");
    in.seq = 12U;
    CHECK("S3-P18", line_if_is_fresh(&rd, line_if_pack(&in)),
          "seq moves again: fresh");
    CHECK("S3-P18", !line_if_is_fresh(NULL, 0U), "NULL reader: stale");
}

static void report_sizes(void)
{
    printf("[PC]          sizeof line_state word   = %u B\n",
           (unsigned)sizeof(uint32_t));
    printf("[PC]          sizeof line_if_state_t   = %u B (unpacked)\n",
           (unsigned)sizeof(line_if_state_t));
    printf("[PC]          sizeof line_logic_t      = %u B\n",
           (unsigned)sizeof(line_logic_t));
    printf("[PC]          sizeof bc_capture_t      = %u B\n",
           (unsigned)sizeof(bc_capture_t));
}

/* ------------------------------------------------------------------ */
/* Demo requirements: A-Z both ways, map, act at the cross-junction    */
/* ------------------------------------------------------------------ */

static void test_all_letters(void)
{
    const uint32_t speeds[] = {50U, 100U, 200U, 400U};
    sim_t s;
    int ok = 1;

    for (char c = 'A'; c <= 'Z'; c++) {
        for (size_t v = 0; v < sizeof(speeds) / sizeof(speeds[0]); v++) {
            for (int dir = 0; dir < 2; dir++) {
                sim_init(&s, 3000U);
                sim_run_to(&s, s.now + 50000U);
                sim_symbol(&s, c, speeds[v], dir, 2);
                if (s.decoded != 1 || s.last != c || s.failed) {
                    ok = 0;
                    printf("[PC]   letter fail %c %u mm/s dir %d\n", c,
                           (unsigned)speeds[v], dir);
                }
            }
        }
    }
    CHECK("S3-P22", ok, "A-Z through capture and decoder, both "
          "directions, with the measured chatter");
}

static void test_nav_map(void)
{
    uint32_t map = NAV_MAP_EMPTY;
    uint8_t m = 99U;

    CHECK("S3-P23", !nav_map_lookup(map, 'A', &m),
          "nothing is mapped until a mapping is received");
    CHECK("S3-P23", nav_map_with(map, LINE_CMD_LEFT, 'Q', &map)
                    && nav_map_with(map, LINE_CMD_RIGHT, 'Z', &map)
                    && nav_map_with(map, LINE_CMD_STRAIGHT, 'A', &map)
                    && nav_map_with(map, LINE_CMD_UTURN, 'M', &map),
          "any four letters can be mapped");
    CHECK("S3-P23", (nav_map_lookup(map, 'Q', &m) && m == LINE_CMD_LEFT)
                    && (nav_map_lookup(map, 'Z', &m) && m == LINE_CMD_RIGHT)
                    && (nav_map_lookup(map, 'A', &m)
                        && m == LINE_CMD_STRAIGHT)
                    && (nav_map_lookup(map, 'M', &m) && m == LINE_CMD_UTURN),
          "each letter finds its own movement");
    CHECK("S3-P23", !nav_map_lookup(map, 'B', &m),
          "an unmapped letter has no movement");
    CHECK("S3-P23", (map >> 20) == 0U, "mapping uses only 20 bits");

    /* A new mapping replaces the old one, slot and letter */
    CHECK("S3-P23", nav_map_with(map, LINE_CMD_LEFT, 'B', &map)
                    && !nav_map_lookup(map, 'Q', &m)
                    && nav_map_lookup(map, 'B', &m) && m == LINE_CMD_LEFT,
          "re-mapping a movement drops its old letter");
    CHECK("S3-P23", nav_map_with(map, LINE_CMD_RIGHT, 'B', &map)
                    && nav_map_lookup(map, 'B', &m) && m == LINE_CMD_RIGHT,
          "a letter drives only one movement");
    {
        uint32_t before = map;
        CHECK("S3-P23", !nav_map_with(map, LINE_CMD_LEFT, 'a', &map)
                        && !nav_map_with(map, LINE_CMD_LEFT, '5', &map)
                        && !nav_map_with(map, 4U, 'C', &map)
                        && map == before,
              "bad letter or movement is refused, mapping unchanged");
    }
}

static nav_event_t nav_n(nav_t *p, int n, uint32_t map, uint8_t mode,
                         bool c, nav_out_t *o)
{
    nav_event_t ev = NAV_EV_NONE;
    for (int i = 0; i < n; i++) {
        ev = nav_step(p, map, mode, c, true, o);
    }
    return ev;
}

static void test_nav_junction(void)
{
    uint32_t map = NAV_MAP_EMPTY;
    nav_t n;
    nav_out_t o;
    nav_event_t ev;

    nav_map_with(map, LINE_CMD_LEFT, 'Q', &map);
    nav_map_with(map, LINE_CMD_RIGHT, 'R', &map);
    nav_map_with(map, LINE_CMD_STRAIGHT, 'S', &map);
    nav_map_with(map, LINE_CMD_UTURN, 'U', &map);

    /* The letter waits for the junction */
    nav_init(&n);
    CHECK("S3-P24", nav_store(&n, 'Q'), "a decoded letter is stored");
    CHECK("S3-P24", !nav_store(&n, '5'), "a non-letter is not stored");
    ev = nav_n(&n, 300, map, LINE_MODE_TRACKING, true, &o);
    CHECK("S3-P24", ev == NAV_EV_NONE && !o.b_override,
          "no movement until the junction");

    /* Left: forward, spin left, stop on the next line */
    ev = nav_n(&n, 1, map, LINE_MODE_JUNCTION, true, &o);
    CHECK("S3-P24", ev == NAV_EV_JUNCTION && o.letter == 'Q' && o.b_mapped
                    && o.movement == LINE_CMD_LEFT,
          "at the junction the stored letter gives its movement");
    ev = nav_n(&n, 1, map, LINE_MODE_JUNCTION, true, &o);
    CHECK("S3-P24", ev == NAV_EV_NONE && o.b_override && o.left == o.right
                    && o.left > 0, "first drives on over the cross");
    nav_n(&n, (int)NAV_PREFWD_TICKS - 1, map, LINE_MODE_TRACKING, true, &o);
    ev = nav_n(&n, 1, map, LINE_MODE_TRACKING, true, &o);
    CHECK("S3-P24", ev == NAV_EV_NONE && o.left < 0 && o.right > 0,
          "left: spins left");
    ev = nav_n(&n, 1, map, LINE_MODE_TRACKING, false, &o);
    nav_n(&n, (int)NAV_SPIN_MIN_TICKS, map, LINE_MODE_TRACKING, false, &o);
    ev = nav_n(&n, 1, map, LINE_MODE_TRACKING, true, &o);
    CHECK("S3-P24", ev == NAV_EV_TURN_DONE && o.b_override,
          "stops on the first line");

    /* The same cross must not count again; the letter is used up */
    ev = nav_n(&n, 1, map, LINE_MODE_TRACKING, true, &o);
    CHECK("S3-P25", !o.b_override, "line following again after the turn");
    nav_n(&n, 1, map, LINE_MODE_TRACKING, true, &o);
    ev = nav_n(&n, 1, map, LINE_MODE_JUNCTION, true, &o);
    CHECK("S3-P25", ev == NAV_EV_NONE, "hold-off ignores the same cross");
    nav_n(&n, (int)NAV_HOLDOFF_TICKS, map, LINE_MODE_JUNCTION, true, &o);
    nav_n(&n, 2, map, LINE_MODE_TRACKING, true, &o);
    ev = nav_n(&n, 1, map, LINE_MODE_JUNCTION, true, &o);
    CHECK("S3-P25", ev == NAV_EV_JUNCTION && o.letter == '\0' && !o.b_mapped
                    && o.movement == LINE_CMD_STRAIGHT,
          "next cross with no new letter: forward");
    nav_n(&n, 1, map, LINE_MODE_JUNCTION, true, &o);
    CHECK("S3-P25", !o.b_override, "forward keeps normal line following");

    /* Unmapped letter, newest letter wins */
    nav_init(&n);
    nav_store(&n, 'K');
    ev = nav_n(&n, 1, map, LINE_MODE_JUNCTION, true, &o);
    CHECK("S3-P25", ev == NAV_EV_JUNCTION && o.letter == 'K' && !o.b_mapped
                    && o.movement == LINE_CMD_STRAIGHT,
          "a letter with no mapping: forward");
    nav_init(&n);
    nav_store(&n, 'Q');
    nav_store(&n, 'S');
    nav_n(&n, 1, map, LINE_MODE_JUNCTION, true, &o);
    CHECK("S3-P25", o.letter == 'S' && o.movement == LINE_CMD_STRAIGHT,
          "a newer barcode replaces the stored letter");

    /* Right spins the other way */
    nav_init(&n);
    nav_store(&n, 'R');
    nav_n(&n, 1, map, LINE_MODE_JUNCTION, true, &o);
    nav_n(&n, (int)NAV_PREFWD_TICKS + 1, map, LINE_MODE_TRACKING, true, &o);
    nav_n(&n, 1, map, LINE_MODE_TRACKING, false, &o);
    CHECK("S3-P26", o.left > 0 && o.right < 0, "right: spins right");

    /* U-turn: side branch first, then the way back */
    nav_init(&n);
    nav_store(&n, 'U');
    nav_n(&n, 1, map, LINE_MODE_JUNCTION, true, &o);
    nav_n(&n, (int)NAV_PREFWD_TICKS + 1, map, LINE_MODE_TRACKING, true, &o);
    nav_n(&n, (int)NAV_SPIN_MIN_TICKS, map, LINE_MODE_TRACKING, false, &o);
    ev = nav_n(&n, 1, map, LINE_MODE_TRACKING, true, &o);
    CHECK("S3-P26", ev == NAV_EV_NONE && o.b_override,
          "U-turn: keeps spinning past the first line");
    nav_n(&n, 5, map, LINE_MODE_TRACKING, true, &o);
    nav_n(&n, 5, map, LINE_MODE_TRACKING, false, &o);
    ev = nav_n(&n, 1, map, LINE_MODE_TRACKING, true, &o);
    CHECK("S3-P26", ev == NAV_EV_TURN_DONE, "U-turn: stops on the second");

    /* The line the spin starts on does not count */
    nav_init(&n);
    nav_store(&n, 'Q');
    nav_n(&n, 1, map, LINE_MODE_JUNCTION, true, &o);
    nav_n(&n, (int)NAV_PREFWD_TICKS + 1, map, LINE_MODE_TRACKING, true, &o);
    nav_n(&n, 3, map, LINE_MODE_TRACKING, false, &o);
    ev = nav_n(&n, 1, map, LINE_MODE_TRACKING, true, &o);
    CHECK("S3-P26", ev == NAV_EV_NONE && o.b_override,
          "black early in the spin is not the next line");

    /* No line found, and the car stopped */
    nav_init(&n);
    nav_store(&n, 'Q');
    nav_n(&n, 1, map, LINE_MODE_JUNCTION, true, &o);
    nav_n(&n, (int)NAV_PREFWD_TICKS + 1, map, LINE_MODE_TRACKING, false, &o);
    /* the last forward tick above was already the first spin tick */
    ev = nav_n(&n, (int)NAV_SPIN_MAX_TICKS - 1, map, LINE_MODE_TRACKING,
               false, &o);
    CHECK("S3-P26", ev == NAV_EV_TURN_TIMEOUT, "spin gives up after 5 s");
    nav_init(&n);
    nav_store(&n, 'Q');
    ev = nav_step(&n, map, LINE_MODE_JUNCTION, true, false, &o);
    CHECK("S3-P27", ev == NAV_EV_NONE && !o.b_override,
          "car not armed: no junction, no movement");
    nav_step(&n, map, LINE_MODE_TRACKING, true, false, &o);
    ev = nav_step(&n, map, LINE_MODE_JUNCTION, true, true, &o);
    CHECK("S3-P27", ev == NAV_EV_JUNCTION && o.letter == 'Q',
          "the stored letter survives until the car may move");
}

/* A barcode bar under the left sensor must not fake a junction, but a
   crossing line still has to count. */
static void test_junction_gate(void)
{
    sim_t s;
    line_logic_t ctx;
    uint8_t e = 0, m = 0;

    sim_init(&s, 4000U);
    sim_run_to(&s, s.now + 50000U);
    CHECK("S3-P28", !bc_capture_busy(&s.cap), "idle: not busy");
    sim_edge(&s, 0U, true);
    sim_run_to(&s, s.now + 20000U);
    CHECK("S3-P28", !bc_capture_busy(&s.cap),
          "first bar (could be a crossing line): not busy");
    sim_edge(&s, 0U, false);
    sim_run_to(&s, s.now + 20000U);
    CHECK("S3-P28", bc_capture_busy(&s.cap),
          "second element: it is a barcode, busy");

    /* First bar of a barcode: too short to be the crossing line */
    line_logic_init(&ctx);
    steps(&ctx, 5, true, true, false, &e, &m);
    steps(&ctx, 2 + (int)LINE_JUNCTION_TICKS - 2, true, true, true, &e, &m);
    CHECK("S3-P28", m != LINE_MODE_JUNCTION,
          "a short bar during a drift is not a junction");
    steps(&ctx, 3, true, true, true, &e, &m);
    CHECK("S3-P28", m == LINE_MODE_JUNCTION,
          "all three black for the full time is a junction");
}

/* ------------------------------------------------------------------ */

static void test_line_if_char(void)
{
    line_if_state_t in;
    uint8_t code = 99U;

    CHECK("S3-P29", line_if_char_code('A', &code) && code == 0U, "'A' is 0");
    CHECK("S3-P29", line_if_char_code('Z', &code) && code == 25U, "'Z' is 25");
    code = 99U;
    CHECK("S3-P29", !line_if_char_code('5', &code) && code == 99U,
          "a digit is not a letter, code untouched");
    CHECK("S3-P29", !line_if_char_code('a', &code) && code == 99U,
          "lower case is not a letter");
    CHECK("S3-P29", !line_if_char_code('A', NULL), "NULL: no write");

    memset(&in, 0, sizeof(in));
    in.bc_char = 25U;
    in.char_seq = 2U;
    CHECK("S3-P29", line_if_char(line_if_pack(&in)) == 25U, "char accessor");
    CHECK("S3-P29", line_if_char_seq(line_if_pack(&in)) == 2U,
          "char_seq accessor");
    in.bc_char = 0U;
    in.char_seq = 3U;
    in.seq = 0U;
    CHECK("S3-P29", line_if_pack(&in) == (3UL << 22),
          "char_seq sits below seq, no overlap");
}

static void test_nav_watch(void)
{
    nav_watch_t w;
    int count = 0;

    nav_watch_init(&w);
    CHECK("S3-P30", !nav_watch_step(&w, LINE_MODE_TRACKING),
          "tracking: no crossing");
    CHECK("S3-P30", nav_watch_step(&w, LINE_MODE_JUNCTION),
          "first junction reading: one crossing");
    CHECK("S3-P30", !nav_watch_step(&w, LINE_MODE_JUNCTION),
          "same crossing: not counted again");
    CHECK("S3-P30", !nav_watch_step(&w, LINE_MODE_TRACKING), "left it");
    CHECK("S3-P30", !nav_watch_step(&w, LINE_MODE_JUNCTION),
          "flicker inside the hold-off: not counted");

    for (unsigned i = 0U; i < NAV_HOLDOFF_TICKS; i++) {
        if (nav_watch_step(&w, LINE_MODE_TRACKING)) count++;
    }
    CHECK("S3-P30", count == 0, "hold-off stays quiet");
    CHECK("S3-P30", nav_watch_step(&w, LINE_MODE_JUNCTION),
          "next crossing after the hold-off counts");

    nav_watch_init(&w);
    (void)nav_watch_step(&w, LINE_MODE_JUNCTION);
    for (unsigned i = 0U; i < NAV_HOLDOFF_TICKS + 5U; i++) {
        (void)nav_watch_step(&w, LINE_MODE_JUNCTION);
    }
    CHECK("S3-P30", !nav_watch_step(&w, LINE_MODE_JUNCTION),
          "still on black after the hold-off: same crossing");
    CHECK("S3-P30", !nav_watch_step(NULL, LINE_MODE_JUNCTION), "NULL: false");
}

int main(void)
{
    test_table();
    test_reference_patterns();
    test_pdf_sample();
    test_random_decode();
    test_decode_errors();
    test_capture();
    test_line_logic();
    test_line_drive();
    test_line_if();
    test_all_letters();
    test_nav_map();
    test_nav_junction();
    test_junction_gate();
    test_line_if_char();
    test_nav_watch();
    report_sizes();
    printf("[PC] %d checks, %d failed\n", g_checks, g_failed);
    return g_failed == 0 ? 0 : 1;
}
