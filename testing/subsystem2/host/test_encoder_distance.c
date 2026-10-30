/*
 * Host test for IF-10 (S2 -> S4 cumulative distance, mm).
 * Runs the production encoder.c against fake GPIO pins.
 * Build and run: ./run_tests.sh
 */
#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <tk/tkernel.h>
#include <bsp/libbsp.h>

UINT host_gpio[32];

#include "../../../app_program/subsystem2/encoder.c"

/* One channel-A edge per wheel; dir +1 = forward, -1 = reverse. */
static void step(int left_dir, int right_dir)
{
    if (left_dir != 0) {
        host_gpio[ENCODER_LEFT_A_PIN] ^= 1U;
        /* encoder.c: left counts +1 when A != B */
        host_gpio[ENCODER_LEFT_B_PIN] = (left_dir > 0)
            ? !host_gpio[ENCODER_LEFT_A_PIN] : host_gpio[ENCODER_LEFT_A_PIN];
    }
    if (right_dir != 0) {
        host_gpio[ENCODER_RIGHT_A_PIN] ^= 1U;
        /* encoder.c: right counts +1 when A == B */
        host_gpio[ENCODER_RIGHT_B_PIN] = (right_dir > 0)
            ? host_gpio[ENCODER_RIGHT_A_PIN] : !host_gpio[ENCODER_RIGHT_A_PIN];
    }
    encoder_sample();
}

static void move(int32_t left_counts, int32_t right_counts)
{
    int32_t l = (left_counts < 0) ? -left_counts : left_counts;
    int32_t r = (right_counts < 0) ? -right_counts : right_counts;
    int32_t n = (l > r) ? l : r;
    int32_t i;

    for (i = 0; i < n; i++) {
        step((i < l) ? ((left_counts > 0) ? 1 : -1) : 0,
             (i < r) ? ((right_counts > 0) ? 1 : -1) : 0);
    }
}

/* Float reference: mean wheel travel in mm. */
static double ref_mm(int32_t left, int32_t right)
{
    double circ = 3.14159265 * 66.0;
    return 0.5 * (left * circ / 1271.0 + right * circ / 1266.0);
}

int main(void)
{
    int32_t lt = 0, rt = 0;

    assert(encoder_init() == E_OK);
    encoder_publish_distance();
    assert(encoder_get_distance_mm() == 0);

    /* 5 wheel turns forward each: ~1036.7 mm */
    move(5 * 1271, 5 * 1266); lt += 5 * 1271; rt += 5 * 1266;
    encoder_publish_distance();
    printf("5 turns: %d mm (ref %.1f)\n", encoder_get_distance_mm(),
           ref_mm(lt, rt));
    assert(fabs(encoder_get_distance_mm() - ref_mm(lt, rt)) <= 1.0);

    /* reset_counts (used by S2 for speed) must not move the odometer */
    encoder_reset_counts();
    encoder_publish_distance();
    assert(fabs(encoder_get_distance_mm() - ref_mm(lt, rt)) <= 1.0);

    /* reverse 1 turn subtracts */
    move(-1271, -1266); lt -= 1271; rt -= 1266;
    encoder_publish_distance();
    assert(fabs(encoder_get_distance_mm() - ref_mm(lt, rt)) <= 1.0);

    /* turning on the spot (left fwd, right back) adds ~0 */
    move(1000, -1000); lt += 1000; rt -= 1000;
    encoder_publish_distance();
    assert(fabs(encoder_get_distance_mm() - ref_mm(lt, rt)) <= 1.0);

    /* long run: 2,000,000 counts (~327 m) without 32-bit overflow */
    move(2000000, 2000000); lt += 2000000; rt += 2000000;
    encoder_publish_distance();
    printf("long run: %d mm (ref %.1f)\n", encoder_get_distance_mm(),
           ref_mm(lt, rt));
    assert(fabs(encoder_get_distance_mm() - ref_mm(lt, rt)) <= 1.0);

    puts("PASS: IF-10 distance: zero, forward, reset-safe, reverse,"
         " spin, long run");
    return 0;
}
