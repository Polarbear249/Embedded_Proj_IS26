/*
 * Host test for the interrupt-driven encoder counting and IF-10
 * (S2 -> S4 cumulative distance, mm). Runs the production encoder.c; the
 * test calls its edge callbacks directly, as the GPIO interrupt would.
 * Build and run: ./run_tests.sh
 */
#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <tk/tkernel.h>
#include <bsp/libbsp.h>

UINT host_gpio[32];
static int attached;

#include "../../../app_program/gpio_irq.h"

/* Stand-in for the shared dispatcher: just record the attach. */
ER gpio_irq_attach(uint32_t pin, uint32_t events, gpio_irq_cb_t p_cb)
{
    (void)pin;
    assert(events == GPIO_IRQ_EDGE_BOTH && p_cb != 0);
    attached++;
    return E_OK;
}

#include "../../../app_program/subsystem2/encoder.c"

static UINT left_a, right_a;

/* One A edge per wheel (alternating rise/fall); dir +1 forward, -1 back,
   B set the way the real quadrature signal would be at that edge. */
static void step(int left_dir, int right_dir)
{
    uint32_t ev;

    if (left_dir != 0) {
        left_a ^= 1U;
        ev = left_a ? GPIO_IRQ_EDGE_RISE : GPIO_IRQ_EDGE_FALL;
        /* left forward = +1: needs A != B after the edge */
        host_gpio[ENCODER_LEFT_B_PIN] = (left_dir > 0) ? !left_a : left_a;
        encoder_left_edge(ENCODER_LEFT_A_PIN, ev);
    }
    if (right_dir != 0) {
        right_a ^= 1U;
        ev = right_a ? GPIO_IRQ_EDGE_RISE : GPIO_IRQ_EDGE_FALL;
        /* right forward = +1: needs A == B after the edge */
        host_gpio[ENCODER_RIGHT_B_PIN] = (right_dir > 0) ? right_a : !right_a;
        encoder_right_edge(ENCODER_RIGHT_A_PIN, ev);
    }
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
    int32_t lt = 0, rt = 0, cl = 0, cr = 0;

    assert(encoder_init() == E_OK);
    assert(attached == 2);
    encoder_publish_distance();
    assert(encoder_get_distance_mm() == 0);

    /* 5 wheel turns forward each: ~1036.7 mm, counts +6355 / +6330 */
    move(5 * 1271, 5 * 1266); lt += 5 * 1271; rt += 5 * 1266;
    encoder_get_counts(&cl, &cr);
    assert(cl == lt && cr == rt);
    encoder_publish_distance();
    printf("5 turns: %d mm (ref %.1f)\n", encoder_get_distance_mm(),
           ref_mm(lt, rt));
    assert(fabs(encoder_get_distance_mm() - ref_mm(lt, rt)) <= 1.0);

    /* reset_counts zeroes the task's view but not the odometer */
    encoder_reset_counts();
    encoder_get_counts(&cl, &cr);
    assert(cl == 0 && cr == 0);
    encoder_publish_distance();
    assert(fabs(encoder_get_distance_mm() - ref_mm(lt, rt)) <= 1.0);

    /* reverse 1 turn subtracts */
    move(-1271, -1266); lt -= 1271; rt -= 1266;
    encoder_get_counts(&cl, &cr);
    assert(cl == -1271 && cr == -1266);
    encoder_publish_distance();
    assert(fabs(encoder_get_distance_mm() - ref_mm(lt, rt)) <= 1.0);

    /* turning on the spot (left fwd, right back) adds ~0 */
    move(1000, -1000); lt += 1000; rt -= 1000;
    encoder_publish_distance();
    assert(fabs(encoder_get_distance_mm() - ref_mm(lt, rt)) <= 1.0);

    /* both edges pending at once (A went there and back): no count */
    encoder_get_counts(&cl, &cr);
    encoder_left_edge(ENCODER_LEFT_A_PIN, GPIO_IRQ_EDGE_BOTH);
    encoder_right_edge(ENCODER_RIGHT_A_PIN, GPIO_IRQ_EDGE_BOTH);
    {
        int32_t l2, r2;
        encoder_get_counts(&l2, &r2);
        assert(l2 == cl && r2 == cr);
    }

    /* long run: 2,000,000 counts (~327 m) without 32-bit overflow */
    move(2000000, 2000000); lt += 2000000; rt += 2000000;
    encoder_publish_distance();
    printf("long run: %d mm (ref %.1f)\n", encoder_get_distance_mm(),
           ref_mm(lt, rt));
    assert(fabs(encoder_get_distance_mm() - ref_mm(lt, rt)) <= 1.0);

    puts("PASS: IRQ encoder counting + IF-10: zero, forward, reset "
         "baseline, reverse, spin, double edge, long run");
    return 0;
}
