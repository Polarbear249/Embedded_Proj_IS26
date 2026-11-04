/*
 * obstacle.c
 * Subsystem 5 (Buddy 5): the obstacle task (forward ranging, coarse and
 * fine scan, profile to the Vehicle Controller), see subsystem5.h.
 *
 * Owns: the HC-SR04+ (ultrasonic.c) and PWM slice 6 (servo.c). Reads
 * only: motion_is_estopped() and motion_get_distance_mm() (IF-10), to
 * know the car has stopped. Sends: IF-09 words to vc_mbf_profile with
 * TMO_POL (never blocks; a full buffer drops the word, the next ping
 * sends again). Worst-case blocking: none besides its own sleeps.
 * Integer only (no FPU).
 */

#include "obstacle.h"
#include <stdbool.h>
#include "subsystem5.h"
#include "ultrasonic.h"
#include "servo.h"
#include "../vehicle_controller.h"          // vc_mbf_profile (IF-09)
#include "../subsystem2/motion_control.h"   // read-only status

#define S5_SETTLE_MS            (500)       // servo move, B5's value
#define S5_POLL_MS              (100)
#define S5_STOP_WAIT_POLLS      (10U)       // 1 s for the VC to stop us
#define S5_STILL_POLLS          (2U)        // distance steady 200 ms
#define S5_DIRECTIONS           (3U)

typedef enum
{
    S5_STATE_FORWARD = 0,       // ping straight ahead
    S5_STATE_WAIT_STOP,         // alert sent, wait for a stationary car
    S5_STATE_SCAN,              // coarse + fine scan, send the profile
    S5_STATE_WAIT_RELEASE       // until the VC releases the stop
} s5_state_t;

static uint32_t const g_scan_angle[S5_DIRECTIONS] =
{
    S5_ANGLE_LEFT, S5_ANGLE_CENTRE, S5_ANGLE_RIGHT
};

static s5_state_t obstacle_forward(void);
static s5_state_t obstacle_wait_stop(void);
static s5_state_t obstacle_scan(void);
static s5_state_t obstacle_wait_release(void);
static void       obstacle_send(uint32_t word);

void
obstacle_task (INT stacd, void *exinf)
{
    s5_state_t state = S5_STATE_FORWARD;

    (void)stacd;
    (void)exinf;

    servo_set_angle(S5_ANGLE_CENTRE);
    (void)tk_dly_tsk(S5_SETTLE_MS);
    tm_printf((UB*)"[S5] obstacle task: alert below %u cm\n",
              (UINT)S5_ALERT_CM);

    for (;;)
    {
        switch (state)
        {
            case S5_STATE_FORWARD:
                state = obstacle_forward();
                break;

            case S5_STATE_WAIT_STOP:
                state = obstacle_wait_stop();
                break;

            case S5_STATE_SCAN:
                state = obstacle_scan();
                break;

            case S5_STATE_WAIT_RELEASE:
                state = obstacle_wait_release();
                break;

            default:
                state = S5_STATE_FORWARD;
                break;
        }
    }
}

/*!
 * IF-09 word, see subsystem5.h. Inputs are clamped/masked to their
 * widths, never wrapped into a neighbour field.
 */
uint32_t
s5_profile_pack (uint32_t left_cm, uint32_t centre_cm, uint32_t right_cm,
                 uint32_t kind, uint32_t best)
{
    uint32_t left = (left_cm > S5_DIST_MASK) ? S5_DIST_MASK : left_cm;
    uint32_t centre = (centre_cm > S5_DIST_MASK) ? S5_DIST_MASK
                                                 : centre_cm;
    uint32_t right = (right_cm > S5_DIST_MASK) ? S5_DIST_MASK : right_cm;

    return (left | (centre << 9) | (right << 18) | ((kind & 0x3U) << 27)
            | ((best & 0x3U) << 29));
}

/*
 * One forward ping (one cycle). Counters are static: they live across
 * calls, and only this task touches them.
 */
static s5_state_t
obstacle_forward (void)
{
    static uint32_t s_close = 0U;
    static uint32_t s_silent = 0U;
    s5_state_t      next = S5_STATE_FORWARD;
    uint32_t        dist_cm = ultrasonic_measure_cm();

    if (ULTRA_NO_ECHO == dist_cm)
    {
        s_close = 0U;

        if (s_silent < S5_NO_ECHO_PINGS)
        {
            s_silent++;

            if (S5_NO_ECHO_PINGS == s_silent)
            {
                obstacle_send(s5_profile_pack(0U, 0U, 0U, S5_KIND_FAULT,
                                              S5_BEST_NONE));
            }
        }
    }
    else
    {
        s_silent = 0U;
        s_close = (dist_cm < S5_ALERT_CM) ? (s_close + 1U) : 0U;

        if (s_close >= S5_CONFIRM_PINGS)
        {
            s_close = 0U;
            obstacle_send(s5_profile_pack(0U, dist_cm, 0U, S5_KIND_ALERT,
                                          S5_BEST_NONE));
            next = S5_STATE_WAIT_STOP;
        }
    }

    return (next);
}

/*
 * Scan only once the car is stopped AND stationary (Tribunal 1): the
 * VC's stop latched and the IF-10 distance steady for S5_STILL_POLLS.
 * If the VC does not stop the car within S5_STOP_WAIT_POLLS (it is not
 * running, e.g. a test build), go back to pinging.
 */
static s5_state_t
obstacle_wait_stop (void)
{
    s5_state_t next = S5_STATE_FORWARD;
    uint32_t   polls = 0U;
    uint32_t   still = 0U;
    int32_t    last_mm = motion_get_distance_mm();
    int32_t    now_mm = last_mm;
    bool       b_done = false;

    while ((false == b_done) && (polls < S5_STOP_WAIT_POLLS))
    {
        (void)tk_dly_tsk(S5_POLL_MS);
        polls++;
        now_mm = motion_get_distance_mm();
        still = (motion_is_estopped() && (now_mm == last_mm))
                ? (still + 1U) : 0U;
        last_mm = now_mm;

        if (still >= S5_STILL_POLLS)
        {
            next = S5_STATE_SCAN;
            b_done = true;
        }
    }

    return (next);
}

/*
 * Coarse scan: one ping per direction. Fine scan: S5_FINE_SAMPLES pings
 * in the best direction, averaged over the valid ones. Then re-centre
 * and send the profile.
 */
static s5_state_t
obstacle_scan (void)
{
    uint32_t dist_cm[S5_DIRECTIONS] = {0U, 0U, 0U};
    uint32_t best = S5_BEST_NONE;
    uint32_t best_cm = 0U;
    uint32_t sum_cm = 0U;
    uint32_t valid = 0U;
    uint32_t sample;
    uint32_t idx;

    for (idx = 0U; idx < S5_DIRECTIONS; idx++)
    {
        servo_set_angle(g_scan_angle[idx]);
        (void)tk_dly_tsk(S5_SETTLE_MS);
        dist_cm[idx] = ultrasonic_measure_cm();

        if (dist_cm[idx] > best_cm)
        {
            best_cm = dist_cm[idx];
            best = idx;     // S5_BEST_LEFT / _CENTRE / _RIGHT = index
        }
    }

    if (S5_BEST_NONE != best)
    {
        servo_set_angle(g_scan_angle[best]);
        (void)tk_dly_tsk(S5_SETTLE_MS);

        for (sample = 0U; sample < S5_FINE_SAMPLES; sample++)
        {
            uint32_t one_cm = ultrasonic_measure_cm();  // one cycle each

            if (ULTRA_NO_ECHO != one_cm)
            {
                sum_cm += one_cm;
                valid++;
            }
        }

        dist_cm[best] = (0U != valid) ? (sum_cm / valid) : dist_cm[best];
    }

    servo_set_angle(S5_ANGLE_CENTRE);
    (void)tk_dly_tsk(S5_SETTLE_MS);
    obstacle_send(s5_profile_pack(dist_cm[0], dist_cm[1], dist_cm[2],
                                  S5_KIND_PROFILE, best));

    return (S5_STATE_WAIT_RELEASE);
}

static s5_state_t
obstacle_wait_release (void)
{
    s5_state_t next = S5_STATE_WAIT_RELEASE;

    (void)tk_dly_tsk(S5_POLL_MS);

    if (false == motion_is_estopped())
    {
        next = S5_STATE_FORWARD;
    }

    return (next);
}

// By value, never blocks: a full buffer (VC not draining) drops it
static void
obstacle_send (uint32_t word)
{
    (void)tk_snd_mbf(vc_mbf_profile, &word, (INT)sizeof(word), TMO_POL);
}
