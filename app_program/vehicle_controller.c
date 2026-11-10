/*
 * vehicle_controller.c
 * Vehicle Controller, see vehicle_controller.h. So far: the GP20
 * emergency stop, the S5 obstacle stop (IF-09) and the test drive.
 */

#include "vehicle_controller.h"
#include <stdbool.h>
#include "subsystem2/motion_control.h"
#include "subsystem5/subsystem5.h"      // IF-09 word layout
#include "subsystem4/imu_terrain.h"
#include "subsystem4/imu_hump.h"
#include "subsystem3/subsystem3.h"      // S3_DRIVE_ENABLE

#if VC_TEST_DRIVE && S3_DRIVE_ENABLE
// Two motor commanders: turn one off (S3_DRIVE_SWITCH in subsystem3.h)
#error "VC_TEST_DRIVE and S3_DRIVE_ENABLE are both on"
#endif

#define VC_MM_PER_TURN          (207)       // pi x 66 mm wheel
#define VC_MS_PER_MIN           (60000)
#define VC_LIMIT_EXTRA_MS       (3000)

#if VC_TEST_DRIVE
typedef enum
{
    VC_STOP_DISTANCE = 0,
    VC_STOP_BUTTON,
    VC_STOP_TIME,
    VC_STOP_IMU_FAULT,
    VC_STOP_MOTION_FAULT,
    VC_STOP_REFUSED
} vc_stop_t;

static char const * const g_stop_name[] =
{
    "distance reached", "button", "time limit", "IMU fault",
    "Motion fault (wheel stalled?)", "Motion refused the command"
};

static void      vc_test_task(INT stacd, void *p_exinf);
static void      vc_wait_press(void);
static vc_stop_t vc_drive(int32_t *p_driven_mm);
#else
static void      vc_estop_task(INT stacd, void *p_exinf);
static void      vc_toggle_estop(void);
static void      vc_drain_profiles(void);
static void      vc_obstacle(uint32_t word);
static void      vc_bypass_task(INT stacd, void *p_exinf);
static bool      vc_wait_move_done(void);
static bool      vc_line_black_now(void);
static bool      vc_seek_line(void);
static bool      vc_bypass_manoeuvre(uint32_t best);
#endif

static bool      vc_button_pressed(void);

EXPORT ID vc_mbf_profile = 0;

/*
 * Obstacle bypass state, normal builds only. The estop task queues a
 * profile and the bypass task acts on it: one writer each, so the
 * volatile words need no lock.
 */
#if !VC_TEST_DRIVE
static volatile bool     g_b_obstacle_stop = false;   // the last stop was ours
static volatile bool     g_b_bypass_pending = false;
static volatile uint32_t g_bypass_profile = 0U;

// Bypass bookkeeping, touched only by vc_bypass_task and its helpers.
static char const       *g_bypass_why = "ok";         // why the last move ended
#endif

/*!
 * Button input with pull-up (pressed = low), and the IF-09 message
 * buffer (owner: VC, receiver; one uint32_t per message, depth 8).
 */
ER
vc_init (void)
{
    ER     er = E_OK;
    T_CMBF cmbf = {0};

    // GP20 (test drive or emergency stop, depending on the build)
    er = gpio_set_pin(VC_BUTTON_PIN, GPIO_MODE_IN);
    set_w(GPIO(VC_BUTTON_PIN), GPIO_PUE);

    if (E_OK <= er)
    {
        cmbf.mbfatr = TA_TFIFO;
        cmbf.maxmsz = (INT)sizeof(uint32_t);
        // each message also stores a size header (sizeof(INT))
        cmbf.bufsz = (SZ)(VC_PROFILE_DEPTH
                          * (sizeof(uint32_t) + sizeof(INT)));
        vc_mbf_profile = tk_cre_mbf(&cmbf);
        er = (vc_mbf_profile < E_OK) ? (ER)vc_mbf_profile : E_OK;
    }

    return (er);
}

ER
vc_start (void)
{
    ER er = E_OK;

#if VC_TEST_DRIVE && !APP_TEST
    ID tid = app_start_task((FP)vc_test_task, VC_TEST_PRI, VC_TEST_PRC,
                            "vc_test_drive");

    er = (tid < E_OK) ? (ER)tid : E_OK;
#elif !APP_TEST
    ID tid = app_start_task((FP)vc_estop_task, VC_ESTOP_PRI, 0,
                            "vc_estop");

    er = (tid < E_OK) ? (ER)tid : E_OK;

    if (E_OK <= er)
    {
        tid = app_start_task((FP)vc_bypass_task, VC_BYPASS_PRI, 0,
                             "vc_bypass");
        er = (tid < E_OK) ? (ER)tid : E_OK;
    }
#endif

    return (er);
}

#if VC_TEST_DRIVE
static void
vc_test_task (INT stacd, void *p_exinf)
{
    int32_t   driven_mm = 0;
    uint32_t  word = 0U;
    vc_stop_t why = VC_STOP_DISTANCE;

    (void)stacd;
    (void)p_exinf;

    for (;;)
    {
        while (IMU_TERRAIN_ACTIVE != imu_terrain_get_state())
        {
            tk_dly_tsk(VC_POLL_MS);     // level zero first, car still
        }

        tm_printf((UB*)"[VC] test drive ready: press GP20 to drive %d mm"
                  " at %d RPM (press again to stop)\n",
                  (INT)VC_TEST_DIST_MM, (INT)VC_TEST_RPM);
        vc_wait_press();

        why = vc_drive(&driven_mm);
        word = imu_terrain_get_hump_word();
        tm_printf((UB*)"[VC] stopped: %s, drove %d mm | humps: last %u mm,"
                  " highest %u mm, valid %u\n", g_stop_name[why],
                  (INT)driven_mm, (UINT)IMU_HUMP_W_LAST(word),
                  (UINT)IMU_HUMP_W_MAX(word), (UINT)IMU_HUMP_W_VALID(word));
    }
}

// Press (low on two polls in a row, debounce) and release
static void
vc_wait_press (void)
{
    uint32_t low = 0U;

    while (low < 2U)
    {
        tk_dly_tsk(VC_POLL_MS);
        low = vc_button_pressed() ? (low + 1U) : 0U;
    }

    while (vc_button_pressed())
    {
        tk_dly_tsk(VC_POLL_MS);
    }
}

/*
 * One run. Always ends with motion_stop(), whatever stopped it.
 */
static vc_stop_t
vc_drive (int32_t *p_driven_mm)
{
    motion_status_t status = {0.0f, 0.0f, 0, 0, MOTION_STATE_STOPPED};
    vc_stop_t       why = VC_STOP_DISTANCE;
    bool            b_done = false;
    int32_t         start_mm = motion_get_distance_mm();
    int32_t         limit_ms = ((VC_TEST_DIST_MM * VC_MS_PER_MIN * 2)
                                / (VC_TEST_RPM * VC_MM_PER_TURN))
                               + VC_LIMIT_EXTRA_MS;
    int32_t         elapsed_ms = 0;

    if (MOTION_OK != motion_set_speed((float)VC_TEST_RPM,
                                      (float)VC_TEST_RPM))
    {
        why = VC_STOP_REFUSED;
        b_done = true;
    }

    while (false == b_done)
    {
        tk_dly_tsk(VC_POLL_MS);
        elapsed_ms += VC_POLL_MS;
        *p_driven_mm = motion_get_distance_mm() - start_mm;
        (void)motion_get_status(&status);

        if (*p_driven_mm >= VC_TEST_DIST_MM)
        {
            why = VC_STOP_DISTANCE;
            b_done = true;
        }
        else if (vc_button_pressed())
        {
            why = VC_STOP_BUTTON;
            b_done = true;
        }
        else if (elapsed_ms >= limit_ms)
        {
            why = VC_STOP_TIME;
            b_done = true;
        }
        else if (IMU_TERRAIN_FAULT == imu_terrain_get_state())
        {
            why = VC_STOP_IMU_FAULT;
            b_done = true;
        }
        else if (MOTION_STATE_FAULT == status.state)
        {
            why = VC_STOP_MOTION_FAULT;
            b_done = true;
        }
        else
        {
            // keep driving
        }
    }

    (void)motion_stop();
    *p_driven_mm = motion_get_distance_mm() - start_mm;

    while (vc_button_pressed())
    {
        tk_dly_tsk(VC_POLL_MS);     // a stop press must not start a run
    }

    return (why);
}
#endif // VC_TEST_DRIVE

#if !VC_TEST_DRIVE
/*
 * GP20 emergency stop. One action per press (on the poll where the
 * button has been low VC_ESTOP_DEBOUNCE times in a row); holding the
 * button does not repeat it.
 */
static void
vc_estop_task (INT stacd, void *p_exinf)
{
    uint32_t low = 0U;

    (void)stacd;
    (void)p_exinf;

    tm_printf((UB*)"[VC] GP20 = emergency stop (press again to release)\n");

    for (;;)
    {
        tk_dly_tsk(VC_ESTOP_POLL_MS);

        if (vc_button_pressed())
        {
            low = (low <= VC_ESTOP_DEBOUNCE) ? (low + 1U) : low;
        }
        else
        {
            low = 0U;
        }

        if (VC_ESTOP_DEBOUNCE == low)
        {
            vc_toggle_estop();
        }

        vc_drain_profiles();
    }
}

/*
 * Every IF-09 word waiting, never blocking (TMO_POL). At most the
 * buffer depth per poll, so a fast producer cannot hold this task.
 */
static void
vc_drain_profiles (void)
{
    uint32_t word = 0U;
    uint32_t count = 0U;
    bool     b_more = true;

    while (b_more && (count < VC_PROFILE_DEPTH))
    {
        // returns the size, or E_TMOUT when empty
        b_more = ((INT)sizeof(word)
                  == tk_rcv_mbf(vc_mbf_profile, &word, TMO_POL));

        if (b_more)
        {
            vc_obstacle(word);
        }

        count++;
    }
}

static void
vc_obstacle (uint32_t word)
{
    static char const * const best_name[] =
    {
        "left", "centre", "right", "none"
    };
    uint32_t kind = S5_P_KIND(word);

    if (S5_KIND_ALERT == kind)
    {
        if (motion_is_estopped())
        {
            tm_printf((UB*)"[VC] obstacle at %u cm (already stopped)\n",
                      (UINT)S5_P_CENTRE(word));
        }
        else
        {
            tm_printf((UB*)"[VC] obstacle at %u cm: STOP (%d), GP20"
                      " releases\n", (UINT)S5_P_CENTRE(word),
                      (INT)motion_estop());
            g_b_obstacle_stop = true;
        }
    }
    else if (S5_KIND_PROFILE == kind)
    {
        tm_printf((UB*)"[VC] obstacle profile L %u C %u R %u cm,"
                  " best %s\n", (UINT)S5_P_LEFT(word),
                  (UINT)S5_P_CENTRE(word), (UINT)S5_P_RIGHT(word),
                  best_name[S5_P_BEST(word)]);

        if (g_b_obstacle_stop)
        {
            g_bypass_profile = word;
            g_b_bypass_pending = true;
        }
    }
    else if (S5_KIND_FAULT == kind)
    {
        tm_printf((UB*)"[VC] ultrasonic: no echo (missing or dead);"
                  " not stopping\n");
    }
    else
    {
        // unused kind code: ignore
    }
}

static void
vc_toggle_estop (void)
{
    motion_result_t result = MOTION_OK;

    g_b_bypass_pending = false;     // the operator takes the car back
    g_b_obstacle_stop = false;

    if (motion_is_estopped())
    {
        result = motion_clear_estop();
        tm_printf((UB*)"[VC] GP20: stop released (%d); the line drive"
                  " resumes if it is still on the line\n", (INT)result);
    }
    else
    {
        result = motion_estop();

        if (MOTION_OK == result)
        {
            tm_printf((UB*)"[VC] GP20: EMERGENCY STOP, motors off, drive"
                      " commands refused\n");
        }
        else
        {
            tm_printf((UB*)"[VC] GP20: emergency stop NOT latched (%d),"
                      " motors zeroed once: press again\n", (INT)result);
        }
    }
}

/*
 * Wait for the move in progress to end. Aborts on a GP20 press or a
 * fault, and treats a move that never ends as a stall. On failure
 * g_bypass_why says which of those it was.
 */
static bool
vc_wait_move_done (void)
{
    motion_status_t status = {0.0f, 0.0f, 0, 0, MOTION_STATE_STOPPED};
    uint32_t        waited_ms = 0U;

    while (waited_ms < VC_BYPASS_TIMEOUT_MS)
    {
        (void)tk_dly_tsk(VC_BYPASS_POLL_MS);
        waited_ms += VC_BYPASS_POLL_MS;

        if (vc_button_pressed())
        {
            g_bypass_why = "GP20 pressed";
            (void)motion_estop();       // the operator aborts the manoeuvre
            return (false);
        }

        if (MOTION_OK != motion_get_status(&status))
        {
            g_bypass_why = "status error";
            return (false);
        }

        if (MOTION_STATE_RUNNING != status.state)
        {
            if (MOTION_STATE_FAULT == status.state)
            {
                g_bypass_why = "motion fault";
                return (false);
            }

            return (true);              // the move finished normally
        }
    }

    g_bypass_why = "timeout";
    (void)motion_estop();               // never ended: treat as a stall
    return (false);
}

/*
 * The line is under the car now. S3 keeps publishing IF-06 while it is
 * stood down (only the drive call is gated), so the raw sensor bits are
 * still live and can be read from here.
 */
static bool
vc_line_black_now (void)
{
    uint32_t word = s3_line_state_read();

    return (0U != ((word >> LINE_IF_RAW_C_SHIFT) & LINE_IF_RAW_MASK))
           || (0U != ((word >> LINE_IF_RAW_R_SHIFT) & LINE_IF_RAW_MASK));
}

/*
 * Drive at the line and stop the moment a sensor sees it.
 * False = the distance ran out, the operator aborted, or the move
 * failed; g_bypass_why says which.
 */
static bool
vc_seek_line (void)
{
    motion_status_t status = {0.0f, 0.0f, 0, 0, MOTION_STATE_STOPPED};
    uint32_t        waited_ms = 0U;

    if (MOTION_OK != motion_move_forward(VC_BYPASS_SEEK_CM))
    {
        g_bypass_why = "seek refused";
        return (false);
    }

    while (waited_ms < VC_BYPASS_SEEK_MS)
    {
        (void)tk_dly_tsk(VC_BYPASS_POLL_MS);
        waited_ms += VC_BYPASS_POLL_MS;

        if (vc_line_black_now())
        {
            (void)motion_stop();        // cancel the rest of the approach
            (void)tk_dly_tsk(VC_BYPASS_POLL_MS);
            return (true);
        }

        if (vc_button_pressed())
        {
            g_bypass_why = "GP20 pressed in seek";
            (void)motion_estop();
            return (false);
        }

        if (MOTION_OK != motion_get_status(&status))
        {
            g_bypass_why = "status error in seek";
            return (false);
        }

        if (MOTION_STATE_RUNNING != status.state)
        {
            g_bypass_why = "line not found (seek distance used up)";
            return (false);
        }
    }

    g_bypass_why = "line not found (seek timeout)";
    (void)motion_stop();
    return (false);
}

/*
 * Turn out, drive past, swing back across the line, seek it, straighten
 * up. The e-stop must already be released.
 */
static bool
vc_bypass_manoeuvre (uint32_t best)
{
    bool  b_left = (S5_BEST_LEFT == best);
    float f_turn = b_left ? S5_BYPASS_TURN_L : S5_BYPASS_TURN_R;

    g_bypass_why = "ok";

    if (MOTION_OK != motion_clear_estop())
    {
        g_bypass_why = "estop clear refused";
        return (false);
    }

    // 1. Out of the lane
    if (MOTION_OK != (b_left ? motion_turn_left(f_turn)
                             : motion_turn_right(f_turn)))
    {
        g_bypass_why = "turn refused";
        return (false);
    }

    if (false == vc_wait_move_done())
    {
        return (false);
    }

    // 2. Past the obstacle
    if (MOTION_OK != motion_move_forward(VC_BYPASS_FWD_CM))
    {
        g_bypass_why = "forward refused";
        return (false);
    }

    if (false == vc_wait_move_done())
    {
        return (false);
    }

    // 3. Back across the line: two turn angles, so the car points at it
    if (MOTION_OK != (b_left ? motion_turn_right(2.0f * f_turn)
                             : motion_turn_left(2.0f * f_turn)))
    {
        g_bypass_why = "return turn refused";
        return (false);
    }

    if (false == vc_wait_move_done())
    {
        return (false);
    }

    // 4. Drive until a sensor is on the line
    if (false == vc_seek_line())
    {
        return (false);
    }

    // 5. Straighten up
    if (MOTION_OK != (b_left ? motion_turn_left(f_turn)
                             : motion_turn_right(f_turn)))
    {
        g_bypass_why = "straighten refused";
        return (false);
    }

    return (vc_wait_move_done());
}

/*
 * Obstacle bypass (TEAM.md 3 item 7): one manoeuvre per PROFILE. S3 and
 * S5 are stood down first, so neither the line drive nor an S5 re-alert
 * can cancel a move; both are re-armed on every exit path.
 */
static void
vc_bypass_task (INT stacd, void *p_exinf)
{
    static char const * const best_name[] =
    {
        "left", "centre", "right", "none"
    };
    uint32_t word = 0U;
    uint32_t best = S5_BEST_NONE;
    bool     b_ok = true;

    (void)stacd;
    (void)p_exinf;

    for (;;)
    {
        if (false == g_b_bypass_pending)
        {
            (void)tk_dly_tsk(VC_BYPASS_POLL_MS);
        }
        else
        {
            word = g_bypass_profile;
            best = S5_P_BEST(word);
            g_b_bypass_pending = false;

            if ((S5_KIND_PROFILE != S5_P_KIND(word))
                || (S5_BEST_NONE == best))
            {
                tm_printf((UB*)"[VC] bypass: no passable direction"
                          " (C %u cm), staying stopped; GP20 releases\n",
                          (UINT)S5_P_CENTRE(word));
            }
            else
            {
                tm_printf((UB*)"[VC] bypass: best %s, L %u C %u R %u cm\n",
                          best_name[best], (UINT)S5_P_LEFT(word),
                          (UINT)S5_P_CENTRE(word), (UINT)S5_P_RIGHT(word));

                (void)s3_drive_set_enabled(false);
                s5_alert_set_enabled(false);

                b_ok = vc_bypass_manoeuvre(best);

                if (false == b_ok)
                {
                    (void)motion_estop();   // stop before handing back
                }

                s5_alert_set_enabled(true);
                (void)s3_drive_set_enabled(true);
                g_b_obstacle_stop = false;

                if (b_ok)
                {
                    tm_printf((UB*)"[VC] bypass done, line drive"
                              " resumes\n");
                }
                else
                {
                    tm_printf((UB*)"[VC] bypass failed (%s), stopped;"
                              " GP20 releases\n", g_bypass_why);
                }
            }
        }
    }
}
#endif // !VC_TEST_DRIVE

// Robo Pico button, pressed = low (input with pull-up)
static bool
vc_button_pressed (void)
{
    return (0U == gpio_get_val(VC_BUTTON_PIN));
}