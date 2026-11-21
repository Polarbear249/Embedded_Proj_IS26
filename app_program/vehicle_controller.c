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
#include "subsystem3/line_drive.h"      // base duty, for SET_SPEED
#include "subsystem1/subsystem1.h"      // IF-08: s1_cmd_recv()

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
static void      vc_drain_commands(void);
static void      vc_reboot(void);
static void      vc_enter_manual(void);
static void      vc_enter_auto(void);
static void      vc_manual_drive(uint8_t direction);
static void      vc_manual_scan(void);
static void      vc_manual_watchdog(void);
static void      vc_manual_assert(uint8_t direction);

/* Names for the LINE_CMD_* movement values, for the serial log only. */
static const char *
vc_movement_name (uint8_t movement)
{
    switch (movement)
    {
    case LINE_CMD_LEFT:     return "LEFT";
    case LINE_CMD_RIGHT:    return "RIGHT";
    case LINE_CMD_STRAIGHT: return "STRAIGHT";
    case LINE_CMD_UTURN:    return "U-TURN";
    default:                return "?";
    }
}
static void      vc_obstacle(uint32_t word);
static void      vc_bypass_task(INT stacd, void *p_exinf);
static bool      vc_wait_move_done(UINT timeout_ms);
static bool      vc_line_centre_black(void);
static bool      vc_seek_line(float distance_cm, float grace_cm,
                              float extra_cm);
static void      vc_seek_stop(void);
static bool      vc_sweep_line(bool b_right_first, float max_deg);
static bool      vc_pivot_to_line(float max_deg, bool b_left);
static bool      vc_hold_arm(void);
static bool      vc_resume_line(float cap_cm, float grace_cm);
static bool      vc_restore_heading(float applied_deg);
static bool      vc_clear_estop(void);
static void      vc_release_to_line(void);
static bool      vc_wait_s3_driving(void);
static void      vc_hand_back(void);
static bool      vc_bypass_manoeuvre(uint32_t best, uint32_t centre_cm,
                                     float *p_applied_deg);
static UINT      vc_move_timeout_ms(float distance_cm);
static bool      vc_wait_profile(UINT timeout_ms);
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
/* Manual mode. g_b_bypass_busy is written by the bypass task only and is
   true from the moment it takes a profile until it is idle again; with
   g_b_bypass_pending it covers the whole bypass, so manual mode can be
   refused for its whole length. g_b_manual is written by the estop task
   only (it runs the command drain) and read by telemetry. */
static volatile bool     g_b_bypass_busy = false;
static volatile bool     g_b_manual = false;
static int16_t           g_manual_duty = DRIVE_BASE_DUTY;
static uint32_t          g_manual_hold_ticks = 0U; // estop-task ticks left
static uint8_t           g_manual_hold_dir = CMD_DRIVE_STOP; // FWD or BACK
static volatile uint32_t g_bypass_profile = 0U;
static volatile uint32_t g_alert_cm = 0U;            // centre distance of the ALERT that stopped us

// Bypass bookkeeping, touched only by vc_bypass_task and its helpers.
static char const       *g_bypass_why = "ok";        // Manouvres on this obstacle
static uint32_t          g_bypass_retries = 0U;      // Reverses on this obstacle
static uint32_t          g_bypass_attempts = 0U;     // Manouvres on this obstacle
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
        vc_drain_commands();
        vc_manual_watchdog();
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

/*
 * MQTT commands (IF-08). S1 has already checked the version, the length
 * and the ranges, so anything arriving here is well formed: this only
 * decides what the car does with it. Nothing in Buddy 2's motor code
 * changes -- STOP uses the emergency stop the GP20 button already
 * drives, and SET_SPEED moves the straight-ahead duty S3 steers around,
 * leaving its steering law and gains alone.
 */
static void
vc_drain_commands (void)
{
    T_CMD    cmd;
    uint32_t count = 0U;

    while ((count < VC_CMD_DEPTH) && (E_OK == s1_cmd_recv(&cmd, TMO_POL)))
    {
        count++;

        switch (cmd.cmd)
        {
        case CMD_STOP:
            tm_printf((UB*)"[VC] MQTT STOP\n");
            (void)s3_drive_set_enabled(false);
            (void)motion_estop();
            break;

        case CMD_START:
            tm_printf((UB*)"[VC] MQTT START\n");

            if (g_b_manual)
            {
                /* Releases the stop only: line following stays off
                   until AUTO is pressed. */
                tm_printf((UB*)"[VC] MANUAL: stop %s, still in manual\n",
                          vc_clear_estop() ? "released" : "would not clear");
            }
            else if (vc_clear_estop())
            {
                (void)s3_drive_set_enabled(true);
            }
            else
            {
                tm_printf((UB*)"[VC] MQTT START: stop would not clear\n");
            }
            break;

        case CMD_SET_SPEED:
            {
                int32_t want = (int32_t)cmd.arg * VC_CMD_DUTY_PER_CMS;
                int32_t duty = (want > DRIVE_DUTY_MAX)
                               ? DRIVE_DUTY_MAX : want;

                line_drive_set_base_duty((int16_t)duty);
                g_manual_duty = (int16_t)duty;   // manual forward/back too

                tm_printf((UB*)"[VC] MQTT SET_SPEED %u cm/s -> duty %d%s\n",
                          (UINT)cmd.arg, (INT)duty,
                          (want > DRIVE_DUTY_MAX) ? " (clamped)" : "");
            }
            break;

        case CMD_SET_MAP:
            {
                /* subsystem3.h: s3_nav_map_set() must have a single
                   writer, and names this task as that writer. */
                uint8_t mv  = CMD_MAP_MOVEMENT(cmd.arg);
                char    ltr = (char)('A' + CMD_MAP_LETTER(cmd.arg));
                ER      er  = s3_nav_map_set(mv, ltr);

                if (er == E_OK) {
                    tm_printf((UB*)"[VC] MQTT SET_MAP '%c' -> %s\n",
                              (INT)ltr, vc_movement_name(mv));
                } else {
                    tm_printf((UB*)"[VC] MQTT SET_MAP '%c' rejected\n",
                              (INT)ltr);
                }
            }
            break;

        case CMD_RESET:
            /* Same end result as the Robo Pico RST button: the firmware
               starts from the top and rejoins WiFi. Motors are cut
               first so the car is stationary while it reboots, and
               the delay lets this line reach the serial console. */
            tm_printf((UB*)"[VC] MQTT RESET: rebooting\n");
            (void)s3_drive_set_enabled(false);
            (void)motion_estop();
            (void)tk_dly_tsk(VC_RESET_FLUSH_MS);
            vc_reboot();
            break;

        case CMD_MODE:
            if (CMD_MODE_MANUAL == cmd.arg)
            {
                vc_enter_manual();
            }
            else
            {
                vc_enter_auto();
            }
            break;

        case CMD_DRIVE:
            vc_manual_drive(cmd.arg);
            break;

        case CMD_SCAN:
            vc_manual_scan();
            break;

        case CMD_PING:
            tm_printf((UB*)"[VC] MQTT PING\n");
            break;

        default:
            /* S1 rejects unknown commands; nothing to do if one slips. */
            break;
        }
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

    if (g_b_manual)
    {
        /* Manual mode turned S5's alerts off, so this is one queued from
           before: the operator is driving, so no stop and no bypass. */
        return;
    }

    if (S5_KIND_ALERT == kind)
    {
        g_alert_cm = S5_P_CENTRE(word);

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
 * How long a move may take before it counts as a stall. Generous: the
 * encoder goal ends the move, this only catches a move that never ends.
 */
static UINT
vc_move_timeout_ms (float distance_cm)
{
    return (VC_BYPASS_MOVE_EXTRA_MS
            + ((UINT)(distance_cm * 1000.0f) / VC_BYPASS_SPEED_CMS));
}

/*
 * Wait for the move in progress to end. Aborts on a GP20 press or a
 * fault, and treats a move that never ends as a stall. On failure
 * g_bypass_why says which of those it was.
 */
static bool
vc_wait_move_done (UINT timeout_ms)
{
    motion_status_t status = {0.0f, 0.0f, 0, 0, MOTION_STATE_STOPPED};
    uint32_t        waited_ms = 0U;

    while (waited_ms < timeout_ms)
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

            if (motion_is_estopped())
            {
                g_bypass_why = "obstacle during move";
                return(false);
            }

            return (true);              // the move finished normally
        }
    }

    g_bypass_why = "timeout";
    (void)motion_estop();               // never ended: treat as a stall
    return (false);
}

/* Wait for the bypass task to be handed a fresh PROFILE after a rescan. */
static bool
vc_wait_profile (UINT timeout_ms)
{
    UINT waited = 0U;

    while (waited < timeout_ms)
    {
        if (g_b_bypass_pending)
        {
            return (true);
        }

        (void)tk_dly_tsk(VC_BYPASS_POLL_MS);
        waited += (UINT)VC_BYPASS_POLL_MS;
    }

    return (false);
}

/*
 * The centre line sensor only. S3's arming rule looks at this one, so it
 * is the only bit a re-join can trust. S3 keeps publishing IF-06 while
 * it is stood down (only the drive call is gated), so the raw bits are
 * live throughout a manoeuvre.
 */
static bool
vc_line_centre_black (void)
{
    uint32_t word = s3_line_state_read();

    return (0U != ((word >> LINE_IF_RAW_C_SHIFT) & LINE_IF_RAW_MASK));
}

/*
 * [ultraline] Stop the creep and report how far the car rolled after the
 * command. That roll is the coast the landing maths depends on.
 */
static void
vc_seek_stop (void)
{
    int32_t at_mm = motion_get_distance_mm();

    (void)motion_stop();
    (void)tk_dly_tsk(300);

    tm_printf((UB*)"[VC] seek stop, coast %d mm\n",
              (int)(motion_get_distance_mm() - at_mm));
}

/*
 * Drive at the line and stop the moment the CENTRE sensor sees it.
 * Centre only: the right sensor is one spot to the side, so stopping on
 * it leaves the car un-armable. grace_cm of the approach ignores black:
 * the turn back swings the sensor across the floor and it can catch the
 * lane, a junction or a barcode at the wrong point, and stopping there
 * leaves the car short of the line. extra_cm keeps the creep running
 * after the black so the axle, which trails the sensor, can reach the
 * line too. False = distance ran out, operator aborted, or the move
 * failed; g_bypass_why says which.
 */
static bool
vc_seek_line (float distance_cm, float grace_cm, float extra_cm)
{
    motion_status_t status = {0.0f, 0.0f, 0, 0, MOTION_STATE_STOPPED};
    uint32_t        waited_ms = 0U;
    UINT            timeout_ms = vc_move_timeout_ms(distance_cm + extra_cm);
    int32_t         start_mm = motion_get_distance_mm();
    int32_t         cap_mm   = (int32_t)((distance_cm + extra_cm) * 10.0f);
    int32_t         grace_mm = (int32_t)(grace_cm * 10.0f);
    int32_t         extra_mm = (int32_t)(extra_cm * 10.0f);
    int32_t         seen_mm  = 0;
    bool            b_seen   = false;

    /*
     * [ultraline] Creep the whole leg instead of a distance move. The move
     * engine stops with a 2 to 3 cm roll, which parked the axle past the
     * tape every time. Speed mode is closed loop, so it holds the same
     * speed on a sagging supply, and its shorter roll is what lets the
     * landing repeat.
     */
    if (MOTION_OK != motion_set_speed(VC_BYPASS_CREEP_RPM,
                                      VC_BYPASS_CREEP_RPM))
    {
        g_bypass_why = "seek refused";
        return (false);
    }

    while (waited_ms < timeout_ms)
    {
        (void)tk_dly_tsk(VC_BYPASS_POLL_MS);
        waited_ms += VC_BYPASS_POLL_MS;

        if (false == b_seen)
        {
            if (((motion_get_distance_mm() - start_mm) >= grace_mm)
                && vc_line_centre_black())
            {
                b_seen  = true;
                seen_mm = motion_get_distance_mm() - start_mm;

                if (extra_mm <= 0)
                {
                    vc_seek_stop();         // cancel the rest of the creep
                    return (true);
                }
            }
        }
        else if (((motion_get_distance_mm() - start_mm) - seen_mm)
                 >= extra_mm)
        {
            /*
             * Still black after the carry that should have taken the
             * sensor off a 6 mm line: this is a black patch wider than
             * the line (a box on the floor), not the lane. Keep driving
             * instead of stopping and handing back on it.
             */
            if (vc_line_centre_black())
            {
                b_seen  = false;
                seen_mm = 0;
            }
            else
            {
                vc_seek_stop();             // axle is on the line now
                return (true);
            }
        }

        /*
         * [ultraline] Speed mode has no goal of its own, so the cap is
         * checked here. Without it the car creeps on until the timeout,
         * well past where the line should have been.
         */
        if ((motion_get_distance_mm() - start_mm) >= cap_mm)
        {
            vc_seek_stop();
            g_bypass_why = "line not found (seek distance used up)";
            return (false);
        }

        if (vc_button_pressed())
        {
            g_bypass_why = "GP20 pressed in seek";
            (void)motion_estop();
            return (false);
        }

        if (MOTION_OK != motion_get_status(&status))
        {
            (void)motion_stop();
            g_bypass_why = "status error in seek";
            return (false);
        }

        /*
         * Speed mode only leaves RUNNING if S2 faulted, so stop rather
         * than leave the creep running.
         */
        if (MOTION_STATE_RUNNING != status.state)
        {
            (void)motion_stop();
            g_bypass_why = "s2 stopped during seek";
            return (false);
        }
    }

    g_bypass_why = "line not found (seek timeout)";
    vc_seek_stop();
    return (false);
}

/*
 * Pivot up to max_deg and stop the moment the centre sensor sees the
 * line. Polling during the turn matters: a stepped sweep can jump over
 * a 6 mm line between steps, and the stop then coasts a little further.
 */
static bool
vc_pivot_to_line (float max_deg, bool b_left)
{
    motion_status_t status = {0.0f, 0.0f, 0, 0, MOTION_STATE_STOPPED};
    uint32_t        waited_ms = 0U;

    if (MOTION_OK != (b_left ? motion_turn_left(max_deg)
                             : motion_turn_right(max_deg)))
    {
        g_bypass_why = "sweep turn refused";
        return (false);
    }

    while (waited_ms < 5000U)
    {
        (void)tk_dly_tsk(VC_BYPASS_POLL_MS);
        waited_ms += VC_BYPASS_POLL_MS;

        if (vc_line_centre_black())
        {
            (void)motion_stop();
            (void)tk_dly_tsk(VC_BYPASS_POLL_MS);
            return (true);
        }

        if ((MOTION_OK != motion_get_status(&status))
            || (MOTION_STATE_RUNNING != status.state))
        {
            return (false);         // the whole angle went by
        }
    }

    (void)motion_stop();
    return (false);
}

/*
 * Search for the line within one pivot of the current heading.
 * b_right_first: which way to look first. Pivot that way up to max_deg,
 * then twice as far the other way, then back to the starting heading.
 * The heading is left where the line was found; S3 corrects the rest as
 * soon as it drives.
 */
static bool
vc_sweep_line (bool b_right_first, float max_deg)
{
    if (vc_pivot_to_line(max_deg, false == b_right_first))
    {
        return (true);
    }

    if (vc_pivot_to_line(2.0f * max_deg, b_right_first))
    {
        return (true);
    }

    return (vc_pivot_to_line(max_deg, false == b_right_first));
}

/*
 * Hold still on the line until S3 has armed itself: 200 cycles of 10 ms
 * with the debounced centre sensor black. S3's arm counter runs even
 * while the VC has it stood down, so this is all the hand-back needs.
 */
static bool
vc_hold_arm (void)
{
    uint32_t held_ms = 0U;
    uint32_t waited_ms = 0U;

    /*
     * S3 keeps its arm through the stand-down (the line task freezes its
     * lost-line timer while the VC owns the car), so there is nothing to
     * wait for once it was driving before the obstacle. Skip the 2.6 s
     * pause in that case, but only when the centre sensor really has the
     * line: the seek and the sweep both stop on a transient black, and
     * releasing S3 with the sensor off the line is what made it spin.
     */
    if (false != s3_drive_is_armed())
    {
        while (held_ms < VC_BYPASS_ARMED_CHECK_MS)
        {
            if (false == vc_line_centre_black())
            {
                break;              // transient: fall through to the hold
            }

            held_ms += VC_BYPASS_POLL_MS;
            (void)tk_dly_tsk(VC_BYPASS_POLL_MS);
        }

        if (held_ms >= VC_BYPASS_ARMED_CHECK_MS)
        {
            tm_printf((UB*)"[VC] S3 already armed\n");
            return (true);
        }

        held_ms = 0U;
    }

    while (waited_ms < VC_BYPASS_ARM_WAIT_MS)
    {
        if (vc_line_centre_black())
        {
            held_ms += VC_BYPASS_POLL_MS;

            if (held_ms >= VC_BYPASS_ARM_MS)
            {
                tm_printf((UB*)"[VC] line held, handing back\n");
                return (true);
            }
        }
        else
        {
            held_ms = 0U;
        }

        (void)tk_dly_tsk(VC_BYPASS_POLL_MS);
        waited_ms += VC_BYPASS_POLL_MS;
    }

    g_bypass_why = "line lost before S3 armed";
    return (false);
}

/*
 * Re-join: seek the line, sweep for it if the seek ran out, then hold
 * until S3 is armed. S5 alerts run only while the car is moving (seek);
 * they are off for the sweep and the hold, so a nearby object cannot
 * latch a fresh stop while the car is not going anywhere.
 */
static bool
vc_resume_line (float cap_cm, float grace_cm)
{
    bool b_found;

    if ((grace_cm < 1.0f) && vc_line_centre_black())
    {
        s5_alert_set_enabled(false);
        return (vc_hold_arm());
    }

    s5_alert_set_enabled(true);
    b_found = vc_seek_line(cap_cm, grace_cm, 0.0f);
    s5_alert_set_enabled(false);

    /*
     * The seek stops the instant black appears and then coasts: on a
     * 6 mm line the sensor can end up just past it. Pivot-search now
     * instead of holding on a line that is already behind the car.
     */
    if (false == vc_line_centre_black())
    {
        tm_printf((UB*)"[VC] re-join: %s, pivot search\n",
                  (false != b_found) ? "coasted past the line"
                                     : g_bypass_why);

        if (false == vc_sweep_line(false, VC_BYPASS_SWEEP_DEG))
        {
            return (false);
        }

        tm_printf((UB*)"[VC] re-join: line found\n");
    }

    return (vc_hold_arm());
}

/*
 * Undo the pivots a failed manoeuvre already made, so the car is back on
 * the lane heading before it reverses and re-plans. Positive angle =
 * left, the same sense as the manoeuvre.
 */
static bool
vc_restore_heading (float applied_deg)
{
    if (applied_deg > 0.0f)
    {
        if (MOTION_OK != motion_turn_right(applied_deg))
        {
            g_bypass_why = "heading restore refused";
            return (false);
        }
    }
    else if (applied_deg < 0.0f)
    {
        if (MOTION_OK != motion_turn_left(-applied_deg))
        {
            g_bypass_why = "heading restore refused";
            return (false);
        }
    }
    else
    {
        return (true);
    }

    return (vc_wait_move_done(3000U));
}

/*
 * Clear the latched stop, retrying the instant the motion lock is busy.
 * motion_clear_estop() returns MOTION_BUSY without clearing when the
 * motion task holds its command mutex; ignoring that used to look like
 * "the reverse did nothing".
 */
static bool
vc_clear_estop (void)
{
    uint32_t tries;

    for (tries = 0U; tries < 3U; tries++)
    {
        if ((MOTION_OK == motion_clear_estop()) && (false == motion_is_estopped()))
        {
            return (true);
        }

        (void)tk_dly_tsk(VC_BYPASS_POLL_MS);
    }

    return (false);
}

/* Hand the car back: clear the stop, start S3, reset this obstacle. */
static void
vc_release_to_line (void)
{
    (void)vc_clear_estop();
    (void)s3_drive_set_enabled(true);
    s5_alert_set_enabled(true);
    g_b_obstacle_stop = false;
    g_bypass_retries = 0U;
    g_bypass_attempts = 0U;
    g_alert_cm = 0U;
}

/*
 * S3 should start driving by itself within a moment of being enabled.
 * If it does not, it is not armed or not on the line: one re-join
 * attempt, then a clear stop rather than a silent park.
 */
static bool
vc_wait_s3_driving (void)
{
    motion_status_t status = {0.0f, 0.0f, 0, 0, MOTION_STATE_STOPPED};
    uint32_t        waited_ms = 0U;

    while (waited_ms < VC_BYPASS_RESUME_MS)
    {
        (void)tk_dly_tsk(VC_BYPASS_POLL_MS);
        waited_ms += VC_BYPASS_POLL_MS;

        if ((MOTION_OK == motion_get_status(&status))
            && (MOTION_STATE_RUNNING == status.state))
        {
            return (true);
        }
    }

    return (false);
}

static void
vc_hand_back (void)
{
    uint32_t try_no;

    for (try_no = 0U; try_no < 2U; try_no++)
    {
        vc_release_to_line();

        if (vc_wait_s3_driving())
        {
            tm_printf((UB*)"[VC] line drive resumes\n");
            return;
        }

        tm_printf((UB*)"[VC] S3 did not take over, re-joining again\n");
        (void)s3_drive_set_enabled(false);

        if (false == vc_resume_line(VC_BYPASS_RESUME_CM, 0.0f))
        {
            break;
        }
    }

    tm_printf((UB*)"[VC] no line drive (%s), stopped; GP20 releases\n",
              g_bypass_why);
    s5_alert_set_enabled(true);
    g_bypass_attempts = 0U;
    g_bypass_retries = 0U;
    (void)motion_estop();
    (void)s3_drive_set_enabled(true);
}

/*
 * Obstacle bypass, half-hexagon path: short 60 deg slants either side of
 * a long parallel pass. Most of the distance the car is parallel to the
 * object, only the two short slants are at an angle.
 *
 *   1. short slant out of the lane
 *   2. the same slant ends with the car clear of the object
 *   3. turn back, now parallel to the lane
 *   4. drive past the object
 *   5. turn back toward the lane
 *   6. slant back onto the lane
 *   7. turn to line up, then hand back to S3
 *
 * Stepping out first is the point. On the old diagonal the forward leg
 * kept the front sonar pointed at the object, S5 re-alerted in the middle
 * of it and aborted the move, and the car reversed instead of passing.
 * The slant puts the object beside the car for the whole pass, where the
 * front beam cannot see it. Every leg is a straight encoder move from a
 * known heading, so the same object gets the same path.
 *
 * *p_applied_deg is the net heading added so far (positive left) so a
 * failed manoeuvre can be undone with one pivot.
 */
static bool
vc_bypass_manoeuvre (uint32_t best, uint32_t centre_cm, float *p_applied_deg)
{
    bool  b_left = (S5_BEST_LEFT == best);
    float f_slant = VC_BYPASS_SIDE_CM / VC_BYPASS_SIN_TURN;
    float f_pass;

    g_bypass_why = "ok";
    *p_applied_deg = 0.0f;

    /*
     * S5 stopped the car with the e-stop latched. Every motion command
     * below is refused while it is latched, so the first turn would fail
     * with "turn refused" and send the car into the reverse-and-rescan
     * branch on every encounter. Release it before anything moves.
     */
    if (false == vc_clear_estop())
    {
        g_bypass_why = "stop would not clear";
        return (false);
    }

    /*
     * Back off only when the pivot would sweep the front corner into
     * the object. At VC_BYPASS_MIN_STOP_CM the front corner clears by
     * about 12 cm, which is enough for a 60 deg pivot.
     */
    if (centre_cm < VC_BYPASS_MIN_STOP_CM)
    {
        float f_back = (float)(VC_BYPASS_MIN_STOP_CM - centre_cm) + 2.0f;

        if (f_back > VC_BYPASS_BACK_CM)
        {
            f_back = VC_BYPASS_BACK_CM;
        }

        tm_printf((UB*)"[VC] too close (%u cm), back off %d cm first\n",
                  (UINT)centre_cm, (INT)f_back);

        if (MOTION_OK != motion_move_backward(f_back))
        {
            g_bypass_why = "back off refused";
            return (false);
        }

        if (false == vc_wait_move_done(vc_move_timeout_ms(f_back)))
        {
            return (false);
        }

        centre_cm += (uint32_t)f_back;
    }

    /*
     * The pass has to carry the rear past the object: how far we stopped
     * away, the object's depth and our own length, plus a margin.
     */
    f_pass = (float)centre_cm + VC_BYPASS_OBJ_DEPTH_CM + VC_CAR_LEN_CM
             + VC_BYPASS_PASS_MARGIN_CM
             - (f_slant * VC_BYPASS_COS_TURN);

    if (f_pass < VC_BYPASS_FWD_CM)
    {
        f_pass = VC_BYPASS_FWD_CM;
    }

    if (f_pass > VC_BYPASS_FWD_MAX_CM)
    {
        f_pass = VC_BYPASS_FWD_MAX_CM;
    }

    tm_printf((UB*)"[VC] bypass: slant %d cm, pass %d cm\n",
              (INT)f_slant, (INT)f_pass);

    // 1. Slant out of the lane
    tm_printf((UB*)"[VC] 1/7 turn %s %d deg\n", b_left ? "left" : "right",
              (INT)VC_BYPASS_TURN_DEG);

    if (MOTION_OK != (b_left ? motion_turn_left(VC_BYPASS_TURN_DEG)
                             : motion_turn_right(VC_BYPASS_TURN_DEG)))
    {
        g_bypass_why = "turn refused";
        return (false);
    }

    if (false == vc_wait_move_done(3000U))
    {
        return (false);
    }

    /*
     * [ultraline] Alerts stay OFF for the rest of the manoeuvre, including
     * the pass. The object passes at 16.5 cm, inside the 25 cm alert
     * distance, and a hit mid-leg latched a stop and refused the next
     * turn. S5 comes back on in vc_release_to_line().
     */

    *p_applied_deg = b_left ? VC_BYPASS_TURN_DEG
                            : -VC_BYPASS_TURN_DEG;

    // 2. Slant out until the car's side clears the object
    tm_printf((UB*)"[VC] 2/7 slant %d cm\n", (INT)f_slant);

    if (MOTION_OK != motion_move_forward(f_slant))
    {
        g_bypass_why = "side step refused";
        return (false);
    }

    if (false == vc_wait_move_done(vc_move_timeout_ms(f_slant)))
    {
        return (false);
    }

    // 3. Turn back to parallel with the lane
    tm_printf((UB*)"[VC] 3/7 turn %s %d deg\n", b_left ? "right" : "left",
              (INT)VC_BYPASS_TURN_DEG);

    if (MOTION_OK != (b_left ? motion_turn_right(VC_BYPASS_TURN_DEG)
                             : motion_turn_left(VC_BYPASS_TURN_DEG)))
    {
        g_bypass_why = "straighten refused";
        return (false);
    }

    if (false == vc_wait_move_done(3000U))
    {
        return (false);
    }

    *p_applied_deg = 0.0f;

    /*
     * 4. Drive past the object. It is beside the car now, so the front
     * sonar points along the lane, not at it; the alerts are already on
     * from the slant, so a new obstacle ahead still stops the move.
     */
    tm_printf((UB*)"[VC] 4/7 forward %d cm\n", (INT)f_pass);

    if (MOTION_OK != motion_move_forward(f_pass))
    {
        g_bypass_why = "forward refused";
        return (false);
    }

    if (false == vc_wait_move_done(vc_move_timeout_ms(f_pass)))
    {
        s5_alert_set_enabled(false);
        return (false);
    }

    s5_alert_set_enabled(false);

    // 5. Turn back toward the lane
    tm_printf((UB*)"[VC] 5/7 turn %s %d deg\n", b_left ? "right" : "left",
              (INT)VC_BYPASS_TURN_DEG);

    if (MOTION_OK != (b_left ? motion_turn_right(VC_BYPASS_TURN_DEG)
                             : motion_turn_left(VC_BYPASS_TURN_DEG)))
    {
        g_bypass_why = "return turn refused";
        return (false);
    }

    if (false == vc_wait_move_done(3000U))
    {
        return (false);
    }

    *p_applied_deg = b_left ? -VC_BYPASS_TURN_DEG
                            : VC_BYPASS_TURN_DEG;

    /*
     * 6. Slant back onto the lane. This is where the line matters
     * again. The centre sensor is watched while the car creeps back; on
     * black it carries on VC_BYPASS_LAND_CM more, so the axle lands on
     * the lane rather than stopping when the nose touches it.
     */
    tm_printf((UB*)"[VC] 6/7 slant back %d cm\n", (INT)f_slant);

    if (false == vc_seek_line(f_slant + VC_BYPASS_LAND_CM, 0.0f,
                              VC_BYPASS_LAND_CM))
    {
        /* No line within the slant plus one nose: still line up and let
           the check after 7/7 sweep for it. */
        tm_printf((UB*)"[VC] 6/7 no line on the return (%s)\n",
                  g_bypass_why);
    }

    // 7. Turn to line up with the lane
    tm_printf((UB*)"[VC] 7/7 align\n");

    if (MOTION_OK != (b_left ? motion_turn_left(VC_BYPASS_TURN_DEG)
                             : motion_turn_right(VC_BYPASS_TURN_DEG)))
    {
        g_bypass_why = "align refused";
        return (false);
    }

    if (false == vc_wait_move_done(3000U))
    {
        return (false);
    }

    *p_applied_deg = 0.0f;

    if (false == vc_line_centre_black())
    {
        if (false == vc_sweep_line(b_left, VC_BYPASS_ALIGN_SWEEP_DEG))
        {
            g_bypass_why = "line lost after bypass";
            return (false);
        }
    }

    return (vc_hold_arm());
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
    uint32_t plan_cm = 0U;
    float    applied_deg = 0.0f;
    bool     b_ok = true;
    /* The last boxed-in view, so an unchanged rescan does not reverse again */
    uint32_t prev_left = 0U;
    uint32_t prev_centre = 0U;
    uint32_t prev_right = 0U;
    uint32_t cur_left = 0U;
    uint32_t cur_centre = 0U;
    uint32_t cur_right = 0U;
    bool     b_prev_valid = false;
    bool     b_same_view = false;

    (void)stacd;
    (void)p_exinf;

    for (;;)
    {
        if (false == g_b_bypass_pending)
        {
            g_b_bypass_busy = false;    // manual mode: idle again
            (void)tk_dly_tsk(VC_BYPASS_POLL_MS);
        }
        else
        {
            g_b_bypass_busy = true;     // manual mode: set before pending clears
            word = g_bypass_profile;
            best = S5_P_BEST(word);
            g_b_bypass_pending = false;

            if (S5_KIND_PROFILE != S5_P_KIND(word))
            {
                // ALERT or FAULT word: nothing to act on
            }
            else if (S5_BEST_CENTRE == best)
            {
                /*
                 * Nothing to drive around. Re-join the line instead of
                 * just clearing the stop: the car may be off the line
                 * (after a boxed-in reverse) and S3 in FAILED, which
                 * would leave it parked with no error at all.
                 */
                tm_printf((UB*)"[VC] bypass: centre clear at %u cm,"
                          " re-joining\n", (UINT)S5_P_CENTRE(word));
                (void)s3_drive_set_enabled(false);
                (void)vc_clear_estop();

                if (vc_resume_line(VC_BYPASS_RESUME_CM, 0.0f))
                {
                    vc_hand_back();
                }
                else
                {
                    tm_printf((UB*)"[VC] no line to re-join (%s),"
                              " stopped\n", g_bypass_why);
                    s5_alert_set_enabled(true);
                    g_bypass_attempts = 0U;
                    g_bypass_retries = 0U;
                    (void)motion_estop();
                    (void)s3_drive_set_enabled(true);
                }
            }
            else if (S5_BEST_NONE != best)
            {
                /*
                 * Plan from the nearest distance S5 has reported: the
                 * alert that stopped us and the scan can disagree by
                 * metres (a hand in front at 7 cm, the room at 46 cm),
                 * and the alert is the one the car actually saw.
                 */
                plan_cm = S5_P_CENTRE(word);

                /*
                 * 0 (no echo) and 400 (nothing in range) are not
                 * distances to plan from. When the profile's centre is
                 * one of those, plan from the alert, which was the last
                 * real reading. Without this a no-echo centre planned a
                 * 0 cm object, the pass came out a third of its length
                 * and the car turned back before it had passed the box.
                 */
                if ((0U == plan_cm) || (plan_cm >= 400U))
                {
                    plan_cm = g_alert_cm;
                }
                else if ((g_alert_cm > 0U) && (g_alert_cm < plan_cm))
                {
                    plan_cm = g_alert_cm;
                }

                tm_printf((UB*)"[VC] bypass: best %s, L %u C %u R %u cm,"
                          " plan %u cm\n", best_name[best],
                          (UINT)S5_P_LEFT(word), (UINT)S5_P_CENTRE(word),
                          (UINT)S5_P_RIGHT(word), (UINT)plan_cm);

                g_bypass_attempts++;

                if (g_bypass_attempts > VC_BYPASS_ATTEMPTS)
                {
                    /* An alert during a forward leg queues a fresh
                       profile and starts another manoeuvre; without this
                       the car nudges forward for ever. Do not back off
                       again here: the attempt counter is reset above, so
                       backing off would repeat this every profile. */
                    g_bypass_why = "too many attempts";
                    g_bypass_attempts = 0U;
                    (void)motion_estop();
                    (void)s3_drive_set_enabled(true);
                    tm_printf((UB*)"[VC] bypass gave up after %u attempts,"
                              " stopped; GP20 releases\n",
                              (UINT)VC_BYPASS_ATTEMPTS);
                }
                else
                {
                    (void)s3_drive_set_enabled(false);
                    s5_alert_set_enabled(false);
                    b_ok = vc_bypass_manoeuvre(best, plan_cm, &applied_deg);

                    if (b_ok)
                    {
                        tm_printf((UB*)"[VC] bypass done\n");
                        vc_hand_back();
                    }
                    else
                    {
                        /*
                         * [Hotfix 3] Latching the e-stop here closed both
                         * doors: S5 only alerts while it is clear, and a
                         * profile is only queued while g_b_obstacle_stop,
                         * so the car parked for good. Undo the pivots,
                         * back off and let the scan plan again.
                         */
                        tm_printf((UB*)"[VC] bypass failed (%s), backing"
                                  " off to re-plan\n", g_bypass_why);
                        tm_printf((UB*)"[VC] s2 fault bits 0x%02x\n",
                                  (UINT)motion_get_fault_bits());

                        (void)vc_clear_estop();

                        /* Back off before turning: a pivot 20 cm from
                           the object sweeps the front into it. */
                        (void)motion_move_backward(VC_BYPASS_BACK_CM);
                        (void)vc_wait_move_done(
                            vc_move_timeout_ms(VC_BYPASS_BACK_CM));
                        (void)vc_restore_heading(applied_deg);

                        s5_alert_set_enabled(true);
                        s5_scan_request();
                        (void)vc_wait_profile(VC_BYPASS_TIMEOUT_MS);
                        g_b_obstacle_stop = true;   // so the next profile is used
                    }
                }
            }
            else
            {
                /*
                 * Boxed in. S3 stays stood down for the whole back-off:
                 * otherwise it drives forward the instant the e-stop
                 * clears, straight at the thing we stopped for, while S5
                 * is still sweeping its 2.5 s scan.
                 */
                (void)s3_drive_set_enabled(false);

                cur_left   = S5_P_LEFT(word);
                cur_centre = S5_P_CENTRE(word);
                cur_right  = S5_P_RIGHT(word);

                /*
                 * A reverse only helps if it changes what the sonar sees.
                 * Compare this scan with the one before it: if all three
                 * figures are within VC_BYPASS_SAME_CM, the car is looking
                 * at the same thing from 12 cm further back and reversing
                 * again will just repeat itself. Stop and wait for GP20
                 * instead of backing away for ever.
                 */
                b_same_view = b_prev_valid
                    && (g_bypass_retries > 0U)
                    && (cur_left   + VC_BYPASS_SAME_CM >= prev_left)
                    && (prev_left  + VC_BYPASS_SAME_CM >= cur_left)
                    && (cur_centre + VC_BYPASS_SAME_CM >= prev_centre)
                    && (prev_centre + VC_BYPASS_SAME_CM >= cur_centre)
                    && (cur_right  + VC_BYPASS_SAME_CM >= prev_right)
                    && (prev_right + VC_BYPASS_SAME_CM >= cur_right);

                if (b_same_view)
                {
                    g_bypass_retries = 0U;
                    b_prev_valid = false;
                    (void)motion_estop();
                    (void)s3_drive_set_enabled(true);
                    tm_printf((UB*)"[VC] boxed in: rescan unchanged,"
                              " not reversing again; GP20 releases\n");
                }
                else if (g_bypass_retries < VC_BYPASS_RETRIES)
                {
                    g_bypass_retries++;
                    prev_left   = cur_left;
                    prev_centre = cur_centre;
                    prev_right  = cur_right;
                    b_prev_valid = true;
                    s5_alert_set_enabled(false);

                    if (false == vc_clear_estop())
                    {
                        tm_printf((UB*)"[VC] boxed in: stop would not"
                                  " clear, not reversing\n");
                    }
                    else
                    {
                        (void)motion_move_backward(VC_BYPASS_BACK_CM);
                        (void)vc_wait_move_done(
                            vc_move_timeout_ms(VC_BYPASS_BACK_CM));
                    }

                    s5_alert_set_enabled(true);
                    s5_scan_request();
                    tm_printf((UB*)"[VC] boxed in: reversed %d cm,"
                              " rescanning (%u/%u)\n",
                              (INT)VC_BYPASS_BACK_CM,
                              (UINT)g_bypass_retries,
                              (UINT)VC_BYPASS_RETRIES);
                    if (false == vc_wait_profile(VC_BYPASS_TIMEOUT_MS))
                    {
                        g_bypass_retries = 0U;
                        (void)motion_estop();
                        (void)s3_drive_set_enabled(true);
                        tm_printf((UB*)"[VC] no profile after the rescan,"
                                  " staying stopped; GP20 releases\n");
                    }
                }
                else
                {
                    g_bypass_retries = 0U;
                    (void)motion_estop();
                    (void)s3_drive_set_enabled(true);
                    tm_printf((UB*)"[VC] boxed in after %u reverses,"
                              " staying stopped; GP20 releases\n",
                              (UINT)VC_BYPASS_RETRIES);
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

/*
 * Reboot the whole chip from software, the way the Pico SDK's
 * watchdog_reboot() does it (RP2040 datasheet 4.7, Watchdog; 2.13, PSM):
 *   - PSM_WDSEL selects what the watchdog resets: every block except the
 *     two oscillators (ROSC bit 0, XOSC bit 1), which keep running.
 *   - SCRATCH4..7 are cleared: the bootrom only jumps to a saved address
 *     when SCRATCH4 holds its magic value, so zero means a normal boot
 *     from flash, exactly as after the RST button.
 *   - Setting CTRL.TRIGGER (bit 31) fires the watchdog at once.
 * Interrupts are masked so nothing runs between the trigger and the reset.
 * Never returns.
 */
#define VC_PSM_WDSEL        (0x40010000UL + 0x08UL)  /* PSM_BASE + WDSEL */
#define VC_PSM_WDSEL_ALL    (0x0001FFFCUL)           /* bits 16..2 */
#define VC_WDOG_CTRL        (0x40058000UL + 0x00UL)  /* WATCHDOG_BASE */
#define VC_WDOG_TRIGGER     (1UL << 31)
#define VC_WDOG_SCRATCH4    (0x40058000UL + 0x1CUL)

static void
vc_reboot (void)
{
    uint32_t i;

    DI(i);
    (void)i;

    out_w(VC_WDOG_SCRATCH4 + 0x0UL, 0U);
    out_w(VC_WDOG_SCRATCH4 + 0x4UL, 0U);
    out_w(VC_WDOG_SCRATCH4 + 0x8UL, 0U);
    out_w(VC_WDOG_SCRATCH4 + 0xCUL, 0U);
    out_w(VC_PSM_WDSEL, VC_PSM_WDSEL_ALL);
    out_w(VC_WDOG_CTRL, in_w(VC_WDOG_CTRL) | VC_WDOG_TRIGGER);

    for (;;)
    {
        /* the reset lands within a few clock cycles */
    }
}

/* ---------------------------------------------------------------------
 * Manual mode: the dashboard's arrow keys drive the car (CMD_MODE,
 * CMD_DRIVE). Only the estop task calls these, from the command drain,
 * which is also the only task that can start a bypass (vc_obstacle()),
 * so the busy test below cannot race a bypass starting.
 *
 * Nothing here runs unless MANUAL is pressed: with it never pressed,
 * the car behaves exactly as before.
 * ------------------------------------------------------------------ */

bool
vc_is_manual (void)
{
    return (g_b_manual);
}

static void
vc_enter_manual (void)
{
    if (g_b_manual)
    {
        tm_printf((UB*)"[VC] MANUAL: already in manual\n");
        return;
    }

    if (g_b_bypass_busy || g_b_bypass_pending)
    {
        /* The bypass drives the motors itself and, when it ends, turns
           line following and S5's alerts back on: it would fight the
           operator. */
        tm_printf((UB*)"[VC] MANUAL refused: going round an obstacle."
                  " Press Stop, wait a few seconds, try again\n");
        return;
    }

    g_b_manual = true;          // first: a queued S5 word is now ignored
    g_b_obstacle_stop = false;  // as GP20 does: the operator takes over
    s5_alert_set_enabled(false);
    (void)s3_drive_set_enabled(false);
    (void)motion_stop();

    if (!vc_clear_estop())
    {
        tm_printf((UB*)"[VC] MANUAL: stop would not clear, press Start\n");
    }

    tm_printf((UB*)"[VC] MANUAL: line following and obstacle"
              " avoidance OFF\n");
}

static void
vc_enter_auto (void)
{
    bool b_was_manual = g_b_manual;

    if (g_b_bypass_busy || g_b_bypass_pending)
    {
        /* Already AUTO and mid-bypass: handing back now would restart
           the line drive under the bypass. */
        tm_printf((UB*)"[VC] AUTO: going round an obstacle, ignored\n");
        return;
    }

    g_b_manual = false;
    g_manual_hold_dir = CMD_DRIVE_STOP;
    g_manual_hold_ticks = 0U;
    (void)motion_stop();
    vc_release_to_line();       // stop cleared, S3 on, S5 alerts on

    tm_printf((UB*)"[VC] AUTO: line following and obstacle avoidance"
              " ON%s\n", b_was_manual ? "" : " (was already auto)");
}

/* Forward / back while held: called on each command and re-asserted every
   estop-task tick by vc_manual_watchdog(). Repeats are harmless: same-
   direction speed keeps the PI state (no re-kick), and a running reverse
   move answers MOTION_BUSY without changing anything. */
static void
vc_manual_assert (uint8_t direction)
{
    float rpm = (float)g_manual_duty / (float)S3_DRIVE_DUTY_PER_RPM;

    if (rpm > VC_MANUAL_RPM_MAX)
    {
        rpm = VC_MANUAL_RPM_MAX;  // same as line following at this Speed
    }

    if (CMD_DRIVE_FWD == direction)
    {
        (void)motion_set_speed(rpm, rpm);
    }
    else if (CMD_DRIVE_BACK == direction)
    {
        (void)motion_move_backward(VC_MANUAL_BACK_CM);
    }
    else
    {
        // turns are steps, not held
    }
}

static void
vc_manual_drive (uint8_t direction)
{
    static uint8_t last_direction = CMD_DRIVE_STOP;
    static char const * const name[] =
    {
        "stop", "forward", "back", "left", "right"
    };

    if (!g_b_manual)
    {
        return;                 // AUTO: the line drive owns the motors
    }

    if ((CMD_DRIVE_FWD == direction) || (CMD_DRIVE_BACK == direction))
    {
        if ((g_manual_hold_dir != direction) &&
            (CMD_DRIVE_STOP != g_manual_hold_dir))
        {
            (void)motion_stop();  // forward <-> back: end the old one first
        }

        /* The operator is holding the key: renew on arrival, whatever
           S2 answered. */
        g_manual_hold_dir = direction;
        g_manual_hold_ticks = VC_MANUAL_LEASE_MS / VC_ESTOP_POLL_MS;
        vc_manual_assert(direction);
    }
    else
    {
        if (CMD_DRIVE_STOP != g_manual_hold_dir)
        {
            (void)motion_stop();  // leaving forward / back
        }

        g_manual_hold_dir = CMD_DRIVE_STOP;
        g_manual_hold_ticks = 0U;

        if (CMD_DRIVE_LEFT == direction)
        {
            /* MOTION_BUSY while a step is still turning: a held key's
               repeats wait for it, then start the next step. */
            (void)motion_turn_left(VC_MANUAL_TURN_DEG);
        }
        else if (CMD_DRIVE_RIGHT == direction)
        {
            (void)motion_turn_right(VC_MANUAL_TURN_DEG);
        }
        else
        {
            (void)motion_stop();
        }
    }

    /* A held key repeats about 10 times a second: log changes only. */
    if ((direction != last_direction) && (direction <= CMD_DRIVE_RIGHT))
    {
        tm_printf((UB*)"[VC] MANUAL %s\n", name[direction]);
        last_direction = direction;
    }
}

/*
 * One servo scan in manual mode, so the dashboard's obstacle card can be
 * refreshed while the operator drives. Uses S5's own s5_scan_request()
 * (the bypass uses it to rescan): S5 sweeps left / centre / right once
 * and sends a PROFILE. vc_obstacle() ignores that word in manual mode, so
 * the car neither stops nor drives round anything; the profile only
 * updates s5_get_obstacle() for telemetry. After the scan S5's
 * wait-for-release step returns to pinging at once, because nothing
 * latched a stop. Refused while the car moves: S5 only scans a
 * stationary car (Tribunal 1), a scan in motion reads wrong.
 */
static void
vc_manual_scan (void)
{
    motion_status_t status = {0.0f, 0.0f, 0, 0, MOTION_STATE_STOPPED};

    if (!g_b_manual)
    {
        tm_printf((UB*)"[VC] SCAN: manual mode only (in auto the"
                  " scanner runs by itself)\n");
        return;
    }

    if ((MOTION_OK != motion_get_status(&status))
        || (MOTION_STATE_RUNNING == status.state))
    {
        tm_printf((UB*)"[VC] SCAN refused: stop the car first\n");
        return;
    }

    s5_scan_request();
    tm_printf((UB*)"[VC] MANUAL scan requested\n");
}

/*
 * motion_set_speed() has no lease, so manual forward gets one here: the
 * estop task calls this every VC_ESTOP_POLL_MS, and when no forward
 * command has renewed the count for VC_MANUAL_LEASE_MS the motors stop.
 * Covers a key-up that never arrived, a dead dashboard and WiFi loss.
 * Same task as the command drain, so no lock is needed.
 */
static void
vc_manual_watchdog (void)
{
    if (CMD_DRIVE_STOP == g_manual_hold_dir)
    {
        return;
    }

    if (g_manual_hold_ticks > 0U)
    {
        g_manual_hold_ticks--;
    }

    if (0U == g_manual_hold_ticks)
    {
        (void)motion_stop();
        g_manual_hold_dir = CMD_DRIVE_STOP;
        tm_printf((UB*)"[VC] MANUAL: no command for %u ms, stopped\n",
                  (UINT)VC_MANUAL_LEASE_MS);
    }
    else
    {
        vc_manual_assert(g_manual_hold_dir);   // retry a BUSY within 20 ms
    }
}
