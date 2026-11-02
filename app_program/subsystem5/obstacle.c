#include "obstacle.h"
#include "ultrasonic.h"
#include "servo.h"
#include "../subsystem2/motion_control.h" // Buddy 2's API
#include "../subsystem2/subsystem2.h"     // Shared interface for mailboxes

typedef enum {
    STATE_SCAN_IDLE,        // Waiting for Vehicle Controller
    STATE_FORWARD_RANGE,    // Ping straight ahead
    STATE_STOP_AND_PROFILE, // Obstacle found, stop and prepare for scan
    STATE_COARSE_SCAN,      // Sweep servo to find best angle
    STATE_EXECUTE           // Send report and go back to IDLE
} OBSTACLE_STATE;

void obstacle_task(INT stacd, void *exinf)
{
    (void)stacd; (void)exinf; 
    OBSTACLE_STATE state = STATE_SCAN_IDLE;
    UINT8 best_angle = 110; // Default to 110 degrees if no better path is found
    float max_dist = 0.0; // Track the maximum distance found during scanning
    UINT8 angles[] = {15, 110, 178}; // Not fine-tuned as of 02/11/2026

    T_MSG *pmsg;
    OBSTACLE_PROFILE report = {0}; // Initialize report structure
    
    // Track why we were woken up
    UINT current_reason = 0;

    for (;;) {
        switch (state) {
            case STATE_SCAN_IDLE:
                // Sleep until Buddy 2 sends a message to wake us up
                // Use a pointer, not a cast
                if (tk_rcv_mbx(mbx_scan_req, &pmsg, TMO_FEVR) == E_OK) {
                    current_reason = ((SCAN_REQUEST *)pmsg)->reason;
                    state = STATE_FORWARD_RANGE; // Wake up and check path
                }
                break;

            case STATE_FORWARD_RANGE: {
                float dist = ultrasonic_measure();
                
                // If we are too close, we MUST stop regardless of the reason
                // Note: A dead sensor (-1.0) is treated as an obstacle
                if (dist < 0.0 || dist < 15.0) {
                    state = STATE_STOP_AND_PROFILE; 
                } 
                // If it's clear, what do we do next?
                else {
                    if (current_reason == 1) {
                        // Reason 1 = Time. Just go back to sleep, don't sweep the servo.
                        // Send a "Clear" report so the VC doesn't timeout
                        report.best_angle = 110;
                        report.max_dist = 999.0;
                        tk_snd_mbf(mbx_obstacle_rpt, &report, sizeof(report), TMO_POL);
                        state = STATE_SCAN_IDLE;
                    } else {
                        // Reason 2, 3, 4 = Turn/Barcode/Line Lost. 
                        // Path ahead is clear, but still need to sweep the servo to find the best angle.
                        state = STATE_COARSE_SCAN;
                    }
                }
                break;
            }

            case STATE_STOP_AND_PROFILE:
                motion_stop(); // Call subsystem2's motion_stop function to stop the vehicle
                tk_dly_tsk(500); // Give a bit of time to account for momentum before scanning
                state = STATE_COARSE_SCAN; // After vehicle stop, move to next state to do a course scan
                break;

            case STATE_COARSE_SCAN:
                // Scan 3 angles to determine best path
                max_dist = 0.0; // Reset max distance for this scan

                for (int i = 0; i < 3; i++) {
                    servo_set_angle(angles[i]);
                    tk_dly_tsk(500); // Wait for servo to settle

                    float dist = ultrasonic_measure(); 
                    
                    if (dist > max_dist) {
                        max_dist = dist;
                        best_angle = angles[i];
                    }
                }

                // --- FINE SCAN ---
                // Move to the best angle found and take N-samples to get an accurate reading
                servo_set_angle(best_angle);
                tk_dly_tsk(500); // Wait for servo to settle

                float total_dist = 0.0;
                int valid_samples = 0;
                const int N_SAMPLES = 5;

                for (int i = 0; i < N_SAMPLES; i++) {
                    float dist = ultrasonic_measure();
                    if (dist > 0.0) {
                        total_dist += dist;
                        valid_samples++;
                    }
                    tk_dly_tsk(60); // Wait 60ms between pings (sensor minimum cycle)
                }

                // Average the fine scan samples
                if (valid_samples > 0) {
                    max_dist = total_dist / valid_samples;
                } else {
                    max_dist = 0.0; // All samples failed
                }

                // Re-center the servo for the next forward ping
                servo_set_angle(110);
                tk_dly_tsk(500);

                state = STATE_EXECUTE;
                break;

            case STATE_EXECUTE:
                // Package the report for Buddy 2
                report.best_angle = best_angle;
                report.max_dist = max_dist;
                
                // Send it back to the Vehicle Controller
                tk_snd_mbf(mbx_obstacle_rpt, &report, sizeof(report), TMO_POL);

                state = STATE_SCAN_IDLE;
                break;
        }
    }
}