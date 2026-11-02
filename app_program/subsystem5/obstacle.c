#include "obstacle.h"
#include "ultrasonic.h"
#include "servo.h"
#include "../subsystem2/motion_control.h" // Buddy 2's API

typedef enum {
    STATE_SCAN_IDLE,        // Initial state, waiting for command or trigger to start scanning
    STATE_FORWARD_RANGE,    // Ping straight ahead to see if path is clear
    STATE_STOP_AND_PROFILE, // Obstacle found, stop and prepare for course scan
    STATE_COARSE_SCAN,      // Coarse scan
    STATE_EXECUTE
} OBSTACLE_STATE;

void obstacle_task(INT stacd, void *exinf)
{
    (void)stacd; (void)exinf; // Suppress unused parameter warnings
    OBSTACLE_STATE state = STATE_SCAN_IDLE; // Initialise the state machine to IDLE
    UINT8 best_angle = 110; // Default to 110 degrees if no better path is found
    float max_dist = 0.0; // Track the maximum distance found during scanning

    UINT8 angles[] = {15, 110, 178}; // Not fine-tuned as of 02/11/2026

    for (;;) {
        switch (state) {
            case STATE_SCAN_IDLE:
                state = STATE_FORWARD_RANGE; // Force wake for testing (need change later)
                break;

            case STATE_FORWARD_RANGE: {
                float dist = ultrasonic_measure();
                if (dist > 2.0 && dist < 15.0) { // Currently set to between 2cm and 15cm (2cm cuz blindspot)
                    tm_printf((UB*)"[STATE] Obstacle detected at %.1f cm!\n", dist);
                    state = STATE_STOP_AND_PROFILE; // Object detected, hence go to stop and profile state
                } else {
                    tk_dly_tsk(260); // Nothing found, wait before next ping
                }
                break;
            }

            case STATE_STOP_AND_PROFILE:
                tm_printf((UB*)"[STATE] Sending STOP command to Buddy 2...\n");
                motion_stop(); // Call subsystem2's motion_stop function to stop the vehicle
                tk_dly_tsk(500); // Give a bit of time to account for momentum before scanning
                state = STATE_COARSE_SCAN; // After vehicle stop, move to next state to do a course scan
                break;

            case STATE_COARSE_SCAN:
                // Scan 3 angles to determine best path
                tm_printf((UB*)"[STATE] Performing course scan...\n");

                max_dist = 0.0; // Reset max distance for this scan

                for (int i = 0; i < 3; i++) {
                    servo_set_angle(angles[i]);
                    tk_dly_tsk(500); // Wait for servo to settle

                    float dist = ultrasonic_measure(); // Same measure straight, but this time the servo is at an angle
                    tm_printf((UB*)"[SCAN] Angle %d: %.1f cm\n", angles[i], dist);
                    
                    // Check if current direction have more space than the previous best
                    if (dist > max_dist) {
                        max_dist = dist;
                        best_angle = angles[i];
                    }
                }
                state = STATE_EXECUTE;
                break;

            case STATE_EXECUTE:
                tm_printf((UB*)"[STATE] Best angle found: %d degrees\n", best_angle); // Print the best angle found for debugging
                tm_printf((UB*)"[STATE] Scan complete. Returning to IDLE.\n");
                // {TODO: Suggest buddy 2 what is the best direction to turn for them to calculate their next move.}
                // Example: motion_suggest_direction(best_angle, max_dist);
                state = STATE_SCAN_IDLE;
                break;
        }
    }
}