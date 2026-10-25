# Subsystem 5 — test log

**Subsystem:** 5 - Adaptive Ultrasonic Scanning, Obstacle Profiling and Recovery   ·  **Student ID:** 2500761  ·  **Code:** `app_program/subsystem5/`

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID     | What is tested                      | Requirement / Interface   | Method        | Steps / How                                                                                | Expected result                                                                     |
| ------ | ----------------------------------- | ------------------------- | ------------- | ------------------------------------------------------------------------------------------ | ----------------------------------------------------------------------------------- |
| S5-U01 | ECHO Pin Voltage (Hardware safety)  | Hardware / A8             | manual        | Power HC-SR04+ at 3.3V. Trigger 100 pings.                                                 | Pico GPIO registers pulses without damage & no overheating.                         |
| S5-U02 | Raw Pulse Width (Hardware function) | HC-SR04+ Interface        | auto          | Trigger sensor and print ISR `pulse_width` to console.                                     | Console prints valid µ's (100µ's - 23,000µ's) when object is in range.              |
| S5-U03 | Distance Calculation                | Math Logic                | auto          | Compare calculated distance against a hardcoded expected range (e.g. object is 10cm away). | Console prints `[TEST] S5-U03 PASS` if math is within ±3mm tolerance.               |
| S5-U04 | Servo Pan & Scan Logic              | PWM / Actuator            | auto / manual | Command 30°, 90°, 150° tilts with `500ms` delays. Tester observes movement visually.       | Servo physically moves; console prints `PASS` for distance readings after settling. |
| S5-U05 | Brake Handshake                     | `Q_motion` dependancy     | auto          | Trigger obstacle warning. Simulate `BRAKE_ACK` via terminal.                               | Servo does not sweep until `BRAKE_ACK` is received, and console prints `PASS`.      |
| S5-U06 | Obstacle Warning Trigger            | State Machine             | auto          | Bring object < 15cm. (Crosses safe stop distance).                                         | State changes to `STOP_AND_PROFILE`, console prints warning and `PASS`.             |
| S5-U07 | Idle State Recovery                 | State Machine / Line Task | auto          | Force `REAQUIRE_LINE` state. Simulate line found via terminal.                             | State changes back to `SCAN_IDLE`, console prints `PASS`.                           |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date       | Test ID | Commit    | Tester | Result      | Log / evidence          | Notes |
| ---------- | ------- | --------- | ------ | ----------- | ----------------------- | ----- |
| 2026-10-25 | S5-U01  | `d44957c` | Daryl  | PASS | `2026-10-25_s5_echo_voltage_run1.txt` | 100/100 pings. Verified 3.3V safe via Grove GP6/GP7. Used GP6 and GP7 due to wiring limitations. |
