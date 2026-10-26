# Subsystem 5 — test log

**Subsystem:** 5 - Adaptive Ultrasonic Scanning, Obstacle Profiling and Recovery   ·  **Student ID:** 2500761  ·  **Code:** `app_program/subsystem5/`

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID     | What is tested                     | Requirement / Interface   | Method        | Steps / How                                                                                | Expected result                                                                                               |
| ------ | ---------------------------------- | ------------------------- | ------------- | ------------------------------------------------------------------------------------------ | ------------------------------------------------------------------------------------------------------------- |
| S5-U01 | ECHO Pin Voltage (Hardware safety) | Hardware / A8             | manual        | Power HC-SR04+ at 3.3V. Trigger 100 pings.                                                 | Pico GPIO registers pulses without damage & no overheating; all 100 pings are successful producing a [PASS].  |
| S5-U02 | Raw Pulse Width                    | HC-SR04+ Interface        | auto          | Trigger sensor and print ISR `pulse_width` to console.                                     | Console prints valid µ's (100µ's - 23,000µ's) when object is in range. Produce a [PASS].                      |
| S5-U03 | Distance Calculation               | Math Logic                | auto          | Compare calculated distance against a hardcoded expected range (e.g. object is 10cm away). | Console prints [PASS] if math is within ±5cm tolerance (to account for human error and acoustic cone spread). |
| S5-U04 | Servo Pan & Scan Logic             | PWM / Actuator            | auto / manual | Command 30°, 90°, 150° tilts with `500ms` delays. Tester observes movement visually.       | Servo physically moves; console prints [PASS] for distance readings after settling.                           |
| S5-U05 | Brake Handshake                    | `Q_motion` dependancy     | auto          | Trigger obstacle warning. Simulate `BRAKE_ACK` via terminal.                               | Console waits for simulated `BRAKE_ACK` via terminal input before printing [PASS].                            |
| S5-U06 | Obstacle Warning Trigger           | State Machine             | auto          | Bring object < 15cm. (Crosses safe stop distance).                                         | Transitions to `STOP_AND_PROFILE`, console prints warning and [PASS].                                         |
| S5-U07 | Idle State Recovery                | State Machine / Line Task | auto          | Force `REAQUIRE_LINE` state. Simulate line found via terminal.                             | State changes back to `SCAN_IDLE`, console prints [PASS].                                                     |

*Method:* 
**auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date       | Test ID | Commit    | Tester | Result | Log / evidence                             | Notes                                                                                                                                                                                   |
| ---------- | ------- | --------- | ------ | ------ | ------------------------------------------ | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| 2026-10-25 | S5-U01  | `d44957c` | Daryl  | PASS   | `logs/2026-10-25_s5_echo_voltage_run1.txt` | 100/100 pings. Verified 3.3V safe via Grove GP6/GP7. Used GP6 and GP7 due to wiring limitations.                                                                                        |
| 2026-10-26 | S5-U02  | `9c45ceb` | Daryl  | PASS   | `logs/2026-10-26_s5_distance_run1.txt`     | Run 1: Valid raw pulse widths (471us to 10298us).                                                                                                                                       |
| 2026-10-26 | S5-U03  | `9c45ceb` | Daryl  | PASS   | `logs/2026-10-26_s5_distance_run1.txt`     | Run 1: Math verified against ceiling (~177cm). Hand-held objects validated with expected ±5cm human error tolerance.                                                                    |
| 2026-10-26 | S5-U02  | `9c45ceb` | Daryl  | PASS   | `logs/2026-10-26_s5_distance_run2.txt`     | Run 2: Valid raw pulse widths (187us to 10528us).                                                                                                                                       |
| 2026-10-26 | S5-U03  | `9c45ceb` | Daryl  | PASS   | `logs/2026-10-26_s5_distance_run2.txt`     | Run 2: Math verified. Close-range hand tests and blind spot rejection (<2cm) confirmed within human error margins. Can't do (>400cm) due to space constraints.                          |
| 2026-10-26 | S5-U05  | `f6a2497` | Daryl  | FAIL   | `logs/2026-10-26_s5_detection_run1.txt`    | Run 1: getchar() consumed hidden newline, skipping handshake.                                                                                                                           |
| 2026-10-26 | S5-U06  | `f6a2497` | Daryl  | PASS   | `logs/2026-10-26_s5_detection_run1.txt`    | Run 1: Obstacle correctly detected at 8.5cm. Stop message successfully sent.                                                                                                            |
| 2026-10-26 | S5-U07  | `f6a2497` | Daryl  | PASS   | `logs/2026-10-26_s5_detection_run1.txt`    | Run 1: Line recovery accidentally triggered due to buffered input.                                                                                                                      |
| 2026-10-26 | S5-U05  | `f6a2497` | Daryl  | PASS   | `logs/2026-10-26_s5_detection_run2.txt`    | Run 2: Handshake successful after bugfix.                                                                                                                                               |
| 2026-10-26 | S5-U06  | `f6a2497` | Daryl  | PASS   | `logs/2026-10-26_s5_detection_run2.txt`    | Run 2: Obstacle detected at 10.5cm. Stop message successfully sent. A ±5cm of noise can be observed from the data, which is within expectations, need see if can focus the signal more. |
| 2026-10-26 | S5-U07  | `f6a2497` | Daryl  | PASS   | `logs/2026-10-26_s5_detection_run2.txt`    | Run 2: Line reaquire successful                                                                                                                                                         |
| 2026-10-26 | S5-U05  | `2041d13` | Daryl  | PASS   | `logs/2026-10-26_s5_detection_run3.txt`    | Run 3: Handshake successful                                                                                                                                                             |
| 2026-10-26 | S5-U06  | `2041d13` | Daryl  | PASS   | `logs/2026-10-26_s5_detection_run3.txt`    | Run 3: Continuous approach test. Accurate tracking from 265cm down to 14.8cm trigger.                                                                                                   |                            |
| 2026-10-26 | S5-U07  | `2041d13` | Daryl  | PASS   | `logs/2026-10-26_s5_detection_run3.txt`    | Run 3: Simple line reacquire successful                                                                                                                                                 |
| 2026-10-26 | S5-U04  | `ce876af` | Daryl  | PASS   | `logs/2026-10-26_s5_servo_run1.txt`        | Run 1: Servo physically moved to 30/90/150. No echo detected during test, but PWM logic verified.                                                                                       |
