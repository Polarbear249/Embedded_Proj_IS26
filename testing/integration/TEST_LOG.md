# Integration — test log

Tests that two or more subsystems work together. There is one test (or more) for every interface listed in TEAM.md section 5. Both owners agree on the test and sign off the result.

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| INT-01 | *retired 2026-10-30 (template S2 producer / S3 consumer removed on `integrate/s4-onto-main`):* samples flow from S2 to S3 (≥ 5 in 2 s) | old template IF-01, IF-02 | auto | — | — |
| INT-02 | *retired 2026-10-30 (template S3 → S4 monitor removed):* summary lines flow from S3 to S4 (≥ 2 in 3.5 s) | old template IF-03 | auto | — | — |
| INT-10 | S2 distance reaches S4: push the car by hand along a ruler, `d` in the `[IMU] t` line grows by the pushed distance | IF-10 | manual | 1. Flash the normal `make -j8` image, car still for 2 s (IMU level zero). 2. Note `d`. 3. Push the car straight 1000 mm along a tape measure, wheels on the floor. 4. Note `d` again. 5. Push back 1000 mm. Repeat 3 times. | `d` rises by 1000 mm ± 5 % (limit to confirm with Buddy 2) and returns to the start ± 5 %; never jumps backwards while pushing forward. If it reads about 0 or negative, one encoder's sign is reversed |
| INT-09 | Obstacle alert reaches the VC and stops the car: S5 sends an ALERT word, the VC latches `motion_estop()`; after the stop S5 sends a PROFILE word | IF-09 | manual | 1. Normal image, car following the line. 2. Hold a flat object about 10 cm in front of the ultrasonic sensor. 3. Remove it, press GP20. Repeat 5 times | `[VC] obstacle at N cm: STOP (0)` with N < 15, wheels stop, telemetry `motion 6`; then `[VC] obstacle profile L .. C .. R ..`; GP20 releases |
| INT-11 | Hump word reaches S1: the value S4 publishes is the value S1's monitor shows | IF-11 | manual | Normal image; cause a hump event (drive over a ramp) and compare the `[IMU] hump end ... height H mm` line with the next `[TELEM]` block | `terrain hump H mm` and `raw s4 ... max H mm` show the same H |
| INT-12 | Lift state reaches the VC and stops the motors; GP20 cannot release while lifted | IF-12 | manual | Same steps as S4-U38 and S4-U41 (`testing/subsystem4/TEST_LOG.md`) | `[IMU] LIFTED` -> `[VC] car lifted / overturned: STOP (0)` -> `motion 6`; GP20 while held up: `stop kept`; after put down: `stop released` |
| INT-13 | GP20 emergency stop: the VC latches Motion's stop and S3's drive commands are refused until the second press | VC -> S2 (`motion_estop`), S3 -> S2 | manual | Normal image, car following the line: press GP20, wait 3 s, press again | `[VC] GP20: EMERGENCY STOP`, wheels stop, `S3 drive: ... motion=-41` (refused); second press `stop released`, the car drives again |
| INT-14 | IMU task timing with Motion and the encoder task at P5 and S3 driving (risk R1: jitter on the 10 ms tasks) | TEAM.md section 4, NFR (P9 period) | manual | Normal image, line following for at least 30 s; read `per a-b late n` in every `[IMU] t` line (boot line excluded) | record min / max period and the late count (period > 15 ms); expected late 0 |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| YYYY-MM-DD | INT-01 | `abc1234` | | PASS / FAIL | `logs/YYYY-MM-DD_….txt` | |
| 2026-10-31 | INT-10 | `70823b4`+ (uncommitted) | Buddy 4 | PASS | `../subsystem4/logs/2026-10-31_int10_run34.txt` | 1 m push along a tape measure: d 990 mm (-1.0 %). Needs the GPIO edge-interrupt encoder counting (gpio_irq.c, subsystem2/encoder.c); with S2's old polling it read -11 mm (run 29). Buddy 2 to sign off |
| 2026-11-16 | INT-13 | `ff187b8`+ (uncommitted) | Buddy 4 | PASS | `../subsystem4/logs/2026-11-16_lift_run09_normal_image_lift_overturn.txt`, `logs/2026-11-16_normal_image_obstacle_estop_p5_timing.txt` | GP20 during line following: `EMERGENCY STOP`, drive refused (`motion=-41`), second press `stop released`. Seen again in S4 lift runs 12 and 13 |
| 2026-11-16 | INT-09 | `ff187b8`+ (uncommitted) | Buddy 4 | PARTIAL | `logs/2026-11-16_normal_image_obstacle_estop_p5_timing.txt` | `[VC] obstacle at 9 cm: STOP (0)` -> `motion 6`; later `obstacle profile L 260 C 257 R 400 cm, best right`. S5 -> VC -> latch works, but the car was already stationary (line lost) and what stood in front of the sensor was not recorded. Still to do: the defined steps on a moving car, 5 times. Buddy 5 to sign off |
| 2026-11-16 | INT-11 | `ff187b8`+ (uncommitted) | Buddy 4 | PASS (transport) | `logs/2026-11-16_normal_image_obstacle_estop_p5_timing.txt` | `[IMU] hump end ... height 106 mm` -> `terrain hump 106 mm`, `raw s4 ... max 106 mm`. The 106 mm itself is a false hump (left wheel stalled, car pivoted): this row tests the word reaching S1, not the height. Buddy 1 to sign off |
| 2026-11-16 | INT-14 | `ff187b8`+ (uncommitted) | Buddy 4 | MEASURED | `logs/2026-11-16_normal_image_obstacle_estop_p5_timing.txt` | 47 one-second lines, S3 driving, S1 monitor printing, laptop USB: period min 2.6 ms, max 17.5 ms (target 10), **late 1** (one period > 15 ms), task work max 2.2 ms, 0 I2C errors. Typical line 7-13 ms. Before P5 (10-31): 9.0-11.0 ms. The mean rate is held by the cyclic handler; the jitter comes from P5-P8 tasks and console printing preempting P9 |
| 2026-11-17 | INT-12 | `9b437ee` | Buddy 4 | PASS | `../subsystem4/logs/2026-11-17_lift_run12_vz_line_follow_lift.txt`, `../subsystem4/logs/2026-11-17_lift_run13_U41_gp20_while_lifted.txt` | lift while line following -> `LIFTED` -> `[VC] ... STOP (0)` about 70 ms after the lift began (plus one 20 ms VC poll, derived); GP20 6x while held up -> `stop kept`; put down + GP20 -> `stop released`. Details: S4-U38, S4-U41 |
