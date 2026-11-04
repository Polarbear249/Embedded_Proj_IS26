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

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| YYYY-MM-DD | INT-01 | `abc1234` | | PASS / FAIL | `logs/YYYY-MM-DD_….txt` | |
| 2026-10-31 | INT-10 | `70823b4`+ (uncommitted) | Buddy 4 | PASS | `../subsystem4/logs/2026-10-31_int10_run34.txt` | 1 m push along a tape measure: d 990 mm (-1.0 %). Needs the GPIO edge-interrupt encoder counting (gpio_irq.c, subsystem2/encoder.c); with S2's old polling it read -11 mm (run 29). Buddy 2 to sign off |
