# Subsystem 4 — test log

**Subsystem:** IMU / hump detection (GY-511 board, chip is an LSM303D at 0x1D, on I2C1: SDA GP2, SCL GP3)  ·  **Student ID:**  2501214  ·  **Code:** `app_program/subsystem4/`

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S4-U01 | *not run since 2026-10-26 (template monitor removed on IMU_Branch):* message buffer max line = S4_SUMMARY_MAX | IF-03 | auto | `subsystem4_test.c` | `[TEST] S4-U01 PASS` |
| S4-U02 | *not run since 2026-10-26 (template LED removed on IMU_Branch):* LED pauses and resumes every 6 reports | IF-05 | manual | 1. Flash normal image. 2. Watch LED and console ~20 s. | console shows "LED task paused"/"resumed"; LED stops/starts |
| S4-U03 | *dropped 2026-10-23:* I2C1 device `iicb` opens (kernel I2C driver no longer used, replaced by S4-U09..U11) | | auto | — | — |
| S4-U04 | *dropped 2026-10-23:* LSM303DLHC magnetometer (0x1E) reads `H43` (board has an LSM303D, see S4-U12) | | auto | — | — |
| S4-U05 | *dropped 2026-10-23:* LSM303DLHC CTRL_REG1_A (0x19) reads back 0x47 (see S4-U13) | | auto | — | — |
| S4-U06 | accelerometer at rest measures gravity: \|a\| = 1000 mg ± 20 % | | auto | `subsystem4_test.c`, board lying still during power-up | `[TEST] S4-U06 PASS`, `[IMU] at rest:` z ≈ ±1000 mg when flat |
| S4-U07 | magnetometer reads a field, no axis saturated (-4096) | | auto | `subsystem4_test.c` | `[TEST] S4-U07 PASS` |
| S4-U09 | I2C1 lines idle high before init (pull-ups present, nothing holding the bus) | | auto | `subsystem4_test.c` | `[TEST] S4-U09 PASS` |
| S4-U10 | *dropped 2026-10-23:* bus scan finds 0x19 and 0x1E (LSM303DLHC addresses; see S4-U14) | | auto | — | — |
| S4-U11 | `drv_i2c1` starts and both sensors ACK their configuration writes | | auto | `subsystem4_test.c` | `[IMU] init: OK`, `[TEST] S4-U11 PASS` |
| S4-U12 | LSM303D answers: WHO_AM_I (0x0F) at 0x1D reads 0x49 | | auto | `subsystem4_test.c` | `[TEST] S4-U12 PASS` |
| S4-U13 | LSM303D configured: CTRL1 (0x20) reads back 0x5F | | auto | `subsystem4_test.c` | `[TEST] S4-U13 PASS` |
| S4-U14 | I2C1 bus scan finds the LSM303D at 0x1D | | auto | `subsystem4_test.c` | `[IMU] I2C1 scan: 0x1d (1 found)`, `[TEST] S4-U14 PASS` |
| S4-U08 | *retired 2026-10-26, replaced by S4-U17..U19:* IMU responds to motion | | manual | 1. Flash `make TEST=1` image. 2. When `[IMU] S4-U08 manual` appears, tilt the board ~90° on each axis, then turn it flat through 360° (5 s window). | the axis pointing down reads ≈ +1000 mg, others ≈ 0; mag X/Y swing through + and − while turning |
| S4-U15 | *retired 2026-10-26, replaced by S4-U17:* IMU mounted on the car: rest vector on the flat floor is 1 g ± 20 % (50 samples, 1 s) | NFR6, FR7 | manual + auto | 1. Flash `make TEST=1` image. 2. When `[IMU] S4-U15 ramp` appears, put the car on the flat floor before the ramp, hands off (5 s). | `[IMU] rest vector` \|a\| 800–1200 mg, 50 of 50 reads ok, `[TEST] S4-U15 PASS`. Record which axis carries gravity (mounting orientation) |
| S4-U16 | *retired 2026-10-26 until the axes are known (S4-U19):* Tilt over a ramp: `tilt` (angle from the rest vector) follows the ramp; 30 s stream with no I2C1 read errors | FR7, A6 | manual + auto | 1. Measure the ramp: rise and run with a ruler, angle = atan(rise/run). 2. When `S4-U16 now push` appears, push the car slowly up, hold on the slope ~2 s, then over and down. | `[TEST] S4-U16 PASS`; while held still on the slope, `tilt` within ±1° of the measured ramp angle (pass limit to confirm after the first run); ≈ 0 again on the flat |
| S4-U17 | Guided axis test, poses held still: FLAT, NOSE UP, FLAT again, LEFT UP each average 50 samples (1 s) with \|a\| = 1 g ± 10 % and max−min ≤ 60 mg on every axis (3 tries per pose) | NFR6 | manual + auto | The car has two rear wheels and a front ball caster (3 contact points). 1. Measure the block height h (flat block, not the curved hump), the distance L from the rear axle to the ball's floor contact, and the wheel track W (centre to centre). 2. Flash the `make TEST=1` image (IMU_Branch), open the terminal. 3. At each `>>> pose n/4` prompt place the car (5 s), then take your hands off when `hands off` appears. NOSE UP = ball on the block, both wheels on the floor; LEFT UP = the car's own left wheel (driver's-seat view) fully on top of the block, right wheel and ball on the floor. | `[TEST] S4-U17 PASS` |
| S4-U18 | Module fixed on its mount: FLAT and FLAT again agree within 30 mg on every axis | NFR6 | auto | runs inside the S4-U17 sequence | `[IMU] flat 1 vs flat 3` all within ±30 mg, `[TEST] S4-U18 PASS` |
| S4-U19 | Axis map found: forward, left and up are each one clear sensor axis (≥ 50 mg change and ≥ 2× the next axis; up ≥ 500 mg) and forward × left = up (right-handed). From 2026-10-26 (after run 10) the left axis is chosen with the forward axis left out, because lifting one wheel of a 3-point car tips it about the diagonal (other wheel to ball), which also changes the forward axis by (W/2)/L of the side change | NFR6 | auto | runs inside the S4-U17 sequence | `[IMU] AXES forward = …  left = …  up = …` with no "(unclear)", `[TEST] S4-U19 PASS`. Record the map. `nose up tilt` ≈ asin(h/L) and `left up tilt` ≈ asin(h/d) with d = L·W / √(L² + (W/2)²) (record both; pass limit to set after the first run) |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| YYYY-MM-DD | S4-U01 | `abc1234` | | PASS / FAIL | `logs/YYYY-MM-DD_….txt` | |
| 2026-10-23 | S4-U01 | `ad3ac7c`+ | | PASS | `logs/2026-10-23_imu_unit_run1.txt` | Run 1 on ad3ac7c + uncommitted IMU code |
| 2026-10-23 | S4-U03 | `ad3ac7c`+ | | PASS | `logs/2026-10-23_imu_unit_run1.txt` | Run 1 on ad3ac7c + uncommitted IMU code |
| 2026-10-23 | S4-U04 | `ad3ac7c`+ | | FAIL | `logs/2026-10-23_imu_unit_run1.txt` | ID read returned E_OBJ (-41). Cause: kernel RP2040 I2C driver sends no STOP after a multi-byte read, bus left held. Fix: drv_imu.c reads one byte at a time |
| 2026-10-23 | S4-U05 | `ad3ac7c`+ | | FAIL | `logs/2026-10-23_imu_unit_run1.txt` | config writes returned E_PAR (-17): TDN_I2C_EXEC rejects rcv_size 0. Fix: drv_imu.c writes with tk_swri_dev(dd, address, ...) |
| 2026-10-23 | S4-U06 | `ad3ac7c`+ | | FAIL | `logs/2026-10-23_imu_unit_run1.txt` | follows from S4-U05 (accelerometer never powered up) |
| 2026-10-23 | S4-U07 | `ad3ac7c`+ | | FAIL | `logs/2026-10-23_imu_unit_run1.txt` | follows from S4-U04/U05 |
| 2026-10-23 | S4-U08 | `ad3ac7c`+ | | FAIL | `logs/2026-10-23_imu_unit_run1.txt` | every read returned an error (bus held, see S4-U04) |
| 2026-10-23 | S4-U08 | `ad3ac7c`+ | | FAIL | `logs/2026-10-23_imu_unit_run2.txt` | run 2: every read E_IO (-57) with the kernel I2C driver; other IDs not captured. Fix: own polled driver `drv_i2c1.c` (build log `logs/2026-10-23_imu_test_build2.txt`) |
| 2026-10-23 | S4-U09 | `ad3ac7c`+ | | PASS | `logs/2026-10-23_imu_unit_run3.txt` | run 3 (own I2C1 driver): bus idle, pull-ups ok |
| 2026-10-23 | S4-U11 | `ad3ac7c`+ | | FAIL | `logs/2026-10-23_imu_unit_run3.txt` | run 3 (own I2C1 driver): NACK_ADDR at 0x19: no LSM303DLHC accelerometer on the bus |
| 2026-10-23 | S4-U10 | `ad3ac7c`+ | | FAIL | `logs/2026-10-23_imu_unit_run3.txt` | run 3 (own I2C1 driver): scan found only 0x1d. Wiring works; module carries a different chip than the LSM303DLHC. Next: identify chip (0x1D ID registers, chip marking) |
| 2026-10-23 | S4-U04 | `ad3ac7c`+ | | FAIL | `logs/2026-10-23_imu_unit_run3.txt` | run 3 (own I2C1 driver): nothing at 0x1E |
| 2026-10-23 | S4-U05 | `ad3ac7c`+ | | FAIL | `logs/2026-10-23_imu_unit_run3.txt` | run 3 (own I2C1 driver): nothing at 0x19 |
| 2026-10-23 | S4-U06 | `ad3ac7c`+ | | FAIL | `logs/2026-10-23_imu_unit_run3.txt` | run 3 (own I2C1 driver): follows from S4-U05 |
| 2026-10-23 | S4-U07 | `ad3ac7c`+ | | FAIL | `logs/2026-10-23_imu_unit_run3.txt` | run 3 (own I2C1 driver): follows from S4-U04 |
| 2026-10-23 | S4-U08 | `ad3ac7c`+ | | FAIL | `logs/2026-10-23_imu_unit_run3.txt` | run 3 (own I2C1 driver): every read NACK_ADDR |
| 2026-10-23 | S4-U09 | `ad3ac7c`+ | | PASS | `logs/2026-10-23_imu_unit_run4.txt` | run 4: bus idle |
| 2026-10-23 | S4-U11 | `ad3ac7c`+ | | FAIL | `logs/2026-10-23_imu_unit_run4.txt` | run 4: NACK at 0x19 (LSM303DLHC address) |
| 2026-10-23 | S4-U10 | `ad3ac7c`+ | | FAIL | `logs/2026-10-23_imu_unit_run4.txt` | run 4: only 0x1d. ID register 0x0F = 0x49: chip is an LSM303D, not LSM303DLHC. Fix: drv_imu.c ported to LSM303D (build log `logs/2026-10-23_imu_test_build3.txt`); S4-U04/U05/U10 dropped, replaced by S4-U12..U14 |
| 2026-10-23 | S4-U04 | `ad3ac7c`+ | | FAIL | `logs/2026-10-23_imu_unit_run4.txt` | run 4: nothing at 0x1E |
| 2026-10-23 | S4-U05 | `ad3ac7c`+ | | FAIL | `logs/2026-10-23_imu_unit_run4.txt` | run 4: nothing at 0x19 |
| 2026-10-23 | S4-U06 | `ad3ac7c`+ | | FAIL | `logs/2026-10-23_imu_unit_run4.txt` | run 4: follows from wrong chip |
| 2026-10-23 | S4-U07 | `ad3ac7c`+ | | FAIL | `logs/2026-10-23_imu_unit_run4.txt` | run 4: follows from wrong chip |
| 2026-10-23 | S4-U08 | `ad3ac7c`+ | | FAIL | `logs/2026-10-23_imu_unit_run4.txt` | run 4: every read NACK_ADDR |
| 2026-10-23 | S4-U09 | `8b86919` | | PASS | `logs/2026-10-23_imu_unit_run5.txt` | run 5 (LSM303D driver) |
| 2026-10-23 | S4-U11 | `8b86919` | | PASS | `logs/2026-10-23_imu_unit_run5.txt` | run 5 |
| 2026-10-23 | S4-U14 | `8b86919` | | PASS | `logs/2026-10-23_imu_unit_run5.txt` | run 5: scan 0x1d |
| 2026-10-23 | S4-U12 | `8b86919` | | PASS | `logs/2026-10-23_imu_unit_run5.txt` | run 5: WHO_AM_I 0x49 |
| 2026-10-23 | S4-U13 | `8b86919` | | PASS | `logs/2026-10-23_imu_unit_run5.txt` | run 5: CTRL1 0x5f |
| 2026-10-23 | S4-U06 | `8b86919` | | PASS | `logs/2026-10-23_imu_unit_run5.txt` | run 5: x=-966 y=8 z=212 mg, \|a\| ≈ 989 mg. Gravity on -X: module was on its side / tilted ~12°, not flat |
| 2026-10-23 | S4-U07 | `8b86919` | | PASS | `logs/2026-10-23_imu_unit_run5.txt` | run 5: mag ≈ 1.02 G total, higher than Earth's field (0.25-0.65 G): needs hard-iron calibration before heading use |
| 2026-10-23 | S4-U01 | `8b86919` | | PASS | `logs/2026-10-23_imu_unit_run5.txt` | run 5 |
| 2026-10-23 | S4-U08 | `8b86919` | | NOT RUN | `logs/2026-10-23_imu_unit_run5.txt` | run 5: readings constant for all 25 samples, board was not moved. Repeat with tilting/turning |
| 2026-10-23 | S4-U09 | `8b86919` | | PASS | `logs/2026-10-23_imu_unit_run6.txt` | run 6 |
| 2026-10-23 | S4-U11 | `8b86919` | | PASS | `logs/2026-10-23_imu_unit_run6.txt` | run 6 |
| 2026-10-23 | S4-U14 | `8b86919` | | PASS | `logs/2026-10-23_imu_unit_run6.txt` | run 6 |
| 2026-10-23 | S4-U12 | `8b86919` | | PASS | `logs/2026-10-23_imu_unit_run6.txt` | run 6 |
| 2026-10-23 | S4-U13 | `8b86919` | | PASS | `logs/2026-10-23_imu_unit_run6.txt` | run 6 |
| 2026-10-23 | S4-U06 | `8b86919` | | PASS | `logs/2026-10-23_imu_unit_run6.txt` | run 6: board flat, x=-42 y=-361 z=884 mg, \|a\| ≈ 956 mg |
| 2026-10-23 | S4-U07 | `8b86919` | | PASS | `logs/2026-10-23_imu_unit_run6.txt` | run 6 |
| 2026-10-23 | S4-U01 | `8b86919` | | PASS | `logs/2026-10-23_imu_unit_run6.txt` | run 6 |
| 2026-10-23 | S4-U08 | `8b86919` | | FAIL | `logs/2026-10-23_imu_unit_run6.txt` | run 6: readings follow the motion (acc Y -545..+113 mg, mag Y -2431..+70), but tilts stayed under ~35°: no axis taken to ±1000 mg, mag X stayed negative (hard-iron offset, see S4-U07 note). Repeat with 90° tilts and a full flat turn |
| 2026-10-23 | S4-U09 | `8b86919` | | PASS | `logs/2026-10-23_imu_unit_run7.txt` | run 7 |
| 2026-10-23 | S4-U11 | `8b86919` | | PASS | `logs/2026-10-23_imu_unit_run7.txt` | run 7 |
| 2026-10-23 | S4-U14 | `8b86919` | | PASS | `logs/2026-10-23_imu_unit_run7.txt` | run 7 |
| 2026-10-23 | S4-U12 | `8b86919` | | PASS | `logs/2026-10-23_imu_unit_run7.txt` | run 7 |
| 2026-10-23 | S4-U13 | `8b86919` | | PASS | `logs/2026-10-23_imu_unit_run7.txt` | run 7 |
| 2026-10-23 | S4-U06 | `8b86919` | | PASS | `logs/2026-10-23_imu_unit_run7.txt` | run 7: board on its side, x=-979 y=0 z=203 mg, \|a\| ≈ 1000 mg |
| 2026-10-23 | S4-U07 | `8b86919` | | PASS | `logs/2026-10-23_imu_unit_run7.txt` | run 7 |
| 2026-10-23 | S4-U01 | `8b86919` | | PASS | `logs/2026-10-23_imu_unit_run7.txt` | run 7 |
| 2026-10-23 | S4-U08 | `8b86919` | | PASS | `logs/2026-10-23_imu_unit_run7.txt` | run 7: every axis reached about 1 g: X -979 (still), Y -984 and +920, Z +1074 and -668. Mag X (+3090..-5364) and Y (-13151..+2069) both changed sign while turning. Samples were taken while moving, so they include hand acceleration (\|a\| up to ~1.5 g) and the other axes were not ≈ 0 |
| 2026-10-26 | S4-U09 | `b59546c`+ | | PASS | `logs/2026-10-26_imu_unit_run8.txt` | run 8: IMU mounted on the car; build log `logs/2026-10-26_imu_test_build4.txt` |
| 2026-10-26 | S4-U11 | `b59546c`+ | | PASS | `logs/2026-10-26_imu_unit_run8.txt` | run 8 |
| 2026-10-26 | S4-U14 | `b59546c`+ | | PASS | `logs/2026-10-26_imu_unit_run8.txt` | run 8 |
| 2026-10-26 | S4-U12 | `b59546c`+ | | PASS | `logs/2026-10-26_imu_unit_run8.txt` | run 8 |
| 2026-10-26 | S4-U13 | `b59546c`+ | | PASS | `logs/2026-10-26_imu_unit_run8.txt` | run 8 |
| 2026-10-26 | S4-U06 | `b59546c`+ | | PASS | `logs/2026-10-26_imu_unit_run8.txt` | run 8: on the car, x=200 y=18 z=1009 mg, \|a\| ≈ 1029 mg. Z is up; X ≈ +200 mg on flat ground means the module is mounted ~10-11° nose-tilted about Y (NFR6: level it or calibrate the offset) |
| 2026-10-26 | S4-U07 | `b59546c`+ | | PASS | `logs/2026-10-26_imu_unit_run8.txt` | run 8 |
| 2026-10-26 | S4-U08 | `b59546c`+ | | NOT RUN | `logs/2026-10-26_imu_unit_run8.txt` | run 8: car not moved during the 5 s window (readings constant, noise about ±10 mg); motion covered by S4-U16 |
| 2026-10-26 | S4-U01 | `b59546c`+ | | PASS | `logs/2026-10-26_imu_unit_run8.txt` | run 8 |
| 2026-10-26 | S4-U15 | `b59546c`+ | | PASS | `logs/2026-10-26_imu_unit_run8.txt` | run 8: rest vector x=126 y=30 z=943, \|a\| = 951 mg, 50/50 reads. Passed the ±20 % limit, but flat stretches later read \|a\| ≈ 1020 mg (x ≈ 170, z ≈ 1005), so the car was probably still being handled: flat ground then shows 1-4° instead of 0°. Limit too loose: tighten and add a stillness check |
| 2026-10-26 | S4-U16 | `b59546c`+ | | PASS | `logs/2026-10-26_imu_unit_run8.txt` | run 8: 300 samples, 0 read errors. Derived from x/z, not measured on the car: flat pitch ≈ +10° (mount offset), slope A ≈ 0° (t 14.4-19.3 s), slope B ≈ +19° (t 21.7-26.6 s), so ramp ≈ 9-11° each side; Y stays ≈ 0, so X is the travel axis. Ruler angle of the ramp not yet recorded. "max tilt 91.2°" is hand-push transients up to ~2 g, not slope. Loop period measured 110 ms for a 100 ms delay (tk_dly_tsk drift) |
| 2026-10-26 | S4-U09, U11, U14, U12, U13, U07, U01 | `b59546c`+ | | PASS | `logs/2026-10-26_imu_unit_run9.txt` | run 9: same image as run 8 |
| 2026-10-26 | S4-U06 | `b59546c`+ | | PASS | `logs/2026-10-26_imu_unit_run9.txt` | run 9: x=139 y=-55 z=1009 mg. Run 8 was x=200 y=18 on the same bench: module shifted ~3-4° between runs unless it was remounted (NFR6: rigid mount) |
| 2026-10-26 | S4-U08 | `b59546c`+ | | NOT RUN | `logs/2026-10-26_imu_unit_run9.txt` | run 9: car not moved in the 5 s window |
| 2026-10-26 | S4-U15 | `b59546c`+ | | PASS | `logs/2026-10-26_imu_unit_run9.txt` | run 9: rest vector x=137 y=-59 z=1012, \|a\| = 1022 mg, car still (fixes run 8 issue) |
| 2026-10-26 | S4-U16 | `b59546c`+ | | PASS | `logs/2026-10-26_imu_unit_run9.txt` | run 9: 0 read errors. Steady poses: t 7.8-11.3 s pure X change (x ≈ -235, -370 mg from rest, tilt ≈ 21°); later poses changed Y too (car rolled/turned), so axes not yet identified. Tester unsure of poses: repeat as a scripted book test (nose up, left side up) |
| 2026-10-26 | S4-U09, U11, U14, U12, U13, U06, U07 | `5867faf` | | PASS | `logs/2026-10-26_imu_unit_run10.txt` | run 10 (IMU_Branch, subsystem 4 only): bench 7/7. At rest x=11 y=-13 z=1028 mg: module now nearly level (run 9 had x=137, so it was remounted) |
| 2026-10-26 | S4-U17 | `5867faf` | | PASS | `logs/2026-10-26_imu_unit_run10.txt` | run 10: all 4 poses still on the first try, spread 19-47 mg, \|a\| 1022-1026 mg |
| 2026-10-26 | S4-U18 | `5867faf` | | PASS | `logs/2026-10-26_imu_unit_run10.txt` | run 10: flat 1 vs flat 3 dx=0 dy=0 dz=1 mg: module fixed on its mount |
| 2026-10-26 | S4-U19 | `5867faf` | | FAIL | `logs/2026-10-26_imu_unit_run10.txt` | run 10: forward = +X (nose up dx=+291 mg, clean) and up = +Z (flat z=1025 mg) are clear. Left came out as -Y, which with +X forward and +Z up is a left-handed frame; the LSM303D frame is right-handed, so pose 4 almost certainly lifted the RIGHT side and left = +Y (derived, not yet shown by a passing run). Pose 4 was also not square (dx=-59 mg, ≈3° pitch). Tilts: nose up 16.6°, side 8.9°, mount 0.7°. h, axle distance and track width not recorded. The printed hint compared the side tilt with the axle distance; it should use the track width (fixed after this run). Repeat with the left wheels on the book |
| 2026-10-26 | S4-U19 | `5867faf` | | NOTE | `logs/2026-10-26_imu_unit_run10.txt` | Correction to the run 10 row above, after the tester described the car: 2 rear wheels + front ball caster, block = the 23 mm hump, wheel track W ≈ 90 mm, axle-to-ball L not yet measured. (1) The dx=-59 mg in pose 4 is expected for a 3-point car (it tips about the wheel-to-ball diagonal), not a crooked pose. (2) Derived, not measured: nose up 16.6° fits L ≈ 80 mm (asin(23/80) = 16.7°); with L ≈ 80 mm the side pose should read ≈ 17°, but 8.9° was measured, so the wheel was probably partly down the curved hump (about half the lift). (3) For L < W the old S4-U19 rule can mark the left axis unclear even on a correct pose (PC simulation of all 24 mountings: old rule fails all at L = 60 and 80 mm, new rule passes all). Test changed: flat block, ball on the block for NOSE UP, left axis chosen without the forward axis |

<!-- Runs 5-7 (`8b86919`): the flashed image also contained subsystem1/task_led.c and subsystem2/task_producer.c restored from e4ec31d, because the repo does not link without them. IMU tests S4-U03..U08: build log in logs/2026-10-23_imu_test_build.txt. After each run, add a row per test after flashing build_make/s4_imu_test_smp1_usb_cdc_wifi_test.uf2 and saving the console to logs/YYYY-MM-DD_imu_unit.txt -->
