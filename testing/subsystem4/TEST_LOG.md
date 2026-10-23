# Subsystem 4 — test log

**Subsystem:** IMU / hump detection (GY-511 LSM303DLHC on I2C1, SDA GP2, SCL GP3)  ·  **Student ID:**  2501214  ·  **Code:** `app_program/subsystem4/`

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S4-U01 | message buffer max line = S4_SUMMARY_MAX | IF-03 | auto | `subsystem4_test.c` | `[TEST] S4-U01 PASS` |
| S4-U02 | LED pauses and resumes every 6 reports | IF-05 | manual | 1. Flash normal image. 2. Watch LED and console ~20 s. | console shows "LED task paused"/"resumed"; LED stops/starts |
| S4-U03 | *dropped 2026-10-23:* I2C1 device `iicb` opens (kernel I2C driver no longer used, replaced by S4-U09..U11) | | auto | — | — |
| S4-U04 | IMU wired and answering: magnetometer (0x1E) ID registers read `H43` | | auto | `subsystem4_test.c` | `[TEST] S4-U04 PASS` |
| S4-U05 | accelerometer (0x19) accepts config: CTRL_REG1_A reads back 0x47 | | auto | `subsystem4_test.c` | `[TEST] S4-U05 PASS` |
| S4-U06 | accelerometer at rest measures gravity: \|a\| = 1000 mg ± 20 % | | auto | `subsystem4_test.c`, board lying still during power-up | `[TEST] S4-U06 PASS`, `[IMU] at rest:` z ≈ ±1000 mg when flat |
| S4-U07 | magnetometer reads a field, no axis saturated (-4096) | | auto | `subsystem4_test.c` | `[TEST] S4-U07 PASS` |
| S4-U09 | I2C1 lines idle high before init (pull-ups present, nothing holding the bus) | | auto | `subsystem4_test.c` | `[TEST] S4-U09 PASS` |
| S4-U10 | I2C1 bus scan finds the accelerometer 0x19 and the magnetometer 0x1E | | auto | `subsystem4_test.c` | `[IMU] I2C1 scan: 0x19 0x1e (2 found)`, `[TEST] S4-U10 PASS` |
| S4-U11 | `drv_i2c1` starts and both sensors ACK their configuration writes | | auto | `subsystem4_test.c` | `[IMU] init: OK`, `[TEST] S4-U11 PASS` |
| S4-U08 | IMU responds to motion | | manual | 1. Flash `make TEST=1` image. 2. When `[IMU] S4-U08 manual` appears, tilt the board ~90° on each axis, then turn it flat through 360° (5 s window). | the axis pointing down reads ≈ +1000 mg, others ≈ 0; mag X/Y swing through + and − while turning |

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

<!-- IMU tests S4-U03..U08: build log in logs/2026-10-23_imu_test_build.txt. After each run, add a row per test after flashing build_make/s4_imu_test_smp1_usb_cdc_wifi_test.uf2 and saving the console to logs/YYYY-MM-DD_imu_unit.txt -->
