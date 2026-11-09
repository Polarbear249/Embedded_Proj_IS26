# Subsystem 3 — test log

**Subsystem:** IR line following and barcode decoding  ·  **Student ID:** <student ID>  ·  **Code:** `app_program/subsystem3/`

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

### PC unit tests (`pc/run_tests.sh`, no hardware)

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S3-P01 | Code 39 table: 44 distinct patterns, 3 wide each, 2 bars + 1 space or 3 spaces | Barcode decoding | auto | `pc/run_tests.sh` | `[PC] S3-P01 PASS` ×4 |
| S3-P02 | A–D patterns match the Code 39 module strings | Barcode decoding | auto | as above | `S3-P02 PASS` ×4 |
| S3-P03 | `docs/Barcode Sample.pdf` bar widths decode to A and Z, both directions | Barcode decoding | auto | as above | `S3-P03 PASS` ×4 |
| S3-P04 | 25 800 random reads: every character, wide:narrow 3.0/2.5/2.2, ±15 % jitter, ±20 % speed drift, both directions | Robust at varying speeds | auto | as above | all correct |
| S3-P05 | Bad input rejected: equal widths, wrong length, NULL, `***`, corrupt char | Barcode decoding | auto | as above | `S3-P05 PASS` ×5 |
| S3-P06 | Edge capture: A–D at 50…1000 mm/s, both directions, across the 31-bit timer wrap | Barcode detection | auto | as above | `S3-P06 PASS` |
| S3-P07 | Same with a 40 µs comparator spike inside every element | Barcode detection | auto | as above | `S3-P07 PASS` |
| S3-P08 | Two barcodes in a row both decode | Barcode detection | auto | as above | `S3-P08 PASS` |
| S3-P09 | Barcode sensor crossing the 18 mm line is ignored | Barcode detection | auto | as above | `S3-P09 PASS` |
| S3-P10 | Half a barcode is reported once as a failed read | Barcode detection | auto | as above | `S3-P10 PASS` |
| S3-P11 | Power-up on a black bar waits for quiet, then decodes | Barcode detection | auto | as above | `S3-P11 PASS` |
| S3-P12…P15 | Line logic: start, debounce, steer left/right, weave bias, junction, lost → search near → search far → FAILED → recovered | Line following, junction, line-lost recovery | auto | as above | all PASS |
| S3-P16…P18 | IF-06 word: round trip, masking, signed error, staleness after 3 reads | IF-06, Change Requirement 1, NFR18 | auto | as above | all PASS |
| S3-P31 | Widths logged on the car (thin bars, wide spaces): a forward `*E*` whose first wide space is 5.8× the first bar still decodes | Barcode detection | auto | as above | `S3-P31 PASS` |

### On-target tests (`make TEST=1`)

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S3-U01 | RP2040 TIMERAWL runs at 1 MHz under µT-Kernel | Barcode timing | auto | `subsystem3_test.c` | 200 ms = 195 000…215 000 µs |
| S3-U02 | `line_state` republished every 10 ms | IF-06, NFR18 | auto | as above | `seq` +18…22 in 200 ms |
| S3-U03 | Line task running | — | auto | as above | mode ≠ INIT |
| S3-U04 | Cycle time < 1 ms, no missed 10 ms slot | NFR11 | auto | as above | `S3-U04 PASS` |
| S3-U05 | Barcode edge ring never overflowed | Barcode detection | auto | as above | `S3-U05 PASS` |
| S3-U06 | Each sensor reads 1 on black, 0 on white | IR sensor integration | manual | set `S3_BENCH_TESTS` to 1, `make TEST=1`, move black tape under L, R, BC for 15 s | printed L/R/BC follow the tape |
| S3-U07 | A printed barcode decodes on the bench | Barcode decoding | manual | same build; pass a printed `*A*`…`*D*` under BC within 30 s | `barcode 'A' cmd=0` etc., `S3-U07 PASS` |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| 2026-11-01 | S3-P01…S3-P18 | (this commit) | Claude Code | PASS (49/49) | `logs/2026-11-01_s3_pc_tests.txt` | MinGW-W64 gcc 15.2.0. Module sources built with `-Wall -Wextra -pedantic -Wconversion -Wshadow -Werror`. S3-P07 was confirmed to FAIL with the glitch filter disabled |
| 2026-11-01 | build | (this commit) | Claude Code | compiles, links | — | All `subsystem3/*.c` cross-compiled with the firmware flags plus `-Wall -Wextra -Wconversion -Wshadow`, `APP_TEST=0` and `1`: 0 warnings. Full image linked with `make CONSOLE=uart WIFI=none` (± `TEST=1`) in a scratch copy, with S1's WiFi status call stubbed out because that build has no WiFi. The default build was NOT run: it needs Pico SDK 2.x, and this PC has 1.5.1 |
| 2026-11-01 | build | (this commit) | Claude Code | compiles | — | Added `S3_BARCODE_ENABLE` (now 0: barcode sensor has hardware issues). All `subsystem3/*.c` cross-compiled with barcode 0 and 1, `APP_TEST` 0 and 1: 0 warnings. PC tests 49/49 still pass |
| — | S3-U01…S3-U07 | | | NOT RUN | | Not yet run on the car |
| 2026-11-09 | build | 2ed56b2 + uncommitted S3 work | Claude Code | compiles, links | — | Normal image (`TEST=0`) built in WSL Ubuntu 24.04 with Pico SDK 2.2.0, `make -j8`: 0 errors, 0 warnings. `S3_BARCODE_ENABLE 1`, `S3_NAV_SWITCH 1`, `S3_DRIVE_USE_SPEED 1` (motion_line_drive) |
| 2026-11-09 | on-car drive (no test ID) | 2ed56b2 + uncommitted S3 work | Buddy 3 | FAIL (did not move) | `logs/2026-11-09_s3_drive_barcode_run.txt` | S3 armed and commanded wheels, but `motion=-41` and telemetry `motion 6 (estop/fault)`, no fault bits: the VC e-stop latch was set (GP20 press or S5 obstacle < 15 cm). Released by pressing GP20. Not an S3 fault |
| 2026-11-09 | on-car drive + barcode (no test ID) | 2ed56b2 + uncommitted S3 work | Buddy 3 | FAIL (barcode not decoded) | `logs/2026-11-09_s3_drive_barcode_run.txt` | USB power. Followed the line about 1.9 m at 6–7 cm/s, lost it once at ~17 s and recovered within 1 s. The car crossed a full barcode, but `decode ok 0 fail 1`: only 13 of the 29 elements, two clipped at the 524 ms width limit, run started during line-lost search pivoting. GP7 still chatters (~1000 edges). Junction (`C=1 R=1`) held for over 30 cm at the end, cause unknown. S4 IMU never finished calibrating while moving. Next: hand test over the barcode with motors e-stopped, then tune the GP7 potentiometer and height |
| 2026-11-09 | on-car barcode tuning (no test ID) | 2ed56b2 + uncommitted S3 work | Buddy 3 | progress | serial logs (not saved) | GP7 tuning: potentiometer alone either missed narrow bars or chattered. A black-tape slit (~1.5–2 mm, parallel to the bars) over the TCRT5000 cut chatter and made narrow bars and spaces visible. Barcode must sit centred under the sensor and on a flat section (reads broke on the hump). Several reads reached 27–29 elements |
| 2026-11-09 | on-car barcode `*E*`, driven ~6 cm/s, forward (no test ID) | 2ed56b2 + uncommitted S3 work | Buddy 3 | 1 of 4 PASS | serial logs (not saved) | First decode on the car: `S3 barcode: 'E' stored (ok 1, failed 1)`. All 3 failures had 27 elements, all correct except the first two of `*` missing: the first wide space (≈175 ms) exceeded 5× the thin first bar (≈30 ms), so the run restarted at the second bar. Fixed in `barcode_capture.c` (next row) |
| 2026-11-09 | S3-P01…S3-P31 | 2ed56b2 + uncommitted S3 work | Claude Code | PASS (124/124) | — | `bc_run_limit_us()`: the 5× run limit now applies from the 3rd element (`BC_CAP_LIMIT_FROM`), not the 2nd. New S3-P31 uses the car's logged widths; it FAILED before the fix and passes after. All earlier capture tests (P06–P11) still pass. Module built with `-Wall -Wextra -pedantic -Wconversion -Wshadow -Werror` |
| 2026-11-09 | build | 2ed56b2 + uncommitted S3 work | Claude Code | compiles, links | — | Normal image with the fix, WSL + Pico SDK 2.2.0, `make -j8`: 0 errors, 0 warnings. Serial shows `fw 20:40:xx`. NOT yet run on the car |
| 2026-11-09 | on-car barcode after the fix, driven ~6 cm/s, both directions (no test ID) | 2ed56b2 + uncommitted S3 work (`fw 20:40:47`) | Buddy 3 | 7 of 8 logged passes decoded | `logs/2026-11-09_s3_barcode_after_fix.txt` | Decoded `E` ×3 (2 normal, 1 flipped = read backwards), `B` ×1, `C` ×3; `S3 junction: 'C' = GO STRAIGHT` via the bench map. 1 failure: the pass right after the GP7 sensor was moved sideways; the read split at >1 s of black mid-barcode (both line sensors black too). Passed again once the sensor was moved back. End counters `ok 8, failed 8` on one power-up, but they include 2 failed reads from handling the car while stopped and 1 decode + 4 failures not in the logs, so this is not a clean NFR2 figure |
| 2026-11-09 | evidence file | — | Claude Code | — | `logs/2026-11-09_s3_barcode_tuning.txt` | Verbatim `S3 barcode:` result lines of the tuning runs before the fix (the two rows above marked "serial logs (not saved)"), grouped by setup change, with the analysis of each |
| 2026-11-09 | on-car barcode `C` under shadow, driven ~6 cm/s (no test ID) | 2ed56b2 + uncommitted S3 work (`fw 20:40:47`) | Buddy 3 | PASS (1 of 1) | `logs/2026-11-09_s3_barcode_after_fix.txt` (shadow section) | Note: shadow was casted right ontop of the barcode to see if the barcode can still be decoded. Fresh power-up: `S3 barcode: 'C' stored (ok 1, failed 0)`, decoded first time, no failed reads. One pass only, so not yet a robustness figure |
| — | NFR2 series (≥ 95 % recognition) | | | NOT RUN | | Power-cycle, then ~20 driven passes, mixed barcodes and directions, no hand passes, sensor untouched. Score = final `(ok, failed)` |
