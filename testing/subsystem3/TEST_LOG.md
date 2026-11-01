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
