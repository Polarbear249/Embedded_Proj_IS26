# Integration log: how the subsystems were put together

Maintained by Buddy 4 (integration branch owner, Vehicle Controller writer).
Last update **2026-11-04**. Integration branch `integrate/s4-onto-main`, head
`880204c` (pushed). Not yet merged into `main`: that is draft PR **#10**.
Test results: `testing/integration/TEST_LOG.md`, `testing/subsystem4/TEST_LOG.md`.

**Read section 0 first if you only have two minutes.**

---

## 0. Where we are (2026-11-04)

| Subsystem | Owner (GitHub) | On the integration branch | Tested on the car (integrated image) |
|---|---|---|---|
| S1 WiFi / telemetry | Buddy 1 (`FitrizDanialy`) | PR #3 + telemetry monitor PR #6 (merged 11-03) | console monitor yes; WiFi/MQTT link not yet in the integrated image |
| S2 Motion | Buddy 2 (`dinesh379`) | PR #2 + integration fixes + duty mode + e-stop latch | encoders (1 m -> 990 mm), PI speed mode drives straight 1.5 m and climbs a 10 mm ramp; e-stop latches |
| S3 Line / barcode | Buddy 3 (`zaynulabidyn-coder`) | `line_tracing_only` + barcode on the GPIO dispatcher | all 3 IR sensors toggle (S3-U06 PASS); **line following loses the line within ~1 s**; barcode disabled |
| S4 IMU / terrain | Buddy 4 (`Polarbear249`) | full task, calibration, side + front hump model, smoothing, trace | calibration done on the raised mount (runs 37-39); hump: real-trace replay 1 hump / 9 mm for a 10 mm ramp, **car re-run pending** |
| S5 Obstacle | Buddy 5 (`TypeAlyce`) | **not yet**: rework in PR #9 (built, not run on the car) | no |
| Vehicle Controller | Buddy 4 | GP20 e-stop, VC test drive; IF-09 obstacle stop in PR #9 | e-stop yes; mission logic (barcode -> junction -> turn) **not written** |

Build state: integration branch `make -j8` and `make TEST=1 -j8` 0 errors, 0 warnings.
**`main` does not build** (12 errors in `subsystem5/` since PR #5); PR #9 fixes it.

---

## 1. Branches: what each one is for

```text
main  (the team's real trunk: PRs #3, #4, #5 merged here)
 └── integrate/s4-onto-main   integration branch (B1+B2+B3+B4+VC)  ── PR #10 (draft) ──> main
      └── s5-rework-onto-integration   Buddy 5's work, reworked     ── PR #9 ──> integration
```

| Branch | Status | What to do |
|---|---|---|
| `main` | **the final branch** everything ends up in; currently broken build (S5) | becomes the only trunk after PR #10 |
| `integrate/s4-onto-main` | where all subsystems are merged and tested together on the car | merge into `main` via PR #10, then retire it |
| `s5-rework-onto-integration` | PR #9, Buddy 5 to review | merge into the integration branch |
| `ultra-implementation1` | PR #8 (and closed #7): Buddy 5's original, already included in #9 | close #8 after #9 merges |
| `feature/buddy2-motion-control` | PR #2: Buddy 2's original, already included in the integration branch | close #2 after #10 merges |
| `buddy1-telem-monitor`, `line_tracing_only` | already merged into the integration branch | delete after #10 |
| `master` | **obsolete.** Last change 10-26; its only unique commits are early S4 work, superseded by the integration branch | **GitHub still shows it as the default branch: switch the default to `main`**, then delete it |
| `IMU_Branch` | obsolete standalone S4 (`6745fa8` deletes other subsystems: never merge) | delete |
| `Buddy-2`, `Buddy-3`, `buddy1-wifi`, `ultrasonic-isolate2` | 0 commits ahead of `main` (fully merged) | delete |

**Is `master` the final branch? No.** All merged work is on `main`; `master` only
looks important because GitHub lists it as the default (Settings -> General ->
Default branch, owner `Polarbear249` can change it). New clones and new PRs default
to `master` until that is switched, which is how work could land in the wrong place.

---

## 2. Integration order (from now)

1. **Buddy 5 reviews PR #9** (section 4). On merge, resolve the only conflict,
   `.vscode/settings.json`, by **deleting** the file: `main` removed it (`9c45ceb`,
   hard-coded absolute path), while the integration branch still has it and
   `e20af4e` added `"cmake.sourceDirectory": "C:/Users/thans/..."` (a personal path).
2. Build `make -j8` + `make TEST=1 -j8`, PC tests, then on the car: obstacle stop
   with a hand at ~10 cm, profile printed, GP20 resumes.
3. **Buddy 2 and Buddy 3 review their folders** on the integration branch (PR #10
   lists Buddy 4's edits in `subsystem2/`).
4. Fix line following enough for a demo lap (Buddy 3 with Buddy 2, section 4).
5. **Merge PR #10 into `main`** (merge commit, not squash: keeps everyone's commits).
6. Switch the GitHub default branch to `main`; close #2 and #8; delete the stale
   branches in section 1.
7. **From then on, one trunk:** every change is a short branch off `main`
   (`s3-left-sensor`, `vc-mission`, ...), a PR to `main`, merged only after it builds
   (both builds, 0 warnings), passes the PC tests and has a car test logged. Changes
   to shared files (`app.h`, `app_main.c`, `TEAM.md`, `gpio_irq.*`, `vehicle_controller.*`)
   get a reviewer from the affected buddy.

Why this order: S5 must go in before `main` (otherwise #10 carries `main`'s broken
S5 build), and reviews before the merge to `main` because PR #10 contains edits in
Buddy 2's and Buddy 3's folders made during integration.

---

## 3. What is still needed for the full mission

| # | Work | Owner | Depends on |
|---|---|---|---|
| 1 | **Line following that keeps the line** (tuning, speed mode or a duty floor, left sensor) | B3 + B2 | - |
| 2 | **Barcode A-Z, both directions, sensor on the left** (CLAUDE.md 1b): code table with source, decoder outputs a 5-bit character, not a command; pin change approved first | B3 | pin approval |
| 3 | **Cross-junction event** distinct from "both sensors black" | B3 | 1 |
| 4 | **Mapping over MQTT** (4 distinct characters -> forward / U-turn / left / right, binary frame, `Q_cmd`), PC script to send it, decoder script for telemetry | B1 | VC frame agreed |
| 5 | **VC mission logic**: pending character slot, applies the MQTT mapping at the cross-junction, sends the movement to Motion (`Q_motion`) | B4 | 2, 3, 4 |
| 6 | **`EVG_stop`, `Q_events` + `EVG_sys`, `g_fault_mask`** (one shared header with every packed layout, round-trip tests) | B4 with all | - |
| 7 | **Obstacle bypass + line reacquisition** (Week 10 metrics: detection, width, direction, reacquisition, < 3 s) | B5 + B4 (VC) + B2 | PR #9, 1 |
| 8 | Hump on the car: right-wheel ramp, full-width ramp, flat run (FR7 evidence) | B4 | - |
| 9 | Priorities: everything on the TEAM.md §4 table (P5..P10), one agreed change | all | - |
| 10 | WiFi + MQTT in the integrated image on the power bank: A1 (WiFi does not disturb motion/sensing) and NFR12 (WiFi loss does not stop the car) | B1 + B4 | 4 |
| 11 | Week 13 evidence: data representation table, RAM / message sizes before-after, P-task timing (measured / estimate / derived) | all | - |

---

## 4. What Buddy 4 needs from each buddy

### Buddy 1 (`FitrizDanialy`)
1. A switch to turn the telemetry console printout off (it prints ~12 lines/s; fine
   for calibration, too much during car runs). The MQTT path stays as it is.
2. Barcode field: report the decoded **character (5 bits, 0-25)** per CLAUDE.md 1b,
   not the old 2-bit LEFT/RIGHT/STRAIGHT/U-TURN command.
3. The MQTT **mapping command** (topic, binary frame with version/length byte,
   validation: four distinct characters A-Z, complete frame) and the PC script.
4. The **telemetry decoder script** (the 13-byte frame is unreadable in `mosquitto_sub`).
5. After PR #9: read the obstacle result from IF-09 (VC owns it) instead of the
   proposed `s5_get_obstacle()` getter, or agree the getter with Buddy 5.

### Buddy 2 (`dinesh379`)
1. **Review `subsystem2/`** on the integration branch: `motion_get_distance_mm()`
   (IF-10), `motion_set_duty()` + lease, `motion_estop`/`clear`/`is_estopped`, PWM reset
   release, 1 kHz divider, edge-interrupt encoders, `command_mutex`.
2. **Decide the final drive API.** Duty mode is open loop: on the 10 mm ramp the
   line follower stalled with the climbing wheel at duty 88 (8.8 %, cap 20 %), while PI
   speed mode climbed it every time (runs 40-45). Line following should go through
   speed mode, or duty mode needs a stall/torque floor.
3. `MOTOR_TEST_HIGH_POWER 1` (50 % max, added for laptop-USB power): keep or tune.
4. Priorities 12/14 -> the TEAM.md table; owner of `g_fault_mask` (VC per Version C?).
5. Encoder L/R labels are confirmed correct on the car (run 38): nothing to change.

### Buddy 3 (`zaynulabidyn-coder`)
1. **Review `subsystem3/`** as merged (barcode moved to `gpio_irq_attach`).
2. **Line following** (on the car the line is lost within ~1 s; sensors are fine,
   S3-U06 PASS): try `DRIVE_BASE_DUTY` ~90, a stronger `LINE_ERR_GUESS`, sensor height
   2-3 mm on the real mat; the right-only + centre design cannot see a line leaving
   on the left. Use one change at a time and log each run.
3. `line_if.h` hard-codes `A = LEFT, B = RIGHT, C = STRAIGHT, D = U-TURN`: per 1b the
   mapping must come over MQTT; output the character instead.
4. Barcode sensor on the **left** + A-Z both directions: propose the pin/role change
   for approval before any wire moves.
5. S3-U01 fails only when it overlaps the WiFi start-up at boot (331 ms for 200 ms);
   with a 5 s delay it passes (209.8 ms). Either move U01 later in the test or the
   team sets `APP_TEST_SETTLE_MS` (default 0) in `app_main.c`.

### Buddy 5 (`TypeAlyce`)
1. **Review PR #9** (details: `testing/integration/S5_REWORK_FOR_BUDDY5.md`).
2. Confirm: which servo end is left (178 deg assumed), 110 deg = straight ahead,
   15 cm alert threshold, whether a VC scan request is still wanted.
3. First car test (section 2 step 2), then the bypass + recovery design with the VC.

---

## 5. Timeline

| Date | Commit | Step |
|---|---|---|
| 10-29 | `59da21b` | base: `main` with Buddy 1 telemetry (PR #3) and Buddy 2's motor foundation |
| 10-30 | `a9491ee` | **Step 0:** template producer/consumer/monitor chain removed, S4 folders added |
| 10-30 | `f53fee2` | **Buddy 2 merged** (PR #2: PI speed control, encoder task) |
| 10-30 | `3138c39`, `e44344f` | S4 `imu_terrain` task (P9, 10 ms); **IF-10** distance S2 -> S4 |
| 10-31 | `4661b43`, `338c1b1` | kernel watchdog-tick fix; encoders on GPIO edge interrupts (`gpio_irq.c`) |
| 10-31 | `aaa249e`, `fbe19dd` | magnetometer calibration; hump detection (IF-11); VC test drive; S2 motor fixes |
| 11-02 | `215ec47` | **Buddy 3 merged** (`line_tracing_only`) |
| 11-02 | `535369b`, `6995300`, `ece5c3d` | S2 duty mode; S3 barcode on the dispatcher; VC test drive off by default |
| 11-02 | `8aaf476` | GP20 emergency stop (latched), hump fixes, docs |
| 11-03 | `10972f0` | IMU recalibrated on the raised mount; `APP_TEST_ONLY` test switch |
| 11-03 | `f011d8c` | **Buddy 1 merged** (PR #6: telemetry console monitor) |
| 11-03 | PR #9 (`a7c2b38`, `e07f370`) | **Buddy 5 merged into a review branch** and reworked (not yet in the integration branch) |
| 11-04 | `d611806` | S4 side-hump model (W 113 mm), thresholds 4.0 / 2.5 deg, impact 800 mg, 16 mm smoothing, drive-off guard, trace switch |
| 11-04 | `3b57c29`, `c1899cc`, `880204c` | S3 tick-loss diagnosis + logs; Buddy 5 note; real-trace replay test |

---

## 6. Problems found during integration and how they were solved

| # | Problem (how it showed) | Root cause | Fix | Evidence |
|---|---|---|---|---|
| 1 | Subsystems could not be replaced one by one | template chain S2 -> S3 -> S4 on `main` | step 0 removed it | `a9491ee` |
| 2 | Timing 42.5x too small after a power cycle | port never starts the RP2040 watchdog tick (1 µs TIMER) | `cpu_clock.c` starts it | runs 22 / 29 |
| 3 | 1 m push read -11 mm | 10 ms kernel tick: polled encoders missed edges | GPIO edge interrupts | run 34: 990 mm |
| 4 | Right encoder read 0 | wiring | fixed on the car | runs 32-33 |
| 5 | IMU never recovered after an SDA dropout | full re-init never completed | probe WHO_AM_I first | run 31: 4/4 |
| 6 | Motors silent | PWM block held in reset (`USE_PTMR 0`) | `motor_init()` releases it | 11-01/02 |
| 7 | PWM at 125 kHz (driver max 20 kHz) | lost clock divider | DIV written in `motor_init()` | `fbe19dd` |
| 8 | `motor.c` ignored its power switch | missing include | include + `#error` | `fbe19dd` |
| 9 | S3 did not compile | needed `main`'s `motion_set_duty` | duty mode next to speed mode | `535369b` |
| 10 | Encoders or barcode could go silent | S3 replaced the shared handler (`tk_def_int(13)`) | `gpio_irq_attach(GP7)` | `6995300` |
| 11 | Two tasks could command the motors | S3 drive + VC test drive | off by default, `#error` if both | `ece5c3d` |
| 12 | Commands raced the PI state | no lock | `TA_INHERIT` mutex | `535369b` |
| 13 | **`main` stopped building** | PR #5's S5 code (UINT8, `hardware/pwm.h`, `gpio_set_pin` args) | PR #9 | worktree builds 11-03 |
| 14 | S5 would fight the line follower / stall the IMU | PR #8: second VC in `subsystem2/` driving the motors; `motion_stop()` from S5; ~35 ms ECHO busy-wait at P8; stack mailbox message; servo DIV in reserved bits; `%f` | PR #9 (dispatcher ECHO, IF-09 to the VC, fixes) | `S5_REWORK_FOR_BUDDY5.md` |
| 15 | S3-U01: 200 ms read 331 ms | test window overlaps the WiFi start-up at boot (first guess "telemetry printing" was **wrong**, disproved by a rerun) | `APP_TEST_SETTLE_MS` 5000 -> 209.8 ms PASS | `logs/2026-11-03_s3_only_tick_loss.txt` |
| 16 | Car stalled half-way up a 10 mm ramp | duty mode open loop, climbing wheel at 8.8 % (cap 20 %) | PI speed mode climbs it (to decide: B2/B3) | runs 40-45 |
| 17 | Hump never ended / false starts / 30 mm on flat | driving vibration (pitch sd 1.9 deg, uncorrelated beyond 6 mm) and drive-off kick (+14 / -20 deg) | 16 mm distance smoothing + 60 mm drive-off guard; real-trace replay test | run 45 trace, `replay_trace.c` |

---

## 7. Interfaces in use (TEAM.md §5)

| ID | From -> to | Mechanism | Status |
|---|---|---|---|
| IF-06 | S3 -> S2 | `line_state` word | published; S2 does not read it yet (S3 drives through `motion_set_duty`) |
| IF-07 / IF-08 | VC <-> S1 | message buffers (telemetry, commands) | S1 side done; VC producer not yet (PR #6's monitor gathers the snapshot meanwhile) |
| IF-09 | S5 -> VC | message buffer `vc_mbf_profile`, packed `uint32_t` (3 x 9-bit cm, kind, best) | **in PR #9**, round-trip tests S5-U08/U09 |
| **IF-10** | S2 -> S4 | aligned `int32_t` distance mm | **tested on the car (INT-10 PASS)** |
| **IF-11** | S4 -> VC, S1 | aligned `uint32_t` hump word | done, round-trip test; height width (8 bits) open |
| GPIO interrupts | S2, S3, S5 | shared dispatcher `gpio_irq.c`, core 0 | S2 on the car; S3 built (barcode disabled); S5 in PR #9 |
| Motion API | VC, S3 -> S2 | `motion_set_speed()` (PI), `motion_set_duty()` (lease), `motion_estop()` | **B2 to choose the final API** (problem 16) |
| Q_motion, EVG_stop, Q_events, EVG_sys, g_fault_mask, Q_cmd mapping | VC | kernel objects | **not built yet** (section 3 items 4-6) |

---

## 8. Rules for the next merges
- Every merge: `make -j8` and `make TEST=1 -j8` with **0 errors / 0 warnings** (grep the
  output; `make`'s exit code is unreliable), the PC tests, and a logged car test.
- GPIO interrupts only through `gpio_irq_attach()` from `subsystemN_init()` or a
  core-0 task; never `tk_def_int(13)`. Never pulse the PWM reset (it wipes the motors).
- **One module commands the motors at a time** until the VC owns `Q_motion`.
- No personal paths or secrets in tracked files (`.vscode/settings.json`,
  `config/wifi_secrets.h`); no build outputs (`.uf2`, `.elf`, `.exe`).
- Never build with `CRASH_REPORT=1` on the car (GP0/GP1 = right encoder).

---

## 9. How to reproduce the checks
```
cd build_make
make -j8                 # grep the output for "error" / "warning"
make TEST=1 -j8
sh testing/subsystem2/host/run_tests.sh    # encoder counting + IF-10
sh testing/subsystem3/pc/run_tests.sh      # line logic, barcode, drive
sh testing/subsystem4/host/run_tests.sh    # hump model: 19 simulated + run 45 replay
```
Images for single-subsystem tests: set `APP_TEST_ONLY` (1..5) in `app_main.c` for a
`TEST=1` build that runs only that subsystem's tests (keep 0 in commits).
(On Windows checkouts strip CR from the scripts first, or run them from WSL.)
