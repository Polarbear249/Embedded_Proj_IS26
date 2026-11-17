# Integration log: how the subsystems were put together

Maintained by Buddy 4 (integration branch owner, Vehicle Controller writer).
Last update **2026-11-17**. `main` is the only trunk (section 2 step 7): PR #10
(all subsystems, 11-13) and PR **#12** (S4 lift / overturn stop, 11-17) are merged.
**Not on `main` yet:** `integration-s2-s5-for-buddy5` (Buddy 2 / 3 / 5 work, section 2
step 8), PR #11 (`vc-mission`), `s2-duty-start-kick` (stopgap for problem 19).
Test results: `testing/integration/TEST_LOG.md`, `testing/subsystem4/TEST_LOG.md`.

**Read section 0 first if you only have two minutes.**

---

## 0. Where we are (2026-11-17)

| Subsystem | Owner (GitHub) | On the integration branch | Tested on the car (integrated image) |
|---|---|---|---|
| S1 WiFi / telemetry | Buddy 1 (`FitrizDanialy`) | PR #3 + telemetry monitor PR #6 (merged 11-03) | console monitor yes; WiFi/MQTT link not yet in the integrated image |
| S2 Motion | Buddy 2 (`dinesh379`) | PR #2 + integration fixes + duty mode + e-stop latch | encoders (1 m -> 990 mm), PI speed mode drives straight 1.5 m and climbs a 10 mm ramp; e-stop latches |
| S3 Line / barcode | Buddy 3 (`zaynulabidyn-coder`) | `line_tracing_only` + barcode on the GPIO dispatcher; **11-13: outputs the character 0-25, no hard-coded mapping** (PC: A-Z both directions PASS) | all 3 IR sensors toggle (S3-U06 PASS); **line following works on `main`** (follows and recovers, S4 lift runs 10-13, up to 3 m); the stop after about 1 s seen on 11-16 was the S4 lift detector (problem 18, fixed). Open: wheels stall from rest (problem 19). Barcode disabled on `main`; Buddy 3's A-Z decode on the car is on the unmerged branch |
| S4 IMU / terrain | Buddy 4 (`Polarbear249`) | full task, calibration, side + front hump model, smoothing, trace | calibration done on the raised mount (runs 37-39); **hump on the car: 23 mm ramp under one wheel -> mean 24.6 mm (n = 8, 21-27, test drive)**; **lift / overturn stop S4-U38..U41 PASS** (PR #12, runs 01-13). Open: hump while line following is unreliable until problem 19 is fixed (a stalled wheel rocks the car -> fake humps); ramp run with the new lift input |
| S5 Obstacle | Buddy 5 (`TypeAlyce`) | PR #9 merged 2026-11-13; turn / reverse / bypass logic is on the unmerged branch | alert -> VC stop and profile word seen once on a stationary car (INT-09 PARTIAL); moving-car test owed |
| Vehicle Controller | Buddy 4 | GP20 e-stop, VC test drive, IF-09 obstacle stop, **IF-12 lift / overturn stop** | e-stop yes (INT-13), lift stop yes (INT-12); mission logic (barcode -> junction -> turn): PR #11 logs the decision only; Buddy 3's branch does it inside S3 as a temporary stand-in |

Build state (2026-11-17, `main` = `c9a478d` + the uncommitted start kick): `make -j8` and
`make TEST=1 -j8` 0 errors, 0 warnings; PC tests S2, S3 (72 checks), S4 (hump, lift
simulation, replay of car runs 03-13) all pass.

Review fixes made by Buddy 4 before the merge (Buddy 2 / 3 / 1 please read the commits):
- `099012b` S3 outputs the barcode **character** (IF-06 `bc_char` 5 bits), telemetry
  frame **v2** (14 bytes, `bc_char` 5 bits). Buddy 1: your PC decoder must use v2.
- `0e1f617` Motion and encoder-publish tasks on **P5** (were 14/12). Run on the car
  11-16/17: the car drives; IMU task period 2.6-17.5 ms, 1 late period in 47 s (INT-14).
- `eef3961` `testing/subsystem5/.vscode` untracked.
Still open from the review: tabs in S1/`app_main.c` (Barr-C layout), `MOTOR_TEST_HIGH_POWER 1`
(B2 to decide), S1 LED on GP19, drive API (problem 16).

---

## 1. Branches: what each one is for

```text
main  (the only trunk: PRs #3, #4, #5, then #9 via the integration branch, then #10)
 └── short branches off main (vc-mission, s3-left-sensor, ...) ── PR ──> main
```

| Branch | Status | What to do |
|---|---|---|
| `main` | **the only trunk**; contains every subsystem since 2026-11-13 (PR #10) | branch off it |
| `integrate/s4-onto-main` | merged into `main` 2026-11-13 (PR #10) | delete once everyone has pulled `main` |
| `s5-rework-onto-integration` | PR #9, merged into the integration branch 2026-11-13 (`d4de6ad`) | delete |
| `ultra-implementation1` | PR #8 (and closed #7): Buddy 5's original, included via #9 | close #8, delete |
| `feature/buddy2-motion-control` | PR #2: Buddy 2's original, included in #10 | close #2, delete |
| `buddy1-telem-monitor`, `line_tracing_only` | merged / superseded | delete |
| **`integration-s2-s5-for-buddy5`** | **NOT merged: DO NOT DELETE** (corrected 2026-11-17; this table wrongly said "merged / superseded"). 12 commits, 11-04 to 11-11, that are not on `main`: Buddy 2 (`e185696`, `a461608`, `2ed56b2`: S2 motion work, IF-06 / IF-10 sign-off, straight-move tuning + ruler logs), Buddy 3 (`35ea83d`: barcode A-Z decode on the car, `barcode_nav.c`), Buddy 5 (`ec4cf41`..`f6639ab`: obstacle stop, turn and reverse logic, `s5_get_obstacle()`, `s3_drive_set_enabled()`) | merge into `main` as its own PR after PR #12 (section 2 step 8) |
| ~~`master`~~ | **deleted 2026-11-04**; GitHub default branch switched to `main` the same day. Its two unique commits (`5368c35`, `6db49e1`, early S4 work, superseded) are kept in the tag `archive/master-2026-10-26` | nothing |
| `IMU_Branch` | obsolete standalone S4 (`6745fa8` deletes other subsystems: never merge) | delete |
| `Buddy-2`, `Buddy-3`, `buddy1-wifi`, `ultrasonic-isolate2` | 0 commits ahead of `main` (fully merged) | delete |

**`main` is the final branch.** All merged work is on `main`. Until 2026-11-04
GitHub listed `master` as the default branch, so the repo front page showed
`master`'s last change (10-26) and new PRs defaulted to it; now `main` is the default
and `master` is gone. **Everyone: `git fetch --prune` and branch off `main`.**

---

## 2. Integration order (from now)

1. ~~Buddy 5 reviews PR #9~~ **merged 2026-11-13** by Buddy 4 (conflict
   `.vscode/settings.json` resolved by deleting it). Buddy 5: please still read
   `S5_REWORK_FOR_BUDDY5.md` and answer the questions in section 4.
2. ~~Build + PC tests~~ done 2026-11-13 (0 errors / 0 warnings, all PC tests pass).
   **Still to do on the car:** obstacle stop with a hand at ~10 cm, profile printed,
   GP20 resumes; re-check timing with Motion on P5.
3. ~~Reviews of `subsystem2/` and `subsystem3/`~~ made by Buddy 4 as review fixes
   (section 0); Buddy 2 and Buddy 3 please read the commits and object if needed.
4. Fix line following enough for a demo lap (Buddy 3 with Buddy 2, section 4).
5. ~~Merge PR #10 into `main`~~ **done 2026-11-13** (merge commit, history kept).
6. Close #2 and #8; delete the stale branches in section 1.
7. **From then on, one trunk:** every change is a short branch off `main`
   (`s3-left-sensor`, `vc-mission`, ...), a PR to `main`, merged only after it builds
   (both builds, 0 warnings), passes the PC tests and has a car test logged. Changes
   to shared files (`app.h`, `app_main.c`, `TEAM.md`, `gpio_irq.*`, `vehicle_controller.*`)
   get a reviewer from the affected buddy.

8. **Merge `integration-s2-s5-for-buddy5` (found unmerged 2026-11-17).** It split
   from `425640b` (11-04), so it has none of the 11-13 review fixes or the lift stop.
   Trial merge into `s4-lift-stop` (`git merge-tree`, nothing changed on disk):
   **10 files conflict**: `TEAM.md`; `subsystem2/subsystem2.c`, `subsystem2.h` (P5
   priorities vs Buddy 2's changes); `subsystem3/line_if.c`, `line_if.h`,
   `subsystem3.h`, `subsystem3_test.c`, `task_line.c`, `testing/subsystem3/pc/test_s3.c`
   (the 11-13 `bc_char` review fix vs Buddy 3's own A-Z work and `barcode_nav`);
   `vehicle_controller.c` (lift check and GP20 rule vs Buddy 5's avoidance logic).
   To decide with the owners while resolving: Buddy 3's `barcode_nav` keeps the
   letter -> movement map and performs the turn inside S3 as a stated TEMPORARY
   stand-in (CLAUDE.md 1b gives both to the VC, PR #11); keep `vc_check_lifted()`
   first in `vc_estop_task` and the "stop kept while lifted" GP20 rule; keep the S4
   lift input (`imu_vert_mg()`). Then both builds, all PC tests, one car run.

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
| 5 | **VC mission logic**: pending character slot, applies the MQTT mapping at the cross-junction, sends the movement to Motion (`Q_motion`). PR #11 has it logged-only; Buddy 3's unmerged `barcode_nav` does map + turn inside S3 (temporary): **decide which one stays** | B4 + B3 | 2, 3, 4 |
| 6 | **`EVG_stop`, `Q_events` + `EVG_sys`, `g_fault_mask`** (one shared header with every packed layout, round-trip tests) | B4 with all | - |
| 7 | **Obstacle bypass + line reacquisition** (Week 10 metrics: detection, width, direction, reacquisition, < 3 s) | B5 + B4 (VC) + B2 | PR #9, 1 |
| 8 | Hump on the car: one-wheel 23 mm ramp done (24.6 mm mean, n = 8, test drive). Still: full-width ramp, and a run under the line follower after problem 19 is fixed | B4 | problem 19 |
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
6. **Wheels stall when starting from rest in duty mode (2026-11-17, section 6 #19).**
   A turning wheel keeps going at low duty, but from rest it often does not start:
   left wheel stuck at 120/1000 while the right turned (S4 lift runs 09, 12); both
   stuck at 152/88 (run 13); inner wheel at 69-88 stalls. Duty mode has no start kick
   and no stall check, so the car silently stops or drags off the line. Asked:
   - a. **Measure** each wheel on the floor, power bank (and once on laptop USB):
     the lowest duty that starts it from rest, and the lowest that keeps it turning.
     Log it in `testing/subsystem2/`. The values below depend on these numbers.
   - b. **Start kick in duty mode** (inside `motion_tick()`, no API change): if a
     wheel is commanded non-zero but its encoder has not moved for ~50 ms, apply
     `MOTION_START_PWM` for 2-3 ticks, then the commanded duty.
   - c. **Minimum running duty**: raise any non-zero command to at least the measured
     keep-turning level, so the inner wheel of a correction does not fade out.
   - Check: stop / release on the line and restart from rest 5 times; both encoder
     counts rise within 0.2 s every time. Logs showing the stalls (S4 folder):
     `testing/subsystem4/logs/2026-11-16_lift_run09_*`, `2026-11-17_lift_run12_*`,
     `2026-11-17_lift_run13_*` (search `enc L`).
   - Long term (with Buddy 3): IF-06 has S2 steering; closed-loop wheel speeds would
     remove this, but the PI loop runs every 100 ms, too slow to steer as it is.

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
6. **Line drive powers are below the wheels' start threshold (2026-11-17, section 6
   #19; Buddy 2 item 6).** `line_drive.h` sends open-loop 69-152 (BASE 120 +-32,
   search -69/200); from rest the wheels often do not start at these values. If Buddy
   2's start kick (Buddy 2 6b) is not ready before the demo, stopgap in
   `subsystem3/line_drive.h` / `line_drive.c`:
   - raise `DRIVE_BASE_DUTY` 120 -> ~150-160 (max correction then ~192, under
     `MOTION_MAX_DUTY` 200);
   - in `line_drive_clamp()`, lift any forward command between 0 and Buddy 2's
     measured keep-turning duty up to that duty;
   - retune `DRIVE_KP_NUM` if the car now overshoots (it drives faster). One change
     at a time, one logged run each.
7. **Side effects for S4, fixed when the stall is fixed:** a car pushing on one
   stalled wheel rocks and the hump detector logs fake humps (20 mm in run 12,
   76 mm in run 11). The lift detector is no longer fooled by the line follower's
   jolts (S4 now uses the acceleration along gravity, section 6 #18).

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
| 11-04..11-11 | `e185696`..`f6639ab` | **Buddy 2 / 3 / 5 on `integration-s2-s5-for-buddy5`** (closed-loop line drive, moves and turns, `Q_motion` / `EVG_stop`; barcode A-Z on the car, `barcode_nav`; obstacle turn / reverse): **not merged** |
| 11-13 | `d4de6ad`, `099012b`, `0e1f617`, `b08435c` | PR #9 (S5) merged; review fixes (S3 character output, telemetry frame v2, S2 at P5); **PR #10 -> `main`** |
| 11-13 | `ff187b8` | prof's change: IMU impact detection removed, **lift / overturn stop** (`imu_lift.c`, VC latch, IF-12) |
| 11-16 | `9b437ee` | lift tuned on the car (clamp 300 mg, impact veto 1 g); line-follower false stops found; **lift input = acceleration along gravity** |
| 11-17 | `31e3628`, `a0ebff9`, `c9a478d` | S4-U38..U41 PASS (runs 12-13); **PR #12 -> `main`** |

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
| 18 | Line follower stopped by itself ~1 s after starting (power bank, after the 11-13 lift/overturn change) | S4 lift detector integrated \|a\| - g: the S3 duty steps shake the car (forward / sideways / rocking), which adds to \|a\| as much as a hand lift (282 vs 233 mm/s); no clamp/leak setting separated them | lift input = acceleration **along gravity** (`imu_vert_mg()`): jolts cancel, lifts stay positive. Line following 0 false stops, lifts caught, GP20 kept while lifted | S4 lift runs 10-13, `replay_lift.c ... vz`, S4 TEST_LOG U38-U41 |
| 19 | Wheel(s) stop and stay stopped while line following | duty mode is open loop with no start kick: from rest the wheels need more than the 69-152 duty S3 sends (static > kinetic friction) | **start kick added 2026-11-17** by Buddy 4 in `subsystem2/motion_control.c` (branch `s2-duty-start-kick`, Buddy 2 to review): a commanded wheel whose encoder has not moved for 50 ms is driven at `MOTION_START_PWM` until it moves (max 50 ms, 200 ms rest). Builds + PC tests pass; **not yet run on the car**. Note: the unmerged `integration-s2-s5-for-buddy5` has Buddy 2's closed-loop `motion_line_drive()`, which replaces this when that branch is merged | S4 lift runs 09, 12, 13 (`enc L` frozen) |

---

## 7. Interfaces in use (TEAM.md §5)

| ID | From -> to | Mechanism | Status |
|---|---|---|---|
| IF-06 | S3 -> S2 | `line_state` word | published; S2 does not read it yet (S3 drives through `motion_set_duty`) |
| IF-07 / IF-08 | VC <-> S1 | message buffers (telemetry, commands) | S1 side done; VC producer not yet (PR #6's monitor gathers the snapshot meanwhile) |
| IF-09 | S5 -> VC | message buffer `vc_mbf_profile`, packed `uint32_t` (3 x 9-bit cm, kind, best) | on `main`; round-trip tests S5-U08/U09; on the car INT-09 **PARTIAL** (stationary car) |
| **IF-10** | S2 -> S4 | aligned `int32_t` distance mm | **tested on the car (INT-10 PASS)** |
| **IF-11** | S4 -> VC, S1 | aligned `uint32_t` hump word | done, round-trip test; S1 monitor shows it (INT-11); height width (8 bits) open |
| **IF-12** | S4 -> VC | `imu_terrain_is_lifted()` (aligned state word), VC polls every 20 ms and latches `motion_estop()` | **tested on the car (INT-12, S4-U38..U41 PASS)** |
| GPIO interrupts | S2, S3, S5 | shared dispatcher `gpio_irq.c`, core 0 | S2 and S5 (ECHO) on the car; S3 built (barcode disabled on `main`) |
| Motion API | VC, S3 -> S2 | `motion_set_speed()` (PI), `motion_set_duty()` (lease), `motion_estop()` | e-stop tested on the car (INT-13). Buddy 2 chose `motion_line_drive()` (PI, with lease) on the unmerged branch; `main` still uses duty mode (problems 16, 19) |
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
sh testing/subsystem4/host/run_tests.sh    # hump model + run 45 replay; lift simulation + replay of car runs 03-13
```
Images for single-subsystem tests: set `APP_TEST_ONLY` (1..5) in `app_main.c` for a
`TEST=1` build that runs only that subsystem's tests (keep 0 in commits).
(On Windows checkouts strip CR from the scripts first, or run them from WSL.)
