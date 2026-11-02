# Integration log: how the subsystems were put together

Maintained by Buddy 4 (integration branch owner, Vehicle Controller writer).
Branch: `integrate/s4-onto-main` (on GitHub, head `ece5c3d`, 2026-11-02), based on
`main` at `59da21b` (2026-10-29). Not yet merged into `main`.
Test results are in `testing/integration/TEST_LOG.md` and
`testing/subsystem4/TEST_LOG.md`; Buddy 4's own status is in
`testing/subsystem4/BUDDY4_STATUS.md`.

---

## 1. Method

Big-bang integration ("everyone builds alone, merge at the end") was rejected: on
`main` the template's example code chained the subsystems together (S2 producer ->
S3 consumer -> S4 monitor), so no subsystem could be replaced alone, and a late merge
would break links, pins and priorities at once.

Used instead:
1. **Step 0, clean base:** remove the template chain in one commit so every subsystem
   can be replaced independently.
2. **One subsystem at a time** onto the integration branch, in dependency order:
   Motion (S2) first, because everyone needs it; then IMU (S4); then Line (S3).
   WiFi (S1) was already on `main`; Obstacle (S5) is still standalone.
3. **Interfaces first:** every arrow between subsystems is a row in `TEAM.md` §5,
   with one aligned 32-bit word or a kernel object, a single writer, and a host test.
4. **Every step must build** (`make -j8` and `make TEST=1 -j8`, 0 errors / 0 warnings,
   checked by grepping the output because `make`'s exit code is unreliable), **pass the
   PC tests**, and be **tested on the car** before the next step.
5. Changes in another buddy's folder are made only where integration needs them, are
   kept minimal, explained in the commit message, and left for that buddy to review.

---

## 2. Timeline

| Date | Commit | Step |
|---|---|---|
| 10-29 | `59da21b` | base: `main` with Buddy 1 telemetry (PR #3) and Buddy 2's motor foundation |
| 10-30 | `a9491ee` | **Step 0:** template producer/consumer/monitor chain removed (S2/S3 folders reduced to what compiles), S4 IMU folders added from `IMU_Branch` |
| 10-30 | `f53fee2` | **Buddy 2 merged:** `feature/buddy2-motion-control` (PR #2: PI speed control, encoder task). Conflicts in 5 motion files resolved by taking the feature branch; template semaphore/pool removed; S2 tests rewritten for the new API |
| 10-30 | `3138c39` | S4 `imu_terrain` task: P9, 10 ms cyclic handler, timing measurement, IMU_FAULT |
| 10-30 | `e44344f` | **IF-10** cumulative distance word S2 -> S4 (`motion_get_distance_mm()`), host test |
| 10-30 | `e2d6fd0` | integration docs: IF-10 row, task cores, INT-10 test defined |
| 10-30 | `70823b4` | first on-car run of the IMU task in the integrated image (run 21) |
| 10-31 | `4661b43`, `338c1b1` | kernel watchdog-tick fix; **encoder counting moved to GPIO edge interrupts** with the shared dispatcher `app_program/gpio_irq.c`; IMU recovery fix |
| 10-31 | `aaa249e`, `fbe19dd` | magnetometer calibration; hump detection (IF-11); Vehicle Controller test drive; S2 motor fixes (1 kHz PWM, PWM reset release, `motor.h` include, test power switch) |
| 11-02 | `215ec47` | **Buddy 3 merged:** `line_tracing_only` (line following, Code 39 barcode) |
| 11-02 | `535369b` | S2 duty mode with lease (`motion_set_duty`) so S3's line drive works next to PI speed mode |
| 11-02 | `6995300` | S3 barcode interrupt moved onto the shared GPIO dispatcher |
| 11-02 | `ece5c3d` | VC test drive off by default; build error if it and S3's drive are both on |

---

## 3. Problems found during integration and how they were solved

| # | Problem (how it showed) | Root cause | Fix | Evidence |
|---|---|---|---|---|
| 1 | Subsystems could not be replaced one by one | template chain S2 -> S3 -> S4 on `main` | step 0 removed it in one commit | `a9491ee` |
| 2 | Timing figures 42.5x too small after a power cycle | µT-Kernel port never starts the RP2040 watchdog tick that clocks the 1 µs TIMER (only the bootrom's USB boot did) | `kernel/sysdepend/pico_rp2040/cpu_clock.c` starts it at 1 µs | runs 22 (bad) / 29 (fixed) |
| 3 | 1 m push read -11 mm | kernel tick is **10 ms** (`CNF_TIMER_PERIOD 10`): S2's `tk_dly_tsk(1)` polled the encoders every 10-20 ms and missed almost every edge | encoder counts from GPIO edge interrupts (`gpio_irq.c` + `subsystem2/encoder.c`); counts single-writer in the ISR, "reset" = baseline in the task | run 29 (bad) / run 34: **990 mm** |
| 4 | Right encoder read 0 | wiring (fixed on the car by Buddy 4) | - | runs 32-33 |
| 5 | IMU never recovered after an SDA dropout | full I2C re-init after a dropout never completed (bus lines were free) | recovery first probes WHO_AM_I on the existing set-up; re-init only if SDA is held low | runs 29-30 (bad) / run 31: 4 of 4 recovered |
| 6 | Motors silent: "Motion fault (wheel stalled?)" | **PWM block held in reset**: `config.h` has `USE_PTMR 0`, so the kernel never releases it and every PWM write was lost (the Robo Pico motor test buttons did turn the wheels) | `motor_init()` releases the PWM reset (release only, no reset pulse, safe to repeat). Found independently by Buddy 3 | 11-01/11-02 car tests |
| 7 | PWM at 125 kHz (driver max 20 kHz) | the µT-Kernel port of S2's driver lost the bench test's clock divider (125 -> 1 kHz) | divider register written in `motor_init()` | `fbe19dd` |
| 8 | `motor.c` ignored its own power-limit switch | `motor.c` did not include `motor.h` | include added + `#error` if the switch is missing | `fbe19dd` |
| 9 | S3 did not compile on the integration branch | S3 uses `main`'s `motion_set_duty(left, right, lease)`; the integration branch had only PR #2's `motion_set_speed()` | duty mode added to S2 next to speed mode (signed duty, lease, 10 ms motion task, one command mutex) | `535369b` |
| 10 | Encoders or barcode would go silent (latent) | S3's `drv_ir.c` installed its own handler with `tk_def_int(13)`, replacing the shared dispatcher (or the reverse) | S3 attaches a callback with `gpio_irq_attach(GP7, …)` | `6995300` |
| 11 | Two tasks could command the motors at once | S3 line drive and VC test drive | VC test drive off by default, `#error` if both enabled | `ece5c3d` |
| 12 | Commands from other cores changed S2's PI state while the motion task used it | no lock on S2's command variables | one `TA_INHERIT` mutex around every command and the task's read | `535369b` |

---

## 4. Interfaces in use (TEAM.md §5)

| ID | From -> to | Mechanism | Status |
|---|---|---|---|
| IF-06 | S3 -> S2 | `line_state` word | S3 publishes it; S2 does not read it yet (S3 drives through `motion_set_duty` as a temporary stand-in) |
| IF-07 / IF-08 | VC <-> S1 | message buffers (telemetry, commands) | on `main` via PR #3; no VC producer yet |
| **IF-10** | S2 -> S4 | aligned `int32_t` distance (mm), single writer, `motion_get_distance_mm()` | **tested on the car (INT-10 PASS)**; Buddy 2 to sign off |
| **IF-11** | S4 -> VC, S1 | aligned `uint32_t` hump word, `imu_terrain_get_hump_word()` | done, round-trip test; layout PROPOSED to the VC/S1 |
| GPIO interrupts | S2, S3 (S5 planned) | shared dispatcher `app_program/gpio_irq.c`, core 0 | S2 encoders on the car; S3 barcode built, not yet enabled |
| Motion API | VC, S3 -> S2 | `motion_set_speed()` (PI), `motion_set_duty()` (lease), `motion_stop()` | both modes built; **Buddy 2 to choose the final API** |
| Q_motion, EVG_stop, Q_events, EVG_sys, g_fault_mask | VC | kernel objects | **not built yet** (Vehicle Controller still to do) |

---

## 5. Current state of each subsystem on the integration branch

| Subsystem | On the branch | Tested on the car in the integrated image |
|---|---|---|
| S1 WiFi / telemetry | from `main` (PR #3) | link-up tests need the hotspot and `NET=lwip MQTT=1`; not exercised |
| S2 Motion | PR #2 + fixes + duty mode | encoders and distance yes; wheels driven by software: pending re-test after the PWM reset fix |
| S3 Line / barcode | Buddy 3's branch, barcode on the dispatcher | not yet (merged 11-02) |
| S4 IMU / terrain | full task, calibration, recovery, hump | yes (runs 21-36); hump on the car pending |
| S5 Obstacle | standalone Pico SDK tests on `main` (PR #4), not in the image | no |
| Vehicle Controller | test-drive mode only | software path built; drive pending |

---

## 6. Rules for the next merges
- **S5 (servo, ultrasonic):** release the PWM reset the same way (never pulse it: that
  would wipe S2's motor set-up); ECHO edges via `gpio_irq_attach(GP17, …)`, never
  `tk_def_int(13)`; one task owns the servo's PWM slice 6.
- **Any new GPIO interrupt:** attach from `subsystemN_init()` or a core-0 task.
- **Only one module commands the motors at a time** until the VC owns `Q_motion`.
- **Priorities:** S2 still uses template values 12/14 on core 1; S3 6 and S4 9 on core 0.
  Move everything to the TEAM.md §4 table (P5..P10) in one agreed change.
- Before opening the integration PR to `main`: Buddy 2 and Buddy 3 review their folders,
  PR #2 is closed as superseded, stale branches (`IMU_Branch`, `master`, `Buddy-2`,
  `Buddy-3`, `ultra-implementation1`) are deleted.

---

## 7. How to reproduce the checks
```
cd build_make
make -j8            # grep the output for "error" / "warning"
make TEST=1 -j8
sh testing/subsystem2/host/run_tests.sh    # encoder counting + IF-10
sh testing/subsystem3/pc/run_tests.sh      # line logic, barcode, drive (70 checks)
sh testing/subsystem4/host/run_tests.sh    # hump height + IF-11 round trip
```
(On Windows checkouts strip CR from the scripts first, or run them from WSL.)
