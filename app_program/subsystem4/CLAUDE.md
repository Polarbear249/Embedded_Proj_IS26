<!-- Moved from the root CLAUDE.md (section 8) on 2026-10-30. Section numbers below refer to the root CLAUDE.md. -->

## 8. IMU / terrain subsystem (Buddy 4)

Design (Week 6 Section 6.4, Tribunal 1): GY-511 on **I2C1 (GP2/GP3), sole user**; task
`imu_terrain` at **P9, 10 ms**; reports events through `Q_events`; **never drives motors**;
hump result published as one aligned 32-bit word. Requirements: FR7 (hump detection and
maximum height in mm), FR8 (classify turning, hump ascent/descent, **lifted / overturned**;
"sudden impact" removed, see below), NFR6 (rigid mount, documented X/Y/Z), NFR17 (single
owner). Assumptions: A6 (stable data) and A7 (I2C1 under µT-Kernel), both Open until the
tests below pass.

**Requirement change (prof, 13 Nov 2026): collision detection removed; lift / overturn
stop added.** "Collision detection using the IMU will be removed. Instead, the IMU should
be used to detect when the robot has been lifted or overturned. When such a condition is
detected, the motors should immediately stop. During the demonstration, lifting the robot
will be used to signal the end of the run."
- Code: `imu_lift.c/.h` (pure, PC-tested in `testing/subsystem4/host/test_lift.c`), called
  every sample by `imu_terrain`. States `IMU_TERRAIN_LIFTED` (6) and
  `IMU_TERRAIN_OVERTURNED` (7) (still 4 bits); events `IMU_EVT_LIFTED` (3, the old IMPACT
  code) and `IMU_EVT_OVERTURNED` (5) (still 3 bits). `imu_terrain_is_lifted()` is the
  reader API.
- **Lifted:** `dev` = acceleration along the gravity estimate minus g_ref (g_ref = |a| at
  level zero; since 2026-11-16 not `|a| - g_ref`, which counted line-follower jolts as
  lifts, runs 10-11), minus a slow baseline
  (about 2.6 s), integrated with a leak (320 ms) = estimated upward speed. LIFTED at
  **200 mm/s** (derived). **Overturned:** pitch or roll beyond **45 deg for 200 ms**.
- **Stop:** the IMU only reports. The Vehicle Controller polls `imu_terrain_is_lifted()`
  every 20 ms, **first** in its loop, and latches `motion_estop()` (same latch as GP20).
  GP20 cannot release it while the car is still lifted. Latency (derived): lift recognised
  70-160 ms after it starts (sim), then within one VC poll (20-30 ms).
- **Back down:** level (< 10 deg) and still (averaged |dev| <= 60 mg) for 1 s, then the IMU
  returns to ACTIVE and hump detection resyncs. The motors stay stopped until GP20.
- **Limits (derived, to measure):** a vertical climb faster than about 200 mm/s on a hump
  would read as a lift; a lift slower than that is missed (demo: lift briskly, >= 10 cm in
  about 0.5 s; overturn still works). A car held level and very still in the hand for 1 s
  counts as put down (motors still stay stopped). The 1 s `[IMU] t` line shows `lv` (peak
  estimated upward speed) to tune the threshold on the car (S4-U38..U41).
- Impact detection (800 mg spike within 50 ms) was deleted. It also fired on ramp edges
  (run 40), so the gravity gate stays for tilt and nothing else depended on it.

Measurement approach
- Sample the accelerometer and magnetometer with one burst read each (6 bytes, set bit 7 of
  the register address). The driver is polled at 100 kHz, so a burst is about 0.8 ms
  (**derived**: 9 bytes x 9 bits at 100 kHz; measure it). Two bursts per period is about
  1.6 ms of P9 time; P5 to P8 preempt it, so measure P9 jitter.
- **Sample rate decision (open):** CTRL1 is 0x5F (50 Hz accelerometer), so a 10 ms task
  reads a new sample only every second period. Either set CTRL1 to **0x6F (100 Hz, verify
  against the datasheet table before use)** or run the task faster than the data. The
  LSM303D magnetometer has no 30 Hz setting (3.125 to 100 Hz steps); 50 Hz is configured.
- **Hump height** from pitch (gravity vector) and wheel-encoder distance:
  `h = sum(v * dt * sin(pitch))`, not double integration of acceleration (it drifts fast).
  It needs the **encoder distance or speed from Motion Control**, so that is an interface
  row in TEAM.md section 5 (proposal: Motion publishes distance as one aligned 32-bit word,
  like `line_state`; IMU only reads it). Report **relative** maximum hump height with a
  stated uncertainty, after calibration.
- Pitch comes from gravity alone (no gyro), so acceleration disturbs it. Check height
  accuracy under vibration and acceleration (assumption A6).
- **Hump vs lift:** a sustained pitch change over a distance is a hump; a sustained
  upward acceleration (estimated upward speed >= 200 mm/s) is a lift. The hump height is
  clamped to 8 bits (<= 255 mm assumed; **measure the tallest course hump** to check).
- **Turning:** heading from the magnetometer is disturbed by motor current and a hard-iron
  offset (measured about 1 gauss in total, above Earth's field). Calibrate the offset,
  and cross-check against the encoder left/right difference.
- **The LSM303D has a WHO_AM_I register (0x0F = 0x49)**: check it at start and set
  `IMU_FAULT` if it does not match. (The old note "no WHO_AM_I" was for the DLHC.)
- No FPU: use fixed-point. If any float maths remains, time its cost.
- Events and widths: 4-bit state, 3-bit event, 8-bit clamped hump height (section 6).
- Calibrate axes at start while the car is stationary (during `IR_CALIBRATION` or
  `READY_IDLE`). Mount near the centre of gravity, fixed and level, document the X/Y/Z
  orientation (NFR6); recalibrate after remounting.

Object invariants (section 12 habit; fill the TODOs with measured values)
| Object | Owner / writer | Readers | Size | Timeout / blocking | Rate |
|---|---|---|---|---|---|
| I2C1 | `imu_terrain` task only | none | n/a | polled; each transfer has a timeout, then `IMU_FAULT` | 2 bursts / 10 ms |
| Hump result word | IMU (writer) | Vehicle Controller, telemetry | one aligned 32-bit word | no lock, no blocking | on hump end |
| `Q_events` (IMU events) | IMU (producer) | Vehicle Controller | TODO packed header bytes | send with `TMO_POL`; overflow flag on full | on event, <= 1 per 10 ms |
| `IMU_FAULT` bit | IMU reports; Vehicle Controller latches in `g_fault_mask` | all | 1 bit | no blocking | on fault |
| IMU state word (`g_state`, LIFTED / OVERTURNED) | `imu_terrain` task (single writer) | Vehicle Controller via `imu_terrain_is_lifted()`, telemetry | one aligned 32-bit word | no lock, no blocking; VC polls every 20 ms | on change |
| Encoder distance word | Motion (writer) | IMU (reader) | one aligned 32-bit word | no lock | 10 ms |

Report (Week 6) lines that disagree with the hardware or other sections, to fix in the
second report version
1. **DLHC versus LSM303D:** Section 6.4 (purpose and explanation), the pin table rows
   ("GY-511 LSM303DLHC") and A7 still say DLHC. Our chip is the LSM303D at **0x1D**
   (section 3). Update them and cite `testing/subsystem4/logs/2026-10-23_imu_unit_run4.txt`
   and `_run5.txt`.
2. **Rates:** 6.4 says accelerometer 100 Hz and magnetometer 30 Hz; the code is at
   50 Hz / 50 Hz. Decide (above) and make the report, `drv_imu.h` and the task period agree.
3. **Mutex:** Section 8.3 mentions a single I2C1 mutex and a 20 ms IMU period. Everywhere
   else I2C1 has one owner task (no mutex) and 10 ms. State the single-owner rule (no
   mutex needed unless a second device or task shares I2C1) and fix 20 ms to 10 ms.
4. **Impact handling -> lift / overturn (prof, 13 Nov 2026):** remove sudden-impact /
   collision from Sections 5 and 6.4 and FR8. Replace with: the IMU detects lifted or
   overturned and reports it; the Vehicle Controller stops the motors at once (latched,
   `motion_estop()` today, `EVG_stop` later); lifting ends the demo run. State the
   lift-to-stop latency against NFR18: recognition 70-160 ms (derived, sim) + VC poll
   20-30 ms + Motion stop. Measure it on the car (S4-U38).
5. **State names:** 6.4 uses both `ACTIVE` and `MONITORING`; use one.
6. **Missing input:** the task table lists only accelerometer and magnetometer as IMU
   inputs. Hump height also needs encoder distance or speed from Motion Control; add it.
7. **A7 status:** the minimal I2C1 read works with our own driver (kernel `device/i2c`
   rejected, section 3). Record that as evidence, keep Open until the read is shown under
   the full task set.
8. **Heading:** A6 should name the hard-iron offset as a known risk (about 1 gauss
   measured at rest).
9. **State table:** add `LIFTED` and `OVERTURNED` to the IMU states (4 bits, values 6 and
   7) and `LIFTED` / `OVERTURNED` to the events (3 bits, values 3 and 5).

Before the Week 10 demo (Buddy 4 plan)
1. Re-run the I2C1 read inside the real `imu_terrain` task (P9, 10 ms) and time the burst
   and P9 loop (replaces the "minimal read" step already done in the unit tests).
2. Agree one packed `Q_events` header and the shared fault-bit map with Buddies 1, 2, 5.
3. Publish the hump result as one 32-bit word with a clamped height; set `IMU_FAULT` as one
   bit via a single writer or a kernel-atomic flag.
4. Record RAM (map file, stack high-water), `sizeof` of packed messages and P9 loop time
   before and after, labelled measured.
5. Keep P9 and 10 ms unless the measured response time says otherwise.

Tests: known-hump and controlled-tilt tests (replay a raw log to check the packed output);
IMU and motor faults injected close together with Buddy 2 (both bits preserved, critical
fault reaches the Vehicle Controller, stop meets the NFR18 deadline). Keep logs under
`testing/subsystem4/logs/` with date, setup, result.
