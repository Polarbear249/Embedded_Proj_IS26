# Subsystem 2 — test log

**Subsystem:** Motion control  ·  **Student ID:** <student ID>  ·  **Code:** `app_program/subsystem2/`

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S2-U01 | NULL status pointer rejected | Motion API | auto | `subsystem2_test.c` | `[TEST] S2-U01 PASS` |
| S2-U02 | negative speed rejected, no wheel moves | Motion API | auto | `subsystem2_test.c` | `[TEST] S2-U02 PASS` |
| S2-U03 | RETIRED 2026-11-04: checked that distance moves were not implemented | | | | |
| S2-U04 | bad distances rejected (negative, 0, NaN, over 1000 cm) | Motion API | auto | `subsystem2_test.c` | `[TEST] S2-U04 PASS` |
| S2-U05 | bad angles rejected (0, over 360, NaN) | Motion API | auto | `subsystem2_test.c` | `[TEST] S2-U05 PASS` |
| S2-U06 | line drive rejects bad speed and lease | Motion API | auto | `subsystem2_test.c` | `[TEST] S2-U06 PASS` |
| S2-U07, S2-U08 | `Q_motion` accepts a word; the task reads it and refuses a reserved op | IF-01 | auto | `subsystem2_test.c` | both PASS |
| S2-U09 to S2-U14 | `EVG_stop` latches, refuses a move, `EVG_STOPPED` set, releases, does not restart | IF-02 | auto | `subsystem2_test.c` | all PASS |
| S2-U15 | `motion_tick` worst-case time under 5 ms; also prints max exec and max period | TEAM.md section 4 | auto | `subsystem2_test.c` | PASS, and record the printed times |
| S2-H01 | encoder counting and IF-10 on a PC | IF-10 | auto (PC) | `testing/subsystem2/host/run_tests.sh` | `PASS` |
| S2-H02 | move law and `Q_motion` word on a PC, with a simple wheel model | IF-01 | auto (PC) | `testing/subsystem2/host/run_tests.sh` | `all passed` |
| S2-M01 | forward and backward 30 cm | Motion API | manual | `S2_MOVE_DEMO 1`, `S3_DRIVE_SWITCH 0`; ruler on the floor | within 2 cm; tune `MOVE_COAST_COUNTS` and duties |
| S2-M02 | left 90, right 90, U-turn | Motion API | manual | same image; mark the heading on the floor | within 10 deg; tune `MOVE_TURN_SCALE_PCT` |
| S2-M03 | wheel held during a move | Motion API, IF-05 | manual | same image, wheels raised; hold one wheel | FAULT within about 1 s, faults 0x1 in the demo line |
| S2-M04 | GP20 during a move | IF-02 | manual | same image; press GP20 mid-move | wheels stop, the next moves are refused |
| S2-M05 | line follow in PI speed mode | IF-06 | manual | default image, on the track, then the 10 mm ramp | keeps the line longer than duty mode; climbs the ramp |
| S2-M06 | no false stale stops | IF-06 | manual | default image; read `stale=` in the S3 drive line while following | `stale=` stays 0 |
| S2-M07 | battery power at `MOTOR_TEST_HIGH_POWER 0` | Motor limits | manual | battery, not USB | wheels start; decide keep or tune |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| YYYY-MM-DD | S2-U01 | `abc1234` | | PASS / FAIL | `logs/YYYY-MM-DD_….txt` | |
| 2026-11-04 | S2-H01 | merge of `425640b` into `5625b60`, uncommitted | Buddy 2 | PASS | `logs/2026-11-04_host_run1.txt` | PC only, built without the sanitizer (MinGW has no libubsan) |
| 2026-11-04 | S2-H02 | merge of `425640b` into `5625b60`, uncommitted | Buddy 2 | PASS | `logs/2026-11-04_host_run1.txt` | PC only, wheel model, not the car |
| 2026-11-04 | S2-U01, S2-U02, S2-U04 to S2-U14 | merge of `425640b` into `5625b60`, uncommitted | Buddy 2 | PASS | `logs/2026-11-04_target_s2_only_run1.txt` | On the Pico, wheels idle, S2 tests only |
| 2026-11-04 | S2-U15 | merge of `425640b` into `5625b60`, uncommitted | Buddy 2 | PASS, see open issue | `logs/2026-11-04_target_s2_only_run1.txt` | max exec 344 us; max period 142075 us |
| 2026-11-04 | S2-M01, S2-M02 (wheels raised only) | merge of `425640b` into `5625b60`, uncommitted | Buddy 2 | Not a floor result | `logs/2026-11-04_target_move_demo_raised_run1.txt` | All five moves ended with no fault, within 8 counts of the goal. Distance and heading on the floor are not measured. |

| 2026-11-04 | S2-M01, S2-M02 (wheel directions, wheels raised) | merge of `425640b` into `5625b60`, uncommitted | Buddy 2 | Not a floor result | Seen by the tester, no log | Forward: both wheels forward. Backward: both backward. Turn direction not clearly recorded (tester's note: "rigth fwd right backward"); encoder signs in the move demo log match the commands. Check turns on the floor. |
| 2026-11-04 | S2-M03 | merge of `425640b` into `5625b60`, uncommitted | Buddy 2 | PASS | `logs/2026-11-04_target_wheel_hold_run1.txt` | Right wheel held, wheels raised. Right stopped at 905 counts, then `faults 0x1`, state 2. Fault came about 1 s after the wheel stopped, estimated from the 1 s telemetry lines. All later moves refused (`accepted 1`). Left wheel not tested. |

| 2026-11-04 | S2-M04 | merge of `425640b` into `5625b60`, uncommitted | Buddy 2 | PASS | `logs/2026-11-04_target_gp20_run1.txt` | Wheels raised, move demo image. Pressed during forward: the move ended at 443/445 counts of about 1839, coasting to 457/452. Backward, left and right refused (`accepted 1`), counts unchanged. Still at 457/452 after release, then the U-turn ran as a new move. Time from press to stop not measured. Line-follow image not tested. |

| 2026-11-04 | S2-M01, S2-M02 (on the floor, not measured) | merge of `425640b` into `5625b60`, uncommitted | Buddy 2 | Not measured | Seen by the tester, no log | The car moves and turns on the floor. Distance and angle not measured, so the 2 cm and 10 deg limits are not checked. |
| 2026-11-04 | S2-M01, S2-M02 (on the floor, by eye) | merge of `425640b` into `5625b60`, uncommitted | Buddy 2 | Not measured | `logs/2026-11-04_target_move_demo_floor_run1.txt` | No faults. Tester: forward and backward fine, turns off by a little. Encoders: straight moves 2 to 13 counts short (at most about 2 mm), turns 9 to 17 counts short (about 2 to 3 deg). Actual distance and angle not measured. |
| 2026-11-04 | S2-M02 (on the floor, by eye, turn scale 97) | merge of `425640b` into `5625b60`, uncommitted | Buddy 2 | Not measured | `logs/2026-11-04_target_move_demo_floor_run2.txt` | No faults. Tester: U-turn about 10 deg short. 90 degree turns not reported. Next step: U-turn asks for 190 deg. |
| 2026-11-04 | S2-M01, S2-M02 (on the floor, by eye, U-turn 190) | merge of `425640b` into `5625b60`, uncommitted | Buddy 2 | Not measured | Seen by the tester, no log | Tester: straight move about 2 cm too far, turns about 2 deg short. Next step: `MOVE_DIST_SCALE_PCT` 93, `MOVE_TURN_SCALE_PCT` 99, U-turn 188. |
| 2026-11-04 | S2-M01, S2-M02 (on the floor, by eye, distance 93, turn 99, U-turn 188) | merge of `425640b` into `5625b60`, uncommitted | Buddy 2 | Not measured | Seen by the tester, no log | Tester: distance fine, car veers right, turns about 5 deg short. Next step: `MOVE_STRAIGHT_LEFT_PCT` 98, `MOVE_TURN_SCALE_PCT` 105, U-turn 182. |
| 2026-11-04 | S2-M01, S2-M02 (on the floor, by eye, distance 93, left share 98, turn 105, U-turn 182) | merge of `425640b` into `5625b60`, uncommitted | Buddy 2 | Looks good by eye, not measured | `logs/2026-11-04_target_move_demo_floor_run3.txt` | No faults. Tester: turns fine; forward veers about 2 cm, side not stated, and returns almost to its start. Left wheel's 98 percent share held: 1669/1695 counts. Odometer reads 274 mm on a move that looks like 30 cm. |
| 2026-11-04 | S2-M05 (first try) | merge of `425640b` into `5625b60`, uncommitted | Buddy 2 | Not passed, under investigation | Seen by the tester, no log | Earlier runs did not arm: `C=0 R=1`, centre sensor off the line. With the centre on the line: tester saw a jerk at the start, then forward, then a veer left after a while. Cause of the veer not known. Ramp not reported. `stale=` not read, no log pasted. Next step: `MOTION_START_PWM` 350 to 250 for the jerk, and a log of the veer. |
| 2026-11-04 | S2-M05, S2-M06 (kick 250) | merge of `425640b` into `5625b60`, uncommitted | Buddy 2 | Not passed, under investigation | `logs/2026-11-04_target_line_follow_run1.txt` | About 9 s of driving, 508 mm. No fault; `stale=0`, `lease=0` for this short run. Wheels 12 to 17 RPM against 20 commanded until about 6 s. Right steer shown from about 2 s, but the left wheel only pulled ahead after about 7 s, then reached 28 RPM. Left guess (`err=48`) seen about 1 s in, with `R=1` 1 s later. Next step: `MOTION_KP` and `MOTION_KI` 2 to 6. |
| 2026-11-04 | S2-M05, S2-M06 (kick 250, probably gains 6) | merge of `425640b` into `5625b60`, uncommitted | Buddy 2 | Not passed yet, promising | `logs/2026-11-04_target_line_follow_run2.txt` | About 9 s of driving, 567 mm, on USB. No fault; `stale=0`, `lease=0` for this short run. Tester: went over the bump, start not too jerky, "in a sense" stays on the line after the hump. Bump height not recorded. Wheels near 19 RPM of 20 by about 3 s. IMU flagged a hump at 349 mm, still in hump state when the log ended. Junction reported from about 41 to 56 cm, with left about 26 RPM and right about 17 in that stretch; cause not known. Duty mode comparison not run. Image not confirmed by the tester; the higher speed suggests gains 6. Log has a gap from seq 3 to 13. |
| 2026-11-05 | S2-M01, S2-M02 (distance 93, left share 98) | `a461608` + demo switches | Buddy 2 | Not passed | `logs/2026-11-05_target_move_demo_floor_run1.txt` | Two runs on a flat floor, first with the real S5 code. Tester: forward about 28 cm, at the 2 cm limit, and about 5 cm left veer. Odometer 275 mm. In this log the left 90 stalled at L -195, R 408 (`faults 0x1`), so the right turn and U-turn were refused. Ruler was beside the wheel path and the USB cable was loose; a snag is suspected, not confirmed. |
| 2026-11-05 | S2-M01, S2-M02 (distance 100, left share 99) | `a461608` + demo switches | Buddy 2 | Not measured | `logs/2026-11-05_target_move_demo_floor_run2.txt` | Ruler moved aside, USB cable held up. Tester: veer about 0.5 cm left; the settings and the cable changed together, so the fix is not separated. All 5 moves ended with no fault. Odometer 295 mm. Forward distance not reported. |
| 2026-11-05 | S2-M01 | `a461608` + demo switches, 20 s pause | Buddy 2 | PASS | `logs/2026-11-05_target_move_demo_floor_run3.txt` | Distance 100, left share 99. Forward about 29 cm: ruler read 14 cm with its 15 cm mark on the 30 cm target line. Backward recorded as the same reading, deviating 0.5 cm left. Odometer 296 mm, within about 2 % of the ruler. One measured run. |
| 2026-11-05 | S2-M02 (distance 100, left share 99) | `a461608` + demo switches, 20 s pause | Buddy 2 | Looks fine by eye, not measured | `logs/2026-11-05_target_move_demo_floor_run3.txt` | No faults. Encoders: left 90 L -558, R 558; right 90 back to L -2, R 0; U-turn L -1146, R 1136. Angles not measured. IMU heading is not usable as a check: about 98 deg for the same left turn in run 2, about 86 deg here. |

S2-M01 passed on 2026-11-05 with one measured run. S2-M02 still needs measuring. S2-M05 and S2-M06 are in progress. S2-M07 has not been run yet.

**Open issue:** the IMU reported humps on the flat floor in all three runs on 2026-11-05: 15, 23 and 35 mm. Pitch swings about 7 to 12 deg while driving and returns to about 0 when stopped. Cause not known; raised with Buddy 4.

**Open issue:** max period 142075 us is about 14 times the 10 ms tick. S2-U15 checks only max exec, so it still passes. The figure is a peak since boot, so the run does not show when it happened or whether it repeats. Cause not known yet. A clue from the move demo run: the IMU task on core 0 also saw a 132609 us period in its first window after boot, then stayed under 14 ms, including during the moves. That points to a start-up stall, but the motion tick itself has not been measured after start-up. The wheel hold and GP20 runs saw the same thing: 132305 us and 132508 us in the first IMU window, then under 15 ms. In the GP20 run the sensor was not answering in that window (loop time at most 335 us), so IMU reads did not cause the gap.

## Week 13 evidence: memory

Measured with `arm-none-eabi-size`, Arm GCC 15.2, Pico SDK 2.3.1, `make -j8`, S5 as the empty stub.
"Before" is the merged integration code without this S2 work.

| | Before | After | Change |
|---|---|---|---|
| RAM (`.data` + `.bss`), bytes | 25680 | 25792 | +112 |
| Flash (`.text` + `.data`), bytes | 301732 | 306056 | +4324 |

| Object | Kind | Size | Notes |
|---|---|---|---|
| `Q_motion` message | 32-bit word | 4 bytes | op 3, left 11, right 11, seq 4, 3 unused bits (`motion_cmd.h`) |
| `Q_motion` buffer | kernel message buffer | 32 bytes | depth 4, each message plus its 4-byte length header; from the kernel heap, not in the RAM row |
| `EVG_stop` | kernel event flag | 2 bits used | `EVG_STOP_REQ`, `EVG_STOPPED` |

P-task timing, wheels idle, WiFi down, no load on the motors: `motion_tick` max exec 344 us, max period 142075 us against a 10 ms tick (see the open issue under Results). Not yet measured while driving.
