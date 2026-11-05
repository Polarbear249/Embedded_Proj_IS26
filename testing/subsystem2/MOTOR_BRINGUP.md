# Buddy 2: motor and motion bring-up

How to build, flash and first drive the Motion subsystem. Results are in
`TEST_LOG.md`; this file does not record any.

## Wiring

Use the Robo Pico onboard motor driver and a suitable motor power supply.
Left motor: GP10 forward, GP11 reverse. Right: GP9 forward, GP8 reverse.
GPIOs 8-11 are control signals for the onboard driver, not direct motor power.
Encoders: left GP4 (A), GP5 (B); right GP0 (A), GP1 (B). Never build with
`CRASH_REPORT=1` on the car, because it takes GP0 and GP1.

## Build and flash

Follow the repository README toolchain and SDK setup. From `build_make`:

```sh
make -j8          # normal image
make TEST=1 -j8   # test image; APP_TEST_ONLY 2 in app_main.c runs only S2
```

Both must build with 0 warnings. `APP_TEST_ONLY` is a local switch, so keep it
at 0 in commits.

Move and turn try-out: set `S2_MOVE_DEMO` to 1 (`subsystem2.h`) and
`S3_DRIVE_SWITCH` to 0 (`subsystem3.h`). The build stops with an error if
`S3_DRIVE_SWITCH` is still on, because two tasks would command the motors. The
car drives by itself after boot, so raise the wheels first and keep
`S2_MOVE_DEMO` at 0 in commits.

## Which call to use

Include `subsystem2/motion_control.h` from a file in `app_program`, or
`../subsystem2/motion_control.h` from another subsystem. Only one task should
command the motors at a time.

| Need | Call | Notes |
|---|---|---|
| Line following | `motion_line_drive(left_rpm, right_rpm, lease_ms)` | The chosen API. PI speed mode, -30 to +30 RPM, renewed by the caller. Stops when the lease runs out or `line_state` is stale or FAILED |
| Fixed wheel speeds | `motion_set_speed(left_rpm, right_rpm)` | Forward only, no lease: the caller must send the stop |
| Distance or turn | `motion_move_forward`, `motion_move_backward`, `motion_turn_left`, `motion_turn_right`, `motion_u_turn` | Returns at once. Poll `motion_get_status()`: RUNNING, then STOPPED, or FAULT if a wheel stalled |
| From the VC | `motion_send(word)` on `Q_motion` (IF-01) | Word layout in `motion_cmd.h` |
| Open-loop fallback | `motion_set_duty(left, right, lease_ms)` | -200 to +200 (1000 = 100 %). No encoder stall watchdog. It stalled on the 10 mm ramp, so don't use it for line following |

## First wheel test

With the wheels raised and `S3_DRIVE_SWITCH` at 0, send one command from a
normal application task, after all subsystem initialization and start calls:

```c
ER er = motion_set_duty(150, 150, 500);
CHECK(er, "first wheel test");
```

This requests 15 % forward duty for at most 500 ms. Do not assume duty equals RPM.

| Command | Left | Right |
|---|---:|---:|
| Forward | 150 | 150 |
| Reverse | -150 | -150 |
| Spin left | -150 | 150 |
| Spin right | 150 | -150 |
| Left wheel only | 150 | 0 |
| Right wheel only | 0 | 150 |

## Rules for callers

- Commands are latest-writer-wins, so buddies must not send competing motion
  commands. `motion_estop()` and `motion_request_stop()` may come from any task.
- `motion_stop()` cancels the command; later commands can move again.
- `motion_estop()` latches a stop and refuses new movement.
  `motion_clear_estop()` releases the latch and any FAULT, and does not restart
  anything.
- Check every return value. `motion_set_duty()` returning `E_TMOUT` means the
  command lock was busy: retry next cycle. A failed stop call has not been
  accepted.
- Stops coast (all driver inputs low), so stopping distance is not guaranteed.
- A lease handles lost commands while the motion task is running. It is not a
  hardware watchdog and cannot handle a frozen kernel.

## Power limit

`MOTOR_TEST_HIGH_POWER` is 1 (`motor.h`): 50 % maximum and 25 % start kick,
because the wheels did not start on laptop USB power at 20 % and 15 %. Every
passing car run used 1, so it stays until S2-M07 is run on battery at 0.

## Validation

PC tests (with MinGW, set `S2_HOST_SAN=` because it has no sanitizer):

```sh
sh testing/subsystem2/host/run_tests.sh
```

These cover encoder counting and IF-10 (`test_encoder_distance.c`), and the move
law and `Q_motion` word against a simple wheel model (`test_motion_move.c`). They
do not replace the on-target tests.

On the car: S2-U01 to S2-U15 and S2-M01 to S2-M07 in `TEST_LOG.md`. Record actual
results rather than marking unrun checks as passed.

## Remaining Buddy 2 work

`TEST_LOG.md` holds the current status of each test. Still open:

- Measure distance and turns with a tape measure (S2-M01, S2-M02). The current
  scale values were tuned by eye.
- Pass line following and the ramp (S2-M05), with no stale stops over a full lap
  (S2-M06).
- Run battery power at `MOTOR_TEST_HIGH_POWER` 0 (S2-M07).
- Find the cause of the 142 ms maximum `motion_tick` period (S2-U15 open issue).
