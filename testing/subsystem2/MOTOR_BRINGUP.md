# Buddy 2: first integrated wheel movement

This change combines Buddy-2 hardware tests with main and adds a micro T-Kernel
open-loop motor service. It does not import the separate feature branch's polling
encoder or PI implementation. No motor moves automatically at startup.

## Wiring assumed from Buddy-2 tests

Use the Robo Pico onboard motor driver and a suitable motor power supply.
Left motor: GP10 forward, GP11 reverse. Right: GP9 forward, GP8 reverse.
GPIOs 8–11 are control signals for the onboard driver, not direct motor power.
Check physical left/right and forward with the wheels raised before floor tests.
The direction mapping is an assumption based on committed tests, not a new
hardware measurement. This implementation does not configure the encoders.

## Build and flash

Follow the repository README toolchain/SDK setup. From `build_make`, run:

```sh
make -j8
```

Use the generated UF2, with USB console and `CRASH_REPORT=0`. Keep that console
choice when encoders on GP0/1 are added later. The build automatically discovers
C files under `app_program/subsystem2`.

## Send a first command

Include `subsystem2/motion_control.h` from a file in `app_program`, or
`../subsystem2/motion_control.h` from another subsystem. Call from a normal
application task, after all subsystem initialization and start calls complete.
Do not call from an ISR. Raise the wheels, then send one command:

```c
ER er = motion_set_duty(150, 150, 500);
CHECK(er, "first wheel test");
```

This requests 15% forward duty for at most 500 ms, then the service removes
motor drive. A loaded motor may not start at this duty: test raised first.
The current limit is 20%; do not assume duty equals RPM.

| Command | Left | Right |
|---|---:|---:|
| Forward | 150 | 150 |
| Reverse | -150 | -150 |
| Spin left | -150 | 150 |
| Spin right | 150 | -150 |
| Left wheel only | 150 | 0 |
| Right wheel only | 0 | 150 |

For continuous movement, renew a 300 ms lease every 100 ms from a single vehicle
supervisor. Commands use latest-writer-wins semantics, so buddies must not send
competing normal motion commands independently. Line following and obstacle
avoidance should feed the supervisor. Emergency stop may be requested by any task.

- `motion_stop()` cancels the command; subsequent commands can move again.
- `motion_estop()` latches a stop and rejects new movement.
- `motion_clear_estop()` clears the latch and leaves movement at zero.
- Every API return must be checked. `E_TMOUT` means mutex contention: retry
  after yielding. A failed stop call has not been accepted.
- The owning task runs with a 10 ms delay. Accepted stops apply on a subsequent
  tick, subject to scheduling; this is not a hard 10 ms or 150 ms guarantee.
- Stops coast (both driver inputs low); stopping distance is not guaranteed.
- A command lease handles lost commands while the task is running. It is not
  an independent hardware watchdog and cannot handle a frozen kernel.

## Validation

```sh
sh testing/subsystem2/host/run_tests.sh
```

Host tests cover production motor/control code against fake registers and kernel
calls: startup stop, input rejection, direction mapping, reverse pause, stop,
expiry, latched estop, time wraparound, and mutex contention. These tests passed.
Native syntax checking against the real project headers also passed. The Arm
compiler and Pico SDK were unavailable in the authoring environment, so a full
firmware link and physical wheel test have NOT been completed.

Next bench checks: each wheel alone, both forward, both reverse, turn directions,
lease expiry, emergency stop, and stop latency under the integrated workload.
Record actual results rather than marking unrun checks as passed.

## Remaining Buddy 2 work

Interrupt-based quadrature encoder decoding (shared GPIO ISR coordination),
counts-per-revolution calibration, measured RPM/distance, PI speed control,
encoder/stall fault handling, and distance/angle maneuvers remain pending.
Other buddies can use the duty/stop API now; do not present it as calibrated
speed, distance, or angle control. Old S3/S4 demo sample resources remain for
source compatibility, but no fake sample producer is started.
