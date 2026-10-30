# Team and subsystems

Team IS26, INF2004 Autonomous Robotic Car Challenge ("PicoCar").

The tables below record the team's decisions from the **Week 6 design
document** and **Tribunal 1 (Week 8, 19 Oct 2026)**. Rows marked **CONFIRM**
are still open (see section 7). The template's example code (LED, producer,
consumer, monitor) is still in `app_program/` until each owner replaces it.

Fill this in at the start of the project and keep it up to date. It is the
single place that says **who owns what**, so that five people can work in
one firmware image without clashing over pins, priorities, or kernel objects.

A **subsystem** is one student's part of the product, with its own folder.
It may contain several tasks and own several peripherals (see section 3).

Rule: **only the owner edits a subsystem's folders.** To change another
subsystem, ask its owner or open a pull request for them to review.

## 1. Members and subsystems

| # | Subsystem (what it does) | Student ID | GitHub username | Code | Tests |
|---|---|---|---|---|---|
| 1 | WiFi communication, MQTT command and telemetry | | | `app_program/subsystem1/` | `testing/subsystem1/` |
| 2 | Motion control: motors, encoders, PID speed, encoder turns | | | `app_program/subsystem2/` | `testing/subsystem2/` |
| 3 | IR line following and barcode decoding | | | `app_program/subsystem3/` | `testing/subsystem3/` |
| 4 | IMU motion and terrain monitoring (GY-511, LSM303D) | | | `app_program/subsystem4/` | `testing/subsystem4/` |
| 5 | Servo ultrasonic scanning and obstacle profiling | | | `app_program/subsystem5/` | `testing/subsystem5/` |
| - | Vehicle Controller: mission state and coordination, no hardware. **CONFIRM** writer: Tribunal 1 lists Buddy 2 with all buddies; Buddy 4 is writing it | | | `app_program/vehicle_controller.*` | `testing/integration/` |

Shared files (`app_program/app_main.c`, `app_program/app.h`, this file)
are changed only by agreement of the team. The exception is claiming an empty
row in this file (a pin, a peripheral, an interface ID), which anyone may do
(section 2).

## 2. Pins

One owner per pin. Claim a pin here **before** you wire or code it.

**Why:** two subsystems using the same pin is one of the most common and
hardest-to-find mistakes in a team project. The code builds without any
error, but the hardware misbehaves. Each subsystem can pass its own tests
and then fail once they are combined, and two outputs driving the same pin
against each other can damage it. Writing the claim down first means a clash
is spotted when someone edits this table, not hours later on the bench.

**Approval before wiring.** The table below is the Week 6 pin table, agreed
at Tribunal 1. The course requires the pin layout to be **approved before any
wire is connected**, and any pin change goes through the same approval and is
then recorded here. Inside the team, claim a free row and agree clashes
between yourselves; only the *reserved* and *board* rows are fixed.

| Pin | Owner | Used for |
|---|---|---|
| GP23, GP24, GP25, GP29 | *reserved* | Pico W radio |
| GP0 | S2 | Right wheel encoder A (Grove 1). **Never build with `CRASH_REPORT=1`** on the car: it takes GP0/GP1 for the crash UART |
| GP1 | S2 | Right wheel encoder B (Grove 1). See GP0 |
| GP2 | S4 | I2C1 SDA, GY-511 IMU (Grove 2 / Maker port) |
| GP3 | S4 | I2C1 SCL, GY-511 IMU (Grove 2 / Maker port) |
| GP4 | S2 | Left wheel encoder A (Grove 3) |
| GP5 | S2 | Left wheel encoder B (Grove 3) |
| GP6 | S3 | Left IR DOUT (Grove 5) |
| GP7 | S3 | Right IR DOUT, barcode (Grove 7; AOUT unused) |
| GP8 | S2 | Motor 1 input A (M1A, fixed by Robo Pico) |
| GP9 | S2 | Motor 1 input B (M1B, fixed by Robo Pico) |
| GP10 | S2 | Motor 2 input A (M2A, fixed by Robo Pico) |
| GP11 | S2 | Motor 2 input B (M2B, fixed by Robo Pico) |
| GP12 | S5 | Scan servo signal (servo port 1) |
| GP13 | | spare servo-port pin; shares PWM slice 6 with GP12, do not use for the LED |
| GP14 | | spare servo-port pin; **CONFIRM** liveness LED here or on GP15 |
| GP15 | | spare servo-port pin |
| GP16 | S5 | HC-SR04+ TRIG (Grove 4) |
| GP17 | S5 | HC-SR04+ ECHO, direct 3.3 V (Grove 4) |
| GP18 | *board* | NeoPixel RGB LED: reserved |
| GP19 | S3 | Centre IR DOUT (GPIO breakout). The template's example LED (`S1_LED_PIN`) still uses GP19 and must move |
| GP20 | *board* | Programmable button: reserved |
| GP21 | *board* | Programmable button: reserved |
| GP22 | *board* | Piezo buzzer: reserved |
| GP26 | S3 | Left IR AOUT (ADC0) |
| GP27 | S3 | Centre IR AOUT (ADC1) |
| GP28 | *board* | VBAT sense or spare ADC2: keep free |

Notes
- GY-511: power it from the **3V3 pin only**; never connect VIN and 3V3
  together. INT1, INT2 and DRDY are not wired; the IMU is polled.
- HC-SR04+ is the 3.3 V variant; never feed 5 V into a GPIO.

## 3. Peripherals

**Subsystems, tasks and peripherals are three different things:**

| Term | What it is | Example |
|---|---|---|
| **Subsystem** | one student's part of the product, one folder | "environment sensing", "motor control", "user interface" |
| **Task** | one thread of execution. A subsystem may have **as many tasks as it needs** | a sampling task and a filtering task, both in "environment sensing" |
| **Peripheral** | one hardware block on the RP2040 | I2C0, UART1, the ADC |

A driver is **not** a subsystem. It is code *inside* the subsystem that
needs that hardware: the "environment sensing" subsystem contains the I2C
sensor code, and owns I2C0. Write it as `drv_<name>.c` / `drv_<name>.h` in
that subsystem's folder, never under `device/`, and call it only from the
peripheral's owning task. The kernel's own drivers in `device/` (serial,
I2C, ADC) may be used instead; switching one on in `config/config_device.h`
changes shared configuration, so record it in the table below first.

**The rule: each peripheral is driven by exactly one task, its *owning
task*.** That task can be in any subsystem, and one subsystem may own
several peripherals (one owning task each, or one task owning all of them).
Every other task, in the same subsystem or another one, asks the owning task
through a kernel object (section 5) and never touches that peripheral itself.

*Why:* this port runs tasks on both cores at the same time. If two tasks
drive the same I2C or UART block at once, their transfers get mixed up, and
the drivers have no locking to stop it.

| Peripheral | Owner subsystem | Owning task | Pins |
|---|---|---|---|
| UART0 | *unused* | (`CRASH_REPORT=1` would take it; not allowed on the car, GP0/GP1 are encoders) | |
| UART1 | | | |
| I2C0 | | | |
| I2C1 (own polled driver `subsystem4/drv_i2c1.c`; kernel I2C driver stays off) | S4 | IMU / Terrain task (tests: initial task) | GP2 SDA, GP3 SCL |
| SPI0 | | | |
| SPI1 | | | |
| ADC | S3 | Line / Barcode task | GP26 (ADC0), GP27 (ADC1); GP28 unused |
| PWM slice 4 (motor 1) | S2 | Motion Control task | GP8, GP9 |
| PWM slice 5 (motor 2) | S2 | Motion Control task | GP10, GP11 |
| PWM slice 6 (servo) | S5 | Obstacle Scan task | GP12 (GP13 shares the slice) |
| GPIO edge interrupts (encoders, ECHO) | **CONFIRM** owner | one shared GPIO interrupt dispatcher, minimal ISR work (store raw counts/timestamps), no interrupt disabling | GP0, GP1, GP4, GP5, GP17 |
| WiFi radio, TCP/IP, MQTT | *kernel* | `cyw43_utk_task` (built in) | GP23–25, GP29 |

### WiFi and MQTT are special

The radio and the network stack belong to the kernel's built-in WiFi service
task, `cyw43_utk_task` (`lib/libwifi/sysdepend/pico_rp2040/`). **No student
task may call `cyw43_...`, lwIP or MQTT functions, not even from a
"WiFi subsystem".** The calls are not safe from any other task, on either core.

What students *can* do:

- **Read the connection state** from any task, which is always safe:
  ```c
  #include "cyw43_utk.h"
  T_CYW43_UTK_STATUS st;
  cyw43_utk_get_status(&st);   /* st.ready, st.link_status, st.ip_addr, ... */
  ```
- **Send or receive data over the network** by adding code that *runs
  inside the service task*. The MQTT client is in
  `lib/liblwip/sysdepend/pico_rp2040/mqtt_utk.c`. It publishes a heartbeat
  today. A "cloud / reporting" subsystem would add a kernel object
  (e.g. a message buffer) that application tasks fill. The MQTT code then
  reads it with `TMO_POL` (never blocking) and publishes it.
  Because this changes shared library code, **agree it with the team and
  the instructor first**, and record it as an interface in section 5.
  It needs the `NET=lwip MQTT=1` build (README section 3).

## 4. Task priorities

1 is the highest. 1–4 are used by the kernel (USB console 3, WiFi 4). The
team's task table (Week 6 Section 3, agreed at Tribunal 1) **replaces the
template's default bands 12–21**. Change it only by team agreement, and
update the `S*_PRI_*` macros in each `subsystemN.h` to match (the example
code still uses 12–21). Write down every task you create.

| Task | Subsystem | Priority | Period | Core |
|---|---|---|---|---|
| Motion Control | 2 | 5 (code still 14: `motion_control_task`, 100 ms; and 12: `encoder_sampling_task`, 1 ms polling) | 10 ms | core 1 (`TP_PRC2`) |
| Line / Barcode | 3 | 6 | 10 ms | |
| Vehicle Controller | shared | 7 | event-driven + 20 ms supervisory tick | |
| Obstacle Scan | 5 | 8 | >= 60 ms between ultrasonic triggers | |
| IMU / Terrain (`imu_terrain`) | 4 | 9 | 10 ms (cyclic handler + event flag) | core 0 (`TP_PRC1`) |
| WiFi / MQTT | 1 | 10 | 200 ms telemetry, 1 s heartbeat | |
| Liveness LED | **CONFIRM** owner | | | |

Core: `TP_PRC1` = core 0, `TP_PRC2` = core 1, or "any". Cores are not yet
decided. The 10 ms tasks use absolute wake-up times or a cyclic handler, not
`tk_dly_tsk`.

## 5. Interfaces between subsystems

Every arrow between two subsystems is listed here. The **provider** creates
the kernel object and publishes its ID in its own `subsystemN.h`; the **user** only
sends to it or waits on it. Agree on the data format before coding.

"VC" is the Vehicle Controller. "Kernel type TBD" means the report names the
object but the µT-Kernel object type is not yet chosen.

| ID | From → To | Mechanism | Object ID name | Data / meaning | Agreed by |
|---|---|---|---|---|---|
| IF-01 | VC → S2 | queue (kernel type TBD, owned by S2) | `Q_motion` | packed movement commands | Tribunal 1 |
| IF-02 | VC → S2 | event flag (owned by S2) | `EVG_stop` | safety stop, checked every control cycle; outranks `Q_motion` | Tribunal 1 |
| IF-03 | S3, S4, S5, S1 → VC | queue (kernel type TBD, owned by VC) | `Q_events` | packed event header, bounded depth, overflow flag; header layout **CONFIRM** | Tribunal 1 |
| IF-04 | S3, S4, S5, S1 → VC | event flag (owned by VC) | `EVG_sys` | event notification with `tk_set_flg`; wait/clear options **CONFIRM** | Tribunal 1 |
| IF-05 | all → VC | shared word, atomic access | `g_fault_mask` | latched fault bits: motor stall, encoder fault, IMU fault, no echo, WiFi loss. Owner **CONFIRM**: VC (Version C) or S2 | Tribunal 1 |
| IF-06 | S3 → S2 | aligned 32-bit word, single writer, no mutex | `line_state` | line error, validity flags, 8-bit `seq`; stale data → safe stop | Tribunal 1 |
| IF-07 | VC → S1 | message buffer, by-value copy (owned by S1) | `mbf_telem` (`Q_telem`) | 13-byte packed snapshot, frame v1, layout in `app_program/subsystem1/telemetry_frame.h`; VC calls `s1_telem_send()` | Tribunal 1 |
| IF-08 | S1 → VC | message buffer (created by S1 until the VC exists; then move to VC) | `mbf_cmd` (`Q_cmd`) | validated 3-byte command frame (`CMD_STOP`, `CMD_START`, `CMD_SET_SPEED`, `CMD_PING`); VC calls `s1_cmd_recv()` | Tribunal 1 (buffer type and temporary owner: S1) |
| IF-09 | S5 → VC | queue (kernel type TBD, owned by VC) | `Q_profile` | packed `ObstacleProfile_t`: three 9-bit distances in 32 bits | Tribunal 1 |
| IF-10 | S2 → S4 | aligned 32-bit word, single writer (S2 encoder task), no lock; read with `motion_get_distance_mm()` (`subsystem2/motion_control.h`) | `distance_mm` (static in `subsystem2/encoder.c`) | `int32_t` cumulative distance since power-up in mm, mean of both wheels, forward positive, never reset; published every 10 encoder samples (≥ 10 ms). Host test `testing/subsystem2/host/`; on target INT-10 | **PROPOSED** (implemented by Buddy 4 in Buddy 2's folder on `integrate/s4-onto-main`; Buddy 2 to confirm) |
| IF-11 | S4 → VC, S1 | aligned 32-bit word, single writer, no lock | **TBD** | hump result, 8-bit height clamped (mm), written on hump end | Tribunal 1 |

MQTT topics (broker on the laptop): `mtk3smp-pico/telem` (binary frame out, 5 Hz),
`mtk3smp-pico/cmd` (binary command in), `mtk3smp-pico/heartbeat` (text, 1 s).
S1 edits two shared library files to carry them: `lib/liblwip/sysdepend/pico_rp2040/mqtt_utk.c/.h`
(weak hooks `mqtt_app_tx` and `mqtt_app_rx`, 1 s heartbeat, reconnect guard) and
`lib/libwifi/sysdepend/pico_rp2040/cyw43_utk.c` (re-join retry made level-triggered).

Safety path: S5 confirms an obstacle and reports it to VC; VC sets
`EVG_stop`; S2 stops the motors and the encoders confirm stationary;
scanning or bypass planning starts only after that. S2 also stops the
motors if VC goes silent (command timeout).

Each interface is tested by at least one integration test
(`testing/integration/TEST_LOG.md`).

## 6. Kernel object budget

Limits are set in `config/config.h`. If the team needs more, raise the
limit there. The numbers below are the **template example's** usage, still
in the code; replace them as each owner creates the objects in section 5.

| Object | Limit | S1 | S2 | S3 | S4 | S5 |
|---|---|---|---|---|---|---|
| Task | 32 | 2 | 1 | 1 | 1 | |
| Semaphore | 16 | | 1 | | | |
| Event flag | 16 | | | | 1 | |
| Mailbox | 12 | | | 1 | | |
| Mutex | 12 | | | 1 | | |
| Message buffer | 12 | 2 | | | 1 | |
| Fixed memory pool | 12 | | 1 | | | |
| Variable memory pool | 12 | | | | | |
| Cyclic handler | 12 | 1 | | | | |
| Alarm handler | 12 | | | | 1 | |

(The kernel's USB and WiFi service tasks also use a few of these.)

## 7. Open points

1. Who writes the Vehicle Controller (Buddy 4 is; Tribunal 1 says Buddy 2
   with all) and who owns `g_fault_mask` (VC or Motion).
2. Liveness LED pin (GP14 or GP15) and its owner; move the example LED off
   GP19.
3. Owner of the shared GPIO interrupt dispatcher (encoders and ECHO).
4. Kernel object type for each `Q_*` queue, the `Q_events` header layout,
   and the `EVG_sys` wait/clear options.
5. Encoder distance word from Motion to IMU (IF-10).
6. Core assignment for each task.
7. MQTT code inside `cyw43_utk_task` needs team and instructor agreement.
   S1's changes are listed under section 5 and are ready for review in the PR.
8. Known limit: after a WiFi drop the link re-joins by itself in about 35 s,
   but MQTT reconnects only about 58 s after the link is back, because the
   MQTT keepalive is 60 s. Lowering it trades recovery time against false
   disconnects on short WiFi stalls.
9. The forced WiFi reconnect with motion and line tasks running (design
   review A1) needs the Motion task; run it at integration.

## Buddy 2 motor foundation

S2 now runs `motion_task`, priority 14 on core 1, with a 10 ms delay
between updates. Only that task writes PWM after initialization.
`motion_control.h` provides signed duty commands with a maximum 1000 ms
lease, stop, latched emergency stop, and status. One mutex protects commands
across cores. Calls are non-blocking: callers must check and retry E_TMOUT.
The existing sample pool and semaphore remain for S3/S4 source compatibility,
but S2 no longer generates fake samples. Those example interfaces are obsolete.
GP0/1 and GP4/5 are not configured by this motor-only implementation.
Encoder feedback, PI tuning and distance/angle commands are pending.
