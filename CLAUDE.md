# CLAUDE.md

Project context and working rules for AI coding assistants (Claude Code, Codex and
similar) in this repo. Read it fully before changing anything. Copy it to `AGENTS.md`
for tools that read that name.

Project: **INF2004 Embedded Systems, Autonomous Robotic Car Challenge ("PicoCar")**, team IS26.
Repo: `github.com/Polarbear249/Embedded_Proj_IS26` (started from the course template
`github.com/sirfonzie/INF2004_ProjectTemplate`).

Sources used for this file: Project Write-up, Project Briefing, Barcode/Line
Specification, PicoCar project context and slides, Robo Pico datasheet, HC-SR04,
IR sensor (ST188/LM393, TCRT5000), GY-511 datasheets (vendor lists LSM303DLHC; **our
board has an LSM303D**, see section 3), Barr-C 2018, the template's README and TEAM.md,
our **Week 6 design document**, **Tribunal 1 (Week 8, 19 Oct 2026)**, **Change
Requirement 1**, the prof's Week 4 lecture hints (`whathesaid.pdf`, section 12), and the
**prof's barcode and demo clarification (31 Oct 2026, section 1b)**.
Where the briefing and Tribunal 1 differ, the **Tribunal 1 team decisions win** for our
design. **Section 1b is the newest instruction and wins over any older barcode, mapping
or sensor-side text in this file or in the Week 6 report.** Where the Week 6 report and
a hardware measurement differ, the **measurement wins** (section 8 lists the report
lines to correct). Items marked **CONFIRM** are unclear
or conflicting. Nothing below has been measured on hardware unless it says "measured";
"estimate" and "derived" are not measurements.

---

## 1. Mission

A car that runs a course **autonomously, without remote control**, on a Raspberry Pi
Pico W mounted on a Cytron Robo Pico board. Course elements: a line-following track,
directional barcodes at selected locations, speed humps of varying heights, and static
obstacles along the track.

Mandatory implementation constraints (Project Write-up):
- **µT-Kernel (micro T-Kernel) RTOS** on the Pico W + Robo Pico platform.
- **C language** and the **Pico C SDK**. (The template builds with GNU make and pulls
  TinyUSB and the CYW43 driver from the Pico SDK; keep using the template's build.)
- Follow the **template GitHub repo** structure.
- Conform to the **Barr C Coding Standard** (Barr-C:2018, see section 9).
- Resource-efficient design: prudent use of processing, memory, communication and power.

The car shall (10 mission requirements):
1. Follow a line track autonomously.
2. Detect and decode barcodes (**A to Z, readable in either direction**) along the track.
3. Execute the movement mapped to the decoded character at the next cross-junction:
   Forward (go straight), U-Turn, Left, Right. The character-to-movement mapping is
   **received over MQTT at run time, never hard-coded** (section 1b).
4. Detect humps along the route.
5. Measure and report the highest hump peak during the run.
6. Detect obstacles on or near the line.
7. Profile obstacle shape and position using ultrasonic sensing.
8. Navigate around obstacles without collision.
9. Reacquire the original line after bypassing an obstacle.
10. Report telemetry through WiFi communication.

Expected behaviour (Write-up):
- **Line following**: track the black line with IR sensors; robust to minor lighting
  changes, slight track imperfections and speed variations.
- **Barcode navigation**: decode, store the character, then perform the mapped movement
  at the next cross-junction (full flow in section 1b). The write-up's example mapping
  (A = Left, B = Right, C = Go Straight, D = U-Turn) is **only an example**: never
  compile it in as a default (it may be a unit-test fixture only). Turns must be
  executed accurately with **encoder-based motion control**.
- **Hump detection**: detect humps and estimate hump height using **IMU data**.
- **Obstacle avoidance**: (1) stop or slow, (2) ultrasonic system profiles the obstacle,
  (3) choose a safe bypass route, (4) leave the line temporarily, (5) navigate around,
  (6) search for and reacquire the original line, (7) continue normal operation,
  (8) keep deviation from the original route small.

---

## 1b. Barcode and demo rules (prof's clarification, 31 Oct 2026)

Newest instruction; it overrides older barcode, mapping and sensor-side text anywhere
else in this file, the Week 6 report and Tribunal 1. Items marked PROPOSED are our
design choices, not the prof's words, and need team agreement.

Prof's requirements
1. **Decode all 26 characters, A to Z.** The PDF examples cover only three characters;
   the other barcode link gives the codes for A to Z. Put the full code table in one
   `const` table in `subsystem3/` and cite its source in the file header.
   **CONFIRM** where the link and its table are saved in the repo (do not invent codes).
2. **Bidirectional.** The same barcode must decode to the **same character** whether the
   car crosses it left to right or right to left. The decoder must recognise the
   direction itself and normalise it before the table lookup.
   **CONFIRM** that the symbology has a start/stop pattern (or another way to tell the
   direction) and that no character's reversed pattern equals another character's
   pattern. If any do, report it to the prof; do not guess.
3. **Barcode on the left side of the track, hump on the right** (prof: "Yes, and the
   hump on the right"). This replaces the old "right side" wording (section 3 track
   spec, section 2 Buddy 3 roles). See the sensor-role conflict in section 4.
4. **Four movements**: forward (go straight), U-turn, left, right.
5. **The character-to-movement mapping is not hard-coded.** Just before the demo the
   instructor says which four characters map to the four movements; the mapping is
   **sent from the PC to the car over MQTT** and the car applies it at run time.

Demo flow the car must perform, repeatedly
1. Follow the line; scan a barcode in either direction.
2. Decode the correct character and **store it in memory**.
3. Keep moving until a **cross-junction** is detected.
4. At the cross-junction, perform the movement mapped to the stored character.
5. Continue, and repeat for the next barcode.

Ownership (PROPOSED, extends Tribunal 1; supersedes "Buddy 3 outputs a 2-bit command")
| Step | Owner | Notes |
|---|---|---|
| Read and decode barcode, normalise direction, output a **character 0 to 25** | Buddy 3 | Reports to the Vehicle Controller through `Q_events` (5-bit field). Decoder does **not** map to a movement |
| Detect cross-junction | Buddy 3 | Report as a junction event. **CONFIRM** it distinguishes a cross-junction from other black-on-both-sensors cases (Week 6 treated both line sensors on black as a junction) |
| Receive, validate and forward the mapping | Buddy 1 | MQTT in, `Q_cmd` to the Vehicle Controller. Binary payload (section 6). Add the mapping to the FR16 command list (START, STOP, STATUS_REQUEST); **topic name and frame layout TBD, CONFIRM with Buddy 1** |
| Hold the mapping table and the stored character; look up the movement at the junction | Vehicle Controller | Single owner of both, so no cross-task race. Sends `Q_motion` |
| Execute forward / U-turn / left / right | Buddy 2 | Encoder-based, unchanged ownership |
| Report decoded character and applied mapping in telemetry | Buddy 1, via `Q_telem` | Lets the PC confirm the car received the mapping |

Behaviour to decide with the team (PROPOSED defaults, all **CONFIRM**)
- **Stored character slot:** one pending slot; a new barcode before the junction
  overwrites it; cleared after the movement is executed. Alternative: a small queue.
- **Character with no mapping** at a cross-junction: go straight and raise a telemetry
  flag; do not stop the car.
- **Mapping not yet received** (or WiFi down at start): do not guess a mapping. Apply
  forward and flag it. NFR12 still holds: WiFi loss never stops navigation.
- **Mapping update mid-run:** the latest complete mapping replaces the old one as one
  whole-word store; never apply a half-received mapping. Re-sent on reconnect or
  retained by the broker so a reboot can recover it (**CONFIRM** with Buddy 1).
- **Four mapped characters must be distinct;** reject a mapping that repeats one.
- **Hump on the right:** record only. Hump detection (pitch plus encoder distance) is
  unchanged; **CONFIRM** with the prof or the track whether the hump sits to one side of
  the line, because then roll as well as pitch may appear (Buddy 4, section 8).

Tests to add (`testing/subsystem3/`, `testing/integration/TEST_LOG.md`)
- All 26 characters decode correctly, **in both directions**, on the real track.
- Mapping applied at the junction: for each of the four movements, send a new mapping
  over MQTT and check the car performs it, with no reflash.
- No mapping, unmapped character, repeated character and mid-run update cases.
- Round-trip test of the packed character and mapping word (section 6).

---

## 2. Team and subsystems

Five "buddies", one subsystem each, plus a shared **Vehicle Controller** (mission state
and coordination). The final robot is assessed as an **integrated system**, not as five
separate parts. All buddies take part in integration, system testing, debugging, the
final demonstration, and must understand every subsystem interface.

| Buddy | Folder | Subsystem | Primary hardware |
|---|---|---|---|
| 1 | `app_program/subsystem1/` | WiFi communication, command and telemetry | WiFi (CYW43439) |
| 2 | `app_program/subsystem2/` | Motion control | Motors + wheel encoders |
| 3 | `app_program/subsystem3/` | Barcode decoding and IR line following | 3x IR sensors |
| 4 | `app_program/subsystem4/` | IMU motion and terrain monitoring | IMU (GY-511) |
| 5 | `app_program/subsystem5/` | Adaptive servo ultrasonic scanning and obstacle profiling | HC-SR04 + servo |
| - | `app_program/vehicle_controller.*` | Mission state and coordination. Tribunal 1 lists Buddy 2 with all buddies as provider; **Buddy 4 is writing it** (CONFIRM with Buddy 2) | none (software only) |

Only the owner edits a subsystem's folder. Shared files (`app.h`, `app_main.c`,
`TEAM.md`) change only by team agreement.

### Buddy 1: WiFi, MQTT, telemetry
Tasks: WiFi connectivity; MQTT; communication topics; telemetry message structures;
telemetry publishing; command subscription; connection recovery; heartbeat/status.
Telemetry examples: robot state, motor speed, encoder counts, line sensor status,
barcode detected, hump measurements, obstacle scan results, distance travelled.
**New (section 1b):** receive the instructor's character-to-movement mapping from the PC
over MQTT, validate it (four distinct characters A to Z, complete frame) and forward it
to the Vehicle Controller via `Q_cmd`; report the decoded character and the applied
mapping in telemetry. Also provide a small PC-side script that publishes a mapping for
testing.
Deliverables: communication API, telemetry framework, MQTT documentation, telemetry demo.

### Buddy 2: Motion control
Tasks: motor driver and encoder integration; speed and distance estimation; **PID speed
control**; straight-line correction; encoder-based turning; motion calibration.
Required APIs (examples): `moveForward(distance)`, `moveBackward(distance)`,
`turnLeft(angle)`, `turnRight(angle)`, `stop()`. (Rename to Barr-C style, e.g.
`motion_move_forward()`.) Objectives: consistent speed, accurate distance, repeatable
turns, stable motion.
Deliverables: motion control library, PID tuning report, motion accuracy evaluation.

### Buddy 3: Line following and barcode
Tasks: IR integration and calibration; line position estimation; line-following
algorithm; junction detection; barcode detection and decoding; navigation command
generation. **Barcode (section 1b):** decode **A to Z**, **in either direction**, and
output the **character** (0 to 25), not a movement; the Vehicle Controller maps it to
one of four movements (forward, U-turn, left, right) using the MQTT-supplied mapping.
Detect the **cross-junction** that triggers the stored movement.
**Sensor roles are in conflict:** the Week 6 design has left IR and centre IR following
the line and the **right IR reading barcodes**, but the prof says the barcode is on the
**left** of the track. **CONFIRM** with Buddy 3 which IR reads the barcode and how the
sensors are re-mounted; any pin or role change needs pin-layout approval (section 4,
section 12).
Deliverables: line-following module, barcode decoder (A to Z, bidirectional), junction
event interface, character output interface.

### Buddy 4: IMU and terrain
Tasks: IMU integration and calibration; tilt detection; hump detection; **peak hump
measurement**; motion classification; **lift / overturn detection**; turn-rate
monitoring; motion event reporting.
**Changed by the prof (13 Nov 2026):** collision detection with the IMU is **removed**.
Instead the IMU detects when the car is **lifted or overturned**, and then the motors
must **stop immediately**. In the demo, **lifting the car signals the end of the run**.
Implemented in `subsystem4/imu_lift.c` (IMU reports LIFTED / OVERTURNED; the Vehicle
Controller latches `motion_estop()`, so the IMU still never drives the motors).
Example motion events: stationary, accelerating, turning, climbing hump, descending
hump, lifted, overturned.
Objectives: reliable hump detection, accurate peak measurement, stable classification.
Deliverables: IMU processing module, hump detection algorithm, terrain analysis report.
Briefing slide outputs: shock alerts, bump height (cm), direction (yaw/pitch/roll),
terrain profile, telemetry data. The briefing suggests `h ~= v^2 / (2 g)` from a vertical
acceleration impulse, but the team chose pitch plus encoder distance instead (section 8).
**Hardware caveat:** the briefing slide shows a gyroscope, but our GY-511
(LSM303D, see section 3) has **accelerometer + magnetometer only, no gyro**. Turn rate and heading
must come from the magnetometer and/or encoders.

### Buddy 5: Ultrasonic scan and obstacle profiling
Stages: (1) **coarse scan** with few angles (example 30, 60, 90, 120, 150 degrees);
(2) **fine scan** back at the obstacle region at higher resolution; (3) **profile**
location, width, closest point, left and right clearance; (4) **avoidance plan**: stop,
continue, turn left, turn right, reverse and reattempt; (5) **line recovery**: search for
the original line, reacquire line tracking, return control to line following.
Width estimate uses centre distance, left/right distances and scan angle:
`W = f(dL, dC, dR, theta)`.
Deliverables: scanning subsystem, obstacle profile generator, avoidance algorithm,
recovery algorithm.
Week 10 demonstration metrics (grades A to F):

| Test | A | B | C | D | F |
|---|---|---|---|---|---|
| Obstacle detection rate | 100% | >=95% | >=90% | >=80% | <80% |
| Width estimation error | <=10% | <=20% | <=30% | <=40% | >40% |
| Direction selection success | 100% | >=95% | >=90% | >=80% | <80% |
| Successful line reacquisition | 100% | >=95% | >=90% | >=80% | <80% |
| Time to bypass obstacle | <3 s | <5 s | <7 s | <10 s | >10 s |

### Vehicle Controller (shared)
**CONFIRM the writer** (Tribunal 1 lists Buddy 2 with all buddies as provider; Buddy 4 is
writing it). Owns mission state and coordination. Gathers inputs from the subsystems, decides what
the car does, sends commands out, and never touches hardware itself.
**Barcode navigation (section 1b):** it holds the runtime character-to-movement mapping
and the stored (pending) character. When the cross-junction event arrives it looks up the
movement for the stored character and sends it to Motion through `Q_motion`. The mapping
is empty until MQTT supplies it; never compile a default mapping in.

| Direction | Data | From / to |
|---|---|---|
| In | line position, cross-junction event, decoded barcode **character** (0 to 25) | Buddy 3 |
| In | character-to-movement mapping, START/STOP/STATUS_REQUEST | Buddy 1 (`Q_cmd`) |
| In | obstacle detected, profile, avoidance decision | Buddy 5 |
| In | hump, lifted / overturned (stop), tilt and motion events | Buddy 4 |
| In | speed, distance, turn completion | Buddy 2 |
| Out | target speed, move/turn commands | Buddy 2 |
| Out | scan requests, servo control | Buddy 5 |
| Out | robot state, events, measurements | Buddy 1 (telemetry) |

System architecture (Write-up): **Sensor layer** (IR, encoders, IMU, ultrasonic,
barcode sensor), **Control layer** (motion control, line following, obstacle avoidance,
navigation logic), **Communication layer** (WiFi, MQTT, telemetry).

---

## 3. Hardware

Raspberry Pi **Pico W** (RP2040, two Cortex-M0+ cores, **no FPU**, 2 MB flash) in the
Cytron **Robo Pico** socket. Parts: two geared DC motors with quadrature encoders, one
servo, HC-SR04+ ultrasonic (team runs it at **3.3 V**), three IR reflective sensor
modules (ST188 + LM393 comparator; each has DOUT and AOUT), GY-511 IMU, external power
source (power bank in the slides).

### Robo Pico electrical limits (datasheet Rev 1.0)
- Power input (USB, LiPo or VIN): **3.6 to 6 V**. Motor and servo voltage equals the
  power input voltage (about VUSB - 0.4 V when powered from USB).
- Motor driver: **1 A continuous, 1.5 A peak (<5 s) per channel**, PWM up to 20 kHz.
- Total +3V3 from the Grove ports: **300 mA max**. Buck-boost for the Pico: 800 mA.
- Digital inputs: low <= 0.8 V, high 2.0 to **3.6 V max**. **Never feed 5 V into a GPIO**
  (the HC-SR04 ECHO output is 5 V on the plain HC-SR04; ours is the 3.3 V "+" variant).
- Analog input: 0 to 3.3 V.

### Motor driver truth table (per channel, inputs on GPIO)
| Input A (GP8 / GP10) | Input B (GP9 / GP11) | Result |
|---|---|---|
| Low | Low | Brake |
| High | Low | Forward (depends on wiring) |
| Low | High | Backward (depends on wiring) |
| High | High | Coast (Hi-Z) |

Swap MxA/MxB wiring to reverse a motor. Use PWM on the A/B inputs for speed.

### Motor + encoder connections (6 wires each)
Encoder GND, encoder A, encoder B, encoder VCC (3.3 V), motor +, motor -.
Motor +/- go to the Robo Pico motor terminals. Encoder A/B go to any free Pico GPIO.

### Servo
3 wires: power, ground, PWM signal. Servo ports on the Robo Pico carry GP12 to GP15
(4 ports); V+ equals the supply voltage.

### HC-SR04 (datasheet)
Range 2 cm to 400 cm, resolution ~3 mm, measuring angle ~15 degrees. Trigger: >= 10 us
high pulse. Module sends eight 40 kHz pulses; ECHO high time is the round trip.
`distance_cm = echo_us / 58` (sound speed 340 m/s, divide by 2). Use a **>= 60 ms
measurement cycle**. Target should be >= 0.5 m^2 and smooth for good readings.

### IR reflective sensor module (Waveshare manual, ST188 + LM393)
Pins: DOUT (digital), AOUT (analog), GND, VCC (3.0 to 5.3 V). Use **one** of DOUT or AOUT
per sensor. DOUT works on any GPIO; AOUT needs an ADC pin (GP26 to GP28, ADC0 to ADC2).
Indicator LED lights when a reflective surface is close. TCRT5000 datasheet: peak
operating distance 2.5 mm, usable range 0.2 to 15 mm, 950 nm emitter. Calibrate
thresholds on the real track, in the real lighting.

### GY-511 IMU (chip on our board: ST LSM303D, I2C)
**Measured 2026-10-23:** our GY-511 carries an **LSM303D**, not the LSM303DLHC the
vendor listing names. A bus scan finds only **0x1D**, and WHO_AM_I
(0x0F) reads **0x49**. Nothing answers at the LSM303DLHC addresses 0x19/0x1E. Evidence:
`testing/subsystem4/logs/2026-10-23_imu_unit_run4.txt` and `_run5.txt`.
- **One 7-bit address for both sensors: 0x1D** (SA0 pulled high on the module; 0x1E if
  SA0 were low). Multi-byte reads: set bit 7 of the register address (auto-increment).
- Accelerometer +/-2, 4, 6, 8, 16 g; magnetometer +/-2, 4, 8, 12 gauss; 16-bit output;
  temperature sensor; FIFO; two programmable interrupt generators. **No gyroscope.**
- Chip supply 2.16 to 3.6 V (module sold as 3 to 5 V). **Power it from 3V3.**
- Key registers: WHO_AM_I 0x0F (= 0x49), OUT_X_L_M 0x08 (mag X/Y/Z, low byte first),
  CTRL0 0x1F, CTRL1 0x20 (acc rate, BDU, axis enable), CTRL2 0x21 (acc full scale),
  CTRL5 0x24 (mag resolution/rate, temp enable), CTRL6 0x25 (mag full scale), CTRL7 0x26
  (mag mode), OUT_X_L_A 0x28 (acc X/Y/Z, low byte first), IG_CFG1 0x30, IG_SRC1 0x31,
  IG_THS1 0x32, IG_DUR1 0x33.
- Sensitivity: acc +/-2 g = 0.061 mg/LSB; mag +/-2 gauss = 0.080 mgauss/LSB.
- Settings in use (`app_program/subsystem4/drv_imu.h`): CTRL1 0x5F (acc 50 Hz, BDU, XYZ),
  CTRL2 0x00 (+/-2 g), CTRL5 0x70 (mag high res, 50 Hz), CTRL6 0x00 (+/-2 gauss),
  CTRL7 0x00 (mag continuous).
- Driver: `subsystem4/drv_i2c1.c` (own polled I2C1 master, 100 kHz) +
  `subsystem4/drv_imu.c`. The kernel's `device/i2c` driver is **not** used: on this port
  it sends no STOP after multi-byte reads and disables the block before the last written
  byte has gone out. `DEVCNF_USE_IIC` stays 0.
- Measured at rest (run 5): acc noise about +/-10 mg; |a| about 990 mg. Magnetometer total
  about 1 gauss, above Earth's field (0.25 to 0.65 gauss): a hard-iron offset from the car
  (motor magnets, steel) that must be calibrated out before using heading.
- Pins on the module: VIN, 3.3V, GND, SCL, SDA, INT2, INT1, DRDY.

### Track specification (Barcode and Line Specification)
- Line width: **18 mm** (black line).
- Barcode: about **141 mm** long, placed **20 mm** from the line (gap between barcode and
  line). **Side: LEFT of the track** (prof, 31 Oct 2026; the spec sheet's earlier "right
  side" is superseded), so the IR sensor that reads it must be on the left. The **hump is
  on the right**.
- Characters **A to Z**, readable **in both directions** (section 1b). The spec PDFs
  show three example characters; the other barcode link has A to Z.
- Bar widths vary. **CONFIRM** the symbology and narrow/wide ratio with Buddy 3 before
  writing the decoder; it is not stated in the spec sheet. Measure bar widths in
  encoder distance, not time, so decoding does not depend on speed.

---

## 4. Pin mapping

Pins GP23, GP24, GP25, GP29 are the Pico W radio: **never use**. The Pico W on-board LED
is on the radio chip (not a GPIO). Usable pins: GP0 to GP22, GP26 to GP28. GP0/GP1 are
taken whenever the team builds with `CRASH_REPORT=1`. **Claim every pin in `TEAM.md`
section 2 before wiring or coding it.**

### Fixed by the Robo Pico board (do not reuse)
| Pin | Fixed function |
|---|---|
| GP8, GP9 | Motor 1 inputs A, B (M1A, M1B) |
| GP10, GP11 | Motor 2 inputs A, B (M2A, M2B) |
| GP12, GP13, GP14, GP15 | Servo ports 1 to 4 signal |
| GP18 | NeoPixel RGB LED (WS2812) |
| GP20, GP21 | Programmable buttons |
| GP22 | Piezo buzzer (mute switch on board) |
| GP28 | Optional VBAT sense via ADC2 if the back jumper is soldered (VADC2 = VBAT / 2) |

### Peripheral options on the Pico (from the pinout)
- I2C0: GP0/1, GP4/5, GP8/9, GP12/13, GP16/17, GP20/21. I2C1: GP2/3, GP6/7, GP10/11,
  GP14/15, GP18/19, GP26/27.
- ADC: GP26 (ADC0), GP27 (ADC1), GP28 (ADC2).
- Grove 2 and the Maker (Qwiic) port share GP2 (SDA1) and GP3 (SCL1).

### Team pin map (Week 6 design document Section 2, agreed at Tribunal 1)
The agreed pin table is TEAM.md section 2 (source of truth; every pin change goes there).

GP2/GP3 are valid I2C1 pins (SDA1/SCL1), so the IMU wiring in the report is consistent
with the pinout. The Week 6 pin table is the source; **record it in TEAM.md section 2**.

Conflicts and cautions
- **GP0/GP1 versus `CRASH_REPORT=1`:** the template reserves GP0/GP1 for the crash UART.
  The right encoder is on GP0/GP1, so **never build with `CRASH_REPORT=1`** on the car, and
  tell Buddy 2. Note it in TEAM.md.
- **GP19:** the template's example LED sits on GP19, now the centre IR DOUT. Remove the
  example. The liveness LED needs another free pin (suggest GP14 or GP15; CONFIRM and
  claim it in TEAM.md; do not use GP13, it shares a PWM slice with the servo).
- ADC is one shared block with a **single owner task** (Line/Barcode). Only GP26 and GP27
  are used.
- **Barcode side conflict (section 1b):** the Week 6 table has the **right** IR (GP7,
  DOUT only, Grove 7) reading barcodes and the left IR (GP6 DOUT, GP26 AOUT) plus centre
  IR (GP19 DOUT, GP27 AOUT) following the line. The barcode is now on the **left**. Do
  not change a pin or swap sensor roles on your own: Buddy 3 proposes the new roles
  (for example, which sensor needs an AOUT), the updated pin table goes for approval
  **before any wire is moved**, then into TEAM.md section 2.
- Encoder A/B and the HC-SR04+ ECHO use GPIO edge interrupts. The team decided on **one
  shared GPIO interrupt dispatcher** with minimal ISR work (store raw timestamps or
  counts, process in tasks) and **no interrupt disabling** around it. Check how the
  template's `libbsp` exposes GPIO interrupts before coding either. The IMU does **not**
  use a GPIO interrupt (INT1/INT2/DRDY are not wired; it is polled).
- The IMU supply: use **only the 3V3 pin** of the GY-511; never connect VIN and 3V3 at the
  same time. Verify the module's I2C pull-ups with a meter (assumption A7).
- Pin-layout approval: see section 12 (get approval before wiring, also for any change).
- The pptx slides show wiring diagrams only as images that could not be read; compare
  them with this table.

---

## 5. Software platform

**µT-Kernel 3.0** is compiled into the same image as our code. Dual-core SMP port for the
RP2040, default `SMP=1`. Our code lives in `app_program/` and runs as tasks on top of
the kernel.

### Build and flash
Linux or WSL (Ubuntu). Needs `make`, host `g++`, `arm-none-eabi-gcc` and
`PICO_SDK_PATH` pointing at Pico SDK 2.2.0 with submodules `lib/tinyusb` and
`lib/cyw43-driver` (also `lib/lwip` for networking).

```
cd build_make
make -j8                 # defaults: SMP=1 CONSOLE=usb_cdc WIFI=cyw43 TEST=0 CRASH_REPORT=0
make TEST=1 -j8          # also runs every subsystemN_test.c at power-up
make SMP=0 -j8           # single-core build
make CRASH_REPORT=1 -j8  # copy console to UART0 on GP0/GP1 for crash reports
make NET=lwip MQTT=1 ... # TCP/IP + MQTT over WiFi (needs lib/lwip and config/wifi_secrets.h)
```
Output: `build_make/mtk3pico_smp1_usb_cdc_wifi.uf2` (the name encodes the options). Options
are **not remembered**: pass them on every `make`. Flash by holding BOOTSEL while
plugging in USB, then copy the `.uf2` to the `RPI-RP2` drive. Console is USB serial at
115200 baud. Kernel limits (max tasks and objects) are in `config/config.h`.
Run `make -j8` **and** `make TEST=1 -j8` before claiming a change works. Do not claim
hardware behaviour that has not been run on the board.

### Repo layout
Our code is in `app_program/` (one `subsystemN/` folder per buddy, plus `app_main.c`, `app.h`, `vehicle_controller.*`); tests and logs in `testing/`. Template and kernel folders: see section 11.

Module files named in the design review: `wifi_telemetry.c` (S1), `motion_control.c` (S2),
`line_barcode.c` (S3), `imu_terrain.c` (S4), `obstacle_scan.c` (S5), `vehicle_controller.c`,
plus `drv_<name>.c/.h` for drivers. Use these names (they are what the report cites).

### Rules from TEAM.md (keep them)
- **One owning task per peripheral** (I2C, UART, SPI, ADC, PWM slice). Other tasks ask the
  owner through a kernel object. Drivers have no locking, and tasks run on both cores at
  once, so two tasks on one block corrupt transfers.
- Drivers go in the subsystem folder as `drv_<name>.c/.h`, never under `device/`. The
  kernel's own I2C/ADC/serial drivers in `device/` may be used, but enabling one in
  `config/config_device.h` is a shared change: record it in TEAM.md first.
- **WiFi/lwIP/MQTT**: no application task may call `cyw43_*`, lwIP or MQTT functions, not
  even from a "WiFi subsystem". Only the built-in `cyw43_utk_task` may. Any task may read
  link state with `cyw43_utk_get_status()` (`#include "cyw43_utk.h"`). To send data, add a
  kernel object (e.g. a message buffer) that app tasks fill and let the MQTT code inside
  the service task read it with `TMO_POL` (never blocking). The MQTT client is
  `lib/liblwip/sysdepend/pico_rp2040/mqtt_utk.c`. Changing it is shared library code:
  **agree with the team and instructor first** and record it as an interface.
- Every arrow between subsystems is a row in TEAM.md section 5. The receiver/owner of the
  data creates the kernel object and publishes its ID in its own `subsystemN.h`.
- Never commit: `config/wifi_secrets.h` (and any `wifi_credentials.h`, `network_config.h`,
  `udp_test_config.h`, `tcp_test_config.h`), or build outputs (`.o`, `.elf`, `.uf2`).

### Priorities and kernel budget
Priority 1 is the highest. The kernel uses 1 to 4 (USB console 3, WiFi service 4). The
team's task table (Week 6 Section 3, Tribunal 1) **replaces the template's default bands
(12 to 21)**; update the `S*_PRI_*` macros and TEAM.md section 4 to match:

The agreed task table (task, priority P5..P10, period, owner, core) is TEAM.md section 4.

P5 preempts P6. The WiFi task stays lowest because the network stack already runs in
kernel service context. Use absolute wake-up times (or a cyclic handler) for the 10 ms
tasks. Limits: 32 tasks, 16 semaphores, 16 event flags, 12 each of mailboxes, mutexes,
message buffers, fixed pools, variable pools, cyclic and alarm handlers (kernel tasks use
some).

### RTOS and RP2040 gotchas
- `TASK_STACK_SIZE` is 2048 bytes; `tm_printf` needs about 1 KB. Blocking kernel calls
  use extra stack under SMP. Raise it if a task crashes oddly.
- `tm_printf` is atomic per call, not per line. Build a line with `tm_sprintf`, then print
  once. It is diagnostic output, not real-time safe.
- `tk_dly_tsk` measures from when it is called, so periods drift by the work time. For a
  steady period (IMU 10 ms, control loops) use a cyclic handler or an absolute deadline.
- Handlers (cyclic, alarm, interrupt) are short and never wait. Set an event flag or
  signal a semaphore and let a task do the work.
- Mailbox messages start with a `T_MSG` header and must stay valid until received: take
  them from a fixed memory pool, not the stack.
- Check kernel call results with `CHECK(er, "name")` from `app.h`.
- **No FPU**: floating point is slow software emulation. Use fixed-point integers in
  fast paths (IMU filtering, PID).
- Publish shared data before signalling another core; `volatile` alone is not
  inter-core synchronisation. Use kernel objects.
- Serial, I2C, ADC, DMA and PWM are single-owner resources on this port; only the
  `tm_printf` USB path is qualified for concurrent cross-core use.
- Do not add flash erase/program code (unsafe with the second core running).
- Keep a console-independent liveness LED task (external LED on a free pin, not GP19 which
  is now the centre IR input): if console output stops but the LED still blinks, the
  console is at fault; if the LED freezes, the kernel is wedged.

---

## 5b. Architecture agreed in Tribunal 1

Principles
- **Single owner per peripheral.** Only Motion Control drives the motors. The Vehicle
  Controller authorises movement and touches **no hardware**. IMU reports events and
  never drives motors. Obstacle Scan controls the servo and sensor only (no independent
  movement). WiFi never controls motors; commands go to the Vehicle Controller.
- **EVG_stop outranks Q_motion** (dedicated safety path).
- WiFi loss must not interrupt autonomous navigation (NFR12); reconnection with
  exponential backoff stays inside the P10 task. Publish summarised state, not raw data.

Shared objects (names used in the report)
| Object | Direction | Meaning and rules |
|---|---|---|
| `Q_motion` | Vehicle Controller to Motion | movement commands |
| `EVG_stop` | Vehicle Controller to Motion | safety stop; checked every control cycle |
| `Q_events` | Line, Obstacle, IMU, WiFi to Vehicle Controller | packed event header, bounded depth, overflow flag |
| `EVG_sys` | producers to Vehicle Controller | event notifications. Use **`tk_set_flg`** (the report also says `tk_set_evt`; the template code uses `tk_cre_flg`/`tk_set_flg`/`tk_wai_flg`, so verify against the headers) and set the wait/clear options explicitly |
| `g_fault_mask` | shared | persistent latched fault bits (motor stall, encoder fault, IMU fault, no echo, WiFi loss). Version C: **Vehicle Controller owns updates**, others report faults via events; clear only through authorised recovery. **CONFIRM:** the Buddy 2 row says Motion is sole writer of the latched mask |
| `line_state` | Line (writer) to Motion (reader) | one packed aligned 32-bit word (line error, validity flags, 8-bit `seq`). Single atomic store and load, no mutex. Stale data triggers a safe stop |
| `Q_telem` | Vehicle Controller to WiFi | **by-value** copy of the 13-byte packed snapshot (no shared struct, no mutex) |
| `Q_cmd` | WiFi to Vehicle Controller | validated commands, including the **character-to-movement mapping** (section 1b). The Vehicle Controller is the single owner of the applied mapping, so no shared mapping object is needed. Frame layout TBD (**CONFIRM** with Buddy 1) |
| `Q_profile` | Obstacle to Vehicle Controller | packed `ObstacleProfile_t` |
| ECHO timestamps | ISR to Obstacle task | raw rise/fall as aligned `uint32_t`; the task needs a **consistent rise/fall pair** before computing width |

Safety and stop path (Version C wording supersedes earlier text where Buddy 5 requests a
stop through `Q_motion`): Obstacle confirms an obstacle and reports it to the Vehicle
Controller; the Vehicle Controller sets `EVG_stop`; Motion Control stops the motors and
the encoders confirm stationary; **scanning or bypass planning starts only after the car
is stationary**. Motion also applies a command timeout (motors stop if the Vehicle
Controller goes silent) and a slew limit to protect the driver's 1 A rating.

Referenced requirement IDs (definitions are in the Week 6 design document; do not invent
values): FR7/FR8 hump detection and height, FR10 stopping clearance (3 cm, not yet
tested), FR18 sensor calibration, NFR1 (>= 9/10 trials), NFR2 (>= 95% over >= 20 trials),
NFR3 drift, NFR12 WiFi loss, NFR18 stop-response deadline.

Open assumptions: A1 WiFi/MQTT does not block motion and sensing; A2 encoders give valid
3.3 V quadrature A/B; A3 power supply and Robo Pico current budget (Buddy 2); A4 left and
centre analog IR give a usable line error and right DOUT supports barcode (**A4 needs
rewording: the barcode is now on the left, section 1b**); A6 IMU data is
stable enough for hump, tilt, turn and lift / overturn; A8 HC-SR04+ ECHO is safe at 3.3 V. All
**Open** at Tribunal 1.

Integration risks (Tribunal 1 register): R1 packing or interrupt-disable adds jitter to P5
and to ECHO/encoder timestamps; R2 false fault bits from unverified encoder/ECHO levels;
R3 producer and consumer disagree on packed layout; R4 lost update on the fault mask or a
torn `line_state` read; R6 simultaneous fault updates overwrite each other.

Cross-subsystem tests planned (all Open): emergency-stop latency (Obstacle + Motion);
stale/torn `line_state` (Line + Motion); simultaneous ECHO and encoder timing after
packing (logic analyzer); NFR1/NFR2 trials before and after packing; IMU and motor faults
injected close together (Buddy 4 + Buddy 2).

Clock: **125 MHz is a provisional assumption**; confirm the real system clock before
converting cycles to time or estimating worst-case execution time (WCET).

---

## 6. Data representation requirement (changed requirement, Oct 2026)

Each application data field uses **only the bits its valid range needs** (C bit-fields or
explicit masks and shifts). Examples from the brief: a value 1 to 5 uses 3 bits; a state
machine with up to 8 states uses 3 bits; fault conditions (motor stall, encoder fault, IMU
fault, no echo, WiFi loss) are **individual bits in one shared fault bitmask**, not
separate integers. It applies to motion and encoder data, line/barcode sensing, obstacle
detection, hump estimation, vehicle and controller states, data shared between tasks or
between an ISR and a task, inter-task queue messages, and MQTT payloads (compact binary,
not text). Shared packed data must be accessed safely: read-modify-write on shared packed
data must not race, so use atomic access or proper synchronisation.

Team decisions
| Area | Decision |
|---|---|
| Telemetry (Buddy 1) | JSON becomes a compact binary frame with a version/length byte and a decoder script. 13-byte packed snapshot, **explicitly serialised into a `uint8_t` buffer** with masks, shifts and defined byte order (do not rely on compiler bit-field layout or padding). JSON example 37 B vs binary is derived, not yet measured. Obstacle telemetry: 4-byte binary frame |
| Motion (Buddy 2) | 7 motion states in 3 bits; review encoder count widths without losing accuracy; pack `Q_motion` fields; measure P5 WCET before and after |
| Line / barcode (Buddy 3) | `line_state` in one 32-bit word with 8-bit `seq`; 3-bit state; ring-buffer head and tail stay separate words. **Updated by section 1b:** the decoder outputs a **5-bit character (0 to 25)**, not the earlier 2-bit command; the 2-bit field is now the **movement** (forward, U-turn, left, right) chosen by the Vehicle Controller |
| Barcode mapping (Buddy 1 and Vehicle Controller) | **PROPOSED:** the four mapped characters are packed in one `uint32_t`: 4 fields x 5 bits = 20 bits, one field per movement (forward, U-turn, left, right), plus a mapping-valid bit (21 bits used). Smaller than a 26-entry table of 2-bit movements (52 bits). Written once as a whole word by the Vehicle Controller after validation; read only by the Vehicle Controller. The MQTT mapping frame is compact binary with the version/length byte (20 bits of payload, so 3 bytes of mapping data), explicitly serialised with masks and shifts. Sizes are **derived**, not measured. Add the character, movement and mapping fields to the data representation table (Week 13) |
| IMU (Buddy 4) | 4-bit state, 3-bit event, hump height 8 bits (assumed <= 255 mm), **clamp, do not wrap**; publish the hump result as one aligned word; raw IMU samples and filter state keep native widths; pack locally before `Q_events`; `IMU_FAULT` is one bit set by a single writer or kernel-atomic flag |
| Obstacle (Buddy 5) | distances are **9-bit unsigned (0 to 511 cm)** covering the 0 to 400 cm range (Version C replaced the 8-bit buffer that overflowed above 255 cm); three distances in one 32-bit `ObstacleProfile_t` (27 bits); fine-scan samples must be **genuinely bit-packed** (an ordinary C array has no 9-bit elements); define handling for invalid and out-of-range readings |
| Unchanged | pin map, ADC channels, task priorities and periods (the requirement changes format, not rates) |

Rules for packed data
- One shared header defines every packed layout, with compile-time size checks
  (`#if`/static assert on `sizeof`) and a **round-trip unit test** (risk R3).
- Prefer single-writer words and whole-word atomic store/load. `volatile` alone is not
  synchronisation. Use the port's atomic primitives (it builds atomics from the RP2040
  hardware spinlocks); use C11 atomics only if the toolchain supports them.
- Barr-C: no bit-fields in signed types, no bitwise operators on signed integers (section 9).

Week 13 repo evidence required: a **data representation table**, **RAM before/after**,
**message size before/after**, and **correctness/performance** results. Record `sizeof`,
linker map RAM and stack high-water, queue element size x depth, P-task loop time and
telemetry bytes, labelled **measured**, **estimate** or **derived**. Keep a test that
proves each packed form round-trips correctly.

---

## 7. Assessment and milestones

Assessed on: **Mission** (objectives completed), **Navigation** (accuracy, control),
**Perception** (barcode, hump, obstacle), **Robustness** (recovery, stable operation),
**Engineering** (software quality, efficiency), **Integration** (interfaces, teamwork),
**Demonstration** (clear evidence under test). Write-up adds: completion time,
reliability, code quality, resource efficiency. "A fast subsystem is not enough if the
integrated robot is unreliable."

Milestones: Week 6 design document (a hurdle, not marked); **Tribunal 1, Week 8 (19 Oct
2026)** done, all readiness areas Amber, second report version due about a week later;
**Week 10 partial integration demo**; Week 12 practical quiz (debugging and
code-understanding); **Week 13 final demo, which must include Change Requirement 1**,
plus repo evidence (section 6). Acceptable evidence: GitHub commit link, short test log,
video timestamp, oscilloscope or serial screenshot, pin test result, task timing
measurement, meeting decision note, wiring photo. AI is a reviewer, not the author: every
member must be able to explain the decisions for their subsystem, and "AI said so" is not
a justification.

Briefing method: (1) define inputs, outputs and ownership; (2) create repeatable
subsystem tests; (3) record measurements and calibration; (4) integrate early through
stable APIs; (5) practise the Week 10 demonstration. Record every test in `testing/`
and link demo videos from `videos/`. Each interface needs at least one integration test
(`testing/integration/TEST_LOG.md`).

---

---

## 8. IMU / terrain subsystem (Buddy 4)

Moved to `app_program/subsystem4/CLAUDE.md`, which loads when working on files in that folder.

## 9. Coding standard: Barr-C:2018 (mandatory)

Applies to **our code** in `app_program/`. Kernel and template code is not ours to
reformat. Key rules:

Layout and style
- C99 only. Lines **<= 80 characters**. Indent **4 spaces**; **no tab characters** in source
  (`\t` inside strings is fine). Comments with `//` are fine.
- Braces **always** around blocks, even single statements. Switch statements always have
  a `default` and each `break` aligned with its case; comment any fall-through.
- Constants go on the left in equality tests (`if (NULL == p_obj)`). `else if` chains end
  with `else`. Nesting of `if/else` at most 2 deep. No assignments inside `if` tests.
- Infinite loops are `for (;;)`. No magic numbers as loop bounds. Avoid `goto` and
  `continue`. Never use `abort()`, `exit()`, `setjmp()`, `longjmp()`.
- Keep `#pragma`, inline assembly and compiler extensions to a few driver modules. Do not
  `#define` keywords.

Names
- Module (file) names: lowercase letters, digits, underscores. One header per source file
  with the same root name; the `.c` includes its own header first.
- Functions and variables: **lowercase snake_case**, descriptive, <= 31 characters,
  variables >= 3 characters, no leading underscore. Public functions are prefixed with
  the module name (e.g. `imu_read_accel()`). Macros: **UPPERCASE**.
- Types: lowercase with `_t`, declared with `typedef`; public types prefixed with the
  module name (`imu_event_t`).
- Variable prefixes: `g_` global, `p_` pointer, `pp_` pointer-to-pointer, `b_` boolean,
  `h_` handle; combined order `g`, `p|pp`, `b|h` (e.g. `g_p_buffer`).
- Task functions end with `_task`. Interrupt handlers end with `_isr`, are `static`, and a
  default/stub ISR exists for unused vectors.

Types and data
- Use **fixed-width integers** (`uint8_t`, `int16_t`, `uint32_t`, ...) whenever width
  matters. Do not use `short` or `long`; `char` only for text.
- No bit-fields in signed types. No bitwise operators on signed integers. Do not mix
  signed and unsigned in expressions.
- Packed or register-mapped structs: guard against compiler padding and check the size
  at compile time (`#if` / static assert).
- Booleans use `bool`; convert other values with relational operators.
- Avoid floating point; use fixed-point. If unavoidable: `float32_t`, `f` suffix, never
  test floats for equality, check `isfinite()`.
- Initialise every variable before use; pointers without an address start as `NULL`.

Structure
- Source file order: comment block, includes, types/constants/macros, static data, private
  prototypes, public functions, private functions. No unused includes. Never `#include`
  a `.c` file. Header guards on every header; no variable storage in headers; a public
  header never includes a private header.
- Private functions are `static`; every public function has a prototype in the header.
  Keep functions short, ideally a single return at the bottom. Use `const` and `volatile`
  wherever appropriate (ISR-shared data is `volatile`).

**Where this conflicts with the template.** The template code uses tabs and kernel types
(`UW`, `INT`, `ER`, `ID`, `EXPORT`/`IMPORT`/`LOCAL`, `UB`). For our own code: use spaces
and fixed-width types for our own data and interfaces; use the kernel types only where
the µT-Kernel API requires them (`tk_*` calls, `T_MSG`, `ID`, `ER`, `T_CTSK`, ...). Do
not reformat template files. Put a header comment in each file stating its purpose and
owner. Confirm with the instructor how strictly the standard is checked.

---

## 10. Open points to resolve with the team
1. Pin table: recorded in TEAM.md section 2 (done). Still open: a liveness LED pin and
   owner; S1's template LED is still on GP19. `CRASH_REPORT=1` clashes with the right
   encoder on GP0/GP1 (never use it on the car).
2. Who owns the persistent fault mask (Vehicle Controller per Version C, or Motion per the
   Buddy 2 row) and who writes `vehicle_controller` (Buddy 4 is doing it; Tribunal 1 says
   Buddy 2 with all).
3. Barcode (section 1b): symbology and A to Z code table location, how the direction is
   detected, whether any reversed pattern collides with another character, which IR reads
   the barcode now that it is on the left (pin approval), cross-junction versus other
   junctions, pending-character rule, unmapped or missing mapping behaviour, mapping
   frame and topic, retained or re-sent mapping. (U-Turn is now confirmed as one of the
   four movements.)
4. Event-flag API name (`tk_set_flg` vs `tk_set_evt`) and the EVG_sys wait/clear options.
5. WiFi/MQTT code runs inside the kernel's WiFi service context; changing it needs team
   and instructor agreement (TEAM.md).
6. System clock (125 MHz assumed) and all timing numbers are unmeasured.
7. Priorities: Motion and the encoder task are at P5 (2026-11-13, run on the car
   2026-11-16/17: IMU P9 period 2.6 to 17.5 ms, 1 late period in 47 s, measured, INT-14).
   Check the remaining `S*_PRI_*` macros against TEAM.md section 4.
8. IMU sample rate (50 versus 100 Hz) (section 8 item 2). Impact detection is removed
   (prof, 13 Nov 2026); lift/overturn stop is owned by the VC (IMU reports, VC stops):
   done and tested on the car (S4-U38 to U41 PASS, 17 Nov 2026, PR #12). Still open: film
   the lift-to-stop delay (NFR18).
9. Encoder distance interface from Motion to the IMU: done (IF-10, INT-10 PASS). The
   Q_events header is still open.
11. Unmerged teammate work (found 17 Nov 2026): branch `integration-s2-s5-for-buddy5` has
   Buddy 2's closed-loop `motion_line_drive()`, moves and turns, `Q_motion` / `EVG_stop`;
   Buddy 3's A to Z decode and `barcode_nav`; Buddy 5's obstacle turn and reverse logic.
   Its merge is on hold (plan: `testing/integration/INTEGRATION.md` section 2 step 8). Do
   not delete the branch. Wheels stalling from rest in duty mode (INTEGRATION problem 19)
   has a stopgap start kick on branch `s2-duty-start-kick`.
10. Where the AI usage log lives (section 12 proposes `docs/ai_usage_log.md`).

## 11. Rules for AI assistants working here
- Keep changes inside the requested subsystem folder. If a task needs a shared-file change
  (`app.h`, `app_main.c`, `TEAM.md`), say so and keep that edit minimal.
- Do not edit `build_make/`, `config/`, `device/`, `kernel/`, `include/`, `lib/`, `etc/`,
  `tools/` unless asked, and say so when you do.
- Do not invent pin numbers, I2C addresses, register values or timings. Take them from
  this file or the datasheets, or leave a `TODO:` comment.
- New code follows section 9. Report anything you could not verify (hardware behaviour,
  build results) instead of claiming it works.
- Run `make -j8` and `make TEST=1 -j8` and report the result. Do not commit or push
  unless asked. Never commit secrets or build outputs.
- Explain the design reasoning with every non-trivial change (section 12): which object,
  who owns it, why this primitive, what the worst-case blocking is. Do not just paste code.
- If you are unsure of a requirement, ask the user one precise question instead of
  guessing, and offer the alternatives with trade-offs.
- Log notable AI-assisted decisions in the AI usage log (section 12) and never write
  "AI said so" as a justification.


---

## 12. Working rules from the prof's lecture (`whathesaid.pdf`)

How the course is graded, and what that means for how we work. The one lever is **design
thinking, not code**: AI makes code ungradeable, so the prof marks how well we own our
design and how we handle a changed requirement.

Graded events
- Week 6 design document: a hurdle, not marked. Silence from the prof means fine, a short
  note means minor, a long message means a major red flag. A thin document means a harder
  requirement change, so keep it detailed.
- **Week 8 tribunal** (recorded, 1.5 h): Level 1 is our own subsystem (data structures,
  APIs, priorities, sequence, missing functions). Level 2 is the team web (how a
  teammate's change reaches us and ours reaches them; even "no change" must be justified).
  Every member speaks, screens are shared (prompts, code, flowcharts). Vague "it will
  affect a lot" earns nothing. A second report version follows about a week later.
- Week 12 practical quiz: debugging and code understanding.
- **Week 13 demo must include the requirement change** (section 6).
- Individual marks come from the subsystem each student owns, repo commits and peer
  evaluation. A subsystem judged too simple lowers that person's mark, so make Buddy 4's
  subsystem substantial: calibration, hump height with stated uncertainty, classification,
  fault handling, measured evidence.

Before wiring
- Submit or update the **pin-layout table and get approval before connecting any wire**
  (a GPIO wired straight to a motor kills the Pico). Replacement is on us if we skipped
  approval. Any pin change goes through the same step, and into TEAM.md section 2.
  Table columns: device, type, signal, label, Pico pin, interface, notes (note when
  wiring goes through the Robo Pico).

Repo and credit
- The repo is public and its link is shared. Commit **small and often**, from our own
  subsystem, with clear messages. "Only worked in the last week" shows and counts against us.
- Peer evaluation: declare what we really did, with proof (commit links, logs, videos).
- A real bug or vulnerability found in µT-Kernel can earn marks: keep evidence, impact and
  a proposed fix.

AI usage log (graded evidence of critical use)
- Keep `docs/ai_usage_log.md` (**PROPOSED path**): for each session record the prompt,
  whether each suggestion was good or bad and why, and accept/reject with a reason.
- Give the AI context (embedded, RTOS, µT-Kernel, Pico W, Robo Pico), ask it why a design is
  good and what the trade-offs are, and cross-check with a teammate who prompted
  independently. Be ready to screen-share the session at the tribunal.

RTOS design habits to show
- Real-time means a **bounded worst-case latency**, not speed. State the worst case.
- For every kernel object define its **invariant**: owner, who manipulates it, size,
  timeout, worst-case blocking, and how often data arrives (see the table in section 8).
- Choosing: shared resource needing safety uses a **mutex** (only mutexes get priority
  inheritance); frequent or large data uses a **mailbox** (pointer, zero-copy, the message
  must outlive the queue and both sides agree who frees it); small occasional messages use
  a **message buffer** (copied); pure state notification uses **event flags**; counted
  resources and producer/consumer use a **semaphore**; blocks use a **fixed memory pool**
  (no `malloc`).
- Critical sections are as short as possible: never block, allocate, log or do slow I/O.
- ISRs are tiny: clear the interrupt, capture the minimum data, wake a task. The
  lowest-priority interrupt outranks the highest-priority task.
- Steady-rate work (PID, the 10 ms loops) uses a **cyclic handler** or absolute deadlines.
- Debugging: disable one task at a time (comment out its create/start line) or change one
  priority at a time, predict the outcome first, and add a print in each task to see who
  runs. Never change two things together.
- Names follow `tk_<operation>_<object>`. The kernel API is stable across v1 to v3.
