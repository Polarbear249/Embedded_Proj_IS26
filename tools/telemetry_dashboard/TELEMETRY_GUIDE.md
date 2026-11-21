# Telemetry Dashboard — Setup Guide (Buddy 1 / Subsystem 1)

How to get the car's live telemetry onto your own laptop screen.

Everything here runs on **your** laptop and **your** phone hotspot. Nothing
depends on the school WiFi, and nothing needs an internet connection once
Mosquitto is installed.

---

## 0. How the whole thing fits together

Read this once so the steps below make sense.

1. The car joins a WiFi network (your hotspot).
2. The car publishes a 27-byte telemetry frame 5 times a second to an **MQTT
   broker** running on your laptop. MQTT is just a small
   publish/subscribe messaging protocol — the broker is the post office in
   the middle.
3. **Mosquitto** is the broker. It listens on two doors at once:
   - port `1883` — plain MQTT, this is the door the **car** knocks on.
   - port `9001` — MQTT over WebSockets, this is the door the **browser**
     knocks on. Browsers cannot speak raw MQTT, only WebSockets, which is
     why we need both.
4. `picocar_dashboard.html` is the dashboard. It opens in your browser, connects to
   port 9001, subscribes to the car's topic, decodes the 27 bytes, and
   draws them.

So: **car → hotspot → laptop → Mosquitto → browser.** If any link in that
chain is down you see nothing, and the troubleshooting section at the end
tells you which link broke.

The topics in use:

- `mtk3smp-pico/telem` — car to dashboard, binary telemetry, 5 Hz
- `mtk3smp-pico/cmd` — dashboard to car, 3-byte commands
- `mtk3smp-pico/heartbeat` — car to dashboard, plain text, proof of life

---

## 1. Install Mosquitto (one time, needs internet)

### Windows

1. Go to https://mosquitto.org/download/
2. Download the **Windows x64 installer** (e.g. `mosquitto-2.x.x-install-windows-x64.exe`).
3. Run it. Accept the defaults. It installs to:
   ```
   C:\Program Files\mosquitto
   ```
4. When the installer offers to install it **as a Windows service**, let it.
   You can still start and stop it by hand later.

### macOS

```
brew install mosquitto
```

Config lives at `/opt/homebrew/etc/mosquitto/` (Apple Silicon) or
`/usr/local/etc/mosquitto/` (Intel).

### Linux

```
sudo apt install mosquitto mosquitto-clients
```

---

## 2. Configure Mosquitto (one time)

**This step is not optional.** Out of the box Mosquitto only accepts
connections from the same machine, and it does not speak WebSockets at
all. The car and the browser will both be refused until you do this.

Create a new file next to the Mosquitto install:

```
C:\Program Files\mosquitto\pico.conf
```

On Windows you will need to open Notepad **as Administrator** to save into
`Program Files`. Right-click Notepad, "Run as administrator", then
File > Open.

Put exactly this in it:

```
# Door 1: plain MQTT, for the Pico.
listener 1883
protocol mqtt

# Door 2: MQTT over WebSockets, for the browser dashboard.
listener 9001
protocol websockets

# No usernames or passwords. Fine for a closed hotspot on demo day.
allow_anonymous true
```

Save and close.

> **Why `allow_anonymous true`?** Mosquitto 2.x refuses all anonymous
> connections by default. Setting up certificates and passwords for a
> two-hour demo on a private hotspot is not worth it. Do not use this
> config on a network you do not control.

---

## 3. Set up the hotspot

### THE ONE THING THAT BREAKS THIS FOR EVERYONE

**The Pico W radio is 2.4 GHz only. It cannot see a 5 GHz network at all.**

If your hotspot is on 5 GHz, the car will report *"SSID not found"* — which
is a misleading error, because the network is obviously right there. It
isn't that the password is wrong or the name is wrong. The car genuinely
cannot see it.

Fix it before you do anything else:

- **iPhone:** Settings > Personal Hotspot > turn **ON** "Maximise
  Compatibility". This drops the hotspot to 2.4 GHz.
- **Android:** Settings > Hotspot & tethering > Wi-Fi hotspot > AP Band >
  choose **2.4 GHz**.
- **Windows laptop hotspot:** Settings > Network & Internet > Mobile
  hotspot > Properties > Band > **2.4 GHz**.

Other hotspot rules:

- Keep the SSID simple. Avoid emoji, apostrophes and spaces — phone
  hotspots are often named things like `Fitz's iPhone`, and the apostrophe
  has to be escaped in C. Rename it to something like `picocar` for the
  duration of the project and save yourself the trouble.
- The password must be at least 8 characters (WPA2 minimum).
- **Connect your laptop to the hotspot too.** The car and the laptop have
  to be on the same network. This is the single most common mistake.

---

## 4. Find your laptop's IP address on the hotspot

The car needs to know where the broker is, and "localhost" means nothing
to the car.

**Windows:**
```
ipconfig
```
Look for the adapter connected to the hotspot (usually "Wireless LAN
adapter Wi-Fi") and read the **IPv4 Address**. It will look like
`10.220.230.193` or `192.168.43.5`.

**macOS / Linux:**
```
ifconfig | grep "inet "
```

Write this number down. You need it twice — once in the firmware, once in
the dashboard.

> **This number changes.** Every time you reconnect to a hotspot, or
> connect to a different one, your laptop can be given a different address.
> If the car suddenly stops reaching the broker, re-run `ipconfig` first —
> nine times out of ten this is why.

---

## 5. Tell the firmware your network details

In the repo:

```
cd ~/pico/Embedded_Proj_IS26
cp config/wifi_secrets.example.h config/wifi_secrets.h
nano config/wifi_secrets.h
```

Fill in four things:

```c
#define WIFI_SSID     "picocar"              /* your hotspot name */
#define WIFI_PASSWORD "your-hotspot-password"
#define MQTT_BROKER_IP   "10.220.230.193"    /* the IP from step 4 */
#define MQTT_BROKER_PORT 1883                /* leave this alone */
```

Save with `Ctrl+O`, `Enter`, then exit with `Ctrl+X`.

> `config/wifi_secrets.h` is in `.gitignore`. It will never be committed,
> so your real WiFi password does not end up on GitHub. **Do not remove it
> from `.gitignore` and do not force-add it.** Only ever commit
> `wifi_secrets.example.h`.

---

## 6. Build the firmware

```
cd ~/pico/Embedded_Proj_IS26/build_make
make NET=lwip MQTT=1 -j8
```

`NET=lwip` compiles in the network stack. `MQTT=1` compiles in the MQTT
client. Without both, the car runs fine but is silent.

A successful build ends with no errors and produces:

```
build_make/mtk3pico_smp1_usb_cdc_wifi_lwip_mqtt.uf2
```

Confirm it's actually fresh — the timestamp should be *now*, not from
yesterday:

```
ls -lh mtk3pico_smp1_usb_cdc_wifi_lwip_mqtt.uf2
```

> If `make` says **"Nothing to be done for 'all'"**, nothing was rebuilt
> because nothing changed. If you *did* change something and still see
> this, run `make clean` first.

---

## 7. Flash the car

1. Hold down the **BOOTSEL** button on the Pico.
2. While still holding it, plug the USB cable into your laptop.
3. Release BOOTSEL. A new drive called **`RPI-RP2`** appears in File
   Explorer.
4. Drag `mtk3pico_smp1_usb_cdc_wifi_lwip_mqtt.uf2` onto that drive.
5. The drive disappears by itself and the car reboots into the new
   firmware. That is normal and means it worked.

To restart the car **without** reflashing, press the **RST** button on the
Robo Pico board. That re-runs whatever is already on it.

> **Watch out:** the car will start driving the moment it boots if the line
> sensors see a line. Put it on a box or hold it up before you plug it in.

---

## 8. Start the broker

**Windows** — open Command Prompt **as Administrator**:

```
cd "C:\Program Files\mosquitto"
mosquitto -c pico.conf -v
```

**macOS / Linux:**

```
mosquitto -c /path/to/pico.conf -v
```

`-v` means verbose. Leave this window open — you'll see every connection
and every message, which makes debugging enormously easier.

You should see two listeners start:

```
Opening ipv4 listen socket on port 1883.
Opening ipv4 listen socket on port 9001.
```

When the car connects you'll see a line naming the client `mtk3smp-pico`.

> **Windows Firewall will probably pop up the first time.** Tick **Private
> networks** and allow it. If you click Cancel by reflex, the car will
> never get through and you'll be debugging the wrong thing. To fix it
> later: Windows Security > Firewall & network protection > Allow an app
> through firewall > find mosquitto > tick Private.

> If Mosquitto was installed as a service it may already be running on
> 1883 with the *default* config, and starting it by hand will fail with
> "Address already in use". Stop the service first: `net stop mosquitto`.

---

## 9. Open the dashboard

1. In File Explorer, go to `tools/telemetry_dashboard/` in the repo.
2. **Double-click `picocar_dashboard.html`.** It opens in your browser. That's it —
   there is no web server to run, no `npm install`, no internet needed.
   Everything including the MQTT client is written into that one file.
3. At the top, in the **Broker** box, type the IP from step 4 (the same
   one you put in `wifi_secrets.h`).
   - If the dashboard is on the **same laptop** that runs Mosquitto, you
     can leave it as `localhost`.
   - If it's on a **different** laptop or a phone, you must type the
     broker laptop's IP.
4. Click **Connect**.

The status pill goes green and frames start arriving at about 5 Hz.

---

## 10. Reading the dashboard

Cards are grouped by whose subsystem owns the data, so you can tell at a
glance whose code to go shout at.

- **Motion (S2)** — vehicle state, motor state, wheel speeds left and
  right, distance travelled in cm, and (greyed) the raw **encoder
  counts** for each wheel. About 61 counts per cm; negative means the
  wheel went backwards. If L and R drift apart while driving straight,
  one wheel is slipping.
- **Line & barcode (S3)** — line error (how far off centre, signed),
  line OK flag, junction detected, and three barcode rows:
  - **Barcode letter** — the last letter scanned, e.g. `A`.
  - **Barcode** — what that letter means under the mapping the car
    reports, e.g. `LEFT`. A letter with no mapping shows
    `not mapped, default STRAIGHT`, which is what the car does with it.
  - **Last crossroad** (greyed, for debugging) — the turn the car made
    at the last cross-junction, e.g. `turned LEFT`. It only changes
    when the car reaches one. The car remembers one letter at a time,
    so scanning B before reaching a crossroad replaces an earlier C.
  - **Barcode reads** (greyed) — barcodes decoded OK vs failed since
    boot. Lots of failures means the scanner is struggling.
- **Terrain (S4)** —
  - **Highest hump** — the highest hump since the car booted. This is
    the figure the brief asks for ("report the highest hump peak
    experienced during the run"), so read it at the end of a run.
    Reset or reflash starts it from zero.
  - **Last hump** — the most recent hump.
  - **IMU** (greyed) — `starting`, `calibrating - keep still`, `active`
    or `FAULT`. Hump heights only count once it says `active`, so keep
    the car still at power-up until it does.
- **Obstacle (S5)** — obstacle valid flag, left and right clearances,
  and the measured width of the obstacle.
- **Faults** — five bits: motor stall, encoder, IMU, no echo, WiFi loss.
- **Link (S1)** — frames received, frames dropped, frames rejected,
  heartbeat, and (greyed) **Commands**: how many dashboard commands the
  car accepted vs rejected. Turns amber if any were rejected.

Three counters worth understanding:

- **received** — frames successfully decoded.
- **dropped** — gaps detected in the sequence number. The frame left the
  car but never arrived. **A few percent is normal over WiFi** and is not
  a bug; MQTT QoS 0 does not retransmit.
- **rejected** — a frame arrived but failed validation (wrong version or
  wrong length). This should stay at **zero**. If it is climbing, the
  firmware and the dashboard disagree about the frame format — someone
  changed one without the other.

**The whole grid greys out when frames stop arriving** (no frame for 2.5
seconds). That is the dashboard telling you the link is down, not that the
car is idle.

---

## 11. The command buttons

The dashboard can send commands back to the car.

- **Stop** — emergency stop. Cuts the motors and disarms line following.
- **Start** — clears the stop and re-arms line following.
- **Reset** — reboots the car, same as the RST button on the Robo Pico.
  It asks you to confirm first. The car stops, reboots and rejoins WiFi
  in about 10 seconds; the dashboard greys out meanwhile and recovers
  by itself. **The barcode mapping is lost on reset — send it again.**
- **Slow / Medium / Fast** — sets the base driving speed to 4, 7 and
  11 cm/s respectively.

> **Why only three speeds?** Below about 4 cm/s there isn't enough torque
> to overcome static friction and the motors just buzz. Above about
> 11 cm/s the duty cycle hits its ceiling and larger numbers do nothing.
> Fixed buttons mean nobody can type a value that silently does neither.

### The barcode mapping — do this before every demo run

The second bar sets which letter means which movement.

Subsystem 3 boots with a built-in table (A=left, B=right, C=straight,
D=U-turn), but `subsystem3.h` calls that a **test stand-in** and says the
real mapping must arrive from the PC over MQTT. That's what this bar
does. Until you press **Send mapping**, the car is running on the
stand-in.

To use it:

1. Pick a letter for each of Left, Right, Straight and U-turn.
2. Press **Send mapping**. That sends four `SET_MAP` commands, one per
   movement.
3. Watch the **Car has** line under the dropdowns. It shows the mapping
   the car itself reports in every frame, so when it changes to what you
   picked, the car has it. If it doesn't change within a second, press
   Send again (sending the same mapping twice is harmless).

   The serial console also shows four lines:
   ```
   [VC] MQTT SET_MAP 'A' -> LEFT
   [VC] MQTT SET_MAP 'B' -> RIGHT
   [VC] MQTT SET_MAP 'C' -> STRAIGHT
   [VC] MQTT SET_MAP 'D' -> U-TURN
   ```

The dashboard refuses to send if you give the same letter to two
movements — the car wouldn't reject that, the second one would just
silently win, so it's caught here instead of on the track.

The mapping takes effect at the next cross-junction. It is not saved
across a reboot, so **re-send it after every reflash or RST** — you'll
see **Car has** go back to `A=LEFT B=RIGHT C=STRAIGHT D=U-TURN`.

To confirm a command actually landed, watch the serial console (step 12).
You should see lines like:

```
[VC] MQTT STOP
[VC] MQTT START
[VC] MQTT SET_SPEED 7 cm/s -> duty 119
```

---

## 12. Watching the serial console (optional but recommended)

The car prints a status line over USB once a second. When something is
wrong this is where it says so.

**Windows** — use PuTTY or the Arduino IDE serial monitor. Find the COM
port in Device Manager under "Ports (COM & LPT)". Speed `115200`.

**WSL / Linux:**
```
sudo screen /dev/ttyACM0 115200
```
Exit screen with `Ctrl+A` then `K`, then `y`.

**macOS:**
```
screen /dev/tty.usbmodem* 115200
```

---

## 13. Troubleshooting

Work down this list in order. Each step narrows which link in the chain is
broken.

**Dashboard won't connect / status pill stays red**

- Is Mosquitto actually running? The window from step 8 should still be
  open.
- Does the `pico.conf` really have `listener 9001` and
  `protocol websockets`? Without those, port 9001 isn't open at all.
- Did you start Mosquitto with `-c pico.conf`? Starting it with no `-c`
  loads the *default* config, which has neither.
- Is the IP in the Broker box right? Re-run `ipconfig`.

**Dashboard connects but no frames arrive**

The browser side is fine; the car isn't publishing.

- Check the Mosquitto window. Do you see `mtk3smp-pico` connect? If not,
  the car never reached the broker.
- Is the car powered and booted? Press RST.
- Is the car on the **same** network as the laptop?
- Did the laptop IP change since you last built? This is the most common
  cause by a mile. Re-run `ipconfig`, update `wifi_secrets.h`, rebuild,
  reflash.

**Car says "SSID not found"**

- The hotspot is on 5 GHz. Go back to step 3. This is almost always it.
- Check the SSID for a typo, or for an apostrophe that needs escaping.

**Car connects to WiFi but never to the broker**

- Windows Firewall is blocking port 1883. See the note in step 8.
- `MQTT_BROKER_IP` doesn't match the laptop's current IP.
- Mosquitto is running with the default config and refusing anonymous
  connections. You need `allow_anonymous true`.

**Frames arrive but every value is zero or nonsense**

- The **rejected** counter is probably climbing. Firmware and dashboard
  disagree on the frame format. Make sure both are from the same commit.

**"rejected" is climbing and nothing ever displays**

The frame is **v4, 27 bytes** as of this version. A car still running
older firmware sends 13-, 14- or 17-byte frames, which this dashboard rejects
on purpose rather than mis-reading. Rebuild and reflash the car from the same
commit as the dashboard.

**Barcode letter shows "none yet" forever**

That is a real reading, not a missing one — `bc_char` stays 0 until S3
reports a decoded letter. If it never changes, the barcode sensor isn't
decoding, which is a Subsystem 3 problem, not a telemetry one.

**Telemetry works but the car never moves**

- It may be in emergency stop. Press **Start** on the dashboard.
- Check the faults card — a motor stall or encoder fault will hold it.
- The line sensors need to actually see the line. They are sensitive to
  mounting height; the TCRT5000 wants roughly 3–8 mm above the surface.

**Everything worked yesterday and nothing works today**

Run `ipconfig`. It's the IP address. It's nearly always the IP address.

---

## 14. Quick reference

Build:
```
cd ~/pico/Embedded_Proj_IS26/build_make && make NET=lwip MQTT=1 -j8
```

Output file:
```
build_make/mtk3pico_smp1_usb_cdc_wifi_lwip_mqtt.uf2
```

Start broker:
```
cd "C:\Program Files\mosquitto" && mosquitto -c pico.conf -v
```

Find IP:
```
ipconfig
```

Dashboard:
```
tools/telemetry_dashboard/picocar_dashboard.html   (double-click)
```

Ports: **1883** car to broker, **9001** broker to browser.

Topics: `mtk3smp-pico/telem`, `mtk3smp-pico/cmd`, `mtk3smp-pico/heartbeat`.

Telemetry frame: **v4, 27 bytes**. Command frame: v1, 3 bytes.

Before each demo run: **Send mapping**, then Start.
