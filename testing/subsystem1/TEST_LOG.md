# Subsystem 1 — test log

**Subsystem:** LED heartbeat (example)  ·  **Student ID:** <student ID>  ·  **Code:** `app_program/subsystem1/`

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S1-U01 | cyclic handler is running | | auto | `subsystem1_test.c` | `[TEST] S1-U01 PASS` |
| S1-U02 | LED task sleeps between ticks (not busy) | | auto | `subsystem1_test.c` | `[TEST] S1-U02 PASS` |
| S1-U03 | LED on GP19 toggles every 250 ms | | manual | 1. Flash normal image. 2. Watch LED / measure with scope. | 2 Hz blink, 50 % duty |
| S1-U04 | WiFi joins the hotspot and gets an IP | P10 WiFi/MQTT task | manual | 1. Set SSID/password in `config/wifi_secrets.h`. 2. `make NET=lwip MQTT=1 TEST=1`, flash. 3. Watch COM9. | `wifi: link up, ip=...` within about 15 s |
| S1-U05 | MQTT client connects to the broker and subscribes | P10 WiFi/MQTT task | manual | 1. Run Mosquitto on the laptop on the same hotspot. 2. Set `MQTT_BROKER_IP`. 3. Flash, watch the broker window. | Broker logs `New client connected` and `SUBSCRIBE mtk3smp-pico/heartbeat` |
| S1-U06 | Heartbeat is published and received back | Design review: 1 s heartbeat | manual | Watch COM9 and the broker for `TX` / `RX` lines and their spacing. | `hb N` every 1 s, each TX followed by an RX |
| S1-U07 | WiFi state task sleeps between polls (not busy) | P10 WiFi task | auto | `subsystem1_test.c` | `[TEST] S1-U07 PASS` |
| S1-U08 | WiFi state task polls about every 200 ms | P10 WiFi task | auto | `subsystem1_test.c` | `[TEST] S1-U08 PASS` |
| S1-U09 | WiFi state task reports link UP with an IP | P10 WiFi task | auto | `subsystem1_test.c`; needs the hotspot | `[TEST] S1-U09 PASS` within 15 s of boot |
| S1-U10 | Telemetry snapshots sent to Q_telem are handed to MQTT at about 5 Hz | IF-07 Q_telem | auto | `subsystem1_test.c` sends 25 snapshots 200 ms apart; needs hotspot, broker and `MQTT=1` build | `[TEST] S1-U10 PASS` (at least 22 handed to MQTT) |
| S1-U11 | Only a valid command reaches the Vehicle Controller | IF-08 Q_cmd | auto | `subsystem1_test.c` feeds one good and three bad frames to the receive hook | `[TEST] S1-U11 PASS` (1 accepted, 3 rejected) |
| S1-U12 | The broker receives 13-byte telemetry frames at about 5 Hz | IF-07 Q_telem | manual | 1. Run `mosquitto_sub -h <broker ip> -t "mtk3smp-pico/#" -F "%t %l"`. 2. Reboot the Pico. | About 25 lines `mtk3smp-pico/telem 13`, about 5 between heartbeat lines |
| S1-U13 | A command published over MQTT is accepted or rejected by S1 | IF-08 Q_cmd | manual | 1. Watch COM9. 2. `mosquitto_pub -t mtk3smp-pico/cmd -f good.bin` (01 32 A9), then `-f bad.bin` (02 32 A9). | `command 3 arg 42 seq 9 accepted`, then `command rejected (code 3)` |
| S1-U14 | WiFi recovers by itself after the hotspot drops | A1; design review forced reconnection | manual | 1. Watch COM9 with normal heartbeats. 2. Turn the hotspot off for 30 s or more. 3. Turn it on. Do not reset the Pico. | `s1: WiFi DOWN`, repeated `wifi: joining`, then `s1: WiFi UP` and `mqtt: connected` with no reset |
| S1-P01 | Telemetry frame is 13 bytes with version byte 1 | Design review: compact binary frame | auto (PC) | `testing/subsystem1/pc/run_tests.sh` | `S1-P01 PASS` |
| S1-P02 | Golden vector fixes the bit order (no struct cast) | Design review: explicit shifts and masks | auto (PC) | same | `S1-P02 PASS` |
| S1-P03 | Last field fills the final bits | same | auto (PC) | same | `S1-P03 PASS` |
| S1-P04 | Maximum of every field round-trips | same | auto (PC) | same | `S1-P04 PASS` |
| S1-P05 | 100000 random valid snapshots round-trip | same | auto (PC) | same | `S1-P05 PASS` |
| S1-P06 | Wrong frame length rejected | Decoder validation | auto (PC) | same | `S1-P06 PASS` |
| S1-P07 | Unknown version rejected | Decoder validation | auto (PC) | same | `S1-P07 PASS` |
| S1-P08 | Out-of-range motion state rejected | Decoder validation | auto (PC) | same | `S1-P08 PASS` |
| S1-P09 | Encoder refuses values wider than their field | Encoder validation | auto (PC) | same | `S1-P09 PASS` |
| S1-P10 | Null pointers rejected | Decoder validation | auto (PC) | same | `S1-P10 PASS` |
| S1-P11 | Command golden bytes and round trip | IF-08 | auto (PC) | same | `S1-P11 PASS` |
| S1-P12 | Bad command, argument, seq, version and length rejected | IF-08 | auto (PC) | same | `S1-P12 PASS` |
| S1-P13 | Random bytes never crash the decoder or yield a bad value | Decoder validation | auto (PC) | same | `S1-P13 PASS` |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| YYYY-MM-DD | S1-U01 | `abc1234` | | PASS / FAIL | `logs/YYYY-MM-DD_….txt` | |
| 2026-10-27 | S1-U04 | `buddy1-wifi` (uncommitted) | Fitz | PASS | `logs/2026-10-27_mqtt_heartbeat.txt` | MQTT only runs after the link is up. One earlier boot failed with `association failure` (log not saved); the retry joined as 10.107.242.243. |
| 2026-10-27 | S1-U05 | `buddy1-wifi` (uncommitted) | Fitz | PASS | `logs/2026-10-27_mqtt_heartbeat.txt` | Broker showed the client connect from 10.107.242.243 and the SUBSCRIBE. |
| 2026-10-27 | S1-U06 | `buddy1-wifi` (uncommitted) | Fitz | FAIL | `logs/2026-10-27_mqtt_heartbeat.txt` | TX and RX both work, but the period is 5 s (`MQTT_UTK_HEARTBEAT_MS` = 5000), not the 1 s in the design. Rate change touches shared MQTT code; agree with team/instructor first. |
| 2026-10-28 | S1-U06 | `buddy1-wifi` (uncommitted) | Fitz | PASS | `logs/2026-10-28_s1_wifi_recovery.txt` | Re-test after `MQTT_UTK_HEARTBEAT_MS` was set to 1000 in `mqtt_utk.c` (shared file; list it in the PR). `hb N` every 1 s with an RX after each TX. |
| 2026-10-28 | S1-U07 | `buddy1-wifi` (uncommitted) | Fitz | PASS | `logs/2026-10-28_s1_boot_no_network.txt` | |
| 2026-10-28 | S1-U08 | `buddy1-wifi` (uncommitted) | Fitz | PASS | `logs/2026-10-28_s1_wifi_recovery.txt` | |
| 2026-10-28 | S1-U09 | `buddy1-wifi` (uncommitted) | Fitz | FAIL | `logs/2026-10-28_s1_boot_no_network.txt` | Boot join failed with `association failure` and no new `wifi: joining` line appeared for about 40 s although the kernel WiFi poll counter kept rising. No network, so FAIL is correct. Re-join fix under test. |
| 2026-10-28 | S1-U09 | `buddy1-wifi` (uncommitted) | Fitz | PASS | `logs/2026-10-28_s1_telemetry_boot.txt` | First status was `association failure`; the library re-joined about 5 s later and the link came up. |
| 2026-10-28 | S1-U10 | `buddy1-wifi` (uncommitted) | Fitz | FAIL | `logs/2026-10-28_s1_boot_no_network.txt` | No network in this boot (see S1-U09 FAIL), so nothing could be published. |
| 2026-10-28 | S1-U10 | `buddy1-wifi` (uncommitted) | Fitz | PASS | `logs/2026-10-28_s1_telemetry_boot.txt` | Needs the hotspot, the broker and the MQTT build. Frames reaching the broker are recorded under S1-U12. |
| 2026-10-28 | S1-U11 | `buddy1-wifi` (uncommitted) | Fitz | PASS | `logs/2026-10-28_s1_telemetry_boot.txt` | One command accepted (3, arg 42, seq 9); rejected with codes 3 (version), 4 (range) and 2 (length). Subsystem 1 summary 7 passed, 0 failed. |
| 2026-10-28 | S1-U12 | `buddy1-wifi` (uncommitted) | Fitz | PASS | `logs/2026-10-28_s1_telemetry_listener.png` | Listener on 10.229.210.193 showed `mtk3smp-pico/telem 13` lines, about 5 between each `heartbeat` line. The top of the window was cut off in the screenshot. |
| 2026-10-28 | S1-U13 | `buddy1-wifi` (uncommitted) | Fitz | PASS | `logs/2026-10-28_s1_command_mqtt.txt` | `s1: command 3 arg 42 seq 9 accepted`, then `s1: command rejected (code 3)` after publishing `good.bin` and `bad.bin` from the laptop. |
| 2026-10-28 | S1-U14 | `buddy1-wifi` (uncommitted) | Fitz | PASS | `logs/2026-10-28_s1_wifi_recovery.txt` | Recovered with no reset. `s1: WiFi DOWN` about 10 s after the last reply; re-join attempts every 6 s; `s1: WiFi UP` about 35 s after the drop. MQTT reconnected about 58 s after WiFi UP because the keepalive is 60 s (known limitation). A first run before the `mqtt_utk.c` start-guard fix printed about 150 `mqtt_client_connect() failed, err=-10` lines (log not saved). |
| 2026-10-28 | S1-P01 | `buddy1-wifi` (uncommitted) | Fitz | PASS | `logs/2026-10-28_s1_pc_frame_tests.txt` | 13 of 13 passed on the PC. |
| 2026-10-28 | S1-P02 | `buddy1-wifi` (uncommitted) | Fitz | PASS | `logs/2026-10-28_s1_pc_frame_tests.txt` | |
| 2026-10-28 | S1-P03 | `buddy1-wifi` (uncommitted) | Fitz | PASS | `logs/2026-10-28_s1_pc_frame_tests.txt` | |
| 2026-10-28 | S1-P04 | `buddy1-wifi` (uncommitted) | Fitz | PASS | `logs/2026-10-28_s1_pc_frame_tests.txt` | |
| 2026-10-28 | S1-P05 | `buddy1-wifi` (uncommitted) | Fitz | PASS | `logs/2026-10-28_s1_pc_frame_tests.txt` | |
| 2026-10-28 | S1-P06 | `buddy1-wifi` (uncommitted) | Fitz | PASS | `logs/2026-10-28_s1_pc_frame_tests.txt` | |
| 2026-10-28 | S1-P07 | `buddy1-wifi` (uncommitted) | Fitz | PASS | `logs/2026-10-28_s1_pc_frame_tests.txt` | |
| 2026-10-28 | S1-P08 | `buddy1-wifi` (uncommitted) | Fitz | PASS | `logs/2026-10-28_s1_pc_frame_tests.txt` | |
| 2026-10-28 | S1-P09 | `buddy1-wifi` (uncommitted) | Fitz | PASS | `logs/2026-10-28_s1_pc_frame_tests.txt` | |
| 2026-10-28 | S1-P10 | `buddy1-wifi` (uncommitted) | Fitz | PASS | `logs/2026-10-28_s1_pc_frame_tests.txt` | |
| 2026-10-28 | S1-P11 | `buddy1-wifi` (uncommitted) | Fitz | PASS | `logs/2026-10-28_s1_pc_frame_tests.txt` | |
| 2026-10-28 | S1-P12 | `buddy1-wifi` (uncommitted) | Fitz | PASS | `logs/2026-10-28_s1_pc_frame_tests.txt` | |
| 2026-10-28 | S1-P13 | `buddy1-wifi` (uncommitted) | Fitz | PASS | `logs/2026-10-28_s1_pc_frame_tests.txt` | |
