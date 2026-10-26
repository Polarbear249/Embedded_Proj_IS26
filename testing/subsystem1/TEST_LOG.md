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
