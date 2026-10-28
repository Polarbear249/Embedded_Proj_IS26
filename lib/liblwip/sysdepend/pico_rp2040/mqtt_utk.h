/*
 *	mqtt_utk.h
 *	MQTT client wiring for micro T-Kernel (RP2040, NO_SYS=1)
 */

#ifndef MQTT_UTK_H
#define MQTT_UTK_H

/*
 * Call once, from cyw43_utk_task() the first time the link reaches
 * CYW43_LINK_UP (see cyw43_utk.c). Starts the MQTT connection to
 * MQTT_BROKER_IP:MQTT_BROKER_PORT (config/wifi_secrets.h) and, once
 * connected, a periodic heartbeat publish on MQTT_TOPIC. Safe to call more
 * than once -- a second call while already connecting/connected is a no-op.
 */
void mqtt_utk_start(void);

/*
 * Binary application traffic (Subsystem 1, TEAM.md section 5, IF-07/IF-08).
 * Telemetry goes out on MQTT_UTK_TOPIC_TELEM every MQTT_UTK_TELEM_MS; commands
 * arrive on MQTT_UTK_TOPIC_CMD. The library knows nothing about the frame
 * layouts: it only moves bytes through two hooks that the application
 * overrides. The defaults in mqtt_utk.c do nothing. Both hooks run inside
 * cyw43_utk_task and must never block.
 */
#include <stdint.h>

#define MQTT_UTK_TOPIC_TELEM	"mtk3smp-pico/telem"
#define MQTT_UTK_TOPIC_CMD	"mtk3smp-pico/cmd"
#define MQTT_UTK_TELEM_MS	200
#define MQTT_UTK_BUF_MAX	32U	/* largest binary frame, bytes */

/* Fill buf (at most max bytes) with the next telemetry frame and return its
 * length, or return 0 when there is nothing to send. */
uint32_t mqtt_app_tx(uint8_t *buf, uint32_t max);

/* One complete inbound command message. data is only valid during the call. */
void mqtt_app_rx(const char *topic, const uint8_t *data, uint32_t len);

#endif /* MQTT_UTK_H */
