/*
 * Host configuration protocol over UART0 - ESP32-S31 audio bridge.
 *
 * See serial_proto.c for the full wire format. The protocol shares the console
 * UART: while a host is active, ESP_LOG output is delivered as LOG frames.
 */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t serial_proto_init(void);

/* True while a host is actively speaking the frame protocol. */
bool serial_proto_host_active(void);

/* Push one diagnostic line to the host as a LOG frame (no-op if inactive). */
void serial_proto_log(const char *text);

/* BLE/SPP 侧字节入口（任务书 42 §3 契约，serial_proto.c :103 预留）：
 * 字节进第二套帧解析器，应答只回发起方。BTU 任务上下文可调，非阻塞。 */
/* Whole-write admission, nonblocking in BTU. False must be an ATT error. */
bool proto_feed_bytes(const uint8_t *data, size_t len);
/* Reset peer generation; the protocol task alone discards/parser-resets it. */
void proto_ble_link_changed(bool connected);
bool proto_ble_generation_matches(uint32_t generation);

#ifdef __cplusplus
}
#endif
