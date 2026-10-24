/*
 * BLE tuning channel - ESP32-S31 audio bridge.
 *
 * 角色：BLE GATT server，把 AA55 主机协议（serial_proto）透传到手机浏览器
 *       （Web Bluetooth）；RX 特征收字节喂协议，TX 特征 notify 回字节。
 *
 * 契约：uac_bridge_main.c 在 a2dp_audio_init()+serial_proto_init() 之后调用
 *       ble_tuning_init()；serial_proto.c 经 proto_feed_bytes() 喂入 RX、经
 *       ble_tuning_send() 取走 TX。BLE 与 UART 是同一协议的两个传输面，
 *       同一时刻只支持一个在班主机（应答只回发起方，见 serial_proto.c）。
 *       失败不回滚：BLE 起不来只记日志，A2DP/串口不受影响。
 *
 * 文档锚点：任务书 42 §3（改动面）、41 §6.5（铁律：开 BLE 不许弄坏 A2DP）；
 *       05 §2（帧格式，字节流语义 ⇒ MTU 分片对端无感）。
 *
 * 是否冻结：否。改动后必须复核 dsp_engine.c.obj 哈希（01 §2）。
 */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 启动 BLE（GATT 服务 + 广播）。要求 Bluedroid 已 enable。 */
esp_err_t ble_tuning_init(void);

/* Protocol-task only. Fragment at MTU-3, bounded congestion wait. False means
 * not completely queued; success is not proof of over-air delivery. The
 * session token prevents replies leaking into a reconnected host. */
bool ble_tuning_send(const uint8_t *data, size_t len, uint32_t session_generation);

/* 诊断用：当前是否有 BLE 主机在线。 */
bool ble_tuning_is_connected(void);

/* 诊断用：因拥塞被丢弃的 notify 字节数。 */
uint32_t ble_tuning_dropped_bytes(void);

#ifdef __cplusplus
}
#endif
