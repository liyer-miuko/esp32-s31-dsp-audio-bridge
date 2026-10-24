/*
 * Host configuration protocol over UART0 - ESP32-S31 audio bridge (v11).
 *
 * Wire format (identical to the WebUI, see html前端/dsp webui.html):
 *
 *   frame   : AA 55 | len | payload[len] | crc8
 *   crc8    : poly 0x07, init 0x00, over [len, payload...]
 *   uart    : 115200 8N1, no flow control
 *
 * Host -> device
 *   [0xA0]                                   PING
 *   [0xA2]                                   SYNC_CONFIG (request full dump)
 *   [0xA5]                                   SAVE_CONFIG (persist to NVS)
 *   [0xA6]                                   SYS_STATUS request
 *   [msg_id,0x01,block,ch,param,idx,value…]  PARAM_UPDATE (4-byte f32 or 1 byte)
 *
 * Device -> host
 *   [0xA1]                                   PONG
 *   [0xA3, ascii…]                           LOG (chunked, best effort)
 *   [0xA4, 24B]                              TELEMETRY (1 Hz while host active)
 *   [0xA6, 26B]                              SYS_STATUS response
 *   [0x00,0x02,total_lo,total_hi,data…]      BULK_CONFIG first frame
 *   [data…]                                  BULK_CONFIG continuation frames
 *   [msg_id,0x80,status]                     ACK
 *   [msg_id,0x82,code,detail]                ERROR
 *
 * Console sharing: the host COM port is also the log output. While a host is
 * speaking the protocol, ESP_LOG output is redirected into LOG frames so raw
 * text cannot corrupt the binary stream. When no valid frame arrives for
 * CONFIG_SERIAL_PROTO_TIMEOUT_MS the session ends and raw text logs resume,
 * which keeps the existing LA/serial debug workflow untouched.
 *
 * Transport: this board exposes the host COM port through the chip's native
 * USB Serial/JTAG peripheral (VID 303A:1001, "USB 串行设备 (COMx)"), NOT through
 * an external USB-UART bridge. The protocol therefore defaults to that
 * transport; CONFIG_SERIAL_PROTO_TRANSPORT_UART0 selects UART0 instead for
 * boards that break the console UART out to a bridge.
 *
 * BLE tuning channel (task 42): the same wire format also runs over BLE GATT
 * (ble_tuning.c). BLE bytes enter via proto_feed_bytes() into a second parser
 * instance, and replies go back only to the transport that sent the frame
 * ("first valid sender owns a bounded lease"); both byte streams therefore never
 * corrupt each other's framing.
 */
#include <math.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#include "esp_flash.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_partition.h"
#include "esp_private/esp_clk.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/message_buffer.h"
#include "freertos/task.h"
#include "sdkconfig.h"

#if CONFIG_SERIAL_PROTO_TRANSPORT_UART0
#include "driver/uart.h"
#else
#include "driver/usb_serial_jtag.h"
#endif

#include "a2dp_audio.h"
#include "ble_tuning.h"
#include "dsp_audio.h"
#include "dsp_config.h"
#include "dsp_version.h"
#include "dsp_engine.h"
#include "hw_mute.h"
#include "i2s_out_v2.h"
#include "serial_proto.h"
#include "amp_control.h"
#include "uac2_audio.h"

static const char *TAG = "proto";

/* ---------------- Protocol constants ---------------- */
#define PROTO_H0            0xAA
#define PROTO_H1            0x55
#define PROTO_MAX_PAYLOAD   255

/* host -> device */
#define MSG_PARAM_UPDATE    0x01
#define MSG_PING            0xA0
#define MSG_SYNC_CONFIG     0xA2
#define MSG_SAVE_CONFIG     0xA5
#define MSG_SYS_STATUS      0xA6
#define MSG_FILTER_DUMP_REQ 0xA7
#define MSG_RESET_CONFIG    0xA8
#define MSG_DSP_SELFTEST    0xA9
#define MSG_PRESET_SAVE     0xAA   /* [0xAA, slot] 当前配置 -> 设备端预设槽 */
#define MSG_PRESET_LOAD     0xAB   /* [0xAB, slot] 槽 -> 当前配置（随后自动回发 BULK） */
#define MSG_PRESET_CLEAR    0xAC   /* [0xAC, slot] 清除槽 */
#define MSG_PRESET_INFO     0xAD   /* 应答 [0xAD, slots, used_mask] */
#define MSG_METERS          0xAE   /* 电平表：应答 [0xAE, ver, flags, gr_mdB u16, peakL u32, peakR u32, clip u32] */
/* ---- 蓝牙状态（v25 / 固件 0.25 实现；播放控制仍预留）----
 *   0xB0 BT_STATUS 请求 -> 0xB1 应答（连接/流状态/编解码/采样率/水位/丢包）
 *   0xB2 BT_CTRL 播放控制：**本轮不做**（需 AVRCP，用户 2026-09-14 决定只做状态上报）
 *   0xB3 DSP over BLE（同 AA55 协议，换传输层即可）
 *   帧格式与现有完全一致，将来把 BLE GATT/SPP 收到的字节喂给 proto_feed_bytes() 即可。 */
#define MSG_BT_STATUS_REQ   0xB0
#define MSG_DSP_PATH_TEST   0xB5   /* query [B5], set [B5, 0/1]; RAM only */
#define MSG_UAC_INPUT_PROBE 0xB6   /* RAM-only pre-FU step probe; see 0.131 wire layout */
/* 0xB1 BT_STATUS 应答：48 B（见 proto_send_bt_status 的布局注释） */

/* ---- 硬件控制层状态（v38 / 固件 0.38，阶段 C1）----
 *   0xAF HW_STATUS 请求 -> 0xAF 应答（与 0xA6/0xA7 同 id 应答的惯例一致）
 *   ★ 为什么需要它：mute 是**一根 GPIO 的时序行为**，光看参数回读无法判断
 *   "现在到底静音没有、上一跳是什么时候"。没有这一帧，验收就只能靠 LA 接线；
 *   有了它，主机侧可以先做一轮**设备自报**的时序核对（LA 仍是权威尺子）。
 *   0xAF is allocated to HW_STATUS below; it is not a free command id. */
#define MSG_HW_STATUS       0xAF

/* device -> host */
#define MSG_BULK_CONFIG     0x02
#define MSG_ACK             0x80
#define MSG_ERROR           0x82
#define MSG_PONG            0xA1
#define MSG_LOG             0xA3
#define MSG_TELEMETRY       0xA4
#define MSG_FILTER_DUMP     0xA7   /* designed biquad coefficients, for host verification */
#define MSG_BT_STATUS       0xB1   /* 蓝牙状态应答（见上方 MSG_BT_STATUS_REQ） */

/* ERROR codes */
#define ERR_UNKNOWN_MSG     0x01
#define ERR_BULK_TOO_BIG    0x02
#define ERR_SAVE_FAILED     0x03
#define ERR_HOST_BUSY       0x04

#define PROTO_RX_BUF        1024
#define PROTO_TX_BUF        2048
#define PROTO_BULK_FIRST_DATA (PROTO_MAX_PAYLOAD - 4)
#define PROTO_LOG_CHUNK     200
#define PROTO_WRITE_TIMEOUT_MS 100

/* ---------------- State ---------------- */
static vprintf_like_t    s_prev_vprintf;
static SemaphoreHandle_t s_tx_lock;        /* recursive */
static volatile bool     s_host_active;
static volatile int64_t  s_last_rx_us;
static volatile bool     s_bulk_active;
static volatile uint32_t s_frames_rx;
static volatile uint32_t s_crc_errors;
static uint32_t s_control_session;

static uint8_t s_rx_buf[PROTO_RX_BUF];
static uint8_t s_bulk[DSP_CONFIG_BLOB_MAX];
static uint8_t s_cpu_load[2];

/* Both parsers execute on proto_task. One transport owns the control lease;
 * another sender receives BUSY on its own link and cannot steal replies or
 * interleave DSP/NVS mutations. BLE callbacks never wait for this task. */
typedef enum { PROTO_ROUTE_UART = 0, PROTO_ROUTE_BLE = 1 } proto_route_t;
static volatile proto_route_t s_reply_route = PROTO_ROUTE_UART;
static MessageBufferHandle_t s_ble_rx_mb;
static portMUX_TYPE s_ble_mux = portMUX_INITIALIZER_UNLOCKED;
static uint32_t s_ble_generation, s_reply_ble_generation, s_ble_rx_lost;
static bool s_ble_online;
#define PROTO_BLE_RX_BYTES 2048
#define PROTO_BLE_WRITE_MAX 512
typedef struct { uint32_t generation; uint8_t bytes[PROTO_BLE_WRITE_MAX]; } ble_write_t;
static uint32_t ble_generation(bool *online)
{
    portENTER_CRITICAL(&s_ble_mux);
    const uint32_t generation = s_ble_generation;
    if (online) *online = s_ble_online;
    portEXIT_CRITICAL(&s_ble_mux);
    return generation;
}

/* ---------------- Transport abstraction ---------------- */
#if CONFIG_SERIAL_PROTO_TRANSPORT_UART0
#define PROTO_TRANSPORT_NAME "UART0"
static esp_err_t proto_transport_install(void)
{
    esp_err_t err = uart_driver_install(UART_NUM_0, PROTO_RX_BUF, PROTO_TX_BUF, 0, NULL, 0);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        return err;
    }
    uart_config_t cfg = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    return uart_param_config(UART_NUM_0, &cfg);
}
static int proto_transport_read(uint8_t *buf, size_t len, uint32_t ticks)
{
    return uart_read_bytes(UART_NUM_0, buf, len, ticks);
}
static int proto_transport_write(const uint8_t *buf, size_t len, TickType_t ticks)
{
    (void)ticks;
    return uart_write_bytes(UART_NUM_0, buf, len);
}
#else
#define PROTO_TRANSPORT_NAME "USB-Serial/JTAG"
static esp_err_t proto_transport_install(void)
{
    if (usb_serial_jtag_is_driver_installed()) {
        return ESP_OK;
    }
    usb_serial_jtag_driver_config_t cfg = {
        .tx_buffer_size = PROTO_TX_BUF,
        .rx_buffer_size = PROTO_RX_BUF,
    };
    return usb_serial_jtag_driver_install(&cfg);
}
static int proto_transport_read(uint8_t *buf, size_t len, uint32_t ticks)
{
    return usb_serial_jtag_read_bytes(buf, (uint32_t)len, ticks);
}
static int proto_transport_write(const uint8_t *buf, size_t len, TickType_t ticks)
{
    return usb_serial_jtag_write_bytes(buf, len, ticks);
}
#endif

/* RX frame parser（每传输面一套状态：两路字节流互不串台） */
typedef enum {
    RX_H0 = 0,
    RX_H1,
    RX_LEN,
    RX_PAYLOAD,
    RX_CRC,
} rx_state_t;

typedef struct {
    rx_state_t     state;
    uint8_t        len;
    uint8_t        pos;
    uint8_t        payload[PROTO_MAX_PAYLOAD];
    proto_route_t  route;
    uint32_t generation; /* BLE frame's peer/queue generation */
} proto_parser_t;

static proto_parser_t s_parser_uart = { .state = RX_H0, .route = PROTO_ROUTE_UART };
static proto_parser_t s_parser_ble  = { .state = RX_H0, .route = PROTO_ROUTE_BLE };

/* Telemetry / monitor sampling */
static float    s_clock_ppm;

#if CONFIG_FREERTOS_GENERATE_RUN_TIME_STATS
static uint32_t     s_idle_prev[2];
static uint32_t     s_total_prev;
#endif

/* ---------------- CRC8 (poly 0x07) ---------------- */
static uint8_t crc8_update(uint8_t crc, const uint8_t *data, size_t len)
{
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int b = 0; b < 8; b++) {
            crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x07) : (uint8_t)(crc << 1);
        }
    }
    return crc;
}

static uint8_t frame_crc(uint8_t len, const uint8_t *payload)
{
    uint8_t crc = crc8_update(0, &len, 1);
    return crc8_update(crc, payload, len);
}

/* ---------------- TX ---------------- */
static bool proto_send_payload_to(proto_route_t route, uint32_t generation,
                                  const uint8_t *payload, size_t len, TickType_t ticks)
{
    if (len > PROTO_MAX_PAYLOAD) {
        return false;
    }
    if (s_tx_lock == NULL || xSemaphoreTakeRecursive(s_tx_lock, ticks) != pdTRUE) {
        return false;
    }

    uint8_t frame[3 + PROTO_MAX_PAYLOAD + 1];
    frame[0] = PROTO_H0;
    frame[1] = PROTO_H1;
    frame[2] = (uint8_t)len;
    if (len > 0) {
        memcpy(frame + 3, payload, len);
    }
    frame[3 + len] = frame_crc((uint8_t)len, payload);
    const size_t frame_len = 3 + len + 1;
    bool sent = true;
    if (route == PROTO_ROUTE_BLE) {
        bool online;
        sent = ble_generation(&online) == generation && online && ble_tuning_send(frame, frame_len, generation);
    } else {
        /* USB Serial/JTAG enqueues this entire frame or returns zero. */
        sent = proto_transport_write(frame, frame_len, ticks) == (int)frame_len;
    }

    xSemaphoreGiveRecursive(s_tx_lock);
    return sent;
}

static bool proto_send_payload_wait(const uint8_t *payload, size_t len, TickType_t ticks)
{
    return proto_send_payload_to(s_reply_route, s_reply_ble_generation, payload, len, ticks);
}

static bool proto_send_payload(const uint8_t *payload, size_t len)
{
    return proto_send_payload_wait(payload, len, pdMS_TO_TICKS(PROTO_WRITE_TIMEOUT_MS));
}

static void proto_send_ack(uint8_t msg_id, uint8_t status)
{
    const uint8_t p[3] = { msg_id, MSG_ACK, status };
    proto_send_payload(p, sizeof(p));
}

static void proto_send_error(uint8_t msg_id, uint8_t code, uint8_t detail)
{
    const uint8_t p[4] = { msg_id, MSG_ERROR, code, detail };
    proto_send_payload(p, sizeof(p));
}

static void proto_send_pong(void)
{
    const uint8_t p[1] = { MSG_PONG };
    proto_send_payload(p, sizeof(p));
}

/* Best-effort log frame: dropped when the host is idle or the UART is busy. */
static void proto_send_log_text(const char *text)
{
    if (!s_host_active || text == NULL || text[0] == '\0') {
        return;
    }
    if (s_tx_lock == NULL || xSemaphoreTakeRecursive(s_tx_lock, 0) != pdTRUE) {
        return;   /* never block an audio/logging task for a diagnostic line */
    }

    /* Unsolicited task logs are USB-only. Sending every diagnostic line over
     * BLE would consume Classic radio/BTU time even when nobody is tuning. */
    if (s_host_active && !s_bulk_active) {
        size_t len = strlen(text);
        size_t off = 0;
        while (off < len) {
            size_t chunk = len - off;
            if (chunk > PROTO_LOG_CHUNK) {
                chunk = PROTO_LOG_CHUNK;
            }
            uint8_t p[PROTO_LOG_CHUNK + 1];
            p[0] = MSG_LOG;
            memcpy(p + 1, text + off, chunk);
            /* Already own the recursive lock. USB logging must never wait
             * on a full TX ring from a decoder or audio control task. */
            if (!proto_send_payload_to(PROTO_ROUTE_UART, 0, p, chunk + 1, 0)) break;
            off += chunk;
        }
    }

    xSemaphoreGiveRecursive(s_tx_lock);
}

void serial_proto_log(const char *text)
{
    proto_send_log_text(text);
}

bool serial_proto_host_active(void)
{
    return s_host_active;
}

/* ---------------- Monitor sampling ---------------- */
/* Output clock error vs the nominal rate.
 *
 * The only clock-domain counter the firmware owns is the pump tick (one I2S
 * DMA descriptor = I2S_OUT_DMA_FRAME_NUM frames), so the measurement is
 * quantised by that granularity and is only valid while the pump actually
 * runs: boot, I2S reconfig and NVS flash writes all stall the DMA and would
 * bias a free-running average low. The window therefore starts on a stream
 * open, accumulates only while streaming, and reports its ratio once it is
 * long enough (resolution ~ 5000 / window_seconds ppm; 30 s -> ~170 ppm at
 * 96k). The logic analyser remains the precise reference. */
static uint64_t s_ck_frames;
static int64_t  s_ck_us;
static uint32_t s_ck_rate;
static uint32_t s_ck_last_tick;
static int64_t  s_ck_last_us;
static bool     s_ck_streaming;

static void proto_sample_clock(void)
{
    const uint32_t tick = uac2_audio_get_pump_tick();
    const int64_t now = esp_timer_get_time();
    const uint32_t nominal = uac2_audio_get_output_rate();
    const bool streaming = uac2_audio_is_streaming();

    if (nominal != s_ck_rate || (streaming && !s_ck_streaming)) {
        /* Start a fresh window at this instant. The interval that just ended
         * covered the reconfiguration, when the pump was stalled, so it must
         * not be counted. */
        s_ck_rate = nominal;
        s_ck_frames = 0;
        s_ck_us = 0;
        s_ck_last_tick = tick;
        s_ck_last_us = now;
        s_ck_streaming = streaming;
        return;
    }
    s_ck_streaming = streaming;

    if (streaming && s_ck_last_us != 0) {
        const uint32_t dtick = tick - s_ck_last_tick;
        const int64_t dus = now - s_ck_last_us;
        if (dtick > 0 && dus > 0) {
            s_ck_frames += (uint64_t)dtick * (uint64_t)I2S_OUT_DMA_FRAME_NUM;
            s_ck_us += dus;
        }
    }
    s_ck_last_tick = tick;
    s_ck_last_us = now;

    if (s_ck_us >= 10000000 && nominal > 0) {   /* 10 s minimum window */
        const double rate = (double)s_ck_frames * 1000000.0 / (double)s_ck_us;
        s_clock_ppm = (float)((rate - (double)nominal) * 1000000.0 / (double)nominal);
    }
}

#if CONFIG_FREERTOS_GENERATE_RUN_TIME_STATS
static void proto_sample_cpu(void)
{
    /* Read the two idle-task run-time counters directly. The previous version
     * called uxTaskGetSystemState(), which walks every task inside a critical
     * section - measured at 800-976 us with interrupts disabled on this core.
     * That starved the USB ISO OUT ISR and dropped ~30 audio samples once per
     * call (one 30-sample gap per 1.001 s, audible as a tick). Reading a single
     * TCB field per core costs microseconds and never disturbs the audio. */
    const uint32_t total = (uint32_t)portGET_RUN_TIME_COUNTER_VALUE();
    const uint32_t dt = total - s_total_prev;
    s_total_prev = total;

    for (int core = 0; core < 2; core++) {
        const uint32_t idle_now = (uint32_t)ulTaskGetIdleRunTimeCounterForCore((BaseType_t)core);
        if (dt == 0) {
            s_idle_prev[core] = idle_now;
            continue;
        }
        const uint32_t idle_delta = idle_now - s_idle_prev[core];
        uint32_t load = (idle_delta >= dt) ? 0u
                      : (uint32_t)(((uint64_t)(dt - idle_delta) * 100u) / dt);
        s_cpu_load[core] = (uint8_t)(load > 100u ? 100u : load);
        s_idle_prev[core] = idle_now;
    }
}
#else
static void proto_sample_cpu(void)
{
    s_cpu_load[0] = 0;
    s_cpu_load[1] = 0;
}
#endif

static void proto_put_u32(uint8_t *dst, uint32_t v)
{
    dst[0] = (uint8_t)v;
    dst[1] = (uint8_t)(v >> 8);
    dst[2] = (uint8_t)(v >> 16);
    dst[3] = (uint8_t)(v >> 24);
}

/* v38（阶段 C1）：HW_STATUS 帧里有两个 u16（延时 A/B），按同样的**小端**口径写。 */
static void proto_put_u16(uint8_t *dst, uint16_t v)
{
    dst[0] = (uint8_t)v;
    dst[1] = (uint8_t)(v >> 8);
}

static void proto_put_f32(uint8_t *dst, float v)
{
    uint32_t u = 0;
    memcpy(&u, &v, 4);
    proto_put_u32(dst, u);
}

static void proto_send_telemetry(void)
{
    dsp_audio_stats_t ds = { 0 };
    dsp_audio_take_stats(&ds);

    uint32_t fill = 0;
    if (uac2_audio_get_output_source() == UAC2_AUDIO_SOURCE_BT) {
        a2dp_audio_stats_t ast = { 0 };
        a2dp_audio_get_stats(&ast);
        if (ast.capacity_bytes > 0) {
            fill = (uint32_t)ast.level_bytes * 100u / ast.capacity_bytes;
        }
    } else {
        fill = uac2_audio_get_fifo_fill_percent();
    }

    uint8_t p[25];
    p[0] = MSG_TELEMETRY;
    proto_put_u32(p + 1, ds.min_us);
    proto_put_u32(p + 5, ds.max_us);
    proto_put_u32(p + 9, ds.avg_us);
    proto_put_u32(p + 13, ds.blocks);
    p[17] = (uint8_t)(fill > 100 ? 100 : fill);
    /* [18..20] 原是 3 个保留字节（恒 0，旧前端不读）—— v21 起启用：
     *   p[18] = 当前输出源：0 = USB(UAC2) / 1 = 蓝牙(A2DP)，与 WebUI 的输入索引一致；
     *   p[19] = 状态位：bit0 当前源正在放音 / bit1 蓝牙已连接 / bit2 蓝牙正在推流；
     *   p[20] = **参数编码版本**（v24 起）：见 dsp_config.h 的 DSP_PROTO_VERSION。
     *           v1 = 初版；v2 = `0x7E` 压缩比由 u8 改 u16。恒 ≥1，所以旧固件的 0
     *           可被客户端识别为"太老、没有这个字段"。
     * 前端顶栏的「当前源」就靠 p[18] 点亮（旧固件该字节为 0，前端显示 USB，
     * 与"未连接时多数场景确实是 USB"的默认一致，向后兼容）。 */
    p[18] = (uint8_t)uac2_audio_get_output_source();
    p[19] = (uint8_t)((uac2_audio_is_streaming() ? 0x01 : 0x00) |
                      (a2dp_audio_is_connected() ? 0x02 : 0x00) |
                      (a2dp_audio_is_streaming() ? 0x04 : 0x00));
    p[20] = (uint8_t)DSP_PROTO_VERSION;
    proto_put_f32(p + 21, s_clock_ppm);
    proto_send_payload(p, sizeof(p));

    /* Core0 retains the same consumed snapshot; the A4 payload stays unchanged.
     * For consecutive windows next_end - end == next_n; n=0 proves no budget.
     * Stage maxima are separate observations; total is the full wall maximum. */
    ESP_LOGI(TAG, "c1slice: end=%lu n=%lu prep=%lu q=%lu dsp=%lu port=%lu total=%lu "
                  "uac=%lu bt=%lu sil=%lu byp=%lu fin=%lu fstage=%lu fundel=%lu "
                  "pend=%lu dl_us=%lu dm=%lu b90=%lu wrerr=%lu wrpart=%lu stale=%lu",
             (unsigned long)ds.blocks,
             (unsigned long)ds.win_blocks,
             (unsigned long)ds.win_prep_max_us,
             (unsigned long)ds.win_queue_max_us,
             (unsigned long)ds.win_dsp_max_us,
             (unsigned long)ds.win_port_max_us,
             (unsigned long)ds.win_total_max_us,
             (unsigned long)ds.uac_blocks,
             (unsigned long)ds.bt_blocks,
             (unsigned long)ds.silence_blocks,
             (unsigned long)ds.bypass_blocks,
             (unsigned long)ds.frames_in,
             (unsigned long)ds.frames_stage,
             (unsigned long)ds.frames_undelivered,
             (unsigned long)ds.stage_pending_bytes,
             (unsigned long)ds.deadline_us,
             (unsigned long)ds.deadline_miss,
             (unsigned long)ds.budget90_over,
             (unsigned long)ds.write_errors,
             (unsigned long)ds.write_partial,
             (unsigned long)ds.stale_epoch);

}

static void proto_send_sys_status(void)
{
    const esp_partition_t *app = esp_partition_find_first(ESP_PARTITION_TYPE_APP,
                                                          ESP_PARTITION_SUBTYPE_ANY, NULL);
    uint32_t flash_size = 0;
    esp_flash_get_size(NULL, &flash_size);

    const uint32_t cpu_hz = (uint32_t)esp_clk_cpu_freq();
    const uint32_t heap_free = (uint32_t)esp_get_free_heap_size();
    const uint32_t heap_total = (uint32_t)heap_caps_get_total_size(MALLOC_CAP_DEFAULT);

    /* 27B 原有字段 + 8B 扩展（固件版本 + DSP 滤波器负载），旧前端只看前 27B，向后兼容 */
    uint8_t p[35];
    p[0] = MSG_SYS_STATUS;
    p[1] = s_cpu_load[0];
    p[2] = s_cpu_load[1];
    proto_put_u32(p + 3, cpu_hz);
    proto_put_u32(p + 7, cpu_hz);   /* 两个字段都填同一个值：ESP32-S31 两核共用时钟源，
                                     * esp_clk_cpu_freq() 在当前核返回的就是全局 CPU 频率。
                                     * 协议保留两个字段只为布局兼容（旧前端读这两个偏移）。 */
    proto_put_u32(p + 11, heap_free);
    proto_put_u32(p + 15, heap_total);
    proto_put_u32(p + 19, app != NULL ? app->size : 0);   /* flash budget in use */
    proto_put_u32(p + 23, flash_size);
    p[27] = (uint8_t)DSP_FW_VERSION_MAJOR;
    p[28] = (uint8_t)DSP_FW_VERSION_MINOR;    p[29] = (uint8_t)(dsp_config_count_biquads() & 0xFF);
    p[30] = (uint8_t)((dsp_config_count_biquads() >> 8) & 0xFF);
    p[31] = (uint8_t)(dsp_config_biquad_limit() & 0xFF);
    p[32] = (uint8_t)((dsp_config_biquad_limit() >> 8) & 0xFF);
    p[33] = dsp_config_preset_mask();
    p[34] = (uint8_t)DSP_PRESET_SLOTS;
    proto_send_payload(p, sizeof(p));
}

/* ---------------- HW_STATUS（v38 / 固件 0.38，阶段 C1） ----------------
 * 主机发 `0xAF`（单字节请求），设备回 `0xAF` 一帧 **20 B**：
 *
 *   [0]     = 0xAF（帧 id）
 *   [1]     = 版本 = 1
 *   [2]     = muted          现在是否处于"静音"状态（0/1）
 *   [3]     = pin_level      **GPIO8 实测电平**（0/1）—— muted 与极性的共同结果
 *   [4]     = playing        状态机判定的"正在播放"（0/1）
 *   [5]     = polarity       0 = 低电平静音 / 1 = 高电平静音
 *   [6..7]  = delay_a_ms     u16 小端（停止播放 → 静音）
 *   [8..9]  = delay_b_ms     u16 小端（开始播放 → 解除静音）
 *   [10..13]= transitions    u32 小端：自上电以来 mute 引脚**实际跳变**的次数
 *   [14..17]= since_us       u32 小端：距上一次跳变的微秒数（读的瞬间算，**不需要两机对时**）
 *   [18..21]= playing_since_us u32 小端：距**最近一次 `playing` 跳变**的微秒数
 *   [22..23] = 保留（0）
 *
 * ★ 为什么有 `playing_since_us`（v38 实测的教训）：主机侧的轮询窗口受串口
 * `f.read(4096)` 阻塞影响可达 ~300 ms，"起播 → 解除静音"只能给 ±300 ms 的区间，
 * **分不开 0 / 30 ms 两档**。把 playing 边沿也放到设备侧之后，两个时刻出自**同一帧**：
 *     间隔 = playing_since_us − since_us
 * 与主机时钟/轮询周期/串口往返**全都无关**，精度 = 状态机周期（10 ms）。 */
static void proto_send_hw_status(void)
{
    uint8_t p[24];
    memset(p, 0, sizeof(p));
    p[0]  = MSG_HW_STATUS;
    p[1]  = 2;                       /* 帧版本 2 = 增加了 playing_since_us */
    p[2]  = hw_mute_is_muted() ? 1 : 0;
    p[3]  = (uint8_t)(hw_mute_pin_level() ? 1 : 0);
    p[4]  = hw_mute_is_playing() ? 1 : 0;
    p[5]  = hw_mute_polarity();
    proto_put_u16(p + 6, hw_mute_delay_a_ms());
    proto_put_u16(p + 8, hw_mute_delay_b_ms());
    proto_put_u32(p + 10, hw_mute_transitions());
    proto_put_u32(p + 14, hw_mute_since_last_change_us());
    proto_put_u32(p + 18, hw_mute_playing_since_us());
    proto_send_payload(p, sizeof(p));
}

/* 0.131 UAC_INPUT_PROBE (all words little-endian):
 * [B6] query; [B6,1] arm / [B6,2] disarm -> ordinary [B6,80,status] ACK.
 * ACK only accepts the command; poll query until requested_seq==applied_seq.
 * Status: [B6,1,0,flags] + 14 words in the order below + threshold + capacity.
 * flags bit0 armed, bit1 command pending. An armed snapshot is in-progress;
 * use the acknowledged disarmed snapshot for final counts/coverage.
 * [B6,3,index] reads a retained event after disarm (no consuming/resetting):
 * [B6,1,3,index] + arm_seq + us,frame,prevL,prevR,curL,curR,block,flags.
 * Unknown index or an armed/pending tape returns ERROR, never an empty event.
 * Controls touch neither audio samples nor NVS; 0xAF HW_STATUS is unchanged. */
static void proto_send_input_probe_status(void)
{
    uac2_input_probe_stats_t s;
    uac2_audio_input_probe_get_stats(&s);
    const uint32_t words[] = {
        s.requested_seq, s.applied_seq, s.arm_seq,
        s.start_us, s.stop_us, s.last_pcm_us,
        s.ticks, s.blocks, s.frames, s.idle_ticks, s.unsupported_ticks,
        s.events, s.dropped, s.stored,
        UAC2_INPUT_PROBE_THRESHOLD, UAC2_INPUT_PROBE_CAPACITY,
    };
    uint8_t p[4 + sizeof(words)];
    p[0] = MSG_UAC_INPUT_PROBE;
    p[1] = 1;
    p[2] = 0;
    p[3] = (s.armed ? 1u : 0u) | (s.requested_seq != s.applied_seq ? 2u : 0u);
    for (size_t i = 0; i < sizeof(words) / sizeof(words[0]); i++)
        proto_put_u32(p + 4 + 4 * i, words[i]);
    proto_send_payload(p, sizeof(p));
}

static void proto_send_input_probe_event(uint8_t index)
{
    uint32_t arm_seq;
    uac2_input_probe_event_t e;
    if (!uac2_audio_input_probe_get_event(index, &arm_seq, &e)) {
        proto_send_error(MSG_UAC_INPUT_PROBE, DSP_STATUS_ERR_UNSUPPORTED, index);
        return;
    }
    const uint32_t words[] = {
        arm_seq, e.us, e.frame, (uint32_t)e.prev_l, (uint32_t)e.prev_r,
        (uint32_t)e.cur_l, (uint32_t)e.cur_r, e.block, e.flags,
    };
    uint8_t p[4 + sizeof(words)];
    p[0] = MSG_UAC_INPUT_PROBE;
    p[1] = 1;
    p[2] = 3;
    p[3] = index;
    for (size_t i = 0; i < sizeof(words) / sizeof(words[0]); i++)
        proto_put_u32(p + 4 + 4 * i, words[i]);
    proto_send_payload(p, sizeof(p));
}

/* 0.132 RX_POST_ENQUEUE 第二份 tape —— 同一个 0xB6 代次控制，旧 0/1/2/3 格式**逐字保留**。
 *
 * [B6,4] -> status: [B6,1,4,flags] + 19 words (76 B)，整帧 80 B。词序：
 *   requested_seq, applied_seq, arm_seq, start_us, stop_us, last_rx_us,
 *   packets, frames, gaps, unsupported, null_ff, rebase, delta_ne, rejected,
 *   events, dropped, stored, threshold, capacity
 *   flags bit0 = 本代次仍在观察（pump 已应用 ARM 且未 DISARM）；
 *         bit1 = 本代次**尚未建立覆盖**（packets == 0：ARM 后还没有成功入环包）。
 * [B6,5,index] -> event: [B6,1,5,index] + 9 words (36 B)，整帧 40 B。词序：
 *   rx_arm_seq, us, rx_frame, prevL, prevR, curL, curR, packet, flags
 *   索引无效、或该条目属于**别的 ARM 代次** ⇒ 显式 ERROR（绝不返回混代次/空事件）。
 *
 * 单位与口径：us = RX 回调时间（esp_timer µs，非样本到达时刻）；rx_frame = 本代次
 * 已扫描输入帧的 0 基索引；packet = 本代次 1 基包号；gaps = 覆盖缺口数
 * （帧不对齐 / NULL FIFO / 整包拒收 / rebase / delta≠n），缺口处**断开 prev**，
 * 不跨缺口制造事件。这是 **RX_POST_ENQUEUE**：不是 PRE_FIFO / USB_WIRE / HOST_SUBMITTED。
 * 旧版设备（0.131）对 4/5 明确返回 ERROR(LENGTH)，不会返回空成功。
 * 控制不写音频样本、不进 NVS；0xAF HW_STATUS 与 0/1/2/3 行为均不变。 */
static void proto_send_rx_content_status(void)
{
    uac2_rx_content_stats_t s;
    uac2_audio_rx_content_get_stats(&s);
    const uint32_t words[] = {
        s.requested_seq, s.applied_seq, s.arm_seq,
        s.start_us, s.stop_us, s.last_rx_us,
        s.packets, s.frames, s.gaps,
        s.unsupported, s.null_ff, s.rebase, s.delta_ne, s.rejected,
        s.events, s.dropped, s.stored,
        UAC2_RX_CONTENT_THRESHOLD, UAC2_RX_CONTENT_CAPACITY,
    };
    uint8_t p[4 + sizeof(words)];
    p[0] = MSG_UAC_INPUT_PROBE;
    p[1] = 1;
    p[2] = 4;
    p[3] = (s.armed ? 1u : 0u) | (s.packets == 0u ? 2u : 0u);
    for (size_t i = 0; i < sizeof(words) / sizeof(words[0]); i++)
        proto_put_u32(p + 4 + 4 * i, words[i]);
    proto_send_payload(p, sizeof(p));
}

static void proto_send_rx_content_event(uint8_t index)
{
    uint32_t arm_seq;
    uac2_rx_content_event_t e;
    if (!uac2_audio_rx_content_get_event(index, &arm_seq, &e)) {
        proto_send_error(MSG_UAC_INPUT_PROBE, DSP_STATUS_ERR_UNSUPPORTED, index);
        return;
    }
    const uint32_t words[] = {
        /* getter 只在条目代次与当前代次一致时成功，并回填该条目自带的代次。 */
        arm_seq, e.us, e.rx_frame, (uint32_t)e.prev_l, (uint32_t)e.prev_r,
        (uint32_t)e.cur_l, (uint32_t)e.cur_r, e.packet, e.flags,
    };
    uint8_t p[4 + sizeof(words)];
    p[0] = MSG_UAC_INPUT_PROBE;
    p[1] = 1;
    p[2] = 5;
    p[3] = index;
    for (size_t i = 0; i < sizeof(words) / sizeof(words[0]); i++)
        proto_put_u32(p + 4 + 4 * i, words[i]);
    proto_send_payload(p, sizeof(p));
}

/* ---------------- BT_STATUS（v25 / 固件 0.25） ----------------
 * 主机发 `0xB0`（控制帧，单字节或两字节都接受），设备回 `0xB1` 一帧 40 B。
 * 数据全部来自 `a2dp_audio_get_stats()` + 现有状态函数，**不新增任何蓝牙侧采集**。
 *
 * 为什么值得做：项目定位是"UAC2 **+ A2DP** 音频桥"，但在此之前**蓝牙侧一个字都读不出来** ——
 * 前端只能显示"蓝牙已连接"这一个位。有了这一帧，前端就能显示当前编解码/采样率/水位/丢包。
 *
 * ⚠ 与遥测帧一样：**水位百分比用 level/capacity 现算**，不要另存一份（会不一致）。 */
static void proto_send_bt_status(void)
{
    a2dp_audio_stats_t st = { 0 };
    a2dp_audio_get_stats(&st);

    uint32_t fill = 0;
    if (st.capacity_bytes > 0) {
        fill = (uint32_t)((st.level_bytes * 100u) / st.capacity_bytes);
        if (fill > 100u) fill = 100u;
    }

    /* 布局（48 B = 1 类型 + 47 载荷）：
     *   [0] 0xB1  [1] 载荷版本=1  [2] flags  [3] codec  [4..7] rate
     *   [8] ch  [9] fill  [10..13] duration_ms  [14..17] level
     *   [18..21] capacity  [22..25] min  [26..29] max
     *   [30..33] packets  [34..37] drops  [38..41] ring_overflows  [42..45] flushes
     *   [46] 预留（0） */
    uint8_t p[48];
    memset(p, 0, sizeof(p));
    p[0] = MSG_BT_STATUS;
    p[1] = 1;                                        /* 载荷版本 */
    p[2] = (uint8_t)((a2dp_audio_is_connected() ? 0x01 : 0) |
                     (a2dp_audio_is_streaming() ? 0x02 : 0) |
                     (a2dp_audio_is_decoder_ready() ? 0x04 : 0) |
                     (a2dp_audio_is_codec_busy() ? 0x08 : 0));
    p[3] = st.codec_type;                            /* esp_a2d_mct_t：0=SBC / 2=M24(AAC) / 0xFF=NON_A2DP */
    proto_put_u32(p + 4, st.sample_rate);
    p[8] = st.channels;
    p[9] = (uint8_t)fill;
    proto_put_u32(p + 10, st.duration_ms);
    proto_put_u32(p + 14, (uint32_t)st.level_bytes);
    proto_put_u32(p + 18, (uint32_t)st.capacity_bytes);
    proto_put_u32(p + 22, (uint32_t)st.min_level_bytes);
    proto_put_u32(p + 26, (uint32_t)st.max_level_bytes);
    proto_put_u32(p + 30, st.packets);
    proto_put_u32(p + 34, st.drops);
    proto_put_u32(p + 38, st.ring_overflows);
    proto_put_u32(p + 42, st.flushes);
    proto_send_payload(p, sizeof(p));
}

/* ---------------- BULK_CONFIG ---------------- */
static void proto_send_meters(void)
{
    uint32_t pl = 0, pr = 0;
    dsp_audio_take_peaks(&pl, &pr);
    float gr = dsp_engine_get_limiter_gr_db();
    if (gr < 0.0f) gr = 0.0f;
    uint32_t gr_mdb = (uint32_t)(gr * 100.0f + 0.5f);
    if (gr_mdb > 0xFFFFu) gr_mdb = 0xFFFFu;

    uint8_t p[17];
    p[0] = MSG_METERS;
    p[1] = 1;                                        /* 载荷版本 */
    p[2] = (uint8_t)((dsp_engine_is_bypass() ? 0x01 : 0) |
                     (dsp_engine_get_limiter_gr_db() > 0.0f ? 0x02 : 0));
    p[3] = (uint8_t)(gr_mdb & 0xFF);
    p[4] = (uint8_t)((gr_mdb >> 8) & 0xFF);
    proto_put_u32(p + 5, pl);
    proto_put_u32(p + 9, pr);
    proto_put_u32(p + 13, dsp_engine_get_clips());
    proto_send_payload(p, sizeof(p));
}

static void proto_send_bulk(void)
{
    const uint32_t fs = uac2_audio_get_output_rate();
    size_t total = dsp_config_serialize(s_bulk, sizeof(s_bulk), fs);
    if (total == 0) {
        proto_send_error(0, ERR_BULK_TOO_BIG, 0);
        return;
    }

    s_bulk_active = true;

    uint8_t first[4 + PROTO_BULK_FIRST_DATA];
    first[0] = 0x00;                        /* msg id (unused by the WebUI) */
    first[1] = MSG_BULK_CONFIG;
    first[2] = (uint8_t)(total & 0xFF);
    first[3] = (uint8_t)((total >> 8) & 0xFF);

    size_t chunk = (total > PROTO_BULK_FIRST_DATA) ? PROTO_BULK_FIRST_DATA : total;
    memcpy(first + 4, s_bulk, chunk);
    if (!proto_send_payload(first, 4 + chunk)) {
        s_bulk_active = false;
        return;
    }

    size_t off = chunk;
    while (off < total) {
        size_t n = total - off;
        if (n > PROTO_MAX_PAYLOAD) {
            n = PROTO_MAX_PAYLOAD;
        }
        if (!proto_send_payload(s_bulk + off, n)) {
            s_bulk_active = false;
            return;
        }
        off += n;
    }

    s_bulk_active = false;

    char msg[128];
    snprintf(msg, sizeof(msg), "config sync: %u bytes, fs=%lu Hz, %u biquads%s",
             (unsigned)total, (unsigned long)fs,
             (unsigned)dsp_engine_active_filter_count(),
             dsp_engine_is_bypass() ? ", bypass (bit-perfect)" : "");
    proto_send_log_text(msg);
}

/* ---------------- FILTER_DUMP ----------------
 * payload: [0xA7, total_lo, total_hi, chunk, n, (kind,a,b,idx, b0,b1,b2,a1,a2)*n] */
static void proto_send_filter_dump(void)
{
    /* 128 太小：48 kHz 下合法配置最多 138 个双二阶 + 2 个响度架式（140 条），
     * 旧的 128 会静默截断（帧里上报的 total 就是 128，主机无法察觉）。 */
    static dsp_filter_dump_t filters[160];
    const size_t total = dsp_engine_collect_filters(filters, sizeof(filters) / sizeof(filters[0]));

    const size_t per_frame = 10;   /* 5 + 10*24 = 245 bytes */
    size_t off = 0;
    uint8_t chunk = 0;
    do {
        const size_t n = (total - off > per_frame) ? per_frame : (total - off);
        uint8_t p[5 + per_frame * 24];
        p[0] = MSG_FILTER_DUMP;
        p[1] = (uint8_t)(total & 0xFF);
        p[2] = (uint8_t)((total >> 8) & 0xFF);
        p[3] = chunk;
        p[4] = (uint8_t)n;
        uint8_t *q = p + 5;
        for (size_t i = 0; i < n; i++) {
            const dsp_filter_dump_t *f = &filters[off + i];
            *q++ = f->kind;
            *q++ = f->a;
            *q++ = f->b;
            *q++ = f->idx;
            proto_put_f32(q, f->c.b0); q += 4;
            proto_put_f32(q, f->c.b1); q += 4;
            proto_put_f32(q, f->c.b2); q += 4;
            proto_put_f32(q, f->c.a1); q += 4;
            proto_put_f32(q, f->c.a2); q += 4;
        }
        proto_send_payload(p, 5 + n * 24);
        off += n;
        chunk++;
    } while (off < total);
}

/* ---------------- Frame dispatch ---------------- */
static void proto_handle_param_update(const uint8_t *p, size_t len)
{
    const uint8_t msg_id = p[0];
    if (len < 6) {
        proto_send_error(msg_id, DSP_STATUS_ERR_LENGTH, (uint8_t)len);
        return;
    }

    const uint8_t block = p[2];
    const uint8_t ch = p[3];
    const uint8_t param = p[4];
    const uint8_t idx = p[5];
    dsp_status_t st = dsp_config_set_param(block, ch, param, idx, p + 6, len - 6);
    proto_send_ack(msg_id, (uint8_t)st);
}

static bool proto_is_amp_command(const uint8_t *p, size_t len)
{
    return len && p[0] == MSG_AMP_CONTROL &&
        !(len >= 6 && p[1] == MSG_PARAM_UPDATE);
}

static void proto_handle_payload(const uint8_t *p, size_t len)
{
    if (len == 0) {
        return;
    }

    /* 参数帧： [msgId, 0x01, block, ch, param, idx, value…] —— 最短 7 字节（u8 值），
     * 最长 10 字节（f32 值）。控制请求最长 3 字节（含 [0xB6,3,index]）。
     * 长度是唯一可靠的分辨依据：只看 p[1]==0x01 会把预设槽 1 的存/载/清
     * （0xAA/0xAB/0xAC + slot=1）误判成参数帧，导致预设第 2 槽永远不可用。 */
    /* DSP message IDs may also equal B8. GET JSON is exactly five bytes;
     * the legacy parameter envelope is at least seven bytes. */
    if (proto_is_amp_command(p, len)) {
        uint8_t reply[255];
        const uint64_t session = ((uint64_t)s_control_session << 32) |
            (s_reply_route == PROTO_ROUTE_BLE ? s_reply_ble_generation : 0);
        const size_t count = amp_control_request(p, len, session, reply);
        proto_send_payload(reply, count);
        return;
    }
    if (len >= 6 && p[1] == MSG_PARAM_UPDATE) {
        proto_handle_param_update(p, len);
        return;
    }

    /* Control requests have exact lengths; garbage appended to SAVE/RESET
     * must not accidentally turn into a persistent/configuration operation. */
    switch (p[0]) {
    case MSG_PRESET_SAVE: case MSG_PRESET_LOAD: case MSG_PRESET_CLEAR:
        if (len != 2) { proto_send_error(p[0], DSP_STATUS_ERR_LENGTH, (uint8_t)len); return; }
        break;
    case MSG_DSP_PATH_TEST: case MSG_UAC_INPUT_PROBE: case MSG_LOG:
        break; /* validated by the variable-length handler below */
    default:
        if (len != 1) { proto_send_error(p[0], DSP_STATUS_ERR_LENGTH, (uint8_t)len); return; }
        break;
    }

    switch (p[0]) {
    case MSG_PING:
        proto_send_pong();
        break;

    case MSG_SYNC_CONFIG:
        proto_send_bulk();
        break;

    case MSG_SAVE_CONFIG: {
        esp_err_t err = dsp_config_save();
        proto_send_ack(p[0], (err == ESP_OK) ? DSP_STATUS_OK : ERR_SAVE_FAILED);
        if (err == ESP_OK) {
            proto_send_log_text("config saved to NVS");
        } else {
            proto_send_log_text("config save FAILED");
        }
        break;
    }

    case MSG_SYS_STATUS:
        proto_send_sys_status();
        break;

    case MSG_HW_STATUS:
        proto_send_hw_status();
        break;

    case MSG_BT_STATUS_REQ:
        proto_send_bt_status();
        break;

    case MSG_DSP_PATH_TEST: {
        if (len != 1 && (len != 2 || p[1] > 1)) {
            proto_send_error(p[0], DSP_STATUS_ERR_LENGTH, (uint8_t)len);
            break;
        }
        if (len == 2) {
            uac2_audio_set_dsp_test_bypass(p[1] != 0);
        }
        const uint8_t reply[] = {MSG_DSP_PATH_TEST, 1,
                                uac2_audio_get_dsp_test_bypass() ? 1 : 0};
        proto_send_payload(reply, sizeof(reply));
        break;
    }

    case MSG_UAC_INPUT_PROBE:
        if (len == 1) {
            proto_send_input_probe_status();
        } else if (len == 2 && (p[1] == 1 || p[1] == 2)) {
            const bool accepted = uac2_audio_input_probe_request(p[1] == 1);
            proto_send_ack(p[0], accepted ? DSP_STATUS_OK : DSP_STATUS_ERR_UNSUPPORTED);
        } else if (len == 3 && p[1] == 3) {
            proto_send_input_probe_event(p[2]);
        } else if (len == 2 && p[1] == 4) {
            proto_send_rx_content_status();      /* 0.132 RX_POST_ENQUEUE stats */
        } else if (len == 3 && p[1] == 5) {
            proto_send_rx_content_event(p[2]);   /* 0.132 RX_POST_ENQUEUE event */
        } else if (len == 2 && p[1] == 6) {
            const uint32_t mode = uac2_audio_get_legacy_diagnostics();
            const uint8_t reply[] = {MSG_UAC_INPUT_PROBE, 6, 1,
                (uint8_t)(mode & 1u), (uint8_t)mode, (uint8_t)(mode >> 8),
                (uint8_t)(mode >> 16), (uint8_t)(mode >> 24)};
            proto_send_payload(reply, sizeof(reply));
        } else if (len == 3 && p[1] == 7 && p[2] <= 1) {
            uac2_audio_set_legacy_diagnostics(p[2] != 0);
            proto_send_ack(p[0], DSP_STATUS_OK);
        } else {
            proto_send_error(p[0], DSP_STATUS_ERR_LENGTH, (uint8_t)len);
        }
        break;

    case MSG_TELEMETRY:
        proto_send_telemetry();
        break;

    case MSG_FILTER_DUMP:
        proto_send_filter_dump();
        break;

    case MSG_RESET_CONFIG:
        dsp_config_reset_defaults();
        dsp_config_apply_all();
        proto_send_ack(p[0], DSP_STATUS_OK);
        proto_send_log_text("config reset to defaults");
        break;

    case MSG_DSP_SELFTEST:
        /* DSP tests use real input/output; core0 must not run the engine. */
        proto_send_ack(p[0], DSP_STATUS_ERR_UNSUPPORTED);
        break;

    case MSG_PRESET_SAVE: {
        if (len < 2 || p[1] >= DSP_PRESET_SLOTS) {
            proto_send_error(p[0], DSP_STATUS_ERR_PRESET, len < 2 ? 0 : p[1]);
            break;
        }
        esp_err_t err = dsp_config_preset_save(p[1]);
        proto_send_ack(p[0], (err == ESP_OK) ? DSP_STATUS_OK : ERR_SAVE_FAILED);
        if (err == ESP_OK) {
            char msg[48];
            snprintf(msg, sizeof(msg), "preset %u saved (mask=0x%02X)", (unsigned)p[1],
                     (unsigned)dsp_config_preset_mask());
            proto_send_log_text(msg);
        }
        break;
    }

    case MSG_PRESET_LOAD: {
        if (len < 2 || p[1] >= DSP_PRESET_SLOTS) {
            proto_send_error(p[0], DSP_STATUS_ERR_PRESET, len < 2 ? 0 : p[1]);
            break;
        }
        esp_err_t err = dsp_config_preset_load(p[1]);
        /* 载入失败要区分原因：超出双二阶预算 -> status=7（前端文案"超出 DSP 滤波器上限"），
         * 其余（槽空 / blob 损坏 / 版本不符）-> status=8。 */
        dsp_status_t st = (err == ESP_OK) ? DSP_STATUS_OK
                        : (err == ESP_ERR_NOT_SUPPORTED) ? DSP_STATUS_ERR_LIMIT
                        : DSP_STATUS_ERR_PRESET;
        proto_send_ack(p[0], (uint8_t)st);
        if (err == ESP_OK) {
            char msg[48];
            snprintf(msg, sizeof(msg), "preset %u loaded", (unsigned)p[1]);
            proto_send_log_text(msg);
            proto_send_bulk();          /* 主机界面自动刷新 */
        }
        break;
    }

    case MSG_PRESET_CLEAR: {
        if (len < 2 || p[1] >= DSP_PRESET_SLOTS) {
            proto_send_error(p[0], DSP_STATUS_ERR_PRESET, len < 2 ? 0 : p[1]);
            break;
        }
        esp_err_t err = dsp_config_preset_clear(p[1]);
        proto_send_ack(p[0], (err == ESP_OK) ? DSP_STATUS_OK : ERR_SAVE_FAILED);
        break;
    }

    case MSG_PRESET_INFO: {
        uint8_t p2[3];
        p2[0] = MSG_PRESET_INFO;
        p2[1] = (uint8_t)DSP_PRESET_SLOTS;
        p2[2] = dsp_config_preset_mask();
        proto_send_payload(p2, sizeof(p2));
        break;
    }

    case MSG_METERS:
        proto_send_meters();
        break;

    case MSG_LOG:
        break;   /* host -> device log echo: ignore */

    default:
        proto_send_error(p[0], ERR_UNKNOWN_MSG, 0);
        break;
    }
}

/* ---------------- Session handling ---------------- */
static void proto_session_begin(void)
{
    ++s_control_session;
    s_host_active = true;
#if CONFIG_FREERTOS_GENERATE_RUN_TIME_STATS
    s_total_prev = (uint32_t)portGET_RUN_TIME_COUNTER_VALUE();
    for (unsigned core = 0; core < 2; ++core)
        s_idle_prev[core] = ulTaskGetIdleRunTimeCounterForCore(core);
#endif
    proto_send_log_text("host protocol session started (AA 55 / CRC8)");
}

static void proto_session_end(void)
{
    amp_control_abort_upload();
    uac2_audio_set_dsp_test_bypass(false);
    s_host_active = false;
    s_bulk_active = false;
    s_reply_route = PROTO_ROUTE_UART;
    s_parser_uart.state = RX_H0;
    s_parser_ble.state = RX_H0;
    /* s_host_active is already false, so this line is emitted as raw text. */
    ESP_LOGI(TAG, "host idle %d ms, raw log output resumed (rx frames=%lu crc_err=%lu)",
             CONFIG_SERIAL_PROTO_TIMEOUT_MS,
             (unsigned long)s_frames_rx, (unsigned long)s_crc_errors);
}

static void proto_feed(proto_parser_t *ps, uint8_t b)
{
    switch (ps->state) {
    case RX_H0:
        if (b == PROTO_H0) {
            ps->state = RX_H1;
        }
        break;

    case RX_H1:
        if (b == PROTO_H1) {
            ps->state = RX_LEN;
        } else if (b != PROTO_H0) {
            ps->state = RX_H0;
        }
        break;

    case RX_LEN:
        ps->len = b;
        ps->pos = 0;
        ps->state = (b == 0) ? RX_CRC : RX_PAYLOAD;
        break;

    case RX_PAYLOAD:
        ps->payload[ps->pos++] = b;
        if (ps->pos >= ps->len) {
            ps->state = RX_CRC;
        }
        break;

    case RX_CRC: {
        uint8_t expect = frame_crc(ps->len, ps->payload);
        ps->state = RX_H0;
        if (expect != b) {
            s_crc_errors++;
            return;
        }
        if (ps->len == 0) return; /* empty frames do not own/renew a host lease */

        const int64_t now = esp_timer_get_time();
        if (ps->route == PROTO_ROUTE_BLE && !proto_ble_generation_matches(ps->generation)) return;
        if (s_host_active && now - s_last_rx_us > (int64_t)CONFIG_SERIAL_PROTO_TIMEOUT_MS * 1000)
            proto_session_end();
        s_frames_rx++;
        if (s_host_active && s_reply_route != ps->route) {
            uint8_t busy[] = {ps->payload[0], MSG_ERROR, ERR_HOST_BUSY, 0};
            if (proto_is_amp_command(ps->payload, ps->len) && ps->len >= 3) {
                busy[1] = ps->payload[1] | 0x80;
                busy[2] = ps->payload[2];
                busy[3] = 3; /* B8 BUSY, not an ambiguous legacy ERROR. */
            }
            proto_send_payload_to(ps->route, ps->generation, busy, sizeof(busy),
                                  pdMS_TO_TICKS(PROTO_WRITE_TIMEOUT_MS));
            return;
        }
        /* The TX lock publishes the route atomically against task log frames.
         * No logging task can observe a half-changed owner. */
        if (xSemaphoreTakeRecursive(s_tx_lock, pdMS_TO_TICKS(200)) != pdTRUE) return;
        s_reply_route = ps->route;
        s_reply_ble_generation = ps->generation;
        s_last_rx_us = now;
        if (!s_host_active) proto_session_begin();
        xSemaphoreGiveRecursive(s_tx_lock);
        proto_handle_payload(ps->payload, ps->len);
        break;
    }

    default:
        ps->state = RX_H0;
        break;
    }
}

/* BTU is the sole producer. MessageBuffer accepts a complete GATT write or
 * none, avoiding successful ATT acknowledgement of a truncated AA55 frame. */
void proto_ble_link_changed(bool connected)
{
    portENTER_CRITICAL(&s_ble_mux);
    ++s_ble_generation; s_ble_online = connected;
    portEXIT_CRITICAL(&s_ble_mux);
}

bool proto_ble_generation_matches(uint32_t generation)
{
    bool online;
    return ble_generation(&online) == generation && online;
}

bool proto_feed_bytes(const uint8_t *data, size_t len)
{
    if (!data || !len || len > PROTO_BLE_WRITE_MAX || !s_ble_rx_mb) return false;
    ble_write_t write;
    bool online;
    write.generation = ble_generation(&online);
    if (!online) return false;
    memcpy(write.bytes, data, len);
    const size_t bytes = sizeof(write.generation) + len;
    if (xMessageBufferSend(s_ble_rx_mb, &write, bytes, 0) == bytes) return true;
    portENTER_CRITICAL(&s_ble_mux);
    s_ble_rx_lost += (uint32_t)len;
    ++s_ble_generation; /* invalidate any partly assembled request */
    portEXIT_CRITICAL(&s_ble_mux);
    return false;
}

/* ---------------- Log redirection ---------------- */
static int proto_vprintf(const char *fmt, va_list args)
{
    if (!s_host_active || xPortInIsrContext()) {
        return (s_prev_vprintf != NULL) ? s_prev_vprintf(fmt, args) : 0;
    }

    /* Logs stay on USB, including while BLE owns control. They never enqueue
     * GATT work or block audio/BTU for an unsubscribed/congested BLE link. */
    char buf[512];
    va_list copy;
    va_copy(copy, args);
    int n = vsnprintf(buf, sizeof(buf), fmt, copy);
    va_end(copy);

    if (n > 0) {
        size_t len = strlen(buf);
        while (len > 0 && (buf[len - 1] == '\n' || buf[len - 1] == '\r')) {
            buf[--len] = '\0';
        }
        /* 超长行必须**显式标记**，不能静默截断：diag 行已接近 300 字符，
         * 一旦被截断，主机看到的字段就"莫名少几个"，会被误判成"固件没编进去"。 */
        if ((size_t)n >= sizeof(buf)) {
            static const char mark[] = " …[TRUNC]";
            const size_t m = sizeof(mark) - 1;
            if (len > m) {
                len -= m;
            }
            memcpy(buf + len, mark, m + 1);
            len += m;
        }
        /* 行边界要随文本一起发：LOG 帧按 200B 分块，去掉换行后主机无法拆行，
         * 实测会把 "wr_dup=0" 和下一行的 "I (364067)" 粘成 "0I"。 */
        if (len > 0 && len + 2 < sizeof(buf)) {
            buf[len++] = '\n';
            buf[len] = '\0';
        }
        if (len > 0) {
            proto_send_log_text(buf);
        }
    }
    return n;
}

/* ---------------- Task ---------------- */
static void proto_task(void *arg)
{
    (void)arg;
    int64_t last_sample = 0;

    for (;;) {
        int n = proto_transport_read(s_rx_buf, sizeof(s_rx_buf), pdMS_TO_TICKS(50));
        for (int i = 0; i < n; i++) {
            proto_feed(&s_parser_uart, s_rx_buf[i]);
        }

        /* Invalidate partial frames and the old BLE lease on disconnect,
         * reconnect or RX overflow. No old peer's bytes reach a new peer. */
        static uint32_t consumed_generation;
        bool online;
        uint32_t generation = ble_generation(&online);
        if (generation != consumed_generation) {
            s_parser_ble.state = RX_H0;
            consumed_generation = generation;
            if (s_host_active && s_reply_route == PROTO_ROUTE_BLE &&
                s_reply_ble_generation != generation) proto_session_end();
        }
        if (s_ble_rx_mb != NULL) {
            static ble_write_t write;
            /* Bounded work per pass: BLE flooding cannot starve USB/timeout. */
            for (unsigned count = 0; count < 4; ++count) {
                const size_t got = xMessageBufferReceive(s_ble_rx_mb, &write, sizeof(write), 0);
                if (!got) break;
                generation = ble_generation(&online);
                if (generation != consumed_generation) {
                    s_parser_ble.state = RX_H0;
                    consumed_generation = generation;
                    if (s_host_active && s_reply_route == PROTO_ROUTE_BLE &&
                        s_reply_ble_generation != generation) proto_session_end();
                }
                if (got <= sizeof(write.generation) || !online || write.generation != generation) continue;
                s_parser_ble.generation = write.generation;
                for (size_t i = 0; i < got - sizeof(write.generation); ++i) {
                    bool current_online;
                    if (ble_generation(&current_online) != generation || !current_online) break;
                    proto_feed(&s_parser_ble, write.bytes[i]);
                }
            }
        }

        int64_t now = esp_timer_get_time();
        if (now - last_sample >= 1000000) {
            last_sample = now;
            proto_sample_clock();
            proto_sample_cpu();
            static uint32_t last_rx_lost, last_tx_lost;
            portENTER_CRITICAL(&s_ble_mux);
            const uint32_t rx_lost = s_ble_rx_lost;
            portEXIT_CRITICAL(&s_ble_mux);
            const uint32_t tx_lost = ble_tuning_dropped_bytes();
            if (rx_lost != last_rx_lost || tx_lost != last_tx_lost) {
                ESP_LOGW(TAG, "ble-transport: generation=%lu rx_rejected_b=%lu tx_unqueued_b=%lu",
                         (unsigned long)generation, (unsigned long)rx_lost, (unsigned long)tx_lost);
                last_rx_lost = rx_lost; last_tx_lost = tx_lost;
            }
            if (s_host_active) {
                proto_send_telemetry();
                if (now - s_last_rx_us > (int64_t)CONFIG_SERIAL_PROTO_TIMEOUT_MS * 1000) {
                    proto_session_end();
                }
            }
        }
    }
}

/* ---------------- Init ---------------- */
esp_err_t serial_proto_init(void)
{
#if !CONFIG_SERIAL_PROTO_ENABLE
    ESP_LOGW(TAG, "host protocol disabled by Kconfig");
    return ESP_OK;
#else
    if (s_tx_lock == NULL) {
        s_tx_lock = xSemaphoreCreateRecursiveMutex();
        if (s_tx_lock == NULL) {
            return ESP_ERR_NO_MEM;
        }
    }

    if (s_ble_rx_mb == NULL) {
        s_ble_rx_mb = xMessageBufferCreate(PROTO_BLE_RX_BYTES);
        if (s_ble_rx_mb == NULL) {
            return ESP_ERR_NO_MEM;
        }
    }

    esp_err_t err = proto_transport_install();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "transport install failed (%s): %s", PROTO_TRANSPORT_NAME, esp_err_to_name(err));
        return err;
    }

    /* Route ESP_LOG output through the frame protocol while a host is active. */
    s_prev_vprintf = esp_log_set_vprintf(proto_vprintf);

    BaseType_t ret = xTaskCreatePinnedToCore(proto_task, "proto", 4096, NULL, 5, NULL, 0);
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "proto task create failed");
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "host protocol ready on %s: 115200 8N1, AA 55 + CRC8, idle timeout %d ms",
             PROTO_TRANSPORT_NAME, CONFIG_SERIAL_PROTO_TIMEOUT_MS);
    return ESP_OK;
#endif
}
