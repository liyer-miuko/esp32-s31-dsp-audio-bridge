/*
 * UAC2 asynchronous speaker stream - ESP32-S31.
 *
 * Firmware-side terminology note: ASIO is a Windows PC audio driver and does
 * not run on the ESP32. What this firmware implements is the USB Audio Class
 * 2.0 mechanism that ASIO/WASAPI rely on: an explicit feedback endpoint and a
 * device-side FIFO water-level control loop (asynchronous synchronization).
 */
#pragma once

#include "esp_err.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    UAC2_AUDIO_SOURCE_UAC = 0,
    UAC2_AUDIO_SOURCE_BT = 1,
} uac2_audio_source_t;

esp_err_t uac2_audio_init(void);

/* I2S audio timebase for future core1 DSP blocks.
 * One pump tick is exactly I2S_OUT_DMA_FRAME_NUM stereo frames
 * (currently 240). DSP on core1 can timestamp a block with:
 *   stream_us = ((uint64_t)tick * I2S_OUT_DMA_FRAME_NUM * 1000000ULL) / rate;
 */
uint32_t uac2_audio_get_pump_tick(void);
uint32_t uac2_audio_get_output_rate(void);

/* RAM-only diagnostic: skip the core1 DSP call for BT and UAC. Default false;
 * not part of the configuration blob/NVS. Set while audio is stopped. */
void uac2_audio_set_dsp_test_bypass(bool bypass);
bool uac2_audio_get_dsp_test_bypass(void);

/* RAM-only legacy seq/zero/register/high-rate FIFO observations, OFF at boot.
 * Setter belongs to the core0 proto task. A mode change discards old queued
 * legacy records and partial anchors, never PCM or B6 dual-tape evidence.
 * Getter: bit0 enabled, whole word is mode generation. Frame indices restart
 * per mode; legacy-only counters accumulate enabled intervals. Basic RX/error
 * counters and the independently armed B6 dual tapes remain active as before. */
void uac2_audio_set_legacy_diagnostics(bool enabled);
uint32_t uac2_audio_get_legacy_diagnostics(void);

/* RAM-only pre-FU input probe for the fixed 96k/32-bit diagnostic tone.
 * Controls apply at the next successful pump tick. Query until requested_seq
 * equals applied_seq; read retained events only after confirmed disarm.
 * No audio/config/NVS mutation. Re-arm starts a new diagnostic generation. */
#define UAC2_INPUT_PROBE_CAPACITY 16u
#define UAC2_INPUT_PROBE_THRESHOLD (1u << 27)
typedef struct {
    uint32_t requested_seq, applied_seq, arm_seq, armed;
    uint32_t start_us, stop_us, last_pcm_us;
    uint32_t ticks, blocks, frames, idle_ticks, unsupported_ticks;
    uint32_t events, dropped, stored;
} uac2_input_probe_stats_t;
typedef struct {
    uint32_t us, frame;  /* pump tick time; observed input stereo frame index */
    int32_t prev_l, prev_r, cur_l, cur_r;
    uint32_t block, flags; /* 1-based input block; bit0 L / bit1 R jump */
} uac2_input_probe_event_t;
bool uac2_audio_input_probe_request(bool arm);
void uac2_audio_input_probe_get_stats(uac2_input_probe_stats_t *stats);
bool uac2_audio_input_probe_get_event(uint8_t index, uint32_t *arm_seq,
                                     uac2_input_probe_event_t *event);

/* ★ 0.132 RX_POST_ENQUEUE content边界的第二份 tape（同一个 0xB6 ARM 代次）。
 *
 * 与上面 pre-FU 探针的区别只在**观察点**：本观察在 RX 生产上下文（TinyUSB 接收
 * 回调）里、**软件 FIFO 写入之后**读回已入环字节，覆盖**本包全部帧**（不沿用
 * seq64 的 `pos<64` 稀疏条件），因此对普通 500 Hz 正弦 PCM 同样有效。
 *
 * 标签纪律：这是 **RX_POST_ENQUEUE**。它不是 PRE_FIFO、不是 USB_WIRE、
 * 也不是 HOST_SUBMITTED；入环之后的观察不能回答"物理线上是什么"。
 *
 * 代次与所有权：pump 在成功 tick 上应用 ARM/DISARM（与 pre-FU 探针同一控制入口），
 * 只发布 `gen/enabled` 两个 volatile；**扫描历史（prev/计数）由 RX 生产上下文独占**，
 * 它看到代次变化时自行断开历史并重开本代次计数。默认 OFF；OFF 时零扫描开销。
 * 事件只保留前 UAC2_RX_CONTENT_CAPACITY 条，满则只增 dropped（不覆盖）。 */
#define UAC2_RX_CONTENT_CAPACITY 16u
#define UAC2_RX_CONTENT_THRESHOLD (1u << 27)
typedef struct {
    uint32_t requested_seq, applied_seq, arm_seq, armed;
    uint32_t start_us, stop_us, last_rx_us;
    uint32_t packets, frames, gaps;
    uint32_t unsupported, null_ff, rebase, delta_ne, rejected;
    uint32_t events, dropped, stored;
} uac2_rx_content_stats_t;
typedef struct {
    uint32_t us, rx_frame;   /* RX 回调时间；本代次已扫描输入帧的 0 基索引 */
    int32_t prev_l, prev_r, cur_l, cur_r;
    uint32_t packet, flags;  /* 本代次 1 基包号；bit0 L 触发 / bit1 R 触发 */
    uint32_t rx_arm_seq;     /* 本条目所属代次：getter 用它拒绝混代次读取 */
} uac2_rx_content_event_t;
void uac2_audio_rx_content_get_stats(uac2_rx_content_stats_t *stats);
bool uac2_audio_rx_content_get_event(uint8_t index, uint32_t *arm_seq,
                                     uac2_rx_content_event_t *event);

/* Silence-filled pump ticks (UAC FIFO underrun). Together with the pump tick
 * this is how the hardware-mute state machine (hw_mute.c, stage C1) decides
 * "the pump is writing real PCM right now", without adding any hook to the
 * audio path:  playing := pump advanced && silence did NOT advance.
 *
 * ⚠ 0.50（L44）：hw_mute **不再**用这个计数器判 playing（它只认 UAC FIFO 空，
 *   在 BT 模式下永不前进 ⇒ playing 恒真、功放整段 BT 会话都开着，现场已实测）。
 *   保留它是因为 `silence_runs` 是 BUG-1 的分流计数器、且被 diag/量具解析。 */
uint32_t uac2_audio_get_silence_tick(void);

/* ★ 0.50（L44）：hw_mute 判 "playing" 用的心跳 —— "本 tick 有没有交付真实音频"。
 * `has_data == false` 的每个 tick 都计一次，覆盖：UAC FIFO 空 / BT 门未开 /
 * BT 欠载 / 源切换中 / 静音窗 / codec 静音（后四种旧计数器一个都不记）。
 * ⚠ 0.113 起 hw_mute **不再**用这个计数器（它仍作诊断保留），改用下面的
 *   `uac2_audio_get_pcm_active_tick()`。 */
uint32_t uac2_audio_get_nonaudio_tick(void);

/* ★ 0.113（用户判据）：MUTE 的活动信号 = **任一路输入正在送 PCM**。
 * UAC ISR 收到完整 PCM 帧或 BT 解码回调交付有效 PCM 时前进；
 * 连接/协商本身不算，全零 PCM 算。与当前选中的输出源无关。 */
uint32_t uac2_audio_get_pcm_active_tick(void);

/* Current output source (UAC or BT) as selected by the priority policy. */
uac2_audio_source_t uac2_audio_get_output_source(void);

/* True while the UAC AS interface is open (alt != 0). */
bool uac2_audio_is_streaming(void);

/* USB OUT FIFO fill level in percent (0 when unmounted). */
uint8_t uac2_audio_get_fifo_fill_percent(void);

#ifdef __cplusplus
}
#endif
