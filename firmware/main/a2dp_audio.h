/*
 * A2DP sink audio source for the ESP32-S31 UAC/A2DP bridge.
 *
 * BT has priority over UAC. This module owns:
 *   - Bluetooth controller / Bluedroid / A2DP sink init
 *   - forked Bluedroid internal codec path
 *   - a 32-bit stereo PCM ring that the I2S pump task reads
 *
 * The UAC control task polls the state functions below every 10 ms and is
 * responsible for source switching and I2S clock reconfiguration.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t a2dp_audio_init(void);

bool a2dp_audio_is_connected(void);
bool a2dp_audio_is_streaming(void);
bool a2dp_audio_is_decoder_ready(void);
bool a2dp_audio_is_codec_busy(void);
/* Reserve only an already-selected BT route: a tentative SUSPEND has a bounded
 * grace for a later CFG/profile-close. Once confirmed, readiness or a terminal
 * event releases it. The confirmed 2 s deadline only reports a wait. */
bool a2dp_audio_is_acl_connected(void);
void a2dp_audio_set_output_selected(bool selected);
bool a2dp_audio_route_hold_active(void);
uint32_t a2dp_audio_get_stream_epoch(void);
void a2dp_audio_route_ready(uint32_t epoch);
uint32_t a2dp_audio_get_sample_rate(void);
uint32_t a2dp_audio_get_stream_duration_ms(void);

/* core0-only coherent source-policy snapshot (same lock as the PCM ring). */
typedef struct {
    bool connected, streaming, decoder_ready, codec_busy, cfg_failed;
    bool acl_connected, route_hold;
    uint32_t epoch, sample_rate, packets, duration_ms;
    size_t level_bytes;
    /* ★ 0.150：起播水位按编码器区分（用户要求"每个编码器都有自己独立的起播水位"）。
     * LDAC 990kbps 的包更大、间隔更长，40 ms 只够 4~8 个包，起播后立刻欠载；
     * 因此 get_state 也把已应用的 codec 标识暴露给源策略，由它选用更高水位。
     * 0 = 非厂商 codec（SBC/AAC），LDAC 为 vendor 0x12D / codec 0xAA。 */
    uint32_t codec_vendor_id;
    uint32_t codec_codec_id;
    uint8_t  codec_type;
} a2dp_audio_state_t;
void a2dp_audio_get_state(a2dp_audio_state_t *state);
/* Control-task fault cut, bound to SDK codec/config epoch. Invalidates PCM and
 * releases only the failed BT route; not a profile/ACL disconnect API. */
bool a2dp_audio_fail_codec(uint32_t codec_id, uint32_t codec_epoch);
/* Commit cut point, serialized with early BT events. Selecting BT preserves
 * its prebuffer; releasing BT invalidates and flushes only the departing BT.
 * A changed epoch/PCM count or recognized transaction cancels a fallback. */
bool a2dp_audio_commit_output(bool selected, uint32_t epoch, uint32_t rate,
                             uint32_t packets);
size_t a2dp_audio_read_pcm32_epoch(uint8_t *dst, size_t bytes, uint32_t epoch);
/* Final core0 cut: validate the BT read epoch and enqueue through the existing
 * nonblocking DSP producer API under the same lock as early BT events. A stale
 * read returns ESP_ERR_INVALID_STATE without transferring slot ownership.
 * Caller owns an acquired slot and the dsp_pcm_begin producer gate. The DSP
 * descriptor still carries the OUTPUT epoch, never this BT epoch. */
esp_err_t a2dp_audio_submit_pcm32_epoch(int slot, size_t samples, uint32_t flags,
                                      uint32_t epoch);
/* Non-blocking. Returns bytes copied (multiple of 8) or 0 if fewer than
 * max_bytes are currently available. Output is always stereo 32-bit PCM. */
size_t a2dp_audio_read_pcm32(uint8_t *dst, size_t max_bytes);
void a2dp_audio_flush_pcm(void);

/* ★ 0.151：PCM 注入（用户要求不再依赖手机）。
 * 开启后 bt_app_a2d_data_cb 跳过"必须有真 A2DP 连接"的检查，其余下游
 * （ring/gate/pump/core1/水位/trim）全部走真实路径，因此能复现设备侧任何
 * 断流/欠载/门控行为。切码 = 用新的 vendor/codec/rate 再调一次 enable()。 */
void a2dp_audio_inject_enable(uint32_t rate, int bits, int channels,
                              uint32_t vendor_id, uint32_t codec_id);
void a2dp_audio_inject_disable(void);
void a2dp_audio_inject_pcm(const uint8_t *data, size_t len);

/* Cheap ring level probe (bytes buffered, stereo 32-bit PCM). Used by the
 * source policy to keep playback muted until a pre-buffer is ready. */
size_t a2dp_audio_get_level_bytes(void);

/* 0.128（工单 12 §B.4）：BT 预热水位的**唯一 core0 常数来源**。
 * core0 两端引用同一对值：ctrl 侧的门控水位（uac2_audio.c）与生产者侧的前移 trim
 * （a2dp_audio_int.c 的 pcm_ring_push）。
 * 0.147：**回退 0.146 的水位改动**。0.146 曾把开门水位提到 60 ms / 闭门排空提到
 * 80 ms 以抗 A2DP 抖动，但用户实测"没有改善"，故恢复 40/60 ms（0.145 口径）。
 * 播放期 trim 仍由独立的 BT_PLAY_HIGH_MS_OTHER(60)/LDAC96(70) 控制，与这里解耦。
 * ⚠ 不放进冻结的 dsp_audio.h。 */
#define A2DP_BT_PREBUFFER_MS   40
#define A2DP_BT_LEVEL_HIGH_MS  60

/* ★ 0.150：按编码器区分的起播/预热水位（用户要求"每个编码器都有自己独立的起播水位"）。
 * 背景：LDAC 990kbps 的编码包更大、包间隔更长，统一 40 ms 在 44.1k 只等于约 8 个
 * 解码块、在高采样率更少，起播后立刻欠载 —— 用户实测 990kbps 完全不能播放。
 *
 * **必须由 push 侧（pcm_ring_push 的 floor/high）和门控侧（起播条件）共用同一组值**，
 * 否则会重演 0.52 的竞态：门要求 N ms，而 push 在未选源时把环排回 40 ms，门永远开不了。
 *
 * ★ 0.156：环已实测安全地翻倍到 131072 B（16384 帧），DRAM 仍余 176 KB。
 * 0.153/0.154 的崩溃已确认是那段 tick-trace 代码的 bug，与 ring/DRAM 无关。
 * 因此把 LDAC 水位提到真正能用满大环的值（96k 环 171 ms）：
 *   44.1k/48k（371/341 ms 环）→ 80/100 ms
 *   88.2k  （186 ms 环）      → 80/100 ms
 *   96k    （171 ms 环）      → 100/120 ms
 * 播放期 trim 与预热期共用同一组值，避免"环很大却按 60 ms 丢块"。 */
#define A2DP_BT_LDAC_PREBUFFER_MS     80u   /* LDAC ≤48k 起播 */
#define A2DP_BT_LDAC_HIGH_MS         100u   /* LDAC ≤48k 预热/播放上限 */
#define A2DP_BT_LDAC_MID_PREBUFFER_MS 80u   /* LDAC 88.2k */
#define A2DP_BT_LDAC_MID_HIGH_MS     100u
#define A2DP_BT_LDAC_HI_PREBUFFER_MS 100u   /* LDAC 96k（128 ms 环，留 28 ms 余量）*/
#define A2DP_BT_LDAC_HI_HIGH_MS      120u

/* 0.171（实测根因）：起播水位必须真的能装进环。
 * 96k 的 100 ms = 96*100*8 = 76800 B；8192 帧的环只有 65536 B（85.3 ms），
 * 于是 uac_poll_source_policy 的 `level >= prebuffer` 永远为假，BT 门永远不开：
 * 现场 pkt 持续增长、ring 钉在 63488 B、ovf 暴涨、delivered=0。
 * 该不等式在编译期钉死，避免以后调水位或改环时再次出现「门数学上不可达」。 */
/* The 100 ms LDAC 96 kHz watermark only exists while the LDAC SEP is
 * compiled in; 0.216 disables LDAC (96k/990 is not realtime on this CPU and
 * its ~30 KB working set starved the BT stack). Non-LDAC 96 kHz streams use
 * the shared 60 ms high watermark below. */
#if defined(CONFIG_BT_PCM_RING_FRAMES) && defined(CONFIG_BT_A2DP_LDAC_EXPERIMENTAL) && \
    (CONFIG_BT_PCM_RING_FRAMES * 8 < 96000 / 1000 * A2DP_BT_LDAC_HI_PREBUFFER_MS * 8)
#error "CONFIG_BT_PCM_RING_FRAMES is smaller than the LDAC 96 kHz prebuffer"
#endif
#if defined(CONFIG_BT_PCM_RING_FRAMES) && \
    (CONFIG_BT_PCM_RING_FRAMES * 8 < 96000 / 1000 * A2DP_BT_LEVEL_HIGH_MS * 8)
#error "CONFIG_BT_PCM_RING_FRAMES is smaller than the 96 kHz high watermark"
#endif

/* LDAC 判定：只看**已成功应用**的 codec 标识（vendor 0x12D / codec 0xAA），
 * 不按手机菜单文字或 "990" 推断，也不改解码器。 */
static inline bool a2dp_bt_is_ldac_id(uint32_t vendor_id, uint32_t codec_id)
{
    return vendor_id == 0x0000012Du && codec_id == 0x00AAu;
}

/* 起播水位（ms）：两侧共用。非 LDAC 一律回到既有 40 ms。 */
static inline uint32_t a2dp_bt_prebuffer_ms(uint32_t rate, uint32_t vendor_id, uint32_t codec_id)
{
    if (!a2dp_bt_is_ldac_id(vendor_id, codec_id)) {
        return A2DP_BT_PREBUFFER_MS;
    }
    if (rate >= 90000u) {
        return A2DP_BT_LDAC_HI_PREBUFFER_MS;
    }
    if (rate >= 80000u) {
        return A2DP_BT_LDAC_MID_PREBUFFER_MS;
    }
    return A2DP_BT_LDAC_PREBUFFER_MS;
}

/* 预热上限（ms）：push 侧未选源时允许环涨到这里再丢最旧块。 */
static inline uint32_t a2dp_bt_level_high_ms(uint32_t rate, uint32_t vendor_id, uint32_t codec_id)
{
    if (!a2dp_bt_is_ldac_id(vendor_id, codec_id)) {
        return A2DP_BT_LEVEL_HIGH_MS;
    }
    if (rate >= 90000u) {
        return A2DP_BT_LDAC_HI_HIGH_MS;
    }
    if (rate >= 80000u) {
        return A2DP_BT_LDAC_MID_HIGH_MS;
    }
    return A2DP_BT_LDAC_HIGH_MS;
}

/* 0.127 拒收理由（Codex 工单 11 §2.1）。
 * 有限 enum + 少量计数，**不加事件环**；顺序即 reject_by_reason 的下标。 */
typedef enum {
    A2DP_REJ_NONE = 0,
    A2DP_REJ_NOT_CONNECTED,      /* 未连接 */
    A2DP_REJ_NOT_STREAMING,      /* 未推流 */
    A2DP_REJ_CFG_PENDING,        /* 配置等待 */
    A2DP_REJ_CFG_FAILED,         /* 配置失败 */
    A2DP_REJ_FORMAT_LEN,         /* 格式/长度不符 */
    A2DP_REJ_EPOCH_STALE,        /* 代次失效 */
    A2DP_REJ_RING_FULL,          /* 环满 */
    A2DP_REJ_REASON_COUNT
} a2dp_reject_reason_t;

typedef struct {
    uint32_t packets;
    uint32_t drops;
    uint32_t ring_overflows;
    uint32_t flushes;
    size_t   level_bytes;
    size_t   capacity_bytes;
    size_t   min_level_bytes;
    size_t   max_level_bytes;
    /* v25（固件 0.25）新增：供 0xB1 BT_STATUS 上报。
     * `codec_type` 用 A2DP 的 `esp_a2d_mct_t` 原值（0=SBC / 2=M24(AAC)），
     * 主机侧不需要另建映射表；`0xFF` = 尚未协商。 */
    uint8_t  codec_type;
    uint8_t  channels;
    uint32_t sample_rate;
    uint32_t duration_ms;
    uint32_t cfg_failures, epoch_rejects, flush_bytes;
    uint32_t prepare_trim_blocks, prepare_trim_frames;
    uint32_t play_trim_blocks, play_trim_frames;
    /* ★ 0.160：排尾（draining）账目。读出不等于已播放，所以分开记：
     *   drain_read_bytes  : 从冻结旧区间读出的字节
     *   drain_blocks      : 排尾期间真正交给 core1 的块数（pump 侧统计）
     * 配合 flush_bytes / play_trim_* 就能判断一次切码到底是"旧尾播完"
     * 还是"直接丢弃"——这正是之前争论不清的地方。 */
    uint32_t stream_autostart;
    uint32_t autostart_bits, autostart_channels;   /* ★ 0.168：无 START 事件的自动起流次数 */
    uint32_t drain_read_bytes;
    uint32_t drain_blocks;
    /* 0.127：拒收记账（Codex 工单 11 §2.1）。
     * 原有 packets/ring_overflows/drops/epoch_rejects 的语义与计数**不变**；
     * 以下是**新增的每理由累计**与"最近一次拒绝"快照，用于把"丢了多少、为什么丢"
     * 从"只有一个总数"变成可判。
     *   reject_by_reason[i] 的理由编号见 a2dp_audio.h 的 A2DP_REJ_*（写进头供 core0 侧引用）
     *   reject_bytes_raw / reject_bytes_conv 分别计 **raw 输入字节** 与 **归一化内部字节**，
     *   两者不混用：没有有效格式时只记 raw 字节，不猜帧数。
     *   last_reject_* 只描述**最近一次**拒绝，不能解释全部历史；
     *   cb_epoch / cur_epoch 是回调采样值与当前值，**不是** SDK 证明的原始解码代次。 */
    uint32_t reject_total;
    uint32_t reject_bytes_raw;
    uint32_t reject_bytes_conv;
    uint32_t reject_by_reason[A2DP_REJ_REASON_COUNT];
    uint32_t last_reject_reason;
    uint32_t last_reject_bytes;
    uint32_t last_reject_cb_epoch;
    uint32_t last_reject_cur_epoch;
    uint32_t acl_incarnation;
    /* 0.127 ACL 迟读证据（工单 11 §2.3）。布尔 ACL 快照会漏掉两次采样间的断连再连，
     * 资格判定必须看**累计计数 + handle + reason**。
     * `acl_other_peer` 是"与当前 peer 不同的 ACL 事件"累计：>0 时资格记**不确定**，
     * 不得悄悄略过。断开事件的 peer/handle/reason 都留最近一次。 */
    uint32_t acl_conn_ok;
    uint32_t acl_conn_fail;
    uint32_t acl_disconn;
    uint32_t acl_other_peer;
    uint16_t acl_handle;
    uint16_t last_acl_disc_handle;
    uint32_t last_acl_disc_reason;
    uint8_t  last_acl_disc_peer[6];
    /* 0.128（工单 12 §B.2）：同锁导出**当前** ACL up 与 peer，以及最近断开 peer。
     * 异 peer 观察以"已知目标音频 peer"为准；目标未明时如实标不确定，不新增 peer 表。 */
    bool     acl_up;
    uint8_t  acl_peer[6];
    uint8_t  audio_peer[6];
    /* 0.127：最近一次成功应用的 A2DP 配置的**厂商** codec 标识。
     * `codec_type` 的 0xFF 只能表示"厂商 codec"，**无法区分** LDAC/aptX/aptX-HD 等；
     * 工单 11 §3.3 要求资格判定时保存实际厂商 codec 名/配置 ⇒ 这里留 vendor/codec id。 */
    uint32_t codec_vendor_id;    /* AOSP A2DP vendor id，0 = 非厂商 codec */
    uint32_t codec_codec_id;     /* 厂商内 codec id（如 LDAC 0x00AA / aptX 0x0001） */
    uint32_t cfg_apply_count;    /* 成功应用配置的累计次数（代次来源，不是 stream epoch） */
    /* One PCM-lock snapshot, including the normalized ring byte ledger.
     * read_bytes counts removal from the ring, not successful I2S delivery. */
    uint32_t snapshot_us, epoch, head_bytes, tail_bytes, read_bytes;
    uint32_t submit_stale_blocks, submit_stale_frames;
    bool connected, streaming, decoder_ready, codec_busy, route_hold;
} a2dp_audio_stats_t;

/* Valid decoder PCM deliveries, without modifying the diagnostic window. */
uint32_t a2dp_audio_get_pcm_packets(void);
void a2dp_audio_get_stats(a2dp_audio_stats_t *stats);

#ifdef __cplusplus
}
#endif
