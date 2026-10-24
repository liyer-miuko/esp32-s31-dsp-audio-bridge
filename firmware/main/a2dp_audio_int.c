/*
 * A2DP sink audio source for the ESP32-S31 UAC/A2DP bridge.
 *
 * This version uses the ESP32-S31-A2DP-Codecs forked Bluedroid's INTERNAL
 * codec path: esp_a2d_sink_register_data_callback() delivers decoded PCM and
 * the stack owns SBC/AAC/LDAC/aptX-family codec lifecycle. This avoids the
 * external esp_audio_codec component and survives suspend/resume because the
 * stack decoder is not closed by the application.
 *
 * Codec exposure, CIE parsing and overload recovery use the core0 codec policy.
 */
#include <string.h>
#include <stdlib.h>
#include <inttypes.h>

#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

#include "esp_log.h"
#include "esp_timer.h"
#include "esp_system.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_gap_bt_api.h"
#include "esp_a2dp_api.h"
#include "esp_avrc_api.h"

#include "a2dp_audio.h"
#include "esp_a2dp_codec_policy.h"
#include "bt_codec_guard.h"
#include "bt_route_hold.h"
/* 只读复用冻结头里的**槽常数**（DSP_PCM_SLOT_BYTES = 240 帧 × 8 B）。
 * 工单 11 §2.2 明确允许"复用既有只读槽常数"；**不修改**该冻结头。 */
#include "dsp_audio.h"

static const char *TAG = "a2dp";

/* ---------------- PCM ring (32-bit stereo, monotonic byte counters) ---------------- */

#ifndef CONFIG_BT_PCM_RING_FRAMES
#define CONFIG_BT_PCM_RING_FRAMES 4096
#endif
#define A2DP_PCM_RING_BYTES         (CONFIG_BT_PCM_RING_FRAMES * 8)

/* 0.128：预热水位常数已收归 **a2dp_audio.h** 一处（工单 12 §B.4）。
 * 本地不再重复定义，避免两处漂移；值仍为 40 ms 开门 / 60 ms 高水位。 */
#define DSP_BT_PREBUFFER_MS   A2DP_BT_PREBUFFER_MS
#define DSP_BT_LEVEL_HIGH_MS  A2DP_BT_LEVEL_HIGH_MS

static uint8_t s_pcm_ring[A2DP_PCM_RING_BYTES] __attribute__((aligned(4)));
static volatile uint32_t s_pcm_head; /* write position */
static volatile uint32_t s_pcm_tail; /* read position */
static uint32_t s_pcm_read_bytes;
static uint32_t s_submit_stale_blocks, s_submit_stale_frames;
static portMUX_TYPE s_pcm_mux = portMUX_INITIALIZER_UNLOCKED;

/* ---------------- State exported to the source policy ---------------- */

static volatile bool s_bt_connected;
static volatile bool s_bt_streaming;
/* Missing START is tolerated only once for a fresh configuration, never as
 * an implicit undo of an explicit SUSPEND. Protected by s_pcm_mux. */
static bool s_pcm_autostart_allowed;
static volatile bool s_cfg_pending;
static bool s_cfg_failed;
static uint32_t s_cfg_generation;
/* The worker owns format; an A2DP profile owns permission to play it. A
 * profile can close/reopen on the SAME ACL while decoder setup yields. Keep
 * that completed format as metadata, without restarting a closed transport.
 * A real ACL disconnect invalidates the cache, including a late completion. */
static uint32_t s_worker_cfg_epoch, s_worker_link_generation, s_link_generation;
static esp_a2dp_codec_format_t s_worker_format;
static bool s_worker_format_ready;
static uint32_t s_start_epoch, s_pcm_epoch;
/* ★ 0.168：无 START 事件时由首个数据包自动起流的次数（诊断） */
static uint32_t s_stream_autostart;
/* ★ 0.170：起流当时看到的格式，供 bt-flow 的 as_bits/as_ch 诊断 */
static uint32_t s_autostart_bits, s_autostart_channels;
static volatile int64_t s_stream_start_us;
static volatile uint32_t s_cfg_rate;
static volatile int s_cfg_channels;
static volatile int s_cfg_bits;      /* decoder PCM width: 16 / 24 / 32 */
static volatile int s_codec_type;    /* esp_a2d_mct_t value */

/* Ring, generation and the final nonblocking DSP enqueue share one short lock.
 * The DSP queue never calls back into A2DP; no logs/codec work under this lock.
 * Old conversion or a filled-but-not-submitted slot cannot cross a CFG flush. */
static bt_route_hold_t s_route_hold;
#define BT_RECONFIG_WARN_MS 2000
/* SUSPEND precedes CFG/profile-close: the recorded 148 ms gap exceeds the
 * 100 ms liveness watchdog plus its 28 ms commit interval. Retain selected BT
 * briefly so CFG can confirm the replacement; an ordinary pause still expires. */
#define BT_SUSPEND_GRACE_MS 300
/* LDAC quality changes in the recorded sender keep the CFG generation and
 * suspend for 415..427 ms. Keep ordinary codec pauses at the accepted 300 ms. */
#define BT_LDAC_SUSPEND_GRACE_MS 600
static bool s_acl_up;
static bool s_peer_valid;
static uint8_t s_acl_peer[6];
static uint8_t s_audio_peer[6];

/* ★ 0.151：PCM 注入状态。开启时 bt_app_a2d_data_cb 跳过"必须有真蓝牙"的检查，
 * 其余（ring/gate/pump/core1/水位/trim）全部走真实路径，因此能复现真实断流。 */
static bool s_inject_active;

static bool route_link_up(void)
{
    return s_acl_up && s_peer_valid && memcmp(s_acl_peer, s_audio_peer, 6) == 0;
}

/* Caller holds s_pcm_mux. This is actual worker configuration completion,
 * not merely a CIE having parsed. Policy never calls app code under its lock. */
static bool decoder_ready_locked(void)
{
    return s_bt_connected && s_cfg_bits != 0 && !s_cfg_pending && !s_cfg_failed &&
        (s_inject_active || esp_a2dp_codec_policy_pcm_ready());
}

static bool decoder_busy_locked(void)
{
    return s_cfg_pending || (!s_cfg_failed && s_cfg_bits != 0 &&
        !s_inject_active && !esp_a2dp_codec_policy_pcm_ready());
}

/* ---------------- Diagnostics counters ---------------- */

static volatile uint32_t s_pcm_packets;
static volatile uint32_t s_pcm_drops;
static volatile uint32_t s_ring_overflows;
static volatile uint32_t s_ring_flushes;
static volatile size_t s_ring_min_bytes = A2DP_PCM_RING_BYTES;
static volatile size_t s_ring_max_bytes;
static uint32_t s_cfg_failures, s_epoch_rejects, s_flush_bytes;
static uint32_t s_prepare_trim_blocks, s_prepare_trim_frames;
static uint32_t s_play_trim_blocks, s_play_trim_frames;

/* ---------------- 0.127 拒收记账（Codex 工单 11 §2.1） ----------------
 * 原有 packets/drops/ring_overflows/epoch_rejects 的语义与计数**一个字不改**；
 * 这里只**新增**：每理由累计数量/拒收字节 + 最近一次拒绝的快照。
 * 三处拒绝点（pcm_ring_push / push_pcm_to_ring 转换支路 / bt_app_a2d_data_cb）
 * 共用一个 inline，确保**同一批数据不在两层重复计**。
 * 全部更新都在既有 s_pcm_mux 临界区内，热路径不 printf、不分配。 */
static uint32_t s_rej_total;
static uint32_t s_rej_bytes_raw;      /* raw 输入字节（未归一化） */
static uint32_t s_rej_bytes_conv;     /* 归一化后的内部 32-bit 立体声字节 */
static uint32_t s_rej_by_reason[A2DP_REJ_REASON_COUNT];
static uint32_t s_last_rej_reason, s_last_rej_bytes;
static uint32_t s_last_rej_cb_epoch, s_last_rej_cur_epoch;
/* 最近一次成功应用的厂商 codec（0xFF 的 codec_type 无法区分厂商 codec） */
static uint32_t s_codec_vendor_id, s_codec_codec_id, s_cfg_apply_count;
/* ACL 代次：成功新连接才递增。**不复用 stream epoch**（切 codec 本来就会改 stream epoch）。 */
static uint32_t s_acl_incarnation;
/* 0.127 ACL 迟读证据（工单 11 §2.3）：布尔 ACL 快照会漏掉两次采样之间的断连再连，
 * 所以这里加**累计计数 + handle + peer + 最近断开 reason**，供资格判定迟读。
 * 所有相关事件都**先记账再沿用原逻辑**；peer 归属不明确也要留事件。 */
static uint32_t s_acl_conn_ok;        /* 成功连接累计 */
static uint32_t s_acl_conn_fail;      /* 失败连接累计（stat != SUCCESS） */
static uint32_t s_acl_disconn;        /* 断开累计 */
static uint32_t s_acl_other_peer;     /* 与当前 peer 不同的 ACL 事件累计（异 peer 单列） */
static uint16_t s_acl_handle;         /* 当前 ACL handle */
static uint16_t s_last_acl_disc_handle;
static uint32_t s_last_acl_disc_reason;
static uint8_t  s_last_acl_disc_peer[6];

/* The controller can retain two Classic ACLs, but this bridge has one A2DP
 * source. A second paired peer/idle ACL must not replace the audio peer's
 * liveness snapshot. Incarnations are minted only by successful GAP connects. */
typedef struct { bool up; uint8_t peer[6]; uint16_t handle; uint32_t incarnation; } acl_record_t;
static acl_record_t s_acl_records[CONFIG_BT_CTRL_BR_EDR_MAX_ACL_CONN];
static uint32_t s_acl_sequence;
static acl_record_t *acl_record(const uint8_t peer[6], bool allocate)
{
    acl_record_t *empty = NULL;
    for (unsigned i = 0; i < sizeof(s_acl_records)/sizeof(s_acl_records[0]); ++i) {
        acl_record_t *r = &s_acl_records[i];
        if (r->up && !memcmp(r->peer, peer, 6)) return r;
        if (!r->up) empty = r;
    }
    return allocate ? empty : NULL;
}
static void acl_select_peer(const uint8_t peer[6])
{
    acl_record_t *r = acl_record(peer, false);
    memcpy(s_acl_peer, peer, 6);
    s_acl_up = r != NULL;
    s_acl_handle = r ? r->handle : 0;
    if (r) s_acl_incarnation = r->incarnation;
}

/* 记账：只累加，不做任何判断。调用方负责选对 reason 与字节口径。 */
static inline void rej_note(a2dp_reject_reason_t reason, uint32_t raw_bytes,
                            uint32_t conv_bytes, uint32_t cb_epoch, uint32_t cur_epoch)
{
    if (reason <= A2DP_REJ_NONE || reason >= A2DP_REJ_REASON_COUNT) {
        return;
    }
    s_rej_total++;
    s_rej_bytes_raw += raw_bytes;
    s_rej_bytes_conv += conv_bytes;
    s_rej_by_reason[reason]++;
    s_last_rej_reason = (uint32_t)reason;
    s_last_rej_bytes = raw_bytes;
    s_last_rej_cb_epoch = cb_epoch;
    s_last_rej_cur_epoch = cur_epoch;
}

/* ---------------- BT app task ---------------- */

#define BT_APP_SIG_WORK_DISPATCH   1
typedef void (*bt_app_cb_t)(uint16_t event, void *param, uint32_t generation);

typedef struct {
    uint16_t    sig;
    uint16_t    event;
    bt_app_cb_t cb;
    void       *param;
    uint32_t    generation;
} bt_app_msg_t;

static QueueHandle_t s_bt_app_q;
static TaskHandle_t s_bt_app_task;

/* ---------------- Forward declarations ---------------- */

static void bt_app_a2d_hdl(uint16_t event, void *param, uint32_t generation);
static void bt_av_hdl_stack_evt(uint16_t event, void *param, uint32_t generation);

/* ---------------- PCM ring helpers ---------------- */

static inline size_t pcm_ring_avail(void)
{
    return (size_t)(s_pcm_head - s_pcm_tail);
}

static inline size_t pcm_ring_space(void)
{
    return (size_t)(A2DP_PCM_RING_BYTES - 1 - pcm_ring_avail());
}

/* 0.128（工单 12 §A）：**拒收只在第一次决定失败的那一次锁内记一次**。
 * 调用方把"这一批还没送进去的剩余量"按两种口径传进来：
 *   raw_remaining  = raw 输入字节
 *   conv_remaining = 归一化后的内部 32-bit 立体声字节
 * 成功入队**不记账**；第一次拒绝时沿用原 overflow/epoch_reject 计数，并在**同一把
 * s_pcm_mux 内**调用一次 rej_note(实际理由, raw_remaining, conv_remaining, cb_epoch, cur_epoch)。
 * 这样"已成功入环的前缀"不会被计成拒收，外层也**不需要**再补记或二次加锁。 */
static bool pcm_ring_push(const uint8_t *data, size_t len, uint32_t epoch,
                          size_t raw_remaining, size_t conv_remaining)
{
    bool ok = false;

    portENTER_CRITICAL(&s_pcm_mux);
    if (epoch != s_route_hold.epoch || !s_bt_streaming || s_cfg_pending || s_cfg_failed) {
        s_epoch_rejects++;
        a2dp_reject_reason_t why = (epoch != s_route_hold.epoch) ? A2DP_REJ_EPOCH_STALE
                                 : (!s_bt_streaming ? A2DP_REJ_NOT_STREAMING
                                    : (s_cfg_failed ? A2DP_REJ_CFG_FAILED : A2DP_REJ_CFG_PENDING));
        rej_note(why, (uint32_t)raw_remaining, (uint32_t)conv_remaining, epoch, s_route_hold.epoch);
    } else {
        /* ---- 0.127：预热段"保留新数据"的前移 trim（Codex 工单 11 §2.2）----
         * 背景：原先预热 trim 在低优先级 ctrl 任务里（uac2_audio.c:1403），生产者可以
         * 先把环填满 ⇒ W2b 起播区间实测 overflow +37。这里把该项 trim **移到生产者入环前**。
         *
         * 只作用于 **`!s_route_hold.selected`（BT 尚未被选中的预热段）**；
         * 已选 BT 的闭门/播放均不做延迟驱动 trim，旧水位不会被冒充成新流。
         * 判据用 `!selected` 实时状态，**不靠过期的水位快照**。
         *
         * 策略：为本次 chunk 留位置时把目标抬到 `max(high, floor + chunk_bytes)`，
         * 这样低采样率下也仍留下 40 ms 预缓冲；超出部分**丢最旧整块**，块数一次算出，
         * 不逐块循环。水位算式与 40 ms 开门 / 60 ms 高水位 / 240 帧整块**都不改**。
         * 整段仍在**同一把 s_pcm_mux** 内完成：epoch/选源检查、tail 推进、写环不跨代次。 */
        const size_t block_bytes = DSP_PCM_SLOT_BYTES;      /* 240 帧 × 8 B */
        if (!s_route_hold.selected && block_bytes > 0) {
            /* 只在**预热段**才算水位，避免给已选 BT 的热路径加无谓算术。
             * ★ 0.150：floor/high **必须与门控侧共用同一组按编码器的值**
             * （a2dp_bt_prebuffer_ms / a2dp_bt_level_high_ms）。否则 LDAC 的
             * 起播门槛被提到 50~80 ms，而这里仍排回 40 ms，门永远打不开 ——
             * 就是 0.52 那个竞态的复现。 */
            const uint32_t rate = s_cfg_rate ? s_cfg_rate : 48000u;
            const uint32_t prebuf_ms = a2dp_bt_prebuffer_ms(rate, s_codec_vendor_id, s_codec_codec_id);
            const uint32_t high_ms = a2dp_bt_level_high_ms(rate, s_codec_vendor_id, s_codec_codec_id);
            const size_t floor_b = (size_t)((uint64_t)rate * prebuf_ms / 1000u) * 8u;
            size_t high_b = (size_t)((uint64_t)rate * high_ms / 1000u) * 8u;
            if (high_b < floor_b + block_bytes) {
                high_b = floor_b + block_bytes;
            }
            size_t target = high_b;
            if (floor_b + len > target) {
                target = floor_b + len;
            }
            size_t level = pcm_ring_avail();
            if (level + len > target) {
                size_t over = level + len - target;
                size_t blocks = (over + block_bytes - 1) / block_bytes;   /* 向上整除 */
                /* 最多只能丢到 floor 为止（仅 level >= floor 时） */
                size_t max_blocks = (level >= floor_b) ? ((level - floor_b) / block_bytes) : 0;
                if (blocks > max_blocks) {
                    blocks = max_blocks;
                }
                if (blocks > 0) {
                    s_pcm_tail += (uint32_t)(blocks * block_bytes);
                    s_prepare_trim_blocks += (uint32_t)blocks;
                    s_prepare_trim_frames += (uint32_t)(blocks * (block_bytes / 8));
                    if (pcm_ring_avail() < s_ring_min_bytes) {
                        s_ring_min_bytes = pcm_ring_avail();
                    }
                }
            }
        }
        if (len <= pcm_ring_space()) {
            uint32_t head = s_pcm_head;
            size_t pos = (size_t)(head % A2DP_PCM_RING_BYTES);
            size_t first = A2DP_PCM_RING_BYTES - pos;
            if (first > len) {
                first = len;
            }
            memcpy(&s_pcm_ring[pos], data, first);
            if (len > first) {
                memcpy(s_pcm_ring, data + first, len - first);
            }
            s_pcm_head = head + (uint32_t)len;
            size_t level2 = (size_t)(s_pcm_head - s_pcm_tail);
            if (level2 > s_ring_max_bytes) {
                s_ring_max_bytes = level2;
            }
            ok = true;
        } else {
            /* 仍空间不足 ⇒ 如实按环满拒收（0.128：记的是**剩余量**，不是本次尝试量） */
            s_ring_overflows++;
            rej_note(A2DP_REJ_RING_FULL, (uint32_t)raw_remaining, (uint32_t)conv_remaining,
                     epoch, s_route_hold.epoch);
        }
    }
    portEXIT_CRITICAL(&s_pcm_mux);

    return ok;
}

static size_t pcm_ring_pop(uint8_t *dst, size_t len, uint32_t epoch)
{
    size_t copied = 0;
    portENTER_CRITICAL(&s_pcm_mux);
    if (epoch == s_route_hold.epoch && s_bt_streaming &&
        !s_cfg_pending && !s_cfg_failed && pcm_ring_avail() >= len) {
        const uint32_t tail = s_pcm_tail;
        const size_t pos = tail % A2DP_PCM_RING_BYTES;
        size_t first = A2DP_PCM_RING_BYTES - pos;
        if (first > len) first = len;
        memcpy(dst, s_pcm_ring + pos, first);
        if (len > first) memcpy(dst + first, s_pcm_ring, len - first);
        s_pcm_tail = tail + (uint32_t)len;
        s_pcm_read_bytes += (uint32_t)len;
        const size_t level = pcm_ring_avail();
        if (level < s_ring_min_bytes) s_ring_min_bytes = level;
        copied = len;
    }
    portEXIT_CRITICAL(&s_pcm_mux);
    return copied;
}

/* Caller holds s_pcm_mux. Invalidating the epoch also rejects conversion work
 * that was already in progress before this flush. */
static void pcm_epoch_flush(void)
{
    ++s_route_hold.epoch;
    s_flush_bytes += (uint32_t)pcm_ring_avail();
    s_pcm_tail = s_pcm_head;
    ++s_ring_flushes;
}





/* ---------------- Decoded PCM -> fixed 32-bit stereo ring ---------------- */

static uint8_t s_conv[4096] __attribute__((aligned(4)));

static int32_t pcm_sample32(const uint8_t *p, int bits)
{
    uint32_t v = (uint32_t)p[0] | ((uint32_t)p[1] << 8);
    if (bits == 16) return (int32_t)(v << 16);
    v |= (uint32_t)p[2] << 16;
    if (bits == 24) return (int32_t)(v << 8);
    return (int32_t)(v | ((uint32_t)p[3] << 24));
}

static void push_pcm_to_ring(const uint8_t *data, size_t len, int bits,
                             int channels, uint32_t epoch)
{
    if (bits == 32 && channels == 2) {
        for (size_t off = 0; off < len;) {
            size_t chunk = len - off;
            if (chunk > sizeof(s_conv)) chunk = sizeof(s_conv);
            /* 32 位双声道：raw 与 conv 同口径（入环的就是 raw 本身）⇒ 剩余量传两次。
             * 成功入队不记账；第一次拒绝时由 pcm_ring_push 在**同一次锁内**记一次剩余量。 */
            if (!pcm_ring_push(data + off, chunk, epoch, len - off, len - off)) break;
            off += chunk;
        }
        return;
    }

    const size_t sample_bytes = (size_t)bits / 8;
    const size_t input_stride = sample_bytes * channels;
    size_t frames = len / input_stride;
    size_t off = 0;
    while (off < frames) {
        size_t chunk = frames - off;
        size_t max_frames = sizeof(s_conv) / 8;
        if (chunk > max_frames) chunk = max_frames;
        int32_t *out = (int32_t *)s_conv;
        for (size_t i = 0; i < chunk; i++) {
            const uint8_t *in = data + (off + i) * input_stride;
            out[2 * i] = pcm_sample32(in, bits);
            out[2 * i + 1] = (channels == 1) ? out[2 * i] : pcm_sample32(in + sample_bytes, bits);
        }
        /* 转换支路：raw 与 conv 分别按剩余帧数换算，一次交给 pcm_ring_push 记。
         * 失败只 break —— 不再在外层补记或二次加锁。 */
        if (!pcm_ring_push((const uint8_t *)out, chunk * 8, epoch,
                           (frames - off) * input_stride, (frames - off) * 8)) {
            break;
        }
        off += chunk;
    }
}

/* ---------------- A2DP codec configuration parsing ---------------- */

/* The decoder worker publishes the actual PCM format before decoding. Public
 * AUDIO_CFG is an independent, possibly delayed notification: logging it must
 * never replace that format, flush a newer ring, or resurrect a failed setup. */
static void log_codec_cfg(uint32_t generation)
{
    portENTER_CRITICAL(&s_pcm_mux);
    const bool current = generation == s_cfg_generation;
    const uint32_t rate = s_cfg_rate;
    const int bits = s_cfg_bits, channels = s_cfg_channels;
    const bool failed = s_cfg_failed;
    portEXIT_CRITICAL(&s_pcm_mux);
    ESP_LOGI(TAG, "codec: generation=%lu current=%u rate=%lu pcm_bits=%d ch=%d failed=%u publish=DECODER_WORKER notification_only=1",
             (unsigned long)generation, current, (unsigned long)rate, bits, channels, failed);
}

/* Caller holds s_pcm_mux. This copies only the actual worker PCM layout;
 * it neither opens a route, starts a stream, nor releases a SUSPEND veto. */
static void publish_worker_format_locked(const esp_a2dp_codec_format_t *format)
{
    const esp_a2dp_codec_descriptor_t *desc =
        esp_a2dp_codec_policy_descriptor(format->id);
    s_cfg_rate = format->rate;
    s_cfg_bits = format->pcm_bits;
    s_cfg_channels = format->channels;
    s_codec_type = format->id == ESP_A2DP_CODEC_SBC ? ESP_A2D_MCT_SBC :
                   format->id == ESP_A2DP_CODEC_AAC ? ESP_A2D_MCT_M24 : ESP_A2D_MCT_NON_A2DP;
    s_codec_vendor_id = desc->vendor_id;
    s_codec_codec_id = desc->codec_id;
}

static void bt_app_codec_configuration(const esp_a2dp_codec_format_t *format,
                                        uint32_t epoch, bool complete, bool success)
{
    const int64_t now_us = esp_timer_get_time();
    const bool valid = format && success &&
        esp_a2dp_codec_policy_allowed(format->id) &&
        format->rate >= 8000 && format->rate <= 96000 &&
        (format->pcm_bits == 16 || format->pcm_bits == 24 || format->pcm_bits == 32) &&
        (format->channels == 1 || format->channels == 2);
    bool began = false;
    portENTER_CRITICAL(&s_pcm_mux);
    bool current = true;
    if (!complete) {
        /* This runs synchronously at begin_config, after the previous decode
         * call and before configuring/decoding the replacement. Only one task
         * can publish a PCM layout; BTC event/log queue timing is irrelevant. */
        esp_a2dp_codec_policy_stream_boundary();
        began = bt_route_hold_begin(&s_route_hold, s_bt_connected && route_link_up(), now_us,
                                    (int64_t)BT_RECONFIG_WARN_MS * 1000);
        s_worker_cfg_epoch = epoch;
        s_worker_link_generation = s_link_generation;
        s_worker_format = valid ? *format : (esp_a2dp_codec_format_t){0};
        s_worker_format_ready = false;
        ++s_cfg_generation;
        s_cfg_pending = true;
        s_cfg_failed = false;
        s_pcm_autostart_allowed = valid;
        if (valid) publish_worker_format_locked(format);
        else {
            s_cfg_rate = s_cfg_bits = s_cfg_channels = 0;
            s_codec_type = ESP_A2D_MCT_NON_A2DP;
            s_codec_vendor_id = s_codec_codec_id = 0;
        }
        s_stream_start_us = 0;
        pcm_epoch_flush();
    } else {
        /* Profile lifetime is NOT decoder lifetime. The former generation
         * check permanently lost a successful layout if profile close arrived
         * during init: CONNECTED/START could then receive PCM with bits==0.
         * Retain success on this ACL, but never resurrect streaming or undo a
         * later SUSPEND. A different worker CFG or ACL remains a hard fence. */
        current = epoch == s_worker_cfg_epoch &&
                  s_worker_link_generation == s_link_generation;
        if (current) {
            s_cfg_pending = false;
            s_cfg_failed = !valid;
            s_worker_format_ready = valid;
            if (valid) {
                s_worker_format = *format;
                if (s_bt_connected) publish_worker_format_locked(format);
                ++s_cfg_apply_count;
            } else {
                ++s_cfg_failures;
                s_cfg_bits = 0;
                s_pcm_autostart_allowed = false;
                s_bt_streaming = false;
                s_stream_start_us = 0;

                bt_route_hold_clear(&s_route_hold);
                pcm_epoch_flush();
            }
        }
    }
    const uint32_t generation = s_cfg_generation;
    const uint32_t pcm_epoch = s_route_hold.epoch;
    const bool profile_up = s_bt_connected, cache_ready = s_worker_format_ready;
    portEXIT_CRITICAL(&s_pcm_mux);
    if (began) ESP_LOGI(TAG, "trace-bt: us=%lu event=route_hold_begin epoch=%lu warn_ms=%u cause=worker_cfg",
        (unsigned long)(uint32_t)now_us, (unsigned long)pcm_epoch, BT_RECONFIG_WARN_MS);
    ESP_LOGI(TAG, "trace-bt: us=%lu event=%s generation=%lu epoch=%lu sdk_epoch=%lu current=%u ok=%u rate=%lu pcm_bits=%u ch=%u profile=%u cached=%u authority=decoder_worker",
        (unsigned long)(uint32_t)now_us, complete ? "cfg_job_end" : "cfg_begin",
        (unsigned long)generation, (unsigned long)pcm_epoch, (unsigned long)epoch, current, valid,
        (unsigned long)(format ? format->rate : 0), format ? format->pcm_bits : 0,
        format ? format->channels : 0, profile_up, cache_ready);
}

/* ---------------- BT app task ---------------- */

static bool bt_app_send_msg(bt_app_msg_t *msg)
{
    if (msg == NULL || s_bt_app_q == NULL) {
        return false;
    }
    return xQueueSend(s_bt_app_q, msg, 0) == pdTRUE;
}

static void bt_app_task_handler(void *arg)
{
    (void)arg;
    bt_app_msg_t msg;

    for (;;) {
        if (xQueueReceive(s_bt_app_q, &msg, portMAX_DELAY) == pdTRUE) {
            if (msg.sig == BT_APP_SIG_WORK_DISPATCH && msg.cb != NULL) {
                msg.cb(msg.event, msg.param, msg.generation);
            }
            free(msg.param);
        }
    }
}

static bool bt_app_work_dispatch(bt_app_cb_t cb, uint16_t event, void *param,
                                 size_t param_len, uint32_t generation)
{
    bt_app_msg_t msg;
    memset(&msg, 0, sizeof(msg));
    msg.sig = BT_APP_SIG_WORK_DISPATCH;
    msg.event = event;
    msg.cb = cb;
    msg.generation = generation;

    if (param_len > 0 && param != NULL) {
        msg.param = malloc(param_len);
        if (msg.param == NULL) {
            ESP_LOGE(TAG, "bt app dispatch malloc failed: %u", (unsigned)param_len);
            return false;
        }
        memcpy(msg.param, param, param_len);
    }

    if (!bt_app_send_msg(&msg)) {
        free(msg.param);
        return false;
    }
    return true;
}

/* ---------------- BT / GAP callbacks ---------------- */

static void bt_app_dev_cb(esp_bt_dev_cb_event_t event, esp_bt_dev_cb_param_t *param)
{
    if (event == ESP_BT_DEV_NAME_RES_EVT && param->name_res.status == ESP_BT_STATUS_SUCCESS) {
        ESP_LOGI(TAG, "local device name: %s", param->name_res.name);
    }
}

static void bt_app_gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param)
{
    switch (event) {
    case ESP_BT_GAP_AUTH_CMPL_EVT:
        if (param->auth_cmpl.stat != ESP_BT_STATUS_SUCCESS) {
            ESP_LOGW(TAG, "authentication failed: %d", param->auth_cmpl.stat);
        } else {
            ESP_LOGI(TAG, "authentication ok: %s", (const char *)param->auth_cmpl.device_name);
        }
        break;
    case ESP_BT_GAP_CFM_REQ_EVT:
        /* Headless sink: accept the SSP numeric comparison ("Just Works").
         * Without this reply the controller never completes authentication and
         * Windows drops the link with HCI 0x05 before A2DP can stream. */
        ESP_LOGI(TAG, "SSP confirm request num=%06lu -> accept",
                 (unsigned long)param->cfm_req.num_val);
        esp_bt_gap_ssp_confirm_reply(param->cfm_req.bda, true);
        break;
    case ESP_BT_GAP_KEY_NOTIF_EVT:
        ESP_LOGI(TAG, "SSP passkey notify (no display)");
        break;
    case ESP_BT_GAP_KEY_REQ_EVT:
        ESP_LOGI(TAG, "SSP passkey request -> fixed 1234");
        esp_bt_gap_ssp_passkey_reply(param->key_req.bda, true, 1234u);
        break;
    case ESP_BT_GAP_ACL_CONN_CMPL_STAT_EVT:
        /* 0.127：**先记账**，再沿用原逻辑（工单 11 §2.3）。 */
        portENTER_CRITICAL(&s_pcm_mux);
        if (param->acl_conn_cmpl_stat.stat == ESP_BT_STATUS_SUCCESS) {
            s_acl_conn_ok++;
            if (s_acl_up && memcmp(s_acl_peer, param->acl_conn_cmpl_stat.bda, 6) != 0) {
                s_acl_other_peer++;   /* 异 peer 事件单列，不悄悄并入目标 peer */
            }
            acl_record_t *r = acl_record(param->acl_conn_cmpl_stat.bda, true);
            const uint32_t incarnation = ++s_acl_sequence;
            if (r) {
                memcpy(r->peer, param->acl_conn_cmpl_stat.bda, 6);
                r->handle = param->acl_conn_cmpl_stat.handle;
                r->incarnation = incarnation; r->up = true;
            }
            if (!s_peer_valid || !memcmp(s_audio_peer, param->acl_conn_cmpl_stat.bda, 6))
                acl_select_peer(param->acl_conn_cmpl_stat.bda);
        } else {
            s_acl_conn_fail++;
        }
        portEXIT_CRITICAL(&s_pcm_mux);
        if (param->acl_conn_cmpl_stat.stat != ESP_BT_STATUS_SUCCESS) {
            /* 0.128 §B.2：失败事件也留 peer/stat 短 LOG，供异 peer 归因（不能漏）。 */
            ESP_LOGW(TAG, "ACL connect failed: %02x:%02x:%02x:%02x:%02x:%02x stat=%d",
                     param->acl_conn_cmpl_stat.bda[0], param->acl_conn_cmpl_stat.bda[1],
                     param->acl_conn_cmpl_stat.bda[2], param->acl_conn_cmpl_stat.bda[3],
                     param->acl_conn_cmpl_stat.bda[4], param->acl_conn_cmpl_stat.bda[5],
                     (int)param->acl_conn_cmpl_stat.stat);
            break;
        }
        ESP_LOGI(TAG, "ACL connected: %02x:%02x:%02x:%02x:%02x:%02x handle=%u",
                 param->acl_conn_cmpl_stat.bda[0], param->acl_conn_cmpl_stat.bda[1],
                 param->acl_conn_cmpl_stat.bda[2], param->acl_conn_cmpl_stat.bda[3],
                 param->acl_conn_cmpl_stat.bda[4], param->acl_conn_cmpl_stat.bda[5],
                 (unsigned)param->acl_conn_cmpl_stat.handle);
        break;
    case ESP_BT_GAP_ACL_DISCONN_CMPL_STAT_EVT:
        /* 0.127：断开事件**必留** peer/handle/reason（原先只打一句 "ACL disconnected"）。 */
        portENTER_CRITICAL(&s_pcm_mux);
        s_acl_disconn++;
        s_last_acl_disc_handle = param->acl_disconn_cmpl_stat.handle;
        s_last_acl_disc_reason = (uint32_t)param->acl_disconn_cmpl_stat.reason;
        memcpy(s_last_acl_disc_peer, param->acl_disconn_cmpl_stat.bda, 6);
        acl_record_t *disconnected = acl_record(param->acl_disconn_cmpl_stat.bda, false);
        if (disconnected) disconnected->up = false;
        if (s_peer_valid && memcmp(s_audio_peer, param->acl_disconn_cmpl_stat.bda, 6) == 0) {
            s_peer_valid = false;
            s_acl_up = false;
            ++s_link_generation;
            s_worker_format_ready = false;
            bt_route_hold_select(&s_route_hold, false);
            esp_a2dp_codec_policy_stream_boundary();
            s_pcm_autostart_allowed = false;
            s_bt_connected = false;
            s_bt_streaming = false;
            s_stream_start_us = 0;
            s_cfg_pending = false;
            s_cfg_bits = 0;
            s_cfg_generation++;
            pcm_epoch_flush();
        }
        portEXIT_CRITICAL(&s_pcm_mux);
        /* GAP may be the only terminal notification. The guard filters peer
         * and ignores a second DISCONNECTED, preserving its one-shot backoff. */
        bt_codec_guard_connection(param->acl_disconn_cmpl_stat.bda,
                                  ESP_A2D_CONNECTION_STATE_DISCONNECTED);
        ESP_LOGI(TAG, "ACL disconnected: %02x:%02x:%02x:%02x:%02x:%02x handle=%u reason=0x%04x",
                 param->acl_disconn_cmpl_stat.bda[0], param->acl_disconn_cmpl_stat.bda[1],
                 param->acl_disconn_cmpl_stat.bda[2], param->acl_disconn_cmpl_stat.bda[3],
                 param->acl_disconn_cmpl_stat.bda[4], param->acl_disconn_cmpl_stat.bda[5],
                 (unsigned)param->acl_disconn_cmpl_stat.handle,
                 (unsigned)param->acl_disconn_cmpl_stat.reason);
        break;
    default:
        break;
    }
}

/* ---------------- A2DP event / PCM callbacks ---------------- */

static void bt_app_a2d_hdl(uint16_t event, void *param, uint32_t generation)
{
    esp_a2d_cb_param_t *a2d = (esp_a2d_cb_param_t *)param;

    switch (event) {
    case ESP_A2D_PROF_STATE_EVT:
        if (a2d->a2d_prof_stat.init_state == ESP_A2D_INIT_SUCCESS) {
            ESP_LOGI(TAG, "A2DP init complete");
        }
        break;

    case ESP_A2D_CONNECTION_STATE_EVT: {
        uint8_t *bda = a2d->conn_stat.remote_bda;
        if (a2d->conn_stat.state == ESP_A2D_CONNECTION_STATE_CONNECTED) {
            ESP_LOGI(TAG, "A2DP connected: %02x:%02x:%02x:%02x:%02x:%02x",
                     bda[0], bda[1], bda[2], bda[3], bda[4], bda[5]);
            /* 0.128（工单 12 §B.1）：**ACL 代次不在这里递增**。
             * A2DP profile CONNECTED ≠ ACL 新建连接；切 codec 会重建 profile 而不动 ACL，
             * 在这里递增会把"同 ACL 的 profile 重建"误判成"ACL 断过"。
             * 代次只在 GAP `ACL_CONN_CMPL_STAT` 成功分支递增（见 bt_app_gap_cb）。 */
        } else if (a2d->conn_stat.state == ESP_A2D_CONNECTION_STATE_DISCONNECTED) {
            ESP_LOGW(TAG, "A2DP disconnected reason=%d", (int)a2d->conn_stat.disc_rsn);
        }
        break;
    }

    case ESP_A2D_AUDIO_STATE_EVT:
        if (a2d->audio_stat.state == ESP_A2D_AUDIO_STATE_STARTED) {
            ESP_LOGI(TAG, "A2DP audio started");
        } else if (a2d->audio_stat.state == ESP_A2D_AUDIO_STATE_SUSPEND) {
            ESP_LOGI(TAG, "A2DP audio suspended");
        }
        break;

    case ESP_A2D_AUDIO_CFG_EVT:
        log_codec_cfg(generation);
        ESP_LOGI(TAG, "trace-bt: us=%lu event=cfg_notify generation=%lu control=NONE",
                 (unsigned long)(uint32_t)esp_timer_get_time(), (unsigned long)generation);
        break;

    case ESP_A2D_SNK_GET_DELAY_VALUE_EVT:
        break;

    default:
        break;
    }
}

/* Transport events publish before queued logging. PCM configuration belongs
 * exclusively to bt_app_codec_configuration in the actual decoder worker. */
static uint32_t publish_audio_event(esp_a2d_cb_event_t event, esp_a2d_cb_param_t *param)
{
    const int64_t now_us = esp_timer_get_time();
    bool began = false, restored = false;
    portENTER_CRITICAL(&s_pcm_mux);
    const uint32_t grace_ms = (s_codec_vendor_id == 0x0000012Du &&
                               s_codec_codec_id == 0x00AAu)
                              ? BT_LDAC_SUSPEND_GRACE_MS : BT_SUSPEND_GRACE_MS;
    if (event == ESP_A2D_CONNECTION_STATE_EVT) {
        if (param->conn_stat.state == ESP_A2D_CONNECTION_STATE_CONNECTED) {
            const bool new_profile = !s_bt_connected;
            memcpy(s_audio_peer, param->conn_stat.remote_bda, 6);
            s_peer_valid = true;
            acl_select_peer(s_audio_peer);
            s_bt_connected = true;
            /* A genuine new profile may confirm a missed START with its first
             * real PCM. Duplicate CONNECTED cannot undo an explicit SUSPEND.
             * A failed/quarantined setup never qualifies for cache restore. */
            if (new_profile) s_pcm_autostart_allowed = true;
            if (new_profile && s_worker_format_ready &&
                s_worker_link_generation == s_link_generation &&
                esp_a2dp_codec_policy_allowed(s_worker_format.id)) {
                publish_worker_format_locked(&s_worker_format);
                s_cfg_pending = false;
                s_cfg_failed = false;
                restored = true;
            }
            /* A worker CFG can finish before the BTC CONNECTED event. Only
             * an actual live profile promotes its finite close grace to a
             * replacement hold; a late CFG on an idle ACL cannot do so. */
            if (s_cfg_bits && !s_cfg_failed && s_worker_cfg_epoch)
                began = bt_route_hold_begin(&s_route_hold, route_link_up(), now_us,
                                            (int64_t)BT_RECONFIG_WARN_MS * 1000);
        } else if (param->conn_stat.state == ESP_A2D_CONNECTION_STATE_DISCONNECTING ||
                   param->conn_stat.state == ESP_A2D_CONNECTION_STATE_DISCONNECTED) {
            esp_a2dp_codec_policy_stream_boundary();
            s_pcm_autostart_allowed = false;
            /* Profile close alone is not proof of a replacement codec. Give
             * an arriving CFG the existing two-second window, then allow UAC
             * if the peer kept an idle ACL without ever resuming audio. */
            began = s_route_hold.selected && route_link_up();
            if (!s_cfg_failed) {
                bt_route_hold_suspend(&s_route_hold, route_link_up(), now_us,
                                      (int64_t)BT_RECONFIG_WARN_MS * 1000);
            } else {
                /* A guard/config failure is terminal for this audio stream,
                 * not a codec replacement to reserve against UAC. */
                bt_route_hold_clear(&s_route_hold);
            }
            s_bt_connected = false;
            s_bt_streaming = false;
            s_stream_start_us = 0;
            s_cfg_pending = false;
            s_cfg_bits = 0;
            s_cfg_generation++;
            /* ★ 0.163：跨会话隔离 —— 断连后旧会话"播出过"的资格作废 */

            /* Retain audio peer while ACL is up: profile replacement must not
             * turn its own route hold off. GAP disconnect clears the binding. */
            pcm_epoch_flush();
            if (param->conn_stat.state == ESP_A2D_CONNECTION_STATE_DISCONNECTED &&
                param->conn_stat.disc_rsn == ESP_A2D_DISC_RSN_ABNORMAL) {
                bt_route_hold_clear(&s_route_hold);
            }
        }
    } else if (event == ESP_A2D_AUDIO_STATE_EVT) {
        s_pcm_autostart_allowed = false;
        if (param->audio_stat.state == ESP_A2D_AUDIO_STATE_STARTED) {
            /* A late START confirming first-PCM autostart is idempotent. It
             * must not invalidate that already valid decoder call or restart
             * the takeover timer. SUSPEND/CFG already fenced the old stream. */
            if (!s_bt_streaming) s_stream_start_us = now_us;
            s_bt_streaming = true;
            s_start_epoch = s_route_hold.epoch;
            /* Same-CFG resumes (e.g. LDAC quality) also need their prebuffer.
             * Restart a finite wait at START, not an indefinite CFG hold.
             * A false START with no PCM therefore still expires. */
            if (s_route_hold.waiting && !s_route_hold.reconfig) {
                bt_route_hold_suspend(&s_route_hold, route_link_up(), now_us,
                                      (int64_t)grace_ms * 1000);
            }
        } else if (param->audio_stat.state == ESP_A2D_AUDIO_STATE_SUSPEND) {
            esp_a2dp_codec_policy_stream_boundary();
            /* No CFG yet is ambiguous: pause or the start of a codec change.
             * Reserve only a selected BT route. Do not cancel a pending CFG
             * or a newer epoch's wait with the preceding stream's SUSPEND.
             * A stop after the new START becomes a bounded pause again. */
            if (!s_cfg_pending && !s_cfg_failed &&
                (!s_route_hold.reconfig || s_start_epoch == s_route_hold.epoch)) {
                bt_route_hold_suspend(&s_route_hold, route_link_up(), now_us,
                                      (int64_t)grace_ms * 1000);
            }
            s_bt_streaming = false;
            s_stream_start_us = 0;
            /* Discard the departing generation immediately. Keep route hold
             * independent: a codec replacement must not expose background UAC.
             * No tail replay, sample rewriting or codec-change fade is applied. */
            pcm_epoch_flush();
        }
    }
    uint32_t generation = s_cfg_generation;
    uint32_t epoch = s_route_hold.epoch;
    const uint32_t worker_epoch = s_worker_cfg_epoch;
    const char *hold_mode = !s_route_hold.waiting ? "none" :
                           s_route_hold.reconfig ? "reconfig" : "grace";
    portEXIT_CRITICAL(&s_pcm_mux);
    if (restored) ESP_LOGI(TAG, "trace-bt: us=%lu event=worker_format_restore generation=%lu epoch=%lu sdk_epoch=%lu stream_started=0",
        (unsigned long)(uint32_t)now_us, (unsigned long)generation,
        (unsigned long)epoch, (unsigned long)worker_epoch);
    if (began) {
        ESP_LOGI(TAG, "trace-bt: us=%lu event=route_hold_begin epoch=%lu warn_ms=%u cause=%s",
                 (unsigned long)(uint32_t)now_us, (unsigned long)epoch, BT_RECONFIG_WARN_MS,
                 event == ESP_A2D_AUDIO_CFG_EVT ? "cfg" : "profile");
    }
    if (event == ESP_A2D_AUDIO_STATE_EVT) {
        ESP_LOGI(TAG, "trace-bt: us=%lu event=audio_state state=%u generation=%lu epoch=%lu hold=%s grace_ms=%u",
                 (unsigned long)(uint32_t)now_us, (unsigned)param->audio_stat.state,
                 (unsigned long)generation, (unsigned long)epoch, hold_mode, grace_ms);
    }
    return generation;
}

static void bt_app_a2d_cb(esp_a2d_cb_event_t event, esp_a2d_cb_param_t *param)
{
    const uint8_t *peer = event == ESP_A2D_CONNECTION_STATE_EVT ? param->conn_stat.remote_bda :
                          event == ESP_A2D_AUDIO_STATE_EVT ? param->audio_stat.remote_bda :
                          event == ESP_A2D_AUDIO_CFG_EVT ? param->audio_cfg.remote_bda : NULL;
    if (peer) {
        portENTER_CRITICAL(&s_pcm_mux);
        const bool foreign = s_peer_valid && memcmp(s_audio_peer, peer, 6) != 0;
        if (!foreign && event == ESP_A2D_CONNECTION_STATE_EVT &&
            param->conn_stat.state == ESP_A2D_CONNECTION_STATE_CONNECTING) {
            memcpy(s_audio_peer, peer, 6); s_peer_valid = true;
            acl_select_peer(peer);
        }
        portEXIT_CRITICAL(&s_pcm_mux);
        if (foreign) {
            if (event == ESP_A2D_CONNECTION_STATE_EVT &&
                param->conn_stat.state == ESP_A2D_CONNECTION_STATE_CONNECTED)
                esp_a2d_sink_disconnect((uint8_t *)peer);
            ESP_LOGW(TAG, "foreign A2DP event=%u ignored; one Classic audio owner", (unsigned)event);
            return;
        }
    }
    /* Publish guard peer/state in the earliest BTC callback, before deferred
     * application work: a busy decoder must not starve its own peer binding. */
    if (event == ESP_A2D_CONNECTION_STATE_EVT) {
        bt_codec_guard_connection(param->conn_stat.remote_bda,param->conn_stat.state);
        if (param->conn_stat.state == ESP_A2D_CONNECTION_STATE_CONNECTED)
            esp_bt_gap_set_scan_mode(ESP_BT_NON_CONNECTABLE, ESP_BT_NON_DISCOVERABLE);
        else if (param->conn_stat.state == ESP_A2D_CONNECTION_STATE_DISCONNECTED)
            esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
    } else if (event == ESP_A2D_PROF_STATE_EVT &&
               param->a2d_prof_stat.init_state == ESP_A2D_INIT_SUCCESS) {
        bt_codec_guard_profile_ready();
        esp_a2d_sink_get_delay_value();
    } else if (event == ESP_A2D_SNK_GET_DELAY_VALUE_EVT) {
        esp_a2d_sink_set_delay_value(param->a2d_get_delay_value_stat.delay_value + 50);
    }
    switch (event) {
    case ESP_A2D_PROF_STATE_EVT:
    case ESP_A2D_SNK_PSC_CFG_EVT:
    case ESP_A2D_SNK_SET_DELAY_VALUE_EVT:
    case ESP_A2D_SNK_GET_DELAY_VALUE_EVT:
    case ESP_A2D_CONNECTION_STATE_EVT:
    case ESP_A2D_AUDIO_STATE_EVT:
    case ESP_A2D_AUDIO_CFG_EVT:
    case ESP_A2D_SEP_REG_STATE_EVT:
    {
        uint32_t generation = publish_audio_event(event, param);
        if (!bt_app_work_dispatch(bt_app_a2d_hdl, (uint16_t)event, param,
                                  sizeof(esp_a2d_cb_param_t), generation)) {
            /* Early state/config is already committed. Only the diagnostic
             * copy was dropped; do not undo a valid stream on logging OOM. */
            ESP_LOGE(TAG, "trace-bt: event=dispatch_failed id=%u generation=%lu",
                     (unsigned)event, (unsigned long)generation);
        }
        break;
    }
    default:
        break;
    }
}

static void bt_app_a2d_data_cb(const uint8_t *data, uint32_t len)
{
    if (data == NULL || len == 0) {
        return;
    }
    const int64_t now_us = esp_timer_get_time();
    portENTER_CRITICAL(&s_pcm_mux);
    int bits = s_cfg_bits;
    int channels = s_cfg_channels;
    uint32_t epoch = s_route_hold.epoch;
    /* Decoder callbacks are synchronous within decode_begin/end. Check the
     * captured transport token while holding the same lock as the app's epoch:
     * SUSPEND/CFG cannot slip between this check and acceptance. Rejected raw
     * bytes have no trustworthy new-format frame count. */
    if (!s_inject_active && !esp_a2dp_codec_policy_accept_pcm(len)) {
        s_epoch_rejects++;
        rej_note(A2DP_REJ_EPOCH_STALE, len, 0, epoch, s_route_hold.epoch);
        portEXIT_CRITICAL(&s_pcm_mux);
        return;
    }
    if (!s_inject_active && s_pcm_autostart_allowed && s_bt_connected &&
        !s_bt_streaming && !s_cfg_pending && !s_cfg_failed &&
        (bits == 16 || bits == 24 || bits == 32) && (channels == 1 || channels == 2)) {
        s_pcm_autostart_allowed = false;
        s_bt_streaming = true;
        s_stream_start_us = now_us;
        s_start_epoch = epoch;
        s_stream_autostart++;
        s_autostart_bits = (uint32_t)bits;
        s_autostart_channels = (uint32_t)channels;
    }
    /* ★ 0.151：PCM 注入模式（用户要求"不再依赖手机，自己切码自己抓包"）。
     * 注入时没有真实 A2DP 连接，但**下游全部走真实路径**（ring/gate/pump/core1），
     * 因此这里只跳过"必须有真蓝牙"的那几个状态位，格式与长度检查照旧。 */
    if (s_inject_active) {
        if ((bits != 16 && bits != 24 && bits != 32) ||
            (channels != 1 && channels != 2) ||
            len % ((uint32_t)(bits / 8) * (uint32_t)channels) != 0) {
            s_pcm_drops++;
            portEXIT_CRITICAL(&s_pcm_mux);
            return;
        }
        s_pcm_packets++;
        portEXIT_CRITICAL(&s_pcm_mux);
        push_pcm_to_ring(data, len, bits, channels, epoch);
        return;
    }
    if (!s_bt_connected || !s_bt_streaming || s_cfg_pending || s_cfg_failed ||
        (bits != 16 && bits != 24 && bits != 32) ||
        (channels != 1 && channels != 2) || len % ((bits / 8) * channels) != 0) {
        s_pcm_drops++;
        /* 0.127 记账：按理由细分；**没有有效格式时只记 raw 字节，不猜帧数**。 */
        a2dp_reject_reason_t why;
        uint32_t conv = 0;
        if (!s_bt_connected) {
            why = A2DP_REJ_NOT_CONNECTED;
        } else if (!s_bt_streaming) {
            why = A2DP_REJ_NOT_STREAMING;
        } else if (s_cfg_failed) {
            why = A2DP_REJ_CFG_FAILED;
        } else if (s_cfg_pending) {
            why = A2DP_REJ_CFG_PENDING;
        } else {
            why = A2DP_REJ_FORMAT_LEN;
            /* 0.128（工单 12 §A）：格式/长度不成立时**归一化字节没有完整有效口径**，
             * 不对非整帧 len 做 floor 除法冒充全量拒收。
             * 这里只在 len 恰好是**整数个有效帧**时才给 conv；否则 conv 记 0，
             * 并如实声明 **conv_b 只覆盖可换算的完整有效输入，不能与 raw_b 相加**。 */
            const bool fmt_ok = (bits == 16 || bits == 24 || bits == 32) &&
                                (channels == 1 || channels == 2);
            if (fmt_ok) {
                const size_t frame_bytes = (size_t)(bits / 8) * (size_t)channels;
                if (frame_bytes != 0 && (len % frame_bytes) == 0) {
                    conv = (uint32_t)((len / frame_bytes) * 8);
                }
            }
        }
        rej_note(why, len, conv, epoch, s_route_hold.epoch);
        portEXIT_CRITICAL(&s_pcm_mux);
        return;
    }

    s_pcm_packets++;
    s_pcm_epoch = epoch;
    s_start_epoch = epoch;
    if (s_stream_start_us == 0) s_stream_start_us = now_us;
    portEXIT_CRITICAL(&s_pcm_mux);
    push_pcm_to_ring(data, len, bits, channels, epoch);
}

#if CONFIG_BT_A2DP_AVRCP_ROLE
/* AVRCP must precede A2DP initialization: the SDK snapshots g_av_with_rc
 * when registering A2DP/SDP. These callbacks observe transport state only; no
 * remote player commands, metadata polling, PCM gain changes or persistent
 * writes are issued here. Kept behind BT_A2DP_AVRCP_ROLE because a sink that
 * opens the AVCTP channel first makes Windows close the paired A2DP link. */
static void bt_app_avrc_ct_cb(esp_avrc_ct_cb_event_t event, esp_avrc_ct_cb_param_t *param)
{
    if (event == ESP_AVRC_CT_PROF_STATE_EVT) {
        ESP_LOGI(TAG, "AVRCP CT init state=%d", param->avrc_ct_init_stat.state);
    } else if (event == ESP_AVRC_CT_CONNECTION_STATE_EVT) {
        ESP_LOGI(TAG, "AVRCP CT connected=%u", param->conn_stat.connected);
    }
}

static void bt_app_avrc_tg_cb(esp_avrc_tg_cb_event_t event, esp_avrc_tg_cb_param_t *param)
{
    if (event == ESP_AVRC_TG_PROF_STATE_EVT) {
        ESP_LOGI(TAG, "AVRCP TG init state=%d", param->avrc_tg_init_stat.state);
    } else if (event == ESP_AVRC_TG_CONNECTION_STATE_EVT) {
        ESP_LOGI(TAG, "AVRCP TG connected=%u", param->conn_stat.connected);
    }
}

static esp_err_t bt_app_avrc_init(void)
{
    esp_err_t err = esp_avrc_ct_register_callback(bt_app_avrc_ct_cb);
    if (err != ESP_OK) return err;
    err = esp_avrc_tg_register_callback(bt_app_avrc_tg_cb);
    if (err != ESP_OK) return err;
    err = esp_avrc_ct_init();
    if (err != ESP_OK) return err;
    err = esp_avrc_tg_init();
    if (err != ESP_OK) return err;
    /* Do not accept player/volume buttons that this bridge does not execute.
     * The SDK's default notification mask is empty; leave it empty rather
     * than advertising remote absolute-volume control over the PCM path. */
    const esp_avrc_psth_bit_mask_t commands = {0};
    return esp_avrc_tg_set_psth_cmd_filter(ESP_AVRC_PSTH_FILTER_SUPPORTED_CMD, &commands);
}
#endif /* CONFIG_BT_A2DP_AVRCP_ROLE */

static void bt_av_hdl_stack_evt(uint16_t event, void *param, uint32_t generation)
{
    (void)param;
    (void)generation;
    if (event != 0) {
        return;
    }

    bt_codec_guard_init();
    esp_a2dp_codec_policy_set_configuration_callback(bt_app_codec_configuration);
    {
        const uint8_t *bda = esp_bt_dev_get_address();
        if (bda) ESP_LOGI(TAG, "local BD address %02x:%02x:%02x:%02x:%02x:%02x",
                          bda[0], bda[1], bda[2], bda[3], bda[4], bda[5]);
    }
    esp_bt_gap_set_device_name(CONFIG_BT_A2DP_DEVICE_NAME);
    esp_bt_dev_register_callback(bt_app_dev_cb);
    esp_bt_gap_register_callback(bt_app_gap_cb);

    esp_err_t err = ESP_OK;
#if CONFIG_BT_A2DP_AVRCP_ROLE
    err = bt_app_avrc_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "AVRCP init request failed: %s", esp_err_to_name(err));
        return;
    }
#endif
    esp_a2d_register_callback(&bt_app_a2d_cb);
    err = esp_a2d_sink_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_a2d_sink_init failed: %s", esp_err_to_name(err));
        return;
    }
    esp_a2d_sink_register_data_callback(bt_app_a2d_data_cb);
    esp_bt_gap_get_device_name();
    esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
    ESP_LOGI(TAG, "A2DP internal-codec sink ready: %s", CONFIG_BT_A2DP_DEVICE_NAME);
}

/* ---------------- Public API ---------------- */

esp_err_t a2dp_audio_init(void)
{
    esp_err_t err;
#if CONFIG_BT_A2DP_APTX_DECODER
    extern const uint32_t a2dp_aptx_budget_patch_abi;
    if (a2dp_aptx_budget_patch_abi != 0x01410001u) return ESP_ERR_INVALID_VERSION;
#endif
#if CONFIG_BT_A2DP_LDAC_DECODER
    extern const uint32_t a2dp_ldac_packet_patch_abi;
    if (a2dp_ldac_packet_patch_abi != 0x01410001u) return ESP_ERR_INVALID_VERSION;
#endif

    /* ★ 任务书 42 §3（BLE 调音，0.69）：控制器从"仅经典"改为双模 BTDM，
     * 并**不再** `esp_bt_controller_mem_release(ESP_BT_MODE_BLE)` —— 旧代码
     * 把 BLE 内存放掉了，BLE 栈起不来。改这三处是"开 BLE 不弄坏 A2DP"
     * 铁律（41 §6.5）的唯一固件落点，A/B 验证过才许接界面。 */
    ESP_LOGI(TAG, "BT controller init (BTDM capability; A2DP service)");

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    err = esp_bt_controller_init(&bt_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "bt controller init failed: %s", esp_err_to_name(err));
        return err;
    }

    err = esp_bt_controller_enable(ESP_BT_MODE_BTDM);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "bt controller enable failed: %s", esp_err_to_name(err));
        return err;
    }

    esp_bluedroid_config_t bluedroid_cfg = BT_BLUEDROID_INIT_CONFIG_DEFAULT();
#if defined(CONFIG_BT_A2DP_SSP_ENABLED) && !CONFIG_BT_A2DP_SSP_ENABLED
    bluedroid_cfg.ssp_en = false;
#endif
    err = esp_bluedroid_init_with_cfg(&bluedroid_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "bluedroid init failed: %s", esp_err_to_name(err));
        return err;
    }

    err = esp_bluedroid_enable();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "bluedroid enable failed: %s", esp_err_to_name(err));
        return err;
    }

    /* Headless sink: SSP "Just Works" instead of passkey entry. The fixed PIN
     * remains as a legacy-pairing fallback. */
    esp_bt_io_cap_t iocap = ESP_BT_IO_CAP_NONE;
    esp_err_t iocap_err = esp_bt_gap_set_security_param(ESP_BT_SP_IOCAP_MODE, &iocap, sizeof(iocap));
    if (iocap_err != ESP_OK) {
        ESP_LOGW(TAG, "set io cap failed: %s", esp_err_to_name(iocap_err));
    }
    esp_bt_pin_type_t pin_type = ESP_BT_PIN_TYPE_FIXED;
    esp_bt_pin_code_t pin_code = { '1', '2', '3', '4' };
    esp_bt_gap_set_pin(pin_type, 4, pin_code);

    s_bt_app_q = xQueueCreate(10, sizeof(bt_app_msg_t));
    if (s_bt_app_q == NULL) {
        ESP_LOGE(TAG, "bt app queue create failed");
        return ESP_ERR_NO_MEM;
    }

    BaseType_t created = xTaskCreatePinnedToCore(bt_app_task_handler, "bt_app", 4096,
                                                 NULL, 10, &s_bt_app_task, 0);
    if (created != pdPASS) {
        ESP_LOGE(TAG, "bt app task create failed");
        return ESP_ERR_NO_MEM;
    }

    if (!bt_app_work_dispatch(bt_av_hdl_stack_evt, 0, NULL, 0, 0)) {
        ESP_LOGE(TAG, "A2DP startup dispatch failed");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "A2DP source initialized, free heap=%lu", (unsigned long)esp_get_free_heap_size());
    return ESP_OK;
}

bool a2dp_audio_is_connected(void)
{
    portENTER_CRITICAL(&s_pcm_mux);
    bool connected = s_bt_connected;
    portEXIT_CRITICAL(&s_pcm_mux);
    return connected;
}

bool a2dp_audio_is_streaming(void)
{
    portENTER_CRITICAL(&s_pcm_mux);
    bool streaming = s_bt_streaming;
    portEXIT_CRITICAL(&s_pcm_mux);
    return streaming;
}

bool a2dp_audio_is_decoder_ready(void)
{
    portENTER_CRITICAL(&s_pcm_mux);
    bool ready = decoder_ready_locked();
    portEXIT_CRITICAL(&s_pcm_mux);
    return ready;
}

bool a2dp_audio_is_codec_busy(void)
{
    portENTER_CRITICAL(&s_pcm_mux);
    bool busy = decoder_busy_locked();
    portEXIT_CRITICAL(&s_pcm_mux);
    return busy;
}

bool a2dp_audio_is_acl_connected(void)
{
    portENTER_CRITICAL(&s_pcm_mux);
    bool up = route_link_up();
    portEXIT_CRITICAL(&s_pcm_mux);
    return up;
}

void a2dp_audio_set_output_selected(bool selected)
{
    portENTER_CRITICAL(&s_pcm_mux);
    bt_route_hold_select(&s_route_hold, selected);
    portEXIT_CRITICAL(&s_pcm_mux);
}

bool a2dp_audio_route_hold_active(void)
{
    const int64_t now_us = esp_timer_get_time();
    portENTER_CRITICAL(&s_pcm_mux);
    bool active = bt_route_hold_active(&s_route_hold, route_link_up(), now_us);
    bool expired = active && s_route_hold.reconfig && s_route_hold.until_us != 0 &&
                   now_us >= s_route_hold.until_us;
    uint32_t epoch = s_route_hold.epoch;
    if (expired) s_route_hold.until_us = 0;
    portEXIT_CRITICAL(&s_pcm_mux);
    if (expired) {
        ESP_LOGW(TAG, "trace-bt: us=%lu event=route_hold_wait epoch=%lu (await PCM/terminal event)",
                 (unsigned long)(uint32_t)now_us, (unsigned long)epoch);
    }
    return active;
}

uint32_t a2dp_audio_get_stream_epoch(void)
{
    portENTER_CRITICAL(&s_pcm_mux);
    uint32_t epoch = s_route_hold.epoch;
    portEXIT_CRITICAL(&s_pcm_mux);
    return epoch;
}

void a2dp_audio_route_ready(uint32_t epoch)
{
    const int64_t now_us = esp_timer_get_time();
    portENTER_CRITICAL(&s_pcm_mux);
    bool ended = false;
    if (s_bt_streaming && !s_cfg_pending && !s_cfg_failed && s_pcm_epoch == epoch) {
        ended = bt_route_hold_ready(&s_route_hold, route_link_up(), epoch);
    }
    portEXIT_CRITICAL(&s_pcm_mux);
    if (ended) {
        ESP_LOGI(TAG, "trace-bt: us=%lu event=route_hold_ready epoch=%lu",
                 (unsigned long)(uint32_t)now_us, (unsigned long)epoch);
    }
}

uint32_t a2dp_audio_get_sample_rate(void)
{
    portENTER_CRITICAL(&s_pcm_mux);
    uint32_t rate = s_cfg_rate;
    portEXIT_CRITICAL(&s_pcm_mux);
    return rate;
}

uint32_t a2dp_audio_get_stream_duration_ms(void)
{
    a2dp_audio_state_t state;
    a2dp_audio_get_state(&state);
    return state.duration_ms;
}

void a2dp_audio_get_state(a2dp_audio_state_t *state)
{
    const int64_t now_us = esp_timer_get_time();
    portENTER_CRITICAL(&s_pcm_mux);
    state->connected = s_bt_connected;
    state->streaming = s_bt_streaming;
    state->decoder_ready = decoder_ready_locked();
    state->codec_busy = decoder_busy_locked();
    state->cfg_failed = s_cfg_failed;
    state->acl_connected = route_link_up();
    state->route_hold = bt_route_hold_active(&s_route_hold, state->acl_connected, now_us);
    state->epoch = s_route_hold.epoch;
    state->sample_rate = s_cfg_rate;
    state->packets = s_pcm_packets;
    state->level_bytes = pcm_ring_avail();
    state->duration_ms = (s_bt_streaming && s_stream_start_us != 0 && now_us > s_stream_start_us)
                          ? (uint32_t)((now_us - s_stream_start_us) / 1000) : 0;
    /* ★ 0.150：把已应用的 codec 标识暴露给源策略，用于按编码器选择起播水位 */
    state->codec_vendor_id = s_codec_vendor_id;
    state->codec_codec_id = s_codec_codec_id;
    state->codec_type = (uint8_t)s_codec_type;
    portEXIT_CRITICAL(&s_pcm_mux);
}

bool a2dp_audio_fail_codec(uint32_t codec_id, uint32_t codec_epoch)
{
    if (codec_id >= ESP_A2DP_CODEC_COUNT) return false;
    const esp_a2dp_codec_descriptor_t *d = esp_a2dp_codec_policy_descriptor(codec_id);
    esp_a2dp_codec_stats_t stats;
    portENTER_CRITICAL(&s_pcm_mux);
    /* PCM -> policy is the existing lock order. No policy callback holds its
     * lock while entering this function. An app CFG can lead worker setup:
     * match the published CIE too, not merely the worker's still-old epoch. */
    const uint8_t type = codec_id == ESP_A2DP_CODEC_SBC ? ESP_A2D_MCT_SBC :
                         codec_id == ESP_A2DP_CODEC_AAC ? ESP_A2D_MCT_M24 : ESP_A2D_MCT_NON_A2DP;
    const bool current = !s_inject_active &&
        esp_a2dp_codec_policy_current() == codec_id &&
        esp_a2dp_codec_policy_snapshot(codec_id, &stats) &&
        stats.epoch == codec_epoch && stats.configuration_complete &&
        !esp_a2dp_codec_policy_allowed(codec_id) && s_codec_type == type &&
        s_codec_vendor_id == d->vendor_id && s_codec_codec_id == d->codec_id &&
        s_cfg_rate == stats.format.rate;
    if (current) {
        esp_a2dp_codec_policy_stream_boundary();
        s_worker_format_ready = false;
        s_pcm_autostart_allowed = false;
        s_cfg_pending = false;
        s_cfg_failed = true;
        s_bt_streaming = false;
        s_stream_start_us = 0;

        bt_route_hold_clear(&s_route_hold);
        pcm_epoch_flush();
    }
    const uint32_t epoch = s_pcm_epoch;
    portEXIT_CRITICAL(&s_pcm_mux);
    if (current) ESP_LOGW(TAG, "trace-bt: event=codec_failed id=%lu cfg_epoch=%lu pcm_epoch=%lu hold_released=1",
        (unsigned long)codec_id, (unsigned long)codec_epoch, (unsigned long)epoch);
    return current;
}



bool a2dp_audio_commit_output(bool selected, uint32_t epoch, uint32_t rate,
                              uint32_t packets)
{
    const int64_t now_us = esp_timer_get_time();
    portENTER_CRITICAL(&s_pcm_mux);
    bool current = epoch == s_route_hold.epoch;
    if (selected) {
        current = current && route_link_up() && s_bt_connected && s_bt_streaming &&
                  !s_cfg_pending && !s_cfg_failed && s_cfg_rate == rate && s_pcm_epoch == epoch;
    } else {
        /* Terminal failure/disconnect cannot cancel UAC just by advancing a
         * dead BT epoch. A live replacement still needs the strict generation,
         * hold and arrival cut, so codec switching never releases UAC early. */
        const bool terminal = !route_link_up() || s_cfg_failed;
        current = terminal || (current &&
                  !bt_route_hold_active(&s_route_hold, true, now_us) && s_pcm_packets == packets);
    }
    if (current) {
        bt_route_hold_select(&s_route_hold, selected);
        if (!selected) pcm_epoch_flush();
    }
    portEXIT_CRITICAL(&s_pcm_mux);
    return current;
}

esp_err_t a2dp_audio_submit_pcm32_epoch(int slot, size_t samples, uint32_t flags,
                                      uint32_t epoch)
{
    esp_err_t ret = ESP_ERR_INVALID_STATE;
    portENTER_CRITICAL(&s_pcm_mux);
    if (s_route_hold.selected && epoch == s_route_hold.epoch && s_pcm_epoch == epoch &&
        route_link_up() && s_bt_connected &&
        s_bt_streaming &&
        !s_cfg_pending && !s_cfg_failed && s_cfg_rate == i2s_out_get_rate()) {
        /* xQueueSend(..., 0) inside the frozen producer API never waits.
         * A2DP events cannot invalidate this read between validation and enqueue.
         * Once enqueued it is a legitimate output tail, not revocable BT storage. */
        ret = dsp_pcm_submit(slot, samples, s_cfg_rate, DSP_SOURCE_BT,
                             flags, i2s_out_get_epoch());

    } else {
        s_submit_stale_blocks++;
        s_submit_stale_frames += (uint32_t)(samples / 2u);
    }
    portEXIT_CRITICAL(&s_pcm_mux);
    return ret;
}

size_t a2dp_audio_read_pcm32(uint8_t *dst, size_t max_bytes)
{
    return a2dp_audio_read_pcm32_epoch(dst, max_bytes, a2dp_audio_get_stream_epoch());
}

size_t a2dp_audio_read_pcm32_epoch(uint8_t *dst, size_t max_bytes, uint32_t epoch)
{
    if (dst == NULL || max_bytes == 0 || (max_bytes % 8) != 0) {
        return 0;
    }
    return pcm_ring_pop(dst, max_bytes, epoch);
}

void a2dp_audio_flush_pcm(void)
{
    portENTER_CRITICAL(&s_pcm_mux);
    pcm_epoch_flush();
    portEXIT_CRITICAL(&s_pcm_mux);
}

/* ================= ★ 0.151：PCM 注入（不依赖手机） =================
 * 目的：用户不再愿意反复操作手机做切码实验。这里把 PC 通过 UAC 送来的
 * 真实音乐 PCM 直接喂进 BT ring，并伪造"已连接/已开始推送/已应用 CFG"的
 * 状态，让**下游全部走真实 BT 路径**（ring → gate → pump → core1 → I2S）。
 *
 * 能做到：复现水位/欠载/门控/trim/起播任何设备侧行为，内容可以是复杂音乐。
 * 做不到：蓝牙链路丢包、手机侧重传/编码器内部 —— 那些本来也不在设备侧。
 *
 * 切码：调用 a2dp_audio_inject_enable() 传入新的 vendor/codec/rate 即可，
 * 与真实 CFG 一样会推进 generation、重置水位判定。
 */
void a2dp_audio_inject_enable(uint32_t rate, int bits, int channels,
                              uint32_t vendor_id, uint32_t codec_id)
{
    portENTER_CRITICAL(&s_pcm_mux);
    const bool changed = !s_inject_active || s_cfg_rate != rate || s_cfg_bits != bits ||
                         s_cfg_channels != channels ||
                         s_codec_vendor_id != vendor_id || s_codec_codec_id != codec_id;
    s_inject_active = true;
    s_bt_connected = true;
    s_bt_streaming = true;
    s_cfg_pending = false;
    s_cfg_failed = false;
    s_cfg_rate = rate;
    s_cfg_bits = bits;
    s_cfg_channels = channels;
    s_codec_vendor_id = vendor_id;
    s_codec_codec_id = codec_id;
    if (changed) {
        /* 与真实 CFG 一样推进代次，让水位/门控重新走一遍"攒够再开" */
        s_cfg_generation++;
        s_stream_start_us = esp_timer_get_time();
        s_start_epoch = s_route_hold.epoch;
    }
    portEXIT_CRITICAL(&s_pcm_mux);
}

void a2dp_audio_inject_disable(void)
{
    portENTER_CRITICAL(&s_pcm_mux);
    s_inject_active = false;
    s_bt_streaming = false;
    s_stream_start_us = 0;
    pcm_epoch_flush();
    portEXIT_CRITICAL(&s_pcm_mux);
}

/* 喂一段 PCM（格式必须与 enable 时一致：bits 16/24/32，channels 1/2）。 */
void a2dp_audio_inject_pcm(const uint8_t *data, size_t len)
{
    if (data == NULL || len == 0 || len > 0xFFFFFFFFu) {
        return;
    }
    bt_app_a2d_data_cb(data, (uint32_t)len);
}

size_t a2dp_audio_get_level_bytes(void)
{
    size_t level;
    portENTER_CRITICAL(&s_pcm_mux);
    level = pcm_ring_avail();
    portEXIT_CRITICAL(&s_pcm_mux);
    return level;
}

uint32_t a2dp_audio_get_pcm_packets(void)
{
    portENTER_CRITICAL(&s_pcm_mux);
    uint32_t packets = s_pcm_packets;
    portEXIT_CRITICAL(&s_pcm_mux);
    return packets;
}

void a2dp_audio_get_stats(a2dp_audio_stats_t *stats)
{
    if (stats == NULL) {
        return;
    }

    portENTER_CRITICAL(&s_pcm_mux);
    const int64_t now_us = esp_timer_get_time();
    stats->level_bytes = (size_t)(s_pcm_head - s_pcm_tail);
    stats->capacity_bytes = A2DP_PCM_RING_BYTES;
    stats->min_level_bytes = s_ring_min_bytes;
    stats->max_level_bytes = s_ring_max_bytes;
    s_ring_min_bytes = stats->level_bytes;
    s_ring_max_bytes = stats->level_bytes;
    stats->packets = s_pcm_packets;
    stats->drops = s_pcm_drops;
    stats->ring_overflows = s_ring_overflows;
    stats->flushes = s_ring_flushes;

    /* v25：供 `0xB1` BT_STATUS 上报。`s_codec_type` 是 A2DP 原值
     * （0=SBC / 2=M24(AAC) / 0xFF=NON_A2DP 厂商编解码，见 apply_codec_cfg）。 */
    stats->codec_type = (uint8_t)s_codec_type;
    stats->channels = (uint8_t)(s_cfg_channels > 0 ? s_cfg_channels : 0);
    stats->sample_rate = s_cfg_rate;
    stats->cfg_failures = s_cfg_failures;
    stats->epoch_rejects = s_epoch_rejects;
    stats->flush_bytes = s_flush_bytes;
    stats->prepare_trim_blocks = s_prepare_trim_blocks;
    stats->prepare_trim_frames = s_prepare_trim_frames;
    stats->play_trim_blocks = s_play_trim_blocks;
    stats->play_trim_frames = s_play_trim_frames;
    /* ★ 0.160：排尾账目（drain_blocks 由 pump 统计，见 uac2_audio.c） */
    stats->stream_autostart = s_stream_autostart;
    stats->autostart_bits = s_autostart_bits;
    stats->autostart_channels = s_autostart_channels;
    stats->drain_read_bytes = 0; /* Retired diagnostic; no BT tail playback. */
    /* 0.127：拒收记账与 ACL/codec 代次（工单 11 §2.1 / §2.3 / §3.3）。 */
    stats->reject_total = s_rej_total;
    stats->reject_bytes_raw = s_rej_bytes_raw;
    stats->reject_bytes_conv = s_rej_bytes_conv;
    for (int i = 0; i < A2DP_REJ_REASON_COUNT; i++) {
        stats->reject_by_reason[i] = s_rej_by_reason[i];
    }
    stats->last_reject_reason = s_last_rej_reason;
    stats->last_reject_bytes = s_last_rej_bytes;
    stats->last_reject_cb_epoch = s_last_rej_cb_epoch;
    stats->last_reject_cur_epoch = s_last_rej_cur_epoch;
    stats->acl_incarnation = s_acl_incarnation;
    stats->acl_conn_ok = s_acl_conn_ok;
    stats->acl_conn_fail = s_acl_conn_fail;
    stats->acl_disconn = s_acl_disconn;
    stats->acl_other_peer = s_acl_other_peer;
    stats->acl_handle = s_acl_handle;
    stats->last_acl_disc_handle = s_last_acl_disc_handle;
    stats->last_acl_disc_reason = s_last_acl_disc_reason;
    for (int i = 0; i < 6; i++) {
        stats->last_acl_disc_peer[i] = s_last_acl_disc_peer[i];
    }
    /* 0.128：当前 ACL up/peer 与已知音频 peer（同锁导出，供异 peer 归因） */
    stats->acl_up = s_acl_up;
    for (int i = 0; i < 6; i++) {
        stats->acl_peer[i] = s_acl_peer[i];
        stats->audio_peer[i] = s_audio_peer[i];
    }
    stats->codec_vendor_id = s_codec_vendor_id;
    stats->codec_codec_id = s_codec_codec_id;
    stats->cfg_apply_count = s_cfg_apply_count;
    stats->snapshot_us = (uint32_t)now_us;
    stats->epoch = s_route_hold.epoch;
    stats->head_bytes = s_pcm_head;
    stats->tail_bytes = s_pcm_tail;
    stats->read_bytes = s_pcm_read_bytes;
    stats->submit_stale_blocks = s_submit_stale_blocks;
    stats->submit_stale_frames = s_submit_stale_frames;
    stats->connected = s_bt_connected;
    stats->streaming = s_bt_streaming;
    stats->decoder_ready = decoder_ready_locked();
    stats->codec_busy = decoder_busy_locked();
    stats->route_hold = bt_route_hold_active(&s_route_hold, route_link_up(), now_us);
    stats->duration_ms = (s_bt_streaming && s_stream_start_us != 0 && now_us > s_stream_start_us)
                        ? (uint32_t)((now_us - s_stream_start_us) / 1000) : 0;
    portEXIT_CRITICAL(&s_pcm_mux);
}
