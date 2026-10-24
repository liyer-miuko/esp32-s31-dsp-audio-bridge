/*
 * I2S0 output driver, v2 - ESP32-S31 UAC bridge.
 *
 * ★ 0.113: the I2S0 peripheral is owned by the core1 audio worker. This file
 * keeps the hardware operations (init/start/reconfig/slot/format) and the
 * descriptor-complete output stage; core0 reaches them only through the
 * marshalling wrappers at the bottom, which post a control item into the core1
 * queue and wait for the answer. `i2s_out_write()` is called by that worker
 * directly and must never be called from core0.
 *
 * The DMA on_sent interrupt fires when one 240-frame descriptor has left the
 * engine; it is what paces the core0 USB pump, which then refills a PCM slot.
 */
#include <string.h>
#include "sdkconfig.h"
#include "esp_log.h"
#include "esp_check.h"
#include "driver/i2s_std.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "i2s_private.h" /* temporary read-only probe against the pinned fork */
#include "i2s_out_v2.h"
#include "i2s1_out.h"
#include "pcm_dma_stage.h"
#include "dsp_audio.h"   /* dsp_core1_ctl: core0 -> core1 marshalling */

static const char *TAG = "i2s_out";

/* Control command ids (private to this file; the worker dispatches them). */
enum {
    I2S_CTL_INIT = 1,
    I2S_CTL_START,
    I2S_CTL_RECONFIG,      /* a0 = rate, a1 = bits   */
    I2S_CTL_SLOT_WIDTH,    /* a0 = slot bits         */
    I2S_CTL_FORMAT,        /* a0 = port format 0..2  */
    I2S_CTL_SLOT_RAW,      /* a0 = raw slot flags    */
    I2S_CTL_P1_FORMAT,     /* a0 = secondary format 0..1 */
};

static i2s_chan_handle_t s_tx = NULL;
static volatile bool s_running;
static SemaphoreHandle_t s_tx_done_sem = NULL;
/* One complete output descriptor, after DSP and slot conversion. USB/DMA/BT
 * capacities are unchanged. Added common output delay is at most one block. */
static uint8_t s_stage_bytes[I2S_OUT_DMA_FRAME_BYTES_MAX];
static pcm_dma_stage_t s_stage = { .bytes = s_stage_bytes };
static volatile uint32_t s_stage_pending_bytes = 0;
static void i2s_drop_tx_done_tokens(void)
{
    /* The channel must be disabled: tokens belong to the old DMA generation. */
    while (s_tx_done_sem != NULL && xSemaphoreTake(s_tx_done_sem, 0) == pdTRUE) {
    }
    pcm_dma_stage_reset(&s_stage, i2s_out_get_frame_bytes());
    s_stage_pending_bytes = 0;
}
/* RJ 模式：槽配置用 LSB 位序（硬件据此右对齐），写入口再把样本位序反过来 → 线上 MSB 先发 */
static bool s_rj_reverse = false;
static uint8_t s_rj_scratch[I2S_OUT_DMA_FRAME_NUM * 2 * 4];

/* 24 位位序反转（分治：1→2→4→8 位块交换，最后交换字节 0 与字节 2）
 * 自检：bitrev24(0x000001) == 0x800000，bitrev24(0x800000) == 0x000001 */
static inline uint32_t bitrev24(uint32_t x)
{
    x &= 0x00FFFFFFu;
    x = ((x & 0x00555555u) << 1) | ((x >> 1) & 0x00555555u);
    x = ((x & 0x00333333u) << 2) | ((x >> 2) & 0x00333333u);
    x = ((x & 0x000F0F0Fu) << 4) | ((x >> 4) & 0x000F0F0Fu);
    x = ((x & 0x000000FFu) << 16) | (x & 0x0000FF00u) | ((x >> 16) & 0x000000FFu);
    return x & 0x00FFFFFFu;
}
/* Written on core1, read on both cores (state mirrors for the core0 wrappers
 * and for the conversion decision in the worker). */
static volatile uint32_t s_rate = 48000;
static volatile int s_bits = 32;
static volatile int s_slot_bits = CONFIG_I2S_OUT_SLOT_BITS;
static volatile int s_port_format = 0;   /* 0=Philips I2S, 1=左对齐, 2=右对齐 */
/* Output-configuration generation. Written on core1, read on both cores; it is
 * only meaningful as "did the output configuration change", so a word-sized
 * read is enough. */
static volatile uint32_t s_epoch = 1;

static inline void i2s_epoch_bump(void)
{
    s_epoch++;
}

/* 【诊断】按原始槽标志位配置，用于在真机上扫出"右对齐"的正确组合（v23 临时）。
 * flags: bit0=tc_msb_shift(bit_shift) bit1=left_align bit2=bit_order_lsb
 *        bit3=big_endian bit4=ws_width 取 data_bits(24) 而非 slot(32)
 * 扫出结论后会被正式的 slot_cfg_for(fmt=2) 实现替换。 */
static esp_err_t i2s_out_core1_set_slot_raw(uint8_t flags)
{
    if (s_tx == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    i2s_std_slot_config_t c =
        I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG((i2s_data_bit_width_t)s_bits, I2S_SLOT_MODE_STEREO);
    c.slot_bit_width = (i2s_slot_bit_width_t)s_slot_bits;
    c.ws_width = (flags & 0x10) ? (uint32_t)s_bits : (uint32_t)s_slot_bits;
    c.bit_shift = (flags & 0x01) ? true : false;
    c.left_align = (flags & 0x02) ? true : false;
    c.bit_order_lsb = (flags & 0x04) ? true : false;
    c.big_endian = (flags & 0x08) ? true : false;

    esp_err_t ret = s_running ? i2s_channel_disable(s_tx) : ESP_OK;
    if (ret == ESP_OK) {
        i2s_drop_tx_done_tokens();
        ret = i2s_channel_reconfig_std_slot(s_tx, &c);
    }
    esp_err_t en = s_running ? i2s_channel_enable(s_tx) : ESP_OK;
    if (ret == ESP_OK) {
        ret = en;
        i2s_epoch_bump();
    }
    ESP_LOGW(TAG, "DIAG slot raw flags=0x%02X -> %s", flags, esp_err_to_name(ret));
    return ret;
}

/* 按线格式生成槽配置。
 *
 * 关键认识（v23 反复踩坑后才理清）：**硬件数据宽度必须永远等于槽宽**。
 * 理由是 DMA 里**始终是 32bit 样本**：泵任务用
 * `uac_convert_pcm32_to_slot(..., i2s_out_get_slot_width())` 生成数据，
 * 槽宽 32 时就是 memcpy，24bit 流的数据也是"左对齐放进 32bit 字"（sample << 8）。
 * 逻辑位宽（24/32）只决定样本在 32bit 字里占高多少位，**不是**硬件数据宽度。
 *
 * 早期版本把 `s_bits` 当数据宽度传进来，于是：
 *   init 时 s_bits 默认 32 → "24bit 流 + 右对齐"被按 32bit 判定 → 配成左对齐，
 *   右对齐静默失效（实测：24b_fmt2 线上退化成左对齐，按 start=8 读只剩 0x100 量级）。
 * 所以这里改成：数据宽度 = 槽宽；位序/对齐只看 **payload_bits**（逻辑位宽）。
 *
 * 各格式要点：
 *   fmt0 Philips：bit_shift=true（WS 提前 1 个 BCLK）。
 *   fmt1 左对齐：bit_shift=false。
 *   fmt2 右对齐：tx_left_align 字段在本机实测**无效**（0/1 输出完全一样）；
 *        真正能让硬件把数据右对齐到槽尾的是 **bit_order_lsb = true**（前 8 位补零、
 *        数据落在 bit8..31），但它是 LSB 先发，所以在写入口把样本位序反过来
 *        （见 i2s_out_write 的 s_rj_reverse）→ 线上即「MSB 先发 + 右对齐」。
 *        payload == 槽宽时**不能**开 bit_order_lsb：数据填满整槽，右对齐与左对齐
 *        在位置上完全等价，而开了之后线上会变成 LSB 先发且没有补救余地
 *        （实测证据：32bit 时抓到 0xFFFFFFF8 = bitrev32(0x1FFFFFFF)）。
 */
static i2s_std_slot_config_t slot_cfg_for(int fmt, int source_bits, int slot_bits)
{
    /* The worker has already quantised/repacked DSP output to the SLOT width.
     * A 24-bit slot therefore contains three bytes of 24-bit output, even for
     * a 16-bit source. It must not enter the four-byte RJ compensation below.
     * Preserve the established 24-bit RJ payload in a 32-bit slot for narrow
     * sources; source32/slot32 still uses the complete 32-bit payload. */
    const int payload_bits = (slot_bits == 32 && source_bits < 32) ? 24 : slot_bits;
    i2s_std_slot_config_t c =
        I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG((i2s_data_bit_width_t)slot_bits, I2S_SLOT_MODE_STEREO);
    /* 槽宽固定，数据宽度也固定为槽宽：DMA 里永远是 32bit 样本 */
    c.slot_bit_width = (i2s_slot_bit_width_t)slot_bits;
    c.ws_width = (uint32_t)slot_bits;   /* WS 高电平占满整槽，保持 50% 占空 */
    switch (fmt) {
    case 1:  /* 左对齐 MSB */
        c.bit_shift = false;
        c.left_align = true;
        break;
    case 2:  /* 右对齐 RJ */
        if (payload_bits < slot_bits) {
            c.bit_shift = false;
            c.left_align = false;
            c.bit_order_lsb = true;
        } else {
            c.bit_shift = false;
            c.left_align = true;      /* 等价于 RJ：数据填满整槽 */
        }
        break;
    default: /* Philips I2S */
        break;
    }
    /* Only slot32/payload24 reaches the four-byte write compensation. */
    s_rj_reverse = (fmt == 2 && payload_bits < slot_bits);
    return c;
}

/* 重新下发槽配置：禁用 → 重配 → 使能。
 * 格式切换与"流逻辑位宽变化"都必须走这里 —— 否则硬件会停留在上一次的线格式上
 * （v23 查出的两个"静默不一致"都源于此：config 说 Philips、线上还是旧格式）。 */
static esp_err_t i2s_apply_slot(void)
{
    if (s_tx == NULL) return ESP_ERR_INVALID_STATE;
    i2s_std_slot_config_t slot = slot_cfg_for(s_port_format, s_bits, s_slot_bits);
    esp_err_t ret = s_running ? i2s_channel_disable(s_tx) : ESP_OK;
    if (ret != ESP_OK) return ret;
    i2s_drop_tx_done_tokens();
    ret = i2s_channel_reconfig_std_slot(s_tx, &slot);
    esp_err_t enabled = s_running ? i2s_channel_enable(s_tx) : ESP_OK;
    if (enabled != ESP_OK) s_running = false;
    return ret != ESP_OK ? ret : enabled;
}
static volatile uint32_t s_sent_events = 0;
static volatile uint32_t s_dma_irq_core = UINT32_MAX;
static volatile uint32_t s_notify_full_events = 0;
static volatile uint32_t s_tail_skip_candidates = 0;
static volatile uint32_t s_last_tail_bytes = 0, s_last_tail_us = 0, s_last_queue_free = 0;
static volatile uint32_t s_queue_overflows = 0;
static volatile uint32_t s_write_calls = 0;
static volatile uint32_t s_write_errors = 0;
static volatile uint32_t s_short_writes = 0;
static volatile uint32_t s_slot_changes = 0;
static volatile uint32_t s_full_blocks = 0, s_backend_partial = 0;
static volatile uint32_t s_stage_accept_frames = 0, s_dma_accept_frames = 0;

static bool i2s_tx_done_cb(i2s_chan_handle_t handle, i2s_event_data_t *event, void *user_ctx)
{
    (void)handle;
    (void)event;
    (void)user_ctx;
    s_sent_events++;
    if (s_dma_irq_core == UINT32_MAX) s_dma_irq_core = xPortGetCoreID();
    BaseType_t woken = pdFALSE;
    if (s_tx_done_sem != NULL) {
        if (xSemaphoreGiveFromISR(s_tx_done_sem, &woken) != pdTRUE) {
            s_notify_full_events++;
        }
    }
    return (woken == pdTRUE);
}

static bool i2s_tx_ovf_cb(i2s_chan_handle_t handle, i2s_event_data_t *event, void *user_ctx)
{
    (void)handle;
    (void)event;
    (void)user_ctx;
    s_queue_overflows++;
    return false;
}

static void i2s_preload_silence(void)
{
    pcm_dma_stage_reset(&s_stage, i2s_out_get_frame_bytes());
    s_stage_pending_bytes = 0;
    i2s_chan_info_t info = {0};
    esp_err_t ret = i2s_channel_get_info(s_tx, &info);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "get info failed (%s), skip silence preload", esp_err_to_name(ret));
        return;
    }

    uint8_t zero[I2S_OUT_DMA_FRAME_BYTES_MAX] = {0};
    size_t remain = info.total_dma_buf_size;
    int guard = 0;

    while (remain > 0 && guard < 32) {
        size_t loaded = 0;
        size_t chunk = (remain > sizeof(zero)) ? sizeof(zero) : remain;
        ret = i2s_channel_preload_data(s_tx, zero, chunk, &loaded);
        if (ret != ESP_OK || loaded == 0) {
            ESP_LOGW(TAG, "preload silence stopped: ret=%s loaded=%u remain=%u",
                     esp_err_to_name(ret), (unsigned)loaded, (unsigned)remain);
            break;
        }
        remain -= loaded;
        guard++;
    }
}

esp_err_t i2s_out_core1_init(void)
{
    ESP_RETURN_ON_FALSE(s_tx == NULL, ESP_ERR_INVALID_STATE, TAG, "already initialized");

    /* I2S1 is created on core0 before this worker (uac_bridge_main.c) so its
     * DMA interrupt is allocated on core0; this worker writes into it. */
    /* 0.84: every completed descriptor must reach the pump. Binary tokens
     * merged completions during BT connection (0.83 notify_full +1, LA
     * discontinuity). Bound the count by the existing six DMA descriptors. */
    s_tx_done_sem = xSemaphoreCreateCounting(I2S_OUT_DMA_DESC_NUM, 0);
    ESP_RETURN_ON_FALSE(s_tx_done_sem != NULL, ESP_ERR_NO_MEM, TAG, "sem create failed");

    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    chan_cfg.dma_desc_num = I2S_OUT_DMA_DESC_NUM;
    chan_cfg.dma_frame_num = I2S_OUT_DMA_FRAME_NUM;
    chan_cfg.auto_clear_after_cb = true;

    esp_err_t ret = i2s_new_channel(&chan_cfg, &s_tx, NULL);
    ESP_GOTO_ON_ERROR(ret, err, TAG, "new channel failed");

    i2s_std_config_t std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(s_rate),
        .slot_cfg = slot_cfg_for(s_port_format, s_bits, s_slot_bits),
        .gpio_cfg = {
            .mclk = DUAL_I2S_P0_MCLK,
            .bclk = DUAL_I2S_P0_BCLK,
            .ws = DUAL_I2S_P0_WS,
            .dout = DUAL_I2S_P0_DOUT,
            .din = I2S_GPIO_UNUSED,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv = false,
            },
        },
    };
    std_cfg.clk_cfg.clk_src = I2S_CLK_SRC_APLL; /* XTAL (40 MHz) is too low for 88.2/96k MCLK */
    std_cfg.clk_cfg.mclk_multiple = (s_slot_bits == 24) ? I2S_MCLK_MULTIPLE_384 : I2S_MCLK_MULTIPLE_256;

    ret = i2s_channel_init_std_mode(s_tx, &std_cfg);
    ESP_GOTO_ON_ERROR(ret, err, TAG, "init std mode failed");

    i2s_event_callbacks_t cbs = {
        .on_sent = i2s_tx_done_cb,
        .on_send_q_ovf = i2s_tx_ovf_cb,
    };
    ret = i2s_channel_register_event_callback(s_tx, &cbs, NULL);
    ESP_GOTO_ON_ERROR(ret, err, TAG, "register callback failed");

    i2s_preload_silence();

    s_bits = 32;
    ESP_LOGI(TAG, "I2S0 initialized: %lu Hz / %d-bit logical (%d-bit slot), MCLK=%d BCLK=%d WS=%d DOUT=%d",
             (unsigned long)s_rate, s_bits, s_slot_bits,
             DUAL_I2S_P0_MCLK, DUAL_I2S_P0_BCLK, DUAL_I2S_P0_WS, DUAL_I2S_P0_DOUT);
    return ESP_OK;

err:
    if (s_tx != NULL) {
        i2s_channel_disable(s_tx);
        i2s_del_channel(s_tx);
        s_tx = NULL;
    }
    if (s_tx_done_sem != NULL) {
        vSemaphoreDelete(s_tx_done_sem);
        s_tx_done_sem = NULL;
    }
    return ret;
}

esp_err_t i2s_out_core1_start(void)
{
    ESP_RETURN_ON_FALSE(s_tx != NULL, ESP_ERR_INVALID_STATE, TAG, "not initialized");
    if (s_running) return ESP_OK;
    esp_err_t ret = i2s_channel_enable(s_tx);
    if (ret == ESP_OK) {
        s_running = true;
        ESP_LOGI(TAG, "I2S0 started (core1 owns the channel and its DMA interrupt)");
    }
    return ret;
}

/* Release BOTH channel references before selecting a new shared APLL
 * frequency. channel_disable alone leaves its clock-source reference alive.
 * A temporary 48k/384 XTAL clock is valid with any supported slot width;
 * no samples/clocks are transmitted while either channel is parked. */
static esp_err_t i2s_park_primary_clock(void)
{
    i2s_std_clk_config_t clock = I2S_STD_CLK_DEFAULT_CONFIG(48000);
    clock.clk_src = I2S_CLK_SRC_XTAL;
    clock.mclk_multiple = I2S_MCLK_MULTIPLE_384;
    return i2s_channel_reconfig_std_clock(s_tx, &clock);
}

/* One block-boundary transaction: stop/park both, configure both, then resume
 * only the ports which were RUNNING. DSP/PCM descriptors/ABI are unchanged. */
static esp_err_t i2s_set_geometry(uint32_t rate, int bits, int slot_bits)
{
    ESP_RETURN_ON_FALSE(s_tx != NULL, ESP_ERR_INVALID_STATE, TAG, "not initialized");
    ESP_RETURN_ON_FALSE(rate >= 8000 && rate <= 96000 && (bits == 24 || bits == 32),
                        ESP_ERR_INVALID_ARG, TAG, "unsupported source geometry");
    ESP_RETURN_ON_FALSE(slot_bits == 16 || slot_bits == 24 || slot_bits == 32,
                        ESP_ERR_INVALID_ARG, TAG, "unsupported slot width");
    if (rate == s_rate && bits == s_bits && slot_bits == s_slot_bits) return ESP_OK;
    const uint32_t old_rate = s_rate;
    const int old_bits = s_bits, old_slot = s_slot_bits;
    const bool running = s_running;
    i2s1_out_stats_t p1_old;
    i2s1_out_get_stats(&p1_old);
    esp_err_t ret = running ? i2s_channel_disable(s_tx) : ESP_OK;
    if (ret != ESP_OK) return ret;
    s_running = false;
    i2s_drop_tx_done_tokens();

    ret = i2s1_out_core1_park_clock();
    if (ret == ESP_OK) ret = i2s_park_primary_clock();
    i2s_std_slot_config_t slot = slot_cfg_for(s_port_format, bits, slot_bits);
    i2s_std_clk_config_t clock = I2S_STD_CLK_DEFAULT_CONFIG(rate);
    clock.clk_src = I2S_CLK_SRC_APLL;
    clock.mclk_multiple = slot_bits == 24 ? I2S_MCLK_MULTIPLE_384 : I2S_MCLK_MULTIPLE_256;
    if (ret == ESP_OK) ret = i2s_channel_reconfig_std_slot(s_tx, &slot);
    if (ret == ESP_OK) ret = i2s_channel_reconfig_std_clock(s_tx, &clock);
    if (ret == ESP_OK) ret = i2s1_out_core1_configure(rate, slot_bits);
    if (ret == ESP_OK) {
        s_rate = rate; s_bits = bits; s_slot_bits = slot_bits;
        i2s_preload_silence();
        if (p1_old.running) ret = i2s1_out_core1_start();
        if (ret == ESP_OK && running) ret = i2s_channel_enable(s_tx);
    }
    if (ret != ESP_OK) {
        /* A partial new-rate attempt also owns APLL references. Park both
         * again before rollback; never restore one clock against the other's
         * new reference. Failed rollback leaves BOTH ports stopped. */
        const esp_err_t park1 = i2s1_out_core1_park_clock();
        const esp_err_t park0 = i2s_park_primary_clock();
        esp_err_t restore = park1 != ESP_OK ? park1 : park0;
        if (restore == ESP_OK) {
            slot = slot_cfg_for(s_port_format, old_bits, old_slot);
            clock.sample_rate_hz = old_rate;
            clock.mclk_multiple = old_slot == 24 ? I2S_MCLK_MULTIPLE_384 : I2S_MCLK_MULTIPLE_256;
            restore = i2s_channel_reconfig_std_slot(s_tx, &slot);
            if (restore == ESP_OK) restore = i2s_channel_reconfig_std_clock(s_tx, &clock);
            if (restore == ESP_OK)
                restore = i2s1_out_core1_configure(p1_old.rate, p1_old.slot_bits);
        }
        s_rate = old_rate; s_bits = old_bits; s_slot_bits = old_slot;
        if (restore == ESP_OK) {
            i2s_preload_silence();
            if (p1_old.running) restore = i2s1_out_core1_start();
            if (restore == ESP_OK && running) restore = i2s_channel_enable(s_tx);
        }
        s_running = running && restore == ESP_OK;
        if (restore != ESP_OK) i2s1_out_core1_stop();
        ESP_LOGE(TAG, "dual geometry rejected: %s; pair_restore=%s held=%u",
                 esp_err_to_name(ret), esp_err_to_name(restore), restore != ESP_OK);
        return ret;
    }
    s_running = running;
    if (slot_bits != old_slot) s_slot_changes++;
    i2s_epoch_bump();
    ESP_LOGI(TAG, "dual master -> %lu Hz, logical=%d slot=%d parked_pair=1", (unsigned long)rate, bits, slot_bits);
    return ESP_OK;
}
static esp_err_t i2s_out_core1_reconfig(uint32_t rate_hz, int bit_depth)
{
    return i2s_set_geometry(rate_hz, bit_depth, s_slot_bits);
}
static esp_err_t i2s_out_core1_set_slot_width(int slot_bits)
{
    return i2s_set_geometry(s_rate, s_bits, slot_bits);
}

int i2s_out_get_slot_width(void)
{
    return s_slot_bits;
}

size_t i2s_out_get_frame_bytes(void)
{
    return (size_t)I2S_OUT_DMA_FRAME_NUM * 2 * (s_slot_bits / 8);
}

/* One stereo sample frame. Byte counters must be divided by THIS, not by the
 * whole descriptor: frame_bytes is 240 frames worth of bytes, so the old
 * "n / frame_bytes" produced 0/1 "frames" and was never a frame count. */
size_t i2s_out_get_sample_stride_bytes(void)
{
    return (size_t)2 * (size_t)(s_slot_bits / 8);
}

uint32_t i2s_out_get_epoch(void)
{
    return s_epoch;
}


typedef struct { esp_err_t ret; } i2s_stage_sink_ctx_t;

static size_t i2s_stage_sink(const void *buf, size_t size, void *ctx)
{
    i2s_stage_sink_ctx_t *result = ctx;
    const uint32_t pos = s_tx->dma.rw_pos;
    const uint32_t capacity = s_tx->dma.buf_size;
    const UBaseType_t free_slots = uxQueueSpacesAvailable(s_tx->msg_queue);
    if (pos > 0 && pos < capacity && free_slots <= 1) {
        s_last_tail_bytes = capacity - pos;
        s_last_tail_us = (uint32_t)esp_timer_get_time();
        s_last_queue_free = free_slots;
        s_tail_skip_candidates++;
    }
    size_t written = 0;
    result->ret = i2s_channel_write(s_tx, buf, size, &written, 0);
    s_dma_accept_frames += (uint32_t)(written / i2s_out_get_sample_stride_bytes());
    if (written == size) s_full_blocks++;
    else if (written != 0) s_backend_partial++;
    return written;
}

esp_err_t i2s_out_write(const void *buf, size_t size, size_t *written)
{
    if (s_tx == NULL || buf == NULL || written == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    /* ---- 右对齐（RJ）的软件补偿（v23 查清）----
     * 本机 I2S 在 bit_order_lsb=1 时**确实**把数据右对齐到槽尾（前 8 位补零，实测），
     * 但按 **LSB 先发**；而 tx_left_align 字段实测无效（0/1 输出相同）。
     * 所以把 24 位样本的位序反过来：硬件按 LSB 先发 → 线上即 **MSB 先发 + 右对齐** = 真正的 RJ。
     * 只对「位宽 < 槽宽」生效：此时 s_rj_reverse 才为真（见 slot_cfg_for）。
     * 32bit 数据占满 32bit 槽时不开 bit_order_lsb，也就没有反转的必要。 */
    if (s_rj_reverse && s_bits < s_slot_bits) {
        const size_t words = size / 4;
        if (words > 0 && words * 4 <= sizeof(s_rj_scratch)) {
            for (size_t i = 0; i < words; i++) {
                uint32_t w;
                memcpy(&w, (const uint8_t *)buf + i * 4, sizeof(w));
                const uint32_t rev = bitrev24(w >> 8) << 8;   /* 数据在字的高 24 位 */
                memcpy(s_rj_scratch + i * 4, &rev, sizeof(rev));
            }
            buf = s_rj_scratch;
            size = words * 4;
        }
    }

    s_write_calls++;
    i2s_stage_sink_ctx_t sink = { .ret = ESP_OK };
    *written = pcm_dma_stage_push(&s_stage, buf, size, i2s_stage_sink, &sink);
    s_stage_accept_frames += (uint32_t)(*written / i2s_out_get_sample_stride_bytes());
    s_stage_pending_bytes = (uint32_t)(s_stage.filled - s_stage.sent);
    esp_err_t ret = (*written == size) ? ESP_OK : ESP_ERR_TIMEOUT;
    if (sink.ret != ESP_OK && sink.ret != ESP_ERR_TIMEOUT) ret = sink.ret;
    if (ret != ESP_OK && ret != ESP_ERR_TIMEOUT) {
        s_write_errors++;
    } else if (ret == ESP_OK && *written != size) {
        s_short_writes++;
    }
    return ret;
}

bool i2s_out_wait_tx_done(TickType_t ticks_to_wait)
{
    if (s_tx_done_sem == NULL) {
        return false;
    }
    return xSemaphoreTake(s_tx_done_sem, ticks_to_wait) == pdTRUE;
}

void i2s_out_get_stats(i2s_out_stats_t *stats)
{
    if (stats == NULL) {
        return;
    }
    stats->sent_events = s_sent_events;
    stats->irq_core = s_dma_irq_core;
    stats->notify_full_events = s_notify_full_events;
    stats->tail_skip_candidates = s_tail_skip_candidates;
    stats->last_tail_bytes = s_last_tail_bytes;
    stats->last_tail_us = s_last_tail_us;
    stats->last_queue_free = s_last_queue_free;
    stats->queue_overflows = s_queue_overflows;
    stats->write_calls = s_write_calls;
    stats->write_errors = s_write_errors;
    stats->short_writes = s_short_writes;
    stats->slot_changes = s_slot_changes;
    stats->full_blocks = s_full_blocks;
    stats->backend_partial = s_backend_partial;
    stats->stage_pending_bytes = s_stage_pending_bytes;
    const uint32_t stride = (uint32_t)i2s_out_get_sample_stride_bytes();
    stats->sample_stride_bytes = stride;
    stats->stage_accept_frames = s_stage_accept_frames;
    stats->dma_accept_frames = s_dma_accept_frames;
}

uint32_t i2s_out_get_rate(void)
{
    return s_rate;
}

int i2s_out_get_bits(void)
{
    return s_bits;
}

static esp_err_t i2s_out_core1_set_format(int fmt)
{
    ESP_RETURN_ON_FALSE(fmt >= 0 && fmt <= 2, ESP_ERR_INVALID_ARG, TAG, "bad format %d", fmt);
    if (fmt == s_port_format) {
        return ESP_OK;
    }
    const int old_format = s_port_format;
    s_port_format = fmt;
    if (s_tx == NULL) {
        return ESP_OK;   /* 初始化前只记状态，init 时按它建槽 */
    }
    esp_err_t ret = i2s_apply_slot();
    if (ret != ESP_OK) {
        s_port_format = old_format;
        (void)i2s_apply_slot();
    }
    ESP_LOGI(TAG, "port I2S format -> %s (%s)", fmt == 0 ? "I2S" : (fmt == 1 ? "left-justified" : "right-justified"),
             esp_err_to_name(ret));
    if (ret == ESP_OK) {
        i2s_epoch_bump();
    }
    return ret;
}

/* ---------------- core0 -> core1 marshalling ----------------
 *
 * Every mutating I2S operation is executed by the core1 worker, in FIFO order
 * behind the PCM descriptors that were already queued. A caller on core0 posts
 * the command and blocks on the worker's answer; there is no shared lock that
 * could deadlock against core1, and no sleep-based sequencing.
 *
 * Before the worker exists (early dsp_config_init), the state-only calls just
 * record the value: the channel is created from it by i2s_out_core1_init().
 */

static bool i2s_port_format_valid(int fmt)
{
    return fmt >= 0 && fmt <= 2;
}

esp_err_t i2s_out_core1_exec(uint32_t cmd, uint32_t a0, uint32_t a1)
{
    switch (cmd) {
    case I2S_CTL_INIT:      return i2s_out_core1_init();
    case I2S_CTL_START:     return i2s_out_core1_start();
    case I2S_CTL_RECONFIG:  return i2s_out_core1_reconfig(a0, (int)a1);
    case I2S_CTL_SLOT_WIDTH: return i2s_out_core1_set_slot_width((int)a0);
    case I2S_CTL_FORMAT:    return i2s_out_core1_set_format((int)a0);
    case I2S_CTL_SLOT_RAW:  return i2s_out_core1_set_slot_raw((uint8_t)a0);
    case I2S_CTL_P1_FORMAT: return i2s1_out_core1_set_format((int)a0);
    default:                return ESP_ERR_INVALID_ARG;
    }
}

esp_err_t i2s_out_start(void)
{
    return dsp_core1_ctl(I2S_CTL_START, 0, 0);
}

esp_err_t i2s_out_reconfig(uint32_t rate_hz, int bit_depth)
{
    if (rate_hz == s_rate && bit_depth == s_bits) {
        return ESP_OK;   /* no real change: no epoch bump, nothing to post */
    }
    return dsp_core1_ctl(I2S_CTL_RECONFIG, rate_hz, (uint32_t)bit_depth);
}

esp_err_t i2s_out_set_slot_width(int slot_bits)
{
    if (slot_bits == s_slot_bits) {
        return ESP_OK;
    }
    return dsp_core1_ctl(I2S_CTL_SLOT_WIDTH, (uint32_t)slot_bits, 0);
}

esp_err_t i2s_out_set_format(int fmt)
{
    if (!i2s_port_format_valid(fmt)) {
        return ESP_ERR_INVALID_ARG;
    }
    if (fmt == s_port_format) {
        return ESP_OK;
    }
    if (!dsp_core1_running()) {
        s_port_format = fmt;   /* recorded; the core1 init builds the slot from it */
        return ESP_OK;
    }
    return dsp_core1_ctl(I2S_CTL_FORMAT, (uint32_t)fmt, 0);
}

esp_err_t i2s_out_set_slot_raw(uint8_t flags)
{
    return dsp_core1_ctl(I2S_CTL_SLOT_RAW, flags, 0);
}

esp_err_t i2s_out_set_secondary_format(int fmt)
{
    if (fmt < 0 || fmt > 1) return ESP_ERR_NOT_SUPPORTED;
    return dsp_core1_ctl(I2S_CTL_P1_FORMAT, (uint32_t)fmt, 0);
}
bool i2s_out_is_running(void) { return s_running; }
