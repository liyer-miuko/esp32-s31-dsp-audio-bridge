/*
 * I2S0 output driver, v2 - ESP32-S31 UAC bridge.
 *
 * Natural pin map; i2s1_out.h selects the complete paired test/product layout.
 * GPIO6 is damaged and intentionally avoided:
 *   MCLK = GPIO2
 *   BCLK = GPIO3
 *   WS   = GPIO4
 *   DOUT = GPIO7
 *
 * The I2S slot width is configurable to 16/24/32 bits. All source PCM is
 * normalized to 32-bit internally, then converted to the selected slot width
 * by the core1 worker. Slot-width changes are performed only through this driver
 * while the caller has muted output, so the I2S bus is never partially
 * reconfigured while a source is playing.
 */
#pragma once

#include <stdint.h>
#include <stddef.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define I2S_OUT_GPIO_MCLK   2
#define I2S_OUT_GPIO_BCLK   3
#define I2S_OUT_GPIO_WS     4
#define I2S_OUT_GPIO_DOUT   7   /* original GPIO6 is damaged */

#define I2S_OUT_DMA_DESC_NUM   6   /* v9: 6 x 240 frames = 15 ms (v4-proven) */
#define I2S_OUT_DMA_FRAME_NUM  240 /* divisible by 3, valid for 16/24/32-bit modes */
/* Maximum one-descriptor size: 32-bit stereo. Actual size depends on slot width. */
#define I2S_OUT_DMA_FRAME_BYTES_MAX (I2S_OUT_DMA_FRAME_NUM * 2 * 4)
#define I2S_OUT_DMA_FRAME_BYTES (I2S_OUT_DMA_FRAME_BYTES_MAX) /* deprecated alias */

typedef struct {
    uint32_t sent_events;
    uint32_t irq_core;                /* actual DMA callback CPU */
    uint32_t notify_full_events; /* on_sent notification could not be queued */
    uint32_t tail_skip_candidates;
    uint32_t last_tail_bytes, last_tail_us, last_queue_free;
    uint32_t queue_overflows;
    uint32_t write_calls;
    uint32_t write_errors;
    uint32_t short_writes;
    uint32_t slot_changes;
    uint32_t full_blocks, backend_partial, stage_pending_bytes;
    /* 0.113 frame accounting, in stereo sample frames (bytes / stride). Never
     * confuse these with the driver queue: stage_accept is a copy into the
     * descriptor stage, dma_accept is the IDF driver queue, and neither of
     * them is proof of what reached the wire. */
    uint32_t sample_stride_bytes;
    uint32_t stage_accept_frames;
    uint32_t dma_accept_frames;
} i2s_out_stats_t;

/* ★ 0.113: the I2S peripheral belongs to core1. Every mutating call below is a
 * core0-side wrapper: it posts a control item into the core1 worker queue and
 * blocks until the worker answers, so the change is ordered strictly behind
 * every PCM descriptor submitted before it (no sleep-based sequencing). The
 * channel itself - and with it the DMA interrupt - is created by the core1
 * worker (dsp_audio_init), which is why none of these may be called before
 * dsp_audio_init()/dsp_audio_wait_ready(). There is no core0-side init call. */
esp_err_t i2s_out_start(void);
esp_err_t i2s_out_reconfig(uint32_t rate_hz, int bit_depth);
esp_err_t i2s_out_set_slot_width(int slot_bits);

/* 端口 I2S 线格式：0=Philips I2S，1=左对齐(MSB)，2=右对齐 */
esp_err_t i2s_out_set_format(int fmt);
/* Independent I2S1 format: Philips=0 or MSB=1; RJ is explicitly unsupported. */
esp_err_t i2s_out_set_secondary_format(int fmt);
bool i2s_out_is_running(void);

/* 【诊断】按原始槽标志位配置，用于在真机上扫出"右对齐"的正确组合。
 * bit0=bit_shift bit1=left_align bit2=bit_order_lsb bit3=big_endian
 * bit4=ws_width 用 data_bits 而不是 slot。 */
esp_err_t i2s_out_set_slot_raw(uint8_t flags);
int i2s_out_get_slot_width(void);
/* Bytes of one complete output descriptor (240 frames at the slot width). */
size_t i2s_out_get_frame_bytes(void);
/* Bytes of one stereo sample frame (2 x slot bytes) - use THIS to turn byte
 * counters into frames; i2s_out_get_frame_bytes() is a whole descriptor. */
size_t i2s_out_get_sample_stride_bytes(void);
/* Output-configuration generation. It changes only when a mutator above really
 * changed something, and a PCM descriptor tagged with an older generation is
 * dropped by the core1 worker instead of being output as if it were current. */
uint32_t i2s_out_get_epoch(void);
/* written is the durable accepted prefix in the output stage, not necessarily
 * already in DMA. Subsequent calls flush it in order; reconfig drops old PCM.
 * ⚠ CORE1 WORKER ONLY - it does not marshal and would self-deadlock. */
esp_err_t i2s_out_write(const void *buf, size_t size, size_t *written);
bool i2s_out_wait_tx_done(TickType_t ticks_to_wait);
void i2s_out_get_stats(i2s_out_stats_t *stats);
uint32_t i2s_out_get_rate(void);
int i2s_out_get_bits(void);

/* ---- core1 side (called by the core1 worker only) ---- */
esp_err_t i2s_out_core1_init(void);
/* Run one marshalled control command; returns the esp_err_t handed back to the
 * core0 caller. Command ids are private to i2s_out_v2.c. */
esp_err_t i2s_out_core1_exec(uint32_t cmd, uint32_t a0, uint32_t a1);

#ifdef __cplusplus
}
#endif
