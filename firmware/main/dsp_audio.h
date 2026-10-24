/*
 * core1 audio worker - ESP32-S31 audio bridge.
 *
 * 0.113: the core1 task is the single owner of the DSP chain AND of the I2S0
 * output port. core0 only produces PCM into a free slot and submits a
 * descriptor; it never waits for the DSP and never touches processed PCM.
 *
 *   core0 (USB pump)                     core1 (this worker)
 *   ----------------                     ------------------
 *   acquire free slot (2 slots)
 *   read USB FIFO into the slot
 *   submit descriptor  --------------->  dequeue (FIFO order)
 *                                          epoch check
 *                                          dsp_engine_process()
 *                                          in-place slot-width conversion
 *                                          i2s_out_write() -> stage -> DMA
 *   release handled by the worker <----  slot back to FREE
 *
 * The same FIFO also carries I2S control commands (dsp_core1_ctl), so a
 * reconfiguration is ordered strictly behind every PCM descriptor submitted
 * before it - no sleep-based guessing anywhere in the switch path.
 */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"

/* `dsp_source_t` lives in dsp_engine.h (structural decoupling, v33); this file
 * depends on it, never the other way round. */
#include "dsp_engine.h"
/* Slot geometry follows the I2S output descriptor (dma_frame_num). */
#include "i2s_out_v2.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ---------------- asynchronous PCM handoff (0.113) ---------------- */

/* One slot carries at most one I2S output descriptor worth of stereo frames.
 * The producer fills interleaved int32 stereo PCM; the core1 worker converts it
 * to the configured slot width IN PLACE (the packed result is never larger than
 * the int32 input, so no separate conversion buffer and no zero buffer exist). */
#define DSP_PCM_SLOT_FRAMES    I2S_OUT_DMA_FRAME_NUM
#define DSP_PCM_SLOT_SAMPLES   (DSP_PCM_SLOT_FRAMES * 2u)
#define DSP_PCM_SLOT_BYTES     (DSP_PCM_SLOT_SAMPLES * sizeof(int32_t))
#define DSP_PCM_SLOT_COUNT     2u

/* Descriptor flags. */
#define DSP_PCM_F_HAS_DATA       0x01u  /* slot holds real PCM from the source   */
#define DSP_PCM_F_SILENCE        0x02u  /* core1 fills/freezes the slot as zeros */
#define DSP_PCM_F_ENGINE_BYPASS  0x04u  /* skip dsp_engine_process (0xB5) only   */

/* Producer side (core0 only).
 * acquire() returns a slot index or -1 when no slot is free (real
 * backpressure: the caller must NOT overwrite a slot and must NOT read source
 * data it cannot hand over). The returned buffer holds DSP_PCM_SLOT_SAMPLES
 * int32 samples. */
/* Nonblocking producer gate: begin/end enclose source read and submission.
 * A format control waits for this short core0 operation before it is queued. */
bool     dsp_pcm_begin(void);
void     dsp_pcm_end(void);
int      dsp_pcm_acquire(void);
int32_t *dsp_pcm_slot(int slot);
/* samples = valid int32 samples in the slot (2..DSP_PCM_SLOT_SAMPLES, even).
 * With DSP_PCM_F_SILENCE the slot content is ignored and samples should be
 * DSP_PCM_SLOT_SAMPLES (a full output descriptor of digital silence). */
esp_err_t dsp_pcm_submit(int slot, size_t samples, uint32_t rate, dsp_source_t source,
                         uint32_t flags, uint32_t epoch);
/* Give a slot back without submitting (producer error path). */
void     dsp_pcm_release(int slot);

/* Blocking control call executed by the core1 worker between PCM blocks; the
 * command namespace belongs to i2s_out_v2.c (I2S_OUT_CTL_*). Returns the
 * executor's esp_err_t, ESP_ERR_TIMEOUT when posting was unsuccessful, or
 * ESP_ERR_INVALID_STATE when the worker has not been started. */
#define DSP_CTL_ENGINE_SYNC 0x100u
esp_err_t dsp_core1_ctl(uint32_t cmd, uint32_t a0, uint32_t a1);

/* ---------------- lifecycle ---------------- */

/* Create the queue and the core1 worker (priority: highest, pinned core1).
 * The worker creates the I2S channel itself so the DMA interrupt belongs to
 * core1; core0 must wait for dsp_audio_wait_ready() before starting any task
 * that can submit PCM or issue control calls. */
esp_err_t dsp_audio_init(void);
esp_err_t dsp_audio_wait_ready(uint32_t timeout_ms);
bool      dsp_core1_running(void);

bool     dsp_audio_is_enabled(void);
uint32_t dsp_audio_get_stack_hwm(void);

/* Reset the min/max/avg block-time window (used by the telemetry report). */
void dsp_audio_reset_block_stats(void);

/* 取"上次调用以来"的左右峰值（绝对 int32 值）并清零 —— 电平表用（主机可 5~20 Hz 轮询）。
 * 这里是 core1 输出侧（DSP 之后）的峰值。 */
void dsp_audio_take_peaks(uint32_t *peak_l, uint32_t *peak_r);

/* Same, for the diagnostics line: output-side peak since the previous call. */
void dsp_audio_take_output_peaks(uint32_t *peak_l, uint32_t *peak_r);

/* ---------------- statistics ---------------- */

/* Window fields (min/max/avg/last) describe the per-block DSP net time and are
 * cleared by dsp_audio_reset_block_stats(). Cumulative counters and maxima
 * never clear; win_* and *_sum_us form the separately reset timing window. */
typedef struct {
    uint32_t blocks;
    uint32_t uac_blocks;
    uint32_t bt_blocks;
    uint32_t test_blocks;
    uint32_t samples;
    uint32_t clipped;
    uint32_t max_peak_l;
    uint32_t max_peak_r;
    uint32_t last_us;
    uint32_t min_us;
    uint32_t max_us;
    uint32_t avg_us;

    /* cumulative block counters */
    uint32_t silence_blocks;      /* digital-silence descriptors output         */
    uint32_t bypass_blocks;       /* blocks that skipped dsp_engine_process     */
    uint32_t stale_epoch;         /* descriptors dropped after an epoch change  */
    uint32_t backpressure;        /* acquire() found no free slot               */
    uint32_t out_zero_blocks;     /* post-DSP blocks that were entirely zero    */
    uint32_t frames_in;           /* int32 stereo frames accepted from core0    */
    uint32_t frames_stage;        /* frames durably copied into the I2S stage   */
    uint32_t frames_undelivered;  /* frames the output stage refused (real loss)*/
    uint32_t write_retry;         /* i2s_out_write calls that needed a retry    */
    uint32_t write_partial;       /* i2s_out_write calls that stayed partial    */
    uint32_t write_errors;        /* hard i2s_out_write errors                  */
    uint32_t stage_pending_bytes; /* bytes still held by the output stage       */

    /* deadlines: 240 frames at the block rate; 90% = the production target */
    uint32_t deadline_us;
    uint32_t deadline_miss;       /* total wall > deadline (cumulative)         */
    uint32_t budget90_over;       /* total wall > 90% deadline (cumulative)     */
    uint32_t control_max_us;      /* rebuild/reconfiguration execution wall    */
    uint32_t control_skips;       /* producer ticks paused for output control  */

    /* per-block stage times: cumulative maxima + window sums/count */
    uint32_t prep_max_us;         /* producer fill                         */
    uint32_t queue_max_us;        /* wait in the descriptor queue          */
    uint32_t dsp_max_us;          /* dsp_engine_process() net              */
    uint32_t port_max_us;         /* conversion + output-stage accept      */
    uint32_t total_max_us;        /* fill start -> output accepted (wall)  */
    uint32_t total_max_uac_us;
    uint32_t total_max_bt_us;
    uint32_t total_max_bypass_us;
    uint32_t win_blocks;
    uint32_t win_prep_max_us, win_queue_max_us, win_dsp_max_us, win_port_max_us, win_total_max_us;
    uint64_t prep_sum_us, queue_sum_us, dsp_sum_us, port_sum_us, total_sum_us;
} dsp_audio_stats_t;

void dsp_audio_get_stats(dsp_audio_stats_t *stats);
/* Snapshot and clear only the timing window in one short critical section. */
void dsp_audio_take_stats(dsp_audio_stats_t *stats);

#ifdef __cplusplus
}
#endif
