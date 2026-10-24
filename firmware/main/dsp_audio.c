/*
 * core1 audio worker - ESP32-S31 audio bridge.
 *
 * Single core1 owner of: PCM descriptor queue -> dsp_engine_process() ->
 * slot-width conversion (in place) -> i2s_out_write() (descriptor stage) ->
 * I2S0 DMA, plus every I2S control command marshalled from core0. All signal
 * processing itself lives in dsp_engine.c; this file owns the handoff, the
 * output stage, the per-block telemetry and the peak/clip diagnostics.
 *
 * The engine is a bit-perfect passthrough when nothing is configured, so a
 * default UAC stream is not touched at all.
 */
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "sdkconfig.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "dsp_audio.h"
#include "dsp_engine.h"
#include "i2s_out_v2.h"
#include "i2s1_out.h"

#define DSP_TASK_PRIO      (configMAX_PRIORITIES - 1)
/* Bytes in this IDF fork (freertos/task.h: uxTaskGetStackHighWaterMark returns
 * bytes, not words) - do not multiply by 4. */
#define DSP_TASK_STACK     4096

/* Descriptor queue: at most DSP_PCM_SLOT_COUNT PCM items can be in flight (one
 * per slot), the rest of the depth absorbs control items. */
#define DSP_QUEUE_LEN      4
/* Posting is bounded; a successfully posted control retains its ACK until
 * completion. The worker must have reported ready before any control call. */
#define DSP_CTL_POST_TIMEOUT_MS   200
#define DSP_CTL_ACK_TIMEOUT_MS    2000
/* Output-stage retry: only a saturated driver queue can refuse bytes. One
 * retry per millisecond, bounded, and every retry is counted. */
#define DSP_OUT_RETRY_MAX         8

static const char *TAG = "dsp";

typedef enum {
    CORE1_ITEM_PCM = 0,
    CORE1_ITEM_CTL = 1,
} core1_item_type_t;

typedef struct {
    uint8_t  type;
    uint8_t  slot;
    uint8_t  source;
    uint8_t  flags;
    uint16_t samples;      /* valid int32 samples in the slot */
    uint16_t reserved;
    uint32_t rate;
    uint32_t epoch;
    uint32_t ctl_cmd, ctl_a0, ctl_a1;
    int64_t  fill_start_us; /* producer began filling the slot  */
    int64_t  submit_us;     /* producer handed the descriptor over */
} core1_item_t;

typedef enum {
    SLOT_FREE = 0,   /* nobody owns it                  */
    SLOT_CORE0,      /* producer filling / queued       */
    SLOT_CORE1,      /* worker processing / outputting  */
} slot_state_t;

static int32_t s_slot_pcm[DSP_PCM_SLOT_COUNT][DSP_PCM_SLOT_SAMPLES];
/* Secondary output: port 1 (CH3/CH4) before slot conversion. One block only;
 * the core1 worker converts/writes it immediately after port 0. */
static int32_t s_port1_pcm[DSP_PCM_SLOT_SAMPLES];

/* Bring-up counters for the secondary output; read over JTAG. Not on the hot
 * path beyond a few increments per block. */
volatile uint32_t g_dbg_i2s1_calls;
volatile uint32_t g_dbg_i2s1_bytes;
volatile uint32_t g_dbg_i2s1_err;
volatile uint32_t g_dbg_p1_nonzero;
volatile uint32_t g_dbg_p0_nonzero;
static int64_t s_slot_fill_us[DSP_PCM_SLOT_COUNT];
static volatile uint8_t s_slot_state[DSP_PCM_SLOT_COUNT];
static portMUX_TYPE s_slot_mux = portMUX_INITIALIZER_UNLOCKED;
static portMUX_TYPE s_stats_mux = portMUX_INITIALIZER_UNLOCKED;

static QueueHandle_t s_queue;
static SemaphoreHandle_t s_ctl_ack;
static SemaphoreHandle_t s_ctl_mutex;
static SemaphoreHandle_t s_supply_mutex;
static volatile bool s_ready;
static SemaphoreHandle_t s_ready_sem;
static volatile esp_err_t s_ctl_result;
static volatile esp_err_t s_ready_err;
static TaskHandle_t s_task;

static bool s_enabled = true;

/* Cumulative counters (never cleared by the telemetry window). */
static volatile uint32_t s_blocks;
static volatile uint32_t s_uac_blocks;
static volatile uint32_t s_bt_blocks;
static volatile uint32_t s_test_blocks;
static volatile uint32_t s_samples_total;
static volatile uint32_t s_max_peak_l;
static volatile uint32_t s_max_peak_r;
static volatile uint32_t s_win_peak_l;      /* since the last take_peaks() */
static volatile uint32_t s_win_peak_r;
static volatile uint32_t s_out_peak_l;      /* since the last take_output_peaks() */
static volatile uint32_t s_out_peak_r;
static volatile uint32_t s_last_us;

static volatile uint32_t s_silence_blocks;
static volatile uint32_t s_bypass_blocks;
static volatile uint32_t s_stale_epoch;
static volatile uint32_t s_backpressure;
static volatile uint32_t s_out_zero_blocks;
static volatile uint32_t s_frames_in;
static volatile uint32_t s_frames_stage;
static volatile uint32_t s_frames_undelivered;
static volatile uint32_t s_write_retry;
static volatile uint32_t s_write_partial;
static volatile uint32_t s_write_errors;
static volatile uint32_t s_deadline_miss;
static volatile uint32_t s_budget90_over;
static volatile uint32_t s_control_max_us, s_control_skips;

static volatile uint32_t s_prep_max_us, s_queue_max_us, s_dsp_max_us;
static volatile uint32_t s_port_max_us, s_total_max_us;
static volatile uint32_t s_total_max_uac_us, s_total_max_bt_us, s_total_max_bypass_us;
static volatile uint64_t s_prep_sum_us, s_queue_sum_us, s_dsp_sum_us;
static volatile uint64_t s_port_sum_us, s_total_sum_us;
static volatile uint32_t s_sum_blocks;
static volatile uint32_t s_w_prep_max_us, s_w_queue_max_us, s_w_dsp_max_us;
static volatile uint32_t s_w_port_max_us, s_w_total_max_us, s_w_blocks;

/* Block-time window of the DSP net pass (reset by the telemetry report). */
static volatile uint32_t s_win_min_us = UINT32_MAX;
static volatile uint32_t s_win_max_us;
static volatile uint64_t s_win_sum_us;
static volatile uint32_t s_win_blocks;

static inline uint32_t dsp_abs_sample(int32_t v)
{
    return (v < 0) ? (uint32_t)(-(int64_t)v) : (uint32_t)v;
}

/* One 240-frame descriptor at the block rate: 5442/5000/2721/2500 us for
 * 44.1/48/88.2/96 kHz. This is the production deadline - the six DMA buffers
 * (15/30 ms) are latency, not a production budget. */
static uint32_t dsp_deadline_us(uint32_t rate)
{
    if (rate < 8000u) {
        rate = 48000u;
    }
    return (uint32_t)(((uint64_t)DSP_PCM_SLOT_FRAMES * 1000000u) / rate);
}

/* ---- slot ownership (producer on core0, release on core1) ---- */

int dsp_pcm_acquire(void)
{
    int got = -1;
    portENTER_CRITICAL(&s_slot_mux);
    for (int i = 0; i < (int)DSP_PCM_SLOT_COUNT; i++) {
        if (s_slot_state[i] == SLOT_FREE) {
            s_slot_state[i] = SLOT_CORE0;
            got = i;
            break;
        }
    }
    portEXIT_CRITICAL(&s_slot_mux);
    if (got < 0) {
        portENTER_CRITICAL(&s_stats_mux);
        s_backpressure++;
        portEXIT_CRITICAL(&s_stats_mux);
    } else {
        /* Producer preparation starts here (fill + read from the source). */
        s_slot_fill_us[got] = esp_timer_get_time();
    }
    return got;
}

int32_t *dsp_pcm_slot(int slot)
{
    if (slot < 0 || slot >= (int)DSP_PCM_SLOT_COUNT) {
        return NULL;
    }
    return s_slot_pcm[slot];
}

void dsp_pcm_release(int slot)
{
    if (slot < 0 || slot >= (int)DSP_PCM_SLOT_COUNT) {
        return;
    }
    portENTER_CRITICAL(&s_slot_mux);
    s_slot_state[slot] = SLOT_FREE;
    portEXIT_CRITICAL(&s_slot_mux);
}

static void dsp_slot_free_core1(int slot)
{
    /* Release under the same short lock used by the producer's acquire. */
    portENTER_CRITICAL(&s_slot_mux);
    s_slot_state[slot] = SLOT_FREE;
    portEXIT_CRITICAL(&s_slot_mux);
}

esp_err_t dsp_pcm_submit(int slot, size_t samples, uint32_t rate, dsp_source_t source,
                         uint32_t flags, uint32_t epoch)
{
    if (slot < 0 || slot >= (int)DSP_PCM_SLOT_COUNT || s_queue == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (samples == 0 || samples > DSP_PCM_SLOT_SAMPLES || (samples & 1u) != 0) {
        return ESP_ERR_INVALID_ARG;
    }

    core1_item_t it;
    memset(&it, 0, sizeof(it));
    it.type = CORE1_ITEM_PCM;
    it.slot = (uint8_t)slot;
    it.source = (uint8_t)source;
    it.flags = (uint8_t)flags;
    it.samples = (uint16_t)samples;
    it.rate = rate;
    it.epoch = epoch;
    it.fill_start_us = s_slot_fill_us[slot];
    it.submit_us = esp_timer_get_time();

    s_slot_state[slot] = SLOT_CORE1;
    if (xQueueSend(s_queue, &it, 0) != pdTRUE) {
        /* The queue is sized for every slot; a full queue means the worker is
         * not draining at all - keep the real error and the slot mine. */
        s_slot_state[slot] = SLOT_CORE0;
        return ESP_ERR_TIMEOUT;
    }
    return ESP_OK;
}

esp_err_t dsp_core1_ctl(uint32_t cmd, uint32_t a0, uint32_t a1)
{
    if (!s_ready || s_queue == NULL || s_ctl_ack == NULL || s_ctl_mutex == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    if (xSemaphoreTake(s_ctl_mutex, pdMS_TO_TICKS(DSP_CTL_ACK_TIMEOUT_MS)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    /* Format/rate/slot controls serialize against the producer's entire fill
     * and submit operation. DSP sync leaves the producer running: any queued
     * PCM measures the rebuild wait in its end-to-end wall time. */
    const bool gate = cmd != DSP_CTL_ENGINE_SYNC;
    if (gate) xSemaphoreTake(s_supply_mutex, portMAX_DELAY);
    core1_item_t it;
    memset(&it, 0, sizeof(it));
    it.type = CORE1_ITEM_CTL;
    it.ctl_cmd = cmd;
    it.ctl_a0 = a0;
    it.ctl_a1 = a1;

    esp_err_t ret = ESP_OK;
    if (xQueueSend(s_queue, &it, pdMS_TO_TICKS(DSP_CTL_POST_TIMEOUT_MS)) != pdTRUE) {
        ret = ESP_ERR_TIMEOUT;
    } else {
        /* A posted command owns this ACK until completion. Abandoning it on a
         * timeout would let the next caller consume this command's late ACK. */
        xSemaphoreTake(s_ctl_ack, portMAX_DELAY);
        ret = s_ctl_result;
    }
    if (gate) xSemaphoreGive(s_supply_mutex);

    xSemaphoreGive(s_ctl_mutex);
    return ret;
}

bool dsp_core1_running(void)
{
    return s_ready;
}

bool dsp_pcm_begin(void)
{
    if (xSemaphoreTake(s_supply_mutex, 0) == pdTRUE) return true;
    portENTER_CRITICAL(&s_stats_mux);
    s_control_skips++;
    portEXIT_CRITICAL(&s_stats_mux);
    return false;
}

void dsp_pcm_end(void)
{
    xSemaphoreGive(s_supply_mutex);
}

/* ---- output stage ---- */

/* Convert interleaved int32 stereo PCM to the configured slot width, in place.
 * Every slot width packs at most as many bytes per sample as the int32 source,
 * so a forward pass can never clobber input that has not been read yet. */
static size_t dsp_convert_slot_inplace(int32_t *pcm, size_t samples, int slot_bits)
{
    if (slot_bits == 32) {
        return samples * sizeof(int32_t);
    }
    uint8_t *dst = (uint8_t *)pcm;
    if (slot_bits == 24) {
        size_t out = 0;
        for (size_t i = 0; i < samples; i++) {
            const int32_t v = pcm[i] >> 8;   /* arithmetic shift keeps the sign */
            dst[out++] = (uint8_t)((uint32_t)v & 0xFFu);
            dst[out++] = (uint8_t)(((uint32_t)v >> 8) & 0xFFu);
            dst[out++] = (uint8_t)(((uint32_t)v >> 16) & 0xFFu);
        }
        return out;
    }
    size_t out = 0;
    for (size_t i = 0; i < samples; i++) {
        const uint32_t v = (uint32_t)pcm[i] >> 16;
        dst[out++] = (uint8_t)v;
        dst[out++] = (uint8_t)(v >> 8);
    }
    return out;
}

/* Push the converted block into the output stage. `written` from
 * i2s_out_write() is the durable prefix copied into the stage; a suffix that
 * the stage could not take is retried from where it stopped (never resent) and
 * counted as undelivered if the driver queue stays saturated past the bound. */
static size_t dsp_output_push(const uint8_t *bytes, size_t size)
{
    size_t accepted = 0;
    int retries = 0;

    while (accepted < size) {
        size_t w = 0;
        esp_err_t ret = i2s_out_write(bytes + accepted, size - accepted, &w);
        accepted += w;
        if (accepted >= size) {
            break;
        }
        if (ret != ESP_OK && ret != ESP_ERR_TIMEOUT) {
            ESP_LOGE(TAG, "i2s_out_write error: %s (%u/%u bytes)",
                     esp_err_to_name(ret), (unsigned)accepted, (unsigned)size);
            portENTER_CRITICAL(&s_stats_mux); s_write_errors++; portEXIT_CRITICAL(&s_stats_mux);
            portENTER_CRITICAL(&s_stats_mux); s_write_partial++; portEXIT_CRITICAL(&s_stats_mux);
            break;
        }
        if (++retries > DSP_OUT_RETRY_MAX) {
            portENTER_CRITICAL(&s_stats_mux); s_write_partial++; portEXIT_CRITICAL(&s_stats_mux);
            break;
        }
        portENTER_CRITICAL(&s_stats_mux); s_write_retry++; portEXIT_CRITICAL(&s_stats_mux);
        vTaskDelay(1);
    }
    return accepted;
}

static void dsp_process_pcm(const core1_item_t *it)
{
    const int64_t t_deq = esp_timer_get_time();
    int32_t *pcm = s_slot_pcm[it->slot];
    int32_t *pcm1 = s_port1_pcm;
    const size_t samples = it->samples;
    const uint32_t flags = it->flags;

    if (it->epoch != i2s_out_get_epoch()) {
        /* Produced for a previous output configuration: the epochs changed
         * while this descriptor was queued. Never output it as new PCM. */
        portENTER_CRITICAL(&s_stats_mux); s_stale_epoch++; portEXIT_CRITICAL(&s_stats_mux);
        dsp_slot_free_core1(it->slot);
        return;
    }

    uint32_t dsp_us = 0, peak_l = 0, peak_r = 0;
    bool all_zero = false;
    if ((flags & DSP_PCM_F_SILENCE) != 0u) {
        memset(pcm, 0, samples * sizeof(int32_t));
        memset(pcm1, 0, samples * sizeof(int32_t));
        /* Counters are committed together after output acceptance. */
    } else {
        if (s_enabled && (flags & DSP_PCM_F_ENGINE_BYPASS) == 0u) {
            const int64_t t0 = esp_timer_get_time();
            dsp_engine_process(pcm, pcm1, samples, it->rate, (dsp_source_t)it->source);
            dsp_us = (uint32_t)(esp_timer_get_time() - t0);
        } else if (s_enabled) {
            /* Forced engine bypass: port 1 mirrors the untouched input. */
            memcpy(pcm1, pcm, samples * sizeof(int32_t));
        } else {
            memset(pcm1, 0, samples * sizeof(int32_t));
        }


        /* One pass for both peak stores and the all-zero block flag (no extra
         * scan of the same block). */
        all_zero = true;
        for (size_t i = 0; i + 1 < samples; i += 2) {
            const uint32_t al = dsp_abs_sample(pcm[i]);
            const uint32_t ar = dsp_abs_sample(pcm[i + 1]);
            if ((al | ar) != 0u) {
                all_zero = false;
            }
            if (al > peak_l) peak_l = al;
            if (ar > peak_r) peak_r = ar;
        }
    }

    /* These counters describe int32 PCM before packed16/24 conversion. */
    for (size_t i = 0; i < samples; i++) {
        if (pcm1[i] != 0) { g_dbg_p1_nonzero++; break; }
    }
    if (!all_zero && (flags & DSP_PCM_F_SILENCE) == 0u) g_dbg_p0_nonzero++;
    const int64_t t_conv0 = esp_timer_get_time();
    const int slot_bits = i2s_out_get_slot_width();
    size_t bytes;
    if ((flags & DSP_PCM_F_SILENCE) != 0u) {
        /* The whole descriptor is already zero: the packed output is a prefix
         * of it, so no separate zero buffer and no conversion is needed. */
        bytes = samples * (size_t)(slot_bits / 8);
    } else {
        bytes = dsp_convert_slot_inplace(pcm, samples, slot_bits);
    }

    const size_t accepted = dsp_output_push((const uint8_t *)pcm, bytes);

    if (i2s1_out_is_running()) {
        size_t bytes1 = dsp_convert_slot_inplace(pcm1, samples, slot_bits);
        size_t written1 = 0;
        esp_err_t r1 = i2s1_out_write(pcm1, bytes1, &written1);
        g_dbg_i2s1_calls++;
        g_dbg_i2s1_bytes += (uint32_t)written1; /* accepted bytes, not requested */
        if (r1 != ESP_OK || written1 != bytes1) {
            g_dbg_i2s1_err++;
            portENTER_CRITICAL(&s_stats_mux);
            if (r1 != ESP_OK) s_write_errors++;
            if (written1 != bytes1) s_write_partial++;
            portEXIT_CRITICAL(&s_stats_mux);
        }
    }

    const int64_t t_done = esp_timer_get_time();

    const size_t stride = i2s_out_get_sample_stride_bytes();
    /* Per-block telemetry: producer fill, queue wait, DSP net, port
     * conversion+submit and the total wall clock from "producer started
     * filling" to "core1 accepted the whole block". */
    const uint32_t prep_us = (it->submit_us > it->fill_start_us)
        ? (uint32_t)(it->submit_us - it->fill_start_us) : 0u;
    const uint32_t queue_us = (t_deq > it->submit_us)
        ? (uint32_t)(t_deq - it->submit_us) : 0u;
    const uint32_t port_us = (uint32_t)(t_done - t_conv0);
    const uint32_t total_us = (t_done > it->fill_start_us)
        ? (uint32_t)(t_done - it->fill_start_us) : 0u;
    const uint32_t deadline = dsp_deadline_us(it->rate);

    portENTER_CRITICAL(&s_stats_mux);
    if ((flags & DSP_PCM_F_SILENCE) != 0u) s_silence_blocks++;
    else {
        if ((flags & DSP_PCM_F_ENGINE_BYPASS) != 0u) s_bypass_blocks++;
        if (all_zero) s_out_zero_blocks++;
        if (peak_l > s_max_peak_l) s_max_peak_l = peak_l;
        if (peak_r > s_max_peak_r) s_max_peak_r = peak_r;
        if (peak_l > s_win_peak_l) s_win_peak_l = peak_l;
        if (peak_r > s_win_peak_r) s_win_peak_r = peak_r;
        if (peak_l > s_out_peak_l) s_out_peak_l = peak_l;
        if (peak_r > s_out_peak_r) s_out_peak_r = peak_r;
    }
    s_last_us = dsp_us;
    s_blocks++;
    s_samples_total += (uint32_t)samples;
    if (it->source == DSP_SOURCE_UAC) s_uac_blocks++;
    else if (it->source == DSP_SOURCE_BT) s_bt_blocks++;
    else s_test_blocks++;
    s_frames_in += (uint32_t)(samples / 2u);
    if (stride != 0) {
        s_frames_stage += (uint32_t)(accepted / stride);
        if (accepted < bytes) {
            s_frames_undelivered += (uint32_t)((bytes - accepted) / stride);
        }
    }

    s_prep_sum_us += prep_us;
    s_queue_sum_us += queue_us;
    s_dsp_sum_us += dsp_us;
    s_port_sum_us += port_us;
    s_total_sum_us += total_us;
    s_sum_blocks++;
    if (prep_us > s_prep_max_us) s_prep_max_us = prep_us;
    if (queue_us > s_queue_max_us) s_queue_max_us = queue_us;
    if (dsp_us > s_dsp_max_us) s_dsp_max_us = dsp_us;
    if (port_us > s_port_max_us) s_port_max_us = port_us;
    if (total_us > s_total_max_us) s_total_max_us = total_us;
    if (prep_us > s_w_prep_max_us) s_w_prep_max_us = prep_us;
    if (queue_us > s_w_queue_max_us) s_w_queue_max_us = queue_us;
    if (dsp_us > s_w_dsp_max_us) s_w_dsp_max_us = dsp_us;
    if (port_us > s_w_port_max_us) s_w_port_max_us = port_us;
    if (total_us > s_w_total_max_us) s_w_total_max_us = total_us;
    s_w_blocks++;

    if (it->source == DSP_SOURCE_UAC) {
        if (total_us > s_total_max_uac_us) s_total_max_uac_us = total_us;
    } else if (it->source == DSP_SOURCE_BT) {
        if (total_us > s_total_max_bt_us) s_total_max_bt_us = total_us;
    }
    if ((flags & DSP_PCM_F_ENGINE_BYPASS) != 0u) {
        if (total_us > s_total_max_bypass_us) s_total_max_bypass_us = total_us;
    }
    if (total_us > deadline) {
        s_deadline_miss++;
    }
    if ((uint64_t)total_us * 10u > (uint64_t)deadline * 9u) {
        s_budget90_over++;
    }

    if (dsp_us < s_win_min_us) s_win_min_us = dsp_us;
    if (dsp_us > s_win_max_us) s_win_max_us = dsp_us;
    s_win_sum_us += dsp_us;
    s_win_blocks++;
    portEXIT_CRITICAL(&s_stats_mux);

    dsp_slot_free_core1(it->slot);
}

static void dsp_task(void *arg)
{
    (void)arg;

    /* The I2S channel is created here so its DMA interrupt is allocated on
     * core1; core0 must never own it. */
    s_ready_err = dsp_engine_init();
    if (s_ready_err == ESP_OK) s_ready_err = i2s_out_core1_init();
    s_ready = s_ready_err == ESP_OK;
    if (s_ready_sem != NULL) {
        xSemaphoreGive(s_ready_sem);
    }
    if (s_ready_err != ESP_OK) {
        ESP_LOGE(TAG, "core1 I2S init failed: %s", esp_err_to_name(s_ready_err));
        s_task = NULL;
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "audio worker ready: actual_core=%d (DSP/I2S0/DMA owner), enabled=%d",
             xPortGetCoreID(), s_enabled ? 1 : 0);

    core1_item_t it;
    for (;;) {
        if (xQueueReceive(s_queue, &it, portMAX_DELAY) != pdTRUE) {
            continue;
        }
        if (it.type == CORE1_ITEM_CTL) {
            const int64_t ctl_start = esp_timer_get_time();
            if (it.ctl_cmd == DSP_CTL_ENGINE_SYNC) {
                dsp_engine_sync();
                s_ctl_result = ESP_OK;
            } else {
                s_ctl_result = i2s_out_core1_exec(it.ctl_cmd, it.ctl_a0, it.ctl_a1);
            }
            const uint32_t ctl_us = (uint32_t)(esp_timer_get_time() - ctl_start);
            portENTER_CRITICAL(&s_stats_mux);
            if (ctl_us > s_control_max_us) s_control_max_us = ctl_us;
            portEXIT_CRITICAL(&s_stats_mux);
            xSemaphoreGive(s_ctl_ack);
            continue;
        }
        dsp_process_pcm(&it);
    }
}

esp_err_t dsp_audio_init(void)
{
    s_enabled = CONFIG_DSP_ENABLE ? true : false;

    if (s_queue != NULL) {
        return ESP_OK;
    }

    for (int i = 0; i < (int)DSP_PCM_SLOT_COUNT; i++) {
        s_slot_state[i] = SLOT_FREE;
    }

    s_queue = xQueueCreate(DSP_QUEUE_LEN, sizeof(core1_item_t));
    s_ctl_ack = xSemaphoreCreateBinary();
    s_ctl_mutex = xSemaphoreCreateMutex();
    s_supply_mutex = xSemaphoreCreateMutex();
    s_ready_sem = xSemaphoreCreateBinary();
    if (s_queue == NULL || s_ctl_ack == NULL || s_ctl_mutex == NULL || s_supply_mutex == NULL || s_ready_sem == NULL) {
        ESP_LOGE(TAG, "queue/semaphore create failed");
        return ESP_ERR_NO_MEM;
    }

    BaseType_t ret = xTaskCreatePinnedToCore(dsp_task, "dsp", DSP_TASK_STACK,
                                             NULL, DSP_TASK_PRIO, &s_task, 1);
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "core1 worker create failed: %d", (int)ret);
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

esp_err_t dsp_audio_wait_ready(uint32_t timeout_ms)
{
    if (s_ready_sem == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    if (xSemaphoreTake(s_ready_sem, pdMS_TO_TICKS(timeout_ms)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    /* The worker only signals once: put the token back so a later caller can
     * still observe the same result. */
    xSemaphoreGive(s_ready_sem);
    return s_ready_err;
}

bool dsp_audio_is_enabled(void)
{
    return s_enabled;
}

static void dsp_reset_window_locked(void)
{
    s_win_min_us = UINT32_MAX;
    s_win_max_us = 0;
    s_win_sum_us = 0;
    s_win_blocks = 0;
    s_w_prep_max_us = 0;
    s_w_queue_max_us = 0;
    s_w_dsp_max_us = 0;
    s_w_port_max_us = 0;
    s_w_total_max_us = 0;
    s_w_blocks = 0;
    s_prep_sum_us = 0;
    s_queue_sum_us = 0;
    s_dsp_sum_us = 0;
    s_port_sum_us = 0;
    s_total_sum_us = 0;
    s_sum_blocks = 0;
}

void dsp_audio_reset_block_stats(void)
{
    portENTER_CRITICAL(&s_stats_mux);
    dsp_reset_window_locked();
    portEXIT_CRITICAL(&s_stats_mux);
}

uint32_t dsp_audio_get_stack_hwm(void)
{
    if (s_task == NULL) {
        return 0;
    }
    /* Bytes in this fork (see DSP_TASK_STACK). */
    return (uint32_t)uxTaskGetStackHighWaterMark(s_task);
}

void dsp_audio_take_peaks(uint32_t *peak_l, uint32_t *peak_r)
{
    portENTER_CRITICAL(&s_stats_mux);
    if (peak_l != NULL) {
        *peak_l = s_win_peak_l;
    }
    if (peak_r != NULL) {
        *peak_r = s_win_peak_r;
    }
    s_win_peak_l = 0;
    s_win_peak_r = 0;
    portEXIT_CRITICAL(&s_stats_mux);
}

void dsp_audio_take_output_peaks(uint32_t *peak_l, uint32_t *peak_r)
{
    portENTER_CRITICAL(&s_stats_mux);
    if (peak_l != NULL) {
        *peak_l = s_out_peak_l;
    }
    if (peak_r != NULL) {
        *peak_r = s_out_peak_r;
    }
    s_out_peak_l = 0;
    s_out_peak_r = 0;
    portEXIT_CRITICAL(&s_stats_mux);
}

static void dsp_snapshot_stats(dsp_audio_stats_t *stats, bool take_window)
{
    if (stats == NULL) {
        return;
    }
    memset(stats, 0, sizeof(*stats));

    stats->clipped = dsp_engine_get_clips();
    i2s_out_stats_t ist = {0};
    i2s_out_get_stats(&ist);
    stats->stage_pending_bytes = ist.stage_pending_bytes;
    stats->deadline_us = dsp_deadline_us(i2s_out_get_rate());
    portENTER_CRITICAL(&s_stats_mux);
    stats->blocks       = s_blocks;
    stats->uac_blocks   = s_uac_blocks;
    stats->bt_blocks    = s_bt_blocks;
    stats->test_blocks  = s_test_blocks;
    stats->samples      = s_samples_total;
    stats->max_peak_l   = s_max_peak_l;
    stats->max_peak_r   = s_max_peak_r;
    stats->last_us      = s_last_us;

    const uint32_t win_blocks = s_win_blocks;
    const uint64_t win_sum    = s_win_sum_us;
    stats->min_us = (win_blocks == 0 || s_win_min_us == UINT32_MAX) ? 0 : s_win_min_us;
    stats->max_us = s_win_max_us;
    stats->avg_us = (win_blocks == 0) ? 0 : (uint32_t)(win_sum / win_blocks);

    stats->silence_blocks     = s_silence_blocks;
    stats->bypass_blocks      = s_bypass_blocks;
    stats->stale_epoch        = s_stale_epoch;
    stats->backpressure       = s_backpressure;
    stats->out_zero_blocks    = s_out_zero_blocks;
    stats->frames_in          = s_frames_in;
    stats->frames_stage       = s_frames_stage;
    stats->frames_undelivered = s_frames_undelivered;
    stats->write_retry        = s_write_retry;
    stats->write_partial      = s_write_partial;
    stats->write_errors       = s_write_errors;

    stats->deadline_miss  = s_deadline_miss;
    stats->budget90_over  = s_budget90_over;

    stats->prep_max_us  = s_prep_max_us;
    stats->queue_max_us = s_queue_max_us;
    stats->dsp_max_us   = s_dsp_max_us;
    stats->port_max_us  = s_port_max_us;
    stats->total_max_us = s_total_max_us;
    stats->total_max_uac_us    = s_total_max_uac_us;
    stats->total_max_bt_us     = s_total_max_bt_us;
    stats->total_max_bypass_us = s_total_max_bypass_us;

    stats->win_blocks        = s_w_blocks;
    stats->win_prep_max_us   = s_w_prep_max_us;
    stats->win_queue_max_us  = s_w_queue_max_us;
    stats->win_dsp_max_us    = s_w_dsp_max_us;
    stats->win_port_max_us   = s_w_port_max_us;
    stats->win_total_max_us  = s_w_total_max_us;
    stats->prep_sum_us  = s_prep_sum_us;
    stats->queue_sum_us = s_queue_sum_us;
    stats->dsp_sum_us   = s_dsp_sum_us;
    stats->port_sum_us  = s_port_sum_us;
    stats->total_sum_us = s_total_sum_us;
    stats->control_max_us = s_control_max_us;
    stats->control_skips = s_control_skips;
    if (take_window) dsp_reset_window_locked();
    portEXIT_CRITICAL(&s_stats_mux);
}

void dsp_audio_get_stats(dsp_audio_stats_t *stats)
{
    dsp_snapshot_stats(stats, false);
}

void dsp_audio_take_stats(dsp_audio_stats_t *stats)
{
    dsp_snapshot_stats(stats, true);
}
