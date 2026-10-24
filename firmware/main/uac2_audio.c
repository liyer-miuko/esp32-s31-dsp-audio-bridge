/*
 * UAC2 asynchronous speaker stream implementation - ESP32-S31.
 *
 * Data path (all timing is derived from the I2S master clock):
 *
 *   USB ISO OUT packet (ISR)
 *     -> TinyUSB internal EP-OUT FIFO (no app copy in USB ISR)
 *     -> [I2S TX-DMA on_sent interrupt]
 *     -> core0 uac_pump_task: FIFO read + descriptor submission
 *     -> core1 DSP / ports / I2S (frozen)
 *     -> I2S0 -> TAS5828/DAC
 *
 * Feedback (the USB equivalent of the clock-domain isolation an ASIO client
 * needs): TinyUSB's built-in FIFO_COUNT regulator targets 2048 bytes in the
 * unchanged 6656-byte OUT FIFO. The FIFO is drained by the I2S-clock-driven pump task,
 * which is the intended operating mode for this feedback method.
 */
#include <string.h>
#include <stdio.h>
#include <inttypes.h>
#include <math.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_system.h"
#include "esp_private/usb_phy.h"
#include "soc/usb_otghs_reg.h"
#include "tusb.h"
#include "i2s_out_v2.h"
#include "i2s1_out.h"
#include "uac2_audio.h"
#include "uac2_descriptors.h"
#include "a2dp_audio.h"
#include "bt_codec_guard.h"
#include "audio_input_budget.h"
#include "hw_mute.h"


/* 0.135: preserve same-format OUT tail, then prebuffer the next stream.
 * A running stream drains every complete frame even below this start level.
 * Start at a whole DSP block, or release the available complete frames after
 * 30 ms. A short prefill is never discarded merely for missing that level. */
#define UAC_RESUME_LEVEL_BYTES  1024u
#define UAC_RESUME_TIMEOUT_MS   30u
#include "dsp_audio.h"
#include "dsp_config.h"
#include "dsp_engine.h"
#include "dsp_version.h"
#include "esp_timer.h"

static const char *TAG = "uac2";

/* ---------------- Output source selection ---------------- */

typedef enum {
    AUDIO_SOURCE_UAC = 0,
    AUDIO_SOURCE_BT = 1,
} audio_source_t;

/* ---------------- UAC constants ---------------- */
#define UAC_EP_OUT              0x01
#define UAC_EP_FB               0x81
#define UAC_FB_EP_SIZE          4
#define UAC_FB_EP_INTERVAL      4   /* HS: 2^(4-1) = 8 uframes = 1 ms */

/* 96 kHz stereo + one sample slack. 24-bit alt uses a 3-byte subslot, so its
 * max packet is 78 bytes; 104 covers both formats and feedback overshoot. */
#define UAC_EP_OUT_MAX_PACKET   104
/* Real FIFO size lives in tusb_config.h (v9: 104*64 = ~8.67 ms @96k/32).
 * This alias is unused; kept only as a synchronized reference. */
#define UAC_EP_OUT_FIFO_BYTES   (UAC_EP_OUT_MAX_PACKET * 64)

#define UAC_PUMP_BUF_BYTES          I2S_OUT_DMA_FRAME_BYTES_MAX /* 32-bit processing buffer */
#define UAC_PUMP_24_USB_MAX_BYTES   (I2S_OUT_DMA_FRAME_NUM * 2 * 3) /* 240 frames packed-24 */
/* 0.137: user-selected 2048-byte target, 1280 below the old half-fill (3328).
 * Keep FIFO capacity, built-in filtering/clamps and 240-frame cadence.
 * FIFO_COUNT derives its quantized gains from this target as well. */
#define UAC_FB_TARGET_BYTES         2048u

/* One time-based startup policy, shared with unselected producer prewarm.
 * Once selected, BT frames are retained until read or a real stream boundary. */
static inline uint32_t bt_level_bytes(uint32_t ms, uint32_t rate)
{
    /* 8 B = one 32-bit stereo frame in the BT ring, any sample rate. */
    return (uint32_t)(((uint64_t)rate * ms / 1000u) * 8u);
}

static inline uint32_t bt_prebuffer_bytes(uint32_t rate, const a2dp_audio_state_t *bt)
{
    return bt_level_bytes(a2dp_bt_prebuffer_ms(rate, bt->codec_vendor_id, bt->codec_codec_id),
                          rate);
}


/* Selected BT content is never trimmed merely to chase a target latency.
 * The producer may trim only unselected startup PCM; those bytes remain
 * explicitly counted as prep_trim. Ring exhaustion is a real error. */

static const uint32_t s_sample_rates[] = {
    44100, 48000, 88200, 96000,
};
#define UAC_N_RATES TU_ARRAY_SIZE(s_sample_rates)

/* Control task events */
#define EVT_RATE        (1 << 0)
#define EVT_FORMAT      (1 << 1)
#define EVT_SWITCH      (1 << 3)

/* Source switch state machine (all transitions go through digital mute). */
typedef enum {
    SWITCH_IDLE = 0,
    SWITCH_SETTLING,
    SWITCH_MUTED,
    SWITCH_RECOVER_WAIT,
    SWITCH_FAULT,
} switch_phase_t;

#define SOURCE_RETRY_US 250000
#define SOURCE_MAX_FAILURES 3u

typedef struct {
    audio_source_t source;
    uint32_t       rate;
    uint32_t       bt_epoch, bt_packets, bt_delivered;
    uint32_t       uac_format_seq;  /* fallback must still target this USB format */
} switch_request_t;

/* ---------------- Runtime state ---------------- */
typedef struct {
    volatile uint32_t sample_rate;   /* applied UAC format, independent of BT clock */
    volatile int      bit_depth;     /* applied USB sample width: 24 or 32 */
    volatile bool     streaming;     /* AS alt != 0 and USB bus not suspended */
    volatile uint32_t usb_pkts;      /* ISO OUT packets received */
    volatile uint32_t pump_runs;     /* DMA-clocked producer ticks */
    volatile uint32_t silence_runs;  /* ticks without UAC data (BUG-1 split counter) */
    volatile uint32_t pump_want0;    /* avail>0 but shorter than one full frame */
    volatile uint32_t pump_nodata;   /* tick delivered no data (incl. the above) */
    volatile uint32_t usb_bytes;     /* bytes read from the USB FIFO */
    volatile uint32_t background_usb_bytes; /* unselected USB discard, not output loss */
    volatile uint32_t background_usb_reads;
    volatile uint32_t short_reads;   /* tud_audio_read returned less than asked */
    volatile uint32_t submit_blocks; /* descriptors handed to core1 */
    volatile uint32_t submit_frames; /* int32 stereo frames handed to core1 */
    volatile uint32_t submit_errors; /* dsp_pcm_submit refused a prepared slot */
    /* 0.113 producer-side window: tick start -> descriptor submitted. core0
     * never waits for core1 here, so this is core0's own production cost;
     * the end-to-end numbers live in the core1 telemetry (`c1out:`). */
    volatile uint32_t p_sum_us, p_n, p_max_us;
    volatile uint32_t def_silent;    /* silence frames outside the audible window */
    volatile int64_t  last_data_us;  /* last time real PCM was read (audible window) */
    /* ★ 0.113 hw_mute activity: one increment per tick in which ANY input path
     * received a complete UAC PCM frame (BT uses its decoder callback count).
     * All-zero PCM is still a stream; a mere connection is not. */
    volatile uint32_t pcm_active_runs;

    /* Latest USB request and completed/cancelled generation. Core0 callbacks
     * publish under s_uac_buffer_mux; ctrl never clears a newer generation. */
    volatile uint32_t requested_rate;
    volatile int      requested_bits;
    volatile uint32_t format_request_seq, format_done_seq;
    volatile uint8_t  stream_alt;
    volatile bool     bus_suspended;
    volatile bool     format_input_busy;  /* USB boundary not yet ready for ctrl */
    volatile bool     fifo_rearm_clear;   /* incompatible old ISO may complete late */

    /* Peak diagnostics: USB-side (before the DSP chain and the UAC volume
     * stage). The output-side peaks are produced by core1 (dsp_audio). */
    volatile uint32_t usb_peak_l;
    volatile uint32_t usb_peak_r;

    /* FIFO water-level and underrun diagnostics */
    volatile uint16_t fifo_min;
    volatile uint16_t fifo_max;
    volatile uint32_t active_silence_runs; /* silence while stream is open */
    volatile uint32_t bt_silence_runs;     /* BT active but PCM ring underrun（纯累计，只增不清）*/
    volatile uint32_t bt_silence_streak;   /* ★ 0.158：**连续**欠载块数，成功出块即清零 */
    /* ★ 0.164：切码静音时长（SUSPEND → 首个真实出块）。这是把"seek 感"量化的
     * 唯一直接指标：如果是"停顿太久"，gap 会很大；如果 gap 很小而听感仍然异常，
     * 那问题就在内容拼接或 core1 侧，不在等待时间。 */
    volatile int64_t  bt_gap_start_us;
    volatile uint32_t bt_last_gap_ms;
    volatile uint32_t bt_max_gap_ms;
    /* ★ 0.165：供给侧停滞（packets 不再增长但 streaming 仍为 true）。
     * 这是把"990 断流"归因到手机侧还是设备侧的关键证据。 */
    volatile int64_t  bt_starve_start_us;
    volatile uint32_t bt_last_starve_ms;
    volatile uint32_t bt_max_starve_ms;
    volatile uint32_t bt_starve_events;    /* ★ 0.50（L44）H3：本 tick **没交付任何真实音频**的累计次数。
     * 这是 hw_mute 判 `playing` 的新依据。旧依据 `silence_runs` 只在 UAC 分支自增
     * （见 pump 里那一处 ++），于是 **BT 模式下它永不前进 ⇒ playing 恒为真 ⇒
     * 功放整段 BT 会话都开着**。现场实测（`l40_v45_retest_serial.txt`）：
     *   675954 `hw_mute: unmuted` ↔ 677767/678768 `peaks: i2s L/R -999.00 dBFS`
     *   —— 功放开着、输出却是数字静音 ⇒ **BT 模式下任何杂音/垃圾帧都满幅可闻**。
     * 这也是 1.5 s / 2.0 s 窗**治不了**"连接时那段异常音"的原因：窗只盖 2 s，
     * 功放却在 BT 会话里一直开着。
     * ⚠ `silence_runs` 的语义**不动**（它是判据/量具契约），这里只**新增**一个计数器。 */
    volatile uint32_t nonaudio_runs;       /* ticks that delivered no real audio */
    volatile bool     have_audio;          /* at least one data chunk has flowed */

    /* Source routing / switch state */
    volatile audio_source_t active_source;
    volatile uint32_t output_rate;
    volatile bool     source_switching;    /* pump must output silence */
    volatile bool     bt_gate_open;        /* BT PCM may be heard only after hold */
    volatile uint32_t bt_gate_epoch;       /* close gate even if CFG finishes between polls */
    volatile switch_phase_t switch_phase; /* ctrl owns; pump reads */
    uint32_t         switch_failures, switch_failed_bt_epoch, switch_failed_uac_seq;
    int64_t          switch_retry_us;
    esp_err_t        switch_last_error;
#if CONFIG_UAC2_BT_INJECT
    volatile bool     inject_mode;         /* 0.151：PCM 注入已启用 */
    volatile uint32_t inject_bytes;        /* 已注入的字节（诊断） */
#endif
    switch_request_t  switch_req;
    volatile bool     switch_req_pending;

    /* ★ 0.61（L46 修 f）：切进度后"等水位稳定再放歌" */
    volatile bool     uac_resume_hold;     /* 正在等 FIFO 水位（期间不读、输出静音） */

    /* Only ctrl owns this 64-bit timestamp. Pump publishes a 32-bit successful
     * block count; ctrl samples it and new decoder arrivals every 10 ms.
     * A residual partial block is not a heartbeat. Digital-zero PCM is. */
    int64_t           bt_alive_us;
    uint32_t          bt_last_packets, bt_last_delivered;
    volatile uint32_t bt_delivered_blocks;
    volatile uint32_t bt_drain_blocks;   /* 0.145：排空旧代次期间成功播出的块数 */

    usb_phy_handle_t phy_hdl;
    TaskHandle_t ctrl_task;
} uac_ctx_t;

static volatile bool s_dsp_test_bypass;
static uac_ctx_t s_ctx = {
    .sample_rate = 48000,
    .bit_depth = 32,
    .streaming = false,
    .requested_rate = 48000,
    .requested_bits = 32,
    .fifo_min = 0xFFFF,
    .fifo_max = 0,
    .have_audio = false,
    .active_source = AUDIO_SOURCE_UAC,
    .output_rate = 48000,
    .source_switching = false,
    .bt_gate_open = false,
    .switch_phase = SWITCH_IDLE,
    .switch_req = { .source = AUDIO_SOURCE_UAC, .rate = 48000 },
    .switch_req_pending = false,


    .uac_resume_hold = false,

    .bt_alive_us = 0,
};

/* 0.155：tick 级轨迹已移除（32 KB 静态数组 + 翻倍环导致 DRAM 不足重启）。 */

/* Volume / mute state (feature unit). These are written from the TinyUSB
 * control task and applied by the audio pump task. */
static volatile int16_t s_volume[3] = {0, 0, 0}; /* 1/256 dB, master + L + R */
static volatile uint8_t s_mute[3] = {0, 0, 0};   /* master + L + R */

/* 0.113: core0 keeps no audio buffer of its own. The 32-bit path reads the USB
 * FIFO straight into the PCM slot handed over by dsp_pcm_acquire(); only the
 * packed-24 path needs a staging buffer before it is expanded into the slot. */
static uint8_t s_usb24_buf[UAC_PUMP_24_USB_MAX_BYTES] __attribute__((aligned(4)));

#if CONFIG_UAC2_BT_INJECT
/* ★ 0.151：注入用的独立搬运行（不与 s_usb24_buf 共用，避免交错）。 */
static uint8_t s_inject_buf[DSP_PCM_SLOT_BYTES] __attribute__((aligned(4)));
#endif

/* 0.208: production BT diagnostics need aggregates, not a permanent 16 KiB
 * tick tape. No reader exports individual entries: the former report scanned
 * 2048 records only to produce these five statistics. Keep exactly the stats
 * for each ctrl-report interval, with explicit window semantics. Raw B6 tapes
 * and all audio/ring/loss counters are independent and unchanged.
 *
 * Pump updates; ctrl takes/reset snapshots. Both tasks run on core0; a short
 * mux prevents a higher-priority pump from partially modifying the snapshot.
 * No heap allocation, PCM writes or UART work in the pump. */
typedef struct {
    uint32_t n, gate_flips, trim_ticks, sil_ticks;
    uint16_t min_cs, max_cs;
    uint8_t prev_st;
    bool prev_valid;
} bt_trace_window_t;
static bt_trace_window_t s_bt_trace_window = { .min_cs = UINT16_MAX };
static portMUX_TYPE s_bt_trace_mux = portMUX_INITIALIZER_UNLOCKED;

static inline void bt_trace_push(uint32_t us, size_t ring_bytes, uint32_t rate,
                                 uint8_t st, uint8_t ev)
{
    (void)us;
    uint32_t cs = 0;
    if (rate != 0u) {
        cs = (uint32_t)(((uint64_t)ring_bytes * 100000u) / ((uint64_t)rate * 8u));
        if (cs > UINT16_MAX) cs = UINT16_MAX;
    }
    portENTER_CRITICAL(&s_bt_trace_mux);
    bt_trace_window_t *w = &s_bt_trace_window;
    if (cs < w->min_cs) w->min_cs = (uint16_t)cs;
    if (cs > w->max_cs) w->max_cs = (uint16_t)cs;
    if (w->prev_valid && ((st ^ w->prev_st) & 0x01u)) w->gate_flips++;
    if (ev & 0x04u) w->trim_ticks++;
    if (ev & 0x02u) w->sil_ticks++;
    w->n++;
    w->prev_st = st;
    w->prev_valid = true;
    portEXIT_CRITICAL(&s_bt_trace_mux);
}

static void bt_trace_report(void)
{
    portENTER_CRITICAL(&s_bt_trace_mux);
    const bt_trace_window_t w = s_bt_trace_window;
    s_bt_trace_window = (bt_trace_window_t){
        .min_cs = UINT16_MAX, .prev_st = w.prev_st, .prev_valid = w.prev_valid,
    };
    portEXIT_CRITICAL(&s_bt_trace_mux);
    if (w.n && (w.gate_flips >= 2u || w.trim_ticks)) {
        ESP_LOGW(TAG, "bt-trace: n=%lu ring=%u.%02u..%u.%02ums gate_flips=%lu trim=%lu sil=%lu window=ctrl_interval",
                 (unsigned long)w.n, (unsigned)(w.min_cs / 100u), (unsigned)(w.min_cs % 100u),
                 (unsigned)(w.max_cs / 100u), (unsigned)(w.max_cs % 100u),
                 (unsigned long)w.gate_flips, (unsigned long)w.trim_ticks, (unsigned long)w.sil_ticks);
    }
}

/* 0.81 diagnostic only: distinguish USB/input zeros from DMA/wire zeros.
 * Counts describe stereo 32-bit data at the USB input boundary (before the
 * volume/mute stage). No PCM/state changes, ISR work, buffer/limit changes. */
typedef struct {
    volatile uint32_t run, run_start_us, events, last_frames, last_start_us;
} uac_zero_observer_t;
static uac_zero_observer_t s_observe_usb;
static uint32_t s_observe_prev_us;
static volatile uint32_t s_observe_gap_max_us;
static volatile uint32_t s_observe_rx_bytes;
static volatile uint32_t s_observe_irq_core = UINT32_MAX;
static portMUX_TYPE s_observe_rx_mux = portMUX_INITIALIZER_UNLOCKED;

/* 0.140: expensive legacy observations are RAM-only and OFF at boot.
 * Bit0 is enabled; the other bits distinguish mode changes. The core0 proto
 * task is the sole writer. RX/pump stay core0; B6 dual tapes are independent. */
static volatile uint32_t s_legacy_diag_mode;
static TaskHandle_t s_fifo_monitor_task;

/* 0.131: first 16 large nonzero-to-nonzero input steps, before FU/DSP.
 * The pump owns scanning/previous-frame state. Serial publishes one pending
 * command; the pump applies it at a tick boundary. Short critical sections
 * protect publication/snapshots, never the scan or wire transmission.
 * Default disabled; no PCM writes, new task, ISR work or hot-path logging. */
static uac2_input_probe_event_t s_input_probe_events[UAC2_INPUT_PROBE_CAPACITY];
static uac2_input_probe_stats_t s_input_probe;
static portMUX_TYPE s_input_probe_mux = portMUX_INITIALIZER_UNLOCKED;
static volatile bool s_input_probe_pending;
static bool s_input_probe_request_arm;
static bool s_input_probe_active, s_input_probe_prev_valid;
static int32_t s_input_probe_prev_l, s_input_probe_prev_r;
static uint32_t s_input_probe_block_frames;
_Static_assert(sizeof(uac2_input_probe_event_t) == 32, "input probe event layout");

/* ★ 0.132 RX_POST_ENQUEUE 内容观察（第二份 tape，同一个 0xB6 ARM 代次）。
 *
 * 观察点：`uac_rx_observe_packet` 已经在 `delta == n_bytes_received` 时读回本包
 * 已入环字节（含环尾物理回绕）。本观察复用**同一个位置**，但覆盖**本包全部帧**
 * （不沿用上面那条 `pos < 64` 的 seq64 稀疏条件）——普通 500 Hz PCM 不做符号判定，
 * 因此不会因 `uac_input_symbol` 返回 -1 而被跳过。
 *
 * 所有权（工单68 §2 要求写清楚）：
 *   ARM      : 串口任务只置 pending（`uac2_audio_input_probe_request`，与 pre-FU 同）；
 *              pump 在成功 tick 上应用（`uac_input_probe_begin`），并**只发布**
 *              `s_rx_content_gen` / `s_rx_content_enabled` 两个 volatile。
 *   RX 扫描  : 全部在 RX 生产上下文（TinyUSB 接收回调）内；`s_rxc_*` 只有它读写。
 *              看到 gen 变化 ⇒ 在该上下文断开 prev、开新代次计数（不跨代次造事件）。
 *   DISARM   : pump 在 RX mux 内清 enabled；发布亦核 enabled/gen，OFF 后不再追加。
 *   getter   : 串口任务在 RX mux 内取一致快照；仅允许 OFF/无 pending 时读事件。
 * 代价：默认 OFF 时 `scan` 首行即返回，零逐帧开销；常驻静态约 0.8 KiB。 */
static uac2_rx_content_event_t s_rx_content_events[UAC2_RX_CONTENT_CAPACITY];
static portMUX_TYPE s_rx_content_mux = portMUX_INITIALIZER_UNLOCKED;
static volatile uint32_t s_rx_content_gen = UINT32_MAX;   /* pump 发布：本代次 arm_seq */
static volatile uint32_t s_rx_content_enabled;            /* pump 发布：1=扫描 */
static volatile uint32_t s_rx_content_requested, s_rx_content_applied, s_rx_content_arm_seq;
static volatile uint32_t s_rx_content_armed, s_rx_content_start_us, s_rx_content_stop_us;
static volatile uint32_t s_rx_content_last_rx_us;
static volatile uint32_t s_rx_content_packets, s_rx_content_frames, s_rx_content_gaps;
static volatile uint32_t s_rx_content_unsupported, s_rx_content_null_ff, s_rx_content_rebase;
static volatile uint32_t s_rx_content_delta_ne, s_rx_content_rejected;
static volatile uint32_t s_rx_content_events_n, s_rx_content_dropped, s_rx_content_stored;
/* RX 生产上下文私有（仅接收回调读写，不跨上下文共享） */
static uint32_t s_rxc_gen_seen = UINT32_MAX;
static bool s_rxc_prev_valid;
static int32_t s_rxc_prev_l, s_rxc_prev_r;
static uint32_t s_rxc_frame, s_rxc_packet;
_Static_assert(sizeof(uac2_rx_content_event_t) == 36, "rx content event layout");

bool uac2_audio_input_probe_request(bool arm)
{
    portENTER_CRITICAL(&s_input_probe_mux);
    if (s_input_probe_pending || (arm && s_input_probe.armed)) {
        portEXIT_CRITICAL(&s_input_probe_mux);
        return false;
    }
    s_input_probe.requested_seq++;
    portENTER_CRITICAL(&s_rx_content_mux);
    s_rx_content_requested = s_input_probe.requested_seq;
    portEXIT_CRITICAL(&s_rx_content_mux);
    s_input_probe_request_arm = arm;
    s_input_probe_pending = true;
    portEXIT_CRITICAL(&s_input_probe_mux);
    return true;
}

void uac2_audio_input_probe_get_stats(uac2_input_probe_stats_t *stats)
{
    portENTER_CRITICAL(&s_input_probe_mux);
    *stats = s_input_probe;
    portEXIT_CRITICAL(&s_input_probe_mux);
}

bool uac2_audio_input_probe_get_event(uint8_t index, uint32_t *arm_seq,
                                     uac2_input_probe_event_t *event)
{
    portENTER_CRITICAL(&s_input_probe_mux);
    const bool valid = !s_input_probe.armed && !s_input_probe_pending &&
                       index < s_input_probe.stored;
    if (valid) {
        *arm_seq = s_input_probe.arm_seq;
        *event = s_input_probe_events[index];
    }
    portEXIT_CRITICAL(&s_input_probe_mux);
    return valid;
}

static bool uac_input_probe_begin(uint32_t us)
{
    if (s_input_probe_pending) {
        portENTER_CRITICAL(&s_input_probe_mux);
        const uint32_t seq = s_input_probe.requested_seq;
        if (s_input_probe_request_arm) {
            /* Old entries are hidden by stored=0; do not clear 512 bytes with
             * interrupts masked. Only the pump resets its private history. */
            s_input_probe = (uac2_input_probe_stats_t){
                .requested_seq = seq, .applied_seq = seq, .arm_seq = seq,
                .armed = 1, .start_us = us,
            };
            s_input_probe_prev_valid = false;
            s_input_probe_active = true;
            /* Reset the small published snapshot, not the retained event array.
             * Lock order is input_probe -> rx_content; RX never takes input_probe.
             * Only RX owns its private previous-frame state and lazy generation. */
            portENTER_CRITICAL(&s_rx_content_mux);
            s_rx_content_requested = s_rx_content_applied = seq;
            s_rx_content_arm_seq = seq;
            s_rx_content_start_us = us;
            s_rx_content_stop_us = s_rx_content_last_rx_us = 0;
            s_rx_content_packets = s_rx_content_frames = s_rx_content_gaps = 0;
            s_rx_content_unsupported = s_rx_content_null_ff = 0;
            s_rx_content_rebase = s_rx_content_delta_ne = s_rx_content_rejected = 0;
            s_rx_content_events_n = s_rx_content_dropped = s_rx_content_stored = 0;
            s_rx_content_gen = seq;
            s_rx_content_armed = s_rx_content_enabled = 1;
            portEXIT_CRITICAL(&s_rx_content_mux);
        } else {
            s_input_probe.armed = 0;
            s_input_probe.applied_seq = seq;
            s_input_probe.stop_us = us;
            s_input_probe_active = false;
            s_input_probe_prev_valid = false;
            portENTER_CRITICAL(&s_rx_content_mux);
            s_rx_content_enabled = s_rx_content_armed = 0;
            s_rx_content_stop_us = us;
            s_rx_content_requested = s_rx_content_applied = seq;
            portEXIT_CRITICAL(&s_rx_content_mux);
        }
        s_input_probe_pending = false;
        portEXIT_CRITICAL(&s_input_probe_mux);
    }
    s_input_probe_block_frames = 0;
    return s_input_probe_active;
}

static void uac_input_probe_scan(const int32_t *pcm, size_t samples, uint32_t us)
{
    if (s_ctx.bit_depth != 32 || s_ctx.sample_rate != 96000) return;
    const uint32_t frames = (uint32_t)(samples / 2);
    for (uint32_t i = 0; i < frames; i++) {
        const int32_t l = pcm[2 * i], r = pcm[2 * i + 1];
        uint32_t flags = 0;
        if (s_input_probe_prev_valid) {
            const int64_t dl = (int64_t)l - s_input_probe_prev_l;
            const int64_t dr = (int64_t)r - s_input_probe_prev_r;
            if (l != 0 && s_input_probe_prev_l != 0 &&
                (dl > UAC2_INPUT_PROBE_THRESHOLD || dl < -(int64_t)UAC2_INPUT_PROBE_THRESHOLD)) flags |= 1;
            if (r != 0 && s_input_probe_prev_r != 0 &&
                (dr > UAC2_INPUT_PROBE_THRESHOLD || dr < -(int64_t)UAC2_INPUT_PROBE_THRESHOLD)) flags |= 2;
        }
        if (flags) {
            const uac2_input_probe_event_t event = {
                .us = us, .frame = s_input_probe.frames + i,
                .prev_l = s_input_probe_prev_l, .prev_r = s_input_probe_prev_r,
                .cur_l = l, .cur_r = r, .block = s_input_probe.blocks + 1,
                .flags = flags,
            };
            portENTER_CRITICAL(&s_input_probe_mux);
            s_input_probe.events++;
            if (s_input_probe.stored < UAC2_INPUT_PROBE_CAPACITY)
                s_input_probe_events[s_input_probe.stored++] = event;
            else s_input_probe.dropped++;
            portEXIT_CRITICAL(&s_input_probe_mux);
        }
        s_input_probe_prev_l = l;
        s_input_probe_prev_r = r;
        s_input_probe_prev_valid = true;
    }
    s_input_probe_block_frames = frames;
}

static void uac_input_probe_end(bool supported, uint32_t us)
{
    portENTER_CRITICAL(&s_input_probe_mux);
    s_input_probe.ticks++;
    if (s_input_probe_block_frames) {
        s_input_probe.blocks++;
        s_input_probe.frames += s_input_probe_block_frames;
        s_input_probe.last_pcm_us = us;
    } else {
        if (supported) s_input_probe.idle_ticks++;
        else s_input_probe.unsupported_ticks++;
        /* No observation across no-PCM/unsupported intervals. These counters
         * must be checked before treating an empty event tape as evidence. */
        s_input_probe_prev_valid = false;
    }
    portEXIT_CRITICAL(&s_input_probe_mux);
}

/* RX owns private history. Only bounded publication/snapshots take this lock.
 * OFF is a publication barrier: no event or counters can be appended after it.
 * The supported device runs the RX IRQ and pump on core0. The generation check
 * also prevents a stale packet from being published into a later ARM. */
enum { RX_GAP_UNSUPPORTED, RX_GAP_NULL, RX_GAP_REBASE, RX_GAP_DELTA, RX_GAP_REJECT };

static void uac_rx_content_gap(unsigned reason)
{
    s_rxc_prev_valid = false;
    portENTER_CRITICAL(&s_rx_content_mux);
    if (s_rx_content_enabled && s_rxc_gen_seen == s_rx_content_gen) {
        s_rx_content_gaps++;
        switch (reason) {
        case RX_GAP_UNSUPPORTED: s_rx_content_unsupported++; break;
        case RX_GAP_NULL: s_rx_content_null_ff++; break;
        case RX_GAP_REBASE: s_rx_content_rebase++; break;
        case RX_GAP_REJECT: s_rx_content_rejected++; break;
        default: s_rx_content_delta_ne++; break;
        }
    }
    portEXIT_CRITICAL(&s_rx_content_mux);
}

static bool uac_rx_content_begin_packet(void)
{
    if (!s_rx_content_enabled) return false;
    const uint32_t gen = s_rx_content_gen;
    if (gen != s_rxc_gen_seen) {
        s_rxc_gen_seen = gen;
        s_rxc_prev_valid = false;
        s_rxc_frame = s_rxc_packet = 0;
    }
    if (s_ctx.bit_depth != 32 || s_ctx.sample_rate != 96000) {
        uac_rx_content_gap(RX_GAP_UNSUPPORTED);
        return false;
    }
    return true;
}

static void uac_rx_content_scan_packet(const tu_fifo_t *ff, uint16_t wr,
                                       uint32_t n_bytes, uint32_t now_us)
{
    const uint32_t gen = s_rxc_gen_seen;
    const uint32_t depth = ff->depth;
    const uint8_t *buf = (const uint8_t *)ff->buffer;
    if (buf == NULL || depth == 0u) {
        uac_rx_content_gap(RX_GAP_NULL);
        return;
    }
    if (n_bytes == 0 || n_bytes > depth || depth % 8u || n_bytes % 8u) {
        uac_rx_content_gap(RX_GAP_UNSUPPORTED);
        return;
    }
    const uint32_t span = depth * 2u;
    uint32_t idx = ((uint32_t)wr + span - n_bytes) % span;
    const uint32_t frames = n_bytes / 8u;
    const uint32_t packet = ++s_rxc_packet;
    for (uint32_t i = 0; i < frames; i++) {
        uint32_t pos = idx >= depth ? idx - depth : idx;
        uint8_t b[8];
        for (unsigned k = 0; k < 8u; k++) {
            b[k] = buf[pos];
            if (++pos == depth) pos = 0;
        }
        int32_t l, r;
        memcpy(&l, b, 4);
        memcpy(&r, b + 4, 4);
        uint32_t flags = 0;
        if (s_rxc_prev_valid) {
            const int64_t dl = (int64_t)l - s_rxc_prev_l;
            const int64_t dr = (int64_t)r - s_rxc_prev_r;
            if (l && s_rxc_prev_l &&
                (dl > UAC2_RX_CONTENT_THRESHOLD || dl < -(int64_t)UAC2_RX_CONTENT_THRESHOLD)) flags |= 1;
            if (r && s_rxc_prev_r &&
                (dr > UAC2_RX_CONTENT_THRESHOLD || dr < -(int64_t)UAC2_RX_CONTENT_THRESHOLD)) flags |= 2;
        }
        if (flags) {
            const uac2_rx_content_event_t e = {
                .us = now_us, .rx_frame = s_rxc_frame,
                .prev_l = s_rxc_prev_l, .prev_r = s_rxc_prev_r, .cur_l = l, .cur_r = r,
                .packet = packet, .flags = flags, .rx_arm_seq = gen,
            };
            portENTER_CRITICAL(&s_rx_content_mux);
            if (s_rx_content_enabled && s_rx_content_gen == gen) {
                s_rx_content_events_n++;
                if (s_rx_content_stored < UAC2_RX_CONTENT_CAPACITY)
                    s_rx_content_events[s_rx_content_stored++] = e;
                else s_rx_content_dropped++;
            }
            portEXIT_CRITICAL(&s_rx_content_mux);
        }
        s_rxc_prev_l = l; s_rxc_prev_r = r;
        s_rxc_prev_valid = true;
        s_rxc_frame++;
        idx += 8;
        if (idx >= span) idx -= span;
    }
    portENTER_CRITICAL(&s_rx_content_mux);
    if (s_rx_content_enabled && s_rx_content_gen == gen) {
        s_rx_content_packets = packet;
        s_rx_content_frames = s_rxc_frame;
        s_rx_content_last_rx_us = now_us;
    }
    portEXIT_CRITICAL(&s_rx_content_mux);
}

void uac2_audio_rx_content_get_stats(uac2_rx_content_stats_t *s)
{
    portENTER_CRITICAL(&s_rx_content_mux);
    *s = (uac2_rx_content_stats_t){
        .requested_seq = s_rx_content_requested, .applied_seq = s_rx_content_applied,
        .arm_seq = s_rx_content_arm_seq, .armed = s_rx_content_armed,
        .start_us = s_rx_content_start_us, .stop_us = s_rx_content_stop_us,
        .last_rx_us = s_rx_content_last_rx_us, .packets = s_rx_content_packets,
        .frames = s_rx_content_frames, .gaps = s_rx_content_gaps,
        .unsupported = s_rx_content_unsupported, .null_ff = s_rx_content_null_ff,
        .rebase = s_rx_content_rebase, .delta_ne = s_rx_content_delta_ne,
        .rejected = s_rx_content_rejected, .events = s_rx_content_events_n,
        .dropped = s_rx_content_dropped, .stored = s_rx_content_stored,
    };
    portEXIT_CRITICAL(&s_rx_content_mux);
}

bool uac2_audio_rx_content_get_event(uint8_t index, uint32_t *arm_seq,
                                     uac2_rx_content_event_t *event)
{
    portENTER_CRITICAL(&s_rx_content_mux);
    const bool valid = !s_rx_content_enabled && !s_rx_content_armed &&
        s_rx_content_requested == s_rx_content_applied &&
        index < s_rx_content_stored &&
        s_rx_content_events[index].rx_arm_seq == s_rx_content_arm_seq;
    if (valid) {
        *arm_seq = s_rx_content_events[index].rx_arm_seq;
        *event = s_rx_content_events[index];
    }
    portEXIT_CRITICAL(&s_rx_content_mux);
    return valid;
}

/* ★ 0.104（DS-04B phase A 设计 → 本轮实现）**纯观测**：producer 侧每包 Δwr 账 +
 * 生产者锚点。用来判定"整包（6 帧 / 48 B）丢失"发生在**入环之前**还是**环读取路径**。
 *
 *   Δwr == n_bytes_received   ⇒ 本包确实整包写进 tu_fifo（eq）。
 *   Δwr == 0 && n > 0         ⇒ **整包被拒收**：dcd_dwc2.c:942 在"环剩余空间 < 本包
 *                                字节数"时把整包从 RxFIFO 读出丢弃，但 RX_COMPLETE 仍用
 *                                xfer->total_len 上报（dcd_dwc2.c:989）⇒ 应用层看到的
 *                                n_bytes_received 与 usb_pkts **一样正常**，只有 wr 游标
 *                                不动。这正是 0.103 没能区分的那条路径。
 *   其它                      ⇒ 口径不成立 ⇒ other，并让当前锚点窗作废。
 *   Δwr >= 2*depth            ⇒ 清环/改配置把 wr/rd 归零 ⇒ rebase，**绝不当丢包**。
 *
 * 生产者锚点：每 8192 接收帧取 64 符号（与 0.88 pump 锚点同相：8192 % 2048 == 0），
 * 8 槽队列；窗内**任一**包丢失/不一致 ⇒ 整窗作废（计 invalid，不入队）。
 * 只有 Δ==n 的包才读环内末端（本包刚追加区）。
 *
 * 边界：ISR 内只做整数运算 + 一次 16 位游标读 + 每帧 8 字节环内读；无浮点/格式化/
 * 日志、无新线程/队列/中断/新 SOF；不碰 rd_idx / remaining / buffer 内容；
 * **不使用 tu_fifo_peek_n**（tusb_fifo.c:471 会改 rd_idx）；
 * 不改音频行为/缓冲/limit/IRQ 级别/IDF/managed_components。
 * 原子性：计数累计单调、从不清零 ⇒ 单计数器读不撕裂；多计数器之间**不是**同一时刻快照。
 * 队列是**单生产者(ISR)/单消费者(ctrl 任务)** 环形队列：发布前先查余量，满则计 ovf
 * 且**不写槽** ⇒ 不会覆盖尚未被读走的槽，也不需要临界区。 */
#define UAC_RX_SEQ_FRAMES 8192u   /* 锚点窗：64 符号 / 8192 接收帧（≈5.86 窗/s） */
#define UAC_RX_SEQ_SLOTS  8u      /* 72 B/槽 ⇒ 576 B */
#define UAC_RX_DELTA_MAX  2048u   /* 单包能推进的字节上界（EP 最大包 104 B）；
                                   * 超过即视为清环/重配置的大跳 ⇒ rebase(unknown) */
#define UAC_RX_GAP_VALID_US 5000u /* 超过此间隔视为停流（不是一次"服务太晚"） */
#define UAC_ISR_GAP_ANOM_US 188u  /* 0.103 保留的标量：软件完成间隔偏长（**不等于**漏服务） */
typedef struct { uint32_t us, frame; uint8_t codes[64]; } uac_rx_seq_t;
static uac_rx_seq_t s_rx_seq[UAC_RX_SEQ_SLOTS];
static uac_rx_seq_t s_rx_pend;            /* 本窗锚点候选（窗末结算才入队） */
static uint32_t s_rx_frame;               /* 生产者累计接收帧（只计 Δ==n 的包） */
static uint16_t s_rx_prev_wr;             /* ISR 私有：上一包写游标 */
static bool s_rx_prev_valid;              /* ISR 私有：上一包游标有效 */
static bool s_rx_win_ok;                  /* ISR 私有：本窗迄今一致 */
static bool s_rx_pend_valid;              /* ISR 私有：本窗已有待结算锚点 */
static volatile uint32_t s_rx_head, s_rx_tail;
static volatile uint32_t s_rx_rec, s_rx_invalid, s_rx_ovf, s_rx_windows;
static volatile uint32_t s_rx_d_eq, s_rx_d_zero_n, s_rx_d_other, s_rx_rebase;
static volatile uint32_t s_rx_written, s_rx_reported, s_rx_drop_b;
static volatile uint32_t s_rx_first_zero_b, s_rx_first_zero_at_us;
static volatile uint32_t s_rx_other_first_n, s_rx_other_first_delta, s_rx_other_first_us;
static volatile uint32_t s_rx_irq_core = UINT32_MAX;
static volatile uint32_t s_rx_late_n;        /* 完成间隔 ≥2 微帧的次数 */
static volatile uint32_t s_rx_miss_expect;   /* 由长间隔**推算**的缺失包数（上界） */
static volatile uint32_t s_rx_stop_gap_n;    /* ≥5 ms 的间隔（停流/启动，不算丢失） */
static volatile int32_t s_rx_probe_l, s_rx_probe_r;   /* 窗口首帧原始样本（肉眼核对） */
static volatile uint32_t s_rx_probe_n;
static volatile uint32_t s_isr_gap_n;            /* 有效间隔样本数 */
static volatile uint32_t s_isr_gap_max_us;       /* 最大完成间隔 */
static volatile uint32_t s_isr_gap_max_at_us;    /* 最大间隔出现时刻（esp_timer µs）*/
static volatile uint32_t s_isr_gap_first_us;     /* 首个异常间隔的值 */
static volatile uint32_t s_isr_gap_first_at_us;  /* 首个异常出现时刻 */
static volatile uint32_t s_isr_gap_anom;         /* 间隔 ≥ 阈值次数 */
static volatile uint32_t s_isr_prev_us;          /* ISR 私有：上次完成时刻 */

/* ★ 0.106 只读 DWC2 状态探针 —— 判定"缺失的包是设备侧丢的还是主机没送"的**设备侧**证据。
 *
 * 地址来自 IDF 头（components/soc/esp32s31/register/soc/usb_otghs_reg.h）:
 *   USB_OTGHS_GINTSTS_REG = DR_REG_USB_OTGHS_BASE + 0x14  (reg_base.h:8 = 0x20300000)
 *   USB_OTGHS_DSTS_REG    = DR_REG_USB_OTGHS_BASE + 0x808
 * 与 TinyUSB 的 `DWC2_HS_REG_BASE 0x20300000`（dwc2_esp32.h:84, ESP32S31 分支）一致。
 *
 *   DSTS.SOFFN(21:8, 14 bit) = 微帧号（HS 每 125 µs 加一）⇒ 相邻两次完成之间的
 *     ΔSOFFN 就是**设备时间轴上经过的微帧数**；Δ-1 = 这一段时间里"没有任何完成"的
 *     微帧数 = 缺失包数（与锚点测得的源删除量互相独立、可直接对照）。
 *   GINTSTS bit21（device 模式 = INCOMPISOOUT，IDF 里同一位名为 INCOMPLP）=
 *     "本微帧内有 ISO OUT 端点的传输没有完成"；DCD 的 gintmsk 不含这位 ⇒ 置位后
 *     不会被清（粘滞）⇒ 只读即可回答"设备侧是否出现过 ISO OUT 未完成"。
 *   ⚠ 全程**只读**：不写 W1C、不清中断、不改 DCD/重装逻辑。
 *   自检：CMODE 必须=0(device)、ENUMSPD 必须=0(HS)、SOFFN 必须按 ≈8000/s 前进；
 *        不满足则本探针结论标 invalid（不许拿坏数据当证据）。 */
#define UAC_DWC2_GINTSTS_REG        0x20300014u
#define UAC_DWC2_DSTS_REG           0x20300808u
#define UAC_DWC2_GINTSTS_CMODE          (1u << 0)
#define UAC_DWC2_GINTSTS_OEPINT         (1u << 19)
#define UAC_DWC2_GINTSTS_IEPINT         (1u << 18)
#define UAC_DWC2_GINTSTS_RXFLVL         (1u << 4)
#define UAC_DWC2_GINTSTS_INCOMPISOIN    (1u << 20)
#define UAC_DWC2_GINTSTS_INCOMPISOOUT   (1u << 21)
#define UAC_DWC2_DSTS_SOFFN_MASK     USB_OTGHS_DSTS_SOFFN_M
#define UAC_DWC2_DSTS_SOFFN_VALUE    USB_OTGHS_DSTS_SOFFN_V
#define UAC_DWC2_DSTS_SOFFN_SHIFT    USB_OTGHS_DSTS_SOFFN_S
_Static_assert(UAC_DWC2_DSTS_SOFFN_MASK == 0x003fff00u,
               "DSTS SOFFN must retain all 14 microframe bits");
#define UAC_DWC2_DSTS_ENUMSPD_MASK   0x6u
#define UAC_DWC2_SOFFN_MAX_DF        64u   /* ΔSOFFN 合理性上界（>此值视为停流/不同步） */

static inline uint32_t uac_dwc2_rd(uint32_t addr)
{
    return *(volatile uint32_t *)(uintptr_t)addr;
}

static uint32_t s_soffn_prev;
static bool s_soffn_prev_valid;
static volatile uint32_t s_soffn_n, s_soffn_df_sum, s_soffn_zero_n, s_soffn_bad;
static volatile uint32_t s_soffn_max_df;
static volatile uint32_t s_soffn_first_miss_at_us;
static volatile uint32_t s_dwc2_gintsts_first, s_dwc2_gintsts_first_at_us;
static volatile uint32_t s_dwc2_dsts_last, s_dwc2_gintsts_iso_seen;

/* ★ 0.108 端点级只读探针：EP1 OUT 的 DOEPINT（地址 = DR_REG_USB_OTGHS_BASE + 0xb28，
 * 即 IDF 头的 `USB_OTGHS_DOEPINT1_REG`；基址 0x20300000 同 GW 探针）。
 *   bit11 `PKTDRPSTS` —— IDF 原文："This bit indicates to the application that an
 *   ISOC OUT packet has been dropped. This bit does not have an associated mask bit and
 *   does not generate an interrupt."
 *   ⇒ 这是**设备侧确实丢掉了一个 ISO OUT 包**的粘滞标志，而且：
 *     · 没有对应的 mask 位（不产生中断）；
 *     · DCD 的 `doepmsk` 只有 `STUPM|XFRCM`（dcd_dwc2.c:763），且它只写回
 *       XFRC / SETUP|STPKTRX / STSPHSRX / EPDISD ⇒ **该位不会被 DCD 清掉**。
 *   bit8 `OUTPKTERR` = OUT 包错误（同样不参与 DCD 的清除路径）。
 *   全程**只读**：不写 W1C、不清任何位。 */
#define UAC_DWC2_DOEPINT1_REG        0x20300b28u
#define UAC_DWC2_DOEPINT_PKTDRPSTS   (1u << 11)
#define UAC_DWC2_DOEPINT_OUTPKTERR   (1u << 8)
#define UAC_DWC2_DOEPINT_XFRC        (1u << 0)
static volatile uint32_t s_ep1_doepint_first, s_ep1_doepint_first_at_us;
static volatile uint32_t s_ep1_doepint_last, s_ep1_pktdrp_seen, s_ep1_outpkterr_seen;

/* Temporary core0 state trace. The pump only stores changes in RAM; the
 * control task prints them later. No serial output in the audio hot path.
 * flags: bit0 BT source, 1 switching, 2 BT gate, 3 codec mute, 4..5 phase,
 * 6 UAC hold, 7 UAC stream, 8 have_audio, 9 BT connected, 10 BT streaming,
 * 11 codec_busy, 12 decoder_ready. why: 0 sample, 1 BT request, 2 UAC
 * request, 3 switch step, 4 watchdog fallback. Times are esp_timer us. */
typedef struct { uint32_t us, flags, rate, why; } uac_state_trace_t;
static uac_state_trace_t s_state_trace[64];
static uint32_t s_state_head, s_state_tail, s_state_dropped;
static uint32_t s_state_prev_flags = UINT32_MAX;
static portMUX_TYPE s_state_mux = portMUX_INITIALIZER_UNLOCKED;

static void uac_trace_state(uint32_t why)
{
    portENTER_CRITICAL(&s_state_mux);
    const uint32_t flags = (s_ctx.active_source == AUDIO_SOURCE_BT ? 1u : 0u) |
        ((uint32_t)s_ctx.source_switching << 1) |
        ((uint32_t)s_ctx.bt_gate_open << 2) |
        /* trace bit 3 reserved: initial BT CFG no longer mutes UAC. */
        ((uint32_t)s_ctx.switch_phase << 4) |
        ((uint32_t)s_ctx.uac_resume_hold << 6) |
        ((uint32_t)s_ctx.streaming << 7) |
        ((uint32_t)s_ctx.have_audio << 8) |
        ((uint32_t)a2dp_audio_is_connected() << 9) |
        ((uint32_t)a2dp_audio_is_streaming() << 10) |
        ((uint32_t)a2dp_audio_is_codec_busy() << 11) |
        ((uint32_t)a2dp_audio_is_decoder_ready() << 12);
    if (why != 0 || flags != s_state_prev_flags) {
        s_state_prev_flags = flags;
        if (s_state_head - s_state_tail < 64) {
            s_state_trace[s_state_head % 64] = (uac_state_trace_t){
                .us = (uint32_t)esp_timer_get_time(), .flags = flags,
                .rate = s_ctx.output_rate, .why = why};
            s_state_head++;
        } else {
            s_state_dropped++;
        }
    }
    portEXIT_CRITICAL(&s_state_mux);
}

static void uac_print_state_trace(void)
{
    for (unsigned i = 0; i < 64; i++) {
        uac_state_trace_t item;
        portENTER_CRITICAL(&s_state_mux);
        const bool available = s_state_tail != s_state_head;
        if (available) item = s_state_trace[s_state_tail++ % 64];
        portEXIT_CRITICAL(&s_state_mux);
        if (!available) break;
        ESP_LOGI(TAG, "state-trace: us=%lu flags=%lu rate=%lu why=%lu",
                 (unsigned long)item.us, (unsigned long)item.flags,
                 (unsigned long)item.rate, (unsigned long)item.why);
    }
    ESP_LOGI(TAG, "state-trace-summary: dropped=%lu", (unsigned long)s_state_dropped);
}

static void uac_observe_zeros(uac_zero_observer_t *o, const void *data,
                              size_t bytes, uint32_t now_us)
{
    const int32_t *p = data;
    for (size_t i = 0; i + 1 < bytes / sizeof(*p); i += 2) {
        if (p[i] == 0 && p[i + 1] == 0) {
            if (o->run == 0) o->run_start_us = now_us;
            o->run++;
        } else {
            if (o->run >= 3) {
                o->last_frames = o->run;
                o->last_start_us = o->run_start_us;
                o->events++;
            }
            o->run = 0;
        }
    }
}

/* 0.88 diagnostic: sample 64 raw USB stereo symbols every 2048 frames,
 * before volume/DSP. The optional test WAV uses odd multiples of 2^26.
 * Invalid anchors (ordinary audio) are not printed. No PCM edits or ISR work;
 * RAM records are emitted by the control task. Cumulative frame indices let
 * an offline golden-sequence comparison locate deletion before the DSP. */
typedef struct { uint32_t us, frame; uint8_t codes[64]; } uac_input_tap_t;
static uac_input_tap_t s_input_tap[32], s_input_anchor;
static uint32_t s_input_frame, s_input_head, s_input_tail, s_input_dropped;
static bool s_input_anchor_valid;
static portMUX_TYPE s_input_mux = portMUX_INITIALIZER_UNLOCKED;

static int uac_input_symbol(int32_t value)
{
    if (((uint32_t)value & 0x03ffffffu) != 0) return -1;
    const int n = value / 67108864;
    if (n < -7 || n > 7 || n % 2 == 0) return -1;
    return (n + 7) / 2;
}

static void uac_observe_input_sequence(const int32_t *pcm, size_t bytes, uint32_t us)
{
    const size_t frames = bytes / 8;
    for (size_t i = 0; i < frames; i++, s_input_frame++) {
        const uint32_t pos = s_input_frame % 2048;
        if (pos >= 64) continue;
        if (pos == 0) {
            s_input_anchor.us = us;
            s_input_anchor.frame = s_input_frame;
            s_input_anchor_valid = true;
        }
        const int l = uac_input_symbol(pcm[2 * i]);
        const int r = uac_input_symbol(pcm[2 * i + 1]);
        if (l < 0 || r < 0) s_input_anchor_valid = false;
        s_input_anchor.codes[pos] = (uint8_t)(l + 8 * r);
        if (pos == 63 && s_input_anchor_valid) {
            portENTER_CRITICAL(&s_input_mux);
            if (s_input_head - s_input_tail < 32) {
                s_input_tap[s_input_head++ % 32] = s_input_anchor;
            } else {
                s_input_dropped++;
            }
            portEXIT_CRITICAL(&s_input_mux);
        }
    }
}

static void uac_print_input_sequence(void)
{
    static const char hex[] = "0123456789abcdef";
    for (unsigned i = 0; i < 32; i++) {
        uac_input_tap_t item;
        portENTER_CRITICAL(&s_input_mux);
        const uint32_t mode = s_legacy_diag_mode;
        const bool available = (mode & 1u) && s_input_tail != s_input_head;
        if (available) item = s_input_tap[s_input_tail++ % 32];
        portEXIT_CRITICAL(&s_input_mux);
        if (!(mode & 1u) || !available) break;
        char codes[129];
        for (unsigned j = 0; j < 64; j++) {
            codes[2 * j] = hex[item.codes[j] >> 4];
            codes[2 * j + 1] = hex[item.codes[j] & 15];
        }
        codes[128] = 0;
        ESP_LOGI(TAG, "input-seq: us=%lu frame=%lu codes=%s mode=%lu",
                 (unsigned long)item.us, (unsigned long)item.frame, codes,
                 (unsigned long)mode);
    }
    ESP_LOGI(TAG, "input-seq-summary: dropped=%lu", (unsigned long)s_input_dropped);
}

/* ---------------- 0.104 producer 侧观测（ISR 内调用；边界见文件上方 s_rx_* 注释） ---------------- */

/* 结算上一窗：只有"窗内每个包都 Δ==n"且"64 个符号全有效"才入队 */
static void uac_rx_seq_close_window(void)
{
    if (!s_rx_pend_valid) {
        return;
    }
    s_rx_windows++;
    if (s_rx_win_ok) {
        if (s_rx_head - s_rx_tail < UAC_RX_SEQ_SLOTS) {
            s_rx_seq[s_rx_head % UAC_RX_SEQ_SLOTS] = s_rx_pend;
            s_rx_head++;
            s_rx_rec++;
        } else {
            s_rx_ovf++;          /* 满：不写槽（绝不覆盖未读走的记录） */
        }
    } else {
        s_rx_invalid++;
    }
    s_rx_pend_valid = false;
}

/* 本包已确认整包入环（Δ==n）⇒ 读环内末端 n 字节，维护生产者锚点。
 * 环索引空间 [0, 2*depth)，物理偏移 = idx>=depth ? idx-depth : idx（tusb_fifo.c 口径）。 */
static void uac_rx_observe_packet(const tu_fifo_t *ff, uint16_t wr, uint32_t n_bytes,
                                  uint32_t now_us)
{
    const uint32_t depth = (uint32_t)ff->depth;
    const uint8_t *buf = (const uint8_t *)ff->buffer;
    if (buf == NULL || depth == 0u || (depth % 8u) != 0u || (n_bytes % 8u) != 0u) {
        s_rx_win_ok = false;     /* 口径不成立（帧不对齐）⇒ 本窗作废，不当丢包 */
        return;
    }
    const uint32_t span = depth * 2u;
    uint32_t idx = ((uint32_t)wr + span - (n_bytes % span)) % span;   /* 本包起点 */
    const uint32_t frames = n_bytes / 8u;
    for (uint32_t i = 0; i < frames; i++, s_rx_frame++) {
        const uint32_t pos = s_rx_frame % UAC_RX_SEQ_FRAMES;
        if (pos == 0u) {
            uac_rx_seq_close_window();      /* 上一窗到边界 ⇒ 结算 */
            s_rx_pend.us = now_us;
            s_rx_pend.frame = s_rx_frame;
            s_rx_pend_valid = true;
            s_rx_win_ok = true;
        }
        if (pos < 64u) {
            uint32_t p = (idx >= depth) ? (idx - depth) : idx;
            uint8_t b[8];
            for (unsigned k = 0; k < 8u; k++) {
                b[k] = buf[p];
                if (++p == depth) {
                    p = 0u;                 /* 包横跨环尾：按物理偏移回绕 */
                }
            }
            int32_t l, r;
            memcpy(&l, b, 4);
            memcpy(&r, b + 4, 4);
            if (pos == 0u) {                 /* 窗口首帧原始样本：供报告里肉眼核对 */
                s_rx_probe_l = l;
                s_rx_probe_r = r;
                s_rx_probe_n++;
            }
            const int ls = uac_input_symbol(l);
            const int rs = uac_input_symbol(r);
            if (ls < 0 || rs < 0) {
                s_rx_win_ok = false;        /* 普通 PCM（非标记源）⇒ 本窗无效 */
            } else {
                s_rx_pend.codes[pos] = (uint8_t)(ls + 8 * rs);
            }
        }
        idx += 8u;
        if (idx >= span) {
            idx -= span;
        }
    }
}

/* ctrl 任务侧打印（非 ISR）：逐条 `usb-rx-seq:` + 一条 summary */
static void uac_print_rx_sequence(void)
{
    static const char hex[] = "0123456789abcdef";
    for (unsigned i = 0; i < UAC_RX_SEQ_SLOTS; i++) {
        uac_rx_seq_t item;
        /* Mode changes discard old diagnostic records on this same core.
         * Copy with IRQs masked so neither that reset nor RX can race it. */
        portENTER_CRITICAL(&s_input_mux);
        const uint32_t mode = s_legacy_diag_mode;
        const bool available = (mode & 1u) && s_rx_tail != s_rx_head;
        if (available) item = s_rx_seq[s_rx_tail++ % UAC_RX_SEQ_SLOTS];
        portEXIT_CRITICAL(&s_input_mux);
        if (!available) break;
        char codes[129];
        for (unsigned j = 0; j < 64; j++) {
            codes[2 * j] = hex[item.codes[j] >> 4];
            codes[2 * j + 1] = hex[item.codes[j] & 15];
        }
        codes[128] = 0;
        ESP_LOGI(TAG, "usb-rx-seq: us=%lu frame=%lu codes=%s mode=%lu",
                 (unsigned long)item.us, (unsigned long)item.frame, codes,
                 (unsigned long)mode);
    }
    ESP_LOGI(TAG, "usb-rx-seq-summary: rec=%lu invalid=%lu ovf=%lu windows=%lu d_eq=%lu d_zero_n=%lu d_other=%lu rebase=%lu frames=%lu written=%lu reported=%lu drop_b=%lu zero_b=%lu zero_at=%lu other_n=%lu other_d=%lu other_at=%lu late_n=%lu miss_exp=%lu stop_gap=%lu probe_n=%lu probe_l=%ld probe_r=%ld irq_core=%lu",
             (unsigned long)s_rx_rec, (unsigned long)s_rx_invalid, (unsigned long)s_rx_ovf,
             (unsigned long)s_rx_windows, (unsigned long)s_rx_d_eq,
             (unsigned long)s_rx_d_zero_n, (unsigned long)s_rx_d_other,
             (unsigned long)s_rx_rebase, (unsigned long)s_rx_frame,
             (unsigned long)s_rx_written, (unsigned long)s_rx_reported,
             (unsigned long)s_rx_drop_b, (unsigned long)s_rx_first_zero_b,
             (unsigned long)s_rx_first_zero_at_us, (unsigned long)s_rx_other_first_n,
             (unsigned long)s_rx_other_first_delta, (unsigned long)s_rx_other_first_us,
             (unsigned long)s_rx_late_n, (unsigned long)s_rx_miss_expect,
             (unsigned long)s_rx_stop_gap_n, (unsigned long)s_rx_probe_n,
             (long)s_rx_probe_l, (long)s_rx_probe_r,
             (unsigned long)s_rx_irq_core);
}

void uac2_audio_set_legacy_diagnostics(bool enabled)
{
    /* core0 only: proto priority5, pump24 and USB RX IRQ cannot be executing
     * concurrently inside this short metadata section. No FIFO/PCM changes. */
    portENTER_CRITICAL(&s_input_mux);
    if (((s_legacy_diag_mode & 1u) != 0) != enabled) {
        s_input_tail = s_input_head;
        s_rx_tail = s_rx_head;
        s_input_frame = s_rx_frame = 0; /* observed-frame indices, per mode */
        s_input_anchor_valid = s_rx_pend_valid = s_rx_win_ok = false;
        s_observe_usb.run = 0;
        s_observe_prev_us = s_isr_prev_us = 0;
        s_soffn_prev_valid = false;
        s_legacy_diag_mode++;
    }
    portEXIT_CRITICAL(&s_input_mux);
    if (s_fifo_monitor_task != NULL) xTaskNotifyGive(s_fifo_monitor_task);
}

uint32_t uac2_audio_get_legacy_diagnostics(void)
{
    return s_legacy_diag_mode;
}

/* ---------------- Descriptors ---------------- */

static const uint8_t s_desc_configuration[] = {
    TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, 0, UAC2_CONFIG_DESC_TOTAL_LEN, 0x00, 100),
    UAC2_SPEAKER_STEREO_2_FORMAT_DESCRIPTOR(ITF_NUM_AUDIO_CONTROL, 0,
                                            UAC_EP_OUT, UAC_EP_OUT_MAX_PACKET,
                                            UAC_EP_FB, UAC_FB_EP_SIZE),
};

TU_VERIFY_STATIC(sizeof(s_desc_configuration) == UAC2_CONFIG_DESC_TOTAL_LEN,
                 "Configuration descriptor size mismatch");

static const tusb_desc_device_t s_desc_device = {
    .bLength = sizeof(tusb_desc_device_t),
    .bDescriptorType = TUSB_DESC_DEVICE,
    .bcdUSB = 0x0200,
    .bDeviceClass = TUSB_CLASS_MISC,
    .bDeviceSubClass = MISC_SUBCLASS_COMMON,
    .bDeviceProtocol = MISC_PROTOCOL_IAD,
    .bMaxPacketSize0 = CFG_TUD_ENDPOINT0_SIZE,
    .idVendor = 0x303A,
    .idProduct = 0x4003,
    .bcdDevice = 0x0100,
    .iManufacturer = 0x01,
    .iProduct = 0x02,
    .iSerialNumber = 0x03,
    .bNumConfigurations = 0x01,
};

static const tusb_desc_device_qualifier_t s_desc_qualifier = {
    .bLength = sizeof(tusb_desc_device_qualifier_t),
    .bDescriptorType = TUSB_DESC_DEVICE_QUALIFIER,
    .bcdUSB = 0x0200,
    .bDeviceClass = TUSB_CLASS_MISC,
    .bDeviceSubClass = MISC_SUBCLASS_COMMON,
    .bDeviceProtocol = MISC_PROTOCOL_IAD,
    .bMaxPacketSize0 = CFG_TUD_ENDPOINT0_SIZE,
    .bNumConfigurations = 0x01,
    .bReserved = 0x00,
};

enum {
    STRID_LANGID = 0,
    STRID_MANUFACTURER,
    STRID_PRODUCT,
    STRID_SERIAL,
};

static const char *const s_string_desc[] = {
    (const char[]){0x09, 0x04},
    "Espressif",
    "S31 UAC2 Async Bridge",
    "S31UAC2A0001",
};

static uint16_t s_desc_str[32 + 1];

/* ---------------- FIFO monitor ----------------
 * Feedback is calculated by the TinyUSB audio driver itself
 * (AUDIO_FEEDBACK_METHOD_FIFO_COUNT). This task only samples the FIFO
 * water level for diagnostics; it must not write the feedback value. */

static void uac_fifo_monitor_task(void *arg)
{
    (void)arg;

    for (;;) {
        if (!(s_legacy_diag_mode & 1u)) {
            ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
            continue;
        }
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(1));

        bool active = (s_legacy_diag_mode & 1u) && s_ctx.streaming && tud_audio_mounted();
        if (!active) {
            continue;
        }

        tu_fifo_t *ff = tud_audio_get_ep_out_ff();
        if (ff == NULL) {
            continue;
        }

        uint16_t level = tu_fifo_count(ff);
        if (level < s_ctx.fifo_min) {
            s_ctx.fifo_min = level;
        }
        if (level > s_ctx.fifo_max) {
            s_ctx.fifo_max = level;
        }
    }
}

/* Retain the previous 28 ms BT fallback commit interval. In the 0.92
 * failing capture, profile-close arrived 123 ms after suspend, after the
 * 100 ms watchdog request. This interval is now digital mute, not a fade. */
#define BT_FALLBACK_COMMIT_MS 28

/* 判"本 tick 是否处于放音态"的时间窗：`streaming`/`have_audio` 在播放器退出后仍为 1，
 * 所以只有"最近 AUDIBLE_GRACE_US 之内还真的拿到过数据"才算放音态。
 * 200 ms 的依据：@96k 是 80 个块；起流瞬态与真实 FIFO 干涸欠载都在这个尺度内恢复，
 * 而"主机已经停止送数据"会稳定超出它。 */
#define AUDIBLE_GRACE_US  (200 * 1000)
/* ---------------- Audio pump task ---------------- */

static size_t uac_convert_24_to_32(uint8_t *dst, const uint8_t *src, size_t len)
{
    size_t frames = len / 6; /* 2 channels * 3 bytes */
    for (size_t i = 0; i < frames; i++) {
        const uint8_t *in = src + i * 6;
        uint8_t *out = dst + i * 8;
        for (int ch = 0; ch < 2; ch++) {
            uint32_t v = (uint32_t)in[0] | ((uint32_t)in[1] << 8) | ((uint32_t)in[2] << 16);
            int32_t s = (int32_t)(v & 0x00FFFFFFu);
            if (v & 0x00800000u) {
                s |= ~0x00FFFFFF; /* sign-extend 24-bit */
            }
            s <<= 8; /* left-justify into the 32-bit I2S slot */
            memcpy(out, &s, sizeof(s));
            in += 3;
            out += 4;
        }
    }
    return frames * 8;
}

static inline uint32_t uac_abs_sample(int32_t s)
{
    return (s < 0) ? (uint32_t)(-(int64_t)s) : (uint32_t)s;
}

static float uac_peak_db_f(uint32_t peak)
{
    if (peak == 0) {
        return -999.0f;
    }
    return 20.0f * log10f((float)peak / 2147483647.0f);
}

static void uac_update_peaks(const int32_t *p, size_t samples,
                             volatile uint32_t *peak_l, volatile uint32_t *peak_r)
{
    for (size_t i = 0; i + 1 < samples; i += 2) {
        uint32_t al = uac_abs_sample(p[i]);
        uint32_t ar = uac_abs_sample(p[i + 1]);
        if (al > *peak_l) *peak_l = al;
        if (ar > *peak_r) *peak_r = ar;
    }
}

/* 0.134 core0 input queue boundary. No second audio buffer or PCM edits.
 * tud (prio5), ctrl (prio6) and pump (prio24) are pinned to core0. Only the
 * pump consumes the OUT FIFO; the USB task publishes its remaining old-byte
 * boundary before deciding whether TinyUSB may clear it. Thus a callback
 * cannot interleave a pump read and its boundary accounting. The short mux
 * protects only this metadata/diagnostic snapshot, never a FIFO mutex/copy.
 * Genuine format/source/reset boundaries invalidate retained old-format data.
 */
extern const uint32_t tud_audio_rx_preserve_fifo_abi;
extern const uint32_t tud_audio_fifo_feedback_q15_abi;
typedef struct {
    uint32_t tail_b, rate, bits, wait_us, tail_wr;
    bool playing, blocked;
    volatile bool closed_tail;
    uint32_t closes, kept_b, drained_b, dropped_b, short_b, reset_b;
    uint32_t short_kept_b;  /* Sub-block prefill retained on close, subset of kept_b. */
    /* short_b stays zero (old discard counter); timeouts counts short starts. */
    uint32_t starts, timeouts, boundary_err, submit_drop_b;
    uint32_t close_us, close_b;
} uac_buffer_t;
static uac_buffer_t s_uac_buffer;
static portMUX_TYPE s_uac_buffer_mux = portMUX_INITIALIZER_UNLOCKED;

static uint16_t uac_buffer_start_bytes(unsigned bits)
{
    uint16_t block = DSP_PCM_SLOT_FRAMES * (bits == 24 ? 6u : 8u);
    return block > UAC_RESUME_LEVEL_BYTES ? block : UAC_RESUME_LEVEL_BYTES;
}

/* Caller holds s_uac_buffer_mux. No FIFO locks or copies under this mux. */
static void uac_buffer_reset_locked(bool blocked)
{
    s_uac_buffer.dropped_b += s_uac_buffer.tail_b;
    s_uac_buffer.tail_b = 0;
    s_uac_buffer.playing = false;
    s_uac_buffer.blocked = blocked;
    s_uac_buffer.closed_tail = false;
    s_uac_buffer.wait_us = 0;
}

static void uac_buffer_reset(bool blocked)
{
    portENTER_CRITICAL(&s_uac_buffer_mux);
    uac_buffer_reset_locked(blocked);
    portEXIT_CRITICAL(&s_uac_buffer_mux);
}

/* SET_CUR and pre-open are serialized by the USB task. Invalidate old-rate
 * bytes at that input boundary, before notifying ctrl; never clear a new FIFO
 * prefix after a potentially blocking core1 ACK. */
static void uac_request_rate(uint32_t rate)
{
    portENTER_CRITICAL(&s_uac_buffer_mux);
    const bool changed = rate != s_ctx.requested_rate;
    s_ctx.format_input_busy = true;
    s_ctx.requested_rate = rate;
    s_ctx.format_request_seq++;
    if (changed) {
        uac_buffer_reset_locked(true);
        s_ctx.have_audio = false;
        if (s_ctx.stream_alt == 0) s_ctx.fifo_rearm_clear = true;
    } else if (rate != s_ctx.sample_rate) {
        s_uac_buffer.blocked = true;
    }
    portEXIT_CRITICAL(&s_uac_buffer_mux);
    if (changed && tud_audio_mounted()) tud_audio_clear_ep_out_ff();
    portENTER_CRITICAL(&s_uac_buffer_mux);
    s_ctx.format_input_busy = false;
    portEXIT_CRITICAL(&s_uac_buffer_mux);
    if (s_ctx.ctrl_task != NULL) xTaskNotify(s_ctx.ctrl_task, EVT_RATE, eSetBits);
}

/* SET_CONFIGURATION/unmount starts an alt0 session. A late ACK from the old
 * session may update the applied clock, but cannot acknowledge this new one. */
static void uac_usb_session_reset(void)
{
    portENTER_CRITICAL(&s_uac_buffer_mux);
    s_ctx.stream_alt = 0;
    s_ctx.bus_suspended = false;
    s_ctx.format_input_busy = false;
    s_ctx.fifo_rearm_clear = false;
    s_ctx.streaming = false;
    s_ctx.have_audio = false;
    s_ctx.requested_rate = s_ctx.sample_rate;
    s_ctx.requested_bits = s_ctx.bit_depth;
    s_ctx.format_request_seq++;
    s_ctx.format_done_seq = s_ctx.format_request_seq;
    uac_buffer_reset_locked(false);
    portEXIT_CRITICAL(&s_uac_buffer_mux);
}

/* Strong override of the project-managed TinyUSB default (clear). Called
 * before its OUT FIFO clear, with RX endpoint's OLD alternate setting. */
bool tud_audio_rx_keep_fifo_cb(uint8_t rhport, tusb_control_request_t const *req,
                               tu_fifo_t *ff, uint8_t old_alt)
{
    (void)rhport;
    const uint8_t itf = tu_u16_low(tu_le16toh(req->wIndex));
    const uint8_t alt = tu_u16_low(tu_le16toh(req->wValue));
    if (itf != ITF_NUM_AUDIO_STREAMING) return false;
    const unsigned bits = old_alt == 1 ? 24u : 32u;
    const unsigned stride = bits == 24 ? 6u : 8u;
    portENTER_CRITICAL(&s_uac_buffer_mux);
    const bool same = (old_alt == 1 || old_alt == 2) &&
        (alt == 0 || alt == old_alt) && bits == (unsigned)s_ctx.bit_depth &&
        s_ctx.requested_rate == s_ctx.sample_rate &&
        s_ctx.sample_rate == i2s_out_get_rate() &&
        s_ctx.active_source == AUDIO_SOURCE_UAC && !s_ctx.source_switching &&
        s_ctx.switch_phase != SWITCH_SETTLING && s_ctx.switch_phase != SWITCH_MUTED;
    s_ctx.stream_alt = 0;
    s_ctx.streaming = false;
    s_ctx.have_audio = false;
    const uint16_t count = tu_fifo_count(ff);
    const uint32_t old_tail = s_uac_buffer.tail_b;
    const bool short_prefill = !s_uac_buffer.playing && !old_tail &&
        count < uac_buffer_start_bytes(bits);
    /* Keep even a sub-block prefill or an empty boundary: a last ISO packet
     * can still complete after this snapshot, before the next OUT arm. */
    const bool keep = same && !s_uac_buffer.blocked &&
        !s_ctx.fifo_rearm_clear && count % stride == 0;
    s_uac_buffer.closes++;
    s_uac_buffer.close_us = (uint32_t)esp_timer_get_time();
    s_uac_buffer.close_b = count;
    s_uac_buffer.playing = false;  /* New stream always needs its own prefill. */
    s_uac_buffer.wait_us = 0;
    if (old_tail > count) s_uac_buffer.boundary_err++;
    if (keep && old_tail <= count) {
        /* Count only the extension, not old bytes already retained. */
        s_uac_buffer.kept_b += count - old_tail;
        if (short_prefill) s_uac_buffer.short_kept_b += count;
        s_uac_buffer.tail_b = count;
        s_uac_buffer.rate = s_ctx.sample_rate;
        s_uac_buffer.bits = bits;
        s_uac_buffer.tail_wr = ff->wr_idx;
        s_uac_buffer.closed_tail = true;
    } else {
        s_ctx.fifo_rearm_clear = true;
        s_uac_buffer.closed_tail = false;
        s_uac_buffer.dropped_b += old_tail;
        s_uac_buffer.tail_b = 0;
        s_uac_buffer.reset_b += count;
    }
    portEXIT_CRITICAL(&s_uac_buffer_mux);
    return keep && old_tail <= count;
}

/* Invoked after ISO activation (old transfer disabled), before arming RX.
 * Do not interpret a retained 24-bit/rate tail as a different new format. */
void tud_audio_rx_open_fifo_cb(uint8_t rhport, tusb_control_request_t const *req,
                               tu_fifo_t *ff)
{
    (void)rhport;
    const unsigned alt = tu_u16_low(tu_le16toh(req->wValue));
    const unsigned bits = alt == 1 ? 24u : 32u;
    portENTER_CRITICAL(&s_uac_buffer_mux);
    const uint32_t rate = s_ctx.requested_rate;
    const bool changed = bits != (unsigned)s_ctx.bit_depth || rate != s_ctx.sample_rate;
    /* Publish before RX is armed. An older ctrl ACK must not open the gate
     * between endpoint activation and the final SET_INTERFACE callback. */
    s_ctx.requested_bits = (int)bits;
    s_ctx.format_request_seq++;
    s_ctx.format_input_busy = true;  /* Final SET_INTERFACE completes RX arming. */
    const bool incompatible = s_uac_buffer.tail_b &&
        (s_uac_buffer.bits != bits || s_uac_buffer.rate != rate);
    /* ISO activation has now disabled the previous transfer. A rejected old
     * boundary may still have received one last packet after its first clear. */
    const bool clear_fifo = changed || incompatible || s_ctx.fifo_rearm_clear;
    s_ctx.fifo_rearm_clear = false;
    s_uac_buffer.playing = false;
    s_uac_buffer.wait_us = 0;
    s_uac_buffer.blocked = s_uac_buffer.blocked || changed;
    s_uac_buffer.closed_tail = false;
    if (clear_fifo) {
        s_uac_buffer.dropped_b += s_uac_buffer.tail_b;
        s_uac_buffer.tail_b = 0;
        s_uac_buffer.reset_b += tu_fifo_count(ff);
    }
    portEXIT_CRITICAL(&s_uac_buffer_mux);
    if (clear_fifo) tu_fifo_clear(ff);
}

/* Return a read ceiling: old boundary first, then bounded new prefill.
 * Once running, never re-arm the hold just because the water level is low. */
static uint16_t uac_buffer_readable(uint16_t avail, uint32_t now_us)
{
    uint16_t readable = 0;
    const uint16_t need = uac_buffer_start_bytes((unsigned)s_ctx.bit_depth);
    const unsigned stride = s_ctx.bit_depth == 24 ? 6u : 8u;
    portENTER_CRITICAL(&s_uac_buffer_mux);
    if (s_ctx.bus_suspended || !tud_audio_mounted()) {
        s_ctx.uac_resume_hold = true;
        portEXIT_CRITICAL(&s_uac_buffer_mux);
        return 0;
    }
#if CONFIG_UAC2_BT_INJECT
    /* ★ 0.151：注入模式下 UAC 分支不得消费 FIFO，否则注入拿不到数据。
     * 这里返回 0 ⇒ UAC 分支只输出静音；FIFO 全部留给注入路径。 */
    if (s_ctx.inject_mode) {
        s_ctx.uac_resume_hold = false;
        portEXIT_CRITICAL(&s_uac_buffer_mux);
        return 0;
    }
#endif
    /* RX may have extended the old boundary after the caller's level sample. */
    avail = tud_audio_available();
    s_ctx.uac_resume_hold = false;
    if (s_uac_buffer.tail_b > avail) {
        s_uac_buffer.boundary_err++;
        s_uac_buffer.dropped_b += s_uac_buffer.tail_b;
        s_uac_buffer.tail_b = 0;
    }
    if (!s_uac_buffer.blocked) {
        if (s_uac_buffer.tail_b) {
            readable = (uint16_t)s_uac_buffer.tail_b;
        } else {
            if (s_uac_buffer.playing && avail < stride) {
                s_uac_buffer.playing = false;
                s_uac_buffer.wait_us = 0;
            }
            if (!s_uac_buffer.playing && s_ctx.streaming && avail >= stride) {
                if (!s_uac_buffer.wait_us) s_uac_buffer.wait_us = now_us;
                const bool full = avail >= need;
                if (full || now_us - s_uac_buffer.wait_us >= UAC_RESUME_TIMEOUT_MS * 1000u) {
                    s_uac_buffer.playing = true;
                    s_uac_buffer.wait_us = 0;
                    s_uac_buffer.starts++;
                    if (!full) s_uac_buffer.timeouts++;
                }
            }
            if (s_uac_buffer.playing) readable = avail;
            else if (avail) s_ctx.uac_resume_hold = true;
            else s_uac_buffer.wait_us = 0;
        }
    } else s_ctx.uac_resume_hold = true;
    portEXIT_CRITICAL(&s_uac_buffer_mux);
    return readable;
}

/* ISO may finish one last old packet after the alt0 callback. Extend the
 * boundary only for bytes that actually entered this FIFO, before RX can wake
 * the pump. The pre-open hook closes this phase before arming the new stream. */
static void uac_buffer_rx_committed(const tu_fifo_t *ff)
{
    if (!s_uac_buffer.closed_tail) return;
    portENTER_CRITICAL_ISR(&s_uac_buffer_mux);
    if (s_uac_buffer.closed_tail && !s_uac_buffer.blocked &&
        s_ctx.active_source == AUDIO_SOURCE_UAC && !s_ctx.source_switching &&
        s_ctx.switch_phase != SWITCH_SETTLING && s_ctx.switch_phase != SWITCH_MUTED) {
        /* The close snapshot can already include a packet whose completion
         * callback is pending. Credit only write-index progress beyond that
         * snapshot, never the callback length a second time. */
        uint32_t wr = ff->wr_idx;
        const uint32_t span = 2u * ff->depth;
        uint32_t bytes = wr >= s_uac_buffer.tail_wr ? wr - s_uac_buffer.tail_wr
            : wr + span - s_uac_buffer.tail_wr;
        s_uac_buffer.tail_wr = wr;
        s_uac_buffer.tail_b += bytes;
        s_uac_buffer.kept_b += bytes;
    }
    portEXIT_CRITICAL_ISR(&s_uac_buffer_mux);
}

static void uac_buffer_consumed(uint16_t bytes, bool delivered)
{
    portENTER_CRITICAL(&s_uac_buffer_mux);
    uint32_t tail = bytes < s_uac_buffer.tail_b ? bytes : s_uac_buffer.tail_b;
    s_uac_buffer.tail_b -= tail;
    if (delivered) s_uac_buffer.drained_b += tail;
    else {
        s_uac_buffer.dropped_b += tail;
        s_uac_buffer.submit_drop_b += bytes;
    }
    portEXIT_CRITICAL(&s_uac_buffer_mux);
}

static void uac_buffer_print(void)
{
    portENTER_CRITICAL(&s_uac_buffer_mux);
    const uac_buffer_t b = s_uac_buffer;
    const uint32_t req = s_ctx.format_request_seq, done = s_ctx.format_done_seq;
    const uint32_t rate = s_ctx.requested_rate;
    const int bits = s_ctx.requested_bits;
    const unsigned alt = s_ctx.stream_alt, suspended = s_ctx.bus_suspended;
    const unsigned input_busy = s_ctx.format_input_busy, rearm_clear = s_ctx.fifo_rearm_clear;
    portEXIT_CRITICAL(&s_uac_buffer_mux);
    ESP_LOGI(TAG, "uac-buffer: closes=%lu kept_b=%lu drained_b=%lu dropped_b=%lu short_b=%lu reset_b=%lu tail_b=%lu starts=%lu timeouts=%lu boundary_err=%lu submit_drop_b=%lu close_us=%lu close_b=%lu playing=%u blocked=%u short_kept_b=%lu",
             (unsigned long)b.closes, (unsigned long)b.kept_b, (unsigned long)b.drained_b,
             (unsigned long)b.dropped_b, (unsigned long)b.short_b, (unsigned long)b.reset_b,
             (unsigned long)b.tail_b, (unsigned long)b.starts, (unsigned long)b.timeouts,
             (unsigned long)b.boundary_err, (unsigned long)b.submit_drop_b,
             (unsigned long)b.close_us, (unsigned long)b.close_b, b.playing, b.blocked,
             (unsigned long)b.short_kept_b);
    ESP_LOGI(TAG, "uac-config: req=%lu done=%lu want_rate=%lu want_bits=%d alt=%u suspended=%u input_busy=%u rearm_clear=%u bg_discard_b=%lu bg_reads=%lu",
             (unsigned long)req, (unsigned long)done, (unsigned long)rate,
             bits, alt, suspended, input_busy, rearm_clear,
             (unsigned long)s_ctx.background_usb_bytes, (unsigned long)s_ctx.background_usb_reads);
}

/* Apply Windows UAC Feature Unit volume/mute to 32-bit left-justified PCM.
 *
 * v9 low-distortion path:
 *  - master/L/R all 0 dB and unmuted -> bit-perfect pass-through, no float math;
 *  - any gain or mute -> gains are evaluated in double precision and rounded
 *    to nearest 32-bit sample. Gains are always <= 1.0 (0..-49.6 dB), so the
 *    conversion cannot overflow. */
static void uac_apply_volume_mute(int32_t *p, size_t samples)
{
    int32_t vol_l = (int32_t)s_volume[0] + (int32_t)s_volume[1];
    int32_t vol_r = (int32_t)s_volume[0] + (int32_t)s_volume[2];
    if (vol_l < -12700) vol_l = -12700;
    if (vol_r < -12700) vol_r = -12700;
    if (vol_l > 0) vol_l = 0;
    if (vol_r > 0) vol_r = 0;

    bool mute_l = (s_mute[0] != 0) || (s_mute[1] != 0);
    bool mute_r = (s_mute[0] != 0) || (s_mute[2] != 0);

    /* Bit-perfect fast path: unity gain and unmuted. */
    if (!mute_l && !mute_r && vol_l == 0 && vol_r == 0) {
        return;
    }

    /* Pump-owned cache: same formula and lround; mute never invalidates a
     * matching gain. Recalculate a channel only when its effective dB changes. */
    static int32_t cached_l = INT32_MAX, cached_r = INT32_MAX;
    static double gain_l, gain_r;
    if (!mute_l && vol_l != cached_l) {
        gain_l = pow(10.0, (double)vol_l / 256.0 / 20.0);
        cached_l = vol_l;
    }
    if (!mute_r && vol_r != cached_r) {
        gain_r = pow(10.0, (double)vol_r / 256.0 / 20.0);
        cached_r = vol_r;
    }

    for (size_t i = 0; i + 1 < samples; i += 2) {
        p[i]     = mute_l ? 0 : (int32_t)lround((double)p[i] * gain_l);
        p[i + 1] = mute_r ? 0 : (int32_t)lround((double)p[i + 1] * gain_r);
    }
}

static void uac_discard_usb_fifo(void)
{
    if (!tud_audio_mounted()) {
        return;
    }

    /* One bounded snapshot: background USB must not keep priority 24 running
     * indefinitely by chasing packets arriving while this tick drains. */
    uint16_t remaining = tud_audio_available();
    while (remaining > 0) {
        uint16_t chunk = (remaining > sizeof(s_usb24_buf)) ? (uint16_t)sizeof(s_usb24_buf) : remaining;
        uint16_t got = tud_audio_read(s_usb24_buf, chunk);
        if (got == 0) {
            break;
        }
        s_ctx.background_usb_bytes += got;
        s_ctx.background_usb_reads++;
        if (got > remaining) break;
        remaining -= got;
    }
}

/* ★ 0.113：把一个已填好的槽交给 core1（唯一的 DSP/输出所有者）。
 * samples 是槽里有效的 int32 立体声样本数；SILENCE 描述符忽略槽内容。
 * 返回 false 表示描述符没能交出去（真实错误，已在别处计数）。 */
static bool uac_slot_submit_result(int slot, size_t samples, esp_err_t ret)
{
    if (ret != ESP_OK) {
        dsp_pcm_release(slot);
        s_ctx.submit_errors++;
        return false;
    }
    s_ctx.submit_blocks++;
    s_ctx.submit_frames += (uint32_t)(samples / 2);
    return true;
}

static bool uac_slot_submit(int slot, size_t samples, audio_source_t source, uint32_t flags)
{
    if (samples == 0) {
        samples = DSP_PCM_SLOT_SAMPLES;
    }
    if (s_dsp_test_bypass) {
        /* 0xB5：只绕引擎，不绕 core1 的格式转换/端口/输出。 */
        flags |= DSP_PCM_F_ENGINE_BYPASS;
    }
    esp_err_t ret = dsp_pcm_submit(slot, samples, i2s_out_get_rate(),
                                   (source == AUDIO_SOURCE_BT) ? DSP_SOURCE_BT : DSP_SOURCE_UAC,
                                   flags, i2s_out_get_epoch());
    return uac_slot_submit_result(slot, samples, ret);
}

static bool uac_bt_slot_submit(int slot, size_t samples, uint32_t epoch, bool *has_data)
{
    uint32_t flags = DSP_PCM_F_HAS_DATA;
    if (s_dsp_test_bypass) flags |= DSP_PCM_F_ENGINE_BYPASS;
    esp_err_t ret = a2dp_audio_submit_pcm32_epoch(slot, samples, flags, epoch);
    *has_data = ret == ESP_OK;
    if (ret == ESP_ERR_INVALID_STATE) {
        /* A real BT event invalidated the read. Keep this output tick as a
         * full silence block; do not submit stale PCM or label it a queue error. */
        return uac_slot_submit(slot, DSP_PCM_SLOT_SAMPLES, AUDIO_SOURCE_BT,
                               DSP_PCM_F_SILENCE);
    }
    return uac_slot_submit_result(slot, samples, ret);
}

static esp_err_t uac_output_reconfigure(uint32_t rate, int bits)
{
    return i2s_out_reconfig(rate, bits);
}

/* Pump work stays bounded; low-priority audio_input_budget owns runtime sampling. */
static void uac_pump_task(void *arg)
{
    (void)arg;

    for (;;) {
        if (!i2s_out_wait_tx_done(portMAX_DELAY)) {
            continue;
        }

        s_ctx.pump_runs++;
        if (!dsp_pcm_begin()) continue;

        /* ★ v33（0.32）：tick 计时 —— 从"上一个描述符播完"到"本块写进 I2S"的完整耗时。
         * 这就是规格 §②① 要的 P（生产者完成一块的真机值），也是 §③b 的 P/T 判据来源。
         * ⚠ 只有**两个** `esp_timer_get_time()`（1 µs 时基，各 ≈1 µs 开销），
         *   高频的逐项分解若要做，另开一次性 PR 单独测，不留在这里。
         * ⚠ 与 core1 无关：core1 的每块 min/avg/max 由既有遥测 0xA4 提供，**不在这里重复采**。 */
        const int64_t tick_t0 = esp_timer_get_time();
        const bool input_probe = uac_input_probe_begin((uint32_t)tick_t0);
        uac_trace_state(0);
        if ((s_legacy_diag_mode & 1u) && s_ctx.active_source == AUDIO_SOURCE_UAC &&
            s_ctx.streaming && s_ctx.have_audio) {
            const uint32_t now_us = (uint32_t)tick_t0;
            if (s_observe_prev_us != 0) {
                const uint32_t gap_us = now_us - s_observe_prev_us;
                if (gap_us > s_observe_gap_max_us) s_observe_gap_max_us = gap_us;
            }
            s_observe_prev_us = now_us;
        } else {
            s_observe_prev_us = 0;
        }
        /* ★ 0.113：core0 每个 DMA 节拍只准备**一个**描述符并交给 core1；
         * 它不等 DSP、不做槽转换、不写 I2S。计时口径见 c1out 遥测。 */
        int      slot = -1;
        size_t   n = 0;               /* valid int32 samples in the slot */
        uint32_t flags = DSP_PCM_F_SILENCE;
        bool     submitted = false;

        uint16_t uac_read_bytes = 0;
        bool has_data = false;
        /* 0.61: 提到 tick 顶层 —— 分支外的 nonaudio_runs 判据也要看它 */
        bool uac_hold_now = false;
        audio_source_t source = (audio_source_t)s_ctx.active_source;
        a2dp_audio_state_t bt = {0};
        if (source == AUDIO_SOURCE_BT) a2dp_audio_get_state(&bt);
        /* Incoming UAC/BT activity is counted at receive callbacks, before
         * source selection or output prebuffering. */
        if (source != AUDIO_SOURCE_UAC || s_ctx.source_switching ||
            s_ctx.switch_phase == SWITCH_MUTED) uac_buffer_reset(false);
#if CONFIG_UAC2_BT_INJECT
        /* ★ 0.151：首帧到达后启用注入（用真实 UAC 格式冒充"已解码的 BT PCM"），
         * 然后每 15 秒自动换一次 vendor/codec 模拟切码。rate 必须等于 UAC 实际率，
         * 否则输出会变调 —— 想测 44.1/48/96 就在 PC 侧改 UAC 采样率。 */
        if (!s_ctx.inject_mode && s_ctx.sample_rate >= 8000u &&
            (s_ctx.bit_depth == 16 || s_ctx.bit_depth == 24 || s_ctx.bit_depth == 32)) {
            s_ctx.inject_mode = true;
            a2dp_audio_inject_enable(s_ctx.sample_rate, s_ctx.bit_depth, 2, 0u, 0u);
            ESP_LOGW(TAG, "BT INJECT on: rate=%u bits=%d (codec 0=SBC)",
                     (unsigned)s_ctx.sample_rate, s_ctx.bit_depth);
        }
        if (s_ctx.inject_mode) {
            static int64_t s_inject_next_switch_us = 0;
            static uint8_t s_inject_cycle = 0;
            const int64_t now_us_inj = esp_timer_get_time();
            if (now_us_inj > s_inject_next_switch_us) {
                /* 0 = SBC（vid/cid 0）, 1 = AAC（同 vid/cid 但 codec_type 由 SDK 决定，
                 * 注入侧只能区分 vendor codec）, 2 = LDAC（0x12D/0xAA → 高水位） */
                static const uint32_t vids[3] = {0u, 0u, 0x0000012Du};
                static const uint32_t cids[3] = {0u, 0u, 0x000000AAu};
                static const char *names[3] = {"SBC", "AAC", "LDAC"};
                a2dp_audio_inject_enable(s_ctx.sample_rate, s_ctx.bit_depth, 2,
                                         vids[s_inject_cycle], cids[s_inject_cycle]);
                ESP_LOGW(TAG, "BT INJECT cycle=%s vendor=0x%X codec=0x%X rate=%u",
                         names[s_inject_cycle], (unsigned)vids[s_inject_cycle],
                         (unsigned)cids[s_inject_cycle], (unsigned)s_ctx.sample_rate);
                s_inject_cycle = (uint8_t)((s_inject_cycle + 1u) % 3u);
                s_inject_next_switch_us = now_us_inj + 15000000LL;
            }
        }
#endif
#if CONFIG_UAC2_BT_INJECT
        /* ★ 0.151：PCM 注入（测试用）。**不再依赖 source==BT** ——
         * 那会形成鸡生蛋死锁：source 要变 BT 需要 ring 先有数据，而注入又要求
         * source 已是 BT。现在只要 UAC 有数据就注入，ring 涨起来后
         * uac_poll_source_policy 自然会把 source 切到 BT。
         * 为避免和 UAC 分支抢 FIFO，注入模式下 uac_buffer_readable() 会返回 0
         * （见该函数的 inject_mode 判断）。 */
        if (s_ctx.inject_mode) {
            const uint16_t a = tud_audio_available();
            if (a >= 8u) {
                uint16_t want = (a > DSP_PCM_SLOT_BYTES) ? (uint16_t)DSP_PCM_SLOT_BYTES : a;
                want -= want % 8u;
                if (want > 0u) {
                    const uint16_t got = tud_audio_read(s_inject_buf, want);
                    if (got >= 8u) {
                        a2dp_audio_inject_pcm(s_inject_buf, got);
                        s_ctx.inject_bytes += got;
                    }
                }
            }
        }
#endif
        /* ★ 0.151：注入模式下**不能**丢 USB FIFO —— 那是注入的数据源。
         * 否则会形成震荡：source=BT 丢掉 FIFO → ring 掉 → 切回 UAC → 注入再读……
         * 实测会打出每几微秒一条 state-trace。 */
        if (source == AUDIO_SOURCE_BT) {
#if CONFIG_UAC2_BT_INJECT
            if (!s_ctx.inject_mode) uac_discard_usb_fifo();
#else
            uac_discard_usb_fifo();
#endif
        }
        if (s_ctx.source_switching || s_ctx.switch_phase == SWITCH_MUTED) {
            /* Only an actual source/clock transition silences this branch.
             * Idle BT connection/CFG does not alter a playing UAC path. */
            slot = dsp_pcm_acquire();
            if (slot >= 0) {
                n = DSP_PCM_SLOT_SAMPLES;
                submitted = uac_slot_submit(slot, n, source, flags);
            }
        } else if (source == AUDIO_SOURCE_BT) {
            /* BT has priority. Keep the TinyUSB OUT FIFO drained so the host
             * feedback loop stays healthy while UAC audio is intentionally
             * discarded. No UAC volume/mute processing is applied to BT PCM. */
            /* ★ 0.164：记录"什么时候开始没有真实流"。
             * !bt.streaming 说明手机已停流（切码的 SUSPEND）——
             * 到下一次真实出块之间的时长就是这段静音，直接量化 seek 感。 */
            if (!bt.streaming) {
                if (s_ctx.bt_gap_start_us == 0) {
                    s_ctx.bt_gap_start_us = esp_timer_get_time();
                }
            }
            if (s_ctx.bt_gate_open && bt.decoder_ready && bt.streaming &&
                bt.epoch == s_ctx.bt_gate_epoch && bt.sample_rate == i2s_out_get_rate()) {
                size_t got = 0;
                const uint32_t rd_epoch = bt.epoch;
                slot = dsp_pcm_acquire();
                if (slot >= 0) {
                    got = a2dp_audio_read_pcm32_epoch((uint8_t *)dsp_pcm_slot(slot),
                                                     DSP_PCM_SLOT_BYTES, rd_epoch);
                }
                if (slot >= 0 && got == DSP_PCM_SLOT_BYTES) {
                    n = got / sizeof(int32_t);
                    submitted = uac_bt_slot_submit(slot, n, rd_epoch, &has_data);
                    if (submitted && has_data) {
                        s_ctx.bt_delivered_blocks++;
                        /* ★ 0.158：真实出块 ⇒ 连续欠载清零 */
                        s_ctx.bt_silence_streak = 0;
                        /* ★ 0.164：若之前在"非流"状态，这一刻就是恢复点 */
                        if (s_ctx.bt_gap_start_us != 0) {
                            const int64_t gap_us = esp_timer_get_time() - s_ctx.bt_gap_start_us;
                            const uint32_t gap_ms = (uint32_t)(gap_us / 1000);
                            s_ctx.bt_last_gap_ms = gap_ms;
                            if (gap_ms > s_ctx.bt_max_gap_ms) s_ctx.bt_max_gap_ms = gap_ms;
                            s_ctx.bt_gap_start_us = 0;
                        }
                    }
                } else {
                    if (slot >= 0) {
                        dsp_pcm_release(slot);   /* underrun: never output junk */
                    }
                    if (a2dp_audio_is_streaming()) {
                        s_ctx.bt_silence_runs++;      /* 纯累计：只增不清 */
                        s_ctx.bt_silence_streak++;    /* ★ 0.158：连续欠载 */
                        if (s_ctx.bt_silence_runs <= 3) {
                            ESP_LOGW(TAG, "BT PCM ring underrun while streaming (#%lu)",
                                     (unsigned long)s_ctx.bt_silence_runs);
                        }
                        /* ★ 0.158（Codex 99 号缺口④）：只有**连续**欠载才关门重预缓冲。
                         * 旧实现用累计 bt_silence_runs 判定，相隔很久的两次欠载也会触发，
                         * 而且把它清零破坏了"单调累计"的口径。现在用独立 streak：
                         * 真实出块即清零，bt_silence_runs 永不重置。 */
                        if (s_ctx.bt_silence_streak >= 2) {
                            s_ctx.bt_gate_open = false;
                            s_ctx.bt_gate_epoch = bt.epoch;
                            s_ctx.bt_silence_streak = 0;
                        }
                    }
                    /* 预缓冲/欠载期仍然每个节拍交一整块数字静音（与旧版每 tick
                     * 写整块静音一致），输出链仍由 core1 负责。 */
                    slot = dsp_pcm_acquire();
                    if (slot >= 0) {
                        submitted = uac_slot_submit(slot, DSP_PCM_SLOT_SAMPLES, source,
                                                    DSP_PCM_F_SILENCE);
                    }
                }
            } else {
                /* Selected-route prebuffer is retained until START/CFG
                 * decides its generation; no latency-driven content deletion. */
                /* 预缓冲期同样每节拍交一整块数字静音（旧版此处写整块零）。 */
                slot = dsp_pcm_acquire();
                if (slot >= 0) {
                    submitted = uac_slot_submit(slot, DSP_PCM_SLOT_SAMPLES, source,
                                                DSP_PCM_F_SILENCE);
                }
            }
        } else {
            uint16_t avail = 0;
#if CONFIG_UAC2_TEST_TONE
            {
                static float phase_cycles = 0.0f;
                static bool logged = false;
                if (!logged) {
                    ESP_LOGW(TAG, "TEST TONE mode: generating 1 kHz sine, USB audio is ignored");
                    logged = true;
                }
                slot = dsp_pcm_acquire();
                uint32_t rate = i2s_out_get_rate();
                float step = 1000.0f / (float)rate; /* cycles per sample */
                if (slot >= 0) {
                    int32_t *pcm = dsp_pcm_slot(slot);
                    for (uint32_t i = 0; i < I2S_OUT_DMA_FRAME_NUM; i++) {
                        int32_t v = (int32_t)(0.25f * 2147483647.0f * sinf(2.0f * 3.14159265f * phase_cycles));
                        pcm[i * 2] = v;
                        pcm[i * 2 + 1] = v;
                        phase_cycles += step;
                        if (phase_cycles >= 1.0f) {
                            phase_cycles -= 1.0f;
                        }
                    }
                    n = DSP_PCM_SLOT_SAMPLES;
                    flags = DSP_PCM_F_HAS_DATA;
                    has_data = true;
                    submitted = uac_slot_submit(slot, n, source, flags);
                }
            }
#else
            avail = tud_audio_available();
            /* True FIFO envelope: sampled on every pump iteration, not only at
             * diag time. A full FIFO means TinyUSB is dropping ISO packets. */
            if (avail < s_ctx.fifo_min) s_ctx.fifo_min = avail;
            if (avail > s_ctx.fifo_max) s_ctx.fifo_max = avail;
            avail = uac_buffer_readable(avail, (uint32_t)tick_t0);
            uac_hold_now = s_ctx.uac_resume_hold;

            /* ★ 0.113：先拿槽，再读 FIFO。没有空槽就**不读**（数据留在 FIFO 里
             * 由反馈环配平），既不覆写别人的槽，也不读了再丢。 */
            slot = dsp_pcm_acquire();
            if (slot >= 0) {
                int32_t *pcm = dsp_pcm_slot(slot);
                if (avail > 0 && !uac_hold_now) {
                    if (s_ctx.bit_depth == 24) {
                        uint16_t want = (avail > sizeof(s_usb24_buf)) ? (uint16_t)sizeof(s_usb24_buf) : avail;
                        want -= want % 6; /* complete stereo 24-bit frame */
                        if (want > 0) {
                            uint16_t got = tud_audio_read(s_usb24_buf, want);
                            uac_read_bytes = got;
                            if (got == want) {
                                /* usb_bytes 记的是**从 USB 读进来的原始字节**（= got），
                                 * 与 32bit 分支口径一致；不记转换后的字节数。 */
                                s_ctx.usb_bytes += got;
                                n = uac_convert_24_to_32((uint8_t *)pcm, s_usb24_buf, got) /
                                    sizeof(int32_t);
                                flags = DSP_PCM_F_HAS_DATA;
                                has_data = true;
                                s_ctx.last_data_us = esp_timer_get_time();
                            } else {
                                s_ctx.short_reads++;
                            }
                        }
                    } else {
                        uint16_t want = (avail > DSP_PCM_SLOT_BYTES) ? (uint16_t)DSP_PCM_SLOT_BYTES : avail;
                        want -= want % 8; /* complete stereo 32-bit frame */
                        if (want > 0) {
                            uint16_t got = tud_audio_read(pcm, want);
                            uac_read_bytes = got;
                            if (got == want) {
                                n = got / sizeof(int32_t);
                                flags = DSP_PCM_F_HAS_DATA;
                                has_data = true;
                                s_ctx.usb_bytes += got;
                                s_ctx.last_data_us = esp_timer_get_time();
                            } else {
                                s_ctx.short_reads++;
                            }
                        } else {
                            /* avail 有 1..7 字节（不够一个立体声 32bit 帧）：
                             * 本 tick 输出整块静音（v23 D1 计数器）。 */
                            s_ctx.pump_want0++;
                        }
                    }
                }

                if (!has_data) {
                    /* FIFO 空 / hold / 读失败：交一整块数字静音，内容和格式都由
                     * core1 生成，core0 不写任何输出缓冲。 */
                    n = DSP_PCM_SLOT_SAMPLES;
                    flags = DSP_PCM_F_SILENCE;
                    s_ctx.silence_runs++;
                    if (s_ctx.streaming && s_ctx.have_audio) {
                        s_ctx.active_silence_runs++;
                        if (s_ctx.active_silence_runs <= 3) {
                            ESP_LOGW(TAG, "FIFO underrun while streaming (#%lu)",
                                     (unsigned long)s_ctx.active_silence_runs);
                        }
                    }
                } else {
                    /* core0 输入侧（不动输出）：输入峰值 + 标准 UAC FU 音量/静音。 */
                    if (input_probe) uac_input_probe_scan(pcm, n, (uint32_t)tick_t0);
                    if (s_legacy_diag_mode & 1u) {
                        uac_observe_input_sequence(pcm, n * sizeof(int32_t), (uint32_t)tick_t0);
                        uac_observe_zeros(&s_observe_usb, pcm, n * sizeof(int32_t), (uint32_t)tick_t0);
                    }
                    uac_update_peaks(pcm, n, &s_ctx.usb_peak_l, &s_ctx.usb_peak_r);
                    uac_apply_volume_mute(pcm, n);
                    s_ctx.have_audio = true;
                }
                submitted = uac_slot_submit(slot, n, source, flags);
                if (uac_read_bytes) uac_buffer_consumed(uac_read_bytes, has_data && submitted);
            }
#endif

        }

        /* ★ 0.50（L44）H3：本 tick 有没有交付**真实音频**（保留的既有计数器，
         * 现在只作诊断；hw_mute 用 pcm_active_runs）。 */
        if (!has_data && !uac_hold_now) {
            s_ctx.nonaudio_runs++;
        }

        if (input_probe) {
            uac_input_probe_end(source == AUDIO_SOURCE_UAC &&
                                s_ctx.bit_depth == 32 && s_ctx.sample_rate == 96000 &&
                                !s_ctx.source_switching && s_ctx.switch_phase != SWITCH_MUTED,
                                (uint32_t)tick_t0);
        }

        /* D1 仪表：本 tick 是不是"没数据 → 输出整块静音"。放音中出现就说明在补静音。 */
        if (!has_data && s_ctx.streaming) {
            s_ctx.pump_nodata++;
        }

        /* 非放音态（起流前 / 停流后）被补的静音帧单独记账，**不计损失**。
         * 判据用时间窗而不是标志位：`streaming`/`have_audio` 在播放器退出后仍为 1。 */
        if (!has_data) {
            const int64_t last_data = s_ctx.last_data_us;
            const int64_t now_us = esp_timer_get_time();
            if (last_data == 0 || (now_us - last_data) > AUDIBLE_GRACE_US) {
                s_ctx.def_silent += DSP_PCM_SLOT_FRAMES;
            }
        }

        dsp_pcm_end();

        /* ★ 0.167：tick 级轨迹 —— **只写数组，绝不在此打日志**
         * （0.154 在 pump 里 ESP_LOGW 是当时崩溃循环的嫌疑之一）。
         * 摘要由 ctrl 任务通过 bt_trace_report() 汇总输出。
         * ev 只标"是否出块/交静音"；trim 次数从 bt-flow 的 play_trim_blk 看，
         * 不在热路径重复取 stats（那样要再进一次 PCM 锁）。 */
        if (s_ctx.active_source == AUDIO_SOURCE_BT) {
            a2dp_audio_state_t tbt = {0};
            a2dp_audio_get_state(&tbt);
            uint8_t st = 0, ev = 0;
            if (s_ctx.bt_gate_open) st |= 0x01;
            if (tbt.streaming) st |= 0x02;
            /* Bit 2 is retired: BT tail playback is disabled. */
            if (submitted && has_data) ev |= 0x01;
            if (submitted && !has_data) ev |= 0x02;
            bt_trace_push((uint32_t)tick_t0, tbt.level_bytes,
                          tbt.sample_rate ? tbt.sample_rate : s_ctx.sample_rate, st, ev);
        }

        /* 0.113 producer-side window: 本 tick 从节拍开始到描述符交出去的核心 0 成本。
         * core0 不再等待 core1 —— 端到端（含排队/DSP/端口/接受）在 c1out 遥测里。 */
        if (submitted) {
            const uint32_t prod_us = (uint32_t)(esp_timer_get_time() - tick_t0);
            s_ctx.p_n++;
            s_ctx.p_sum_us += prod_us;
            if (prod_us > s_ctx.p_max_us) {
                s_ctx.p_max_us = prod_us;
            }
        }
    }
}
/* ---------------- Control / diagnostics task ---------------- */

static void uac_apply_pending(void)
{
    portENTER_CRITICAL(&s_uac_buffer_mux);
    const uint32_t seq = s_ctx.format_request_seq;
    if (seq == s_ctx.format_done_seq || s_ctx.format_input_busy) {
        portEXIT_CRITICAL(&s_uac_buffer_mux);
        return;
    }
    const uint32_t rate = s_ctx.requested_rate;
    const int bits = s_ctx.requested_bits;
    const bool bt_selected = s_ctx.active_source == AUDIO_SOURCE_BT;
    const bool format_changed = !bt_selected &&
        (rate != i2s_out_get_rate() || bits != i2s_out_get_bits());
    if (format_changed) s_uac_buffer.blocked = true;
    portEXIT_CRITICAL(&s_uac_buffer_mux);

    esp_err_t ret = ESP_OK;
    if (bt_selected) {
        uac_buffer_reset(false);
        /* Do not touch the I2S clock while BT owns the output. UAC keeps its
         * FIFO drained by the pump; apply the host format only as state. */
        /* Only the pump consumes background USB. A control-task clear
         * racing its read would create a second consumer of the OUT FIFO. */
    } else {
        /* 0.113: 停供给 → core1 块边界重配 → 新 epoch 恢复供给（uac_output_reconfigure）。 */
        ret = format_changed ? uac_output_reconfigure(rate, bits) : ESP_OK;
    }

    portENTER_CRITICAL(&s_uac_buffer_mux);
    const uint32_t latest = s_ctx.format_request_seq;
    const bool current = seq == latest && !s_ctx.format_input_busy;
    if (ret == ESP_OK) {
        /* Record the actual/logical UAC format even when an ACK is obsolete;
         * requested_* still names the newer host request. Never clear its FIFO
         * or release its format gate here. USB callbacks own input boundaries. */
        s_ctx.sample_rate = rate;
        s_ctx.bit_depth = bits;
        if (!bt_selected) s_ctx.output_rate = rate;
        if (bt_selected || format_changed) s_ctx.have_audio = false;
        if (current) {
            s_ctx.format_done_seq = seq;
            s_uac_buffer.blocked = false;
        }
    }
    const bool streaming = s_ctx.streaming;
    portEXIT_CRITICAL(&s_uac_buffer_mux);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "stream config applied: %lu Hz / %d bit request=%lu latest=%lu (%s)",
                 (unsigned long)rate, bits, (unsigned long)seq, (unsigned long)latest,
                 !current ? "superseded" : bt_selected ? "BT owns output" :
                 streaming ? "RUNNING" : "idle");
    } else {
        /* Leave this request outstanding and the gate closed. A fresh host
         * request/resume may retry; the 10 ms policy poll is not a retry loop. */
        ESP_LOGE(TAG, "apply %lu Hz / %d bit request=%lu failed: %s",
                 (unsigned long)rate, bits, (unsigned long)seq, esp_err_to_name(ret));
    }
}

static void uac_request_switch(audio_source_t source, uint32_t rate,
                               const a2dp_audio_state_t *bt)
{
    if (rate < 8000 || rate > 96000) return;
    if (s_ctx.active_source == source && s_ctx.output_rate == rate &&
        i2s_out_get_rate() == rate && !s_ctx.source_switching) return;

    uint32_t uac_seq = 0;
    if (source == AUDIO_SOURCE_UAC) {
        portENTER_CRITICAL(&s_uac_buffer_mux);
        uac_seq = s_ctx.format_request_seq;
        const bool ready = uac_seq == s_ctx.format_done_seq;
        portEXIT_CRITICAL(&s_uac_buffer_mux);
        if (!ready) return;
    }
    s_ctx.switch_req = (switch_request_t){
        .source = source, .rate = rate, .bt_epoch = bt->epoch,
        .bt_packets = bt->packets, .bt_delivered = s_ctx.bt_delivered_blocks,
        .uac_format_seq = uac_seq};
    s_ctx.switch_req_pending = true;
    uac_trace_state(source == AUDIO_SOURCE_BT ? 1 : 2);
    if (s_ctx.ctrl_task != NULL) xTaskNotify(s_ctx.ctrl_task, EVT_SWITCH, eSetBits);
}

static bool uac_switch_current(const switch_request_t *req)
{
    a2dp_audio_state_t bt;
    a2dp_audio_get_state(&bt);
    if (req->source == AUDIO_SOURCE_BT) {
        return bt.epoch == req->bt_epoch && bt.acl_connected && bt.connected && bt.streaming && bt.decoder_ready &&
               bt.sample_rate == req->rate &&
               bt.duration_ms >= CONFIG_BT_A2DP_SWITCH_HOLD_MS &&
               /* ★ 0.150：按编码器区分起播水位（LDAC 更高） */
               bt.level_bytes >= bt_prebuffer_bytes(req->rate, &bt);
    }
    portENTER_CRITICAL(&s_uac_buffer_mux);
    const bool uac_current = req->uac_format_seq == s_ctx.format_request_seq &&
        req->uac_format_seq == s_ctx.format_done_seq;
    portEXIT_CRITICAL(&s_uac_buffer_mux);
    const bool terminal = !bt.acl_connected || bt.cfg_failed;
    return uac_current && (terminal || (bt.epoch == req->bt_epoch && !bt.route_hold &&
           bt.packets == req->bt_packets && s_ctx.bt_delivered_blocks == req->bt_delivered));
}

static void uac_switch_failure(esp_err_t err)
{
    a2dp_audio_state_t bt;
    a2dp_audio_get_state(&bt);
    if (bt.epoch != s_ctx.switch_failed_bt_epoch ||
        s_ctx.format_request_seq != s_ctx.switch_failed_uac_seq) s_ctx.switch_failures = 0;
    ++s_ctx.switch_failures;
    s_ctx.switch_last_error = err;
    s_ctx.switch_failed_bt_epoch = bt.epoch;
    s_ctx.switch_failed_uac_seq = s_ctx.format_request_seq;
    s_ctx.switch_retry_us = esp_timer_get_time() + SOURCE_RETRY_US;
    s_ctx.switch_phase = s_ctx.switch_failures < SOURCE_MAX_FAILURES ?
                         SWITCH_RECOVER_WAIT : SWITCH_FAULT;
    s_ctx.switch_req_pending = false;
    ESP_LOGE(TAG, "trace-route: event=switch_failed rc=%s failures=%lu retry=%u p0run=%u epoch=%lu uac_seq=%lu",
        esp_err_to_name(err), (unsigned long)s_ctx.switch_failures,
        s_ctx.switch_phase == SWITCH_RECOVER_WAIT, i2s_out_is_running(),
        (unsigned long)bt.epoch, (unsigned long)s_ctx.switch_failed_uac_seq);
}

static bool uac_abort_switch(uint32_t old_rate, int old_bits, bool clock_attempted)
{
    esp_err_t ret = clock_attempted ? uac_output_reconfigure(old_rate, old_bits) : ESP_OK;
    bool restored = ret == ESP_OK && i2s_out_is_running() && i2s_out_get_rate() == old_rate &&
                    i2s_out_get_bits() == old_bits;
    s_ctx.bt_gate_open = false;
    s_ctx.source_switching = !restored;
    s_ctx.switch_phase = restored ? SWITCH_IDLE : SWITCH_MUTED;
    s_ctx.switch_req_pending = false;
    /* Failed rollback stays explicitly silent. Never interpret the old source
     * with a partially changed clock; no fake rate/format fence is used. */
    if (!restored) {
        uac_switch_failure(ret != ESP_OK ? ret : ESP_ERR_INVALID_STATE);
        ESP_LOGE(TAG, "source switch rollback unconfirmed; output held silent, bounded recovery pending");
    }
    else ESP_LOGI(TAG, "trace-route: us=%lu event=switch_cancelled (source/config changed)",
                  (unsigned long)(uint32_t)esp_timer_get_time());
    uac_trace_state(3);
    return restored;
}

static bool uac_apply_switch_request(void)
{
    if (!s_ctx.switch_req_pending) return true;
    const switch_request_t req = s_ctx.switch_req;
    const audio_source_t old_source = s_ctx.active_source;
    const uint32_t old_rate = i2s_out_get_rate();
    const int old_bits = i2s_out_get_bits();
    if (!uac_switch_current(&req)) {
        s_ctx.switch_req_pending = false;
        return true;
    }

    /* PCM production stops at its next tick. Already-submitted descriptors
     * retain their legitimate FIFO/DMA tail at the same rate (no output epoch
     * is invented). An actual rate change uses the frozen public control path. */
    s_ctx.source_switching = true;
    s_ctx.switch_phase = SWITCH_SETTLING;
    uac_trace_state(3);
    if (req.source == AUDIO_SOURCE_UAC && old_source == AUDIO_SOURCE_BT) {
        vTaskDelay(pdMS_TO_TICKS(BT_FALLBACK_COMMIT_MS));
    }
    if (!uac_switch_current(&req)) return uac_abort_switch(old_rate, old_bits, false);

    s_ctx.switch_phase = SWITCH_MUTED;
    esp_err_t ret = uac_output_reconfigure(req.rate, 32);
    if (ret == ESP_OK && (!i2s_out_is_running() || i2s_out_get_rate() != req.rate ||
                          i2s_out_get_bits() != 32)) ret = ESP_ERR_INVALID_STATE;
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "source switch reconfig to %lu Hz failed: %s",
                 (unsigned long)req.rate, esp_err_to_name(ret));
        /* A failed low-level mutation can leave the physical clock different
         * from its cached rate. A same-rate wrapper no-op is not a rollback. */
        /* Restore through the existing frozen control API, then retry a fresh
         * policy decision at most twice. Never stay MUTED after one failed
         * command, spin every tick, or fake a successful same-rate restart. */
        if (uac_abort_switch(old_rate, old_bits, true)) uac_switch_failure(ret);
        return false;
    }
    /* Reconfiguration yields. Recheck the requested BT generation/rate (or
     * fallback's hold/arrival snapshot) before the atomic BT selection cut. */
    if (!uac_switch_current(&req) ||
        !a2dp_audio_commit_output(req.source == AUDIO_SOURCE_BT, req.bt_epoch,
                                  req.rate, req.bt_packets)) {
        return uac_abort_switch(old_rate, old_bits, true);
    }

    /* BT's target prebuffer survives. Only the departing source is flushed;
     * commit_output already invalidated/cleared departing BT under its lock. */
    if (old_source == AUDIO_SOURCE_UAC && req.source == AUDIO_SOURCE_BT && tud_audio_mounted()) {
        tud_audio_clear_ep_out_ff();
    }
    s_ctx.active_source = req.source;
    s_ctx.output_rate = req.rate;
    s_ctx.have_audio = false;
    /* ★ 0.158：切源边界是既有重置点，累计口径 = "本段 BT 会话内累计"，不是全启动周期。
     * 连续判据改用独立的 bt_silence_streak，同样在这里归零。 */
    s_ctx.bt_silence_runs = 0;
    s_ctx.bt_silence_streak = 0;
    /* 0.173: uac_switch_current() already proved BT streaming + prebuffer +
     * rate for this epoch. Open the gate in the same locked switch instead of
     * leaving it false for the next 10 ms policy poll: that poll runs in the
     * low-priority ctrl task, which can be busy printing the diag block for
     * hundreds of ms (measured at the 96 kHz switch). The policy poll still
     * closes the gate if BT dies; this only removes the startup hole. */
    s_ctx.bt_gate_open = (req.source == AUDIO_SOURCE_BT);
    s_ctx.bt_gate_epoch = req.bt_epoch;
    s_ctx.bt_alive_us = req.source == AUDIO_SOURCE_BT ? esp_timer_get_time() : 0;
    s_ctx.bt_last_packets = req.bt_packets;
    s_ctx.bt_last_delivered = s_ctx.bt_delivered_blocks;
    s_ctx.source_switching = false;
    s_ctx.switch_phase = SWITCH_IDLE;
    s_ctx.switch_failures = 0;
    s_ctx.switch_retry_us = 0;
    s_ctx.switch_last_error = ESP_OK;
    s_ctx.switch_req_pending = false;
    uac_trace_state(3);
    ESP_LOGI(TAG, "output source switched to %s @ %lu Hz epoch=%lu",
             req.source == AUDIO_SOURCE_BT ? "BT" : "UAC",
             (unsigned long)req.rate, (unsigned long)req.bt_epoch);
    return true;
}

static void uac_poll_source_policy(void)
{
    a2dp_audio_state_t bt;
    a2dp_audio_get_state(&bt);
    if (s_ctx.switch_phase == SWITCH_RECOVER_WAIT || s_ctx.switch_phase == SWITCH_FAULT) {
        const bool fresh = bt.epoch != s_ctx.switch_failed_bt_epoch ||
                           s_ctx.format_request_seq != s_ctx.switch_failed_uac_seq;
        if (fresh) {
            s_ctx.switch_failures = 0;
            s_ctx.switch_retry_us = 0;
        }
        if ((s_ctx.switch_phase == SWITCH_FAULT && !fresh) ||
            esp_timer_get_time() < s_ctx.switch_retry_us) return;
        s_ctx.switch_phase = SWITCH_IDLE;
    }
    if (s_ctx.switch_phase != SWITCH_IDLE) return;
    if (s_ctx.bt_gate_epoch != bt.epoch) {
        s_ctx.bt_gate_open = false;
        s_ctx.bt_gate_epoch = bt.epoch;
    }
    /* Initial connection/CFG stays on UAC. Only a selected BT route can own a
     * replacement wait, and the pump's BT gate then supplies silence.
     * ★ 0.150：起播门槛按编码器取 —— LDAC 需要更高水位，其余仍 40 ms。 */
    if (bt.connected && bt.streaming && bt.decoder_ready && bt.sample_rate >= 8000 &&
        bt.sample_rate <= 96000 && bt.duration_ms >= CONFIG_BT_A2DP_SWITCH_HOLD_MS &&
        (s_ctx.bt_gate_open || bt.level_bytes >= bt_prebuffer_bytes(bt.sample_rate, &bt))) {
        s_ctx.bt_gate_open = s_ctx.active_source == AUDIO_SOURCE_BT &&
                             i2s_out_get_rate() == bt.sample_rate;
        if (s_ctx.bt_gate_open) a2dp_audio_route_ready(bt.epoch);
        else uac_request_switch(AUDIO_SOURCE_BT, bt.sample_rate, &bt);
    } else {
        s_ctx.bt_gate_open = false;
    }

    if (s_ctx.active_source == AUDIO_SOURCE_BT) {
        const int64_t now_us = esp_timer_get_time();
        const uint32_t delivered = s_ctx.bt_delivered_blocks;
        /* ★ 0.165：区分"手机侧停流"与"设备侧消费过快"。
         * packets 是**收到解码包**的累计数（手机 → 设备的供给侧证据）；
         * delivered 是**交给 core1 的块数**（设备消费侧证据）。
         * 若 streaming 仍为 true 而 packets 长时间不涨 ⇒ 手机侧真的停流/降码率；
         * 若 packets 在涨而 ring 掉空 ⇒ 设备消费过快。两种情况修法完全不同，
         * 之前一直靠猜，就是因为没把这两个数分别记下来。 */
        if (bt.streaming && bt.packets == s_ctx.bt_last_packets) {
            if (s_ctx.bt_starve_start_us == 0) {
                s_ctx.bt_starve_start_us = now_us;
            }
        } else {
            if (s_ctx.bt_starve_start_us != 0) {
                const uint32_t ms = (uint32_t)((now_us - s_ctx.bt_starve_start_us) / 1000);
                s_ctx.bt_last_starve_ms = ms;
                if (ms > s_ctx.bt_max_starve_ms) s_ctx.bt_max_starve_ms = ms;
                if (ms >= 200u) s_ctx.bt_starve_events++;   /* 只统计 >=200ms 的真停滞 */
                s_ctx.bt_starve_start_us = 0;
            }
        }
        if (bt.packets != s_ctx.bt_last_packets || delivered != s_ctx.bt_last_delivered ||
            s_ctx.bt_alive_us == 0) s_ctx.bt_alive_us = now_us;
        s_ctx.bt_last_packets = bt.packets;
        s_ctx.bt_last_delivered = delivered;
        if (bt.route_hold) (void)a2dp_audio_route_hold_active(); /* one-shot 2 s diagnostic */
        const bool bt_dead = !bt.acl_connected || bt.cfg_failed ||
            (!bt.route_hold && (!bt.connected || (!bt.codec_busy &&
             now_us - s_ctx.bt_alive_us >= (int64_t)CONFIG_BT_A2DP_SWITCH_BACK_MS * 1000)));
        if (bt_dead) {
            uac_trace_state(4);
            ESP_LOGW(TAG, "BT no new PCM/delivery for %lld ms (connected=%d streaming=%d level=%u) -> UAC",
                     (long long)((now_us - s_ctx.bt_alive_us) / 1000), bt.connected, bt.streaming,
                     (unsigned)bt.level_bytes);
            uac_request_switch(AUDIO_SOURCE_UAC, s_ctx.sample_rate ? s_ctx.sample_rate : 48000, &bt);
        }
    } else {
        s_ctx.bt_alive_us = 0;
        /* A failed BT takeover may already have stopped the old clock. If no
         * BT stream now qualifies, recover the current USB target rather than
         * waiting forever for a BT-selected branch which was never committed. */
        if (s_ctx.source_switching && !s_ctx.switch_req_pending && !bt.route_hold)
            uac_request_switch(AUDIO_SOURCE_UAC, s_ctx.sample_rate, &bt);
        /* 0.127（工单 11 §2.2）：**预热段的高水位 trim 已移到生产者入环前**
         * （a2dp_audio_int.c 的 pcm_ring_push，条件 `!s_route_hold.selected`）。
         * 原因：原先在这里（低优先级 ctrl）做，生产者可以先把环填满 ⇒ 起播区间实测
         * overflow +37，丢弃的是**最新**数据。移到生产者后改为"丢最旧整块、保留新数据"。
         * ⚠ 本分支原来这一处 trim 已删除，避免与生产者重复 trim。
         * 注意：**已选 BT 的闭门 trim（:1013）与播放 trim（:1168）不在此处、未挪动。** */
    }
    uac_trace_state(0);
}

static void uac_ctrl_task(void *arg)
{
    (void)arg;
    TickType_t last_diag = xTaskGetTickCount();
    /* USB may have published a request before this task handle was installed. */
    uac_apply_pending();

    for (;;) {
        uint32_t evt = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(10));

        if (evt & (EVT_RATE | EVT_FORMAT)) {
            uac_apply_pending();
        }

        /* A finalized codec failure releases its own hold BEFORE source policy.
         * Apply pending USB state logically while BT still owns the clock. */
        bt_codec_guard_poll(s_ctx.bt_silence_runs,
            s_ctx.active_source == AUDIO_SOURCE_BT,
            s_ctx.bt_gate_open);
        if (s_ctx.active_source == AUDIO_SOURCE_BT &&
            s_ctx.format_request_seq != s_ctx.format_done_seq) uac_apply_pending();
        uac_poll_source_policy();
        audio_input_budget_poll(s_ctx.active_source == AUDIO_SOURCE_BT);
        if (s_ctx.switch_req_pending && s_ctx.switch_phase == SWITCH_IDLE) {
            uac_apply_switch_request();
        }
        TickType_t now = xTaskGetTickCount();
        bool any_active = s_ctx.streaming || a2dp_audio_is_streaming();
        uint32_t diag_ms = any_active ? 1000 : 5000;
        if ((now - last_diag) >= pdMS_TO_TICKS(diag_ms)) {
            last_diag = now;
            const uint32_t legacy_mode = s_legacy_diag_mode;
            const bool legacy = (legacy_mode & 1u) != 0;
            ESP_LOGI(TAG, "uac-legacy: mode=%lu enabled=%u coverage=%s fifo_extrema=%s",
                     (unsigned long)legacy_mode, legacy,
                     legacy ? "ENABLED_INTERVALS_ONLY" : "NOT_OBSERVED",
                     legacy ? "SAMPLED_1MS" : "DIAG_ENDPOINT_ONLY");
            uac_print_state_trace();
            if (legacy) {
                uac_print_input_sequence();
                /* The USB producer runs on core0; this blocks the core0
                 * reader. Balance is advisory on this version, since an in-flight
                 * ISR can update the FIFO before its received-byte callback. Raw
                 * input anchors and the LA golden sequence are the loss criterion. */
                portENTER_CRITICAL(&s_observe_rx_mux);
                const uint32_t rx_bytes = s_observe_rx_bytes;
                const uint32_t read_bytes = s_ctx.usb_bytes;
                const uint32_t input_level = tud_audio_available();
                portEXIT_CRITICAL(&s_observe_rx_mux);
                ESP_LOGI(TAG, "observe-usb: received=%lu read=%lu fifo=%lu balance=%lu irq_core=%lu atomic=0",
                         (unsigned long)rx_bytes, (unsigned long)read_bytes,
                         (unsigned long)input_level,
                         (unsigned long)(rx_bytes - read_bytes - input_level),
                         (unsigned long)s_observe_irq_core);
            }
            i2s_out_stats_t ist = {0};
            i2s_out_get_stats(&ist);
            ESP_LOGI(TAG, "output-owner: dma_irq_core=%lu", (unsigned long)ist.irq_core);
            dsp_audio_stats_t ds = {0};
            dsp_audio_get_stats(&ds);
            ESP_LOGI(TAG, "c1ctl: max=%lu skipped_ticks=%lu",
                     (unsigned long)ds.control_max_us, (unsigned long)ds.control_skips);
            a2dp_audio_stats_t ast = {0};
            a2dp_audio_get_stats(&ast);
            uint16_t fifo_level = tud_audio_available();
            uint16_t fifo_depth = 0;
            uint16_t fifo_min = s_ctx.fifo_min;
            uint16_t fifo_max = s_ctx.fifo_max;
            uint32_t active_sil = s_ctx.active_silence_runs;
            s_ctx.fifo_min = fifo_level;
            s_ctx.fifo_max = fifo_level;
            tu_fifo_t *ff = tud_audio_get_ep_out_ff();
            if (ff != NULL) {
                fifo_depth = tu_fifo_depth(ff);
            }
            ESP_LOGI(TAG, "diag: src=%s out=%lu slot=%d uac=%lu/%d stream=%d usb_pkt=%lu fifo=%u/%u min=%u max=%u pump=%lu silence=%lu active_sil=%lu bt_sil=%lu wr_err=%lu short=%lu short_rd=%lu i2s_ovf=%lu wr_retry=%lu wr_part=%lu wr_dup=%lu nodata=%lu want0=%lu usb_b=%lu lost=%lu dsil=%lu slow=%lu pmax=%lu pn=%lu wait1=%lu busy=%lu",
                     s_ctx.active_source == AUDIO_SOURCE_BT ? "BT" : "UAC",
                     (unsigned long)i2s_out_get_rate(), i2s_out_get_slot_width(),
                     (unsigned long)s_ctx.sample_rate, s_ctx.bit_depth,
                     s_ctx.streaming ? 1 : 0,
                     (unsigned long)s_ctx.usb_pkts, (unsigned)fifo_level, (unsigned)fifo_depth,
                     (unsigned)fifo_min, (unsigned)fifo_max,
                     (unsigned long)s_ctx.pump_runs, (unsigned long)s_ctx.silence_runs,
                     (unsigned long)active_sil, (unsigned long)s_ctx.bt_silence_runs,
                     /* 0.113：写路径搬到 core1。`wr_err/short` 取 I2S 输出层的真实
                      * 错误计数，`wr_retry/wr_part` 取 core1 输出循环，
                      * `wr_dup` 恒 0（重试只从接受前缀继续，不可能重复发送），
                      * `lost` 是**未交付帧数**（stride 口径，即旧 lost 的正确单位版），
                      * `wait1` 恒 0（core0 不再等 core1），`pmax/pn/busy` 是 core0 生产侧窗口。 */
                     (unsigned long)ist.write_errors, (unsigned long)ist.short_writes,
                     (unsigned long)s_ctx.short_reads,
                     (unsigned long)ist.queue_overflows,
                     (unsigned long)ds.write_retry, (unsigned long)ds.write_partial,
                     0ul,
                     (unsigned long)s_ctx.pump_nodata, (unsigned long)s_ctx.pump_want0,
                     (unsigned long)s_ctx.usb_bytes,
                     (unsigned long)ds.frames_undelivered, (unsigned long)s_ctx.def_silent,
                     (unsigned long)ds.deadline_miss, (unsigned long)s_ctx.p_max_us,
                     (unsigned long)s_ctx.p_n, 0ul,
                     (unsigned long)s_ctx.p_sum_us);
            /* ★ 0.50（L44）H3 / 0.113：`nonaudio` 与新的输入活动计数器单独一行 ——
             * hw_mute 的 playing 现在看 `pcm_active`（UAC 或 BT 任一在送 PCM）。 */
            ESP_LOGI(TAG, "diag-ext: nonaudio=%lu pcm_active=%lu (input activity OR; drives hw_mute playing)",
                     (unsigned long)s_ctx.nonaudio_runs,
                     (unsigned long)uac2_audio_get_pcm_active_tick());
            if (legacy) ESP_LOGI(TAG, "observe: us=%lu usb_zero_ev=%lu usb_last=%lu usb_at=%lu out_zero_blocks=%lu pump_gap_max=%lu",
                     (unsigned long)(uint32_t)esp_timer_get_time(),
                     (unsigned long)s_observe_usb.events, (unsigned long)s_observe_usb.last_frames,
                     (unsigned long)s_observe_usb.last_start_us,
                     (unsigned long)ds.out_zero_blocks,
                     (unsigned long)s_observe_gap_max_us);
            uac_buffer_print();
            ESP_LOGI(TAG, "source-recovery: phase=%u switching=%u failures=%lu last_rc=0x%x retry_us=%lld",
                (unsigned)s_ctx.switch_phase, s_ctx.source_switching,
                (unsigned long)s_ctx.switch_failures, (unsigned)s_ctx.switch_last_error,
                (long long)s_ctx.switch_retry_us);
            i2s1_out_stats_t dual;
            i2s1_out_get_stats(&dual);
            ESP_LOGI(TAG, "dual-i2s: p0run=%d p1ready=%d p1run=%d rate=%lu slot=%d fmt=%d calls=%lu bytes=%lu err=%lu partial=%lu last=%d layout=%u",
                     i2s_out_is_running(), dual.ready, dual.running,
                     (unsigned long)dual.rate, dual.slot_bits, dual.format,
                     (unsigned long)dual.calls, (unsigned long)dual.bytes,
                     (unsigned long)dual.errors, (unsigned long)dual.partial,
                     dual.last_error, DUAL_I2S_TEST_P0_PINS_ALT);
            /* ★ 0.113 core1 输出/预算遥测（**累计量不清零**，窗口量另有 win_* 字段）：
             * 每块分别给出 producer 准备/排队/DSP 净耗时/端口转换+提交/总墙钟，
             * 以及 deadline_miss、90% 预算越界、无空槽回压、旧 epoch 作废、
             * write error/partial/未交付字节与输出层 pending。 */
            ESP_LOGI(TAG, "c1out: blk=%lu uac=%lu bt=%lu sil=%lu byp=%lu stale=%lu bp=%lu dm=%lu b90=%lu wrerr=%lu wrpart=%lu undel_b=%lu fin=%lu fstage=%lu fundel=%lu pend=%lu dl_us=%lu",
                     (unsigned long)ds.blocks, (unsigned long)ds.uac_blocks,
                     (unsigned long)ds.bt_blocks, (unsigned long)ds.silence_blocks,
                     (unsigned long)ds.bypass_blocks, (unsigned long)ds.stale_epoch,
                     (unsigned long)ds.backpressure, (unsigned long)ds.deadline_miss,
                     (unsigned long)ds.budget90_over, (unsigned long)ds.write_errors,
                     (unsigned long)ds.write_partial,
                     (unsigned long)(ds.frames_undelivered * i2s_out_get_sample_stride_bytes()),
                     (unsigned long)ds.frames_in, (unsigned long)ds.frames_stage,
                     (unsigned long)ds.frames_undelivered, (unsigned long)ds.stage_pending_bytes,
                     (unsigned long)ds.deadline_us);
            ESP_LOGI(TAG, "c1max: prep=%lu q=%lu dsp=%lu port=%lu total=%lu uac=%lu bt=%lu byp=%lu | win prep=%lu q=%lu dsp=%lu port=%lu total=%lu n=%lu",
                     (unsigned long)ds.prep_max_us, (unsigned long)ds.queue_max_us,
                     (unsigned long)ds.dsp_max_us, (unsigned long)ds.port_max_us,
                     (unsigned long)ds.total_max_us, (unsigned long)ds.total_max_uac_us,
                     (unsigned long)ds.total_max_bt_us, (unsigned long)ds.total_max_bypass_us,
                     (unsigned long)ds.win_prep_max_us, (unsigned long)ds.win_queue_max_us,
                     (unsigned long)ds.win_dsp_max_us, (unsigned long)ds.win_port_max_us,
                     (unsigned long)ds.win_total_max_us, (unsigned long)ds.win_blocks);
            ESP_LOGI(TAG, "c1avg: prep=%lu q=%lu dsp=%lu port=%lu total=%lu n=%lu",
                     (unsigned long)(ds.win_blocks ? (uint32_t)(ds.prep_sum_us / ds.win_blocks) : 0),
                     (unsigned long)(ds.win_blocks ? (uint32_t)(ds.queue_sum_us / ds.win_blocks) : 0),
                     (unsigned long)(ds.win_blocks ? (uint32_t)(ds.dsp_sum_us / ds.win_blocks) : 0),
                     (unsigned long)(ds.win_blocks ? (uint32_t)(ds.port_sum_us / ds.win_blocks) : 0),
                     (unsigned long)(ds.win_blocks ? (uint32_t)(ds.total_sum_us / ds.win_blocks) : 0),
                     (unsigned long)ds.win_blocks);
            if (legacy) {
                ESP_LOGI(TAG, "observe-dma: sent=%lu notify_full=%lu pump=%lu tail_skip=%lu tail_b=%lu tail_at=%lu queue_free=%lu",
                         (unsigned long)ist.sent_events, (unsigned long)ist.notify_full_events,
                         (unsigned long)s_ctx.pump_runs, (unsigned long)ist.tail_skip_candidates,
                         (unsigned long)ist.last_tail_bytes, (unsigned long)ist.last_tail_us,
                         (unsigned long)ist.last_queue_free);
                ESP_LOGI(TAG, "observe-stage: full_blocks=%lu backend_partial=%lu pending=%lu",
                         (unsigned long)ist.full_blocks, (unsigned long)ist.backend_partial,
                         (unsigned long)ist.stage_pending_bytes);
            }
            /* ★ 0.103（DS-04A）观测行：**独立新行**，不改上面任何既有 diag/observe
             * 行的字段顺序与口径（解析量具不受影响）。计数是累计单调的，直接读；
             * 原子性限制见文件上方 s_isr_* 注释块。irq_core=4294967295 表示回调
             * 从未执行过。格式化为 diag 任务侧（非 ISR），无浮点。 */
            /* ★ 0.104 观测行：**独立新行**，不改上面任何既有 diag/observe 行的字段
             * 顺序与口径。`usbisr:` 只保留仍然存在的标量（0.103 的包长/间隔直方图随
             * 340 B 累加观测一起退场，见文件上方 s_rx_* 注释块）。格式化为 diag 任务
             * 侧（非 ISR），无浮点。原子性限制见该注释块。 */
            ESP_LOGI(TAG, "uac-rx: eq=%lu rejected=%lu rejected_b=%lu other=%lu rebase=%lu reported_b=%lu written_b=%lu",
                     (unsigned long)s_rx_d_eq, (unsigned long)s_rx_d_zero_n,
                     (unsigned long)s_rx_drop_b, (unsigned long)s_rx_d_other,
                     (unsigned long)s_rx_rebase, (unsigned long)s_rx_reported,
                     (unsigned long)s_rx_written);
            if (legacy) {
                ESP_LOGI(TAG, "usbisr: n=%lu bytes=%lu irq_core=%lu gap_n=%lu gap_max=%lu gap_max_at=%lu anom_n=%lu anom_first=%lu anom_first_at=%lu",
                         (unsigned long)s_ctx.usb_pkts, (unsigned long)s_observe_rx_bytes,
                         (unsigned long)s_observe_irq_core,
                         (unsigned long)s_isr_gap_n, (unsigned long)s_isr_gap_max_us,
                         (unsigned long)s_isr_gap_max_at_us, (unsigned long)s_isr_gap_anom,
                         (unsigned long)s_isr_gap_first_us,
                         (unsigned long)s_isr_gap_first_at_us);
                uac_print_rx_sequence();
                /* ★ 0.106 只读 DWC2 状态（任务侧读，**不写 W1C**）：粘滞位回答"设备侧
                 * 是否出现过 ISO OUT 传输未完成"; SOFFN 计数回答"设备时间轴上少了多少个
                 * 微帧"。自检字段 cmode/speed 必须分别为 0/0，否则本行结论标 invalid。 */
                {
                    const uint32_t gintsts = uac_dwc2_rd(UAC_DWC2_GINTSTS_REG);
                    const uint32_t dsts_d = uac_dwc2_rd(UAC_DWC2_DSTS_REG);
                    s_dwc2_dsts_last = dsts_d;
                    if ((gintsts & (UAC_DWC2_GINTSTS_INCOMPISOIN |
                                    UAC_DWC2_GINTSTS_INCOMPISOOUT)) != 0u
                        && s_dwc2_gintsts_first == 0u) {
                        s_dwc2_gintsts_first = gintsts;
                        s_dwc2_gintsts_first_at_us = (uint32_t)esp_timer_get_time();
                    }
                    s_dwc2_gintsts_iso_seen =
                        (gintsts & (UAC_DWC2_GINTSTS_INCOMPISOIN |
                                    UAC_DWC2_GINTSTS_INCOMPISOOUT));
                    ESP_LOGI(TAG, "dwc2: gintsts=0x%08lx cmode=%lu oepint=%lu iepint=%lu rxflvl=%lu iso_in_sticky=%lu iso_out_sticky=%lu first=0x%08lx first_at=%lu dsts=0x%08lx soffn=%lu speed=%lu soffn_n=%lu soffn_miss_net=%lu soffn_df_sum=%lu soffn_zero=%lu soffn_bad=%lu soffn_max=%lu soffn_first_at=%lu",
                             (unsigned long)gintsts,
                             (unsigned long)((gintsts & UAC_DWC2_GINTSTS_CMODE) ? 1u : 0u),
                             (unsigned long)((gintsts & UAC_DWC2_GINTSTS_OEPINT) ? 1u : 0u),
                             (unsigned long)((gintsts & UAC_DWC2_GINTSTS_IEPINT) ? 1u : 0u),
                             (unsigned long)((gintsts & UAC_DWC2_GINTSTS_RXFLVL) ? 1u : 0u),
                             (unsigned long)((gintsts & UAC_DWC2_GINTSTS_INCOMPISOIN) ? 1u : 0u),
                             (unsigned long)((gintsts & UAC_DWC2_GINTSTS_INCOMPISOOUT) ? 1u : 0u),
                             (unsigned long)s_dwc2_gintsts_first,
                             (unsigned long)s_dwc2_gintsts_first_at_us,
                             (unsigned long)dsts_d,
                             (unsigned long)((dsts_d & UAC_DWC2_DSTS_SOFFN_MASK) >>
                                             UAC_DWC2_DSTS_SOFFN_SHIFT),
                             (unsigned long)((dsts_d & UAC_DWC2_DSTS_ENUMSPD_MASK) >> 1),
                             (unsigned long)s_soffn_n,
                             (unsigned long)((s_soffn_df_sum >= s_soffn_n)
                                             ? (s_soffn_df_sum - s_soffn_n) : 0u),
                             (unsigned long)s_soffn_df_sum,
                             (unsigned long)s_soffn_zero_n,
                             (unsigned long)s_soffn_bad, (unsigned long)s_soffn_max_df,
                             (unsigned long)s_soffn_first_miss_at_us);
                    /* ★ 0.108 EP1 OUT DOEPINT：设备侧"是否丢掉过 ISOC OUT 包"的粘滞标志 */
                    const uint32_t doepint1 = uac_dwc2_rd(UAC_DWC2_DOEPINT1_REG);
                    s_ep1_doepint_last = doepint1;
                    if ((doepint1 & (UAC_DWC2_DOEPINT_PKTDRPSTS |
                                     UAC_DWC2_DOEPINT_OUTPKTERR)) != 0u
                        && s_ep1_doepint_first == 0u) {
                        s_ep1_doepint_first = doepint1;
                        s_ep1_doepint_first_at_us = (uint32_t)esp_timer_get_time();
                    }
                    s_ep1_pktdrp_seen = (doepint1 & UAC_DWC2_DOEPINT_PKTDRPSTS) ? 1u : 0u;
                    s_ep1_outpkterr_seen = (doepint1 & UAC_DWC2_DOEPINT_OUTPKTERR) ? 1u : 0u;
                    ESP_LOGI(TAG, "dwc2-ep: doepint1=0x%08lx pktdrpsts=%lu outpkterr=%lu xfrc_now=%lu first=0x%08lx first_at=%lu",
                             (unsigned long)doepint1,
                             (unsigned long)s_ep1_pktdrp_seen,
                             (unsigned long)s_ep1_outpkterr_seen,
                             (unsigned long)((doepint1 & UAC_DWC2_DOEPINT_XFRC) ? 1u : 0u),
                             (unsigned long)s_ep1_doepint_first,
                             (unsigned long)s_ep1_doepint_first_at_us);
                }
            }
            /* ★ `pmax` / `pn` / `busy` 是**窗口量**：复制出去之后在这里清零，
             * 使下一条 diag 报的是**这 1 s 窗口内**的值。core1 的窗口量在
             * dsp_audio_reset_block_stats() 里清（0xA4 遥测每次调用）。
             * ⚠ `def_silent` / `deadline_miss` 等是**累计量**，**不清零**。 */
            s_ctx.p_max_us = 0;
            s_ctx.p_n = 0;
            s_ctx.p_sum_us = 0;
              uint32_t eng_resets = 0, eng_rebuilds = 0, eng_calls = 0;
              dsp_engine_get_diag(&eng_resets, &eng_rebuilds, &eng_calls);
              uint32_t eng_jb = 0, eng_jm = 0;
              dsp_engine_get_jumps(&eng_jb, &eng_jm);
              /* 0.113：`direct` 恒 0 —— 所有 PCM 都经 core1 的格式转换与端口输出。
               * B5（RAM 诊断）只绕 DSP 引擎，不绕 core1 输出，见 engine_bypass。 */
              ESP_LOGI(TAG, "dsp-path: direct=0 engine_bypass=%d core1_out=1",
                       s_dsp_test_bypass ? 1 : 0);
              ESP_LOGI(TAG, "dsp: en=%d master=%.1f filt=%lu bypass=%d eng_reset=%lu eng_rebuild=%lu eng_calls=%lu injump=%lu/%lu avg_frames=%lu blocks=%lu uac=%lu bt=%lu test=%lu clip=%lu peak_l=%lu peak_r=%lu last_us=%lu stack=%lu",
                       dsp_audio_is_enabled() ? 1 : 0, (double)dsp_config_master_gain_db(),
                       (unsigned long)dsp_engine_active_filter_count(),
                       dsp_engine_is_bypass() ? 1 : 0,
                       (unsigned long)eng_resets, (unsigned long)eng_rebuilds,
                       (unsigned long)eng_calls,
                       (unsigned long)eng_jb, (unsigned long)eng_jm,
                       (unsigned long)(ds.blocks ? (ds.samples / ds.blocks / 2) : 0),
                       (unsigned long)ds.blocks, (unsigned long)ds.uac_blocks,
                       (unsigned long)ds.bt_blocks, (unsigned long)ds.test_blocks,
                       (unsigned long)ds.clipped, (unsigned long)ds.max_peak_l,
                       (unsigned long)ds.max_peak_r, (unsigned long)ds.last_us,
                       (unsigned long)dsp_audio_get_stack_hwm());
            uint32_t up_l = s_ctx.usb_peak_l;
            uint32_t up_r = s_ctx.usb_peak_r;
            uint32_t ip_l = 0;
            uint32_t ip_r = 0;
            s_ctx.usb_peak_l = 0;
            s_ctx.usb_peak_r = 0;
            /* 输出侧峰值由 core1 在转换前那一次扫描里给出（与旧位置同一数据）。 */
            dsp_audio_take_output_peaks(&ip_l, &ip_r);
            ESP_LOGI(TAG, "peaks: usb L/R %.2f/%.2f dBFS, i2s L/R %.2f/%.2f dBFS (delta %.2f dB)",
                     (double)uac_peak_db_f(up_l), (double)uac_peak_db_f(up_r),
                     (double)uac_peak_db_f(ip_l), (double)uac_peak_db_f(ip_r),
                     (double)(uac_peak_db_f(ip_l) - uac_peak_db_f(up_l)));
            ESP_LOGI(TAG, "volume: master %d dB, L %d dB, R %d dB | mute master/L/R %u/%u/%u",
                     s_volume[0] / 256, s_volume[1] / 256, s_volume[2] / 256,
                     s_mute[0], s_mute[1], s_mute[2]);
            ESP_LOGI(TAG, "usb: mounted=%d speed=%s i2s_rate=%lu heap=%lu",
                     tud_audio_mounted() ? 1 : 0,
                     tud_speed_get() == TUSB_SPEED_HIGH ? "HIGH" : (tud_speed_get() == TUSB_SPEED_FULL ? "FULL" : "?"),
                     (unsigned long)i2s_out_get_rate(),
                     (unsigned long)esp_get_free_heap_size());
            ESP_LOGI(TAG, "bt: connected=%d streaming=%d dur=%lums ready=%d busy=%d gate=%d mute=%d ring=%u/%u min=%u max=%u pkt=%lu drop=%lu ovf=%lu flush=%lu guard=%d idle=%lums",
                     ast.connected ? 1 : 0,
                     ast.streaming ? 1 : 0,
                     (unsigned long)ast.duration_ms,
                     ast.decoder_ready ? 1 : 0,
                     ast.codec_busy ? 1 : 0,
                     s_ctx.bt_gate_open ? 1 : 0,
                     0, /* legacy mute field reserved; CFG never mutes UAC */
                     (unsigned)ast.level_bytes, (unsigned)ast.capacity_bytes,
                     (unsigned)ast.min_level_bytes, (unsigned)ast.max_level_bytes,
                     (unsigned long)ast.packets, (unsigned long)ast.drops,
                     (unsigned long)ast.ring_overflows, (unsigned long)ast.flushes,
                     ast.route_hold ? 1 : 0,
                     (unsigned long)((s_ctx.active_source == AUDIO_SOURCE_BT && s_ctx.bt_alive_us != 0)
                                         ? (esp_timer_get_time() - s_ctx.bt_alive_us) / 1000
                                         : 0));
            /* 0.171 evidence line: threshold vs physical ring. The LDAC 96 kHz
             * prebuffer was 76800 B while an 8192-frame ring held only 65536 B,
             * so the gate could never open. Keep this line short and always on. */
            const uint32_t ldac96_prebuf_b =
                bt_level_bytes(a2dp_bt_prebuffer_ms(96000u, 0x0000012Du, 0x00AAu), 96000u);
            const uint32_t now_prebuf_b = (ast.sample_rate >= 8000u && ast.sample_rate <= 96000u)
                ? bt_level_bytes(a2dp_bt_prebuffer_ms(ast.sample_rate, ast.codec_vendor_id,
                                                     ast.codec_codec_id), ast.sample_rate) : 0u;
            const uint32_t now_high_b = (ast.sample_rate >= 8000u && ast.sample_rate <= 96000u)
                ? bt_level_bytes(a2dp_bt_level_high_ms(ast.sample_rate, ast.codec_vendor_id,
                                                       ast.codec_codec_id), ast.sample_rate) : 0u;
            ESP_LOGI(TAG, "bt-policy: fw=%d ring_b=%u ldac96_prebuf_b=%u rate=%u prebuf_b=%u high_b=%u",
                     DSP_FW_VERSION_MINOR, (unsigned)ast.capacity_bytes,
                     (unsigned)ldac96_prebuf_b, (unsigned)ast.sample_rate,
                     (unsigned)now_prebuf_b, (unsigned)now_high_b);
            ESP_LOGI(TAG, "bt-flow: cfg_fail=%lu epoch_reject=%lu flush_b=%lu prep_trim_blk=%lu prep_trim_fr=%lu play_trim_blk=%lu play_trim_fr=%lu delivered=%lu drain_read=%lu drain_blk=%lu gap_last=%lums gap_max=%lums starve_last=%lums starve_max=%lums starve_n=%lu autostart=%lu as_bits=%lu as_ch=%lu",
                     (unsigned long)ast.cfg_failures, (unsigned long)ast.epoch_rejects,
                     (unsigned long)ast.flush_bytes, (unsigned long)ast.prepare_trim_blocks,
                     (unsigned long)ast.prepare_trim_frames, (unsigned long)ast.play_trim_blocks,
                     (unsigned long)ast.play_trim_frames, (unsigned long)s_ctx.bt_delivered_blocks, (unsigned long)ast.drain_read_bytes, (unsigned long)s_ctx.bt_drain_blocks, (unsigned long)s_ctx.bt_last_gap_ms, (unsigned long)s_ctx.bt_max_gap_ms, (unsigned long)s_ctx.bt_last_starve_ms, (unsigned long)s_ctx.bt_max_starve_ms, (unsigned long)s_ctx.bt_starve_events, (unsigned long)ast.stream_autostart, (unsigned long)ast.autostart_bits, (unsigned long)ast.autostart_channels);
            /* ★ 0.167：tick 级摘要在这里（ctrl 任务）打印，绝不在 pump 里打 */
            bt_trace_report();
            /* All ring terms below share ast's single PCM-lock snapshot.
             * The log prefix is PRINT time; us is SNAPSHOT time. read_b means
             * removed from the BT ring, not proven on-wire audio. */
            ESP_LOGI(TAG, "bt-ring: us=%lu epoch=%lu head_b=%lu tail_b=%lu level_b=%u read_b=%lu flush_b=%lu prep_fr=%lu play_fr=%lu cut_blk=%lu cut_fr=%lu",
                     (unsigned long)ast.snapshot_us, (unsigned long)ast.epoch,
                     (unsigned long)ast.head_bytes, (unsigned long)ast.tail_bytes,
                     (unsigned)ast.level_bytes, (unsigned long)ast.read_bytes,
                     (unsigned long)ast.flush_bytes, (unsigned long)ast.prepare_trim_frames,
                     (unsigned long)ast.play_trim_frames, (unsigned long)ast.submit_stale_blocks,
                     (unsigned long)ast.submit_stale_frames);
            /* 0.127（工单 11 §2.1/§2.3/§3.3）：拒收按理由细分 + ACL 迟读证据 + 厂商 codec。
             * 单独一行，避免上面那行过长被串口 200B 截断。
             * 理由顺序见 a2dp_audio.h 的 a2dp_reject_reason_t：
             *   1未连接 2未推流 3配置等待 4配置失败 5格式长度 6代次失效 7环满 */
            ESP_LOGI(TAG, "bt-rej: total=%lu raw_b=%lu conv_b=%lu r1=%lu r2=%lu r3=%lu r4=%lu r5=%lu r6=%lu r7=%lu last_r=%lu last_b=%lu cb_ep=%lu cur_ep=%lu",
                     (unsigned long)ast.reject_total, (unsigned long)ast.reject_bytes_raw,
                     (unsigned long)ast.reject_bytes_conv,
                     (unsigned long)ast.reject_by_reason[A2DP_REJ_NOT_CONNECTED],
                     (unsigned long)ast.reject_by_reason[A2DP_REJ_NOT_STREAMING],
                     (unsigned long)ast.reject_by_reason[A2DP_REJ_CFG_PENDING],
                     (unsigned long)ast.reject_by_reason[A2DP_REJ_CFG_FAILED],
                     (unsigned long)ast.reject_by_reason[A2DP_REJ_FORMAT_LEN],
                     (unsigned long)ast.reject_by_reason[A2DP_REJ_EPOCH_STALE],
                     (unsigned long)ast.reject_by_reason[A2DP_REJ_RING_FULL],
                     (unsigned long)ast.last_reject_reason, (unsigned long)ast.last_reject_bytes,
                     (unsigned long)ast.last_reject_cb_epoch, (unsigned long)ast.last_reject_cur_epoch);
            /* 0.128（工单 12 §B.2）：短行输出**当前** peer/up/handle 与累计计数，
             * 以及最近断开的身份与 reason；peer 用完整地址，缺失即明显可见。
             * 失败连接也留 peer/stat（在 GAP 回调里另打），便于异 peer 归因。 */
            ESP_LOGI(TAG, "bt-acl: inc=%lu ok=%lu fail=%lu disc=%lu other_peer=%lu up=%d handle=%u cur=%02x:%02x:%02x:%02x:%02x:%02x audio=%02x:%02x:%02x:%02x:%02x:%02x last_disc=%02x:%02x:%02x:%02x:%02x:%02x h=%u reason=0x%04x cfg_apply=%lu vid=0x%08lx cid=0x%04x",
                     (unsigned long)ast.acl_incarnation, (unsigned long)ast.acl_conn_ok,
                     (unsigned long)ast.acl_conn_fail, (unsigned long)ast.acl_disconn,
                     (unsigned long)ast.acl_other_peer, ast.acl_up ? 1 : 0,
                     (unsigned)ast.acl_handle,
                     ast.acl_peer[0], ast.acl_peer[1], ast.acl_peer[2],
                     ast.acl_peer[3], ast.acl_peer[4], ast.acl_peer[5],
                     ast.audio_peer[0], ast.audio_peer[1], ast.audio_peer[2],
                     ast.audio_peer[3], ast.audio_peer[4], ast.audio_peer[5],
                     ast.last_acl_disc_peer[0], ast.last_acl_disc_peer[1], ast.last_acl_disc_peer[2],
                     ast.last_acl_disc_peer[3], ast.last_acl_disc_peer[4], ast.last_acl_disc_peer[5],
                     (unsigned)ast.last_acl_disc_handle,
                     (unsigned)ast.last_acl_disc_reason, (unsigned long)ast.cfg_apply_count,
                     (unsigned long)ast.codec_vendor_id, (unsigned)ast.codec_codec_id);
        }
    }
}

/* ---------------- TinyUSB task and callbacks ---------------- */

static void tusb_device_task(void *arg)
{
    (void)arg;
    while (1) {
        tud_task();
    }
}

void tud_mount_cb(void)
{
    uac_usb_session_reset();
    ESP_LOGI(TAG, "USB mounted at %s", tud_speed_get() == TUSB_SPEED_HIGH ? "HIGH" : "FULL");
}

void tud_umount_cb(void)
{
    uac_usb_session_reset();
    ESP_LOGI(TAG, "USB unmounted");
}

void tud_suspend_cb(bool remote_wakeup_en)
{
    (void)remote_wakeup_en;
    portENTER_CRITICAL(&s_uac_buffer_mux);
    s_ctx.bus_suspended = true;
    s_ctx.streaming = false;
    s_ctx.have_audio = false;
    s_uac_buffer.playing = false;
    s_uac_buffer.wait_us = 0;
    const unsigned alt = s_ctx.stream_alt;
    portEXIT_CRITICAL(&s_uac_buffer_mux);
    /* Bus suspension is not alt0 or a format change. Retain the bounded FIFO
     * and any old-tail boundary, but do not consume them while suspended. */
    ESP_LOGI(TAG, "USB suspended (alt=%u)", alt);
}

void tud_resume_cb(void)
{
    portENTER_CRITICAL(&s_uac_buffer_mux);
    s_ctx.bus_suspended = false;
    s_ctx.streaming = s_ctx.stream_alt != 0 && tud_audio_mounted();
    s_ctx.have_audio = false;
    s_uac_buffer.playing = false;
    s_uac_buffer.wait_us = 0;
    const bool pending = s_ctx.format_request_seq != s_ctx.format_done_seq;
    const unsigned alt = s_ctx.stream_alt, streaming = s_ctx.streaming;
    portEXIT_CRITICAL(&s_uac_buffer_mux);
    if (pending && s_ctx.ctrl_task != NULL) xTaskNotify(s_ctx.ctrl_task, EVT_FORMAT, eSetBits);
    ESP_LOGI(TAG, "USB resumed (alt=%u stream=%u)", alt, streaming);
}

bool tud_audio_rx_done_isr(uint8_t rhport, uint16_t n_bytes_received, uint8_t func_id,
                           uint8_t ep_out, uint8_t cur_alt_setting)
{
    (void)rhport;
    (void)func_id;
    (void)ep_out;
    (void)cur_alt_setting;
    s_ctx.usb_pkts++;
    /* Input activity is independent of selection, amplitude and output hold. */
    if (n_bytes_received >= (s_ctx.bit_depth == 24 ? 6u : 8u)) s_ctx.pcm_active_runs++;
    s_observe_rx_bytes += n_bytes_received;
    if (s_observe_irq_core == UINT32_MAX) s_observe_irq_core = xPortGetCoreID();
    /* ★ 0.104 producer 侧观测（见文件上方 s_rx_* 注释块）：Δwr 账 + 生产者锚点。
     * 本回调在 DCD 重新武装**之后**执行（phase A §2）⇒ 这里只读游标与环内数据，
     * 不改变音频行为；观测开销也不会延长"未武装窗口"。 */
    const uint32_t now_us = (uint32_t)esp_timer_get_time();
    const bool legacy = (s_legacy_diag_mode & 1u) != 0;
    s_rx_reported += n_bytes_received;
    if (s_rx_irq_core == UINT32_MAX) s_rx_irq_core = xPortGetCoreID();
    if (legacy) {
        const uint32_t prev_us = s_isr_prev_us;
        if (prev_us != 0) {
            const uint32_t gap = now_us - prev_us;        /* 无符号：跨回绕正确 */
            s_isr_gap_n++;
            if (gap < UAC_RX_GAP_VALID_US) {
                /* ★ 由完成间隔**推算**该间隔内本应到来的微帧数（125 µs/微帧）。
                 * 这是"服务太晚 ⇒ 端点未武装 ⇒ 整包被硬件吞掉"这一机制的**定量预测**：
                 * 与锚点实测删除包数比较即可判定该机制是否足以解释丢失。 */
                const uint32_t exp = (gap + 62u) / 125u;
                if (exp >= 2u) {
                    s_rx_late_n++;
                    s_rx_miss_expect += (exp - 1u);
                }
            } else {
                s_rx_stop_gap_n++;                        /* 启动/停流：不算音频丢失 */
            }
            if (gap >= UAC_ISR_GAP_ANOM_US) {
                if (s_isr_gap_anom == 0) {
                    s_isr_gap_first_us = gap;
                    s_isr_gap_first_at_us = now_us;
                }
                s_isr_gap_anom++;
            }
            if (gap > s_isr_gap_max_us) {
                s_isr_gap_max_us = gap;
                s_isr_gap_max_at_us = now_us;
            }
        }
        s_isr_prev_us = now_us;

        /* ★ 0.107 只读 DSTS.SOFFN：设备侧微帧号（bits 21:8）。
         * **缺失包数 = ΣΔSOFFN − 样本数**（而不是 Σmax(0,Δ−1)）：
         * 前者对"完成被推迟一个微帧"(Δ=2 后紧跟 Δ=0) 天然抵消，只有真正少掉的微帧
         * 才会留下净 +1；后者会把单纯的推迟也算成丢包。只在 <5 ms 的间隔上累计。 */
        {
            const uint32_t dsts_now = uac_dwc2_rd(UAC_DWC2_DSTS_REG);
            const uint32_t soffn = (dsts_now & UAC_DWC2_DSTS_SOFFN_MASK) >>
                                   UAC_DWC2_DSTS_SOFFN_SHIFT;
            if (s_soffn_prev_valid && prev_us != 0) {
                const uint32_t df = (soffn - s_soffn_prev) & UAC_DWC2_DSTS_SOFFN_VALUE;
                const uint32_t gapv = now_us - prev_us;
                if (gapv < UAC_RX_GAP_VALID_US && df < UAC_DWC2_SOFFN_MAX_DF) {
                    s_soffn_n++;
                    s_soffn_df_sum += df;
                    if (df == 0u) s_soffn_zero_n++;
                    if (df > s_soffn_max_df) s_soffn_max_df = df;
                    if (s_soffn_df_sum > s_soffn_n && s_soffn_first_miss_at_us == 0u) {
                        s_soffn_first_miss_at_us = now_us;
                    }
                } else {
                    s_soffn_bad++;      /* 停流/不同步：两边都不计（不制造假缺失） */
                }
            }
            s_soffn_prev = soffn;
            s_soffn_prev_valid = true;
        }
    }

    const bool rx_tape = uac_rx_content_begin_packet();
    tu_fifo_t *ff = tud_audio_get_ep_out_ff();
    if (ff == NULL) {
        if (rx_tape) uac_rx_content_gap(RX_GAP_NULL);
        s_rx_d_other++;
        s_rx_win_ok = false;
    } else {
        const uint16_t wr = ff->wr_idx;               /* 单次 16 位读 */
        const uint32_t span = (uint32_t)ff->depth * 2u;
        if (!s_rx_prev_valid || span == 0u) {
            /* 首包：只建立基线（不参与 eq/other 判定） */
            if (rx_tape) uac_rx_content_gap(RX_GAP_DELTA);
        } else {
            /* ★ 真正的模 2*depth 推进量。wr_idx 由 advance_index() 维护在 [0,2*depth)
             * 且回绕时加 UINT16_MAX-(2*depth-1)（tusb_fifo.c:420-431）⇒ **不能**用
             * `(uint16_t)(wr - prev)` 当推进量（0.104 首版就是这里错，把每次环回绕
             * 都误判成 rebase，导致所有锚点窗被作废）。 */
            const uint32_t cur = (uint32_t)wr % span;
            const uint32_t prv = (uint32_t)s_rx_prev_wr % span;
            const uint32_t delta = (cur + span - prv) % span;
            if (delta > UAC_RX_DELTA_MAX) {
                s_rx_rebase++;                        /* 清环/改配置/异常大跳 ⇒ unknown */
                s_rx_win_ok = false;
                if (rx_tape) uac_rx_content_gap(RX_GAP_REBASE);
            } else if (delta == (uint32_t)n_bytes_received && n_bytes_received > 0u) {
                s_rx_d_eq++;
                s_rx_written += delta;
                uac_buffer_rx_committed(ff);
                if (legacy) uac_rx_observe_packet(ff, wr, n_bytes_received, now_us);
                /* 0.132 RX_POST_ENQUEUE：同一位置、同一包，读**已入环**字节做内容边界观察。
                 * 只读 ff->buffer，不改 wr/rd_idx、不消费、不动音频样本。 */
                if (rx_tape) uac_rx_content_scan_packet(ff, wr, n_bytes_received, now_us);
            } else if (delta == 0u && n_bytes_received > 0u) {
                if (s_rx_d_zero_n == 0) {
                    s_rx_first_zero_b = n_bytes_received;
                    s_rx_first_zero_at_us = now_us;
                }
                s_rx_d_zero_n++;                      /* 整包被拒收（dcd_dwc2.c:942） */
                s_rx_drop_b += n_bytes_received;
                s_rx_win_ok = false;
                if (rx_tape) uac_rx_content_gap(RX_GAP_REJECT);
            } else {
                if (s_rx_d_other == 0) {
                    s_rx_other_first_n = n_bytes_received;
                    s_rx_other_first_delta = delta;
                    s_rx_other_first_us = now_us;
                }
                s_rx_d_other++;
                s_rx_win_ok = false;
                if (rx_tape) uac_rx_content_gap(RX_GAP_DELTA);
            }
        }
        s_rx_prev_wr = wr;
        s_rx_prev_valid = true;
    }
    return true;
}

void tud_audio_feedback_params_cb(uint8_t func_id, uint8_t alt_itf,
                                  audio_feedback_params_t *feedback_param)
{
    (void)func_id;
    (void)alt_itf;
    /* Built-in FIFO_COUNT regulator. The pump drains the FIFO at the I2S
     * clock rate, which is the intended operating mode for this method. */
    feedback_param->method = AUDIO_FEEDBACK_METHOD_FIFO_COUNT;
    feedback_param->sample_freq = s_ctx.requested_rate;
    feedback_param->fifo_count.fifo_threshold = UAC_FB_TARGET_BYTES;
}

/* ---------------- UAC2 clock-source requests ---------------- */

static bool uac_clock_get_request(uint8_t rhport, audio_control_request_t const *request)
{
    TU_ASSERT(request->bEntityID == UAC2_ENTITY_CLOCK);

    if (request->bControlSelector == AUDIO_CS_CTRL_SAM_FREQ) {
        if (request->bRequest == AUDIO_CS_REQ_CUR) {
            audio_control_cur_4_t curf = {
                .bCur = (int32_t)tu_htole32(s_ctx.requested_rate),
            };
            return tud_audio_buffer_and_schedule_control_xfer(
                rhport, (tusb_control_request_t const *)request, &curf, sizeof(curf));
        }

        if (request->bRequest == AUDIO_CS_REQ_RANGE) {
            audio_control_range_4_n_t(UAC_N_RATES) rangef;
            memset(&rangef, 0, sizeof(rangef));
            rangef.wNumSubRanges = tu_htole16(UAC_N_RATES);
            for (uint8_t i = 0; i < UAC_N_RATES; i++) {
                rangef.subrange[i].bMin = (int32_t)tu_htole32(s_sample_rates[i]);
                rangef.subrange[i].bMax = (int32_t)tu_htole32(s_sample_rates[i]);
                rangef.subrange[i].bRes = 0;
            }
            return tud_audio_buffer_and_schedule_control_xfer(
                rhport, (tusb_control_request_t const *)request, &rangef, sizeof(rangef));
        }
    } else if (request->bControlSelector == AUDIO_CS_CTRL_CLK_VALID &&
               request->bRequest == AUDIO_CS_REQ_CUR) {
        audio_control_cur_1_t cur_valid = { .bCur = 1 };
        return tud_audio_buffer_and_schedule_control_xfer(
            rhport, (tusb_control_request_t const *)request, &cur_valid, sizeof(cur_valid));
    }

    return false;
}

static bool uac_clock_set_request(uint8_t rhport, audio_control_request_t const *request,
                                  uint8_t const *buf)
{
    (void)rhport;
    TU_ASSERT(request->bEntityID == UAC2_ENTITY_CLOCK);
    TU_VERIFY(request->bRequest == AUDIO_CS_REQ_CUR);

    if (request->bControlSelector != AUDIO_CS_CTRL_SAM_FREQ) {
        return false;
    }

    TU_VERIFY(request->wLength == sizeof(audio_control_cur_4_t));
    uint32_t target = (uint32_t)((audio_control_cur_4_t const *)buf)->bCur;
    ESP_LOGI(TAG, "clock SET_CUR: %lu Hz", (unsigned long)target);

    for (uint8_t i = 0; i < UAC_N_RATES; i++) {
        if (s_sample_rates[i] == target) {
            /* Always publish, including a return to the current physical rate:
             * it may cancel a different request whose ACK is still pending. */
            uac_request_rate(target);
            return true;
        }
    }

    ESP_LOGW(TAG, "unsupported clock rate %lu Hz -> STALL", (unsigned long)target);
    return false;
}

/* ---------------- Feature unit requests ---------------- */

static bool uac_feature_unit_get_request(uint8_t rhport, audio_control_request_t const *request)
{
    TU_ASSERT(request->bEntityID == UAC2_ENTITY_SPK_FEATURE_UNIT);
    uint8_t ch = request->bChannelNumber;

    if (request->bControlSelector == AUDIO_FU_CTRL_MUTE &&
        request->bRequest == AUDIO_CS_REQ_CUR && ch <= 2) {
        audio_control_cur_1_t mute1 = { .bCur = s_mute[ch] };
        return tud_audio_buffer_and_schedule_control_xfer(
            rhport, (tusb_control_request_t const *)request, &mute1, sizeof(mute1));
    }

    if (request->bControlSelector == AUDIO_FU_CTRL_VOLUME && ch <= 2) {
        if (request->bRequest == AUDIO_CS_REQ_CUR) {
            audio_control_cur_2_t cur_vol = { .bCur = tu_htole16(s_volume[ch]) };
            return tud_audio_buffer_and_schedule_control_xfer(
                rhport, (tusb_control_request_t const *)request, &cur_vol, sizeof(cur_vol));
        }
        if (request->bRequest == AUDIO_CS_REQ_RANGE) {
            audio_control_range_2_n_t(1) range_vol = {
                .wNumSubRanges = tu_htole16(1),
                .subrange[0] = {
                    .bMin = tu_htole16(-12800), /* -50 dB */
                    .bMax = tu_htole16(0),
                    .bRes = tu_htole16(256),    /* 1 dB */
                },
            };
            return tud_audio_buffer_and_schedule_control_xfer(
                rhport, (tusb_control_request_t const *)request, &range_vol, sizeof(range_vol));
        }
    }

    return false;
}

static bool uac_feature_unit_set_request(uint8_t rhport, audio_control_request_t const *request,
                                         uint8_t const *buf)
{
    (void)rhport;
    TU_ASSERT(request->bEntityID == UAC2_ENTITY_SPK_FEATURE_UNIT);
    TU_VERIFY(request->bRequest == AUDIO_CS_REQ_CUR);

    uint8_t ch = request->bChannelNumber;

    if (request->bControlSelector == AUDIO_FU_CTRL_MUTE && ch <= 2) {
        TU_VERIFY(request->wLength == sizeof(audio_control_cur_1_t));
        s_mute[ch] = ((audio_control_cur_1_t const *)buf)->bCur;
        ESP_LOGI(TAG, "mute ch%u: %u", ch, s_mute[ch]);
        return true;
    }

    if (request->bControlSelector == AUDIO_FU_CTRL_VOLUME && ch <= 2) {
        TU_VERIFY(request->wLength == sizeof(audio_control_cur_2_t));
        int16_t v = (int16_t)((audio_control_cur_2_t const *)buf)->bCur;
        if (v < -12700) v = -12700; /* hard-mute below -50 dB */
        if (v > 0) v = 0;
        s_volume[ch] = v;
        ESP_LOGI(TAG, "volume ch%u: %d dB", ch, s_volume[ch] / 256);
        return true;
    }

    return false;
}

bool tud_audio_get_req_entity_cb(uint8_t rhport, tusb_control_request_t const *p_request)
{
    audio_control_request_t const *request = (audio_control_request_t const *)p_request;

    if (request->bEntityID == UAC2_ENTITY_CLOCK) {
        return uac_clock_get_request(rhport, request);
    }
    if (request->bEntityID == UAC2_ENTITY_SPK_FEATURE_UNIT) {
        return uac_feature_unit_get_request(rhport, request);
    }
    return false;
}

bool tud_audio_set_req_entity_cb(uint8_t rhport, tusb_control_request_t const *p_request,
                                  uint8_t *buf)
{
    audio_control_request_t const *request = (audio_control_request_t const *)p_request;

    if (request->bEntityID == UAC2_ENTITY_CLOCK) {
        return uac_clock_set_request(rhport, request, buf);
    }
    if (request->bEntityID == UAC2_ENTITY_SPK_FEATURE_UNIT) {
        return uac_feature_unit_set_request(rhport, request, buf);
    }
    return false;
}

/* ---------------- Streaming interface selection ---------------- */

bool tud_audio_set_itf_cb(uint8_t rhport, tusb_control_request_t const *p_request)
{
    (void)rhport;
    uint8_t const itf = tu_u16_low(tu_le16toh(p_request->wIndex));
    uint8_t const alt = tu_u16_low(tu_le16toh(p_request->wValue));

    if (itf != ITF_NUM_AUDIO_STREAMING) {
        return false;
    }

    if (alt > 2) return false;
    portENTER_CRITICAL(&s_uac_buffer_mux);
    s_ctx.stream_alt = alt;
    s_ctx.format_input_busy = false;
    s_ctx.streaming = alt != 0 && !s_ctx.bus_suspended;
    s_ctx.have_audio = false;
    if (alt != 0) s_ctx.active_silence_runs = 0;
    const bool pending = s_ctx.format_request_seq != s_ctx.format_done_seq;
    portEXIT_CRITICAL(&s_uac_buffer_mux);
    if (pending && s_ctx.ctrl_task != NULL) xTaskNotify(s_ctx.ctrl_task, EVT_FORMAT, eSetBits);

    switch (alt) {
    case 0:
        /* The pre-close hook retained eligible old bytes. The pump drains
         * that exact boundary before qualifying the next stream; no STOP clear. */
        ESP_LOGI(TAG, "AS alt0: stream closed");
        break;
    case 1:
        ESP_LOGI(TAG, "AS alt1: 24-bit stream opened");
        break;
    case 2:
        ESP_LOGI(TAG, "AS alt2: 32-bit stream opened");
        break;
    }

    return true;
}

bool tud_audio_set_itf_close_ep_cb(uint8_t rhport, tusb_control_request_t const *p_request)
{
    (void)rhport;
    (void)p_request;
    return true;
}

/* ---------------- Descriptor callbacks ---------------- */

uint8_t const *tud_descriptor_device_cb(void)
{
    return (uint8_t const *)&s_desc_device;
}

uint8_t const *tud_descriptor_configuration_cb(uint8_t index)
{
    (void)index;
    return s_desc_configuration;
}

uint8_t const *tud_descriptor_device_qualifier_cb(void)
{
    return (uint8_t const *)&s_desc_qualifier;
}

uint8_t const *tud_descriptor_other_speed_configuration_cb(uint8_t index)
{
    (void)index;
    return s_desc_configuration;
}

uint16_t const *tud_descriptor_string_cb(uint8_t index, uint16_t langid)
{
    (void)langid;
    size_t chr_count = 0;

    switch (index) {
    case STRID_LANGID:
        memcpy(&s_desc_str[1], s_string_desc[0], 2);
        chr_count = 1;
        break;
    default:
        if (index >= TU_ARRAY_SIZE(s_string_desc)) {
            return NULL;
        }
        const char *str = s_string_desc[index];
        chr_count = strlen(str);
        if (chr_count > 32) {
            chr_count = 32;
        }
        for (size_t i = 0; i < chr_count; i++) {
            s_desc_str[1 + i] = (uint16_t)(uint8_t)str[i];
        }
        break;
    }

    s_desc_str[0] = (uint16_t)((TUSB_DESC_STRING << 8) | (2 * chr_count + 2));
    return s_desc_str;
}

/* ---------------- USB PHY / init ---------------- */

static void usb_phy_init(void)
{
    usb_phy_config_t phy_conf = {
        .controller = USB_PHY_CTRL_OTG,
        .otg_mode = USB_OTG_MODE_DEVICE,
        .target = USB_PHY_TARGET_INT,
        .otg_speed = USB_PHY_SPEED_HIGH,
    };
    ESP_ERROR_CHECK(usb_new_phy(&phy_conf, &s_ctx.phy_hdl));
}

uint32_t uac2_audio_get_pump_tick(void)
{
    return s_ctx.pump_runs;
}

/* Stage C1 (hardware mute): read-only accessor for the silence counter the
 * pump already keeps. An accessor was preferred over a hook in the audio path,
 * and it lives on the core0/UAC side, so core1 (frozen) stays untouched. */
uint32_t uac2_audio_get_silence_tick(void)
{
    return s_ctx.silence_runs;
}

/* ★ 0.50（L44）H3：给 hw_mute 的"本 tick 有没有真实音频"心跳。
 * 见 `nonaudio_runs` 与 pump 里那处 `if (!has_data) ++` 的长注释。
 * ⚠ 与 `silence_runs` 的分工：`silence_runs` = **只**统计 UAC FIFO 空（BUG-1 分流用的
 *   既有计数器，语义冻结）；本计数器 = "没交付真实音频"，含 BT 门未开/欠载/源切换/
 *   codec 静音。hw_mute 从 0.50 起用**本**计数器判 playing。 */
uint32_t uac2_audio_get_nonaudio_tick(void)
{
    return s_ctx.nonaudio_runs;
}

/* ★ 0.113：hw_mute 的 `playing` 判据 —— **任一路输入真的在送 PCM** 就前进。
 * UAC 侧 = 端点在开且主机正在往 FIFO 里送（全零 PCM 也算流）；
 * BT 侧 = 解码回调交付有效 PCM（含预缓冲期）。只有"连接上了"不算活动。
 * 与输出选中哪一路无关：BT 连接/切编码期间 UAC 持续送流不会被强制静音。 */
uint32_t uac2_audio_get_pcm_active_tick(void)
{
    return s_ctx.pcm_active_runs + a2dp_audio_get_pcm_packets();
}

uint32_t uac2_audio_get_output_rate(void)
{
    return i2s_out_get_rate();
}

void uac2_audio_set_dsp_test_bypass(bool bypass)
{
    s_dsp_test_bypass = bypass;
}

bool uac2_audio_get_dsp_test_bypass(void)
{
    return s_dsp_test_bypass;
}

uac2_audio_source_t uac2_audio_get_output_source(void)
{
    return (s_ctx.active_source == AUDIO_SOURCE_BT) ? UAC2_AUDIO_SOURCE_BT
                                                    : UAC2_AUDIO_SOURCE_UAC;
}

bool uac2_audio_is_streaming(void)
{
    return s_ctx.streaming;
}

uint8_t uac2_audio_get_fifo_fill_percent(void)
{
    if (!tud_audio_mounted()) {
        return 0;
    }
    tu_fifo_t *ff = tud_audio_get_ep_out_ff();
    if (ff == NULL) {
        return 0;
    }
    uint16_t depth = tu_fifo_depth(ff);
    if (depth == 0) {
        return 0;
    }
    uint32_t pct = (uint32_t)tud_audio_available() * 100u / depth;
    return (uint8_t)(pct > 100u ? 100u : pct);
}

esp_err_t uac2_audio_init(void)
{
    ESP_RETURN_ON_FALSE(tud_audio_rx_preserve_fifo_abi == 0x01340001u,
                        ESP_ERR_INVALID_VERSION, TAG, "TinyUSB FIFO patch ABI");
    ESP_RETURN_ON_FALSE(tud_audio_fifo_feedback_q15_abi == 0x01400001u,
                        ESP_ERR_INVALID_VERSION, TAG, "apply sdk_patches/0.140_uac_feedback_q15.patch");
    usb_phy_init();
    ESP_RETURN_ON_FALSE(tusb_init(), ESP_FAIL, TAG, "USB init failed");

    /* Force a clean re-enumeration. Without this, Windows often keeps the
     * previous USB device state after a flash/reset and never re-reads the
     * configuration descriptor. */
    tud_disconnect();
    vTaskDelay(pdMS_TO_TICKS(120));
    tud_connect();

    BaseType_t ret = xTaskCreatePinnedToCore(tusb_device_task, "tud", 4096, NULL, 5, NULL, 0);
    ESP_RETURN_ON_FALSE(ret == pdPASS, ESP_FAIL, TAG, "create tud task failed");

    /* 0.88: raw input and LA both lost 6+78 frames on connection, FIFO
     * reached 6640/6656 B, pump gap 9839 us. BT controller priority is 23.
     * Let the short DMA-clocked pump preempt controller task work; it still
     * blocks while waiting for frozen core1, and USB reception stays ISR-led. */
    ret = xTaskCreatePinnedToCore(uac_pump_task, "uac_pump", 4096, NULL, 24, NULL, 0);
    ESP_RETURN_ON_FALSE(ret == pdPASS, ESP_FAIL, TAG, "create pump task failed");

    ret = xTaskCreatePinnedToCore(uac_fifo_monitor_task, "uac_mon", 3072, NULL, 10, &s_fifo_monitor_task, 0);
    ESP_RETURN_ON_FALSE(ret == pdPASS, ESP_FAIL, TAG, "create fifo monitor task failed");

    ret = xTaskCreatePinnedToCore(uac_ctrl_task, "uac_ctrl", 4096, NULL, 6, &s_ctx.ctrl_task, 0);
    ESP_RETURN_ON_FALSE(ret == pdPASS, ESP_FAIL, TAG, "create ctrl task failed");

    /* Never disconnect USB on a timer after playback has started. The
     * single initial re-enumeration above precedes creation of the USB task. */

    ESP_LOGI(TAG, "UAC2 ready: stereo speaker, alt1=24bit alt2=32bit, rates 44.1/48/88.2/96k");
    ESP_LOGI(TAG, "async feedback: EP0x%02X, 16.16, FIFO water-level PI controller", UAC_EP_FB);
    return ESP_OK;
}
