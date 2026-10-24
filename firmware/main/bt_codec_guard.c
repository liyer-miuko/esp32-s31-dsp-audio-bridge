#include <string.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "esp_attr.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "esp_a2dp_codec_policy.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "bt_codec_guard.h"
#include "a2dp_audio.h"
typedef enum { GUARD_IDLE, GUARD_OBSERVE, GUARD_WAIT_DISCONNECT,
               GUARD_BACKOFF, GUARD_WAIT_CONNECT } guard_phase_t;
static const char *phase_name[] = {"IDLE","OBSERVE","WAIT_DISCONNECT","BACKOFF","WAIT_CONNECT"};
static const char *TAG = "bt-codec";
/* This is a disconnected recovery cooldown, not a decode-budget change. */
#define QUARANTINE_COOLDOWN_US (30LL * 1000000)
static portMUX_TYPE guard_mux = portMUX_INITIALIZER_UNLOCKED;
static struct {
    bool initialized, profile_ready, connected, can_reconnect;
    uint8_t peer[6];
    guard_phase_t phase;
    uint32_t generation, handled_epoch, failures, stale_faults;
    esp_a2dp_codec_id_t handled_id;
    esp_a2dp_codec_failure_t last_reason;
    int64_t deadline_us, quarantine_until_us;
} session;
static struct {
    bool valid;
    esp_a2dp_codec_id_t id;
    uint32_t epoch;
    esp_a2dp_codec_failure_t reason;
} pending_fault;
static volatile uint32_t all_wdt_events;
static int64_t last_poll_us, last_state_us, epoch_start_us;
static esp_a2dp_codec_stats_t baseline;
static bool baseline_valid, catalog_printed;
static uint32_t last_silence, underrun_rebases;

typedef struct { uint64_t cpu_us, dt_us; uint32_t pct; bool valid; } worker_cost_t;
static worker_cost_t worker_cost(uint32_t epoch) {
    worker_cost_t cost = {0};
#if configUSE_TRACE_FACILITY && configGENERATE_RUN_TIME_STATS && INCLUDE_xTaskGetHandle && CONFIG_FREERTOS_RUN_TIME_STATS_USING_ESP_TIMER
    static TaskHandle_t prior_handle;
    static configRUN_TIME_COUNTER_TYPE prior_cpu;
    static int64_t prior_us;
    static uint32_t prior_epoch, prior_number;
    TaskHandle_t handle = xTaskGetHandle("A2DP_DECODER");
    if (!handle) { prior_handle = NULL; return cost; }
    TaskStatus_t task = {0};
    /* Low-priority core0 control samples the existing core0 worker. No stack
     * scan, all-task allocation, new counter hook or change to scheduling. */
    vTaskGetInfo(handle, &task, pdFALSE, eInvalid);
    const int64_t now = esp_timer_get_time();
    if (handle == prior_handle && task.xTaskNumber == prior_number &&
        epoch == prior_epoch && now > prior_us) {
        cost.cpu_us = (configRUN_TIME_COUNTER_TYPE)(task.ulRunTimeCounter - prior_cpu);
        cost.dt_us = (uint64_t)(now - prior_us);
        cost.pct = (uint32_t)(cost.cpu_us * 100u / cost.dt_us);
        cost.valid = cost.cpu_us <= cost.dt_us;
    }
    prior_handle = handle; prior_number = task.xTaskNumber;
    prior_cpu = task.ulRunTimeCounter; prior_us = now; prior_epoch = epoch;
#endif
    return cost;
}

static bool any_codec_allowed(void) {
    for (unsigned i=0;i<ESP_A2DP_CODEC_COUNT;++i)
        if (esp_a2dp_codec_policy_allowed(i)) return true;
    return false;
}
static const char *failure_name(esp_a2dp_codec_failure_t reason) {
    return reason == ESP_A2DP_CODEC_FAIL_CONFIGURATION ? "CONFIG_FAILED" :
           reason == ESP_A2DP_CODEC_FAIL_WDT_CONTEXT ? "WDT_CONTEXT" : "SUSTAINED_OVERLOAD";
}
/* The SDK media worker publishes one bounded, epoch-bound fault. The control
 * task consumes it; neither this callback nor the WDT ISR calls Bluetooth. */
static void worker_failure(esp_a2dp_codec_id_t id, uint32_t epoch,
                           esp_a2dp_codec_failure_t reason) {
    portENTER_CRITICAL(&guard_mux);
    pending_fault.id = id; pending_fault.epoch = epoch;
    pending_fault.reason = reason; pending_fault.valid = true;
    portEXIT_CRITICAL(&guard_mux);
}

void IRAM_ATTR esp_task_wdt_isr_user_handler(void) {
    all_wdt_events++;
    esp_a2dp_codec_policy_wdt_isr();
}

void bt_codec_guard_init(void) {
    esp_a2dp_codec_policy_init();
#if CONFIG_BT_A2DP_CODEC_GUARD
    esp_a2dp_codec_policy_set_failure_callback(worker_failure);
#endif
    uint8_t peer[6]; esp_a2dp_codec_id_t failed;
    bool fault = esp_a2dp_codec_policy_boot_fault(peer, &failed);
    bool available = any_codec_allowed();
    const int64_t now = esp_timer_get_time();
    portENTER_CRITICAL(&guard_mux);
    session.initialized = true; session.phase = GUARD_IDLE;
    session.handled_id = ESP_A2DP_CODEC_NONE;
    session.can_reconnect = available;
    if (fault) {
        memcpy(session.peer, peer, 6);
        session.phase = available ? GUARD_BACKOFF : GUARD_IDLE;
        session.deadline_us = now + 1000000;
        session.quarantine_until_us = now + QUARANTINE_COOLDOWN_US;
    }
    portEXIT_CRITICAL(&guard_mux);
    if (fault) ESP_LOGW(TAG, "boot_wdt codec=%s blocked=0x%lx context_only=1",
        esp_a2dp_codec_policy_descriptor(failed)->name,
        (unsigned long)esp_a2dp_codec_policy_blocked_mask());
}
void bt_codec_guard_profile_ready(void) {
    portENTER_CRITICAL(&guard_mux); session.profile_ready = true; portEXIT_CRITICAL(&guard_mux);
}
void bt_codec_guard_connection(const uint8_t peer[6], esp_a2d_connection_state_t state) {
    const int64_t now = esp_timer_get_time();
    const bool available = any_codec_allowed();
    portENTER_CRITICAL(&guard_mux);
    if (state == ESP_A2D_CONNECTION_STATE_CONNECTED) {
        memcpy(session.peer, peer, 6); session.connected = true;
        session.generation++; session.phase = GUARD_OBSERVE;
        session.can_reconnect = available;
    } else if (state == ESP_A2D_CONNECTION_STATE_DISCONNECTED && session.connected &&
               memcmp(session.peer, peer, 6) == 0) {
        session.connected = false; session.generation++;
        if (session.phase == GUARD_WAIT_DISCONNECT && session.can_reconnect) {
            session.phase = GUARD_BACKOFF; session.deadline_us = now + 500000;
        } else {
            /* Normal disconnect or failed reconnect returns to discoverable
             * idle. It never inherits a permanent WAIT_SOURCE state. */
            session.phase = GUARD_IDLE; session.deadline_us = 0;
        }
    }
    portEXIT_CRITICAL(&guard_mux);
    if (state == ESP_A2D_CONNECTION_STATE_CONNECTED) esp_a2dp_codec_policy_set_peer(peer);
}

static void begin_fallback(esp_a2dp_codec_id_t id, uint32_t epoch,
                           esp_a2dp_codec_failure_t reason, int64_t now) {
    uint8_t peer[6]; uint32_t generation; bool connected; guard_phase_t prior_phase;
    int64_t prior_deadline;
    portENTER_CRITICAL(&guard_mux);
    if ((session.connected && session.phase != GUARD_OBSERVE) ||
        (session.handled_id == id && session.handled_epoch == epoch)) {
        portEXIT_CRITICAL(&guard_mux); return;
    }
    memcpy(peer, session.peer, 6); generation = session.generation;
    connected = session.connected; prior_phase = session.phase; prior_deadline = session.deadline_us;
    session.phase = connected ? GUARD_WAIT_DISCONNECT : GUARD_IDLE;
    session.deadline_us = connected ? now + 8000000 : 0;
    portEXIT_CRITICAL(&guard_mux);

    /* Revalidate inside the SDK stats lock before changing any capability.
     * A stale callback cannot quarantine the current/new codec by name alone. */
    const bool current = esp_a2dp_codec_policy_quarantine_current(id, epoch);
    /* End only the exact failed audio configuration. Release its route hold
     * before requesting disconnect; an idle ACL or failed reconnect cannot
     * reserve output against UAC. A newer app CFG makes this old fault stale. */
    const bool audio_ended = current && a2dp_audio_fail_codec((uint32_t)id, epoch);
    const bool available = any_codec_allowed();
    portENTER_CRITICAL(&guard_mux);
    if (!current || !audio_ended) session.stale_faults++;
    if (current) session.quarantine_until_us = now + QUARANTINE_COOLDOWN_US;
    if (session.generation != generation || session.connected != connected) {
        portEXIT_CRITICAL(&guard_mux); return;
    }
    if (!current || !audio_ended) {
        session.phase = prior_phase;
        session.deadline_us = prior_deadline;
        portEXIT_CRITICAL(&guard_mux); return;
    }
    session.handled_id = id; session.handled_epoch = epoch;
    session.last_reason = reason; session.failures++;
    session.can_reconnect = available;
    session.quarantine_until_us = now + QUARANTINE_COOLDOWN_US;
    portEXIT_CRITICAL(&guard_mux);

    esp_a2dp_codec_stats_t fault_stats;
    if (esp_a2dp_codec_policy_snapshot(id, &fault_stats) && fault_stats.epoch == epoch)
        ESP_LOGW(TAG, "failure_detail codec=%s epoch=%lu ready=%u complete=%u cfgerr=%lu ws=%lu aux=%lu scratch=%lu heap_before=%lu heap_after=%lu largest_after=%lu",
            esp_a2dp_codec_policy_descriptor(id)->name, (unsigned long)epoch,
            fault_stats.ready, fault_stats.configuration_complete,
            (unsigned long)fault_stats.config_errors, (unsigned long)fault_stats.workspace_bytes,
            (unsigned long)fault_stats.auxiliary_bytes, (unsigned long)fault_stats.scratch_bytes,
            (unsigned long)fault_stats.heap_before, (unsigned long)fault_stats.heap_after,
            (unsigned long)fault_stats.largest_after);
    if (!connected) {
        ESP_LOGW(TAG, "fallback codec=%s epoch=%lu reason=%s profile_connected=0 hold_released=1 no_disconnect_request=1",
            esp_a2dp_codec_policy_descriptor(id)->name, (unsigned long)epoch, failure_name(reason));
        return;
    }
    /* A new worker CFG can preempt the failure handling above on the SAME
     * profile. Profile generation alone misses it. Check codec+epoch again
     * at the last safe point before queuing disconnect; no BT API under lock.
     * A request already queued to BTC cannot be retracted by this check. */
    esp_a2dp_codec_stats_t before_disconnect;
    const bool same_codec = esp_a2dp_codec_policy_current() == id &&
        esp_a2dp_codec_policy_snapshot(id, &before_disconnect) &&
        before_disconnect.epoch == epoch && before_disconnect.configuration_complete;
    portENTER_CRITICAL(&guard_mux);
    const bool same_profile = session.generation == generation && session.connected &&
                              session.phase == GUARD_WAIT_DISCONNECT;
    if (!same_codec && same_profile) {
        session.phase = GUARD_OBSERVE;
        session.deadline_us = 0;
        ++session.stale_faults;
    }
    portEXIT_CRITICAL(&guard_mux);
    if (!same_codec || !same_profile) {
        ESP_LOGW(TAG, "fallback_superseded codec=%s epoch=%lu disconnect_not_requested=1",
            esp_a2dp_codec_policy_descriptor(id)->name, (unsigned long)epoch);
        return;
    }
    esp_err_t err = esp_a2d_sink_disconnect(peer);
    ESP_LOGW(TAG, "fallback codec=%s epoch=%lu reason=%s blocked=0x%lx available=%u disconnect_rc=%d source_renegotiation_required=1",
        esp_a2dp_codec_policy_descriptor(id)->name, (unsigned long)epoch, failure_name(reason),
        (unsigned long)esp_a2dp_codec_policy_blocked_mask(), available, err);
    if (err != ESP_OK) {
        portENTER_CRITICAL(&guard_mux);
        if (session.generation == generation && session.phase == GUARD_WAIT_DISCONNECT)
            session.phase = session.connected ? GUARD_OBSERVE : GUARD_IDLE;
        portEXIT_CRITICAL(&guard_mux);
    }
}

static void consume_fault(int64_t now) {
    esp_a2dp_codec_id_t id; uint32_t epoch;
    esp_a2dp_codec_failure_t reason;
    portENTER_CRITICAL(&guard_mux);
    const bool valid = pending_fault.valid;
    id = pending_fault.id; epoch = pending_fault.epoch; reason = pending_fault.reason;
    portEXIT_CRITICAL(&guard_mux);
    if (!valid) return;
    esp_a2dp_codec_stats_t stats;
    const bool current = esp_a2dp_codec_policy_current() == id &&
        esp_a2dp_codec_policy_snapshot(id, &stats) && stats.epoch == epoch;
    /* A failed replacement may never emit CONNECTED. Waiting for it would
     * leave the selected CFG hold forever. Finalized faults can release that
     * audio route even without a live profile; partial setup still waits. */
    if (current && !stats.configuration_complete) return;
    portENTER_CRITICAL(&guard_mux);
    if (pending_fault.valid && pending_fault.id == id && pending_fault.epoch == epoch)
        pending_fault.valid = false;
    if (!current) session.stale_faults++;
    portEXIT_CRITICAL(&guard_mux);
    if (current) begin_fallback(id, epoch, reason, now);
}

/* Playback may already have disconnected before a host can start its serial
 * session. Keep these two target codecs readable then; active-only perf logs
 * otherwise hide precisely the configuration/header/decoder failure needed.
 * All values below are retained totals, NOT deltas or a playback PASS. */
static void log_retained_codec(esp_a2dp_codec_id_t id)
{
    esp_a2dp_codec_stats_t s;
    if (!esp_a2dp_codec_policy_snapshot(id, &s) || !s.epoch) return;
    const char *name = esp_a2dp_codec_policy_descriptor(id)->name;
    ESP_LOGI(TAG, "history name=%s epoch=%lu rate=%lu pcm_bits=%u ready=%u complete=%u cfgerr=%lu rx=%lu calls=%lu pcm=%lu hdrerr=%lu decerr=%lu qfull=%lu pressure=%lu wdt_ctx=%lu ws=%lu aux=%lu scratch=%lu heap=%lu largest=%lu net_wall_us=%llu net_max_us=%lu stale_pcm=%lu scope=LIFETIME_NOT_CONNECTION",
        name, (unsigned long)s.epoch, (unsigned long)s.format.rate, s.format.pcm_bits,
        s.ready, s.configuration_complete, (unsigned long)s.config_errors,
        (unsigned long)s.rx_packets, (unsigned long)s.decode_calls, (unsigned long)s.pcm_frames,
        (unsigned long)s.header_errors, (unsigned long)s.decode_errors, (unsigned long)s.queue_full,
        (unsigned long)s.pressure_drop, (unsigned long)s.wdt_during_active,
        (unsigned long)s.workspace_bytes, (unsigned long)s.auxiliary_bytes,
        (unsigned long)s.scratch_bytes, (unsigned long)s.heap_after, (unsigned long)s.largest_after,
        (unsigned long long)s.decode_net_wall_us, (unsigned long)s.decode_net_max_us,
        (unsigned long)s.stale_pcm_callbacks);
    esp_a2dp_packet_stats_t p;
    if (!esp_a2dp_codec_policy_packet_snapshot(id, &p) || !p.packets) return;
    ESP_LOGI(TAG, "wire-history name=%s packets=%lu wire_b=%lu first_epoch=%lu first_len=%lu first4_be=0x%08lx last_epoch=%lu bad_rtp=%lu bad_media=%lu bad_frame=%lu decode_error=%lu bounds=%lu seq_gap=%lu scope=LIFETIME_NOT_WINDOW",
        name, (unsigned long)p.packets, (unsigned long)p.wire_bytes,
        (unsigned long)p.first_epoch, (unsigned long)p.first_len, (unsigned long)p.first4_be,
        (unsigned long)p.last_epoch, (unsigned long)p.errors[ESP_A2DP_PACKET_BAD_RTP],
        (unsigned long)p.errors[ESP_A2DP_PACKET_BAD_MEDIA], (unsigned long)p.errors[ESP_A2DP_PACKET_BAD_FRAME],
        (unsigned long)p.errors[ESP_A2DP_PACKET_DECODE_ERROR], (unsigned long)p.errors[ESP_A2DP_PACKET_RESULT_BOUNDS],
        (unsigned long)p.errors[ESP_A2DP_PACKET_SEQUENCE_GAP]);
    ESP_LOGI(TAG, "wire-error name=%s error_epoch=%lu error_kind=%lu detail=%ld error_len=%lu error4_be=0x%08lx observed=%u",
        name, (unsigned long)p.error_epoch,
        (unsigned long)p.error_kind, (long)p.error_detail,
        (unsigned long)p.error_len, (unsigned long)p.error4_be, p.error_epoch != 0);
}

void bt_codec_guard_poll(uint32_t silence, bool selected, bool underrun_valid) {
    const int64_t now = esp_timer_get_time();
    consume_fault(now);
    if (now-last_poll_us < 1000000) return;
    const uint64_t elapsed = last_poll_us ? (uint64_t)(now-last_poll_us) : 1000000;
    last_poll_us = now;
    uint8_t peer[6]; guard_phase_t phase; bool connected, ready; int64_t deadline;
    portENTER_CRITICAL(&guard_mux);
    const bool initialized = session.initialized;
    memcpy(peer, session.peer, 6); phase = session.phase;
    connected = session.connected; ready = session.profile_ready; deadline = session.deadline_us;
    portEXIT_CRITICAL(&guard_mux);
    if (!initialized) return;
    if (!catalog_printed) {
        for (unsigned i=0;i<ESP_A2DP_CODEC_COUNT;++i) {
            const esp_a2dp_codec_descriptor_t *d = esp_a2dp_codec_policy_descriptor(i);
            ESP_LOGI(TAG, "cap name=%s enabled=%u priority=%u experimental=%u evidence=%s",
                     d->name, esp_a2dp_codec_policy_allowed(i), d->priority, d->experimental, d->evidence);
        }
        catalog_printed = true;
    }
    /* No unbounded reconnect loop. One fallback attempt; timeouts return to
     * an inbound-connectable state. Quarantine expires only while idle. */
    portENTER_CRITICAL(&guard_mux);
    const bool release = !session.connected && session.phase == GUARD_IDLE &&
        session.quarantine_until_us && now >= session.quarantine_until_us;
    if (release) {
        esp_a2dp_codec_policy_release_quarantine();
        session.quarantine_until_us = 0; session.can_reconnect = true;
    }
    const uint32_t failures = session.failures, stale = session.stale_faults;
    const bool pending = pending_fault.valid, reconnect = session.can_reconnect;
    const uint32_t fault_epoch = session.handled_epoch;
    const esp_a2dp_codec_id_t fault_id = session.handled_id;
    const esp_a2dp_codec_failure_t fault_reason = session.last_reason;
    portEXIT_CRITICAL(&guard_mux);
    if (now-last_state_us >= 5000000 || release) {
        last_state_us = now;
        ESP_LOGI(TAG, "state=%s connected=%u profile=%u blocked=0x%lx available=%u pending=%u faults=%lu stale_faults=%lu last_id=%u last_epoch=%lu last_reason=%s quarantine_expired=%u heap=%lu largest=%lu",
            phase_name[phase], connected, ready,
            (unsigned long)esp_a2dp_codec_policy_blocked_mask(), reconnect, pending,
            (unsigned long)failures, (unsigned long)stale, (unsigned)fault_id,
            (unsigned long)fault_epoch, fault_id == ESP_A2DP_CODEC_NONE ? "NONE" : failure_name(fault_reason), release,
            (unsigned long)heap_caps_get_free_size(MALLOC_CAP_INTERNAL|MALLOC_CAP_8BIT),
            (unsigned long)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL|MALLOC_CAP_8BIT));
        log_retained_codec(ESP_A2DP_CODEC_APTX_HD);
    }
    if (phase == GUARD_WAIT_DISCONNECT || phase == GUARD_WAIT_CONNECT) {
        if (now >= deadline) {
            portENTER_CRITICAL(&guard_mux);
            if (session.phase == phase && session.deadline_us == deadline)
                session.phase = session.connected ? GUARD_OBSERVE : GUARD_IDLE;
            portEXIT_CRITICAL(&guard_mux);
            ESP_LOGW(TAG, "recovery reason=%s_timeout manual_connection_allowed=1 no_auto_retry=1", phase_name[phase]);
        }
        return;
    }
    if (phase == GUARD_BACKOFF) {
        if (!ready || now < deadline) return;
        portENTER_CRITICAL(&guard_mux);
        const bool start = session.phase == GUARD_BACKOFF && !session.connected;
        if (start) {
            session.phase = GUARD_WAIT_CONNECT; session.deadline_us = now + 10000000;
        }
        portEXIT_CRITICAL(&guard_mux);
        if (!start) return;
        esp_err_t err = esp_a2d_sink_connect(peer);
        ESP_LOGW(TAG, "reconnect rc=%d blocked=0x%lx expected=SOURCE_RENEGOTIATION", err,
                 (unsigned long)esp_a2dp_codec_policy_blocked_mask());
        if (err != ESP_OK) {
            portENTER_CRITICAL(&guard_mux);
            if (session.phase == GUARD_WAIT_CONNECT) session.phase = GUARD_IDLE;
            portEXIT_CRITICAL(&guard_mux);
        }
        return;
    }
    esp_a2dp_codec_id_t id = esp_a2dp_codec_policy_current();
    esp_a2dp_codec_stats_t s;
    if (!connected || !esp_a2dp_codec_policy_snapshot(id, &s)) {
        baseline_valid = false; last_silence = silence; return;
    }
    if (!s.configuration_complete) {
        baseline_valid = false; last_silence = silence; return;
    }
#if CONFIG_BT_A2DP_CODEC_GUARD
    /* A failed config may finish before the CONNECTED callback. Its finalized
     * state is checked here too; partial setup is never treated as failure. */
    if (!s.ready && s.config_errors && phase == GUARD_OBSERVE) {
        begin_fallback(id,s.epoch,ESP_A2DP_CODEC_FAIL_CONFIGURATION,now);
        return;
    }
#endif
    const esp_a2dp_codec_descriptor_t *d = esp_a2dp_codec_policy_descriptor(id);
    const worker_cost_t cpu = worker_cost(s.epoch);
    if (!baseline_valid || s.epoch != baseline.epoch || s.format.id != baseline.format.id) {
        baseline = s; baseline_valid = true;
        epoch_start_us = now; last_silence = silence;
        ESP_LOGI(TAG, "config name=%s epoch=%lu rate=%lu ch=%u input_bits=%u pcm_bits=%u nominal_us=%lu ready=%u ws=%lu aux=%lu pcm_scratch=%lu heap=%lu->%lu largest=%lu evidence=%s",
            d->name, (unsigned long)s.epoch, (unsigned long)s.format.rate, s.format.channels,
            s.format.input_bits, s.format.pcm_bits, (unsigned long)s.format.nominal_frame_us,
            s.ready, (unsigned long)s.workspace_bytes, (unsigned long)s.auxiliary_bytes,
            (unsigned long)s.scratch_bytes, (unsigned long)s.heap_before,
            (unsigned long)s.heap_after, (unsigned long)s.largest_after, d->evidence);

        return;
    }
    const uint32_t calls = s.decode_calls-baseline.decode_calls;
    const uint32_t pcm = s.pcm_frames-baseline.pcm_frames;
    esp_a2dp_codec_policy_cpu_window(id, s.epoch, cpu.cpu_us, cpu.dt_us, pcm, cpu.valid);
    const uint32_t rx = s.rx_packets-baseline.rx_packets;
    const uint32_t rx_bytes = s.rx_bytes-baseline.rx_bytes;
    const uint32_t qfull = s.queue_full-baseline.queue_full;
    const uint32_t errors = s.decode_errors-baseline.decode_errors;
    const uint64_t dec_us = s.decode_net_wall_us-baseline.decode_net_wall_us;
    const uint64_t media_us = s.format.rate ? (uint64_t)pcm*1000000u/s.format.rate : 0;
    const uint32_t pct = media_us ? (uint32_t)(dec_us*100u/media_us) : 0;
    const uint32_t equiv = media_us ? (uint32_t)(dec_us*2667u/media_us) : 0;
    const uint32_t wdt = s.wdt_during_active-baseline.wdt_during_active;
    const uint32_t pressure = s.pressure_drop-baseline.pressure_drop;
    const bool stable = now-epoch_start_us >= 2000000;
    uint32_t und = 0;
    if (silence >= last_silence && selected && underrun_valid && stable) und = silence-last_silence;
    else if (silence < last_silence) underrun_rebases++;
    if (und) esp_a2dp_codec_policy_underrun(und);
    last_silence = silence;
    ESP_LOGI(TAG, "perf name=%s epoch=%lu dt_us=%llu rx=%lu rx_b=%lu calls=%lu pcm=%lu pcm_s=%lu dec_avg_call_us=%lu dec_max_call_us=%lu equiv2667_us=%lu busy_pct=%lu qfull=%lu badwin=%lu qpeak=%lu pressure_drop=%lu decerr=%lu wdt_ctx=%lu wdt_total=%lu und=%lu und_rebase=%lu state=%s coverage=%s",
        d->name, (unsigned long)s.epoch, (unsigned long long)elapsed,
        (unsigned long)rx, (unsigned long)rx_bytes, (unsigned long)calls, (unsigned long)pcm,
        (unsigned long)(pcm*1000000ULL/elapsed), (unsigned long)(calls ? dec_us/calls : 0),
        (unsigned long)s.decode_net_max_us, (unsigned long)equiv, (unsigned long)pct,
        (unsigned long)qfull, (unsigned long)s.guard_bad_windows, (unsigned long)s.queue_peak, (unsigned long)pressure, (unsigned long)errors,
        (unsigned long)wdt, (unsigned long)all_wdt_events, (unsigned long)und, (unsigned long)underrun_rebases,
        phase_name[phase], pcm ? "PCM_OBSERVED_NOT_ACCEPTANCE" : "NO_PCM_NOT_TESTED");
    ESP_LOGI(TAG, "flow epoch=%lu pcm_b=%lu cb_wall_us=%llu hdrerr=%lu alignerr=%lu cfgerr_total=%lu blocked=0x%lx heap=%lu largest=%lu worker_cpu_us=%llu worker_dt_us=%llu worker_cpu_pct=%lu worker_cpu_valid=%u worker_scope=TASK_INCLUDES_CALLBACK",
        (unsigned long)s.epoch, (unsigned long)(s.pcm_bytes-baseline.pcm_bytes),
        (unsigned long long)(s.callback_wall_us-baseline.callback_wall_us),
        (unsigned long)(s.header_errors-baseline.header_errors),
        (unsigned long)(s.pcm_alignment_errors-baseline.pcm_alignment_errors),
        (unsigned long)s.config_errors, (unsigned long)esp_a2dp_codec_policy_blocked_mask(),
        (unsigned long)heap_caps_get_free_size(MALLOC_CAP_INTERNAL|MALLOC_CAP_8BIT),
        (unsigned long)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL|MALLOC_CAP_8BIT),
        (unsigned long long)cpu.cpu_us, (unsigned long long)cpu.dt_us,
        (unsigned long)cpu.pct, cpu.valid);
    if (s.wall_budget_windows != baseline.wall_budget_windows)
        ESP_LOGW(TAG, "budget epoch=%lu wall_over_total=%lu guard_cpu_valid=%u guard_cpu_us=%lu guard_media_us=%lu wall_alone_disconnects=0",
            (unsigned long)s.epoch, (unsigned long)s.wall_budget_windows, s.guard_cpu_valid,
            (unsigned long)s.guard_cpu_us, (unsigned long)s.guard_cpu_media_us);
    const uint32_t stale_callbacks = s.stale_pcm_callbacks - baseline.stale_pcm_callbacks;
    const uint32_t stale_bytes = s.stale_pcm_bytes - baseline.stale_pcm_bytes;
    if (stale_callbacks) ESP_LOGW(TAG, "stale_pcm epoch=%lu callbacks=%lu raw_b=%lu transport_boundary=1",
        (unsigned long)s.epoch, (unsigned long)stale_callbacks, (unsigned long)stale_bytes);
    baseline = s;

}
