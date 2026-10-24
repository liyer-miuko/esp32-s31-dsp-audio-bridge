/* Project-private Classic A2DP codec policy. No LE Audio or DSP ABI. */
#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "esp_a2dp_api.h"
#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    ESP_A2DP_CODEC_SBC, ESP_A2DP_CODEC_AAC, ESP_A2DP_CODEC_APTX,
    ESP_A2DP_CODEC_APTX_HD, ESP_A2DP_CODEC_APTX_LL, ESP_A2DP_CODEC_LHDCV5,
    ESP_A2DP_CODEC_LC3PLUS, ESP_A2DP_CODEC_LDAC,
    ESP_A2DP_CODEC_COUNT, ESP_A2DP_CODEC_NONE = ESP_A2DP_CODEC_COUNT
} esp_a2dp_codec_id_t;

typedef struct {
    esp_a2dp_codec_id_t id;
    const char *name;
    uint32_t vendor_id;
    uint16_t codec_id;
    uint8_t priority;       /* advisory for a peer-initiated configuration */
    bool compiled, experimental;
    const char *evidence;   /* prior evidence, never a live-window PASS */
    int sink_index;         /* this fork's private enum; not a shared ABI */
} esp_a2dp_codec_descriptor_t;

typedef struct {
    esp_a2dp_codec_id_t id;
    uint32_t rate, nominal_frame_us;
    uint8_t channels, input_bits, pcm_bits;
} esp_a2dp_codec_format_t;

typedef struct {
    esp_a2dp_codec_format_t format;
    uint32_t epoch;
    bool ready, configuration_complete;
    uint32_t rx_packets, rx_bytes, queue_full, queue_peak, pressure_drop;
    uint32_t decode_calls, decode_errors, header_errors, config_errors, guard_bad_windows;
    uint32_t wall_budget_windows; /* diagnostic only: wall includes preemption */
    uint32_t guard_cpu_us, guard_cpu_media_us;
    bool guard_cpu_valid;
    uint32_t pcm_callbacks, pcm_frames, pcm_bytes, pcm_alignment_errors;
    uint32_t stale_pcm_callbacks, stale_pcm_bytes; /* decoder output rejected at transport boundary */
    uint64_t decode_net_wall_us, callback_wall_us;
    uint32_t decode_net_max_us, callback_max_us, underrun_blocks;
    uint32_t wdt_during_active; /* observed context; not a root-cause verdict */
    uint32_t workspace_bytes, auxiliary_bytes, scratch_bytes;
    uint32_t heap_before, heap_after, largest_after;
} esp_a2dp_codec_stats_t;

/* Core0-only transport evidence. Retained after decoder cleanup/profile close;
 * counters are since boot, epochs identify the first/last bad packet.
 * No coded audio buffer is retained and no log/BT call runs in the producer. */
typedef enum {
    ESP_A2DP_PACKET_BAD_RTP, ESP_A2DP_PACKET_BAD_MEDIA,
    ESP_A2DP_PACKET_BAD_FRAME, ESP_A2DP_PACKET_DECODE_ERROR,
    ESP_A2DP_PACKET_RESULT_BOUNDS, ESP_A2DP_PACKET_SEQUENCE_GAP,
    ESP_A2DP_PACKET_ERROR_COUNT
} esp_a2dp_packet_error_t;
typedef struct {
    uint32_t packets, wire_bytes, first_epoch, first_len, first4_be;
    uint32_t last_epoch, last_len, last4_be;
    uint32_t errors[ESP_A2DP_PACKET_ERROR_COUNT];
    uint32_t error_epoch, error_len, error4_be, error_kind;
    int32_t error_detail;
} esp_a2dp_packet_stats_t;
void esp_a2dp_codec_policy_packet_wire(esp_a2dp_codec_id_t id,
                                      const uint8_t *wire, size_t len);
void esp_a2dp_codec_policy_packet_error(esp_a2dp_codec_id_t id,
                                       esp_a2dp_packet_error_t kind, int32_t detail);
bool esp_a2dp_codec_policy_packet_snapshot(esp_a2dp_codec_id_t id,
                                          esp_a2dp_packet_stats_t *stats);

void esp_a2dp_codec_policy_init(void);
const esp_a2dp_codec_descriptor_t *esp_a2dp_codec_policy_descriptor(esp_a2dp_codec_id_t id);
esp_a2dp_codec_id_t esp_a2dp_codec_policy_identify(uint8_t type, const uint8_t *cie, size_t len);
bool esp_a2dp_codec_policy_parse(const esp_a2d_mcc_t *mcc, esp_a2dp_codec_format_t *format);
bool esp_a2dp_codec_policy_parse_avdt(const uint8_t *info, esp_a2dp_codec_format_t *format);
bool esp_a2dp_codec_policy_allowed(esp_a2dp_codec_id_t id);
bool esp_a2dp_codec_policy_allowed_avdt(const uint8_t *info);
bool esp_a2dp_codec_policy_allowed_index(int index);
int esp_a2dp_codec_policy_sink_index_at(unsigned ordinal);
int esp_a2dp_codec_policy_priority_avdt(const uint8_t *info);
bool esp_a2dp_codec_policy_quarantine(esp_a2dp_codec_id_t id);
uint32_t esp_a2dp_codec_policy_blocked_mask(void);
/* Core0-private: reject stale configuration faults before changing any SEP. */
bool esp_a2dp_codec_policy_quarantine_current(esp_a2dp_codec_id_t id, uint32_t epoch);
void esp_a2dp_codec_policy_release_quarantine(void);
void esp_a2dp_codec_policy_set_peer(const uint8_t peer[6]);
bool esp_a2dp_codec_policy_boot_fault(uint8_t peer[6], esp_a2dp_codec_id_t *id);
void esp_a2dp_codec_policy_clear_fault_receipt(void);

typedef enum {
    ESP_A2DP_CODEC_FAIL_OVERLOAD, ESP_A2DP_CODEC_FAIL_WDT_CONTEXT,
    ESP_A2DP_CODEC_FAIL_CONFIGURATION
} esp_a2dp_codec_failure_t;
typedef void (*esp_a2dp_codec_failure_cb_t)(esp_a2dp_codec_id_t, uint32_t epoch,
                                           esp_a2dp_codec_failure_t);
/* Callback runs in the existing decoder worker, outside the stats lock.
 * Publish only; recovery/logging belongs to the core0 control task. */
void esp_a2dp_codec_policy_set_failure_callback(esp_a2dp_codec_failure_cb_t callback);
/* Authoritative format lifecycle, synchronous in the one decoder worker.
 * complete=false announces the parsed CIE before decoder setup; success then
 * means CIE-valid, not PCM-ready. complete=true reports final setup success.
 * format is NULL for an invalid CIE. Copy it; never retain this stack pointer.
 * Called outside the policy lock, before any PCM from that configuration.
 * Public AUDIO_CFG notifications may be delayed; they are not this barrier. */
typedef void (*esp_a2dp_codec_configuration_cb_t)(const esp_a2dp_codec_format_t *format,
                                                 uint32_t epoch, bool complete, bool success);
void esp_a2dp_codec_policy_set_configuration_callback(esp_a2dp_codec_configuration_cb_t callback);
void esp_a2dp_codec_policy_worker_config_done(void);
/* Low-priority core0 sampler. CPU is the existing worker task INCLUDING its
 * callbacks, not wall time or a pure kernel benchmark. Identity/window checks
 * are mandatory; invalid/missing runtime stats never become a synthetic zero. */
void esp_a2dp_codec_policy_cpu_window(esp_a2dp_codec_id_t id, uint32_t epoch,
                                     uint64_t cpu_us, uint64_t dt_us,
                                     uint32_t pcm_frames, bool valid);

/* Called only by the existing core0 decoder worker / its PCM callback. */
bool esp_a2dp_codec_policy_begin_config(const uint8_t *info);
void esp_a2dp_codec_policy_config_result(bool ok, uint32_t workspace, uint32_t auxiliary,
                                        uint32_t scratch, uint32_t before,
                                        uint32_t after, uint32_t largest);
bool esp_a2dp_codec_policy_ready(void);
/* Application readiness, after worker setup/scratch allocation has finished. */
bool esp_a2dp_codec_policy_pcm_ready(void);
size_t esp_a2dp_codec_policy_pcm_scratch(void);
void esp_a2dp_codec_policy_rx(uint32_t bytes, bool full, uint32_t queue_depth);
void esp_a2dp_codec_policy_pressure_drop(void);
/* Boundary is published before application state changes. A decode
 * already in progress cannot label its old PCM as the next transport stream. */
void esp_a2dp_codec_policy_stream_boundary(void);
bool esp_a2dp_codec_policy_accept_pcm(uint32_t bytes);
void esp_a2dp_codec_policy_decode_begin(void);
void esp_a2dp_codec_policy_decode_end(uint32_t wall_us, bool ok, uint32_t header_errors);
void esp_a2dp_codec_policy_pcm(uint32_t bytes, uint32_t callback_us);
void esp_a2dp_codec_policy_underrun(uint32_t blocks);
bool esp_a2dp_codec_policy_snapshot(esp_a2dp_codec_id_t id, esp_a2dp_codec_stats_t *stats);
esp_a2dp_codec_id_t esp_a2dp_codec_policy_current(void);
/* ISR-safe observation only; never calls Bluetooth, allocates, feeds or disables WDT. */
void esp_a2dp_codec_policy_wdt_isr(void);
#ifdef __cplusplus
}
#endif
