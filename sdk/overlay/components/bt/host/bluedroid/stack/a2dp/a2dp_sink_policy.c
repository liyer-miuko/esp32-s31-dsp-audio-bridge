#include <string.h>
#include "sdkconfig.h"
#include "esp_attr.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_a2dp_codec_policy.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "stack/a2dp_codec_api.h"
#include "common/bt_target.h"

#if (A2D_INCLUDED == TRUE)
#ifndef CONFIG_BT_A2DP_SBC_SEP
#define CONFIG_BT_A2DP_SBC_SEP 0
#endif
#ifndef CONFIG_BT_A2DP_APTX_CLASSIC_SEP
#define CONFIG_BT_A2DP_APTX_CLASSIC_SEP 0
#endif
#ifndef CONFIG_BT_A2DP_APTX_HD_SEP
#define CONFIG_BT_A2DP_APTX_HD_SEP 0
#endif
#ifndef CONFIG_BT_A2DP_APTX_LL_SEP
#define CONFIG_BT_A2DP_APTX_LL_SEP 0
#endif
#define NO_INDEX (-1)
#if defined(AAC_DEC_INCLUDED) && AAC_DEC_INCLUDED == TRUE
#define AAC_INDEX BTAV_A2DP_CODEC_INDEX_SINK_AAC
#else
#define AAC_INDEX NO_INDEX
#endif
#if defined(APTX_DEC_INCLUDED) && APTX_DEC_INCLUDED == TRUE
#define APTX_INDEX BTAV_A2DP_CODEC_INDEX_SINK_APTX
#define HD_INDEX BTAV_A2DP_CODEC_INDEX_SINK_APTX_HD
#define LL_INDEX BTAV_A2DP_CODEC_INDEX_SINK_APTX_LL
#else
#define APTX_INDEX NO_INDEX
#define HD_INDEX NO_INDEX
#define LL_INDEX NO_INDEX
#endif
#if defined(LHDCV5_DEC_INCLUDED) && LHDCV5_DEC_INCLUDED == TRUE
#define LHDC_INDEX BTAV_A2DP_CODEC_INDEX_SINK_LHDCV5
#else
#define LHDC_INDEX NO_INDEX
#endif
#if defined(LC3PLUS_DEC_INCLUDED) && LC3PLUS_DEC_INCLUDED == TRUE
#define LC3_INDEX BTAV_A2DP_CODEC_INDEX_SINK_LC3PLUS
#else
#define LC3_INDEX NO_INDEX
#endif
#if defined(LDAC_DEC_INCLUDED) && LDAC_DEC_INCLUDED == TRUE && CONFIG_BT_A2DP_LDAC_EXPERIMENTAL
#define LDAC_INDEX BTAV_A2DP_CODEC_INDEX_SINK_LDAC
#else
#define LDAC_INDEX NO_INDEX
#endif

/* One identity/priority source. Enabling a SEP is not a performance claim.
 * Prior verified low-cost aptX family precedes unmeasured vendor extensions.
 * SBC remains the final interoperable fallback. LDAC needs two opt-ins. */
static const esp_a2dp_codec_descriptor_t catalog[ESP_A2DP_CODEC_COUNT] = {
    {ESP_A2DP_CODEC_SBC, "SBC", 0, 0, 10, CONFIG_BT_A2DP_SBC_SEP, false, "PRIOR_FUNCTIONAL", BTAV_A2DP_CODEC_INDEX_SINK_SBC},
    {ESP_A2DP_CODEC_AAC, "AAC", 0, 0, 60, AAC_INDEX >= 0, false, "PRIOR_FUNCTIONAL", AAC_INDEX},
    {ESP_A2DP_CODEC_APTX, "aptX", 0x4f, 1, 80, APTX_INDEX >= 0 && CONFIG_BT_A2DP_APTX_CLASSIC_SEP, false, "PRIOR_LOW_CPU", APTX_INDEX},
    {ESP_A2DP_CODEC_APTX_HD, "aptX-HD", 0xd7, 0x24, 100, HD_INDEX >= 0 && CONFIG_BT_A2DP_APTX_HD_SEP, false, "PRIOR_LOW_CPU", HD_INDEX},
    {ESP_A2DP_CODEC_APTX_LL, "aptX-LL", 0x0a, 2, 90, LL_INDEX >= 0 && CONFIG_BT_A2DP_APTX_LL_SEP, false, "SOURCE_NOT_ASSURED", LL_INDEX},
    {ESP_A2DP_CODEC_LHDCV5, "LHDC-v5", 0x53a, 0x4c35, 70, LHDC_INDEX >= 0, true, "NOT_TESTED", LHDC_INDEX},
    {ESP_A2DP_CODEC_LC3PLUS, "LC3plus-A2DP", 0x8a9, 1, 50, LC3_INDEX >= 0, true, "NOT_TESTED", LC3_INDEX},
    {ESP_A2DP_CODEC_LDAC, "LDAC", 0x12d, 0xaa, 1, LDAC_INDEX >= 0, true, "FAILED_REALTIME", LDAC_INDEX},
};
static const esp_a2dp_codec_id_t order[] = {
    ESP_A2DP_CODEC_APTX_HD, ESP_A2DP_CODEC_APTX_LL, ESP_A2DP_CODEC_APTX,
    ESP_A2DP_CODEC_LHDCV5, ESP_A2DP_CODEC_AAC, ESP_A2DP_CODEC_LC3PLUS,
    ESP_A2DP_CODEC_SBC, ESP_A2DP_CODEC_LDAC
};
static portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
static esp_a2dp_codec_stats_t counters[ESP_A2DP_CODEC_COUNT];
/* Only the two diagnostic targets pay RAM/hot-path cost. */
static esp_a2dp_packet_stats_t packet_counters[2];
static volatile uint32_t active = ESP_A2DP_CODEC_NONE;
static volatile bool inside_decode;
static uint32_t blocked, cfg_epoch, call_callback_us;
static uint32_t transport_generation, call_transport_generation;
static bool config_ok;
static uint8_t peer_address[6];
static volatile uint32_t wdt_counts[ESP_A2DP_CODEC_COUNT];
static bool boot_fault;
static esp_a2dp_codec_id_t boot_codec;
static uint8_t boot_peer[6];
static esp_a2dp_codec_failure_cb_t failure_callback;
static esp_a2dp_codec_configuration_cb_t configuration_callback;
static struct {
    int64_t ready_us, checked_us;
    uint64_t net_us;
    uint32_t pcm, calls, qfull, pressure, seen_wdt;
    uint32_t cpu_serial, cpu_checked_serial;
    int64_t cpu_sample_us;
    bool cpu_over, fired;
} watchdog_window; /* serialized by mux; worker plus low-priority CPU sampler */

/* Reset evidence only. No NVS writes. A hard WDT cannot reconnect from ISR;
 * the next boot consumes this receipt and suppresses the failed preference. */
typedef struct { uint32_t magic, codec, checksum; uint8_t peer[6]; } fault_receipt_t;
static RTC_NOINIT_ATTR volatile fault_receipt_t rtc_fault;
#define FAULT_MAGIC 0x434F4431u

static bool valid_id(esp_a2dp_codec_id_t id) { return (unsigned)id < ESP_A2DP_CODEC_COUNT; }
static esp_a2dp_packet_stats_t *packet_counter(esp_a2dp_codec_id_t id) {
    if (id == ESP_A2DP_CODEC_APTX_HD) return &packet_counters[0];
    if (id == ESP_A2DP_CODEC_LHDCV5) return &packet_counters[1];
    return NULL;
}
void esp_a2dp_codec_policy_packet_wire(esp_a2dp_codec_id_t id,
                                      const uint8_t *wire, size_t len) {
    esp_a2dp_packet_stats_t *s=packet_counter(id);
    if (!s || (!wire && len)) return;
    uint32_t prefix=0;
    for (size_t i=0; i<len && i<4; ++i) prefix=(prefix<<8)|wire[i];
    portENTER_CRITICAL(&mux);
    if (active == (uint32_t)id) {
        ++s->packets; s->wire_bytes+=(uint32_t)len;
        if (s->first_epoch != cfg_epoch) {
            s->first_epoch=cfg_epoch; s->first_len=(uint32_t)len; s->first4_be=prefix;
        }
        s->last_epoch=cfg_epoch; s->last_len=(uint32_t)len; s->last4_be=prefix;
    }
    portEXIT_CRITICAL(&mux);
}
void esp_a2dp_codec_policy_packet_error(esp_a2dp_codec_id_t id,
                                       esp_a2dp_packet_error_t kind, int32_t detail) {
    esp_a2dp_packet_stats_t *s=packet_counter(id);
    if (!s || (unsigned)kind >= ESP_A2DP_PACKET_ERROR_COUNT) return;
    portENTER_CRITICAL(&mux);
    if (active == (uint32_t)id) {
        ++s->errors[kind]; s->error_epoch=cfg_epoch; s->error_kind=kind;
        s->error_detail=detail; s->error_len=s->last_len; s->error4_be=s->last4_be;
    }
    portEXIT_CRITICAL(&mux);
}
bool esp_a2dp_codec_policy_packet_snapshot(esp_a2dp_codec_id_t id,
                                          esp_a2dp_packet_stats_t *s) {
    const esp_a2dp_packet_stats_t *p=packet_counter(id);
    if (!p || !s) return false;
    portENTER_CRITICAL(&mux); *s=*p; portEXIT_CRITICAL(&mux);
    return true;
}
static uint32_t peer_sum(const uint8_t p[6]) {
    return ((uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) |
        ((uint32_t)p[3] << 24)) ^ ((uint32_t)p[4] << 7) ^ ((uint32_t)p[5] << 19);
}

const esp_a2dp_codec_descriptor_t *esp_a2dp_codec_policy_descriptor(esp_a2dp_codec_id_t id) {
    return valid_id(id) ? &catalog[id] : NULL;
}
esp_a2dp_codec_id_t esp_a2dp_codec_policy_identify(uint8_t type, const uint8_t *cie, size_t len) {
    if (type == ESP_A2D_MCT_SBC) return ESP_A2DP_CODEC_SBC;
    if (type == ESP_A2D_MCT_M24) return ESP_A2DP_CODEC_AAC;
    if (type != ESP_A2D_MCT_NON_A2DP || !cie || len < 6) return ESP_A2DP_CODEC_NONE;
    const uint32_t vid = cie[0] | ((uint32_t)cie[1] << 8) |
        ((uint32_t)cie[2] << 16) | ((uint32_t)cie[3] << 24);
    const uint16_t cid = cie[4] | ((uint16_t)cie[5] << 8);
    for (unsigned i = ESP_A2DP_CODEC_APTX; i < ESP_A2DP_CODEC_COUNT; ++i)
        if (catalog[i].vendor_id == vid && catalog[i].codec_id == cid) return i;
    return ESP_A2DP_CODEC_NONE;
}
static bool one(unsigned value) { return value && !(value & (value-1)); }

bool esp_a2dp_codec_policy_parse(const esp_a2d_mcc_t *m, esp_a2dp_codec_format_t *f) {
    if (!m || !f) return false;
    memset(f, 0, sizeof(*f));
    const uint8_t *r = (const uint8_t *)&m->cie;
    f->id = esp_a2dp_codec_policy_identify(m->type, r, sizeof(m->cie));
    f->channels = 2; f->input_bits = f->pcm_bits = 16;
    if (f->id == ESP_A2DP_CODEC_SBC) {
        unsigned sf = m->cie.sbc_info.samp_freq;
        if (!one(sf)) return false;
        if (sf & ESP_A2D_SBC_CIE_SF_48K) f->rate = 48000;
        else if (sf & ESP_A2D_SBC_CIE_SF_44K) f->rate = 44100;
        else if (sf & ESP_A2D_SBC_CIE_SF_32K) f->rate = 32000;
        else if (sf & ESP_A2D_SBC_CIE_SF_16K) f->rate = 16000;
        if (!one(m->cie.sbc_info.ch_mode)) return false;
        f->channels = (m->cie.sbc_info.ch_mode & ESP_A2D_SBC_CIE_CH_MODE_MONO) ? 1 : 2;
    } else if (f->id == ESP_A2DP_CODEC_AAC) {
        unsigned sf = ((unsigned)m->cie.m24_info.samp_freq1 << 4) | m->cie.m24_info.samp_freq2;
        if (!one(sf)) return false;
        const uint32_t rates[] = {8000,11025,12000,16000,22050,24000,32000,44100,48000,64000,88200,96000};
        for (unsigned i=0;i<12;++i) if (sf & (1u << (11-i))) f->rate = rates[i];
        if (!one(m->cie.m24_info.ch)) return false;
        f->channels = (m->cie.m24_info.ch & ESP_A2D_M24_CIE_CH_1) ? 1 : 2;
        f->nominal_frame_us = f->rate ? 1024000000u/f->rate : 0;
    } else if (f->id >= ESP_A2DP_CODEC_APTX && f->id <= ESP_A2DP_CODEC_APTX_LL) {
        const unsigned sf = r[6] & 0xf0;
        if (sf == 0x20) f->rate = 44100;
        else if (sf == 0x10) f->rate = 48000;
        /* This fork's freeaptx callback is always stereo Q1.31. */
        if ((r[6] & 0x0f) != 0x02) return false;
        f->input_bits = f->id == ESP_A2DP_CODEC_APTX_HD ? 24 : 16;
        f->pcm_bits = 32;
    } else if (f->id == ESP_A2DP_CODEC_LDAC) {
        unsigned sf = r[6] & 0x3f;
        if (sf == 0x20) f->rate = 44100;
        else if (sf == 0x10) f->rate = 48000;
        else if (sf == 0x08) f->rate = 88200;
        else if (sf == 0x04) f->rate = 96000;
        if (!one(r[7] & 7)) return false;
        f->channels = (r[7] & 4) ? 1 : 2;
        f->input_bits = f->pcm_bits = 24;
        f->nominal_frame_us = f->rate ? (f->rate > 48000 ? 256000000u : 128000000u)/f->rate : 0;
    } else if (f->id == ESP_A2DP_CODEC_LHDCV5) {
        const unsigned sf = r[6] & 0x35;
        if (sf == 0x20) f->rate = 44100;
        else if (sf == 0x10) f->rate = 48000;
#if CONFIG_BT_A2DP_LHDCV5_96K_EXPERIMENTAL
        else if (sf == 0x04) f->rate = 96000;
#endif
        const unsigned bd = r[7] & 7;
        if (bd == 2) { f->input_bits = 24; f->pcm_bits = 32; }
        else if (bd == 4) { f->input_bits = f->pcm_bits = 16; }
        else return false;
        if ((r[8] & 0x1f) != 0x11 || r[9] || r[10]) return false; /* v1, 5ms only */
        f->nominal_frame_us = 5000;
    } else if (f->id == ESP_A2DP_CODEC_LC3PLUS) {
        const unsigned hz = ((unsigned)r[8] << 8) | r[9];
        if (hz == 0x0100) f->rate = 48000;
        else if (hz == 0x0080) f->rate = 96000;
        if (r[7] == 0x80) f->channels = 1;
        else if (r[7] != 0x40) return false;
        if (r[6] == 0x10) f->nominal_frame_us = 2500;
        else if (r[6] == 0x20) f->nominal_frame_us = 5000;
        else if (r[6] == 0x40) f->nominal_frame_us = 10000;
        else return false;
        f->input_bits = f->pcm_bits = 24; /* explicitly packed S24_3LE */
    } else return false;
    return f->rate >= 8000 && f->rate <= 96000;
}

bool esp_a2dp_codec_policy_parse_avdt(const uint8_t *info, esp_a2dp_codec_format_t *f) {
    if (!info || info[0] < 2 || info[0] >= AVDT_CODEC_SIZE) return false;
    esp_a2d_mcc_t m = {0};
    size_t n = info[0] - 2;
    if (n > sizeof(m.cie)) return false;
    m.type = info[2]; memcpy(&m.cie, info+3, n);
    return esp_a2dp_codec_policy_parse(&m, f);
}
bool esp_a2dp_codec_policy_allowed(esp_a2dp_codec_id_t id) {
    return valid_id(id) && catalog[id].compiled && catalog[id].sink_index >= 0 &&
        !(__atomic_load_n(&blocked, __ATOMIC_RELAXED) & (1u << id));
}
bool esp_a2dp_codec_policy_allowed_avdt(const uint8_t *info) {
    if (!info || info[0] < 2 || info[0] >= AVDT_CODEC_SIZE) return false;
    return esp_a2dp_codec_policy_allowed(esp_a2dp_codec_policy_identify(info[2], info+3, info[0]-2));
}
bool esp_a2dp_codec_policy_allowed_index(int index) {
    for (unsigned i=0;i<ESP_A2DP_CODEC_COUNT;++i)
        if (catalog[i].sink_index == index) return esp_a2dp_codec_policy_allowed(i);
    return false;
}
int esp_a2dp_codec_policy_sink_index_at(unsigned ordinal) {
    unsigned seen = 0;
    for (unsigned i=0;i<sizeof(order)/sizeof(order[0]);++i)
        if (esp_a2dp_codec_policy_allowed(order[i]) && seen++ == ordinal)
            return catalog[order[i]].sink_index;
    return NO_INDEX;
}
int esp_a2dp_codec_policy_priority_avdt(const uint8_t *info) {
    if (!esp_a2dp_codec_policy_allowed_avdt(info)) return -1;
    return catalog[esp_a2dp_codec_policy_identify(info[2], info+3, info[0]-2)].priority;
}
uint32_t esp_a2dp_codec_policy_blocked_mask(void) { return __atomic_load_n(&blocked, __ATOMIC_RELAXED); }
bool esp_a2dp_codec_policy_quarantine(esp_a2dp_codec_id_t id) {
    if (!valid_id(id)) return false;
    /* A priority is a preference, not shared decoder provenance. In particular,
     * an unmeasured LHDC failure says nothing about the three aptX decoders. */
    __atomic_fetch_or(&blocked, 1u << id, __ATOMIC_RELAXED);
    for (unsigned i=0;i<ESP_A2DP_CODEC_COUNT;++i) if (esp_a2dp_codec_policy_allowed(i)) return true;
    return false;
}
bool esp_a2dp_codec_policy_quarantine_current(esp_a2dp_codec_id_t id, uint32_t epoch) {
    portENTER_CRITICAL(&mux);
    const bool current = valid_id(id) && active == (uint32_t)id &&
        counters[id].epoch == epoch && counters[id].configuration_complete;
    if (current) __atomic_fetch_or(&blocked, 1u << id, __ATOMIC_RELAXED);
    portEXIT_CRITICAL(&mux);
    return current;
}
void esp_a2dp_codec_policy_release_quarantine(void) {
    /* Disconnected cooldown only; do not erase any failure/performance counter. */
    __atomic_store_n(&blocked, 0, __ATOMIC_RELAXED);
}
void esp_a2dp_codec_policy_init(void) {
    boot_fault = false;
    if (esp_reset_reason() == ESP_RST_TASK_WDT && rtc_fault.magic == FAULT_MAGIC &&
        rtc_fault.codec < ESP_A2DP_CODEC_COUNT) {
        for (unsigned i=0;i<6;++i) boot_peer[i] = rtc_fault.peer[i];
        if (rtc_fault.checksum == (FAULT_MAGIC ^ rtc_fault.codec ^ peer_sum(boot_peer))) {
            boot_codec = rtc_fault.codec; boot_fault = true;
            esp_a2dp_codec_policy_quarantine(boot_codec);
            wdt_counts[boot_codec] = 1;
        }
    }
    rtc_fault.magic = 0;
}
void esp_a2dp_codec_policy_set_peer(const uint8_t peer[6]) {
    portENTER_CRITICAL(&mux); memcpy(peer_address, peer, 6); portEXIT_CRITICAL(&mux);
}
bool esp_a2dp_codec_policy_boot_fault(uint8_t peer[6], esp_a2dp_codec_id_t *id) {
    if (!boot_fault) return false;
    memcpy(peer, boot_peer, 6); *id = boot_codec; boot_fault = false; return true;
}
void esp_a2dp_codec_policy_clear_fault_receipt(void) { rtc_fault.magic = 0; }
void IRAM_ATTR esp_a2dp_codec_policy_wdt_isr(void) {
    const uint32_t id = active;
    if (id >= ESP_A2DP_CODEC_COUNT || !inside_decode) return;
    wdt_counts[id]++;
    rtc_fault.magic = 0; rtc_fault.codec = id;
    uint32_t sum = (uint32_t)peer_address[0] | ((uint32_t)peer_address[1] << 8) |
        ((uint32_t)peer_address[2] << 16) | ((uint32_t)peer_address[3] << 24);
    sum ^= ((uint32_t)peer_address[4] << 7) ^ ((uint32_t)peer_address[5] << 19);
    for (unsigned i=0;i<6;++i) rtc_fault.peer[i] = peer_address[i];
    rtc_fault.checksum = FAULT_MAGIC ^ id ^ sum;
    rtc_fault.magic = FAULT_MAGIC;
}
void esp_a2dp_codec_policy_set_failure_callback(esp_a2dp_codec_failure_cb_t callback) {
    portENTER_CRITICAL(&mux); failure_callback=callback; portEXIT_CRITICAL(&mux);
}
void esp_a2dp_codec_policy_set_configuration_callback(esp_a2dp_codec_configuration_cb_t callback) {
    portENTER_CRITICAL(&mux); configuration_callback=callback; portEXIT_CRITICAL(&mux);
}
void esp_a2dp_codec_policy_worker_config_done(void) {
    esp_a2dp_codec_failure_cb_t callback=NULL;
    esp_a2dp_codec_configuration_cb_t configured;
    esp_a2dp_codec_format_t format={0};
    bool success=false;
    const int64_t now=esp_timer_get_time();
    esp_a2dp_codec_id_t id;
    uint32_t epoch;
    portENTER_CRITICAL(&mux);
    id=active; epoch=cfg_epoch;
    configured=configuration_callback;
    memset(&watchdog_window,0,sizeof(watchdog_window));
    watchdog_window.ready_us=watchdog_window.checked_us=now;
    if (active < ESP_A2DP_CODEC_COUNT) {
        esp_a2dp_codec_stats_t *s=&counters[active];
        s->configuration_complete=true;
        format=s->format; success=config_ok && s->ready;
        s->guard_bad_windows=0;
        s->guard_cpu_valid=false;
        s->guard_cpu_us=s->guard_cpu_media_us=0;
        watchdog_window.net_us=s->decode_net_wall_us;
        watchdog_window.pcm=s->pcm_frames; watchdog_window.calls=s->decode_calls;
        watchdog_window.qfull=s->queue_full; watchdog_window.pressure=s->pressure_drop;
        watchdog_window.seen_wdt=wdt_counts[active];
#if CONFIG_BT_A2DP_CODEC_GUARD
        if (!s->ready && s->config_errors) {
            watchdog_window.fired=true; callback=failure_callback;
        }
#endif
    }
    portEXIT_CRITICAL(&mux);
    /* No PCM can run between setup and this publication: both are in the
     * existing decoder worker. Never enter the application under mux. */
    if (configured) configured(id < ESP_A2DP_CODEC_COUNT ? &format : NULL,
                               epoch,true,success);
    if (callback) callback(id,epoch,ESP_A2DP_CODEC_FAIL_CONFIGURATION);
}

void esp_a2dp_codec_policy_cpu_window(esp_a2dp_codec_id_t id, uint32_t epoch,
                                     uint64_t cpu_us, uint64_t dt_us,
                                     uint32_t pcm_frames, bool valid) {
    const int64_t now=esp_timer_get_time();
    portENTER_CRITICAL(&mux);
    if (valid_id(id) && active == (uint32_t)id && counters[id].epoch == epoch &&
        counters[id].configuration_complete) {
        esp_a2dp_codec_stats_t *s=&counters[id];
        const uint64_t media=s->format.rate ? (uint64_t)pcm_frames*1000000u/s->format.rate : 0;
        valid=valid && dt_us >= 250000 && dt_us <= 3000000 && cpu_us <= dt_us && media;
        s->guard_cpu_valid=valid;
        s->guard_cpu_us=valid ? (uint32_t)cpu_us : 0;
        s->guard_cpu_media_us=valid ? (uint32_t)media : 0;
        watchdog_window.cpu_over=valid && cpu_us*100u > media*CONFIG_BT_A2DP_CODEC_BUDGET_PCT;
        watchdog_window.cpu_sample_us=now;
        ++watchdog_window.cpu_serial;
    }
    portEXIT_CRITICAL(&mux);
}

bool esp_a2dp_codec_policy_begin_config(const uint8_t *info) {
    esp_a2dp_codec_format_t f={0};
    bool ok = esp_a2dp_codec_policy_allowed_avdt(info) && esp_a2dp_codec_policy_parse_avdt(info, &f);
    portENTER_CRITICAL(&mux);
    config_ok = ok; ++cfg_epoch; call_callback_us = 0;
    if (ok) {
        active = f.id; counters[f.id].format = f; counters[f.id].epoch = cfg_epoch;
        counters[f.id].ready = false;
        counters[f.id].configuration_complete = false;
    } else active = ESP_A2DP_CODEC_NONE;
    const uint32_t epoch=cfg_epoch;
    const esp_a2dp_codec_configuration_cb_t configured=configuration_callback;
    portEXIT_CRITICAL(&mux);
    /* Unlike the independent BTC notification queue, this is an ordered
     * format fence: old decode is finished, new decode has not begun. */
    if (configured) configured(ok ? &f : NULL,epoch,false,ok);
    return ok;
}
void esp_a2dp_codec_policy_config_result(bool ok, uint32_t ws, uint32_t aux, uint32_t scratch,
                                        uint32_t before, uint32_t after, uint32_t largest) {
    portENTER_CRITICAL(&mux);
    if (active < ESP_A2DP_CODEC_COUNT) {
        esp_a2dp_codec_stats_t *s = &counters[active];
        config_ok = ok; s->ready = ok;
        if (!ok) s->config_errors++;
        s->workspace_bytes = ws; s->auxiliary_bytes = aux; s->scratch_bytes = scratch;
        s->heap_before = before; s->heap_after = after; s->largest_after = largest;
    }
    portEXIT_CRITICAL(&mux);
}
bool esp_a2dp_codec_policy_ready(void) {
    /* Worker setup calls this BEFORE worker_config_done; retain that contract. */
    portENTER_CRITICAL(&mux); const bool ready = config_ok; portEXIT_CRITICAL(&mux); return ready;
}
bool esp_a2dp_codec_policy_pcm_ready(void) {
    portENTER_CRITICAL(&mux);
    const bool ready = config_ok && active < ESP_A2DP_CODEC_COUNT &&
        counters[active].ready && counters[active].configuration_complete;
    portEXIT_CRITICAL(&mux); return ready;
}
size_t esp_a2dp_codec_policy_pcm_scratch(void) {
    portENTER_CRITICAL(&mux);
    const esp_a2dp_codec_format_t f = active < ESP_A2DP_CODEC_COUNT ? counters[active].format : (esp_a2dp_codec_format_t){0};
    portEXIT_CRITICAL(&mux);
    if (f.id == ESP_A2DP_CODEC_LHDCV5 && f.rate)
        return (f.rate > 48000 ? 480u : 240u)*f.channels*(f.pcm_bits/8u);
    if (f.id == ESP_A2DP_CODEC_LC3PLUS && f.rate)
        return (uint64_t)f.rate*f.nominal_frame_us/1000000u*f.channels*3u;
    return 0;
}
void esp_a2dp_codec_policy_rx(uint32_t bytes, bool full, uint32_t depth) {
    portENTER_CRITICAL(&mux);
    if (active < ESP_A2DP_CODEC_COUNT) {
        esp_a2dp_codec_stats_t *s = &counters[active];
        if (full) s->queue_full++;
        else { s->rx_packets++; s->rx_bytes += bytes; }
        if (depth > s->queue_peak) s->queue_peak = depth;
    }
    portEXIT_CRITICAL(&mux);
}
void esp_a2dp_codec_policy_pressure_drop(void) {
    portENTER_CRITICAL(&mux);
    if (active < ESP_A2DP_CODEC_COUNT) counters[active].pressure_drop++;
    portEXIT_CRITICAL(&mux);
}
void esp_a2dp_codec_policy_stream_boundary(void) {
    portENTER_CRITICAL(&mux); ++transport_generation; portEXIT_CRITICAL(&mux);
}
bool esp_a2dp_codec_policy_accept_pcm(uint32_t bytes) {
    portENTER_CRITICAL(&mux);
    const bool ok = inside_decode && config_ok && active < ESP_A2DP_CODEC_COUNT &&
        counters[active].ready && counters[active].configuration_complete &&
        call_transport_generation == transport_generation;
    if (!ok && active < ESP_A2DP_CODEC_COUNT) {
        counters[active].stale_pcm_callbacks++;
        counters[active].stale_pcm_bytes += bytes;
    }
    portEXIT_CRITICAL(&mux);
    return ok;
}
void esp_a2dp_codec_policy_decode_begin(void) {
    portENTER_CRITICAL(&mux);
    call_callback_us = 0; call_transport_generation = transport_generation; inside_decode = true;
    portEXIT_CRITICAL(&mux);
}
void esp_a2dp_codec_policy_decode_end(uint32_t wall, bool ok, uint32_t header_errors) {
    const int64_t now=esp_timer_get_time();
    esp_a2dp_codec_failure_cb_t callback=NULL;
    esp_a2dp_codec_failure_t reason=ESP_A2DP_CODEC_FAIL_OVERLOAD;
    esp_a2dp_codec_id_t id;
    uint32_t epoch;
    portENTER_CRITICAL(&mux); inside_decode=false;
    id=active; epoch=cfg_epoch;
    if (active < ESP_A2DP_CODEC_COUNT) {
        esp_a2dp_codec_stats_t *s=&counters[active];
        uint32_t net=wall >= call_callback_us ? wall-call_callback_us : 0;
        s->decode_calls++; s->decode_net_wall_us+=net;
        if (net > s->decode_net_max_us) s->decode_net_max_us=net;
        if (!ok) s->decode_errors++;
        s->header_errors+=header_errors;
#if CONFIG_BT_A2DP_CODEC_GUARD
        if (!watchdog_window.fired && wdt_counts[active] != watchdog_window.seen_wdt) {
            reason=ESP_A2DP_CODEC_FAIL_WDT_CONTEXT;
            watchdog_window.fired=true; callback=failure_callback;
        } else if (!watchdog_window.fired && now-watchdog_window.checked_us >= 1000000) {
            const uint32_t frames=s->pcm_frames-watchdog_window.pcm;
            const uint32_t calls=s->decode_calls-watchdog_window.calls;
            const uint64_t media=s->format.rate ? (uint64_t)frames*1000000u/s->format.rate : 0;
            const uint64_t elapsed=s->decode_net_wall_us-watchdog_window.net_us;
            const bool warm=now-watchdog_window.ready_us >= 2000000;
            const bool wall_over=media && elapsed*100u > media*CONFIG_BT_A2DP_CODEC_BUDGET_PCT;
            if (warm && wall_over) ++s->wall_budget_windows;
            /* Packet wall time includes HCI/UAC/BLE preemption. It may diagnose
             * delay but cannot by itself quarantine a healthy codec. Use the
             * epoch-bound worker CPU/media sample for the configured budget.
             * Queue pressure/no PCM/WDT remain independent safety failures. */
            const bool cpu_new=watchdog_window.cpu_serial != watchdog_window.cpu_checked_serial;
            const bool cpu_valid=s->guard_cpu_valid &&
                now-watchdog_window.cpu_sample_us <= 3000000;
            const bool hard_over=warm && (
                s->queue_full != watchdog_window.qfull || s->pressure_drop != watchdog_window.pressure ||
                (calls && !frames));
            if (hard_over || (warm && cpu_new && cpu_valid && watchdog_window.cpu_over))
                ++s->guard_bad_windows;
            else if (!warm || cpu_new || !cpu_valid)
                s->guard_bad_windows=0;
            /* One stale CPU sample never counts as several bad CPU windows. */
            watchdog_window.cpu_checked_serial=watchdog_window.cpu_serial;
            watchdog_window.checked_us=now; watchdog_window.net_us=s->decode_net_wall_us;
            watchdog_window.pcm=s->pcm_frames; watchdog_window.calls=s->decode_calls;
            watchdog_window.qfull=s->queue_full; watchdog_window.pressure=s->pressure_drop;
            if (s->guard_bad_windows >= CONFIG_BT_A2DP_CODEC_BAD_WINDOWS) {
                watchdog_window.fired=true; callback=failure_callback;
            }
        }
#endif
    }
    portEXIT_CRITICAL(&mux);
    /* No Bluetooth calls/UART on this hot path. The fault belongs to the exact
     * configuration sampled above, even if control consumes it after a switch. */
    if (callback) callback(id,epoch,reason);
}
void esp_a2dp_codec_policy_pcm(uint32_t bytes, uint32_t callback_us) {
    portENTER_CRITICAL(&mux);
    if (active < ESP_A2DP_CODEC_COUNT) {
        esp_a2dp_codec_stats_t *s = &counters[active];
        const unsigned stride = s->format.channels*(s->format.pcm_bits/8u);
        s->pcm_callbacks++; s->pcm_bytes += bytes;
        if (stride && !(bytes % stride)) s->pcm_frames += bytes/stride;
        else s->pcm_alignment_errors++;
        s->callback_wall_us += callback_us;
        if (callback_us > s->callback_max_us) s->callback_max_us = callback_us;
        call_callback_us += callback_us;
    }
    portEXIT_CRITICAL(&mux);
}
void esp_a2dp_codec_policy_underrun(uint32_t blocks) {
    portENTER_CRITICAL(&mux);
    if (active < ESP_A2DP_CODEC_COUNT) counters[active].underrun_blocks += blocks;
    portEXIT_CRITICAL(&mux);
}
bool esp_a2dp_codec_policy_snapshot(esp_a2dp_codec_id_t id, esp_a2dp_codec_stats_t *s) {
    if (!valid_id(id) || !s) return false;
    portENTER_CRITICAL(&mux); *s = counters[id]; s->wdt_during_active = wdt_counts[id]; portEXIT_CRITICAL(&mux);
    return true;
}
esp_a2dp_codec_id_t esp_a2dp_codec_policy_current(void) { return active; }
#endif
