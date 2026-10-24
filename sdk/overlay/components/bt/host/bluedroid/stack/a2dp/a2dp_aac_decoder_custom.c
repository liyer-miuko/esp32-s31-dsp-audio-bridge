/*
 * SPDX-FileCopyrightText: 2024 ESP32 A2DP Project
 * SPDX-License-Identifier: Apache-2.0
 *
 * A2DP AAC Decoder - Custom Implementation
 * 
 * Integrates lightweight LATM parser with custom AAC-LC decoder
 * for Bluetooth A2DP audio streaming.
 * 
 * Implements tA2DP_DECODER_INTERFACE for use by btc_a2dp_sink.
 */

#include <string.h>
#include <stdbool.h>

#include "common/bt_defs.h"
#include "common/bt_target.h"
#include "osi/allocator.h"
#include "osi/mutex.h"
#include "stack/bt_types.h"
#include "stack/a2dp_codec_api.h"
#include "stack/a2dp_aac.h"
#include "stack/a2dp_rtp_bounds.h"

#include "latm_parser.h"
#include "aac_decoder.h"

#include "esp_log.h"
#include "esp_heap_caps.h"
#include "esp_a2dp_codec_policy.h"

#define TAG "A2DP_AAC_CUSTOM"

/*******************************************************************************
 * Decoder State
 ******************************************************************************/

typedef struct {
    bool initialized;
    latm_parser_t latm;
    aac_decoder_t *decoder;  /* Allocated from internal RAM */
    
    /* Callback to deliver decoded PCM */
    decoded_data_callback_t decode_callback;
    
    /* Configuration from A2DP */
    uint32_t sample_rate;
    uint8_t channels;
    
    /* Statistics */
    uint32_t frames_decoded;
    uint32_t frames_failed;
    
    /* Output buffer for decoded PCM */
    int16_t *pcm_buffer;
    size_t pcm_buffer_size;
    
    /* Raw AAC buffer for LATM extraction */
    uint8_t raw_aac_buffer[1024];
} a2dp_aac_decoder_state_t;

static a2dp_aac_decoder_state_t s_decoder = {0};
static osi_mutex_t s_mutex = NULL;

/* Forward declarations */
void a2dp_aac_decoder_cleanup(void);

/*******************************************************************************
 * Private Functions
 ******************************************************************************/

static void log_memory_info(const char *label)
{
    ESP_LOGI(TAG, "%s - Free internal: %lu, largest: %lu",
             label,
             (unsigned long)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
             (unsigned long)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL));
}

/*******************************************************************************
 * Public API - tA2DP_DECODER_INTERFACE Implementation
 ******************************************************************************/

/**
 * Initialize the AAC decoder
 * Called from btc_a2dp_sink when AAC codec is selected
 */
bool a2dp_aac_decoder_init(decoded_data_callback_t decode_callback)
{
    if (s_decoder.initialized) {
        ESP_LOGW(TAG, "Already initialized, reinitializing");
        a2dp_aac_decoder_cleanup();
    }
    
    if (!decode_callback) {
        ESP_LOGE(TAG, "No callback provided");
        return false;
    }
    
    log_memory_info("Before AAC decoder init");
    
    memset(&s_decoder, 0, sizeof(s_decoder));
    s_decoder.decode_callback = decode_callback;
    
    /* Create mutex */
    if (s_mutex == NULL && osi_mutex_new(&s_mutex) != 0) return false;
    
    /* Allocate decoder from internal RAM */
    s_decoder.decoder = (aac_decoder_t *)heap_caps_malloc(
        sizeof(aac_decoder_t),
        MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT
    );
    
    if (!s_decoder.decoder) {
        ESP_LOGE(TAG, "Failed to allocate decoder (%u bytes)", sizeof(aac_decoder_t));
        return false;
    }
    
    ESP_LOGI(TAG, "Decoder allocated: %u bytes internal RAM", sizeof(aac_decoder_t));
    
    /* Initialize decoder */
    int err = aac_decoder_init(s_decoder.decoder);
    if (err != AAC_OK) {
        ESP_LOGE(TAG, "Decoder init failed: %d", err);
        heap_caps_free(s_decoder.decoder);
        s_decoder.decoder = NULL;
        return false;
    }
    
    /* Initialize LATM parser */
    latm_parser_init(&s_decoder.latm);
    
    /* Allocate PCM output buffer (1 frame worth - 1024 samples stereo) */
    s_decoder.pcm_buffer_size = 1024 * 2 * sizeof(int16_t);
    s_decoder.pcm_buffer = (int16_t *)heap_caps_malloc(
        s_decoder.pcm_buffer_size,
        MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT
    );
    
    if (!s_decoder.pcm_buffer) {
        ESP_LOGE(TAG, "Failed to allocate PCM buffer");
        aac_decoder_deinit(s_decoder.decoder); /* release the acquired Helix workspace */
        heap_caps_free(s_decoder.decoder);
        s_decoder.decoder = NULL;
        return false;
    }
    
    s_decoder.frames_decoded = 0;
    s_decoder.frames_failed = 0;
    s_decoder.initialized = true;
    
    log_memory_info("After AAC decoder init");
    
    ESP_LOGI(TAG, "Custom AAC decoder initialized successfully");
    return true;
}

/**
 * Cleanup the AAC decoder
 */
void a2dp_aac_decoder_cleanup(void)
{
    /* Clean acquired resources even if initialization failed halfway. A freed
     * wrapper does not free Helix's separate ~27 KB workspace by itself. */
    if (s_mutex) {
        osi_mutex_lock(&s_mutex, OSI_MUTEX_MAX_TIMEOUT);
    }
    
    if (s_decoder.pcm_buffer) {
        heap_caps_free(s_decoder.pcm_buffer);
        s_decoder.pcm_buffer = NULL;
    }
    
    if (s_decoder.decoder) {
        aac_decoder_deinit(s_decoder.decoder);
        heap_caps_free(s_decoder.decoder);
        s_decoder.decoder = NULL;
    }
    
    latm_parser_reset(&s_decoder.latm);
    
    ESP_LOGI(TAG, "AAC decoder cleaned up. Decoded: %lu, Failed: %lu",
             (unsigned long)s_decoder.frames_decoded,
             (unsigned long)s_decoder.frames_failed);
    
    s_decoder.initialized = false;
    
    if (s_mutex) {
        osi_mutex_unlock(&s_mutex);
    }
}

/**
 * Decode packet header - check for valid AAC frame
 * Returns the header length on success, -1 on error
 */
ssize_t a2dp_aac_decoder_decode_packet_header(BT_HDR *p_buf)
{
    size_t header, payload;
    uint16_t sequence;
    /* CSRC/extension/padding belong to RTP, not LATM. Validate the entire
     * header before advancing; malformed packets never become AAC input. */
    if (!a2dp_rtp_bounds(p_buf, 1, &header, &payload, &sequence)) return -1;
    p_buf->offset += header;
    p_buf->len = payload;
    p_buf->layer_specific = sequence;
    return 0;
}

/**
 * Decode AAC packet and deliver PCM via callback
 */
bool a2dp_aac_decoder_decode_packet(BT_HDR *p_buf, unsigned char *buf, size_t buf_len)
{
    (void)buf;
    (void)buf_len;
    
    if (!s_decoder.initialized || !s_decoder.decoder || !p_buf) {
        ESP_LOGW(TAG, "decode_packet: not initialized or null buffer");
        return false;
    }
    
    if (!s_decoder.decode_callback) {
        ESP_LOGE(TAG, "No decode callback!");
        return false;
    }
    
    osi_mutex_lock(&s_mutex, OSI_MUTEX_MAX_TIMEOUT);
    
    bool success = false;
    uint8_t *data = (uint8_t *)(p_buf + 1) + p_buf->offset;
    size_t data_len = p_buf->len;
    size_t raw_aac_len = 0;
    
    /* Log every 100 packets */
    static uint32_t pkt_count = 0;
    pkt_count++;
    
    /* The packet-header stage already validated and removed RTP. */
    uint8_t *latm_data = data;
    size_t latm_len = data_len;

    /* Parse LATM wrapper to extract raw AAC frame */
    latm_error_t latm_result = latm_parse_frame(&s_decoder.latm, latm_data, latm_len,
                                                 s_decoder.raw_aac_buffer, 
                                                 sizeof(s_decoder.raw_aac_buffer),
                                                 &raw_aac_len, NULL);
    
    if (latm_result != LATM_OK) {
        if (latm_result == LATM_ERR_NOT_ENOUGH_DATA) {
            success = true;  /* Not a fatal error */
        } else {
            if (pkt_count <= 10 || pkt_count % 100 == 0) {
                ESP_LOGW(TAG, "LATM parse error: %d (pkt %lu)", latm_result, (unsigned long)pkt_count);
            }
            s_decoder.frames_failed++;
        }
        goto done;
    }
    
    if (pkt_count == 1) {
        ESP_LOGI(TAG, "LATM extracted %d bytes raw AAC", (int)raw_aac_len);
    }
    
    /* A negotiated AAC-LC stream cannot silently change its container/rate
     * underneath the app's PCM contract. New configuration requires AVDTP. */
    const latm_audio_config_t *audio_cfg = latm_get_audio_config(&s_decoder.latm);
    if (!audio_cfg || !audio_cfg->config_valid || audio_cfg->object_type != 2 ||
        audio_cfg->frame_length_flag != 0 || audio_cfg->sample_rate != s_decoder.sample_rate ||
        audio_cfg->channel_config != s_decoder.channels) {
        s_decoder.frames_failed++;
        goto done;
    }

    /* Decode raw AAC frame */
    int samples_out = 0;
    int err = aac_decoder_decode(s_decoder.decoder,
                                  s_decoder.raw_aac_buffer, raw_aac_len,
                                  s_decoder.pcm_buffer, &samples_out);
    
    if (err != AAC_OK) {
        if (pkt_count <= 10 || pkt_count % 100 == 0) {
            ESP_LOGW(TAG, "AAC decode error: %d (pkt %lu)", err, (unsigned long)pkt_count);
        }
        s_decoder.frames_failed++;
        goto done;
    }
    
    if (samples_out > 0) {
        /* Calculate output size */
        uint8_t ch = s_decoder.channels > 0 ? s_decoder.channels : 2;
        size_t pcm_bytes = (size_t)samples_out * ch * sizeof(int16_t);
        if (pcm_bytes > s_decoder.pcm_buffer_size) {
            s_decoder.frames_failed++;
            goto done; /* do not truncate and publish a false complete frame */
        }
        
        /* Debug: Log first few samples to verify data */
        if (s_decoder.frames_decoded < 5) {
            int16_t *pcm = s_decoder.pcm_buffer;
            ESP_LOGI(TAG, "PCM[0..3]: %d %d %d %d (bytes=%d)", 
                     pcm[0], pcm[1], pcm[2], pcm[3], (int)pcm_bytes);
        }
        
        /* Deliver PCM to callback */
        s_decoder.decode_callback((uint8_t *)s_decoder.pcm_buffer, pcm_bytes);
        s_decoder.frames_decoded++;
        
        if (s_decoder.frames_decoded == 1) {
            ESP_LOGI(TAG, "Decoded %lu frames, %d samples/frame, failed=%lu",
                     (unsigned long)s_decoder.frames_decoded,
                     (int)samples_out,
                     (unsigned long)s_decoder.frames_failed);
        }
        success = true;
    }
    
done:
    osi_mutex_unlock(&s_mutex);
    return success;
}

/**
 * Configure decoder from A2DP codec info
 */
void a2dp_aac_decoder_configure(const uint8_t *p_codec_info)
{
    if (!s_decoder.initialized || !s_decoder.decoder || !p_codec_info) {
        return;
    }
    
    const unsigned caps = MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT;
    const uint32_t before = heap_caps_get_free_size(caps);
    esp_a2dp_codec_format_t format = {0};
    const bool valid = esp_a2dp_codec_policy_parse_avdt(p_codec_info, &format) &&
        format.id == ESP_A2DP_CODEC_AAC && format.channels >= 1 && format.channels <= 2;
    const bool ok = valid && aac_decoder_configure(s_decoder.decoder, format.rate, format.channels) == AAC_OK;
    if (ok) {
        s_decoder.sample_rate = format.rate; s_decoder.channels = format.channels;
        latm_parser_reset(&s_decoder.latm);
        s_decoder.frames_decoded = 0; s_decoder.frames_failed = 0;
    }
    esp_a2dp_codec_policy_config_result(ok, 0, sizeof(s_decoder), 0, before,
        heap_caps_get_free_size(caps), heap_caps_get_largest_free_block(caps));
    ESP_LOGI(TAG, "AVDTP config rate=%lu ch=%u ok=%u", (unsigned long)format.rate, format.channels, ok);
}
