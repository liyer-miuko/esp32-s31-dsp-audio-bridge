/* SPDX-License-Identifier: Apache-2.0 */
#include <errno.h>
#include <string.h>
#include "esp_heap_caps.h"
#include "esp_a2dp_codec_policy.h"
#include "stack/a2dp_vendor_lc3plus_constants.h"
#include "stack/a2dp_vendor_lc3plus_decoder.h"
#include "stack/a2dp_vendor_lc3plus.h"
#include "osi/allocator.h"
#include "lc3.h"

#if defined(LC3PLUS_DEC_INCLUDED) && LC3PLUS_DEC_INCLUDED == TRUE
/* Classic vendor transport only; never instantiates LE Audio/ISO/BAP. */
#define FRAGMENT_BUF_SIZE 2500u /* at most two 1250-byte HR channel frames */
typedef struct {
    uint32_t fragment_size;
    uint8_t fragments_left, channels;
    int rate, duration_us, samples;
    uint8_t fragment[FRAGMENT_BUF_SIZE];
    void *memory[2];
    lc3_decoder_t decoder[2];
    decoded_data_callback_t callback;
} lc3plus_cb_t;
static lc3plus_cb_t cb;

bool a2dp_lc3plus_decoder_init(decoded_data_callback_t callback) {
    cb.callback = callback;
    return callback != NULL;
}
void a2dp_lc3plus_decoder_cleanup(void) {
    for (unsigned ch=0;ch<2;++ch) {
        if (cb.memory[ch]) osi_free(cb.memory[ch]);
        cb.memory[ch] = NULL; cb.decoder[ch] = NULL;
    }
    cb.channels = 0; cb.fragments_left = 0; cb.fragment_size = 0;
}
void a2dp_lc3plus_decoder_configure(const uint8_t *info) {
    a2dp_lc3plus_decoder_cleanup();
    const unsigned caps = MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT;
    const uint32_t before = heap_caps_get_free_size(caps);
    esp_a2dp_codec_format_t f = {0};
    bool ok = esp_a2dp_codec_policy_parse_avdt(info,&f) && f.id == ESP_A2DP_CODEC_LC3PLUS;
    size_t size = 0, pcm_bytes = 0;
    if (ok) {
        cb.channels = f.channels; cb.rate = f.rate; cb.duration_us = f.nominal_frame_us;
        cb.samples = lc3_hr_frame_samples(true,cb.duration_us,cb.rate);
        size = lc3_hr_decoder_size(true,cb.duration_us,cb.rate);
        pcm_bytes = cb.samples > 0 ? (size_t)cb.samples*cb.channels*3u : 0;
        ok = size && pcm_bytes && before >= size*cb.channels+pcm_bytes+CONFIG_BT_A2DP_CODEC_HEAP_RESERVE &&
             heap_caps_get_largest_free_block(caps) >= size;
    }
    for (unsigned ch=0;ok && ch<cb.channels;++ch) {
        cb.memory[ch] = osi_malloc(size);
        if (!cb.memory[ch]) { ok = false; break; }
        cb.decoder[ch] = lc3_hr_setup_decoder(true,cb.duration_us,cb.rate,cb.rate,cb.memory[ch]);
        if (!cb.decoder[ch]) ok = false;
    }
    if (!ok) a2dp_lc3plus_decoder_cleanup();
    esp_a2dp_codec_policy_config_result(ok,(uint32_t)(size*f.channels),sizeof(cb),pcm_bytes,
        before,heap_caps_get_free_size(caps),heap_caps_get_largest_free_block(caps));
}

ssize_t a2dp_lc3plus_decoder_decode_packet_header(BT_HDR *packet) {
    if (!packet || packet->len < 13) return -EINVAL;
    const uint8_t *p = (const uint8_t *)(packet+1)+packet->offset;
    const size_t length = packet->len;
    if ((p[0] >> 6) != 2) return -EINVAL;
    size_t h = 12u+4u*(p[0]&15u);
    if (h >= length) return -EINVAL;
    if (p[0]&0x10) {
        if (length-h < 4) return -EINVAL;
        size_t words = ((size_t)p[h+2]<<8)|p[h+3];
        h += 4u+4u*words;
        if (h >= length) return -EINVAL;
    }
    size_t padding = (p[0]&0x20) ? p[length-1] : 0;
    if ((p[0]&0x20) && (!padding || padding >= length-h)) return -EINVAL;
    uint8_t ph = p[h++], count = ph&15u;
    if (h+padding >= length || !count || (ph&0x10)) return -EINVAL;
    if (ph&0x80) {
        if (ph&0x40) {
            cb.fragment_size = 0;
            if (count < 2 || (ph&0x20)) return -EINVAL;
        } else if (!cb.fragments_left || count+1 != cb.fragments_left ||
                   ((count == 1) != !!(ph&0x20))) {
            cb.fragments_left = 0; cb.fragment_size = 0;
            return -EINVAL;
        }
        cb.fragments_left = count;
    } else {
        /* This existing vendor profile carries one complete frame per packet. */
        if (count != 1 || (ph&0x60)) return -EINVAL;
        cb.fragments_left = 0; cb.fragment_size = 0;
    }
    packet->offset += h; packet->len -= h+padding;
    return 0;
}
bool a2dp_lc3plus_decoder_decode_packet(BT_HDR *packet, unsigned char *pcm, size_t capacity) {
    if (!packet || !pcm || !cb.callback || cb.channels < 1 || cb.channels > 2 || cb.samples <= 0)
        return false;
    const uint8_t *in = (const uint8_t *)(packet+1)+packet->offset;
    size_t size = packet->len;
    if (cb.fragments_left) {
        if (size > sizeof(cb.fragment)-cb.fragment_size) {
            cb.fragments_left=0;cb.fragment_size=0;return false;
        }
        memcpy(cb.fragment+cb.fragment_size,in,size); cb.fragment_size+=size;
        if (cb.fragments_left > 1) return true;
        in=cb.fragment;size=cb.fragment_size;cb.fragments_left=0;cb.fragment_size=0;
    }
    size_t needed = (size_t)cb.samples*cb.channels*3u;
    if (needed > capacity || size < cb.channels) return false;
    for (unsigned ch=0;ch<cb.channels;++ch) {
        size_t bytes = size/cb.channels+(ch < size%cb.channels);
        if (!cb.decoder[ch] || lc3_decode(cb.decoder[ch],in,bytes,
                LC3_PCM_FORMAT_S24_3LE,pcm+3u*ch,cb.channels) != 0) return false;
        in += bytes;
    }
    cb.callback(pcm,needed);
    return true;
}
#endif
