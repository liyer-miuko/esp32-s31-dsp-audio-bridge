/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include "common/bt_trace.h"
#include "stack/a2dp_vendor_aptx.h"
#include "stack/a2dp_vendor_aptx_hd.h"
#include "stack/a2dp_vendor_aptx_ll.h"
#include "stack/a2dp_vendor_aptx_decoder.h"
#include "freeaptx.h"
#include <string.h>
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "esp_a2dp_codec_policy.h"


#if (defined(APTX_DEC_INCLUDED) && APTX_DEC_INCLUDED == TRUE)

const uint32_t a2dp_aptx_budget_patch_abi = 0x01410001u;

typedef enum
{
    APTX_STANDARD,
    APTX_HD,
    APTX_LL,
} tA2DP_APTX_TYPE;

typedef struct {
  struct aptx_context* decoder_context;
  tA2DP_APTX_TYPE aptx_type;
  decoded_data_callback_t decode_callback;
  /* Decoder-lifetime modulo-2^32 counters, accessed by the sink worker only.
   * resync_b is reported by freeaptx on recovery, not an all-loss byte count.
   * boundary_b counts only cached partial codewords discarded at stream reset.
   * unsynced counts calls, seq_jumps counts discontinuities, neither is frames. */
  uint32_t packets, input_b, pcm_frames, bad_header, seq_jumps;
  uint32_t unsynced, resync_b, stalls, header_reject_b, decode_reject_b, boundary_b, faults;
  uint32_t bounds_errors;
  uint16_t next_seq;
  bool seq_valid;
  uint32_t decode_wall_max_us;
} tA2DP_APTX_DECODER_CB;

static tA2DP_APTX_DECODER_CB a2dp_aptx_decoder_cb;

static void aptx_report(const char *reason) {
    const tA2DP_APTX_DECODER_CB *s = &a2dp_aptx_decoder_cb;
    LOG_INFO("aptx-stat reason=%s type=%u pkt=%u in_b=%u pcm_f=%u hdr=%u seq_jump=%u unsync_calls=%u resync_b=%u stall=%u hdr_reject_b=%u dec_reject_b=%u boundary_b=%u faults=%u wall_max_us=%u bounds=%u",
             reason, (unsigned)s->aptx_type, (unsigned)s->packets,
             (unsigned)s->input_b, (unsigned)s->pcm_frames,
             (unsigned)s->bad_header, (unsigned)s->seq_jumps,
             (unsigned)s->unsynced, (unsigned)s->resync_b,
             (unsigned)s->stalls, (unsigned)s->header_reject_b, (unsigned)s->decode_reject_b,
             (unsigned)s->boundary_b, (unsigned)s->faults,
             (unsigned)s->decode_wall_max_us, (unsigned)s->bounds_errors);
}

/* Packet wall time includes scheduler preemption and synchronous PCM delivery;
 * it is not pure codec CPU time. First PCM / 1024 packets only, no log flood. */
static bool aptx_packet_done(bool ok, uint32_t start_us, uint32_t frames_before) {
    uint32_t elapsed = (uint32_t)esp_timer_get_time() - start_us;
    if (elapsed > a2dp_aptx_decoder_cb.decode_wall_max_us)
        a2dp_aptx_decoder_cb.decode_wall_max_us = elapsed;
    if ((!frames_before && a2dp_aptx_decoder_cb.pcm_frames) ||
        !(a2dp_aptx_decoder_cb.packets & 1023u))
        aptx_report(frames_before ? "periodic" : "first_pcm_or_periodic");
    return ok;
}

static void aptx_note_fault(void) {
    uint32_t n = ++a2dp_aptx_decoder_cb.faults;
    /* No per-packet logging under corruption: first/powers-of-two only. */
    if (n && !(n & (n - 1))) aptx_report("fault");
}

static void aptx_end_stream(const char *reason) {
    a2dp_aptx_decoder_cb.boundary_b +=
        aptx_decode_sync_finish(a2dp_aptx_decoder_cb.decoder_context);
    a2dp_aptx_decoder_cb.seq_valid = false;
    aptx_report(reason);
}

bool a2dp_aptx_decoder_init(decoded_data_callback_t decode_callback) {
    /* The worker calls configure immediately afterwards. Allocate the right
     * mode there once: HD entry must not allocate/reset/free a temporary
     * Standard decoder before allocating the actual HD context. */
    memset(&a2dp_aptx_decoder_cb, 0, sizeof(a2dp_aptx_decoder_cb));
    a2dp_aptx_decoder_cb.decode_callback = decode_callback;
    a2dp_aptx_decoder_cb.aptx_type = APTX_STANDARD;
    return decode_callback != NULL;
}

void a2dp_aptx_decoder_cleanup(void) {
    struct aptx_context* decoder_context = a2dp_aptx_decoder_cb.decoder_context;
    if (!decoder_context) {
        /* A failed configure can leave a correctly empty partial instance. */
        return;
    }

    aptx_end_stream("cleanup");
    aptx_finish(decoder_context);
    a2dp_aptx_decoder_cb.decoder_context = NULL;
}

bool a2dp_aptx_decoder_reset(void) {
    struct aptx_context* decoder_context = a2dp_aptx_decoder_cb.decoder_context;
    if (!decoder_context) {
        APPL_TRACE_ERROR("%s decoder context not initialized", __func__);
        return false;
    }

    aptx_end_stream("reset");
    return true;
}

ssize_t a2dp_aptx_decoder_decode_packet_header(BT_HDR* p_buf) {
    a2dp_aptx_decoder_cb.packets++;
    if (a2dp_aptx_decoder_cb.aptx_type == APTX_HD)
        esp_a2dp_codec_policy_packet_wire(ESP_A2DP_CODEC_APTX_HD,
            (const uint8_t *)(p_buf + 1) + p_buf->offset, p_buf->len);
    if (a2dp_aptx_decoder_cb.packets == 1) {
        const uint8_t *wire = (const uint8_t *)(p_buf + 1) + p_buf->offset;
        uint32_t prefix = 0;
        for (size_t i = 0; i < p_buf->len && i < 4; ++i) prefix = (prefix << 8) | wire[i];
        LOG_INFO("aptx-wire type=%u len=%u first_up_to4_be=0x%08lx header_contract=%s",
            (unsigned)a2dp_aptx_decoder_cb.aptx_type, (unsigned)p_buf->len,
            (unsigned long)prefix, a2dp_aptx_decoder_cb.aptx_type == APTX_HD ? "RTP" : "BARE");
    }
    /* This fork's avdt_scb_hdl_pkt_no_frag forwards the original offset/len.
     * Standard/LL retain their raw-payload contract; HD carries RTP here.
     * Do not infer an RTP header from compressed Standard/LL payload bytes. */
    if (a2dp_aptx_decoder_cb.aptx_type != APTX_HD) {
        return 0;
    }
    const uint8_t *p = (const uint8_t *)(p_buf + 1) + p_buf->offset;
    size_t len = p_buf->len;
    size_t header_len = 12;
    size_t padding = 0;
    if (len < header_len || (p[0] >> 6) != 2) goto bad_header;
    header_len += 4 * (p[0] & 0x0f);
    if (header_len > len) goto bad_header;
    if (p[0] & 0x10) {
        if (header_len + 4 > len) goto bad_header;
        size_t words = ((size_t)p[header_len + 2] << 8) | p[header_len + 3];
        header_len += 4 + words * 4;
        if (header_len > len) goto bad_header;
    }
    if (p[0] & 0x20) {
        padding = p[len - 1];
        if (!padding || padding > len - header_len) goto bad_header;
    }
    if (header_len + padding >= len) goto bad_header;

    uint16_t seq = ((uint16_t)p[2] << 8) | p[3];
    if (a2dp_aptx_decoder_cb.seq_valid && seq != a2dp_aptx_decoder_cb.next_seq) {
        a2dp_aptx_decoder_cb.seq_jumps++;
        esp_a2dp_codec_policy_packet_error(ESP_A2DP_CODEC_APTX_HD, ESP_A2DP_PACKET_SEQUENCE_GAP,
            (int32_t)(((uint32_t)a2dp_aptx_decoder_cb.next_seq << 16) | seq));
        aptx_note_fault();
    }
    a2dp_aptx_decoder_cb.next_seq = (uint16_t)(seq + 1);
    a2dp_aptx_decoder_cb.seq_valid = true;
    p_buf->offset += header_len;
    p_buf->len -= header_len + padding;
    return 0;

bad_header:
    esp_a2dp_codec_policy_packet_error(ESP_A2DP_CODEC_APTX_HD, ESP_A2DP_PACKET_BAD_RTP, p_buf->len);
    a2dp_aptx_decoder_cb.bad_header++;
    a2dp_aptx_decoder_cb.header_reject_b += p_buf->len; /* wire packet including header */
    aptx_note_fault();
    return -1;
}

bool a2dp_aptx_decoder_decode_packet(BT_HDR* p_buf, unsigned char* buf, size_t buf_len) {
    const uint32_t start_us = (uint32_t)esp_timer_get_time();
    const uint32_t frames_before = a2dp_aptx_decoder_cb.pcm_frames;
    struct aptx_context* decoder_context = a2dp_aptx_decoder_cb.decoder_context;

    unsigned char* src = ((unsigned char *)(p_buf + 1) + p_buf->offset);
    const size_t words = buf_len / 32; /* one codeword -> four stereo Q1.31 frames */
    if (!decoder_context || !buf || !a2dp_aptx_decoder_cb.decode_callback || words < 2) {
        if (a2dp_aptx_decoder_cb.aptx_type == APTX_HD)
            esp_a2dp_codec_policy_packet_error(ESP_A2DP_CODEC_APTX_HD, ESP_A2DP_PACKET_RESULT_BOUNDS,
                (int32_t)buf_len);
        a2dp_aptx_decoder_cb.stalls++;
        a2dp_aptx_decoder_cb.decode_reject_b += p_buf->len; /* compressed payload only */
        aptx_note_fault();
        return aptx_packet_done(false, start_us, frames_before);
    }

    const size_t codeword_b = a2dp_aptx_decoder_cb.aptx_type == APTX_HD ? 6 : 4;
    /* sync requires capacity for all input PLUS one cached codeword. Packed
     * S24 uses 24 bytes/word; reverse expansion fits the original Q1.31 scratch.
     * The library owns the at-most-5-byte partial tail across packet calls. */
    const size_t input_limit = (words - 1) * codeword_b;
    bool fault = false;
    while (p_buf->len) {
        size_t chunk = p_buf->len < input_limit ? p_buf->len : input_limit;
        size_t written, dropped;
        int synced;
        size_t processed = aptx_decode_sync(decoder_context, src, chunk,
                                            buf, words * 24, &written, &synced, &dropped);
        /* Validate the library boundary BEFORE expanding S24 or advancing the
         * BT_HDR. A malformed result must not wrap len or overwrite the heap. */
        if (processed > chunk || written > words * 24 || written % 6 != 0) {
            if (a2dp_aptx_decoder_cb.aptx_type == APTX_HD)
                esp_a2dp_codec_policy_packet_error(ESP_A2DP_CODEC_APTX_HD, ESP_A2DP_PACKET_RESULT_BOUNDS,
                    (int32_t)written);
            a2dp_aptx_decoder_cb.bounds_errors++;
            a2dp_aptx_decoder_cb.decode_reject_b += p_buf->len;
            aptx_note_fault();
            aptx_end_stream("invalid_result");
            return aptx_packet_done(false, start_us, frames_before);
        }
        a2dp_aptx_decoder_cb.input_b += processed;
        a2dp_aptx_decoder_cb.resync_b += dropped;
        if (!synced) a2dp_aptx_decoder_cb.unsynced++;
        fault |= dropped != 0 || (!synced && chunk >= codeword_b);

        /* Read each complete S24 sample before writing its expanded container.
         * Byte stores avoid signed-shift UB and unaligned/aliasing assumptions. */
        size_t samples = written / 3;
        for (size_t i = samples; i > 0; --i) {
            size_t s = (i - 1) * 3, d = (i - 1) * 4;
            uint8_t lo = buf[s], mid = buf[s + 1], hi = buf[s + 2];
            buf[d] = 0; buf[d + 1] = lo; buf[d + 2] = mid; buf[d + 3] = hi;
        }
        if (samples) {
            a2dp_aptx_decoder_cb.pcm_frames += samples / 2;
            a2dp_aptx_decoder_cb.decode_callback(buf, samples * 4);
        }
        src += processed;
        p_buf->offset += processed;
        p_buf->len -= processed;
        if (processed != chunk) {
            if (a2dp_aptx_decoder_cb.aptx_type == APTX_HD)
                esp_a2dp_codec_policy_packet_error(ESP_A2DP_CODEC_APTX_HD, ESP_A2DP_PACKET_DECODE_ERROR,
                    (int32_t)(chunk - processed));
            /* Capacity is proven above; a short consume is a decoder fault,
             * not normal buffer pressure. Never silently report success. */
            a2dp_aptx_decoder_cb.stalls++;
            a2dp_aptx_decoder_cb.decode_reject_b += p_buf->len;
            aptx_note_fault();
            aptx_end_stream("short_consume");
            return aptx_packet_done(false, start_us, frames_before);
        }
    }
    if (fault) aptx_note_fault();
    return aptx_packet_done(true, start_us, frames_before);
}

void a2dp_aptx_decoder_configure(const uint8_t* p_codec_info) {
    struct aptx_context* decoder_context = a2dp_aptx_decoder_cb.decoder_context;
    btav_a2dp_codec_index_t index = A2DP_SinkCodecIndex(p_codec_info);
    const unsigned caps = MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT;
    const uint32_t before = heap_caps_get_free_size(caps);
    union {
        tA2DP_APTX_CIE standard;
        tA2DP_APTX_HD_CIE hd;
        tA2DP_APTX_LL_CIE ll;
    } cie = {0};
    tA2D_STATUS status = A2D_FAIL;
    tA2DP_APTX_TYPE type = APTX_STANDARD;
    if (index == BTAV_A2DP_CODEC_INDEX_SINK_APTX_HD) {
        status = A2DP_ParseInfoAptxHd(&cie.hd, p_codec_info, FALSE);
        type = APTX_HD;
    } else if (index == BTAV_A2DP_CODEC_INDEX_SINK_APTX_LL) {
        status = A2DP_ParseInfoAptxLl(&cie.ll, p_codec_info, FALSE);
        type = APTX_LL;
    } else if (index == BTAV_A2DP_CODEC_INDEX_SINK_APTX) {
        status = A2DP_ParseInfoAptx(&cie.standard, p_codec_info, FALSE);
    }
    esp_a2dp_codec_format_t f = {0};
    bool ready = status == A2D_SUCCESS &&
        esp_a2dp_codec_policy_parse_avdt(p_codec_info, &f) &&
        f.id >= ESP_A2DP_CODEC_APTX && f.id <= ESP_A2DP_CODEC_APTX_LL;
    if (ready) {
        const bool was_hd = a2dp_aptx_decoder_cb.aptx_type == APTX_HD;
        if (decoder_context) aptx_end_stream("configure");
        /* Same-codec/rate reconfiguration needs a reset, not heap churn.
         * Standard and LL share the non-HD kernel; HD changes codeword width. */
        if (decoder_context && was_hd != (type == APTX_HD)) {
            aptx_finish(decoder_context);
            decoder_context = NULL;
        }
        if (!decoder_context) decoder_context = aptx_init(type == APTX_HD);
        a2dp_aptx_decoder_cb.decoder_context = decoder_context;
        a2dp_aptx_decoder_cb.aptx_type = type;
        ready = decoder_context != NULL;
    }
    /* A void configure() must explicitly report failure: the worker may not
     * mark an allocation/parse failure ready just because init() once worked. */
    esp_a2dp_codec_policy_config_result(ready,0,0,0,before,
        heap_caps_get_free_size(caps),heap_caps_get_largest_free_block(caps));
    LOG_INFO("%s: type=%u rate=%lu ready=%u heap_before=%lu heap_after=%lu largest=%lu", __func__, (unsigned)type,
             (unsigned long)f.rate, ready, (unsigned long)before,
             (unsigned long)heap_caps_get_free_size(caps),
             (unsigned long)heap_caps_get_largest_free_block(caps));
}

#endif /* defined(APTX_DEC_INCLUDED) && APTX_DEC_INCLUDED == TRUE) */
