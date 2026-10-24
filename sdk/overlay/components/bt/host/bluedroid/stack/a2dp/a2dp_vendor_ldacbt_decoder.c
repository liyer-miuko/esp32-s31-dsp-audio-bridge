/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include "common/bt_trace.h"
#include "stack/a2dp_vendor_ldac.h"
#include "stack/a2dp_vendor_ldac_constants.h"
#include "stack/a2dp_vendor_ldac_decoder.h"

#include <ldacBT.h>
#include <string.h>
#include "esp_timer.h"


#if (defined(LDAC_DEC_INCLUDED) && LDAC_DEC_INCLUDED == TRUE)

const uint32_t a2dp_ldac_packet_patch_abi = 0x01410001u;

typedef struct {
    HANDLE_LDAC_BT ldac_handle;
    bool has_ldac_handle;
    bool ready;
    LDACBT_SMPL_FMT_T pcm_fmt;

    decoded_data_callback_t decode_callback;
    uint32_t packets, input_b, pcm_frames, bad_header, fragmented;
    uint32_t truncated, bad_frame, decode_errors, init_errors, seq_jumps, faults;
    uint32_t decode_wall_max_us, frame_max_b, sample_rate;
    /* 0.172: split the wall time into pure ldacBT_decode() and the app PCM
     * callback. Without this split a 20 ms/packet wall time cannot be
     * attributed to the decoder or to the callback/lock path. */
    uint64_t dec_us_sum, cb_us_sum;
    uint32_t dec_us_max, cb_us_max, timing_calls;
    unsigned channels;
    uint16_t next_seq;
    bool seq_valid;
} tA2DP_LDAC_DECODER_CB;

static tA2DP_LDAC_DECODER_CB a2dp_ldac_decoder_cb;

extern volatile uint32_t g_ldac_stage_cpu_us[4];
extern volatile uint32_t g_ldac_stage_calls;

static void ldac_report(const char *reason) {
    const tA2DP_LDAC_DECODER_CB *s = &a2dp_ldac_decoder_cb;
    const uint32_t dec_avg = s->timing_calls ? (uint32_t)(s->dec_us_sum / s->timing_calls) : 0u;
    const uint32_t cb_avg = s->timing_calls ? (uint32_t)(s->cb_us_sum / s->timing_calls) : 0u;
    LOG_INFO("ldac-stat reason=%s ready=%u rate=%u pkt=%u in_b=%u pcm_f=%u hdr=%u frag=%u trunc=%u bad_frame=%u dec_err=%u init_err=%u seq_jump=%u frame_max_b=%u wall_max_us=%u dec_avg_us=%u dec_max_us=%u cb_avg_us=%u cb_max_us=%u calls=%u faults=%u",
             reason, (unsigned)s->ready, (unsigned)s->sample_rate,
             (unsigned)s->packets, (unsigned)s->input_b, (unsigned)s->pcm_frames,
             (unsigned)s->bad_header, (unsigned)s->fragmented, (unsigned)s->truncated,
             (unsigned)s->bad_frame,
             (unsigned)s->decode_errors, (unsigned)s->init_errors, (unsigned)s->seq_jumps,
             (unsigned)s->frame_max_b, (unsigned)s->decode_wall_max_us,
             (unsigned)dec_avg, (unsigned)s->dec_us_max,
             (unsigned)cb_avg, (unsigned)s->cb_us_max,
             (unsigned)s->timing_calls, (unsigned)s->faults);
    if (g_ldac_stage_calls) {
        const uint32_t n = g_ldac_stage_calls;
        LOG_INFO("ldac-stage-cpu unpack_us=%u decode_us=%u imdct_us=%u setpcm_us=%u calls=%u",
                 (unsigned)(g_ldac_stage_cpu_us[0] / n), (unsigned)(g_ldac_stage_cpu_us[1] / n),
                 (unsigned)(g_ldac_stage_cpu_us[2] / n), (unsigned)(g_ldac_stage_cpu_us[3] / n),
                 (unsigned)n);
    }
}

/* 0.188: measure the A2DP_DECODER task's own CPU time over 1 s windows.
 * ldac_avg is wall time and includes preemption; the task run-time counter
 * tells whether the decoder is CPU-bound or merely starved/blocked. */
static void ldac_cpu_report(void) {
    static uint32_t s_win_wall, s_win_task;
    TaskStatus_t st;
    vTaskGetInfo(NULL, &st, pdFALSE, eInvalid);
    const uint32_t now = (uint32_t)esp_timer_get_time();
    if (s_win_wall != 0) {
        const uint32_t w = now - s_win_wall;
        const uint32_t c = (uint32_t)st.ulRunTimeCounter - s_win_task;
        if (w != 0) {
            LOG_INFO("ldac-cpu task_us=%u wall_us=%u pct=%u",
                     (unsigned)c, (unsigned)w, (unsigned)(((uint64_t)c * 100u) / w));
        }
    }
    s_win_wall = now;
    s_win_task = (uint32_t)st.ulRunTimeCounter;
}

static void ldac_note_fault(void) {
    uint32_t n = ++a2dp_ldac_decoder_cb.faults;
    if (n && !(n & (n - 1))) ldac_report("fault");
}

/* Includes preemption and PCM callback work, not pure decoder CPU time. */
static bool ldac_packet_done(bool ok, uint32_t start_us, uint32_t frames_before) {
    uint32_t elapsed = (uint32_t)esp_timer_get_time() - start_us;
    if (elapsed > a2dp_ldac_decoder_cb.decode_wall_max_us)
        a2dp_ldac_decoder_cb.decode_wall_max_us = elapsed;
    if ((!frames_before && a2dp_ldac_decoder_cb.pcm_frames) ||
        !(a2dp_ldac_decoder_cb.packets & 1023u))
        ldac_report(frames_before ? "periodic" : "first_pcm_or_periodic");
    {
        static uint32_t s_cpu_next;
        const uint32_t now = (uint32_t)esp_timer_get_time();
        if (now - s_cpu_next >= 1000000u) {
            s_cpu_next = now;
            ldac_cpu_report();
        }
    }
    return ok;
}

bool a2dp_ldac_decoder_init(decoded_data_callback_t decode_callback) {
    memset(&a2dp_ldac_decoder_cb, 0, sizeof(a2dp_ldac_decoder_cb));
    HANDLE_LDAC_BT hndl = ldacBT_get_handle();
    if (!hndl) {
        APPL_TRACE_ERROR("%s: could not get decoder handle", __func__);
        a2dp_ldac_decoder_cb.has_ldac_handle = false;
        return false;
    }

    a2dp_ldac_decoder_cb.ldac_handle = hndl;
    a2dp_ldac_decoder_cb.has_ldac_handle = true;
    a2dp_ldac_decoder_cb.pcm_fmt = LDACBT_SMPL_FMT_S24;
    a2dp_ldac_decoder_cb.decode_callback = decode_callback;
    return true;
}

void a2dp_ldac_decoder_cleanup() {
    ldac_report("cleanup");
    if (a2dp_ldac_decoder_cb.has_ldac_handle) {
        HANDLE_LDAC_BT hndl = a2dp_ldac_decoder_cb.ldac_handle;
        ldacBT_close_handle(hndl);
        ldacBT_free_handle(hndl);
    }

    memset(&a2dp_ldac_decoder_cb, 0, sizeof(a2dp_ldac_decoder_cb));
    a2dp_ldac_decoder_cb.has_ldac_handle = false;
}

ssize_t a2dp_ldac_decoder_decode_packet_header(BT_HDR* p_buf) {
    a2dp_ldac_decoder_cb.packets++;
    const uint8_t *p = (const uint8_t *)(p_buf + 1) + p_buf->offset;
    const size_t len = p_buf->len;
    size_t header_len = 12, padding = 0;
    if (len < header_len || (p[0] >> 6) != 2) goto bad_header;
    header_len += 4u * (p[0] & 0x0fu);
    if (header_len > len) goto bad_header;
    if (p[0] & 0x10u) {
        if (header_len + 4u > len) goto bad_header;
        size_t words = ((size_t)p[header_len + 2] << 8) | p[header_len + 3];
        header_len += 4u + words * 4u;
        if (header_len > len) goto bad_header;
    }
    if (p[0] & 0x20u) {
        padding = p[len - 1];
        if (!padding || padding > len - header_len) goto bad_header;
    }
    if (header_len + A2DP_LDAC_MPL_HDR_LEN + padding >= len) goto bad_header;
    const uint16_t seq = ((uint16_t)p[2] << 8) | p[3];
    if (a2dp_ldac_decoder_cb.seq_valid && seq != a2dp_ldac_decoder_cb.next_seq)
        a2dp_ldac_decoder_cb.seq_jumps++;
    a2dp_ldac_decoder_cb.next_seq = (uint16_t)(seq + 1u);
    a2dp_ldac_decoder_cb.seq_valid = true;
    /* The old adapter never assembled fragmented LDAC. Keep that limitation
     * explicit instead of treating a fragment as a complete compressed frame. */
    if (p[header_len] & (A2DP_LDAC_HDR_F_MSK | A2DP_LDAC_HDR_S_MSK | A2DP_LDAC_HDR_L_MSK)) {
        a2dp_ldac_decoder_cb.fragmented++;
        ldac_note_fault();
        return -1;
    }
    header_len += A2DP_LDAC_MPL_HDR_LEN;
    p_buf->offset += header_len;
    p_buf->len -= header_len + padding;
    return 0;
bad_header:
    a2dp_ldac_decoder_cb.bad_header++;
    ldac_note_fault();
    return -1;
}

bool a2dp_ldac_decoder_decode_packet(BT_HDR* p_buf, unsigned char* buf, size_t buf_len) {
    const uint32_t start_us = (uint32_t)esp_timer_get_time();
    const uint32_t frames_before = a2dp_ldac_decoder_cb.pcm_frames;
    HANDLE_LDAC_BT hndl = a2dp_ldac_decoder_cb.ldac_handle;

    if (!a2dp_ldac_decoder_cb.ready || !a2dp_ldac_decoder_cb.has_ldac_handle) {
        a2dp_ldac_decoder_cb.decode_errors++;
        ldac_note_fault();
        return ldac_packet_done(false, start_us, frames_before);
    }

    /* ldacBT_decode() writes one frame's interleaved PCM into buf without any
     * bounds check. The largest frame is the 2x rates (88.2/96 kHz, 256
     * samples/ch): stereo S24 = 2*3*256 = 1536 bytes, S32/F32 = 2048. If the
     * scratch buffer is too small, drop the packet rather than overflow the
     * heap (which silently corrupts the adjacent LDAC decode structs). */
    if (buf_len < 2048) {
        a2dp_ldac_decoder_cb.decode_errors++;
        ldac_note_fault();
        return ldac_packet_done(false, start_us, frames_before);
    }

    uint8_t *src = (uint8_t*)((unsigned char *)(p_buf + 1) + p_buf->offset);
    int32_t in_count = p_buf->len;
    int32_t in_used = 0;
    int32_t used_bytes = 0;
    int32_t out_used = 0;

    /* Per-frame decode + callback: decode each LDAC frame and immediately
     * deliver PCM via callback. This reuses the same ~2KB of decode buffer
     * per frame instead of accumulating all frames (up to 8KB for SQ mode).
     * Benefits:
     *  - Constant 2KB buffer usage regardless of quality mode
     *  - Lower decode-to-playback latency
     *  - No risk of decode buffer overflow
     */
    while ((in_count - in_used) > 0) {
        const int remaining = in_count - in_used;
        const uint8_t *frame = src + in_used;
        if (remaining < 3) {
            a2dp_ldac_decoder_cb.truncated++;
            ldac_note_fault();
            return ldac_packet_done(false, start_us, frames_before);
        }
        if (frame[0] != 0xaau) {
            a2dp_ldac_decoder_cb.bad_frame++;
            ldac_note_fault();
            return ldac_packet_done(false, start_us, frames_before);
        }
        /* Transport header: sync8 + rate3 + channels2 + length9 + status2.
         * The 9-bit length is raw-frame bytes minus1; add its3-byte header. */
        const int frame_bytes = (((frame[1] & 7u) << 6) | (frame[2] >> 2)) + 4;
        if (frame_bytes > remaining) {
            a2dp_ldac_decoder_cb.truncated++;
            ldac_note_fault();
            return ldac_packet_done(false, start_us, frames_before);
        }
        const uint32_t dec_t0 = (uint32_t)esp_timer_get_time();
        int result = ldacBT_decode(hndl,
                               src + in_used,
                               buf,
                               a2dp_ldac_decoder_cb.pcm_fmt,
                                frame_bytes,
                               (int *)&used_bytes,
                               (int *)&out_used);
        const uint32_t dec_t1 = (uint32_t)esp_timer_get_time();
        const uint32_t dec_dt = dec_t1 - dec_t0;
        a2dp_ldac_decoder_cb.dec_us_sum += dec_dt;
        if (dec_dt > a2dp_ldac_decoder_cb.dec_us_max)
            a2dp_ldac_decoder_cb.dec_us_max = dec_dt;
        if (result != 0 || used_bytes != frame_bytes || out_used <= 0 ||
            (size_t)out_used > buf_len || out_used % (3u * a2dp_ldac_decoder_cb.channels)) {
            a2dp_ldac_decoder_cb.decode_errors++;
            ldac_note_fault();
            return ldac_packet_done(false, start_us, frames_before);
        }
        in_used += used_bytes;
        a2dp_ldac_decoder_cb.input_b += used_bytes;
        a2dp_ldac_decoder_cb.pcm_frames += out_used / (3u * a2dp_ldac_decoder_cb.channels);
        if ((uint32_t)frame_bytes > a2dp_ldac_decoder_cb.frame_max_b)
            a2dp_ldac_decoder_cb.frame_max_b = frame_bytes;

        /* Flush decoded frame immediately — only ~2KB per frame */
        const uint32_t cb_t0 = (uint32_t)esp_timer_get_time();
        a2dp_ldac_decoder_cb.decode_callback(buf, out_used);
        const uint32_t cb_dt = (uint32_t)esp_timer_get_time() - cb_t0;
        a2dp_ldac_decoder_cb.cb_us_sum += cb_dt;
        if (cb_dt > a2dp_ldac_decoder_cb.cb_us_max)
            a2dp_ldac_decoder_cb.cb_us_max = cb_dt;
        a2dp_ldac_decoder_cb.timing_calls++;
    }

    return ldac_packet_done(true, start_us, frames_before);
}

void a2dp_ldac_decoder_configure(const uint8_t* p_codec_info) {
    a2dp_ldac_decoder_cb.ready = false;
    a2dp_ldac_decoder_cb.seq_valid = false;
    tA2DP_LDAC_CIE cie;
    tA2D_STATUS status;
    status = A2DP_ParseInfoLdac(&cie, (uint8_t*)p_codec_info, false);
    if (status != A2D_SUCCESS) {
        LOG_ERROR("%s: failed parsing codec info. %d", __func__, status);
        return;
    }

    uint32_t sf = 0;

    if (cie.sampleRate == A2DP_LDAC_SAMPLING_FREQ_44100) {
        sf = 44100;
    } else if (cie.sampleRate == A2DP_LDAC_SAMPLING_FREQ_48000) {
        sf = 48000;
    } else if (cie.sampleRate == A2DP_LDAC_SAMPLING_FREQ_88200) {
        sf = 88200;
    } else if (cie.sampleRate == A2DP_LDAC_SAMPLING_FREQ_96000) {
        sf = 96000;
    } else if (cie.sampleRate == A2DP_LDAC_SAMPLING_FREQ_176400) {
        sf = 176400;
    } else if (cie.sampleRate == A2DP_LDAC_SAMPLING_FREQ_192000) {
        sf = 192000;
    }
    LOG_INFO("%s: LDAC Sampling frequency = %lu", __func__, sf);
    if (sf == 0) {
        APPL_TRACE_ERROR("%s: unsupported LDAC sample rate", __func__);
        return;
    }


    int cm = 0;
    if (cie.channelMode == A2DP_LDAC_CHANNEL_MODE_MONO) {
        cm = LDACBT_CHANNEL_MODE_MONO;
        LOG_INFO("%s: LDAC Channel mode: Mono", __func__);
    } else if (cie.channelMode == A2DP_LDAC_CHANNEL_MODE_DUAL) {
        cm = LDACBT_CHANNEL_MODE_DUAL_CHANNEL;
        LOG_INFO("%s: LDAC Channel mode: Dual", __func__);
    } else if (cie.channelMode == A2DP_LDAC_CHANNEL_MODE_STEREO) {
        cm = LDACBT_CHANNEL_MODE_STEREO;
        // cm = LDACBT_CHANNEL_MODE_DUAL_CHANNEL;
        LOG_INFO("%s: LDAC Channel mode: Stereo", __func__);
    }

    int res;
    HANDLE_LDAC_BT hndl = a2dp_ldac_decoder_cb.ldac_handle;
    if (!a2dp_ldac_decoder_cb.has_ldac_handle || !hndl) {
        hndl = ldacBT_get_handle();
        a2dp_ldac_decoder_cb.ldac_handle = hndl;
        a2dp_ldac_decoder_cb.has_ldac_handle = hndl != NULL;
        if (!hndl) {
            a2dp_ldac_decoder_cb.init_errors++;
            ldac_note_fault();
            return;
        }
    }
    res = ldacBT_init_handle_decode(hndl, cm, sf, 0, 0, 0);
    if (res != 0) {
        a2dp_ldac_decoder_cb.init_errors++;
        int err = ldacBT_get_error_code(hndl);
        APPL_TRACE_ERROR("%s: decoder init failed res = %d, err = %d", __func__,
                         res, LDACBT_API_ERR(err));

        ldacBT_close_handle(hndl);
        ldacBT_free_handle(hndl);
        a2dp_ldac_decoder_cb.ldac_handle = NULL;
        a2dp_ldac_decoder_cb.has_ldac_handle = false;

        return;
    }
    a2dp_ldac_decoder_cb.sample_rate = sf;
    a2dp_ldac_decoder_cb.channels = cm == LDACBT_CHANNEL_MODE_MONO ? 1u : 2u;
    a2dp_ldac_decoder_cb.ready = true;
    ldac_report("configured");
}

#endif /* defined(LDAC_DEC_INCLUDED) && LDAC_DEC_INCLUDED == TRUE) */
