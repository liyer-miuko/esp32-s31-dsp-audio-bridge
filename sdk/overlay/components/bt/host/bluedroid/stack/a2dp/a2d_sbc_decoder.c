/*
 * SPDX-FileCopyrightText: 2016 The Android Open Source Project
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <stdlib.h>
#include <string.h>
#include "common/bt_trace.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "esp_a2dp_codec_policy.h"
#include "stack/a2dp_rtp_bounds.h"
#include "stack/a2d_sbc.h"
#include "stack/a2d_sbc_decoder.h"
#include "stack/bt_types.h"
#include "oi_codec_sbc.h"
#include "oi_status.h"
#include "osi/future.h"

#if (defined(SBC_DEC_INCLUDED) && SBC_DEC_INCLUDED == TRUE)

typedef struct {
  OI_CODEC_SBC_DECODER_CONTEXT decoder_context;
  OI_UINT32 context_data[CODEC_DATA_WORDS(2, SBC_CODEC_FAST_FILTER_BUFFERS)];
  OI_UINT8 maxChannels;
  OI_UINT8 pcmStride;
  decoded_data_callback_t decode_callback;
  uint32_t rate;
  uint16_t fragment_size, fragment_next_seq;
  bool fragment_active;
  /* Also covers the dual-channel bitpool bound. Allocated only with SBC. */
  uint8_t fragment[2u * SBC_MAX_FRAME_LEN];
} tA2DP_SBC_DECODER_CB;

/* Allocated lazily on init, freed on cleanup. The control block embeds a ~2 KB
 * codec context; keeping it off .bss frees that RAM whenever SBC is not the
 * active codec (only one A2DP codec runs at a time). */
static tA2DP_SBC_DECODER_CB *s_sbc_cb = NULL;

bool a2dp_sbc_decoder_init(decoded_data_callback_t decode_callback) {
  if (!s_sbc_cb) {
    s_sbc_cb = (tA2DP_SBC_DECODER_CB *)calloc(1, sizeof(*s_sbc_cb));
    if (!s_sbc_cb) {
      LOG_ERROR("%s: cannot allocate SBC decoder cb", __func__);
      return false;
    }
  }
  s_sbc_cb->maxChannels = 2;
  s_sbc_cb->pcmStride = 2;
  if (!a2dp_sbc_decoder_reset()) {
    a2dp_sbc_decoder_cleanup();
    return false;
  }
  s_sbc_cb->decode_callback = decode_callback;
  return true;
}

void a2dp_sbc_decoder_cleanup(void) {
  if (s_sbc_cb) {
    free(s_sbc_cb);
    s_sbc_cb = NULL;
  }
}

bool a2dp_sbc_decoder_reset() {
  if (!s_sbc_cb) {
    return false;
  }
  s_sbc_cb->fragment_active = false;
  s_sbc_cb->fragment_size = 0;
  OI_STATUS status = OI_CODEC_SBC_DecoderReset(
      &s_sbc_cb->decoder_context, s_sbc_cb->context_data,
      sizeof(s_sbc_cb->context_data),
      s_sbc_cb->maxChannels, s_sbc_cb->pcmStride,
      false, false);
  if (!OI_SUCCESS(status)) {
    LOG_ERROR("%s: OI_CODEC_SBC_DecoderReset failed with error code %d",
              __func__, status);
    return false;
  }
  return true;
}

void a2dp_sbc_decoder_configure(const uint8_t* p_codec_info) {
  tA2D_SBC_CIE cie;
  tA2D_STATUS status;
  status = A2D_ParsSbcInfo(&cie, (uint8_t*)p_codec_info, false);
  if (status != A2D_SUCCESS) {
    LOG_ERROR("%s: failed parsing codec info. %d", __func__, status);
    esp_a2dp_codec_policy_config_result(false,0,0,0,
        heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT),
        heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT),
        heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
    return;
  }

  uint32_t sr = 0;
  if (cie.samp_freq == A2D_SBC_IE_SAMP_FREQ_16) {
    sr = 16000;
  } else if (cie.samp_freq == A2D_SBC_IE_SAMP_FREQ_32) {
    sr = 32000;
  } else if (cie.samp_freq == A2D_SBC_IE_SAMP_FREQ_44) {
    sr = 44100;
  } else if (cie.samp_freq == A2D_SBC_IE_SAMP_FREQ_48) {
    sr = 48000;
  }
  LOG_INFO("%s: SBC Sampling frequency = %lu", __func__, sr);

  if (cie.ch_mode == A2D_SBC_IE_CH_MD_MONO) {
    LOG_INFO("%s: SBC Channel mode: Mono", __func__);
  } else if (cie.ch_mode == A2D_SBC_IE_CH_MD_DUAL) {
    LOG_INFO("%s: SBC Channel mode: Dual", __func__);
  } else if (cie.ch_mode == A2D_SBC_IE_CH_MD_STEREO) {
    LOG_INFO("%s: SBC Channel mode: Stereo", __func__);
  } else if (cie.ch_mode == A2D_SBC_IE_CH_MD_JOINT) {
    LOG_INFO("%s: SBC Channel mode: Joint", __func__);
  }

  uint8_t block_len = 0;
  if (cie.block_len == A2D_SBC_IE_BLOCKS_4) {
    block_len = 4;
  } else if (cie.block_len == A2D_SBC_IE_BLOCKS_8) {
    block_len = 8;
  } else if (cie.block_len == A2D_SBC_IE_BLOCKS_12) {
    block_len = 12;
  } else if (cie.block_len == A2D_SBC_IE_BLOCKS_16) {
    block_len = 16;
  }
  LOG_INFO("%s: SBC Block length = %lu", __func__, block_len);

  uint8_t subbands = 0;
  if (cie.num_subbands == A2D_SBC_IE_SUBBAND_4) {
    subbands = 4;
  } else if (cie.num_subbands == A2D_SBC_IE_SUBBAND_8) {
    subbands = 8;
  }
  LOG_INFO("%s: SBC Number of subbands = %lu", __func__, subbands);

  if (cie.alloc_mthd == A2D_SBC_IE_ALLOC_MD_S) {
    LOG_INFO("%s: SBC Allocation method: SNR", __func__);
  } else if (cie.alloc_mthd == A2D_SBC_IE_ALLOC_MD_L) {
    LOG_INFO("%s: SBC Allocation method: Loudness", __func__);
  }

  LOG_INFO("%s: SBC Maximum bitpool = %lu", __func__, cie.max_bitpool);
  LOG_INFO("%s: SBC Minimum bitpool = %lu", __func__, cie.min_bitpool);

  if (!s_sbc_cb) {
    esp_a2dp_codec_policy_config_result(false,0,0,0,0,0,0);
    return;
  }
  if (cie.ch_mode == A2D_SBC_IE_CH_MD_MONO) {
    s_sbc_cb->maxChannels = 1;
  } else {
    s_sbc_cb->maxChannels = 2;
  }
  /* OI rejects pcmStride > maxChannels. Also keep the callback's interleaving
   * identical to the mono/stereo format published to the application. */
  s_sbc_cb->pcmStride = s_sbc_cb->maxChannels;
  s_sbc_cb->rate = sr;
  const bool ready = sr != 0 && a2dp_sbc_decoder_reset();
  const uint32_t heap = heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  esp_a2dp_codec_policy_config_result(ready,sizeof(*s_sbc_cb),0,0,heap,heap,
      heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
}

ssize_t a2dp_sbc_decoder_decode_packet_header(BT_HDR* p_buf) {
  size_t header_len, payload_len;
  uint16_t sequence;
  if (!a2dp_rtp_bounds(p_buf, 2u, &header_len, &payload_len, &sequence)) {
    if (s_sbc_cb) s_sbc_cb->fragment_active = false;
    return -EINVAL;
  }
  p_buf->offset += header_len;
  p_buf->len = payload_len;
  p_buf->layer_specific = sequence;
  return 0;
}

bool a2dp_sbc_decoder_decode_packet(BT_HDR* p_buf, unsigned char* buf, size_t buf_len) {
    UINT8 *src;
    UINT32 src_len;
    size_t num_frames;
    OI_INT16 *dst;
    UINT32 written;
    OI_STATUS status;
    int count;

    if (!s_sbc_cb || !s_sbc_cb->decode_callback || !p_buf || !buf ||
        p_buf->len < 2u || buf_len < 512u) {
        return false;
    }
    UINT8 *data = (UINT8 *)(p_buf + 1) + p_buf->offset;
    const uint8_t ph = data[0];
    if (ph & 0x10u) { s_sbc_cb->fragment_active = false; return false; }
    src = data + 1;
    src_len = p_buf->len - 1u;
    num_frames = ph & 15u;
    if (ph & 0x80u) {
        const bool first = (ph & 0x40u) != 0, last = (ph & 0x20u) != 0;
        if (first) {
            s_sbc_cb->fragment_size = 0;
            s_sbc_cb->fragment_active = !last;
            if (last) return false;
        } else if (!s_sbc_cb->fragment_active ||
                   p_buf->layer_specific != s_sbc_cb->fragment_next_seq) {
            s_sbc_cb->fragment_active = false;
            return false;
        }
        if (src_len > sizeof(s_sbc_cb->fragment) - s_sbc_cb->fragment_size) {
            s_sbc_cb->fragment_active = false;
            return false;
        }
        memcpy(s_sbc_cb->fragment + s_sbc_cb->fragment_size, src, src_len);
        s_sbc_cb->fragment_size += src_len;
        s_sbc_cb->fragment_next_seq = (uint16_t)(p_buf->layer_specific + 1u);
        if (!last) return true;
        s_sbc_cb->fragment_active = false;
        src = s_sbc_cb->fragment;
        src_len = s_sbc_cb->fragment_size;
        num_frames = 1; /* Only a complete reassembled frame reaches OI. */
    } else {
        s_sbc_cb->fragment_active = false;
        if ((ph & 0x60u) || !num_frames) return false;
    }

    APPL_TRACE_DEBUG("Number of sbc frames %d, frame_len %d\n", num_frames, src_len);

    for (count = 0; count < num_frames && src_len != 0; count ++) {
        static const uint32_t rates[4] = {16000, 32000, 44100, 48000};
        if (src_len < 4u || src[0] != OI_SBC_SYNCWORD ||
            rates[src[1] >> 6] != s_sbc_cb->rate ||
            (((src[1] >> 2) & 3u) ? 2u : 1u) != s_sbc_cb->maxChannels) return false;
        const UINT32 before = src_len;
        written = buf_len;  /* Reset available space each frame */
        dst = (OI_INT16*)buf;  /* Reset output position to start of buffer */
        status = OI_CODEC_SBC_DecodeFrame(&s_sbc_cb->decoder_context,
                                          (const OI_BYTE **)&src,
                                          (OI_UINT32 *)&src_len,
                                          (OI_INT16 *)dst,
                                          (OI_UINT32 *)&written);
        if (!OI_SUCCESS(status) || src_len >= before || written > buf_len ||
            !written || written % (s_sbc_cb->pcmStride * sizeof(OI_INT16))) {
            APPL_TRACE_ERROR("Decoding failure: %d\n", status);
            return false;
        }
        /* Deliver each decoded frame immediately via callback */
        if (written > 0) {
            s_sbc_cb->decode_callback((uint8_t*)buf, written);
        }
    }

    return count == num_frames && src_len == 0;
}

#endif /* #if (defined(SBC_DEC_INCLUDED) && SBC_DEC_INCLUDED == TRUE) */
