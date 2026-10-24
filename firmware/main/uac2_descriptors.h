/*
 * UAC2 descriptor set for ESP32-S31 USB audio bridge.
 *
 * Design:
 * - UAC 2.0, High-Speed only
 * - Speaker topology: IT(USB streaming) -> FU(mute/volume) -> OT(speaker)
 * - One programmable clock source, discrete rates: 44100 / 48000 / 88200 / 96000
 * - AS interface alt0 = zero bandwidth, alt1 = 24-bit packed subslot (3 bytes),
 *   alt2 = 32-bit subslot (4 bytes). The endpoint wMaxPacketSize is identical
 *   for both alternates (96 kHz stereo plus one sample of feedback headroom).
 * - Explicit asynchronous feedback endpoint (16.16, HS interval = 1 ms).
 */
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

enum {
    ITF_NUM_AUDIO_CONTROL = 0,
    ITF_NUM_AUDIO_STREAMING = 1,
    ITF_NUM_TOTAL = 2,
};

/* AudioControl entity IDs */
#define UAC2_ENTITY_CLOCK               0x04
#define UAC2_ENTITY_SPK_INPUT_TERMINAL  0x01
#define UAC2_ENTITY_SPK_FEATURE_UNIT    0x02
#define UAC2_ENTITY_SPK_OUTPUT_TERMINAL 0x03

/*
 * TinyUSB 0.21 TUD_AUDIO20_DESC_STD_AS_ISO_FB_EP emits bmAttributes=0x11
 * (No-Sync + explicit feedback). UAC2 4.10.2.1 requires Asynchronous +
 * explicit feedback (0x15) for an async source/sink. Use the compliant form.
 */
#define UAC20_DESC_STD_AS_ISO_FB_EP_FIX(_ep, _epsize, _interval) \
    7, TUSB_DESC_ENDPOINT, _ep, \
    (uint8_t)((uint8_t)TUSB_XFER_ISOCHRONOUS | \
              (uint8_t)TUSB_ISO_EP_ATT_ASYNCHRONOUS | \
              (uint8_t)TUSB_ISO_EP_ATT_EXPLICIT_FB), \
    U16_TO_U8S_LE(_epsize), _interval

#define UAC2_SPK_AC_TOTAL_LEN ( \
    TUD_AUDIO20_DESC_CLK_SRC_LEN + \
    TUD_AUDIO20_DESC_INPUT_TERM_LEN + \
    TUD_AUDIO20_DESC_OUTPUT_TERM_LEN + \
    TUD_AUDIO20_DESC_FEATURE_UNIT_LEN(2))

/* One streaming alternate: std AS + CS AS + Type-I + iso data EP + CS EP + FB EP */
#define UAC2_SPK_AS_ALT_LEN ( \
    TUD_AUDIO20_DESC_STD_AS_LEN + \
    TUD_AUDIO20_DESC_CS_AS_INT_LEN + \
    TUD_AUDIO20_DESC_TYPE_I_FORMAT_LEN + \
    TUD_AUDIO20_DESC_STD_AS_ISO_EP_LEN + \
    TUD_AUDIO20_DESC_CS_AS_ISO_EP_LEN + \
    TUD_AUDIO20_DESC_STD_AS_ISO_FB_EP_LEN)

#define UAC2_AUDIO_DESC_LEN ( \
    TUD_AUDIO20_DESC_IAD_LEN + \
    TUD_AUDIO20_DESC_STD_AC_LEN + \
    TUD_AUDIO20_DESC_CS_AC_LEN + \
    UAC2_SPK_AC_TOTAL_LEN + \
    TUD_AUDIO20_DESC_STD_AS_LEN /* alt0 */ + \
    2 * UAC2_SPK_AS_ALT_LEN /* alt1 + alt2 */)

#define UAC2_CONFIG_DESC_TOTAL_LEN (TUD_CONFIG_DESC_LEN + UAC2_AUDIO_DESC_LEN)

#define UAC2_FU_CH_CTRL \
    (AUDIO20_CTRL_RW << AUDIO20_FEATURE_UNIT_CTRL_MUTE_POS) | \
    (AUDIO20_CTRL_RW << AUDIO20_FEATURE_UNIT_CTRL_VOLUME_POS)

#define UAC2_SPEAKER_STEREO_2_FORMAT_DESCRIPTOR(_itfnum, _stridx, _epout, _epoutsize, _epfb, _epfbsize) \
    /* Standard Interface Association Descriptor (IAD) */\
    TUD_AUDIO20_DESC_IAD(_itfnum, 0x02, 0x00),\
    /* Standard AC Interface Descriptor(4.7.1) */\
    TUD_AUDIO20_DESC_STD_AC(_itfnum, 0x00, _stridx),\
    /* Class-Specific AC Interface Header Descriptor(4.7.2) */\
    TUD_AUDIO20_DESC_CS_AC(0x0200, AUDIO20_FUNC_DESKTOP_SPEAKER, UAC2_SPK_AC_TOTAL_LEN, AUDIO20_CS_AS_INTERFACE_CTRL_LATENCY_POS),\
    /* Clock Source Descriptor(4.7.2.1): internal programmable clock, freq RW, validity R */\
    TUD_AUDIO20_DESC_CLK_SRC(UAC2_ENTITY_CLOCK, AUDIO20_CLOCK_SOURCE_ATT_INT_PRO_CLK, \
        ((AUDIO20_CTRL_RW << AUDIO20_CLOCK_SOURCE_CTRL_CLK_FRQ_POS) | \
         (AUDIO20_CTRL_R << AUDIO20_CLOCK_SOURCE_CTRL_CLK_VAL_POS)), 0x01, 0x00),\
    /* Input Terminal Descriptor(4.7.2.4): stereo USB streaming */\
    TUD_AUDIO20_DESC_INPUT_TERM(UAC2_ENTITY_SPK_INPUT_TERMINAL, AUDIO_TERM_TYPE_USB_STREAMING, 0x00, UAC2_ENTITY_CLOCK, \
        0x02, (AUDIO20_CHANNEL_CONFIG_FRONT_LEFT | AUDIO20_CHANNEL_CONFIG_FRONT_RIGHT), 0x00, \
        (AUDIO20_CTRL_R << AUDIO20_IN_TERM_CTRL_CONNECTOR_POS), 0x00),\
    /* Feature Unit Descriptor(4.7.2.8): mute + volume, master + 2 channels */\
    TUD_AUDIO20_DESC_FEATURE_UNIT(UAC2_ENTITY_SPK_FEATURE_UNIT, UAC2_ENTITY_SPK_INPUT_TERMINAL, 0x00, \
        UAC2_FU_CH_CTRL, UAC2_FU_CH_CTRL, UAC2_FU_CH_CTRL),\
    /* Output Terminal Descriptor(4.7.2.5) */\
    TUD_AUDIO20_DESC_OUTPUT_TERM(UAC2_ENTITY_SPK_OUTPUT_TERMINAL, AUDIO_TERM_TYPE_OUT_DESKTOP_SPEAKER, 0x01, \
        UAC2_ENTITY_SPK_FEATURE_UNIT, UAC2_ENTITY_CLOCK, 0x0000, 0x00),\
    /* Interface 1, Alternate 0 - zero bandwidth */\
    TUD_AUDIO20_DESC_STD_AS_INT((uint8_t)((_itfnum) + 1), 0x00, 0x00, 0x00),\
    /* Interface 1, Alternate 1 - 24-bit packed subslot (3 bytes) */\
    TUD_AUDIO20_DESC_STD_AS_INT((uint8_t)((_itfnum) + 1), 0x01, 0x02, 0x00),\
    TUD_AUDIO20_DESC_CS_AS_INT(UAC2_ENTITY_SPK_INPUT_TERMINAL, AUDIO20_CTRL_NONE, AUDIO20_FORMAT_TYPE_I, \
        AUDIO20_DATA_FORMAT_TYPE_I_PCM, 0x02, \
        (AUDIO20_CHANNEL_CONFIG_FRONT_LEFT | AUDIO20_CHANNEL_CONFIG_FRONT_RIGHT), 0x00),\
    TUD_AUDIO20_DESC_TYPE_I_FORMAT(3, 24),\
    TUD_AUDIO20_DESC_STD_AS_ISO_EP(_epout, \
        (uint8_t)((uint8_t)TUSB_XFER_ISOCHRONOUS | (uint8_t)TUSB_ISO_EP_ATT_ASYNCHRONOUS | (uint8_t)TUSB_ISO_EP_ATT_DATA), \
        _epoutsize, 0x01),\
    TUD_AUDIO20_DESC_CS_AS_ISO_EP(AUDIO20_CS_AS_ISO_DATA_EP_ATT_NON_MAX_PACKETS_OK, AUDIO20_CTRL_NONE, \
        AUDIO20_CS_AS_ISO_DATA_EP_LOCK_DELAY_UNIT_MILLISEC, 0x0001),\
    UAC20_DESC_STD_AS_ISO_FB_EP_FIX(_epfb, _epfbsize, 4),\
    /* Interface 1, Alternate 2 - 32-bit in 4-byte subslot */\
    TUD_AUDIO20_DESC_STD_AS_INT((uint8_t)((_itfnum) + 1), 0x02, 0x02, 0x00),\
    TUD_AUDIO20_DESC_CS_AS_INT(UAC2_ENTITY_SPK_INPUT_TERMINAL, AUDIO20_CTRL_NONE, AUDIO20_FORMAT_TYPE_I, \
        AUDIO20_DATA_FORMAT_TYPE_I_PCM, 0x02, \
        (AUDIO20_CHANNEL_CONFIG_FRONT_LEFT | AUDIO20_CHANNEL_CONFIG_FRONT_RIGHT), 0x00),\
    TUD_AUDIO20_DESC_TYPE_I_FORMAT(4, 32),\
    TUD_AUDIO20_DESC_STD_AS_ISO_EP(_epout, \
        (uint8_t)((uint8_t)TUSB_XFER_ISOCHRONOUS | (uint8_t)TUSB_ISO_EP_ATT_ASYNCHRONOUS | (uint8_t)TUSB_ISO_EP_ATT_DATA), \
        _epoutsize, 0x01),\
    TUD_AUDIO20_DESC_CS_AS_ISO_EP(AUDIO20_CS_AS_ISO_DATA_EP_ATT_NON_MAX_PACKETS_OK, AUDIO20_CTRL_NONE, \
        AUDIO20_CS_AS_ISO_DATA_EP_LOCK_DELAY_UNIT_MILLISEC, 0x0001),\
    UAC20_DESC_STD_AS_ISO_FB_EP_FIX(_epfb, _epfbsize, 4)

#ifdef __cplusplus
}
#endif
