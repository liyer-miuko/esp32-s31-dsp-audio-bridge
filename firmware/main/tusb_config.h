/*
 * TinyUSB configuration - ESP32-S31 UAC2 asynchronous speaker bridge.
 *
 * HS only. OUT EP carries up to 96 kHz stereo 4-byte-subslot data:
 *   packet budget = 96 kHz / 8 kHz = 12 samples -> 96 bytes;
 *   +1 sample slack -> 104 bytes wMaxPacketSize.
 * The internal FIFO is 8.67 ms deep at 96k/32-bit (64 HS microframes). The
 * built-in FIFO_COUNT feedback controller keeps it near half fill (~4.3 ms @96k / ~8.7 ms @48k)
 * while the pump task is clocked by the I2S TX DMA.
 */
#ifndef TUSB_CONFIG_H_
#define TUSB_CONFIG_H_

#ifdef __cplusplus
extern "C" {
#endif

/* ---------------- Common ---------------- */
#define CFG_TUSB_OS                 OPT_OS_FREERTOS
#define CFG_TUSB_DEBUG              0   /* keep the USB ISR path short */

#define CFG_TUD_ENABLED             1
#define CFG_TUSB_RHPORT0_MODE       OPT_MODE_DEVICE
#define CFG_TUD_MAX_SPEED           OPT_MODE_HIGH_SPEED
#define CFG_TUD_ENDPOINT0_SIZE      64

/* ---------------- Device classes: AUDIO ---------------- */
#define CFG_TUD_AUDIO               1
#define CFG_TUD_CDC                 0
#define CFG_TUD_MSC                 0
/* Unused device classes stay listed as 0 next to CFG_TUD_HID. */
#define CFG_TUD_HID                 0
#define CFG_TUD_MIDI                0
#define CFG_TUD_VENDOR              0

/* ---------------- UAC2 speaker (host -> device) ---------------- */
#define CFG_TUD_AUDIO_ENABLE_EP_OUT         1
#define CFG_TUD_AUDIO_ENABLE_EP_IN          0
#define CFG_TUD_AUDIO_ENABLE_FEEDBACK_EP    1

#define CFG_TUD_AUDIO_FUNC_1_N_CHANNELS_RX          2
#define CFG_TUD_AUDIO_FUNC_1_N_BYTES_PER_SAMPLE_RX  4
#define CFG_TUD_AUDIO_FUNC_1_RESOLUTION_RX          24
#define CFG_TUD_AUDIO_FUNC_1_MAX_SAMPLE_RATE        96000

/* 96 kHz stereo 4-byte subslot, +1 sample of feedback headroom */
#define CFG_TUD_AUDIO_FUNC_1_EP_OUT_SZ_MAX          104
/* v9: ~8.67 ms deep at 96k/32 (64 HS microframes). v4-proven safe depth;
 * built-in FIFO_COUNT keeps it near half fill (~4.3 ms). */
#define CFG_TUD_AUDIO_FUNC_1_EP_OUT_SW_BUF_SZ       (104 * 64)

#define CFG_TUD_AUDIO_FUNC_1_CTRL_BUF_SZ            64

#ifdef __cplusplus
}
#endif

#endif /* TUSB_CONFIG_H_ */
