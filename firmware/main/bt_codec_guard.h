#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "esp_a2dp_api.h"
/* Core0 control only. No PCM ownership, DSP state or NVS writes. */
void bt_codec_guard_init(void);
void bt_codec_guard_profile_ready(void);
void bt_codec_guard_connection(const uint8_t peer[6], esp_a2d_connection_state_t state);
void bt_codec_guard_poll(uint32_t bt_silence_blocks, bool bt_selected, bool underrun_observation_valid);
