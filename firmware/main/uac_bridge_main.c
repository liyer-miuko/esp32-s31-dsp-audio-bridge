/*
 * ESP32-S31 UAC audio bridge - DSP v1.
 *
 * 0.113: core1 owns the DSP chain and the I2S0 output port; core0 is the active
 * core (USB/UAC input, input normalisation, UAC feature unit, source policy,
 * protocol, MUTE). PCM prepared on core0 is handed over in fixed slots and is
 * never read back there.
 */
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "i2s_out_v2.h"
#include "i2s1_out.h"
#include "uac2_audio.h"
#include "a2dp_audio.h"
#include "dsp_audio.h"
#include "dsp_config.h"
#include "dsp_engine.h"
#include "hw_mute.h"
#include "serial_proto.h"
#include "ble_tuning.h"
#include "amp_control.h"

static const char *TAG = "main";

/* Classic audio and BLE tuning share the already enabled BTDM stack.
 * BLE transports control only; PCM remains on the Classic/core0 path. */
#define APP_A2DP_ENABLE 1

void app_main(void)
{
    ESP_LOGI(TAG, "=== S31 UAC2/A2DP bridge with core1 DSP (UAC/BT -> DSP -> dual I2S masters) ===");
    ESP_LOGI(TAG, "target: %s", CONFIG_IDF_TARGET);

    /* ★ Stage C1, spec §1.4 hard requirement: the amp's MUTE pin must be at its
     * MUTED level at the VERY BEGINNING of the init sequence - before the I2S
     * channel exists, before the DSP task exists, before anything can put
     * activity on DOUT. While the pin is high-Z the amp's own 100k pull-down
     * keeps it muted; the moment we configure the pin as an output the level
     * becomes our choice, so it has to be "muted" from the first instruction.
     * The stored polarity/delays are adopted later (hw_mute_start). */
    ESP_ERROR_CHECK(hw_mute_early_init());

    esp_err_t ret = nvs_flash_init();
    /* A firmware startup must not erase user NVS on an init error. */
    ESP_ERROR_CHECK(ret);

    /* ★ 0.113 init order: the core1 worker is created first and creates the
     * I2S0 channel itself (so its DMA interrupt is allocated on core1); core0
     * only starts USB tasks after that succeeded. I2S starts at 48 kHz /
     * 32-bit container. GPIO6 is damaged and unused.
     *
     * I2S1 is created on core0 first: its DMA interrupt then lives on core0,
     * while the core1 worker only writes PCM into it. This mirrors the proven
     * standalone probe (init+enable on core0) and avoids a core1 interrupt
     * conflict that left the secondary DMA stalled. */
    ESP_ERROR_CHECK(i2s1_out_core1_init());
    ESP_ERROR_CHECK(dsp_audio_init());
    ESP_ERROR_CHECK(dsp_audio_wait_ready(2000));

    /* Parameter store (NVS-backed). Loaded before any audio can flow so a
     * stored configuration overrides the Kconfig default; its I2S line-format
     * call is marshalled to core1 and applied at a block boundary. */
    ESP_ERROR_CHECK(dsp_config_init());
    const esp_err_t amp_init_err=amp_control_init();
    if(amp_init_err!=ESP_OK) ESP_LOGE(TAG,"optional amplifier control unavailable: %s",esp_err_to_name(amp_init_err));

    /* UAC2 speaker with asynchronous feedback (creates the pump/ctrl tasks).
     * The pump only starts producing once I2S is running, i.e. after
     * i2s_out_start() below. */
    ESP_ERROR_CHECK(uac2_audio_init());

#if APP_A2DP_ENABLE
    /* BT before I2S is started so Bluedroid/controller startup on core0 cannot
     * delay the pump and overflow the I2S event queue. */
    ESP_ERROR_CHECK(a2dp_audio_init());
#else
    ESP_LOGW(TAG, "A2DP startup disabled; core1 DSP/ports run normally");
#endif

    /* Host protocol on the console UART (WebUI config + telemetry). */
    ESP_ERROR_CHECK(serial_proto_init());

#if CONFIG_BT_BLE_TUNING_ENABLE
    /* BLE tuning channel (same AA55 protocol over GATT). Failure is not fatal:
     * BLE must not take A2DP/serial down with it. This service keeps all
     * BLE traffic on core0 and applies parameters at core1 block boundaries. */
    esp_err_t ble_err = ble_tuning_init();
    if (ble_err != ESP_OK) {
        ESP_LOGE(TAG, "ble_tuning_init failed: %s (A2DP/serial keep running)",
                 esp_err_to_name(ble_err));
    }
#endif

    /* ★ Stage C1: start the mute state machine. Placed AFTER dsp_config_init so
     * the stored polarity/delays are in effect, and BEFORE i2s_out_start so the
     * "release mute" delay B (default 30 ms) is already ticking when the very
     * first audio can appear - that 30 ms belongs to B's side (spec §1.3). */
    ESP_ERROR_CHECK(hw_mute_start());

    /* Start the I2S clock only after the core1 worker (DSP + output owner) and
     * the producer tasks exist: no startup DMA queue overflow, no stale-buffer
     * click. The call is marshalled to core1. */
    ESP_ERROR_CHECK(i2s_out_start());

    /* Core0 allocation/start retain the verified I2S1 interrupt affinity;
     * all ongoing PCM conversion and writes belong to core1. */
    ESP_ERROR_CHECK(i2s1_out_core1_start());
    /* The amplifier's DSP can leave reset only after its I2S clocks exist.
     * This optional service runs on core0 and leaves GPIO8 mute unchanged. */
    if(amp_init_err==ESP_OK) {
        const esp_err_t amp_start_err=amp_control_start();
        if(amp_start_err!=ESP_OK) ESP_LOGE(TAG,"optional amplifier worker unavailable: %s",esp_err_to_name(amp_start_err));
    }

    ESP_LOGI(TAG, "startup done: dual masters layout=%u; P0=%d/%d/%d/%d P1=%d/%d/%d/%d",
             DUAL_I2S_TEST_P0_PINS_ALT,
             DUAL_I2S_P0_MCLK, DUAL_I2S_P0_BCLK, DUAL_I2S_P0_WS, DUAL_I2S_P0_DOUT,
             I2S1_OUT_GPIO_MCLK, I2S1_OUT_GPIO_BCLK, I2S1_OUT_GPIO_WS, I2S1_OUT_GPIO_DOUT);
    ESP_LOGI(TAG, "connect USB_DP/USB_DM to a High-Speed host port; BT name: %s",
             CONFIG_BT_A2DP_DEVICE_NAME);
}
