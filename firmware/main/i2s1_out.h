/* Dual-master output layout. DATA never returns to core0 for processing.
 * Core0 creates I2S1 to retain the proven DMA interrupt allocation; core1 owns
 * all runtime reconfiguration and PCM writes. Shared APLL does not imply
 * aligned master phases or aligned output sample positions. */
#pragma once
#include "sdkconfig.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"

#ifndef DUAL_I2S_TEST_P0_PINS_ALT
#ifdef CONFIG_DUAL_I2S_TEST_P0_PINS_ALT
#define DUAL_I2S_TEST_P0_PINS_ALT 1
#else
#define DUAL_I2S_TEST_P0_PINS_ALT 0
#endif
#endif
#if DUAL_I2S_TEST_P0_PINS_ALT != 0 && DUAL_I2S_TEST_P0_PINS_ALT != 1
#error "Choose exactly one dual-I2S pin layout"
#endif
#define DUAL_I2S_P0_MCLK 2
#define DUAL_I2S_P0_DOUT 7
#define I2S1_OUT_GPIO_MCLK 13
#define I2S1_OUT_GPIO_DOUT 16
#if DUAL_I2S_TEST_P0_PINS_ALT
#define DUAL_I2S_P0_BCLK 14
#define DUAL_I2S_P0_WS 15
#define I2S1_OUT_GPIO_BCLK 3
#define I2S1_OUT_GPIO_WS 4
#else
#define DUAL_I2S_P0_BCLK 3
#define DUAL_I2S_P0_WS 4
#define I2S1_OUT_GPIO_BCLK 14
#define I2S1_OUT_GPIO_WS 15
#endif

#ifdef __cplusplus
extern "C" {
#endif
typedef struct {
    bool ready, running;
    uint32_t rate, calls, bytes, errors, partial;
    int slot_bits, format, last_error;
} i2s1_out_stats_t;
esp_err_t i2s1_out_core1_init(void);
void i2s1_out_route_clocks(void);
esp_err_t i2s1_out_core1_start(void);
esp_err_t i2s1_out_core1_stop(void);
/* Block-boundary dual-clock transaction only: stop and park on XTAL so the
 * shared APLL has no retained P1 reference before P0 changes its frequency. */
esp_err_t i2s1_out_core1_park_clock(void);
esp_err_t i2s1_out_core1_set_rate(uint32_t rate);
esp_err_t i2s1_out_core1_configure(uint32_t rate,int bits);
esp_err_t i2s1_out_core1_set_slot_width(int bits);
esp_err_t i2s1_out_core1_set_format(int format);
esp_err_t i2s1_out_write(const void *buf, size_t size, size_t *written);
bool i2s1_out_is_ready(void);
bool i2s1_out_is_running(void);
void i2s1_out_get_stats(i2s1_out_stats_t *stats);
#ifdef __cplusplus
}
#endif
