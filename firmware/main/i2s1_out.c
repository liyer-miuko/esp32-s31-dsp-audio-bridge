#include <string.h>
#include "esp_log.h"
#include "driver/i2s_std.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "i2s1_out.h"

static const char *TAG="i2s1_out";
static i2s_chan_handle_t s_tx1;
static bool s_clock_parked;
static portMUX_TYPE s_mux=portMUX_INITIALIZER_UNLOCKED;
static i2s1_out_stats_t s_state={.rate=48000,.slot_bits=CONFIG_I2S_OUT_SLOT_BITS};
/* Retain the existing bring-up symbol names for read-only diagnostics. */
volatile int g_i2s1_start_err=-1, g_i2s1_start_calls, g_i2s1_reenable;
volatile int g_i2s1_stop_calls, g_i2s1_set_err=-1;
volatile int g_i2s1_reconfigs; /* diagnostic: actual P1 disable/reconfig/enable events */

static i2s_std_slot_config_t make_slot(int fmt,int bits) {
    i2s_std_slot_config_t slot=I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(
        (i2s_data_bit_width_t)bits,I2S_SLOT_MODE_STEREO);
    if (fmt==1) {
        i2s_std_slot_config_t left=I2S_STD_MSB_SLOT_DEFAULT_CONFIG(
            (i2s_data_bit_width_t)bits,I2S_SLOT_MODE_STEREO);
        slot=left;
    }
    slot.slot_bit_width=(i2s_slot_bit_width_t)bits;
    return slot;
}
static i2s_std_clk_config_t make_clock(uint32_t rate,int bits) {
    i2s_std_clk_config_t clock=I2S_STD_CLK_DEFAULT_CONFIG(rate);
    clock.clk_src=I2S_CLK_SRC_APLL;
    clock.mclk_multiple=bits==24 ? I2S_MCLK_MULTIPLE_384 : I2S_MCLK_MULTIPLE_256;
    return clock;
}
static void publish_error(esp_err_t error) {
    portENTER_CRITICAL(&s_mux);s_state.last_error=error;portEXIT_CRITICAL(&s_mux);
}
static void preload_silence(int bits) {
    static const uint8_t zero[128];
    size_t remaining=6u*240u*2u*(bits/8u);
    while (remaining) {
        size_t loaded=0, n=remaining>sizeof(zero) ? sizeof(zero) : remaining;
        esp_err_t e=i2s_channel_preload_data(s_tx1,zero,n,&loaded);
        if (e!=ESP_OK || !loaded) break;
        remaining-=loaded;
    }
}
esp_err_t i2s1_out_core1_init(void) {
    if (s_tx1) return ESP_ERR_INVALID_STATE;
    i2s_chan_config_t channel=I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_1,I2S_ROLE_MASTER);
    channel.dma_desc_num=6;channel.dma_frame_num=240;
    channel.auto_clear_before_cb=false;
    channel.auto_clear_after_cb=true;
    esp_err_t e=i2s_new_channel(&channel,&s_tx1,NULL);
    if (e!=ESP_OK) {publish_error(e);return e;}
    i2s_std_config_t standard={
        .clk_cfg=make_clock(s_state.rate,s_state.slot_bits),
        .slot_cfg=make_slot(s_state.format,s_state.slot_bits),
        .gpio_cfg={.mclk=I2S1_OUT_GPIO_MCLK,.bclk=I2S1_OUT_GPIO_BCLK,
            .ws=I2S1_OUT_GPIO_WS,.dout=I2S1_OUT_GPIO_DOUT,.din=I2S_GPIO_UNUSED},
    };
    e=i2s_channel_init_std_mode(s_tx1,&standard);
    if (e!=ESP_OK) {i2s_del_channel(s_tx1);s_tx1=NULL;publish_error(e);return e;}
    preload_silence(s_state.slot_bits);
    portENTER_CRITICAL(&s_mux);s_state.ready=true;s_state.last_error=ESP_OK;portEXIT_CRITICAL(&s_mux);
    i2s1_out_route_clocks();
    return ESP_OK;
}
void i2s1_out_route_clocks(void) {
    ESP_LOGI(TAG,"master layout=%u MCLK=%d BCLK=%d WS=%d DOUT=%d shared_apll=1 phase_aligned=0",
        DUAL_I2S_TEST_P0_PINS_ALT,I2S1_OUT_GPIO_MCLK,I2S1_OUT_GPIO_BCLK,I2S1_OUT_GPIO_WS,I2S1_OUT_GPIO_DOUT);
}
esp_err_t i2s1_out_core1_start(void) {
    g_i2s1_start_calls++;
    if (!s_tx1 || s_clock_parked) return ESP_ERR_INVALID_STATE;
    if (s_state.running) return ESP_OK;
    esp_err_t e=i2s_channel_enable(s_tx1);g_i2s1_start_err=e;
    portENTER_CRITICAL(&s_mux);s_state.running=e==ESP_OK;s_state.last_error=e;portEXIT_CRITICAL(&s_mux);
    return e;
}
esp_err_t i2s1_out_core1_stop(void) {
    g_i2s1_stop_calls++;
    if (!s_tx1) return ESP_ERR_INVALID_STATE;
    if (!s_state.running) return ESP_OK;
    esp_err_t e=i2s_channel_disable(s_tx1);
    portENTER_CRITICAL(&s_mux);if (e==ESP_OK) s_state.running=false;s_state.last_error=e;portEXIT_CRITICAL(&s_mux);
    return e;
}
esp_err_t i2s1_out_core1_park_clock(void) {
    if (!s_tx1) return ESP_OK; /* No P1 clock reference exists before init. */
    esp_err_t e=i2s1_out_core1_stop();
    if (e!=ESP_OK) return e;
    /* 48k * 384 = 18.432 MHz is valid from the 40 MHz XTAL for every supported
     * 16/24/32-bit slot. Outputs are stopped; this is not an emitted rate. */
    i2s_std_clk_config_t clock=I2S_STD_CLK_DEFAULT_CONFIG(48000);
    clock.clk_src=I2S_CLK_SRC_XTAL;
    clock.mclk_multiple=I2S_MCLK_MULTIPLE_384;
    e=i2s_channel_reconfig_std_clock(s_tx1,&clock);
    if (e==ESP_OK) s_clock_parked=true;
    publish_error(e);
    return e;
}
/* Only called on core1, between PCM descriptors. The common geometry caller
 * parks both ports first and restores their original run/READY states last. */
static esp_err_t reconfigure(uint32_t rate,int bits,int fmt) {
    if (rate<8000 || rate>96000 || (bits!=16 && bits!=24 && bits!=32)) return ESP_ERR_INVALID_ARG;
    if (fmt<0 || fmt>1) return ESP_ERR_NOT_SUPPORTED;
    i2s1_out_stats_t old;i2s1_out_get_stats(&old);
    if (!s_clock_parked && old.rate==rate && old.slot_bits==bits && old.format==fmt) return ESP_OK;
    g_i2s1_reconfigs++;
    ESP_LOGW(TAG,"P1 reconfig run=%d old=%u/%d/%d new=%u/%d/%d",(int)old.running,
        (unsigned)old.rate,old.slot_bits,old.format,(unsigned)rate,bits,fmt);
    if (!s_tx1) {
        portENTER_CRITICAL(&s_mux);s_state.rate=rate;s_state.slot_bits=bits;s_state.format=fmt;portEXIT_CRITICAL(&s_mux);
        return ESP_OK;
    }
    if (old.running) {
        esp_err_t e=i2s_channel_disable(s_tx1);
        if (e!=ESP_OK) {publish_error(e);return e;}
        portENTER_CRITICAL(&s_mux);s_state.running=false;portEXIT_CRITICAL(&s_mux);
    }
    i2s_std_slot_config_t slot=make_slot(fmt,bits);
    i2s_std_clk_config_t clock=make_clock(rate,bits);
    esp_err_t e=i2s_channel_reconfig_std_slot(s_tx1,&slot);
    if (e==ESP_OK) e=i2s_channel_reconfig_std_clock(s_tx1,&clock);
    if (e!=ESP_OK) {
        i2s_std_slot_config_t prior_slot=make_slot(old.format,old.slot_bits);
        i2s_std_clk_config_t prior_clock=make_clock(old.rate,old.slot_bits);
        esp_err_t restore=i2s_channel_reconfig_std_slot(s_tx1,&prior_slot);
        if (restore==ESP_OK) restore=i2s_channel_reconfig_std_clock(s_tx1,&prior_clock);
        if (restore==ESP_OK) s_clock_parked=false;
        if (restore==ESP_OK && old.running) restore=i2s_channel_enable(s_tx1);
        portENTER_CRITICAL(&s_mux);s_state.running=old.running && restore==ESP_OK;s_state.last_error=e;portEXIT_CRITICAL(&s_mux);
        g_i2s1_set_err=e;
        return e;
    }
    s_clock_parked=false;
    preload_silence(bits);
    esp_err_t enabled=old.running ? i2s_channel_enable(s_tx1) : ESP_OK;
    portENTER_CRITICAL(&s_mux);
    s_state.rate=rate;s_state.slot_bits=bits;s_state.format=fmt;
    s_state.running=old.running && enabled==ESP_OK;s_state.last_error=enabled;
    portEXIT_CRITICAL(&s_mux);
    if (old.running && enabled==ESP_OK) g_i2s1_reenable++;
    g_i2s1_set_err=enabled;
    return enabled;
}
esp_err_t i2s1_out_core1_set_rate(uint32_t rate) {return reconfigure(rate,s_state.slot_bits,s_state.format);}
esp_err_t i2s1_out_core1_configure(uint32_t rate,int bits) {return reconfigure(rate,bits,s_state.format);}
esp_err_t i2s1_out_core1_set_slot_width(int bits) {return reconfigure(s_state.rate,bits,s_state.format);}
esp_err_t i2s1_out_core1_set_format(int fmt) {return reconfigure(s_state.rate,s_state.slot_bits,fmt);}
esp_err_t i2s1_out_write(const void *buf,size_t size,size_t *written) {
    size_t accepted=0;
    esp_err_t e=(!s_tx1 || !s_state.running || !buf || !size) ? ESP_ERR_INVALID_STATE :
        i2s_channel_write(s_tx1,buf,size,&accepted,1000);
    if (written) *written=accepted;
    portENTER_CRITICAL(&s_mux);
    s_state.calls++;s_state.bytes+=(uint32_t)accepted;
    if (e!=ESP_OK) s_state.errors++;
    if (accepted!=size) s_state.partial++;
    s_state.last_error=e;
    portEXIT_CRITICAL(&s_mux);
    return e;
}
bool i2s1_out_is_ready(void) {return s_state.ready;}
bool i2s1_out_is_running(void) {return s_state.running;}
void i2s1_out_get_stats(i2s1_out_stats_t *stats) {
    if (!stats) return;
    portENTER_CRITICAL(&s_mux);*stats=s_state;portEXIT_CRITICAL(&s_mux);
}
