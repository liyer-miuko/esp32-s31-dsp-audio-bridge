/*
 * Hardware mute output (stage C1) - implementation. See hw_mute.h for the
 * spec references and the reasoning behind the ordering requirement.
 *
 * "Playing" detection is deliberately HOOK-FREE: the task samples one counter
 * that uac2_audio.c already keeps:
 *
 *   pcm_active_runs - advances while ANY input path is really sending PCM
 *                     (UAC endpoint open and the host filling the FIFO, or a
 *                     decoded BT block delivered)
 *
 *   playing := that counter advanced since the previous sample
 *
 * ★ 0.113（用户判据）：MUTE 的活动信号 = UAC 有效 PCM 流 **OR** BT 有效 PCM 流。
 *   全零 PCM 仍是流；"连接上了"不算。旧写法用"选中输出的 pump 在推进 && nonaudio
 *   没推进"，于是 BT 第一次接管时被无条件压 2 s 静音窗（UAC 正在播也被吃掉）；
 *   现在那个强制窗已删除，`nonaudio_runs` 只作诊断保留。
 */
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "dsp_config.h"
#include "uac2_audio.h"
#include "hw_mute.h"

static const char *TAG = "hw_mute";

/* Current applied state + the parameters copied out of dsp_config. */
static volatile bool     s_muted        = true;
static volatile bool     s_playing      = false;
static volatile uint8_t  s_polarity     = 0;      /* 0 = low = muted (default) */
static volatile uint16_t s_delay_a_ms   = DSP_HW_DELAY_DEFAULT;
static volatile uint16_t s_delay_b_ms   = DSP_HW_DELAY_DEFAULT;
static volatile uint32_t s_transitions  = 0;
static volatile int64_t  s_last_change  = 0;      /* esp_timer_get_time() */
static volatile int64_t  s_last_playing = 0;      /* 最近一次 playing 跳变的时刻 */
static bool              s_task_started = false;
/* ★ L43-A：BT 连接窗强制静音截止时刻（0 = 无强制）；窗内 muted 语义生效。 */
static volatile int64_t  s_force_until_us = 0;

int hw_mute_level_for(bool muted, uint8_t polarity)
{
    const int active = (polarity != 0) ? 1 : 0;   /* level that means "muted" */
    return muted ? active : (active ? 0 : 1);
}

static void hw_mute_write(bool muted)
{
    const int level = hw_mute_level_for(muted, s_polarity);
    gpio_set_level(HW_MUTE_GPIO, level);
    s_muted       = muted;
    s_last_change = esp_timer_get_time();
    s_transitions++;
    ESP_LOGI(TAG, "%s (pin GPIO%d -> %d, polarity=%u, t=%lld us)",
             muted ? "MUTED" : "unmuted", HW_MUTE_GPIO, level,
             (unsigned)s_polarity, (long long)s_last_change);
}

esp_err_t hw_mute_early_init(void)
{
    const gpio_config_t cfg = {
        .pin_bit_mask = (1ULL << HW_MUTE_GPIO),
        /* ★ 必须是 INPUT_OUTPUT：纯 OUTPUT 时**输入通路是断开的**，
         *   `gpio_get_level()` 读到的是输入寄存器（恒 0），于是"引脚实测电平"
         *   这个诊断字段会永远报 0 —— 实测踩到：极性反转后引脚其实已经是高，
         *   但 0xAF 仍报 pin=0（诊断在撒谎，而不是驱动错了）。
         *   带输入通路没有代价，却让"引脚到底什么电平"可自证。 */
        .mode         = GPIO_MODE_INPUT_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,   /* external 100k does this */
        .intr_type    = GPIO_INTR_DISABLE,
    };
    esp_err_t err = gpio_config(&cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "gpio_config(GPIO%d) failed: %s", HW_MUTE_GPIO, esp_err_to_name(err));
        return err;
    }

    /* Drive the MUTED level immediately, using the built-in default polarity.
     * The stored polarity is applied later by hw_mute_apply_config(). */
    s_polarity = 0;
    gpio_set_level(HW_MUTE_GPIO, hw_mute_level_for(true, 0));
    s_muted       = true;
    s_last_change = esp_timer_get_time();
    ESP_LOGW(TAG, "early init: GPIO%d driven to MUTED level (%d), before I2S/DSP exist",
             HW_MUTE_GPIO, hw_mute_level_for(true, 0));
    return ESP_OK;
}

void hw_mute_apply_config(void)
{
    const uint8_t  pol = dsp_config_hw_mute_polarity();
    const uint16_t a   = dsp_config_hw_mute_delay_a_ms();
    const uint16_t b   = dsp_config_hw_mute_delay_b_ms();
    const bool     pol_changed = (pol != s_polarity);

    s_delay_a_ms = a;
    s_delay_b_ms = b;
    s_polarity   = pol;

    if (pol_changed) {
        /* Keep the physical level honest: while muted the pin must show the
         * (new) active level; while unmuted the (new) inactive level. */
        gpio_set_level(HW_MUTE_GPIO, hw_mute_level_for(s_muted, pol));
        ESP_LOGW(TAG, "polarity -> %u (%s muted); pin re-driven to %d (muted=%d)",
                 (unsigned)pol, (pol != 0) ? "high = " : "low = ",
                 hw_mute_level_for(s_muted, pol), (int)s_muted);
    }
    ESP_LOGI(TAG, "config: polarity=%u delayA=%u ms delayB=%u ms",
             (unsigned)pol, (unsigned)a, (unsigned)b);
}

static void hw_mute_task(void *arg)
{
    (void)arg;

    /* Seed the counter; the very first tick must not look like "playback
     * started" (that would unmute during boot). */
    uint32_t act_prev = uac2_audio_get_pcm_active_tick();
    bool     playing_prev = false;

    bool    pending       = false;
    bool    target_muted  = true;
    int64_t deadline_us   = 0;

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(HW_MUTE_TASK_PERIOD_MS));

        const uint32_t act = uac2_audio_get_pcm_active_tick();
        const bool playing = (act != act_prev);
        act_prev = act;
        s_playing = playing;

        const int64_t now_us = esp_timer_get_time();

        if (playing != playing_prev) {
            playing_prev = playing;
            s_last_playing = esp_timer_get_time();
            const bool want_muted = !playing;
            /* One pending delay at a time: the new event cancels the old and
             * the latest state wins (spec §1.3 arbitration). */
            target_muted = want_muted;
            deadline_us  = esp_timer_get_time() +
                           (int64_t)(want_muted ? s_delay_a_ms : s_delay_b_ms) * 1000;
            pending      = true;
            ESP_LOGD(TAG, "event: playing=%d -> will %s after %u ms",
                     (int)playing, want_muted ? "MUTE" : "unmute",
                     (unsigned)(want_muted ? s_delay_a_ms : s_delay_b_ms));
        }

        if (pending && esp_timer_get_time() >= deadline_us) {
            pending = false;
            if (target_muted != s_muted) {
                hw_mute_write(target_muted);
            }
        }
    }
}

esp_err_t hw_mute_start(void)
{
    if (s_task_started) {
        return ESP_OK;
    }
    hw_mute_apply_config();   /* adopt the stored polarity/delays first */

    /* core0, lowest useful priority: this task must never compete with the
     * I2S pump or the USB stack, and it must never touch core1 (spec §2: no
     * additional core1 load). */
    if (xTaskCreatePinnedToCore(hw_mute_task, "hw_mute", 2560, NULL, 1, NULL, 0) != pdPASS) {
        ESP_LOGE(TAG, "task create failed");
        return ESP_ERR_NO_MEM;
    }
    s_task_started = true;
    ESP_LOGI(TAG, "task started (core0, prio 1, period %d ms)", HW_MUTE_TASK_PERIOD_MS);
    return ESP_OK;
}

/* ★ L43-A：BT 连接窗预静音（ms>0 延长取 max；ms==0 显式清窗）。见 hw_mute.h。 */
void hw_mute_force_mute_window(uint32_t ms)
{
    if (ms == 0) {
        s_force_until_us = 0;
        ESP_LOGW(TAG, "quiet window cleared");
        return;
    }
    const int64_t until = esp_timer_get_time() + (int64_t)ms * 1000;
    if (until > s_force_until_us) {
        s_force_until_us = until;
    }
    ESP_LOGW(TAG, "quiet window: force MUTED for %u ms (BT connect churn cover)", (unsigned)ms);
}

/* ---------------- diagnostics ---------------- */
bool     hw_mute_is_muted(void)      { return s_muted; }
int      hw_mute_pin_level(void)     { return gpio_get_level(HW_MUTE_GPIO); }
bool     hw_mute_is_playing(void)    { return s_playing; }
uint8_t  hw_mute_polarity(void)      { return s_polarity; }
uint16_t hw_mute_delay_a_ms(void)    { return s_delay_a_ms; }
uint16_t hw_mute_delay_b_ms(void)    { return s_delay_b_ms; }
uint32_t hw_mute_transitions(void)   { return s_transitions; }

uint32_t hw_mute_since_last_change_us(void)
{
    const int64_t d = esp_timer_get_time() - s_last_change;
    return (d < 0) ? 0u : (uint32_t)d;
}

/* ★ 为什么需要它（v38 实测踩出来的）：主机侧无论怎么轮询，"playing 边沿"都只能
 * 定位到一个轮询窗口内（本机串口 f.read(4096) 的阻塞让窗口达 ~300 ms），
 * 于是"起播 → 解除静音"只能给一个 ±300 ms 的区间，分不开 0 / 30 ms 两档。
 * 把**边沿时刻也放到设备侧**之后，两个时刻出自**同一帧**：
 *     间隔 = playing_since_us − since_last_change_us
 * 与主机时钟、轮询周期、串口往返**全都无关**，精度 = 状态机周期（10 ms）。 */
uint32_t hw_mute_playing_since_us(void)
{
    const int64_t d = esp_timer_get_time() - s_last_playing;
    return (d < 0) ? 0u : (uint32_t)d;
}
