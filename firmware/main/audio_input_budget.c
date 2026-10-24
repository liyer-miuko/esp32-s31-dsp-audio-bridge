#include <stdint.h>
#include "sdkconfig.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "audio_input_budget.h"

/* The control task samples a fixed set of long-lived core0 tasks. Never format
 * an unbounded task list from the priority-24 audio pump. All values include
 * the sampled task's callbacks; they are NOT decoder-kernel timings.
 * The existing codec policy's unused percentage is a burst margin, not a
 * reservation or priority change. Missing task windows stay invalid. */
void audio_input_budget_poll(bool bt_selected)
{
#if configUSE_TRACE_FACILITY && configGENERATE_RUN_TIME_STATS && INCLUDE_xTaskGetHandle && CONFIG_FREERTOS_RUN_TIME_STATS_USING_ESP_TIMER
    static const char *names[] = {"A2DP_DECODER", "uac_pump", "tud"};
    static TaskHandle_t prior_handle[3];
    static UBaseType_t prior_number[3];
    static configRUN_TIME_COUNTER_TYPE prior_cpu[3], prior_idle;
    static int64_t prior_us;
    const int64_t now = esp_timer_get_time();
    if (prior_us && now - prior_us < 1000000) return;
    const uint64_t dt = prior_us ? (uint64_t)(now - prior_us) : 0;
    const configRUN_TIME_COUNTER_TYPE idle = ulTaskGetIdleRunTimeCounterForCore(0);
    const uint64_t idle_us = (configRUN_TIME_COUNTER_TYPE)(idle - prior_idle);
    uint64_t cpu[3] = {0};
    bool valid[3] = {false};
    for (unsigned i = 0; i < 3; ++i) {
        TaskHandle_t handle = xTaskGetHandle(names[i]);
        if (!handle) { prior_handle[i] = NULL; continue; }
        TaskStatus_t task = {0};
        vTaskGetInfo(handle, &task, pdFALSE, eInvalid);
        cpu[i] = (configRUN_TIME_COUNTER_TYPE)(task.ulRunTimeCounter - prior_cpu[i]);
        valid[i] = dt && handle == prior_handle[i] && task.xTaskNumber == prior_number[i] && cpu[i] <= dt;
        prior_handle[i] = handle; prior_number[i] = task.xTaskNumber;
        prior_cpu[i] = task.ulRunTimeCounter;
    }
    const bool budget_valid = dt && idle_us <= dt && valid[0] && cpu[0] + idle_us <= dt;
    const unsigned used = budget_valid ? (unsigned)(cpu[0] * 100u / dt) : 0;
    const unsigned capacity = budget_valid ? (unsigned)((idle_us + cpu[0]) * 100u / dt) : 0;
    const unsigned reserve = 100u - CONFIG_BT_A2DP_CODEC_BUDGET_PCT;
    const unsigned safe = capacity > reserve ? capacity - reserve : 0;
    if (dt) ESP_LOGI("core0-budget", "dt_us=%llu bt_selected=%u idle_us=%llu dec_us=%llu pump_us=%llu usb_task_us=%llu dec_valid=%u pump_valid=%u usb_valid=%u budget_valid=%u decode_used_pct=%u decode_capacity_pct=%u decode_safe_pct=%u reserve_pct=%u scope=TASK_NOT_KERNEL",
        (unsigned long long)dt, bt_selected, (unsigned long long)idle_us,
        (unsigned long long)cpu[0], (unsigned long long)cpu[1], (unsigned long long)cpu[2],
        valid[0], valid[1], valid[2], budget_valid, used, capacity, safe, reserve);
    prior_idle = idle; prior_us = now;
#else
    (void)bt_selected;
#endif
}
