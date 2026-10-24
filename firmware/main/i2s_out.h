/*
 * I2S0 输出驱动 - ESP32-S31 UAC 音频桥
 *
 * 引脚分配（GPIO6 已损坏，DOUT 避开）：
 *   MCLK = GPIO2（供 TAS5828/外部 DAC，256fs）
 *   BCLK = GPIO3
 *   WS   = GPIO4
 *   DOUT = GPIO7
 *
 * 特性：
 * - master 模式，APLL 时钟源，采样率跟随 USB（44.1k~96k）
 * - 32bit slot 直通（24bit 数据为 24-in-32，零转换）
 * - auto_clear_before_cb=true（防 DMA 残留循环噪音）
 * - 无锁设计：reconfig 前 disable 通道，i2s_channel_write 对
 *   disabled 通道立即返回 ESP_ERR_INVALID_STATE -> 写任务感知后重试
 */
#ifndef I2S_OUT_H_
#define I2S_OUT_H_

#include <stdint.h>
#include <stddef.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 引脚定义 */
#define I2S_OUT_GPIO_MCLK   2
#define I2S_OUT_GPIO_BCLK   3
#define I2S_OUT_GPIO_WS     4
#define I2S_OUT_GPIO_DOUT   7   /* 原 GPIO6 已损坏，改 GPIO7 */

/**
 * @brief 初始化 I2S0（默认 48k / 32bit 并启用）
 */
esp_err_t i2s_out_init(void);

/**
 * @brief 写入 PCM 数据（阻塞直到全部写入 DMA）
 * @note 通道被 disable（reconfig 中）时返回 ESP_ERR_INVALID_STATE，
 *       调用方应短暂等待后重试
 */
esp_err_t i2s_out_write(const void *buf, size_t size);

/**
 * @brief 重配置采样率与位深（控制任务上下文调用）
 * @param rate 采样率 Hz（44100/48000/88200/96000）
 * @param bits 位深（24 或 32；24 = 24bit-in-32bit slot）
 */
esp_err_t i2s_out_reconfig(uint32_t rate, int bits);

/**
 * @brief 当前是否正在运行（enable 状态）
 */
bool i2s_out_is_running(void);

#ifdef __cplusplus
}
#endif

#endif /* I2S_OUT_H_ */
