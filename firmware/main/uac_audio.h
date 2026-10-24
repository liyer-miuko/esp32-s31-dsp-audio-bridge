/*
 * 自研 ASIO 式 UAC2 音频流 - ESP32-S31
 *
 * 设计核心（对照官方 usb_device_uac 组件的爆音根因）：
 * 1. 深环形缓冲（48KB ~ 60ms@96k）：ISR 只做 memcpy，USB 抖动全部吸收
 * 2. 数据驱动写任务：阻塞等信号量 -> 收空 ring -> i2s 直写（无轮询）
 * 3. UAC2 异步模式 + Feedback EP：设备 I2S/APLL 时钟为权威时钟源，
 *    Windows 依据 16.16 feedback 值自适应发送速率（ASIO 设备行为）
 * 4. 采样率/格式切换全部由独立控制任务执行（绝不在 USB 回调里 reconfig）
 * 5. 24/32bit 均 4 字节子槽直通，零重采样、零逐样本处理
 */
#ifndef UAC_AUDIO_H_
#define UAC_AUDIO_H_

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化 UAC2 设备（USB PHY + TinyUSB + ring + 任务）
 */
esp_err_t uac_audio_init(void);

#ifdef __cplusplus
}
#endif

#endif /* UAC_AUDIO_H_ */
