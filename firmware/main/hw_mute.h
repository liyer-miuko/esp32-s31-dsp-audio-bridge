/*
 * Hardware mute output (stage C1) - ESP32-S31 audio bridge.
 *
 * Drives the TAS5828 MUTE pin from GPIO8. Spec: 规格_硬件控制层_Key与Mute.md §1
 * (v8/v9). Electrical facts (verified by the user, 规格 §1.1):
 *   - TAS5828 MUTE pin: 3.3 V .. 1.8 V logic, absolute max 3.6 V
 *   - **low = muted**, high = play
 *   - a 100 kOhm pull-down on the ESP side keeps it muted while the MCU is
 *     unpowered / in reset / being flashed (spec §1.2)
 *
 * ★ The hard requirement (spec §1.4): the pin must be driven to its MUTED
 * level **at the very beginning of the init sequence** - before the I2S
 * channel is created, before the DSP task exists, before anything can make
 * DOUT carry activity. Reason: while the MCU pin is high-Z the amp's own
 * pull-down protects us, but the moment we configure it as an output the
 * level is OUR choice; if I2S were already clocking garbage at that point the
 * amp would play it, and no "30 ms delay" could save the design.
 *
 * Behaviour (parameters 0x81/0x82/0x84, stored by dsp_config):
 *   - delay A (0x81): stop playing  -> apply mute after A ms
 *   - delay B (0x84): start playing -> release mute after B ms
 *   - polarity (0x82): 0 = low = muted (default), 1 = high = muted
 * Arbitration: only ONE delay may be pending; a new event cancels the old one
 * and the LATEST state wins (so a stop/start pair inside the delay window ends
 * up unmuted, without ever having muted in between -> no click).
 *
 * Mute is NOT implemented by writing P_MASTER_VOL (spec §0.0 hard constraint 1).
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

/* GPIO8 is the frozen B1 pin-table choice (规格_B1引脚候选表.md):
 * "上电 / 未播放 = 低电平（静音）；播放并过延时 B 之后才拉高（不静音）". */
#define HW_MUTE_GPIO              8

/* Sampling period of the state machine. Spec asks for a low-priority 5~10 ms
 * task; ⚠ This period is also the
 * TIMING QUANTISATION of both delays: an edge is observed on the first tick
 * after the event, so a measured delay is [A, A + period] plus scheduler
 * jitter. The LA acceptance must use that tolerance (documented in the report). */
#define HW_MUTE_TASK_PERIOD_MS    10

/* Pin truth: muted ? active_level : !active_level, active_level = polarity ? 1 : 0 */
int  hw_mute_level_for(bool muted, uint8_t polarity);

/* Earliest init: configure the pin as an output and drive DEFAULT-polarity
 * MUTED. Must be called before i2s_out_init(). Uses the built-in defaults
 * (low-active, 30/30 ms) because the stored config is not loaded yet - "must be
 * muted even before the configuration is known" is the hard requirement. */
esp_err_t hw_mute_early_init(void);

/* Re-read polarity/delays from dsp_config and apply them. Called after an
 * accepted write to 0x81/0x82/0x84 (and once after dsp_config_init()). If we
 * are muted and the polarity changed, the pin is re-driven so the physical
 * level keeps telling the truth ("this output is muted right now"). */
void hw_mute_apply_config(void);

/* Create the state-machine task (core0, low priority). */
esp_err_t hw_mute_start(void);

/* ★ L43-A（2026-09-26，固件 0.47）：BT 连接窗预静音（**0.113 起不再被自动调用**）。
 * 0.113 的 MUTE 判据改成"任一路输入正在送 PCM"（uac2_audio_get_pcm_active_tick），
 * UAC 持续送流时 BT 连接/切编码不能再把功放静音；此接口只留给显式诊断使用，
 * 纯 UAC 基线与 BT 接管路径都不调用它。
 * 语义：ms>0 延长窗口（取 max）；ms==0 显式清窗。窗内新事件不打断强制。 */
#define HW_MUTE_BT_CONNECT_QUIET_MS  2000
void hw_mute_force_mute_window(uint32_t ms);

/* ---- diagnostics (also exposed to the host via frame 0xAF HW_STATUS) ---- */
bool     hw_mute_is_muted(void);
int      hw_mute_pin_level(void);
bool     hw_mute_is_playing(void);
uint8_t  hw_mute_polarity(void);
uint16_t hw_mute_delay_a_ms(void);
uint16_t hw_mute_delay_b_ms(void);
uint32_t hw_mute_transitions(void);
uint32_t hw_mute_since_last_change_us(void);
uint32_t hw_mute_playing_since_us(void);
