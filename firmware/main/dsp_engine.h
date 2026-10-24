/*
 * DSP engine - ESP32-S31 audio bridge.
 *
 * Signal flow (matches html前端/dsp webui.html):
 *
 *   USB / BT stereo block
 *     -> 输入级: 增益 / 静音 / 反相 + 输入 PEQ + 房间 EQ (per source, stereo)
 *     -> 路由矩阵 2x4: 每格 = 开关 + 源声道选择(L/R/单声道) + 增益
 *        （默认与旧语义一致：通道 1 取源 L，通道 2 取源 R）
 *     -> DSP 通道 1..4: PEQ + HP + LP（各自独立的单声道链路）
 *     -> 主音量
 *     -> CH1/2 响度补偿 -> I2S0声道配置 -> 端口HP/LP -> 输出增益
 *        -> 压缩器 -> 削波保护（立体声联动）-> 延时 -> int32槽数据
 *     -> core1本地格式转换与I2S提交；I2S1 输出 CH3/4；中性旁通时镜像输入，port1 HP/LP/增益/延时不在本轮实现
 *
 * 所有参数来自 dsp_config 的只读快照；core0 协议任务提交控制命令，core1
 * 在块边界调用 dsp_engine_sync() 重建系数。系数与状态都由 core1 执行处理。
 *
 * Bypass: when nothing is configured the engine leaves the block untouched,
 * which keeps 0 dB UAC playback bit-perfect (the pump's int32 block is not
 * round-tripped through float).
 */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

/* Shared source enum without depending on the worker's handshake header.
 * Core0 input/control and core1 processing use these same values. */
typedef enum {
    DSP_SOURCE_UAC = 0,
    DSP_SOURCE_BT = 1,
    DSP_SOURCE_TEST = 2,
} dsp_source_t;

#include "dsp_config.h"
#include "dsp_filter.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Filter dump record kinds (protocol diagnostics). */
typedef enum {
    DSP_FILTER_KIND_INPUT_PEQ = 0,
    DSP_FILTER_KIND_INPUT_REQ = 1,
    DSP_FILTER_KIND_CHANNEL_PEQ = 2,
    DSP_FILTER_KIND_CHANNEL_HP = 3,
    DSP_FILTER_KIND_CHANNEL_LP = 4,
    DSP_FILTER_KIND_PORT_HP = 5,     /* a = 端口号（系数两槽共用） */
    DSP_FILTER_KIND_PORT_LP = 6,
    DSP_FILTER_KIND_LOUDNESS = 7,    /* 响度补偿：idx 0 = 低架 / 1 = 高架 */
} dsp_filter_kind_t;

typedef struct {
    uint8_t kind;
    uint8_t a;      /* source or channel index */
    uint8_t b;      /* reserved */
    uint8_t idx;    /* band / stage index */
    dsp_biquad_coeff_t c;
} dsp_filter_dump_t;

esp_err_t dsp_engine_init(void);

/* Rebuild coefficients/states from the current dsp_config snapshot.
 * The core1 worker invokes this at a block boundary; core0 control callers
 * submit through the dsp_audio control interface instead of rebuilding here. */
void dsp_engine_sync(void);

/* Process one interleaved stereo int32 block in place (DSP task only).
 * ⚠ v0.29 起整条热路径（本函数 + 它调用的内层）**放在 IRAM**：旧版跑在 flash
 *   （链接后地址 0x4001a0b8，IROM 区间），每块都要吃 flash cache 抖动，
 *   而 max/avg 的那点余量本来就是留给它的；IRAM 只用了 84 KB / 512 KB。
 *   数值零影响（同一份代码换一个段），只是取指不再走 cache。
 * ⚠ `IRAM_ATTR` **只写在 dsp_engine.c 的定义上**，不能两边都写 ——
 *   IDF v6.1 的 `IRAM_ATTR` 展开成 `.iram1.<__COUNTER__>`，两处会拿到不同的
 *   段名，GCC 直接报 `-Werror=attributes`（已实测踩过）。 */
void dsp_engine_process(int32_t *pcm, int32_t *pcm_port1, size_t samples, uint32_t rate, dsp_source_t source);

/* Clear filter states and delay lines (source switch / stream restart). */
void dsp_engine_reset_state(void);

/* Number of biquads currently active (diagnostics). */
uint32_t dsp_engine_active_filter_count(void);

/* True when the engine is a bit-perfect passthrough for the current config. */
bool dsp_engine_is_bypass(void);

/* Clip counter (samples clamped by the engine). */
uint32_t dsp_engine_get_clips(void);

/* 限幅器当前增益衰减（dB，0 = 未动作；返回正值表示压了多少 dB）。 */
float dsp_engine_get_limiter_gr_db(void);

/* 响度补偿当前生效的补偿量（dB，0 = 未开启或主音量高于参考点）。 */
float dsp_engine_loudness_boost_db(void);

/* ⚠ **单元的"是否开启"必须用下面两个，不要拿上面的量去推断**：
 *   响度 boost 在"主音量高于参考点"时本来就等于 0，用它判开启会漏算 266 µs/块；
 *   限幅增益衰减在阈值以下也是 0，同理。CPU 预算计算（dsp_config_biquad_limit）
 *   依赖这两个开关状态，判错会直接导致预算算少 → 过载丢块（v24 的 C8）。 */
bool dsp_engine_loudness_enabled(void);
bool dsp_engine_compressor_enabled(void);

/* Diagnostics: state resets / coefficient rebuilds / process calls. */
void dsp_engine_get_diag(uint32_t *resets, uint32_t *rebuilds, uint32_t *calls);

/* Diagnostics: input-stream discontinuities (block boundary / mid-block). */
void dsp_engine_get_jumps(uint32_t *boundary, uint32_t *mid);

/* Enumerate the designed biquads for the filter-dump protocol.
 * Returns the number of records written (<= max). */
size_t dsp_engine_collect_filters(dsp_filter_dump_t *out, size_t max);

#ifdef __cplusplus
}
#endif
