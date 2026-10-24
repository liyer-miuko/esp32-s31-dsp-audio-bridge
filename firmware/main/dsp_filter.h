/*
 * Biquad filter design and processing - ESP32-S31 audio bridge.
 *
 * The design equations are the RBJ audio-EQ cookbook, ported 1:1 from the
 * WebUI (html前端/dsp webui.html, function coeff()) so that the drawn response
 * curve is the response the firmware actually produces. Crossover stage
 * derivation mirrors the WebUI's xoStageList()/butterQ() helpers exactly.
 *
 * Processing is single precision: the S31 core has a hardware single-precision
 * FPU (rv32imafc) but no hardware double, so float keeps the per-sample cost
 * bounded. States are kept separately from coefficients so one designed
 * coefficient set can drive the left and right state pairs.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_attr.h"

#include "dsp_config.h"

#ifdef __cplusplus
extern "C" {
#endif

#define DSP_XO_MAX_STAGES 4

typedef struct {
    float b0, b1, b2, a1, a2;   /* a0 normalised to 1 */
} dsp_biquad_coeff_t;

typedef struct {
    float z1, z2;
} dsp_biquad_state_t;

typedef struct {
    dsp_biquad_coeff_t c;
    dsp_biquad_state_t st;
} dsp_biquad_t;

/* Design one RBJ biquad. type = dsp_filter_type_t, g_db is ignored by
 * lowpass/highpass/bandpass/notch/allpass, q is the raw Q. */
void dsp_biquad_design(dsp_biquad_coeff_t *c, uint8_t type, float f, float g_db, float q, uint32_t fs);

/* Design a crossover as a cascade of 1..DSP_XO_MAX_STAGES biquads.
 * Returns the number of stages (0 when the crossover is off). */
uint8_t dsp_xo_design(dsp_biquad_coeff_t *stages, const dsp_xo_t *xo, bool is_highpass, uint32_t fs);

/* ⚠ v0.29 起热路径全部 IRAM_ATTR（见 dsp_engine.h 的说明）。
 * `dsp_biquad_s31.S` 保持本函数在固定工具链中的逐条浮点运算顺序；
 * 改递归算式时须同时核对该内核。 */
static inline IRAM_ATTR float dsp_biquad_process(const dsp_biquad_coeff_t *c, dsp_biquad_state_t *s, float x)
{
    const float y = c->b0 * x + s->z1;
    s->z1 = c->b1 * x - c->a1 * y + s->z2;
    s->z2 = c->b2 * x - c->a2 * y;
    return y;
}

/* ---------------- 快速 log2 / exp2（v0.29：压缩器去 libm） ----------------
 * RISC-V 上没有 log10f/powf 的硬件指令，libm 的软件实现每次约 400 周期；
 * 压缩器**每采样各调一次** ⇒ 847 µs 里的大部分（实测模型，见 HANDOFF §4.7）。
 * 这里用"2 的幂分解 + 尾数多项式"代替：
 *     20·log10(x) = 20·log2(x)·log10(2) = DSP_LOG2_10   · log2(x)
 *     10^(g/20)   = exp2(g·log2(10)/20) = exp2(DSP_LOG2_10_20 · g)
 *
 * 精度（float32 + Horner **实测**，不是拿 double 充数）——
 *   复算脚本 `测试与抓包\v32_dsp_perf\fit_fastmath.py`：
 *     · log2 尾数 7 阶：最大绝对误差 **4.06e-07**（log2 单位）⇒ 2.4e-06 dB
 *     · exp2 尾数 7 阶：最大相对误差 **9.64e-08**（已是 float32 舍入地板）
 *     · **端到端静态增益律最大偏差 1.0e-05 dB**（100 万点扫描：24k 输入电平 ×
 *       6 阈值 × 7 压缩比），规格要求 < 1e-3 dB ⇒ 余量 100×
 * 每个都是 7 次乘加 + 几次位操作，无分支、无查表、无 libm 调用。 */
#define DSP_LOG2_10     6.020599913279624f    /* 20·log10(2)      */
#define DSP_LOG2_10_20  0.16609640474436813f  /* log2(10)/20      */

/* log2: x = m·2^e（m∈[1,2)）⇒ log2(x) = e + p(m-1) */
static inline IRAM_ATTR float dsp_fast_log2(float x)
{
    union { float f; uint32_t u; } v;
    v.f = x;
    const int32_t e = (int32_t)((v.u >> 23) & 0xFFu) - 127;
    v.u = (v.u & 0x007FFFFFu) | 0x3F800000u;      /* 尾数 -> [1,2) */
    const float t = v.f - 1.0f;
    float p = 1.47787211e-02f;
    p = p * t + (-7.68487230e-02f);
    p = p * t + 1.90420836e-01f;
    p = p * t + (-3.23115945e-01f);
    p = p * t + 4.72499520e-01f;
    p = p * t + (-7.20386624e-01f);
    p = p * t + 1.44265211e+00f;
    p = p * t + 3.19697818e-07f;
    return (float)e + p;
}

/* exp2: y = n + f（n=floor(y)，f∈[0,1)）⇒ 2^n · q(f) */
static inline IRAM_ATTR float dsp_fast_exp2(float y)
{
    /* 压缩器实际取值 y = comp_gain_db·0.166，comp_gain_db∈[-60,60] ⇒ y∈[-10,10]；
     * 这里钳到 ±120 只是防御异常配置（端口增益把输入推到 0 dBFS 以上）。 */
    if (!(y > -120.0f)) { y = -120.0f; }
    if (y > 120.0f)     { y = 120.0f; }

    int32_t n = (int32_t)y;                   /* 向零截断 */
    float   nf = (float)n;
    if (nf > y) { n -= 1; nf -= 1.0f; }       /* 负数补一步 ⇒ 等价 floor */
    const float f = y - nf;                   /* [0,1) */

    float p = 2.16517237e-05f;
    p = p * f + 1.42994017e-04f;
    p = p * f + 1.34298112e-03f;
    p = p * f + 9.61353537e-03f;
    p = p * f + 5.55052906e-02f;
    p = p * f + 2.40226358e-01f;
    p = p * f + 6.93147182e-01f;
    p = p * f + 1.0f;

    union { float f; uint32_t u; } v;
    v.u = (uint32_t)(n + 127) << 23;          /* n∈[-120,120] ⇒ 指数域合法 */
    return p * v.f;
}

static inline void dsp_biquad_reset(dsp_biquad_state_t *s)
{
    s->z1 = 0.0f;
    s->z2 = 0.0f;
}

static inline bool dsp_biquad_is_passthrough(const dsp_biquad_coeff_t *c)
{
    return (c->b0 == 1.0f && c->b1 == 0.0f && c->b2 == 0.0f && c->a1 == 0.0f && c->a2 == 0.0f);
}

#ifdef __cplusplus
}
#endif
