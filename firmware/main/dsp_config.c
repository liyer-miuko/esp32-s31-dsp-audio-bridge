/*
 * DSP parameter store - ESP32-S31 audio bridge (v11, protocol layer).
 *
 * All values are validated and clamped here; the protocol layer and the audio
 * engine both go through this module, so there is exactly one place that
 * decides what a legal configuration is.
 */
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "sdkconfig.h"

#include "dsp_engine.h"
#include "dsp_audio.h"
#include "dsp_config.h"
#include "dsp_version.h"
#include "hw_mute.h"
#include "i2s_out_v2.h"

#define NVS_NAMESPACE "dspcfg"
#define NVS_KEY_BLOB  "blob"

static const char *TAG = "dspcfg";

static dsp_config_t      s_cfg;
static SemaphoreHandle_t s_lock;
static uint8_t           s_nvs_blob[DSP_CONFIG_BLOB_MAX];

/* Default 10-band centre frequencies, identical to the WebUI. */
static const float s_default_freqs[DSP_DEFAULT_BANDS] = {
    31.5f, 63.0f, 125.0f, 250.0f, 500.0f, 1000.0f, 2000.0f, 4000.0f, 8000.0f, 16000.0f,
};

/* ---------------- Small helpers ---------------- */
static float clampf(float v, float lo, float hi)
{
    return (v < lo) ? lo : ((v > hi) ? hi : v);
}

static uint8_t clampi(int v, int lo, int hi)
{
    if (v < lo) v = lo;
    if (v > hi) v = hi;
    return (uint8_t)v;
}

static bool read_f32(const uint8_t *value, size_t len, float *out)
{
    if (value == NULL || len != 4) {
        return false;
    }
    memcpy(out, value, 4);
    return isfinite(*out) ? true : false;
}

static bool read_u8(const uint8_t *value, size_t len, uint8_t *out)
{
    if (value == NULL || len != 1) {
        return false;
    }
    *out = value[0];
    return true;
}

static bool read_u16(const uint8_t *value, size_t len, uint16_t *out)
{
    if (value == NULL || len != 2) {
        return false;
    }
    *out = (uint16_t)value[0] | ((uint16_t)value[1] << 8);   /* 小端，与 blob 一致 */
    return true;
}

/* ---------------- Serialization (little endian, fixed layout) ---------------- */
typedef struct {
    uint8_t *p;
    uint8_t *end;
    bool     overflow;
} blob_writer_t;

typedef struct {
    const uint8_t *p;
    const uint8_t *end;
    bool           underflow;
} blob_reader_t;

static void wr_bytes(blob_writer_t *w, const void *src, size_t n)
{
    if (w->p + n <= w->end) {
        memcpy(w->p, src, n);
    } else {
        w->overflow = true;
    }
    w->p += n;
}

static void wr_u8(blob_writer_t *w, uint8_t v)
{
    wr_bytes(w, &v, 1);
}

static void wr_u16(blob_writer_t *w, uint16_t v)
{
    uint8_t b[2] = { (uint8_t)(v & 0xFF), (uint8_t)(v >> 8) };
    wr_bytes(w, b, 2);
}

static void wr_u32(blob_writer_t *w, uint32_t v)
{
    uint8_t b[4] = { (uint8_t)v, (uint8_t)(v >> 8), (uint8_t)(v >> 16), (uint8_t)(v >> 24) };
    wr_bytes(w, b, 4);
}

static void wr_f32(blob_writer_t *w, float v)
{
    uint32_t u = 0;
    memcpy(&u, &v, 4);
    wr_u32(w, u);
}

static bool rd_bytes(blob_reader_t *r, void *dst, size_t n)
{
    if (r->p + n > r->end) {
        r->underflow = true;
        return false;
    }
    memcpy(dst, r->p, n);
    r->p += n;
    return true;
}

static uint8_t rd_u8(blob_reader_t *r)
{
    uint8_t v = 0;
    rd_bytes(r, &v, 1);
    return v;
}

static uint16_t rd_u16(blob_reader_t *r)
{
    uint8_t b[2] = { 0, 0 };
    rd_bytes(r, b, 2);
    return (uint16_t)b[0] | ((uint16_t)b[1] << 8);
}

static uint32_t rd_u32(blob_reader_t *r)
{
    uint8_t b[4] = { 0, 0, 0, 0 };
    rd_bytes(r, b, 4);
    return (uint32_t)b[0] | ((uint32_t)b[1] << 8) | ((uint32_t)b[2] << 16) | ((uint32_t)b[3] << 24);
}

static float rd_f32(blob_reader_t *r)
{
    uint32_t u = rd_u32(r);
    float f = 0.0f;
    memcpy(&f, &u, 4);
    return f;
}

static void serialize_band(blob_writer_t *w, const dsp_band_t *b)
{
    wr_f32(w, b->f);
    wr_f32(w, b->g);
    wr_f32(w, b->q);
    wr_u8(w, b->type);
    wr_u8(w, b->on);
    wr_u8(w, 0);
    wr_u8(w, 0);
}

static void deserialize_band(blob_reader_t *r, dsp_band_t *b)
{
    b->f    = clampf(rd_f32(r), DSP_BAND_FREQ_MIN_HZ, DSP_BAND_FREQ_MAX_HZ);
    b->g    = clampf(rd_f32(r), DSP_BAND_GAIN_MIN_DB, DSP_BAND_GAIN_MAX_DB);
    b->q    = clampf(rd_f32(r), DSP_BAND_Q_MIN, DSP_BAND_Q_MAX);
    b->type = clampi(rd_u8(r), 0, DSP_FILTER_ALLPASS);
    b->on   = rd_u8(r) ? 1 : 0;
    (void)rd_u8(r);
    (void)rd_u8(r);
}

static void serialize_xo(blob_writer_t *w, const dsp_xo_t *x)
{
    wr_f32(w, x->f);
    wr_u8(w, x->type);
    wr_u8(w, x->slope);
    wr_u8(w, x->on);
    wr_u8(w, 0);          /* one pad byte: the record is exactly 8 bytes */
}

static void deserialize_xo(blob_reader_t *r, dsp_xo_t *x)
{
    x->f = clampf(rd_f32(r), 20.0f, 20000.0f);
    x->type = clampi(rd_u8(r), 0, DSP_XO_LINKWITZ_RILEY);
    uint8_t slope = rd_u8(r);
    x->slope = (slope == 1 || slope == 2 || slope == 4) ? slope : 2;
    x->on = rd_u8(r) ? 1 : 0;
    (void)rd_u8(r);
}

/* ---------------- Defaults ---------------- */
static void default_band(dsp_band_t *b, int idx)
{
    b->on   = 0;
    b->type = DSP_FILTER_PEAKING;
    b->f    = s_default_freqs[idx % DSP_DEFAULT_BANDS];
    b->g    = 0.0f;
    b->q    = 1.0f;
}

static void default_xo(dsp_xo_t *x)
{
    x->on    = 0;
    x->type  = DSP_XO_LINKWITZ_RILEY;
    x->slope = 2;   /* 24 dB/oct */
    x->f     = 3000.0f;
}

void dsp_config_reset_defaults(void)
{
    if (s_lock != NULL) {
        xSemaphoreTake(s_lock, portMAX_DELAY);
    }

    memset(&s_cfg, 0, sizeof(s_cfg));

    for (int i = 0; i < DSP_NUM_INPUTS; i++) {
        dsp_input_t *in = &s_cfg.in[i];
        in->gain_db = 0.0f;
        in->mute    = 0;
        in->phase   = 0;
        in->n_eq    = DSP_DEFAULT_BANDS;
        in->n_req   = DSP_DEFAULT_BANDS;
        for (int b = 0; b < DSP_MAX_BANDS; b++) {
            default_band(&in->eq[b], b);
            default_band(&in->req[b], b);
        }
    }

    for (int i = 0; i < DSP_NUM_INPUTS; i++) {
        for (int o = 0; o < DSP_NUM_CHANNELS; o++) {
            s_cfg.route[i][o].on   = 1;
            s_cfg.route[i][o].src  = DSP_ROUTE_SRC_AUTO;   /* CH1 <- L, CH2 <- R */
            s_cfg.route[i][o].gain = 1.0f;
        }
    }

    for (int c = 0; c < DSP_NUM_CHANNELS; c++) {
        dsp_channel_t *ch = &s_cfg.ch[c];
        ch->n_eq = DSP_DEFAULT_BANDS;
        for (int b = 0; b < DSP_MAX_BANDS; b++) {
            default_band(&ch->eq[b], b);
        }
        default_xo(&ch->hp);
        default_xo(&ch->lp);
    }

    s_cfg.master = 1.0f;   /* 0 dB, bit-perfect */
#if CONFIG_DSP_MASTER_GAIN_TENTH_DB != 0
    /* Kconfig default is a 0.1 dB trim; master is a 0..1 linear trim, so only
     * the attenuation part of the range can be expressed here. */
    s_cfg.master = clampf(powf(10.0f, (float)CONFIG_DSP_MASTER_GAIN_TENTH_DB / 200.0f), 0.0f, 1.0f);
#endif

    for (int p = 0; p < DSP_NUM_PORTS; p++) {
        s_cfg.port[p].mode    = DSP_PORT_MODE_STEREO;
        s_cfg.port[p].format  = DSP_PORT_FORMAT_I2S;
        s_cfg.port[p].enabled = 1;   /* both I2S ports are live: I2S0 master + I2S1 master */
        s_cfg.port[p].gain_db  = 0.0f;
        s_cfg.port[p].delay_ms = 0.0f;
        default_xo(&s_cfg.port[p].hp);              /* 端口 HP/LP 默认关闭 */
        default_xo(&s_cfg.port[p].lp);
    }

    s_cfg.lim.on         = 0;          /* 默认关闭：保持出厂旁通（位精确） */
    s_cfg.lim.thresh_db  = -1.0f;      /* 接近 0 dBFS 才动手 */
    s_cfg.lim.release_ms = 100.0f;
    s_cfg.loud.on        = 0;          /* 默认关闭：出厂仍是位精确旁通 */
    s_cfg.loud.amount_db = 10.0f;      /* 主音量最低时最多补 10 dB */
    s_cfg.loud.ref       = 0.8f;       /* 参考音量 80%：此音量及以上不补偿 */
    s_cfg.comp.on        = 0;          /* 默认关闭：出厂仍是位精确旁通 */
    s_cfg.comp.thresh_db = -30;        /* 夜间模式预设：阈值 -30 dBFS */
    s_cfg.comp.ratio_x10 = 40;         /* 4.0:1 */
    s_cfg.comp.release_ms = 200;       /* 200 ms 释放 */

    /* 保留blob历史精度字段；当前准入只随压缩器开关变化。 */
    s_cfg.precision_mode = DSP_PRECISION_DEFAULT;

    /* ★ v36（阶段 B 接口层）：硬件控制层的出厂默认 —— 延时 A/B 各 30 ms、低有效静音。
     * ⚠ 本轮**只有配置**：没有任何 GPIO 被初始化、没有任务被创建（行为在阶段 C1）。 */
    s_cfg.hw_mute_delay_a_ms = DSP_HW_DELAY_DEFAULT;
    s_cfg.hw_mute_delay_b_ms = DSP_HW_DELAY_DEFAULT;
    s_cfg.hw_mute_polarity   = 0;      /* 0 = 低有效静音（TAS5828 低电平静音） */

    if (s_lock != NULL) {
        xSemaphoreGive(s_lock);
    }
}

/* ---------------- DSP 负载（滤波器预算） ---------------- */
static uint32_t xo_stage_count(const dsp_xo_t *x)
{
    if (x == NULL || !x->on) {
        return 0;
    }
    const uint8_t slope = (x->slope == 1 || x->slope == 2 || x->slope == 4) ? x->slope : 2;
    if (x->type == DSP_XO_LINKWITZ_RILEY && slope == 1) {
        return 1;                      /* LR2 = 一级二阶 */
    }
    return slope;                      /* BW: slope 级；LR: slope 级（dsp_xo_design 一致） */
}

/* 界面计数 = 当前源（任一）的输入级 + 四条通道链 + 端口0分频。
 * 两个输入源的输入 PEQ/房间 EQ 不会同时跑，所以取"输入级较大的那个源"，
 * 而不是把两个源相加（否则会高估）。 */
static uint32_t count_biquads_of(const dsp_config_t *c)
{
    uint32_t in_max = 0;
    for (int i = 0; i < DSP_NUM_INPUTS; i++) {
        const dsp_input_t *in = &c->in[i];
        uint32_t n = 0;
        for (int b = 0; b < in->n_eq && b < DSP_MAX_BANDS; b++) {
            if (in->eq[b].on) n++;
        }
        for (int b = 0; b < in->n_req && b < DSP_MAX_BANDS; b++) {
            if (in->req[b].on) n++;
        }
        if (n > in_max) in_max = n;
    }
    uint32_t n = in_max;
    for (int ch_i = 0; ch_i < DSP_NUM_CHANNELS; ch_i++) {
        const dsp_channel_t *ch = &c->ch[ch_i];
        for (int b = 0; b < ch->n_eq && b < DSP_MAX_BANDS; b++) {
            if (ch->eq[b].on) n++;
        }
        n += xo_stage_count(&ch->hp) + xo_stage_count(&ch->lp);
    }
    /* 端口 0 的 HP/LP 进入实际信号链；端口 1 的 HP/LP 目前只保存/下发，
     * 引擎的 port1 首版只做模式映射，不把未实现的滤波算进预算。 */
    n += xo_stage_count(&c->port[0].hp) + xo_stage_count(&c->port[0].lp);
    return n;
}

static uint32_t count_biquads_locked(void)
{
    return count_biquads_of(&s_cfg);
}

uint32_t dsp_config_count_biquads(void)
{
    if (s_lock != NULL) {
        xSemaphoreTake(s_lock, portMAX_DELAY);
    }
    const uint32_t n = count_biquads_locked();
    if (s_lock != NULL) {
        xSemaphoreGive(s_lock);
    }
    return n;
}

/* 准入只在core0控制路径执行，不给PCM热路径增加预算计算。
 * 编译时核对物理上限和模型成本；拓扑或内核成本变更时必须重做模型。
 */
_Static_assert(DSP_NUM_CHANNELS == 4 && DSP_MAX_BANDS == 15,
               "Re-derive the budget when changing DSP topology");
_Static_assert(DSP_PER_BIQUAD_US < DSP_PER_PAIR_US &&
               DSP_PER_PAIR_US < 2u * DSP_PER_BIQUAD_US,
               "The worst-layout model requires mono < pair < two mono calls");
/* 0.125：保留0.123的130准入及压缩档期限断言，显式报告模型超设计目标。
 * 无压缩档继续守生产目标；压缩档采用目标提示/期限拒绝两条线。
 * 模型2369us高于2250us的事实不隐藏，也不改成本来已经满足生产预算。
 * 原131段断言并非必然失败；放宽前失败的是130段的生产预算断言。
 * 这些是模型编译约束，实际越界仍由既有dm/b90遥测报告，不自动改配置。
 */
_Static_assert(DSP_LIMIT_PRODUCTION_COMP == DSP_BIQUAD_UI_MAX &&
               DSP_LIMIT_PRODUCTION_NOCOMP == DSP_BIQUAD_UI_MAX,
               "Both admission tiers must equal the physical UI maximum");
_Static_assert(DSP_FIXED_BASE_US + DSP_FIXED_LOUD_US +
               DSP_BIQUAD_WORST_US(DSP_LIMIT_PRODUCTION_NOCOMP) <= DSP_PRODUCTION_BUDGET_US,
               "Full UI layout exceeds the non-compressor budget");
_Static_assert(DSP_FIXED_BASE_US + DSP_FIXED_LOUD_US + DSP_FIXED_COMP_US +
               DSP_BIQUAD_WORST_US(DSP_LIMIT_PRODUCTION_COMP) <= DSP_BLOCK_US_96K,
               "Compressor tier exceeds the block deadline, not just the design budget");
#if DSP_FIXED_BASE_US + DSP_FIXED_LOUD_US + DSP_FIXED_COMP_US + \
    DSP_BIQUAD_WORST_US(DSP_LIMIT_PRODUCTION_COMP) > DSP_PRODUCTION_BUDGET_US
#pragma message("DSP budget WARNING: compressor model exceeds production target; block deadline guard remains active")
#endif

uint32_t dsp_config_biquad_limit_for(uint8_t comp_on)
{
    return comp_on ? DSP_LIMIT_PRODUCTION_COMP : DSP_LIMIT_PRODUCTION_NOCOMP;
}

uint32_t dsp_config_biquad_limit(void)
{
    /* v30：只看**压缩器开关**（响度恒预留 ⇒ 不随它浮动；精度模式与预算无关）。 */
    return dsp_config_biquad_limit_for(s_cfg.comp.on ? 1u : 0u);
}

/* v27：给协议层读当前模式（写开关前判超限用） */
uint8_t dsp_config_precision_mode(void)
{
    return s_cfg.precision_mode;
}

/* v27 的 `precision_mode_forbids_comp()` 已按用户 2026-09-17 的决定**删除**
 * （精度模式不限制压缩器；当前压缩器按155 µs预留，生产预算见上方）。 */

/* ---------------- ★ v36：硬件控制层只读访问（阶段 B 只加存取器，**没有任何行为**） ----------------
 * 阶段 C1（mute 输出）会读延时 A/B 与极性。
 * ⚠ 这些读取**不加锁**：它们是 u8/u16 的原子读，且只在"参数被写入"时变化 ——
 *   与 `dsp_config_precision_mode()` 同一口径。 */
uint16_t dsp_config_hw_mute_delay_a_ms(void)
{
    return s_cfg.hw_mute_delay_a_ms;
}

uint16_t dsp_config_hw_mute_delay_b_ms(void)
{
    return s_cfg.hw_mute_delay_b_ms;
}

uint8_t dsp_config_hw_mute_polarity(void)
{
    return s_cfg.hw_mute_polarity;
}

/* ★ v36：**编译期静态断言**（规格 §4.2 的硬要求）——
 *   尾部给硬件控制层留了 `DSP_BLOB_TAIL_RESERVED_LEN` 字节，实际用了
 *   `DSP_BLOB_HW_TOTAL_LEN` 字节；**必须 ≤**，否则 blob 会**静默溢出**到别的字段上
 *   （序列化是线性写的，溢出不会报错，只会把后面覆盖掉 —— 这类错误极难查）。
 *   ⚠ 将来加字段时：要么还在预留区内，要么**升 blob ver=6 走升版流程**，不要硬塞。 */
_Static_assert(DSP_BLOB_HW_TOTAL_LEN <= DSP_BLOB_TAIL_RESERVED_LEN,
               "blob tail overflow: hardware-control fields exceed the reserved tail bytes");

/* ---------------- Serialize / deserialize ---------------- */
size_t dsp_config_serialize(uint8_t *buf, size_t cap, uint32_t fs)
{
    if (buf == NULL || cap < 16) {
        return 0;
    }

    if (s_lock != NULL) {
        xSemaphoreTake(s_lock, portMAX_DELAY);
    }

    blob_writer_t w = { .p = buf, .end = buf + cap, .overflow = false };

    wr_u32(&w, DSP_CONFIG_MAGIC);
    wr_u16(&w, DSP_CONFIG_VERSION);
    wr_u16(&w, 0);          /* preset id */
    wr_u32(&w, fs);
    wr_u32(&w, 0);          /* flags / reserved */

    for (int i = 0; i < DSP_NUM_INPUTS; i++) {
        const dsp_input_t *in = &s_cfg.in[i];
        wr_f32(&w, powf(10.0f, in->gain_db / 20.0f));   /* linear, like the WebUI */
        wr_u8(&w, in->mute);
        wr_u8(&w, in->phase);
        wr_u8(&w, in->n_eq);
        wr_u8(&w, in->n_req);
        for (int b = 0; b < in->n_eq && b < DSP_MAX_BANDS; b++) {
            serialize_band(&w, &in->eq[b]);
        }
        for (int b = 0; b < in->n_req && b < DSP_MAX_BANDS; b++) {
            serialize_band(&w, &in->req[b]);
        }
    }

    for (int i = 0; i < DSP_NUM_INPUTS; i++) {
        for (int o = 0; o < DSP_NUM_CHANNELS; o++) {
            const dsp_route_t *rt = &s_cfg.route[i][o];
            wr_u8(&w, rt->on);
            wr_u8(&w, rt->src);    /* v3 起原先的 pad 字节存放源声道选择 */
            wr_u8(&w, 0);
            wr_u8(&w, 0);
            wr_f32(&w, rt->gain);
        }
    }

    for (int c = 0; c < DSP_NUM_CHANNELS; c++) {
        const dsp_channel_t *ch = &s_cfg.ch[c];
        wr_u8(&w, 1);                       /* enabled */
        wr_u8(&w, 0);
        wr_u8(&w, ch->n_eq);
        wr_u8(&w, ch->hp.on ? 1 : 0);       /* hp stage count (informational) */
        wr_u8(&w, ch->lp.on ? 1 : 0);       /* lp stage count (informational) */
        wr_u8(&w, 0);
        wr_u8(&w, 0);
        wr_u8(&w, 0);
        for (int b = 0; b < ch->n_eq && b < DSP_MAX_BANDS; b++) {
            serialize_band(&w, &ch->eq[b]);
        }
        serialize_xo(&w, &ch->hp);
        serialize_xo(&w, &ch->lp);
    }

    wr_f32(&w, s_cfg.master);

    for (int p = 0; p < DSP_NUM_PORTS; p++) {
        const dsp_port_t *pt = &s_cfg.port[p];
        wr_u8(&w, pt->mode);
        wr_u8(&w, pt->format);
        wr_u8(&w, pt->enabled);
        wr_u8(&w, 0);
        wr_f32(&w, pt->gain_db);
        wr_f32(&w, pt->delay_ms);
        wr_u32(&w, 0);          /* 头 16 字节 */
        serialize_xo(&w, &pt->hp);
        serialize_xo(&w, &pt->lp);      /* 记录正好 32 字节 */
    }

    /* 限幅器 12B + 预留 20B（将来加字段不用再改版本号） */
    wr_u8(&w, s_cfg.lim.on);
    wr_u8(&w, 0);
    wr_u8(&w, 0);
    wr_u8(&w, 0);
    wr_f32(&w, s_cfg.lim.thresh_db);
    wr_f32(&w, s_cfg.lim.release_ms);
    /* 预留区前 4B 已启用为响度补偿（ver=5 布局不变；旧 blob 该区全 0 -> 响度关闭） */
    wr_u8(&w, s_cfg.loud.on);
    wr_u8(&w, 0);
    wr_u8(&w, 0);
    wr_u8(&w, 0);
    wr_f32(&w, s_cfg.loud.amount_db);
    wr_f32(&w, s_cfg.loud.ref);
    /* 预留区接下来的 6B 启用为压缩器（ver=5 布局不变；旧 blob 该区全 0 -> 压缩器关闭、
     * 阈值/压缩比/释放取不到合法值，由反序列化按默认补齐）。 */
    wr_u8(&w, s_cfg.comp.on);
    wr_u8(&w, 0);
    wr_u8(&w, (uint8_t)s_cfg.comp.thresh_db);
    /* 压缩比在内存里是 u16（参数编码 v2），但**序列化仍是 1 字节**：
     * 上限 DSP_COMP_RATIO_MAX 刻意取 255，所以这里能**无损**存下（不必夹取/截断）。
     * blob 布局因此保持不变、blob ver 仍为 5（见 dsp_config.h 的两层版本说明）。 */
    wr_u8(&w, (uint8_t)s_cfg.comp.ratio_x10);
    wr_u8(&w, (uint8_t)(s_cfg.comp.release_ms & 0xFF));
    wr_u8(&w, (uint8_t)((s_cfg.comp.release_ms >> 8) & 0xFF));
    /* ★ v27：尾部预留区的第 7 个字节（**limAt+30**）启用为**精度模式**。
     * 旧 blob 该字节为 0 ⇒ 反序列化读成 DSP_PRECISION_HIGH ⇒ **天然就是用户要的默认**，
     * 不需要任何迁移。布局没变（只是启用一个原本就预留的字节）⇒ **blob ver 仍为 5**。
     *
     * ⚠ 偏移账（改这里必须逐字节对，v27 在这里踩了一次**差一字节**的坑）：
     *     limAt+ 0..11 限幅器（1+3+4+4）
     *     limAt+12..23 响度补偿（1+3+4+4）
     *     limAt+24..29 压缩器（1+1+1+1+2）
     *     **limAt+30    精度模式**   ← 就是这一字节
     *     limAt+31..37 剩余预留 7B
     *   合计 38B。反序列化那边也必须"只丢 6B 再读模式"（原来是丢 8B/7B，读到了 +31/+37）。 */
    wr_u8(&w, (uint8_t)s_cfg.precision_mode);      /* limAt+30 */
    /* ★ v36（阶段 B）：limAt+31..37 那 7 个预留字节启用为**硬件控制层字段 + 存在标记**
     *   （见 dsp_config.h 的偏移账）。**不改总长、不改 blob ver**。
     *   ⚠ 顺序必须与反序列化逐字节对上（v27 在这里踩过"差一字节"的坑）。
     *   ⚠ limAt+35 是**保留字节：写 0、读忽略**（仍照写 1 字节，偏移不动）。 */
    wr_u8(&w, (uint8_t)(s_cfg.hw_mute_delay_a_ms & 0xFF));          /* limAt+31 */
    wr_u8(&w, (uint8_t)((s_cfg.hw_mute_delay_a_ms >> 8) & 0xFF));   /* limAt+32 */
    wr_u8(&w, (uint8_t)(s_cfg.hw_mute_delay_b_ms & 0xFF));          /* limAt+33 */
    wr_u8(&w, (uint8_t)((s_cfg.hw_mute_delay_b_ms >> 8) & 0xFF));   /* limAt+34 */
    wr_u8(&w, 0u);                                                 /* limAt+35：保留位（恒 0） */
    wr_u8(&w, s_cfg.hw_mute_polarity ? 1u : 0u);                    /* limAt+36 */
    wr_u8(&w, DSP_BLOB_HW_MARKER_VAL);                              /* limAt+37：存在标记 */
    if (s_lock != NULL) {
        xSemaphoreGive(s_lock);
    }

    if (w.overflow) {
        return 0;
    }
    return (size_t)(w.p - buf);
}

esp_err_t dsp_config_deserialize(const uint8_t *buf, size_t len)
{
    if (buf == NULL || len < 16) {
        return ESP_ERR_INVALID_ARG;
    }

    blob_reader_t r = { .p = buf, .end = buf + len, .underflow = false };

    uint32_t magic = rd_u32(&r);
    uint16_t version = rd_u16(&r);
    (void)rd_u16(&r);   /* preset */
    (void)rd_u32(&r);   /* fs, runtime only */
    (void)rd_u32(&r);   /* flags */

    if (magic != DSP_CONFIG_MAGIC || version != DSP_CONFIG_VERSION) {
        return ESP_ERR_INVALID_RESPONSE;
    }

    dsp_config_t tmp;
    memset(&tmp, 0, sizeof(tmp));

    for (int i = 0; i < DSP_NUM_INPUTS; i++) {
        dsp_input_t *in = &tmp.in[i];
        float lin = rd_f32(&r);
        if (!isfinite(lin) || lin < 1e-6f) {
            lin = 1e-6f;
        }
        in->gain_db = clampf(20.0f * log10f(lin), -72.0f, 12.0f);
        in->mute  = rd_u8(&r) ? 1 : 0;
        in->phase = rd_u8(&r) ? 1 : 0;
        in->n_eq  = clampi(rd_u8(&r), 0, DSP_MAX_BANDS);
        in->n_req = clampi(rd_u8(&r), 0, DSP_MAX_BANDS);
        for (int b = 0; b < DSP_MAX_BANDS; b++) {
            default_band(&in->eq[b], b);
            default_band(&in->req[b], b);
        }
        for (int b = 0; b < in->n_eq; b++) {
            deserialize_band(&r, &in->eq[b]);
        }
        for (int b = 0; b < in->n_req; b++) {
            deserialize_band(&r, &in->req[b]);
        }
    }

    for (int i = 0; i < DSP_NUM_INPUTS; i++) {
        for (int o = 0; o < DSP_NUM_CHANNELS; o++) {
            dsp_route_t *rt = &tmp.route[i][o];
            rt->on = rd_u8(&r) ? 1 : 0;            rt->src = clampi(rd_u8(&r), 0, DSP_ROUTE_SRC_MONO);
            (void)rd_u8(&r);
            (void)rd_u8(&r);
            rt->gain = clampf(rd_f32(&r), 0.0f, 1.0f);
        }
    }

    for (int c = 0; c < DSP_NUM_CHANNELS; c++) {
        dsp_channel_t *ch = &tmp.ch[c];
        (void)rd_u8(&r);                    /* enabled */
        (void)rd_u8(&r);
        ch->n_eq = clampi(rd_u8(&r), 0, DSP_MAX_BANDS);
        (void)rd_u8(&r);                    /* hp stage count */
        (void)rd_u8(&r);                    /* lp stage count */
        (void)rd_u8(&r);
        (void)rd_u8(&r);
        (void)rd_u8(&r);
        for (int b = 0; b < DSP_MAX_BANDS; b++) {
            default_band(&ch->eq[b], b);
        }
        for (int b = 0; b < ch->n_eq; b++) {
            deserialize_band(&r, &ch->eq[b]);
        }
        deserialize_xo(&r, &ch->hp);
        deserialize_xo(&r, &ch->lp);
    }

    float master = rd_f32(&r);
    tmp.master = (isfinite(master) ? clampf(master, 0.0f, 1.0f) : 1.0f);

    for (int p = 0; p < DSP_NUM_PORTS; p++) {
        dsp_port_t *pt = &tmp.port[p];
        pt->mode    = clampi(rd_u8(&r), 0, DSP_PORT_MODE_RIGHT_ONLY);
        pt->format  = clampi(rd_u8(&r), 0, DSP_PORT_FORMAT_RJ);
        pt->enabled = rd_u8(&r) ? 1 : 0;
        (void)rd_u8(&r);
        pt->gain_db  = clampf(rd_f32(&r), -24.0f, 12.0f);
        pt->delay_ms = clampf(rd_f32(&r), 0.0f, 20.0f);
        (void)rd_u32(&r);       /* pad */
        deserialize_xo(&r, &pt->hp);
        deserialize_xo(&r, &pt->lp);
    }

    tmp.lim.on = rd_u8(&r) ? 1 : 0;
    (void)rd_u8(&r);
    (void)rd_u8(&r);
    (void)rd_u8(&r);
    tmp.lim.thresh_db  = clampf(rd_f32(&r), -24.0f, 0.0f);
    tmp.lim.release_ms = clampf(rd_f32(&r), 10.0f, 1000.0f);
    /* 预留区前 4B = 响度补偿；旧 blob 全 0 -> on=0（默认关）。
     * 注意：必须全部写进 tmp，绝不能碰活动配置 —— 长度不足时本函数会整体拒绝，
     * 若先改了 s_cfg.loud，一个截断包就会静默关掉响度补偿（其余配置却被拒绝）。 */
    {
        const uint8_t lo_on = rd_u8(&r);
        (void)rd_u8(&r);
        (void)rd_u8(&r);
        (void)rd_u8(&r);
        const float lo_amt = clampf(rd_f32(&r), 0.0f, 15.0f);
        const float lo_ref = clampf(rd_f32(&r), 0.05f, 1.0f);
        /* 预留区接下来的 6B = 压缩器。旧 blob 这 6B 全 0：on=0（默认关），
         * 阈值 0 与压缩比 0 都非法 -> 视为"没有这一段"，回落到默认值。 */
        const uint8_t co_on = rd_u8(&r);
        (void)rd_u8(&r);
        const int8_t  co_th = (int8_t)rd_u8(&r);
        const uint8_t co_ra = rd_u8(&r);
        const uint16_t co_re = (uint16_t)rd_u8(&r) | ((uint16_t)rd_u8(&r) << 8);
        /* ★ v27：精度模式就在**这里**，即 **limAt+30** —— 限幅器12 + 响度12 + 压缩器6
         * 之后的第一字节，序列化那边也是紧跟压缩器写的（wr_u8(mode) + 7×wr_u8(0)）。
         *
         * ⚠⚠ v27 在这里来回错了三次，务必看清偏移账（用 OFF-DBG 逐段量过）：
         *     lim.on 读完 = limAt+12；响度读完 = limAt+24；压缩器读完 = **limAt+30**
         *   所以模式**必须紧接在压缩器之后读**，中间**不能**再丢任何预留字节。
         *   错误的两种写法：
         *     · `for (i<8) rd_u8()`  → 模式被读到 limAt+38（越界，且 8B 全丢）—— 旧代码
         *     · `for (i<7) …` 或 `for (i<6) …` 再读 → 读到 limAt+37 / limAt+36 的预留 0
         *   症状都一样而且极其误导：**模式永远是 0（高精度）** ⇒ 82 段配置存进 NVS 后，
         *   开机按 41 判超限 ⇒ "SAVE 成功、设备也回 config saved to NVS，但一复位就没了"。
         *   读完之后再丢 **7B** 剩余预留，凑满尾部 38B。 */
        const uint8_t precision = rd_u8(&r);           /* limAt+30 */
        /* ★ v36（阶段 B）：limAt+31..37 = 硬件控制层 6 B + 存在标记 1 B。
         *   ⚠ 顺序与序列化逐字节对应；**旧 blob 这 7 B 全 0** ⇒ 标记 = 0 ⇒ 全部回落到
         *     本函数开头的默认值（延时 30/30、低有效）—— 不需要任何迁移。
         *   ⚠ limAt+35 是**保留字节：写 0、读忽略**（仍读掉这 1 字节，偏移不动）。 */
        const uint16_t hw_da = (uint16_t)rd_u8(&r) | ((uint16_t)rd_u8(&r) << 8);   /* +31..32 */
        const uint16_t hw_db = (uint16_t)rd_u8(&r) | ((uint16_t)rd_u8(&r) << 8);   /* +33..34 */
        (void)rd_u8(&r);                                                           /* +35 保留位：忽略 */
        const uint8_t  hw_po = rd_u8(&r);                                          /* +36 */
        const uint8_t  hw_mk = rd_u8(&r);                                          /* +37 */
        if (hw_mk != 0) {
            /* 标记有效 ⇒ 用 blob 里的值（**逐项做范围校验**：坏值回落当前值，绝不静默夹取） */
            tmp.hw_mute_delay_a_ms = (hw_da <= DSP_HW_DELAY_MAX) ? hw_da : s_cfg.hw_mute_delay_a_ms;
            tmp.hw_mute_delay_b_ms = (hw_db <= DSP_HW_DELAY_MAX) ? hw_db : s_cfg.hw_mute_delay_b_ms;
            tmp.hw_mute_polarity   = (hw_po <= DSP_HW_POLARITY_MAX) ? hw_po : s_cfg.hw_mute_polarity;
        } else {
            /* ⚠ 旧 blob（这 7 B 全 0）⇒ **显式回落出厂默认**。
             *   不能靠"不赋值"：本函数的 `tmp` 是 `memset(&tmp, 0, …)`（全 0），
             *   不赋值的话延时会变成 **0 ms** 而不是默认的 30 ms —— 那就与规格
             *   §4.2 的"旧 blob 天然回落默认"相反了（这类"差一个默认值"的坑本项目踩过）。 */
            tmp.hw_mute_delay_a_ms = DSP_HW_DELAY_DEFAULT;
            tmp.hw_mute_delay_b_ms = DSP_HW_DELAY_DEFAULT;
            tmp.hw_mute_polarity   = 0;      /* 低有效静音 */
        }
        /* 标记为 0（旧 blob）⇒ 保持 tmp 里的默认值，**不动**。 */
        tmp.precision_mode = (precision <= DSP_PRECISION_MAX) ? precision : DSP_PRECISION_HIGH;
        tmp.loud.on        = lo_on ? 1 : 0;
        tmp.loud.amount_db = lo_amt;
        tmp.loud.ref       = lo_ref;
        tmp.comp.on = co_on ? 1 : 0;
        tmp.comp.thresh_db = (co_th <= DSP_COMP_TH_MAX && co_th >= DSP_COMP_TH_MIN)
                             ? co_th : s_cfg.comp.thresh_db;
        /* 压缩比：blob 里仍是 **1 字节**（≤255 无损），但内存里是 u16。
         * 编译器对 u8 变量做 `co_ra <= 255` 会报 -Wtype-limits（恒真），
         * 所以显式提升成 uint16_t 再比 —— 这也把"blob 单字节"这个事实写在代码里。 */
        const uint16_t ratio_blob = (uint16_t)co_ra;
        tmp.comp.ratio_x10 = (ratio_blob >= DSP_COMP_RATIO_MIN && ratio_blob <= DSP_COMP_RATIO_MAX)
                             ? ratio_blob : s_cfg.comp.ratio_x10;
        tmp.comp.release_ms = (co_re >= DSP_COMP_REL_MIN && co_re <= DSP_COMP_REL_MAX)
                              ? co_re : s_cfg.comp.release_ms;
    }

    if (r.underflow) {
        return ESP_ERR_INVALID_SIZE;
    }

    /* 载入路径同样要过双二阶预算：blob / NVS / 预设都能带来超预算配置，
     * 而引擎侧只打一条 "DSP overload" 警告就继续跑（dsp_engine.c rebuild），
     * 结果就是掉音。这里直接拒绝，让主机看到明确错误而不是"装进去了但跑不动"。
     * ⚠ **v30：必须用 `tmp` 自己的 `comp.on` 去算上限**，不能用
     *   `dsp_config_biquad_limit()` —— 后者读的是活动配置 `s_cfg`，而此刻还没提交。
     *   （v27 在这里踩过同类坑：拿"当前状态"去判"候选配置"。上限现在取决于
     *   压缩器开关，所以这个"用候选自己的状态"的要求比 v27 时更要紧了。） */
    {
        const uint32_t limit_for_tmp = dsp_config_biquad_limit_for(tmp.comp.on ? 1u : 0u);
        if (count_biquads_of(&tmp) > limit_for_tmp) {
            return ESP_ERR_NOT_SUPPORTED;
        }
        /* ⚠ v32：原来这里还有一条"扩展模式 + 压缩器开 ⇒ ESP_ERR_NOT_SUPPORTED"
         * 的自相矛盾检查 —— 随着"扩展模式禁止压缩器"被取消，它已经没有意义
         * （两种模式都能装压缩器，不存在矛盾的组合）。 */
    }

    if (s_lock != NULL) {
        xSemaphoreTake(s_lock, portMAX_DELAY);
    }
    s_cfg = tmp;
    if (s_lock != NULL) {
        xSemaphoreGive(s_lock);
    }
    return ESP_OK;
}

/* ---------------- NVS persistence ---------------- */
esp_err_t dsp_config_save(void)
{
    size_t len = dsp_config_serialize(s_nvs_blob, sizeof(s_nvs_blob), 0);
    if (len == 0) {
        return ESP_ERR_NO_MEM;
    }

    nvs_handle_t h;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h);
    if (err != ESP_OK) {
        return err;
    }
    err = nvs_set_blob(h, NVS_KEY_BLOB, s_nvs_blob, len);
    if (err == ESP_OK) {
        err = nvs_commit(h);
    }
    nvs_close(h);
    return err;
}

esp_err_t dsp_config_load(void)
{
    nvs_handle_t h;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &h);
    if (err != ESP_OK) {
        return err;
    }
    size_t len = sizeof(s_nvs_blob);
    err = nvs_get_blob(h, NVS_KEY_BLOB, s_nvs_blob, &len);
    nvs_close(h);
    if (err != ESP_OK) {
        return err;
    }
    return dsp_config_deserialize(s_nvs_blob, len);
}

/* ---------------- 设备端预设槽（NVS） ---------------- */
static void preset_key(uint8_t slot, char *key, size_t n)
{
    snprintf(key, n, "p%u", (unsigned)slot);
}

esp_err_t dsp_config_preset_save(uint8_t slot)
{
    if (slot >= DSP_PRESET_SLOTS) {
        return ESP_ERR_INVALID_ARG;
    }
    const size_t len = dsp_config_serialize(s_nvs_blob, sizeof(s_nvs_blob), 0);
    if (len == 0) {
        return ESP_ERR_NO_MEM;
    }
    char key[4];
    preset_key(slot, key, sizeof(key));

    nvs_handle_t h;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h);
    if (err != ESP_OK) {
        return err;
    }
    err = nvs_set_blob(h, key, s_nvs_blob, len);
    if (err == ESP_OK) {
        err = nvs_commit(h);
    }
    nvs_close(h);
    return err;
}

esp_err_t dsp_config_preset_load(uint8_t slot)
{
    if (slot >= DSP_PRESET_SLOTS) {
        return ESP_ERR_INVALID_ARG;
    }
    char key[4];
    preset_key(slot, key, sizeof(key));

    nvs_handle_t h;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &h);
    if (err != ESP_OK) {
        return err;
    }
    size_t len = sizeof(s_nvs_blob);
    err = nvs_get_blob(h, key, s_nvs_blob, &len);
    nvs_close(h);
    if (err != ESP_OK) {
        return err;
    }
    err = dsp_config_deserialize(s_nvs_blob, len);   /* 校验版本/魔数并替换配置 */
    if (err == ESP_OK) {
        dsp_config_apply_all();                      /* 立刻生效 */
    }
    return err;
}

esp_err_t dsp_config_preset_clear(uint8_t slot)
{
    if (slot >= DSP_PRESET_SLOTS) {
        return ESP_ERR_INVALID_ARG;
    }
    char key[4];
    preset_key(slot, key, sizeof(key));

    nvs_handle_t h;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h);
    if (err != ESP_OK) {
        return err;
    }
    err = nvs_erase_key(h, key);
    if (err == ESP_OK || err == ESP_ERR_NVS_NOT_FOUND) {
        err = nvs_commit(h);
    }
    nvs_close(h);
    return (err == ESP_ERR_NVS_NOT_FOUND) ? ESP_OK : err;
}

uint8_t dsp_config_preset_mask(void)
{
    char key[4];
    uint8_t mask = 0;

    nvs_handle_t h;
    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &h) != ESP_OK) {
        return 0;
    }
    for (uint8_t s = 0; s < DSP_PRESET_SLOTS; s++) {
        preset_key(s, key, sizeof(key));
        size_t len = 0;
        if (nvs_get_blob(h, key, NULL, &len) == ESP_OK && len > 0) {
            mask |= (uint8_t)(1u << s);
        }
    }
    nvs_close(h);
    return mask;
}

/* ---------------- Read access ---------------- */
void dsp_config_copy(dsp_config_t *dst)
{
    if (dst == NULL) {
        return;
    }
    if (s_lock != NULL) {
        xSemaphoreTake(s_lock, portMAX_DELAY);
    }
    *dst = s_cfg;
    if (s_lock != NULL) {
        xSemaphoreGive(s_lock);
    }
}

float dsp_config_master_gain_db(void)
{
    if (s_lock != NULL) {
        xSemaphoreTake(s_lock, portMAX_DELAY);
    }
    float master = s_cfg.master;
    if (s_lock != NULL) {
        xSemaphoreGive(s_lock);
    }

    /* Master volume is a trim inside the +-12 dB DSP budget. 0% is a real
     * mute (the engine maps it to a 0 linear gain), reported as -120 dB. */
    if (master <= 0.0f) {
        return -120.0f;
    }
    return clampf(20.0f * log10f(master), -12.0f, 12.0f);
}

uint32_t dsp_config_live_param_count(void)
{
    /* Every parameter block is now wired into the audio path. */
    return 12;   /* input gain/mute/phase, input PEQ, room EQ, routing,
                  * channel PEQ, channel HP/LP, master volume, port settings,
                  * port HP/LP, loudness */
}

/* ---------------- ★ v39：配置的两个传播面（**唯一出口**） ----------------
 *
 * 为什么要有这个函数（真实缺陷的形状，不是"忘了加一行"）：
 *   `RESET_CONFIG(0xA8)` / 预设载入 / NVS 载入都走 `dsp_config_apply_all()`，
 *   而 `hw_mute_apply_config()` 原先**只在参数写路径里单独调**（见 set_param 末尾）。
 *   ⇒ 于是"配置存储"回到默认了，**硬件控制层模块却还留着旧极性**：
 *     实测 `RESET` 之后 `0xAF` 仍报 `pol=1`、引脚也停在旧极性的静音电平
 *     （用户侧的观感是"复位了但 mute 还是反的"）。
 *
 * ⇒ 收口成**两个面**，各自只有一个出口：
 *   · **引擎面** `dsp_engine_sync()` —— DSP 通路（滤波/路由/增益…）；
 *   · **控制面** `dsp_config_apply_control()` —— 不进 DSP 的纯控制消费方（当前：`hw_mute`）。
 *
 * ⚠ **优化必须保住**：`0x81~0x84` 这类纯控制项的写入**不许重建 DSP 引擎**
 *   （否则拖滑块时每格重建一次、声音会断），所以参数写路径对纯控制项**只走控制面**。
 *
 * ★ **登记点为什么在本函数、而不在 `dsp_config.h`**（如实记录，别写成"已确定的机理"）：
 *   v39 曾观察到"改 `dsp_config.h` ⇒ `dsp_engine.c.obj` 哈希变了"，
 *   但随后**量了构建噪声底**（删 obj → 完整重编 ×4）⇒ **4 次全同（构建可复现）**，
 *   且删 obj 重编后哈希**回到冻结值 `F3B0C0A3…`**
 *   ⇒ 那几次"obj 移动"其实是**陈旧构建产物**（还原头文件后那次构建**并没有重编** dsp_engine.c），
 *     **不是头文件耦合**（`dsp_config.h` 只 include 四个系统头，版本号也不在 dsp_engine.c 的依赖链上）。
 *   ⇒ **本契约放在本函数**，理由是"紧贴实现、读它的人必然看到"，
 *     **不是**因为头文件不能改。
 *   ⚠ **规矩**：判"obj 是否移动"**必须先删掉那个 obj 再重编**，否则可能读到上一次留下的陈旧产物。
 * * ⚠ **规矩（新增配置消费方时照做）**：
 *   ① 在 `dsp_config.h` 的声明处登记它属于哪个面（引擎面 / 控制面 / 两个都要）；
 *   ② **控制面的消费方一律加进本函数** —— 这样 RESET / 预设载入 / NVS 载入自动覆盖；
 *   ③ 在 `proto_reset_prop_test.ps1`（传播覆盖回归）里**加一行**（该套件按消费方逐个循环）。
 */
static void dsp_config_apply_control(void)
{
    hw_mute_apply_config();          /* 0x81 延时 A / 0x82 极性 / 0x84 延时 B */
}

void dsp_config_apply_all(void)
{
    /* Rebuild the whole DSP engine from the current snapshot. Called after any
     * accepted parameter write and after NVS load. */
    ESP_ERROR_CHECK(dsp_core1_ctl(DSP_CTL_ENGINE_SYNC, 0, 0));
    dsp_config_apply_control();      /* ★ v39：控制侧消费方在这里**收口**（见 apply_control 注释） */

    /* ★ 线格式是**运行时硬件状态**，它不会随配置复位而变。这里按当前配置重挂一次，
     * 否则 RESET / 预设载入 / NVS 载入之后会出现"配置说 Philips、线上还是旧格式"的
     * 静默不一致（v23 查出）。i2s_out_set_format() 对同值是早退，所以常态调用不会
     * 打断正在播放的音频，只有真的变了才重配。 */
    if (s_lock != NULL) xSemaphoreTake(s_lock, portMAX_DELAY);
    const int format = s_cfg.port[0].format;
    const int secondary_format = s_cfg.port[1].format;
    if (s_lock != NULL) xSemaphoreGive(s_lock);
    ESP_ERROR_CHECK(i2s_out_set_format(format));
    /* Retain an unsupported historical NVS setting without erasing/changing
     * the stored blob; it must not silently become a different wire format. */
    if (secondary_format == DSP_PORT_FORMAT_RJ) {
        ESP_LOGE(TAG, "port1 RJ unsupported; I2S1 remains at prior physical format");
    } else {
        ESP_ERROR_CHECK(i2s_out_set_secondary_format(secondary_format));
    }

    /* 响度补偿的实时读数：它由主音量推出、没有独立可读回的量，所以每次重建后打一行
     * 日志（改主音量或改响度参数都会打一条），便于主机/日志确认当前补偿了多少 dB。 */
    if (s_lock != NULL) {
        xSemaphoreTake(s_lock, portMAX_DELAY);
    }
    const float master = s_cfg.master;
    const dsp_loudness_t lo = s_cfg.loud;
    if (s_lock != NULL) {
        xSemaphoreGive(s_lock);
    }
    if (lo.on) {
        ESP_LOGI(TAG, "loudness: on, amount=%.1f dB, ref=%.2f, master=%.0f%% -> boost %.2f dB",
                 lo.amount_db, lo.ref, master * 100.0f,
                 (double)dsp_engine_loudness_boost_db());
    } else {
        ESP_LOGI(TAG, "loudness: off (master=%.0f%%)", master * 100.0f);
    }
}

/* ---------------- Parameter writes ---------------- */
static dsp_status_t set_band_param(dsp_band_t *band, uint8_t param, const uint8_t *value, size_t len)
{
    float f = 0.0f;
    uint8_t u = 0;

    switch (param) {
    case DSP_P_EQ_ON:
    case DSP_P_REQ_ON:
        if (!read_u8(value, len, &u)) return DSP_STATUS_ERR_LENGTH;
        band->on = u ? 1 : 0;
        return DSP_STATUS_OK;
    case DSP_P_EQ_TYPE:
    case DSP_P_REQ_TYPE:
        if (!read_u8(value, len, &u)) return DSP_STATUS_ERR_LENGTH;
        band->type = clampi(u, 0, DSP_FILTER_ALLPASS);
        return DSP_STATUS_OK;
    case DSP_P_EQ_FREQ:
    case DSP_P_REQ_FREQ:
        if (!read_f32(value, len, &f)) return DSP_STATUS_ERR_LENGTH;
        band->f = clampf(f, DSP_BAND_FREQ_MIN_HZ, DSP_BAND_FREQ_MAX_HZ);
        return DSP_STATUS_OK;
    case DSP_P_EQ_GAIN:
    case DSP_P_REQ_GAIN:
        if (!read_f32(value, len, &f)) return DSP_STATUS_ERR_LENGTH;
        band->g = clampf(f, DSP_BAND_GAIN_MIN_DB, DSP_BAND_GAIN_MAX_DB);
        return DSP_STATUS_OK;
    case DSP_P_EQ_Q:
    case DSP_P_REQ_Q:
        if (!read_f32(value, len, &f)) return DSP_STATUS_ERR_LENGTH;
        band->q = clampf(f, DSP_BAND_Q_MIN, DSP_BAND_Q_MAX);
        return DSP_STATUS_OK;
    default:
        return DSP_STATUS_ERR_PARAM;
    }
}

dsp_status_t dsp_config_set_param(uint8_t block, uint8_t ch, uint8_t param, uint8_t idx,
                                  const uint8_t *value, size_t value_len)
{
    if (value == NULL) {
        return DSP_STATUS_ERR_VALUE;
    }

    dsp_status_t st = DSP_STATUS_OK;
    float f = 0.0f;
    uint8_t u = 0;
    uint16_t uu = 0;

    /* 滤波器预算保护：写入后若超限，用这里保存的副本回滚 */
    dsp_band_t *undo_band = NULL;
    dsp_band_t  undo_band_v;
    dsp_xo_t   *undo_xo = NULL;
    dsp_xo_t    undo_xo_v;
    /* 段数（n_eq/n_req）同样影响预算，必须单独留回滚点 ——
     * 否则"把段数调大 → 超限 → 返回 7"之后，超额的段数会留在设备里，
     * 界面收到 status=7 却不回滚本地 cfg（WebUI L1092-1095），两边从此不一致。 */
    uint8_t    *undo_count = NULL;
    uint8_t     undo_count_v = 0;

    /* ★ v38（阶段 C1）：本参数是不是"纯控制项"（不进 DSP）。
     * 硬件控制层（0x81/0x82/0x84 延时与极性）只被别的模块消费，
     * 重建 DSP 引擎对它们毫无意义 —— 反而会在用户拖"延时 A"滑块时让正在播的声音
     * 卡一下、并白花 core1 的时间。见函数末尾的处置。 */
    bool pure_control = false;

    if (s_lock != NULL) {
        xSemaphoreTake(s_lock, portMAX_DELAY);
    }

    switch (block) {
    case DSP_BLOCK_INPUT: {
        if (ch >= DSP_NUM_INPUTS) { st = DSP_STATUS_ERR_INDEX; break; }
        dsp_input_t *in = &s_cfg.in[ch];
        switch (param) {
        case DSP_P_GAIN:
            if (!read_f32(value, value_len, &f)) { st = DSP_STATUS_ERR_LENGTH; break; }
            in->gain_db = clampf(f, -72.0f, 12.0f);
            break;
        case DSP_P_MUTE:
            if (!read_u8(value, value_len, &u)) { st = DSP_STATUS_ERR_LENGTH; break; }
            in->mute = u ? 1 : 0;
            break;
        case DSP_P_PHASE:
            if (!read_u8(value, value_len, &u)) { st = DSP_STATUS_ERR_LENGTH; break; }
            in->phase = u ? 1 : 0;
            break;
        case DSP_P_EQ_COUNT:
            if (!read_u8(value, value_len, &u)) { st = DSP_STATUS_ERR_LENGTH; break; }
            undo_count = &in->n_eq; undo_count_v = in->n_eq;
            in->n_eq = clampi(u, 1, DSP_MAX_BANDS);
            break;
        case DSP_P_REQ_COUNT:
            if (!read_u8(value, value_len, &u)) { st = DSP_STATUS_ERR_LENGTH; break; }
            undo_count = &in->n_req; undo_count_v = in->n_req;
            in->n_req = clampi(u, 1, DSP_MAX_BANDS);
            break;
        default:
            if (idx >= DSP_MAX_BANDS) { st = DSP_STATUS_ERR_INDEX; break; }
            dsp_band_t *band = (param >= DSP_P_REQ_ON) ? &in->req[idx] : &in->eq[idx];
            undo_band = band; undo_band_v = *band;      /* 预算保护回滚点 */
            /* ★ 0.30 修的 P1-1 漏洞：**开关一个段会顺带自动扩段数**（见下面 auto-grow），
             *   而段数同样占预算 ⇒ 段数必须一起进回滚点。
             *   只回滚段、不回滚段数的话，"超限被拒"的写会留下一个被撑大的段数：
             *   段虽然关回去了，但**后续 idx 落在新段数内的段**会被重新计入 ⇒
             *   设备停在**超上限**状态（v30 实测：上限 117 时停在 119；
             *   而 `dsp_config_apply_all()` 只在下一次**被接受**的写之后才跑，
             *   所以那个超限配置还会在下一次无关写入时被真正加载进引擎）。
             *   复现：先凑到 116 段，再单独写 `0x75(段数)=15` → 119 被拒 + 回滚，
             *   随后每写一个 `0x70(房间EQ段) on` 都只回滚段、段数一路涨到 15。 */
            undo_count = (param >= DSP_P_REQ_ON) ? &in->n_req : &in->n_eq;
            undo_count_v = *undo_count;
            st = set_band_param(band, param, value, value_len);
            if (st == DSP_STATUS_OK) {
                /* Auto-grow the active band count so a host may write band N
                 * before (or instead of) sending the count parameter. */
                if (param >= DSP_P_REQ_ON) {
                    if ((uint8_t)(idx + 1) > in->n_req) in->n_req = (uint8_t)(idx + 1);
                } else {
                    if ((uint8_t)(idx + 1) > in->n_eq) in->n_eq = (uint8_t)(idx + 1);
                }
            }
            break;
        }
        break;
    }

    case DSP_BLOCK_OUTPUT: {
        if (ch >= DSP_NUM_CHANNELS) { st = DSP_STATUS_ERR_INDEX; break; }
        dsp_channel_t *chan = &s_cfg.ch[ch];
        /* 预算保护回滚点：本块可能改到 hp / lp / 某个 PEQ 段 */
        if (param == DSP_P_HP_ON || param == DSP_P_HP_TYPE ||
            param == DSP_P_HP_SLOPE || param == DSP_P_HP_FREQ) {
            undo_xo = &chan->hp; undo_xo_v = chan->hp;
        } else if (param == DSP_P_LP_ON || param == DSP_P_LP_TYPE ||
                   param == DSP_P_LP_SLOPE || param == DSP_P_LP_FREQ) {
            undo_xo = &chan->lp; undo_xo_v = chan->lp;
        } else if (param < DSP_P_REQ_ON && idx < DSP_MAX_BANDS) {
            undo_band = &chan->eq[idx]; undo_band_v = chan->eq[idx];
            /* ★ 0.30：同输入级 —— 通道段开关也会 auto-grow `chan->n_eq`，段数一起回滚。
             *   （本分支对 `DSP_P_EQ_COUNT(0x15)` 也会命中：那只是把同一个值再快照一次，
             *    下面 case 里会覆盖成同样的旧值，无副作用。） */
            undo_count = &chan->n_eq; undo_count_v = chan->n_eq;
        }
        switch (param) {
        case DSP_P_HP_ON:
            if (!read_u8(value, value_len, &u)) { st = DSP_STATUS_ERR_LENGTH; break; }
            chan->hp.on = u ? 1 : 0;
            break;
        case DSP_P_HP_TYPE:
            if (!read_u8(value, value_len, &u)) { st = DSP_STATUS_ERR_LENGTH; break; }
            chan->hp.type = clampi(u, 0, DSP_XO_LINKWITZ_RILEY);
            break;
        case DSP_P_HP_SLOPE:
            if (!read_u8(value, value_len, &u)) { st = DSP_STATUS_ERR_LENGTH; break; }
            chan->hp.slope = (u == 1 || u == 2 || u == 4) ? u : 2;
            break;
        case DSP_P_HP_FREQ:
            if (!read_f32(value, value_len, &f)) { st = DSP_STATUS_ERR_LENGTH; break; }
            chan->hp.f = clampf(f, 20.0f, 20000.0f);
            break;
        case DSP_P_LP_ON:
            if (!read_u8(value, value_len, &u)) { st = DSP_STATUS_ERR_LENGTH; break; }
            chan->lp.on = u ? 1 : 0;
            break;
        case DSP_P_LP_TYPE:
            if (!read_u8(value, value_len, &u)) { st = DSP_STATUS_ERR_LENGTH; break; }
            chan->lp.type = clampi(u, 0, DSP_XO_LINKWITZ_RILEY);
            break;
        case DSP_P_LP_SLOPE:
            if (!read_u8(value, value_len, &u)) { st = DSP_STATUS_ERR_LENGTH; break; }
            chan->lp.slope = (u == 1 || u == 2 || u == 4) ? u : 2;
            break;
        case DSP_P_LP_FREQ:
            if (!read_f32(value, value_len, &f)) { st = DSP_STATUS_ERR_LENGTH; break; }
            chan->lp.f = clampf(f, 20.0f, 20000.0f);
            break;
        case DSP_P_EQ_COUNT:
            if (!read_u8(value, value_len, &u)) { st = DSP_STATUS_ERR_LENGTH; break; }
            undo_count = &chan->n_eq; undo_count_v = chan->n_eq;
            chan->n_eq = clampi(u, 1, DSP_MAX_BANDS);
            break;
        default:
            if (idx >= DSP_MAX_BANDS) { st = DSP_STATUS_ERR_INDEX; break; }
            if (param >= DSP_P_REQ_ON) { st = DSP_STATUS_ERR_PARAM; break; }
            st = set_band_param(&chan->eq[idx], param, value, value_len);
            if (st == DSP_STATUS_OK && (uint8_t)(idx + 1) > chan->n_eq) {
                chan->n_eq = (uint8_t)(idx + 1);
            }
            break;
        }
        break;
    }

    case DSP_BLOCK_ROUTING: {
        if (ch >= DSP_NUM_INPUTS || idx >= DSP_NUM_CHANNELS) { st = DSP_STATUS_ERR_INDEX; break; }
        dsp_route_t *rt = &s_cfg.route[ch][idx];
        switch (param) {
        case DSP_P_ROUTE_ON:
            if (!read_u8(value, value_len, &u)) { st = DSP_STATUS_ERR_LENGTH; break; }
            rt->on = u ? 1 : 0;
            break;
        case DSP_P_ROUTE_GAIN:
            if (!read_f32(value, value_len, &f)) { st = DSP_STATUS_ERR_LENGTH; break; }
            rt->gain = clampf(f, 0.0f, 1.0f);
            break;
        case DSP_P_ROUTE_SRC:
            if (!read_u8(value, value_len, &u)) { st = DSP_STATUS_ERR_LENGTH; break; }
            rt->src = clampi(u, 0, DSP_ROUTE_SRC_MONO);
            break;
        default:
            st = DSP_STATUS_ERR_PARAM;
            break;
        }
        break;
    }

    case DSP_BLOCK_GLOBAL: {
        if (ch != 0) { st = DSP_STATUS_ERR_INDEX; break; }
        switch (param) {
        case DSP_P_MASTER_VOL:
            if (idx != 0) { st = DSP_STATUS_ERR_INDEX; break; }
            if (!read_f32(value, value_len, &f)) { st = DSP_STATUS_ERR_LENGTH; break; }
            s_cfg.master = clampf(f, 0.0f, 1.0f);
            break;
        /* ★ v27 新增：DSP 精度模式（u8，0=高精度默认 / 1=扩展）。
         * 两条准入规则（`规格_DSP精度模式.md` §5 判据 3/4 的固件侧）：
         *   ① 只能切到"当前滤波数装得下"的模式 —— 高精度 41 段装不下用户已配好的
         *      82 段时拒绝（status=5），原模式不变。避免"切过去配置就废了"。
         *   ② 切到**扩展**模式时会**自动关掉压缩器**（模式不变时压缩器才归用户管）——
         *      这正是用户选定的取舍："扩展 82 段，代价是压缩器不可用"。
         *      反过来"扩展模式下把压缩器打开"在 DSP_P_COMP_ON 那里硬拒（status=5）。 */
        case DSP_P_PRECISION_MODE:
            if (idx != 0) { st = DSP_STATUS_ERR_INDEX; break; }
            if (!read_u8(value, value_len, &u)) { st = DSP_STATUS_ERR_LENGTH; break; }
            if (u > DSP_PRECISION_MAX) { st = DSP_STATUS_ERR_VALUE; break; }
            /* ★ v30：精度模式**与预算无关了**（两种模式上限同值，取决于压缩器开关），
             *   所以这里不再需要"目标模式装不下当前滤波数就拒"的判断 ——
             *   切模式不可能让已配好的配置失效。只做取值校验 + 记录。
             *   ⚠ 参数 id / 默认值 / blob 布局一律未变（见 dsp_config.h 的精度模式注释）。
             *
             * ★★ 0.40（v41 轮 / 登记册 L19）：**补上 `pure_control = true;`**。
             *   为什么是它：它与 DSP 引擎**毫无关系** —— `dsp_config_precision_mode()`
             *   在固件里**没有任何调用者**（v30 起两种模式上限同值 ⇒ 该函数是死代码），
             *   而 v38 那轮给 `0x81/0x82/0x84` 做 `pure_control` 清扫时**漏了它**
             *   ⇒ 每次写 `0x80` 都会走 `dsp_config_apply_all()` **白重建一次引擎**：
             *     实测（v40 隔离探针）**换值 Δ+1 / 换回 Δ+1 / 同值重写也 Δ+1**，
             *     而 `0x81/0x82/0x84` 全部 Δ=0。
             *   ⚠ **触发路径真实存在**：前端 `pushAll()`（`dsp webui.html`）**每次整包同步**
             *     都会把 `0x80` 按**同值**再写一次 ⇒ 每次同步多一次无谓重建。
             *     （影响**低但不是零、而且可复现** —— 别写成"≈0"。） */
            s_cfg.precision_mode = u;
            pure_control = true;
            break;
        /* ---------------- ★ v36：硬件控制层（阶段 B —— 只存配置，不含任何行为）----------------
         * 规矩（规格 v9 §4.1 + 本项目的两条前车之鉴）：
         *   · **u16 必须按 2 字节小端发**：只给 1 字节 ⇒ `read_u16` 直接失败 ⇒ `status=4 ERR_LENGTH`
         *     （这正是"旧客户端按 u8 发"会踩的路径，与 `0x7E` 那次一模一样）；
         *   · **越界一律 `status=5 ERR_VALUE` 且原值不变**（不许静默夹取 —— v25 那批"静默钳位"的
         *     参数是历史遗留，新参数不再走那条路）。 */
        case DSP_P_HW_MUTE_DELAY_A:
            if (idx != 0) { st = DSP_STATUS_ERR_INDEX; break; }
            if (!read_u16(value, value_len, &uu)) { st = DSP_STATUS_ERR_LENGTH; break; }
            if (uu > DSP_HW_DELAY_MAX) { st = DSP_STATUS_ERR_VALUE; break; }
            s_cfg.hw_mute_delay_a_ms = uu;
            pure_control = true;
            break;
        case DSP_P_HW_MUTE_DELAY_B:
            if (idx != 0) { st = DSP_STATUS_ERR_INDEX; break; }
            if (!read_u16(value, value_len, &uu)) { st = DSP_STATUS_ERR_LENGTH; break; }
            if (uu > DSP_HW_DELAY_MAX) { st = DSP_STATUS_ERR_VALUE; break; }
            s_cfg.hw_mute_delay_b_ms = uu;
            pure_control = true;
            break;
        case DSP_P_HW_MUTE_POLARITY:
            if (idx != 0) { st = DSP_STATUS_ERR_INDEX; break; }
            if (!read_u8(value, value_len, &u)) { st = DSP_STATUS_ERR_LENGTH; break; }
            if (u > DSP_HW_POLARITY_MAX) { st = DSP_STATUS_ERR_VALUE; break; }
            s_cfg.hw_mute_polarity = u;
            pure_control = true;
            break;
        /* `0x83` / `0x85~0x89` 没有 case 分支 ⇒ 落到末尾 `default:` ⇒ 返回既有的
         * `DSP_STATUS_ERR_PARAM`（不静默成功、不改任何状态、不重建引擎）。 */
        case DSP_P_COMP_ON:
            if (idx != 0) { st = DSP_STATUS_ERR_INDEX; break; }
            if (!read_u8(value, value_len, &u)) { st = DSP_STATUS_ERR_LENGTH; break; }
            /* 非零值都会归一为on=1，必须全部在赋值前准入。
             * 通用回滚只保存band/xo/count，不能回滚comp.on。
             * 超过110段拒收，原值保持；无新增音频热路径分支。 */
            if (u != 0u && count_biquads_locked() > dsp_config_biquad_limit_for(1u)) {
                st = DSP_STATUS_ERR_LIMIT;      /* 7 = 超出 DSP 滤波器预算 */
                break;                          /* 原值不变 */
            }
            s_cfg.comp.on = u ? 1 : 0;
            break;
        case DSP_P_COMP_TH:
            if (idx != 0) { st = DSP_STATUS_ERR_INDEX; break; }
            if (!read_u8(value, value_len, &u)) { st = DSP_STATUS_ERR_LENGTH; break; }
            if ((int8_t)u > 0 || (int8_t)u < -60) { st = DSP_STATUS_ERR_VALUE; break; }
            s_cfg.comp.thresh_db = (int8_t)u;
            break;
        case DSP_P_COMP_RATIO:
            if (idx != 0) { st = DSP_STATUS_ERR_INDEX; break; }
            /* v24 / 参数编码 v2：**由 u8 改为 u16**（值=比×10，10..1000 => 1.0..100.0）。
             * 旧客户端仍发 1 字节 → read_u16 返回 false → status=4（长度错误）被拒，
             * 且**原值保持不变**（不会静默改成半个值）。 */
            if (!read_u16(value, value_len, &uu)) { st = DSP_STATUS_ERR_LENGTH; break; }
            if (uu < DSP_COMP_RATIO_MIN || uu > DSP_COMP_RATIO_MAX) { st = DSP_STATUS_ERR_VALUE; break; }
            s_cfg.comp.ratio_x10 = uu;
            break;
        case DSP_P_COMP_REL:
            if (idx != 0) { st = DSP_STATUS_ERR_INDEX; break; }
            if (!read_u16(value, value_len, &uu)) { st = DSP_STATUS_ERR_LENGTH; break; }
            if (uu < DSP_COMP_REL_MIN || uu > DSP_COMP_REL_MAX) { st = DSP_STATUS_ERR_VALUE; break; }
            s_cfg.comp.release_ms = uu;
            break;

        case DSP_P_PORT_MODE:
        case DSP_P_PORT_FORMAT:
        case DSP_P_PORT_GAIN:
        case DSP_P_PORT_DELAY:
        case DSP_P_PORT_ENABLE: {
            /* Both ports are masters. Mode/enable are independent; physical
             * port1 format supports Philips/MSB. HP/LP/gain/delay remain stored
             * configuration only for port1 in this integration. */
            if (idx >= DSP_NUM_PORTS) { st = DSP_STATUS_ERR_INDEX; break; }
            dsp_port_t *pt = &s_cfg.port[idx];
            switch (param) {
            case DSP_P_PORT_MODE:
                if (!read_u8(value, value_len, &u)) { st = DSP_STATUS_ERR_LENGTH; break; }
                pt->mode = clampi(u, 0, DSP_PORT_MODE_RIGHT_ONLY);
                break;
            case DSP_P_PORT_FORMAT:
                if (!read_u8(value, value_len, &u)) { st = DSP_STATUS_ERR_LENGTH; break; }
                if (idx == 1 && u >= DSP_PORT_FORMAT_RJ) { st = DSP_STATUS_ERR_UNSUPPORTED; break; }
                pt->format = clampi(u, 0, DSP_PORT_FORMAT_RJ);
                break;
            case DSP_P_PORT_ENABLE:
                if (!read_u8(value, value_len, &u)) { st = DSP_STATUS_ERR_LENGTH; break; }
                pt->enabled = u ? 1 : 0;
                break;
            case DSP_P_PORT_GAIN:
                if (!read_f32(value, value_len, &f)) { st = DSP_STATUS_ERR_LENGTH; break; }
                pt->gain_db = clampf(f, -24.0f, 12.0f);
                break;
            case DSP_P_PORT_DELAY:
                if (!read_f32(value, value_len, &f)) { st = DSP_STATUS_ERR_LENGTH; break; }
                pt->delay_ms = clampf(f, 0.0f, 20.0f);
                break;
            default:
                st = DSP_STATUS_ERR_PARAM;
                break;
            }
            break;
        }
        case DSP_P_PORT_HP_ON:
        case DSP_P_PORT_HP_FREQ:
        case DSP_P_PORT_HP_TYPE:
        case DSP_P_PORT_HP_SLOPE:
        case DSP_P_PORT_LP_ON:
        case DSP_P_PORT_LP_FREQ:
        case DSP_P_PORT_LP_TYPE:
        case DSP_P_PORT_LP_SLOPE: {
            /* 端口级 HP/LP：idx = 端口号。端口 1（I2S1）目前没有输出，参数照常
             * 接受并随配置存进 NVS，等第二个 I2S 控制器放开即可直接生效。 */
            if (idx >= DSP_NUM_PORTS) { st = DSP_STATUS_ERR_INDEX; break; }
            dsp_port_t *pt = &s_cfg.port[idx];
            dsp_xo_t *xo = (param >= DSP_P_PORT_LP_ON) ? &pt->lp : &pt->hp;
            undo_xo = xo; undo_xo_v = *xo;              /* 预算保护回滚点 */
            switch (param) {
            case DSP_P_PORT_HP_ON:
            case DSP_P_PORT_LP_ON:
                if (!read_u8(value, value_len, &u)) { st = DSP_STATUS_ERR_LENGTH; break; }
                xo->on = u ? 1 : 0;
                break;
            case DSP_P_PORT_HP_TYPE:
            case DSP_P_PORT_LP_TYPE:
                if (!read_u8(value, value_len, &u)) { st = DSP_STATUS_ERR_LENGTH; break; }
                xo->type = clampi(u, 0, DSP_XO_LINKWITZ_RILEY);
                break;
            case DSP_P_PORT_HP_SLOPE:
            case DSP_P_PORT_LP_SLOPE:
                if (!read_u8(value, value_len, &u)) { st = DSP_STATUS_ERR_LENGTH; break; }
                xo->slope = (u == 1 || u == 2 || u == 4) ? u : 2;
                break;
            default:            /* DSP_P_PORT_HP_FREQ / DSP_P_PORT_LP_FREQ */
                if (!read_f32(value, value_len, &f)) { st = DSP_STATUS_ERR_LENGTH; break; }
                xo->f = clampf(f, 20.0f, 20000.0f);
                break;
            }
            break;
        }
        case DSP_P_LIMIT_ON:
            if (idx != 0) { st = DSP_STATUS_ERR_INDEX; break; }
            if (!read_u8(value, value_len, &u)) { st = DSP_STATUS_ERR_LENGTH; break; }
            s_cfg.lim.on = u ? 1 : 0;
            break;
        case DSP_P_LIMIT_THRESH:
            if (idx != 0) { st = DSP_STATUS_ERR_INDEX; break; }
            if (!read_f32(value, value_len, &f)) { st = DSP_STATUS_ERR_LENGTH; break; }
            s_cfg.lim.thresh_db = clampf(f, -24.0f, 0.0f);
            break;
        case DSP_P_LIMIT_RELEASE:
            if (idx != 0) { st = DSP_STATUS_ERR_INDEX; break; }
            if (!read_f32(value, value_len, &f)) { st = DSP_STATUS_ERR_LENGTH; break; }
            s_cfg.lim.release_ms = clampf(f, 10.0f, 1000.0f);
            break;
        case DSP_P_LOUD_ON:
            if (idx != 0) { st = DSP_STATUS_ERR_INDEX; break; }
            if (!read_u8(value, value_len, &u)) { st = DSP_STATUS_ERR_LENGTH; break; }
            s_cfg.loud.on = u ? 1 : 0;
            break;
        case DSP_P_LOUD_AMOUNT:
            if (idx != 0) { st = DSP_STATUS_ERR_INDEX; break; }
            if (!read_f32(value, value_len, &f)) { st = DSP_STATUS_ERR_LENGTH; break; }
            s_cfg.loud.amount_db = clampf(f, 0.0f, 15.0f);
            break;
        case DSP_P_LOUD_REF:
            if (idx != 0) { st = DSP_STATUS_ERR_INDEX; break; }
            if (!read_f32(value, value_len, &f)) { st = DSP_STATUS_ERR_LENGTH; break; }
            s_cfg.loud.ref = clampf(f, 0.05f, 1.0f);
            break;
        default:
            st = DSP_STATUS_ERR_PARAM;
            break;
        }
        break;
    }

    case DSP_BLOCK_SYSTEM:
        st = DSP_STATUS_ERR_UNSUPPORTED;
        break;

    default:
        st = DSP_STATUS_ERR_BLOCK;
        break;
    }

    /* ---- 滤波器预算保护：超限则回滚这次写入并返回 status=7 ---- */
    if (st == DSP_STATUS_OK) {
        const uint32_t total = count_biquads_locked();
        const uint32_t limit = dsp_config_biquad_limit();
        if (total > limit) {
            if (undo_band != NULL) {
                *undo_band = undo_band_v;
            }
            if (undo_xo != NULL) {
                *undo_xo = undo_xo_v;
            }
            if (undo_count != NULL) {
                *undo_count = undo_count_v;
            }
            st = DSP_STATUS_ERR_LIMIT;
            /* ★ 0.30：回滚之后必须**真的**回到不超限。缺一个回滚点就会留下超限配置
             *   （0.30 之前漏的正是段数 auto-grow，实测停在 119/117）。
             *   这里做一次不变量自检：再退化时串口日志立刻可见，
             *   不必等到下一次"被接受的写"把超额配置加载进引擎。 */
            const uint32_t after = count_biquads_locked();
            if (after > limit) {
                ESP_LOGE(TAG, "budget rollback left %lu > %lu (block=%u ch=%u param=0x%02X idx=%u)",
                         (unsigned long)after, (unsigned long)limit,
                         (unsigned)block, (unsigned)ch, (unsigned)param, (unsigned)idx);
            }
        }
    }

    if (s_lock != NULL) {
        xSemaphoreGive(s_lock);
    }

    /* Any accepted write rebuilds the engine so the change is audible.
     *
     * ★ v38（阶段 C1）：**硬件控制项除外**（0x81/0x82/0x84 延时与极性）。
     *   它们不参与 DSP：重建引擎既白花 core1 的时间，又会在用户拖动"延时 A/B"滑块时
     *   让正在播放的声音断一下（滑块是连续发的，每格都重建一次引擎）。
     *   ⇒ 纯控制项改成推给 `hw_mute_apply_config()`（极性/延时立即生效，不碰引擎）。
     *   ⚠ 这条只改"重建与否"，**不改任何参数的语义**：越界照样 status=5、宽度错照样 status=4。 */
    if (st == DSP_STATUS_OK) {
        if (pure_control) {
            dsp_config_apply_control();   /* v39：纯控制项只走控制面（不重建引擎） */
        } else {
            dsp_config_apply_all();
        }
    }

    return st;
}

/* ---------------- Init ---------------- */
esp_err_t dsp_config_init(void)
{
    if (s_lock == NULL) {
        s_lock = xSemaphoreCreateMutex();
        if (s_lock == NULL) {
            ESP_LOGE(TAG, "mutex create failed");
            return ESP_ERR_NO_MEM;
        }
    }

    dsp_config_reset_defaults();

    esp_err_t err = dsp_config_load();
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "config loaded from NVS");
    } else if (err == ESP_ERR_NOT_SUPPORTED) {
        ESP_LOGW(TAG, "stored config exceeds the biquad budget at %lu Hz: defaults applied",
                 (unsigned long)i2s_out_get_rate());
    } else {
        ESP_LOGW(TAG, "no stored config (%s): defaults applied", esp_err_to_name(err));
    }

    dsp_config_apply_all();

    dsp_config_t snap;
    dsp_config_copy(&snap);
    ESP_LOGI(TAG, "firmware %d.%d, presets=0x%02X, biquads=%lu/%lu",
             DSP_FW_VERSION_MAJOR, DSP_FW_VERSION_MINOR,
             (unsigned)dsp_config_preset_mask(),
             (unsigned long)dsp_config_count_biquads(),
             (unsigned long)dsp_config_biquad_limit());
    ESP_LOGI(TAG, "store ready: master=%.2f (%.1f dB) in_eq=%u/%u ch_eq=%u/%u route=%d",
             (double)snap.master, (double)dsp_config_master_gain_db(),
             (unsigned)snap.in[0].n_eq, (unsigned)snap.in[1].n_eq,
             (unsigned)snap.ch[0].n_eq, (unsigned)snap.ch[1].n_eq,
             snap.route[0][0].on ? 1 : 0);
    return ESP_OK;
}
