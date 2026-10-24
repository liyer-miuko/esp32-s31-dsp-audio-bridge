/*
 * DSP engine - ESP32-S31 audio bridge. See dsp_engine.h for the topology.
 */
#include <math.h>
#include <string.h>

#include "esp_heap_caps.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "sdkconfig.h"

#include "dsp_engine.h"
#include "dsp_biquad_s31.h"

static const char *TAG = "dspeng";

/* 20 ms at 96 kHz; the WebUI caps the delay slider at 20 ms. */
#define DSP_PORT_DELAY_MAX_FRAMES 1920

typedef struct {
    uint8_t n;
    dsp_biquad_coeff_t c[DSP_MAX_BANDS];
} dsp_bank_t;

typedef struct {
    /* ---- input stage: stereo per source (0 = UAC, 1 = BT) ---- */
    float   in_gain[2];
    bool    in_mute[2];
    bool    in_phase[2];
    dsp_bank_t in_eq[2];
    dsp_bank_t in_req[2];
    dsp_biquad_state_t in_eq_z[2][2][DSP_MAX_BANDS];   /* [src][lr][k] */
    dsp_biquad_state_t in_req_z[2][2][DSP_MAX_BANDS];

    /* ---- routing: route[source][channel] ----
     * 每格把「开关 + 源声道选择 + 增益」折叠成两个系数（L/R 各自的权重），
     * 处理循环里就是两次乘加、没有分支：
     *   ch[c] = route_l[s][c] * in_L + route_r[s][c] * in_R  */
    float route_l[2][DSP_NUM_CHANNELS];
    float route_r[2][DSP_NUM_CHANNELS];

    /* ---- channel chains (mono) ---- */
    /* 4 条通道 = 端口0左/右 + 端口1左/右 */
    dsp_bank_t ch_eq[DSP_NUM_CHANNELS];
    dsp_biquad_state_t ch_eq_z[DSP_NUM_CHANNELS][DSP_MAX_BANDS];
    uint8_t ch_hp_n[DSP_NUM_CHANNELS];
    uint8_t ch_lp_n[DSP_NUM_CHANNELS];
    dsp_biquad_coeff_t ch_hp_c[DSP_NUM_CHANNELS][DSP_XO_MAX_STAGES];
    dsp_biquad_coeff_t ch_lp_c[DSP_NUM_CHANNELS][DSP_XO_MAX_STAGES];
    dsp_biquad_state_t ch_hp_z[DSP_NUM_CHANNELS][DSP_XO_MAX_STAGES];
    dsp_biquad_state_t ch_lp_z[DSP_NUM_CHANNELS][DSP_XO_MAX_STAGES];

    /* ---- master ---- */
    float master;

    /* ---- 响度补偿（等响）：主音量之后、端口之前，四通道共用一对架式滤波器 ---- */
    bool  loud_on;
    float loud_boost_db;                             /* 当前补偿量 dB（由主音量推出） */
    dsp_biquad_coeff_t loud_hi_c, loud_lo_c;
    dsp_biquad_state_t loud_hi_z[2], loud_lo_z[2];    /* [lr] */

    /* ---- ports ---- */
    bool     port_enabled[2];
    uint8_t  port_mode[2];
    float    port_gain[2];
    /* 端口级 HP/LP：系数每个端口一套（两个声道槽共用），状态每槽独立，
     * 否则左右会串进同一条延迟线（声像糊、左右串音）。 */
    uint8_t  port_hp_n[2];
    uint8_t  port_lp_n[2];
    dsp_biquad_coeff_t port_hp_c[DSP_NUM_PORTS][DSP_XO_MAX_STAGES];
    dsp_biquad_coeff_t port_lp_c[DSP_NUM_PORTS][DSP_XO_MAX_STAGES];
    dsp_biquad_state_t port_hp_z[DSP_NUM_PORTS][2][DSP_XO_MAX_STAGES];
    dsp_biquad_state_t port_lp_z[DSP_NUM_PORTS][2][DSP_XO_MAX_STAGES];
    uint32_t port_delay_frames[2];
    float   *port_delay_buf[2];
    uint32_t port_delay_cap[2];
    uint32_t port_delay_pos[2];

    /* ---- 削波保护（限幅器）：端口增益之后、延时之前 ---- */
    bool  lim_on;
    float lim_thresh;        /* 线性阈值 */
    float lim_release_coef;  /* 每样本释放系数 */
    float lim_gain;          /* 当前增益（1.0 = 未动作） */

    /* ---- 压缩器 / 夜间模式（端口增益之后、削波保护之前）----
     * 与限幅器一样是立体声联动。曲线（硬拐点，dB 域）：
     *   x ≤ th : g = makeup
     *   x > th : g = makeup - (x - th)·(1 - 1/R)
     * 其中 makeup = |th|·(1 - 1/R)，所以 x=0 dBFS 时 out=0 dBFS，任何输入都不会削顶。
     * 增益在 dB 域做一阶平滑：增益下降用 attack(5 ms)，上升用 release(参数)。 */
    bool  comp_on;
    float comp_thresh;       /* dBFS */
    float comp_slope;        /* 1 - 1/R */
    float comp_makeup_db;    /* |th|·slope */
    float comp_att_coef;
    float comp_rel_coef;
    float comp_gain_db;      /* 当前增益（dB），跨块保持 */

    /* ---- runtime ---- */
    bool     overloaded;     /* 当前配置的双二阶数超过本采样率的预算 */
    uint32_t fs;
    bool     bypass;
    bool     ready;
    int      last_source;
    volatile uint32_t clips;
    volatile uint32_t resets;      /* diagnostic: state resets */
    volatile uint32_t rebuilds;    /* diagnostic: coefficient rebuilds */
    volatile uint32_t process_calls;
    volatile uint32_t in_jumps;    /* diagnostic: block-boundary input jumps */
    volatile uint32_t in_jumps_mid;/* diagnostic: within-block input jumps */
    float    last_in_l;
} dsp_engine_t;

static dsp_engine_t      s_engine;
static SemaphoreHandle_t s_lock;

/* ---------------- helpers ---------------- */
static inline int32_t float_to_i32(float x)
{
    const float v = x * 2147483648.0f;
    if (v >= 2147483647.0f) {
        return INT32_MAX;
    }
    if (v <= -2147483648.0f) {
        return INT32_MIN;
    }
    return (int32_t)lrintf(v);
}

#define DSP_LOUD_LO_HZ   100.0f    /* 低频架点：等响曲线的低频抬升区 */
#define DSP_LOUD_HI_HZ 10000.0f    /* 高频架点 */
#define DSP_LOUD_Q        0.707f   /* 架式滤波器 Q */

/* 主音量 -> dB（与 master_to_linear 同一钳位：-12 dB 以下按 -12 计） */
static float master_db(float master)
{
    if (master <= 0.0f) {
        return -120.0f;
    }
    float db = 20.0f * log10f(master);
    if (db < -12.0f) db = -12.0f;
    if (db > 12.0f) db = 12.0f;
    return db;
}

/* 当前应补偿的 dB 数：主音量从参考点降到最低（-12 dB）时，从 0 线性升到 amount_db。 */
static float loudness_amount(const dsp_loudness_t *L, float master)
{
    if (L == NULL || !L->on || L->amount_db <= 0.0f) {
        return 0.0f;
    }
    float ref = L->ref;
    if (ref < 0.05f) ref = 0.05f;
    if (ref > 1.0f)  ref = 1.0f;
    const float ref_db  = master_db(ref);
    const float cur_db  = master_db(master);
    const float span_db = ref_db + 12.0f;      /* 参考点 -> 最低音量的 dB 跨度 */
    if (span_db <= 0.01f) {
        return 0.0f;
    }
    float k = (ref_db - cur_db) / span_db;
    if (k <= 0.0f) return 0.0f;
    if (k > 1.0f)  k = 1.0f;
    return L->amount_db * k;
}

static float master_to_linear(float master)
{
    if (master <= 0.0f) {
        return 0.0f;      /* 0% = 真静音（旧行为是钳到 -12 dB，关不死） */
    }
    float db = 20.0f * log10f(master);
    if (db < -12.0f) db = -12.0f;
    if (db > 12.0f) db = 12.0f;
    return powf(10.0f, db / 20.0f);
}

static void reset_states(dsp_engine_t *e)
{
    e->resets++;
    memset(e->in_eq_z, 0, sizeof(e->in_eq_z));
    memset(e->in_req_z, 0, sizeof(e->in_req_z));
    memset(e->ch_eq_z, 0, sizeof(e->ch_eq_z));
    memset(e->ch_hp_z, 0, sizeof(e->ch_hp_z));
    memset(e->ch_lp_z, 0, sizeof(e->ch_lp_z));
    memset(e->port_hp_z, 0, sizeof(e->port_hp_z));
    memset(e->port_lp_z, 0, sizeof(e->port_lp_z));
    memset(e->loud_hi_z, 0, sizeof(e->loud_hi_z));
    memset(e->loud_lo_z, 0, sizeof(e->loud_lo_z));
    e->lim_gain = 1.0f;
    for (int p = 0; p < DSP_NUM_PORTS; p++) {
        if (e->port_delay_buf[p] != NULL && e->port_delay_cap[p] > 0) {
            memset(e->port_delay_buf[p], 0, (size_t)e->port_delay_cap[p] * 2 * sizeof(float));
        }
        e->port_delay_pos[p] = 0;
    }
}

static bool compute_bypass(const dsp_engine_t *e)
{
    if (e->master != 1.0f) {
        return false;
    }
    if (e->lim_on) {
        return false;      /* 削波保护开着就交给 DSP 处理 */
    }
    if (e->loud_on) {
        return false;      /* 响度补偿开着也交给 DSP */
    }
    if (e->comp_on) {
        return false;      /* 压缩器开着也交给 DSP */
    }
    if (!e->port_enabled[0] || e->port_mode[0] != DSP_PORT_MODE_STEREO ||
        e->port_gain[0] != 1.0f || e->port_delay_frames[0] != 0) {
        return false;
    }
    for (int s = 0; s < DSP_NUM_INPUTS; s++) {
        if (e->in_gain[s] != 1.0f || e->in_mute[s] || e->in_phase[s]) {
            return false;
        }
        if (e->in_eq[s].n != 0 || e->in_req[s].n != 0) {
            return false;
        }
        /* 只有「奇数通道 = L、偶数通道 = R、增益 1」才算中性（等价于单位矩阵） */
        for (int o = 0; o < DSP_NUM_CHANNELS; o++) {
            const float want_l = ((o & 1) == 0) ? 1.0f : 0.0f;
            const float want_r = ((o & 1) == 0) ? 0.0f : 1.0f;
            if (e->route_l[s][o] != want_l || e->route_r[s][o] != want_r) {
                return false;
            }
        }
    }
    for (int c = 0; c < DSP_NUM_CHANNELS; c++) {
        if (e->ch_eq[c].n != 0 || e->ch_hp_n[c] != 0 || e->ch_lp_n[c] != 0) {
            return false;
        }
    }
    /* 旁通判断只看**端口 0** 的 HP/LP：配置侧也只计端口 0（I2S1／端口 1 灰显不启用），
     * 端口 1 即使被配了也不进信号链，不能据此判"非旁通"。 */
    if (e->port_hp_n[0] != 0 || e->port_lp_n[0] != 0) {
        return false;          /* 端口 0 的 HP/LP 只要配了就不是中性 */
    }
    return true;
}

static void ensure_delay_buffer(dsp_engine_t *e, int port, uint32_t frames)
{
    if (frames == 0) {
        e->port_delay_frames[port] = 0;
        return;
    }
    if (frames > DSP_PORT_DELAY_MAX_FRAMES) {
        frames = DSP_PORT_DELAY_MAX_FRAMES;
    }

    const uint32_t need = frames + 1;   /* a ring of N holds N-1 frames of delay */
    if (e->port_delay_cap[port] < need) {
        float *nb = (float *)heap_caps_realloc(e->port_delay_buf[port],
                                               (size_t)need * 2 * sizeof(float),
                                               MALLOC_CAP_8BIT);
        if (nb == NULL) {
            ESP_LOGE(TAG, "delay buffer alloc failed (%u frames)", (unsigned)need);
            e->port_delay_frames[port] = 0;
            return;
        }
        memset(nb, 0, (size_t)need * 2 * sizeof(float));
        e->port_delay_buf[port] = nb;
        e->port_delay_cap[port] = need;
        e->port_delay_pos[port] = 0;
    }
    e->port_delay_frames[port] = frames;
}

static void rebuild(dsp_engine_t *e, const dsp_config_t *cfg, uint32_t fs)
{
    e->fs = fs;
    e->rebuilds++;

    for (int s = 0; s < DSP_NUM_INPUTS; s++) {
        const dsp_input_t *in = &cfg->in[s];
        e->in_gain[s]  = powf(10.0f, in->gain_db / 20.0f);
        e->in_mute[s]  = in->mute != 0;
        e->in_phase[s] = in->phase != 0;

        e->in_eq[s].n = 0;
        for (int b = 0; b < in->n_eq && b < DSP_MAX_BANDS; b++) {
            const dsp_band_t *band = &in->eq[b];
            if (!band->on) {
                continue;
            }
            dsp_biquad_design(&e->in_eq[s].c[e->in_eq[s].n], band->type, band->f, band->g, band->q, fs);
            e->in_eq[s].n++;
        }

        e->in_req[s].n = 0;
        for (int b = 0; b < in->n_req && b < DSP_MAX_BANDS; b++) {
            const dsp_band_t *band = &in->req[b];
            if (!band->on) {
                continue;
            }
            dsp_biquad_design(&e->in_req[s].c[e->in_req[s].n], band->type, band->f, band->g, band->q, fs);
            e->in_req[s].n++;
        }
    }

    for (int s = 0; s < DSP_NUM_INPUTS; s++) {
        for (int o = 0; o < DSP_NUM_CHANNELS; o++) {
            const dsp_route_t *rt = &cfg->route[s][o];
            const float g = rt->on ? rt->gain : 0.0f;
            float gl = 0.0f, gr = 0.0f;
            switch (rt->src) {
            case DSP_ROUTE_SRC_L:    gl = g; break;
            case DSP_ROUTE_SRC_R:    gr = g; break;
            case DSP_ROUTE_SRC_MONO: gl = 0.5f * g; gr = 0.5f * g; break;
            default:                 /* AUTO：CH1/CH3 <- 源 L，CH2/CH4 <- 源 R */
                if ((o & 1) == 0) { gl = g; } else { gr = g; }
                break;
            }
            e->route_l[s][o] = gl;
            e->route_r[s][o] = gr;
        }
    }

    for (int c = 0; c < DSP_NUM_CHANNELS; c++) {
        const dsp_channel_t *ch = &cfg->ch[c];

        e->ch_eq[c].n = 0;
        for (int b = 0; b < ch->n_eq && b < DSP_MAX_BANDS; b++) {
            const dsp_band_t *band = &ch->eq[b];
            if (!band->on) {
                continue;
            }
            dsp_biquad_design(&e->ch_eq[c].c[e->ch_eq[c].n], band->type, band->f, band->g, band->q, fs);
            e->ch_eq[c].n++;
        }

        e->ch_hp_n[c] = dsp_xo_design(e->ch_hp_c[c], &ch->hp, true, fs);
        e->ch_lp_n[c] = dsp_xo_design(e->ch_lp_c[c], &ch->lp, false, fs);
    }

    e->master = master_to_linear(cfg->master);

    /* ---- 响度补偿：主音量越低补得越多（参考音量及以上为 0 dB） ---- */
    e->loud_on = cfg->loud.on != 0;
    e->loud_boost_db = loudness_amount(&cfg->loud, cfg->master);
    dsp_biquad_design(&e->loud_hi_c, DSP_FILTER_HIGHSHELF, DSP_LOUD_HI_HZ, e->loud_boost_db, DSP_LOUD_Q, fs);
    dsp_biquad_design(&e->loud_lo_c, DSP_FILTER_LOWSHELF,  DSP_LOUD_LO_HZ, e->loud_boost_db, DSP_LOUD_Q, fs);

    for (int p = 0; p < DSP_NUM_PORTS; p++) {
        e->port_enabled[p] = cfg->port[p].enabled != 0;
        e->port_mode[p]    = cfg->port[p].mode;
        e->port_gain[p]    = powf(10.0f, cfg->port[p].gain_db / 20.0f);

        /* 端口分频系数：与 WebUI 的 xoStageList() 同源（dsp_xo_design）。 */
        e->port_hp_n[p] = dsp_xo_design(e->port_hp_c[p], &cfg->port[p].hp, true, fs);
        e->port_lp_n[p] = dsp_xo_design(e->port_lp_c[p], &cfg->port[p].lp, false, fs);

        uint32_t frames = (uint32_t)lrintf(cfg->port[p].delay_ms * (float)fs / 1000.0f);
        ensure_delay_buffer(e, p, frames);
    }

    /* 限幅器：阈值 dBFS -> 线性，释放时间 -> 每样本系数 */
    e->lim_on = cfg->lim.on != 0;
    e->lim_thresh = powf(10.0f, cfg->lim.thresh_db / 20.0f);
    if (e->lim_thresh > 1.0f) e->lim_thresh = 1.0f;
    if (e->lim_thresh < 1e-4f) e->lim_thresh = 1e-4f;
    float rel_ms = cfg->lim.release_ms;
    if (rel_ms < 10.0f) rel_ms = 10.0f;
    const float tau = rel_ms * 0.001f * (float)(fs ? fs : 48000);
    e->lim_release_coef = (tau > 1.0f) ? (1.0f - expf(-1.0f / tau)) : 1.0f;
    if (e->lim_gain > 1.0f || e->lim_gain <= 0.0f) {
        e->lim_gain = 1.0f;
    }

    /* 压缩器 / 夜间模式：阈值与压缩比 -> dB 域斜率与自动补偿增益；释放 -> 每样本系数 */
    e->comp_on = cfg->comp.on != 0;
    e->comp_thresh = (float)cfg->comp.thresh_db;
    {
        float ratio = (float)cfg->comp.ratio_x10 / 10.0f;
        if (ratio < 1.0f)  ratio = 1.0f;
        if (ratio > 20.0f) ratio = 20.0f;
        e->comp_slope = 1.0f - 1.0f / ratio;
        e->comp_makeup_db = -e->comp_thresh * e->comp_slope;
        const float fs_f = (float)(fs ? fs : 48000);
        /* 攻击固定 5 ms；释放取参数（10..2000 ms） */
        const float tau_a = 0.005f * fs_f;
        e->comp_att_coef = (tau_a > 1.0f) ? (1.0f - expf(-1.0f / tau_a)) : 1.0f;
        float rel = (float)cfg->comp.release_ms;
        if (rel < 10.0f) rel = 10.0f;
        const float tau_r = rel * 0.001f * fs_f;
        e->comp_rel_coef = (tau_r > 1.0f) ? (1.0f - expf(-1.0f / tau_r)) : 1.0f;
        /* 状态直接落在静态工作点上，避免开开关时从 0 dB 慢慢爬（听感上是"渐强"） */
        e->comp_gain_db = e->comp_makeup_db;
    }

    /* 本采样率下的实际处理量：超预算就告警（例如 48k 存好的配置切到 96k 播放）。
     * 注意：这里在引擎锁内，不能再去拿配置锁（会与 dsp_engine_sync 形成 ABBA 死锁），
     * 所以直接数引擎自己的系数。 */
    {
        uint32_t in_max = 0;
        for (int s = 0; s < DSP_NUM_INPUTS; s++) {
            const uint32_t n = (uint32_t)e->in_eq[s].n + (uint32_t)e->in_req[s].n;
            if (n > in_max) in_max = n;
        }
        uint32_t nb = in_max;
        for (int c = 0; c < DSP_NUM_CHANNELS; c++) {
            nb += e->ch_eq[c].n + e->ch_hp_n[c] + e->ch_lp_n[c];
        }
        /* 与 dsp_config_count_biquads() 同口径：只数**端口 0**（端口 1 保留但不启用） */
        nb += e->port_hp_n[0] + e->port_lp_n[0];
        const bool over = (nb > dsp_config_biquad_limit());
        if (over != e->overloaded) {
            if (over) {
                ESP_LOGW(TAG, "DSP overload: %lu biquads > %lu at %lu Hz — 可能掉音，请减少滤波段数",
                         (unsigned long)nb, (unsigned long)dsp_config_biquad_limit(),
                         (unsigned long)fs);
            }
            e->overloaded = over;
        }
    }

    e->bypass = compute_bypass(e);
    e->ready = true;
}

/* ---------------- public API ---------------- */
esp_err_t dsp_engine_init(void)
{
    if (s_lock == NULL) {
        s_lock = xSemaphoreCreateMutex();
        if (s_lock == NULL) {
            return ESP_ERR_NO_MEM;
        }
    }
    memset(&s_engine, 0, sizeof(s_engine));
    s_engine.last_source = -1;
    s_engine.fs = 48000;
    ESP_LOGI(TAG, "DSP engine ready (%u biquads max per chain)", (unsigned)DSP_MAX_BANDS);
    return ESP_OK;
}

void dsp_engine_sync(void)
{
    if (s_lock == NULL) {
        return;
    }

    dsp_config_t snap;
    dsp_config_copy(&snap);          /* config lock first, then engine lock */

    xSemaphoreTake(s_lock, portMAX_DELAY);
    rebuild(&s_engine, &snap, s_engine.fs ? s_engine.fs : 48000);
    xSemaphoreGive(s_lock);
}

void dsp_engine_reset_state(void)
{
    if (s_lock == NULL) {
        return;
    }
    xSemaphoreTake(s_lock, portMAX_DELAY);
    reset_states(&s_engine);
    xSemaphoreGive(s_lock);
}

uint32_t dsp_engine_active_filter_count(void)
{
    if (s_lock == NULL) {
        return 0;
    }
    xSemaphoreTake(s_lock, portMAX_DELAY);
    /* 与 dsp_config_count_biquads() 同口径：两个输入源不会同时跑，取较大的那个 */
    uint32_t n = 0;
    for (int s = 0; s < DSP_NUM_INPUTS; s++) {
        const uint32_t m = (uint32_t)s_engine.in_eq[s].n + (uint32_t)s_engine.in_req[s].n;
        if (m > n) n = m;
    }
    for (int c = 0; c < DSP_NUM_CHANNELS; c++) {
        n += s_engine.ch_eq[c].n + s_engine.ch_hp_n[c] + s_engine.ch_lp_n[c];
    }
    /* 端口 0 的 HP/LP 进入实际信号链；端口 1 的 HP/LP 暂未处理，不能计。 */
    n += s_engine.port_hp_n[0] + s_engine.port_lp_n[0];
    xSemaphoreGive(s_lock);
    return n;
}

bool dsp_engine_is_bypass(void)
{
    if (s_lock == NULL) {
        return true;
    }
    xSemaphoreTake(s_lock, portMAX_DELAY);
    const bool b = s_engine.bypass;
    xSemaphoreGive(s_lock);
    return b;
}

uint32_t dsp_engine_get_clips(void)
{
    return s_engine.clips;
}

float dsp_engine_get_limiter_gr_db(void)
{
    const float g = s_engine.lim_gain;
    if (!s_engine.lim_on || g >= 1.0f || g <= 0.0f) {
        return 0.0f;
    }
    return -20.0f * log10f(g);      /* 正值 = 压缩量 dB */
}

float dsp_engine_loudness_boost_db(void)
{
    if (!s_engine.loud_on) {
        return 0.0f;
    }
    return s_engine.loud_boost_db;
}

bool dsp_engine_loudness_enabled(void)
{
    return s_engine.loud_on;
}

bool dsp_engine_compressor_enabled(void)
{
    return s_engine.comp_on;
}

void dsp_engine_get_diag(uint32_t *resets, uint32_t *rebuilds, uint32_t *calls)
{
    if (resets != NULL)   *resets = s_engine.resets;
    if (rebuilds != NULL) *rebuilds = s_engine.rebuilds;
    if (calls != NULL)    *calls = s_engine.process_calls;
}

void dsp_engine_get_jumps(uint32_t *boundary, uint32_t *mid)
{
    if (boundary != NULL) *boundary = s_engine.in_jumps;
    if (mid != NULL)      *mid = s_engine.in_jumps_mid;
}

size_t dsp_engine_collect_filters(dsp_filter_dump_t *out, size_t max)
{
    if (out == NULL || max == 0 || s_lock == NULL) {
        return 0;
    }

    xSemaphoreTake(s_lock, portMAX_DELAY);
    size_t n = 0;

#define PUSH(kind_, a_, idx_, coeff_)                              \
    do {                                                           \
        if (n < max) {                                             \
            out[n].kind = (kind_);                                 \
            out[n].a = (a_);                                       \
            out[n].b = 0;                                          \
            out[n].idx = (idx_);                                   \
            out[n].c = *(coeff_);                                  \
            n++;                                                   \
        }                                                          \
    } while (0)

    for (int s = 0; s < DSP_NUM_INPUTS; s++) {
        for (uint8_t k = 0; k < s_engine.in_eq[s].n; k++) {
            PUSH(DSP_FILTER_KIND_INPUT_PEQ, (uint8_t)s, k, &s_engine.in_eq[s].c[k]);
        }
    }
    for (int s = 0; s < DSP_NUM_INPUTS; s++) {
        for (uint8_t k = 0; k < s_engine.in_req[s].n; k++) {
            PUSH(DSP_FILTER_KIND_INPUT_REQ, (uint8_t)s, k, &s_engine.in_req[s].c[k]);
        }
    }
    for (int c = 0; c < DSP_NUM_CHANNELS; c++) {
        for (uint8_t k = 0; k < s_engine.ch_eq[c].n; k++) {
            PUSH(DSP_FILTER_KIND_CHANNEL_PEQ, (uint8_t)c, k, &s_engine.ch_eq[c].c[k]);
        }
    }
    for (int c = 0; c < DSP_NUM_CHANNELS; c++) {
        for (uint8_t k = 0; k < s_engine.ch_hp_n[c]; k++) {
            PUSH(DSP_FILTER_KIND_CHANNEL_HP, (uint8_t)c, k, &s_engine.ch_hp_c[c][k]);
        }
    }
    for (int c = 0; c < DSP_NUM_CHANNELS; c++) {
        for (uint8_t k = 0; k < s_engine.ch_lp_n[c]; k++) {
            PUSH(DSP_FILTER_KIND_CHANNEL_LP, (uint8_t)c, k, &s_engine.ch_lp_c[c][k]);
        }
    }
    for (int p = 0; p < DSP_NUM_PORTS; p++) {
        for (uint8_t k = 0; k < s_engine.port_hp_n[p]; k++) {
            PUSH(DSP_FILTER_KIND_PORT_HP, (uint8_t)p, k, &s_engine.port_hp_c[p][k]);
        }
    }
    for (int p = 0; p < DSP_NUM_PORTS; p++) {
        for (uint8_t k = 0; k < s_engine.port_lp_n[p]; k++) {
            PUSH(DSP_FILTER_KIND_PORT_LP, (uint8_t)p, k, &s_engine.port_lp_c[p][k]);
        }
    }
    if (s_engine.loud_on) {
        PUSH(DSP_FILTER_KIND_LOUDNESS, 0, 0, &s_engine.loud_lo_c);
        PUSH(DSP_FILTER_KIND_LOUDNESS, 0, 1, &s_engine.loud_hi_c);
    }

#undef PUSH

    xSemaphoreGive(s_lock);
    return n;
}

/* ================= v0.29：双二阶改「块处理」(band-major)；0.120：换 S31 内核 =================
 * 旧实现是 **sample-major**：每个采样把整条链走一遍。反汇编证实这样每个滤波器的
 * z1/z2 每采样都要 load + store 一次，而且 store→load 正好压在关键路径上。
 * band-major 后：一个滤波器连续跑完 n 帧再换下一个 ⇒ 系数与 z1/z2 一直待在浮点寄存器里。
 *
 * 运算契约：每一路保持固定工具链0.119实际生成的六条浮点运算顺序。
 * 这属于源码／反汇编审查结论，尚不代表本版实机逐位验证通过。每段循环使用
 * main/dsp_biquad_s31.S 的 HWLOOP 内核：
 *   - dsp_biquad_s31_pair()：两个独立滤波器（输入级/响度/端口的 L 与 R，通道的 CH0|CH1、
 *     CH2|CH3）交错在一个 esp.lp.setup 循环里，各自系数/状态独立，任何一路的算子顺序都不变；
 *   - dsp_biquad_s31_mono()：单路（只有一路有这一级时）。
 * 有跨采样状态的阶段（压缩器 comp_gain_db / 限幅器 lim_gain / 端口延时环 port_delay_pos /
 * 连续性检查 last_in_l）**保持逐采样顺序不动**（见下面 ⑧⑨）。
 * 不在生产代码中保留启动探针或逐段测速。 */

#define DSP_CHUNK_FRAMES 240        /* = I2S_OUT_DMA_FRAME_NUM；更大的块会自动分段 */

static float s_buf_l[DSP_CHUNK_FRAMES];
static float s_buf_r[DSP_CHUNK_FRAMES];
static float s_ch_buf[DSP_NUM_CHANNELS][DSP_CHUNK_FRAMES];
static float s_ol_buf[DSP_CHUNK_FRAMES];
static float s_orr_buf[DSP_CHUNK_FRAMES];
IRAM_ATTR void dsp_engine_process(int32_t *pcm, int32_t *pcm_port1, size_t samples, uint32_t rate, dsp_source_t source)
{
    if (pcm == NULL || samples < 2 || s_lock == NULL) {
        if (pcm_port1 != NULL) memset(pcm_port1, 0, samples * sizeof(int32_t));
        return;
    }

    xSemaphoreTake(s_lock, portMAX_DELAY);

    dsp_engine_t *e = &s_engine;
    if (!e->ready) {
        if (pcm_port1 != NULL) {
            memset(pcm_port1, 0, samples * sizeof(int32_t));
        }
        xSemaphoreGive(s_lock);
        return;
    }

    if (rate != 0 && rate != e->fs) {
        dsp_config_t snap;
        dsp_config_copy(&snap);
        rebuild(e, &snap, rate);
        reset_states(e);      /* new sample rate: stale state is meaningless */
        e->last_source = -1;
    }

    const int src = (source == DSP_SOURCE_BT) ? 1 : 0;
    if (src != e->last_source) {
        reset_states(e);
        e->last_source = src;
    }

    if (e->bypass) {
        if (pcm_port1 != NULL) {
            /* Neutral engine: port 1 carries the same bit-perfect stereo as
             * port 0. If port 1 is disabled, keep it silent. */
            if (e->port_enabled[1]) {
                if (e->port_mode[1] == DSP_PORT_MODE_STEREO) {
                    memcpy(pcm_port1, pcm, samples * sizeof(int32_t));
                } else {
                    const size_t ch = e->port_mode[1] == DSP_PORT_MODE_RIGHT_ONLY ? 1u : 0u;
                    for (size_t i = 0; i + 1 < samples; i += 2) {
                        pcm_port1[i] = pcm_port1[i + 1] = pcm[i + ch];
                    }
                }
            } else {
                memset(pcm_port1, 0, samples * sizeof(int32_t));
            }
        }
        xSemaphoreGive(s_lock);
        return;                      /* bit-perfect: leave the int32 block alone */
    }

    const float scale = 1.0f / 2147483648.0f;
    const size_t frames = samples / 2;
    e->process_calls++;
    const uint32_t call_no = e->process_calls;   /* 连续性检查判"头两块不查"用 */

    const bool delay0 = (e->port_delay_frames[0] > 0 && e->port_delay_buf[0] != NULL);
    const uint32_t dcap0 = e->port_delay_cap[0];
    const uint32_t dfr0  = e->port_delay_frames[0];

    const dsp_bank_t *in_eq = &e->in_eq[src];
    const dsp_bank_t *in_req = &e->in_req[src];
    const float in_gain = e->in_gain[src];
    const bool  in_mute = e->in_mute[src];
    const bool  in_phase = e->in_phase[src];
    float rl[DSP_NUM_CHANNELS], rr[DSP_NUM_CHANNELS];
    for (int ci = 0; ci < DSP_NUM_CHANNELS; ci++) {
        rl[ci] = e->route_l[src][ci];
        rr[ci] = e->route_r[src][ci];
    }
    const float master = e->master;
    const bool  loud_on = e->loud_on;
    const bool  port_en = e->port_enabled[0];
    const uint8_t port_mode = e->port_mode[0];
    const float port_gain = e->port_gain[0];
    const uint8_t port_hp_n = e->port_hp_n[0];
    const uint8_t port_lp_n = e->port_lp_n[0];
    const bool  lim_on = e->lim_on;
    const float lim_thresh = e->lim_thresh;
    const float lim_release_coef = e->lim_release_coef;
    float lim_gain = e->lim_gain;
    const bool  comp_on = e->comp_on;
    const float comp_thresh = e->comp_thresh;
    const float comp_slope = e->comp_slope;
    const float comp_makeup_db = e->comp_makeup_db;
    const float comp_att_coef = e->comp_att_coef;
    const float comp_rel_coef = e->comp_rel_coef;
    float comp_gain_db = e->comp_gain_db;
    uint32_t clips = 0;
    uint32_t jumps = 0;
    uint32_t jumps_mid = 0;

    for (size_t off = 0; off < frames; off += DSP_CHUNK_FRAMES) {
        size_t n = frames - off;
        if (n > DSP_CHUNK_FRAMES) {
            n = DSP_CHUNK_FRAMES;      /* 实际块恒为 240，这段只是防御性分段 */
        }

        /* ① 载入 + 输入级 增益/静音/反相 + 连续性检查（逐采样，顺序与旧版一致） */
        for (size_t i = 0; i < n; i++) {
            float l = (float)pcm[2 * (off + i)] * scale;
            float r = (float)pcm[2 * (off + i) + 1] * scale;

            /* Continuity check against the previous input sample (which may be the
             * last sample of the previous block). */
            if (call_no > 2 && fabsf(l - e->last_in_l) > 0.05f) {
                if (off + i == 0) {
                    jumps++;
                } else {
                    jumps_mid++;
                }
            }
            e->last_in_l = l;

            if (in_mute) {
                l = 0.0f;
                r = 0.0f;
            } else {
                if (in_gain != 1.0f) {
                    l *= in_gain;
                    r *= in_gain;
                }
                if (in_phase) {
                    l = -l;
                    r = -r;
                }
            }
            s_buf_l[i] = l;
            s_buf_r[i] = r;
        }

        /* ② 输入级双二阶：**立体声** —— L/R 各一条独立递归线。每个"段"跑两次，
         *    所以计入预算的 1 段在输入级其实是 2 段的工作量（HANDOFF §4.7 早写明）。 */
        /* L/R 同系数、独立状态，在同一个 HWLOOP 中交错计算。 */
        for (uint8_t k = 0; k < in_eq->n; k++) {
            dsp_biquad_s31_pair(&in_eq->c[k], &in_eq->c[k],
                                &e->in_eq_z[src][0][k], &e->in_eq_z[src][1][k],
                                s_buf_l, s_buf_r, (int)n);
        }
        for (uint8_t k = 0; k < in_req->n; k++) {
            dsp_biquad_s31_pair(&in_req->c[k], &in_req->c[k],
                                &e->in_req_z[src][0][k], &e->in_req_z[src][1][k],
                                s_buf_l, s_buf_r, (int)n);
        }

        /* ③ 路由：每格可选源声道（默认 CH1<-L / CH2<-R，也可 L/R 互换或单声道） */
        for (int ci = 0; ci < DSP_NUM_CHANNELS; ci++) {
            const float al = rl[ci];
            const float ar = rr[ci];
            float *const cb = s_ch_buf[ci];
            for (size_t i = 0; i < n; i++) {
                cb[i] = al * s_buf_l[i] + ar * s_buf_r[i];
            }
        }

        /* ④ 通道链（单声道）：PEQ → 高通 → 低通 → ×master */
        for (int ci = 0; ci < DSP_NUM_CHANNELS; ci += 2) {
            const int cj = ci + 1;
            float *const ba = s_ch_buf[ci];
            float *const bb = s_ch_buf[cj];
            const dsp_bank_t *ea = &e->ch_eq[ci];
            const dsp_bank_t *eb = &e->ch_eq[cj];
            /* PEQ → HP → LP：每级取 max(nA,nB)；两级都在就用 pair（各自系数/状态），
             * 只有一路有的尾级退回 mono。链序与逐通道块处理逐位一致。 */
            uint8_t nmax = (ea->n > eb->n) ? ea->n : eb->n;
            for (uint8_t k = 0; k < nmax; k++) {
                if (k < ea->n && k < eb->n) {
                    dsp_biquad_s31_pair(&ea->c[k], &eb->c[k], &e->ch_eq_z[ci][k], &e->ch_eq_z[cj][k],
                                        ba, bb, (int)n);
                } else if (k < ea->n) {
                    dsp_biquad_s31_mono(&ea->c[k], &e->ch_eq_z[ci][k], ba, (int)n);
                } else {
                    dsp_biquad_s31_mono(&eb->c[k], &e->ch_eq_z[cj][k], bb, (int)n);
                }
            }
            nmax = (e->ch_hp_n[ci] > e->ch_hp_n[cj]) ? e->ch_hp_n[ci] : e->ch_hp_n[cj];
            for (uint8_t k = 0; k < nmax; k++) {
                if (k < e->ch_hp_n[ci] && k < e->ch_hp_n[cj]) {
                    dsp_biquad_s31_pair(&e->ch_hp_c[ci][k], &e->ch_hp_c[cj][k],
                                        &e->ch_hp_z[ci][k], &e->ch_hp_z[cj][k], ba, bb, (int)n);
                } else if (k < e->ch_hp_n[ci]) {
                    dsp_biquad_s31_mono(&e->ch_hp_c[ci][k], &e->ch_hp_z[ci][k], ba, (int)n);
                } else {
                    dsp_biquad_s31_mono(&e->ch_hp_c[cj][k], &e->ch_hp_z[cj][k], bb, (int)n);
                }
            }
            nmax = (e->ch_lp_n[ci] > e->ch_lp_n[cj]) ? e->ch_lp_n[ci] : e->ch_lp_n[cj];
            for (uint8_t k = 0; k < nmax; k++) {
                if (k < e->ch_lp_n[ci] && k < e->ch_lp_n[cj]) {
                    dsp_biquad_s31_pair(&e->ch_lp_c[ci][k], &e->ch_lp_c[cj][k],
                                        &e->ch_lp_z[ci][k], &e->ch_lp_z[cj][k], ba, bb, (int)n);
                } else if (k < e->ch_lp_n[ci]) {
                    dsp_biquad_s31_mono(&e->ch_lp_c[ci][k], &e->ch_lp_z[ci][k], ba, (int)n);
                } else {
                    dsp_biquad_s31_mono(&e->ch_lp_c[cj][k], &e->ch_lp_z[cj][k], bb, (int)n);
                }
            }
            for (size_t i = 0; i < n; i++) {
                ba[i] *= master;
                bb[i] *= master;
            }
        }

        /* ⑤ 响度补偿：主音量之后、端口映射之前（L/R 各一对架式滤波器）。
         *    lo 与 hi 的状态序列互不影响 ⇒「整块跑完 lo 再整块跑 hi」与逐采样
         *    「lo→hi」逐位等价；L 与 R 同理。 */
        if (loud_on) {
            dsp_biquad_s31_pair(&e->loud_lo_c, &e->loud_lo_c, &e->loud_lo_z[0], &e->loud_lo_z[1],
                                s_ch_buf[0], s_ch_buf[1], (int)n);
            dsp_biquad_s31_pair(&e->loud_hi_c, &e->loud_hi_c, &e->loud_hi_z[0], &e->loud_hi_z[1],
                                s_ch_buf[0], s_ch_buf[1], (int)n);
        }

        /* ⑥ 输出端口 I2S0：模式映射（端口 p 用 CH(2p+1)/CH(2p+2)，此处即 ch[0]/ch[1]） */
        if (port_en) {
            const float *const c0 = s_ch_buf[0];
            const float *const c1 = s_ch_buf[1];
            if (port_mode == DSP_PORT_MODE_LEFT_ONLY) {
                for (size_t i = 0; i < n; i++) { s_ol_buf[i] = c0[i]; s_orr_buf[i] = c0[i]; }
            } else if (port_mode == DSP_PORT_MODE_RIGHT_ONLY) {
                for (size_t i = 0; i < n; i++) { s_ol_buf[i] = c1[i]; s_orr_buf[i] = c1[i]; }
            } else {
                for (size_t i = 0; i < n; i++) { s_ol_buf[i] = c0[i]; s_orr_buf[i] = c1[i]; }
            }

            /* ⑦ 端口 HP/LP：模式映射之后、端口增益之前 ----
             * 系数两个槽共用，状态每槽独立（[port][slot][stage]），所以左右
             * 声道各走自己的延迟线，不会互相串音。 */
            for (uint8_t k = 0; k < port_hp_n; k++) {
                dsp_biquad_s31_pair(&e->port_hp_c[0][k], &e->port_hp_c[0][k],
                                    &e->port_hp_z[0][0][k], &e->port_hp_z[0][1][k],
                                    s_ol_buf, s_orr_buf, (int)n);
            }
            for (uint8_t k = 0; k < port_lp_n; k++) {
                dsp_biquad_s31_pair(&e->port_lp_c[0][k], &e->port_lp_c[0][k],
                                    &e->port_lp_z[0][0][k], &e->port_lp_z[0][1][k],
                                    s_ol_buf, s_orr_buf, (int)n);
            }

            if (port_gain != 1.0f) {
                for (size_t i = 0; i < n; i++) {
                    s_ol_buf[i] *= port_gain;
                    s_orr_buf[i] *= port_gain;
                }
            }
        } else {
            for (size_t i = 0; i < n; i++) {
                s_ol_buf[i] = 0.0f;
                s_orr_buf[i] = 0.0f;
            }
        }

        /* ⑧⑨ 跨采样状态阶段：压缩器 → 限幅器 → 端口延时 → 削波计数 → 转回 int32。
         *      **必须逐采样**（状态跨采样/跨块），顺序与旧版一字不差。 */
        for (size_t i = 0; i < n; i++) {
            float ol = s_ol_buf[i];
            float orr = s_orr_buf[i];

            if (port_en) {
                /* ---- 压缩器 / 夜间模式：端口增益之后、削波保护之前 ----
                 * 立体声联动（取两槽较大者做检波），增益在 dB 域一阶平滑。
                 * 静态信号下增益会精确收敛到曲线值，所以电平可以被逐条预测与实测比对。
                 * v0.29：log10f/powf 换成快速 log2/exp2（见 dsp_filter.h）——
                 *        静态增益律最大偏差实测 1.0e-05 dB（要求 < 1e-3 dB）。 */
                if (comp_on) {
                    const float cpeak = fmaxf(fabsf(ol), fabsf(orr));
                    const float x_db = DSP_LOG2_10 * dsp_fast_log2(cpeak + 1e-12f);
                    const float over = x_db - comp_thresh;
                    float g_target = comp_makeup_db;
                    if (over > 0.0f) {
                        g_target -= over * comp_slope;
                    }
                    if (g_target < comp_gain_db) {
                        comp_gain_db += (g_target - comp_gain_db) * comp_att_coef;
                    } else {
                        comp_gain_db += (g_target - comp_gain_db) * comp_rel_coef;
                    }
                    const float cg = dsp_fast_exp2(comp_gain_db * DSP_LOG2_10_20);
                    ol *= cg;
                    orr *= cg;
                }
                /* ---- 削波保护：立体声联动峰值限幅（瞬时压制 + 指数释放） ---- */
                if (lim_on) {
                    const float peak = fmaxf(fabsf(ol), fabsf(orr));
                    if (peak > lim_thresh) {
                        const float need = lim_thresh / peak;
                        if (need < lim_gain) {
                            lim_gain = need;          /* 瞬时 attack，本样本就压住 */
                        }
                    }
                    lim_gain += (1.0f - lim_gain) * lim_release_coef;
                    if (lim_gain > 1.0f) {
                        lim_gain = 1.0f;
                    }
                    ol *= lim_gain;
                    orr *= lim_gain;
                }
                if (delay0) {
                    float *buf = e->port_delay_buf[0];
                    uint32_t w = e->port_delay_pos[0];
                    buf[2 * w] = ol;
                    buf[2 * w + 1] = orr;
                    int32_t rd = (int32_t)w - (int32_t)dfr0;
                    if (rd < 0) {
                        rd += (int32_t)dcap0;
                    }
                    ol = buf[2 * rd];
                    orr = buf[2 * rd + 1];
                    w++;
                    if (w >= dcap0) {
                        w = 0;
                    }
                    e->port_delay_pos[0] = w;
                }
            }

            if (ol > 1.0f || ol < -1.0f || orr > 1.0f || orr < -1.0f) {
                clips++;
            }

            pcm[2 * (off + i)] = float_to_i32(ol);
            pcm[2 * (off + i) + 1] = float_to_i32(orr);

            if (pcm_port1 != NULL) {
                if (e->port_enabled[1]) {
                    float p1l = s_ch_buf[2][i];
                    float p1r = s_ch_buf[3][i];
                    if (e->port_mode[1] == DSP_PORT_MODE_LEFT_ONLY) {
                        p1r = p1l;
                    } else if (e->port_mode[1] == DSP_PORT_MODE_RIGHT_ONLY) {
                        p1l = p1r;
                    }
                    pcm_port1[2 * (off + i)] = float_to_i32(p1l);
                    pcm_port1[2 * (off + i) + 1] = float_to_i32(p1r);
                } else {
                    pcm_port1[2 * (off + i)] = 0;
                    pcm_port1[2 * (off + i) + 1] = 0;
                }
            }
        }
    }

    e->lim_gain = lim_gain;      /* 把限幅器状态写回（跨块保持） */
    e->comp_gain_db = comp_gain_db;  /* 压缩器状态同理 */

    if (clips != 0) {
        e->clips += clips;
    }
    if (jumps != 0) {
        e->in_jumps += jumps;
    }
    if (jumps_mid != 0) {
        e->in_jumps_mid += jumps_mid;
    }

    xSemaphoreGive(s_lock);
}
