/*
 * Biquad filter design - ESP32-S31 audio bridge.
 *
 * Formulas match the WebUI's coeff() and xoStageList() exactly; keep the two
 * in sync when either side changes.
 */
#include <math.h>
#include <string.h>

#include "dsp_filter.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static float clampf_local(float v, float lo, float hi)
{
    return (v < lo) ? lo : ((v > hi) ? hi : v);
}

void dsp_biquad_design(dsp_biquad_coeff_t *c, uint8_t type, float f, float g_db, float q, uint32_t fs)
{
    if (c == NULL) {
        return;
    }

    if (fs == 0) {
        fs = 48000;
    }

    /* WebUI clamps the same way before designing. */
    f = clampf_local(f, DSP_BAND_FREQ_MIN_HZ, DSP_BAND_FREQ_MAX_HZ);
    g_db = clampf_local(g_db, DSP_BAND_GAIN_MIN_DB, DSP_BAND_GAIN_MAX_DB);
    q = (q < 0.01f) ? 0.01f : q;

    /* 0 dB gain on the three gain-carrying types is mathematically the identity
     * transfer function: b = [1,0,0], a = [1,0,0]. Emit it exactly instead of
     * dividing by a0 -- inv_a0 rounding plus fmadd/fmsub residue is amplified by
     * the biquad recursion, which is what a 0 dB EQ chain shows as sample error.
     * HP/LP/bandpass/notch/allpass never take this branch; non-zero gain is
     * unchanged (same formulas, same order of operations). */
    if (g_db == 0.0f &&
        (type == DSP_FILTER_PEAKING || type == DSP_FILTER_LOWSHELF || type == DSP_FILTER_HIGHSHELF)) {
        c->b0 = 1.0f; c->b1 = 0.0f; c->b2 = 0.0f; c->a1 = 0.0f; c->a2 = 0.0f;
        return;
    }

    const float A = powf(10.0f, g_db / 40.0f);
    const float w0 = 2.0f * (float)M_PI * f / (float)fs;
    const float cw = cosf(w0);
    const float sw = sinf(w0);
    const float alpha = sw / (2.0f * q);

    float b0 = 1.0f, b1 = 0.0f, b2 = 0.0f, a0 = 1.0f, a1 = 0.0f, a2 = 0.0f;

    switch (type) {
    case DSP_FILTER_PEAKING:
        b0 = 1.0f + alpha * A;
        b1 = -2.0f * cw;
        b2 = 1.0f - alpha * A;
        a0 = 1.0f + alpha / A;
        a1 = -2.0f * cw;
        a2 = 1.0f - alpha / A;
        break;

    case DSP_FILTER_LOWSHELF: {
        const float beta = 2.0f * sqrtf(A) * alpha;
        b0 = A * ((A + 1.0f) - (A - 1.0f) * cw + beta);
        b1 = 2.0f * A * ((A - 1.0f) - (A + 1.0f) * cw);
        b2 = A * ((A + 1.0f) - (A - 1.0f) * cw - beta);
        a0 = (A + 1.0f) + (A - 1.0f) * cw + beta;
        a1 = -2.0f * ((A - 1.0f) + (A + 1.0f) * cw);
        a2 = (A + 1.0f) + (A - 1.0f) * cw - beta;
        break;
    }

    case DSP_FILTER_HIGHSHELF: {
        const float beta = 2.0f * sqrtf(A) * alpha;
        b0 = A * ((A + 1.0f) + (A - 1.0f) * cw + beta);
        b1 = -2.0f * A * ((A - 1.0f) + (A + 1.0f) * cw);
        b2 = A * ((A + 1.0f) + (A - 1.0f) * cw - beta);
        a0 = (A + 1.0f) - (A - 1.0f) * cw + beta;
        a1 = 2.0f * ((A - 1.0f) - (A + 1.0f) * cw);
        a2 = (A + 1.0f) - (A - 1.0f) * cw - beta;
        break;
    }

    case DSP_FILTER_LOWPASS:
        b0 = (1.0f - cw) / 2.0f;
        b1 = 1.0f - cw;
        b2 = (1.0f - cw) / 2.0f;
        a0 = 1.0f + alpha;
        a1 = -2.0f * cw;
        a2 = 1.0f - alpha;
        break;

    case DSP_FILTER_HIGHPASS:
        b0 = (1.0f + cw) / 2.0f;
        b1 = -(1.0f + cw);
        b2 = (1.0f + cw) / 2.0f;
        a0 = 1.0f + alpha;
        a1 = -2.0f * cw;
        a2 = 1.0f - alpha;
        break;

    case DSP_FILTER_BANDPASS:
        b0 = alpha;
        b1 = 0.0f;
        b2 = -alpha;
        a0 = 1.0f + alpha;
        a1 = -2.0f * cw;
        a2 = 1.0f - alpha;
        break;

    case DSP_FILTER_NOTCH:
        b0 = 1.0f;
        b1 = -2.0f * cw;
        b2 = 1.0f;
        a0 = 1.0f + alpha;
        a1 = -2.0f * cw;
        a2 = 1.0f - alpha;
        break;

    case DSP_FILTER_ALLPASS:
        b0 = 1.0f - alpha;
        b1 = -2.0f * cw;
        b2 = 1.0f + alpha;
        a0 = 1.0f + alpha;
        a1 = -2.0f * cw;
        a2 = 1.0f - alpha;
        break;

    default:
        /* Unknown type: bit-perfect passthrough. */
        c->b0 = 1.0f; c->b1 = 0.0f; c->b2 = 0.0f; c->a1 = 0.0f; c->a2 = 0.0f;
        return;
    }

    const float inv_a0 = 1.0f / a0;
    c->b0 = b0 * inv_a0;
    c->b1 = b1 * inv_a0;
    c->b2 = b2 * inv_a0;
    c->a1 = a1 * inv_a0;
    c->a2 = a2 * inv_a0;
}

static float butter_q(int order, int stage)
{
    /* WebUI: 1/(2*cos(pi*(2*stage+1)/(2*order))) */
    return 1.0f / (2.0f * cosf((float)M_PI * (float)(2 * stage + 1) / (float)(2 * order)));
}

uint8_t dsp_xo_design(dsp_biquad_coeff_t *stages, const dsp_xo_t *xo, bool is_highpass, uint32_t fs)
{
    if (stages == NULL || xo == NULL || !xo->on) {
        return 0;
    }

    const uint8_t type = is_highpass ? DSP_FILTER_HIGHPASS : DSP_FILTER_LOWPASS;
    uint8_t n = 0;

    if (xo->type == DSP_XO_BUTTERWORTH) {
        const int slope = (xo->slope == 1 || xo->slope == 2 || xo->slope == 4) ? xo->slope : 2;
        const int order = slope * 2;
        for (int i = 0; i < slope && n < DSP_XO_MAX_STAGES; i++) {
            dsp_biquad_design(&stages[n], type, xo->f, 0.0f, butter_q(order, i), fs);
            n++;
        }
        return n;
    }

    /* Linkwitz-Riley: cascade of two Butterworth sections. */
    if (xo->slope == 1) {
        dsp_biquad_design(&stages[n], type, xo->f, 0.0f, 0.5f, fs);
        return 1;
    }

    const int slope = (xo->slope == 2 || xo->slope == 4) ? xo->slope : 2;
    const int bw_order = slope;
    const int sections = (slope / 2 > 1) ? slope / 2 : 1;
    for (int pass = 0; pass < 2; pass++) {
        for (int i = 0; i < sections && n < DSP_XO_MAX_STAGES; i++) {
            dsp_biquad_design(&stages[n], type, xo->f, 0.0f, butter_q(bw_order, i), fs);
            n++;
        }
    }
    return n;
}
