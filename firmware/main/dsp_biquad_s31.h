/*
 * dsp_biquad_s31.h — S31 硬件循环双二阶内核（0.121；仅构建，待外部验证）
 *
 * 固定工具链运算契约：保持0.119块宏实际反汇编中的每路浮点算子及顺序；
 * 此契约经过静态审查，尚未宣称本内核实机逐位验证通过：
 *     y  = fmadd(b0, x, z1);
 *     t  = a1 * y;              u  = a2 * y;
 *     z1 = fmsub(b1, x, t);     z1 = z1 + z2;     z2 = fmsub(b2, x, u);
 * 单精度、无 fast-math 假设、不改 fcsr；HWLOOP 内无函数调用、无栈、无 spill。
 *
 * n <= 0 直接返回（不碰任何内存，不读 buf[n] 之外的样本）；n = 1 / 奇数 / 240 都可以。
 * buf 原地处理：先把 y 写回，再让承载 y 的浮点寄存器变成新的 z1。
 *
 * pair：两路完全独立的递归交错在同一个 esp.lp.setup 循环里，可带各自的系数与状态；
 *       L/R 用同一组系数时两个 c 指针传同一个即可。两路之间不共享任何状态。
 *       ABI 顺序：cL, cR, stL, stR, bufL, bufR, n（a0..a6）。
 */
#pragma once

#include "dsp_filter.h"

#ifdef __cplusplus
extern "C" {
#endif

void dsp_biquad_s31_mono(const dsp_biquad_coeff_t *c, dsp_biquad_state_t *st, float *buf, int n);

void dsp_biquad_s31_pair(const dsp_biquad_coeff_t *cL, const dsp_biquad_coeff_t *cR,
                         dsp_biquad_state_t *stL, dsp_biquad_state_t *stR,
                         float *bufL, float *bufR, int n);

#ifdef __cplusplus
}
#endif
