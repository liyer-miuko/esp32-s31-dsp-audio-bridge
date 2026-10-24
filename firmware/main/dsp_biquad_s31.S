/*
 * dsp_biquad_s31.S - ESP32-S31 (rv32imafc, ilp32f) transposed-DF2 biquad kernels.
 *
 * Bit-exact contract with dsp_filter.h dsp_biquad_process() / the 0.119 C block:
 *     y  = fmadd(b0, x, z1);
 *     t  = a1 * y;             u  = a2 * y;
 *     z1 = fmsub(b1, x, t);    z1 = z1 + z2;    z2 = fmsub(b2, x, u);
 * buf is processed in place: y is stored first, then that same FP register
 * carries the new z1 (no fs registers, no spill, no call inside the loop).
 *
 * HWLOOP: esp.lp.setup 0, <n>, .Lend ; .Lend marks the last executed instruction
 * (the trailing pointer addi), .option norvc keeps every instruction 4-byte wide.
 * n <= 0 falls through to ret without touching memory.
 *
 * caller-saved FP only: ft0-ft11, fa0-fa7 (20 registers, ilp32f).
 */
	.option norvc
	.section .iram1, "ax", @progbits
	.p2align 2

/* void dsp_biquad_s31_mono(const dsp_biquad_coeff_t *c, dsp_biquad_state_t *st,
 *                          float *buf, int n)   -> a0, a1, a2, a3 */
	.globl	dsp_biquad_s31_mono
	.type	dsp_biquad_s31_mono, @function
dsp_biquad_s31_mono:
	blez	a3, .Lmono_ret
	flw	ft0, 0(a0)		/* b0 */
	flw	ft1, 4(a0)		/* b1 */
	flw	ft2, 8(a0)		/* b2 */
	flw	ft3, 12(a0)		/* a1 */
	flw	ft4, 16(a0)		/* a2 */
	flw	fa0, 0(a1)		/* z1 (becomes y) */
	flw	fa1, 4(a1)		/* z2 */
	esp.lp.setup 0, a3, .Lmono_end
	flw	fa4, 0(a2)		/* x */
	fmadd.s	fa0, ft0, fa4, fa0	/* y  = b0*x + z1 */
	fmul.s	ft10, ft3, fa0		/* t  = a1*y */
	fmul.s	fa6, ft4, fa0		/* u  = a2*y */
	fsw	fa0, 0(a2)		/* buf[i] = y, fa0 free afterwards */
	fmsub.s	fa0, ft1, fa4, ft10	/* z1 = b1*x - t */
	fadd.s	fa0, fa0, fa1		/* z1 = z1 + z2 */
	fmsub.s	fa1, ft2, fa4, fa6	/* z2 = b2*x - u */
.Lmono_end:
	addi	a2, a2, 4		/* last executed instruction = .Lend */
	fsw	fa0, 0(a1)
	fsw	fa1, 4(a1)
.Lmono_ret:
	ret
	.size	dsp_biquad_s31_mono, . - dsp_biquad_s31_mono

/* void dsp_biquad_s31_pair(const dsp_biquad_coeff_t *cL, const dsp_biquad_coeff_t *cR,
 *                          dsp_biquad_state_t *stL, dsp_biquad_state_t *stR,
 *                          float *bufL, float *bufR, int n)
 *   a0=cL a1=cR a2=stL a3=stR a4=bufL a5=bufR a6=n */
	.globl	dsp_biquad_s31_pair
	.type	dsp_biquad_s31_pair, @function
	.p2align 2
dsp_biquad_s31_pair:
	blez	a6, .Lpair_ret
	flw	ft0, 0(a0)		/* L: b0 */
	flw	ft1, 4(a0)		/* L: b1 */
	flw	ft2, 8(a0)		/* L: b2 */
	flw	ft3, 12(a0)		/* L: a1 */
	flw	ft4, 16(a0)		/* L: a2 */
	flw	ft5, 0(a1)		/* R: b0 */
	flw	ft6, 4(a1)		/* R: b1 */
	flw	ft7, 8(a1)		/* R: b2 */
	flw	ft8, 12(a1)		/* R: a1 */
	flw	ft9, 16(a1)		/* R: a2 */
	flw	fa0, 0(a2)		/* L: z1 */
	flw	fa1, 4(a2)		/* L: z2 */
	flw	fa2, 0(a3)		/* R: z1 */
	flw	fa3, 4(a3)		/* R: z2 */
	esp.lp.setup 0, a6, .Lpair_end
	flw	fa4, 0(a4)		/* xL */
	flw	fa5, 0(a5)		/* xR */
	fmadd.s	fa0, ft0, fa4, fa0	/* yL = b0L*xL + z1L */
	fmadd.s	fa2, ft5, fa5, fa2	/* yR = b0R*xR + z1R */
	addi	a4, a4, 4              /* independent pointer work while y becomes ready */
	fmul.s	ft10, ft3, fa0		/* tL = a1L*yL */
	fmul.s	ft11, ft8, fa2		/* tR = a1R*yR */
	fmul.s	fa6, ft4, fa0		/* uL = a2L*yL */
	fmul.s	fa7, ft9, fa2		/* uR = a2R*yR */
	fsw	fa0, -4(a4)		/* bufL[i] = yL */
	fsw	fa2, 0(a5)		/* bufR[i] = yR */
	fmsub.s	fa0, ft1, fa4, ft10	/* z1L = b1L*xL - tL */
	fmsub.s	fa2, ft6, fa5, ft11	/* z1R = b1R*xR - tR */
	fadd.s	fa0, fa0, fa1		/* z1L = z1L + z2L */
	fadd.s	fa2, fa2, fa3		/* z1R = z1R + z2R */
	fmsub.s	fa1, ft2, fa4, fa6	/* z2L = b2L*xL - uL */
	fmsub.s	fa3, ft7, fa5, fa7	/* z2R = b2R*xR - uR */
.Lpair_end:
	addi	a5, a5, 4		/* last executed instruction = .Lend */
	fsw	fa0, 0(a2)
	fsw	fa1, 4(a2)
	fsw	fa2, 0(a3)
	fsw	fa3, 4(a3)
.Lpair_ret:
	ret
	.size	dsp_biquad_s31_pair, . - dsp_biquad_s31_pair
