/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * RC2 "core" reset, copied to IMEM and run there with the MMU and caches
 * off (onyx-rc2.c rc2_core_reset). Position independent: only PC-relative
 * literals. The order follows the stock 3.0 kernel (rk30_arch_reset /
 * soft_reset): all PLLs to slow mode (24 MHz, DDR unusable from here on),
 * a short wait, then SOFTRST_CON0 = 0x801cffff (MCORE, CORE0, CORE1, L2C).
 */
#include <linux/linkage.h>

	.section .rodata.rc2_core_reset, "a"
	.arm
	.align	2
	.global	rc2_core_reset_start
	.global	rc2_core_reset_end
rc2_core_reset_start:
	ldr	r0, 10f			@ CRU
	ldr	r1, 11f
	str	r1, [r0, #0x40]		@ CRU_MODE_CON: every PLL to slow mode
	dsb
	mov	r2, #0x4000		@ a few ms at 24 MHz
1:	subs	r2, r2, #1
	bne	1b
	ldr	r1, 12f
	str	r1, [r0, #0x110]	@ CRU_SOFTRST_CON0: core resets
	dsb
2:	wfi
	b	2b
	.align	2
10:	.word	0x20000000
11:	.word	0xffff0000
12:	.word	0x801cffff
rc2_core_reset_end:
