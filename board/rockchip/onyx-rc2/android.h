/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * ONYX RC2: booting the stock kernel from the eMMC (android.c).
 */

#ifndef __ONYX_RC2_ANDROID_H
#define __ONYX_RC2_ANDROID_H

#include <linux/types.h>

/* record the pads as the stock loader left them; before any probe */
void rc2a_pads_save(void);

/* GRF_OS_REG0..5 as U-Boot found them (onyx-rc2.c) */
#define RC2_OS_REGS		6
#define RC2_FLASHDATA_WARM	0x13247865	/* the stock DDR init's warm marker in OS_REG3 */
extern u32 rc2_os_reg_entry[RC2_OS_REGS];

#endif
