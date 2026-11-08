/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * ONYX RC2, U-Boot started by the stock loader from the eMMC (emmc.c).
 */

#ifndef __ONYX_RC2_EMMC_H
#define __ONYX_RC2_EMMC_H

/* the eMMC to its pre-boot state (RST_n, GO_PRE_IDLE); right before a reset only */
void rc2_emmc_preboot(void);

#endif
