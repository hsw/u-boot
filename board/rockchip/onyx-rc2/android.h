/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * ONYX RC2: booting the stock kernel from the eMMC (android.c).
 */

#ifndef __ONYX_RC2_ANDROID_H
#define __ONYX_RC2_ANDROID_H

/* record the pads as the stock loader left them; before any probe */
void rc2a_pads_save(void);

#endif
