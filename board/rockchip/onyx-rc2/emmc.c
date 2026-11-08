// SPDX-License-Identifier: GPL-2.0+
/*
 * ONYX RC2, U-Boot started by the stock loader from the eMMC
 * (RC2_EMMC_LOADER): keep the eMMC as the stock loader expects it.
 *
 * Only the user area. MMC_WRITE off removes every block write, but a
 * hardware partition selected by name ("ums 0 mmc 1.1", "load mmc 1.2
 * ...") still sends CMD6 SWITCH to EXT_CSD[179] PARTITION_CONFIG, whose
 * boot partition enable and boot ack bits are written back with it (as
 * U-Boot read them) and are non-volatile. Once an MMC block device has
 * probed (mmc_init() done, on the user area), mark its hardware
 * partitions unavailable: mmc_select_hwpart() then refuses every
 * partition but the one it is on with -EMEDIUMTYPE, and no SWITCH is
 * sent. A microSD card has none anyway.
 */

#include <dm.h>
#include <event.h>
#include <mmc.h>

static int rc2_emmc_user_area_only(void *ctx, struct event *event)
{
	struct udevice *dev = event->data.dm.dev;
	struct mmc *mmc;

	if (device_get_uclass_id(dev) != UCLASS_BLK ||
	    device_get_uclass_id(dev_get_parent(dev)) != UCLASS_MMC)
		return 0;
	mmc = mmc_get_mmc_dev(dev_get_parent(dev));
	if (mmc && !IS_SD(mmc))
		mmc->part_config = MMCPART_NOAVAILABLE;

	return 0;
}

EVENT_SPY_FULL(EVT_DM_POST_PROBE, rc2_emmc_user_area_only);
