// SPDX-License-Identifier: GPL-2.0+
/*
 * ONYX RC2, U-Boot started by the stock loader from the eMMC
 * (RC2_EMMC_LOADER): keep the eMMC as the stock loader and the BootROM
 * expect it.
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

#include <command.h>
#include <dm.h>
#include <event.h>
#include <mmc.h>
#include <stdio.h>
#include <time.h>
#include <asm/io.h>
#include <linux/bitops.h>
#include <linux/delay.h>
#include "emmc.h"

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

/*
 * The eMMC back to its pre-boot state. rc2_sysreset_request() runs it
 * before every reset this U-Boot issues ("reset", panic, and the watchdog
 * fire while the watchdog runs, which goes through the same path).
 *
 * After a warm reset the RK3026 BootROM reads the eMMC only with the boot
 * operation: CMD held low, 1-bit, no boot ack. It resets the controller,
 * its FIFO and DMA (CTRL = 3) and programs the card clock (CLKDIV, CLKENA,
 * a clock update), but leaves the bus width (CTYPE) and UHS_REG as it
 * finds them, sends no CMD0 and does not pulse RST_n. A device that
 * FlashBoot and this U-Boot left in Transfer state, 8-bit, behind a
 * controller set up for that, never sends boot data, and the ROM ends in
 * MaskROM mode: with no card in the slot, the book does not come back.
 * The stock kernel sends CMD0 0xF0F0F0F0 in its flash driver's shutdown,
 * FlashBoot does the same before its soft reset.
 *
 * Tried on the book: CMD0 0xF0F0F0F0 alone, an RST_n pulse alone, both
 * together and the controller's CTYPE alone each still end in MaskROM;
 * the whole sequence below reaches Android. It is not bisected, so it is
 * kept whole and in the tested order: CTYPE 0 (1-bit), CLKDIV 0 and
 * UHS_REG 0; an RST_n pulse (RST_n_FUNCTION is enabled in the book's
 * EXT_CSD; JEDEC asks for tRSTW >= 1 us, the test held it 1 s, this holds
 * it 100 ms, then waits 1 ms for tRSCA >= 200 us); CMD0 GO_PRE_IDLE.
 * CLKDIV 0 is there because the test wrote it, not for a known reason:
 * the controller loads CLKDIV only on a clock-update command, which this
 * sequence does not send, and the ROM programs its own divider.
 *
 * The pulse on the controller's RST_N register reaches the device only
 * while GPIO1_C7 is muxed to the controller's RST_n output: the eMMC
 * node's pinctrl (emmc_rstnout) does that when mmc1 probes, and the
 * BootROM muxes the pad the same way on its eMMC path.
 *
 * No data is written, no clock is gated or changed in the CRU, CLKENA
 * stays as it is. The eMMC is unusable afterwards until it is
 * initialised again: only a reset may follow. Run twice ("rc2preboot"
 * and then "reset") it does the same steps again, which is harmless.
 *
 * Not covered: a watchdog reset of a hung system runs no code first, and
 * "wdt expire" goes to the watchdog driver directly; both leave the eMMC
 * as it was.
 */
#define EMMC_BASE		0x1021c000	/* dw_mmc of the eMMC */
#define EMMC_CLKDIV		(EMMC_BASE + 0x08)
#define EMMC_CTYPE		(EMMC_BASE + 0x18)
#define EMMC_CMDARG		(EMMC_BASE + 0x28)
#define EMMC_CMD		(EMMC_BASE + 0x2c)
#define EMMC_RINTSTS		(EMMC_BASE + 0x44)
#define EMMC_UHS_REG		(EMMC_BASE + 0x74)
#define EMMC_RST_N		(EMMC_BASE + 0x78)
#define EMMC_CMD_START		BIT(31)
#define EMMC_CMD_HOLD		BIT(29)		/* use_hold_reg */
#define EMMC_CMD_PRVDATA	BIT(13)		/* wait_prvdata_complete */
#define EMMC_INT_CMD_DONE	BIT(2)
#define EMMC_INT_HLE		BIT(12)
#define EMMC_ARG_PRE_IDLE	0xf0f0f0f0
#define EMMC_RST_LOW_MS		100
#define EMMC_CMD_TIMEOUT_MS	100

void rc2_emmc_preboot(void)
{
	u32 cmd, rint;
	ulong start;

	writel(0, (void __iomem *)EMMC_CTYPE);
	writel(0, (void __iomem *)EMMC_CLKDIV);
	writel(0, (void __iomem *)EMMC_UHS_REG);
	writel(0, (void __iomem *)EMMC_RST_N);
	mdelay(EMMC_RST_LOW_MS);
	writel(1, (void __iomem *)EMMC_RST_N);
	mdelay(1);
	writel(0xffffffff, (void __iomem *)EMMC_RINTSTS);
	writel(EMMC_ARG_PRE_IDLE, (void __iomem *)EMMC_CMDARG);
	/* CMD0, no response */
	writel(EMMC_CMD_START | EMMC_CMD_HOLD | EMMC_CMD_PRVDATA, (void __iomem *)EMMC_CMD);

	start = get_timer(0);
	do {
		cmd = readl((void __iomem *)EMMC_CMD);
		rint = readl((void __iomem *)EMMC_RINTSTS);
		if (!(cmd & EMMC_CMD_START) && (rint & (EMMC_INT_CMD_DONE | EMMC_INT_HLE)))
			break;
	} while (get_timer(start) < EMMC_CMD_TIMEOUT_MS);

	if (!(cmd & EMMC_CMD_START) && (rint & EMMC_INT_CMD_DONE) && !(rint & EMMC_INT_HLE))
		printf("rc2: eMMC to pre-boot state: 1-bit, RST_n %d ms, GO_PRE_IDLE done\n",
		       EMMC_RST_LOW_MS);
	else
		printf("rc2: eMMC to pre-boot state: 1-bit, RST_n %d ms, GO_PRE_IDLE not done (CMD %08x, RINTSTS %08x)\n",
		       EMMC_RST_LOW_MS, cmd, rint);
}

static int do_rc2preboot(struct cmd_tbl *cmdtp, int flag, int argc, char *const argv[])
{
	rc2_emmc_preboot();

	return CMD_RET_SUCCESS;
}

U_BOOT_CMD(rc2preboot, 1, 0, do_rc2preboot,
	   "put the eMMC back in its pre-boot state, right before a reset",
	   "\n"
	   "    as every reset from this U-Boot does first; the eMMC is unusable\n"
	   "    until it is initialised again");
