// SPDX-License-Identifier: GPL-2.0+
/*
 * ONYX BOOX Robinson Crusoe 2 (MC_Kepler_R2), RK3026.
 *
 * U-Boot proper only. The BootROM loads the stock DDR init (FlashData) and
 * our sdboot stage2 from the microSD card; stage2 loads this image
 * from the card's boot table like a kernel and jumps to it at TEXT_BASE
 * (0x60408000) in SVC mode with the MMU and caches off, power hold high,
 * CPU1 held in reset and, for the v1 stage2, the IMEM remap set
 * (GRF_SOC_CON0 bit 12).
 *
 * Board rules:
 *  - no eMMC / NAND / SDIO node is described, so no driver can reach the
 *    eMMC; the microSD card is the only block device;
 *  - no PMIC access at all (no I2C, no regulator driver);
 *  - the IMEM remap is cleared at once and never set again: with it set,
 *    a reset restarts CPU0 in IMEM instead of the BootROM;
 *  - nothing above 0x7ff00000 is used by U-Boot (waveform window, ramoops,
 *    stage2 log and flag pages, bootcount).
 *
 * The OTG PHY bring-up and the gadget hand-off are Mario Rugiero's
 * (board/rockchip/boox-c67ml/boox-c67ml.c), same SoC.
 */

#include <cpu_func.h>
#include <dm.h>
#include <env.h>
#include <init.h>
#include <iomux.h>
#include <stdio.h>
#include <sysreset.h>
#include <asm/global_data.h>
#include <asm/io.h>
#include <asm/system.h>
#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/string.h>

DECLARE_GLOBAL_DATA_PTR;

#define CRU_BASE		0x20000000
#define CRU_CLKGATE_CON1	(CRU_BASE + 0xd4)
#define CRU_CLKGATE_CON5	(CRU_BASE + 0xe4)
#define CRU_CLKGATE_CON7	(CRU_BASE + 0xec)
#define CRU_CLKGATE_CON8	(CRU_BASE + 0xf0)
#define CRU_GLB_SRST_FST	(CRU_BASE + 0x100)
#define CRU_GLB_SRST_SND	(CRU_BASE + 0x104)
#define CRU_SOFTRST_CON4	(CRU_BASE + 0x120)
#define CRU_SOFTRST_CON5	(CRU_BASE + 0x124)

#define GRF_BASE		0x20008000
#define GRF_GPIO1A_IOMUX	(GRF_BASE + 0xb8)
#define GRF_SOC_CON0		(GRF_BASE + 0x140)
#define GRF_UOC1_CON4		(GRF_BASE + 0x190)

#define OTG_DCTL		(0x10180000 + 0x804)
#define OTG_DCTL_SFTDISCON	BIT(1)

#define GPIO1_BASE		0x20080000
#define GPIO2_BASE		0x20084000
#define GPIO_SWPORTA_DR		0x00
#define GPIO_SWPORTA_DDR	0x04
#define POWER_HOLD_BIT		BIT(2)		/* GPIO1_A2 */
#define LED_BIT			BIT(15)		/* GPIO2_B7, blue, active high */

#define IMEM_BASE		0x10080000
#define CORE_RESET_IMEM		(IMEM_BASE + 0x1000)	/* 8 KiB IMEM; stock soft_reset is at +0xf10 */

/* everything from here up is not U-Boot's (waveform, ramoops, loader pages) */
#define RC2_RAM_TOP		0x7ff00000

static void rc2_remap_off(void)
{
	/* GRF_SOC_CON0 bit 12 (hiword mask bit 28) = 0: address 0 is the BootROM */
	writel(BIT(12 + 16), (void __iomem *)GRF_SOC_CON0);
	dsb();
}

static void rc2_power_hold(void)
{
	/* as stage2 and stock FlashBoot: pin to GPIO, data high, then output */
	writel(0x00300000, (void __iomem *)GRF_GPIO1A_IOMUX);
	setbits_le32((void __iomem *)(GPIO1_BASE + GPIO_SWPORTA_DR), POWER_HOLD_BIT);
	setbits_le32((void __iomem *)(GPIO1_BASE + GPIO_SWPORTA_DDR), POWER_HOLD_BIT);
}

static void rc2_led(bool on)
{
	if (on)
		setbits_le32((void __iomem *)(GPIO2_BASE + GPIO_SWPORTA_DR), LED_BIT);
	else
		clrbits_le32((void __iomem *)(GPIO2_BASE + GPIO_SWPORTA_DR), LED_BIT);
}

int board_early_init_f(void)
{
	rc2_remap_off();
	/* pclk_gpio1, pclk_gpio2 on (CLKGATE_CON8 bits 10, 11; 0 = running) */
	writel(BIT(10 + 16) | BIT(11 + 16), (void __iomem *)CRU_CLKGATE_CON8);
	rc2_power_hold();
	setbits_le32((void __iomem *)(GPIO2_BASE + GPIO_SWPORTA_DDR), LED_BIT);
	rc2_led(true);
	/*
	 * Watchdog: pclk_wdt on (CLKGATE_CON7 bit 15) and SRST_WDT released
	 * (SOFTRST_CON5 bit 6), as stage2's wdt_arm(). The DT node carries no
	 * "resets": the rk3026-cru.h reset ids in this tree do not follow the
	 * 16 * register + bit numbering the Rockchip reset driver decodes.
	 * Nothing here enables the watchdog; "wdt start" in the boot script does.
	 */
	writel(BIT(15 + 16), (void __iomem *)CRU_CLKGATE_CON7);
	writel(BIT(6 + 16), (void __iomem *)CRU_SOFTRST_CON5);
	return 0;
}

int dram_init(void)
{
	return fdtdec_setup_mem_size_base();
}

int dram_init_banksize(void)
{
	return fdtdec_setup_memory_banksize();
}

phys_addr_t board_get_usable_ram_top(phys_size_t total_size)
{
	return min_t(phys_addr_t, gd->ram_top, RC2_RAM_TOP);
}

#if CONFIG_IS_ENABLED(USB_GADGET_DWC2_OTG)
struct dwc2_udc;

#define GATE_HCLK_OTG0		BIT(29)		/* CLKGATE_CON5 bit 13 */
#define GATE_OTGPHY0		BIT(21)		/* CLKGATE_CON1 bit 5 */

/* Same sequence as boox-c67ml.c (rk30usbplug device-mode blob). */
void otg_phy_init(struct dwc2_udc *dev)
{
	writel(GATE_HCLK_OTG0, (void __iomem *)CRU_CLKGATE_CON5);
	writel(GATE_OTGPHY0, (void __iomem *)CRU_CLKGATE_CON1);
	/*
	 * Every gadget start (the ACM console at boot, ums, ACM again after
	 * ums): off the bus for 100 ms first, so the host sees a detach and
	 * enumerates the new gadget from scratch (vendor U-Boot: 500 ms; the
	 * core's own disconnect in reconfig_usbd() is 20 us).
	 */
	setbits_le32((void __iomem *)OTG_DCTL, OTG_DCTL_SFTDISCON);
	mdelay(100);
	writel(0x34000000, (void __iomem *)GRF_UOC1_CON4);
	writel(0x00e000e0, (void __iomem *)CRU_SOFTRST_CON4);
	udelay(3000);
	writel(0x00e00000, (void __iomem *)CRU_SOFTRST_CON4);
	udelay(3000);
}

/* pulse, never park the OTG blocks in reset (boox-c67ml.c otg_phy_off) */
void otg_phy_off(struct dwc2_udc *dev)
{
	writel(0x00e000e0, (void __iomem *)CRU_SOFTRST_CON4);
	udelay(3000);
	writel(0x00e00000, (void __iomem *)CRU_SOFTRST_CON4);
}

/*
 * Called by bootm right before the kernel jump: release the ACM gadget
 * (moving the consoles off usbacm unregisters it) and make sure the remap is
 * off, so a watchdog reset during the kernel boot does not restart in IMEM.
 */
void board_quiesce_devices(void)
{
	iomux_doenv(stdin, "serial");
	iomux_doenv(stdout, "serial");
	iomux_doenv(stderr, "serial");
	rc2_remap_off();
	rc2_led(false);
}
#endif

/* ---- reset ---------------------------------------------------------- */

extern char rc2_core_reset_start[], rc2_core_reset_end[];

/*
 * The stock 3.0 kernel's reset (rk30_arch_reset -> soft_reset in IMEM):
 * PLLs to slow mode, which makes DDR unusable, so the last part runs from
 * IMEM with the MMU and caches off.
 */
static void __noreturn rc2_core_reset(void)
{
	void (*fn)(void) = (void *)CORE_RESET_IMEM;
	size_t n = rc2_core_reset_end - rc2_core_reset_start;

	/* caller ran cleanup_before_linux(): IRQs, D-cache (cleaned), MMU, I-cache off */
	memcpy((void *)CORE_RESET_IMEM, rc2_core_reset_start, n);
	dsb();
	isb();
	fn();
	while (1)
		;
}

static int rc2_sysreset_request(struct udevice *dev, enum sysreset_t type)
{
	const char *m = env_get("rc2_reset");

	if (type == SYSRESET_POWER || type == SYSRESET_POWER_OFF)
		return -EPROTONOSUPPORT;
	if (!m || !*m)
		m = CONFIG_RC2_RESET_METHOD;

	printf("rc2: reset (%s)\n", m);
	mdelay(20);
	/*
	 * Leave the bus before the reset (soft disconnect: D+ pull-up off) and
	 * give the host time to see it, so the next boot's gadget enumerates
	 * as a new device whatever the reset method leaves of the OTG block.
	 * After a U-Boot fst reset the host never saw U-Boot's ACM port again
	 * (core and snd were fine; book round of 2026-10-17).
	 */
	setbits_le32((void __iomem *)OTG_DCTL, OTG_DCTL_SFTDISCON);
	mdelay(100);
	rc2_remap_off();
	/*
	 * Clean and switch off the caches first: the Cortex-A9 keeps its L1
	 * across a warm reset, and dirty lines written back later would land
	 * on whatever the next boot has loaded.
	 */
	cleanup_before_linux();

	if (!strcmp(m, "fst"))
		writel(0xfdb9, (void __iomem *)CRU_GLB_SRST_FST);
	else if (!strcmp(m, "snd"))
		writel(0xeca8, (void __iomem *)CRU_GLB_SRST_SND);
	else
		rc2_core_reset();

	return -EINPROGRESS;
}

static struct sysreset_ops rc2_sysreset_ops = {
	.request	= rc2_sysreset_request,
};

U_BOOT_DRIVER(rc2_sysreset) = {
	.name	= "rc2_sysreset",
	.id	= UCLASS_SYSRESET,
	.ops	= &rc2_sysreset_ops,
};
