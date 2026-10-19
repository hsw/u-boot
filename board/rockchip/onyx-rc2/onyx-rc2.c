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
 *  - no PMIC access (no I2C, no regulator driver); the mspl SPL's ACT8931
 *    voltage restore (spl.c) is only reported here, from its DDR record;
 *  - the IMEM remap is cleared at once and never set again: with it set,
 *    a reset restarts CPU0 in IMEM instead of the BootROM;
 *  - nothing above 0x7ff00000 is used by U-Boot (waveform window, ramoops,
 *    stage2 log and flag pages, bootcount).
 *
 * The OTG PHY bring-up and the gadget hand-off are Mario Rugiero's
 * (board/rockchip/boox-c67ml/boox-c67ml.c), same SoC.
 */

#include <command.h>
#include <console.h>
#include <cpu_func.h>
#include <dm.h>
#include <env.h>
#include <init.h>
#include <iomux.h>
#include <stdio.h>
#include <sysreset.h>
#include <time.h>
#include <watchdog.h>
#include <asm/global_data.h>
#include <asm/io.h>
#include <asm/system.h>
#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/usb/gadget.h>
#include <usb/dwc2_udc.h>
#include "act8931-restore.h"

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

/*
 * The SPL's ACT8931 restore record (act8931-restore.h, spl.c): one line per
 * raised selector, nothing when every selector was at or above its target.
 * Runs before relocation with the caches off, so the lines land in the
 * pre-console buffer and reach the ACM console with the banner. The record
 * stays for Linux, marked as seen. The stage2 chain has no SPL: a record
 * there is an older boot's and is dropped.
 */
static void rc2_act8931_report(void)
{
	static const char *const names[RC2_ACT_RAILS] = RC2_ACT_RAIL_NAMES;
	u32 *rec = (u32 *)RC2_ACT_REC;
	u32 st, w, err, at;
	int i;

	if (!IS_ENABLED(CONFIG_SPL)) {
		if (readl(&rec[0]) == RC2_ACT_MAGIC || readl(&rec[0]) == RC2_ACT_SEEN)
			writel(0, &rec[0]);
		return;
	}
	if (readl(&rec[0]) != RC2_ACT_MAGIC) {
		printf("act8931: no voltage restore record from the SPL\n");
		return;
	}
	st = readl(&rec[1]);
	err = st & 0xff;
	at = (st >> 12) & 0xf;
	for (i = 0; i < RC2_ACT_RAILS; i++) {
		w = readl(&rec[2 + i]);
		if (!(w & 0xff0000))
			continue;
		printf("act8931: %s%s 0x%02x -> 0x%02x%s\n", names[i],
		       (w & 0xff) < 0x50 && !(w & 1) ? " VSET1" : "",
		       (w >> 8) & 0xff, (w >> 16) & 0xff,
		       err && at == i ? " (not confirmed)" : "");
	}
	if (err == RC2_ACT_EBUS)
		printf("act8931: I2C bus held low, voltages not checked; booting on\n");
	else if (err == RC2_ACT_EXFER && at < RC2_ACT_RAILS)
		printf("act8931: no answer at %s, restore stopped; booting on\n", names[at]);
	else if (err == RC2_ACT_EVERIFY && at < RC2_ACT_RAILS)
		printf("act8931: %s reads 0x%02x after the write, restore stopped; booting on\n",
		       names[at], readl(&rec[2 + at]) >> 24);
	else if (err)
		printf("act8931: restore status %#x; booting on\n", st);
	writel(RC2_ACT_SEEN, &rec[0]);
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
	 * "resets" (SRST_WDT = 86 since the 16 * CON + bit renumbering), so
	 * the watchdog driver never pulses the reset; the board releases it.
	 * Nothing here enables the watchdog; "wdt start" in the boot script does.
	 */
	writel(BIT(15 + 16), (void __iomem *)CRU_CLKGATE_CON7);
	writel(BIT(6 + 16), (void __iomem *)CRU_SOFTRST_CON5);
	rc2_act8931_report();
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

/* ---- ums and USB diagnostics ---------------------------------------- */

/*
 * Two blocks in DDR that survive a warm reset, a watchdog reset and the
 * pinhole, readable from U-Boot (md.l) or a slot kernel (devmem); each
 * write is cleaned to DDR at once, so a hang loses nothing.
 *
 * 0x7fff6100, "rc2_ustat start|save" around ums, little endian words:
 *   0 magic 0x54534d55 ("UMST")   1 state: 1 ums running, 2 ums returned
 *   2 settings: bit 0 rc2_ums_dcache != off, bit 1 rc2_ums_wdt = 1
 *   3.. struct dwc2_udc_diag: in_compl, in_bytes, ahberr, timeout,
 *       intknepmis, txfundrn, bna, gahbcfg (8 words)
 *
 * 0x7fff6200-0x7fff6fff, USB event trace (usb_gadget_trace()), started by
 * "rc2_ustat start" and kept for that boot and the next one (the boot after
 * a watchdog reset or pinhole), then off until the next start. Header of 16
 * words, then RC2_UTRACE_CAP entries of 4 words (event | aux << 8, ms since
 * boot, a, b; USB_TRACE_* in linux/usb/gadget.h, board events below); when
 * full, later events only count. The ACM data path is counted, and logged
 * only for the first few after each SET_INTERFACE/set_alt or on an error.
 */
#define RC2_USTAT		0x7fff6100
#define RC2_USTAT_MAGIC		0x54534d55
#define RC2_USTAT_WORDS		(3 + 8)
#define RC2_UTRACE		0x7fff6200
#define RC2_UTRACE_MAGIC	0x43525455	/* "UTRC" */
#define RC2_UTRACE_END		0x7fff7000
#define RC2_UTRACE_HDR		16
#define RC2_UTRACE_CAP		((RC2_UTRACE_END - RC2_UTRACE - 4 * RC2_UTRACE_HDR) / 16)
/* entries 0..HEAD-1: the first events; HEAD..CAP-1: a ring of the latest ones */
#define RC2_UTRACE_HEAD		100
#define RC2_UTRACE_CAPWORD	(RC2_UTRACE_CAP | RC2_UTRACE_HEAD << 16)

enum {	/* trace header words */
	UT_MAGIC, UT_N, UT_CAP, UT_BOOTS,	/* events offered; capacity; boots left */
	UT_TXQ, UT_TXDONE, UT_TXERR, UT_RXDONE, UT_RXBYTES, UT_RXERR,
	UT_LINESTATE,				/* last SET_CONTROL_LINE_STATE */
	UT_HEARTBEAT,				/* rc2_acmwait: +1 every 100 ms */
	UT_LAST_MS,				/* time of the last event */
	UT_USBRST,				/* bus resets seen */
	UT_ENUM,				/* ENUMDONE: high speed | other << 16 */
	UT_SETUPS,				/* SETUP packets */
};

enum {	/* board events */
	RC2_TRACE_BOOT = USB_TRACE_BOARD,	/* first event of a later boot; a: bootcount */
	RC2_TRACE_USTAT_START,			/* a: settings */
	RC2_TRACE_USTAT_SAVE,			/* a: in_compl, b: in_bytes */
	RC2_TRACE_ACMWAIT,			/* a: seconds */
	RC2_TRACE_ACMWAIT_END,			/* a: 1 key, 0 timeout; b: ms */
};

/* 0 not decided yet in this boot, 1 recording, -1 off */
static int rc2_utrace_state;
static unsigned int rc2_utrace_burst;

static void rc2_clean(ulong a, ulong len)
{
	ulong s = a & ~(ulong)(CONFIG_SYS_CACHELINE_SIZE - 1);

	flush_dcache_range(s, ALIGN(a + len, CONFIG_SYS_CACHELINE_SIZE));
}

static void rc2_utrace_log(u32 *h, unsigned int ev, u32 a, u32 b)
{
	u32 n = h[UT_N]++;
	u32 slot = n < RC2_UTRACE_HEAD ? n : RC2_UTRACE_HEAD +
		   (n - RC2_UTRACE_HEAD) % (RC2_UTRACE_CAP - RC2_UTRACE_HEAD);
	u32 *e = (u32 *)(RC2_UTRACE + 4 * RC2_UTRACE_HDR) + 4 * slot;

	e[0] = ev;
	e[1] = get_timer(0);
	e[2] = a;
	e[3] = b;
	rc2_clean((ulong)e, 16);
}

void usb_gadget_trace(unsigned int ev, u32 a, u32 b)
{
	u32 *h = (u32 *)RC2_UTRACE;
	bool log = true;

	if (!rc2_utrace_state) {
		rc2_utrace_state = -1;
		if (h[UT_MAGIC] == RC2_UTRACE_MAGIC && h[UT_BOOTS] &&
		    h[UT_CAP] == RC2_UTRACE_CAPWORD) {
			h[UT_BOOTS]--;
			rc2_utrace_state = 1;
			rc2_utrace_log(h, RC2_TRACE_BOOT, *(u32 *)0x7fff6000, 0);
		}
	}
	if (rc2_utrace_state < 0)
		return;

	switch (ev & 0xff) {
	case USB_TRACE_USBRST:
		h[UT_USBRST]++;
		break;
	case USB_TRACE_ENUMDONE:
		h[UT_ENUM] += a ? 0x10000 : 1;	/* DSTS.EnumSpd 0 = high speed */
		break;
	case USB_TRACE_SETUP:
		h[UT_SETUPS]++;
		break;
	case USB_TRACE_ACM_SET_ALT:
		rc2_utrace_burst = 0;
		break;
	case USB_TRACE_ACM_LINE_STATE:
		h[UT_LINESTATE] = a;
		break;
	case USB_TRACE_ACM_TX_QUEUE:
		h[UT_TXQ]++;
		h[UT_TXERR] += !!b;
		log = b || rc2_utrace_burst++ < 4;
		break;
	case USB_TRACE_ACM_TX_DONE:
		h[UT_TXDONE]++;
		h[UT_TXERR] += !!a;
		log = a || rc2_utrace_burst++ < 4;
		break;
	case USB_TRACE_ACM_RX_DONE:
		h[UT_RXDONE]++;
		h[UT_RXBYTES] += b;
		h[UT_RXERR] += !!a;
		log = a || rc2_utrace_burst++ < 4;
		break;
	}
	if (log)
		rc2_utrace_log(h, ev, a, b);
	h[UT_LAST_MS] = get_timer(0);
	rc2_clean((ulong)h, 4 * RC2_UTRACE_HDR);
}

static u32 rc2_ustat_cfg;

static void rc2_ustat_write(u32 state)
{
	u32 *w = (u32 *)RC2_USTAT;

	w[0] = RC2_USTAT_MAGIC;
	w[1] = state;
	w[2] = rc2_ustat_cfg;
	memcpy(&w[3], &dwc2_udc_diag, 8 * sizeof(u32));
	rc2_clean(RC2_USTAT, 4 * RC2_USTAT_WORDS);
}

static int do_rc2_ustat(struct cmd_tbl *cmdtp, int flag, int argc, char *const argv[])
{
	struct dwc2_udc_diag *d = &dwc2_udc_diag;
	const char *dc = env_get("rc2_ums_dcache");
	u32 *h = (u32 *)RC2_UTRACE;

	if (argc != 2)
		return CMD_RET_USAGE;
	if (!strcmp(argv[1], "start")) {
		memset(d, 0, sizeof(*d));
		rc2_ustat_cfg = (dc && !strcmp(dc, "off") ? 0 : 1) |
				(env_get_yesno("rc2_ums_wdt") == 1) << 1;
		rc2_ustat_write(1);
		memset(h, 0, 4 * RC2_UTRACE_HDR);
		h[UT_MAGIC] = RC2_UTRACE_MAGIC;
		h[UT_CAP] = RC2_UTRACE_CAPWORD;
		h[UT_BOOTS] = 1;
		rc2_utrace_state = 1;
		usb_gadget_trace(RC2_TRACE_USTAT_START, rc2_ustat_cfg, 0);
		return 0;
	}
	if (!strcmp(argv[1], "save")) {
		rc2_ustat_write(2);
		usb_gadget_trace(RC2_TRACE_USTAT_SAVE, d->in_compl, d->in_bytes);
		printf("rc2: ums stats (settings %#x): IN compl %u bytes %u; ahberr %u timeout %u "
		       "intknepmis %u txfundrn %u bna %u; gahbcfg %#x\n", rc2_ustat_cfg,
		       d->in_compl, d->in_bytes, d->ahberr, d->timeout, d->intknepmis,
		       d->txfundrn, d->bna, d->gahbcfg);
		return 0;
	}
	return CMD_RET_USAGE;
}

U_BOOT_CMD(rc2_ustat, 2, 0, do_rc2_ustat,
	   "ums diagnostics: counters to DDR 0x7fff6100, USB trace at 0x7fff6200",
	   "start - clear the counters, start the USB trace (this boot and the next)\n"
	   "rc2_ustat save - write the counters to DDR and print them");

/*
 * "rc2_acmwait SECS": wait up to SECS seconds for a key on the console (the
 * ACM port after ums): 0 = a key came (or SECS is 0), 1 = none. While it
 * waits, U-Boot services the gadget and feeds a running watchdog.
 */
static int do_rc2_acmwait(struct cmd_tbl *cmdtp, int flag, int argc, char *const argv[])
{
	ulong secs = argc > 1 ? dectoul(argv[1], NULL) : 0;
	ulong start = get_timer(0), beat = start;
	u32 *h = (u32 *)RC2_UTRACE;

	if (!secs)
		return 0;
	printf("rc2: press a key on the console within %lu s (else: run rc2_ums_noacm)\n", secs);
	usb_gadget_trace(RC2_TRACE_ACMWAIT, secs, 0);
	while (get_timer(start) < secs * 1000) {
		if (tstc()) {
			getchar();
			usb_gadget_trace(RC2_TRACE_ACMWAIT_END, 1, get_timer(start));
			puts("rc2: console ok\n");
			return 0;
		}
		if (get_timer(beat) >= 100 && rc2_utrace_state > 0) {
			beat = get_timer(0);
			h[UT_HEARTBEAT]++;
			rc2_clean((ulong)h, 4 * RC2_UTRACE_HDR);
		}
		schedule();
	}
	usb_gadget_trace(RC2_TRACE_ACMWAIT_END, 0, get_timer(start));
	puts("rc2: no key on the console\n");
	return 1;
}

U_BOOT_CMD(rc2_acmwait, 2, 0, do_rc2_acmwait,
	   "wait for a key on the console",
	   "SECS - 0: a key came within SECS seconds (or SECS is 0), 1: none");
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
