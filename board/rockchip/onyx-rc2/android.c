// SPDX-License-Identifier: GPL-2.0+
/*
 * Boot the stock Rockchip 3.0 kernel of the ONYX RC2 from its eMMC the way
 * the stock loader (FlashBoot) does, from a U-Boot that the stock loader
 * started in place of that kernel.
 *
 * The stock loader's contract, as the kernel sees it:
 *  - the "parameter" block at eMMC user sector 0x2000 ("PARM", u32 length,
 *    text, u32 CRC; eight copies 0x400 sectors apart) names the machine
 *    number (MACHINE), the ATAG address (ATAG), the kernel address
 *    (KERNEL_IMG) and the command line (CMDLINE), whose mtdparts= gives
 *    the partitions in sectors relative to user sector 0x2000;
 *  - "kernel" and "boot" hold Rockchip KRNL images ("KRNL", u32 size,
 *    payload, u32 CRC): the kernel Image goes to KERNEL_IMG, the boot
 *    ramdisk to the initrd= address of the command line, whose size field
 *    the loader rewrites to (size + 0x3ffff) & ~0xffff;
 *  - ATAG_CORE (flags, page size, root device all 0) + ATAG_CMDLINE +
 *    ATAG_NONE at ATAG; r0 = 0, r1 = MACHINE, r2 = ATAG; SVC mode, MMU and
 *    caches off; GRF_SOC_CON0 bit 12 set (IMEM at address 0, which the
 *    kernel's CPU1 bring-up uses); CPU1 held in soft reset;
 *  - the loader's eMMC hand-off block in the NAND controller SRAM at
 *    0x10501000: tag "SYSD", boot medium (word 2) and the partition offset
 *    0x2000 (word 5). The stock flash driver takes the partition offset
 *    from there; without the block it would address the partitions 4 MiB
 *    too low. This command never writes that block: it refuses to start
 *    the kernel when the block is not the loader's.
 *
 * Kernel source: the "kernel" partition. When this U-Boot itself is that
 * partition's payload, the kernel Image is appended to it in a bundle: a
 * 512-byte header at a 64 KiB boundary ("RC2KBNDL", u32 version 1, u32
 * size, u32 CRC of the image, u32 load address, u32 CRC of the 24 bytes
 * before it), the Image right after. CRCs are the Rockchip CRC32 (poly
 * 0x04c10db7, MSB first, initial value 0).
 *
 * The eMMC is only read (blk_dread on mmc 1, user area).
 */

#include <blk.h>
#include <blkmap.h>
#include <bootm.h>
#include <command.h>
#include <cpu_func.h>
#include <dm/device.h>
#include <env.h>
#include <hang.h>
#include <malloc.h>
#include <mmc.h>
#include <stdio.h>
#include <vsprintf.h>
#include <asm/global_data.h>
#include <asm/io.h>
#include <asm/system.h>
#include <asm/unaligned.h>
#include <linux/bitops.h>
#include <linux/ctype.h>
#include <linux/delay.h>
#include <linux/sizes.h>
#include <linux/string.h>
#include "android.h"

DECLARE_GLOBAL_DATA_PTR;

#define RC2A_PARM_LBA		0x2000
#define RC2A_PARM_COPIES	8
#define RC2A_PARM_STEP		0x400
#define RC2A_PARM_MAX		0x4000		/* bytes of parameter text */
#define RC2A_PART_BASE		0x2000		/* mtdparts sector 0 = user sector 0x2000 */

#define RC2A_TAG_PARM		0x4d524150	/* "PARM" */
#define RC2A_TAG_KRNL		0x4c4e524b	/* "KRNL" */
#define RC2A_TAG_SYSD		0x44535953	/* "SYSD" */

#define RC2A_SCRATCH		0x61000000	/* kernel_addr_r: the "kernel" payload */
#define RC2A_KERNEL_MAX		0x00af8000	/* up to the pre-console buffer 0x60f00000 */
#define RC2A_INITRD_MAX		0x01000000
#define RC2A_BUNDLE_ALIGN	0x10000
#define RC2A_BUNDLE_SCAN	0x00200000
#define RC2A_CMDLINE_MAX	2048

#define ATAG_NONE		0x00000000
#define ATAG_CORE		0x54410001
#define ATAG_CMDLINE		0x54410009

#define NANDC_SRAM		0x10501000
#define GRF_SOC_CON0		0x20008140
#define CRU_SOFTRST_CON0	0x20000110
#define WDT_CR			0x2004c000
#define CRU_CLKGATE_CON8	0x200000f0	/* bits 9..12: pclk_gpio0..3, 1 = gated */
#define GRF_IOMUX		0x200080a8	/* GPIO0A..GPIO3D, 2 bits a pad */
#define GRF_PULL		0x20008118	/* 16 pads a register, 1 = pull off */
#define GPIO0_BASE		0x2007c000
#define GPIO1_BASE		0x20080000
#define GPIO2_BASE		0x20084000
#define GPIO3_BASE		0x20088000
#define GPIO_INTEN		0x30

static u32 rkcrc_table[256];

static u32 rkcrc(const void *data, ulong len)
{
	const u8 *p = data;
	u32 c = 0;
	int i, j;

	if (!rkcrc_table[1]) {
		for (i = 0; i < 256; i++) {
			c = (u32)i << 24;
			for (j = 0; j < 8; j++)
				c = c & BIT(31) ? (c << 1) ^ 0x04c10db7 : c << 1;
			rkcrc_table[i] = c;
		}
		c = 0;
	}
	while (len--)
		c = (c << 8) ^ rkcrc_table[((c >> 24) ^ *p++) & 0xff];

	return c;
}

struct rc2a {
	int dev;			/* mmc device number of the eMMC */
	struct blk_desc *desc;
	char *parm;			/* parameter text, NUL terminated */
	ulong machine, atag, kernel_img;
	char cmdline[RC2A_CMDLINE_MAX];
	ulong initrd_addr;
	ulong kernel_size, initrd_size;
};

static int rc2a_read(struct rc2a *a, lbaint_t lba, lbaint_t n, void *dst)
{
	if (blk_dread(a->desc, lba, n, dst) != n) {
		printf("rc2android: eMMC read of %#lx sectors at %#lx failed\n",
		       (ulong)n, (ulong)lba);
		return -EIO;
	}
	return 0;
}

/*
 * The loader reads the parameter text line by line up to '\n' and drops
 * every '\r' on the way, so a '\r' inside a line does not end it: the
 * stock CMDLINE line is "...(user)\r pcb_ver=2 onyx_emmc=1\n", and the
 * kernel gets " pcb_ver=2 onyx_emmc=1" as part of its command line. Copy
 * the text the same way.
 */
static void rc2a_unix_lines(char *dst, const u8 *src, ulong len)
{
	while (len--) {
		if (*src != '\r')
			*dst++ = *src;
		src++;
	}
	*dst = '\0';
}

static int rc2a_parameter(struct rc2a *a)
{
	ulong secs = DIV_ROUND_UP(8 + RC2A_PARM_MAX + 4, 512);
	u8 *buf = malloc(secs * 512);
	int k;

	if (!buf)
		return -ENOMEM;
	for (k = 0; k < RC2A_PARM_COPIES; k++) {
		u32 len;

		if (rc2a_read(a, RC2A_PARM_LBA + k * RC2A_PARM_STEP, secs, buf))
			continue;
		len = get_unaligned_le32(buf + 4);
		if (get_unaligned_le32(buf) != RC2A_TAG_PARM || len > RC2A_PARM_MAX ||
		    get_unaligned_le32(buf + 8 + len) != rkcrc(buf + 8, len)) {
			printf("rc2android: parameter copy %d invalid\n", k);
			continue;
		}
		a->parm = malloc(len + 1);
		if (!a->parm)
			break;
		rc2a_unix_lines(a->parm, buf + 8, len);
		free(buf);
		return 0;
	}
	free(buf);
	printf("rc2android: no valid parameter block\n");
	return -ENOENT;
}

/* value of "KEY:" at the start of a line of the parameter text, or NULL */
static const char *rc2a_key(struct rc2a *a, const char *key, int *len)
{
	const char *p = a->parm;
	size_t kl = strlen(key);

	while (p && *p) {
		const char *eol = strchr(p, '\n');
		int n = eol ? eol - p : strlen(p);

		if (n > kl && !strncmp(p, key, kl) && p[kl] == ':') {
			const char *v = p + kl + 1;

			while (*v == ' ' || *v == '\t')
				v++;
			*len = p + n - v;
			return v;
		}
		p = eol ? eol + 1 : NULL;
	}
	return NULL;
}

static int rc2a_num(struct rc2a *a, const char *key, ulong *val)
{
	int len;
	const char *v = rc2a_key(a, key, &len);

	if (!v) {
		printf("rc2android: parameter has no %s\n", key);
		return -ENOENT;
	}
	*val = simple_strtoul(v, NULL, 0);
	return 0;
}

/* one entry of the parameter's mtdparts= list */
struct rc2a_mtdpart {
	const char *name;		/* not NUL terminated */
	int name_len;
	lbaint_t off, size;		/* user-area sectors */
	bool rest;			/* "-": up to the end of the device */
};

/* the first entry of mtdparts= in the command line, or NULL */
static const char *rc2a_mtdparts(struct rc2a *a)
{
	const char *p = strstr(a->cmdline, "mtdparts=");

	if (p)
		p = strchr(p, ':');
	return p ? p + 1 : NULL;
}

/*
 * Parse the entry at *@pp ("SIZE@OFFSET(NAME)" or "-@OFFSET(NAME)", sectors
 * relative to RC2A_PART_BASE) and step past it: 0, -ENOENT at the end of
 * the list, -EINVAL on a malformed entry.
 */
static int rc2a_mtdpart_next(struct rc2a *a, const char **pp, struct rc2a_mtdpart *mp)
{
	const char *p = *pp;
	char *e;
	ulong sz = 0;

	if (!p || !*p || *p == ' ')
		return -ENOENT;
	if (*p == '-')
		e = (char *)p + 1;
	else
		sz = simple_strtoul(p, &e, 0);
	if (*e != '@')
		return -EINVAL;
	mp->off = RC2A_PART_BASE + simple_strtoul(e + 1, &e, 0);
	if (*e != '(')
		return -EINVAL;
	mp->name = e + 1;
	e = strchr(mp->name, ')');
	if (!e)
		return -EINVAL;
	mp->name_len = e - mp->name;
	mp->rest = !sz;
	mp->size = sz ? sz : a->desc->lba - mp->off;
	if (*++e == ',')
		e++;
	*pp = e;
	return 0;
}

/* offset and size (sectors, user area) of a partition of mtdparts= */
static int rc2a_part(struct rc2a *a, const char *name, lbaint_t *off, lbaint_t *size)
{
	const char *p = rc2a_mtdparts(a);
	struct rc2a_mtdpart mp;
	int ret;

	if (!p)
		return -ENOENT;
	while (!(ret = rc2a_mtdpart_next(a, &p, &mp))) {
		if (mp.name_len == strlen(name) && !strncmp(mp.name, name, mp.name_len)) {
			*off = mp.off;
			*size = mp.size;
			return 0;
		}
	}
	if (ret != -ENOENT)
		return ret;
	printf("rc2android: no partition \"%s\" in mtdparts\n", name);
	return -ENOENT;
}

/*
 * A KRNL image of partition @name to @dst (header and CRC around it, so
 * dst - 8 .. dst + size + 4 rounded up to a sector are written); returns
 * the payload size in @size.
 */
static int rc2a_krnl(struct rc2a *a, const char *name, ulong dst, ulong max, ulong *size)
{
	lbaint_t off, psize, n;
	u8 *hdr = (u8 *)dst - 8;
	ulong len;
	int ret;

	ret = rc2a_part(a, name, &off, &psize);
	if (ret)
		return ret;
	ret = rc2a_read(a, off, 1, hdr);
	if (ret)
		return ret;
	len = get_unaligned_le32(hdr + 4);
	if (get_unaligned_le32(hdr) != RC2A_TAG_KRNL) {
		printf("rc2android: %s: no KRNL image\n", name);
		return -EINVAL;
	}
	n = DIV_ROUND_UP(8 + len + 4, 512);
	if (len > max || n > psize) {
		printf("rc2android: %s: KRNL size %#lx too large\n", name, len);
		return -EFBIG;
	}
	ret = rc2a_read(a, off, n, hdr);
	if (ret)
		return ret;
	if (get_unaligned_le32((u8 *)dst + len) != rkcrc((void *)dst, len)) {
		printf("rc2android: %s: KRNL CRC mismatch\n", name);
		return -EBADMSG;
	}
	*size = len;
	return 0;
}

static int rc2a_kernel(struct rc2a *a)
{
	ulong len, off;
	u8 *p = (u8 *)RC2A_SCRATCH;
	int ret;

	ret = rc2a_krnl(a, "kernel", RC2A_SCRATCH, RC2A_INITRD_MAX, &len);
	if (ret)
		return ret;

	for (off = RC2A_BUNDLE_ALIGN; off + 512 <= min(len, (ulong)RC2A_BUNDLE_SCAN);
	     off += RC2A_BUNDLE_ALIGN) {
		u8 *h = p + off;
		ulong isz;

		if (memcmp(h, "RC2KBNDL", 8) || get_unaligned_le32(h + 24) != rkcrc(h, 24))
			continue;
		isz = get_unaligned_le32(h + 12);
		if (get_unaligned_le32(h + 8) != 1 || off + 512 + isz > len ||
		    isz > RC2A_KERNEL_MAX || get_unaligned_le32(h + 20) != a->kernel_img) {
			printf("rc2android: kernel bundle header at %#lx invalid\n", off);
			return -EINVAL;
		}
		if (get_unaligned_le32(h + 16) != rkcrc(h + 512, isz)) {
			printf("rc2android: kernel bundle CRC mismatch\n");
			return -EBADMSG;
		}
		memmove((void *)a->kernel_img, h + 512, isz);
		a->kernel_size = isz;
		printf("rc2android: kernel: bundle in \"kernel\" at %#lx, %#lx bytes\n", off, isz);
		return 0;
	}

	/* the partition holds this U-Boot without a kernel: do not start it again */
	if (!memcmp(p, (void *)gd->relocaddr, 32)) {
		printf("rc2android: \"kernel\" holds this U-Boot and no kernel bundle\n");
		return -ENOENT;
	}
	if (len > RC2A_KERNEL_MAX) {
		printf("rc2android: kernel too large (%#lx)\n", len);
		return -EFBIG;
	}
	memmove((void *)a->kernel_img, p, len);
	a->kernel_size = len;
	printf("rc2android: kernel: \"kernel\" KRNL, %#lx bytes\n", len);
	return 0;
}

/* "initrd=0xADDR,0xSIZE": the address, and the size field rewritten as the loader does */
static int rc2a_initrd(struct rc2a *a, bool rewrite)
{
	char *p = strstr(a->cmdline, "initrd=0x"), *c, *e;
	char hex[11];

	if (!p) {
		printf("rc2android: no initrd= in CMDLINE\n");
		return -ENOENT;
	}
	a->initrd_addr = simple_strtoul(p + 7, &c, 16);
	if (*c != ',' || c[1] != '0' || c[2] != 'x')
		return -EINVAL;
	if (!rewrite)
		return 0;
	c += 3;
	for (e = c; isxdigit(*e); e++)
		;
	if (e - c != 8) {
		printf("rc2android: initrd size field is not 8 digits\n");
		return -EINVAL;
	}
	snprintf(hex, sizeof(hex), "%08lX", (a->initrd_size + 0x3ffff) & ~0xffffUL);
	memcpy(c, hex, 8);
	return 0;
}

static int rc2a_cmdline(struct rc2a *a)
{
	const char *v, *extra;
	int len, fl = 0;
	const char *fw = rc2a_key(a, "FIRMWARE_VER", &fl);
	const char *bootver = env_get("rc2a_bootver");

	v = rc2a_key(a, "CMDLINE", &len);
	if (!v || len >= RC2A_CMDLINE_MAX - 128) {
		printf("rc2android: no usable CMDLINE in the parameter\n");
		return -ENOENT;
	}
	memcpy(a->cmdline, v, len);
	a->cmdline[len] = '\0';
	/* the loader appends these two; nothing in the kernel reads them */
	snprintf(a->cmdline + len, sizeof(a->cmdline) - len, " bootver=%s firmware_ver=%.*s",
		 bootver ? bootver : "2016-04-12#2.32", fw ? min(fl, 32) : 0, fw ? fw : "");
	extra = env_get("rc2a_args");
	if (extra && strlen(a->cmdline) + 1 + strlen(extra) < sizeof(a->cmdline)) {
		strcat(a->cmdline, " ");
		strcat(a->cmdline, extra);
	}
	return 0;
}

static ulong rc2a_atags(struct rc2a *a)
{
	u32 *t = (u32 *)a->atag;
	size_t n = strlen(a->cmdline);

	memset(t, 0, 5 * 4 + 8 + n + 4 + 8);
	t[0] = 5;
	t[1] = ATAG_CORE;
	t[2] = 0;
	t[3] = 0;
	t[4] = 0;
	t += 5;
	t[0] = (n + 13) >> 2;
	t[1] = ATAG_CMDLINE;
	memcpy(&t[2], a->cmdline, n + 1);
	t += t[0];
	t[0] = 0;
	t[1] = ATAG_NONE;
	return (ulong)(t + 2) - a->atag;
}

/* the stock loader's hand-off block for the stock flash driver */
static int rc2a_sram_check(void)
{
	u32 tag = readl(NANDC_SRAM), media = readl(NANDC_SRAM + 8), off = readl(NANDC_SRAM + 0x14);

	printf("rc2android: NANDC SRAM: tag %08x media %u offset %#x\n", tag, media, off);
	if (tag != RC2A_TAG_SYSD || media != 2 || off != RC2A_PART_BASE) {
		printf("rc2android: no stock-loader hand-off block in the NANDC SRAM: the stock\n"
		       "            flash driver would address the partitions 4 MiB too low\n");
		return -ENODEV;
	}
	return 0;
}

/*
 * Pads as the loader left them. The stock kernel was only ever started by
 * the loader, so its pad setup may rely on what the loader leaves: record
 * the pad mux, pull and GPIO data/direction at U-Boot's entry, show in
 * "rc2android -n" what U-Boot changed since, and put it all back right
 * before the jump. GPIO interrupts are only shown: no GPIO driver is
 * built and the board code never enables one.
 *
 * Kept:
 *  - GPIO1_A2, the power hold. U-Boot drives it high as the loader does;
 *    the loader picks that pin or GPIO1_A1 from board straps, and the
 *    hold must not drop whichever it drove.
 * Put back with the rest:
 *  - the eMMC pads (GPIO1_C6/C7, GPIO1_D0..D7, GPIO2_A5, GPIO2_A7): the
 *    loader read the eMMC through them as well, and the stock flash
 *    driver has only ever started from the loader's setup.
 *
 * Taken first thing in board_early_init_f(), before any driver probes:
 * pinctrl applies a device's "default" state when the device probes, and
 * initf_dm() before it only binds (nothing in this build is probed after
 * bind). The record is in .data: written before relocation, it moves
 * with the image.
 */
enum { RC2A_SHOW, RC2A_PULL, RC2A_GPIO, RC2A_MUX };	/* restore order */

static const struct rc2a_pad_grp {
	u32 base;
	u8 n;		/* registers, 4 bytes apart */
	u8 kind;
	s8 bank;	/* GPIO bank whose pclk the registers need, or -1 */
	const char *name;
} rc2a_pad_grps[] = {
	{ CRU_CLKGATE_CON8,		1,	RC2A_SHOW,	-1, "cru gate8" },
	{ GRF_PULL,			8,	RC2A_PULL,	-1, "grf pull" },
	{ GPIO0_BASE,			2,	RC2A_GPIO,	0, "gpio0 dr/ddr" },
	{ GPIO1_BASE,			2,	RC2A_GPIO,	1, "gpio1 dr/ddr" },
	{ GPIO2_BASE,			2,	RC2A_GPIO,	2, "gpio2 dr/ddr" },
	{ GPIO3_BASE,			2,	RC2A_GPIO,	3, "gpio3 dr/ddr" },
	{ GPIO0_BASE + GPIO_INTEN,	1,	RC2A_SHOW,	0, "gpio0 inten" },
	{ GPIO1_BASE + GPIO_INTEN,	1,	RC2A_SHOW,	1, "gpio1 inten" },
	{ GPIO2_BASE + GPIO_INTEN,	1,	RC2A_SHOW,	2, "gpio2 inten" },
	{ GPIO3_BASE + GPIO_INTEN,	1,	RC2A_SHOW,	3, "gpio3 inten" },
	{ GRF_IOMUX,			16,	RC2A_MUX,	-1, "grf iomux" },
};

#define RC2A_PAD_REGS		(1 + 8 + 4 * 2 + 4 + 16)	/* the n above */
#define RC2A_PADS_MAGIC		0x53444150	/* "PADS" */

static struct {
	u32 magic;
	u32 unrecorded;		/* groups whose bank clock was gated at entry */
	u32 val[RC2A_PAD_REGS];
} rc2a_pads __section(".data");

static u32 rc2a_rd(u32 addr)
{
	return readl((void __iomem *)(ulong)addr);
}

static bool rc2a_pad_readable(const struct rc2a_pad_grp *g, u32 gate8)
{
	return g->bank < 0 || !(gate8 & BIT(9 + g->bank));
}

/* bits of a register the restore leaves as they are now: the power hold */
static u32 rc2a_pad_keep(u32 addr)
{
	if (addr == GRF_IOMUX + 4 * 4)				/* GPIO1A */
		return GENMASK(5, 4);				/* A2 */
	if (addr == GPIO1_BASE || addr == GPIO1_BASE + 4)	/* DR, DDR */
		return BIT(2);
	return 0;
}

void rc2a_pads_save(void)
{
	u32 gate8 = rc2a_rd(CRU_CLKGATE_CON8);
	int g, i, k = 0;

	for (g = 0; g < ARRAY_SIZE(rc2a_pad_grps); g++) {
		const struct rc2a_pad_grp *grp = &rc2a_pad_grps[g];

		for (i = 0; i < grp->n; i++, k++) {
			if (rc2a_pad_readable(grp, gate8))
				rc2a_pads.val[k] = rc2a_rd(grp->base + 4 * i);
			else
				rc2a_pads.unrecorded |= BIT(g);
		}
	}
	rc2a_pads.magic = RC2A_PADS_MAGIC;
}

static void rc2a_pads_report(void)
{
	u32 gate8 = rc2a_rd(CRU_CLKGATE_CON8);
	int g, i, k = 0, n = 0;

	if (rc2a_pads.magic != RC2A_PADS_MAGIC) {
		printf("rc2android: no pad record from U-Boot's entry\n");
		return;
	}
	for (g = 0; g < ARRAY_SIZE(rc2a_pad_grps); g++) {
		const struct rc2a_pad_grp *grp = &rc2a_pad_grps[g];

		if (rc2a_pads.unrecorded & BIT(g) || !rc2a_pad_readable(grp, gate8)) {
			printf("rc2android: %s: clock gated %s, not compared\n", grp->name,
			       rc2a_pads.unrecorded & BIT(g) ? "at entry" : "now");
			k += grp->n;
			continue;
		}
		for (i = 0; i < grp->n; i++, k++) {
			u32 addr = grp->base + 4 * i;
			u32 now = rc2a_rd(addr);

			if (now == rc2a_pads.val[k])
				continue;
			printf("rc2android: pad reg %08x %-12s entry %08x now %08x%s\n",
			       addr, grp->name, rc2a_pads.val[k], now,
			       grp->kind == RC2A_SHOW ? " (not restored)" : "");
			n++;
		}
	}
	printf("rc2android: %d pad register(s) changed since entry; mux, pull, GPIO\n"
	       "            data and direction go back before the jump\n", n);
}

static void rc2a_pads_restore(void)
{
	u32 gate8 = rc2a_rd(CRU_CLKGATE_CON8);
	int kind, g, i, k;

	if (rc2a_pads.magic != RC2A_PADS_MAGIC)
		return;
	/* pulls, then GPIO data before direction, then the mux */
	for (kind = RC2A_PULL; kind <= RC2A_MUX; kind++) {
		for (g = 0, k = 0; g < ARRAY_SIZE(rc2a_pad_grps); k += rc2a_pad_grps[g++].n) {
			const struct rc2a_pad_grp *grp = &rc2a_pad_grps[g];

			if (grp->kind != kind || rc2a_pads.unrecorded & BIT(g) ||
			    !rc2a_pad_readable(grp, gate8))
				continue;
			for (i = 0; i < grp->n; i++) {
				u32 addr = grp->base + 4 * i;
				u32 keep = rc2a_pad_keep(addr), val = rc2a_pads.val[k + i] & ~keep;

				if (kind == RC2A_GPIO)		/* plain register */
					val |= rc2a_rd(addr) & keep;
				else				/* GRF: write mask in [31:16] */
					val = (~keep & 0xffff) << 16 | (val & 0xffff);
				writel(val, (void __iomem *)(ulong)addr);
			}
		}
	}
}

static void __noreturn rc2a_jump(struct rc2a *a)
{
	void (*entry)(int, int, uint) = (void *)a->kernel_img;

	bootm_final(0);
	cleanup_before_linux();
	rc2a_pads_restore();
	/* as the stock loader leaves them: CPU1 in soft reset, IMEM at address 0 */
	writel(0x00100010, (void __iomem *)CRU_SOFTRST_CON0);
	writel(BIT(12 + 16) | BIT(12), (void __iomem *)GRF_SOC_CON0);
	dsb();
	isb();
	entry(0, a->machine, a->atag);
	hang();
}

/* the eMMC (env rc2a_mmc, default 1) on its user area */
static int rc2a_open(struct rc2a *a)
{
	struct mmc *mmc;

	a->dev = env_get_ulong("rc2a_mmc", 10, 1);
	mmc = find_mmc_device(a->dev);
	if (!mmc || mmc_init(mmc)) {
		printf("rc2android: no eMMC at mmc %d\n", a->dev);
		return -ENODEV;
	}
	a->desc = mmc_get_blk_desc(mmc);
	if (!a->desc || a->desc->hwpart) {
		printf("rc2android: mmc %d is not on its user area\n", a->dev);
		return -ENODEV;
	}
	return 0;
}

static int do_rc2android(struct cmd_tbl *cmdtp, int flag, int argc, char *const argv[])
{
	struct rc2a a = { };
	bool dry = argc > 1 && !strcmp(argv[1], "-n");
	int ret = CMD_RET_FAILURE;

	if (rc2a_open(&a))
		return CMD_RET_FAILURE;
	if (rc2a_parameter(&a) || rc2a_num(&a, "MACHINE", &a.machine) ||
	    rc2a_num(&a, "ATAG", &a.atag) || rc2a_num(&a, "KERNEL_IMG", &a.kernel_img) ||
	    rc2a_cmdline(&a) || rc2a_initrd(&a, false))
		goto out;
	/*
	 * Layout this command relies on (the stock parameter: 0x60000800,
	 * 0x60408000, 0x62000000): ATAGs in the first MiB (the 3.0 kernel maps
	 * only that much early), the kernel below the pre-console buffer, the
	 * "kernel" payload at RC2A_SCRATCH, the ramdisk above it and far below
	 * U-Boot's relocated copy.
	 */
	if (a.atag < CFG_SYS_SDRAM_BASE ||
	    a.atag + RC2A_CMDLINE_MAX + 64 > CFG_SYS_SDRAM_BASE + SZ_1M ||
	    a.kernel_img != 0x60408000 || a.initrd_addr < RC2A_SCRATCH + RC2A_INITRD_MAX ||
	    a.initrd_addr + RC2A_INITRD_MAX > CFG_SYS_SDRAM_BASE + SZ_256M) {
		printf("rc2android: ATAG %#lx / KERNEL_IMG %#lx / initrd %#lx out of the expected layout\n",
		       a.atag, a.kernel_img, a.initrd_addr);
		goto out;
	}
	if (rc2a_kernel(&a) ||
	    rc2a_krnl(&a, "boot", a.initrd_addr, RC2A_INITRD_MAX, &a.initrd_size))
		goto out;
	/* the loader passes the size rounded up; the unpacker skips zero padding */
	memset((void *)(a.initrd_addr + a.initrd_size), 0,
	       ((a.initrd_size + 0x3ffff) & ~0xffffUL) - a.initrd_size);
	if (rc2a_initrd(&a, true))
		goto out;
	printf("rc2android: ramdisk: \"boot\" KRNL, %#lx bytes at %#lx\n",
	       a.initrd_size, a.initrd_addr);
	printf("rc2android: ATAGs at %#lx (%lu bytes), machine %lu, entry %#lx\n",
	       a.atag, rc2a_atags(&a), a.machine, a.kernel_img);
	printf("rc2android: cmdline: %s\n", a.cmdline);
	rc2a_pads_report();
	if (readl(WDT_CR) & 1)
		printf("rc2android: warning: the watchdog runs; the stock kernel may not feed it\n");
	if (rc2a_sram_check())
		goto out;
	if (dry) {
		printf("rc2android: -n: all checks passed, not starting the kernel\n");
		ret = CMD_RET_SUCCESS;
		goto out;
	}
	rc2a_jump(&a);
out:
	free(a.parm);
	return ret;
}

U_BOOT_CMD(rc2android, 2, 0, do_rc2android,
	   "boot the stock kernel from the eMMC as the stock loader does",
	   "[-n]\n"
	   "    -n: load and check everything, report, do not start the kernel\n"
	   "    env rc2a_mmc (default 1), rc2a_bootver, rc2a_args (appended)");

/*
 * rc2cache: the eMMC "cache" partition as a block device for the Linux
 * slots, through a blkmap ("rc2cache") linearly mapped onto the eMMC user
 * area. Where "cache" lies comes from the parameter's mtdparts=; the
 * command refuses a parameter whose "cache" is not where this board puts
 * it (RC2C_CACHE_LBA, RC2C_CACHE_SECTORS), so a different layout never
 * gets a filesystem read from the wrong place. Nothing is written: with
 * MMC_WRITE off the eMMC has no block write op, so a write through the
 * map ends in blk_write() with -ENOSYS.
 *
 * Sets rc2_bm (the blkmap device number, for "blkmap ${rc2_bm}:0") and
 * rc2_blkdevparts: "blkdevparts=mmcblkN:SIZE@OFFSET(NAME),..." with every
 * entry of mtdparts= in bytes from the start of the user area, the kernel's
 * cmdline partition syntax (N = the U-Boot mmc number; the kernel DT
 * gives the eMMC the same mmc alias).
 */
#define RC2C_CACHE_LBA		(RC2A_PART_BASE + 0x120000)
#define RC2C_CACHE_SECTORS	0x100000
#define RC2C_BLKDEVPARTS_MAX	512

static bool rc2c_name_ok(const struct rc2a_mtdpart *mp)
{
	int i;

	for (i = 0; i < mp->name_len; i++)
		if (!isalnum(mp->name[i]) && mp->name[i] != '_' && mp->name[i] != '-')
			return false;
	return mp->name_len > 0;
}

static int rc2c_blkdevparts(struct rc2a *a, char *buf, size_t len)
{
	const char *p = rc2a_mtdparts(a);
	struct rc2a_mtdpart mp;
	size_t n;
	int ret = 0;

	n = snprintf(buf, len, "blkdevparts=mmcblk%d:", a->dev);
	while (n < len && !(ret = rc2a_mtdpart_next(a, &p, &mp))) {
		if (!rc2c_name_ok(&mp) || mp.off >= a->desc->lba ||
		    (!mp.rest && mp.size > a->desc->lba - mp.off)) {
			printf("rc2cache: mtdparts entry \"%.*s\" not usable\n",
			       mp.name_len, mp.name);
			return -EINVAL;
		}
		if (buf[n - 1] != ':')
			buf[n++] = ',';
		if (mp.rest)
			n += snprintf(buf + n, len - n, "-");
		else
			n += snprintf(buf + n, len - n, "%#llx", (u64)mp.size << 9);
		if (n < len)
			n += snprintf(buf + n, len - n, "@%#llx(%.*s)", (u64)mp.off << 9,
				      mp.name_len, mp.name);
	}
	if (n >= len) {
		printf("rc2cache: blkdevparts= longer than %zu bytes\n", len - 1);
		return -E2BIG;
	}
	if (ret != -ENOENT || buf[n - 1] == ':') {
		printf("rc2cache: malformed or empty mtdparts=\n");
		return -EINVAL;
	}
	return 0;
}

static int do_rc2cache(struct cmd_tbl *cmdtp, int flag, int argc, char *const argv[])
{
	char parts[RC2C_BLKDEVPARTS_MAX];
	struct rc2a a = { };
	struct udevice *bm_dev;
	struct blk_desc *bd;
	lbaint_t off, size;
	int ret;

	if (rc2a_open(&a))
		return CMD_RET_FAILURE;
	ret = rc2a_parameter(&a) || rc2a_cmdline(&a) || rc2a_part(&a, "cache", &off, &size) ||
	      rc2c_blkdevparts(&a, parts, sizeof(parts));
	if (ret)
		goto out;
	ret = -EINVAL;
	if (off != RC2C_CACHE_LBA || size != RC2C_CACHE_SECTORS || off + size > a.desc->lba) {
		printf("rc2cache: \"cache\" at sectors %#lx+%#lx, expected %#x+%#x: refused\n",
		       (ulong)off, (ulong)size, RC2C_CACHE_LBA, RC2C_CACHE_SECTORS);
		goto out;
	}
	bm_dev = blkmap_from_label("rc2cache");
	if (bm_dev) {
		ret = blkmap_destroy(bm_dev);
		if (ret)
			goto fail;
	}
	ret = blkmap_create("rc2cache", &bm_dev);
	if (ret)
		goto fail;
	ret = blkmap_map_linear(bm_dev, 0, size, a.desc->bdev, off);
	if (ret) {
		blkmap_destroy(bm_dev);
		goto fail;
	}
	bd = dev_get_uclass_plat(((struct blkmap *)dev_get_plat(bm_dev))->blk);
	env_set_hex("rc2_bm", bd->devnum);
	env_set("rc2_blkdevparts", parts);
	printf("rc2cache: \"cache\" = mmc %d sectors %#lx+%#lx as blkmap %d, read only\n",
	       a.dev, (ulong)off, (ulong)size, bd->devnum);
	goto out;
fail:
	printf("rc2cache: blkmap failed (%d)\n", ret);
out:
	free(a.parm);
	return ret ? CMD_RET_FAILURE : CMD_RET_SUCCESS;
}

U_BOOT_CMD(rc2cache, 1, 0, do_rc2cache,
	   "map the eMMC \"cache\" partition read-only as blkmap \"rc2cache\"",
	   "\n"
	   "    sets rc2_bm (blkmap device number) and rc2_blkdevparts (kernel\n"
	   "    blkdevparts= from the parameter's mtdparts=); env rc2a_mmc (default 1)");

/*
 * rc2key: true while the Page-Down pad (GPIO2_B6, active low) is touched.
 * The pad chip calibrates for about half a second after power-on, so a
 * finger already on it at power-on reads as released: touch it after
 * switching on. Page-Up (GPIO2_B5) is the stock loader's RECOVER_KEY and
 * would send the loader to Rockusb or recovery, so it is not used here.
 */
#define GRF_GPIO2B_IOMUX	0x200080cc
#define GRF_GPIO2B_PULL		0x20008128
#define PAGEDOWN_BIT		BIT(14)

static int do_rc2key(struct cmd_tbl *cmdtp, int flag, int argc, char *const argv[])
{
	int low = 0, i;

	writel(BIT(12 + 16) | BIT(13 + 16), (void __iomem *)GRF_GPIO2B_IOMUX);	/* GPIO */
	writel(BIT(14 + 16) | BIT(14), (void __iomem *)GRF_GPIO2B_PULL);	/* pull off */
	clrbits_le32((void __iomem *)(GPIO2_BASE + 0x04), PAGEDOWN_BIT);	/* input */
	udelay(1000);
	for (i = 0; i < 3; i++) {
		if (!(readl((void __iomem *)(GPIO2_BASE + 0x50)) & PAGEDOWN_BIT))
			low++;
		udelay(1000);
	}

	return low >= 2 ? CMD_RET_SUCCESS : CMD_RET_FAILURE;
}

U_BOOT_CMD(rc2key, 1, 0, do_rc2key,
	   "true while the Page-Down pad is touched", "");
