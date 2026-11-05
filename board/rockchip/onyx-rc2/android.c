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
#include <bootm.h>
#include <command.h>
#include <cpu_func.h>
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
		memcpy(a->parm, buf + 8, len);
		a->parm[len] = '\0';
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
		const char *eol = strpbrk(p, "\r\n");
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

/* offset and size (sectors, partition-relative) of a partition of mtdparts= */
static int rc2a_part(struct rc2a *a, const char *name, lbaint_t *off, lbaint_t *size)
{
	const char *p = strstr(a->cmdline, "mtdparts=");
	size_t nl = strlen(name);

	if (p)
		p = strchr(p, ':');
	if (!p)
		return -ENOENT;
	for (p++; *p && *p != ' '; ) {
		char *e;
		ulong sz = 0, of;

		if (*p == '-')
			e = (char *)p + 1;
		else
			sz = simple_strtoul(p, &e, 0);
		if (*e != '@')
			return -EINVAL;
		of = simple_strtoul(e + 1, &e, 0);
		if (*e != '(')
			return -EINVAL;
		if (!strncmp(e + 1, name, nl) && e[1 + nl] == ')') {
			*off = RC2A_PART_BASE + of;
			*size = sz ? sz : a->desc->lba - *off;
			return 0;
		}
		p = strchr(e, ')');
		if (!p)
			return -EINVAL;
		p++;
		if (*p == ',')
			p++;
	}
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

static void __noreturn rc2a_jump(struct rc2a *a)
{
	void (*entry)(int, int, uint) = (void *)a->kernel_img;

	bootm_final(0);
	cleanup_before_linux();
	/* as the stock loader leaves them: CPU1 in soft reset, IMEM at address 0 */
	writel(0x00100010, (void __iomem *)CRU_SOFTRST_CON0);
	writel(BIT(12 + 16) | BIT(12), (void __iomem *)GRF_SOC_CON0);
	dsb();
	isb();
	entry(0, a->machine, a->atag);
	hang();
}

static int do_rc2android(struct cmd_tbl *cmdtp, int flag, int argc, char *const argv[])
{
	struct rc2a a = { };
	bool dry = argc > 1 && !strcmp(argv[1], "-n");
	int dev = env_get_ulong("rc2a_mmc", 10, 1);
	struct mmc *mmc = find_mmc_device(dev);
	int ret = CMD_RET_FAILURE;

	if (!mmc || mmc_init(mmc)) {
		printf("rc2android: no eMMC at mmc %d\n", dev);
		return CMD_RET_FAILURE;
	}
	a.desc = mmc_get_blk_desc(mmc);
	if (!a.desc || a.desc->hwpart) {
		printf("rc2android: mmc %d is not on its user area\n", dev);
		return CMD_RET_FAILURE;
	}
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
 * rc2key: true while the Page-Down pad (GPIO2_B6, active low) is touched.
 * The pad chip calibrates for about half a second after power-on, so a
 * finger already on it at power-on reads as released: touch it after
 * switching on. Page-Up (GPIO2_B5) is the stock loader's RECOVER_KEY and
 * would send the loader to Rockusb or recovery, so it is not used here.
 */
#define GRF_GPIO2B_IOMUX	0x200080cc
#define GRF_GPIO2B_PULL		0x20008128
#define GPIO2_BASE		0x20084000
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
