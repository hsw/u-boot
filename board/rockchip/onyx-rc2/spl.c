// SPDX-License-Identifier: GPL-2.0+
/*
 * ONYX BOOX Robinson Crusoe 2: SPL board code, only for the variant that
 * boots through the RK3026 SPL (onyx_rc2_mspl_defconfig) instead of the
 * stock DDR init and the sdboot stage2. No storage access: the power
 * hold, the ACT8931 voltage restore (act8931-restore.h) and the DDR die
 * geometry, nothing else.
 */

#include <asm/io.h>
#include <asm/arch-rockchip/sdram_rk3026.h>
#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include "act8931-restore.h"

#define CRU_CLKGATE_CON8	0x200000f0
#define GRF_GPIO0A_IOMUX	0x200080a8
#define GRF_GPIO1A_IOMUX	0x200080b8
#define GPIO0_BASE		0x2007c000
#define GPIO1_BASE		0x20080000
#define GPIO_SWPORTA_DR		0x00
#define GPIO_SWPORTA_DDR	0x04
#define GPIO_EXT_PORTA		0x50
#define POWER_HOLD_BIT		BIT(2)		/* GPIO1_A2 */
#define SCL			BIT(0)		/* GPIO0_A0 = I2C0_SCL */
#define SDA			BIT(1)		/* GPIO0_A1 = I2C0_SDA */
#define VSEL			BIT(3)		/* GPIO0_A3 = ACT8931 VSEL */

void rk3026_spl_board_init_f(void)
{
	/*
	 * Power hold high at once, as stock FlashBoot, the sdboot stage2 and
	 * U-Boot proper do it: pclk_gpio1 on, pin to GPIO, data high, output.
	 * On battery the PMIC drops the rails when the power key is released
	 * unless this line is high.
	 */
	writel(BIT(10 + 16), (void __iomem *)CRU_CLKGATE_CON8);
	writel(0x00300000, (void __iomem *)GRF_GPIO1A_IOMUX);
	setbits_le32((void __iomem *)(GPIO1_BASE + GPIO_SWPORTA_DR), POWER_HOLD_BIT);
	setbits_le32((void __iomem *)(GPIO1_BASE + GPIO_SWPORTA_DDR), POWER_HOLD_BIT);
}

void get_ddr_config(struct rk3026_ddr_config *config)
{
	/*
	 * One rank of DDR3, x16 die, 8 banks, 15 row and 10 column bits:
	 * 512 MiB. Read back on an RC2 after the stock DDR init: GRF_OS_REG1
	 * = 0x6281, the word sdram_all_config() writes for this geometry.
	 */
	config->ddr_type = 3;
	config->rank = 1;
	config->cs0_row = 15;
	config->cs1_row = 15;
	config->bank = 3;
	config->col = 10;
	config->bw = 1;
}

/*
 * ACT8931 voltage restore (act8931-restore.h), before the DDR PLL starts.
 *
 * Here the CPU still runs from the 24 MHz oscillator: the BootROM leaves
 * the PLLs in bypass (non-secure boot) and this SPL raises only the DPLL,
 * in sdram_init(); APLL/GPLL follow in U-Boot proper's clock driver. So
 * this is the last point before any clock leaves 24 MHz, and everything
 * after it runs at the restored voltages.
 *
 * I2C by GPIO bit-bang on i2c0's pins: open drain through the direction
 * register (data 0; input = released, the board's pull-ups lift the line),
 * about 100 kHz with the SPL timebase (CFG_SYS_TIMER_RATE, valid while the
 * PLLs are in bypass), clock stretching bounded. No controller, no GRF
 * select, and a stuck bus (a watchdog reset in the middle of a read leaves
 * a slave driving SDA) is freed with nine clocks first. The pin mux goes
 * back to how it was found, the lines are left released. Any error ends the
 * restore and the boot goes on; the record says what happened. Raw
 * accessors: the MMU is off, every access is strongly ordered.
 */
static u32 rc2_act_rec[RC2_ACT_REC_WORDS];	/* .bss is not cleared in this SPL */

#define GPIO0_REG(off)		((void __iomem *)(GPIO0_BASE + (off)))

static void bb_set(u32 line, int high)
{
	u32 ddr = __raw_readl(GPIO0_REG(GPIO_SWPORTA_DDR));

	__raw_writel(high ? ddr & ~line : ddr | line, GPIO0_REG(GPIO_SWPORTA_DDR));
	udelay(5);
}

static u32 bb_get(u32 line)
{
	return __raw_readl(GPIO0_REG(GPIO_EXT_PORTA)) & line;
}

/* release SCL and wait (at most ~1 ms) for a stretching slave; 0 when it is high */
static int bb_scl_high(void)
{
	int n;

	bb_set(SCL, 1);
	for (n = 0; n < 200 && !bb_get(SCL); n++)
		udelay(5);
	return !bb_get(SCL);
}

/* one clock with SDA = out: SDA as sampled during SCL high, < 0 on a stuck SCL */
static int bb_bit(int out)
{
	int in;

	bb_set(SDA, out);
	if (bb_scl_high())
		return -1;
	in = !!bb_get(SDA);
	bb_set(SCL, 0);
	return in;
}

/* (repeated) start; fails when SDA or SCL does not come up */
static int bb_start(void)
{
	bb_set(SDA, 1);
	if (bb_scl_high() || !bb_get(SDA))
		return -1;
	bb_set(SDA, 0);
	bb_set(SCL, 0);
	return 0;
}

static void bb_stop(void)
{
	bb_set(SDA, 0);
	bb_scl_high();
	bb_set(SDA, 1);
}

/* eight bits out, then the ACK clock: 0 when the slave ACKed */
static int bb_write(int b)
{
	int i;

	for (i = 7; i >= 0; i--)
		if (bb_bit(b >> i & 1) < 0)
			return -1;
	return bb_bit(1);
}

/* write one register (val >= 0) or read it: the byte read, 0 after a write, < 0 on an error */
static int act_xfer(int reg, int val)
{
	int i, b, r = -1;

	if (bb_start() || bb_write(ACT8931_I2C_ADDR << 1) || bb_write(reg))
		goto out;
	if (val >= 0) {
		r = -bb_write(val);
		goto out;
	}
	if (bb_start() || bb_write(ACT8931_I2C_ADDR << 1 | 1))
		goto out;
	for (r = 0, i = 0; i < 8 && r >= 0; i++) {
		b = bb_bit(1);
		r = b < 0 ? -1 : r << 1 | b;
	}
	bb_bit(1);			/* NACK: one byte only */
out:
	bb_stop();
	return r;
}

/* every transfer gets a second try */
static int act_xfer2(int reg, int val)
{
	int r = act_xfer(reg, val);

	return r < 0 ? act_xfer(reg, val) : r;
}

void rk3026_spl_board_pre_dram(void)
{
	static const struct rc2_act_rail rails[RC2_ACT_RAILS] = RC2_ACT_RAIL_TABLE;
	void __iomem *iomux = (void __iomem *)GRF_GPIO0A_IOMUX;
	u32 mux = __raw_readl(iomux) & 0xf;
	u32 vsel, err = RC2_ACT_EOK, w;
	int i, v, nv, reg, wrote = 0;

	/* pclk_gpio0 on; SCL/SDA data 0 and input (released), then the pins to GPIO */
	__raw_writel(BIT(9 + 16), (void __iomem *)CRU_CLKGATE_CON8);
	__raw_writel(__raw_readl(GPIO0_REG(GPIO_SWPORTA_DR)) & ~(SCL | SDA), GPIO0_REG(GPIO_SWPORTA_DR));
	bb_set(SCL | SDA, 1);
	__raw_writel(0x000f0000, iomux);
	vsel = !!bb_get(VSEL);

	/* bus recovery: clock until the slave lets SDA go (at most 9), then a stop */
	for (i = 0; i < 9 && !bb_get(SDA); i++) {
		bb_set(SCL, 0);
		bb_scl_high();
	}
	bb_stop();
	if (!bb_get(SDA) || !bb_get(SCL))
		err = RC2_ACT_EBUS;

	for (i = 0; i < RC2_ACT_RAILS; i++) {
		w = 0;
		if (err == RC2_ACT_EOK) {
			reg = vsel && rails[i].vset2 ? rails[i].vset2 : rails[i].vset1;
			v = act_xfer2(reg, -1);
			w = reg | v << 8;
			if (v < 0) {
				w = reg;
				err = RC2_ACT_EXFER;
			} else if (rc2_act_needs_restore(v, rails[i].target)) {
				nv = rc2_act_restored(v, rails[i].target);
				w |= nv << 16;
				wrote = 1;
				if (act_xfer2(reg, nv) < 0 || (v = act_xfer2(reg, -1)) < 0)
					err = RC2_ACT_EXFER;
				else if (v != nv)
					err = RC2_ACT_EVERIFY, w |= v << 24;
			}
			if (err)
				err |= i << 12;
		}
		rc2_act_rec[2 + i] = w;
	}
	rc2_act_rec[1] = err | vsel << 8;

	__raw_writel(0x000f0000 | mux, iomux);
	/* the bucks and LDOs settle well within this before the DPLL starts */
	if (wrote)
		udelay(2000);
}

/* the record to DDR for U-Boot proper (and Linux), magic last */
void rk3026_spl_board_post_dram(void)
{
	u32 *rec = (u32 *)RC2_ACT_REC;
	int i;

	for (i = 1; i < RC2_ACT_REC_WORDS; i++)
		__raw_writel(rc2_act_rec[i], &rec[i]);
	__raw_writel(RC2_ACT_MAGIC, &rec[0]);
}
