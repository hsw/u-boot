// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2013 Altera Corporation <www.altera.com>
 */

#include <clk.h>
#include <dm.h>
#include <limits.h>
#include <reset.h>
#include <wdt.h>
#include <asm/io.h>
#include <linux/bitops.h>

#define DW_WDT_CR	0x00
#define DW_WDT_TORR	0x04
#define DW_WDT_CRR	0x0C

#define DW_WDT_CR_EN_OFFSET	0x00
#define DW_WDT_CR_RMOD_OFFSET	0x01
#define DW_WDT_CR_RPL_OFFSET	0x02
#define DW_WDT_CRR_RESTART_VAL	0x76

struct designware_wdt_priv {
	void __iomem	*base;
	unsigned int	clk_khz;
	unsigned int	rpl;
	struct reset_ctl_bulk resets;
};

/*
 * Set the watchdog time interval.
 * TOP i gives a period of 2^(16 + i) clock cycles, i = 0..15; pick the
 * shortest period that is not shorter than the timeout, or the longest one.
 */
static int designware_wdt_settimeout(void __iomem *base, unsigned int clk_khz,
				     u64 timeout)
{
	u64 cycles;
	int i;

	/* calculate the timeout range value, in 64 bits: ms * kHz overflows 32 */
	if (timeout > U64_MAX / clk_khz)
		cycles = U64_MAX;
	else
		cycles = timeout * clk_khz;
	for (i = 0; i < 15 && (1ULL << (16 + i)) < cycles; i++)
		;

	writel(i | (i << 4), base + DW_WDT_TORR);

	return 0;
}

static void designware_wdt_enable(void __iomem *base, unsigned int rpl)
{
	writel(BIT(DW_WDT_CR_EN_OFFSET) | rpl << DW_WDT_CR_RPL_OFFSET,
	       base + DW_WDT_CR);
}

static unsigned int designware_wdt_is_enabled(void __iomem *base)
{
	return readl(base + DW_WDT_CR) & BIT(0);
}

static void designware_wdt_reset_common(void __iomem *base)
{
	if (designware_wdt_is_enabled(base))
		/* restart the watchdog counter */
		writel(DW_WDT_CRR_RESTART_VAL, base + DW_WDT_CRR);
}

static int designware_wdt_reset(struct udevice *dev)
{
	struct designware_wdt_priv *priv = dev_get_priv(dev);

	designware_wdt_reset_common(priv->base);

	return 0;
}

static int designware_wdt_stop(struct udevice *dev)
{
	struct designware_wdt_priv *priv = dev_get_priv(dev);
	__maybe_unused int ret;

	designware_wdt_reset(dev);
	/*
	 * Software cannot clear WDT_EN, a running watchdog keeps counting:
	 * keep its reset pulse length. An OS that sets WDT_EN later with a
	 * read-modify-write of CR inherits it too.
	 */
	writel(priv->rpl << DW_WDT_CR_RPL_OFFSET, priv->base + DW_WDT_CR);

	if (CONFIG_IS_ENABLED(DM_RESET) &&
	    ofnode_read_prop(dev_ofnode(dev), "resets", &ret)) {
		ret = reset_assert_bulk(&priv->resets);
		if (ret)
			return ret;

		ret = reset_deassert_bulk(&priv->resets);
		if (ret)
			return ret;
	}

	return 0;
}

static int designware_wdt_start(struct udevice *dev, u64 timeout, ulong flags)
{
	struct designware_wdt_priv *priv = dev_get_priv(dev);

	designware_wdt_stop(dev);

	/* set timer in miliseconds */
	designware_wdt_settimeout(priv->base, priv->clk_khz, timeout);

	designware_wdt_enable(priv->base, priv->rpl);

	/* reset the watchdog */
	return designware_wdt_reset(dev);
}

static int designware_wdt_probe(struct udevice *dev)
{
	struct designware_wdt_priv *priv = dev_get_priv(dev);
	__maybe_unused int ret;

	priv->base = dev_remap_addr(dev);
	if (!priv->base)
		return -EINVAL;

	priv->rpl = dev_get_driver_data(dev);

#if CONFIG_IS_ENABLED(CLK)
	struct clk clk;

	ret = clk_get_by_index(dev, 0, &clk);
	if (ret)
		return ret;

	ret = clk_enable(&clk);
	if (ret)
		return ret;

	priv->clk_khz = clk_get_rate(&clk) / 1000;
	if (!priv->clk_khz)
		return -EINVAL;
#else
	priv->clk_khz = CFG_DW_WDT_CLOCK_KHZ;
#endif

	if (CONFIG_IS_ENABLED(DM_RESET) &&
	    ofnode_read_prop(dev_ofnode(dev), "resets", &ret)) {
		ret = reset_get_bulk(dev, &priv->resets);
		if (ret)
			return ret;

		ret = reset_deassert_bulk(&priv->resets);
		if (ret)
			return ret;
	}

	/* reset to disable the watchdog */
	return designware_wdt_stop(dev);
}

static const struct wdt_ops designware_wdt_ops = {
	.start = designware_wdt_start,
	.reset = designware_wdt_reset,
	.stop = designware_wdt_stop,
};

/*
 * .data is the reset pulse length (CR.RPL, 2^(RPL + 1) pclk cycles). The
 * RK3026's reset logic misses the default 2-cycle pulse now and then, so the
 * watchdog resets one or more periods late; Rockchip's own driver uses 32.
 */
static const struct udevice_id designware_wdt_ids[] = {
	{ .compatible = "rockchip,rk3026-wdt", .data = 4 },
	{ .compatible = "snps,dw-wdt"},
	{}
};

U_BOOT_DRIVER(designware_wdt) = {
	.name = "designware_wdt",
	.id = UCLASS_WDT,
	.of_match = designware_wdt_ids,
	.priv_auto	= sizeof(struct designware_wdt_priv),
	.probe = designware_wdt_probe,
	.ops = &designware_wdt_ops,
	.flags = DM_FLAG_PRE_RELOC,
};
