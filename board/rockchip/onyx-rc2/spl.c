// SPDX-License-Identifier: GPL-2.0+
/*
 * ONYX BOOX Robinson Crusoe 2: SPL board code, only for the variant that
 * boots through the RK3026 SPL (onyx_rc2_mspl_defconfig) instead of the
 * stock DDR init and the sdboot stage2. No storage, no PMIC access: the
 * power hold and the DDR die geometry, nothing else.
 */

#include <asm/io.h>
#include <asm/arch-rockchip/sdram_rk3026.h>
#include <linux/bitops.h>

#define CRU_CLKGATE_CON8	0x200000f0
#define GRF_GPIO1A_IOMUX	0x200080b8
#define GPIO1_BASE		0x20080000
#define GPIO_SWPORTA_DR		0x00
#define GPIO_SWPORTA_DDR	0x04
#define POWER_HOLD_BIT		BIT(2)		/* GPIO1_A2 */

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
