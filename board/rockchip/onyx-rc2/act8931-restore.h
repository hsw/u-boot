/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * ONYX BOOX Robinson Crusoe 2: ACT8931 output voltages restored at boot.
 *
 * The ACT8931 keeps its registers over every SoC reset and over a power-off
 * on battery; only VSYS below UVLO brings back the OTP defaults. A kernel
 * that lowered a rail (DVFS sweeps; the kernel's caps have no floor) and
 * then hung leaves the BootROM, the SPL's DDR init and U-Boot running at
 * that voltage after the watchdog reset. The SPL therefore raises every
 * voltage selector the kernel can lower back to the book's power-on value
 * before it starts the DDR PLL (spl.c), and U-Boot proper prints what the
 * SPL did (onyx-rc2.c).
 *
 * Shared by spl.c, onyx-rc2.c and a host test, so it needs only u8 and
 * _Static_assert.
 */

#ifndef __RC2_ACT8931_RESTORE_H
#define __RC2_ACT8931_RESTORE_H

#define ACT8931_I2C_ADDR	0x5b		/* on i2c0: GPIO0_A0 SCL, GPIO0_A1 SDA */
#define ACT8931_VSET_MASK	0x3f		/* VSET[5:0]; bits 7:6 reserved, kept */

/*
 * Targets: the active VSET of each output in the book's power-on dumps
 * (under stock Android and under mainline): DCDC1
 * VCCIO 0x3a (stock idle; 0x3b with the Wi-Fi/BT module powered, which
 * every mainline boot reads), DCDC2 DDR 0x1b, DCDC3 CPU 0x1c, LDO1-4 0x37.
 * The restore only raises: a selector below its target is set to the
 * target, one at or above it is left alone (DCDC1 0x3b stays), so nothing
 * is ever lowered. Caps: the kernel's compiled-in stock caps.
 */
#define RC2_ACT_T_DCDC1		0x3a
#define RC2_ACT_T_DCDC2		0x1b
#define RC2_ACT_T_DCDC3		0x1c
#define RC2_ACT_T_LDO		0x37
#define RC2_ACT_CAP_DCDC1	0x3b
#define RC2_ACT_CAP_DCDC2	0x1b
#define RC2_ACT_CAP_DCDC3	0x1c
#define RC2_ACT_CAP_LDO		0x37

_Static_assert(RC2_ACT_T_DCDC1 <= RC2_ACT_CAP_DCDC1 && RC2_ACT_T_DCDC2 <= RC2_ACT_CAP_DCDC2 &&
	       RC2_ACT_T_DCDC3 <= RC2_ACT_CAP_DCDC3 && RC2_ACT_T_LDO <= RC2_ACT_CAP_LDO,
	       "an ACT8931 restore target above the stock cap");

/*
 * One output: VSET1 (selected while the VSEL pin GPIO0_A3 is low), VSET2
 * (VSEL high; 0 for the LDOs, which have one VSET) and the target. The
 * SPL reads VSEL once and restores the live set only.
 */
struct rc2_act_rail {
	u8 vset1;
	u8 vset2;
	u8 target;
};

#define RC2_ACT_RAILS		7
#define RC2_ACT_RAIL_TABLE {					\
	{ 0x20, 0x21, RC2_ACT_T_DCDC1 },	/* DCDC1 */	\
	{ 0x30, 0x31, RC2_ACT_T_DCDC2 },	/* DCDC2 */	\
	{ 0x40, 0x41, RC2_ACT_T_DCDC3 },	/* DCDC3 */	\
	{ 0x50, 0,    RC2_ACT_T_LDO },		/* LDO1 */	\
	{ 0x54, 0,    RC2_ACT_T_LDO },		/* LDO2 */	\
	{ 0x60, 0,    RC2_ACT_T_LDO },		/* LDO3 */	\
	{ 0x64, 0,    RC2_ACT_T_LDO },		/* LDO4 */	\
}
#define RC2_ACT_RAIL_NAMES { "DCDC1", "DCDC2", "DCDC3", "LDO1", "LDO2", "LDO3", "LDO4" }

/* raise only, VSET field only */
static inline int rc2_act_needs_restore(u8 old, u8 target)
{
	return (old & ACT8931_VSET_MASK) < target;
}

static inline u8 rc2_act_restored(u8 old, u8 target)
{
	return (old & ~ACT8931_VSET_MASK) | target;
}

/*
 * The SPL's record, in the reserved top MiB between the UMS request words
 * (0x7fff6010/14) and the ums block (0x7fff6100); written after the DDR
 * init, magic last:
 *   word 0  RC2_ACT_MAGIC (this boot's SPL) or RC2_ACT_SEEN (U-Boot proper
 *           printed it; the rest stays for Linux: devmem 0x7fff6020)
 *   word 1  error (RC2_ACT_E*) | VSEL level << 8 | failing rail << 12
 *   word 2+ one per rail in table order: register | old << 8 | new << 16
 *           (new 0: not written; every target is > 0) | read-back << 24
 *           (only with RC2_ACT_EVERIFY); 0 for a rail not reached
 */
#define RC2_ACT_REC		0x7fff6020
#define RC2_ACT_REC_WORDS	(2 + RC2_ACT_RAILS)
#define RC2_ACT_MAGIC		0xac893101
#define RC2_ACT_SEEN		0xac893102

#define RC2_ACT_EOK		0
#define RC2_ACT_EBUS		1	/* SDA or SCL held low after the bus recovery */
#define RC2_ACT_EXFER		2	/* no ACK or SCL held low in a transfer */
#define RC2_ACT_EVERIFY		3	/* read back differs from the value written */

#endif /* __RC2_ACT8931_RESTORE_H */
