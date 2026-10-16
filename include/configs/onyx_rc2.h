/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * ONYX BOOX Robinson Crusoe 2 board config.
 *
 * Boot flow: bootcmd tries slot B (the build under test), then slot A (known
 * good); altbootcmd (bootcount > bootlimit) boots slot A only. A slot is
 * /rc2boot/<a|b>/{zImage, rc2.dtb[, initrd.gz]} on partition 2 (FAT32
 * RC2DATA, writable from the host over "ums") or else partition 1 (ext4
 * rc2root, in the card image). The watchdog is started right before bootz
 * and the kernel DT's watchdog node is enabled, so a kernel with the dw_wdt
 * driver keeps it fed and a hung one is reset.
 *
 * Words in DDR (inside the kernel's no-map loader-log@7ffc0000, above the
 * stage2 pages 0x7fff0000-0x7fff4fff):
 *   0x7fff6000/04  bootcount + magic 0xb001c041 (BOOTCOUNT_GENERIC); Linux
 *                  clears it after a good boot: devmem 0x7fff6000 32 0
 *   0x7fff6010/14  0x52325553 / 0xadcdaaac: "export the card over USB mass
 *                  storage on the next boot" (one shot, cleared by preboot)
 */

#ifndef __CONFIG_H
#define __CONFIG_H

#include <configs/rk3026_common.h>

#undef CFG_EXTRA_ENV_SETTINGS
#define CFG_EXTRA_ENV_SETTINGS \
	"stdout=serial,usbacm\0" \
	"stderr=serial,usbacm\0" \
	"stdin=serial,usbacm\0" \
	"scriptaddr=0x60c00000\0" \
	"kernel_addr_r=0x61000000\0" \
	"fdt_addr_r=0x61f00000\0" \
	"ramdisk_addr_r=0x62000000\0" \
	"fdt_high=0xffffffff\0" \
	"initrd_high=0xffffffff\0" \
	"bootlimit=2\0" \
	"rc2_wdt=1\0" \
	"rc2_wdt_ms=15000\0" \
	"rc2_wdt_dt=1\0" \
	"rc2_args=console=ttyGS0,115200 loglevel=8 ignore_loglevel " \
		"clk_ignore_unused softlockup_panic=1 hung_task_panic=1 " \
		"rc2ml.savelog=1\0" \
	"rc2_find=setenv rc2_part; for p in 2 1; do " \
		"if test -z \"${rc2_part}\" && " \
		"test -e mmc 0:${p} /rc2boot/${rc2_slot}/zImage; then " \
		"setenv rc2_part ${p}; fi; done; test -n \"${rc2_part}\"\0" \
	"rc2_load_rd=setenv rc2_rd -; " \
		"if test -e mmc 0:${rc2_part} /rc2boot/${rc2_slot}/initrd.gz; then " \
		"load mmc 0:${rc2_part} ${ramdisk_addr_r} " \
		"/rc2boot/${rc2_slot}/initrd.gz && " \
		"setenv rc2_rd ${ramdisk_addr_r}:${filesize}; fi\0" \
	"rc2_load=run rc2_find && " \
		"load mmc 0:${rc2_part} ${kernel_addr_r} /rc2boot/${rc2_slot}/zImage && " \
		"load mmc 0:${rc2_part} ${fdt_addr_r} /rc2boot/${rc2_slot}/rc2.dtb && " \
		"run rc2_load_rd\0" \
	"rc2_prep=fdt addr ${fdt_addr_r} && fdt resize 0x1000 && " \
		"if test \"${rc2_wdt_dt}\" = 1; then " \
		"fdt set /watchdog@2004c000 status okay; fi; " \
		"setenv bootargs \"${rc2_args} rc2.loader=uboot rc2.slot=${rc2_slot} " \
		"rc2.part=${rc2_part} rc2.bootcount=${bootcount}\"; " \
		"if test \"${rc2_wdt}\" = 1; then " \
		"wdt dev watchdog@2004c000 && wdt start ${rc2_wdt_ms}; fi\0" \
	"rc2_boot=echo rc2: slot ${rc2_slot}; run rc2_load && run rc2_prep && " \
		"bootz ${kernel_addr_r} ${rc2_rd} ${fdt_addr_r}\0" \
	"rc2_bootcmd=setenv rc2_slot b; run rc2_boot; " \
		"setenv rc2_slot a; run rc2_boot; echo rc2: no bootable slot\0" \
	"altbootcmd=echo rc2: bootcount ${bootcount} above bootlimit ${bootlimit}, slot a; " \
		"setenv rc2_slot a; run rc2_boot; echo rc2: slot a failed\0" \
	"rc2_ums=echo rc2: microSD over USB mass storage, eject on the host to end; " \
		"setenv stdin serial; setenv stdout serial; setenv stderr serial; " \
		"ums 0 mmc 0; " \
		"setenv stdin serial,usbacm; setenv stdout serial,usbacm; " \
		"setenv stderr serial,usbacm; echo rc2: ums ended\0" \
	"preboot=if itest.l *0x7fff6010 == 0x52325553 && " \
		"itest.l *0x7fff6014 == 0xadcdaaac; then " \
		"mw.l 0x7fff6010 0 2; dcache flush; run rc2_ums; reset; fi\0"

#endif
