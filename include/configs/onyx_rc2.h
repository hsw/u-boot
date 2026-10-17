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
 * driver keeps it fed and a hung one is reset. Optional steps end in
 * "else true": the old hush gives an "if" without a taken branch the
 * condition's status, which would fail "run rc2_load" / "run rc2_prep".
 *
 * "run rc2_ums" exports the card over USB mass storage until the host
 * ejects it, then brings the CDC ACM console back:
 *   rc2_ums_dcache=off   ums with the D-cache (and MMU) off: ~1 MiB/s, the
 *                        fallback; default on. "dcache on" follows ums.
 *   rc2_ums_then=CMDS    run right after ums returns, before the console
 *                        goes back to usbacm (e.g. "boot")
 *   rc2_ums_wdt=1        (default) start the watchdog (rc2_wdt_ms) before
 *                        the console restart: U-Boot feeds it while it
 *                        runs, a hang resets the book (bootcount + 1, then
 *                        autoboot). It stays running afterwards.
 *   rc2_ums_acmwait=SECS (default 60) after the restart wait that long for a
 *                        key on the console; none: run rc2_ums_noacm
 *                        (default "boot"). 0: no wait.
 * "rc2_ustat" keeps the ums counters at 0x7fff6100 and a USB event trace at
 * 0x7fff6200-0x7fff6fff (board file). The DDR request path (preboot) runs
 * ums without the watchdog and the wait, then resets as before.
 *
 * Words in DDR (inside the kernel's no-map loader-log@7ffc0000, above the
 * stage2 pages 0x7fff0000-0x7fff4fff):
 *   0x7fff6000/04  bootcount + magic 0xb001c041 (BOOTCOUNT_GENERIC); Linux
 *                  clears it after a good boot: devmem 0x7fff6000 32 0
 *   0x7fff6010/14  0x52325553 / 0xadcdaaac: "export the card over USB mass
 *                  storage on the next boot" (one shot, cleared by preboot)
 *   0x7fff6100-2b  ums counters (rc2_ustat, board file)
 *   0x7fff6200-0x7fff6fff  USB event trace (rc2_ustat, board file)
 */

#ifndef __CONFIG_H
#define __CONFIG_H

#include <configs/rk3026_common.h>

#ifdef CONFIG_XPL_BUILD
/*
 * SPL timebase (onyx_rc2_mspl_defconfig only): the A9 global timer through
 * lib/time.c's pre-driver-model path, at the mask ROM's clocks (CORE_PERI =
 * 24 MHz / 4, measured in the SPL on the same SoC, include/configs/
 * boox_c67ml.h). Valid only while the SPL leaves the PLLs alone.
 */
#define CFG_SYS_TIMER_COUNTER	0x1013c200
#define CFG_SYS_TIMER_RATE	6000000
#endif

#undef CFG_EXTRA_ENV_SETTINGS
#define CFG_EXTRA_ENV_SETTINGS \
	"stdout=serial,usbacm\0" \
	"stderr=serial,usbacm\0" \
	"stdin=serial,usbacm\0" \
	"scriptaddr=0x60c00000\0" \
	"kernel_addr_r=0x61000000\0" \
	"fdt_addr_r=0x61f00000\0" \
	"ramdisk_addr_r=0x62000000\0" \
	"bootm_size=0x10000000\0" \
	"bootlimit=2\0" \
	"rc2_wdt=1\0" \
	"rc2_wdt_ms=15000\0" \
	"rc2_wdt_dt=1\0" \
	"rc2_ums_dcache=on\0" \
	"rc2_ums_wdt=1\0" \
	"rc2_ums_acmwait=60\0" \
	"rc2_ums_noacm=boot\0" \
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
		"setenv rc2_rd ${ramdisk_addr_r}:${filesize}; else true; fi\0" \
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
		"wdt dev watchdog@2004c000 && wdt start ${rc2_wdt_ms}; " \
		"else true; fi\0" \
	"rc2_boot=echo rc2: slot ${rc2_slot}; run rc2_load && run rc2_prep && " \
		"bootz ${kernel_addr_r} ${rc2_rd} ${fdt_addr_r}\0" \
	"rc2_bootcmd=setenv rc2_slot b; run rc2_boot; " \
		"setenv rc2_slot a; run rc2_boot; echo rc2: no bootable slot\0" \
	"altbootcmd=echo rc2: bootcount ${bootcount} above bootlimit ${bootlimit}, slot a; " \
		"setenv rc2_slot a; run rc2_boot; echo rc2: slot a failed\0" \
	"rc2_ums=echo rc2: microSD over USB mass storage, eject on the host to end; " \
		"setenv stdin serial; setenv stdout serial; setenv stderr serial; " \
		"rc2_ustat start; " \
		"if test \"${rc2_ums_dcache}\" = off; then dcache off; fi; " \
		"ums 0 mmc 0; dcache on; rc2_ustat save; " \
		"if test -n \"${rc2_ums_then}\"; then run rc2_ums_then; fi; " \
		"if test \"${rc2_ums_wdt}\" = 1; then " \
		"wdt dev watchdog@2004c000 && wdt start ${rc2_wdt_ms}; fi; " \
		"setenv stdin serial,usbacm; setenv stdout serial,usbacm; " \
		"setenv stderr serial,usbacm; echo rc2: ums ended; " \
		"if rc2_acmwait ${rc2_ums_acmwait}; then true; " \
		"else run rc2_ums_noacm; fi\0" \
	"preboot=if itest.l *0x7fff6010 == 0x52325553 && " \
		"itest.l *0x7fff6014 == 0xadcdaaac; then " \
		"mw.l 0x7fff6010 0 2; dcache flush; " \
		"setenv rc2_ums_wdt 0; setenv rc2_ums_acmwait 0; " \
		"run rc2_ums; reset; fi\0"

#endif
