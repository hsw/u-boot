# U-Boot for the ONYX BOOX Robinson Crusoe 2 (RK3026)

Build as for the C67ML (`../boox-c67ml/README.md`), then write the SPL and
U-Boot proper as one rksd image at sector 64 of the card:

```bash
make ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf- boox_rc2_defconfig
make ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf-
tools/mkimage -n rk3026 -T rksd -d spl/u-boot-spl.bin:u-boot.bin idbloader.img
sudo dd if=idbloader.img of=/dev/sdX seek=64 conv=fsync
```

Not u-boot-rockchip.bin: the SPL returns to the mask ROM, which loads
the next stage from the blocks right after the SPL, and that image has
U-Boot proper at 0x7f8000 with padding in between. idbloader.img is
about 280 KiB (sectors 64 to about 612); the first partition has to
start above it.

To boot a kernel by hand, pass a DTB:

```
load mmc 0:1 ${kernel_addr_r} zImage
load mmc 0:1 ${fdt_addr_r} rk3026-onyx-rc2.dtb
bootz ${kernel_addr_r} - ${fdt_addr_r}
```

A bootz without a DTB does not start the kernel: this defconfig has no
ATAGS support, so U-Boot panics ("FDT and ATAGS support not compiled
in") and resets (from the code; not run).

On a cold start the mask ROM boots the microSD card before the internal
eMMC. The eMMC keeps the vendor firmware: this U-Boot neither describes
nor probes it, and nothing here writes it. Without the card the book
boots the vendor firmware as before.

The console is the USB CDC-ACM gadget; UART2 has no known connector.

`reset` and `reset -w` are CRU global resets, and both clear the GPIO1
bank (a value written to a GPIO1 register reads 0 after either, checked
with the cable in): the power hold (GPIO1_A2) drops until the SPL raises
it again, so on battery the book powers off instead of restarting. On
battery this was seen with the SND form, which `reset -w` uses; `reset`
was not tried on battery. With the cable in both restart.
