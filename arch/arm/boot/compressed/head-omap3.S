/*
 * linux/arch/arm/boot/compressed/head-omap3.S
 *
 * OMAP3 specific tweaks.  This is merged into head.S by the linker.
 *
 */

#include <linux/autoconf.h>
#include <linux/linkage.h>
#include <asm/mach-types.h>

		.section        ".start", "ax"

__OMAP_start:
#ifdef CONFIG_MACH_OMAP3_DEVKIT8000
		/* support for booting without u-boot */
		mov	r7, #(MACH_TYPE_OMAP3_BEAGLE & ~0xf)
		orr	r7, r7, #(MACH_TYPE_OMAP3_BEAGLE & 0xf)
#endif
