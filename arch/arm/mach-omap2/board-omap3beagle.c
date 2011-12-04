/*
 * linux/arch/arm/mach-omap2/board-omap3beagle.c
 *
 * Copyright (C) 2008 Texas Instruments
 *
 * Modified from mach-omap2/board-3430sdp.c
 *
 * Initial code: Syed Mohammed Khasim
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/leds.h>
#include <linux/gpio.h>
#include <linux/input.h>
#include <linux/gpio_keys.h>

#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/mtd/nand.h>

#include <linux/regulator/machine.h>
#include <linux/i2c/twl4030.h>
#include <linux/omapfb.h>

#ifdef CONFIG_MACH_OMAP3_DEVKIT8000
#include <linux/dm9000.h>
#endif

#include <mach/hardware.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/mach/flash.h>

#include <mach/board.h>
#include <mach/usb.h>
#include <mach/common.h>
#include <mach/gpmc.h>
#include <mach/nand.h>
#include <mach/mux.h>
#include <mach/display.h>
#include <mach/omap-pm.h>
#include <mach/clock.h>
#include <mach/mmc.h>

#include "twl4030-generic-scripts.h"
#include "mmc-twl4030.h"
#include "pm.h"
#include "omap3-opp.h"
#ifdef CONFIG_MACH_OMAP3_DEVKIT8000
#include <mach/mcspi.h>
#include <linux/spi/spi.h>
#include <linux/spi/ads7846.h>
#endif
#define GPMC_CS0_BASE  0x60
#define GPMC_CS_SIZE   0x30

#define NAND_BLOCK_SIZE		SZ_128K

static struct mtd_partition omap3beagle_nand_partitions[] = {
	/* All the partition sizes are listed in terms of NAND block size */
	{
		.name		= "X-Loader",
		.offset		= 0,
		.size		= 4 * NAND_BLOCK_SIZE,
		.mask_flags	= MTD_WRITEABLE,	/* force read-only */
	},
	{
		.name		= "U-Boot",
		.offset		= MTDPART_OFS_APPEND,	/* Offset = 0x80000 */
		.size		= 15 * NAND_BLOCK_SIZE,
		.mask_flags	= MTD_WRITEABLE,	/* force read-only */
	},
	{
		.name		= "U-Boot Env",
		.offset		= MTDPART_OFS_APPEND,	/* Offset = 0x260000 */
		.size		= 1 * NAND_BLOCK_SIZE,
	},
	{
		.name		= "Kernel",
		.offset		= MTDPART_OFS_APPEND,	/* Offset = 0x280000 */
		.size		= 32 * NAND_BLOCK_SIZE,
	},
	{
		.name		= "File System",
		.offset		= MTDPART_OFS_APPEND,	/* Offset = 0x680000 */
		.size		= MTDPART_SIZ_FULL,
	},
};

static struct omap_nand_platform_data omap3beagle_nand_data = {
	.options	= NAND_BUSWIDTH_16,
	.parts		= omap3beagle_nand_partitions,
	.nr_parts	= ARRAY_SIZE(omap3beagle_nand_partitions),
	.dma_channel	= -1,		/* disable DMA in OMAP NAND driver */
	.nand_setup	= NULL,
	.dev_ready	= NULL,
};

static struct resource omap3beagle_nand_resource = {
	.flags		= IORESOURCE_MEM,
};

static struct platform_device omap3beagle_nand_device = {
	.name		= "omap2-nand",
	.id		= -1,
	.dev		= {
		.platform_data	= &omap3beagle_nand_data,
	},
	.num_resources	= 1,
	.resource	= &omap3beagle_nand_resource,
};

/* DSS */

static int beagle_enable_dvi(struct omap_dss_device *dssdev)
{
	if (dssdev->reset_gpio != -1)
		gpio_set_value(dssdev->reset_gpio, 1);

	return 0;
}

static void beagle_disable_dvi(struct omap_dss_device *dssdev)
{
	if (dssdev->reset_gpio != -1)
		gpio_set_value(dssdev->reset_gpio, 0);
}

static struct omap_dss_device beagle_dvi_device = {
	.type = OMAP_DISPLAY_TYPE_DPI,
	.name = "dvi",
	.driver_name = "generic_panel",
	.phy.dpi.data_lines = 24,
	.reset_gpio = 170,
	.platform_enable = beagle_enable_dvi,
	.platform_disable = beagle_disable_dvi,
};

static int beagle_panel_enable_tv(struct omap_dss_device *dssdev)
{
#define ENABLE_VDAC_DEDICATED           0x03
#define ENABLE_VDAC_DEV_GRP             0x20

	twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER,
			ENABLE_VDAC_DEDICATED,
			TWL4030_VDAC_DEDICATED);
	twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER,
			ENABLE_VDAC_DEV_GRP, TWL4030_VDAC_DEV_GRP);

	return 0;
}

static void beagle_panel_disable_tv(struct omap_dss_device *dssdev)
{
	twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER, 0x00,
			TWL4030_VDAC_DEDICATED);
	twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER, 0x00,
			TWL4030_VDAC_DEV_GRP);
}

static struct omap_dss_device beagle_tv_device = {
	.name = "tv",
	.driver_name = "venc",
	.type = OMAP_DISPLAY_TYPE_VENC,
	.phy.venc.type = OMAP_DSS_VENC_TYPE_SVIDEO,
	.platform_enable = beagle_panel_enable_tv,
	.platform_disable = beagle_panel_disable_tv,
};

static struct omap_dss_device *beagle_dss_devices[] = {
	&beagle_dvi_device,
	&beagle_tv_device,
};

static struct omap_dss_board_info beagle_dss_data = {
	.num_devices = ARRAY_SIZE(beagle_dss_devices),
	.devices = beagle_dss_devices,
	.default_device = &beagle_dvi_device,
};

static struct platform_device beagle_dss_device = {
	.name          = "omapdss",
	.id            = -1,
	.dev            = {
		.platform_data = &beagle_dss_data,
	},
};

static struct regulator_consumer_supply beagle_vdda_dac_supply = {
	.supply		= "vdda_dac",
	.dev		= &beagle_dss_device.dev,
};

static struct regulator_consumer_supply beagle_vdds_dsi_supply = {
	.supply		= "vdds_dsi",
	.dev		= &beagle_dss_device.dev,
};

static void __init beagle_display_init(void)
{
	int r;

	r = gpio_request(beagle_dvi_device.reset_gpio, "DVI reset");
	if (r < 0) {
		printk(KERN_ERR "Unable to get DVI reset GPIO\n");
		return;
	}

	gpio_direction_output(beagle_dvi_device.reset_gpio, 0);
}

#include "sdram-micron-mt46h32m32lf-6.h"

static struct omap_uart_config omap3_beagle_uart_config __initdata = {
	.enabled_uarts	= ((1 << 0) | (1 << 1) | (1 << 2)),
};

static struct twl4030_usb_data beagle_usb_data = {
	.usb_mode	= T2_USB_MODE_ULPI,
};

static struct twl4030_hsmmc_info mmc[] = {
	{
		.mmc		= 1,
		.wires		= 8,
		.gpio_wp	= 29,
	},
	{
       .mmc        = 2,
       .wires      = 4,
       .gpio_wp    = -EINVAL,
       .ext_clock  = 0,
       .ocr_mask   = MMC_VDD_165_195 | MMC_VDD_30_31 |
						MMC_VDD_32_33   | MMC_VDD_33_34,
	},
	{}	/* Terminator */
};

static struct regulator_consumer_supply beagle_vmmc1_supply = {
	.supply			= "vmmc",
};

static struct regulator_consumer_supply beagle_vsim_supply = {
	.supply			= "vmmc_aux",
};

static struct gpio_led gpio_leds[];

static int beagle_twl_gpio_setup(struct device *dev,
		unsigned gpio, unsigned ngpio)
{
	printk("%s:%d\n", __func__, __LINE__);

	/* gpio + 0 is "mmc0_cd" (input/IRQ) */
	omap_cfg_reg(AH8_34XX_GPIO29);
	mmc[0].gpio_cd = gpio + 0;
	twl4030_mmc_init(mmc);

	/* link regulators to MMC adapters */
	beagle_vmmc1_supply.dev = mmc[0].dev;
	beagle_vsim_supply.dev = mmc[0].dev;

	return 0;
}

static struct twl4030_gpio_platform_data beagle_gpio_data = {
	.gpio_base	= OMAP_MAX_GPIO_LINES,
	.irq_base	= TWL4030_GPIO_IRQ_BASE,
	.irq_end	= TWL4030_GPIO_IRQ_END,
	.use_leds	= true,
	.pullups	= BIT(1),
	.pulldowns	= BIT(2) | BIT(6) | BIT(7) | BIT(8) | BIT(13)
				| BIT(15) | BIT(16) | BIT(17),
	.setup		= beagle_twl_gpio_setup,
};

/* VMMC1 for MMC1 pins CMD, CLK, DAT0..DAT3 (20 mA, plus card == max 220 mA) */
static struct regulator_init_data beagle_vmmc1 = {
	.constraints = {
		.min_uV			= 1850000,
		.max_uV			= 3150000,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask		= REGULATOR_CHANGE_VOLTAGE
					| REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &beagle_vmmc1_supply,
};

/* VSIM for MMC1 pins DAT4..DAT7 (2 mA, plus card == max 50 mA) */
static struct regulator_init_data beagle_vsim = {
	.constraints = {
		.min_uV			= 1800000,
		.max_uV			= 3000000,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask		= REGULATOR_CHANGE_VOLTAGE
					| REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &beagle_vsim_supply,
};

/* VDAC for DSS driving S-Video (8 mA unloaded, max 65 mA) */
static struct regulator_init_data beagle_vdac = {
	.constraints = {
		.min_uV			= 1800000,
		.max_uV			= 1800000,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask		= REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &beagle_vdda_dac_supply,
};

/* VPLL2 for digital video outputs */
static struct regulator_init_data beagle_vpll2 = {
	.constraints = {
		.name			= "VDVI",
		.min_uV			= 1800000,
		.max_uV			= 1800000,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask		= REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &beagle_vdds_dsi_supply,
};

static const struct twl4030_resconfig beagle_resconfig[] = {
	/* disable regulators that u-boot left enabled; the
	 * devices' drivers should be managing these.
	 */
	{ .resource = RES_VAUX3, },	/* not even connected! */
	{ .resource = RES_VMMC1, },
	{ .resource = RES_VSIM, },
	{ .resource = RES_VPLL2, },
	{ .resource = RES_VDAC, },
	{ .resource = RES_VUSB_1V5, },
	{ .resource = RES_VUSB_1V8, },
	{ .resource = RES_VUSB_3V1, },
	{ 0, },
};

static struct twl4030_power_data beagle_power_data = {
	.resource_config	= beagle_resconfig,
	/* REVISIT can't use GENERIC3430_T2SCRIPTS_DATA;
	 * among other things, it makes reboot fail.
	 */
};

static struct twl4030_platform_data beagle_twldata = {
	.irq_base	= TWL4030_IRQ_BASE,
	.irq_end	= TWL4030_IRQ_END,

	/* platform_data for children goes here */
	.usb		= &beagle_usb_data,
	.gpio		= &beagle_gpio_data,
	.power		= &beagle_power_data,
	.vmmc1		= &beagle_vmmc1,
	.vsim		= &beagle_vsim,
	.vdac		= &beagle_vdac,
	.vpll2		= &beagle_vpll2,
};

static struct i2c_board_info __initdata beagle_i2c_boardinfo[] = {
	{
		I2C_BOARD_INFO("twl4030", 0x48),
		.flags = I2C_CLIENT_WAKE,
		.irq = INT_34XX_SYS_NIRQ,
		.platform_data = &beagle_twldata,
	},
};

static int __init omap3_beagle_i2c_init(void)
{
	omap_register_i2c_bus(1, 2600, beagle_i2c_boardinfo,
			ARRAY_SIZE(beagle_i2c_boardinfo));
	/* Bus 3 is attached to the DVI port where devices like the pico DLP
	 * projector don't work reliably with 400kHz */
	omap_register_i2c_bus(3, 100, NULL, 0);
	return 0;
}

#ifdef CONFIG_MACH_OMAP3_DEVKIT8000

#define OMAP_DM9000_GPIO_IRQ    25

static void __init omap_dm9000_init(void)
{
        if (gpio_request(OMAP_DM9000_GPIO_IRQ, "dm9000 irq") < 0) {
                printk(KERN_ERR "Failed to request GPIO%d for dm9000 IRQ\n",
                        OMAP_DM9000_GPIO_IRQ);
                return;
        }

        gpio_direction_input(OMAP_DM9000_GPIO_IRQ);
}
#endif
#ifdef CONFIG_MACH_OMAP3_DEVKIT8000

/* ethernet for dm9000 for devkit 8000 */
#define OMAP_DM9000_BASE        0x2c000000

static struct resource omap_dm9000_resources[] = {
        [0] = {
                .start          = OMAP_DM9000_BASE,
                .end            = (OMAP_DM9000_BASE + 0x4 - 1),
                .flags          = IORESOURCE_MEM,
        },
        [1] = {
                .start          = (OMAP_DM9000_BASE + 0x400),
                .end            = (OMAP_DM9000_BASE + 0x400 + 0x4 - 1),
                .flags          = IORESOURCE_MEM,
        },
        [2] = {
                .start          = OMAP_GPIO_IRQ(OMAP_DM9000_GPIO_IRQ),
                .end            = OMAP_GPIO_IRQ(OMAP_DM9000_GPIO_IRQ),
                .flags          = IORESOURCE_IRQ | IRQF_TRIGGER_LOW,
        },
};

static struct dm9000_plat_data omap_dm9000_platdata = {
        .flags = DM9000_PLATF_16BITONLY,
};

static struct platform_device omap_dm9000_dev = {
        .name = "dm9000",
        .id = -1,
        .num_resources  = ARRAY_SIZE(omap_dm9000_resources),
        .resource       = omap_dm9000_resources,
        .dev = {
                .platform_data = &omap_dm9000_platdata,
        },
};
#endif

#define OMAP3_BEAGLE_TS_GPIO       27

static void ads7846_dev_init(void)
{
    if (gpio_request(OMAP3_BEAGLE_TS_GPIO, "ADS7846 pendown") < 0)
        printk(KERN_ERR "can't get ads7846 pen down GPIO\n");

    gpio_direction_input(OMAP3_BEAGLE_TS_GPIO);

    omap_set_gpio_debounce(OMAP3_BEAGLE_TS_GPIO, 1);
    omap_set_gpio_debounce_time(OMAP3_BEAGLE_TS_GPIO, 0xa);
}

static int ads7846_get_pendown_state(void)
{
    return !gpio_get_value(OMAP3_BEAGLE_TS_GPIO);
}

struct ads7846_platform_data ads7846_conf = {
#ifdef CONFIG_MACH_OMAP3_DEVKIT8000
    .x_min			= 310,
    .y_min			= 217,
    //.x_max			= 3716,
    //.y_max			= 3782,
    .x_max			= 0x0fff,
    .y_max			= 0x0fff,
#else
    .x_max			= 0x0fff,
    .y_max			= 0x0fff,
#endif
    .x_plate_ohms		= 180,
    .pressure_max		= 255,
    .debounce_max          	= 10,
    .debounce_tol          	= 5,
    .debounce_rep          	= 1,
    .get_pendown_state  	= ads7846_get_pendown_state,
    .keep_vref_on       	= 1,
    .settle_delay_usecs		= 150,
};

static struct omap2_mcspi_device_config ads7846_mcspi_config = {
    .turbo_mode = 0,
    .single_channel = 1,  /* 0: slave, 1: master */
};

struct spi_board_info omap3devkit9100_spi_board_info[] = {
    [0] = {
        .modalias       = "ads7846",
        .bus_num        = 2,
        .chip_select        = 0,
        .max_speed_hz       = 1500000,
        .controller_data    = &ads7846_mcspi_config,
        .irq            = OMAP_GPIO_IRQ(OMAP3_BEAGLE_TS_GPIO),
        .platform_data      = &ads7846_conf,
    },
};

static void __init omap3_beagle_init_irq(void)
{
	omap2_init_common_hw(mt46h32m32lf6_sdrc_params, omap3_mpu_rate_table,
			     omap3_dsp_rate_table, omap3_l3_rate_table);
	omap_init_irq();
	omap_gpio_init();
#ifdef CONFIG_MACH_OMAP3_DEVKIT8000
    omap_dm9000_init();
    ads7846_dev_init();
#endif
}

static struct gpio_led gpio_leds[] = {
	{
		.name			= "beagleboard::usr0",
		.default_trigger	= "heartbeat",
		.gpio			= 150,
	},
	{
		.name			= "beagleboard::usr1",
		.default_trigger	= "mmc0",
		.gpio			= 149,
	},
	{
		.name			= "beagleboard::pmu_stat",
		.gpio			= -EINVAL,	/* gets replaced */
		.active_low		= true,
	},
};

static struct gpio_led_platform_data gpio_led_info = {
	.leds		= gpio_leds,
	.num_leds	= ARRAY_SIZE(gpio_leds),
};

static struct platform_device leds_gpio = {
	.name	= "leds-gpio",
	.id	= -1,
	.dev	= {
		.platform_data	= &gpio_led_info,
	},
};

static struct gpio_keys_button gpio_buttons[] = {
	{
		.code			= BTN_EXTRA,
		.gpio			= 7,
		.desc			= "user",
		.wakeup			= 1,
	},
};

static struct gpio_keys_platform_data gpio_key_info = {
	.buttons	= gpio_buttons,
	.nbuttons	= ARRAY_SIZE(gpio_buttons),
};

static struct platform_device keys_gpio = {
	.name	= "gpio-keys",
	.id	= -1,
	.dev	= {
		.platform_data	= &gpio_key_info,
	},
};

static struct platform_device omap3_devkit8000_lcd_device = {
    .name       = "omap3devkit9100_lcd",
    .id     = -1,
};

static struct omap_lcd_config omap3_devkit8000_lcd_config __initdata = {
    .ctrl_name  = "internal",
};

static struct omap_board_config_kernel omap3_beagle_config[] __initdata = {
	{ OMAP_TAG_UART,	&omap3_beagle_uart_config },
#ifdef CONFIG_MACH_OMAP3_DEVKIT8000
    { OMAP_TAG_LCD,     &omap3_devkit8000_lcd_config },
#endif
};

static struct platform_device *omap3_beagle_devices[] __initdata = {
#ifdef CONFIG_MACH_OMAP3_DEVKIT8000
    &omap3_devkit8000_lcd_device,
#else
	&beagle_dss_device,
#endif
	&leds_gpio,
	&keys_gpio,
#ifdef CONFIG_MACH_OMAP3_DEVKIT8000
	&omap_dm9000_dev,
#endif
};

static void __init omap3beagle_flash_init(void)
{
	u8 cs = 0;
	u8 nandcs = GPMC_CS_NUM + 1;

	u32 gpmc_base_add = OMAP34XX_GPMC_VIRT;

	/* find out the chip-select on which NAND exists */
	while (cs < GPMC_CS_NUM) {
		u32 ret = 0;
		ret = gpmc_cs_read_reg(cs, GPMC_CS_CONFIG1);

		if ((ret & 0xC00) == 0x800) {
			printk(KERN_INFO "Found NAND on CS%d\n", cs);
			if (nandcs > GPMC_CS_NUM)
				nandcs = cs;
		}
		cs++;
	}

	if (nandcs > GPMC_CS_NUM) {
		printk(KERN_INFO "NAND: Unable to find configuration "
				 "in GPMC\n ");
		return;
	}

	if (nandcs < GPMC_CS_NUM) {
		omap3beagle_nand_data.cs = nandcs;
		omap3beagle_nand_data.gpmc_cs_baseaddr = (void *)
			(gpmc_base_add + GPMC_CS0_BASE + nandcs * GPMC_CS_SIZE);
		omap3beagle_nand_data.gpmc_baseaddr = (void *) (gpmc_base_add);

		printk(KERN_INFO "Registering NAND on CS%d\n", nandcs);
		if (platform_device_register(&omap3beagle_nand_device) < 0)
			printk(KERN_ERR "Unable to register NAND device\n");
	}
}

static void __init omap3_beagle_init(void)
{
	omap3_beagle_i2c_init();
	platform_add_devices(omap3_beagle_devices,
			ARRAY_SIZE(omap3_beagle_devices));
	omap_board_config = omap3_beagle_config;
	omap_board_config_size = ARRAY_SIZE(omap3_beagle_config);
	omap_serial_init();
#ifdef CONFIG_MACH_OMAP3_DEVKIT8000
    spi_register_board_info(omap3devkit9100_spi_board_info,
                                ARRAY_SIZE(omap3devkit9100_spi_board_info));
#endif
	omap_cfg_reg(J25_34XX_GPIO170);

	usb_musb_init();
	usb_ehci_init();
	omap3beagle_flash_init();
	beagle_display_init();
}

static void __init omap3_beagle_map_io(void)
{
	omap2_set_globals_343x();
	omap2_map_common_io();
}

MACHINE_START(OMAP3_BEAGLE, "OMAP3 Beagle Board")
	/* Maintainer: Syed Mohammed Khasim - http://beagleboard.org */
	.phys_io	= 0x48000000,
	.io_pg_offst	= ((0xd8000000) >> 18) & 0xfffc,
	.boot_params	= 0x80000100,
	.map_io		= omap3_beagle_map_io,
	.init_irq	= omap3_beagle_init_irq,
	.init_machine	= omap3_beagle_init,
	.timer		= &omap_timer,
MACHINE_END
