/*
 * LCD panel support for the TI OMAP3 Beagle board
 *
 * Author: Koen Kooi <koen@openembedded.org>
 *
 * Derived from drivers/video/omap/lcd-omap3evm.c
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/i2c/twl4030.h>

#include <mach/mux.h>
// hychen 08112009
//#include <mach/omapfb.h>
#include <linux/omapfb.h>

#include <asm/mach-types.h>

#define DVI
//#undef DVI

#define LCD_PANEL_ENABLE_GPIO       170
/*
#define LCD_PANEL_ENABLE_GPIO       173
#define LCD_PANEL_LR                2
#define LCD_PANEL_UD                3
#define LCD_PANEL_INI               152
#define LCD_PANEL_QVGA              154
#define LCD_PANEL_RESB              155
*/

#ifdef DVI
//#define LCD_XRES		1024
//#define LCD_YRES 		768
//#define LCD_PIXCLOCK		64000 /* in kHz */

#define LCD_XRES		1280
#define LCD_YRES 		720
#define LCD_PIXCLOCK		74250 /* in kHz */
#else
//#define LCD_XRES		480
//#define LCD_YRES 		640
//#define LCD_PIXCLOCK		26000 /* in kHz */


#define LCD_XRES		480
#define LCD_YRES 		272
#define LCD_PIXCLOCK		10000 /* in kHz */
#endif

static int omap3devkit9100_panel_init(struct lcd_panel *panel,
				struct omapfb_device *fbdev)
{
	printk("%s was invoked\n", __FUNCTION__);
#if 0
	gpio_request(LCD_PANEL_ENABLE_GPIO, "LCD enable");
#else
	twl4030_i2c_write_u8(TWL4030_MODULE_GPIO, 0x80, REG_GPIODATADIR1);//set to output
	twl4030_i2c_write_u8(TWL4030_MODULE_LED, 0x0, 0x0);
#endif
/*
 	gpio_request(LCD_PANEL_LR, "LCD lr");
	gpio_request(LCD_PANEL_UD, "LCD ud");
	gpio_request(LCD_PANEL_INI, "LCD ini");
	gpio_request(LCD_PANEL_RESB, "LCD resb");
	gpio_request(LCD_PANEL_QVGA, "LCD qvga");

	gpio_direction_output(LCD_PANEL_RESB, 1);
	gpio_direction_output(LCD_PANEL_INI, 1);
	gpio_direction_output(LCD_PANEL_QVGA, 0);
	gpio_direction_output(LCD_PANEL_LR, 1);
	gpio_direction_output(LCD_PANEL_UD, 1);

	twl4030_i2c_write_u8(TWL4030_MODULE_LED, 0x11, TWL_LED_LEDEN);
	twl4030_i2c_write_u8(TWL4030_MODULE_PWMA, 0x01, TWL_PWMA_PWMAON);
	twl4030_i2c_write_u8(TWL4030_MODULE_PWMA, 0x02, TWL_PWMA_PWMAOFF);
	*/
	return 0;
}

static void omap3devkit9100_panel_cleanup(struct lcd_panel *panel)
{
}

static int omap3devkit9100_panel_enable(struct lcd_panel *panel)
{
#if 0
	gpio_set_value(LCD_PANEL_ENABLE_GPIO, 1);
#else
	twl4030_i2c_write_u8(TWL4030_MODULE_GPIO, 0x80, REG_GPIODATAOUT1);//output '1'
	twl4030_i2c_write_u8(TWL4030_MODULE_LED, 0x0, 0x0);
#endif
	return 0;
}

static void omap3devkit9100_panel_disable(struct lcd_panel *panel)
{
#if 0
	gpio_set_value(LCD_PANEL_ENABLE_GPIO, 0);
#else
	twl4030_i2c_write_u8(TWL4030_MODULE_GPIO, 0x00, REG_GPIODATAOUT1);
	twl4030_i2c_write_u8(TWL4030_MODULE_LED, 0x33, 0x0);
#endif
}

static unsigned long omap3devkit9100_panel_get_caps(struct lcd_panel *panel)
{
	return 0;
}

struct lcd_panel omap3devkit9100_panel = {
	.name		= "omap3devkit9100",
#ifndef DVI
	.config		= OMAP_LCDC_PANEL_TFT | OMAP_LCDC_INV_VSYNC | OMAP_LCDC_INV_HSYNC,
#else
	.config		= OMAP_LCDC_PANEL_TFT, /*| OMAP_LCDC_HSVS_OPPOSITE*/
	//| OMAP_LCDC_HSVS_RISING_EDGE | OMAP_LCDC_HSVS_OPPOSITE,
#endif

#ifndef DVI
	.bpp		= 16,
	.data_lines	= 24,
	.x_res		= LCD_XRES,
	.y_res		= LCD_YRES,
	.hsw		= 41,		/* hsync_len (4) - 1 */
	.hfp		= 2,		/* right_margin (4) - 1 */
	.hbp		= 2,		/* left_margin (40) - 1 */
	.vsw		= 10,		/* vsync_len (2) - 1 */
	.vfp		= 2,		/* lower_margin */
	.vbp		= 2,		/* upper_margin (8) - 1 */
#else


#if 0	/********  mistral config ***************/
	.bpp		= 16,
	.data_lines	= 18,
	.x_res		= LCD_XRES,
	.y_res		= LCD_YRES,
	.hsw		= 4,		/* hsync_len (4) - 1 */
	.hfp		= 75,		/* right_margin (4) - 1 */
	.hbp		= 65,		/* left_margin (40) - 1 */
	.vsw		= 2,		/* vsync_len (2) - 1 */
	.vfp		= 2,		/* lower_margin */
	.vbp		= 2,		/* upper_margin (8) - 1 */
#endif

	/***********  devkit9000  config *******************/
	.bpp		= 16,
	.data_lines	= 24,
	.x_res		= LCD_XRES,
	.y_res		= LCD_YRES,
	.hsw		= 40,		/* hsync_len (4) - 1 */
	.hfp		= 110,		/* right_margin (4) - 1 */
	.hbp		= 220,		/* left_margin (40) - 1 */
	.vsw		= 5,		/* vsync_len (2) - 1 */
	.vfp		= 20,		/* lower_margin */
	.vbp		= 5,		/* upper_margin (8) - 1 */

#if 0	/***********  devkit9000  config *******************/
	.bpp		= 16,
	.data_lines	= 24,
	.x_res		= LCD_XRES,
	.y_res		= LCD_YRES,
	.hsw		= 0x31,		/* hsync_len (4) - 1 */
	.hfp		= 0x3f,		/* right_margin (4) - 1 */
	.hbp		= 0xff,		/* left_margin (40) - 1 */
	.vsw		= 0x4,		/* vsync_len (2) - 1 */
	.vfp		= 0x5,		/* lower_margin */
	.vbp		= 0x14,		/* upper_margin (8) - 1 */
#endif

#if 0	/***********  devkit9000  config *******************/
	.bpp		= 24,
	.data_lines	= 24,
	.x_res		= LCD_XRES,
	.y_res		= LCD_YRES,
	.hsw		= 0x1,		/* hsync_len (4) - 1 */
	.hfp		= 0x20,		/* right_margin (4) - 1 */
	.hbp		= 0x20,		/* left_margin (40) - 1 */
	.vsw		= 0x1,		/* vsync_len (2) - 1 */
	.vfp		= 0x20,		/* lower_margin */
	.vbp		= 0x20,		/* upper_margin (8) - 1 */
#endif
#endif

	.pixel_clock	= LCD_PIXCLOCK,
	.init		= omap3devkit9100_panel_init,
	.cleanup	= omap3devkit9100_panel_cleanup,
	.enable		= omap3devkit9100_panel_enable,
	.disable	= omap3devkit9100_panel_disable,
	.get_caps	= omap3devkit9100_panel_get_caps,
};


static int omap3devkit9100_panel_probe(struct platform_device *pdev)
{
	omapfb_register_panel(&omap3devkit9100_panel);
	return 0;
}

static int omap3devkit9100_panel_remove(struct platform_device *pdev)
{
	return 0;
}

static int omap3devkit9100_panel_suspend(struct platform_device *pdev,
				   pm_message_t mesg)
{
	return 0;
}

static int omap3devkit9100_panel_resume(struct platform_device *pdev)
{
	return 0;
}

struct platform_driver omap3devkit9100_panel_driver = {
	.probe		= omap3devkit9100_panel_probe,
	.remove		= omap3devkit9100_panel_remove,
	.suspend	= omap3devkit9100_panel_suspend,
	.resume		= omap3devkit9100_panel_resume,
	.driver		= {
		.name	= "omap3devkit9100_lcd",
		.owner	= THIS_MODULE,
	},
};

static int __init omap3devkit9100_panel_drv_init(void)
{
	return platform_driver_register(&omap3devkit9100_panel_driver);
}

static void __exit omap3devkit9100_panel_drv_exit(void)
{
	platform_driver_unregister(&omap3devkit9100_panel_driver);
}

module_init(omap3devkit9100_panel_drv_init);
module_exit(omap3devkit9100_panel_drv_exit);
