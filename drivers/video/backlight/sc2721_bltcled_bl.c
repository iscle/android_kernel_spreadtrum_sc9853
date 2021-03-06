/*
 * Copyright (C) 2017 Spreadtrum Communications Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/backlight.h>
#include <linux/err.h>
#include <linux/fb.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/mfd/sprd/sc2721_glb.h>

/*#define BACKLIGHT_BLTC_DEBUG*/

#ifdef pr_fmt
#undef pr_fmt
#define pr_fmt(__fmt) "[sprd-sc2721-backlight][%20s] "__fmt, __func__
#endif

/*Breathing light control:0xXXXX_X180 ~ 0xXXXX_X19F*/
#define REG_BLTC_BASE		0x180
#define REG_BLTC_R_PRESCL	(REG_BLTC_BASE + 0x004)
#define REG_BLTC_R_DUTY		(REG_BLTC_BASE + 0x008)
#define REG_BLTC_R_CURVE0	(REG_BLTC_BASE + 0x00C)
#define REG_BLTC_R_CURVE1	(REG_BLTC_BASE + 0x010)
#define REG_BLTC_G_PRESCL	(REG_BLTC_BASE + 0x014)
#define REG_BLTC_G_DUTY		(REG_BLTC_BASE + 0x018)
#define REG_BLTC_G_CURVE0	(REG_BLTC_BASE + 0x01C)
#define REG_BLTC_G_CURVE1	(REG_BLTC_BASE + 0x020)
#define REG_BLTC_B_PRESCL	(REG_BLTC_BASE + 0x024)
#define REG_BLTC_B_DUTY		(REG_BLTC_BASE + 0x028)
#define REG_BLTC_B_CURVE0	(REG_BLTC_BASE + 0x02C)
#define REG_BLTC_B_CURVE1	(REG_BLTC_BASE + 0x030)
#define REG_BLTC_STS		(REG_BLTC_BASE + 0x034)

struct sprd_bl_devdata {
	int suspend;
	int brightness_max;
	int brightness_min;
	unsigned int led_level;
	unsigned int ib_trim_cal_data;
	struct clk *clk;
	struct backlight_device *bldev;
};

static struct sprd_bl_devdata sprdbl = {
	.brightness_max = 255,
	.brightness_min = 0,
};

static struct regmap *handle;


#ifdef BACKLIGHT_BLTC_DEBUG
static void sprd_bltc_whtled_get_brightness(void)
{
	int module_en, rgb_ctrl, bltc_base, rtc_clk_en;
	int r_duty, g_duty, b_duty;

	regmap_read(handle, ANA_REG_GLB_MODULE_EN0, &module_en);
	regmap_read(handle, ANA_REG_GLB_RGB_CTRL, &rgb_ctrl);
	regmap_read(handle, REG_BLTC_BASE, &bltc_base);
	regmap_read(handle, ANA_REG_GLB_RTC_CLK_EN0, &rtc_clk_en);
	regmap_read(handle, REG_BLTC_R_DUTY, &r_duty);
	regmap_read(handle, REG_BLTC_G_DUTY, &g_duty);
	regmap_read(handle, REG_BLTC_B_DUTY, &b_duty);

	pr_info("GLB_MODULE_EN0 = %x, GLB_RTC_CLK_EN0 = %x\n",
			module_en, rtc_clk_en);
	pr_info("BLTC_BASE = %x, GLB_RGB_CTRL = %x\n",
			bltc_base, rgb_ctrl);
	pr_info("BLTC: R_DUTY = %x, G_DUTY = %x, B_DUTY = %x\n",
			r_duty, g_duty, b_duty);
}
#endif

static void sprd_bltc_whtled_off(void)
{
	int cnt = 3;
	int bltc_base = 0;

	do {
		regmap_write(handle, REG_BLTC_BASE, 0x0);

		regmap_write(handle, REG_BLTC_R_DUTY, 0x0);
		regmap_write(handle, REG_BLTC_G_DUTY, 0x0);
		regmap_write(handle, REG_BLTC_B_DUTY, 0x0);

		regmap_write(handle, REG_BLTC_R_CURVE0, 0x0);
		regmap_write(handle, REG_BLTC_G_CURVE0, 0x0);
		regmap_write(handle, REG_BLTC_B_CURVE0, 0x0);

		regmap_write(handle, REG_BLTC_R_CURVE1, 0x0);
		regmap_write(handle, REG_BLTC_G_CURVE1, 0x0);
		regmap_write(handle, REG_BLTC_B_CURVE1, 0x0);

		regmap_read(handle, REG_BLTC_BASE, &bltc_base);
	} while (bltc_base && (--cnt));

	regmap_update_bits(handle, ANA_REG_GLB_RGB_CTRL,
		BIT_SLP_RGB_PD_EN | BIT_RGB_PD_SW,
		BIT_SLP_RGB_PD_EN | BIT_RGB_PD_SW);

	regmap_update_bits(handle, ANA_REG_GLB_RTC_CLK_EN0,
		BIT_RTC_BLTC_EN, ~BIT_RTC_BLTC_EN);

	regmap_update_bits(handle, ANA_REG_GLB_MODULE_EN0,
		BIT_BLTC_EN, ~BIT_BLTC_EN);

#ifdef BACKLIGHT_BLTC_DEBUG
	pr_info("cnt = %d\n", cnt);
	sprd_bltc_whtled_get_brightness();
#endif
}


static void sprd_bltc_whtled_set_brightness(u32 level)
{
	int duty = (level << 8) | 0xFF;

	regmap_update_bits(handle, ANA_REG_GLB_MODULE_EN0,
		BIT_BLTC_EN, BIT_BLTC_EN);

	regmap_update_bits(handle, ANA_REG_GLB_RTC_CLK_EN0,
		BIT_RTC_BLTC_EN, BIT_RTC_BLTC_EN);

	regmap_update_bits(handle, ANA_REG_GLB_RGB_CTRL,
		BIT_SLP_RGB_PD_EN | BIT_RGB_PD_SW,
		~(BIT_SLP_RGB_PD_EN | BIT_RGB_PD_SW));

	regmap_write(handle, REG_BLTC_BASE, 0x333);

	/*SET BLTC DUTY(duty_counter + mod_counter)*/
	regmap_write(handle, REG_BLTC_R_DUTY, duty);
	regmap_write(handle, REG_BLTC_G_DUTY, duty);
	regmap_write(handle, REG_BLTC_B_DUTY, duty);

	/*SET BLTC Output RISE/FALL Time*/
	regmap_write(handle, REG_BLTC_R_CURVE0, 0x3F3F);
	regmap_write(handle, REG_BLTC_G_CURVE0, 0x3F3F);
	regmap_write(handle, REG_BLTC_B_CURVE0, 0x3F3F);

	/*SET BLTC Output HIGH/LOW Time*/
	regmap_write(handle, REG_BLTC_R_CURVE1, 0xFFFF);
	regmap_write(handle, REG_BLTC_G_CURVE1, 0xFFFF);
	regmap_write(handle, REG_BLTC_B_CURVE1, 0xFFFF);

#ifdef BACKLIGHT_BLTC_DEBUG
	sprd_bltc_whtled_get_brightness();
#endif
}


static int sprd_bl_whtled_update_status(struct backlight_device *bldev)
{
	u32 led_level;

	if ((bldev->props.state & (BL_CORE_SUSPENDED | BL_CORE_FBBLANK)) ||
		bldev->props.power != FB_BLANK_UNBLANK || sprdbl.suspend ||
		bldev->props.brightness == 0) {
		/* disable backlight */
		sprd_bltc_whtled_off();
		pr_info("disabled\n");
	} else {
		led_level = bldev->props.brightness & 0xff;
		sprdbl.led_level = led_level;
		sprd_bltc_whtled_set_brightness(led_level);
		pr_info("brightness = %d, led_level = %d\n",
			bldev->props.brightness, led_level);
	}

	return 0;
}


static int sprd_bl_whtled_get_brightness(struct backlight_device *bldev)
{
	return sprdbl.led_level;
}


static const struct backlight_ops sprd_backlight_whtled_ops = {
	.update_status = sprd_bl_whtled_update_status,
	.get_brightness = sprd_bl_whtled_get_brightness,
};


static int whtled_backlight_probe(struct platform_device *pdev)
{
	struct backlight_properties props;
	struct backlight_device *bldev;
	const char *backlight_name = "sprd_backlight";

	memset(&props, 0, sizeof(struct backlight_properties));
	props.max_brightness = 0xff;
	props.type = BACKLIGHT_RAW;
	/*the default brightness = 1/2 max brightness*/
	props.brightness = 0xff >> 1;
	props.power = FB_BLANK_UNBLANK;

	bldev = backlight_device_register(
		backlight_name, &pdev->dev,
		&sprdbl, &sprd_backlight_whtled_ops, &props);
	if (IS_ERR(bldev)) {
		pr_err("Failed to register backlight device\n");
		return -ENOMEM;
	}

	sprdbl.bldev = bldev;
	platform_set_drvdata(pdev, bldev);

	handle = dev_get_regmap(pdev->dev.parent, NULL);
	if (!handle) {
		pr_err("Unable to get regmap\n");
		return -EINVAL;
	}

	/*SET BLTC prescale coefficient*/
	regmap_write(handle, REG_BLTC_R_PRESCL, 0x0);
	regmap_write(handle, REG_BLTC_G_PRESCL, 0x0);
	regmap_write(handle, REG_BLTC_B_PRESCL, 0x0);

	/*USE MAX CURRENT 85.71mA= {(1.69+0.84*32)*3}*/
	/*AS THE backlight DEFAULT OUTPUT CURRENT*/
	regmap_update_bits(handle, ANA_REG_GLB_RGB_CTRL,
		BITS_RGB_V(0x1F), BITS_RGB_V(0x1F));

	bldev->ops->update_status(bldev);

	pr_info("bltc backlight probe success!\n");

	return 0;
}


static int whtled_backlight_remove(struct platform_device *pdev)
{
	struct backlight_device *bldev;

	bldev = platform_get_drvdata(pdev);

	bldev->props.power = FB_BLANK_UNBLANK;
	bldev->props.brightness = 0;

	backlight_update_status(bldev);

	backlight_device_unregister(bldev);

	platform_set_drvdata(pdev, NULL);

	return 0;
}


static void whtled_backlight_shutdown(struct platform_device *pdev)
{
	struct backlight_device *bldev;

	bldev = platform_get_drvdata(pdev);

	bldev->props.brightness = 0;

	backlight_update_status(bldev);
}


#ifdef CONFIG_PM_SLEEP
static int whtled_backlight_suspend(struct device *pdev)
{
	sprd_bltc_whtled_off();

	return 0;
}

static int whtled_backlight_resume(struct device *pdev)
{
	struct backlight_device *bldev;

	bldev = dev_get_drvdata(pdev);

	backlight_update_status(bldev);
	return 0;
}
#endif

static const struct dev_pm_ops whtled_backlight_pm_ops = {
#ifdef CONFIG_PM_SLEEP
	.suspend = whtled_backlight_suspend,
	.resume = whtled_backlight_resume,
	.poweroff = whtled_backlight_suspend,
	.restore = whtled_backlight_resume,
#endif
};

static const struct of_device_id whtled_of_match[] = {
	{ .compatible = "sprd,sc2721-bltc-rgb", },
	{ }
};

static struct platform_driver whtled_backlight_driver = {
	.probe = whtled_backlight_probe,
	.remove = whtled_backlight_remove,
	.shutdown = whtled_backlight_shutdown,
	.driver = {
		.name = "bltc-whtled-backlight",
		.pm = &whtled_backlight_pm_ops,
		.of_match_table = whtled_of_match,
	},
};


module_platform_driver(whtled_backlight_driver);

MODULE_DESCRIPTION("Spreadtrum backlight Driver");
MODULE_AUTHOR("dong1.wang@spreadtrum.com");
MODULE_LICENSE("GPL");
