/*
 * arch/arm/mach-tegra/board-ast-panel.c
 *
 * Copyright (c) 2010-2012, NVIDIA Corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include <linux/delay.h>
#include <linux/ion.h>
#include <linux/tegra_ion.h>
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/resource.h>
#include <asm/mach-types.h>
#include <linux/platform_device.h>
#include <linux/earlysuspend.h>
#include <linux/pwm_backlight.h>
#include <linux/charging-img.h>
#include <asm/atomic.h>
#include <linux/nvhost.h>
#include <linux/nvmap.h>
#include <mach/irqs.h>
#include <mach/iomap.h>
#include <mach/dc.h>
#include <mach/fb.h>
#include <mach/smmu.h>

#include "board.h"
#include "board-ast.h"
#include "devices.h"
#include "gpio-names.h"
#include "tegra3_host1x_devices.h"

#define BRIGHTNESS_MAX 255
#define LCD_ON_OFF_TIME_MIN 500

static struct mutex lcd_lock;
static unsigned int lcd_off_timestamp;

static struct regulator *ast_lvds_vdd_bl = NULL;
static struct regulator *ast_lvds_vdd_panel = NULL;

static struct regulator *ast_hdmi_reg = NULL;
static struct regulator *ast_hdmi_pll = NULL;
static struct regulator *ast_hdmi_vddio = NULL;

static atomic_t sd_brightness = ATOMIC_INIT(BRIGHTNESS_MAX);

static tegra_dc_bl_output ast_bl_output_measured = {
#ifdef CONFIG_MACH_AVALON
  0,   1,   2,   3,   4,   6,   7,   8,   9,  10,
 11,  12,  13,  14,  15,  16,  17,  18,  19,  20,
 21,  22,  23,  24,  25,  26,  26,  27,  28,  29,
 30,  31,  32,  33,  34,  35,  35,  36,  37,  38,
 39,  40,  41,  42,  43,  45,  46,  47,  48,  49,
 50,  51,  52,  53,  54,  55,  56,  57,  58,  59,
 60,  61,  62,  63,  64,  65,  65,  66,  67,  68,
 69,  70,  71,  72,  73,  75,  76,  77,  78,  79,
 80,  81,  82,  83,  84,  85,  85,  86,  87,  88,
 89,  90,  91,  92,  93,  94,  94,  95,  96,  97,
 98,  99, 100, 100, 101, 102, 103, 104, 104, 105,
106, 107, 108, 110, 111, 112, 113, 114, 116, 117,
118, 119, 120, 121, 122, 124, 125, 126, 127, 128,
129, 130, 131, 132, 133, 135, 136, 137, 138, 139,
140, 141, 142, 143, 144, 146, 147, 148, 149, 150,
151, 152, 153, 154, 155, 157, 158, 159, 160, 161,
162, 163, 164, 165, 166, 167, 167, 168, 169, 170,
171, 172, 173, 175, 176, 177, 178, 179, 181, 182,
183, 184, 184, 185, 186, 187, 187, 188, 189, 189,
190, 191, 192, 193, 194, 195, 195, 196, 197, 198,
199, 200, 201, 203, 204, 205, 206, 207, 209, 210,
211, 212, 212, 213, 214, 215, 215, 216, 217, 217,
218, 219, 220, 221, 222, 224, 225, 226, 227, 228,
229, 230, 231, 232, 233, 234, 235, 236, 237, 238,
239, 240, 241, 243, 244, 245, 246, 247, 249, 250,
251, 252, 253, 253, 254, 255
#endif
#ifdef CONFIG_MACH_TITAN
  0,   1,   2,   3,   4,   6,   7,   8,   9,  10,
 11,  12,  13,  14,  15,  17,  18,  19,  20,  21,
 22,  23,  24,  25,  26,  27,  27,  28,  29,  30,
 31,  32,  33,  34,  35,  36,  37,  38,  39,  40,
 41,  42,  43,  44,  45,  46,  47,  48,  49,  50,
 51,  52,  53,  54,  55,  56,  56,  57,  58,  59,
 60,  61,  62,  63,  64,  66,  67,  68,  69,  70,
 71,  72,  73,  74,  75,  76,  76,  77,  78,  79,
 80,  81,  82,  83,  84,  85,  86,  87,  88,  89,
 90,  91,  92,  93,  94,  95,  96,  97,  98,  99,
100, 101, 102, 103, 104, 105, 105, 106, 107, 108,
109, 110, 111, 112, 113, 114, 115, 116, 117, 118,
119, 120, 121, 122, 123, 125, 126, 127, 128, 129,
130, 131, 132, 133, 134, 135, 135, 136, 137, 138,
139, 140, 141, 142, 143, 144, 145, 146, 147, 148,
149, 150, 151, 152, 153, 155, 156, 157, 158, 159,
160, 161, 162, 163, 164, 166, 167, 168, 169, 170,
171, 172, 173, 174, 175, 176, 177, 178, 179, 180,
181, 182, 183, 184, 185, 186, 187, 188, 189, 190,
191, 192, 193, 194, 195, 196, 197, 198, 199, 200,
201, 202, 203, 204, 205, 206, 207, 208, 209, 210,
211, 212, 213, 214, 215, 216, 217, 218, 219, 220,
221, 222, 223, 223, 224, 225, 226, 227, 227, 228,
229, 230, 231, 232, 233, 235, 236, 237, 238, 239,
240, 241, 242, 243, 244, 245, 246, 247, 248, 249,
250, 251, 252, 253, 254, 255
#endif
};

static p_tegra_dc_bl_output bl_output;

static int ast_panel_power_on(void)
{
	if (gpio_get_value(LVDS_SHUTDOWN_GPIO) == 1 ||
		mutex_lock_interruptible(&lcd_lock))
		return 0;

	if (ast_lvds_vdd_panel && ast_lvds_vdd_bl) {
		unsigned int lcd_on_off_time =
			jiffies_to_msecs(jiffies) - lcd_off_timestamp;

		if (lcd_on_off_time < LCD_ON_OFF_TIME_MIN &&
			lcd_on_off_time > 0)
			msleep(LCD_ON_OFF_TIME_MIN - lcd_on_off_time);

		regulator_enable(ast_lvds_vdd_panel);
		regulator_enable(ast_lvds_vdd_bl);
		msleep(1);
	}

	gpio_set_value(LVDS_SHUTDOWN_GPIO, 1);
	mutex_unlock(&lcd_lock);

	return 0;
}

static int ast_panel_power_off(void)
{
	if (gpio_get_value(LVDS_SHUTDOWN_GPIO) == 0 ||
		mutex_lock_interruptible(&lcd_lock))
		return 0;

	gpio_set_value(LVDS_SHUTDOWN_GPIO, 0);
	if (ast_lvds_vdd_panel && ast_lvds_vdd_bl) {
		msleep(1);
		regulator_disable(ast_lvds_vdd_bl);
		regulator_disable(ast_lvds_vdd_panel);
		lcd_off_timestamp = jiffies_to_msecs(jiffies);
	}

	mutex_unlock(&lcd_lock);

	return 0;
}

static int ast_panel_enable(void)
{
	if (!ast_lvds_vdd_panel) {
		ast_lvds_vdd_panel = regulator_get(NULL, "vdd_lcd_panel");
		if (WARN_ON(IS_ERR(ast_lvds_vdd_panel))) {
			pr_err("%s: couldn't get regulator vdd_lcd_panel: %ld\n",
			       __func__, PTR_ERR(ast_lvds_vdd_panel));
			ast_lvds_vdd_panel = NULL;
			return -ENODEV;
		}

		regulator_enable(ast_lvds_vdd_panel);
	}

	if (!ast_lvds_vdd_bl) {
		ast_lvds_vdd_bl = regulator_get(NULL, "vdd_backlight");
		if (WARN_ON(IS_ERR(ast_lvds_vdd_bl))) {
			pr_err("%s: couldn't get regulator vdd_backlight: %ld\n",
			       __func__, PTR_ERR(ast_lvds_vdd_bl));
			ast_lvds_vdd_bl = NULL;
			return -ENODEV;
		}

		regulator_enable(ast_lvds_vdd_bl);
	}

	return 0;
}

static int ast_backlight_init(struct device *dev) {
	int err;

	bl_output = ast_bl_output_measured;
	if (WARN_ON(ARRAY_SIZE(ast_bl_output_measured) != BRIGHTNESS_MAX + 1))
		pr_err("bl_output array does not have 256 elements\n");

	tegra_gpio_disable(BL_PWM_GPIO);
	err = gpio_request(BL_ENB_GPIO, "backlight_enb");
	if (err < 0)
		return err;

	gpio_direction_output(BL_ENB_GPIO, 1);
	tegra_gpio_enable(BL_ENB_GPIO);

	return err;
};

static void ast_backlight_enable(int enable)
{
	if (gpio_get_value(BL_ENB_GPIO) == enable)
		return;

	if (enable)
		ast_panel_power_on();

	if (mutex_lock_interruptible(&lcd_lock))
		return;

	if (enable)
		msleep(200);

	gpio_set_value(BL_ENB_GPIO, enable);
	if (!enable)
		msleep(200);

	mutex_unlock(&lcd_lock);
	if (!enable)
		ast_panel_power_off();
}

static int ast_backlight_notify(struct device *unused, int brightness)
{
	int cur_sd_brightness = atomic_read(&sd_brightness);

	/* SD brightness is a percentage, 8-bit value. */
	brightness = (brightness * cur_sd_brightness) / BRIGHTNESS_MAX;

	/* Apply any backlight response curve */
	if (brightness > BRIGHTNESS_MAX || brightness < 0)
		pr_info("Error: Brightness > %d! or Brightness < 0\n", BRIGHTNESS_MAX);
	else
		brightness = bl_output[brightness];

	return brightness;
}

static void ast_backlight_exit(struct device *dev) {
	ast_backlight_enable(0);
	gpio_free(BL_ENB_GPIO);
	tegra_gpio_disable(BL_ENB_GPIO);
}

static int ast_disp1_check_fb(struct device *dev, struct fb_info *info);

static struct platform_pwm_backlight_data ast_backlight_data = {
	.pwm_id = 0,
	.max_brightness = BRIGHTNESS_MAX,
	.dft_brightness = 224,
	.pwm_period_ns = 1000000,
	.init = ast_backlight_init,
	.exit = ast_backlight_exit,
	.notify = ast_backlight_notify,
	/* Only toggle backlight on fb blank notifications for disp1 */
	.check_fb = ast_disp1_check_fb,
	.backlight_enable = ast_backlight_enable,
};

static struct platform_device ast_backlight_device = {
	.name = "pwm-backlight",
	.id = -1,
	.dev = {
		.platform_data = &ast_backlight_data,
	},
};

static int ast_hdmi_vddio_enable(void)
{
	int ret;

	if (!ast_hdmi_vddio) {
		ast_hdmi_vddio = regulator_get(NULL, "vdd_hdmi_con");
		if (IS_ERR_OR_NULL(ast_hdmi_vddio)) {
			ret = PTR_ERR(ast_hdmi_vddio);
			pr_err("hdmi: couldn't get regulator vdd_hdmi_con\n");
			ast_hdmi_vddio = NULL;
			return ret;
		}
	}

	ret = regulator_enable(ast_hdmi_vddio);
	if (ret < 0) {
		pr_err("hdmi: couldn't enable regulator vdd_hdmi_con\n");
		regulator_put(ast_hdmi_vddio);
		ast_hdmi_vddio = NULL;
		return ret;
	}

	return ret;
}

static int ast_hdmi_vddio_disable(void)
{
	if (ast_hdmi_vddio) {
		regulator_disable(ast_hdmi_vddio);
		regulator_put(ast_hdmi_vddio);
		ast_hdmi_vddio = NULL;
	}

	return 0;
}

static int ast_hdmi_enable(void)
{
	int ret;

	if (!ast_hdmi_reg) {
		ast_hdmi_reg = regulator_get(NULL, "avdd_hdmi");
		if (IS_ERR_OR_NULL(ast_hdmi_reg)) {
			pr_err("hdmi: couldn't get regulator avdd_hdmi\n");
			ast_hdmi_reg = NULL;
			return PTR_ERR(ast_hdmi_reg);
		}
	}

	ret = regulator_enable(ast_hdmi_reg);
	if (ret < 0) {
		pr_err("hdmi: couldn't enable regulator avdd_hdmi\n");
		return ret;
	}

	if (!ast_hdmi_pll) {
		ast_hdmi_pll = regulator_get(NULL, "avdd_hdmi_pll");
		if (IS_ERR_OR_NULL(ast_hdmi_pll)) {
			pr_err("hdmi: couldn't get regulator avdd_hdmi_pll\n");
			ast_hdmi_pll = NULL;
			regulator_put(ast_hdmi_reg);
			ast_hdmi_reg = NULL;
			return PTR_ERR(ast_hdmi_pll);
		}
	}

	ret = regulator_enable(ast_hdmi_pll);
	if (ret < 0) {
		pr_err("hdmi: couldn't enable regulator avdd_hdmi_pll\n");
		return ret;
	}

	return 0;
}

static int ast_hdmi_disable(void)
{
	regulator_disable(ast_hdmi_reg);
	regulator_put(ast_hdmi_reg);
	ast_hdmi_reg = NULL;

	regulator_disable(ast_hdmi_pll);
	regulator_put(ast_hdmi_pll);
	ast_hdmi_pll = NULL;

	return 0;
}

static struct resource ast_disp1_resources[] = {
	{
		.name = "irq",
		.start = INT_DISPLAY_GENERAL,
		.end = INT_DISPLAY_GENERAL,
		.flags = IORESOURCE_IRQ,
	},
	{
		.name = "regs",
		.start = TEGRA_DISPLAY_BASE,
		.end = TEGRA_DISPLAY_BASE + TEGRA_DISPLAY_SIZE-1,
		.flags = IORESOURCE_MEM,
	},
	{
		.name = "fbmem",
		.start = 0, /* Filled in by ast_panel_init() */
		.end = 0,   /* Filled in by ast_panel_init() */
		.flags = IORESOURCE_MEM,
	},
};

static struct resource ast_disp2_resources[] = {
	{
		.name = "irq",
		.start = INT_DISPLAY_B_GENERAL,
		.end = INT_DISPLAY_B_GENERAL,
		.flags = IORESOURCE_IRQ,
	},
	{
		.name = "regs",
		.start = TEGRA_DISPLAY2_BASE,
		.end = TEGRA_DISPLAY2_BASE + TEGRA_DISPLAY2_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
	{
		.name = "fbmem",
		.flags = IORESOURCE_MEM,
		.start = 0,
		.end = 0,
	},
	{
		.name = "hdmi_regs",
		.start = TEGRA_HDMI_BASE,
		.end = TEGRA_HDMI_BASE + TEGRA_HDMI_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
};

#ifdef CONFIG_MACH_AVALON
static struct tegra_dc_mode avalon_panel_modes[] = {
	{
		/* 1280x800@60Hz */
		/* .pclk = 102000000, */
		.pclk = 74181818,
		.h_ref_to_sync = 0,
		.v_ref_to_sync = 1,
		.h_sync_width = 48,
		.v_sync_width = 3,
		.h_back_porch = 96,
		.v_back_porch = 19,
		.h_active = 1280,
		.v_active = 800,
		.h_front_porch = 16,
		.v_front_porch = 2,
	},
};

static struct tegra_fb_data avalon_fb_data = {
	.win = 0,
	.xres = 1280,
	.yres = 800,
	.bits_per_pixel = 32,
	.flags = TEGRA_FB_FLIP_ON_PROBE,
};

static struct tegra_fb_data avalon_hdmi_fb_data = {
	.win = 0,
	.xres = 1280,
	.yres = 800,
	.bits_per_pixel = 32,
	.flags = TEGRA_FB_FLIP_ON_PROBE,
};
#endif

#ifdef CONFIG_MACH_TITAN
static struct tegra_fb_data titan_fb_data = {
	.win = 0,
	.xres = 1600,
	.yres = 900,
	.bits_per_pixel = 32,
	.flags = TEGRA_FB_FLIP_ON_PROBE,
};

static struct tegra_fb_data titan_hdmi_fb_data = {
	.win = 0,
	.xres = 1600,
	.yres = 900,
	.bits_per_pixel = 32,
	.flags = TEGRA_FB_FLIP_ON_PROBE,
};

static struct tegra_dc_mode titan_panel_modes[] = {
	{
		/* 1600x900@60Hz */
		.pclk = 102000000,
		.h_ref_to_sync = 0,
		.v_ref_to_sync = 1,
		.h_sync_width = 48,
		.v_sync_width = 3,
		.h_back_porch = 88,
		.v_back_porch = 7,
		.h_active = 1600,
		.v_active = 900,
		.h_front_porch = 48,
		.v_front_porch = 2,
	},
};
#endif

static struct tegra_dc_sd_settings ast_sd_settings = {
	.enable = 1, /* enabled by default. */
	.use_auto_pwm = false,
	.hw_update_delay = 0,
	.bin_width = -1,
	.aggressiveness = 1,
	.phase_in_adjustments = false,
	.use_vid_luma = false,
	/* Default video coefficients */
	.coeff = {5, 9, 2},
	.fc = {0, 0},
	/* Immediate backlight changes */
	.blp = {1024, 255},
	/* Gammas: R: 2.2 G: 2.2 B: 2.2 */
	/* Default BL TF */
	.bltf = {
			{
				{57, 65, 74, 83},
				{93, 103, 114, 126},
				{138, 151, 165, 179},
				{194, 209, 225, 242},
			},
			{
				{58, 66, 75, 84},
				{94, 105, 116, 127},
				{140, 153, 166, 181},
				{196, 211, 227, 244},
			},
			{
				{60, 68, 77, 87},
				{97, 107, 119, 130},
				{143, 156, 170, 184},
				{199, 215, 231, 248},
			},
			{
				{64, 73, 82, 91},
				{102, 113, 124, 137},
				{149, 163, 177, 192},
				{207, 223, 240, 255},
			},
		},
	/* Default LUT */
	.lut = {
			{
				{250, 250, 250},
				{194, 194, 194},
				{149, 149, 149},
				{113, 113, 113},
				{82, 82, 82},
				{56, 56, 56},
				{34, 34, 34},
				{15, 15, 15},
				{0, 0, 0},
			},
			{
				{246, 246, 246},
				{191, 191, 191},
				{147, 147, 147},
				{111, 111, 111},
				{80, 80, 80},
				{55, 55, 55},
				{33, 33, 33},
				{14, 14, 14},
				{0, 0, 0},
			},
			{
				{239, 239, 239},
				{185, 185, 185},
				{142, 142, 142},
				{107, 107, 107},
				{77, 77, 77},
				{52, 52, 52},
				{30, 30, 30},
				{12, 12, 12},
				{0, 0, 0},
			},
			{
				{224, 224, 224},
				{173, 173, 173},
				{133, 133, 133},
				{99, 99, 99},
				{70, 70, 70},
				{46, 46, 46},
				{25, 25, 25},
				{7, 7, 7},
				{0, 0, 0},
			},
		},
	.sd_brightness = &sd_brightness,
	.bl_device = &ast_backlight_device,
};

static struct tegra_dc_out ast_disp1_out = {
	.align = TEGRA_DC_ALIGN_MSB,
	.order = TEGRA_DC_ORDER_RED_BLUE,
	.dcc_bus = 0,
	.sd_settings = &ast_sd_settings,
	.parent_clk = "pll_p",
	.parent_clk_backup = "pll_d2_out0",
	.type = TEGRA_DC_OUT_RGB,
	.depth = 18,
	.dither = TEGRA_DC_ORDERED_DITHER,
#ifdef CONFIG_MACH_AVALON
	.modes = avalon_panel_modes,
	.n_modes = ARRAY_SIZE(avalon_panel_modes),
#endif
#ifdef CONFIG_MACH_TITAN
	.modes = titan_panel_modes,
	.n_modes = ARRAY_SIZE(titan_panel_modes),
#endif
	.enable = ast_panel_enable,
	.postpoweron = ast_panel_power_on,
};

static struct tegra_dc_platform_data ast_disp1_pdata = {
	.flags = TEGRA_DC_FLAG_ENABLED,
	.default_out = &ast_disp1_out,
	.emc_clk_rate = 300000000,
#ifdef CONFIG_MACH_AVALON
	.fb = &avalon_fb_data,
#endif
#ifdef CONFIG_MACH_TITAN
	.fb = &titan_fb_data,
#endif
};

static struct nvhost_device ast_disp1_device = {
	.name = "tegradc",
	.id = 0,
	.resource = ast_disp1_resources,
	.num_resources = ARRAY_SIZE(ast_disp1_resources),
	.dev = {
		.platform_data = &ast_disp1_pdata,
	},
};

static int ast_disp1_check_fb(struct device *dev, struct fb_info *info)
{
	return info->device == &ast_disp1_device.dev;
}

static struct tegra_dc_out ast_disp2_out = {
	.type = TEGRA_DC_OUT_HDMI,
	.flags = TEGRA_DC_OUT_HOTPLUG_HIGH,
	.parent_clk = "pll_d2_out0",
	.dcc_bus = 3,
	.hotplug_gpio = HDMI_HPD_GPIO,
	.max_pixclock = KHZ2PICOS(148500),
	.align = TEGRA_DC_ALIGN_MSB,
	.order = TEGRA_DC_ORDER_RED_BLUE,
	.enable = ast_hdmi_enable,
	.disable = ast_hdmi_disable,
	.postsuspend = ast_hdmi_vddio_disable,
	.hotplug_init = ast_hdmi_vddio_enable,
};

static struct tegra_dc_platform_data ast_disp2_pdata = {
	.flags = 0,
	.default_out = &ast_disp2_out,
#ifdef CONFIG_MACH_AVALON
	.fb = &avalon_hdmi_fb_data,
#endif
#ifdef CONFIG_MACH_TITAN
	.fb = &titan_hdmi_fb_data,
#endif
	.emc_clk_rate = 300000000,
};

static struct nvhost_device ast_disp2_device = {
	.name = "tegradc",
	.id = 1,
	.resource = ast_disp2_resources,
	.num_resources = ARRAY_SIZE(ast_disp2_resources),
	.dev = {
		.platform_data = &ast_disp2_pdata,
	},
};

#if defined(CONFIG_TEGRA_NVMAP)
static struct nvmap_platform_carveout ast_carveouts[] = {
	[0] = NVMAP_HEAP_CARVEOUT_IRAM_INIT,
	[1] = {
		.name = "generic-0",
		.usage_mask = NVMAP_HEAP_CARVEOUT_GENERIC,
		.base = 0, /* Filled in by ast_panel_init() */
		.size = 0, /* Filled in by ast_panel_init() */
		.buddy_size = SZ_32K,
	},
};

static struct nvmap_platform_data ast_nvmap_data = {
	.carveouts = ast_carveouts,
	.nr_carveouts = ARRAY_SIZE(ast_carveouts),
};

static struct platform_device ast_nvmap_device = {
	.name = "tegra-nvmap",
	.id = -1,
	.dev = {
		.platform_data = &ast_nvmap_data,
	},
};
#endif

#if defined(CONFIG_ION_TEGRA)

static struct platform_device tegra_iommu_device = {
	.name = "tegra_iommu_device",
	.id = -1,
	.dev = {
		.platform_data = (void *)((1 << HWGRP_COUNT) - 1),
	},
};

static struct ion_platform_data tegra_ion_data = {
	.nr = 4,
	.heaps = {
		{
			.type = ION_HEAP_TYPE_CARVEOUT,
			.id = TEGRA_ION_HEAP_CARVEOUT,
			.name = "carveout",
			.base = 0,
			.size = 0,
		},
		{
			.type = ION_HEAP_TYPE_CARVEOUT,
			.id = TEGRA_ION_HEAP_IRAM,
			.name = "iram",
			.base = TEGRA_IRAM_BASE + TEGRA_RESET_HANDLER_SIZE,
			.size = TEGRA_IRAM_SIZE - TEGRA_RESET_HANDLER_SIZE,
		},
		{
			.type = ION_HEAP_TYPE_CARVEOUT,
			.id = TEGRA_ION_HEAP_VPR,
			.name = "vpr",
			.base = 0,
			.size = 0,
		},
		{
			.type = ION_HEAP_TYPE_IOMMU,
			.id = TEGRA_ION_HEAP_IOMMU,
			.name = "iommu",
			.base = TEGRA_SMMU_BASE,
			.size = TEGRA_SMMU_SIZE,
			.priv = &tegra_iommu_device.dev,
		},
	},
};

static struct platform_device tegra_ion_device = {
	.name = "ion-tegra",
	.id = -1,
	.dev = {
		.platform_data = &tegra_ion_data,
	},
};
#endif

static struct charging_img_data charging_data = {
    .ac_in_pin = AP_ACOK_GPIO,
    .bl_pdev = &ast_backlight_device,
};

static struct platform_device charging_img_device = {
    .name  = "charging-img",
    .id = -1,
    .dev   = {
        .platform_data = &charging_data,
    },
};

static struct platform_device *ast_gfx_devices[] __initdata = {
#if defined(CONFIG_TEGRA_NVMAP)
	&ast_nvmap_device,
#endif
#if defined(CONFIG_ION_TEGRA)
	&tegra_ion_device,
#endif
	&tegra_pwfm0_device,
	&ast_backlight_device,
	&charging_img_device,
};

#ifdef CONFIG_HAS_EARLYSUSPEND
/*
 * put early_suspend/late_resume handlers here for the display in order
 * to keep the code out of the display driver, keeping it closer to upstream
 */
struct early_suspend ast_panel_early_suspender;

extern int is_boot_on_charging;
static void ast_panel_early_suspend(struct early_suspend *h)
{
	/* power down LCD, add use a black screen for HDMI */
	if (num_registered_fb > 0)
		fb_blank(registered_fb[0], FB_BLANK_POWERDOWN);

	if (num_registered_fb > 1)
		fb_blank(registered_fb[1], FB_BLANK_NORMAL);
	//tegra_gpio_disable(HDMI_HPD_GPIO);

#ifdef CONFIG_TEGRA_CONVSERVATIVE_GOV_ON_EARLYSUPSEND
       cpufreq_store_default_gov();
       cpufreq_change_gov(cpufreq_conservative_gov);
#endif
}

static void ast_panel_late_resume(struct early_suspend *h)
{
	unsigned i;

#ifdef CONFIG_TEGRA_CONVSERVATIVE_GOV_ON_EARLYSUPSEND
       cpufreq_restore_default_gov();
#endif

	for (i = 0; i < num_registered_fb; i++)
		fb_blank(registered_fb[i], FB_BLANK_UNBLANK);

	tegra_gpio_enable(HDMI_HPD_GPIO);
}
#endif

static int avalon_panel_init(void)
{
	int err;
	struct resource *res;

	mutex_init(&lcd_lock);

#if defined(CONFIG_TEGRA_NVMAP)
	ast_carveouts[1].base = tegra_carveout_start;
	ast_carveouts[1].size = tegra_carveout_size;
#endif

#if defined(CONFIG_ION_TEGRA)
	tegra_ion_data.heaps[0].base = tegra_carveout_start;
	tegra_ion_data.heaps[0].size = tegra_carveout_size;
#endif

	err = gpio_request(LVDS_SHUTDOWN_GPIO, "lvds_shutdown");
	if (err < 0)
		return err;

	gpio_direction_output(LVDS_SHUTDOWN_GPIO, 1);
	tegra_gpio_enable(LVDS_SHUTDOWN_GPIO);
	gpio_request(HDMI_HPD_GPIO, "hdmi_hpd");
	gpio_direction_input(HDMI_HPD_GPIO);
	tegra_gpio_enable(HDMI_HPD_GPIO);

#ifdef CONFIG_HAS_EARLYSUSPEND
	ast_panel_early_suspender.suspend = ast_panel_early_suspend;
	ast_panel_early_suspender.resume = ast_panel_late_resume;
	ast_panel_early_suspender.level = EARLY_SUSPEND_LEVEL_DISABLE_FB;
	register_early_suspend(&ast_panel_early_suspender);
#endif

#ifdef CONFIG_TEGRA_GRHOST
	err = tegra3_register_host1x_devices();
	if (err)
		return err;
#endif

	err = platform_add_devices(ast_gfx_devices,
				ARRAY_SIZE(ast_gfx_devices));

#if defined(CONFIG_TEGRA_GRHOST) && defined(CONFIG_TEGRA_DC)
	res = nvhost_get_resource_byname(&ast_disp1_device,
					 IORESOURCE_MEM, "fbmem");
	res->start = tegra_fb_start;
	res->end = tegra_fb_start + tegra_fb_size - 1;
#endif

	/* Copy the bootloader fb to the fb. */
	tegra_move_framebuffer(tegra_fb_start, tegra_bootloader_fb_start,
			       min(tegra_fb_size, tegra_bootloader_fb_size));

#if defined(CONFIG_TEGRA_GRHOST) && defined(CONFIG_TEGRA_DC)
	if (!err)
		err = nvhost_device_register(&ast_disp1_device);

	res = nvhost_get_resource_byname(&ast_disp2_device,
					 IORESOURCE_MEM, "fbmem");
	res->start = tegra_fb2_start;
	res->end = tegra_fb2_start + tegra_fb2_size - 1;

	/* Copy the bootloader fb to the fb2. */
	tegra_move_framebuffer(tegra_fb2_start, tegra_bootloader_fb_start,
			       min(tegra_fb2_size, tegra_bootloader_fb_size));
	if (!err)
		err = nvhost_device_register(&ast_disp2_device);
#endif

#if defined(CONFIG_TEGRA_GRHOST) && defined(CONFIG_TEGRA_NVAVP)
	if (!err)
		err = nvhost_device_register(&nvavp_device);
#endif

	return err;
}

int __init ast_panel_init(void)
{
    if (machine_is_avalon() || machine_is_titan()) {
        return avalon_panel_init();
    } else if (machine_is_sphinx()) {
        /* panel: add sphinx-specific here */
        /* NOTE:  sphinx LCD panel initialization is quiet different from avalon/titan */
        /* We implement it in different file, board-sphinx-panel.c */
        return sphinx_panel_init();
    }

    return 0;
}
