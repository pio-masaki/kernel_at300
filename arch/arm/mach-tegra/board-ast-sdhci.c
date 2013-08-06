/*
 * arch/arm/mach-tegra/board-ast-sdhci.c
 *
 * Copyright (C) 2010 Google, Inc.
 * Copyright (C) 2011 NVIDIA Corporation.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/resource.h>
#include <linux/platform_device.h>
#include <linux/wlan_plat.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/mmc/host.h>

#include <asm/mach-types.h>
#include <mach/irqs.h>
#include <mach/iomap.h>
#include <mach/sdhci.h>
#include <mach/io_dpd.h>

#include "gpio-names.h"
#include "board.h"
#include "board-ast.h"
#include <linux/regulator/consumer.h>

DEFINE_MUTEX(wireless_power_lock);

static struct regulator *ast_wifi_1v8_vdd = NULL;
static struct regulator *ast_wifi_3v3_vdd = NULL;
static unsigned int wireless_regulator_use_count_wifi = 0;
static unsigned int wireless_regulator_use_count_bt = 0;


static void (*wifi_status_cb)(int card_present, void *dev_id);
static void *wifi_status_cb_devid;
static int ast_wifi_status_register(void (*callback)(int , void *), void *);

static int ast_wifi_reset(int on);
static int ast_wifi_power(int on);
static int ast_wifi_set_carddetect(int val);

int enable_wireless_regulator(int enable, int power_type);
/*int enable_wireless_regulator(int enable);*/
static struct wifi_platform_data ast_wifi_control = {
	.set_power	= ast_wifi_power,
	.set_reset	= ast_wifi_reset,
	.set_carddetect	= ast_wifi_set_carddetect,
};

static struct resource wifi_resource[] = {
	[0] = {
		.name	= "bcm4329_wlan_irq",
		.start	= TEGRA_GPIO_TO_IRQ(WLAN_WOW_GPIO),
		.end	= TEGRA_GPIO_TO_IRQ(WLAN_WOW_GPIO),
		.flags	= IORESOURCE_IRQ | IORESOURCE_IRQ_HIGHLEVEL | IORESOURCE_IRQ_SHAREABLE,
	},
};

static struct platform_device ast_wifi_device = {
	.name		= "bcm4329_wlan",
	.id		= 1,
	.num_resources	= 1,
	.resource	= wifi_resource,
	.dev		= {
		.platform_data = &ast_wifi_control,
	},
};

static struct resource sdhci_resource0[] = {
	[0] = {
		.start  = INT_SDMMC1,
		.end    = INT_SDMMC1,
		.flags  = IORESOURCE_IRQ,
	},
	[1] = {
		.start	= TEGRA_SDMMC1_BASE,
		.end	= TEGRA_SDMMC1_BASE + TEGRA_SDMMC1_SIZE-1,
		.flags	= IORESOURCE_MEM,
	},
};

static struct resource sdhci_resource2[] = {
	[0] = {
		.start  = INT_SDMMC3,
		.end    = INT_SDMMC3,
		.flags  = IORESOURCE_IRQ,
	},
	[1] = {
		.start	= TEGRA_SDMMC3_BASE,
		.end	= TEGRA_SDMMC3_BASE + TEGRA_SDMMC3_SIZE-1,
		.flags	= IORESOURCE_MEM,
	},
};

static struct resource sdhci_resource3[] = {
	[0] = {
		.start  = INT_SDMMC4,
		.end    = INT_SDMMC4,
		.flags  = IORESOURCE_IRQ,
	},
	[1] = {
		.start	= TEGRA_SDMMC4_BASE,
		.end	= TEGRA_SDMMC4_BASE + TEGRA_SDMMC4_SIZE-1,
		.flags	= IORESOURCE_MEM,
	},
};

static struct embedded_sdio_data embedded_sdio_data2 = {
	.cccr   = {
		.sdio_vsn       = 2,
		.multi_block    = 1,
		.low_speed      = 0,
		.wide_bus       = 0,
		.high_power     = 1,
		.high_speed     = 1,
	},
	.cis  = {
		.vendor         = 0x02d0,
		.device         = 0x4330,
	},
};

static struct tegra_sdhci_platform_data tegra_sdhci_platform_data2 = {
	.mmc_data = {
		.register_status_notify	= ast_wifi_status_register,
		.embedded_sdio = &embedded_sdio_data2,
		.built_in = 1,
	},
	.cd_gpio = -1,
	.wp_gpio = -1,
	.power_gpio = -1,
	.tap_delay = 0x0F,
	.ddr_clk_limit = 41000000,
/*	.is_voltage_switch_supported = false,
	.vdd_rail_name = NULL,
	.slot_rail_name = NULL,
	.vdd_max_uv = -1,
	.vdd_min_uv = -1,
	.max_clk = 0,
	.is_8bit_supported = false, */
};

static struct tegra_sdhci_platform_data tegra_sdhci_platform_data0 = {
	.cd_gpio = SD_CD_GPIO,
	.wp_gpio = SD_WP_GPIO,
	.power_gpio = -1,
	.tap_delay = 0x0F,
	.ddr_clk_limit = 41000000,
/*	.is_voltage_switch_supported = true,
	.vdd_rail_name = "vddio_sdmmc1",
	.slot_rail_name = "vddio_sd_slot",
	.vdd_max_uv = 3320000,
	.vdd_min_uv = 3280000,
	.max_clk = 208000000,
	.is_8bit_supported = false, */
};

static struct tegra_sdhci_platform_data tegra_sdhci_platform_data3 = {
	.cd_gpio = -1,
	.wp_gpio = -1,
	.power_gpio = -1,
	.is_8bit = 1,
	.tap_delay = 0x0F,
	.ddr_clk_limit = 41000000,
	.mmc_data = {
		.built_in = 1,
	}
/*	.is_voltage_switch_supported = false,
	.vdd_rail_name = NULL,
	.slot_rail_name = NULL,
	.vdd_max_uv = -1,
	.vdd_min_uv = -1,
	.max_clk = 48000000,
	.is_8bit_supported = true, */
};

static struct platform_device tegra_sdhci_device0 = {
	.name		= "sdhci-tegra",
	.id		= 0,
	.resource	= sdhci_resource0,
	.num_resources	= ARRAY_SIZE(sdhci_resource0),
	.dev = {
		.platform_data = &tegra_sdhci_platform_data0,
	},
};

static struct platform_device tegra_sdhci_device2 = {
	.name		= "sdhci-tegra",
	.id		= 2,
	.resource	= sdhci_resource2,
	.num_resources	= ARRAY_SIZE(sdhci_resource2),
	.dev = {
		.platform_data = &tegra_sdhci_platform_data2,
	},
};

static struct platform_device tegra_sdhci_device3 = {
	.name		= "sdhci-tegra",
	.id		= 3,
	.resource	= sdhci_resource3,
	.num_resources	= ARRAY_SIZE(sdhci_resource3),
	.dev = {
		.platform_data = &tegra_sdhci_platform_data3,
	},
};

static int ast_wifi_status_register(
		void (*callback)(int card_present, void *dev_id),
		void *dev_id)
{
	if (wifi_status_cb)
		return -EAGAIN;
	wifi_status_cb = callback;
	wifi_status_cb_devid = dev_id;
	return 0;
}

static int ast_wifi_set_carddetect(int val)
{
	pr_err("%s: %d\n", __func__, val);
	if (wifi_status_cb)
		wifi_status_cb(val, wifi_status_cb_devid);
	else
		pr_warning("%s: Nobody to notify\n", __func__);
	return 0;
}

int  enable_wireless_regulator(int enable, int power_type)
{
	mutex_lock(&wireless_power_lock);

	printk(KERN_ERR "%s(): enable = %d, power_type=%d\n", __func__, enable, power_type);
	switch (power_type){
		case WIFI_POWER:
			if (enable) {
				if (wireless_regulator_use_count_wifi == 0) {
					if (wireless_regulator_use_count_bt == 0) {
						ast_wifi_1v8_vdd = regulator_get(NULL, "vdd_1v8_wlan");
						if (IS_ERR(ast_wifi_1v8_vdd)) {
							WARN_ON(IS_ERR(ast_wifi_1v8_vdd));
							mutex_unlock(&wireless_power_lock);
							printk(KERN_ERR "Dennis...ast_wifi_1v8_vdd\n");
							return -1;
						}

						ast_wifi_3v3_vdd = regulator_get(NULL, "vdd_3v3_wlan");
						if (IS_ERR(ast_wifi_3v3_vdd)) {
							regulator_put(ast_wifi_1v8_vdd);
							ast_wifi_1v8_vdd = NULL;
							WARN_ON(IS_ERR(ast_wifi_3v3_vdd));
							mutex_unlock(&wireless_power_lock);
							printk(KERN_ERR "Dennis...ast_wifi_3v3_vdd\n");
							return -1;
						}
						printk("%s(): Regulator_enable - 1v8 and 3v3\n", __func__);
						regulator_enable(ast_wifi_1v8_vdd);
						regulator_enable(ast_wifi_3v3_vdd);
					}
					else{
						printk("%s():BT power is already enabled! - No action to regulator\n", __func__);
					}
				}
				wireless_regulator_use_count_wifi++;
			} else {
				if (wireless_regulator_use_count_wifi != 0) {
					wireless_regulator_use_count_wifi--;
					if (wireless_regulator_use_count_wifi == 0) {
						if (wireless_regulator_use_count_bt == 0) {
							regulator_disable(ast_wifi_1v8_vdd);
							regulator_disable(ast_wifi_3v3_vdd);

							regulator_put(ast_wifi_1v8_vdd);
							regulator_put(ast_wifi_3v3_vdd);

							ast_wifi_1v8_vdd = NULL;
							ast_wifi_3v3_vdd = NULL;
						}
						else{
							printk("%s():BT power is still enabled! - No action to regulator\n", __func__);
						}
					}
				} else {
					printk("%s():Wirless regulator use count for WIFI is 0! - No action to regulator\n", __func__);
				}
			}
			break;
		case BT_POWER:
			if (enable) {
				if (wireless_regulator_use_count_bt == 0) {
					if (wireless_regulator_use_count_wifi == 0) {
						ast_wifi_1v8_vdd = regulator_get(NULL, "vdd_1v8_wlan");
						if (IS_ERR(ast_wifi_1v8_vdd)) {
							WARN_ON(IS_ERR(ast_wifi_1v8_vdd));
							mutex_unlock(&wireless_power_lock);
							printk(KERN_ERR "Dennis...ast_bt_1v8_vdd\n");
							return -1;
						}

						ast_wifi_3v3_vdd = regulator_get(NULL, "vdd_3v3_wlan");
						if (IS_ERR(ast_wifi_3v3_vdd)) {
							regulator_put(ast_wifi_1v8_vdd);
							ast_wifi_1v8_vdd = NULL;
							WARN_ON(IS_ERR(ast_wifi_3v3_vdd));
							mutex_unlock(&wireless_power_lock);
							printk(KERN_ERR "Dennis...ast_bt_3v3_vdd\n");
							return -1;
						}
						printk("%s(): Regulator_disable - 1v8 and 3v3\n", __func__);
						regulator_enable(ast_wifi_1v8_vdd);
						regulator_enable(ast_wifi_3v3_vdd);
					}
					else{
						printk("%s():WIFI power is already enabled! - No action to regulator\n", __func__);
					}
				}
				wireless_regulator_use_count_bt++;
			} else {
				if (wireless_regulator_use_count_bt != 0) {
					wireless_regulator_use_count_bt--;
					if (wireless_regulator_use_count_bt == 0) {
						if (wireless_regulator_use_count_wifi == 0) {
							regulator_disable(ast_wifi_1v8_vdd);
							regulator_disable(ast_wifi_3v3_vdd);

							regulator_put(ast_wifi_1v8_vdd);
							regulator_put(ast_wifi_3v3_vdd);

							ast_wifi_1v8_vdd = NULL;
							ast_wifi_3v3_vdd = NULL;
						}
						else{
							printk("%s():WIFI power is still enabled! - No action to regulator\n", __func__);
						}
					}
				} else {
					printk("%s():Wirless regulator use count for BT is 0! - No action to regulator\n", __func__);
				}
			}
			break;
		default:
			mutex_unlock(&wireless_power_lock);
			printk(KERN_ERR "%s(): Unknown power type (must be WIFI  or BT).\n", __func__);
			return -1;
	}

	mutex_unlock(&wireless_power_lock);
	mdelay(200);
	return 0;
}

static int ast_wifi_power(int on)
{
	struct tegra_io_dpd *sd_dpd;

	pr_err("%s: %d\n", __func__, on);

#if defined(CONFIG_MACH_AVALON) || defined(CONFIG_MACH_SPHINX)
	sd_dpd = tegra_io_dpd_get(&tegra_sdhci_device2.dev);
	if (sd_dpd) {
		mutex_lock(&sd_dpd->delay_lock);
		tegra_io_dpd_disable(sd_dpd);
		mutex_unlock(&sd_dpd->delay_lock);
	}
#endif

	switch (on) {
	case 0:
		gpio_set_value(WLAN_RST_GPIO, on);
		mdelay(200);
		if (enable_wireless_regulator(0,WIFI_POWER) == -1)
			printk(KERN_ERR "%s(): disable regulator failed!\n", __func__);
		break;
	case 1:
		if (enable_wireless_regulator(1,WIFI_POWER) == -1)
			printk(KERN_ERR "%s(): enable regulator failed!\n", __func__);
		gpio_set_value(WLAN_RST_GPIO, on);
		mdelay(200);
		break;
	case 2:
		/* Set Wi-Fi at RESET OFF state */
		gpio_set_value(WLAN_RST_GPIO, 0);
		mdelay(200);
		break;
	case 3:
		/* Set Wi-Fi at RESET ON state */
		gpio_set_value(WLAN_RST_GPIO, 1);
		mdelay(200);
		break;
	default:
		pr_err("%s: Wrong power command %d\n", __func__, on);
		break;
	}

#if defined(CONFIG_MACH_AVALON) || defined(CONFIG_MACH_SPHINX)
	if (sd_dpd) {
		mutex_lock(&sd_dpd->delay_lock);
		tegra_io_dpd_enable(sd_dpd);
		mutex_unlock(&sd_dpd->delay_lock);
	}
#endif

	return 0;
}


static int ast_wifi_reset(int on)
{
	pr_debug("%s: do nothing\n", __func__);
	return 0;
}

static int __init ast_wifi_init(void)
{
	int rc;
	pr_err("%s: Enter\n", __func__);
	rc = gpio_request(WLAN_RST_GPIO, "wlan_rst");
	if (rc)
		pr_err("WLAN_RST gpio request failed:%d\n", rc);
	rc = gpio_request(WLAN_WOW_GPIO, "bcmsdh_sdmmc");
	if (rc)
		pr_err("WLAN_WOW gpio request failed:%d\n", rc);
	gpio_direction_output(WLAN_RST_GPIO, 0);
	if (rc)
		pr_err("WLAN_RST gpio direction configuration failed:%d\n", rc);
	rc = gpio_direction_input(WLAN_WOW_GPIO);
	if (rc)
		pr_err("WLAN_WOW gpio direction configuration failed:%d\n", rc);

	tegra_gpio_enable(WLAN_RST_GPIO);
	tegra_gpio_enable(WLAN_WOW_GPIO);
	platform_device_register(&ast_wifi_device);
	return 0;
}


int __init ast_sdhci_init(void)
{
	platform_device_register(&tegra_sdhci_device3);
	platform_device_register(&tegra_sdhci_device2);
	platform_device_register(&tegra_sdhci_device0);

	ast_wifi_init();
	return 0;
}
EXPORT_SYMBOL_GPL (wireless_power_lock);
