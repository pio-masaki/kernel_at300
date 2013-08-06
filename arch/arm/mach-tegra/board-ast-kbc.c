/*
 * arch/arm/mach-tegra/board-ast-kbc.c
 * Keys configuration for Nvidia tegra3 ast platform.
 *
 * Copyright (C) 2011 NVIDIA, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA
 * 02111-1307, USA
 */

#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/device.h>
#include <linux/gpio.h>
#include <linux/gpio_keys.h>
#include <linux/switch.h>
#include <linux/mfd/tps6591x.h>
#include <linux/interrupt_keys.h>
#include <linux/gpio_scrollwheel.h>
#include <linux/io.h>

#include <mach/irqs.h>
#include <mach/io.h>
#include <mach/iomap.h>
#include <mach/kbc.h>
#include "board.h"
#include "board-ast.h"

#include "gpio-names.h"
#include "wakeups-t3.h"

#define GPIO_KEY(_id, _gpio, _iswake)		\
	{					\
		.code = _id,			\
		.gpio = TEGRA_GPIO_##_gpio,	\
		.active_low = 1,		\
		.desc = #_id,			\
		.type = EV_KEY,			\
		.wakeup = _iswake,		\
		.debounce_interval = 10,	\
	}

static struct gpio_keys_button ast_gpio_keys[] = {
    [0] = GPIO_KEY(KEY_POWER, PV0, 1),
	[1] = GPIO_KEY(KEY_VOLUMEUP, PQ0, 0),
	[2] = GPIO_KEY(KEY_VOLUMEDOWN, PQ1, 0),
};

#define PMC_WAKE_STATUS		0x14
static int ast_wakeup_key(void)
{
    unsigned long status = readl(IO_ADDRESS(TEGRA_PMC_BASE) + PMC_WAKE_STATUS);
  
    if ((status & TEGRA_WAKE_GPIO_PV0) || (status & TEGRA_WAKE_GPIO_PV1)) {
        return KEY_POWER;
    } else {
        return KEY_RESERVED;
    }
}

static const char* ast_wakeup_name(void)
{
    unsigned long status = readl(IO_ADDRESS(TEGRA_PMC_BASE) + PMC_WAKE_STATUS);

    if (status & TEGRA_WAKE_GPIO_PV0) {
	return "KEY_POWER";
    } else if (status & TEGRA_WAKE_GPIO_PV1) {
	return "ac_in";
    } else if (status & TEGRA_WAKE_GPIO_PS4) {
	return "nvdock";
    }
    return "None";
}

static struct gpio_keys_platform_data ast_gpio_keys_platform_data = {
	.buttons	= ast_gpio_keys,
	.nbuttons	= ARRAY_SIZE(ast_gpio_keys),
    .wakeup_key = ast_wakeup_key,
    .wakeup_name = ast_wakeup_name,
};

static struct platform_device ast_gpio_keys_device = {
	.name   = "gpio-keys",
	.id     = 0,
	.dev    = {
		.platform_data  = &ast_gpio_keys_platform_data,
	},
};

/* #define INT_KEY(_id, _irq, _iswake, _deb_int)	\ */
/* 	{					\ */
/* 		.code = _id,			\ */
/* 		.irq = _irq,			\ */
/* 		.active_low = 1,		\ */
/* 		.desc = #_id,			\ */
/* 		.type = EV_KEY,			\ */
/* 		.wakeup = _iswake,		\ */
/* 		.debounce_interval = _deb_int,	\ */
/* 	} */
/* static struct interrupt_keys_button ast_int_keys[] = { */
/* 	[0] = INT_KEY(KEY_POWER, TPS6591X_IRQ_BASE + TPS6591X_INT_PWRON, 0, 100), */
/* }; */

/* static struct interrupt_keys_platform_data ast_int_keys_pdata = { */
/* 	.int_buttons	= ast_int_keys, */
/* 	.nbuttons       = ARRAY_SIZE(ast_int_keys), */
/* }; */

/* static struct platform_device ast_int_keys_device = { */
/* 	.name   = "interrupt-keys", */
/* 	.id     = 0, */
/* 	.dev    = { */
/* 		.platform_data  = &ast_int_keys_pdata, */
/* 	}, */
/* }; */

#define AST_MULTIFUNC_SWITCH         TEGRA_GPIO_PR5
#define MULTI_FUNC_ACTIVE_LOW	     (1)
static struct gpio_switch_platform_data ast_sw_platform_data = {
    .name           = "multi-func",
    .gpio           = AST_MULTIFUNC_SWITCH,
    .active_low     = MULTI_FUNC_ACTIVE_LOW,
};

static struct platform_device ast_sw_device = {
    .name   = "switch-gpio",
    .id     = 0,
    .dev    = {
        .platform_data  = &ast_sw_platform_data,
    },
};

int __init ast_keys_init(void)
{
	int i;

	pr_info("Registering gpio keys and switch\n");

    for (i = 0; i < ARRAY_SIZE(ast_gpio_keys); i++)
        tegra_gpio_enable(ast_gpio_keys[i].gpio);

    platform_device_register(&ast_gpio_keys_device);

    tegra_gpio_enable(AST_MULTIFUNC_SWITCH);
    platform_device_register(&ast_sw_device);

	return 0;
}
