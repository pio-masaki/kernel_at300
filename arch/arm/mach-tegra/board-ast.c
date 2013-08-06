/*
 * arch/arm/mach-tegra/board-ast.c
 *
 * Copyright (c) 2011-2012, NVIDIA Corporation.
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

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/ctype.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/serial_8250.h>
#include <linux/i2c.h>
#include <linux/i2c/panjit_ts.h>
#include <linux/dma-mapping.h>
#include <linux/delay.h>
#include <linux/i2c-tegra.h>
#include <linux/gpio.h>
#include <linux/input.h>
#include <linux/platform_data/tegra_usb.h>
#include <linux/spi/spi.h>
#ifdef CONFIG_TOUCHSCREEN_ATMEL_MXT768E
#include <linux/i2c/atmel_maxtouch.h>
#endif
#ifdef CONFIG_TOUCHSCREEN_ATMEL_MXT
#include <linux/i2c/atmel_mxt_ts.h>
#endif
#ifdef CONFIG_TOUCHSCREEN_GOODIX_BIG
#include <linux/i2c/goodix_touch.h>
#endif
#include <linux/tegra_uart.h>
#include <linux/memblock.h>
#include <linux/spi-tegra.h>
#include <linux/ast_dock.h>
#ifdef CONFIG_SENSORS_IQS128
#include <linux/iqs128_cap_sensor.h>
#endif
#include <linux/rfkill.h>
#include <sound/wm8903.h>

#include <mach/clk.h>
#include <mach/iomap.h>
#include <mach/irqs.h>
#include <mach/pinmux.h>
#include <mach/iomap.h>
#include <mach/io.h>
#include <mach/i2s.h>
#include <mach/tegra_wm8903_pdata.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <mach/usb_phy.h>
#include <linux/nfc/pn544.h>
#include <sound/fm34.h>
#include <mach/thermal.h>
#include <mach/ast_tspdrv_pdata.h>
#ifdef CONFIG_USB_ISDBT_POWER
#include <linux/isdbt_power.h>
#endif

#include "board.h"
#include "clock.h"
#include "board-ast.h"
#include "devices.h"
#include "gpio-names.h"
#include "fuse.h"
#include "pm.h"
#include "baseband-xmm-power.h"
#include "wdt-recovery.h"

static struct board_version s_hw_info = {
	.pj_id = PROJECT_UNKNOWN,
	.hw_version = BOARD_VERSION_UNKNOWN
};

extern int enable_wireless_regulator(int enable, int power_type);
/*extern int enable_wireless_regulator(int enable);*/

/* All units are in millicelsius */
static struct tegra_thermal_data thermal_data = {
    .temp_throttle = 85000,
    .temp_shutdown = 90000,
    .temp_offset = TDIODE_OFFSET, /* temps based on tdiode */
#ifdef CONFIG_TEGRA_EDP_LIMITS
    .edp_offset = TDIODE_OFFSET,  /* edp based on tdiode */
    .hysteresis_edp = 3000,
#endif
#ifdef CONFIG_TEGRA_THERMAL_SYSFS
    .tc1 = 0,
    .tc2 = 1,
    .passive_delay = 2000,
#else
    .hysteresis_throttle = 1000,
#endif
};

/* !!!TODO: Change for ast (Taken from Ventana) */
static struct tegra_utmip_config utmi_phy_config[] = {
	[0] = {
			.hssync_start_delay = 0,
			.idle_wait_delay = 17,
			.elastic_limit = 16,
			.term_range_adj = 6,
			.xcvr_setup = 15,
			.xcvr_setup_offset = 0,
			.xcvr_use_fuses = 1,
			.xcvr_lsfslew = 2,
			.xcvr_lsrslew = 2,
	},
	[1] = {
			.hssync_start_delay = 0,
			.idle_wait_delay = 17,
			.elastic_limit = 16,
			.term_range_adj = 6,
			.xcvr_setup = 15,
			.xcvr_setup_offset = 0,
			.xcvr_use_fuses = 1,
			.xcvr_lsfslew = 2,
			.xcvr_lsrslew = 2,
	},
	[2] = {
			.hssync_start_delay = 0,
			.idle_wait_delay = 17,
			.elastic_limit = 16,
			.term_range_adj = 6,
			.xcvr_setup = 8,
			.xcvr_setup_offset = 0,
			.xcvr_use_fuses = 1,
			.xcvr_lsfslew = 2,
			.xcvr_lsrslew = 2,
	},
};

static struct resource ast_bcm4329_rfkill_resources[] = {
	{
		.name   = "bcm4329_nshutdown_gpio",
		.start  = TEGRA_GPIO_PB2,
		.end    = TEGRA_GPIO_PB2,
		.flags  = IORESOURCE_IO,
	},
	{
		.name   = "bcm4329_nreset_gpio",
		.start  = TEGRA_GPIO_PU0,
		.end    = TEGRA_GPIO_PU0,
		.flags  = IORESOURCE_IO,
	},
};

static struct wireless_power_control ast_wireless_power_control = {
	.enable_wireless_regulator_control = enable_wireless_regulator,
};

static struct platform_device ast_bcm4329_rfkill_device = {
	.name = "bcm4329_rfkill",
	.id             = -1,
	.num_resources  = ARRAY_SIZE(ast_bcm4329_rfkill_resources),
	.resource       = ast_bcm4329_rfkill_resources,
	.dev = {
		.platform_data = &ast_wireless_power_control,
	},
};

static struct resource ast_bluesleep_resources[] = {
    [0] = {
        .name = "gpio_host_wake",
        .start  = TEGRA_GPIO_PU6,
        .end    = TEGRA_GPIO_PU6,
        .flags  = IORESOURCE_IO,
    },
    [1] = {
        .name = "gpio_ext_wake",
        .start  = TEGRA_GPIO_PU1,
        .end    = TEGRA_GPIO_PU1,
        .flags  = IORESOURCE_IO,
    },
    [2] = {
        .name = "host_wake",
        .start  = TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PU6),
        .end    = TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PU6),
        .flags  = IORESOURCE_IRQ | IORESOURCE_IRQ_HIGHEDGE,
    },
};

static struct platform_device ast_bluesleep_device = {
    .name           = "bluesleep",
    .id             = -1,
    .num_resources  = ARRAY_SIZE(ast_bluesleep_resources),
    .resource       = ast_bluesleep_resources,
};

static noinline void __init ast_setup_bluesleep(void)
{
    platform_device_register(&ast_bluesleep_device);
	tegra_gpio_enable(TEGRA_GPIO_PU6);
	tegra_gpio_enable(TEGRA_GPIO_PU1);

	return;
}

static __initdata struct tegra_clk_init_table ast_clk_init_table[] = {
	/* name		parent		rate		enabled */
	{ "pll_m",	NULL,		0,		false},
	{ "hda",	"pll_p",	108000000,	false},
	{ "hda2codec_2x","pll_p",	48000000,	false},
	{ "pwm",	"pll_p",	6375000,	false}, /* for haptic pwm */
	{ "blink",	"clk_32k",	32768,		true},
	{ "i2s1",	"pll_a_out0",	0,		false},
	{ "i2s3",	"pll_a_out0",	0,		false},
	{ "spdif_out",	"pll_a_out0",	0,		false},
	{ "d_audio",	"pll_a_out0",	12000000,	false},
	{ "dam0",	"pll_a_out0",	    12000000,	false},
	{ "dam1",	"pll_a_out0",	    12000000,	false},
	{ "dam2",	"pll_a_out0",	    12000000,	false},
    { "audio1", "i2s1_sync",    0,  false},
    { "audio3", "i2s3_sync",    0,  false},
	{ "vi_sensor",	"pll_p",	150000000,	false},
	{ "i2c1",	"pll_p",	3200000,	false},
	{ "i2c2",	"pll_p",	3200000,	false},
	{ "i2c3",	"pll_p",	3200000,	false},
	{ "i2c4",	"pll_p",	3200000,	false},
	{ "i2c5",	"pll_p",	3200000,	false},
	{ NULL,		NULL,		0,		0},
};

static struct pn544_i2c_platform_data avalon_nfc_pdata = {
	.irq_gpio = TEGRA_GPIO_PS7,
	.ven_gpio = TEGRA_GPIO_PX7,
	.firm_gpio = TEGRA_GPIO_PQ7,
	};

static struct i2c_board_info __initdata avalon_i2c_gen1_board_info[] = {
	{
		I2C_BOARD_INFO("pn544", 0x29),
		.platform_data = &avalon_nfc_pdata,
		.irq = TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PS7),
	},
};

static struct pn544_i2c_platform_data sphinx_nfc_pdata = {
	.irq_gpio = TEGRA_GPIO_PS7,
	.ven_gpio = TEGRA_GPIO_PD2,
	.firm_gpio = TEGRA_GPIO_PS2,
	};

static struct i2c_board_info __initdata sphinx_i2c_ddc_board_info[] = {
	{
		I2C_BOARD_INFO("pn544", 0x29),
		.platform_data = &sphinx_nfc_pdata,
		.irq = TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PS7),
	},
};

#ifdef CONFIG_SND_SOC_FM34
const static u16 fm34_property[][2] = {
	{0x22C8, 0x0026},
	{0x22D2, 0x8A94},
	{0x22E3, 0x50C2},
	{0x22EE, 0x0000},
	{0x22F2, 0x0040},
	{0x22F6, 0x0002},
	{0x22F8, 0x8005},
	{0x22F9, 0x085F},
	{0x22FA, 0x2483},
	{0x2301, 0x0002},
	{0x2303, 0x0021},
	{0x2305, 0x0004},
	{0x2307, 0xF8F8},	// Entry MIC in Gain
	{0x2309, 0x0800},
	{0x230C, 0x1000},	// Leave MIC In Gain
	{0x230D, 0x0100},	// Speaker Out Gain
	{0x2310, 0x1880},
	{0x2325, 0x5000},
	{0x232F, 0x0080},
	{0x2332, 0x0200},
	{0x2333, 0x0020},
	{0x2339, 0x0010},
	{0x2357, 0x0100},
	{0x2391, 0x4000},
	{0x2392, 0x4000},
	{0x2393, 0x4000},
	{0x2394, 0x4000},
	{0x2395, 0x4000},
	{0x23CF, 0x0050},
	{0x23D5, 0x4B00},
	{0x23D7, 0x002A},
	{0x23E0, 0x4000},
	{0x23E1, 0x4000},
	{0x23E2, 0x4000},
	{0x23E3, 0x4000},
	{0x23E4, 0x4000},
	{0x3FD2, 0x0032},
	{0x22FB, 0x0000}
};

static struct fm34_conf fm34_conf = {
	.pwdn = TEGRA_GPIO_PN1,
	.rst = TEGRA_GPIO_PN0,
	.bp = TEGRA_GPIO_PN3,
	.cprop = sizeof(fm34_property)/sizeof(u16)/2,
	.pprop = (u16 *)fm34_property,
};

static struct i2c_board_info __initdata ast_fm34_board_info[] = {
	{
		I2C_BOARD_INFO("fm34_i2c", 0x60),
		.platform_data = &fm34_conf,
	},
};

static int __init ast_fm34_init(void)
{
	tegra_gpio_enable(fm34_conf.pwdn);
	if(fm34_conf.bp != -1)
		tegra_gpio_enable(fm34_conf.bp);
	if(fm34_conf.rst != -1)
		tegra_gpio_enable(fm34_conf.rst);

	i2c_register_board_info(0, ast_fm34_board_info, 1);

	return 0;
}
#endif

/* GEN1_I2C */
static struct tegra_i2c_platform_data ast_i2c1_platform_data = {
	.adapter_nr	= 0,
	.bus_count	= 1,
	.bus_clk_rate	= { 400000, 0 },
	.scl_gpio		= {TEGRA_GPIO_PC4, 0},
	.sda_gpio		= {TEGRA_GPIO_PC5, 0},
	.arb_recovery = arb_lost_recovery,
	.i2c_bus_reset_devices = NULL,
};

/* GEN2_I2C */
static struct tegra_i2c_platform_data ast_i2c2_platform_data = {
	.adapter_nr	= 1,
	.bus_count	= 1,
	.bus_clk_rate	= { 400000, 0 },
	.is_clkon_always = true,
	.scl_gpio		= {TEGRA_GPIO_PT5, 0},
	.sda_gpio		= {TEGRA_GPIO_PT6, 0},
	.arb_recovery = arb_lost_recovery,
	.i2c_bus_reset_devices = NULL,
};

/* CAM_I2C */
static struct tegra_i2c_platform_data ast_i2c3_platform_data = {
	.adapter_nr	= 2,
	.bus_count	= 1,
	.bus_clk_rate	= { 400000, 0 },
	.scl_gpio		= {TEGRA_GPIO_PBB1, 0},
	.sda_gpio		= {TEGRA_GPIO_PBB2, 0},
	.arb_recovery = arb_lost_recovery,
	.i2c_bus_reset_devices = NULL,
};

/* DDC_I2C */
static struct tegra_i2c_platform_data ast_i2c4_platform_data = {
	.adapter_nr	= 3,
	.bus_count	= 1,
	.bus_clk_rate	= { 100000, 0 },
	.scl_gpio		= {TEGRA_GPIO_PV4, 0},
	.sda_gpio		= {TEGRA_GPIO_PV5, 0},
	.arb_recovery = arb_lost_recovery,
	.i2c_bus_reset_devices = NULL,
};

/* PWR_I2C */
static struct tegra_i2c_platform_data ast_i2c5_platform_data = {
	.adapter_nr	= 4,
	.bus_count	= 1,
	.bus_clk_rate	= { 400000, 0 },
	.scl_gpio		= {TEGRA_GPIO_PZ6, 0},
	.sda_gpio		= {TEGRA_GPIO_PZ7, 0},
	.arb_recovery = arb_lost_recovery,
};


#if 0
struct tegra_wired_jack_conf audio_wr_jack_conf = {
	.hp_det_n = TEGRA_GPIO_PW2,
	.en_mic_ext = TEGRA_GPIO_PX1,
	.en_mic_int = TEGRA_GPIO_PX0,
};
#endif

static struct wm8903_platform_data ast_wm8903_pdata = {
	.irq_active_low = 0,
	.micdet_cfg = 0,
	.micdet_delay = 100,
	.gpio_base = AST_GPIO_WM8903(0),
	.gpio_cfg = {
		(WM8903_GPn_FN_DMIC_LR_CLK_OUTPUT << WM8903_GP1_FN_SHIFT),
		(WM8903_GPn_FN_DMIC_LR_CLK_OUTPUT << WM8903_GP2_FN_SHIFT) |
			WM8903_GP2_DIR,
		0,
		WM8903_GPIO_NO_CONFIG,
		WM8903_GPIO_NO_CONFIG,
	},
#ifdef CONFIG_MACH_AVALON
	.adc_digital_volume = 0xEF,
	.adc_analogue_volume = 0x1B,
	.dac_digital_volume = 0xBD,
	.dac_headphone_volume = 0x3B,
	.dac_lineout_volume = 0x35,
	.dac_speaker_volume = 0x36,
#endif
#ifdef CONFIG_MACH_SPHINX
	.adc_digital_volume = 0xEF,
	.adc_analogue_volume = 0x1B,
	.dac_digital_volume = 0xBD,
	.dac_headphone_volume = 0x3B,
	.dac_lineout_volume = 0x35,
	.dac_speaker_volume = 0x36,
#endif
#ifdef CONFIG_MACH_TITAN
	.adc_digital_volume = 0xEF,
	.adc_analogue_volume = 0x1B,
	.dac_digital_volume = 0xBD,
	.dac_headphone_volume = 0x3B,
	.dac_lineout_volume = 0x34,
	.dac_speaker_volume = 0x39,
#endif
};

static struct i2c_board_info __initdata wm8903_board_info = {
	I2C_BOARD_INFO("wm8903", 0x1a),
	.platform_data = &ast_wm8903_pdata,
	.irq = 0,
};

static void ast_i2c_init(void)
{
	tegra_i2c_device1.dev.platform_data = &ast_i2c1_platform_data;
	tegra_i2c_device2.dev.platform_data = &ast_i2c2_platform_data;
	tegra_i2c_device3.dev.platform_data = &ast_i2c3_platform_data;
	tegra_i2c_device4.dev.platform_data = &ast_i2c4_platform_data;
	tegra_i2c_device5.dev.platform_data = &ast_i2c5_platform_data;

	platform_device_register(&tegra_i2c_device5);
	platform_device_register(&tegra_i2c_device4);
	platform_device_register(&tegra_i2c_device3);
	platform_device_register(&tegra_i2c_device2);
	platform_device_register(&tegra_i2c_device1);

	i2c_register_board_info(0, &wm8903_board_info, 1);
}

static struct platform_device *ast_uart_devices[] __initdata = {
	&tegra_uarta_device,
	&tegra_uartb_device,
	&tegra_uartc_device,
	&tegra_uartd_device,
	&tegra_uarte_device,
};
static struct uart_clk_parent uart_parent_clk[] = {
	[0] = {.name = "clk_m"},
	[1] = {.name = "pll_p"},
#ifndef CONFIG_TEGRA_PLLM_RESTRICTED
	[2] = {.name = "pll_m"},
#endif
};

static struct tegra_uart_platform_data ast_uart_pdata;
static struct tegra_uart_platform_data ast_loopback_uart_pdata;

static void __init uart_debug_init(void)
{
	int debug_port_id;

	debug_port_id = get_tegra_uart_debug_port_id();
	if (debug_port_id < 0) {
		debug_port_id = 0;
	}

    tegra_init_debug_uart_rate();
	switch (debug_port_id) {
	case 0:
		/* UARTA is the debug port. */
		pr_info("Selecting UARTA as the debug console\n");
		ast_uart_devices[0] = &debug_uarta_device;
		debug_uart_clk = clk_get_sys("serial8250.0", "uarta");
		debug_uart_port_base = ((struct plat_serial8250_port *)(
			debug_uarta_device.dev.platform_data))->mapbase;
		break;

	case 1:
		/* UARTB is the debug port. */
		pr_info("Selecting UARTB as the debug console\n");
		ast_uart_devices[1] = &debug_uartb_device;
		debug_uart_clk =  clk_get_sys("serial8250.0", "uartb");
		debug_uart_port_base = ((struct plat_serial8250_port *)(
			debug_uartb_device.dev.platform_data))->mapbase;
		break;

	case 2:
		/* UARTC is the debug port. */
		pr_info("Selecting UARTC as the debug console\n");
		ast_uart_devices[2] = &debug_uartc_device;
		debug_uart_clk =  clk_get_sys("serial8250.0", "uartc");
		debug_uart_port_base = ((struct plat_serial8250_port *)(
			debug_uartc_device.dev.platform_data))->mapbase;
		break;

	case 3:
		/* UARTD is the debug port. */
		pr_info("Selecting UARTD as the debug console\n");
		ast_uart_devices[3] = &debug_uartd_device;
		debug_uart_clk =  clk_get_sys("serial8250.0", "uartd");
		debug_uart_port_base = ((struct plat_serial8250_port *)(
			debug_uartd_device.dev.platform_data))->mapbase;
		break;

	case 4:
		/* UARTE is the debug port. */
		pr_info("Selecting UARTE as the debug console\n");
		ast_uart_devices[4] = &debug_uarte_device;
		debug_uart_clk =  clk_get_sys("serial8250.0", "uarte");
		debug_uart_port_base = ((struct plat_serial8250_port *)(
			debug_uarte_device.dev.platform_data))->mapbase;
		break;

	default:
		pr_info("The debug console id %d is invalid, Assuming UARTA", debug_port_id);
		ast_uart_devices[0] = &debug_uarta_device;
		debug_uart_clk = clk_get_sys("serial8250.0", "uarta");
		debug_uart_port_base = ((struct plat_serial8250_port *)(
			debug_uarta_device.dev.platform_data))->mapbase;
		break;
	}
	return;
}

static void __init ast_uart_init(void)
{
	struct clk *c;
	int i;

	for (i = 0; i < ARRAY_SIZE(uart_parent_clk); ++i) {
		c = tegra_get_clock_by_name(uart_parent_clk[i].name);
		if (IS_ERR_OR_NULL(c)) {
			pr_err("Not able to get the clock for %s\n",
						uart_parent_clk[i].name);
			continue;
		}
		uart_parent_clk[i].parent_clk = c;
		uart_parent_clk[i].fixed_clk_rate = clk_get_rate(c);
	}
	ast_uart_pdata.parent_clk_list = uart_parent_clk;
	ast_uart_pdata.parent_clk_count = ARRAY_SIZE(uart_parent_clk);
    ast_loopback_uart_pdata.parent_clk_list = uart_parent_clk;
    ast_loopback_uart_pdata.parent_clk_count = ARRAY_SIZE(uart_parent_clk);
    ast_loopback_uart_pdata.is_loopback = true;
	tegra_uarta_device.dev.platform_data = &ast_uart_pdata;
	tegra_uartb_device.dev.platform_data = &ast_uart_pdata;
	tegra_uartc_device.dev.platform_data = &ast_uart_pdata;
	tegra_uartd_device.dev.platform_data = &ast_uart_pdata;
    /* UARTE is used for loopback test purpose */
    tegra_uarte_device.dev.platform_data = &ast_loopback_uart_pdata;

	/* Register low speed only if it is selected */
	if (!is_tegra_debug_uartport_hs()) {
		uart_debug_init();
		/* Clock enable for the debug channel */
		if (!IS_ERR_OR_NULL(debug_uart_clk)) {
			pr_info("The debug console clock name is %s\n",
						debug_uart_clk->name);
			c = tegra_get_clock_by_name("pll_p");
			if (IS_ERR_OR_NULL(c))
				pr_err("Not getting the parent clock pll_p\n");
			else
				clk_set_parent(debug_uart_clk, c);

			clk_enable(debug_uart_clk);
			clk_set_rate(debug_uart_clk, clk_get_rate(c));
		} else {
			pr_err("Not getting the clock %s for debug console\n",
					debug_uart_clk->name);
		}
	}

	platform_add_devices(ast_uart_devices,
				ARRAY_SIZE(ast_uart_devices));
}

static struct platform_device tegra_camera = {
	.name = "tegra_camera",
	.id = -1,
};

/* Levi add for ics */
static struct dock_platform_data dock_on_platform_data = {
		.irq		      = TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PS4),
		.dock_on_pin	  = TEGRA_GPIO_PS4,
        .dock_amp_enb_pin = TEGRA_GPIO_PB1,
		.ac_dock_pin	  = TEGRA_GPIO_PV1,
};
static struct platform_device tegra_dock_device =
{
    .name = "tegra_dock",
    .id   = -1,
    .dev = {
		.platform_data = &dock_on_platform_data,
	},
};

static int __init ast_hw_info_init(void)
{
	static bool flag = false;
	int ret = 0;
	int i = 0;
	int board_gpios[] = {
		TEGRA_GPIO_PR0,
		TEGRA_GPIO_PR1,
		TEGRA_GPIO_PR2,
		TEGRA_GPIO_PR3,
		TEGRA_GPIO_PR4,
	};

	int board_groups[] = {
		TEGRA_PINGROUP_KB_ROW0,
		TEGRA_PINGROUP_KB_ROW1,
		TEGRA_PINGROUP_KB_ROW2,
		TEGRA_PINGROUP_KB_ROW3,
		TEGRA_PINGROUP_KB_ROW4,
	};

	if (flag == false) {
		flag = true;

		for (i = 0; i < ARRAY_SIZE(board_gpios); i++) {
			tegra_gpio_enable(board_gpios[i]);
			ret = gpio_request(board_gpios[i], "HwInfo");
			if (ret < 0) {
				pr_err("%s: gpio_request failed for gpio #%d\n", __func__, board_gpios[i]);
				goto fail;
			}

			tegra_pinmux_set_pullupdown(board_groups[i], TEGRA_PUPD_PULL_DOWN);
			gpio_direction_input(board_gpios[i]);
			gpio_export(board_gpios[i], false);
		}

		s_hw_info.pj_id = ((gpio_get_value(board_gpios[1]) << 1) |
						gpio_get_value(board_gpios[0]));

		s_hw_info.hw_version = ((gpio_get_value(board_gpios[4]) << 2) |
						(gpio_get_value(board_gpios[3]) << 1) |
						gpio_get_value(board_gpios[2]));

	} else {
		pr_info("HwInfo : Already Initial.\n");
	}

	pr_info("HwInfo : project=%s, board=%s(%d)\n",
			(s_hw_info.pj_id==PROJECT_AVALON)? "Avalon":
			(s_hw_info.pj_id==PROJECT_SPHINX ? "Sphinx":
            (s_hw_info.pj_id==PROJECT_TITAN ? "Titan":"Unknown")),
			(s_hw_info.hw_version==BOARD_VERSION_WS) ? "WS" :
			(s_hw_info.hw_version==BOARD_VERSION_ES1 ? "ES1":
			(s_hw_info.hw_version==BOARD_VERSION_ES2 ? "ES2":
			(s_hw_info.hw_version==BOARD_VERSION_CS ? "CS":"MP"))), s_hw_info.hw_version);

	while (i--)
		gpio_free(board_gpios[i]);
	return 0;
fail:
	while (i--)
		gpio_free(board_gpios[i]);
	return ret;
}

void ast_get_hw_info(struct board_version *bv)
{
	ast_hw_info_init();
	memcpy(bv, &s_hw_info, sizeof(*bv));
}

#ifdef CONFIG_USB_ISDBT_POWER

static struct isdbt_platform_data isdbt_power_platform_data;
static struct regulator *ast_isdb_3V3 = NULL;
static struct regulator *ast_isdb_1V8 = NULL;
#define ISDBT_RESET_MP TEGRA_GPIO_PK2
#define ISDBT_RESET_CS TEGRA_GPIO_PI3
#define ISDBT_D1V8 TEGRA_GPIO_PP0
#define ISDBT_D3V3 TEGRA_GPIO_PO2
#define ISDBT_ON 1
#define ISDBT_OFF 0


static int isdbt_get_reset_pin(void)
{
	static bool flag = false;
	static int reset_pin_id = 0;
	struct board_version bv;

	if (flag == false) {

		flag = true;
		ast_get_hw_info(&bv);

		if (( bv.pj_id == PROJECT_TITAN ) & (bv.hw_version > BOARD_VERSION_CS)) {
			reset_pin_id = TEGRA_GPIO_PK2;
		} else {
			reset_pin_id = TEGRA_GPIO_PI3;
		}
	}
	return reset_pin_id;
}

static void isdbt_power_init(void) {

	int ret = 0;
	isdbt_power_platform_data.isdbt_reset_pin = isdbt_get_reset_pin();

	ret = gpio_request(isdbt_get_reset_pin(), "ISDBT_RESET");
    if (ret < 0)
        pr_err("%s(): gpio request() fails for gpio %d (ISDBT_RESET)\n",
            __func__, isdbt_get_reset_pin());
    else {
        tegra_gpio_enable(isdbt_get_reset_pin());
        gpio_direction_output(isdbt_get_reset_pin(), 0);
        gpio_export(isdbt_get_reset_pin(), false);
    }

}

static int isdbt_power_status(void) {

	int isdbt_1v8, isdbt_3v3 = 0;
	int rc = 0;

	if (ast_isdb_3V3== NULL) {

		ast_isdb_3V3 = regulator_get(NULL, "vdd_3v3_isdb_t");
		if (IS_ERR_OR_NULL(ast_isdb_3V3)) {
			rc = PTR_ERR(ast_isdb_3V3);
			pr_err("%s: unable to get vdd_3v3_isdb regulator, error %d\n", __FUNCTION__, rc);
			ast_isdb_3V3 = NULL;
			return rc;
		}
	}

	if (ast_isdb_1V8 == NULL) {
		ast_isdb_1V8 = regulator_get(NULL, "vdd_1v8_isdb_t");
		if (IS_ERR_OR_NULL(ast_isdb_1V8)) {
			rc = PTR_ERR(ast_isdb_1V8);
			pr_err("%s: unable to get vdd_1V8_isdb regulator, error %d\n", __FUNCTION__, rc);
			ast_isdb_1V8 = NULL;
			return rc;
		}
	}

    isdbt_3v3 = regulator_is_enabled(ast_isdb_3V3);
	printk(KERN_INFO "isdbt_3v3 = %d\n", isdbt_3v3 );
	isdbt_1v8 = regulator_is_enabled(ast_isdb_1V8);
	printk(KERN_INFO "isdbt_1v8 = %d\n", isdbt_1v8);

	if ((isdbt_3v3 == 1) && (isdbt_1v8 == 1))
		return ISDBT_ON;
	else
		return ISDBT_OFF;

}


static int isdbt_power_control(int enable) {

	int rc = 0;

	if (ast_isdb_3V3== NULL) {

		ast_isdb_3V3 = regulator_get(NULL, "vdd_3v3_isdb_t");
		if (IS_ERR_OR_NULL(ast_isdb_3V3)) {
			rc = PTR_ERR(ast_isdb_3V3);
			pr_err("%s: unable to get vdd_3v3_isdb regulator, error %d\n", __FUNCTION__, rc);
			ast_isdb_3V3 = NULL;
			return rc;
		}
	}

	if (ast_isdb_1V8 == NULL) {
		ast_isdb_1V8 = regulator_get(NULL, "vdd_1v8_isdb_t");
		if (IS_ERR_OR_NULL(ast_isdb_1V8)) {
			rc = PTR_ERR(ast_isdb_1V8);
			pr_err("%s: unable to get vdd_1V8_isdb regulator, error %d\n", __FUNCTION__, rc);
			ast_isdb_1V8 = NULL;
			return rc;
		}
	}
	if(enable == 1) {

		if (isdbt_power_status() == ISDBT_OFF) {

			gpio_direction_output(isdbt_get_reset_pin(), 0);

			rc = regulator_enable(ast_isdb_3V3);
			if (rc) {
				pr_err("%s: unable to enable vdd_3v3_isdb regulator, error %d\n", __FUNCTION__, rc);
				regulator_put(ast_isdb_3V3);
				return rc;
			}
			rc = regulator_enable(ast_isdb_1V8);
			if (rc) {
				pr_err("%s: unable to enable vdd_1V8_isdb regulator, error %d\n", __FUNCTION__, rc);
				regulator_put(ast_isdb_1V8);
				return rc;
			}
			msleep(10);
			gpio_direction_output(isdbt_get_reset_pin(), 1);

		}
		else {
			pr_err("%s: ISDBT_POWER_Already_ON \n", __FUNCTION__);
			return rc;
		}

	} else {

		if(isdbt_power_status() == ISDBT_ON) {

			gpio_direction_output(isdbt_get_reset_pin(), 0);

			rc = regulator_disable(ast_isdb_1V8);
			if (rc) {
				pr_err("%s: unable to disable vdd_1V8_isdb regulator, error %d\n", __FUNCTION__, rc);
				regulator_put(ast_isdb_1V8);
				return rc;
			}
			rc = regulator_disable(ast_isdb_3V3);
			if (rc) {
				pr_err("%s: unable to disable vdd_3v3_isdb regulator, error %d\n", __FUNCTION__, rc);
				regulator_put(ast_isdb_3V3);
				return rc;
			}

		} else {
			pr_err("%s: ISDBT_POWER_Already_OFF \n", __FUNCTION__);
			return rc;
		}
	}
	return rc;

}

static struct isdbt_platform_data isdbt_power_platform_data = {
	.enable_isdbt_regulator_control = isdbt_power_control,
	.isdbt_regulator_status = isdbt_power_status,
	.isdbt_reset_pin = 0, /*differ from board id*/
	.isdb_1v8 = ISDBT_D1V8,
	.isdb_3v3 = ISDBT_D3V3,
};

static struct platform_device isdbt_power_control_device = {
    .name = "isdbt_power",
    .id   = -1,
    .dev = {
		.platform_data = &isdbt_power_platform_data,
	},
};
#endif

#ifdef CONFIG_SENSORS_IQS128
static struct cap_sensor_platform_data cap_sensor_platform_data = {
		.irq		= TEGRA_GPIO_TO_IRQ(CAP_GPIO_INT),
		.gpio_num	= CAP_GPIO_INT,
};
static struct platform_device tegra_cap_device = {
	.name		= "tegra_cap",
	.id 		= -1,
	.dev		= {
		.platform_data = &cap_sensor_platform_data,
	},
};
#endif

#ifdef CONFIG_TEGRA_INFO_DEV
static struct platform_device tegra_info_device = {
    .name = "tegra_info",
    .id = -1,
};
#endif

/* avalon-specific & titan-specific spi devices */
/* static struct platform_device *avalon_spi_devices[] __initdata = { */
/*     &tegra_spi_device4, */
/* }; */

/* sphinx-specific spi devices */
static struct platform_device *sphinx_spi_devices[] __initdata = {
    &tegra_spi_device5,
};

struct spi_clk_parent spi_parent_clk[] = {
	[0] = {.name = "pll_p"},
#ifndef CONFIG_TEGRA_PLLM_RESTRICTED
	[1] = {.name = "pll_m"},
	[2] = {.name = "clk_m"},
#else
    [1] = {.name = "clk_m"},
#endif
};

static struct tegra_spi_platform_data ast_spi_pdata = {
	.is_dma_based		= true,
	.max_dma_buffer		= (16 * 1024),
	.is_clkon_always	= false,
	.max_rate		= 100000000,
};

static struct spi_board_info sphinx_spi_board_info[] __initdata = {
	{
	 .modalias = "spi_panel",
	 .bus_num = 4,
	 .chip_select = 2,
	 .mode = SPI_MODE_3,
	 .max_speed_hz = 5000000,
	 .platform_data = NULL,
	 .irq = 0,
	},
};

/* static void __init avalon_spi_init(void) */
/* { */
/* 	int i; */
/* 	struct clk *c; */

/* 	for (i = 0; i < ARRAY_SIZE(spi_parent_clk); ++i) { */
/* 		c = tegra_get_clock_by_name(spi_parent_clk[i].name); */
/* 		if (IS_ERR_OR_NULL(c)) { */
/* 			pr_err("Not able to get the clock for %s\n", */
/* 						spi_parent_clk[i].name); */
/* 			continue; */
/* 		} */
/* 		spi_parent_clk[i].parent_clk = c; */
/* 		spi_parent_clk[i].fixed_clk_rate = clk_get_rate(c); */
/* 	} */
/* 	ast_spi_pdata.parent_clk_list = spi_parent_clk; */
/* 	ast_spi_pdata.parent_clk_count = ARRAY_SIZE(spi_parent_clk); */
/* 	tegra_spi_device4.dev.platform_data = &ast_spi_pdata; */
/* 	platform_add_devices(avalon_spi_devices, */
/* 				ARRAY_SIZE(avalon_spi_devices)); */

/* } */

static void __init sphinx_spi_init(void)
{
	int i;
	struct clk *c;

    spi_register_board_info(sphinx_spi_board_info,
                            ARRAY_SIZE(sphinx_spi_board_info));

	for (i = 0; i < ARRAY_SIZE(spi_parent_clk); ++i) {
		c = tegra_get_clock_by_name(spi_parent_clk[i].name);
		if (IS_ERR_OR_NULL(c)) {
			pr_err("Not able to get the clock for %s\n",
						spi_parent_clk[i].name);
			continue;
		}
		spi_parent_clk[i].parent_clk = c;
		spi_parent_clk[i].fixed_clk_rate = clk_get_rate(c);
	}
	ast_spi_pdata.parent_clk_list = spi_parent_clk;
	ast_spi_pdata.parent_clk_count = ARRAY_SIZE(spi_parent_clk);
	tegra_spi_device5.dev.platform_data = &ast_spi_pdata;
	platform_add_devices(sphinx_spi_devices,
				ARRAY_SIZE(sphinx_spi_devices));

}

static void __init ast_spi_init(void)
{
    if (machine_is_avalon() || machine_is_titan()) {
        return;
        /* avalon_spi_init(); */
    } else if (machine_is_sphinx()) {
        sphinx_spi_init();
    }
}
static struct resource tegra_rtc_resources[] = {
	[0] = {
		.start = TEGRA_RTC_BASE,
		.end = TEGRA_RTC_BASE + TEGRA_RTC_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = INT_RTC,
		.end = INT_RTC,
		.flags = IORESOURCE_IRQ,
	},
};

static struct platform_device tegra_rtc_device = {
	.name = "tegra_rtc",
	.id   = -1,
	.resource = tegra_rtc_resources,
	.num_resources = ARRAY_SIZE(tegra_rtc_resources),
};

static struct tegra_wm8903_platform_data ast_audio_pdata = {
	.gpio_spkr_en		= TEGRA_GPIO_SPKR_EN,
	.gpio_hp_det		= TEGRA_GPIO_HP_DET,
	.gpio_hp_mute		= -1,
	.gpio_int_mic_en	= -1,
	.gpio_ext_mic_en	= -1,
	.gpio_ext_mic_det	= TEGRA_GPIO_AMIC_DET,
	.gpio_lineout_det	= TEGRA_GPIO_LINEOUT_DET,
};

static struct platform_device ast_audio_device = {
	.name	= "tegra-snd-wm8903",
	.id	= 0,
	.dev	= {
		.platform_data  = &ast_audio_pdata,
	},
};

static struct ast_tspdrv_platform_data ast_tspdrv_pdata = {
    .vibrator_en_pin  = TEGRA_GPIO_PH7,
    .pwm_id           = 3,
    .pwm_period       = 43000,  /* 23.44k */
    .pwm_duty_cycle   = 1,
#ifdef CONFIG_MACH_SPHINX
    .rail_name = "vdd_3v3_vibrator",
#else
    .rail_name = NULL,
#endif
};

static struct platform_device ast_tspdrv_device = {
    .name   = "tspdrv",
    .id     = -1,
    .dev    = {
        .platform_data = &ast_tspdrv_pdata,
    },
};

static struct platform_device *ast_devices[] __initdata = {
	&tegra_pmu_device,
	&tegra_rtc_device,
	&tegra_udc_device,
#if defined(CONFIG_TEGRA_IOVMM_SMMU)
	&tegra_smmu_device,
#endif
	&tegra_wdt_device,
#if defined(CONFIG_TEGRA_AVP)
	&tegra_avp_device,
#endif
	&tegra_camera,
#if defined(CONFIG_CRYPTO_DEV_TEGRA_SE)
	&tegra_se_device,
#endif
	&tegra_ahub_device,
	&tegra_dam_device0,
	&tegra_dam_device1,
	&tegra_dam_device2,
	&tegra_i2s_device1,
	&tegra_i2s_device3,
	&tegra_spdif_device,
	&spdif_dit_device,
	&bluetooth_dit_device,
    &ast_bcm4329_rfkill_device,
	&tegra_pcm_device,
	&ast_audio_device,
	&tegra_hda_device,
#if defined(CONFIG_CRYPTO_DEV_TEGRA_AES)
	&tegra_aes_device,
#endif
	&tegra_dock_device,  /* Levi add for ics */
	&tegra_pwfm3_device, /* added by Levi */
#ifdef CONFIG_USB_ISDBT_POWER
	&isdbt_power_control_device,
#endif
#ifdef CONFIG_SENSORS_IQS128
	&tegra_cap_device,
#endif
#ifdef CONFIG_TEGRA_INFO_DEV
    &tegra_info_device,
#endif
#ifdef CONFIG_TSPDRV
    &ast_tspdrv_device,
#endif
};

/* touch: please add avalon-specific here */
#ifdef CONFIG_TOUCHSCREEN_GOODIX_BIG
static struct goodix_i2c_platform_data goodix_data = {
	.gpio_reset = TEGRA_GPIO_PH6,
};

static struct i2c_board_info __initdata goodix_i2c_info[] = {
    {
	  I2C_BOARD_INFO("Goodix-TS", 0x55),
	 .irq = TEGRA_GPIO_TO_IRQ(TEGRA_GPIO_PH4),
	 .platform_data = &goodix_data,
    },
};


static int __init avalon_touch_init(void)
{
    tegra_gpio_enable(TEGRA_GPIO_PH4);
    tegra_gpio_enable(TEGRA_GPIO_PH6);
    i2c_register_board_info(1, goodix_i2c_info, 1);

    return 0;
}
#else
static int __init avalon_touch_init(void) { return 0; }
#endif

/* touch: please add sphinx-specific here */
#ifdef CONFIG_MACH_SPHINX
#define TOUCH_GPIO_IRQ_ATMEL_T9        	TEGRA_GPIO_PO7
#define TOUCH_GPIO_RST_ATMEL_T9        	TEGRA_GPIO_PO0
#define TOUCH_BUS_ATMEL_T9             	1
#define MXT_I2C_ADDRESS        		MXT768E_I2C_ADDR1
#define MXT_CONFIG_CRC_768E  		0xA5A0AF

static const u8 config_768e_v2_0[] = {
	0x20, 0xFF, 0x32, 					    // T7
	0x40, 0x00, 0x05, 0x01, 0x00, 0x00, 0xFF, 0x01, 0x00, 0x00, // T8
	0x8B, 0x00, 0x00, 0x18, 0x20, 0x00, 0xB0, 0x2A, 0x02, 0x07, // T9, instance 0
	0x0A, 0x0A, 0x01, 0x0D, 0x0A, 0x14, 0x00, 0x14, 0x1F, 0x03,
	0xFF, 0x04, 0x00, 0x00, 0x00, 0x00, 0x88, 0x32, 0x88, 0x28,
	0x18, 0x0F, 0x2B, 0x33, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // T9, instance 1
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // T15, instance 0
	0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // T15, instance 1
	0x00,
	0x00, 0x00, 						    // T18
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // T19
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // T25
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 				    // T40, instance 0
	0x00, 0x00, 0x00, 0x00, 0x00, 				    // T40, instance 1
	0x03, 0x0F, 0x64, 0x40, 0xE0, 0x02, 0x00, 0x00, 0xC8, 0xC8, // T42, instance 0
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // T42, instance 1
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 		    // T43
	0x00, 0x00, 0x18, 0x18, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, // T46
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // T47, instance 0
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // T47, instance 1
	0x03, 0x88, 0xF2, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // T48
	0xB0, 0x1B, 0x00, 0x06, 0x06, 0x00, 0x00, 0x30, 0x04, 0x40,
	0x00, 0x00, 0x14, 0x00, 0x00, 0x00, 0x00, 0x0F, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x70, 0x28, 0x02, 0x10, 0x02, 0x50,
	0x0A, 0x14, 0x00, 0xF9, 0xF9, 0x05, 0x05, 0x8F, 0x32, 0x88,
	0x1E, 0x40, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // T52, instance 0
	0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // T52, instance 1
	0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 		 	    // T55, instance 0
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00,			    // T55, instance 1
	0x00, 0x00, 0x01, 0x2F, 0x0E, 0x0F, 0x0F, 0x10, 0x0F, 0x11, // T56
	0x10, 0x10, 0x10, 0x10, 0x11, 0x10, 0x10, 0x10, 0x10, 0x10,
	0x10, 0x10, 0x0F, 0x0F, 0x0E, 0x0D, 0x0C, 0x00, 0x30, 0x01,
	0x01, 0x1B, 0x04, 0x00,
	0x83, 0x0F, 0x00,  					    // T57, instance 0
	0x00, 0x00, 0x00					    // T57, instance 1
};

static u8 read_chg(void)
{
    return gpio_get_value(TOUCH_GPIO_IRQ_ATMEL_T9);
}

static struct mxt_platform_data atmel_mxt768e_info = {
    	.x_line         = 24,
	.y_line         = 32,
	.x_size         = 800,
	.y_size         = 1280,
	.blen           = 0xB0,
	.threshold      = 0x3C,
	//.voltage        = 3300000,              /* 3.3V */
	.orient         = 7,
	.config         = config_768e_v2_0,
	.config_length  = sizeof(config_768e_v2_0),
	.config_crc     = MXT_CONFIG_CRC_768E,
	.config_fw_ver	= 0x20,			// firmware version 2.0
	.irqflags       = IRQF_TRIGGER_FALLING,
	.read_chg       = &read_chg,
};

static struct i2c_board_info __initdata atmxt768e_i2c_info[] = {
    {
      I2C_BOARD_INFO("atmel_mxt_ts", MXT_I2C_ADDRESS),
      .irq = TEGRA_GPIO_TO_IRQ(TOUCH_GPIO_IRQ_ATMEL_T9),
      .platform_data = &atmel_mxt768e_info,
    },
};

static int __init sphinx_touch_init(void)
{
    int ret = 0;

    ret = gpio_request(TOUCH_GPIO_RST_ATMEL_T9, "touch-reset");
    if (ret < 0)
        pr_err("%s(): gpio request() fails for gpio %d (touch-reset)\n",
            __func__, TOUCH_GPIO_RST_ATMEL_T9);
    else {
        tegra_gpio_enable(TOUCH_GPIO_RST_ATMEL_T9);
        gpio_direction_output(TOUCH_GPIO_RST_ATMEL_T9, 1);
        gpio_export(TOUCH_GPIO_RST_ATMEL_T9, false);
    }

    ret = gpio_request(TOUCH_GPIO_IRQ_ATMEL_T9, "touch-irq");
    if (ret < 0)
        pr_err("%s(): gpio request() fails for gpio %d (touch-irq)\n",
            __func__, TOUCH_GPIO_IRQ_ATMEL_T9);
    else {
        gpio_direction_input(TOUCH_GPIO_IRQ_ATMEL_T9);
        gpio_export(TOUCH_GPIO_IRQ_ATMEL_T9, false);
        tegra_gpio_enable(TOUCH_GPIO_IRQ_ATMEL_T9);
    }

    msleep(150);

    i2c_register_board_info(TOUCH_BUS_ATMEL_T9, atmxt768e_i2c_info, 1);

    return ret;
}
#else
static int __init sphinx_touch_init(void) { return 0; }
#endif

#ifdef CONFIG_MACH_TITAN

#define MXT_IRQ_GPIO	TEGRA_GPIO_PH4
#define MXT_RESET_GPIO	TEGRA_GPIO_PH6
#define MXT_WAKE_GPIO	TEGRA_GPIO_PI7
#define AC_GPIO		TEGRA_GPIO_PV1
#define HDMI_GPIO	TEGRA_GPIO_PN7

#define MXT_I2C_ADDRESS	MXT1386_I2C_ADDR1

#define MXT_CONFIG_CRC_1716E  0x5BC12D

static const u8 config_1716e[] = {	/* CRC related */
	0x3E, 0x0A, 0x0A,						/* T7  [ 3]	*/
	0x17, 0x00, 0x0A, 0x0A, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00,	/* T8  [10]	*/
	0x8F, 0x00, 0x00, 0x21, 0x34, 0x00, 0x10, 0x46, 0x02, 0x05,	/* T9  [35]	*/
	0x14, 0x05, 0x05, 0x0F, 0x0a, 0x14, 0x19, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x09, 0x14, 0x00, 0x00, 0x00,
	0x00, 0x00,							/* T18 [ 2]	*/
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,	/* T24 [19]	*/
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x03, 0x00, 0x48, 0x71, 0xf0, 0x55,				/* T25 [ 6]	*/
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,			/* T27 [ 7]	*/
	0x00, 0x00, 0x00, 0x00, 0x00,					/* T40 [ 5]	*/
	0x31, 0x1E, 0x46, 0x3C, 0x00, 0x00, 0x00, 0x00, 0x0A, 0x02,	/* T42 [10]	*/
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,	/* T43 [11]	*/
	0x00,
	0x40, 0x00, 0x10, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00,		/* T46 [ 9]	*/
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,	/* T47 [10]	*/
	0x01, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,	/* T48 [54]	*/
	0x10, 0x23, 0x00, 0x00, 0x00, 0x00, 0x00, 0x30, 0x04, 0x40,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x24, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,	/* T56 [51]	*/
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00,
};

static struct mxt_cfg_noise cfg_noises[] = {
#if 0
	{.type = 9,	.offset = 7,	.noise_value = 80, },
	{.type = 9,	.offset = 8,	.noise_value = 4, },
#endif
	{.type = 9,	.offset = 34,	.noise_value = 2, },
	{.type = 46,	.offset = 3,	.noise_value = 24, },
};

static struct mxt_cfg_object cfg_object[] = {
	{.type = 24,	.length = 1,},
	{.type = 27,	.length = 1,},
	{.type = 40,	.length = 1,},
	{.type = 43,	.length = 1,},
	{.type = 47,	.length = 1,},

	{.type = 18,	.length = 2,},
	{.type = 25,	.length = 6,},
	{.type = 46,	.length = 7,},
	{.type = 56,	.length = 5,},

	{.type = 7,	.length = 3,},
	{.type = 8,	.length = 10,},
	{.type = 9,	.length = 35,},
	{.type = 42,	.length = 10,},
	{.type = 48,	.length = 29,},
	{.type = 38,	.length = 5,},
};

static struct mxt_platform_data atmel_mxt_info = {
	.x_line         = 33,
	.y_line         = 52,
	.x_size         = 4096,
	.y_size         = 4096,
	.blen           = 0x10,
	.threshold      = 0x3C,
	.voltage        = 3300000,              /* 3.3V */
	.orient         = 5,
	.config         = config_1716e,
	.config_length  = sizeof(config_1716e),
	.config_crc     = MXT_CONFIG_CRC_1716E,
	.config_fw_ver	= 0x00,			// firmware version 2.0 and 2.1
	.irqflags       = IRQF_TRIGGER_FALLING,
/*	.read_chg       = &read_chg, */
	.read_chg       = NULL,

	.cfg_objects	= &cfg_object[0],
	.cfg_objects_length = ARRAY_SIZE(cfg_object),
	.cfg_noises	= &cfg_noises[0],
	.cfg_noises_length = ARRAY_SIZE(cfg_noises),
	.gpio_wake	= MXT_WAKE_GPIO,
	.gpio_reset	= MXT_RESET_GPIO,
	.gpio_ac	= AC_GPIO,
	.gpio_hdmi	= HDMI_GPIO,
};

static struct i2c_board_info __initdata atmel_i2c_info[] = {
	{
		I2C_BOARD_INFO("atmel_mxt_ts", MXT_I2C_ADDRESS),
		.irq = TEGRA_GPIO_TO_IRQ(MXT_IRQ_GPIO),
		.platform_data = &atmel_mxt_info,
	}
};


static int titan_touch_init(void)
{

	tegra_gpio_enable(MXT_IRQ_GPIO);
	tegra_gpio_enable(MXT_WAKE_GPIO);
	tegra_gpio_enable(MXT_RESET_GPIO);

	gpio_request(MXT_IRQ_GPIO, "ts_irq");
	gpio_request(MXT_WAKE_GPIO, "ts_wake");
	gpio_request(MXT_RESET_GPIO, "ts_reset");

	gpio_export(MXT_IRQ_GPIO, false);
	gpio_export(MXT_WAKE_GPIO, false);
	gpio_export(MXT_RESET_GPIO, false);

	gpio_direction_input(MXT_IRQ_GPIO);
	gpio_direction_output(MXT_WAKE_GPIO, 0);
	gpio_direction_output(MXT_RESET_GPIO, 0);

	i2c_register_board_info(1, atmel_i2c_info, 1);
	return 0;
}

static int titan_touch_power_enable(void)
{
	struct regulator *titan_touch_3v3 = NULL;
	titan_touch_3v3 = regulator_get(NULL, "vdd_3v3_ts");

	if (IS_ERR(titan_touch_3v3)) {
		pr_err("%s failed to get vdd_3v3_ts\n", __func__);
		return -1;
	}

	if (regulator_enable(titan_touch_3v3)) {
		pr_err("%s: unable to enable vdd_3v3_ts\n", __func__);
		regulator_put(titan_touch_3v3);
		return -1;
	}
	/* based on 1716E's power sequence,
	 * it should be longer than 10ms */
	mdelay(11);
	gpio_set_value(MXT_RESET_GPIO, 1);
	return 0;
}
fs_initcall(titan_touch_power_enable);

#else
static int titan_touch_init(void)
{
	return 0;
}
#endif

static int __init ast_touch_init(void)
{
    if (machine_is_avalon()) {
        return avalon_touch_init();
    } else if (machine_is_sphinx()) {
        return sphinx_touch_init();
    } else if (machine_is_titan()) {
        return titan_touch_init();
    }

	return 0;
}

static struct tegra_ehci_platform_data tegra_ehci_pdata[] = {
	[0] = {
			.phy_config = &utmi_phy_config[0],
			.operating_mode = TEGRA_USB_HOST,
			.power_down_on_bus_suspend = 1,
            .default_enable = true,
	},
	[1] = {
			.phy_config = &utmi_phy_config[1],
			.operating_mode = TEGRA_USB_HOST,
			.power_down_on_bus_suspend = 1,
            .default_enable = true,
	},
	[2] = {
			.phy_config = &utmi_phy_config[2],
			.operating_mode = TEGRA_USB_HOST,
			.power_down_on_bus_suspend = 1,
			.hotplug = 1,
            .default_enable = true,
	},
};

static struct tegra_otg_platform_data tegra_otg_pdata = {
	.ehci_device = &tegra_ehci1_device,
	.ehci_pdata = &tegra_ehci_pdata[0],
};

/* #define SERIAL_NUMBER_LENGTH 20 */
/* static char usb_serial_num[SERIAL_NUMBER_LENGTH]; */

#ifdef CONFIG_USB_SUPPORT
static struct usb_phy_plat_data tegra_usb_phy_pdata[] = {
	[0] = {
			.instance = 0,
			.vbus_gpio = -1,
			.vbus_reg_supply = "vdd_vbus_micro_usb",
	},
	[1] = {
			.instance = 1,
			.vbus_gpio = -1,
	},
	[2] = {
			.instance = 2,
			.vbus_gpio = -1,
			/* .vbus_reg_supply = "vdd_vbus_typea_usb", */
	},
};

static void __init ast_usb_init(void)
{
	tegra_usb_phy_init(tegra_usb_phy_pdata,
			ARRAY_SIZE(tegra_usb_phy_pdata));

	tegra_otg_device.dev.platform_data = &tegra_otg_pdata;
	platform_device_register(&tegra_otg_device);

    tegra_ehci2_device.dev.platform_data = &tegra_ehci_pdata[1];
    platform_device_register(&tegra_ehci2_device);

	tegra_ehci3_device.dev.platform_data = &tegra_ehci_pdata[2];
	platform_device_register(&tegra_ehci3_device);
}
#else
static void __init ast_usb_init(void) { }
#endif

static void __init ast_gps_init(void)
{
	tegra_gpio_enable(TEGRA_GPIO_PU2);
	tegra_gpio_enable(TEGRA_GPIO_PU3);
}

/* nfc: avalon-specific */
static void __init avalon_nfc_init(void)
{
    tegra_gpio_enable(TEGRA_GPIO_PX7);
	tegra_gpio_enable(TEGRA_GPIO_PS7);
	tegra_gpio_enable(TEGRA_GPIO_PQ7);

    i2c_register_board_info(0, avalon_i2c_gen1_board_info, 1);
}

/* nfc: sphinx-specific */
static void __init sphinx_nfc_init(void)
{
    tegra_gpio_enable(TEGRA_GPIO_PS7);
	tegra_gpio_enable(TEGRA_GPIO_PD2);
	tegra_gpio_enable(TEGRA_GPIO_PS2);

    i2c_register_board_info(3, sphinx_i2c_ddc_board_info, 1);
}

static void __init ast_nfc_init(void)
{
    if (machine_is_avalon() || machine_is_titan()) {
        avalon_nfc_init();
    } else if (machine_is_sphinx()) {
        sphinx_nfc_init();
    }
}
#define R_EN_3V3_MODEM TEGRA_GPIO_PP0
#define R_ON1 TEGRA_GPIO_PO2
#define W_DISABLE_N TEGRA_GPIO_PV6
#define WW_WAKE TEGRA_GPIO_PN2
#define R_HW_READY_3G TEGRA_GPIO_PQ6
#define R_SW_READY_3G TEGRA_GPIO_PQ5
#define R_TX_ON_3G TEGRA_GPIO_PQ4

static void __init ast_modem_init(void)
{
    int ret;

    if (machine_is_titan()) {
        pr_info("%s: No 3G module init\n",__func__);
        return;
    }

    pr_info("%s: Enable Ericsson C5621 control pin \n",__func__);
    /*-------To enable R_EN_3V3_MODEM  to provide VBAT_3G -------*/
    ret = gpio_request(R_EN_3V3_MODEM, "MODEM_VBAT");
    if (ret < 0)
        pr_err("%s: gpio_request failed for gpio %d\n",__func__, R_EN_3V3_MODEM);
    gpio_direction_output(R_EN_3V3_MODEM, 1);
    tegra_gpio_enable(R_EN_3V3_MODEM);
    gpio_export(R_EN_3V3_MODEM, true);
    mdelay(200);
    /*-------To enable R_ON_1  for power on modem-------*/
    ret = gpio_request(R_ON1, "R_ON_1");
    if (ret < 0)
        pr_err("%s: gpio_request failed for gpio %d\n",__func__, R_ON1);
    gpio_direction_output(R_ON1, 1);
    tegra_gpio_enable(R_ON1);
    gpio_export(R_ON1, true);
    /*-------To enable W_DISABLE_N for enable RF function---------*/
    ret = gpio_request(W_DISABLE_N, "W_DISABLE_N");
    if (ret < 0)
        pr_err("%s: gpio_request failed for gpio %d\n",__func__, W_DISABLE_N);
    gpio_direction_output(W_DISABLE_N, 1);
    tegra_gpio_enable(W_DISABLE_N);
    gpio_export(W_DISABLE_N, true);
    /*-------To enable WW_WAKE ------- */
    ret = gpio_request(WW_WAKE, "R_HW_READY_3G");
    if (ret < 0)
        pr_err("%s: gpio_request failed for gpio %d\n",__func__, WW_WAKE);
    gpio_direction_input(WW_WAKE);
    tegra_gpio_enable(WW_WAKE);
    gpio_export(WW_WAKE, true);
    /*-------To enable R_HW_READY_3G ------- */
    ret = gpio_request(R_HW_READY_3G, "R_HW_READY_3G");
    if (ret < 0)
        pr_err("%s: gpio_request failed for gpio %d\n",__func__, R_HW_READY_3G);
    gpio_direction_input(R_HW_READY_3G);
    tegra_gpio_enable(R_HW_READY_3G);
    gpio_export(R_HW_READY_3G, true);
    /*-------To enable R_SW_READY_3G ------- */
    ret = gpio_request(R_SW_READY_3G, "R_SW_READY_3G");
    if (ret < 0)
        pr_err("%s: gpio_request failed for gpio %d\n",__func__, R_SW_READY_3G);
    gpio_direction_input(R_SW_READY_3G);
    tegra_gpio_enable(R_SW_READY_3G);
    gpio_export(R_SW_READY_3G, true);
    /*-------To enable R_TX_ON_3G ------- */
    ret = gpio_request(R_TX_ON_3G, "R_EX_ON_3G");
    if (ret < 0)
        pr_err("%s: gpio_request failed for gpio %d\n",__func__, R_TX_ON_3G);
    gpio_direction_input(R_TX_ON_3G);
    tegra_gpio_enable(R_TX_ON_3G);
    gpio_export(R_TX_ON_3G, true);
}

#ifdef CONFIG_SATA_AHCI_TEGRA
static void ast_sata_init(void)
{
	platform_device_register(&tegra_sata_device);
}
#else
static void ast_sata_init(void) { }
#endif

#ifdef CONFIG_SWITCH_AST_DOCK
static void __init ast_dock_init(void)
{
    gpio_request(TEGRA_GPIO_PS4, "dock_on");
    gpio_direction_input(TEGRA_GPIO_PS4);
    tegra_gpio_enable(TEGRA_GPIO_PS4);

    gpio_request(TEGRA_GPIO_PB1, "dock_amp_enb");
    gpio_direction_output(TEGRA_GPIO_PB1, 0);
    tegra_gpio_enable(TEGRA_GPIO_PB1);
}
#else
static void __init ast_dock_init(void) { }
#endif

#ifdef CONFIG_TSPDRV
static void ast_tspdrv_init(void)
{
    gpio_request(TEGRA_GPIO_PH7, "vib_amp_shutdown");
    gpio_direction_output(TEGRA_GPIO_PH7, 0);
    tegra_gpio_enable(TEGRA_GPIO_PH7);
}
#else
static void ast_tspdrv_init(void) { }
#endif

static void __init tegra_ast_init(void)
{
    tegra_thermal_init(&thermal_data);
	tegra_clk_init_from_table(ast_clk_init_table);
#ifdef CONFIG_USB_ISDBT_POWER
	isdbt_power_init();
#endif
	ast_pinmux_init();
	ast_i2c_init();
	ast_spi_init();
	ast_usb_init();
#ifdef CONFIG_TEGRA_EDP_LIMITS
	ast_edp_init();
#endif
	ast_uart_init();
	ast_tsensor_init();
	platform_add_devices(ast_devices, ARRAY_SIZE(ast_devices));
    tegra_ram_console_debug_init();
#ifdef CONFIG_SND_SOC_FM34
	ast_fm34_init();
#endif
	ast_sdhci_init();
	ast_regulator_init();
	ast_gpio_switch_regulator_init();
	ast_ec_init();		/* Daniel Wang */
	ast_suspend_init();
	ast_power_off_init();
	ast_touch_init();
	ast_gps_init();
	ast_modem_init();
	/* ast_kbc_init(); */
	/* ast_scroll_init(); */
	ast_keys_init();
	ast_panel_init();
	/* ast_pmon_init(); */
	ast_sensors_init();
	ast_setup_bluesleep();
	ast_sata_init();
	//audio_wired_jack_init();
	ast_pins_state_init();
	ast_emc_init();
	tegra_release_bootloader_fb();
	ast_nfc_init();
#ifdef CONFIG_TEGRA_WDT_RECOVERY
    tegra_wdt_recovery_init();
#endif
    ast_dock_init();
    ast_tspdrv_init();
}

static void __init tegra_ast_reserve(void)
{
#if defined(CONFIG_NVMAP_CONVERT_CARVEOUT_TO_IOVMM)
        if (machine_is_titan()) {
            tegra_reserve(0, SZ_8M + SZ_4M, SZ_8M + SZ_4M);
        }else{
	    tegra_reserve(0, SZ_8M + SZ_1M, SZ_8M + SZ_1M);
        }
#else
	tegra_reserve(SZ_128M, SZ_8M, SZ_8M);
#endif
	tegra_ram_console_debug_reserve(SZ_1M);
}

#ifdef CONFIG_MACH_AVALON
MACHINE_START(AVALON, "avalon")
	.boot_params    = 0x80000100,
	.map_io         = tegra_map_common_io,
	.reserve        = tegra_ast_reserve,
	.init_early	= tegra_init_early,
	.init_irq       = tegra_init_irq,
	.timer          = &tegra_timer,
	.init_machine   = tegra_ast_init,
MACHINE_END
#endif

#ifdef CONFIG_MACH_SPHINX
MACHINE_START(SPHINX, "sphinx")
	.boot_params    = 0x80000100,
	.map_io         = tegra_map_common_io,
	.reserve        = tegra_ast_reserve,
	.init_early	= tegra_init_early,
	.init_irq       = tegra_init_irq,
	.timer          = &tegra_timer,
	.init_machine   = tegra_ast_init,
MACHINE_END
#endif

#ifdef CONFIG_MACH_TITAN
MACHINE_START(TITAN, "titan")
	.boot_params    = 0x80000100,
	.map_io         = tegra_map_common_io,
	.reserve        = tegra_ast_reserve,
	.init_early	= tegra_init_early,
	.init_irq       = tegra_init_irq,
	.timer          = &tegra_timer,
	.init_machine   = tegra_ast_init,
MACHINE_END
#endif
