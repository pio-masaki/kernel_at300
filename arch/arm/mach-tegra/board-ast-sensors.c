/*
 * arch/arm/mach-tegra/board-ast-sensors.c
 *
 * Copyright (c) 2010-2011, NVIDIA CORPORATION, All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 * Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 *
 * Neither the name of NVIDIA CORPORATION nor the names of its contributors
 * may be used to endorse or promote products derived from this software
 * without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
 * IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
 * TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/clk.h>
#include <linux/slab.h>
#include <linux/nct1008.h>
#ifdef CONFIG_TEGRA_EDP_LIMITS
#include <mach/edp.h>
#include <mach/thermal.h>
#endif
#include <linux/mpu.h>
#include <media/mt9p111.h>
#include <media/mt9d115.h>
#include <media/tps61050.h>

#include "gpio-names.h"
#include "board-ast.h"
#include "cpu-tegra.h"

#include <asm/mach-types.h>

static struct regulator *ast_vddio_vi = NULL;
static struct regulator *ast_vddio_1v8_cam1 = NULL;
static struct regulator *ast_vdd_1v8_cam2 = NULL;
static struct regulator *ast_avdd_2v8_cam1 = NULL;
static struct regulator *ast_vddio_1v8_cam2 = NULL;
static struct regulator *ast_vdd_2v8_cam1 = NULL;
static struct regulator *ast_avdd_2v8_cam2 = NULL;
static struct regulator *ast_vdd_3v3_cam1 = NULL;

static struct clk *vi_sensor_clk = NULL;
static struct clk *csus_clk = NULL;
static int ast_mt9d115_led_state = 0;

static int ast_camera_mclk_on(int rate)
{
	if (!ast_vddio_vi) {
		ast_vddio_vi = regulator_get(NULL, "vddio_vi");
		if (WARN_ON(IS_ERR(ast_vddio_vi))) {
			pr_err("%s: couldn't get regulator vddio_vi: %ld\n",
			       __func__, PTR_ERR(ast_vddio_vi));
			ast_vddio_vi = NULL;
			return -ENODEV;
		}
	}

	regulator_enable(ast_vddio_vi);
	if (!vi_sensor_clk) {
		vi_sensor_clk = clk_get_sys("tegra_camera", "vi_sensor");
		if (IS_ERR_OR_NULL(vi_sensor_clk)) {
			pr_err("%s: unable to get vi_sensor clock: %ld\n",
			       __func__, PTR_ERR(vi_sensor_clk));
			vi_sensor_clk = NULL;
			return -ENODEV;
		}
	}

	if (!csus_clk) {
		csus_clk = clk_get_sys("tegra_camera", "csus");
		if (IS_ERR_OR_NULL(csus_clk)) {
			pr_err("%s: unable to get csus clock: %ld\n", __func__,
			       PTR_ERR(csus_clk));
			csus_clk = NULL;
			return -ENODEV;
		}
	}

	clk_set_rate(vi_sensor_clk, rate);
	clk_enable(vi_sensor_clk);
	clk_enable(csus_clk);

	return 0;
}

static int ast_camera_mclk_off(void)
{
	if (csus_clk) {
		clk_disable(csus_clk);
		clk_put(csus_clk);
		csus_clk = NULL;
	}

	if (vi_sensor_clk) {
		clk_disable(vi_sensor_clk);
		clk_put(vi_sensor_clk);
		vi_sensor_clk = NULL;
	}

	if (ast_vddio_vi)
		regulator_disable(ast_vddio_vi);

	return 0;
}

#ifdef CONFIG_VIDEO_MT9P111
static int ast_mt9p111_init(void)
{
	int err;

	if (machine_is_titan()) {
		if (!ast_vdd_3v3_cam1) {
			ast_vdd_3v3_cam1 = regulator_get(NULL, "vdd_3v3_cam1");
			if (WARN_ON(IS_ERR(ast_vdd_3v3_cam1))) {
				pr_err("%s: couldn't get regulator vdd_3v3_cam1: %ld\n",
			               __func__, PTR_ERR(ast_vdd_3v3_cam1));
				ast_vdd_3v3_cam1 = NULL;
				return -ENODEV;
			}
		}

		if (!ast_avdd_2v8_cam1) {
			ast_avdd_2v8_cam1 = regulator_get(NULL,
				"avdd_2v8_cam1_titan");
			if (WARN_ON(IS_ERR(ast_avdd_2v8_cam1))) {
				pr_err("%s: couldn't get regulator avdd_2v8_cam1: %ld\n",
			               __func__, PTR_ERR(ast_avdd_2v8_cam1));
				ast_avdd_2v8_cam1 = NULL;
				return -ENODEV;
			}
		}
	} else {
		if (!ast_avdd_2v8_cam1) {
			ast_avdd_2v8_cam1 = regulator_get(NULL,
				"avdd_2v8_cam1");
			if (WARN_ON(IS_ERR(ast_avdd_2v8_cam1))) {
				pr_err("%s: couldn't get regulator avdd_2v8_cam1: %ld\n",
			               __func__, PTR_ERR(ast_avdd_2v8_cam1));
				ast_avdd_2v8_cam1 = NULL;
				return -ENODEV;
			}
		}

		if (!ast_vdd_2v8_cam1) {
			ast_vdd_2v8_cam1 = regulator_get(NULL, "vdd_2v8_cam1");
			if (WARN_ON(IS_ERR(ast_vdd_2v8_cam1))) {
				pr_err("%s: couldn't get regulator vdd_2v8_cam1: %ld\n",
				       __func__, PTR_ERR(ast_vdd_2v8_cam1));
				ast_vdd_2v8_cam1 = NULL;
				return -ENODEV;
			}
		}
	}

	if (!ast_vddio_1v8_cam1) {
		ast_vddio_1v8_cam1 = regulator_get(NULL, "vddio_1v8_cam1");
		if (WARN_ON(IS_ERR(ast_vddio_1v8_cam1))) {
			pr_err("%s: couldn't get regulator vddio_1v8_cam1: %ld\n",
			__func__, PTR_ERR(ast_vddio_1v8_cam1));
			ast_vddio_1v8_cam1 = NULL;
			return -ENODEV;
		}
	}

	err = gpio_request(CAM1_PWDN_GPIO, "cam1_power_down");
	if (err < 0)
		return err;

	gpio_direction_output(CAM1_PWDN_GPIO, 0);
	tegra_gpio_enable(CAM1_PWDN_GPIO);
	err = gpio_request(CAM1_RST_GPIO, "cam1_reset");
	if (err < 0)
		return err;

	gpio_direction_output(CAM1_RST_GPIO, 0);
	tegra_gpio_enable(CAM1_RST_GPIO);

	return 0;
}

static int ast_mt9p111_power_on(void)
{
	if (machine_is_titan()) {
		if (!ast_vddio_1v8_cam1 || !ast_avdd_2v8_cam1 ||
			!ast_vdd_3v3_cam1)
			return -ENODEV;
	} else {
		if (!ast_vddio_1v8_cam1 || !ast_avdd_2v8_cam1 ||
			!ast_vdd_2v8_cam1)
			return -ENODEV;
	}

	if (machine_is_titan()) {
		regulator_enable(ast_vddio_1v8_cam1);
		regulator_enable(ast_vdd_3v3_cam1);
		regulator_enable(ast_avdd_2v8_cam1);
		udelay(100);
		gpio_set_value(CAM1_RST_GPIO, 1);
		udelay(100);
		ast_camera_mclk_on(24000000);
	} else {
		regulator_enable(ast_vddio_1v8_cam1);
		regulator_enable(ast_avdd_2v8_cam1);
		regulator_enable(ast_vdd_2v8_cam1);
		udelay(100);
		gpio_set_value(CAM1_RST_GPIO, 1);
		udelay(100);
		ast_camera_mclk_on(24000000);
	}

	return 0;
}

static int ast_mt9p111_power_off(void)
{
	if (machine_is_titan()) {
		if (!ast_vddio_1v8_cam1 || !ast_avdd_2v8_cam1 ||
			!ast_vdd_3v3_cam1)
			return -ENODEV;
	} else {
		if (!ast_vddio_1v8_cam1 || !ast_avdd_2v8_cam1 ||
			!ast_vdd_2v8_cam1)
			return -ENODEV;
	}

	if (machine_is_titan()) {
		gpio_set_value(CAM1_RST_GPIO, 0);
		ast_camera_mclk_off();
		regulator_disable(ast_avdd_2v8_cam1);
		regulator_disable(ast_vdd_3v3_cam1);
		regulator_disable(ast_vddio_1v8_cam1);
	} else {
		gpio_set_value(CAM1_RST_GPIO, 0);
		ast_camera_mclk_off();
		regulator_disable(ast_vdd_2v8_cam1);
		regulator_disable(ast_avdd_2v8_cam1);
		regulator_disable(ast_vddio_1v8_cam1);
	}

	return 0;
}

struct mt9p111_platform_data ast_mt9p111_data = {
	.init = ast_mt9p111_init,
	.power_on = ast_mt9p111_power_on,
	.power_off = ast_mt9p111_power_off,
};
#endif

#ifdef CONFIG_VIDEO_MT9D115
static int ast_mt9d115_init(void)
{
	int err;

	if (!ast_vdd_1v8_cam2) {
		ast_vdd_1v8_cam2 = regulator_get(NULL, "vdd_1v8_cam2");
		if (WARN_ON(IS_ERR(ast_vdd_1v8_cam2))) {
			pr_err("%s: couldn't get regulator vdd_1v8_cam2: %ld\n",
			       __func__, PTR_ERR(ast_vdd_1v8_cam2));
			ast_vdd_1v8_cam2 = NULL;
			return -ENODEV;
		}
	}

	if (!ast_vddio_1v8_cam2) {
		ast_vddio_1v8_cam2 = regulator_get(NULL, "vddio_1v8_cam2");
		if (WARN_ON(IS_ERR(ast_vddio_1v8_cam2))) {
			pr_err("%s: couldn't get regulator vddio_1v8_cam2: %ld\n",
			       __func__, PTR_ERR(ast_vddio_1v8_cam2));
			ast_vddio_1v8_cam2 = NULL;
			return -ENODEV;
		}
	}

	if (!ast_avdd_2v8_cam2) {
		ast_avdd_2v8_cam2 = regulator_get(NULL, "avdd_2v8_cam2");
		if (WARN_ON(IS_ERR(ast_avdd_2v8_cam2))) {
			pr_err("%s: couldn't get regulator avdd_2v8_cam2: %ld\n",
			       __func__, PTR_ERR(ast_avdd_2v8_cam2));
			ast_avdd_2v8_cam2 = NULL;
			return -ENODEV;
		}
	}

	err = gpio_request(CAM2_PWDN_GPIO, "cam2_power_down");
	if (err < 0)
		return err;

	gpio_direction_output(CAM2_PWDN_GPIO, 0);
	tegra_gpio_enable(CAM2_PWDN_GPIO);
	err = gpio_request(CAM2_RST_GPIO, "cam2_reset");
	if (err < 0)
		return err;

	gpio_direction_output(CAM2_RST_GPIO, 0);
	tegra_gpio_enable(CAM2_RST_GPIO);
	err = gpio_request(CAM2_LED_GPIO, "cam2_led");
	if (err < 0)
		return err;

	gpio_direction_output(CAM2_LED_GPIO, 0);
	tegra_gpio_enable(CAM2_LED_GPIO);

	return 0;
}

static void ast_mt9d115_led(int on)
{
	ast_mt9d115_led_state = on;
	gpio_set_value(CAM2_LED_GPIO, on);
}

int ast_mt9d115_led_get_state (void)
{
    return ast_mt9d115_led_state;
}

static int ast_mt9d115_power_on(void)
{
	if (!ast_vdd_1v8_cam2 || !ast_vddio_1v8_cam2 || !ast_avdd_2v8_cam2)
		return -ENODEV;

	gpio_set_value(CAM2_RST_GPIO, 1);
	regulator_enable(ast_vddio_1v8_cam2);
	udelay(1200);
	regulator_enable(ast_vdd_1v8_cam2);
	udelay(200);
	ast_camera_mclk_on(24000000);
	udelay(1);
	gpio_set_value(CAM2_RST_GPIO, 0);
	udelay(1);
	gpio_set_value(CAM2_RST_GPIO, 1);
	udelay(1);
	regulator_enable(ast_avdd_2v8_cam2);
	udelay(300);

	return 0;
}

static int ast_mt9d115_power_off(void)
{
	if (!ast_vdd_1v8_cam2 || !ast_vddio_1v8_cam2 || !ast_avdd_2v8_cam2)
		return -ENODEV;

	gpio_set_value(CAM2_RST_GPIO, 0);
	ast_camera_mclk_off();
	regulator_disable(ast_avdd_2v8_cam2);
	regulator_disable(ast_vdd_1v8_cam2);
	regulator_disable(ast_vddio_1v8_cam2);

	return 0;
}

struct mt9d115_platform_data ast_mt9d115_data = {
	.init = ast_mt9d115_init,
	.led = ast_mt9d115_led,
	.power_on = ast_mt9d115_power_on,
	.power_off = ast_mt9d115_power_off,
};
#endif

#ifdef CONFIG_TORCH_TPS61050
static struct nvc_torch_pin_state ast_tps61050_pinstate = {
	.mask = 0x0008, /*VGP3*/
	.values = 0x0008,
};

static struct tps61050_platform_data ast_tps61050_pdata = {
    .dev_name = "torch",
	.pinstate = &ast_tps61050_pinstate,
};
#endif

static const struct i2c_board_info ast_i2c2_cam_board_info[] = {
#ifdef CONFIG_TORCH_TPS61050
	{
		I2C_BOARD_INFO("tps61050", 0x33),
		.platform_data = &ast_tps61050_pdata,
	},
#endif
#ifdef CONFIG_VIDEO_MT9P111
	{
		I2C_BOARD_INFO("mt9p111", 0x3D),
		.platform_data = &ast_mt9p111_data,
	},
#endif
#ifdef CONFIG_VIDEO_MT9D115
	{
		I2C_BOARD_INFO("mt9d115", 0x3C),
		.platform_data = &ast_mt9d115_data,
	},
#endif
};

static int nct_get_temp(void *_data, long *temp)
{
	struct nct1008_data *data = _data;
	return nct1008_thermal_get_temp(data, temp);
}

static int nct_get_temp_low(void *_data, long *temp)
{
	struct nct1008_data *data = _data;
	return nct1008_thermal_get_temp_low(data, temp);
}

static int nct_set_limits(void *_data,
			long lo_limit_milli,
			long hi_limit_milli)
{
	struct nct1008_data *data = _data;
	return nct1008_thermal_set_limits(data,
					lo_limit_milli,
					hi_limit_milli);
}

static int nct_set_alert(void *_data,
				void (*alert_func)(void *),
				void *alert_data)
{
	struct nct1008_data *data = _data;
	return nct1008_thermal_set_alert(data, alert_func, alert_data);
}

static int nct_set_shutdown_temp(void *_data, long shutdown_temp)
{
	struct nct1008_data *data = _data;
	return nct1008_thermal_set_shutdown_temp(data, shutdown_temp);
}

#ifdef CONFIG_TEGRA_SKIN_THROTTLE
static int nct_get_itemp(void *dev_data, long *temp)
{
	struct nct1008_data *data = dev_data;
	return nct1008_thermal_get_temps(data, NULL, temp);
}
#endif

static void nct1008_probe_callback(struct nct1008_data *data)
{
	struct tegra_thermal_device *ext_nct;

	ext_nct = kzalloc(sizeof(struct tegra_thermal_device),
					GFP_KERNEL);
	if (!ext_nct) {
		pr_err("unable to allocate thermal device\n");
		return;
	}

	ext_nct->name = "nct_ext";
	ext_nct->id = THERMAL_DEVICE_ID_NCT_EXT;
	ext_nct->data = data;
	ext_nct->offset = TDIODE_OFFSET;
	ext_nct->get_temp = nct_get_temp;
	ext_nct->get_temp_low = nct_get_temp_low;
	ext_nct->set_limits = nct_set_limits;
	ext_nct->set_alert = nct_set_alert;
	ext_nct->set_shutdown_temp = nct_set_shutdown_temp;

	tegra_thermal_device_register(ext_nct);

#ifdef CONFIG_TEGRA_SKIN_THROTTLE
	{
		struct tegra_thermal_device *int_nct;
		int_nct = kzalloc(sizeof(struct tegra_thermal_device),
						GFP_KERNEL);
		if (!int_nct) {
			kfree(int_nct);
			pr_err("unable to allocate thermal device\n");
			return;
		}

		int_nct->name = "nct_int";
		int_nct->id = THERMAL_DEVICE_ID_NCT_INT;
		int_nct->data = data;
		int_nct->get_temp = nct_get_itemp;

		tegra_thermal_device_register(int_nct);
	}
#endif
}
#ifdef CONFIG_SENSORS_NCT1008
static struct nct1008_platform_data ast_nct1008_pdata = {
	.supported_hwrev = true,
	.ext_range = true,
	.conv_rate = 0x08,
	.offset = 8, /* 4 * 2C. Bug 844025 - 1C for device accuracies */
    .probe_callback = nct1008_probe_callback,
};

static struct i2c_board_info ast_i2c4_nct1008_board_info[] = {
	{
		I2C_BOARD_INFO("nct1008", 0x4C),
		.platform_data = &ast_nct1008_pdata,
		.irq = -1,
	}
};

static int __init ast_nct1008_init(void)
{
	int ret = 0;

	/* FIXME: enable irq when throttling is supported */
	ast_i2c4_nct1008_board_info[0].irq = TEGRA_GPIO_TO_IRQ(TEMP_ALERT_GPIO);

	ret = gpio_request(TEMP_ALERT_GPIO, "temp_alert");
	if (ret < 0)
		return ret;

	ret = gpio_direction_input(TEMP_ALERT_GPIO);
	if (ret < 0)
		gpio_free(TEMP_ALERT_GPIO);
	else
		tegra_gpio_enable(TEMP_ALERT_GPIO);

	i2c_register_board_info(4, ast_i2c4_nct1008_board_info,
				ARRAY_SIZE(ast_i2c4_nct1008_board_info));

	return ret;
}
#else
static int __init ast_nct1008_init(void) { return 0; }
#endif

#ifdef CONFIG_MPU_SENSORS_MPU3050
static struct mpu_platform_data mpu3050_data = {
	.int_config  = 0x10,
	.orientation = AST_GYRO_ORIENTATION,
	.level_shifter = 0,
};

static struct ext_slave_platform_data mpu3050_accel_data = {
	.adapt_num = 2,
	.bus = EXT_SLAVE_BUS_SECONDARY,
	.address = AST_ACCEL_ADDR,
	.orientation = AST_ACCEL_ORIENTATION,
};

static struct ext_slave_platform_data mpu_compass_data = {
	.adapt_num = 2,
	.bus = EXT_SLAVE_BUS_PRIMARY,
	.address = AST_COMPASS_ADDR,
	.orientation = AST_COMPASS_ORIENTATION,
};

static struct i2c_board_info __initdata inv_mpu_i2c2_boardinfo[] = {
	{
		I2C_BOARD_INFO(AST_GYRO_NAME, AST_GYRO_ADDR),
		.irq = TEGRA_GPIO_TO_IRQ(AST_GYRO_INT),
		.platform_data = &mpu3050_data,
	},
	{
		I2C_BOARD_INFO(AST_ACCEL_NAME, AST_ACCEL_ADDR),
		.irq = TEGRA_GPIO_TO_IRQ(AST_ACC_INT),
		.platform_data = &mpu3050_accel_data,
	},
	{
		I2C_BOARD_INFO(AST_COMPASS_NAME, AST_COMPASS_ADDR),
		.irq = TEGRA_GPIO_TO_IRQ(AST_COMPASS_INT),
		.platform_data = &mpu_compass_data,
	},
};

static void __init ast_mpuirq_init(void)
{
	pr_info("*** MPU START *** ast_mpuirq_init...\n");
	tegra_gpio_enable(AST_GYRO_INT);
	gpio_request(AST_GYRO_INT, AST_GYRO_NAME);
	gpio_direction_input(AST_GYRO_INT);
#ifdef CONFIG_MPU_SENSORS_KXTF9
	tegra_gpio_enable(AST_ACC_INT);
	gpio_request(AST_ACC_INT, "KXTF9_INT");
	gpio_direction_input(AST_ACC_INT);
#endif
#ifdef CONFIG_MPU_SENSORS_AK8975
	tegra_gpio_enable(AST_COMPASS_INT);
	gpio_request(AST_COMPASS_INT, "AK8975_DRDY");
	gpio_direction_input(AST_COMPASS_INT);
#endif
	pr_info("*** MPU END *** ast_mpuirq_init...\n");

	i2c_register_board_info(2, inv_mpu_i2c2_boardinfo,
		ARRAY_SIZE(inv_mpu_i2c2_boardinfo));
}
#else
static void __init ast_mpuirq_init(void) { }
#endif

static struct i2c_board_info ast_i2c2_isl_board_info[] = {
	{
		I2C_BOARD_INFO("isl29023", 0x44),
	}
};

int __init ast_sensors_init(void)
{
	i2c_register_board_info(2, ast_i2c2_isl_board_info,
				ARRAY_SIZE(ast_i2c2_isl_board_info));
	if (ARRAY_SIZE(ast_i2c2_cam_board_info))
		i2c_register_board_info(2, ast_i2c2_cam_board_info,
					ARRAY_SIZE(ast_i2c2_cam_board_info));

	ast_nct1008_init();
	ast_mpuirq_init();

	return 0;
}
