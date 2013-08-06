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
#ifdef CONFIG_SENSORS_NCT1008
#include <linux/nct1008.h>
#ifdef CONFIG_TEGRA_EDP_LIMITS
#include <mach/edp.h>
#include <mach/thermal.h>
#endif
#endif
#ifdef CONFIG_MPU_SENSORS_MPU3050
#include <linux/mpu.h>
#endif
#ifdef CONFIG_VIDEO_MT9P111
#include <media/mt9p111.h>
#endif
#ifdef CONFIG_VIDEO_MT9D115
#include <media/mt9d115.h>
#endif
#ifdef CONFIG_TORCH_TPS61050
#include <media/tps61050.h>
#endif

#include "gpio-names.h"
#include "board-ast.h"
#include "cpu-tegra.h"

#include <asm/mach-types.h>

#define AST_TEMP_ALERT       TEGRA_GPIO_PCC2

#define AST_CAM1_PWDN        TEGRA_GPIO_PBB5
#define AST_CAM2_PWDN        TEGRA_GPIO_PBB6
#define AST_CAM1_RST         TEGRA_GPIO_PBB3
#define AST_CAM2_RST         TEGRA_GPIO_PBB0
#define AST_CAM2_LED         TEGRA_GPIO_PU4

#define CAM1_MCLK_RATE  24000000
#define CAM2_MCLK_RATE  24000000

static struct regulator *ast_vddio_cam = NULL;
static struct regulator *ast_avdd_cam = NULL;
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
	if (!ast_vddio_cam) {
		ast_vddio_cam = regulator_get(NULL, "vdd_gen1v8");
		if (WARN_ON(IS_ERR(ast_vddio_cam))) {
			pr_err("%s: couldn't get regulator vdd_gen1v8: %ld\n",
			       __func__, PTR_ERR(ast_vddio_cam));
			ast_vddio_cam = NULL;
			return -ENODEV;
		}
	}

	regulator_enable(ast_vddio_cam);
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

	if (ast_vddio_cam)
		regulator_disable(ast_vddio_cam);

	return 0;
}

#ifdef CONFIG_VIDEO_MT9P111
static int ast_mt9p111_init(void)
{
	int err;

	if (!ast_avdd_cam) {
		ast_avdd_cam = regulator_get(NULL, "vdd_3v3_cam");
		if (WARN_ON(IS_ERR(ast_avdd_cam))) {
			pr_err("%s: couldn't get regulator vdd_3v3_cam: %ld\n",
			       __func__, PTR_ERR(ast_avdd_cam));
			ast_avdd_cam = NULL;
			return -ENODEV;
		}
	}

	if (machine_is_titan()) {
		if (!ast_vddio_cam) {
			ast_vddio_cam = regulator_get(NULL, "vdd_gen1v8");
			if (WARN_ON(IS_ERR(ast_vddio_cam))) {
				pr_err("%s: couldn't get regulator vdd_gen1v8: %ld\n",
				       __func__, PTR_ERR(ast_vddio_cam));
				ast_vddio_cam = NULL;
				return -ENODEV;
			}
		}

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

	err = gpio_request(AST_CAM1_PWDN, "cam1_power_down");
	if (err < 0)
		return err;

	gpio_direction_output(AST_CAM1_PWDN, 0);
	tegra_gpio_enable(AST_CAM1_PWDN);
	err = gpio_request(AST_CAM1_RST, "cam1_reset");
	if (err < 0)
		return err;

	gpio_direction_output(AST_CAM1_RST, 0);
	tegra_gpio_enable(AST_CAM1_RST);

	return 0;
}

static int ast_mt9p111_power_on(void)
{
	if (machine_is_titan()) {
		if (!ast_vddio_cam || !ast_avdd_cam || !ast_vddio_1v8_cam1 ||
			!ast_avdd_2v8_cam1 || !ast_vdd_3v3_cam1)
			return -ENODEV;
	} else {
		if (!ast_avdd_cam || !ast_vddio_1v8_cam1 ||
			!ast_avdd_2v8_cam1 || !ast_vdd_2v8_cam1)
			return -ENODEV;
	}

	if (machine_is_titan()) {
		gpio_set_value(AST_CAM1_PWDN, 0);
		regulator_enable(ast_vddio_cam);
		regulator_enable(ast_vddio_1v8_cam1);
		mdelay(1);
		regulator_enable(ast_avdd_cam);
		regulator_enable(ast_vdd_3v3_cam1);
		regulator_enable(ast_avdd_2v8_cam1);
		ast_camera_mclk_on(CAM1_MCLK_RATE);
		mdelay(1);
		gpio_set_value(AST_CAM1_RST, 1);
	} else {
		gpio_set_value(AST_CAM1_PWDN, 0);
		regulator_enable(ast_avdd_cam);
		regulator_enable(ast_vddio_1v8_cam1);
		mdelay(1);
		regulator_enable(ast_avdd_2v8_cam1);
		regulator_enable(ast_vdd_2v8_cam1);
		ast_camera_mclk_on(CAM1_MCLK_RATE);
		mdelay(1);
		gpio_set_value(AST_CAM1_RST, 1);
	}

	return 0;
}

static int ast_mt9p111_power_off(void)
{
	if (machine_is_titan()) {
		if (!ast_vddio_cam || !ast_avdd_cam || !ast_vddio_1v8_cam1 ||
			!ast_avdd_2v8_cam1 || !ast_vdd_3v3_cam1)
			return -ENODEV;
	} else {
		if (!ast_avdd_cam || !ast_vddio_1v8_cam1 ||
			!ast_avdd_2v8_cam1 || !ast_vdd_2v8_cam1)
			return -ENODEV;
	}

	if (machine_is_titan()) {
		gpio_set_value(AST_CAM1_RST, 0);
		mdelay(1);
		gpio_set_value(AST_CAM1_PWDN, 1);
		mdelay(1);
		regulator_disable(ast_avdd_2v8_cam1);
		regulator_disable(ast_vdd_3v3_cam1);
		mdelay(1);
		regulator_disable(ast_vddio_1v8_cam1);
		regulator_disable(ast_avdd_cam);
		regulator_disable(ast_vddio_cam);
		ast_camera_mclk_off();
		gpio_set_value(AST_CAM1_PWDN, 0);
	} else {
		gpio_set_value(AST_CAM1_RST, 0);
		mdelay(1);
		gpio_set_value(AST_CAM1_PWDN, 1);
		mdelay(1);
		regulator_disable(ast_vdd_2v8_cam1);
		regulator_disable(ast_avdd_2v8_cam1);
		mdelay(1);
		regulator_disable(ast_vddio_1v8_cam1);
		regulator_disable(ast_avdd_cam);
		ast_camera_mclk_off();
		gpio_set_value(AST_CAM1_PWDN, 0);
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

	if (!ast_vddio_cam) {
		ast_vddio_cam = regulator_get(NULL, "vdd_gen1v8");
		if (WARN_ON(IS_ERR(ast_vddio_cam))) {
			pr_err("%s: couldn't get regulator vdd_gen1v8: %ld\n",
			       __func__, PTR_ERR(ast_vddio_cam));
			ast_vddio_cam = NULL;
			return -ENODEV;
		}
	}

	if (!ast_avdd_cam) {
		ast_avdd_cam = regulator_get(NULL, "vdd_3v3_cam");
		if (WARN_ON(IS_ERR(ast_avdd_cam))) {
			pr_err("%s: couldn't get regulator vdd_3v3_cam: %ld\n",
			       __func__, PTR_ERR(ast_avdd_cam));
			ast_avdd_cam = NULL;
			return -ENODEV;
		}
	}

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

	err = gpio_request(AST_CAM2_PWDN, "cam2_power_down");
	if (err < 0)
		return err;

	gpio_direction_output(AST_CAM2_PWDN, 0);
	tegra_gpio_enable(AST_CAM2_PWDN);
	err = gpio_request(AST_CAM2_RST, "cam2_reset");
	if (err < 0)
		return err;

	gpio_direction_output(AST_CAM2_RST, 0);
	tegra_gpio_enable(AST_CAM2_RST);
	err = gpio_request(AST_CAM2_LED, "cam2_led");
	if (err < 0)
		return err;

	gpio_direction_output(AST_CAM2_LED, 0);
	tegra_gpio_enable(AST_CAM2_LED);

	return 0;
}

static void ast_mt9d115_led(int on)
{
        ast_mt9d115_led_state = on;
	gpio_set_value(AST_CAM2_LED, on);
}

int ast_mt9d115_led_get_state ( void )
{
        return ( ast_mt9d115_led_state );
}

static int ast_mt9d115_power_on(void)
{
	if (!ast_vddio_cam || !ast_avdd_cam || !ast_vdd_1v8_cam2 ||
		!ast_vddio_1v8_cam2 || !ast_avdd_2v8_cam2)
		return -ENODEV;

	gpio_set_value(AST_CAM2_PWDN, 0);
	gpio_set_value(AST_CAM2_RST, 1);
	regulator_enable(ast_vddio_cam);
	regulator_enable(ast_vddio_1v8_cam2);
	udelay(1200);
	regulator_enable(ast_vdd_1v8_cam2);
	mdelay(1);
	ast_camera_mclk_on(CAM2_MCLK_RATE);
	mdelay(1);
	gpio_set_value(AST_CAM2_RST, 0);
	mdelay(1);
	gpio_set_value(AST_CAM2_RST, 1);
	mdelay(1);
	regulator_enable(ast_avdd_cam);
	regulator_enable(ast_avdd_2v8_cam2);

	return 0;
}

static int ast_mt9d115_power_off(void)
{
	if (!ast_vddio_cam || !ast_avdd_cam || !ast_vdd_1v8_cam2 ||
		!ast_vddio_1v8_cam2 || !ast_avdd_2v8_cam2)
		return -ENODEV;

	gpio_set_value(AST_CAM2_RST, 0);
	mdelay(1);
	gpio_set_value(AST_CAM2_PWDN, 1);
	mdelay(1);
	regulator_disable(ast_avdd_2v8_cam2);
	regulator_disable(ast_vdd_1v8_cam2);
	regulator_disable(ast_vddio_1v8_cam2);
	regulator_disable(ast_avdd_cam);
	regulator_disable(ast_vddio_cam);
	ast_camera_mclk_off();
	gpio_set_value(AST_CAM2_PWDN, 0);

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

#ifndef CONFIG_TEGRA_INTERNAL_TSENSOR_EDP_SUPPORT
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
static void nct1008_probe_callback(struct nct1008_data *data)
{
       struct tegra_thermal_device *thermal_device;

       thermal_device = kzalloc(sizeof(struct tegra_thermal_device),
                                       GFP_KERNEL);
       if (!thermal_device) {
               pr_err("unable to allocate thermal device\n");
               return;
       }

       thermal_device->name = "nct1008";
       thermal_device->data = data;
       thermal_device->offset = TDIODE_OFFSET;
       thermal_device->get_temp = nct_get_temp;
       thermal_device->get_temp_low = nct_get_temp_low;
       thermal_device->set_limits = nct_set_limits;
       thermal_device->set_alert = nct_set_alert;
       thermal_device->set_shutdown_temp = nct_set_shutdown_temp;

       tegra_thermal_set_device(thermal_device);
}
#endif

#ifdef CONFIG_SENSORS_NCT1008
static struct nct1008_platform_data ast_nct1008_pdata = {
	.supported_hwrev = true,
	.ext_range = true,
	.conv_rate = 0x08,
	.offset = 8, /* 4 * 2C. Bug 844025 - 1C for device accuracies */
#ifndef CONFIG_TEGRA_INTERNAL_TSENSOR_EDP_SUPPORT
    .probe_callback = nct1008_probe_callback,
#endif
};

static struct i2c_board_info ast_i2c4_nct1008_board_info[] = {
	{
		I2C_BOARD_INFO("nct1008", 0x4C),
		.platform_data = &ast_nct1008_pdata,
		.irq = -1,
	}
};

static int ast_nct1008_init(void)
{
	int ret = 0;

	/* FIXME: enable irq when throttling is supported */
	ast_i2c4_nct1008_board_info[0].irq = TEGRA_GPIO_TO_IRQ(AST_TEMP_ALERT);

	ret = gpio_request(AST_TEMP_ALERT, "temp_alert");
	if (ret < 0)
		return ret;

	ret = gpio_direction_input(AST_TEMP_ALERT);
	if (ret < 0)
		gpio_free(AST_TEMP_ALERT);
	else
		tegra_gpio_enable(AST_TEMP_ALERT);

	i2c_register_board_info(4, ast_i2c4_nct1008_board_info,
				ARRAY_SIZE(ast_i2c4_nct1008_board_info));

	return ret;
}
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

static void ast_mpuirq_init(void)
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

#ifdef CONFIG_SENSORS_NCT1008
	ast_nct1008_init();
#endif

#ifdef CONFIG_MPU_SENSORS_MPU3050
	ast_mpuirq_init();
#endif

	return 0;
}
