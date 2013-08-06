#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/platform_device.h>
#include <linux/resource.h>

#include <linux/io.h>
#include <linux/mfd/nvtec.h>

#include <mach/iomap.h>
#include <mach/irqs.h>

#include <asm/mach-types.h>

#include "board-ast.h"

#include "gpio-names.h"
#include "board.h"

#define TS_PIN_NULL -1

#define get_ap_wake_gpio()          ((!machine_is_titan() && is_ws_board()) \
                                     ? AP_WAKE_GPIO_SR1 \
                                     : AP_WAKE_GPIO)
#define get_batt_low_gpio()         ((!machine_is_titan() && is_ws_board()) \
                                     ? BATT_LOW_GPIO_SR1 \
                                     : BATT_LOW_GPIO)
#define get_ec_reset_gpio()         ((!machine_is_sphinx()) \
                                     ? EC_RST_GPIO \
                                     : TS_PIN_NULL)
#define get_hdmi_sw_en_gpio()       ((machine_is_avalon()) \
                                     ? HDMI_SW_EN_GPIO \
                                     : TS_PIN_NULL)

static struct nvtec_battery_platform_data nvtec_battery_pdata = {
    .ac_in_pin = AP_ACOK_GPIO,
    .batt_low_pin = TS_PIN_NULL, /* determined by runtime */
};

static struct nvtec_subdev_info nvtec_devs[] = {
    {
        .id = 0,
        .name = "nvtec-battery",
        .platform_data = &nvtec_battery_pdata,
    },
    {
        .id = 0,
        .name = "nvtec-cir",
        .platform_data = NULL,
    },
    {
        .id = 0,
        .name = "nvtec-undock",
        .platform_data = NULL,
    },

};

static struct nvtec_platform_data nvtec_pdata = {
    .num_subdevs = ARRAY_SIZE(nvtec_devs),
    .subdevs = nvtec_devs,
    .request_pin = EC_REQUEST_GPIO,
    .pwr_state_pin = EC_PWR_STATE_GPIO,
    .ap_wake_pin = TS_PIN_NULL, /* determined by runtime */
    .ec_wake_pin = TS_PIN_NULL,
    .hdmi_sw_pin = TS_PIN_NULL, /* determined by runtime */
};

static struct i2c_board_info __initdata ast_ec[] = {
    {
        I2C_BOARD_INFO("nvtec", 0x1b),
        .irq           = -1,
        .platform_data = &nvtec_pdata,
    },
};

int __init ast_ec_init(void)
{
	int ac_in_gpio = nvtec_battery_pdata.ac_in_pin;
	int batt_low_gpio = get_batt_low_gpio();
	int ec_request_gpio = nvtec_pdata.request_pin;
	int pwr_state_gpio = nvtec_pdata.pwr_state_pin;
	int ap_wake_gpio = get_ap_wake_gpio() ;
	int ec_reset_gpio = get_ec_reset_gpio();
	int hdmi_sw_gpio = get_hdmi_sw_en_gpio();

	nvtec_battery_pdata.batt_low_pin = batt_low_gpio;
	nvtec_pdata.ap_wake_pin = ap_wake_gpio;
    nvtec_pdata.hdmi_sw_pin = hdmi_sw_gpio ;
	
    if (TS_PIN_NULL != ec_request_gpio) {
        tegra_gpio_enable(ec_request_gpio);
        gpio_request(ec_request_gpio, "ec_request");
        gpio_direction_output(ec_request_gpio, 1);
    }

    if (TS_PIN_NULL != pwr_state_gpio) {
        tegra_gpio_enable(pwr_state_gpio);
        gpio_request(pwr_state_gpio, "ec_pwr_state");
        gpio_direction_output(pwr_state_gpio, 1);
    }

    if (TS_PIN_NULL != ap_wake_gpio) {
        tegra_gpio_enable(ap_wake_gpio);
        gpio_request(ap_wake_gpio, "ap_wake");
        gpio_direction_input(ap_wake_gpio);
        ast_ec[0].irq = TEGRA_GPIO_TO_IRQ(ap_wake_gpio);
    }

    if (TS_PIN_NULL != ac_in_gpio) {
        tegra_gpio_enable(ac_in_gpio);
        gpio_request(ac_in_gpio, "ac_in");
        gpio_direction_input(ac_in_gpio);
    }
    
    if (TS_PIN_NULL != batt_low_gpio) {
        tegra_gpio_enable(batt_low_gpio);
        gpio_request(batt_low_gpio, "batt_low");
        gpio_direction_input(batt_low_gpio);
    }

    if (TS_PIN_NULL != ec_reset_gpio) {
        tegra_gpio_enable(ec_reset_gpio);
        gpio_request(ec_reset_gpio, "ec_rst");
        gpio_direction_output(ec_reset_gpio, 0);
    }

    if (TS_PIN_NULL != hdmi_sw_gpio) {
        tegra_gpio_enable(hdmi_sw_gpio);
        gpio_request(hdmi_sw_gpio, "ec_hdmi_sw");
        gpio_direction_output(hdmi_sw_gpio, 1);
    }

    i2c_register_board_info(0, ast_ec, ARRAY_SIZE(ast_ec));
    return 0;
}
