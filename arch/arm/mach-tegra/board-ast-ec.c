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

extern unsigned int system_rev;

#define T3_SR_EC_REQUEST_GPIO    TEGRA_GPIO_PI3
#define T3_SR_AP_WAKE_GPIO       TEGRA_GPIO_PK2
#define T3_SR_AC_IN_GPIO         TEGRA_GPIO_PV1  /* NOT CONNECT */
#define T3_SR_BATT_LOW_GPIO      TEGRA_GPIO_PX2


#define T3_ER_EC_REQUEST_GPIO    TEGRA_GPIO_PCC1
#define T3_ER_AP_WAKE_GPIO       TEGRA_GPIO_PO5
#define T3_ER_AC_IN_GPIO         TEGRA_GPIO_PV1  /* NOT CONNECT */
#define T3_ER_BATT_LOW_GPIO      TEGRA_GPIO_PX2

#define TS_PINS_NULL 0

/* Daniel Wang */
#define T3_MICP_EC_REQUEST_GPIO		TEGRA_GPIO_PB0		/* EC_REQUEST Output pin */
#define T3_MICP_EC_PWR_STATE_GPIO	TEGRA_GPIO_PDD7 	/* EC_PWR_STATE Output pin */
#define T3_MICP_AP_WAKE_GPIO_SR1	TEGRA_GPIO_PCC6  	/* AP_WAKE Input pin */
#define T3_MICP_AP_WAKE_GPIO		TEGRA_GPIO_PC7  	/* AP_WAKE Input pin */
#define T3_MICP_AP_ACOK_GPIO		TEGRA_GPIO_PV1		/* AP_ACOK Input pin */
#define T3_MICP_BATT_LOW_GPIO_SR1	TEGRA_GPIO_PB1		/* LOW_BAT Input pin */
#define T3_MICP_BATT_LOW_GPIO		TEGRA_GPIO_PI6		/* LOW_BAT Input pin */
#define T3_MICP_EC_WAKE_GPIO		TEGRA_GPIO_PJ7		/* EC_WAKE Output pin */
#define T3_HDMI_SW_EN_GPIO		TEGRA_GPIO_PH5		/* HDMI_SW_EN Ouput pin */	
#if !defined(CONFIG_MACH_SPHINX)
#define T3_MICP_RST			TEGRA_GPIO_PD2		/* T30_MCU_RST */
#endif

static struct nvtec_battery_platform_data nvtec_battery_pdata = {
    .ac_in_pin = TS_PINS_NULL,
    .batt_low_pin = TS_PINS_NULL,
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
    .request_pin = TS_PINS_NULL,
    .pwr_state_pin = TS_PINS_NULL,
    .ap_wake_pin = TS_PINS_NULL,
    .ec_wake_pin = TS_PINS_NULL,
    .hdmi_sw_pin = TS_PINS_NULL,
};

static struct i2c_board_info __initdata ast_ec[] = {
    {
        I2C_BOARD_INFO("nvtec", 0x1b),
        .irq         = TEGRA_GPIO_TO_IRQ(TS_PINS_NULL),
        .platform_data = &nvtec_pdata,
    },
};

int __init ast_ec_init(void)
{
	int ec_request_gpio = 0 ;
	int ec_ap_wake_gpio = 0 ;
	int ec_pwr_state_gpio = 0 ;
	int ec_ac_in_gpio = 0 ;
	int ec_bat_low_gpio = 0 ;
	int ec_reset = 0 ;
	int ec_hdmi_sw = 0 ;

	ec_request_gpio = T3_MICP_EC_REQUEST_GPIO ;
	ec_pwr_state_gpio = T3_MICP_EC_PWR_STATE_GPIO ;
	ec_ac_in_gpio = T3_MICP_AP_ACOK_GPIO ;

	if (machine_is_avalon())
	    ec_hdmi_sw = T3_HDMI_SW_EN_GPIO; 

	printk("nvtec: Board Revision is [%d] ",system_rev);
#if !defined(CONFIG_MACH_TITAN)
	if (system_rev == 0){
	    printk(" [1.0] \n");
            ec_ap_wake_gpio = T3_MICP_AP_WAKE_GPIO_SR1 ;
            ec_bat_low_gpio = T3_MICP_BATT_LOW_GPIO_SR1 ;
	}else{
	    printk(" [2.0] \n");
	    ec_ap_wake_gpio = T3_MICP_AP_WAKE_GPIO ;
	    ec_bat_low_gpio = T3_MICP_BATT_LOW_GPIO ;
	}
#else
	ec_ap_wake_gpio = T3_MICP_AP_WAKE_GPIO ;
	ec_bat_low_gpio = T3_MICP_BATT_LOW_GPIO ;
#endif

#if !defined(CONFIG_MACH_SPHINX)
	ec_reset = T3_MICP_RST;
#endif

	nvtec_battery_pdata.ac_in_pin = ec_ac_in_gpio ;
	nvtec_battery_pdata.batt_low_pin = ec_bat_low_gpio ;

	nvtec_pdata.ap_wake_pin = ec_ap_wake_gpio ;
	nvtec_pdata.request_pin = ec_request_gpio ;
	nvtec_pdata.pwr_state_pin = ec_pwr_state_gpio ;
	if (machine_is_avalon())
	    nvtec_pdata.hdmi_sw_pin = ec_hdmi_sw ;

	/* ast_ec[0].irq = TEGRA_GPIO_TO_IRQ(ec_ap_wake_gpio) ; */
	
    	tegra_gpio_enable(ec_request_gpio);
	gpio_request(ec_request_gpio, "ec_request");
    	gpio_direction_output(ec_request_gpio, 1);

	/* Daniel Wang */    
    	tegra_gpio_enable(ec_pwr_state_gpio);
    	gpio_request(ec_pwr_state_gpio, "ec_pwr_state");
    	gpio_direction_output(ec_pwr_state_gpio, 1);

	/*tegra_gpio_enable(ec_wake_gpio);*/
	/*gpio_request(ec_wake_gpio, "ec_wake");*/
	/*gpio_direction_input(ec_wake_gpio);*/

    	tegra_gpio_enable(ec_ap_wake_gpio);
    	gpio_request(ec_ap_wake_gpio, "ap_wake");
    	gpio_direction_input(ec_ap_wake_gpio);

	ast_ec[0].irq = TEGRA_GPIO_TO_IRQ(ec_ap_wake_gpio) ;

    	tegra_gpio_enable(ec_ac_in_gpio);
    	gpio_request(ec_ac_in_gpio, "ac_in");
    	gpio_direction_input(ec_ac_in_gpio);
    
    	tegra_gpio_enable(ec_bat_low_gpio);
    	gpio_request(ec_bat_low_gpio, "batt_low");
    	gpio_direction_input(ec_bat_low_gpio);

#if !defined(CONFIG_MACH_SPHINX)
    	tegra_gpio_enable(ec_reset);
    	gpio_request(ec_reset, "ec_rst");
    	gpio_direction_output(ec_reset, 0);
#endif

	if (machine_is_avalon()){
	   tegra_gpio_enable(ec_hdmi_sw);
	   gpio_request(ec_hdmi_sw, "ec_hdmi_sw");
	   gpio_direction_output(ec_hdmi_sw, 1);
	}

    	i2c_register_board_info(0, ast_ec, ARRAY_SIZE(ast_ec));
    	return 0;
}
