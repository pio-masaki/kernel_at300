/*
 * arch/arm/mach-tegra/board-ast.h
 *
 * Copyright (c) 2011, NVIDIA Corporation.
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

#ifndef _MACH_TEGRA_BOARD_AST_H
#define _MACH_TEGRA_BOARD_AST_H

#include <mach/gpio.h>
#include <mach/irqs.h>
#include <linux/mfd/tps6591x.h>
#include <linux/mfd/ricoh583.h>

/* External peripheral act as gpio */
/* TPS6591x GPIOs */
#define TPS6591X_GPIO_BASE	TEGRA_NR_GPIOS
#define TPS6591X_GPIO_0		(TPS6591X_GPIO_BASE + TPS6591X_GPIO_GP0)
#define TPS6591X_GPIO_1		(TPS6591X_GPIO_BASE + TPS6591X_GPIO_GP1)
#define TPS6591X_GPIO_2		(TPS6591X_GPIO_BASE + TPS6591X_GPIO_GP2)
#define TPS6591X_GPIO_3		(TPS6591X_GPIO_BASE + TPS6591X_GPIO_GP3)
#define TPS6591X_GPIO_4		(TPS6591X_GPIO_BASE + TPS6591X_GPIO_GP4)
#define TPS6591X_GPIO_5		(TPS6591X_GPIO_BASE + TPS6591X_GPIO_GP5)
#define TPS6591X_GPIO_6		(TPS6591X_GPIO_BASE + TPS6591X_GPIO_GP6)
#define TPS6591X_GPIO_7		(TPS6591X_GPIO_BASE + TPS6591X_GPIO_GP7)
#define TPS6591X_GPIO_8		(TPS6591X_GPIO_BASE + TPS6591X_GPIO_GP8)
#define TPS6591X_GPIO_END	(TPS6591X_GPIO_BASE + TPS6591X_GPIO_NR)

/* WM8903 GPIOs */
#define AST_GPIO_WM8903(_x_)		(TPS6591X_GPIO_END + (_x_))
#define AST_GPIO_WM8903_END		    AST_GPIO_WM8903(4)

/*****************Interrupt tables ******************/
/* External peripheral act as interrupt controller */
/* TPS6591x IRQs */
#define TPS6591X_IRQ_BASE	TEGRA_NR_IRQS
#define TPS6591X_IRQ_END	(TPS6591X_IRQ_BASE + 18)

/********************************************
*              |  A4 |  A3 |  A2 |  A1 | A0
* -------------+-----+-----+-----+-----+-----
*  Avalon      |  -  |  -  |  -  |  0  |  0
*  Sphinx      |  -  |  -  |  -  |  0  |  1
*  Titan       |  -  |  -  |  -  |  1  |  0
* -------------+-----+-----+-----+-----+-----
*   WS         |  0  |  0  |  0  |  -  |  -
*   ES1        |  1  |  0  |  0  |  -  |  -
*   ES2        |  0  |  1  |  0  |  -  |  -
*   CS         |  1  |  1  |  0  |  -  |  -
*   MP         |  0  |  0  |  1  |  -  |  -
*********************************************/

enum hw_version {
	BOARD_VERSION_UNKNOWN = -1,
	BOARD_VERSION_WS = 0,
	BOARD_VERSION_ES1 = 1,
	BOARD_VERSION_ES2 = 2,
	BOARD_VERSION_CS = 3,
    BOARD_VERSION_MP = 4,
};

enum power_type{
    WIFI_POWER=0,
    BT_POWER=1
};

extern unsigned int system_rev;
#define is_ws_board()    (system_rev == BOARD_VERSION_WS)
#define is_es1_board()   (system_rev == BOARD_VERSION_ES1)
#define is_es2_board()   (system_rev == BOARD_VERSION_ES2)
#define is_cs_board()    (system_rev == BOARD_VERSION_CS)
#define is_mp_board()    (system_rev == BOARD_VERSION_MP)

int ast_regulator_init(void);
int ast_suspend_init(void);
int ast_sdhci_init(void);
int ast_pinmux_init(void);
int ast_panel_init(void);
int sphinx_panel_init(void);
int ast_sensors_init(void);
int ast_kbc_init(void);
int ast_keys_init(void);
int ast_ec_init(void);
int ast_pins_state_init(void);
int ast_emc_init(void);
int ast_edp_init(void);
/* void ast_get_hw_info(struct board_version *); */

#define TDIODE_OFFSET               (10000) /* in millicelsius */

/* Add common definition */
/* Invensense MPU Definitions */
#define MPU_TYPE_MPU3050	        1
#define MPU_TYPE_MPU6050	        2
#define MPU_GYRO_TYPE		        MPU_TYPE_MPU3050
#define AST_GYRO_NAME        	    "mpu3050"
#define AST_GYRO_INT         	    TEGRA_GPIO_PX1
#define AST_GYRO_ADDR        	    0x68
#define AST_ACCEL_NAME       	    "kxtf9"
#define AST_ACC_INT          	    TEGRA_GPIO_PO5
#define AST_ACCEL_ADDR       	    0x0F
#define AST_COMPASS_NAME     	    "ak8975"
#define AST_COMPASS_INT      	    TEGRA_GPIO_PW0
#define AST_COMPASS_ADDR 	        0x0C

/* Audio-related GPIOs */
#define TEGRA_GPIO_CDC_IRQ		    TEGRA_GPIO_PW3
#define TEGRA_GPIO_SPKR_EN		    AST_GPIO_WM8903(2)
#define TEGRA_GPIO_HP_DET		    TEGRA_GPIO_PW2
#define TEGRA_GPIO_AMIC_DET		    TEGRA_GPIO_PS6
#define TEGRA_GPIO_LINEOUT_DET	    TEGRA_GPIO_PS3
#define VD_PWD_GPIO                 TEGRA_GPIO_PN1
#define VD_RST_GPIO                 TEGRA_GPIO_PN0
#define VD_BP_GPIO                  TEGRA_GPIO_PN3
#define DMIC_1V8_EN_GPIO            TEGRA_GPIO_PX0
#define AMIC_5V0_EN_GPIO            TEGRA_GPIO_PO1
#define AUDIO_1V8_EN_GPIO           TEGRA_GPIO_PD4 /* sphinx only */

/* SD/eMMC GPIO definitions */
#define SD_CD_GPIO                  TEGRA_GPIO_PI5
#define EMMC_3V3_EN_GPIO            TEGRA_GPIO_PD1
#define SD_VDD_EN_GPIO              TEGRA_GPIO_PD4

/* MCU GPIO definitions */
#define EC_REQUEST_GPIO		        TEGRA_GPIO_PB0		/* EC_REQUEST Output pin */
#define EC_PWR_STATE_GPIO	        TEGRA_GPIO_PDD7 	/* EC_PWR_STATE Output pin */
#define AP_WAKE_GPIO_SR1	        TEGRA_GPIO_PCC6  	/* AP_WAKE Input pin for A/S SR1*/
#define AP_WAKE_GPIO		        TEGRA_GPIO_PC7  	/* AP_WAKE Input pin */
#define AP_ACOK_GPIO		        TEGRA_GPIO_PV1		/* AP_ACOK Input pin */
#define BATT_LOW_GPIO_SR1	        TEGRA_GPIO_PB1		/* LOW_BAT Input pin for A/S SR1*/
#define BATT_LOW_GPIO		        TEGRA_GPIO_PI6		/* LOW_BAT Input pin */
#define EC_WAKE_GPIO		        TEGRA_GPIO_PJ7		/* EC_WAKE Output pin */
#define EC_RST_GPIO			        TEGRA_GPIO_PD2		/* T30_MCU_RST for A/T*/

/* BT GPIO definitions */
#define BT_SHUTDOWN_GPIO            TEGRA_GPIO_PB2
#define BT_RESET_GPIO               TEGRA_GPIO_PU0
#define BT_IRQ_GPIO                 TEGRA_GPIO_PU6
#define BT_WAKEUP_GPIO              TEGRA_GPIO_PU1

/* WiFi GPIO definitions */
#define WLAN_RST_GPIO	            TEGRA_GPIO_PD3
#define WLAN_WOW_GPIO	            TEGRA_GPIO_PO4
#define WLAN_3V3_EN_GPIO            TEGRA_GPIO_PV3
#define WLAN_1V8_EN_GPIO            TEGRA_GPIO_PCC5

/* Dock GPIO definitions, avalon/sphinx */
#define DOCK_DET_GPIO               TEGRA_GPIO_PS4
#define DOCK_AMP_EN_GPIO            TEGRA_GPIO_PB1

/* Board ID GPIO definitions */
#define BOARD_ID_A0_GPIO            TEGRA_GPIO_PR0
#define BOARD_ID_A1_GPIO            TEGRA_GPIO_PR1
#define BOARD_ID_A2_GPIO            TEGRA_GPIO_PR2
#define BOARD_ID_A3_GPIO            TEGRA_GPIO_PR3
#define BOARD_ID_A4_GPIO            TEGRA_GPIO_PR4

/* ISDB-T GPIO definitions: titan only */
#define ISDBT_RESET_MP_GPIO         TEGRA_GPIO_PK2
#define ISDBT_RESET_CS_GPIO         TEGRA_GPIO_PI3
#define ISDBT_1V8_EN_GPIO           TEGRA_GPIO_PP0
#define ISDBT_3V3_EN_GPIO           TEGRA_GPIO_PO2

/* Vibrator GPIO definition */
#define VIBRATOR_EN_GPIO            TEGRA_GPIO_PH7
#define AVALON_VIB_3V3_EN_GPIO      TEGRA_GPIO_PX6 /* avalon only */
#define SPHINX_VIB_3V3_EN_GPIO      TEGRA_GPIO_PK6 /* sphinx only */

/* GPS GPIO definitions */
#define GPS_EN_GPIO                 TEGRA_GPIO_PU2
#define GPS_RST_GPIO                TEGRA_GPIO_PU3
#define GPS_3V3_EN_GPIO             TEGRA_GPIO_PV2
#define GPS_1V8_EN_GPIO             TEGRA_GPIO_PW5

/* Modem GPIO definitions, avalon/sphinx */
#define R_EN_3V3_MODEM              TEGRA_GPIO_PP0
#define R_ON1                       TEGRA_GPIO_PO2
#define W_DISABLE_N                 TEGRA_GPIO_PV6
#define WW_WAKE                     TEGRA_GPIO_PN2
#define R_HW_READY_3G               TEGRA_GPIO_PQ6
#define R_SW_READY_3G               TEGRA_GPIO_PQ5
#define R_TX_ON_3G                  TEGRA_GPIO_PQ4

/* button/switch GPIO definition */
#define MULTIFUNC_SWITCH_GPIO       TEGRA_GPIO_PR5
#define VOL_UP_BUTTON_GPIO          TEGRA_GPIO_PQ0
#define VOL_DOWN_BUTTON_GPIO        TEGRA_GPIO_PQ1
#define AP_ONKEY_GPIO               TEGRA_GPIO_PV0

/* LCD panel GPIO definitions: avalon/titan */
#define LVDS_SHUTDOWN_GPIO          TEGRA_GPIO_PN6
#define BL_ENB_GPIO                 TEGRA_GPIO_PH2
#define BL_PWM_GPIO                 TEGRA_GPIO_PH0
#define EN_VDD_BL1_GPIO             TEGRA_GPIO_PK7
#define EN_VDD_PNL1_GPIO            TEGRA_GPIO_PW1

/* LCD panel GPIO definitions: sphinx only */
#define MIPI_1V8_EN_GPIO            TEGRA_GPIO_PX2
#define EN_1V8_PNL_GPIO             TEGRA_GPIO_PC6
#define EN_5V8_BL_GPIO              TEGRA_GPIO_PH1

/* HDMI GPIO definitions */
#define HDMI_HPD_GPIO	            TEGRA_GPIO_PN7
#define HDMI_SW_EN_GPIO	   	        TEGRA_GPIO_PH5 /* avalon only*/

/* Camera GPIO definitions */
#define CAM1_PWDN_GPIO              TEGRA_GPIO_PBB5
#define CAM2_PWDN_GPIO              TEGRA_GPIO_PBB6
#define CAM1_RST_GPIO               TEGRA_GPIO_PBB3
#define CAM2_RST_GPIO               TEGRA_GPIO_PBB0
#define CAM2_LED_GPIO               TEGRA_GPIO_PU4
#define CAM1_LDO1_EN_GPIO           TEGRA_GPIO_PR6
#define CAM1_LDO2_EN_GPIO           TEGRA_GPIO_PP1
#define CAM1_LDO3_EN_GPIO           TEGRA_GPIO_PP3
#define CAM2_LDO1_EN_GPIO           TEGRA_GPIO_PR7
#define CAM2_LDO2_EN_GPIO           TEGRA_GPIO_PBB4
#define CAM2_LDO3_EN_GPIO           TEGRA_GPIO_PBB7

/* Goodix touch definition: avalon only*/
#define TS_RESET_GPIO               TEGRA_GPIO_PH6
#define TS_INT_GPIO                 TEGRA_GPIO_PH4
#define TS_3V3_EN_GPIO              TEGRA_GPIO_PH1

/* ATMEL Touch GPIO definitions: sphinx only*/
#define TOUCH_GPIO_IRQ_ATMEL_T9     TEGRA_GPIO_PO7
#define TOUCH_GPIO_RST_ATMEL_T9     TEGRA_GPIO_PO0

/* ATMEL Touch GPIO definitions: titan only */
#define MXT_IRQ_GPIO	            TEGRA_GPIO_PH4
#define MXT_RESET_GPIO	            TEGRA_GPIO_PH6
#define MXT_WAKE_GPIO	            TEGRA_GPIO_PI7

/* Sensor GPIO definitions */
#define TEMP_ALERT_GPIO             TEGRA_GPIO_PCC2
#define SEN_3V3_EN_GPIO             TEGRA_GPIO_PK5
#define LS_3V3_EN_GPIO              TEGRA_GPIO_PD0
#define AVALON_CAP_3V3_EN_GPIO      TEGRA_GPIO_PC6 /* avalon only */
#define SPHINX_CAP_3V3_EN_GPIO      TEGRA_GPIO_PN6 /* sphinx only */
#define SPHINX_SEN_1V8_EN_GPIO      TEGRA_GPIO_PD1 /* sphix only */
#define TITAN_SEN_1V8_EN_GPIO       TEGRA_GPIO_PS1 /* titan only */

/* Fuse GPIO definitions */
#define EN_3V3_FUSE_GPIO            TEGRA_GPIO_PC1

/* USB GPIO definitions */
#define EN_USB1_VBUS_GPIO           TEGRA_GPIO_PDD3
#define EN_VDDIO_VID_GPIO           TEGRA_GPIO_PP2

/* NFC GPIO definitions: avalon/titan */
#define NFC_IRQ_GPIO                TEGRA_GPIO_PS7
#define AVALON_NFC_VEN_GPIO         TEGRA_GPIO_PX7 /* avalon/titan */
#define AVALON_NFC_FIRM_GPIO        TEGRA_GPIO_PQ7 /* avalon/titan */
#define SPHINX_NFC_VEN_GPIO         TEGRA_GPIO_PD2
#define SPHINX_NFC_FIRM_GPIO        TEGRA_GPIO_PS2


/* avalon: add avalon-specific definition here */
#ifdef CONFIG_MACH_AVALON
/* Orientation matrix */
#define AST_GYRO_ORIENTATION	    { 0, 1, 0, 1, 0, 0, 0, 0, -1 }
#define AST_ACCEL_ORIENTATION	    { 1, 0, 0, 0, -1, 0, 0, 0, -1 }
#define AST_COMPASS_ORIENTATION	    { 1, 0, 0, 0, -1, 0, 0, 0, -1 }
/* iqs128 cap sensor gpio */
#define CAP_GPIO_INT	            TEGRA_GPIO_PK3
#define VDD_BACKLIGHT               (9000)
#define SD_WP_GPIO                  TEGRA_GPIO_PZ4
#endif


/* sphinx: add sphinx-specific defintion here */
#ifdef CONFIG_MACH_SPHINX
/* Orientation matrix */
#define AST_GYRO_ORIENTATION	    { 0, -1, 0, 1, 0, 0, 0, 0, 1 }
#define AST_ACCEL_ORIENTATION	    { 0, -1, 0, 1, 0, 0, 0, 0, 1 }
#define AST_COMPASS_ORIENTATION	    { 1, 0, 0, 0, -1, 0, 0, 0, -1 }
/* iqs128 cap sensor gpio */
#define CAP_GPIO_INT	            TEGRA_GPIO_PQ3
#define VDD_BACKLIGHT               (6300)
#define SD_WP_GPIO                  -1
#endif


/* titan: add titan-specific definition here */
#ifdef CONFIG_MACH_TITAN
/* Orientation matrix */
#define AST_GYRO_ORIENTATION	    { -1, 0, 0, 0, 1, 0, 0, 0, -1 }
#define AST_ACCEL_ORIENTATION	    {-1, 0, 0, 0, 1, 0, 0, 0, -1 }
#define AST_COMPASS_ORIENTATION	    { 0, 1, 0, 1, 0, 0, 0, 0, -1 }
/* iqs128 cap sensor gpio */
#define CAP_GPIO_INT	            -1
#define VDD_BACKLIGHT               (5000)
#define SD_WP_GPIO                  TEGRA_GPIO_PZ4
#endif

#endif
