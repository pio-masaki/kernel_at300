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

/* Processor Board  ID */
#define BOARD_E1187   0x0B57
#define BOARD_E1186   0x0B56
#define BOARD_E1198   0x0B62
#define BOARD_E1256   0x0C38
#define BOARD_E1257   0x0C39
#define BOARD_E1291   0x0C5B
#define BOARD_PM267   0x0243
#define BOARD_PM269   0x0245
#define BOARD_E1208   0x0C08
#define BOARD_PM305   0x0305
#define BOARD_PM311   0x030B
#define BOARD_PMU_PM298   0x0262
#define BOARD_PMU_PM299   0x0263

/* SKU Information */
#define SKU_DCDC_TPS62361_SUPPORT	0x1
#define SKU_SLT_ULPI_SUPPORT		0x2
#define SKU_T30S_SUPPORT		0x4
#define SKU_TOUCHSCREEN_MECH_FIX	0x0100

#define SKU_TOUCH_MASK			0xFF00
#define SKU_TOUCH_2000			0x0B00

#define SKU_MEMORY_TYPE_BIT		0x3
#define SKU_MEMORY_TYPE_MASK		0x7
/* If BOARD_PM269 */
#define SKU_MEMORY_SAMSUNG_EC		0x0
#define SKU_MEMORY_ELPIDA		0x2
#define SKU_MEMORY_SAMSUNG_EB		0x4
/* If BOARD_PM272 */
#define SKU_MEMORY_1GB_1R_HYNIX		0x0
#define SKU_MEMORY_2GB_2R_HYH9		0x2
/* If other BOARD_ variants */
#define SKU_MEMORY_AST_1GB_1R	0x0
#define SKU_MEMORY_AST_2GB_2R	0x2
#define SKU_MEMORY_AST_2GB_1R_HYK0	0x4
#define SKU_MEMORY_AST_2GB_1R_HYH9	0x6
#define SKU_MEMORY_AST_2GB_1R_HYNIX	0x1
#define MEMORY_TYPE(sku) (((sku) >> SKU_MEMORY_TYPE_BIT) & SKU_MEMORY_TYPE_MASK)

/* Board Fab version */
#define BOARD_FAB_A00			0x0
#define BOARD_FAB_A01			0x1
#define BOARD_FAB_A02			0x2
#define BOARD_FAB_A03			0x3
#define BOARD_FAB_A04			0x4
#define BOARD_FAB_A05			0x5

/* Display Board ID */
#define BOARD_DISPLAY_PM313		0x030D
#define BOARD_DISPLAY_E1247		0x0C2F

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

/* RICOH583 GPIO */
#define RICOH583_GPIO_BASE	TEGRA_NR_GPIOS
#define RICOH583_GPIO_END	(RICOH583_GPIO_BASE + 8)

/* WM8903 GPIOs */
#define AST_GPIO_WM8903(_x_)		(TPS6591X_GPIO_END + (_x_))
#define AST_GPIO_WM8903_END		AST_GPIO_WM8903(4)

/* Audio-related GPIOs */
#define TEGRA_GPIO_CDC_IRQ		TEGRA_GPIO_PW3
#define TEGRA_GPIO_SPKR_EN		AST_GPIO_WM8903(2)
#define TEGRA_GPIO_HP_DET		TEGRA_GPIO_PW2
#define TEGRA_GPIO_AMIC_DET		TEGRA_GPIO_PS6
#define TEGRA_GPIO_LINEOUT_DET		TEGRA_GPIO_PS3

#define AC_PRESENT_GPIO		TPS6591X_GPIO_4

/*****************Interrupt tables ******************/
/* External peripheral act as interrupt controller */
/* TPS6591x IRQs */
#define TPS6591X_IRQ_BASE	TEGRA_NR_IRQS
#define TPS6591X_IRQ_END	(TPS6591X_IRQ_BASE + 18)

/* RICOH583 IRQs */
#define RICOH583_IRQ_BASE	TEGRA_NR_IRQS
#define RICOH583_IRQ_END	(RICOH583_IRQ_BASE + RICOH583_NR_IRQS)

int ast_charge_init(void);
int ast_regulator_init(void);
int ast_suspend_init(void);
int ast_sdhci_init(void);
int ast_pinmux_init(void);
int ast_panel_init(void);
int sphinx_panel_init(void);
int ast_sensors_init(void);
int ast_kbc_init(void);
int ast_scroll_init(void);
int ast_keys_init(void);
int ast_gpio_switch_regulator_init(void);
int ast_ec_init(void);
int ast_pins_state_init(void);
int ast_emc_init(void);
int ast_power_off_init(void);
int ast_edp_init(void);
int ast_pmon_init(void);
int ast_pm299_gpio_switch_regulator_init(void);
int ast_pm299_regulator_init(void);
void __init ast_tsensor_init(void);


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

enum project_id {
	PROJECT_UNKNOWN = -1,
	PROJECT_AVALON,
	PROJECT_SPHINX,
    PROJECT_TITAN,
};

enum hw_version {
	BOARD_VERSION_UNKNOWN = -1,
	BOARD_VERSION_WS = 0,
	BOARD_VERSION_ES1 = 1,
	BOARD_VERSION_ES2 = 2,
	BOARD_VERSION_CS = 3,
    BOARD_VERSION_MP = 4,
};

struct board_version {
	u8 pj_id;
	u8 hw_version;
};

void ast_get_hw_info(struct board_version *);

#ifdef CONFIG_MACH_SPHINX
#define TOUCH_GPIO_IRQ_ATMEL_T9        TEGRA_GPIO_PO7
#define TOUCH_GPIO_RST_ATMEL_T9        TEGRA_GPIO_PO0
#define TOUCH_BUS_ATMEL_T9             1
#endif

/* Invensense MPU Definitions */
#define AST_GYRO_NAME        	"mpu3050"
#define AST_GYRO_INT         	TEGRA_GPIO_PX1
#define AST_GYRO_ADDR        	0x68
#define AST_ACCEL_NAME       	"kxtf9"
#define AST_ACC_INT          	TEGRA_GPIO_PO5
#define AST_ACCEL_ADDR       	0x0F
#define AST_COMPASS_NAME     	"ak8975"
#define AST_COMPASS_INT      	TEGRA_GPIO_PW0
#define AST_COMPASS_ADDR 	0x0C

/* Orientation matrix */
#ifdef CONFIG_MACH_AVALON
#define AST_GYRO_ORIENTATION	{ 0, 1, 0, 1, 0, 0, 0, 0, -1 }
#define AST_ACCEL_ORIENTATION	{ 1, 0, 0, 0, -1, 0, 0, 0, -1 }
#define AST_COMPASS_ORIENTATION	{ 1, 0, 0, 0, -1, 0, 0, 0, -1 }	
#endif

#ifdef CONFIG_MACH_SPHINX
#define AST_GYRO_ORIENTATION	{ 0, -1, 0, 1, 0, 0, 0, 0, 1 }
#define AST_ACCEL_ORIENTATION	{ 0, -1, 0, 1, 0, 0, 0, 0, 1 }
#define AST_COMPASS_ORIENTATION	{ 1, 0, 0, 0, -1, 0, 0, 0, -1 }
#endif

#ifdef CONFIG_MACH_TITAN
#define AST_GYRO_ORIENTATION	{ -1, 0, 0, 0, 1, 0, 0, 0, -1 }
#define AST_ACCEL_ORIENTATION	{-1, 0, 0, 0, 1, 0, 0, 0, -1 }
#define AST_COMPASS_ORIENTATION	{ 0, 1, 0, 1, 0, 0, 0, 0, -1 }
#endif

/* iqs128 cap sensor gpio */
#ifdef CONFIG_MACH_AVALON
#define CAP_GPIO_INT	TEGRA_GPIO_PK3
#endif
#ifdef CONFIG_MACH_SPHINX
#define CAP_GPIO_INT	TEGRA_GPIO_PQ3
#endif

/* Baseband GPIO addresses */
#define BB_GPIO_BB_EN			TEGRA_GPIO_PR5
#define BB_GPIO_BB_RST			TEGRA_GPIO_PS4
#define BB_GPIO_SPI_INT			TEGRA_GPIO_PS6
#define BB_GPIO_SPI_SS			TEGRA_GPIO_PV0
#define BB_GPIO_AWR			TEGRA_GPIO_PS7
#define BB_GPIO_CWR			TEGRA_GPIO_PU5

#define XMM_GPIO_BB_ON			BB_GPIO_BB_EN
#define XMM_GPIO_BB_RST			BB_GPIO_BB_RST
#define XMM_GPIO_IPC_HSIC_ACTIVE	BB_GPIO_SPI_INT
#define XMM_GPIO_IPC_HSIC_SUS_REQ	BB_GPIO_SPI_SS
#define XMM_GPIO_IPC_BB_WAKE		BB_GPIO_AWR
#define XMM_GPIO_IPC_AP_WAKE		BB_GPIO_CWR

/* Common Definition */
#define TDIODE_OFFSET                 (10000) /* in millicelsius */
#define AST_SD_CD                     TEGRA_GPIO_PI5

/* avalon: we may add avalon-specific definition here */
#ifdef CONFIG_MACH_AVALON
#define VDD_BACKLIGHT_VOLTAGE         (5000)
#define AST_SD_WP                     TEGRA_GPIO_PZ4
#endif

/* sphinx: we may add sphinx-specific definition here */
#ifdef CONFIG_MACH_SPHINX
#define VDD_BACKLIGHT_VOLTAGE         (6300)
#define AST_SD_WP                     -1

#endif

/* titan: we may add titan-specific definition here */
#ifdef CONFIG_MACH_TITAN
#define VDD_BACKLIGHT_VOLTAGE         (5000)
#define AST_SD_WP                     TEGRA_GPIO_PZ4
#endif
enum power_type{
 WIFI_POWER=0,
 BT_POWER=1
};
#endif
