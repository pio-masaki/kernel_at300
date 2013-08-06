/*
 * arch/arm/mach-tegra/board-ast-pinmux.c
 *
 * Copyright (C) 2010-2011 NVIDIA Corporation
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

#include <linux/kernel.h>
#include <linux/init.h>
#include <mach/pinmux.h>
#include <asm/mach-types.h>

#include "board.h"
#include "board-ast.h"
#include "gpio-names.h"

#define DEFAULT_DRIVE(_name)					\
	{							\
		.pingroup = TEGRA_DRIVE_PINGROUP_##_name,	\
		.hsm = TEGRA_HSM_DISABLE,			\
		.schmitt = TEGRA_SCHMITT_ENABLE,		\
		.drive = TEGRA_DRIVE_DIV_1,			\
		.pull_down = TEGRA_PULL_31,			\
		.pull_up = TEGRA_PULL_31,			\
		.slew_rising = TEGRA_SLEW_SLOWEST,		\
		.slew_falling = TEGRA_SLEW_SLOWEST,		\
	}
/* Setting the drive strength of pins
 * hsm: Enable High speed mode (ENABLE/DISABLE)
 * Schimit: Enable/disable schimit (ENABLE/DISABLE)
 * drive: low power mode (DIV_1, DIV_2, DIV_4, DIV_8)
 * pulldn_drive - drive down (falling edge) - Driver Output Pull-Down drive
 *                strength code. Value from 0 to 31.
 * pullup_drive - drive up (rising edge)  - Driver Output Pull-Up drive
 *                strength code. Value from 0 to 31.
 * pulldn_slew -  Driver Output Pull-Up slew control code  - 2bit code
 *                code 11 is least slewing of signal. code 00 is highest
 *                slewing of the signal.
 *                Value - FASTEST, FAST, SLOW, SLOWEST
 * pullup_slew -  Driver Output Pull-Down slew control code -
 *                code 11 is least slewing of signal. code 00 is highest
 *                slewing of the signal.
 *                Value - FASTEST, FAST, SLOW, SLOWEST
 */
#define SET_DRIVE(_name, _hsm, _schmitt, _drive, _pulldn_drive, _pullup_drive, _pulldn_slew, _pullup_slew) \
	{                                               \
		.pingroup = TEGRA_DRIVE_PINGROUP_##_name,   \
		.hsm = TEGRA_HSM_##_hsm,                    \
		.schmitt = TEGRA_SCHMITT_##_schmitt,        \
		.drive = TEGRA_DRIVE_##_drive,              \
		.pull_down = TEGRA_PULL_##_pulldn_drive,    \
		.pull_up = TEGRA_PULL_##_pullup_drive,		\
		.slew_rising = TEGRA_SLEW_##_pulldn_slew,   \
		.slew_falling = TEGRA_SLEW_##_pullup_slew,	\
	}

/* !!!FIXME!!!! POPULATE THIS TABLE */
static __initdata struct tegra_drive_pingroup_config ast_drive_pinmux[] = {
	/* DEFAULT_DRIVE(<pin_group>), */
	/* SET_DRIVE(ATA, DISABLE, DISABLE, DIV_1, 31, 31, FAST, FAST) */
	SET_DRIVE(DAP2, 	DISABLE, ENABLE, DIV_1, 31, 31, FASTEST, FASTEST),

	/* All I2C pins are driven to maximum drive strength */
	/* GEN1 I2C */
	SET_DRIVE(DBG,		DISABLE, ENABLE, DIV_1, 31, 31, FASTEST, FASTEST),

	/* GEN2 I2C */
	SET_DRIVE(AT5,		DISABLE, ENABLE, DIV_1, 31, 31, FASTEST, FASTEST),

	/* CAM I2C */
	SET_DRIVE(GME,		DISABLE, ENABLE, DIV_1, 31, 31, FASTEST, FASTEST),

	/* DDC I2C */
	SET_DRIVE(DDC,		DISABLE, ENABLE, DIV_1, 31, 31, FASTEST, FASTEST),

	/* PWR_I2C */
	SET_DRIVE(AO1,		DISABLE, ENABLE, DIV_1, 31, 31, FASTEST, FASTEST),

	/* UART3 */
	SET_DRIVE(UART3,	DISABLE, ENABLE, DIV_1, 31, 31, FASTEST, FASTEST),

	/* SDMMC1 */
	SET_DRIVE(SDIO1,	DISABLE, DISABLE, DIV_1, 46, 42, FAST, FAST),

	/* SDMMC3 */
	SET_DRIVE(SDIO3,	DISABLE, DISABLE, DIV_1, 46, 42, FAST, FAST),

	/* SDMMC4 */
	SET_DRIVE(GMA,		DISABLE, DISABLE, DIV_1, 9, 9, SLOWEST, SLOWEST),
	SET_DRIVE(GMB,		DISABLE, DISABLE, DIV_1, 9, 9, SLOWEST, SLOWEST),
	SET_DRIVE(GMC,		DISABLE, DISABLE, DIV_1, 9, 9, SLOWEST, SLOWEST),
	SET_DRIVE(GMD,		DISABLE, DISABLE, DIV_1, 9, 9, SLOWEST, SLOWEST),

};

#define DEFAULT_PINMUX(_pingroup, _mux, _pupd, _tri, _io)	\
	{							\
		.pingroup	= TEGRA_PINGROUP_##_pingroup,	\
		.func		= TEGRA_MUX_##_mux,		\
		.pupd		= TEGRA_PUPD_##_pupd,		\
		.tristate	= TEGRA_TRI_##_tri,		\
		.io		= TEGRA_PIN_##_io,		\
		.lock		= TEGRA_PIN_LOCK_DEFAULT,	\
		.od		= TEGRA_PIN_OD_DEFAULT,		\
		.ioreset	= TEGRA_PIN_IO_RESET_DEFAULT,	\
	}

#define I2C_PINMUX(_pingroup, _mux, _pupd, _tri, _io, _lock, _od) \
	{							\
		.pingroup	= TEGRA_PINGROUP_##_pingroup,	\
		.func		= TEGRA_MUX_##_mux,		\
		.pupd		= TEGRA_PUPD_##_pupd,		\
		.tristate	= TEGRA_TRI_##_tri,		\
		.io		= TEGRA_PIN_##_io,		\
		.lock		= TEGRA_PIN_LOCK_##_lock,	\
		.od		= TEGRA_PIN_OD_##_od,		\
		.ioreset	= TEGRA_PIN_IO_RESET_DEFAULT,	\
	}

#define VI_PINMUX(_pingroup, _mux, _pupd, _tri, _io, _lock, _ioreset) \
	{							\
		.pingroup	= TEGRA_PINGROUP_##_pingroup,	\
		.func		= TEGRA_MUX_##_mux,		\
		.pupd		= TEGRA_PUPD_##_pupd,		\
		.tristate	= TEGRA_TRI_##_tri,		\
		.io		= TEGRA_PIN_##_io,		\
		.lock		= TEGRA_PIN_LOCK_##_lock,	\
		.od		= TEGRA_PIN_OD_DEFAULT,		\
		.ioreset	= TEGRA_PIN_IO_RESET_##_ioreset	\
	}

static __initdata struct tegra_pingroup_config ast_pinmux_common[] = {
	/* SDMMC1 pinmux */
	DEFAULT_PINMUX(SDMMC1_CLK,      SDMMC1,          NORMAL,     NORMAL,     INPUT),
	DEFAULT_PINMUX(SDMMC1_CMD,      SDMMC1,          PULL_UP,    NORMAL,     INPUT),
	DEFAULT_PINMUX(SDMMC1_DAT3,     SDMMC1,          PULL_UP,    NORMAL,     INPUT),
	DEFAULT_PINMUX(SDMMC1_DAT2,     SDMMC1,          PULL_UP,    NORMAL,     INPUT),
	DEFAULT_PINMUX(SDMMC1_DAT1,     SDMMC1,          PULL_UP,    NORMAL,     INPUT),
	DEFAULT_PINMUX(SDMMC1_DAT0,     SDMMC1,          PULL_UP,    NORMAL,     INPUT),

	/* SDMMC3 pinmux */
	DEFAULT_PINMUX(SDMMC3_CLK,      SDMMC3,          NORMAL,     NORMAL,     INPUT),
	DEFAULT_PINMUX(SDMMC3_CMD,      SDMMC3,          PULL_UP,    NORMAL,     INPUT),
	DEFAULT_PINMUX(SDMMC3_DAT0,     SDMMC3,          PULL_UP,    NORMAL,     INPUT),
	DEFAULT_PINMUX(SDMMC3_DAT1,     SDMMC3,          PULL_UP,    NORMAL,     INPUT),
	DEFAULT_PINMUX(SDMMC3_DAT2,     SDMMC3,          PULL_UP,    NORMAL,     INPUT),
	DEFAULT_PINMUX(SDMMC3_DAT3,     SDMMC3,          PULL_UP,    NORMAL,     INPUT),
	DEFAULT_PINMUX(SDMMC3_DAT7,     SDMMC3,          PULL_UP,    NORMAL,     INPUT),

	/* SDMMC4 pinmux */
	DEFAULT_PINMUX(SDMMC4_CLK,      SDMMC4,          NORMAL,     NORMAL,     INPUT),
	DEFAULT_PINMUX(SDMMC4_CMD,      SDMMC4,          PULL_UP,    NORMAL,     INPUT),
	DEFAULT_PINMUX(SDMMC4_DAT0,     SDMMC4,          PULL_UP,    NORMAL,     INPUT),
	DEFAULT_PINMUX(SDMMC4_DAT1,     SDMMC4,          PULL_UP,    NORMAL,     INPUT),
	DEFAULT_PINMUX(SDMMC4_DAT2,     SDMMC4,          PULL_UP,    NORMAL,     INPUT),
	DEFAULT_PINMUX(SDMMC4_DAT3,     SDMMC4,          PULL_UP,    NORMAL,     INPUT),
	DEFAULT_PINMUX(SDMMC4_DAT4,     SDMMC4,          PULL_UP,    NORMAL,     INPUT),
	DEFAULT_PINMUX(SDMMC4_DAT5,     SDMMC4,          PULL_UP,    NORMAL,     INPUT),
	DEFAULT_PINMUX(SDMMC4_DAT6,     SDMMC4,          PULL_UP,    NORMAL,     INPUT),
	DEFAULT_PINMUX(SDMMC4_DAT7,     SDMMC4,          PULL_UP,    NORMAL,     INPUT),
	DEFAULT_PINMUX(SDMMC4_RST_N,    RSVD1,           PULL_UP,    NORMAL,     OUTPUT),

	/* I2C1 pinmux */
	I2C_PINMUX(GEN1_I2C_SCL,	I2C1,		NORMAL,	NORMAL,	INPUT,	DISABLE,	ENABLE),
	I2C_PINMUX(GEN1_I2C_SDA,	I2C1,		NORMAL,	NORMAL,	INPUT,	DISABLE,	ENABLE),

	/* I2C2 pinmux */
	I2C_PINMUX(GEN2_I2C_SCL,	I2C2,		NORMAL,	NORMAL,	INPUT,	DISABLE,	ENABLE),
	I2C_PINMUX(GEN2_I2C_SDA,	I2C2,		NORMAL,	NORMAL,	INPUT,	DISABLE,	ENABLE),

	/* I2C3 pinmux */
	I2C_PINMUX(CAM_I2C_SCL,		I2C3,		NORMAL,	NORMAL,	INPUT,	DISABLE,	ENABLE),
	I2C_PINMUX(CAM_I2C_SDA,		I2C3,		NORMAL,	NORMAL,	INPUT,	DISABLE,	ENABLE),

	/* I2C4 pinmux */
	I2C_PINMUX(DDC_SCL,		I2C4,		NORMAL,	NORMAL,	INPUT,	DISABLE,	ENABLE),
	I2C_PINMUX(DDC_SDA,		I2C4,		NORMAL,	NORMAL,	INPUT,	DISABLE,	ENABLE),

	/* Power I2C pinmux */
	I2C_PINMUX(PWR_I2C_SCL,		I2CPWR,		NORMAL,	NORMAL,	INPUT,	DISABLE,	ENABLE),
	I2C_PINMUX(PWR_I2C_SDA,		I2CPWR,		NORMAL,	NORMAL,	INPUT,	DISABLE,	ENABLE),

	DEFAULT_PINMUX(ULPI_DATA7,      UARTA,           PULL_UP,   TRISTATE,   OUTPUT), /* unused */
    DEFAULT_PINMUX(ULPI_CLK,        UARTD,           NORMAL,    NORMAL,     OUTPUT),
	DEFAULT_PINMUX(ULPI_DIR,        UARTD,           NORMAL,    NORMAL,     INPUT),
	DEFAULT_PINMUX(ULPI_NXT,        UARTD,           NORMAL,    NORMAL,     INPUT),
	DEFAULT_PINMUX(ULPI_STP,        UARTD,           NORMAL,    NORMAL,     OUTPUT),

	DEFAULT_PINMUX(LCD_WR_N,        DISPLAYA,        NORMAL,    NORMAL,     INPUT),
	DEFAULT_PINMUX(LCD_PCLK,        DISPLAYA,        NORMAL,    NORMAL,     INPUT),
	DEFAULT_PINMUX(LCD_DE,          DISPLAYA,        NORMAL,    NORMAL,     INPUT),
	DEFAULT_PINMUX(LCD_HSYNC,       DISPLAYA,        NORMAL,    NORMAL,     INPUT),
	DEFAULT_PINMUX(LCD_VSYNC,       DISPLAYA,        NORMAL,    NORMAL,     INPUT),

	DEFAULT_PINMUX(LCD_D0,          DISPLAYA,        NORMAL,    NORMAL,     OUTPUT),
	DEFAULT_PINMUX(LCD_D1,          DISPLAYA,        NORMAL,    NORMAL,     OUTPUT),
	DEFAULT_PINMUX(LCD_D2,          DISPLAYA,        NORMAL,    NORMAL,     OUTPUT),
	DEFAULT_PINMUX(LCD_D3,          DISPLAYA,        NORMAL,    NORMAL,     OUTPUT),
	DEFAULT_PINMUX(LCD_D4,          DISPLAYA,        NORMAL,    NORMAL,     OUTPUT),
	DEFAULT_PINMUX(LCD_D5,          DISPLAYA,        NORMAL,    NORMAL,     OUTPUT),
	DEFAULT_PINMUX(LCD_D6,          DISPLAYA,        NORMAL,    NORMAL,     OUTPUT),
	DEFAULT_PINMUX(LCD_D7,          DISPLAYA,        NORMAL,    NORMAL,     OUTPUT),
	DEFAULT_PINMUX(LCD_D8,          DISPLAYA,        NORMAL,    NORMAL,     OUTPUT),
	DEFAULT_PINMUX(LCD_D9,          DISPLAYA,        NORMAL,    NORMAL,     OUTPUT),
	DEFAULT_PINMUX(LCD_D10,         DISPLAYA,        NORMAL,    NORMAL,     OUTPUT),
	DEFAULT_PINMUX(LCD_D11,         DISPLAYA,        NORMAL,    NORMAL,     OUTPUT),
	DEFAULT_PINMUX(LCD_D12,         DISPLAYA,        NORMAL,    NORMAL,     OUTPUT),
	DEFAULT_PINMUX(LCD_D13,         DISPLAYA,        NORMAL,    NORMAL,     OUTPUT),
	DEFAULT_PINMUX(LCD_D14,         DISPLAYA,        NORMAL,    NORMAL,     OUTPUT),
	DEFAULT_PINMUX(LCD_D15,         DISPLAYA,        NORMAL,    NORMAL,     OUTPUT),
	DEFAULT_PINMUX(LCD_D16,         DISPLAYA,        NORMAL,    NORMAL,     OUTPUT),
	DEFAULT_PINMUX(LCD_D17,         DISPLAYA,        NORMAL,    NORMAL,     OUTPUT),

	DEFAULT_PINMUX(VI_D0,           RSVD1,           NORMAL,    NORMAL,     INPUT),
	DEFAULT_PINMUX(VI_D1,           SDMMC2,          NORMAL,    NORMAL,     INPUT),
	DEFAULT_PINMUX(VI_D2,           SDMMC2,          NORMAL,    NORMAL,     INPUT),
	DEFAULT_PINMUX(VI_D3,           SDMMC2,          NORMAL,    NORMAL,     INPUT),
	DEFAULT_PINMUX(VI_D4,           VI,              NORMAL,    NORMAL,     OUTPUT),
	DEFAULT_PINMUX(VI_D5,           SDMMC2,          NORMAL,    NORMAL,     INPUT),
	DEFAULT_PINMUX(VI_D7,           SDMMC2,          NORMAL,    NORMAL,     INPUT),
	DEFAULT_PINMUX(VI_D10,          RSVD1,           NORMAL,    NORMAL,     INPUT),
	DEFAULT_PINMUX(VI_MCLK,         VI,              PULL_UP,   NORMAL,     INPUT),

	DEFAULT_PINMUX(UART2_RXD,       IRDA,            NORMAL,    NORMAL,     INPUT),
	DEFAULT_PINMUX(UART2_TXD,       IRDA,            NORMAL,    NORMAL,     OUTPUT),
	DEFAULT_PINMUX(UART2_RTS_N,     UARTB,           NORMAL,    NORMAL,     OUTPUT),
	DEFAULT_PINMUX(UART2_CTS_N,     UARTB,           NORMAL,    NORMAL,     INPUT),
	DEFAULT_PINMUX(UART3_TXD,       UARTC,           NORMAL,    NORMAL,     OUTPUT),
	DEFAULT_PINMUX(UART3_RXD,       UARTC,           NORMAL,    NORMAL,     INPUT),
	DEFAULT_PINMUX(UART3_CTS_N,     UARTC,           NORMAL,    NORMAL,     INPUT),
	DEFAULT_PINMUX(UART3_RTS_N,     UARTC,           NORMAL,    NORMAL,     OUTPUT),

	DEFAULT_PINMUX(GPIO_PU5,        PWM2,            PULL_DOWN, TRISTATE,   OUTPUT), /* unused */

	DEFAULT_PINMUX(DAP4_FS,         I2S3,            NORMAL,    NORMAL,     INPUT),
	DEFAULT_PINMUX(DAP4_DIN,        I2S3,            NORMAL,    NORMAL,     INPUT),
	DEFAULT_PINMUX(DAP4_DOUT,       I2S3,            NORMAL,    NORMAL,     INPUT),
	DEFAULT_PINMUX(DAP4_SCLK,       I2S3,            NORMAL,    NORMAL,     INPUT),
	DEFAULT_PINMUX(CLK3_OUT,        EXTPERIPH3,      PULL_DOWN, TRISTATE,   OUTPUT), /* unused */
	DEFAULT_PINMUX(CLK3_REQ,        DEV3,            PULL_DOWN, TRISTATE,   OUTPUT), /* unused */

    DEFAULT_PINMUX(KB_ROW8,         KBC,             PULL_DOWN, TRISTATE,   OUTPUT), /* unused */
    DEFAULT_PINMUX(KB_ROW13,        KBC,             PULL_DOWN, TRISTATE,   OUTPUT), /* unused */

	DEFAULT_PINMUX(GMI_ADV_N,       NAND,            NORMAL,    NORMAL,     OUTPUT),
	DEFAULT_PINMUX(GMI_CLK,         NAND,            NORMAL,    NORMAL,     OUTPUT),
	DEFAULT_PINMUX(GMI_CS3_N,       NAND,            PULL_UP,   TRISTATE,   OUTPUT), /* unused */
	DEFAULT_PINMUX(GMI_AD0,         NAND,            NORMAL,    NORMAL,     INPUT),
	DEFAULT_PINMUX(GMI_AD1,         NAND,            NORMAL,    NORMAL,     INPUT),
	DEFAULT_PINMUX(GMI_AD2,         NAND,            NORMAL,    NORMAL,     INPUT),
	DEFAULT_PINMUX(GMI_AD3,         NAND,            NORMAL,    NORMAL,     INPUT),
	DEFAULT_PINMUX(GMI_AD4,         NAND,            NORMAL,    NORMAL,     INPUT),
	DEFAULT_PINMUX(GMI_AD5,         NAND,            NORMAL,    NORMAL,     INPUT),
	DEFAULT_PINMUX(GMI_AD6,         NAND,            NORMAL,    NORMAL,     INPUT),
	DEFAULT_PINMUX(GMI_AD7,         NAND,            NORMAL,    NORMAL,     INPUT),
	DEFAULT_PINMUX(GMI_WR_N,        NAND,            NORMAL,    NORMAL,     OUTPUT),
	DEFAULT_PINMUX(GMI_OE_N,        NAND,            NORMAL,    NORMAL,     OUTPUT),

    DEFAULT_PINMUX(GMI_RST_N,       NAND,            PULL_UP,   TRISTATE,   OUTPUT), /* unused */
    DEFAULT_PINMUX(GMI_AD11,        PWM3,            NORMAL,    NORMAL,     OUTPUT), /* HAPTIC_PWM */

	DEFAULT_PINMUX(CAM_MCLK,        VI_ALT2,         PULL_UP,   NORMAL,     INPUT),
	DEFAULT_PINMUX(GPIO_PCC1,       RSVD1,           PULL_UP,   TRISTATE,   OUTPUT), /* unused */

	DEFAULT_PINMUX(JTAG_RTCK,       RTCK,            NORMAL,    NORMAL,     OUTPUT),

	DEFAULT_PINMUX(CLK_32K_OUT,     BLINK,           NORMAL,    NORMAL,     OUTPUT),
	DEFAULT_PINMUX(SYS_CLK_REQ,     SYSCLK,          PULL_DOWN, TRISTATE,   OUTPUT), /* unused */
	DEFAULT_PINMUX(OWR,             OWR,             PULL_UP,   TRISTATE,   OUTPUT), /* unused */

	DEFAULT_PINMUX(CLK1_REQ,        DAP,             PULL_DOWN, TRISTATE,   OUTPUT), /* unused */
	DEFAULT_PINMUX(CLK1_OUT,        EXTPERIPH1,      NORMAL,    NORMAL,     INPUT),

	DEFAULT_PINMUX(DAP2_FS,         I2S1,            NORMAL,    NORMAL,     INPUT),
	DEFAULT_PINMUX(DAP2_DIN,        I2S1,            NORMAL,    NORMAL,     INPUT),
	DEFAULT_PINMUX(DAP2_DOUT,       I2S1,            NORMAL,    NORMAL,     INPUT),
	DEFAULT_PINMUX(DAP2_SCLK,       I2S1,            NORMAL,    NORMAL,     INPUT),

	DEFAULT_PINMUX(PEX_L0_PRSNT_N,  PCIE,            PULL_DOWN, TRISTATE,   OUTPUT), /* unused */
	DEFAULT_PINMUX(PEX_L0_RST_N,    PCIE,            PULL_DOWN, TRISTATE,   OUTPUT), /* unused */
	DEFAULT_PINMUX(PEX_L0_CLKREQ_N, PCIE,            PULL_DOWN, TRISTATE,   OUTPUT), /* unused */

	DEFAULT_PINMUX(PEX_L1_PRSNT_N,  PCIE,            PULL_DOWN, TRISTATE,   OUTPUT), /* unused */
	DEFAULT_PINMUX(PEX_L1_RST_N,    PCIE,            PULL_DOWN, TRISTATE,   OUTPUT), /* unused */
	DEFAULT_PINMUX(PEX_L1_CLKREQ_N, PCIE,            PULL_DOWN, TRISTATE,   OUTPUT), /* unused */

	DEFAULT_PINMUX(HDMI_CEC,        CEC,             PULL_DOWN, TRISTATE,   OUTPUT), /* unused */
	DEFAULT_PINMUX(HDMI_INT,        RSVD0,           NORMAL,    TRISTATE,   INPUT),

    /* AST Common GPIOs */
    DEFAULT_PINMUX(DAP1_DIN,        I2S0,            NORMAL,    NORMAL,     OUTPUT), /* VD_PWD# */
    DEFAULT_PINMUX(DAP1_FS,         I2S0,            NORMAL,    NORMAL,     OUTPUT), /* VD_RST# */
    DEFAULT_PINMUX(DAP1_SCLK,       I2S0,            NORMAL,    NORMAL,     OUTPUT), /* VD_BP# */
    DEFAULT_PINMUX(SPI2_CS1_N,      SPI2,            NORMAL,    NORMAL,     INPUT), /* HP_DET */
    DEFAULT_PINMUX(SPI2_CS2_N,      SPI2,            PULL_UP,   NORMAL,     INPUT), /* CDC_IRQ */
    DEFAULT_PINMUX(SPI2_MISO,       SPI2,            NORMAL,    NORMAL,     INPUT), /* GYRO_IRQ */
    DEFAULT_PINMUX(SPI2_MOSI,       SPI6,            NORMAL,    NORMAL,     OUTPUT), /* D_MIC_EN */
    DEFAULT_PINMUX(GPIO_PV0,        RSVD,            PULL_UP,   NORMAL,     INPUT), /* AP_ONKEY */
    DEFAULT_PINMUX(GPIO_PV1,        RSVD,            PULL_UP,   NORMAL,     INPUT), /* AP_ACOK */
    DEFAULT_PINMUX(ULPI_DATA0,      UARTA,           NORMAL,    NORMAL,     OUTPUT), /* EN_MIC_EXT# */
    DEFAULT_PINMUX(ULPI_DATA3,      UARTA,           NORMAL,    NORMAL,     INPUT), /* WF_WAKEUP */
    DEFAULT_PINMUX(ULPI_DATA4,      UARTA,           NORMAL,    NORMAL,     INPUT), /* G_SENSOR_INT */
    DEFAULT_PINMUX(GPIO_PBB0,       RSVD1,           NORMAL,    NORMAL,     OUTPUT), /* CAM2_RST */
	DEFAULT_PINMUX(GPIO_PBB3,       VGP3,            NORMAL,    NORMAL,     OUTPUT), /* CAM1_RST */
	DEFAULT_PINMUX(GPIO_PBB5,       VGP5,            NORMAL,    NORMAL,     OUTPUT), /* CAM1_PWDN */
	DEFAULT_PINMUX(GPIO_PBB6,       VGP6,            NORMAL,    NORMAL,     OUTPUT), /* CAM2_PWDN */
    DEFAULT_PINMUX(GPIO_PCC2,       I2S4,            NORMAL,    NORMAL,     INPUT), /* TEMP_ALERT */
    DEFAULT_PINMUX(GMI_A16,         SPI4,            NORMAL,    NORMAL,     INPUT), /* EC_WAKE# */
	DEFAULT_PINMUX(GMI_A17,         SPI4,            NORMAL,    NORMAL,     OUTPUT), /* EC_REQUEST# */
    DEFAULT_PINMUX(GMI_AD15,        NAND,            NORMAL,    NORMAL,     OUTPUT), /* VIBRATOR_EN */
    DEFAULT_PINMUX(GMI_CS0_N,       NAND,            NORMAL,    NORMAL,     INPUT), /* USB1_WAKE */
    DEFAULT_PINMUX(GMI_CS1_N,       NAND,            NORMAL,    NORMAL,     INPUT), /* USB2_WAKE */
    DEFAULT_PINMUX(GMI_IORDY,       RSVD1,           PULL_UP,   NORMAL,     INPUT), /* SDMMC_CD */
    DEFAULT_PINMUX(LCD_CS1_N,       DISPLAYA,        NORMAL,    NORMAL,     INPUT), /* COMPASS_DRDY */
    DEFAULT_PINMUX(LCD_PWR0,        DISPLAYA,        NORMAL,    NORMAL,     OUTPUT), /* BT_SHUTDOWN */
    DEFAULT_PINMUX(PEX_L2_CLKREQ_N, PCIE,            NORMAL,    NORMAL,     INPUT), /* EN_3V3_PU */
    DEFAULT_PINMUX(PEX_L2_PRSNT_N,  PCIE,            NORMAL,    NORMAL,     OUTPUT), /* EC_PWR_STATE */
    DEFAULT_PINMUX(PEX_WAKE_N,      PCIE,            NORMAL,    NORMAL,     OUTPUT), /* EN_USB1_VBUS_OC */
    DEFAULT_PINMUX(SDMMC3_DAT6,     SDMMC3,          PULL_UP,   NORMAL,     OUTPUT), /* WF_RST */
    DEFAULT_PINMUX(KB_COL0,         KBC,             PULL_UP,   NORMAL,     INPUT), /* VOL_UP_BUTTON */
    DEFAULT_PINMUX(KB_COL1,         KBC,             PULL_UP,   NORMAL,     INPUT), /* VOL_DWN_BUTTON */
    DEFAULT_PINMUX(KB_ROW0,         KBC,             NORMAL,    TRISTATE,   INPUT), /* BOARD_ID_A0 */
	DEFAULT_PINMUX(KB_ROW1,         KBC,             NORMAL,    TRISTATE,   INPUT), /* BOARD_ID_A1 */
	DEFAULT_PINMUX(KB_ROW2,         KBC,             NORMAL,    TRISTATE,   INPUT), /* BOARD_ID_A2 */
	DEFAULT_PINMUX(KB_ROW3,         KBC,             NORMAL,    TRISTATE,   INPUT), /* BOARD_ID_A3 */
	DEFAULT_PINMUX(KB_ROW4,         KBC,             NORMAL,    TRISTATE,   INPUT), /* BOARD_ID_A4 */
    DEFAULT_PINMUX(KB_ROW5,         OWR,             PULL_UP,   NORMAL,     INPUT), /* LOCK# */
    DEFAULT_PINMUX(KB_ROW14,        KBC,             NORMAL,    NORMAL,     INPUT), /* MIC_DETECT# */
	DEFAULT_PINMUX(KB_ROW15,        KBC,             NORMAL,    NORMAL,     INPUT), /* A_NFC_IRQ_EE */

    DEFAULT_PINMUX(GPIO_PU0,        RSVD1,           NORMAL,    NORMAL,     OUTPUT), /* T30S_BT_RST */
	DEFAULT_PINMUX(GPIO_PU1,        RSVD1,           NORMAL,    NORMAL,     OUTPUT), /* T30S_BT_WAKEUP */
	DEFAULT_PINMUX(GPIO_PU2,        RSVD1,           NORMAL,    NORMAL,     OUTPUT), /* T30_GPS_EN */
	DEFAULT_PINMUX(GPIO_PU3,        RSVD1,           NORMAL,    NORMAL,     OUTPUT), /* T30_GPS_RST */
	DEFAULT_PINMUX(GPIO_PU4,        PWM1,            NORMAL,    NORMAL,     OUTPUT), /* Camera_LED_EN */
    DEFAULT_PINMUX(GPIO_PU6,        RSVD1,           NORMAL,    NORMAL,     INPUT), /* BT_IRQ */

    /* AST Common Power rails */
    DEFAULT_PINMUX(SPDIF_OUT,       SPDIF,           NORMAL,    NORMAL,     OUTPUT), /* SEN_3V3_EN */
    DEFAULT_PINMUX(DAP3_DIN,        I2S2,            NORMAL,    NORMAL,     OUTPUT), /* CAM1_LDO2_EN */
    DEFAULT_PINMUX(DAP3_SCLK,       I2S2,            NORMAL,    NORMAL,     OUTPUT), /* CAM1_LDO3_EN */
    DEFAULT_PINMUX(GPIO_PBB4,       VGP4,            NORMAL,    NORMAL,     OUTPUT), /* CAM2_LDO2_EN */
	DEFAULT_PINMUX(GPIO_PBB7,       I2S4,            NORMAL,    NORMAL,     OUTPUT), /* CAM2_LDO3_EN */
    DEFAULT_PINMUX(GMI_A19,         SPI4,            PULL_UP,   NORMAL,     OUTPUT), /* EN_VDD_BL1 */

    DEFAULT_PINMUX(LCD_M1,          DISPLAYA,        PULL_UP,    NORMAL,     OUTPUT), /* EN_VDD_PNL1 */

	DEFAULT_PINMUX(LCD_PWR1,        DISPLAYA,        NORMAL,    NORMAL,     OUTPUT), /* EN_3V3_FUSE */
    DEFAULT_PINMUX(CLK2_OUT,        EXTPERIPH2,      NORMAL,    NORMAL,     OUTPUT), /* EN_GPS_1V8 */
	DEFAULT_PINMUX(CLK2_REQ,        DAP,             NORMAL,    NORMAL,     OUTPUT), /* EN_WLAN_1V8 */
	DEFAULT_PINMUX(GPIO_PV2,        OWR,             NORMAL,    NORMAL,     OUTPUT), /* EN_GPS_3V3 */
	DEFAULT_PINMUX(GPIO_PV3,        RSVD1,           NORMAL,    NORMAL,     OUTPUT), /* EN_WLAN_3V3 */
    DEFAULT_PINMUX(SDMMC3_DAT5,     SDMMC3,          PULL_UP,   NORMAL,     OUTPUT), /* LS_3V3_EN */
    DEFAULT_PINMUX(KB_ROW6,         KBC,             NORMAL,    NORMAL,     OUTPUT), /* CAM1_LDO1_EN */
    DEFAULT_PINMUX(KB_ROW7,         KBC,             NORMAL,    NORMAL,     OUTPUT), /* CAM2_LDO1_EN */

    /* These pinmuxs should be not available in T30S */
	VI_PINMUX(VI_D6,           VI,              PULL_DOWN,    TRISTATE,     OUTPUT,  DISABLE, DISABLE),
	VI_PINMUX(VI_D8,           SDMMC2,          PULL_DOWN,    TRISTATE,     OUTPUT,  DISABLE, DISABLE),
	VI_PINMUX(VI_D9,           SDMMC2,          PULL_DOWN,    TRISTATE,     OUTPUT,  DISABLE, DISABLE),
	VI_PINMUX(VI_D11,          RSVD1,           PULL_DOWN,    TRISTATE,     OUTPUT,  DISABLE, DISABLE),
	VI_PINMUX(VI_PCLK,         RSVD1,           PULL_DOWN,    TRISTATE,     OUTPUT,  DISABLE, DISABLE),
	VI_PINMUX(VI_HSYNC,        RSVD1,           PULL_DOWN,    TRISTATE,     OUTPUT,  DISABLE, DISABLE),
	VI_PINMUX(VI_VSYNC,        RSVD1,           PULL_DOWN,    TRISTATE,     OUTPUT,  DISABLE, DISABLE),
};

/* pinmux: please add avalon-specific pinmux here */
static __initdata struct tegra_pingroup_config avalon_pinmux[] = {
                  /* pingrp         mux              pupd       tri         io  */
    DEFAULT_PINMUX(DAP1_DOUT,       I2S0,            NORMAL,    NORMAL,     INPUT), /* WW_WAKE */
    DEFAULT_PINMUX(SPDIF_IN,        SPDIF,           PULL_UP,   TRISTATE,   OUTPUT), /* unused */
    DEFAULT_PINMUX(SPI1_MISO,       SPI1,            NORMAL,    NORMAL,     OUTPUT), /* R_NFC_VEN_EE */
    DEFAULT_PINMUX(SPI1_MOSI,       SPI1,            PULL_UP,   TRISTATE,   OUTPUT), /* unused */
    DEFAULT_PINMUX(SPI1_SCK,        SPI1,            PULL_UP,   TRISTATE,   OUTPUT), /* unused */
    DEFAULT_PINMUX(SPI2_CS0_N,      SPI2,            PULL_UP,   TRISTATE,   OUTPUT), /* unused */
    DEFAULT_PINMUX(SPI2_SCK,        GMI,             PULL_UP,   TRISTATE,   OUTPUT), /* unused */
    DEFAULT_PINMUX(DAP3_DOUT,       I2S2,            PULL_UP,   NORMAL,     OUTPUT), /* EN_VDDIO_VID_OC */
    DEFAULT_PINMUX(ULPI_DATA1,      UARTA,           NORMAL,    NORMAL,     OUTPUT), /* R_ON1 */
    DEFAULT_PINMUX(ULPI_DATA2,      UARTA,           PULL_UP,   TRISTATE,   OUTPUT), /* unused */
    DEFAULT_PINMUX(ULPI_DATA5,      UARTA,           PULL_UP,   TRISTATE,   OUTPUT), /* unused */
	DEFAULT_PINMUX(ULPI_DATA6,      UARTA,           PULL_UP,   TRISTATE,   OUTPUT), /* unused */
    DEFAULT_PINMUX(GMI_A18,         SPI4,            PULL_UP,   NORMAL,     OUTPUT), /* LOW_BAT#_EC */
    DEFAULT_PINMUX(GMI_AD10,        NAND,            PULL_UP,   NORMAL,     OUTPUT), /* LCD1_BL_EN */
    DEFAULT_PINMUX(GMI_AD12,        NAND,            NORMAL,    NORMAL,     INPUT), /* TS_INT */
    DEFAULT_PINMUX(GMI_AD13,        NAND,            NORMAL,    NORMAL,     OUTPUT), /* HDMI_SW_EN */
	DEFAULT_PINMUX(GMI_AD14,        NAND,            NORMAL,    NORMAL,     OUTPUT), /* TS_RESET */
    DEFAULT_PINMUX(GMI_AD8,         PWM0,            PULL_UP,   NORMAL,     OUTPUT), /* A_LCD1_BL_PWM */
	DEFAULT_PINMUX(GMI_CS4_N,       NAND,            NORMAL,    NORMAL,     INPUT), /* USB3_WAK */
	DEFAULT_PINMUX(GMI_CS6_N,       NAND_ALT,        PULL_UP,   TRISTATE,   OUTPUT), /* unused */
	DEFAULT_PINMUX(GMI_CS7_N,       NAND_ALT,        PULL_UP,    NORMAL,     INPUT), /* LOW_BAT#_EC SR2 */
    DEFAULT_PINMUX(GMI_WAIT,        NAND,            PULL_UP,   TRISTATE,   OUTPUT), /* unused */
    DEFAULT_PINMUX(GMI_WP_N,        GMI,             PULL_UP,   NORMAL,     INPUT), /* AP_WAKE SR2 */
    DEFAULT_PINMUX(CRT_HSYNC,       CRT,             NORMAL,    NORMAL,     OUTPUT), /* W_DISABLE_N */
	DEFAULT_PINMUX(CRT_VSYNC,       CRT,             PULL_UP,   TRISTATE,   OUTPUT), /* unused */
    DEFAULT_PINMUX(LCD_CS0_N,       DISPLAYA,        PULL_UP,   TRISTATE,   OUTPUT), /* unused */
    DEFAULT_PINMUX(LCD_D18,         DISPLAYA,        PULL_DOWN, TRISTATE,   OUTPUT), /* unused */
	DEFAULT_PINMUX(LCD_D19,         DISPLAYA,        PULL_DOWN, TRISTATE,   OUTPUT), /* unused */
	DEFAULT_PINMUX(LCD_D20,         DISPLAYA,        PULL_DOWN, TRISTATE,   OUTPUT), /* unused */
	DEFAULT_PINMUX(LCD_D21,         DISPLAYA,        PULL_DOWN, TRISTATE,   OUTPUT), /* unused */
	DEFAULT_PINMUX(LCD_D22,         DISPLAYA,        PULL_DOWN, TRISTATE,   OUTPUT), /* unused */
	DEFAULT_PINMUX(LCD_D23,         DISPLAYA,        PULL_DOWN, TRISTATE,   OUTPUT), /* unused */
    DEFAULT_PINMUX(LCD_DC0,         DISPLAYA,        PULL_UP,   NORMAL,     OUTPUT), /* LVDS1_SHTDN */
    DEFAULT_PINMUX(LCD_DC1,         DISPLAYA,        NORMAL,    NORMAL,     OUTPUT), /* T30_MCU_RST */
    DEFAULT_PINMUX(LCD_SCK,         DISPLAYA,        PULL_UP,   NORMAL,     INPUT), /* SDMMC_WP */
    DEFAULT_PINMUX(LCD_SDIN,        DISPLAYA,        NORMAL,    NORMAL,     INPUT), /* LIGHT_SENSOR_INT */
    DEFAULT_PINMUX(LCD_SDOUT,       DISPLAYA,        PULL_UP,   TRISTATE,   OUTPUT), /* unused */
    DEFAULT_PINMUX(PEX_L2_RST_N,    PCIE,            PULL_UP,   NORMAL,     INPUT),	/* AP_WAKE# */
    DEFAULT_PINMUX(KB_COL2,         KBC,             PULL_UP,   NORMAL,     INPUT), /* SIM_DET# */
	DEFAULT_PINMUX(KB_COL3,         KBC,             PULL_UP,   TRISTATE,     OUTPUT), /* unused */
	DEFAULT_PINMUX(KB_COL4,         KBC,             PULL_UP,   NORMAL,     INPUT), /* R_TX_ON_T30 */
	DEFAULT_PINMUX(KB_COL5,         KBC,             PULL_UP,   NORMAL,     INPUT), /* R_SW_READY_T30 */
    DEFAULT_PINMUX(KB_COL6,         KBC,             PULL_UP,   NORMAL,     INPUT), /* R_HW_READY_T30 */
    DEFAULT_PINMUX(KB_COL7,         KBC,             PULL_UP,   NORMAL,     OUTPUT), /* R_NFC_GPIO4_EE */
    DEFAULT_PINMUX(KB_ROW10,        KBC,             PULL_DOWN, TRISTATE,   OUTPUT), /* unused */
    DEFAULT_PINMUX(KB_ROW11,        KBC,             PULL_UP,    NORMAL,     INPUT), /* LINE_OUT_DET */
    DEFAULT_PINMUX(KB_ROW12,        KBC,             NORMAL,    NORMAL,     INPUT), /* DOCK_DET# */
    DEFAULT_PINMUX(GMI_DQS,         NAND,            PULL_DOWN, TRISTATE,   OUTPUT), /* unused */
    DEFAULT_PINMUX(GMI_CS2_N,       RSVD1,           PULL_UP,   NORMAL,   INPUT), /* CAP_SENSOR_OUT */

    /* Avalon-specific power rails */
    DEFAULT_PINMUX(SPI1_CS0_N,      SPI1,            NORMAL,    NORMAL,     OUTPUT), /* Haptic_3V3_EN */
    DEFAULT_PINMUX(DAP3_FS,         I2S2,            NORMAL,    NORMAL,     OUTPUT), /* 3G_POWER_ON */
    DEFAULT_PINMUX(LCD_PWR2,        DISPLAYA,        NORMAL,    NORMAL,     OUTPUT), /* CAP_3V3_EN */
    DEFAULT_PINMUX(SDMMC3_DAT4,     SDMMC3,          PULL_UP,   NORMAL,     OUTPUT), /* EN_3V3_EMMC */
    DEFAULT_PINMUX(SDMMC3_DAT7,     SDMMC3,          PULL_UP,   NORMAL,     OUTPUT), /* SD_3V3_EN */
    DEFAULT_PINMUX(KB_ROW9,         KBC,             NORMAL,    NORMAL,     OUTPUT), /* SEN_1V8_EN */
    DEFAULT_PINMUX(GMI_AD9,         NAND,            NORMAL,   NORMAL,     OUTPUT), /* TS_EN */
};

/* pinmux: please add sphinx-specific pinmux here */
static __initdata struct tegra_pingroup_config sphinx_pinmux[] = {
    DEFAULT_PINMUX(DAP1_DOUT,       I2S0,            NORMAL,    NORMAL,     INPUT), /* WW_WAKE */
    DEFAULT_PINMUX(SPI1_CS0_N,      SPI1,            NORMAL,    NORMAL,     INPUT), /* SPI1_CS0_N */
    DEFAULT_PINMUX(SPI1_MISO,       SPI1,            NORMAL,    NORMAL,     INPUT), /* SPI1_MISO */
    DEFAULT_PINMUX(SPI1_MOSI,       SPI1,            NORMAL,    NORMAL,     INPUT), /* SPI1_MOSI */
    DEFAULT_PINMUX(SPI1_SCK,        SPI1,            NORMAL,    NORMAL,     INPUT), /* SPI1_SCK */
    DEFAULT_PINMUX(SPI2_CS0_N,      SPI2,            PULL_UP,   NORMAL,     INPUT), /* LIGHT_SENSOR_INT_N */
    DEFAULT_PINMUX(DAP3_DOUT,       I2S2,            PULL_DOWN, TRISTATE,   OUTPUT), /* unused */
    DEFAULT_PINMUX(ULPI_DATA1,      UARTA,           NORMAL,    NORMAL,     OUTPUT), /* R_ON1 */
    DEFAULT_PINMUX(ULPI_DATA2,      UARTA,           NORMAL,    NORMAL,     OUTPUT), /* OLED_TE */
    DEFAULT_PINMUX(ULPI_DATA5,      UARTA,           NORMAL,    NORMAL,     OUTPUT), /* LCD_PCD */
	DEFAULT_PINMUX(ULPI_DATA6,      UARTA,           PULL_UP,   NORMAL,     INPUT), /* TSP_INT */
    DEFAULT_PINMUX(GMI_A18,         SPI4,            NORMAL,    NORMAL,     INPUT), /* LOW_BAT#_EC */
	DEFAULT_PINMUX(GMI_AD10,        NAND,            PULL_DOWN, TRISTATE,   OUTPUT), /* unused */
    DEFAULT_PINMUX(GMI_AD12,        NAND,            PULL_DOWN, TRISTATE,   OUTPUT), /* unused */
    DEFAULT_PINMUX(GMI_AD13,        NAND,            PULL_DOWN, TRISTATE,   OUTPUT), /* unused */
	DEFAULT_PINMUX(GMI_AD14,        NAND,            PULL_DOWN, TRISTATE,   OUTPUT), /* unused */
	DEFAULT_PINMUX(GMI_AD8,         NAND,            PULL_DOWN, TRISTATE,   OUTPUT), /* unused */
	DEFAULT_PINMUX(GMI_CS4_N,       NAND,            NORMAL,    NORMAL,     INPUT), /* USB_WAKE */
	DEFAULT_PINMUX(GMI_CS6_N,       NAND_ALT,        PULL_UP,   TRISTATE,   OUTPUT), /* unused */
	DEFAULT_PINMUX(GMI_CS7_N,       NAND_ALT,        PULL_UP,    NORMAL,     INPUT), /* LOW_BAT#_EC SR2 */
    DEFAULT_PINMUX(GMI_WAIT,        NAND,            PULL_UP,   TRISTATE,   OUTPUT), /* unused */
    DEFAULT_PINMUX(GMI_WP_N,        GMI,             PULL_UP,   NORMAL,     INPUT), /* AP_WAKE SR2 */
	DEFAULT_PINMUX(CRT_HSYNC,       CRT,             NORMAL,    NORMAL,     OUTPUT), /* W_DISABLE_N */
	DEFAULT_PINMUX(CRT_VSYNC,       CRT,             PULL_UP,   NORMAL,     OUTPUT), /* LCD_SHTDN */
    DEFAULT_PINMUX(LCD_CS0_N,       SPI5,            PULL_UP,   NORMAL,     INPUT), /* LCD_SPI_CS0_N */
	DEFAULT_PINMUX(LCD_D18,         DISPLAYA,        NORMAL,    NORMAL,     OUTPUT), /* LCD_DATA18 */
	DEFAULT_PINMUX(LCD_D19,         DISPLAYA,        NORMAL,    NORMAL,     OUTPUT), /* LCD_DATA19 */
	DEFAULT_PINMUX(LCD_D20,         DISPLAYA,        NORMAL,    NORMAL,     OUTPUT), /* LCD_DATA20 */
	DEFAULT_PINMUX(LCD_D21,         DISPLAYA,        NORMAL,    NORMAL,     OUTPUT), /* LCD_DATA21 */
	DEFAULT_PINMUX(LCD_D22,         DISPLAYA,        NORMAL,    NORMAL,     OUTPUT), /* LCD_DATA22 */
	DEFAULT_PINMUX(LCD_D23,         DISPLAYA,        NORMAL,    NORMAL,     OUTPUT), /* LCD_DATA23 */
    DEFAULT_PINMUX(LCD_DC1,         DISPLAYA,        NORMAL,    NORMAL,     OUTPUT), /* R_NFC_VEN_EE */
    DEFAULT_PINMUX(LCD_SCK,         SPI5,            PULL_UP,   NORMAL,     INPUT), /* LCD_SPI_SCK */
    DEFAULT_PINMUX(LCD_SDIN,        SPI5,            PULL_UP,   NORMAL,     INPUT), /* LCD_SPI_SDOUT */
    DEFAULT_PINMUX(LCD_SDOUT,       SPI5,            PULL_UP,   NORMAL,     INPUT), /* LCD_SPI_SDIN */
    DEFAULT_PINMUX(PEX_L2_RST_N,    PCIE,            PULL_UP,   NORMAL,     INPUT),	/* AP_WAKE# */
	DEFAULT_PINMUX(KB_COL2,         KBC,             PULL_UP,   NORMAL,     INPUT), /* SIM_DET# */
	DEFAULT_PINMUX(KB_COL3,         KBC,             PULL_UP,   NORMAL,     INPUT), /* CAP_SENSOR_OUT */
	DEFAULT_PINMUX(KB_COL4,         KBC,             PULL_UP,   NORMAL,     INPUT), /* R_TX_ON_T30 */
	DEFAULT_PINMUX(KB_COL5,         KBC,             PULL_UP,   NORMAL,     INPUT), /* R_SW_READY_T30 */
    DEFAULT_PINMUX(KB_COL6,         KBC,             PULL_UP,   NORMAL,     INPUT), /* R_HW_READY_T30 */
    DEFAULT_PINMUX(KB_COL7,         KBC,             PULL_UP,   TRISTATE,   OUTPUT), /* unused */
    DEFAULT_PINMUX(KB_ROW9,         KBC,             PULL_DOWN, TRISTATE,   OUTPUT), /* unused */
    DEFAULT_PINMUX(KB_ROW10,        KBC,             NORMAL,    NORMAL,     OUTPUT), /* R_NFC_GPIO_EE */
    DEFAULT_PINMUX(KB_ROW11,        KBC,             PULL_UP,    NORMAL,     INPUT), /* LINE_OUT_DET */
    DEFAULT_PINMUX(KB_ROW12,        KBC,             NORMAL,    NORMAL,     INPUT), /* DOCK_DET# */
    DEFAULT_PINMUX(GMI_DQS,         NAND,            PULL_DOWN, TRISTATE,   OUTPUT), /* unused */
    DEFAULT_PINMUX(GMI_CS2_N,       RSVD1,           PULL_UP,   TRISTATE,   OUTPUT), /* unused */

    /* Sphinx-specific power rails */
    DEFAULT_PINMUX(SPDIF_IN,        SPDIF,           NORMAL,    NORMAL,     OUTPUT), /* Haptic_3V3_EN */
    DEFAULT_PINMUX(SPI2_SCK,        GMI,             PULL_UP,   NORMAL,     OUTPUT), /* EN_VDD1V8_MIPI */
    DEFAULT_PINMUX(DAP3_FS,         I2S2,            NORMAL,    NORMAL,     OUTPUT), /* EN_3V3_MODEM */
    DEFAULT_PINMUX(GMI_AD9,         NAND,            NORMAL,    NORMAL,     OUTPUT), /* EN_5V8_BL */
    DEFAULT_PINMUX(LCD_DC0,         DISPLAYA,        PULL_UP,   NORMAL,     OUTPUT), /* CAP_3V3_EN */
    DEFAULT_PINMUX(LCD_PWR2,        DISPLAYA,        PULL_DOWN, NORMAL,     OUTPUT), /* EN_VDD1V8_PNL1 */
    DEFAULT_PINMUX(SDMMC3_DAT4,     SDMMC3,          PULL_UP,   NORMAL,     OUTPUT), /* SEN_1V8_EN */
    DEFAULT_PINMUX(SDMMC3_DAT7,     SDMMC3,          PULL_UP,   NORMAL,     OUTPUT), /* AUD_1V8_EN */
};

/* pinmux: please add titan-specific pinmux here */
static __initdata struct tegra_pingroup_config titan_pinmux[] = {
    DEFAULT_PINMUX(DAP1_DOUT,       I2S0,            PULL_DOWN, TRISTATE,   OUTPUT), /* unused */
    DEFAULT_PINMUX(SPDIF_IN,        SPDIF,           PULL_UP,   TRISTATE,   OUTPUT), /* unused */
    DEFAULT_PINMUX(SPI1_CS0_N,      SPI1,            PULL_UP,   TRISTATE,   OUTPUT), /* unused */
    DEFAULT_PINMUX(SPI1_MISO,       SPI1,            NORMAL,    NORMAL,     OUTPUT), /* R_NFC_VEN_EE */
    DEFAULT_PINMUX(SPI1_MOSI,       SPI1,            PULL_UP,   TRISTATE,   OUTPUT), /* unused */
    DEFAULT_PINMUX(SPI1_SCK,        SPI1,            PULL_UP,   TRISTATE,   OUTPUT), /* unused */
    DEFAULT_PINMUX(SPI2_CS0_N,      SPI2,            PULL_UP,   TRISTATE,   OUTPUT), /* unused */
    DEFAULT_PINMUX(SPI2_SCK,        GMI,             PULL_UP,   TRISTATE,   OUTPUT), /* unused */
    DEFAULT_PINMUX(DAP3_DOUT,       I2S2,            PULL_UP,   NORMAL,     OUTPUT), /* EN_VDDIO_VID_OC */
    DEFAULT_PINMUX(ULPI_DATA5,      UARTA,           PULL_UP,   TRISTATE,   OUTPUT), /* unused */
	DEFAULT_PINMUX(ULPI_DATA6,      UARTA,           PULL_UP,   TRISTATE,   OUTPUT), /* unused */
    DEFAULT_PINMUX(GMI_A18,         SPI4,            PULL_DOWN, TRISTATE,   OUTPUT), /* unused */
	DEFAULT_PINMUX(GMI_AD10,        NAND,            PULL_UP,   NORMAL,     OUTPUT), /* LCD1_BL_EN */
    DEFAULT_PINMUX(GMI_AD12,        NAND,            NORMAL,    NORMAL,     INPUT), /* TS_IRQ */
    DEFAULT_PINMUX(GMI_AD13,        NAND,            PULL_DOWN, TRISTATE,   OUTPUT), /* unused */
	DEFAULT_PINMUX(GMI_AD14,        NAND,            NORMAL,    NORMAL,     OUTPUT), /* TS_RESET */
	DEFAULT_PINMUX(GMI_AD8,         PWM0,            PULL_UP,   NORMAL,     OUTPUT), /* A_LCD_BL_PWM */
	DEFAULT_PINMUX(GMI_CS4_N,       NAND,            PULL_DOWN, NORMAL,     OUTPUT), /* ISDB-T_RESET_N */
	DEFAULT_PINMUX(GMI_CS6_N,       NAND_ALT,        PULL_DOWN, NORMAL,     OUTPUT), /* unused */
	DEFAULT_PINMUX(GMI_CS7_N,       NAND_ALT,        PULL_UP,    NORMAL,     INPUT), /* LOW_BAT#_EC SR2 */
    DEFAULT_PINMUX(GMI_WAIT,        NAND,            NORMAL,    NORMAL,     INPUT), /* TS_WAKE */
    DEFAULT_PINMUX(GMI_WP_N,        GMI,             PULL_UP,   NORMAL,     INPUT), /* AP_WAKE SR2 */
	DEFAULT_PINMUX(CRT_HSYNC,       CRT,             PULL_UP,   TRISTATE,   OUTPUT), /* unused */
	DEFAULT_PINMUX(CRT_VSYNC,       CRT,             PULL_UP,   TRISTATE,   OUTPUT), /* unused */
    DEFAULT_PINMUX(LCD_CS0_N,       DISPLAYA,        PULL_UP,   TRISTATE,   OUTPUT), /* unused */
	DEFAULT_PINMUX(LCD_D18,         DISPLAYA,        PULL_DOWN, TRISTATE,   OUTPUT), /* unused */
	DEFAULT_PINMUX(LCD_D19,         DISPLAYA,        PULL_DOWN, TRISTATE,   OUTPUT), /* unused */
	DEFAULT_PINMUX(LCD_D20,         DISPLAYA,        PULL_DOWN, TRISTATE,   OUTPUT), /* unused */
	DEFAULT_PINMUX(LCD_D21,         DISPLAYA,        PULL_DOWN, TRISTATE,   OUTPUT), /* unused */
	DEFAULT_PINMUX(LCD_D22,         DISPLAYA,        PULL_DOWN, TRISTATE,   OUTPUT), /* unused */
	DEFAULT_PINMUX(LCD_D23,         DISPLAYA,        PULL_DOWN, TRISTATE,   OUTPUT), /* unused */
    DEFAULT_PINMUX(LCD_DC0,         DISPLAYA,        PULL_UP,   NORMAL,     OUTPUT), /* LVDS1_SHTDN */
    DEFAULT_PINMUX(LCD_DC1,         DISPLAYA,        NORMAL,    NORMAL,     OUTPUT), /* T30_MCU_RST */
    DEFAULT_PINMUX(LCD_PWR2,        DISPLAYA,        PULL_DOWN, TRISTATE,   OUTPUT), /* unused */
    DEFAULT_PINMUX(LCD_SCK,         DISPLAYA,        PULL_UP,   NORMAL,     INPUT), /* SDMMC_WP */
    DEFAULT_PINMUX(LCD_SDIN,        DISPLAYA,        NORMAL,    NORMAL,     INPUT), /* LIGHT_SENSOR_INT_N */
    DEFAULT_PINMUX(LCD_SDOUT,       DISPLAYA,        PULL_UP,   TRISTATE,   OUTPUT), /* unused */
    DEFAULT_PINMUX(PEX_L2_RST_N,    PCIE,            PULL_DOWN, TRISTATE,   OUTPUT), /* unused */
	DEFAULT_PINMUX(KB_COL2,         KBC,             PULL_UP,   TRISTATE,   OUTPUT), /* unused */
	DEFAULT_PINMUX(KB_COL3,         KBC,             PULL_UP,   TRISTATE,   OUTPUT), /* unused */
	DEFAULT_PINMUX(KB_COL4,         KBC,             PULL_UP,   TRISTATE,   OUTPUT), /* unused */
	DEFAULT_PINMUX(KB_COL5,         KBC,             PULL_UP,   TRISTATE,   OUTPUT), /* unused */
    DEFAULT_PINMUX(KB_COL6,         KBC,             PULL_UP,   TRISTATE,   OUTPUT), /* unused */
    DEFAULT_PINMUX(KB_COL7,         KBC,             PULL_UP,   NORMAL,     OUTPUT), /* R_NFC_GPIO4_EE */
    DEFAULT_PINMUX(KB_ROW10,        KBC,             PULL_DOWN, TRISTATE,   OUTPUT), /* unused */
    DEFAULT_PINMUX(KB_ROW11,        KBC,             PULL_DOWN, TRISTATE,   OUTPUT), /* unused */
    DEFAULT_PINMUX(KB_ROW12,        KBC,             PULL_DOWN, TRISTATE,   OUTPUT), /* unused */
    DEFAULT_PINMUX(GMI_DQS,         NAND,            NORMAL,    NORMAL,     INPUT), /* DEBUG_MODE_SEL */
    DEFAULT_PINMUX(GMI_CS2_N,       RSVD1,           PULL_UP,   TRISTATE,   OUTPUT), /* unused */

    /* Titan-specific Power rails */
    DEFAULT_PINMUX(DAP3_FS,         I2S2,            PULL_DOWN,    NORMAL,     OUTPUT), /* ISDB-T_1V8_PWR_EN */
    DEFAULT_PINMUX(ULPI_DATA1,      UARTA,           PULL_DOWN,    NORMAL,     OUTPUT), /* ISDB-T_3V3_PWR_EN */
    DEFAULT_PINMUX(SDMMC3_DAT4,     SDMMC3,          PULL_UP,   NORMAL,     OUTPUT), /* EN_3V3_EMMC */
    DEFAULT_PINMUX(SDMMC3_DAT7,     SDMMC3,          PULL_UP,   NORMAL,     OUTPUT), /* SD_3V3_EN */
    DEFAULT_PINMUX(KB_ROW9,         KBC,             NORMAL,    NORMAL,     OUTPUT), /* SEN_1V8_EN */
    DEFAULT_PINMUX(GMI_AD9,         NAND,            NORMAL,    NORMAL,     OUTPUT), /* TS_EN */
};

static void __init ast_pinmux_audio_init(void)
{
	tegra_gpio_enable(TEGRA_GPIO_HP_DET);
    tegra_gpio_enable(TEGRA_GPIO_AMIC_DET);
    tegra_gpio_enable(TEGRA_GPIO_LINEOUT_DET);
}

#define GPIO_INIT_PIN_MODE(_gpio, _is_input, _value)	\
	{					\
		.gpio_nr	= _gpio,	\
		.is_input	= _is_input,	\
		.value		= _value,	\
	}


/* E1198-A02/E1291 specific  fab >= A03 */
static struct gpio_init_pin_info init_gpio_mode_e1291_a03[] = {
	GPIO_INIT_PIN_MODE(TEGRA_GPIO_PDD6, false, 0),
	GPIO_INIT_PIN_MODE(TEGRA_GPIO_PDD4, false, 0),
};

static void __init ast_gpio_init_configure(void)
{
	int len;
	int i;
	struct gpio_init_pin_info *pins_info;

    len = ARRAY_SIZE(init_gpio_mode_e1291_a03);
    pins_info = init_gpio_mode_e1291_a03;

	for (i = 0; i < len; ++i) {
		tegra_gpio_init_configure(pins_info->gpio_nr,
			pins_info->is_input, pins_info->value);
		pins_info++;
	}
}

int __init ast_pinmux_init(void)
{
	ast_gpio_init_configure();

	tegra_pinmux_config_table(ast_pinmux_common, ARRAY_SIZE(ast_pinmux_common));
	tegra_drive_pinmux_config_table(ast_drive_pinmux,
					ARRAY_SIZE(ast_drive_pinmux));

    if (machine_is_avalon()) {
        tegra_pinmux_config_table(avalon_pinmux,
                                  ARRAY_SIZE(avalon_pinmux));
    } else if (machine_is_sphinx()) {
        tegra_pinmux_config_table(sphinx_pinmux,
                                  ARRAY_SIZE(sphinx_pinmux));
    } else if (machine_is_titan()) {
        tegra_pinmux_config_table(titan_pinmux,
                                  ARRAY_SIZE(titan_pinmux));
    }

	ast_pinmux_audio_init();

	return 0;
}

#define PIN_GPIO_LPM(_name, _gpio, _is_input, _value)	\
	{					\
		.name		= _name,	\
		.gpio_nr	= _gpio,	\
		.is_gpio	= true,		\
		.is_input	= _is_input,	\
		.value		= _value,	\
	}

/* unused pin:  add avalon-specific unused gpio pin here*/
struct gpio_init_pin_info pin_lpm_avalon[] = {

};

/* unused pin: add sphinx-specific unused gpio pin here */
struct gpio_init_pin_info pin_lpm_sphinx[] = {

};

/* unused pin: add titan-specific unused gpio pin here */
struct gpio_init_pin_info pin_lpm_titan[] = {
    PIN_GPIO_LPM("uu KB_ROW11", TEGRA_GPIO_PS3, 0, 1),
};

static void set_unused_pin_gpio(struct gpio_init_pin_info *lpm_pin_info,
		int list_count)
{
	int i;
	struct gpio_init_pin_info *pin_info;
	int ret;

	for (i = 0; i < list_count; ++i) {
		pin_info = (struct gpio_init_pin_info *)(lpm_pin_info + i);
		if (!pin_info->is_gpio)
			continue;

		ret = gpio_request(pin_info->gpio_nr, pin_info->name);
		if (ret < 0) {
			pr_err("%s() Error in gpio_request() for gpio %d\n",
					__func__, pin_info->gpio_nr);
			continue;
		}
		if (pin_info->is_input)
			ret = gpio_direction_input(pin_info->gpio_nr);
		else
			ret = gpio_direction_output(pin_info->gpio_nr,
							pin_info->value);
		if (ret < 0) {
			pr_err("%s() Error in setting gpio %d to in/out\n",
				__func__, pin_info->gpio_nr);
			gpio_free(pin_info->gpio_nr);
			continue;
		}
		tegra_gpio_enable(pin_info->gpio_nr);
	}
}

/* Initialize the pins to desired state as per power/asic/system-eng
 * recomendation */
int __init ast_pins_state_init(void)
{
    if (machine_is_avalon()) {
        set_unused_pin_gpio(&pin_lpm_avalon[0],
                            ARRAY_SIZE(pin_lpm_avalon));
    } else if (machine_is_sphinx()) {
        set_unused_pin_gpio(&pin_lpm_sphinx[0],
                            ARRAY_SIZE(pin_lpm_sphinx));
    } else if (machine_is_titan()) {
        set_unused_pin_gpio(&pin_lpm_titan[0],
                            ARRAY_SIZE(pin_lpm_titan));
    }

	return 0;
}
