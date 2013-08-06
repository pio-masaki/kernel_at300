/*
 * arch/arm/mach-tegra/board-sphinx-panel.c
 *
 * Copyright (c) 2010-2012, NVIDIA Corporation.
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

#include <linux/delay.h>
#include <linux/ion.h>
#include <linux/tegra_ion.h>
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/resource.h>
#include <asm/mach-types.h>
#include <linux/debugfs.h>
#include <linux/platform_device.h>
#include <linux/earlysuspend.h>
#include <linux/charging-img.h>
#include <linux/backlight.h>
#include <linux/spi/spi.h>
#include <linux/workqueue.h>
#include <asm/atomic.h>
#include <linux/nvhost.h>
#include <linux/nvmap.h>
#include <mach/irqs.h>
#include <mach/iomap.h>
#include <mach/dc.h>
#include <mach/fb.h>
#include <mach/smmu.h>

#include "board.h"
#include "board-ast.h"
#include "devices.h"
#include "gpio-names.h"
#include "tegra3_host1x_devices.h"

//#define BRIGHTNESS_MAX 255
//#define DSI_PANEL_RESET 0

#define ENABLE_DEBUG_PRINTS	0

#if ENABLE_DEBUG_PRINTS
#define PNL_PRINT(x) pr_info x
#else
#define PNL_PRINT(x)
#endif

#define MIN_BRIGHTNESS		(0)
#define MAX_BRIGHTNESS		(255)

#define ast_lcd_shutdown	TEGRA_GPIO_PV7
#define ast_lcd_reset	        TEGRA_GPIO_PZ3

#define ast_lcd_spiclk   	TEGRA_GPIO_PZ4
#define ast_lcd_cs0n	        TEGRA_GPIO_PN4
#define ast_lcd_spisdin   	TEGRA_GPIO_PZ2
#define ast_lcd_spisdout	TEGRA_GPIO_PN5
#define ELVSS_2H_OFFSET         0x80

static u8 gpio_config_tbl[] = {
  ast_lcd_shutdown,ast_lcd_reset,ast_lcd_spiclk,
  ast_lcd_cs0n, ast_lcd_spisdin, ast_lcd_spisdout
};

#define ESDON_MIPICLOCK_WORKAROUND
#ifdef ESDON_MIPICLOCK_WORKAROUND
static void mytimer_work(struct work_struct *work);
static DECLARE_DELAYED_WORK(mytimer_workqueue, mytimer_work);
static int display_off_cnt = 0;
#endif

static struct mutex lcd_lock;
static struct regulator *ast_en_vdd_bl   = NULL;
static struct regulator *ast_en_1v8_pnl  = NULL;
static struct regulator *ast_en_vdd_pnl  = NULL;
static struct regulator *ast_en_5v8_bl   = NULL;
static struct regulator *ast_en_1v8_mipi = NULL;

static struct spi_device *w1_disp1_spi = NULL;
static struct regulator *ast_hdmi_reg = NULL;
static struct regulator *ast_hdmi_pll = NULL;
//static struct regulator *ast_hdmi_vddio = NULL;
static int panel_on = 0;
static int is_panel_init_on = 0;
static int panel_id3 = 0;
static int prev_brightness = 0;

#ifdef CONFIG_DEBUG_FS
static struct dentry *debugfs_spi_panel_root = NULL;
static struct dentry *debugfs_spi_bridge_root = NULL;
static struct dentry *debugfs_spi_brightness_root = NULL;
static int spi_panel_addr_r = 0;
static int spi_bridge_index = 0xB0;
#endif

static atomic_t sd_brightness = ATOMIC_INIT(MAX_BRIGHTNESS);

#define NV_ARRAY_SIZE(x) (sizeof(x) / sizeof((x)[0]))
static int ast_spi_write(int Data);
static int ast_spi_read(int Data, int *pRxBuffer);

#define SEND_SSD2825_SEQ_WS(p,s) \
do { \
    int i; \
    for (i = 0; i < s; i++) { \
        if (p[i] < 0) { \
            mdelay(-1 * p[i]); \
        } else if (p[i]){ \
            ast_spi_write(p[i]); \
        } else \
            break; \
    } \
} while(0)

#define SEND_SSD2825_SEQ(p) \
    SEND_SSD2825_SEQ_WS(p,NV_ARRAY_SIZE(p))

/*
 * Solomaon MIPI register for SSD2825
 */
typedef enum
{
    SSD_DIR    = 0xB0,
    SSD_VICR1, SSD_VICR2, SSD_VICR3, SSD_VICR4, SSD_VICR5, SSD_VICR6, SSD_CFGR,
    SSD_VCR,   SSD_PCR,   SSD_PLCR,  SSD_CCR,   SSD_PSCR1, SSD_PSCR2, SSD_PSCR3, SSD_GPDR,

    SSD_OCR    = 0xC0,
    SSD_MRSR,  SSD_RDCR,  SSD_ARSR,  SSD_LCR,   SSD_ICR,   SSD_ISR,   SSD_ESR,
    SSD_DFR,   SSD_DAR1,  SSD_DAR2,  SSD_DAR3,  SSD_DAR4,  SSD_DAR5,  SSD_DAR6,  SSD_HTTR1,

    SSD_HTTR2  = 0xD0,
    SSD_LRTR1, SSD_LRTR2, SSD_TSR,   SSD_LRR,   SSD_PLLR,  SSD_TR,    SSD_TRCR,
    SSD_ACR1,  SSD_ACR2,  SSD_ACR3,  SSD_ACR4,  SSD_IOCR,  SSD_VICR7, SSD_LCFR,  SSD_DAR7,

    SSD_IPCR1  = 0xE0,
    SSD_IPCR2, SSD_BICR1, SSD_BICR2, SSD_BICR3, SSD_BICR4, SSD_BICR5, SSD_BICR6,
    SSD_BICR7, SSD_CBCR1, SSD_CBCR2, SSD_CBSR,

    SSD_RR     = 0xFF

} SSDRegIndex;

#define _CMDCYCL	0x700000
#define _DATCYCL	0x720000
#define _RDCYCL		0x730000
#define _SEND(reg)	(_CMDCYCL + (SSD_##reg))
#define _SET(dat)	(_DATCYCL + (dat))
#define _DCS(cmd)	(_CMDCYCL + (cmd))
#define _CKBTA		0xEEEEEE

#define SET_GAMMA	1
#define DYNAMIC_ELVSS	1
#define SET_NVM		1
#define SMART_D_ELVSS	0
#define OPTIMUM_ELVSS	1
#define ACLCTRL         1

#define SINTVL		1		// A small interval for delay
#define MINTVL		5		// A medium interval for delay
#define LINTVL		10		// A long  interval for delay

#define _HFP		39 //128
#define _VFP		13
#define _VICR3		((_VFP << 8) + _HFP)

#define MAX_GAMMA_LEVEL		22
#define BRIGHT_GAMMA		_SET(0x0001)
#define DARK_GAMMA		_SET(0x0002)

#define CFGR_S_RD_DCS_VDS_LP	_SET(0x03C2)
#define CFGR_S_WR_DCS_VDS_LP	_SET(0x0342)
#define CFGR_S_WR_DCS_VEN_HS	_SET(0x034B)
#define CFGR_S_WR_DCS_VEN_LP	_SET(0x034A)

#if ACLCTRL //ACL ON Brightness level = 19
static tegra_dc_bl_output ast_bl_output_measured = {
         3,  3,  3,  3,  3,  3,  3,  3,  3,  3,  //10
         3,  3,  3,  3,  3,  3,  3,  3,  3,  3,  //20

         3,  3,  3,  3,  3,  3,  3,  3,  3,  3,  //30
         3,  3,  3,  4,  4,  4,  4,  4,  4,  4,  //40
         4,  4,  4,  4,  4,  4,  5,  5,  5,  5,  //50
         5,  5,  5,  5,  5,  5,  5,  5,  5,  6,  //60
         6,  6,  6,  6,  6,  6,  6,  6,  6,  6,  //70
         6,  6,  7,  7,  7,  7,  7,  7,  7,  7,  //80
         7,  7,  7,  7,  7,  8,  8,  8,  8,  8,  //90
         8,  8,  8,  8,  8,  8,  8,  8,  9,  9,  //100
         9,  9,  9,  9,  9,  9,  9,  9,  9,  9,  //110
         9, 10, 10, 10, 10, 10, 10, 10, 10, 10,  //120
        10, 10, 10, 10, 11, 11, 11, 11, 11, 11,  //130
        11, 11, 11, 11, 11, 11, 12, 12, 12, 12,  //140
        12, 12, 12, 12, 12, 12, 12, 12, 13, 13,  //150
        13, 13, 13, 13, 13, 13, 13, 13, 13, 13,  //160
        14, 14, 14, 14, 14, 14, 14, 14, 14, 14,  //170
        14, 14, 15, 15, 15, 15, 15, 15, 15, 15,  //180
        15, 15, 15, 15, 16, 16, 16, 16, 16, 16,  //190
        16, 16, 16, 16, 16, 16, 17, 17, 17, 17,  //200
        17, 17, 17, 17, 17, 17, 17, 17, 18, 18,  //210
        18, 18, 18, 18, 18, 18, 18, 18, 18, 18,  //220
        19, 19, 19, 19, 19, 19, 19, 19, 19, 19,  //230
        19, 19, 20, 20, 20, 20, 20, 20, 20, 20,  //240
        20, 20, 20, 20, 21, 21, 21, 21, 21, 21,  //250
        21, 21, 21, 21, 21, 21
	};
#else 	//ACL OFF Brightness level = 22
static tegra_dc_bl_output ast_bl_output_measured = {
         0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  //10
         0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  //20

         0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  //30
         1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  //40
         2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  //50
         3,  3,  3,  3,  3,  3,  3,  3,  3,  3,  //60
         4,  4,  4,  4,  4,  4,  4,  4,  4,  4,  //70
         5,  5,  5,  5,  5,  5,  5,  5,  5,  5,  //80
         6,  6,  6,  6,  6,  6,  6,  6,  6,  6,  //90
         6,  7,  7,  7,  7,  7,  7,  7,  7,  7,  //100
         7,  7,  8,  8,  8,  8,  8,  8,  8,  8,  //110
         8,  8,  8,  9,  9,  9,  9,  9,  9,  9,  //120
         9,  9,  9,  9, 10, 10, 10, 10, 10, 10,  //130
        10, 10, 10, 10, 10, 11, 11, 11, 11, 11,  //140
        11, 11, 11, 11, 11, 11, 12, 12, 12, 12,  //150
        12, 12, 12, 12, 12, 12, 12, 13, 13, 13,  //160
        13, 13, 13, 13, 13, 13, 13, 13, 14, 14,  //170
        14, 14, 14, 14, 14, 14, 14, 14, 14, 15,  //180
        15, 15, 15, 15, 15, 15, 15, 15, 15, 15,  //190
        16, 16, 16, 16, 16, 16, 16, 16, 16, 16,  //200
        16, 17, 17, 17, 17, 17, 17, 17, 17, 17,  //210
        17, 17, 18, 18, 18, 18, 18, 18, 18, 18,  //220
        18, 18, 18, 19, 19, 19, 19, 19, 19, 19,  //230
        19, 19, 19, 19, 20, 20, 20, 20, 20, 20,  //240
        20, 20, 20, 20, 20, 21, 21, 21, 21, 21,  //250
        21, 21, 21, 21, 21, 21
	};
#endif
static int ast_gamma_40[] = {
    _SEND(PSCR1), _SET(25),             // Transmit Data Count = 25 bytes
    _SEND(GPDR),                        // <<[3] Gamma Condition Set -- Gamma Setting>>
    _SET(0x1FFA), _SET(0x431F), _SET(0x8181), _SET(0xE592), _SET(0xCBD5), _SET(0xE4E8), _SET(0xCFD4),
    _SET(0xBDD7), _SET(0xD5D5), _SET(0x00D5), _SET(0x006B), _SET(0x0074), _SET(0x0082),
    -(LINTVL),
};

//0
static int ast_gamma_50[] = {
    _SEND(PSCR1), _SET(25),		// Transmit Data Count = 25 bytes
    _SEND(GPDR),			// <<[3] Gamma Condition Set -- Gamma Setting>>
    _SET(0x1FFA), _SET(0x431F), _SET(0x9C9D), _SET(0xE1AC), _SET(0xC5D1), _SET(0xE8E8), _SET(0xCDD9),
    _SET(0xBBD2), _SET(0xD3D4), _SET(0x00D4), _SET(0x0071), _SET(0x007B), _SET(0x0089),
    -(LINTVL),
};
//1
static int ast_gamma_60[] = {
    _SEND(PSCR1), _SET(25),		// Transmit Data Count = 25 bytes
    _SEND(GPDR),			// <<[3] Gamma Condition Set -- Gamma Setting>>
    _SET(0x1FFA), _SET(0x431F), _SET(0x9799), _SET(0xE4A4), _SET(0xC9D3), _SET(0xE8E6), _SET(0xCCD9),
    _SET(0xBACF), _SET(0xD1D1), _SET(0x00D2), _SET(0x0077), _SET(0x0081), _SET(0x008E),
    -(LINTVL),
};
//2
static int ast_gamma_70[] = {
    _SEND(PSCR1), _SET(25),		// Transmit Data Count = 25 bytes
    _SEND(GPDR),			// <<[3] Gamma Condition Set -- Gamma Setting>>
    _SET(0x1FFA), _SET(0x431F), _SET(0xABAE), _SET(0xE0B6), _SET(0xC5D0), _SET(0xEDE7), _SET(0xCBDE),
    _SET(0xB9CD), _SET(0xCFD0), _SET(0x00D0), _SET(0x007D), _SET(0x0087), _SET(0x0093),
    -(LINTVL),
};
static int ast_gamma_80[] = {
    _SEND(PSCR1), _SET(25),             // Transmit Data Count = 25 bytes
    _SEND(GPDR),                        // <<[3] Gamma Condition Set -- Gamma Setting>>
    _SET(0x1FFA), _SET(0x431F), _SET(0xA9AB), _SET(0xDFB2), _SET(0xC4CF), _SET(0xEEE6), _SET(0xCAE0),
    _SET(0xB9CA), _SET(0xCDCE), _SET(0x00CF), _SET(0x0082), _SET(0x008C), _SET(0x0098),
    -(LINTVL),
};

static int ast_gamma_90[] = {
    _SEND(PSCR1), _SET(25),             // Transmit Data Count = 25 bytes
    _SEND(GPDR),                        // <<[3] Gamma Condition Set -- Gamma Setting>>
    _SET(0x1FFA), _SET(0x431F), _SET(0xBCC0), _SET(0xDFC3), _SET(0xC5D0), _SET(0xEDE5), _SET(0xC9DF),
    _SET(0xB8C8), _SET(0xCCCD), _SET(0x00CD), _SET(0x0087), _SET(0x0091), _SET(0x009C),
    -(LINTVL),
};

static int ast_gamma_100[] = {
    _SEND(PSCR1), _SET(25),             // Transmit Data Count = 25 bytes
    _SEND(GPDR),                        // <<[3] Gamma Condition Set -- Gamma Setting>>
    _SET(0x1FFA), _SET(0x431F), _SET(0xB7BB), _SET(0xE3BC), _SET(0xCED6), _SET(0xEBE2), _SET(0xC9DB),
    _SET(0xB8C6), _SET(0xCACB), _SET(0x00CC), _SET(0x008B), _SET(0x0096), _SET(0x00A1),
    -(LINTVL),
};

static int ast_gamma_110[] = {
    _SEND(PSCR1), _SET(25),             // Transmit Data Count = 25 bytes
    _SEND(GPDR),                        // <<[3] Gamma Condition Set -- Gamma Setting>>
    _SET(0x1FFA), _SET(0x431F), _SET(0xC9CF), _SET(0xE2CC), _SET(0xCFD7), _SET(0xEBE2), _SET(0xC8D7),
    _SET(0xB7C4), _SET(0xC9CA), _SET(0x00CA), _SET(0x008F), _SET(0x009A), _SET(0x00A5),
    -(LINTVL),
};

static int ast_gamma_120[] = {
    _SEND(PSCR1), _SET(25),             // Transmit Data Count = 25 bytes
    _SEND(GPDR),                        // <<[3] Gamma Condition Set -- Gamma Setting>>
    _SET(0x1FFA), _SET(0x431F), _SET(0xC7CD), _SET(0xE1C9), _SET(0xCFD8), _SET(0xEAE2), _SET(0xC7DC),
    _SET(0xB6C3), _SET(0xC9CA), _SET(0x00C8), _SET(0x0093), _SET(0x009F), _SET(0x00A9),
    -(LINTVL),
};

static int ast_gamma_130[] = {
    _SEND(PSCR1), _SET(25),             // Transmit Data Count = 25 bytes
    _SEND(GPDR),                        // <<[3] Gamma Condition Set -- Gamma Setting>>
    _SET(0x1FFA), _SET(0x431F), _SET(0xD9E0), _SET(0xE2D8), _SET(0xD2DA), _SET(0xE8E1), _SET(0xC5DB),
    _SET(0xB5C1), _SET(0xC8C9), _SET(0x00C7), _SET(0x0096), _SET(0x00A3), _SET(0x00AD),
    -(LINTVL),
};

static int ast_gamma_140[] = {
    _SEND(PSCR1), _SET(25),             // Transmit Data Count = 25 bytes
    _SEND(GPDR),                        // <<[3] Gamma Condition Set -- Gamma Setting>>
    _SET(0x1FFA), _SET(0x431F), _SET(0xD9E0), _SET(0xDFD8), _SET(0xCED7), _SET(0xE8E2), _SET(0xC4DC),
    _SET(0xB4C0), _SET(0xC7C8), _SET(0x00C5), _SET(0x009A), _SET(0x00A7), _SET(0x00B1),
    -(LINTVL),
};

static int ast_gamma_150[] = {
    _SEND(PSCR1), _SET(25),             // Transmit Data Count = 25 bytes
    _SEND(GPDR),                        // <<[3] Gamma Condition Set -- Gamma Setting>>
    _SET(0x1FFA), _SET(0x431F), _SET(0xD6DE), _SET(0xDFD5), _SET(0xD1D9), _SET(0xE7E1), _SET(0xC3DB),
    _SET(0xB3BF), _SET(0xC5C7), _SET(0x00C2), _SET(0x009D), _SET(0x00AB), _SET(0x00B6),
    -(LINTVL),
};

static int ast_gamma_160[] = {
    _SEND(PSCR1), _SET(25),             // Transmit Data Count = 25 bytes
    _SEND(GPDR),                        // <<[3] Gamma Condition Set -- Gamma Setting>>
    _SET(0x1FFA), _SET(0x431F), _SET(0xDEE6), _SET(0xDEDB), _SET(0xD1DA), _SET(0xE5E1), _SET(0xC3DB),
    _SET(0xB3BF), _SET(0xC4C6), _SET(0x00C1), _SET(0x00A1), _SET(0x00AE), _SET(0x00B9),
    -(LINTVL),
};

static int ast_gamma_170[] = {
    _SEND(PSCR1), _SET(25),             // Transmit Data Count = 25 bytes
    _SEND(GPDR),                        // <<[3] Gamma Condition Set -- Gamma Setting>>
    _SET(0x1FFA), _SET(0x431F), _SET(0xDCE4), _SET(0xDFD8), _SET(0xD3DB), _SET(0xE4E0), _SET(0xC2DA),
    _SET(0xB2BE), _SET(0xC4C6), _SET(0x00C0), _SET(0x00A3), _SET(0x00B2), _SET(0x00BD),
    -(LINTVL),
};

static int ast_gamma_180[] = {
    _SEND(PSCR1), _SET(25),             // Transmit Data Count = 25 bytes
    _SEND(GPDR),                        // <<[3] Gamma Condition Set -- Gamma Setting>>
    _SET(0x1FFA), _SET(0x431F), _SET(0xDAE2), _SET(0xDFD5), _SET(0xD5DC), _SET(0xE4E1), _SET(0xC0DA),
    _SET(0xB0BC), _SET(0xC3C6), _SET(0x00BF), _SET(0x00A7), _SET(0x00B5), _SET(0x00C0),
    -(LINTVL),
};

static int ast_gamma_190[] = {   
    _SEND(PSCR1), _SET(25),             // Transmit Data Count = 25 bytes
    _SEND(GPDR),                        // <<[3] Gamma Condition Set -- Gamma Setting>>
    _SET(0x1FFA), _SET(0x431F), _SET(0xE4ED), _SET(0xDCDE), _SET(0xD1DA), _SET(0xE4E1), _SET(0xC0DB),
    _SET(0xB0BC), _SET(0xC2C4), _SET(0x00BD), _SET(0x00AA), _SET(0x00B9), _SET(0x00C4),
    -(LINTVL),
};

static int ast_gamma_200[] = {
    _SEND(PSCR1), _SET(25),             // Transmit Data Count = 25 bytes
    _SEND(GPDR),                        // <<[3] Gamma Condition Set -- Gamma Setting>>
    _SET(0x1FFA), _SET(0x431F), _SET(0xE2EB), _SET(0xDCDB), _SET(0xD3DC), _SET(0xE3E1), _SET(0xBFDB),
    _SET(0xB0BB), _SET(0xC2C4), _SET(0x00BC), _SET(0x00AC), _SET(0x00BC), _SET(0x00C7),
    -(LINTVL),
};

static int ast_gamma_210[] = {
    _SEND(PSCR1), _SET(25),             // Transmit Data Count = 25 bytes
    _SEND(GPDR),                        // <<[3] Gamma Condition Set -- Gamma Setting>>
    _SET(0x1FFA), _SET(0x431F), _SET(0xDFE9), _SET(0xDDD8), _SET(0xD5DD), _SET(0xE2E0), _SET(0xBFDA),
    _SET(0xB0BB), _SET(0xC1C3), _SET(0x00BB), _SET(0x00AF), _SET(0x00BF), _SET(0x00CA),
    -(LINTVL),
};

static int ast_gamma_220[] = {
   _SEND(PSCR1), _SET(25),              // Transmit Data Count = 25 bytes
    _SEND(GPDR),                        // <<[3] Gamma Condition Set -- Gamma Setting>>
    _SET(0x1FFA), _SET(0x431F), _SET(0xE7F1), _SET(0xDDDE), _SET(0xD7DE), _SET(0xE0DF), _SET(0xBED9),
    _SET(0xAFBA), _SET(0xC1C3), _SET(0x00BA), _SET(0x00B2), _SET(0x00C2), _SET(0x00CE),
    -(LINTVL),
};

static int ast_gamma_230[] = {
    _SEND(PSCR1), _SET(25),             // Transmit Data Count = 25 bytes
    _SEND(GPDR),                        // <<[3] Gamma Condition Set -- Gamma Setting>>
    _SET(0x1FFA), _SET(0x431F), _SET(0xE7F1), _SET(0xDCDE), _SET(0xD5DD), _SET(0xE0E0), _SET(0xBDDA),
    _SET(0xAEB9), _SET(0xC0C3), _SET(0x00B9), _SET(0x00B5), _SET(0x00C5), _SET(0x00D1),
    -(LINTVL),
}; 

static int ast_gamma_240[] = {
    _SEND(PSCR1), _SET(25),             // Transmit Data Count = 25 bytes
    _SEND(GPDR),                        // <<[3] Gamma Condition Set -- Gamma Setting>>
    _SET(0x1FFA), _SET(0x431F), _SET(0xE5EF), _SET(0xDCDB), _SET(0xD7DF), _SET(0xDFDF), _SET(0xBDD9),
    _SET(0xAEB9), _SET(0xBFC2), _SET(0x00B7), _SET(0x00B7), _SET(0x00C8), _SET(0x00D4),
    -(LINTVL),
};

//32
static int ast_gamma_250[] = { //Default Gamma Value
    _SEND(PSCR1), _SET(25),		// Transmit Data Count = 25 bytes
    _SEND(GPDR),			// <<[3] Gamma Condition Set -- Gamma Setting>>
    _SET(0x1FFA), _SET(0x431F), _SET(0xECF7), _SET(0xDBE1), _SET(0xD7DF), _SET(0xDFE0), _SET(0xBCDA),
    _SET(0xADB8), _SET(0xBFC2), _SET(0x00B7), _SET(0x00BA), _SET(0x00CB), _SET(0x00D7),
    -(LINTVL),
};

typedef struct{
  unsigned char nits;
  int *gamma_table;
} ast_gamma_table;

static ast_gamma_table brightness_table[MAX_GAMMA_LEVEL]= {
        {40, ast_gamma_40},    //0
        {50, ast_gamma_50},    //1
        {60, ast_gamma_60},    //2
        {70, ast_gamma_70},    //3
        {80, ast_gamma_80},    //4
        {90, ast_gamma_90},    //5
        {100,ast_gamma_100},   //6
        {110,ast_gamma_110},   //7
        {120,ast_gamma_120},   //8
        {130,ast_gamma_130},   //9
        {140,ast_gamma_140},   //10
        {150,ast_gamma_150},   //11
        {160,ast_gamma_160},   //12
        {170,ast_gamma_170},   //13
        {180,ast_gamma_180},   //14
        {190,ast_gamma_190},   //15
        {200,ast_gamma_200},   //16
        {210,ast_gamma_210},   //17
        {220,ast_gamma_220},   //18
        {230,ast_gamma_230},   //19
        {240,ast_gamma_240},   //20
        {250,ast_gamma_250},   //21
};
static struct platform_device ast_backlight_device = {
	.name = "pwm-backlight",
	.id = -1,
};

#define SSD2825_WritePanel_WS(p) SSD2825_WritePanel(p,NV_ARRAY_SIZE(p))
static void SSD2825_WritePanel(const int *cmdSeq, int cmdCnt)
{
    int i, iGpdr=0, iCnt=0;
    for (i = 0; i < cmdCnt; i++) {
        if (cmdSeq[i] < 0) {
            mdelay(-1 * cmdSeq[i]);
        } else if (cmdSeq[i] == _CKBTA) {
            int j, buffer;
            for (j = 0; j < 10; j++) {
                ast_spi_write(_SEND(ISR));
                ast_spi_read(_RDCYCL, &buffer);
                if ((buffer&0x1C) == 0x1C) {      // Check BTAR == 1, ARR == 1, ATR == 1
                    break;
                }
                mdelay(1);
            }
            if ((j >= 10) && (++iCnt <= 5)) {     // Acknowledge Error on BTA? then, Has chance to retry?
                printk(KERN_INFO "%s: ACK Op, CMD(%x)=%x\n", __func__, cmdSeq[iGpdr+1], buffer);
                ast_spi_write(cmdSeq[i = iGpdr]);
            }
        } else {
            if (cmdSeq[i] == _SEND(GPDR)) {
                iGpdr = i;
                iCnt = 0;
            }
            ast_spi_write(cmdSeq[i]);
        }
    }
}

static void ssd2825_init_pll(int plcr, int ccr)
{
    int data_to_send[] =
    {
        _SEND(PLCR),  plcr,			// Configure PLL: fOUT = (fIN)/(MS)*(NF)
        _SEND(CCR),   ccr,			// LP mode Clock Divider
        _SEND(PCR),   _SET(0x0001),		// PLL enable
        -(LINTVL),
        _SEND(VCR),   _SET(0x0000),		// Virtual Channel Control
        _SEND(LCR),   _SET(0x0001),		// Force BTA After Write
        _SEND(MRSR),  _SET(2),			// Maximum Return Size >= 2
    };

    SEND_SSD2825_SEQ(data_to_send);
}

//First Sending LSB and than MSB. It is different from booloader
static void ssd2825_init_seq(void)
{
    static int valPLCR = 0, valCCR;
    static const int data_to_send[] =
    {
        _SEND(VICR1), _SET(0x0240),		// _SET(0x0202),
        _SEND(VICR2), _SET(0x05C0),		// _SET(0x0580),
        _SEND(VICR3), _SET(_VICR3),
        _SEND(VICR4), _SET(1280),
        _SEND(VICR5), _SET(800),
        _SEND(VICR6), _SET(0xC01B),		// VSP+,HSP+,VCS+Burst(Solomon solution for ESD),24bpp
        _SEND(ACR1),  _SET(0x161C),		// Bit[12:10]: bias current of MIPI output
        -(SINTVL),
        _SEND(LCFR),  _SET(0x0003),		// 4 lane mode
        _SEND(TR),    _SET(0x0005),		// CO: BGR (Color Order:: s_Frontbuffer.ColorFormat == NvColorFormat_A8B8G8R8)
        _SEND(PCR),   _SET(0x0000),		// PLL power down
        -(SINTVL),
    };

    if (valPLCR == 0) {
        if (system_rev >= 4) {		// bridge TX_CLK == 20M
            valPLCR = _SET(0xC019);		// Configure PLL: fOUT = 20M(fIN)/1(MS)*25(NF) = 500M
            valCCR  = _SET(0x0007);		// LP mode Clock Divider:: derived from PLL
        } else {			// bridge TX_CLK == 24M
            valPLCR = _SET(0x8453);		// Configure PLL: fOUT = 24M(fIN)/4(MS)*83(NF) = 498M
            valCCR  = _SET(0x0007);		// LP mode Clock Divider:: derived from PLL
        }
    }

    SEND_SSD2825_SEQ(data_to_send);
    ssd2825_init_pll(valPLCR, valCCR);		// LPX of panel drive IC is 16Mbps
}

static void ssd2825_config_reg(int cfgr)
{
    int data_to_send[2] =
    {
        _SEND(CFGR), cfgr,
    };

    SEND_SSD2825_SEQ(data_to_send);
}

static void s6e8ab0x_panel_cond(void)
{
    static const int data_to_send[] =
    {
        _SEND(PSCR1), _SET(41),			// Transmit Data Count = 41 bytes
        _SEND(GPDR),				// <<[1] Panel Condition Set>>
        _SET(0x01F8), _SET(0x008E), _SET(0x0000), _SET(0x00AC), _SET(0x8D9E), _SET(0x4E1F), _SET(0x7D9C),
        _SET(0x103F), _SET(0x2000), _SET(0x1002), _SET(0x107D), _SET(0x0000), _SET(0x0802), _SET(0x3410),
        _SET(0x3434), _SET(0xC1C0), _SET(0x0001), _SET(0x82C1), _SET(0xC800), _SET(0xE3C1), _SET(0x0001),
        -(LINTVL),
        _CKBTA,
    };

    SSD2825_WritePanel_WS(data_to_send);
}

static void s6e8ab0x_apply_level2_key(void)
{
    static const int data_to_send[] =
    {
        _SEND(PSCR1), _SET(0x0003),		// Transmit Data Count = 3 bytes
        _SEND(GPDR),				// Generic Packet Drop
        _SET(0x5AF0), _SET(0x005A),		// <<Apply Level2 Key (Auto Power On)>>
        -(SINTVL),
        _CKBTA,
    };

    SSD2825_WritePanel_WS(data_to_send);
}

static void s6e8ab0x_sleep_out(void)
{
    static const int data_to_send[] =
    {
        _SEND(PSCR1), _SET(0),			// Transmit Data Count = 0 (w/o Generic Packet Drop)
        _DCS(0x11),				// <<Sleep Out>>
        -(LINTVL),
    };

    SEND_SSD2825_SEQ(data_to_send);
}

static void s6e8ab0x_power_control_set(void)
{
    static const int data_to_send[] =
    {
        _SEND(PSCR1), _SET(0x0009),		// Transmit Data Count = 3 bytes
        _SEND(GPDR),			        // Generic Packet Drop
        _SET(0x50F5), _SET(0x3333),_SET(0x5400),_SET(0x3396),_SET(0x0001),	// <<Power Control Set >>
        -(SINTVL),
        _CKBTA,
    };

    SSD2825_WritePanel_WS(data_to_send);
}

static void s6e8ab0x_apply_mtp_key(void)
{
    static const int data_to_send[] =
    {
        _SEND(PSCR1), _SET(0x0003),		// Transmit Data Count = 3 bytes
        _SEND(GPDR),				// Generic Packet Drop
        _SET(0x5AF1), _SET(0x005A),		// <<Apply MTP Key >>
        -(SINTVL),
        _CKBTA,
    };

    SSD2825_WritePanel_WS(data_to_send);
}

static void s6e8ab0x_display_cond(void)
{
    static const int data_to_send[] =
    {
        _SEND(PSCR1), _SET(4),			// Transmit Data Count = 4 bytes
        _SEND(GPDR),				// Generic Packet Drop
        _SET(0xC8F2), _SET(0x0D05),		// <<[2] Display Condition Set>>
        -(MINTVL),
        _CKBTA,
    };

    SSD2825_WritePanel_WS(data_to_send);
}

static void s6e8ab0x_gamma_select(int gamma)
{
    // NOT TO USE _CHBTA OUTSIDE PANEL INITIALIZATION
    static const int data_to_send[] =
    {
        _SEND(PSCR1), _SET(2),			// Transmit Data Count = 1 (w/o Generic Packet Drop)???
        _SEND(GPDR),
        _SET(0x0126),          			// <<[3] Gamma Condition Set -- Gamma Select>>
    };
    //data_to_send[3] = gamma;
    SEND_SSD2825_SEQ(data_to_send);
}
/* Gamma 2.2 Setting (250cd, 7500K) */
#if SET_GAMMA
static int s6e8ab0x_gamma_cond(void)
{
    struct backlight_device *bd = platform_get_drvdata(&ast_backlight_device);

    int brightness = bd->props.brightness;
    int *gamma,gamma_table_index;
    //PNL_PRINT(("%s: brightness = %d \n",__func__,brightness));

    gamma_table_index = ast_bl_output_measured[brightness];	
    gamma = brightness_table[gamma_table_index].gamma_table;
    SEND_SSD2825_SEQ_WS(gamma,NV_ARRAY_SIZE(ast_gamma_50));
    return 0;
}
#endif
static void s6e8ab0x_gamma_update(void)
{
    // NOT TO USE _CHBTA OUTSIDE PANEL INITIALIZATION
    static const int data_to_send[] =
    {
        _SEND(PSCR1), _SET(2),			// Transmit Data Count = 2 bytes
        _SEND(GPDR),				// <<[3] Gamma Condition Set -- Gamma/LTPS Set Update>>
        _SET(0x03F7),
        -(LINTVL),
    };

    SEND_SSD2825_SEQ(data_to_send);
}
#if DYNAMIC_ELVSS
static void s6e8ab0x_elvss_set(void)
{
    static const int data_to_send[] =
    {
        _SEND(PSCR1), _SET(3),			// Transmit Data Count = 3 bytes
        _SEND(GPDR),				// Generic Packet Drop
        _SET(0x04B1), _SET(0x0000),		// <<[5-1] Dynamic ELVSS set: DEFAULT>>
        -(SINTVL),
        _CKBTA,
    };

    SSD2825_WritePanel_WS(data_to_send);
}
#endif
#if OPTIMUM_ELVSS
static void s6e8ab0x_optimum_elvss_ctrl(void)
{
    struct backlight_device *bd = platform_get_drvdata(&ast_backlight_device);
    int brightness = bd->props.brightness;  	
    int luminance,gamma_table_index;

    static int data_to_send[] =
    {
        _SEND(PSCR1), _SET(3),			// Transmit Data Count = 3 bytes
        _SEND(GPDR),				// <<[5-3] Optimum ELVSS set: (ID3 <> 44h)>>
        _SET(0x84B1), _SET(0x0000),		// <<Set Brightness>>
    };

    gamma_table_index = ast_bl_output_measured[brightness];
    luminance = brightness_table[gamma_table_index].nits;

    if (luminance <= 250 && luminance > 200)
    {
        data_to_send[4] = _SET((u8)(panel_id3)+ELVSS_2H_OFFSET);      //0x9E
    }
    else if (luminance <= 200 && luminance > 150) 
    {
        data_to_send[4] = _SET((u8)(panel_id3)+ELVSS_2H_OFFSET+0x04); //0xA2
    }
    else if (luminance <= 150 && luminance > 100) 
    {
        data_to_send[4] = _SET((u8)(panel_id3)+ELVSS_2H_OFFSET+0x08); //0xA6
    }
    else
    {
        data_to_send[4] = _SET((u8)(panel_id3)+ELVSS_2H_OFFSET+0x0C); //0xA9
    }

    if ((u8)(data_to_send[4]) >= 0xa9 ) 
        data_to_send[4] = _SET(0xa9);
    //PNL_PRINT(("%s: luminance = %d elvss 2 = 0x%x\n",__func__,luminance,data_to_send[4]));

    // NOT TO USE _CHBTA OUTSIDE PANEL INITIALIZATION
    SEND_SSD2825_SEQ(data_to_send);
}
#endif
static int s6e8ab0x_update_gamma_ctrl(int brightness)
{
    int *gamma,gamma_table_index;

    gamma_table_index = ast_bl_output_measured[brightness];
    gamma = brightness_table[gamma_table_index].gamma_table;
    ssd2825_config_reg(CFGR_S_WR_DCS_VEN_HS);

    s6e8ab0x_gamma_select(BRIGHT_GAMMA);
    SEND_SSD2825_SEQ_WS(gamma,NV_ARRAY_SIZE(ast_gamma_50));

    s6e8ab0x_gamma_update();
#if OPTIMUM_ELVSS
    s6e8ab0x_optimum_elvss_ctrl();
#endif
    return 0;
}

static void s6e8ab0x_etc_cond1(void)
{
    static const int data_to_send[] =
    {
        _SEND(PSCR1), _SET(7),			// Transmit Data Count = 7 bytes
        _SEND(GPDR),			        // <<[4] Etc Condition Set>>
        _SET(0x0BF4), _SET(0x060A), _SET(0x330B), _SET(0x0002),
        -(SINTVL),
        _CKBTA,
        _SEND(PSCR1), _SET(4),			// Transmit Data Count = 4 bytes
        _SEND(GPDR),			        // Generic Packet Drop
        _SET(0x85F6), _SET(0x0200),		// <<[4] Etc Condition Set -- rotate 180 degree>>
        -(LINTVL),
        _CKBTA,
    };

    SSD2825_WritePanel_WS(data_to_send);
}

#if SET_NVM
static void s6e8ab0x_elvss_nvm_set(void)
{
    static const int data_to_send[] =
    {
        _SEND(PSCR1), _SET(22),			// Transmit Data Count = 22 bytes
        _SEND(GPDR),				// <<[5-1] Dynamic ELVSS set: DEFAULT -- NVM SETTING>>
        _SET(0x14D9), _SET(0x205C), _SET(0x0F0C), _SET(0x0041), _SET(0x1110), _SET(0xA812), _SET(0x0055),
        _SET(0x0000), _SET(0x8000), _SET(0xEDCB), _SET(0xAF7F),
        -(LINTVL),
        _CKBTA,
    };

    SSD2825_WritePanel_WS(data_to_send);
}
#endif

#if ACLCTRL
static void s6e8ab0x_acl_on(void)
{
    static const int data_to_send[] =
    {
        _SEND(PSCR1), _SET(2),			// Transmit Data Count = 2 bytes
        _SEND(GPDR),				//
        _SET(0x01C0),				// <ACL on>>
        -(SINTVL),
        _CKBTA,
    };

    SSD2825_WritePanel_WS(data_to_send);
}

/* Full white 40% reducing setting */
static void s6e8ab0x_acl_ctrl_set(void)
{
    /* Full white 40% reducing setting */
    static const int cutoff_40[] =
    {
        _SEND(PSCR1), _SET(31),			// Transmit Data Count = 31 bytes
        _SEND(GPDR),
        _SET(0x4DC1),_SET(0x1D96),_SET(0x0000),_SET(0xFF04),_SET(0x0000),_SET(0x1F03),
        _SET(0x0000),_SET(0x0000),_SET(0x0000),_SET(0x2100),_SET(0x3737),_SET(0x3737),
        _SET(0x3737),_SET(0x3737),_SET(0x4D1D),_SET(0x0096),
        -(SINTVL),
        _CKBTA,
    };

    SSD2825_WritePanel_WS(cutoff_40);
}
#endif
static void s6e8ab0x_display_on(void)
{
    static const int data_to_send[] =
    {
        _SEND(LCR),   _SET(0x0000),		// Not BTA After Write
        _SEND(PSCR1), _SET(0),			// Transmit Data Count = 0 (w/o Generic Packet Drop)
        _DCS(0x29),				// <<Display On>>
    };

    SEND_SSD2825_SEQ(data_to_send);
}

static void s6e8ab0x_global_parameter(void)
{
    static const int data_to_send[] =
    {
        _SEND(PSCR1), _SET(2),			// Transmit Data Count = 2 bytes
        _SEND(GPDR),				// Global Parameter
        _SET(0x03B0),				// to Skip 3 bytes for IDs
        -(SINTVL),
        _CKBTA,
    };

    SSD2825_WritePanel_WS(data_to_send);
}

static void s6e8ab0x_vregout_set(void)
{
    static const int data_to_send[] =
    {
        -120,
        _SEND(PSCR1), _SET(9),			// Transmit Data Count = 9 bytes
        _SEND(GPDR),				// <<[6] VREGOUT SET>>
        _SET(0x01D1), _SET(0x000B), _SET(0x4000), _SET(0x000D), _SET(0x0000),	// <<Display On>>
        -(SINTVL),
        _CKBTA,
    };

    SSD2825_WritePanel_WS(data_to_send);
}

static int s6e8ab0x_panel_init(void)
{
        ssd2825_init_seq();
        ssd2825_config_reg(CFGR_S_WR_DCS_VDS_LP);
	s6e8ab0x_apply_level2_key();
	s6e8ab0x_power_control_set();
	s6e8ab0x_sleep_out();	
	s6e8ab0x_apply_mtp_key();
	s6e8ab0x_panel_cond();
	s6e8ab0x_display_cond();
	s6e8ab0x_gamma_select(BRIGHT_GAMMA);
#if SET_GAMMA
	s6e8ab0x_gamma_cond();
#endif
	s6e8ab0x_gamma_update();
	s6e8ab0x_etc_cond1();
#if ACLCTRL				// must set ACL associated within ETC (see specs)
	s6e8ab0x_acl_on();
	s6e8ab0x_acl_ctrl_set();
#endif
#if SET_NVM
	s6e8ab0x_elvss_nvm_set();
#endif
#if DYNAMIC_ELVSS
	s6e8ab0x_elvss_set();
#endif
        s6e8ab0x_global_parameter();	// Global Parameter must be issued before MIPI VIDEO ON
        s6e8ab0x_vregout_set();

	ssd2825_config_reg(CFGR_S_WR_DCS_VEN_HS);
	s6e8ab0x_display_on();

	return 0;
}

static int ast_spi_write(int Data)
{

        struct spi_message	msg;
        struct spi_transfer	xfer;
        int			ret;

        // Initialize the SPI message and transfer data structures
        spi_message_init(&msg);
        memset(&xfer, 0, sizeof(xfer));
        xfer.tx_buf = &Data;//pTxBuffer;
        xfer.len = 3;

        // Add our only transfer to the message
        spi_message_add_tail(&xfer, &msg);

        // Send the message and wait for completion
        ret = spi_sync(w1_disp1_spi, &msg);
        if (ret < 0)
           pr_err("spi_write: spi couldn't write \n");
        //PNL_PRINT(("dump reg = %02x \n",Data));
        return ret;
}

static int ast_spi_read(int Data, int *pRxBuffer)
{
        struct spi_message	msg;
        struct spi_transfer	xfer;
        int			ret;

        // Initialize the SPI message and transfer data structures
        spi_message_init(&msg);
        memset(&xfer, 0, sizeof(xfer));
        xfer.tx_buf = &Data;//pTxBuffer;
        xfer.rx_buf = pRxBuffer;
        xfer.len = 3;

        // Add our only transfer to the message
        spi_message_add_tail(&xfer, &msg);

        // Send the message and wait for completion
        ret = spi_sync(w1_disp1_spi, &msg);
        if (ret < 0)
           pr_err("spi_read: spi couldn't read \n");

        (*pRxBuffer) &= 0xFFFF;
        return ret;
}

#ifdef CONFIG_DEBUG_FS
static int panel_reg_open_file(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;
	return 0;
}

static ssize_t w_panel_reg_write_file(struct file *file,
		const char __user *user_buf, size_t count, loff_t *ppos)
{
	char buf[256];
	int wdata[32];
	char *start = buf;
	unsigned long value;
	int step = 0;
	//int i;

	if(count > sizeof(buf)-1)
		return -EFAULT;
	if (copy_from_user(buf, user_buf, count))
		return -EFAULT;
	buf[count-1] = '\0';

	//printk(KERN_INFO "w_panel_reg_write_file: %s", buf);
	memset(wdata, 0x00, sizeof(wdata));
	wdata[0] = _SEND(CFGR);
	wdata[1] = _SET(0x034B);
	wdata[2] = -5;
	wdata[3] = _SEND(PSCR1);
	wdata[5] = _SEND(GPDR);
	while(*start) {
		while (*start == ' ')
			start++;

		value = simple_strtoul(start, &start, 16);
		if(0 == (step & 0x01)) {
			wdata[(step/2)+6] = _SET(value);
		} else {
			wdata[(step/2)+6] |= (value<<8);
		}
		step++;
	}
	wdata[4] = _SET(step);

	/*
	printk(KERN_INFO "Write SPI Panel:\n");
	step = ((step-1)/2)+3;
	for(i = 0; i <= step; i++) {
		printk(KERN_INFO "0x%08X", wdata[i]);
	}
	*/

	mutex_lock(&lcd_lock);
	SEND_SSD2825_SEQ(wdata);
	mutex_unlock(&lcd_lock);
	return count;
}

static const struct file_operations w_panel_reg_fops = {
	.open = panel_reg_open_file,
	.write = w_panel_reg_write_file,
	.llseek = default_llseek,
};

#define PANEL_READ_NUM	48
#define SPI_RTRY	500
static ssize_t r_panel_reg_read_file(struct file *file, char __user *user_buf,
				   size_t count, loff_t *ppos)
{
	ssize_t ret;
	char *buf;
	int buffer;
	int i, j;
	int wdata[] = {
		/*_SEND(LCR), _SET(0x0001),*/ _SEND(MRSR), _SET(PANEL_READ_NUM), _SEND(CFGR),
		_SET(0x03CB), _SEND(PSCR1), _SET(1), _SEND(GPDR), _SET(spi_panel_addr_r) };
	int rdata = _RDCYCL;

	if (*ppos < 0 || !count)
		return -EINVAL;
	if(*ppos > 0)
		return 0;

	buf = kmalloc(count, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	mutex_lock(&lcd_lock);
	SEND_SSD2825_SEQ(wdata);
	for (i=0; i<SPI_RTRY; i++)
	{
		buffer = 0;
		ast_spi_write(_SEND(ISR));
		ast_spi_read(rdata, &buffer);

		if (buffer & 0x01)	//Check RDR (Read Data Ready)
		{
			sprintf(buf, "Read Panel Register: 0x%02X\n", spi_panel_addr_r);
			ast_spi_write(_SEND(RR));
			for(j=0; j<PANEL_READ_NUM; j+=2) {
				buffer = 0;
				ast_spi_read(rdata, &buffer);
				//printk(KERN_INFO "addr %d: 0x%08X\n", j, buffer);
				if(0x00 == (j % 8))
					sprintf(buf, "%s%02d: ", buf, j);
				sprintf(buf, "%s%02X %02X ", buf, (buffer&0xFF), (buffer&0xFF00)>>8);
				if(0x06 == (j & 0x06))
					sprintf(buf, "%s\n", buf);
			}
			break;
		}
		mdelay(1);
	}
	mutex_unlock(&lcd_lock);

	if (i < SPI_RTRY) {
		ret = strlen(buf);
		if (copy_to_user(user_buf, buf, ret))
			ret = -EFAULT;
		else
			*ppos += ret;
	} else {
		ret = -EFAULT;
	}

	kfree(buf);
	return ret;
}

static ssize_t r_panel_reg_write_file(struct file *file,
		const char __user *user_buf, size_t count, loff_t *ppos)
{
	char buf[16];
	char *start = buf;
	unsigned long value;

	if(count > sizeof(buf)-1)
		return -EFAULT;
	if (copy_from_user(buf, user_buf, count))
		return -EFAULT;
	buf[count-1] = '\0';

	//printk(KERN_INFO "r_panel_reg_write_file: %s", buf);
	value = simple_strtoul(start, &start, 16);
	spi_panel_addr_r = value;

	return count;
}

static const struct file_operations r_panel_reg_fops = {
	.open = panel_reg_open_file,
	.read = r_panel_reg_read_file,
	.write = r_panel_reg_write_file,
	.llseek = default_llseek,
};

static ssize_t w_panel_msg_write_file(struct file *file,
		const char __user *user_buf, size_t count, loff_t *ppos)
{
	char buf[256];
	char stop[] = "BUG!BUG!STOP!BUG!BUG!";

	if(count > sizeof(buf)-1)
		return -EFAULT;
	if (copy_from_user(buf, user_buf, count))
		return -EFAULT;
	buf[count-1] = '\0';

	printk(KERN_INFO "%s: %s", __func__, buf);
	if (memcmp(buf, stop, sizeof(stop)-1) == 0)
		BUG();
	return count;
}

static const struct file_operations w_panel_msg_fops = {
	.open = panel_reg_open_file,
	.write = w_panel_msg_write_file,
	.llseek = default_llseek,
};

static ssize_t bridge_index_read_file(struct file *file, char __user *user_buf,
				   size_t count, loff_t *ppos)
{
	char buf[32];

	if (*ppos < 0 || !count)
		return -EINVAL;
	if(*ppos > 0)
		return 0;

	sprintf(buf, "0x%02X\n", spi_bridge_index);
	count = strlen(buf);
	if(copy_to_user(user_buf, buf, count))
		return -EFAULT;
	*ppos += count;
	return count;
}

static ssize_t bridge_index_write_file(struct file *file,
		const char __user *user_buf, size_t count, loff_t *ppos)
{
	char buf[16];
	char *start = buf;
	unsigned long index;

	if(count > sizeof(buf)-1)
		return -EFAULT;
	if (copy_from_user(buf, user_buf, count))
		return -EFAULT;
	buf[count-1] = '\0';

	//printk(KERN_INFO "bridge_index_write_file: %s", buf);
	index = simple_strtoul(start, &start, 0);
	if(index < 0xB0 || index > 0xFF)
		return -EINVAL;

	spi_bridge_index = index;
	return count;
}

static const struct file_operations bridge_index_fops = {
	.open = panel_reg_open_file,
	.read = bridge_index_read_file,
	.write = bridge_index_write_file,
	.llseek = default_llseek,
};

static ssize_t bridge_register_read_file(struct file *file, char __user *user_buf,
				   size_t count, loff_t *ppos)
{
	ssize_t ret;
	char *buf;
	int buffer;
	int rdata = _RDCYCL;

	if (*ppos < 0 || !count)
		return -EINVAL;
	if(*ppos > 0)
		return 0;

	buf = kmalloc(count, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	mutex_lock(&lcd_lock);
	ast_spi_write(_CMDCYCL + spi_bridge_index);
	ast_spi_read(rdata, &buffer);
	mutex_unlock(&lcd_lock);

	sprintf(buf, "0x%04X\n", buffer);
	ret = strlen(buf);
	if (copy_to_user(user_buf, buf, ret))
		ret = -EFAULT;
	else
		*ppos += ret;

	kfree(buf);
	return ret;
}

static ssize_t bridge_register_write_file(struct file *file,
		const char __user *user_buf, size_t count, loff_t *ppos)
{
	char buf[32];
	int wdata[2];
	char *start = buf;
	unsigned long value;

	if(count > sizeof(buf)-1)
		return -EFAULT;
	if (copy_from_user(buf, user_buf, count))
		return -EFAULT;
	buf[count-1] = '\0';

	//printk(KERN_INFO "bridge_register_write_file: %s", buf);
	while (*start == ' ')
		start++;

	memset(wdata, 0x00, sizeof(wdata));
	value = simple_strtoul(start, &start, 0);
	wdata[0] = _CMDCYCL + spi_bridge_index;
	wdata[1] = _SET(value & 0xFFFF);

	mutex_lock(&lcd_lock);
	SEND_SSD2825_SEQ(wdata);
	mutex_unlock(&lcd_lock);
	return count;
}

static const struct file_operations bridge_register_fops = {
	.open = panel_reg_open_file,
	.read = bridge_register_read_file,
	.write = bridge_register_write_file,
	.llseek = default_llseek,
};

static ssize_t brightness_level_read_file(struct file *file, char __user *user_buf,
				   size_t count, loff_t *ppos)
{
	char buf[50];

         struct backlight_device *bd = platform_get_drvdata(&ast_backlight_device);

	int gamma_table_index,brightness = bd->props.brightness;
	 
	if (*ppos < 0 || !count)
		return -EINVAL;
	if(*ppos > 0)
		return 0;
        gamma_table_index= ast_bl_output_measured[brightness];
#if ACLCTRL
	sprintf(buf, "Brightness Level = %d with %d nits\n", brightness,brightness_table[gamma_table_index].nits*6/10);
#else
	sprintf(buf, "Brightness Level = %d with %d nits\n", brightness,brightness_table[gamma_table_index].nits);
#endif
	count = strlen(buf);
	if(copy_to_user(user_buf, buf, count))
		return -EFAULT;
	*ppos += count;
	return count;
}

static const struct file_operations brightness_level_fops = {
	.open = panel_reg_open_file,
	.read = brightness_level_read_file,	
	.llseek = default_llseek,
};
#endif

static void ast_panel_pins_request(void)
{
        int i,err;

        for (i = 0; i<NV_ARRAY_SIZE(gpio_config_tbl);i++)
        {
           err = gpio_request(gpio_config_tbl[i], "lcd_gpio_config");

           if (err < 0)
           {
	     pr_err("%s: lcd_gpio_config request fail = %d\n",__func__,err);
           }
        }
}

static void ast_panel_config_pins(void)
{
        int i;

        for (i = 0; i<NV_ARRAY_SIZE(gpio_config_tbl);i++)
        {
           gpio_direction_input(gpio_config_tbl[i]);
           tegra_gpio_enable(gpio_config_tbl[i]);

        }
/*
        err = gpio_request(ast_lcd_spiclk, "ast_lcd_spiclk");
           if (err < 0){
	    pr_err("%s: ast_lcd_spiclk request fail = %d\n",__func__,err);

        }
           gpio_direction_input(ast_lcd_spiclk);
           tegra_gpio_enable(ast_lcd_spiclk);
*/
}

static void ast_panel_reconfig_pins(void)
{
        int i;

        for (i = 0; i<NV_ARRAY_SIZE(gpio_config_tbl);i++)
        {
           tegra_gpio_disable(gpio_config_tbl[i]);
        }

        gpio_direction_output(ast_lcd_shutdown, 0);
	tegra_gpio_enable(ast_lcd_shutdown);
	gpio_direction_output(ast_lcd_reset, 1);
	tegra_gpio_enable(ast_lcd_reset);
}

static int enable_panel(void)
{
	// Panel 6.3v Enable
	if (ast_en_vdd_bl == NULL) {
		ast_en_vdd_bl = regulator_get(NULL, "vdd_backlight");
		if (WARN_ON(IS_ERR(ast_en_vdd_bl))){
			pr_err("%s: couldn't get regulator vdd_6v3_bl: %ld\n", __func__, PTR_ERR(ast_en_vdd_bl));
			return -ENODEV;
		}
	}

	// Panel 1.8v Enable
	if (ast_en_1v8_pnl == NULL) {
		ast_en_1v8_pnl = regulator_get(NULL, "vdd_1v8_panel");
		if (WARN_ON(IS_ERR(ast_en_1v8_pnl))){
			pr_err("%s: couldn't get regulator vdd_1v8_panel: %ld\n", __func__, PTR_ERR(ast_en_1v8_pnl));
			return -ENODEV;
		}
	}

	// Panel 3.3v Enable
	if (ast_en_vdd_pnl == NULL) {
		ast_en_vdd_pnl = regulator_get(NULL, "vdd_lcd_panel");
		if (WARN_ON(IS_ERR(ast_en_vdd_pnl))){
			pr_err("%s: couldn't get regulator vdd_3v3_pnl: %ld\n", __func__, PTR_ERR(ast_en_vdd_pnl));
			return -ENODEV;
		}
	}

	//Panel 5.8v Enable
	if (ast_en_5v8_bl == NULL) {
		ast_en_5v8_bl = regulator_get(NULL, "vdd_5v8_backlight");
		if (WARN_ON(IS_ERR(ast_en_5v8_bl))){
			pr_err("%s: couldn't get regulator vdd_5v8_backlight: %ld\n", __func__, PTR_ERR(ast_en_5v8_bl));
			return -ENODEV;
		}
	}

	//mipi bridge 1.8v Enable
	if (ast_en_1v8_mipi == NULL) {
		ast_en_1v8_mipi = regulator_get(NULL, "vdd_1v8_mipi");
		if (WARN_ON(IS_ERR(ast_en_1v8_mipi))){
			pr_err("%s: couldn't get regulator vdd_1v8_mipi: %ld\n", __func__, PTR_ERR(ast_en_1v8_mipi));
			return -ENODEV;
		}
	}

	//*JHC* No HW Pull up for reset, so set it HIGH firstly!
	ast_panel_reconfig_pins();

	// Panel 6.3v Enable
	regulator_enable(ast_en_vdd_bl);
	mdelay(5);
	regulator_enable(ast_en_1v8_pnl);
	mdelay(5);
	regulator_enable(ast_en_vdd_pnl);
	mdelay(5);
	regulator_enable(ast_en_5v8_bl);
	mdelay(5);
	regulator_enable(ast_en_1v8_mipi);
	mdelay(5);

	gpio_set_value(ast_lcd_shutdown, 0);

	/* A long delay inserted for 5.8V soft start */
	mdelay(50);

	//ast_spi_read(0x730000,buffer);
	//PNL_PRINT("%s: Panel ID=%02x %02x \n",__func__,buffer[1],buffer[0]);
	if (is_panel_init_on == false)
	{
		//PNL_PRINT(("%s: is_panel_init_on = false \n",__func__));
		gpio_set_value(ast_lcd_reset, 1);
		mdelay(5);
		gpio_set_value(ast_lcd_reset, 0);
		mdelay(5);
		gpio_set_value(ast_lcd_reset, 1);
		mdelay(10);

		s6e8ab0x_panel_init();
	}

	return 0;
}

static int read_panel_status(int regno)
{
	int wdata[] = {
		/*_SEND(LCR),   _SET(0x0001),*/
		_SEND(MRSR),  _SET(4),		// should be enough for 1-time read
		_SEND(CFGR),  _SET(0x03CB),
		_SEND(PSCR1), _SET(1),
		_SEND(GPDR),  _SET(regno)
	};
	int i, buffer = 0;
	int rdata = _RDCYCL;

	mutex_lock(&lcd_lock);
	SEND_SSD2825_SEQ(wdata);
	for (i=0; i<50; i++) {
		buffer = 0;
		ast_spi_write(_SEND(ISR));
		ast_spi_read(rdata, &buffer);
		if (buffer & 0x01) {		//Check RDR (Read Data Ready)
			buffer = 0;
			ast_spi_write(_SEND(RR));
			ast_spi_read(rdata, &buffer);
			goto _exit;
		}
		mdelay(1);
	}
	buffer = -1;	// Failure
_exit:
	mutex_unlock(&lcd_lock);
	return buffer;
}

static int ast_panel_enable(void)
{
	int ret;

	PNL_PRINT(("%s: start \n",__func__));

//	if (panel_on || mutex_lock_interruptible(&lcd_lock))
	if (panel_on)
		return 0;
	mutex_lock(&lcd_lock);

	ret = enable_panel();

	mutex_unlock(&lcd_lock);
	panel_on = 1;

        if (is_panel_init_on == true )
	{
	   panel_id3 = read_panel_status(0xDC);
	   if (panel_id3 < 0)
                pr_err("spi: couldn't read panel id3 reg.\n");

           PNL_PRINT(("%s: panel_id3 = 0x%x \n",__func__,panel_id3));
	}

	#ifdef ESDON_MIPICLOCK_WORKAROUND
	if (ret == 0) {
		schedule_delayed_work(&mytimer_workqueue, HZ);	// delay is necessary
	}
	#endif

        PNL_PRINT(("%s: end \n",__func__));
	return ret;
}

static int ast_panel_disable(void)
{
	PNL_PRINT(("%s:\n",__func__));

	//**JHC** panel and bridge is reset in ast_panel_early_suspend,
	//        so spi command is NOT allowed here 

	if (panel_on)
	{
		#ifdef ESDON_MIPICLOCK_WORKAROUND
		cancel_delayed_work_sync(&mytimer_workqueue);
		#endif

		//if (mutex_lock_interruptible(&lcd_lock))
		//	return 0;
		mutex_lock(&lcd_lock);

		regulator_disable(ast_en_5v8_bl);
		mdelay(5);
		regulator_disable(ast_en_vdd_pnl);
		mdelay(5);
		regulator_disable(ast_en_1v8_pnl);
		mdelay(5);
		regulator_disable(ast_en_vdd_bl);
		mdelay(5);
		regulator_disable(ast_en_1v8_mipi);
		mdelay(5);
		gpio_set_value(ast_lcd_shutdown, 1);

		ast_panel_config_pins();
                //tegra_gpio_disable(HDMI_HPD_GPIO);
		mutex_unlock(&lcd_lock);

		panel_on = 0;
		is_panel_init_on = false;
		prev_brightness = -1;
	}
	return 0;
}

#ifdef ESDON_MIPICLOCK_WORKAROUND
static void mytimer_work(struct work_struct *work)
{
	if (panel_on) {
		int status;
		status = read_panel_status(0x0A);
		if ((status < 0) || ((status & 0x4) == 0)) {	//Display is NOT On
			goto _fail;
		}
		status = read_panel_status(0x0E);
///		if ((status < 0) || (status & 0x3E)) {		//D5~D1 must be zeros
		if ((status < 0) || (status & 0x3F)) {		//D5~D0 must be zeros
			goto _fail;
		}
		display_off_cnt = 0;
		goto _exit;
_fail:
		if (++display_off_cnt > 2) {
			display_off_cnt = 0;
			is_panel_init_on = false;

			mutex_lock(&lcd_lock);
			enable_panel();
			mutex_unlock(&lcd_lock);
		}
_exit:
		schedule_delayed_work(&mytimer_workqueue, HZ);
	}
}
#endif

/*
static int ast_hdmi_vddio_enable(void)
{

	int ret;

	if (!ast_hdmi_vddio) {
		ast_hdmi_vddio = regulator_get(NULL, "vdd_hdmi_con");
		if (IS_ERR_OR_NULL(ast_hdmi_vddio)) {
			ret = PTR_ERR(ast_hdmi_vddio);
			pr_err("hdmi: couldn't get regulator vdd_hdmi_con\n");
			ast_hdmi_vddio = NULL;
			return ret;
		}
	}

	ret = regulator_enable(ast_hdmi_vddio);
	if (ret < 0) {
		pr_err("hdmi: couldn't enable regulator vdd_hdmi_con\n");
		regulator_put(ast_hdmi_vddio);
		ast_hdmi_vddio = NULL;
		return ret;
	}

	return ret;
}

static int ast_hdmi_vddio_disable(void)
{
	PNL_PRINT(("%s: \n",__func__));

        if (ast_hdmi_vddio) {
		regulator_disable(ast_hdmi_vddio);
		regulator_put(ast_hdmi_vddio);
		ast_hdmi_vddio = NULL;
	}

	return 0;
}
*/
static int ast_hdmi_enable(void)
{
	int ret;
	//PNL_PRINT(("%s: \n",__func__));

	if (!ast_hdmi_reg) {
		ast_hdmi_reg = regulator_get(NULL, "avdd_hdmi");
		if (IS_ERR_OR_NULL(ast_hdmi_reg)) {
			pr_err("hdmi: couldn't get regulator avdd_hdmi\n");
			ast_hdmi_reg = NULL;
			return PTR_ERR(ast_hdmi_reg);
		}
	}

	ret = regulator_enable(ast_hdmi_reg);
	if (ret < 0) {
		pr_err("hdmi: couldn't enable regulator avdd_hdmi\n");
		return ret;
	}

	if (!ast_hdmi_pll) {
		ast_hdmi_pll = regulator_get(NULL, "avdd_hdmi_pll");
		if (IS_ERR_OR_NULL(ast_hdmi_pll)) {
			pr_err("hdmi: couldn't get regulator avdd_hdmi_pll\n");
			ast_hdmi_pll = NULL;
			regulator_put(ast_hdmi_reg);
			ast_hdmi_reg = NULL;
			return PTR_ERR(ast_hdmi_pll);
		}
	}

	ret = regulator_enable(ast_hdmi_pll);
	if (ret < 0) {
		pr_err("hdmi: couldn't enable regulator avdd_hdmi_pll\n");
		return ret;
	}

	return 0;
}

static int ast_hdmi_disable(void)
{
	//PNL_PRINT(("%s: \n",__func__));
        regulator_disable(ast_hdmi_reg);
	regulator_put(ast_hdmi_reg);
	ast_hdmi_reg = NULL;

	regulator_disable(ast_hdmi_pll);
	regulator_put(ast_hdmi_pll);
	ast_hdmi_pll = NULL;

	return 0;
}

static struct resource ast_disp1_resources[] =
{
	{
		.name	= "irq",
		.start	= INT_DISPLAY_GENERAL,
		.end	= INT_DISPLAY_GENERAL,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.name	= "regs",
		.start	= TEGRA_DISPLAY_BASE,
		.end	= TEGRA_DISPLAY_BASE + TEGRA_DISPLAY_SIZE-1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.name	= "fbmem",
		.start	= 0,	/* Filled in by ast_panel_init() */
		.end	= 0,	/* Filled in by ast_panel_init() */
		.flags	= IORESOURCE_MEM,
	},
};

static struct resource ast_disp2_resources[] =
{
	{
		.name	= "irq",
		.start	= INT_DISPLAY_B_GENERAL,
		.end	= INT_DISPLAY_B_GENERAL,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.name	= "regs",
		.start	= TEGRA_DISPLAY2_BASE,
		.end	= TEGRA_DISPLAY2_BASE + TEGRA_DISPLAY2_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.name	= "fbmem",
		.flags	= IORESOURCE_MEM,
		.start	= 0,
		.end	= 0,
	},
	{
		.name	= "hdmi_regs",
		.start	= TEGRA_HDMI_BASE,
		.end	= TEGRA_HDMI_BASE + TEGRA_HDMI_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
};

static struct tegra_dc_mode ast_panel_modes[] =
{
	{
		/* 1280x800@60Hz */
		.pclk = 74180000,
		.h_ref_to_sync = 0,
		.v_ref_to_sync = 1,
		.h_sync_width = 64,
		.v_sync_width = 2,
		.h_back_porch = 128,
		.v_back_porch = 3,
		.h_active = 1280,
		.v_active = 800,
		.h_front_porch = _HFP, //128,
		.v_front_porch = _VFP,
	},
};

static struct tegra_dc_sd_settings ast_sd_settings =
{
	.enable = 0, /* enabled by default. */
	.use_auto_pwm = false,
	.hw_update_delay = 0,
	.bin_width = -1,
	.aggressiveness = 1,
	.phase_in_adjustments = true,
	.use_vid_luma = false,
	/* Default video coefficients */
	.coeff = {5, 9, 2},
	.fc = {0, 0},
	/* Immediate backlight changes */
	.blp = {1024, 255},
	/* Gammas: R: 2.2 G: 2.2 B: 2.2 */
	/* Default BL TF */
	.bltf = {
			{
				{57, 65, 74, 83},
				{93, 103, 114, 126},
				{138, 151, 165, 179},
				{194, 209, 225, 242},
			},
			{
				{58, 66, 75, 84},
				{94, 105, 116, 127},
				{140, 153, 166, 181},
				{196, 211, 227, 244},
			},
			{
				{60, 68, 77, 87},
				{97, 107, 119, 130},
				{143, 156, 170, 184},
				{199, 215, 231, 248},
			},
			{
				{64, 73, 82, 91},
				{102, 113, 124, 137},
				{149, 163, 177, 192},
				{207, 223, 240, 255},
			},
		},
	/* Default LUT */
	.lut = {
			{
				{250, 250, 250},
				{194, 194, 194},
				{149, 149, 149},
				{113, 113, 113},
				{82, 82, 82},
				{56, 56, 56},
				{34, 34, 34},
				{15, 15, 15},
				{0, 0, 0},
			},
			{
				{246, 246, 246},
				{191, 191, 191},
				{147, 147, 147},
				{111, 111, 111},
				{80, 80, 80},
				{55, 55, 55},
				{33, 33, 33},
				{14, 14, 14},
				{0, 0, 0},
			},
			{
				{239, 239, 239},
				{185, 185, 185},
				{142, 142, 142},
				{107, 107, 107},
				{77, 77, 77},
				{52, 52, 52},
				{30, 30, 30},
				{12, 12, 12},
				{0, 0, 0},
			},
			{
				{224, 224, 224},
				{173, 173, 173},
				{133, 133, 133},
				{99, 99, 99},
				{70, 70, 70},
				{46, 46, 46},
				{25, 25, 25},
				{7, 7, 7},
				{0, 0, 0},
			},
		},
	.sd_brightness = &sd_brightness,
	.bl_device = &ast_backlight_device,
};

static struct tegra_fb_data ast_fb_data =
{
	.win		= 0,
	.xres = 1280,
	.yres = 800,
	.bits_per_pixel = 32,
	.flags		= TEGRA_FB_FLIP_ON_PROBE,
};

static struct tegra_fb_data ast_hdmi_fb_data =
{
	.win		= 0,
	.xres = 1280,
	.yres = 800,
	.bits_per_pixel = 32,
	.flags		= TEGRA_FB_FLIP_ON_PROBE,
};

static struct tegra_dc_out ast_disp1_out =
{
	.align		= TEGRA_DC_ALIGN_MSB,
	.order		= TEGRA_DC_ORDER_RED_BLUE,
	.dcc_bus	= -1,
	.sd_settings	= &ast_sd_settings,
	.parent_clk	= "pll_d_out0",
	.type		= TEGRA_DC_OUT_RGB,
	.depth		= 24,
	.dither		= TEGRA_DC_ORDERED_DITHER,
	.modes	 	= ast_panel_modes,
	.n_modes 	= ARRAY_SIZE(ast_panel_modes),
	.height         = 103,
	.width          = 165,
	.enable		= ast_panel_enable,
	.disable	= ast_panel_disable,
	//.postpoweron	= ast_panel_postpoweron,
	//.postsuspend	= ast_panel_postsuspend,
};

static struct tegra_dc_platform_data ast_disp1_pdata =
{
	.flags		= TEGRA_DC_FLAG_ENABLED,
	.default_out	= &ast_disp1_out,
	.emc_clk_rate	= 300000000,
	.fb		= &ast_fb_data,
};

static struct nvhost_device ast_disp1_device =
{
	.name		= "tegradc",
	.id		= 0,
	.resource	= ast_disp1_resources,
	.num_resources	= ARRAY_SIZE(ast_disp1_resources),
	.dev = {
		.platform_data = &ast_disp1_pdata,
	},
};

static struct tegra_dc_out ast_disp2_out =
{
	.type		= TEGRA_DC_OUT_HDMI,
	.flags		= TEGRA_DC_OUT_HOTPLUG_HIGH,
    .parent_clk = "pll_d2_out0",
	.dcc_bus	= 3,
	.hotplug_gpio = HDMI_HPD_GPIO,
	.max_pixclock	= KHZ2PICOS(148500),
	.align		= TEGRA_DC_ALIGN_MSB,
	.order		= TEGRA_DC_ORDER_RED_BLUE,
	.enable		= ast_hdmi_enable,
	.disable	= ast_hdmi_disable,
	//.postsuspend	= ast_hdmi_vddio_disable,
	//.hotplug_init	= ast_hdmi_vddio_enable,
};

static struct tegra_dc_platform_data ast_disp2_pdata =
{
	.flags		= 0,
	.default_out	= &ast_disp2_out,
	.fb		= &ast_hdmi_fb_data,
	.emc_clk_rate	= 300000000,
};

static struct nvhost_device ast_disp2_device =
{
	.name		= "tegradc",
	.id		= 1,
	.resource	= ast_disp2_resources,
	.num_resources	= ARRAY_SIZE(ast_disp2_resources),
	.dev = {
		.platform_data = &ast_disp2_pdata,
	},
};

#if defined(CONFIG_TEGRA_NVMAP)
static struct nvmap_platform_carveout ast_carveouts[] =
{
	[0] = NVMAP_HEAP_CARVEOUT_IRAM_INIT,
	[1] = {
		.name		= "generic-0",
		.usage_mask	= NVMAP_HEAP_CARVEOUT_GENERIC,
		.base		= 0,	/* Filled in by ast_panel_init() */
		.size		= 0,	/* Filled in by ast_panel_init() */
		.buddy_size	= SZ_32K,
	},
};

static struct nvmap_platform_data ast_nvmap_data =
{
	.carveouts	= ast_carveouts,
	.nr_carveouts	= ARRAY_SIZE(ast_carveouts),
};

static struct platform_device ast_nvmap_device = {
	.name	= "tegra-nvmap",
	.id	= -1,
	.dev	= {
		.platform_data = &ast_nvmap_data,
	},
};
#endif

#if defined(CONFIG_ION_TEGRA)

static struct platform_device tegra_iommu_device = {
	.name = "tegra_iommu_device",
	.id = -1,
	.dev = {
		.platform_data = (void *)((1 << HWGRP_COUNT) - 1),
	},
};

static struct ion_platform_data tegra_ion_data = {
	.nr = 4,
	.heaps = {
		{
			.type = ION_HEAP_TYPE_CARVEOUT,
			.id = TEGRA_ION_HEAP_CARVEOUT,
			.name = "carveout",
			.base = 0,
			.size = 0,
		},
		{
			.type = ION_HEAP_TYPE_CARVEOUT,
			.id = TEGRA_ION_HEAP_IRAM,
			.name = "iram",
			.base = TEGRA_IRAM_BASE + TEGRA_RESET_HANDLER_SIZE,
			.size = TEGRA_IRAM_SIZE - TEGRA_RESET_HANDLER_SIZE,
		},
		{
			.type = ION_HEAP_TYPE_CARVEOUT,
			.id = TEGRA_ION_HEAP_VPR,
			.name = "vpr",
			.base = 0,
			.size = 0,
		},
		{
			.type = ION_HEAP_TYPE_IOMMU,
			.id = TEGRA_ION_HEAP_IOMMU,
			.name = "iommu",
			.base = TEGRA_SMMU_BASE,
			.size = TEGRA_SMMU_SIZE,
			.priv = &tegra_iommu_device.dev,
		},
	},
};

static struct platform_device tegra_ion_device = {
	.name = "ion-tegra",
	.id = -1,
	.dev = {
		.platform_data = &tegra_ion_data,
	},
};
#endif

#define AC_IN_GPIO         TEGRA_GPIO_PV1
static struct charging_img_data charging_data = {
    .ac_in_pin = AC_IN_GPIO,
    .bl_pdev = &ast_backlight_device,
};

static struct platform_device charging_img_device = {
    .name  = "charging-img",
    .id = -1,
    .dev   = {
        .platform_data = &charging_data,
    },
};

static struct platform_device *ast_gfx_devices[] __initdata = {
#if defined(CONFIG_TEGRA_NVMAP)
	&ast_nvmap_device,
#endif
#if defined(CONFIG_ION_TEGRA)
	&tegra_ion_device,
#endif
	&charging_img_device,
};

#ifdef CONFIG_BACKLIGHT_CLASS_DEVICE
static int s6e8ab0x_set_brightness(struct backlight_device *bl)
{
	int  ret = 0, brightness = bl->props.brightness;
	int  prevnitsindex = -1,currnitsindex;
	//int max = bl->props.max_brightness;
        //PNL_PRINT(("%s: brightness = %d \n",__func__,brightness));
//	if (!panel_on || mutex_lock_interruptible(&lcd_lock))
	if (!panel_on)
	    return 0;
	mutex_lock(&lcd_lock);

        if (brightness < MIN_BRIGHTNESS || brightness > bl->props.max_brightness) {
		pr_err("%s: lcd brightness should be %d to %d. brightness =%d\n",
			__func__,MIN_BRIGHTNESS, MAX_BRIGHTNESS,brightness);
		mutex_unlock(&lcd_lock);	
		return -EINVAL;
	}
	
        if (prev_brightness >= 0)
           prevnitsindex = ast_bl_output_measured[prev_brightness];

        currnitsindex = ast_bl_output_measured[brightness];  

        if (prevnitsindex != currnitsindex)
        {
	    ret = s6e8ab0x_update_gamma_ctrl(brightness);
	    //PNL_PRINT(("%s: prevnitsindex = %d \n",__func__,prevnitsindex));
	    PNL_PRINT(("%s: currnitsindex = %d \n",__func__,currnitsindex));
	    if (ret) {
	 	pr_err("%s: lcd brightness setting failed.\n",__func__);
	 	mutex_unlock(&lcd_lock);
	 	return -EIO;
	    }
	}
	prev_brightness = brightness;  
	
	mutex_unlock(&lcd_lock);

	return 0;//ret;
}

static int s6e8ab0x_get_brightness(struct backlight_device *bl)
{
        //PNL_PRINT(("%s: \n",__func__));
	return bl->props.brightness;
}

static int gamma_backlight_check_fb(struct backlight_device *bl,
				  struct fb_info *info)
{
	return info->device == &ast_disp1_device.dev;
}

static const struct backlight_ops gamma_backlight_ops = {
	.update_status	= s6e8ab0x_set_brightness,
	.get_brightness	= s6e8ab0x_get_brightness,
	.check_fb	= gamma_backlight_check_fb,
};
#endif

#ifdef CONFIG_HAS_EARLYSUSPEND
/* put early_suspend/late_resume handlers here for the display in order
 * to keep the code out of the display driver, keeping it closer to upstream
 */
struct early_suspend ast_panel_early_suspender;

static void ast_panel_early_suspend(struct early_suspend *h)
{
        PNL_PRINT(("%s: \n",__func__));

	//**JHC**  Code snippet below prevents screen from fading in lines!

	#ifdef ESDON_MIPICLOCK_WORKAROUND
	cancel_delayed_work_sync(&mytimer_workqueue);
	#endif

	gpio_set_value(ast_lcd_reset, 1);
	mdelay(5);
	gpio_set_value(ast_lcd_reset, 0);
	mdelay(5);
	gpio_set_value(ast_lcd_reset, 1);
	mdelay(10);

	/* power down LCD, add use a black screen for HDMI */
	if (num_registered_fb > 0)
		fb_blank(registered_fb[0], FB_BLANK_POWERDOWN);

	if (num_registered_fb > 1)
		fb_blank(registered_fb[1], FB_BLANK_NORMAL);
}

static void ast_panel_late_resume(struct early_suspend *h)
{
	unsigned i;

	PNL_PRINT(("%s: \n",__func__));

	for (i = 0; i < num_registered_fb; i++)
		fb_blank(registered_fb[i], FB_BLANK_UNBLANK);
	tegra_gpio_enable(HDMI_HPD_GPIO);
}
#endif

int __init ast_panel_init(void){

	int err;
	struct resource *res;

	PNL_PRINT(("%s: \n",__func__));

        prev_brightness = -1;

        mutex_init(&lcd_lock);
#if defined(CONFIG_TEGRA_NVMAP)
	ast_carveouts[1].base = tegra_carveout_start;
	ast_carveouts[1].size = tegra_carveout_size;
#endif

#if defined(CONFIG_ION_TEGRA)
	tegra_ion_data.heaps[0].base = tegra_carveout_start;
	tegra_ion_data.heaps[0].size = tegra_carveout_size;
#endif

	is_panel_init_on = true;

	ast_panel_pins_request();

	gpio_request(HDMI_HPD_GPIO, "hdmi_hpd");
	gpio_direction_input(HDMI_HPD_GPIO);
	tegra_gpio_enable(HDMI_HPD_GPIO);

#ifdef CONFIG_HAS_EARLYSUSPEND
	ast_panel_early_suspender.suspend = ast_panel_early_suspend;
	ast_panel_early_suspender.resume = ast_panel_late_resume;
	ast_panel_early_suspender.level = EARLY_SUSPEND_LEVEL_DISABLE_FB;
	register_early_suspend(&ast_panel_early_suspender);
#endif

#ifdef CONFIG_TEGRA_GRHOST
	err = tegra3_register_host1x_devices();
	if (err)
		return err;
#endif

	err = platform_add_devices(ast_gfx_devices,
				ARRAY_SIZE(ast_gfx_devices));

#if defined(CONFIG_TEGRA_GRHOST) && defined(CONFIG_TEGRA_DC)
	res = nvhost_get_resource_byname(&ast_disp1_device,
					 IORESOURCE_MEM, "fbmem");
	res->start = tegra_fb_start;
	res->end = tegra_fb_start + tegra_fb_size - 1;
#endif

	/* Copy the bootloader fb to the fb. */
	tegra_move_framebuffer(tegra_fb_start, tegra_bootloader_fb_start,
				min(tegra_fb_size, tegra_bootloader_fb_size));

	if (!err)
		err = nvhost_device_register(&ast_disp1_device);

	res = nvhost_get_resource_byname(&ast_disp2_device,
					 IORESOURCE_MEM, "fbmem");
	res->start = tegra_fb2_start;
	res->end = tegra_fb2_start + tegra_fb2_size - 1;
	if (!err)
		err = nvhost_device_register(&ast_disp2_device);

#if defined(CONFIG_TEGRA_NVAVP)
	if (!err)
		err = nvhost_device_register(&nvavp_device);
#endif

	return err;
}

static int ast_panel_spi_probe(struct spi_device *spi)
{
        int ret;
		#ifdef CONFIG_BACKLIGHT_CLASS_DEVICE
		struct backlight_properties props;
		struct backlight_device *bl;
		#endif

        //PNL_PRINT(("%s: \n",__func__));

        spi->chip_select = 2;
        spi->mode = SPI_MODE_3;
        spi->max_speed_hz = 2000000;		// 2.5M is max speed for TX_CLK=20M, set 2.0M for enough margin
        spi->bits_per_word = 24;
        ret = spi_setup(spi);
        if (ret < 0) {
		dev_err(&spi->dev, "spi_setup failed: %d\n", ret);
		return ret;
        }

        w1_disp1_spi = spi;

	#ifdef CONFIG_DEBUG_FS
	debugfs_spi_panel_root = debugfs_create_dir("panel", NULL);
	debugfs_create_file("write_reg", S_IWUSR,
			debugfs_spi_panel_root,
			w1_disp1_spi, &w_panel_reg_fops);
	debugfs_create_file("read_reg", S_IRUSR | S_IWUSR,
			debugfs_spi_panel_root,
			w1_disp1_spi, &r_panel_reg_fops);
	debugfs_create_file("kernel_msg", S_IWUSR,
			debugfs_spi_panel_root,
			w1_disp1_spi, &w_panel_msg_fops);
	debugfs_spi_bridge_root = debugfs_create_dir("bridge", NULL);
	debugfs_create_file("index", S_IRUSR | S_IWUSR,
			debugfs_spi_bridge_root,
			w1_disp1_spi, &bridge_index_fops);
	debugfs_create_file("register", S_IRUSR | S_IWUSR,
			debugfs_spi_bridge_root,
			w1_disp1_spi, &bridge_register_fops);
        debugfs_spi_brightness_root = debugfs_create_dir("brightness", NULL);
	debugfs_create_file("brightness_level", S_IRUSR | S_IWUSR,
			debugfs_spi_brightness_root,
			w1_disp1_spi, &brightness_level_fops);
	#endif

	#ifdef CONFIG_BACKLIGHT_CLASS_DEVICE
	memset(&props, 0, sizeof(struct backlight_properties));
	props.type = BACKLIGHT_RAW;
	props.max_brightness = MAX_BRIGHTNESS;
	bl = backlight_device_register("pwm-backlight", &spi->dev, NULL,
				       &gamma_backlight_ops, &props);
	if (IS_ERR(bl)) {
		dev_err(&spi->dev, "failed to register backlight\n");
		ret = PTR_ERR(bl);
	} else {
		bl->props.brightness = MAX_BRIGHTNESS;
		backlight_update_status(bl);
		platform_set_drvdata(&ast_backlight_device, bl);
	}

	#endif

	return 0;
}

static int __devexit ast_panel_spi_remove(struct spi_device *spi)
{
	struct backlight_device *bl;

	//PNL_PRINT(("%s: \n",__func__));

	#ifdef CONFIG_DEBUG_FS
	debugfs_remove(debugfs_spi_panel_root);
	debugfs_remove(debugfs_spi_bridge_root);
	debugfs_remove(debugfs_spi_brightness_root);
	#endif

	#ifdef CONFIG_BACKLIGHT_CLASS_DEVICE
	bl = platform_get_drvdata(&ast_backlight_device);
	backlight_device_unregister(bl);
	#endif

	#ifdef ESDON_MIPICLOCK_WORKAROUND
	cancel_delayed_work_sync(&mytimer_workqueue);
	#endif
	return 0;
}

static void ast_panel_spi_shutdown(struct spi_device *spi)
{
	//PNL_PRINT(("%s: \n",__func__));
#if 0 //**JHC**
	ssd2825_config_reg(CFGR_S_WR_DCS_VEN_HS);
        //Panel Power Sequence for power off
        //1.Display off
        s6e8ab0x_display_off();
        //2.Sleep in
        s6e8ab0x_sleep_in();
        mdelay(120);
        //3.SSD2825 power down
        ssd2825_config_reg(CFGR_S_WR_DCS_VDS_LP);
        ssd2825_pll_disable();
        mdelay(100);
#endif
}

static const struct spi_device_id table_id[] = {
	{"spi_panel", 0},
	{}
};

static struct spi_driver panel_spi_driver = {
//static struct platform_driver panel_spi_driver = {
	.driver = {
		.name	= "spi_panel",
		.bus	= &spi_bus_type,
		.owner	= THIS_MODULE,
	},
	.probe = ast_panel_spi_probe,
	.remove = __devexit_p(ast_panel_spi_remove),
	.shutdown = ast_panel_spi_shutdown,
	.id_table = table_id,
};

static int __init ast_panel_spi_init(void){
	int ret;
	//PNL_PRINT(("%s: \n",__func__));

	ret = spi_register_driver(&panel_spi_driver);
	if (unlikely(ret)) {
		pr_err("Could not register %s driver\n",
			panel_spi_driver.driver.name);
		return ret;
	}
	//PNL_PRINT(("Initialized tegra spi driver\n"));
	return 0;
}

static void __exit ast_panel_spi_exit(void){
	//PNL_PRINT(("%s: \n",__func__));
	spi_unregister_driver(&panel_spi_driver);
}

module_init(ast_panel_spi_init);
module_exit(ast_panel_spi_exit);

MODULE_DESCRIPTION("LCD driver");
MODULE_LICENSE("GPL");
