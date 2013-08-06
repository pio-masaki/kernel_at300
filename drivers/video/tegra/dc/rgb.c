/*
 * drivers/video/tegra/dc/rgb.c
 *
 * Copyright (C) 2010 Google, Inc.
 * Author: Erik Gilling <konkers@android.com>
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

#include <mach/dc.h>
#include <mach/fb.h>
#include <asm/mach-types.h>

#include "dc_reg.h"
#include "dc_priv.h"
#include "edid.h"

#define DTD_H_SIZE      0x42
#define DTD_V_SIZE      0x43

static const u32 tegra_dc_rgb_enable_partial_pintable[] = {
	DC_COM_PIN_OUTPUT_ENABLE0,	0x00000000,
	DC_COM_PIN_OUTPUT_ENABLE1,	0x00000000,
	DC_COM_PIN_OUTPUT_ENABLE2,	0x00000000,
	DC_COM_PIN_OUTPUT_ENABLE3,	0x00000000,
	DC_COM_PIN_OUTPUT_POLARITY0,	0x00000000,
	DC_COM_PIN_OUTPUT_POLARITY2,	0x00000000,
	DC_COM_PIN_OUTPUT_DATA0,	0x00000000,
	DC_COM_PIN_OUTPUT_DATA1,	0x00000000,
	DC_COM_PIN_OUTPUT_DATA2,	0x00000000,
	DC_COM_PIN_OUTPUT_DATA3,	0x00000000,
};

static const u32 tegra_dc_rgb_enable_pintable[] = {
	DC_COM_PIN_OUTPUT_ENABLE0,	0x00000000,
	DC_COM_PIN_OUTPUT_ENABLE1,	0x00000000,
	DC_COM_PIN_OUTPUT_ENABLE2,	0x00000000,
	DC_COM_PIN_OUTPUT_ENABLE3,	0x00000000,
	DC_COM_PIN_OUTPUT_POLARITY0,	0x00000000,
	DC_COM_PIN_OUTPUT_POLARITY1,	0x01000000,
	DC_COM_PIN_OUTPUT_POLARITY2,	0x00000000,
	DC_COM_PIN_OUTPUT_POLARITY3,	0x00000000,
	DC_COM_PIN_OUTPUT_DATA0,	0x00000000,
	DC_COM_PIN_OUTPUT_DATA1,	0x00000000,
	DC_COM_PIN_OUTPUT_DATA2,	0x00000000,
	DC_COM_PIN_OUTPUT_DATA3,	0x00000000,
};

static const u32 tegra_dc_rgb_enable_out_sel_pintable[] = {
	DC_COM_PIN_OUTPUT_SELECT0,	0x00000000,
	DC_COM_PIN_OUTPUT_SELECT1,	0x00000000,
	DC_COM_PIN_OUTPUT_SELECT2,	0x00000000,
#ifdef CONFIG_TEGRA_SILICON_PLATFORM
	DC_COM_PIN_OUTPUT_SELECT3,	0x00000000,
#else
	/* The display panel sub-board used on FPGA platforms (panel 86)
	   is non-standard. It expects the Data Enable signal on the WR
	   pin instead of the DE pin. */
	DC_COM_PIN_OUTPUT_SELECT3,	0x00200000,
#endif
	DC_COM_PIN_OUTPUT_SELECT4,	0x00210222,
	DC_COM_PIN_OUTPUT_SELECT5,	0x00002200,
	DC_COM_PIN_OUTPUT_SELECT6,	0x00020000,
};

static const u32 tegra_dc_rgb_disable_pintable[] = {
	DC_COM_PIN_OUTPUT_ENABLE0,	0x55555555,
	DC_COM_PIN_OUTPUT_ENABLE1,	0x55150005,
	DC_COM_PIN_OUTPUT_ENABLE2,	0x55555555,
	DC_COM_PIN_OUTPUT_ENABLE3,	0x55555555,
	DC_COM_PIN_OUTPUT_POLARITY0,	0x00000000,
	DC_COM_PIN_OUTPUT_POLARITY1,	0x00000000,
	DC_COM_PIN_OUTPUT_POLARITY2,	0x00000000,
	DC_COM_PIN_OUTPUT_POLARITY3,	0x00000000,
	DC_COM_PIN_OUTPUT_DATA0,	0xaaaaaaaa,
	DC_COM_PIN_OUTPUT_DATA1,	0xaaaaaaaa,
	DC_COM_PIN_OUTPUT_DATA2,	0xaaaaaaaa,
	DC_COM_PIN_OUTPUT_DATA3,	0xaaaaaaaa,
	DC_COM_PIN_OUTPUT_SELECT0,	0x00000000,
	DC_COM_PIN_OUTPUT_SELECT1,	0x00000000,
	DC_COM_PIN_OUTPUT_SELECT2,	0x00000000,
	DC_COM_PIN_OUTPUT_SELECT3,	0x00000000,
	DC_COM_PIN_OUTPUT_SELECT4,	0x00000000,
	DC_COM_PIN_OUTPUT_SELECT5,	0x00000000,
	DC_COM_PIN_OUTPUT_SELECT6,	0x00000000,
};

static int tegra_dc_rgb_init(struct tegra_dc *dc)
{
	struct tegra_edid *edid;
	struct tegra_dc_edid *dc_edid;
	struct fb_monspecs specs;
	int err;

	if (dc->out == NULL || dc->out->dcc_bus < 0)
		return 0;

	edid = tegra_edid_create(dc->out->dcc_bus);
	if (IS_ERR_OR_NULL(edid)) {
		dev_err(&dc->ndev->dev, "rgb: can't create edid\n");
		return PTR_ERR(edid);
	}

	err = tegra_edid_get_monspecs(edid, &specs);
	if (err < 0) {
		dev_err(&dc->ndev->dev, "error reading edid\n");
		return err;
	}

	if (dc->pdata->default_out->n_modes > 0) {
		dc->mode.h_sync_width = specs.modedb->hsync_len;
		dc->mode.v_sync_width = specs.modedb->vsync_len;
		dc->mode.h_back_porch = specs.modedb->left_margin;
		dc->mode.v_back_porch = specs.modedb->upper_margin;
		dc->mode.h_active = specs.modedb->xres;
		dc->mode.v_active = specs.modedb->yres;
		dc->mode.h_front_porch = specs.modedb->right_margin;
		if (!machine_is_avalon() && !machine_is_titan())
			dc->mode.pclk = PICOS2KHZ(specs.modedb->pixclock) * 1000;
		if (!machine_is_avalon())
			dc->mode.v_front_porch = specs.modedb->lower_margin;
	}

	if (dc->pdata->fb) {
		dc->pdata->fb->xres = specs.modedb->xres;
		dc->pdata->fb->yres = specs.modedb->yres;
	}

	dc_edid = tegra_edid_get_data(edid);
	if (IS_ERR_OR_NULL(dc_edid))
		return PTR_ERR(dc_edid);

	dc->out->width = dc_edid->buf[DTD_H_SIZE];
	dc->out->height = dc_edid->buf[DTD_V_SIZE];
	tegra_edid_put_data(dc_edid);
	if (specs.modedb->xres > specs.modedb->yres) {
		if (dc->out->width <= dc->out->height)
			dc->out->width += 0xFF;
	} else {
		if (dc->out->width > dc->out->height)
			dc->out->height += 0xFF;
	}

	fb_destroy_modedb(specs.modedb);

	return 0;
}

void tegra_dc_rgb_enable(struct tegra_dc *dc)
{
	int i;
	u32 out_sel_pintable[ARRAY_SIZE(tegra_dc_rgb_enable_out_sel_pintable)];

	tegra_dc_writel(dc, PW0_ENABLE | PW1_ENABLE | PW2_ENABLE | PW3_ENABLE |
			PW4_ENABLE | PM0_ENABLE | PM1_ENABLE,
			DC_CMD_DISPLAY_POWER_CONTROL);

	tegra_dc_writel(dc, DISP_CTRL_MODE_C_DISPLAY, DC_CMD_DISPLAY_COMMAND);

	if (dc->out->out_pins) {
		tegra_dc_set_out_pin_polars(dc, dc->out->out_pins,
			dc->out->n_out_pins);
		tegra_dc_write_table(dc, tegra_dc_rgb_enable_partial_pintable);
	} else {
		tegra_dc_write_table(dc, tegra_dc_rgb_enable_pintable);
	}

	memcpy(out_sel_pintable, tegra_dc_rgb_enable_out_sel_pintable,
		sizeof(tegra_dc_rgb_enable_out_sel_pintable));

	if (dc->out && dc->out->out_sel_configs) {
		u8 *out_sels = dc->out->out_sel_configs;
		for (i = 0; i < dc->out->n_out_sel_configs; i++) {
			switch (out_sels[i]) {
			case TEGRA_PIN_OUT_CONFIG_SEL_LM1_M1:
				out_sel_pintable[5*2+1] =
					(out_sel_pintable[5*2+1] &
					~PIN5_LM1_LCD_M1_OUTPUT_MASK) |
					PIN5_LM1_LCD_M1_OUTPUT_M1;
				break;
			case TEGRA_PIN_OUT_CONFIG_SEL_LM1_LD21:
				out_sel_pintable[5*2+1] =
					(out_sel_pintable[5*2+1] &
					~PIN5_LM1_LCD_M1_OUTPUT_MASK) |
					PIN5_LM1_LCD_M1_OUTPUT_LD21;
				break;
			case TEGRA_PIN_OUT_CONFIG_SEL_LM1_PM1:
				out_sel_pintable[5*2+1] =
					(out_sel_pintable[5*2+1] &
					~PIN5_LM1_LCD_M1_OUTPUT_MASK) |
					PIN5_LM1_LCD_M1_OUTPUT_PM1;
				break;
			default:
				dev_err(&dc->ndev->dev,
					"Invalid pin config[%d]: %d\n",
					 i, out_sels[i]);
				break;
			}
		}
	}

	tegra_dc_write_table(dc, out_sel_pintable);
}

void tegra_dc_rgb_disable(struct tegra_dc *dc)
{
	tegra_dc_writel(dc, 0x00000000, DC_CMD_DISPLAY_POWER_CONTROL);

	tegra_dc_write_table(dc, tegra_dc_rgb_disable_pintable);
}

struct tegra_dc_out_ops tegra_dc_rgb_ops = {
	.init = tegra_dc_rgb_init,
	.enable = tegra_dc_rgb_enable,
	.disable = tegra_dc_rgb_disable,
};

