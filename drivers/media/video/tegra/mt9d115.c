/*
 * kernel/drivers/media/video/tegra
 *
 * Aptina MT9D115 sensor driver
 *
 * Copyright (C) 2010 NVIDIA Corporation
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
#include <linux/fs.h>
#include <linux/i2c.h>
#include <linux/miscdevice.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#include <linux/uaccess.h>
#include <media/mt9d115.h>

#include "mt9d115_reg.h"

#define MCU_VAR_ADDR_ADDR               0x098C
#define MCU_VAR_DATA_ADDR               0x0990
#define CHIP_ID_ADDR                    0x0000
#define CHIP_ID_VAL                     0x2580
#define AE_VIRT_GAIN_ADDR               0xA21C
#define AE_R9_ADDR                      0xA222
#define LINE_LENGTH_PCK_ADDR            0x300C
#define FINE_INTEGRATION_TIME_ADDR      0x3014
#define OUTPUT_CLK                      42

static struct mt9d115_info *info;

/*
 * Get the state of mt9d115 indicating led, 0-OFF, ON otherwise
 * defined in board_ast_sensors.c
 */
extern int ast_mt9d115_led_get_state ( void );

static ssize_t mt9d115_led_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int state;

	state = ast_mt9d115_led_get_state();
	return sprintf(buf, "%d\n",state);
}

static ssize_t mt9d115_led_store(struct device *dev,
				 struct device_attribute *attr, const char *buf,
				 size_t count)
{
	long int num;
	int sw_on = 0;

	if (strict_strtol(buf, 0, &num)) {
		dev_err(dev, "\n file: %s, line=%d return %s() ", __FILE__,
			__LINE__, __func__);
		return -EINVAL;
	}

	sw_on = num > 0 ? 1: 0;
	if (info->pdata && info->pdata->led)
		info->pdata->led(sw_on);

	return count;
}

static DEVICE_ATTR(mt9d115_led, S_IWUSR | S_IRUGO, mt9d115_led_show,
		   mt9d115_led_store);

static int mt9d115_write_reg(struct i2c_client *client, u8 *buf, u16 len)
{
	struct i2c_msg msg;
	int retry = 0;

	if (len < 4 || buf == NULL)
		return -EIO;

	msg.addr = client->addr;
	msg.flags = 0;
	msg.len = len;
	msg.buf = buf;
	do {
		if (i2c_transfer(client->adapter, &msg, 1) == 1)
			return 0;

		retry++;
		pr_err("%s : i2c transfer failed, addr: 0x%x%x, len:%u\n",
		       __func__, buf[1], buf[0], len);
		msleep(MT9D115_MAX_WAITMS);
	} while (retry < MT9D115_MAX_RETRIES);

	return -EIO;
}

static int mt9d115_write_reg_help(struct i2c_client *client,
				  const struct mt9d115_reg **table)
{
	int err, index;
	u16 len;
	u8 *buf = NULL;

	err = index = 0;
	if ((*table + 1)->purpose == MT9D115_REG &&
		(*table)->addr + 2 == (*table + 1)->addr) {
		const struct mt9d115_reg *count_reg = *table + 2;

		len = 6;
		while (count_reg->purpose == MT9D115_REG &&
			count_reg->addr == (count_reg - 1)->addr + 2) {
			len += 2;
			count_reg++;
		}
	} else {
		len = 4;
	}

	buf = kmalloc(len, GFP_ATOMIC);
	if (!buf)
		return -ENOMEM;

	buf[index++] = (*table)->addr >> 8;
	buf[index++] = (*table)->addr;
	while ((*table)->purpose == MT9D115_REG) {
		buf[index++] = (*table)->val >> 8;
		buf[index++] = (*table)->val;
		if (index >= len)
			break;

		(*table)++;
	}

	err = mt9d115_write_reg(client, buf, index);
	kfree(buf);

	return err;
}

static int mt9d115_read_reg(struct i2c_client *client, u16 addr)
{
	struct i2c_msg msg[2];
	u16 buf0, buf1;
	int retry = 0;

	if (addr & 0x8000) {
		u8 buf[4];

		buf[0] = (u8) (MCU_VAR_ADDR_ADDR >> 8);
		buf[1] = (u8) MCU_VAR_ADDR_ADDR;
		buf[2] = addr >> 8;
		buf[3] = addr;
		mt9d115_write_reg(client, buf, 4);
		buf0 = swab16(MCU_VAR_DATA_ADDR);
	} else {
		buf0 = swab16(addr);
	}

	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].len = 2;
	msg[0].buf = (u8 *) &buf0;
	msg[1].addr = client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len = 2;
	msg[1].buf = (u8 *) &buf1;
	do {
		if (i2c_transfer(client->adapter, msg, 2) == 2)
			return swab16(buf1);

		retry++;
		pr_err("%s : i2c address 0x%x read failed.\n", __func__, addr);
		msleep(MT9D115_MAX_WAITMS);
	} while (retry < MT9D115_MAX_RETRIES);

	return -EIO;
}

static int mt9d115_poll_reg(struct i2c_client *client, u16 addr, u16 expect_val)
{
	int i;

	for (i = 0; i < MT9D115_POLL_RETRIES; i++) {
		if (mt9d115_read_reg(client, addr) == expect_val)
				return 0;

		msleep(MT9D115_POLL_WAITMS);
	}

	return -ETIME;
}

static int mt9d115_write_table(struct i2c_client *client,
			       const struct mt9d115_reg *table)
{
	int err = 0;
	const struct mt9d115_reg *next;

	for (next = table; next->purpose != MT9D115_TABLE_END; next++) {
		switch (next->purpose) {
		case MT9D115_REG:
			err = mt9d115_write_reg_help(client, &next);
			break;

		case MT9D115_POLL:
			err = mt9d115_poll_reg(client, next->addr, next->val);
			break;

		case MT9D115_WAIT_MS:
			msleep(next->val);
			break;

		default:
			pr_err("%s: invalid operation 0x%x\n", __func__,
			       next->purpose);
			break;
		}

		if (err)
			break;
	}

	return err;
}

static int mt9d115_set_mode(struct mt9d115_info *info,
			    struct mt9d115_mode *mode)
{
	if (mode->xres == 800 && mode->yres == 600) {
		if (info->mode == MT9D115_MODE_PREVIEW)
			return 0;

		info->mode = MT9D115_MODE_PREVIEW;
	} else if (mode->xres == 1600 && mode->yres == 1200) {
		info->mode = MT9D115_MODE_CAPTURE;
	} else {
		pr_err("%s: invalid resolution supplied to set mode %d %d\n",
		       __func__, mode->xres, mode->yres);
		return -EINVAL;
	}

	return mt9d115_write_table(info->i2c_client, mode_table[info->mode]);
}

static long mt9d115_ioctl(struct file *file, unsigned int cmd,
			  unsigned long arg)
{
	struct mt9d115_info *info = file->private_data;
	struct mt9d115_mode mode;
	struct mt9d115_rect rect;
	struct mt9d115_iso *iso;
	int ret, et;

	ret = mutex_lock_interruptible(&info->lock);
	if (ret)
		return ret;

	switch (cmd) {
	case MT9D115_IOCTL_SET_MODE:
		if (copy_from_user(&mode,(const void __user *)arg,
			sizeof(struct mt9d115_mode))) {
			ret = -EFAULT;
			break;
		}

		ret = mt9d115_set_mode(info, &mode);
		break;

	case MT9D115_IOCTL_SET_COLOR_EFFECT:
		switch (arg & MT9D115_COLOR_EFFECT_MASK) {
		case MT9D115_COLOR_EFFECT_NONE:
			ret = mt9d115_write_table(info->i2c_client,
						  color_effect_none);
			break;

		case MT9D115_COLOR_EFFECT_MONO:
			ret = mt9d115_write_table(info->i2c_client,
						  color_effect_mono);
			break;

		case MT9D115_COLOR_EFFECT_SEPIA:
			ret = mt9d115_write_table(info->i2c_client,
						  color_effect_sepia);
			break;

		case MT9D115_COLOR_EFFECT_NEGATIVE:
			ret = mt9d115_write_table(info->i2c_client,
						  color_effect_negative);
			break;

		case MT9D115_COLOR_EFFECT_SOLARIZE:
			ret = mt9d115_write_table(info->i2c_client,
						  color_effect_solarize);
			break;

		default:
			ret = -EINVAL;
			break;
		}

		break;

	case MT9D115_IOCTL_SET_WHITE_BALANCE:
		switch (arg) {
		case MT9D115_WHITE_BALANCE_AUTO:
			ret = mt9d115_write_table(info->i2c_client,
						  white_balance_auto);
			break;

		case MT9D115_WHITE_BALANCE_INCANDESCENT:
			ret = mt9d115_write_table(info->i2c_client,
						  white_balance_incandescent);
			break;

		case MT9D115_WHITE_BALANCE_DAYLIGHT:
			ret = mt9d115_write_table(info->i2c_client,
						  white_balance_daylight);
			break;

		case MT9D115_WHITE_BALANCE_FLUORESCENT:
			ret = mt9d115_write_table(info->i2c_client,
						  white_balance_fluorescent);
			break;

		case MT9D115_WHITE_BALANCE_CLOUDY:
			ret = mt9d115_write_table(info->i2c_client,
						  white_balance_cloudy);
			break;

		default:
			ret = -EINVAL;
			break;
		}

		break;

	case MT9D115_IOCTL_SET_EXPOSURE:
		switch ((int)arg) {
		case MT9D115_EXPOSURE_0:
			ret = mt9d115_write_table(info->i2c_client, exposure_0);
			break;

		case MT9D115_EXPOSURE_PLUS_1:
			ret = mt9d115_write_table(info->i2c_client,
						  exposure_plus_1);
			break;

		case MT9D115_EXPOSURE_PLUS_2:
			ret = mt9d115_write_table(info->i2c_client,
						  exposure_plus_2);
			break;

		case MT9D115_EXPOSURE_MINUS_1:
			ret = mt9d115_write_table(info->i2c_client,
						  exposure_minus_1);
			break;

		case MT9D115_EXPOSURE_MINUS_2:
			ret = mt9d115_write_table(info->i2c_client,
						  exposure_minus_2);
			break;

		default:
			ret = -EINVAL;
			break;
		}

		break;

	case MT9D115_IOCTL_SET_FPS:
		switch (arg) {
		case MT9D115_FPS_MIN:
			ret = mt9d115_write_table(info->i2c_client, fps_min);
			break;

		case MT9D115_FPS_MAX:
			ret = mt9d115_write_table(info->i2c_client, fps_max);
			break;

		case MT9D115_FPS_MID:
		case MT9D115_FPS_DEFAULT:
			ret = mt9d115_write_table(info->i2c_client,
						  fps_default);
			break;

		default:
			ret = -EINVAL;
			break;
		}

		break;

	case MT9D115_IOCTL_GET_ISO:
		ret = mt9d115_read_reg(info->i2c_client, AE_VIRT_GAIN_ADDR);
		if (ret < 0)
			break;

		for (iso = iso_table + 1; iso->value >= 0; iso++) {
			struct mt9d115_iso *pre_iso;

			if (iso->again < ret)
				continue;

			pre_iso = iso - 1;

			ret = (iso->value - pre_iso->value) *
				(ret - pre_iso->again) /
				(iso->again - pre_iso->again) +
				pre_iso->value;
			break;
		}
		if (iso->value == -1)
			ret = (iso - 1)->value;

		break;

	case MT9D115_IOCTL_GET_EXPOSURE_TIME:
		ret = mt9d115_read_reg(info->i2c_client, AE_R9_ADDR);
		if (ret < 0)
			break;

		et = ret * 625;
		ret = mt9d115_read_reg(info->i2c_client, LINE_LENGTH_PCK_ADDR);
		if (ret < 0)
			break;

		et = ret * et;
		ret = mt9d115_read_reg(info->i2c_client,
				       FINE_INTEGRATION_TIME_ADDR);
		if (ret < 0)
			break;

		ret = (ret + et) / OUTPUT_CLK;
		break;

	/* rectangular of exposure */
	case MT9D115_IOCTL_SET_EXPOSURE_RECT:
		if (copy_from_user(&rect, (const void __user *)arg,
			sizeof(struct mt9d115_rect))) {
			ret = -EFAULT;
			break;
		}

		if (!rect.width || !rect.height) {
			if (exposoure_rect[1].val ==
				exposoure_rect_deafult[1].val &&
				exposoure_rect[3].val ==
				exposoure_rect_deafult[3].val)
				break;

			/* set back to deafult */
			exposoure_rect[1].val = exposoure_rect_deafult[1].val;
			exposoure_rect[3].val = exposoure_rect_deafult[3].val;
		} else {
			exposoure_rect[1].val = ((rect.x & 0x00F0) >> 4) |
				(rect.y & 0x00F0);
			exposoure_rect[3].val = ((rect.width & 0x00F0) >> 4) |
				(rect.height & 0x00F0);
		}

		ret = mt9d115_write_table(info->i2c_client, exposoure_rect);
		break;

	default:
		ret = -EINVAL;
	}

	mutex_unlock(&info->lock);

	return ret;
}

static int mt9d115_open(struct inode *inode, struct file *file)
{
	int err = 0;

	file->private_data = info;
	if (info->pdata && info->pdata->power_on)
		info->pdata->power_on();

	err = mt9d115_write_table(info->i2c_client, mode_init);
	if (err)
		return err;

	if (info->pdata && info->pdata->led)
		info->pdata->led(1);

	info->mode = MT9D115_MODE_PREVIEW;

	return 0;
}

static int mt9d115_release(struct inode *inode, struct file *file)
{
	if (info->pdata && info->pdata->power_off)
		info->pdata->power_off();

	if (info->pdata && info->pdata->led)
		info->pdata->led(0);

	file->private_data = NULL;

	return 0;
}


static const struct file_operations mt9d115_fileops = {
	.owner = THIS_MODULE,
	.open = mt9d115_open,
	.unlocked_ioctl = mt9d115_ioctl,
	.release = mt9d115_release,
};

static struct miscdevice mt9d115_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = MT9D115_NAME,
	.fops = &mt9d115_fileops,
};

static int mt9d115_remove(struct i2c_client *client)
{
	struct mt9d115_info *info;

	info = i2c_get_clientdata(client);
	misc_deregister(&mt9d115_device);
	mutex_destroy(&info->lock);
	kfree(info);

	return 0;
}

static int mt9d115_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	int err = 0;

	pr_info("%s\n", __func__);
	info = kzalloc(sizeof(struct mt9d115_info), GFP_KERNEL);
	if (!info) {
		pr_err("%s : Unable to allocate memory!\n", __func__);
		return -ENOMEM;
	}

	info->pdata = client->dev.platform_data;
	info->i2c_client = client;
	i2c_set_clientdata(client, info);
	if (info->pdata && info->pdata->init)
		err = info->pdata->init();

	if (info->pdata && info->pdata->power_on)
		info->pdata->power_on();

	if (mt9d115_read_reg(info->i2c_client, CHIP_ID_ADDR) != CHIP_ID_VAL) {
		pr_err("%s : Unable to read chip ID!\n", __func__);
		err = -ENODEV;
	}

	if (info->pdata && info->pdata->power_off)
		info->pdata->power_off();

	if (err) {
		kfree(info);
		return err;
	}

	err = misc_register(&mt9d115_device);
	if (err) {
		pr_err("mt9d115 : Unable to register misc device!\n");
		kfree(info);
		return err;
	}

	if (device_create_file(&client->dev, &dev_attr_mt9d115_led))
		pr_err("mt9d115 : device create file fail!\n");

	mutex_init(&info->lock);

	return 0;
}

static const struct i2c_device_id mt9d115_id[] = {
	{MT9D115_NAME, 0},
};

MODULE_DEVICE_TABLE(i2c, mt9d115_id);

static struct i2c_driver mt9d115_i2c_driver = {
	.driver = {
		.name = MT9D115_NAME,
		.owner = THIS_MODULE,
	},
	.probe = mt9d115_probe,
	.remove = mt9d115_remove,
	.id_table = mt9d115_id,
};

static int __init mt9d115_init(void)
{
	return i2c_add_driver(&mt9d115_i2c_driver);
}

static void __exit mt9d115_exit(void)
{
	i2c_del_driver(&mt9d115_i2c_driver);
}

module_init(mt9d115_init);
module_exit(mt9d115_exit);
