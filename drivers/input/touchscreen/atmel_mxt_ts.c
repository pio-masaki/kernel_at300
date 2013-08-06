/*
 * Atmel maXTouch Touchscreen driver
 *
 * Copyright (C) 2010 Samsung Electronics Co.Ltd
 * Copyright (C) 2011 Atmel Corporation
 * Copyright (C) 2011-2012 NVIDIA Corporation
 * Author: Joonyoung Shim <jy0922.shim@samsung.com>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/i2c.h>
#include <linux/i2c/atmel_mxt_ts.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#if defined(CONFIG_HAS_EARLYSUSPEND)
#include <linux/earlysuspend.h>
#endif
#include <asm/mach-types.h>
#include <linux/debugfs.h>
#include <linux/uaccess.h>
#include <linux/cdev.h>
#include <linux/gpio.h>

/* Family ID */
#define MXT224_ID		0x80
#define MXT768E_ID		0xA1
#define MXT1386_ID		0xA0

/* Version */
#define MXT_VER_20		20
#define MXT_VER_21		21
#define MXT_VER_22		22

/* Slave addresses */
#define MXT_APP_LOW		0x4a
#define MXT_APP_HIGH		0x4b
#define MXT_BOOT_LOW		0x24
#define MXT_BOOT_HIGH		0x25

/* Firmware */
#define MXT_FW_NAME		"maxtouch.fw"

/* Registers */
#define MXT_FAMILY_ID		0x00
#define MXT_VARIANT_ID		0x01
#define MXT_VERSION		0x02
#define MXT_BUILD		0x03
#define MXT_MATRIX_X_SIZE	0x04
#define MXT_MATRIX_Y_SIZE	0x05
#define MXT_OBJECT_NUM		0x06
#define MXT_OBJECT_START	0x07

#define MXT_OBJECT_SIZE		6

/* Object types */
#define MXT_DEBUG_DIAGNOSTIC_T37		37
#define MXT_GEN_MESSAGE_T5			5
#define MXT_GEN_COMMAND_T6			6
#define MXT_GEN_POWER_T7			7
#define MXT_GEN_ACQUIRE_T8			8
#define MXT_GEN_DATASOURCE_T53			53
#define MXT_TOUCH_MULTI_T9			9
#define MXT_TOUCH_KEYARRAY_T15			15
#define MXT_TOUCH_PROXIMITY_T23			23
#define MXT_TOUCH_PROXKEY_T52			52
#define MXT_PROCI_GRIPFACE_T20			20
#define MXT_PROCG_NOISE_T22			22
#define MXT_PROCI_ONETOUCH_T24			24
#define MXT_PROCI_TWOTOUCH_T27			27
#define MXT_PROCI_GRIP_T40			40
#define MXT_PROCI_PALM_T41			41
#define MXT_PROCI_TOUCHSUPPRESSION_T42		42
#define MXT_PROCI_STYLUS_T47			47
#define MXT_PROCG_NOISESUPPRESSION_T48		48
#define MXT_PROCI_ADAPTIVETHRESHOLD_T55		55
#define MXT_PROCI_SHIELDLESS_T56		56
#define MXT_PROCI_EXTRATOUCHSCREENDATA_T57	57
#define MXT_SPT_COMMSCONFIG_T18			18
#define MXT_SPT_GPIOPWM_T19			19
#define MXT_SPT_SELFTEST_T25			25
#define MXT_SPT_CTECONFIG_T28			28
#define MXT_SPT_USERDATA_T38			38
#define MXT_SPT_DIGITIZER_T43			43
#define MXT_SPT_MESSAGECOUNT_T44		44
#define MXT_SPT_CTECONFIG_T46			46
#define MXT_PROCG_NOISESUPPRESSION_T62	62

/* MXT_DEBUG_DIAGNOSTIC_T37 field */
#define MXT_ADR_T37_MODE	0x00
#define MXT_ADR_T37_PAGE	0x01
#define	MXT_ADR_T37_DATA	0x02

/* MXT_GEN_COMMAND_T6 field */
#define MXT_COMMAND_RESET	0
#define MXT_COMMAND_BACKUPNV	1
#define MXT_COMMAND_CALIBRATE	2
#define MXT_COMMAND_REPORTALL	3
#define MXT_COMMAND_DIAGNOSTIC	5

/* MXT_GEN_POWER_T7 field */
#define MXT_POWER_IDLEACQINT	0
#define MXT_POWER_ACTVACQINT	1
#define MXT_POWER_ACTV2IDLETO	2

/* MXT_GEN_ACQUIRE_T8 field */
#define MXT_ACQUIRE_CHRGTIME	0
#define MXT_ACQUIRE_TCHDRIFT	2
#define MXT_ACQUIRE_DRIFTST	3
#define MXT_ACQUIRE_TCHAUTOCAL	4
#define MXT_ACQUIRE_SYNC	5
#define MXT_ACQUIRE_ATCHCALST	6
#define MXT_ACQUIRE_ATCHCALSTHR	7

#define MXT_ACQUIRE_ATCHFRCCALST	8
#define MXT_ACQUIRE_ATCHFRCCALRATIO	9

/* MXT_TOUCH_MULTI_T9 field */
#define MXT_TOUCH_CTRL		0
#define MXT_TOUCH_XORIGIN	1
#define MXT_TOUCH_YORIGIN	2
#define MXT_TOUCH_XSIZE		3
#define MXT_TOUCH_YSIZE		4
#define MXT_TOUCH_BLEN		6
#define MXT_TOUCH_TCHTHR	7
#define MXT_TOUCH_TCHDI		8
#define MXT_TOUCH_ORIENT	9
#define MXT_TOUCH_MOVHYSTI	11
#define MXT_TOUCH_MOVHYSTN	12
#define MXT_TOUCH_NUMTOUCH	14
#define MXT_TOUCH_MRGHYST	15
#define MXT_TOUCH_MRGTHR	16
#define MXT_TOUCH_AMPHYST	17
#define MXT_TOUCH_XRANGE_LSB	18
#define MXT_TOUCH_XRANGE_MSB	19
#define MXT_TOUCH_YRANGE_LSB	20
#define MXT_TOUCH_YRANGE_MSB	21
#define MXT_TOUCH_XLOCLIP	22
#define MXT_TOUCH_XHICLIP	23
#define MXT_TOUCH_YLOCLIP	24
#define MXT_TOUCH_YHICLIP	25
#define MXT_TOUCH_XEDGECTRL	26
#define MXT_TOUCH_XEDGEDIST	27
#define MXT_TOUCH_YEDGECTRL	28
#define MXT_TOUCH_YEDGEDIST	29
#define MXT_TOUCH_JUMPLIMIT	30

/* MXT_PROCI_GRIPFACE_T20 field */
#define MXT_GRIPFACE_CTRL	0
#define MXT_GRIPFACE_XLOGRIP	1
#define MXT_GRIPFACE_XHIGRIP	2
#define MXT_GRIPFACE_YLOGRIP	3
#define MXT_GRIPFACE_YHIGRIP	4
#define MXT_GRIPFACE_MAXTCHS	5
#define MXT_GRIPFACE_SZTHR1	7
#define MXT_GRIPFACE_SZTHR2	8
#define MXT_GRIPFACE_SHPTHR1	9
#define MXT_GRIPFACE_SHPTHR2	10
#define MXT_GRIPFACE_SUPEXTTO	11

/* MXT_PROCI_NOISE field */
#define MXT_NOISE_CTRL		0
#define MXT_NOISE_OUTFLEN	1
#define MXT_NOISE_GCAFUL_LSB	3
#define MXT_NOISE_GCAFUL_MSB	4
#define MXT_NOISE_GCAFLL_LSB	5
#define MXT_NOISE_GCAFLL_MSB	6
#define MXT_NOISE_ACTVGCAFVALID	7
#define MXT_NOISE_NOISETHR	8
#define MXT_NOISE_FREQHOPSCALE	10
#define MXT_NOISE_FREQ0		11
#define MXT_NOISE_FREQ1		12
#define MXT_NOISE_FREQ2		13
#define MXT_NOISE_FREQ3		14
#define MXT_NOISE_FREQ4		15
#define MXT_NOISE_IDLEGCAFVALID	16

/* MXT_SPT_COMMSCONFIG_T18 */
#define MXT_COMMS_CTRL		0
#define MXT_COMMS_CMD		1

/* MXT_SPT_CTECONFIG_T28 field */
#define MXT_CTE_CTRL		0
#define MXT_CTE_CMD		1
#define MXT_CTE_MODE		2
#define MXT_CTE_IDLEGCAFDEPTH	3
#define MXT_CTE_ACTVGCAFDEPTH	4
#define MXT_CTE_VOLTAGE		5

/* MXT_SPT_SELFTEST_T25 field */
#define	MXT_ADDR_T25_CTRL				0
#define	MXT_ADDR_T25_CMD				1
#define	MXT_ADDR_T25_HISIGLIM_L				2
#define	MXT_ADDR_T25_HISIGLIM_H				3
#define	MXT_ADDR_T25_LOSIGLIM_L				4
#define	MXT_ADDR_T25_LOSIGLIM_H				5

#define MXT_VOLTAGE_DEFAULT	2700000
#define MXT_VOLTAGE_STEP	10000

/* Defines for MXT_TOUCH_CTRL */
#define MXT_TOUCH_DISABLE	0
#define MXT_TOUCH_ENABLE	0x83

/* Define for MXT_GEN_COMMAND_T6 */
#define MXT_BOOT_VALUE		0xa5
#define MXT_BACKUP_VALUE	0x55
#define MXT_BACKUP_TIME		200	/* msec */
#define MXT224_RESET_TIME	65	/* msec */
#define MXT768E_RESET_TIME	250	/* msec */
#define MXT1386_RESET_TIME	200	/* msec */
#define MXT_RESET_TIME		200	/* msec */
#define MXT_RESET_NOCHGREAD	400	/* msec */
/* Define for T6 Debug Diagnostics Commands */
#define	MXT_CMD_T6_PAGE_UP				0x01
#define	MXT_CMD_T6_PAGE_DOWN				0x02
#define	MXT_CMD_T6_DELTAS_MODE				0x10
#define	MXT_CMD_T6_REFERENCES_MODE			0x11
#define	MXT_CMD_T6_CTE_MODE				0x31

#define MXT_WAKEUP_TIME		25	/* msec */

#define MXT_FWRESET_TIME	175	/* msec */

/* Defines for MXT_SLOWSCAN_EXTENSIONS */
#define SLOSCAN_DISABLE		0	/* Disable slow scan */
#define SLOSCAN_ENABLE		1	/* Enable slow scan */
#define SLOSCAN_SET_ACTVACQINT	2	/* Set ACTV scan rate */
#define SLOSCAN_SET_IDLEACQINT	3	/* Set IDLE scan rate */
#define SLOSCAN_SET_ACTV2IDLETO 4	/* Set the ACTIVE to IDLE TimeOut */

/* Command to unlock bootloader */
#define MXT_UNLOCK_CMD_MSB	0xaa
#define MXT_UNLOCK_CMD_LSB	0xdc

/* Bootloader mode status */
#define MXT_WAITING_BOOTLOAD_CMD	0xc0	/* valid 7 6 bit only */
#define MXT_WAITING_FRAME_DATA	0x80	/* valid 7 6 bit only */
#define MXT_FRAME_CRC_CHECK	0x02
#define MXT_FRAME_CRC_FAIL	0x03
#define MXT_FRAME_CRC_PASS	0x04
#define MXT_APP_CRC_FAIL	0x40	/* valid 7 8 bit only */
#define MXT_BOOT_STATUS_MASK	0x3f

/* Touch status */
#define MXT_SUPPRESS		(1 << 1)
#define MXT_AMP			(1 << 2)
#define MXT_VECTOR		(1 << 3)
#define MXT_MOVE		(1 << 4)
#define MXT_RELEASE		(1 << 5)
#define MXT_PRESS		(1 << 6)
#define MXT_DETECT		(1 << 7)

/* Touch orient bits */
#define MXT_XY_SWITCH		(1 << 0)
#define MXT_X_INVERT		(1 << 1)
#define MXT_Y_INVERT		(1 << 2)

/* Touchscreen absolute values */
#define MXT_MAX_AREA		0xff

/* Fixed Report ID values */
#define MXT_RPTID_NOMSG		0xFF	/* No messages available to read */

#define MXT_MAX_FINGER		10

#define RESUME_READS		100

#define	MXT_CMD_T25_ALL_TEST	0xFE
#define	MXT_MSGR_T25_OK		0xFE
#define I2C_PAYLOAD_SIZE 	254

/* IOCTL commands */
#define MXT_SET_ADDRESS		1	/* Sets the internal address pointer */
#define MXT_RESET		2	/* Resets the device */
#define MXT_CALIBRATE		3	/* Calibrates the device */
#define MXT_BACKUP		4	/* Backups the current state of registers to
					   NVM */
#define MXT_NONTOUCH_MSG	5	/* Only non-touch messages can be read from
					   the message buffer
					   (/dev/maXTouch_messages) */
#define MXT_ALL_MSG		6	/* All messages can be read from the message
					   buffer */
#define AUTO_CAL_TIMEOUT	10	/* SEC */

static int bootloader_mode = 0;
bool is_cfg_format_enabled = true;

enum T25_self_test_result {
	e_self_test_none,
	e_self_test_fail,
	e_self_test_pass,
};


struct mxt_info {
	u8 family_id;
	u8 variant_id;
	u8 version;
	u8 build;
	u8 matrix_xsize;
	u8 matrix_ysize;
	u8 object_num;
};

struct mxt_object {
	u8 type;
	u16 start_address;
	u16 size;
	u16 instances;
	u8 num_report_ids;

	/* to map object and message */
	u8 min_reportid;
	u8 max_reportid;
};

struct mxt_message {
	u8 reportid;
	u8 message[7];
	u8 checksum;
};

struct mxt_finger {
	int status;
	int x;
	int y;
	int area;
};

/* Each client has this additional data */
struct mxt_data {
	struct i2c_client *client;
	struct input_dev *input_dev;
	const struct mxt_platform_data *pdata;
	struct mxt_object *object_table;
	struct mxt_info info;
	struct mxt_finger finger[MXT_MAX_FINGER];
	unsigned int irq;
	unsigned int max_x;
	unsigned int max_y;
	u8(*read_chg) (void);
	u16 msg_address;
	u16 last_address;
	u8 actv_cycle_time;
	u8 idle_cycle_time;
	u8 actv2idle_timeout;
	u8 is_stopped;
	struct mutex access_mutex;
#if defined(CONFIG_HAS_EARLYSUSPEND)
	struct early_suspend early_suspend;
#endif
	unsigned int driver_paused;
	struct bin_attribute mem_access_attr;
	int debug_enabled;
	int slowscan_enabled;
	u8 slowscan_actv_cycle_time;
	u8 slowscan_idle_cycle_time;
	u8 slowscan_actv2idle_timeout;
	u8 slowscan_shad_actv_cycle_time;
	u8 slowscan_shad_idle_cycle_time;
	u8 slowscan_shad_actv2idle_timeout;

	/* debugfs variables */
	struct dentry *debug_dir;
	int current_debug_datap;
	struct mutex debug_mutex;
	u16 *debug_data;
	u8 self_test_result;

	/* Character device variables */
	struct cdev cdev;
	struct cdev cdev_messages;	/* 2nd Char dev for messages */
	dev_t dev_num;
	struct class *mxt_class;
	u16 address_pointer;

	/* for sphinx */
	struct delayed_work dwork;

	struct delayed_work dwork_no_auto_cal;
};

#if defined(CONFIG_HAS_EARLYSUSPEND)
static void mxt_early_suspend(struct early_suspend *es);
static void mxt_early_resume(struct early_suspend *es);
#endif

static bool mxt_object_readable(unsigned int type)
{
	switch (type) {
	case MXT_GEN_MESSAGE_T5:
	case MXT_GEN_COMMAND_T6:
	case MXT_GEN_POWER_T7:
	case MXT_GEN_ACQUIRE_T8:
	case MXT_GEN_DATASOURCE_T53:
	case MXT_TOUCH_MULTI_T9:
	case MXT_TOUCH_KEYARRAY_T15:
	case MXT_TOUCH_PROXIMITY_T23:
	case MXT_TOUCH_PROXKEY_T52:
	case MXT_PROCI_GRIPFACE_T20:
	case MXT_PROCG_NOISE_T22:
	case MXT_PROCI_ONETOUCH_T24:
	case MXT_PROCI_TWOTOUCH_T27:
	case MXT_PROCI_GRIP_T40:
	case MXT_PROCI_PALM_T41:
	case MXT_PROCI_TOUCHSUPPRESSION_T42:
	case MXT_PROCI_STYLUS_T47:
	case MXT_PROCG_NOISESUPPRESSION_T48:
	case MXT_PROCI_ADAPTIVETHRESHOLD_T55:
	case MXT_PROCI_SHIELDLESS_T56:
	case MXT_PROCI_EXTRATOUCHSCREENDATA_T57:
	case MXT_SPT_COMMSCONFIG_T18:
	case MXT_SPT_GPIOPWM_T19:
	case MXT_SPT_SELFTEST_T25:
	case MXT_SPT_CTECONFIG_T28:
	case MXT_SPT_USERDATA_T38:
	case MXT_SPT_DIGITIZER_T43:
	case MXT_SPT_CTECONFIG_T46:
	case MXT_PROCG_NOISESUPPRESSION_T62:
		return true;
	default:
		return false;
	}
}

static bool mxt_object_writable(unsigned int type)
{
	switch (type) {
#if 0
	case MXT_GEN_COMMAND_T6:
#endif
	case MXT_GEN_POWER_T7:
	case MXT_GEN_ACQUIRE_T8:
	case MXT_TOUCH_MULTI_T9:
	case MXT_TOUCH_KEYARRAY_T15:
	case MXT_TOUCH_PROXIMITY_T23:
	case MXT_TOUCH_PROXKEY_T52:
	case MXT_PROCI_GRIPFACE_T20:
	case MXT_PROCG_NOISE_T22:
	case MXT_PROCI_ONETOUCH_T24:
	case MXT_PROCI_TWOTOUCH_T27:
	case MXT_PROCI_GRIP_T40:
	case MXT_PROCI_PALM_T41:
	case MXT_PROCI_TOUCHSUPPRESSION_T42:
	case MXT_PROCI_STYLUS_T47:
	case MXT_PROCG_NOISESUPPRESSION_T48:
	case MXT_PROCI_ADAPTIVETHRESHOLD_T55:
	case MXT_PROCI_SHIELDLESS_T56:
	case MXT_PROCI_EXTRATOUCHSCREENDATA_T57:
	case MXT_SPT_COMMSCONFIG_T18:
	case MXT_SPT_GPIOPWM_T19:
	case MXT_SPT_SELFTEST_T25:
	case MXT_SPT_CTECONFIG_T28:
	case MXT_SPT_DIGITIZER_T43:
	case MXT_SPT_CTECONFIG_T46:
	case MXT_PROCG_NOISESUPPRESSION_T62:
		return true;
	default:
		return false;
	}
}

static void mxt_dump_message(struct device *dev,
				  struct mxt_message *message)
{
	dev_dbg(dev, "reportid:\t0x%x\n", message->reportid);
	dev_dbg(dev, "message1:\t0x%x\n", message->message[0]);
	dev_dbg(dev, "message2:\t0x%x\n", message->message[1]);
	dev_dbg(dev, "message3:\t0x%x\n", message->message[2]);
	dev_dbg(dev, "message4:\t0x%x\n", message->message[3]);
	dev_dbg(dev, "message5:\t0x%x\n", message->message[4]);
	dev_dbg(dev, "message6:\t0x%x\n", message->message[5]);
	dev_dbg(dev, "message7:\t0x%x\n", message->message[6]);
	dev_dbg(dev, "checksum:\t0x%x\n", message->checksum);
}

static int mxt_check_bootloader(struct i2c_client *client,
				     unsigned int state)
{
	u8 val;

recheck:
	if (i2c_master_recv(client, &val, 1) != 1) {
		dev_err(&client->dev, "%s: i2c recv failed\n", __func__);
		return -EIO;
	}

	switch (state) {
	case MXT_WAITING_BOOTLOAD_CMD:
	case MXT_WAITING_FRAME_DATA:
		val &= ~MXT_BOOT_STATUS_MASK;
		break;
	case MXT_FRAME_CRC_PASS:
		if (val == MXT_FRAME_CRC_CHECK)
			goto recheck;
		break;
	default:
		return -EINVAL;
	}

	if (val != state && val != MXT_APP_CRC_FAIL) {
		dev_err(&client->dev, "Unvalid bootloader mode state. state = 0x%02X\n", val);
		return -EINVAL;
	}

	return 0;
}

static int mxt_unlock_bootloader(struct i2c_client *client)
{
	u8 buf[2];

	buf[0] = MXT_UNLOCK_CMD_LSB;
	buf[1] = MXT_UNLOCK_CMD_MSB;

	if (i2c_master_send(client, buf, 2) != 2) {
		dev_err(&client->dev, "%s: i2c send failed\n", __func__);
		return -EIO;
	}

	return 0;
}

static int mxt_fw_write(struct i2c_client *client,
			     const u8 *data, unsigned int frame_size)
{
	if (i2c_master_send(client, data, frame_size) != frame_size) {
		dev_err(&client->dev, "%s: i2c send failed\n", __func__);
		return -EIO;
	}

	return 0;
}

static int __mxt_read_reg(struct i2c_client *client,
			       u16 reg, u16 len, void *val)
{
	u8 buf[2];
	int retval = 0;
	struct mxt_data *data = i2c_get_clientdata(client);

	buf[0] = reg & 0xff;
	buf[1] = (reg >> 8) & 0xff;

	mutex_lock(&data->access_mutex);

	if ((data->last_address != reg) || (reg != data->msg_address)) {
		if (i2c_master_send(client, (u8 *)buf, 2) != 2) {
			dev_dbg(&client->dev, "i2c retry\n");
			msleep(MXT_WAKEUP_TIME);

			if (i2c_master_send(client, (u8 *)buf, 2) != 2) {
				dev_err(&client->dev, "%s: i2c send failed\n",
					__func__);
				retval = -EIO;
				goto mxt_read_exit;
			}
		}
	}

	if (i2c_master_recv(client, (u8 *)val, len) != len) {
		dev_dbg(&client->dev, "i2c retry\n");
		msleep(MXT_WAKEUP_TIME);

		if (i2c_master_recv(client, (u8 *)val, len) != len) {
			dev_err(&client->dev, "%s: i2c recv failed\n",
				__func__);
			retval = -EIO;
			goto mxt_read_exit;
		}
	}

	data->last_address = reg;

mxt_read_exit:
	mutex_unlock(&data->access_mutex);
	return retval;
}

static int mxt_read_reg(struct i2c_client *client, u16 reg, u8 *val)
{
	return __mxt_read_reg(client, reg, 1, val);
}

static int mxt_write_reg(struct i2c_client *client, u16 reg, u8 val)
{
	u8 buf[3];
	int retval = 0;
	struct mxt_data *data = i2c_get_clientdata(client);

	buf[0] = reg & 0xff;
	buf[1] = (reg >> 8) & 0xff;
	buf[2] = val;

	mutex_lock(&data->access_mutex);
	if (i2c_master_send(client, buf, 3) != 3) {
		dev_dbg(&client->dev, "i2c retry\n");
		msleep(MXT_WAKEUP_TIME);

		if (i2c_master_send(client, buf, 3) != 3) {
			dev_err(&client->dev, "%s: i2c send failed\n", __func__);
			retval = -EIO;
			goto mxt_write_exit;
		}
	}
	data->last_address = reg + 1;

mxt_write_exit:
	mutex_unlock(&data->access_mutex);
	return retval;
}

static int mxt_read_object_table(struct i2c_client *client,
				      u16 reg, u8 *object_buf)
{
	return __mxt_read_reg(client, reg, MXT_OBJECT_SIZE,
				   object_buf);
}

static struct mxt_object *
mxt_get_object(struct mxt_data *data, u8 type)
{
	struct mxt_object *object;
	int i;

	for (i = 0; i < data->info.object_num; i++) {
		object = data->object_table + i;
		if (object->type == type)
			return object;
	}

	dev_err(&data->client->dev, "Invalid object type T%d\n", type);
	return NULL;
}

static int mxt_read_message(struct mxt_data *data,
				 struct mxt_message *message)
{
	struct mxt_object *object;
	u16 reg;

	object = mxt_get_object(data, MXT_GEN_MESSAGE_T5);
	if (!object)
		return -EINVAL;

	reg = object->start_address;
	return __mxt_read_reg(data->client, reg,
			sizeof(struct mxt_message), message);
}

static int mxt_read_object(struct mxt_data *data,
				u8 type, u8 offset, u8 *val)
{
	struct mxt_object *object;
	u16 reg;

	object = mxt_get_object(data, type);
	if (!object)
		return -EINVAL;

	reg = object->start_address;
	return __mxt_read_reg(data->client, reg + offset, 1, val);
}

static int mxt_write_object(struct mxt_data *data,
				 u8 type, u8 offset, u8 val)
{
	struct mxt_object *object;
	u16 reg;

	object = mxt_get_object(data, type);
	if (!object)
		return -EINVAL;

	reg = object->start_address;
	return mxt_write_reg(data->client, reg + offset, val);
}

/*********************************************************************************************/
int mxt_write_block(struct i2c_client *client, u16 addr, u16 length, u8 *value);

ssize_t debug_data_read(struct mxt_data *mxt, char *buf, size_t count,
			loff_t *ppos, u8 debug_command)
{
	int i;
	u16 *data;
	u16 diagnostics_reg;
	int offset = 0;
	int size;
	int read_size;
	int error;
	char *buf_start;
	u16 debug_data_addr;
	u16 page_address;
	u8 page;
	u8 debug_command_reg;
	struct mxt_object *object;
	int num_nodes;

	data = mxt->debug_data;
	if (data == NULL)
		return -EIO;
	num_nodes = mxt->pdata->x_line * mxt->pdata->y_line;
	/* If first read after open, read all data to buffer. */
	if (mxt->current_debug_datap == 0) {

		object = mxt_get_object(mxt, MXT_GEN_COMMAND_T6);
		diagnostics_reg = object->start_address + MXT_COMMAND_DIAGNOSTIC;

		if (count > (num_nodes * 2))
			count = num_nodes;

		object = mxt_get_object(mxt, MXT_DEBUG_DIAGNOSTIC_T37);
		debug_data_addr = object->start_address + MXT_ADR_T37_DATA;
		page_address = object->start_address + MXT_ADR_T37_PAGE;

		error = __mxt_read_reg(mxt->client, page_address, 1, &page);
		if (error < 0)
			return error;
		//mxt_debug(DEBUG_TRACE, "debug data page = %d\n", page);
		while (page != 0) {
			error = mxt_write_reg(mxt->client,
					       diagnostics_reg,
					       MXT_CMD_T6_PAGE_DOWN);
			if (error < 0)
				return error;
			/* Wait for command to be handled; when it has, the
			   register will be cleared. */
			debug_command_reg = 1;
			while (debug_command_reg != 0) {
				error = __mxt_read_reg(mxt->client,
						       diagnostics_reg, 1,
						       &debug_command_reg);
				if (error < 0)
					return error;
				/*mxt_debug(DEBUG_TRACE,
					  "Waiting for debug diag command "
					  "to propagate...\n");*/

			}
			error = __mxt_read_reg(mxt->client, page_address, 1,
						&page);
			if (error < 0)
				return error;
			//mxt_debug(DEBUG_TRACE, "debug data page = %d\n", page);
		}

		/*
		 * Lock mutex to prevent writing some unwanted data to debug
		 * command register. User can still write through the char
		 * device interface though. TODO: fix?
		 */

		mutex_lock(&mxt->debug_mutex);
		/* Configure Debug Diagnostics object to show deltas/refs */
		error = mxt_write_reg(mxt->client, diagnostics_reg,
				       debug_command);

		/* Wait for command to be handled; when it has, the
		 * register will be cleared. */
		debug_command_reg = 1;
		while (debug_command_reg != 0) {
			error = __mxt_read_reg(mxt->client,
					       diagnostics_reg, 1,
					       &debug_command_reg);
			if (error < 0)
				return error;
			//mxt_debug(DEBUG_TRACE, "Waiting for debug diag command "
			//	  "to propagate...\n");

		}

		if (error < 0) {
			printk(KERN_WARNING
			       "Error writing to maXTouch device!\n");
			return error;
		}

		size = num_nodes * sizeof(u16);

		while (size > 0) {
			read_size = size > 128 ? 128 : size;
			/*mxt_debug(DEBUG_TRACE,
				  "Debug data read loop, reading %d bytes...\n",
				  read_size);*/
			error = __mxt_read_reg(mxt->client,
					       debug_data_addr,
					       read_size,
					       (u8 *) &data[offset]);
			if (error < 0) {
				printk(KERN_WARNING
				       "Error reading debug data\n");
				goto error;
			}
			offset += read_size / 2;
			size -= read_size;

			error = mxt_write_reg(mxt->client, diagnostics_reg,
					       MXT_CMD_T6_PAGE_UP);
			if (error < 0) {
				printk(KERN_WARNING
				       "Error writing to maXTouch device!\n");
				goto error;
			}
		}
		mutex_unlock(&mxt->debug_mutex);
	}

	buf_start = buf;
	i = mxt->current_debug_datap;

	while (((buf - buf_start) < (count - 6)) && (i < num_nodes)) {

		mxt->current_debug_datap++;
		if (debug_command == MXT_CMD_T6_REFERENCES_MODE)
			buf += sprintf(buf, "%d: %5d\n", i,
				       (u16) le16_to_cpu(data[i]));
		else if (debug_command == MXT_CMD_T6_DELTAS_MODE)
			buf += sprintf(buf, "%d: %5d\n", i,
				       (s16) le16_to_cpu(data[i]));
		i++;
	}

	return buf - buf_start;
 error:
	mutex_unlock(&mxt->debug_mutex);
	return error;
}

ssize_t deltas_read(struct file *file, char *buf, size_t count, loff_t *ppos)
{
	return debug_data_read(file->private_data, buf, count, ppos,
			       MXT_CMD_T6_DELTAS_MODE);
}

ssize_t refs_read(struct file *file, char *buf, size_t count, loff_t *ppos)
{
	return debug_data_read(file->private_data, buf, count, ppos,
			       MXT_CMD_T6_REFERENCES_MODE);
}

int debug_data_open(struct inode *inode, struct file *file)
{
	struct mxt_data *mxt;
	int i;
	int num_nodes;
	mxt = inode->i_private;
	if (mxt == NULL)
		return -EIO;
	mxt->current_debug_datap = 0;
	num_nodes = mxt->pdata->x_line * mxt->pdata->y_line;
	mxt->debug_data = kmalloc(num_nodes * sizeof(u16), GFP_KERNEL);
	if (mxt->debug_data == NULL)
		return -ENOMEM;

	for (i = 0; i < num_nodes; i++)
		mxt->debug_data[i] = 7777;

	file->private_data = mxt;
	return 0;
}

int debug_data_release(struct inode *inode, struct file *file)
{
	struct mxt_data *mxt;
	mxt = file->private_data;
	kfree(mxt->debug_data);
	return 0;
}

const struct file_operations delta_fops = {
	.owner = THIS_MODULE,
	.open = debug_data_open,
	.release = debug_data_release,
	.read = deltas_read,
};

const struct file_operations refs_fops = {
	.owner = THIS_MODULE,
	.open = debug_data_open,
	.release = debug_data_release,
	.read = refs_read,
};

static int selftest_write(struct file *file, const char __user *user_buf, size_t count, loff_t *ppos)
{
	char *buf;
	int ret = 0;
	int high_limit = 0;
	int low_limit = 0;
	struct mxt_data *mxt = file->private_data;
	struct mxt_object *object;

	object = mxt_get_object(mxt, MXT_SPT_SELFTEST_T25);

	if (user_buf != NULL && count > 0) {
		buf = kmalloc(count, GFP_KERNEL);
		if (buf == NULL) {
			printk(KERN_WARNING "Memory allocation failed!\n");
			return -ENOMEM;
		}
		if (copy_from_user(buf, user_buf, count)) {
			kfree(buf);
			return -EFAULT;
		}
		ret = sscanf(buf, "high=%d", &high_limit);
		if (ret != 0) {
			printk(KERN_INFO "SELFTEST, set high limit to %d\n", high_limit);
			mxt_write_reg(mxt->client, object->start_address + MXT_ADDR_T25_HISIGLIM_L, high_limit & 0xff);
			msleep(10);
			mxt_write_reg(mxt->client, object->start_address + MXT_ADDR_T25_HISIGLIM_H, (high_limit >> 8) & 0xff);
			msleep(10);
		}

		ret = sscanf(buf, "low=%d", &low_limit);
		if (ret != 0) {
			printk(KERN_INFO "SELFTEST, set low limit to %d\n", low_limit);
			mxt_write_reg(mxt->client, object->start_address + MXT_ADDR_T25_LOSIGLIM_L, low_limit & 0xff);
			msleep(10);
			mxt_write_reg(mxt->client, object->start_address + MXT_ADDR_T25_LOSIGLIM_H, (low_limit >> 8) & 0xff);
			msleep(10);
		}
		kfree(buf);
	}

	return count;
}

ssize_t selftest_read(struct file *file, char __user *usrbuf, size_t count, loff_t *ppos)
{
	int high_limit = 0;
	int low_limit = 0;
	u8 selftest_count = 0;
	u8 data[4] = { 0 };
	char cstr[100];
	int length = 0;
	struct mxt_data *mxt;
	struct i2c_client *client;
	struct mxt_object *object;
	mxt = file->private_data;

	client = mxt->client;

	/* set the initial status */
	mxt->self_test_result = e_self_test_none;
	/* read high limit and low limit */
	object = mxt_get_object(mxt, MXT_SPT_SELFTEST_T25);
	__mxt_read_reg(client, object->start_address + MXT_ADDR_T25_HISIGLIM_L, sizeof(data), data);
	high_limit = data[0] | (data[1] << 8);
	low_limit = data[2] | (data[3] << 8);
	/* enable selftest object */
	mxt_write_reg(client, object->start_address + MXT_ADDR_T25_CTRL, 0x03);
	/* do selftest */
	mxt_write_reg(client, object->start_address + MXT_ADDR_T25_CMD, MXT_CMD_T25_ALL_TEST);

	while (mxt->self_test_result == e_self_test_none && selftest_count < 20) {
		//printk(KERN_INFO "SELFTEST, selftest status none\n");
		selftest_count++;
		msleep(100);
	}
	mxt_write_reg(client, object->start_address + MXT_ADDR_T25_CTRL, 0x0);

	length += snprintf((cstr + length), sizeof(cstr) - length, "SELFTEST, high limit: %d\n", high_limit);
	length += snprintf((cstr + length), sizeof(cstr) - length, "SELFTEST, low limit: %d\n", low_limit);
	length += snprintf((cstr + length), sizeof(cstr) - length, "SELFTEST RESULT: %s\n", (mxt->self_test_result == e_self_test_pass) ? "PASS" : "FAIL");
	return simple_read_from_buffer(usrbuf, count, ppos, cstr, length);;
}

static int selftest_open(struct inode *inode, struct file *file)
{
	struct mxt_data *mxt;

	mxt = inode->i_private;
	file->private_data = mxt;
	return 0;
}

static struct file_operations selftest_fops = {
	.open 		= selftest_open,
	.read 		= selftest_read,
	.write		= selftest_write,
};

int mxt_memory_open(struct inode *inode, struct file *file)
{
	struct mxt_data *mxt;
	mxt = container_of(inode->i_cdev, struct mxt_data, cdev);
	if (mxt == NULL)
		return -EIO;
	file->private_data = mxt;
	return 0;
}

ssize_t mxt_memory_read(struct file *file, char __user *buf, size_t count,
			loff_t *ppos)
{
	int i;
	struct mxt_data *mxt;

	u8 *data;
	data = kzalloc(count, GFP_KERNEL);

	mxt = file->private_data;

	i = __mxt_read_reg(mxt->client, mxt->address_pointer, count,
				   data);

	if (copy_to_user(buf, data, count) != 0) {
		printk("Failed to copy data to userspace\n");
		kfree(data);
		return -EFAULT;
	}
	kfree(data);

	return i;
}

ssize_t mxt_memory_write(struct file *file, const char __user *buf, size_t count,
			 loff_t *ppos)
{
	int i;
	int whole_blocks;
	int last_block_size;
	struct mxt_data *mxt;
	u16 address;

	u8 *data;
	data = kzalloc(count, GFP_KERNEL);
	if (copy_from_user(data, buf, count)) {
		printk("Failed to copy data from userspace\n");
		kfree(data);
		return -EFAULT;
	}

	mxt = file->private_data;
	address = mxt->address_pointer;

	//mxt_debug(DEBUG_TRACE, "mxt_memory_write entered\n");
	whole_blocks = count / I2C_PAYLOAD_SIZE;
	last_block_size = count % I2C_PAYLOAD_SIZE;

	disable_irq(mxt->client->irq);
	mutex_lock(&mxt->access_mutex);

	for (i = 0; i < whole_blocks; i++) {
		//mxt_debug(DEBUG_TRACE, "About to write to %d...", address);
		mxt_write_block(mxt->client, address, I2C_PAYLOAD_SIZE,
				(u8 *) data);
		address += I2C_PAYLOAD_SIZE;
		buf += I2C_PAYLOAD_SIZE;
	}

	mxt_write_block(mxt->client, address, last_block_size, (u8 *) data);

	mxt->last_address = -1;
	mutex_unlock(&mxt->access_mutex);
	enable_irq(mxt->client->irq);

	kfree(data);
	return count;
}

long (*unlocked_ioctl) (struct file *, unsigned int, unsigned long);
static long mxt_ioctl(struct file *file,
		     unsigned int cmd, unsigned long arg)
{
	int retval;
	struct mxt_data *mxt;
	struct mxt_object *object;

	retval = 0;
	mxt = file->private_data;
	object = mxt_get_object(mxt, MXT_GEN_COMMAND_T6);

	switch (cmd) {
	case MXT_SET_ADDRESS:
		mxt->address_pointer = (u16) arg;
		break;
	case MXT_RESET:
		/*retval = mxt_write_byte(mxt->client,
					MXT_BASE_ADDR
					(MXT_GEN_COMMANDPROCESSOR_T6,
					 mxt) + MXT_ADR_T6_RESET, 1);*/
		retval = mxt_write_reg(mxt->client,
					object->start_address + MXT_COMMAND_RESET, 1);
		break;
	case MXT_CALIBRATE:
		/*retval = mxt_write_byte(mxt->client,
					MXT_BASE_ADDR
					(MXT_GEN_COMMANDPROCESSOR_T6,
					 mxt) + MXT_ADR_T6_CALIBRATE, 1);*/
		retval = mxt_write_reg(mxt->client,
					object->start_address + MXT_COMMAND_CALIBRATE, 1);
		break;
	case MXT_BACKUP:
		/*retval = mxt_write_byte(mxt->client,
					MXT_BASE_ADDR
					(MXT_GEN_COMMANDPROCESSOR_T6,
					 mxt) + MXT_ADR_T6_BACKUPNV,
					MXT_CMD_T6_BACKUP);*/
		retval = mxt_write_reg(mxt->client,
					object->start_address + MXT_COMMAND_BACKUPNV,
					MXT_BACKUP_VALUE);
		break;
	/*case MXT_NONTOUCH_MSG:
		mxt->nontouch_msg_only = 1;
		break;
	case MXT_ALL_MSG:
		mxt->nontouch_msg_only = 0;
		break;*/
	default:
		return (long)(-EIO);
	}

	return (long)retval;
}

const struct file_operations mxt_memory_fops = {
	.owner = THIS_MODULE,
	.open = mxt_memory_open,
	.read = mxt_memory_read,
	.write = mxt_memory_write,
	.unlocked_ioctl = mxt_ioctl,
};

/*********************************************************************************************/

static void mxt_input_report(struct mxt_data *data, int single_id)
{
	struct mxt_finger *finger = data->finger;
	struct input_dev *input_dev = data->input_dev;
	int status = finger[single_id].status;
	int finger_num = 0;
	int id;

	for (id = 0; id < MXT_MAX_FINGER; id++) {
		if (!finger[id].status)
			continue;
		if (finger[id].status != MXT_RELEASE) {
			input_report_abs(input_dev, ABS_MT_TOUCH_MAJOR,
					finger[id].area);
			input_report_abs(input_dev, ABS_MT_POSITION_X,
					finger[id].x);
			input_report_abs(input_dev, ABS_MT_POSITION_Y,
					finger[id].y);
			input_report_abs(input_dev, ABS_MT_TRACKING_ID, id);
			input_mt_sync(input_dev);
		}


		if (finger[id].status == MXT_RELEASE)
			finger[id].status = 0;
		else
			finger_num++;
	}

	input_report_key(input_dev, BTN_TOUCH, finger_num > 0);

	if (status != MXT_RELEASE) {
		input_report_abs(input_dev, ABS_X, finger[single_id].x);
		input_report_abs(input_dev, ABS_Y, finger[single_id].y);
	}

	input_sync(input_dev);
}

static void mxt_input_touchevent(struct mxt_data *data,
				      struct mxt_message *message, int id)
{
	struct mxt_finger *finger = data->finger;
	struct device *dev = &data->client->dev;
	u8 status = message->message[0];
	int x;
	int y;
	int area;

	/* Check the touch is present on the screen */
	if (!(status & MXT_DETECT)) {
		if (status & (MXT_RELEASE | MXT_SUPPRESS)) {
			dev_dbg(dev, "[%d] released\n", id);

			finger[id].status = MXT_RELEASE;
			mxt_input_report(data, id);
		}
		return;
	}

	/* Check only AMP detection */
	if (!(status & (MXT_PRESS | MXT_MOVE)))
		return;

	x = (message->message[1] << 4) | ((message->message[3] >> 4) & 0xf);
	y = (message->message[2] << 4) | ((message->message[3] & 0xf));
	if (data->max_x < 1024)
		x = x >> 2;
	if (data->max_y < 1024)
		y = y >> 2;

	area = message->message[4];

	dev_dbg(dev, "[%d] %s x: %d, y: %d, area: %d\n", id,
		status & MXT_MOVE ? "moved" : "pressed",
		x, y, area);

	finger[id].status = status & MXT_MOVE ?
				MXT_MOVE : MXT_PRESS;
	finger[id].x = x;
	finger[id].y = y;
	finger[id].area = area;

	mxt_input_report(data, id);
}

static void mxt_worker(struct work_struct *work)
{
	struct mxt_data *data;
	struct mxt_message message;
	struct mxt_object *touch_object;
	struct mxt_object *selftest_object;
	struct mxt_object *touch_suppress_object;
	struct device *dev;
	struct mxt_finger *finger;
	int id;
	u8 reportid;

	data = container_of(work, struct mxt_data, dwork.work);
	dev = &data->client->dev;
	finger = data->finger;
	disable_irq(data->client->irq);

	selftest_object = mxt_get_object(data, MXT_SPT_SELFTEST_T25);
	touch_object = mxt_get_object(data, MXT_TOUCH_MULTI_T9);
	touch_suppress_object = mxt_get_object(data, MXT_PROCI_TOUCHSUPPRESSION_T42);
	if (!touch_object || !selftest_object || !touch_suppress_object)
		goto end;

	do {
		if (mxt_read_message(data, &message)) {
			dev_err(dev, "Failed to read message\n");
			goto end;
		}

		reportid = message.reportid;

		if (data->debug_enabled)
			print_hex_dump(KERN_DEBUG, "MXT MSG:", DUMP_PREFIX_NONE,
				16, 1, &message, sizeof(struct mxt_message), false);

		if (reportid >= touch_object->min_reportid
			&& reportid <= touch_object->max_reportid) {
			id = reportid - touch_object->min_reportid;
			mxt_input_touchevent(data, &message, id);
		} else if (reportid >= selftest_object->min_reportid
			&& reportid <= selftest_object->max_reportid) {
			if (message.message[0] == MXT_MSGR_T25_OK)
				data->self_test_result = e_self_test_pass;
			else {
				data->self_test_result = e_self_test_fail;
				printk("Touch selftest erro result code: 0x%02X \n", message.message[0]);
				printk("Touch selftest error info: 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X\n",
					message.message[1], message.message[2], message.message[3],
					message.message[4], message.message[5]);
			}
		} else if (reportid >= touch_suppress_object->min_reportid
			&& reportid <= touch_suppress_object->max_reportid) {
			if (message.message[0] | 0x01) {
				for (id = 0; id < MXT_MAX_FINGER; id++) {
					if (finger[id].status & (MXT_MOVE | MXT_PRESS))
						finger[id].status = MXT_RELEASE;
				}
				mxt_input_report(data, id);
			}
		} else if (reportid != MXT_RPTID_NOMSG)
			mxt_dump_message(dev, &message);
	} while (reportid != MXT_RPTID_NOMSG);

	enable_irq(data->client->irq);
	/* Make sure we don't miss any interrupts and read changeline. */
	if (data->read_chg() == 0)
		schedule_delayed_work(&data->dwork, 0);
	return;
end:
	enable_irq(data->client->irq);
	printk(KERN_ERR "mxt_worker: error reading message!!!\n");
	return;
}

static irqreturn_t mxt_interrupt(int irq, void *dev_id)
{
	struct mxt_data *data = dev_id;
	struct mxt_message message;
	struct mxt_object *touch_object;
	struct mxt_object *selftest_object;
	struct device *dev = &data->client->dev;
	int touchid;
	u8 reportid;

	if (!machine_is_sphinx()) {
		selftest_object = mxt_get_object(data, MXT_SPT_SELFTEST_T25);
		do {
			if (mxt_read_message(data, &message)) {
				dev_err(dev, "Failed to read message\n");
				goto end;
			}

			reportid = message.reportid;

			touch_object = mxt_get_object(data, MXT_TOUCH_MULTI_T9);
			if (!touch_object)
				goto end;

			if (data->debug_enabled)
				print_hex_dump(KERN_DEBUG, "MXT MSG:", DUMP_PREFIX_NONE,
					16, 1, &message, sizeof(struct mxt_message), false);

			if (reportid >= touch_object->min_reportid
				&& reportid <= touch_object->max_reportid) {
				touchid = reportid - touch_object->min_reportid;
				mxt_input_touchevent(data, &message, touchid);
			} else if (reportid >= selftest_object->min_reportid
				&& reportid <= selftest_object->max_reportid) {
				if (message.message[0] == MXT_MSGR_T25_OK)
					data->self_test_result = e_self_test_pass;
				else
					data->self_test_result = e_self_test_fail;
			} else if (reportid != MXT_RPTID_NOMSG)
				mxt_dump_message(dev, &message);
		} while (reportid != MXT_RPTID_NOMSG);
	} else {
		schedule_delayed_work(&data->dwork, 0);
	}
end:
	return IRQ_HANDLED;
}

static void mxt_noise_worker(struct work_struct *work)
{
	struct mxt_data *data = container_of(work, struct mxt_data, dwork.work);
	const struct mxt_platform_data *pdata = data->pdata;
	struct mxt_cfg_noise *cfg_noises = pdata->cfg_noises;
	int gpio_ac = 0, gpio_hdmi = 0, i = 0;

	gpio_ac = gpio_get_value(data->pdata->gpio_ac);
	gpio_hdmi = gpio_get_value(data->pdata->gpio_hdmi);

	printk(KERN_INFO "[Touch] AC: %d, HDMI: %d\n", gpio_ac, gpio_hdmi);

	if (gpio_ac || gpio_hdmi) {
		for (i = 0; i < pdata->cfg_noises_length; i++) {
			mxt_write_object(data, cfg_noises[i].type, cfg_noises[i].offset, cfg_noises[i].noise_value);
/* 			printk(KERN_INFO "[Touch] T%02d[%2d] = %d\n", cfg_noises[i].type, cfg_noises[i].offset, cfg_noises[i].noise_value); */
		}
	}
	else {
		for (i = 0; i < pdata->cfg_noises_length; i++) {
			mxt_write_object(data, cfg_noises[i].type, cfg_noises[i].offset, cfg_noises[i].normal_value);
/* 			printk(KERN_INFO "[Touch] T%02d[%2d] = %d\n", cfg_noises[i].type, cfg_noises[i].offset, cfg_noises[i].normal_value); */
		}
	}
}

static irqreturn_t mxt_interrupt_noise(int irq, void *dev_id)
{
	struct mxt_data *data = dev_id;

	cancel_delayed_work(&data->dwork);
	schedule_delayed_work(&data->dwork, msecs_to_jiffies(200));

	return IRQ_HANDLED;
}

static int mxt_make_highchg(struct mxt_data *data)
{
	struct device *dev = &data->client->dev;
	struct mxt_message message;
	int count = 30;
	int error;

	/* Read dummy message to make high CHG pin */
	do {
		error = mxt_read_message(data, &message);
		if (error)
			return error;
	} while (message.reportid != MXT_RPTID_NOMSG && --count);

	if (!count) {
		dev_err(dev, "CHG pin isn't cleared\n");
		return -EBUSY;
	}

	return 0;
}

static int mxt_check_reg_init(struct mxt_data *data)
{
	struct i2c_client *client = data->client;
	const struct mxt_platform_data *pdata = data->pdata;
	struct mxt_object *object;
	struct mxt_message message;
	struct device *dev = &data->client->dev;
	int index = 0;
	int timeout_counter = 0;
	int i, j, config_offset;
	int error, reg;
	u8 val;
	unsigned long current_crc;
	u8 command_register;

	if (!pdata->config) {
		dev_dbg(dev, "No cfg data defined, skipping reg init\n");
		return 0;
	}

	if ((pdata->config_fw_ver != data->info.version) &&
		(pdata->config_fw_ver != 0x00)) {
		dev_info(dev, "firmware version not match, skipping reg init\n");
		return 0;
	}

	object = mxt_get_object(data, MXT_SPT_USERDATA_T38);
	if (!object)
		return -EIO;
	reg = object->start_address;
	error = mxt_read_reg(client, reg + 1, &val);
	if (val == 0xAA) {
		printk(KERN_INFO "[Touch] Force skipping reg init\n");
		return 0;
	}

	/* Try to read the config checksum of the existing cfg */
	mxt_write_object(data, MXT_GEN_COMMAND_T6,
			 MXT_COMMAND_REPORTALL, 1);
	msleep(30);

	error = mxt_read_message(data, &message);
	if (error)
		return error;

	object = mxt_get_object(data, MXT_GEN_COMMAND_T6);
	if (!object)
		return -EIO;

	/* Check if this message is from command processor (which has
	   only one reporting ID), if so, bytes 1-3 are the checksum. */
	if (message.reportid == object->max_reportid) {
		current_crc = message.message[1] | (message.message[2] << 8) |
			      (message.message[3] << 16);
	} else {
		dev_info(dev, "Couldn't retrieve the current cfg checksum, "
			 "forcing load\n");
		current_crc = 0xFFFFFFFF;
	}
	dev_info(dev,
		 "Config CRC read from the mXT: %X\n",
		 (unsigned int) current_crc);

	if (current_crc == pdata->config_crc) {
		dev_info(dev,
			 "Matching CRC's, skipping CFG load.\n");
		return 0;
	} else {
		dev_info(dev, "Doesn't match platform data config CRC (%X), "
			 "writing config from platform data...\n",
			 (unsigned int) pdata->config_crc);
	}

	for (i = 0; i < data->info.object_num; i++) {
		object = data->object_table + i;

		if (!mxt_object_writable(object->type))
			continue;
		dev_info(dev, "Writing object type %d, config offset %d", data->object_table[i].type, index);
		for (j = 0;
		     j < object->size * object->instances;
		     j++) {
			config_offset = index + j;
			if (config_offset > pdata->config_length) {
				dev_err(dev, "Not enough config data!\n");
				dev_err(dev, "config base is %d, offset is %d\n", index, config_offset);
				return -EINVAL;
			}
			mxt_write_object(data, object->type, j,
					 pdata->config[config_offset]);
		}
		index += object->size * object->instances;
	}
	dev_info(dev, "Config written!");

	error = mxt_make_highchg(data);
	if (error)
		return error;

	/* Backup to memory */
	mxt_write_object(data, MXT_GEN_COMMAND_T6,
			MXT_COMMAND_BACKUPNV,
			MXT_BACKUP_VALUE);
	msleep(MXT_BACKUP_TIME);
	do {
		error =  mxt_read_object(data, MXT_GEN_COMMAND_T6,
					MXT_COMMAND_BACKUPNV,
					&command_register);
		if (error)
			return error;
		msleep(10);
	} while ((command_register != 0) && (timeout_counter++ <= 100));
	if (timeout_counter > 100) {
		dev_err(&client->dev, "No response after backup!\n");
		return -EIO;
	}

	/* Clear the interrupt line */
	error = mxt_make_highchg(data);
	if (error)
		return error;

	/* Soft reset */
	mxt_write_object(data, MXT_GEN_COMMAND_T6,
			MXT_COMMAND_RESET, 1);
	if (data->pdata->read_chg == NULL) {
		msleep(MXT_RESET_NOCHGREAD);
	} else {
		switch (data->info.family_id) {
		case MXT224_ID:
			msleep(MXT224_RESET_TIME);
			break;
		case MXT768E_ID:
			msleep(MXT768E_RESET_TIME);
			break;
		case MXT1386_ID:
			msleep(MXT1386_RESET_TIME);
			break;
		default:
			msleep(MXT_RESET_TIME);
		}
		timeout_counter = 0;
		while ((timeout_counter++ <= 100) && data->pdata->read_chg())
			msleep(10);
		if (timeout_counter > 100) {
			dev_err(&client->dev, "No response after reset!\n");
			return -EIO;
		}
	}

	return 0;
}


static void mxt_handle_pdata(struct mxt_data *data)
{
	const struct mxt_platform_data *pdata = data->pdata;

	if (pdata->read_chg != NULL)
		data->read_chg = pdata->read_chg;
}

static int mxt_set_power_cfg(struct mxt_data *data, u8 sleep)
{
	struct device *dev = &data->client->dev;
	int error;
	u8 actv_cycle_time = 0;
	u8 idle_cycle_time = 0;
	u8 actv2idle_timeout = data->actv2idle_timeout;

	if (!sleep) {
		actv_cycle_time = data->actv_cycle_time;
		idle_cycle_time = data->idle_cycle_time;
	}

	error = mxt_write_object(data, MXT_GEN_POWER_T7, MXT_POWER_ACTVACQINT,
				 actv_cycle_time);
	if (error)
		goto i2c_error;

	error = mxt_write_object(data, MXT_GEN_POWER_T7, MXT_POWER_IDLEACQINT,
				 idle_cycle_time);
	if (error)
		goto i2c_error;

	error = mxt_write_object(data, MXT_GEN_POWER_T7, MXT_POWER_ACTV2IDLETO,
				 actv2idle_timeout);
	if (error)
		goto i2c_error;

	dev_dbg(dev, "%s: Set ACTV %d, IDLE %d", __func__,
		actv_cycle_time, idle_cycle_time);

	return 0;

i2c_error:
	dev_err(dev, "Failed to set power cfg");
	return error;
}

static int mxt_init_power_cfg(struct mxt_data *data)
{
	const struct mxt_platform_data *pdata = data->pdata;
	struct device *dev = &data->client->dev;
	int error;

	data->slowscan_actv_cycle_time = 120;	/* 120mS */
	data->slowscan_idle_cycle_time = 10;	/* 10mS */
	data->slowscan_actv2idle_timeout = 100;	/* 10 seconds */
	if (pdata->actv_cycle_time > 0 && pdata->idle_cycle_time > 0) {
		data->actv_cycle_time = pdata->actv_cycle_time;
		data->idle_cycle_time = pdata->idle_cycle_time;
	} else {
		error = mxt_read_object(data, MXT_GEN_POWER_T7,
			MXT_POWER_ACTVACQINT,
			&data->actv_cycle_time);

		if (error)
			return error;

		error = mxt_read_object(data, MXT_GEN_POWER_T7,
			MXT_POWER_IDLEACQINT,
			&data->idle_cycle_time);

		if (error)
			return error;
	}

	error = mxt_read_object(data, MXT_GEN_POWER_T7,
		MXT_POWER_ACTV2IDLETO,
		&data->actv2idle_timeout);

	if (error)
		return error;

	/* On init, power up */
	if (!machine_is_sphinx()) {
		error = mxt_set_power_cfg(data, 0);
		if (error)
			return error;
	}

	dev_info(dev, "Initialised power cfg: ACTV %d, IDLE %d",
		data->actv_cycle_time, data->idle_cycle_time);

	return 0;
}

static int mxt_get_info(struct mxt_data *data)
{
	struct i2c_client *client = data->client;
	struct mxt_info *info = &data->info;
	int error;
	u8 val;

	/* force send of address pointer on first read during probe */
	data->last_address = -1;

	error = mxt_read_reg(client, MXT_FAMILY_ID, &val);
	if (error)
		return error;
	info->family_id = val;

	error = mxt_read_reg(client, MXT_VARIANT_ID, &val);
	if (error)
		return error;
	info->variant_id = val;

	error = mxt_read_reg(client, MXT_VERSION, &val);
	if (error)
		return error;
	info->version = val;

	error = mxt_read_reg(client, MXT_BUILD, &val);
	if (error)
		return error;
	info->build = val;

	error = mxt_read_reg(client, MXT_OBJECT_NUM, &val);
	if (error)
		return error;
	info->object_num = val;

	return 0;
}

static int mxt_get_object_table(struct mxt_data *data)
{
	struct device *dev = &data->client->dev;
	int error;
	int i;
	u16 reg;
	u8 reportid = 0;
	u8 buf[MXT_OBJECT_SIZE];

	for (i = 0; i < data->info.object_num; i++) {
		struct mxt_object *object = data->object_table + i;

		reg = MXT_OBJECT_START + MXT_OBJECT_SIZE * i;
		error = mxt_read_object_table(data->client, reg, buf);
		if (error)
			return error;

		object->type = buf[0];
		object->start_address = (buf[2] << 8) | buf[1];
		object->size = buf[3] + 1;
		object->instances = buf[4] + 1;
		object->num_report_ids = buf[5];

		if (object->num_report_ids) {
			reportid += object->num_report_ids * object->instances;
			object->max_reportid = reportid;
			object->min_reportid = object->max_reportid -
				object->instances * object->num_report_ids + 1;
		}

		/* Store message window address so we don't have to
		   search the object table every time we read message */
		if (object->type == MXT_GEN_MESSAGE_T5)
			data->msg_address = object->start_address;

		dev_dbg(dev, "T%d, start:%d size:%d instances:%d "
			"min_reportid:%d max_reportid:%d\n",
			object->type, object->start_address, object->size,
			object->instances,
			object->min_reportid, object->max_reportid);
	}

	return 0;
}

static int mxt_initialize(struct mxt_data *data)
{
	struct i2c_client *client = data->client;
	struct mxt_info *info = &data->info;
	int error;

	data->object_table = kcalloc(info->object_num,
				     sizeof(struct mxt_object),
				     GFP_KERNEL);
	if (!data->object_table) {
		dev_err(&client->dev, "Failed to allocate memory\n");
		return -ENOMEM;
	}

	/* Get object table information */
	error = mxt_get_object_table(data);
	if (error) {
		dev_err(&client->dev, "Failed to read object table\n");
		return error;
	}

	/* Load initial touch chip configuration */
	error = mxt_check_reg_init(data);
	if (error) {
		dev_err(&client->dev, "Failed to initialize configuration\n");
		return error;
	}

	mxt_handle_pdata(data);

	error = mxt_init_power_cfg(data);
	if (error) {
		dev_err(&client->dev, "Failed to initialize power cfg\n");
		return error;
	}

	dev_info(&client->dev,
			"Family ID: %d Variant ID: %d Version: %02X Build: %02X\n",
			info->family_id, info->variant_id, info->version,
			info->build);

	dev_info(&client->dev,
			"Matrix X Size: %d Matrix Y Size: %d Object Num: %d\n",
			info->matrix_xsize, info->matrix_ysize,
			info->object_num);

	return 0;
}

static void mxt_calc_resolution(struct mxt_data *data)
{
	unsigned int max_x = data->pdata->x_size - 1;
	unsigned int max_y = data->pdata->y_size - 1;

	if (data->pdata->orient & MXT_XY_SWITCH) {
		data->max_x = max_y;
		data->max_y = max_x;
	} else {
		data->max_x = max_x;
		data->max_y = max_y;
	}
}

static ssize_t mxt_id_information_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	struct mxt_data *data = dev_get_drvdata(dev);
	struct mxt_info *info = &data->info;
	int count = 0;

	count += sprintf(buf + count, "Family ID  : 0x%02X\n", info->family_id);
	count += sprintf(buf + count, "Variant ID : 0x%02X\n", info->variant_id);
	count += sprintf(buf + count, "Version    : 0x%02X -> %d.%d\n", info->version, info->version / 16, info->version % 16);
	count += sprintf(buf + count, "Build      : 0x%02X\n", info->build);

	return count;
}

static ssize_t mxt_object_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	struct mxt_data *data = dev_get_drvdata(dev);
	struct mxt_object *object;
	int count = 0;
	int i, j;
	int error;
	u8 val;

	for (i = 0; i < data->info.object_num; i++) {
		object = data->object_table + i;

		count += snprintf(buf + count, PAGE_SIZE - count,
				"Object[%d] (Type %d)\n",
				i + 1, object->type);
		if (count >= PAGE_SIZE)
			return PAGE_SIZE - 1;

		if (!mxt_object_readable(object->type)) {
			count += snprintf(buf + count, PAGE_SIZE - count,
					"\n");
			if (count >= PAGE_SIZE)
				return PAGE_SIZE - 1;
			continue;
		}

		for (j = 0; j < object->size; j++) {
			error = mxt_read_object(data,
						object->type, j, &val);
			if (error)
				return error;

			count += snprintf(buf + count, PAGE_SIZE - count,
					"\t[%2d]: %02x (%d)\n", j, val, val);
			if (count >= PAGE_SIZE)
				return PAGE_SIZE - 1;
		}

		count += snprintf(buf + count, PAGE_SIZE - count, "\n");
		if (count >= PAGE_SIZE)
			return PAGE_SIZE - 1;
	}

	return count;
}

static int mxt_load_fw(struct device *dev, const char *fn)
{
	struct mxt_data *data = dev_get_drvdata(dev);
	struct i2c_client *client = data->client;
	const struct firmware *fw = NULL;
	unsigned int frame_size;
	unsigned int pos = 0;
	int ret;

	ret = request_firmware(&fw, fn, dev);
	if (ret) {
		dev_err(dev, "Unable to open firmware %s\n", fn);
		return ret;
	}

	/* Change to the bootloader mode */
	if (bootloader_mode == 0) {
		mxt_write_object(data, MXT_GEN_COMMAND_T6,
				MXT_COMMAND_RESET, MXT_BOOT_VALUE);
		msleep(MXT_RESET_TIME * 2);
	}

	/* Change to slave address of bootloader */
#if 0
	if (client->addr == MXT_APP_LOW)
		client->addr = MXT_BOOT_LOW;
	else
		client->addr = MXT_BOOT_HIGH;
#else
	if (client->addr >= MXT_APP_LOW)
		client->addr -= 0x26;
	printk(KERN_INFO "[Touch] client->addr: 0x%X ---> %s (%d)\n",client->addr , __FUNCTION__, __LINE__);
#endif

	ret = mxt_check_bootloader(client, MXT_WAITING_BOOTLOAD_CMD);
	if (ret)
		goto out;

	/* Unlock bootloader */
	mxt_unlock_bootloader(client);

	while (pos < fw->size) {
		ret = mxt_check_bootloader(client,
						MXT_WAITING_FRAME_DATA);
		if (ret)
			goto out;

		frame_size = ((*(fw->data + pos) << 8) | *(fw->data + pos + 1));

		printk(KERN_INFO "[Touch] pos: %d, frame_size: %d ---> %s (%d)\n", pos, frame_size, __FUNCTION__, __LINE__);

		/* We should add 2 at frame size as the the firmware data is not
		 * included the CRC bytes.
		 */
		frame_size += 2;
		/* Write one frame to device */
		mxt_fw_write(client, fw->data + pos, frame_size);

		ret = mxt_check_bootloader(client,
						MXT_FRAME_CRC_PASS);
		if (ret)
			goto out;

		pos += frame_size;

		dev_dbg(dev, "Updated %d bytes / %zd bytes\n", pos, fw->size);
	}

out:
	release_firmware(fw);

	/* Change to slave address of application */
#if 0
	if (client->addr == MXT_BOOT_LOW)
		client->addr = MXT_APP_LOW;
	else
		client->addr = MXT_APP_HIGH;
#else
	if (client->addr < MXT_APP_LOW)
		client->addr += 0x26;
	printk(KERN_INFO "[Touch] client->addr: 0x%X ---> %s (%d)\n",client->addr , __FUNCTION__, __LINE__);
#endif

	return ret;
}

static ssize_t mxt_update_fw_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	struct mxt_data *data = dev_get_drvdata(dev);
	int error;
	char event[32] = {0};
	char *envp[] = {event, NULL};

	disable_irq(data->irq);

	error = mxt_load_fw(dev, MXT_FW_NAME);
	if (error) {
		snprintf(event, sizeof(event), "EVENT=FIRMWARE_UPDATE_FAIL");
		kobject_uevent_env(&dev->kobj, KOBJ_CHANGE, envp);
		printk(KERN_INFO "[Touch] FIRMWARE_UPDATE_FAIL ---> %s (%d)\n", __FUNCTION__, __LINE__);

		dev_err(dev, "The firmware update failed(%d)\n", error);
		count = error;
	} else {
		snprintf(event, sizeof(event), "EVENT=FIRMWARE_UPDATE_SUCCESS");
		kobject_uevent_env(&dev->kobj, KOBJ_CHANGE, envp);
		printk(KERN_INFO "[Touch] FIRMWARE_UPDATE_SUCCESS ---> %s (%d)\n", __FUNCTION__, __LINE__);

		dev_dbg(dev, "The firmware update succeeded\n");

		/* Wait for reset */
		msleep(MXT_FWRESET_TIME);

		kfree(data->object_table);
		data->object_table = NULL;

#if 0
		mxt_initialize(data);
#endif
	}

#if 0
	enable_irq(data->irq);

	error = mxt_make_highchg(data);
#endif

	if (error)
		return error;

	return count;
}

static ssize_t mxt_pause_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct mxt_data *data = dev_get_drvdata(dev);
	int count = 0;

	count += sprintf(buf + count, "%d", data->driver_paused);
	count += sprintf(buf + count, "\n");

	return count;
}

static ssize_t mxt_pause_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct mxt_data *data = dev_get_drvdata(dev);
	int i;

	if (sscanf(buf, "%u", &i) == 1 && i < 2) {
		data->driver_paused = i;

		dev_dbg(dev, "%s\n", i ? "paused" : "unpaused");
	} else {
		dev_dbg(dev, "pause_driver write error\n");
	}
	return count;
}

static ssize_t mxt_debug_enable_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct mxt_data *data = dev_get_drvdata(dev);
	int count = 0;

	count += sprintf(buf + count, "%d", data->debug_enabled);
	count += sprintf(buf + count, "\n");

	return count;
}

static ssize_t mxt_debug_enable_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct mxt_data *data = dev_get_drvdata(dev);
	int i;

	if (sscanf(buf, "%u", &i) == 1 && i < 2) {
		data->debug_enabled = i;

		dev_dbg(dev, "%s\n", i ? "debug enabled" : "debug disabled");
	} else {
		dev_dbg(dev, "debug_enabled write error\n");
	}
	return count;
}

static ssize_t mxt_slowscan_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct mxt_data *data = dev_get_drvdata(dev);
	int count = 0;
	int error;
	u8 actv_cycle_time;
	u8 idle_cycle_time;
	u8 actv2idle_timeout;
	dev_info(dev, "Calling mxt_slowscan_show()\n");

	error = mxt_read_object(data, MXT_GEN_POWER_T7,
		MXT_POWER_ACTVACQINT,
		&actv_cycle_time);

	if (error)
		return error;

	error = mxt_read_object(data, MXT_GEN_POWER_T7,
		MXT_POWER_IDLEACQINT,
		&idle_cycle_time);

	if (error)
		return error;

	error = mxt_read_object(data, MXT_GEN_POWER_T7,
		MXT_POWER_ACTV2IDLETO,
		&actv2idle_timeout);

	if (error)
		return error;

	count += sprintf(buf + count, "SLOW SCAN (enable/disable) = %s.\n", data->slowscan_enabled ? "enabled" : "disabled");
	count += sprintf(buf + count, "SLOW SCAN (actv_cycle_time) = %umS.\n", data->slowscan_actv_cycle_time);
	count += sprintf(buf + count, "SLOW SCAN (idle_cycle_time) = %umS.\n", data->slowscan_idle_cycle_time);
	count += sprintf(buf + count, "SLOW SCAN (actv2idle_timeout) = %u.%0uS.\n", data->slowscan_actv2idle_timeout / 10, \
											data->slowscan_actv2idle_timeout % 10);
	count += sprintf(buf + count, "CURRENT   (actv_cycle_time) = %umS.\n", actv_cycle_time);
	count += sprintf(buf + count, "CURRENT   (idle_cycle_time) = %umS.\n", idle_cycle_time);
	count += sprintf(buf + count, "CURRENT   (actv2idle_timeout) = %u.%0uS.\n", actv2idle_timeout / 10, \
											actv2idle_timeout % 10);

	return count;
}

static ssize_t mxt_slowscan_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct mxt_data *data = dev_get_drvdata(dev);
	int fn;
	int val;
	int ret;

	dev_info(dev, "Calling mxt_slowscan_store()\n");
	ret = sscanf(buf, "%u %u", &fn, &val);
	if ((ret == 1) || (ret == 2)) {
		switch (fn) {
		case SLOSCAN_DISABLE:
			if (data->slowscan_enabled) {
				data->actv_cycle_time = data->slowscan_shad_actv_cycle_time;
				data->idle_cycle_time = data->slowscan_shad_idle_cycle_time;
				data->actv2idle_timeout = data->slowscan_shad_actv2idle_timeout;
				data->slowscan_enabled = 0;
				mxt_set_power_cfg(data, 0);
			}
			break;

		case SLOSCAN_ENABLE:
			if (!data->slowscan_enabled) {
				data->slowscan_shad_actv_cycle_time = data->actv_cycle_time;
				data->slowscan_shad_idle_cycle_time = data->idle_cycle_time;
				data->slowscan_shad_actv2idle_timeout = data->actv2idle_timeout;
				data->actv_cycle_time = data->slowscan_actv_cycle_time;
				data->idle_cycle_time = data->slowscan_idle_cycle_time;
				data->actv2idle_timeout = data->slowscan_actv2idle_timeout;
				data->slowscan_enabled = 1;
				mxt_set_power_cfg(data, 0);
			}
			break;

		case SLOSCAN_SET_ACTVACQINT:
			data->slowscan_actv_cycle_time = val;
			break;

		case SLOSCAN_SET_IDLEACQINT:
			data->slowscan_idle_cycle_time = val;
			break;

		case SLOSCAN_SET_ACTV2IDLETO:
			data->slowscan_actv2idle_timeout = val;
			break;
		}
	}
	return count;
}

#define BUF_I2C_SIZE	64

static ssize_t mxt_cfg_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct mxt_data *data = dev_get_drvdata(dev);
	struct mxt_platform_data *pdata = data->client->dev.platform_data;
	struct mxt_cfg_object *mxt_object_list = pdata->cfg_objects;
	int object_length = pdata->cfg_objects_length;
	int i = 0, j = 0;
	char *buf_iter = buf, buf_i2c[BUF_I2C_SIZE];
	u8 val_u8 = 0;
	struct mxt_object *object;
	u16 reg;
	static bool is_hex = false;
	int error;

	if (is_cfg_format_enabled)
		is_hex = !is_hex;
	else
		is_cfg_format_enabled = true;

	error = mxt_read_object(data, MXT_GEN_POWER_T7,
		MXT_POWER_ACTVACQINT,
		&val_u8);
	if (error || object_length == 0)
		return 0;

	for (i = 0; i < object_length; i++) {
		buf_iter += sprintf(buf_iter, "- T%d -", mxt_object_list[i].type);

		if (mxt_object_list[i].instance != 0)
			buf_iter += sprintf(buf_iter, "\t[%d]", mxt_object_list[i].instance);

		if (mxt_object_list[i].length <= BUF_I2C_SIZE) {
			object = mxt_get_object(data, mxt_object_list[i].type);
			if (!object)
				goto out;

			reg = object->start_address;
			error = __mxt_read_reg(data->client, reg, mxt_object_list[i].length, &buf_i2c[0]);
			if (error == -EIO)
				goto out;
		}

		for (j = 0; j < mxt_object_list[i].length; j++) {
			if (j % 5 == 0)
				buf_iter += sprintf(buf_iter, "\n");
			if (is_hex)
				buf_iter += sprintf(buf_iter, "[%02d]: 0x%02X  ", j, buf_i2c[j]);
			else
				buf_iter += sprintf(buf_iter, "[%02d]: %-4d  ", j, buf_i2c[j]);
		}

		buf_iter += sprintf(buf_iter, "\n");
	}


	buf_iter += sprintf(buf_iter, "\nBuf size: %4d\n", buf_iter - buf + 16);
						// sizeof("\nBuf size: %4d\n")
	if  (data->info.version != data->pdata->config_fw_ver)
		buf_iter += sprintf(buf_iter, "FW version (%02X) != (%02X) Cfg FW version\n",
				data->info.version, data->pdata->config_fw_ver);
	else
		buf_iter += sprintf(buf_iter, "FW version (%02X)\n",
				data->info.version);
out:
	return buf_iter - buf;
}

static ssize_t mxt_cfg_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct mxt_data *data = dev_get_drvdata(dev);
	int object = 0, offset = 0, offset_count = 0;
	int val = 0;
	u8 val_u8 = 0;
	int ret = 0;

	ret = sscanf(buf, "t%u", &object);
	if (ret == 0)
		goto out;

	buf = strchr(buf, ' ');
	if (buf)
		buf = skip_spaces(buf);
	if (buf)
		ret = sscanf(buf, "%u", &offset);

	if (ret == 0 || buf == 0)
		goto out;

	buf = strchr(buf, ' ');
	if (buf)
		buf = skip_spaces(buf);
	if (buf) {
		ret = sscanf(buf, "s 0x%X", (int *) &val);
		if (ret == 0)
			ret = sscanf(buf, "s %d", &val);
	}

	if (ret == 0 || buf == 0)
		goto out;

	printk(KERN_INFO "[Touch] T%u[%u] = 0x%02X ---> %s (%d)\n",
		object, offset + offset_count, val, __FUNCTION__, __LINE__);
	mxt_write_object(data, object, offset + offset_count, val);
	offset_count++;

	buf = strchr(buf, ' ');
	if (buf)
		buf = skip_spaces(buf);

	while (1) {
		buf = strchr(buf, ' ');
		if (buf)
			buf = skip_spaces(buf);
		if (buf) {
			ret = sscanf(buf, "0x%X", (int *) &val);
			if (ret == 0)
				ret = sscanf(buf, "%d", &val);
		}

		if (ret == 0 || buf == 0)
			break;

		printk(KERN_INFO "[Touch] T%u[%u] = 0x%02X ---> %s (%d)\n",
			object, offset + offset_count, val, __FUNCTION__, __LINE__);
		mxt_write_object(data, object, offset + offset_count, val);
		offset_count++;
	}

	ret = mxt_read_object(data, MXT_GEN_POWER_T7, MXT_POWER_IDLEACQINT, &val_u8);
	if (ret == 0 && val_u8 != 0)
		data->idle_cycle_time = val_u8;
	ret = mxt_read_object(data, MXT_GEN_POWER_T7, MXT_POWER_ACTVACQINT, &val_u8);
	if (ret == 0 && val_u8 != 0)
		data->actv_cycle_time = val_u8;
	ret = mxt_read_object(data, MXT_GEN_POWER_T7, MXT_POWER_ACTV2IDLETO, &val_u8);
	if (ret == 0 && val_u8 != 0)
		data->actv2idle_timeout= val_u8;

	is_cfg_format_enabled = false;
out:
	return count;
}

static ssize_t mxt_fw_version_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct mxt_data *data = dev_get_drvdata(dev);
	int count = 0;

	count += sprintf(buf + count, "FW version: %02X build: %02X\n",
					data->info.version, data->info.build);

	return count;
}

static ssize_t mxt_mem_access_read(struct file *filp, struct kobject *kobj,
	struct bin_attribute *bin_attr, char *buf, loff_t off, size_t count)
{
	struct device *dev = container_of(kobj, struct device, kobj);
	struct mxt_data *data = dev_get_drvdata(dev);
	int ret = 0;

	if (off >= 32768)
		return -EIO;

	if (off + count > 32768)
		count = 32768 - off;

	if (count > 256)
		count = 256;

	if (count > 0)
		ret = __mxt_read_reg(data->client, off, count, buf);

	return ret == 0 ? count : ret;
}

int mxt_write_block(struct i2c_client *client, u16 addr, u16 length, u8 *value)
{
	int i;
	struct {
		__le16 le_addr;
		u8  data[256];
	} i2c_block_transfer;

	if (length > 256)
		return -EINVAL;

	i2c_get_clientdata(client);

	for (i = 0; i < length; i++)
		i2c_block_transfer.data[i] = *value++;

	i2c_block_transfer.le_addr = cpu_to_le16(addr);

	i = i2c_master_send(client, (u8 *) &i2c_block_transfer, length + 2);

	if (i == (length + 2))
		return 0;
	else
		return -EIO;
}

static ssize_t mxt_mem_access_write(struct file *filp, struct kobject *kobj,
	struct bin_attribute *bin_attr, char *buf, loff_t off, size_t count)
{
	struct device *dev = container_of(kobj, struct device, kobj);
	struct mxt_data *data = dev_get_drvdata(dev);
	int ret = 0;

	if (off >= 32768)
		return -EIO;

	if (off + count > 32768)
		count = 32768 - off;

	if (count > 256)
		count = 256;

	if (count > 0)
		ret = mxt_write_block(data->client, off, count, buf);

	return ret == 0 ? count : 0;
}

static DEVICE_ATTR(object, 0444, mxt_object_show, NULL);
static DEVICE_ATTR(fw_version, 0644, mxt_fw_version_show, NULL);
static DEVICE_ATTR(cfg, 0644, mxt_cfg_show, mxt_cfg_store);
static DEVICE_ATTR(update_fw, 0644, NULL, mxt_update_fw_store);
static DEVICE_ATTR(pause_driver, 0644, mxt_pause_show, mxt_pause_store);
static DEVICE_ATTR(debug_enable, 0644, mxt_debug_enable_show, mxt_debug_enable_store);
static DEVICE_ATTR(slowscan_enable, 0644, mxt_slowscan_show, mxt_slowscan_store);
static DEVICE_ATTR(id_information, 0644, mxt_id_information_show, NULL);

static struct attribute *mxt_attrs[] = {
	&dev_attr_object.attr,
	&dev_attr_cfg.attr,
	&dev_attr_fw_version.attr,
#if 0
	&dev_attr_update_fw.attr,
#endif
	&dev_attr_pause_driver.attr,
	&dev_attr_debug_enable.attr,
	&dev_attr_slowscan_enable.attr,
	&dev_attr_id_information.attr,
	NULL
};

static const struct attribute_group mxt_attr_group = {
	.attrs = mxt_attrs,
};

static void mxt_auto_cal(struct mxt_data *data, bool is_enable)
{
	int error = 0;
	int atchcalst = 0xFF, atchcalsthr = 1, atchfrccalst = 0, atchfrccalratio = 0;

	if (is_enable) {
		atchcalst = 0;
		atchcalsthr = 0;
		atchfrccalst = 50;
		atchfrccalratio = 25;
	}

	error = mxt_write_object(data, MXT_GEN_ACQUIRE_T8, MXT_ACQUIRE_ATCHCALST,
				atchcalst);
	if (error)
		goto i2c_error;

	error = mxt_write_object(data, MXT_GEN_ACQUIRE_T8, MXT_ACQUIRE_ATCHCALSTHR,
			 atchcalsthr);
	if (error)
		goto i2c_error;

	error = mxt_write_object(data, MXT_GEN_ACQUIRE_T8, MXT_ACQUIRE_ATCHFRCCALST,
			 atchfrccalst);
	if (error)
		goto i2c_error;

	error = mxt_write_object(data, MXT_GEN_ACQUIRE_T8, MXT_ACQUIRE_ATCHFRCCALRATIO,
			 atchfrccalratio);
	if (error)
		goto i2c_error;

	return;

i2c_error:
	dev_err(&data->client->dev, "Failed to set auto calibration");
}

static void mxt_worker_no_auto_cal(struct work_struct *work)
{
	struct mxt_data *data = container_of(work, struct mxt_data, dwork_no_auto_cal.work);

	mxt_auto_cal(data, false);
}

static void mxt_start(struct mxt_data *data)
{
	int error = 0;
	int id;
	struct device *dev = &data->client->dev;
	struct mxt_finger *finger = data->finger;

	//dev_info(dev, "mxt_start:  is_stopped = %d\n", data->is_stopped);
	if (data->is_stopped == 0)
		return;

	for (id = 0; id < MXT_MAX_FINGER; id++) {
		if (finger[id].status & (MXT_MOVE | MXT_PRESS))
			finger[id].status = MXT_RELEASE;
	}
	mxt_input_report(data, id);

#if 0
		/* Touch enable */
		error = mxt_write_object(data, MXT_TOUCH_MULTI_T9, MXT_TOUCH_CTRL, MXT_TOUCH_ENABLE);
#else
	if (machine_is_sphinx()) {
		if (data->read_chg() == 0)
			schedule_delayed_work(&data->dwork, 0);
	} else {
		if (data->pdata->gpio_wake) {
			gpio_set_value(data->pdata->gpio_wake, 0);
			msleep(25);
		}

		if(gpio_get_value(irq_to_gpio(data->irq)) == 0) {
			mxt_make_highchg(data);
			printk(KERN_INFO "[Touch] mxt_make_highchg!!\n");
		}

		error = mxt_set_power_cfg(data, 0);
		mxt_write_object(data, MXT_GEN_COMMAND_T6, MXT_COMMAND_CALIBRATE, 1);

		mxt_auto_cal(data, true);
		schedule_delayed_work(&data->dwork_no_auto_cal, msecs_to_jiffies(AUTO_CAL_TIMEOUT * 1000));
		schedule_delayed_work(&data->dwork, 0);
	}
#endif
	if (error)
		dev_info(dev, "MXT enable touch fail\n");

	data->is_stopped = 0;
}

static void mxt_stop(struct mxt_data *data)
{
	int error = 0;
	//int id;
	struct device *dev = &data->client->dev;
	//struct mxt_finger *finger = data->finger;

	//dev_info(dev, "mxt_stop:  is_stopped = %d\n", data->is_stopped);
	if (data->is_stopped)
		return;

#if 0
		/* Touch disable */
		error = mxt_write_object(data, MXT_TOUCH_MULTI_T9, MXT_TOUCH_CTRL, MXT_TOUCH_DISABLE);
#else
	if (machine_is_sphinx()) {
		cancel_delayed_work_sync(&data->dwork);
	} else {
		cancel_delayed_work_sync(&data->dwork_no_auto_cal);
		cancel_delayed_work_sync(&data->dwork);

		if (data->pdata->gpio_wake)
			gpio_set_value(data->pdata->gpio_wake, 1);

		error = mxt_set_power_cfg(data, 1);
	}
#endif

	if (error)
		dev_info(dev, "MXT disable touch fail.\n");

	/*error = mxt_make_highchg(data);
	if (error) {
		dev_err(&data->client->dev, "Failed to make high CHG\n");
	}

	for (id = 0; id < MXT_MAX_FINGER; id++) {
		if (finger[id].status & (MXT_MOVE | MXT_PRESS))
			finger[id].status = MXT_RELEASE;
	}
	mxt_input_report(data, id);*/

	data->is_stopped = 1;
}

static int mxt_input_open(struct input_dev *dev)
{
	struct mxt_data *data = input_get_drvdata(dev);

	mxt_start(data);

	return 0;
}

static void mxt_input_close(struct input_dev *dev)
{
	struct mxt_data *data = input_get_drvdata(dev);

	mxt_stop(data);
}

static int __devinit mxt_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	const struct mxt_platform_data *pdata = client->dev.platform_data;
	struct mxt_data *data;
	struct input_dev *input_dev;
	int error, i = 0;
	u8 val_u8 = 0;
	int data_num;

	if (!pdata)
		return -EINVAL;

	data = kzalloc(sizeof(struct mxt_data), GFP_KERNEL);
	input_dev = input_allocate_device();
	if (!data || !input_dev) {
		dev_err(&client->dev, "Failed to allocate memory\n");
		error = -ENOMEM;
		goto err_free_mem;
	}

	input_dev->name = "atmel-maxtouch";
	input_dev->id.bustype = BUS_I2C;
	input_dev->dev.parent = &client->dev;
	input_dev->open = mxt_input_open;
	input_dev->close = mxt_input_close;

	data->client = client;
	data->input_dev = input_dev;
	data->pdata = pdata;
	data->irq = client->irq;
	data->is_stopped = 0;

	i2c_set_clientdata(client, data);
	mutex_init(&data->access_mutex);

	error = mxt_get_info(data);
	if (error)
		goto err_free_mem;

	data_num = data->pdata->data_num;

	for (i = 0; i < data_num; i++) {
		if (data->pdata->config_fw_ver == data->info.version)
			break;
		else if (i == data_num - 1)
			break;
		else
			data->pdata++;
	}

	mxt_calc_resolution(data);

	__set_bit(EV_ABS, input_dev->evbit);
	__set_bit(EV_KEY, input_dev->evbit);
	__set_bit(BTN_TOUCH, input_dev->keybit);

	/* For single touch */
	input_set_abs_params(input_dev, ABS_X,
			     0, data->max_x, 0, 0);
	input_set_abs_params(input_dev, ABS_Y,
			     0, data->max_y, 0, 0);

	/* For multi touch */
	input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR,
			     0, MXT_MAX_AREA, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_X,
			     0, data->max_x, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_Y,
			     0, data->max_y, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_TRACKING_ID, 0, MXT_MAX_FINGER, 0, 0);

	input_set_drvdata(input_dev, data);

	error = device_create_file(&client->dev, &dev_attr_update_fw);
	if (error)
		printk(KERN_INFO "[Touch] - ERROR - Create dev_attr_update_fw fail ---> %s (%d)\n", __FUNCTION__, __LINE__);

	error = mxt_initialize(data);
	if (error) {
		if (client->addr >= MXT_APP_LOW)
			client->addr -= 0x26;
		printk(KERN_INFO "[Touch] client->addr: 0x%X ---> %s (%d)\n",client->addr , __FUNCTION__, __LINE__);

		error = mxt_check_bootloader(client, MXT_WAITING_BOOTLOAD_CMD);
		if (error)
			goto err_free_object;

		bootloader_mode = 1;
		client->addr += 0x26;
		return 0;
	}

	if (machine_is_sphinx()) {
		INIT_DELAYED_WORK(&data->dwork, mxt_worker);
	}
	else {
		for (i = 0; i < pdata->cfg_noises_length; i++) {
			error = mxt_read_object(data, pdata->cfg_noises[i].type,
					pdata->cfg_noises[i].offset,
					&val_u8);
			pdata->cfg_noises[i].normal_value = val_u8;
		}

		INIT_DELAYED_WORK(&data->dwork_no_auto_cal, mxt_worker_no_auto_cal);
		INIT_DELAYED_WORK(&data->dwork, mxt_noise_worker);
		mxt_auto_cal(data, true);
		schedule_delayed_work(&data->dwork, 0);
		schedule_delayed_work(&data->dwork_no_auto_cal, msecs_to_jiffies(30 * 1000));

	}

	error = request_threaded_irq(client->irq, NULL, mxt_interrupt,
			pdata->irqflags, client->dev.driver->name, data);
	if (error) {
		dev_err(&client->dev, "Failed to register interrupt\n");
		goto err_free_object;
	}

	if (pdata->gpio_ac)
	{
		error = request_threaded_irq(gpio_to_irq(pdata->gpio_ac), NULL, mxt_interrupt_noise,
				IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING | IRQF_SHARED, "AC detection for touch", data);
		if (error) {
			dev_err(&data->client->dev, "Failed to register interrupt for AC: %d\n", error);
			goto err_free_object;
		}
	}

	if (pdata->gpio_hdmi)
	{
		error = request_threaded_irq(gpio_to_irq(pdata->gpio_hdmi), NULL, mxt_interrupt_noise,
				IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING | IRQF_SHARED, "HDMI detection for touch", data);
		if (error) {
			dev_err(&data->client->dev, "Failed to register interrupt for HDMI: %d\n", error);
			goto err_free_object;
		}
	}

#if defined(CONFIG_HAS_EARLYSUSPEND)
	if (machine_is_sphinx()) {
		data->early_suspend.level = EARLY_SUSPEND_LEVEL_DISABLE_FB - 1;
	} else {
		data->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	}
	data->early_suspend.suspend = mxt_early_suspend;
	data->early_suspend.resume = mxt_early_resume;
	register_early_suspend(&data->early_suspend);
#endif

	error = mxt_make_highchg(data);
	if (error) {
		dev_err(&client->dev, "Failed to make high CHG\n");
		goto err_free_irq;
	}

	error = input_register_device(input_dev);
	if (error) {
		dev_err(&client->dev, "Failed to register input device\n");
		goto err_free_irq;
	}

	error = sysfs_create_group(&client->dev.kobj, &mxt_attr_group);
	if (error) {
		dev_err(&client->dev, "Failed to create sysfs group\n");
		goto err_unregister_device;
	}

	sysfs_bin_attr_init(&data->mem_access_attr);
	data->mem_access_attr.attr.name = "mem_access";
	data->mem_access_attr.attr.mode = S_IRUGO | S_IWUSR | S_IWGRP;
	data->mem_access_attr.read = mxt_mem_access_read;
	data->mem_access_attr.write = mxt_mem_access_write;
	data->mem_access_attr.size = 65535;

	if (sysfs_create_bin_file(&client->dev.kobj, &data->mem_access_attr) < 0) {
		dev_err(&client->dev, "Failed to create %s\n", data->mem_access_attr.attr.name);
		goto err_unregister_device;
	}

	mutex_init(&data->debug_mutex);

	/* Create debugfs entries. */
	data->debug_dir = debugfs_create_dir("maXTouch", NULL);
	if (data->debug_dir == ERR_PTR(-ENODEV)) {
		/* debugfs is not enabled. */
		printk(KERN_WARNING "debugfs not enabled in kernel\n");
	} else if (data->debug_dir == NULL) {
		printk(KERN_WARNING "error creating debugfs dir\n");
	} else {
		debugfs_create_file("deltas", S_IRUSR, data->debug_dir, data,
				    &delta_fops);
		debugfs_create_file("refs", S_IRUSR, data->debug_dir, data,
				    &refs_fops);
		debugfs_create_file("SELFTEST", S_IFREG | S_IRUSR, data->debug_dir, data,
				    &selftest_fops);
	}

	if (machine_is_sphinx()) {
		/* Create character device nodes for reading & writing registers */
		data->mxt_class = class_create(THIS_MODULE, "maXTouch_memory");

		error = alloc_chrdev_region(&data->dev_num, 0, 1, "maXTouch_memory");
		/*mxt_debug(DEBUG_VERBOSE,
			  "device number %d allocated!\n", MAJOR(mxt->dev_num));*/
		if (error)
			printk(KERN_WARNING "Error registering device\n");
		cdev_init(&data->cdev, &mxt_memory_fops);

		//mxt_debug(DEBUG_VERBOSE, "cdev initialized\n");
		data->cdev.owner = THIS_MODULE;

		error = cdev_add(&data->cdev, data->dev_num, 1);
		if (error)
			printk(KERN_WARNING "Bad cdev\n");

		//mxt_debug(DEBUG_VERBOSE, "cdev added\n");

		device_create(data->mxt_class, NULL, MKDEV(MAJOR(data->dev_num), 0), NULL,
			      "maXTouch");

		/* Schedule a worker routine to read any messages that might have
		 * been sent before interrupts were enabled. */
		cancel_delayed_work(&data->dwork);
		schedule_delayed_work(&data->dwork, 0);
	}

	return 0;

err_unregister_device:
	input_unregister_device(input_dev);
	input_dev = NULL;
err_free_irq:
	free_irq(client->irq, data);
err_free_object:
	kfree(data->object_table);
err_free_mem:
	input_free_device(input_dev);
	kfree(data);
	return error;
}

static int __devexit mxt_remove(struct i2c_client *client)
{
	struct mxt_data *data = i2c_get_clientdata(client);

	/* Remove debug dir entries */
	debugfs_remove_recursive(data->debug_dir);
	debugfs_remove(data->debug_dir);

	if (machine_is_sphinx()) {
		unregister_chrdev_region(data->dev_num, 1);
		device_destroy(data->mxt_class, MKDEV(MAJOR(data->dev_num), 0));
		cdev_del(&data->cdev);

		cancel_delayed_work_sync(&data->dwork);
	}
	else {
		cancel_delayed_work_sync(&data->dwork);
		cancel_delayed_work_sync(&data->dwork_no_auto_cal);
	}

	sysfs_remove_bin_file(&client->dev.kobj, &data->mem_access_attr);
	sysfs_remove_group(&client->dev.kobj, &mxt_attr_group);
	device_remove_file(&client->dev, &dev_attr_update_fw);
	free_irq(data->irq, data);
	input_unregister_device(data->input_dev);
	kfree(data->object_table);
	kfree(data);

	return 0;
}

#ifdef CONFIG_PM
static int mxt_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct mxt_data *data = i2c_get_clientdata(client);
	struct input_dev *input_dev = data->input_dev;

	mutex_lock(&input_dev->mutex);

	if (input_dev->users)
		mxt_stop(data);

	mutex_unlock(&input_dev->mutex);

	return 0;
}

static int mxt_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct mxt_data *data = i2c_get_clientdata(client);
	struct input_dev *input_dev = data->input_dev;

#if 0
	/* Soft reset */
	if (!machine_is_sphinx()) {
		mxt_write_object(data, MXT_GEN_COMMAND_T6, MXT_COMMAND_RESET, 1);

		msleep(MXT_RESET_TIME);
	}
#endif

	mutex_lock(&input_dev->mutex);

	if (input_dev->users)
		mxt_start(data);

	mutex_unlock(&input_dev->mutex);

	return 0;
}

#if defined(CONFIG_HAS_EARLYSUSPEND)
static void mxt_early_suspend(struct early_suspend *es)
{
	struct mxt_data *mxt;
	struct device *dev;
	mxt = container_of(es, struct mxt_data, early_suspend);
	dev = &mxt->client->dev;
	//dev_info(dev, "MXT Early Suspend entered\n");

	if (machine_is_sphinx())
		disable_irq(mxt->irq);

	if (mxt_suspend(&mxt->client->dev) != 0)
		dev_err(dev, "%s: failed\n", __func__);
	//dev_info(dev, "MXT Early Suspended\n");
}

static void mxt_early_resume(struct early_suspend *es)
{
	struct mxt_data *mxt;
	struct device *dev;
	mxt = container_of(es, struct mxt_data, early_suspend);
	dev = &mxt->client->dev;
	//dev_info(dev, "MXT Early Resume entered\n");

	if (mxt_resume(&mxt->client->dev) != 0)
		dev_err(dev, "%s: failed\n", __func__);
	//dev_info(dev, "MXT Early Resumed\n");

	if (machine_is_sphinx())
		enable_irq(mxt->irq);
}
#else
static const struct dev_pm_ops mxt_pm_ops = {
	.suspend	= mxt_suspend,
	.resume		= mxt_resume,
};
#endif
#endif

static const struct i2c_device_id mxt_id[] = {
	{ "qt602240_ts", 0 },
	{ "atmel_mxt_ts", 0 },
	{ "mXT224", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, mxt_id);

static struct i2c_driver mxt_driver = {
	.driver = {
		.name	= "atmel_mxt_ts",
		.owner	= THIS_MODULE,
#if defined(CONFIG_PM) && !defined(CONFIG_HAS_EARLYSUSPEND)
		.pm	= &mxt_pm_ops,
#endif
	},
	.probe		= mxt_probe,
	.remove		= __devexit_p(mxt_remove),
	.id_table	= mxt_id,
};

static int __init mxt_init(void)
{
	return i2c_add_driver(&mxt_driver);
}

static void __exit mxt_exit(void)
{
	i2c_del_driver(&mxt_driver);
}

module_init(mxt_init);
module_exit(mxt_exit);

/* Module information */
MODULE_AUTHOR("Joonyoung Shim <jy0922.shim@samsung.com>");
MODULE_DESCRIPTION("Atmel maXTouch Touchscreen driver");
MODULE_LICENSE("GPL");
