/******************************************************************************
 * tegra_info.c - Linux kernel module for PEGATRON
 *
 * Copyright 2008-2011 Pegatron Corporation.
 *
 * DESCRIPTION:
 *  - This is the linux driver for querying TEGRA information 
 *
 * modification history
 * --------------------
 * v0.1   2011/12/27, Query Tegra's UID, revision, and SKU
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 ******************************************************************************/

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/errno.h>
#include <linux/fs.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include <linux/ioctl.h>
#include <linux/types.h>
#include <linux/input.h>
#include <linux/miscdevice.h> /* for misc register, let user to use ioctl */
#include "tegra_info.h"

extern unsigned long long tegra_chip_uid(void);
extern int tegra_sku_id(void);
extern enum tegra_revision tegra_get_revision(void);


static int tegrainfo_open(struct inode *inode, struct file *filp)
{
	return 0;
}

static int tegrainfo_close(struct inode *inode, struct file *filp)
{
	return 0;
}

static long tegrainfo_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	int retval = 0;
    u64 chip_id = 0;
	
    switch (cmd)
    {
        case TEGRAINFO_G_UID:
            chip_id = tegra_chip_uid();
            retval = copy_to_user(argp, &chip_id, sizeof(chip_id));
            break;
        case TEGRAINFO_G_REVISION:
            //printk(KERN_INFO "CPU Revision	: %s(%d)\n", tegra_revision_name[tegra_get_revision()], strlen(tegra_revision_name[tegra_get_revision()]));
            retval = copy_to_user(argp, tegra_get_revision_name(), strlen(tegra_get_revision_name())+1);
            break;
        case TEGRAINFO_G_SKU:
            //printk(KERN_INFO "SKU ID	: %d\n", tegra_sku_id());
            retval = tegra_sku_id();
            break;
        default:
			retval = -EINVAL;
    }

    return retval;
}


static const struct file_operations tegrainfo_fops = {
	.owner	 = THIS_MODULE,
	.open	 = tegrainfo_open,
	.release = tegrainfo_close,
	.unlocked_ioctl	 = tegrainfo_ioctl,
};

static struct miscdevice tegrainfo_dev = {
	.name	= "tegra_info",
	.fops	= &tegrainfo_fops,
	.minor	= MISC_DYNAMIC_MINOR,
};


static int __devinit tegrainfo_probe(struct platform_device *pdev)
{
	int rv = 0, ret;
    
	ret = misc_register(&tegrainfo_dev);
	if (ret) {
		printk(KERN_ERR "dmiepp: failed to register misc, err=%d\n", rv);
		goto out1;
	}

	//printk(KERN_NOTICE DRIVER_NAME ": tegrainfo_probe() OUT\n");
	return 0;

out1:
    return ret;
}

static int tegrainfo_remove(struct platform_device *pdev)
{
	misc_deregister(&tegrainfo_dev);
    //printk(KERN_INFO "tegrainfo is removed\n");
	return 0;
}


static struct platform_driver tegrainfo_driver = {
    .probe      = tegrainfo_probe,
    .remove     = tegrainfo_remove,
    .driver     = {
        .name   = DRIVER_NAME,
    },
};

static int __init tegrainfo_init(void)
{
    int rv;
    printk(KERN_NOTICE DRIVER_NAME ": version %s loaded\n", DRIVER_VERSION);
    rv = platform_driver_register(&tegrainfo_driver);
    //printk(KERN_NOTICE DRIVER_NAME ": platform_driver_register rv=%d\n", rv);
    return rv;
}

static void __exit tegrainfo_exit(void)
{
    platform_driver_unregister(&tegrainfo_driver);
}

module_init(tegrainfo_init);
module_exit(tegrainfo_exit);

MODULE_VERSION(DRIVER_VERSION);
MODULE_AUTHOR("tom_sun@pegatroncorp.com");
MODULE_LICENSE("GPL v2");

