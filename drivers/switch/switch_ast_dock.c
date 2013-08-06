/*
 * drivers/input/misc/tegra_odm_dock.c
 *
 * Dock input device using NVIDIA Tegra ODM kit
 *
 * Copyright (c) 2009, NVIDIA Corporation.
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
#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/platform_device.h>
#include <linux/switch.h>
#include <linux/earlysuspend.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/ast_dock.h>
#include <linux/delay.h>
#include <linux/workqueue.h>
#include <linux/mfd/nvtec.h>

struct tegra_dock_dev
{
	struct switch_dev sdev;
	struct early_suspend dock_early_suspend;
    int dock_on_pin;
    int dock_amp_enb_pin;
	int ac_dock_pin;
    struct delayed_work work;
	struct notifier_block ac_dock_notifier;
};

static BLOCKING_NOTIFIER_HEAD(ast_dock_notifier);

int ast_dock_notifier_register(struct notifier_block *nb)
{
    return blocking_notifier_chain_register(&ast_dock_notifier, nb);
}
EXPORT_SYMBOL(ast_dock_notifier_register);

int ast_dock_notifier_unregister(struct notifier_block *nb)
{
    return blocking_notifier_chain_unregister(&ast_dock_notifier, nb);
}
EXPORT_SYMBOL(ast_dock_notifier_unregister);

static void nvdock_detect_worker(struct work_struct *work)
{
	struct tegra_dock_dev *dock =
		container_of(to_delayed_work(work), struct tegra_dock_dev, work);
	int value, ac_value;

	value = !gpio_get_value(dock->dock_on_pin);
    switch_set_state(&dock->sdev, value);
	if (value) {                /* device on dock */
		ac_value = gpio_get_value(dock->ac_dock_pin);
		if (ac_value) {         /* AC adapter is inserted */
			gpio_direction_output(dock->dock_amp_enb_pin, 1);
		} else {                /* AC adapter is removed */
			gpio_direction_output(dock->dock_amp_enb_pin, 0);
		}
	} else {                    /* device not on dock */
        gpio_direction_output(dock->dock_amp_enb_pin, 0);
	}
	
    blocking_notifier_call_chain(&ast_dock_notifier, value, NULL);
}

static irqreturn_t nvdock_irq(int irq, void *data)
{
    struct tegra_dock_dev *dock = (struct tegra_dock_dev *)data;

	queue_delayed_work(system_nrt_wq, &dock->work,
					   msecs_to_jiffies(40));

    return IRQ_HANDLED;
}

static int ac_dock_notify(struct notifier_block *nb, unsigned long state, void *unused)
{
    struct tegra_dock_dev *dock =
        container_of(nb, struct tegra_dock_dev, ac_dock_notifier);
	int value;
	
	value = !gpio_get_value(dock->dock_on_pin);
	
	if (value) {                /* device on dock */
		if (state) {   /* ac inserted */
			gpio_direction_output(dock->dock_amp_enb_pin, 1);
		} else {   /* ac removed */
			gpio_direction_output(dock->dock_amp_enb_pin, 0);
		}
	} else {                    /* device not on dock */
		gpio_direction_output(dock->dock_amp_enb_pin, 0);
	}
    
    return NOTIFY_OK;
}

static ssize_t dock_print_state(struct switch_dev *sdev, char *buf)
{
    struct tegra_dock_dev *dock =
        container_of(sdev, struct tegra_dock_dev, sdev);

	return sprintf(buf, "%d\n", !gpio_get_value(dock->dock_on_pin));
}

static void tegra_dock_early_suspend(struct early_suspend *h)
{
    return;
}

static void tegra_dock_late_resume(struct early_suspend *h)
{
    struct tegra_dock_dev *dock =
        container_of(h, struct tegra_dock_dev, dock_early_suspend);

    queue_delayed_work(system_nrt_wq, &dock->work,
                       msecs_to_jiffies(40));

	return;
}

static int __devinit tegra_dock_probe(struct platform_device *pdev)
{
	struct dock_platform_data *pdata = pdev->dev.platform_data;
	struct tegra_dock_dev *dock = NULL;
	int ret = 0;

    dock = kzalloc(sizeof(struct tegra_dock_dev), GFP_KERNEL);
	if (!dock) {
		printk("Fails to alloc dock device\n");
		return -ENOMEM;
	}

	dock->dock_on_pin = pdata->dock_on_pin;
    dock->dock_amp_enb_pin = pdata->dock_amp_enb_pin;
	dock->ac_dock_pin = pdata->ac_dock_pin;
	
	INIT_DELAYED_WORK(&dock->work, nvdock_detect_worker);

	ret = request_threaded_irq(pdata->irq, NULL, nvdock_irq, 
                               IRQF_TRIGGER_RISING|IRQF_TRIGGER_FALLING, 
                               "nvdock", dock);
    if (ret) {
        goto fail_request_irq;
    }

    enable_irq_wake(pdata->irq);

	dock->sdev.name = "dock";
    dock->sdev.print_state = dock_print_state;
	ret = switch_dev_register(&dock->sdev);
	if (ret) {
		goto fail_register_switch;
	}

	platform_set_drvdata(pdev, dock);

	queue_delayed_work(system_nrt_wq, &dock->work,
					   msecs_to_jiffies(40));

	dock->dock_early_suspend.suspend = tegra_dock_early_suspend;
	dock->dock_early_suspend.resume = tegra_dock_late_resume;
	register_early_suspend(&dock->dock_early_suspend);

	dock->ac_dock_notifier.notifier_call = ac_dock_notify;
    nvtec_ac_notifier_register(&dock->ac_dock_notifier);
	
    printk("Registered dock switch driver\n");

	return 0;

 fail_register_switch:
    free_irq(pdata->irq, dock);
 fail_request_irq:
    gpio_free(dock->dock_on_pin);
    gpio_free(dock->dock_amp_enb_pin);
	kfree(dock);

	return ret;
}

static int __devexit tegra_dock_remove(struct platform_device *pdev)
{
    struct dock_platform_data *pdata = pdev->dev.platform_data;
	struct tegra_dock_dev *dock = platform_get_drvdata(pdev);

    nvtec_ac_notifier_unregister(&dock->ac_dock_notifier);
    unregister_early_suspend(&dock->dock_early_suspend);
    switch_dev_unregister(&dock->sdev);
    free_irq(pdata->irq, dock);
    gpio_free(dock->dock_on_pin);
    gpio_free(dock->dock_amp_enb_pin);
	kfree(dock);

	return 0;
}

static struct platform_driver tegra_dock_driver = {
	.probe	= tegra_dock_probe,
	.remove	= tegra_dock_remove,
	.driver	= {
		.name = "tegra_dock",
	},
};

static int __init tegra_dock_init(void)
{
	return platform_driver_register(&tegra_dock_driver);
}

static void __exit tegra_dock_exit(void)
{
	platform_driver_unregister(&tegra_dock_driver);
}

module_init(tegra_dock_init);
module_exit(tegra_dock_exit);

MODULE_AUTHOR("Levi Huang <levi_huang@pegatroncorp.com>");
MODULE_DESCRIPTION("Dock driver for Antares");
MODULE_LICENSE("GPL");

