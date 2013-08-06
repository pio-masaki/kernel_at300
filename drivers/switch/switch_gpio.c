/*
 *  drivers/switch/switch_gpio.c
 *
 * Copyright (C) 2008 Google, Inc.
 * Author: Mike Lockwood <lockwood@android.com>
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

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/switch.h>
#include <linux/workqueue.h>
#include <linux/gpio.h>
#include <linux/notifier.h>
#include <linux/switch_gpio.h>
#include <linux/mfd/nvtec.h>

struct gpio_switch_data {
	struct switch_dev sdev;
	unsigned gpio;
	const char *name_on;
	const char *name_off;
	const char *state_on;
	const char *state_off;
	int irq;
	struct work_struct work;
	/* default active low */
	int state_locked;
	int state_unlocked;
};

static BLOCKING_NOTIFIER_HEAD(switch_gpio_notifier);
static BLOCKING_NOTIFIER_HEAD(switch_gpio_lock_hardware_notifier);
static bool lock_hardware = false;

int switch_gpio_notifier_register(struct notifier_block *nb)
{
    return blocking_notifier_chain_register(&switch_gpio_notifier, nb);
}
EXPORT_SYMBOL(switch_gpio_notifier_register);

int switch_gpio_notifier_unregister(struct notifier_block *nb)
{
    return blocking_notifier_chain_unregister(&switch_gpio_notifier, nb);
}
EXPORT_SYMBOL(switch_gpio_notifier_unregister);

int switch_gpio_lock_hardware_notifier_register(struct notifier_block *nb)
{
    return blocking_notifier_chain_register(&switch_gpio_lock_hardware_notifier, nb);
}
EXPORT_SYMBOL(switch_gpio_lock_hardware_notifier_register);

int switch_gpio_lock_hardware_notifier_unregister(struct notifier_block *nb)
{
    return blocking_notifier_chain_unregister(&switch_gpio_lock_hardware_notifier, nb);
}
EXPORT_SYMBOL(switch_gpio_lock_hardware_notifier_unregister);

static void gpio_switch_work(struct work_struct *work)
{
	int state;
	int report_value = -1;
	struct gpio_switch_data	*data =
		container_of(work, struct gpio_switch_data, work);

	state = gpio_get_value(data->gpio);

	if (state == data->state_locked) {
		report_value = SWITCH_LOCKED;
	} else if (state == data->state_unlocked) {
		report_value = SWITCH_UNLOCKED;
	}

	switch_set_state(&data->sdev, report_value);
	blocking_notifier_call_chain(
			&switch_gpio_notifier, 
			lock_hardware ? report_value : SWITCH_UNLOCKED, 
			NULL);
}

static irqreturn_t gpio_irq_handler(int irq, void *dev_id)
{
	struct gpio_switch_data *switch_data =
	    (struct gpio_switch_data *)dev_id;

	schedule_work(&switch_data->work);
	return IRQ_HANDLED;
}

static ssize_t switch_gpio_print_state(struct switch_dev *sdev, char *buf)
{
	struct gpio_switch_data	*switch_data =
		container_of(sdev, struct gpio_switch_data, sdev);
	const char *state;
	if (switch_get_state(sdev))
		state = switch_data->state_on;
	else
		state = switch_data->state_off;

	if (state)
		return sprintf(buf, "%s\n", state);
	return -1;
}

static ssize_t lock_hardware_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%s\n", lock_hardware ? "enable" : "disable");
}

static ssize_t lock_hardware_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int ret = count;
	struct switch_dev *sdev = (struct switch_dev *)dev_get_drvdata(dev);
	struct gpio_switch_data	*switch_data =
		container_of(sdev, struct gpio_switch_data, sdev);

	if (!strncmp(buf, "enable", 6)) {
		lock_hardware = true;
	} else if (!strncmp(buf, "disable", 7)) {
		lock_hardware = false;
	} else {
		ret = 0;
	}

	blocking_notifier_call_chain(
			&switch_gpio_lock_hardware_notifier, 
			lock_hardware ? SWITCH_LOCK_ENABLED : SWITCH_LOCK_DISABLED,
			NULL);

	schedule_work(&switch_data->work);

	return ret;
}

static DEVICE_ATTR(lock_hardware, 0600, lock_hardware_show, lock_hardware_store);

static int gpio_switch_probe(struct platform_device *pdev)
{
	struct gpio_switch_platform_data *pdata = pdev->dev.platform_data;
	struct gpio_switch_data *switch_data;
	int ret = 0;

	if (!pdata)
		return -EBUSY;

	switch_data = kzalloc(sizeof(struct gpio_switch_data), GFP_KERNEL);
	if (!switch_data)
		return -ENOMEM;

	switch_data->sdev.name = pdata->name;
    switch_data->gpio = pdata->gpio;
    switch_data->name_on = pdata->name_on;
	switch_data->name_off = pdata->name_off;
	switch_data->state_on = pdata->state_on;
	switch_data->state_off = pdata->state_off;
	switch_data->sdev.print_state = switch_gpio_print_state;

	if (pdata->active_low == 1) {
		switch_data->state_locked = 0;
		switch_data->state_unlocked = 1;
	} else {
		switch_data->state_locked = 1;
		switch_data->state_unlocked = 0;
	}

    ret = switch_dev_register(&switch_data->sdev);
	if (ret < 0)
		goto err_switch_dev_register;

	ret = device_create_file(switch_data->sdev.dev, &dev_attr_lock_hardware);
	if (ret < 0)
		printk("%s: create sys node lock hardware failed\n", __func__);

	ret = gpio_request(switch_data->gpio, pdev->name);
	if (ret < 0)
		goto err_request_gpio;

	ret = gpio_direction_input(switch_data->gpio);
	if (ret < 0)
		goto err_set_gpio_input;

	INIT_WORK(&switch_data->work, gpio_switch_work);

	switch_data->irq = gpio_to_irq(switch_data->gpio);
	if (switch_data->irq < 0) {
		ret = switch_data->irq;
		goto err_detect_irq_num_failed;
	}

	platform_set_drvdata(pdev, switch_data);

	ret = request_irq(switch_data->irq, gpio_irq_handler,
                      IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING, 
                      pdev->name, switch_data);
	if (ret < 0)
		goto err_request_irq;

	/* Reading from nvetc */
	lock_hardware = nvtec_get_lock_key_enabled();
	printk("%s: init lock_hardware is %s\n", __func__, lock_hardware ? "enable" : "disable");

	/* Perform initial detection */
	gpio_switch_work(&switch_data->work);

	return 0;

err_request_irq:
err_detect_irq_num_failed:
err_set_gpio_input:
	gpio_free(switch_data->gpio);
err_request_gpio:
    switch_dev_unregister(&switch_data->sdev);
err_switch_dev_register:
	kfree(switch_data);

	return ret;
}

static int __devexit gpio_switch_remove(struct platform_device *pdev)
{
	struct gpio_switch_data *switch_data = platform_get_drvdata(pdev);

	cancel_work_sync(&switch_data->work);
	gpio_free(switch_data->gpio);
	device_remove_file(switch_data->sdev.dev, &dev_attr_lock_hardware);
    switch_dev_unregister(&switch_data->sdev);
	kfree(switch_data);

	return 0;
}

static int switch_gpio_resume(struct platform_device *pdev) {
	struct gpio_switch_data *switch_data = platform_get_drvdata(pdev);
	schedule_work(&switch_data->work);
	return 0;
}

static struct platform_driver gpio_switch_driver = {
	.probe		= gpio_switch_probe,
	.remove		= __devexit_p(gpio_switch_remove),
	.driver		= {
		.name	= "switch-gpio",
		.owner	= THIS_MODULE,
	},
	.resume 	= switch_gpio_resume,
};

static int __init gpio_switch_init(void)
{
	return platform_driver_register(&gpio_switch_driver);
}

static void __exit gpio_switch_exit(void)
{
	platform_driver_unregister(&gpio_switch_driver);
}

module_init(gpio_switch_init);
module_exit(gpio_switch_exit);

MODULE_AUTHOR("Mike Lockwood <lockwood@android.com>");
MODULE_DESCRIPTION("GPIO Switch driver");
MODULE_LICENSE("GPL");
