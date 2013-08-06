#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/switch.h>
#include <linux/workqueue.h>
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/iqs128_cap_sensor.h>
#include <linux/err.h>

struct tegra_cap_dev {
	struct switch_dev sdev;
	unsigned gpio;
	int irq;
	int value;
	struct work_struct work;
	struct regulator *cap_3v3_reg;
};

static void tegra_cap_work(struct work_struct *work)
{
	int state;
	struct tegra_cap_dev *dev =
		container_of(work, struct tegra_cap_dev, work);

	state = gpio_get_value(dev->gpio);
	if (dev->value != state) {
		//printk("%s: cap sensor value changed from %d to %d\n", __func__, dev->value, state);
		dev->value = state;
		switch_set_state(&dev->sdev, state);
	}
}

static irqreturn_t tegra_cap_irq_handler(int irq, void *dev_id)
{
	struct tegra_cap_dev *dev =
	    (struct tegra_cap_dev *)dev_id;

	schedule_work(&dev->work);
	return IRQ_HANDLED;
}

static ssize_t tegra_cap_print_state(struct switch_dev *sdev, char *buf)
{
	struct tegra_cap_dev *dev =
		container_of(sdev, struct tegra_cap_dev, sdev);
	return sprintf(buf, "%d", dev->value);
}

static int tegra_cap_regulator_enabled(int enable, struct tegra_cap_dev *dev) 
{
	if (!dev->cap_3v3_reg && enable) {
		dev->cap_3v3_reg = regulator_get(NULL, "vdd_3v3_cap");
		if (IS_ERR_OR_NULL(dev->cap_3v3_reg)) {
			printk("%s: get regulator failed\n", __func__);
			dev->cap_3v3_reg = NULL;
			return -1;
		}
		regulator_enable(dev->cap_3v3_reg);
		printk("%s: setting enabled\n", __func__);
	} else if (dev->cap_3v3_reg && !enable) {
		regulator_disable(dev->cap_3v3_reg);
		regulator_put(dev->cap_3v3_reg);
		dev->cap_3v3_reg = NULL;
		printk("%s: setting disabled\n", __func__);
	} else
		return -1;

	return 0;
}

static int tegra_cap_probe(struct platform_device *pdev)
{
	struct cap_sensor_platform_data *pdata = pdev->dev.platform_data;
	struct tegra_cap_dev *dev = NULL;
	int ret = 0;

	dev = kzalloc(sizeof(struct tegra_cap_dev), GFP_KERNEL);
	if (!dev)
		return -ENOMEM;
	
	/* Turning on device power */
	tegra_cap_regulator_enabled(1, dev);

	/* Register switch device */
	dev->sdev.name = "iqs128_cap";
	dev->sdev.print_state = tegra_cap_print_state;

    	ret = switch_dev_register(&dev->sdev);
	if (ret < 0)
		goto err_switch_dev_register;

	/* Setting out gpio pin status */
	dev->gpio = pdata->gpio_num;

	ret = gpio_request(dev->gpio, pdev->name);
	if (ret < 0)
		goto err_request_gpio;

	ret = gpio_direction_input(dev->gpio);
	if (ret < 0)
		goto err_set_gpio_input;
	
	gpio_export(dev->gpio, 0);

	dev->irq = pdata->irq;
	if (dev->irq < 0) {
		ret = dev->irq;
		goto err_detect_irq_num_failed;
	}

	tegra_gpio_enable(dev->gpio);

	/* Init work queue and irq handler */
	INIT_WORK(&dev->work, tegra_cap_work);

	ret = request_irq(dev->irq, tegra_cap_irq_handler,
			  IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING, pdev->name, dev);
	if (ret < 0)
		goto err_request_irq;

	/* Reading current value */
	tegra_cap_work(&dev->work);

	return 0;

err_request_irq:
err_detect_irq_num_failed:
err_gpio_set_active_low:
err_gpio_enable:
err_set_gpio_input:
	gpio_free(dev->gpio);
err_request_gpio:
    	switch_dev_unregister(&dev->sdev);
err_switch_dev_register:
	kfree(dev);

	return ret;
}

static int __devexit tegra_cap_remove(struct platform_device *pdev) 
{
	struct tegra_cap_dev *dev = platform_get_drvdata(pdev);

	cancel_work_sync(&dev->work);
	tegra_cap_regulator_enabled(0, dev);
	gpio_free(dev->gpio);
    	switch_dev_unregister(&dev->sdev);
	kfree(dev);

	return 0;
}

static struct platform_driver tegra_cap_driver = {
	.probe		= tegra_cap_probe,
	.remove		= __devexit_p(tegra_cap_remove),
	.driver		= {
		.name	= "tegra_cap",
		.owner	= THIS_MODULE,
	},
};

static int __init tegra_cap_init(void)
{
	return platform_driver_register(&tegra_cap_driver);
}

static void __exit tegra_cap_exit(void)
{
	platform_driver_unregister(&tegra_cap_driver);
}

module_init(tegra_cap_init);
module_exit(tegra_cap_exit);

MODULE_AUTHOR("Jeff Yu <jeff_yu@pegatroncorp.com>");
MODULE_DESCRIPTION("IQS128 cap Sensor driver");
MODULE_LICENSE("GPL");
