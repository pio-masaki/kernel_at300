#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/isdbt_power.h>
#include <linux/delay.h>

static struct class *isdbt_class;

struct isdbt_power_data {
	struct device *dev;
    int isdbt_reset;
	struct isdbt_platform_data *pdata;
	struct mutex isdbt_power_mutex;
};

static ssize_t isdbt_power_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{

	int gpio_pin = 0;
	int reset_pin_value, regulator_status = 0;
	struct isdbt_power_data *data= dev_get_drvdata(dev);
	gpio_pin = data->isdbt_reset;
	reset_pin_value = gpio_get_value(gpio_pin);
	regulator_status = data->pdata->isdbt_regulator_status();

	return sprintf(buf, "%d, %d\n", regulator_status, reset_pin_value);

}

static ssize_t isdbt_power_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct isdbt_power_data *data= dev_get_drvdata(dev);
	int value = 0;

	sscanf(buf, "%d", &value);

	if(value == 1) {
		mutex_lock(&data->isdbt_power_mutex);
		data->pdata->enable_isdbt_regulator_control(value);
		mutex_unlock(&data->isdbt_power_mutex);
		dev_info(dev, "ISDBT Power ON\n");

	} else {
		mutex_lock(&data->isdbt_power_mutex);
		data->pdata->enable_isdbt_regulator_control(0);
		mutex_unlock(&data->isdbt_power_mutex);
		dev_info(dev, "ISDBT Power OFF\n");
	}
	return size;
}

static DEVICE_ATTR(isdbt_power, S_IRUGO | S_IWUSR, isdbt_power_show, isdbt_power_store);


static int create_isdbt_class(void)
{
	if (!isdbt_class) {
		isdbt_class = class_create(THIS_MODULE, "isdbt");
		if (IS_ERR(isdbt_class))
			return PTR_ERR(isdbt_class);
	}
	return 0;
}

static int __devinit tegra_isdbt_power_probe(struct platform_device *pdev)
{

    struct isdbt_platform_data *pdata = pdev->dev.platform_data;
    struct isdbt_power_data *isdbt = NULL;
    int ret = -ENODEV;

	isdbt = kzalloc(sizeof(struct isdbt_power_data), GFP_KERNEL);
	if (!isdbt) {
		pr_err("Fails to alloc isdbt power device\n");
		return -ENOMEM;
	}

	isdbt->pdata = pdata;
	isdbt->isdbt_reset = pdata->isdbt_reset_pin;

	mutex_init(&isdbt->isdbt_power_mutex);

    if (!isdbt_class) {
	    ret = create_isdbt_class();
	    if (ret < 0)
		    goto err_create_class;
    }

    isdbt->dev = device_create(isdbt_class, NULL,
		MKDEV(0, 0), NULL, "power_control");
    if (IS_ERR(isdbt->dev))
		goto err_device_create;

    ret = device_create_file(isdbt->dev, &dev_attr_isdbt_power);
	if (ret < 0)
		goto err_createfile;

	dev_set_drvdata(isdbt->dev, isdbt);
	platform_set_drvdata(pdev, isdbt);

	return 0;


err_createfile:
    device_remove_file(isdbt->dev, &dev_attr_isdbt_power);
err_device_create:
    device_destroy(isdbt_class, MKDEV(0, 0));
err_create_class:
    class_destroy(isdbt_class);
	kfree(isdbt);
	return ret;

}

static int __devexit tegra_isdbt_power_remove(struct platform_device *pdev)
{

	struct isdbt_power_data *isdbt = platform_get_drvdata(pdev);

	device_remove_file(isdbt->dev, &dev_attr_isdbt_power);
	device_destroy(isdbt_class, MKDEV(0, 0));
	class_destroy(isdbt_class);
	kfree(isdbt);


	return 0;
}

#ifdef CONFIG_PM
static int tegra_isdbt_power_suspend(struct platform_device *pdev, pm_message_t state)
{

	int res = -EPERM;
	struct isdbt_power_data *isdbt= platform_get_drvdata(pdev);

	mutex_lock(&isdbt->isdbt_power_mutex);
	res = isdbt->pdata->enable_isdbt_regulator_control(0);
	mutex_unlock(&isdbt->isdbt_power_mutex);

    dev_info(&pdev->dev, "ISDBT Power Suspend OFF\n");

	return res;

}

static int tegra_isdbt_power_resume(struct platform_device *pdev)
{
	return 0;
}

#endif

static struct platform_driver isdbt_power_driver = {
	.probe	= tegra_isdbt_power_probe,
	.remove	= tegra_isdbt_power_remove,
	.driver	= {
		.name = "isdbt_power",
	},
#ifdef CONFIG_PM
	.suspend	= tegra_isdbt_power_suspend,
	.resume		= tegra_isdbt_power_resume,
#endif
};

static int __init isdbt_power_init(void)
{
	return platform_driver_register(&isdbt_power_driver);
}

static void __exit isdbt_power_exit(void)
{
	platform_driver_unregister(&isdbt_power_driver);
}

module_init(isdbt_power_init);
module_exit(isdbt_power_exit);

MODULE_AUTHOR("Albert Hsu <Albert_hsu@pegatroncorp.com>");
MODULE_DESCRIPTION("isdbt power driver for Titan");
MODULE_LICENSE("GPL");

