#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/debugfs.h>
#include <linux/power_supply.h>
#include <linux/wakelock.h>
#include <linux/i2c.h>
#include <linux/kthread.h>
#include <linux/notifier.h>
#include <linux/slab.h>
#include <asm/uaccess.h>
#include <linux/mfd/nvtec.h>
#include <linux/nvcommon.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/earlysuspend.h>

#include <linux/timer.h>
#include <../gpio-names.h>

#define SBS_MFG_ACCESS                  0x00
#define SBS_REMAIN_CAPACITY_ALARM       0x01
#define SBS_REMNAIN_TIME_ALARM          0x02
#define SBS_BATTERY_MODE                0x03
#define SBS_AT_RATE                     0x04
#define SBS_AT_RATE_TIME_TO_FULL        0x05
#define SBS_AT_RATE_TIME_TO_EMPTY       0x06
#define SBS_AT_RATE_OK                  0x07
#define SBS_TEMP                        0x08
#define SBS_VOLTAGE                     0x09
#define SBS_CURRENT                     0x0A
#define SBS_AVG_CURRENT                 0x0B
#define SBS_MAX_ERROR                   0x0C
#define SBS_REL_STATE_OF_CHARGE         0x0D
#define SBS_ABS_STATE_OF_CHARGE         0x0E
#define SBS_REMAIN_CAPABILITY           0x0F
#define SBS_FULL_CHARGE_CAPACITY        0x10
#define SBS_RUN_TIME_TO_EMPTY           0x11
#define SBS_AVG_TIME_TO_EMPTY           0x12
#define SBS_AVG_TIME_TO_FULL            0x13
#define SBS_CHARGE_CURRENT              0x14
#define SBS_CHARGE_VOLTAGE              0x15
#define SBS_BATTERY_STATUS              0x16
#define SBS_CYCLE_COUNT                 0x17
#define SBS_DESIGN_CAPACITY             0x18
#define SBS_DESIGN_VOLTAGE              0x19
#define SBS_SPEC_INFO                   0x1A
#define SBS_MFG_DATE                    0x1B
#define SBS_SERIAL_NO                   0x1C
#define SBS_MFG_NAME                    0x20
#define SBS_DEV_NAME                    0x21
#define SBS_DEV_CHEMISTRY               0x22
#define SBS_MFG_DATA                    0x23
#define SBS_BATTERY_USAGE               0x30
#define SBS_PERMANENT_FAILURE           0x31
#define SBS_BATTERY_LOG1                0x32
#define SBS_BATTERY_LOG2                0x33
#define SBS_FET_TEMP                    0x3B
#define SBS_OPTION_MFG_FUNC5            0x2F
#define SBS_OPTION_MFG_FUNC4            0x3C
#define SBS_OPTION_MFG_FUNC3            0x3D
#define SBS_OPTION_MFG_FUNC2            0x3E
#define SBS_OPTION_MFG_FUNC1            0x3F

#define SBS_AC_STATUS					0xD4

/* SBS_BATTERY_STATUS Register Bit Mapping */
#define SBS_STATUS_OVER_CHARGE_ALARM    0x8000
#define SBS_STATUS_TERM_CHARGE_ALARM    0x4000
#define SBS_STATUS_OVER_TEMP_ALARM      0x1000
#define SBS_STATUS_TERM_DISCHARGE_ALARM 0x0800
#define SBS_STATUS_REMAIN_CAPACITY_ALARM  0x0200
#define SBS_STATUS_REMAIN_TIME_ALARM    0x0100

#define SBS_STATUS_INITIALIZED          0x0080
#define SBS_STATUS_DISCHARGING          0x0040
#define SBS_STATUS_FULLY_CHARGED        0x0020
#define SBS_STATUS_FULLY_DISCHARGED     0x0010

typedef struct _nvtec_battery_dev {
	struct device *nvtec_dev;

    struct power_supply bat_psy;
    struct power_supply ac_psy;
    struct power_supply usb_psy;

    struct delayed_work work;
	struct notifier_block nb;
	bool battery_is_exist;
	bool ac_is_exist ;

	unsigned int ac_in_pin;
	unsigned int batt_low_pin;

	unsigned int capacity;
	struct mutex lock;

    int ac_in_source;
    int usb_charg_support;
    struct wake_lock wlock;
} nvtec_battery_dev;

static nvtec_battery_dev *g_batt_dev = NULL;
static bool stop_thread = false;

static int inline is_ac_online(void);

static inline struct device *to_nvtec_dev(struct device *dev)
{
    return dev->parent;
}

static char *supply_list[] = {
	"battery",
};

static enum power_supply_property nvtec_battery_properties[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_CAPACITY_LEVEL,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CURRENT_AVG,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_MODEL_NAME,
	POWER_SUPPLY_PROP_MANUFACTURER
};

static enum power_supply_property ac_power_properties[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

static enum power_supply_property usb_power_properties[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

static BLOCKING_NOTIFIER_HEAD(nvtec_ac_notifier);

int nvtec_ac_notifier_register(struct notifier_block *nb)
{
    blocking_notifier_chain_register(&nvtec_ac_notifier, nb);
    blocking_notifier_call_chain(&nvtec_ac_notifier, is_ac_online(), NULL);
    return 0;
}
EXPORT_SYMBOL(nvtec_ac_notifier_register);

int nvtec_ac_notifier_unregister(struct notifier_block *nb)
{
    return blocking_notifier_chain_unregister(&nvtec_ac_notifier, nb);
}
EXPORT_SYMBOL(nvtec_ac_notifier_unregister);

static int nvtec_battery_get_capacity(nvtec_battery_dev *batt_dev, uint16_t *value)
{
    uint16_t cap = 0;
    int ret;
    int retry = 5;

    if (!batt_dev->battery_is_exist) {
        dev_info(batt_dev->nvtec_dev, "battery is not exist\n");
        *value = 0;
        return -EINVAL;
    }

    do {
        ret = nvtec_read_word(batt_dev->nvtec_dev,
                          SBS_REL_STATE_OF_CHARGE,
                          &cap);
        if (ret < 0)
            continue;

        if (cap <= 100)
            break;

        /* restore the previous capacity and retry again */
        cap = batt_dev->capacity;
    } while (retry--);

    /* low battery and no AC plugin, wake lock system */
    if (cap <= 5 && !is_ac_online()) {
        /* check wlock is initialized, flag 0x100: WAKE_LOCK_INITIALIZED */
        if (batt_dev->wlock.flags & 0x100)
            wake_lock(&(batt_dev->wlock));
    }

    *value = cap;

    /* save the current capacity */
    batt_dev->capacity = cap;

    return ret;
}


static int nvtec_battery_get_status(nvtec_battery_dev *batt_dev, uint16_t *value)
{
    return nvtec_read_word(batt_dev->nvtec_dev,
                           SBS_BATTERY_STATUS,
                           value);
}

static int nvtec_battery_get_voltage(nvtec_battery_dev *batt_dev, uint16_t *value)
{
    return nvtec_read_word(batt_dev->nvtec_dev,
                           SBS_VOLTAGE,
                           value);
}

static int nvtec_battery_get_current_now(nvtec_battery_dev *batt_dev, int16_t *value)
{
    return nvtec_read_word(batt_dev->nvtec_dev,
                           SBS_AVG_CURRENT,
                           value);
}

static int nvtec_battery_get_current_avg(nvtec_battery_dev *batt_dev, int16_t *value)
{
    return nvtec_read_word(batt_dev->nvtec_dev,
                           SBS_AVG_CURRENT,
                           value);
}


static int nvtec_battery_get_temperature(nvtec_battery_dev *batt_dev, uint16_t *value)
{
    int ret;
    int retry = 3;
    uint16_t temp;

 retry:
    ret = nvtec_read_word(batt_dev->nvtec_dev,
                          SBS_TEMP,
                          &temp);
    if (ret < 0)
        return ret;

    if (retry != 0 && temp > 3410) {         /* 68 C degree */
        retry --;
        dev_info(batt_dev->nvtec_dev, "retry read temperature:%d=%d\n", retry, temp);
        goto retry;
    }

    *value = temp;

    return ret;
}

static int nvtec_battery_get_technology(nvtec_battery_dev *batt_dev)
{
    int ret;
    char chem[32] = {0};

    ret = nvtec_read_block(batt_dev->nvtec_dev,
                           SBS_DEV_CHEMISTRY,
                           32, chem);

    if (ret < 0)
        return -EINVAL;

    if (chem[0] >= 32)
        return POWER_SUPPLY_TECHNOLOGY_UNKNOWN;

    if (!strcasecmp("NiCd", &chem[1]))
        return POWER_SUPPLY_TECHNOLOGY_NiCd;
    if (!strcasecmp("NiMH", &chem[1]))
        return POWER_SUPPLY_TECHNOLOGY_NiMH;
    if (!strcasecmp("LION", &chem[1]))
        return POWER_SUPPLY_TECHNOLOGY_LION;
    if (!strcasecmp("LI-ION", &chem[1]))
        return POWER_SUPPLY_TECHNOLOGY_LION;
    if (!strcasecmp("LiP", &chem[1]))
        return POWER_SUPPLY_TECHNOLOGY_LIPO;

    return POWER_SUPPLY_TECHNOLOGY_UNKNOWN;
}

static int nvtec_battery_get_model_name(nvtec_battery_dev *batt_dev, char *model_str)
{
    int ret;
    char model[32] = {0};

    ret = nvtec_read_block(batt_dev->nvtec_dev,
                           SBS_DEV_NAME,
                           32, model);

    if (ret < 0)
        return -EINVAL;

    if (model[0] <= 32)
        strncpy(model_str, &model[1], model[0]);
    else
        return -EINVAL;

    return 0;
}

static int nvtec_battery_get_mfg_name(nvtec_battery_dev *batt_dev, char *mfg_str)
{
    int ret;
    char mfg[32] = {0};

    ret = nvtec_read_block(batt_dev->nvtec_dev,
                           SBS_MFG_NAME,
                           32, mfg);

    if (ret < 0)
        return -EINVAL;

    if (mfg[0] <= 32)
        strncpy(mfg_str, &mfg[1], mfg[0]);
    else
        return -EINVAL;

    return 0;
}

static int nvtec_battery_get_level(nvtec_battery_dev *batt_dev)
{
    uint16_t cap;
    uint16_t __cap;
    uint16_t status;
    int ret;

    nvtec_battery_get_status(batt_dev, &status);
    if (status & SBS_STATUS_FULLY_CHARGED)
        return POWER_SUPPLY_CAPACITY_LEVEL_FULL;

    ret = nvtec_battery_get_capacity(batt_dev, &cap);
    if (ret || cap < 0 || cap > 100) {
        return POWER_SUPPLY_CAPACITY_LEVEL_UNKNOWN;
    }

    if (cap < 5)
        __cap = 0;
    else
        __cap = (int) (cap-5) * 100 / 95;

    if ((__cap >= 0) && (__cap <= 5))
        return POWER_SUPPLY_CAPACITY_LEVEL_CRITICAL;
    if ((__cap > 5) && (__cap <=35))
        return POWER_SUPPLY_CAPACITY_LEVEL_LOW;
    if ((__cap > 35) && (__cap <= 70))
        return POWER_SUPPLY_CAPACITY_LEVEL_NORMAL;
    if ((__cap > 70) && (__cap <= 99))
        return POWER_SUPPLY_CAPACITY_LEVEL_HIGH;
    else
        return POWER_SUPPLY_CAPACITY_LEVEL_FULL;
}

static int inline is_ac_online(void)
{
    if (NULL == g_batt_dev)
        return 0;

    if (g_batt_dev->ac_in_pin)
        return gpio_get_value(g_batt_dev->ac_in_pin);
    else
        return 0;
}

static int ac_power_get_property(struct power_supply *psy,
	enum power_supply_property psp, union power_supply_propval *val)
{
    nvtec_battery_dev *batt_dev = container_of(psy, nvtec_battery_dev, ac_psy);

    if (NULL == batt_dev)
        return -ENODEV;

    switch (psp) {
    case POWER_SUPPLY_PROP_ONLINE:
		if(batt_dev->usb_charg_support == 1){
			if(batt_dev->ac_in_source == 0x1)
        		val->intval = 1;
			else
				val->intval = 0;
		} else {
            val->intval = is_ac_online();
		}
        break;
    default:
        return -EINVAL;
    }

    return 0;
}

static int usb_power_get_property(struct power_supply *psy,
	enum power_supply_property psp, union power_supply_propval *val)
{
    nvtec_battery_dev *batt_dev = container_of(psy, nvtec_battery_dev, usb_psy);

    if (NULL == batt_dev)
        return -ENODEV;

    switch (psp) {
    case POWER_SUPPLY_PROP_ONLINE:
		if(batt_dev->usb_charg_support == 1){
			if(batt_dev->ac_in_source == 0x3)
        		val->intval = 1;
			else
				val->intval = 0;
		}else{
			val->intval = 0;
		}

        break;
    default:
        return -EINVAL;
    }

    return 0;
}

static int nvtec_battery_get_property(struct power_supply *psy,
	enum power_supply_property psp, union power_supply_propval *val)
{
    nvtec_battery_dev *batt_dev = container_of(psy, nvtec_battery_dev, bat_psy);
    static char mfg_name[32];
    static char model_name[32];
    int16_t value = 0;
    int ret;

    if (NULL == batt_dev)
        return -ENODEV;

    if (stop_thread)
      return -EBUSY;

    switch (psp) {
    case POWER_SUPPLY_PROP_STATUS:
        ret = nvtec_battery_get_status(batt_dev, &value);
        if (ret != 0) {
            val->intval = POWER_SUPPLY_STATUS_UNKNOWN;
            return 0;
        }
        if (value == 0) {
            val->intval = POWER_SUPPLY_STATUS_UNKNOWN;
            if (batt_dev->battery_is_exist) {
                break;
            } else {
                return 0;
            }
        }

        if (!(value & SBS_STATUS_INITIALIZED)) {
            val->intval = POWER_SUPPLY_STATUS_UNKNOWN;
            break;
        }

        if (value & SBS_STATUS_FULLY_CHARGED) {
            val->intval = POWER_SUPPLY_STATUS_FULL;
            break;
        }

        if (is_ac_online())
            if (value & SBS_STATUS_DISCHARGING)
				val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
			else
                val->intval = POWER_SUPPLY_STATUS_CHARGING;
        else
            val->intval = POWER_SUPPLY_STATUS_DISCHARGING;

        break;
    case POWER_SUPPLY_PROP_PRESENT:
        if (batt_dev->battery_is_exist)
            val->intval = 1;
        else
            val->intval = 0;
        break;
    case POWER_SUPPLY_PROP_TECHNOLOGY:
        ret = nvtec_battery_get_technology(batt_dev);
        if (ret < 0)
            return -EINVAL;
        val->intval = ret;
        break;
    case POWER_SUPPLY_PROP_HEALTH:
        ret = nvtec_battery_get_status(batt_dev, &value);
        if (ret != 0)
            val->intval = POWER_SUPPLY_HEALTH_UNKNOWN;
        else {
            if(value & SBS_STATUS_OVER_TEMP_ALARM)
                val->intval = POWER_SUPPLY_HEALTH_OVERHEAT;
            else
                val->intval = POWER_SUPPLY_HEALTH_GOOD;
        }
        break;
    case POWER_SUPPLY_PROP_VOLTAGE_NOW:
        ret = nvtec_battery_get_voltage(batt_dev, &value);
        if (ret != 0)
            return ret;
        val->intval = value * 1000;
        break;
    case POWER_SUPPLY_PROP_CURRENT_NOW:
        ret = nvtec_battery_get_current_now(batt_dev, &value);
        if (ret != 0)
            return ret;
        val->intval = value * 1000;
        break;
    case POWER_SUPPLY_PROP_CURRENT_AVG:
        ret = nvtec_battery_get_current_avg(batt_dev, &value);
        if (ret != 0)
            return ret;
        val->intval = value * 1000;
        break;
	case POWER_SUPPLY_PROP_CAPACITY:
        ret = nvtec_battery_get_capacity(batt_dev, &value);
        if (ret != 0)
            return ret;

        if (value < 5)
            val->intval = 0;
        else
            val->intval = (int)(value-5)*100/95;
        break;
	/* Add for DOCTOR to detect battery is inserted or not */
	case POWER_SUPPLY_PROP_CAPACITY_LEVEL:
        val->intval = nvtec_battery_get_level(batt_dev);
        break;
	case POWER_SUPPLY_PROP_TEMP:
        ret = nvtec_battery_get_temperature(batt_dev, &value);
        if (ret != 0)
            return ret;
        val->intval = value - 2730;
        break;
    case POWER_SUPPLY_PROP_MODEL_NAME:
        ret = nvtec_battery_get_model_name(batt_dev, model_name);
        if (ret < 0)
            return ret;
        val->strval = model_name;
        break;
    case POWER_SUPPLY_PROP_MANUFACTURER:
        ret = nvtec_battery_get_mfg_name(batt_dev, mfg_name);
        if (ret < 0)
            return ret;
        val->strval = mfg_name;
        break;
    default:
        break;
    }

    return 0;
}

static void nvtec_external_power_changed(struct power_supply *psy)
{
    nvtec_battery_dev *batt_dev = container_of(psy, nvtec_battery_dev, bat_psy);

    cancel_delayed_work_sync(&batt_dev->work);
    if (batt_dev->ac_is_exist)
        /* EC need about 4 sec to check the power source and change to charging state */
        schedule_delayed_work(&batt_dev->work, 4 * HZ);
    else
        schedule_delayed_work(&batt_dev->work, HZ);
}

static int nvtec_battery_event_notifier(struct notifier_block *nb,
                                        unsigned long event,
                                        void *ignored)
{
    nvtec_battery_dev *batt_dev = container_of(nb, nvtec_battery_dev, nb);
    uint8_t event_type;
    uint8_t event_data;

    event_type = (event >> 8) & 0xff;
    event_data = event & 0xff;

    mutex_lock(&batt_dev->lock);
    cancel_delayed_work_sync(&batt_dev->work);

    switch (event_type) {
    case NVTEC_EVENT_SYSTEM:
        batt_dev->usb_charg_support =  event_data >> 4;
        batt_dev->ac_in_source = event_data & 0xF;
        batt_dev->ac_is_exist = (event_data & 0x01) ? true : false;
        break;
    case NVTEC_EVENT_BATTERY:
        batt_dev->battery_is_exist = event_data ? true : false;
        break;
    default:
        break;
    }

    /* EC send event to notify kernel to update status(query EC) */
    schedule_delayed_work(&batt_dev->work, 0);
    mutex_unlock(&batt_dev->lock);

    return NOTIFY_OK;
}

static irqreturn_t ac_in_isr(int irq, void *dev_data)
{
    nvtec_battery_dev *batt_dev = dev_data;

    batt_dev->ac_is_exist = is_ac_online();
	dev_info(batt_dev->nvtec_dev, "AC or USB %s\n", batt_dev->ac_is_exist ? "Insert" : "Remove");
    
    if (batt_dev->ac_is_exist)
        wake_unlock(&(batt_dev->wlock));

    power_supply_changed(&batt_dev->ac_psy);
    power_supply_changed(&batt_dev->usb_psy);

	blocking_notifier_call_chain(&nvtec_ac_notifier, is_ac_online(), NULL);

	return IRQ_HANDLED;
}

static irqreturn_t batt_low_isr(int irq, void *dev_id)
{
    nvtec_battery_dev * batt_dev = dev_id;

	wake_lock(&(batt_dev->wlock));
	dev_info(batt_dev->nvtec_dev, "battery low wake_lock locked\n");

    return IRQ_HANDLED; 
}

static void nvtec_battery_delayed_work(struct work_struct *work)
{
    struct delayed_work *dwork = container_of(work, struct delayed_work, work);
    nvtec_battery_dev *batt_dev = container_of(dwork, nvtec_battery_dev, work);

    power_supply_changed(&batt_dev->bat_psy);

    schedule_delayed_work(&batt_dev->work, 60 * HZ);
}

static int nvtec_power_supply_register(nvtec_battery_dev *batt_dev)
{
    int ret;

    batt_dev->bat_psy.name = "battery";
    batt_dev->bat_psy.type = POWER_SUPPLY_TYPE_BATTERY;
    batt_dev->bat_psy.properties = nvtec_battery_properties;
    batt_dev->bat_psy.num_properties = ARRAY_SIZE(nvtec_battery_properties);
    batt_dev->bat_psy.get_property = nvtec_battery_get_property;
    batt_dev->bat_psy.external_power_changed = nvtec_external_power_changed;
    ret = power_supply_register(batt_dev->nvtec_dev, &batt_dev->bat_psy);
    if (ret < 0)
        return ret;

    batt_dev->ac_psy.name = "ac";
    batt_dev->ac_psy.type = POWER_SUPPLY_TYPE_MAINS;
    batt_dev->ac_psy.supplied_to = supply_list;
    batt_dev->ac_psy.num_supplicants = ARRAY_SIZE(supply_list);
    batt_dev->ac_psy.properties = ac_power_properties;
    batt_dev->ac_psy.num_properties = ARRAY_SIZE(ac_power_properties);
    batt_dev->ac_psy.get_property = ac_power_get_property;
    ret = power_supply_register(batt_dev->nvtec_dev, &batt_dev->ac_psy);
    if (ret < 0) {
        power_supply_unregister(&batt_dev->bat_psy);
        return ret;
    }

    batt_dev->usb_psy.name = "usb";
    batt_dev->usb_psy.type = POWER_SUPPLY_TYPE_USB;
    batt_dev->usb_psy.supplied_to = supply_list;
    batt_dev->usb_psy.num_supplicants = ARRAY_SIZE(supply_list);
    batt_dev->usb_psy.properties = usb_power_properties;
    batt_dev->usb_psy.num_properties = ARRAY_SIZE(usb_power_properties);
    batt_dev->usb_psy.get_property = usb_power_get_property;
    ret = power_supply_register(batt_dev->nvtec_dev, &batt_dev->usb_psy);
    if (ret < 0) {
        power_supply_unregister(&batt_dev->ac_psy);
        power_supply_unregister(&batt_dev->bat_psy);
        return ret;
    }

    return ret;
}

static void nvtec_power_supply_unregister(nvtec_battery_dev *batt_dev)
{
    power_supply_unregister(&batt_dev->bat_psy);
    power_supply_unregister(&batt_dev->ac_psy);
    power_supply_unregister(&batt_dev->usb_psy);
}

static int nvtec_battery_probe(struct platform_device *pdev)
{
    struct nvtec_battery_platform_data *pdata = pdev->dev.platform_data;
    nvtec_battery_dev *batt_dev;
    int ret;
    uint16_t value;

    batt_dev = kzalloc(sizeof(*batt_dev), GFP_KERNEL);
    if (!batt_dev) {
        return -ENOMEM;
    }
    g_batt_dev = batt_dev;
    batt_dev->nvtec_dev = to_nvtec_dev(&pdev->dev);

    batt_dev->ac_in_pin = pdata->ac_in_pin;
    batt_dev->batt_low_pin = pdata->batt_low_pin;
    mutex_init(&batt_dev->lock);

    wake_lock_init(&(batt_dev->wlock), WAKE_LOCK_SUSPEND, "NvBattLowSuspendLock");

    INIT_DELAYED_WORK(&batt_dev->work, nvtec_battery_delayed_work);

    nvtec_battery_get_status(batt_dev, &value);
    batt_dev->battery_is_exist = value ? true : false;
    batt_dev->ac_is_exist = is_ac_online();
    dev_info(batt_dev->nvtec_dev, "Battery %s, AC %s\n",
           batt_dev->battery_is_exist ? "Insert" : "Remove",
           batt_dev->ac_is_exist ? "Insert" : "Remove");

    ret = nvtec_power_supply_register(batt_dev);
    if (ret) {
        goto error_register_power;
    }

    if (batt_dev->ac_in_pin) {
        gpio_set_debounce(batt_dev->ac_in_pin, 10 * 1000);
        ret = request_threaded_irq(gpio_to_irq(batt_dev->ac_in_pin), NULL,
                                   ac_in_isr, IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING | IRQF_SHARED,
                                   "ac_in", batt_dev);
        if (ret < 0)
            goto error_ac_in_irq;
        enable_irq_wake(gpio_to_irq(batt_dev->ac_in_pin));
    }

    if (batt_dev->batt_low_pin) {
        gpio_set_debounce(batt_dev->batt_low_pin, 10 * 1000);
        enable_irq_wake(gpio_to_irq(batt_dev->batt_low_pin));
    }

    batt_dev->nb.notifier_call = &nvtec_battery_event_notifier;
    nvtec_register_event_notifier(batt_dev->nvtec_dev,
                                  NVTEC_EVENT_BATTERY,
                                  &batt_dev->nb);
    nvtec_register_event_notifier(batt_dev->nvtec_dev,
                                   NVTEC_EVENT_SYSTEM,
                                   &batt_dev->nb);

    dev_info(batt_dev->nvtec_dev, "%s: battery driver registered\n", pdev->name);
    platform_set_drvdata(pdev, batt_dev);

    return 0;

 error_ac_in_irq:
    if (batt_dev->ac_in_pin)
        gpio_free(batt_dev->ac_in_pin);

 error_register_power:

    cancel_delayed_work_sync(&batt_dev->work);
    wake_lock_destroy(&(batt_dev->wlock));
    mutex_destroy(&batt_dev->lock);
    kfree(batt_dev);
    g_batt_dev = NULL;

    return -EINVAL;
}

static int nvtec_battery_remove(struct platform_device *pdev)
{
    nvtec_battery_dev *batt_dev = platform_get_drvdata(pdev);

    wake_lock_destroy(&(batt_dev->wlock));
    cancel_delayed_work_sync(&batt_dev->work);

    nvtec_unregister_event_notifier(batt_dev->nvtec_dev,
                                    NVTEC_EVENT_BATTERY,
                                    &batt_dev->nb);
    nvtec_unregister_event_notifier(batt_dev->nvtec_dev,
                                    NVTEC_EVENT_SYSTEM,
                                    &batt_dev->nb);

    nvtec_power_supply_unregister(batt_dev);

    if (batt_dev->batt_low_pin) {
        disable_irq_wake(gpio_to_irq(batt_dev->batt_low_pin));
        free_irq(gpio_to_irq(batt_dev->batt_low_pin), batt_dev);
        gpio_free(batt_dev->batt_low_pin);
    }

    if (batt_dev->ac_in_pin) {
        disable_irq_wake(gpio_to_irq(batt_dev->ac_in_pin));
        free_irq(gpio_to_irq(batt_dev->ac_in_pin), batt_dev);
        gpio_free(batt_dev->ac_in_pin);
    }

    mutex_destroy(&batt_dev->lock);
    kfree(batt_dev);
    g_batt_dev = NULL;

    return 0;
}

static int nvtec_battery_suspend(struct platform_device *pdev, pm_message_t state)
{
    nvtec_battery_dev *batt_dev = platform_get_drvdata(pdev);

    cancel_delayed_work_sync(&batt_dev->work);
    return 0;
}

static int nvtec_battery_resume(struct platform_device *pdev)
{
    nvtec_battery_dev *batt_dev = platform_get_drvdata(pdev);

    /* to make sure query EC will not fail when system resume */
    cancel_delayed_work_sync(&batt_dev->work);
    schedule_delayed_work(&batt_dev->work, HZ);
    return 0;
}

void suspend_battery_thread(void)
{
    if (NULL == g_batt_dev)
        return;

    stop_thread = true;
    cancel_delayed_work_sync(&g_batt_dev->work);
}

void resume_battery_thread(void)
{
    if (NULL == g_batt_dev)
        return;

    stop_thread = false;
    cancel_delayed_work_sync(&g_batt_dev->work);
    schedule_delayed_work(&g_batt_dev->work, HZ);
}

static void nvtec_battery_shutdown(struct platform_device *pdev)
{
    nvtec_battery_dev *batt_dev = platform_get_drvdata(pdev);

    stop_thread = true;
    cancel_delayed_work_sync(&batt_dev->work);
}

static struct platform_driver nvtec_battery_driver =
{
    .probe = nvtec_battery_probe,
    .remove = __devexit_p(nvtec_battery_remove),
    .suspend = nvtec_battery_suspend,
    .resume = nvtec_battery_resume,
    .shutdown = nvtec_battery_shutdown,
    .driver = {
        .name = "nvtec-battery",
        .owner = THIS_MODULE,
    },
};

static int __init nvtec_battery_init(void)
{
    platform_driver_register(&nvtec_battery_driver);
    return 0;
}

static void __exit nvtec_battery_exit(void)
{
    platform_driver_unregister(&nvtec_battery_driver);
}

module_init(nvtec_battery_init);
module_exit(nvtec_battery_exit);

MODULE_DESCRIPTION("Nuvoton EC Battery Driver");
MODULE_AUTHOR("Ron Lee <ron1_lee@pegatroncorp.com>");
MODULE_LICENSE("GPL");
