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
/* Daniel Wang */
#include <linux/timer.h>
#include <../gpio-names.h>

/* #include <linux/i2c/atmel_maxtouch.h> */

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
	char mfg_name[32];
	char model_name[32];
	char chemistry[32];
	struct task_struct *task;
	struct notifier_block nb;
	int battery_is_exist;
	int ac_is_exist ;
	unsigned int ac_in_pin;
	unsigned int batt_low_pin;

	unsigned int batt_capacity ;
	unsigned int batt_temperature ;
	wait_queue_head_t wait_q;
	struct task_struct *bat_task;
	int stop_thread;
	int suspend_thread;
	struct mutex    lock;
} nvtec_battery_dev;

static nvtec_battery_dev *batt_dev;
static int is_psy_changed = 0;
#ifdef CONFIG_HAS_EARLYSUSPEND
static struct input_dev *ac_input;
struct early_suspend ac_early_suspender;
static int is_ac_early_suspend;
#endif

/* Daniel Wang */
static int ac_in_source = 0;
static int usb_charg_support = 0;

static struct timer_list mcu_sys_event_delay_timer;
static struct wake_lock nvtecb_lock;
static NvBool nvtecb_lock_flag = NV_FALSE;
static int inline is_ac_online(void);


#undef DISCHARGE_SIMULATION
#ifdef DISCHARGE_SIMULATION
static unsigned int capacity = 20;
#endif

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

/* Daniel Wang */
static enum power_supply_property usb_power_properties[] = {
	POWER_SUPPLY_PROP_ONLINE,
};


static int ac_power_get_property(struct power_supply *psy,
	enum power_supply_property psp, union power_supply_propval *val);

/* Daniel Wang */
static int usb_power_get_property(struct power_supply *psy,
	enum power_supply_property psp, union power_supply_propval *val);


static int nvtec_battery_get_property(struct power_supply *psy,
	enum power_supply_property psp, union power_supply_propval *val);


static struct power_supply nvtec_supplies[] = {
    {
        .name = "battery",
        .type = POWER_SUPPLY_TYPE_BATTERY,
        .properties = nvtec_battery_properties,
        .num_properties = ARRAY_SIZE(nvtec_battery_properties),
        .get_property = nvtec_battery_get_property,
    },
    {
        .name = "ac",
        .type = POWER_SUPPLY_TYPE_MAINS,
        .supplied_to = supply_list,
        .num_supplicants = ARRAY_SIZE(supply_list),
        .properties = ac_power_properties,
        .num_properties = ARRAY_SIZE(ac_power_properties),
        .get_property = ac_power_get_property,
    },
    {
	/* Daniel Wang */
        .name = "usb",
        .type = POWER_SUPPLY_TYPE_USB,
        .properties = usb_power_properties,
        .num_properties = ARRAY_SIZE(usb_power_properties),
        .get_property = usb_power_get_property,
    }
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

static int nvtec_battery_get_capacity(uint16_t *value)
{
#ifdef DISCHARGE_SIMULATION
    *value = (uint16_t) capacity;
    return 0;
#endif
    uint16_t cap = 0;
    int ret;
    int retry = 5;
    unsigned long ac_online;

 retry:
    ret = nvtec_read_word(batt_dev->nvtec_dev,
                          SBS_REL_STATE_OF_CHARGE,
                          &cap);
    if (ret < 0)
    return ret;

    ac_online = is_ac_online();

    mutex_lock(&batt_dev->lock);

    if (batt_dev->battery_is_exist){
        if (cap > 100){
		if (retry != 0 )
		{
			if (batt_dev->batt_capacity == 0){
			        retry --;
			        printk("nvtec: retry read capacity:%d=%d\n", retry, cap);
				mutex_unlock(&batt_dev->lock);
			        goto retry;
			}else{
				cap = batt_dev->batt_capacity ;
			}
		}else{
			printk("nvtec: battery capacity error (>100) over 5 times \n") ;
			cap = 0 ;
		}
	}else if (cap <= 5){
		if (ac_online){
			printk("nvtec: battery capacity low then 5 percent and ac plugin \n") ;
			if (nvtecb_lock_flag){
				wake_unlock(&nvtecb_lock);
				nvtecb_lock_flag = NV_FALSE;
				printk("nvtec: battery low wake_lock unlocked\n");
			}
		}else{
			printk("nvtec: battery capacity low then 5 percent \n") ;
			if (!nvtecb_lock_flag){
			    wake_lock(&nvtecb_lock);
			    nvtecb_lock_flag = NV_TRUE;
			    printk("nvtec: battery low wake_lock locked\n");
			}
		}
		batt_dev->batt_capacity = cap ;
        }else{
		batt_dev->batt_capacity = cap ;
	}
    }else{
	printk("nvtec: battery Eject \n") ;
	cap = 0 ;
    }

    mutex_unlock(&batt_dev->lock);

    /*printk("nvtec: read capacity: %d\n", cap);*/
    *value = cap;

    return ret;
}

/*static int nvtec_ac_get_status(uint16_t *value)*/
/*{*/
/*    return nvtec_read_word(batt_dev->nvtec_dev,*/
/*                           SBS_AC_STATUS,*/
/*                           value);*/

/*}*/

static int nvtec_battery_get_status(uint16_t *value)
{
    return nvtec_read_word(batt_dev->nvtec_dev,
                           SBS_BATTERY_STATUS,
                           value);
}

static int nvtec_battery_get_voltage(uint16_t *value)
{
    return nvtec_read_word(batt_dev->nvtec_dev,
                           SBS_VOLTAGE,
                           value);
}

static int nvtec_battery_get_current_now(int16_t *value)
{
    return nvtec_read_word(batt_dev->nvtec_dev,
                           SBS_AVG_CURRENT,	/* Daniel Wang */
                           value);
}

static int nvtec_battery_get_current_avg(int16_t *value)
{
    return nvtec_read_word(batt_dev->nvtec_dev,
                           SBS_AVG_CURRENT,
                           value);
}


static int nvtec_battery_get_temperature(uint16_t *value)
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
        printk("nvtec: retry read temperature:%d=%d\n", retry, temp);
        goto retry;
    }

    *value = temp;

    return ret;
}

static int nvtec_battery_get_technology(void)
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

static int nvtec_battery_get_model_name(char *model_str)
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

static int nvtec_battery_get_mfg_name(char *mfg_str)
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

static int inline is_ac_online(void)
{
	if (!batt_dev)
		return gpio_get_value(TEGRA_GPIO_PV1);
	else
		return gpio_get_value(batt_dev->ac_in_pin);
}

static int ac_power_get_property(struct power_supply *psy,
	enum power_supply_property psp, union power_supply_propval *val)
{
    switch (psp) {
    case POWER_SUPPLY_PROP_ONLINE:
		if(usb_charg_support == 1){
			if(ac_in_source == 0x1)
        		val->intval = 1;
			else
				val->intval = 0;
		}else{
        val->intval = is_ac_online();
		}
        break;
    default:
        return -EINVAL;
    }

    return 0;
}

/* Daniel Wang */
static int usb_power_get_property(struct power_supply *psy,
	enum power_supply_property psp, union power_supply_propval *val)
{
    switch (psp) {
    case POWER_SUPPLY_PROP_ONLINE:
		if(usb_charg_support == 1){
			if(ac_in_source == 0x3)
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
    int16_t value = 0;
    int ret;

    if (batt_dev->stop_thread || batt_dev->suspend_thread)
      return -ENODEV;

    switch (psp) {
    case POWER_SUPPLY_PROP_STATUS:
        ret = nvtec_battery_get_status(&value);
        if (ret != 0) {
            val->intval = POWER_SUPPLY_STATUS_UNKNOWN;
            return 0;
        }
        if (value == 0) {
            val->intval = POWER_SUPPLY_STATUS_UNKNOWN;
            if (batt_dev->battery_is_exist) {
		  /*is_psy_changed = 1;*/
		  /*wake_up(&batt_dev->wait_q);*/
                break;
            } else {
                return 0;
            }
        }

        if (!(value & SBS_STATUS_INITIALIZED)) {
            val->intval = POWER_SUPPLY_STATUS_UNKNOWN;
            if (batt_dev->battery_is_exist) {
		  /*is_psy_changed = 1;*/
		  /*wake_up(&batt_dev->wait_q);*/
            }
            break;
        }

        if (value & SBS_STATUS_FULLY_CHARGED) {
            val->intval = POWER_SUPPLY_STATUS_FULL;
            break;
        }

	/* Daniel Wang */
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
        ret = nvtec_battery_get_technology();
        if (ret < 0)
            return -EINVAL;
        val->intval = ret;
        break;
    case POWER_SUPPLY_PROP_HEALTH:
        ret = nvtec_battery_get_status(&value);
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
        ret = nvtec_battery_get_voltage(&value);
        if (ret != 0)
            return ret;
        val->intval = value * 1000;
        break;
    case POWER_SUPPLY_PROP_CURRENT_NOW:
        ret = nvtec_battery_get_current_now(&value);
        if (ret != 0)
            return ret;
        val->intval = value * 1000;
        break;
    case POWER_SUPPLY_PROP_CURRENT_AVG:
        ret = nvtec_battery_get_current_avg(&value);
        if (ret != 0)
            return ret;
        val->intval = value * 1000;
        break;
	case POWER_SUPPLY_PROP_CAPACITY:
        ret = nvtec_battery_get_capacity(&value);
        if (ret != 0)
            return ret;

        if (value < 5)
            val->intval = 0;
        else
            val->intval = (int)(value-5)*100/95;
        break;
	/* Add for DOCTOR to detect battery is inserted or not */
	case POWER_SUPPLY_PROP_CAPACITY_LEVEL:
        ret = nvtec_battery_get_capacity(&value);
        if (ret != 0)
            return ret;
        if (value == 0)
            val->intval = POWER_SUPPLY_CAPACITY_LEVEL_UNKNOWN;
        else
            val->intval = POWER_SUPPLY_CAPACITY_LEVEL_NORMAL;
        break;
	case POWER_SUPPLY_PROP_TEMP:
        ret = nvtec_battery_get_temperature(&value);
        if (ret != 0)
            return ret;
        val->intval = value - 2730;
        break;
    case POWER_SUPPLY_PROP_MODEL_NAME:
        ret = nvtec_battery_get_model_name(batt_dev->model_name);
        if (ret < 0)
            return ret;
        val->strval = batt_dev->model_name;
        break;
    case POWER_SUPPLY_PROP_MANUFACTURER:
        ret = nvtec_battery_get_mfg_name(batt_dev->mfg_name);
        if (ret < 0)
            return ret;
        val->strval = batt_dev->mfg_name;
    default:
        break;
    }

    return 0;
}

static int nvtec_battery_event_notifier(struct notifier_block *nb,
                                        unsigned long event,
                                        void *ignored)
{
    uint8_t event_type;
    uint8_t event_data;
    int ac_is_exist = 0;

    event_type = (event >> 8) & 0xff;
    event_data = event & 0xff;

    mutex_lock(&batt_dev->lock);
    switch (event_type) {
    case NVTEC_EVENT_SYSTEM:
    	 /* Daniel Wang */
	 usb_charg_support =  event_data >> 4;
	 ac_in_source = event_data &0xF;
         batt_dev->ac_is_exist = ac_is_exist = (event_data & 0x01) ? 1 : 0;
         is_psy_changed = 1;
         wake_up(&batt_dev->wait_q);
/*         printk("nvtec: ACflag %s\n", ac_is_exist ? "Insert" : "Remove");*/
/*         printk("nvtec: MCU PWR Source Event: %s\n",(ac_in_source!=0) ? ((ac_in_source==3) ? "USB" : "Adaptor") : "Batt ONLY");*/
	 printk("nvtec: System Event with data [0x%X] \n", event_data);
         break;
    case NVTEC_EVENT_BATTERY:
        batt_dev->battery_is_exist = event_data ? 1 : 0;
        is_psy_changed = 1;
        wake_up(&batt_dev->wait_q);
/*        printk("nvtec: Battery %s\n", batt_dev->battery_is_exist ? "Insert" : "Remove");*/
	printk("nvtec: Battery Event with data [0x%X] \n", event_data);
        break;
    default:
        break;
    }
    mutex_unlock(&batt_dev->lock);

    return NOTIFY_OK;
}

static inline void sleep(unsigned jiffies)
{
	current->state = TASK_INTERRUPTIBLE;
	schedule_timeout(jiffies);
	return ;
}

static int nvtec_battery_thread(void *args)
{
    do {
#ifdef DISCHARGE_SIMULATION
        wait_event_interruptible_timeout(batt_dev->wait_q, is_psy_changed, 10 * HZ);
#else
        wait_event_interruptible_timeout(batt_dev->wait_q, is_psy_changed, 60 * HZ);
#endif

        /* wait 0.5 seconds until status stable */
        sleep(50);

#ifdef DISCHARGE_SIMULATION
        capacity --;
#endif

        if (kthread_should_stop() || batt_dev->stop_thread)
            break;

	if (batt_dev->suspend_thread)
	    continue;

        power_supply_changed(&nvtec_supplies[0]);
        power_supply_changed(&nvtec_supplies[1]);
	power_supply_changed(&nvtec_supplies[2]);	/* Daniel Wang */

        is_psy_changed = 0;
    } while (!kthread_should_stop());

    return 0;
}

/* Daniel Wang */
static void ac_in_isr_delay_function(unsigned long val){ /*In order to avoid battery 60s polling*/
	is_psy_changed = 1;
        /* printk("nvtec: Polling Battery after 4sec !\n");*/
	wake_up(&batt_dev->wait_q);
	return ;
}

/* Daniel Wang */
static irqreturn_t ac_in_isr(int irq, void *dev_id)
{
	//int value;

	printk("nvtec: AC or USB %s\n", is_ac_online() ? "Insert" : "Remove");
	mod_timer(&mcu_sys_event_delay_timer,jiffies + (4*HZ));

	blocking_notifier_call_chain(&nvtec_ac_notifier, is_ac_online(), NULL);

	#ifdef CONFIG_HAS_EARLYSUSPEND
	if(is_ac_early_suspend) {
		input_event(ac_input, EV_KEY, KEY_POWER, 1);
		input_event(ac_input, EV_KEY, KEY_POWER, 0);
		input_sync(ac_input);
	}
	#endif

	return IRQ_HANDLED;
}

static irqreturn_t batt_low_isr(int irq, void *dev_id) 
{ 
/*    printk("batt_low_isr\n"); */
    if (!nvtecb_lock_flag){
	wake_lock(&nvtecb_lock);
	nvtecb_lock_flag = NV_TRUE;
	printk("batt_low_isr: battery low wake_lock locked\n");
    }
    return IRQ_HANDLED; 
} 

#ifdef CONFIG_HAS_EARLYSUSPEND
static void ac_early_suspend(struct early_suspend *h)
{
	is_ac_early_suspend = 1;
}

static void ac_late_resume(struct early_suspend *h)
{
	is_ac_early_suspend = 0;
}
#endif

static int nvtec_battery_probe(struct platform_device *pdev)
{
    struct nvtec_battery_platform_data *pdata = pdev->dev.platform_data;
    int i, ret;
    uint16_t value;

    batt_dev = kzalloc(sizeof(*batt_dev), GFP_KERNEL);
    if (!batt_dev) {
        return -ENOMEM;
    }
    memset(batt_dev, 0, sizeof(*batt_dev));

    batt_dev->nvtec_dev = to_nvtec_dev(&pdev->dev);

    batt_dev->ac_in_pin = pdata->ac_in_pin;
    batt_dev->batt_low_pin = pdata->batt_low_pin;
    mutex_init(&batt_dev->lock);

    gpio_set_debounce(batt_dev->ac_in_pin, 10 * 1000);
    gpio_set_debounce(batt_dev->batt_low_pin, 10 * 1000);

    enable_irq_wake(gpio_to_irq(batt_dev->ac_in_pin));
    enable_irq_wake(gpio_to_irq(batt_dev->batt_low_pin)); 

    for (i=0; i < ARRAY_SIZE(nvtec_supplies); i++) {
        ret = power_supply_register(&pdev->dev, &nvtec_supplies[i]);
        if (ret) {
            printk(KERN_ERR "%s: failed to register power supply\n", pdev->name);
            while (i--)
                power_supply_unregister(&nvtec_supplies[i]);
            kfree(batt_dev);
            return ret;
        }
    }

    init_waitqueue_head(&batt_dev->wait_q);

    batt_dev->nb.notifier_call = &nvtec_battery_event_notifier;
    nvtec_register_event_notifier(batt_dev->nvtec_dev,
                                  NVTEC_EVENT_BATTERY,
                                  &batt_dev->nb);
    nvtec_register_event_notifier(batt_dev->nvtec_dev,
                                   NVTEC_EVENT_SYSTEM,
                                   &batt_dev->nb);

    nvtec_battery_get_status(&value);
    if (value == 0){
	printk("nvtec: Battery Eject \n");
	batt_dev->battery_is_exist = 0;
    }else{
	printk("nvtec: Battery Inject \n");
	batt_dev->battery_is_exist = 1;
    }

    /* Daniel Wang */
    init_timer(&mcu_sys_event_delay_timer);
    mcu_sys_event_delay_timer.function = ac_in_isr_delay_function;
    mcu_sys_event_delay_timer.expires = jiffies;
    mcu_sys_event_delay_timer.data = 0;

    request_irq(gpio_to_irq(batt_dev->batt_low_pin), 
		 batt_low_isr, 
		 IRQF_TRIGGER_LOW | IRQF_ONESHOT, 
		 "batt_low", batt_dev); 

    batt_dev->stop_thread = 0;
    batt_dev->suspend_thread = 0;
    batt_dev->bat_task = kthread_run(nvtec_battery_thread, NULL, "nvtec_bat");
    if (IS_ERR(batt_dev->bat_task))
		goto error;

    ret = request_threaded_irq(gpio_to_irq(batt_dev->ac_in_pin), NULL,
                               ac_in_isr, IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING | IRQF_SHARED,
                               "ac_in", batt_dev);
    if (ret < 0)
        goto error;

    wake_lock_init(&nvtecb_lock, WAKE_LOCK_SUSPEND, "NvBattLowSuspendLock");	/* Daniel Wang */

	#ifdef CONFIG_HAS_EARLYSUSPEND
	ac_input = input_allocate_device();
	if (!ac_input) {
		goto error;
	}

	ac_input->name = pdev->name;
	ac_input->id.bustype = BUS_HOST;
	ac_input->dev.parent = &pdev->dev;

	input_set_capability(ac_input, EV_KEY, KEY_POWER);
	ret = input_register_device(ac_input);
	if(ret) {
		input_free_device(ac_input);
		goto error;
	}

	is_ac_early_suspend = 0;
	ac_early_suspender.suspend = ac_early_suspend;
	ac_early_suspender.resume = ac_late_resume;
	register_early_suspend(&ac_early_suspender);
	#endif

    printk(KERN_INFO "%s: battery driver registered\n", pdev->name);

    return 0;
 error:
    for (i=0; i < ARRAY_SIZE(nvtec_supplies); i++) {
        power_supply_unregister(&nvtec_supplies[i]);
    }

    kfree(batt_dev);

    return -EINVAL;


}

static int nvtec_battery_remove(struct platform_device *pdev)
{
    int i;

	#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&ac_early_suspender);
	input_unregister_device(ac_input);
	#endif
    wake_lock_destroy(&nvtecb_lock);	/* Daniel Wang */
    for (i=0; i < ARRAY_SIZE(nvtec_supplies); i++) {
        power_supply_unregister(&nvtec_supplies[i]);
    }

    kthread_stop(batt_dev->bat_task);
    nvtec_unregister_event_notifier(batt_dev->nvtec_dev,
                                    NVTEC_EVENT_BATTERY,
                                    &batt_dev->nb);
    nvtec_unregister_event_notifier(batt_dev->nvtec_dev,
                                    NVTEC_EVENT_SYSTEM,
                                    &batt_dev->nb);

    gpio_free(batt_dev->ac_in_pin);
    gpio_free(batt_dev->batt_low_pin);
    mutex_destroy(&batt_dev->lock) ;

    kfree(batt_dev);
    del_timer(&mcu_sys_event_delay_timer);	/* Daniel Wang */
    return 0;
}

static int nvtec_battery_suspend(struct platform_device *pdev, pm_message_t state)
{
    return 0;
}

static int nvtec_battery_resume(struct platform_device *pdev)
{
    is_psy_changed = 1;
    wake_up(&batt_dev->wait_q);
    return 0;
}

void suspend_battery_thread(void)
{
	batt_dev->suspend_thread = 1;
	return ;
}

void resume_battery_thread(void)
{
	batt_dev->suspend_thread = 0;
	return ;
}

static void nvtec_battery_shutdown(struct platform_device *pdev)
{
    batt_dev->stop_thread = 1;
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
