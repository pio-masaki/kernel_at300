#include <linux/fb.h>
#include <linux/input.h>
#include <linux/kthread.h>
#include <linux/delay.h>
#include <linux/suspend.h>
#include <linux/mutex.h>
#include <linux/gpio.h>
#include <linux/charging-img.h>
#include <linux/mfd/nvtec.h>
#include <linux/switch_gpio.h>
#include <linux/notifier.h>
#include <linux/power_supply.h>
#include <asm/mach-types.h>

#include "image/battery.h"

#define fb_width(fb)	((fb)->var.xres)
#define fb_height(fb)	((fb)->var.yres)

struct charging_img {
    struct backlight_device *bl_dev;
    struct power_supply *psy;
    struct task_struct *task;
    struct notifier_block switch_gpio_notifier;
    struct workqueue_struct *suspend_workq;
    struct work_struct suspend_work;
    struct mutex drawing_lock;
    wait_queue_head_t wait_show_img;
    int backlight_status;
    unsigned long light_time;
    bool show_img;
    bool system_resume;
    int bat_level;
    bool lock_fn_enabled;
    bool power_key_locked;
    bool power_key_ignored;
};

typedef struct ScreenRec
{
    unsigned short width;
    unsigned short height;
    int stride;
    unsigned char bpp;
    int byte[MAX_BAT_LEVEL][MAX_LOGO_NUM];
    unsigned short *data[MAX_BAT_LEVEL][MAX_LOGO_NUM];
} Screen;

static struct input_handler charging_img_handler;
static DECLARE_WAIT_QUEUE_HEAD(wait_on_charging);

int is_boot_on_charging = 0;
EXPORT_SYMBOL(is_boot_on_charging);

enum{
	BACKLIGHT_LIGHT,
	BACKLIGHT_DIM,
	BACKLIGHT_OFF,
};

static void battery_level_update(struct charging_img *chargimg)
{
    struct power_supply *psy = chargimg->psy;
    union power_supply_propval level;

    /* the power supply may not get during driver initial, get it again */
    if (NULL == psy) {
        psy = power_supply_get_by_name("battery");
        if (NULL == psy) {
            printk("battery_level_update: couldn't get battery power supply\n");
            return;
        }
    }

    if (psy->get_property(psy, POWER_SUPPLY_PROP_CAPACITY_LEVEL, &level))
        return;

    printk("battery_level_update: %d\n", level.intval);
    chargimg->bat_level = level.intval - 1;
}

static void memset32(void *_ptr, unsigned int val, unsigned int count)
{
    unsigned int *ptr = _ptr;
    while (count--)
        *ptr++ = val;
}

static inline void sleep(unsigned jiffies)
{
    current->state = TASK_INTERRUPTIBLE;
    schedule_timeout(jiffies);
    return ;
}

static void show_charging_image(struct charging_img *chargimg)
{
    struct fb_info *info;
    struct fb_var_screeninfo var;

    int i = 0, j = 0;
    int index = 0;
    int byte;
    unsigned int lux, luy;
    unsigned int remain;
    unsigned char *pDstPtr;
    unsigned short *ptr;
    Screen *pImage;

    info = registered_fb[0];
    if (!info) {
        printk(KERN_WARNING "%s: Can not access framebuffer\n",
               __func__);
        return;
    }

    mutex_lock(&chargimg->drawing_lock);

    for (i=0; i<MAX_LOGO_NUM*2-1; i++) {
        if (chargimg->backlight_status == BACKLIGHT_OFF)
            break;

        if (chargimg->system_resume){
            fb_blank(registered_fb[0], FB_BLANK_UNBLANK);
            break;
        }
        
        if (i >= MAX_LOGO_NUM) {
            j++;
        }
        index = i-(2*j);

        pImage = (Screen*)&s_batterylogo;
        lux = (fb_width(info)-pImage->width) / 2;
        luy = (fb_height(info)-pImage->height) / 2;
        if (pImage->height > fb_height(info)) {
            luy = 0;
        }

        pDstPtr = (unsigned char*)(info->screen_base)+ luy*fb_width(info)*4 + lux*4;
        ptr =pImage->data[chargimg->bat_level][index];
        byte = pImage->byte[chargimg->bat_level][index];
        remain = pImage->width;

        while (byte >= 6) {
            unsigned short n = ptr[0];
            unsigned int c = ptr[1] + (ptr[2]<<16);

            while (n) {
                if (n >= remain) {
                    memset32(pDstPtr, c, remain), pDstPtr += remain*4;
                    n -= remain, remain = 0;
                } else {
                    memset32(pDstPtr, c, n), pDstPtr += n*4;
                    remain -= n, n = 0;
                }
                if (remain == 0) {
                    remain = pImage->width;
                    pDstPtr += fb_width(info)*4- pImage->stride;
                }
            }

            ptr += 3;
            byte -= 6;
        }

        sleep(6);
    }

    memcpy(&var, &info->var, sizeof(struct fb_var_screeninfo));

    var.activate = FB_ACTIVATE_NOW | FB_ACTIVATE_FORCE;
    fb_set_var(info, &var);
    mutex_unlock(&chargimg->drawing_lock);
}

static void light_backlight(struct charging_img *chargimg)
{
    struct backlight_device *bl = chargimg->bl_dev;

    if ( bl == NULL) {
        printk(KERN_WARNING "%s: Can not get backlight device\n",
               __func__);
        return;
    }

    chargimg->light_time = jiffies;
    chargimg->backlight_status = BACKLIGHT_LIGHT;
    bl->props.brightness = 224;
    backlight_update_status(bl);
    printk("Light the panel backlight\n");
}

static void dim_backlight(struct charging_img *chargimg)
{
    struct backlight_device *bl = chargimg->bl_dev;

    if ( bl == NULL) {
        printk(KERN_WARNING "%s: Can not get backlight device\n",
               __func__);
        return;
    }

    chargimg->light_time = jiffies;
    chargimg->backlight_status = BACKLIGHT_DIM;
    bl->props.brightness = 20;
    backlight_update_status(bl);
    printk("Dim the panel backlight\n");
}

static void turnoff_backlight(struct charging_img *chargimg)
{
    struct backlight_device *bl = chargimg->bl_dev;

    if ( bl == NULL) {
        printk(KERN_WARNING "%s: Can not get backlight device\n",
               __func__);
        return;
    }

    chargimg->backlight_status = BACKLIGHT_OFF;
    bl->props.brightness = 0;
    backlight_update_status(bl);
    printk("Turn off the panel backlight\n");
}

static int switch_gpio_notify(struct notifier_block *nb, unsigned long state, void *unused)
{
    struct charging_img *chargimg = 
        container_of(nb, struct charging_img, switch_gpio_notifier);

    if (chargimg->lock_fn_enabled) {
        if (state == SWITCH_LOCKED)
            chargimg->power_key_locked = true;
        else if (state == SWITCH_UNLOCKED)
            chargimg->power_key_locked = false;

        printk("%s: lock hardware is %s\n", __func__,  
               chargimg->power_key_locked ? "enable" : "disable");
    }

    return NOTIFY_DONE;
}

static int charging_img_thread(void *args)
{
    struct charging_img *chargimg = args;
    light_backlight(chargimg);

    do {
        if(!is_boot_on_charging)
            break;

        if (chargimg->backlight_status == BACKLIGHT_LIGHT &&
            ((jiffies - chargimg->light_time) > msecs_to_jiffies(60000))) {
            dim_backlight(chargimg);
        }

        if(chargimg->backlight_status == BACKLIGHT_DIM &&
           ((jiffies - chargimg->light_time) > msecs_to_jiffies(60000))) {
            queue_work(chargimg->suspend_workq, &chargimg->suspend_work);
        }

        battery_level_update(chargimg);
        if (chargimg->bat_level == -1){  //if get battery level fail, don't show animation
	    sleep(50);
	    continue;
	}

        wait_event_timeout(chargimg->wait_show_img, chargimg->show_img, msecs_to_jiffies(4000));

        show_charging_image(chargimg);

        chargimg->show_img = false;

        if (chargimg->system_resume) {
            chargimg->show_img = true;
            chargimg->system_resume = false;
        } else {
            chargimg->show_img = false;
        }
    } while (!kthread_should_stop());

    return 0;
}

extern void request_suspend_state(suspend_state_t new_state);
static void suspend(struct work_struct *work)
{
    struct charging_img *chargimg = container_of(work, struct charging_img, suspend_work);

    printk("suspending system....\n");
    mutex_lock(&chargimg->drawing_lock);
    chargimg->power_key_ignored = true;
    turnoff_backlight(chargimg);
    request_suspend_state(PM_SUSPEND_MEM);
    pm_suspend(PM_SUSPEND_MEM);
    request_suspend_state(PM_SUSPEND_ON);
    mutex_unlock(&chargimg->drawing_lock);

    if (machine_is_avalon())
        sleep(100);

    if (chargimg->power_key_locked) {
        turnoff_backlight(chargimg);  //even resume, don't turn on backlight
        queue_work(chargimg->suspend_workq, &chargimg->suspend_work);
    } else {
        chargimg->system_resume = true;
        chargimg->show_img = true;
        wake_up(&chargimg->wait_show_img);
        light_backlight(chargimg);
        chargimg->power_key_ignored = false;
    }
}

static void monitor_power_key(struct charging_img *chargimg, int is_pressed)
{
    printk("power key %s\n", is_pressed ? "pressed" : "released");

    if (chargimg->power_key_locked || 
        chargimg->suspend_workq == NULL ||
        chargimg->power_key_ignored)
        return;

    if (!is_pressed) { //action while key released
        queue_work(chargimg->suspend_workq, &chargimg->suspend_work);
    }
}

static void charging_img_event(struct input_handle *handle, unsigned int type,
                               unsigned int code, int value)
{
    struct charging_img *chargimg = handle->private;

    switch (code) {
    case KEY_POWER:
        monitor_power_key(chargimg, value);
        break;
    default:
        break;
    }
}

static int charging_img_connect(struct input_handler *handler,
                                struct input_dev *dev,
                                const struct input_device_id *id)
{
    struct input_handle *handle;
    int ret;

    handle = kzalloc(sizeof(struct input_handle), GFP_KERNEL);
    if (!handle)
        return -ENOMEM;

    handle->dev = dev;
    handle->handler = handler;
    handle->name = "charging-img";
    handle->private = handler->private;

    ret = input_register_handle(handle);
    if (ret) {
        printk (KERN_ERR "charging-img: Failed to register input power handler");
        kfree(handle);
        return ret;
    }

    ret = input_open_device(handle);
    if (ret) {
        printk(KERN_ERR "charging-img: Failed to open input power device");
        input_unregister_handle(handle);
        kfree(handle);
        return ret;
    }

    return 0;
}

static void charging_img_disconnect(struct input_handle *handle)
{
    input_close_device(handle);
    input_unregister_handle(handle);
    kfree(handle);
}

static const struct input_device_id charging_img_ids[] = {
    {
        .flags = INPUT_DEVICE_ID_MATCH_EVBIT,
        .evbit = { BIT_MASK(EV_KEY) },
    },
    { },
};

MODULE_DEVICE_TABLE(input, charging_img_ids);

static struct input_handler charging_img_handler = {
    .event = charging_img_event,
    .connect = charging_img_connect,
    .disconnect = charging_img_disconnect,
    .name = "charging-img",
    .id_table = charging_img_ids,
};

static int charging_img_probe(struct platform_device *pdev)
{
    struct charging_img_data *pdata = pdev->dev.platform_data;
    struct charging_img *chargimg;
    int ret;

    if (!is_boot_on_charging)
        return -ENODEV;

    chargimg = kzalloc(sizeof(struct charging_img), GFP_KERNEL);
    if (!chargimg)
        return -ENOMEM;

    chargimg->bat_level = -1;
    chargimg->lock_fn_enabled = nvtec_is_switch_lock_enabled();
    mutex_init(&chargimg->drawing_lock);

    chargimg->bl_dev = platform_get_drvdata(pdata->bl_pdev);
    /* may not get the battery power supply because battery driver not yet initialized*/
    chargimg->psy = power_supply_get_by_name("battery");
    chargimg->switch_gpio_notifier.notifier_call = switch_gpio_notify;
    switch_gpio_notifier_register(&chargimg->switch_gpio_notifier);

    chargimg->suspend_workq = create_singlethread_workqueue("suspend");
    if (chargimg->suspend_workq == NULL) {
        ret = -ENOMEM;
        goto error_create_wq;
    }

    INIT_WORK(&chargimg->suspend_work, suspend);
    init_waitqueue_head(&chargimg->wait_show_img);

    chargimg->task = kthread_run(charging_img_thread, chargimg, "charging-img");
    if (!chargimg->task) {
        ret = -EFAULT;
        goto error_thread;
    }

    charging_img_handler.private = chargimg;
    ret = input_register_handler(&charging_img_handler);
    if (ret < 0) {
        ret = -EFAULT;
        goto error_input_reg;
    }

    platform_set_drvdata(pdev, chargimg);

    return 0;
 error_input_reg:
    cancel_work_sync(&chargimg->suspend_work);
    kthread_stop(chargimg->task);
 error_thread:
    destroy_workqueue(chargimg->suspend_workq);
 error_create_wq:
    mutex_destroy(&chargimg->drawing_lock);
    kfree(chargimg);

    return ret;
}

static int charging_img_remove(struct platform_device *pdev)
{
    struct charging_img *chargimg = platform_get_drvdata(pdev);
    
    printk("charging_img_remove\n");
    input_unregister_handler(&charging_img_handler);
    cancel_work_sync(&chargimg->suspend_work);
    kthread_stop(chargimg->task);
    destroy_workqueue(chargimg->suspend_workq);
    switch_gpio_notifier_unregister(&chargimg->switch_gpio_notifier);
    mutex_destroy(&chargimg->drawing_lock);
    kfree(chargimg);

    return 0;
}

static struct platform_driver charging_img_driver = 
    {
        .probe = charging_img_probe,
        .remove = charging_img_remove,
        .driver = {
            .name = "charging-img",
            .owner = THIS_MODULE,
        },
    };

static int __init charging_img_init(void)
{
    if (!is_boot_on_charging)
        return -ENODEV;

    return platform_driver_register(&charging_img_driver);
}

static void __exit charging_img_exit(void)
{
    platform_driver_unregister(&charging_img_driver);
}

module_init(charging_img_init);
module_exit(charging_img_exit);

MODULE_DESCRIPTION("Power off Chaging Driver");
MODULE_AUTHOR("YuYang Chao <YuYang_Chao@pegatroncorp.com>");
MODULE_LICENSE("GPL");

void wait_boot_on_charging(void)
{
    if (is_boot_on_charging)
        {
            printk("wait for boot on charging ....\n");
            wait_event(wait_on_charging, !is_boot_on_charging);
        }
}

EXPORT_SYMBOL(wait_boot_on_charging);

static int __init set_boot_on_charging(char *str)
{
    is_boot_on_charging = 1;
    return 1;
}

__setup("poweroff_charge", set_boot_on_charging);

