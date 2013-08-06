#include <linux/fb.h>
#include <linux/input.h>
#include <linux/kthread.h>
#include <linux/delay.h>
#include <linux/suspend.h>
#include <linux/mutex.h>
#include <linux/reboot.h>
#include <linux/gpio.h>
#include <linux/charging-img.h>
#include <linux/mfd/nvtec.h>
#include <linux/switch_gpio.h>
#include <linux/notifier.h>

#ifdef CONFIG_TSPDRV
#include <asm/mach-types.h>
#include <linux/pwm.h>
#include <../gpio-names.h>
#endif

#include "image/battery.h"

#define fb_width(fb)	((fb)->var.xres)
#define fb_height(fb)	((fb)->var.yres)

struct charging_img {
  struct task_struct *task;
  unsigned int ac_in_pin;
  struct notifier_block switch_gpio_notifier;
  struct notifier_block ac_notifier;
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

#ifdef CONFIG_TSPDRV
struct pwm_vibrator {
  int vibrator_en_pin;
  struct pwm_device *pwm;
  unsigned long duty_cycle;
  unsigned long pwm_period;
  unsigned long	timeout;
  struct notifier_block dock_notifier;
  struct regulator *reg;
  bool on_dock;
  bool amp_enabled;
};
#endif

int is_boot_on_charging = 0;
EXPORT_SYMBOL(is_boot_on_charging);
struct workqueue_struct *suspend_workq;

static struct charging_img *chargimg;
static DEFINE_MUTEX(drawing_lock);

static unsigned long dim_time = 0;
static int backlight_is_dimmed = 0;
static void suspend(struct work_struct *work);
static DECLARE_WORK(suspend_work, suspend);
static struct input_handler charging_img_handler;
static DECLARE_WAIT_QUEUE_HEAD(wait_show_img);
static DECLARE_WAIT_QUEUE_HEAD(wait_on_charging);

static struct platform_driver charging_img_driver;
static struct timer_list powerkey_press_timer;

static unsigned long update_time = 0;
static int show_img = 0;
static int suspend_keep = 0;
static int bat_level = -1;
static int reboot = 0;
static int lock_fn_enabled = 0;
static int power_key_locked = 0;
static int power_key_ignored = 0;
static int init_done = 0;
static int shutdown_by_mcu = 0;

static void battery_level_update(void)
{
  int value = -EINVAL;
  int retry = 3;

  do {
    value = nvtec_battery_get_level();
    if (value < 0) {
      printk (KERN_ERR "charging-img: Failed to get battery level\n");
      retry--;

      if (retry == 0)
        value = bat_level;  //if failed, keep last battery level
    }
  }  while (retry > 0 && value == -EINVAL);

  if (nvtec_battery_full_charged()) {
    value = 4;
  }

  bat_level = value;
  update_time = jiffies;
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

void show_charging_image(void)
{
  struct fb_info *info;
  struct fb_var_screeninfo var;

  int i=0, j=0;
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

  mutex_lock(&drawing_lock);

  for (i=0; i<MAX_LOGO_NUM*2-1; i++)
  {
    if (suspend_keep || reboot) break;

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
    ptr =pImage->data[bat_level][index];
    byte = pImage->byte[bat_level][index];
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
  mutex_unlock(&drawing_lock);
}

extern struct backlight_device *get_backlight_dev(void);
void light_backlight(void)
{
  struct backlight_device *bl;

  bl = get_backlight_dev();
  if ( bl == NULL)
  {
    printk(KERN_WARNING "%s: Can not get backlight device\n",
	   __func__);
    return;
  }

  printk("Light the panel backlight\n");
  dim_time = jiffies;
  backlight_is_dimmed = 0;
  bl->props.brightness = 224;
  backlight_update_status(bl);
  
}

void dim_backlight(void)
{
  struct backlight_device *bl;

  bl = get_backlight_dev();
  if ( bl == NULL)
  {
    printk(KERN_WARNING "%s: Can not get backlight device\n",
           __func__);
    return;
  }

  printk("Dim the panel backlight\n");
  bl->props.brightness = 20;
  backlight_update_status(bl);
  dim_time = jiffies;
  backlight_is_dimmed = 1;
}

void turnoff_backlight(void)
{
  struct backlight_device *bl;

  bl = get_backlight_dev();
  if ( bl == NULL)
  {
    printk(KERN_WARNING "%s: Can not get backlight device\n",
           __func__);
    return;
  }

  printk("Turn off the panel backlight\n");
  bl->props.brightness = 0;
  backlight_update_status(bl);
}

static int switch_gpio_notify(struct notifier_block *nb, unsigned long state, void *unused)
{
  if (lock_fn_enabled) {
    if (state == SWITCH_LOCKED)
      power_key_locked = true;
    else if (state == SWITCH_UNLOCKED)
      power_key_locked = false;

    printk("%s: lock hardware is %s\n", __func__,  power_key_locked ? "enable" : "disable");
  }

  return NOTIFY_DONE;
}

static int ac_notify(struct notifier_block *nb, unsigned long state, void *unused)
{
  if(state==0){
    printk("AC removed, shutdown system....\n");
    turnoff_backlight();
    kernel_power_off();
  }
  return NOTIFY_DONE;
}

static int charging_img_thread(void *args)
{
  battery_level_update();
  light_backlight();
  show_img = 1;

  do {
    if(!is_boot_on_charging)
      break;

    if (reboot) {
      kernel_restart("charging-img");
    }
	
    if ((jiffies - update_time) > msecs_to_jiffies(30000)) {
      battery_level_update();
    }

    if (!backlight_is_dimmed &&
       ((jiffies - dim_time) > msecs_to_jiffies(60000))) {
      dim_backlight();
    }

    if(backlight_is_dimmed &&
      ((jiffies - dim_time) > msecs_to_jiffies(60000))) {
      queue_work(suspend_workq, &suspend_work);
    }

    if (bat_level == -1 || !init_done)  //if get battery level fail, don't show animation
	  continue;

    wait_event_timeout(wait_show_img, show_img, msecs_to_jiffies(4000));

    show_charging_image();

    if (suspend_keep) {
      show_img = 1;
      suspend_keep = 0;
    } else {
      show_img = 0;
    }
  } while (!kthread_should_stop());

  platform_driver_unregister(&charging_img_driver);

  /* input_unregister_handler(&charging_img_handler); */
  /* destroy_workqueue(suspend_workq); */
  /* kfree(chargimg); */
  return 0;
}

extern void request_suspend_state(suspend_state_t new_state);
static void suspend(struct work_struct *work)
{
  printk("suspending system....\n");
  mutex_lock(&drawing_lock);
  request_suspend_state(PM_SUSPEND_MEM);
  pm_suspend(PM_SUSPEND_MEM);
  request_suspend_state(PM_SUSPEND_ON);
  mutex_unlock(&drawing_lock);

  if (power_key_locked) {
    turnoff_backlight();  //even resume, don't turn on backlight
    queue_work(suspend_workq, &suspend_work);
  } else {
    suspend_keep = 1;
    show_img = 1;
    battery_level_update();
    wake_up(&wait_show_img);
    light_backlight();
    power_key_ignored = 0;
  }
}

#ifdef CONFIG_TSPDRV
extern struct pwm_vibrator *get_pwm_vib(void);
static void vibrate(void)
{
  unsigned long duty_cycle = 0;
  struct pwm_vibrator *vib;

  vib = get_pwm_vib();
  if ( vib == NULL)
  {
    printk(KERN_WARNING "%s: Can not get pwm device\n",
	   __func__);
    return;
  }

  if (machine_is_avalon())
    duty_cycle = 15000;
  else if (machine_is_sphinx())
    duty_cycle = 16000;
  else if (machine_is_titan())
    duty_cycle = 16500;

  if (vib->on_dock==false || machine_is_titan()) {
    pwm_config(vib->pwm, duty_cycle, vib->pwm_period);
    pwm_enable(vib->pwm);
    gpio_direction_output(vib->vibrator_en_pin, 1);
    mdelay(300);
    gpio_direction_output(vib->vibrator_en_pin, 0);
    pwm_disable(vib->pwm);
  }
}
#else
static void vibrate(void)
{
  return;
}
#endif
static void boot_android(unsigned long val)
{
  del_timer(&powerkey_press_timer);

  reboot = 1;
  vibrate();
}

static void monitor_power_key(int is_pressed)
{
  printk("power key %s\n", is_pressed?"pressed":"released");

  if (power_key_ignored)
    return;

  if (is_pressed) {
    if (shutdown_by_mcu == 0) {
      init_timer(&powerkey_press_timer);
      powerkey_press_timer.function = boot_android;
      powerkey_press_timer.expires = jiffies;
      powerkey_press_timer.data = 0;
      mod_timer(&powerkey_press_timer,jiffies + (2*HZ));
    }
  } else {
    if (!reboot)
    {
      if (shutdown_by_mcu == 0) {
        del_timer(&powerkey_press_timer);
      }
      power_key_ignored = 1;
      queue_work(suspend_workq, &suspend_work);
    }
  }
}

static void charging_img_event(struct input_handle *handle, unsigned int type,
                               unsigned int code, int value)
{
  switch (code) {
    case KEY_POWER:
      if (init_done)
        monitor_power_key(value);
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
  int ret;

  struct charging_img_data *pdata = pdev->dev.platform_data;
  struct charging_img *charging;

  if (!is_boot_on_charging)
    return -ENODEV;

  charging = kzalloc(sizeof(struct charging_img), GFP_KERNEL);
  if (!charging)
      return -ENOMEM;

  lock_fn_enabled = nvtec_hw_button_lock_configured();

  shutdown_by_mcu = nvtec_mcu_chk_version();
  if (shutdown_by_mcu < 0)
    printk(KERN_ERR "charging-img: Can't MCU firmware version\n");

  chargimg = charging;
  charging->ac_in_pin = pdata->ac_in_pin;
  charging->switch_gpio_notifier.notifier_call = switch_gpio_notify;
  switch_gpio_notifier_register(&charging->switch_gpio_notifier);

  charging->ac_notifier.notifier_call = ac_notify;
  nvtec_ac_notifier_register(&charging->ac_notifier);

  suspend_workq = create_singlethread_workqueue("suspend");
  if (suspend_workq == NULL) {
    kfree(charging);
    return -ENOMEM;
  }

  charging->task = kthread_run(charging_img_thread, charging, "charging-img");
  if (!charging->task) {
    destroy_workqueue(suspend_workq);
    kfree(charging);
    return -EFAULT;
  }

  ret = input_register_handler(&charging_img_handler);
  if (ret < 0) {
    kthread_stop(charging->task);
    destroy_workqueue(suspend_workq);
    kfree(charging);
    return -EFAULT;
  }

  return 0;
}

static int charging_img_remove(struct platform_device *pdev) {
  printk("charging_img_remove\n");
  input_unregister_handler(&charging_img_handler);
  /* kthread_stop(chargimg->task); */
  destroy_workqueue(suspend_workq);
  switch_gpio_notifier_unregister(&chargimg->switch_gpio_notifier);
  nvtec_ac_notifier_unregister(&chargimg->ac_notifier);
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

void wait_boot_on_charging(void)
{
  if (is_boot_on_charging)
  {
    printk("wait for boot on charging ....\n");

    init_done = 1;
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

