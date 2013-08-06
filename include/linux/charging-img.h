#ifndef __CHARGING_IMG__
#define __CHARGING_IMG__

struct charging_img_data {
    unsigned int ac_in_pin;
    struct platform_device *bl_pdev;
};

extern void wait_boot_on_charging(void);
extern int is_boot_on_charging;

#endif

