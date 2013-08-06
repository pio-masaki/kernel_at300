#ifndef __SWITCH_GPIO__
#define __SWITCH_GPIO__

/* switch_gpio_notifier status */
/* Locking status */
#define SWITCH_UNLOCKED		(0)
#define SWITCH_LOCKED		(1)
/* Lock hardware is enabled or disabled */
#define SWITCH_LOCK_ENABLED	(2)
#define SWITCH_LOCK_DISABLED	(3)

extern int switch_gpio_notifier_register(struct notifier_block *nb);
extern int switch_gpio_notifier_unregister(struct notifier_block *nb);
extern int switch_gpio_lock_hardware_notifier_register(struct notifier_block *nb);
extern int switch_gpio_lock_hardware_notifier_unregister(struct notifier_block *nb);

#endif
