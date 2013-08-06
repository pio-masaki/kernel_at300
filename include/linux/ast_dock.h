#ifndef _AST_DOCK_H
#define _AST_DOCK_H

#include <linux/notifier.h>

struct dock_platform_data {
	unsigned int	irq;
	unsigned int    dock_on_pin;
    unsigned int    dock_amp_enb_pin;
	unsigned int	ac_dock_pin;
};

#ifdef CONFIG_SWITCH_AST_DOCK
extern int ast_dock_notifier_register(struct notifier_block *nb);
extern int ast_dock_notifier_unregister(struct notifier_block *nb);
#else
static inline int ast_dock_notifier_register(struct notifier_block *nb)
{
    return 0;
}
static inline int ast_dock_notifier_unregister(struct notifier_block *nb)
{
    return 0;
}
#endif

#endif
