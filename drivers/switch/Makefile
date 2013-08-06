# Switch Class Driver
obj-$(CONFIG_SWITCH)		+= switch_class.o
obj-$(CONFIG_SWITCH_GPIO)	+= switch_gpio.o
obj-$(CONFIG_SWITCH_AST_DOCK)  += switch_ast_dock.o
# Building only when borad have 3G
ifeq ($(BOARD_HAVE_3G),true)
obj-$(CONFIG_SENSORS_IQS128)	+= iqs128_cap_sensor.o
endif

