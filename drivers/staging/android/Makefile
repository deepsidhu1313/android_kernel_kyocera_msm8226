#
# This software is contributed or developed by KYOCERA Corporation.
# (C) 2014 KYOCERA Corporation
# (C) 2015 KYOCERA Corporation
#
obj-$(CONFIG_ANDROID_BINDER_IPC)	+= binder.o
obj-$(CONFIG_ASHMEM)			+= ashmem.o
obj-$(CONFIG_ANDROID_LOGGER)		+= logger.o
obj-$(CONFIG_ANDROID_PERSISTENT_RAM)	+= persistent_ram.o
obj-$(CONFIG_ANDROID_RAM_CONSOLE)	+= ram_console.o
obj-$(CONFIG_ANDROID_TIMED_OUTPUT)	+= timed_output.o
obj-$(CONFIG_ANDROID_TIMED_GPIO)	+= timed_gpio.o
obj-$(CONFIG_ANDROID_LOW_MEMORY_KILLER)	+= lowmemorykiller.o
obj-$(CONFIG_ANDROID_SWITCH)		+= switch/
obj-$(CONFIG_ANDROID_INTF_ALARM_DEV)	+= alarm-dev.o
obj-$(CONFIG_PERSISTENT_TRACER)		+= trace_persistent.o
obj-y                                   += kcjlog.o

CFLAGS_REMOVE_trace_persistent.o = -pg

# /* RESETLOG_BUILD_DISPLAY_ID は、build/core/Makefile への対応が必要 */
#CFLAGS_logger.o += -DBUILD_DISPLAY_ID='"$(RESETLOG_BUILD_DISPLAY_ID)"'
# /* RESETLOG_PRODUCT_MODEL_NAME は、 device/qcom/msm8226/msm8226.mk への対応が必要 */
#CFLAGS_logger.o += -DPRODUCT_MODEL_NAME='"$(RESETLOG_PRODUCT_MODEL_NAME)"'
CFLAGS_logger.o += -DBUILD_DISPLAY_ID='"JSS15J eng,release-keys"'
CFLAGS_logger.o += -DPRODUCT_MODEL_NAME='"G-81"'
