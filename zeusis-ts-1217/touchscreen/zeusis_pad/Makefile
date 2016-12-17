# Makefile for the irda drivers.
#

# Each configuration option enables a list of files.

obj-$(CONFIG_TOUCHSCREEN_ZEUSIS_PAD)  += zeusis_touchpad.o
obj-$(CONFIG_TOUCHSCREEN_ZEUSIS_PAD) += panel_pad/
obj-$(CONFIG_TOUCHSCREEN_ZEUSIS_PAD)  += zeusis_touchpad_algo.o
obj-$(CONFIG_ZEUSIS_TS_TEST)  += zeusis_touchpad_test.o

obj-$(CONFIG_ZEUSIS_TP_COLOR_INFO) += tp_color.o

ifeq ($(CONFIG_ZEUSIS_TS_TEST),y)
EXTRA_CFLAGS += -DZEUSIS_TOUCHSCREEN_TEST
endif
