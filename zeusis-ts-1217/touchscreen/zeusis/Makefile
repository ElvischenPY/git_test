# Makefile for the irda drivers.
#

# Each configuration option enables a list of files.

obj-$(CONFIG_TOUCHSCREEN_ZEUSIS_TS)  += zeusis_touchscreen.o
obj-$(CONFIG_TOUCHSCREEN_ZEUSIS_TS) += panel/
obj-$(CONFIG_TOUCHSCREEN_ZEUSIS_TS)  += zeusis_touchscreen_algo.o
obj-$(CONFIG_ZEUSIS_TS_TEST)  += zeusis_touchscreen_test.o

obj-$(CONFIG_ZEUSIS_TP_COLOR_INFO) += tp_color.o

ifeq ($(CONFIG_ZEUSIS_TS_TEST),y)
EXTRA_CFLAGS += -DZEUSIS_TOUCHSCREEN_TEST
endif
