#
# Makefile for the input core drivers.
#

# Each configuration option enables a list of files.
obj-$(CONFIG_TOUCHSCREEN_FOCALTECH_FWVGA_H670_KEWEI)  += focaltech_ts.o
#focaltech_ts-objs += focaltech.o focaltech_ctl.o focaltech_ex_fun.o
focaltech_ts-objs := focaltech.o
focaltech_ts-objs += focaltech_ctl.o 
focaltech_ts-objs += focaltech_ex_fun.o
focaltech_ts-objs += ft6x36_download_lib.a 
#focaltech_ts-objs += ft_gesture_lib.a 
focaltech_ts-objs += ft_gesture1209V01_lib.a
#focaltech_ts-y    += focaltech.o focaltech_ctl.o focaltech_ex_fun.o
