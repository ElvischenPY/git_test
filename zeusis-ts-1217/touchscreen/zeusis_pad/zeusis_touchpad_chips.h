/*
 * Zeusis Touchscreen chips
 *
 * Copyright (C) 2013 Zeusis Device Co.Ltd
 * License terms: GNU General Public License (GPL) version 2
 *
 */
 /* this file list all support ts chip information */
#ifndef __ZEUSIS_TOUCHPAD_CHIP_H_
#define __ZEUSIS_TOUCHPAD_CHIP_H_
#include <linux/input/zeusis_touchpad.h>
#ifdef CONFIG_TOUCHSCREEN_SYNAPTICS_PAD
extern struct ts_device_ops pad_ts_synaptics_ops;
#endif
#ifdef CONFIG_CYPRESS_PAD
extern struct ts_device_ops ts_cypress_ops;
#endif
#ifdef CONFIG_WACOM_PAD
extern struct ts_device_ops ts_wacom_ops;
#endif
#ifdef CONFIG_HIDEEP_PAD
extern struct ts_device_ops ts_hideep_ops;
#endif
#ifdef CONFIG_TOUCHSCREEN_GOODIX_PAD
extern struct ts_device_ops pad_ts_goodix_ops;
#endif
#endif
