/*
 * Zeusis Touchscreen chips
 *
 * Copyright (C) 2013 Zeusis Device Co.Ltd
 * License terms: GNU General Public License (GPL) version 2
 *
 */
 /* this file list all support ts chip information */
#ifndef __ZEUSIS_TOUCHSCREEN_CHIP_H_
#define __ZEUSIS_TOUCHSCREEN_CHIP_H_
#include <linux/input/zeusis_touchscreen.h>
#ifdef CONFIG_TOUCHSCREEN_SYNAPTICS_TS
extern struct ts_device_ops ts_synaptics_ops;
#endif
#ifdef CONFIG_CYPRESS_TS
extern struct ts_device_ops ts_cypress_ops;
#endif
#ifdef CONFIG_WACOM_TS
extern struct ts_device_ops ts_wacom_ops;
#endif
#ifdef CONFIG_HIDEEP_TS
extern struct ts_device_ops ts_hideep_ops;
#endif
#ifdef CONFIG_TOUCHSCREEN_GOODIX_TS
extern struct ts_device_ops ts_goodix_ops;
#endif
#endif
