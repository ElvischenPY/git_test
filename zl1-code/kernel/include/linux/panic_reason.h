/* Copyright (c) 2013-2014, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef __RESTART_REASON_H__
#define __RESTART_REASON_H__

#define TRIG_INIT_STATE (0x01)

#define TRIG_CMD_REBOOT (0x21)
#define TRIG_MODEM_RESET (0x22)
#define TRIG_SYSRQ_CRASH (0x23)
#define TRIG_OVER_TEMPERATURE (0x24)
#define TRIG_KERNEL_BUG (0x25)
#define TRIG_SLUB_BUG (0x26)
#define TRIG_NULL_POINTER (0x27)
#define TRIG_LONG_PRESS_PWR_KEY (0x28)
#define KERNEL_ALIVE (1 << 7)

#define PANIC_REASON_BROKEN (0xff)

extern void set_panic_trig_rsn(u8 val);
void panic_reason_hook(void *p);
#endif
