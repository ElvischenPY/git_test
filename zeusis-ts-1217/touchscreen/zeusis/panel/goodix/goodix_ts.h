/*
 * goodix_ts.h - Touch driver header file of Goodix 
 * 
 * Copyright (C) 2015 - 2016 Goodix Technology Incorporated   
 * Copyright (C) 2015 - 2016 Yulong Cai <caiyulong@goodix.com>
 * 
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be a reference 
 * to you, when you are integrating the GOODiX's CTP IC into your system, 
 * but WITHOUT ANY WARRANTY; without even the implied warranty of 
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU 
 * General Public License for more details.
 *
 */

#ifndef _GOODIX_TS_H_
#define _GOODIX_TS_H_
#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/gpio.h>
#ifdef CONFIG_OF
#include <linux/regulator/consumer.h>
#include <linux/of_gpio.h>
#endif
#include <linux/input/zeusis_touchscreen.h>
#include "../../zeusis_touchscreen_algo.h"

#define GTP_DRIVER_VERSION          "v1.6<2016/07/08>"

#define IIC_MAX_TRANSFER_SIZE       250
#define GTP_CONFIG_ORG_LENGTH       239
#define GTP_CONFIG_EXT_LENGTH       128
#define GTP_CONFIG_MAX_LENGTH       240
#define GTP_CONFIG_MIN_LENGTH       186
#define SWITCH_OFF                  0
#define SWITCH_ON                   1
//for incell
#define GTP_INCELL_PANEL	1

/* request type */
#define GTP_RQST_CONFIG                 0x01
#define GTP_RQST_RESET                  0x03
#define GTP_RQST_NOISE_CFG				0x10
#define GTP_RQST_NORMA_CFG				0x11
#define GTP_RQST_RESPONDED              0x00
#define GTP_RQST_IDLE                   0xFF

/* Register define */
#define GTP_READ_COOR_ADDR          0x814E
#define GTP_REG_CMD                 0x8040
#define GTP_REG_SENSOR_ID           0x814A
#define GTP_REG_CONFIG_DATA         0x8050
#define GTP_REG_CONFIG_RESOLUTION   0x8051
#define GTP_REG_CONFIG_TRIGGER      0x8056
#define GTP_REG_CONFIG_CHECKSUM     0x813C
#define GTP_REG_CONFIG_UPDATE       0x813E
#define GTP_REG_EXT_CONFIG          0xBF7B
#define GTP_REG_VERSION             0x8140
#define GTP_REG_HW_INFO             0x4220
#define GTP_REG_REFRESH_RATE	    0x8056
#define GTP_REG_ESD_CHECK           0x8043
#define GTP_REG_FLASH_PASSBY        0x8006
#define GTP_REG_HN_PAIRED           0x81AA
#define GTP_REG_HN_MODE             0x81A8
#define GTP_REG_MODULE_SWITCH3      0x8058
#define GTP_REG_FW_CHK_MAINSYS      0x41E4
#define GTP_REG_FW_CHK_SUBSYS       0x5095
#define GTP_REG_WAKEUP_GESTURE		0x814C
#define GTP_REG_RQST                    0x8044
#define GTP_REG_HAVE_KEY                0x8057

/* cmd define */
#define GTP_CMD_SLEEP               0x05
#define GTP_CMD_CHARGER_ON          0x06
#define GTP_CMD_CHARGER_OFF         0x07
#define GTP_CMD_GESTURE_WAKEUP      0x08
#define GTP_CMD_CLEAR_CFG           0x10
#define GTP_CMD_ESD                 0xAA
#define GTP_CMD_HN_TRANSFER         0x22
#define GTP_CMD_HN_EXIT_SLAVE       0x28

#define GTP_MAX_TOUCH    			TS_MAX_FINGER
#define GTP_MAX_KEY_NUM  			4
#define GTP_ADDR_LENGTH  			2

#define SLEEP_MODE					0x01
#define GESTURE_MODE				0x02

/* define offset in the config*/
#define RESOLUTION_LOC              (GTP_REG_CONFIG_RESOLUTION - GTP_REG_CONFIG_DATA)
#define TRIGGER_LOC                 (GTP_REG_CONFIG_TRIGGER - GTP_REG_CONFIG_DATA)
#define MODULE_SWITCH3_LOC			(GTP_REG_MODULE_SWITCH3 - GTP_REG_CONFIG_DATA)
#define EXTERN_CFG_OFFSET			(0x805A - GTP_REG_CONFIG_DATA)

#ifdef ROI
#define ROI_STA_REG     0xA6A2
#define ROI_HEAD_LEN    4  
#define ROI_DATA_REG    (ROI_STA_REG + ROI_HEAD_LEN)
#define ROI_READY_MASK    0x10
#define ROI_TRACKID_MASK  0x0f
#endif

#define GOODIX_OF_NAME	"goodix"

#pragma pack(1)
struct goodix_hw_info {
	u8 product_id[5];
	u32 patch_id;
	u32 mask_id;
	u8 sensor_id;
	u8 match_opt;
};
#pragma pack()

struct fw_update_info {
	int update_type;
	int status;
	int progress;
	int max_progress;
	int force_update;
	struct fw_info *firmware_info;
	u32 fw_length;
	const struct firmware *fw;

	// file update
	char *fw_name;
	u8 *buffer;
	mm_segment_t old_fs;
	struct file *fw_file;

	// header update
	u8 *fw_data;
};

enum goodix_ts_feature {
	TS_FEATURE_NONE = 0,
	TS_FEATURE_GLOVE,
	TS_FEATURE_HOLSTER,
	TS_FEATURE_POCKET,
};

struct goodix_ts_config {
	bool initialized;
	char *name;
	u8 data[GTP_CONFIG_ORG_LENGTH + GTP_CONFIG_EXT_LENGTH];
	int size;
	int delay_ms;
};

#ifdef ROI
struct goodix_ts_roi {
	bool enabled;
	bool data_ready;
	int track_id;
	unsigned int roi_rows;
	unsigned int roi_cols;
	struct mutex mutex;
	u16 *rawdata;
};
#endif

struct goodix_ts_data;

struct goodix_ts_ops {
	int (*i2c_write)(u16 addr, u8 * buffer, s32 len);
	int (*i2c_read)(u16 addr, u8 * buffer, s32 len);
	int (*chip_reset)(void);
	int (*send_cmd)(u8 cmd, u8 data);
	int (*send_cfg)(struct goodix_ts_config *cfg_ptr);
	int (*i2c_read_dbl_check)(u16 addr, u8 * buffer, s32 len);
	int (*read_version)(struct goodix_hw_info * hw_info);
	int (*parse_cfg_data)(struct goodix_ts_data *ts,
			char *cfg_type, u8 *cfg, int *cfg_len, u8 sid);
	int (*feature_resume)(struct goodix_ts_data *ts);
};

struct goodix_ts_data {
	struct platform_device *pdev;
	struct regulator *vdd_ana;
	struct regulator *vcc_i2c;
	struct pinctrl *pinctrl;
	struct pinctrl_state *pins_default;
	struct pinctrl_state *pins_suspend;
	struct pinctrl_state *pins_gesture;
	struct ts_device_data *dev_data;

	struct goodix_ts_ops ops;
	struct goodix_hw_info hw_info;
	struct goodix_ts_config normal_config;
	struct goodix_ts_config normal_noise_config;
	struct goodix_ts_config glove_config;
	struct goodix_ts_config glove_noise_config;
	struct goodix_ts_config holster_config;
	struct goodix_ts_config charger_config;
	struct goodix_ts_config pocket_config;
#ifdef ROI
	struct goodix_ts_roi roi;
#endif
	int max_x;
	int max_y;
	int slide_max_x;
	int slide_max_y;
	u32 vdd_value;
	u32 vio_value;
	bool flip_x;
	bool flip_y;

	char firmware_name[64];
	volatile bool rawdiff_mode;
	bool noise_env;
	bool tools_support;
// debug+
	bool sensor_id_valid;
	bool fw_update_ok;
// debug-
};

extern struct goodix_ts_data *goodix_ts;
extern struct fw_update_info update_info;
extern struct ts_data g_ts_data;

#define UPDATE_TYPE_HEADER 0
#define UPDATE_TYPE_FILE   1

//Log define
#define  GTP_FW_UPDATE_VERIFY   0 // verify fw when updating
//#define INPUT_TYPE_B_PROTOCOL

#define GTP_DEBUG_FUNC_ON 	0
#define GTP_INFO(fmt,arg...)           TS_LOG_INFO("[GTP-INF][%s:%d] "fmt"\n", __func__, __LINE__, ##arg)
#define GTP_ERROR(fmt,arg...)          TS_LOG_ERR("[GTP-ERR][%s:%d] "fmt"\n", __func__, __LINE__, ##arg)
#define GTP_DEBUG(fmt,arg...)          TS_LOG_DEBUG("[GTP-DBG][%s:%d]"fmt"\n",__func__, __LINE__, ##arg)
#define GTP_DEBUG_FUNC()               do{\
										 if(GTP_DEBUG_FUNC_ON)\
										 TS_LOG_DEBUG("[GTP-FUNC] Func:%s@Line:%d\n",__func__,__LINE__);\
									   }while(0)
#define IS_NUM_OR_CHAR(x)    (((x) >= 'A' && (x) <= 'Z') || ((x) >= '0' && (x) <= '9'))

int goodix_i2c_write(u16 addr, u8 * buffer, s32 len);
int goodix_i2c_read(u16 addr, u8 * buffer, s32 len);
int goodix_chip_reset(void);
int goodix_read_version(struct goodix_hw_info * hw_info);
int goodix_i2c_read_dbl_check(u16 addr, u8 * buffer, s32 len);
extern int gt1x_init_tool_node(void);
extern int gt1x_auto_update_proc(int type);
extern int goodix_get_rawdata(struct ts_rawdata_info *info,
				struct ts_cmd_node *out_cmd);
#endif /* _GOODIX_TS_H_ */
