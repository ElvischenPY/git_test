/*
 * goodix_ts.c - Main touch driver file of Goodix
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
#include <linux/fs.h>

#include <linux/irq.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include "goodix_ts.h"

struct goodix_ts_data *goodix_ts;

/**
 * goodix_i2c_write - i2c write.
 * @addr: register address.
 * @buffer: data buffer.
 * @len: the bytes of data to write.
 * Return: 0: success, otherwise: failed
 */
int goodix_i2c_write(u16 addr, u8 * buffer, s32 len)
{
	struct ts_device_data *dev_data = goodix_ts->dev_data;
	u8 stack_mem[32], *data;
	int ret;

	if (unlikely(!dev_data || !dev_data->bops->bus_write))
		return -ENODEV;

	if (len + 2 > sizeof(stack_mem)) {
		data = kmalloc(len + 2, GFP_KERNEL);
		if (!data) {
			GTP_ERROR("No memory");
			return -ENOMEM;
		}
	} else {
		data = &stack_mem[0];
	}

	data[0] = addr >> 8 & 0xff;
	data[1] = addr & 0xff;
	memcpy(&data[2], buffer, len);
	ret = dev_data->bops->bus_write(data, len + 2);
	if (ret < 0)
		GTP_ERROR("i2c write error,addr:%04x bytes:%d", addr, len);

	if (data != &stack_mem[0])
		kfree(data);

	return ret;
}

/**
 * goodix_i2c_read - i2c read.
 * @addr: register address.
 * @buffer: data buffer.
 * @len: the bytes of data to write.
 * Return: 0: success, otherwise: failed
 */
int goodix_i2c_read(u16 addr, u8 * buffer, s32 len)
{
	struct ts_device_data *dev_data = goodix_ts->dev_data;
	int ret;
	
	if (unlikely(!dev_data || !dev_data->bops->bus_read))
		return -ENODEV;

	addr = cpu_to_be16(addr);
	ret = dev_data->bops->bus_read((u8 *)&addr, 2, buffer, len);
	if (ret < 0)
		GTP_ERROR("i2c read error,addr:%04x bytes:%d", addr, len);

	return ret;
}

/**
 * goodix_i2c_read_dbl_check - read twice and double check
 * @addr: register address
 * @buffer: data buffer
 * @len: bytes to read
 * Return    <0: i2c error, 0: ok, 1:fail
 */
int goodix_i2c_read_dbl_check(u16 addr, u8 * buffer, s32 len)
{
	u8 buf[16] = {0};
	u8 confirm_buf[16] = {0};
	int ret;

	if (len > 16) {
		GTP_ERROR("i2c_read_dbl_check length %d is too long, exceed %zu",
			len, sizeof(buf));
		return -EINVAL;
	}

	memset(buf, 0xAA, sizeof(buf));
	ret = goodix_i2c_read(addr, buf, len);
	if (ret < 0)
		return ret;

	msleep(5);
	memset(confirm_buf, 0, sizeof(confirm_buf));
	ret = goodix_i2c_read(addr, confirm_buf, len);
	if (ret < 0)
		return ret;

	if (!memcmp(buf, confirm_buf, len)) {
		memcpy(buffer, confirm_buf, len);
		return 0;
	}

	GTP_ERROR("i2c read 0x%04X, %d bytes, double check failed!", addr, len);
	return 1;
}

/**
 * goodix_send_cfg - Send config data to hw
 * @cfg_ptr: pointer to config data structure
 * Return 0--success,non-0--fail.
 */
static int goodix_send_cfg(struct goodix_ts_config *cfg_ptr)
{
	static DEFINE_MUTEX(mutex_cfg);
	u8 *config;
	int i, cfg_len;
	s32 ret = 0, retry = 0;
	u16 checksum = 0;

	if (!cfg_ptr || !cfg_ptr->initialized) {
		GTP_ERROR("Invalid config data");
		return -EINVAL;
	}

	config = &cfg_ptr->data[0];
	cfg_len = cfg_ptr->size;

	mutex_lock(&mutex_cfg);
	GTP_INFO("Send %s,ver:%02x size:%d", cfg_ptr->name, config[0], cfg_len);

	if (cfg_len != GTP_CONFIG_ORG_LENGTH
		&& cfg_len != GTP_CONFIG_ORG_LENGTH + GTP_CONFIG_EXT_LENGTH) {
		GTP_ERROR("Invalid config size:%d", cfg_len);
		mutex_unlock(&mutex_cfg);
		return -1;
	}

	/* Extends config */
	if (config[EXTERN_CFG_OFFSET] & 0x40) {
		int total_len = GTP_CONFIG_EXT_LENGTH + GTP_CONFIG_ORG_LENGTH;
		for (i = GTP_CONFIG_ORG_LENGTH; i < total_len; i += 2)
			checksum += (config[i] << 8) + config[i + 1]; 

		if (!checksum){
			GTP_ERROR("Invalid ext config,all of the bytes is zero");
			mutex_unlock(&mutex_cfg);
			return -1;
		}

		checksum = 0 - checksum;
		config[total_len - 2] = (checksum >> 8) & 0xFF;
		config[total_len - 1] = checksum && 0xFF;

		do {
			ret = goodix_i2c_write(GTP_REG_EXT_CONFIG, 
				&config[GTP_CONFIG_ORG_LENGTH], GTP_CONFIG_EXT_LENGTH);
		} while (ret < 0 && retry++ < 3);
	}

	for (i = 0, checksum = 0; i < cfg_len - 3; i += 2)
		checksum += (config[i] << 8) + config[i + 1];
	
	if (!checksum) {
		GTP_ERROR("Invalid config,all of the bytes is zero");
		mutex_unlock(&mutex_cfg);
		return -1;
	}

	checksum = 0 - checksum;
	config[cfg_len - 3] = (checksum >> 8) & 0xFF;
	config[cfg_len - 2] = checksum & 0xFF;
	config[cfg_len - 1] = 0x01;
	retry = 0;
	while (retry++ < 3) {
		ret = goodix_i2c_write(GTP_REG_CONFIG_DATA, config, cfg_len);
		if (!ret) {
			if (cfg_ptr->delay_ms > 0)
				msleep(cfg_ptr->delay_ms);
			mutex_unlock(&mutex_cfg);
			GTP_INFO("Send config successfully");
			return 0;
		}
	}

	GTP_ERROR("Send config failed");
	mutex_unlock(&mutex_cfg);
	return ret;
}

/**
 * goodix_send_cmd - seng cmd
 * must write data & checksum first
 * byte    content
 * 0       cmd
 * 1       data
 * 2       checksum
 * Returns 0 - succeed,non-0 - failed
 */
static int goodix_send_cmd(u8 cmd, u8 data)
{
	s32 ret;
	static DEFINE_MUTEX(cmd_mutex);
	u8 buffer[3] = { cmd, data, 0 };

	GTP_DEBUG("Send command:%u", cmd);
	mutex_lock(&cmd_mutex);
	buffer[2] = (u8) ((0 - cmd - data) & 0xFF);
	ret = goodix_i2c_write(GTP_REG_CMD + 1, &buffer[1], 2);
	ret |= goodix_i2c_write(GTP_REG_CMD, &buffer[0], 1);
	msleep(50);
	mutex_unlock(&cmd_mutex);

	return ret;
}

/**
 * goodix_init_watchdog - esd mechannism
 * 
 * Returns  0--success,non-0--fail.
 */
static int goodix_init_watchdog(void)
{
	/* 0x8040 ~ 0x8043 */
	u8 value[] = {0xAA, 0x00, 0x56, 0xAA};

	GTP_DEBUG("Init watchdog");
	return goodix_i2c_write(GTP_REG_CMD, &value[0], 4);
}

/**
 * goodix_switch_wrokmode - Switch working mode.
 * @workmode: GTP_CMD_SLEEP - Sleep mode
 *			  GESTURE_MODE - gesture mode
 * Returns  0--success,non-0--fail.
 */
static int goodix_switch_wrokmode(int wrokmode)
{
	s32 retry = 0;
	u8 cmd;

	switch (wrokmode) {
	case SLEEP_MODE:
		cmd = GTP_CMD_SLEEP;
		break;
	case GESTURE_MODE:
		cmd = GTP_CMD_GESTURE_WAKEUP;
		break;
	default:
		return -EINVAL;
	}

	GTP_INFO("Switch working mode[%02X]", cmd);
	while (retry++ < 3) {
		if (!goodix_send_cmd(cmd, 0))
			return 0;
		msleep(20);
	}

	GTP_ERROR("Failed to switch working mode");
	return -1;
}
int noise_test(void)
{
	int irq_gpio;
	irq_gpio = goodix_ts->dev_data->irq_gpio;
	
	gpio_direction_output(irq_gpio, 1);
	goodix_send_cmd(GTP_CMD_SLEEP,0);	
	return 0;
}

static int goodix_feature_switch(struct goodix_ts_data *ts,
	enum goodix_ts_feature fea, int on)
{
	struct ts_feature_info *info = &g_ts_data.feature_info;
	struct goodix_ts_config *config = NULL;
	int ret;

	if (!ts || !info)
		return -EINVAL;

	if (on == SWITCH_ON) {
		switch (fea) {
		case TS_FEATURE_NONE:
			config = &ts->normal_config;
			break;
		case TS_FEATURE_GLOVE:
			config = &ts->glove_config;
			break;
		case TS_FEATURE_HOLSTER:
			config = &ts->holster_config;
			break;
		case TS_FEATURE_POCKET:
			config = &ts->pocket_config;
			break;
		default:
			return -EINVAL;
		}
	} else if (on == SWITCH_OFF) {
		if (info->holster_info.holster_switch)
			config = &ts->holster_config;
		/**else if (info->glove_info.glove_switch)
			config = &ts->glove_config;*/
		else
			config = &ts->normal_config;
	}

	ts->noise_env = false;
	ret = goodix_send_cfg(config);
	return ret;
}

static int goodix_feature_resume(struct goodix_ts_data *ts)
{

	struct ts_feature_info *info = &g_ts_data.feature_info;
	struct goodix_ts_config *config = NULL;
	int ret = 0;

	
	if (info->holster_info.holster_switch) {
		config = &ts->holster_config;
	/*} else if (info->glove_info.glove_switch) {
		if (ts->noise_env)
			config = &ts->glove_noise_config;
		else
			config = &ts->glove_config;*/
	} else {
		if (ts->noise_env)
			config = &ts->normal_noise_config;
		else
			config = &ts->normal_config;	
	}

	ret = goodix_send_cfg(config);
#if 0
	if (info->charger_info.charger_switch) {
		ret = goodix_send_cmd(GTP_CMD_CHARGER_ON, 0x00);
		GTP_INFO("Charger switch on");
	}
#endif
	return ret;
}

static int goodix_noise_ctrl(struct goodix_ts_data *ts, bool on)
{
	struct ts_feature_info *info = &g_ts_data.feature_info;
	struct goodix_ts_config *config = NULL;
	int ret = 0;

	GTP_DEBUG("Noise ctrl:%d", on);
	if (info->holster_info.holster_switch) {
		/* reserve */
	/*} else if (info->glove_info.glove_switch) {
		if (on)
			config = &ts->glove_noise_config;
		else
			config = &ts->glove_config;*/
	} else {
		if (on)
			config = &ts->normal_noise_config;
		else
			config = &ts->normal_config;
	}

	if (config) {
		ret = goodix_send_cfg(config);
		if (ret < 0)
			return ret;
	}

#if 0
	if (info->charger_info.charger_switch) {
		ret = goodix_send_cmd(GTP_CMD_CHARGER_ON, 0x00);
		GTP_INFO("Charger switch on");
	}
#endif

	return ret;
}

#ifdef ROI
static int goodix_ts_roi_init(struct goodix_ts_roi *roi)
{
	unsigned int roi_bytes;

	if (!roi)
		return -EINVAL;

	if (!roi->roi_rows || !roi->roi_cols) {
		GTP_ERROR("Invalid roi config,rows:%d,cols:%d",
				roi->roi_rows, roi->roi_cols);
		return -EINVAL;
	}

	mutex_init(&roi->mutex);

	roi_bytes = (roi->roi_rows * roi->roi_cols + 1) * 2;
	roi->rawdata = kmalloc(roi_bytes + ROI_HEAD_LEN, GFP_KERNEL);
	if (!roi->rawdata) {
		GTP_ERROR("Failed to alloc memory for roi");
		return -ENOMEM;
	}

	GTP_INFO("ROI init,rows:%d,cols:%d",
				roi->roi_rows, roi->roi_cols);

	return 0;
}

static int goodix_cache_roidata(struct goodix_ts_roi *roi)
{
	unsigned roi_bytes;
	unsigned char status[ROI_HEAD_LEN];
	u16 checksum = 0;
	int i, ret;
	
	if (unlikely(!roi || !roi->enabled))
		return -EINVAL;

	ret = goodix_i2c_read(ROI_STA_REG, status, ROI_HEAD_LEN);
	if (unlikely(ret < 0))
		return ret;

	for (i = 0; i < ROI_HEAD_LEN; i++)
		checksum += status[i];

	if (unlikely((u8)checksum != 0)) { /* cast to 8bit checksum,*/
		GTP_ERROR("roi status checksum error{%02x %02x %02x %02x}",
				status[0], status[1], status[2], status[3]);
		return -1;
	}

	if (likely(status[0] & ROI_READY_MASK)) /* roi data ready */
		roi->track_id = status[0] & ROI_TRACKID_MASK;
	else
		return -1; /* not ready */

	mutex_lock(&roi->mutex);
	roi->data_ready = false;
	roi_bytes = (roi->roi_rows * roi->roi_cols + 1) * 2;

	ret = goodix_i2c_read(ROI_DATA_REG,
			(u8 *)(roi->rawdata) + ROI_HEAD_LEN, roi_bytes);
	if (unlikely(ret < 0)) {
		mutex_unlock(&roi->mutex);
		return ret;
	}

	for (i = 0, checksum = 0; i < roi_bytes / 2; i++) /* 16bits */
		checksum += roi->rawdata[i + ROI_HEAD_LEN / 2];
	memcpy(&roi->rawdata[0], &status[0], ROI_HEAD_LEN);
	
	if (unlikely(checksum != 0))
		GTP_ERROR("roi data checksum error");
	else
		roi->data_ready = true;

	mutex_unlock(&roi->mutex);

	status[0] = 0x00;
	ret = goodix_i2c_write(ROI_STA_REG, status, 1);

	return ret;
}
#endif

/**
 * goodix_request_event_handler - firmware request 
 * Return    <0: failed, 0: succeed
 */
static int goodix_request_event_handler(struct goodix_ts_data *ts)
{
	u8 rqst_data = 0;
	int ret;

	ret = goodix_i2c_read(GTP_REG_RQST, &rqst_data, 1);
	if (ret)
		return ret;

	GTP_DEBUG("Request state:0x%02x", rqst_data);
	switch (rqst_data & 0x0F) {
	case GTP_RQST_CONFIG:
		GTP_INFO("Request Config.");
		ret = goodix_send_cfg(&ts->normal_config);
		if (ret) {
			GTP_ERROR("Send config error");
		} else {
			GTP_INFO("Send config success");
			rqst_data = GTP_RQST_RESPONDED;
			goodix_i2c_write(GTP_REG_RQST, &rqst_data, 1);
		}
		break;
	case GTP_RQST_RESET:
		GTP_INFO("Request Reset.");
		goodix_i2c_read(0x5097, &rqst_data, 1);
		GTP_INFO("Reason code[0x5097]:%02x", rqst_data);
		goodix_chip_reset();
		msleep(40);
		goodix_feature_resume(ts);
		rqst_data = GTP_RQST_RESPONDED;
		goodix_i2c_write(GTP_REG_RQST, &rqst_data, 1);
		break;
	case GTP_RQST_NOISE_CFG:
		GTP_INFO("Request noise config");
		ret = goodix_noise_ctrl(ts, true);
		if (!ret)
			ts->noise_env = true;
		rqst_data = GTP_RQST_IDLE;
		goodix_i2c_write(GTP_REG_RQST, &rqst_data, 1);
		break;
	case GTP_RQST_NORMA_CFG:
		GTP_INFO("Request normal config");
		ret = goodix_noise_ctrl(ts, false);
		if (!ret)
			ts->noise_env = false;
		rqst_data = GTP_RQST_IDLE;
		goodix_i2c_write(GTP_REG_RQST, &rqst_data, 1);
		break;
	default:
		break;
	}
	return 0;
}

/**
 * gt1x_touch_evt_handler - handle touch event 
 * (pen event, key event, finger touch envent)
 * Return    <0: failed, 0: succeed
 */
static int goodix_touch_evt_handler(struct goodix_ts_data  *ts,
				struct ts_fingers *info)
{
	u8 touch_data[1 + 8 * GTP_MAX_TOUCH + 2] = {0};
	static u16 pre_index = 0;
	u16 cur_index = 0;
	u8 touch_num;
	u8 *coor_data = NULL;
	u8 check_sum = 0;
	int id, x, y, w, i;
	int ret = -1;
	static bool last_slide_flag;
	static bool last_finger_flag;
	u8 slide_num = 0;

	ret = goodix_i2c_read(GTP_READ_COOR_ADDR, &touch_data[0], 1 + 8 + 2);
	if (unlikely(ret))
		goto exit;

	if (unlikely(!touch_data[0])) {
		/* hw request */
		goodix_request_event_handler(ts);
		ret = 1;
		goto exit;
	}

	touch_num = touch_data[0] & 0x0f;
	if (unlikely(touch_num > GTP_MAX_TOUCH)) {
		GTP_ERROR("Illegal finger number!");
		goto exit;
	}

	/* read the remaining coor data 
		* 0x814E(touch status) + 
		* 8(every coordinate consist of 8 bytes data) * touch num + 
		* keycode + checksum
		*/
	if (touch_num > 1) {
		ret = goodix_i2c_read((GTP_READ_COOR_ADDR + 11),
					&touch_data[11], (touch_num - 1) * 8);
		if (ret)
			goto exit;
	}

	/* calc checksum */
	for (i = 0; i < 1 + 8 * touch_num + 2; i++)
		check_sum += touch_data[i];

	if (unlikely(check_sum)) { /* checksum error*/
		ret = goodix_i2c_read(GTP_READ_COOR_ADDR, touch_data,
					3 + 8 * touch_num);
		if (ret)
			goto exit;

		for (i = 0, check_sum = 0; i < 3 + 8 * touch_num; i++)
			check_sum += touch_data[i];

		if (check_sum) {
			GTP_ERROR("Checksum error[%x]",check_sum);
			ret = -EINVAL;
			goto exit;
		}
	}

#if 0
	key_value = touch_data[1 + 8 * touch_num];
	/*  start check current event */
	if ((touch_data[0] & 0x10) && (key_value & 0x0F)) {
		//for (i = 0; i < GTP_MAX_KEY_NUM; i++) {
//			input_report_key(dev, gt1x_touch_key_array[i],
//					key_value & (0x01 << i));
		// key
	}
#endif

	memset(info, 0x00, sizeof(struct ts_fingers));
	coor_data = &touch_data[1];

	for (i = 0; i < touch_num; i++) {
		id = coor_data[i * 8] & 0x7f;
		x = le16_to_cpup((__le16 *)&coor_data[i * 8 + 1]);
		y = le16_to_cpup((__le16 *)&coor_data[i * 8 + 3]);
		w = le16_to_cpup((__le16 *)&coor_data[i * 8 + 5]);

		if((y>ts->max_y) && (y < ts->max_y + ts->slide_max_y)){
			slide_num++;
			info->slide_flag = true;
			last_slide_flag = info->slide_flag;

			info->slide_fingers[id].x = x;
			info->slide_fingers[id].y = y - ts->max_y;
			info->slide_fingers[id].major = w;
			info->slide_fingers[id].minor = w;
			info->slide_fingers[id].pressure = w;
			info->slide_fingers[id].status = TP_FINGER;
		}else{
			info->finger_flag = true;
			last_finger_flag = info->finger_flag;
			if (unlikely(ts->flip_x))
				info->fingers[id].x = ts->max_x - x;
			else
				info->fingers[id].x = x;

			if (unlikely(ts->flip_y))
				info->fingers[id].y = ts->max_y - y;
			else
				info->fingers[id].y = y;

			info->fingers[id].major = w;
			info->fingers[id].minor = w;
			info->fingers[id].pressure = w;
			info->fingers[id].status = TP_FINGER;
			cur_index |= 1 << id;
		}
		GTP_DEBUG("[%d](%d, %d, %d)", id, x, y, w);
	}

	if(last_slide_flag && !info->slide_flag){
		last_slide_flag = info->slide_flag;
		info->slide_flag = true;
	}
	if(last_finger_flag && !info->finger_flag){
		last_finger_flag = info->finger_flag;
		info->finger_flag = true;
	}

	info->cur_finger_number = touch_num - slide_num;
	info->slide_cur_finger_number = slide_num;

#ifdef ROI
	if (pre_index != cur_index && (cur_index & pre_index) != cur_index)
		goodix_cache_roidata(&ts->roi);
#endif

	pre_index = cur_index;
exit:
	return ret;
}

#if 0
static int goodix_gesture_evt_handler(struct goodix_ts_data *ts,
				struct ts_fingers *info)
{
	u8 doze_buf[4] = {0}, ges_type;
	int len;
	int ret = 0;

	/** package: -head 4B + track points + extra info- 
		* - head -
		*  doze_buf[0]: gesture type, 
		*  doze_buf[1]: number of gesture points ,
		*  doze_buf[2]: protocol type, 
		*  doze_buf[3]: gesture extra data length.
		*/
	ret = goodix_i2c_read(GTP_REG_WAKEUP_GESTURE, doze_buf, 4);
	if (ret < 0)
		return 0;

	ges_type = doze_buf[0];
	len = doze_buf[1];
/*	need_chk = doze_buf[2] & 0x80;
	extra_len = doze_buf[3];
*/

	GTP_DEBUG("0x%x = 0x%02X,0x%02X,0x%02X,0x%02X", GTP_REG_WAKEUP_GESTURE,
		doze_buf[0], doze_buf[1], doze_buf[2], doze_buf[3]);

	/* check gesture type (if available?) */
	if (ges_type == 0) {
		GTP_INFO("Invalid gesture");
		doze_buf[0] = 0x00;
		//goodix_i2c_write(GTP_REG_WAKEUP_GESTURE, doze_buf, 1);
		//gesture_enter_doze();
		info->gesture_wakeup_value = TS_GESTURE_INVALID;
		return 0;
	}

	switch (ges_type) {
	case 0xAA:
		break;
	default:
		break;
	}

	info->gesture_wakeup_value = ges_type;
	ret = 1;

	doze_buf[0] = 0; // clear ges flag
	goodix_i2c_write(GTP_REG_WAKEUP_GESTURE, doze_buf, 1);
	return ret;
}
#endif

/**
 * goodix_irq_bottom_half - Goodix touchscreen work function.
 */
static int goodix_irq_bottom_half(struct ts_cmd_node *in_cmd,
				struct ts_cmd_node *out_cmd)
{
	struct goodix_ts_data *ts = goodix_ts;
	struct ts_fingers *ts_fingers;
	u8 sync_val = 0;
	int ret = 0;

	if (unlikely(!goodix_ts))
		return -ENODEV;

	ts_fingers = &out_cmd->cmd_param.pub_params.algo_param.info;
	out_cmd->command = TS_INVAILD_CMD;
	out_cmd->cmd_param.pub_params.algo_param.algo_order =
			ts->dev_data->algo_id;
	// debug+
	//GTP_DEBUG("Algo-order: %d", ts->dev_data->algo_id);

#if 0
	ret = goodix_gesture_evt_handler(ts, ts_fingers);
	if (ret == 1)
		goto sync_evt;
#endif

	/* handle touch event 
	 * return: <0 - error, 0 - touch event handled,
	 * 			1 - hw request event handledv */
	ret = goodix_touch_evt_handler(ts, ts_fingers);
	if (ret == 0)
		out_cmd->command = TS_INPUT_ALGO;

/*sync_evt:*/
	if (!ts->rawdiff_mode)
		ret = goodix_i2c_write(GTP_READ_COOR_ADDR, &sync_val, 1);
	else
		GTP_DEBUG("Firmware rawdiff mode");

	return 0;
}

static int goodix_i2c_test(void)
{
	u32 hw_info;
	int ret;

	ret = goodix_i2c_read(GTP_REG_HW_INFO, (u8 *)&hw_info,
					sizeof(hw_info));

	GTP_INFO("Hardware Info:%08X", hw_info);
	return ret;
}

static int goodix_request_gpio(struct ts_device_data *dev_data)
{
	int reset_gpio = dev_data->reset_gpio;
	int vdd_en_gpio = dev_data->vdd_en_gpio;
	int irq_gpio = dev_data->irq_gpio;
	int ret = 0;

	if (!gpio_is_valid(irq_gpio) || !gpio_is_valid(reset_gpio) || !gpio_is_valid(vdd_en_gpio)) {
		GTP_ERROR("Invalid gpios");
		return -EINVAL;
	}

	ret = gpio_request(irq_gpio, "GTP-INT-GPIO");
	if (ret < 0) {
		GTP_ERROR("Failed to request GPIO:%d, errno:%d", irq_gpio, ret);
		return -ENODEV;
	}
	gpio_direction_input(irq_gpio);

	ret = gpio_request(reset_gpio, "GTP-RST-GPIO");
	if (ret < 0) {
		GTP_ERROR("Failed to request GPIO:%d, errno:%d", reset_gpio, ret);
		gpio_free(irq_gpio);
		return -ENODEV;
	}

	ret = gpio_request(vdd_en_gpio, "GTP-VDD-EN-GPIO");
	if (ret < 0) {
		GTP_ERROR("Failed to request GPIO:%d, errno:%d",vdd_en_gpio , ret);
		gpio_free(irq_gpio);
		return -ENODEV;
	}

	GTP_INFO("Request gpio: OK");
	return 0;
}

static void goodix_release_gpio(struct ts_device_data *dev_data)
{
	int reset_gpio = dev_data->reset_gpio;
	int irq_gpio = dev_data->irq_gpio;

	if (gpio_is_valid(reset_gpio))
		gpio_free(reset_gpio);

	if (gpio_is_valid(irq_gpio))
		gpio_free(irq_gpio);
}

static int goodix_get_regulators(struct goodix_ts_data *ts)
{
	int ret;

	ts->vdd_ana = regulator_get(&ts->pdev->dev, "goodix-vdd");
	if (IS_ERR(ts->vdd_ana)) {
		ret = PTR_ERR(ts->vdd_ana);
		GTP_ERROR("Regulator get of vdd_ana failed:%d", ret);
		ts->vdd_ana = NULL;
		return ret;
	}

	ts->vcc_i2c = regulator_get(&ts->pdev->dev, "goodix-io");
	if (IS_ERR(ts->vcc_i2c)) {
		ret = PTR_ERR(ts->vcc_i2c);
		GTP_ERROR("Regulator get of vcc_i2c failed:%d", ret);
		ts->vcc_i2c = NULL;
		goto err_get_vcc;
	}

#if 0
	ret = regulator_set_voltage(ts->vdd_ana,
			ts->dev_data->regulator_ctrl.vci_value,
			ts->dev_data->regulator_ctrl.vci_value);
	if (ret < 0) {
		GTP_ERROR("Failed to set vdd voltage:%d", ret);
		goto err_set_vol;
	}

	if (ts->dev_data->regulator_ctrl.need_set_vddio_value) {
		ret = regulator_set_voltage(ts->vcc_i2c,
				ts->dev_data->regulator_ctrl.vddio_value,
				ts->dev_data->regulator_ctrl.vddio_value);
		if (ret < 0) {
			GTP_ERROR("Failed to set vddio voltage:%d", ret);
			goto err_set_vol;
		}
	}
#endif

	GTP_INFO("Regulator get: OK");
	return 0;

//err_set_vol:
	regulator_put(ts->vcc_i2c);
	ts->vcc_i2c = NULL;
err_get_vcc:
	regulator_put(ts->vdd_ana);
	ts->vdd_ana = NULL;
	return ret;
}

static void goodix_put_regulators(struct goodix_ts_data *ts)
{
	if (ts->vdd_ana) {
		regulator_put(ts->vdd_ana);
		ts->vdd_ana = NULL;
	}

	if (ts->vcc_i2c) {
		regulator_put(ts->vcc_i2c);
		ts->vcc_i2c = NULL;
	}
}

/**
 * gt1x_power_switch - power switch .
 * @on: 1-switch on, 0-switch off.
 * return: 0-succeed, -1-faileds
 */
int goodix_power_switch(struct goodix_ts_data *ts, int on)
{
	int ret = 0, reset_gpio;
	int vdd_en_gpio;
#if 0
	if (!ts || !ts->vdd_ana || !ts->vcc_i2c)
		return -EINVAL;

	reset_gpio = ts->dev_data->reset_gpio;
	gpio_direction_output(reset_gpio, 0);
	udelay(100);

	if (on) {
		GTP_INFO("GTP power ON");
		ret |= regulator_enable(ts->vdd_ana);
		udelay(2);
		GTP_INFO("%s:%d ret=%d ",__func__,__LINE__,ret);
		ret |= regulator_enable(ts->vcc_i2c);
		usleep_range(10000, 10010);
		GTP_INFO("%s:%d ret=%d ",__func__,__LINE__,ret);
	} else {
		GTP_INFO("GTP power OFF");
		ret |= regulator_disable(ts->vcc_i2c);
		udelay(2);
		ret |= regulator_disable(ts->vdd_ana);
		udelay(2);
	}
#endif
	reset_gpio = ts->dev_data->reset_gpio;
	gpio_direction_output(reset_gpio, 0);
	udelay(100);

	if (on) {
		GTP_INFO("GTP power ON");
		ret |= regulator_enable(ts->vdd_ana);
		udelay(2);
		gpio_direction_output(vdd_en_gpio,1);
	} else {
		GTP_INFO("GTP power OFF");
		ret |= regulator_disable(ts->vdd_ana);
		udelay(2);
		gpio_direction_output(vdd_en_gpio,0);
	}

	GTP_INFO("%s:%d ret=%d ",__func__,__LINE__,ret);
	return ret;
}

/**
 * goodix_pinctrl_init - pinctrl init
 */
static int goodix_pinctrl_init(struct goodix_ts_data *ts)
{
	int ret = 0;

	ts->pinctrl = devm_pinctrl_get(&ts->pdev->dev);
	if (IS_ERR_OR_NULL(ts->pinctrl)) {
		GTP_ERROR("Failed to get pinctrl");
		ret = PTR_ERR(ts->pinctrl);
		return ret;
	}

	ts->pins_default = pinctrl_lookup_state(ts->pinctrl, "default");
	if (IS_ERR_OR_NULL(ts->pins_default)) {
		GTP_ERROR("Pin state[default] not found");
		ret = PTR_ERR(ts->pins_default);
		goto exit_put;
	}

	ts->pins_suspend = pinctrl_lookup_state(ts->pinctrl, "idle");
	if (IS_ERR_OR_NULL(ts->pins_suspend)) {
		GTP_ERROR("Pin state[suspend] not found");
		ret = PTR_ERR(ts->pins_suspend);
		goto exit_put;
	}

	ts->pins_gesture = pinctrl_lookup_state(ts->pinctrl, "gesture");
	if (IS_ERR_OR_NULL(ts->pins_gesture)) {
		GTP_ERROR("Pin state[gesture] not found");
		ret = PTR_ERR(ts->pins_gesture);
	}

	return 0;
exit_put:
	devm_pinctrl_put(ts->pinctrl);
	ts->pinctrl = NULL;
	ts->pins_gesture = NULL;
	ts->pins_suspend = NULL;
	ts->pins_default = NULL;
	return ret;
}

static void goodix_pinctrl_release(struct goodix_ts_data *ts)
{
	if (ts->pinctrl)
		devm_pinctrl_put(ts->pinctrl);
	ts->pinctrl = NULL;
	ts->pins_gesture = NULL;
	ts->pins_suspend = NULL;
	ts->pins_default = NULL;
}

/**
 * goodix_pinctrl_select_normal - set normal pin state
 *  Irq pin *must* be set to *pull-up* state.
 */
static int goodix_pinctrl_select_normal(struct goodix_ts_data *ts)
{
	int ret = 0;

	if (ts->pinctrl && ts->pins_default) {
		ret = pinctrl_select_state(ts->pinctrl, ts->pins_default);
		if (ret < 0)
			GTP_ERROR("Set normal pin state error:%d", ret);
	}

	return ret;
}

/**
 * goodix_pinctrl_select_suspend - set suspend pin state
 *  Irq pin *must* be set to *pull-up* state.
 */
static int goodix_pinctrl_select_suspend(struct goodix_ts_data *ts)
{
	int ret = 0;

	if (ts->pinctrl && ts->pins_suspend) {
		ret = pinctrl_select_state(ts->pinctrl, ts->pins_suspend);
		if (ret < 0)
			GTP_ERROR("Set suspend pin state error:%d", ret);
	}

	return ret;
}

/**
 * goodix_pinctrl_select_gesture - set gesture pin state
 *  Irq pin *must* be set to *pull-up* state.
 */
static int goodix_pinctrl_select_gesture(struct goodix_ts_data *ts)
{
	int ret = 0;

	if (ts->pinctrl && ts->pins_gesture) {
		ret = pinctrl_select_state(ts->pinctrl, ts->pins_gesture);
		if (ret < 0)
			GTP_ERROR("Set gesture pin state error:%d", ret);
	}
	
	return ret;
}

/**
 * goodix_read_version - Read gt1x version info.
 * @hw_info: address to store version info
 * Return 0-succeed.
 */
int goodix_read_version(struct goodix_hw_info * hw_info)
{
	u8 buf[12] = { 0 };
	u32 mask_id, patch_id;
	u8 product_id[5] = {0};
	u8 sensor_id, match_opt;
	int i, retry = 3;
	u8 checksum = 0;
	int ret = -1;

	while (retry--) {
		ret = goodix_i2c_read(GTP_REG_VERSION, buf, sizeof(buf));
		if (!ret) {
			for (i = 0, checksum = 0; i < sizeof(buf); i++)
				checksum += buf[i];

			if (checksum == 0 &&/* first 3 bytes must be number or char */
				IS_NUM_OR_CHAR(buf[0]) && IS_NUM_OR_CHAR(buf[1])
				&& IS_NUM_OR_CHAR(buf[2]) && buf[10] != 0xFF) {
				break;
			} else if (checksum == (u8)(buf[11] * 2) && buf[10] != 0xFF) {
				/* checksum calculated by boot code */
				break;
			} else {
				GTP_ERROR("Invalid version info:%c%c%c", buf[0], buf[1], buf[2]);
			}
		}

		GTP_DEBUG("Read version failed,retry: %d", retry);
		msleep(100);
	}

	if (retry <= 0)
		return -ENODEV;

	mask_id = (u32) ((buf[7] << 16) | (buf[8] << 8) | buf[9]);
	patch_id = (u32) ((buf[4] << 16) | (buf[5] << 8) | buf[6]);
	memcpy(product_id, buf, 4);
	sensor_id = 0;//buf[10] & 0x0F;
	match_opt = (buf[10] >> 4) & 0x0F;

	GTP_INFO("IC Version:GT%s_%06X(FW)_%04X(Boot)_%02X(SensorID)",
		product_id, patch_id, mask_id >> 8, sensor_id);

	if (hw_info != NULL) {
		hw_info->mask_id = mask_id;
		hw_info->patch_id = patch_id;
		memcpy(hw_info->product_id, product_id, 5);
		hw_info->sensor_id = sensor_id;
		hw_info->match_opt = match_opt;
	}

	goodix_ts->sensor_id_valid = true;
	return 0;
}

static int goodix_parse_dts(struct goodix_ts_data *ts)
{
	struct device_node *device = ts->pdev->dev.of_node;
	int ret = 0;

	ret = of_property_read_u32(device, "x_max_mt", &ts->max_x);
	if (ret) {
		GTP_ERROR("Get x_max_mt failed");
		ret = -EINVAL;
		goto err;
	}

	ret = of_property_read_u32(device, "y_max_mt", &ts->max_y);
	if (ret) {
		GTP_ERROR("Get y_max_mt failed");
		ret = -EINVAL;
		goto err;
	}

#ifdef ROI
	ret = of_property_read_u32_index(device, "roi_data_size", 0,
			&ts->roi.roi_rows);
	if (ret) {
		GTP_ERROR("Get ROI rows failed");
		ret = -EINVAL;
		goto err;
	}

	ret = of_property_read_u32_index(device, "roi_data_size", 1,
			&ts->roi.roi_cols);
	if (ret) {
		GTP_ERROR("Get ROI cols failed");
		ret = -EINVAL;
		goto err;
	}
#endif
	ret = of_property_read_bool(device, "tools_support");
	if (!ret) {
		ts->tools_support = true;
		GTP_INFO("Tools support enabled");
	}

err:
	return ret;
}

static int goodix_parse_specific_dts(struct goodix_ts_data *ts)
{
	struct device_node *device;
	char project_id[20] = {0};
	u32 value;
	int ret = 0;

	sprintf(project_id, "goodix-sensorid-%u", ts->hw_info.sensor_id);
	GTP_INFO("Parse specific dts:%s", project_id);
	device = of_find_compatible_node(ts->pdev->dev.of_node, NULL, project_id);
	if (!device) {
		GTP_INFO("No chip specific dts:%s, need to prase", project_id);
		return -EINVAL;
	}

	ret = of_property_read_u32(device, "x_max_mt", &value);
	if (!ret) 
		ts->max_x = value;
	
	ret =  of_property_read_u32(device, "y_max_mt", &value);
	if (!ret)
		ts->max_y = value;

	return 0;
}

/**
 * goodix_parse_dt_cfg - parse config data from devices tree.
 * @dev: device that this driver attached.
 * @cfg: pointer of the config array.
 * @cfg_len: pointer of the config length.
 * @sid: sensor id.
 * Return: 0-succeed, -1-faileds
 */
int goodix_parse_cfg_data(struct goodix_ts_data *ts,
				char *cfg_type, u8 *cfg, int *cfg_len, u8 sid)
{
	struct device_node *self = ts->pdev->dev.of_node;
	struct property *prop;
	char project_id[20];
	int correct_len;

	sprintf(project_id,  "goodix-sensorid-%u", ts->hw_info.sensor_id);
	self = of_find_compatible_node(ts->pdev->dev.of_node,
				NULL, project_id);
	if (!self) {
		GTP_ERROR("No chip specific dts:%s, need to parse", project_id);
		return -EINVAL;
	}

	GTP_INFO("Parse [%s] data from dts[SENSORID%u]", cfg_type, sid);
	prop = of_find_property(self, cfg_type, cfg_len);
	if (!prop || !prop->value || *cfg_len == 0)
		return -EINVAL;/* fail */

	memcpy(cfg, prop->value, *cfg_len);
	if (*cfg_len > GTP_CONFIG_ORG_LENGTH &&
					cfg[EXTERN_CFG_OFFSET] & 0x40)
		/* extends config */
		correct_len = GTP_CONFIG_ORG_LENGTH + GTP_CONFIG_EXT_LENGTH;
	else
		correct_len = GTP_CONFIG_ORG_LENGTH;

	if (*cfg_len != correct_len) {
		GTP_ERROR("Invalid config size:%d", *cfg_len);
		return -EINVAL;
	}

	return 0;
}

/**
 * goodix_init_panel - Prepare config data for touch ic,\
 * don't call this function after initialization.
 *
 * Return 0--success,<0 --fail.
 */
static int goodix_init_configs(struct goodix_ts_data *ts)
{
	u8 sensor_id, *cfg_data;
	int cfg_len = 0;
	int ret = 0;

	sensor_id = ts->hw_info.sensor_id;
	if (sensor_id > 5) {
		GTP_ERROR("Invalid sensor ID");
		return -EINVAL;
	}

	/* max config data length */
	cfg_len = sizeof(ts->normal_config.data);
	cfg_data = kzalloc(cfg_len, GFP_KERNEL);
	if (!cfg_data)
		return -ENOMEM;

	/* parse normal config data */
	ret = goodix_parse_cfg_data(ts, "normal_config", cfg_data,
				&cfg_len, sensor_id);
	if (ret < 0) {
		GTP_ERROR("Failed to parse normal_config data:%d", ret);
		goto exit_kfree;
	}

	//cfg_data[0] &= 0x7F; /* mask config version */
	GTP_INFO("Normal config version:%d,size:%d", cfg_data[0], cfg_len);
	memcpy(&ts->normal_config.data[0], cfg_data, cfg_len);
	ts->normal_config.size = cfg_len;
	ts->normal_config.delay_ms = 200;
	ts->normal_config.name = "normal_config";
	ts->normal_config.initialized = true;

	/* parse normal noise config data*/
	ret = goodix_parse_cfg_data(ts, "normal_noise_config",
			cfg_data, &cfg_len, sensor_id);
	if (ret < 0) {
		GTP_ERROR("Failed to parse normal_noise_config data:%d", ret);
		ts->normal_noise_config.initialized = false;
	} else if (cfg_len == ts->normal_config.size) {
		cfg_data[0] &= 0x7f;
		GTP_INFO("Normal noise config version:%d,size:%d", cfg_data[0], cfg_len);
		memcpy(&ts->normal_noise_config.data[0], cfg_data, cfg_len);
		ts->normal_noise_config.size = cfg_len;
		ts->normal_noise_config.delay_ms = 100;
		ts->normal_noise_config.name = "normal_noise_config";
		ts->normal_noise_config.initialized = true;
	}

	/* parse glove config data */
	ret = goodix_parse_cfg_data(ts, "glove_config", cfg_data,
				&cfg_len, sensor_id);
	if (ret < 0) {
		GTP_ERROR("Failed to parse glove_config data:%d", ret);
		ts->glove_config.initialized = false;
		ret = 0;
	} else if (cfg_len == ts->normal_config.size) {
		cfg_data[0] &= 0x7F; /* mask config version */
		GTP_INFO("Glove config version:%d,size:%d", cfg_data[0], cfg_len);
		memcpy(&ts->glove_config.data[0], cfg_data, cfg_len);
		ts->glove_config.size = cfg_len;
		ts->glove_config.delay_ms = 20;
		ts->glove_config.name = "glove_config";
		ts->glove_config.initialized = true;
	} else {
		ts->glove_config.initialized = false;
	}

	/* parse glove noise config data*/
	ret = goodix_parse_cfg_data(ts, "glove__noise_config",
			cfg_data, &cfg_len, sensor_id);
	if (ret < 0) {
		GTP_ERROR("Failed to parse glove__noise_config data:%d", ret);
		ts->glove_noise_config.initialized = false;
	} else if (cfg_len == ts->normal_config.size) {
		cfg_data[0] &= 0x7f;
		GTP_INFO("Normal noise config version:%d,size:%d", cfg_data[0], cfg_len);
		memcpy(&ts->glove_noise_config.data[0], cfg_data, cfg_len);
		ts->glove_noise_config.size = cfg_len;
		ts->glove_noise_config.delay_ms = 100;
		ts->glove_noise_config.name = "normal_noise_config";
		ts->glove_noise_config.initialized = true;
	}

	/* parse holster config data */
	ret = goodix_parse_cfg_data(ts, "holster_config", cfg_data,
				&cfg_len, sensor_id);
	if (ret < 0) {
		GTP_ERROR("Failed to parse holster_config data:%d", ret);
		ts->holster_config.initialized = false;
		ret = 0;
	} else if (cfg_len == ts->normal_config.size) {
		cfg_data[0] &= 0x7F; /* mask config version */
		GTP_INFO("Holster config version:%d,size:%d", cfg_data[0], cfg_len);
		memcpy(&ts->holster_config.data[0], cfg_data, cfg_len);
		ts->holster_config.size = cfg_len;
		ts->holster_config.delay_ms = 20;
		ts->holster_config.name = "holster_config";
		ts->holster_config.initialized = true;
	} else {
		ts->holster_config.initialized = false;
	}

	/* parse charger config data*/
	ret = goodix_parse_cfg_data(ts, "charger_config",
			cfg_data, &cfg_len, sensor_id);
	if (ret < 0) {
		GTP_ERROR("Failed to parse charger_config data:%d", ret);
		ts->charger_config.initialized = false;
	} else if (cfg_len == ts->normal_config.size) {
		cfg_data[0] &= 0x7f;
		GTP_INFO("Charger config version:%d,size:%d", cfg_data[0], cfg_len);
		memcpy(&ts->charger_config.data[0], cfg_data, cfg_len);
		ts->charger_config.size = cfg_len;
		ts->charger_config.delay_ms = 100;
		ts->charger_config.name = "charger_config";
		ts->charger_config.initialized = true;
	}

	/* parse pocket config data*/
	ret = goodix_parse_cfg_data(ts, "pocket_config",
			cfg_data, &cfg_len, sensor_id);
	if (ret < 0) {
		GTP_ERROR("Failed to parse pocket_config data:%d", ret);
		ts->pocket_config.initialized = false;
	} else if (cfg_len == ts->normal_config.size) {
		cfg_data[0] &= 0x7f;
		GTP_INFO("Pocket config version:%d,size:%d", cfg_data[0], cfg_len);
		memcpy(&ts->pocket_config.data[0], cfg_data, cfg_len);
		ts->pocket_config.size = cfg_len;
		ts->pocket_config.delay_ms = 100;
		ts->pocket_config.name = "pocket_config";
		ts->pocket_config.initialized = true;
	}

exit_kfree:
	kfree(cfg_data);
	return ret;
}

/**
 * Hisi Platform Touchscreen Interface
 */
static int goodix_chip_parse_config(struct device_node *device,
				struct ts_device_data *chip_data)
{
	int ret = 0, gpio;
	u32 value;

	GTP_INFO("goodix parse config");
	if (!device || !chip_data)
		return -ENODEV;

	gpio = of_get_named_gpio(device, "irq_gpio", 0);
	if (!gpio_is_valid(gpio)) {
		GTP_ERROR("Invalid irq-gpio:%d", gpio);
		ret = -EINVAL;
		goto err;
	}
	chip_data->irq_gpio = gpio;

	gpio = of_get_named_gpio(device, "reset_gpio", 0);
	if (!gpio_is_valid(gpio)) {
		GTP_ERROR("Invalid irq-gpio:%d", gpio);
		ret = -EINVAL;
		goto err;
	}
	chip_data->reset_gpio = gpio;

	gpio = of_get_named_gpio(device, "vdd_en_gpio", 0);
	if (!gpio_is_valid(gpio)) {
		GTP_ERROR("Invalid vdd_en-gpio:%d", gpio);
		ret = -EINVAL;
		goto err;
	}
	chip_data->vdd_en_gpio = gpio;


	ret = of_property_read_u32(device, "irq_config",
						&chip_data->irq_config);
	if (ret) {
		GTP_ERROR("Get irq config failed");
		ret = -EINVAL;
		goto err;
	}

	ret = of_property_read_u32(device, "algo_id",
						&chip_data->algo_id);
	if (ret) {
		GTP_ERROR("Get algo id failed");
		ret = -EINVAL;
		goto err;
	}

	ret = of_property_read_u32(device, "slide_max_x", &chip_data->slide_max_x);
	if (ret) {
		GTP_ERROR("get device slide_max_x failed\n");
		//ret = -EINVAL;
	}
	ret = of_property_read_u32(device, "slide_max_y", &chip_data->slide_max_y);
	if (ret) {
		GTP_ERROR("get device slide_max_y failed\n");
		//ret = -EINVAL;
	}

#if 0
	ret = of_property_read_u32(device, "vci_value",
			&chip_data->regulator_ctrl.vci_value);
	if (ret < 0) {
		chip_data->regulator_ctrl.vci_value = 3100000;
		GTP_INFO("Use default vdd voltage: 3.1v");
	}

	ret = of_property_read_u32(device, "need_set_vddio_value",
			&chip_data->regulator_ctrl.need_set_vddio_value);
	if (ret) {
		GTP_INFO("Not defined need_set_vddio_value");
		chip_data->regulator_ctrl.need_set_vddio_value = 0;
	} else {
		ret = of_property_read_u32(device, "vddio_value", 
		&chip_data->regulator_ctrl.vddio_value);
		if (ret) {
			GTP_INFO("Not defined vddio value in dts, use default value");
			chip_data->regulator_ctrl.vddio_value = 1800000;
		}
	}
#endif

#if 0
	ret = of_property_read_string(device, "tp_test_type",
			&chip_data->tp_test_type); 
	if (ret) {
		GTP_INFO("tp_test_type not exist, use default value");
		strncpy(chip_data->tp_test_type, "Normalize_type:judge_different_result",
			TS_CAP_TEST_TYPE_LEN);
	}
#endif

#if 0
	ret = of_property_read_u32(device, "charger_supported", &value);
	if (!ret) {
		GTP_INFO("get chip specific charger_supported = %d",
			value);
		g_ts_data.feature_info.charger_info.charger_supported = (u8)value;
	}
#endif

	ret = of_property_read_u32(device, "roi_supported", &value);
	if (!ret) {
		GTP_INFO("get chip specific roi_supported = %d", value);
		g_ts_data.feature_info.roi_info.roi_supported = (u8)value;
	} else {
		GTP_INFO("Can not get roi_supported value");
		g_ts_data.feature_info.roi_info.roi_supported = 0;
	}

	ret = 0;
err:
	return ret;
}

static int goodix_chip_detect(struct device_node *device, 
	 struct ts_device_data *dev_data, struct platform_device *ts_dev)
{
	struct i2c_client *client;
	int ret = -1;

	if (!device || !dev_data || !ts_dev) {
		GTP_ERROR("device, dev_data or ts_dev is NULL");
		return -ENODEV;
	}

	GTP_INFO("Chip detect");

	goodix_ts = kzalloc(sizeof(struct goodix_ts_data), GFP_KERNEL);
	if (!goodix_ts)
		return -ENOMEM;

	client = dev_data->client;
	goodix_ts->pdev = ts_dev;
	goodix_ts->pdev->dev.of_node = device;
	goodix_ts->dev_data = dev_data;
	goodix_ts->ops.i2c_read = goodix_i2c_read;
	goodix_ts->ops.i2c_write = goodix_i2c_write;
	goodix_ts->ops.chip_reset = goodix_chip_reset;
	goodix_ts->ops.send_cmd = goodix_send_cmd;
	goodix_ts->ops.send_cfg = goodix_send_cfg;
	goodix_ts->ops.i2c_read_dbl_check = goodix_i2c_read_dbl_check;
	goodix_ts->ops.read_version = goodix_read_version;
	goodix_ts->ops.parse_cfg_data = goodix_parse_cfg_data;
	goodix_ts->ops.feature_resume = goodix_feature_resume;
	goodix_ts->tools_support = true;

	goodix_ts->slide_max_x = goodix_ts->dev_data->slide_max_x;
	goodix_ts->slide_max_y = goodix_ts->dev_data->slide_max_y;

	/* do NOT remove these logs */
	GTP_INFO("Driver Version: %s", GTP_DRIVER_VERSION);

	ret = goodix_get_regulators(goodix_ts);
	if (ret < 0)
		//goto err_get_regs;

	ret = goodix_request_gpio(dev_data);
	if (ret < 0)
		goto err_req_gpio;

	ret = goodix_pinctrl_init(goodix_ts);
	if (ret < 0)
		goto err_pinctrl_init;

	ret = goodix_pinctrl_select_normal(goodix_ts);
	if (ret < 0)
		goto err_pinctrl_init;

	/* power on */
	ret = goodix_power_switch(goodix_ts, SWITCH_ON);
	if (ret < 0)
		goto err_power_on;
	/* detect chip */
	ret = goodix_chip_reset();
	if (ret < 0)
		goto err_power_on;

	ret = goodix_i2c_test();
	if (ret < 0)
		goto err_power_on;

	return 0;

err_power_on:
	goodix_pinctrl_release(goodix_ts);
	goodix_power_switch(goodix_ts, SWITCH_OFF);
err_pinctrl_init:
	goodix_release_gpio(dev_data);
err_req_gpio:
	goodix_put_regulators(goodix_ts);
//err_get_regs:
	kfree(goodix_ts);
	goodix_ts = NULL;
	return ret;
}

static int goodix_chip_init(void)
{
	struct goodix_ts_data *ts = goodix_ts;
	int ret = -1;
	u8 reg_val[1];

	/* check main system firmware */
	ret = goodix_i2c_read_dbl_check(GTP_REG_FW_CHK_MAINSYS, reg_val, 1);
	if (!ret && reg_val[0] != 0xBE) {
		GTP_ERROR("Check main system not pass[0x%2X]", reg_val[0]);
	}

	/* check subsystem firmware */
	ret = goodix_i2c_read_dbl_check(GTP_REG_FW_CHK_SUBSYS, reg_val, 1);
	if (!ret && reg_val[0] == 0xAA) {
		GTP_ERROR("Check subsystem not pass[0x%2X]", reg_val[0]);
	}

	/* read version information. pid/vid/sensor id */
	ret = goodix_read_version(&ts->hw_info);
	if (ret < 0)
		return ret;

	/* obtain goodix dt properties */
	ret = goodix_parse_dts(ts);
	if (ret < 0)
		return ret;

	ret = goodix_parse_specific_dts(ts);
	if (ret < 0)
		return ret;

	if (ts->tools_support)
		gt1x_init_tool_node();

	/* init config data, normal/glove/hoslter config data */
	ret = goodix_init_configs(ts);
	if (ret < 0) {
		GTP_ERROR("Init panel failed");
		return ret;
	}

	ret = goodix_feature_resume(ts);
	if (ret < 0)
		return ret;

#ifdef ROI
	ret = goodix_ts_roi_init(&goodix_ts->roi);
	if (ret < 0)
		return ret;
#endif

	return 0;
}

static int goodix_input_config(struct input_dev *input_dev)
{
	struct goodix_ts_data *ts = goodix_ts;

	if (ts == NULL)
		return -ENODEV;

	set_bit(EV_SYN, input_dev->evbit);
	set_bit(EV_KEY, input_dev->evbit);
	set_bit(EV_ABS, input_dev->evbit);
	set_bit(BTN_TOUCH, input_dev->keybit);
	set_bit(BTN_TOOL_FINGER, input_dev->keybit);

	set_bit(TS_DOUBLE_CLICK, input_dev->keybit);
	set_bit(TS_SLIDE_L2R, input_dev->keybit);
	set_bit(TS_SLIDE_R2L, input_dev->keybit);
	set_bit(TS_SLIDE_T2B, input_dev->keybit);
	set_bit(TS_SLIDE_B2T, input_dev->keybit);
	set_bit(TS_CIRCLE_SLIDE, input_dev->keybit);
	set_bit(TS_LETTER_c, input_dev->keybit);
	set_bit(TS_LETTER_e, input_dev->keybit);
	set_bit(TS_LETTER_m, input_dev->keybit);
	set_bit(TS_LETTER_w, input_dev->keybit);
	set_bit(TS_PALM_COVERED, input_dev->keybit);
	set_bit(INPUT_PROP_DIRECT, input_dev->propbit);

#ifdef INPUT_TYPE_B_PROTOCOL
#if (LINUX_VERSION_CODE > KERNEL_VERSION(3, 7, 0))
	input_mt_init_slots(input_dev, GTP_MAX_TOUCH, INPUT_MT_DIRECT);
#else
	input_mt_init_slots(input_dev, GTP_MAX_TOUCH);
#endif
#endif
	input_set_abs_params(input_dev, ABS_MT_POSITION_X, 0, ts->max_x, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_Y, 0, ts->max_y, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_PRESSURE, 0, 255, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_TRACKING_ID, 0, GTP_MAX_TOUCH, 0, 0);

	return 0;
}

static int goodix_slide_config(struct input_dev *slide_dev)
{
	struct goodix_ts_data *ts = goodix_ts;

	if (ts == NULL)
		return -ENODEV;

	set_bit(EV_SYN, slide_dev->evbit);
	set_bit(EV_KEY, slide_dev->evbit);
	set_bit(EV_ABS, slide_dev->evbit);
	set_bit(BTN_TOUCH, slide_dev->keybit);
	set_bit(BTN_TOOL_FINGER, slide_dev->keybit);

#ifdef INPUT_PROP_DIRECT
	set_bit(INPUT_PROP_DIRECT, slide_dev->propbit);
#endif
#ifdef INPUT_TYPE_B_PROTOCOL
#if (LINUX_VERSION_CODE > KERNEL_VERSION(3, 7, 0))
	input_mt_init_slots(slide_dev, TS_MAX_SLIDE_FINGER, INPUT_MT_DIRECT);
#else
	input_mt_init_slots(slide_dev, TS_MAX_SLIDE_FINGER);
#endif
#endif
	input_set_abs_params(slide_dev, ABS_MT_POSITION_X, 0, ts->max_x, 0, 0);
	input_set_abs_params(slide_dev, ABS_MT_POSITION_Y, 0, ts->max_y, 0, 0);
	input_set_abs_params(slide_dev, ABS_MT_PRESSURE, 0, 255, 0, 0);
	input_set_abs_params(slide_dev, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params(slide_dev, ABS_MT_TRACKING_ID, 0, TS_MAX_SLIDE_FINGER, 0, 0);

	return 0;
}

static int goodix_chip_resume(void)
{
	struct goodix_ts_data *ts = goodix_ts;

	if (ts == NULL)
		return -ENODEV;

	GTP_INFO("Resume start");
	switch (ts->dev_data->easy_wakeup_info.sleep_mode) {
	case TS_POWER_OFF_MODE:
		goodix_power_switch(ts, SWITCH_ON);
		goodix_pinctrl_select_normal(ts);
		goodix_chip_reset();
		break;
	case TS_GESTURE_MODE:
	default:
		goodix_pinctrl_select_normal(ts);
		goodix_chip_reset();
		break;
	}

	goodix_feature_resume(ts);
	GTP_INFO("Resume end");
	return 0;
}

static int goodix_chip_after_resume(void *feature_info)
{
	if (goodix_ts) {
		msleep(40);
		goodix_ts->rawdiff_mode = false;
		goodix_feature_resume(goodix_ts);
	}

	return 0;
}

static int goodix_chip_suspend(void)
{
	struct goodix_ts_data *ts = goodix_ts;

	if (ts == NULL)
		return -ENODEV;

	GTP_INFO("Suspend start");
	switch (ts->dev_data->easy_wakeup_info.sleep_mode) {
	case TS_POWER_OFF_MODE:
		goodix_pinctrl_select_suspend(ts);
		goodix_power_switch(ts, SWITCH_OFF);
		break;
	case TS_GESTURE_MODE:
		goodix_pinctrl_select_gesture(ts);
		goodix_switch_wrokmode(GESTURE_MODE);
		break;
	default:
		goodix_pinctrl_select_suspend(ts);
		goodix_switch_wrokmode(SLEEP_MODE);
		break;
	}

	GTP_INFO("Suspend end");
	return 0;
}

static int goodix_fw_update_boot(char *file_name)
{
	if (goodix_ts == NULL || !file_name)
		return -ENODEV;

	snprintf(goodix_ts->firmware_name,
			sizeof(goodix_ts->firmware_name),
            "ts/%sgt1152.img", file_name);
    GTP_INFO("Requesting firmware image %s\n",goodix_ts->firmware_name);
	return gt1x_auto_update_proc(UPDATE_TYPE_HEADER);
}

static int goodix_fw_update_sd(void)
{
	if (goodix_ts == NULL)
		return -ENODEV;

	return gt1x_auto_update_proc(UPDATE_TYPE_FILE);
}

static int goodix_chip_get_info(struct ts_chip_info_param *info)
{
	struct goodix_ts_data *ts = goodix_ts;
	int ret = -1;
	u8 config_version = 0;
	if (!info || !ts)
		return -EINVAL;
	ret = goodix_i2c_read(0x8050, &config_version, 1);
	if (ret < 0 )
		GTP_ERROR("read config version error!");
	GTP_INFO("config version is %d",config_version);
    sprintf(info->mod_vendor, "GT%s_FW(%06X)_Boot(%04x)_config_version(%#x)", ts->hw_info.product_id, ts->hw_info.patch_id, ts->hw_info.mask_id >> 8,config_version);
	sprintf(info->ic_vendor, "GOODIX");
	//sprintf(info->fw_vendor, "sumsung");
	info->ttconfig_version = 0x00;
	//info->fw_verctrl_num = 0x00;

	return 0;
}

int goodix_reset_select_addr(void)
{	
	int reset_gpio;
	int irq_gpio;

	if (goodix_ts == NULL)
		return -ENODEV;

	reset_gpio = goodix_ts->dev_data->reset_gpio;
	irq_gpio = goodix_ts->dev_data->irq_gpio;
	
	gpio_direction_output(reset_gpio, 0);
	gpio_direction_output(irq_gpio, 1);
	usleep_range(100, 110);
	gpio_direction_output(reset_gpio, 1);
	msleep(8);     //must >= 6ms
	return 0;
}

#if GTP_INCELL_PANEL
int goodix_write_and_readback(u16 addr, u8 * buffer, s32 len)
{
    int ret;
    u8 d[len];
    
    ret = goodix_i2c_write(addr, buffer, len);
    if (ret < 0)
        return -1;

    ret = goodix_i2c_read(addr, d, len);
    if (ret < 0 || memcmp(buffer, d, len))
        return -1;

    return 0;
}

int goodix_incell_reset(void)
{
#define RST_RETRY       5
    int ret, retry = RST_RETRY;
    u8 d[2];


    do {
    	/* select i2c address */
	goodix_reset_select_addr();

    	/* test i2c */
        ret = goodix_i2c_read(0x4220, d, 1);

    } while (--retry && ret < 0);
    
    if (ret < 0) {
        return -1;
    }

    /* Stop cpu of the touch ic */
    retry = RST_RETRY;
    do {
        d[0] = 0x0C;
        ret = goodix_write_and_readback(0x4180, d, 1);

    } while (--retry && ret < 0);
    
    if (ret < 0) {
        GTP_ERROR("Hold error.");
        return -1;
    }

    /* skip sensor id check. [start] */
    retry = RST_RETRY;
    do {
        d[0] = 0x00;
        ret = goodix_write_and_readback(0x4305, d, 1);
        if (ret < 0)
            continue;
        
        d[0] = 0x2B;
        d[1] = 0x24;
        ret = goodix_write_and_readback(0x42c4, d, 2);
        if (ret < 0)
            continue;
        
        d[0] = 0xE1;
        d[1] = 0xD3;
        ret = goodix_write_and_readback(0x42e4, d, 2);
        if (ret < 0)
            continue;   
        
        d[0] = 0x01;
        ret = goodix_write_and_readback(0x4305, d, 1);
        if (ret < 0)
            continue;
        else
            break;
    } while (--retry ); 

    if (!retry)
        return -1;
    /* skip sensor id check. [end] */

    /* release hold of cpu */
    retry = RST_RETRY;
    do {
        d[0] = 0x00;
        ret = goodix_write_and_readback(0x4180, d, 1);
        
    } while (--retry && ret < 0);

    if (ret < 0)
        return -1;

    return 0;

}
#endif

int goodix_chip_reset(void)
{
	
	int ret;
	int irq_gpio;

	if (goodix_ts == NULL)
		return -ENODEV;

	GTP_INFO("Chip reset");
	irq_gpio = goodix_ts->dev_data->irq_gpio;

#if GTP_INCELL_PANEL
	ret = goodix_incell_reset();
	if (ret < 0)
		return ret;
#else 
	/* select i2c address */
	goodix_reset_select_addr();
#endif

	 /* int synchronization */
	gpio_direction_output(irq_gpio, 0);
	msleep(60);
	gpio_direction_input(irq_gpio);

	return goodix_init_watchdog();
}

static int goodix_glove_switch(struct ts_glove_info *info)
{
	static bool glove_en = false;
	int ret = 0;
	u8 buf = 0;

	if (!info || !goodix_ts) {
		GTP_ERROR("info is Null");
		return -ENOMEM;
	}

	switch (info->op_action) {
	case TS_ACTION_READ:
		if (glove_en)
			info->glove_switch = 1;
		else
			info->glove_switch = 0;
		break;
	case TS_ACTION_WRITE:
		if (info->glove_switch) {
			/* enable glove feature */
			ret = goodix_feature_switch(goodix_ts,
					TS_FEATURE_GLOVE, SWITCH_ON);
			if (!ret)
				glove_en = true;
		} else {
			/* disable glove feature */
			ret = goodix_feature_switch(goodix_ts,
					TS_FEATURE_GLOVE, SWITCH_OFF);
			if (!ret)
				glove_en = false;
		}

		if (ret < 0)
			GTP_ERROR("set glove switch(%d), failed : %d", buf, ret);
		break;
	default:
		GTP_ERROR("invalid switch status: %d", info->glove_switch);
		ret = -EINVAL;
		break;
	}

	return ret;
}

static void goodix_chip_shutdown(void)
{
	struct goodix_ts_data *ts = goodix_ts;

	if (ts == NULL)
		return;

	goodix_power_switch(ts, SWITCH_OFF);
	goodix_release_gpio(ts->dev_data);
	goodix_put_regulators(ts);
}

#if 1
static int goodix_charger_switch(struct ts_charger_info *info)
{
	int ret = 0;

	if (info == NULL)
		return -ENOMEM;

	switch (info->op_action) {
	case TS_ACTION_WRITE:
		if (info->charger_switch) {
			ret = goodix_send_cmd(GTP_CMD_CHARGER_ON, 0x00);
			GTP_INFO("Charger cmd switch on");
		} else {
			ret = goodix_send_cmd(GTP_CMD_CHARGER_OFF, 0x00);
			GTP_INFO("Charger cmd switch off");
		}
		break;
	case TS_ACTION_READ:
		if (info->charger_switch) {
			ret = goodix_send_cfg(&goodix_ts->charger_config);
			GTP_INFO("Charger cfg switch on");
		} else {
			ret = goodix_send_cfg(&goodix_ts->normal_config);
			GTP_INFO("Charger cfg switch off");
		}
		break;
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}
#endif

static int goodix_palm_switch(struct ts_palm_info *info)
{
	if (goodix_ts == NULL)
		return -ENODEV;

	return 0;
}

static int goodix_holster_switch(struct ts_holster_info *info)
{
	int ret = 0;

	if (!info || !goodix_ts) {
		GTP_ERROR("holster_switch: info is Null\n");
		ret = -ENOMEM;
		return ret;
	}

	switch (info->op_action) {
		case TS_ACTION_WRITE:
			if (info->holster_switch)
				ret = goodix_feature_switch(goodix_ts,
						TS_FEATURE_HOLSTER, SWITCH_ON);
			else 
				ret = goodix_feature_switch(goodix_ts,
						TS_FEATURE_HOLSTER, SWITCH_OFF);
			if (ret < 0)
				GTP_ERROR("set holster switch(%d), failed: %d",
							info->holster_switch, ret);
			break;
		case TS_ACTION_READ:
			GTP_INFO("invalid holster switch(%d) action: TS_ACTION_READ",
							info->holster_switch);
			break;
		default:
			GTP_INFO("invalid holster switch(%d) action: %d\n",
							info->holster_switch, info->op_action);
			ret = -EINVAL;
			break;
	}

	return ret;
}

static int goodix_pocket_switch(struct ts_pocket_info *info)
{
	int ret = 0;

	if (!info || !goodix_ts) {
		GTP_ERROR("pocket_switch: info is Null\n");
		ret = -ENOMEM;
		return ret;
	}

	switch (info->op_action) {
		case TS_ACTION_WRITE:
			if (info->pocket_switch)
				ret = goodix_feature_switch(goodix_ts,
						TS_FEATURE_POCKET, SWITCH_ON);
			else
				ret = goodix_feature_switch(goodix_ts,
						TS_FEATURE_POCKET, SWITCH_OFF);
			if (ret < 0)
				GTP_ERROR("set pocket switch(%d), failed: %d",
							info->pocket_switch, ret);
			break;
		case TS_ACTION_READ:
			GTP_INFO("invalid pocket switch(%d) action: TS_ACTION_READ",
							info->pocket_switch);
			break;
		default:
			GTP_INFO("invalid pocket switch(%d) action: %d\n",
							info->pocket_switch, info->op_action);
			ret = -EINVAL;
			break;
	}

	return ret;
}

/*
 * goodix_check_hw_status - Hw exception checking
 */
static int goodix_check_hw_status(void)
{
	u8 esd_buf[4] = {0};
	u8 feed_wd[3] = {0xAA, 0x00, 0x55};
	int ret = -1, i;

	if (goodix_ts == NULL)
		return -ENODEV;

	for (i = 0; i < 3; i++) {
		ret = goodix_i2c_read(GTP_REG_CMD, esd_buf, 4);
		if (unlikely(ret < 0))
			break;
		if (esd_buf[0] != 0xAA && esd_buf[3] == 0xAA)
			break; /* status:OK */
		msleep(50);
	}

	if (likely(!ret && i < 3)) {
		GTP_DEBUG("HW status:OK");
		/* IC works normally, feed the watchdog */
		ret = goodix_i2c_write(GTP_REG_CMD, &feed_wd[0],
							sizeof(feed_wd));
	} else {
		GTP_ERROR("HW status:Error,reg{8040h}=%02X,reg{8043h}=%02X",
					esd_buf[0], esd_buf[3]);
		ret = -EIO;
	}

	return ret;
}

static int goodix_regs_operate(struct ts_regs_info *info)
{
	int retval = NO_ERR;
	unsigned int regs_addr = info->fhandler;
	u8 value[TS_MAX_REG_VALUE_NUM] = {0};
	int i = 0;

	GTP_INFO("register operate test!\n");
	if(regs_addr < 0) {
		GTP_ERROR("get regs_addr fail, regs_addr = %d\n", regs_addr);
		return regs_addr;
	}
	switch (info->op_action) {
	case TS_ACTION_WRITE:
		for (i = 0; i < info->num; i++) {
			value[i] = info->values[i];
		}
		retval = goodix_i2c_write(regs_addr, value, info->num);

		if (retval < 0) {
			GTP_ERROR("TS_ACTION_WRITE error, fhandler(%d) type: %d offset: %d\n", info->fhandler, info->type, info->offset);
			retval = -EINVAL;
			goto out;
		}
		break;
	case TS_ACTION_READ:
		retval = goodix_i2c_read(regs_addr, value, info->num);
		if (retval < 0) {
			GTP_ERROR("TS_ACTION_READ error, fhandler(%d) type: %d offset: %d\n", info->fhandler, info->type, info->offset);
			retval = -EINVAL;
			goto out;
		}
		for (i = 0; i < info->num; i++) {
			info->values[i] = value[i];
			GTP_INFO("%s : %d  value[%d] : 0x%4x\n",__func__,__LINE__, i,  value[i]);
		}
		break;
	default:
		GTP_ERROR("reg operate default invalid action %d\n", info->op_action);
		retval = -EINVAL;
		break;
	}
out:
	return retval;
}

#ifdef ROI
static int goodix_roi_switch(struct ts_roi_info *info)
{
	int ret = 0;

	if (!info || !goodix_ts) {
		GTP_ERROR("roi_switch: info is Null");
		ret = -ENOMEM;
		return ret;
	}

	switch (info->op_action) {
	case TS_ACTION_WRITE:
		if (info->roi_switch == 1) {
			goodix_ts->roi.enabled = true;
		} else if (info->roi_switch == 0) {
			goodix_ts->roi.enabled = false;
		} else {
			GTP_ERROR("Invalid roi switch value:%d", info->roi_switch);
			ret = -EINVAL;
		}
		break;
	case TS_ACTION_READ:
		// read
		break;
	default:
		break;
	}
	return ret;
}

static u8* goodix_roi_rawdata(void)
{
	u8 * rawdata_ptr = NULL;

	if (goodix_ts == NULL)
		return NULL;

	mutex_lock(&goodix_ts->roi.mutex);
	if (goodix_ts->roi.enabled && goodix_ts->roi.data_ready)
		rawdata_ptr = (u8 *)goodix_ts->roi.rawdata;
	mutex_unlock(&goodix_ts->roi.mutex);

	return rawdata_ptr;
}
#endif

#if 0
static int goodix_chip_get_capacitance_test_type(
		struct ts_test_type_info *info)
{
	int ret = 0;

	if (!info) {
		GTP_INFO("info is null");
		return -EINVAL;
	}

	switch (info->op_action) {
	case TS_ACTION_READ:
		memcpy(info->tp_test_type, 
			goodix_ts->dev_data->tp_test_type,
			TS_CAP_TEST_TYPE_LEN);
		GTP_INFO("test_type= %s", info->tp_test_type);
		break;
	case TS_ACTION_WRITE:
		break;
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}
#endif

struct ts_device_ops ts_goodix_ops = {
	.chip_detect = goodix_chip_detect,
	.chip_init = goodix_chip_init,
	.chip_parse_config = goodix_chip_parse_config,
	.chip_input_config = goodix_input_config,
	.chip_slide_config = goodix_slide_config,
//	.chip_irq_top_half = goodix_irq_top_half,
	.chip_irq_bottom_half = goodix_irq_bottom_half,
	.chip_reset = goodix_chip_reset,
	.chip_fw_update_boot = goodix_fw_update_boot,
	.chip_fw_update_sd = goodix_fw_update_sd,
	.chip_get_info = goodix_chip_get_info,
//    .chip_set_info_flag = goodix_set_info_flag,
//    .chip_before_suspend = goodix_before_suspend,
	.chip_suspend = goodix_chip_suspend,
	.chip_resume = goodix_chip_resume,
    .chip_after_resume = goodix_chip_after_resume,
    .chip_get_rawdata = goodix_get_rawdata,
	.chip_glove_switch = goodix_glove_switch,
	.chip_shutdown = goodix_chip_shutdown,
#if 1
	.chip_charger_switch = goodix_charger_switch,
#endif
	.chip_palm_switch = goodix_palm_switch,
	.chip_holster_switch = goodix_holster_switch,
	.chip_pocket_switch = goodix_pocket_switch,
#ifdef ROI
	.chip_roi_switch = goodix_roi_switch,
	.chip_roi_rawdata = goodix_roi_rawdata,
#endif
	.chip_check_status = goodix_check_hw_status,
    .chip_regs_operate = goodix_regs_operate,
#if 0
	.chip_get_capacitance_test_type =
		goodix_chip_get_capacitance_test_type,
#endif
#if defined (CONFIG_ZEUSIS_DSM)
//    .chip_dsm_debug = goodix_dsm_debug,
#endif

#ifdef ZEUSIS_TOUCHSCREEN_TEST
//    .chip_test = test_dbg_cmd_test,
#endif
//    .chip_wrong_touch=goodix_wrong_touch,
};

