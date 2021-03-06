/*
 * goodix_gt1x_update.c - Firmware update
 *
 * 2010 - 2016 Goodix Technology.
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
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/wait.h>
#include <linux/time.h>
#include <linux/delay.h>
#include <linux/namei.h>
#include <linux/mount.h>
#include <asm/uaccess.h>
#include <linux/firmware.h>
#include <linux/slab.h>
#include "goodix_ts.h"

#define GT1X_FW_NAME      "gt1x_fw.bin"

#define UPDATE_FILE_PATH_1          "/data/_goodix_update_.bin"
#define UPDATE_FILE_PATH_2          "/sdcard/_goodix_update_.bin"

#define CONFIG_FILE_PATH_1          "/data/_gt1x_config_.cfg"
#define CONFIG_FILE_PATH_2          "/sdcard/_gt1x_config_.cfg"

#define FOUND_FW_PATH_1              0x01
#define FOUND_FW_PATH_2              0x02
#define FOUND_CFG_PATH_1             0x04
#define FOUND_CFG_PATH_2             0x08

#define PACK_SIZE                    256

// hardware register define
#define _bRW_MISCTL__SRAM_BANK       0x4048
#define _bRW_MISCTL__MEM_CD_EN       0x4049
#define _bRW_MISCTL__CACHE_EN        0x404B
#define _bRW_MISCTL__TMR0_EN         0x40B0
#define _rRW_MISCTL__SWRST_B0_       0x4180
#define _bWO_MISCTL__CPU_SWRST_PULSE 0x4184
#define _rRW_MISCTL__BOOTCTL_B0_     0x4190
#define _rRW_MISCTL__BOOT_OPT_B0_    0x4218
#define _rRW_MISCTL__BOOT_CTL_       0x5094
#define _bRW_MISCTL__DSP_MCU_PWR_    0x4010
#define _bRW_MISCTL__PATCH_AREA_EN_  0x404D

#define FW_HEAD_SIZE                         128
#define FW_HEAD_SUBSYSTEM_INFO_SIZE          8
#define FW_HEAD_OFFSET_SUBSYSTEM_INFO_BASE   32

#define FW_SECTION_TYPE_SS51_ISP               0x01
#define FW_SECTION_TYPE_SS51_PATCH             0x02
#define FW_SECTION_TYPE_SS51_PATCH_OVERLAY     0x03
#define FW_SECTION_TYPE_DSP                    0x04
#define FW_SECTION_TYPE_HOTKNOT                0x05
#define FW_SECTION_TYPE_GESTURE                0x06
#define FW_SECTION_TYPE_GESTURE_OVERLAY        0x07
#define FW_SECTION_TYPE_FLASHLESS_FAST_POWER   0x08

#define UPDATE_STATUS_IDLE     0
#define UPDATE_STATUS_RUNNING  1
#define UPDATE_STATUS_ABORT    2

#define _ERROR(e)      ((0x01 << e) | (0x01 << (sizeof(s32) * 8 - 1)))
#define ERROR          _ERROR(1)	//for common use
//system relevant
#define ERROR_IIC      _ERROR(2)	//IIC communication error.
#define ERROR_MEM      _ERROR(3)	//memory error.

//system irrelevant
#define ERROR_HN_VER   _ERROR(10)	//HotKnot version error.
#define ERROR_CHECK    _ERROR(11)	//Compare src and dst error.
#define ERROR_RETRY    _ERROR(12)	//Too many retries.
#define ERROR_PATH     _ERROR(13)	//Mount path error
#define ERROR_FW       _ERROR(14)
#define ERROR_FILE     _ERROR(15)
#define ERROR_VALUE    _ERROR(16)	//Illegal value of variables

struct fw_subsystem_info {
	int type;
	int length;
	u32 address;
	int offset;
};

#pragma pack(1)
struct fw_info {
	u32 length;
	u16 checksum;
	u8 target_mask[6];
	u8 target_mask_version[3];
	u8 pid[6];
	u8 version[3];
	u8 subsystem_count;
	u8 chip_type;
	u8 reserved[6];
	struct fw_subsystem_info subsystem[12];
};
#pragma pack()

// debug+
struct fw_update_info pad_update_info = {
// debug-
//static struct fw_update_info pad_update_info = {
	.status = UPDATE_STATUS_IDLE,
	.progress = 0,
	.max_progress = 9,
	.force_update = 0
};

int pad_gt1x_update_prepare(char *filename);
int pad_gt1x_check_firmware(void);
u8 *pad_gt1x_get_fw_data(u32 offset, int length);
int pad_gt1x_update_judge(void);
int pad_gt1x_run_ss51_isp(u8 * ss51_isp, int length);
int pad_gt1x_burn_subsystem(struct fw_subsystem_info *subsystem);
u16 pad_gt1x_calc_checksum(u8 * fw, u32 length);
int pad_gt1x_recall_check(u8 * chk_src, u16 start_rd_addr, u16 chk_length);
void pad_gt1x_update_cleanup(void);
int pad_gt1x_check_subsystem_in_flash(struct fw_subsystem_info *subsystem);
int pad_gt1x_read_flash(u32 addr, int length);
int pad_gt1x_error_erase(void);
void pad_dump_to_file(u16 addr, int length, char *filepath);

void goodix_fw_update_sd(void);
void goodix_fw_update_boot(char *file_name);

int pad_gt1x_update_firmware(char *filename);

static int gt1x_search_update_files(void);

int pad_gt1x_hold_ss51_dsp(void);
void pad_gt1x_leave_update_mode(void);

/**
 * @return: return 0 if success, otherwise return a negative number
 *          which contains the error code.
 */
s32 pad_gt1x_check_fs_mounted(char *path_name)
{
	struct path root_path;
	struct path path;
	s32 err;

	err = kern_path("/", LOOKUP_FOLLOW, &root_path);
	if (err)
		return ERROR_PATH;

	err = kern_path(path_name, LOOKUP_FOLLOW, &path);
	if (err) {
		err = ERROR_PATH;
		goto check_fs_fail;
	}

	if (path.mnt->mnt_sb == root_path.mnt->mnt_sb) {
		// not mounted
		err = ERROR_PATH;
	} else {
		err = 0;
	}

	path_put(&path);
check_fs_fail:
	path_put(&root_path);
	return err;
}

int pad_gt1x_i2c_write_with_readback(u16 addr, u8 * buffer, int length)
{
	u8 buf[100];
	int ret = pad_goodix_i2c_write(addr, buffer, length);
	if (ret) {
		return ret;
	}
	ret = pad_goodix_i2c_read(addr, buf, length);
	if (ret) {
		return ret;
	}
	if (memcmp(buf, buffer, length)) {
		return ERROR_CHECK;
	}
	return 0;
}

#define getU32(a) ((u32)pad_getUint((u8 *)(a), 4))
#define getU16(a) ((u16)pad_getUint((u8 *)(a), 2))
u32 pad_getUint(u8 * buffer, int len)
{
	u32 num = 0;
	int i;
	for (i = 0; i < len; i++) {
		num <<= 8;
		num += buffer[i];
	}
	return num;
}

static u8 ascii2hex(u8 a)
{
	s8 value = 0;
	if (a >= '0' && a <= '9') {
		value = a - '0';
	} else if (a >= 'A' && a <= 'F') {
		value = a - 'A' + 0x0A;
	} else if (a >= 'a' && a <= 'f') {
		value = a - 'a' + 0x0A;
	} else {
		value = 0xff;
	}
	return value;
}

int pad_gt1x_parse_config(char *filename, u8 * config)
{
	mm_segment_t old_fs;
	struct file *fp = NULL;
	u8 *buf;
	int i;
	int len;
	int cur_len = -1;
	u8 high, low;

	old_fs = get_fs();
	set_fs(KERNEL_DS);

	fp = filp_open(filename, O_RDONLY, 0);
	if (IS_ERR(fp)) {
		GTP_ERROR("Open config file error!(file: %s)", filename);
		goto parse_cfg_fail1;
	}
	len = fp->f_op->llseek(fp, 0, SEEK_END);
	if (len > GTP_CONFIG_MAX_LENGTH * 6 || len < GTP_CONFIG_MAX_LENGTH) {
		GTP_ERROR("Config is invalid!(length: %d)", len);
		goto parse_cfg_fail2;
	}
	buf = (u8 *) kzalloc(len, GFP_KERNEL);
	if (buf == NULL) {
		GTP_ERROR("Allocate memory failed!(size: %d)", len);
		goto parse_cfg_fail2;
	}
	fp->f_op->llseek(fp, 0, SEEK_SET);
	if (fp->f_op->read(fp, (char *)buf, len, &fp->f_pos) != len) {
		GTP_ERROR("Read %d bytes from file failed!", len);
	}

	GTP_INFO("Parse config file: %s (%d bytes)", filename, len);

	for (i = 0, cur_len = 0; i < len && cur_len < GTP_CONFIG_MAX_LENGTH;) {
		if (buf[i] == ' ' || buf[i] == '\r' || buf[i] == '\n' || buf[i] == ',') {
			i++;
			continue;
		}
		if (buf[i] == '0' && (buf[i + 1] == 'x' || buf[i + 1] == 'X')) {

			high = ascii2hex(buf[i + 2]);
			low = ascii2hex(buf[i + 3]);

			if (high != 0xFF && low != 0xFF) {
				config[cur_len++] = (high << 4) + low;
				i += 4;
				continue;
			}
		}
		GTP_ERROR("Illegal config file!");
		cur_len = -1;
		break;
	}

	if (cur_len < GTP_CONFIG_MIN_LENGTH || config[cur_len - 1] != 0x01) {
		cur_len = -1;
	} else {
		for (i = 0; i < cur_len; i++) {
			if (i % 10 == 0) {
				printk("\n<<GTP-DBG>>:");
			}
			printk("0x%02x,", config[i]);
		}
		printk("\n");
	}

	kfree(buf);
parse_cfg_fail2:
	filp_close(fp, NULL);
parse_cfg_fail1:
	set_fs(old_fs);

	return cur_len;
}

// temp solution
static void goodix_select_addr(void)
{
	int reset_gpio = pad_goodix_ts->dev_data->reset_gpio;

	/* reset */
	gpio_direction_output(reset_gpio, 0);
	usleep_range(2000, 2010);
	gpio_direction_output(reset_gpio, 1);
	usleep_range(2000, 2010);
}

int pad_gt1x_auto_update_proc(int type)
{
	if (type != UPDATE_TYPE_FILE && type != UPDATE_TYPE_HEADER) {
		GTP_ERROR("Invalid update type");
		return -EINVAL;
	}

	pad_update_info.update_type = type;
	if (pad_update_info.update_type == UPDATE_TYPE_FILE) {
		int ret;
		char *filename;
		u8 config[GTP_CONFIG_ORG_LENGTH + GTP_CONFIG_EXT_LENGTH] = { 0 };

		GTP_INFO("Start auto sdcard update thread...");
		ret = gt1x_search_update_files();
		if (ret & (FOUND_FW_PATH_1 | FOUND_FW_PATH_2)) {
			if (ret & FOUND_FW_PATH_1) {
				filename = UPDATE_FILE_PATH_1;
			} else {
				filename = UPDATE_FILE_PATH_2;
			}
			pad_gt1x_update_firmware(filename);
		}

		if (ret & (FOUND_CFG_PATH_1 | FOUND_CFG_PATH_2)) {
			if (ret & FOUND_CFG_PATH_1) {
				filename = CONFIG_FILE_PATH_1;
			} else {
				filename = CONFIG_FILE_PATH_2;
			}

			if (pad_gt1x_parse_config(filename, config) > 0) {
				ret = pad_goodix_i2c_write(GTP_REG_CONFIG_DATA, config, GTP_CONFIG_ORG_LENGTH);
				if (ret < 0) {
					GTP_ERROR("Update config failed!");
					return 0;
				}

				/* extends config */
				if (config[0x805A - GTP_REG_CONFIG_DATA] & 0x40) {
					ret = pad_goodix_i2c_write(GTP_REG_EXT_CONFIG,
						&config[GTP_CONFIG_ORG_LENGTH], GTP_CONFIG_EXT_LENGTH);

					if (ret < 0) {
						GTP_ERROR("Update ext config failed!");
						return 0;
					}
				}

				GTP_INFO("Update config successfully!");
			}
		}
	}else{
		GTP_INFO("Start auto boot update thread...");
		pad_gt1x_update_firmware(NULL);
	}
		return 0;
}

static int gt1x_search_update_files(void)
{
	int retry = 20 * 2;	//wait 10s(max) if fs is not ready
	struct file *pfile = NULL;
	mm_segment_t old_fs;
	int found = 0;

	old_fs = get_fs();
	set_fs(KERNEL_DS);

	GTP_INFO("Search firmware file...");
	while (retry-- > 0) {
		msleep(500);

		// check if rootfs is ready
		if (pad_gt1x_check_fs_mounted("/data")) {
			GTP_DEBUG("filesystem is not ready");
			continue;
		}
		// search firmware
		pfile = filp_open(UPDATE_FILE_PATH_1, O_RDONLY, 0);
		if (IS_ERR_OR_NULL(pfile)) {
			pfile = filp_open(UPDATE_FILE_PATH_2, O_RDONLY, 0);
			if (!IS_ERR_OR_NULL(pfile))
				found |= FOUND_FW_PATH_2;
		} else {
			found |= FOUND_FW_PATH_1;
		}

		if (!IS_ERR_OR_NULL(pfile)) {
			filp_close(pfile, NULL);
		}

		// search config file
		pfile = filp_open(CONFIG_FILE_PATH_1, O_RDONLY, 0);
		if (IS_ERR(pfile)) {
			pfile = filp_open(CONFIG_FILE_PATH_2, O_RDONLY, 0);
			if (!IS_ERR(pfile)) {
				found |= FOUND_CFG_PATH_2;
			}
		} else {
			found |= FOUND_CFG_PATH_1;
		}
		if (!IS_ERR(pfile)) {
			filp_close(pfile, NULL);
		}

		if (found) {
			break;
		}

		GTP_INFO("Not found firmware or config file, retry.");
	}
	set_fs(old_fs);

	return found;
}


void pad_gt1x_enter_update_mode(void)
{
	GTP_DEBUG("Enter FW update mode.");

	// debug+
	//gt1x_irq_disable();
}

int pad_gt1x_update_firmware(char *filename)
{
	int i = 0;
	int ret = 0;
	u8 *p;

	if (pad_update_info.status != UPDATE_STATUS_IDLE) {
		GTP_ERROR("Update process is running!");
		return ERROR;
	}
	pad_update_info.status = UPDATE_STATUS_RUNNING;
	pad_update_info.progress = 0;

	pad_gt1x_enter_update_mode();

	GTP_INFO("The filename is %s",filename);

	ret = pad_gt1x_update_prepare(filename);
	if (ret) {
		pad_update_info.status = UPDATE_STATUS_IDLE;
		return ret;
	}

	ret = pad_gt1x_check_firmware();
	if (ret) {
		pad_update_info.status = UPDATE_STATUS_ABORT;
		goto gt1x_update_exit;
	}
#if GTP_FW_UPDATE_VERIFY
	pad_update_info.max_progress =
		6 + pad_update_info.firmware_info->subsystem_count;
#else
	pad_update_info.max_progress =
		3 + pad_update_info.firmware_info->subsystem_count;
#endif
	pad_update_info.progress++; // 1

	ret = pad_gt1x_update_judge();
	if (ret) {
		pad_update_info.status = UPDATE_STATUS_ABORT;
		goto gt1x_update_exit;
	}
	pad_update_info.progress++; // 2

	p = pad_gt1x_get_fw_data(pad_update_info.firmware_info->subsystem[0].offset, pad_update_info.firmware_info->subsystem[0].length);
	if (p == NULL) {
		GTP_ERROR("get isp fail");
		ret = ERROR_FW;
		pad_update_info.status = UPDATE_STATUS_ABORT;
		goto gt1x_update_exit;
	}
	pad_update_info.progress++; // 3

	ret = pad_gt1x_run_ss51_isp(p, pad_update_info.firmware_info->subsystem[0].length);
	if (ret) {
		GTP_ERROR("run isp fail");
		goto gt1x_update_exit;
	}
	pad_update_info.progress++; // 4
	msleep(800);

	for (i = 1; i < pad_update_info.firmware_info->subsystem_count; i++) {
		GTP_INFO("subsystem: %d", pad_update_info.firmware_info->subsystem[i].type);
		GTP_INFO("Length: %d", pad_update_info.firmware_info->subsystem[i].length);
		GTP_INFO("Address: %d", pad_update_info.firmware_info->subsystem[i].address);

		ret = pad_gt1x_burn_subsystem(&(pad_update_info.firmware_info->subsystem[i]));
		if (ret) {
			GTP_ERROR("burn subsystem fail!");
			goto gt1x_update_exit;
		}
		pad_update_info.progress++;
	}

#if GTP_FW_UPDATE_VERIFY
	pad_goodix_chip_reset();

	p = pad_gt1x_get_fw_data(pad_update_info.firmware_info->subsystem[0].offset, pad_update_info.firmware_info->subsystem[0].length);
	if (p == NULL) {
		GTP_ERROR("get isp fail");
		ret = ERROR_FW;
		goto gt1x_update_exit;
	}
	pad_update_info.progress++;

	ret = pad_gt1x_run_ss51_isp(p, pad_update_info.firmware_info->subsystem[0].length);
	if (ret) {
		GTP_ERROR("run isp fail");
		goto gt1x_update_exit;
	}
	pad_update_info.progress++;

	GTP_INFO("Reset guitar & check firmware in flash.");
	for (i = 1; i < pad_update_info.firmware_info->subsystem_count; i++) {
		GTP_INFO("subsystem: %d", pad_update_info.firmware_info->subsystem[i].type);
		GTP_INFO("Length: %d", pad_update_info.firmware_info->subsystem[i].length);
		GTP_INFO("Address: %d", pad_update_info.firmware_info->subsystem[i].address);

		ret = pad_gt1x_check_subsystem_in_flash(&(pad_update_info.firmware_info->subsystem[i]));
		if (ret) {
			pad_gt1x_error_erase();
			break;
		}
	}
	pad_update_info.progress++;
#endif

gt1x_update_exit:
	pad_gt1x_update_cleanup();
	pad_gt1x_leave_update_mode();
	// debug+
	pad_goodix_read_version(&pad_goodix_ts->hw_info);
	if (ret) {
			pad_update_info.progress = 2 * pad_update_info.max_progress;
			// debug+
			pad_goodix_ts->fw_update_ok = false;
			// debug-
				if (pad_update_info.status == UPDATE_STATUS_ABORT)
					GTP_ERROR("No need to update firmware");
				else
			GTP_ERROR("Update firmware failed!");
				goto exit;
	}
	// debug+
	pad_goodix_ts->fw_update_ok = true;
	// debug-
	GTP_INFO("Update firmware succeefully!");
exit:
	pad_update_info.status = UPDATE_STATUS_IDLE;
	return ret;
}

int pad_gt1x_update_prepare(char *filename)
{

	int ret = 0;
	int retry = 5;

	if (filename == NULL) {
		pad_update_info.fw_name = NULL;
		pad_update_info.fw = NULL;
		ret = request_firmware(&pad_update_info.fw, pad_goodix_ts->firmware_name,
			&pad_goodix_ts->pdev->dev);
		if (ret < 0) {
			GTP_ERROR("Request firmware failed - %s (%d)\n",
				pad_goodix_ts->firmware_name, ret);
			return ERROR_FW;
		}
		GTP_INFO("Firmware size: %zd", pad_update_info.fw->size);

		pad_update_info.update_type = UPDATE_TYPE_HEADER;
		pad_update_info.fw_data = (u8 *)pad_update_info.fw->data;
		pad_update_info.fw_length = pad_update_info.fw->size;
	} else {
		GTP_INFO("Firmware: %s", filename);
		pad_update_info.old_fs = get_fs();
		set_fs(KERNEL_DS);
		pad_update_info.fw_name = filename;
		pad_update_info.update_type = UPDATE_TYPE_FILE;
		pad_update_info.fw_file = filp_open(pad_update_info.fw_name, O_RDONLY, 0);
		if (IS_ERR_OR_NULL(pad_update_info.fw_file)) {
			GTP_ERROR("Open update file(%s) error:%zd!", pad_update_info.fw_name,
							PTR_ERR(pad_update_info.fw_file));
			set_fs(pad_update_info.old_fs);
			return ERROR_FILE;
		}
		pad_update_info.fw_file->f_op->llseek(pad_update_info.fw_file, 0, SEEK_SET);
		pad_update_info.fw_length = pad_update_info.fw_file->f_op->llseek(pad_update_info.fw_file, 0, SEEK_END);
	}

	while (retry-- > 0) {
		pad_update_info.firmware_info = (struct fw_info *)kzalloc(sizeof(struct fw_info), GFP_KERNEL);
		if (pad_update_info.firmware_info == NULL) {
			GTP_INFO("Alloc %zu bytes memory fail.", sizeof(struct fw_info));
			continue;
		} else {
			GTP_INFO("Alloc %zu bytes memory success.", sizeof(struct fw_info));
			break;
		}
	}
	if (retry <= 0) {
		ret = ERROR_RETRY;
		goto gt1x_update_pre_fail1;
	}

	retry = 5;
	while (retry-- > 0) {
		pad_update_info.buffer = (u8 *) kzalloc(1024 * 4, GFP_KERNEL);
		if (pad_update_info.buffer == NULL) {
			GTP_ERROR("Alloc %d bytes memory fail.", 1024 * 4);
			continue;
		} else {
			GTP_INFO("Alloc %d bytes memory success.", 1024 * 4);
			break;
		}
	}
	if (retry <= 0) {
		ret = ERROR_RETRY;
		goto gt1x_update_pre_fail0;
	}

	return 0;

gt1x_update_pre_fail0:
	kfree(pad_update_info.firmware_info);
gt1x_update_pre_fail1:
	if (pad_update_info.update_type == UPDATE_TYPE_HEADER) {
		if (pad_update_info.fw != NULL) {
			release_firmware(pad_update_info.fw);
			pad_update_info.fw = NULL;
		}
	}else{
		filp_close(pad_update_info.fw_file, NULL);
		set_fs(pad_update_info.old_fs);
	}

	return ret;
}

void pad_gt1x_update_cleanup(void)
{
	if (pad_update_info.update_type == UPDATE_TYPE_FILE) {
		if (pad_update_info.fw_file != NULL) {
			filp_close(pad_update_info.fw_file, NULL);
			pad_update_info.fw_file = NULL;
		}
		set_fs(pad_update_info.old_fs);
	}

	if (pad_update_info.update_type == UPDATE_TYPE_HEADER) {
		if (pad_update_info.fw != NULL) {
			release_firmware(pad_update_info.fw);
			pad_update_info.fw = NULL;
		}
	}

	if (pad_update_info.buffer != NULL) {
		kfree(pad_update_info.buffer);
		pad_update_info.buffer = NULL;
	}
	if (pad_update_info.firmware_info != NULL) {
		kfree(pad_update_info.firmware_info);
		pad_update_info.firmware_info = NULL;
	}
}

int pad_gt1x_check_firmware(void)
{
	u16 checksum;
	u16 checksum_in_header;
	u8 *p;
	struct fw_info *firmware_tmpinfo;
	int i;
	int offset;

	// compare file length with the length field in the firmware header
	if (pad_update_info.fw_length < FW_HEAD_SIZE) {
		GTP_ERROR("Bad firmware!(file length: %d)", pad_update_info.fw_length);
		return ERROR_CHECK;
	}
	p = pad_gt1x_get_fw_data(0, 6);
	if (p == NULL) {
		return ERROR_FW;
	}

	if (getU32(p) + 6 != pad_update_info.fw_length) {
		GTP_ERROR("Bad firmware!(file length: %d, header define: %d)", pad_update_info.fw_length, getU32(p));
		return ERROR_CHECK;
	}
	// check firmware's checksum
	checksum_in_header = getU16(&p[4]);
	checksum = 0;
	for (i = 6; i < pad_update_info.fw_length; i++) {
		p = pad_gt1x_get_fw_data(i, 1);
		if (p == NULL) {
			return ERROR_FW;
		}
		checksum += p[0];
	}

	if (checksum != checksum_in_header) {
		GTP_ERROR("Bad firmware!(checksum: 0x%04X, header define: 0x%04X)", checksum, checksum_in_header);
		return ERROR_CHECK;
	}
	// parse firmware
	p = pad_gt1x_get_fw_data(0, FW_HEAD_SIZE);
	if (p == NULL) {
		return ERROR_FW;
	}
	memcpy((u8 *) pad_update_info.firmware_info, p, FW_HEAD_SIZE - 8 * 12);
	pad_update_info.firmware_info->pid[5] = 0;

	p = &p[FW_HEAD_OFFSET_SUBSYSTEM_INFO_BASE];
	firmware_tmpinfo = pad_update_info.firmware_info;
	offset = FW_HEAD_SIZE;
	for (i = 0; i < firmware_tmpinfo->subsystem_count; i++) {
		firmware_tmpinfo->subsystem[i].type = p[i * FW_HEAD_SUBSYSTEM_INFO_SIZE];
		firmware_tmpinfo->subsystem[i].length = getU16(&p[i * FW_HEAD_SUBSYSTEM_INFO_SIZE + 1]);
		firmware_tmpinfo->subsystem[i].address = getU16(&p[i * FW_HEAD_SUBSYSTEM_INFO_SIZE + 3]) * 256;
		firmware_tmpinfo->subsystem[i].offset = offset;
		offset += firmware_tmpinfo->subsystem[i].length;
	}

	// print update information
	GTP_INFO("Update type: %s", pad_update_info.update_type == UPDATE_TYPE_HEADER ? "Header" : "File");
	GTP_INFO("Firmware length: %d", pad_update_info.fw_length);
	GTP_INFO("Firmware product: GT%s", pad_update_info.firmware_info->pid);
	GTP_INFO("Firmware patch: %02X%02X%02X", pad_update_info.firmware_info->version[0], pad_update_info.firmware_info->version[1], pad_update_info.firmware_info->version[2]);
	GTP_INFO("Firmware chip: 0x%02X", pad_update_info.firmware_info->chip_type);
	GTP_INFO("Subsystem count: %d", pad_update_info.firmware_info->subsystem_count);
	for (i = 0; i < pad_update_info.firmware_info->subsystem_count; i++) {
		GTP_DEBUG("------------------------------------------");
		GTP_DEBUG("Subsystem: %d", i);
		GTP_DEBUG("Type: %d", pad_update_info.firmware_info->subsystem[i].type);
		GTP_DEBUG("Length: %d", pad_update_info.firmware_info->subsystem[i].length);
		GTP_DEBUG("Address: 0x%08X", pad_update_info.firmware_info->subsystem[i].address);
		GTP_DEBUG("Offset: %d", pad_update_info.firmware_info->subsystem[i].offset);
	}

	return 0;
}

/**
 * @return: return a pointer pointed at the content of firmware
 *          if success, otherwise return NULL.
 */
u8 *pad_gt1x_get_fw_data(u32 offset, int length)
{
	int ret;
	if (pad_update_info.update_type == UPDATE_TYPE_FILE) {
		pad_update_info.fw_file->f_op->llseek(pad_update_info.fw_file, offset, SEEK_SET);
		ret = vfs_read(pad_update_info.fw_file, (char *)pad_update_info.buffer, length, &pad_update_info.fw_file->f_pos);
		if (ret < 0) {
			GTP_ERROR("Read data error!");
			return NULL;
		}
		return pad_update_info.buffer;
	} else {
		return &pad_update_info.fw_data[offset];
	}
}

int pad_gt1x_update_judge(void)
{
	int ret;
	u8 reg_val[2] = {0};
	u8 retry = 2;
	struct goodix_hw_info ver_info;
	struct goodix_hw_info fw_ver_info;

	fw_ver_info.mask_id = (pad_update_info.firmware_info->target_mask_version[0] << 16)
		| (pad_update_info.firmware_info->target_mask_version[1] << 8)
		| (pad_update_info.firmware_info->target_mask_version[2]);
	fw_ver_info.patch_id = (pad_update_info.firmware_info->version[0] << 16)
		| (pad_update_info.firmware_info->version[1] << 8)
		| (pad_update_info.firmware_info->version[2]);
	memcpy(fw_ver_info.product_id, pad_update_info.firmware_info->pid, 4);
	fw_ver_info.product_id[4] = 0;

	/* check fw status reg */
	do {
		ret = pad_goodix_i2c_read_dbl_check(GTP_REG_FW_CHK_MAINSYS, reg_val, 1);
		if (ret < 0) {	/* read reg failed */
			goto _reset;
		} else if (ret > 0) {
			continue;
		}

		ret = pad_goodix_i2c_read_dbl_check(GTP_REG_FW_CHK_SUBSYS, &reg_val[1], 1);
		if (ret < 0) {
			goto _reset;
		} else if (ret > 0) {
			continue;
		}

		break;
_reset:
		pad_goodix_chip_reset();
	}while (--retry);

	if (!retry) {
		GTP_INFO("Update abort because of i2c error.");
		return ERROR_CHECK;
	}
	if (reg_val[0] != 0xBE || reg_val[1] == 0xAA) {
		GTP_INFO("Check fw status reg not pass,reg[0x814E]=0x%2X,reg[0x5095]=0x%2X!",
			reg_val[0], reg_val[1]);
		return 0;
	}

	if (pad_update_info.force_update)
		return 0;

	ret = pad_goodix_read_version(&ver_info);
	if (ret < 0) {
		GTP_INFO("Get IC's version info failed, force update!");
		return 0;
	}

	if (memcmp(fw_ver_info.product_id, ver_info.product_id, 4)) {
		GTP_INFO("Product id is not match!");
		return ERROR_CHECK;
	}
	if ((fw_ver_info.mask_id & 0xFFFFFF00) != (ver_info.mask_id & 0xFFFFFF00)) {
		GTP_INFO("Mask id is not match!");
		return ERROR_CHECK;
	}
	if ((fw_ver_info.patch_id & 0xFF0000) != (ver_info.patch_id & 0xFF0000)){
		GTP_INFO("CID is not equal, need update!");
		return 0;
	}

	if ((fw_ver_info.patch_id & 0xFFFF) == (ver_info.patch_id & 0xFFFF)) {
		GTP_INFO("The version of the fw is not high than the IC's!");
		return ERROR_CHECK;
	}
	return 0;
}

int __pad_gt1x_hold_ss51_dsp_20(void)
{
	int ret = -1;
	int retry = 0;
	u8 buf[1];
	int hold_times = 0;

	while (retry++ < 30) {
		// Hold ss51 & dsp
		buf[0] = 0x0C;
		ret = pad_goodix_i2c_write(_rRW_MISCTL__SWRST_B0_, buf, 1);
		if (ret) {
			GTP_ERROR("Hold ss51 & dsp I2C error,retry:%d", retry);
			continue;
		}
		// Confirm hold
		buf[0] = 0x00;
		ret = pad_goodix_i2c_read(_rRW_MISCTL__SWRST_B0_, buf, 1);
		if (ret) {
			GTP_ERROR("Hold ss51 & dsp I2C error,retry:%d", retry);
			continue;
		}
		if (0x0C == buf[0]) {
			if (hold_times++ < 20) {
				continue;
			} else {
				break;
			}
		}
		GTP_ERROR("Hold ss51 & dsp confirm 0x4180 failed,value:%d", buf[0]);
	}
	if (retry >= 30) {
		GTP_ERROR("Hold ss51&dsp failed!");
		return ERROR_RETRY;
	}

	GTP_INFO("Hold ss51&dsp successfully.");
	return 0;
}

int pad_gt1x_hold_ss51_dsp(void)
{
	int ret = ERROR, retry = 5;
	u8 buffer[2];

	do {
		goodix_select_addr();
		usleep_range(20000, 20010);
		ret =  pad_goodix_i2c_read(0x4220, buffer, 1);
	} while (retry-- && ret < 0);

	if (ret < 0)
		return ERROR;

	//hold ss51_dsp
	ret = __pad_gt1x_hold_ss51_dsp_20();
	if (ret) {
		return ret;
	}
	// enable dsp & mcu power
	buffer[0] = 0x00;
	ret = pad_gt1x_i2c_write_with_readback(_bRW_MISCTL__DSP_MCU_PWR_, buffer, 1);
	if (ret) {
		GTP_ERROR("enabel dsp & mcu power fail!");
		return ret;
	}
	// disable watchdog
	buffer[0] = 0x00;
	ret = pad_gt1x_i2c_write_with_readback(_bRW_MISCTL__TMR0_EN, buffer, 1);
	if (ret) {
		GTP_ERROR("disable wdt fail!");
		return ret;
	}
	// clear cache
	buffer[0] = 0x00;
	ret = pad_gt1x_i2c_write_with_readback(_bRW_MISCTL__CACHE_EN, buffer, 1);
	if (ret) {
		GTP_ERROR("clear cache fail!");
		return ret;
	}
	// soft reset
	buffer[0] = 0x01;
	ret = pad_goodix_i2c_write(_bWO_MISCTL__CPU_SWRST_PULSE, buffer, 1);
	if (ret) {
		GTP_ERROR("software reset fail!");
		return ret;
	}
	// set scramble
	buffer[0] = 0x00;
	ret = pad_gt1x_i2c_write_with_readback(_rRW_MISCTL__BOOT_OPT_B0_, buffer, 1);
	if (ret) {
		GTP_ERROR("set scramble fail!");
		return ret;
	}

	return 0;
}

int pad_gt1x_run_ss51_isp(u8 * ss51_isp, int length)
{
	int ret;
	u8 buffer[10];

	ret = pad_gt1x_hold_ss51_dsp();
	if (ret) {
		return ret;
	}
	// select bank4
	buffer[0] = 0x04;
	ret = pad_gt1x_i2c_write_with_readback(_bRW_MISCTL__SRAM_BANK, buffer, 1);
	if (ret) {
		GTP_ERROR("select bank4 fail.");
		return ret;
	}
	// enable patch area access
	buffer[0] = 0x01;
	ret = pad_gt1x_i2c_write_with_readback(_bRW_MISCTL__PATCH_AREA_EN_, buffer, 1);
	if (ret) {
		GTP_ERROR("enable patch area access fail!");
		return ret;
	}

	GTP_INFO("ss51_isp length: %d, checksum: 0x%04X", length, pad_gt1x_calc_checksum(ss51_isp, length));
	// load ss51 isp
	ret = pad_goodix_i2c_write(0xC000, ss51_isp, length);
	if (ret) {
		GTP_ERROR("load ss51 isp fail!");
		return ret;
	}
	// recall compare
	ret = pad_gt1x_recall_check(ss51_isp, 0xC000, length);
	if (ret) {
		GTP_ERROR("recall check ss51 isp fail!");
		return ret;
	}

	memset(buffer, 0xAA, 10);
	ret = pad_gt1x_i2c_write_with_readback(0x8140, buffer, 10);

	// disable patch area access
	buffer[0] = 0x00;
	ret = pad_gt1x_i2c_write_with_readback(_bRW_MISCTL__PATCH_AREA_EN_, buffer, 1);
	if (ret) {
		GTP_ERROR("disable patch area access fail!");
		return ret;
	}
	// set 0x8006
	memset(buffer, 0x55, 8);
	ret = pad_gt1x_i2c_write_with_readback(0x8006, buffer, 8);
	if (ret) {
		GTP_ERROR("set 0x8006[0~7] 0x55 fail!");
		return ret;
	}
	// release ss51
	buffer[0] = 0x08;
	ret = pad_gt1x_i2c_write_with_readback(_rRW_MISCTL__SWRST_B0_, buffer, 1);
	if (ret) {
		GTP_ERROR("release ss51 fail!");
		return ret;
	}

	msleep(100);
	// check run state
	ret = pad_goodix_i2c_read(0x8006, buffer, 2);
	if (ret) {
		GTP_ERROR("read 0x8006 fail!");
		return ret;
	}
	if (!(buffer[0] == 0xAA && buffer[1] == 0xBB)) {
		GTP_ERROR("ERROR: isp is not running! 0x8006: %02X %02X", buffer[0], buffer[1]);
		return ERROR_CHECK;
	}

	return 0;
}

u16 pad_gt1x_calc_checksum(u8 * fw, u32 length)
{
	u32 i = 0;
	u32 checksum = 0;

	for (i = 0; i < length; i += 2) {
		checksum += (((int)fw[i]) << 8);
		checksum += fw[i + 1];
	}
	return (checksum & 0xFFFF);
}

int pad_gt1x_recall_check(u8 * chk_src, u16 start_addr, u16 chk_length)
{
	u8 rd_buf[PACK_SIZE];
	s32 ret = 0;
	u16 len = 0;
	u32 compared_length = 0;

	while (chk_length > 0) {
		len = (chk_length > PACK_SIZE ? PACK_SIZE : chk_length);

		ret = pad_goodix_i2c_read(start_addr + compared_length, rd_buf, len);
		if (ret) {
			GTP_ERROR("recall i2c error,exit!");
			return ret;
		}

		if (memcmp(rd_buf, &chk_src[compared_length], len)) {
			GTP_ERROR("Recall frame not equal(addr: 0x%04X)",
					start_addr + compared_length);
			return ERROR_CHECK;
		}

		chk_length -= len;
		compared_length += len;
	}

	GTP_DEBUG("Recall check %d bytes(address: 0x%04X) success.", compared_length, start_addr);
	return 0;
}

int pad_gt1x_burn_subsystem(struct fw_subsystem_info *subsystem)
{
	int block_len;
	u16 checksum;
	int burn_len = 0;
	u16 cur_addr;
	u32 length = subsystem->length;
	u8 buffer[10];
	int ret;
	int wait_time;
	int burn_state;
	int retry = 5;
	u8 *fw;

	GTP_INFO("Subsystem: %d", subsystem->type);
	GTP_INFO("Length: %d", subsystem->length);
	GTP_INFO("Address: 0x%08X", subsystem->address);

	while (length > 0 && retry > 0) {
		retry--;

		block_len = length > 1024 * 4 ? 1024 * 4 : length;

		GTP_INFO("Burn block ==> length: %d, address: 0x%08X", block_len, subsystem->address + burn_len);
		fw = pad_gt1x_get_fw_data(subsystem->offset + burn_len, block_len);
		if (fw == NULL) {
			return ERROR_FW;
		}

		cur_addr = ((subsystem->address + burn_len) >> 8);

		checksum = 0;
		checksum += block_len;
		checksum += cur_addr;
		checksum += pad_gt1x_calc_checksum(fw, block_len);
		checksum = (0 - checksum);

		buffer[0] = ((block_len >> 8) & 0xFF);
		buffer[1] = (block_len & 0xFF);
		buffer[2] = ((cur_addr >> 8) & 0xFF);
		buffer[3] = (cur_addr & 0xFF);

		ret = pad_gt1x_i2c_write_with_readback(0x8100, buffer, 4);
		if (ret) {
			GTP_ERROR("write length & address fail!");
			continue;
		}

		ret = pad_goodix_i2c_write(0x8100 + 4, fw, block_len);
		if (ret) {
			GTP_ERROR("write fw data fail!");
			continue;
		}

		buffer[0] = ((checksum >> 8) & 0xFF);
		buffer[1] = (checksum & 0xFF);
		ret = pad_gt1x_i2c_write_with_readback(0x8100 + 4 + block_len, buffer, 2);
		if (ret) {
			GTP_ERROR("write checksum fail!");
			continue;
		}

		buffer[0] = 0;
		ret = pad_gt1x_i2c_write_with_readback(0x8022, buffer, 1);
		if (ret) {
			GTP_ERROR("clear control flag fail!");
			continue;
		}

		buffer[0] = subsystem->type;
		buffer[1] = subsystem->type;
		ret = pad_gt1x_i2c_write_with_readback(0x8020, buffer, 2);
		if (ret) {
			GTP_ERROR("write subsystem type fail!");
			continue;
		}
		burn_state = ERROR;
		wait_time = 200;
		msleep(5);

		while (wait_time-- > 0) {
			u8 confirm = 0x55;

			ret = pad_goodix_i2c_read(0x8022, buffer, 1);
			if (ret < 0) {
				continue;
			}
			msleep(5);
			ret = pad_goodix_i2c_read(0x8022, &confirm, 1);
			if (ret < 0) {
				continue;
			}
			if (buffer[0] != confirm) {
				continue;
			}

			if (buffer[0] == 0xAA) {
				GTP_DEBUG("burning.....");
				continue;
			} else if (buffer[0] == 0xDD) {
				GTP_ERROR("checksum error!");
				break;
			} else if (buffer[0] == 0xBB) {
				GTP_INFO("burning success.");
				burn_state = 0;
				break;
			} else if (buffer[0] == 0xCC) {
				GTP_ERROR("burning failed!");
				break;
			} else {
				GTP_DEBUG("unknown state!(0x8022: 0x%02X)", buffer[0]);
			}
		}

		if (!burn_state) {
			length -= block_len;
			burn_len += block_len;
			retry = 5;
		}
	}
	if (length == 0) {
		return 0;
	} else {
		return ERROR_RETRY;
	}
}

int pad_gt1x_check_subsystem_in_flash(struct fw_subsystem_info *subsystem)
{
	int block_len;
	int checked_len = 0;
	u32 length = subsystem->length;
	int ret;
	int check_state = 0;
	int retry = 5;
	u8 *fw;

	GTP_INFO("Subsystem: %d", subsystem->type);
	GTP_INFO("Length: %d", subsystem->length);
	GTP_INFO("Address: 0x%08X", subsystem->address);

	while (length > 0) {
		block_len = length > 1024 * 4 ? 1024 * 4 : length;

		GTP_INFO("Check block ==> length: %d, address: 0x%08X", block_len, subsystem->address + checked_len);
		fw = pad_gt1x_get_fw_data(subsystem->offset + checked_len, block_len);
		if (fw == NULL) {
			return ERROR_FW;
		}
		ret = pad_gt1x_read_flash(subsystem->address + checked_len, block_len);
		if (ret) {
			check_state |= ret;
		}

		ret = pad_gt1x_recall_check(fw, 0x8100, block_len);
		if (ret) {
			GTP_ERROR("Block in flash is broken!");
			check_state |= ret;
		}

		length -= block_len;
		checked_len += block_len;
		retry = 5;
	}
	if (check_state) {
		GTP_ERROR("Subsystem in flash is broken!");
	} else {
		GTP_INFO("Subsystem in flash is correct!");
	}
	return check_state;
}

int pad_gt1x_read_flash(u32 addr, int length)
{
	int wait_time;
	int ret = 0;
	u8 buffer[4];
	u16 read_addr = (addr >> 8);

	GTP_INFO("Read flash: 0x%04X, length: %d", addr, length);

	buffer[0] = 0;
	ret = pad_gt1x_i2c_write_with_readback(0x8022, buffer, 1);

	buffer[0] = ((length >> 8) & 0xFF);
	buffer[1] = (length & 0xFF);
	buffer[2] = ((read_addr >> 8) & 0xFF);
	buffer[3] = (read_addr & 0xFF);
	ret |= pad_gt1x_i2c_write_with_readback(0x8100, buffer, 4);

	buffer[0] = 0xAA;
	buffer[1] = 0xAA;
	ret |= pad_goodix_i2c_write(0x8020, buffer, 2);
	if (ret) {
		GTP_ERROR("Error occured.");	//comment
		return ret;
	}

	wait_time = 200;
	while (wait_time > 0) {
		wait_time--;
		msleep(5);
		ret = pad_goodix_i2c_read_dbl_check(0x8022, buffer, 1);
		if (ret) {
			continue;
		}
		if (buffer[0] == 0xBB) {
			GTP_INFO("Read success(addr: 0x%04X, length: %d)", addr, length);
			break;
		}
	}
	if (wait_time == 0) {
		GTP_ERROR("Read Flash FAIL!");
		return ERROR_RETRY;
	}
	return 0;
}


int pad_gt1x_error_erase(void)
{
	int block_len;
	u16 checksum;
	u16 erase_addr;
	u8 buffer[10];
	int ret;
	int wait_time;
	int burn_state = ERROR;
	int retry = 5;
	u8 *fw = NULL;

	GTP_INFO("Erase flash area of ss51.");

	pad_goodix_chip_reset();

	fw = pad_gt1x_get_fw_data(pad_update_info.firmware_info->subsystem[0].offset,
		pad_update_info.firmware_info->subsystem[0].length);
	if (fw == NULL) {
		GTP_ERROR("get isp fail");
		return ERROR_FW;
	}
	ret = pad_gt1x_run_ss51_isp(fw, pad_update_info.firmware_info->subsystem[0].length);
	if (ret) {
		GTP_ERROR("run isp fail");
		return ERROR_PATH;
	}

	fw = kmalloc(1024 * 4, GFP_KERNEL);
	if (!fw) {
		GTP_ERROR("error when alloc mem.");
		return ERROR_MEM;
	}

	memset(fw, 0xFF, 1024 * 4);
	erase_addr = 0x00;
	block_len = 1024 * 4;

	while (retry-- > 0) {

		checksum = 0;
		checksum += block_len;
		checksum += erase_addr;
		checksum += pad_gt1x_calc_checksum(fw, block_len);
		checksum = (0 - checksum);

		buffer[0] = ((block_len >> 8) & 0xFF);
		buffer[1] = (block_len & 0xFF);
		buffer[2] = ((erase_addr >> 8) & 0xFF);
		buffer[3] = (erase_addr & 0xFF);

		ret = pad_gt1x_i2c_write_with_readback(0x8100, buffer, 4);
		if (ret) {
			GTP_ERROR("write length & address fail!");
			continue;
		}

		ret = pad_goodix_i2c_write(0x8100 + 4, fw, block_len);
		if (ret) {
			GTP_ERROR("write fw data fail!");
			continue;
		}

		ret = pad_gt1x_recall_check(fw, 0x8100 + 4, block_len);
		if (ret) {
			continue;
		}

		buffer[0] = ((checksum >> 8) & 0xFF);
		buffer[1] = (checksum & 0xFF);
		ret = pad_gt1x_i2c_write_with_readback(0x8100 + 4 + block_len, buffer, 2);
		if (ret) {
			GTP_ERROR("write checksum fail!");
			continue;
		}

		buffer[0] = 0;
		ret = pad_gt1x_i2c_write_with_readback(0x8022, buffer, 1);
		if (ret) {
			GTP_ERROR("clear control flag fail!");
			continue;
		}

		buffer[0] = FW_SECTION_TYPE_SS51_PATCH;
		buffer[1] = FW_SECTION_TYPE_SS51_PATCH;
		ret = pad_gt1x_i2c_write_with_readback(0x8020, buffer, 2);
		if (ret) {
			GTP_ERROR("write subsystem type fail!");
			continue;
		}
		burn_state = ERROR;
		wait_time = 200;
		while (wait_time > 0) {
			wait_time--;
			msleep(5);
			ret = pad_goodix_i2c_read_dbl_check(0x8022, buffer, 1);
			if (ret) {
				continue;
			}

			if (buffer[0] == 0xAA) {
				GTP_DEBUG("burning.....");
				continue;
			} else if (buffer[0] == 0xDD) {
				GTP_ERROR("checksum error!");
				break;
			} else if (buffer[0] == 0xBB) {
				GTP_INFO("burning success.");
				burn_state = 0;
				break;
			} else if (buffer[0] == 0xCC) {
				GTP_ERROR("burning failed!");
				break;
			} else {
				GTP_DEBUG("unknown state!(0x8022: 0x%02X)", buffer[0]);
			}
		}
	}

	kfree(fw);
	if (burn_state == 0) {
		return 0;
	} else {
		return ERROR_RETRY;
	}
}

void pad_gt1x_leave_update_mode(void)
{
	GTP_DEBUG("Leave FW update mode.");
	if (pad_update_info.status != UPDATE_STATUS_ABORT) {
		if (pad_goodix_ts) {
			pad_goodix_ts->ops.chip_reset();
			pad_goodix_ts->noise_env = false;
			pad_goodix_ts->ops.feature_resume(pad_goodix_ts);
		}
	}
	pad_update_info.status = UPDATE_STATUS_IDLE;

	// debug+
	//gt1x_irq_enable();
}

void pad_dump_to_file(u16 addr, int length, char *filepath)
{
	struct file *flp = NULL;
	u8 buf[128];
	const int READ_BLOCK_SIZE = 128;
	int read_length = 0;
	int len = 0;

	GTP_INFO("Dump(0x%04X, %d bytes) to file: %s\n", addr, length, filepath);
	flp = filp_open(filepath, O_RDWR | O_CREAT, 0666);
	if (IS_ERR(flp)) {
		GTP_ERROR("can not open file: %s\n", filepath);
		return;
	}
	flp->f_op->llseek(flp, 0, SEEK_SET);

	while (length > 0) {
		len = (length > READ_BLOCK_SIZE ? READ_BLOCK_SIZE : length);
		memset(buf, 0x33, len);
		if (pad_goodix_i2c_read(addr + read_length, buf, len)) {
			memset(buf, 0x33, len);
		}
		flp->f_op->write(flp, (char *)buf, len, &flp->f_pos);
		read_length += len;
		length -= len;
	}
	filp_close(flp, NULL);
}

int pad_gt1x_hold_ss51_dsp_no_reset(void)
{
	int ret = ERROR;
	u8 buffer[2];

	//hold ss51_dsp
	ret = __pad_gt1x_hold_ss51_dsp_20();
	if (ret) {
		return ret;
	}
	// enable dsp & mcu power
	buffer[0] = 0x00;
	ret = pad_gt1x_i2c_write_with_readback(_bRW_MISCTL__DSP_MCU_PWR_, buffer, 1);
	if (ret) {
		GTP_ERROR("enabel dsp & mcu power fail!");
		return ret;
	}
	// disable watchdog
	buffer[0] = 0x00;
	ret = pad_gt1x_i2c_write_with_readback(_bRW_MISCTL__TMR0_EN, buffer, 1);
	if (ret) {
		GTP_ERROR("disable wdt fail!");
		return ret;
	}
	// clear cache
	buffer[0] = 0x00;
	ret = pad_gt1x_i2c_write_with_readback(_bRW_MISCTL__CACHE_EN, buffer, 1);
	if (ret) {
		GTP_ERROR("clear cache fail!");
		return ret;
	}
	// soft reset
	buffer[0] = 0x01;
	ret = pad_goodix_i2c_write(_bWO_MISCTL__CPU_SWRST_PULSE, buffer, 1);
	if (ret) {
		GTP_ERROR("software reset fail!");
		return ret;
	}
	// set scramble
	buffer[0] = 0x00;
	ret = pad_gt1x_i2c_write_with_readback(_rRW_MISCTL__BOOT_OPT_B0_, buffer, 1);
	if (ret) {
		GTP_ERROR("set scramble fail!");
		return ret;
	}

	return 0;
}

#define GT1X_LOAD_PACKET_SIZE (1024 * 2)

int pad_gt1x_load_patch(u8 * patch, u32 patch_size, int offset, int bank_size)
{
	s32 loaded_length = 0;
	s32 len = 0;
	s32 ret = 0;
	u8 bank = 0, tmp;
	u16 address;

	GTP_INFO("Load patch code(size: %d, checksum: 0x%04X, position: 0x%04X, bank-size: %d", patch_size, pad_gt1x_calc_checksum(patch, patch_size), 0xC000 + offset, bank_size);
	while (loaded_length != patch_size) {
		if (loaded_length == 0 || (loaded_length + offset) % bank_size == 0) {
			// select bank
			bank = 0x04 + (loaded_length + offset) / bank_size;
			ret = pad_goodix_i2c_write(_bRW_MISCTL__SRAM_BANK, &bank, 1);
			if (ret) {
				GTP_ERROR("select bank%d fail!", bank);
				return ret;
			}
			GTP_INFO("Select bank%d success.", bank);
			// enable patch area access
			tmp = 0x01;
			ret = pad_gt1x_i2c_write_with_readback(_bRW_MISCTL__PATCH_AREA_EN_ + bank - 4, &tmp, 1);
			if (ret) {
				GTP_ERROR("enable patch area access fail!");
				return ret;
			}
		}

		len = patch_size - loaded_length > GT1X_LOAD_PACKET_SIZE ? GT1X_LOAD_PACKET_SIZE : patch_size - loaded_length;
		address = 0xC000 + (loaded_length + offset) % bank_size;

		ret = pad_goodix_i2c_write(address, &patch[loaded_length], len);
		if (ret) {
			GTP_ERROR("load 0x%04X, %dbytes fail!", address, len);
			return ret;
		}
		ret = pad_gt1x_recall_check(&patch[loaded_length], address, len);
		if (ret) {
			GTP_ERROR("Recall check 0x%04X, %dbytes fail!", address, len);
			return ret;
		}
		GTP_INFO("load code 0x%04X, %dbytes success.", address, len);

		loaded_length += len;
	}

	return 0;
}

int pad_gt1x_startup_patch(void)
{
	s32 ret = 0;
	u8 buffer[8] = { 0x55 };
	buffer[0] = 0x00;
	buffer[1] = 0x00;
	ret |= pad_goodix_i2c_write(_bRW_MISCTL__PATCH_AREA_EN_, buffer, 2);

	memset(buffer, 0x55, 8);
	ret |= pad_goodix_i2c_write(GTP_REG_FLASH_PASSBY, buffer, 8);
	ret |= pad_goodix_i2c_write(GTP_REG_VERSION, buffer, 5);

	buffer[0] = 0xAA;
	ret |= pad_goodix_i2c_write(GTP_REG_CMD, buffer, 1);
	ret |= pad_goodix_i2c_write(GTP_REG_ESD_CHECK, buffer, 1);

	buffer[0] = 0x00;
	ret |= pad_goodix_i2c_write(_rRW_MISCTL__SWRST_B0_, buffer, 1);

	msleep(200);

	return ret;
}
