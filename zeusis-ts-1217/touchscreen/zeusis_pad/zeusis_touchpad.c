/*
 * Zeusis Touchscreen Driver
 *
 * Copyright (C) 2013 Zeusis Device Co.Ltd
 * License terms: GNU General Public License (GPL) version 2
 *
 */
#include <linux/module.h>
#include <linux/init.h>
#include <linux/ctype.h>
#include <linux/delay.h>
#include <linux/input/mt.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/debugfs.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/regulator/consumer.h>
#include <linux/string.h>
#include <linux/of_gpio.h>
#include <linux/kthread.h>
#include <linux/uaccess.h>
#include <linux/sched/rt.h>
#include <linux/fb.h>
#include <linux/i2c.h>
#if defined(CONFIG_FB)
#include <linux/notifier.h>
#include <linux/fb.h>
#elif defined(CONFIG_HAS_EARLYSUSPEND)
#include <linux/earlysuspend.h>
#endif
#include "zeusis_touchpad_chips.h"
#include "zeusis_touchpad_algo.h"
#ifdef CONFIG_ZEUSIS_HW_DEV_DCT
#include <linux/hw_dev_dec.h>
#endif
//#include <linux/mfd/hisi_hi6xxx_pmic.h>
//#include <linux/hisi/hi6xxx-lcd_type.h>

#if defined (CONFIG_ZEUSIS_DSM)
#include <zeusis_platform/dsm/dsm_pub.h>

#define LDO17_PHYS_ADDR 	(0X93)
#define LSW50_PHYS_ADDR	(0xAC)
#endif

//#define DATA_SIZE	(4)
#define MAX_LOTUS_NUM  6
static struct mutex  easy_wake_guesure_lock;
#if defined (CONFIG_ZEUSIS_DSM)
static struct dsm_dev dsm_tp = {
	.name = "dsm_tp",
	.device_name = "TP",
	.ic_name = "syn", //just for testing, waiting for the module owner revised
	.module_name = "NNN",
	.fops = NULL,
	.buff_size = 1024,
};

struct dsm_client *tp_dclient = NULL;

static unsigned char LDO17_status = 0;
static unsigned char LSW50_status = 0;
#endif

struct ts_data g_tp_data;

u8 g_tp_log_cfg = 0;
bool pad_gesture_func = false;

static struct ts_cmd_node ping_cmd_buff;
static struct ts_cmd_node pang_cmd_buff;

/*external variable declare*/

/*global variable declare*/

/*external function declare*/
extern int i2c_check_addr_busy(struct i2c_adapter *adapter, int addr);
//extern unsigned int get_boot_into_recovery_flag(void);
static void tp_stop_wd_timer(struct ts_data *cd);
void pad_ts_thread_stop_notify(void);

static struct ts_device_data g_ts_device_map[] =
{
#ifdef CONFIG_TOUCHSCREEN_SYNAPTICS_PAD
	[0]	=   {
		.chip_name = "synaptics_pad",
		.irq_gpio = TS_IO_UNDEFINE, //must be defined in dts/boardid
		.irq_config = TS_IRQ_CFG_UNDEFINE,//must be defined in dts/boardid
		.reset_gpio = TS_IO_UNDEFINE,//must be defined in dts/boardid
		.ops = &pad_ts_synaptics_ops,
	},
#endif
#ifdef CONFIG_CYPRESS_PAD
	[1]	=	{
		.chip_name = "cypress",
		.irq_gpio = TS_IO_UNDEFINE,
		.irq_config = TS_IRQ_CFG_UNDEFINE,
		.reset_gpio = TS_IO_UNDEFINE,
		.ops = &ts_cypress_ops,
	},
#endif
#ifdef CONFIG_WACOM_PAD
	[2]	=	{
		.chip_name = "wacom",
		.irq_gpio = TS_IO_UNDEFINE,
		.irq_config = TS_IRQ_CFG_UNDEFINE,
		.reset_gpio = TS_IO_UNDEFINE,
		.ops = &ts_wacom_ops,
	},
#endif
#ifdef CONFIG_HIDEEP_PAD
	[3]	=	{
		.chip_name = "hideep",
		.irq_gpio = TS_IO_UNDEFINE,
		.irq_config = TS_IRQ_CFG_UNDEFINE,
		.reset_gpio = TS_IO_UNDEFINE,
		.ops = &ts_hideep_ops,
	},
#endif
#ifdef CONFIG_TOUCHSCREEN_GOODIX_PAD
	[4]	=   {
		.chip_name = "goodix_pad",
		.irq_gpio = TS_IO_UNDEFINE, //must be defined in dts/boardid
		.irq_config = TS_IRQ_CFG_UNDEFINE,//must be defined in dts/boardid
		.reset_gpio = TS_IO_UNDEFINE,//must be defined in dts/boardid
		.ops = &pad_ts_goodix_ops,
	},
#endif
};

static int tp_i2c_write(u8 *buf, u16 length)
{
	int count = 0;
	int ret;

	do {
		ret = i2c_master_send(g_tp_data.client, buf, length);
		if (ret == length) {
			return NO_ERR;
		}

#if defined (CONFIG_ZEUSIS_DSM)
		else
			g_tp_data.dsm_info.constraints_I2C_status = ret;
#endif

		msleep(I2C_WAIT_TIME);
	} while (++count < I2C_RW_TRIES);

#if defined (CONFIG_ZEUSIS_DSM)
	LDO17_status = 0;//hi6xxx_pmic_reg_read(LDO17_PHYS_ADDR);
	LSW50_status = 0;//hi6xxx_pmic_reg_read(LSW50_PHYS_ADDR);
	if(!dsm_client_ocuppy(tp_dclient)){
		dsm_client_record(tp_dclient, "irq_gpio:%d\tvalue:%d.\n\
reset_gpio:%d\t value:%d.\nLDO17_status is 0x%x.\nLSW50_status is 0x%x\nI2C_status:%d.\n",\
			g_tp_data.chip_data->irq_gpio,\
			gpio_get_value(g_tp_data.chip_data->irq_gpio),\
			g_tp_data.chip_data->reset_gpio,\
			gpio_get_value(g_tp_data.chip_data->reset_gpio),\
			LDO17_status, LSW50_status, g_tp_data.dsm_info.constraints_I2C_status);
		dsm_client_notify(tp_dclient, DSM_TP_I2C_RW_ERROR_NO);
	}
#endif
	TP_LOG_ERR("ts_i2c_write failed\n");
	return -EIO;
}

static int tp_spi_write(u8 *buf, u16 length)
{
	return NO_ERR;
}

static int tp_i2c_read(u8 *reg_addr, u16 reg_len, u8 *buf, u16 len)
{
	int count = 0;
	int ret;
	struct i2c_msg xfer[2];

	/*register addr*/
	xfer[0].addr = g_tp_data.client->addr;
	xfer[0].flags = 0;
	xfer[0].len = reg_len;
	xfer[0].buf = reg_addr;

	/* Read data */
	xfer[1].addr = g_tp_data.client->addr;
	xfer[1].flags = I2C_M_RD;
	xfer[1].len = len;
	xfer[1].buf = buf;

	do {
		ret = i2c_transfer(g_tp_data.client->adapter, xfer, 2);
		if (ret == 2) {
			return NO_ERR;
		}

#if defined (CONFIG_ZEUSIS_DSM)
		else
			g_tp_data.dsm_info.constraints_I2C_status = ret;
#endif

		msleep(I2C_WAIT_TIME);
	} while (++count < I2C_RW_TRIES);

#if defined (CONFIG_ZEUSIS_DSM)
	LDO17_status = 0;//hi6xxx_pmic_reg_read(LDO17_PHYS_ADDR);
	LSW50_status = 0;//hi6xxx_pmic_reg_read(LSW50_PHYS_ADDR);
	if(!dsm_client_ocuppy(tp_dclient)){
		dsm_client_record(tp_dclient, "irq_gpio:%d\tvalue:%d.\n\
reset_gpio:%d\t value:%d.\nLDO17_status is 0x%x.\nLSW50_status is 0x%x\nI2C_status:%d.\n",\
			g_tp_data.chip_data->irq_gpio,\
			gpio_get_value(g_tp_data.chip_data->irq_gpio),\
			g_tp_data.chip_data->reset_gpio,\
			gpio_get_value(g_tp_data.chip_data->reset_gpio),\
			LDO17_status, LSW50_status, g_tp_data.dsm_info.constraints_I2C_status);
		dsm_client_notify(tp_dclient, DSM_TP_I2C_RW_ERROR_NO);
	}
#endif
	TP_LOG_ERR("ts_i2c_read failed\n");
	return -EIO;
}

static int tp_spi_read(u8 *reg_addr, u16 reg_len, u8 *buf, u16 len)
{
	return NO_ERR;
}

static struct ts_bus_info tp_bus_i2c_info = {
	.btype		= TS_BUS_I2C,
	.bus_write	= tp_i2c_write,
	.bus_read	= tp_i2c_read,
};

static struct ts_bus_info tp_bus_spi_info = {
	.btype		= TS_BUS_SPI,
	.bus_write	= tp_spi_write,
	.bus_read	= tp_spi_read,
};

int pad_register_algo_func(struct ts_device_data *chip_data, struct ts_algo_func *fn)
{
	int error = -EIO;

	if (!chip_data ||!fn)
		goto out;

	fn->algo_index = chip_data->algo_size;
	list_add_tail( &fn->node, &chip_data->algo_head);
	chip_data->algo_size++;
	smp_mb();
	error = NO_ERR;

out:
	return error;
}

int pad_put_one_cmd(struct ts_cmd_node *cmd, int timeout)
{
	int error = -EIO;
	unsigned long flags;
	struct ts_cmd_queue *q;
	struct ts_cmd_sync *sync = NULL;

	if (!cmd) {
		TP_LOG_ERR("find null pointer\n");
		goto out;
	}

	if(TS_UNINIT == atomic_read(&g_tp_data.state)){
		TP_LOG_ERR("ts module not initialize\n");
		goto out;
	}

	if (timeout) {
		sync = (struct ts_cmd_sync *)kzalloc(sizeof(struct ts_cmd_sync), GFP_KERNEL);
		if (NULL == sync) {
			TP_LOG_ERR("failed to kzalloc completion\n");
			error = -ENOMEM;
			goto out;
		}
		init_completion(&sync->done);
		atomic_set(&sync->timeout_flag, TS_NOT_TIMEOUT);
		cmd->sync = sync;
	} else {
		cmd->sync = NULL;
	}

	q = &g_tp_data.queue;

	spin_lock_irqsave(&q->spin_lock, flags);
	smp_wmb();
	if (q->cmd_count == q->queue_size) {
		spin_unlock_irqrestore(&q->spin_lock, flags);
		TP_LOG_ERR("queue is full\n");
		BUG_ON(1);
		error = -EIO;
		goto free_sync;
	}
	memcpy(&q->ring_buff[q->wr_index], cmd, sizeof(struct ts_cmd_node));
	q->cmd_count++;
	q->wr_index++;
	q->wr_index %= q->queue_size;
	smp_mb();
	spin_unlock_irqrestore(&q->spin_lock, flags);
	TP_LOG_DEBUG("put one cmd :%d in ring buff\n", cmd->command);
	error = NO_ERR;
	wake_up_process(g_tp_data.ts_task); //wakeup process

	if (timeout && !(wait_for_completion_timeout(&sync->done, abs(timeout)*HZ))) {
		atomic_set(&sync->timeout_flag, TS_TIMEOUT);
		TP_LOG_ERR("wait for cmd respone timeout\n");
		error = -EBUSY;
		goto out;
	}
	smp_wmb();

free_sync:
	if (sync) {
		kfree(sync);
	}
out:
	return error;
}

static int pad_get_one_cmd(struct ts_cmd_node *cmd)
{
	unsigned long flags;
	int error = -EIO;
	struct ts_cmd_queue *q;

	if (unlikely(!cmd)) {
		TP_LOG_ERR("find null pointer\n");
		goto out;
	}

	q = &g_tp_data.queue;

	spin_lock_irqsave(&q->spin_lock, flags);
	smp_wmb();
	if (!q->cmd_count) {
		TP_LOG_DEBUG("queue is empty\n");
		spin_unlock_irqrestore(&q->spin_lock, flags);
		goto out;
	}
	memcpy(cmd,&q->ring_buff[q->rd_index], sizeof(struct ts_cmd_node));
	q->cmd_count--;
	q->rd_index++;
	q->rd_index %= q->queue_size;
	smp_mb();
	spin_unlock_irqrestore(&q->spin_lock, flags);
	TP_LOG_DEBUG("get one cmd :%d from ring buff\n", cmd->command);
	error = NO_ERR;

out:
	return error;
}

ssize_t pad_ts_chip_info_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int error = NO_ERR;
	struct ts_cmd_node *cmd = NULL;
	struct ts_chip_info_param *info = NULL;

	TP_LOG_INFO("pad_ts_chip_info_show called\n");

	if (dev == NULL) {
		TP_LOG_ERR("dev is null\n");
		error = -EINVAL;
		goto out;
	}

	cmd = (struct ts_cmd_node *)kzalloc(sizeof(struct ts_cmd_node), GFP_KERNEL);
	if (!cmd) {
		TP_LOG_ERR("malloc failed\n");
		error = -ENOMEM;
		goto out;
	}

	info = (struct ts_chip_info_param *)kzalloc(sizeof(struct ts_chip_info_param), GFP_KERNEL);
	if (!info) {
		TP_LOG_ERR("malloc failed\n");
		error = -ENOMEM;
		goto out;
	}

	cmd->command = TS_GET_CHIP_INFO;
	cmd->cmd_param.prv_params = (void *)info;

	error = pad_put_one_cmd(cmd, SHORT_SYNC_TIMEOUT);
	if (error) {
		TP_LOG_ERR("put cmd error :%d\n", error);
		error = -EBUSY;
		goto out;
	}

	if (info->status != TS_ACTION_SUCCESS) {
		TP_LOG_ERR("read action failed\n");
		error = -EIO;
		goto out;
	}
	if(g_tp_data.get_info_flag)
	{
		error = snprintf(buf, CHIP_INFO_LENGTH,"%s",info->mod_vendor);
		g_tp_data.get_info_flag =false;
	}
	else
	{
		error = snprintf(buf, CHIP_INFO_LENGTH*2+CHIP_INFO_LENGTH*2+1,
				"%s-%s-%s\n", info->ic_vendor, info->mod_vendor, info->fw_vendor);
	}
out:
	if (cmd)
		kfree(cmd);
	if (info)
		kfree(info);
	TP_LOG_DEBUG("pad_ts_chip_info_show done\n");
	return error;
}
ssize_t pad_ts_chip_info_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned int value;
	int error;
	struct ts_cmd_node *cmd = NULL;
	struct ts_data *info = NULL;

	TP_LOG_INFO("pad_ts_chip_info_store called\n");

	if (dev == NULL) {
		TP_LOG_ERR("dev is null\n");
		error = -EINVAL;
		goto out;
	}

	error = sscanf(buf, "%u", &value);
	if (error <= 0) {
		TP_LOG_ERR("sscanf return invaild :%d\n", error);
		error = -EINVAL;
		goto out;
	}
	TP_LOG_DEBUG("sscanf value is %u\n", value);

	cmd = (struct ts_cmd_node *)kzalloc(sizeof(struct ts_cmd_node), GFP_KERNEL);
	if (!cmd) {
		TP_LOG_ERR("malloc failed\n");
		error = -ENOMEM;
		goto out;
	}
	info = (struct ts_data *)kzalloc(sizeof(struct ts_data), GFP_KERNEL);
	if (!info) {
		TP_LOG_ERR("malloc failed\n");
		error = -ENOMEM;
		goto out;
	}
	info->get_info_flag = value;
	cmd->command = TS_SET_INFO_FLAG;
	cmd->cmd_param.prv_params = (void *)info;
	error = pad_put_one_cmd(cmd, SHORT_SYNC_TIMEOUT);
	if (error) {
		TP_LOG_ERR("put cmd error :%d\n", error);
		error = -EBUSY;
		goto out;
	}
	error = count;
out:
	if (cmd)
		kfree(cmd);
	if (info)
		kfree(info);
	TP_LOG_DEBUG("pad_ts_chip_info_store done\n");
	return error;
}
static ssize_t tp_dsm_debug_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int error = NO_ERR;
	struct ts_cmd_node *cmd = NULL;
	struct ts_dsm_debug_info info;

	TP_LOG_INFO("ts_dsm_debug_show called\n");

	if (dev == NULL) {
		TP_LOG_ERR("dev is null\n");
		error = -EINVAL;
		goto out;
	}

	cmd = (struct ts_cmd_node *)kzalloc(sizeof(struct ts_cmd_node), GFP_KERNEL);
	if (!cmd) {
		TP_LOG_ERR("malloc failed\n");
		error = -ENOMEM;
		goto out;
	}

	cmd->command = TS_DSM_DEBUG;
	cmd->cmd_param.prv_params = (void *)&info;
	error = pad_put_one_cmd(cmd, LONG_SYNC_TIMEOUT);
	if (error) {
		TP_LOG_ERR("put cmd error :%d\n", error);
		goto out;
	}
	if (info.status == TS_ACTION_SUCCESS)
		error = snprintf(buf, MAX_STR_LEN, "%s\n", "success");
	else
		error = snprintf(buf, MAX_STR_LEN, "%s\n", "failed");

out:
	if (cmd)
		kfree(cmd);
	TP_LOG_DEBUG("ts_debug_show done\n");
	return error;
}


static ssize_t tp_calibrate_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int error = NO_ERR;
	struct ts_cmd_node *cmd = NULL;
	struct ts_calibrate_info info;

	TP_LOG_INFO("ts_calibrate_show called\n");

	if (dev == NULL) {
		TP_LOG_ERR("dev is null\n");
		error = -EINVAL;
		goto out;
	}

	cmd = (struct ts_cmd_node *)kzalloc(sizeof(struct ts_cmd_node), GFP_KERNEL);
	if (!cmd) {
		TP_LOG_ERR("malloc failed\n");
		error = -ENOMEM;
		goto out;
	}

	cmd->command = TS_CALIBRATE_DEVICE;
	cmd->cmd_param.prv_params = (void *)&info;
	error = pad_put_one_cmd(cmd, LONG_SYNC_TIMEOUT);
	if (error) {
		TP_LOG_ERR("put cmd error :%d\n", error);
		error = -EBUSY;
		goto out;
	}

	if (info.status == TS_ACTION_SUCCESS)
		error = snprintf(buf, MAX_STR_LEN, "%s\n", "success");
	else
		error = snprintf(buf, MAX_STR_LEN, "%s\n", "failed");

out:
	if (cmd)
		kfree(cmd);
	TP_LOG_DEBUG("ts_calibrate_show done\n");
	return error;
}

static ssize_t tp_glove_mode_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int error = NO_ERR;
	struct ts_cmd_node *cmd = NULL;
	struct ts_glove_info *info = NULL;

	TP_LOG_INFO("ts_glove_mode_show called\n");

	if (dev == NULL) {
		TP_LOG_ERR("dev is null\n");
		error = -EINVAL;
		goto out;
	}

	cmd = (struct ts_cmd_node *)kzalloc(sizeof(struct ts_cmd_node), GFP_KERNEL);
	if (!cmd) {
		TP_LOG_ERR("malloc failed\n");
		error = -ENOMEM;
		goto out;
	}
	info = (struct ts_glove_info *)kzalloc(sizeof(struct ts_glove_info), GFP_KERNEL);
	if (!info) {
		TP_LOG_ERR("malloc failed\n");
		error = -ENOMEM;
		goto out;
	}

	info->op_action = TS_ACTION_READ;
	cmd->command = TS_GLOVE_SWITCH;
	cmd->cmd_param.prv_params = (void *)info;
	error = pad_put_one_cmd(cmd, SHORT_SYNC_TIMEOUT);
	if (error) {
		TP_LOG_ERR("put cmd error :%d\n", error);
		error = -EBUSY;
		goto out;
	}

	if (info->status == TS_ACTION_SUCCESS)
		error = snprintf(buf, MAX_STR_LEN, "%d\n", info->glove_switch);
	else
		error = -EFAULT;

out:
	if (cmd)
		kfree(cmd);
	if (info)
		kfree(info);
	TP_LOG_DEBUG("ts_glove_mode_show done\n");
	return error;
}

static ssize_t tp_glove_mode_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned int value;
	int error;
	struct ts_cmd_node *cmd = NULL;
	struct ts_glove_info *info = NULL;

	TP_LOG_INFO("ts_glove_mode_store called\n");

	if (dev == NULL) {
		TP_LOG_ERR("dev is null\n");
		error = -EINVAL;
		goto out;
	}

	error = sscanf(buf, "%u", &value);
	if (error <= 0) {
		TP_LOG_ERR("sscanf return invaild :%d\n", error);
		error = -EINVAL;
		goto out;
	}
	TP_LOG_DEBUG("sscanf value is %u\n", value);

	cmd = (struct ts_cmd_node *)kzalloc(sizeof(struct ts_cmd_node), GFP_KERNEL);
	if (!cmd) {
		TP_LOG_ERR("malloc failed\n");
		error = -ENOMEM;
		goto out;
	}
	info = (struct ts_glove_info *)kzalloc(sizeof(struct ts_glove_info), GFP_KERNEL);
	if (!info) {
		TP_LOG_ERR("malloc failed\n");
		error = -ENOMEM;
		goto out;
	}

	info->op_action = TS_ACTION_WRITE;
	info->glove_switch = value;
	cmd->command = TS_GLOVE_SWITCH;
	cmd->cmd_param.prv_params = (void *)info;
	error = pad_put_one_cmd(cmd, SHORT_SYNC_TIMEOUT);
	if (error) {
		TP_LOG_ERR("put cmd error :%d\n", error);
		error = -EBUSY;
		goto out;
	}

	if (info->status != TS_ACTION_SUCCESS) {
		TP_LOG_ERR("action failed\n");
		error = -EIO;
		goto out;
	}

	error = count;

out:
	if (cmd)
		kfree(cmd);
	if (info)
		kfree(info);
	TP_LOG_DEBUG("ts_glove_mode_store done\n");
	return error;
}


static ssize_t tp_touch_weight_switch_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int error = NO_ERR;
	struct ts_cmd_node *cmd = NULL;
	struct ts_single_touch_info *info = NULL;

	TP_LOG_INFO("ts_touch_weight_switch_show called\n");

	if (dev == NULL) {
		TP_LOG_ERR("dev is null\n");
		error = -EINVAL;
		goto out;
	}

	cmd = (struct ts_cmd_node *)kzalloc(sizeof(struct ts_cmd_node), GFP_KERNEL);
	if (!cmd) {
		TP_LOG_ERR("malloc failed\n");
		error = -ENOMEM;
		goto out;
	}
	info = (struct ts_single_touch_info *)kzalloc(sizeof(struct ts_single_touch_info), GFP_KERNEL);
	if (!info) {
		TP_LOG_ERR("malloc failed\n");
		error = -ENOMEM;
		goto out;
	}

	info->op_action = TS_ACTION_READ;
	cmd->command = TS_TOUCH_WEIGHT_SWITCH;
	cmd->cmd_param.prv_params = (void *)info;
	error = pad_put_one_cmd(cmd, SHORT_SYNC_TIMEOUT);
	if (error) {
		TP_LOG_ERR("put cmd error :%d\n", error);
		error = -EBUSY;
		goto out;
	}

	if (info->status == TS_ACTION_SUCCESS)
		error = snprintf(buf, MAX_STR_LEN, "%d\n", info->single_touch_switch);
	else
		error = -EFAULT;

out:
	if (cmd)
		kfree(cmd);
	if (info)
		kfree(info);
	TP_LOG_DEBUG("ts_touch_weight_switch_show done\n");
	return error;
}

static ssize_t tp_touch_weight_switch_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned int value;
	int error;
	struct ts_cmd_node *cmd = NULL;
	struct ts_single_touch_info *info = NULL;

	TP_LOG_INFO("ts_touch_weight_switch_store called\n");

	if (dev == NULL) {
		TP_LOG_ERR("dev is null\n");
		error = -EINVAL;
		goto out;
	}

	error = sscanf(buf, "%u", &value);
	if (error <= 0) {
		TP_LOG_ERR("sscanf return invaild :%d\n", error);
		error = -EINVAL;
		goto out;
	}
	TP_LOG_DEBUG("sscanf value is %u\n", value);

	cmd = (struct ts_cmd_node *)kzalloc(sizeof(struct ts_cmd_node), GFP_KERNEL);
	if (!cmd) {
		TP_LOG_ERR("malloc failed\n");
		error = -ENOMEM;
		goto out;
	}
	info = (struct ts_single_touch_info *)kzalloc(sizeof(struct ts_single_touch_info), GFP_KERNEL);
	if (!info) {
		TP_LOG_ERR("malloc failed\n");
		error = -ENOMEM;
		goto out;
	}

	info->op_action = TS_ACTION_WRITE;
	info->single_touch_switch = value;
	cmd->command = TS_TOUCH_WEIGHT_SWITCH;
	cmd->cmd_param.prv_params = (void *)info;
	error = pad_put_one_cmd(cmd, SHORT_SYNC_TIMEOUT);
	if (error) {
		TP_LOG_ERR("put cmd error :%d\n", error);
		error = -EBUSY;
		goto out;
	}

	if (info->status != TS_ACTION_SUCCESS) {
		TP_LOG_ERR("action failed\n");
		error = -EIO;
		goto out;
	}

	error = count;

out:
	if (cmd)
		kfree(cmd);
	if (info)
		kfree(info);
	TP_LOG_DEBUG("ts_touch_weight_switch_store done\n");
	return error;
}

static int tp_send_holster_cmd(void)
{
	int error;
	struct ts_cmd_node cmd;

	TP_LOG_DEBUG("set holster\n");
	cmd.command = TS_HOLSTER_SWITCH;
	cmd.cmd_param.prv_params = (void *)&g_tp_data.feature_info.holster_info;
	error = pad_put_one_cmd(&cmd, SHORT_SYNC_TIMEOUT);
	if (error) {
		TP_LOG_ERR("put cmd error :%d\n", error);
		error = -EBUSY;
		goto out;
	}
	if (g_tp_data.feature_info.holster_info.status != TS_ACTION_SUCCESS) {
		TP_LOG_ERR("action failed\n");
		error = -EIO;
		goto out;
	}

out:
	return error;
}

static int tp_wakeup_gesture_enable_cmd(u8 switch_value)
{
	int error = NO_ERR;
	struct ts_cmd_node cmd;
	struct ts_wakeup_gesture_enable_info *info = &g_tp_data.feature_info.wakeup_gesture_enable_info;

	info->op_action = TS_ACTION_WRITE;
	info->switch_value= switch_value;
	cmd.command = TS_WAKEUP_GESTURE_ENABLE;
	cmd.cmd_param.prv_params = (void *)info;

	if (TS_WORK == atomic_read(&g_tp_data.state)){
		TP_LOG_ERR("can not enable/disable wakeup_gesture when tp is working in normal mode\n");
		error = -EINVAL;
		goto out;
	}

	error = pad_put_one_cmd(&cmd, SHORT_SYNC_TIMEOUT);
	if (error) {
		TP_LOG_ERR("%s: put cmd error :%d\n", __func__, error);
		error = -EBUSY;
		goto out;
	}
	if (g_tp_data.feature_info.wakeup_gesture_enable_info.status != TS_ACTION_SUCCESS) {
		TP_LOG_ERR("%s action failed\n", __func__);
		error = -EIO;
		goto out;
	}

out:
	return error;
}


static int tp_send_roi_cmd(enum ts_action_status read_write_type, int timeout)
{
	int error = NO_ERR;
	struct ts_cmd_node cmd;

	TP_LOG_INFO("ts_send_roi_cmd, read_write_type=%d\n", read_write_type);
	if (g_tp_data.feature_info.roi_info.roi_supported) {
		g_tp_data.feature_info.roi_info.op_action = read_write_type;

		cmd.command = TS_ROI_SWITCH;
		cmd.cmd_param.prv_params = (void *)&g_tp_data.feature_info.roi_info;
		error = pad_put_one_cmd(&cmd, timeout);
		if (error) {
			TP_LOG_ERR("put cmd error :%d\n", error);
			error = -EBUSY;
		}
	}
	return error;
}


static ssize_t tp_touch_window_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int window_enable;
	int x0 = 0;
	int y0 = 0;
	int x1 = 0;
	int y1 = 0;
	int error;
	struct ts_window_info info = {0};
	struct ts_cmd_node cmd;

	error = sscanf(buf, "%d %d %d %d %d",&window_enable, &x0, &y0, &x1, &y1);
	if (error <= 0) {
		TP_LOG_ERR("sscanf return invaild :%d\n", error);
		error = -EINVAL;
		goto out;
	}
	TP_LOG_INFO("sscanf value is %d (%d,%d), (%d,%d)\n", window_enable, x0, y0, x1, y1);
	if (window_enable && ((x0 < 0) || (y0 < 0) || (x1 <= x0) || (y1 <= y0))) {
		TP_LOG_ERR("value is %d (%d,%d), (%d,%d)\n", window_enable, x0, y0, x1, y1);
		error = -EINVAL;
		goto out;
	}

	info.top_left_x0 = x0;
	info.top_left_y0 = y0;
	info.bottom_right_x1 = x1;
	info.bottom_right_y1 = y1;
	info.window_enable = window_enable;

	cmd.command = TS_TOUCH_WINDOW;
	cmd.cmd_param.prv_params = (void *)&info;
	error = pad_put_one_cmd(&cmd, SHORT_SYNC_TIMEOUT);
	if (error) {
		TP_LOG_ERR("put cmd error :%d\n", error);
		error = -EBUSY;
		goto out;
	}
	if (info.status != TS_ACTION_SUCCESS) {
		TP_LOG_ERR("action failed\n");
		error = -EIO;
		goto out;
	}

	error = count;

out:
	TP_LOG_DEBUG("ts_touch_window_store done\n");
	return error;
}

static void tp_check_touch_window(struct ts_fingers *finger)
{
	int id;
	int flag = 0;
	int x0, y0, x1, y1;
	int window_enable;

	window_enable = g_tp_data.feature_info.window_info.window_enable;
	x0 = g_tp_data.feature_info.window_info.top_left_x0;
	y0 = g_tp_data.feature_info.window_info.top_left_y0;
	x1 = g_tp_data.feature_info.window_info.bottom_right_x1;
	y1 = g_tp_data.feature_info.window_info.bottom_right_y1;

	if (0 == window_enable) {
		TP_LOG_DEBUG("no need to part report\n");
		return;
	}

	if (finger->fingers[0].status != TS_FINGER_RELEASE) {
		for (id = 0; id < TP_MAX_FINGER; id++) {
			if (finger->fingers[id].status != 0) {
				if ((finger->fingers[id].x >= x0) && (finger->fingers[id].x <= x1)
					&& (finger->fingers[id].y >= y0) && (finger->fingers[id].y <= y1)) {
					flag = 1;
				} else {
					finger->fingers[id].status = 0;
				}
			}
		}
		if (!flag)
			finger->fingers[0].status = TS_FINGER_RELEASE;
	}
}

static ssize_t tp_sensitivity_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned int value;
	int error;
	struct ts_holster_info *info = &g_tp_data.feature_info.holster_info;

	error = sscanf(buf, "%u", &value);
	if (error <= 0) {
		TP_LOG_ERR("sscanf return invaild :%d\n", error);
		error = -EINVAL;
		goto out;
	}
	TP_LOG_DEBUG("sscanf value is %u\n", value);

	info->op_action = TS_ACTION_WRITE;
	info->holster_switch = value;

	error = tp_send_holster_cmd();
	if (error) {
		TP_LOG_ERR("ts_send_holster_cmd failed\n");
		error = -ENOMEM;
		goto out;
	}

	error = count;

out:
	TP_LOG_DEBUG("ts_sensitivity_store done\n");
	return error;
}

static ssize_t tp_hand_detect_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int error = NO_ERR;
	struct ts_cmd_node *cmd = NULL;
	struct ts_hand_info *info = NULL;

	TP_LOG_INFO("ts_hand_detect_show called\n");

	if (dev == NULL) {
		TP_LOG_ERR("dev is null\n");
		error = -EINVAL;
		goto out;
	}

	cmd = (struct ts_cmd_node *)kzalloc(sizeof(struct ts_cmd_node), GFP_KERNEL);
	if (!cmd) {
		TP_LOG_ERR("malloc failed\n");
		error = -ENOMEM;
		goto out;
	}
	info = (struct ts_hand_info *)kzalloc(sizeof(struct ts_hand_info), GFP_KERNEL);
	if (!info) {
		TP_LOG_ERR("malloc failed\n");
		error = -ENOMEM;
		goto out;
	}

	info->op_action = TS_ACTION_READ;
	cmd->command = TS_HAND_DETECT;
	cmd->cmd_param.prv_params = (void *)info;
	error = pad_put_one_cmd(cmd, SHORT_SYNC_TIMEOUT);
	if (error) {
		TP_LOG_ERR("put cmd error :%d\n", error);
		error = -EBUSY;
		goto out;
	}

	if (info->status == TS_ACTION_SUCCESS)
		error = snprintf(buf, MAX_STR_LEN ,"%d\n", info->hand_value);
	 else
		error =  -EFAULT;

out:
	if (cmd)
		kfree(cmd);
	if (info)
		kfree(info);
	TP_LOG_DEBUG("done\n");
	return error;
}

static ssize_t TP_LOGlevel_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int error = NO_ERR;

	TP_LOG_INFO("TP_LOGlevel_show called\n");

	if (dev == NULL) {
		TP_LOG_ERR("dev is null\n");
		error = -EINVAL;
		goto out;
	}

	error = snprintf(buf, MAX_STR_LEN, "%d\n", g_tp_log_cfg);

out:
	TP_LOG_INFO("TP_LOGlevel_show done\n");
	return error;
}

static ssize_t tp_touch_window_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int error = NO_ERR;
	struct ts_window_info *info = &g_tp_data.feature_info.window_info;

	error = snprintf(buf, MAX_STR_LEN, "%d %d %d %d %d\n", info->window_enable, info->top_left_x0,
				info->top_left_y0, info->bottom_right_x1, info->bottom_right_y1);

	return error;
}

static ssize_t TP_LOGlevel_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned int value;
	int error;

	TP_LOG_INFO("TP_LOGlevel_store called\n");

	if (dev == NULL) {
		TP_LOG_ERR("dev is null\n");
		error = -EINVAL;
		goto out;
	}

	error = sscanf(buf, "%u", &value);
	if (!error) {
		TP_LOG_ERR("sscanf return invaild :%d\n", error);
		error = -EINVAL;
		goto out;
	}
	TP_LOG_DEBUG("sscanf value is %u\n", value);
	g_tp_log_cfg = value;
	error = count;

out:
	TP_LOG_INFO("TP_LOGlevel_store done\n");
	return error;
}

static ssize_t tp_support_tp_feature_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int error = NO_ERR;
	struct ts_device_data *dev_data = g_tp_data.chip_data;

	TP_LOG_INFO("ts_support_tp_feature_show called\n");

	if (dev == NULL) {
		TP_LOG_ERR("dev is null\n");
		error = -EINVAL;
		goto out;
	}

	error = snprintf(buf, MAX_STR_LEN, "0x%x\n", dev_data->tp_func_flag);

out:
	TP_LOG_INFO("ts_support_tp_feature_show done\n");
	return error;
}

static ssize_t tp_fw_update_sd_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int error = NO_ERR;
	struct ts_cmd_node cmd;

	TP_LOG_INFO("ts_fw_update_sd_store called\n");

	memset(&cmd, 0, sizeof(struct ts_cmd_node));
	cmd.command = TS_FW_UPDATE_SD;
	error = pad_put_one_cmd(&cmd, LONG_LONG_SYNC_TIMEOUT);
	if (error) {
		TP_LOG_ERR("ts_fw_update_sd_store failed:%d\n", error);
	}
	TP_LOG_INFO("ts_fw_update_sd_store done\n");
	return count;
}

static ssize_t tp_register_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int error = NO_ERR;
	int count = 0;
	char *buf_value = NULL;
	int i = 0;
	int ret = 0;

	TP_LOG_INFO("ts_register_show is called\n");
	if (dev == NULL) {
		TP_LOG_ERR("dev is null\n");
		error = -EINVAL;
		return error;
	}
	buf_value = buf;
	for (i = 0; i < g_tp_data.chip_data->reg_num; i++) {
		ret = snprintf(buf_value, TS_MAX_REG_VALUE_NUM, "0x%2x", g_tp_data.chip_data->reg_values[i]);
		buf_value += ret;
		count += ret;		/*0x%2x,one num need four char to show*/
	}
	TP_LOG_DEBUG("show reg value error,maybe no store first,count = %d, buf = %s\n", count, buf);
	if (count <= 0) {
		TP_LOG_ERR("show reg value error,maybe no store first,count = %d, buf = %s\n", count, buf);
		error = -EINVAL;
		return error;
	}
	TP_LOG_INFO("ts_register_show done\n");
	return count;
}

static ssize_t tp_register_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned int fhandler = 0;
	int type = 0;
	int offset = 0;
	char *value = NULL;
	u8 value_u8[TS_MAX_REG_VALUE_NUM] = {0};
	int num = 0;
	int flag = 0;
	int error = NO_ERR;
	int i = 0;
	int j = 0;
	struct ts_cmd_node *cmd = NULL;
	struct ts_regs_info *info = NULL;

	TP_LOG_INFO("ts_register_store called\n");
	if (dev == NULL) {
		TP_LOG_ERR("dev is null\n");
		error = -EINVAL;
		goto out;
	}
	value = (char *)kzalloc(TS_MAX_REG_VALUE_NUM, GFP_KERNEL);
	if (!value) {
		TP_LOG_ERR("value kzalloc error\n");
		error = -ENOMEM;
		goto out;
	}

	error = sscanf(buf, "0x%4x %d %d %s %d %d", &fhandler, &type, &offset, value, &num, &flag);
	if (error <= 0) {
		TP_LOG_ERR("sscanf return invaild fhandler:%d, type :%d, offset:%d, value:%s , num:%d, flag:%d\n", fhandler, type, offset, value, num, flag);
		error = -EINVAL;
		goto free_memory;
	}
	TP_LOG_INFO("sscanf return data is fhandler:0x%2x, type :%d, offset:%d, value:%s ,num:%d, flag:%d\n", fhandler, type, offset, value, num, flag);

	cmd = (struct ts_cmd_node *)kzalloc(sizeof(struct ts_cmd_node), GFP_KERNEL);
	if (!cmd) {
		TP_LOG_ERR("malloc failed\n");
		error = -ENOMEM;
		goto free_memory;
	}
	info = (struct ts_regs_info *)kzalloc(sizeof(struct ts_regs_info), GFP_KERNEL);
	if (!info) {
		TP_LOG_ERR("malloc failed\n");
		error = -ENOMEM;
		goto free_memory;
	}

	if (0 == flag) {
		info->op_action = TS_ACTION_READ;
	} else {
		info->op_action = TS_ACTION_WRITE;
	}
	info->fhandler = fhandler;
	info->type = type;
	info->offset = offset;
	info->num = num/2;
	g_tp_data.chip_data->reg_num = num/2;

	/*convert char to u8 because sscanf char from buf*/
	for(i = 0; i < num; i++) {
		if('0' <= value[i] && value[i] <= '9')
			value_u8[i] = value[i] - '0';
		else if ('a' <= _tolower(value[i]) && _tolower(value[i]) <= 'f')
			value_u8[i] = _tolower(value[i]) - 'a' + 10;
	}
	/*convert input value to reg_values*/
	for (i = 0, j = 0; i < num; i += 2, j++) {
		info->values[j] = (value_u8[i] << 4) | (value_u8[i+1]);
	}

	cmd->command = TS_REGS_STORE;
	cmd->cmd_param.prv_params = (void *)info;
	error = pad_put_one_cmd(cmd, SHORT_SYNC_TIMEOUT);
	if (error) {
		TP_LOG_ERR("put cmd error :%d\n", error);
		error = -EBUSY;
		goto free_memory;
	}
	if (info->status != TS_ACTION_SUCCESS) {
		TP_LOG_ERR("action failed\n");
		error = -EIO;
		goto free_memory;
	}
	error = count;

free_memory:
	if (info)
		kfree(info);
	if (cmd)
		kfree(cmd);
	if (value)
		kfree(value);
out:
	TP_LOG_INFO("ts_reg_operate_store done\n");
	return error;
}

static ssize_t tp_wakeup_gesture_enable_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned int value;
	int error;

	TP_LOG_INFO("%s called\n", __func__);

	error = sscanf(buf, "%u", &value);
	if (error <= 0) {
		TP_LOG_ERR("sscanf return invaild :%d\n", error);
		error = -EINVAL;
		goto out;
	}

	error = tp_wakeup_gesture_enable_cmd(value);
	if (error) {
		TP_LOG_ERR("ts_wakeup_gesture_enable_cmd failed\n");
		error = -ENOMEM;
		goto out;
	}

	error = count;

out:
	return error;
}

static ssize_t tp_wakeup_gesture_enable_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct ts_wakeup_gesture_enable_info *info = &g_tp_data.feature_info.wakeup_gesture_enable_info;
	ssize_t ret;

	TP_LOG_INFO("%s called\n", __func__);

	if (dev == NULL) {
		TP_LOG_ERR("dev is null\n");
		ret = -EINVAL;
		goto out;
	}

	ret = snprintf(buf, MAX_STR_LEN, "%d\n", info->switch_value);
out:
	return ret;
}

static ssize_t tp_easy_wakeup_gesture_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct ts_easy_wakeup_info *info = &g_tp_data.chip_data->easy_wakeup_info;
	ssize_t ret;

	TP_LOG_INFO("ts_easy_wakeup_gesture_show_called\n");

	if (dev == NULL) {
		TP_LOG_ERR("dev is null\n");
		ret = -EINVAL;
		goto out;
	}

	if ((TS_SLEEP== atomic_read(&g_tp_data.state))||(TS_WORK_IN_SLEEP == atomic_read(&g_tp_data.state))) {
		TP_LOG_ERR("do not echo this node when tp work in sleep or tp is sleep\n");
		ret = -EINVAL;
		goto out;
	}

	ret = snprintf(buf, sizeof(info->easy_wakeup_gesture)*2, "0x%04X\n",	info->easy_wakeup_gesture + info->palm_cover_flag*TS_GET_CALCULATE_NUM);
out:
	return ret;
}

static ssize_t tp_easy_wakeup_gesture_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{

	struct ts_easy_wakeup_info *info = &g_tp_data.chip_data->easy_wakeup_info;
	unsigned long value;
	int ret;
	struct ts_cmd_node *cmd = NULL;
	struct ts_palm_info *palm_info = NULL;

	TP_LOG_INFO("ts_easy_wakeup_gesture_store_called\n");

	if (dev == NULL) {
		TP_LOG_ERR("dev is null\n");
		ret = -EINVAL;
		goto out;
	}

	if ((TS_SLEEP== atomic_read(&g_tp_data.state))||(TS_WORK_IN_SLEEP == atomic_read(&g_tp_data.state))) {
		TP_LOG_ERR("do not echo this node when tp work in sleep or tp is sleep\n");
		ret = -EINVAL;
		goto out;
	}

	ret = kstrtoul(buf, 10, &value);
	if (ret < 0)
		return ret;
	if (value > TS_GESTURE_INVALID_COMMAND || value < 0)
		return -1;
	info->easy_wakeup_gesture = (u16)value & TS_GESTURE_COMMAND;
	info->palm_cover_flag = (u16)(value & TS_GESTURE_PALM_BIT) /TS_GET_CALCULATE_NUM;
	if (true == info->palm_cover_flag)
		info->palm_cover_control = true;
	else
		info->palm_cover_control = false;
	TP_LOG_INFO("easy_wakeup_gesture=0x%x,palm_cover_flag=0x%x\n", info->easy_wakeup_gesture, info->palm_cover_flag);
	if (false == info->easy_wakeup_gesture) {
		info->sleep_mode = TS_POWER_OFF_MODE;
		pad_gesture_func = false;
		TP_LOG_INFO("poweroff mode\n");
	} else {
		info->sleep_mode = TS_GESTURE_MODE;
		pad_gesture_func = true;
		TP_LOG_INFO("gesture mode\n");
	}

	cmd = (struct ts_cmd_node *)kzalloc(sizeof(struct ts_cmd_node), GFP_KERNEL);
	if (!cmd) {
		TP_LOG_ERR("malloc failed\n");
		ret = -ENOMEM;
		goto out;
	}
	palm_info = (struct ts_palm_info *)kzalloc(sizeof(struct ts_palm_info), GFP_KERNEL);
	if (!palm_info) {
		TP_LOG_ERR("malloc failed\n");
		ret = -ENOMEM;
		goto out;
	}
	palm_info->op_action = TS_ACTION_WRITE;
	palm_info->palm_switch = info->palm_cover_control;
	cmd->command = TS_PALM_SWITCH;
	cmd->cmd_param.prv_params = (void *)palm_info;
	ret = pad_put_one_cmd(cmd, SHORT_SYNC_TIMEOUT);
	if (ret) {
		TP_LOG_ERR("put cmd error :%d\n", ret);
		ret = -EBUSY;
		goto out;
	}
	if (palm_info->status != TS_ACTION_SUCCESS) {
		TP_LOG_ERR("action failed\n");
		ret = -EIO;
		goto out;
	}
	ret = size;
out:
	if (palm_info)
		kfree(palm_info);
	if (cmd)
		kfree(cmd);
	TP_LOG_DEBUG("ts gesture wakeup no done\n");

	return ret;
}

static ssize_t tp_easy_wakeup_control_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{

	unsigned long value;
	int ret;
	int error = NO_ERR;

	TP_LOG_INFO("called\n");

	ret = kstrtoul(buf, 10, &value);
	if (ret < 0)
		return ret;

	if (value > TS_GESTURE_INVALID_CONTROL_NO && value < 0)
		return -1;

	value = (u8)value & TS_GESTURE_COMMAND;
	if(1 == value)
	{
		if( NULL!=g_tp_data.chip_data->ops->chip_wrong_touch)
		{
			error=g_tp_data.chip_data->ops->chip_wrong_touch();
			if(error<0)
			{
				TP_LOG_INFO("chip_wrong_touch error\n");
			}
		}
		else
		{
			TP_LOG_INFO("chip_wrong_touch not init\n");
		}
		value = 0;
	}
	TP_LOG_INFO("done\n");
	return size;
}

static ssize_t tp_easy_wakeup_position_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int ret = 0;
	char temp[9] = {0};
	int i = 0;
	TP_LOG_INFO("ts_position_show\n");
	if (dev == NULL) {
		TP_LOG_ERR("dev is null\n");
		ret = -EINVAL;
		goto out;
	}
	mutex_lock(&easy_wake_guesure_lock);
	for(i = 0; i < MAX_LOTUS_NUM; i ++)
	{
		ret = snprintf(temp, (sizeof(u32)*2+1), "%08x", g_tp_data.chip_data->easy_wakeup_info.easywake_position[i]);
		strncat(buf, temp ,(sizeof(u32)*2+1));
	}
	strncat(buf,"\n",1);
	strncat(buf, "\0",1);
	mutex_unlock(&easy_wake_guesure_lock);
	ret = (strlen(buf)+1);
out:
	return ret;
}

static ssize_t tp_roi_enable_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned int value;
	int error;

	error = tp_send_roi_cmd(TS_ACTION_READ, SHORT_SYNC_TIMEOUT);
	if (error) {
		TP_LOG_ERR("ts_send_roi_show_cmd failed\n");
		error = -ENOMEM;
		goto out;
	}
	value = g_tp_data.feature_info.roi_info.roi_switch;
	error = snprintf(buf, MAX_STR_LEN, "%d\n", value);
out:
	TP_LOG_DEBUG("roi_enable_show done\n");
	return error;
}


static ssize_t tp_roi_enable_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned int value;
	int error;
	struct ts_roi_info *info = &g_tp_data.feature_info.roi_info;

	error = sscanf(buf, "%u", &value);
	if (error <= 0) {
		TP_LOG_ERR("sscanf return invaild :%d\n", error);
		error = -EINVAL;
		goto out;
	}
	TP_LOG_DEBUG("sscanf value is %u\n", value);

	info->roi_switch = value;
	error = tp_send_roi_cmd(TS_ACTION_WRITE, SHORT_SYNC_TIMEOUT);
	if (error) {
		TP_LOG_ERR("ts_send_roi_store_cmd failed\n");
		error = -ENOMEM;
		goto out;
	}
	error = count;
out:
	TP_LOG_DEBUG("roi_enable_store done\n");
	return error;
}

static ssize_t tp_roi_data_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	//int i, j = 0;
	unsigned char *roi_data_p = NULL;
	struct ts_device_data *dev_data = g_tp_data.chip_data;

	if (dev_data->ops->chip_roi_rawdata)
		roi_data_p = dev_data->ops->chip_roi_rawdata();
	if (NULL == roi_data_p) {
		TP_LOG_ERR("not define ROI for roi_data_show \n");
		return -ENOMEM;
	}
	//roi_data_temp <-- This is the buffer that has the ROI data you want to send to Qeexo
	memcpy(buf, roi_data_p+ROI_HEAD_DATA_LENGTH, ROI_DATA_SEND_LENGTH);
	return ROI_DATA_SEND_LENGTH;
}

static ssize_t tp_roi_data_debug_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int cnt;
	int count = 0;
	int i, j = 0;
	short roi_data_16[ROI_DATA_SEND_LENGTH/2];
	unsigned char *roi_data_p = NULL;
	struct ts_device_data *dev_data = g_tp_data.chip_data;

	if (dev_data->ops->chip_roi_rawdata)
		roi_data_p = dev_data->ops->chip_roi_rawdata();
	if (NULL == roi_data_p) {
		TP_LOG_ERR("not define ROI for roi_data_show \n");
		return -ENOMEM;
	}
	TP_LOG_DEBUG("ts_roi_data_debug_show CALLED \n");

	for (i=0; i<ROI_HEAD_DATA_LENGTH; i++) {
		cnt = snprintf(buf, PAGE_SIZE-count, "%2d\n", (char)roi_data_p[i]);
		buf += cnt;
		count += cnt;
	}

	for (i=ROI_HEAD_DATA_LENGTH; i<ROI_DATA_READ_LENGTH; i+=2, j++){
		roi_data_16[j] = roi_data_p[i] | (roi_data_p[i+1]<<8);
		cnt = snprintf(buf, PAGE_SIZE-count, "%4d", roi_data_16[j]);
		buf += cnt;
		count += cnt;

		if((j+1)%7 == 0){
			cnt = snprintf(buf, PAGE_SIZE-count, "\n");
			buf += cnt;
			count += cnt;
		}
	}
	snprintf(buf, PAGE_SIZE-count, "\n");
	count++;
	return count;
}

static DEVICE_ATTR(touch_chip_info, (S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH), pad_ts_chip_info_show, pad_ts_chip_info_store);
static DEVICE_ATTR(calibrate, S_IRUSR, tp_calibrate_show, NULL);
static DEVICE_ATTR(touch_glove, (S_IRUSR|S_IWUSR), tp_glove_mode_show, tp_glove_mode_store);
static DEVICE_ATTR(touch_weight, (S_IRUSR|S_IWUSR), tp_touch_weight_switch_show, tp_touch_weight_switch_store);
static DEVICE_ATTR(touch_sensitivity, S_IWUSR, NULL, tp_sensitivity_store);
static DEVICE_ATTR(hand_detect, S_IRUSR, tp_hand_detect_show, NULL);
static DEVICE_ATTR(support_tp_feature, S_IRUSR, tp_support_tp_feature_show, NULL);
static DEVICE_ATTR(loglevel, (S_IRUSR|S_IWUSR), TP_LOGlevel_show, TP_LOGlevel_store);
static DEVICE_ATTR(touch_window, (S_IRUSR|S_IWUSR), tp_touch_window_show, tp_touch_window_store);
static DEVICE_ATTR(fw_update_sd, S_IWUSR, NULL, tp_fw_update_sd_store);
static DEVICE_ATTR(easy_wakeup_gesture, (S_IRUSR | S_IWUSR), tp_easy_wakeup_gesture_show, tp_easy_wakeup_gesture_store);
static DEVICE_ATTR(wakeup_gesture_enable, (S_IRUSR | S_IWUSR), tp_wakeup_gesture_enable_show, tp_wakeup_gesture_enable_store);
static DEVICE_ATTR(touch_dsm_debug, S_IRUSR | S_IRGRP | S_IROTH, tp_dsm_debug_show, NULL);
static DEVICE_ATTR(easy_wakeup_control, S_IWUSR, NULL, tp_easy_wakeup_control_store);
static DEVICE_ATTR(easy_wakeup_position, S_IRUSR, tp_easy_wakeup_position_show, NULL);
static DEVICE_ATTR(touch_register_operation, S_IRUSR | S_IRGRP | S_IWUSR | S_IWGRP, tp_register_show, tp_register_store);
static DEVICE_ATTR(roi_enable, (S_IRUSR | S_IRGRP | S_IWUSR | S_IWGRP), tp_roi_enable_show, tp_roi_enable_store);
static DEVICE_ATTR(roi_data, (S_IRUSR | S_IRGRP), tp_roi_data_show, NULL);
static DEVICE_ATTR(roi_data_debug, (S_IRUSR | S_IRGRP), tp_roi_data_debug_show, NULL);


static struct attribute *ts_attributes[] = {
	&dev_attr_touch_chip_info.attr,
       &dev_attr_calibrate.attr,
       &dev_attr_touch_glove.attr,
       &dev_attr_touch_weight.attr,
       &dev_attr_touch_sensitivity.attr,
       &dev_attr_hand_detect.attr,
       &dev_attr_loglevel.attr,
       &dev_attr_support_tp_feature.attr,
       &dev_attr_touch_window.attr,
       &dev_attr_fw_update_sd.attr,
       &dev_attr_easy_wakeup_gesture.attr,
       &dev_attr_wakeup_gesture_enable.attr,
       &dev_attr_touch_dsm_debug.attr,
       &dev_attr_easy_wakeup_control.attr,
       &dev_attr_easy_wakeup_position.attr,
	   &dev_attr_touch_register_operation.attr,
	   &dev_attr_roi_enable.attr,
	   &dev_attr_roi_data.attr,
	   &dev_attr_roi_data_debug.attr,
	NULL
};

static const struct attribute_group ts_attr_group = {
	.attrs = ts_attributes,
};

static int tp_rawdata_proc_show(struct seq_file *m, void *v)
{
	int index;
	int index1;
	short row_size = 0;
	int range_size = 0;
	int error = NO_ERR;
	struct ts_cmd_node *cmd = NULL;
	struct ts_rawdata_info *info = NULL;

	TP_LOG_INFO("rawdata_proc_show, buffer size = %ld\n", m->size);
	if(m->size <= RAW_DATA_SIZE) {
		m->count = m->size;
		return 0;
	}

	cmd = (struct ts_cmd_node *)kzalloc(sizeof(struct ts_cmd_node), GFP_KERNEL);
	if (!cmd) {
		TP_LOG_ERR("malloc failed\n");
		error = -ENOMEM;
		goto out;
	}

	info = (struct ts_rawdata_info *)kzalloc(sizeof(struct ts_rawdata_info), GFP_KERNEL);
	if (!info) {
		TP_LOG_ERR("malloc failed\n");
		error = -ENOMEM;
		goto out;
	}

	info->used_size = 0;
	cmd->command = TS_READ_RAW_DATA;
	cmd->cmd_param.prv_params = (void *)info;

    error = pad_put_one_cmd(cmd, LONG_SYNC_TIMEOUT);  //rawdate test timeout
	if (error) {
		TP_LOG_ERR("put cmd error :%d\n", error);
		error = -EBUSY;
		goto out;
	}

	if (info->status != TS_ACTION_SUCCESS) {
		TP_LOG_ERR("read action failed\n");
		error = -EIO;
		goto out;
	}
	seq_printf(m, "result:%s\n", info->result);
	seq_printf(m, "*************touch data*************\n");

	row_size = info->buff[0];
	range_size = info->buff[1];
	seq_printf(m, "rx: %d, tx : %d\n", row_size, range_size);

	if (g_tp_data.chip_data->rawdata_arrange_swap) {
		row_size = info->buff[1];
		range_size = info->buff[0];
	}

	for (index=0; row_size*index+2 < info->used_size; index++) {
		if (0 == index) {
			seq_printf(m, "rawdata begin\n");							/*print the title*/
		}
		for (index1=0; index1 < row_size; index1++) {
			seq_printf(m, "%d,", info->buff[2+row_size*index+index1]);		/*print oneline*/
		}
		//index1 = 0;
		seq_printf(m, "\n ");

		if ((range_size -1) == index) {
			seq_printf(m, "rawdata end\n");
			seq_printf(m, "noisedata begin\n");
		}
	}
	seq_printf(m, "noisedata end\n");

	error = NO_ERR;

out:
	if (info)
		kfree(info);
	if (cmd)
		kfree(cmd);

	TP_LOG_INFO("rawdata_proc_show done\n");
	return error;
}

static int tp_rawdata_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, tp_rawdata_proc_show, NULL);
}

static const struct file_operations rawdata_proc_fops = {
	.open		= tp_rawdata_proc_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static void tp_procfs_create(void)
{
	if (!proc_mkdir("touchpad", NULL))
		return;
	proc_create("touchpad/rawdata", S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH, NULL, &rawdata_proc_fops);
	return;
}

int pad_ts_power_control_notify(enum ts_pm_type pm_type, int timeout)
{
	int error;
	struct ts_cmd_node cmd;

	cmd.command = TS_POWER_CONTROL;
	cmd.cmd_param.pub_params.pm_type = pm_type;
	error = pad_put_one_cmd(&cmd, timeout);
	if (error) {
		TP_LOG_ERR("pad_ts_power_control_notify, put cmd error :%d\n", error);
		error = -EBUSY;
	}
	if (TS_AFTER_RESUME == pm_type) {
		TP_LOG_INFO("ts_resume_send_roi_cmd\n");
		tp_send_roi_cmd(TS_ACTION_WRITE, NO_SYNC_TIMEOUT);				/*force to write the roi function*/
		if (error) {
			TP_LOG_ERR("ts_resume_send_roi_cmd failed\n");
		}
	}
	return error;
}

void pad_ts_thread_stop_notify(void)
{
	struct ts_device_data *dev = g_tp_data.chip_data;

	TP_LOG_INFO("ts thread stop called by lcd only shutdown\n");
	atomic_set(&g_tp_data.state, TS_UNINIT);
	disable_irq(g_tp_data.irq_id);
	tp_stop_wd_timer(&g_tp_data);
	if (dev->ops->chip_shutdown)
		dev->ops->chip_shutdown();
	/*there is something wrong about system, now abandon the kthread_stop to avoid unkown bug*/
	//kthread_stop(g_tp_data.ts_task);
}

#if defined(CONFIG_FB)
static int tp_fb_notifier_callback(struct notifier_block *self, unsigned long event, void *data)
{
	int error = NO_ERR;
	int i;
	struct fb_event *fb_event = data;
	int *blank = fb_event->data;
	unsigned char ts_state = 0;
	int times = 0;

	if(blank == NULL){
		TP_LOG_ERR("%s:blank is NULL\n",__func__);
		return error;
	}

	for (i = 0 ; i < FB_MAX; i++) {
		if (registered_fb[i] == fb_event->info) {
			if (i == 0) {
				TP_LOG_DEBUG("Ts-index:%d, go on !\n", i);
				break;
			} else {
				TP_LOG_INFO("Ts-index:%d, exit !\n", i);
				return error;
			}
		}
	}
	TP_LOG_INFO("fb_notifier_callback, blank: %d, event:%lu, state: %d\n", *blank, event, atomic_read(&g_tp_data.state));
	switch(*blank){
	case FB_BLANK_UNBLANK:
			/*resume device*/
			switch(event) {

			case FB_EARLY_EVENT_BLANK:
				while(1) {
					ts_state = atomic_read(&g_tp_data.state);
					if ((TS_SLEEP == ts_state) || (TS_WORK_IN_SLEEP == ts_state)) {
						error = pad_ts_power_control_notify(TS_RESUME_DEVICE, SHORT_SYNC_TIMEOUT);		/*touch power on*/
						if (error)
							TP_LOG_ERR("ts resume device err\n");
						error = pad_ts_power_control_notify(TS_AFTER_RESUME, NO_SYNC_TIMEOUT);	/*enable irq*/
						if (error)
							TP_LOG_ERR("ts after resume err\n");
						break;
					} else {
						msleep(TP_FB_WAIT_TIME);
						if (times++ > TP_FB_LOOP_COUNTS) {
							times = 0;
							TP_LOG_ERR("no resume, blank: %d, event:%lu, state: %d\n", *blank, event, ts_state);
							break;
						}
				}
				break;
			case FB_EVENT_BLANK:
				TP_LOG_DEBUG("resume: event = %lu, not care\n", event);
				break;
			default:
				TP_LOG_DEBUG("resume: event = %lu, not care\n", event);
				break;
			}
		}
		break;
	case FB_BLANK_VSYNC_SUSPEND:
	case FB_BLANK_HSYNC_SUSPEND:
	case FB_BLANK_NORMAL:
	case FB_BLANK_POWERDOWN:
	default:
			/*suspend device*/
			switch(event) {
			case FB_EARLY_EVENT_BLANK:
				while(1) {
					ts_state = atomic_read(&g_tp_data.state);
					if ((TS_WORK == ts_state) || (TS_WORK_IN_SLEEP == ts_state)) {
						error = pad_ts_power_control_notify(TS_BEFORE_SUSPEND, SHORT_SYNC_TIMEOUT);	/*disable irq*/
						if (error)
							TP_LOG_ERR("ts suspend device err\n");
						break;
					} else {
						msleep(TP_FB_WAIT_TIME);
						if (times++ > TP_FB_LOOP_COUNTS) {
							times = 0;
							TP_LOG_ERR("no early suspend, blank: %d, event:%lu, state: %d\n", *blank, event, ts_state);
							break;
						}
					}
				}
				break;
			case FB_EVENT_BLANK:
				while(1) {
					ts_state = atomic_read(&g_tp_data.state);
					if ((TS_WORK == ts_state) || (TS_WORK_IN_SLEEP == ts_state)) {
						error = pad_ts_power_control_notify(TS_SUSPEND_DEVICE, NO_SYNC_TIMEOUT);	/*touch power off*/
						if (error)
							TP_LOG_ERR("ts before suspend err\n");
						break;
					} else {
						msleep(TP_FB_WAIT_TIME);
						if (times++ > TP_FB_LOOP_COUNTS) {
							times = 0;
							TP_LOG_ERR("no suspend, blank: %d, event:%lu, state: %d\n", *blank, event, ts_state);
							break;
						}
					}
				}
				break;
			default:
				TP_LOG_DEBUG("suspend: event = %lu, not care\n", event);
				break;
			}
		break;
	}

	return NO_ERR;
}
#elif defined(CONFIG_HAS_EARLYSUSPEND)
static void tp_early_suspend(struct early_suspend *h)
{
	bool is_in_cell = g_tp_data.chip_data->is_in_cell;
	int error = NO_ERR;

	TP_LOG_INFO("ts early suspend, %s\n", (is_in_cell == false) ? "need suspend" : "no need suspend");

	/*for the in-cell, ts_suspend_notify called by lcd, not here*/
	if (false == is_in_cell) {
		error = pad_ts_power_control_notify(TS_BEFORE_SUSPEND, SHORT_SYNC_TIMEOUT);
		if (error)
			TP_LOG_ERR("ts before suspend err\n");
		error = pad_ts_power_control_notify(TS_SUSPEND_DEVICE, SHORT_SYNC_TIMEOUT);
		if (error)
			TP_LOG_ERR("ts suspend device err\n");
	}

	TP_LOG_INFO("ts_early_suspend done\n");
}

static void tp_late_resume(struct early_suspend *h)
{
	bool is_in_cell = g_tp_data.chip_data->is_in_cell;
	int error = NO_ERR;
	TP_LOG_INFO("ts late resume, %s\n", (is_in_cell == false) ? "need resume" : "no need resume");

	/*for the in-cell, ts_resume_notify called by lcd, not here*/
	if (false == is_in_cell) {
		error = pad_ts_power_control_notify(TS_RESUME_DEVICE, SHORT_SYNC_TIMEOUT);
		if (error)
			TP_LOG_ERR("ts resume device err\n");
		error = pad_ts_power_control_notify(TS_AFTER_RESUME, SHORT_SYNC_TIMEOUT);
		if (error)
			TP_LOG_ERR("ts after resume err\n");
	}
	tp_send_holster_cmd();

	TP_LOG_INFO("ts_late_resume done\n");
}
#endif

static irqreturn_t tp_irq_handler(int irq, void *dev_id)
{
	int error = NO_ERR;
	struct ts_cmd_node cmd;

	wake_lock_timeout(&g_tp_data.ts_wake_lock, HZ);
	if (g_tp_data.chip_data->ops->chip_irq_top_half)
		error = g_tp_data.chip_data->ops->chip_irq_top_half(&cmd);

	if (error)//unexpected error happen, put err cmd to ts thread
		cmd.command = TS_INT_ERR_OCCUR;
	else
		cmd.command = TS_INT_PROCESS;
	TP_LOG_ERR("--cbw-- %s %d\n",__func__,__LINE__);
	disable_irq_nosync(g_tp_data.irq_id);

	if (pad_put_one_cmd(&cmd, NO_SYNC_TIMEOUT))
		enable_irq(g_tp_data.irq_id);

	return IRQ_HANDLED;
}

static inline void tp_proc_bottom_half(struct ts_cmd_node *in_cmd, struct ts_cmd_node *out_cmd)
{
	struct ts_device_data *dev = g_tp_data.chip_data;

	TP_LOG_DEBUG("bottom half called\n");
	//related event need process, use out cmd to notify
	if (dev->ops->chip_irq_bottom_half)
		dev->ops->chip_irq_bottom_half(in_cmd, out_cmd);
}

static void tp_watchdog_work(struct work_struct *work)
{
	int error = NO_ERR;
	struct ts_cmd_node cmd;

	TP_LOG_DEBUG("ts_watchdog_work\n");
	cmd.command = TS_CHECK_STATUS;

	error=pad_put_one_cmd(&cmd, NO_SYNC_TIMEOUT);
	if (error) {
		TP_LOG_ERR("put TS_CHECK_STATUS cmd error :%d\n", error);
	}
	return;
}

static void tp_watchdog_timer(unsigned long data)
{
	struct ts_data *cd = (struct ts_data *)data;

	TP_LOG_DEBUG("Timer triggered\n");

	if (!cd)
		return;

	if (!work_pending(&cd->watchdog_work))
		schedule_work(&cd->watchdog_work);

	return;
}

static void tp_start_wd_timer(struct ts_data *cd)
{
	if (!TS_WATCHDOG_TIMEOUT)
		return;

	TP_LOG_DEBUG("start wd\n");
	mod_timer(&cd->watchdog_timer, jiffies +
			msecs_to_jiffies(TS_WATCHDOG_TIMEOUT));
	return;
}

static void tp_stop_wd_timer(struct ts_data *cd)
{
	if (!TS_WATCHDOG_TIMEOUT)
		return;

	TP_LOG_DEBUG("stop wd\n");
	del_timer(&cd->watchdog_timer);
	cancel_work_sync(&cd->watchdog_work);
	del_timer(&cd->watchdog_timer);
	return;
}

static inline void tp_algo_calibrate(struct ts_cmd_node *in_cmd, struct ts_cmd_node *out_cmd)
{
	int id;
	int algo_size = g_tp_data.chip_data->algo_size;
	u32 order = in_cmd->cmd_param.pub_params.algo_param.algo_order;
	struct ts_fingers *in_finger = &in_cmd->cmd_param.pub_params.algo_param.info;
	struct ts_fingers *out_finger = &out_cmd->cmd_param.pub_params.algo_param.info;
	struct ts_algo_func *algo;

	if (!algo_size) {
		TP_LOG_INFO("no algo handler, direct report\n");
		goto out;
	}

	TP_LOG_DEBUG("algo order: %d, algo_size :%d\n", order, algo_size);

	for(id=0; id < algo_size; id++)
	{
		if (order & BIT_MASK(id)) {
			TP_LOG_DEBUG("algo id:%d is setted\n", id);
			list_for_each_entry(algo, &g_tp_data.chip_data->algo_head, node) {
				if (algo->algo_index == id) {//found the right algo func
					TP_LOG_DEBUG("algo :%s called\n", algo->algo_name);
					algo->chip_algo_func(g_tp_data.chip_data, in_finger, out_finger);
					memcpy(in_finger, out_finger, sizeof(struct ts_fingers));
					break;
				}
			}
		}
	}

out:
	memcpy(&out_cmd->cmd_param.pub_params.report_info, in_finger, sizeof(struct ts_fingers));
	out_cmd->command = TS_REPORT_INPUT;
	return;
}

/**
 * ts_film_touchplus()
 *
 * Called by ts_report_input()
 *
 * touchplus(LingXiYiZhi) report key in this function
 */
static void tp_film_touchplus(struct ts_fingers *finger, int finger_num,struct input_dev *input_dev)
{
	static int pre_special_button_key = TS_TOUCHPLUS_INVALID;
	int key_max = TS_TOUCHPLUS_KEY2;
	int key_min = TS_TOUCHPLUS_KEY3;
	unsigned char ts_state = 0;

	TP_LOG_DEBUG("ts_film_touchplus called\n");

	/*discard touchplus report in gesture wakeup mode*/
	ts_state = atomic_read(&g_tp_data.state);
	if ((TS_SLEEP == ts_state) || (TS_WORK_IN_SLEEP == ts_state)) {
		return;
	}

	/*touchplus(LingXiYiZhi) report ,  The difference between ABS_report and touchpls key_report
	*when ABS_report is running, touchpls key will not report
	*when touchpls key is not in range of touchpls keys, will not report key
	*/
	if ((finger_num != 0) || (finger->special_button_key > key_max) || (finger->special_button_key < key_min)) {
		if (finger->special_button_flag != 0) {
			input_report_key(input_dev, finger->special_button_key, 0);
			input_sync(input_dev);
		}
		return;
	}

	/*touchplus(LingXiYiZhi) report ,  store touchpls key data(finger->special_button_key)
	*when special_button_flag report touchpls key DOWN , store current touchpls key
	*till the key report UP, then other keys will not report
	*/
	if (finger->special_button_flag == 1) {
		input_report_key(input_dev, finger->special_button_key, finger->special_button_flag);
		input_sync(input_dev);
	} else if ((finger->special_button_flag == 0) && (pre_special_button_key == finger->special_button_key)) {
		input_report_key(input_dev, finger->special_button_key, finger->special_button_flag);
		input_sync(input_dev);
	} else if ((finger->special_button_flag == 0) && (pre_special_button_key != finger->special_button_key)) {
		input_report_key(input_dev, pre_special_button_key, 0);
		input_sync(input_dev);
	}
	pre_special_button_key = finger->special_button_key;

	return;
}

static inline void tp_report_input(struct ts_cmd_node *in_cmd, struct ts_cmd_node *out_cmd)
{
	struct ts_fingers *finger = &in_cmd->cmd_param.pub_params.report_info;
	struct input_dev *input_dev = g_tp_data.input_dev;
	struct input_dev *slide_dev = g_tp_data.slide_dev;
	int finger_num = 0,slide_finger_num = 0;
	int id;

	TP_LOG_DEBUG("ts_report_input\n");
	tp_check_touch_window(finger);

	if(g_tp_data.feature_info.slide_info.slide_supported && finger->slide_flag){
		for(id = 0; id < TP_MAX_SLIDE_FINGER; id++) {
			if(finger->slide_fingers[id].status == 0) {
				TP_LOG_DEBUG("never touch before: id is %d\n", id);
				continue;
			}
			if(finger->slide_fingers[id].status == TS_FINGER_PRESS) {
				TP_LOG_DEBUG("down: id is %d, finger->slide_fingers[id].pressure = %d, finger->slide_fingers[id].x = %d, finger->slide_fingers[id].y = %d\n",
					id, finger->slide_fingers[id].pressure, finger->slide_fingers[id].x, finger->slide_fingers[id].y);
				slide_finger_num++;
				input_report_abs(slide_dev, ABS_MT_PRESSURE, finger->slide_fingers[id].pressure);
				input_report_abs(slide_dev, ABS_MT_POSITION_X, finger->slide_fingers[id].x);
				input_report_abs(slide_dev, ABS_MT_POSITION_Y, finger->slide_fingers[id].y);
				input_report_abs(slide_dev, ABS_MT_TRACKING_ID, id);
				input_mt_sync(slide_dev);				//modfiy by mengkun
			} else if (finger->slide_fingers[id].status == TS_FINGER_RELEASE) {
				TP_LOG_DEBUG("up: id is %d, status = %d\n", id, finger->slide_fingers[id].status);
				input_mt_sync(slide_dev);	//modfiy by mengkun
			}
		}

		input_report_key(slide_dev, BTN_TOUCH, slide_finger_num);
		input_sync(slide_dev);
	}
	if(finger->finger_flag || !g_tp_data.feature_info.slide_info.slide_supported){
		for (id = 0; id < TP_MAX_FINGER; id++) {
			if (finger->fingers[id].status == 0) {
				TP_LOG_DEBUG("never touch before: id is %d\n", id);
				continue;
			}
			if (finger->fingers[id].status == TS_FINGER_PRESS) {
				TP_LOG_DEBUG("down: id is %d, finger->fingers[id].pressure = %d, finger->fingers[id].x = %d, finger->fingers[id].y = %d\n",
					id, finger->fingers[id].pressure, finger->fingers[id].x, finger->fingers[id].y);
				TP_LOG_ERR ("--cbw-- down:id=%d,fingers[id].pressure=%d,fingers[id].x=%d,fingers[id].y=%d\n",
					id, finger->fingers[id].pressure, finger->fingers[id].x, finger->fingers[id].y);
				finger_num++;
				input_report_abs(input_dev, ABS_MT_PRESSURE, finger->fingers[id].pressure);
				input_report_abs(input_dev, ABS_MT_POSITION_X, finger->fingers[id].x);
				input_report_abs(input_dev, ABS_MT_POSITION_Y, finger->fingers[id].y);
				input_report_abs(input_dev, ABS_MT_TRACKING_ID, id);
				input_mt_sync(input_dev);				//modfiy by mengkun
			} else if (finger->fingers[id].status == TS_FINGER_RELEASE) {
				TP_LOG_DEBUG("up: id is %d, status = %d\n", id, finger->fingers[id].status);
				input_mt_sync(input_dev);	//modfiy by mengkun
			}
		}

		input_report_key(input_dev, BTN_TOUCH, finger_num);
		input_sync(input_dev);

	}
	tp_film_touchplus(finger, finger_num, input_dev);
	if(((g_tp_data.chip_data->easy_wakeup_info.sleep_mode == TS_GESTURE_MODE) ||
		(g_tp_data.chip_data->easy_wakeup_info.palm_cover_flag == true)) &&
		(g_tp_data.feature_info.holster_info.holster_switch == 0)) {
		input_report_key (input_dev, finger->gesture_wakeup_value, 1);
		input_sync(input_dev);
		input_report_key (input_dev, finger->gesture_wakeup_value, 0);
		input_sync (input_dev);
	}
	TP_LOG_DEBUG("ts_report_input done, finger_num = %d\n", finger_num);
	return;
}

void tp_send_up_msg_in_resume(void)
{
	struct input_dev *input_dev = g_tp_data.input_dev;

	input_report_key(input_dev, BTN_TOUCH, 0);
	input_mt_sync(input_dev);
	input_sync(input_dev);
	TP_LOG_DEBUG("pad_send_up_msg_in_resume\n");
	return;
}

static int tp_power_control(int irq_id,
	struct ts_cmd_node *in_cmd, struct ts_cmd_node *out_cmd)
{
	int error = NO_ERR;
	enum ts_pm_type pm_type = in_cmd->cmd_param.pub_params.pm_type;
	struct ts_device_data *dev = g_tp_data.chip_data;

	if (g_tp_data.chip_data->easy_wakeup_info.sleep_mode == TS_POWER_OFF_MODE) {
		switch (pm_type) {
		case TS_BEFORE_SUSPEND: 	/*do something before suspend*/
			tp_stop_wd_timer(&g_tp_data);
			disable_irq(irq_id);
			if (dev->ops->chip_before_suspend)
				error = dev->ops->chip_before_suspend();
			break;
		case TS_SUSPEND_DEVICE:  	/*device power off or sleep*/
			atomic_set(&g_tp_data.state, TS_SLEEP);
			if (dev->ops->chip_suspend)
				error = dev->ops->chip_suspend();
			break;
		case TS_IC_SHUT_DOWN:
			disable_irq(irq_id);
			if (dev->ops->chip_shutdown)
				dev->ops->chip_shutdown();
			break;
		case TS_RESUME_DEVICE:	 /*device power on or wakeup*/
			if (dev->ops->chip_resume)
				error = dev->ops->chip_resume();
			break;
		case TS_AFTER_RESUME:		/*do something after resume*/
			if (dev->ops->chip_after_resume)
				error = dev->ops->chip_after_resume((void *)&g_tp_data.feature_info);
			tp_send_up_msg_in_resume();
			atomic_set(&g_tp_data.state, TS_WORK);
			enable_irq(irq_id);
			tp_start_wd_timer(&g_tp_data);
			break;
		default:
			TP_LOG_ERR("pm_type = %d\n", pm_type);
			error = -EINVAL;
			break;
		}
	} else if (g_tp_data.chip_data->easy_wakeup_info.sleep_mode == TS_GESTURE_MODE) {
		switch (pm_type) {
		case TS_BEFORE_SUSPEND: 		/*do something before suspend*/
			tp_stop_wd_timer(&g_tp_data);
			disable_irq(irq_id);
			if (dev->ops->chip_before_suspend)
				error = dev->ops->chip_before_suspend();
			break;
		case TS_SUSPEND_DEVICE:  	/*switch to easy-wakeup mode, and enable interrupts*/
			atomic_set(&g_tp_data.state, TS_WORK_IN_SLEEP);
			if (dev->ops->chip_suspend)
				error = dev->ops->chip_suspend();
			enable_irq(irq_id);
			out_cmd->command = TS_WAKEUP_GESTURE_ENABLE;
			out_cmd->cmd_param.prv_params = (void *)&g_tp_data.feature_info.wakeup_gesture_enable_info;
			break;
		case TS_IC_SHUT_DOWN:
			disable_irq(irq_id);
			if (dev->ops->chip_shutdown)
				dev->ops->chip_shutdown();
			break;
		case TS_RESUME_DEVICE:	 	/*exit easy-wakeup mode and restore sth*/
			disable_irq(irq_id);
			if (dev->ops->chip_resume)
				error = dev->ops->chip_resume();
			break;
		case TS_AFTER_RESUME:		/*do nothing*/
			if (dev->ops->chip_after_resume)
				error = dev->ops->chip_after_resume((void *)&g_tp_data.feature_info);
			tp_send_up_msg_in_resume();
			atomic_set(&g_tp_data.state, TS_WORK);
			enable_irq(irq_id);
			tp_start_wd_timer(&g_tp_data);
			break;
		default:
			TP_LOG_ERR("pm_type = %d\n", pm_type);
			error = -EINVAL;
			break;
		}

	} else {
		TP_LOG_ERR("no such mode\n");
		error = -EINVAL;
	}
	return error;
}

static inline int tp_fw_update_boot(struct ts_cmd_node *in_cmd, struct ts_cmd_node *out_cmd)
{
	char *fw_name = in_cmd->cmd_param.pub_params.firmware_info.fw_name;
	int error = NO_ERR;
	struct ts_device_data *dev = g_tp_data.chip_data;

	if (dev->ops->chip_fw_update_boot)
		error = dev->ops->chip_fw_update_boot(fw_name);

#if defined (CONFIG_ZEUSIS_DSM)
	if (error) {
		if(!dsm_client_ocuppy(tp_dclient)){
			dsm_client_record(tp_dclient, "fw update result: failed.\nupdata_status is %d.\n",\
				g_tp_data.dsm_info.constraints_UPDATE_status);
			dsm_client_notify(tp_dclient, DSM_TP_FWUPDATE_ERROR_NO);
		}
		strncpy(g_tp_data.dsm_info.fw_update_result, "failed", strlen("failed"));
	}
#endif
	TP_LOG_INFO("process firmware update boot, return value:%d\n", error);

	return error;
}

static inline int tp_fw_update_sd(struct ts_cmd_node *in_cmd, struct ts_cmd_node *out_cmd)
{
	int error = NO_ERR;
	struct ts_device_data *dev = g_tp_data.chip_data;

	TP_LOG_INFO("process firmware update sd\n");

	if (dev->ops->chip_fw_update_sd)
		error = dev->ops->chip_fw_update_sd();

	return error;
}

static inline int tp_calibrate(struct ts_cmd_node *in_cmd, struct ts_cmd_node *out_cmd)
{
	int error = NO_ERR;
	struct ts_device_data *dev = g_tp_data.chip_data;
	struct ts_calibrate_info *info = (struct ts_calibrate_info *)in_cmd->cmd_param.prv_params;

	TP_LOG_DEBUG("process firmware calibrate\n");

	if (dev->ops->chip_calibrate)
		error = dev->ops->chip_calibrate();

	if (error)
		info->status = TS_ACTION_FAILED;
	else
		info->status = TS_ACTION_SUCCESS;

	return error;
}

static inline int tp_dsm_debug(struct ts_cmd_node *in_cmd, struct ts_cmd_node *out_cmd)
{
	int error = NO_ERR;
	struct ts_device_data *dev = g_tp_data.chip_data;
	struct ts_dsm_debug_info *info = (struct ts_dsm_debug_info *)in_cmd->cmd_param.prv_params;

	TP_LOG_INFO("ts dsm debug is called\n");

	if (dev->ops->chip_dsm_debug)
		error = dev->ops->chip_dsm_debug();

	if (error)
		info->status = TS_ACTION_FAILED;
	else
		info->status = TS_ACTION_SUCCESS;

	return error;
}

static int tp_get_chip_info(struct ts_cmd_node *in_cmd, struct ts_cmd_node *out_cmd)
{
	int error = NO_ERR;
	struct ts_device_data *dev = g_tp_data.chip_data;
	struct ts_chip_info_param *info = (struct ts_chip_info_param *)in_cmd->cmd_param.prv_params;

	TP_LOG_INFO("get chip info called\n");

	if (dev->ops->chip_get_info)
		error = dev->ops->chip_get_info(info);

	if (error)
		info->status = TS_ACTION_FAILED;
	else
		info->status = TS_ACTION_SUCCESS;

	return error;
}
static int tp_set_info_flag(struct ts_cmd_node *in_cmd, struct ts_cmd_node *out_cmd)
{
	int error = NO_ERR;
	struct ts_device_data *dev = g_tp_data.chip_data;
	struct ts_data *info = (struct ts_data *)in_cmd->cmd_param.prv_params;

	TP_LOG_INFO("ts_set_info_flag called\n");

	if (dev->ops->chip_set_info_flag)
		error = dev->ops->chip_set_info_flag(info);
	return error;
}
static inline int tp_force_reset(struct ts_cmd_node *in_cmd, struct ts_cmd_node *out_cmd)
{
	int error = NO_ERR;
	struct ts_device_data *dev = g_tp_data.chip_data;

	TP_LOG_INFO("ts force reset called\n");

	if (dev->ops->chip_reset)
		error = dev->ops->chip_reset();

	if (error) {
		out_cmd->command = TP_ERR_OCCUR;
		goto out;
	}

out:
	return error;
}

static inline int tp_read_rawdata(struct ts_cmd_node *in_cmd, struct ts_cmd_node *out_cmd)
{
	int error = NO_ERR;
	struct ts_device_data *dev = g_tp_data.chip_data;
	struct ts_rawdata_info *info = (struct ts_rawdata_info *)in_cmd->cmd_param.prv_params;

	TP_LOG_INFO("ts read rawdata called\n");

	if (dev->ops->chip_get_rawdata)
		error = dev->ops->chip_get_rawdata(info, out_cmd);

	if (!error) {
		TP_LOG_INFO("read rawdata success\n");
		info->status = TS_ACTION_SUCCESS;
		info->time_stamp = ktime_get();
		goto out;
	}

	info->status = TS_ACTION_FAILED;
	TP_LOG_ERR("read rawdata failed :%d\n", error);

out:
	return error;
}

static inline int tp_palm_switch(struct ts_cmd_node *in_cmd, struct ts_cmd_node *out_cmd)
{
	int error = -EIO;
	struct ts_device_data *dev = g_tp_data.chip_data;
	struct ts_palm_info *info = (struct ts_palm_info *)in_cmd->cmd_param.prv_params;

	TP_LOG_DEBUG("palm action :%d, value:%d", info->op_action, info->palm_switch);

	if (dev->ops->chip_palm_switch)
		error = dev->ops->chip_palm_switch(info);

	if (error)
		info->status = TS_ACTION_FAILED;
	else
		info->status = TS_ACTION_SUCCESS;

	TP_LOG_DEBUG("palm switch process result: %d\n", error);

	return error;
}

static inline int tp_touch_weight_switch(struct ts_cmd_node *in_cmd, struct ts_cmd_node *out_cmd)
{
	int error = -EIO;
	struct ts_device_data *dev = g_tp_data.chip_data;
	struct ts_single_touch_info *info = (struct ts_single_touch_info *)in_cmd->cmd_param.prv_params;

	TP_LOG_DEBUG("weight action :%d, value:%d", info->op_action, info->single_touch_switch);

	if (dev->ops->chip_single_touch_switch)
		error = dev->ops->chip_single_touch_switch(info);

	if (error)
		info->status = TS_ACTION_FAILED;
	else
		info->status = TS_ACTION_SUCCESS;

	TP_LOG_DEBUG("touch_weight switch process result: %d\n", error);

	return error;
}

static inline int tp_glove_switch(struct ts_cmd_node *in_cmd, struct ts_cmd_node *out_cmd)
{
	int error = -EIO;
	struct ts_device_data *dev = g_tp_data.chip_data;
	struct ts_glove_info *info = (struct ts_glove_info *)in_cmd->cmd_param.prv_params;

	TP_LOG_DEBUG("glove action :%d, value:%d", info->op_action, info->glove_switch);

	if (dev->ops->chip_glove_switch)
		error = dev->ops->chip_glove_switch(info);

	if (error)
		info->status = TS_ACTION_FAILED;
	else
		info->status = TS_ACTION_SUCCESS;

	TP_LOG_DEBUG("glove switch process result: %d\n", error);

	return error;
}

static inline int tp_wakeup_gesture_enable_switch(struct ts_cmd_node *in_cmd, struct ts_cmd_node *out_cmd)
{
	int error = -EIO;
	struct ts_device_data *dev = g_tp_data.chip_data;
	struct ts_wakeup_gesture_enable_info *info = (struct ts_wakeup_gesture_enable_info *)in_cmd->cmd_param.prv_params;

	TP_LOG_INFO("%s: write value: %d", __func__, info->switch_value);

	if (atomic_read(&g_tp_data.state) == TS_WORK_IN_SLEEP && dev->ops->chip_wakeup_gesture_enable_switch){
		error = dev->ops->chip_wakeup_gesture_enable_switch(info);
	}

	info->op_action = TS_ACTION_UNDEF;
	if (error) {
		info->status = TS_ACTION_FAILED;
		TP_LOG_ERR("%s, process error: %d\n", __func__, error);
	} else {
		info->status = TS_ACTION_SUCCESS;
	}

	TP_LOG_DEBUG("%s, process result: %d\n", __func__, error);

	return error;
}

static inline int tp_holster_switch(struct ts_cmd_node *in_cmd, struct ts_cmd_node *out_cmd)
{
	int error = -EIO;
	struct ts_device_data *dev = g_tp_data.chip_data;
	struct ts_holster_info *info = (struct ts_holster_info *)in_cmd->cmd_param.prv_params;

	TP_LOG_DEBUG("Holster action :%d, value:%d", info->op_action, info->holster_switch);

	if (dev->ops->chip_holster_switch)
		error = dev->ops->chip_holster_switch(info);

	if (error) {
		info->status = TS_ACTION_FAILED;
		TP_LOG_ERR("holster switch process error: %d\n", error);
	} else
		info->status = TS_ACTION_SUCCESS;

	TP_LOG_DEBUG("holster switch process result: %d\n", error);

	return error;
}

static inline int tp_roi_switch(struct ts_cmd_node *in_cmd, struct ts_cmd_node *out_cmd)
{
	int error = -EIO;
	struct ts_device_data *dev = g_tp_data.chip_data;
	struct ts_roi_info *info = (struct ts_roi_info *)in_cmd->cmd_param.prv_params;

	TP_LOG_DEBUG("roi action :%d, value:%d", info->op_action, info->roi_switch);

	if (dev->ops->chip_roi_switch)
		error = dev->ops->chip_roi_switch(info);

	if (error) {
		info->status = TS_ACTION_FAILED;
		TP_LOG_ERR("roi switch process error: %d\n", error);
	} else
		info->status = TS_ACTION_SUCCESS;

	TP_LOG_DEBUG("roi switch process result: %d\n", error);

	return error;
}


static inline int tp_touch_window(struct ts_cmd_node *in_cmd, struct ts_cmd_node *out_cmd)
{
	struct ts_window_info *info = (struct ts_window_info *)in_cmd->cmd_param.prv_params;

	g_tp_data.feature_info.window_info.window_enable = info->window_enable;
	g_tp_data.feature_info.window_info.top_left_x0 = info->top_left_x0;
	g_tp_data.feature_info.window_info.top_left_y0 = info->top_left_y0;
	g_tp_data.feature_info.window_info.bottom_right_x1 = info->bottom_right_x1;
	g_tp_data.feature_info.window_info.bottom_right_y1 = info->bottom_right_y1;

	info->status = TS_ACTION_SUCCESS;

	return NO_ERR;
}

static inline int tp_hand_detect(struct ts_cmd_node *in_cmd, struct ts_cmd_node *out_cmd)
{
	int error = -EIO;
	struct ts_device_data *dev = g_tp_data.chip_data;
	struct ts_hand_info *info = (struct ts_hand_info *)in_cmd->cmd_param.prv_params;

	if (dev->ops->chip_hand_detect)
		error = dev->ops->chip_hand_detect(info);

	if (error)
		info->status = TS_ACTION_FAILED;
	else
		info->status = TS_ACTION_SUCCESS;

	return error;
}

static inline int tp_chip_regs_operate(struct ts_cmd_node *in_cmd, struct ts_cmd_node *out_cmd)
{
	int error = -EIO;
	struct ts_device_data *dev = g_tp_data.chip_data;
	struct ts_regs_info *info = (struct ts_regs_info *)in_cmd->cmd_param.prv_params;

	if (dev->ops->chip_regs_operate)
		error = dev->ops->chip_regs_operate(info);

	if (error < 0)
		info->status = TS_ACTION_FAILED;
	else
		info->status = TS_ACTION_SUCCESS;

	return error;
}

static inline int tp_err_process(struct ts_cmd_node *in_cmd, struct ts_cmd_node *out_cmd)
{
	static int error_count = 0;
	int error = NO_ERR;
	struct ts_device_data *dev = g_tp_data.chip_data;

	TP_LOG_INFO("error process\n");

	if (dev->ops->chip_reset)
		error = dev->ops->chip_reset();
	if (error) {//error nest occurred, we define nest level
		out_cmd->command = TP_ERR_OCCUR;
		//BUG_ON(unlikely(++error_count == TS_ERR_NEST_LEVEL));
		goto out;
	}

	error_count = 0;
out:
	return error;
}

static inline int tp_int_err_process(struct ts_cmd_node *in_cmd, struct ts_cmd_node *out_cmd)
{
	int error = NO_ERR;
	struct ts_device_data *dev = g_tp_data.chip_data;

	if (dev->ops->chip_reset)
		error = dev->ops->chip_reset();

	if (error) {//error nest occurred, we define nest level
		out_cmd->command = TP_ERR_OCCUR;
		goto out;
	}

out:
	return error;
}

static inline int tp_test_cmd(struct ts_cmd_node *in_cmd, struct ts_cmd_node *out_cmd)
{
	int error = NO_ERR;
	struct ts_device_data *dev = g_tp_data.chip_data;

	if (dev->ops->chip_test)
		error = dev->ops->chip_test(in_cmd, out_cmd);

	if (error) {
		out_cmd->command = TP_ERR_OCCUR;
		goto out;
	}

out:
	return error;
}

static inline int tp_check_status(struct ts_cmd_node *in_cmd, struct ts_cmd_node *out_cmd)
{
	int error = NO_ERR;
	struct ts_device_data *dev = g_tp_data.chip_data;

	if (dev->ops->chip_check_status)
		error = dev->ops->chip_check_status();
	if (error) {
		out_cmd->command = TP_ERR_OCCUR;
	}

	tp_start_wd_timer(&g_tp_data);
	return error;
}

static inline bool tp_cmd_need_process(struct ts_cmd_node *cmd)
{
	bool is_need_process = true;
	struct ts_cmd_sync *sync = cmd->sync;
	enum ts_pm_type pm_type = cmd->cmd_param.pub_params.pm_type;
	if (unlikely((atomic_read(&g_tp_data.state) == TS_SLEEP) || (atomic_read(&g_tp_data.state) == TS_WORK_IN_SLEEP))) {
		if(atomic_read(&g_tp_data.state) == TS_SLEEP)
		{
			switch(cmd->command) {
			case TS_POWER_CONTROL:
				if ((pm_type != TS_RESUME_DEVICE) && (pm_type != TS_AFTER_RESUME))
					is_need_process = false;
				break;
			case TS_TOUCH_WINDOW:
				is_need_process = true;
				break;
			case TS_GET_CHIP_INFO:
				is_need_process = true;
				break;
			case TS_INT_PROCESS:
			case TS_INT_ERR_OCCUR:
				enable_irq(g_tp_data.irq_id);
				is_need_process = false;
				break;
			default:
				is_need_process = false;
				break;
			}
		}
		else
		{
			switch(cmd->command) {
			case TS_POWER_CONTROL:
				if ((pm_type != TS_RESUME_DEVICE) && (pm_type != TS_AFTER_RESUME))
					is_need_process = false;
				break;
			case TS_TOUCH_WINDOW:
				is_need_process = true;
				break;
			default:
				is_need_process = true;
				break;
			}
		}
	}

	if(!is_need_process && sync) {
		if (atomic_read(&sync->timeout_flag) == TS_TIMEOUT) {
			kfree(sync);
		} else {
			complete(&sync->done);
		}
	}

	return is_need_process;
}
static int tp_proc_command(struct ts_cmd_node *cmd)
{
	int error = NO_ERR;
	struct ts_cmd_sync *sync = cmd->sync;
	struct ts_cmd_node *proc_cmd = cmd;
	struct ts_cmd_node *out_cmd = &pang_cmd_buff;

	if (!tp_cmd_need_process(proc_cmd)) {
		TP_LOG_INFO("no need to process cmd:%d", proc_cmd->command);
		goto out;
	}

related_proc:
	out_cmd->command = TS_INVAILD_CMD;

	switch(proc_cmd->command) {
		case TS_INT_PROCESS:
			tp_proc_bottom_half(proc_cmd, out_cmd);
			enable_irq(g_tp_data.irq_id);
			break;
		case TS_INPUT_ALGO:
			tp_algo_calibrate(proc_cmd, out_cmd);
			break;
		case TS_REPORT_INPUT:
			tp_report_input(proc_cmd, out_cmd);
			break;
		case TS_POWER_CONTROL:
			tp_power_control(g_tp_data.irq_id, proc_cmd, out_cmd);
			break;
		case TS_FW_UPDATE_BOOT:
			disable_irq(g_tp_data.irq_id);
			tp_fw_update_boot(proc_cmd, out_cmd);
			enable_irq(g_tp_data.irq_id);
			break;
		case TS_FW_UPDATE_SD:
			disable_irq(g_tp_data.irq_id);
			tp_fw_update_sd(proc_cmd, out_cmd);
			enable_irq(g_tp_data.irq_id);
			break;
		case TS_READ_RAW_DATA:
			disable_irq(g_tp_data.irq_id);
			tp_read_rawdata(proc_cmd, out_cmd);
			enable_irq(g_tp_data.irq_id);
			break;
		case TS_GET_CHIP_INFO:
			tp_get_chip_info(proc_cmd, out_cmd);
			break;
		case TS_SET_INFO_FLAG:
			tp_set_info_flag(proc_cmd, out_cmd);
			break;
		case TS_CALIBRATE_DEVICE:
			tp_calibrate(proc_cmd, out_cmd);
			break;
		case TS_DSM_DEBUG:
			tp_dsm_debug(proc_cmd, out_cmd);
			break;
		case TS_GLOVE_SWITCH:
			tp_glove_switch(proc_cmd, out_cmd);
			break;
		case TS_TOUCH_WEIGHT_SWITCH:
			tp_touch_weight_switch(proc_cmd, out_cmd);
			break;
		case TS_PALM_SWITCH:
			tp_palm_switch(proc_cmd, out_cmd);
			break;
		case TS_HAND_DETECT:
			tp_hand_detect(proc_cmd, out_cmd);
			break;
		case TS_FORCE_RESET:
			disable_irq(g_tp_data.irq_id);
			tp_stop_wd_timer(&g_tp_data);
			tp_force_reset(proc_cmd, out_cmd);
			tp_start_wd_timer(&g_tp_data);
			enable_irq(g_tp_data.irq_id);
			break;
		case TS_INT_ERR_OCCUR:
			tp_stop_wd_timer(&g_tp_data);
			tp_int_err_process(proc_cmd, out_cmd);
			enable_irq(g_tp_data.irq_id);
			tp_start_wd_timer(&g_tp_data);
			break;
		case TP_ERR_OCCUR:
			disable_irq(g_tp_data.irq_id);
			tp_stop_wd_timer(&g_tp_data);
			tp_err_process(proc_cmd, out_cmd);
			tp_start_wd_timer(&g_tp_data);
			enable_irq(g_tp_data.irq_id);
			break;
		case TS_CHECK_STATUS:
			tp_check_status(proc_cmd, out_cmd);
			break;
		case TS_WAKEUP_GESTURE_ENABLE:
			tp_wakeup_gesture_enable_switch(proc_cmd, out_cmd);
			break;
		case TS_HOLSTER_SWITCH:
			tp_holster_switch(proc_cmd, out_cmd);
			break;
		case TS_ROI_SWITCH:
			tp_roi_switch(proc_cmd, out_cmd);
			break;
		case TS_TOUCH_WINDOW:
			tp_touch_window(proc_cmd, out_cmd);
			break;
		case TS_REGS_STORE:
			tp_chip_regs_operate(proc_cmd, out_cmd);
			break;
		case TS_TEST_CMD:
			tp_test_cmd(proc_cmd, out_cmd);
		default:
			break;
	}

	TP_LOG_DEBUG("command :%d process result:%d \n", proc_cmd->command, error);

	if (out_cmd->command != TS_INVAILD_CMD) {
		TP_LOG_DEBUG("related command :%d  need process\n", out_cmd->command);
		swap(proc_cmd, out_cmd);//ping - pang
		goto related_proc;
	}

	if (sync) {//notify wait threads by completion
		smp_mb();
		TP_LOG_DEBUG("wakeup threads in waitqueue\n");
		if (atomic_read(&sync->timeout_flag) == TS_TIMEOUT) {
			kfree(sync);
		} else {
			complete(&sync->done);
		}
	}

out:
	return error;
}

static bool tp_task_continue(void)
{
	bool task_continue = true;
	unsigned long flags;
	TP_LOG_DEBUG("prepare enter idle\n");

repeat:
	if (unlikely(kthread_should_stop())) {
		task_continue = false;
		goto out;
	}
	spin_lock_irqsave(&g_tp_data.queue.spin_lock, flags);
	smp_wmb();
	if (g_tp_data.queue.cmd_count) {
		set_current_state(TASK_RUNNING);
		TP_LOG_DEBUG("ts task state to  TASK_RUNNING\n");
		goto out_unlock;
	} else {
		set_current_state(TASK_INTERRUPTIBLE);
		TP_LOG_DEBUG("ts task state to  TASK_INTERRUPTIBLE\n");
		spin_unlock_irqrestore(&g_tp_data.queue.spin_lock, flags);
		schedule();
		goto repeat;
	}

out_unlock:
	spin_unlock_irqrestore(&g_tp_data.queue.spin_lock, flags);
out:
	return task_continue;
}

static int tp_input_open(struct input_dev *dev)
{

	TP_LOG_DEBUG("input_open called:%d\n", dev->users);
	return NO_ERR;
}

static void tp_input_close(struct input_dev *dev)
{
	TP_LOG_DEBUG("input_close called:%d\n", dev->users);
}

static int tp_chip_detect(const char *chip_name)
{
	bool found = false;
	int index;
	int error = -EIO;
	int len;
	const __be32 *addr;
	struct device_node *child= NULL;
	struct device_node *root = g_tp_data.node;
	struct ts_device_data *ptr = &g_ts_device_map[0];
	for_each_child_of_node(root, child) {//find the chip node
	//	if (of_device_is_compatible(child, chip_name)) {
		error = of_device_is_compatible(child,chip_name);
		if(error){
			found = true;
			break;
		}
	}

	if (!found) {
		TP_LOG_ERR("not find chip :%s's node\n", chip_name);
		error = -EINVAL;
		goto out;
	}
	g_tp_data.dev_node=child;
	if (g_tp_data.bops->btype == TS_BUS_I2C) {//i2c ts need get chip addr
		addr = of_get_property(child, "slave_address", &len);
		if (!addr || (len < sizeof(int))) {
			TP_LOG_ERR("invalid slave_address on %s, len:%d\n", chip_name, len);
			goto out;
		}
		if (g_tp_data.client->addr != be32_to_cpup(addr)) {
			error = i2c_check_addr_busy(g_tp_data.client->adapter, be32_to_cpup(addr));
			if (error) {
				TP_LOG_ERR("0x%x slave addr conflict\n", *addr);
				goto out;
			}
			TP_LOG_DEBUG("slave addr :0x%x not occupied\n", *addr);
			g_tp_data.client->addr = be32_to_cpup(addr);
		}
	}

	for(index = 0; ptr!= NULL && index< ARRAY_SIZE(g_ts_device_map); ptr++, index++)
	{
		if (!strcmp (chip_name, ptr->chip_name)) {
			if (g_tp_data.bops->btype == TS_BUS_I2C)
				ptr->slave_addr = g_tp_data.client->addr;

			ptr->bops = g_tp_data.bops;
			INIT_LIST_HEAD(&ptr->algo_head);
			g_tp_data.chip_data = ptr;

			//have parse dt func , execute it now
			if (ptr->ops->chip_parse_config && ptr->ops->chip_parse_config(child, ptr)) {
				TP_LOG_ERR("call %s's chip self parse config failed\n", chip_name);
				error = -EINVAL;
				goto out;
			}

			if (ptr->ops->chip_detect) {
				TP_LOG_DEBUG("call chip self init handle\n");
				error = ptr->ops->chip_detect(child, ptr, g_tp_data.ts_dev);
			}
			goto out;
		}
	}
	if (ARRAY_SIZE(g_ts_device_map) == index) {
		TP_LOG_ERR("we can not find %s's chip data in device map\n", chip_name);
		error = -EINVAL;
	}
out:
	return error;
}

static int tp_get_support_chip(void)
{
	struct device_node *np = g_tp_data.node;
	const char *support_chip_name = NULL;
	//const char *bus_type;
	int rc = 0;
	int index = 0;
	int found = -ENXIO;
	int array_len = 0;

	if (np) {//support dts
		array_len = of_property_count_strings(np, "support_chip_name");
		if (array_len <= 0) {
			TP_LOG_ERR("chip name length invaild:%d\n", array_len);
			return -EINVAL;
		}

		for(index=0; index<array_len; index++)//try to detect active ts ic
		{
			rc = of_property_read_string_index(np,
				"support_chip_name", index,
				&support_chip_name);
			if (rc) {
				TP_LOG_ERR("read %d - chip name :%s, err:%d\n", index, support_chip_name, rc);
				continue;
			}
			if (!tp_chip_detect(support_chip_name)) {
				found = NO_ERR;
				TP_LOG_INFO("chip: %s found success\n", support_chip_name);
				break;
			}
		}
	} else {//not support dts
		TP_LOG_ERR("dts feature not support: %d\n", rc);
	}
	return found;
}

static int tp_parse_config(void)
{
	int error = NO_ERR;
	int rc;
	int index;
	char *tmp_buff = NULL;

	if (g_tp_data.node) {
		rc = of_property_read_string(g_tp_data.node, "product", (const char **)&tmp_buff);
		if (rc) {
			TP_LOG_ERR("product read failed:%d\n", rc);
			error = -EINVAL;
			goto out;
		}
	}

	if (!tmp_buff) {
		TP_LOG_ERR("tmp_buff failed\n");
		error = -EINVAL;
		goto out;
	}

	for(index = 0; tmp_buff[index] && index < strlen(tmp_buff); index++)//exchange name to lower
		g_tp_data.product_name[index] = tolower(tmp_buff[index]);

	if (index == MAX_STR_LEN)
		g_tp_data.product_name[MAX_STR_LEN-1] = '\0';

out:
	return error;
}

static ssize_t tp_get_virtual_keys_range(void)
{
	const __be32 *values = NULL;
	int size = 0;
	int len = 0;
	u16 *val_array = NULL;
	int ret = 0;
	int count = 0;
	values = of_get_property(g_tp_data.node, "virtual_keys", &len);
	if (NULL == values){
		TP_LOG_ERR("%s:of_get_property read fail, line=%d\n", __func__,__LINE__);
		ret = -EINVAL;
		goto fail;
	}
	size = len / sizeof(u32);
	val_array = kzalloc(size * sizeof(u16), GFP_KERNEL);
	if (NULL == val_array) {
		TP_LOG_ERR("%s:memory not enough,fail line=%d\n", __func__,__LINE__);
		ret = -EINVAL;
		goto fail;
	}

	for (count = 0; count < size; count++){
		val_array[count] = (u16)be32_to_cpup(values++);
	}

	if (size % VIRTUAL_KEY_ELEMENT_SIZE) {
		ret = -ENOMEM;
		TP_LOG_ERR("%s:fail line=%d virtual keys size error\n", __func__,__LINE__);
		goto fail_del_values;
		}
	g_tp_data.virtual_keys_size = size;
	g_tp_data.virtual_keys_values = val_array;
	return ret;

fail_del_values:
	kfree(val_array);
fail:
	return ret;
}

static int tp_parse_virtualkey(void)
{
	int error = NO_ERR;

	if (g_tp_data.node) {
		error = of_property_read_u32(g_tp_data.node, "has_virtualkey", &g_tp_data.chip_data->has_virtualkey);
		if (error) {
			TP_LOG_ERR("%s:get has_virtualkey failed\n",__func__);
			return -EINVAL;
		}
	}
	if(g_tp_data.chip_data->has_virtualkey) {
		error = of_property_read_u32(g_tp_data.node,"lcd_full", &g_tp_data.chip_data->lcd_full);
		if (error) {
			TP_LOG_ERR("%s:get device lcd_full failed\n",__func__);
			error = -EINVAL;
		}
		error = tp_get_virtual_keys_range();
		if(error) {
			TP_LOG_ERR("%s:fail to get virtual_keys_range form dts\n",__func__);
			error = -EINVAL;
		}
	}
	TP_LOG_INFO("ts_data.chip_data->has_virtualkey = %d\n", g_tp_data.chip_data->has_virtualkey);
	return error;
}

static int tp_parse_roi_supported(void)
{
	int error = NO_ERR;

	if (g_tp_data.node) {
		error = of_property_read_u8(g_tp_data.node, "roi_supported", &g_tp_data.feature_info.roi_info.roi_supported);
		if (error) {
			TP_LOG_ERR("%s:get roi_supported failed\n",__func__);
			g_tp_data.feature_info.roi_info.roi_supported = 0;
			return -EINVAL;
		}
		TP_LOG_INFO("roi_supported = %d\n", g_tp_data.feature_info.roi_info.roi_supported);

		error = of_property_read_u16(g_tp_data.node, "roi_control_addr_offset", &g_tp_data.feature_info.roi_info.roi_control_addr_offset);
		if (error) {
			TP_LOG_ERR("%s:get roi_control_addr_offset failed, used default.\n",__func__);
			g_tp_data.feature_info.roi_info.roi_control_addr_offset = ROI_CTRL_DEFAULT_OFFSET;
			error = NO_ERR;
		}
		TP_LOG_INFO("roi_control_addr_offset = %d\n", g_tp_data.feature_info.roi_info.roi_control_addr_offset);

		error = of_property_read_u16(g_tp_data.node, "roi_data_addr_offset", &g_tp_data.feature_info.roi_info.roi_data_addr_offset);
		if (error) {
			TP_LOG_ERR("%s:get roi_data_addr_offset failed, used default.\n",__func__);
			g_tp_data.feature_info.roi_info.roi_data_addr_offset = ROI_DATA_DEFAULT_OFFSET;
			error = NO_ERR;
		}
		TP_LOG_INFO("roi_data_addr_offset = %d\n", g_tp_data.feature_info.roi_info.roi_data_addr_offset);
	}

	return error;
}


static int tp_parse_slide_supported(void)
{
	int error = NO_ERR;
	u32 ts_slide_supported = 0;
	u32 chip_slide_supported = 0;

	if (g_tp_data.node) {//touchscreen slide support
		error = of_property_read_u32(g_tp_data.node,"ts_slide_supported",&ts_slide_supported);
		if (error) {
			TP_LOG_ERR("%s:get ts_slide_supported failed\n",__func__);
			g_tp_data.feature_info.slide_info.slide_supported = 0;
			return -EINVAL;
		}
	}

	if(g_tp_data.dev_node){//chip slide support
		error = of_property_read_u32(g_tp_data.dev_node,"chip_slide_supported",&chip_slide_supported);
		if (error) {
			TP_LOG_ERR("%s:get chip_slide_supported failed\n",__func__);
			g_tp_data.feature_info.slide_info.slide_supported = 0;
			return -EINVAL;
		}
	}
	g_tp_data.feature_info.slide_info.slide_supported = ts_slide_supported && chip_slide_supported;

	TP_LOG_INFO("ts_slide_supported=%d  chip_slide_supported=%d slide_supported=%d\n",
		ts_slide_supported,chip_slide_supported,g_tp_data.feature_info.slide_info.slide_supported);

	return error;
}

static int get_tp_bus_info(void)
{
	const char *bus_type;

	int rc;
	int error = NO_ERR;
	u32 bus_id = 0;

	g_tp_data.node = NULL;
	g_tp_data.node = of_find_compatible_node(NULL, NULL, TP_DEV_NAME);
	if (!g_tp_data.node) {
		TP_LOG_ERR("can't find ts module node\n");
		error = -EINVAL;
		goto out;
	}

	rc = of_property_read_string(g_tp_data.node, "bus_type", &bus_type);
	if (rc) {
		TP_LOG_ERR("bus type read failed:%d\n", rc);
		error = -EINVAL;
		goto out;
	}
	if (!strcmp (bus_type,"i2c")) {
		g_tp_data.bops = &tp_bus_i2c_info;
	} else if (!strcmp (bus_type,"spi")) {
		g_tp_data.bops = &tp_bus_spi_info;
	} else {
		TP_LOG_ERR("bus type invaild:%s\n", bus_type);
		error = -EINVAL;
	}

	rc = of_property_read_u32(g_tp_data.node, "bus_id", &bus_id);
	if (rc) {
		TP_LOG_ERR("bus id read failed\n");
		error = -EINVAL;
		goto out;
	}
	g_tp_data.bops->bus_id = bus_id;
	TP_LOG_DEBUG("bus id :%d\n", bus_id);

out:
	return error;
}

static int tp_chip_init(void)
{
	int error = NO_ERR;
	struct ts_device_data *dev = g_tp_data.chip_data;

	TP_LOG_INFO("ts_chip_init called\n");
	mutex_init(&easy_wake_guesure_lock);
	if (dev->ops->chip_init){
		error = dev->ops->chip_init();
	}
	if (error) {
		TP_LOG_ERR("chip init failed\n");
	}

	return error;
}

static int tp_register_algo(void)
{
	int error = NO_ERR;
	struct ts_device_data *dev = g_tp_data.chip_data;

	TP_LOG_INFO("register algo called\n");

	error = pad_ts_register_algo_func(dev);

	return error;
}

static void tp_ic_shutdown(void)
{
	struct ts_device_data *dev = g_tp_data.chip_data;
	if (dev->ops->chip_shutdown)
		dev->ops->chip_shutdown();
	return;
}

static ssize_t tp_virtual_keys_show(struct kobject *kobj,
			       struct kobj_attribute *attr, char *buf)
{
	u16 *data = g_tp_data.virtual_keys_values;
	int size = g_tp_data.virtual_keys_size;
	int index = 0;
	int i = 0;
	int count = 0;
	TP_LOG_ERR("%s:keys_show begin\n",__func__);

	for (i = 0; i < size; i += VIRTUAL_KEY_ELEMENT_SIZE)
	{
		count = scnprintf(buf + index, MAX_PRBUF_SIZE - index,
			"0x01:%d:%d:%d:%d:%d\n",
			data[i], data[i + 1], data[i + 2], data[i + 3], data[i + 4]);

		if (count > 0) {
			index += count;
		} else {
			TP_LOG_ERR("%s:print to buff error,err num=%d\n",__func__,count);
			return count;
		}
	}

	return index;
}

static struct kobj_attribute virtual_keys_attr = {
	.attr = {
		.name = "virtualkeys.zeusis,touchpad",
		.mode = S_IRUGO,
	},
	.show = &tp_virtual_keys_show,
};

static struct attribute *properties_attrs[] = {
	&virtual_keys_attr.attr,
	NULL
};

static struct attribute_group properties_attr_group = {
	.attrs = properties_attrs,
};

static void __init tp_virtual_keys_init(void)
{
	struct kobject *properties_kobj;
	int ret = 0;

	properties_kobj = kobject_create_and_add("board_properties", NULL);
	if (properties_kobj)
		ret = sysfs_create_group(properties_kobj,
			&properties_attr_group);
	if (!properties_kobj || ret)
		pr_err("%s: failed to create board_properties!\n", __func__);
}

static int tp_try_update_firmware(void)
{
	char joint_chr = '_';
	int error = NO_ERR;
	char *fw_name;
	struct ts_cmd_node cmd;

	memset(&cmd, 0, sizeof(struct ts_cmd_node));
	cmd.command = TS_FW_UPDATE_BOOT;
	fw_name = cmd.cmd_param.pub_params.firmware_info.fw_name;

	/*firmware name [product_name][ic_name][module][vendor]*/
	strncat(fw_name, g_tp_data.product_name, MAX_STR_LEN);
	strncat(fw_name, &joint_chr, 1);
	strncat(fw_name, g_tp_data.chip_data->chip_name, MAX_STR_LEN);
	strncat(fw_name, &joint_chr, 1);
#if 0
	strncat(fw_name, &joint_chr, 1);
	strncat(fw_name, g_tp_data.chip_data->module_name, MAX_STR_LEN);
	strncat(fw_name, &joint_chr, 1);
	strncat(fw_name, g_tp_data.chip_data->version_name, MAX_STR_LEN);
#endif
//open fw boot upgrade bill 2016-10-14
    error = pad_put_one_cmd(&cmd, NO_SYNC_TIMEOUT);

	return error;
}

static int tp_init(void)
{
	int error = NO_ERR;
	unsigned int touch_recovery_mode = 0;
	unsigned int irq_flags;
	struct input_dev *input_dev;
	struct input_dev *slide_dev;

	atomic_set(&g_tp_data.state, TS_UNINIT);

	g_tp_data.queue.rd_index = 0;
	g_tp_data.queue.wr_index = 0;
	g_tp_data.queue.cmd_count = 0;
	g_tp_data.queue.queue_size = TP_CMD_QUEUE_SIZE;
	spin_lock_init(&g_tp_data.queue.spin_lock);
	TP_LOG_DEBUG("ts init: cmd queue size : %d\n", TP_CMD_QUEUE_SIZE);
	wake_lock_init(&g_tp_data.ts_wake_lock, WAKE_LOCK_SUSPEND, "tp_wake_lock");

	error = tp_parse_config();
	if (error) {
		TP_LOG_ERR("ts parse config failed : %d\n", error);
		goto err_out;
	}

	g_tp_data.ts_dev = platform_device_alloc("zeusis_touchpad", -1);
	if (!g_tp_data.ts_dev) {
		TP_LOG_ERR("platform device malloc failed\n");
		error = -ENOMEM;
		goto err_out;
	}

	error = platform_device_add(g_tp_data.ts_dev);
	if (error){
		TP_LOG_ERR("platform device add failed :%d\n", error);
		goto err_put_platform_dev;
	}

	error = tp_get_support_chip();
	if (error) {
		TP_LOG_ERR("get support chip failed : %d\n", error);
		goto err_del_platform_dev;
	}

	error = tp_parse_virtualkey();
	if (error) {
		TP_LOG_ERR("ts parse vitualkey config failed : %d\n", error);
		goto err_out;
	}

	error = tp_parse_slide_supported();
	if (error) {
		TP_LOG_ERR("ts parse slide supported config failed : %d\n", error);
	}

	error = tp_parse_roi_supported();
	if (error) {
		TP_LOG_ERR("ts parse roi supported config failed : %d\n", error);
	}

	error = tp_chip_init();
	if (error) {
		TP_LOG_ERR("chip init failed : %d,  try fw update again\n", error);
	}

	error = tp_register_algo();
	if (error) {
		TP_LOG_ERR("ts register algo failed : %d\n", error);
		goto err_del_platform_dev;
	}

	if (1 == g_tp_data.chip_data->has_virtualkey) {
		TP_LOG_INFO("g_tp_data.chip_data->has_virtualkey = %d\n", g_tp_data.chip_data->has_virtualkey);
		tp_virtual_keys_init();
	}

	input_dev = input_allocate_device();
	if (!input_dev) {
		TP_LOG_ERR("failed to allocate memory for input dev\n");
		error = -ENOMEM;
		goto err_del_platform_dev;
	}

	//input_dev->name = TP_DEV_NAME;
	input_dev->name = TS_SLIDE_DEV_NAME;
	if (g_tp_data.bops->btype == TS_BUS_I2C)
		input_dev->id.bustype = BUS_I2C;
	else if (g_tp_data.bops->btype == TS_BUS_SPI)
		input_dev->id.bustype = BUS_SPI;
	input_dev->dev.parent = &g_tp_data.ts_dev->dev;
	input_dev->open = tp_input_open;
	input_dev->close = tp_input_close;
	g_tp_data.input_dev = input_dev;

	if (g_tp_data.chip_data->ops->chip_input_config) //config input for diff chip
		error = g_tp_data.chip_data->ops->chip_input_config(g_tp_data.input_dev);
	if (error)
		goto err_free_dev;

	input_set_drvdata(input_dev, &g_tp_data);

	error = input_register_device(input_dev);
	if (error) {
		TP_LOG_ERR("input dev register failed : %d\n", error);
		goto err_free_dev;
	}

	//register slide key devices
	if(g_tp_data.feature_info.slide_info.slide_supported){
		slide_dev = input_allocate_device();
		if (!slide_dev) {
			TP_LOG_ERR("failed to allocate memory for input slide dev\n");
			error = -ENOMEM;
			goto err_del_platform_dev;
		}

		slide_dev->name = TS_SLIDE_DEV_NAME;
		if (g_tp_data.bops->btype == TS_BUS_I2C)
			slide_dev->id.bustype = BUS_I2C;
		else if (g_tp_data.bops->btype == TS_BUS_SPI)
			slide_dev->id.bustype = BUS_SPI;
		slide_dev->dev.parent = &g_tp_data.ts_dev->dev;
		slide_dev->open = tp_input_open;
		slide_dev->close = tp_input_close;
		g_tp_data.slide_dev = slide_dev;

		if (g_tp_data.chip_data->ops->chip_slide_config) //config input for diff chip
			error = g_tp_data.chip_data->ops->chip_slide_config(g_tp_data.slide_dev);
		if (error)
			goto err_free_dev;

		input_set_drvdata(slide_dev, &g_tp_data);

		error = input_register_device(slide_dev);
		if (error) {
			TP_LOG_ERR("input slide dev register failed : %d\n", error);
			goto err_free_dev;
		}
	}

#if defined(CONFIG_FB)
	g_tp_data.fb_notify.notifier_call = tp_fb_notifier_callback;
	error = fb_register_client(&g_tp_data.fb_notify);
	if (error){
		TP_LOG_ERR("unable to register fb_notifier: %d\n", error);
		goto err_free_input_dev;
	}
#elif defined(CONFIG_HAS_EARLYSUSPEND)
	g_tp_data.early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + TS_SUSPEND_LEVEL;
	g_tp_data.early_suspend.suspend = tp_early_suspend;
	g_tp_data.early_suspend.resume = tp_late_resume;
	register_early_suspend(&g_tp_data.early_suspend);
#endif

	error = sysfs_create_group(&g_tp_data.ts_dev->dev.kobj, &ts_attr_group);
	if (error) {
		TP_LOG_ERR("can't create ts's sysfs\n");
		goto err_unregister_suspend;
	}

	tp_procfs_create();

	g_tp_data.irq_id= g_tp_data.client->irq = gpio_to_irq(g_tp_data.chip_data->irq_gpio);
	switch (g_tp_data.chip_data->irq_config) {
		case TS_IRQ_LOW_LEVEL:
			irq_flags = IRQF_TRIGGER_LOW;
			break;
		case TS_IRQ_HIGH_LEVEL:
			irq_flags = IRQF_TRIGGER_HIGH;
			break;
		case TS_IRQ_RAISE_EDGE:
			irq_flags = IRQF_TRIGGER_RISING;
			break;
		case TS_IRQ_FALL_EDGE:
			irq_flags = IRQF_TRIGGER_FALLING;
			break;
		default:
			TP_LOG_ERR("ts irq_config invaild\n");
			goto err_free_sysfs;
	}

	atomic_set(&g_tp_data.state, TS_WORK);//avoid 1st irq unable to handler

	error = request_irq(g_tp_data.irq_id, tp_irq_handler, irq_flags | IRQF_NO_SUSPEND, "tp", &g_tp_data);
	if (error) {
		TP_LOG_ERR("ts request_irq failed\n");
		goto err_free_sysfs;
	}

//	touch_recovery_mode = get_boot_into_recovery_flag();

	TP_LOG_INFO("touch_recovery_mode is %d\n", touch_recovery_mode);

	if (!touch_recovery_mode) {
		error = tp_try_update_firmware();
		if (error) {
			TP_LOG_ERR("return fail : %d\n", error);
			goto err_firmware_update;
		}
	}

	tp_send_roi_cmd(TS_ACTION_READ, NO_SYNC_TIMEOUT);		/*roi function set as default by TP firmware*/

	INIT_WORK(&(g_tp_data.watchdog_work), tp_watchdog_work);
	setup_timer(&(g_tp_data.watchdog_timer),tp_watchdog_timer,(unsigned long)(&g_tp_data));
	tp_start_wd_timer(&g_tp_data);

#ifdef CONFIG_ZEUSIS_HW_DEV_DCT
	/* detect current device successful, set the flag as present */
	//set_hw_dev_flag(DEV_PERIPHIAL_TOUCH_PANEL);
#endif
	error = sysfs_create_link(NULL,&g_tp_data.ts_dev->dev.kobj,"touchpad");
	if (error) {
		TP_LOG_ERR("%s: Fail create link error = %d\n", __func__, error);
		goto err_firmware_update;
	}
	TP_LOG_INFO("ts_init called out\n");
	goto out;

err_firmware_update:
	free_irq(g_tp_data.irq_id, &g_tp_data);
err_free_sysfs:
	sysfs_remove_group(&g_tp_data.ts_dev->dev.kobj, &ts_attr_group);
err_unregister_suspend:
#if defined(CONFIG_FB)
	if (fb_unregister_client(&g_tp_data.fb_notify))
		TP_LOG_ERR("error occurred while unregistering fb_notifier.\n");
#elif defined(CONFIG_HAS_EARLYSUSPEND)
	unregister_early_suspend(&g_tp_data.early_suspend);
#endif
#if defined(CONFIG_FB)
err_free_input_dev:
#endif
	input_unregister_device(input_dev);
err_free_dev:
	input_free_device(input_dev);
err_del_platform_dev:
	platform_device_del(g_tp_data.ts_dev);
err_put_platform_dev:
	platform_device_put(g_tp_data.ts_dev);
err_out:
	atomic_set(&g_tp_data.state, TS_UNINIT);
	wake_lock_destroy(&g_tp_data.ts_wake_lock);
out:
	TP_LOG_INFO("ts_init, g_tp_data.state : %d\n", atomic_read(&g_tp_data.state));
	return error;
}

static int tp_creat_i2c_client(void)
{
	struct i2c_adapter *adapter = NULL;
	struct i2c_client *client = NULL;
	struct i2c_board_info board_info;

	memset(&board_info, 0, sizeof(struct i2c_board_info));
	strncpy(board_info.type, TP_DEV_NAME, I2C_NAME_SIZE);
	board_info.addr = I2C_DEFAULT_ADDR;
	board_info.flags = true;
	adapter = i2c_get_adapter(g_tp_data.bops->bus_id);
	if (!adapter) {
		TP_LOG_ERR("i2c_get_adapter failed\n");
		return -EIO;
	}

	client = i2c_new_device(adapter, &board_info);
	if (!client) {
		TP_LOG_ERR("i2c_new_device failed\n");
		return -EIO;
	}
	g_tp_data.client = client;
	i2c_set_clientdata(client, &g_tp_data);

#if defined (CONFIG_ZEUSIS_DSM)
	if (!tp_dclient) {
		tp_dclient = dsm_register_client(&dsm_tp);
	}
#endif
	return NO_ERR;
}

static int tp_destory_i2c_client(void)
{
	TP_LOG_ERR("destory i2c device\n");
	i2c_unregister_device(g_tp_data.client);
	return NO_ERR;
}

static int tp_create_client(void)
{
	int error = -EINVAL;

	switch(g_tp_data.bops->btype) {
		case TS_BUS_I2C:
			TP_LOG_DEBUG("create ts's i2c device\n");
			error = tp_creat_i2c_client();
			break;
		case TS_BUS_SPI:
			TP_LOG_DEBUG("create ts's spi device\n");
			break;
		default:
			TP_LOG_ERR("unknown ts's device\n");
			break;
	}

	return error;
}

static int tp_destory_client(void)
{
	TP_LOG_ERR("destory touchpad device\n");

	switch(g_tp_data.bops->btype) {
		case TS_BUS_I2C:
			TP_LOG_DEBUG("destory ts's i2c device\n");
			tp_destory_i2c_client();
			break;
		case TS_BUS_SPI:
			TP_LOG_DEBUG("destory ts's spi device\n");
			break;
		default:
			TP_LOG_ERR("unknown ts's device\n");
			break;
	}
	return NO_ERR;
}

static int tp_thread(void *p)
{
	static const struct sched_param param = {
		.sched_priority = MAX_USER_RT_PRIO/2,
	};
	smp_wmb();
	if (tp_init()) {
		TP_LOG_ERR("ts_init  failed\n");
		goto out;
	}
	memset(&ping_cmd_buff, 0, sizeof(struct ts_cmd_node));
	memset(&pang_cmd_buff, 0, sizeof(struct ts_cmd_node));
	smp_mb();
	sched_setscheduler(current, SCHED_FIFO, &param);

	while(tp_task_continue())
	{
		while(!pad_get_one_cmd(&ping_cmd_buff))//get one command
		{
			tp_proc_command(&ping_cmd_buff);
			memset(&ping_cmd_buff, 0, sizeof(struct ts_cmd_node));
			memset(&pang_cmd_buff, 0, sizeof(struct ts_cmd_node));
		}
	}

	TP_LOG_ERR("ts thread stop\n");
	atomic_set(&g_tp_data.state, TS_UNINIT);
	disable_irq(g_tp_data.irq_id);
	tp_ic_shutdown();
	free_irq(g_tp_data.irq_id, &g_tp_data);
	sysfs_remove_group(&g_tp_data.ts_dev->dev.kobj, &ts_attr_group);
#if defined(CONFIG_FB)
	if (fb_unregister_client(&g_tp_data.fb_notify))
		TP_LOG_ERR("error occurred while unregistering fb_notifier.\n");
#elif defined(CONFIG_HAS_EARLYSUSPEND)
	unregister_early_suspend(&g_tp_data.early_suspend);
#endif
	input_unregister_device(g_tp_data.input_dev);
	input_free_device(g_tp_data.input_dev);
	platform_device_unregister(g_tp_data.ts_dev);
out:
	tp_destory_client();
	memset(&g_tp_data, 0, sizeof(struct ts_data));
	TP_LOG_ERR("ts_thread exited\n");
	return NO_ERR;
}

static int __init tp_module_init(void)
{
	int error = NO_ERR;
	memset(&g_tp_data, 0, sizeof(struct ts_data));

	g_tp_data.dev_id = 23;
	error = get_tp_bus_info();
	if (error) {
		TP_LOG_ERR("get bus info failed :%d\n", error);
		memset(&g_tp_data, 0, sizeof(struct ts_data));
		goto out;
	}

	error = tp_create_client();
	if (error) {
		TP_LOG_ERR("create device failed :%d\n", error);
		goto out;
	}

	g_tp_data.ts_task = kthread_create(tp_thread, &g_tp_data, "tp_thread:%d", g_tp_data.dev_id);
	if (IS_ERR(g_tp_data.ts_task)) {
		TP_LOG_ERR("create ts_thread failed\n");
		tp_destory_client();
		memset(&g_tp_data, 0, sizeof(struct ts_data));
		error = -EINVAL;
		goto out;
	}
	/* Attention about smp_mb/rmb/wmb
	     Add these driver to avoid  data consistency problem
	     ts_thread/ts_probe/irq_handler/pad_put_one_cmd/pad_get_one_cmd
	     may run in different cpus and L1/L2 cache data consistency need
	     to conside. We use barrier to make sure data consistently
	*/
	smp_mb();
	wake_up_process(g_tp_data.ts_task);

out:
	return error;
}

static void __exit tp_module_exit(void)
{
	TP_LOG_ERR("tp_module_exit called here\n");
	if (g_tp_data.ts_task)
		kthread_stop(g_tp_data.ts_task);
	return;
}

late_initcall(tp_module_init);
module_exit(tp_module_exit);

MODULE_AUTHOR("Zeusis Device Company");
MODULE_DESCRIPTION("Zeusis TouchScreen Driver");
MODULE_LICENSE("GPL v2");
