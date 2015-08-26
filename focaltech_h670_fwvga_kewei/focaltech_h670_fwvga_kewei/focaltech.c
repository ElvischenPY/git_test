/*
 * drivers/input/touchscreen/ft5x0x_ts.c
 *
 * FocalTech ft5x0x TouchScreen driver.
 *
 * Copyright (c) 2010  Focal tech Ltd.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * VERSION      	DATE			AUTHOR
 *    1.0		  2010-01-05			WenFS
 *
 * note: only support mulititouch	Wenfs 2010-10-01
 */

#include <linux/firmware.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <asm/uaccess.h>
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>
//#include <linux/i2c/ft53x6_ts.h>
#include <mach/regulator.h>
#include <linux/input/mt.h>
#include <linux/of_device.h>
#include <linux/of_address.h>
#include <linux/of_gpio.h>

#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/completion.h>
#include <linux/err.h>
#include <mach/adc.h>

#ifdef CONFIG_I2C_SPRD
#include <mach/i2c-sprd.h>
#endif
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif


#define FT53X6_pr_info

#ifdef FT53X6_pr_info
#define ENTER printk(KERN_INFO "[FT53X6_pr_info] func: %s  line: %04d\n", __func__, __LINE__);
#define PRINT_pr_info(x...)  printk(KERN_INFO "[FT53X6_pr_info] " x)
#define PRINT_INFO(x...)  printk(KERN_INFO "[FT53X6_INFO] " x)
#define PRINT_WARN(x...)  printk(KERN_INFO "[FT53X6_WARN] " x)
#define PRINT_ERR(format,x...)  printk(KERN_ERR "[FT53X6_ERR] func: %s  line: %04d  info: " format, __func__, __LINE__, ## x)
#else
#define ENTER
#define PRINT_pr_info(x...)
#define PRINT_INFO(x...)  printk(KERN_INFO "[FT53X6_INFO] " x)
#define PRINT_WARN(x...)  printk(KERN_INFO "[FT53X6_WARN] " x)
#define PRINT_ERR(format,x...)  printk(KERN_ERR "[FT53X6_ERR] func: %s  line: %04d  info: " format, __func__, __LINE__, ## x)
#endif

#define APK_DEBUG
//#define SPRD_AUTO_UPGRADE
//#define FT6x36_DOWNLOAD
#define SYSFS_DEBUG
#define FTS_CTL_IIC
//#define TPD_PROXIMITY
#define FTS_GESTRUE

#include <linux/i2c/focaltech.h>
#include <linux/i2c/focaltech_ex_fun.h>
#include <linux/i2c/focaltech_ctl.h>

#ifdef TPD_PROXIMITY
#include <linux/miscdevice.h>
#endif

#define	USE_WAIT_QUEUE	1
#define	USE_THREADED_IRQ	0
#define	USE_WORK_QUEUE	0

#define	TOUCH_VIRTUAL_KEYS
#define	MULTI_PROTOCOL_TYPE_B	0
#define	TS_MAX_FINGER		5

#define	FTS_PACKET_LENGTH	128



#ifdef FT6x36_DOWNLOAD

#include "ft6x36_download_lib.h"
static struct i2c_client *g_i2c_client = NULL;
static unsigned char CTPM_MAIN_FW1[]=
{
    #include "FT6336_C20-1_C1243_FWVGA_0x1a_all.i"
};
static unsigned char CTPM_MAIN_FW2[]=
{
	#include "FT6336_C20-1_GS00390F045_FWVGA_0x14_all.i"
};
static int TP_VOL;

#endif



#if USE_WAIT_QUEUE
static struct task_struct *thread = NULL;
static DECLARE_WAIT_QUEUE_HEAD(waiter);
static int tpd_flag = 0;
#endif

#if 0
static unsigned char FT5316_FW[]=
{
#include "ft5316_720p.i"
};

static unsigned char FT5306_FW[]=
{
#include "ft5306_qHD.i"
};

static unsigned char *CTPM_FW = FT5306_FW;
#endif
static int fw_size;

static struct ft5x0x_ts_data *g_ft5x0x_ts;
static struct i2c_client *this_client;

struct ft5x0x_ts_data *ft5x0x_ts;
	
static unsigned char suspend_flag = 0;


 struct Upgrade_Info fts_updateinfo[] =
{
    {0x55,"FT5x06",TPD_MAX_POINTS_5,AUTO_CLB_NEED,50, 30, 0x79, 0x03, 1, 2000},
    {0x08,"FT5606",TPD_MAX_POINTS_5,AUTO_CLB_NEED,50, 30, 0x79, 0x06, 100, 2000},
	{0x0a,"FT5x16",TPD_MAX_POINTS_5,AUTO_CLB_NEED,50, 30, 0x79, 0x07, 1, 1500},
	{0x05,"FT6208",TPD_MAX_POINTS_2,AUTO_CLB_NONEED,60, 30, 0x79, 0x05, 10, 2000},
	{0x06,"FT6x06",TPD_MAX_POINTS_2,AUTO_CLB_NONEED,100, 30, 0x79, 0x08, 10, 2000},
	{0x55,"FT5x06i",TPD_MAX_POINTS_5,AUTO_CLB_NEED,50, 30, 0x79, 0x03, 1, 2000},
	{0x14,"FT5336",TPD_MAX_POINTS_5,AUTO_CLB_NEED,30, 30, 0x79, 0x11, 10, 2000},
	{0x13,"FT3316",TPD_MAX_POINTS_5,AUTO_CLB_NEED,30, 30, 0x79, 0x11, 10, 2000},
	{0x12,"FT5436i",TPD_MAX_POINTS_5,AUTO_CLB_NEED,30, 30, 0x79, 0x11, 10, 2000},
	{0x11,"FT5336i",TPD_MAX_POINTS_5,AUTO_CLB_NEED,30, 30, 0x79, 0x11, 10, 2000},
	{0x36,"FT6x36",TPD_MAX_POINTS_2,AUTO_CLB_NONEED,100, 30, 0x79, 0x18, 10, 2000},
};
				
struct Upgrade_Info fts_updateinfo_curr;

#ifdef TPD_PROXIMITY
#ifdef FT53X6_DBG
#define APS_ERR(fmt,arg...)           	printk("<<proximity>> "fmt"\n",##arg)
#define TPD_PROXIMITY_DEBUG(fmt,arg...) printk("<<proximity>> "fmt"\n",##arg)
#define TPD_PROXIMITY_DMESG(fmt,arg...) printk("<<proximity>> "fmt"\n",##arg)
#else
#define APS_ERR(fmt,arg...)
#define TPD_PROXIMITY_DEBUG(fmt,arg...)
#define TPD_PROXIMITY_DMESG(fmt,arg...)
#endif

#define FOCALTECH_PLS_DEVICE_NAME		"ltr_558als"
#define FOCALTECH_PLS_INPUT_NAME			"alps_pxy"
#define FOCALTECH_IOCTL_MAGIC			0x1C
#define FOCALTECH_IOCTL_GET_PFLAG		_IOR(FOCALTECH_IOCTL_MAGIC, 1, int)
#define FOCALTECH_IOCTL_GET_LFLAG		_IOR(FOCALTECH_IOCTL_MAGIC, 2, int)
#define FOCALTECH_IOCTL_SET_PFLAG		_IOW(FOCALTECH_IOCTL_MAGIC, 3, int)
#define FOCALTECH_IOCTL_SET_LFLAG		_IOW(FOCALTECH_IOCTL_MAGIC, 4, int)

static u8 tpd_proximity_flag 			= 0;
static u8 tpd_proximity_suspend 		= 0;
static u8 tpd_proximity_detect 		= 1;//0-->close ; 1--> far away
static struct input_dev *focaltech_pls_input_dev;
static int focaltech_pls_opened=0;

static int tpd_enable_ps(int enable);
static int tpd_read_ps(void);
static int tpd_get_ps_value(void);
#endif

#ifdef FTS_GESTRUE
#define GESTURE_LEFT		0x20
#define GESTURE_RIGHT		0x21
#define GESTURE_UP		    0x22
#define GESTURE_DOWN		0x23
#define GESTURE_DOUBLECLICK	0x24
#define GESTURE_O		    0x30
#define GESTURE_W		    0x31
#define GESTURE_M		    0x32
#define GESTURE_E		    0x33
#define GESTURE_C		    0x34
#define GESTURE_S		    0x46
#define GESTURE_V		    0x54
#define GESTURE_Z		    0x41
#define GESTURE_A		    0x36

#include "ft_gesture_lib.h"
static bool TP_gesture_Switch;
#define GESTURE_SWITCH_FILE 		"/data/data/com.android.settings/shared_prefs/gesture_open.xml"  //总开关文件,获取第一个value的值,为1开,为0关

#define FTS_GESTRUE_POINTS 255
#define FTS_GESTRUE_POINTS_ONETIME  62
#define FTS_GESTRUE_POINTS_HEADER 8
#define FTS_GESTURE_OUTPUT_ADRESS 0xD3
#define FTS_GESTURE_OUTPUT_UNIT_LENGTH 4

//suspend_state_t get_suspend_state(void);

unsigned short coordinate_x[150] = {0};
unsigned short coordinate_y[150] = {0};
#endif


#if 0//dennis
/* Attribute */
static ssize_t ft5x0x_show_suspend(struct device* cd,struct device_attribute *attr, char* buf);
static ssize_t ft5x0x_store_suspend(struct device* cd, struct device_attribute *attr,const char* buf, size_t len);
static ssize_t ft5x0x_show_version(struct device* cd,struct device_attribute *attr, char* buf);
static ssize_t ft5x0x_update(struct device* cd, struct device_attribute *attr, const char* buf, size_t len);
#endif
static unsigned char ft5x0x_read_fw_ver(void);

#ifdef CONFIG_HAS_EARLYSUSPEND
static void ft5x0x_ts_suspend(struct early_suspend *handler);
static void ft5x0x_ts_resume(struct early_suspend *handler);
#endif
//static int fts_ctpm_fw_update(void);
static int fts_ctpm_fw_upgrade_with_i_file(void);

struct ts_event {
	u16	x1;
	u16	y1;
	u16	x2;
	u16	y2;
	u16	x3;
	u16	y3;
	u16	x4;
	u16	y4;
	u16	x5;
	u16	y5;
	u16	pressure;
    u8  touch_point;
};

struct ft5x0x_ts_data {
	struct input_dev	*input_dev;
	struct i2c_client	*client;
	struct ts_event	event;
#if USE_WORK_QUEUE
	struct work_struct	pen_event_work;
	struct workqueue_struct	*ts_workqueue;
#endif
#ifdef CONFIG_HAS_EARLYSUSPEND
	struct work_struct       resume_work;
	struct workqueue_struct *ts_resume_workqueue;
	struct early_suspend	early_suspend;
#endif
	struct ft5x0x_ts_platform_data	*platform_data;
};

#if 0//dennis
static DEVICE_ATTR(suspend, S_IRUGO | S_IWUSR, ft5x0x_show_suspend, ft5x0x_store_suspend);
static DEVICE_ATTR(update, S_IRUGO | S_IWUSR, ft5x0x_show_version, ft5x0x_update);

static ssize_t ft5x0x_show_suspend(struct device* cd,
				     struct device_attribute *attr, char* buf)
{
	ssize_t ret = 0;

	if(suspend_flag==1)
		sprintf(buf, "FT5x0x Suspend\n");
	else
		sprintf(buf, "FT5x0x Resume\n");

	ret = strlen(buf) + 1;

	return ret;
}

static ssize_t ft5x0x_store_suspend(struct device* cd, struct device_attribute *attr,
		       const char* buf, size_t len)
{
	unsigned long on_off = simple_strtoul(buf, NULL, 10);
	suspend_flag = on_off;

	if(on_off==1)
	{
		pr_info("FT5x0x Entry Suspend\n");
	#ifdef CONFIG_HAS_EARLYSUSPEND
		ft5x0x_ts_suspend(NULL);
	#endif
	}
	else
	{
		pr_info("FT5x0x Entry Resume\n");
	#ifdef CONFIG_HAS_EARLYSUSPEND
		ft5x0x_ts_resume(NULL);
	#endif
	}

	return len;
}

static ssize_t ft5x0x_show_version(struct device* cd,
				     struct device_attribute *attr, char* buf)
{
	ssize_t ret = 0;
	unsigned char uc_reg_value; 

	uc_reg_value = ft5x0x_read_fw_ver();

	sprintf(buf, "ft5x0x firmware id is V%x\n", uc_reg_value);

	ret = strlen(buf) + 1;

	return ret;
}

static ssize_t ft5x0x_update(struct device* cd, struct device_attribute *attr,
		       const char* buf, size_t len)
{
	unsigned long on_off = simple_strtoul(buf, NULL, 10);
	unsigned char uc_reg_value;

	uc_reg_value = ft5x0x_read_fw_ver();

	if(on_off==1)
	{
		pr_info("ft5x0x update, current firmware id is V%x\n", uc_reg_value);
		//fts_ctpm_fw_update();
		fts_ctpm_fw_upgrade_with_i_file();
	}

	return len;
}

static int ft5x0x_create_sysfs(struct i2c_client *client)
{
	int err;
	struct device *dev = &(client->dev);

	err = device_create_file(dev, &dev_attr_suspend);
	err = device_create_file(dev, &dev_attr_update);

	return err;
}
#endif

static int ft5x0x_i2c_rxdata(char *rxdata, int length)
{
	int ret = 0;

	struct i2c_msg msgs[] = {
		{
			.addr	= this_client->addr,
			.flags	= 0,
			.len	= 1,
			.buf	= rxdata,
		},
		{
			.addr	= this_client->addr,
			.flags	= I2C_M_RD,
			.len	= length,
			.buf	= rxdata,
		},
	};

	if (i2c_transfer(this_client->adapter, msgs, 2) != 2) {
		ret = -EIO;
		pr_err("msg %s i2c read error: %d\n", __func__, ret);
	}

	return ret;
}

static int ft5x0x_i2c_txdata(char *txdata, int length)
{
	int ret = 0;

	struct i2c_msg msg[] = {
		{
			.addr	= this_client->addr,
			.flags	= 0,
			.len	= length,
			.buf	= txdata,
		},
	};

	if (i2c_transfer(this_client->adapter, msg, 1) != 1) {
		ret = -EIO;
		pr_err("%s i2c write error: %d\n", __func__, ret);
	}

	return ret;
}

/***********************************************************************************************
Name	:	 ft5x0x_write_reg

Input	:	addr -- address
                     para -- parameter

Output	:

function	:	write register of ft5x0x

***********************************************************************************************/
static int ft5x0x_write_reg(u8 addr, u8 para)
{
	u8 buf[3];
	int ret = -1;

	buf[0] = addr;
	buf[1] = para;
	ret = ft5x0x_i2c_txdata(buf, 2);
	if (ret < 0) {
		pr_err("write reg failed! %#x ret: %d", buf[0], ret);
		return -1;
	}

	return 0;
}

/***********************************************************************************************
Name	:	ft5x0x_read_reg

Input	:	addr
                     pdata

Output	:

function	:	read register of ft5x0x

***********************************************************************************************/
static int ft5x0x_read_reg(u8 addr, u8 *pdata)
{
	int ret = 0;
	u8 buf[2] = {addr, 0};

	struct i2c_msg msgs[] = {
		{
			.addr	= this_client->addr,
			.flags	= 0,
			.len	= 1,
			.buf	= buf,
		},
		{
			.addr	= this_client->addr,
			.flags	= I2C_M_RD,
			.len	= 1,
			.buf	= buf+1,
		},
	};

	if (i2c_transfer(this_client->adapter, msgs, 2) != 2) {
		ret = -EIO;
		pr_err("msg %s i2c read error: %d\n", __func__, ret);
	}

	*pdata = buf[1];
	return ret;
}


#ifdef TOUCH_VIRTUAL_KEYS

static ssize_t virtual_keys_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	struct ft5x0x_ts_data *data = i2c_get_clientdata(this_client);
	struct ft5x0x_ts_platform_data *pdata = data->platform_data;
	return sprintf(buf,"%s:%s:%d:%d:%d:%d:%s:%s:%d:%d:%d:%d:%s:%s:%d:%d:%d:%d\n"
		,__stringify(EV_KEY), __stringify(KEY_MENU),pdata ->virtualkeys[0],pdata ->virtualkeys[1],pdata ->virtualkeys[2],pdata ->virtualkeys[3]
		,__stringify(EV_KEY), __stringify(KEY_HOMEPAGE),pdata ->virtualkeys[4],pdata ->virtualkeys[5],pdata ->virtualkeys[6],pdata ->virtualkeys[7]
		,__stringify(EV_KEY), __stringify(KEY_BACK),pdata ->virtualkeys[8],pdata ->virtualkeys[9],pdata ->virtualkeys[10],pdata ->virtualkeys[11]);
}

static struct kobj_attribute virtual_keys_attr = {
    .attr = {
        .name = "virtualkeys.focaltech_ts",
        .mode = S_IRUGO,
    },
    .show = &virtual_keys_show,
};

static struct attribute *properties_attrs[] = {
    &virtual_keys_attr.attr,
    NULL
};

static struct attribute_group properties_attr_group = {
    .attrs = properties_attrs,
};

static void ft5x0x_ts_virtual_keys_init(void)
{
    int ret = 0;
    struct kobject *properties_kobj;

    pr_info("[FST] %s\n",__func__);

    properties_kobj = kobject_create_and_add("board_properties", NULL);
    if (properties_kobj)
        ret = sysfs_create_group(properties_kobj,
                     &properties_attr_group);
    if (!properties_kobj || ret)
        pr_err("failed to create board_properties\n");
}

#endif

/***********************************************************************************************
Name	:	 ft5x0x_read_fw_ver

Input	:	 void

Output	:	 firmware version

function	:	 read TP firmware version

***********************************************************************************************/
static unsigned char ft5x0x_read_fw_ver(void)
{
	unsigned char ver;
	ft5x0x_read_reg(FT5X0X_REG_FIRMID, &ver);
	return(ver);
}



static void ft5x0x_clear_report_data(struct ft5x0x_ts_data *ft5x0x_ts)
{
	int i;

	for(i = 0; i < TS_MAX_FINGER; i++) {
	#if MULTI_PROTOCOL_TYPE_B
		input_mt_slot(ft5x0x_ts->input_dev, i);
		input_mt_report_slot_state(ft5x0x_ts->input_dev, MT_TOOL_FINGER, false);
	#endif
		input_report_key(ft5x0x_ts->input_dev, BTN_TOUCH, 0);
	#if !MULTI_PROTOCOL_TYPE_B
		input_mt_sync(ft5x0x_ts->input_dev);
	#endif
	}
	input_sync(ft5x0x_ts->input_dev);
}

#ifdef FTS_GESTRUE
static void check_gesture(int gesture_id)
{
	
    printk("kaka gesture_id==0x%x\n ",gesture_id);
    
	switch(gesture_id)
	{
		case GESTURE_LEFT:		     
			input_report_key(ft5x0x_ts->input_dev, KEY_LEFT, 1);
			input_sync(ft5x0x_ts->input_dev);
			input_report_key(ft5x0x_ts->input_dev, KEY_LEFT, 0);
			input_sync(ft5x0x_ts->input_dev);
			break;
		case GESTURE_RIGHT:	
			input_report_key(ft5x0x_ts->input_dev, KEY_RIGHT, 1);
			input_sync(ft5x0x_ts->input_dev);
			input_report_key(ft5x0x_ts->input_dev, KEY_RIGHT, 0);
			input_sync(ft5x0x_ts->input_dev);
			break;
		case GESTURE_UP:	
			input_report_key(ft5x0x_ts->input_dev, KEY_UP, 1);
			input_sync(ft5x0x_ts->input_dev);
			input_report_key(ft5x0x_ts->input_dev, KEY_UP, 0);
			input_sync(ft5x0x_ts->input_dev);
			break;
		case GESTURE_DOWN:	
			input_report_key(ft5x0x_ts->input_dev, KEY_DOWN, 1);
			input_sync(ft5x0x_ts->input_dev);
			input_report_key(ft5x0x_ts->input_dev, KEY_DOWN, 0);
			input_sync(ft5x0x_ts->input_dev);		    
			break;
		case GESTURE_DOUBLECLICK:	
			input_report_key(ft5x0x_ts->input_dev, KEY_U, 1);
			input_sync(ft5x0x_ts->input_dev);
			input_report_key(ft5x0x_ts->input_dev, KEY_U, 0);
			input_sync(ft5x0x_ts->input_dev);		    
			break;
		case GESTURE_O:	
			input_report_key(ft5x0x_ts->input_dev, KEY_O, 1);
			input_sync(ft5x0x_ts->input_dev);
			input_report_key(ft5x0x_ts->input_dev, KEY_O, 0);
			input_sync(ft5x0x_ts->input_dev);
			break;
		case GESTURE_W:			
			input_report_key(ft5x0x_ts->input_dev, KEY_W, 1);
			input_sync(ft5x0x_ts->input_dev);
			input_report_key(ft5x0x_ts->input_dev, KEY_W, 0);
			input_sync(ft5x0x_ts->input_dev);
			break;
		case GESTURE_M:		
			input_report_key(ft5x0x_ts->input_dev, KEY_M, 1);
			input_sync(ft5x0x_ts->input_dev);
			input_report_key(ft5x0x_ts->input_dev, KEY_M, 0);
			input_sync(ft5x0x_ts->input_dev);
			break;
		case GESTURE_E:		
			input_report_key(ft5x0x_ts->input_dev, KEY_E, 1);
			input_sync(ft5x0x_ts->input_dev);
			input_report_key(ft5x0x_ts->input_dev, KEY_E, 0);
			input_sync(ft5x0x_ts->input_dev);			    
			break;
		case GESTURE_C:		
			input_report_key(ft5x0x_ts->input_dev, KEY_C, 1);
			input_sync(ft5x0x_ts->input_dev);
			input_report_key(ft5x0x_ts->input_dev, KEY_C, 0);
			input_sync(ft5x0x_ts->input_dev);
			break;
		case GESTURE_S:		
			input_report_key(ft5x0x_ts->input_dev, KEY_S, 1);
			input_sync(ft5x0x_ts->input_dev);
			input_report_key(ft5x0x_ts->input_dev, KEY_S, 0);
			input_sync(ft5x0x_ts->input_dev);
			break;
		case GESTURE_V:
			input_report_key(ft5x0x_ts->input_dev, KEY_V, 1);
			input_sync(ft5x0x_ts->input_dev);
			input_report_key(ft5x0x_ts->input_dev, KEY_V, 0);
			input_sync(ft5x0x_ts->input_dev);
			break;
		case GESTURE_Z:	
			input_report_key(ft5x0x_ts->input_dev, KEY_Z, 1);
			input_sync(ft5x0x_ts->input_dev);
			input_report_key(ft5x0x_ts->input_dev, KEY_Z, 0);
			input_sync(ft5x0x_ts->input_dev);
			break;
		default:
		
			break;
	}

}


static int ft5x0x_read_Touchdata(void)
{
    unsigned char buf[FTS_GESTRUE_POINTS * 4] = { 0 };
    int ret = -1;
    int i = 0;
	int j = 0;
    buf[0] = 0xd3;
    int gestrue_id = 0;
    short pointnum = 0;
    ret = fts_i2c_Read(this_client, buf, 1, buf, FTS_GESTRUE_POINTS_HEADER);
    if (ret < 0)
    {
        printk( "%s read touchdata failed.\n", __func__);
        return ret;
    }
    /* FW ?±?ó??3?ê?ê? */
    if (0x24 == buf[0] || 0x20 == buf[0] || 0x21 == buf[0] || 0x22 == buf[0] || 0x23 == buf[0])
    {
        gestrue_id =  buf[0];
        check_gesture(gestrue_id);
        return -1;
    }

    pointnum = (short)(buf[1]) & 0xff;
    buf[0] = 0xd3;
	
	//printk( "yedongyue1 ----%s  buf[0] = %d,pointnum = %d\n", __func__,buf[0],pointnum);
    if((pointnum * 4 + 2+6)<255)
    {
		ret = fts_i2c_Read(this_client, buf, 1, buf, (pointnum * 4 + 2 + 6));
    }
    else
    {
		ret = fts_i2c_Read(this_client, buf, 1, buf, 255);
		ret = fts_i2c_Read(this_client, buf, 0, buf+255, (pointnum * 4 + 2 +6)-255);
    }
    if (ret < 0)
    {
        printk( "%s read touchdata failed.\n", __func__);
        return ret;
    }
//	printk( "yedongyue2 ----%s  buf[0] = %d,pointnum = %d\n", __func__,buf[0],pointnum);
	gestrue_id = fetch_object_sample(buf, pointnum);
//	printk( "yedongyue3  ----%s   buf[0] = %d,pointnum = %d,gestrue_id = %d\n", __func__,buf[0],pointnum,gestrue_id);
    for(i = 0;i < pointnum;i++)
    {
        coordinate_x[i] =  (((s16) buf[0 + (4 * i)]) & 0x0F) <<
            8 | (((s16) buf[1 + (4 * i)])& 0xFF);
        coordinate_y[i] = (((s16) buf[2 + (4 * i)]) & 0x0F) <<
            8 | (((s16) buf[3 + (4 * i)]) & 0xFF);
	//	printk( "yedongyue  -------   x=%d ,y=%d----------\n",buf[0],coordinate_x[i],coordinate_y[i]);
    }
	
	check_gesture(gestrue_id);
    return -1;
}
#endif

static int ft5x0x_update_data(void)
{
	struct ft5x0x_ts_data *data = i2c_get_clientdata(this_client);
	struct ts_event *event = &data->event;
	u8 buf[33] = {0};
	int ret = -1;
	int i;
	u16 x , y;
	u8 ft_pressure , ft_size;

#ifdef TPD_PROXIMITY
	int err;
	u8 proximity_status;
	u8 state;
#endif


#ifdef FTS_GESTRUE
	u8 sta;
	i2c_smbus_read_i2c_block_data(this_client, 0xd0, 1, &sta);
	// if((get_suspend_state() == PM_SUSPEND_MEM) && (state ==1))
	 if(sta ==1)
	{
		ft5x0x_read_Touchdata();
		return 0;
	}
#endif

{
//wangcq327 add when charging write 0x01 to 0x8b start
//wangcq327 add start
	struct filp * fp=NULL;
	mm_segment_t fs;
	loff_t pos = 0;
	char buf[200]={0};
	char *p=NULL;
	fp = filp_open("/sys/class/power_supply/battery/status",O_RDONLY , 0);
	if(IS_ERR(fp)){
		printk("wangcq327 --- touchpanle Open file fail error!!\n");
	}else{
		//read start
		fs = get_fs();//get old fs;
		set_fs(KERNEL_DS);
		vfs_read(fp,buf,sizeof(buf),&pos);
		//printk("wangcq327 --- read == %s\n",buf);
		//read end

		if(0 == strncmp("Charging",buf,8)){
			fts_write_reg(this_client, 0x8b, 0x01);
			//printk("wangcq327 --- Charging\n");
		}else{
    		fts_write_reg(this_client, 0x8b, 0x00);
			//printk("wangcq327 --- NO Charging\n");
		}
//		printk("wangcq327 --- *p == %c\n",*p);
	//	printk("wangcq327 --- TP_gesture_Switch == %d\n",(TP_gesture_Switch==true)?1:0);

		filp_close(fp,NULL);
		set_fs(fs);
	}
//wangcq327 add end
//wangcq327 add when charging write 0x01 to 0x8b end
}

#ifdef TPD_PROXIMITY
		if (tpd_proximity_flag == 1)
		{
			i2c_smbus_read_i2c_block_data(this_client, 0xB0, 1, &state);
			TPD_PROXIMITY_DEBUG("proxi_5206 0xB0 state value is 1131 0x%02X\n", state);

			if(!(state&0x01))
			{
				tpd_enable_ps(1);
			}

			i2c_smbus_read_i2c_block_data(this_client, 0x01, 1, &proximity_status);
			TPD_PROXIMITY_DEBUG("proxi_5206 0x01 value is 1139 0x%02X\n", proximity_status);
			
			if (proximity_status == 0xC0)
			{
				tpd_proximity_detect = 0;	
			}
			else if(proximity_status == 0xE0)
			{
				tpd_proximity_detect = 1;
			}

			TPD_PROXIMITY_DEBUG("tpd_proximity_detect 1149 = %d\n", tpd_proximity_detect);

			if ((err = tpd_read_ps()))
			{
//				TPD_PROXIMITY_DMESG("proxi_5206 read ps data 1156: %d\n", err);	
			}

			input_report_abs(focaltech_pls_input_dev, ABS_DISTANCE, tpd_get_ps_value());
			input_sync(focaltech_pls_input_dev);
		}  
#endif

	
	ret = ft5x0x_i2c_rxdata(buf, 31);

	if (ret < 0) {
		pr_err("%s read_data i2c_rxdata failed: %d\n", __func__, ret);
		return ret;
	}

	memset(event, 0, sizeof(struct ts_event));
	event->touch_point = buf[2] & 0x07;

	for(i = 0; i < TS_MAX_FINGER; i++) {
		if((buf[6*i+3] & 0xc0) == 0xc0)
			continue;
		x = (s16)(buf[6*i+3] & 0x0F)<<8 | (s16)buf[6*i+4];	
		y = (s16)(buf[6*i+5] & 0x0F)<<8 | (s16)buf[6*i+6];
		if((buf[6*i+3] & 0x40) == 0x0) {
		#if MULTI_PROTOCOL_TYPE_B
			input_mt_slot(data->input_dev, buf[6*i+5]>>4);
			input_mt_report_slot_state(data->input_dev, MT_TOOL_FINGER, true);
		#endif
			input_report_abs(ft5x0x_ts->input_dev, ABS_MT_POSITION_X, x);
			input_report_abs(ft5x0x_ts->input_dev, ABS_MT_POSITION_Y, y);
			input_report_key(ft5x0x_ts->input_dev, BTN_TOUCH, 1);
		#if !MULTI_PROTOCOL_TYPE_B
			input_mt_sync(ft5x0x_ts->input_dev);
		//	printk("//chinapalms 1111test report===x%d = %d,y%d = %d ====\n",i, x, i, y);
		#endif
			pr_debug("===x%d = %d,y%d = %d ====",i, x, i, y);
		//	printk("//chinapalms ===x%d = %d,y%d = %d ====\n",i, x, i, y);
		}
		else {
		#if MULTI_PROTOCOL_TYPE_B
			input_mt_slot(ft5x0x_ts->input_dev, buf[6*i+5]>>4);
			input_mt_report_slot_state(ft5x0x_ts->input_dev, MT_TOOL_FINGER, false);
		#endif
		}
	}
	if(0 == event->touch_point) {
		input_report_key(ft5x0x_ts->input_dev, BTN_TOUCH, 0);
		#if 1//!MULTI_PROTOCOL_TYPE_B
			input_mt_sync(ft5x0x_ts->input_dev);
		#endif

	}
	input_sync(ft5x0x_ts->input_dev);

	return 0;
}

#if USE_WAIT_QUEUE
static int touch_event_handler(void *unused)
{
	struct sched_param param = { .sched_priority = 5 };
	sched_setscheduler(current, SCHED_RR, &param);

	do {
		set_current_state(TASK_INTERRUPTIBLE);
		wait_event_interruptible(waiter, (0 != tpd_flag));
		tpd_flag = 0;
		set_current_state(TASK_RUNNING);
		ft5x0x_update_data();
		
	} while (!kthread_should_stop());

	return 0;
}
#endif

#if USE_WORK_QUEUE
static void ft5x0x_ts_pen_irq_work(struct work_struct *work)
{
	ft5x0x_update_data();
	enable_irq(this_client->irq);
}
#endif

static irqreturn_t ft5x0x_ts_interrupt(int irq, void *dev_id)
{
#if USE_WAIT_QUEUE
	tpd_flag = 1;
	wake_up_interruptible(&waiter);
	return IRQ_HANDLED;
#endif

#if USE_WORK_QUEUE
	struct ft5x0x_ts_data *ft5x0x_ts = (struct ft5x0x_ts_data *)dev_id;

	if (!work_pending(&ft5x0x_ts->pen_event_work)) {
		queue_work(ft5x0x_ts->ts_workqueue, &ft5x0x_ts->pen_event_work);
	}
	return IRQ_HANDLED;
#endif

#if USE_THREADED_IRQ
	ft5x0x_update_data();
	return IRQ_HANDLED;
#endif

}

static void ft5x0x_ts_reset(void)
{
	struct ft5x0x_ts_platform_data *pdata = g_ft5x0x_ts->platform_data;

	gpio_direction_output(pdata->reset_gpio_number, 1);
	msleep(10);
	gpio_set_value(pdata->reset_gpio_number, 1);
	msleep(10);
	gpio_set_value(pdata->reset_gpio_number, 0);
	msleep(10);
	gpio_set_value(pdata->reset_gpio_number, 1);
	//msleep(5);
}

#if 0
//for future use
struct regulator *vdd28 = NULL;

static void ft53x6_power_off(void)
{
	if(vdd28 != NULL)
		regulator_force_disable(vdd28);
	PRINT_INFO("power off\n");
}

static void ft53x6_power_on(void)
{
	int err = 0;

	if(vdd28 == NULL) {
		vdd28 = regulator_get(NULL, "vdd28");
		if (IS_ERR(vdd28)) {
			PRINT_ERR("regulator_get failed\n");
			return;
		}
		err = regulator_set_voltage(vdd28,2800000,2800000);
		if (err)
			PRINT_ERR("regulator_set_voltage failed\n");
	}
	regulator_enable(vdd28);

	PRINT_INFO("power on\n");
}
#endif

#ifdef TPD_PROXIMITY
static int tpd_read_ps(void)
{
//	tpd_proximity_detect;
	return 0;    
}

static int tpd_get_ps_value(void)
{
	return tpd_proximity_detect;
}

static int tpd_enable_ps(int enable)
{
	u8 state;
	int ret = -1;

	i2c_smbus_read_i2c_block_data(this_client, 0xB0, 1, &state);
	TPD_PROXIMITY_DEBUG("[proxi_5206]read: 999 0xb0's value is 0x%02X\n", state);
	if (enable){
	      #ifdef WAKE_LOCK_LPSENSOR
            	wake_lock(&pls_delayed_work_wake_lock);
            	#endif
		state |= 0x01;
		tpd_proximity_flag = 1;
		TPD_PROXIMITY_DEBUG("[proxi_5206]ps function is on\n");	
		
	}else{
	        #ifdef WAKE_LOCK_LPSENSOR
            	wake_unlock(&pls_delayed_work_wake_lock);
            	#endif
		state &= 0x00;	
		tpd_proximity_flag = 0;
		TPD_PROXIMITY_DEBUG("[proxi_5206]ps function is off\n");
		
	}

	ret = i2c_smbus_write_i2c_block_data(this_client, 0xB0, 1, &state);
	TPD_PROXIMITY_DEBUG("[proxi_5206]write: 0xB0's value is 0x%02X\n", state);
	return 0;
}

static int focaltech_pls_open(struct inode *inode, struct file *file)
{
	TPD_PROXIMITY_DEBUG("%s", __func__);
	if (focaltech_pls_opened)
		return -EBUSY;
	focaltech_pls_opened = 1;
	return 0;
}
static int focaltech_pls_release(struct inode *inode, struct file *file)
{
	TPD_PROXIMITY_DEBUG("%s", __func__);
	focaltech_pls_opened = 0;
	return 0;//ap3212c_pls_disable(AP3212C_PLS_BOTH);
}
static long focaltech_pls_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	int flag;
	int err = 0;

	TPD_PROXIMITY_DEBUG("%s: cmd %d", __func__, _IOC_NR(cmd));

	switch (cmd)
	{
		case FOCALTECH_IOCTL_SET_PFLAG:
			if (copy_from_user(&flag, argp, sizeof(flag)))
				return -EFAULT;
			TPD_PROXIMITY_DEBUG("FOCALTECH_IOCTL_SET_PFLAG");
			if(flag)
			{
				if((err=tpd_enable_ps(1)) != 0)
				{
					APS_ERR("enable ps fail: %d\n", err); 
					return -EFAULT;
				}
			}
			else
			{
				if((err=tpd_enable_ps(0)) != 0)
				{
					APS_ERR("disable ps fail: %d\n", err); 
					return -EFAULT;
				}
			}
			break;

		case FOCALTECH_IOCTL_SET_LFLAG:
			break;

		case FOCALTECH_IOCTL_GET_PFLAG:
			flag = tpd_proximity_flag;
			if (copy_to_user(argp, &flag, sizeof(flag)))
				return -EFAULT;
			break;

		case FOCALTECH_IOCTL_GET_LFLAG:
			flag = 0;
			if (copy_to_user(argp, &flag, sizeof(flag)))
				return -EFAULT;
			break;

		default:
			TPD_PROXIMITY_DEBUG("%s: invalid cmd %d", __func__, _IOC_NR(cmd));
			return -EINVAL;
	}

	return 0;
}

 static struct file_operations focaltech_pls_fops = {
	 .owner 			 = THIS_MODULE,
	 .open				 = focaltech_pls_open,
	 .release			 = focaltech_pls_release,
	 .unlocked_ioctl 	 = focaltech_pls_ioctl,
 };
 static struct miscdevice focaltech_pls_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = FOCALTECH_PLS_DEVICE_NAME,
	.fops = &focaltech_pls_fops,
};
#endif	//#ifdef TPD_PROXIMITY


#ifdef CONFIG_HAS_EARLYSUSPEND
static void ft5x0x_ts_suspend(struct early_suspend *handler)
{
	int ret = -1;
	pr_info("==%s==\n", __FUNCTION__);

#ifdef FTS_GESTRUE	
	//wangcq327 add start
	struct filp * fp=NULL;
	mm_segment_t fs;
	loff_t pos = 0;
	char buf[200]={0};
	char *p=NULL;
	fp = filp_open(GESTURE_SWITCH_FILE ,O_RDONLY , 0);
	if(IS_ERR(fp)){
		printk("wangcq327 --- touchpanle Open file fail !!\n");
		TP_gesture_Switch = true;
	}else{
		//read start
		fs = get_fs();//get old fs;
		set_fs(KERNEL_DS);
		vfs_read(fp,buf,sizeof(buf),&pos);
		//printk("wangcq327 --- read == %s\n",buf);
		//read end

		p = strstr(buf,"value=");//value="1"
		p += 7;

		//printk("wangcq327 --- *p == %c\n",*p);
		if(*p == '1'){
			TP_gesture_Switch = true;
		}else{
			TP_gesture_Switch = false;
		}
		//printk("wangcq327 --- TP_gesture_Switch == %d\n",(TP_gesture_Switch==true)?1:0);
		
		filp_close(fp,NULL);
		set_fs(fs);
	}
	//wangcq327 add end
#endif
#ifdef TPD_PROXIMITY
	if (tpd_proximity_flag == 1)
	{
		tpd_proximity_suspend = 0;	
		return;
	}
	else
	{
		tpd_proximity_suspend = 1;
	}
#endif
	
	#ifdef FTS_GESTRUE
	if(true == TP_gesture_Switch){//wangcq327 add
		fts_write_reg(this_client, 0xd0, 0x01);
		//fts_write_reg(this_client, 0xd1, 0x1f);
		// fts_write_reg(this_client, 0xd2, 0x1f);
		//tpd_halt = 1;
		return;
	#endif
	}

	ret = ft5x0x_write_reg(FT5X0X_REG_PMODE, PMODE_HIBERNATE);
	if(ret){
		PRINT_ERR("==ft5x0x_ts_suspend==  ft5x0x_write_reg fail\n");
	}
	disable_irq(this_client->irq);
	ft5x0x_clear_report_data(g_ft5x0x_ts);
	//gpio_set_value(g_ft5x0x_ts->platform_data->reset_gpio_number, 0);//for future use
}

static void ft5x0x_ts_resume(struct early_suspend *handler)
{
	struct ft5x0x_ts_data  *ft5x0x_ts = (struct ft5x0x_ts_data *)i2c_get_clientdata(this_client);
	
#ifdef TPD_PROXIMITY	
	if (tpd_proximity_suspend == 0)
	{
		return;
	}
	else
	{
		tpd_proximity_suspend = 0;
	}
#endif	

	queue_work(ft5x0x_ts->ts_resume_workqueue, &ft5x0x_ts->resume_work);
}

static void ft5x0x_ts_resume_work(struct work_struct *work)
{
	pr_info("==%s==\n", __FUNCTION__);
	ft5x0x_ts_reset();
	msleep(200);
	ft5x0x_write_reg(FT5X0X_REG_PERIODACTIVE, 7);
	enable_irq(this_client->irq);
}
#endif

static void ft5x0x_ts_hw_init(struct ft5x0x_ts_data *ft5x0x_ts)
{
	struct regulator *reg_vdd;
	struct i2c_client *client = ft5x0x_ts->client;
	struct ft5x0x_ts_platform_data *pdata = ft5x0x_ts->platform_data;

	pr_info("[FST] %s  chenbowei [irq=%d];[rst=%d]\n",__func__,
		pdata->irq_gpio_number,pdata->reset_gpio_number);
	gpio_request(pdata->irq_gpio_number, "ts_irq_pin");
	gpio_request(pdata->reset_gpio_number, "ts_rst_pin");
	gpio_direction_output(pdata->reset_gpio_number, 1);
	gpio_direction_input(pdata->irq_gpio_number);

	reg_vdd = regulator_get(&client->dev, pdata->vdd_name);
	if (!WARN(IS_ERR(reg_vdd), "[FST] ft5x0x_ts_hw_init regulator: failed to get %s.\n", pdata->vdd_name)) {
		regulator_set_voltage(reg_vdd, 2800000, 2800000);
		regulator_enable(reg_vdd);
	}
	msleep(100);
	ft5x0x_ts_reset();
	msleep(100);
}

void focaltech_get_upgrade_array(struct i2c_client *client)
{

	u8 chip_id;
	u32 i;

	i2c_smbus_read_i2c_block_data(client,FT_REG_CHIP_ID,1,&chip_id);
	
	printk("%s chip_id = %x\n", __func__, chip_id);
	for(i=0;i<sizeof(fts_updateinfo)/sizeof(struct Upgrade_Info);i++)
	{
		if(chip_id==fts_updateinfo[i].CHIP_ID)
		{
			memcpy(&fts_updateinfo_curr, &fts_updateinfo[i], sizeof(struct Upgrade_Info));
			break;
		}
	}
	printk("%s chip_id = %x,i = %d\n", __func__, chip_id,i);
	if(i >= sizeof(fts_updateinfo)/sizeof(struct Upgrade_Info))
	{
		memcpy(&fts_updateinfo_curr, &fts_updateinfo[0], sizeof(struct Upgrade_Info));
	}
}

#ifdef CONFIG_OF
static struct ft5x0x_ts_platform_data *ft5x0x_ts_parse_dt(struct device *dev)
{
	struct ft5x0x_ts_platform_data *pdata;
	struct device_node *np = dev->of_node;
	int ret;

	pdata = kzalloc(sizeof(*pdata), GFP_KERNEL);
	if (!pdata) {
		dev_err(dev, "Could not allocate struct ft5x0x_ts_platform_data");
		return NULL;
	}
	pdata->reset_gpio_number = of_get_gpio(np, 0);
	if(pdata->reset_gpio_number < 0){
		dev_err(dev, "fail to get reset_gpio_number\n");
		goto fail;
	}
	pdata->irq_gpio_number = of_get_gpio(np, 1);
	if(pdata->reset_gpio_number < 0){
		dev_err(dev, "fail to get reset_gpio_number\n");
		goto fail;
	}
	ret = of_property_read_string(np, "vdd_name", &pdata->vdd_name);
	if(ret){
		dev_err(dev, "fail to get vdd_name\n");
		goto fail;
	}
	ret = of_property_read_u32_array(np, "virtualkeys", &pdata->virtualkeys,12);
	if(ret){
		dev_err(dev, "fail to get virtualkeys\n");
		goto fail;
	}
	ret = of_property_read_u32(np, "TP_MAX_X", &pdata->TP_MAX_X);
	if(ret){
		dev_err(dev, "fail to get TP_MAX_X\n");
		goto fail;
	}
	ret = of_property_read_u32(np, "TP_MAX_Y", &pdata->TP_MAX_Y);
	if(ret){
		dev_err(dev, "fail to get TP_MAX_Y\n");
		goto fail;
	}

	return pdata;
fail:
	kfree(pdata);
	return NULL;
}
#endif


#ifdef FT6x36_DOWNLOAD

int ft5x0x_download_i2c_Read(unsigned char *writebuf,
            int writelen, unsigned char *readbuf, int readlen)
{
    int ret;

	ret=fts_i2c_Read(g_i2c_client, writebuf,writelen,readbuf,readlen);
  
    return ret;
}
/*write data by i2c*/
int ft5x0x_download_i2c_Write(unsigned char *writebuf, int writelen)
{
    int ret;

   ret=fts_i2c_Write(g_i2c_client, writebuf,writelen);

    return ret;
}


int ft6x36_Enter_Debug(void)
{
    ft5x0x_ts_reset();
   // msleep(2); 
	pr_info("ft6x36_Enter_Debug\n");
    return ft6x36_Lib_Enter_Download_Mode();
}
//if return 0, main flash is ok, else download.
int ft6x36_IsDownloadMain(void)
{
    //add condition to check main flash
    return -1;
}
int ft6x36_DownloadMain(void)
{
    unsigned short fwlen = 0;
	pr_info("ft6x36_DownloadMain 1");
    if (ft6x36_Enter_Debug() < 0) {
        pr_info("-----enter debug mode failed\n");
        return -1;
    }
	pr_info("ft6x36_DownloadMain 2");
	if(1 == TP_VOL){
	    fwlen = sizeof(CTPM_MAIN_FW1);
	    pr_info("wangcq327 ---1 ----fwlen=%d\n", fwlen);

	    //return ft6x06_Lib_DownloadMain(CTPM_MAIN_FW, fwlen);
	    return ft6x36_Lib_DownloadMain(CTPM_MAIN_FW1,fwlen);
	}else if(0 == TP_VOL){
	    fwlen = sizeof(CTPM_MAIN_FW2);
	    pr_info("wangcq327 ---2 ----fwlen=%d\n", fwlen);

	    //return ft6x06_Lib_DownloadMain(CTPM_MAIN_FW, fwlen);
	    return ft6x36_Lib_DownloadMain(CTPM_MAIN_FW2,fwlen);	
	}
	return -1;
}

#if 1
//wangcq327 --- add start
int Check_TP_ID(void)
{
		int data[4] = {0};
		int res = 0;
		int rawdata = 0;
		int tp_vol = 0;
#define AUXADC_TP_VOLTAGE_CHANNEL 1
#ifdef AUXADC_TP_VOLTAGE_CHANNEL
		extern int sci_adc_get_value(unsigned chan, int scale);
		rawdata = sci_adc_get_value(AUXADC_TP_VOLTAGE_CHANNEL,false);
		if(rawdata < 0)
		{ 
#ifdef BUILD_LK
				printf("[adc_uboot]: get data error\n");
#endif
				return 0;

		}
#endif
		//tp_vol = data[0]*1000+data[1]*10;
		//printk("wangcq327 --- data[0]:%d  data[1]:%d  tp_vol:%d\n",data[0],data[1],tp_vol);
		printk("yedongyue ----------adc value= %d----------\n",rawdata);
		if(rawdata >2700 && rawdata < 3300){//欧雷登
			return TP_VOL = 0;
		}
		else if(rawdata < 4400 && rawdata > 3800){
			return TP_VOL = 1;
		}
		else{
			return TP_VOL = -1;
		}
}
//wangcq327 --- add end
#endif
int fts_ctpm_auto_download(struct i2c_client *client)
{
	u8 uc_host_fm_ver = FT_REG_FW_VER;
	u8 uc_tp_fm_ver;
	u8 uc_host_vendor_id = FT_REG_VENDOR_ID;
	u8 uc_tp_vendor_id=0;
	int i_ret;

    //fts_read_reg(client, FT_REG_VENDOR_ID, &uc_tp_vendor_id);
	//uc_host_vendor_id = fts_ctpm_get_i_file_vendor();
	//if(uc_tp_vendor_id!=uc_host_vendor_id)return 0;
	//wangcq327 --- add start	   
	fts_read_reg(client, FT_REG_VENDOR_ID,&uc_tp_vendor_id);
	printk("  yedongyue wangcq327 --- uc_tp_vendor_id == %d\n",uc_tp_vendor_id);
	printk("wangcq327 --- ID_pin_vol == %d \n",Check_TP_ID());
	//TP_VOL = 1;
	if(TP_VOL == 1){//鑫鹏?		//if(uc_tp_vendor_id == 0xA9) {//鑫鹏?
			fts_read_reg(client, FT_REG_FW_VER, &uc_tp_fm_ver);
			uc_host_fm_ver =CTPM_MAIN_FW1[0x90a];// fts_ctpm_get_i_file_ver();
			uc_host_vendor_id=CTPM_MAIN_FW1[0x908];
			pr_info("wangcq327 --- [FTS] uc_tp_fm_ver = 0x%x, uc_host_fm_ver = 0x%x\n",uc_tp_fm_ver, uc_host_fm_ver);
			if(uc_tp_fm_ver >= uc_host_fm_ver && uc_tp_vendor_id == uc_host_vendor_id)  return -1;
		//}else if(uc_tp_vendor_id != 0x49) {//若不为鑫鹏达，且不为冠盛，退出，若为冠盛，往下升
			//return -1;
		//};

	}else if(0 == TP_VOL){//冠盛
		//if(uc_tp_vendor_id == 0x49) {//冠盛
			fts_read_reg(client, FT_REG_FW_VER, &uc_tp_fm_ver);
			uc_host_fm_ver =CTPM_MAIN_FW2[0x90a];// fts_ctpm_get_i_file_ver();
			uc_host_vendor_id=CTPM_MAIN_FW2[0x908];
			if(uc_tp_fm_ver >= uc_host_fm_ver && uc_tp_vendor_id == uc_host_vendor_id )  return -1;
		//}else if(uc_tp_vendor_id != 0xd1){
	//		return -1;
		//};

	}else{
		printk("wangcq327 --- ID_pin_vol error!!! \n");
		return -1;
	}
//wangcq327 --- add end
       pr_info("wangcq327 --- [FTS] uc_tp_fm_ver = 0x%x, uc_host_fm_ver = 0x%x\n",uc_tp_fm_ver, uc_host_fm_ver);

#if 0 
	   if (/*the firmware in touch panel maybe corrupted */
		uc_tp_fm_ver == FT_REG_FW_VER ||
		/*the firmware in host flash is new, need upgrade */
	     uc_tp_fm_ver < uc_host_fm_ver ||

	    ) 
#endif
	    {
			g_i2c_client = client;
        	FTS_I2c_Read_Function fun_i2c_read = ft5x0x_download_i2c_Read;
        	FTS_I2c_Write_Function fun_i2c_write = ft5x0x_download_i2c_Write;
        	Init_I2C_Read_Func(fun_i2c_read);
        	Init_I2C_Write_Func(fun_i2c_write);
        	 if(ft6x36_IsDownloadMain() < 0)
        	 {
				#if 1
					pr_info("--------FTS---------download main\n");
					if(ft6x36_DownloadMain()<0)
					{
						pr_info("---------FTS---------Download main failed\n");
					}
				#endif
        	 } else
        		pr_info("--------FTS---------no download main\n");
		}

	return 0;
}

#endif


static int ft5x0x_ts_probe(struct i2c_client *client, const struct i2c_device_id *id)
{

	struct input_dev *input_dev;
	struct ft5x0x_ts_platform_data *pdata = client->dev.platform_data;
	int err = 0;
	unsigned char uc_reg_value;
	u8 chip_id,i;

	pr_info("[FST] %s: probe\n",__func__);
#ifdef CONFIG_OF
	struct device_node *np = client->dev.of_node;
	if (np && !pdata){
		pdata = ft5x0x_ts_parse_dt(&client->dev);
		if(pdata){
			client->dev.platform_data = pdata;
		}
		else{
			err = -ENOMEM;
			goto exit_alloc_platform_data_failed;
		}
	}
#endif
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		err = -ENODEV;
		goto exit_check_functionality_failed;
	}

	ft5x0x_ts = kzalloc(sizeof(*ft5x0x_ts), GFP_KERNEL);
	if (!ft5x0x_ts)	{
		err = -ENOMEM;
		goto exit_alloc_data_failed;
	}

	g_ft5x0x_ts = ft5x0x_ts;
	ft5x0x_ts->platform_data = pdata;
	this_client = client;
	ft5x0x_ts->client = client;
	ft5x0x_ts_hw_init(ft5x0x_ts);
	i2c_set_clientdata(client, ft5x0x_ts);
	client->irq = gpio_to_irq(pdata->irq_gpio_number);

	#ifdef CONFIG_I2C_SPRD
	sprd_i2c_ctl_chg_clk(client->adapter->nr, 400000);
	#endif

	err = ft5x0x_read_reg(FT5X0X_REG_CIPHER, &uc_reg_value);
	if (err < 0)
	{
		pr_err("[FST] read chip id error %x\n", uc_reg_value);
		err = -ENODEV;
		goto exit_chip_check_failed;
	}
       
	/* set report rate, about 70HZ */
	ft5x0x_write_reg(FT5X0X_REG_PERIODACTIVE, 7);
#if USE_WORK_QUEUE
	INIT_WORK(&ft5x0x_ts->pen_event_work, ft5x0x_ts_pen_irq_work);

	ft5x0x_ts->ts_workqueue = create_singlethread_workqueue("focal-work-queue");
	if (!ft5x0x_ts->ts_workqueue) {
		err = -ESRCH;
		goto exit_create_singlethread;
	}
#endif

#ifdef CONFIG_HAS_EARLYSUSPEND
	INIT_WORK(&ft5x0x_ts->resume_work, ft5x0x_ts_resume_work);
	ft5x0x_ts->ts_resume_workqueue = create_singlethread_workqueue("ft5x0x_ts_resume_work");
	if (!ft5x0x_ts->ts_resume_workqueue) {
		err = -ESRCH;
		goto exit_create_singlethread;
	}
#endif
	input_dev = input_allocate_device();
	if (!input_dev) {
		err = -ENOMEM;
		dev_err(&client->dev, "[FST] failed to allocate input device\n");
		goto exit_input_dev_alloc_failed;
	}
#ifdef TOUCH_VIRTUAL_KEYS
	ft5x0x_ts_virtual_keys_init();
#endif
	ft5x0x_ts->input_dev = input_dev;

	__set_bit(ABS_MT_TOUCH_MAJOR, input_dev->absbit);
	__set_bit(ABS_MT_POSITION_X, input_dev->absbit);
	__set_bit(ABS_MT_POSITION_Y, input_dev->absbit);
	__set_bit(ABS_MT_WIDTH_MAJOR, input_dev->absbit);
	__set_bit(KEY_MENU,  input_dev->keybit);
	__set_bit(KEY_BACK,  input_dev->keybit);
	__set_bit(KEY_HOMEPAGE,  input_dev->keybit);
	__set_bit(KEY_POWER,  input_dev->keybit);
	__set_bit(KEY_LEFT,  input_dev->keybit);
	__set_bit(KEY_RIGHT,  input_dev->keybit);
	__set_bit(KEY_UP,  input_dev->keybit);
	__set_bit(KEY_DOWN,  input_dev->keybit);
	__set_bit(KEY_U,  input_dev->keybit);
	__set_bit(KEY_O,  input_dev->keybit);
	__set_bit(KEY_W,  input_dev->keybit);
	__set_bit(KEY_M,  input_dev->keybit);
	__set_bit(KEY_E,  input_dev->keybit);
	__set_bit(KEY_C,  input_dev->keybit);
	__set_bit(KEY_S,  input_dev->keybit);
	__set_bit(KEY_V,  input_dev->keybit);
	__set_bit(KEY_Z,  input_dev->keybit);
	__set_bit(KEY_A,  input_dev->keybit);
	__set_bit(BTN_TOUCH, input_dev->keybit);

#if MULTI_PROTOCOL_TYPE_B
	input_mt_init_slots(input_dev, TS_MAX_FINGER,0);
#endif
	input_set_abs_params(input_dev,ABS_MT_POSITION_X, 0, pdata->TP_MAX_X, 0, 0);
	input_set_abs_params(input_dev,ABS_MT_POSITION_Y, 0, pdata->TP_MAX_Y, 0, 0);
    
	//input_set_abs_params(input_dev,ABS_MT_POSITION_X, 0, 540, 0, 0);
	//input_set_abs_params(input_dev,ABS_MT_POSITION_Y, 0, 960, 0, 0);

	input_set_abs_params(input_dev,ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params(input_dev,ABS_MT_WIDTH_MAJOR, 0, 255, 0, 0);
	#if 0
	/*ft5306's firmware is qhd, ft5316's firmware is 720p*/
	if (uc_reg_value == 0x0a || uc_reg_value == 0x0) {
		input_set_abs_params(input_dev,
			     ABS_MT_POSITION_X, 0, 720, 0, 0);
		input_set_abs_params(input_dev,
			     ABS_MT_POSITION_Y, 0, 1280, 0, 0);
	} else {
		input_set_abs_params(input_dev,
			     ABS_MT_POSITION_X, 0, 540, 0, 0);
		input_set_abs_params(input_dev,
			     ABS_MT_POSITION_Y, 0, 960, 0, 0);
	}
	input_set_abs_params(input_dev,
			     ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params(input_dev,
			     ABS_MT_WIDTH_MAJOR, 0, 255, 0, 0);

	#endif

	set_bit(EV_ABS, input_dev->evbit);
	set_bit(EV_KEY, input_dev->evbit);

	input_dev->name = FOCALTECH_TS_NAME;
	err = input_register_device(input_dev);
	if (err) {
		dev_err(&client->dev,
		"[FST] ft5x0x_ts_probe: failed to register input device: %s\n",
		dev_name(&client->dev));
		goto exit_input_register_device_failed;
	}

#if USE_THREADED_IRQ
	err = request_threaded_irq(client->irq, NULL, ft5x0x_ts_interrupt, 
		IRQF_TRIGGER_FALLING | IRQF_ONESHOT| IRQF_NO_SUSPEND, client->name, ft5x0x_ts);
#else
	err = request_irq(client->irq, ft5x0x_ts_interrupt,
		IRQF_TRIGGER_FALLING | IRQF_ONESHOT| IRQF_NO_SUSPEND, client->name, ft5x0x_ts);
#endif
	if (err < 0) {
		dev_err(&client->dev, "[FST] ft5x0x_probe: request irq failed %d\n",err);
		goto exit_irq_request_failed;
	}

#ifdef CONFIG_HAS_EARLYSUSPEND
	ft5x0x_ts->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	ft5x0x_ts->early_suspend.suspend = ft5x0x_ts_suspend;
	ft5x0x_ts->early_suspend.resume	= ft5x0x_ts_resume;
	register_early_suspend(&ft5x0x_ts->early_suspend);
#endif

focaltech_get_upgrade_array(client);

#ifdef SYSFS_DEBUG	
fts_create_sysfs(client);
#endif

#ifdef FTS_CTL_IIC	
if (ft_rw_iic_drv_init(client) < 0)	
{
	dev_err(&client->dev, "%s:[FTS] create fts control iic driver failed\n",	__func__);
}
#endif

#ifdef FTS_GESTRUE
	init_para(480,800,60,0,0);
    //fts_write_reg(i2c_client, 0xd0, 0x01);
#endif

#ifdef SPRD_AUTO_UPGRADE
	printk("********************Enter CTP Auto Upgrade********************\n");
	fts_ctpm_auto_upgrade(client);
#endif   

#ifdef FT6x36_DOWNLOAD
	fts_ctpm_auto_download(client);
#endif

#ifdef APK_DEBUG
	ft5x0x_create_apk_debug_channel(client);
#endif

#if USE_WAIT_QUEUE
	thread = kthread_run(touch_event_handler, 0, "focal-wait-queue");
	if (IS_ERR(thread))
	{
		err = PTR_ERR(thread);
		PRINT_ERR("failed to create kernel thread: %d\n", err);
	}
#endif

#ifdef TPD_PROXIMITY
	err = misc_register(&focaltech_pls_device);
	if (err)
	{
		APS_ERR("%s: focaltech_pls_device register failed err = %d \n", __func__,err);
	}
	focaltech_pls_input_dev = input_allocate_device();
	if (!focaltech_pls_input_dev) 
	{
		APS_ERR("%s: input allocate device failed\n", __func__);
	}
	else
	{
		focaltech_pls_input_dev->name = FOCALTECH_PLS_INPUT_NAME;
		focaltech_pls_input_dev->phys  = FOCALTECH_PLS_INPUT_NAME;
		focaltech_pls_input_dev->id.bustype = BUS_I2C;
		focaltech_pls_input_dev->dev.parent = &client->dev;
		focaltech_pls_input_dev->id.vendor = 0x0001;
		focaltech_pls_input_dev->id.product = 0x0001;
		focaltech_pls_input_dev->id.version = 0x0010;
		__set_bit(EV_ABS, focaltech_pls_input_dev->evbit);	
		input_set_abs_params(focaltech_pls_input_dev, ABS_DISTANCE, 0, 1, 0, 0);
		input_set_abs_params(focaltech_pls_input_dev, ABS_MISC, 0, 100001, 0, 0);
		err = input_register_device(focaltech_pls_input_dev);
		if (err < 0)
		{
			APS_ERR("%s: input device regist failed err = %d \n", __func__,err);
		}
	}
#endif


	return 0;

exit_irq_request_failed:
	input_unregister_device(input_dev);
exit_input_register_device_failed:
	input_free_device(input_dev);
exit_input_dev_alloc_failed:
exit_create_singlethread:
exit_chip_check_failed:
	gpio_free(pdata->irq_gpio_number);
	gpio_free(pdata->reset_gpio_number);
	kfree(ft5x0x_ts);
exit_alloc_data_failed:
exit_check_functionality_failed:
	ft5x0x_ts = NULL;
	i2c_set_clientdata(client, ft5x0x_ts);
exit_alloc_platform_data_failed:
	return err;
}

static int ft5x0x_ts_remove(struct i2c_client *client)
{
	struct ft5x0x_ts_data *ft5x0x_ts = i2c_get_clientdata(client);

	pr_info("==ft5x0x_ts_remove=\n");
	
	#ifdef SYSFS_DEBUG
	fts_release_sysfs(client);
	#endif
	#ifdef FTS_CTL_IIC	
	ft_rw_iic_drv_exit();
	#endif
	#ifdef APK_DEBUG
	ft5x0x_release_apk_debug_channel();
	#endif
	
#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&ft5x0x_ts->early_suspend);
#endif
	free_irq(client->irq, ft5x0x_ts);
	input_unregister_device(ft5x0x_ts->input_dev);
	input_free_device(ft5x0x_ts->input_dev);
#if USE_WORK_QUEUE
	cancel_work_sync(&ft5x0x_ts->pen_event_work);
	destroy_workqueue(ft5x0x_ts->ts_workqueue);
#endif
#ifdef CONFIG_HAS_EARLYSUSPEND
	cancel_work_sync(&ft5x0x_ts->resume_work);
	destroy_workqueue(ft5x0x_ts->ts_resume_workqueue);
#endif
	kfree(ft5x0x_ts);
	ft5x0x_ts = NULL;
	i2c_set_clientdata(client, ft5x0x_ts);

	return 0;
}

static const struct i2c_device_id ft5x0x_ts_id[] = {
	{ FOCALTECH_TS_NAME, 0 },{ }
};

static int ft5x0x_suspend(struct i2c_client *client, pm_message_t mesg)
{
	PRINT_INFO("ft5x0x_suspend\n");
	return 0;
}
static int ft5x0x_resume(struct i2c_client *client)
{
	PRINT_INFO("ft5x0x_resume\n");
	return 0;
}

MODULE_DEVICE_TABLE(i2c, ft5x0x_ts_id);

static const struct of_device_id focaltech_of_match[] = {
       { .compatible = "focaltech,focaltech_ts", },
       { }
};
MODULE_DEVICE_TABLE(of, focaltech_of_match);
static struct i2c_driver ft5x0x_ts_driver = {
	.probe		= ft5x0x_ts_probe,
	.remove		= ft5x0x_ts_remove,
	.id_table	= ft5x0x_ts_id,
	.driver	= {
		.name	= FOCALTECH_TS_NAME,
		.owner	= THIS_MODULE,
		.of_match_table = focaltech_of_match,
	},
	.suspend = ft5x0x_suspend,
	.resume = ft5x0x_resume,
};

static int __init ft5x0x_ts_init(void)
{
	return i2c_add_driver(&ft5x0x_ts_driver);
}

static void __exit ft5x0x_ts_exit(void)
{
	i2c_del_driver(&ft5x0x_ts_driver);
}

module_init(ft5x0x_ts_init);
module_exit(ft5x0x_ts_exit);

MODULE_AUTHOR("<wenfs@Focaltech-systems.com>");
MODULE_DESCRIPTION("FocalTech ft5x0x TouchScreen driver");
MODULE_LICENSE("GPL");
