#include <linux/delay.h>
#include <linux/device.h>
#include <linux/freezer.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/workqueue.h>
#include <linux/of_gpio.h>
#include <linux/regulator/consumer.h>
#include <asm/ioctl.h>
#include <linux/miscdevice.h>
#include <linux/fs.h>

#define NXU8010_IOCTL_TYPE            'N'        // 'N' XU magic number
#define NXU8010_IOCTL_SIZE             unsigned char[2]         // 1byte addr, 1byte data
#define NXU8010_IOCTL_READ_NUM         1         // select Read
#define NXU8010_IOCTL_WRITE_NUM        2         // select Write

#define NXU8010_READ  _IOWR(NXU8010_IOCTL_TYPE, NXU8010_IOCTL_READ_NUM, NXU8010_IOCTL_SIZE)
#define NXU8010_WRITE _IOWR(NXU8010_IOCTL_TYPE, NXU8010_IOCTL_WRITE_NUM, NXU8010_IOCTL_SIZE)

#define NAME "nxu8010"
#define NXU8010_VDD_MIN_UV  2850000
#define NXU8010_VDD_MAX_UV  28500000

#define CHIP_ID             0x2A
#define R_DATA_LOW_BYTE     0x20
#define R_DATA_HIGHT_BYTE   0x21
#define G_DATA_LOW_BYTE     0x22
#define G_DATA_HIGHT_BYTE   0x23
#define B_DATA_LOW_BYTE     0x24
#define B_DATA_HIGHT_BYTE   0x25
#define W_DATA_LOW_BYTE     0x26
#define W_DATA_HIGHT_BYTE   0x27

struct init_array{
	uint8_t addr;
	uint8_t value;
};

static struct init_array nxu8010_init_code[]=
{
	{0x01, 0x08},
	{0x00, 0x02},
	{0x02, 0x00},
	{0x03, 0x70},
	{0x04, 0x28},
	{0x05, 0x00},
	{0x06, 0x40},
	{0x07, 0x0A},
	{0x30, 0x0C},
	{0x31, 0x00},
	{0x32, 0x00},
	{0x33, 0x00},
	{0x08, 0x08},
	{0x09, 0x18},
	{0x0A, 0x00},
	{0x0B, 0x00},
	{0x0C, 0x00},
	{0x0D, 0x00},
	{0x0E, 0x08},
	{0x0F, 0x18},
	{0x10, 0x00},
	{0x11, 0x00},
	{0x12, 0x00},
	{0x13, 0x00},
	{0x14, 0x08},
	{0x15, 0x18},
	{0x16, 0x00},
	{0x17, 0x00},
	{0x18, 0x00},
	{0x19, 0x00},
	{0x1A, 0x08},
	{0x1B, 0x18},
	{0x1C, 0x00},
	{0x1D, 0x00},
	{0x1E, 0x00},
	{0x1F, 0x00},
	{0x35, 0x15},
	{0x50, 0x01},
	{0x51, 0x23},
	{0x52, 0x45},
	{0x53, 0x67},
	{0x54, 0x89},
	{0x55, 0xAB},
	{0x56, 0xCD},
	{0x57, 0xEF},
	{0x41, 0x00},
	{0x40, 0x01},
	{0x01, 0x04},
	{0x34, 0x00},
	{0x00, 0x22},
	{0x00, 0x02},
	{0x07, 0x0A},
	{0x08, 0x08},
	{0x0E, 0x08},
	{0x14, 0x08},
	{0x1A, 0x08},
	{0x34, 0x03},
 };

struct sensor_data {
	uint16_t data_r;
	uint16_t data_g;
	uint16_t data_b;
	uint16_t data_w;
};

struct nxu8010_info {
	struct i2c_client	*i2c;
	struct mutex lock;

	int power_on;
	struct regulator *vdd;

	struct sensor_data data;
	struct class *nxu8010_class;
	struct device *nxu8010_device;
};

struct nxu8010_info *_sinfo;

static int nxu8010_i2c_rxdata(struct i2c_client *i2c,
	uint8_t *rxData, int length)
{
	int ret;
	struct i2c_msg msgs[] = {
		{
			.addr = i2c->addr,
			.flags = 0,
			.len = 1,
			.buf = rxData,
		},
		{
			.addr = i2c->addr,
			.flags = I2C_M_RD,
			.len = length,
			.buf = rxData,
		},
	};
	uint8_t addr = rxData[0];

	ret = i2c_transfer(i2c->adapter, msgs, ARRAY_SIZE(msgs));
	if (ret < 0) {
		dev_err(&i2c->dev, "%s: transfer failed.", __func__);
		return ret;
	} else if (ret != ARRAY_SIZE(msgs)) {
		dev_err(&i2c->dev, "%s: transfer failed(size error).\n",
				__func__);
		return -ENXIO;
	}

	dev_info(&i2c->dev, "RxData: len=%02x, addr=%02x, data=%02x",
		length, addr, rxData[0]);

	return 0;
}

static int nxu8010_i2c_txdata(struct i2c_client *i2c,
	uint8_t *txData, int length)
{
	int ret;

	struct i2c_msg msg[] = {
		{
			.addr = i2c->addr,
			.flags = 0,
			.len = length,
			.buf = txData,
		},
	};

	ret = i2c_transfer(i2c->adapter, msg, ARRAY_SIZE(msg));
	if (ret < 0) {
		dev_err(&i2c->dev, "%s: transfer failed.", __func__);
		return ret;
	} else if (ret != ARRAY_SIZE(msg)) {
		dev_err(&i2c->dev, "%s: transfer failed(size error).",
				__func__);
		return -ENXIO;
	}

	dev_info(&i2c->dev, "TxData: len=%02x, addr=%02x data=%02x",
		length, txData[0], txData[1]);

	return 0;
}

static int sensor_regulator_configure(struct nxu8010_info *data, bool on)
{
	int rc;

	if (!on) {
		if (regulator_count_voltages(data->vdd) > 0)
			regulator_set_voltage(data->vdd, 0,
				NXU8010_VDD_MAX_UV);

		regulator_put(data->vdd);
	} 
	else {
		data->vdd = regulator_get(&data->i2c->dev, "vdd");
		if (IS_ERR(data->vdd)) {
			rc = PTR_ERR(data->vdd);
			dev_err(&data->i2c->dev, "Regulator get failed vdd rc=%d\n", rc);
			return rc;
		}

		if (regulator_count_voltages(data->vdd) > 0) {
			rc = regulator_set_voltage(data->vdd,
				NXU8010_VDD_MIN_UV, NXU8010_VDD_MAX_UV);
			if (rc) {
				dev_err(&data->i2c->dev, "Regulator set failed vdd rc=%d\n", rc);
				return rc;
			}
		}
	}

	return 0;
}

static int sensor_regulator_power_on(struct nxu8010_info *data, bool on)
{
	int rc = 0;

	if (!on) {
		rc = regulator_disable(data->vdd);
		if (rc) {
			dev_err(&data->i2c->dev, "Regulator vdd disable failed rc=%d\n", rc);
			return rc;
		}
	} 
	else {
		rc = regulator_enable(data->vdd);
		if (rc) {
			dev_err(&data->i2c->dev, "Regulator vdd enable failed rc=%d\n", rc);
			return rc;
		}
	}

	pr_info("Sensor regulator power on =%d\n", on);
	return rc;
}

static void nxu8010_sensor_init(struct nxu8010_info *sinfo)
{
	uint8_t buf[2];
	int i;
	int err = 0;

	int num = sizeof(nxu8010_init_code)/sizeof(struct init_array);

	pr_info("%s num = %d",__func__, num);

	for(i=0; i<num; i++)
	{
		buf[0] = nxu8010_init_code[i].addr;
		buf[1] = nxu8010_init_code[i].value;
		err = nxu8010_i2c_txdata(sinfo->i2c, buf, sizeof(buf));
		if (err < 0)
			dev_err(&sinfo->i2c->dev, "%s failed.", __func__);
	}
}

static int get_sensor_data(struct nxu8010_info *sinfo)
{
	uint8_t buf, buffer[8];
	int i = 0;
	int err = 0;

	for(i=0; i<8; i++)
	{
		buf = R_DATA_LOW_BYTE + i;
		err = nxu8010_i2c_rxdata(sinfo->i2c, &buf, 1);
		if (err < 0)
			dev_err(&sinfo->i2c->dev, "%s failed.", __func__);

		buffer[i] = buf;
	}

	sinfo->data.data_r = buffer[1] << 8 | buffer[0];
	sinfo->data.data_g = buffer[3] << 8 | buffer[2];
	sinfo->data.data_b = buffer[5] << 8 | buffer[4];
	sinfo->data.data_w = buffer[7] << 8 | buffer[6];

	dev_err(&sinfo->i2c->dev, "RGB sensor read data R[0x%x], G[0x%x], B[0x%x], W[0x%x]",
		sinfo->data.data_r, sinfo->data.data_g, sinfo->data.data_b, sinfo->data.data_w);

	return err;
}

static ssize_t nxu8010_reg_show(
	struct device *dev, struct device_attribute *attr, char *buf)
{
	struct nxu8010_info *sinfo = dev_get_drvdata(dev);
	int i=0, count=0, err=0;
	int num = sizeof(nxu8010_init_code)/sizeof(struct init_array);
	uint8_t buffer[num];

	for(i=0; i<num; i++)
	{
		buffer[i] = nxu8010_init_code[i].addr;
		err = nxu8010_i2c_rxdata(sinfo->i2c, &buffer[i], 1);
		if (err < 0)
			dev_err(&sinfo->i2c->dev, "%s failed.", __func__);

		count += snprintf(&buf[count], PAGE_SIZE, "0x%2x: 0x%2x\n",
			nxu8010_init_code[i].addr, buffer[i]);
	}

	return count;
}

static ssize_t nxu8010_reg_store(
	struct device *dev, struct device_attribute *attr, char const *buf, size_t size)
{
	struct nxu8010_info *sinfo = dev_get_drvdata(dev);
	int err = 0;
	u32 value[2];
	uint8_t buffer[2];

	sscanf(buf, "%x,%x", &value[0], &value[1]);
	buffer[0] = (uint8_t)value[0];
	buffer[1] = (uint8_t)value[1];
	pr_err("register[0x%02x], velue[0x%02x]\n",buffer[0], buffer[1]);

	err = nxu8010_i2c_txdata(sinfo->i2c, buffer, 2);
	if (err < 0)
		dev_err(&sinfo->i2c->dev, "%s failed.", __func__);

	return size;
}


static ssize_t nxu8010_value_show(
	struct device *dev, struct device_attribute *attr, char *buf)
{
	struct nxu8010_info *sinfo = dev_get_drvdata(dev);

	sinfo->data.data_r = 0;
	sinfo->data.data_g = 0;
	sinfo->data.data_b = 0;
	sinfo->data.data_w = 0;

	get_sensor_data(sinfo);

	return snprintf(buf, PAGE_SIZE, "%d %d %d %d\n",
			sinfo->data.data_r,
			sinfo->data.data_g,
			sinfo->data.data_b,
			sinfo->data.data_w);
}

static ssize_t nxu8010_enable_show(
	struct device *dev, struct device_attribute *attr, char *buf)
{
	struct nxu8010_info *sinfo = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%d\n", sinfo->power_on);
}

static ssize_t nxu8010_enable_store(
	struct device *dev, struct device_attribute *attr, char const *buf, size_t size)
{
	struct nxu8010_info *sinfo = dev_get_drvdata(dev);
	uint8_t buffer[2];
	int err;

	sscanf(buf, "%d", &sinfo->power_on);
	sensor_regulator_power_on(sinfo, sinfo->power_on);
	
	buffer[0] = 0x00;
	if(sinfo->power_on)
		buffer[1] = 0x02;
	else
		buffer[1] = 0x22;
	pr_err("register[0x%02X], velue[0x%02X]\n",buffer[0], buffer[1]);

	err = nxu8010_i2c_txdata(sinfo->i2c, buffer, 2);
	if (err < 0)
	{
		dev_err(&sinfo->i2c->dev, "%s failed.", __func__);
		sinfo->power_on = 0;
	}

	return size;
}

static ssize_t nxu8010_correlation_show(
	struct device *dev, struct device_attribute *attr, char *buf)
{
	struct nxu8010_info *sinfo = dev_get_drvdata(dev);
	uint8_t buffer;

	buffer = 0x2A;
	nxu8010_i2c_rxdata(sinfo->i2c, &buffer, 1);

	return snprintf(buf, PAGE_SIZE, "0x%2X\n", buffer);
}

static DEVICE_ATTR(correlation, S_IRUGO, nxu8010_correlation_show, NULL);
static DEVICE_ATTR(register, S_IRUGO|S_IWUSR|S_IWGRP, nxu8010_reg_show, nxu8010_reg_store);
static DEVICE_ATTR(value, S_IRUGO, nxu8010_value_show, NULL);
static DEVICE_ATTR(enable, S_IRUGO|S_IWUSR|S_IWGRP, nxu8010_enable_show, nxu8010_enable_store);

static int sys_device_create(struct nxu8010_info *sinfo)
{
	int err = 0;

	sinfo->nxu8010_class= class_create(THIS_MODULE, NAME);
	if (IS_ERR(sinfo->nxu8010_class))
		dev_err(&sinfo->i2c->dev, "Failed to create class(%s)!\n", NAME);

	sinfo->nxu8010_device = device_create(sinfo->nxu8010_class, NULL, 0, NULL, "device");
	if (IS_ERR(sinfo->nxu8010_device))
		dev_err(&sinfo->i2c->dev,"Failed to create device(%s_device)!\n", NAME);

	dev_set_drvdata(sinfo->nxu8010_device, sinfo);

	if ((err = device_create_file(sinfo->nxu8010_device, &dev_attr_enable)) < 0)
		dev_err(&sinfo->i2c->dev,"Failed to create device file(%s)!\n", dev_attr_enable.attr.name);

	if ((err = device_create_file(sinfo->nxu8010_device, &dev_attr_value)) < 0)
		dev_err(&sinfo->i2c->dev,"Failed to create device file(%s)!\n", dev_attr_value.attr.name);

	if ((err = device_create_file(sinfo->nxu8010_device, &dev_attr_register)) < 0)
		dev_err(&sinfo->i2c->dev,"Failed to create device file(%s)!\n", dev_attr_register.attr.name);

	if ((err = device_create_file(sinfo->nxu8010_device, &dev_attr_correlation)) < 0)
		dev_err(&sinfo->i2c->dev,"Failed to create device file(%s)!\n", dev_attr_correlation.attr.name);

	return err;
}

static int nxu8010_Open(struct inode *inode, struct file *file)
{
	struct nxu8010_info *sinfo = _sinfo;

	sensor_regulator_power_on(sinfo, 1);
	sinfo->power_on = 1;

	return 0;
}

static int nxu8010_Release(struct inode *inode, struct file *file)
{
	struct nxu8010_info *sinfo = _sinfo;

	sensor_regulator_power_on(sinfo, 0);
	sinfo->power_on = 0;

	return 0;
}

static long nxu8010_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	struct nxu8010_info *sinfo = _sinfo;

	uint8_t data[2];
	uint8_t buffer;
	int ret = 0;

	switch (cmd) {
		case NXU8010_READ:
		case NXU8010_WRITE:
			if (argp == NULL) {
				dev_err(&sinfo->i2c->dev, "invalid argument.");
				return -EINVAL;
			}
			if (copy_from_user(&data, argp, sizeof(data))) {
				dev_err(&sinfo->i2c->dev, "copy_from_user failed.");
				return -EFAULT;
			}
			break;
	}

	switch (cmd) {
		case NXU8010_READ:
			dev_vdbg(&sinfo->i2c->dev, "IOCTL_READ called.");
			buffer = data[0];
			ret = nxu8010_i2c_rxdata(sinfo->i2c, &buffer, 1);
			if (ret < 0)
				return ret;
			data[1] = buffer;

			if (copy_to_user(argp, &data, sizeof(data))) {
				dev_err(&sinfo->i2c->dev, "copy_to_user failed.");
				return -EFAULT;
			}
			break;

		case NXU8010_WRITE:
			dev_vdbg(&sinfo->i2c->dev, "IOCTL_WRITE called.");
			ret = nxu8010_i2c_txdata(sinfo->i2c, data, 2);
			if (ret < 0)
				return ret;
			break;

		default:
			dev_err(&sinfo->i2c->dev, "wrong command.");
			return -ENOTTY;
	}

	return 0;
}


static const struct file_operations nxu8010_fops = {
	.owner = THIS_MODULE,
	.open = nxu8010_Open,
	.release = nxu8010_Release,
	.unlocked_ioctl = nxu8010_ioctl,
};

static struct miscdevice nxu8010_dev = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = NAME,
	.fops = &nxu8010_fops,
};

static int nxu8010_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	static struct nxu8010_info *sinfo;
	int rc = 0;
	uint8_t buffer,buf[2];

	pr_info("[St]+%s\n", __func__);
	sinfo = devm_kzalloc(&client->dev, sizeof(struct nxu8010_info),
				 GFP_KERNEL);
	if (!sinfo) {
		dev_err(&client->dev, "[St]%s: no mem for sinfo\n", __func__);
		return -ENOMEM;
	}

	mutex_init(&sinfo->lock);
	sinfo->i2c = client;
	i2c_set_clientdata(client, sinfo);

	_sinfo = sinfo;
	pr_info("nxu8010 register misc device");
	rc = misc_register(&nxu8010_dev);
	if (rc < 0) {
		dev_err(&client->dev, "%s: nxu8010 register failed", __func__);
		goto err1;
	}

	pr_info("nxu8010 power configure and on");
	rc = sensor_regulator_configure(sinfo, 1);
	if(rc < 0)
		goto err2;

	rc = sensor_regulator_power_on(sinfo, 1);
	if(rc < 0)
		goto err3;

	nxu8010_sensor_init(sinfo);

	buffer = CHIP_ID;
	rc = nxu8010_i2c_rxdata(sinfo->i2c, &buffer, 1);
	if(rc < 0)
	{
		dev_err(&client->dev,"read chip id failed");
		goto err4;
	}
	pr_info("nxu8010 chip id = [%x]", buffer);

	get_sensor_data(sinfo);

	rc = sys_device_create(sinfo);
	if(rc < 0)
		goto err4;

	buf[0] = 0x00;
	buf[1] = 0x22;
	nxu8010_i2c_txdata(sinfo->i2c, buf, 2);
	sensor_regulator_power_on(sinfo, 0);

	pr_info("[St]-%s\n", __func__);

	return 0;

err4:
	sensor_regulator_power_on(sinfo, 0);
err3:
	sensor_regulator_configure(sinfo, 0);
err2:
	misc_deregister(&nxu8010_dev);
err1:
	devm_kfree(&client->dev, sinfo);

	return rc;
}

static int nxu8010_remove(struct i2c_client *client)
{
	struct nxu8010_info *sinfo = i2c_get_clientdata(client);

	sensor_regulator_power_on(sinfo, 0);
	sensor_regulator_configure(sinfo, 0);
	mutex_destroy(&sinfo->lock);
	devm_kfree(&client->dev, sinfo);
	return 0;
}

static int nxu8010_suspend(struct device *dev)
{
	return 0;
}

static int nxu8010_resume(struct device *dev)
{
	return 0;
}

static const struct dev_pm_ops nxu8010_pm_ops = {
	.suspend	= nxu8010_suspend,
	.resume		= nxu8010_resume,
};

static const struct i2c_device_id nxu8010_id[] = {
	{NAME, 0 },
	{ }
};

static struct of_device_id nxu8010_match_table[] = {
	{ .compatible = "nxu,nxu8010", },
	{ },
};

static struct i2c_driver nxu8010_driver = {
	.probe		= nxu8010_probe,
	.remove		= nxu8010_remove,
	.id_table	= nxu8010_id,
	.driver = {
		.name	= NAME,
		.owner  = THIS_MODULE,
		.of_match_table = nxu8010_match_table,
		.pm		= &nxu8010_pm_ops,
	},
};

static int __init nxu8010_init(void)
{
	pr_info("nxu8010 driver: initialize.");
	return i2c_add_driver(&nxu8010_driver);
}

static void __exit nxu8010_exit(void)
{
	pr_info("nxu8010 driver: release.");
	i2c_del_driver(&nxu8010_driver);
}

module_init(nxu8010_init);
module_exit(nxu8010_exit);

MODULE_AUTHOR("Shuaitao <shuait@huatune.com>");
MODULE_DESCRIPTION("nxu8010 RGB sensor driver");
MODULE_LICENSE("GPL");
