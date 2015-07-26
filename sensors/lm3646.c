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
#include <linux/leds.h>

#define NAME "lm3646"
#define FLASH_BRIGHTNESS 16
#define TORCH_BRIGHTNESS 8

#define SILICON_REVISION   0x00
#define ENABLE_REGISTER    0x01
#define IVFM_MODE          0x02
#define NTC_TORCH_RAMP     0x03
#define FLASH_TIMING       0x04
#define MAX_LED_CURRENT_CONTROL    0x05
#define LED1_FLASH_CURRENT_CONTROL 0x06
#define LED1_TORCH_CURRENT_CONTROL 0x07
#define FLAG_REGISTER1     0x08
#define FLAG_REGISTER2     0x09

struct lm3646_info {
	struct i2c_client	*i2c;
	struct led_classdev cdev_flash;
	struct led_classdev cdev_torch;
	struct workqueue_struct *queue;
	struct delayed_work flash_work;
	struct delayed_work torch_work;
	struct mutex lock;
	int flash_brightness;
	int torch_brightness;
	uint16_t gpio_flash_en;
	uint16_t gpio_standby;
	uint16_t gpio_torch_en;
	struct class *lm3646_class;
	struct device *lm3646_device;
	int flash_current_velue;
};

static struct lm3646_info *_sinfo;

static int lm3646_i2c_rxdata(struct i2c_client *i2c,
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

	dev_vdbg(&i2c->dev, "RxData: len=%02x, addr=%02x, data=%02x",
		length, addr, rxData[0]);

	return 0;
}

static int lm3646_i2c_txdata(struct i2c_client *i2c,
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

	dev_vdbg(&i2c->dev, "TxData: len=%02x, addr=%02x data=%02x",
		length, txData[0], txData[1]);

	return 0;
}

static int lm3646_check_flag_register(void)
{
	struct lm3646_info *sinfo = _sinfo;
	uint8_t buffer;
	int err = 0;

	buffer = FLAG_REGISTER1;
	err = lm3646_i2c_rxdata(sinfo->i2c, &buffer, 1);
	if (err < 0) {
		dev_err(&sinfo->i2c->dev, "%s read FLAG_REGISTER1 failed.", __func__);
		return err;
	}
	dev_err(&sinfo->i2c->dev, "%s flag1 = 0x%x.", __func__, buffer);

	buffer = FLAG_REGISTER2;
	err = lm3646_i2c_rxdata(sinfo->i2c, &buffer, 1);
	if (err < 0) {
		dev_err(&sinfo->i2c->dev, "%s read FLAG_REGISTER1 failed.", __func__);
		return err;
	}
	dev_err(&sinfo->i2c->dev, "%s flag2 = 0x%x.", __func__, buffer);

	return 0;
}

static void lm3646_flash_worker(struct work_struct *work)
{
	struct lm3646_info *sinfo = _sinfo;
	uint8_t buffer_w[2],buffer;
	int err = 0;

	err = lm3646_check_flag_register();
	if(err < 0){
		dev_err(&sinfo->i2c->dev, "%s check flag register failed.", __func__);
		goto out;
	}

	if(sinfo->torch_brightness == 0)
	{
		gpio_direction_output(sinfo->gpio_torch_en, 0);
		gpio_direction_output(sinfo->gpio_flash_en, 0);
		gpio_direction_output(sinfo->gpio_standby, 0);
		goto out;
	}

	gpio_direction_output(sinfo->gpio_standby, 1);

	buffer = MAX_LED_CURRENT_CONTROL;
	err = lm3646_i2c_rxdata(sinfo->i2c, &buffer, 1);
	if (err < 0) {
		dev_err(&sinfo->i2c->dev, "%s failed.", __func__);
		goto out;
	}

	buffer_w[0] = MAX_LED_CURRENT_CONTROL;
	buffer_w[1] = buffer & (~0x0F);
	buffer_w[1] |= (sinfo->torch_brightness-1);
	err = lm3646_i2c_txdata(sinfo->i2c, buffer_w, 2);
	if (err < 0) {
		dev_err(&sinfo->i2c->dev, "%s failed.", __func__);
		goto out;
	}

	buffer = LED1_FLASH_CURRENT_CONTROL;
	err = lm3646_i2c_rxdata(sinfo->i2c, &buffer, 1);
	if (err < 0) {
		dev_err(&sinfo->i2c->dev, "%s failed.", __func__);
		goto out;
	}

	buffer_w[0] = LED1_FLASH_CURRENT_CONTROL;
	buffer_w[1] = buffer & (~0x7F);
	buffer_w[1] = (sinfo->flash_current_velue - 1);
	err = lm3646_i2c_txdata(sinfo->i2c, buffer_w, 2);
	if (err < 0) {
		dev_err(&sinfo->i2c->dev, "%s failed.", __func__);
		goto out;
	}

	gpio_direction_output(sinfo->gpio_flash_en, 1);
	msleep(50);
	gpio_direction_output(sinfo->gpio_flash_en, 0);
//	gpio_direction_output(sinfo->gpio_standby, 0);

out:
	pr_err("%s out,err = %d",__func__, err);
}

static void lm3646_flash_brightness_set(struct led_classdev *led_cdev,
	enum led_brightness value)
{
	struct lm3646_info *sinfo = _sinfo;

	pr_err("[St]%s:flashen=%d, enable=%d\n",__func__,sinfo->gpio_flash_en, value);
	sinfo->flash_brightness = value;

	queue_delayed_work(sinfo->queue,&sinfo->flash_work,msecs_to_jiffies(0));
}

static enum led_brightness lm3646_flash_brightness_get(struct led_classdev *led_cdev)
{
	struct lm3646_info *sinfo = _sinfo;

	pr_err("[St]%s: flash_brightness=%d\n",
		__func__, sinfo->flash_brightness);

	return sinfo->flash_brightness;
}


static void lm3646_torch_worker(struct work_struct *work)
{
	struct lm3646_info *sinfo = _sinfo;
	uint8_t buffer_w[2],buffer;
	int err = 0;

	err = lm3646_check_flag_register();
	if(err < 0){
		dev_err(&sinfo->i2c->dev, "%s check flag register failed.", __func__);
		goto out;
	}

	if(sinfo->torch_brightness == 0)
	{
		gpio_direction_output(sinfo->gpio_torch_en, 0);
		gpio_direction_output(sinfo->gpio_standby, 0);
		goto out;
	}

	gpio_direction_output(sinfo->gpio_standby, 1);

	buffer = MAX_LED_CURRENT_CONTROL;
	err = lm3646_i2c_rxdata(sinfo->i2c, &buffer, 1);
	if (err < 0) {
		dev_err(&sinfo->i2c->dev, "%s failed.", __func__);
		goto out;
	}

	buffer_w[0] = MAX_LED_CURRENT_CONTROL;
	buffer_w[1] = buffer & (~0x70);
	buffer_w[1] |= (sinfo->torch_brightness-1) << 4;
	err = lm3646_i2c_txdata(sinfo->i2c, buffer_w, 2);
	if (err < 0) {
		dev_err(&sinfo->i2c->dev, "%s failed.", __func__);
		goto out;
	}

	gpio_direction_output(sinfo->gpio_torch_en, 1);

out:
	pr_err("%s out,err = %d",__func__, err);
}

static void lm3646_torch_brightness_set(struct led_classdev *led_cdev,
	enum led_brightness value)
{
	struct lm3646_info *sinfo = _sinfo;

	pr_err("[St]%s:torchen=%d, enable=%d\n",__func__,sinfo->gpio_torch_en, value);
	sinfo->torch_brightness = value;

	queue_delayed_work(sinfo->queue,&sinfo->torch_work,msecs_to_jiffies(0));
}

static enum led_brightness lm3646_torch_brightness_get(struct led_classdev *led_cdev)
{
	struct lm3646_info *sinfo = _sinfo;

	pr_err("[St]%s: torch_brightness=%d\n",
		__func__, sinfo->torch_brightness);
	return sinfo->torch_brightness;
}

static int lm3646_device_init(void)
{
	struct lm3646_info *sinfo = _sinfo;
	uint8_t buffer_w[2],buffer;
	int err = 0;

	buffer = SILICON_REVISION;
	err = lm3646_i2c_rxdata(sinfo->i2c, &buffer, 1);
	if (err < 0) {
		dev_err(&sinfo->i2c->dev, "%s failed.", __func__);
		return err;
	}

	pr_err("%s read SILICON_REVISION [0x%x]\n",__func__, buffer);

	buffer_w[0] = LED1_TORCH_CURRENT_CONTROL;
	buffer_w[1] = 0xFF;
	err = lm3646_i2c_txdata(sinfo->i2c, &buffer, 1);
	if (err < 0) {
		dev_err(&sinfo->i2c->dev, "%s failed.", __func__);
		return err;
	}

	return 0;
}

static ssize_t lm3646_flash_current_velue_show(
	struct device *dev, struct device_attribute *attr, char *buf)
{
	struct lm3646_info *sinfo = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%d\n", sinfo->flash_current_velue);
}

static ssize_t lm3646_flash_current_velue_store(
	struct device *dev, struct device_attribute *attr, char const *buf, size_t size)
{
	struct lm3646_info *sinfo = dev_get_drvdata(dev);

	sscanf(buf, "%d", &sinfo->flash_current_velue);
	sinfo->flash_current_velue &= 0x7F;

	return size;
}

static DEVICE_ATTR(flash_current_velue, S_IRUGO|S_IWUSR|S_IWGRP, lm3646_flash_current_velue_show, lm3646_flash_current_velue_store);

static int sys_device_create(struct lm3646_info *sinfo)
{
	int err = 0;

	sinfo->lm3646_class= class_create(THIS_MODULE, NAME);
	if (IS_ERR(sinfo->lm3646_class))
		dev_err(&sinfo->i2c->dev, "Failed to create class(%s)!\n", NAME);

	sinfo->lm3646_device = device_create(sinfo->lm3646_class, NULL, 0, NULL, "device");
	if (IS_ERR(sinfo->lm3646_device))
		dev_err(&sinfo->i2c->dev,"Failed to create device(%s_device)!\n", NAME);

	dev_set_drvdata(sinfo->lm3646_device, sinfo);

	if ((err = device_create_file(sinfo->lm3646_device, &dev_attr_flash_current_velue)) < 0)
		dev_err(&sinfo->i2c->dev,"Failed to create device file(%s)!\n", dev_attr_flash_current_velue.attr.name);

	return err;
}

static int lm3646_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int rc = 0;
	struct device_node *node = client->dev.of_node;
	struct device_node *sub_node;
	static struct lm3646_info *sinfo;

	pr_info("[St]+%s\n", __func__);
	sinfo = devm_kzalloc(&client->dev, sizeof(struct lm3646_info),
				 GFP_KERNEL);
	if (!sinfo) {
		pr_err("[St]%s: no mem for sinfo\n", __func__);
		return -ENOMEM;
	}

	_sinfo = sinfo;
	mutex_init(&sinfo->lock);
	sinfo->i2c = client;
	i2c_set_clientdata(client, sinfo);

	rc = lm3646_device_init();
	if(rc < 0) {
		pr_err("lm3646_device_init fail!");
		goto error1;
	}

	sinfo->gpio_standby = of_get_named_gpio(sub_node, "lm3646,standby", 0);
	gpio_request(sinfo->gpio_standby, "FLASH_STANDBY");
	gpio_direction_output(sinfo->gpio_standby, 0);
	pr_info("[St]%s: gpio_standby=%d\n",__func__, sinfo->gpio_standby);

	sub_node = of_find_node_by_name(node, "lm3646,flash");
	if (!sub_node) {
		pr_err("[St]%s: lm3646,flash not found\n", __func__);
		goto error1;
	}
	rc = of_property_read_string(sub_node, "flash,name",
		&sinfo->cdev_flash.name);
	if (rc < 0) {
		pr_err("[St]%s: flash,name not found\n", __func__);
		goto error1;
	}
	if (!of_gpio_named_count(sub_node, "lm3646,flash-en")) {
		pr_err("[St]%s: lm3646,flash-en not found\n", __func__);
		goto error1;
	}
	sinfo->gpio_flash_en = of_get_named_gpio(sub_node, "lm3646,flash-en", 0);
	gpio_request(sinfo->gpio_flash_en, "FLASH_EN");
	gpio_direction_output(sinfo->gpio_flash_en, 0);
	pr_info("[St]%s: gpio_flash_en=%d\n",__func__, sinfo->gpio_flash_en);
	rc = of_property_read_string(sub_node, "linux,default-trigger",
		&sinfo->cdev_flash.default_trigger);
	if (rc < 0) {
		pr_err("[St]%s: flash default_trigger not found\n", __func__);
		goto error1;
	}
	sinfo->cdev_flash.max_brightness = FLASH_BRIGHTNESS;
	sinfo->cdev_flash.brightness_set = lm3646_flash_brightness_set;
	sinfo->cdev_flash.brightness_get = lm3646_flash_brightness_get;
	rc = led_classdev_register(&client->dev, &sinfo->cdev_flash);
	if (rc) {
		pr_err("[St]%s: cdev_flash registed error\n", __func__);
		goto error1;
	}

	sub_node = of_find_node_by_name(node, "lm3646,torch");
	if (!sub_node)
	{
		pr_err("[St]%s: lm3646,torch not found\n", __func__);
		goto error2;
	}
	rc = of_property_read_string(sub_node, "torch,name", &sinfo->cdev_torch.name);
	if (rc < 0) {
		pr_err("[St]%s: torch,name not found\n", __func__);
		goto error2;
	}
	if (!of_gpio_named_count(sub_node, "lm3646,torch-en"))
	{
		pr_err("[St]%s: lm3646,torch-en not found\n", __func__);
		goto error2;
	}
	sinfo->gpio_torch_en = of_get_named_gpio(sub_node, "lm3646,torch-en", 0);
	gpio_request(sinfo->gpio_torch_en, "TORCH_EN");
	gpio_direction_output(sinfo->gpio_torch_en, 0);
	pr_info("[St]%s: gpio_torch_en=%d\n",__func__, sinfo->gpio_torch_en);
	rc = of_property_read_string(sub_node, "linux,default-trigger",
		&sinfo->cdev_torch.default_trigger);
	if (rc < 0) {
		pr_err("[St]%s: torch default_trigger not found\n", __func__);
		goto error2;
	}
	sinfo->cdev_torch.max_brightness = TORCH_BRIGHTNESS;
	sinfo->cdev_torch.brightness_set = lm3646_torch_brightness_set;
	sinfo->cdev_torch.brightness_get = lm3646_torch_brightness_get;
	rc = led_classdev_register(&client->dev, &sinfo->cdev_torch);
	if (rc < 0) {
		pr_err("[St]%s: cdev_torch registed error\n", __func__);
		goto error2;
	}
	sinfo->queue = alloc_ordered_workqueue("lm3646", 0);
	if (!sinfo->queue) {
		rc = -ENOMEM;
		goto error3;
	}
	INIT_DELAYED_WORK(&sinfo->flash_work, lm3646_flash_worker);
	INIT_DELAYED_WORK(&sinfo->torch_work, lm3646_torch_worker);

	rc = sys_device_create(sinfo);
	if(rc < 0){
		pr_err("[St]%s: create sys device error\n", __func__);
		goto error3;
	}

	pr_info("[St]-%s\n", __func__);
	return 0;

error3:
	led_classdev_unregister(&sinfo->cdev_torch);
error2:
	led_classdev_unregister(&sinfo->cdev_flash);
error1:
	devm_kfree(&client->dev, sinfo);

	return rc;
}

static int lm3646_remove(struct i2c_client *client)
{
	struct lm3646_info *sinfo = _sinfo;

	mutex_destroy(&sinfo->lock);
	destroy_workqueue(sinfo->queue);
	led_classdev_unregister(&sinfo->cdev_torch);
	led_classdev_unregister(&sinfo->cdev_flash);
	devm_kfree(&client->dev, sinfo);
	return 0;
}

static struct of_device_id lm3646_match_table[] = {
	{ .compatible = "TI,lm3646", },
	{ },
};

static struct i2c_driver lm3646_driver = {
	.probe		= lm3646_probe,
	.remove		= lm3646_remove,
	.driver = {
		.name	= NAME,
		.owner  = THIS_MODULE,
		.of_match_table = lm3646_match_table,
	},
};

static int __init lm3646_init(void)
{
	pr_info("lm3646 driver: initialize.");
	return i2c_add_driver(&lm3646_driver);
}

static void __exit lm3646_exit(void)
{
	pr_info("lm3646 driver: release.");
	i2c_del_driver(&lm3646_driver);
}

module_init(lm3646_init);
module_exit(lm3646_exit);

MODULE_AUTHOR("Shuaitao <shuait@huatune.com>");
MODULE_DESCRIPTION("lm3646 LED driver");
MODULE_LICENSE("GPL");
