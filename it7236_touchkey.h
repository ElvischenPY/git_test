#ifndef IT7236_TOUCHKEY_H
#define IT7236_TOUCHKEY_H
#define MAX_BUFFER_SIZE		144
#define DEVICE_NAME		"IT7236"
#define DEVICE_VENDOR		0
#define DEVICE_PRODUCT		0
#define DEVICE_VERSION		0
#define VERSION_ABOVE_ANDROID_20
#define IOC_MAGIC		'd'
#define IOCTL_SET 		_IOW(IOC_MAGIC, 1, struct ioctl_cmd168)
#define IOCTL_GET 		_IOR(IOC_MAGIC, 2, struct ioctl_cmd168)

struct ioctl_cmd168 {
	unsigned short bufferIndex;
	unsigned short length;
	unsigned short buffer[MAX_BUFFER_SIZE];
};

struct IT7236_ts_data {
	struct i2c_client *client;
	struct input_dev *input_dev;
	int use_irq;
	struct workqueue_struct *ts_workqueue;
	struct mutex device_mode_mutex;
	const char *name;
	bool reg_enable;
	u32 power_enable;
	u32 enable_gpio;
	u32 enable_gpio_flags;
	u32 irqflags;
	u32 irq_gpio;
	u32 irq_gpio_flags;
	u32 panel_miny;
	u32 panel_maxy;
	u32 y_min;
	u32 y_max;
	struct regulator *vdd;
	struct regulator *vcc_i2c;
	uint8_t debug_log_level;
	const char *gold_sample;
	int gold_sample_size;
};

static struct IT7236_ts_data *gl_ts;

#endif