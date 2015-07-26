struct ioctl_cmd168 {
	unsigned short slaveAddr;
	unsigned short bufferIndex;
	unsigned short length;
	unsigned short buffer[MAX_BUFFER_SIZE];
};

//====================================================================================

static long ite7236_ioctl(struct file *filp, unsigned int cmd,unsigned long arg)
{
	int retval = 0;
	int i;
	unsigned char buffer[MAX_BUFFER_SIZE];
	struct ioctl_cmd168 data;

	memset(&data, 0, sizeof(struct ioctl_cmd168));

	switch (cmd) {
	case IOCTL_SET:
		pr_info("[IT7236] : =IOCTL_SET=\n");

		if (!access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd))) {
			retval = -EFAULT;
			return (retval);
		}

		if ( copy_from_user(&data, (int __user *)arg, sizeof(struct ioctl_cmd168)) ) {
			retval = -EFAULT;
			return (retval);
		}

		i2c_client->addr = data.slaveAddr;
		retval = i2cAdvancedWriteToIt7236( i2c_client,
				(unsigned char) data.bufferIndex,
				&(buffer[1]),
				(unsigned char)data.length );
		break;

	case IOCTL_GET:
		pr_info("[IT7236] : =IOCTL_GET=\n");

		if (!access_ok(VERIFY_WRITE, (void __user *)arg, _IOC_SIZE(cmd))) {
			retval = -EFAULT;
			return (retval);
		}

		if (!access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd))) {
			retval = -EFAULT;
			return (retval);
		}

		if ( copy_from_user(&data, (int __user *)arg, sizeof(struct ioctl_cmd168)) ) {
			retval = -EFAULT;
			return (retval);
		}

		i2c_client->addr = data.slaveAddr;
		retval = i2cAdvancedReadFromIt7236(i2c_client,
				(unsigned char) data.bufferIndex,
				(unsigned char*) buffer,
				(unsigned char) data.length);

		for (i = 0; i < data.length; i++) {
			data.buffer[i] = (unsigned short) buffer[i];
		}

		if ( copy_to_user((int __user *)arg, &data, sizeof(struct ioctl_cmd168)) ) {
			retval = -EFAULT;
			return (retval);
		}
		break;

	default:
		retval = -ENOTTY;
		break;
	}

	return (retval);
}
