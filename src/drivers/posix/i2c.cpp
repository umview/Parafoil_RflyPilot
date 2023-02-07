#include "i2c.h"
#define PX4_ERROR (-1)
#define PX4_OK 0
#define ENOENT    2
#define OK 0
#define EINVAL   22

i2c::i2c(unsigned char device_addr)
{
    _addr = device_addr;
}


int i2c::init(const char *i2c_dev)
{
	int ret = PX4_ERROR;

	// Open the actual I2C device
	// char dev_path[16] {};
    // const char      *i2c_dev = PCA9685_DEVICE_BASE_PATH;
	// snprintf(dev_path, sizeof(dev_path), "/dev/i2c-%i", get_device_bus());
	_fd = open(i2c_dev, O_RDWR);

	if (_fd < 0) {
		printf("failed to init I2C code: %d \n",_fd);
		ret = -ENOENT;
		goto out;
	}
	else{
		ret = PX4_OK;
	}

	// call the probe function to check whether the device is present
	// ret = probe();

	// if (ret != OK) {
	// 	DEVICE_DEBUG("probe failed");
	// 	goto out;
	// }

	// do base class init, which will create device node, etc
	// ret = CDev::init();

	// if (ret != OK) {
	// 	DEVICE_DEBUG("cdev init failed");
	// 	goto out;
	// }

	// tell the world where we are
	// DEVICE_DEBUG("on I2C bus %d at 0x%02x", get_device_bus(), get_device_address());

out:

	if ((ret != OK) && !(_fd < 0)) {
		close(_fd);
		printf("I2C is closed. _fd: %d \n",_fd);
		_fd = -1;
	}

	return ret;
}


int i2c::transfer(const uint8_t *send, const unsigned send_len, uint8_t *recv, const unsigned recv_len)
{
	int ret = PX4_ERROR;
	unsigned retry_count = 0;
    unsigned char device_addr = _addr;
    // device_addr >>= 1;

	if (_fd < 0) {
		printf("I2C device not opened\n");
		return PX4_ERROR;
	}

	do {
		// printf("transfer out %p/%u  in %p/%u \n\r", send, send_len, recv, recv_len);

		unsigned msgs = 0;
		struct i2c_msg msgv[2] {};

		if (send_len > 0) {
			msgv[msgs].addr = device_addr;
			msgv[msgs].flags = 0;
			msgv[msgs].buf = const_cast<uint8_t *>(send);
			msgv[msgs].len = send_len;
			msgs++;
		}

		if (recv_len > 0) {
			msgv[msgs].addr = device_addr;
			msgv[msgs].flags = I2C_M_RD;
			msgv[msgs].buf = recv;
			msgv[msgs].len = recv_len;
			msgs++;
		}

		if (msgs == 0) {
			return -EINVAL;
		}

		i2c_rdwr_ioctl_data packets{};
		packets.msgs  = msgv;
		packets.nmsgs = msgs;

		int ret_ioctl = ioctl(_fd, I2C_RDWR, (unsigned long)&packets);

		if (ret_ioctl == -1) {
			printf("I2C transfer failed\n");
			ret = PX4_ERROR;

		} else {
			// success
			ret = PX4_OK;
			break;
		}

	} while (retry_count++ < _retries);

	return ret;
}