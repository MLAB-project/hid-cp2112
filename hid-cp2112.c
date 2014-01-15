/*
  hid-cp2112.c - Silicon Labs HID USB to SMBus master bridge
  Copyright (c) 2013 Uplogix, Inc.
  David Barksdale <dbarksdale@uplogix.com>

  This program is free software; you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation; either version 2 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program; if not, write to the Free Software
  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

/*
  The Silicon Labs CP2112 chip is a USB HID device which provides an
  SMBus controller for talking to slave devices. The host communicates
  with the CP2112 via raw HID reports.
 */

#include <linux/hid.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include "hid-ids.h"

enum {
	CP2112_GET_VERSION_INFO = 0x05,
	CP2112_SMBUS_CONFIG = 0x06,
	CP2112_DATA_READ_REQUEST = 0x10,
	CP2112_DATA_WRITE_READ_REQUEST = 0x11,
	CP2112_DATA_READ_FORCE_SEND = 0x12,
	CP2112_DATA_READ_RESPONSE = 0x13,
	CP2112_DATA_WRITE_REQUEST = 0x14,
	CP2112_TRANSFER_STATUS_REQUEST = 0x15,
	CP2112_TRANSFER_STATUS_RESPONSE = 0x16,
	CP2112_CANCEL_TRANSFER = 0x17,
};

enum {
	STATUS0_IDLE = 0x00,
	STATUS0_BUSY = 0x01,
	STATUS0_COMPLETE = 0x02,
	STATUS0_ERROR = 0x03,
};

enum {
	STATUS1_TIMEOUT_NACK = 0x00,
	STATUS1_TIMEOUT_BUS = 0x01,
	STATUS1_ARBITRATION_LOST = 0x02,
	STATUS1_READ_INCOMPLETE = 0x03,
	STATUS1_WRITE_INCOMPLETE = 0x04,
	STATUS1_SUCCESS = 0x05,
};

/* All values are in big-endian */
struct __attribute__ ((__packed__)) cp2112_smbus_config {
	uint32_t clock_speed; /* Hz */
	uint8_t device_address; /* Stored in the upper 7 bits */
	uint8_t auto_send_read; /* 1 = enabled, 0 = disabled */
	uint16_t write_timeout; /* ms, 0 = no timeout */
	uint16_t read_timeout; /* ms, 0 = no timeout */
	uint8_t scl_low_timeout; /* 1 = enabled, 0 = disabled */
	uint16_t retry_time; /* # of retries, 0 = no limit */
};

/* Number of times to request transfer status before giving up waiting for a
   transfer to complete. */
static const int XFER_TIMEOUT = 100;

/* Time in ms to wait for a CP2112_DATA_READ_RESPONSE or
   CP2112_TRANSFER_STATUS_RESPONSE. */
static const int RESPONSE_TIMEOUT = 50;

static const struct hid_device_id cp2112_devices[] = {
	{ HID_USB_DEVICE(USB_VENDOR_ID_CYGNAL, USB_DEVICE_ID_CYGNAL_CP2112) },
	{ }
};
MODULE_DEVICE_TABLE(hid, cp2112_devices);

struct cp2112_device {
	struct i2c_adapter adap;
	struct hid_device *hdev;
	wait_queue_head_t wait;
	uint8_t read_data[61];
	uint8_t read_length;
	int xfer_status;
	atomic_t read_avail;
	atomic_t xfer_avail;
};

static int cp2112_wait(struct cp2112_device *dev, atomic_t *avail)
{
	int ret = 0;

	ret = wait_event_interruptible_timeout(dev->wait,
		atomic_read(avail), msecs_to_jiffies(RESPONSE_TIMEOUT));
	if (-ERESTARTSYS == ret)
		return ret;
	if (!ret)
		return -ETIMEDOUT;
	atomic_set(avail, 0);
	return 0;
}

static int cp2112_hid_get(struct hid_device *hdev, unsigned char report_number,
			  uint8_t *data, size_t count,
			  unsigned char report_type)
{
	uint8_t *buf;
	int ret;

	buf = kmalloc(count, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;
	ret = hdev->hid_get_raw_report(hdev, report_number, buf, count,
				       report_type);
	memcpy(data, buf, count);
	kfree(buf);
	return ret;
}

static int cp2112_hid_output(struct hid_device *hdev, uint8_t *data,
			     size_t count, unsigned char report_type)
{
	uint8_t *buf;
	int ret;

	buf = kmemdup(data, count, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;
	ret = hdev->hid_output_raw_report(hdev, buf, count, report_type);
	kfree(buf);
	return ret;
}

static int cp2112_xfer_status(struct cp2112_device *dev)
{
	struct hid_device *hdev = dev->hdev;
	uint8_t buf[2];
	int ret;

	buf[0] = CP2112_TRANSFER_STATUS_REQUEST;
	buf[1] = 0x01;
	atomic_set(&dev->xfer_avail, 0);
	ret = cp2112_hid_output(hdev, buf, 2, HID_OUTPUT_REPORT);
	if (ret < 0) {
		hid_warn(hdev, "Error requesting status: %d\n", ret);
		return ret;
	}
	ret = cp2112_wait(dev, &dev->xfer_avail);
	if (ret)
		return ret;
	return dev->xfer_status;
}

static int cp2112_read(struct cp2112_device *dev, uint8_t *data, size_t size)
{
	struct hid_device *hdev = dev->hdev;
	uint8_t buf[3];
	int ret;

	buf[0] = CP2112_DATA_READ_FORCE_SEND;
	*(uint16_t *)&buf[1] = htons(size);
	atomic_set(&dev->read_avail, 0);
	ret = cp2112_hid_output(hdev, buf, 3, HID_OUTPUT_REPORT);
	if (ret < 0) {
		hid_warn(hdev, "Error requesting data: %d\n", ret);
		return ret;
	}
	ret = cp2112_wait(dev, &dev->read_avail);
	if (ret)
		return ret;
	hid_dbg(hdev, "read %d of %d bytes requested\n",
		dev->read_length, size);
	if (size > dev->read_length)
		size = dev->read_length;
	memcpy(data, dev->read_data, size);
	return dev->read_length;
}

static int cp2112_read_req(uint8_t *buf, uint8_t slave_address, uint16_t length)
{
	if (length < 1 || length > 512)
		return -EINVAL;
	buf[0] = CP2112_DATA_READ_REQUEST;
	buf[1] = slave_address << 1;
	*(uint16_t *)&buf[2] = htons(length);
	return 4;
}

static int cp2112_write_read_req(uint8_t *buf, uint8_t slave_address,
				 uint16_t length, uint8_t command,
				 uint8_t *data, uint8_t data_length)
{
	if (length < 1 || length > 512 || data_length > 15)
		return -EINVAL;
	buf[0] = CP2112_DATA_WRITE_READ_REQUEST;
	buf[1] = slave_address << 1;
	*(uint16_t *)&buf[2] = htons(length);
	buf[4] = data_length + 1;
	buf[5] = command;
	memcpy(&buf[6], data, data_length);
	return data_length + 6;
}

static int cp2112_write_req(uint8_t *buf, uint8_t slave_address,
			    uint8_t command, uint8_t *data, uint8_t data_length)
{
	if (data_length > 60)
		return -EINVAL;
	buf[0] = CP2112_DATA_WRITE_REQUEST;
	buf[1] = slave_address << 1;
	buf[2] = data_length + 1;
	buf[3] = command;
	memcpy(&buf[4], data, data_length);
	return data_length + 4;
}

static int cp2112_xfer(struct i2c_adapter *adap, uint16_t addr,
		       unsigned short flags, char read_write, uint8_t command,
		       int size, union i2c_smbus_data *data)
{
	struct cp2112_device *dev = (struct cp2112_device *)adap->algo_data;
	struct hid_device *hdev = dev->hdev;
	uint8_t buf[64];
	uint16_t word;
	size_t count;
	size_t read_length = 0;
	size_t timeout;
	int ret;

	hid_dbg(hdev, "%s addr 0x%x flags 0x%x cmd 0x%x size %d\n",
		read_write == I2C_SMBUS_WRITE ? "write" : "read",
		addr, flags, command, size);
	switch (size) {
	case I2C_SMBUS_BYTE:
		read_length = 1;
		if (I2C_SMBUS_READ == read_write)
			count = cp2112_read_req(buf, addr, read_length);
		else
			count = cp2112_write_req(buf, addr, data->byte, NULL,
						 0);
		break;
	case I2C_SMBUS_BYTE_DATA:
		read_length = 1;
		if (I2C_SMBUS_READ == read_write)
			count = cp2112_write_read_req(buf, addr, read_length,
						      command, NULL, 0);
		else
			count = cp2112_write_req(buf, addr, command,
						 &data->byte, 1);
		break;
	case I2C_SMBUS_WORD_DATA:
		read_length = 2;
		word = htons(data->word);
		if (I2C_SMBUS_READ == read_write)
			count = cp2112_write_read_req(buf, addr, read_length,
						      command, NULL, 0);
		else
			count = cp2112_write_req(buf, addr, command,
						 (uint8_t *)&word, 2);
		break;
	case I2C_SMBUS_PROC_CALL:
		size = I2C_SMBUS_WORD_DATA;
		read_write = I2C_SMBUS_READ;
		read_length = 2;
		word = htons(data->word);
		count = cp2112_write_read_req(buf, addr, read_length, command,
					      (uint8_t *)&word, 2);
		break;
	case I2C_SMBUS_I2C_BLOCK_DATA:
		size = I2C_SMBUS_BLOCK_DATA;
		/* fallthrough */
	case I2C_SMBUS_BLOCK_DATA:
		if (I2C_SMBUS_READ == read_write) {
			count = cp2112_write_read_req(buf, addr,
						      I2C_SMBUS_BLOCK_MAX,
						      command, NULL, 0);
		} else {
			count = cp2112_write_req(buf, addr, command,
						 data->block,
						 data->block[0] + 1);
		}
		break;
	case I2C_SMBUS_BLOCK_PROC_CALL:
		size = I2C_SMBUS_BLOCK_DATA;
		read_write = I2C_SMBUS_READ;
		count = cp2112_write_read_req(buf, addr, I2C_SMBUS_BLOCK_MAX,
					      command, data->block,
					      data->block[0] + 1);
		break;
	default:
		hid_warn(hdev, "Unsupported transaction %d\n", size);
		return -EOPNOTSUPP;
	}
	if (count < 0)
		return count;
	ret = hid_hw_open(hdev);
	if (ret) {
		hid_err(hdev, "hw open failed\n");
		return ret;
	}
	ret = hid_hw_power(hdev, PM_HINT_FULLON);
	if (ret < 0) {
		hid_err(hdev, "power management error: %d\n", ret);
		goto hid_close;
	}
	ret = cp2112_hid_output(hdev, buf, count, HID_OUTPUT_REPORT);
	if (ret < 0) {
		hid_warn(hdev, "Error starting transaction: %d\n", ret);
		goto power_normal;
	}
	for (timeout = 0; timeout < XFER_TIMEOUT; ++timeout) {
		ret = cp2112_xfer_status(dev);
		if (-EBUSY == ret)
			continue;
		if (ret < 0)
			goto power_normal;
		break;
	}
	if (XFER_TIMEOUT <= timeout) {
		hid_warn(hdev, "Transfer timed out, cancelling.\n");
		buf[0] = CP2112_CANCEL_TRANSFER;
		buf[1] = 0x01;
		ret = cp2112_hid_output(hdev, buf, 2, HID_OUTPUT_REPORT);
		if (ret < 0) {
			hid_warn(hdev, "Error cancelling transaction: %d\n",
				 ret);
		}
		ret = -ETIMEDOUT;
		goto power_normal;
	}
	if (I2C_SMBUS_WRITE == read_write) {
		ret = 0;
		goto power_normal;
	}
	if (I2C_SMBUS_BLOCK_DATA == size)
		read_length = ret;
	ret = cp2112_read(dev, buf, read_length);
	if (ret < 0)
		goto power_normal;
	if (ret != read_length) {
		hid_warn(hdev, "short read: %d < %d\n", ret, read_length);
		ret = -EIO;
		goto power_normal;
	}
	switch (size) {
	case I2C_SMBUS_BYTE:
	case I2C_SMBUS_BYTE_DATA:
		data->byte = buf[0];
		break;
	case I2C_SMBUS_WORD_DATA:
		data->word = ntohs(*(uint16_t *)buf);
		break;
	case I2C_SMBUS_BLOCK_DATA:
		if (read_length > I2C_SMBUS_BLOCK_MAX) {
			ret = -EPROTO;
			goto power_normal;
		}
		memcpy(data->block, buf, read_length);
		break;
	}
	ret = 0;
power_normal:
	hid_hw_power(hdev, PM_HINT_NORMAL);
hid_close:
	hid_hw_close(hdev);
	hid_dbg(hdev, "transfer finished: %d\n", ret);
	return ret;
}

static uint32_t cp2112_functionality(struct i2c_adapter *adap)
{
	return I2C_FUNC_SMBUS_BYTE |
		I2C_FUNC_SMBUS_BYTE_DATA |
		I2C_FUNC_SMBUS_WORD_DATA |
		I2C_FUNC_SMBUS_BLOCK_DATA |
		I2C_FUNC_SMBUS_I2C_BLOCK |
		I2C_FUNC_SMBUS_PROC_CALL |
		I2C_FUNC_SMBUS_BLOCK_PROC_CALL;
}

static const struct i2c_algorithm smbus_algorithm = {
	.smbus_xfer	= cp2112_xfer,
	.functionality	= cp2112_functionality,
};

static int cp2112_probe(struct hid_device *hdev, const struct hid_device_id *id)
{
	struct cp2112_device *dev;
	uint8_t buf[64];
	struct cp2112_smbus_config *config;
	int ret;

	ret = hid_parse(hdev);
	if (ret) {
		hid_err(hdev, "parse failed\n");
		return ret;
	}
	ret = hid_hw_start(hdev, HID_CONNECT_HIDRAW);
	if (ret) {
		hid_err(hdev, "hw start failed\n");
		return ret;
	}
	ret = hid_hw_open(hdev);
	if (ret) {
		hid_err(hdev, "hw open failed\n");
		goto hid_stop;
	}
	ret = hid_hw_power(hdev, PM_HINT_FULLON);
	if (ret < 0) {
		hid_err(hdev, "power management error: %d\n", ret);
		goto hid_close;
	}
	ret = cp2112_hid_get(hdev, CP2112_GET_VERSION_INFO, buf, 3,
			     HID_FEATURE_REPORT);
	if (ret != 3) {
		hid_err(hdev, "error requesting version\n");
		if (ret >= 0)
			ret = -EIO;
		goto power_normal;
	}
	hid_info(hdev, "Part Number: 0x%02X Device Version: 0x%02X\n",
		 buf[1], buf[2]);
	ret = cp2112_hid_get(hdev, CP2112_SMBUS_CONFIG, buf,
			     sizeof(*config) + 1, HID_FEATURE_REPORT);
	if (ret != sizeof(*config) + 1) {
		hid_err(hdev, "error requesting SMBus config\n");
		if (ret >= 0)
			ret = -EIO;
		goto power_normal;
	}
	config = (struct cp2112_smbus_config *)&buf[1];
	config->retry_time = htons(1);
	ret = cp2112_hid_output(hdev, buf, sizeof(*config) + 1,
				HID_FEATURE_REPORT);
	if (ret != sizeof(*config) + 1) {
		hid_err(hdev, "error setting SMBus config\n");
		if (ret >= 0)
			ret = -EIO;
		goto power_normal;
	}
	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (!dev) {
		hid_err(hdev, "out of memory\n");
		ret = -ENOMEM;
		goto power_normal;
	}
	dev->hdev = hdev;
	hid_set_drvdata(hdev, (void *)dev);
	dev->adap.owner		= THIS_MODULE;
	dev->adap.class		= I2C_CLASS_HWMON;
	dev->adap.algo		= &smbus_algorithm;
	dev->adap.algo_data	= dev;
	dev->adap.dev.parent	= &hdev->dev;
	snprintf(dev->adap.name, sizeof(dev->adap.name),
		 "CP2112 SMBus Bridge on hiddev%d", hdev->minor);
	init_waitqueue_head(&dev->wait);
	hid_device_io_start(hdev);
	ret = i2c_add_adapter(&dev->adap);
	hid_device_io_stop(hdev);
	hid_hw_power(hdev, PM_HINT_NORMAL);
	hid_hw_close(hdev);
	if (ret) {
		hid_err(hdev, "error registering i2c adapter\n");
		kfree(dev);
		goto hid_stop;
	}
	hid_dbg(hdev, "adapter registered\n");
	return ret;
power_normal:
	hid_hw_power(hdev, PM_HINT_NORMAL);
hid_close:
	hid_hw_close(hdev);
hid_stop:
	hid_hw_stop(hdev);
	return ret;
}

static void cp2112_remove(struct hid_device *hdev)
{
	struct cp2112_device *dev = hid_get_drvdata(hdev);

	hid_hw_stop(hdev);
	i2c_del_adapter(&dev->adap);
	kfree(dev);
}

static int cp2112_raw_event(struct hid_device *hdev, struct hid_report *report,
			    uint8_t *data, int size)
{
	struct cp2112_device *dev = hid_get_drvdata(hdev);

	switch (data[0]) {
	case CP2112_TRANSFER_STATUS_RESPONSE:
		hid_dbg(hdev, "xfer status: %02x %02x %04x %04x\n",
			data[1], data[2], htons(*(uint16_t *)&data[3]),
			htons(*(uint16_t *)&data[5]));
		switch (data[1]) {
		case STATUS0_IDLE:
			dev->xfer_status = -EAGAIN;
			break;
		case STATUS0_BUSY:
			dev->xfer_status = -EBUSY;
			break;
		case STATUS0_COMPLETE:
			dev->xfer_status = ntohs(*(uint16_t *)&data[5]);
			break;
		case STATUS0_ERROR:
			switch (data[2]) {
			case STATUS1_TIMEOUT_NACK:
			case STATUS1_TIMEOUT_BUS:
				dev->xfer_status = -ETIMEDOUT;
				break;
			default:
				dev->xfer_status = -EIO;
			}
			break;
		default:
			dev->xfer_status = -EINVAL;
			break;
		}
		atomic_set(&dev->xfer_avail, 1);
		break;
	case CP2112_DATA_READ_RESPONSE:
		hid_dbg(hdev, "read response: %02x %02x\n", data[1], data[2]);
		dev->read_length = data[2];
		if (dev->read_length > sizeof(dev->read_data))
			dev->read_length = sizeof(dev->read_data);
		memcpy(dev->read_data, &data[3], dev->read_length);
		atomic_set(&dev->read_avail, 1);
		break;
	default:
		hid_err(hdev, "unknown report\n");
		return 0;
	}
	wake_up_interruptible(&dev->wait);
	return 1;
}

static struct hid_driver cp2112_driver = {
	.name = "cp2112",
	.id_table = cp2112_devices,
	.probe = cp2112_probe,
	.remove = cp2112_remove,
	.raw_event = cp2112_raw_event,
};

module_hid_driver(cp2112_driver);
MODULE_DESCRIPTION("Silicon Labs HID USB to SMBus master bridge");
MODULE_AUTHOR("David Barksdale <dbarksdale@uplogix.com>");
MODULE_LICENSE("GPL");

