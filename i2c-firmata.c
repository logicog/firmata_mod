// SPDX-License-Identifier: GPL-2.0
/* I2C Driver for microcontrollers running Firmata firmware making
 * peripherial devices of the microntroller available through a UART
 * to the host
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include "firmata.h"

// TODO: base this on the FIRMATA transfer size
#define FIRMATA_I2C_BUF_SIZE	100

/* Message codes in an I2C SYSEX message being sent */
#define I2C_REQUEST		0x76
#define I2C_CONFIG		0x78

struct firmata_i2c {
	struct platform_device *pdev;
	struct i2c_adapter adapter;
	u8 port;
	/*
	 * Buffer to hold the packet for read or write transfers to be sent
	 * over the serial line. One is enough since we can't have multiple
	 * transfers in parallel on the i2c bus.
	 */
	u8 *buf;
	int buf_fill;
	// Since we have to sleep waiting for I2C_REPLYs, we must ensure only one request is ongoing
	// a time
	struct mutex lock;
	struct completion reply_received;
};


static int i2c_reply_cb(struct platform_device *pdev, const u8 rxbuf[], int len)
{
	struct firmata_i2c *i2c = platform_get_drvdata(pdev);
	uint16_t addr, reg;
	int i;

	pr_info("i2c_reply_cb: firmata_i2c pointer %p, bytes %02x %02x %02x, len %d\n",
		(void *) i2c, rxbuf[0], rxbuf[1], rxbuf[2], len);

	if (rxbuf[0] != I2C_REPLY) {
		pr_err("firmata i2c_reply_cb: expected I2C_REPLY message, got message code 0x%02x\n", rxbuf[0]);
		return -1;
	}
	// Discard any data sent outside a read transaction
//	if (spin_trylock(&i2c->lock))
//		return len;

	pr_info("i2c_reply_cb: in read transfer\n");
	addr = rxbuf[1] | (rxbuf[2] << 7); // TODO: 10 bit mode?? Transaction sequence???
	addr &= 0x7f;
	reg = rxbuf[3] | (rxbuf[4] << 7);
	pr_info("i2c_reply_cb: addr 0x%04x reg 0x%04x\n", addr, reg);

	i = 5;
	while ((rxbuf[i] != END_SYSEX) && (i / 2 < FIRMATA_I2C_BUF_SIZE)) {
		i2c->buf[(i - 5) / 2] = (rxbuf[i] & 0x7F) | ((rxbuf[i + 1] & 0x7F) << 7);
		i += 2;
	}

	pr_info("i2c_reply_cb: complete called\n");
	complete(&i2c->reply_received);
	pr_info("i2c_reply_cb: complete done\n");

	return len;
}

static int i2c_read(struct firmata_i2c *i2c, u16 addr, u8 *data, u16 data_len)
{
	int timeout;

	pr_info("firmata: i2c_read called, len: %d\n", data_len);
	if (data_len * 2 + 5 > FIRMATA_I2C_BUF_SIZE) // TODO: put this into the quirks
		return -EPROTO;

	mutex_lock(&i2c->lock);
	i2c->buf[0] = START_SYSEX;
	i2c->buf[1] = I2C_REQUEST;
	i2c->buf[2] = addr & 0x7f;
	i2c->buf[3] = 0b01 << 3;  // Read mode, 7 bit address
	i2c->buf[4] = data_len & 0x7f;
	i2c->buf[5] = data_len >> 7;
	i2c->buf[6] = END_SYSEX;

	firmata_serial_tx(i2c->pdev, i2c->buf, 7);
	if (completion_done(&i2c->reply_received)) {
		pr_info("i2c_read: Reiniting\n");
		reinit_completion(&i2c->reply_received);
	}

	timeout = wait_for_completion_timeout(&i2c->reply_received, msecs_to_jiffies(200));
	if (!timeout) {
		pr_info("i2c_read timeout\n");
		mutex_unlock(&i2c->lock);
                return -ETIMEDOUT;
	}
	memcpy(data, i2c->buf, data_len);

	mutex_unlock(&i2c->lock);
	return data_len;
}

static int i2c_write(struct firmata_i2c *i2c, u16 addr, u8 *data, u16 data_len)
{
	int i;

	pr_info("firmata: i2c_write called, len: %d\n", data_len);
	if (data_len * 2 + 4 > FIRMATA_I2C_BUF_SIZE) // TODO: put this into the quirks
		return -EPROTO;

	mutex_lock(&i2c->lock);
	i2c->buf[0] = START_SYSEX;
	i2c->buf[1] = I2C_REQUEST;
	i2c->buf[2] = addr & 0x7f;
	i2c->buf[3] = 0b00 << 3;  // Write mode, 7 bit address

	for (i = 0; i < data_len; i++) {
		i2c->buf[i * 2 + 4] = data[i] & 0x7f;
		i2c->buf[i * 2 + 5] = data[i] >> 7;
	}
	i2c->buf[data_len * 2 + 4] = END_SYSEX;

	firmata_serial_tx(i2c->pdev, i2c->buf, data_len * 2 + 5);
	mutex_unlock(&i2c->lock);

	return data_len;
}

static int firmata_i2c_xfer(struct i2c_adapter *adapter, struct i2c_msg *msgs, int num)
{
	struct firmata_i2c *i2c = i2c_get_adapdata(adapter);
	struct i2c_msg *pmsg;
	int i;

	for (i = 0; i < num; i++) {
		int ret;

		pmsg = &msgs[i];

		if (pmsg->flags & I2C_M_RD) {
			ret = i2c_read(i2c, pmsg->addr, pmsg->buf, pmsg->len);
			if (ret < 0)
				return ret;

			pmsg->len = ret;
		} else {
			ret = i2c_write(i2c, pmsg->addr, pmsg->buf, pmsg->len);
			if (ret < 0)
				return ret;
			if (ret != pmsg->len)
				return -EPROTO;
		}
	}

	return num;
}

static u32 firmata_i2c_func(struct i2c_adapter *a)
{
	return I2C_FUNC_I2C | I2C_FUNC_SMBUS_BYTE | I2C_FUNC_SMBUS_BYTE_DATA |
		I2C_FUNC_SMBUS_WORD_DATA | I2C_FUNC_SMBUS_BLOCK_PROC_CALL |
		I2C_FUNC_SMBUS_I2C_BLOCK;
}

static const struct i2c_algorithm firmata_i2c_usb_algorithm = {
	.master_xfer = firmata_i2c_xfer,
	.functionality = firmata_i2c_func,
};

static const struct i2c_adapter_quirks firmata_i2c_quirks = {
	.max_read_len = FIRMATA_I2C_BUF_SIZE,
	.max_write_len = FIRMATA_I2C_BUF_SIZE,
};

static int firmata_i2c_probe(struct platform_device *pdev)
{
	int ret;
	struct firmata_i2c *i2c;
	struct device *dev = &pdev->dev;
	struct firmata_platform_data *pdata = dev_get_platdata(&pdev->dev);

	i2c = devm_kzalloc(dev, sizeof(*i2c), GFP_KERNEL);
	if (!i2c)
		return -ENOMEM;

	i2c->buf = devm_kmalloc(dev, FIRMATA_I2C_BUF_SIZE, GFP_KERNEL);
	if (!i2c->buf)
		return -ENOMEM;

	i2c->pdev = pdev;
	i2c->port = pdata->port;

	/* setup i2c adapter description */
	i2c->adapter.owner = THIS_MODULE;
	i2c->adapter.class = I2C_CLASS_HWMON;
	i2c->adapter.algo = &firmata_i2c_usb_algorithm;
	i2c->adapter.quirks = &firmata_i2c_quirks;
	i2c->adapter.dev.parent = dev;
	ACPI_COMPANION_SET(&i2c->adapter.dev, ACPI_COMPANION(&pdev->dev));
	i2c->adapter.dev.of_node = dev->of_node;
	i2c_set_adapdata(&i2c->adapter, i2c);
	snprintf(i2c->adapter.name, sizeof(i2c->adapter.name), "%s-%s-%d",
		 "firmata-i2c", dev_name(pdev->dev.parent), i2c->port);

	mutex_init(&i2c->lock);
	init_completion(&i2c->reply_received);

	platform_set_drvdata(pdev, i2c);

	ret = firmata_register_event_cb(pdev, SYSEX_ID | I2C_REPLY, i2c_reply_cb);
	if (ret) {
		dev_err(dev, "failed to register event cb: %d\n", ret);
		return ret;
	}

	/* and finally attach to i2c layer */
	ret = i2c_add_adapter(&i2c->adapter);
	if (ret < 0)
		goto out_disable ;

	return 0;

out_disable:
	firmata_unregister_event_cb(pdev, SYSEX_ID | I2C_REPLY);
	return ret;
}

static void firmata_i2c_remove(struct platform_device *pdev)
{
	struct firmata_i2c *i2c = platform_get_drvdata(pdev);

	firmata_unregister_event_cb(pdev, SYSEX_ID | I2C_REPLY);
	i2c_del_adapter(&i2c->adapter);
}

static struct platform_driver i2c_firmata_driver = {
	.driver = {
		.name	= "firmata-i2c",
	},
	.probe		= firmata_i2c_probe,
	.remove		= firmata_i2c_remove,
};
module_platform_driver(i2c_firmata_driver);

MODULE_DESCRIPTION("I2C driver for Firmata firmware on Microcontrollers");
MODULE_AUTHOR("Birger Koblitz");
MODULE_LICENSE("GPL v2");
MODULE_VERSION("0.1");
MODULE_ALIAS("platform:firmata-i2c");
