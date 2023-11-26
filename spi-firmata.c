// SPDX-License-Identifier: GPL-2.0
/* SPI Driver for microcontrollers running Firmata firmware making
 * peripherial devices of the microntroller available through a UART
 * to the host
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/spi/spi.h>
#include <linux/delay.h>
#include "firmata.h"

/* Message codes in an SPI_DATA SYSEX message */
#define SPI_BEGIN		0x00
#define SPI_DEVICE_CONFIG	0x01
#define SPI_TRANSFER		0x02
#define SPI_WRITE		0x03
#define SPI_READ		0x04
#define SPI_REPLY		0x05
#define SPI_END			0x06
#define SPI_WRITE_ACK		0x07

#define SPI_SEND_NO_REPLY 0
#define SPI_SEND_NORMAL_REPLY 1
#define SPI_SEND_EMPTY_REPLY 2

#define FIRMATA_SPI_MSB_FIRST 1
#define FIRMATA_SPI_LSB_FIRST 0

#define FIRMATA_SPI_BUF_SIZE 200

struct spi_device *firmata_spi_device;

/* Mapping of SPI channel to CS pin */
u8 cs_channel_mapping[] = {
	10,
	9
};

struct firmata_spi {
	struct platform_device *pdev;
	struct spi_controller *controller;
	u8 *buf;
	int buf_fill;
	struct mutex lock;
	struct completion reply_received;
	bool spi_enabled;
	uint8_t req_id;
	unsigned int speed;
};



static int spi_reply_cb(struct platform_device *pdev, const u8 rxbuf[], int len)
{
	struct firmata_spi *spi = platform_get_drvdata(pdev);
	uint8_t id, channel;
	int i;

	pr_info("%s: received %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x, len %d\n",
		__func__, rxbuf[0], rxbuf[1], rxbuf[2], rxbuf[3], rxbuf[4], rxbuf[5], rxbuf[6],
		rxbuf[7], rxbuf[8], rxbuf[9], rxbuf[10], rxbuf[11], len);

	// Discard any data sent outside an SPI transaction, e.g. spurious data on serial interface
	if (!mutex_is_locked(&spi->lock)) {
		pr_err("spurious SPI data received\n");
		return len;
	}

	if (rxbuf[0] != SPI_DATA) {
		dev_err(&pdev->dev, "%s: expected SPI_DATA message, got message code 0x%02x\n",
			__func__, rxbuf[0]);
		return -1;
	}

	if (rxbuf[1] != SPI_REPLY) {
		dev_err(&pdev->dev, "unknown SPI reply received: command byte 0x%02x\n", rxbuf[1]);
		return len;
	}
	id = rxbuf[2] >> 3;
	channel = rxbuf[2] & 0x7;
	pr_info("%s: device-id %d, channel %d\n", __func__, id, channel);
	if (spi->req_id != rxbuf[3]) {
		dev_warn(&pdev->dev, "%s: request-id mismatch, expecting %d, got %d",
			 __func__, spi->req_id, rxbuf[4]);
		return -EPROTO;
	}
	if ((len - 6) / 2 != rxbuf[4]) {
		dev_warn(&pdev->dev, "%s: length mismatch, sent %d, got %d\n", __func__, (len - 6) / 2, rxbuf[4]);
		return -EPROTO;
	}

	i = 5;
	while ((rxbuf[i] != END_SYSEX) && (i / 2 < FIRMATA_SPI_BUF_SIZE)) {
		spi->buf[(i - 5) / 2] = (rxbuf[i] & 0x7F) | ((rxbuf[i + 1] & 0x7F) << 7);
		i += 2;
	}

	complete(&spi->reply_received);

	return len;
}

static void send_dev_config(struct firmata_spi *spi, struct spi_device *spi_dev)
{
	struct firmata_platform_data *pdata = dev_get_platdata(&spi->pdev->dev);
	uint8_t buf[14];
	int id = 0;
	uint32_t speed = spi_dev->max_speed_hz; // TODO: use minimum of spi->speed and speed

	buf[0] = START_SYSEX;
	buf[1] = SPI_DATA;
	buf[2] = SPI_DEVICE_CONFIG;
	buf[3] = spi_dev->chip_select | id << 3;
	buf[4] = ((spi_dev->mode & 0x3) << 1) | (spi_dev->mode & SPI_LSB_FIRST ? 0 : BIT(0));
	buf[5] = speed & 0x7f;
	buf[6] = (speed >> 7) & 0x7f;
	buf[7] = (speed >> 15) & 0x7f;
	buf[8] = (speed >> 22) & 0x7f;
	buf[9] = (speed >> 29) & 0x7f;
	// Be compatible with implementations which only support the default (0 -> 8 bit)
	buf[10] = spi_dev->bits_per_word != 8 ? spi_dev->bits_per_word : 0;
	buf[11] = (spi_dev->mode & SPI_CS_HIGH ? BIT(1) : 0
			| pdata->host_controls_cs ? 0 : BIT(0));
	pr_info("%s: chip_select %d, pin %d\n", __func__,
		spi_dev->chip_select, cs_channel_mapping[spi_dev->chip_select]);
	buf[12] = cs_channel_mapping[spi_dev->chip_select] & 0x7f;
	buf[13] = END_SYSEX;

	firmata_serial_tx(spi->pdev, buf, 14);
}

static void send_spi_begin(struct firmata_spi *spi, int channel)
{
	uint8_t buf[5];

	buf[0] = START_SYSEX;
	buf[1] = SPI_DATA;
	buf[2] = SPI_BEGIN;
	buf[3] = channel;
	buf[4] = END_SYSEX;

	firmata_serial_tx(spi->pdev, buf, 5);
	msleep(1000);
}

static void send_spi_end(struct firmata_spi *spi, int channel)
{
	uint8_t buf[5];

	buf[0] = START_SYSEX;
	buf[1] = SPI_DATA;
	buf[2] = SPI_END;
	buf[3] = channel;
	buf[4] = END_SYSEX;

	firmata_serial_tx(spi->pdev, buf, 5);
}

static int firmata_setup(struct spi_device *spi_dev)
{
	struct spi_controller *ctrl = spi_dev->master;
	struct firmata_spi *spi = spi_controller_get_devdata(ctrl);

	pr_info("%s: called\n", __func__);
	mutex_lock(&spi->lock);
	if (!spi->spi_enabled)
		send_spi_begin(spi, spi_dev->chip_select);
	spi->spi_enabled = true;
	send_dev_config(spi, spi_dev);
	mutex_unlock(&spi->lock);
	return 0;
}

static void firmata_set_cs(struct spi_device *spi_dev, bool active)
{
	struct firmata_spi *spi = spi_controller_get_devdata(spi_dev->controller);
	struct firmata_platform_data *pdata = dev_get_platdata(&spi->pdev->dev);

	pr_info("%s called, active: %d, cs: %d\n", __func__, active, spi_dev->chip_select);
	if (!spi->spi_enabled)
		firmata_setup(spi_dev);
	if (pdata->host_controls_cs) {
		pr_info("%s: Would set CS now\n", __func__);
		// TODO: do this via gpiod_ calls
	}
}

static int firmata_spi_tx(struct firmata_spi *spi, u8 deselect_cs, const u8 *data, u16 data_len)
{
	int i;
	uint8_t device_id = 0, channel = 0;

	pr_info("firmata: spi_write called, len: %d\n", data_len);
	if (data_len * 2 + 4 > FIRMATA_SPI_BUF_SIZE) // TODO: put this into the quirks
		return -EPROTO;

	spi->buf[0] = START_SYSEX;
	spi->buf[1] = SPI_DATA;
	spi->buf[2] = SPI_TRANSFER;
	spi->buf[3] = channel | device_id << 3;
	spi->req_id = (spi->req_id + 1) % 0x7f;
	spi->buf[4] = spi->req_id;
	spi->buf[5] = deselect_cs;
	spi->buf[6] = data_len;

	for (i = 0; i < data_len; i++) {
		spi->buf[i * 2 + 7] = data[i] & 0x7f;
		spi->buf[i * 2 + 8] = data[i] >> 7;
	}
	spi->buf[data_len * 2 + 7] = END_SYSEX;

	firmata_serial_tx(spi->pdev, spi->buf, data_len * 2 + 8);

	return data_len;
}

static int firmata_transfer_one(struct spi_controller *ctrl, struct spi_device *spi_dev,
				struct spi_transfer *xfer)
{
	struct firmata_spi *spi = spi_controller_get_devdata(ctrl);
	int deselect_cs = 1;
	int timeout;
	int err = 0;

	pr_info("%s: transfer_one called\n", __FILE__);

	mutex_lock(&spi->lock);
	// TODO: Do we need to call transfer setup every time here to set
	// e.g. per-device transfer speed?

	if (!xfer->cs_change && !spi_transfer_is_last(ctrl, xfer))
		deselect_cs = 0; // attr = SPI_ATTR_LEAVE_SS_LOW;

	firmata_spi_tx(spi, deselect_cs, xfer->tx_buf, xfer->len);
	if (completion_done(&spi->reply_received)) {
		pr_info("spi_read: Reiniting\n");
		reinit_completion(&spi->reply_received);
	}

	timeout = wait_for_completion_timeout(&spi->reply_received, msecs_to_jiffies(500));
	if (!timeout) {
		dev_warn(&spi_dev->dev, "spi_read timeout\n");
		mutex_unlock(&spi->lock);
		return -ETIMEDOUT;
	}
	memcpy(xfer->rx_buf, spi->buf, xfer->len);

	mutex_unlock(&spi->lock);

	spi_finalize_current_transfer(ctrl);

	return err;
}

static int spi_reset_cb(struct platform_device *pdev, const u8 rxbuf[], int len)
{
	struct firmata_spi *spi = platform_get_drvdata(pdev);

	pr_info("%s: called\n", __func__);
	// There was a reset on the device, and SPI is now disabled. We need to resend
	// configuration data before the next transfer
	spi->spi_enabled = false;
	return 0;
}

struct spi_board_info firmata_chip = {
	.modalias = "spi-firmata",
	.max_speed_hz = 100000,
	.chip_select = 0, // Must be smaller than #CS of the controller
	.mode = SPI_MODE_0,
	.platform_data = NULL
};

static int firmata_spi_probe(struct platform_device *pdev)
{
	struct spi_controller *controller;
	struct device *dev = &pdev->dev;
	struct firmata_platform_data *pdata = dev_get_platdata(&pdev->dev);
	struct firmata_spi *spi;
	int err;


	pr_info("%s probing 1\n", __func__);
	controller = devm_spi_alloc_master(&pdev->dev, sizeof(*spi));
	if (!controller) {
		dev_info(&pdev->dev, "controller allocation failed\n");
		return -ENOMEM;
	}

	spi = spi_controller_get_devdata(controller);
	spi->controller = controller;
	spi->pdev = pdev;
	spi->spi_enabled = false;
	mutex_init(&spi->lock);
	init_completion(&spi->reply_received);

	spi->buf = devm_kmalloc(&pdev->dev, FIRMATA_SPI_BUF_SIZE, GFP_KERNEL);
	if (!spi->buf) {
		err = -ENOMEM;
		goto exit_free_master;
	}

	platform_set_drvdata(pdev, spi);

	controller->bus_num = -1;
	controller->flags = 0;
	controller->set_cs = firmata_set_cs;
	controller->transfer_one = firmata_transfer_one;
	controller->setup = firmata_setup;
	controller->num_chipselect = 1;
	controller->mode_bits = SPI_CPOL | SPI_CPHA;

	err = firmata_register_event_cb(pdev, SYSEX_ID | SPI_DATA, spi_reply_cb);
	if (err) {
		dev_err(dev, "failed to register event cb: %d\n", err);
		return err;
	}

	err = firmata_register_event_cb(pdev, RESET_CB, spi_reset_cb);
	err = devm_spi_register_controller(&pdev->dev, controller);
	if (err) {
		dev_err(dev, "Could not register SPI controller\n");
		return -ENODEV;
	}

	pr_info("%s: got bus_num %d\n", __func__, controller->bus_num);

	firmata_spi_device = spi_new_device(controller, &firmata_chip);
	if (firmata_spi_device) {
		dev_info(&firmata_spi_device->dev, "firmata_spi_dev at %s\n",
			dev_name(&firmata_spi_device->dev));
	} else {
		dev_warn(dev, "spi_new_device failed\n");
		err = -ENODEV;
		goto exit_free_master;
	}
	return 0;

exit_free_master:
	spi_master_put(controller);

	return err;
}

// TODO: void in 6.4
static int firmata_spi_remove(struct platform_device *pdev)
{
/*	struct spi_master *ctrl = platform_get_drvdata(pdev);
 *	pr_info("ctrl: %p\n", (void *)ctrl);
 *	struct firmata_spi *rs = spi_controller_get_devdata(ctrl);
 *	pr_info("ctrl: %p\n", (void *)ctrl);
 */
	struct firmata_spi *rs;
	struct spi_master *ctrl = platform_get_drvdata(pdev);

	pr_info("%s: pdev: %p, ctrl: %p\n", __func__, (void *)pdev, (void *)ctrl);
	firmata_unregister_event_cb(pdev, SYSEX_ID | SPI_DATA);
	return 0;  // TODO: void in 6.4
}


#ifdef CONFIG_PM
static int firmata_spi_suspend(struct device *dev)
{
	struct spi_master *ctrl = dev_get_drvdata(dev);
	struct firmata_spi *rs = spi_controller_get_devdata(ctrl);

	return 0;
}

static int firmata_spi_resume(struct device *pdev)
{
	return 0;
}

static SIMPLE_DEV_PM_OPS(firmata_pm, firmata_spi_suspend, firmata_spi_resume);
#define FIRMATA_SPI_PM_OPS	(&firmata_pm)
#else
#define FIRMATA_SPI_PM_OPS	NULL
#endif

static struct platform_driver spi_firmata_driver = {
	.driver = {
		.name	= "firmata-spi",
		.pm     = FIRMATA_SPI_PM_OPS,
	},
	.probe		= firmata_spi_probe,
	.remove		= firmata_spi_remove,  // TODO: remove_new in 6.4
};
module_platform_driver(spi_firmata_driver);

MODULE_DESCRIPTION("Driver for SPI master for Firmata firmware on Microcontrollers");
MODULE_AUTHOR("Birger Koblitz");
MODULE_LICENSE("GPL v2");
MODULE_VERSION("0.1");
MODULE_ALIAS("platform:firmata-spi");
