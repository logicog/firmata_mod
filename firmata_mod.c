// SPDX-License-Identifier: GPL-2.0
/* Driver for microcontrollers running Firmata firmware making
 * peripherial devices of the microntroller available through a UART
 * to the host
 * The code related to the mfd functionality is heavily based on the
 * dln2.c driver. The code related to using a ldisc for within a device
 * driver is based on the speakup code.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/mfd/core.h>
#include <linux/tty.h>
#include <linux/tty_driver.h>
#include <linux/completion.h>
#include "firmata.h"

#define FIRMATA_DRIVER	"firmata"
static char *firmata_port = "ttyUSB0";
static int baud_rate = 57600;

#define N_FIRMATA N_DEVELOPMENT

static DEFINE_MUTEX(firmata_tty_mutex);

static struct platform_device *pdevice;
struct tty_struct *firmata_tty;

struct firmata_priv {
	/* TTY buffers */
	u8 txbuf[FIRMATA_SIZE_TXBUF];
	u8 rxbuf[FIRMATA_SIZE_RXBUF];

	/* Per-Device lock */
	spinlock_t lock;

	/* TTY buffer accounting */
	struct work_struct tx_work;	/* Flushes TTY TX buffer */
	u8 *txhead;			/* Next TX byte */
	size_t txleft;			/* Bytes left to TX */
	int rxfill;			/* Bytes already RX'd in buffer */
	struct tty_struct *tty;

	struct list_head event_cb_list;
	spinlock_t event_cb_lock;

	struct completion capabilites_read;
	struct completion firmware_initialized;
	/* Some Arduino devices will reset the moment their USB<->UART
	 * converter is initialized. We need to wait until this is done,
	 * at which point these devices send a complete REPORT_FIRMWARE
	 * message
	 */
	bool boot_completed;
	int protocol_major;
	int protocol_minor;
	int npins;
	bool supports_i2c;
	bool supports_spi;
	int analog_res;
	u32 pin_caps[FIRMATA_MAX_PINS];
	struct gpio_chip gpio;
};

struct firmata_event_cb_entry {
	struct list_head list;
	u16 id;
	struct platform_device *pdev;
	firmata_event_cb_t callback;
};

int firmata_register_event_cb(struct platform_device *pdev, u16 id,
			   firmata_event_cb_t event_cb)
{
	struct firmata_priv *firmata = dev_get_drvdata(pdev->dev.parent);
	struct firmata_event_cb_entry *i, *entry;
	int ret = 0;

	pr_info("%s firmata: %p, 0x%04x", __func__, firmata, id);
	entry = kzalloc(sizeof(*entry), GFP_KERNEL);
	if (!entry)
		return -ENOMEM;

	entry->id = id;
	entry->callback = event_cb;
	entry->pdev = pdev;

	spin_lock(&firmata->event_cb_lock);

	list_for_each_entry(i, &firmata->event_cb_list, list) {
		if (i->id == id) {
			ret = -EBUSY;
			break;
		}
	}

	if (!ret)
		list_add_rcu(&entry->list, &firmata->event_cb_list);

	spin_unlock(&firmata->event_cb_lock);

	if (ret)
		kfree(entry);

	return ret;
}
EXPORT_SYMBOL(firmata_register_event_cb);

void firmata_unregister_event_cb(struct platform_device *pdev, u16 id)
{
	struct firmata_priv *firmata = dev_get_drvdata(pdev->dev.parent);
	struct firmata_event_cb_entry *i;
	bool found = false;

	spin_lock(&firmata->event_cb_lock);

	list_for_each_entry(i, &firmata->event_cb_list, list) {
		if (i->id == id) {
			list_del_rcu(&i->list);
			found = true;
			break;
		}
	}

	spin_unlock(&firmata->event_cb_lock);

	if (found) {
		synchronize_rcu();
		kfree(i);
	}
}
EXPORT_SYMBOL(firmata_unregister_event_cb);

/* Write out remaining transmit buffer.
 * Scheduled when TTY is writable.
 */
static void firmata_ldisc_tx_worker(struct work_struct *work)
{
	struct firmata_priv *firmata = container_of(work, struct firmata_priv, tx_work);
	ssize_t written;

	spin_lock_bh(&firmata->lock);

	if (firmata->txleft) {
		written = firmata->tty->ops->write(firmata->tty, firmata->txhead,
						   firmata->txleft);
		if (written < 0) {
			dev_err(firmata->tty->dev, "Failed to write to tty %s.\n",
				firmata->tty->name);

			spin_unlock_bh(&firmata->lock);
			return;
		}

		firmata->txleft -= written;
		firmata->txhead += written;
	}

	if (!firmata->txleft)
		clear_bit(TTY_DO_WRITE_WAKEUP, &firmata->tty->flags);

	spin_unlock_bh(&firmata->lock);
}

static int firmata_ldisc_open(struct tty_struct *tty)
{
	struct firmata_priv *firmata;

	// TODO: Check for a uart tty, here

	if (!tty->ops->write)
		return -EOPNOTSUPP;

	firmata = kzalloc(sizeof(*firmata), GFP_KERNEL);
	if (!firmata)
		return -ENOMEM;

	tty->disc_data = firmata;
	firmata->tty = tty;

	spin_lock_init(&firmata->lock);
	INIT_WORK(&firmata->tx_work, firmata_ldisc_tx_worker);

	INIT_LIST_HEAD(&firmata->event_cb_list);

	init_completion(&firmata->firmware_initialized);
	init_completion(&firmata->capabilites_read);

	return 0;
}

static void firmata_ldisc_close(struct tty_struct *tty)
{
	struct firmata_priv *firmata = tty->disc_data;

	pr_info("In %s\n", __func__);
	// TODO: Here we need to shut down all revices on the busses of this channel
	/* Give UART one final chance to flush.
	 * No need to clear TTY_DO_WRITE_WAKEUP since .write_wakeup() is
	 * serialised against .close() and will not be called once we return.
	 */
	flush_work(&firmata->tx_work);

	/* Mark channel as dead */
	spin_lock_bh(&firmata->lock);
	kfree(tty->disc_data);
	tty->disc_data = NULL;
	spin_unlock_bh(&firmata->lock);
}

/* Called by the driver when there's room for more data. */
static void firmata_ldisc_tx_wakeup(struct tty_struct *tty)
{
	struct firmata_priv *firmata = tty->disc_data;

	pr_info("In %s\n", __func__);
	schedule_work(&firmata->tx_work);
}

static void drop_bytes(struct firmata_priv *firmata, size_t i)
{
	lockdep_assert_held(&firmata->lock);

	memmove(&firmata->rxbuf[0], &firmata->rxbuf[i], FIRMATA_SIZE_RXBUF - i);
	firmata->rxfill -= i;
	if (firmata->rxfill < 0)
		firmata->rxfill = 0;
}

static int serial_send(struct tty_struct *tty, uint8_t *buf, int n)
{
	struct firmata_priv *firmata = tty->disc_data;
	int written, i;

	pr_info("%s: sending %d >", __func__, n);
	for (i = 0; i < n; i++)
		pr_cont(" %02x", buf[i]);
	pr_cont("<\n");

	lockdep_assert_held(&firmata->lock);

	set_bit(TTY_DO_WRITE_WAKEUP, &tty->flags);
	written = tty->ops->write(tty, buf, n);  // TODO: return value? can327_send
	if (written < 0) {
		pr_err("%s: error writing %d", __func__, written); // TODO: dev_err, see above?
		return written;
	}

	pr_info("%s: sent %d\n", __func__, written);
	return 0;
}

int firmata_serial_tx(struct platform_device *pdev, char *buf, int len)
{
	struct firmata_priv *firmata = dev_get_drvdata(pdev->dev.parent);
	int ret;

	spin_lock_bh(&firmata->lock);
	ret = serial_send(firmata->tty, buf, len);
	spin_unlock_bh(&firmata->lock);
	return ret;
}
EXPORT_SYMBOL(firmata_serial_tx);

static int run_event_callbacks(struct firmata_priv *firmata, u16 id, char data[], int len)
{
	struct firmata_event_cb_entry *i;
	int bytes_read = 0;

	lockdep_assert_held(&firmata->lock);
	rcu_read_lock();

	list_for_each_entry_rcu(i, &firmata->event_cb_list, list) {
		if (i->id == id) {
			bytes_read = i->callback(i->pdev, data, len);
			break;
		}
	}

	rcu_read_unlock();
	return bytes_read;
}

// FIXME: This needs to be aligned with the message sizes in spi/i2c and the overall buffer
#define MSG_BUFFER_SIZE 500
static void parse_sysex(struct firmata_priv *firmata, int len)
{
	int i = 1;
	uint8_t buf[MSG_BUFFER_SIZE];
	uint16_t sysex_code;

	sysex_code = firmata->rxbuf[i];
	switch (sysex_code) {
	case REPORT_FIRMWARE:
		pr_info("Found REPORT_FIRMWARE, len %d\n", len);
		// Is this a reset on the device?
		if (firmata->boot_completed) {
			run_event_callbacks(firmata, RESET_CB, firmata->rxbuf + i, len);
			break;
		}
		if (len >= 4) {
			firmata->protocol_major = firmata->rxbuf[i + 1];
			firmata->protocol_minor = firmata->rxbuf[i + 2];
		} else {
			return;
		}
		i = 4;
		while ((firmata->rxbuf[i] != END_SYSEX) && (i / 2 < MSG_BUFFER_SIZE)) {
			buf[i / 2 - 2] = (firmata->rxbuf[i] & 0x7F)
						| ((firmata->rxbuf[i+1] & 0x7F) << 7);
			i += 2;
		}
		buf[i / 2 - 2] = '\0';
		pr_info("FIRMATA protocol version %c.%c, version string %s\n",
			firmata->protocol_major + '0', firmata->protocol_minor + '0', buf);
		firmata->boot_completed = true;
		complete(&firmata->firmware_initialized);
		break;

	case CAPABILITY_RESPONSE:
		int pin = 0;

		i = 2;
		while ((firmata->rxbuf[i] != END_SYSEX)) {
			if (firmata->rxbuf[i] == MODE_IGNORE) {
				pin++;
				i++;
				continue;
			}
			if (firmata->rxbuf[i] < 0x1f) {
				firmata->pin_caps[pin] |= BIT(firmata->rxbuf[i]);
				if (firmata->rxbuf[i] == MODE_ANALOG)
					firmata->analog_res = buf[i+1];
				if (firmata->rxbuf[i] == MODE_I2C)
					firmata->supports_i2c = true;
				if (firmata->rxbuf[i] == MODE_SPI)
					firmata->supports_spi = true;
				i += 2;
			}
		}
		firmata->npins = pin;
		complete(&firmata->capabilites_read);
		break;

	case STRING_MESSAGE:
		i = 2;
		while ((firmata->rxbuf[i] != END_SYSEX) && (i / 2 < MSG_BUFFER_SIZE)) {
			buf[i / 2 - 1] = (firmata->rxbuf[i] & 0x7F)
						| ((firmata->rxbuf[i+1] & 0x7F) << 7);
			i += 2;
		}
		buf[i / 2 - 1] = '\0';
		pr_info("Firmata: STRING_MESSAGE %s\n", buf);
		break;

	case END_SYSEX:
		// Empty message, ignore
		break;

	default:
		run_event_callbacks(firmata, SYSEX_ID | sysex_code, firmata->rxbuf + i, len);
	}
}

/*
 * The function must only be called if there is at least one valid response byte in rxbuf
 * returns 0 if no more messages in buffer
 * returns 1 if there is possibly another message in the buffer
 */
static int parse_rxbuf(struct firmata_priv *firmata)
{
	size_t pos = 0;

	pr_info("%s: fill %d, Message %02x %02x %02x\n", __func__, firmata->rxfill,
		firmata->rxbuf[pos], firmata->rxbuf[pos + 1], firmata->rxbuf[pos + 2]);
	lockdep_assert_held(&firmata->lock);
	// We have at least one byte and it is always the start of a message

	if (firmata->rxbuf[pos] == START_SYSEX) {
		pr_info("%s: Found START_SYSEX\n", __func__);
		for (pos = 0; pos < firmata->rxfill; pos++) {
			if (firmata->rxbuf[pos] ==  END_SYSEX)
				break;
		}
		// Message is longer than buffer, assume some transmission problem and drop everything
		if (pos >= FIRMATA_SIZE_RXBUF) {
			drop_bytes(firmata, pos);
			return 0;
		}
		if (firmata->rxbuf[pos] ==  END_SYSEX) {
			parse_sysex(firmata, pos);
			drop_bytes(firmata, pos + 1);
			return 1;
		}
		// We do not have all of the message yet and have space for more
		return 0;
	} else if (firmata->rxbuf[pos] == REPORT_VERSION) {
		firmata->protocol_major = firmata->rxbuf[pos + 1];
		firmata->protocol_minor = firmata->rxbuf[pos + 2];
		drop_bytes(firmata, 3);
		return 1;
	} else if ((firmata->rxbuf[pos] & DIGITAL_IO_MESSAGE) == DIGITAL_IO_MESSAGE) {
		pr_info("Found DIGITAL_IO_MESSAGE\n");
		if (pos + 2 >= firmata->rxfill)
			return 0;

		run_event_callbacks(firmata, FIRMATA_GPIO_EVENT, firmata->rxbuf + pos, 3);
		drop_bytes(firmata, 3);
		return 1;
	}

	dev_warn(firmata->tty->dev, "%s: Unknown message %02x\n",
		 __func__, firmata->rxbuf[pos]);
	drop_bytes(firmata, 1);
	return 1;
}

/* Handle incoming data from the Firmata device
 * This will not be re-entered while running, but other ldisc
 * functions may be called in parallel.
 */
static int firmata_ldisc_rx(struct tty_struct *tty, const unsigned char *cp,
			    const char *fp, int count)
{
	struct firmata_priv *firmata = tty->disc_data;
	int received = count;

	pr_info("In %s, received %d\n", __func__, received);
	spin_lock_bh(&firmata->lock);

	pr_info("RX: ");
	while (received-- && firmata->rxfill < FIRMATA_SIZE_RXBUF) {
		if (fp && *fp++) {
			dev_err(firmata->tty->dev,
				"Error in received character stream. Check your wiring.");

			spin_unlock_bh(&firmata->lock);
			return -EIO;
		}

		firmata->rxbuf[firmata->rxfill++] = *cp;
		pr_cont(" %02x", *cp);
		cp++;
	}
	pr_cont(", RX end\n");

	if (received >= 0) {
		dev_err(firmata->tty->dev,
			"Receive buffer overflowed. Bad chip or wiring? received = %i", received);

		spin_unlock_bh(&firmata->lock);
		return -ENOMEM;
	}

	while (firmata->rxfill) {
		if (!parse_rxbuf(firmata))
			break;
	}

	spin_unlock_bh(&firmata->lock);

	return count;
}

static void firmata_read_caps(struct firmata_priv *firmata)
{
	uint8_t get_caps[3];
	int timeout;

	pr_info("%s: called\n", __func__);
	get_caps[0] = START_SYSEX;
	get_caps[1] = CAPABILITY_QUERY;
	get_caps[2] = END_SYSEX;

	spin_lock_bh(&firmata->lock);
	serial_send(firmata->tty, get_caps, 3);
	spin_unlock_bh(&firmata->lock);

	timeout = wait_for_completion_timeout(&firmata->capabilites_read, msecs_to_jiffies(50));
	if (!timeout)
		dev_warn(firmata->tty->dev, "%s: timeout!\n", __func__);
}

static struct tty_ldisc_ops firmata_ldisc_ops = {
//	.owner		= THIS_MODULE TODO: For now, so we can test, need to split modules later
	.num		= N_FIRMATA,
	.name		= "firmata_ldisc",
	.open		= firmata_ldisc_open,
	.close		= firmata_ldisc_close,
	.receive_buf2	= firmata_ldisc_rx,
	.write_wakeup	= firmata_ldisc_tx_wakeup,
};

static void set_uart_config(struct tty_struct *uart, int baudrate)
{
	struct ktermios new_termios;

	down_read(&uart->termios_rwsem);
	new_termios = uart->termios;
	up_read(&uart->termios_rwsem);
	tty_termios_encode_baud_rate(&new_termios, baudrate, baudrate);
	pr_info("%s termios c_cflags %x c_iflag %x\n", __func__, new_termios.c_cflag, new_termios.c_iflag);
	// Ignore input errors and parity errors
	new_termios.c_iflag = IGNBRK | IGNPAR;
//	TODO: see https://linux.die.net/man/3/termios
//	TODO: understand flags by enabling them one by one: new_termios.c_cflag = CS8 | CREAD | HUPCL | CLOCAL;

	// Make sure HW flow control is off
	new_termios.c_cflag &= ~CRTSCTS;
	tty_set_termios(uart, &new_termios);

	clear_bit(TTY_HUPPED, &uart->flags);
}


static struct tty_struct *firmata_tty_init(struct device *dev)
{
	struct tty_struct *tty;
	dev_t devno;
	int err;

	err = tty_dev_name_to_number(firmata_port, &devno);
	if (err < 0) {
		// A device with this name might appear later, keep retrying
		return ERR_PTR(-EAGAIN);
	}
	pr_info("Probing %s, devno is %x\n", __FILE__, devno);
	tty = tty_kopen_exclusive(devno);
	if (IS_ERR(tty) || !tty) {
		dev_err(dev, "could not open TTY\n");
		return tty;
	}

	if (tty->ops->open)
		err = tty->ops->open(tty, NULL);
	else
		err = -ENODEV;

	if (err) {
		pr_info("FIRMATA: opening TTY %s failed\n", firmata_port);
		tty_unlock(tty);
		return ERR_PTR(err);
	}

	set_uart_config(tty, baud_rate);
	pr_info("TTY Baud Rates is %d\n", cpu_to_le32(tty_get_baud_rate(tty)));

	tty_unlock(tty);

	mutex_lock(&firmata_tty_mutex);
	err = tty_set_ldisc(tty, N_FIRMATA);
	mutex_unlock(&firmata_tty_mutex);
	printk(KERN_INFO "ldisk ptr %p now", (void *)(tty->ldisc));

	pr_info("Setting dev drvdata to firmata line discipline, firmata pointer %p\n", (void *)(tty->disc_data));
	dev_set_drvdata(dev, tty->disc_data);

	if (!err) {
		/* Success */
		return tty;
	}

	pr_err("firmata: Failed to set N_FIRMATA on tty\n");

	tty_lock(tty);
	if (tty->ops->close)
		tty->ops->close(tty, NULL);
	tty_unlock(tty);

	tty_kclose(tty);

	return ERR_PTR(err);
}

enum {
	FIRMATA_ACPI_MATCH_ADC		= 0,
	FIRMATA_ACPI_MATCH_I2C		= 1,
	FIRMATA_ACPI_MATCH_SPI		= 2,
	FIRMATA_ACPI_MATCH_PINCTRL	= 3,
};

/* Only one SPI port supported */
static struct firmata_platform_data firmata_pdata_spi = {
	.port = 0,
	.host_controls_cs = false,
};

static struct mfd_cell_acpi_match firmata_acpi_match_spi = {
	.adr = FIRMATA_ACPI_MATCH_SPI,
};

/* Only one I2C port supported */
static struct firmata_platform_data firmata_pdata_i2c = {
	.port = 0,
};

static struct mfd_cell_acpi_match firmata_acpi_match_i2c = {
	.adr = FIRMATA_ACPI_MATCH_I2C,
};

static struct firmata_platform_data firmata_pdata_pinctrl = {
	.port = 0,
};

static struct mfd_cell_acpi_match firmata_acpi_match_pinctrl = {
	.adr = FIRMATA_ACPI_MATCH_PINCTRL,
};

static const struct mfd_cell firmata_privs[] = {
	{
		.name = "firmata-i2c",
		.acpi_match = &firmata_acpi_match_i2c,
		.platform_data = &firmata_pdata_i2c,
		.pdata_size = sizeof(struct firmata_platform_data),
	},
	{
		.name = "firmata-spi",
		.acpi_match = &firmata_acpi_match_spi,
		.platform_data = &firmata_pdata_spi,
		.pdata_size = sizeof(struct firmata_platform_data),
	},
	{
		.name = "pinctrl-firmata",
		.acpi_match = &firmata_acpi_match_pinctrl,
		.platform_data = &firmata_pdata_pinctrl,
		.pdata_size = sizeof(struct firmata_platform_data),
	},
};

static int firmata_probe(struct platform_device *pdev)
{
	struct firmata_priv *firmata;
	int err = 0;
	struct tty_struct *tty;
	uint8_t get_firmware_version[3] = {START_SYSEX, REPORT_FIRMWARE, END_SYSEX};
	int timeout;

	dev_info(&pdev->dev, "Probing %s\n", __FILE__);
	tty = firmata_tty_init(&pdev->dev);
	if (IS_ERR(tty))
		return PTR_ERR(tty);

	firmata = tty->disc_data;

	spin_lock_bh(&firmata->lock);
	serial_send(tty, get_firmware_version, 3); // TODO: This needs to come from the driver data

	spin_unlock_bh(&firmata->lock);

	firmata_tty = tty;

	timeout = wait_for_completion_timeout(&firmata->firmware_initialized,
					      msecs_to_jiffies(10000));
	if (!timeout) {
		dev_err(&pdev->dev, "timeout waiting for firmware initialization\n");
		return -ENODEV;
	}

	firmata_read_caps(firmata);
	firmata_pdata_pinctrl.npins = firmata->npins;
	firmata_pdata_pinctrl.pin_caps = firmata->pin_caps;

	err = mfd_add_hotplug_devices(&pdev->dev, firmata_privs, ARRAY_SIZE(firmata_privs));
	if (err)
		dev_err(&pdev->dev, "failed to add mfd devices to core\n");
	return err;
}

#ifdef CONFIG_PM_SLEEP
static int firmata_suspend(struct device *pdev)
{
	return 0;
}

static int firmata_resume(struct device *pdev)
{
	return 0;
}

static SIMPLE_DEV_PM_OPS(firmata_pm, firmata_suspend, firmata_resume);
#define FIRMATA_PM_OPS	(&firmata_pm)
#else
#define FIRMATA_PM_OPS	NULL
#endif

static struct platform_driver firmata_driver = {
	.probe		= firmata_probe,
	.driver		= {
		.name	= FIRMATA_DRIVER,
		.pm	= FIRMATA_PM_OPS,
	},
};

static int __init firmata_init(void)
{
	int err = 0;

	pr_info("%s: Loading firmata module...\n", __func__);

	err = tty_register_ldisc(&firmata_ldisc_ops);
	if (err)
		pr_err("%s, Can't register line discipline\n", __func__);

	err = platform_driver_register(&firmata_driver);
	if (err < 0)
		return err;

	pdevice = platform_device_register_simple(FIRMATA_DRIVER, 0, NULL, 0);
	if (IS_ERR(pdevice)) {
		pr_err("failed registering %s: %ld\n", FIRMATA_DRIVER, PTR_ERR(pdevice));
		return -ENODEV;
	}

	pr_info("%s: Firmata module inserted, tty port %s\n", __func__, firmata_port);

	return 0;
}

static void __exit firmata_exit(void)
{
	struct tty_struct *tty = firmata_tty;

	// TODO: Instead of using the global firmata_tty, we should make use only of getting the LDISC
	// and the go for the firmata structure
	pr_info("%s: called, removing module\n", __func__);

	if (tty) {
		tty_lock(tty);
		if (tty->ops->close)
			tty->ops->close(tty, NULL);
		tty_ldisc_flush(tty);
		tty_unlock(tty);
		tty_kclose(tty);
		pr_info("%s: Closed tty!\n", __func__);
	}

	firmata_tty = NULL;
	platform_device_unregister(pdevice);
	platform_driver_unregister(&firmata_driver);

	/* This will only be called when all channels have been closed by
	 * userspace - tty_ldisc.c takes care of the module's refcount.
	 */
	tty_unregister_ldisc(&firmata_ldisc_ops);
}

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Birger Koblitz");
MODULE_DESCRIPTION("Driver for Arduino Devices running Firmata Firmware");
MODULE_VERSION("0.1");
MODULE_ALIAS_LDISC(N_FIRMATA);

module_param(firmata_port, charp, 0);
module_param(baud_rate, int, 0);
MODULE_PARM_DESC(firmata_port, "TTY Interface Device");
MODULE_PARM_DESC(baud_rate, "Baud Rate");

module_init(firmata_init);
module_exit(firmata_exit);
