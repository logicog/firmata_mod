// SPDX-License-Identifier: GPL-2.0
/* GPIO Driver for microcontrollers running Firmata firmware making
 * the GPIO's of the microntroller available through a UART
 * to the host
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/gpio/driver.h>
#include <linux/bitfield.h>
#include "firmata.h"

#define N_GPIO 20
#define FIRMATA_GPIO_MASK GENMASK(N_GPIO, 0)
#define PORT_WIDTH 8
#define PORT_PIN_MASK GENMASK(PORT_WIDTH, 0)
#define FIRMATA_DRIVER	"firmata-gpio"
#define DIGITAL_INPUT	1
#define DIGITAL_OUTPUT	2

struct firmata_gpio {
	struct gpio_chip gc;
	struct platform_device *pdev;

	/* Allow only one R/W transaction at a time */
	struct mutex lock;
	struct completion pin_state_read;
	struct completion pin_value_read;
	uint32_t pin_directions;
	uint16_t reported_ports;
	unsigned long requested_reports;
	uint32_t pin_values;
};

static int request_pin_report(struct firmata_gpio *gpio, u32 pin_mask)
{
	uint8_t buf[2];
	uint16_t required_ports = 0;
	int timeout, i;

	// Check whether the pin value is already reported on as it is part of a reported port
	for (i = 0; i < 4; i++) {
		if (pin_mask & PORT_PIN_MASK)
			required_ports |= BIT(i);
		pin_mask >>= PORT_WIDTH;
	}
	pr_info("request_pin_report: pin_mask 0x%08x, required_ports: %04x, reported_ports %04x\n",
		pin_mask, required_ports, gpio->reported_ports);
	if (!(required_ports & ~gpio->reported_ports))
		return 0;

	/* We request a report on the value of the input pin's port
	 * and wait for the value to be reported back before returning.
	 * We will hence-force receive automatically every time an update if
	 * the pin-value changed and can report back the cached value
	 * in firmata_gpio_get/multiple_get */

	mutex_lock(&gpio->lock);
	gpio->requested_reports = required_ports & ~gpio->reported_ports;
	for_each_set_bit(i, &gpio->requested_reports, N_GPIO / PORT_WIDTH) {
		buf[0] = REPORT_DIGITAL | i;
		buf[1] = 1;

		firmata_serial_tx(gpio->pdev, buf, 2);
		if (completion_done(&gpio->pin_value_read)) {
			pr_info("firmata_gpio_get: Reiniting\n");
			reinit_completion(&gpio->pin_value_read);
		}
	}
	pr_info("Requested ports: %lu\n", gpio->requested_reports);
	timeout = wait_for_completion_timeout(&gpio->pin_value_read, msecs_to_jiffies(10));

	pr_info("Requested ports now: %lu\n", gpio->requested_reports);
	mutex_unlock(&gpio->lock);

	if (!timeout) {
		pr_info("firmata_gpio_get timeout\n");
		return -ETIMEDOUT;
	}

	return 0;
}

static void disable_port_report(struct firmata_gpio *gpio, int port)
{
	uint8_t buf[2];

	buf[0] = REPORT_DIGITAL | port;
	buf[1] = 0;

	pr_info("disable_port_report port %d\n", port);
	firmata_serial_tx(gpio->pdev, buf, 2);
	gpio->reported_ports &= ~BIT(port);
}

static int firmata_gpio_direction_output(struct gpio_chip *gc, unsigned int off, int val)
{
	struct firmata_gpio *gpio = gpiochip_get_data(gc);
	uint8_t buf[6];
	int port_num;
	uint8_t port_val;
	uint32_t pin_mask = FIRMATA_GPIO_MASK;

	gpio->pin_directions |= BIT(off);
	port_num = off / PORT_WIDTH;
	if ((gpio->reported_ports & BIT(port_num))
	    && (gpio->pin_directions & (pin_mask << port_num)))
		disable_port_report(gpio, port_num);

	if (val)
		gpio->pin_values |= BIT(off);
	else
		gpio->pin_values &= ~BIT(off);

	pr_info("firmata_gpio_direction_output: pin %d, value %d", off, val);
	buf[0] = SET_PIN_MODE_MESSAGE;
	buf[1] = off;
	buf[2] = MODE_OUTPUT;

	port_val = gpio->pin_values >> (8 * port_num);
	buf[3] = DIGITAL_IO_MESSAGE | port_num;
	buf[4] = port_val & 0x7F;
	buf[5] = (port_val >> 7) & 0x7F;

	return firmata_serial_tx(gpio->pdev, buf, 6);
}

static int firmata_gpio_direction_input(struct gpio_chip *gc, unsigned int off)
{
	struct firmata_gpio *gpio = gpiochip_get_data(gc);
	uint8_t buf[3];

	pr_info("firmata_gpio_direction_input: pin %d", off);
	gpio->pin_directions &= ~BIT(off);

	buf[0] = SET_PIN_MODE_MESSAGE;
	buf[1] = off;
	buf[2] = MODE_INPUT;
	firmata_serial_tx(gpio->pdev, buf, 3);

	return 0;
}

static void firmata_gpio_set(struct gpio_chip *gc, unsigned int off, int val)
{
	struct firmata_gpio *gpio = gpiochip_get_data(gc);
	int port_num;
	uint8_t port_val;
	uint8_t buf[3];

	pr_info("firmata_gpio_set: pin %d, value %d", off, val);
	if (val)
		gpio->pin_values |= BIT(off);
	else
		gpio->pin_values &= ~BIT(off);

	port_num = off / 8;
	port_val = gpio->pin_values >> (8 * port_num);

	buf[0] = DIGITAL_IO_MESSAGE | port_num;
	buf[1] = port_val & 0x7F;
	buf[2] = (port_val >> 7) & 0x7F;

	firmata_serial_tx(gpio->pdev, buf, 3);
}


static void firmata_gpio_set_multiple(struct gpio_chip *gc,
				      unsigned long *maskp, unsigned long *bitsp)
{
	struct firmata_gpio *gpio = gpiochip_get_data(gc);
	uint32_t mask, val;
	int port_num;
	uint8_t port_val;
	uint8_t buf[3];

	mask = FIELD_GET(FIRMATA_GPIO_MASK, maskp[0]);
	val = FIELD_GET(FIRMATA_GPIO_MASK, bitsp[0]);

	pr_info("firmata_gpio_set_multiple: maskp %08lx, bitsp %08lx, mask %08x, val %08x",
		maskp[0], bitsp[0], mask, val);

	if (!mask)
		return;

	gpio->pin_values &= mask;
	gpio->pin_values |= val;

	for (port_num = 0; port_num < 3; port_num++){
		if (!((mask >> (8 * port_num)) & 0xff))
			continue;
		port_val = val >> (8 * port_num);
		port_val &= mask >> (8 * port_num);

		buf[0] = DIGITAL_IO_MESSAGE | port_num;
		buf[1] = port_val & 0x7F;
		buf[2] = (port_val >> 7) & 0x7F;

		firmata_serial_tx(gpio->pdev, buf, 3);
	}
}

static int firmata_gpio_get(struct gpio_chip *gc, unsigned int offset)
{
	struct firmata_gpio *gpio = gpiochip_get_data(gc);
	int v;

	pr_info("firmata_gpio_get called, cached pin-values 0x%08x, offset %d\n", gpio->pin_values, offset);
	request_pin_report(gpio, BIT(offset));
	v = gpio->pin_values & BIT(offset);
	pr_info("firmata_gpio_get: value %d -> %d", v, !!v);

	return !!v;
}

static int firmata_gpio_get_multiple(struct gpio_chip *gc,
				     unsigned long *maskp, unsigned long *bitsp)
{
	struct firmata_gpio *gpio = gpiochip_get_data(gc);
	uint32_t v;

	pr_info("firmata_gpio_get_multiple called, mask\n");
	request_pin_report(gpio, maskp[0]);
	v = gpio->pin_values & maskp[0];
	bitsp[0] = v;
	return 0;
}

static int firmata_gpio_get_direction(struct gpio_chip *gc, unsigned int offset)
{
	struct firmata_gpio *gpio = gpiochip_get_data(gc);

	pr_info("firmata_gpio_get_direction called, cached pin-direction 0x%08x, offset %d\n", gpio->pin_directions, offset);
	// The pin direction is always cached
	return gpio->pin_directions & BIT(offset);
}


static int gpio_pin_value_cb(struct platform_device *pdev, const u8 rxbuf[], int len)
{
	struct firmata_gpio *gpio = platform_get_drvdata(pdev);
	int port = rxbuf[0] & 0x03;
	uint32_t port_val;

	pr_info("gpio_pin_value_cb: command 0x%02x 0x%02x 0x%02x\n", rxbuf[0], rxbuf[1], rxbuf[2]);
	port_val = rxbuf[1] | (rxbuf[2] << 7);
	port_val &= 0xff;
	gpio->pin_values &= ~(0xff << (port * 8));
	gpio->pin_values |= port_val << (port * 8);
	gpio->requested_reports &= ~BIT(port);
	if(mutex_is_locked(&gpio->lock) && !gpio->requested_reports)
		complete(&gpio->pin_value_read);
	pr_info("gpio_pin_value_cb: complete\n");
	return 3;
}

static int gpio_pin_state_cb(struct platform_device *pdev, const u8 rxbuf[], int len)
{
	struct firmata_gpio *gpio = platform_get_drvdata(pdev);

	pr_info("gpio_pin_state_cb: command 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x, len %d\n",
		rxbuf[0], rxbuf[1], rxbuf[2], rxbuf[3], rxbuf[4], len);
	if (rxbuf[0] != PIN_STATE_RESPONSE) {
		pr_err("gpio_pin_state_cb: expected PIN_STATE_RESPONSE message, got message code 0x%02x\n", rxbuf[0]);
		return -1;
	}

	if (rxbuf[2] == DIGITAL_OUTPUT)
		gpio->pin_directions |= BIT(rxbuf[1]);

	complete(&gpio->pin_state_read);
	return len;
}


static void firmata_gpio_get_pin_state(struct firmata_gpio *gpio, u8 pin)
{
	uint8_t buf[4];
	int timeout;

	pr_info("firmata_gpio_get_pin_state called, pin %d\n", pin);
	buf[0] = START_SYSEX;
	buf[1] = PIN_STATE_QUERY;
	buf[2] = pin;
	buf[3] = END_SYSEX;

	mutex_lock(&gpio->lock);

	if (completion_done(&gpio->pin_state_read)) {
		pr_info("firmata_gpio_get_pin_state: Reiniting\n");
		reinit_completion(&gpio->pin_state_read);
	}

	firmata_serial_tx(gpio->pdev, buf, 4);

	timeout = wait_for_completion_timeout(&gpio->pin_state_read, msecs_to_jiffies(50));
	if (!timeout)
		pr_info("firmata_gpio_get_pin_state timeout, pin %d\n", pin);

	mutex_unlock(&gpio->lock);
}

static int firmata_gpio_probe(struct platform_device *pdev)
{
	struct firmata_gpio *gpio;
// FIXME:	struct firmata_platform_data *pdata = dev_get_platdata(&pdev->dev);
	struct device *dev = &pdev->dev;
	int err;
	int i;

	printk(KERN_INFO "Probing %s firmata_gpio_probe 1 \n", __FILE__);
	gpio = devm_kzalloc(&pdev->dev, sizeof(*gpio), GFP_KERNEL);
	if (gpio == NULL) {
		dev_info(&pdev->dev, "gpio chip allocation failed\n");
		return -ENOMEM;
	}

	gpio->pdev = pdev;

	gpio->gc.label = FIRMATA_DRIVER;
	gpio->gc.parent = dev;
	gpio->gc.owner = THIS_MODULE;
	gpio->gc.get_direction = firmata_gpio_get_direction;
	gpio->gc.direction_output = firmata_gpio_direction_output;
	gpio->gc.direction_input = firmata_gpio_direction_input;
	gpio->gc.get = firmata_gpio_get;
	gpio->gc.get_multiple = firmata_gpio_get_multiple;
	gpio->gc.set = firmata_gpio_set;
	gpio->gc.set_multiple = firmata_gpio_set_multiple;
	gpio->gc.base = -1;
	gpio->gc.ngpio = 20;  // TODO: Set this from platform data?
	gpio->gc.can_sleep = true;

	mutex_init(&gpio->lock);
	init_completion(&gpio->pin_value_read);
	init_completion(&gpio->pin_state_read);

	platform_set_drvdata(pdev, gpio);
	pr_info("firmata_gpio_probe: firmata_gpio pointer %p", (void *) gpio);

	err = devm_gpiochip_add_data(dev, &gpio->gc, gpio);

	if (err)  {
		dev_err(dev, "Could not register GPIO controller\n");
		// TODO: unregister SPI
		return -ENODEV;
	}

	err = firmata_register_event_cb(pdev, FIRMATA_GPIO_EVENT, gpio_pin_value_cb);
	if (err) {
		dev_err(dev, "failed to register event cb: %d\n", err);
		return err;
	}
	err = firmata_register_event_cb(pdev, SYSEX_ID | PIN_STATE_RESPONSE, gpio_pin_state_cb);
	if (err) {
		dev_err(dev, "failed to register event cb: %d\n", err);
		return err;
	}

	// Read initial states
	for (i = 0; i < gpio->gc.ngpio; i++)
		firmata_gpio_get_pin_state(gpio, i);

	printk(KERN_INFO "Probing %s firmata_gpio_probe 2 \n", __FILE__);
	return 0;
}


static void firmata_gpio_remove(struct platform_device *pdev)
{
//	struct firmata_gpio *gpio = platform_get_drvdata(pdev);

	firmata_unregister_event_cb(pdev, FIRMATA_GPIO_EVENT);
	firmata_unregister_event_cb(pdev, SYSEX_ID | PIN_STATE_RESPONSE);
}


#ifdef CONFIG_PM
static int firmata_gpio_suspend(struct device *dev)
{
	return 0;
}

static int firmata_gpio_resume(struct device *pdev)
{
	return 0;
}

static SIMPLE_DEV_PM_OPS(firmata_pm, firmata_gpio_suspend, firmata_gpio_resume);
#define FIRMATA_SPI_PM_OPS	&firmata_pm
#else
#define FIRMATA_SPI_PM_OPS	NULL
#endif

static struct platform_driver gpio_firmata_driver = {
	.driver = {
		.name	= "firmata-gpio",
                .pm     = FIRMATA_SPI_PM_OPS,
	},
	.probe		= firmata_gpio_probe,
	.remove		= firmata_gpio_remove,
};
module_platform_driver(gpio_firmata_driver);

MODULE_DESCRIPTION("GPIO driver for Firmata firmware on Microcontrollers");
MODULE_AUTHOR("Birger Koblitz");
MODULE_LICENSE("GPL v2");
MODULE_VERSION("0.1");
MODULE_ALIAS("platform:firmata-gpio");
