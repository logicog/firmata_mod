// SPDX-License-Identifier: GPL-2.0
/* Pinctrl Driver for microcontrollers running Firmata firmware making
 * peripherial devices of the microntroller available through a UART
 * to the host
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/pinctrl/pinctrl.h>
#include <linux/pinctrl/pinmux.h>
#include <linux/pinctrl/pinconf.h>
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

struct firmata_pinctrl {
	struct gpio_chip gc;
	struct platform_device *pdev;
	struct pinctrl_dev *pctl;

	/* Allow only one R/W transaction at a time */
	struct mutex lock;
	struct completion pin_state_read;
	struct completion pin_value_read;

	u32 *pin_caps;
	uint32_t pin_directions;
	uint16_t reported_ports;
	unsigned long requested_reports;
	uint32_t pin_values;
};

const struct pinctrl_pin_desc firmata_pins[] = {
	PINCTRL_PIN(0, "D0"),
	PINCTRL_PIN(1, "D1"),
	PINCTRL_PIN(2, "D2"),
	PINCTRL_PIN(3, "D3"),
	PINCTRL_PIN(4, "D4"),
	PINCTRL_PIN(5, "D5"),
	PINCTRL_PIN(6, "D6"),
	PINCTRL_PIN(7, "D7"),
	PINCTRL_PIN(8, "D8"),
	PINCTRL_PIN(9, "D9"),
	PINCTRL_PIN(10, "D10"),
	PINCTRL_PIN(11, "D11"),
	PINCTRL_PIN(12, "D12"),
	PINCTRL_PIN(13, "D13"),
	PINCTRL_PIN(14, "D14"),
	PINCTRL_PIN(15, "D15"),
	PINCTRL_PIN(16, "D16"),
	PINCTRL_PIN(17, "D17"),
	PINCTRL_PIN(18, "D18"),
	PINCTRL_PIN(19, "D19"),
};

static const unsigned int spi0_pins[] = { 10, 11, 12, 13 };
static const unsigned int i2c0_pins[] = { 18, 19 };

struct firmata_pin_group {
	const char *name;
	const unsigned int *pins;
	unsigned int num_pins;
	struct pic32_desc_function *functions;
};

static const struct firmata_pin_group firmata_groups[] = {
	{
		.name = "spi0_grp",
		.pins = spi0_pins,
		.num_pins = ARRAY_SIZE(spi0_pins),
	},
	{
		.name = "i2c0_grp",
		.pins = i2c0_pins,
		.num_pins = ARRAY_SIZE(i2c0_pins),
	},
};


static int firmata_pin_config_get(struct pinctrl_dev *pctldev, unsigned int offset,
				  unsigned long *config)
{
	enum pin_config_param param = pinconf_to_config_param(*config);
	u16 arg = 0;

	pr_info("%s offset %d, %lx\n", __func__, offset, *config);
	switch (param) {
	case PIN_CONFIG_BIAS_DISABLE:
		return -EINVAL;
	case PIN_CONFIG_BIAS_PULL_DOWN:
		arg = 1;

		break;
	default:
		return -ENOTSUPP;
	}

	*config = pinconf_to_config_packed(param, arg);
	return 0;
}

static int firmata_pin_config_set(struct pinctrl_dev *pctldev, unsigned int line,
				  unsigned long *configs, unsigned int num_configs)
{
	struct firmata_pinctrl *pctrl = pinctrl_dev_get_drvdata(pctldev);
	unsigned int param, arg;
	int i, offset, ret = 0;
	uint8_t buf[3];

	offset = line-pctrl->gc.base;
	buf[0] = SET_PIN_MODE_MESSAGE;
	buf[1] = offset;

	pr_info("%s: line %d, %lx, num_configs %d\n", __func__, line, *configs, num_configs);
	for (i = 0; i < num_configs; i++) {
		param = pinconf_to_config_param(configs[i]);
		arg = pinconf_to_config_argument(configs[i]);

		pr_info("%s: offset %d, param %x, arg %d\n", __func__, offset, param, arg);
		switch (param) {
		case PIN_CONFIG_BIAS_DISABLE:
			pr_info("%s: PIN_CONFIG_BIAS_DISABLE\n", __func__);
			buf[2] = MODE_INPUT;
			break;
		case PIN_CONFIG_BIAS_PULL_UP:
			pr_info("%s: PIN_CONFIG_BIAS_PULL_UP\n", __func__);
			buf[2] = MODE_PULLUP;
			break;
		case PIN_CONFIG_BIAS_PULL_DOWN:
		default:
			dev_err(&pctrl->pdev->dev, "%s: pin-mode not supported, pin %d\n",
				__func__, offset);
			ret = -ENOTSUPP;
		}

		if (ret)
			break;
		firmata_serial_tx(pctrl->pdev, buf, 3);
	}
	return ret;
}

static int firmata_pin_config_group_get(struct pinctrl_dev *pctldev, unsigned int selector,
					unsigned long *config)
{
	pr_info("%s: called\n", __func__);
	return -ENOTSUPP;
}

static int firmata_pin_config_group_set(struct pinctrl_dev *pctldev,
					unsigned int group,
					unsigned long *configs,
					unsigned int num_configs)
{
	pr_info("%s: called\n", __func__);
	return -ENOTSUPP;
}

static const struct pinconf_ops firmata_pconf_ops = {
	.pin_config_get = firmata_pin_config_get,
	.pin_config_set = firmata_pin_config_set,
	.pin_config_group_get = firmata_pin_config_group_get,
	.pin_config_group_set = firmata_pin_config_group_set,
};

static int request_pin_report(struct firmata_pinctrl *pctrl, u32 pin_mask)
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
	pr_info("%s: pin_mask 0x%08x, required_ports: %04x, reported_ports %04x\n",
		__func__, pin_mask, required_ports, pctrl->reported_ports);
	if (!(required_ports & ~pctrl->reported_ports))
		return 0;

	/* We request a report on the value of the input pin's port
	 * and wait for the value to be reported back before returning.
	 * We will hence-force receive automatically every time an update if
	 * the pin-value changed and can report back the cached value
	 * in firmata_gpio_get/multiple_get
	 */

	mutex_lock(&pctrl->lock);
	pctrl->requested_reports = required_ports & ~pctrl->reported_ports;
	for_each_set_bit(i, &pctrl->requested_reports, N_GPIO / PORT_WIDTH) {
		buf[0] = REPORT_DIGITAL | i;
		buf[1] = 1;

		firmata_serial_tx(pctrl->pdev, buf, 2);
		if (completion_done(&pctrl->pin_value_read))
			reinit_completion(&pctrl->pin_value_read);
	}
	pr_info("Requested ports: %lu\n", pctrl->requested_reports);
	timeout = wait_for_completion_timeout(&pctrl->pin_value_read, msecs_to_jiffies(10));

	pr_info("Requested ports now: %lu\n", pctrl->requested_reports);
	mutex_unlock(&pctrl->lock);

	if (!timeout) {
		pr_info("firmata_gpio_get timeout\n");
		return -ETIMEDOUT;
	}

	return 0;
}

static void disable_port_report(struct firmata_pinctrl *pctrl, int port)
{
	uint8_t buf[2];

	buf[0] = REPORT_DIGITAL | port;
	buf[1] = 0;

	firmata_serial_tx(pctrl->pdev, buf, 2);
	pctrl->reported_ports &= ~BIT(port);
}

static int firmata_gpio_direction_output(struct gpio_chip *gc, unsigned int off, int val)
{
	struct firmata_pinctrl *pctrl = gpiochip_get_data(gc);
	uint8_t buf[6];
	int port_num;
	uint8_t port_val;
	uint32_t pin_mask = FIRMATA_GPIO_MASK;

	pctrl->pin_directions |= BIT(off);
	port_num = off / PORT_WIDTH;
	if ((pctrl->reported_ports & BIT(port_num))
	    && (pctrl->pin_directions & (pin_mask << port_num)))
		disable_port_report(pctrl, port_num);

	if (val)
		pctrl->pin_values |= BIT(off);
	else
		pctrl->pin_values &= ~BIT(off);

	pr_info("firmata_gpio_direction_output: pin %d, value %d", off, val);
	buf[0] = SET_PIN_MODE_MESSAGE;
	buf[1] = off;
	buf[2] = MODE_OUTPUT;

	port_val = pctrl->pin_values >> (8 * port_num);
	buf[3] = DIGITAL_IO_MESSAGE | port_num;
	buf[4] = port_val & 0x7F;
	buf[5] = (port_val >> 7) & 0x7F;

	return firmata_serial_tx(pctrl->pdev, buf, 6);
}

static int firmata_gpio_direction_input(struct gpio_chip *gc, unsigned int off)
{
	struct firmata_pinctrl *pctrl = gpiochip_get_data(gc);
	uint8_t buf[3];

	pr_info("%s: pin %d", __func__, off);
	pctrl->pin_directions &= ~BIT(off);

	buf[0] = SET_PIN_MODE_MESSAGE;
	buf[1] = off;
	buf[2] = MODE_INPUT;
	firmata_serial_tx(pctrl->pdev, buf, 3);

	return 0;
}

static void firmata_gpio_set(struct gpio_chip *gc, unsigned int off, int val)
{
	struct firmata_pinctrl *pctrl = gpiochip_get_data(gc);
	int port_num;
	uint8_t port_val;
	uint8_t buf[3];

	pr_info("%s: pin %d, value %d", __func__, off, val);
	if (val)
		pctrl->pin_values |= BIT(off);
	else
		pctrl->pin_values &= ~BIT(off);

	port_num = off / 8;
	port_val = pctrl->pin_values >> (8 * port_num);

	buf[0] = DIGITAL_IO_MESSAGE | port_num;
	buf[1] = port_val & 0x7F;
	buf[2] = (port_val >> 7) & 0x7F;

	firmata_serial_tx(pctrl->pdev, buf, 3);
}


static void firmata_gpio_set_multiple(struct gpio_chip *gc,
				      unsigned long *maskp, unsigned long *bitsp)
{
	struct firmata_pinctrl *pctrl = gpiochip_get_data(gc);
	uint32_t mask, val;
	int port_num;
	uint8_t port_val;
	uint8_t buf[3];

	mask = FIELD_GET(FIRMATA_GPIO_MASK, maskp[0]);
	val = FIELD_GET(FIRMATA_GPIO_MASK, bitsp[0]);

	pr_info("%s: maskp %08lx, bitsp %08lx, mask %08x, val %08x",
		__func__, maskp[0], bitsp[0], mask, val);

	if (!mask)
		return;

	pctrl->pin_values &= mask;
	pctrl->pin_values |= val;

	for (port_num = 0; port_num < 3; port_num++) {
		if (!((mask >> (8 * port_num)) & 0xff))
			continue;
		port_val = val >> (8 * port_num);
		port_val &= mask >> (8 * port_num);

		buf[0] = DIGITAL_IO_MESSAGE | port_num;
		buf[1] = port_val & 0x7F;
		buf[2] = (port_val >> 7) & 0x7F;

		firmata_serial_tx(pctrl->pdev, buf, 3);
	}
}

static int firmata_gpio_get(struct gpio_chip *gc, unsigned int offset)
{
	struct firmata_pinctrl *pctrl = gpiochip_get_data(gc);
	int v;

	pr_info("%s called, cached pin-values 0x%08x, offset %d\n", __func__, pctrl->pin_values, offset);
	request_pin_report(pctrl, BIT(offset));
	v = pctrl->pin_values & BIT(offset);

	return !!v;
}

static int firmata_gpio_get_multiple(struct gpio_chip *gc,
				     unsigned long *maskp, unsigned long *bitsp)
{
	struct firmata_pinctrl *pctrl = gpiochip_get_data(gc);
	uint32_t v;

	pr_info("%s called, mask\n", __func__);
	request_pin_report(pctrl, maskp[0]);
	v = pctrl->pin_values & maskp[0];
	bitsp[0] = v;
	return 0;
}

static int firmata_gpio_get_direction(struct gpio_chip *gc, unsigned int offset)
{
	struct firmata_pinctrl *pctrl = gpiochip_get_data(gc);

	pr_info("%s called, cached pin-direction 0x%08x, offset %d\n",
		__func__, pctrl->pin_directions, offset);
	// The pin direction is always cached after initial synchronization
	return pctrl->pin_directions & BIT(offset);
}


static int gpio_pin_value_cb(struct platform_device *pdev, const u8 rxbuf[], int len)
{
	struct firmata_pinctrl *pctrl = platform_get_drvdata(pdev);
	int port = rxbuf[0] & 0x03;
	uint32_t port_val;

	pr_info("%s: command 0x%02x 0x%02x 0x%02x\n", __func__, rxbuf[0], rxbuf[1], rxbuf[2]);
	port_val = rxbuf[1] | (rxbuf[2] << 7);
	port_val &= 0xff;
	pctrl->pin_values &= ~(0xff << (port * 8));
	pctrl->pin_values |= port_val << (port * 8);
	pctrl->requested_reports &= ~BIT(port);
	if (mutex_is_locked(&pctrl->lock) && !pctrl->requested_reports)
		complete(&pctrl->pin_value_read);
	return 3;
}

static int gpio_pin_state_cb(struct platform_device *pdev, const u8 rxbuf[], int len)
{
	struct firmata_pinctrl *pctrl = platform_get_drvdata(pdev);

	pr_info("%s: command 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x, len %d\n",
		__func__, rxbuf[0], rxbuf[1], rxbuf[2], rxbuf[3], rxbuf[4], len);
	if (rxbuf[0] != PIN_STATE_RESPONSE) {
		pr_err("%s: expected PIN_STATE_RESPONSE message, got message code 0x%02x\n",
		       __func__, rxbuf[0]);
		return -1;
	}

	if (rxbuf[2] == DIGITAL_OUTPUT)
		pctrl->pin_directions |= BIT(rxbuf[1]);

	complete(&pctrl->pin_state_read);
	return len;
}

static void firmata_gpio_get_pin_state(struct firmata_pinctrl *pctrl, u8 pin)
{
	uint8_t buf[4];
	int timeout;

	pr_info("%s called, pin %d\n", __func__, pin);
	buf[0] = START_SYSEX;
	buf[1] = PIN_STATE_QUERY;
	buf[2] = pin;
	buf[3] = END_SYSEX;

	mutex_lock(&pctrl->lock);

	if (completion_done(&pctrl->pin_state_read))
		reinit_completion(&pctrl->pin_state_read);

	firmata_serial_tx(pctrl->pdev, buf, 4);

	timeout = wait_for_completion_timeout(&pctrl->pin_state_read, msecs_to_jiffies(50));
	if (!timeout)
		dev_warn(&pctrl->pdev->dev, "%s timeout, pin %d\n", __func__, pin);

	mutex_unlock(&pctrl->lock);
}

static int firmata_get_groups_count(struct pinctrl_dev *pctldev)
{
	return ARRAY_SIZE(firmata_groups);
}

static const char *firmata_get_group_name(struct pinctrl_dev *pctldev, unsigned int selector)
{
	return firmata_groups[selector].name;
}

static int firmata_get_group_pins(struct pinctrl_dev *pctldev, unsigned int selector,
				  const unsigned int **pins, unsigned int *num_pins)
{
	*pins = (unsigned int *) firmata_groups[selector].pins;
	*num_pins = firmata_groups[selector].num_pins;
	return 0;
}

static const struct pinctrl_ops firmata_pctrl_ops = {
	.get_groups_count = firmata_get_groups_count,
	.get_group_name = firmata_get_group_name,
	.get_group_pins = firmata_get_group_pins,
};

static struct pinctrl_desc firmata_desc = {
	.name = "firmata",
	.pins = firmata_pins,
	.npins = ARRAY_SIZE(firmata_pins),
	.pctlops = &firmata_pctrl_ops,
	.confops = &firmata_pconf_ops,
	.owner = THIS_MODULE,
};

static struct pinctrl_gpio_range firmata_range = {
	.name = "chip a",
	.id = 0,
};

static int pinctrl_firmata_probe(struct platform_device *pdev)
{
	struct firmata_pinctrl *pctrl;
	struct firmata_platform_data *pdata = dev_get_platdata(&pdev->dev);
	struct device *dev = &pdev->dev;
	int err, i;

	pr_info("%s: called", __func__);
	pctrl = devm_kzalloc(dev, sizeof(*pctrl), GFP_KERNEL);
	if (!pctrl)
		return -ENOMEM;

	pctrl->pdev = pdev;
	err = devm_pinctrl_register_and_init(dev, &firmata_desc, pctrl, &pctrl->pctl);
	if (err)
		return err;

	platform_set_drvdata(pdev, pctrl);

	pctrl->pdev = pdev;
	pctrl->gc.label = FIRMATA_DRIVER;
	pctrl->gc.parent = dev;
	pctrl->gc.owner = THIS_MODULE;
	pctrl->gc.get_direction = firmata_gpio_get_direction;
	pctrl->gc.direction_output = firmata_gpio_direction_output;
	pctrl->gc.direction_input = firmata_gpio_direction_input;
	pctrl->gc.get = firmata_gpio_get;
	pctrl->gc.get_multiple = firmata_gpio_get_multiple;
	pctrl->gc.set = firmata_gpio_set;
	pctrl->gc.set_multiple = firmata_gpio_set_multiple;
	pctrl->gc.set_config = gpiochip_generic_config,
	pctrl->gc.base = -1;
	pctrl->gc.ngpio = pdata->npins;
	pctrl->gc.can_sleep = true;
	pctrl->pin_caps = pdata->pin_caps;

	pr_info("%s: npins: %d", __func__, pctrl->gc.ngpio);

	mutex_init(&pctrl->lock);
	init_completion(&pctrl->pin_value_read);
	init_completion(&pctrl->pin_state_read);

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

	// Read initial pin states/directions
	for (i = 0; i < pctrl->gc.ngpio; i++)
		firmata_gpio_get_pin_state(pctrl, i);

	err = devm_gpiochip_add_data(dev, &pctrl->gc, pctrl);
	if (err)  {
		dev_err(dev, "Could not register GPIO controller\n");
		// TODO: unregister SPI
		return -ENODEV;
	}

	pr_info("%s probing 2\n", __func__);

	firmata_range.pin_base = pctrl->gc.base;
	firmata_range.base = pctrl->gc.base;
	firmata_range.npins = pctrl->gc.ngpio;
	firmata_range.gc = &pctrl->gc;
	pinctrl_add_gpio_range(pctrl->pctl, &firmata_range);

	return pinctrl_enable(pctrl->pctl);
}

static int pinctrl_firmata_remove(struct platform_device *pdev)
{
//	struct firmata_pinctrl *pctrl = platform_get_drvdata(pdev);

	dev_info(&pdev->dev, "%s: called", __func__);

	firmata_unregister_event_cb(pdev, FIRMATA_GPIO_EVENT);
	firmata_unregister_event_cb(pdev, SYSEX_ID | PIN_STATE_RESPONSE);
	return 0;  // TODO: void in 6.4
}

#ifdef CONFIG_PM
static int firmata_pinctrl_suspend(struct device *dev)
{
	return 0;
}

static int firmata_pinctrl_resume(struct device *pdev)
{
	return 0;
}

static SIMPLE_DEV_PM_OPS(firmata_pm, firmata_pinctrl_suspend, firmata_pinctrl_resume);
#define FIRMATA_PINCTRL_PM_OPS	(&firmata_pm)
#else
#define FIRMATA_PINCTRL_PM_OPS	NULL
#endif
static struct platform_driver pinctrl_firmata_driver = {
	.probe = pinctrl_firmata_probe,
	.remove = pinctrl_firmata_remove,
	.driver = {
		.name = "pinctrl-firmata",
		.pm     = FIRMATA_PINCTRL_PM_OPS,
	},
};

module_platform_driver(pinctrl_firmata_driver);

MODULE_DESCRIPTION("Pinctrl Driver for Firmata firmware on Microcontrollers");
MODULE_AUTHOR("Birger Koblitz");
MODULE_LICENSE("GPL v2");
MODULE_VERSION("0.1");
MODULE_ALIAS("platform:pinctrl-firmata");
