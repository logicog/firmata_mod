/* SPDX-License-Identifier: GPL-2.0 */
#ifndef __LINUX_FIRMATA_H
#define __LINUX_FIRMATA_H

// TODO: Move this to the gpio module
#include <linux/gpio/driver.h>


/* Mode settings sent via Firmata Protocol */
#define MODE_INPUT		0x00
#define MODE_OUTPUT		0x01
#define MODE_ANALOG		0x02
#define MODE_PWM		0x03
#define MODE_SERVO		0x04
#define MODE_SHIFT		0x05
#define MODE_I2C		0x06
#define MODE_ONEWIRE		0x07
#define MODE_STEPPER		0x08
#define MODE_ENCODER		0x09
#define MODE_SERIAL		0x0A
#define MODE_PULLUP		0x0B
#define MODE_IGNORE		0x7F

/* Message codes */
#define START_SYSEX		0xF0 // start a MIDI Sysex message
#define END_SYSEX		0xF7 // end a MIDI Sysex message
#define STRING_MESSAGE		0x71
#define PIN_MODE_QUERY		0x72 // ask for current and supported pin modes
#define PIN_MODE_RESPONSE	0x73 // reply with current and supported pin modes
#define PIN_STATE_QUERY		0x6D
#define PIN_STATE_RESPONSE	0x6E
#define CAPABILITY_QUERY	0x6B
#define CAPABILITY_RESPONSE	0x6C
#define ANALOG_MAPPING_QUERY	0x69
#define ANALOG_MAPPING_RESPONSE	0x6A
#define SPI_DATA		0x68
#define REPORT_FIRMWARE		0x79 // report name and version of firmware
#define REPORT_DIGITAL		0xd0
#define DIGITAL_IO_MESSAGE	0x90
#define SET_PIN_MODE_MESSAGE	0xf4
#define I2C_REPLY		0x77
#define REPORT_VERSION		0xf9
#define SYSEX_ID		(START_SYSEX << 8)
#define RESET_CB			0x00

#define FIRMATA_SIZE_TXBUF 32
#define FIRMATA_SIZE_RXBUF 1024

#define FIRMATA_GPIO_EVENT 1

/**
 * firmata_event_cb_t - event callback function signature
 *
 * @pdev  - the sub-device that registered this callback
 * @rxbuf - the reiceive buffer data
 * @len   - the data length
 *
 * The callback function is called in interrupt context and the data payload is
 * only valid during the call. If the user needs later access of the data, it
 * must copy it.
 */
typedef int (*firmata_event_cb_t)(struct platform_device *pdev, const u8 rxbuf[], int len);
// TODO: const char?


int firmata_serial_tx(struct platform_device *pdev, char *buf, int len);
int firmata_register_event_cb(struct platform_device *pdev, u16 id,
			   firmata_event_cb_t event_cb);
void firmata_unregister_event_cb(struct platform_device *pdev, u16 id);


struct firmata_platform_data {
	bool host_controls_cs;
	// maybe use tty device number? tty_dev_name_to_number
	uint8_t port;
};

#endif
