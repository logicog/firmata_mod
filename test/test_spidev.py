#! /usr/bin/python
import spidev
spi = spidev.SpiDev()
spi.open(1, 0) # bus, device

# Settings (for example)
# spi.max_speed_hz = 5000
spi.mode = 0b00

# to_send = [0x01, 0x02, 0x03]
to_send = [0x33]
spi.xfer(to_send)
