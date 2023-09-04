# What's this?
Firmata_mod is a collection of Linux kernel modules that allow you to access
GPIO pins, I2C and SPI devices on an Arduino running the Firmata firmware
in such a way, that they appear as GPIO/SPI/I2C devices on the host computer
connecting to the Arduino.

# Installation
Install build dependencies to build external kernel modules. On Debian you
can simply install the linux headers and all necessary build dependencies
such as the compiler used to build the running kernel will be pulled in,
too:
```
$ sudo apt-get install linux-headers-`uname -r`
```

In the firmata_mod directory do:
```
$ make
$ sudo make modules_install
```
You can ignore the warnings regarding BTF generation.

Now you can load the firmata kernel module:

```
$ sudo modprobe firmata_mod [firmata_port=TTY_PORT] [baud_rate=N]
```
where TTY_PORT is your tty-port which connects to the Arduino. The default
is "ttyUSB0". This must match the Port setting under Tools in the Arduino
configuration menu, but without the "/dev/" part. The optional baud_rate
argument gives the baud_rate. By default it is 57600 like in
StandardFirmata, ConfigurableFirmata would use 115200.
The modprobe command above may take several seconds to return, since the
module verifies the Arduino with Firmata firmware is correctly configured.
You can verify whether the drivers are loaded using lsmod:
```
$ lsmod | grep firmata
spi_firmata            16384  0
i2c_firmata            16384  0
gpio_firmata           16384  0
firmata_mod            20480  3 spi_firmata,i2c_firmata,gpio_firmata
```
You can unload the driver again with:
```
$ sudo rmmod gpio_firmata spi_firmata i2c_firmata firmata_mod
```

# GPIO-support
In order to get more information about the now available GPIO lines, you can install
the gpiod tools. Alternatively, the information is also directly available
in the /sys file-system. In order to test gpio-access via python, the
python3-libgpiod library is very convenient: 
```
$ sudo apt install gpiod python3-libgpiod
```
After loading firmata_mod, a new GPIO chip should be available with the 20
pins of the Arduino:
```
$ sudo gpiodetect
gpiochip0 [INT34C8:00] (340 lines)
gpiochip1 [ftdi-cbus] (4 lines)
gpiochip2 [firmata-gpio] (20 lines)
```
The on-board LED of the Arduino which is by default using pin 13 can then be
enabled/disabled with:
```
$ sudo gpioset gpiochip2 13=1
$ sudo gpioset gpiochip2 13=0
```
You can also directly manipulate GPIOs from within the /sys filesystem
```
$ cd /sys/class/gpio
$ ls -l
 total 0
 --w------- 1 root root 4096 Aug 29 08:31 export
 lrwxrwxrwx 1 root root    0 Aug 29 08:31 gpiochip660 -> ../../devices/platform/firmata.0/firmata-gpio.3.auto/gpio/gpiochip660
 lrwxrwxrwx 1 root root    0 Aug 29 08:31 gpiochip680 -> ../../devices/pci0000:00/0000:00:14.0/usb1/1-1/1-1:1.0/gpio/gpiochip680
 lrwxrwxrwx 1 root root    0 Aug 29 07:43 gpiochip684 -> ../../devices/platform/INT34C8:00/gpio/gpiochip684
 --w------- 1 root root 4096 Aug 29 08:31 unexport

$ echo 677 > export
$ echo "out" > gpio677/direction 
$ echo 1 > gpio677/value 
$ echo 0 > gpio677/value 
```

Be aware that pin 13 (which is typically the on-board LED) may be assigned to SPI
when running ConfigurableFirmata. That means that with the spi-firmata
module loaded setting that pin via gpio will not work.

# I2C-support
If you want to work with I2C devices, you need to make sure i2c is
accessible through userspace. Verify that the "i2c-tools" package is
installed, otherwise install it with
```
$ sudo apt install i2c-tools
```
which will make udev rules for the i2c-devices available and provide
userspace tools to communicate with i2c devices.
You also need to load the i2c_dev module, which creates i2c-devices
such as /dev/i2c-1 with the help of the udev rules:
```
$ sudo modprobe i2c_dev
$ ls -l /dev | grep i2c
crw-rw----  1 root   i2c      89,     0 Aug 15 14:28 i2c-0
crw-rw----  1 root   i2c      89,     1 Aug 15 14:28 i2c-1
crw-rw----  1 root   i2c      89,    10 Aug 15 14:28 i2c-10
crw-rw----  1 root   i2c      89,    11 Aug 15 14:28 i2c-11
crw-rw----  1 root   i2c      89,    12 Aug 15 14:28 i2c-12
crw-rw----  1 root   i2c      89,    13 Aug 15 14:28 i2c-13
crw-rw----  1 root   i2c      89,    14 Aug 15 14:28 i2c-14
crw-rw----  1 root   i2c      89,    15 Aug 15 14:28 i2c-15
crw-rw----  1 root   i2c      89,    16 Aug 15 14:28 i2c-16
crw-rw----  1 root   i2c      89,     2 Aug 15 14:28 i2c-2
crw-rw----  1 root   i2c      89,     3 Aug 15 14:28 i2c-3
crw-rw----  1 root   i2c      89,     4 Aug 15 14:28 i2c-4
crw-rw----  1 root   i2c      89,     5 Aug 15 14:28 i2c-5
crw-rw----  1 root   i2c      89,     6 Aug 15 14:28 i2c-6
crw-rw----  1 root   i2c      89,     7 Aug 15 14:28 i2c-7
crw-rw----  1 root   i2c      89,     8 Aug 15 14:28 i2c-8
crw-rw----  1 root   i2c      89,     9 Aug 15 14:28 i2c-9
```

A python test-script can be found in the test source directory, which
expects an SHT21 temperature / relative humidity sensor connected to the
primary I2C bus on the Arduino. It uses the smbus python package, which can
be installed under debian with
```
$ sudo apt install python3-smbus
$ cd test
$ sudo ./test_sht21.py
1692102741 20.8 86.3
```

Note, that due to the permissions under /dev/ for the i2c devices, the user
must either be in the i2c group, or the command must be executed as "root".
Evidently for the the python implementation of smbus to work, the i2c_dev
kernel module will need to be loaded, otherwise you will get something like:
```
    bus = smbus.SMBus(16)
          ^^^^^^^^^^^^^^^
FileNotFoundError: [Errno 2] No such file or directory
```
The bus-number give (16 in the above case) can be learned from running
```
$ i2cdetect -l
i2c-0   i2c             Synopsys DesignWare I2C adapter         I2C adapter
i2c-1   smbus           SMBus I801 adapter at efa0              SMBus adapter
i2c-2   i2c             Synopsys DesignWare I2C adapter         I2C adapter
i2c-3   i2c             Synopsys DesignWare I2C adapter         I2C adapter
i2c-4   i2c             Synopsys DesignWare I2C adapter         I2C adapter
i2c-5   i2c             Synopsys DesignWare I2C adapter         I2C adapter
i2c-6   i2c             i915 gmbus dpa                          I2C adapter
i2c-7   i2c             i915 gmbus dpb                          I2C adapter
i2c-8   i2c             i915 gmbus dpc                          I2C adapter
i2c-9   i2c             i915 gmbus tc1                          I2C adapter
i2c-10  i2c             i915 gmbus tc2                          I2C adapter
i2c-11  i2c             i915 gmbus tc3                          I2C adapter
i2c-12  i2c             i915 gmbus tc4                          I2C adapter
i2c-13  i2c             i915 gmbus tc5                          I2C adapter
i2c-14  i2c             i915 gmbus tc6                          I2C adapter
i2c-15  i2c             AUX B/DDI B/PHY B                       I2C adapter
i2c-16  i2c             firmata-i2c-firmata.0-0                 I2C adapter
```
It is also possible to use existing kernel modules with I2C hardware, to use
e.g. the in-kernel SHT21 module, do:
```
/sys/bus/i2c/devices/i2c-16# echo sht21 0x40 >new_device
# dmesg | tail
 [ 6735.579122] i2c i2c-16: new_device: Instantiated device sht21 at 0x40
/sys/bus/i2c/devices/i2c-16/16-0040/hwmon/hwmon4# cat temp1_input 
  25404
```
Which means it is 25.4 degrees warm.

# SPI-Support
To test SPI, again we first need a udev rule for the creation of
an spidev device under /dev/. You can create one manually (note that on a
Raspberry Pi such rules are already present under "99-com.rules"), while on
a Debian PC I created one manually:
```
$ cat /lib/udev/rules.d/60-spidev.rules
$ SUBSYSTEM=="spidev", KERNEL=="spidev[0-9]*.[0-9]*", GROUP="spi", MODE="0660"
```
Make sure there is an spi group available. To reload the udev rules without
a reboot, do:
```
$ sudo udevadm control --reload-rules && sudo udevadm trigger
```
In order for the spi kernel module to be able to create an spidev user-space
device, the driver needs to be listed in the spidev_spi_ids[] in the spidev
module, see https://docs.kernel.org/spi/spidev.html

The easiest approach here is to copy spidev.c from your kernel sources into
the firmata_mod directory, add it among the modules in the Makefile and
compile them along with the firmata modules, after having added the line
```
static const struct spi_device_id spidev_spi_ids[] = {
	[...]
	{ .name = "spi-firmata" },
	{},
};
```
You can query the spi device using the spi-tools and python3-spidev
packages.

First, figure out which spi-device is being made available:
```
$ cd /sys/bus/spi/devices
$ ls -l
total 0
lrwxrwxrwx 1 root root 0 Sep  3 16:07 spi1.0 -> ../../../devices/platform/firmata.0/firmata-spi.5.auto/spi_master/spi1/spi1.0
```
This means that firmata makes spi-device 0 on bus 1 available (device spi1.0)
in userspace through the spidev module.
