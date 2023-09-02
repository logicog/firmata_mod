ifneq ($(KERNELRELEASE),)
# kbuild part of makefile
obj-m := firmata_mod.o spi-firmata.o gpio-firmata.o i2c-firmata.o

else
# normal makefile
KDIR ?= /lib/modules/`uname -r`/build

default:
	make -C $(KDIR) M=$$PWD

modules_install:
	cp *.ko  /lib/modules/`uname -r`/kernel
	depmod
#	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) INSTALL_MOD_STRIP=1 modules_install

clean:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) clean

endif
