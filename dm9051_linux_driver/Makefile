##================================================================
##     Davicom Semiconductor Inc.
##   --------------------------------------------------------
## Description..:
##              Compile driver dm9051.c to dm9051.o and dm9051.ko
##
## Modification List:
##================================================================
KERNELVER := $(shell uname -r)
INSTALLDIR := /lib/modules/$(KERNELVER)/kernel/drivers/net/ethernet/davicom
DRIVER_VERSION_STR = "\"1.1.3\""
ccflags-y += -DDRIVER_VERSION=$(DRIVER_VERSION_STR)

#ifneq ($(KERNELRELEASE),)

#(call from kernel build system)
obj-m	:=	dm9051.o

# Check if CONFIG_PTP_1588_CLOCK is set to 'm' or 'y', and add dm9051_ptp.o accordingly
#ifneq ($(filter y m,$(CONFIG_PTP_1588_CLOCK)),)
#    obj-m += dm9051_ptp.o
#endif

dm9051-objs := dm9051_main.o dm9051_ptp.o

#else
# MODULE_INSTALDIR ?= /lib/modules/$(shell uname -r)/kernel/drivers/net/usb
#endif

#KBUILD_EXTRA_SYMBOLS += /home/pi/linux_depth1_1026/linux/Module.symvers
#	$(MAKE) -C $(KERNELDIR) M=$(PWD) modules

KERNELDIR ?= /lib/modules/$(shell uname -r)/build
PWD		:=	$(shell pwd)

default:
	$(MAKE) -C $(KERNELDIR) M=$(PWD)
	xz -z -f $(obj-m:.o=.ko)

clean:
	rm -rf *.ko.xz *.ko *.o *~ core .depend .*.cmd *.mod.c .tmp_versions .*.o.d *.symvers *.order *.bak *.mod

install: default
	sudo mkdir -p $(INSTALLDIR)
	sudo cp dm9051.ko.xz $(INSTALLDIR)/
	sudo depmod -a
	sudo modprobe dm9051

interrupt:
	sudo mkdir -p $(INSTALLDIR)
	sudo cp dm9051.ko.xz $(INSTALLDIR)/
	sudo depmod -a
	sudo modprobe dm9051 use_interrupt=1

	#手動插入 .ko
	# insmod dm9051.ko use_interrupt=1

remove:
#	sudo ifconfig eth1 down
	sudo rmmod dm9051

first:
	sudo ifconfig eth0 down
	sudo insmod dm9051.ko
	sudo ifconfig eth1

second:
	sudo ifconfig eth0 down
	sudo ifconfig eth1 down
	sudo rmmod dm9051
	sudo insmod dm9051.ko
	modinfo dm9051
	sudo ifconfig eth1

reinstall:
test:
tst1:
	#(modprobe -r dm9051)
	# install -c -m 0644 dm9051.ko $(MODULE_INSTALDIR)
	# depmod -a -e
	sudo ifconfig eth1 down
	sudo rmmod dm9051
	sudo insmod dm9051.ko
	sudo ifconfig eth1

eth:
eth1:
	sudo ifconfig eth1

ping1:
	ping 192.168.6.1 -s 32750
ping3:
	ping 10.0.0.3 -s 32750

dm:
	sudo ifconfig eth1 192.168.1.186

int:
	cat /proc/interrupts

info:
	modinfo dm9051
