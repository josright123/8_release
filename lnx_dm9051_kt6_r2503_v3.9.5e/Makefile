##================================================================
##     Davicom Semiconductor Inc.  	
##   --------------------------------------------------------
## Description..:
##              Compile driver dm9051.c to dm9051.o and dm9051.ko
##
## Modification List:
##================================================================

#ifneq ($(KERNELRELEASE),)

#(call from kernel build system)
obj-m	:=	dm9051a.o
dm9051a-objs := dm9051.o dm9051_plug.o
#dm9051a-objs := dm9051.o dm9051_ptp.o dm9051_proc.o
#dm9051a-objs := dm9051.o dm9051_plug.o dm9051_crypt.o
#dm9051a-objs := dm9051.o dm9051_ptp.o dm9051_proc.o dm9051_adv.o
#else
# MODULE_INSTALDIR ?= /lib/modules/$(shell uname -r)/kernel/drivers/net/usb
#endif

#KBUILD_EXTRA_SYMBOLS += /home/pi/linux_depth1_1026/linux/Module.symvers 
#	$(MAKE) -C $(KERNELDIR) M=$(PWD) modules 

KERNELDIR ?= /lib/modules/$(shell uname -r)/build
PWD		:=	$(shell pwd)

default:
	$(MAKE) -C $(KERNELDIR) M=$(PWD)
	chmod 644 dm9051a.ko

save:
	cp dm9051a.ko dm9051a_has_tested.ko

clean:
	rm -rf *.o *~ core .depend .*.cmd *.mod.c .tmp_versions .*.o.d *.symvers *.order *.bak *.mod

install:
	sudo insmod dm9051a.ko

remove:
	sudo ifconfig eth1 down
	sudo rmmod dm9051a

first:
	sudo ifconfig eth0 down
	sudo insmod dm9051a.ko
	sudo ifconfig eth1

second:
	sudo ifconfig eth0 down
	sudo ifconfig eth1 down
	sudo rmmod dm9051a
	sudo insmod dm9051a.ko
	modinfo dm9051a
	sudo ifconfig eth1

reinstall:
test:
tst1:
	#(modprobe -r dm9051)
	# install -c -m 0644 dm9051.ko $(MODULE_INSTALDIR)
	# depmod -a -e
	sudo ifconfig eth1 down
	sudo rmmod dm9051a
	sudo insmod dm9051a.ko
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

