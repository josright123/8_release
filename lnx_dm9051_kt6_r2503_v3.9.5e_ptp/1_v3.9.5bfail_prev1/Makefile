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
#dm9051a-objs := dm9051.o dm9051_open.o ((dm9051_conti.o)) (dm9051_ptpd.o) (dm9051_crypt.o)
#dm9051a-objs := dm9051.o dm9051_open.o dm9051_ptpd.o
#dm9051a-objs := dm9051.o
dm9051a-objs := dm9051.o dm9051_ptpd.o
# dm9051_open.o
# dm9051_ptpd.o
# dm9051_conti.o
#
#dm9051a-objs := _dm9051_monitor.o
#dm9051a-objs := dm9051.o dm9051_proc.o dm9051_adv.o
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
	rm -rf *.o plug/*.o .*.cmd plug/.*.cmd *~ core .depend *.mod.c .tmp_versions .*.o.d *.symvers *.order *.bak *.mod

ins install :
	sudo insmod dm9051a.ko

rm remove :
#	sudo ifconfig eth1 down
	sudo rmmod dm9051a

ls:
	lsmod | grep dm9
msg:
	sudo dmesg
rmmsg:
	sudo dmesg -c

down:
	sudo ifconfig eth1 down
up:
	sudo ifconfig eth1 up
if:
	ifconfig eth1
cat:
	cat /proc/interrupts

ifpin pin pin1:
	ping 192.168.1.254 -I eth1 -s 128
ifp p p1:
	ping 192.168.6.1 -I eth1 -s 64
#	ping 192.168.6.1 -I eth1 -s 64

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

test test1 reinstall :
	#(modprobe -r dm9051)
	# install -c -m 0644 dm9051.ko $(MODULE_INSTALDIR)
	# depmod -a -e
	sudo ifconfig eth1 down
	sudo rmmod dm9051a
	sudo insmod dm9051a.ko
	sudo ifconfig eth1

eth eth1:
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

