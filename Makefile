#########################################################################################
#             Makefile for AX78140/AX78120/MCS7840/MCS7820/MCS7810   LINUX Driver          #      
#########################################################################################

KDIR:=/lib/modules/$(shell uname -r)/build
MDIR:=/lib/modules/$(shell uname -r)/kernel/drivers/usb/serial

EXTRA_CFLAGS += -I$(KDIR)/drivers/usb/serial


obj-m:= mos7840.o

default:
	$(MAKE) -C $(KDIR) $(EXTRA_CFLAGS) M=$(PWD) modules
	gcc -pthread ioctl.c -o ioctl

clean:
	$(MAKE) -C $(KDIR) $(EXTRA_CFLAGS) M=$(PWD) clean
	rm -rf .tmp_versions Module.symvers *.mod.c *.o *.ko .*.cmd Module.markers modules.order ioctl *~ com test *.txt

load:
	-modprobe usbserial
	modprobe lp
	insmod mos7840.ko

unload:
	modprobe -r lp
	rmmod mos7840

install:
	sh mosinstall

uninstall:
	sh mosuninstall






