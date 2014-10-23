obj-m += cypress_usbdownloader.o
	
KDIR := /work/imap/Infotm/linux
all:
	make -C $(KDIR) M=$(PWD) 
clean:
	rm -f *.o *.mod.o *.mod.c *.symvers *.ko *.order
