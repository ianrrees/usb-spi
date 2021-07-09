
MODNAME := usb-spi

obj-m := $(MODNAME).o

.PHONY: all
all:
	$(MAKE) -C /lib/modules/$(shell uname -r)/build M=$(PWD) modules

.PHONY: clean
clean:
	$(MAKE) -C /lib/modules/$(shell uname -r)/build M=$(PWD) clean

.PHONY: load
load:
	sudo insmod $(MODNAME).ko

.PHONY: unload
unload:
	sudo rmmod $(MODNAME)