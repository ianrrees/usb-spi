
MODNAME := test_driver

obj-m := $(MODNAME).o
ccflags-y := -DDEV_VID=$(VID) -DDEV_PID=$(PID) -DDEV_INTERFACE=$(INTERFACE)

ifneq ($(words $(PWD)),1)
  $(error "Kernel makefiles won't work with spaces in the path")
endif

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
