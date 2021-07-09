
MODNAME := usb_spi

# The VID:PID and interface number of the USB device and its SPI interface
ifeq (x,x$(VID))
  ERROR_STRING += "VID"
endif
ifeq (x,x$(PID))
  ERROR_STRING += "PID"
endif
ifeq (x,x$(INTERFACE))
  ERROR_STRING += "INTERFACE"
endif

obj-m := $(MODNAME).o
ccflags-y := -DDEV_VID=$(VID) -DDEV_PID=$(PID) -DDEV_INTERFACE=$(INTERFACE)

ifneq ($(words $(PWD)),1)
  $(error "Kernel makefiles won't work with spaces in the path")
endif

.PHONY: all
all:
ifeq (x,x$(ERROR_STRING))
	$(MAKE) -C /lib/modules/$(shell uname -r)/build M=$(PWD) modules
else
	@echo "$(ERROR_STRING) unset"
	@echo "Set the device VID, PID, and SPI interface number 'make VID=0x1234 PID=0x5678 INTERFACE=0'"
	@false
endif

.PHONY: clean
clean:
	$(MAKE) -C /lib/modules/$(shell uname -r)/build M=$(PWD) clean

.PHONY: load
load:
	sudo insmod $(MODNAME).ko

.PHONY: unload
unload:
	sudo rmmod $(MODNAME)
