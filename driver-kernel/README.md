Linux kernel driver for USB-SPI driver
===
Provides a Linux SPI controller interface, where the SPI hardware is provided by a USB-connected device.

Supported Kernels
---
Currently, this is expected to work with Linux 3.2.26 (and possibly older) through the version used by current Ubuntu LTS (tested up to 5.4.0).  Some small compromises have been made to accommodate the older kernels.

Building
---
This is intended to be built as a kernel module at present.  From an Ubuntu bases, some prerequisites will be required:
  * git (to clone this repo)
  * build-essential (provides make, among others)
  * linux-headers-$(uname -r) (headers for your particular Linux kernel)

The driver needs to know which USB interface to bind to, but since the USB-SPI interface isn't standard (like CDC ACM is, for example) it can't be automatically detected without potentially conflicting with other USB interfaces.  So, the makefile requires a VID:PID:Interface tuple for the relevant device and interface on that device.  Once the prerequisites are installed, the driver module can be built with:
```
make VID=0x1234 PID=0x5678 INTERFACE=1
```

Using
---
Load the module using `# insmod usb_spi.ko`, unload with `# rmmod usb_spi`.  When the module is loaded, and a device matching the VID:PID specified above is attached, debug information is made available using the usual kernel logging facilities like `dmesg`, including when the device is attached and when there are USB errors.  Currently, the logging is a bit noisy - in particular it generates fairly frequent (every few minutes) error-level like `[70402.311782] usb 1-5.1.4.1.1: Control IN GetEvent failed -32: EPIPE` - these are actually mostly harmless.

Only a subset of the Linux SPI interface is supported, but that subset appears to include the most commonly-used parts.  See the source code for more details.