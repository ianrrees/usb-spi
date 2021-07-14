# atsamd-usbd-spi

Work in progress toward a firmware USB Class for ATSAMD parts, which implements a USB-to-SPI-master.

## TODO
* Think about a better way to organise the SpiDevice instances - maybe find/make
  a crate that implements a linked list so the UsbSpi can just hold a single
  `&'static mut dyn SpiDevice` which then refers to the next.  Might be a good
  model for usb-device to use in a new iteration where the Bus owns any used
  Class(es).