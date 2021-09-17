# USB-SPI Kit
This repo contains a stack of [ATSAMDx1 USB device firmware](/atsamd-usbd-spi) (only tested on ATSAMD21), [USB protocol](/protocol), Linux kernel [usb-spi driver](/driver-kernel), and a [demo spi driver](test-driver).  These work together to provide a standard Linux SPI controller, which connects to the SAMDx1 over USB and uses its SERCOM peripheral for the physical layer.

For instance, a person could wire an SPI peripheral (a radio module, analog to digital converter, memory, display, etc) to a SAMD21 development board (Adafruit Feather M0, Arduino Zero, etc), connect the board to a Linux host via USB, and run a Linux program that interacts with the peripheral as if it were connected directly to an SPI controller in the host computer (via a kernel driver for the SPI peripheral, or via [spidev](https://www.kernel.org/doc/html/latest/spi/spidev.html)).

## Design goals and non-goals
I originally wrote this for a work project, basically taking an SPI peripheral that had been connected directly to a SoC and moving the device to a SAMD21 that connects to the main SoC over USB.  So, the main aim was to support the SPI functionality that peripheral and its driver use.  But, this seems sufficiently generic to possibly be useful for other things.

Speed is not much of a concern in the original application.  So, this might not be a good choice for higher bandwidth or lower-latency applications.  I have not profiled this stack, but offhand there are some easy gains that could be made in the firmware to reduce `memcpy()`-ing the SPI data to/from the USB bulk endpoints.  The protocol only puts one transaction in each USB transfer, that could probably be changed fairly easily to a streaming approach.

## TODO
* Add a complete example firmware application
