// SPDX-License-Identifier: GPL-2.0+
//
// USB device that provides a SPI interface

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/usb.h>

static int usb_spi_probe(struct usb_interface *interface, const struct usb_device_id *id)
{
    printk(KERN_INFO "USB-SPI (%04X:%04X) found\n", id->idVendor, id->idProduct);
    return 0;
}

static void usb_spi_disconnect(struct usb_interface *interface)
{
    printk(KERN_INFO "USB-SPI removed\n");
}

static struct usb_device_id usb_spi_id[] =
{
    { USB_DEVICE(0x1234, 0x5678) },
    {}
};
MODULE_DEVICE_TABLE (usb, usb_spi_id);

static struct usb_driver usb_spi_driver =
{
    .name = "usb_spi",
    .id_table = usb_spi_id,
    .probe = usb_spi_probe,
    .disconnect = usb_spi_disconnect,
};

static int __init usb_spi_init(void)
{
    return usb_register(&usb_spi_driver);
}

static void __exit usb_spi_exit(void)
{
    usb_deregister(&usb_spi_driver);
}

module_init(usb_spi_init);
module_exit(usb_spi_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Ian Rees <code@ianrees.nz>");
MODULE_DESCRIPTION("USB SPI Driver");
