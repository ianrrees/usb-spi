// SPDX-License-Identifier: GPL-2.0+
//
// Provides a SPI “Controller Driver” for an attached USB device

#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/usb.h>

typedef struct {

} usb_spi_t;

static int usb_spi_probe(struct usb_interface *probing_interface, const struct usb_device_id *id)
{
    printk(KERN_INFO "USB-SPI candidate device (%04X:%04X) found\n", id->idVendor, id->idProduct);

    // For now, just assume the table only gives us valid interfaces
    usb_spi_t *usb_spi = usb_spi = kzalloc(sizeof(usb_spi_t), GFP_KERNEL);
    if (!usb_spi) {
        return -ENOMEM;
    }

    // TODO	- do we need to call usb_get_intf() to increase refcount on the interface?

    usb_set_intfdata(probing_interface, usb_spi);
    return 0;
}

static void usb_spi_disconnect(struct usb_interface *interface)
{
    usb_spi_t *usb_spi = usb_get_intfdata(interface);

    printk(KERN_INFO "USB-SPI removed\n");
    
    kfree(usb_spi);
}

static struct usb_device_id usb_spi_id[] =
{
    // TODO This and/or poll() could check the class/subclass/protocol
    {USB_DEVICE_INTERFACE_NUMBER((DEV_VID), (DEV_PID), (DEV_INTERFACE))},
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
