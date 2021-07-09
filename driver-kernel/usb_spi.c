// SPDX-License-Identifier: GPL-2.0+
//
// Provides a SPI “Controller Driver” for an attached USB device

#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/usb.h>
#include <linux/spi/spi.h>

struct usb_spi_device {
	struct usb_device *usb_dev;
	struct spi_master *spi_master;
};

static int usb_spi_setup(struct spi_device *spi)
{
	printk(KERN_INFO "usb_spi_setup()"); // TODO
	return 0;
}

static void usb_spi_cleanup(struct spi_device *spi)
{
	printk(KERN_INFO "usb_spi_cleanup()"); // TODO
}

static int usb_spi_transfer_one_message(struct spi_master *master, struct spi_message *mesg)
{
	struct usb_spi_device *usb_spi = spi_master_get_devdata(master);

	dev_info(&usb_spi->usb_dev->dev, "usb_spi_transfer_one_message()"); // TODO
	return 0;
}

static int usb_spi_probe(struct usb_interface *usb_if, const struct usb_device_id *id)
{
	struct usb_device *usb_dev = NULL;
	struct usb_spi_device *usb_spi = NULL;
	struct spi_master *spi_master = NULL;
	int ret = 0;

	usb_dev = usb_get_dev(interface_to_usbdev(usb_if));
	if (!usb_dev) {
		return -ENODEV;
	}

	// For now, just assume the table only gives us valid USB interfaces

	usb_spi = kzalloc(sizeof(*usb_spi), GFP_KERNEL);
	if (!usb_spi) {
		return -ENOMEM;
	}

	usb_spi->usb_dev = usb_dev;

	spi_master = spi_alloc_master(&usb_spi->usb_dev->dev, sizeof(usb_spi));
	if (!spi_master) {
		ret = -ENOMEM;
		goto err;
	}

	spi_master_set_devdata(spi_master, usb_spi);

	// TODO get parameters from the device
	spi_master->min_speed_hz = 1000;
	spi_master->max_speed_hz = 1000;

	spi_master->bus_num = -1; // dynamic
	spi_master->num_chipselect = 1; // TODO hook up GPIO
	spi_master->mode_bits = SPI_MODE_0 | SPI_MODE_1 | SPI_MODE_2 | SPI_MODE_3;

	spi_master->flags = 0;
	spi_master->setup = usb_spi_setup;
	spi_master->cleanup = usb_spi_cleanup;
	spi_master->transfer_one_message = usb_spi_transfer_one_message;

	ret = spi_register_master(spi_master);
	if (ret) {
		spi_master_put(spi_master); // Typically handled by spi_unregister_master
		spi_master = NULL;
		goto err;
	}
	usb_spi->spi_master = spi_master;

	dev_info(&usb_spi->usb_dev->dev, "New USP-SPI device %04X:%04X SPI %d",
	         id->idVendor, id->idProduct, spi_master->bus_num);

	usb_set_intfdata(usb_if, usb_spi);
	return ret;

err:
	if (spi_master) {
		spi_unregister_master(spi_master);
	}
	if (usb_spi) {
		usb_put_dev(usb_spi->usb_dev);
		kfree(usb_spi);
	}

	return ret;
}

static void usb_spi_disconnect(struct usb_interface *interface)
{
	struct usb_spi_device *usb_spi = usb_get_intfdata(interface);

	dev_info(&usb_spi->usb_dev->dev, "USB-SPI disconnected");

	spi_unregister_master(usb_spi->spi_master);

	usb_put_dev(usb_spi->usb_dev);
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
