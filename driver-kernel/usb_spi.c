// SPDX-License-Identifier: GPL-2.0+
//
// Provides a SPI “Controller Driver” for an attached USB device

#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/usb.h>
#include <linux/spi/spi.h>

// Generated from a Rust package
#include "../protocol/usb-spi-protocol.h"

#define USB_TIMEOUT_MS 1000

// struct usb_spi_connected_peripheral {
// 	// May not be necessary to store this?
// 	char modalias[SPI_NAME_SIZE];
// 	// during probe, bool whether the hardware provides an IRQ. Then the IRQ
// 	int irq;
// 	// TODO platform data
// };

struct usb_spi_device {
	struct usb_device *usb_dev;
	struct spi_master *spi_master;
	// struct usb_spi_connected_peripheral *connected;
	struct mutex usb_mutex;
	size_t usb_buf_sz;
	u16 usb_interface_index;
	u16 connected_count;
	u8 *usb_buffer; // Protected by usb_mutex
};

// TODO this is bound to exist somewhere else...
// Error list is from https://www.kernel.org/doc/html/latest/driver-api/usb/error-codes.html
static const char * error_to_string(int err)
{
	if (err >= 0) {
		return "None";
	}

	switch (err) {
		case -ENOMEM:       return "ENOMEM";
		case -EBUSY:        return "EBUSY";
		case -ENODEV:       return "ENODEV";
		case -ENOENT:       return "ENOENT";
		case -ENXIO:        return "ENXIO";
		case -EINVAL:       return "EINVAL";
		case -EXDEV:        return "EXDEV";
		case -EFBIG:        return "EFBIG";
		case -EPIPE:        return "EPIPE"; //usb_control_msg returns this when the firmware rejects
		case -EMSGSIZE:     return "EMSGSIZE";
		case -EBADR:        return "EBADR";
		case -ENOSPC:       return "ENOSPC";
		case -ESHUTDOWN:    return "ESHUTDOWN";
		case -EPERM:        return "EPERM";
		case -EHOSTUNREACH: return "EHOSTUNREACH";
		case -ENOEXEC:      return "ENOEXEC";
		case -ETIMEDOUT:    return "ETIMEDOUT";
		default:            return "???";
	}
}

/// Called after spi_new_device(), not when a driver attaches
static int usb_spi_setup(struct spi_device *spi) { return 0; }

/// Called by spi_unregister_master(), not when a driver detaches
static void usb_spi_cleanup(struct spi_device *spi) { }

static int usb_spi_transfer_one_message(struct spi_master *master, struct spi_message *mesg)
{
	struct usb_spi_device *usb_spi = spi_master_get_devdata(master);
	struct spi_transfer *xfer = NULL;
	int ret = 0;

	dev_info(&usb_spi->usb_dev->dev, "usb_spi_transfer_one_message() CS:%d",
	         mesg->spi->chip_select); // TODO

	list_for_each_entry(xfer, &mesg->transfers, transfer_list) {
		dev_info(&usb_spi->usb_dev->dev, "SPI transfer: %p, %p, %d",
		        xfer->tx_buf, xfer->rx_buf, xfer->len);
		if (xfer->bits_per_word != 8) {
			dev_warn(&usb_spi->usb_dev->dev,
			         "SPI transfer has %d bits per word, only 8 is supported",
					 xfer->bits_per_word);
		}

		// TODO actually make the transfer over USB

		mesg->actual_length += xfer->len;
	}

	mesg->status = ret;
	spi_finalize_current_message(master);
	return ret;
}

static struct spi_board_info spi_board_info[] = {
{
        .modalias       = "spi-test",
        // .platform_data  = &ads_info,
        .mode           = SPI_MODE_0,
        .irq            = 1234,
        .max_speed_hz   = 120000 /* max sample rate at 3V */ * 16,
        .bus_num        = 1,
        .chip_select    = 0,
},
};

/// Returns number of bytes transferred, or negative error code
static int usb_spi_control_in(struct usb_spi_device *usb_spi, u8 request, u16 value, void *buf, u16 len)
{
	return usb_control_msg(
		usb_spi->usb_dev,
		usb_sndctrlpipe(usb_spi->usb_dev, 0),
		request,
		USB_TYPE_VENDOR | USB_RECIP_INTERFACE | USB_DIR_IN,
		value,
		usb_spi->usb_interface_index,
		buf,
		len,
		USB_TIMEOUT_MS
	);
}

/// Returns 0 on success, or negative error code
static int usb_spi_get_master_info(struct usb_spi_device *usb_spi)
{
	int ret = 0;
	const struct usb_spi_MasterInfo *info = NULL;

	mutex_lock(&usb_spi->usb_mutex);
	ret = usb_spi_control_in(usb_spi, REQUEST_IN_HW_INFO, 0, usb_spi->usb_buffer, sizeof(*info));
	if (ret < 0) {
		dev_err(&usb_spi->usb_dev->dev, "Control IN REQUEST_IN_HW_INFO failed %d: %s", ret, error_to_string(ret));
		goto err;
	} else if (ret != sizeof(*info)) {
		dev_err(&usb_spi->usb_dev->dev, "Invalid length response (%d) to REQUEST_IN_HW_INFO", ret);
		ret = -EINVAL;
		goto err;
	} else {
		ret = 0;
	}

	info = (const struct usb_spi_MasterInfo *) usb_spi->usb_buffer;

	if (info->hardware != SpiMaster) {
		dev_err(&usb_spi->usb_dev->dev, "Device isn't an SPI master, unsupported");
		ret = -EINVAL;
		goto err;
	}

	usb_spi->connected_count = info->slave_count;

err:
	mutex_unlock(&usb_spi->usb_mutex);
	return ret;
}

static int usb_spi_probe(struct usb_interface *usb_if, const struct usb_device_id *id)
{
	struct usb_device *usb_dev = NULL;
	struct usb_spi_device *usb_spi = NULL;
	struct spi_master *spi_master = NULL;
	struct usb_endpoint_descriptor *ep_in = NULL;
	struct usb_endpoint_descriptor *ep_out = NULL;
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
	usb_spi->usb_interface_index = usb_if->cur_altsetting->desc.bInterfaceNumber;

	// Find size of biggest endpoint we'll deal with
	ret = usb_find_common_endpoints(usb_if->cur_altsetting, &ep_in, &ep_out, NULL, NULL);
	if (ret) {
		// TODO Support half-duplex with only one bulk endpoint
		dev_err(&usb_dev->dev, "Missing/corrupt bulk endpoint descriptors");
		goto err;
	}

	usb_spi->usb_buf_sz = (size_t)usb_endpoint_maxp(&usb_dev->ep0.desc);
	usb_spi->usb_buf_sz = max(usb_spi->usb_buf_sz, (size_t)usb_endpoint_maxp(ep_in));
	usb_spi->usb_buf_sz = max(usb_spi->usb_buf_sz, (size_t)usb_endpoint_maxp(ep_out));

	usb_spi->usb_buffer = kmalloc(usb_spi->usb_buf_sz, GFP_KERNEL);
	if (!usb_spi->usb_buffer) {
		ret = -ENOMEM;
		goto err;
	}

	mutex_init(&usb_spi->usb_mutex);

	ret = usb_spi_get_master_info(usb_spi);
	if (ret < 0) {
		goto err;
	}

	spi_master = spi_alloc_master(&usb_dev->dev, sizeof(usb_spi));
	if (!spi_master) {
		ret = -ENOMEM;
		goto err;
	}

	spi_master_set_devdata(spi_master, usb_spi);

	// TODO get parameters from the device?  Do they matter?
	spi_master->min_speed_hz = 1000;
	spi_master->max_speed_hz = 1000;

	spi_master->bus_num = -1; // Kernel should allocate our bus_num
	spi_master->num_chipselect = usb_spi->connected_count;
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

	dev_info(&usb_dev->dev, "USB-SPI device %s %s %s providing SPI %d",
	         usb_dev->manufacturer, usb_dev->product, usb_dev->serial, spi_master->bus_num);


	// for (i = 0; i < count; ++i) {
	// 	// Slight chicken-and-egg problem here: we need to read in devices to
	// 	// determine if they provide IRQs, so we don't know how many IRQs to
	// 	// allocate.  Add an irq_count field, then fetch each?

	// 	const struct usb_spi_ConnectedSlaveInfoLinux *slave_info = NULL;
	// 	// TODO this pattern needs to check the size of the usb_buffer
	// 	mutex_lock(&usb_spi->usb_mutex);
	// 	ret = usb_spi_control_in(usb_spi, REQUEST_IN_LINUX_SLAVE_INFO, i, usb_spi->usb_buffer, sizeof(*slave_info));
	// 	if (ret < 0) {
	// 		dev_err(&usb_spi->usb_dev->dev, "Control IN REQUEST_IN_LINUX_SLAVE_INFO failed %d: %s", ret, error_to_string(ret));
	// 		goto err;
	// 	}

	// 	slave_info = (const struct usb_spi_ConnectedSlaveInfoLinux *) usb_spi->usb_buffer;

	// 	if (slave_info->has_interrupt) {
	// 		dev_info(&usb_spi->usb_dev->dev, "\t\"%s\", has interrupt", slave_info->modalias);
	// 	} else {
	// 		dev_info(&usb_spi->usb_dev->dev, "\t\"%s\"", slave_info->modalias);
	// 	}
	// 	mutex_unlock(&usb_spi->usb_mutex);

		spi_new_device(spi_master, spi_board_info);
	// }

	usb_set_intfdata(usb_if, usb_spi);

	return ret;

err:
	if (spi_master) {
		spi_unregister_master(spi_master);
	}
	if (usb_spi) {
		usb_put_dev(usb_spi->usb_dev);
		if (usb_spi->usb_buffer) {
			kfree(usb_spi->usb_buffer);
		}
		kfree(usb_spi);
	}

	return ret;
}

static void usb_spi_disconnect(struct usb_interface *interface)
{
	struct usb_spi_device *usb_spi = usb_get_intfdata(interface);

	spi_unregister_master(usb_spi->spi_master);

	dev_info(&usb_spi->usb_dev->dev, "USB-SPI disconnected");

	usb_put_dev(usb_spi->usb_dev);
	kfree(usb_spi->usb_buffer);
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
