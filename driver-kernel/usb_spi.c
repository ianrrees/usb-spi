// SPDX-License-Identifier: GPL-2.0+
//
// Provides a SPI “Controller Driver” for an attached USB device

#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/irqdomain.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>
#include <linux/usb.h>
#include <linux/workqueue.h>

// Generated from a Rust package
#include "../protocol/usb-spi-protocol.h"

#define USB_TIMEOUT_MS 1000

// TODO add max SPI speed, mode, etc (reset state?) and check
// against them in usb_spi_transfer_one_message()
struct usb_spi_connected_chip {
	int virq;
	bool irq_mask;
};

struct usb_spi_device {
	struct usb_device *usb_dev;
	struct spi_master *spi_master;

	struct usb_spi_connected_chip *connected_chips;
	u16 connected_chip_count;
	/// -1 initially, then the index of the selected device
	int selected_chip;

	// struct usb_spi_connected_peripheral *connected;
	struct mutex usb_mutex;
	u16 usb_interface_index;
	unsigned int bulk_out_pipe;
	unsigned int bulk_in_pipe;
	unsigned hardware_buf_size;
	unsigned usb_buf_sz;
	void *usb_buffer; // Protected by usb_mutex

	/// Worker checking for events (namely interrupts) from the device
	struct work_struct poll_work;
	/// Microseconds between polls for events from the device
	int poll_interval;

	struct irq_domain *irq_domain;
	struct mutex irq_mutex;
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

/// Returns number of bytes transferred, or negative error code
static int usb_spi_control_in(struct usb_spi_device *usb_spi, u8 request, u16 value, void *buf, u16 len)
{
	return usb_control_msg(
		usb_spi->usb_dev,
		usb_rcvctrlpipe(usb_spi->usb_dev, 0),
		request,
		USB_TYPE_VENDOR | USB_RECIP_INTERFACE | USB_DIR_IN,
		value,
		usb_spi->usb_interface_index,
		buf,
		len,
		USB_TIMEOUT_MS
	);
}

// /// Returns number of bytes transferred, or negative error code
// static int usb_spi_control_out(struct usb_spi_device *usb_spi, u8 request, u16 value, void *buf, u16 len)
// {
// 	return usb_control_msg(
// 		usb_spi->usb_dev,
// 		usb_sndctrlpipe(usb_spi->usb_dev, 0),
// 		request,
// 		USB_TYPE_VENDOR | USB_RECIP_INTERFACE | USB_DIR_OUT,
// 		value,
// 		usb_spi->usb_interface_index,
// 		buf,
// 		len,
// 		USB_TIMEOUT_MS
// 	);
// }

// Called after spi_new_device(), not when a driver attaches
static int usb_spi_setup(struct spi_device *spi)
{
	struct usb_spi_device *usb_spi = spi_master_get_devdata(spi->master);
	dev_info(&usb_spi->usb_dev->dev, "usb_spi_setup() for %s on CS %d",
	         spi->modalias, spi->chip_select);
	return 0;
}

// Called by spi_unregister_master(), not when a driver detaches
static void usb_spi_cleanup(struct spi_device *spi)
{
	struct usb_spi_device *usb_spi = spi_master_get_devdata(spi->master);
	dev_info(&usb_spi->usb_dev->dev, "usb_spi_cleanup() for %s on CS %d",
	         spi->modalias, spi->chip_select);
}

/// Transfers data over the bulk endpoints
/// Returns negative error code, or number of bytes transferred
static int usb_spi_transfer_chunk(struct usb_spi_device *usb_spi, struct spi_transfer *xfer, unsigned offset)
{
	int ret = 0;
	int actual_len = 0;
	unsigned out_len = 0;
	unsigned data_len = xfer->len - offset;

	// caller has locked usb_mutex already

	struct usb_spi_TransferHeader *header = (struct usb_spi_TransferHeader *) usb_spi->usb_buffer;
	memset(header, 0, sizeof(*header));

	if (xfer->tx_buf) {
		data_len = min(data_len, usb_spi->usb_buf_sz - (unsigned)sizeof(*header));
		if (xfer->rx_buf) {
			data_len = min(data_len, usb_spi->hardware_buf_size);
			header->direction = usb_spi_Direction_Both;
		} else {
			header->direction = usb_spi_Direction_OutOnly;
		}
		header->bytes = cpu_to_le16(data_len);
		out_len = sizeof(*header) + data_len;

		memcpy(usb_spi->usb_buffer + sizeof(*header), xfer->tx_buf + offset, data_len);

	} else if (xfer->rx_buf) {
		data_len = min(data_len, usb_spi->usb_buf_sz);
		data_len = min(data_len, usb_spi->hardware_buf_size);
		header->direction = usb_spi_Direction_InOnly;
		header->bytes = cpu_to_le16(data_len);
		out_len = sizeof(header);

	} else {
		dev_warn(&usb_spi->usb_dev->dev, "Transfer has no Tx or Rx buf!?");
		ret = -EINVAL;
		goto err;
	}

	{ // TODO make a bulk transfer fn that handles retries automatically
		unsigned wrote_len = 0;
		while (wrote_len < out_len) {
			ret = usb_bulk_msg(usb_spi->usb_dev,
			                   usb_spi->bulk_out_pipe,
			                   usb_spi->usb_buffer,
			                   out_len,
			                   &actual_len,
			                   USB_TIMEOUT_MS);
			if (ret < 0 && ret != -ETIMEDOUT) {
				dev_err(&usb_spi->usb_dev->dev, "Error sending bulk OUT: %s (%d)", error_to_string(ret), ret);
				goto err;
			}
			wrote_len += actual_len;
		}
	}

	if (xfer->rx_buf) {
		unsigned read_len = 0;
		while (read_len < data_len) {
			actual_len = 0;
			// TODO shorter timeout? need to loop on this, could predict time required if we knew baud
			ret = usb_bulk_msg(usb_spi->usb_dev,
			                   usb_spi->bulk_in_pipe,
							   usb_spi->usb_buffer,
							   data_len - read_len,
							   &actual_len,
							   USB_TIMEOUT_MS);
			memcpy(xfer->rx_buf + offset + read_len, usb_spi->usb_buffer, actual_len);
			read_len += actual_len;
			if (ret < 0 && ret != -ETIMEDOUT) {
				dev_err(&usb_spi->usb_dev->dev, "Error receiving bulk IN: %s (%d)", error_to_string(ret), ret);
				goto err;
			}
		}
	}

	return data_len;
err:
	return ret;
}

// Chip select -1 deasserts the CS, nonnegative asserts
static int usb_spi_chip_select(struct usb_spi_device *usb_spi, int chip_select) {
	int ret = 0;
	int actual_len = 0;
	int total_len = 0;

	// Caller has locked usb_mutex
	struct usb_spi_TransferHeader *header = (struct usb_spi_TransferHeader *) usb_spi->usb_buffer;
	memset(header, 0, sizeof(*header));

	if (chip_select < 0) {
		header->direction = usb_spi_Direction_CsDeassert;

	} else if (chip_select >= usb_spi->connected_chip_count) {
		dev_err(&usb_spi->usb_dev->dev,
		        "Chip select %d is out of range [0, %d)",
		        chip_select, usb_spi->connected_chip_count);
		ret = -EINVAL;
		goto err;

	} else {
		header->direction = usb_spi_Direction_CsAssert;
		header->bytes = cpu_to_le16(chip_select);
	}

	while (total_len < sizeof(*header)) {
		ret = usb_bulk_msg(usb_spi->usb_dev,
		                   usb_spi->bulk_out_pipe,
		                   usb_spi->usb_buffer,
		                   sizeof(*header),
		                   &actual_len,
		                   USB_TIMEOUT_MS);
		total_len += actual_len;
		if (ret < 0 && ret != -ETIMEDOUT) {
			dev_err(&usb_spi->usb_dev->dev, "Error sending bulk OUT: %s (%d)", error_to_string(ret), ret);
			goto err;
		}
	}

err:
	return ret;
}

static int usb_spi_transfer_one_message(struct spi_master *master, struct spi_message *mesg)
{
	struct usb_spi_device *usb_spi = spi_master_get_devdata(master);
	struct spi_transfer *xfer = NULL;
	int ret = 0;

	mutex_lock(&usb_spi->usb_mutex);

	ret = usb_spi_chip_select(usb_spi, mesg->spi->chip_select);
	if (ret < 0) {
		goto err;
	}

	list_for_each_entry(xfer, &mesg->transfers, transfer_list) {
		int transferred = 0;

		dev_dbg(&usb_spi->usb_dev->dev, "SPI transfer: %s%s, %dB, Speed:%d",
		         xfer->tx_buf?"Tx":"", xfer->rx_buf?"Rx":"", xfer->len, xfer->speed_hz);
		if (xfer->bits_per_word != 8) {
			dev_warn(&usb_spi->usb_dev->dev,
			         "SPI transfer has %d bits per word, only 8 is supported",
					 xfer->bits_per_word);
		}

	// TODO check SPI mode

		while (transferred < xfer->len) {
			ret = usb_spi_transfer_chunk(usb_spi, xfer, transferred);
			if (ret < 0) {
				goto err;
			}

			transferred += ret;
			mesg->actual_length += ret;
			ret = 0;
		}
	}

	ret = usb_spi_chip_select(usb_spi, -1);

err:
	mutex_unlock(&usb_spi->usb_mutex);
	mesg->status = ret;
	spi_finalize_current_message(master);
	return ret;
}

static void usb_spi_poll_events(struct work_struct *work)
{
	struct usb_spi_device *usb_spi = container_of(work, struct usb_spi_device, poll_work);
	int ret = 0;

loop:
	mutex_lock(&usb_spi->usb_mutex);

	ret = usb_spi_control_in(usb_spi, usb_spi_ControlIn_GetEvent, 0, usb_spi->usb_buffer, sizeof(struct usb_spi_Event));
	if (ret == -ENODEV) {
		return; // Plug got yanked

	} if (ret < 0) {
		mutex_unlock(&usb_spi->usb_mutex);
		dev_err(&usb_spi->usb_dev->dev, "Control IN GetEvent failed %d: %s", ret, error_to_string(ret));

	} else if (ret != sizeof(struct usb_spi_Event)) {
		mutex_unlock(&usb_spi->usb_mutex);
		dev_err(&usb_spi->usb_dev->dev, "Invalid length response (%d) to GetEvent", ret);

	} else {
		const struct usb_spi_Event *received = (const struct usb_spi_Event *)usb_spi->usb_buffer;
		struct usb_spi_Event event = {
			.data = le16_to_cpu(received->data),
			.event_type = received->event_type,
		};

		mutex_unlock(&usb_spi->usb_mutex);

		switch (event.event_type) {
			case usb_spi_EventType_NoEvent:
				dev_dbg(&usb_spi->usb_dev->dev, "Got NoEvent");
				goto no_event;

			case usb_spi_EventType_Interrupt: {
				size_t chip_id = event.data;
				if (chip_id < usb_spi->connected_chip_count) {
					dev_dbg(&usb_spi->usb_dev->dev, "Hardware IRQ %ld", chip_id);

					if (usb_spi->connected_chips[chip_id].irq_mask) {
						unsigned long flags = 0;
						local_irq_save(flags);
						generic_handle_irq(usb_spi->connected_chips[chip_id].virq);
						local_irq_restore(flags);
					}
				} else {
					dev_err(&usb_spi->usb_dev->dev,
					        "Got interrupt message for invalid chip ID %ld", chip_id);
				}
				break;
			}
		}
		
		// If we're getting events, read them as fast as possible
		goto loop;
	}

no_event:
	if (usb_spi->poll_interval >= 0) {
		usleep_range(usb_spi->poll_interval, usb_spi->poll_interval + 20);
	}
	if (usb_spi->poll_interval >= 0) {
		goto loop;
	}
}

/// Returns 0 on success, or negative error code
static int usb_spi_get_master_info(struct usb_spi_device *usb_spi)
{
	int ret = 0;

	struct usb_spi_MasterInfo *info = usb_spi->usb_buffer;
	mutex_lock(&usb_spi->usb_mutex);

	ret = usb_spi_control_in(usb_spi, usb_spi_ControlIn_HwInfo, 0, info, sizeof(*info));
	if (ret < 0) {
		dev_err(&usb_spi->usb_dev->dev, "Control IN HwInfo failed %d: %s", ret, error_to_string(ret));
		goto err;
	} else if (ret != sizeof(*info)) {
		dev_err(&usb_spi->usb_dev->dev, "Invalid length response (%d) to HwInfo", ret);
		ret = -EINVAL;
		goto err;
	} else {
		ret = 0;
	}

	if (info->hardware != usb_spi_HardwareType_SpiMaster) {
		dev_err(&usb_spi->usb_dev->dev, "Device isn't an SPI master, unsupported");
		ret = -EINVAL;
		goto err;
	}

	usb_spi->selected_chip = -1;
	usb_spi->connected_chip_count = le16_to_cpu(info->slave_count);
	usb_spi->hardware_buf_size = le16_to_cpu(info->in_buf_size);

err:
	mutex_unlock(&usb_spi->usb_mutex);
	return ret;
}

static void usb_spi_irq_mask(struct irq_data *data)
{
	struct usb_spi_device *usb_spi = irq_data_get_irq_chip_data(data);

	size_t hwirq = data->hwirq;
	if (hwirq < usb_spi->connected_chip_count) {
		dev_dbg(&usb_spi->usb_dev->dev, "Masking hardware IRQ %lu", hwirq);
		usb_spi->connected_chips[hwirq].irq_mask = true;
	} else {
		dev_err(&usb_spi->usb_dev->dev,
		        "Request to mask out-of-range hardware IRQ %lu", hwirq);
	}
}

static void usb_spi_irq_unmask(struct irq_data *data)
{
	struct usb_spi_device *usb_spi = irq_data_get_irq_chip_data(data);

	size_t hwirq = data->hwirq;
	if (hwirq < usb_spi->connected_chip_count) {
		dev_dbg(&usb_spi->usb_dev->dev, "Unmasking hardware IRQ %lu", hwirq);
		usb_spi->connected_chips[hwirq].irq_mask = false;
	} else {
		dev_err(&usb_spi->usb_dev->dev,
		        "Request to unmask out-of-range hardware IRQ %lu", hwirq);
	}
}

static int usb_spi_irq_set_type(struct irq_data *data, unsigned int type)
{
	return 0;
}

static void usb_spi_irq_bus_lock(struct irq_data *data)
{
	struct usb_spi_device *usb_spi = irq_data_get_irq_chip_data(data);
	mutex_lock(&usb_spi->irq_mutex);
}

static void usb_spi_irq_bus_unlock(struct irq_data *data)
{
	struct usb_spi_device *usb_spi = irq_data_get_irq_chip_data(data);
	mutex_unlock(&usb_spi->irq_mutex);
}

static struct irq_chip usb_spi_irq_chip = {
	.name                = "usb-spi-irqs",
	.irq_mask            = usb_spi_irq_mask,
	.irq_unmask          = usb_spi_irq_unmask,
	.irq_set_type        = usb_spi_irq_set_type,
	.irq_bus_lock        = usb_spi_irq_bus_lock,
	.irq_bus_sync_unlock = usb_spi_irq_bus_unlock,
};

/// Can we make a 5-TLD fn?
static int usb_spi_irq_map(struct irq_domain *domain, unsigned int virq, irq_hw_number_t hwirq)
{
	irq_set_chip_data(virq, domain->host_data);
	irq_set_chip(virq, &usb_spi_irq_chip);
	irq_set_chip_and_handler(virq, &usb_spi_irq_chip, handle_simple_irq);
	irq_set_noprobe(virq);
	return 0;
}

static const struct irq_domain_ops usb_spi_irq_domain_ops = {
	.map = usb_spi_irq_map,
};

static int usb_spi_probe(struct usb_interface *usb_if, const struct usb_device_id *id)
{
	struct usb_device *usb_dev = NULL;
	struct usb_spi_device *usb_spi = NULL;
	struct spi_master *spi_master = NULL;
	struct usb_endpoint_descriptor *ep_in = NULL;
	struct usb_endpoint_descriptor *ep_out = NULL;
	int ret = 0;
	int i;

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

	// usb_find_common_endpoints() isn't available in old kernels, so reinvent that wheel
	for (i = 0; i < usb_if->cur_altsetting->desc.bNumEndpoints; ++i) {
		struct usb_endpoint_descriptor *desc = &usb_if->cur_altsetting->endpoint[i].desc;
		if (usb_endpoint_type(desc) == USB_ENDPOINT_XFER_BULK) {
			if (usb_endpoint_dir_in(desc)) {
				if (ep_in == NULL) {
					ep_in = desc;
				} else {
					dev_err(&usb_dev->dev, "Multiple IN endpoint descriptors");
					ret = -EINVAL;
					goto err;
				}
			} else if (ep_out == NULL) {
				ep_out = desc;
			} else {
				dev_err(&usb_dev->dev, "Multiple OUT endpoint descriptors");
				ret = -EINVAL;
				goto err;
			}
		}
	}

	if (ep_in == NULL || ep_out == NULL) {
		// TODO Support half-duplex with only one bulk endpoint
		dev_err(&usb_dev->dev, "Missing/corrupt bulk endpoint descriptors");
		goto err;
	}

	usb_spi->bulk_in_pipe = usb_rcvbulkpipe(usb_dev, ep_in->bEndpointAddress);
	usb_spi->bulk_out_pipe = usb_sndbulkpipe(usb_dev, ep_out->bEndpointAddress);

	usb_spi->usb_buf_sz = (unsigned)usb_endpoint_maxp(&usb_dev->ep0.desc);
	usb_spi->usb_buf_sz = max(usb_spi->usb_buf_sz, (unsigned)usb_endpoint_maxp(ep_in));
	usb_spi->usb_buf_sz = max(usb_spi->usb_buf_sz, (unsigned)usb_endpoint_maxp(ep_out));

	// TODO Pick a reasonable minimum size, verify with assertions
	usb_spi->usb_buf_sz = max(usb_spi->usb_buf_sz, (unsigned)sizeof(struct usb_spi_TransferHeader));
	// TODO could probably do with a bigger buffer for the bulk transfers

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

	/// The spi_board_info needs IRQ, so set those up before SPI

	mutex_init(&usb_spi->irq_mutex);

	usb_spi->connected_chips = kcalloc(
		usb_spi->connected_chip_count,
		sizeof(struct usb_spi_connected_chip),
		GFP_KERNEL);
	if (!usb_spi->connected_chips) {
		ret = -ENOMEM;
		goto err;
	}

	usb_spi->irq_domain = irq_domain_add_linear(
		usb_dev->dev.of_node, usb_spi->connected_chip_count,
		&usb_spi_irq_domain_ops, usb_spi
	);
	if (usb_spi->irq_domain) {
		for (i = 0; i < usb_spi->connected_chip_count; ++i) {
			usb_spi->connected_chips[i].virq = irq_create_mapping(usb_spi->irq_domain, i);
			dev_dbg(&usb_dev->dev, "USB-SPI mapping hardware IRQ %d to virtual IRQ %d",
			        i, usb_spi->connected_chips[i].virq);
		}
	} else {
		dev_err(&usb_dev->dev, "Failed to register IRQ domain");
		ret = -ENODEV;
		goto err;
	}

	/// SPI setup starts here; move to another function?

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
	spi_master->num_chipselect = usb_spi->connected_chip_count;
	spi_master->mode_bits = SPI_MODE_0 | SPI_MODE_1 | SPI_MODE_2 | SPI_MODE_3;

	spi_master->flags = 0;
	spi_master->setup = usb_spi_setup;
	spi_master->cleanup = usb_spi_cleanup;
	spi_master->transfer_one_message = usb_spi_transfer_one_message;

	ret = spi_register_master(spi_master);
	if (ret) {
		dev_err(&usb_dev->dev, "Error registering SPI: %s (%d)", error_to_string(ret), ret);
		spi_master_put(spi_master); // Typically handled by spi_unregister_master
		spi_master = NULL;
		goto err;
	}
	usb_spi->spi_master = spi_master;

	dev_info(&usb_dev->dev, "USB-SPI device %s %s %s providing SPI %d",
	         usb_dev->manufacturer, usb_dev->product, usb_dev->serial, spi_master->bus_num);

	// TODO this needs to be in a separate method - maybe for all the SPI setup?
	for (i = 0; i < usb_spi->connected_chip_count; ++i) {
		struct spi_board_info board_info = {0};

		// TODO this pattern needs to check the size of the usb_buffer
		struct usb_spi_ConnectedSlaveInfoLinux *slave_info = usb_spi->usb_buffer;
		mutex_lock(&usb_spi->usb_mutex);

		ret = usb_spi_control_in(usb_spi, usb_spi_ControlIn_LinuxSlaveInfo, i, slave_info, sizeof(*slave_info));
		if (ret < 0) {
			dev_err(&usb_spi->usb_dev->dev, "Control IN LinuxSlaveInfo failed %d: %s", ret, error_to_string(ret));
			mutex_unlock(&usb_spi->usb_mutex);
			goto err;
		} else if (ret != sizeof(*slave_info)) {
			dev_err(&usb_spi->usb_dev->dev, "Invalid length response (%d) to LinuxSlaveInfo", ret);
			ret = -EINVAL;
			goto err;
		} else {
			ret = 0;
		}

		dev_info(&usb_spi->usb_dev->dev, "Registering \"%s\" on CS %d", slave_info->modalias, i);

		board_info.bus_num = spi_master->bus_num;
		board_info.chip_select = i;
		board_info.irq = usb_spi->connected_chips[i].virq;
		strncpy(board_info.modalias, slave_info->modalias, sizeof(board_info.modalias));

		mutex_unlock(&usb_spi->usb_mutex);

		spi_new_device(spi_master, &board_info);
	}

	usb_spi->poll_interval = 100 * 1000; // microseconds
	INIT_WORK(&usb_spi->poll_work, usb_spi_poll_events);
	schedule_work(&usb_spi->poll_work);

	usb_set_intfdata(usb_if, usb_spi);

	return ret;

err:
	if (spi_master) {
		spi_unregister_master(spi_master);
	}

	if (usb_spi) {
		if (usb_spi->irq_domain) {
			if (usb_spi->connected_chips) {
				size_t i = 0;
				for (i=0; i < usb_spi->connected_chip_count; ++i) {
					irq_dispose_mapping(usb_spi->connected_chips[i].virq);
				}
			}
			irq_domain_remove(usb_spi->irq_domain);
		}
		if (usb_spi->connected_chips) {
			kfree(usb_spi->connected_chips);
		}
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
	int i = 0;

	// Stop polling for events from the device
	i = usb_spi->poll_interval;
	usb_spi->poll_interval = -1;
	usleep_range(i, i*2);

	// Tear down SPI before IRQ, so drivers can free their allocated interrupts
	spi_unregister_master(usb_spi->spi_master);

	for (i=0; i < usb_spi->connected_chip_count; ++i) {
		irq_dispose_mapping(usb_spi->connected_chips[i].virq);
	}
	irq_domain_remove(usb_spi->irq_domain);

	kfree(usb_spi->connected_chips);

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
