// Minimal device driver to test the USB-SPI kernel driver against
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/spi/spi.h>
#include <linux/interrupt.h>

#define BUF_SIZE 10

static irqreturn_t test_driver_irq(int irq, void *ptr)
{
	dev_info(&((const struct spi_device *)(ptr))->dev, "In test_driver_irq() for IRQ %d!", irq);

	return IRQ_HANDLED;
}

static int test_driver_probe(struct spi_device *dev)
{
	int ret = 0;

	u8 *buf = kmalloc(BUF_SIZE, GFP_KERNEL);
	if (!buf) {
		return -ENOMEM;
	}
	memset(buf, 0xFF, BUF_SIZE);

	dev_info(&dev->dev, "Test driver probed, IRQ: %d", dev->irq);

	if (dev->irq >= 0) {
		// TODO try without IRQF_TRIGGER_RISING
		ret = request_irq(dev->irq, test_driver_irq, IRQF_SHARED | IRQF_TRIGGER_RISING, "test driver", dev);
		if (ret < 0) {
			dev_err(&dev->dev, "Failed to register IRQ handler, ret: %d", ret);
			goto err;
		}
	}

	if (ret >= 0) {
		ret = spi_write(dev, "\xA0", 1);
	}
	if (ret >= 0) {
		ret = spi_write(dev, "\x01", 1);
	}
	if (ret >= 0) {
		ret = spi_write(dev, "\x03", 1);
	}
	if (ret >= 0) {
		ret = spi_write(dev, "\x60", 1);
	}
	if (ret >= 0) {
		ret = spi_write(dev, "\x03", 1);
	}

	// ret = spi_write(dev, "\xA0\x01\x03\x60\x03", 5);

	if (ret >= 0) {
		ret = spi_read(dev, buf, 5);

		dev_info(&dev->dev, "Read: 0x%02X %02X %02X %02X %02X %02X %02X %02X %02X %02X",
			buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6], buf[7], buf[8], buf[9]);
	}

	dev_info(&dev->dev, "Test transfers done, return code %d", ret);

err:
	if (buf) {
		kfree(buf);
	}
	
    return 0;
}

int test_driver_remove(struct spi_device *dev)
{
	if (dev->irq >= 0) {
		free_irq(dev->irq, dev);
	}
	dev_info(&dev->dev, "Removing test driver, had IRQ %d", dev->irq);
    return 0;
}

struct spi_driver test_driver = {
	.driver	= {
		.name	= "spi-test",
		.owner	= THIS_MODULE,
	},
	.probe	= test_driver_probe,
	.remove	= test_driver_remove,
};

static int __init test_driver_module_init(void)
{
    spi_register_driver(&test_driver);
	return 0;
}

static void __exit test_driver_module_exit(void)
{
	spi_unregister_driver(&test_driver);
}

module_init(test_driver_module_init);
module_exit(test_driver_module_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Ian Rees <code@ianrees.nz>");
MODULE_DESCRIPTION("SPI Driver to test USB-SPI driver against");