// Minimal device driver to test the USB-SPI kernel driver against
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/spi/spi.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/delay.h>

#define BUF_SIZE 10

struct test_driver_device {
	struct spi_device *spi_dev;
	struct work_struct irq_work;
	int irq_interval; // microseconds, negative to shut down
	size_t irq_count;
};

static void test_dev_irq_work(struct work_struct *work)
{
	struct test_driver_device *test_dev = container_of(work, struct test_driver_device, irq_work);
	size_t last_irq_count = 0;

loop:
	if (test_dev->irq_count != last_irq_count) {
		dev_info(&test_dev->spi_dev->dev, "test_driver IRQ fired!");
		last_irq_count = test_dev->irq_count;
	}

	if (test_dev->irq_interval >= 0) {
		usleep_range(test_dev->irq_interval, test_dev->irq_interval+1000);
	}
	if (test_dev->irq_interval >= 0) {
		goto loop;
	}
}

static irqreturn_t test_driver_irq(int irq, void *ptr)
{
	struct test_driver_device *test_dev = (struct test_driver_device *)ptr;
	++(test_dev->irq_count);

	return IRQ_HANDLED;
}

static int test_driver_probe(struct spi_device *spi_dev)
{
	int ret = 0;

	struct test_driver_device *test_dev = NULL;
	u8 *buf = NULL;

	test_dev = kzalloc(sizeof(*test_dev), GFP_KERNEL);
	if (!test_dev) {
		return -ENOMEM;
	}

	buf = kmalloc(BUF_SIZE, GFP_KERNEL);
	if (!buf) {
		ret = -ENOMEM;
		goto err;
	}
	memset(buf, 0xFF, BUF_SIZE);

	dev_info(&spi_dev->dev, "Test driver probed, IRQ: %d", spi_dev->irq);

	if (spi_dev->irq >= 0) {
		// TODO try without IRQF_TRIGGER_RISING
		ret = request_irq(spi_dev->irq, test_driver_irq, IRQF_SHARED, "test driver", test_dev);
		if (ret < 0) {
			dev_err(&spi_dev->dev, "Failed to register IRQ handler, ret: %d", ret);
			goto err;
		}
		dev_info(&spi_dev->dev, "Registered IRQ %d, ret: %d", spi_dev->irq, ret);
	}

	ret = spi_write(spi_dev, "\xA0\x01\x03\x60\x03", 5);

	if (ret >= 0) {
		ret = spi_read(spi_dev, buf, 5);

		dev_info(&spi_dev->dev, "Read: 0x%02X %02X %02X %02X %02X %02X %02X %02X %02X %02X",
			buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6], buf[7], buf[8], buf[9]);
	}

	dev_info(&spi_dev->dev, "Test transfers done, return code %d", ret);
	ret = 0;

	test_dev->spi_dev = spi_dev;
	spi_set_drvdata(spi_dev, test_dev);
	test_dev->irq_interval = 10 * 1000;

	INIT_WORK(&test_dev->irq_work, test_dev_irq_work);
	schedule_work(&test_dev->irq_work);

err:
	if (buf) {
		kfree(buf);
	}

	if (ret<0 && test_dev) {
		kfree(test_dev);
	}

	return ret;
}

int test_driver_remove(struct spi_device *spi_dev)
{
	int interval;
	struct test_driver_device *test_dev = spi_get_drvdata(spi_dev);

	dev_info(&spi_dev->dev, "Removing test driver, had IRQ %d", spi_dev->irq);
	if (spi_dev->irq >= 0) {
		free_irq(spi_dev->irq, test_dev);
	}

	interval = test_dev->irq_interval;
	test_dev->irq_interval = -1;
	usleep_range(interval, 2*interval);

	kfree(test_dev);

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
	printk(KERN_INFO "test_driver_module_init()");
    spi_register_driver(&test_driver);
	return 0;
}

static void __exit test_driver_module_exit(void)
{
	printk(KERN_INFO "test_driver_module_exit()");
	spi_unregister_driver(&test_driver);
}

module_init(test_driver_module_init);
module_exit(test_driver_module_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Ian Rees <code@ianrees.nz>");
MODULE_DESCRIPTION("SPI Driver to test USB-SPI driver against");