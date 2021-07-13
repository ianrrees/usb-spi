// Minimal device driver to test the USB-SPI kernel driver against
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/spi/spi.h>

static int test_driver_probe(struct spi_device *dev)
{
	int ret = 0;
	dev_info(&dev->dev, "Test driver probed, IRQ: %d", dev->irq);

	ret = spi_write(dev, "hello!", 7);

	dev_info(&dev->dev, "Write done, return code %d", ret);
	
    return 0;
}

int test_driver_remove(struct spi_device *device)
{
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