/*****************************************************************************
*
* Copyright (c) 2014, Advanced Micro Devices, Inc.   
* All rights reserved.   
*
* Redistribution and use in source and binary forms, with or without   
* modification, are permitted provided that the following conditions are met:   
*     * Redistributions of source code must retain the above copyright   
*       notice, this list of conditions and the following disclaimer.   
*     * Redistributions in binary form must reproduce the above copyright   
*       notice, this list of conditions and the following disclaimer in the   
*       documentation and/or other materials provided with the distribution.   
*     * Neither the name of Advanced Micro Devices, Inc. nor the names of    
*       its contributors may be used to endorse or promote products derived    
*       from this software without specific prior written permission.   
*    
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND   
* ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED   
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE   
* DISCLAIMED. IN NO EVENT SHALL ADVANCED MICRO DEVICES, INC. BE LIABLE FOR ANY   
* DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES   
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;   
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND   
* ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT   
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS   
* SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.   
*    
*   
***************************************************************************/
#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt
#include <linux/init.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/gpio.h>
#include <linux/pci.h>
#include <linux/ioport.h>
#include <linux/platform_device.h>
#include <linux/uaccess.h>
#include <linux/io.h>

#include <asm/io.h>

#include "gpio-amd.h"

/* Module and version information */
#define GPIO_VERSION "0.1"
#define GPIO_MODULE_NAME "AMD GPIO"
#define GPIO_DRIVER_NAME   GPIO_MODULE_NAME ", v" GPIO_VERSION

#define DRV_NAME "amd-gpio"

static u32 gpiobase_phys;
static u32 iomuxbase_phys;
static struct pci_dev *amd_gpio_pci;
static struct platform_device *amd_gpio_platform_device;

/*
 * The following GPIO pins are reserved:
 *   0-31
 *   34-42
 *   44
 *   46
 *   53-54
 *   72
 *   81-95
 *   120-160
 *   171-173
 *   175-183
 *   185-226
 */
static u8 mask[] = {
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 1,
	1, 1, 1, 1, 1, 0, 0, 1, 1, 1, 1, 1,
	1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
	0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
	1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1,
	1, 1, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
	1
};

static int gpio_mask[AMD_GPIO_NUM_PINS];
static unsigned int num_mask;
module_param_array(gpio_mask, int, &num_mask, 0);
MODULE_PARM_DESC(gpio_mask, "GPIO mask which marks them as reserved");

static int gpio_mode[AMD_GPIO_NUM_PINS];
static unsigned int num_modes;
module_param_array(gpio_mode, int, &num_modes, 0);
MODULE_PARM_DESC(gpio_mode, "Specifies whether the GPIO mentioned "
		 "in gpio_mask is 0-reserved, 1-available, 2-GPI only, "
		 "3-GPO only");

static struct amd_gpio_chip {
	struct gpio_chip gpio;

	void __iomem *gpiobase;
	void __iomem *iomuxbase;

	struct platform_device *pdev;
	spinlock_t lock;
} amd_gpio_chip;

static int amd_gpio_request(struct gpio_chip *c, unsigned offset)
{
	struct amd_gpio_chip *chip = container_of(c, struct amd_gpio_chip, gpio);
	unsigned long flags;
	u8 iomux_reg;
	u8 gpio_reg;

	spin_lock_irqsave(&chip->lock, flags);

	/* check if this pin is available */
	if (mask[offset] == AMD_GPIO_MODE_RESV) {
		spin_unlock_irqrestore(&chip->lock, flags);
		pr_info("GPIO pin %u not available\n", offset);
		return -EINVAL;
	}

	/* check if EC owns the GPIO pin */
	gpio_reg = ioread8((u8 *)amd_gpio_chip.gpiobase + offset);
	if (gpio_reg & AMD_GPIO_OWNER_EC){
		spin_unlock_irqrestore(&chip->lock, flags);
		pr_info("GPIO pin %u not available\n", offset);
		return -ENODEV;
	}

	/* Set Host as owner of GPIO, disable both Pull Up and Pull Down */
	gpio_reg |= AMD_GPIO_OWNER_HOST | AMD_GPIO_DISABLE_PULLUP;
	gpio_reg &= ~AMD_GPIO_ENABLE_PULLDOWN;
	iowrite8(gpio_reg, ((u8 *)amd_gpio_chip.gpiobase + offset));

	gpio_reg = ioread8((u8 *)amd_gpio_chip.gpiobase + offset);
	/* Enable GPIO by writing to the corresponding IOMUX register */
	iomux_reg = ioread8((u8 *)amd_gpio_chip.iomuxbase + offset);
	iomux_reg &= ~AMD_IOMUX_GPIO_MASK;
	iomux_reg |= AMD_IOMUX_ENABLE_FUNC3;
	iowrite8(iomux_reg, ((u8 *)amd_gpio_chip.iomuxbase + offset));

	gpio_reg = ioread8((u8 *)amd_gpio_chip.gpiobase + offset);
	spin_unlock_irqrestore(&chip->lock, flags);

	return 0;
}

static int amd_gpio_get(struct gpio_chip *c, unsigned offset)
{
	struct amd_gpio_chip *chip = container_of(c, struct amd_gpio_chip, gpio);
	unsigned long flags;
	u8 gpio_reg;

	spin_lock_irqsave(&chip->lock, flags);

	/* Read the GPIO register */
	gpio_reg = ioread8((u8 *)amd_gpio_chip.gpiobase + offset);

	spin_unlock_irqrestore(&chip->lock, flags);

	return (gpio_reg & AMD_GPIO_GET_INPUT) ? 1 : 0;
}

static void amd_gpio_set(struct gpio_chip *c, unsigned offset, int val)
{
	struct amd_gpio_chip *chip = container_of(c, struct amd_gpio_chip, gpio);
	unsigned long flags;
	u8 gpio_reg;

	spin_lock_irqsave(&chip->lock, flags);

	gpio_reg = ioread8((u8 *)amd_gpio_chip.gpiobase + offset);

	/* Set GPIO Output depending on 'val' */
	if (val)
		gpio_reg |= AMD_GPIO_SET_OUTPUT;
	else
		gpio_reg &= ~AMD_GPIO_SET_OUTPUT;

	iowrite8(gpio_reg, ((u8 *)amd_gpio_chip.gpiobase + offset));

	spin_unlock_irqrestore(&chip->lock, flags);
}

static int amd_gpio_direction_input(struct gpio_chip *c, unsigned offset)
{
	struct amd_gpio_chip *chip = container_of(c, struct amd_gpio_chip, gpio);
	unsigned long flags;
	u8 gpio_reg;

	spin_lock_irqsave(&chip->lock, flags);

	/* If the mask says the pin should be GPO, we return from here */
	if (mask[offset] == AMD_GPIO_MODE_OUTPUT) {
		pr_info("GPIO %u can only be set in output mode\n", offset);
		spin_unlock_irqrestore(&chip->lock, flags);
		return -EINVAL;
	}

	gpio_reg = ioread8((u8 *)amd_gpio_chip.gpiobase + offset);
	/* Disable output */
	gpio_reg |= AMD_GPIO_DISABLE_OUTPUT;
	iowrite8(gpio_reg, ((u8 *)amd_gpio_chip.gpiobase + offset));

	spin_unlock_irqrestore(&chip->lock, flags);

	return 0;
}

static int amd_gpio_direction_output(struct gpio_chip *c, unsigned offset,
				     int val)
{
	struct amd_gpio_chip *chip = container_of(c, struct amd_gpio_chip, gpio);
	unsigned long flags;
	u8 gpio_reg;

	spin_lock_irqsave(&chip->lock, flags);

	/* If the mask says the pin should be GPI, we return from here */
	if (mask[offset] == AMD_GPIO_MODE_INPUT) {
		pr_info("GPIO %u can only be set in input mode\n", offset);
		spin_unlock_irqrestore(&chip->lock, flags);
		return -EINVAL;
	}

	gpio_reg = ioread8((u8 *)amd_gpio_chip.gpiobase + offset);
	/* Enable output */
	gpio_reg &= ~AMD_GPIO_DISABLE_OUTPUT;
	iowrite8(gpio_reg, ((u8 *)amd_gpio_chip.gpiobase + offset));

	/* Read the GPIO register back */
	gpio_reg = ioread8((u8 *)amd_gpio_chip.gpiobase + offset);

	/* Set GPIO Output depending on 'val' */
	if (val)
		gpio_reg |= AMD_GPIO_SET_OUTPUT;
	else
		gpio_reg &= ~AMD_GPIO_SET_OUTPUT;

	iowrite8(gpio_reg, ((u8 *)amd_gpio_chip.gpiobase + offset));

	spin_unlock_irqrestore(&chip->lock, flags);

	return 0;
}

static struct amd_gpio_chip amd_gpio_chip = {
	.gpio = {
		.owner = THIS_MODULE,
		.label = DRV_NAME,

		.base = 0,
		.ngpio = AMD_GPIO_NUM_PINS,
		.names = NULL,
		.request = amd_gpio_request,
		.get = amd_gpio_get,
		.set = amd_gpio_set,
		.direction_input = amd_gpio_direction_input,
		.direction_output = amd_gpio_direction_output,
	},
};

/*
 * The PCI Device ID table below is used to identify the platform
 * the driver is supposed to work for. Since this is a platform
 * driver, we need a way for us to be able to find the correct
 * platform when the driver gets loaded, otherwise we should
 * bail out.
 */
static DEFINE_PCI_DEVICE_TABLE(amd_gpio_pci_tbl) = {
	{ PCI_VENDOR_ID_AMD, PCI_DEVICE_ID_AMD_HUDSON2_SMBUS, PCI_ANY_ID,
	  PCI_ANY_ID, },
	{ 0, },
};
MODULE_DEVICE_TABLE(pci, amd_gpio_pci_tbl);

static void amd_update_gpio_mask(void)
{
	u8 iomux_reg;

	/*
	 * Some of the GPIO pins have an alternate function assigned to
	 * them. That will be reflected in the corresponding IOMUX
	 * registers. If so, we mark these GPIO pins as reserved.
	 */
	/* GPIO45 */
	iomux_reg = ioread8((u8 *)amd_gpio_chip.iomuxbase + 0x2D);
	if ((iomux_reg & AMD_IOMUX_GPIO_MASK) == AMD_IOMUX_ENABLE_FUNC0)
		mask[45] = AMD_GPIO_MODE_RESV;

	/* GPIO48 */
	iomux_reg = ioread8((u8 *)amd_gpio_chip.iomuxbase + 0x30);
	if ((iomux_reg & AMD_IOMUX_GPIO_MASK) == AMD_IOMUX_ENABLE_FUNC0)
		mask[48] = AMD_GPIO_MODE_RESV;

	/* GPIO49 */
	iomux_reg = ioread8((u8 *)amd_gpio_chip.iomuxbase + 0x31);
	if (((iomux_reg & AMD_IOMUX_GPIO_MASK) == AMD_IOMUX_ENABLE_FUNC0) ||
	    ((iomux_reg & AMD_IOMUX_GPIO_MASK) == AMD_IOMUX_ENABLE_FUNC1))
		mask[49] = AMD_GPIO_MODE_RESV;

	/* GPIO50 */
	iomux_reg = ioread8((u8 *)amd_gpio_chip.iomuxbase + 0x32);
	if (((iomux_reg & AMD_IOMUX_GPIO_MASK) == AMD_IOMUX_ENABLE_FUNC0) ||
	    ((iomux_reg & AMD_IOMUX_GPIO_MASK) == AMD_IOMUX_ENABLE_FUNC1))
		mask[50] = AMD_GPIO_MODE_RESV;

	/* GPIO51 */
	iomux_reg = ioread8((u8 *)amd_gpio_chip.iomuxbase + 0x33);
	if (((iomux_reg & AMD_IOMUX_GPIO_MASK) == AMD_IOMUX_ENABLE_FUNC0) ||
	    ((iomux_reg & AMD_IOMUX_GPIO_MASK) == AMD_IOMUX_ENABLE_FUNC1))
		mask[51] = AMD_GPIO_MODE_RESV;

	/* GPIO52 */
	iomux_reg = ioread8((u8 *)amd_gpio_chip.iomuxbase + 0x34);
	if ((iomux_reg & AMD_IOMUX_GPIO_MASK) == AMD_IOMUX_ENABLE_FUNC0)
		mask[52] = AMD_GPIO_MODE_RESV;

	/* GPIO55 */
	iomux_reg = ioread8((u8 *)amd_gpio_chip.iomuxbase + 0x37);
	if (((iomux_reg & AMD_IOMUX_GPIO_MASK) == AMD_IOMUX_ENABLE_FUNC0) ||
	    ((iomux_reg & AMD_IOMUX_GPIO_MASK) == AMD_IOMUX_ENABLE_FUNC1))
		mask[55] = AMD_GPIO_MODE_RESV;

	/* GPIO56 */
	iomux_reg = ioread8((u8 *)amd_gpio_chip.iomuxbase + 0x38);
	if ((iomux_reg & AMD_IOMUX_GPIO_MASK) == AMD_IOMUX_ENABLE_FUNC0)
		mask[56] = AMD_GPIO_MODE_RESV;

	/* GPIO57 */
	iomux_reg = ioread8((u8 *)amd_gpio_chip.iomuxbase + 0x39);
	if ((iomux_reg & AMD_IOMUX_GPIO_MASK) == AMD_IOMUX_ENABLE_FUNC0)
		mask[57] = AMD_GPIO_MODE_RESV;

	/* GPIO58 */
	iomux_reg = ioread8((u8 *)amd_gpio_chip.iomuxbase + 0x3A);
	if ((iomux_reg & AMD_IOMUX_GPIO_MASK) == AMD_IOMUX_ENABLE_FUNC0)
		mask[58] = AMD_GPIO_MODE_RESV;

	/* GPIO59 */
	iomux_reg = ioread8((u8 *)amd_gpio_chip.iomuxbase + 0x3B);
	if (((iomux_reg & AMD_IOMUX_GPIO_MASK) == AMD_IOMUX_ENABLE_FUNC0) ||
	    ((iomux_reg & AMD_IOMUX_GPIO_MASK) == AMD_IOMUX_ENABLE_FUNC1))
		mask[59] = AMD_GPIO_MODE_RESV;

	/* GPIO60 */
	iomux_reg = ioread8((u8 *)amd_gpio_chip.iomuxbase + 0x3C);
	if ((iomux_reg & AMD_IOMUX_GPIO_MASK) == AMD_IOMUX_ENABLE_FUNC0)
		mask[60] = AMD_GPIO_MODE_RESV;

	/* GPIO61 */
	iomux_reg = ioread8((u8 *)amd_gpio_chip.iomuxbase + 0x3D);
	if ((iomux_reg & AMD_IOMUX_GPIO_MASK) == AMD_IOMUX_ENABLE_FUNC0)
		mask[61] = AMD_GPIO_MODE_RESV;

	/* GPIO62 */
	iomux_reg = ioread8((u8 *)amd_gpio_chip.iomuxbase + 0x3E);
	if ((iomux_reg & AMD_IOMUX_GPIO_MASK) == AMD_IOMUX_ENABLE_FUNC0)
		mask[62] = AMD_GPIO_MODE_RESV;

	/* GPIO63 */
	iomux_reg = ioread8((u8 *)amd_gpio_chip.iomuxbase + 0x3F);
	if ((iomux_reg & AMD_IOMUX_GPIO_MASK) == AMD_IOMUX_ENABLE_FUNC0)
		mask[63] = AMD_GPIO_MODE_RESV;

	/* GPIO64 */
	iomux_reg = ioread8((u8 *)amd_gpio_chip.iomuxbase + 0x40);
	if (((iomux_reg & AMD_IOMUX_GPIO_MASK) == AMD_IOMUX_ENABLE_FUNC0) ||
	    ((iomux_reg & AMD_IOMUX_GPIO_MASK) == AMD_IOMUX_ENABLE_FUNC1))
		mask[64] = AMD_GPIO_MODE_RESV;

	/* GPIO65 */
	iomux_reg = ioread8((u8 *)amd_gpio_chip.iomuxbase + 0x41);
	if ((iomux_reg & AMD_IOMUX_GPIO_MASK) == AMD_IOMUX_ENABLE_FUNC0)
		mask[65] = AMD_GPIO_MODE_RESV;

	/* GPIO66 */
	iomux_reg = ioread8((u8 *)amd_gpio_chip.iomuxbase + 0x42);
	if ((iomux_reg & AMD_IOMUX_GPIO_MASK) == AMD_IOMUX_ENABLE_FUNC0)
		mask[66] = AMD_GPIO_MODE_RESV;

	/* GPIO67 */
	iomux_reg = ioread8((u8 *)amd_gpio_chip.iomuxbase + 0x43);
	if ((iomux_reg & AMD_IOMUX_GPIO_MASK) == AMD_IOMUX_ENABLE_FUNC0)
		mask[67] = AMD_GPIO_MODE_RESV;

	/* GPIO166 */
	iomux_reg = ioread8((u8 *)amd_gpio_chip.iomuxbase + 0xA6);
	if ((iomux_reg & AMD_IOMUX_GPIO_MASK) == AMD_IOMUX_ENABLE_FUNC0)
		mask[166] = AMD_GPIO_MODE_RESV;

	/* GPIO167 */
	iomux_reg = ioread8((u8 *)amd_gpio_chip.iomuxbase + 0xA7);
	if ((iomux_reg & AMD_IOMUX_GPIO_MASK) == AMD_IOMUX_ENABLE_FUNC0)
		mask[167] = AMD_GPIO_MODE_RESV;

	/* GPIO168 */
	iomux_reg = ioread8((u8 *)amd_gpio_chip.iomuxbase + 0xA8);
	if ((iomux_reg & AMD_IOMUX_GPIO_MASK) == AMD_IOMUX_ENABLE_FUNC0)
		mask[168] = AMD_GPIO_MODE_RESV;

	/* GPIO169 */
	iomux_reg = ioread8((u8 *)amd_gpio_chip.iomuxbase + 0xA9);
	if ((iomux_reg & AMD_IOMUX_GPIO_MASK) == AMD_IOMUX_ENABLE_FUNC0)
		mask[169] = AMD_GPIO_MODE_RESV;

	/* GPIO170 */
	iomux_reg = ioread8((u8 *)amd_gpio_chip.iomuxbase + 0xAA);
	if ((iomux_reg & AMD_IOMUX_GPIO_MASK) == AMD_IOMUX_ENABLE_FUNC0)
		mask[170] = AMD_GPIO_MODE_RESV;

	/* GPIO184 */
	iomux_reg = ioread8((u8 *)amd_gpio_chip.iomuxbase + 0xB8);
	if ((iomux_reg & AMD_IOMUX_GPIO_MASK) == AMD_IOMUX_ENABLE_FUNC0)
		mask[184] = AMD_GPIO_MODE_RESV;
}

static int amd_gpio_init(struct platform_device *pdev)
{
	struct pci_dev *dev = NULL;
	u32 val;
	u8 *byte;
	int i;
	int err;

	/* Match the PCI device */
	for_each_pci_dev(dev) {
		if (pci_match_id(amd_gpio_pci_tbl, dev) != NULL) {
			amd_gpio_pci = dev;
			break;
		}
	}

	if (!amd_gpio_pci)
		return -ENODEV;

	/* Locate ACPI MMIO Base Address. */
	byte = (u8 *)&val;

	outb(AMD_PM_GPIO_BASE0, AMD_IO_PM_INDEX_REG);
	byte[0] = inb(AMD_IO_PM_DATA_REG);
	outb(AMD_PM_GPIO_BASE1, AMD_IO_PM_INDEX_REG);
	byte[1] = inb(AMD_IO_PM_DATA_REG);
	outb(AMD_PM_GPIO_BASE2, AMD_IO_PM_INDEX_REG);
	byte[2] = inb(AMD_IO_PM_DATA_REG);
	outb(AMD_PM_GPIO_BASE3, AMD_IO_PM_INDEX_REG);
	byte[3] = inb(AMD_IO_PM_DATA_REG);

	/* Bits 31:13 is the actual ACPI MMIO Base Address */
	val &= ~0x1FFF;

	/* GPIO Base Address starts from ACPI MMIO Base Address + 0x100 */
	if (!request_mem_region_exclusive(val + 0x100, AMD_GPIO_MEM_MAP_SIZE,
								"AMD GPIO")) {
		pr_err("mmio address 0x%04x already in use\n", val + 0x100);
		goto exit;
	}
	gpiobase_phys = val + 0x100;

	amd_gpio_chip.gpiobase = ioremap(gpiobase_phys, AMD_GPIO_MEM_MAP_SIZE);
	if (!amd_gpio_chip.gpiobase) {
		pr_err("failed to get gpiobase address\n");
		goto unreg_gpio_region;
	}

	/* IOMUX Base Address starts from ACPI MMIO Base Address + 0xD00 */
	if (!request_mem_region_exclusive(val + 0xD00, AMD_IOMUX_MEM_MAP_SIZE,
								"AMD IOMUX")) {
		pr_err("mmio address 0x%04x already in use\n", val + 0xD00);
		goto unmap_gpio_region;
	}
	iomuxbase_phys = val + 0xD00;

	amd_gpio_chip.iomuxbase = ioremap(iomuxbase_phys, AMD_IOMUX_MEM_MAP_SIZE);
	if (!amd_gpio_chip.iomuxbase) {
		pr_err("failed to get iomuxbase address\n");
		goto unreg_iomux_region;
	}

	/* Set up driver specific struct */
	amd_gpio_chip.pdev = pdev;
	spin_lock_init(&amd_gpio_chip.lock);

	/* Register ourself with the GPIO core */
	err = gpiochip_add(&amd_gpio_chip.gpio);
	if (err)
		goto unmap_iomux_region;

	/*
	 * Lets take care of special GPIO pins, and mark them as reserved
	 * as appropriate.
	 */
	amd_update_gpio_mask();

	/*
	 * If the number of GPIO pins provided during module loading does
	 * not match the number of GPIO modes, we fall back to the default
	 * mask.
	 */
	if (num_mask == num_modes) {
		/*
		 * If the number of masks or the number of modes specified
		 * is more than the maximum number of GPIO pins supported
		 * by the driver, we set the limit to the one supported
		 * driver.
		 */
		if (num_mask > AMD_GPIO_NUM_PINS)
			num_mask = num_modes = AMD_GPIO_NUM_PINS;

		/*
		 * The default mask is our de facto standard. The GPIO
		 * pins marked reserved in the default mask stay reserved
		 * no matter what the module load parameter says. Also, we
		 * set the mode of the GPIO pins depending on the value
		 * of gpio_mode provided.
		 */
		for (i = 0; i < num_mask; i++) {
			if (mask[gpio_mask[i]] != AMD_GPIO_MODE_RESV) {
				mask[gpio_mask[i]] = gpio_mode[i];

				/*
				 * gpio_request() can fail, in which case we
				 * won't set the GPIO modes.
				 */
				if(!gpio_request(gpio_mask[i], DRV_NAME)) {
					if (gpio_mode[i] == AMD_GPIO_MODE_INPUT)
						gpio_direction_input(gpio_mask[i]);
					else if (gpio_mode[i] == AMD_GPIO_MODE_OUTPUT)
						gpio_direction_output(gpio_mask[i], 0);

					gpio_free(gpio_mask[i]);
				}
			}
		}
	}

	return 0;

unmap_iomux_region:
	iounmap(amd_gpio_chip.iomuxbase);
unreg_iomux_region:
	release_mem_region(iomuxbase_phys, AMD_IOMUX_MEM_MAP_SIZE);
unmap_gpio_region:
	iounmap(amd_gpio_chip.gpiobase);
unreg_gpio_region:
	release_mem_region(gpiobase_phys, AMD_GPIO_MEM_MAP_SIZE);
exit:
	return -ENODEV;
}

static int amd_gpio_remove(struct platform_device *pdev)
{
	/* gpiochip_remove now returns NULL
	int err;

	err = gpiochip_remove(&amd_gpio_chip.gpio);
	if (err) {
		dev_err(&pdev->dev, "Unable to remove gpio chip\n");
		return err;
	}
	*/

	gpiochip_remove(&amd_gpio_chip.gpio);
	iounmap(amd_gpio_chip.iomuxbase);
	release_mem_region(iomuxbase_phys, AMD_IOMUX_MEM_MAP_SIZE);
	iounmap(amd_gpio_chip.gpiobase);
	release_mem_region(gpiobase_phys, AMD_GPIO_MEM_MAP_SIZE);

	return 0;
}

static struct platform_driver amd_gpio_driver = {
	.probe		= amd_gpio_init,
	.remove		= amd_gpio_remove,
	.driver		= {
		.owner	= THIS_MODULE,
		.name	= GPIO_MODULE_NAME,
	},
};

static int __init amd_gpio_init_module(void)
{
	int err;

	pr_info("AMD GPIO Driver v%s\n", GPIO_VERSION);

	err = platform_driver_register(&amd_gpio_driver);
	if (err)
		return err;

	amd_gpio_platform_device = platform_device_register_simple(
					GPIO_MODULE_NAME, -1, NULL, 0);
	if (IS_ERR(amd_gpio_platform_device)) {
		err = PTR_ERR(amd_gpio_platform_device);
		goto unreg_platform_driver;
	}

	return 0;

unreg_platform_driver:
	platform_driver_unregister(&amd_gpio_driver);
	return err;
}

static void __exit amd_gpio_cleanup_module(void)
{
	platform_device_unregister(amd_gpio_platform_device);
	platform_driver_unregister(&amd_gpio_driver);
	pr_info("AMD GPIO Module Unloaded\n");
}

module_init(amd_gpio_init_module);
module_exit(amd_gpio_cleanup_module);

MODULE_AUTHOR("Arindam Nath <arindam.nath@amd.com>");
MODULE_DESCRIPTION("GPIO driver for AMD chipsets");
MODULE_LICENSE("Dual BSD/GPL");
