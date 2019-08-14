/*
 * Copyright (c) FLIR Systems AB.
 *
 * bifrost_rpmsg.c
 *
 *  Created on: June 26, 2018
 *      Author: Peter Fitger <peter.fitger@flir.se>
 *
 * RPMSG bus parts of driver.
 *
 */

#include <linux/dma-mapping.h>
#include <linux/module.h>
#include <linux/pagemap.h>
#include <linux/platform_device.h>
#include <linux/rpmsg.h>

#include "bifrost.h"

extern struct bifrost_device *bdev;

irqreturn_t FVDIRQ1Service(int irq, void *dev_id);
irqreturn_t FVDIRQ2Service(int irq, void *dev_id);

#define FPGA_IRQ_0      ((3-1)*32 + 16)
#define FPGA_IRQ_1      ((3-1)*32 + 17) // GPIO3.17
#define FPGA_IRQ_2      ((3-1)*32 + 18) // GPIO3.18

#define FRAME_GRAB_IMAGE_SIZE (160*120*2)

enum msg_type {
	REG_M4_WRITE,	// To M4, reg+value
	REG_M4_READ,	// To M4, reg, From M4 reg+value
	M4_EXEC_IRQ,	// From M4, value = irq status
	M4_HW_IRQ,	// From M4, value = irq num
};

struct m4_msg {
	uint32_t type;
	uint32_t reg;
	uint32_t value;
};

static uint32_t recv_reg;
static uint32_t recv_value;

int rpmsg_write_device_memory(void *handle, u32 offset, u32 value)
{
	struct m4_msg msg;

	msg.type = REG_M4_WRITE;
	msg.reg = offset;
	msg.value = value;

	return rpmsg_send(bdev->rpmsg_dev->ept, &msg, sizeof(msg));
}

int rpmsg_read_device_memory(void *handle, u32 offset, u32 *value)
{
	struct m4_msg msg;

	msg.type = REG_M4_READ;
	msg.reg = offset;
	msg.value = *value;
	recv_reg = 0;

	if (bdev->rpmsg_dev && bdev->rpmsg_dev->ept)
		rpmsg_send(bdev->rpmsg_dev->ept, &msg, sizeof(msg));
	else
		pr_info("Endpoint missing %p %p!\n", bdev, bdev->rpmsg_dev);

	wait_for_completion_timeout(&bdev->completion, msecs_to_jiffies(1000));

	if (recv_reg != offset) {
		ALERT("!!! Register mismatch expected 0x%X, got reply from 0x%X\n", offset, recv_reg);
		return -EIO;
	}
	*value = recv_value;

	return 0;
}

static void init_device_memory(int n, struct device_memory *mem)
{
	memset(mem, 0, sizeof(*mem));

	mem->bar = n;
	mem->addr_bus = 0;
	mem->flags = 0;
	mem->size = SZ_4K;
	mem->handle = mem;
	mem->rd = rpmsg_read_device_memory;
	mem->wr = rpmsg_write_device_memory;
	mutex_init(&mem->lock);
}

static int map_device_memory(struct device_memory *mem)
{
	if (mem->size == 0)
		return -EINVAL; /* Nothing to I/O map */
	if (mem->bar >= 2)
		return -EINVAL; /* Only two chip selects active */

	mem->enabled = 1;

	return 0;
}

static void unmap_device_memory(struct device_memory *mem)
{
	mem->enabled = 0;
}

static int setup_io_regions(struct bifrost_device *bifrost)
{
	int n, nbars;

	for (nbars = 0, n = 0; n < ARRAY_SIZE(bifrost->regb); n++) {
		init_device_memory(n, &bifrost->regb[n]);
		if (map_device_memory(&bifrost->regb[n]) == 0)
			nbars++;
	}

	return nbars;
}

static void remove_io_regions(struct bifrost_device *bifrost)
{
	int n;

	for (n = 0; n < ARRAY_SIZE(bifrost->regb); n++)
		unmap_device_memory(&bifrost->regb[n]);
}


/**
 * Handle received RPMSG
 */

static int rpmsg_bifrost_callback(struct rpmsg_device *dev, void *data, int len,
		void *priv, u32 src)
{
	struct m4_msg *msg = data;

	switch (msg->type) {
		case REG_M4_READ:
			recv_reg = msg->reg;
			recv_value = msg->value;
			complete(&bdev->completion);
			break;

		case M4_HW_IRQ:
			break;

		case M4_EXEC_IRQ:
			{
				struct bifrost_event event;
				memset(&event, 0, sizeof(event));
				event.type = BIFROST_EVENT_TYPE_IRQ;
				event.data.irq_source = 1;
				event.data.irqstatus.value = msg->value;
				bifrost_create_event_in_atomic(bdev, &event);
			}
			break;

		default:
			break;
	}

	return 0;
}

/**
 * Set up Lepton-M4 interface
 */

static int rpmsg_bifrost_probe(struct rpmsg_device *dev)
{
	struct m4_msg msg;
	void *vaddr = NULL;
	dma_addr_t paddr;

	dev_info(&dev->dev, "new channel: 0x%x -> 0x%x!\n", dev->src, dev->dst);

	bdev->rpmsg_dev = dev;
	vaddr = dma_alloc_coherent(NULL, PAGE_ALIGN(FRAME_GRAB_IMAGE_SIZE*2), &paddr, GFP_DMA | GFP_KERNEL);

	msg.type = REG_M4_WRITE;
	msg.reg = 0x33;
	msg.value = paddr;

	return rpmsg_send(dev->ept, &msg, sizeof(msg));
}

static int rpmsg_bifrost_suspend(struct device *dev)
{
	pr_info("rpmsg_bifrost_suspend\n");

	rpmsg_write_device_memory(NULL, 0x55, 0xAAAA);

	wait_for_completion_timeout(&bdev->completion, msecs_to_jiffies(1000));

	return 0;
}

static int rpmsg_bifrost_resume(struct device *dev)
{
	pr_info("rpmsg_bifrost_resume\n");

	return rpmsg_write_device_memory(NULL, 0x55, 0x5555);
}

static void rpmsg_bifrost_shutdown(struct device *dev)
{
	rpmsg_bifrost_suspend(dev);
}

static struct rpmsg_device_id rpmsg_driver_bifrost_id_table[] = {
	{ .name	= "rpmsg-client-bifrost" },
	{ },
};
MODULE_DEVICE_TABLE(rpmsg, rpmsg_driver_bifrost_id_table);

static SIMPLE_DEV_PM_OPS(rpmsg_bifrost_pm, rpmsg_bifrost_suspend, rpmsg_bifrost_resume);

static struct rpmsg_driver rpmsg_bifrost_client = {
	.drv = {
		.name	= KBUILD_MODNAME,
		.pm	= &rpmsg_bifrost_pm,
		.shutdown	= rpmsg_bifrost_shutdown,
	},
	.id_table	= rpmsg_driver_bifrost_id_table,
	.probe		= rpmsg_bifrost_probe,
	.callback	= rpmsg_bifrost_callback,
};

int __init bifrost_rpmsg_init(struct bifrost_device *dev)
{
	int rc;

	rc = register_rpmsg_driver(&rpmsg_bifrost_client);
	if (rc)
		return rc;

	/* Gain CPU access to FPGA register map */
	rc = setup_io_regions(dev);
	if (rc < 2)
		goto err_pci_iomap_regb;

	/* Run post init and make driver accessible */
	if (bifrost_cdev_init(dev) != 0)
		goto err_pci_iomap_regb;

	if (bifrost_fvd_init(dev) != 0)
		goto err_pci_iomap_regb;

	init_completion(&bdev->completion);

	return 0;

	/* stack-like cleanup on error */
err_pci_iomap_regb:
	remove_io_regions(dev);
	return -ENODEV;
}

int  bifrost_fvd_init(struct bifrost_device *dev)
{
	// Register platform driver
	dev->pMemDev = platform_device_alloc(BIFROST_DEVICE_NAME, 1);
	if (dev->pMemDev == NULL) {
		ALERT("Registering rpmsg driver failed\n");
		goto err_platform_alloc;
	}
	platform_device_add(dev->pMemDev);

	dev->pClass = class_create(THIS_MODULE, BIFROST_DEVICE_NAME);
	device_create(dev->pClass, NULL, dev->cdev.dev, NULL, "bif0");

	return 0;

err_platform_alloc:
	return -ENODEV;

}

void bifrost_fvd_exit(struct bifrost_device *dev)
{
	INFO("\n");

	device_destroy(dev->pClass, dev->cdev.dev);
	class_destroy(dev->pClass);
	platform_device_unregister(dev->pMemDev);
}

void bifrost_rpmsg_exit(struct bifrost_device *dev)
{
	INFO("\n");

	remove_io_regions(dev);
	bifrost_fvd_exit(dev);
}

////////////////////////////////////////////////////////
//
// WriteSDRAM
//
////////////////////////////////////////////////////////
static void WriteSDRAM(struct bifrost_device *dev,
		u32 pSrc,
		u32 addr,    // SDRAM byte offset
		u32 sz)  // Number of bytes to write
{
	memcpy((void *)addr, (void *)pSrc, sz);
}

////////////////////////////////////////////////////////
//
// ReadSDRAM
//
////////////////////////////////////////////////////////
static void ReadSDRAM(struct bifrost_device *dev,
		u32 pDst,
		u32 addr,  // SDRAM byte offset
		u32 sz)    // Number of bytes to read
{
	memcpy((void *)pDst, (void *)addr, sz);
}

int do_rpmsg_xfer(struct bifrost_device *dev,
		struct bifrost_dma_transfer *xfer,
		int up_down)
{
	switch (up_down) {
		case BIFROST_DMA_DIRECTION_DOWN: /* system memory -> FPGA memory */
			WriteSDRAM(dev, xfer->system, xfer->device, xfer->size);
			break;
		case BIFROST_DMA_DIRECTION_UP: /* FPGA memory -> system memory */
			ReadSDRAM(dev, xfer->system, xfer->device, xfer->size);
			break;
		default:
			return -EINVAL;
	}
	return 0;
}

/////////////////////////////////////////////////
// This is the interrupt service thread
/////////////////////////////////////////////////
irqreturn_t FVDInterruptService(int irq, void *dev_id)
{
	struct bifrost_device *dev = (struct bifrost_device *)dev_id;
	struct bifrost_event event;
	u32 exec_status;

	INFO("Irq %d\n", irq);
	memset(&event, 0, sizeof(event));

	// Clear interrupt by reading Execute Status Register
	rpmsg_read_device_memory(dev->regb[0].handle, 0x1C, &exec_status);

	// Indicate completion
	event.type = BIFROST_EVENT_TYPE_IRQ;
	event.data.irq_source = 1;
	event.data.irqstatus.value = exec_status;
	bifrost_create_event_in_atomic(dev, &event);

	return IRQ_HANDLED;
}

//////////////////////////////////////////////////////////
// This is the interrupt service thread for HostIntr_n(1)
//////////////////////////////////////////////////////////
irqreturn_t FVDIRQ1Service(int irq, void *dev_id)
{
	struct bifrost_device *dev = (struct bifrost_device *)dev_id;
	struct bifrost_event event;
	u32 vector, mask;

	memset(&event, 0, sizeof(event));

	INFO("Irq1 %d\n", irq);

	// Read interrupt mask
	rpmsg_read_device_memory(dev->regb[0].handle, 0x38, &mask);

	// Read interrupt vector
	rpmsg_read_device_memory(dev->regb[0].handle, 0x37, &vector);

	return IRQ_HANDLED;
}
//////////////////////////////////////////////////////////
// This is the interrupt service thread for HostIntr_n(2)
//////////////////////////////////////////////////////////
irqreturn_t FVDIRQ2Service(int irq, void *dev_id)
{
	struct bifrost_device *dev = (struct bifrost_device *)dev_id;
	struct bifrost_event event;

	memset(&event, 0, sizeof(event));

	// Indicate completion
	event.type = BIFROST_EVENT_TYPE_IRQ;
	event.data.irq_source = 0x20;
	getnstimeofday(&event.data.frame.time);
	bifrost_create_event_in_atomic(dev, &event);

	return IRQ_HANDLED;
}


