/*
 * Copyright (c) FLIR Systems AB.
 *
 * bifrost.h
 *
 *  Created on: Mar 1, 2010
 *      Author: Jonas Romfelt <jonas.romfelt@flir.se>
 *
 * Internal header.
 *
 */

#ifndef BIFROST_H_
#define BIFROST_H_

#include <linux/cdev.h>
#include <linux/fs.h>
#include <linux/list.h>
#include <linux/sched.h>
#include <linux/spinlock.h>
#include <linux/mutex.h>
#include <linux/timer.h>
#include <linux/poll.h>
#include <linux/proc_fs.h>
#include <linux/version.h>
#include <linux/interrupt.h>

#include "bifrost_api.h"
#include <linux/slab.h>
/*
 * Define driver information (displayed using modinfo)
 */

#define BIFROST_DEVICE_NAME "bifrost"

#define BIFROST_VERSION_MAJOR 0
#define BIFROST_VERSION_MINOR 1
#define BIFROST_VERSION_MICRO 2
#define BIFROST_VERSION_APPEND_STR "-alpha"
#define BIFROST_VERSION_NUMBER ((BIFROST_VERSION_MAJOR * 10000) + \
                                (BIFROST_VERSION_MINOR*100) + \
                                (BIFROST_VERSION_MICRO))

/*
 * Misc
 */
#define BIFROST_EVENT_BUFFER_SIZE 20

#define DMA_BUSY_BIT 0
#define CIRCULAR_BUFFER_SIZE 10

#define BIFROST_DMA_DIRECTION_UP 0 /* up-stream: FPGA-RAM -> CPU-RAM */
#define BIFROST_DMA_DIRECTION_DOWN 1 /* down-stream: CPU-RAM -> FPGA-RAM */


#ifndef VM_RESERVED
#define VM_RESERVED (VM_DONTEXPAND | VM_DONTDUMP)
#endif

/*
 * Debug macros
 */
#define BIFROST_DEBUG 1
#ifndef BIFROST_DEBUG
  #define INFO(fmt, args...)
  #define NOTICE(fmt, args...)
#else
  #define INFO(fmt, args...) printk(KERN_INFO "%s %s:%d> " fmt, \
                                    BIFROST_DEVICE_NAME, __FUNCTION__, \
                                    __LINE__, ##args)
  #define NOTICE(fmt, args...) printk(KERN_NOTICE "%s %s:%d> " fmt, \
                                      BIFROST_DEVICE_NAME, __FUNCTION__, \
                                      __LINE__, ##args)
#endif
#define ALERT(fmt, args...) printk(KERN_ALERT "%s %s:%d> " fmt, \
                                   BIFROST_DEVICE_NAME, __FUNCTION__, \
                                   __LINE__, ##args)

/*
 * Driver statistics
 */
struct bifrost_stats {
        bool enabled;

        /* file operations */
        unsigned long opens;
        unsigned long releases;
        unsigned long polls;
        unsigned long reads;
        unsigned long writes;
        unsigned long ioctls;
        unsigned long llseeks;
        unsigned long procfsreads;

        /* data throughput */
        unsigned long read_buf;         /* number of buffers read from driver */
        unsigned long write_buf;        /* number of buffers written to driver */
        unsigned long read_b;           /* number of bytes read from driver */
        unsigned long write_b;          /* number of bytes written to driver */
        unsigned long write_speed_avg;  /* write speed average in kB/s */
        unsigned long write_speed_last; /* write speed for last access in kB/s */
        unsigned long read_speed_avg;   /* read speed average in kB/s */
        unsigned long read_speed_last;  /* read speed for last access in kB/s  */
};

/*
 * place holder for timers
 */
struct timers {
        struct timer_list debug;        /* periodic debug timer */
};

/*
 * Device memory representation, used when memory-mapping a PCI BAR to CPU
 * address space, making memory CPU accessible via the readb(), readw(),
 * readl(), writeb() ... functions.
 */
struct device_memory {
        int bar;                /* BAR number */
        int enabled;            /* Memory successfully mapped */
        unsigned long addr_bus; /* PCIe bus address */
        unsigned long size;     /* Length of memory */
        unsigned long flags;
        void __iomem *addr;     /* Kernel logical address */
        struct pci_dev *pdev;   /* Handle to PCI device struct */
        struct mutex lock;        /* Use this to make atomic changes */

        /* Access functions */
        void *handle;
        int (*wr)(void *handle, u32 offset, u32 value);
        int (*rd)(void *handle, u32 offset, u32 *value);
};

/*
 * Bifrost device representation
 */
struct bifrost_device {
        struct bifrost_info info;
        struct list_head list;          /* list of user handles open to Bifrost */
        spinlock_t lock_list;
        struct bifrost_stats stats;     /* driver statistics */
        int cdev_initialized;           /* set when cdev has been initialized */
        struct cdev cdev;               /* char device structure */
        struct proc_dir_entry *proc;    /* proc fs entry */
        struct pci_dev *pdev;           /* PCI device structure */
        int irq;                        /* PCIe MSI interrupt line */
        struct timers timers;           /* timers */
        struct device_memory regb[6];   /* FPGA register bank (PCIe => max 6 BARs) */
        struct device_memory* regb_dma;   /* BAR used for DMA registers*/
        struct device_memory ddr;       /* FPGA DDR memory */

        /* rpmsg addons */
	struct rpmsg_device *rpmsg_dev;
        struct platform_device * pMemDev;
        struct class *pClass;
        struct completion completion;
};

/*
 * The bifrost_event struct is exported to user-space, so we need
 * a container for it in kernel-space so we don't pollute it
 */
struct bifrost_event_cont {
        struct list_head node;
        struct bifrost_event event;
};

/*
 * User space handle, i.e. someone that have called open(). Allow driver to be opened
 * by multiple users!
 */
struct bifrost_user_handle {
        struct list_head node;            /* list of user handles open to Bifrost */
        struct bifrost_device *dev;
        wait_queue_head_t waitq;          /* wait queue used by poll */
        u32 event_enable_mask;
        u32 irq_forwarding_mask;
        atomic_t use_count;

        struct list_head event_list;
        spinlock_t event_list_lock;
        unsigned int event_list_count;
};

int bifrost_cdev_init(struct bifrost_device *dev);
void bifrost_cdev_exit(struct bifrost_device *dev);

void bifrost_create_event(struct bifrost_device *dev,
                          struct bifrost_event *event);
void bifrost_create_event_in_atomic(struct bifrost_device *dev,
                                    struct bifrost_event *event);

int bifrost_attach_msis_to_irq(int hw_irq, struct bifrost_device *dev);
void bifrost_detach_msis(void);
int bifrost_simulate_msi(unsigned int msi);
int bifrost_dma_init(int hw_irq, struct bifrost_device *dev);
void bifrost_dma_cleanup(struct bifrost_device *dev);

int bifrost_rpmsg_init(struct bifrost_device *dev);
void bifrost_rpmsg_exit(struct bifrost_device *dev);
int do_rpmsg_xfer(struct bifrost_device *dev, struct bifrost_dma_transfer *xfer, int up_down);
irqreturn_t FVDInterruptService(int irq, void *dev_id);
int  bifrost_fvd_init(struct bifrost_device *dev);
void bifrost_fvd_exit(struct bifrost_device *dev);
int rpmsg_write_device_memory(void *handle, u32 offset, u32 value);
int rpmsg_read_device_memory(void *handle, u32 offset, u32 *value);


#endif /* BIFROST_H_ */
