/*
 * Copyright (c) FLIR Systems AB.
 *
 * bifrost_api.h
 *
 *  Created on: Mar 1, 2010
 *      Author: Jonas Romfelt <jonas.romfelt@flir.se>
 *
 * Public Bifrost driver interface
 *
 */

#ifndef BIFROST_API_H_
#define BIFROST_API_H_

#ifdef __KERNEL__
#include <linux/time.h>
#else
#include <sys/time.h>
#endif
#include <linux/ioctl.h>
#include <linux/types.h>

#define BIFROST_IOC_MAGIC 'B'

struct bifrost_info {
        struct {
                __u8 major;
                __u8 minor;
                __u8 revision;
        } version;
        __u8 simulator; /* Set to 0 if simulator interface disabled */

        /*
         * TODO:
         * - add stats here?
         * - average IRQ servicing time?
         */
};

struct bifrost_access {
        __u32 bar;    /* BAR number */
        __u32 offset; /* Register offset (within a BAR) */
        __u32 value;  /* Value to read/write */
};

struct bifrost_access_range {
        __u32 bar;    /* BAR number */
        __u32 offset; /* Register offset (within a BAR) */
        unsigned long values; /* User pointer to memory that can hold at least
                                   `count' number of u32:s to read/write */
        __u32 count;  /* Number of registers to access */  
};

struct bifrost_modify {
        __u32 bar;    /* BAR number */
        __u32 offset; /* Register offset */
        __u32 value;  /* New modified value */
        __u32 clear;  /* Clear these bits */
        __u32 set;    /* Set these bits (higher prio than clear) */
};

struct bifrost_dma_transfer {
        /*
         * System (CPU) memory address.
         *
         * IMPORTANT! In simulator mode the system address should be
         * an address in kernel virtual memory, otherwise it is expected to
         * be a physical address in DMA'able memory.
         */
        unsigned long system;
        __u32 device; /* Device (e.g. FPGA) memory offset. */
        __u32 size;   /* Size of transfer in bytes. */
};
#define BIFROST_DMA_USER_BUFFER       (1<<0)   //buffer is allocated in user space, physical Non-Contiguous


#define BIFROST_EVENT_TYPE_IRQ        (1 << 0)
#define BIFROST_EVENT_TYPE_WRITE_REGB (1 << 1)
#define BIFROST_EVENT_TYPE_READ_REGB  (1 << 2)
#define BIFROST_EVENT_TYPE_DMA_DONE   (1 << 3)

struct bifrost_dma {
        __u32 id;
        __s64 time;
        __u64 cookie;
};

/*
 * Used in bifrost_membus.c
 * This struct is returned when FPGA has JPEGLS Frame to deliver
 * irq_source = 4
 */
struct bifrost_membus_frame {
        __u32 irq_source;      /* irq_source, note this is needed so we match
                                    the event struct. */
        __u32 frameNo;         /* Buffer number where we can find the frame. */
        __u32 frameSize;       /* Size of the compressed frame. */
        struct timespec time;  /* time stamp when we got the irq. */
};

/*
 * Used in bifrost_membus.c
 * When irq_source = 1: (FPGA Execute IRQ)
 * This struct is sent when FPGA generates an
 * "execute finished" IRQ.
 * 'value' : now contains irq status (reg: 0x1c).
 *
 * When irq_source = 0x08  (DIO)
 * This struct is sent when digital IO is changed
 * 'value' : The digital IO state.
 *
 * When irq_source = 0x10 (HSI cable):
 * This struct is sent when HSI cable is inserted/removed
 * 'value' : now contains cable state.
 *     0: cable disconnected.
 *     1: cable connected (link is up).
 */
struct bifrost_membus_irqstatus {
        __u32 irq_source;    /* irq_source, note this is needed so we match the
                                  event struct. */
        __u32 value;         /* FPGA irq status */
};

/*
 * When using membus irq, the following irq_sources are defined:
 * 0x01: Execute interrupt
 * 0x02: HSI (BOB) interrupt
 * 0x04: JPEGLS frame interrupt
 * 0x08: Digital IO
 * 0x10: HSI cable in/out.
 */

struct bifrost_event {
        __u32 type;
        union {
                __u32 irq_source;
                struct bifrost_access regb_access;
                struct bifrost_dma dma;
                struct bifrost_membus_irqstatus irqstatus;
                struct bifrost_membus_frame frame;
        } data;
        struct {
                struct timeval received;
                struct timeval forwarded;
        } timestamp;
};

struct bifrost_simulator_memory {
        unsigned long address; /* User space address to malloc'd memory. */
        __u32 size;            /* Size of memory in bytes. */
};

struct bifrost_simulator_memory_transfer {
        unsigned long system;    /* Kernel virtual address */
        unsigned long simulator; /* User space address to malloc'd memory. */
        int to;                  /* If to is set data is copied TO simulator
                                  * FROM system, i.e. down stream. */
        __u32 size;              /* Size of transfer in bytes. */
};

/*
 * Bifrost ioctls
 */

#define BIFROST_IOCTL_INFO \
        _IOR(BIFROST_IOC_MAGIC, 0, struct bifrost_info)

/* Read FPGA register */
#define BIFROST_IOCTL_READ_REGB \
        _IOWR(BIFROST_IOC_MAGIC, 1, struct bifrost_access)

/* Read multiple FPGA registers */
#define BIFROST_IOCTL_READ_RANGE_REGB \
        _IOWR(BIFROST_IOC_MAGIC, 4, struct bifrost_access_range)

/* Read same FPGA register multiple times */
#define BIFROST_IOCTL_READ_REPEAT_REGB \
        _IOWR(BIFROST_IOC_MAGIC, 5, struct bifrost_access_range)

/* Write FPGA register */
#define BIFROST_IOCTL_WRITE_REGB \
        _IOW(BIFROST_IOC_MAGIC, 2, struct bifrost_access)

/* Write same FPGA register multiple times */
#define BIFROST_IOCTL_WRITE_REPEAT_REGB \
        _IOWR(BIFROST_IOC_MAGIC, 6, struct bifrost_access_range)

/*
 * Modify FPGA register, i.e. read register, clear bits, set bits and
 * write new value to register
 */
#define BIFROST_IOCTL_MODIFY_REGB \
        _IOWR(BIFROST_IOC_MAGIC, 3, struct bifrost_modify)

/* Start up-stream DMA transfer (to CPU) */
#define BIFROST_IOCTL_START_DMA_UP \
        _IOW(BIFROST_IOC_MAGIC, 10, struct bifrost_dma_transfer)

/* Start down-stream DMA transfer (from CPU) */
#define BIFROST_IOCTL_START_DMA_DOWN \
        _IOW(BIFROST_IOC_MAGIC, 11, struct bifrost_dma_transfer)


/* Start down-stream DMA transfer (from CPU), extra flag for buffer allocated in user space */
#define BIFROST_IOCTL_START_DMA_DOWN_USER \
        _IOW(BIFROST_IOC_MAGIC, 12, struct bifrost_dma_transfer)

/* Start up-stream DMA transfer (to CPU),     extra flag for buffer allocated in user space */
#define BIFROST_IOCTL_START_DMA_UP_USER \
        _IOW(BIFROST_IOC_MAGIC, 13, struct bifrost_dma_transfer)



/*
 * Bifrost Events
 */

/* Set event enable mask */
#define BIFROST_IOCTL_ENABLE_EVENT \
        _IOW(BIFROST_IOC_MAGIC, 20, __u32)

/*
 * Set IRQ forwarding mask, takes an interrupt mask of irqs to forward
 * to user space via events
 */
#define BIFROST_IOCTL_IRQ_FORWARDING \
        _IOW(BIFROST_IOC_MAGIC, 21, __u32)

/* Dequeue and read event from event-queue */
#define BIFROST_IOCTL_DEQUEUE_EVENT \
        _IOR(BIFROST_IOC_MAGIC, 22, struct bifrost_event)

/*
 * Bifrost Simulator
 */

/*
 * Setup simulated memory, client should supply size of the allocated
 * memory
 */
#define BIFROST_IOCTL_SETUP_SIMULATOR_MEMORY \
        _IOW(BIFROST_IOC_MAGIC, 50, struct bifrost_simulator_memory)

/*
 * Simulate a DMA transfer, i.e. copy data to/from simulated device
 * memory and system DMA'able buffer
 */
#define BIFROST_IOCTL_SIMULATE_DMA_TRANSFER \
        _IOW(BIFROST_IOC_MAGIC, 51, struct bifrost_simulator_memory_transfer)

/* Simulate IRQ, only available when driver loaded with test enabled */
#define BIFROST_IOCTL_SIMULATE_IRQ \
        _IOW(BIFROST_IOC_MAGIC, 52, __u32)

/*
 * By default, all registers are read/writable and does not trigger
 * any events.
 */
#define BIFROST_REGB_MODE_READABLE    (1 << 0) /* register is readable */
#define BIFROST_REGB_MODE_WRITABLE    (1 << 1) /* register is writable */
#define BIFROST_REGB_MODE_WRITE_EVENT (1 << 2) /* event on write */
#define BIFROST_REGB_MODE_READ_EVENT  (1 << 3) /* event on read */

/* Set simulated register mode flags mask, with access.value set to mode */
#define BIFROST_IOCTL_SET_REGB_MODE \
        _IOW(BIFROST_IOC_MAGIC, 52, struct bifrost_access)

#define BIFROST_IOCTL_RESET_DMA \
        _IO(BIFROST_IOC_MAGIC, 53)

#endif /* BIFROST_API_H_ */
