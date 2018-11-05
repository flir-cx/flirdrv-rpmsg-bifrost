/*
 * Copyright (c) FLIR Systems AB.
 *
 * bifrost_main.c
 *
 *  Created on: Mar 1, 2010
 *      Author: Jonas Romfelt <jonas.romfelt@flir.se>
 *
 * Main driver file that implements entry and exit points.
 *
 */

#include <linux/kernel.h>
#include <linux/moduleparam.h>
#include <linux/stat.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/poll.h>
#include <linux/errno.h>
#include <linux/mm.h>
#include <linux/interrupt.h>
#include <linux/time.h>

#include <asm/uaccess.h>
#include <asm/byteorder.h>
#include <asm/atomic.h>

#include "bifrost.h"

struct bifrost_device *bdev;

#include <linux/mempool.h>
#include <linux/slab.h>
#include <linux/workqueue.h>

struct bifrost_work {
        struct work_struct work;
        struct bifrost_device *dev;
        struct bifrost_event event;
};

static mempool_t *work_pool;
static struct workqueue_struct *work_queue;

static void *mempool_alloc_work(gfp_t flags, void *pool_data)
{
        (void)pool_data;
        return kmalloc(sizeof(struct bifrost_work), flags);
}

static void mempool_free_work(void *work, void *pool_data)
{
        (void)pool_data;
        kfree(work);
}

static void work_create_event(struct work_struct *work)
{
        struct bifrost_work *w;

        w = container_of(work, struct bifrost_work, work);
        bifrost_create_event(w->dev, &w->event);
        mempool_free(w, work_pool);
}

void bifrost_create_event_in_atomic(struct bifrost_device *dev,
                                    struct bifrost_event *event)
{
        struct bifrost_work *w;

        w = mempool_alloc(work_pool, GFP_ATOMIC);
        if (w == NULL) {
                ALERT("dropped event type=%d", event->type);
                return;
        }

        INIT_WORK(&w->work, work_create_event);
        w->dev = dev;
        memcpy(&w->event, event, sizeof(*event));
        queue_work(work_queue, &w->work);
}

/*
 * Entry point to driver.
 */
static int __init bifrost_init(void)
{
        if ((bdev = kzalloc(sizeof(struct bifrost_device), GFP_KERNEL)) == NULL) {
                ALERT("failed to allocate BIFROST_DEVICE\n");
                return -ENOMEM;
        }

        bdev->info.version.major = BIFROST_VERSION_MAJOR;
        bdev->info.version.minor = BIFROST_VERSION_MINOR;
        bdev->info.version.revision = BIFROST_VERSION_MICRO;

        INIT_LIST_HEAD(&bdev->list);
        spin_lock_init(&bdev->lock_list);

        work_pool = mempool_create(20, mempool_alloc_work, mempool_free_work,
                                   NULL);
        if (work_pool == NULL)
                return -ENOMEM;

        work_queue = create_singlethread_workqueue("bifrost");
        if (work_queue == NULL) {
                mempool_destroy(work_pool);
                return -ENOMEM;
        }

        /*
         * Setup /proc file system entry. Create an read-only entry in
         * proc root with a file name same as the device (use NULL as
         * parent dir)
         */

	if (bifrost_rpmsg_init(bdev) != 0)
	{
	    bifrost_rpmsg_exit(bdev);
	    goto err_pci;
	}

        INFO("init done\n");
        return 0;

  err_pci:
        remove_proc_entry(BIFROST_DEVICE_NAME, NULL);
        flush_workqueue(work_queue);
        destroy_workqueue(work_queue);
        mempool_destroy(work_pool);
        kfree(bdev);
        ALERT("init failed\n");
        return -1;
}

/**
 * Exit point from driver.
 */
static void __exit bifrost_exit(void)
{
        INFO("exit\n");

        bifrost_cdev_exit(bdev);

        bifrost_rpmsg_exit(bdev);

        flush_workqueue(work_queue);
        destroy_workqueue(work_queue);
        mempool_destroy(work_pool);
        kfree(bdev);
}

/*
 * Specify kernel module enter and exit points
 */

module_init(bifrost_init);
module_exit(bifrost_exit);

/*
 * Specify kernel module info
 */

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Peter Fitger <peter.fitger@flir.se>");
MODULE_DESCRIPTION("FLIR FPGA driver");
