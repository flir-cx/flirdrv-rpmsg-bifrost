
# typically use the following to compile
# make ARCH=arm CROSS_COMPILE=/home/fredrik/mentor/arm-2011.03/bin/arm-none-linux-gnueabi
#
# Also modify 'KERNELDIR' to fit your system

ifneq ($(KERNEL_PATH),)
       KERNEL_SRC = $(KERNEL_PATH)
endif

EXTRA_CFLAGS = -Werror

	obj-m := bifrost.o
	bifrost-objs += bifrost_cdev.o
	bifrost-objs += bifrost_main.o
	bifrost-objs += bifrost_rpmsg.o
	PWD := $(shell pwd)

all: 
	$(MAKE) -C $(KERNEL_SRC) M=$(PWD) modules

modules_install:
	$(MAKE) -C $(KERNEL_SRC) M=$(PWD) modules_install

clean:
	$(MAKE) -C $(KERNEL_SRC) M=$(PWD) clean

