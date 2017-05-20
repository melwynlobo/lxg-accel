#######################################
#  Makefile for lis3dh accell driver  #
#  Author : Melwyn Lobo               #
#  Copyright: LX Group                #
#######################################

KERNELDIR := linux-headers-3.8.13-bone70
ARM_COMPILER := arm-linux-gnueabihf-
ARCH := arm
PWD := ${shell pwd}

obj-m = lxg-accel.o
lxg-accel-objs = lis3dh.o

all: BBLIS3DH 

.PHONY: clean

BBLIS3DH:
	make -C $(KERNELDIR) M=$(PWD) modules ARCH=$(ARCH) CROSS_COMPILE=$(ARM_COMPILER)

clean:
	make -C $(KERNELDIR) M=$(PWD) clean

