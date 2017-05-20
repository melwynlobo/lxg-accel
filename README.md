# lxg-accel
LIS3DH Accelerometer driver
Solution Test Setup
======================================

Board Configuration Pin Mapping Table
======================================
Using Jumper cable connect the LIS3DH breakout 
P9 connector on BeagleBone Black	|	Adafruit LIS3Dh breakout pin
----------------------------------- |  -------------------------------
Pin 1 (DGND) 						|	Gnd
Pin 3 (VDD_3V3)						|	Vin
Pin 15 (GPIO_48)					|	INT
Pin 19 (I2C2_SCL)					|	SCL
Pin 20 (I2C2_SDA)					|	SDA

Module building procedure
=============================

1. Update dtc tool on local (Debian Wheezy) host. [OPTIONAL STEP]
	Execute ./dtc.sh

1. Compile device tree overlay cape to Binary blob [OPTIONAL STEP]
		/usr/local/bin/dtc -O dtb -o lis3dh-00A0.dtbo -b 0 -@ lis3dh.dts	

1. After compiling or you can use existing lis3dh-00A0.dtbo, copy the binary to target's /lib/firmware folder:
		scp lis3dh-00A0.dtbo 192.168.7.2:/lib/firmware/

1. Compile lxg-accel.ko driver module, just type make on current folder. Copy the driver to /lib/modules or any folder of your choice
		scp lxg-accel.ko 192.168.7.2:/lib/modules

1. Enter target using: "ssh root@192.168.7.2"

1. Install dynamic cape on to device tree of beaglebone
	echo lis3dh > /sys/devices/bone_capemgr.9/slots

1. Goto location of driver module (/lib/modules as per Step 4) and install driver.
		insmod lxg-accel.ko

1. Read /dev/lis3dh (misc device node)
	Sample log snippet:
	-147,-1005,-43
	-147,-1005,-43
	-147,-1005,-43
	-147,-1005,-43
	-147,-1005,-43
	-147,-1005,-43
	-137,-1012,-60
	-137,-1012,-60
	-137,-1012,-60
	-137,-1012,-60
	-137,-1012,-60
	-125,-456,36
	-125,-456,36
	-125,-456,36

