#LIS3DH Accelerometer driver

Board Configuration Pin Mapping Table
======================================
Using Jumper cable connect the LIS3DH breakout 


P9 connector on BB |Adafruit LIS3DH
-------------------|------------------
Pin 1 (DGND)       | Gnd
Pin 3 (VDD_3V3)    | Vin
Pin 15 (GPIO_48)   | INT
Pin 19 (I2C2_SCL)  | SCL
Pin 20 (I2C2_SDA)  | SDA

Module building procedure
=============================

1. Update dtc tool on local (Debian Wheezy) host. [OPTIONAL STEP if using precompiled lis3dh-00A0.dtbo in this folder]

	Execute ./dtc.sh

1. Compile device tree overlay cape to Binary blob [OPTIONAL STEP if using precompiled lis3dh-00A0.dtbo provided in this folder]

		/usr/local/bin/dtc -O dtb -o lis3dh-00A0.dtbo -b 0 -@ lis3dh.dts	

1. After compiling or you can use existing lis3dh-00A0.dtbo, copy the binary to target's /lib/firmware folder:

		scp lis3dh-00A0.dtbo 192.168.7.2:/lib/firmware/

1. Compiling lxg-accel.ko driver module, requires kernel headers for the kernel image running on your beagleboe black. 
	
		[TO SKIP BELOW STEPS SIMPLY untar linux-headers-3.8.13-bone70.tgz, using tar xfz linux-headers-3.8.13-bone70.tgz]

		To download kernel headers to current folder, enter target (192.168.7.2):
			
			sudo apt-get install linux-headers-`uname -r`

			logout (exit to host again)

			scp -r 192.168.7.2:/usr/src/linux-headers-3.8.13-bone70 ./

			Next steps are on host (Debian Wheezy), for module compilation procedure. End result of follwing steps is to copy x86 cross compiled kernel/scripts folder to ./linux-headers-3.8.13-bone70) 


			Install Debian Wheezy cross compiler toolchain for beaglebon black

			sudo apt-get install crossbuild-essential-armhf

			Download kernel for building kernel using cross compiler. (Steps taken from "elinux.org/Building_BBB_Kernel#Downloading_and_building_the_Linux_Kernel")

			git clone git://github.com/beagleboard/kernel.git

			Need to perform some further preparations before kernel is built:

			cd kernel

			git checkout 3.8

			./patch.sh

			cp configs/beaglebone kernel/arch/arm/configs/beaglebone_defconfig

			cp -r ~/kernel/scripts linux-headers-3.8.13-bone70/	

			wget http://arago-project.org/git/projects/?p=am33x-cm3.git\;a=blob_plain\;f=bin/am335x-pm-firmware.bin\;hb=HEAD -O kernel/firmware/am335x-pm-firmware.bin

			cd kernel 

			make ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf- beaglebone_defconfig uImage dtbs
		
			After making kernel, overwrite kernel/scripts folder to linux-headers-3.8.13-bone70/ folder, to ensure module building is successful from here:

			cp -r ~kernel/scripts $(GITHUBCLONEDIR)/lxg-accel/linux-headers-3.8.13-bone70/
			
1. After downloading kernel headers, (I have alredy uploaded linux-3.8.13-bone70.tgz kernel headers used for my beablebone black on this repository, just extract it and type "make" on current folder to build the lxg-accel.ko driver module.. Copy the driver to /lib/modules or any folder of your choice

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


