/******************** (C) COPYRIGHT 2017 LX Group ***********************
*
* File Name          : lis3dh.h STMicroelectronics LIS3DH driver header
* Authors            : Melwyn Lobo
*
*************************************************************************
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License version 2 as
* published by the Free Software Foundation.
*
* THE PRESENT SOFTWARE IS PROVIDED ON AN "AS IS" BASIS, WITHOUT WARRANTIES
* OR CONDITIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED, FOR THE SOLE
* PURPOSE TO SUPPORT YOUR APPLICATION DEVELOPMENT.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM
THE CONTENT OF SUCH SOFTWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*
* THIS SOFTWARE IS SPECIFICALLY DESIGNED FOR EXCLUSIVE USE WITH ST PARTS.
*
***************************************************************************/
#ifndef	__LIS3DH_H__
#define	__LIS3DH_H__
#include	<linux/ioctl.h>	/* For IOCTL macros */
#include	<linux/input.h>
#ifndef DEBUG
#define DEBUG
#endif
#define SAD0L			0x00
#define SAD0H			0x01
#define LIS3DH_ACC_I2C_SADROOT	0x0C
#define LIS3DH_ACC_I2C_SAD_L	((LIS3DH_ACC_I2C_SADROOT<<1)|SAD0L)
#define LIS3DH_ACC_I2C_SAD_H	((LIS3DH_ACC_I2C_SADROOT<<1)|SAD0H)
#define	LIS3DH_ACC_DEV_NAME	"lxaccell"
/* SAO pad is connected to GND, set LSB of SAD '0' */
#define LIS3DH_ACC_I2C_ADDR     LIS3DH_ACC_I2C_SAD_L
#define LIS3DH_ACC_I2C_NAME     LIS3DH_ACC_DEV_NAME
#define	LIS3DH_ACC_IOCTL_BASE 77
/** The following define the IOCTL command values via the ioctl macros */
#define	LIS3DH_ACC_IOCTL_SET_DELAY	_IOW(LIS3DH_ACC_IOCTL_BASE, 0, int)
#define	LIS3DH_ACC_IOCTL_GET_DELAY	_IOR(LIS3DH_ACC_IOCTL_BASE, 1, int)
#define	LIS3DH_ACC_IOCTL_SET_ENABLE	_IOW(LIS3DH_ACC_IOCTL_BASE, 2, int)
#define	LIS3DH_ACC_IOCTL_GET_ENABLE	_IOR(LIS3DH_ACC_IOCTL_BASE, 3, int)
#define	LIS3DH_ACC_IOCTL_SET_FULLSCALE	_IOW(LIS3DH_ACC_IOCTL_BASE, 4, int)
#define	LIS3DH_ACC_IOCTL_SET_G_RANGE	LIS3DH_ACC_IOCTL_SET_FULLSCALE
#define	LIS3DH_ACC_IOCTL_SET_CTRL_REG3	_IOW(LIS3DH_ACC_IOCTL_BASE, 6, int)
#define	LIS3DH_ACC_IOCTL_SET_CTRL_REG6	_IOW(LIS3DH_ACC_IOCTL_BASE, 7, int)
#define	LIS3DH_ACC_IOCTL_SET_DURATION1	_IOW(LIS3DH_ACC_IOCTL_BASE, 8, int)
#define	LIS3DH_ACC_IOCTL_SET_THRESHOLD1	_IOW(LIS3DH_ACC_IOCTL_BASE, 9, int)
#define	LIS3DH_ACC_IOCTL_SET_CONFIG1	_IOW(LIS3DH_ACC_IOCTL_BASE, 10, int)
#define	LIS3DH_ACC_IOCTL_SET_DURATION2	_IOW(LIS3DH_ACC_IOCTL_BASE, 11, int)
#define	LIS3DH_ACC_IOCTL_SET_THRESHOLD2	_IOW(LIS3DH_ACC_IOCTL_BASE, 12, int)
#define	LIS3DH_ACC_IOCTL_SET_CONFIG2	_IOW(LIS3DH_ACC_IOCTL_BASE, 13, int)
#define	LIS3DH_ACC_IOCTL_GET_SOURCE1	_IOW(LIS3DH_ACC_IOCTL_BASE, 14, int)
#define	LIS3DH_ACC_IOCTL_GET_SOURCE2	_IOW(LIS3DH_ACC_IOCTL_BASE, 15, int)
#define	LIS3DH_ACC_IOCTL_GET_TAP_SOURCE	_IOW(LIS3DH_ACC_IOCTL_BASE, 16, int)
#define	LIS3DH_ACC_IOCTL_SET_TAP_TW	_IOW(LIS3DH_ACC_IOCTL_BASE, 17, int)
#define	LIS3DH_ACC_IOCTL_SET_TAP_CFG	_IOW(LIS3DH_ACC_IOCTL_BASE, 18, int)
#define	LIS3DH_ACC_IOCTL_SET_TAP_TLIM	_IOW(LIS3DH_ACC_IOCTL_BASE, 19, int)
#define	LIS3DH_ACC_IOCTL_SET_TAP_THS	_IOW(LIS3DH_ACC_IOCTL_BASE, 20, int)
#define	LIS3DH_ACC_IOCTL_SET_TAP_TLAT	_IOW(LIS3DH_ACC_IOCTL_BASE, 21, int)
#define LIS3DH_ACC_IOCTL_GET_COOR_XYZ	_IOW(LIS3DH_ACC_IOCTL_BASE, 22, int)
#define LIS3DH_ACC_IOCTL_GET_CHIP_ID	 _IOR(LIS3DH_ACC_IOCTL_BASE, \
						255, char[32])
/************************************************/
/*	Accelerometer defines section		*/
/************************************************/
/* Accelerometer Sensor Full Scale */
#define	LIS3DH_ACC_FS_MASK		0x30
#define LIS3DH_ACC_G_2G			0x00
#define LIS3DH_ACC_G_4G			0x10
#define LIS3DH_ACC_G_8G			0x20
#define LIS3DH_ACC_G_16G		0x30
/* Accelerometer Sensor Operating Mode */
#define LIS3DH_ACC_ENABLE		0x01
#define LIS3DH_ACC_DISABLE		0x00
#define LXACCELL_OUTPUT_RATE_MASK 0XF0
#define LXACCELL_OUTPUT_RATE1 0x10
#define LXACCELL_OUTPUT_RATE10 0x20
#define LXACCELL_OUTPUT_RATE25 0x30
#define LXACCELL_OUTPUT_RATE50 0x40
#define LXACCELL_OUTPUT_RATE100 0x50
#define LXACCELL_OUTPUT_RATE200 0x60
#define LXACCELL_OUTPUT_RATE400 0x70
#define LXACCELL_OUTPUT_RATE1250 0x90
#define	IA 0x40
#define	ZH 0x20
#define	ZL 0x10
#define	YH 0x08
#define	YL 0x04
#define	XH 0x02
#define	XL 0x01

#define	CONTROL_REG3_I1_AOI1 0x40
#define	CONTROL_REG6_I2_TAPEN 0x80
#define	CONTROL_REG6_HLACTIVE 0x02
/* TAP_SOURCE_REG BIT */
#define	DTAP 0x20
#define	STAP 0x10
#define	SIGNTAP 0x08
#define	ZTAP 0x04
#define	YTAP 0x02
#define	XTAZ 0x01
#define	FUZZ 32
#define	FLAT 32
#define	I2C_RETRY_DELAY 5
#define	I2C_RETRIES 5
#define	I2C_AUTO_INCREMENT 0x80
/* RESUME STATE INDICES */
#define	RES_CONTROL_REG1 0
#define	RES_CONTROL_REG2 1
#define	RES_CONTROL_REG3 2
#define	RES_CONTROL_REG4 3
#define	RES_CONTROL_REG5 4
#define	RES_CONTROL_REG6 5
#define	RES_INT_CFG1 6
#define	RES_INT_THS1 7
#define	RES_INT_DUR1 8
#define	RES_INT_CFG2 9
#define	RES_INT_THS2 10
#define	RES_INT_DUR2 11
#define	RES_TT_CFG 12
#define	RES_TT_THS 13
#define	RES_TT_LIM 14
#define	RES_TT_TLAT 15
#define	RES_TT_TW 16
#define	RES_TEMPERATURE_CONFIG_REG 17
#define	RES_REFERENCE_REG 18
#define	RES_FIFO_CONTROL_REG 19
#define	SAVE 20
#define DEVICE_INFO "st,BBLIS3DH"
#define DEVICE_INFO_LEN 32
/* end RESUME STATE INDICES */

#define GPIO_INTERRUPT1 48
struct {
	unsigned int lxaccell_upper_ms;
	unsigned int mask;
} lxaccell_data_rate[] = {
		{ 1, LXACCELL_OUTPUT_RATE1250 },
		{ 3, LXACCELL_OUTPUT_RATE400 },
		{ 5, LXACCELL_OUTPUT_RATE200 },
		{ 10, LXACCELL_OUTPUT_RATE100 },
		{ 20, LXACCELL_OUTPUT_RATE50 },
		{ 40, LXACCELL_OUTPUT_RATE25 },
		{ 100, LXACCELL_OUTPUT_RATE10 },
		{ 1000, LXACCELL_OUTPUT_RATE1 },
	};
#ifdef	__KERNEL__
struct lxaccell_platform_data {
	int poll_interval;
	int min_interval;
	u8 g_range;
	u8 axis_map_x;
	u8 axis_map_y;
	u8 axis_map_z;
	u8 negate_x;
	u8 negate_y;
	u8 negate_z;
	int gpio_int1;
	int gpio_int2;
};
#endif	/* __KERNEL__ */
#endif	/* __LIS3DH_H__ */
