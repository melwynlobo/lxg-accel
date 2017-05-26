/******************** (C) COPYRIGHT 2017 LXGROUP ********************
 *
 * File Name          : lis3dh.c
 * Authors            : Melwyn Lobo
 * Description        : STMicroelectronics LIS3DH driver
 *
 ****************************************************************************
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THE PRESENT SOFTWARE IS PROVIDED ON AN "AS IS" BASIS, WITHOUT WARRANTIES
 * OR CONDITIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED, FOR THE SOLE
 * PURPOSE TO SUPPORT YOUR APPLICATION DEVELOPMENT.
 * AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
 * INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
 * CONTENT OF SUCH SOFTWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
 * INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
 *
 * THIS SOFTWARE IS SPECIFICALLY DESIGNED FOR EXCLUSIVE USE WITH ST PARTS.
 *
 ******************************************************************************/
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/input.h>
#include <linux/input-polldev.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/workqueue.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif
#include <linux/of.h>
#include <linux/of_fdt.h>
#include <linux/of_platform.h>
#include "lis3dh.h"
#define	INTERRUPT_MANAGEMENT 1
#define	G_MAX 16000 /*Maximum polled-device-reported g value*/
/*
#define	SHIFT_ADJ_2G 4
#define	SHIFT_ADJ_4G 3
#define	SHIFT_ADJ_8G 2
#define	SHIFT_ADJ_16G 1
*/
#define SENSITIVITY_2G 1	/*mg/LSB*/
#define SENSITIVITY_4G 2	/*mg/LSB*/
#define SENSITIVITY_8G 4	/*mg/LSB*/
#define SENSITIVITY_16G 12	/*mg/LSB*/
#define	HIGH_RESOLUTION 0x08
#define	AXISDATA_REG 0x28
#define WHOAMI_LIS3DH_ACC 0x33	/*Expctd content for WAI*/
/*CONTROL REGISTERS*/
#define POWON	0x47
#define WHO_AM_I 0x0F	/*WhoAmI register*/
#define	TEMP_CFG_REG 0x1F	/*temper sens control reg*/
/* ctrl 1: ODR3 ODR2 ODR ODR0 LPen Zenable Yenable Zenable*/
#define	CTRL_REG1 0x20	/*control reg 1*/
#define	CTRL_REG2 0x21	/*control reg 2*/
#define	CTRL_REG3 0x22	/*control reg 3*/
#define	CTRL_REG4 0x23	/*control reg 4*/
#define	CTRL_REG5 0x24	/*control reg 5*/
#define	CTRL_REG6 0x25	/*control reg 6*/
#define	FIFO_CTRL_REG 0x2E	/*FiFo control reg*/
#define	INT_CFG1 0x30	/*interrupt 1 config*/
#define	INT_SRC1 0x31	/*interrupt 1 source*/
#define	INT_THS1 0x32	/*interrupt 1 threshold*/
#define	INT_DUR1 0x33	/*interrupt 1 duration*/
#define	INT_CFG2 0x34	/*interrupt 2 config*/
#define	INT_SRC2 0x35	/*interrupt 2 source*/
#define	INT_THS2 0x36	/*interrupt 2 threshold*/
#define	INT_DUR2 0x37	/*interrupt 2 duration*/
#define	TT_CFG 0x38	/*tap config*/
#define	TT_SRC 0x39	/*tap source*/
#define	TT_THS 0x3A	/*tap threshold*/
#define	TT_LIM 0x3B	/*tap time limit*/
#define	TT_TLAT 0x3C	/*tap time latency*/
#define	TT_TW 0x3D	/*tap time window*/
#define ENABLE_HIGH_RESOLUTION	1
#define LIS3DH_ACC_PM_OFF 0x00
#define LIS3DH_ACC_ENABLE_ALL_AXES 0x07
#define PMODE_MASK 0x08
#define ODR_MASK 0XF0
#define ODR1 0x10	/* 1Hz output data rate */
#define ODR10 0x20	/* 10Hz output data rate */
#define ODR25 0x30	/* 25Hz output data rate */
#define ODR50 0x40	/* 50Hz output data rate */
#define ODR100 0x50	/* 100Hz output data rate */
#define ODR200 0x60	/* 200Hz output data rate */
#define ODR400 0x70	/* 400Hz output data rate */
#define ODR1250 0x90	/* 1250Hz output data rate */
#define	IA 0x40
#define	ZH 0x20
#define	ZL 0x10
#define	YH 0x08
#define	YL 0x04
#define	XH 0x02
#define	XL 0x01
/* CTRL REG BITS*/
#define	CTRL_REG3_I1_AOI1 0x40
#define	CTRL_REG6_I2_TAPEN 0x80
#define	CTRL_REG6_HLACTIVE 0x02
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
#define	RES_CTRL_REG1 0
#define	RES_CTRL_REG2 1
#define	RES_CTRL_REG3 2
#define	RES_CTRL_REG4 3
#define	RES_CTRL_REG5 4
#define	RES_CTRL_REG6 5
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
#define	RES_TEMP_CFG_REG 17
#define	RES_REFERENCE_REG 18
#define	RES_FIFO_CTRL_REG 19
#define	RESUME_ENTRIES 20
#define DEVICE_INFO "ST, LIS3DH"
#define DEVICE_INFO_LEN 32
/* end RESUME STATE INDICES */
#define GSENSOR_GINT1_GPI 0
#define GSENSOR_GINT2_GPI 1
#define GSENSOR_GINT 48
struct {
	unsigned int cutoff_ms;
	unsigned int mask;
} lis3dh_acc_odr_table[] = {
	{
	1, ODR1250}, {
	3, ODR400}, {
	5, ODR200}, {
	10, ODR100}, {
	20, ODR50}, {
	40, ODR25}, {
	100, ODR10}, {
1000, ODR1},};
struct lis3dh_acc_data {
	struct i2c_client *client;
	struct lis3dh_acc_platform_data *pdata;
	struct mutex lock;
	struct delayed_work input_work;
	struct input_dev *input_dev;
	int hw_initialized;
	/* hw_working=-1 means not tested yet */
	int hw_working;
	atomic_t enabled;
	int on_before_suspend;
	u8 sensitivity;
	u8 resume_state[RESUME_ENTRIES];
	int irq1;
	struct work_struct irq1_work;
	struct workqueue_struct *irq1_work_queue;
	int irq2;
	struct work_struct irq2_work;
	struct workqueue_struct *irq2_work_queue;
	struct fasync_struct *async_queue;
#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend early_suspend;
#endif
};
/*
 * Because misc devices can not carry a pointer from driver register to
 * open, we keep this global.  This limits the driver to a single instance.
 */
/*#define pr_debug	printk
#define pr_info		printk
#define pr_err		printk */

struct lis3dh_acc_data *lis3dh_acc_misc_data;
struct i2c_client *lis3dh_i2c_client;
static int lis3dh_acc_i2c_read(struct lis3dh_acc_data *acc, u8 *buf, int len)
{
	int err;
	int tries = 0;
	struct i2c_msg msgs[] = {
		{
			.addr = acc->client->addr,
			.flags = acc->client->flags & I2C_M_TEN,
			.len = 1,
			.buf = buf,},
		{
			.addr = acc->client->addr,
			.flags = (acc->client->flags & I2C_M_TEN) | I2C_M_RD,
			.len = len,
			.buf = buf,},
	};
	do {
		err = i2c_transfer(acc->client->adapter, msgs, 2);
		if (err != 2)
			msleep_interruptible(I2C_RETRY_DELAY);
	} while ((err != 2) && (++tries < I2C_RETRIES));
	if (err != 2) {
		dev_err(&acc->client->dev, "read transfer error\n");
		err = -EIO;
	} else
		err = 0;
	return err;
}
static int lis3dh_acc_i2c_write(struct lis3dh_acc_data *acc, u8 *buf, int len)
{
	int err;
	int tries = 0;
	struct i2c_msg msgs[] = {
		{
			.addr = acc->client->addr,
			.flags = acc->client->flags & I2C_M_TEN,
			.len = len + 1,
			.buf = buf,
		},
	};
	do {
		err = i2c_transfer(acc->client->adapter, msgs, 1);
		if (err != 1)
			msleep_interruptible(I2C_RETRY_DELAY);
	} while ((err != 1) && (++tries < I2C_RETRIES));
	if (err != 1) {
		dev_err(&acc->client->dev, "write transfer error\n");
		err = -EIO;
	} else {
		err = 0;
	}
	return err;
}
static int lis3dh_acc_hw_init(struct lis3dh_acc_data *acc)
{
	int err = -1;
	u8 buf[7];
	pr_debug("%s: hw init start\n", LIS3DH_ACC_DEV_NAME);
	buf[0] = WHO_AM_I;
	err = lis3dh_acc_i2c_read(acc, buf, 1);
	if (err < 0)
		goto error_firstread;
	else
		acc->hw_working = 1;
	if (buf[0] != WHOAMI_LIS3DH_ACC) {
		err = -1;	/* choose the right coded error */
		goto error_unknown_device;
	}
	buf[0] = CTRL_REG1;
	buf[1] = acc->resume_state[RES_CTRL_REG1];
	err = lis3dh_acc_i2c_write(acc, buf, 1);
	if (err < 0)
		goto error1;
	buf[0] = TEMP_CFG_REG;
	buf[1] = acc->resume_state[RES_TEMP_CFG_REG];
	err = lis3dh_acc_i2c_write(acc, buf, 1);
	if (err < 0)
		goto error1;
	buf[0] = FIFO_CTRL_REG;
	buf[1] = acc->resume_state[RES_FIFO_CTRL_REG];
	err = lis3dh_acc_i2c_write(acc, buf, 1);
	if (err < 0)
		goto error1;
	buf[0] = (I2C_AUTO_INCREMENT | TT_THS);
	buf[1] = acc->resume_state[RES_TT_THS];
	buf[2] = acc->resume_state[RES_TT_LIM];
	buf[3] = acc->resume_state[RES_TT_TLAT];
	buf[4] = acc->resume_state[RES_TT_TW];
	err = lis3dh_acc_i2c_write(acc, buf, 4);
	if (err < 0)
		goto error1;
	buf[0] = TT_CFG;
	buf[1] = acc->resume_state[RES_TT_CFG];
	err = lis3dh_acc_i2c_write(acc, buf, 1);
	if (err < 0)
		goto error1;
	buf[0] = (I2C_AUTO_INCREMENT | INT_THS1);
	buf[1] = acc->resume_state[RES_INT_THS1];
	buf[2] = acc->resume_state[RES_INT_DUR1];
	err = lis3dh_acc_i2c_write(acc, buf, 2);
	if (err < 0)
		goto error1;
	buf[0] = INT_CFG1;
	buf[1] = acc->resume_state[RES_INT_CFG1];
	err = lis3dh_acc_i2c_write(acc, buf, 1);
	if (err < 0)
		goto error1;
	buf[0] = (I2C_AUTO_INCREMENT | INT_THS2);
	buf[1] = acc->resume_state[RES_INT_THS2];
	buf[2] = acc->resume_state[RES_INT_DUR2];
	err = lis3dh_acc_i2c_write(acc, buf, 2);
	if (err < 0)
		goto error1;
	buf[0] = INT_CFG2;
	buf[1] = acc->resume_state[RES_INT_CFG2];
	err = lis3dh_acc_i2c_write(acc, buf, 1);
	if (err < 0)
		goto error1;
	buf[0] = (I2C_AUTO_INCREMENT | CTRL_REG2);
	buf[1] = acc->resume_state[RES_CTRL_REG2];
	buf[2] = acc->resume_state[RES_CTRL_REG3];
	buf[3] = acc->resume_state[RES_CTRL_REG4];
	buf[4] = acc->resume_state[RES_CTRL_REG5];
	buf[5] = acc->resume_state[RES_CTRL_REG6];
	err = lis3dh_acc_i2c_write(acc, buf, 5);
	if (err < 0)
		goto error1;
	acc->hw_initialized = 1;
	pr_debug("%s: hw init done\n", LIS3DH_ACC_DEV_NAME);
	return 0;
error_firstread:
	acc->hw_working = 0;
	dev_warn(&acc->client->dev, "Error reading available/working?\n");
	goto error1;
error_unknown_device:
	dev_err(&acc->client->dev,
		"device unknown. Expected: 0x%x,Replies: 0x%x\n",
		WHOAMI_LIS3DH_ACC, buf[0]);
error1:
	acc->hw_initialized = 0;
	dev_err(&acc->client->dev, "hw init error 0x%x,0x%x: %d\n", buf[0],
		buf[1], err);
	return err;
}
static void lis3dh_acc_device_power_off(struct lis3dh_acc_data *acc)
{
	int err;
	u8 buf[2] = { CTRL_REG1, LIS3DH_ACC_PM_OFF };
	err = lis3dh_acc_i2c_write(acc, buf, 1);
	if (err < 0)
		dev_err(&acc->client->dev, "soft power off failed: %d\n", err);
	if (acc->hw_initialized) {
		if (acc->irq1 != 0)
			disable_irq_nosync(acc->irq1);
		if (acc->irq2 != 0)
			disable_irq_nosync(acc->irq2);
		acc->hw_initialized = 0;
	}
}
static int lis3dh_acc_device_power_on(struct lis3dh_acc_data *acc)
{
	int err = -1;
	if (!acc->hw_initialized) {
		err = lis3dh_acc_hw_init(acc);
		if (acc->hw_working == 1 && err < 0) {
			lis3dh_acc_device_power_off(acc);
			return err;
		}
	}
	return 0;
}
static irqreturn_t lis3dh_acc_isr1(int irq, void *dev)
{
	struct lis3dh_acc_data *acc = dev;
	//disable_irq_nosync(irq);
	//queue_work(acc->irq1_work_queue, &acc->irq1_work);
	//pr_debug("%s: isr1 queued\n", LIS3DH_ACC_DEV_NAME);
	printk("%s: isr1 queued\n", LIS3DH_ACC_DEV_NAME);
	if (acc->async_queue) {
		kill_fasync(&acc->async_queue, SIGIO, POLL_IN);
	}

	return IRQ_HANDLED;
}
static irqreturn_t lis3dh_acc_isr2(int irq, void *dev)
{
	struct lis3dh_acc_data *acc = dev;
	disable_irq_nosync(irq);
	queue_work(acc->irq2_work_queue, &acc->irq2_work);
	printk("%s: isr2 queued\n", LIS3DH_ACC_DEV_NAME);
	return IRQ_HANDLED;
}
static void lis3dh_acc_irq1_work_func(struct work_struct *work)
{
	/*struct lis3dh_acc_data *acc =
	    container_of(work, struct lis3dh_acc_data, irq1_work);
	*/
	/* TODO  add interrupt service procedure.
	   ie:lis3dh_acc_get_int1_source(acc); */
	;
	/*  */
	pr_debug("%s: IRQ1 triggered\n", LIS3DH_ACC_DEV_NAME);
}
static void lis3dh_acc_irq2_work_func(struct work_struct *work)
{
	/*struct lis3dh_acc_data *acc =
	    container_of(work, struct lis3dh_acc_data, irq2_work);
	*/
	/* TODO  add interrupt service procedure.
	   ie:lis3dh_acc_get_tap_source(acc); */
	;
	/*  */
	pr_debug("%s: IRQ2 triggered\n", LIS3DH_ACC_DEV_NAME);
}

int lis3dh_acc_update_g_range(struct lis3dh_acc_data *acc, u8 new_g_range)
{
	int err;
	u8 sensitivity;
	u8 buf[2];
	u8 updated_val;
	u8 init_val;
	u8 new_val;
	u8 mask = LIS3DH_ACC_FS_MASK | HIGH_RESOLUTION;
	pr_debug("%s\n", __func__);
	switch (new_g_range) {
	case LIS3DH_ACC_G_2G:
		sensitivity = SENSITIVITY_2G;
		break;
	case LIS3DH_ACC_G_4G:
		sensitivity = SENSITIVITY_4G;
		break;
	case LIS3DH_ACC_G_8G:
		sensitivity = SENSITIVITY_8G;
		break;
	case LIS3DH_ACC_G_16G:
		sensitivity = SENSITIVITY_16G;
		break;
	default:
		dev_err(&acc->client->dev, "invalid g range requested: %u\n",
			new_g_range);
		return -EINVAL;
	}
	if (atomic_read(&acc->enabled)) {
		/* Set configuration register 4, which contains g range setting
		 *  NOTE: this is a straight overwrite because this driver does
		 *  not use any of the other configuration bits in this
		 *  register.  Should this become untrue, we will have to read
		 *  out the value and only change the relevant bits --XX----
		 *  (marked by X) */
		buf[0] = CTRL_REG4;
		err = lis3dh_acc_i2c_read(acc, buf, 1);
		if (err < 0)
			goto error;
		init_val = buf[0];
		acc->resume_state[RES_CTRL_REG4] = init_val;
		new_val = new_g_range | HIGH_RESOLUTION;
		updated_val = ((mask & new_val) | ((~mask) & init_val));
		buf[1] = updated_val;
		buf[0] = CTRL_REG4;
		err = lis3dh_acc_i2c_write(acc, buf, 1);
		if (err < 0)
			goto error;
		acc->resume_state[RES_CTRL_REG4] = updated_val;
		acc->sensitivity = sensitivity;
		printk("%s sensitivity %d g-range %d\n", __func__,
			sensitivity, new_g_range);
	}
	return 0;
error:
	dev_err(&acc->client->dev, "update g range failed 0x%x,0x%x: %d\n",
		buf[0], buf[1], err);
	return err;
}
int lis3dh_acc_update_odr(struct lis3dh_acc_data *acc, int poll_interval_ms)
{
	int err = -1;
	int i;
	u8 config[2];
	/* Convert the poll interval into an output data rate configuration
	 *  that is as low as possible.  The ordering of these checks must be
	 *  maintained due to the cascading cut off values - poll intervals are
	 *  checked from shortest to longest.  At each check, if the next lower
	 *  ODR cannot support the current poll interval, we stop searching */
	for (i = ARRAY_SIZE(lis3dh_acc_odr_table) - 1; i >= 0; i--) {
		if (lis3dh_acc_odr_table[i].cutoff_ms <= poll_interval_ms)
			break;
	}
	config[1] = lis3dh_acc_odr_table[i].mask;
	config[1] |= LIS3DH_ACC_ENABLE_ALL_AXES;
	/* If device is currently enabled, we need to write new
	 *  configuration out to it */
	if (atomic_read(&acc->enabled)) {
		config[0] = CTRL_REG1;
		err = lis3dh_acc_i2c_write(acc, config, 1);
		if (err < 0)
			goto error;
		acc->resume_state[RES_CTRL_REG1] = config[1];
	}
	return 0;
error:
	dev_err(&acc->client->dev, "update odr failed 0x%x,0x%x: %d\n",
		config[0], config[1], err);
	return err;
}

static int lis3dh_acc_get_acceleration_data(struct lis3dh_acc_data *acc,
					    int *xyz)
{
	int err = -1;
	/* Data bytes from hardware xL, xH, yL, yH, zL, zH */
	u8 acc_data[6];
	/* x,y,z hardware data */
	s16 hw_d[3] = { 0 };
	acc_data[0] = (I2C_AUTO_INCREMENT | AXISDATA_REG);
	err = lis3dh_acc_i2c_read(acc, acc_data, 6);
	if (err < 0) {
		pr_debug("%s I2C read error %d\n", LIS3DH_ACC_I2C_NAME,
		       err);
		return err;
	}
	hw_d[0] = (((s16) ((acc_data[1] << 8) | acc_data[0])) >> 4);
	hw_d[1] = (((s16) ((acc_data[3] << 8) | acc_data[2])) >> 4);
	hw_d[2] = (((s16) ((acc_data[5] << 8) | acc_data[4])) >> 4);
	hw_d[0] = hw_d[0] * acc->sensitivity;
	hw_d[1] = hw_d[1] * acc->sensitivity;
	hw_d[2] = hw_d[2] * acc->sensitivity;
	 xyz[0] = ((acc->pdata->negate_x) ? (-hw_d[acc->pdata->axis_map_x])
		  : (hw_d[acc->pdata->axis_map_x]));
	xyz[1] = ((acc->pdata->negate_y) ? (-hw_d[acc->pdata->axis_map_y])
		  : (hw_d[acc->pdata->axis_map_y]));
	xyz[2] = ((acc->pdata->negate_z) ? (-hw_d[acc->pdata->axis_map_z])
		  : (hw_d[acc->pdata->axis_map_z]));
	pr_debug("%s read x=%d, y=%d, z=%d\n",
	       LIS3DH_ACC_DEV_NAME, xyz[0], xyz[1], xyz[2]);
	return err;
}
static void lis3dh_acc_report_values(struct lis3dh_acc_data *acc, int *xyz)
{
	input_report_abs(acc->input_dev, ABS_X, xyz[0]);
	input_report_abs(acc->input_dev, ABS_Y, xyz[1]);
	input_report_abs(acc->input_dev, ABS_Z, xyz[2]);
	input_sync(acc->input_dev);
}
static int lis3dh_acc_enable(struct lis3dh_acc_data *acc)
{
	int err;
	if (!atomic_cmpxchg(&acc->enabled, 0, 1)) {
		err = lis3dh_acc_device_power_on(acc);
		if (err < 0) {
			atomic_set(&acc->enabled, 0);
			return err;
		}
		if (acc->hw_initialized) {
			if (acc->irq1 != 0)
				enable_irq(acc->irq1);
			if (acc->irq2 != 0)
				enable_irq(acc->irq2);
			pr_debug("%s: power on: irq enabled\n",
			       LIS3DH_ACC_DEV_NAME);
		}
		schedule_delayed_work(&acc->input_work,
				      msecs_to_jiffies(acc->pdata->
						       poll_interval));
	}
	return 0;
}
static int lis3dh_acc_disable(struct lis3dh_acc_data *acc)
{
	if (atomic_cmpxchg(&acc->enabled, 1, 0)) {
		cancel_delayed_work_sync(&acc->input_work);
		lis3dh_acc_device_power_off(acc);
	}
	return 0;
}
static int lis3dh_acc_misc_open(struct inode *inode, struct file *file)
{
	int err;
	err = nonseekable_open(inode, file);
	if (err < 0) {
		printk("Error open %d\n", err);
		return err;
	}
	lis3dh_acc_enable(lis3dh_acc_misc_data);
	file->private_data = lis3dh_acc_misc_data;
	printk("Device open\n");
	return 0;
}

static ssize_t lis3dh_acc_misc_read(struct file *filp, char __user *buf,
                                                size_t len, loff_t *off)
{   
    int status = 0;
	int xyz[3] = { 0 };
	struct lis3dh_acc_data *acc = filp->private_data;
    
    if (acc && acc->client) {

		status = lis3dh_acc_get_acceleration_data(acc, xyz);
		if (status >= 0)
        	len = snprintf(buf, len, "%d,%d,%d\n", 
									xyz[0], xyz[1], xyz[2]);

    } else {
		printk("Unexpected error %p:%p:%p\n",
			acc,acc->client,acc->client->adapter);
		status = -1;
    }
        
    return ((status==0) ? len : -EIO);
}

static ssize_t lis3dh_acc_misc_write(struct file *filp, const char __user *buf,
												size_t len, loff_t *off)
{
    int err = 0;
	u8 write[3] = { 0 };
	struct lis3dh_acc_data *acc = filp->private_data;
    unsigned long threshold = 0;
	printk("Inside write %c%c%c%c(%d)%c%c, %d\n",buf[0],buf[1],buf[2],
										buf[3],buf[3],buf[4],buf[5],len);
    if (buf[0]=='d' && buf[1]=='b' && buf[2]==',') {
		if(len == 7)
		threshold = (((int)buf[3] - 48 ) * 100) 
						+ (((int)buf[4] - 48 ) * 10) 
						+ (((int)buf[5] - 48 ));
		else if (len == 6)
			threshold = (((int)buf[3] - 48 ) * 10) + (((int)buf[4] - 48 ));
		else
			threshold = 0; //invalid value or too low
        printk("Applying new threshold: %lu\n", threshold);
    }
	write[0] = (I2C_AUTO_INCREMENT | INT_THS1);
	acc->resume_state[RES_INT_THS1] = threshold;
	write[1] = acc->resume_state[RES_INT_THS1];
	write[2] = acc->resume_state[RES_INT_DUR1];
	err = lis3dh_acc_i2c_write(acc, write, 2);
	if (err < 0)
		return 0;

	printk("Misc write Success\n");
    return len;
}
 
static int lis3dh_acc_misc_fasync(int fd, struct file *file, int mode)
{
    struct lis3dh_acc_data *acc = file->private_data;
    printk("Inside async\n");
    return fasync_helper(fd, file, mode, &acc->async_queue);
}

static int lis3dh_acc_misc_release(struct inode *inode, struct file *file)
{
    printk("Closing device\n");
    /* removing this file from async queue */
    lis3dh_acc_misc_fasync(-1, file, 0);
    return 0;
}

static const struct file_operations lis3dh_acc_misc_fops = {
	.owner = THIS_MODULE,
	.open = lis3dh_acc_misc_open,
	.read = lis3dh_acc_misc_read,
	.write = lis3dh_acc_misc_write,
	.release = lis3dh_acc_misc_release,
	.fasync = lis3dh_acc_misc_fasync

};
static struct miscdevice lis3dh_acc_misc_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = LIS3DH_ACC_DEV_NAME,
	.fops = &lis3dh_acc_misc_fops,
};
static void lis3dh_acc_input_work_func(struct work_struct *work)
{
	struct lis3dh_acc_data *acc;
	int xyz[3] = { 0 };
	int err;
	acc = container_of((struct delayed_work *)work,
			   struct lis3dh_acc_data, input_work);
	mutex_lock(&acc->lock);
	err = lis3dh_acc_get_acceleration_data(acc, xyz);
	if (err < 0)
		dev_err(&acc->client->dev, "get_acceleration_data failed\n");
	else
		lis3dh_acc_report_values(acc, xyz);
	schedule_delayed_work(&acc->input_work,
			      msecs_to_jiffies(acc->pdata->poll_interval));
	mutex_unlock(&acc->lock);
}
#ifdef LIS3DH_OPEN_ENABLE
int lis3dh_acc_input_open(struct input_dev *input)
{
	struct lis3dh_acc_data *acc = input_get_drvdata(input);
	return lis3dh_acc_enable(acc);
}
void lis3dh_acc_input_close(struct input_dev *dev)
{
	struct lis3dh_acc_data *acc = input_get_drvdata(dev);
	lis3dh_acc_disable(acc);
}
#endif
static int lis3dh_acc_input_init(struct lis3dh_acc_data *acc)
{
	int err;
	/* Polling rx data when the interrupt is not used.*/
	INIT_DELAYED_WORK(&acc->input_work, lis3dh_acc_input_work_func);
	acc->input_dev = input_allocate_device();
	if (!acc->input_dev) {
		err = -ENOMEM;
		dev_err(&acc->client->dev, "input device allocate failed\n");
		goto err0;
	}
#ifdef LIS3DH_ACC_OPEN_ENABLE
	acc->input_dev->open = lis3dh_acc_input_open;
	acc->input_dev->close = lis3dh_acc_input_close;
#endif
	input_set_drvdata(acc->input_dev, acc);
	set_bit(EV_ABS, acc->input_dev->evbit);
	/*      next is used for interruptA sources data if the case */
	set_bit(ABS_MISC, acc->input_dev->absbit);
	/*      next is used for interruptB sources data if the case */
	set_bit(ABS_WHEEL, acc->input_dev->absbit);
	input_set_abs_params(acc->input_dev, ABS_X, -G_MAX, G_MAX, FUZZ, FLAT);
	input_set_abs_params(acc->input_dev, ABS_Y, -G_MAX, G_MAX, FUZZ, FLAT);
	input_set_abs_params(acc->input_dev, ABS_Z, -G_MAX, G_MAX, FUZZ, FLAT);
	/*      next is used for interruptA sources data if the case */
	input_set_abs_params(acc->input_dev, ABS_MISC, INT_MIN, INT_MAX, 0, 0);
	/*      next is used for interruptB sources data if the case */
	input_set_abs_params(acc->input_dev, ABS_WHEEL, INT_MIN, INT_MAX, 0, 0);
	acc->input_dev->name = "accelerometer";
	err = input_register_device(acc->input_dev);
	if (err) {
		dev_err(&acc->client->dev,
			"unable to register input polled device %s\n",
			acc->input_dev->name);
		goto err1;
	}
	return 0;
err1:
	input_free_device(acc->input_dev);
err0:
	return err;
}
static void lis3dh_acc_input_cleanup(struct lis3dh_acc_data *acc)
{
	input_unregister_device(acc->input_dev);
	input_free_device(acc->input_dev);
}
#ifdef CONFIG_HAS_EARLYSUSPEND
static void lis3dh_early_suspend(struct early_suspend *es);
static void lis3dh_early_resume(struct early_suspend *es);
#endif

int lis3dh_acc_power_on(struct i2c_client *client)
{
	struct lis3dh_acc_data *acc = i2c_get_clientdata(client);
    int status;
    u8 enable_power[] = {
        CTRL_REG1,
        POWON
    };
    status = lis3dh_acc_i2c_write(acc, enable_power, 2);
    return (status >= 0);
}

static int lis3dh_acc_probe(struct i2c_client *client,const struct i2c_device_id *id)
{
	struct lis3dh_acc_data *acc;
	struct device_node *np;
	u32 val;
	int err = -1;
	int tempvalue;
	pr_info("%s: probe start.\n", LIS3DH_ACC_DEV_NAME);
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "client not i2c capable\n");
		err = -ENODEV;
		goto exit_check_functionality_failed;
	}
	if (!i2c_check_functionality(client->adapter,
				     I2C_FUNC_SMBUS_BYTE |
				     I2C_FUNC_SMBUS_BYTE_DATA |
				     I2C_FUNC_SMBUS_WORD_DATA)) {
		dev_err(&client->dev, "client not smb-i2c capable:2\n");
		err = -EIO;
		goto exit_check_functionality_failed;
	}
	if (!i2c_check_functionality(client->adapter,
		I2C_FUNC_SMBUS_I2C_BLOCK)) {
		dev_err(&client->dev, "client not smb-i2c capable:3\n");
		err = -EIO;
		goto exit_check_functionality_failed;
	}
	/*
	 * OK. From now, we presume we have a valid client. We now create the
	 * client structure, even though we cannot fill it completely yet.
	 */
	acc = kzalloc(sizeof(struct lis3dh_acc_data), GFP_KERNEL);
	if (acc == NULL) {
		err = -ENOMEM;
		dev_err(&client->dev,
			"failed to allocate memory for module data: "
			"%d\n", err);
		goto exit_alloc_data_failed;
	}
	mutex_init(&acc->lock);
	mutex_lock(&acc->lock);
	acc->client = client;
	lis3dh_i2c_client = client;
	i2c_set_clientdata(client, acc);
	pr_debug("%s: %s has set irq1 to irq: %d\n",
	       LIS3DH_ACC_DEV_NAME, __func__, acc->irq1);
	pr_debug("%s: %s has set irq2 to irq: %d\n",
	       LIS3DH_ACC_DEV_NAME, __func__, acc->irq2);
	gpio_request(GSENSOR_GINT, "GSENSOR_GINT");
	gpio_direction_input(GSENSOR_GINT);
	acc->irq1 = gpio_to_irq(GSENSOR_GINT);
	printk("GPIO %d is valid? %s\n", GSENSOR_GINT, 
			gpio_is_valid(GSENSOR_GINT) ? "yes" : "no");
	
	//gpio_request(GSENSOR_GINT2_GPI, "GSENSOR_INT2");
	//acc->irq1 = 0; /* gpio_to_irq(GSENSOR_GINT1_GPI); */
	acc->irq2 = 0; /* gpio_to_irq(GSENSOR_GINT2_GPI); */
	if (acc->irq1 != 0) {
		pr_debug("%s request irq1\n", __func__);
		err =
		    request_irq(acc->irq1, lis3dh_acc_isr1, IRQF_TRIGGER_RISING,
				"lis3dh_acc_irq1", acc);
		if (err < 0) {
			dev_err(&client->dev, "request irq1 failed: %d\n", err);
			goto err_mutexunlockfreedata;
		}
		//disable_irq_nosync(acc->irq1);
		INIT_WORK(&acc->irq1_work, lis3dh_acc_irq1_work_func);
		acc->irq1_work_queue =
		    create_singlethread_workqueue("lis3dh_acc_wq1");
		if (!acc->irq1_work_queue) {
			err = -ENOMEM;
			dev_err(&client->dev, "cannot create work queue1: %d\n",
				err);
			goto err_free_irq1;
		}
	}
	if (acc->irq2 != 0) {
		err =
		    request_irq(acc->irq2, lis3dh_acc_isr2, IRQF_TRIGGER_RISING,
				"lis3dh_acc_irq2", acc);
		if (err < 0) {
			dev_err(&client->dev, "request irq2 failed: %d\n", err);
			goto err_destoyworkqueue1;
		}
		disable_irq_nosync(acc->irq2);
		/* Create workqueue for IRQ.*/
		INIT_WORK(&acc->irq2_work, lis3dh_acc_irq2_work_func);
		acc->irq2_work_queue =
		    create_singlethread_workqueue("lis3dh_acc_wq2");
		if (!acc->irq2_work_queue) {
			err = -ENOMEM;
			dev_err(&client->dev, "cannot create work queue2: %d\n",
				err);
			goto err_free_irq2;
		}
	}
	acc->pdata = kzalloc(sizeof(*acc->pdata), GFP_KERNEL);
	if (acc->pdata == NULL) {
		err = -ENOMEM;
		dev_err(&client->dev,
			"failed to allocate memory for pdata: %d\n", err);
		goto err_destoyworkqueue2;
	}
	if (client->dev.platform_data)
		acc->pdata = client->dev.platform_data;
	 {
		np = lis3dh_i2c_client->dev.of_node;
		if(np)
			pr_debug("$$$$$$$$$$$$DEVICE OF NODE PROCESSING$$$$$$\n");
		else
			pr_debug("$$$$$$$$$$$$DEVICE OF NODE MISSING$$$$$$\n");
		err = of_property_read_u32(np, "poll_interval", &val);
		if (err) {
			dev_err(&client->dev,
				"dts config poll_interval missed");
			acc->pdata->poll_interval = 200;
		} else
			acc->pdata->poll_interval = val;
		err = of_property_read_u32(np, "min_interval", &val);
		if (err) {
			dev_err(&client->dev, "dts config min_interval missed");
			acc->pdata->min_interval = 5;
		} else
			acc->pdata->min_interval = val;
		err = of_property_read_u32(np, "g_range", &val);
		if (err) {
			dev_err(&client->dev, "dts config g_range missed");
			acc->pdata->g_range = 0;
		} else
			acc->pdata->g_range = val;
		err = of_property_read_u32(np, "axis_map_x", &val);
		if (err) {
			dev_err(&client->dev, "dts config axis_map_x missed");
			acc->pdata->axis_map_x = 0;
		} else
			acc->pdata->axis_map_x = val;
		err = of_property_read_u32(np, "axis_map_y", &val);
		if (err) {
			dev_err(&client->dev, "dts config axis_map_y missed");
			acc->pdata->axis_map_y = 1;
		} else
			acc->pdata->axis_map_y = val;
		err = of_property_read_u32(np, "axis_map_z", &val);
		if (err) {
			dev_err(&client->dev, "dts config axis_map_z missed");
			acc->pdata->axis_map_z = 2;
		} else
			acc->pdata->axis_map_z = val;
		err = of_property_read_u32(np, "negate_x", &val);
		if (err) {
			dev_err(&client->dev, "dts config negate_x  missed");
			acc->pdata->negate_x = 1;
		} else
			acc->pdata->negate_x = val;
		err = of_property_read_u32(np, "negate_y", &val);
		if (err) {
			dev_err(&client->dev, "dts config negate_y missed");
			acc->pdata->negate_y = 1;
		} else
			acc->pdata->negate_y = val;
		err = of_property_read_u32(np, "gpio_int1", &val);
		if (err) {
			dev_err(&client->dev, "dts config gpio_int1 missed");
			acc->pdata->gpio_int1 = 0;
		} else
			acc->pdata->gpio_int1 = val;
		err = of_property_read_u32(np, "gpio_int2", &val);
		if (err) {
			dev_err(&client->dev, "dts config gpio_int2 missed");
			acc->pdata->gpio_int2 = 0;
		} else
			acc->pdata->gpio_int2 = val;
	}
	err = lis3dh_acc_device_power_on(acc);
	if (err < 0) {
		dev_err(&client->dev, "power on failed: %d\n", err);
		goto err2;
	}
	if (i2c_smbus_read_byte(client) < 0) {
		pr_err("i2c_smbus_read_byte error!!\n");
		goto err_destoyworkqueue2;
	} else
		pr_debug("%s Device detected!\n", LIS3DH_ACC_DEV_NAME);
	/* read chip id */
	tempvalue = i2c_smbus_read_word_data(client, WHO_AM_I);
	if ((tempvalue & 0x00FF) == WHOAMI_LIS3DH_ACC)
		pr_debug("%s I2C driver registered!\n",
		       LIS3DH_ACC_DEV_NAME);
	else {
		acc->client = NULL;
		pr_info("I2C driver not registered!Device unknown 0x%x\n",
		tempvalue);
		goto err_destoyworkqueue2;
	}
	i2c_set_clientdata(client, acc);
	memset(acc->resume_state, 0, ARRAY_SIZE(acc->resume_state));
	acc->resume_state[RES_CTRL_REG1] = LIS3DH_ACC_ENABLE_ALL_AXES;
	acc->resume_state[RES_CTRL_REG2] = 0x00;
	acc->resume_state[RES_CTRL_REG3] = CTRL_REG3_I1_AOI1;
	acc->resume_state[RES_CTRL_REG4] = 0x00;
	acc->resume_state[RES_CTRL_REG5] = 0x00;
	acc->resume_state[RES_CTRL_REG6] = 0x00;
	acc->resume_state[RES_TEMP_CFG_REG] = 0x00;
	acc->resume_state[RES_FIFO_CTRL_REG] = 0x00;
	acc->resume_state[RES_INT_CFG1] = 0x3F;
	acc->resume_state[RES_INT_THS1] = 0x04;//defualt 100
	acc->resume_state[RES_INT_DUR1] = 0x30;
	acc->resume_state[RES_INT_CFG2] = 0x00;
	acc->resume_state[RES_INT_THS2] = 0x00;
	acc->resume_state[RES_INT_DUR2] = 0x00;
	acc->resume_state[RES_TT_CFG] = 0x00;
	acc->resume_state[RES_TT_THS] = 0x00;
	acc->resume_state[RES_TT_LIM] = 0x00;
	acc->resume_state[RES_TT_TLAT] = 0x00;
	acc->resume_state[RES_TT_TW] = 0x00;
	acc->pdata->g_range = 0;
	atomic_set(&acc->enabled, 1);
	err = lis3dh_acc_update_g_range(acc, acc->pdata->g_range);
	if (err < 0) {
		dev_err(&client->dev, "update_g_range failed\n");
		goto err_power_off;
	}
	err = lis3dh_acc_update_odr(acc, acc->pdata->poll_interval);
	if (err < 0) {
		dev_err(&client->dev, "update_odr failed\n");
		goto err_power_off;
	}
	err = lis3dh_acc_input_init(acc);
	if (err < 0) {
		dev_err(&client->dev, "input init failed\n");
		goto err_power_off;
	}
	lis3dh_acc_misc_data = acc;
	err = misc_register(&lis3dh_acc_misc_device);
	if (err < 0) {
		dev_err(&client->dev,
			"misc LIS3DH_ACC_DEV_NAME register failed\n");
		goto err_input_cleanup;
	}
	//lis3dh_acc_device_power_off(acc);
	/* As default, do not report information */
	//atomic_set(&acc->enabled, 0);
#ifdef CONFIG_HAS_EARLYSUSPEND
	/* acc->early_suspend.suspend = lis3dh_early_suspend;
	acc->early_suspend.resume = lis3dh_early_resume;
	acc->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN;
	register_early_suspend(&acc->early_suspend); */
#endif
	mutex_unlock(&acc->lock);
	dev_info(&client->dev, "###%s###\n", __func__);
	pr_info("%s -- success !\n", __func__);
	return 0;
err_input_cleanup:
	lis3dh_acc_input_cleanup(acc);
err_power_off:
	lis3dh_acc_device_power_off(acc);
err2:
	kfree(acc->pdata);
err_destoyworkqueue2:
	if (acc->irq2_work_queue)
		destroy_workqueue(acc->irq2_work_queue);
err_free_irq2:
	if (acc->irq2) {
		free_irq(acc->irq2, acc);
		gpio_free(GSENSOR_GINT2_GPI);
	}
err_destoyworkqueue1:
	if (acc->irq1_work_queue)
		destroy_workqueue(acc->irq1_work_queue);
err_free_irq1:
	if (acc->irq1) {
		free_irq(acc->irq1, acc);
		gpio_free(GSENSOR_GINT);
	}
err_mutexunlockfreedata:
	kfree(acc);
	mutex_unlock(&acc->lock);
	i2c_set_clientdata(client, NULL);
	lis3dh_acc_misc_data = NULL;
exit_alloc_data_failed:
exit_check_functionality_failed:
	pr_err("%s: Driver Init failed\n", LIS3DH_ACC_DEV_NAME);
	return err;
}
static int lis3dh_acc_remove(struct i2c_client *client)
{
	/* TODO: revisit ordering here once _probe order is finalized */
	struct lis3dh_acc_data *acc = i2c_get_clientdata(client);
	if (acc != NULL) {
		if (acc->irq1) {
			free_irq(acc->irq1, acc);
			gpio_free(GSENSOR_GINT);
		}
		if (acc->irq2) {
			free_irq(acc->irq2, acc);
			gpio_free(GSENSOR_GINT2_GPI);
		}
		if (acc->irq1_work_queue)
			destroy_workqueue(acc->irq1_work_queue);
		if (acc->irq2_work_queue)
			destroy_workqueue(acc->irq2_work_queue);
		misc_deregister(&lis3dh_acc_misc_device);
		lis3dh_acc_input_cleanup(acc);
		lis3dh_acc_device_power_off(acc);
		kfree(acc->pdata);
		kfree(acc);
	}
	return 0;
}
static int lis3dh_acc_resume(struct i2c_client *client)
{
	struct lis3dh_acc_data *acc = i2c_get_clientdata(client);
	dev_dbg(&client->dev, "###%s###\n", __func__);
	if (acc != NULL && acc->on_before_suspend)
		return lis3dh_acc_enable(acc);
	return 0;
}
static int lis3dh_acc_suspend(struct i2c_client *client, pm_message_t mesg)
{
	struct lis3dh_acc_data *acc = i2c_get_clientdata(client);
	dev_dbg(&client->dev, "###%s###\n", __func__);
	if (acc != NULL) {
		acc->on_before_suspend = atomic_read(&acc->enabled);
		return lis3dh_acc_disable(acc);
	}
	return 0;
}
#ifdef CONFIG_HAS_EARLYSUSPEND
static void lis3dh_early_suspend(struct early_suspend *es)
{
	lis3dh_acc_suspend(lis3dh_i2c_client, (pm_message_t) {
			   .event = 0});
}
static void lis3dh_early_resume(struct early_suspend *es)
{
	lis3dh_acc_resume(lis3dh_i2c_client);
}
#endif /* CONFIG_HAS_EARLYSUSPEND */
static const struct i2c_device_id lis3dh_acc_id[]
= { {LIS3DH_ACC_DEV_NAME, 0}, {}, };
MODULE_DEVICE_TABLE(i2c, lis3dh_acc_id);
static const struct of_device_id lis3dh_of_match[] = {
	{.compatible = "st,BBLIS3DH",},
	{},
};
MODULE_DEVICE_TABLE(of, lis3dh_of_match);

static struct i2c_driver lis3dh_acc_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = LIS3DH_ACC_I2C_NAME,
		.of_match_table = lis3dh_of_match,
	},
	.probe = lis3dh_acc_probe,
	.remove = lis3dh_acc_remove,
	.resume = lis3dh_acc_resume,
	.suspend = lis3dh_acc_suspend,
	.id_table = lis3dh_acc_id
};
static int __init lis3dh_acc_init(void)
{
	pr_debug("%s accelerometer driver: init\n", LIS3DH_ACC_I2C_NAME);
	return i2c_add_driver(&lis3dh_acc_driver);
}
static void __exit lis3dh_acc_exit(void)
{
	pr_debug("%s accelerometer driver exit\n", LIS3DH_ACC_DEV_NAME);
	i2c_del_driver(&lis3dh_acc_driver);
	return;
}

module_init(lis3dh_acc_init);
module_exit(lis3dh_acc_exit);
MODULE_DESCRIPTION("lis3dh driver");
MODULE_AUTHOR("Melwyn Lobo<melwyn.lobo@gmail.com>");
MODULE_LICENSE("GPL");
