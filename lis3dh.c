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

#define	LIS3DH_GACC_MAX 16000 
#define LIS3DH_SENS_2G 1	/*mg/LSB*/
#define LIS3DH_SENS_4G 2	/*mg/LSB*/
#define LIS3DH_SENS_8G 4	/*mg/LSB*/
#define LIS3DH_SENS_16G 12	/*mg/LSB*/
#define	HIGH_RESOLUTION 0x08
#define	LXACCELL_AXISDATA_REG 0x28
#define WHOAMI_LIS3DH_ACC 0x33

#define LXACCELL_POWON	0x47
#define WHO_AM_I 0x0F
#define	LXACCELL_TEMPERATURE_CONFIG_REG 0x1F

#define	LXACCELL_CONTROL_REG1 0x20
#define	LXACCELL_CONTROL_REG2 0x21
#define	LXACCELL_CONTROL_REG3 0x22
#define	LXACCELL_CONTROL_REG4 0x23
#define	LXACCELL_CONTROL_REG5 0x24
#define	LXACCELL_CONTROL_REG6 0x25
#define	LXACCELL_FIFO_CONTROL_REG 0x2E
#define	LXACCELL_INT_CFG1 0x30
#define	LXACCELL_INT_SRC1 0x31
#define	LXACCELL_INT_THS1 0x32
#define	LXACCELL_INT_DUR1 0x33
#define	LXACCELL_INT_CFG2 0x34
#define	LXACCELL_INT_SRC2 0x35
#define	LXACCELL_INT_THS2 0x36
#define	LXACCELL_INT_DUR2 0x37

#define HI_RES_EN	1
#define LXACCELL_ACC_PM_OFF 0x00
#define LXACCELL_ACC_ENABLE_ALL_AXES 0x07
#define LXACCELL_PMMODE_MASK 0x08

struct lxaccell_lis3dh_data {
	struct i2c_client *client;
	struct lxaccell_platform_data *pdata;
	struct mutex lock;
	struct delayed_work input_work;
	struct input_dev *input_dev;
	int hw_initialized;
	/* hw_working=-1 means not tested yet */
	int hw_working;
	atomic_t enabled;
	int on_before_suspend;
	u8 sensitivity;
	u8 register_area[SAVE];
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

struct lxaccell_lis3dh_data *lis3dh_acc_misc_data;
struct i2c_client *lis3dh_i2c_client;
static int lis3dh_acc_i2c_read(struct lxaccell_lis3dh_data *lxaccell_data, u8 *buf, int len)
{
	int err;
	int tries = 0;
	struct i2c_msg msgs[] = {
		{
			.addr = lxaccell_data->client->addr,
			.flags = lxaccell_data->client->flags & I2C_M_TEN,
			.len = 1,
			.buf = buf,},
		{
			.addr = lxaccell_data->client->addr,
			.flags = (lxaccell_data->client->flags & I2C_M_TEN) | I2C_M_RD,
			.len = len,
			.buf = buf,},
	};
	do {
		err = i2c_transfer(lxaccell_data->client->adapter, msgs, 2);
		if (err != 2)
			msleep_interruptible(I2C_RETRY_DELAY);
	} while ((err != 2) && (++tries < I2C_RETRIES));
	if (err != 2) {
		dev_err(&lxaccell_data->client->dev, "read transfer error\n");
		err = -EIO;
	} else
		err = 0;
	return err;
}
static int lis3dh_acc_i2c_write(struct lxaccell_lis3dh_data *lxaccell_data, u8 *buf, int len)
{
	int err;
	int tries = 0;
	struct i2c_msg msgs[] = {
		{
			.addr = lxaccell_data->client->addr,
			.flags = lxaccell_data->client->flags & I2C_M_TEN,
			.len = len + 1,
			.buf = buf,
		},
	};
	do {
		err = i2c_transfer(lxaccell_data->client->adapter, msgs, 1);
		if (err != 1)
			msleep_interruptible(I2C_RETRY_DELAY);
	} while ((err != 1) && (++tries < I2C_RETRIES));
	if (err != 1) {
		dev_err(&lxaccell_data->client->dev, "write transfer error\n");
		err = -EIO;
	} else {
		err = 0;
	}
	return err;
}
static int lis3dh_acc_hw_init(struct lxaccell_lis3dh_data *lxaccell_data)
{
	int err = -1;
	u8 buf[7];
	pr_debug("%s: hw init start\n", LIS3DH_ACC_DEV_NAME);
	buf[0] = WHO_AM_I;
	err = lis3dh_acc_i2c_read(lxaccell_data, buf, 1);
	if (err < 0)
		goto error_firstread;
	else
		lxaccell_data->hw_working = 1;
	if (buf[0] != WHOAMI_LIS3DH_ACC) {
		err = -1;	/* choose the right coded error */
		goto error_unknown_device;
	}
	buf[0] = LXACCELL_CONTROL_REG1;
	buf[1] = lxaccell_data->register_area[RES_CONTROL_REG1];
	err = lis3dh_acc_i2c_write(lxaccell_data, buf, 1);
	if (err < 0)
		goto error1;
	buf[0] = LXACCELL_TEMPERATURE_CONFIG_REG;
	buf[1] = lxaccell_data->register_area[RES_TEMPERATURE_CONFIG_REG];
	err = lis3dh_acc_i2c_write(lxaccell_data, buf, 1);
	if (err < 0)
		goto error1;
	buf[0] = LXACCELL_FIFO_CONTROL_REG;
	buf[1] = lxaccell_data->register_area[RES_FIFO_CONTROL_REG];
	err = lis3dh_acc_i2c_write(lxaccell_data, buf, 1);
	if (err < 0)
		goto error1;

	buf[0] = (I2C_AUTO_INCREMENT | LXACCELL_INT_THS1);
	buf[1] = lxaccell_data->register_area[RES_INT_THS1];
	buf[2] = lxaccell_data->register_area[RES_INT_DUR1];
	err = lis3dh_acc_i2c_write(lxaccell_data, buf, 2);
	if (err < 0)
		goto error1;
	buf[0] = LXACCELL_INT_CFG1;
	buf[1] = lxaccell_data->register_area[RES_INT_CFG1];
	err = lis3dh_acc_i2c_write(lxaccell_data, buf, 1);
	if (err < 0)
		goto error1;
	buf[0] = (I2C_AUTO_INCREMENT | LXACCELL_INT_THS2);
	buf[1] = lxaccell_data->register_area[RES_INT_THS2];
	buf[2] = lxaccell_data->register_area[RES_INT_DUR2];
	err = lis3dh_acc_i2c_write(lxaccell_data, buf, 2);
	if (err < 0)
		goto error1;
	buf[0] = LXACCELL_INT_CFG2;
	buf[1] = lxaccell_data->register_area[RES_INT_CFG2];
	err = lis3dh_acc_i2c_write(lxaccell_data, buf, 1);
	if (err < 0)
		goto error1;
	buf[0] = (I2C_AUTO_INCREMENT | LXACCELL_CONTROL_REG2);
	buf[1] = lxaccell_data->register_area[RES_CONTROL_REG2];
	buf[2] = lxaccell_data->register_area[RES_CONTROL_REG3];
	buf[3] = lxaccell_data->register_area[RES_CONTROL_REG4];
	buf[4] = lxaccell_data->register_area[RES_CONTROL_REG5];
	buf[5] = lxaccell_data->register_area[RES_CONTROL_REG6];
	err = lis3dh_acc_i2c_write(lxaccell_data, buf, 5);
	if (err < 0)
		goto error1;
	lxaccell_data->hw_initialized = 1;
	pr_debug("%s: hw init done\n", LIS3DH_ACC_DEV_NAME);
	return 0;
error_firstread:
	lxaccell_data->hw_working = 0;
	dev_warn(&lxaccell_data->client->dev, "Error reading available/working?\n");
	goto error1;
error_unknown_device:
	dev_err(&lxaccell_data->client->dev,
		"device unknown. Expected: 0x%x,Replies: 0x%x\n",
		WHOAMI_LIS3DH_ACC, buf[0]);
error1:
	lxaccell_data->hw_initialized = 0;
	dev_err(&lxaccell_data->client->dev, "hw init error 0x%x,0x%x: %d\n", buf[0],
		buf[1], err);
	return err;
}
static void lis3dh_acc_device_power_off(struct lxaccell_lis3dh_data *lxaccell_data)
{
	int err;
	u8 buf[2] = { LXACCELL_CONTROL_REG1, LXACCELL_ACC_PM_OFF };
	err = lis3dh_acc_i2c_write(lxaccell_data, buf, 1);
	if (err < 0)
		dev_err(&lxaccell_data->client->dev, "soft power off failed: %d\n", err);
	if (lxaccell_data->hw_initialized) {
		if (lxaccell_data->irq1 != 0)
			disable_irq_nosync(lxaccell_data->irq1);
		lxaccell_data->hw_initialized = 0;
	}
}
static int lis3dh_acc_device_power_on(struct lxaccell_lis3dh_data *lxaccell_data)
{
	int err = -1;
	if (!lxaccell_data->hw_initialized) {
		err = lis3dh_acc_hw_init(lxaccell_data);
		if (lxaccell_data->hw_working == 1 && err < 0) {
			lis3dh_acc_device_power_off(lxaccell_data);
			return err;
		}
	}
	return 0;
}
static irqreturn_t lxacell_acc_interrupt_handler1(int irq, void *dev)
{
	struct lxaccell_lis3dh_data *lxaccell_data = dev;
	printk("%s: isr1 queued\n", LIS3DH_ACC_DEV_NAME);
	if (lxaccell_data->async_queue) {
		kill_fasync(&lxaccell_data->async_queue, SIGIO, POLL_IN);
	}

	return IRQ_HANDLED;
}

#ifdef USE_IRQ2
static irqreturn_t lxacell_acc_interrupt_handler2(int irq, void *dev)
{
	struct lxaccell_lis3dh_data *lxaccell_data = dev;
	disable_irq_nosync(irq);
	printk("%s: isr2 queued\n", LIS3DH_ACC_DEV_NAME);
	return IRQ_HANDLED;
}
#endif

static void lis3dh_acc_irq1_work_func(struct work_struct *work)
{
	/*struct lxaccell_lis3dh_data *lxaccell_data =
	    container_of(work, struct lxaccell_lis3dh_data, irq1_work);
	*/
	pr_debug("%s: IRQ1 triggered\n", LIS3DH_ACC_DEV_NAME);
}

int lis3dh_acc_update_g_range(struct lxaccell_lis3dh_data *lxaccell_data, u8 new_g_range)
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
		sensitivity = LIS3DH_SENS_2G;
		break;
	case LIS3DH_ACC_G_4G:
		sensitivity = LIS3DH_SENS_4G;
		break;
	case LIS3DH_ACC_G_8G:
		sensitivity = LIS3DH_SENS_8G;
		break;
	case LIS3DH_ACC_G_16G:
		sensitivity = LIS3DH_SENS_16G;
		break;
	default:
		dev_err(&lxaccell_data->client->dev, "invalid g range requested: %u\n",
			new_g_range);
		return -EINVAL;
	}
	if (atomic_read(&lxaccell_data->enabled)) {
		/* Set configuration register 4, which contains g range setting
		 *  NOTE: this is a straight overwrite because this driver does
		 *  not use any of the other configuration bits in this
		 *  register.  Should this become untrue, we will have to read
		 *  out the value and only change the relevant bits --XX----
		 *  (marked by X) */
		buf[0] = LXACCELL_CONTROL_REG4;
		err = lis3dh_acc_i2c_read(lxaccell_data, buf, 1);
		if (err < 0)
			goto error;
		init_val = buf[0];
		lxaccell_data->register_area[RES_CONTROL_REG4] = init_val;
		new_val = new_g_range | HIGH_RESOLUTION;
		updated_val = ((mask & new_val) | ((~mask) & init_val));
		buf[1] = updated_val;
		buf[0] = LXACCELL_CONTROL_REG4;
		err = lis3dh_acc_i2c_write(lxaccell_data, buf, 1);
		if (err < 0)
			goto error;
		lxaccell_data->register_area[RES_CONTROL_REG4] = updated_val;
		lxaccell_data->sensitivity = sensitivity;
		printk("%s sensitivity %d g-range %d\n", __func__,
			sensitivity, new_g_range);
	}
	return 0;
error:
	dev_err(&lxaccell_data->client->dev, "update g range failed 0x%x,0x%x: %d\n",
		buf[0], buf[1], err);
	return err;
}
int lis3dh_acc_update_odr(struct lxaccell_lis3dh_data *lxaccell_data, int poll_interval_ms)
{
	int err = -1;
	int i;
	u8 config[2];
	/* Convert the poll interval into an output data rate configuration
	 *  that is as low as possible.  The ordering of these checks must be
	 *  maintained due to the cascading cut off values - poll intervals are
	 *  checked from shortest to longest.  At each check, if the next lower
	 *  LXACCELL_OUTPUT_RATE cannot support the current poll interval, we stop searching */
	for (i = ARRAY_SIZE(lxaccell_data_rate) - 1; i >= 0; i--) {
		if (lxaccell_data_rate[i].lxaccell_upper_ms <= poll_interval_ms)
			break;
	}
	config[1] = lxaccell_data_rate[i].mask;
	config[1] |= LXACCELL_ACC_ENABLE_ALL_AXES;
	/* If device is currently enabled, we need to write new
	 *  configuration out to it */
	if (atomic_read(&lxaccell_data->enabled)) {
		config[0] = LXACCELL_CONTROL_REG1;
		err = lis3dh_acc_i2c_write(lxaccell_data, config, 1);
		if (err < 0)
			goto error;
		lxaccell_data->register_area[RES_CONTROL_REG1] = config[1];
	}
	return 0;
error:
	dev_err(&lxaccell_data->client->dev, "update odr failed 0x%x,0x%x: %d\n",
		config[0], config[1], err);
	return err;
}

static int lis3dh_acc_get_acceleration_data(struct lxaccell_lis3dh_data *lxaccell_data,
					    int *xyz)
{
	int err = -1;
	/* Data bytes from hardware xL, xH, yL, yH, zL, zH */
	u8 acc_data[6];
	/* x,y,z hardware data */
	s16 hw_d[3] = { 0 };
	acc_data[0] = (I2C_AUTO_INCREMENT | LXACCELL_AXISDATA_REG);
	err = lis3dh_acc_i2c_read(lxaccell_data, acc_data, 6);
	if (err < 0) {
		pr_debug("%s I2C read error %d\n", LIS3DH_ACC_I2C_NAME,
		       err);
		return err;
	}
	hw_d[0] = (((s16) ((acc_data[1] << 8) | acc_data[0])) >> 4);
	hw_d[1] = (((s16) ((acc_data[3] << 8) | acc_data[2])) >> 4);
	hw_d[2] = (((s16) ((acc_data[5] << 8) | acc_data[4])) >> 4);
	hw_d[0] = hw_d[0] * lxaccell_data->sensitivity;
	hw_d[1] = hw_d[1] * lxaccell_data->sensitivity;
	hw_d[2] = hw_d[2] * lxaccell_data->sensitivity;
	 xyz[0] = ((lxaccell_data->pdata->negate_x) ? (-hw_d[lxaccell_data->pdata->axis_map_x])
		  : (hw_d[lxaccell_data->pdata->axis_map_x]));
	xyz[1] = ((lxaccell_data->pdata->negate_y) ? (-hw_d[lxaccell_data->pdata->axis_map_y])
		  : (hw_d[lxaccell_data->pdata->axis_map_y]));
	xyz[2] = ((lxaccell_data->pdata->negate_z) ? (-hw_d[lxaccell_data->pdata->axis_map_z])
		  : (hw_d[lxaccell_data->pdata->axis_map_z]));
	pr_debug("%s read x=%d, y=%d, z=%d\n",
	       LIS3DH_ACC_DEV_NAME, xyz[0], xyz[1], xyz[2]);
	return err;
}
static void lis3dh_acc_report_values(struct lxaccell_lis3dh_data *lxaccell_data, int *xyz)
{
	input_report_abs(lxaccell_data->input_dev, ABS_X, xyz[0]);
	input_report_abs(lxaccell_data->input_dev, ABS_Y, xyz[1]);
	input_report_abs(lxaccell_data->input_dev, ABS_Z, xyz[2]);
	input_sync(lxaccell_data->input_dev);
}
static int lis3dh_acc_enable(struct lxaccell_lis3dh_data *lxaccell_data)
{
	int err;
	if (!atomic_cmpxchg(&lxaccell_data->enabled, 0, 1)) {
		err = lis3dh_acc_device_power_on(lxaccell_data);
		if (err < 0) {
			atomic_set(&lxaccell_data->enabled, 0);
			return err;
		}
		if (lxaccell_data->hw_initialized) {
			if (lxaccell_data->irq1 != 0)
				enable_irq(lxaccell_data->irq1);
			if (lxaccell_data->irq2 != 0)
				enable_irq(lxaccell_data->irq2);
			pr_debug("%s: power on: irq enabled\n",
			       LIS3DH_ACC_DEV_NAME);
		}
		schedule_delayed_work(&lxaccell_data->input_work,
				      msecs_to_jiffies(lxaccell_data->pdata->
						       poll_interval));
	}
	return 0;
}
static int lis3dh_acc_disable(struct lxaccell_lis3dh_data *lxaccell_data)
{
	if (atomic_cmpxchg(&lxaccell_data->enabled, 1, 0)) {
		cancel_delayed_work_sync(&lxaccell_data->input_work);
		lis3dh_acc_device_power_off(lxaccell_data);
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
	struct lxaccell_lis3dh_data *lxaccell_data = filp->private_data;
    
    if (lxaccell_data && lxaccell_data->client) {

		status = lis3dh_acc_get_acceleration_data(lxaccell_data, xyz);
		if (status >= 0)
        	len = snprintf(buf, len, "%d,%d,%d\n", 
									xyz[0], xyz[1], xyz[2]);

    } else {
		printk("Unexpected error %p:%p:%p\n",
			lxaccell_data,lxaccell_data->client,lxaccell_data->client->adapter);
		status = -1;
    }
        
    return ((status==0) ? len : -EIO);
}

static ssize_t lis3dh_acc_misc_write(struct file *filp, const char __user *buf,
												size_t len, loff_t *off)
{
    int err = 0;
	u8 write[3] = { 0 };
	struct lxaccell_lis3dh_data *lxaccell_data = filp->private_data;
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
	write[0] = (I2C_AUTO_INCREMENT | LXACCELL_INT_THS1);
	lxaccell_data->register_area[RES_INT_THS1] = threshold;
	write[1] = lxaccell_data->register_area[RES_INT_THS1];
	write[2] = lxaccell_data->register_area[RES_INT_DUR1];
	err = lis3dh_acc_i2c_write(lxaccell_data, write, 2);
	if (err < 0)
		return 0;

	printk("Misc write Success\n");
    return len;
}
 
static int lis3dh_acc_misc_fasync(int fd, struct file *file, int mode)
{
    struct lxaccell_lis3dh_data *lxaccell_data = file->private_data;
    printk("Inside async\n");
    return fasync_helper(fd, file, mode, &lxaccell_data->async_queue);
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

void init_regs(struct lxaccell_lis3dh_data *lxaccell_data)
{
	memset(lxaccell_data->register_area, 0, ARRAY_SIZE(lxaccell_data->register_area));
	lxaccell_data->register_area[RES_CONTROL_REG1] = LXACCELL_ACC_ENABLE_ALL_AXES;
	lxaccell_data->register_area[RES_CONTROL_REG2] = 0x00;
	lxaccell_data->register_area[RES_CONTROL_REG3] = CONTROL_REG3_I1_AOI1;
	lxaccell_data->register_area[RES_CONTROL_REG4] = 0x00;
	lxaccell_data->register_area[RES_CONTROL_REG5] = 0x00;
	lxaccell_data->register_area[RES_CONTROL_REG6] = 0x00;
	lxaccell_data->register_area[RES_TEMPERATURE_CONFIG_REG] = 0x00;
	lxaccell_data->register_area[RES_FIFO_CONTROL_REG] = 0x00;
	lxaccell_data->register_area[RES_INT_CFG1] = 0x3F;
	lxaccell_data->register_area[RES_INT_THS1] = 0x04;//defualt 100
	lxaccell_data->register_area[RES_INT_DUR1] = 0x30;
	lxaccell_data->register_area[RES_INT_CFG2] = 0x00;
	lxaccell_data->register_area[RES_INT_THS2] = 0x00;
	lxaccell_data->register_area[RES_INT_DUR2] = 0x00;
	lxaccell_data->register_area[RES_TT_CFG] = 0x00;
	lxaccell_data->register_area[RES_TT_THS] = 0x00;
	lxaccell_data->register_area[RES_TT_LIM] = 0x00;
	lxaccell_data->register_area[RES_TT_TLAT] = 0x00;
	lxaccell_data->register_area[RES_TT_TW] = 0x00;
	lxaccell_data->pdata->g_range = 0;
}

static void lis3dh_acc_input_work_func(struct work_struct *work)
{
	struct lxaccell_lis3dh_data *lxaccell_data;
	int xyz[3] = { 0 };
	int err;
	lxaccell_data = container_of((struct delayed_work *)work,
			   struct lxaccell_lis3dh_data, input_work);
	mutex_lock(&lxaccell_data->lock);
	err = lis3dh_acc_get_acceleration_data(lxaccell_data, xyz);
	if (err < 0)
		dev_err(&lxaccell_data->client->dev, "get_acceleration_data failed\n");
	else
		lis3dh_acc_report_values(lxaccell_data, xyz);
	schedule_delayed_work(&lxaccell_data->input_work,
			      msecs_to_jiffies(lxaccell_data->pdata->poll_interval));
	mutex_unlock(&lxaccell_data->lock);
}
#ifdef LIS3DH_OPEN_ENABLE
int lis3dh_acc_input_open(struct input_dev *input)
{
	struct lxaccell_lis3dh_data *lxaccell_data = input_get_drvdata(input);
	return lis3dh_acc_enable(lxaccell_data);
}
void lis3dh_acc_input_close(struct input_dev *dev)
{
	struct lxaccell_lis3dh_data *lxaccell_data = input_get_drvdata(dev);
	lis3dh_acc_disable(lxaccell_data);
}
#endif
static int lis3dh_acc_input_init(struct lxaccell_lis3dh_data *lxaccell_data)
{
	int err;
	/* Polling rx data when the interrupt is not used.*/
	INIT_DELAYED_WORK(&lxaccell_data->input_work, lis3dh_acc_input_work_func);
	lxaccell_data->input_dev = input_allocate_device();
	if (!lxaccell_data->input_dev) {
		err = -ENOMEM;
		dev_err(&lxaccell_data->client->dev, "input device allocate failed\n");
		goto err0;
	}
#ifdef LIS3DH_ACC_OPEN_ENABLE
	lxaccell_data->input_dev->open = lis3dh_acc_input_open;
	lxaccell_data->input_dev->close = lis3dh_acc_input_close;
#endif
	input_set_drvdata(lxaccell_data->input_dev, lxaccell_data);
	set_bit(EV_ABS, lxaccell_data->input_dev->evbit);
	/*      next is used for interruptA sources data if the case */
	set_bit(ABS_MISC, lxaccell_data->input_dev->absbit);
	/*      next is used for interruptB sources data if the case */
	set_bit(ABS_WHEEL, lxaccell_data->input_dev->absbit);
	input_set_abs_params(lxaccell_data->input_dev, ABS_X, -LIS3DH_GACC_MAX, LIS3DH_GACC_MAX, FUZZ, FLAT);
	input_set_abs_params(lxaccell_data->input_dev, ABS_Y, -LIS3DH_GACC_MAX, LIS3DH_GACC_MAX, FUZZ, FLAT);
	input_set_abs_params(lxaccell_data->input_dev, ABS_Z, -LIS3DH_GACC_MAX, LIS3DH_GACC_MAX, FUZZ, FLAT);
	/*      next is used for interruptA sources data if the case */
	input_set_abs_params(lxaccell_data->input_dev, ABS_MISC, INT_MIN, INT_MAX, 0, 0);
	/*      next is used for interruptB sources data if the case */
	input_set_abs_params(lxaccell_data->input_dev, ABS_WHEEL, INT_MIN, INT_MAX, 0, 0);
	lxaccell_data->input_dev->name = "accelerometer";
	err = input_register_device(lxaccell_data->input_dev);
	if (err) {
		dev_err(&lxaccell_data->client->dev,
			"unable to register input polled device %s\n",
			lxaccell_data->input_dev->name);
		goto err1;
	}
	return 0;
err1:
	input_free_device(lxaccell_data->input_dev);
err0:
	return err;
}
static void lis3dh_acc_input_cleanup(struct lxaccell_lis3dh_data *lxaccell_data)
{
	input_unregister_device(lxaccell_data->input_dev);
	input_free_device(lxaccell_data->input_dev);
}
#ifdef CONFIG_HAS_EARLYSUSPEND
static void lis3dh_early_suspend(struct early_suspend *es);
static void lis3dh_early_resume(struct early_suspend *es);
#endif

int lis3dh_acc_power_on(struct i2c_client *client)
{
	struct lxaccell_lis3dh_data *lxaccell_data = i2c_get_clientdata(client);
    int status;
    u8 enable_power[] = {
        LXACCELL_CONTROL_REG1,
        LXACCELL_POWON
    };
    status = lis3dh_acc_i2c_write(lxaccell_data, enable_power, 2);
    return (status >= 0);
}

void init_open_firmware_node(struct i2c_client *client, 
					struct lxaccell_lis3dh_data *lxaccell_data)
{ 
	struct device_node *np;
 	u32 val;
	int err = -1;
	
	np = client->dev.of_node;
	if(np) 
		pr_debug("$$$$$$$$$$$$DEVICE OF NODE PROCESSING$$$$$$\n");
	else {
		pr_debug("$$$$$$$$$$$$DEVICE OF NODE MISSING$$$$$$\n");
		return;
	}
	err = of_property_read_u32(np, "poll_interval", &val);
	if (err) {
		dev_err(&client->dev,
			"dts config poll_interval missed");
		lxaccell_data->pdata->poll_interval = 200;
		} else
		lxaccell_data->pdata->poll_interval = val;
	err = of_property_read_u32(np, "min_interval", &val);
	if (err) {
		dev_err(&client->dev, "dts config min_interval missed");
		lxaccell_data->pdata->min_interval = 5;
	} else
		lxaccell_data->pdata->min_interval = val;
	err = of_property_read_u32(np, "g_range", &val);
	if (err) {
		dev_err(&client->dev, "dts config g_range missed");
		lxaccell_data->pdata->g_range = 0;
	} else
		lxaccell_data->pdata->g_range = val;
	err = of_property_read_u32(np, "axis_map_x", &val);
	if (err) {
		dev_err(&client->dev, "dts config axis_map_x missed");
		lxaccell_data->pdata->axis_map_x = 0;
	} else
		lxaccell_data->pdata->axis_map_x = val;
	err = of_property_read_u32(np, "axis_map_y", &val);
	if (err) {
		dev_err(&client->dev, "dts config axis_map_y missed");
		lxaccell_data->pdata->axis_map_y = 1;
	} else
		lxaccell_data->pdata->axis_map_y = val;
	err = of_property_read_u32(np, "axis_map_z", &val);
	if (err) {
		dev_err(&client->dev, "dts config axis_map_z missed");
		lxaccell_data->pdata->axis_map_z = 2;
	} else
		lxaccell_data->pdata->axis_map_z = val;
	err = of_property_read_u32(np, "negate_x", &val);
	if (err) {
		dev_err(&client->dev, "dts config negate_x  missed");
		lxaccell_data->pdata->negate_x = 1;
	} else
		lxaccell_data->pdata->negate_x = val;
	err = of_property_read_u32(np, "negate_y", &val);
	if (err) {
		dev_err(&client->dev, "dts config negate_y missed");
		lxaccell_data->pdata->negate_y = 1;
	} else
		lxaccell_data->pdata->negate_y = val;
	err = of_property_read_u32(np, "gpio_int1", &val);
	if (err) {
		dev_err(&client->dev, "dts config gpio_int1 missed");
		lxaccell_data->pdata->gpio_int1 = 0;
	} else
		lxaccell_data->pdata->gpio_int1 = val;
	err = of_property_read_u32(np, "gpio_int2", &val);
	if (err) {
		dev_err(&client->dev, "dts config gpio_int2 missed");
		lxaccell_data->pdata->gpio_int2 = 0;
	} else
		lxaccell_data->pdata->gpio_int2 = val;
}
static int lis3dh_acc_probe(struct i2c_client *client,const struct i2c_device_id *id)
{
	struct lxaccell_lis3dh_data *lxaccell_data;
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
	lxaccell_data = kzalloc(sizeof(struct lxaccell_lis3dh_data), GFP_KERNEL);
	if (lxaccell_data == NULL) {
		err = -ENOMEM;
		dev_err(&client->dev,
			"failed to allocate memory for module data: "
			"%d\n", err);
		goto exit_alloc_data_failed;
	}
	mutex_init(&lxaccell_data->lock);
	mutex_lock(&lxaccell_data->lock);
	lxaccell_data->client = client;
	lis3dh_i2c_client = client;
	i2c_set_clientdata(client, lxaccell_data);
	pr_debug("%s: %s has set irq1 to irq: %d\n",
	       LIS3DH_ACC_DEV_NAME, __func__, lxaccell_data->irq1);
	pr_debug("%s: %s has set irq2 to irq: %d\n",
	       LIS3DH_ACC_DEV_NAME, __func__, lxaccell_data->irq2);
	gpio_request(GPIO_INTERRUPT1, "GPIO_INTERRUPT1");
	gpio_direction_input(GPIO_INTERRUPT1);
	lxaccell_data->irq1 = gpio_to_irq(GPIO_INTERRUPT1);
	printk("GPIO %d is valid? %s\n", GPIO_INTERRUPT1, 
			gpio_is_valid(GPIO_INTERRUPT1) ? "yes" : "no");
	
	if (lxaccell_data->irq1 != 0) {
		pr_debug("%s request irq1\n", __func__);
		err =
		    request_irq(lxaccell_data->irq1, lxacell_acc_interrupt_handler1, IRQF_TRIGGER_RISING,
				"lis3dh_acc_irq1", lxaccell_data);
		if (err < 0) {
			dev_err(&client->dev, "request irq1 failed: %d\n", err);
			goto err_mutexunlockfreedata;
		}
		//disable_irq_nosync(lxaccell_data->irq1);
		INIT_WORK(&lxaccell_data->irq1_work, lis3dh_acc_irq1_work_func);
		lxaccell_data->irq1_work_queue =
		    create_singlethread_workqueue("lis3dh_acc_wq1");
		if (!lxaccell_data->irq1_work_queue) {
			err = -ENOMEM;
			dev_err(&client->dev, "cannot create work queue1: %d\n",
				err);
			goto err_free_irq1;
		}
	}
	
	lxaccell_data->pdata = kzalloc(sizeof(*lxaccell_data->pdata), GFP_KERNEL);
	if (lxaccell_data->pdata == NULL) {
		err = -ENOMEM;
		dev_err(&client->dev,
			"failed to allocate memory for pdata: %d\n", err);
		goto err_destoyworkqueue1;
	}
	if (client->dev.platform_data)
		lxaccell_data->pdata = client->dev.platform_data;

	init_open_firmware_node(lis3dh_i2c_client, lxaccell_data);
	
	err = lis3dh_acc_device_power_on(lxaccell_data);
	if (err < 0) {
		dev_err(&client->dev, "power on failed: %d\n", err);
		goto err2;
	}
	if (i2c_smbus_read_byte(client) < 0) {
		pr_err("i2c_smbus_read_byte error!!\n");
		goto err_destoyworkqueue1;
	} else
		pr_debug("%s Device detected!\n", LIS3DH_ACC_DEV_NAME);
	
	tempvalue = i2c_smbus_read_word_data(client, WHO_AM_I);
	if ((tempvalue & 0x00FF) == WHOAMI_LIS3DH_ACC)
		pr_debug("%s I2C driver registered!\n",
		       LIS3DH_ACC_DEV_NAME);
	else {
		lxaccell_data->client = NULL;
		pr_info("I2C driver not registered!Device unknown 0x%x\n",
		tempvalue);
		goto err_destoyworkqueue1;
	}
	i2c_set_clientdata(client, lxaccell_data);
	
	atomic_set(&lxaccell_data->enabled, 1);
	err = lis3dh_acc_update_g_range(lxaccell_data, lxaccell_data->pdata->g_range);
	if (err < 0) {
		dev_err(&client->dev, "update_g_range failed\n");
		goto err_power_off;
	}
	err = lis3dh_acc_update_odr(lxaccell_data, lxaccell_data->pdata->poll_interval);
	if (err < 0) {
		dev_err(&client->dev, "update_odr failed\n");
		goto err_power_off;
	}
	err = lis3dh_acc_input_init(lxaccell_data);
	if (err < 0) {
		dev_err(&client->dev, "input init failed\n");
		goto err_power_off;
	}
	lis3dh_acc_misc_data = lxaccell_data;
	err = misc_register(&lis3dh_acc_misc_device);
	if (err < 0) {
		dev_err(&client->dev,
			"misc LIS3DH_ACC_DEV_NAME register failed\n");
		goto err_input_cleanup;
	}
	
#ifdef CONFIG_HAS_EARLYSUSPEND
	lxaccell_data->early_suspend.suspend = lis3dh_early_suspend;
	lxaccell_data->early_suspend.resume = lis3dh_early_resume;
	lxaccell_data->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN;
	register_early_suspend(&lxaccell_data->early_suspend);
#endif
	mutex_unlock(&lxaccell_data->lock);
	dev_info(&client->dev, "###%s###\n", __func__);
	pr_info("%s -- success !\n", __func__);
	return 0;
err_input_cleanup:
	lis3dh_acc_input_cleanup(lxaccell_data);
err_power_off:
	lis3dh_acc_device_power_off(lxaccell_data);
err2:
	kfree(lxaccell_data->pdata);
err_destoyworkqueue1:
	if (lxaccell_data->irq1_work_queue)
		destroy_workqueue(lxaccell_data->irq1_work_queue);
err_free_irq1:
	if (lxaccell_data->irq1) {
		free_irq(lxaccell_data->irq1, lxaccell_data);
		gpio_free(GPIO_INTERRUPT1);
	}
err_mutexunlockfreedata:
	kfree(lxaccell_data);
	mutex_unlock(&lxaccell_data->lock);
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
	struct lxaccell_lis3dh_data *lxaccell_data = i2c_get_clientdata(client);
	if (lxaccell_data != NULL) {
		if (lxaccell_data->irq1) {
			free_irq(lxaccell_data->irq1, lxaccell_data);
			gpio_free(GPIO_INTERRUPT1);
		}
		if (lxaccell_data->irq1_work_queue)
			destroy_workqueue(lxaccell_data->irq1_work_queue);
		misc_deregister(&lis3dh_acc_misc_device);
		lis3dh_acc_input_cleanup(lxaccell_data);
		lis3dh_acc_device_power_off(lxaccell_data);
		kfree(lxaccell_data->pdata);
		kfree(lxaccell_data);
	}
	return 0;
}
static int lis3dh_acc_resume(struct i2c_client *client)
{
	struct lxaccell_lis3dh_data *lxaccell_data = i2c_get_clientdata(client);
	dev_dbg(&client->dev, "###%s###\n", __func__);
	if (lxaccell_data != NULL && lxaccell_data->on_before_suspend)
		return lis3dh_acc_enable(lxaccell_data);
	return 0;
}
static int lis3dh_acc_suspend(struct i2c_client *client, pm_message_t mesg)
{
	struct lxaccell_lis3dh_data *lxaccell_data = i2c_get_clientdata(client);
	dev_dbg(&client->dev, "###%s###\n", __func__);
	if (lxaccell_data != NULL) {
		lxaccell_data->on_before_suspend = atomic_read(&lxaccell_data->enabled);
		return lis3dh_acc_disable(lxaccell_data);
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
