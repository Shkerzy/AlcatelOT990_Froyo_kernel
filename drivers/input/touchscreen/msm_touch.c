/* drivers/input/touchscreen/msm_touch.c
 *
 * Copyright (c) 2008-2009, Code Aurora Forum. All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include <linux/miscdevice.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/platform_device.h>
#include <linux/jiffies.h>
#include <linux/io.h>

#include <mach/msm_touch.h>
#include <asm/uaccess.h>
/* HW register map */
#define TSSC_CTL_REG      0x100
#define TSSC_SI_REG       0x108
#define TSSC_OPN_REG      0x104
#define TSSC_STATUS_REG   0x10C
#define TSSC_AVG12_REG    0x110

/* status bits */
#define TSSC_STS_OPN_SHIFT 0x6
#define TSSC_STS_OPN_BMSK  0x1C0
#define TSSC_STS_NUMSAMP_SHFT 0x1
#define TSSC_STS_NUMSAMP_BMSK 0x3E

/* CTL bits */
#define TSSC_CTL_EN		(0x1 << 0)
#define TSSC_CTL_SW_RESET	(0x1 << 2)
#define TSSC_CTL_MASTER_MODE	(0x3 << 3)
#define TSSC_CTL_AVG_EN		(0x1 << 5)
#define TSSC_CTL_DEB_EN		(0x1 << 6)
#define TSSC_CTL_DEB_12_MS	(0x2 << 7)	/* 1.2 ms */
#define TSSC_CTL_DEB_16_MS	(0x3 << 7)	/* 1.6 ms */
#define TSSC_CTL_DEB_2_MS	(0x4 << 7)	/* 2 ms */
#define TSSC_CTL_DEB_3_MS	(0x5 << 7)	/* 3 ms */
#define TSSC_CTL_DEB_4_MS	(0x6 << 7)	/* 4 ms */
#define TSSC_CTL_DEB_6_MS	(0x7 << 7)	/* 6 ms */
#define TSSC_CTL_INTR_FLAG1	(0x1 << 10)
#define TSSC_CTL_DATA		(0x1 << 11)
#define TSSC_CTL_SSBI_CTRL_EN	(0x1 << 13)

/* control reg's default state */
#define TSSC_CTL_STATE	  ( \
		TSSC_CTL_DEB_12_MS | \
		TSSC_CTL_DEB_EN | \
		TSSC_CTL_AVG_EN | \
		TSSC_CTL_MASTER_MODE | \
		TSSC_CTL_EN)

#define TSSC_NUMBER_OF_OPERATIONS 2
#define TS_PENUP_TIMEOUT_MS 30

#define TS_DRIVER_NAME "msm_touchscreen"

#define X_MAX	1024
#define Y_MAX	1024
#define P_MAX	256

#define TSIOSETPOINT   10
#define TSIOSETCAL     11

struct ts {
	struct input_dev *input;
	struct timer_list timer;
	int irq;
	unsigned int x_max;
	unsigned int y_max;
};

/* AJ define this flag in case we want to perform calibration in the driver */
/* calibration is now done in RawInputEvent.java */
//#define CALIBRATION


static void __iomem *virt;
#define TSSC_REG(reg) (virt + TSSC_##reg##_REG)

typedef struct Point {
	int x;
	int y;
}mpoint;
typedef struct Matrix {
	int An;
	int Bn;
	int Cn;
	int Dn;
	int En;
	int Fn;
	int Divider;
}matrix;
matrix mMtrx;
mpoint sPtr[3] = {
	{
		660,	/*top left*/
		300,
	}, {
		377,	/*bot right*/
		730,
	}, {
		660,	/*bot left*/
		732,
	}
};
mpoint dPtr[3] = {
	{
		50,	/*top left*/
		50,
	}, {
		190,	/*bot right*/
		270,
	}, {
		50,	/*bot left*/
		270,
	}
};
static int phy_logic_flg =0;
int calibration_set_matrix( mpoint * displayPtr,
		mpoint * screenPtr,
		matrix * matrixPtr)
{
	int  retvalue = 0 ;
	matrixPtr->Divider = ((screenPtr[0].x - screenPtr[2].x) * (screenPtr[1].y - screenPtr[2].y)) -
			     ((screenPtr[1].x - screenPtr[2].x) * (screenPtr[0].y - screenPtr[2].y)) ;
	if( matrixPtr->Divider == 0 )
		retvalue = -1 ;
	else{
		matrixPtr->An = ((displayPtr[0].x - displayPtr[2].x) * (screenPtr[1].y - screenPtr[2].y)) -
  				((displayPtr[1].x - displayPtr[2].x) * (screenPtr[0].y - screenPtr[2].y)) ;
  		matrixPtr->Bn = ((screenPtr[0].x - screenPtr[2].x) * (displayPtr[1].x - displayPtr[2].x)) -
   				((displayPtr[0].x - displayPtr[2].x) * (screenPtr[1].x - screenPtr[2].x)) ;
  		matrixPtr->Cn = (screenPtr[2].x * displayPtr[1].x - 
				 screenPtr[1].x * displayPtr[2].x) * screenPtr[0].y +
   				(screenPtr[0].x * displayPtr[2].x - 
				 screenPtr[2].x * displayPtr[0].x) * screenPtr[1].y +
   				(screenPtr[1].x * displayPtr[0].x - 
				 screenPtr[0].x * displayPtr[1].x) * screenPtr[2].y ;
  		matrixPtr->Dn = ((displayPtr[0].y - displayPtr[2].y) * (screenPtr[1].y - screenPtr[2].y)) -
   				((displayPtr[1].y - displayPtr[2].y) * (screenPtr[0].y - screenPtr[2].y)) ;
  		matrixPtr->En = ((screenPtr[0].x - screenPtr[2].x) * (displayPtr[1].y - displayPtr[2].y)) -
   				((displayPtr[0].y - displayPtr[2].y) * (screenPtr[1].x - screenPtr[2].x)) ;
  		matrixPtr->Fn = (screenPtr[2].x * displayPtr[1].y - 
				 screenPtr[1].x * displayPtr[2].y) * screenPtr[0].y +
   				(screenPtr[0].x * displayPtr[2].y - 
				 screenPtr[2].x * displayPtr[0].y) * screenPtr[1].y +
   				(screenPtr[1].x * displayPtr[0].y - 
				 screenPtr[0].x * displayPtr[1].y) * screenPtr[2].y ;
 	}
 	return( retvalue ) ;
} /* end of calibration_set_matrix() */

int display_get_point( mpoint * displayPtr,
		mpoint * screenPtr,
		matrix * matrixPtr )
{
	int  retvalue = 0 ;
	if( matrixPtr->Divider != 0 ){
		/* Operation order is important since we are doing integer */
		/*  math. Make sure you add all terms together before      */
		/*  dividing, so that the remainder is not rounded off     */
		/*  prematurely.                                           */
		displayPtr->x = ( (matrixPtr->An * screenPtr->x) +
				(matrixPtr->Bn * screenPtr->y) +
				matrixPtr->Cn
				) / matrixPtr->Divider ;
		displayPtr->y = ( (matrixPtr->Dn * screenPtr->x) +
				(matrixPtr->En * screenPtr->y) +
				matrixPtr->Fn
				) / matrixPtr->Divider ;
	} else
		retvalue = -1 ;
	return retvalue ;
} /* end of display_get_point() */ 

static void ts_update_pen_state(struct ts *ts, int x, int y, int pressure)
{
	if (pressure) {
		input_report_abs(ts->input, ABS_X, x);
		input_report_abs(ts->input, ABS_Y, y);
		input_report_abs(ts->input, ABS_PRESSURE, pressure);
		input_report_key(ts->input, BTN_TOUCH, !!pressure);
	} else {
		input_report_abs(ts->input, ABS_PRESSURE, 0);
		input_report_key(ts->input, BTN_TOUCH, 0);
	}

	input_sync(ts->input);
}

static void ts_timer(unsigned long arg)
{
	struct ts *ts = (struct ts *)arg;

	ts_update_pen_state(ts, 0, 0, 0);
}

static irqreturn_t ts_interrupt(int irq, void *dev_id)
{
	u32 avgs, x, y, lx, ly;
	u32 num_op, num_samp;
	u32 status;
	 mpoint disPtr;
 	mpoint scrPtr;

	struct ts *ts = dev_id;

	status = readl(TSSC_REG(STATUS));
	avgs = readl(TSSC_REG(AVG12));
	x = avgs & 0xFFFF;
	y = avgs >> 16;

	/* For pen down make sure that the data just read is still valid.
	 * The DATA bit will still be set if the ARM9 hasn't clobbered
	 * the TSSC. If it's not set, then it doesn't need to be cleared
	 * here, so just return.
	 */
	if (!(readl(TSSC_REG(CTL)) & TSSC_CTL_DATA))
		goto out;

	/* Data has been read, OK to clear the data flag */
	writel(TSSC_CTL_STATE, TSSC_REG(CTL));

	/* Valid samples are indicated by the sample number in the status
	 * register being the number of expected samples and the number of
	 * samples collected being zero (this check is due to ADC contention).
	 */
	num_op = (status & TSSC_STS_OPN_BMSK) >> TSSC_STS_OPN_SHIFT;
	num_samp = (status & TSSC_STS_NUMSAMP_BMSK) >> TSSC_STS_NUMSAMP_SHFT;

	if ((num_op == TSSC_NUMBER_OF_OPERATIONS) && (num_samp == 0)) {
		/* TSSC can do Z axis measurment, but driver doesn't support
		 * this yet.
		 */

		/*
		 * REMOVE THIS:
		 * These x, y co-ordinates adjustments will be removed once
		 * Android framework adds calibration framework.
		 */
#ifdef CONFIG_ANDROID_TOUCHSCREEN_MSM_HACKS
		lx = ts->x_max - x;
		ly = ts->y_max - y;
#else
		lx = x;
		ly = y;
#endif
		if (phy_logic_flg == 0) {
			scrPtr.x = lx;
   			scrPtr.y = ly;
   			display_get_point(&disPtr, &scrPtr, &mMtrx);
			ts_update_pen_state(ts, disPtr.x, disPtr.y, 255);
			/*printk("logic:x=%d,y=%d\n",disPtr.x, disPtr.y);*/
		} else {
			ts_update_pen_state(ts, lx, ly, 255);
			/*printk("phy:x=%d,y=%d\n",lx, ly);*/
		}	
		mod_timer(&ts->timer,
			jiffies + msecs_to_jiffies(TS_PENUP_TIMEOUT_MS));

	} else
		printk(KERN_INFO "Ignored interrupt: {%3d, %3d},"
				" op = %3d samp = %3d\n",
				 x, y, num_op, num_samp);

out:
	return IRQ_HANDLED;
}
#ifdef CALIBRATION
static int ts_calibration_open(struct inode *inode, struct file *file)
{
	file->private_data = NULL;
	printk("ts_calibration_open\n");
	return 0;
}

static int ts_calibration_release(struct inode *inode, struct file *file)
{
	return 0;
}

static int
ts_calibration_ioctl(struct inode *inode, struct file *file, unsigned int cmd, 
						unsigned long arg)
{
	int __user *cal = (int __user *)arg;
	int point[6];
	int value = 0;
	switch(cmd) {
		case TSIOSETPOINT:
			if(copy_from_user(&point, cal, 6 * sizeof(int)))
				return -EFAULT;
			sPtr[0].x = point[0];
			sPtr[0].y = point[1];
			sPtr[1].x = point[2];
			sPtr[1].y = point[3];
			sPtr[2].x = point[4];
			sPtr[2].y = point[5];
			printk("top left:(%d,%d),bot right:(%d,%d),bot right:(%d,%d)\n",
				point[0],point[1],point[2],point[3],point[4],point[5]);
			/*get this value from calibration's file*/
			calibration_set_matrix(dPtr,sPtr,&mMtrx);
			return 0;	
		case TSIOSETCAL:
			if(copy_from_user(&value, cal, sizeof(int)))
				return -EFAULT;
			phy_logic_flg = value;
			/*printk("phy_logic_flg=%d\n", value);*/
			return 0;
		default:
			return -ENOIOCTLCMD;	
	}
	return 0;
}

static struct file_operations ts_calibration_fops = {
	.owner = THIS_MODULE,
	.ioctl = ts_calibration_ioctl,
	.open = ts_calibration_open,
	.release = ts_calibration_release,
};

static struct miscdevice ts_calibration_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "ts_calibration",
	.fops = &ts_calibration_fops,
};
#endif
static int __devinit ts_probe(struct platform_device *pdev)
{
	int result;
	struct input_dev *input_dev;
	struct resource *res, *ioarea;
	struct ts *ts;
	unsigned int x_max, y_max, pressure_max;
	struct msm_ts_platform_data *pdata = pdev->dev.platform_data;

	/* The primary initialization of the TS Hardware
	 * is taken care of by the ADC code on the modem side
	 */

	ts = kzalloc(sizeof(struct ts), GFP_KERNEL);
	input_dev = input_allocate_device();
	if (!input_dev || !ts) {
		result = -ENOMEM;
		goto fail_alloc_mem;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "Cannot get IORESOURCE_MEM\n");
		result = -ENOENT;
		goto fail_alloc_mem;
	}

	ts->irq = platform_get_irq(pdev, 0);
	if (!ts->irq) {
		dev_err(&pdev->dev, "Could not get IORESOURCE_IRQ\n");
		result = -ENODEV;
		goto fail_alloc_mem;
	}

	ioarea = request_mem_region(res->start, resource_size(res), pdev->name);
	if (!ioarea) {
		dev_err(&pdev->dev, "Could not allocate io region\n");
		result = -EBUSY;
		goto fail_alloc_mem;
	}

	virt = ioremap(res->start, resource_size(res));
	if (!virt) {
		dev_err(&pdev->dev, "Could not ioremap region\n");
		result = -ENOMEM;
		goto fail_ioremap;
	}

	input_dev->name = TS_DRIVER_NAME;
	input_dev->phys = "msm_touch/input0";
	input_dev->id.bustype = BUS_HOST;
	input_dev->id.vendor = 0x0001;
	input_dev->id.product = 0x0002;
	input_dev->id.version = 0x0100;
	input_dev->dev.parent = &pdev->dev;

	input_dev->evbit[0] = BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);
	input_dev->absbit[0] = BIT(ABS_X) | BIT(ABS_Y) | BIT(ABS_PRESSURE);
	input_dev->absbit[BIT_WORD(ABS_MISC)] = BIT_MASK(ABS_MISC);
	input_dev->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);

	if (pdata) {
		x_max = pdata->x_max ? : X_MAX;
		y_max = pdata->y_max ? : Y_MAX;
		pressure_max = pdata->pressure_max ? : P_MAX;
	} else {
		x_max = X_MAX;
		y_max = Y_MAX;
		pressure_max = P_MAX;
	}

	ts->x_max = x_max;
	ts->y_max = y_max;
#ifndef CALIBRATION
	phy_logic_flg = 1;	/*physical value*/
#else
	phy_logic_flg = 0;	/*logic value*/
	calibration_set_matrix(dPtr,sPtr,&mMtrx);
	misc_register(&ts_calibration_device);
	printk("add ts_calibration device\n");
#endif
	input_set_abs_params(input_dev, ABS_X, 0, 240, 0, 0);
	input_set_abs_params(input_dev, ABS_Y, 0, 320, 0, 0);
	input_set_abs_params(input_dev, ABS_PRESSURE, 0, pressure_max, 0, 0);

	result = input_register_device(input_dev);
	if (result)
		goto fail_ip_reg;

	ts->input = input_dev;

	setup_timer(&ts->timer, ts_timer, (unsigned long)ts);
	result = request_irq(ts->irq, ts_interrupt, IRQF_TRIGGER_RISING,
				 "touchscreen", ts);
	if (result)
		goto fail_req_irq;

	platform_set_drvdata(pdev, ts);
	
	return 0;

fail_req_irq:
	input_unregister_device(input_dev);
	input_dev = NULL;
fail_ip_reg:
	iounmap(virt);
fail_ioremap:
	release_mem_region(res->start, resource_size(res));
fail_alloc_mem:
	input_free_device(input_dev);
	kfree(ts);
	return result;
}

static int __devexit ts_remove(struct platform_device *pdev)
{
	struct resource *res;
	struct ts *ts = platform_get_drvdata(pdev);

	free_irq(ts->irq, ts);
	del_timer_sync(&ts->timer);
#ifdef CALIBRATION
	misc_deregister(&ts_calibration_device);
#endif
	input_unregister_device(ts->input);
	iounmap(virt);
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	release_mem_region(res->start, resource_size(res));
	platform_set_drvdata(pdev, NULL);
	kfree(ts);

	return 0;
}

static struct platform_driver ts_driver = {
	.probe		= ts_probe,
	.remove		= __devexit_p(ts_remove),
	.driver		= {
		.name = TS_DRIVER_NAME,
		.owner = THIS_MODULE,
	},
};

static int __init ts_init(void)
{
	return platform_driver_register(&ts_driver);
}
module_init(ts_init);

static void __exit ts_exit(void)
{
	platform_driver_unregister(&ts_driver);
}
module_exit(ts_exit);

MODULE_DESCRIPTION("MSM Touch Screen driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:msm_touchscreen");
