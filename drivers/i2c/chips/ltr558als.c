/* Lite-On LTR-558ALS Linux Driver
 * *
 * * Copyright (C) 2011 Lite-On Technology Corp (Singapore)
 * *
 * * This program is distributed in the hope that it will be useful, but
 * * WITHOUT ANY WARRANTY; without even the implied warranty of
 * * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * *
 * */

#include <linux/delay.h>
#include <linux/earlysuspend.h>
#include <linux/fs.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/irq.h>
#include <linux/kernel.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/pm.h>
#include <linux/poll.h>
#include <linux/wakelock.h>
#include <asm/gpio.h>
#include <asm/uaccess.h>
#include "ltr558als.h"
#include <mach/msm_sensors_input_dev.h>
#include <linux/timer.h>
#include <linux/earlysuspend.h>


#define DRIVER_VERSION       "1.0"
#define DEVICE_NAME          "LTR_558ALS"
#define DEVICE_NAME_ALS      "ltr558_als"
#define DEVICE_NAME_PS       "ltr558_ps"

//timer function
#define ALS_TIMER_FUNC       1
#define PROX_TIMER_FUNC      1

static int ps_opened;
static int als_opened;
static struct wake_lock proximity_wake_lock;
static struct work_struct irq_workqueue;
static int ps_data_changed;
static int als_data_changed;
static int ps_active;
static int als_active;

static int ps_gainrange;
static int als_gainrange;

static int final_prox_val;
static int final_lux_val;

static int ltr558_irq;

static DECLARE_WAIT_QUEUE_HEAD(ps_waitqueue);
static DECLARE_WAIT_QUEUE_HEAD(als_waitqueue);

struct ltr558_data {
	struct i2c_client *client;
	struct mutex lock;
	struct work_struct   als_workqueue;
	struct work_struct   prox_workqueue;
#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend ltr558_early_suspend;
#endif
};

static struct ltr558_data *the_data = NULL;

// device configuration
struct ltr558_cfg *ltr558_cfgp;
static u8 prox_led_param      = 0x6d;
static u8 prox_pusle_param    = 0xcf;
static u8 als_meas_rate_param = 0x02;

// prox info
static struct ltr558_prox_info prox_cal_info[20];
static struct ltr558_prox_info prox_cur_info;
static struct ltr558_prox_info *prox_cur_infop = &prox_cur_info;
static u8 prox_history_hi = 0;
static u8 prox_history_lo = 0;
#if PROX_TIMER_FUNC
static struct timer_list prox_poll_timer;
#endif
static int prox_on = 0;
static u16 sat_prox = 0;

//als info
#if ALS_TIMER_FUNC
static struct timer_list als_poll_timer;
#endif
static int als_on;

extern struct mutex __msm_sensors_lock;

#if PROX_TIMER_FUNC
static void ltr558_prox_poll_timer_func(unsigned long param);
static void ltr558_prox_poll_timer_start(unsigned int t);
static void ltr558_prox_timer_del(void);
static void ltr558_prox_work_f(struct work_struct *work);
#endif

#if ALS_TIMER_FUNC
static void ltr558_als_poll_timer_func(unsigned long param);
static void ltr558_als_poll_timer_start(unsigned int t);
static void ltr558_als_timer_del(void);
static void ltr558_als_work_f(struct work_struct *work);
#endif
#ifdef CONFIG_HAS_EARLYSUSPEND
static void ltr558_als_early_suspend(struct early_suspend *h);
static void ltr558_als_late_resume(struct early_suspend *h);
#endif

// I2C Read
static int ltr558_i2c_read_reg(u8 regnum)
{
	int readdata;
	/*
	* * i2c_smbus_read_byte_data - SMBus "read byte" protocol
	* * @client: Handle to slave device
	* * @command: Byte interpreted by slave
	* *
	* * This executes the SMBus "read byte" protocol, returning negative errno
	* * else a data byte received from the device.
	* */
	readdata = i2c_smbus_read_byte_data(the_data->client, regnum);

	return readdata;
}

// I2C Write
static int ltr558_i2c_write_reg(u8 regnum, u8 value)
{
		int writeerror;
		/*
		 * * i2c_smbus_write_byte_data - SMBus "write byte" protocol
		 * * @client: Handle to slave device
		 * * @command: Byte interpreted by slave
		 * * @value: Byte being written
		 * *
		 * * This executes the SMBus "write byte" protocol, returning negative errno
		 * * else zero on success.
		 * */
		writeerror = i2c_smbus_write_byte_data(the_data->client, regnum, value);
		if (writeerror < 0)
			return writeerror;
		else
			return 0;
}

/*
 * * ###############
 * * ## PS CONFIG ##
 * * ###############
 * */

static int ltr558_ps_enable(int gainrange)
{
	int error;
	int setgain;

	switch(gainrange) {
	case PS_RANGE1:
		setgain = MODE_PS_ON_Gain1;
		break;
	case PS_RANGE2:
		setgain = MODE_PS_ON_Gain2;
		break;
	case PS_RANGE4:
		setgain = MODE_PS_ON_Gain4;
		break;
	case PS_RANGE8:
		setgain = MODE_PS_ON_Gain8;
		break;
	default:
		setgain = MODE_PS_ON_Gain1;
		break;
	}
	error = ltr558_i2c_write_reg(LTR558_PS_CONTR, setgain);
	if (error < 0)
		goto out;

	// prox led
	error = ltr558_i2c_write_reg(LTR558_PS_LED, ltr558_cfgp->prox_led);
	if (error < 0)
		goto out;

	// prox pulse
	error = ltr558_i2c_write_reg(LTR558_PS_N_PULSES, ltr558_cfgp->prox_pulse);
	if (error < 0)
		goto out;
	mdelay(WAKEUP_DELAY);
	/* ===============
	 * * ** IMPORTANT **
	 * * ===============
	 * * Other settings like timing and threshold to be set here, if required.
	 * * Not set and kept as device default for now.
	 * */

out:
	return error;
}

// Put PS into Standby mode
static int ltr558_ps_disable(void)
{
	int error;

	error = ltr558_i2c_write_reg(LTR558_PS_CONTR, MODE_PS_StdBy);

	prox_history_hi = 0;
	prox_history_lo = 0;

	return error;
}

static int ltr558_ps_read(void)
{
	u8 psval_lo, psval_hi;
	int psdata;

	psval_lo = ltr558_i2c_read_reg(LTR558_PS_DATA_0);
	if (psval_lo < 0) {
		psdata = psval_lo;
		goto out;
	}
	psval_hi = ltr558_i2c_read_reg(LTR558_PS_DATA_1);
	if (psval_hi < 0) {
		psdata = psval_hi;
		goto out;
	}
	psdata = ((psval_hi & 0x07) << 8) + psval_lo;

out:
	final_prox_val = psdata;

	return psdata;
}

static int ltr558_ps_read_status(void)
{
	int intval;

	intval = ltr558_i2c_read_reg(LTR558_ALS_PS_STATUS);
	if (intval < 0)
		goto out;
	intval = intval & 0x02;

out:
	return intval;
}

static int ltr558_ps_open(struct inode *inode, struct file *file)
{
	if (ps_opened) {
		printk(KERN_ALERT "%s: already opened\n", __func__);
		return -EBUSY;
	}
	ps_opened = 1;

	return 0;
}

static int ltr558_ps_release(struct inode *inode, struct file *file)
{
	printk(KERN_ALERT "%s\n", __func__);
	ps_opened = 0;

	return ltr558_ps_disable();
}

static int ltr558_ps_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg)
{
	int ret = 0;
	int buffer = 0;
	void __user *argp = (void __user *)arg;

	switch (cmd) {
	case LTR558_IOCTL_PS_ENABLE:
		if (copy_from_user(&buffer, argp, 1)) {
			ret = -EFAULT;
			break;
		}
		ps_active = buffer;
		if (ps_active) {
			ret = ltr558_ps_enable(ps_gainrange);
			if (ret < 0)
				break;
			else
				enable_irq_wake(MSM_GPIO_TO_INT(GPIO_INT_NO));
		}
		prox_on = 1;
#if PROX_TIMER_FUNC
		ltr558_prox_poll_timer_start(400);
#endif
		printk("XXXXXXXXX LTR558_LOCTL_PROX_ON XXXXXXXXXX\n");

		break;
	case LTR558_IOCTL_READ_PS_DATA:
		buffer = ltr558_ps_read();
		if (buffer < 0) {
			ret = -EFAULT;
			break;
		}
		ret = copy_to_user(argp, &buffer, 1);
		break;
	case LTR558_IOCTL_READ_PS_INT:
		buffer = ltr558_ps_read_status();
		if (&buffer < 0) {
			ret = -EFAULT;
			break;
		}
		ret = copy_to_user(argp, &buffer, 1);
		break;
	case LTR558_IOCTL_PS_DISABLE:
#if PROX_TIMER_FUNC	
		ltr558_prox_timer_del();
#endif
		ret = ltr558_ps_disable();
		prox_on = 0;

		printk("XXXXXXXXX LTR558_LOCTL_PROX_OFF XXXXXXXXXX\n");

		break;
	}

	return ret;
}

static unsigned int ltr558_ps_poll(struct file *fp, poll_table * wait)
{
	if(ps_data_changed) {
		ps_data_changed = 0;
		return POLLIN | POLLRDNORM;
	}
	poll_wait(fp, &ps_waitqueue, wait);

	return 0;
}

static struct file_operations ltr558_ps_fops = {
	.owner   = THIS_MODULE,
	.open    = ltr558_ps_open,
	.release = ltr558_ps_release,
	.ioctl   = ltr558_ps_ioctl,
	.poll    = ltr558_ps_poll,
};

static struct miscdevice ltr558_ps_dev = {
	.minor  = MISC_DYNAMIC_MINOR,
	.name   = DEVICE_NAME_PS,
	.fops   = &ltr558_ps_fops,
};

/*
* ################
* ## ALS CONFIG ##
* ################
**/

static int ltr558_als_enable(int gainrange)
{
	int error;
	if (gainrange == 1) {
		error = ltr558_i2c_write_reg(LTR558_ALS_CONTR, MODE_ALS_ON_Range1);
		if (error < 0)
			goto out;
	} else if (gainrange == 2) {
		error = ltr558_i2c_write_reg(LTR558_ALS_CONTR, MODE_ALS_ON_Range2);
		if (error < 0)
			goto out;
	} else {
		error = -1;
		goto out;
	}

	// set als measurement rate
	error = ltr558_i2c_write_reg(LTR558_ALS_MEAS_RATE, ltr558_cfgp->als_meas_rate);
	if (error < 0)
		goto out;

	mdelay(WAKEUP_DELAY);
	/* ===============
	 * * ** IMPORTANT **
	 * * ===============
	 * * Other settings like timing and threshold to be set here, if required.
	 * * Not set and kept as device default for now.
	 * */

out:
	return error;
}

// Put ALS into Standby mode
static int ltr558_als_disable(void)
{
	int error;

	error = ltr558_i2c_write_reg(LTR558_ALS_CONTR, MODE_ALS_StdBy);

	return error;
}

static int ltr558_als_read(int gainrange)
{
	u8 alsval_ch0_lo, alsval_ch0_hi;
	u8 alsval_ch1_lo, alsval_ch1_hi;
	int luxdata_int, alsval_ch0, alsval_ch1, alsval_ch;
	int ratio, luxdata_flt;
	int ch0_coeff, ch1_coeff;

	alsval_ch0_lo = ltr558_i2c_read_reg(LTR558_ALS_DATA_CH0_0);
	alsval_ch0_hi = ltr558_i2c_read_reg(LTR558_ALS_DATA_CH0_1);
	alsval_ch0 = (alsval_ch0_hi << 8) + alsval_ch0_lo;

	//printk("XXXXXXXXX alsval_ch0 = %d XXXXXXXX\n", alsval_ch0);
	
	alsval_ch1_lo = ltr558_i2c_read_reg(LTR558_ALS_DATA_CH1_0);
	alsval_ch1_hi = ltr558_i2c_read_reg(LTR558_ALS_DATA_CH1_1);
	alsval_ch1 = (alsval_ch1_hi << 8) + alsval_ch1_lo;

	//printk("XXXXXXXXX alsval_ch1 = %d XXXXXXXX\n", alsval_ch1);

	alsval_ch = alsval_ch0 + alsval_ch1;
	if (0 == alsval_ch)
		alsval_ch = 1;

	ratio = (alsval_ch1 * 100) / alsval_ch;

	if (ratio < 45) {
		ch0_coeff = 17743;
		ch1_coeff = -11059;
	} else if ((ratio >= 45) && (ratio < 64)) {
		ch0_coeff = 37725;
		ch1_coeff = 13363;
	} else if ((ratio >= 64) && (ratio < 85)) {
		ch0_coeff = 16900;
		ch1_coeff = 1690;
	} else if (ratio >= 85) {
		ch0_coeff = 0;
		ch1_coeff = 0;
	}

	luxdata_flt = (alsval_ch0 * ch0_coeff) - (alsval_ch1 * ch1_coeff);

#if 0
	// convert float to integer;
	if (luxdata_flt % 10000  > 5000) {
		luxdata_int = luxdata_flt / 10000 + 1;
	} else {
		luxdata_int = luxdata_flt / 10000;
	}
	luxdata_int = luxdata_flt / 1000;
	if ((luxdata_flt - luxdata_int) > 5) {
		luxdata_int = luxdata_int + 10;
	} else {
		luxdata_int = luxdata_flt;
	}

	luxdata_int = luxdata_int / 10;
#endif

	luxdata_int = luxdata_flt / 10;

	// For Range1
	if (gainrange == ALS_RANGE1_320)
		luxdata_int = luxdata_int / 200;

	//printk("als_lux_data = %d\n", luxdata_int);

	if (luxdata_int < 1900)
		luxdata_int *= 40;
	else if (luxdata_int < 7500)
		luxdata_int *= 26;
	else if (luxdata_int < 32000)
		luxdata_int *= 14;
	else if (luxdata_int < 50000)
		luxdata_int *= 7;
	else if (luxdata_int < 1000000)
		luxdata_int *= 4;
	else
		luxdata_int *= 2;

	final_lux_val = luxdata_int;

	return luxdata_int;
}

static int ltr558_als_read_status(void)
{
	int intval;
	intval = ltr558_i2c_read_reg(LTR558_ALS_PS_STATUS);
	if (intval < 0)
		goto out;
	intval = intval & 0x08;
out:
	return intval;
}

static int ltr558_als_open(struct inode *inode, struct file *file)
{
	if (als_opened) {
		printk(KERN_ALERT "%s: already opened\n", __func__);
		return -EBUSY;
	}
	als_opened = 1;
	return 0;
}

static int ltr558_als_release(struct inode *inode, struct file *file)
{
	printk(KERN_ALERT "%s\n", __func__);
	als_opened = 0;
	return ltr558_als_disable();
}

static int ltr558_als_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg)
{
	int ret = 0;
	int buffer = 1;
	void __user *argp = (void __user *)arg;

	switch(cmd) {
	case LTR558_IOCTL_ALS_ENABLE:
		if(copy_from_user(&buffer, argp, 1)) {
			ret = -EFAULT;
			break;
		}
		als_active = buffer;
		if(als_active) {
			ret = ltr558_als_enable(als_gainrange);
			if (ret < 0)
				break;
			else
				enable_irq_wake(MSM_GPIO_TO_INT(GPIO_INT_NO));
		}
		als_on = 1;

#if ALS_TIMER_FUNC
		ltr558_als_poll_timer_start(250);
#endif

		printk("XXXXXXX LTR558_IOCTL_ALS_ON XXXXXXXXXXXXXX\n");

		break;
	case LTR558_IOCTL_READ_ALS_DATA:
		buffer = ltr558_als_read(als_gainrange);
		if(buffer < 0) {
			ret = -EFAULT;
			break;
		}
		ret = copy_to_user(argp, &buffer, 1);
		break;
	case LTR558_IOCTL_READ_PS_INT:
		buffer = ltr558_als_read_status();
		if(buffer < 0) {
			ret = -EFAULT;
			break;
		}
		ret = copy_to_user(argp, &buffer, 1);
		break;
	case LTR558_IOCTL_ALS_DISABLE:
#if PROX_TIMER_FUNC	
		ltr558_als_timer_del();
#endif
		ret = ltr558_als_disable();
		als_on = 0;
		printk("XXXXXXXXX LTR558_LOCTL_PROX_OFF XXXXXXXXXX\n");

		break;
	}

	return ret;
}

static unsigned int ltr558_als_poll(struct file *fp, poll_table * wait)
{
	if(als_data_changed) {
		als_data_changed = 0;
		return POLLIN | POLLRDNORM;
	}
	poll_wait(fp, &als_waitqueue, wait);

	return 0;
}

static struct file_operations ltr558_als_fops = {
	.owner   = THIS_MODULE,
	.open    = ltr558_als_open,
	.release = ltr558_als_release,
	.ioctl   = ltr558_als_ioctl,
	.poll    = ltr558_als_poll,
};

static struct miscdevice ltr558_als_dev = {
		.minor = MISC_DYNAMIC_MINOR,
		.name  = DEVICE_NAME_ALS,
		.fops  = &ltr558_als_fops,
};

static void ltr558_schedwork(struct work_struct *work)
{
	int als_ps_status;
	int interrupt, newdata;

	als_ps_status = ltr558_i2c_read_reg(LTR558_ALS_PS_STATUS);
	interrupt = als_ps_status & 10;
	newdata = als_ps_status & 5;

	switch(interrupt){
	case 2:
		// PS interrupt
		if((newdata == 1) | (newdata == 5)) {
			final_prox_val = ltr558_ps_read();
			ps_data_changed = 1;
		}
		wake_up_interruptible(&ps_waitqueue);
		break;
	case 8:
		// ALS interrupt
		if((newdata == 4) | (newdata == 5)) {
			final_lux_val = ltr558_als_read(als_gainrange);
			als_data_changed = 1;
		}
		wake_up_interruptible(&als_waitqueue);
		break;
	case 10:
		// Both interrupt
		if((newdata == 1) | (newdata == 5)) {
			final_prox_val = ltr558_ps_read();
			ps_data_changed = 1;
		}
		wake_up_interruptible(&ps_waitqueue);
		if((newdata == 4) | (newdata == 5)) {
			final_lux_val = ltr558_als_read(als_gainrange);
			als_data_changed = 1;
		}
		wake_up_interruptible(&als_waitqueue);
		break;
	}

	if (wake_lock_active(&proximity_wake_lock)) {
		wake_unlock(&proximity_wake_lock);
	}
	enable_irq(MSM_GPIO_TO_INT(GPIO_INT_NO));
}

static irqreturn_t ltr558_irq_handler(int irq, void *dev_id)
{
	/* disable an irq without waiting */
	disable_irq_nosync(MSM_GPIO_TO_INT(GPIO_INT_NO));
	/* schedule_work - put work task in global workqueue
	 * * @work: job to be done
	 * *
	 * * Returns zero if @work was already on the kernel-global workqueue and
	 * * non-zero otherwise.
	 * *
	 * * This puts a job in the kernel-global workqueue if it was not already
	 * * queued and leaves it in the same position on the kernel-global
	 * * workqueue otherwise.
	 * */
	schedule_work(&irq_workqueue);
	return IRQ_HANDLED;
}

static int ltr558_gpio_irq(void)
{
	int ret;

#if 1
	ret = gpio_request(GPIO_INT_NO, DEVICE_NAME);
	if (ret) {
		printk(KERN_ALERT "%s: LTR-558ALS request gpio failed.\n", __func__);
		return ret;
	}
	ret = gpio_direction_output(GPIO_INT_NO, 1);
	if (ret) {
		printk(KERN_ALERT "%s: LTR-558ALS gpio direction output failed.\n", __func__);
		return ret;
	}
#endif

	ltr558_irq = MSM_GPIO_TO_INT(GPIO_INT_NO);
	ret = request_irq(ltr558_irq, ltr558_irq_handler, IRQF_TRIGGER_LOW, DEVICE_NAME, NULL);
	if (ret) {
		printk(KERN_ALERT "%s: LTR-558ALS request irq failed.\n", __func__);
		return ret;
	}

	return 0;
}

static int ltr558_devinit(void)
{
	int error;
	int init_ps_gain;
	int init_als_gain;

	mdelay(PON_DELAY);
	// Enable PS to Gain1 at startup
	init_ps_gain = PS_RANGE1;
	ps_gainrange = init_ps_gain;
	error = ltr558_ps_enable(init_ps_gain);
	if(error < 0)
		goto out;
	// Enable ALS to Full Range at startup
	init_als_gain = ALS_RANGE2_64K;
	als_gainrange = init_als_gain;
	error = ltr558_als_enable(init_als_gain);
	if(error < 0)
		goto out;

	error = 0;
out:
	return error;
}

static void ltr558_prox_poll(struct ltr558_prox_info *proxp)
{
	static int event = 0;

	proxp->prox_data = ltr558_ps_read();
	//printk("ps_data = %d\n", proxp->prox_data);

	prox_history_hi <<= 1;
	prox_history_hi |= ((proxp->prox_data > ltr558_cfgp->prox_threshold_hi) ? 1 : 0);
	prox_history_hi &= 0x07;
	prox_history_lo <<= 1;
	prox_history_lo |= ((proxp->prox_data > ltr558_cfgp->prox_threshold_lo) ? 1 : 0);
	prox_history_lo &= 0x01;
	if (prox_history_hi == 0x07)
		event = 0;

	else {
		if (prox_history_lo == 0)
			event = 1;
	}

	proxp->prox_event = event;
}

static int ltr558_als_data(void)
{
	int als_lux_data;

	als_lux_data = ltr558_als_read(als_gainrange);
	//printk("als_lux_data = %d\n", als_lux_data);

	return als_lux_data;
}

static void ltr558_prox_data(void)
{
	ltr558_prox_poll(prox_cur_infop);
}

static void ltr558_prox_calibrate(void)
{
	int prox_sum = 0, prox_mean = 0, prox_max = 0;
	int i = 0;

	printk("XXXXXXXXXXXX LTR558_PROX_CALIBRATE: XXXXXXXXX\n");

	for (i = 0; i < 20; i++) {
		ltr558_prox_poll(&prox_cal_info[i]);
		prox_sum += prox_cal_info[i].prox_data;
		if (prox_cal_info[i].prox_data > prox_max)
			prox_max = prox_cal_info[i].prox_data;

		msleep(60);
	}
	prox_mean = prox_sum / 20;
	ltr558_cfgp->prox_threshold_hi = ((((prox_max - prox_mean) * 200) + 50) / 100) + prox_mean;
	ltr558_cfgp->prox_threshold_lo = ((((prox_max - prox_mean) * 170) + 50) / 100) + prox_mean;

	/* get smaller value */
	if (ltr558_cfgp->prox_threshold_lo < ((sat_prox * 2) / 100)) {
		ltr558_cfgp->prox_threshold_lo = ((sat_prox * 3) / 100);
		ltr558_cfgp->prox_threshold_hi = ((sat_prox * 5) / 100);
	}

	/* panel down */
	if (ltr558_cfgp->prox_threshold_hi > ((sat_prox * 50) / 100)) {
		ltr558_cfgp->prox_threshold_lo = sat_prox * 25 / 100;
		ltr558_cfgp->prox_threshold_hi = sat_prox * 30 / 100;
	}

	printk("XXXXXXXXXXX prox_threshold_lo = %d\nXXXXXXXXXXX prox_threshold_hi = %d\n", ltr558_cfgp->prox_threshold_lo, ltr558_cfgp->prox_threshold_hi);

	/* prox off */
	ltr558_ps_disable();

	prox_history_hi = 0;
	prox_history_lo = 0;
}

static int ltr558_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int ret = 0;
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);

	printk("XXXXXXXXXXXXXXXX ltr558_probe XXXXXXXXXXXXXX\n");

	/* Return 1 if adapter supports everything we need, 0 if not. */
	if(!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_WRITE_BYTE | I2C_FUNC_SMBUS_READ_BYTE_DATA)) {
		printk(KERN_ALERT "%s: LTR-558ALS functionality check failed.\n", __func__);
		ret = -EIO;
		goto out;
	}
	/* data memory allocation */
	the_data = kzalloc(sizeof(struct ltr558_data), GFP_KERNEL);
	if(the_data == NULL) {
		printk(KERN_ALERT "%s: LTR-558ALS kzalloc failed.\n", __func__);
		ret = -ENOMEM;
		goto out;
	}
	the_data->client = client;
	i2c_set_clientdata(client, the_data);

	ltr558_cfgp = kmalloc(sizeof(struct ltr558_cfg), GFP_KERNEL);
	if (ltr558_cfgp == NULL) {
		printk(KERN_ALERT "LTR558: kmalloc for struct ltr558_cfg failed in ltr558_probe()\n");
		return -ENOMEM;
		goto out;
	}

	ltr558_cfgp->prox_led      = prox_led_param;
	ltr558_cfgp->prox_pulse    = prox_pusle_param;
	ltr558_cfgp->als_meas_rate = als_meas_rate_param;
	sat_prox = 1 << 10;

	ret = ltr558_i2c_read_reg(LTR558_MANUFACTURER_ID);
	if (ret != 0x05) {
		printk(KERN_ALERT "%s: LTR-558ALS read MANUFACTURER_ID failed.\n ", __func__);
		goto out;
	}

	ret = ltr558_devinit();
	if(ret) {
		printk(KERN_ALERT "%s: LTR-558ALS device init failed.\n", __func__);
		goto out;
	}

	ret = misc_register(&ltr558_ps_dev);
	if (ret) {
		printk(KERN_ALERT "%s: LTR-558ALS misc_register ps failed.\n", __func__);
		goto out;
	}
	
	ret = misc_register(&ltr558_als_dev);
	if (ret) {
		printk(KERN_ALERT "%s: LTR-558ALS misc_register als failed.\n", __func__);
		goto outps;
	}

#if 0
	wake_lock_init(&proximity_wake_lock, WAKE_LOCK_SUSPEND, "proximity");
	INIT_WORK(&irq_workqueue, ltr558_schedwork);
	ret = ltr558_gpio_irq();
	if(ret) {
		printk(KERN_ALERT "%s: LTR-558ALS gpio_irq failed.\n", __func__);
		goto outals;
	}
#endif

#if PROX_TIMER_FUNC
	//int prox timer
	init_timer(&prox_poll_timer);
	prox_poll_timer.function = ltr558_prox_poll_timer_func;
	INIT_WORK(&the_data->prox_workqueue, ltr558_prox_work_f);
#endif
#if ALS_TIMER_FUNC
	//init als timer
	init_timer(&als_poll_timer);
	als_poll_timer.function = ltr558_als_poll_timer_func;
	INIT_WORK(&the_data->als_workqueue, ltr558_als_work_f);
#endif

	/* prox calibrate */
	ltr558_prox_calibrate();

#ifdef CONFIG_HAS_EARLYSUSPEND
	the_data->ltr558_early_suspend.suspend = ltr558_als_early_suspend;
	the_data->ltr558_early_suspend.resume = ltr558_als_late_resume;
	register_early_suspend(&the_data->ltr558_early_suspend);
#endif

	printk("XXXXXXXXXXXXXXXX ltr558_probe OK XXXXXXXXXXXXXX\n");

	ret = 0;
	goto out;
outals:
	misc_deregister(&ltr558_als_dev);
outps:
	misc_deregister(&ltr558_ps_dev);
out:
	return ret;
}

static int ltr558_remove(struct i2c_client *client)
{
	kfree(i2c_get_clientdata(client));
	//free_irq(MSM_GPIO_TO_INT(GPIO_INT_NO), NULL);
	//gpio_free(GPIO_INT_NO);
	//wake_lock_destroy(&proximity_wake_lock);
	misc_deregister(&ltr558_ps_dev);
	misc_deregister(&ltr558_als_dev);
	ltr558_ps_disable();
	ltr558_als_disable();

	return 0;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
//als suspend
static void ltr558_als_early_suspend(struct early_suspend *h)
{
	int ret;

	if (als_on) {
#if ALS_TIMER_FUNC
		ltr558_als_timer_del();
#endif
		ret = ltr558_als_disable();
		if (ret < 0)
			printk("XXXXXXXXXXXXXXXXX LTR558_ALS_EARLY_SUSPEND ERROR!!! XXXXXXXXXXXXX\n");
	}

	printk("XXXXXXXXXXX ltr558_als_early_suspend XXXXXXXXXXXX\n");
}

//als_resume
static void ltr558_als_late_resume(struct early_suspend *h)
{
	int ret;

	if (als_on) {
		ret = ltr558_als_enable(als_gainrange);
		if (ret < 0) {
			printk("XXXXXXXXXXXXXXXXX LTR558_ALS_LARE_RESUME_ ERROR!!! XXXXXXXXXXXXX\n");
			return;
		}
#if ALS_TIMER_FUNC
		ltr558_als_poll_timer_start(400);
#endif
	}

	printk("XXXXXXXXXXx ltr558_als_late_resume XXXXXXXXXXXX\n");
}
#endif

static int ltr558_prox_suspend(struct device *dev)
{
	int ret = 0;

	if (prox_on) {
#if PROX_TIMER_FUNC
			ltr558_prox_timer_del();
#endif

		ret = ltr558_ps_disable();
		if (ret < 0) {
			printk("XXXXXXXXXXXXXXXXX LTR558_PROX_SUSPEND ERROR!!! XXXXXXXXXXXXX\n");
			return ret;
		}
	}

	printk("XXXXXXXXXXx ltr558_prox_suspend XXXXXXXXXXXX\n");

	return ret;
}

static int ltr558_prox_resume(struct device *dev)
{
	int ret = 0;

	if (prox_on) {
		ret = ltr558_ps_enable(ps_gainrange);
		if (ret < 0) {
			printk("XXXXXXXXXXXXXXXXX LTR558_PROX_RESUME_ ERROR!!! XXXXXXXXXXXXX\n");
			return ret;
		}

#if PROX_TIMER_FUNC
		ltr558_prox_poll_timer_start(400);
#endif
	}

	printk("XXXXXXXXXXX ltr558_prox_resume XXXXXXXXXXXX\n");

	return ret;
}

#if PROX_TIMER_FUNC
static void ltr558_prox_work_f(struct work_struct *work)
{
	if (prox_on) {
		ltr558_prox_data();

		mutex_lock(&__msm_sensors_lock);
		msm_sensors_input_report_value(EV_MSC, MSC_GESTURE, prox_cur_infop->prox_event);
		msm_sensors_input_sync();
		mutex_unlock(&__msm_sensors_lock);

		//printk("XXXXXXXXXXXXX prox->event == %d XXXXXXXXX\n", prox_cur_infop->prox_event);  //for test
	} else
		printk("XXXXXXXXXX ltr558_prox_work_f status error !! XXXXXXXXXX\n");
}

// prox poll timer function
static void ltr558_prox_poll_timer_func(unsigned long param)
{
	schedule_work(&the_data->prox_workqueue);
	ltr558_prox_poll_timer_start(400);
}

// start prox poll timer
static void ltr558_prox_poll_timer_start(unsigned int t)
{
	//if (1 == prox_timer_count)
		mod_timer(&prox_poll_timer, jiffies + t * HZ / 1000);
}

static void ltr558_prox_timer_del(void)
{
	//if (1 == prox_timer_count)
		del_timer(&prox_poll_timer);
}
#endif

#if ALS_TIMER_FUNC
//als_workqueue_function
static void ltr558_als_work_f(struct work_struct *work)
{
	int lux_value;

	if (als_on) {
		lux_value = ltr558_als_data();
		if (lux_value < 0) {
			printk("XXXXXXXXXXX taos_als_poll_timer_func error XXXXXXXXXXXx\n");
			return;
		}

		mutex_lock(&__msm_sensors_lock);
		msm_sensors_input_report_value(EV_MSC, MSC_RAW, lux_value);
		msm_sensors_input_sync();
		mutex_unlock(&__msm_sensors_lock);

		//printk("XXXXXXXXXXXXX lux_value== %d XXXXXXXXX\n", lux_value);  //for test
	} else
		printk("XXXXXXXXXX ltr558_als_work_f status error !! XXXXXXXXXX\n");

}

//als poll timer func
static void ltr558_als_poll_timer_func(unsigned long param)
{
	schedule_work(&the_data->als_workqueue);
	ltr558_als_poll_timer_start(250);
}

static void ltr558_als_poll_timer_start(unsigned int t)
{
	//if (1 == als_timer_count)
		mod_timer(&als_poll_timer, jiffies + t * HZ / 1000);
}

static void ltr558_als_timer_del(void)
{
	//if (1 == als_timer_count)
		del_timer(&als_poll_timer);
}
#endif

static const struct i2c_device_id ltr558_id[] = {
	{ DEVICE_NAME, 0 },
	{}
};

static struct dev_pm_ops ltr558_pm_ops = {
	.suspend = ltr558_prox_suspend,
	.resume  = ltr558_prox_resume,
};

static struct i2c_driver ltr558_driver = {
	.probe = ltr558_probe,
	.remove = ltr558_remove,
	.id_table = ltr558_id,
	.driver = {
		.owner = THIS_MODULE,
		.name  = DEVICE_NAME,
		.pm    = &ltr558_pm_ops,
	},
};

static int __init ltr558_driverinit(void)
{
	printk(KERN_ALERT "<<< %s: LTR-558ALS Driver Module LOADED >>>\n", __func__);
	return i2c_add_driver(&ltr558_driver);
}

static void __exit ltr558_driverexit(void)
{
	i2c_del_driver(&ltr558_driver);
	printk(KERN_ALERT ">>> %s: LTR-558ALS Driver Module REMOVED <<<\n", __func__);
	printk(KERN_INFO "%s\n", "0012345678900");
}

module_init(ltr558_driverinit);
module_exit(ltr558_driverexit);
MODULE_AUTHOR("Lite-On Technology Corp");
MODULE_DESCRIPTION("LTR-558ALS Driver");
MODULE_LICENSE("Dual BSD/GPL");
MODULE_VERSION(DRIVER_VERSION);

