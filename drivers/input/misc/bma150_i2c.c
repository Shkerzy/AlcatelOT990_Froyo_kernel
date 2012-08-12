/* 	drivers/i2c/chips/bma150.c - bma150 gsensor driver
 *
 * Copyright (C) 2009 JRD Corporation.
 * Author: erjun.ding <dingerjun@gmail.com>
 *	Protocol driver for Bosch BMA150 accelerometer
 *	Porting bma150 driver from spi to i2c protocol
 *
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

/*===========================================================================

                        EDIT HISTORY FOR MODULE
10/03/03   qian.liu		  Add motion sensor calibration interface
10/01/26   erjun.ding     Cut down power  consumption  when sleep                    
10/01/25   erjun.ding     Fix sleep exception                      
09/12/23   erjun.ding     Move bma150_accel_data structure to bma150_i2c.h
09/12/23   erjun.ding     Close PROC_FS_DEBUG macro and remove useless comment
09/12/23   erjun.ding     Add akmd compass sensor native interface .
09/12/03   erjun.ding     Clear debug log
09/11/17   erjun.ding     Initial Release

===========================================================================*/

 
#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/ctype.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/irq.h>
#include <linux/platform_device.h>
#include <linux/spinlock.h>
#include <linux/interrupt.h>
#include <linux/fs.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/uaccess.h>
#include <linux/debugfs.h>
#include <linux/bma150_i2c.h>
#include <linux/miscdevice.h>
#include <linux/time.h>
#include <mach/gpio.h>
#include <linux/delay.h>

#define BMA150_CMD_READ                  0x80
#define BMA150_REG_CHIPID                0x00
#define BMA150_REG_ACCX_LS               0x02
#define BMA150_REG_CONTROL_0A            0x0A
#define BMA150_REG_CONTROL_0B            0x0B
#define BMA150_REG_ANY_MOTION_THRESH     0x10
#define BMA150_REG_WIDTH_BANDW           0x14
#define BMA150_REG_CONTROL_15            0x15
#define BMA150_LAST_REG                  0x15

#define BMA150_REG_C0A_RESET_INT         0x40
#define BMA150_REG_C0A_SLEEP             0x01

#define BMA150_REG_C0B_ANY_MOTION        0x40
#define BMA150_REG_C0B_ENABLE_HG         0x02
#define BMA150_REG_C0B_ENABLE_LG         0x01

#define BMA150_REG_RANGE_MASK 			 0x18
#define BMA150_REG_WID_BANDW_MASK        0x07

#define BMA150_REG_C15_SPI4              0x80
#define BMA150_REG_C15_EN_ADV_INT        0x40
#define BMA150_REG_C15_NEW_DATA_INT      0x20
#define BMA150_REG_C15_LATCH_INT         0x10

#define BMA150_BANDW_INIT                0x04
#define BMA150_ANY_MOTION_INIT           0x02

#define BMA150_REG_XOFFSET		 0x1A
#define BMA150_REG_YOFFSET		 0x17
#define BMA150_REG_ZOFFSET		 0x18


#define BMA150_XOFFSET_DATA		 0xff	

#define OFFS_GAIN_X_REG		0x16
#define OFFS_GAIN_Y_REG		0x17
#define OFFS_GAIN_Z_REG		0x18
#define OFFS_GAIN_T_REG		0x19
#define OFFSET_X_REG		0x1a
#define OFFSET_Y_REG		0x1b
#define OFFSET_Z_REG		0x1c
#define OFFSET_T_REG		0x1d


/* temperature offset of -30 degrees in units of 0.5 degrees */
#define BMA150_TEMP_OFFSET               60

#define BMA150_NAME                      "bma150_i2c"
#define BMA150_VENDORID                  0x0001

MODULE_LICENSE("GPL v2");
MODULE_ALIAS("bma150_i2c");


//#define CONFIG_BMA150_I2C_DEBUG  /*switch of debug*/
#define TRUE_DEVICE		   /*switch of debug about whether use i2c real device*/
//#define BMA150_IRQ	           /*switch of whether use sensor irq*/
//#define PROC_FS_DEBUG	           /*switch of whether use proc register setting interface*/
#define ECOMPASS_DAEMON		   /*switch of whether support ECOMPASS daemon*/


#ifdef CONFIG_BMA150_I2C_DEBUG
#define BDBG(fmt, args...) printk(KERN_INFO "bma150_i2c: " fmt, ##args)
#else
#define BDBG(fmt, args...) do { } while (0)
#endif



#ifdef PROC_FS_DEBUG
#include <linux/proc_fs.h>
	static int reg;
	static struct proc_dir_entry *bmaproc;
#endif



struct bma150_i2c_driver_data {
	struct input_dev         *ip_dev;
	struct i2c_client		*i2c;

	char                      rx_buf[16];
	char                      bits_per_transfer;
	struct work_struct        work_data;
	bool                      config;
	struct list_head          next_dd;
//	struct dentry            *dbfs_root;
//	struct dentry            *dbfs_bpw;
//	struct dentry            *dbfs_regs;
	
};

static struct bma150_i2c_driver_data *bma150_driver_data;

//typedef struct bma150_i2c_driver_data bma150_i2c_driver_data;

static struct mutex               bma150_dd_lock;
static struct list_head           dd_list;

//ejding++
	unsigned long		bma_delay=1000; //default value 200ms
	atomic_t 		ctl_open_count =ATOMIC_INIT(0);
	struct timer_list 	refresh_timer;
//ejding<<<

#ifdef ECOMPASS_DAEMON
	atomic_t 		ecomd_open_count =ATOMIC_INIT(0);
#endif


#ifdef  BMA150_IRQ
#define GPIO_BMA150_I2C_INT 27
static struct msm_gpio bma_i2c_gpio_config_data[] = {
	{ GPIO_CFG(GPIO_BMA150_I2C_INT, 0, GPIO_INPUT,  GPIO_NO_PULL, GPIO_2MA), "bma_irq" },
};

static int msm_bma_i2c_gpio_setup(void )
{
	int rc;
	rc = msm_gpios_request_enable(bma_i2c_gpio_config_data,
		ARRAY_SIZE(bma_i2c_gpio_config_data));
	return rc;
}

static void msm_bma_i2c_gpio_teardown(void )
{
	msm_gpios_disable_free(bma_i2c_gpio_config_data,
		ARRAY_SIZE(bma_i2c_gpio_config_data));
}

#endif


static void update_accel_data(unsigned long );
static void refresh_data(unsigned long data ){
//		update_accel_data(data);	
	struct bma150_i2c_driver_data *dd=(struct bma150_i2c_driver_data *)data;

		mod_timer(&refresh_timer, jiffies + bma_delay*HZ/1000);
		schedule_work(&dd->work_data);
//		BDBG("refresh_data is called! \n");

}

static int32_t bma150_i2c_write(struct bma150_i2c_driver_data *dd,
	unsigned char *txdata, int length)
{
	if(!dd->i2c )
		return -EIO;

	struct i2c_msg msg[] = {
		{
			.addr = dd->i2c->addr,
			.flags = 0,
			.len = length,
			.buf = txdata,
		},
	};

#ifdef TRUE_DEVICE
	if (i2c_transfer(dd->i2c->adapter, msg, 1) < 0) {
		BDBG("bma150_i2c_write faild\n");
		return -EIO;
	}
#endif
	return 0;
}

/* read bma150 via i2c address + register offset, return # bytes read */
static int bma150_i2c_read(struct bma150_i2c_driver_data *dd, uint8_t regaddr,
		     uint8_t *buf, uint32_t rdlen)
{
	uint8_t ldat = regaddr;

	if(!dd->i2c )
		return -EIO;
	
	struct i2c_msg msgs[] = {
		[0] = {
			.addr	= dd->i2c->addr,
			.flags	= 0,
			.buf	= (void *)&ldat,
			.len	= 1
		},
		[1] = {
			.addr	= dd->i2c->addr,
			.flags	= I2C_M_RD,
			.buf	= (void *)buf,
			.len	= rdlen
		}
	};

#ifdef TRUE_DEVICE
	if (i2c_transfer(dd->i2c->adapter, msgs, 2) < 0) {
		BDBG("bma150_i2c_read failed!\n");
		return -EIO;
	}
#else
	memset(buf, 0xff, rdlen);
#endif
	return 0;

}



#ifdef CONFIG_PM
static int bma150_i2c_suspend(struct i2c_client *i2c, pm_message_t mesg)
{

//	disable_irq(i2c->irq);
	if( refresh_timer.function)
		del_timer_sync(&refresh_timer);

	return 0;
}

static int bma150_i2c_resume(struct i2c_client *i2c)
{
//	enable_irq(i2c->irq);
	if(refresh_timer.function)
		mod_timer(&refresh_timer, jiffies + bma_delay*HZ/1000);
	BDBG("bma150_data port is closed!\n");
	return 0;
}
#else
#define bma150_i2c_suspend NULL
#define bma150_i2c_resume NULL
#endif /* CONFIG_PM */

#define bma150_calibrate_onedir_cmd 0x01
int bma150_get_offset(unsigned char xyz, unsigned short *offset)
{
	u8 				buf[2];
	int 			rc=0;

	rc = bma150_i2c_read(bma150_driver_data, 0x16+xyz, buf, 1);
	rc += bma150_i2c_read(bma150_driver_data, 0x1a+xyz, &buf[1], 1);
	
	*offset = buf[1] << 2 | buf[0] >> 6;
	return rc;
}

int bma150_set_offset(unsigned char xyz, unsigned short offset)
{
	u8 				buf[4];
	int 			rc=0;

	buf[0] = 0x16+xyz;
	rc = bma150_i2c_read(bma150_driver_data, 0x16+xyz, &buf[1], 1);
	buf[1] = buf[1] & 0x3f | (offset & 0x03) << 6;
	buf[2] = 0x1a+xyz;
	buf[3] = offset >>2;
	rc += bma150_i2c_write(bma150_driver_data, buf, 4);
	return rc;	
}

/** write offset data to SMB380 image
   \param eew 0 = lock EEPROM 1 = unlock EEPROM 
   \return result of bus communication function
*/
int bma150_set_ee_w(unsigned char eew) 
{
	unsigned char data;
    int rc;
	u8				buf[8];
	
	rc = bma150_i2c_read(bma150_driver_data, 0xa0, buf, 1);
	buf[1] = (buf[0] & ~0x10 ) | (eew ? 0x10 : 0x00);
	buf[0] = 0xa0;
	rc += bma150_i2c_write(bma150_driver_data, buf, 2);
	
	return rc;
}


int bma150_read_accel_xyz(bma150acc_t * acc)
{
	    int rc;
	    u8 		buf[8];

		rc = bma150_i2c_read(bma150_driver_data, BMA150_REG_ACCX_LS,buf,8);
		
		acc->x = ((buf[0] & 0xC0) >> 6) | (buf[1] << 2);
		acc->y = ((buf[2] & 0xC0) >> 6) | (buf[3] << 2);
		acc->z = ((buf[4] & 0xC0) >> 6) | (buf[5] << 2);
		/* convert 10-bit signed value to 32-bit */
		if (acc->x & 0x200)
				acc->x = acc->x - 0x400;
		if (acc->y & 0x200)
				acc->y = acc->y - 0x400;
		if (acc->z & 0x200)
				acc->z = acc->z - 0x400;
		
		return rc;
}

/** reads out acceleration data and averages them, measures min and max
 *  param orientation pass orientation one axis needs to be absolute 1 the others need to be 0
 *  param num_avg numer of samples for averaging
 *  param *min returns the minimum measured value
 *  param *max returns the maximum measured value
 *  param *average returns the average value
 *  */
int bma150_read_accel_avg(int num_avg, bma150acc_t *min, bma150acc_t *max, bma150acc_t *avg )
{
	    long x_avg=0, y_avg=0, z_avg=0;
		int comres=0;
		int i;
		bma150acc_t accel;                      /* read accel data */

		x_avg = 0; y_avg=0; z_avg=0;
		max->x = -512; max->y =-512; max->z = -512;
		min->x = 512;  min->y = 512; min->z = 512;
	
		for (i=0; i<num_avg; i++) {
	        comres += bma150_read_accel_xyz(&accel);      /* read 10 acceleration data triples */
	        if (accel.x > max->x)
		        max->x = accel.x;
		    if (accel.x < min->x)
		        min->x = accel.x;
	        
			if (accel.y > max->y)
    	        max->y = accel.y;
	        if (accel.y < min->y)
        	    min->y = accel.y;

        	if (accel.z > max->z)
	            max->z = accel.z;
        	if (accel.z < min->z)
	            min->z = accel.z;

        	x_avg += accel.x;
        	y_avg += accel.y;
        	z_avg += accel.z;

        	udelay(10);
    }
    avg->x = (x_avg / num_avg);                             /* calculate averages, min and max values */
    avg->y = (y_avg / num_avg);
    avg->z = (z_avg / num_avg);
    return comres;
}
/** verifies the accerleration values to be good enough for calibration calculations
 *  \param min takes the minimum measured value
 *  \param max takes the maximum measured value
 *  \param takes returns the average value
 *  \return 1: min,max values are in range, 0: not in range
 *  */
int bma150_verify_min_max(bma150acc_t min, bma150acc_t max, bma150acc_t avg)
{
    short dx, dy, dz;
    int ver_ok=1;

    dx =  ((max.x) - (min.x));    /* calc delta max-min */
    dy =  ((max.y) - (min.y));
    dz =  ((max.z) - (min.z));

	if ((dx> 10) || (dx<-10))
	    ver_ok = 0;
	if ((dy> 10) || (dy<-10))
	    ver_ok = 0;
	if ((dz> 10) || (dz<-10))
	    ver_ok = 0;

	return ver_ok;
}

/** calculates new offset in respect to acceleration data and old offset register values
  \param orientation pass orientation one axis needs to be absolute 1 the others need to be 0
  \param accel holds the measured acceleration value of one axis
  \param *offset takes the old offset value and modifies it to the new calculated one
  \param stepsize holds the value of the calculated stepsize for one axis
  \param *min_accel_adapt stores the minimal accel_adapt to see if calculations make offset worse
  \param *min_offset stores old offset bevor actual calibration to make resetting possible in next function call
  \param *min_reached stores value to prevent further calculations if minimum for an axis reached 
  \return	0x00: Axis was already calibrated
			0x01: Axis needs calibration
			0x11: Axis needs calibration and the calculated offset was outside boundaries
 */
int bma150_calc_new_offset(short orientation, short accel, short *offset, short *stepsize, short *min_accel_adapt, short *min_offset, short *min_reached, short *prev_steps, int iteration_counter)
{
	short old_offset;
	short new_offset;
	short accel_adapt;
	short steps;

   	unsigned char  calibrated = 0;

   	old_offset = *offset;
   
   	accel_adapt = accel - (orientation * 256);
	if(accel_adapt < -512)
	{
	 	accel_adapt = -512;
	}
	else if(accel_adapt > 511)
	{
	 	accel_adapt = 511;
	}
		                                
   	if (((accel_adapt > 7) || (accel_adapt < -7)) && !(*min_reached))	/* does the axis need calibration? minimum for this axis not yet reached? */
   	{
		if (abs(accel_adapt) <= abs(*min_accel_adapt))				/* accel_adapt smaller than minimal accel_adapt ?
															   			means: previous offset calculation lead to better result */
		{
			if(((3*(*prev_steps) * (*stepsize)) <= 2*(abs((*min_accel_adapt) - (accel_adapt)))) && (iteration_counter >= 1))/* if calculated stepsize is too small compared to real stepsize*/
			{
				(*stepsize) = (abs((*min_accel_adapt) - (accel_adapt))) / (*prev_steps);	/* adapt stepsize */
			}
						
			if ((accel_adapt < (*stepsize)) && (accel_adapt > 0))	/* check for values less than quantisation of offset register */
				new_offset = old_offset -1;          
		 	else if ((accel_adapt > -(*stepsize)) && (accel_adapt < 0))    
		   		new_offset = old_offset +1;
	     	else
			{
				steps = (accel_adapt/(*stepsize));					/* calculate offset LSB */
				if((2*(abs(steps * (*stepsize) - accel_adapt))) > (*stepsize))	/* for better performance (example: accel_adapt = -25, stepsize = 13 --> two steps better than one) */
				{
					if(accel_adapt < 0)
						steps-=1;
					else
						steps+=1;	
				} 
	       		new_offset = old_offset - steps;					/* calculate new offset */
			}
	     	
			if (new_offset < 0)										/* check for register boundary */
			{
				new_offset = 0;										/* <0 ? */
				calibrated = 0x10;
			}
	     	else if (new_offset > 1023)
		 	{
				new_offset = 1023;									/* >1023 ? */
				calibrated = 0x10;
		 	}

			*prev_steps = abs(steps);								/* store number of steps */
			if(*prev_steps==0)										/* at least 1 step is done */
				*prev_steps = 1;
						
			*min_accel_adapt = accel_adapt; 						/* set current accel_adapt value as minimum */
			*min_offset = old_offset;								/* store old offset (value before calculations above) */

			*offset = new_offset;									/* set offset as calculated */
		}
		else
		{
			*offset = *min_offset;									/* restore old offset */
			if((2*(*prev_steps) * (*stepsize)) <= (abs((*min_accel_adapt) - (accel_adapt))))/* if calculated stepsize is too small compared to real stepsize*/
			{
				(*stepsize) = (abs((*min_accel_adapt) - (accel_adapt))) / (*prev_steps);	/* adapt stepsize */
			}
			else
			{
				*min_reached = 0x01;								/* prevent further calibrations */
			}	
		}
					    
	 	calibrated |= 0x01;
   	}
  	return calibrated;
}


/** overall calibration process. This function takes care about all other functions 
 *  \param orientation input for orientation [0;0;1] for measuring the device in horizontal surface up
 *  \param *offset output structure for offset values (x, y and z)
 *  \param tries takes the number of wanted iteration steps, this pointer returns the number of calculated steps after this routine has finished
 *  \return 0: calibration passed; {0x1,0x2,0x4}: did not pass within N steps; 0x80 : Too much movement 
 *  */
#define CALIB_X_AXIS            1
#define CALIB_Y_AXIS            2
#define CALIB_Z_AXIS            4

int bma150_calibration(bma150acc_t orientation, bma150acc_t *offset, int tries)
{

	short offset_x, offset_y, offset_z;
	int need_calibration=0, min_max_ok=0;
	int error_flag=0;
	int iteration_counter =0; 
	int dummy;

	//****************************************************************************  
    unsigned char dummy2;
    short min_accel_adapt_x = 0xFE00, min_accel_adapt_y = 0xFE00, min_accel_adapt_z = 0xFE00;
    short min_offset_x = 0, min_offset_y = 0, min_offset_z = 0;  
    short min_reached_x = 0, min_reached_y  = 0, min_reached_z  = 0;
    short prev_steps_x = 0x01FF, prev_steps_y = 0x01FF, prev_steps_z = 0x01FF;
    //****************************************************************************
	u8 						buf[8];
	u8 						regval;

	//****************************************************************************
    bma150acc_t min,max,avg,gain,step;
	bma150_i2c_read(bma150_driver_data, 0x14, buf, 1);
	regval = buf[0];
	regval &= ~(BMA150_REG_RANGE_MASK | BMA150_REG_WID_BANDW_MASK);
	buf[0] = 0x14;
	buf[1] = regval;
	bma150_i2c_write(bma150_driver_data, buf, 2);	

    //smb380_set_range(SMB380_RANGE_2G);
    //smb380_set_bandwidth(SMB380_BW_25HZ);
	
	bma150_set_ee_w(1);	
    //smb380_set_ee_w(1);
	
    //smb380_get_offset(0, &offset_x);
    //smb380_get_offset(1, &offset_y);
    //smb380_get_offset(2, &offset_z);
	bma150_get_offset(0, &offset_x);
	bma150_get_offset(1, &offset_y);
	bma150_get_offset(2, &offset_z);
//****************************************************************************
    bma150_i2c_read(bma150_driver_data, 0x16, &dummy2, 1);
	//smb380_read_reg(0x16, &dummy2, 1);          /* read gain registers from image */
    gain.x = dummy2 & 0x3F;
    bma150_i2c_read(bma150_driver_data, 0x17, &dummy2, 1);
	//smb380_read_reg(0x17, &dummy2, 1);
    gain.y = dummy2 & 0x3F;
	bma150_i2c_read(bma150_driver_data, 0x18, &dummy2, 1);
    //smb380_read_reg(0x18, &dummy2, 1);
    gain.z = dummy2 & 0x3F;
	bma150_set_ee_w(0);
    //smb380_set_ee_w(0);

    step.x = gain.x * 15/64 + 7;                /* calculate stepsizes for all 3 axes */
    step.y = gain.y * 15/64 + 7;
    step.z = gain.z * 15/64 + 7;

	udelay(50);
    //smb380_pause(50);                           /* needed to prevent CALIB_ERR_MOV */
    do {
	        bma150_read_accel_avg(10, &min, &max, &avg);        /* read acceleration data min, max, avg */
	        min_max_ok = bma150_verify_min_max(min, max, avg);
			BDBG( "qian.liu bma150_verify_min_max: min_max_ok  %d\n", min_max_ok);

	        if (!min_max_ok)                                    /* check if calibration is possible */
	            return 0x80;//return CALIB_ERR_MOV;

			need_calibration = 0;
	        /* x-axis */
	        dummy = bma150_calc_new_offset(orientation.x, avg.x, &offset_x, &step.x, &min_accel_adapt_x, &min_offset_x, &min_reached_x, &prev_steps_x, iteration_counter);
			
			bma150_set_ee_w(1);
	        //smb380_set_ee_w(1);
	        if (dummy & 0x01)                                   /* x-axis calibrated ? */
	        {
	            bma150_set_offset(0, offset_x);
	            need_calibration = CALIB_X_AXIS;
	        }
	        if (dummy & 0x10)                                   /* x-axis offset register boundary reached */
	            error_flag |= (CALIB_X_AXIS << 4);
	        /* y-axis */
	        dummy = bma150_calc_new_offset(orientation.y, avg.y, &offset_y, &step.y, &min_accel_adapt_y, &min_offset_y, &min_reached_y, &prev_steps_y, iteration_counter);

	        if (dummy & 0x01)                                   /* y-axis calibrated ? */
	        {
	            bma150_set_offset(1, offset_y);
	            need_calibration |= CALIB_Y_AXIS;
	        }
	        if (dummy & 0x10)                                   /* y-axis offset register boundary reached ? */
	            error_flag |= (CALIB_Y_AXIS << 4);
            /* z-axis */
	        dummy = bma150_calc_new_offset(orientation.z, avg.z, &offset_z, &step.z, &min_accel_adapt_z, &min_offset_z, &min_reached_z, &prev_steps_z, iteration_counter);

	        if (dummy & 0x01)                                   /* z-axis calibrated ? */
	        {
				bma150_set_offset(2, offset_z);
		        need_calibration |= CALIB_Z_AXIS;
            }
	        if (dummy & 0x10)                                   /* z-axis offset register boundary reached */
		        error_flag |= (CALIB_Z_AXIS << 4);
			
			bma150_set_ee_w(0);
	        //smb380_set_ee_w(0);
	        iteration_counter++;
	        if (need_calibration)   /* if one of the offset got changed wait for the filter to fill with the new acceleration */
	        {
				udelay(50);
				//smb380_pause(50);
			}
		} while (need_calibration && (iteration_counter != tries));

		offset->x = offset_x;
		offset->y = offset_y;
		offset->z = offset_z;
		BDBG( "qian.liu bma150_calibration: iteration_counter  %d\n, need_calibration %d\n", iteration_counter, need_calibration);

	    if(iteration_counter == 1)                  /* no calibration needed at all */
	    {
		    error_flag |= 0x100;//error_flag |= NO_CALIB;
		    return error_flag;
		}
	    else if (iteration_counter == tries)        /* further calibration needed */
		{
		    error_flag |= need_calibration;
		    return error_flag;
		}

		return 0;
}

static int bma150_i2c_command(struct i2c_client *client, unsigned int cmd, void *arg)
{
	int rc = 0;
	bma150acc_t calibra_ori_data[6] = {
									{0,  0, -1},//正放
									{0,  0,  1},//反放
									{1,  0,  0},//正立
									{-1, 0,  0},//倒立
									{0,  1,  0},//左倒
									{0, -1,  0},//右倒
									};
	bma150acc_t offset;
	if(cmd == bma150_calibrate_onedir_cmd)
	{
		if((*(int*)arg) < 0 || (*(int*)arg) > 5)
			return -EINVAL;
		
		rc = bma150_calibration(calibra_ori_data[*(int*)arg], &offset, 20);
		if(rc != 0)
		{
			printk(KERN_WARNING "bma150_calibration failed %d",rc);
			return rc;
		}
	}
	else
		return -1;//command not supported
	return rc;
}

#ifdef  BMA150_IRQ
static irqreturn_t bma150_irq(int irq, void *dev_id)
{
	struct device      *dev = dev_id;
	struct bma150_i2c_driver_data *dd;
	BDBG("bma150_irq is called!\n");
	dd = dev_get_drvdata(dev);
	schedule_work(&dd->work_data);
	return IRQ_HANDLED;
}
#endif

static int bma150_open(struct input_dev *dev)
{
	int                 rc = 0;
	struct bma150_i2c_driver_data *dd = input_get_drvdata(dev);
#ifdef BMA150_IRQ
	if (!dd->i2c->irq)
		return -1;

	rc = request_irq(dd->i2c->irq,
			 &bma150_irq,
			 IRQF_TRIGGER_RISING,
			 BMA150_NAME,
			 &dd->i2c->dev);
#else
	rc=0;
#endif
	refresh_timer.function = refresh_data;
	refresh_timer.data =(unsigned long)dd;
	refresh_timer.expires  =  jiffies +bma_delay*HZ/1000;
	add_timer(&refresh_timer);
	BDBG("bma150_data port is opened!\n");

	return rc;
}

static void bma150_release(struct input_dev *dev)
{
#ifdef BMA150_IRQ
	struct bma150_i2c_driver_data *dd = input_get_drvdata(dev);
	free_irq(dd->i2c->irq, &dd->i2c->dev);
#endif
	refresh_timer.function = NULL;
	refresh_timer.data =0;
	refresh_timer.expires  = 0;

	
	del_timer_sync(&refresh_timer);
	BDBG("bma150_data port is closed!\n");

}

static void convert_regdata_to_accel_data(u8 *buf, struct bma150_accel_data *a)
{
	/* The BMA150 returns 10-bit values split over 2 bytes */
	a->accel_x = ((buf[0] & 0xC0) >> 6) | (buf[1] << 2);
	a->accel_y = ((buf[2] & 0xC0) >> 6) | (buf[3] << 2);
	a->accel_z = ((buf[4] & 0xC0) >> 6) | (buf[5] << 2);
	/* convert 10-bit signed value to 32-bit */
	if (a->accel_x & 0x200)
		a->accel_x = a->accel_x - 0x400;
	if (a->accel_y & 0x200)
		a->accel_y = a->accel_y - 0x400;
	if (a->accel_z & 0x200)
		a->accel_z = a->accel_z - 0x400;
	/* 0-based, units are 0.5 degree C */
	a->temp = buf[6] - BMA150_TEMP_OFFSET;
}

static void update_accel_data(unsigned long data){
	u8                          tx_buf[8];
	u8                          rx_buf[8];
	int                         rc;

	struct bma150_i2c_driver_data *dd=(struct bma150_i2c_driver_data *)data;
	struct bma150_accel_data    acc_data;


	rc = bma150_i2c_read(dd,BMA150_REG_ACCX_LS,rx_buf,8);
	
	if (rc)
		goto workf_exit;
	BDBG( "bma150_i2c: detected 1  %x\n", rx_buf[0]);
	BDBG( "bma150_i2c: detected 2  %x\n", rx_buf[1]);
	BDBG( "bma150_i2c: detected 3  %x\n", rx_buf[2]);
	BDBG( "bma150_i2c: detected 4  %x\n", rx_buf[3]);
	BDBG( "bma150_i2c: detected 5  %x\n", rx_buf[4]);
	BDBG( "bma150_i2c: detected 6  %x\n", rx_buf[5]);
	convert_regdata_to_accel_data(rx_buf, &acc_data);
	input_report_rel(dd->ip_dev, REL_X, acc_data.accel_x);
	input_report_rel(dd->ip_dev, REL_Y, acc_data.accel_y);
	input_report_rel(dd->ip_dev, REL_Z, acc_data.accel_z);
	input_report_rel(dd->ip_dev, REL_MISC, acc_data.temp);
	
	BDBG( "bma150_i2c:  acc_data.accel_x is %d\n\t   \
		                acc_data.accel_y is %d \n\t    \
			       	acc_data.accel_z  is %d \n\t", \
			       	acc_data.accel_x,  \
				acc_data.accel_y, \
				acc_data.accel_z);

	input_sync(dd->ip_dev);

	
#ifdef TRUE_DEVICE
	tx_buf[0] = BMA150_REG_CONTROL_0A;
	tx_buf[1] = BMA150_REG_C0A_RESET_INT;
	rc = bma150_i2c_write(dd,tx_buf,2);
	if (rc)
		goto workf_exit;
#endif
	return;

workf_exit:
	dev_err(&dd->ip_dev->dev, "%s: exit with error %d\n", __func__, rc);

}

static void bma150_work_f(struct work_struct *work)
{
	struct bma150_i2c_driver_data         *dd =
		container_of(work, struct bma150_i2c_driver_data, work_data);

	update_accel_data((unsigned long)dd);
}

static int __devexit bma150_power_down(struct bma150_i2c_driver_data *dd)
{
	char                tx_buf[2];
	int                 rc;

	tx_buf[0] = BMA150_REG_CONTROL_0A;
	tx_buf[1] = BMA150_REG_C0A_SLEEP;

	rc=bma150_i2c_write( dd, tx_buf, 2);
	return rc;
}

static int __devinit bma150_power_up(struct bma150_i2c_driver_data *dd)
{
	char                tx_buf[8];
	int                 rc;

	u8  				reg_15 = 0, reg_0a = 0;

	//set wakeup bit = 0
	rc = bma150_i2c_read(dd, BMA150_REG_CONTROL_15, &reg_15, 1);  
	tx_buf[0] = BMA150_REG_CONTROL_15;
	tx_buf[1] = reg_15 & ~0x01;
	rc += bma150_i2c_write(dd, tx_buf, 2);

	//set sleep bit = 0
	rc += bma150_i2c_read(dd, BMA150_REG_CONTROL_0A, &reg_0a, 1);
	tx_buf[0] = BMA150_REG_CONTROL_0A;
	tx_buf[1] = reg_0a & ~0x01;
	rc += bma150_i2c_write(dd, tx_buf, 2);

/*	
	while(1){
		tx_buf[0]=0xff;
		rc=bma150_i2c_write (dd ,tx_buf ,1);
		udelay(100);
	}

*/	
	return rc;
}

static int __devinit bma150_config(struct bma150_i2c_driver_data *dd)
{
	char                buf[8];
	int                 rc;
	u8                  reg_bandw;
	u8                  reg_15;

	bma150_power_up(dd);
	memset(&buf, 0, sizeof(buf));

	
	rc = bma150_i2c_read(dd, BMA150_REG_CHIPID, buf, 8);
	if (rc)
		goto config_exit;
	if (buf[0] == 0x00) {
		printk(KERN_ERR "bma150 accelerometer not detected\n");
		rc = -ENODEV;
		goto config_exit;
	}

	BDBG("bma150_i2c: detected chip id %d\n\n\n\n", buf[0] & 0x07);
	rc = bma150_i2c_read(dd, BMA150_REG_WIDTH_BANDW, buf, 2);
	if (rc)
		goto config_exit;

	reg_bandw = buf[0];
	reg_15    = buf[1];

	buf[0] = BMA150_REG_CONTROL_15;
	buf[1] = reg_15 | BMA150_REG_C15_EN_ADV_INT |
		BMA150_REG_C15_LATCH_INT;
	buf[2] = BMA150_REG_CONTROL_0B;
//ejding_20100129++  fix 7mA current waste:do not enable any interupt condition
//	buf[3] = BMA150_REG_C0B_ANY_MOTION |
//		BMA150_REG_C0B_ENABLE_HG  |
//		BMA150_REG_C0B_ENABLE_LG;
	buf[3] = 0; 
//ejding_20100129<<<
	buf[4] = BMA150_REG_ANY_MOTION_THRESH;
	buf[5] = BMA150_ANY_MOTION_INIT;
	buf[6] = BMA150_REG_WIDTH_BANDW;
	buf[7] = (reg_bandw & ~BMA150_REG_WID_BANDW_MASK) |
		BMA150_BANDW_INIT;
 
	rc = bma150_i2c_write(dd,buf, 8);

	if (rc)
		goto config_exit;

	//buf[0] = BMA150_REG_XOFFSET;
	//buf[1] = BMA150_XOFFSET_DATA;
	//buf[2] = 0x16;
	//buf[3] = 0xff;
	//rc = bma150_i2c_write(dd,buf, 4);
	return rc;


	BDBG( "bma150_i2c: bma150_i2c_config finished!\n");
config_exit:
	BDBG("WRONG NUMBER IS %d\n",rc);
	printk(KERN_ERR "bma150_i2c: bma150_i2c_config wrong!\n");
	return rc;
}


static int bma_ctl_open(struct inode *inode, struct file *file)
{
	if (atomic_cmpxchg(&ctl_open_count, 0, 1) == 0) {
		atomic_set(&ctl_open_count, 1);
		BDBG("BMA control file is opened ok!\n");

		return 0;
	}
	printk(KERN_ERR"BMA control file has been opened!\n");
	return -EBUSY;
}


static int bma_ctl_release(struct inode *inode, struct file *file)
{
	atomic_set(&ctl_open_count, 0);
	return 0;
}


static int bma_ctl_ioctl(struct inode *inode, struct file *file,
		      unsigned int cmd, unsigned long arg)

{

	void __user *argp = (void __user *)arg;
	short flag;

	switch (cmd) {
	case BMA_IOCTL_APP_SET_DELAY:
		if (copy_from_user(&flag, argp, sizeof(flag)))
			return -EFAULT;
		bma_delay = flag;
		break;
	case BMA_IOCTL_APP_GET_DELAY:
		flag = bma_delay;
		if (copy_to_user(argp,&flag, sizeof(flag)))
			return -EFAULT;
		break;
	default:
		return -ENOTTY;
	}

	return 0;

}

static struct file_operations bma_ctl_fops = {
	.owner = THIS_MODULE,
	.open = bma_ctl_open,
	.release = bma_ctl_release,
	.ioctl = bma_ctl_ioctl,
};


static struct miscdevice bma_ctl_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "bma150_ctl",
	.fops = &bma_ctl_fops,
};


#ifdef ECOMPASS_DAEMON
static int bma_ecomd_open(struct inode *inode, struct file *file)
{
	if (atomic_cmpxchg(&ecomd_open_count, 0, 1) == 0) {
		atomic_set(&ecomd_open_count, 1);
		BDBG("BMA ecompass deamon interface is opened ok!\n");
		return 0;
	}
	printk(KERN_ERR"BMA ecompass deamon has been opened ,busy!\n");
	return -EBUSY;
}


static int bma_ecomd_release(struct inode *inode, struct file *file)
{
	atomic_set(&ecomd_open_count, 0);
	return 0;
}


static int bma_ecomd_ioctl(struct inode *inode, struct file *file,
		      unsigned int cmd, unsigned long arg)

{

	void __user *argp = (void __user *)arg;
	u8                          rx_buf[8];
	struct bma150_accel_data    acc_data;

	switch (cmd) {
	case BMA_IOCTL_GETDATA:
		if (bma150_i2c_read(bma150_driver_data,BMA150_REG_ACCX_LS,rx_buf,8))
			return -EIO;
		convert_regdata_to_accel_data(rx_buf, &acc_data);
		if (copy_to_user(argp, &acc_data, sizeof(acc_data)))
			return -EFAULT;
		break;
	default:
		return -ENOTTY;
	}

	return 0;

}

static struct file_operations bma_ecomd_fops = {
	.owner = THIS_MODULE,
	.open = bma_ecomd_open,
	.release = bma_ecomd_release,
	.ioctl = bma_ecomd_ioctl,
};


static struct miscdevice bma_ecomd_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "bma150_ecomd",
	.fops = &bma_ecomd_fops,
};
#endif


#ifdef PROC_FS_DEBUG
static int proc_bma_swrite(struct file *file,
                            const char *buffer,
                            unsigned long count,
                            void *data)
{
	int rc=0;
        int len;
	int regdata;
	char buf[32];

	memset(buf, 0, 32);
        if(count > 32)
                len = 32;
        else
                len = count;


	//printk("count = %d\n", len);
        if(copy_from_user(buf, buffer, len))
                return -EFAULT;
	//printk("%s\n", ov2655_proc_buffer);
	sscanf(buf, "%x %x ", &reg, &regdata);
	BDBG("reg ==%x,regdata ==%x\n");
	buf[0] = reg;
	buf[1] = regdata;
	rc = bma150_i2c_write(bma150_driver_data,buf, 2);
	if(rc)
		printk(KERN_ERR "PROC WRITE WRONG\n");
	//sscanf(buf, "%d", &bma_delay);
	//printk("bmadelay proc set as: %d\n", bma_delay );

        return len;
}

static int proc_bma_sread(char *page, char **start,
                            off_t off, int count,
                            int *eof, void *data)
{
        int len;
	int rc = 0;
	char buf[4];

	rc = bma150_i2c_read(bma150_driver_data, reg, buf, 1);
	
       	len = sprintf(page, "reg = %d\n regdata = %d\n", reg,buf[0]);

        return len;
}
#endif


static int __devinit bma150_i2c_probe(struct i2c_client *i2c,
	const struct i2c_device_id *id)
{
	struct bma150_i2c_driver_data *dd;
	int                 rc;
//	char               *devname;
//	struct bma150_platform_data *pdata = i2c->dev.platform_data;

	BDBG("bma150_i2c_probe\n");
	BDBG("%x==================================\n",i2c->addr);
	BDBG("i2c->adapter.name is %s\n",i2c->adapter->name);
	BDBG("i2c->adapter.nr is %d===============\n",i2c->adapter->nr);

	if (!i2c_check_functionality(i2c->adapter, I2C_FUNC_I2C)) {
		rc = -ENOTSUPP;
		goto probe_exit;
	}

	dd = kzalloc(sizeof(struct bma150_i2c_driver_data), GFP_KERNEL);
	if (!dd) {
		rc = -ENOMEM;
		goto probe_exit;
	}
	bma150_driver_data=dd;

	mutex_lock(&bma150_dd_lock);
	list_add_tail(&dd->next_dd, &dd_list);
	mutex_unlock(&bma150_dd_lock);
	INIT_WORK(&dd->work_data, bma150_work_f);
	dd->i2c = i2c;
#ifdef TRUE_DEVICE	
	rc = bma150_config(dd);
#else
	rc=0;
#endif
	if (rc)
		goto probe_err_cfg;
#ifdef  BMA150_IRQ
	rc =msm_bma_i2c_gpio_setup();
	if(rc)
		goto probe_err_cfg;
#endif	
	i2c_set_clientdata(i2c,dd);
	dd->ip_dev = input_allocate_device();
	if (!dd->ip_dev) {
		rc = -ENOMEM;
		goto probe_err_reg;
	}
	input_set_drvdata(dd->ip_dev, dd);
//	snprintf(devname, sizeof(BMA150_DEVICE_NAME) + 1, BMA150_DEVICE_NAME,
//		 i2c->master->bus_num, i2c->chip_select);
	dd->ip_dev->open       = bma150_open;
	dd->ip_dev->close      = bma150_release;
	dd->ip_dev->name       = BMA150_NAME;
	dd->ip_dev->phys       = BMA150_NAME;
	dd->ip_dev->id.vendor  = BMA150_VENDORID;
	dd->ip_dev->id.product = 1;
	dd->ip_dev->id.version = 1;
	__set_bit(EV_REL,    dd->ip_dev->evbit);
	__set_bit(REL_X,     dd->ip_dev->relbit);
	__set_bit(REL_Y,     dd->ip_dev->relbit);
	__set_bit(REL_Z,     dd->ip_dev->relbit);
	__set_bit(REL_MISC,  dd->ip_dev->relbit);
	rc = input_register_device(dd->ip_dev);
	if (rc) {
		dev_err(&dd->ip_dev->dev,
			"bma150_probe: input_register_device rc=%d\n",
		       rc);
		goto probe_err_reg_dev;
	}

	rc = misc_register(&bma_ctl_device);
	if (rc) {
		printk(KERN_ERR
		       "bma150_probe: bma_ctl_device register failed\n");
		goto probe_misc_device_register_failed;
	}

#ifdef ECOMPASS_DAEMON
	rc = misc_register(&bma_ecomd_device);
	if (rc) {
		printk(KERN_ERR
		       "bma150_probe: bma_ecomd_device register failed\n");
		goto probe_misc_device_register_failed;
	}

#endif

	init_timer(&refresh_timer);
//	dump_stack();

#ifdef PROC_FS_DEBUG
       /* create readfile */
	bmaproc = create_proc_entry("bmadelay",
	                                 0644, NULL);
	if(bmaproc == NULL) {
	          rc  = -ENOMEM;
	          goto proc_entry_create_failed;
	}
        bmaproc->read_proc = proc_bma_sread;
        bmaproc->write_proc = proc_bma_swrite;
	bmaproc->owner = THIS_MODULE;
#endif
	BDBG("bma150_i2c_probe ok!\n\n\n\n");
//	dev_set_drvdata(bma_ctl_device.this_device,dd);
	return rc;

#ifdef PROC_FS_DEBUG
proc_entry_create_failed:
#endif	
probe_misc_device_register_failed:
probe_err_reg_dev:
	dd->ip_dev = NULL;
	input_free_device(dd->ip_dev);
probe_err_reg:

	i2c_set_clientdata(i2c,NULL);
probe_err_cfg:
	mutex_lock(&bma150_dd_lock);
	list_del(&dd->next_dd);
	mutex_unlock(&bma150_dd_lock);
	kfree(dd);
probe_exit:
	return rc;
}

static int __devexit bma150_i2c_remove(struct i2c_client *i2c)
{
	struct bma150_i2c_driver_data *dd;
	int                 rc;
//	const char	   *devname;
//	struct bma150_platform_data *pdata = i2c->dev.platform_data;

	dd = i2c_get_clientdata(i2c);
	
//	devname = dd->ip_dev->phys;

	rc = bma150_power_down(dd);
	if (rc)
		dev_err(&dd->ip_dev->dev,
			"%s: power down failed with error %d\n",
			__func__, rc);
	input_unregister_device(dd->ip_dev);
	i2c_set_clientdata(i2c, NULL);
#ifdef  BMA150_IRQ
	msm_bma_i2c_gpio_teardown();
#endif
	mutex_lock(&bma150_dd_lock);
	list_del(&dd->next_dd);
	mutex_unlock(&bma150_dd_lock);
//	kfree(devname);
	kfree(dd);

	del_timer(&refresh_timer);

return 0;
}



static const struct i2c_device_id bma150_i2c_id[] = {
	{ "bma150_i2c", 0},
	{ },
};

static struct i2c_driver bma150_i2c_driver = {
	.id_table = bma150_i2c_id,
	.probe  = bma150_i2c_probe,
	.remove = __exit_p(bma150_i2c_remove),
	.suspend  = bma150_i2c_suspend,
	.resume   = bma150_i2c_resume,
	.command  = bma150_i2c_command,

	.driver = {
		.owner = THIS_MODULE,
		.name = "bma150_i2c",
	},
};



static int __init bma150_i2c_init(void)
{
	int rc;

	INIT_LIST_HEAD(&dd_list);
	mutex_init(&bma150_dd_lock);
	rc= i2c_add_driver(&bma150_i2c_driver);
	BDBG("bma150_i2c_init ok!\n\n\n\n");
	return rc;
}
module_init(bma150_i2c_init);

static void __exit bma150_i2c_exit(void)
{
	i2c_del_driver(&bma150_i2c_driver);

}
module_exit(bma150_i2c_exit);
