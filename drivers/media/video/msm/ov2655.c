/* Copyright (c) 2009, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 *
 */

#include <linux/delay.h>
#include <linux/types.h>
#include <linux/i2c.h>
#include <linux/uaccess.h>
#include <linux/miscdevice.h>
#include <media/msm_camera.h>
#include <mach/gpio.h>
#include <mach/camera.h>
#include <mach/vreg.h>
#include <linux/proc_fs.h>

#undef CDBG 
#define CDBG(fmt, args...) printk("ov2655: %s %d() | " fmt, __func__, __LINE__,  ##args)
#define CDB printk(KERN_INFO "litao debug - %s %5d %s\n", __FILE__, __LINE__, __func__);

/* OV2655 Registers and their values */
/* Sensor Core Registers */
#define  REG_OV2655_MODEL_ID 0x3012
#define  OV2655_MODEL_ID     0x5626

/*  SOC Registers Page 1  */
#define  REG_OV2655_SENSOR_RESET     0x3012

unsigned char exposure_line_h = 0, exposure_line_l = 0;
unsigned char extra_exposure_line_h = 0, extra_exposure_line_l = 0;

struct ov2655_work {
	struct work_struct work;
};

static struct  ov2655_work *ov2655_sensorw;
static struct  i2c_client *ov2655_client;

struct ov2655_ctrl_t {
	const struct msm_camera_sensor_info *sensordata;
};


static struct ov2655_ctrl_t *ov2655_ctrl;
static void OV2655_AE_Transfer(void);

static DECLARE_WAIT_QUEUE_HEAD(ov2655_wait_queue);
DECLARE_MUTEX(ov2655_sem);

/*litao add for ov2655 proc file*/
struct ov2655_proc_t {
        unsigned int i2c_addr;
        unsigned int i2c_data;
};


#define OV2655_PROC_NAME "ov2655"
#define SINGLE_OP_NAME "sbdata"
#define DOUBLE_READ_NAME "double_byte_read"
#define I2C_ADDR_NAME "i2c_addr"
#define OV2655_PROC_LEN 16
static struct proc_dir_entry *ov2655_dir, *s_file,
	             *w_file, *dr_file;
static char ov2655_proc_buffer[OV2655_PROC_LEN];
static struct ov2655_proc_t ov2655_proc_dt = {
	.i2c_addr = 0,
	.i2c_data = 0,
};
/*litao add end*/
/*=============================================================*/


static int ov2655_reset(const struct msm_camera_sensor_info *dev)
{
	int rc = 0;
	CDBG("ov2655 reset\n");

	rc = gpio_request(dev->sensor_reset, "ov2655");

	if (!rc) {
		rc = gpio_direction_output(dev->sensor_reset, 0);
		mdelay(20);
		rc = gpio_direction_output(dev->sensor_reset, 1);
	}

	gpio_free(dev->sensor_reset);
	return rc;
}

static int32_t ov2655_i2c_txdata(unsigned short saddr,
	unsigned char *txdata, int length)
{
	struct i2c_msg msg[] = {
		{
			.addr = saddr,
			.flags = 0,
			.len = length,
			.buf = txdata,
		},
	};

	if (i2c_transfer(ov2655_client->adapter, msg, 1) < 0) {
		CDBG("ov2655_i2c_txdata faild\n");
		return -EIO;
	}

	return 0;
}

static int32_t write_cmos_sensor(unsigned short waddr, unsigned char wdata)
{
	int32_t rc = -EFAULT;
	unsigned char buf[4];
	printk("w %4x %2x\n", waddr, wdata);

	memset(buf, 0, sizeof(buf));
	buf[0] = (waddr & 0xFF00)>>8;
	buf[1] = (waddr & 0x00FF);
	buf[2] = wdata;
	rc = ov2655_i2c_txdata(ov2655_client->addr, buf, 3);

	if (rc < 0)
		CDBG(
		"i2c_write failed, addr = 0x%x, val = 0x%x!\n",
		waddr, wdata);

	return rc;
}

static int ov2655_i2c_rxdata(unsigned short saddr,
	unsigned char *rxdata, int length)
{
	struct i2c_msg msgs[] = {
	{
		.addr   = saddr,
		.flags = 0,
		.len   = 2,
		.buf   = rxdata,
	},
	{
		.addr   = saddr,
		.flags = I2C_M_RD,
		.len   = length,
		.buf   = rxdata,
	},
	};

	if (i2c_transfer(ov2655_client->adapter, msgs, 2) < 0) {
		CDBG("ov2655_i2c_rxdata failed!\n");
		return -EIO;
	}

	return 0;
}

static int32_t read_cmos_sensor_2(unsigned short raddr, void *rdata)
{
	int32_t rc = 0;
	unsigned char buf[4];
	int len = 2;

//printk("raddr=%4x, len=%4x\n", raddr, len);
	memset(buf, 0, sizeof(buf));

	buf[0] = (raddr & 0xFF00)>>8;
	buf[1] = (raddr & 0x00FF);

	rc = ov2655_i2c_rxdata(ov2655_client->addr, buf, len);
	memcpy(rdata, buf, len);
	//*(char*)rdata = buf[1];
	//*(char*)(rdata+1) = buf[0];


	if (rc < 0)
		CDBG("ov2655_i2c_read failed!\n");

	return rc;
}

static void reg_dump()
{
	unsigned short a, d;
	int rc = 0;

	printk("reg dump===\n");
	a = 0x3000;
	while(a < 0x3500){
		if(!(a & 0x0f))
			printk("\n%4x: ", a);
		rc = read_cmos_sensor_2(a, &d);
		if(rc < 0){
			printk("error\n");
			return;
		}
		printk("%4x ", d);
		a += 2;
	}
	printk("\nreg dump over===\n");
}

static int32_t read_cmos_sensor(unsigned short raddr, void *rdata, int len)
{
	int32_t rc = 0;
	unsigned char buf[4];
printk("raddr=%x, len=%x\n", raddr, len);

	if (!rdata || len > 4)
		return -EIO;

	memset(buf, 0, sizeof(buf));

	buf[0] = (raddr & 0xFF00)>>8;
	buf[1] = (raddr & 0x00FF);

	rc = ov2655_i2c_rxdata(ov2655_client->addr, buf, len);

	memcpy(rdata, buf, len);

	if (rc < 0)
		CDBG("ov2655_i2c_read failed!\n");

	return rc;
}

static unsigned char read_cmos_sensor_1(unsigned short raddr)
{
	unsigned short tmp16;
	unsigned char tmp;

	read_cmos_sensor(raddr, &tmp16, 2);
	tmp = tmp16 & 0xff;
	return tmp;
}

static long ov2655_reg_init(void)
{
	long rc;
//////////////////////////////////////////////////////////////////////////////////
	/* PLL Setup Start */
	/* PLL Setup End   */
	/* Configure sensor for Preview mode and Snapshot mode */
	/* Configure for Noise Reduction, Saturation and Aperture Correction */
	/* Set Color Kill Saturation point to optimum value */
//////////////////////////////////////////////////////////////////////////////////

 	//IO & Clock & Analog Setup
	CDBG("ov2655_reg_init\n");
	rc = write_cmos_sensor(0x308c,0x80); //TMC12: DIS_MIPI_RW
	if (rc < 0) return rc;
	rc = write_cmos_sensor(0x308d,0x0e); //TMC13: MIPI disable
	if (rc < 0) return rc;
	rc = write_cmos_sensor(0x360b,0x00);
	if (rc < 0) return rc;
	rc = write_cmos_sensor(0x30b0,0xff); //IO_CTRL0: Cy[7:0]
	if (rc < 0) return rc;
	rc = write_cmos_sensor(0x30b1,0xff); //IO_CTRL1: C_VSYNC,C_STROBE,C_PCLK,C_HREF,Cy[9:8]
	if (rc < 0) return rc;
	rc = write_cmos_sensor(0x30b2,0x24); //IO_CTRL2: R_PAD[3:0]
	if (rc < 0) return rc;

	rc = write_cmos_sensor(0x300f,0xa6);
	if (rc < 0) return rc;
	rc = write_cmos_sensor(0x3010,0x81);
	if (rc < 0) return rc;
	rc = write_cmos_sensor(0x3082,0x01);
	if (rc < 0) return rc;
	rc = write_cmos_sensor(0x30f4,0x01);
	if (rc < 0) return rc;
	rc = write_cmos_sensor(0x3090,0x33);
	if (rc < 0) return rc;
	rc = write_cmos_sensor(0x3091,0xc0);
	if (rc < 0) return rc;
	rc = write_cmos_sensor(0x30ac,0x42);
	if (rc < 0) return rc;

	rc = write_cmos_sensor(0x30d1,0x08);
	if (rc < 0) return rc;
	rc = write_cmos_sensor(0x30a8,0x56);
	if (rc < 0) return rc;
	rc = write_cmos_sensor(0x3015,0x42); //AUTO_3: AGC ceiling = 4x, 5dummy frame
	if (rc < 0) return rc;
	rc = write_cmos_sensor(0x3093,0x00);
	if (rc < 0) return rc;
	rc = write_cmos_sensor(0x307e,0xe5); //TMC8: AGC[7:6]=b'11
	if (rc < 0) return rc;
	rc = write_cmos_sensor(0x3079,0x00);
	if (rc < 0) return rc;
	rc = write_cmos_sensor(0x30aa,0x42);
	if (rc < 0) return rc;
	rc = write_cmos_sensor(0x3017,0x40); //AUTO_5: disable data drop, manual banding counter=0
	if (rc < 0) return rc;
	rc = write_cmos_sensor(0x30f3,0x82);
	if (rc < 0) return rc;
	rc = write_cmos_sensor(0x306a,0x0f); //0x0c->0x0f Joe 0814 : BLC
	if (rc < 0) return rc;
	rc = write_cmos_sensor(0x306d,0x00);
	if (rc < 0) return rc;
	rc = write_cmos_sensor(0x336a,0x3c);
	if (rc < 0) return rc;
	rc = write_cmos_sensor(0x3076,0x6a); //TMC0: VSYNC drop option: drop
	if (rc < 0) return rc;
	rc = write_cmos_sensor(0x30d9,0x95);
	if (rc < 0) return rc;
	rc = write_cmos_sensor(0x3016,0xc2); //Joe=0xc2 0808 //org=0x82//AUTO_4: max exposure adjust option=2
	if (rc < 0) return rc;
	rc = write_cmos_sensor(0x3601,0x30);
	if (rc < 0) return rc;
	rc = write_cmos_sensor(0x304e,0x88);
	if (rc < 0) return rc;
	rc = write_cmos_sensor(0x30f1,0x82);
	if (rc < 0) return rc;
	rc = write_cmos_sensor(0x306f,0x14);
	if (rc < 0) return rc;
	rc = write_cmos_sensor(0x302a,0x02);
	if (rc < 0) return rc;
	rc = write_cmos_sensor(0x302b,0x6a);
	if (rc < 0) return rc;

	rc = write_cmos_sensor(0x3012,0x10);
	if (rc < 0) return rc;
	/*
	rc = write_cmos_sensor(0x3011,0x01);
	if (rc < 0) return rc;
	*/

	//AEC/AGC
	rc = write_cmos_sensor(0x3013,0xf7); //AUTO_1: banding filter, AGC auto, AEC auto
	if (rc < 0) return rc;
	rc = write_cmos_sensor(0x301c,0x03); //AECG_MAX50: 50Hz smooth banding max step=0x13
	if (rc < 0) return rc;
	rc = write_cmos_sensor(0x301d,0x03); //AECG_MAX60: 60Hz smooth banding max step=0x17
	if (rc < 0) return rc;
	rc = write_cmos_sensor(0x3070,0x3e); //BD50:LSB
	if (rc < 0) return rc;
	rc = write_cmos_sensor(0x3072,0x4d); //BD60:LSB
	if (rc < 0) return rc;

	rc = write_cmos_sensor(0x3018,0x60); //high limit //AE target   78 70 78 6a
	if (rc < 0) return rc;            
	rc = write_cmos_sensor(0x3019,0x50); //low limit  68 60 60 5a
	if (rc < 0) return rc;
	rc = write_cmos_sensor(0x301a,0xd4); //0xa3 by Joe 0805 //AE step  a3 93 93 d4
	if (rc < 0) return rc;

	//D5060
	rc = write_cmos_sensor(0x30af,0x00);
	if (rc < 0) return rc;
	rc = write_cmos_sensor(0x3048,0x1f);
	if (rc < 0) return rc;
	rc = write_cmos_sensor(0x3049,0x4e);
	if (rc < 0) return rc;
	rc = write_cmos_sensor(0x304a,0x20);
	if (rc < 0) return rc;
	rc = write_cmos_sensor(0x304b,0x02);
	if (rc < 0) return rc;
	rc = write_cmos_sensor(0x304c,0x00);
	if (rc < 0) return rc;
	rc = write_cmos_sensor(0x304d,0x02);
	if (rc < 0) return rc;
	rc = write_cmos_sensor(0x304f,0x20);
	if (rc < 0) return rc;
	rc = write_cmos_sensor(0x30a3,0x10);
	if (rc < 0) return rc;
	rc = write_cmos_sensor(0x3013,0xf7); 
	if (rc < 0) return rc;
	rc = write_cmos_sensor(0x3014,0xa5); //R1D bit6 always = 0 , bit[5]=1, bit[0]=1
	if (rc < 0) return rc;
	rc = write_cmos_sensor(0x3071,0x00);
	if (rc < 0) return rc;
	rc = write_cmos_sensor(0x3070,0x5d); //50hz banding
	if (rc < 0) return rc;
	rc = write_cmos_sensor(0x3073,0x00);
	if (rc < 0) return rc;
	rc = write_cmos_sensor(0x3072,0x4d); //60hz banding
	if (rc < 0) return rc;
	rc = write_cmos_sensor(0x301c,0x06); //50hz banding
	if (rc < 0) return rc;
	rc = write_cmos_sensor(0x301d,0x07); //60hz banding
	if (rc < 0) return rc;
	rc = write_cmos_sensor(0x304d,0x42);
	if (rc < 0) return rc;
	rc = write_cmos_sensor(0x304a,0x00); //Disable 50/60 auto detection function, due to ov2650 no this function
	if (rc < 0) return rc;
	rc = write_cmos_sensor(0x304f,0x00); //Disable 50/60 auto detection function, due to ov2650 no this function
	if (rc < 0) return rc;
	rc = write_cmos_sensor(0x3095,0x07);
	if (rc < 0) return rc;
	rc = write_cmos_sensor(0x3096,0x16);
	if (rc < 0) return rc;
	rc = write_cmos_sensor(0x3097,0x1d);
	if (rc < 0) return rc;

	//Window Setup
	rc = write_cmos_sensor(0x3020,0x01); //HS
	if (rc < 0) return rc;
	rc = write_cmos_sensor(0x3021,0x18); //HS 0x18 0813
	if (rc < 0) return rc;
	rc = write_cmos_sensor(0x3022,0x00);
	if (rc < 0) return rc;
	rc = write_cmos_sensor(0x3023,0x06);
	if (rc < 0) return rc;
	rc = write_cmos_sensor(0x3024,0x06);
	if (rc < 0) return rc;
	rc = write_cmos_sensor(0x3025,0x58);
	if (rc < 0) return rc;
	rc = write_cmos_sensor(0x3026,0x02);
	if (rc < 0) return rc;
	rc = write_cmos_sensor(0x3027,0x5e);
	if (rc < 0) return rc;
	rc = write_cmos_sensor(0x3088,0x03);
	if (rc < 0) return rc;
	rc = write_cmos_sensor(0x3089,0x20);
	if (rc < 0) return rc;
	rc = write_cmos_sensor(0x308a,0x02);
	if (rc < 0) return rc;
	rc = write_cmos_sensor(0x308b,0x58);
	if (rc < 0) return rc;
	rc = write_cmos_sensor(0x3316,0x64);
	if (rc < 0) return rc;
	rc = write_cmos_sensor(0x3317,0x25);
	if (rc < 0) return rc;
	rc = write_cmos_sensor(0x3318,0x80);
	if (rc < 0) return rc;
	rc = write_cmos_sensor(0x3319,0x08);
	if (rc < 0) return rc;
	rc = write_cmos_sensor(0x331a,0x64);
	if (rc < 0) return rc;
	rc = write_cmos_sensor(0x331b,0x4b);
	if (rc < 0) return rc;
	rc = write_cmos_sensor(0x331c,0x00);
	if (rc < 0) return rc;
	rc = write_cmos_sensor(0x331d,0x38);
	if (rc < 0) return rc;
	rc = write_cmos_sensor(0x3100,0x00);
	if (rc < 0) return rc;

	//AWB
	rc = write_cmos_sensor(0x3320,0xfa);   
	if (rc < 0) return rc;
	rc = write_cmos_sensor(0x3321,0x11);   
	if (rc < 0) return rc;
	rc = write_cmos_sensor(0x3322,0x92);   
	if (rc < 0) return rc;
	rc = write_cmos_sensor(0x3323,0x01);   
	if (rc < 0) return rc;
	rc = write_cmos_sensor(0x3324,0x97); // 92   
	if (rc < 0) return rc;
	rc = write_cmos_sensor(0x3325,0x02);   
	if (rc < 0) return rc;
	rc = write_cmos_sensor(0x3326,0xff);   
	if (rc < 0) return rc;
	rc = write_cmos_sensor(0x3327,0x0c); // 0f  
	if (rc < 0) return rc;
	rc = write_cmos_sensor(0x3328,0x10); // 0f   
	if (rc < 0) return rc;
	rc = write_cmos_sensor(0x3329,0x13); // 14   
	if (rc < 0) return rc;
	rc = write_cmos_sensor(0x332a,0x5c); // 66   
	if (rc < 0) return rc;
	rc = write_cmos_sensor(0x332b,0x59);   //5f -> 5c
	if (rc < 0) return rc;
	rc = write_cmos_sensor(0x332c,0xbe);   //a5 -> 89
	if (rc < 0) return rc;
	rc = write_cmos_sensor(0x332d,0xa8);   //ac -> 96
	if (rc < 0) return rc;
	rc = write_cmos_sensor(0x332e,0x3c);   //35 -> 3d
	if (rc < 0) return rc;
	rc = write_cmos_sensor(0x332f,0x1a); // 2f  
	if (rc < 0) return rc;
	rc = write_cmos_sensor(0x3330,0x4f); // 57   
	if (rc < 0) return rc;
	rc = write_cmos_sensor(0x3331,0x29); // 3d   
	if (rc < 0) return rc;
	rc = write_cmos_sensor(0x3332,0xf0); // f0   
	if (rc < 0) return rc;
	rc = write_cmos_sensor(0x3333,0x0a); // 10  
	if (rc < 0) return rc;
	rc = write_cmos_sensor(0x3334,0xf0);   
	if (rc < 0) return rc;
	rc = write_cmos_sensor(0x3335,0xf0);   
	if (rc < 0) return rc;
	rc = write_cmos_sensor(0x3336,0xf0);   
	if (rc < 0) return rc;
	rc = write_cmos_sensor(0x3337,0x40);   
	if (rc < 0) return rc;
	rc = write_cmos_sensor(0x3338,0x40);   
	if (rc < 0) return rc;
	rc = write_cmos_sensor(0x3339,0x40);   
	if (rc < 0) return rc;
	rc = write_cmos_sensor(0x333a,0x00);   
	if (rc < 0) return rc;
	rc = write_cmos_sensor(0x333b,0x00);   
	if (rc < 0) return rc;

	//Color Matrix
	rc = write_cmos_sensor(0x3380,0x27); //28->2d
	if (rc < 0) return rc;            
	rc = write_cmos_sensor(0x3381,0x5c); //48->4d
	if (rc < 0) return rc;            
	rc = write_cmos_sensor(0x3382,0x0a); //10->13 0819 Joe for ?G?��
	if (rc < 0) return rc;            
	rc = write_cmos_sensor(0x3383,0x2a);  
	if (rc < 0) return rc;            
	rc = write_cmos_sensor(0x3384,0xae);  
	if (rc < 0) return rc;            
	rc = write_cmos_sensor(0x3385,0xd8);  
	if (rc < 0) return rc;            
	rc = write_cmos_sensor(0x3386,0xc2);  
	if (rc < 0) return rc;            
	rc = write_cmos_sensor(0x3387,0xbf);  
	if (rc < 0) return rc;            
	rc = write_cmos_sensor(0x3388,0x03);  
	if (rc < 0) return rc;            
	rc = write_cmos_sensor(0x3389,0x98);
	if (rc < 0) return rc;            
	rc = write_cmos_sensor(0x338a,0x01);
	if (rc < 0) return rc;

	//Gamma
	rc = write_cmos_sensor(0x3340,0x06);  
	if (rc < 0) return rc;            
	rc = write_cmos_sensor(0x3341,0x14);  
	if (rc < 0) return rc;            
	rc = write_cmos_sensor(0x3342,0x2b);  
	if (rc < 0) return rc;            
	rc = write_cmos_sensor(0x3343,0x42);
	if (rc < 0) return rc;            
	rc = write_cmos_sensor(0x3344,0x55);  
	if (rc < 0) return rc;            
	rc = write_cmos_sensor(0x3345,0x65);  
	if (rc < 0) return rc;            
	rc = write_cmos_sensor(0x3346,0x70);  
	if (rc < 0) return rc;            
	rc = write_cmos_sensor(0x3347,0x7c);  
	if (rc < 0) return rc;            
	rc = write_cmos_sensor(0x3348,0x86);  
	if (rc < 0) return rc;            
	rc = write_cmos_sensor(0x3349,0x96);
	if (rc < 0) return rc;            
	rc = write_cmos_sensor(0x334a,0xa3);
	if (rc < 0) return rc;
	rc = write_cmos_sensor(0x334b,0xaf);
	if (rc < 0) return rc;
	rc = write_cmos_sensor(0x334c,0xc4);
	if (rc < 0) return rc;
	rc = write_cmos_sensor(0x334d,0xd7);
	if (rc < 0) return rc;
	rc = write_cmos_sensor(0x334e,0xe8);
	if (rc < 0) return rc;
 	rc = write_cmos_sensor(0x334f,0x20);
	if (rc < 0) return rc;

	rc = write_cmos_sensor(0x30d9, 0x95);
	if (rc < 0) return rc;

	//Lens correction
	//R
	rc = write_cmos_sensor(0x3350,0x32);
	if (rc < 0) return rc;            
	rc = write_cmos_sensor(0x3351,0x25);
	if (rc < 0) return rc;            
	rc = write_cmos_sensor(0x3352,0x80);
	if (rc < 0) return rc;            
	rc = write_cmos_sensor(0x3353,0x5a);
	if (rc < 0) return rc;            
	rc = write_cmos_sensor(0x3354,0x00);
	if (rc < 0) return rc;            
	rc = write_cmos_sensor(0x3355,0xf6);
	if (rc < 0) return rc;
	//G
	rc = write_cmos_sensor(0x3356,0x32);
	if (rc < 0) return rc;            
	rc = write_cmos_sensor(0x3357,0x25);
	if (rc < 0) return rc;            
	rc = write_cmos_sensor(0x3358,0x80);
	if (rc < 0) return rc;            
	rc = write_cmos_sensor(0x3359,0x50);
	if (rc < 0) return rc;            
	rc = write_cmos_sensor(0x335a,0x00);
	if (rc < 0) return rc;            
	rc = write_cmos_sensor(0x335b,0xf6);
	if (rc < 0) return rc;
	//B
	rc = write_cmos_sensor(0x335c,0x32);
	if (rc < 0) return rc;            
	rc = write_cmos_sensor(0x335d,0x25);
	if (rc < 0) return rc;            
	rc = write_cmos_sensor(0x335e,0x80);
	if (rc < 0) return rc;            
	rc = write_cmos_sensor(0x335f,0x53);
	if (rc < 0) return rc;            
	rc = write_cmos_sensor(0x3360,0x00);
	if (rc < 0) return rc;            
	rc = write_cmos_sensor(0x3361,0xf6);

	rc = write_cmos_sensor(0x3363,0x70);
	if (rc < 0) return rc;
	rc = write_cmos_sensor(0x3364,0x7f);
	if (rc < 0) return rc;
	rc = write_cmos_sensor(0x3365,0x00);
	if (rc < 0) return rc;
	rc = write_cmos_sensor(0x3366,0x00);
	if (rc < 0) return rc;

	rc = write_cmos_sensor(0x3362,0x80);
	if (rc < 0) return rc;

	//saturation
	rc = write_cmos_sensor(0x3391,0x06);
	if (rc < 0) return rc;
	rc = write_cmos_sensor(0x3394,0x3f); // value
	if (rc < 0) return rc;
	rc = write_cmos_sensor(0x3395,0x3f); // value
	if (rc < 0) return rc;

	//UVadjust
	rc = write_cmos_sensor(0x3301,0xff);
	if (rc < 0) return rc;
	rc = write_cmos_sensor(0x338B,0x1b);
	if (rc < 0) return rc;
	rc = write_cmos_sensor(0x338c,0x1f);
	if (rc < 0) return rc;
	rc = write_cmos_sensor(0x338d,0x40);
	if (rc < 0) return rc;

	//Sharpness/De-noise
	rc = write_cmos_sensor(0x3370,0xff);
	if (rc < 0) return rc;
	rc = write_cmos_sensor(0x3371,0x00);
	if (rc < 0) return rc;
	rc = write_cmos_sensor(0x3372,0x00);
	if (rc < 0) return rc;
	rc = write_cmos_sensor(0x3373,0x80);
	if (rc < 0) return rc;
	rc = write_cmos_sensor(0x3374,0x10);
	if (rc < 0) return rc;
	rc = write_cmos_sensor(0x3375,0x10);
	if (rc < 0) return rc;
	rc = write_cmos_sensor(0x3376,0x04);
	if (rc < 0) return rc;
	rc = write_cmos_sensor(0x3377,0x00);
	if (rc < 0) return rc;
	rc = write_cmos_sensor(0x3378,0x04);
	if (rc < 0) return rc;
	rc = write_cmos_sensor(0x3379,0x80);
	if (rc < 0) return rc;

	//BLC
	rc = write_cmos_sensor(0x3069,0x92);
	if (rc < 0) return rc;
	rc = write_cmos_sensor(0x307c,0x10);
	if (rc < 0) return rc;
	rc = write_cmos_sensor(0x3087,0x02);
	if (rc < 0) return rc;

	//Other functions
	rc = write_cmos_sensor(0x3300,0xfc);
	if (rc < 0) return rc;
	rc = write_cmos_sensor(0x3302,0x11);
	if (rc < 0) return rc;
	rc = write_cmos_sensor(0x3400,0x00);
	if (rc < 0) return rc;
	rc = write_cmos_sensor(0x3606,0x20);
	if (rc < 0) return rc;
	rc = write_cmos_sensor(0x3601,0x30);
	if (rc < 0) return rc;
	rc = write_cmos_sensor(0x30f3,0x83);
	if (rc < 0) return rc;
	rc = write_cmos_sensor(0x304e,0x88);
	if (rc < 0) return rc;
	
	if (rc < 0) return rc;
	rc = write_cmos_sensor(0x3086,0x00);
	if (rc < 0) return rc;

	rc = write_cmos_sensor(0x302d, 0x00); 
	if (rc < 0) return rc;
	rc = write_cmos_sensor(0x302e, 0x00);
	if (rc < 0) return rc;
	//litao add test bar
	//rc = write_cmos_sensor(0x3308, 0x01);
	//if (rc < 0) return rc;
	/* OV2650_YUV_sensor_initial_setting */

	return 0;
}

static long ov2655_priview(void)
{
    long rc = 0;
    unsigned char tmp;
    unsigned short tmp16;

    //4  <1> preview config sequence
CDBG("ov2655 preview:\n");

    /*To be debug*/
    //write_cmos_sensor(0x3002, exposure_line_h);
    //write_cmos_sensor(0x3003, exposure_line_l);
    write_cmos_sensor(0x3002, 0);
    write_cmos_sensor(0x3003, 0);

    //YUV
    //SVGA (800x600)
    // setup sensor output ROI
    write_cmos_sensor(0x3016,0xc2); //0xd3->c3 0818 (normal mode AE)
    write_cmos_sensor(0x30d9,0x95);
    
    write_cmos_sensor(0x3023, 0x06); //array output vertical start, VS[7:0] = 6
    write_cmos_sensor(0x3012, 0x10); //<4>vario
    write_cmos_sensor(0x302A, 0x02); //VTS[15:8], V total size = 618
    write_cmos_sensor(0x302B, 0x6a); //VTS[7:0]
    write_cmos_sensor(0x306f, 0x14); //<6:4>window,<3:0>targ_s
    write_cmos_sensor(0x3024, 0x06); //HW[15:8], H width = 1624
    write_cmos_sensor(0x3025, 0x58); //HW[7:0]
    write_cmos_sensor(0x3026, 0x02); //VH[15:8], V hight = 609
    write_cmos_sensor(0x3027, 0x5e); //VH[7:0]
    write_cmos_sensor(0x3088, 0x03); //ISP_XOUT[15:8]: x_size_in[11:0] = 800
    write_cmos_sensor(0x3089, 0x20); //ISP_XOUT[7:0]:
    write_cmos_sensor(0x308a, 0x02); //ISP_YOUT[15:8]: y_size_in[11:0] = 600
    write_cmos_sensor(0x308b, 0x58); //ISP_YOUT[7:0]:
    write_cmos_sensor(0x331a, 0x64); //H
    write_cmos_sensor(0x331b, 0x4b); //V
    write_cmos_sensor(0x331c, 0x00); //VH
    write_cmos_sensor(0x3100, 0x00); //<3>bypass_win,<2>bypass_fmt,<1>rgb_sel,<0>snr_sel
    write_cmos_sensor(0x3319, 0x08); //for awb timing
    write_cmos_sensor(0x331d, 0x38); //for awb timing
    write_cmos_sensor(0x3302, 0x11);
    write_cmos_sensor(0x3316, 0x64);
    write_cmos_sensor(0x3317, 0x25);
    write_cmos_sensor(0x3318, 0x80);
    write_cmos_sensor(0x30dc, 0x00);
    
    write_cmos_sensor(0x3021, 0x18); //HS

    write_cmos_sensor(0x300f, 0xa6); //must set, from FAE
    //===preview setting end===

    //PCLK 36Mhz -> 18Mhz ISP_preview
    //litao --09.09.14----------------------------------------------------
    write_cmos_sensor(0x3011, 0x01);
    //if MT6235    //36Mhz out No ISP preview  
    //write_cmos_sensor(0x3011, 0x00); //CLK = XVCLK/(0+1)

    /* ==Camera Preview, MT6235 use 36MHz PCLK, 30fps== */
    /* ==Video Preview, MT6235 use 9MHz PCLK, 7.5fps== */

    /* after set exposure line, there should be delay for 2~4 frame time, then enable AEC */
    //mdelay(10);

    // turn on AEC/AGC
    rc = read_cmos_sensor_2(0x300A, &tmp16);
    if(rc < 0)
	return rc;
    printk("modelid=%x\n", tmp16);
    /*
    rc = read_cmos_sensor(0x300A, &tmp, 1);
    if(rc < 0)
	return rc;
    printk("300a=%x\n", tmp);
    */
    rc = read_cmos_sensor_2(0x3013, &tmp16);
    if(rc < 0)
	return rc;
    printk("3013=%x\n", tmp16);
    tmp = tmp16&0xff;
    /*
    rc = read_cmos_sensor(0x3013, &tmp, 1);
    if(rc < 0)
	return rc;
    */
    write_cmos_sensor(0x3013, tmp | 0x05);

    //enable Auto WB
    rc = read_cmos_sensor_2(0x3323, &tmp16);
    if(rc < 0)
	return rc;
    printk("3323=%x\n", tmp16);
    tmp = (tmp16&0xff00)>>8;
    /*
    rc = read_cmos_sensor(0x3324, &tmp, 1);
    if(rc < 0)
	return rc;
    */
    write_cmos_sensor(0x3324, tmp & ~0x40);
    
//set valid window
//set sensor pclk
//may set ISP mclk/pclk again
//set mirror, flip
//set dummy
//write shutter

    //if (sensor_config_data->frame_rate == 0x0) {   // vedio: MPEG4/MJPEG Encode Mode
    //if (sensor_config_data->isp_op_mode != ISP_PREVIEW_MODE) 
    {
        //4  <2> if preview of capture VIDEO

        /* video capture: 19.6 fps with 24M PCLK */

        /* config TG of ISP to match the setting of image sensor*/
        // MCLK = 24MHz
        // may be add code here --@
        // PCLK = 24MHz

        /* to fix VSYNC, to fix frame rate */
	write_cmos_sensor(0x302d, 0x00);
	write_cmos_sensor(0x302e, 0x00);

    }
    //else { //-----------------------------------------------------
	//sensor_config_data->frame_rate == 30
	//ISP_PREVIEW_MODE
	//4  <2> if preview of capture PICTURE
	
	/* preview: 30 fps with 36M PCLK */
	
	
	// set clock
	//SET_TG_PIXEL_CLK_DIVIDER(3);    // PCLK = 24MHz
	//SET_CMOS_DATA_LATCH(1);
	
    //} //-----------------------------------------------------

    //preview_pclk_division = ((DRV_Reg32(ISP_TG_PHASE_COUNTER_REG)&0xF0)>>4)+1;

    //4 <3> set mirror and flip

    //4 <6> set dummy

    //4 <7> set shutter
    return rc;
}	/* OV2650_Preview */

static void OV2655_AE_Transfer(void)
{

//From preview to capture, because of size change, you need re-calculate exposure/gain,in order to achieve right exposure time.
//For OV2650

/*1.*/
//Read out preview exposure/gain values
	
        unsigned char reg3002, reg3003,reg3013, reg302d,reg302e;
      
	unsigned short PrvExp, PrvGain,extra_exp_lines,TgtExp,TgtGain;
                   
        unsigned int  TgtExpGain;     //ADJUST 200902


	reg3003 = read_cmos_sensor_1(0x3003);	// AEC[b7~b0]
	reg3002 = read_cmos_sensor_1(0x3002);	// AEC[b15~b8]
	PrvExp= reg3002 & 0xFF;
        PrvExp= (reg3003 & 0xFF) | (PrvExp << 8);


	reg302e = read_cmos_sensor_1(0x302E);	// EXVTS[b7~b0]
	reg302d = read_cmos_sensor_1(0x302D);	// EXVTS[b15~b8]
       extra_exp_lines=reg302d & 0xFF;
       extra_exp_lines = (reg302e & 0xFF) | (extra_exp_lines << 8);

        PrvExp=PrvExp+ extra_exp_lines ;

	PrvGain= read_cmos_sensor_1(0x3000);

       write_cmos_sensor(0x3014, (read_cmos_sensor_1(0x3014)&0x08)|0x84);
      // write_cmos_sensor(0x3014, 0x84);

	reg3013 = read_cmos_sensor_1(0x3013);//AGC

//turn off AE/AG
	reg3013 = reg3013 & 0xFA;
	write_cmos_sensor(0x3013, reg3013); 
	//*3.TgtMaxExp =1236;PrvMaxExp = 617;PrvFps = 15, TgtFps = 7.5;*/
//TgtMaxExp: UXGA maximum exposure lines
//PrvMaxExp: SVGA maximum exposure lines
//PrvFps:	Preview frame rate
//TgtFps:	Capture frame rate
	TgtExp = (1236* 15 * PrvExp) / (617 * 15);      
          



	TgtGain = (PrvGain & ((unsigned short)0x0F)) | (unsigned short)0x10;
	if (PrvGain & 0x10)
	{
	  TgtGain = TgtGain << 1;
	}
	if (PrvGain & 0x20)
	{
	  TgtGain = TgtGain << 1;
	}
	if (PrvGain & 0x40)
	{
	  TgtGain = TgtGain << 1;
	}
	if (PrvGain & 0x80)
	{
	  TgtGain = TgtGain << 1;
	}

/*4.*/
	if(read_cmos_sensor_1(0x3014) & 0x08){
        	TgtExpGain = (int)TgtExp * TgtGain*15/100 ;  //night mode
		printk("night mode=================15=================\n");
	}
	else{
        	TgtExpGain = (int)TgtExp * TgtGain*65/100 ;  //normal mode
		printk("normal mode==================================\n");
	}

	 if (TgtExpGain < 95 * 16)     // Exposure < 1/100/120
    {
       if(TgtExpGain<16){//exposure line smaller than 2 lines and gain smaller than 0x08 
			TgtExpGain = TgtExpGain*4;     
			TgtExp = 1;
			TgtGain = (int)((TgtExpGain*2 + 1)/TgtExp/2/4);
		        }
		else
		{  
                   TgtExp = TgtExpGain /16;//1180
                  
		TgtGain = (int)((TgtExpGain*2 + 1)/TgtExp/2);//16

		}
    }
    else 
    {
        if (TgtExpGain > 1236 * 16)     // Exposure > Capture_Maximum_Shutter 19776
        {
           
            TgtExp = 1236;
            TgtGain = (TgtExpGain*2 + 1)/TgtExp/2;
            if (TgtGain > 16*6) 
            {
                // gain reach maximum, insert extra line
                TgtExp = TgtExpGain/96;
                
                // Exposure = n/100/120
                  TgtExp = TgtExp/95*95;
				
                TgtGain = ((TgtExpGain *4)/ TgtExp+3)/4;


            }                                    
        }
        else  // 1/100 < Exposure < Capture_Maximum_Shutter, Exposure = n/100/120
        {
            TgtExp = TgtExpGain/16/95*95;
            
            TgtGain = (TgtExpGain*2 +1) / TgtExp/2;
        }
    }
    
   
    extra_exp_lines = (TgtExp > 1260)?(TgtExp - 1260):0; 
                
	
    TgtExp = TgtExp - extra_exp_lines;

   
            
 


	/*5. transfer to UXGA*/
        /*
		          write_cmos_sensor(0x30d9,0x95); //0816 Joe
	                 write_cmos_sensor(0x3016,0xd2);     
    
                        write_cmos_sensor(0x3023,0x0a); //array output vertical start, VS[7:0] = 10
                        write_cmos_sensor(0x3012,0x00);
                        write_cmos_sensor(0x302A,0x04); //VTS[15:8], V total size = 1236
                        write_cmos_sensor(0x302B,0xd4); //VTS[7:0]
                        write_cmos_sensor(0x306f,0x54); //<6:4>window,<3:0>targ_s
                        write_cmos_sensor(0x3024,0x06); //HW[15:8], H width = 1624
                        write_cmos_sensor(0x3025,0x58); //HW[7:0]
                        write_cmos_sensor(0x3026,0x04); //VH[15:8], V hight = 1212
                        write_cmos_sensor(0x3027,0xbc); //VH[7:0]
                        write_cmos_sensor(0x3302,0x00); //scale_en
                        write_cmos_sensor(0x3088,0x06); //ISP_XOUT[15:8]: x_size_in[11:0] = 800
                        write_cmos_sensor(0x3089,0x40); //ISP_XOUT[7:0]:
                        write_cmos_sensor(0x308a,0x04); //ISP_YOUT[15:8]: y_size_in[11:0] = 600
                        write_cmos_sensor(0x308b,0xb0); //ISP_YOUT[7:0]:
                        write_cmos_sensor(0x3100,0x00); //<3>bypass_win,<2>bypass_fmt,<1>rgb_sel,<0>snr_sel    
                        write_cmos_sensor(0x3319,0x6c); //for awb timing 0813 Joe
                        write_cmos_sensor(0x331d,0x6c); //for awb timing 0813 Joe    
                        write_cmos_sensor(0x3316,0x64);
                        write_cmos_sensor(0x3317,0x4b);
                        write_cmos_sensor(0x3318,0x00);
                        write_cmos_sensor(0x30dc,0x00);
                        write_cmos_sensor(0x331a,0x64); //H
                        write_cmos_sensor(0x331b,0x4b); //V
                        write_cmos_sensor(0x331c,0x00); //VH
                        write_cmos_sensor(0x3362,0x80);    
                        write_cmos_sensor(0x3021,0x0c); //HS    
                        write_cmos_sensor(0x300f,0xa6); //PCLK 72Mhz, because no OV verio mode
          */

   /*6   write back register*/
//write calculate exposure values to register

	reg3003 = TgtExp & 0xFF;
	write_cmos_sensor(0x3003, reg3003);
	reg3002 = (TgtExp >> 8 )& 0xFF;
	write_cmos_sensor(0x3002, reg3002);

        reg302e = extra_exp_lines & 0xFF;
	write_cmos_sensor(0x302e, reg302e);
	reg302d = (extra_exp_lines >> 8 )& 0xFF;
	write_cmos_sensor(0x302d, reg302d);
	
	PrvGain = 0;
	if( TgtGain > 31)
	{
	  PrvGain = PrvGain | 0x10;
	  TgtGain = TgtGain >> 1;
	}
	if( TgtGain > 31)
	{
	  PrvGain = PrvGain | 0x20;
	  TgtGain = TgtGain >> 1;
	}
	if( TgtGain > 31)
	{
	  PrvGain = PrvGain | 0x40;
	  TgtGain = TgtGain >> 1;
	}
	if( TgtGain > 31)
	{
	  PrvGain = PrvGain | 0x80;
	  TgtGain = TgtGain >> 1;
	}
	if ( TgtGain > 16)
	{
	  PrvGain = PrvGain | ((TgtGain - 16 ) & 0x0F);
	}
//write calculated gain value to register
	write_cmos_sensor(0x3000, PrvGain);
        write_cmos_sensor(0x3002, reg3002);
        write_cmos_sensor(0x3003, reg3003);
        write_cmos_sensor(0x302D, reg302d);
        write_cmos_sensor(0x302E, reg302e);

//write back extra_exp_lines to 0X302D  0X302E
	/*7.*/
//wait 3 frames and capture the four frame
	  mdelay(190);
}

void OV2650_set_dummy(unsigned short pixels, unsigned short lines)
{
	unsigned char temp_reg1, temp_reg2;
	unsigned short temp_reg;

	write_cmos_sensor(0x302c,(pixels&0xFF)); //EXHTS[7:0]

	// read out and + line
	temp_reg1 = read_cmos_sensor_1(0x302B);	// VTS[b7~b0]
	temp_reg2 = read_cmos_sensor_1(0x302A);	// VTS[b15~b8]
	temp_reg = (temp_reg1 & 0xFF) | (temp_reg2 << 8);

	temp_reg += lines;

	write_cmos_sensor(0x302B,(temp_reg&0xFF));         //VTS[7:0]
	write_cmos_sensor(0x302A,((temp_reg&0xFF00)>>8));  //VTS[15:8]
}	/* OV2650_set_dummy */

static long ov2655_snapshot(void)
{
    long rc = 0;
    unsigned char tmp;
    unsigned short tmp16;

CDBG("ov2655 snapshot:\n");
    /* If not WEBCAM mode */
    /* webcam: frame_rate = 0xF0 ? */   
    //if ( (sensor_config_data->frame_rate != 0xF0)  ) {
    //if ( (sensor_config_data->isp_op_mode != ISP_PREVIEW_MODE)  ) {
       /* Webcam mode, AE/AWB must be on */

    // Disable Auto WB 0813 Joe
    //temp_AWB_reg = read_cmos_sensor(0x3324);
    //write_cmos_sensor(0x3324, temp_AWB_reg|0x40);
    
    
    // turn off AEC/AGC
    //tmp = tmp16&0xff;
    //tmp = read_cmos_sensor(0x3013, &tmp, 1);
    write_cmos_sensor(0x3013, read_cmos_sensor_1(0x3013) & (~0x05) );        
///////////////////////////////////////////////
/*
    read_cmos_sensor(0x3002, &exposure_line_h, 1);
    read_cmos_sensor(0x3003, &exposure_line_l, 1);
    read_cmos_sensor(0x302D, &extra_exposure_line_h, 1);
    read_cmos_sensor(0x302E, &extra_exposure_line_l, 1);
*/

    //shutter = OV2650_read_shutter();
    //g_iCap_Shutter = shutter;

    // 1600x1200	
    write_cmos_sensor(0x30d9,0x95); //0816 Joe
    write_cmos_sensor(0x3016,0xd2);     
    
    write_cmos_sensor(0x3023,0x0a); //array output vertical start, VS[7:0] = 10
    write_cmos_sensor(0x3012,0x00);
    write_cmos_sensor(0x302A,0x04); //VTS[15:8], V total size = 1236
    write_cmos_sensor(0x302B,0xd4); //VTS[7:0]
    write_cmos_sensor(0x306f,0x54); //<6:4>window,<3:0>targ_s
    write_cmos_sensor(0x3024,0x06); //HW[15:8], H width = 1624
    write_cmos_sensor(0x3025,0x58); //HW[7:0]
    write_cmos_sensor(0x3026,0x04); //VH[15:8], V hight = 1212
    write_cmos_sensor(0x3027,0xbc); //VH[7:0]
    write_cmos_sensor(0x3302,0x00); //scale_en
    write_cmos_sensor(0x3088,0x06); //ISP_XOUT[15:8]: x_size_in[11:0] = 800
    write_cmos_sensor(0x3089,0x40); //ISP_XOUT[7:0]:
    write_cmos_sensor(0x308a,0x04); //ISP_YOUT[15:8]: y_size_in[11:0] = 600
    write_cmos_sensor(0x308b,0xb0); //ISP_YOUT[7:0]:
    write_cmos_sensor(0x3100,0x00); //<3>bypass_win,<2>bypass_fmt,<1>rgb_sel,<0>snr_sel    
    write_cmos_sensor(0x3319,0x6c); //for awb timing 0813 Joe
    write_cmos_sensor(0x331d,0x6c); //for awb timing 0813 Joe    
    write_cmos_sensor(0x3316,0x64);
    write_cmos_sensor(0x3317,0x4b);
    write_cmos_sensor(0x3318,0x00);
    write_cmos_sensor(0x30dc,0x00);
    write_cmos_sensor(0x331a,0x64); //H
    write_cmos_sensor(0x331b,0x4b); //V
    write_cmos_sensor(0x331c,0x00); //VH
    write_cmos_sensor(0x3362,0x80);    
    write_cmos_sensor(0x3021,0x0c); //HS    
    write_cmos_sensor(0x300f,0xa6); //PCLK 72Mhz, because no OV verio mode

    /*para setting*/
    /*
    write_cmos_sensor(0x300e,0x38);
    write_cmos_sensor(0x3311,0x00);
    write_cmos_sensor(0x332c,0x00);
    write_cmos_sensor(0x3071,0x00);
    write_cmos_sensor(0x3370,0x5d);
    write_cmos_sensor(0x331c,0x0d);
    write_cmos_sensor(0x3073,0x00);
    write_cmos_sensor(0x3372,0x4e);
    write_cmos_sensor(0x331d,0x0f);
    */
    /*para setting*/
			//36Mhz
    write_cmos_sensor(0x3011, 0x01); //CLK = XVCLK/(0+1)
			
			/* ==UXGA capture: 7.5 fps with 36M PCLK== */


    OV2655_AE_Transfer();

    // set dummy
    //OV2650_set_dummy(dummy_pixels, dummy_lines);

    // set shutter
    //OV2650_write_shutter(shutter);

    // AEC/AGC/AWB will be enable in preview and param_wb function

    /* total delay 4 frame for AE stable */
    mdelay(100);
    printk("+++++++++++++++++++++++++++++++++++++++++++++++++++++\n3390 is %x, 3391 is %x, 339a is %x\n", read_cmos_sensor_1(0x3390), read_cmos_sensor_1(0x3391), read_cmos_sensor_1(0x339a));
    printk("out of snapshot\n");
   // reg_dump();
    return rc;

}	/* OV2650_Capture */

/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
static long ov2655_set_sensor_mode(int mode)
{
	long rc = 0;

CDBG("ov2655 set sensor mode %d:\n", mode);
	switch (mode) {
		case SENSOR_PREVIEW_MODE:
			rc = ov2655_priview();
			break;
		case SENSOR_SNAPSHOT_MODE:
			rc = ov2655_snapshot();
			break;
		default:
			return -EFAULT;
	}

	return rc;
}

static long ov2655_set_antibanding(int value)
{
	long rc = 0;
	unsigned char banding;

CDBG("ov2655 set antibanding %d:\n", value);
	//rc = read_cmos_sensor(0x3014, &banding, 1); 
	banding = read_cmos_sensor_1(0x3014);
	if (rc < 0) return rc;
	switch(value){
		case CAMERA_ANTIBANDING_OFF:
			printk("%s:%d:%s-CAMERA_ANTIBANDING_OFF\n", __FILE__, __LINE__, __func__);
			write_cmos_sensor(0x3014, banding & 0xf7);    /* disable banding */
			break;
		case CAMERA_ANTIBANDING_50HZ:
			printk("%s:%d:%s-CAMERA_ANTIBANDING_50hz\n", __FILE__, __LINE__, __func__);
			write_cmos_sensor(0x300e, 0x34);
			write_cmos_sensor( 0x3014, banding|0x81 );    /* enable banding and 50 Hz */
			write_cmos_sensor(0x301c, 0x06);
			write_cmos_sensor(0x3070, 0x5d);
			break;
		case CAMERA_ANTIBANDING_60HZ:
			printk("%s:%d:%s-CAMERA_ANTIBANDING_60hz\n", __FILE__, __LINE__, __func__);
			write_cmos_sensor(0x300e, 0x34);
			write_cmos_sensor( 0x3014, (banding & ~0x80)|0x01 );    /* enable banding and 60 Hz */
			write_cmos_sensor(0x301d, 0x07);
			write_cmos_sensor(0x3072, 0x4d);
			break;
		case CAMERA_ANTIBANDING_AUTO:
			printk("%s:%d:%s-CAMERA_ANTIBANDING_auto\n", __FILE__, __LINE__, __func__);
			write_cmos_sensor(0x3014, banding | 0x41);    /* enable auto banding */
			break;
		default:
			return -EFAULT;
	}
	return rc;
}

static long ov2655_set_wb(int value)
{
	long rc = 0;
	unsigned char tmp_reg;

CDBG("ov2655 set wb %d:\n", value);
	//rc = read_cmos_sensor(0x3306, &tmp_reg, 1);
	tmp_reg = read_cmos_sensor_1(0x3306);
	switch(value){
		case CAMERA_WB_AUTO:
			printk("%s:%d:%s-whitebalance auto\n", __FILE__, __LINE__, __func__);
			write_cmos_sensor(0x3306, tmp_reg&~0x2);   // select Auto WB
			break;
		case CAMERA_WB_INCANDESCENT:
			printk("%s:%d:%s-CAMERA_WB_INCANDESCENT\n", __FILE__, __LINE__, __func__);
                        write_cmos_sensor(0x3306, tmp_reg|0x2);  // Disable AWB
                        write_cmos_sensor(0x3337, 0x5e);
                        write_cmos_sensor(0x3338, 0x40);
                        write_cmos_sensor(0x3339, 0x58);
			break;
		case CAMERA_WB_FLUORESCENT:
			printk("%s:%d:%s-CAMERA_WB_FLUORESCENT\n", __FILE__, __LINE__, __func__);
			break;
		case CAMERA_WB_DAYLIGHT:
			printk("%s:%d:%s-CAMERA_WB_DAYLIGHT\n", __FILE__, __LINE__, __func__);
			write_cmos_sensor(0x3306, tmp_reg|0x2);  // Disable AWB
			write_cmos_sensor(0x3337, 0x5e);
			write_cmos_sensor(0x3338, 0x40);
			write_cmos_sensor(0x3339, 0x46);
			break;
		case CAMERA_WB_CLOUDY_DAYLIGHT:
			printk("%s:%d:%s-CAMERA_WB_CLOUDY_DAYLIGHT\n", __FILE__, __LINE__, __func__);
			write_cmos_sensor(0x3306, tmp_reg|0x2);  // select manual WB
			write_cmos_sensor(0x3337, 0x68); //manual R G B
			write_cmos_sensor(0x3338, 0x40);
			write_cmos_sensor(0x3339, 0x4e);
			break;
		default:
			return -EFAULT;
	}
	return rc;
}

static long ov2655_set_brightness(int value)
{
	long rc = 0;
	printk("%s:%d:%s-brightness is %d\n", __FILE__, __LINE__, __func__, value);
	switch(value){
		case 8:
		case 7:
			printk("%s:%d:%s-whitebalance auto\n", __FILE__, __LINE__, __func__);
			rc = write_cmos_sensor(0x3390, 0x41); //bit[3] sign of brightness
			if (rc < 0) return rc;
			rc = write_cmos_sensor(0x339a, 0x30);
			if (rc < 0) return rc;
			break;
		case 6:
			printk("%s:%d:%s-CAMERA_WB_INCANDESCENT\n", __FILE__, __LINE__, __func__);
			rc = write_cmos_sensor(0x3390, 0x41);
			if (rc < 0) return rc;
			rc = write_cmos_sensor(0x339a, 0x20);
			if (rc < 0) return rc;
			break;
		case 5:
			printk("%s:%d:%s-CAMERA_WB_INCANDESCENT\n", __FILE__, __LINE__, __func__);
			rc = write_cmos_sensor(0x3390, 0x41);
			if (rc < 0) return rc;
			rc = write_cmos_sensor(0x339a, 0x10);
			if (rc < 0) return rc;
			break;
		case 4:
			printk("%s:%d:%s-CAMERA_WB_INCANDESCENT\n", __FILE__, __LINE__, __func__);
			rc = write_cmos_sensor(0x3390, 0x41);
			if (rc < 0) return rc;
			rc = write_cmos_sensor(0x339a, 0x00);
			if (rc < 0) return rc;
			break;
		case 3:
			printk("%s:%d:%s-CAMERA_WB_INCANDESCENT\n", __FILE__, __LINE__, __func__);
			rc = write_cmos_sensor(0x3390, 0x49);
			if (rc < 0) return rc;
			rc = write_cmos_sensor(0x339a, 0x10);
			if (rc < 0) return rc;
			break;
		case 2:
			printk("%s:%d:%s-CAMERA_WB_INCANDESCENT\n", __FILE__, __LINE__, __func__);
			rc = write_cmos_sensor(0x3390, 0x49);
			if (rc < 0) return rc;
			rc = write_cmos_sensor(0x339a, 0x20);
			if (rc < 0) return rc;
			break;
		case 1:
		case 0:
			printk("%s:%d:%s-CAMERA_WB_INCANDESCENT\n", __FILE__, __LINE__, __func__);
			rc = write_cmos_sensor(0x3390, 0x49);
			if (rc < 0) return rc;
			rc = write_cmos_sensor(0x339a, 0x30);
			if (rc < 0) return rc;
			break;
		default:
			return -EFAULT;
	}
        printk("--------------------------------+++++++++++++++++++++\n3390 is %x, 3391 is %x, 339a is %x\n", read_cmos_sensor_1(0x3390), read_cmos_sensor_1(0x3391), read_cmos_sensor_1(0x339a));
	return rc;
}

static long ov2655_set_nightmode(int mode)
{
	long rc = 0;
	unsigned short tmp16;
	unsigned char night;

	printk("%s:%d:%s-set nightmode %d\n", __FILE__, __LINE__, __func__, mode);
	night = read_cmos_sensor_1(0x3014); //bit[3], 0: disable, 1:enable
	printk("night %x\n", night);
	if(mode == CAMERA_NIGHTSHOT_MODE_ON){
		printk("%s:%d:%s-CAMERA_NIGHTSHOT_MODE_ON\n", __FILE__, __LINE__, __func__);
                //write_cmos_sensor(0x302D, extra_exposure_line_h);
                //write_cmos_sensor(0x302E, extra_exposure_line_l);

                rc = write_cmos_sensor(0x3015, 0x44); //0x04 -> 0x23 Joe 0816
		if (rc < 0) return rc;
                rc = write_cmos_sensor(0x3014, night | 0x08); //Enable night mode 
		if (rc < 0) return rc;

	}else{
		printk("%s:%d:%s-CAMERA_NIGHTSHOT_MODE_OFF\n", __FILE__, __LINE__, __func__);
                rc = write_cmos_sensor(0x3015, 0x42); //0x51
		if (rc < 0) return rc;
		rc = write_cmos_sensor(0x3014, night & 0xf7); //Disable night mode
		if (rc < 0) return rc;
                //rc = write_cmos_sensor(0x302d, 0x00);
                //rc = write_cmos_sensor(0x302e, 0x00);

	}
	return rc;
}

static long ov2655_set_effect(
	int mode,
	int8_t effect
)
{
	long rc = 0;
	unsigned char tmp8;

	printk("%s:%d:%s-set effect %d\n", __FILE__, __LINE__, __func__, effect);
	switch (effect) {
		case CAMERA_EFFECT_OFF:
			rc = write_cmos_sensor(0x3391, 0x06);
			if (rc < 0) return rc;
			tmp8 = read_cmos_sensor_1(0x3390) & 0x08;
			rc = write_cmos_sensor(0x3390, 0x41 | tmp8);
			if (rc < 0) return rc;
			break;

		case CAMERA_EFFECT_SEPIA:
			rc = write_cmos_sensor(0x3391, 0x1e);
			if (rc < 0) return rc;
			rc = write_cmos_sensor(0x3396, 0x40);
			if (rc < 0) return rc;
			rc = write_cmos_sensor(0x3397, 0xa6);
			if (rc < 0) return rc;
			break;

		case CAMERA_EFFECT_NEGATIVE:
                        rc = write_cmos_sensor(0x3391, 0x44);
			if (rc < 0) return rc;
			break;

                case CAMERA_EFFECT_MONO: //B&W
                        rc = write_cmos_sensor(0x3391, 0x26);
			if (rc < 0) return rc;
		        break;

		default:
			rc = -1;
	}
	return rc;
}

/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
static int ov2655_sensor_init_probe(const struct msm_camera_sensor_info *data)
{
	uint16_t model_id = 0;
	int rc = 0;

	CDBG("init entry \n");
        rc = gpio_request(data->sensor_pwd, "ov2655");

        if (!rc) {
                rc = gpio_direction_output(data->sensor_pwd, 0);
        }

        gpio_free(data->sensor_pwd);

	rc = ov2655_reset(data);
	if (rc < 0) {
		CDBG("reset failed!\n");
		goto init_probe_fail;
	}

	mdelay(5);
    //<4>set polarity (to BB)
    //<5>set data format to ISP (to BB)
    //<6>use 48MHz source (to BB)
    //<7>clk / 2 = 24MHz (to BB)
    //<8>set ISP driving current (to BB)

	mdelay(5);


    //<9>software reset sensor and wait (to sensor)
        write_cmos_sensor(REG_OV2655_MODEL_ID, 0x80);
	mdelay(5);
	mdelay(5);

	/* Read the Model ID of the sensor */
	rc = read_cmos_sensor_2(0x300A, &model_id);

	if (rc < 0)
		goto init_probe_fail;

	CDBG("ov2655 model_id = 0x%x\n", model_id);

	/* Check if it matches it with the value in Datasheet */
	if (model_id != OV2655_MODEL_ID) {
		rc = -EFAULT;
		goto init_probe_fail;
	}

	rc = ov2655_reg_init();
	if (rc < 0)
		goto init_probe_fail;

	return rc;

init_probe_fail:
	return rc;
}
//=============================//

static int proc_ov2655_dread(char *page, char **start,
                             off_t off, int count,
                             int *eof, void *data)
{
        int len, rc;
	unsigned short tmp16;
CDB
	/************************/
	/************************/
	rc = read_cmos_sensor_2(ov2655_proc_dt.i2c_addr, &tmp16);
	if(rc < 0){
        	len = sprintf(page, "double 0x%x@ov2655_i2c read error\n",
		                      ov2655_proc_dt.i2c_addr);
	}
	else{
        	len = sprintf(page, "double 0x%x@ov2655_i2c = %4x\n",
		                      ov2655_proc_dt.i2c_addr, tmp16);
	}
CDB

        return len;
}


static int proc_ov2655_sread(char *page, char **start,
                            off_t off, int count,
                            int *eof, void *data)
{
        int len, rc;
	unsigned char tmp8;

	//rc = read_cmos_sensor(ov2655_proc_dt.i2c_addr, &tmp8, 1);
	tmp8 = read_cmos_sensor_1(ov2655_proc_dt.i2c_addr);
	rc =0;
	if(rc < 0){
        	len = sprintf(page, "single byte 0x%x@ov2655_i2c read error\n",
		                      ov2655_proc_dt.i2c_addr);
	}
	else{
        	len = sprintf(page, "single byte 0x%x@ov2655_i2c = %4x\n",
		                      ov2655_proc_dt.i2c_addr, tmp8);
	}

        return len;
#if 0
        int len, rc;
	unsigned char tmp[16];
	tmp[0] = 8;
	rc = 0;
CDB

	//rc = read_cmos_sensor(ov2655_proc_dt.i2c_addr, tmp, 3);
	/*
	if(rc < 0){
        	len = sprintf(page, "single byte 0x%x@ov2655_i2c read error\n",
		                      ov2655_proc_dt.i2c_addr);
	}
	else{
        	len = sprintf(page, "single byte 0x%x@ov2655_i2c = %4x\n",
		                      ov2655_proc_dt.i2c_addr, tmp[0]);
	}
	*/
       	len = sprintf(page, "%x", tmp[0]);
CDB

        return len;
#endif
}

static int proc_ov2655_swrite(struct file *file,
                            const char *buffer,
                            unsigned long count,
                            void *data)
{
        int len;

//CDB
	memset(ov2655_proc_buffer, 0, OV2655_PROC_LEN);
        if(count > OV2655_PROC_LEN)
                len = OV2655_PROC_LEN;
        else
                len = count;

	//printk("count = %d\n", len);
        if(copy_from_user(ov2655_proc_buffer, buffer, len))
                return -EFAULT;
	//printk("%s\n", ov2655_proc_buffer);
	sscanf(ov2655_proc_buffer, "%x", &ov2655_proc_dt.i2c_data);
	//printk("received %x\n", ov2655_proc_dt.i2c_addr);
	write_cmos_sensor(ov2655_proc_dt.i2c_addr, ov2655_proc_dt.i2c_data);
//CDB
        return len;

}

static int proc_ov2655_addr_read(char *page, char **start,
                            off_t off, int count,
                            int *eof, void *data)
{
        int len;

CDB
       	len = sprintf(page, "addr is 0x%x\n", ov2655_proc_dt.i2c_addr);
CDB

        return len;
}

static int proc_ov2655_addr_write(struct file *file,
                            const char *buffer,
                            unsigned long count,
                            void *data)
{
        int len;

CDB
	memset(ov2655_proc_buffer, 0, OV2655_PROC_LEN);
        if(count > OV2655_PROC_LEN)
                len = OV2655_PROC_LEN;
        else
                len = count;

	//printk("count = %d\n", len);
        if(copy_from_user(ov2655_proc_buffer, buffer, len))
                return -EFAULT;
	//printk("%s\n", ov2655_proc_buffer);
	sscanf(ov2655_proc_buffer, "%x", &ov2655_proc_dt.i2c_addr);
	//printk("received %x\n", ov2655_proc_dt.i2c_addr);
CDB
        return len;

}

int ov2655_add_proc(void)
{
	int rc;
/* add for proc*/
	/* create directory */
        ov2655_dir = proc_mkdir(OV2655_PROC_NAME, NULL);
	if(ov2655_dir == NULL) {
	          rc = -ENOMEM;
	          goto init_fail;
	}

	//ov2655_dir->owner = THIS_MODULE;

        /* create readfile */
	s_file = create_proc_entry(SINGLE_OP_NAME,
	                                 0644, ov2655_dir);
	if(s_file == NULL) {
	          rc  = -ENOMEM;
	          goto no_s;
	}
        s_file->read_proc = proc_ov2655_sread;
        s_file->write_proc = proc_ov2655_swrite;
	//s_file->owner = THIS_MODULE;

	dr_file = create_proc_read_entry(DOUBLE_READ_NAME,
	                                 0444, ov2655_dir,
	                                 proc_ov2655_dread,
	                                 NULL);
	if(dr_file == NULL) {
	          rc  = -ENOMEM;
	          goto no_dr;
	}

	//dr_file->owner = THIS_MODULE;

        /* create write file */
        w_file = create_proc_entry(I2C_ADDR_NAME, 0644, ov2655_dir);
        if(w_file == NULL) {
                rc = -ENOMEM;
                goto no_wr;
        }

        w_file->read_proc = proc_ov2655_addr_read;
        w_file->write_proc = proc_ov2655_addr_write;
        //w_file->owner = THIS_MODULE;

        /* OK, out debug message */
        printk(KERN_INFO "%s %s %s initialised\n",
		              SINGLE_OP_NAME, DOUBLE_READ_NAME, I2C_ADDR_NAME);

/*litao add end*/
	return 0;
/*litao add for proc*/
no_wr:
        remove_proc_entry(DOUBLE_READ_NAME, ov2655_dir);
no_dr:
        remove_proc_entry(SINGLE_OP_NAME, ov2655_dir);
no_s:
        remove_proc_entry(OV2655_PROC_NAME, NULL);
/*litao add end*/
init_fail:
	return 1;


}

int ov2655_del_proc(void)
{
        remove_proc_entry(I2C_ADDR_NAME, ov2655_dir);
        remove_proc_entry(DOUBLE_READ_NAME, ov2655_dir);
        remove_proc_entry(SINGLE_OP_NAME, ov2655_dir);
        remove_proc_entry(OV2655_PROC_NAME, NULL);
        printk(KERN_INFO "%s %s %s removed\n",
	              SINGLE_OP_NAME, DOUBLE_READ_NAME, I2C_ADDR_NAME);
	return 0;
}


//=============================================//
void ov2655_open_power(void)
{
       int rc;

       struct vreg * vreg;
       printk("vreg_set_camera power\n");

       vreg =vreg_get(0, "gp3");
       rc = vreg_set_level(vreg, 2800);
       if (rc){
               printk("error vreg gp3\n");
       }
       rc = vreg_enable(vreg);

       vreg =vreg_get(0, "gp2");
       rc = vreg_set_level(vreg, 1800);
       if (rc){
               printk("error vreg gp2\n");
       }
       rc = vreg_enable(vreg);
       mdelay(100);
}


int ov2655_sensor_init(const struct msm_camera_sensor_info *data)
{
	int rc = 0;

	CDBG("ov2655_sensor_init\n");
	ov2655_open_power();
	ov2655_ctrl = kzalloc(sizeof(struct ov2655_ctrl_t), GFP_KERNEL);
	if (!ov2655_ctrl) {
		CDBG("ov2655_init failed!\n");
		rc = -ENOMEM;
		goto init_done;
	}

	if (data)
		ov2655_ctrl->sensordata = data;

	/* Input MCLK = 24MHz */
	msm_camio_clk_rate_set(24000000);
	mdelay(5);

	msm_camio_camif_pad_reg_reset();

  	rc = ov2655_sensor_init_probe(data);
	if (rc < 0) {
		CDBG("ov2655_sensor_init failed!\n");
		goto init_fail;
	}
	ov2655_add_proc();

init_done:
	return rc;

init_fail:
	kfree(ov2655_ctrl);
	return rc;
}

static int ov2655_init_client(struct i2c_client *client)
{
	/* Initialize the MSM_CAMI2C Chip */
	CDBG("ov2655_init_client\n");
	init_waitqueue_head(&ov2655_wait_queue);
	return 0;
}

int ov2655_sensor_config(void __user *argp)
{
	struct sensor_cfg_data cfg_data;
	long   rc = 0;

	CDBG("ov2655_sensor_config\n");
	if (copy_from_user(
				&cfg_data,
				(void *)argp,
				sizeof(struct sensor_cfg_data)))
		return -EFAULT;

	/* down(&ov2655_sem); */

	CDBG("ov2655_ioctl, cfgtype = %d, mode = %d\n",
		cfg_data.cfgtype, cfg_data.mode);

	switch (cfg_data.cfgtype) {
	case CFG_SET_MODE:
		rc = ov2655_set_sensor_mode(
					cfg_data.mode);
		break;

	case CFG_SET_EFFECT:
		rc = ov2655_set_effect(
					cfg_data.mode,
					cfg_data.cfg.effect);
		break;
#if 0 //fix compile error rbxue 20100913
	case CFG_SET_NIGHTMODE:
		rc = ov2655_set_nightmode(cfg_data.cfg.value);
		break;
#endif

	case CFG_SET_BRIGHTNESS:
		rc = ov2655_set_brightness(cfg_data.cfg.value);
		break;

	case CFG_SET_WB:
		rc = ov2655_set_wb(cfg_data.cfg.value);
		break;

	case CFG_SET_ANTIBANDING:
		rc = ov2655_set_antibanding(cfg_data.cfg.value);
		break;

	default:
		rc = -EFAULT;
		break;
	}

	/* up(&ov2655_sem); */

	return rc;
}

void ov2655_close_power(void)
{
       int rc;
       struct vreg * vreg;
       printk("close power of camera\n");

       vreg =vreg_get(0, "gp3");
       //rc = vreg_set_level(vreg, 2850);
       rc = vreg_disable(vreg);
       if (rc){
               printk("error vreg gp3\n");
       }

       vreg =vreg_get(0, "gp2");
       rc = vreg_disable(vreg);
       //rc = vreg_set_level(vreg, 4500);
       if (rc){
               printk("error vreg gp2\n");
       }
}

int ov2655_sensor_release(void)
{
	int rc = 0;
 
	CDBG("ov2655_sensor_release\n");
	/* down(&ov2655_sem); */

	kfree(ov2655_ctrl);
	/* up(&ov2655_sem); */

	ov2655_del_proc();
	ov2655_close_power();
	return rc;
}

static int __exit ov2655_i2c_remove(struct i2c_client *client)
{
	struct ov2655_work_t *sensorw = i2c_get_clientdata(client);

	CDBG("ov2655_i2c_remove\n");
	free_irq(client->irq, sensorw);
	ov2655_client = NULL;
	ov2655_sensorw = NULL;
	kfree(sensorw);
	return 0;
}

static int ov2655_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	int rc = 0;

	CDBG("ov2655_i2c_probe\n");
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		rc = -ENOTSUPP;
		goto probe_failure;
	}

	ov2655_sensorw =
		kzalloc(sizeof(struct ov2655_work), GFP_KERNEL);

	if (!ov2655_sensorw) {
		rc = -ENOMEM;
		goto probe_failure;
	}

	i2c_set_clientdata(client, ov2655_sensorw);
	ov2655_init_client(client);
	ov2655_client = client;

	CDBG("ov2655_probe succeeded!\n");

	return 0;

probe_failure:
	//kfree(ov2655_sensorw);
	ov2655_sensorw = NULL;
	CDBG("ov2655_probe failed!\n");
	return rc;
}

static const struct i2c_device_id ov2655_i2c_id[] = {
	{ "ov2655", 0},
	{ },
};

static struct i2c_driver ov2655_i2c_driver = {
	.id_table = ov2655_i2c_id,
	.probe  = ov2655_i2c_probe,
	.remove = __exit_p(ov2655_i2c_remove),
	.driver = {
		.name = "ov2655",
	},
};

static int ov2655_sensor_probe(const struct msm_camera_sensor_info *info,
				struct msm_sensor_ctrl *s)
{
	int rc;

	CDBG("ov2655_sensor_probe\n");
        ov2655_open_power();
	rc = i2c_add_driver(&ov2655_i2c_driver);

	if (rc < 0 || ov2655_client == NULL) {
		rc = -ENOTSUPP;
		goto probe_done;
	}

	/* Input MCLK = 24MHz */
	msm_camio_clk_rate_set(24000000);
	mdelay(5);

	rc = ov2655_sensor_init_probe(info);
	if (rc < 0)
		goto probe_done;

	s->s_init = ov2655_sensor_init;
	s->s_release = ov2655_sensor_release;
	s->s_config  = ov2655_sensor_config;

probe_done:
	CDBG("%s %s:%d\n", __FILE__, __func__, __LINE__);
	return rc;
}

static int __ov2655_probe(struct platform_device *pdev)
{
	return msm_camera_drv_start(pdev, ov2655_sensor_probe);
}

static struct platform_driver msm_camera_driver = {
	.probe = __ov2655_probe,
	.driver = {
		.name = "msm_camera_ov2655",
		.owner = THIS_MODULE,
	},
};

static int __init ov2655_init(void)
{
	int rc;

	CDBG("ov2655 init\n");
	rc = platform_driver_register(&msm_camera_driver);
	ov2655_close_power();
	return rc;

}

module_init(ov2655_init);
