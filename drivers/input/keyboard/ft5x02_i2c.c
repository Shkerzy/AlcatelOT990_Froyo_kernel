/* drivers/input/keyboard/ft5x02_i2c.c
 *
 *
 */

#include <linux/module.h>
#include <linux/delay.h>
#include <linux/earlysuspend.h>
#include <linux/hrtimer.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <mach/gpio.h>
#include "ft5x02_i2c.h"
#define FT5X02_I2C_NAME "ft5x02-ts"

#include <linux/proc_fs.h>
#include "ft5x02_i2c.h"

#include <mach/vreg.h>


#undef DEBUG_FT5X02

#define REQUIRED_VERSION 0x13

#define CDBG(fmt, arg...) printk(fmt, ##arg)

static struct workqueue_struct *ft5x02_wq;

static int down_position_y = 0;

struct ft5x02_ts_data {
	uint16_t addr;
	struct i2c_client *client;
	struct input_dev *input_dev;
	int use_irq;
	struct hrtimer timer;
	struct work_struct  work;
	uint16_t max[2];
	uint32_t flags;
	int (*power)(int on);
	struct early_suspend early_suspend;
};

#ifdef CONFIG_HAS_EARLYSUSPEND
static void ft5x02_ts_early_suspend(struct early_suspend *h);
static void ft5x02_ts_late_resume(struct early_suspend *h);
#endif

struct i2c_client *ft5x02_client = NULL;

static char write_reg(unsigned char addr, char v);
char read_reg(unsigned char addr);
static int32_t read_info(void * buf);
static void reg_dump(void );

static ST_TOUCH_POINT touch_point[5];
static ST_TOUCH_INFO ft_ts_data =
{
	.pst_point_info	= touch_point,
};

#ifdef DEBUG_FT5X02
static void reg_dump(void)
{
	char i;
	
	for (i =0;i<0x40;i++){
		if((i & 0xf) == 0)
			printk("\n%02x:", i);
		printk(" %02x", read_reg(i));
	}
	printk("\n");
}

static struct proc_dir_entry *ft5x02proc;
static int proc_ft5x02_read(char *page, char **start,
                             off_t off, int count,
                             int *eof, void *data)
{
        int len, ret, i;
	char comm_buf[128];
/*
	ret = read_reg(0x3d, comm_buf, 0x4);
	*/

 
	ret = read_info(comm_buf);
	printk("read info bytes, return %d\n", ret);
	if (ret < 0) {
		printk(KERN_ERR "i2c_smbus_read_byte_data failed\n");
        	len = sprintf(page, "read failed\n");
		return len;
	}
	printk("ft5x02_ts_probe: 0x3b: %2x %2x %2x %2x\n",
	       comm_buf[0], comm_buf[1], comm_buf[2], comm_buf[3]);
        	len = sprintf(page, "read reg ok\n");


	for (i = 0;i<26;i++)
		printk(" %2x", comm_buf[i]);

	ret = write_reg(0x08, 0x12);
	printk("wrte return %d\n", ret);

	reg_dump();
	//while(1)
		printk("get gpio 20 %d\n", gpio_get_value(20));

	return len;
}

static void add_ft5x02_proc(void)
{
	ft5x02proc = create_proc_read_entry("ft5x02proc",
	                                 0444, NULL,
	                                 proc_ft5x02_read,
	                                 NULL);
	if(ft5x02proc== NULL) {
		printk("creat sleep time proc failed\n");
	        return;
	}

//	ft5x02proc->owner = THIS_MODULE;
}
#endif


/********************************/

int32_t ft5x02_i2c_txdata(unsigned short saddr,
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

	if (i2c_transfer(ft5x02_client->adapter, msg, 1) < 0) {
		CDBG("ft5x02_i2c_txdata faild\n");
		return -EIO;
	}

	return 0;
}

int ft5x02_i2c_rxdata(int txlen, unsigned char *rxdata, int length)
{
	struct i2c_msg msgs[] = {
	{
		.addr   = ft5x02_client->addr,
		.flags = 0,
		.len   = txlen,
		.buf   = rxdata,
	},
	{
		.addr   = ft5x02_client->addr,
		.flags = I2C_M_RD,
		.len   = length,
		.buf   = rxdata,
	},
	};

	if (i2c_transfer(ft5x02_client->adapter, msgs, 2) < 0) {
		CDBG("ft5x02_i2c_rxdata failed!\n");
		return -EIO;
	}
	return 0;
}

static char write_reg(unsigned char addr, char v)
{
	char tmp[4], ecc = 0;
	int32_t rc = 0;

	memset(tmp, 0, 2);
	tmp[0] = 0xfc;
	ecc ^= 0xfc;
	tmp[1] = addr;
	ecc ^= addr;
	tmp[2] = v;
	ecc ^= v;
	tmp[3] = ecc;

	rc = ft5x02_i2c_txdata(ft5x02_client->addr, tmp, 4);
	if (rc < 0){
		CDBG("ft5x02 write reg failed!\n");
		return rc;
	}
	return 0;
}

char read_reg(unsigned char addr)
{
	char tmp[2];
	int32_t rc = 0;

	memset(tmp, 0, 2);
	tmp[0] = 0xfc;
	tmp[1] = addr +0x40;
	rc = ft5x02_i2c_rxdata(2, tmp, 2);
	if (rc < 0){
		CDBG("ft5x02_i2c_read failed!\n");
		return rc;
	}
	return tmp[0];
}

static int32_t read_info(void * buf)
{
	char tmp[32];
	int32_t rc = 0, len;

	memset(tmp, 0, 32);
	tmp[0] = 0xf9;
	len = 26;
	rc = ft5x02_i2c_rxdata(1, tmp, len);
	if (rc < 0){
		CDBG("ft5x02_i2c_read failed!\n");
		return rc;
	}
	memcpy(buf, tmp, len);
	return len;
}

static int ft5x02_init_panel(struct ft5x02_ts_data *ts)
{
	int ret;

	ret = write_reg(0x3c, 0x01);
	if(ret < 0)
		return -1;
	ret = write_reg(0x3a, 0x01);
	if(ret < 0)
		return -1;
	ret = write_reg(0x06, 0x01);
	if(ret < 0)
		return -1;
	ret = write_reg(0x07, 200);
	if(ret < 0)
		return -1;
	ret = write_reg(0x08, 0x10);//ret = write_reg(0x08, 0x12);
	if(ret < 0)
		return -1;
	ret = write_reg(0x09, 0x28);//ret = write_reg(0x09, 0x30);
	if(ret < 0)
		return -1;
	

	return 0;
}

/*********************************************************/
/****** Function porting from Ft5x02 Sample code *********/

FTS_BYTE bt_parser_fts(FTS_BYTE* pbt_buf, FTS_BYTE bt_len, ST_TOUCH_INFO* pst_touch_info)
{
	FTS_WORD low_byte	= 0;
	FTS_WORD high_byte	= 0;
	FTS_BYTE point_num 	= 0;
	FTS_BYTE i 			= 0;
	FTS_BYTE ecc 		= 0;
	char j = 0;
	/*check the pointer*/
	POINTER_CHECK(pbt_buf);
	POINTER_CHECK(pst_touch_info);

	/*check the length of the protocol data*/
	if(bt_len < PROTOCOL_LEN)
	{
		return CTPM_ERR_PARAMETER;
	}
	pst_touch_info->bt_tp_num= 0;
	
	/*check packet head: 0xAAAA.*/
	if(pbt_buf[1]!= 0xaa || pbt_buf[0] != 0xaa)
	{
		return CTPM_ERR_PROTOCOL;
	}
	/*check data length*/
	if((pbt_buf[2] & 0x3f) != PROTOCOL_LEN)
	{
		return CTPM_ERR_PROTOCOL;
	}			
	/*check points number.*/
	point_num = pbt_buf[3] & 0x0f;
	if(point_num > CTPM_26BYTES_POINTS_MAX)
	{
		return CTPM_ERR_PROTOCOL;
	}			
	/*remove the touch point information into pst_touch_info.*/
	for(i = 0; i < point_num; i++)
	{	
			
		high_byte = pbt_buf[5+4*i];
		high_byte <<= 8;
		low_byte = pbt_buf[5+4*i+1];
		pst_touch_info->pst_point_info[i].w_tp_x = (high_byte |low_byte) & 0x0fff;
		
		high_byte = pbt_buf[5+4*i+2];
		high_byte <<= 8;
		low_byte = pbt_buf[5+4*i+3];
		pst_touch_info->pst_point_info[i].w_tp_y = (high_byte |low_byte) & 0x0fff;
		
		pst_touch_info->bt_tp_num++;
	}
	j = pbt_buf[5];
		j >>= 6;
		if (j == 0){
	down_position_y = pst_touch_info->pst_point_info[0].w_tp_y;
	}
	/*check ecc*/
	ecc = 0;
	for (i=0; i<bt_len-1; i++)
	{ 
		ecc ^= pbt_buf[i];	
	}
	if(ecc != pbt_buf[bt_len-1]) 
	{
		/*ecc error*/
		return CTPM_ERR_ECC;
	}
	
	return CTPM_NOERROR;
}

/*
[function]: 
	get all the information of one touch.
[parameters]:
	pst_touch_info[out]	:stored all the information of one touch;	
[return]:
	CTPM_NOERROR		:success;
	CTPM_ERR_I2C		:io fail;
	CTPM_ERR_PROTOCOL	:protocol data error;
	CTPM_ERR_ECC		:ecc error.
*/
FTS_BYTE fts_ctpm_get_touch_info(ST_TOUCH_INFO* pst_touch_info)
{
	FTS_BYTE* p_data_buf= FTS_NULL;
	FTS_BYTE read_cmd[2]= {0};
	FTS_BYTE cmd_len 	= 0;
	FTS_BYTE data_buf[26]= {0}; 
	
	POINTER_CHECK(pst_touch_info);
	POINTER_CHECK(pst_touch_info->pst_point_info);

	p_data_buf = data_buf;

	read_cmd[0] 	= I2C_STARTTCH_READ;
	cmd_len 		= 1;
	read_info(p_data_buf);
	//for (i = 0;i<26;i++)
	//	printk(" %02x", p_data_buf[i]);
	/*parse the data read out from ctpm and put the touch point information into pst_touch_info*/
	return bt_parser_fts(p_data_buf, PROTOCOL_LEN, pst_touch_info);
}

/****** Function porting from Ft5x02 Sample code end *****/
/*********************************************************/

static void ft5x02_ts_work_func(struct work_struct *work)
{
	struct ft5x02_ts_data *ts = container_of(work, struct ft5x02_ts_data, work);
	fts_ctpm_get_touch_info(&ft_ts_data);
	static int i = 0, j = 0, k = 0;
				
	if (ft_ts_data.bt_tp_num == 0) { //release touch
		if((i == 1)||(j == 1)||(k == 1)||(i == 2)||(j == 2)||(k == 2))
			{i = 0;
			 j = 0;
			 k = 0;
			}
		input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0);
		input_report_key(ts->input_dev, KEY_MENU, 0);
		input_report_key(ts->input_dev, KEY_SEARCH, 0);
		input_report_key(ts->input_dev, KEY_BACK, 0);
	}
	if (ft_ts_data.bt_tp_num >= 1) {

			if(touch_point[0].w_tp_y<480){
			input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR,255);
			input_report_abs(ts->input_dev, ABS_MT_POSITION_X,touch_point[0].w_tp_x);
		        input_report_abs(ts->input_dev, ABS_MT_POSITION_Y,touch_point[0].w_tp_y);
			input_mt_sync(ts->input_dev);
			//}else if(touch_point[0].w_tp_y>500){
			}else if(touch_point[0].w_tp_y>500 && down_position_y > 479){
					

			 	if(touch_point[0].w_tp_x>9 && touch_point[0].w_tp_x<81){
					i++;
					if(i == 3)
					{
					input_report_key(ts->input_dev, KEY_MENU, 1);
					i =0;
					}		
				}else if(touch_point[0].w_tp_x>124&&touch_point[0].w_tp_x<196){
					j++;
					if(j == 3)
					{						
					input_report_key(ts->input_dev, KEY_SEARCH, 1);
						j =0;
						}				
					}else if(touch_point[0].w_tp_x>239&&touch_point[0].w_tp_x<311){
					k++;
					if(k == 3)
					{					
					input_report_key(ts->input_dev, KEY_BACK, 1);
						k =0;
					}				
				}

			}
	
#if defined( DEBUG_FT5X02)
			printk("P1:%d %d %d %d %d  ", touch_point[0].w_tp_x,
							touch_point[0].w_tp_y,	
							touch_point[0].bt_tp_id,	
							touch_point[0].bt_tp_property,	
							touch_point[0].w_tp_strenth);
			printk("\n");
#endif

	}
	
	if (ft_ts_data.bt_tp_num >= 2) {
		
		//	input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR,255);
			
			if(touch_point[1].w_tp_y<480){
			input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR,255);                  
      			input_report_abs(ts->input_dev, ABS_MT_POSITION_X,touch_point[1].w_tp_x);
                        input_report_abs(ts->input_dev, ABS_MT_POSITION_Y,touch_point[1].w_tp_y);
			input_mt_sync(ts->input_dev);
			}
			else if(touch_point[1].w_tp_y>500){
			 	if(touch_point[1].w_tp_x>9&&touch_point[1].w_tp_x<81){
					input_report_key(ts->input_dev, KEY_MENU, 1);
				}else if(touch_point[1].w_tp_x>124&&touch_point[1].w_tp_x<196){
					input_report_key(ts->input_dev, KEY_SEARCH, 1);
				}else if(touch_point[1].w_tp_x>239&&touch_point[1].w_tp_x<311){
					input_report_key(ts->input_dev, KEY_BACK, 1);
				}

			}
		
		
#if defined( DEBUG_FT5X02)
	        printk("P2:%d %d %d %d %d", touch_point[1].w_tp_x,
							touch_point[1].w_tp_y,	
							touch_point[1].bt_tp_id,	
							touch_point[1].bt_tp_property,	
							touch_point[1].w_tp_strenth);
		printk("\n");
#endif
	}
	

	input_sync(ts->input_dev);
	
}

static enum hrtimer_restart ft5x02_ts_timer_func(struct hrtimer *timer)
{
	struct ft5x02_ts_data *ts = container_of(timer, struct ft5x02_ts_data, timer);

	queue_work(ft5x02_wq, &ts->work);

	hrtimer_start(&ts->timer, ktime_set(0, 12500000), HRTIMER_MODE_REL);
	return HRTIMER_NORESTART;
}

static irqreturn_t ft5x02_ts_irq_handler(int irq, void *dev_id)
{
	struct ft5x02_ts_data *ts = dev_id;

	queue_work(ft5x02_wq, &ts->work);
	return IRQ_HANDLED;
}

int ft5x02_power(int on_off)
{
	struct vreg * vreg_tp;
	int rc;
	vreg_tp= vreg_get(NULL, "gp6"); 
	
	if (IS_ERR(vreg_tp)) {
		printk(KERN_ERR "%s: vreg get failed (%ld)\n",
		       		__func__, PTR_ERR(vreg_tp));
		return PTR_ERR(vreg_tp);
	}
	if(on_off){
			rc = vreg_set_level(vreg_tp, 3000);
			if (rc) {
				printk(KERN_ERR "%s: vreg set level failed (%d)\n",
			       			__func__, rc);
				return -EIO;
			}
			rc = vreg_enable(vreg_tp);
			if (rc) {
				printk(KERN_ERR "%s: vreg enable failed (%d)\n",
			       			__func__, rc);
				return -EIO;
			}
		}else{
		        rc = vreg_set_level(vreg_tp, 0);
			if (rc) {
				printk(KERN_ERR "%s: vreg set level failed (%d)\n",
			       			__func__, rc);
				return -EIO;
			}
			
			rc=vreg_disable(vreg_tp);
			if (rc) {
				printk(KERN_ERR "%s: vreg disable failed (%d)\n",
			       			__func__, rc);
				return -EIO;
			}
			
	}
	msleep(80);
	return 0;
}

static int ft5x02_ts_probe(
	struct i2c_client *client, const struct i2c_device_id *id)
{
	struct ft5x02_ts_data *ts;
	int ret = 0;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		printk(KERN_ERR "ft5x02_ts_probe: need I2C_FUNC_I2C\n");
		ret = -ENODEV;
		goto err_check_functionality_failed;
	}

	ts = kzalloc(sizeof(*ts), GFP_KERNEL);
	if (ts == NULL) {
		ret = -ENOMEM;
		goto err_alloc_data_failed;
	}
	INIT_WORK(&ts->work, ft5x02_ts_work_func);
	client->irq=MSM_GPIO_TO_INT(FT5X02_INT);
	ts->client = client;
	ft5x02_client = client;
	i2c_set_clientdata(client, ts);
	ts->power = ft5x02_power;
	if (ts->power) {
		ret = ts->power(1);
		if (ret < 0) {
			printk(KERN_ERR "ft5x02_ts_probe power on failed\n");
			goto err_power_failed;
		}
	}

	/* read i2c data from chip */
	ret = read_reg(0x3d);
	printk("ft5x02 id %02x\n", ret);
//	if (ret != 0x79) {
//		printk(KERN_ERR "ft5x02 probe failed\n");
//		goto err_detect_failed;
//	}
//	printk("found ft5x02 touch screen!!!\n");
	

     /****************upgrade tp******************/ 
        ret = read_reg(0x3b);
	printk("focaltec's version is %x\n",ret);
	
	if(ret <= 0)
	   {printk(KERN_ERR "ft5x02 read 0x3b failed\n");
                goto err_detect_failed;     
	    }

	if((ret != REQUIRED_VERSION)&&(ret != 0x07)&&(ret >= 0)){
		printk("upgrade.........\n");
		ret = fts_ctpm_fw_upgrade_with_i_file();         //upgrade firmware
		if (ret != ERR_OK){
			printk(KERN_ERR "ft5x02 upgrade failed\n");
                  goto err_upgrade_failed;
		}
	}
      /***************upgrade tp end**************/  	

	/**	notice: request 300kHZ i2c speed when upgrade  **/

	/*
	printk(KERN_INFO "ft5x02_ts_probe: 0xe0: %x %x %x %x %x %x %x %x\n",
	       buf1[0], buf1[1], buf1[2], buf1[3],
	       buf1[4], buf1[5], buf1[6], buf1[7]);
	       */

	ret = ft5x02_init_panel(ts); /* will also switch back to page 0x04 */
	if (ret < 0) {
		printk(KERN_ERR "ft5x02_init_panel failed\n");
		goto err_detect_failed;
	}

	ts->input_dev = input_allocate_device();
	if (ts->input_dev == NULL) {
		ret = -ENOMEM;
		printk(KERN_ERR "ft5x02_ts_probe: Failed to allocate input device\n");
		goto err_input_dev_alloc_failed;
	}
	ts->input_dev->name = "ft5x02-touchscreen";
	set_bit(EV_SYN, ts->input_dev->evbit);
	set_bit(EV_KEY, ts->input_dev->evbit);
	set_bit(BTN_TOUCH, ts->input_dev->keybit);
	set_bit(BTN_2, ts->input_dev->keybit);
	set_bit(KEY_MENU, ts->input_dev->keybit);
	set_bit(KEY_SEARCH, ts->input_dev->keybit);
	set_bit(KEY_BACK, ts->input_dev->keybit);
	set_bit(EV_ABS, ts->input_dev->evbit);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_X,0, 320, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_Y,0, 480, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_TOUCH_MAJOR,0, 255,0, 0);
	input_set_abs_params(ts->input_dev, ABS_HAT0X, 0, 320, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_HAT0Y, 0, 480+96, 0, 0);

	/* ts->input_dev->name = ts->keypad_info->name; */
	ret = input_register_device(ts->input_dev);
	if (ret) {
		printk(KERN_ERR "ft5x02_ts_probe: Unable to register %s input device\n", ts->input_dev->name);
		goto err_input_register_device_failed;
	}
	ret = request_irq(MSM_GPIO_TO_INT(19), ft5x02_ts_irq_handler, IRQF_TRIGGER_RISING, client->name, ts);
	if (ret == 0)
		ts->use_irq = 1;
	else
		dev_err(&client->dev, "request_irq failed\n");
	if (!ts->use_irq) {
		hrtimer_init(&ts->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
		ts->timer.function = ft5x02_ts_timer_func;
		hrtimer_start(&ts->timer, ktime_set(1, 0), HRTIMER_MODE_REL);
	}
#ifdef CONFIG_HAS_EARLYSUSPEND
	ts->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	ts->early_suspend.suspend = ft5x02_ts_early_suspend;
	ts->early_suspend.resume = ft5x02_ts_late_resume;
	register_early_suspend(&ts->early_suspend);
#endif

	printk(KERN_INFO "ft5x02_ts_probe: Start touchscreen %s in %s mode\n", ts->input_dev->name, ts->use_irq ? "interrupt" : "polling");
#ifdef DEBUG_FT5X02
	reg_dump();
	add_ft5x02_proc();
#endif

	return 0;
err_input_register_device_failed:
	input_free_device(ts->input_dev);

err_input_dev_alloc_failed:
err_detect_failed:
err_power_failed:
err_upgrade_failed:
	kfree(ts);
err_alloc_data_failed:
err_check_functionality_failed:
	return ret;
}

static int ft5x02_ts_remove(struct i2c_client *client)
{
	struct ft5x02_ts_data *ts = i2c_get_clientdata(client);
	unregister_early_suspend(&ts->early_suspend);
	if (ts->use_irq)
		free_irq(client->irq, ts);
	else
		hrtimer_cancel(&ts->timer);
	input_unregister_device(ts->input_dev);
	kfree(ts);
	return 0;
}

static int ft5x02_ts_suspend(struct i2c_client *client, pm_message_t mesg)
{
	int ret;
	printk("=========Enter %s\n",__func__);
	struct ft5x02_ts_data *ts = i2c_get_clientdata(client);

	disable_irq(client->irq);
	ret = cancel_work_sync(&ts->work);
	if (ret && ts->use_irq) /* if work was pending disable-count is now 2 */
		enable_irq(client->irq);
	cancel_work_sync(&ts->work);
	ret=write_reg(0x3a,Hib_Mode);//set tp power state :Hibernate mode
	if(ret<0)
		printk(KERN_ERR " set tp power state failed\n");

	if (ts->power) {
		ret = ts->power(0);
		if (ret < 0)
			printk(KERN_ERR "ft5x02_ts_resume power off failed\n");
	}	
	return 0;
}

static int ft5x02_ts_resume(struct i2c_client *client)
{
	int ret;

	printk("=========Enter %s\n",__func__);
	
	struct ft5x02_ts_data *ts = i2c_get_clientdata(client);

	if (ts->power) {
		ret = ts->power(1);
		if (ret < 0)
			printk(KERN_ERR "ft5x02_ts_resume power on failed\n");
	}
	gpio_set_value(FT5x02_WK,1);
	gpio_set_value(FT5x02_WK,0);
	udelay(500);
	gpio_set_value(FT5x02_WK,1);
	
	ret = write_reg(0x3a, Mtr_Mode);//set tp power state:Monitor mode



	
	if(ret < 0)
		return -1;

	if (ts->use_irq)
		enable_irq(client->irq);

	return 0;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void ft5x02_ts_early_suspend(struct early_suspend *h)
{
	struct ft5x02_ts_data *ts;
	ts = container_of(h, struct ft5x02_ts_data, early_suspend);
	ft5x02_ts_suspend(ts->client, PMSG_SUSPEND);
}

static void ft5x02_ts_late_resume(struct early_suspend *h)
{
	struct ft5x02_ts_data *ts;
	ts = container_of(h, struct ft5x02_ts_data, early_suspend);
	ft5x02_ts_resume(ts->client);
}
#endif

static const struct i2c_device_id ft5x02_ts_id[] = {
	{FT5X02_I2C_NAME, 0 },
	{ }
};

static struct i2c_driver ft5x02_ts_driver = {
	.probe		= ft5x02_ts_probe,
	.remove		= ft5x02_ts_remove,
#ifndef CONFIG_HAS_EARLYSUSPEND
	.suspend	= ft5x02_ts_suspend,
	.resume		= ft5x02_ts_resume,
#endif
	.id_table	= ft5x02_ts_id,
	.driver = {
		.name	= FT5X02_I2C_NAME,
	},
};

static int __devinit ft5x02_ts_init(void)
{
	int ret;

	ret = gpio_tlmm_config(GPIO_CFG(FT5X02_INT, 0, GPIO_INPUT,
				GPIO_NO_PULL, GPIO_4MA), GPIO_ENABLE);
        if (ret) {
                printk(KERN_ERR "%s: gpio_tlmm_config=%d\n",
                                 __func__, ret);
        }

	ret = gpio_tlmm_config(GPIO_CFG(FT5x02_WK, 0, GPIO_OUTPUT,
				GPIO_NO_PULL, GPIO_4MA), GPIO_ENABLE);
        if (ret) {
                printk(KERN_ERR "%s: gpio_tlmm_config=%d\n",
                                 __func__, ret);
        }

	gpio_set_value(FT5x02_WK,1);
		
	ft5x02_wq = create_singlethread_workqueue("ft5x02_wq");
	if (!ft5x02_wq)
		return -ENOMEM;
	ret = i2c_add_driver(&ft5x02_ts_driver);
	return ret;
}

static void __exit ft5x02_ts_exit(void)
{

	i2c_del_driver(&ft5x02_ts_driver);
	if (ft5x02_wq)
		destroy_workqueue(ft5x02_wq);
}

module_init(ft5x02_ts_init);
module_exit(ft5x02_ts_exit);

MODULE_DESCRIPTION("Ft5x02 Touchscreen Driver");
MODULE_LICENSE("GPL");

