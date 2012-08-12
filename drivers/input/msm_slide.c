#include <linux/module.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/delay.h>
#include <linux/seq_file.h>
#include <asm/atomic.h>
#include <asm/ioctls.h>
#include <mach/board.h>
#include <linux/input.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h> //liuyu added
#include <mach/gpio.h> //liuyu added
#include <linux/proc_fs.h> //liuyu added
#include <mach/mpp.h> //liuyu added
#include <mach/pmic.h>

int SlideIRQ; //liuyu added
#define SLIDE_GPIO 94 //liuyu added

struct proc_dir_entry	*SlideProcEntry; //liuyu added
struct input_dev *input_dev;
struct work_struct SlideWork;
struct delayed_work	IrqStartWork; //delay a time then start the slide IRQ
short bSlideEnable;

static uint32_t slide_gpio_table[] = {
	GPIO_CFG(SLIDE_GPIO, 0, GPIO_INPUT, GPIO_NO_PULL, GPIO_2MA)
};


void SN3226SetBacklight(unsigned char level);

static void SlideWorkFunc(struct work_struct *work)
{
	int gpiovalue=gpio_get_value(SLIDE_GPIO);
	int i;
	
	if(gpiovalue==1)
	{
		input_report_switch(input_dev,0, 0);
		printk("/////////////////////////jishaku ga toozakatta.\n");
/*
		for(i=0;i<6;i++)
		{
			SN3226SetBacklight(i);
			mdelay(50);
		}
*/
		//pmic_vib_mot_set_volt(0); //turn off the Vibiator
	}
	else
	{
		input_report_switch(input_dev, 0, 1);
		printk("/////////////////////////jishaku ga chikazuita.\n");
/*
		for(i=5;i>-1;i--)
		{
			SN3226SetBacklight(i);
			mdelay(50);
		}
*/
		//pmic_vib_mot_set_volt(3000); //turn on the Vibiator
	}

	input_sync(input_dev);
	
}

static void IrqStartWorkFunc(struct work_struct *work)
{
	bSlideEnable=1;
}

static irqreturn_t SlideIrqProc(int irq, void *dev_id)
{
	if(bSlideEnable)
		schedule_work(&SlideWork);
	
	return IRQ_HANDLED;
}


static int SlideStatusReadProc(char *buf, char **start, off_t offset, int request, int *eof, void *data)
{
	char *out = buf;
	int len;

	if(gpio_get_value(SLIDE_GPIO)==1)
		sprintf(out, "1");
	else
		sprintf(out, "0");

	len = 1;

	return len;
}

static int __devinit slide_probe(struct platform_device *pdev)
{
	//create input device
	
	input_dev = input_allocate_device();
	input_dev->name = "msm_slide";
	input_dev->phys = "msm_slide/input3";
	input_dev->id.bustype = BUS_HOST;
	input_dev->dev.parent = &pdev->dev;

	input_dev->evbit[0] = BIT_MASK(EV_SW) | BIT_MASK(EV_KEY);
	input_dev->swbit[BIT_WORD(0)] = BIT_MASK(0);

	int res = input_register_device(input_dev);
	if(res!=0)
		printk("$$$$$$$$$$$$$$$$$$$$$$$$$$$$Slide input device register failed\n");

	//setup GPIO functions
	config_lcdc_gpio_table(slide_gpio_table, 1, 1);
	SlideIRQ=MSM_GPIO_TO_INT(SLIDE_GPIO);

	//init vibrator
	//pmic_vib_mot_set_volt(0);
	//pmic_vib_mot_set_mode(PM_VIB_MOT_MODE__MANUAL);
	//pmic_vib_mot_set_polarity(PM_VIB_MOT_POL__ACTIVE_HIGH);

	bSlideEnable=0; //disalbe the IRQ when init
	int result=request_irq(SlideIRQ, SlideIrqProc,
				IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
				 "Slide Detect IRQ", NULL);

	INIT_WORK(&SlideWork, SlideWorkFunc);
	INIT_DELAYED_WORK(&IrqStartWork, IrqStartWorkFunc); 

	//Enable the slide IRQ in 20 mSec
	schedule_delayed_work(&IrqStartWork,
				msecs_to_jiffies(1000*20));

	//SlideProcEntry=create_proc_entry("SlideStatus", S_IFREG|S_IRUGO, NULL);
	//if (SlideProcEntry)
	//	SlideProcEntry->read_proc = SlideStatusReadProc;
	//else
	//	printk("/////////////////////proc create failed\n");
	
	return 0;
}


static struct platform_driver slide_driver = {
	.probe		= slide_probe,
	.driver		= {
		.name = "msm_slide",
		.owner = THIS_MODULE,
	},
};

static int __init msm_slide_init(void)
{
	return platform_driver_register(&slide_driver);
}

module_init(msm_slide_init);

