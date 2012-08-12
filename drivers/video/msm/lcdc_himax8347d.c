/* Copyright (c) 2009, Code Aurora Forum. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Code Aurora Forum nor
 *       the names of its contributors may be used to endorse or promote
 *       products derived from this software without specific prior written
 *       permission.
 *
 * Alternatively, provided that this notice is retained in full, this software
 * may be relicensed by the recipient under the terms of the GNU General Public
 * License version 2 ("GPL") and only version 2, in which case the provisions of
 * the GPL apply INSTEAD OF those given above.  If the recipient relicenses the
 * software under the GPL, then the identification text in the MODULE_LICENSE
 * macro must be changed to reflect "GPLv2" instead of "Dual BSD/GPL".  Once a
 * recipient changes the license terms to the GPL, subsequent recipients shall
 * not relicense under alternate licensing terms, including the BSD or dual
 * BSD/GPL terms.  In addition, the following license statement immediately
 * below and between the words START and END shall also then apply when this
 * software is relicensed under the GPL:
 *
 * START
 *
 * This program is free software; you can redistribute it and/or modify it under
 * the terms of the GNU General Public License version 2 and only version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more
 * details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 *
 * END
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <linux/delay.h>
#include <mach/gpio.h>
#include "msm_fb.h"
#include <mach/mpp.h> //liuyu added
#include <linux/leds.h>
#include <mach/pmic.h>
#include <linux/workqueue.h>

static int lcdc_hx8347d_panel_off(struct platform_device *pdev);
void WriteDatHX8347D(unsigned int c); //liuyu added 
void WriteRegHX8347D (unsigned int c); //liuyu added
void WriteSPIGPIO (u8 startbyte, unsigned int val); //liuyu added
u8 ReadDatHX8347D(void); //liuyu added, only test using
void InitHimax8347D(void);
void InitHimax8347D_new(void);
void Entersleep(void);
void ExitSleep(void);
void Reset8347D(void);
void config_lcdc_gpio_table(uint32_t *table, int len, unsigned enable);


static int spi_cs;
static int spi_sclk;
static int spi_sdo;
static int spi_sdi;
static int spi_dac;

struct hx8347d_state_type{
	boolean disp_initialized;
	boolean display_on;
	boolean disp_powered_up;
};

static struct hx8347d_state_type hx8347d_state = { 0 };
static struct msm_panel_common_pdata *lcdc_hx8347d_pdata;

#define SPI_DELAY_TIME 0

#define GPIO_OUT_103 103
static uint32_t spi_sclk_input[] =
{
	GPIO_CFG(GPIO_OUT_103, 0, GPIO_INPUT, GPIO_NO_PULL, GPIO_2MA)
};

static uint32_t spi_sclk_output[] =
{
	GPIO_CFG(GPIO_OUT_103, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA)
};



//liuyu added
static uint32_t lcdc_gpio_table_sleep[] = {
	GPIO_CFG(132, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA),
	GPIO_CFG(131, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA),
	GPIO_CFG(103, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA),
	GPIO_CFG(102, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA),
	GPIO_CFG(88,  0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA),

	GPIO_CFG(90,  0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), //LCD Backlight
	GPIO_CFG(96,  0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA) //NRESET
};


u8 ReadDatHX8347D()
{
	u8 SDA;
	int temp,i;
	u8 retval=0;
	int tmp;
	gpio_set_value(spi_cs, 0); //put cs low

	temp=0x73; //01110011 RS=1,RW=1
    for(i=7;i>=0;i--)
    {
        SDA=((temp>>i)&0x1);
		gpio_set_value(spi_sdi, SDA);

		gpio_set_value(spi_sclk, 0); 
        //SCL=0;

        udelay(SPI_DELAY_TIME); //_nop_();
        gpio_set_value(spi_sclk, 1); 
        //SCL=1;
        

        udelay(SPI_DELAY_TIME);//_nop_();
     
    }

	//change spi_sdi to input
	config_lcdc_gpio_table(spi_sclk_input, 1, 1);
    
    for(i=7;i>=0;i--)
    {
        //SCL=0;
        gpio_set_value(spi_sclk, 0); 
        

        udelay(SPI_DELAY_TIME); //_nop_();        
        //SCL=1;
        gpio_set_value(spi_sclk, 1); 

        udelay(SPI_DELAY_TIME); //_nop_(); 

		tmp=gpio_get_value(spi_sdi);
		//printk("///////////Read sdi= %d\n", tmp);
		if(tmp==0)
			retval|=0;
		else
			retval|=(0x1 << i);
    }
 	//lcd_cs=1;
 	gpio_set_value(spi_cs, 1);

	//change spi_sdi back to output
	config_lcdc_gpio_table(spi_sclk_output, 1, 1);

	return retval;
}
	
void WriteDatHX8347D(unsigned int c)
{
	WriteSPIGPIO(0x72, //01110010 RS=1,RW=0
		c);
}
void WriteRegHX8347D (unsigned int c)
{
	WriteSPIGPIO(0x70, //01110000 RS=0,RW=0
		c);
}
void WriteSPIGPIO (u8 startbyte, unsigned int val)
{
	u8 SDA;//, SCL;
	int temp,i;  
	//lcd_cs=0;
	gpio_set_value(spi_cs, 0);

	temp=startbyte;
    for(i=7;i>=0;i--)
    {
        SDA=((temp>>i)&0x1);
		gpio_set_value(spi_sdi, SDA);

		gpio_set_value(spi_sclk, 0); 
        //SCL=0;

        udelay(SPI_DELAY_TIME); //_nop_();
        gpio_set_value(spi_sclk, 1); 
        //SCL=1;
        

        udelay(SPI_DELAY_TIME);//_nop_();
     
    }
    
    for(i=7;i>=0;i--)
    {
        SDA=((val>>i)&0x1);
		gpio_set_value(spi_sdi, SDA);
        //SCL=0;
        gpio_set_value(spi_sclk, 0); 
        

        udelay(SPI_DELAY_TIME); //_nop_();        
        //SCL=1;
        gpio_set_value(spi_sclk, 1); 

        udelay(SPI_DELAY_TIME); //_nop_(); 
    }
 	//lcd_cs=1;
 	gpio_set_value(spi_cs, 1);
}

#define DelayX1ms mdelay

void InitHimax8347D()
{
	  WriteRegHX8347D(0xea);WriteDatHX8347D(0x00);
      WriteRegHX8347D(0xeb);WriteDatHX8347D(0x20);
	  //following change is for cold capable mode
      WriteRegHX8347D(0xec);WriteDatHX8347D(0x0c);//WriteDatHX8347D(0xcc);
      WriteRegHX8347D(0xed);WriteDatHX8347D(0xc4);//WriteDatHX8347D(0xfc);
      WriteRegHX8347D(0xe8);WriteDatHX8347D(0x40);//WriteDatHX8347D(0xfe);
      WriteRegHX8347D(0xe9);WriteDatHX8347D(0x38);//WriteDatHX8347D(0x10);
      WriteRegHX8347D(0xf1);WriteDatHX8347D(0x01);
      WriteRegHX8347D(0xf2);WriteDatHX8347D(0x10);


      
      //gamma 2.2 setting
      WriteRegHX8347D(0x40);WriteDatHX8347D(0x00);
      WriteRegHX8347D(0x41);WriteDatHX8347D(0x00);
      WriteRegHX8347D(0x42);WriteDatHX8347D(0x01);
      WriteRegHX8347D(0x43);WriteDatHX8347D(0x13);
      WriteRegHX8347D(0x44);WriteDatHX8347D(0x10);
      WriteRegHX8347D(0x45);WriteDatHX8347D(0x26);
      WriteRegHX8347D(0x46);WriteDatHX8347D(0x08);
      WriteRegHX8347D(0x47);WriteDatHX8347D(0x51);
      WriteRegHX8347D(0x48);WriteDatHX8347D(0x02);
      WriteRegHX8347D(0x49);WriteDatHX8347D(0x12);
      WriteRegHX8347D(0x4a);WriteDatHX8347D(0x18);
      WriteRegHX8347D(0x4b);WriteDatHX8347D(0x19);
      WriteRegHX8347D(0x4c);WriteDatHX8347D(0x14);
      
      WriteRegHX8347D(0x50);WriteDatHX8347D(0x19);
      WriteRegHX8347D(0x51);WriteDatHX8347D(0x2f);
      WriteRegHX8347D(0x52);WriteDatHX8347D(0x2c);
      WriteRegHX8347D(0x53);WriteDatHX8347D(0x3e);
      WriteRegHX8347D(0x54);WriteDatHX8347D(0x3f);
      WriteRegHX8347D(0x55);WriteDatHX8347D(0x3f);
      WriteRegHX8347D(0x56);WriteDatHX8347D(0x2e);
      WriteRegHX8347D(0x57);WriteDatHX8347D(0x77);
      WriteRegHX8347D(0x58);WriteDatHX8347D(0x0b);
      WriteRegHX8347D(0x59);WriteDatHX8347D(0x06);
      WriteRegHX8347D(0x5a);WriteDatHX8347D(0x07);
      WriteRegHX8347D(0x5b);WriteDatHX8347D(0x0d);
      WriteRegHX8347D(0x5c);WriteDatHX8347D(0x1d);
      WriteRegHX8347D(0x5d);WriteDatHX8347D(0xcc);
      
      //Power voltage setting
   
      WriteRegHX8347D(0x1b);WriteDatHX8347D(0x1b);
      WriteRegHX8347D(0x1a);WriteDatHX8347D(0x01);


     WriteRegHX8347D(0x24);WriteDatHX8347D(0x2F);
     WriteRegHX8347D(0x25);WriteDatHX8347D(0x5a);



      // line/frame inversion
     //WriteRegHX8347D(0x2f);WriteDatHX8347D(0x11);  //frame inversion
      
      ////****VCOM offset**///
      WriteRegHX8347D(0x23);WriteDatHX8347D(0x92);//0x86
      
      //Power on Setting
      //WriteRegHX8347D(0x18);WriteDatHX8347D(0x36);

      WriteRegHX8347D(0x19);WriteDatHX8347D(0x01);
      WriteRegHX8347D(0x01);WriteDatHX8347D(0x00);
      WriteRegHX8347D(0x1f);WriteDatHX8347D(0x88);
      DelayX1ms(5);
      WriteRegHX8347D(0x1f);WriteDatHX8347D(0x80);
      DelayX1ms(5);
       WriteRegHX8347D(0x1f);WriteDatHX8347D(0x90);
      DelayX1ms(5);
      WriteRegHX8347D(0x1f);WriteDatHX8347D(0xd0);
      DelayX1ms(5);
      
      //262k/65k color selection
      WriteRegHX8347D(0x17);WriteDatHX8347D(0x60); //0x06
      
      //SET PANEL
      WriteRegHX8347D(0x36);WriteDatHX8347D(0x00);
      
      //Display ON Setting
      WriteRegHX8347D(0x28);WriteDatHX8347D(0x38);
      DelayX1ms(40);
      WriteRegHX8347D(0x28);WriteDatHX8347D(0x3c);
      
      //Set GRAM Area
      WriteRegHX8347D(0x02);WriteDatHX8347D(0x00);
      WriteRegHX8347D(0x03);WriteDatHX8347D(0x00);
      WriteRegHX8347D(0x04);WriteDatHX8347D(0x00);
      WriteRegHX8347D(0x05);WriteDatHX8347D(0xef);
      WriteRegHX8347D(0x06);WriteDatHX8347D(0x00);
      WriteRegHX8347D(0x07);WriteDatHX8347D(0x00);
      WriteRegHX8347D(0x08);WriteDatHX8347D(0x01);
      WriteRegHX8347D(0x09);WriteDatHX8347D(0x3f);
	  

      WriteRegHX8347D(0x31);WriteDatHX8347D(0x02);//RCM[1:0]='10'vs+hs+de mode 
      WriteRegHX8347D(0x32);WriteDatHX8347D(0x0b);//WriteDatHX8347D(0x07);
	  

     WriteRegHX8347D(0x22);
}

void Set_LCD_SPI_REG_8b(int x, int y)
{
	WriteRegHX8347D(x);
	WriteDatHX8347D(y);
}

void Entersleep()
{
	WriteRegHX8347D(0x28);WriteDatHX8347D(0xb8); //GON=\uff61\uff6f1\uff61\uff6f DTE=\uff61\uff6f1\uff61\uff6f D[1:0]=\uff61\uff6f10\uff61\uff6f
	DelayX1ms(40);
	WriteRegHX8347D(0x1F);WriteDatHX8347D(0x89); // GAS=1, VOMG=00, PON=0, DK=1, XDK=0, DVDH_TRI=0,STB=1
	DelayX1ms(40);
	WriteRegHX8347D(0x28);WriteDatHX8347D(0x04); //GON=\uff61\uff6f0\uff61\uff6f DTE=\uff61\uff6f0\uff61\uff6f D[1:0]=\uff61\uff6f01\uff61\uff6f
	DelayX1ms(40);
	WriteRegHX8347D(0x19);WriteDatHX8347D(0x00); //OSC_EN=\uff61\uff6f0\uff61\uff6f
	DelayX1ms(40);
}

void ExitSleep()
{
	WriteRegHX8347D(0x18);WriteDatHX8347D(0x36); //I/P_RADJ,N/P_RADJ, Normal mode 75Hz
	WriteRegHX8347D(0x19);WriteDatHX8347D(0x01); //OSC_EN='1', start Osc
	WriteRegHX8347D(0x1F);WriteDatHX8347D(0x88);// GAS=1, VOMG=00, PON=0, DK=1, XDK=0, DVDH_TRI=0,STB=0
	DelayX1ms(5);
	WriteRegHX8347D(0x1F);WriteDatHX8347D(0x80);// GAS=1, VOMG=00, PON=0, DK=0, XDK=0, DVDH_TRI=0,STB=0
	DelayX1ms(5);
	WriteRegHX8347D(0x1F);WriteDatHX8347D(0x90);// GAS=1, VOMG=00, PON=1, DK=0, XDK=0, DVDH_TRI=0,STB=0
	DelayX1ms(5);
	WriteRegHX8347D(0x1F);WriteDatHX8347D(0xD0);// GAS=1, VOMG=10, PON=1, DK=0, XDK=0, DDVDH_TRI=0,STB=0
	DelayX1ms(5);
	WriteRegHX8347D(0x28);WriteDatHX8347D(0x38); //GON=1, DTE=1, D=1000
	DelayX1ms(40);
	WriteRegHX8347D(0x28);WriteDatHX8347D(0x3F); //GON=1, DTE=1, D=1100
	DelayX1ms(5);
	WriteRegHX8347D(0x22);
}


static void spi_init(void)
{
	/* Setting the Default GPIO's */
	spi_sclk = *(lcdc_hx8347d_pdata->gpio_num);
	spi_cs   = *(lcdc_hx8347d_pdata->gpio_num + 1);
	spi_sdi  = *(lcdc_hx8347d_pdata->gpio_num + 2);
	spi_sdo  = *(lcdc_hx8347d_pdata->gpio_num + 3);

	/* Set the output so that we dont disturb the slave device */
	gpio_set_value(spi_sclk, 1); //liuyu changed
	gpio_set_value(spi_sdi, 1); //liuyu changed

	/* Set the Chip Select De-asserted */
	gpio_set_value(spi_cs, 1); //liuyu changed

}

static void hx8347d_disp_powerup(void)
{
	if (!hx8347d_state.disp_powered_up && !hx8347d_state.display_on) {
		/* Reset the hardware first */
		/* Include DAC power up implementation here */
	      hx8347d_state.disp_powered_up = TRUE;
	}
}

static uint32_t msm_mdp_lcdc_gpio_table[] = {
	GPIO_CFG(96, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA)
//	GPIO_CFG(86, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA)
};

//GPIO 90 for LCD_BK controlling, liuyu added 
#define GPIO_LCD_BK 90
static uint32_t sn3226_en_set_gpio_table[] = {
	GPIO_CFG(GPIO_LCD_BK, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA)
};

//SN3226 LCD backlight levels
uint8 sn3226_lcdbk_level[6] = {0, 12, 9, 6, 3, 1}; 

void Reset8347D()
{
	mdelay(100);
	mpp_config_digital_out(13,
					     MPP_CFG(MPP_DLOGIC_LVL_MSME,
					     MPP_DLOGIC_OUT_CTRL_LOW));
	
	mdelay(100);
	mpp_config_digital_out(13,
					     MPP_CFG(MPP_DLOGIC_LVL_MSME,
					     MPP_DLOGIC_OUT_CTRL_HIGH));



	/*
	mdelay(100);
	gpio_set_value(96, 0); //liuyu changed
	mdelay(100);
	gpio_set_value(96, 1); //liuyu changed
	*/

}

int msm_hsusb_phy_reset_liuyu();
struct delayed_work	LedWork;
#define LED_FLASH_MSEC 2000
static void LedWorkFunc(struct work_struct *work)
{
	printk("$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$ LedWorkFunc()\n");
	SN3226SetBacklight(0);
	mdelay(10);
	SN3226SetBacklight(1);
	mdelay(10);
	SN3226SetBacklight(2);
	mdelay(10);
	SN3226SetBacklight(3);
	mdelay(10);
	SN3226SetBacklight(4);
	mdelay(10);
	SN3226SetBacklight(5);

/*
	mdelay(5000);

	SN3226SetBacklight(5);
	mdelay(100);
	SN3226SetBacklight(4);
	mdelay(100);
	SN3226SetBacklight(3);
	mdelay(100);
	SN3226SetBacklight(2);
	mdelay(100);
	SN3226SetBacklight(1);
	mdelay(100);
	SN3226SetBacklight(0);

	schedule_delayed_work(&LedWork,
				msecs_to_jiffies(LED_FLASH_MSEC));
*/
/*
	//printk("////////////////////lcdc_gordon_panel_off\n");
	//Entersleep();
	//printk("///////////////////Turn on Keypad LED in LCDC\n");
	static short on_off=1;
	if(pmic_set_led_intensity(LED_KEYPAD, on_off?1:0)!=0)
		printk("//////////////LED control failed.\n");

	if(pmic_set_led_intensity(LED_LCD, on_off?0:1)!=0)
		printk("//////////////LED control failed.\n");

	on_off=!on_off;


	//init the Vibiator
	
	static int bFirst=1;
	if(bFirst)
	{
		pmic_vib_mot_set_volt(0);
		pmic_vib_mot_set_mode(PM_VIB_MOT_MODE__MANUAL);
		pmic_vib_mot_set_polarity(PM_VIB_MOT_POL__ACTIVE_HIGH);
		//pmic_vib_mot_set_volt(3000);
		bFirst=0;
	}
	



	static short LCDBacklightLevel=0;
	static short add_dic=1;

	if(LCDBacklightLevel>=5)
		add_dic=-1;

	if(LCDBacklightLevel<=0)
		add_dic=1;

	LCDBacklightLevel+=add_dic;
	SN3226SetBacklight(LCDBacklightLevel);

*/
}

//controlling SN3226_EN_SET up and down, liuyu added 
void SN3226_EN_SET(uint8 count)
{
	if(count>14 || count==0)
		return;
	
	int i;
	for(i=0;i<count;i++)
	{
		gpio_set_value(GPIO_LCD_BK, 0);
		udelay(50);
		gpio_set_value(GPIO_LCD_BK, 1);
		udelay(50);
		
	}
}

//turn on the Backlight, liuyu added
void SN3226SetBacklight(uint8 level)
{
	//printk("^^^^^^^^^^^^ SN3226SetBacklight(%d)\n", level);
	if(level >5)
		return;

	if(level==0)
		gpio_set_value(GPIO_LCD_BK, 0); //turn off the LEDs
	else
		SN3226_EN_SET(sn3226_lcdbk_level[level]);

	mdelay(1);
}
	
static void hx8347d_init(void)
{

	u8 id=0;

	int i;
	
	WriteRegHX8347D(0x0);
	id=ReadDatHX8347D();
	printk("////////////Read 8347D Panel ID is 0x%x\n", id);
	

	//printk("//////////////////Start to Init Himax 8347D\n");

	Reset8347D();

	InitHimax8347D();

	//mdelay(500);

	
	//SN3226SetBacklight(5);

	

	//Turn on the keypad LED in 10 seconds
	//INIT_DELAYED_WORK(&LedWork, LedWorkFunc);
	//schedule_delayed_work(&LedWork,
	//			msecs_to_jiffies(LED_FLASH_MSEC));

}

static void hx8347d_disp_on(void)
{
	if (hx8347d_state.disp_powered_up && !hx8347d_state.display_on)
	{
		hx8347d_init();
		//mdelay(2000);
		hx8347d_state.display_on = TRUE;
	}
}

static int lcdc_hx8347d_panel_on(struct platform_device *pdev)
{
	if (!hx8347d_state.disp_initialized) {
		/* Configure reset GPIO that drives DAC */
		lcdc_hx8347d_pdata->panel_config_gpio(1);
		//configure the GPIO 90 for SN3226 EN/SET liuyu added
		config_lcdc_gpio_table(sn3226_en_set_gpio_table, 1, 1);
		spi_dac = *(lcdc_hx8347d_pdata->gpio_num + 4);
		gpio_set_value(spi_dac, 0);
		udelay(15);
		gpio_set_value(spi_dac, 1);
		spi_init();	/* LCD needs SPI */
		hx8347d_disp_powerup();
		hx8347d_disp_on();
		hx8347d_state.disp_initialized = TRUE;
	}
	return 0;
}

static int lcdc_hx8347d_panel_off(struct platform_device *pdev)
{
//	msm_hsusb_phy_reset_liuyu();
	SN3226SetBacklight(0); //make sure to turn off backlight when LCD turned off
	//cancel_delayed_work_sync(&LedWork);
	
	Entersleep();
	//turn off GPIOs

	//mdelay(150);
	config_lcdc_gpio_table(lcdc_gpio_table_sleep,
		ARRAY_SIZE(lcdc_gpio_table_sleep), 1);
	//turn off MPP14
	mpp_config_digital_out(13,
					     MPP_CFG(MPP_DLOGIC_LVL_MSME,
					     MPP_DLOGIC_OUT_CTRL_LOW));
	
	if (hx8347d_state.disp_powered_up && hx8347d_state.display_on) {
		lcdc_hx8347d_pdata->panel_config_gpio(0);
		hx8347d_state.display_on = FALSE;
		hx8347d_state.disp_initialized = FALSE;
	}
	
	return 0;
}

static int __init hx8347d_probe(struct platform_device *pdev)
{
	//INIT_DELAYED_WORK(&LedWork, LedWorkFunc); //liuyu added
	
	if (pdev->id == 0) {
		lcdc_hx8347d_pdata = pdev->dev.platform_data;
		return 0;
	}
	msm_fb_add_device(pdev);

	SN3226SetBacklight(5); //liuyu added, turn on the backlight when system booting
	
	return 0;
}

static struct platform_driver this_driver = {
	.probe  = hx8347d_probe,
	.driver = {
		.name   = "lcdc_gordon_vga",
	},
};

static struct msm_fb_panel_data hx8347d_panel_data = {
	.on = lcdc_hx8347d_panel_on,
	.off = lcdc_hx8347d_panel_off,
};

static struct platform_device this_device = {
	.name   = "lcdc_gordon_vga",
	.id	= 1,
	.dev	= {
		.platform_data = &hx8347d_panel_data,
	}
};

static int __init lcdc_hx8347d_panel_init(void)
{

	int ret;
	struct msm_panel_info *pinfo;


	printk("///////////////////lcdc_hx8347d_panel_init\n");

#ifdef CONFIG_FB_MSM_TRY_MDDI_CATCH_LCDC_PRISM
	if (msm_fb_detect_client("lcdc_gordon_vga"))
		return 0;
#endif
	ret = platform_driver_register(&this_driver);
	if (ret)
		return ret;

	pinfo = &hx8347d_panel_data.panel_info;
	pinfo->xres = 240;
	pinfo->yres = 320;
	pinfo->type = LCDC_PANEL;
	pinfo->pdest = DISPLAY_1;
	pinfo->wait_cycle = 0;
	pinfo->bpp = 18;
	pinfo->fb_num = 2;
	pinfo->clk_rate = 6024096;

	pinfo->lcdc.h_back_porch = 8;
	pinfo->lcdc.h_front_porch = 4;
	pinfo->lcdc.h_pulse_width = 2;
	pinfo->lcdc.v_back_porch = 4;//0;
	pinfo->lcdc.v_front_porch = 2;
	pinfo->lcdc.v_pulse_width = 2;
	pinfo->lcdc.border_clr = 0;     /* blk */
	pinfo->lcdc.underflow_clr = 0xff;       /* blue */
	pinfo->lcdc.hsync_skew = 4;//4;

	ret = platform_device_register(&this_device);
	if (ret)
		platform_driver_unregister(&this_driver);

	return ret;
}

module_init(lcdc_hx8347d_panel_init);
