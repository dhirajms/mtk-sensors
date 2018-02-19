/*****************************************************************************
*  Copyright Statement:
*  --------------------
*  This software is protected by Copyright and the information contained
*  herein is confidential. The software may not be copied and the information
*  contained herein may not be used or disclosed except with the written
*  permission of MediaTek Inc. (C) 2008
*
*  BY OPENING THIS FILE, BUYER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
*  THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE")
*  RECEIVED FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO BUYER ON
*  AN "AS-IS" BASIS ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES,
*  EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF
*  MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR NONINFRINGEMENT.
*  NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY WHATSOEVER WITH RESPECT TO THE
*  SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY, INCORPORATED IN, OR
*  SUPPLIED WITH THE MEDIATEK SOFTWARE, AND BUYER AGREES TO LOOK ONLY TO SUCH
*  THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. MEDIATEK SHALL ALSO
*  NOT BE RESPONSIBLE FOR ANY MEDIATEK SOFTWARE RELEASES MADE TO BUYER'S
*  SPECIFICATION OR TO CONFORM TO A PARTICULAR STANDARD OR OPEN FORUM.
*
*  BUYER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S ENTIRE AND CUMULATIVE
*  LIABILITY WITH RESPECT TO THE MEDIATEK SOFTWARE RELEASED HEREUNDER WILL BE,
*  AT MEDIATEK'S OPTION, TO REVISE OR REPLACE THE MEDIATEK SOFTWARE AT ISSUE,
*  OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE CHARGE PAID BY BUYER TO
*  MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE.
*
*  THE TRANSACTION CONTEMPLATED HEREUNDER SHALL BE CONSTRUED IN ACCORDANCE
*  WITH THE LAWS OF THE STATE OF CALIFORNIA, USA, EXCLUDING ITS CONFLICT OF
*  LAWS PRINCIPLES.  ANY DISPUTES, CONTROVERSIES OR CLAIMS ARISING THEREOF AND
*  RELATED THERETO SHALL BE SETTLED BY ARBITRATION IN SAN FRANCISCO, CA, UNDER
*  THE RULES OF THE INTERNATIONAL CHAMBER OF COMMERCE (ICC).
*
*****************************************************************************/
#ifdef BUILD_LK

#else
#include <linux/string.h>
#endif
	
#include "lcm_drv.h"
	
#ifdef BUILD_LK
#include <mt_gpio.h>
#else
#include <mach/mt_gpio.h>
#endif


// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------

#define FRAME_WIDTH  										(480)
#define FRAME_HEIGHT 										(800)

#define REGFLAG_DELAY             							0xAB
#define REGFLAG_END_OF_TABLE      							0xAA   // END OF REGISTERS MARKER

#ifndef FALSE
  #define FALSE (0)
#endif

#ifndef TRUE
  #define TRUE  (1)
#endif


// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------

static LCM_UTIL_FUNCS lcm_util = {0};

#define SET_RESET_PIN(v)    								(lcm_util.set_reset_pin((v)))

#define UDELAY(n) 											(lcm_util.udelay(n))
#define MDELAY(n) 											(lcm_util.mdelay(n))


// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------

#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)	lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)										lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)					lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd)                                                                                   lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size)                                   lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)  


 
struct LCM_setting_table {
    unsigned char cmd;
    unsigned char count;
    unsigned char para_list[64];
};


static struct LCM_setting_table lcm_initialization_setting[] = {
	
	/*
	Note :

	Data ID will depends on the following rule.
	
		count of parameters > 1	=> Data ID = 0x39
		count of parameters = 1	=> Data ID = 0x15
		count of parameters = 0	=> Data ID = 0x05

	Structure Format :

	{DCS command, count of parameters, {parameter list}}
	{REGFLAG_DELAY, milliseconds of time, {}},

	...

	Setting ending by predefined flag
	
	{REGFLAG_END_OF_TABLE, 0x00, {}}
	*/
{0xFF,5,{0xFF,0x98,0x06,0x04,0x01}}, // Change to Page 1
{0x08,1,{0x10}},                     // output SDA
{0x21,1,{0x01}},                 // DE = 1 Active
{0x30,1,{0x02}},                 // 480 X 800
{0x31,1,{0x02}},                 // 2-Dot Inversion
{0x60,1,{0x07}},                 // SDTI
{0x61,1,{0x06}},                 // CRTI
{0x62,1,{0x06}},                 // EQTI
{0x63,1,{0x04}},                 // PCTI
{0x40,1,{0x14}},                 // BT  +2.5/-2.5 pump for DDVDH-L  //14
{0x41,1,{0x44}},                 // DVDDH DVDDL clamp   //77 AVDD5.8 5.2 AVEE -5.4  
{0x42,1,{0x01}},                 // VGH/VGL 
{0x43,1,{0x89}},                 // VGH/VGL 
{0x44,1,{0x89}},                 // VGH/VGL  
{0x45,1,{0x1B}},                 // VGL_REG   
{0x46,1,{0x44}},                 // DDVDL_PK1 2 
{0x47,1,{0x44}},                 // DDVDL_PK1 2
{0x50,1,{0x85}},                 // VGMP(+4.2)
{0x51,1,{0x85}},                 // VGMN(-4.2)
{0x52,1,{0x00}},                 //Flicker
{0x53,1,{0x64}},                 //Flicker

//++++++++++++++++++ Gamma Setting ++++++++++++++++++//
{0xA0,1,{0x00}}, // Gamma 0 /255
{0xA1,1,{0x00}}, // Gamma 4 /251
{0xA2,1,{0x03}}, // Gamma 8 /247
{0xA3,1,{0x0E}}, // Gamma 16 /239
{0xA4,1,{0x08}}, // Gamma 24 /231
{0xA5,1,{0x1F}}, // Gamma 52 / 203
{0xA6,1,{0x0F}}, // Gamma 80 / 175
{0xA7,1,{0x0B}}, // Gamma 108 /147
{0xA8,1,{0x03}}, // Gamma 147 /108
{0xA9,1,{0x06}}, // Gamma 175 / 80
{0xAA,1,{0x05}}, // Gamma 203 / 52
{0xAB,1,{0x02}}, // Gamma 231 / 24
{0xAC,1,{0x0E}}, // Gamma 239 / 16
{0xAD,1,{0x25}}, // Gamma 247 / 8
{0xAE,1,{0x1D}}, // Gamma 251 / 4
{0xAF,1,{0x00}}, // Gamma 255 / 0
///==============Nagitive
{0xC0,1,{0x00}}, // Gamma 0 
{0xC1,1,{0x04}}, // Gamma 4
{0xC2,1,{0x0F}}, // Gamma 8
{0xC3,1,{0x10}}, // Gamma 16
{0xC4,1,{0x0B}}, // Gamma 24
{0xC5,1,{0x1E}}, // Gamma 52
{0xC6,1,{0x09}}, // Gamma 80
{0xC7,1,{0x0A}}, // Gamma 108
{0xC8,1,{0x00}}, // Gamma 147
{0xC9,1,{0x0A}}, // Gamma 175
{0xCA,1,{0x01}}, // Gamma 203
{0xCB,1,{0x06}}, // Gamma 231
{0xCC,1,{0x09}}, // Gamma 239
{0xCD,1,{0x2A}}, // Gamma 247
{0xCE,1,{0x28}}, // Gamma 251
{0xCF,1,{0x00}}, // Gamma 255

//****************************************************************************//
//****************************** Page 6 Command ******************************//
//****************************************************************************//
{0xFF,5,{0xFF,0x98,0x06,0x04,0x06}},     // Change to Page 6
{0x00,1,{0xA0}},
{0x01,1,{0x05}},
{0x02,1,{0x00}},    
{0x03,1,{0x00}},
{0x04,1,{0x01}},
{0x05,1,{0x01}},
{0x06,1,{0x88}},    
{0x07,1,{0x04}},
{0x08,1,{0x01}},
{0x09,1,{0x90}},    
{0x0A,1,{0x04}},    
{0x0B,1,{0x01}},    
{0x0C,1,{0x01}},
{0x0D,1,{0x01}},
{0x0E,1,{0x00}},
{0x0F,1,{0x00}},
{0x10,1,{0x55}},
{0x11,1,{0x50}},
{0x12,1,{0x01}},
{0x13,1,{0x85}},
{0x14,1,{0x85}}, 
{0x15,1,{0xC0}}, 
{0x16,1,{0x0B}}, 
{0x17,1,{0x00}}, 
{0x18,1,{0x00}}, 
{0x19,1,{0x00}}, 
{0x1A,1,{0x00}}, 
{0x1B,1,{0x00}}, 
{0x1C,1,{0x00}}, 
{0x1D,1,{0x00}}, 
{0x20,1,{0x01}}, 
{0x21,1,{0x23}}, 
{0x22,1,{0x45}}, 
{0x23,1,{0x67}}, 
{0x24,1,{0x01}}, 
{0x25,1,{0x23}}, 
{0x26,1,{0x45}}, 
{0x27,1,{0x67}}, 
{0x30,1,{0x02}}, 
{0x31,1,{0x22}}, 
{0x32,1,{0x11}},
{0x33,1,{0xAA}},
{0x34,1,{0xBB}},
{0x35,1,{0x66}},
{0x36,1,{0x00}},
{0x37,1,{0x22}},
{0x38,1,{0x22}},
{0x39,1,{0x22}},
{0x3A,1,{0x22}},
{0x3B,1,{0x22}},
{0x3C,1,{0x22}},
{0x3D,1,{0x22}},
{0x3E,1,{0x22}},
{0x3F,1,{0x22}},
{0x40,1,{0x22}},
{0x53,1,{0x1A}},  //VGLO refer VGL_REG
     
//****************************************************************************//
//****************************** Page 7 Command ******************************//
//****************************************************************************//
{0xFF,5,{0xFF,0x98,0x06,0x04,0x07}},     // Change to Page 7
{0x18,1,{0x1D}},
{0x17,1,{0x12}},  // VGL_REG ON
{0x02,1,{0x77}},
{0xE1,1,{0x79}},
{0x06,1,{0x13}},    //VCL = -2 VCI
{0xFF,5,{0xFF,0x98,0x06,0x04,0x00}},     // Change to Page 0

{0x11,1, {0x00}},
{REGFLAG_DELAY, 125, {}},
 // Display ON
{0x29,1, 	{0x00}},
{REGFLAG_DELAY, 20, {}},
{0x3A,1,{0x66}},
{0x36,1, {0x00}},
{0xFF,5,{0xff,0x98,0x06,0x04,0x08}},//zhaoshaopeng 20140122
//{0x2c, 1, {0x00}},
//{REGFLAG_DELAY, 20, {}},

 {REGFLAG_END_OF_TABLE, 0x00, {}}	
};

static struct LCM_setting_table lcm_ese_pre_setting[] = {
{0xFF,5,{0xFF,0x98,0x06,0x04,0x00}},
{REGFLAG_END_OF_TABLE, 0x00, {}}	
};
/*
static struct LCM_setting_table lcm_set_window[] = {
	{0x2A,	4,	{0x00, 0x00, (FRAME_WIDTH>>8), (FRAME_WIDTH&0xFF)}},
	{0x2B,	4,	{0x00, 0x00, (FRAME_HEIGHT>>8), (FRAME_HEIGHT&0xFF)}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};
*/

static struct LCM_setting_table lcm_sleep_out_setting[] = {
    // Sleep Out
    {0xFF,5,{0xFF,0x98,0x06,0x04,0x00}},
	{0x11, 0, {0x00}},
       {REGFLAG_DELAY, 125, {}},
    // Display ON
	{0x29, 0, {0x00}},
	{REGFLAG_DELAY, 20, {}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}

};


static struct LCM_setting_table lcm_deep_sleep_mode_in_setting[] = {
	// Display off sequence
{0xFF,5,{0xFF,0x98,0x06,0x04,0x00}},
	{0x28, 0, {0x00}},
       {REGFLAG_DELAY, 120, {}},
       // Sleep Mode On
	{0x10, 0, {0x00}},
       {REGFLAG_DELAY, 20, {}},    
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};


static struct LCM_setting_table lcm_backlight_level_setting[] = {
	{0x51, 1, {0xFF}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table lcm_backlight_mode_setting[] = {
	{0x55, 1, {0x1}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};


static void push_table(struct LCM_setting_table *table, unsigned int count, unsigned char force_update)
{
	unsigned int i;

    for(i = 0; i < count; i++) {
		
        unsigned cmd;
        cmd = table[i].cmd;
		
        switch (cmd) {
			
            case REGFLAG_DELAY :
                MDELAY(table[i].count);
                break;
				
            case REGFLAG_END_OF_TABLE :
                break;
				
            default:
				dsi_set_cmdq_V2(cmd, table[i].count, table[i].para_list, force_update);
       	}
    }
	
}


// ---------------------------------------------------------------------------
//  LCM Driver Implementations
// ---------------------------------------------------------------------------

static void lcm_set_util_funcs(const LCM_UTIL_FUNCS *util)
{
    memcpy(&lcm_util, util, sizeof(LCM_UTIL_FUNCS));
}


static void lcm_get_params(LCM_PARAMS *params)
{
    memset(params, 0, sizeof(LCM_PARAMS));
    
    params->type   = LCM_TYPE_DSI;
    
    params->width  = FRAME_WIDTH;
    params->height = FRAME_HEIGHT;
    
    // enable tearing-free
    params->dbi.te_mode				= LCM_DBI_TE_MODE_VSYNC_ONLY;
    params->dbi.te_edge_polarity		= LCM_POLARITY_RISING;
    
    //params->dsi.mode   = BURST_VDO_MODE;
    params->dsi.mode   = SYNC_PULSE_VDO_MODE;	
    
    // DSI
    /* Command mode setting */
    params->dsi.LANE_NUM				= LCM_TWO_LANE;
    
    //The following defined the fomat for data coming from LCD engine.
    params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
    params->dsi.data_format.trans_seq	= LCM_DSI_TRANS_SEQ_MSB_FIRST;
    params->dsi.data_format.padding 	= LCM_DSI_PADDING_ON_LSB;
    params->dsi.data_format.format	  = LCM_DSI_FORMAT_RGB888;
    
    // Video mode setting		
    params->dsi.intermediat_buffer_num = 2;
    
    params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;
    
    params->dsi.word_count=480*3;	//DSI CMD mode need set these two bellow params, different to 6577
    params->dsi.vertical_active_line=800;

    params->dsi.vertical_sync_active				= 4;
    params->dsi.vertical_backporch					= 16;
    params->dsi.vertical_frontporch					= 20;	
    params->dsi.vertical_active_line				= FRAME_HEIGHT;
    
    params->dsi.horizontal_sync_active				= 10;
    params->dsi.horizontal_backporch				= 40;
    params->dsi.horizontal_frontporch				= 40;
    params->dsi.horizontal_blanking_pixel				= 80;
    params->dsi.horizontal_active_pixel				= FRAME_WIDTH;
    params->dsi.compatibility_for_nvk = 0;		// this parameter would be set to 1 if DriverIC is NTK's and when force match DSI clock for NTK's
    
    // Bit rate calculation
#ifdef CONFIG_MT6589_FPGA
    params->dsi.pll_div1=2;		// div1=0,1,2,3;div1_real=1,2,4,4
    params->dsi.pll_div2=2;		// div2=0,1,2,3;div1_real=1,2,4,4
    params->dsi.fbk_div =8;		// fref=26MHz, fvco=fref*(fbk_div+1)*2/(div1_real*div2_real)
#else//zhaoshaopeng for ili9806c 390Mhz
    params->dsi.pll_div1=1;		// div1=0,1,2,3;div1_real=1,2,4,4
    params->dsi.pll_div2=1;		// div2=0,1,2,3;div2_real=1,2,4,4
    params->dsi.fbk_div =28;//0x27		// fref=26MHz, fvco=fref*(fbk_div+1)*2/(div1_real*div2_real)		
#endif

}

static void lcm_init(void)
{


    SET_RESET_PIN(1);
    MDELAY(20 );
    SET_RESET_PIN(0);
    MDELAY(30);
    SET_RESET_PIN(1);
    MDELAY(120);
	push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);
}


static void lcm_suspend(void)
{
    push_table(lcm_deep_sleep_mode_in_setting, sizeof(lcm_deep_sleep_mode_in_setting) / sizeof(struct LCM_setting_table), 1);
    SET_RESET_PIN(1);
    MDELAY(20 );
    SET_RESET_PIN(0);
    MDELAY(50);
    SET_RESET_PIN(1);
    MDELAY(120);	

}


static void lcm_resume(void)
{
   // push_table(lcm_sleep_out_setting, sizeof(lcm_sleep_out_setting) / sizeof(struct LCM_setting_table), 1);
    lcm_init();
  	
}


static void lcm_update(unsigned int x, unsigned int y,
                       unsigned int width, unsigned int height)
{
	unsigned int x0 = x;
	unsigned int y0 = y;
	unsigned int x1 = x0 + width - 1;
	unsigned int y1 = y0 + height - 1;

	unsigned char x0_MSB = ((x0>>8)&0xFF);
	unsigned char x0_LSB = (x0&0xFF);
	unsigned char x1_MSB = ((x1>>8)&0xFF);
	unsigned char x1_LSB = (x1&0xFF);
	unsigned char y0_MSB = ((y0>>8)&0xFF);
	unsigned char y0_LSB = (y0&0xFF);
	unsigned char y1_MSB = ((y1>>8)&0xFF);
	unsigned char y1_LSB = (y1&0xFF);

	unsigned int data_array[16];


	data_array[0]= 0x00053902;
	data_array[1]= (x1_MSB<<24)|(x0_LSB<<16)|(x0_MSB<<8)|0x2a;
	data_array[2]= (x1_LSB);
    dsi_set_cmdq(data_array, 3, 1);

    data_array[0]= 0x00053902;
    data_array[1]= (y1_MSB<<24)|(y0_LSB<<16)|(y0_MSB<<8)|0x2b;
    data_array[2]= (y1_LSB);
    dsi_set_cmdq(data_array, 3, 1);

    data_array[0]= 0x002c3909;
    dsi_set_cmdq(data_array, 1, 0);
}


static void lcm_setbacklight(unsigned int level)
{
	unsigned int default_level = 0;
	unsigned int mapped_level = 0;

	//for LGE backlight IC mapping table
	if(level > 255) 
			level = 255;

	if(level >0) 
			mapped_level = default_level+(level)*(255-default_level)/(255);
	else
			mapped_level=0;

	// Refresh value of backlight level.
	lcm_backlight_level_setting[0].para_list[0] = mapped_level;

	push_table(lcm_backlight_level_setting, sizeof(lcm_backlight_level_setting) / sizeof(struct LCM_setting_table), 1);
}

static void lcm_setbacklight_mode(unsigned int mode)
{
	lcm_backlight_mode_setting[0].para_list[0] = mode;
	push_table(lcm_backlight_mode_setting, sizeof(lcm_backlight_mode_setting) / sizeof(struct LCM_setting_table), 1);
}

//static void lcm_setpwm(unsigned int divider)
//{
	// TBD
//}


//static unsigned int lcm_getpwm(unsigned int divider)
//{
	// ref freq = 15MHz, B0h setting 0x80, so 80.6% * freq is pwm_clk;
	// pwm_clk / 255 / 2(lcm_setpwm() 6th params) = pwm_duration = 23706
//	unsigned int pwm_clk = 23706 / (1<<divider);	
//	return pwm_clk;
//}
static struct LCM_setting_table lcm_compare_id_setting[] = {
        {0xFF, 5 ,{0xFF,0x98,0x06,0x04,0x01}},
        {REGFLAG_DELAY, 10, {}},

    // Sleep Mode On
//      {0xC3, 1, {0xFF}},

        {REGFLAG_END_OF_TABLE, 0x00, {}}
};
static unsigned int lcm_compare_id(void)
{

        int array[4];
        char buffer[5];
        char id_high=0;
        char id_midd=0;
        char id_low=0;
        int id=0,lcd_id=0;
		
    mt_set_gpio_mode(GPIO_LCM_ID, GPIO_LCM_ID_M_GPIO);
	mt_set_gpio_dir(GPIO_LCM_ID, GPIO_DIR_IN);		
	mt_set_gpio_pull_enable(GPIO_LCM_ID,GPIO_PULL_DISABLE);
	mt_set_gpio_pull_select(GPIO_LCM_ID,GPIO_PULL_DOWN);
    MDELAY(1);
	
        //Do reset here
        SET_RESET_PIN(1);
	MDELAY(20);
	SET_RESET_PIN(0);
        MDELAY(50);
	SET_RESET_PIN(1);
	MDELAY(50); 

        array[0]=0x00063902;
        array[1]=0x0698ffff;
        array[2]=0x00000104;
        dsi_set_cmdq(array, 3, 1);
        MDELAY(10);
        array[0] = 0x00033700;
        dsi_set_cmdq(array, 1, 1);
        //read_reg_9806(0x04, buffer, 3);//if read 0x04,should get 0x008000,that is both OK.
        read_reg_v2(0x00, buffer,1);
        id_high = buffer[0]; /////////////////////////0x98
        read_reg_v2(0x01, buffer,1);
        id_midd = buffer[0]; ///////////////////////0x06
		read_reg_v2(0x02, buffer,1);
        id_low = buffer[0]; ///////////////////////0x04
        id = (id_midd << 8) | id_low;
        
		lcd_id =  mt_get_gpio_in(GPIO_LCM_ID);
	#if defined(BUILD_LK)
	printf("*******lanhong 9806E***************  id_high=%x,id_midd=%x,id_low=%x ,id=%x ,lcd_id=%d \n", id_high,id_midd, id_low,id,lcd_id);
	#endif
	
		return (lcd_id ==0 && 0x0604 == id) ? 1: 0;
}

// ---------------------------------------------------------------------------
//  Get LCM Driver Hooks
// ---------------------------------------------------------------------------
#if 1
static unsigned int lcm_esd_check(void)
{
	char  buffer[5];
	char  buffer1[5];
	int   array[4];
	//push_table(lcm_compare_id_setting, sizeof(lcm_compare_id_setting) / sizeof(struct LCM_setting_table), 1);
	push_table(lcm_ese_pre_setting, sizeof(lcm_ese_pre_setting) /sizeof(struct LCM_setting_table), 1);

	array[0] = 0x00013700;
	dsi_set_cmdq(array, 1, 1);

	read_reg_v2(0x0a, buffer, 1);	
	#ifdef BUILD_LK
	printf("leanda ILI9806 buffer1[0] = %x\n",buffer[0]);
	#else
	printk("leanda2  ILI9806 buffer1[0] 0x0a= %x\n",buffer[0]);
	#endif


	array[0] = 0x00013700;
	dsi_set_cmdq(array, 1, 1);

	read_reg_v2(0x0b, buffer1, 1);	
	#ifdef BUILD_LK
	printf("leanda ILI9806 buffer1[0] = %x\n",buffer1[0]);
	#else
	printk("leanda2  ILI9806 buffer1[0] 0x0b= %x\n",buffer1[0]);
	#endif
	
	
	//array[0] = 0x00043700;
       //dsi_set_cmdq(array, 1, 1);
	//read_reg_v2(0x09, buffer, 4);
	#ifdef BUILD_LK
	//printf("leanda  ILI9806 buffer[0] = %x,buffer[1] = %x,buffer[2] = %x,buffer[3] = %x,\n",buffer[0],buffer[1],buffer[2],buffer[3]);
	#else
	//printk("leanda2 ILI9806 buffer[0] = %x,buffer[1] = %x,buffer[2] = %x,buffer[3] = %x\n",buffer[0],buffer[1],buffer[2],buffer[3]);
	#endif
	if((buffer[0]==0x9c)&&(buffer1[0]==0x00))
	{
		return FALSE;
	}
        else
	{	
		return TRUE;
	}
}



static unsigned int lcm_esd_recover(void)
{	
	lcm_init();
	return TRUE;
}
#endif
// ---------------------------------------------------------------------------
//  Get LCM Driver Hooks
// ---------------------------------------------------------------------------
LCM_DRIVER ili9806e_wvga_dsi_vdo_v815w_azet_lcm_drv = 
{
    .name          = "ili9806e_wvga_dsi_vdo_v815w_azet",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params     = lcm_get_params,
	.init           = lcm_init,
	.suspend        = lcm_suspend,
	.resume         = lcm_resume,
	//.update         = lcm_update,
	//.set_backlight	= lcm_setbacklight,
	//.set_backlight_mode = lcm_setbacklight_mode,
	//.set_pwm        = lcm_setpwm,
	//.get_pwm        = lcm_getpwm  
     	.compare_id    = lcm_compare_id,
	//.esd_check      = lcm_esd_check,
	//.esd_recover    = lcm_esd_recover,

};
