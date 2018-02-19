/* Copyright Statement:
 *
 * This software/firmware and related documentation ("MediaTek Software") are
 * protected under relevant copyright laws. The information contained herein
 * is confidential and proprietary to MediaTek Inc. and/or its licensors.
 * Without the prior written permission of MediaTek inc. and/or its licensors,
 * any reproduction, modification, use or disclosure of MediaTek Software,
 * and information contained herein, in whole or in part, shall be strictly prohibited.
 */
/* MediaTek Inc. (C) 2010. All rights reserved.
 *
 * BY OPENING THIS FILE, RECEIVER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
 * THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE")
 * RECEIVED FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO RECEIVER ON
 * AN "AS-IS" BASIS ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR NONINFRINGEMENT.
 * NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY WHATSOEVER WITH RESPECT TO THE
 * SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY, INCORPORATED IN, OR
 * SUPPLIED WITH THE MEDIATEK SOFTWARE, AND RECEIVER AGREES TO LOOK ONLY TO SUCH
 * THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. RECEIVER EXPRESSLY ACKNOWLEDGES
 * THAT IT IS RECEIVER'S SOLE RESPONSIBILITY TO OBTAIN FROM ANY THIRD PARTY ALL PROPER LICENSES
 * CONTAINED IN MEDIATEK SOFTWARE. MEDIATEK SHALL ALSO NOT BE RESPONSIBLE FOR ANY MEDIATEK
 * SOFTWARE RELEASES MADE TO RECEIVER'S SPECIFICATION OR TO CONFORM TO A PARTICULAR
 * STANDARD OR OPEN FORUM. RECEIVER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S ENTIRE AND
 * CUMULATIVE LIABILITY WITH RESPECT TO THE MEDIATEK SOFTWARE RELEASED HEREUNDER WILL BE,
 * AT MEDIATEK'S OPTION, TO REVISE OR REPLACE THE MEDIATEK SOFTWARE AT ISSUE,
 * OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE CHARGE PAID BY RECEIVER TO
 * MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE.
 *
 * The following software/firmware and/or related documentation ("MediaTek Software")
 * have been modified by MediaTek Inc. All revisions are subject to any receiver's
 * applicable license agreements with MediaTek Inc.
 */

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
#include <string.h>
#else
#include <linux/string.h>
#endif


#ifdef BUILD_LK
#include "cust_gpio_usage.h"
#else
#include "cust_gpio_usage.h"
#endif

#ifndef BUILD_LK
#include <mach/mt_gpio.h>
#endif

/*
#ifndef BUILD_LK
#include <linux/printk.h>
#endif
*/

#ifndef BUILD_LK
#include <linux/kernel.h>
#endif

#include "lcm_drv.h"

// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------

#define FRAME_WIDTH  										(480)
#define FRAME_HEIGHT 										(800)
#define LCM_ID       (0x00)
#define REGFLAG_DELAY             							0XAB
#define REGFLAG_END_OF_TABLE      							0xAA   // END OF REGISTERS MARKER


#ifndef TRUE
    #define TRUE 1
#endif

#ifndef FALSE
    #define FALSE 0
#endif

static unsigned int lcm_esd_test = FALSE;      ///only for ESD test
#define LCM_DSI_CMD_MODE									0

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

#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)	        lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)										lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)					lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd)											lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size)   				lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)

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

       
       // add by zhuqiang for FR437058 at 2013.4.25 begin
       {0xFF,	3,	{0xFF, 0x98, 0x16}},

	{0xBA,	1,	{0x60}},

       {0xB0,	1,	{0x01}},
        // add by zhuqiang for FR437058 at 2013.4.25 end
     
       // add by zhuqiang for FR454775 at 2013.5.20 begin
       {0xB9,	1,	{0x00}},
       // add by zhuqiang for FR454775 at 2013.5.20 end

       {0xBC,	18,	{0x03, 0x0D, 0x61, 0xFF, 0x01, 0x01, 0x1B, 0x11, 0x38, 0x63, 0xFF,0xFF, 0x01, 0x01, 0x10, 0x00, 0xFF, 0xF2}},

   	{0xBD,	8,	{0x02, 0x13, 0x45, 0x67, 0x45, 0x67, 0x01, 0x23}},

       {0xBE,	17,	{0x03, 0x22, 0x22, 0x22, 0x22, 0xDD, 0xCC, 0xBB, 0xAA, 0x66, 0x77,0x22, 0x22, 0x22, 0x22, 0x22, 0x22}},

      	{0xED,	2,	{0x7F, 0x0F}},

    	{0xF3,	1,	{0x70}},
   
      	{0xB4,	1,	{0x02}},

    	{0xC0,	3,	{0x0F, 0x0B, 0x0A}},

       {0xC1,	4,	{0x17, 0x88, 0x88, 0x20}},

      	{0xD8,	1,	{0x50}},

       {0xFC,	1,	{0x08}},     
       {0xE0,	16,	{0x00, 0x01, 0x06, 0x10, 0x11, 0x1C, 0xC8, 0x06, 0x03, 0x02, 0x02,0x1B, 0x13, 0x30, 0x27, 0x00}},
 
       {0xE1,	16,	{0x00, 0x04, 0x07, 0x0D, 0x0F, 0x16, 0x78, 0x07, 0x06, 0x0F, 0x07,0x04, 0x06, 0x2A, 0x26, 0x00}},

       {0xD5,	8,	{0x0A, 0x0A, 0x08, 0x07, 0xCB, 0xA5, 0x01, 0x04}},   
  
       {0xF7,	1,	{0x8A}},    

       {0xC7,	1,	{0x6E}},

       {0x11,	1,	{0x00}},
	{REGFLAG_DELAY, 120, {}},

    {0XEE,	9,	{0x0A, 0x1B, 0x5F, 0x40, 0x00, 0X00, 0x10, 0x00, 0x58}}, // Source Timing Adjust

	{0x29,	1,	{0x00}},
	{REGFLAG_DELAY, 50, {}},

   	{0x2C,	1,	{0x00}},

        // add by zhuqiang for FR437058 at 2013.4.25 end

   	//{REGFLAG_DELAY, 50, {}},


	// Note
	// Strongly recommend not to set Sleep out / Display On here. That will cause messed frame to be shown as later the backlight is on.

	// Setting ending by predefined flag

	{REGFLAG_END_OF_TABLE, 0x00, {}}  
    

 
};


#if 0
static struct LCM_setting_table lcm_set_window[] = {
	{0x2A,	4,	{0x00, 0x00, (FRAME_WIDTH>>8), (FRAME_WIDTH&0xFF)}},
	{0x2B,	4,	{0x00, 0x00, (FRAME_HEIGHT>>8), (FRAME_HEIGHT&0xFF)}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};
#endif

static struct LCM_setting_table lcm_sleep_out_setting[] = {
    // Sleep Out
       // add by zhuqiang for FR437058 at 2013.4.26 begin
    //   {0x00, 1, {0x00}},
       // add by zhuqiang for FR437058 at 2013.4.26 end
	
       // add by zhuqiang for FR437058 at 2013.4.26 begin
       {0x11, 1, {0x00}},
      {0x29, 1, {0x00}},
      // add by zhuqiang for FR437058 at 2013.4.26 end
       {0x11, 1, {0x00}},
        {REGFLAG_DELAY, 150, {}},

    // Display ON
	{0x29, 1, {0x00}},
        {REGFLAG_DELAY, 50, {}},

	{0x2C, 1, {0x00}},

	{REGFLAG_END_OF_TABLE, 0x00, {}}
};


static struct LCM_setting_table lcm_deep_sleep_mode_in_setting[] = {
	// Display off sequence
	{0x28, 1, {0x00}},
        {REGFLAG_DELAY, 50, {}},

    // Sleep Mode On
	{0x10, 1, {0x00}},
        {REGFLAG_DELAY, 120, {}},

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
		params->dbi.te_mode 				= LCM_DBI_TE_MODE_VSYNC_ONLY;
		params->dbi.te_edge_polarity		= LCM_POLARITY_RISING;

#if (LCM_DSI_CMD_MODE)
		params->dsi.mode   = CMD_MODE;
#else
		params->dsi.mode   = SYNC_PULSE_VDO_MODE;
#endif

		// DSI
		/* Command mode setting */
		params->dsi.LANE_NUM				= LCM_TWO_LANE;
		//The following defined the fomat for data coming from LCD engine.
		params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
		params->dsi.data_format.trans_seq   = LCM_DSI_TRANS_SEQ_MSB_FIRST;
		params->dsi.data_format.padding     = LCM_DSI_PADDING_ON_LSB;
		params->dsi.data_format.format      = LCM_DSI_FORMAT_RGB888;

		// Highly depends on LCD driver capability.
		params->dsi.packet_size=256;

		// Video mode setting	
               
               // add by zhuqiang for FR437058 at 2013.4.25 begin
              params->dsi.intermediat_buffer_num = 2;	
              // add by zhuqiang for FR437058 at 2013.4.25 end
		params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;

              params->dsi.word_count=480*3;

             //delete by zhuqiang 2013.3.4 
	//	params->dsi.word_count=FRAME_WIDTH*3;	
             // add by zhuqiang for FR437058 at 2013.4.25 begin
		params->dsi.vertical_sync_active=4;  
		params->dsi.vertical_backporch=16;
		params->dsi.vertical_frontporch=20;
               // add by zhuqiang for FR437058 at 2013.4.25 end
		params->dsi.vertical_active_line=FRAME_HEIGHT;
	      
             //delete by zhuqiang 2013.3.4 
	//	params->dsi.line_byte=2180;		
              // add by zhuqiang for FR437058 at 2013.4.25 begin
		params->dsi.horizontal_sync_active=10;  
		params->dsi.horizontal_backporch=41;      
		params->dsi.horizontal_frontporch=41;    
              // add by zhuqiang for FR437058 at 2013.4.25 end
              params->dsi.horizontal_active_pixel = FRAME_WIDTH;	//added by zhuqiang 2013.3.4 
	       
             /* delete by zhuqiang 2013.3.4 
        	params->dsi.rgb_byte=(FRAME_WIDTH*3+6);		// NC
	       
		params->dsi.horizontal_sync_active_word_count=20;	
		params->dsi.horizontal_backporch_word_count=200;
		params->dsi.horizontal_frontporch_word_count=200;

             */
                 /*
                // added by zhuqiang for one lane speed maximum is 270MHz start.20121024
		// Bit rate calculation
		params->dsi.pll_div1=35;		// fref=26MHz, fvco=fref*(div1+1)	(div1=0~63, fvco=500MHZ~1GHz)
		params->dsi.pll_div2=1;			// div2=0~15: fout=fvo/(2*div2) 300MHZ
                // added by zhuqiang for one lane speed maximum is 270MHz end.20121024
               */
             // add by zhuqiang for FR437058 at 2013.4.25 begin
             params->dsi.pll_div1=0;         //  div1=0,1,2,3;  div1_real=1,2,4,4
             params->dsi.pll_div2=2;         // div2=0,1,2,3;div2_real=1,2,4,4
             params->dsi.fbk_div =0x1E;              // fref=26MHz,  fvco=fref*(fbk_div+1)*2/(div1_real*div2_real)

            // add by zhuqiang for FR437058 at 2013.4.25 end
}


static void lcm_init(void)
{
    unsigned int data_array[16];

    SET_RESET_PIN(1);
    SET_RESET_PIN(0);
    MDELAY(20);
    SET_RESET_PIN(1);
    MDELAY(120);

    push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);

}


static void lcm_suspend(void)
{
	push_table(lcm_deep_sleep_mode_in_setting, sizeof(lcm_deep_sleep_mode_in_setting) / sizeof(struct LCM_setting_table), 1);
}


static void lcm_resume(void)
{
      // add by zhuqiang for FR437058 at 2013.4.26 begin
	//lcm_init();
      // add by zhuqiang for FR437058 at 2013.4.26 end
	push_table(lcm_sleep_out_setting, sizeof(lcm_sleep_out_setting) / sizeof(struct LCM_setting_table), 1);
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
	data_array[3]= 0x00053902;
	data_array[4]= (y1_MSB<<24)|(y0_LSB<<16)|(y0_MSB<<8)|0x2b;
	data_array[5]= (y1_LSB);
	data_array[6]= 0x002c3909;

	dsi_set_cmdq(data_array, 7, 0);

}

// added by zhuqiang for lcd esd begin 2012.11.19

static unsigned int lcm_esd_check(void)
{
#ifndef BUILD_UBOOT

       unsigned char buffer_vcom[4];

       unsigned char buffer_0a[1];

       unsigned int array[16];


       array[0]=0x00341500;

       dsi_set_cmdq(&array, 1, 1);

       array[0] = 0x00013700;

       dsi_set_cmdq(array, 1, 1);

      read_reg_v2(0x0A,buffer_0a, 1);

      array[0] = 0x00043700;

      dsi_set_cmdq(array, 1, 1);

      read_reg_v2(0xC5, buffer_vcom, 4);

      array[0]=0x00351500;

      dsi_set_cmdq(&array, 1, 1);
     //printk("lcm 0x0a is %x--------------\n", buffer_0a[0]);
     //printk("lcm 0xc5 is %x,%x,%x,%x--------------\n", buffer_vcom[0], buffer_vcom[1] ,buffer_vcom[2], buffer_vcom[3]);
    //  if ((buffer_vcom[0]==0x00)&&(buffer_vcom[1]==0x41)&&(buffer_vcom[3]==0x41)&&(buffer_0a[0]==0x9C)){

               return 0;


   //   }else{

  //            return 1;
  //    }
#endif
}


static unsigned int lcm_esd_recover(void)
{

   #ifndef BUILD_UBOOT


       lcm_init();

       return 1;

      #endif 
}

 // added by zhuqiang for lcd esd end 2012.11.19

static unsigned int lcm_compare_id(void)
{
    unsigned char buffer_vcom[4];

       unsigned int array[16];
      unsigned int id=0;

       array[0]=0x00341500;

       dsi_set_cmdq(&array, 1, 1);

      array[0] = 0x00043700;

      dsi_set_cmdq(array, 1, 1);
 
      read_reg_v2(0xD3, buffer_vcom, 4);

      array[0]=0x00351500;

      dsi_set_cmdq(&array, 1, 1);
 
// #ifdef BUILD_LK
//      printf("lcm ili9486 id is %x,%x,%x,%x--------------\n", buffer_vcom[0], buffer_vcom[1] ,buffer_vcom[2], buffer_vcom[3]);
//#else
//     printk(KERN_ERR "lcm ili9486 id is %x,%x,%x,%x--------------\n", buffer_vcom[0], buffer_vcom[1] ,buffer_vcom[2], buffer_vcom[3]);
//#endif

        mt_set_gpio_mode(18,0);  // gpio mode   high
	mt_set_gpio_pull_enable(18,0);
	mt_set_gpio_dir(18,0);  //input
   	//mt_set_gpio_pull_select(50, 1);
        MDELAY(5);
	id = mt_get_gpio_in(18) ;

 //      #ifdef BUILD_LK
//      printf("lcm  txd txd txd txd txd txd txd ili9488 driver-----pin number id is %x---------\n",id);
//#else
//      printk(KERN_ERR "lcm  txd txd txd txd txd txd txd ili9488 driver-----pin number id is %x---------\n",id);
//#endif


      if (id==0x01){

               return 1;

     }else{

              return 0;
     }

}
// ---------------------------------------------------------------------------
//  Get LCM Driver Hooks
// ---------------------------------------------------------------------------
LCM_DRIVER ili9806_dsi_6572_lcm_drv_txd =
{
    .name			= "ili9806_dsi_6572_txd",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params     = lcm_get_params,
	.init           = lcm_init,
	.suspend        = lcm_suspend,
	.resume         = lcm_resume,
//#if (LCM_DSI_CMD_MODE)
//	.update         = lcm_update,
//	.set_backlight	= lcm_setbacklight,
//	.set_pwm        = lcm_setpwm,
//	.get_pwm        = lcm_getpwm,
   
      // added by zhuqiang for lcd esd begin 2012.11.19
//	.esd_check   = lcm_esd_check,
 //  	.esd_recover   = lcm_esd_recover,
      // added by zhuqiang for lcd esd end 2012.11.19
	.compare_id    = lcm_compare_id,
//#endif
};
