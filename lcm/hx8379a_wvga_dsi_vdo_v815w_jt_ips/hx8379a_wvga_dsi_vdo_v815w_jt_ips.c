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
#else
    #include <linux/string.h>
#endif

#ifdef BUILD_LK
#include <mt_gpio.h>
#else
#include <mach/mt_gpio.h>
#endif

#include "lcm_drv.h"
 
// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------
 
#define FRAME_WIDTH   (480)
#define FRAME_HEIGHT  (800)
#define ILI9806_LCM_ID           (128)
 
#define REGFLAG_DELAY                    0XFFE
#define REGFLAG_END_OF_TABLE             0xFFF//gxf 0xFF   // END OF REGISTERS MARKER
//#define LCM_DSI_CMD_MODE          1
//#define DRV_LCM_ONE_LANE

#ifndef TRUE
    #define TRUE 	1
#endif

#ifndef FALSE
    #define FALSE 	0
#endif

static unsigned int lcm_esd_test = FALSE;      ///only for ESD test

// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------
 
static LCM_UTIL_FUNCS lcm_util = {0};
 
#define SET_RESET_PIN(v)            (lcm_util.set_reset_pin((v)))
 
#define UDELAY(n)            (lcm_util.udelay(n))
#define MDELAY(n)            (lcm_util.mdelay(n))
 

// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------
 
#define dsi_set_cmdq_V2(cmd, count, ppara, force_update) lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)  lcm_util.dsi_set_cmdq(pdata, queue_size, force_update); MDELAY(10) 
#define wrtie_cmd(cmd)          lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)     lcm_util.dsi_write_regs(addr, pdata, byte_nums)
//#define read_reg           lcm_util.dsi_read_reg()
#define read_reg(cmd)											lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size)       lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)
 
#define dsi_set_cmdq_HQ(cmd, count, ppara, force_update) lcm_util.dsi_set_cmdq_HQ(cmd, count, ppara, force_update)
extern int IMM_GetOneChannelValue(int dwChannel, int deCount);
//int lcd_id_voltage_nt35510h_dsi_BM9000B = 0;
struct LCM_setting_table {
    unsigned cmd;
    unsigned char count;
    unsigned char para_list[128];
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

{0xb9,3,{//寄存器
0xFF,//参数1
0x83,//参数2
0x79}},//参数3

{0xB1,20,{
0x44,
0x18,
0x18,
0x31,
0x51,
0x90,
0xD0,
0xEE,
0x94,
0x80,
0x38,
0x38,
0xF8,
0x22,
0x22,
0x22,
0x00,
0x80,
0x30,
0x00}},

{0xB2,9,{
0x82,
0x3C,
0x0B,
0x04,
0x00,
0x50,
0x11,
0x42,
0x1D}},

{0xB4,10,{
0x50,
0x51,
0x50,
0x51,
0x50,
0x51,
0x12,
0xA0,
0x13,
0xA0}},

{0xCC,1,{
0x02}},

{0xD2,1,{
0x33}},

{0xD3,29,{
0x00,
0x07,
0x00,
0x00,
0x00,
0x06,
0x06,
0x32,
0x10,
0x05,
0x00,
0x05,
0x03,
0x6F,
0x03,
0x6F,
0x00,
0x07,
0x00,
0x07,
0x21,
0x22,
0x05,
0x05,
0x23,
0x05,
0x05,
0x23,
0x09}},

{0xD5,32,{
0x18,
0x18,
0x19,
0x19,
0x01,
0x00,
0x03,
0x02,
0x21,
0x20,
0x18,
0x18,
0x18,
0x18,
0x18,
0x18,
0x18,
0x18,
0x18,
0x18,
0x18,
0x18,
0x18,
0x18,
0x18,
0x18,
0x18,
0x18,
0x18,
0x18,
0x18,
0x18}},

{0xD6,32,{
0x18,
0x18,
0x18,
0x18,
0x02,
0x03,
0x00,
0x01,
0x20,
0x21,
0x19,
0x19,
0x18,
0x18,
0x18,
0x18,
0x18,
0x18,
0x18,
0x18,
0x18,
0x18,
0x18,
0x18,
0x18,
0x18,
0x18,
0x18,
0x18,
0x18,
0x18,
0x18}},

{0xE0,42,{
0x00,
0x00,
0x09,
0x1A,
0x1C,
0x3F,
0x33,
0x3F,
0x0A,
0x10,
0x12,
0x1A,
0x12,
0x16,
0x19,
0x16,
0x17,
0x08,
0x12,
0x12,
0x17,
0x00,
0x00,
0x09,
0x1A,
0x1C,
0x3F,
0x34,
0x3F,
0x09,
0x10,
0x12,
0x1A,
0x11,
0x15,
0x17,
0x15,
0x16,
0x07,
0x11,
0x13,
0x17}},

{0xB6,2,{
0x74,
0x74}},

{0x11,1, {0x00}},
{REGFLAG_DELAY, 150, {}},
 // Display ON
{0x29,1, 	{0x00}},

 {REGFLAG_END_OF_TABLE, 0x00, {}}	
};

static void BOE8379c_JINGTAI_LCM_INIT(void)
{
unsigned int data_array[33];

data_array[0]=0x00043902;
data_array[1]=0x7983ffb9;
dsi_set_cmdq(&data_array, 2, 1);
MDELAY(1);
		
data_array[0]=0x00153902;
data_array[1]=0x181844b1;
data_array[2]=0xd0905131;
data_array[3]=0x388094ee;
data_array[4]=0x2222f838;
data_array[5]=0x30800022;
data_array[6]=0x0;
dsi_set_cmdq(&data_array, 7, 1);
MDELAY(1);
		
data_array[0]=0x00a3902;
data_array[1]=0xb3c82b2;
data_array[2]=0x11500004;
data_array[3]=0x1d42;
dsi_set_cmdq(&data_array, 4, 1);
MDELAY(1);

data_array[0]=0x00b3902;
data_array[1]=0x505150b4;
data_array[2]=0x12515051;
data_array[3]=0xa013a0;
dsi_set_cmdq(&data_array, 4, 1);
MDELAY(1);

data_array[0]=0x0023902;
data_array[1]=0x2cc;
dsi_set_cmdq(&data_array, 2, 1);
MDELAY(1);

data_array[0]=0x0023902;
data_array[1]=0x33d2;
dsi_set_cmdq(&data_array, 2, 1);
MDELAY(1);

data_array[0]=0x001e3902;
data_array[1]=0x700d3;
data_array[2]=0x6060000;
data_array[3]=0x51032;
data_array[4]=0x36f0305;
data_array[5]=0x7006f;
data_array[6]=0x5222107;
data_array[7]=0x5052305;
data_array[8]=0x923;
dsi_set_cmdq(&data_array, 9, 1);
MDELAY(1);

data_array[0]=0x00213902;
data_array[1]=0x191818d5;
data_array[2]=0x3000119;
data_array[3]=0x18202102;
data_array[4]=0x18181818;
data_array[5]=0x18181818;
data_array[6]=0x18181818;
data_array[7]=0x18181818;
data_array[8]=0x18181818;
data_array[9]=0x18;
dsi_set_cmdq(&data_array, 10, 1);
MDELAY(1);

data_array[0]=0x00213902;
data_array[1]=0x181818d6;
data_array[2]=0x30218;
data_array[3]=0x19212001;
data_array[4]=0x18181819;
data_array[5]=0x18181818;
data_array[6]=0x18181818;
data_array[7]=0x18181818;
data_array[8]=0x18181818;
data_array[9]=0x18;
dsi_set_cmdq(&data_array, 10, 1);
MDELAY(1);

data_array[0]=0x002b3902;
data_array[1]=0x90000e0;
data_array[2]=0x333f1c1a;
data_array[3]=0x12100a3f;
data_array[4]=0x1916121a;
data_array[5]=0x12081716;
data_array[6]=0x1712;
data_array[7]=0x3f1c1a09;
data_array[8]=0x10093f34;
data_array[9]=0x15111a12;
data_array[10]=0x7161517;
data_array[11]=0x171311;
dsi_set_cmdq(&data_array, 12, 1);
MDELAY(1);

data_array[0]=0x0033902;
data_array[1]=0x7474b6;
dsi_set_cmdq(&data_array, 2, 1);
MDELAY(1);
			
data_array[0] = 0x00110500; 	   //exit sleep mode 
dsi_set_cmdq(&data_array, 1, 1); 
MDELAY(150); 

data_array[0] = 0x00290500; 	   //exit sleep mode 
dsi_set_cmdq(&data_array, 1, 1); 
MDELAY(20);

}



static struct LCM_setting_table lcm_id_read[] = {
{0xf0,5,{0x55,0xaa,0x52,0x08,0x01}},
{REGFLAG_END_OF_TABLE, 0x00, {}}
};
 
static struct LCM_setting_table lcm_set_window[] = {
 {0x2A, 4, {0x00, 0x00, (FRAME_WIDTH>>8), (FRAME_WIDTH&0xFF)}},
 {0x2B, 4, {0x00, 0x00, (FRAME_HEIGHT>>8), (FRAME_HEIGHT&0xFF)}},
 {REGFLAG_END_OF_TABLE, 0x00, {}}
};
 
static struct LCM_setting_table lcm_read_lcm_compare_id[] = {
 {0xB9,  3, {0xFF,0x83,0x79}},
 {0xBA,  2, {0x51,0x93}}
};

static struct LCM_setting_table lcm_sleep_out_setting[] = {
 // Sleep Out
 {0x11, 0, {0x00}},
 {REGFLAG_DELAY, 120, {}},
 
 // Display ON
 {0x29, 0, {0x00}},
 {REGFLAG_DELAY, 10, {}},
 
 {REGFLAG_END_OF_TABLE, 0x00, {}}
 
};
 

static struct LCM_setting_table lcm_deep_sleep_mode_in_setting[] = {
 // Display off sequence
 {0x28, 0, {0x00}},
 {REGFLAG_DELAY, 10, {}},
 
 // Sleep Mode On
 {0x10, 0, {0x00}},
 {REGFLAG_DELAY, 10, {}},
 
 {REGFLAG_END_OF_TABLE, 0x00, {}}
};
 

static struct LCM_setting_table lcm_backlight_level_setting[] = {
 {0x51, 1, {0xFF}},
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
  params->dbi.te_mode     =LCM_DBI_TE_MODE_VSYNC_ONLY;//LCM_DBI_TE_MODE_VSYNC_ONLY; //LCM_DBI_TE_MODE_DISABLED;
  params->dbi.te_edge_polarity  = LCM_POLARITY_RISING;
 
#ifdef LCM_DSI_CMD_MODE
  params->dsi.mode   = CMD_MODE;//pulse_MODE
#else
  params->dsi.mode   = SYNC_PULSE_VDO_MODE;
#endif
 
  // DSI
  /* Command mode setting */
#ifdef DRV_LCM_ONE_LANE
  params->dsi.LANE_NUM    = LCM_ONE_LANE;
#else
  params->dsi.LANE_NUM    = LCM_TWO_LANE;
#endif
  //The following defined the fomat for data coming from LCD engine.
  params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
  params->dsi.data_format.trans_seq   = LCM_DSI_TRANS_SEQ_MSB_FIRST;
  params->dsi.data_format.padding     = LCM_DSI_PADDING_ON_LSB;
  params->dsi.data_format.format      = LCM_DSI_FORMAT_RGB888;
 
  // Highly depends on LCD driver capability.
  // Not support in MT6573
  params->dsi.packet_size=256;
 
  // Video mode setting  
  params->dsi.intermediat_buffer_num = 0;
 
  params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;
  params->dsi.word_count=480*3;
 

  params->dsi.vertical_sync_active    =5;//4;//16;//4 2;//4;//2;//4;4
  params->dsi.vertical_backporch     =11;// 9;//10;//26;//8 18;//16;//4;//8;167
  params->dsi.vertical_frontporch    = 10;//20;//10;//18;// 8 18;//4;//8;16 6
 
  params->dsi.vertical_active_line    = FRAME_HEIGHT; //810
 
  params->dsi.horizontal_sync_active    =45;//20;//20;//6;//28;//24;//6;
  params->dsi.horizontal_backporch    =65;//32;//35;//20;// 50;//36;//30;//28;//37;
  params->dsi.horizontal_frontporch    =65;//32;//35; //20;//50;//36;//30;//28;//37;
  params->dsi.horizontal_active_pixel   = FRAME_WIDTH;//496
 
  params->dsi.compatibility_for_nvk = 0;		// this parameter would be set to 1 if DriverIC is NTK's and when force match DSI clock for NTK's
  
  params->dsi.pll_div1=1;		// div1=0,1,2,3;div1_real=1,2,4,4
  params->dsi.pll_div2=1;		// div2=0,1,2,3;div2_real=1,2,4,4
  params->dsi.fbk_div =26;   // 20

 

 
}
 

static void  lcm_init(void)
{

	
 SET_RESET_PIN(1); 
 MDELAY(1);
 SET_RESET_PIN(0);
 MDELAY(20);//10 
 SET_RESET_PIN(1);
 MDELAY(120); // 150
 BOE8379c_JINGTAI_LCM_INIT();
 //lcm_initialization_setting();
//push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);

}
 
 
 
static void lcm_suspend(void)
{
		     unsigned int data_array[16];
				
				 SET_RESET_PIN(1); 
				 MDELAY(1);
				 SET_RESET_PIN(0);
				 MDELAY(10);//10 
				 SET_RESET_PIN(1);
				 MDELAY(120); // 150
				 push_table(lcm_deep_sleep_mode_in_setting, sizeof(lcm_deep_sleep_mode_in_setting) / sizeof(struct LCM_setting_table), 1);


}
 
static void lcm_resume(void)
{
 
 
    SET_RESET_PIN(1);
    MDELAY(1);    
    SET_RESET_PIN(0);
    MDELAY(10);
    SET_RESET_PIN(1);
    MDELAY(20); 
    //lcm_initialization_setting();
	lcm_init();
}
 

static void lcm_update(unsigned int x, unsigned int y,
                       unsigned int width, unsigned int height)
{
 
//        printk("gxf_lcm_update\n");
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
 
 dsi_set_cmdq(&data_array, 7, 0);
 
}
 

static void lcm_setbacklight(unsigned int level)
{
  //  printk("gxf_lcm_setbacklight\n");
#if 1//gxf 
 // Refresh value of backlight level.
 lcm_backlight_level_setting[0].para_list[0] = level;
 
 //push_table(lcm_backlight_level_setting, sizeof(lcm_backlight_level_setting) / sizeof(struct LCM_setting_table), 1);
#else 
 unsigned int default_level = 0;//145 lenovo-sw jixu modify to 0 20120213
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
#endif 
}
 

static void lcm_setpwm(unsigned int divider)
{
 // TBD
}
 

static unsigned int lcm_getpwm(unsigned int divider)
{
 // ref freq = 15MHz, B0h setting 0x80, so 80.6% * freq is pwm_clk;
 // pwm_clk / 255 / 2(lcm_setpwm() 6th params) = pwm_duration = 23706
 unsigned int pwm_clk = 23706 / (1<<divider); 
 return pwm_clk;
}

/*
void lcm_esd_recover()
{
 #ifdef BUILD_LK
        printf("%s, LK lcm_esd_recover \n", __func__); 
        #else
        printk("%s, Kernel lcm_esd_recover\n", __func__);   
        #endif
 
 // printk("adaadfadadsads===lcm_esd_recover\n");
 //lcm_init();
 //push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);
 //lcm_resume();
  
}
*/
// ---------------------------------------------------------------------------
//  Get LCM ID Information JUST MOVE OUT SB NOUSE CODE
// ---------------------------------------------------------------------------

static unsigned int lcm_esd_check(void)
{
#ifndef BUILD_LK
        if(lcm_esd_test)
        {
            lcm_esd_test = FALSE;
            return TRUE;
        }

        if(read_reg(0x0A) == 0x9C)
        {
            return FALSE;
        }
        else
        {            
            return TRUE;
        }
#endif
}

static unsigned int lcm_esd_recover(void)
{
    unsigned char para = 0;
	unsigned int data_array1[16];

	#ifndef BUILD_UBOOT
	  printk("lcm_esd_recover enter");
	#endif
    

    SET_RESET_PIN(1);
    MDELAY(10);
    SET_RESET_PIN(0);
    MDELAY(30);
    SET_RESET_PIN(1);
    MDELAY(130);
    #if 0
	push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);
	MDELAY(10);
	push_table(lcm_sleep_out_setting, sizeof(lcm_sleep_out_setting) / sizeof(struct LCM_setting_table), 1);
    	MDELAY(10);
    #else
       // lcm_init();
    #endif
   
    data_array1[0]= 0x00320500;
	dsi_set_cmdq(&data_array1, 1, 1);
	MDELAY(50);

    return 1;
}

static unsigned int lcm_compare_id()
{
  unsigned int id = 0;
  unsigned int lcd_id=0;
  unsigned char buffer[4];
  unsigned int data_array[16];
  
  SET_RESET_PIN(1);  //TE:should reset LCM firstly
  MDELAY(1);
  SET_RESET_PIN(0);
  MDELAY(20);
  SET_RESET_PIN(1);
  MDELAY(120);
  #ifdef BUILD_LK
    DSI_clk_HS_mode(1);
  #endif
  MDELAY(10);
  #ifdef BUILD_LK
    DSI_clk_HS_mode(0);
  #endif
 //data_array[0]=0x00063902;
 //data_array[1]=0x52AA55F0; // SET password
 //data_array[2]=0x00000108; 
 push_table(lcm_read_lcm_compare_id, sizeof(lcm_read_lcm_compare_id) / sizeof(struct LCM_setting_table), 1);
 read_reg_v2(0xF4, buffer, 3);
 
 id = buffer[0]; //we only need ID 0x00 0x80 0x00
 #ifndef BUILD_LK
   printk("<6>zhangjun [lcm_compare_id %x,%x,%x\n",buffer[0],buffer[1],buffer[2]);
 #else
   printf("<6>zhangjun [lcm_compare_id %x,%x,%x\n",buffer[0],buffer[1],buffer[2]);
 #endif
	if(0x79!= id)
		return 0;

	mt_set_gpio_mode(GPIO_LCM_ID, GPIO_LCM_ID_M_GPIO);
	mt_set_gpio_dir(GPIO_LCM_ID, GPIO_DIR_IN);		
	mt_set_gpio_pull_enable(GPIO_LCM_ID,GPIO_PULL_DISABLE);
	mt_set_gpio_pull_select(GPIO_LCM_ID,GPIO_PULL_DOWN);
	MDELAY(1);
	lcd_id =  mt_get_gpio_in(GPIO_LCM_ID);
#ifndef BUILD_LK
	printk("<6>vendor jingtai [lcm_compare_id] lcd_id =%x\n",lcd_id);
#else
	printf("<6>vendor jingtai [lcm_compare_id] lcd_id =%x\n",lcd_id);
#endif

	if(lcd_id)
		return 1;
	else
		return 0;

}
LCM_DRIVER hx8379a_wvga_dsi_vdo_v815w_jt_ips_lcm_drv = 
{
    .name   = "hx8379a_wvga_dsi_vdo_v815w_jt_ips",
 .set_util_funcs = lcm_set_util_funcs,
 .get_params     = lcm_get_params,
 .init           = lcm_init,
 .suspend        = lcm_suspend,
 .resume         = lcm_resume,
 .compare_id    = lcm_compare_id, 
#if (LCM_DSI_CMD_MODE)
	.esd_check   = lcm_esd_check,
    .esd_recover   = lcm_esd_recover,
     .set_backlight = lcm_setbacklight,
    .update         = lcm_update,
#endif
};
 
 
