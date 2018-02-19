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
 
static void lcm_initialization_setting(void)
{
  unsigned int data_array[16];
 // static unsigned char temp_data = 0x84;
  data_array[0] = 0x00043902;                          
  data_array[1] = 0x7983FFb9;
  dsi_set_cmdq(&data_array, 2, 1);
  //MDELAY(1);
  data_array[0] = 0x00033902;                          
  data_array[1] = 0x009351BA;
  dsi_set_cmdq(&data_array, 2, 1);
  //MDELAY(1);
  data_array[0] = 0x00303902;                          
  data_array[1] = 0x0A0000D5;
  data_array[2] = 0x00000100;
  data_array[3] = 0x99881000;
  data_array[4] = 0x88103210;
  data_array[5] = 0x88888888;
  data_array[6] = 0x88888888;
  data_array[7] = 0x88888888;
  data_array[8] = 0x99010123;
  data_array[9] = 0x88888888;
  data_array[10] = 0x88888888;
  data_array[11] = 0x00048888;
  data_array[12] = 0x00000000;
  dsi_set_cmdq(&data_array, 13, 1);
  //MDELAY(1);
  data_array[0] = 0x00143902;                          
  data_array[1] = 0x445000B1;
  data_array[2] = 0x11086fE3;
  data_array[3] = 0x38361111;//vrhp,vrhn 0x3028  0x2c247412
  //data_array[3] = 0x372F7818;
  data_array[4] = 0x0B4229A9;
  data_array[5] = 0xE600F166;
  dsi_set_cmdq(&data_array, 6, 1);
//MDELAY(1);
  data_array[0] = 0x000E3902;                          
  data_array[1] = 0x3C0000B2;
  data_array[2] = 0x2218040c;
  data_array[3] = 0x040cFF00;
  data_array[4] = 0x00002018;
  dsi_set_cmdq(&data_array, 5, 1); 
  
  data_array[0] = 0x00203902;                          
 // data_array[1] = 0x000880B4;//column inversion
  data_array[1] = 0x000c80B4;//2 dot inversion
  data_array[2] = 0x00061030;
  data_array[3] = 0x00000000;
  data_array[4] = 0x48001100;
  
  //data_array[5] = 0x3c0c1702;
  //data_array[6] = 0x073c3e0a; 
  data_array[5] = 0x403c2307;
  data_array[6] = 0x04303008;
  data_array[7] = 0x28084000;
  data_array[8] = 0x04303008;
  dsi_set_cmdq(&data_array, 9, 1);
  
  data_array[0] = 0x00023902;                          
  data_array[1] = 0x000002CC; 
  dsi_set_cmdq(&data_array, 2, 1); 

  data_array[0] = 0x00053902;                          
  data_array[1] = 0x007b00B6; ///////////aa a3
  data_array[2] = 0x0000007b;
  dsi_set_cmdq(&data_array, 3, 1);
	
 // data_array[0] = 0x00053902;
 // data_array[1] = 0x000000B6|((temp_data&0xFF)<<16);//8a
 // data_array[2] = 0x00000000|(temp_data&0xFF);
 // #ifndef BUILD_LK
 // printk("hx8379a init debug %x ,%x\n",data_array[1],data_array[2]);
 // #endif
 // dsi_set_cmdq(&data_array, 3, 1);
 // temp_data-=1;
//MDELAY(1);
  data_array[0] = 0x00243902;                          
  data_array[1] = 0x080079E0;
  data_array[2] = 0x3f3a330e;
  data_array[3] = 0x07014b1e;
  data_array[4] = 0x1817150e;
  data_array[5] = 0x001a1f17;
  data_array[6] = 0x3a330e08;
  data_array[7] = 0x014b1e3f;
  data_array[8] = 0x17150e07;
  data_array[9] = 0x1a1f1718;
  dsi_set_cmdq(&data_array, 10, 1);
  MDELAY(5);
  
  data_array[0] = 0x00023902;  
  data_array[1] = 0x0000773A; // RGB 18bits D[17:0]
  dsi_set_cmdq(&data_array, 2, 1);
  
  data_array[0] = 0x00023902;  
  data_array[1] = 0x00000036; // RGB 18bits D[17:0]
  dsi_set_cmdq(&data_array, 2, 1);
  
  data_array[0] = 0x00110500; // Sleep Out
  dsi_set_cmdq(&data_array, 1, 1);
  MDELAY(120);
  data_array[0] = 0x00290500;  
  dsi_set_cmdq(&data_array, 1, 1);
  MDELAY(10);
  
};
 
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
  params->dsi.horizontal_frontporch    =60;//32;//35; //20;//50;//36;//30;//28;//37;
  params->dsi.horizontal_active_pixel   = FRAME_WIDTH;//496
 
  params->dsi.compatibility_for_nvk = 0;		// this parameter would be set to 1 if DriverIC is NTK's and when force match DSI clock for NTK's
  
  params->dsi.pll_div1=1;		// div1=0,1,2,3;div1_real=1,2,4,4
  params->dsi.pll_div2=1;		// div2=0,1,2,3;div2_real=1,2,4,4
  params->dsi.fbk_div =31;   // 20

 

 
}
 

static void  lcm_init(void)
{

	
 SET_RESET_PIN(1); 
 MDELAY(1);
 SET_RESET_PIN(0);
 MDELAY(20);//10 
 SET_RESET_PIN(1);
 MDELAY(20); // 150
 
 lcm_initialization_setting();


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
    lcm_initialization_setting();
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
		return 0;
	else
		return 1;

}
LCM_DRIVER hx8379a_wvga_dsi_vdo_v815w_txd_ips_lcm_drv = 
{
    .name   = "hx8379a_wvga_dsi_vdo_v815w_txd_ips",
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
 
 
