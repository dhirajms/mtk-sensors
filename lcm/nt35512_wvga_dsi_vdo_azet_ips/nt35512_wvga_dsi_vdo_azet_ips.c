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

#define REGFLAG_DELAY             							0XFE
#define REGFLAG_END_OF_TABLE      							0x00   // END OF REGISTERS MARKER

#define LCM_DSI_CMD_MODE									0

// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------

static LCM_UTIL_FUNCS lcm_util = {0};

#define SET_RESET_PIN(v)    								(lcm_util.set_reset_pin((v)))

#define UDELAY(n) 											(lcm_util.udelay(n))
#define MDELAY(n) 											(lcm_util.mdelay(n))

//static kal_bool IsFirstBoot = KAL_TRUE;

// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------

#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)	lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)										lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)					lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg											lcm_util.dsi_read_reg()
#define read_reg_v2(cmd, buffer, buffer_size)   				lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)    

static struct LCM_setting_table {
    unsigned cmd;
    unsigned char count;
    unsigned char para_list[120];
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

//LV2 Page 1 enable
{ 0xF0,5,{0x55,0xAA,0x52,0x08,0x01}},

//AVDD Set AVDD 5.2V
{ 0xB0,3,{0x0D,0x0D,0x0D}},

//AVDD ratio
{ 0xB6,3,{0x44,0x44,0x44}},
 
//AVEE  -5.2V
{ 0xB1,3,{0x0D,0x0D,0x0D}},

//AVEE ratio
{ 0xB7,3,{0x34,0x34,0x34}},

//VCL  -2.5V
{ 0xB2,3,{0x00,0x00,0x00}},

//VCL ratio
{ 0xB8,3,{0x24,0x24,0x24}},


//VGH 15V  (Free pump)
{ 0xBF,1,{0x01}},
{ 0xB3,3,{0x0F,0x0F,0x0F}},

//VGH ratio
{ 0xB9,3,{0x34,0x34,0x34}},

//VGL_REG -10V
{ 0xB5,3,{0x08,0x08,0x08}},

{ 0xC2,1,{0x03}},

//VGLX ratio
{ 0xBA,3,{0x34,0x34,0x34}},

//VGMP/VGSP 4.5V/0V
{ 0xBC,3,{0x00,0x78,0x00}},

//VGMN/VGSN -4.5V/0V
{ 0xBD,3,{0x00,0x78,0x00}},

//VCOM  
{ 0xBE,2,{0x00,0x69}},

//Gamma Setting
//R(+) MCR cmd
{ 0xD1,52,{0x00,0x01,0x00,0x02,0x00,0x0E,0x00,0x25,0x00,0x3F,0x00,0x71,0x00,0x9D,0x00,0xDF,0x01,0x0E,0x01,0x52,0x01,0x82,0x01,0xC7,0x01,0xFE,0x02,0x00,0x02,0x2C,0x02,0x5A,0x02,0x75,0x02,0x93,0x02,0xA6,0x02,0xBD,0x02,0xCC,0x02,0xE0,0x02,0xEE,0x03,0x03,0x03,0x37,0x03,0xFF}},

//G(+) MCR cmd
{ 0xD2,52,{0x00,0x01,0x00,0x02,0x00,0x0E,0x00,0x25,0x00,0x3F,0x00,0x71,0x00,0x9D,0x00,0xDF,0x01,0x0E,0x01,0x52,0x01,0x82,0x01,0xC7,0x01,0xFE,0x02,0x00,0x02,0x2C,0x02,0x5A,0x02,0x75,0x02,0x93,0x02,0xA6,0x02,0xBD,0x02,0xCC,0x02,0xE0,0x02,0xEE,0x03,0x03,0x03,0x37,0x03,0xFF}},

//B(+) MCR cmd
{ 0xD3,52,{0x00,0x01,0x00,0x02,0x00,0x0E,0x00,0x25,0x00,0x3F,0x00,0x71,0x00,0x9D,0x00,0xDF,0x01,0x0E,0x01,0x52,0x01,0x82,0x01,0xC7,0x01,0xFE,0x02,0x00,0x02,0x2C,0x02,0x5A,0x02,0x75,0x02,0x93,0x02,0xA6,0x02,0xBD,0x02,0xCC,0x02,0xE0,0x02,0xEE,0x03,0x03,0x03,0x37,0x03,0xFF}},

//R(-) MCR cmd
{ 0xD4,52,{0x00,0x01,0x00,0x02,0x00,0x0E,0x00,0x25,0x00,0x3F,0x00,0x71,0x00,0x9D,0x00,0xDF,0x01,0x0E,0x01,0x52,0x01,0x82,0x01,0xC7,0x01,0xFE,0x02,0x00,0x02,0x2C,0x02,0x5A,0x02,0x75,0x02,0x93,0x02,0xA6,0x02,0xBD,0x02,0xCC,0x02,0xE0,0x02,0xEE,0x03,0x03,0x03,0x37,0x03,0xFF}},

//G(-) MCR cmd
{ 0xD5,52,{0x00,0x01,0x00,0x02,0x00,0x0E,0x00,0x25,0x00,0x3F,0x00,0x71,0x00,0x9D,0x00,0xDF,0x01,0x0E,0x01,0x52,0x01,0x82,0x01,0xC7,0x01,0xFE,0x02,0x00,0x02,0x2C,0x02,0x5A,0x02,0x75,0x02,0x93,0x02,0xA6,0x02,0xBD,0x02,0xCC,0x02,0xE0,0x02,0xEE,0x03,0x03,0x03,0x37,0x03,0xFF}},

//B(-) MCR cmd
{ 0xD6,52,{0x00,0x01,0x00,0x02,0x00,0x0E,0x00,0x25,0x00,0x3F,0x00,0x71,0x00,0x9D,0x00,0xDF,0x01,0x0E,0x01,0x52,0x01,0x82,0x01,0xC7,0x01,0xFE,0x02,0x00,0x02,0x2C,0x02,0x5A,0x02,0x75,0x02,0x93,0x02,0xA6,0x02,0xBD,0x02,0xCC,0x02,0xE0,0x02,0xEE,0x03,0x03,0x03,0x37,0x03,0xFF}},

//LV2 Page 0 enable
{ 0xF0,5,{0x55,0xAA,0x52,0x08,0x00}},

//480x854
{ 0xB5, 1,{0x6B}},

//Display control
{ 0xB1, 2,{0xFC, 0x00}},

//Source hold time
{ 0xB6,1,{0x05}},

//Gate EQ control
{ 0xB7,2,{0x70,0x70}},

//Source EQ control (Mode 2)
{ 0xB8,4,{0x01,0x03,0x03,0x03}},

//Inversion mode  (2-dot)
{ 0xBC,3,{0x02,0x00,0x00}},

//Frame rate
{ 0xBD,5,{0x01,0x6C,0x1E,0x1D,0x00}},

//Timing control 4H w/ 4-delay 
{ 0xC9,5,{0xD0,0x02,0x50,0x50,0x50}},

{ 0x36,1,{0x00}},
{ 0x35,1,{0x00}},

{0x11,1, {0x00}},
{REGFLAG_DELAY, 125, {}},
 // Display ON
{0x29,1, 	{0x00}},
{REGFLAG_DELAY, 20, {}},

	// Note
	// Strongly recommend not to set Sleep out / Display On here. That will cause messed frame to be shown as later the backlight is on.

	// Setting ending by predefined flag
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};



static struct LCM_setting_table lcm_sleep_out_setting[] = {
    // Sleep Out
	{0x11, 0, {0x00}},
    {REGFLAG_DELAY, 120, {}},

    // Display ON
	{0x29, 0, {0x00}},
    {REGFLAG_DELAY, 100, {}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};


static struct LCM_setting_table lcm_deep_sleep_mode_in_setting[] = {
	// Display off sequence
	{0x28, 0, {0x00}},

    // Sleep Mode On
	{0x10, 0, {0x00}},

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
				//MDELAY(10);//soso add or it will fail to send register
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


		params->dsi.mode   = SYNC_EVENT_VDO_MODE;
	
		// DSI
		/* Command mode setting */
		params->dsi.LANE_NUM				= LCM_TWO_LANE;
		//The following defined the fomat for data coming from LCD engine.
		params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
		params->dsi.data_format.trans_seq   = LCM_DSI_TRANS_SEQ_MSB_FIRST;
		params->dsi.data_format.padding     = LCM_DSI_PADDING_ON_LSB;
		params->dsi.data_format.format      = LCM_DSI_FORMAT_RGB888;

		// Highly depends on LCD driver capability.
		// Not support in MT6573
		params->dsi.packet_size=256;

		// Video mode setting		
		params->dsi.intermediat_buffer_num = 2;

		params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;

		params->dsi.vertical_sync_active				= 2;
		params->dsi.vertical_backporch					= 50;//50
		params->dsi.vertical_frontporch					= 20;//20
		params->dsi.vertical_active_line				= FRAME_HEIGHT; 

		params->dsi.horizontal_sync_active				= 2;
		params->dsi.horizontal_backporch				= 100;
		params->dsi.horizontal_frontporch				= 100;
		params->dsi.horizontal_active_pixel				= FRAME_WIDTH;


		// Bit rate calculation
		params->dsi.pll_div1=1;		// div1=0,1,2,3;div1_real=1,2,4,4
		params->dsi.pll_div2=1;		// div2=0,1,2,3;div2_real=1,2,4,4
		params->dsi.fbk_div =28;//0x27		// fref=26MHz, fvco=fref*(fbk_div+1)*2/(div1_real*div2_real)		
}

static void lcm_init(void)
{
	unsigned int data_array[64];

    SET_RESET_PIN(1);
    SET_RESET_PIN(0);
    MDELAY(10);//Must > 10ms
    SET_RESET_PIN(1);
    MDELAY(120);//Must > 120ms
    
	//IsFirstBoot = KAL_TRUE;

#if 0
	// 1
#else
	push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);
#endif
}


static void lcm_suspend(void)
{
    SET_RESET_PIN(1);
    SET_RESET_PIN(0);
    MDELAY(20);//Must > 10ms
    SET_RESET_PIN(1);
    MDELAY(150);//Must > 120ms

	push_table(lcm_deep_sleep_mode_in_setting, sizeof(lcm_deep_sleep_mode_in_setting) / sizeof(struct LCM_setting_table), 1);
}


static void lcm_resume(void)
{
	//lcm_compare_id();

	lcm_init();
	
	push_table(lcm_sleep_out_setting, sizeof(lcm_sleep_out_setting) / sizeof(struct LCM_setting_table), 1);
}
static unsigned int lcm_compare_id(void)
{
	unsigned int id = 0;
	unsigned char buffer[3];
	unsigned int array[16];
	
	SET_RESET_PIN(1);  //NOTE:should reset LCM firstly
	SET_RESET_PIN(0);
	MDELAY(10);
	SET_RESET_PIN(1);
	MDELAY(120);

	array[0] = 0x00033700;// read id return two byte,version and id
	dsi_set_cmdq(array, 1, 1);
	read_reg_v2(0x04, buffer, 3);
	id = buffer[1]; //we only need ID
#if defined(BUILD_LK)
	/*The Default Value should be 0x00,0x80,0x00*/
	printf("\n\n\n\n[soso]%s, id0 = 0x%08x,id1 = 0x%08x,id2 = 0x%08x\n", __func__, buffer[0],buffer[1],buffer[2]);
#endif
    return (id == 0x80)?1:0;
}


LCM_DRIVER nt35512_wvga_dsi_vdo_azet_ips_lcm_drv = 
{
    .name			= "nt35512_wvga_dsi_vdo_azet_ips",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params     = lcm_get_params,
	.init           = lcm_init,
	.suspend        = lcm_suspend,
	.resume         = lcm_resume,
	//.compare_id    = lcm_compare_id,
};

