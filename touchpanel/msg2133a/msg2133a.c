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


#include "tpd.h"
#include <linux/interrupt.h>
#include <cust_eint.h>
#include <linux/i2c.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/rtpm_prio.h>
#include <linux/wait.h>
#include <linux/time.h>
#include <linux/delay.h>
#include <linux/input.h>

#include <mach/mt_pm_ldo.h>
#include <mach/mt_typedefs.h>
#include <mach/mt_boot.h>
#include <mach/eint.h>
#include <linux/miscdevice.h>
#include "cust_gpio_usage.h"
#include"tpd_custom_mstar.h"

#include <linux/mount.h>
#include <linux/netdevice.h>
#include <linux/proc_fs.h>

#define  GPIO_CTP_EN_PIN   GPIO22
#define  GPIO_CTP_EN_PIN_M_GPIO  GPIO_MODE_00 


#define TPD_HAVE_BUTTON


unsigned char MSG_FIRMWARE_JD[94*1024] =
{
  #include "Yarism_jd_2138A_V2.006_131210.h"
};


unsigned char MSG_FIRMWARE_EA[94*1024] =
{
  //#include "Soul3.5_2133A_Each_V5.02_140221.h"
  //#include "Soul3.5_2133A_Each_V5.03_140325.h"
  //#include "Soul3.5_2133A_Each_V5.06_140404.h"
  //#include "Soul3.5_2133A_Each_V5.07_140417.h"
  //#include "Soul3.5_2133A_Each_V5.08_140507.h"
  //#include "Soul3.5_2133A_Each_V5.09_140513.h"
  //#include "Soul3.5_2133A_Each_V5.11_140528.h"
    #include "Soul3.5_2133A_Each_V5.13_140607.h"
};

volatile static u8 Fmr_Loader[1024];
 
extern struct tpd_device *tpd;

enum wk_wdt_type {
	WK_WDT_LOC_TYPE,
	WK_WDT_EXT_TYPE,
	WK_WDT_LOC_TYPE_NOLOCK,
	WK_WDT_EXT_TYPE_NOLOCK,
	
};
extern void mtk_wdt_restart(enum wk_wdt_type type);
 
struct i2c_client *i2c_clientma = NULL;
struct task_struct *mthread = NULL;
 
static DECLARE_WAIT_QUEUE_HEAD(waiter);
 
 
static void tpd_eint_interrupt_handler(void);
 
 /*
 extern void mt65xx_eint_unmask(unsigned int line);
 extern void mt65xx_eint_mask(unsigned int line);
 extern void mt65xx_eint_set_hw_debounce(kal_uint8 eintno, kal_uint32 ms);
 extern kal_uint32 mt65xx_eint_set_sens(kal_uint8 eintno, kal_bool sens);
 extern void mt65xx_eint_registration(kal_uint8 eintno, kal_bool Dbounce_En,
									  kal_bool ACT_Polarity, void (EINT_FUNC_PTR)(void),
									  kal_bool auto_umask);
*/
 extern int Log_record_msg(unsigned int module_id,unsigned int exception_id,char *buf);

 
static int __devinit tpd_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int tpd_detect(struct i2c_client *client, int kind, struct i2c_board_info *info);
static int __devexit tpd_remove(struct i2c_client *client);
static int touch_event_handler(void *unused);
 
//---------------------------------------------------------------------------------------------
static int tpd_flag = 0;

#define u8         unsigned char
#define u32        unsigned int
#define s32        signed int

#define MAX_TOUCH_FINGER 2
typedef struct
{
    u16 X;
    u16 Y;
} TouchPoint_t;

typedef struct
{
    u8 nTouchKeyMode;
    u8 nTouchKeyCode;
    u8 nFingerNum;
    TouchPoint_t Point[MAX_TOUCH_FINGER];
} TouchScreenInfo_t;


#define REPORT_PACKET_LENGTH    (8)
#ifdef SOUL4_KK_SUPPORT
#define MS_TS_MSG21XX_X_MAX   (480)
#define MS_TS_MSG21XX_Y_MAX   (800)
#else
 #define MS_TS_MSG21XX_X_MAX   (320)
#ifdef HZ_SOUL45_SUPPORT
    #define MS_TS_MSG21XX_Y_MAX   (480)
#else
    #define MS_TS_MSG21XX_Y_MAX   (480)
#endif
#endif

#define TPD_OK 0
#define TPD_DEVICEMA       "MSG2133A"
#if defined(CTP_CHECK_FM)
static int16_t fm_current_frequency=0;
#endif


static const struct i2c_device_id tpd_idm[] = {{TPD_DEVICEMA,0},{}};
//unsigned short forcem[] = {0,0x4c,I2C_CLIENT_END,I2C_CLIENT_END}; 
//static const unsigned short * const forcesm[] = { forcem, NULL };
//static struct i2c_client_address_data addr_datam = { .forces = forcesm, };
 static struct i2c_board_info __initdata i2c_tpd={ I2C_BOARD_INFO(TPD_DEVICEMA, (0x4C>>1))};
 
static struct i2c_driver tpd_i2c_driver =
{
  .driver = {
	 .name = TPD_DEVICEMA,
//	 .owner = THIS_MODULE,
  },
  .probe = tpd_probe,
  .remove = __devexit_p(tpd_remove),
  .id_table = tpd_idm,
  .detect = tpd_detect,
 // .address_data = &addr_datam,
};

#define	__FIRMWARE_UPDATE__
#ifdef __FIRMWARE_UPDATE__

#define FW_ADDR_MSG21XX   (0xC4>>1)
#define FW_ADDR_MSG21XX_TP   (0x4C>>1)
#define FW_UPDATE_ADDR_MSG21XX   (0x92>>1)
#define TP_DEBUG	printk
static  char *fw_version;
static u8 temp[94][1024];
static int FwDataCnt;

#if 0
static void HalTscrCReadI2CSeq(u8 addr, u8* read_data, u8 size)
{
   //according to your platform.
   	int rc;

	struct i2c_msg msgs[] =
    {
		{
			.addr = addr,
			.flags = I2C_M_RD,
			.len = size,
			.buf = read_data,
		},
	};

	rc = i2c_transfer(i2c_clientma->adapter, msgs, 1);
	if( rc < 0 )
    {
		printk("HalTscrCReadI2CSeq error %d\n", rc);
	}
}

static void HalTscrCDevWriteI2CSeq(u8 addr, u8* data, u16 size)
{
    //according to your platform.
   	int rc;

	struct i2c_msg msgs[] =
    {
		{
			.addr = addr,
			.flags = 0,
			.len = size,
			.buf = data,
		},
	};
	rc = i2c_transfer(i2c_clientma->adapter, msgs, 1);
	if( rc < 0 )
    {
		printk("HalTscrCDevWriteI2CSeq error %d,addr = %d\n", rc,addr);
	}
}

#endif
#ifdef DMA_IIC
#include <linux/dma-mapping.h>
static unsigned char *I2CDMABuf_va = NULL;
static volatile unsigned int I2CDMABuf_pa = NULL;
static void _msg_dma_alloc(void)
{
    I2CDMABuf_va = (u8 *)dma_alloc_coherent(NULL, 4096, &I2CDMABuf_pa, GFP_KERNEL);
}
static void _msg_dma_free(void)
{
    if(NULL!=I2CDMABuf_va)
    {
        dma_free_coherent(NULL, 4096, I2CDMABuf_va, I2CDMABuf_pa);
	    I2CDMABuf_va = NULL;
	    I2CDMABuf_pa = 0;
    }
}
#endif

/******************************
ITO
******************************/
#define ITO_TEST
#ifdef ITO_TEST

//modify:¸ù¾ÝÏîÄ¿ÐÞ¸Ä
#include <open_test_ANA1_yeji.h>
#include <open_test_ANA2_yeji.h>
#include <open_test_ANA1_B_yeji.h>
#include <open_test_ANA2_B_yeji.h>
#include <open_test_ANA3_yeji.h>
/*
#include <open_test_ANA1_lianchuang.h>
#include <open_test_ANA2_lianchuang.h>
#include <open_test_ANA1_B_lianchuang.h>
#include <open_test_ANA2_B_lianchuang.h>
#include <open_test_ANA3_lianchuang.h>
*/
///////////////////////////////////////////////////////////////////////////
u8 bItoTestDebug = 0;
#define ITO_TEST_DEBUG(format, ...) \
{ \
    if(bItoTestDebug) \
    { \
        printk(KERN_ERR "ito_test ***" format "\n", ## __VA_ARGS__); \
        mdelay(5); \
    } \
}
#define ITO_TEST_DEBUG_MUST(format, ...)	printk(KERN_ERR "ito_test ***" format "\n", ## __VA_ARGS__);mdelay(5)
//#define HQ_CTP_PS_TEST 1
s16  s16_raw_data_1[48] = {0};
s16  s16_raw_data_2[48] = {0};
s16  s16_raw_data_3[48] = {0};
u8 ito_test_keynum = 0;
u8 ito_test_dummynum = 0;
u8 ito_test_trianglenum = 0;
u8 ito_test_2r = 0;
u8 g_LTP = 1;	
uint16_t *open_1 = NULL;
uint16_t *open_1B = NULL;
uint16_t *open_2 = NULL;
uint16_t *open_2B = NULL;
uint16_t *open_3 = NULL;
u8 *MAP1 = NULL;
u8 *MAP2=NULL;
u8 *MAP3=NULL;
u8 *MAP40_1 = NULL;
u8 *MAP40_2 = NULL;
u8 *MAP40_3 = NULL;
u8 *MAP40_4 = NULL;
u8 *MAP41_1 = NULL;
u8 *MAP41_2 = NULL;
u8 *MAP41_3 = NULL;
u8 *MAP41_4 = NULL;

static U8 g_fail_channel[48] = {0};
static int fail_channel_count = 0;

#define ITO_TEST_ADDR_TP  (0x4C>>1)
#define ITO_TEST_ADDR_REG (0xC4>>1)
#define REG_INTR_FIQ_MASK           0x04
#define FIQ_E_FRAME_READY_MASK      ( 1 << 8 )
#define MAX_CHNL_NUM (48)
#define BIT0  (1<<0)
#define BIT1  (1<<1)
#define BIT5  (1<<5)
#define BIT11 (1<<11)
#define BIT15 (1<<15)

static int ito_test_i2c_read(U8 addr, U8* read_data, U16 size)//modify : ¸ù¾ÝÏîÄ¿ÐÞ¸Ä i2c_clientma
{
    int rc;
    U8 addr_before = i2c_clientma->addr;
    i2c_clientma->addr = addr;

    #ifdef DMA_IIC
    if(size>8&&NULL!=I2CDMABuf_va)
    {
        int i = 0;
        i2c_clientma->ext_flag = i2c_clientma->ext_flag | I2C_DMA_FLAG ;
        rc = i2c_master_recv(i2c_clientma, (unsigned char *)I2CDMABuf_pa, size);
        for(i = 0; i < size; i++)
   		{
        	read_data[i] = I2CDMABuf_va[i];
    	}
    }
    else
    {
        rc = i2c_master_recv(i2c_clientma, read_data, size);
    }
    i2c_clientma->ext_flag = i2c_clientma->ext_flag & (~I2C_DMA_FLAG);	
    #else
    rc = i2c_master_recv(i2c_clientma, read_data, size);
    #endif

    i2c_clientma->addr = addr_before;
    if( rc < 0 )
    {
        ITO_TEST_DEBUG_MUST("ito_test_i2c_read error %d,addr=%d\n", rc,addr);
    }
    return rc;
}

static int ito_test_i2c_write(U8 addr, U8* data, U16 size)//modify : ¸ù¾ÝÏîÄ¿ÐÞ¸Ä i2c_clientma
{
    int rc;
    U8 addr_before = i2c_clientma->addr;
    i2c_clientma->addr = addr;

#ifdef DMA_IIC
    if(size>8&&NULL!=I2CDMABuf_va)
	{
	    int i = 0;
	    for(i=0;i<size;i++)
    	{
    		 I2CDMABuf_va[i]=data[i];
    	}
		i2c_clientma->ext_flag = i2c_clientma->ext_flag | I2C_DMA_FLAG ;
		rc = i2c_master_send(i2c_clientma, (unsigned char *)I2CDMABuf_pa, size);
	}
	else
	{
		rc = i2c_master_send(i2c_clientma, data, size);
	}
    i2c_clientma->ext_flag = i2c_clientma->ext_flag & (~I2C_DMA_FLAG);	
#else
    rc = i2c_master_send(i2c_clientma, data, size);
#endif

    i2c_clientma->addr = addr_before;
    if( rc < 0 )
    {
        ITO_TEST_DEBUG_MUST("ito_test_i2c_write error %d,addr = %d,data[0]=%d\n", rc, addr,data[0]);
    }
    return rc;
}

static void ito_test_reset(void)//modify:¸ù¾ÝÏîÄ¿ÐÞ¸Ä
{
    ITO_TEST_DEBUG("reset tp\n");
    
	mt_set_gpio_mode(GPIO_CTP_EN_PIN, GPIO_CTP_EN_PIN_M_GPIO);
	mt_set_gpio_dir(GPIO_CTP_EN_PIN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_CTP_EN_PIN, GPIO_OUT_ONE);
	mt_set_gpio_out(GPIO_CTP_EN_PIN, GPIO_OUT_ZERO);
	mdelay(10);
	mt_set_gpio_out(GPIO_CTP_EN_PIN, GPIO_OUT_ONE);
	mdelay(50);
}

static void ito_test_disable_irq(void)//modify:¸ù¾ÝÏîÄ¿ÐÞ¸Ä
{
//	disable_irq_nosync(irq_msg21xx);
}

static void ito_test_enable_irq(void)//modify:¸ù¾ÝÏîÄ¿ÐÞ¸Ä
{
//	enable_irq(irq_msg21xx);
}

static void ito_test_set_iic_rate(u32 iicRate)//modify:¸ù¾ÝÆ½Ì¨ÐÞ¸Ä,iicËÙÂÊÒªÇó50K
{
	#ifdef CONFIG_I2C_SPRD//Õ¹Ñ¶Æ½Ì¨
        sprd_i2c_ctl_chg_clk(i2c_clientma->adapter->nr, iicRate);
        mdelay(100);
	#endif
    #ifdef MTK//MTKÆ½Ì¨
        i2c_clientma->timing = iicRate/1000;
    #endif
}

static void ito_test_WriteReg( u8 bank, u8 addr, u16 data )
{
    u8 tx_data[5] = {0x10, bank, addr, data & 0xFF, data >> 8};
    ito_test_i2c_write( ITO_TEST_ADDR_REG, &tx_data[0], 5 );
}

static void ito_test_WriteReg8Bit( u8 bank, u8 addr, u8 data )
{
    u8 tx_data[4] = {0x10, bank, addr, data};
    ito_test_i2c_write ( ITO_TEST_ADDR_REG, &tx_data[0], 4 );
}

static unsigned short ito_test_ReadReg( u8 bank, u8 addr )
{
    u8 tx_data[3] = {0x10, bank, addr};
    u8 rx_data[2] = {0};

    ito_test_i2c_write( ITO_TEST_ADDR_REG, &tx_data[0], 3 );
    ito_test_i2c_read ( ITO_TEST_ADDR_REG, &rx_data[0], 2 );
    return ( rx_data[1] << 8 | rx_data[0] );
}

static u32 ito_test_get_TpType(void)
{
    u8 tx_data[3] = {0};
    u8 rx_data[4] = {0};
    u32 Major = 0, Minor = 0;

    ITO_TEST_DEBUG("GetTpType\n");
        
    tx_data[0] = 0x53;
    tx_data[1] = 0x00;
    tx_data[2] = 0x2A;
    ito_test_i2c_write(ITO_TEST_ADDR_TP, &tx_data[0], 3);
    mdelay(50);
    ito_test_i2c_read(ITO_TEST_ADDR_TP, &rx_data[0], 4);
    Major = (rx_data[1]<<8) + rx_data[0];
    Minor = (rx_data[3]<<8) + rx_data[2];

    ITO_TEST_DEBUG("***TpTypeMajor = %d ***\n", Major);
    ITO_TEST_DEBUG("***TpTypeMinor = %d ***\n", Minor);
    
    return Major;
}

//modify:×¢Òâ¸ÃÏîÄ¿tpÊýÄ¿
#define TP_OF_LIANCHUANG    (2)
#define TP_OF_JUNDA         (4)

static u32 ito_test_choose_TpType(void)
{
    u32 tpType = 0;
    u8 i = 0;
    open_1 = NULL;
    open_1B = NULL;
    open_2 = NULL;
    open_2B = NULL;
    open_3 = NULL;
    MAP1 = NULL;
    MAP2 = NULL;
    MAP3 = NULL;
    MAP40_1 = NULL;
    MAP40_2 = NULL;
    MAP40_3 = NULL;
    MAP40_4 = NULL;
    MAP41_1 = NULL;
    MAP41_2 = NULL;
    MAP41_3 = NULL;
    MAP41_4 = NULL;
    ito_test_keynum = 0;
    ito_test_dummynum = 0;
    ito_test_trianglenum = 0;
    ito_test_2r = 0;

    for(i=0;i<10;i++)
    {
        tpType = ito_test_get_TpType();
        ITO_TEST_DEBUG("tpType=%d;i=%d;\n",tpType,i);
        if(TP_OF_LIANCHUANG==tpType
           ||TP_OF_JUNDA==tpType)//modify:×¢Òâ¸ÃÏîÄ¿tpÊýÄ¿
        {
            break;
        }
        else if(i<5)
        {
            mdelay(100);  
        }
        else
        {
            ito_test_reset();
        }
    }
  /*  
    if(TP_OF_LIANCHUANG==tpType)//modify:×¢Òâ¸ÃÏîÄ¿tpÊýÄ¿
    {
        open_1 = open_1_lianchuang;
        open_1B = open_1B_lianchuang;
        open_2 = open_2_lianchuang;
        open_2B = open_2B_lianchuang;
        open_3 = open_3_lianchuang;
        MAP1 = MAP1_lianchuang;
        MAP2 = MAP2_lianchuang;
        MAP3 = MAP3_lianchuang;
        MAP40_1 = MAP40_1_lianchuang;
        MAP40_2 = MAP40_2_lianchuang;
        MAP40_3 = MAP40_3_lianchuang;
        MAP40_4 = MAP40_4_lianchuang;
        MAP41_1 = MAP41_1_lianchuang;
        MAP41_2 = MAP41_2_lianchuang;
        MAP41_3 = MAP41_3_lianchuang;
        MAP41_4 = MAP41_4_lianchuang;
        ito_test_keynum = NUM_KEY_LIANCHUANG;
        ito_test_dummynum = NUM_DUMMY_LIANCHUANG;
        ito_test_trianglenum = NUM_SENSOR_LIANCHUANG;
        ito_test_2r = ENABLE_2R_LIANCHUANG;
    }
    else if(TP_OF_JUNDA==tpType)
    {
        open_1 = open_1_junda;
        open_1B = open_1B_junda;
        open_2 = open_2_junda;
        open_2B = open_2B_junda;
        open_3 = open_3_junda;
        MAP1 = MAP1_junda;
        MAP2 = MAP2_junda;
        MAP3 = MAP3_junda;
        MAP40_1 = MAP40_1_junda;
        MAP40_2 = MAP40_2_junda;
        MAP40_3 = MAP40_3_junda;
        MAP40_4 = MAP40_4_junda;
        MAP41_1 = MAP41_1_junda;
        MAP41_2 = MAP41_2_junda;
        MAP41_3 = MAP41_3_junda;
        MAP41_4 = MAP41_4_junda;
        ito_test_keynum = NUM_KEY_JUNDA;
        ito_test_dummynum = NUM_DUMMY_JUNDA;
        ito_test_trianglenum = NUM_SENSOR_JUNDA;
        ito_test_2r = ENABLE_2R_JUNDA;
    }
    else*/
    {
        open_1 = open_1_yeji;
        open_1B = open_1B_yeji;
        open_2 = open_2_yeji;
        open_2B = open_2B_yeji;
        open_3 = open_3_yeji;
        MAP1 = MAP1_yeji;
        MAP2 = MAP2_yeji;
        MAP3 = MAP3_yeji;
        MAP40_1 = MAP40_1_yeji;
        MAP40_2 = MAP40_2_yeji;
        MAP40_3 = MAP40_3_yeji;
        MAP40_4 = MAP40_4_yeji;
        MAP41_1 = MAP41_1_yeji;
        MAP41_2 = MAP41_2_yeji;
        MAP41_3 = MAP41_3_yeji;
        MAP41_4 = MAP41_4_yeji;
        ito_test_keynum = NUM_KEY_YEJI;
        ito_test_dummynum = NUM_DUMMY_YEJI;
        ito_test_trianglenum = NUM_SENSOR_YEJI;
        ito_test_2r = ENABLE_2R_YEJI;
    }
    return tpType;
}

static void ito_test_EnterSerialDebugMode(void)
{
    u8 data[5];

    data[0] = 0x53;
    data[1] = 0x45;
    data[2] = 0x52;
    data[3] = 0x44;
    data[4] = 0x42;
    ito_test_i2c_write(ITO_TEST_ADDR_REG, &data[0], 5);

    data[0] = 0x37;
    ito_test_i2c_write(ITO_TEST_ADDR_REG, &data[0], 1);

    data[0] = 0x35;
    ito_test_i2c_write(ITO_TEST_ADDR_REG, &data[0], 1);

    data[0] = 0x71;
    ito_test_i2c_write(ITO_TEST_ADDR_REG, &data[0], 1);
}

static uint16_t ito_test_get_num( void )
{
    uint16_t    num_of_sensor,i;
    uint16_t 	RegValue1,RegValue2;
 
    num_of_sensor = 0;
        
    RegValue1 = ito_test_ReadReg( 0x11, 0x4A); //bank:ana, addr:h0025  
    ITO_TEST_DEBUG("ito_test_get_num,RegValue1=%d\n",RegValue1);
    if ( ( RegValue1 & BIT1) == BIT1 )
    {
    	RegValue1 = ito_test_ReadReg( 0x12, 0x0A); //bank:ana2, addr:h0005  			
    	RegValue1 = RegValue1 & 0x0F;
    	
    	RegValue2 = ito_test_ReadReg( 0x12, 0x16); //bank:ana2, addr:h000b    		
    	RegValue2 = (( RegValue2 >> 1 ) & 0x0F) + 1;
    	
    	num_of_sensor = RegValue1 * RegValue2;
    }
	else
	{
	    for(i=0;i<4;i++)
	    {
	        num_of_sensor+=(ito_test_ReadReg( 0x12, 0x0A)>>(4*i))&0x0F; //bank:ana2, addr:h0005  
	    }
	}
    ITO_TEST_DEBUG("ito_test_get_num,num_of_sensor=%d\n",num_of_sensor);
    return num_of_sensor;        
}

static void ito_test_polling( void )
{
    uint16_t    reg_int = 0x0000;
    uint8_t     dbbus_tx_data[5];
    uint8_t     dbbus_rx_data[4];
    uint16_t    reg_value;


    reg_int = 0;

    ito_test_WriteReg( 0x13, 0x0C, BIT15 ); //bank:fir, addr:h0006         
    ito_test_WriteReg( 0x12, 0x14, (ito_test_ReadReg(0x12,0x14) | BIT0) ); //bank:ana2, addr:h000a        
            
    ITO_TEST_DEBUG("polling start\n");
    while( ( reg_int & BIT0 ) == 0x0000 )
    {
        dbbus_tx_data[0] = 0x10;
        dbbus_tx_data[1] = 0x3D; //bank:intr_ctrl, addr:h000c    
        dbbus_tx_data[2] = 0x18;
        ito_test_i2c_write(ITO_TEST_ADDR_REG, dbbus_tx_data, 3);
        ito_test_i2c_read(ITO_TEST_ADDR_REG,  dbbus_rx_data, 2);
        reg_int = dbbus_rx_data[1];
    }
    ITO_TEST_DEBUG("polling end\n");
    reg_value = ito_test_ReadReg( 0x3D, 0x18 ); 
    ito_test_WriteReg( 0x3D, 0x18, reg_value & (~BIT0) );      
}

static uint16_t ito_test_get_data_out( int16_t* s16_raw_data )
{
    uint8_t     i,dbbus_tx_data[8];
    uint16_t    raw_data[48]={0};
    uint16_t    num_of_sensor;
    uint16_t    reg_int;
    uint8_t		dbbus_rx_data[96]={0};
  
    num_of_sensor = ito_test_get_num();
    if(num_of_sensor*2>96)
    {
        ITO_TEST_DEBUG("danger,num_of_sensor=%d\n",num_of_sensor);
        return num_of_sensor;
    }

    reg_int = ito_test_ReadReg( 0x3d, REG_INTR_FIQ_MASK<<1 ); 
    ito_test_WriteReg( 0x3d, REG_INTR_FIQ_MASK<<1, (reg_int & (uint16_t)(~FIQ_E_FRAME_READY_MASK) ) ); 
    ito_test_polling();
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x13; //bank:fir, addr:h0020 
    dbbus_tx_data[2] = 0x40;
    ito_test_i2c_write(ITO_TEST_ADDR_REG, dbbus_tx_data, 3);
    mdelay(20);
    ito_test_i2c_read(ITO_TEST_ADDR_REG, &dbbus_rx_data[0], (num_of_sensor * 2));
    mdelay(100);
    for(i=0;i<num_of_sensor * 2;i++)
    {
        ITO_TEST_DEBUG("dbbus_rx_data[%d]=%d\n",i,dbbus_rx_data[i]);
    }
 
    reg_int = ito_test_ReadReg( 0x3d, REG_INTR_FIQ_MASK<<1 ); 
    ito_test_WriteReg( 0x3d, REG_INTR_FIQ_MASK<<1, (reg_int | (uint16_t)FIQ_E_FRAME_READY_MASK ) ); 


    for( i = 0; i < num_of_sensor; i++ )
    {
        raw_data[i] = ( dbbus_rx_data[ 2 * i + 1] << 8 ) | ( dbbus_rx_data[2 * i] );
        s16_raw_data[i] = ( int16_t )raw_data[i];
    }
    
    return(num_of_sensor);
}

static void ito_test_send_data_in( uint8_t step )
{
    uint16_t	i;
    uint8_t 	dbbus_tx_data[512];
    uint16_t 	*Type1=NULL;        

    ITO_TEST_DEBUG("ito_test_send_data_in step=%d\n",step);
	if( step == 4 )
    {
        Type1 = &open_1[0];        
    }
    else if( step == 5 )
    {
        Type1 = &open_2[0];      	
    }
    else if( step == 6 )
    {
        Type1 = &open_3[0];      	
    }
    else if( step == 9 )
    {
        Type1 = &open_1B[0];        
    }
    else if( step == 10 )
    {
        Type1 = &open_2B[0];      	
    } 
     
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x11; //bank:ana, addr:h0000
    dbbus_tx_data[2] = 0x00;    
    for( i = 0; i <= 0x3E ; i++ )
    {
        dbbus_tx_data[3+2*i] = Type1[i] & 0xFF;
        dbbus_tx_data[4+2*i] = ( Type1[i] >> 8 ) & 0xFF;    	
    }
    ito_test_i2c_write(ITO_TEST_ADDR_REG, dbbus_tx_data, 3+0x3F*2);
 
    dbbus_tx_data[2] = 0x7A * 2; //bank:ana, addr:h007a
    for( i = 0x7A; i <= 0x7D ; i++ )
    {
        dbbus_tx_data[3+2*(i-0x7A)] = 0;
        dbbus_tx_data[4+2*(i-0x7A)] = 0;    	    	
    }
    ito_test_i2c_write(ITO_TEST_ADDR_REG, dbbus_tx_data, 3+8);  
    
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x12; //bank:ana2, addr:h0005
      
    dbbus_tx_data[2] = 5 * 2;
    dbbus_tx_data[3] = Type1[128+5] & 0xFF;
    dbbus_tx_data[4] = ( Type1[128+5] >> 8 ) & 0xFF;
    ito_test_i2c_write(ITO_TEST_ADDR_REG, dbbus_tx_data, 5);
    
    dbbus_tx_data[2] = 0x0B * 2; //bank:ana2, addr:h000b
    dbbus_tx_data[3] = Type1[128+0x0B] & 0xFF;
    dbbus_tx_data[4] = ( Type1[128+0x0B] >> 8 ) & 0xFF;
    ito_test_i2c_write(ITO_TEST_ADDR_REG, dbbus_tx_data, 5);
    
    dbbus_tx_data[2] = 0x12 * 2; //bank:ana2, addr:h0012
    dbbus_tx_data[3] = Type1[128+0x12] & 0xFF;
    dbbus_tx_data[4] = ( Type1[128+0x12] >> 8 ) & 0xFF;
    ito_test_i2c_write(ITO_TEST_ADDR_REG, dbbus_tx_data, 5);
    
    dbbus_tx_data[2] = 0x15 * 2; //bank:ana2, addr:h0015
    dbbus_tx_data[3] = Type1[128+0x15] & 0xFF;
    dbbus_tx_data[4] = ( Type1[128+0x15] >> 8 ) & 0xFF;
    ito_test_i2c_write(ITO_TEST_ADDR_REG, dbbus_tx_data, 5);        
}

static void ito_test_set_v( uint8_t Enable, uint8_t Prs)	
{
    uint16_t    u16RegValue;        
    
    u16RegValue = ito_test_ReadReg( 0x12, 0x08); //bank:ana2, addr:h0004
    u16RegValue = u16RegValue & 0xF1; 							
    if ( Prs == 0 )
    {
    	ito_test_WriteReg( 0x12, 0x08, u16RegValue| 0x0C); 		
    }
    else if ( Prs == 1 )
    {
    	ito_test_WriteReg( 0x12, 0x08, u16RegValue| 0x0E); 		     	
    }
    else
    {
    	ito_test_WriteReg( 0x12, 0x08, u16RegValue| 0x02); 			
    }    
    
    if ( Enable )
    {
        u16RegValue = ito_test_ReadReg( 0x11, 0x06); //bank:ana, addr:h0003  
        ito_test_WriteReg( 0x11, 0x06, u16RegValue| 0x03);   	
    }
    else
    {
        u16RegValue = ito_test_ReadReg( 0x11, 0x06);    
        u16RegValue = u16RegValue & 0xFC;					
        ito_test_WriteReg( 0x11, 0x06, u16RegValue);         
    }
}

static void ito_test_set_c( uint8_t Csub_Step )
{
    uint8_t i;
    uint8_t dbbus_tx_data[MAX_CHNL_NUM+3];
    uint8_t HighLevel_Csub = false;
    uint8_t Csub_new;
     
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x11; //bank:ana, addr:h0042       
    dbbus_tx_data[2] = 0x84;        
    for( i = 0; i < MAX_CHNL_NUM; i++ )
    {
		Csub_new = Csub_Step;        
        HighLevel_Csub = false;   
        if( Csub_new > 0x1F )
        {
            Csub_new = Csub_new - 0x14;
            HighLevel_Csub = true;
        }
           
        dbbus_tx_data[3+i] = Csub_new & 0x1F;        
        if( HighLevel_Csub == true )
        {
            dbbus_tx_data[3+i] |= BIT5;
        }
    }
    ito_test_i2c_write(ITO_TEST_ADDR_REG, dbbus_tx_data, MAX_CHNL_NUM+3);

    dbbus_tx_data[2] = 0xB4; //bank:ana, addr:h005a        
    ito_test_i2c_write(ITO_TEST_ADDR_REG, dbbus_tx_data, MAX_CHNL_NUM+3);
}

static void ito_test_sw( void )
{
    ito_test_WriteReg( 0x11, 0x00, 0xFFFF ); //bank:ana, addr:h0000
    ito_test_WriteReg( 0x11, 0x00, 0x0000 );
    mdelay( 50 );
}

static void ito_test_first( uint8_t item_id , int16_t* s16_raw_data)		
{
    uint8_t     loop;
    uint8_t     i,j;
    int16_t     s16_raw_data_tmp[48]={0};
    uint8_t     num_of_sensor, num_of_sensor2,total_sensor = 0;
    uint16_t	u16RegValue;
    uint8_t 	*pMapping=NULL;
    
    num_of_sensor = 0;
    num_of_sensor2 = 0;	
	
    ITO_TEST_DEBUG("ito_test_first item_id=%d\n",item_id);
    // stop cpu
    ito_test_WriteReg( 0x0F, 0xE6, 0x01 ); //bank:mheg5, addr:h0073

    ito_test_WriteReg( 0x1E, 0x24, 0x0500 ); //bank:chip, addr:h0012
    ito_test_WriteReg( 0x1E, 0x2A, 0x0000 ); //bank:chip, addr:h0015
    ito_test_WriteReg( 0x1E, 0xE6, 0x6E00 ); //bank:chip, addr:h0073
    ito_test_WriteReg( 0x1E, 0xE8, 0x0071 ); //bank:chip, addr:h0074
	    
    if ( item_id == 40 )    			
    {
        pMapping = &MAP1[0];
        if ( ito_test_2r )
        {
            total_sensor = ito_test_trianglenum/2; 
        }
        else
        {
		        total_sensor = ito_test_trianglenum/2 + ito_test_keynum + ito_test_dummynum;
        }
    }
    else if( item_id == 41 )    		
    {
        pMapping = &MAP2[0];
        if ( ito_test_2r )
        {
            total_sensor = ito_test_trianglenum/2; 
        }
        else
        {
		        total_sensor = ito_test_trianglenum/2 + ito_test_keynum + ito_test_dummynum;
        }
    }
    else if( item_id == 42 )    		
    {
        pMapping = &MAP3[0];      
        total_sensor = ito_test_trianglenum + ito_test_keynum+ ito_test_dummynum; 
    }
        	    
    loop = 1;
    if ( item_id != 42 )
    {
	      if(total_sensor>11)
        {
            loop = 2;
        }
    }	
    
    ITO_TEST_DEBUG("loop=%d\n", loop);
	
    for ( i = 0; i < loop; i ++ )
    {
        if ( i == 0 )
        {
            ito_test_send_data_in( item_id - 36 );
        }
        else
        { 
            if ( item_id == 40 )
            { 
                ito_test_send_data_in( 9 );
            }
            else
            { 		
                ito_test_send_data_in( 10 );
            }
        }
	
        ito_test_set_v(1,0);    
        u16RegValue = ito_test_ReadReg( 0x11, 0x0E ); //bank:ana, addr:h0007   			
        ito_test_WriteReg( 0x11, 0x0E, u16RegValue | BIT11 );				 		
	
        if ( g_LTP == 1 )
        {
	    	    ito_test_set_c( 32 );
	    	}	    	
        else
        {	    	
	    	    ito_test_set_c( 0 );
        }
        
        ito_test_sw();
		
        if ( i == 0 )	 
        {      
            num_of_sensor=ito_test_get_data_out(  s16_raw_data_tmp );
            ITO_TEST_DEBUG("num_of_sensor=%d;\n",num_of_sensor);
        }
        else	
        {      
            num_of_sensor2=ito_test_get_data_out(  &s16_raw_data_tmp[num_of_sensor] );
            ITO_TEST_DEBUG("num_of_sensor=%d;num_of_sensor2=%d\n",num_of_sensor,num_of_sensor2);
        }
    }
    
    for ( j = 0; j < total_sensor; j ++ )
    {
        if ( g_LTP == 1 )
        {
            s16_raw_data[pMapping[j]] = s16_raw_data_tmp[j] + 4096;
        }
        else
        {
            s16_raw_data[pMapping[j]] = s16_raw_data_tmp[j];	
        }
    }	

    return;
}

typedef enum
{
	ITO_TEST_OK = 0,
	ITO_TEST_FAIL,
	ITO_TEST_GET_TP_TYPE_ERROR,
} ITO_TEST_RET;

ITO_TEST_RET ito_test_second (u8 item_id)
{
    ITO_TEST_RET ret = ITO_TEST_OK;
	u8 i = 0;
    
	s32  s16_raw_data_jg_tmp1 = 0;
	s32  s16_raw_data_jg_tmp2 = 0;
	s32  jg_tmp1_avg_Th_max =0;
	s32  jg_tmp1_avg_Th_min =0;
	s32  jg_tmp2_avg_Th_max =0;
	s32  jg_tmp2_avg_Th_min =0;

	u8  Th_Tri = 40;        
	u8  Th_bor = 40;        

	if ( item_id == 40 )    			
    {
        for (i=0; i<(ito_test_trianglenum/2)-2; i++)
        {
			s16_raw_data_jg_tmp1 += s16_raw_data_1[MAP40_1[i]];
		}
		for (i=0; i<2; i++)
        {
			s16_raw_data_jg_tmp2 += s16_raw_data_1[MAP40_2[i]];
		}
    }
    else if( item_id == 41 )    		
    {
        for (i=0; i<(ito_test_trianglenum/2)-2; i++)
        {
			s16_raw_data_jg_tmp1 += s16_raw_data_2[MAP41_1[i]];
		}
		for (i=0; i<2; i++)
        {
			s16_raw_data_jg_tmp2 += s16_raw_data_2[MAP41_2[i]];
		}
    }

	    jg_tmp1_avg_Th_max = (s16_raw_data_jg_tmp1 / ((ito_test_trianglenum/2)-2)) * ( 100 + Th_Tri) / 100 ;
	    jg_tmp1_avg_Th_min = (s16_raw_data_jg_tmp1 / ((ito_test_trianglenum/2)-2)) * ( 100 - Th_Tri) / 100 ;
        jg_tmp2_avg_Th_max = (s16_raw_data_jg_tmp2 / 2) * ( 100 + Th_bor) / 100 ;
	    jg_tmp2_avg_Th_min = (s16_raw_data_jg_tmp2 / 2 ) * ( 100 - Th_bor) / 100 ;
	
        ITO_TEST_DEBUG("item_id=%d;sum1=%d;max1=%d;min1=%d;sum2=%d;max2=%d;min2=%d\n",item_id,s16_raw_data_jg_tmp1,jg_tmp1_avg_Th_max,jg_tmp1_avg_Th_min,s16_raw_data_jg_tmp2,jg_tmp2_avg_Th_max,jg_tmp2_avg_Th_min);

	if ( item_id == 40 ) 
	{
		for (i=0; i<(ito_test_trianglenum/2)-2; i++)
		{
			if (s16_raw_data_1[MAP40_1[i]] > jg_tmp1_avg_Th_max || s16_raw_data_1[MAP40_1[i]] < jg_tmp1_avg_Th_min)
			{
				g_fail_channel[fail_channel_count] = MAP40_1[i];
				fail_channel_count ++; 
				ret = ITO_TEST_FAIL;
			}
		}
		
		for (i=0; i<(ito_test_trianglenum/2)-3; i++)//modify: 
		{
            if (s16_raw_data_1[MAP40_1[i]] > s16_raw_data_1[MAP40_1[i+1]])
            { 
                g_fail_channel[fail_channel_count] = MAP40_1[i];
                fail_channel_count ++; 
                ret = ITO_TEST_FAIL;
            }
		}
		
		for (i=0; i<2; i++)
		{
			if (s16_raw_data_1[MAP40_2[i]] > jg_tmp2_avg_Th_max || s16_raw_data_1[MAP40_2[i]] < jg_tmp2_avg_Th_min) 
			{
				g_fail_channel[fail_channel_count] = MAP40_2[i];
				fail_channel_count ++; 
				ret = ITO_TEST_FAIL;
			}
		} 
	}

	if ( item_id == 41 ) 
	{
		for (i=0; i<(ito_test_trianglenum/2)-2; i++)
		{
			if (s16_raw_data_2[MAP41_1[i]] > jg_tmp1_avg_Th_max || s16_raw_data_2[MAP41_1[i]] < jg_tmp1_avg_Th_min) 
			{
				g_fail_channel[fail_channel_count] = MAP41_1[i];
				fail_channel_count ++; 
				ret = ITO_TEST_FAIL;
			}	
		}
    
        for (i=0; i<(ito_test_trianglenum/2)-3; i++)//modify: 
        {
            if (s16_raw_data_2[MAP41_1[i]] < s16_raw_data_2[MAP41_1[i+1]])
            { 
                g_fail_channel[fail_channel_count] = MAP41_1[i];
                fail_channel_count ++; 
                ret = ITO_TEST_FAIL;
            }
        }

		for (i=0; i<2; i++)
		{
			if (s16_raw_data_2[MAP41_2[i]] > jg_tmp2_avg_Th_max || s16_raw_data_2[MAP41_2[i]] < jg_tmp2_avg_Th_min)
			{ 
				g_fail_channel[fail_channel_count] = MAP41_2[i];
				fail_channel_count ++; 
				ret = ITO_TEST_FAIL;
			}
		} 
	}

	return ret;
}

ITO_TEST_RET ito_test_second_2r (u8 item_id)
{
    ITO_TEST_RET ret = ITO_TEST_OK;
	u8 i = 0;
    
	s32  s16_raw_data_jg_tmp1 = 0;
	s32  s16_raw_data_jg_tmp2 = 0;
	s32  s16_raw_data_jg_tmp3 = 0;
	s32  s16_raw_data_jg_tmp4 = 0;
	
	s32  jg_tmp1_avg_Th_max =0;
	s32  jg_tmp1_avg_Th_min =0;
	s32  jg_tmp2_avg_Th_max =0;
	s32  jg_tmp2_avg_Th_min =0;
	s32  jg_tmp3_avg_Th_max =0;
	s32  jg_tmp3_avg_Th_min =0;
	s32  jg_tmp4_avg_Th_max =0;
	s32  jg_tmp4_avg_Th_min =0;

	u8  Th_Tri = 40;    // non-border threshold    
	u8  Th_bor = 40;    // border threshold    

	if ( item_id == 40 )    			
  {
    for (i=0; i<(ito_test_trianglenum/4)-2; i++)
    {
      s16_raw_data_jg_tmp1 += s16_raw_data_1[MAP40_1[i]];  //first region: non-border 
		}
		
		for (i=0; i<2; i++)
		{
		  s16_raw_data_jg_tmp2 += s16_raw_data_1[MAP40_2[i]];  //first region: border
		}

		for (i=0; i<(ito_test_trianglenum/4)-2; i++)
		{
		  s16_raw_data_jg_tmp3 += s16_raw_data_1[MAP40_3[i]];  //second region: non-border
		}
		
		for (i=0; i<2; i++)
		{
		  s16_raw_data_jg_tmp4 += s16_raw_data_1[MAP40_4[i]];  //second region: border
		}
  }
  else if( item_id == 41 )    		
  {
		for (i=0; i<(ito_test_trianglenum/4)-2; i++)
		{
		  s16_raw_data_jg_tmp1 += s16_raw_data_2[MAP41_1[i]];  //first region: non-border
		}
		
		for (i=0; i<2; i++)
		{
		  s16_raw_data_jg_tmp2 += s16_raw_data_2[MAP41_2[i]];  //first region: border
		}
		
		for (i=0; i<(ito_test_trianglenum/4)-2; i++)
		{
		  s16_raw_data_jg_tmp3 += s16_raw_data_2[MAP41_3[i]];  //second region: non-border
		}
		
		for (i=0; i<2; i++)
		{
		  s16_raw_data_jg_tmp4 += s16_raw_data_2[MAP41_4[i]];  //second region: border
		}
	}

	    jg_tmp1_avg_Th_max = (s16_raw_data_jg_tmp1 / ((ito_test_trianglenum/4)-2)) * ( 100 + Th_Tri) / 100 ;
	    jg_tmp1_avg_Th_min = (s16_raw_data_jg_tmp1 / ((ito_test_trianglenum/4)-2)) * ( 100 - Th_Tri) / 100 ;
        jg_tmp2_avg_Th_max = (s16_raw_data_jg_tmp2 / 2) * ( 100 + Th_bor) / 100 ;
	    jg_tmp2_avg_Th_min = (s16_raw_data_jg_tmp2 / 2) * ( 100 - Th_bor) / 100 ;
		jg_tmp3_avg_Th_max = (s16_raw_data_jg_tmp3 / ((ito_test_trianglenum/4)-2)) * ( 100 + Th_Tri) / 100 ;
	    jg_tmp3_avg_Th_min = (s16_raw_data_jg_tmp3 / ((ito_test_trianglenum/4)-2)) * ( 100 - Th_Tri) / 100 ;
        jg_tmp4_avg_Th_max = (s16_raw_data_jg_tmp4 / 2) * ( 100 + Th_bor) / 100 ;
	    jg_tmp4_avg_Th_min = (s16_raw_data_jg_tmp4 / 2) * ( 100 - Th_bor) / 100 ;
		
	
        ITO_TEST_DEBUG("item_id=%d;sum1=%d;max1=%d;min1=%d;sum2=%d;max2=%d;min2=%d;sum3=%d;max3=%d;min3=%d;sum4=%d;max4=%d;min4=%d;\n",item_id,s16_raw_data_jg_tmp1,jg_tmp1_avg_Th_max,jg_tmp1_avg_Th_min,s16_raw_data_jg_tmp2,jg_tmp2_avg_Th_max,jg_tmp2_avg_Th_min,s16_raw_data_jg_tmp3,jg_tmp3_avg_Th_max,jg_tmp3_avg_Th_min,s16_raw_data_jg_tmp4,jg_tmp4_avg_Th_max,jg_tmp4_avg_Th_min);


	if ( item_id == 40 ) 
	{
		for (i=0; i<(ito_test_trianglenum/4)-2; i++)
		{
			if (s16_raw_data_1[MAP40_1[i]] > jg_tmp1_avg_Th_max || s16_raw_data_1[MAP40_1[i]] < jg_tmp1_avg_Th_min) 
			{
				g_fail_channel[fail_channel_count] = MAP40_1[i];
				fail_channel_count ++; 
				ret = ITO_TEST_FAIL;
			}
		}
		
		for (i=0; i<2; i++)
		{
			if (s16_raw_data_1[MAP40_2[i]] > jg_tmp2_avg_Th_max || s16_raw_data_1[MAP40_2[i]] < jg_tmp2_avg_Th_min) 
			{
				g_fail_channel[fail_channel_count] = MAP40_2[i];				
				fail_channel_count ++; 
				ret = ITO_TEST_FAIL;
			}	
		} 
		
		for (i=0; i<(ito_test_trianglenum/4)-2; i++)
		{
			if (s16_raw_data_1[MAP40_3[i]] > jg_tmp3_avg_Th_max || s16_raw_data_1[MAP40_3[i]] < jg_tmp3_avg_Th_min) 
			{
				g_fail_channel[fail_channel_count] = MAP40_3[i];
				fail_channel_count ++; 
				ret = ITO_TEST_FAIL;
			}
		}
		
		for (i=0; i<2; i++)
		{
			if (s16_raw_data_1[MAP40_4[i]] > jg_tmp4_avg_Th_max || s16_raw_data_1[MAP40_4[i]] < jg_tmp4_avg_Th_min) 
			{
				g_fail_channel[fail_channel_count] = MAP40_4[i];
				fail_channel_count ++; 
				ret = ITO_TEST_FAIL;
			}
		} 
	}

	if ( item_id == 41 ) 
	{
		for (i=0; i<(ito_test_trianglenum/4)-2; i++)
		{
			if (s16_raw_data_2[MAP41_1[i]] > jg_tmp1_avg_Th_max || s16_raw_data_2[MAP41_1[i]] < jg_tmp1_avg_Th_min) 
			{
				g_fail_channel[fail_channel_count] = MAP41_1[i];
				fail_channel_count ++; 
				ret = ITO_TEST_FAIL;
			}	
		}
		
		for (i=0; i<2; i++)
		{
			if (s16_raw_data_2[MAP41_2[i]] > jg_tmp2_avg_Th_max || s16_raw_data_2[MAP41_2[i]] < jg_tmp2_avg_Th_min) 
			{
				g_fail_channel[fail_channel_count] = MAP41_2[i];
				fail_channel_count ++; 
				ret = ITO_TEST_FAIL;
			}
		}
		
		for (i=0; i<(ito_test_trianglenum/4)-2; i++)
		{
			if (s16_raw_data_2[MAP41_3[i]] > jg_tmp3_avg_Th_max || s16_raw_data_2[MAP41_3[i]] < jg_tmp3_avg_Th_min) 
			{	
				g_fail_channel[fail_channel_count] = MAP41_3[i];
				fail_channel_count ++; 
				ret = ITO_TEST_FAIL;
			}	
		}
		
		for (i=0; i<2; i++)
		{
			if (s16_raw_data_2[MAP41_4[i]] > jg_tmp4_avg_Th_max || s16_raw_data_2[MAP41_4[i]] < jg_tmp4_avg_Th_min) 
			{
				g_fail_channel[fail_channel_count] = MAP41_4[i];
				fail_channel_count ++; 
				ret = ITO_TEST_FAIL;
			}	
		} 
	}

	return ret;
}

static ITO_TEST_RET ito_test_interface(void)
{
    ITO_TEST_RET ret1 = ITO_TEST_OK, ret2 = ITO_TEST_OK, ret3 = ITO_TEST_OK;
    uint16_t i = 0;
#ifdef DMA_IIC
    _msg_dma_alloc();
#endif
    ito_test_set_iic_rate(50000);
    ITO_TEST_DEBUG("start\n");
    ito_test_disable_irq();
	ito_test_reset();
    if(!ito_test_choose_TpType())
    {
        ITO_TEST_DEBUG("choose tpType fail\n");
        ret1 = ITO_TEST_GET_TP_TYPE_ERROR;
        goto ITO_TEST_END;
    }
    ito_test_EnterSerialDebugMode();
    mdelay(100);
    ITO_TEST_DEBUG("EnterSerialDebugMode\n");
    // stop cpu
    ito_test_WriteReg8Bit ( 0x0F, 0xE6, 0x01 ); //bank:mheg5, addr:h0073
    // stop watch dog
    ito_test_WriteReg ( 0x3C, 0x60, 0xAA55 ); //bank:reg_PIU_MISC_0, addr:h0030
    ITO_TEST_DEBUG("stop mcu and disable watchdog V.005\n");   
    mdelay(50);
    
	for(i = 0;i < 48;i++)
	{
		s16_raw_data_1[i] = 0;
		s16_raw_data_2[i] = 0;
		s16_raw_data_3[i] = 0;
	}	
	
    fail_channel_count = 0; // Reset fail_channel_count to 0 before test start
	
    ito_test_first(40, s16_raw_data_1);
    ITO_TEST_DEBUG("40 get s16_raw_data_1\n");
    if(ito_test_2r)
    {
        ret2=ito_test_second_2r(40);
    }
    else
    {
        ret2=ito_test_second(40);
    }
    
    ito_test_first(41, s16_raw_data_2);
    ITO_TEST_DEBUG("41 get s16_raw_data_2\n");
    if(ito_test_2r)
    {
        ret3=ito_test_second_2r(41);
    }
    else
    {
        ret3=ito_test_second(41);
    }
    
    ito_test_first(42, s16_raw_data_3);
    ITO_TEST_DEBUG("42 get s16_raw_data_3\n");
    
    ITO_TEST_END:
#ifdef DMA_IIC
    _msg_dma_free();
#endif
    ito_test_set_iic_rate(100000);
	ito_test_reset();
    ito_test_enable_irq();
    ITO_TEST_DEBUG("end\n");
    
    if ((ret1 != ITO_TEST_OK) && (ret2 == ITO_TEST_OK) && (ret3 == ITO_TEST_OK))
    {
        return ITO_TEST_GET_TP_TYPE_ERROR;		
    }
    else if ((ret1 == ITO_TEST_OK) && ((ret2 != ITO_TEST_OK) || (ret3 != ITO_TEST_OK)))
    {
    		return ITO_TEST_FAIL;	
    }
    else
    {
    		return ITO_TEST_OK;	
    }
}

#include <linux/proc_fs.h>
#define ITO_TEST_AUTHORITY 0777 
static struct proc_dir_entry *msg_ito_test = NULL;
static struct proc_dir_entry *debug = NULL;
static struct proc_dir_entry *debug_on_off = NULL;
static struct proc_dir_entry *fail_channel = NULL;
static struct proc_dir_entry *data = NULL;
#define PROC_MSG_ITO_TESE      "msg-ito-test"
#define PROC_ITO_TEST_DEBUG      "debug"
#define PROC_ITO_TEST_DEBUG_ON_OFF     "debug-on-off"
#define PROC_ITO_TEST_FAIL_CHANNEL     "fail-channel"
#define PROC_ITO_TEST_DATA      "data"
ITO_TEST_RET g_ito_test_ret = ITO_TEST_OK;

static int ito_test_proc_read_debug(char *page, char **start, off_t off, int count, int *eof, void *data)
{
    int cnt = 0;
    
    g_ito_test_ret = ito_test_interface();
    if(ITO_TEST_OK==g_ito_test_ret)
    {
        ITO_TEST_DEBUG_MUST("ITO_TEST_OK");
    }
    else if(ITO_TEST_FAIL==g_ito_test_ret)
    {
        ITO_TEST_DEBUG_MUST("ITO_TEST_FAIL");
    }
    else if(ITO_TEST_GET_TP_TYPE_ERROR==g_ito_test_ret)
    {
        ITO_TEST_DEBUG_MUST("ITO_TEST_GET_TP_TYPE_ERROR");
    }
    
    cnt = sprintf(page, "%d", g_ito_test_ret);

    *eof = 1;

    return cnt;
}

static int ito_test_proc_write_debug(struct file *file, const char *buffer, unsigned long count, void *data)
{    
    u16 i = 0;
    mdelay(5);

    ITO_TEST_DEBUG_MUST("ito_test_ret = %d",g_ito_test_ret);
    mdelay(5);
    for(i=0;i<48;i++)
    {
        ITO_TEST_DEBUG_MUST("data_1[%d]=%d;\n",i,s16_raw_data_1[i]);
    }
    mdelay(5);
    for(i=0;i<48;i++)
    {
        ITO_TEST_DEBUG_MUST("data_2[%d]=%d;\n",i,s16_raw_data_2[i]);
    }
    mdelay(5);
    for(i=0;i<48;i++)
    {
        ITO_TEST_DEBUG_MUST("data_3[%d]=%d;\n",i,s16_raw_data_3[i]);
    }
    mdelay(5);

    return count;
}

static int ito_test_proc_read_debug_on_off(char *page, char **start, off_t off, int count, int *eof, void *data)
{
    int cnt= 0;
    
    bItoTestDebug = 1;
    ITO_TEST_DEBUG_MUST("on debug bItoTestDebug = %d",bItoTestDebug);
    
    *eof = 1;
    return cnt;
}

static int ito_test_proc_write_debug_on_off(struct file *file, const char *buffer, unsigned long count, void *data)
{    
    bItoTestDebug = 0;
    ITO_TEST_DEBUG_MUST("off debug bItoTestDebug = %d",bItoTestDebug);
    return count;
}

static int ito_test_proc_read_fail_channel(char *page, char **start, off_t off, int count, int *eof, void *data)
{
    int cnt = 0;
    int i;

    ITO_TEST_DEBUG_MUST("ito_test_proc_read_fail_channel()\n");
    ITO_TEST_DEBUG_MUST("fail_channel_count = %d\n", fail_channel_count);
    
    for (i = 0; i < fail_channel_count; i ++)
    {
    	  page[i] = g_fail_channel[i];
    }

    *eof = 1;

    cnt = fail_channel_count;

    return cnt;
}

static int ito_test_proc_write_fail_channel(struct file *file, const char *buffer, unsigned long count, void *data)
{    
    ITO_TEST_DEBUG_MUST("ito_test_proc_write_fail_channel()\n");

    return count;
}

static int ito_test_proc_read_data(char *page, char **start, off_t off, int count, int *eof, void *data)
{
    int cnt = 0;
    int i;
    u8 high_byte, low_byte;

    ITO_TEST_DEBUG_MUST("ito_test_proc_read_data()\n");
    
    for (i = 0; i < 48; i ++)
    {
    	  high_byte = (s16_raw_data_1[i] & 0xFF00) >> 8;
    	  low_byte = (s16_raw_data_1[i] & 0x00FF) << 8;
    	  
        page[i*2] = high_byte;
        page[i*2+1] = low_byte;
    }

    for (i = 0; i < 48; i ++)
    {
        high_byte = (s16_raw_data_2[i] & 0xFF00) >> 8;
        low_byte = (s16_raw_data_2[i] & 0x00FF) << 8;
        
        page[i*2+96] = high_byte;
        page[i*2+1+96] = low_byte;
    }

    *eof = 1;
    
    cnt = 96*2;

    return cnt;
}

static int ito_test_proc_write_data(struct file *file, const char *buffer, unsigned long count, void *data)
{    
    ITO_TEST_DEBUG_MUST("ito_test_proc_write_data()\n");

    return count;
}

static void ito_test_create_entry(void)
{
    printk(KERN_ALERT "Enter here!\n");
    msg_ito_test = proc_mkdir(PROC_MSG_ITO_TESE, NULL);
    debug = create_proc_entry(PROC_ITO_TEST_DEBUG, ITO_TEST_AUTHORITY, msg_ito_test);
    debug_on_off= create_proc_entry(PROC_ITO_TEST_DEBUG_ON_OFF, ITO_TEST_AUTHORITY, msg_ito_test);
    fail_channel = create_proc_entry(PROC_ITO_TEST_FAIL_CHANNEL, ITO_TEST_AUTHORITY, msg_ito_test);
    data = create_proc_entry(PROC_ITO_TEST_DATA, ITO_TEST_AUTHORITY, msg_ito_test);

    if (NULL==debug) 
    {
        ITO_TEST_DEBUG_MUST("create_proc_entry ITO TEST DEBUG failed\n");
    } 
    else 
    {
        debug->read_proc = ito_test_proc_read_debug;
        debug->write_proc = ito_test_proc_write_debug;
        ITO_TEST_DEBUG_MUST("create_proc_entry ITO TEST DEBUG OK\n");
    }

    if (NULL==debug_on_off) 
    {
        ITO_TEST_DEBUG_MUST("create_proc_entry ITO TEST ON OFF failed\n");
    } 
    else 
    {
        debug_on_off->read_proc = ito_test_proc_read_debug_on_off;
        debug_on_off->write_proc = ito_test_proc_write_debug_on_off;
        ITO_TEST_DEBUG_MUST("create_proc_entry ITO TEST ON OFF OK\n");
    }

    if (NULL==fail_channel) 
    {
        ITO_TEST_DEBUG_MUST("create_proc_entry ITO TEST FAIL CHANNEL failed\n");
    } 
    else 
    {
        fail_channel->read_proc = ito_test_proc_read_fail_channel;
        fail_channel->write_proc = ito_test_proc_write_fail_channel;
        ITO_TEST_DEBUG_MUST("create_proc_entry ITO TEST FAIL CHANNEL OK\n");
    }

    if (NULL==data) 
    {
        ITO_TEST_DEBUG_MUST("create_proc_entry ITO TEST DATA failed\n");
    } 
    else 
    {
        data->read_proc = ito_test_proc_read_data;
        data->write_proc = ito_test_proc_write_data;
        ITO_TEST_DEBUG_MUST("create_proc_entry ITO TEST DATA OK\n");
    }
}
#endif
static u8 *dma_bufferma = NULL;//FTS_BYTE=u8;
static u32 dma_handlema = NULL;
static struct mutex dma_model_mutexma;

/*reset the chip*/
static void _HalTscrHWReset(void)
{
	mt_set_gpio_mode(GPIO_CTP_EN_PIN, GPIO_CTP_EN_PIN_M_GPIO);
	mt_set_gpio_dir(GPIO_CTP_EN_PIN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_CTP_EN_PIN, GPIO_OUT_ONE);
	mt_set_gpio_out(GPIO_CTP_EN_PIN, GPIO_OUT_ZERO);
	mdelay(10);
	mt_set_gpio_out(GPIO_CTP_EN_PIN, GPIO_OUT_ONE);
	mdelay(50);
}

static void I2cDMA_init()
{
	mutex_init(&dma_model_mutexma);
	dma_bufferma= (u8 *)dma_alloc_coherent(NULL, 255, &dma_handlema, GFP_KERNEL);
    	if(!dma_bufferma)
	{
    		printk("[MSG2138A][TSP] dma_alloc_coherent error\n");
	}
}

static void I2cDMA_exit()
{
	if(dma_bufferma)
	{
		dma_free_coherent(NULL, 255, dma_bufferma, dma_handlema);
		dma_bufferma = NULL;
		dma_handlema= 0;
	}
	_HalTscrHWReset();
	printk("[MSG2138A][TSP] dma_free_coherent OK\n");
}

static void HalTscrCReadI2CSeq(u8 addr, u8* read_data, u8 size)
{
	int ret;
	u16 addrt;
	//mutex_lock(&tp_mutex); 

	addrt=i2c_clientma->addr;
	i2c_clientma->addr = addr;
	if(size<9){
		i2c_clientma->addr = i2c_clientma->addr & I2C_MASK_FLAG;	
		ret=i2c_master_recv(i2c_clientma, &read_data[0], size);
	}
	else
	{
		mutex_lock(&dma_model_mutexma);
		i2c_clientma->addr = i2c_clientma->addr & I2C_MASK_FLAG | I2C_DMA_FLAG;
		mutex_unlock(&dma_model_mutexma);
		ret = i2c_master_recv(i2c_clientma, dma_handlema,size);

		if(ret<=0)
		{
			i2c_clientma->addr=addr;
			TPD_DMESG("[MSG2138A]i2c_read_byte error line = %d, ret = %d\n", __LINE__, ret);
			return ;
		}
		memcpy(read_data,dma_bufferma,size);
	}

	i2c_clientma->addr = addrt;
	if(ret <=  0) {
		printk("addr: %d i2c read interface error!\n",addr);	
	}
	//mutex_unlock(&tp_mutex);
}

static void HalTscrCDevWriteI2CSeq(u8 addr, u8* data, u16 size)
{
	int ret;
	u16 addrt;

	addrt=i2c_clientma->addr;
	i2c_clientma->addr = addr;
	//mutex_lock(&tp_mutex);
	if(size<9){
		i2c_clientma->addr = i2c_clientma->addr & I2C_MASK_FLAG;
		ret = i2c_master_send(i2c_clientma, &data[0], size);  
	}
	else
	{
		memcpy(dma_bufferma,data,size);
		//TPD_DMESG("[FT6306] I2C  DMA transfer----byte_write------llf\n");
		mutex_lock(&dma_model_mutexma);
		i2c_clientma->addr = i2c_clientma->addr & I2C_MASK_FLAG | I2C_DMA_FLAG;
		mutex_unlock(&dma_model_mutexma);
		ret = i2c_master_send(i2c_clientma, dma_handlema, size);

	}

	i2c_clientma->addr = addrt;
	if(ret <=  0) {
		printk("addr: %d i2c write interface error!\n",addr);	
	}  
	//mutex_unlock(&tp_mutex);
}
static void dbbusDWIICEnterSerialDebugMode(void)
{
    u8 data[5];

    // Enter the Serial Debug Mode
    data[0] = 0x53;
    data[1] = 0x45;
    data[2] = 0x52;
    data[3] = 0x44;
    data[4] = 0x42;
    HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, data, 5);
}

static void dbbusDWIICStopMCU(void)
{
    u8 data[1];

    // Stop the MCU
    data[0] = 0x37;
    HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, data, 1);
}

static void dbbusDWIICIICUseBus(void)
{
    u8 data[1];

    // IIC Use Bus
    data[0] = 0x35;
    HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, data, 1);
}

static void dbbusDWIICIICReshape(void)
{
    u8 data[1];

    // IIC Re-shape
    data[0] = 0x71;
    HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, data, 1);
}

static void dbbusDWIICIICNotUseBus(void)
{
    u8 data[1];

    // IIC Not Use Bus
    data[0] = 0x34;
    HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, data, 1);
}

static void dbbusDWIICNotStopMCU(void)
{
    u8 data[1];

    // Not Stop the MCU
    data[0] = 0x36;
    HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, data, 1);
}

static void dbbusDWIICExitSerialDebugMode(void)
{
    u8 data[1];

    // Exit the Serial Debug Mode
    data[0] = 0x45;
    HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, data, 1);

    // Delay some interval to guard the next transaction
    udelay ( 200 );        // delay about 0.2ms
}

int crc_tab[256];
static u8 g_dwiic_info_data[1024];
static u8 Auto_APK_flag=0;//if apk upgrade =1;
unsigned short fw_major_version=0,fw_minor_version=0;

/*add by liukai for release the touch action*/
static void msg21xx_release(void)
{
	printk("[%s]: Enter!\n", __func__);
	input_report_key(tpd->dev,KEY_BACK , 0);
	input_report_key(tpd->dev, KEY_HOME, 0);
	input_report_key(tpd->dev, KEY_MENU, 0);
	input_report_key(tpd->dev, KEY_SEARCH, 0);
	input_report_abs(tpd->dev, ABS_MT_TOUCH_MAJOR, 0);
	input_mt_sync(tpd->dev);
	input_sync(tpd->dev);
}

/* update the firmware part, used by apk*/
/*show the fw version*/

static u32 Reflect ( u32 ref, char ch ) //unsigned int Reflect(unsigned int ref, char ch)
{
    u32 value = 0;
    u32 i = 0;

    for ( i = 1; i < ( ch + 1 ); i++ )
    {
        if ( ref & 1 )
        {
            value |= 1 << ( ch - i );
        }
        ref >>= 1;
    }
    return value;
}

u32 Get_CRC ( u32 text, u32 prevCRC, u32 *crc32_table )
{
    u32  ulCRC = prevCRC;
	ulCRC = ( ulCRC >> 8 ) ^ crc32_table[ ( ulCRC & 0xFF ) ^ text];
    return ulCRC ;
}

static void Init_CRC32_Table ( u32 *crc32_table )
{
    u32 magicnumber = 0x04c11db7;
    u32 i = 0, j;

    for ( i = 0; i <= 0xFF; i++ )
    {
        crc32_table[i] = Reflect ( i, 8 ) << 24;
        for ( j = 0; j < 8; j++ )
        {
            crc32_table[i] = ( crc32_table[i] << 1 ) ^ ( crc32_table[i] & ( 0x80000000L ) ? magicnumber : 0 );
        }
        crc32_table[i] = Reflect ( crc32_table[i], 32 );
    }
}

static void Init_CRC32_Table_A ( u32 *crc32_table )
{
    u32 magicnumber = 0x04c11db7;
    u32 i = 0, j;

    for ( i = 0; i <= 0xFF; i++ )
    {
        crc32_table[i] = Reflect ( i, 8 ) << 24;
        for ( j = 0; j < 8; j++ )
        {
            crc32_table[i] = ( crc32_table[i] << 1 ) ^ ( crc32_table[i] & ( 0x80000000L ) ? magicnumber : 0 );
        }
        crc32_table[i] = Reflect ( crc32_table[i], 32 );
    }
}

typedef enum
{
	EMEM_ALL = 0,
	EMEM_MAIN,
	EMEM_INFO,
} EMEM_TYPE_t;

static void drvDB_WriteReg8Bit ( u8 bank, u8 addr, u8 data )
{
    u8 tx_data[4] = {0x10, bank, addr, data};
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, tx_data, 4 );
}

static void drvDB_WriteReg ( u8 bank, u8 addr, u16 data )
{
    u8 tx_data[5] = {0x10, bank, addr, data & 0xFF, data >> 8};
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, tx_data, 5 );
}

static unsigned short drvDB_ReadReg ( u8 bank, u8 addr )
{
    u8 tx_data[3] = {0x10, bank, addr};
    u8 rx_data[2] = {0};

    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, tx_data, 3 );
    HalTscrCReadI2CSeq ( FW_ADDR_MSG21XX, &rx_data[0], 2 );
    return ( rx_data[1] << 8 | rx_data[0] );
}


static int drvTP_erase_emem_c33 ( EMEM_TYPE_t emem_type )
{
    // stop mcu
    drvDB_WriteReg ( 0x0F, 0xE6, 0x0001 );

    //disable watch dog
    drvDB_WriteReg8Bit ( 0x3C, 0x60, 0x55 );
    drvDB_WriteReg8Bit ( 0x3C, 0x61, 0xAA );

    // set PROGRAM password
    drvDB_WriteReg8Bit ( 0x16, 0x1A, 0xBA );
    drvDB_WriteReg8Bit ( 0x16, 0x1B, 0xAB );

    //proto.MstarWriteReg(F1.loopDevice, 0x1618, 0x80);
    drvDB_WriteReg8Bit ( 0x16, 0x18, 0x80 );

    if ( emem_type == EMEM_ALL )
    {
        drvDB_WriteReg8Bit ( 0x16, 0x08, 0x10 ); //mark
    }

    drvDB_WriteReg8Bit ( 0x16, 0x18, 0x40 );
    mdelay ( 10 );

    drvDB_WriteReg8Bit ( 0x16, 0x18, 0x80 );

    // erase trigger
    if ( emem_type == EMEM_MAIN )
    {
        drvDB_WriteReg8Bit ( 0x16, 0x0E, 0x04 ); //erase main
    }
    else
    {
        drvDB_WriteReg8Bit ( 0x16, 0x0E, 0x08 ); //erase all block
    }

    return ( 1 );
}

static int drvTP_read_emem_dbbus_c33 ( EMEM_TYPE_t emem_type, u16 addr, size_t size, u8 *p, size_t set_pce_high )
{
    u32 i;

    // Set the starting address ( must before enabling burst mode and enter riu mode )
    drvDB_WriteReg ( 0x16, 0x00, addr );

    // Enable the burst mode ( must before enter riu mode )
    drvDB_WriteReg ( 0x16, 0x0C, drvDB_ReadReg ( 0x16, 0x0C ) | 0x0001 );

    // Set the RIU password
    drvDB_WriteReg ( 0x16, 0x1A, 0xABBA );

    // Enable the information block if pifren is HIGH
    if ( emem_type == EMEM_INFO )
    {
        // Clear the PCE
        drvDB_WriteReg ( 0x16, 0x18, drvDB_ReadReg ( 0x16, 0x18 ) | 0x0080 );
        mdelay ( 10 );

        // Set the PIFREN to be HIGH
        drvDB_WriteReg ( 0x16, 0x08, 0x0010 );
    }

    // Set the PCE to be HIGH
    drvDB_WriteReg ( 0x16, 0x18, drvDB_ReadReg ( 0x16, 0x18 ) | 0x0040 );
    mdelay ( 10 );

    // Wait pce becomes 1 ( read data ready )
    while ( ( drvDB_ReadReg ( 0x16, 0x10 ) & 0x0004 ) != 0x0004 );

    for ( i = 0; i < size; i += 4 )
    {
        // Fire the FASTREAD command
        drvDB_WriteReg ( 0x16, 0x0E, drvDB_ReadReg ( 0x16, 0x0E ) | 0x0001 );

        // Wait the operation is done
        while ( ( drvDB_ReadReg ( 0x16, 0x10 ) & 0x0001 ) != 0x0001 );

        p[i + 0] = drvDB_ReadReg ( 0x16, 0x04 ) & 0xFF;
        p[i + 1] = ( drvDB_ReadReg ( 0x16, 0x04 ) >> 8 ) & 0xFF;
        p[i + 2] = drvDB_ReadReg ( 0x16, 0x06 ) & 0xFF;
        p[i + 3] = ( drvDB_ReadReg ( 0x16, 0x06 ) >> 8 ) & 0xFF;
    }

    // Disable the burst mode
    drvDB_WriteReg ( 0x16, 0x0C, drvDB_ReadReg ( 0x16, 0x0C ) & ( ~0x0001 ) );

    // Clear the starting address
    drvDB_WriteReg ( 0x16, 0x00, 0x0000 );

    //Always return to main block
    if ( emem_type == EMEM_INFO )
    {
        // Clear the PCE before change block
        drvDB_WriteReg ( 0x16, 0x18, drvDB_ReadReg ( 0x16, 0x18 ) | 0x0080 );
        mdelay ( 10 );
        // Set the PIFREN to be LOW
        drvDB_WriteReg ( 0x16, 0x08, drvDB_ReadReg ( 0x16, 0x08 ) & ( ~0x0010 ) );

        drvDB_WriteReg ( 0x16, 0x18, drvDB_ReadReg ( 0x16, 0x18 ) | 0x0040 );
        while ( ( drvDB_ReadReg ( 0x16, 0x10 ) & 0x0004 ) != 0x0004 );
    }

    // Clear the RIU password
    drvDB_WriteReg ( 0x16, 0x1A, 0x0000 );

    if ( set_pce_high )
    {
        // Set the PCE to be HIGH before jumping back to e-flash codes
        drvDB_WriteReg ( 0x16, 0x18, drvDB_ReadReg ( 0x16, 0x18 ) | 0x0040 );
        while ( ( drvDB_ReadReg ( 0x16, 0x10 ) & 0x0004 ) != 0x0004 );
    }

    return ( 1 );
}


static int drvTP_read_info_dwiic_c33 ( void )
{
    u8  dwiic_tx_data[5];
    u8  index=0;
    u16 reg_data=0;
    //unsigned char dbbus_rx_data[2] = {0};

    mdelay ( 300 );


    // Stop Watchdog
    drvDB_WriteReg8Bit ( 0x3C, 0x60, 0x55 );
    drvDB_WriteReg8Bit ( 0x3C, 0x61, 0xAA );

    drvDB_WriteReg ( 0x3C, 0xE4, 0xA4AB );

	drvDB_WriteReg ( 0x1E, 0x04, 0x7d60 );

    // TP SW reset
    drvDB_WriteReg ( 0x1E, 0x04, 0x829F );
	mdelay (1);

    dwiic_tx_data[0] = 0x10;
    dwiic_tx_data[1] = 0x0F;
    dwiic_tx_data[2] = 0xE6;
    dwiic_tx_data[3] = 0x00;
    mdelay(15);
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dwiic_tx_data, 4 );

        // stop mcu
  //  drvDB_WriteReg ( 0x1E, 0xE6, 0x0001 );

    mdelay ( 100 );
	TP_DEBUG ( "read infor 1\n");

    do{
        reg_data = drvDB_ReadReg ( 0x3C, 0xE4 );
    }
    while ( reg_data != 0x5B58 );
	TP_DEBUG ( "read infor +++2\n");
    dwiic_tx_data[0] = 0x72;

   // dwiic_tx_data[3] = 0x04;
  //  dwiic_tx_data[4] = 0x00;
    dwiic_tx_data[3] = 0x00;
    dwiic_tx_data[4] = 0x80;
	
    for(reg_data=0;reg_data<8;reg_data++)
    {
    	dwiic_tx_data[1] = 0x80+(((reg_data*128)&0xff00)>>8);
    	dwiic_tx_data[2] = (reg_data*128)&0x00ff;
    	HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX_TP , dwiic_tx_data, 5 );
    	mdelay (50 );

    // recive info data
    	HalTscrCReadI2CSeq ( FW_ADDR_MSG21XX_TP, &g_dwiic_info_data[reg_data*128], 128);
    }

    fw_major_version = (g_dwiic_info_data[15]<<8)+g_dwiic_info_data[14];
    fw_minor_version = (g_dwiic_info_data[17]<<8)+g_dwiic_info_data[16];
	if(fw_major_version != 5)
	{
		    if((g_dwiic_info_data[0x300]>=0x30 && g_dwiic_info_data[0x300]<=0x39)
			&&(g_dwiic_info_data[0x301]>=0x30 && g_dwiic_info_data[0x301]<=0x39)
			&&(g_dwiic_info_data[0x302]>=0x31 && g_dwiic_info_data[0x302]<=0x39))
		    {
			    fw_major_version = (g_dwiic_info_data[0x300]-0x30)*100+(g_dwiic_info_data[0x301]-0x30)*10+(g_dwiic_info_data[0x302]-0x30);	
				fw_minor_version = 1;
		    }
	}

    TP_DEBUG("***major = %d ***\n", fw_major_version);
    TP_DEBUG("***minor = %d ***\n", fw_minor_version);
/*
    if((fw_major_version+fw_minor_version)==0)
    {
	//fw_major_version=1; //default test for MT -------------------------------------llf
	//fw_minor_version=2;
	fw_major_version=2; //default test for junda -------------------------------------llf
	fw_minor_version=0;
    }
*/	
    for(index=0;index<16;index++){
		TP_DEBUG("g_dwiic_info_data [%d] = %c \n",index,g_dwiic_info_data[index]);
    }
    TP_DEBUG ( "\n g_dwiic_info_data display complete!! \n");

    return ( 1 );
}

static int drvTP_info_updata_C33 ( u16 start_index, u8 *data, u16 size )
{
    // size != 0, start_index+size !> 1024
    u16 i;
    for ( i = 0; i < size; i++ )
    {
        g_dwiic_info_data[start_index] = * ( data + i );
        start_index++;
    }
    return ( 1 );
}

static ssize_t firmware_update_c33 ( size_t size, EMEM_TYPE_t emem_type )
{
    u8  dbbus_tx_data[4];
    u8  dbbus_rx_data[2] = {0};
    u8  life_counter[2];
    u32 i, j;
    u32 crc_main, crc_main_tp;
    u32 crc_info, crc_info_tp;

    int update_pass = 1;
    u16 reg_data = 0;
    int count=0;

    crc_main = 0xffffffff;
    crc_info = 0xffffffff;

    printk ( "[MSG2138A]++firmware_update_c33++\n" );

	mtk_wdt_restart(WK_WDT_EXT_TYPE_NOLOCK);//avoid watchdog timeout

	/*-----------------------------------//Auto upgrade to get date-------------------------------------------*/
	if(Auto_APK_flag==0){   
		for (i = 0; i < 33; i++)   // total  33 KB : 1 byte per R/W
		{
			for ( j = 0; j < 1024; j++ )        //Read 1k bytes
			{
				//temp[i][j] = MSG_FIRMWARE_JD[(i*1024)+j];
                                temp[i][j] = MSG_FIRMWARE_EA[(i*1024)+j]; // Read the bin files of slave firmware from the baseband file system 
                        }
                }
		printk("[MSG2138A] Date transf completed !\n"); 
        }
	  /*-----------------------------------//Auto upgrade to get date-------------------------------------------*/ 

#if (0)
    drvTP_read_info_dwiic_c33();//check

    mtk_wdt_restart(WK_WDT_EXT_TYPE_NOLOCK);//avoid watchdog timeout

    if (1) //( g_dwiic_info_data[0] == 'M' && g_dwiic_info_data[1] == 'S' && g_dwiic_info_data[2] == 'T' && g_dwiic_info_data[3] == 'A' && g_dwiic_info_data[4] == 'R' && g_dwiic_info_data[5] == 'T' && g_dwiic_info_data[6] == 'P' && g_dwiic_info_data[7] == 'C' )
    {
		// updata FW Version
		//drvTP_info_updata_C33 ( 8, &temp[32][8], 5 );

		g_dwiic_info_data[8]=temp[32][8];
		g_dwiic_info_data[9]=temp[32][9];
		g_dwiic_info_data[10]=temp[32][10];
		g_dwiic_info_data[11]=temp[32][11];
		// updata life counter
		life_counter[1] = (( ( (g_dwiic_info_data[13] << 8 ) | g_dwiic_info_data[12]) + 1 ) >> 8 ) & 0xFF;
		life_counter[0] = ( ( (g_dwiic_info_data[13] << 8 ) | g_dwiic_info_data[12]) + 1 ) & 0xFF;
		g_dwiic_info_data[12]=life_counter[0];
		g_dwiic_info_data[13]=life_counter[1];
		//drvTP_info_updata_C33 ( 10, &life_counter[0], 3 );
		drvDB_WriteReg ( 0x3C, 0xE4, 0x78C5 );
		drvDB_WriteReg ( 0x1E, 0x04, 0x7d60 );
		// TP SW reset
		drvDB_WriteReg ( 0x1E, 0x04, 0x829F );

		mdelay ( 50 );

		//polling 0x3CE4 is 0x2F43
		do
		{
		    reg_data = drvDB_ReadReg ( 0x3C, 0xE4 );

		}
		while ( reg_data != 0x2F43 );

		// transmit lk info data
		for(count=0;count<8;count++)
		{
			HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX_TP , &g_dwiic_info_data[count*128], 128 );
		}
		//polling 0x3CE4 is 0xD0BC
		do
		{
		    reg_data = drvDB_ReadReg ( 0x3C, 0xE4 );
		}
		while ( reg_data != 0xD0BC );

    	}
	else
	{
		printk ( "[MSG2138A] update math faild!\n" );
		_HalTscrHWReset();
		FwDataCnt = 0;
		//enable_irq(msg21xx_irq);
		mt65xx_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);
		return size;
	}
#endif
    //erase main
    drvTP_erase_emem_c33 ( EMEM_MAIN );
    mdelay ( 1000 );

    printk("[MSG2138A]  Earse flash completed! \n");

    //ResetSlave();
    _HalTscrHWReset();

    //drvDB_EnterDBBUS();
    dbbusDWIICEnterSerialDebugMode();
    dbbusDWIICStopMCU();
    dbbusDWIICIICUseBus();
    dbbusDWIICIICReshape();
    mdelay ( 100 );

    /////////////////////////
    // Program
    /////////////////////////

    //polling 0x3CE4 is 0x1C70
    if ( ( emem_type == EMEM_ALL ) || ( emem_type == EMEM_MAIN ) )
    {
        do
        {
            reg_data = drvDB_ReadReg ( 0x3C, 0xE4 );
        }
        while ( reg_data != 0x1C70 );
    }

	printk("[MSG2133] Polling OK \n");

    switch ( emem_type )
    {
        case EMEM_ALL:
            drvDB_WriteReg ( 0x3C, 0xE4, 0xE38F );  // for all-blocks
            break;
        case EMEM_MAIN:
            drvDB_WriteReg ( 0x3C, 0xE4, 0x7731 );  // for main block
            break;
        case EMEM_INFO:
            drvDB_WriteReg ( 0x3C, 0xE4, 0x7731 );  // for info block

            drvDB_WriteReg8Bit ( 0x0F, 0xE6, 0x01 );

            drvDB_WriteReg8Bit ( 0x3C, 0xE4, 0xC5 ); //
            drvDB_WriteReg8Bit ( 0x3C, 0xE5, 0x78 ); //

            drvDB_WriteReg8Bit ( 0x1E, 0x04, 0x9F );
            drvDB_WriteReg8Bit ( 0x1E, 0x05, 0x82 );

            drvDB_WriteReg8Bit ( 0x0F, 0xE6, 0x00 );
            mdelay ( 100 );
            break;
    }


    printk("[MSG2133] Emem_type init OK\n");
    mdelay ( 100 );
    // polling 0x3CE4 is 0x2F43
    do
    {
        reg_data = drvDB_ReadReg ( 0x3C, 0xE4 );
    }
    while ( reg_data != 0x2F43 );

     printk("[MSG2133] Polling again OK\n");

    // calculate CRC 32
    Init_CRC32_Table_A ( &crc_tab[0] );

    mtk_wdt_restart(WK_WDT_EXT_TYPE_NOLOCK);//avoid watchdog timeout
	for ( i = 0; i < 32; i++ ) // total  33 KB : last 1KB don't write : 2 byte per R/W
	{
		if ( emem_type == EMEM_INFO )
			i = 32;

		if ( i < 32 )   //emem_main
		{
			if ( i == 31 )
			{
				temp[i][1014] = 0x5A; //Fmr_Loader[1014]=0x5A;
				temp[i][1015] = 0xA5; //Fmr_Loader[1015]=0xA5;

				for ( j = 0; j < 1016; j++ )
				{
					//crc_main=Get_CRC(Fmr_Loader[j],crc_main,&crc_tab[0]);
					crc_main = Get_CRC ( temp[i][j], crc_main, &crc_tab[0] );
				}
			}	
			else
			{
				for ( j = 0; j < 1024; j++ )
				{
					//crc_main=Get_CRC(Fmr_Loader[j],crc_main,&crc_tab[0]);
					crc_main = Get_CRC ( temp[i][j], crc_main, &crc_tab[0] );
				}
			}
		}
		else  //emem_info
		{
			for ( j = 0; j < 1024; j++ )
			{
				//crc_info=Get_CRC(Fmr_Loader[j],crc_info,&crc_tab[0]);
				crc_info = Get_CRC ( g_dwiic_info_data[j], crc_info, &crc_tab[0] );
			}
			if ( emem_type == EMEM_MAIN ) 
				break;
		}

		//drvDWIIC_MasterTransmit( DWIIC_MODE_DWIIC_ID, 1024, Fmr_Loader );
		for( j = 0; j < 8; j++)
		{
			HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX_TP, &temp[i][j*128], 128 );
       		}
		printk("[MSG2138A]Wrete Flash the %dKB OK \n",i+1);
		mdelay ( 100 );

		// polling 0x3CE4 is 0xD0BC
		do
		{
			reg_data = drvDB_ReadReg ( 0x3C, 0xE4 );
		}
		while ( reg_data != 0xD0BC );
		mdelay ( 100 );

		drvDB_WriteReg ( 0x3C, 0xE4, 0x2F43 );
		mtk_wdt_restart(WK_WDT_EXT_TYPE_NOLOCK);//avoid watchdog timeout
	}

	printk("[MSG2138A] Write Flash Completed \n");

	if ( ( emem_type == EMEM_ALL ) || ( emem_type == EMEM_MAIN ) )
	{
		// write file done and check crc
		drvDB_WriteReg ( 0x3C, 0xE4, 0x1380 );
	}
	mdelay ( 100 ); //MCR_CLBK_DEBUG_DELAY ( 10, MCU_LOOP_DELAY_COUNT_MS );

	if ( ( emem_type == EMEM_ALL ) || ( emem_type == EMEM_MAIN ) )
	{
	// polling 0x3CE4 is 0x9432
		do
		{
			reg_data = drvDB_ReadReg ( 0x3C, 0xE4 );
		}while ( reg_data != 0x9432 );
	}

	crc_main = crc_main ^ 0xffffffff;
	crc_info = crc_info ^ 0xffffffff;

	if ( ( emem_type == EMEM_ALL ) || ( emem_type == EMEM_MAIN ) )
	{
		// CRC Main from TP
		crc_main_tp = drvDB_ReadReg ( 0x3C, 0x80 );
		crc_main_tp = ( crc_main_tp << 16 ) | drvDB_ReadReg ( 0x3C, 0x82 );

		// CRC Info from TP
		crc_info_tp = drvDB_ReadReg ( 0x3C, 0xA0 );
		crc_info_tp = ( crc_info_tp << 16 ) | drvDB_ReadReg ( 0x3C, 0xA2 );
	}
	printk ( "crc_main=0x%x, crc_info=0x%x, crc_main_tp=0x%x, crc_info_tp=0x%x\n",crc_main, crc_info, crc_main_tp, crc_info_tp );

	//drvDB_ExitDBBUS();

	update_pass = 1;
	if ( ( emem_type == EMEM_ALL ) || ( emem_type == EMEM_MAIN ) )
	{
		if ( crc_main_tp != crc_main )
			update_pass = 0;

		if ( crc_info_tp != crc_info )
			update_pass = 0;
	}

#if 0
	if ( !update_pass )
	{
	printk ( "update FAILED\n" );
	_HalTscrHWReset();
	FwDataCnt = 0;
	//enable_irq(msg21xx_irq);

	mt65xx_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);
	return ( 0 );
	}
#endif

	printk ( "[MSG2138A] update OK\n" );
	_HalTscrHWReset();
	FwDataCnt = 0;
	//enable_irq(msg21xx_irq);
	mt65xx_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);
	mtk_wdt_restart(WK_WDT_EXT_TYPE_NOLOCK);//avoid watchdog timeout
	return size;
}

static void firmware_version()
{
    unsigned char dbbus_tx_data[3];
    unsigned char dbbus_rx_data[4] ;

    _HalTscrHWReset();
    dbbusDWIICEnterSerialDebugMode();
    dbbusDWIICStopMCU();
    dbbusDWIICIICUseBus();
    dbbusDWIICIICReshape();


    fw_version = kzalloc(sizeof(char), GFP_KERNEL);

    //Get_Chip_Version();
    dbbus_tx_data[0] = 0x53;
    dbbus_tx_data[1] = 0x00;
    dbbus_tx_data[2] = 0x2A;
    HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX_TP, &dbbus_tx_data[0], 3);
    HalTscrCReadI2CSeq(FW_ADDR_MSG21XX_TP, &dbbus_rx_data[0], 4);

    fw_major_version = (dbbus_rx_data[1]<<8)+dbbus_rx_data[0];
    fw_minor_version = (dbbus_rx_data[3]<<8)+dbbus_rx_data[2];

    TP_DEBUG("***major = %d ***\n", fw_major_version);
    TP_DEBUG("***minor = %d ***\n", fw_minor_version);
    sprintf(fw_version,"%03d%03d", fw_major_version, fw_minor_version);
    _HalTscrHWReset();
    return ;
    //TP_DEBUG(printk("***fw_version = %s ***\n", fw_version);)
}

static int firmware_updata()
{
	u8 dbbus_tx_data[4];
	unsigned char dbbus_rx_data[2] = {0};
	mt65xx_eint_mask(CUST_EINT_TOUCH_PANEL_NUM);

	_HalTscrHWReset();
	i2c_clientma->timing = 50;

	// Erase TP Flash first
	dbbusDWIICEnterSerialDebugMode();
	dbbusDWIICStopMCU();
	dbbusDWIICIICUseBus();
	dbbusDWIICIICReshape();
	mdelay ( 300 );

	// Disable the Watchdog
	dbbus_tx_data[0] = 0x10;
	dbbus_tx_data[1] = 0x3C;
	dbbus_tx_data[2] = 0x60;
	dbbus_tx_data[3] = 0x55;
	HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );
	dbbus_tx_data[0] = 0x10;
	dbbus_tx_data[1] = 0x3C;
	dbbus_tx_data[2] = 0x61;
	dbbus_tx_data[3] = 0xAA;
	HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );
	// Stop MCU
	dbbus_tx_data[0] = 0x10;
	dbbus_tx_data[1] = 0x0F;
	dbbus_tx_data[2] = 0xE6;
	dbbus_tx_data[3] = 0x01;
	HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );
	/////////////////////////
	// Difference between C2 and C3
	/////////////////////////
	// c2:2133 c32:2133a(2) c33:2133a
	//check id
	dbbus_tx_data[0] = 0x10;
	dbbus_tx_data[1] = 0x1E;
	dbbus_tx_data[2] = 0xCC;
	HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 3 );
	HalTscrCReadI2CSeq ( FW_ADDR_MSG21XX, &dbbus_rx_data[0], 2 );
	if ( dbbus_rx_data[0] == 2 )
	{
	    // check version
	    dbbus_tx_data[0] = 0x10;
	    dbbus_tx_data[1] = 0x3C;
	    dbbus_tx_data[2] = 0xEA;
	    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 3 );
	    HalTscrCReadI2CSeq ( FW_ADDR_MSG21XX, &dbbus_rx_data[0], 2 );
	    TP_DEBUG ( "dbbus_rx version[0]=0x%x", dbbus_rx_data[0] );

	    if ( dbbus_rx_data[0] == 3 ){
			firmware_update_c33 ( 1, EMEM_MAIN );
		}
	}
	_HalTscrHWReset();
	 i2c_clientma->timing = 100;
	 i2c_clientma->addr = FW_ADDR_MSG21XX_TP;
	return 1;
}

#define APK_UPGRADE

#ifdef APK_UPGRADE

struct class *firmware_class;
struct device *firmware_cmd_dev;

static ssize_t firmware_update_store ( struct device *dev,
                                       struct device_attribute *attr, const char *buf, size_t size )
{
    u8 i;
    u8 dbbus_tx_data[4];
    unsigned char dbbus_rx_data[2] = {0};
	
    mt65xx_eint_mask(CUST_EINT_TOUCH_PANEL_NUM);
    I2cDMA_init();//DMA MODE open

    _HalTscrHWReset();
    i2c_clientma->timing = 50;

    // Erase TP Flash first
    dbbusDWIICEnterSerialDebugMode();
    dbbusDWIICStopMCU();
    dbbusDWIICIICUseBus();
    dbbusDWIICIICReshape();
    mdelay ( 300 );

    // Disable the Watchdog
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x3C;
    dbbus_tx_data[2] = 0x60;
    dbbus_tx_data[3] = 0x55;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x3C;
    dbbus_tx_data[2] = 0x61;
    dbbus_tx_data[3] = 0xAA;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );
    // Stop MCU
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x0F;
    dbbus_tx_data[2] = 0xE6;
    dbbus_tx_data[3] = 0x01;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );
    /////////////////////////
    // Difference between C2 and C3
    /////////////////////////
    // c2:2133 c32:2133a(2) c33:2133a
    //check id
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0xCC;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 3 );
    HalTscrCReadI2CSeq ( FW_ADDR_MSG21XX, &dbbus_rx_data[0], 2 );
    if ( dbbus_rx_data[0] == 2 )
    {
        // check version
        dbbus_tx_data[0] = 0x10;
        dbbus_tx_data[1] = 0x3C;
        dbbus_tx_data[2] = 0xEA;
        HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 3 );
        HalTscrCReadI2CSeq ( FW_ADDR_MSG21XX, &dbbus_rx_data[0], 2 );
        TP_DEBUG ( "dbbus_rx version[0]=0x%x\n", dbbus_rx_data[0] );

        if ( dbbus_rx_data[0] == 3 ){
		firmware_update_c33 ( size, EMEM_MAIN );
	}
    }
     I2cDMA_exit();//DMA MODE close;
     i2c_clientma->timing = 100;
    i2c_clientma->addr = FW_ADDR_MSG21XX_TP;
    return size;
}

static ssize_t firmware_update_show ( struct device *dev,
                                      struct device_attribute *attr, char *buf )
{
    return sprintf ( buf, "%s\n", fw_version );
}

static DEVICE_ATTR(update, 0644, firmware_update_show, firmware_update_store);
/*test=================*/
/*Add by Tracy.Lin for update touch panel firmware and get fw version*/

static ssize_t firmware_version_show(struct device *dev,
                                     struct device_attribute *attr, char *buf)
{
    TP_DEBUG("*** firmware_version_show fw_version = %s***\n", fw_version);
    return sprintf(buf, "%s\n", fw_version);
}

static ssize_t firmware_version_store(struct device *dev,
                                      struct device_attribute *attr, const char *buf, size_t size)
{
	firmware_version();
    	return size;
}
static DEVICE_ATTR(version, 0644, firmware_version_show, firmware_version_store);

static ssize_t firmware_data_show(struct device *dev,
                                  struct device_attribute *attr, char *buf)
{
    return FwDataCnt;
}

static ssize_t firmware_data_store(struct device *dev,
                                   struct device_attribute *attr, const char *buf, size_t size)
{
    int i;
    Auto_APK_flag=1;
    TP_DEBUG("***FwDataCnt = %d ***\n", FwDataCnt);
    memcpy(temp[FwDataCnt], buf, 1024);
    FwDataCnt++;
    return size;
}
static DEVICE_ATTR(data, 0644, firmware_data_show, firmware_data_store);
#endif  // APK_UPGRADE
#endif  //__FIRMWARE_UPDATE__

static ssize_t firmware_versions_show(struct device *dev,struct device_attribute *attr, char *buf)
{
	firmware_version();
	TPD_DMESG("*** firmware_version_show fw_version = %s***\n", fw_version);
	return sprintf(buf, "%d.%03d \n", fw_major_version, fw_minor_version);
}

static DEVICE_ATTR(versions, 0444, firmware_versions_show,NULL);

 static u8 Calculate_8BitsChecksum( u8 *msg, s32 s32Length )
 {
	 s32 s32Checksum = 0;
	 s32 i;
 
	 for ( i = 0 ; i < s32Length; i++ )
	 {
		 s32Checksum += msg[i];
	 }
 
	 return (u8)( ( -s32Checksum ) & 0xFF );
 }

 #ifdef TPD_HAVE_BUTTON 
static int tpd_keys_local[TPD_KEY_COUNT] = TPD_KEYS;
static int tpd_keys_dim_local[TPD_KEY_COUNT][4] = TPD_KEYS_DIM;
#endif

/**********************************************/



/*********************************************/


#ifdef HQ_CTP_PS_TEST

#define HQALSPS_DEVICE_NAME               		"hq-sprd-alsps-tpd-dev"
#define HQALSPS_INPUT_NAME               		"hq-sprd-alsps-tpd-input"
#define HQALSPS_IOCTL_MAGIC        			0XCF
#define HQALSPS_IOCTL_PROX_ON		_IO(HQALSPS_IOCTL_MAGIC, 7)
#define HQALSPS_IOCTL_PROX_OFF		_IO(HQALSPS_IOCTL_MAGIC, 8)
int ps_state = -1;
static int mstar2133_pls_opened=0;
extern int tp_pls_status;

static void  msg2133_ps_mode_enable(bool enable)
{
	int err = 0;
	unsigned char dbbus_tx_data[4];
	unsigned char dbbus_rx_data[4];
	if(enable)
	{
	      dbbus_tx_data[0] = 0x52;
	      dbbus_tx_data[1] = 0x00;
	      dbbus_tx_data[2] = 0x4A;
	      dbbus_tx_data[3] = 0xA0;
	      err = i2c_master_send(i2c_clientma, &dbbus_tx_data[0], 4);
	      printk("i2c_client-addr=%d\n",i2c_clientma->addr);
	      printk(" pls wuyongtao_ctp ctp ps open err = %d \n",err);
	}
	else
	{     
	      dbbus_tx_data[0] = 0x52;
	      dbbus_tx_data[1] = 0x00;
	      dbbus_tx_data[2] = 0x4A;
	      dbbus_tx_data[3] = 0xA1;
	      err = i2c_master_send(i2c_clientma, &dbbus_tx_data[0], 4);
	      printk("pls  wuyongtao_ctp ctp ps close err = %d \n",err);
	}
	printk("pls  wuyongtao_ctp ctp ps msg2133a_ps_mode_enable\n");
} 

 int mstar2133_pls_enable(void)
 {
	 printk("<6> wuyongtao ==>ap3212c_pls_enable\n");
	 msg2133_ps_mode_enable(true);
	 mstar2133_pls_opened = 1;
	 ps_state = -1;
	 //input_report_abs(input_dev, ABS_DISTANCE, 1);
	 //input_sync(input_dev);
	 //mod_timer(&ps_timer, jiffies + msecs_to_jiffies(200) );
	 return 0;
 }
 
 int mstar2133_pls_disable(void)
 {

	 printk("<6> wuyongtao ==>ap3212c_pls_disable");
	msg2133_ps_mode_enable(false);
	 mstar2133_pls_opened = 0;
	ps_state = -1;
	// del_timer_sync(&ps_timer);	 
	 return 0;
 }
 static int mstar2133_pls_open(struct inode *inode, struct file *file)
 {
	 printk("%s\n", __func__);
	 if (mstar2133_pls_opened)
		 return -EBUSY;
	 mstar2133_pls_opened = 1;
	 return 0;
 }
 static int mstar2133_pls_release(struct inode *inode, struct file *file)
 {
	 printk("%s", __func__);
	 mstar2133_pls_opened = 0;
	 return 0;//ap3212c_pls_disable(AP3212C_PLS_BOTH);
 }
static int mstar2133_pls_unlocked_ioctl(struct file *file, unsigned int cmd,unsigned long arg)
 {
	 void __user *argp = (void __user *)arg;
	 printk("%s: cmd %d", __func__, _IOC_NR(cmd));
	 switch (cmd) {
	 case 1:
		 printk("%s: HQALSPS_IOCTL_PROX_ON*************************\n", __func__);
		 mstar2133_pls_enable();
		 break;
	 case 2:
		 printk("%s: HQALSPS_IOCTL_PROX_OFF*************************\n", __func__);
		 mstar2133_pls_disable();
		 break;
	 default:
		 printk("%s: invalid cmd %d\n", __func__, _IOC_NR(cmd));
		 return -EINVAL;
	 }
	 return 0;
 }

 static struct file_operations mstar2133_pls_fops = {
	 .owner 		 = THIS_MODULE,
	 .open			 = mstar2133_pls_open,
	 .release		 = mstar2133_pls_release,
	 .unlocked_ioctl	 = mstar2133_pls_unlocked_ioctl,
 };
 static struct miscdevice mstar2133_pls_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = HQALSPS_DEVICE_NAME,
	.fops = &mstar2133_pls_fops,
};

int  get_msg2133_data(void )
{
	return  ps_state;
}

#endif

 static void msg21xx_data_disposal(void)
 {
       u8 val[8] = {0};
       u8 Checksum = 0;
	u8 i;
	u32 delta_x = 0, delta_y = 0;
	u32 u32X = 0;
	u32 u32Y = 0;
	u8 touchkeycode = 0;
	TouchScreenInfo_t  touchData;
	static u32 preKeyStatus=0;
	//static u32 preFingerNum=0;
	
#ifdef SOUL4_KK_SUPPORT
#else
#define SWAP_X_Y   (1)
#define REVERSE_X  (1)
#define REVERSE_Y  (1)
#endif
#ifdef SWAP_X_Y
	int tempx;
	int tempy;
#endif


	//printk("*************msg21xx_data_disposal\n");
	i2c_master_recv(i2c_clientma,&val[0],REPORT_PACKET_LENGTH);
	Checksum = Calculate_8BitsChecksum(&val[0], 7); //calculate checksum
	if ((Checksum == val[7]) && (val[0] == 0x52))   //check the checksum  of packet
	{
		u32X = (((val[1] & 0xF0) << 4) | val[2]);         //parse the packet to coordinates
		u32Y = (((val[1] & 0x0F) << 8) | val[3]);

		delta_x = (((val[4] & 0xF0) << 4) | val[5]);
		delta_y = (((val[4] & 0x0F) << 8) | val[6]);

#ifdef SWAP_X_Y
		tempy = u32X;
		tempx = u32Y;
	        u32X = tempx;
	        u32Y = tempy;

		tempy = delta_x;
		tempx = delta_y;
	        delta_x = tempx;
	        delta_y = tempy;
#endif
#ifdef REVERSE_X
		u32X = 2047 - u32X;
		delta_x = 4095 - delta_x;
#endif
#ifdef REVERSE_Y
		u32Y = 2047 - u32Y;
		delta_y = 4095 - delta_y;
#endif
 //  printk ("[HAL] u32X = %x, u32Y = %x", u32X, u32Y);
//  printk ("[HAL] delta_x = %x, delta_y = %x", delta_x, delta_y);

	if ((val[1] == 0xFF) && (val[2] == 0xFF) && (val[3] == 0xFF) && (val[4] == 0xFF) && (val[6] == 0xFF))
	{
		touchData.Point[0].X = 0; // final X coordinate
		touchData.Point[0].Y = 0; // final Y coordinate

		if((val[5]==0x0)||(val[5]==0xFF))
		{
			touchData.nFingerNum = 0; //touch end
			touchData.nTouchKeyCode = 0; //TouchKeyMode
			touchData.nTouchKeyMode = 0; //TouchKeyMode
		}
		else
		{
#ifdef HQ_CTP_PS_TEST
			if(0x80 == val[5])
			{	
				ps_state = 0;
				printk("[MSG2133A] ps_state=%d\n", ps_state);//add zy
				return 1;
			}
			else if (0x40 == val[5])
			{	
				ps_state = 1;
				printk("[MSG2133A] ps_state=%d\n", ps_state);//add zy
				return 1;
			}	
#endif
			touchData.nTouchKeyMode = 1; //TouchKeyMode
			touchData.nTouchKeyCode = val[5]; //TouchKeyCode
			touchData.nFingerNum = 1;
		//	printk("key***%d****wwl****\n",val[5]);
		//	printk("finger num X= %d\n",touchData.Point[0].X);
		//	printk("finger num Y= %d\n",touchData.Point[0].Y);

		}
	}
	else
	{
		touchData.nTouchKeyMode = 0; //Touch on screen...

		if(
		   #ifdef REVERSE_X
		       (delta_x == 4095)
		   #else
		       (delta_x == 0) 
		   #endif
		   && 
		   #ifdef REVERSE_Y
		       (delta_y == 4095)
		   #else
		       (delta_y == 0)
		   #endif
		  )
			{
				touchData.nFingerNum = 1; //one touch
				touchData.Point[0].X = (u32X * MS_TS_MSG21XX_X_MAX) / 2048;
				touchData.Point[0].Y = (u32Y * MS_TS_MSG21XX_Y_MAX) / 2048;
			}
			else
			{
				u32 x2, y2;

				touchData.nFingerNum = 2; //two touch

				/* Finger 1 */
				touchData.Point[0].X = (u32X * MS_TS_MSG21XX_X_MAX) / 2048;
				touchData.Point[0].Y = (u32Y * MS_TS_MSG21XX_Y_MAX) / 2048;

				/* Finger 2 */
				if (delta_x > 2048)     //transform the unsigh value to sign value
				{
					delta_x -= 4096;
				}
				if (delta_y > 2048)
				{
					delta_y -= 4096;
				}

				x2 = (u32)(u32X + delta_x);
				y2 = (u32)(u32Y + delta_y);

				touchData.Point[1].X = (x2 * MS_TS_MSG21XX_X_MAX) / 2048;
				touchData.Point[1].Y = (y2 * MS_TS_MSG21XX_Y_MAX) / 2048;
			}
		}
		
		//report...
		if(touchData.nTouchKeyMode)
		{
			printk("[MSG2138A]useful key code report touch key code = %d\n",touchData.nTouchKeyCode);
			if (touchData.nTouchKeyCode == 1)
				{
					#ifdef SOUL4_KK_SUPPORT
					touchkeycode = KEY_MENU;
					touchData.Point[0].X=60;	 //change key code to point ABS by wwl
					touchData.Point[0].Y=870;
					#else
					touchkeycode = KEY_MENU;
					touchData.Point[0].X=265;     //change key code to point ABS by wwl
					touchData.Point[0].Y=510;	
					#endif
				}
			
			if (touchData.nTouchKeyCode == 2)
				{
					#ifdef SOUL4_KK_SUPPORT
					touchkeycode = KEY_HOMEPAGE;
					touchData.Point[0].X=170;	 //change key code to point ABS by wwl
					touchData.Point[0].Y=870;
					#else
					touchkeycode =KEY_HOMEPAGE;
					touchData.Point[0].X=159;
					touchData.Point[0].Y=510;	
					#endif
				}
			if (touchData.nTouchKeyCode == 4)
				{
					#ifdef SOUL4_KK_SUPPORT
					touchkeycode = KEY_BACK;
					touchData.Point[0].X=300;	 //change key code to point ABS by wwl
					touchData.Point[0].Y=870;
					#else
					touchkeycode =KEY_BACK;
					touchData.Point[0].X=53;
					touchData.Point[0].Y=510;
					#endif
				}
			if (touchData.nTouchKeyCode == 8)
				touchkeycode = KEY_SEARCH;
		//PR623092 change key code to report ABS value
		/*if(preKeyStatus!=touchkeycode)
			{
				preKeyStatus=touchkeycode;
				input_report_key(tpd->dev, touchkeycode, 1);
			}

			input_sync(tpd->dev);
			*/
		}
        //else
        {
		preKeyStatus=0; //clear key status..
//		printk("finger num = %d\n",touchData.nFingerNum);
            if((touchData.nFingerNum) == 0)   //touch end
            {

			if (FACTORY_BOOT == get_boot_mode()|| RECOVERY_BOOT == get_boot_mode())
				{   
					printk("factory mode \n");
					input_report_key(tpd->dev,BTN_TOUCH,0);
			
				}
			//preFingerNum=0;
				/*input_report_key(tpd->dev, KEY_BACK , 0);*/
				/*input_report_key(tpd->dev, KEY_HOME, 0);*/
				/*input_report_key(tpd->dev, KEY_MENU, 0);*/
				/*input_report_key(tpd->dev, KEY_SEARCH, 0);*/
				/*input_report_key(tpd->dev, KEY_HOMEPAGE, 0);*/
				input_report_key(tpd->dev, BTN_TOUCH, 0);
				input_report_abs(tpd->dev, ABS_PRESSURE, 0);
				
				/*input_report_abs(tpd->dev, ABS_MT_TOUCH_MAJOR, 0);*/
				input_mt_sync(tpd->dev);
				input_sync(tpd->dev);
            }
            else //touch on screen
            {
			    /*
				if(preFingerNum!=touchData.nFingerNum)   //for one touch <--> two touch issue
				{
					printk("langwenlong number has changed\n");
					preFingerNum=touchData.nFingerNum;
					input_report_abs(tpd->dev, ABS_MT_TOUCH_MAJOR, 0);
				    input_mt_sync(tpd->dev);
				    input_sync(tpd->dev);
				}*/

				for(i = 0;i < (touchData.nFingerNum);i++)
				{
					if (FACTORY_BOOT == get_boot_mode()|| RECOVERY_BOOT == get_boot_mode())
						{   
							printk("factory mode \n");
							input_report_key(tpd->dev,BTN_TOUCH,1);
			
						}
					input_report_abs(tpd->dev, ABS_MT_TOUCH_MAJOR, 100);
	 				input_report_abs(tpd->dev, ABS_PRESSURE, 100);
	 				input_report_key(tpd->dev, BTN_TOUCH, 1);
					input_report_abs(tpd->dev, ABS_MT_POSITION_X, touchData.Point[i].X);
					input_report_abs(tpd->dev, ABS_MT_POSITION_Y, touchData.Point[i].Y);
					input_mt_sync(tpd->dev);
					//printk("finger num X= %d Y= %d\n",touchData.Point[i].X,touchData.Point[i].Y);
				}					
				input_sync(tpd->dev);
			}
		}
    }
    else
    {
        printk("Packet error 0x%x, 0x%x, 0x%x\n", val[0], val[1], val[2]);
        printk("             0x%x, 0x%x, 0x%x\n", val[3], val[4], val[5]);
        printk("             0x%x, 0x%x, 0x%x\n", val[6], val[7], Checksum);
	printk(KERN_ERR "err status in tp\n");
    }
 }


#if defined(CTP_CHECK_FM)
void tpd_get_fm_frequency(int16_t freq) {
		fm_current_frequency = freq;
}
EXPORT_SYMBOL(tpd_get_fm_frequency);
#endif


 static int touch_event_handler(void *unused)
 {
	 struct sched_param param = { .sched_priority = RTPM_PRIO_TPD };
	 sched_setscheduler(current, SCHED_RR, &param);
 
	 do
	 {
	  mt65xx_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM); 
		 set_current_state(TASK_INTERRUPTIBLE); 
		  wait_event_interruptible(waiter,tpd_flag!=0);		 
		tpd_flag = 0;
		 set_current_state(TASK_RUNNING);
		 msg21xx_data_disposal();
 }while(!kthread_should_stop());
 
	 return 0;
 }
 
 static int tpd_detect (struct i2c_client *client, int kind, struct i2c_board_info *info) 
 {
	 printk("[MSG2138A]-------------------------------------- tpd_detect \n",__func__);
	 strcpy(info->type, TPD_DEVICEMA);	
	 return 0;
 }
 
 static void tpd_eint_interrupt_handler(void)
 {
	// TPD_DEBUG("TPD interrupt has been triggered\n");
	 tpd_flag = 1;
	 wake_up_interruptible(&waiter);
	 
 }

static int FW_repair_detect (void)
{
    u8 i;
    u8 dbbus_tx_data[4];
    unsigned char dbbus_rx_data[2] = {0};

    printk("[MSG2138A]--------------------------------------FW_repair_detect\n");
    mt65xx_eint_mask(CUST_EINT_TOUCH_PANEL_NUM);

    _HalTscrHWReset();
    i2c_clientma->timing = 50;

    // Erase TP Flash first
    dbbusDWIICEnterSerialDebugMode();
    dbbusDWIICStopMCU();
    dbbusDWIICIICUseBus();
    dbbusDWIICIICReshape();
    mdelay ( 300 );

    // Disable the Watchdog
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x3C;
    dbbus_tx_data[2] = 0x60;
    dbbus_tx_data[3] = 0x55;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x3C;
    dbbus_tx_data[2] = 0x61;
    dbbus_tx_data[3] = 0xAA;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );
    // Stop MCU
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x0F;
    dbbus_tx_data[2] = 0xE6;
    dbbus_tx_data[3] = 0x01;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );
    /////////////////////////
    // Difference between C2 and C3
    /////////////////////////
    // c2:2133 c32:2133a(2) c33:2133a
    //check id
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0xCC;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 3 );
    HalTscrCReadI2CSeq ( FW_ADDR_MSG21XX, &dbbus_rx_data[0], 2 );
    if ( dbbus_rx_data[0] == 2 )
    {
        // check version
        dbbus_tx_data[0] = 0x10;
        dbbus_tx_data[1] = 0x3C;
        dbbus_tx_data[2] = 0xEA;
        HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 3 );
        HalTscrCReadI2CSeq ( FW_ADDR_MSG21XX, &dbbus_rx_data[0], 2 );
        TP_DEBUG ( "dbbus_rx version[0]=0x%x   \n", dbbus_rx_data[0] );

        if ( dbbus_rx_data[0] == 3 ){
		drvTP_read_info_dwiic_c33();
	}
    }
    _HalTscrHWReset();
    i2c_clientma->timing = 100;
    i2c_clientma->addr = FW_ADDR_MSG21XX_TP;
    return 0;
}

static int Upgrade_detect()
{
	unsigned short wanted_major_version=0,wanted_minor_version=0;

	printk("[MSG2138A]--------------------------------------Upgrade_detect\n");

	if(fw_major_version==2)//junda
	{
		wanted_major_version= (MSG_FIRMWARE_JD[0x7f4f] << 8) + MSG_FIRMWARE_JD[0x7f4e];
		wanted_minor_version= (MSG_FIRMWARE_JD[0x7f51] << 8) + MSG_FIRMWARE_JD[0x7f50];
	}

	else if(fw_major_version==5) //EACH
	{
		//TPMSG="EACH";
		wanted_major_version= (MSG_FIRMWARE_EA[0x7f4f] << 8) + MSG_FIRMWARE_EA[0x7f4e];
		wanted_minor_version= (MSG_FIRMWARE_EA[0x7f51] << 8) + MSG_FIRMWARE_EA[0x7f50];
	}

	else 
	{
		FW_repair_detect();
		if(fw_major_version==2)//junda
		{
			wanted_major_version= (MSG_FIRMWARE_JD[0x7f4f] << 8) + MSG_FIRMWARE_JD[0x7f4e];
			wanted_minor_version= (MSG_FIRMWARE_JD[0x7f51] << 8) + MSG_FIRMWARE_JD[0x7f50];
		}
	
		else if(fw_major_version==5) //EACH
		{
			//TPMSG="EACH";
			wanted_major_version= (MSG_FIRMWARE_EA[0x7f4f] << 8) + MSG_FIRMWARE_EA[0x7f4e];
			wanted_minor_version= (MSG_FIRMWARE_EA[0x7f51] << 8) + MSG_FIRMWARE_EA[0x7f50];
		}
	
		else if(fw_major_version == 0xFFFF && fw_minor_version == 0xFFFF) //only for soul35:when touchpanel firmware upgrade failed, the fw_major_version and fw_minor_version turn into 0xFFFF.
		{
			printk("[MSG2138A]---------YUELI:force to upgrade\n");
			firmware_updata();//force to upgrade
			return 0;
		}

		else
		{
			return -1;
		}
	}

	printk("[MSG2138A][File_fw_version ] major = %d ,minor = %d ************************************\n",wanted_major_version,wanted_minor_version);
	printk("[MSG2138A][CTP_fw_version] major = %d ,minor = %d ************************************\n",fw_major_version,fw_minor_version);

	if(wanted_major_version==fw_major_version && wanted_minor_version>fw_minor_version)//version detect
		firmware_updata();//force to upgrade
	return 0;
}
 
 static int __devinit tpd_probe(struct i2c_client *client, const struct i2c_device_id *id)
 {	 
	printk("[MSG2138A]--------------------------------------tpd_probe\n",__func__);
	int retval = TPD_OK;
	char data;
	unsigned char dbbus_tx_data[3];
	unsigned char dbbus_rx_data[4];
	i2c_clientma = client;

	unsigned short wanted_major_version=0,wanted_minor_version=0;

        mt_set_gpio_mode(GPIO_CTP_EINT_PIN, GPIO_CTP_EINT_PIN_M_EINT);
        mt_set_gpio_dir(GPIO_CTP_EINT_PIN, GPIO_DIR_IN);
        mt_set_gpio_pull_enable(GPIO_CTP_EINT_PIN, GPIO_PULL_ENABLE);
        mt_set_gpio_pull_select(GPIO_CTP_EINT_PIN, GPIO_PULL_UP);

        mt65xx_eint_set_sens(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_SENSITIVE);
        mt65xx_eint_set_hw_debounce(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_DEBOUNCE_CN);
        mt65xx_eint_registration(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_DEBOUNCE_EN, 1, tpd_eint_interrupt_handler, 1);
        mt65xx_eint_mask(CUST_EINT_TOUCH_PANEL_NUM);
	mdelay(100);

	//power on
	mt_set_gpio_mode(GPIO_CTP_EN_PIN, GPIO_CTP_EN_PIN_M_GPIO);
	mt_set_gpio_dir(GPIO_CTP_EN_PIN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_CTP_EN_PIN, GPIO_OUT_ONE);
	mt_set_gpio_out(GPIO_CTP_EN_PIN, GPIO_OUT_ZERO);
	msleep(5);
	hwPowerOn(MT65XX_POWER_LDO_VGP2, VOL_2800, "TP");
	msleep(5);
	mt_set_gpio_out(GPIO_CTP_EN_PIN, GPIO_OUT_ONE);
	msleep(500);
	I2cDMA_init();

	if((i2c_smbus_read_i2c_block_data(i2c_clientma, 0x00, 1, &data))< 0)
	   {
		   printk("[CTP-I2C-ERROR] [MSG2138A]transfer error addr=0x%x------llf, line: %d\n", i2c_clientma->addr,__LINE__);
		  retval = Upgrade_detect();
		  if(retval<0){
		   	return -1; 
		  }
		  else
		  {
			tpd_load_status = 1;
			printk("[MSG2138A][The CTP is MSG2138A ---llf] OK!**********\n");
			I2cDMA_exit();
		  }
		//return -1;
	   }

	/*frameware upgrade*/

	if(tpd_load_status==0){	
		dbbusDWIICEnterSerialDebugMode();
		dbbusDWIICStopMCU();
		dbbusDWIICIICUseBus();
		dbbusDWIICIICReshape();
		mdelay ( 300 );
		dbbus_tx_data[0] = 0x10;
		dbbus_tx_data[1] = 0x1E;
		dbbus_tx_data[2] = 0xCC;
		HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 3 );
		HalTscrCReadI2CSeq ( FW_ADDR_MSG21XX, &dbbus_rx_data[0], 2 );
		printk("[MSG2138A]The CTP ID ---:0x%x !**********\n",dbbus_rx_data[0]);
		if ( dbbus_rx_data[0] == 2 )
		{
			// check version
			dbbus_tx_data[0] = 0x10;
			dbbus_tx_data[1] = 0x3C;
			dbbus_tx_data[2] = 0xEA;
			HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 3 );
			HalTscrCReadI2CSeq ( FW_ADDR_MSG21XX, &dbbus_rx_data[0], 2 );
			TP_DEBUG ( "[MSG2138A] dbbus_rx version[0]=0x%x \n", dbbus_rx_data[0] );

			if ( dbbus_rx_data[0] == 3 ){
				tpd_load_status = 1;
				printk("[MSG2138A][The CTP is MSG2138A ---llf] OK!**********\n");
#ifdef __FIRMWARE_UPDATE__
				firmware_version();
				Upgrade_detect();
#endif
				I2cDMA_exit();
			}
			else
			{
				I2cDMA_exit();
				printk("[MSG2138A][The CTP is  Not MSG2138A ---llf] %s OK!**********\n");
				return -1;
			}
		}
		else
		{
			I2cDMA_exit();
			printk("[MSG2138A][The CTP is  Not MSG2138A ---llf] %s OK!**********\n");
			return -1;
		}
	}
#ifdef APK_UPGRADE
	firmware_class = class_create(THIS_MODULE, "ms-touchscreen-msg20xx");
	if (IS_ERR(firmware_class))
		pr_err("Failed to create class(firmware)!\n");
	firmware_cmd_dev = device_create(firmware_class,
	                             NULL, 0, NULL, "device");
	if (IS_ERR(firmware_cmd_dev))
		pr_err("Failed to create device(firmware_cmd_dev)!\n");

	// versions
	if (device_create_file(firmware_cmd_dev, &dev_attr_versions) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_version.attr.name);
	// update
	if (device_create_file(firmware_cmd_dev, &dev_attr_update) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_update.attr.name);
	// data
	if (device_create_file(firmware_cmd_dev, &dev_attr_data) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_data.attr.name);
	dev_set_drvdata(firmware_cmd_dev, NULL);
#endif

#ifdef HQ_CTP_PS_TEST
	tp_pls_status = 0;
	retval = misc_register(&mstar2133_pls_device);
	if (retval != 0) {
		printk("cannot register miscdev on mstar2133_pls_device\n");
	}
#endif
	mdelay(100);//reduce waiting time to speed up booting up add by wwl   

	input_set_abs_params(tpd->dev, ABS_MT_POSITION_X, 0, TPD_RES_X, 0, 0);

	mthread = kthread_run(touch_event_handler, 0, TPD_DEVICEMA);
	 if (IS_ERR(mthread))
		 { 
		  retval = PTR_ERR(mthread);
		  TPD_DMESG(TPD_DEVICEMA " failed to create kernel mthread: %d\n", retval);
		}
#ifdef ITO_TEST
    ito_test_create_entry();
#endif
	if(device_create_file(&i2c_clientma->dev, &dev_attr_versions)>=0)
	{            
		printk("device_create_file_version \n");        
	}
	TPD_DMESG("Touch Panel Device Probe %s\n", (retval < TPD_OK) ? "FAIL" : "PASS");
	mt65xx_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);
	tpd_load_status = 1;
	return 0;
   
 }

 static int __devexit tpd_remove(struct i2c_client *client)
 {
   
	 TPD_DEBUG("TPD removed\n");
 	return 0;
 }
 
 
 static int tpd_local_init(void)
 {
	printk("[MSG2138A]--------------------------------------tpd_local_init\n",__func__);
	if(i2c_add_driver(&tpd_i2c_driver)!=0)
	{
			TPD_DMESG("unable to add i2c driver.\n");
	  		return -1;
	}
	if(tpd_load_status==0)  
	{
		TPD_DMESG("[MSG2138A] add i2c driver faild.\n");
		i2c_del_driver(&tpd_i2c_driver);
		return -1;
	}
  
#if (defined(TPD_WARP_START) && defined(TPD_WARP_END))    
	TPD_DO_WARP = 1;
	memcpy(tpd_wb_start, tpd_wb_start_local, TPD_WARP_CNT*4);
	memcpy(tpd_wb_end, tpd_wb_start_local, TPD_WARP_CNT*4);
#endif 

#ifdef TPD_HAVE_BUTTON     
	tpd_button_setting(TPD_KEY_COUNT, tpd_keys_local, tpd_keys_dim_local);// initialize tpd button data
#endif   

#if (defined(TPD_HAVE_CALIBRATION) && !defined(TPD_CUSTOM_CALIBRATION))
	memcpy(tpd_calmat, tpd_def_calmat_local, 8*4);
	memcpy(tpd_def_calmat, tpd_def_calmat_local, 8*4);	
#endif  
	TPD_DMESG("end %s, %d\n", __FUNCTION__, __LINE__);  
	tpd_type_cap = 1;
	return 0; 
 }

 static int tpd_resume(struct i2c_client *client)
 {
	int retval = TPD_OK;
 
	TPD_DEBUG("TPD wake up\n");
	mt_set_gpio_mode(GPIO_CTP_EN_PIN, GPIO_CTP_EN_PIN_M_GPIO);
	mt_set_gpio_dir(GPIO_CTP_EN_PIN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_CTP_EN_PIN, GPIO_OUT_ONE);
	mt_set_gpio_out(GPIO_CTP_EN_PIN, GPIO_OUT_ZERO);
	msleep(5);
	hwPowerOn(MT65XX_POWER_LDO_VGP2, VOL_2800, "TP");
	msleep(5);
	mt_set_gpio_out(GPIO_CTP_EN_PIN, GPIO_OUT_ONE);
	msleep(500);
	mt65xx_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);  
 #ifdef HQ_CTP_PS_TEST

	if(mstar2133_pls_opened)
		{
			msg2133_ps_mode_enable(1);
			printk("==%s= mstar2133_pls_opened ! return\n", __func__);
		}
#endif

	
	 return retval;
 }
 
 static int tpd_suspend(struct i2c_client *client, pm_message_t message)
 {
	 int retval = TPD_OK;
 
	TPD_DEBUG("TPD enter sleep\n");
#ifdef HQ_CTP_PS_TEST
 
	if(mstar2133_pls_opened)
	{
		printk("==%s== mstar2133_pls_opened ! return \n", __func__);
		return;
	}
	
#endif
	mt65xx_eint_mask(CUST_EINT_TOUCH_PANEL_NUM);

	mt_set_gpio_mode(GPIO_CTP_EN_PIN, GPIO_CTP_EN_PIN_M_GPIO);
	mt_set_gpio_dir(GPIO_CTP_EN_PIN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_CTP_EN_PIN, GPIO_OUT_ONE);
	mt_set_gpio_out(GPIO_CTP_EN_PIN, GPIO_OUT_ZERO);
	msleep(5);
	hwPowerDown(MT65XX_POWER_LDO_VGP2,"TP");
	msleep(5);

	return retval;
 } 


 static struct tpd_driver_t tpd_device_driver =
 {
		 .tpd_device_name = "MSG2133A",
		 .tpd_local_init = tpd_local_init,
		 .suspend = tpd_suspend,
		 .resume = tpd_resume,
#ifdef TPD_HAVE_BUTTON
		 .tpd_have_button = 1,
#else
		 .tpd_have_button = 0,
#endif		
 };
 /* called when loaded into kernel */
 static int __init tpd_driver_init(void)
 {
	 printk("MediaTek msg2138a touch panel driver init\n");
	 i2c_register_board_info(1, &i2c_tpd, 1);
	 if(tpd_driver_add(&tpd_device_driver) < 0)
         	TPD_DMESG("add msg21xx driver failed\n");
	 return 0;
 }
 
 /* should never be called */
 static void __exit tpd_driver_exit(void)
 {
	 TPD_DMESG("MediaTek msg2138a touch panel driver exit\n");
	 //input_unregister_device(tpd->dev);
	 tpd_driver_remove(&tpd_device_driver);
 }
 
 module_init(tpd_driver_init);
 module_exit(tpd_driver_exit);



