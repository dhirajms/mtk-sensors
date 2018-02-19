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
#include <linux/dma-mapping.h>

#include "tpd_custom_ft53x6t.h"
//#ifdef MT6575
//#include <mach/mt6575_pm_ldo.h>
//#include <mach/mt6575_typedefs.h>
//#include <mach/mt6575_boot.h>
//#endif

#ifdef MT6575
#include <mach/mt6575_pm_ldo.h>
#include <mach/mt6575_typedefs.h>
#include <mach/mt6575_boot.h>
#else
#include <mach/mt_pm_ldo.h>
#include <mach/mt_typedefs.h>
#include <mach/mt_boot.h>
#endif

#include "cust_gpio_usage.h"
#include "ft_psensor_drv.h"
#include "ft6x06_ex_fun.h"
// add  for m536
//#define FTS_CTL_FACE_DETECT
#ifdef FTS_CTL_FACE_DETECT
#include <linux/hwmsensor.h>
#include <linux/hwmsen_dev.h>

#include "ft_psensor_drv.h"
#include <linux/wakelock.h>

#define FT_FACE_DETECT_POS		1
#define FT_FACE_DETECT_ENABLE	1
#define FT_FACE_DETECT_DISABLE	0
#define FT_FACE_DETECT_REG		0xB0

#define FT_FACE_DETECT_ON		0xc0
#define FT_FACE_DETECT_OFF		0xe0


static unsigned char face_detect_enable=0;
static unsigned char  tp_suspend_flag = 0;

static unsigned char is_face_detect=0;
static unsigned char face_detect_value=0;
static struct wake_lock ps_lock;
static unsigned char tpd_suspend_status=0;


#define  FT_PSENSOR_DRV  "ft_psensor_drv"
#define FT_PSENSOR_DRV_MAJOR 211    /*预设的FT_PSENSOR_DRV的主设备号*/

#define FT_PSENSOR_IOCTL_ENABLE          	11
#define FT_PSENSOR_IOCTL_GET_ENABLE          	12

#define PSENSOR_ENABLE_REG		0xB0	
#define ENABLE_PSENSOR		1
#define DISABLE_PSENSOR 	0


#endif

// end

static DEFINE_MUTEX(ps_operator_lock);


#define FTS_CTL_IIC
#ifdef FTS_CTL_IIC
#include "focaltech_ctl.h"
extern int ft_rw_iic_drv_init(struct i2c_client *client);
extern void  ft_rw_iic_drv_exit(void);
#endif


extern struct tpd_device *tpd;
 
static struct i2c_client *i2c_client = NULL;
static struct task_struct *thread = NULL;

static DECLARE_WAIT_QUEUE_HEAD(waiter);
 
 
static void tpd_eint_interrupt_handler(void);
 
#ifdef MT6575 
 extern void mt65xx_eint_unmask(unsigned int line);
 extern void mt65xx_eint_mask(unsigned int line);
 extern void mt65xx_eint_set_hw_debounce(kal_uint8 eintno, kal_uint32 ms);
 extern kal_uint32 mt65xx_eint_set_sens(kal_uint8 eintno, kal_bool sens);
 extern void mt65xx_eint_registration(kal_uint8 eintno, kal_bool Dbounce_En,
									  kal_bool ACT_Polarity, void (EINT_FUNC_PTR)(void),
									  kal_bool auto_umask);
#endif
#ifdef MT6577
	extern void mt65xx_eint_unmask(unsigned int line);
	extern void mt65xx_eint_mask(unsigned int line);
	extern void mt65xx_eint_set_hw_debounce(unsigned int eint_num, unsigned int ms);
	extern unsigned int mt65xx_eint_set_sens(unsigned int eint_num, unsigned int sens);
	extern void mt65xx_eint_registration(unsigned int eint_num, unsigned int is_deb_en, unsigned int pol, void (EINT_FUNC_PTR)(void), unsigned int is_auto_umask);
#endif
//zhaoshaopeng add for ft driver firmware update 20120903 start
#define FT_FM_UPDATE
#ifdef FT_FM_UPDATE
extern int fts_ctpm_fw_upgrade_with_i_file_ctp1(struct i2c_client *client);
extern int fts_ctpm_fw_upgrade_with_i_file_ctp2(struct i2c_client *client);
extern int ft5x0x_read_reg(struct i2c_client *client, u8 regaddr, u8 *regvalue);
extern u8 fts_ctpm_get_i_file_ver_ctp1(void);
extern u8 fts_ctpm_get_i_file_ver_ctp2(void);

u8 uc_factory_id = 0;
 struct task_struct *fw_update_thread = NULL;

int fts_ctpm_auto_upgrade(struct i2c_client *client)
{
	u8 uc_host_fm_ver = FT5x0x_REG_FW_VER;
	u8 uc_tp_fm_ver;
	u8 id;
	int i_ret;

   // ?y?¨2¨o1¨??1¨??t????à?o?
	ft5x0x_read_reg(client, FT5x0x_REG_FW_VER, &uc_tp_fm_ver);
    msleep(5);
	// ?¨′?Y3?ì¨|¨?id?D??1¨??t????à?o??ê??y??￥??????à?o????¨°¨|y??
	if (FTS_CTP_VENDOR_CTP1 == uc_factory_id)
	{
		uc_host_fm_ver = fts_ctpm_get_i_file_ver_ctp1();
		printk("[FTS] CTP1 tp uc_tp_fm_ver = 0x%x, uc_host_fm_ver = 0x%x\n", uc_tp_fm_ver, uc_host_fm_ver);

		if (uc_tp_fm_ver == FT5x0x_REG_FW_VER || uc_tp_fm_ver < uc_host_fm_ver) 
		{
			msleep(100);
			i_ret = fts_ctpm_fw_upgrade_with_i_file_ctp1(client);
			if (i_ret == 0)	
			{
				msleep(300);
				uc_host_fm_ver = fts_ctpm_get_i_file_ver_ctp1();
				printk("\r\n [FTS] upgrade to new version 0x%x; stop fw_update_thread \n",
				uc_host_fm_ver);
				kthread_stop(fw_update_thread);
			}
			else 
			{
				printk("\r\n [FTS] upgrade failed ret=%d.and stop fw_update_thread\n", i_ret);
				kthread_stop(fw_update_thread); 
				return -EIO;
			}
		}
	       kthread_stop(fw_update_thread);//zhaoshaopeng add 
	       return 0;
	}
	else if (FTS_CTP_VENDOR_CTP2 == uc_factory_id)
	{
	    uc_host_fm_ver = fts_ctpm_get_i_file_ver_ctp2();
		printk("[FTS] CTP2 tp uc_tp_fm_ver = 0x%x, uc_host_fm_ver = 0x%x\n", uc_tp_fm_ver, uc_host_fm_ver);

		if (uc_tp_fm_ver == FT5x0x_REG_FW_VER || uc_tp_fm_ver < uc_host_fm_ver) 
		{
		    msleep(100);
			i_ret = fts_ctpm_fw_upgrade_with_i_file_ctp2(client);
			if (i_ret == 0)	
			{
				msleep(300);
				uc_host_fm_ver = fts_ctpm_get_i_file_ver_ctp2();
				printk("\r\n [FTS] upgrade to new version 0x%x; stop fw_update_thread \n",
				uc_host_fm_ver);
				kthread_stop(fw_update_thread);
			}
			else 
			{
				printk("\r\n [FTS] upgrade failed ret=%d.and stop fw_update_thread\n", i_ret);
				kthread_stop(fw_update_thread); 
				return -EIO;
			}
		}
	       kthread_stop(fw_update_thread);//zhaoshaopeng add 
	       return 0;
	}
	else
	{
	    printk("[FTS] fts_ctpm_auto_upgrade unknow factory id =  0x%x, return -EIO\n", uc_factory_id);
        return -EIO;
	}
	
	return 0;

}

//zhaoshaopeng start
 u8  fts_fw_auto_update(struct i2c_client *client)
 {	
    printk("\r\n zhaoshaopeng fw_upgrade_with_i_file Ready to run update thread");
    //thread = kthread_run(fts_ctpm_fw_upgrade_with_i_file, (void *)NULL, "fw_update");
    fw_update_thread = kthread_run(fts_ctpm_auto_upgrade, (void *)i2c_client, "fw_update");
    
    if (IS_ERR(fw_update_thread))
    {
        printk("\r\n zhaoshaopeng Failed to create update thread.\n");
        return -1;
    }
    return 0;
}
//zhaoshaopeng end

#endif
//zhaoshaopeng end

 // add yyf
 static void try_to_reset(void);
 
static int __devinit tpd_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int tpd_detect (struct i2c_client *client, struct i2c_board_info *info);
static int __devexit tpd_remove(struct i2c_client *client);
static int touch_event_handler(void *unused);
 
static int boot_mode = 0;
static int tpd_halt=0; 
static int tpd_flag = 0;
static int point_num = 0;
static int p_point_num = 0;
static bool discard_resume_first_eint = KAL_FALSE;
static int tpd_state = 0;
//#define TPD_CLOSE_POWER_IN_SLEEP

#define TPD_OK 0
//register define

#define DEVICE_MODE 0x00
#define GEST_ID 0x01
#define TD_STATUS 0x02

#define TOUCH1_XH 0x03
#define TOUCH1_XL 0x04
#define TOUCH1_YH 0x05
#define TOUCH1_YL 0x06

#define TOUCH2_XH 0x09
#define TOUCH2_XL 0x0A
#define TOUCH2_YH 0x0B
#define TOUCH2_YL 0x0C

#define TOUCH3_XH 0x0F
#define TOUCH3_XL 0x10
#define TOUCH3_YH 0x11
#define TOUCH3_YL 0x12

/*add for portugal evt*/
/* Lenovo-sw yexm1 modify, 2012-10-18, open the FW upgrade fun */
//#define CONFIG_SUPPORT_FTS_CTP_UPG

//#define ESD_CHECK //zhaoshaopeng delete for hw limit

#define TPD_RESET_ISSUE_WORKAROUND

#define TPD_MAX_RESET_COUNT 3

#ifdef ESD_CHECK
static struct delayed_work ctp_read_id_work;
static struct workqueue_struct * ctp_read_id_workqueue = NULL;
#endif

#ifdef TPD_HAVE_BUTTON 
static int tpd_keys_local[TPD_KEY_COUNT] = TPD_KEYS;
static int tpd_keys_dim_local[TPD_KEY_COUNT][4] = TPD_KEYS_DIM;
#endif
#if (defined(TPD_WARP_START) && defined(TPD_WARP_END))
static int tpd_wb_start_local[TPD_WARP_CNT] = TPD_WARP_START;
static int tpd_wb_end_local[TPD_WARP_CNT]   = TPD_WARP_END;
#endif
#if (defined(TPD_HAVE_CALIBRATION) && !defined(TPD_CUSTOM_CALIBRATION))
static int tpd_calmat_local[8]     = TPD_CALIBRATION_MATRIX;
static int tpd_def_calmat_local[8] = TPD_CALIBRATION_MATRIX;
#endif

//#define VELOCITY_CUSTOM_FT5206 //zhaoshaopeng close this, i dont know how does this work
#ifdef VELOCITY_CUSTOM_FT5206
#include <linux/device.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>

// for magnify velocity********************************************

#ifndef TPD_VELOCITY_CUSTOM_X
#define TPD_VELOCITY_CUSTOM_X 10
#endif
#ifndef TPD_VELOCITY_CUSTOM_Y
#define TPD_VELOCITY_CUSTOM_Y 10
#endif

#define TOUCH_IOC_MAGIC 'A'

#define TPD_GET_VELOCITY_CUSTOM_X _IO(TOUCH_IOC_MAGIC,0)
#define TPD_GET_VELOCITY_CUSTOM_Y _IO(TOUCH_IOC_MAGIC,1)

static int g_v_magnify_x =TPD_VELOCITY_CUSTOM_X;
static int g_v_magnify_y =TPD_VELOCITY_CUSTOM_Y;
static int tpd_misc_open(struct inode *inode, struct file *file)
{
/*
	file->private_data = adxl345_i2c_client;

	if(file->private_data == NULL)
	{
		printk("tpd: null pointer!!\n");
		return -EINVAL;
	}
	*/
	return nonseekable_open(inode, file);
}
/*----------------------------------------------------------------------------*/
static int tpd_misc_release(struct inode *inode, struct file *file)
{
	//file->private_data = NULL;
	return 0;
}
/*----------------------------------------------------------------------------*/
//static int adxl345_ioctl(struct inode *inode, struct file *file, unsigned int cmd,
//       unsigned long arg)
static long tpd_unlocked_ioctl(struct file *file, unsigned int cmd,
       unsigned long arg)
{
	//struct i2c_client *client = (struct i2c_client*)file->private_data;
	//struct adxl345_i2c_data *obj = (struct adxl345_i2c_data*)i2c_get_clientdata(client);	
	//char strbuf[256];
	void __user *data;
	
	long err = 0;
	
	if(_IOC_DIR(cmd) & _IOC_READ)
	{
		err = !access_ok(VERIFY_WRITE, (void __user *)arg, _IOC_SIZE(cmd));
	}
	else if(_IOC_DIR(cmd) & _IOC_WRITE)
	{
		err = !access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd));
	}

	if(err)
	{
		printk("tpd: access error: %08X, (%2d, %2d)\n", cmd, _IOC_DIR(cmd), _IOC_SIZE(cmd));
		return -EFAULT;
	}

	switch(cmd)
	{
		case TPD_GET_VELOCITY_CUSTOM_X:
			data = (void __user *) arg;
			if(data == NULL)
			{
				err = -EINVAL;
				break;	  
			}			
			
			if(copy_to_user(data, &g_v_magnify_x, sizeof(g_v_magnify_x)))
			{
				err = -EFAULT;
				break;
			}				 
			break;

	   case TPD_GET_VELOCITY_CUSTOM_Y:
			data = (void __user *) arg;
			if(data == NULL)
			{
				err = -EINVAL;
				break;	  
			}			
			
			if(copy_to_user(data, &g_v_magnify_y, sizeof(g_v_magnify_y)))
			{
				err = -EFAULT;
				break;
			}				 
			break;


		default:
			printk("tpd: unknown IOCTL: 0x%08x\n", cmd);
			err = -ENOIOCTLCMD;
			break;
			
	}

	return err;
}


static struct file_operations tpd_fops = {
//	.owner = THIS_MODULE,
	.open = tpd_misc_open,
	.release = tpd_misc_release,
	.unlocked_ioctl = tpd_unlocked_ioctl,
};
/*----------------------------------------------------------------------------*/
static struct miscdevice tpd_misc_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "touch",
	.fops = &tpd_fops,
};

//**********************************************
#endif

struct touch_info {
    int y[5];
    int x[5];
    int p[5];
	int id[5];
    int count;
};

 static const struct i2c_device_id ft5206_tpd_id[] = {{"ft5206",0},{}};
 //unsigned short force[] = {0,0x70,I2C_CLIENT_END,I2C_CLIENT_END}; 
 //static const unsigned short * const forces[] = { force, NULL };
 //static struct i2c_client_address_data addr_data = { .forces = forces, };
 static struct i2c_board_info __initdata ft5206_i2c_tpd={ I2C_BOARD_INFO("ft5206", (0x70>>1))};
 
 
 static struct i2c_driver tpd_i2c_driver = {
  .driver = {
	 .name = "ft5206",//.name = TPD_DEVICE,
//	 .owner = THIS_MODULE,
  },
  .probe = tpd_probe,
  .remove = __devexit_p(tpd_remove),
  .id_table = ft5206_tpd_id,
  .detect = tpd_detect,
//  .address_data = &addr_data,
 };
 #ifdef CONFIG_SUPPORT_FTS_CTP_UPG
static u8 *CTPI2CDMABuf_va = NULL;
static u32 CTPI2CDMABuf_pa = NULL;
typedef enum
{
    ERR_OK,
    ERR_MODE,
    ERR_READID,
    ERR_ERASE,
    ERR_STATUS,
    ERR_ECC,
    ERR_DL_ERASE_FAIL,
    ERR_DL_PROGRAM_FAIL,
    ERR_DL_VERIFY_FAIL
}E_UPGRADE_ERR_TYPE;

typedef unsigned char         FTS_BYTE;     //8 bit
typedef unsigned short        FTS_WORD;    //16 bit
typedef unsigned int          FTS_DWRD;    //16 bit
typedef unsigned char         FTS_BOOL;    //8 bit

#define FTS_NULL                0x0
#define FTS_TRUE                0x01
#define FTS_FALSE              0x0

#define I2C_CTPM_ADDRESS       0x70

/***********************************************************************************************
Name	:	ft5x0x_i2c_rxdata 

Input	:	*rxdata
                     *length

Output	:	ret

function	:	

***********************************************************************************************/
static int ft5x0x_i2c_rxdata(char *rxdata, int length)
{
	int ret;

	struct i2c_msg msgs[] = {
		{
			.addr	= i2c_client->addr,
			.flags	= 0,
			.len	= 1,
			.buf	= rxdata,
		},
		{
			.addr	= i2c_client->addr,
			.flags	= I2C_M_RD,
			.len	= length,
			.buf	= rxdata,
		},
	};

    //msleep(1);
	ret = i2c_transfer(i2c_client->adapter, msgs, 2);
	if (ret < 0)
		pr_err("msg %s i2c read error: %d\n", __func__, ret);
	
	return ret;
}
/***********************************************************************************************
Name	:	 

Input	:	
                     

Output	:	

function	:	

***********************************************************************************************/
static int ft5x0x_i2c_txdata(char *txdata, int length)
{
	int ret;

	struct i2c_msg msg[] = {
		{
			.addr	= i2c_client->addr,
			.flags	= 0,
			.len	= length,
			.buf	= txdata,
		},
	};

   	//msleep(1);
	ret = i2c_transfer(i2c_client->adapter, msg, 1);
	if (ret < 0)
		pr_err("%s i2c write error: %d\n", __func__, ret);

	return ret;
}
/***********************************************************************************************
Name	:	 ft5x0x_write_reg

Input	:	addr -- address
                     para -- parameter

Output	:	

function	:	write register of ft5x0x

***********************************************************************************************/
static int ft5x0x_write_reg(u8 addr, u8 para)
{
    u8 buf[3];
    int ret = -1;

    buf[0] = addr;
    buf[1] = para;
    ret = ft5x0x_i2c_txdata(buf, 2);
    if (ret < 0) {
        pr_err("write reg failed! %#x ret: %d", buf[0], ret);
        return -1;
    }
    
    return 0;
}


/***********************************************************************************************
Name	:	ft5x0x_read_reg 

Input	:	addr
                     pdata

Output	:	

function	:	read register of ft5x0x

***********************************************************************************************/
static int ft5x0x_read_reg(u8 addr, u8 *pdata)
{
	int ret;
	u8 buf[2] = {0};

	buf[0] = addr;
	struct i2c_msg msgs[] = {
		{
			.addr	= i2c_client->addr,
			.flags	= 0,
			.len	= 1,
			.buf	= buf,
		},
		{
			.addr	= i2c_client->addr,
			.flags	= I2C_M_RD,
			.len	= 1,
			.buf	= buf,
		},
	};

    //msleep(1);
	ret = i2c_transfer(i2c_client->adapter, msgs, 2);
	if (ret < 0)
		pr_err("msg %s i2c read error: %d\n", __func__, ret);

	*pdata = buf[0];
	return ret;
  
}


/***********************************************************************************************
Name	:	 ft5x0x_read_fw_ver

Input	:	 void
                     

Output	:	 firmware version 	

function	:	 read TP firmware version

***********************************************************************************************/
static unsigned char ft5x0x_read_fw_ver(void)
{
	unsigned char ver;
	ft5x0x_read_reg(0xa6, &ver);
	return(ver);
}
static unsigned char ft5x0x_read_ID_ver(void)
{
	unsigned char ver;
	ft5x0x_read_reg(0xa8, &ver);
	return(ver);
}

void delay_qt_ms(unsigned long  w_ms)
{
    unsigned long i;
    unsigned long j;

    for (i = 0; i < w_ms; i++)
    {
        for (j = 0; j < 1000; j++)
        {
            udelay(1);
        }
    }
}


/*
[function]: 
    callback: read data from ctpm by i2c interface,implemented by special user;
[parameters]:
    bt_ctpm_addr[in]    :the address of the ctpm;
    pbt_buf[out]        :data buffer;
    dw_lenth[in]        :the length of the data buffer;
[return]:
    FTS_TRUE     :success;
    FTS_FALSE    :fail;
*/
FTS_BOOL i2c_read_interface(FTS_BYTE bt_ctpm_addr, FTS_BYTE* pbt_buf, FTS_DWRD dw_lenth)
{
    int ret;
    
    ret=i2c_master_recv(i2c_client, pbt_buf, dw_lenth);

    if(ret<=0)
    {
        printk("[TSP]i2c_read_interface error\n");
        return FTS_FALSE;
    }
  
    return FTS_TRUE;
}

/*
[function]: 
    callback: write data to ctpm by i2c interface,implemented by special user;
[parameters]:
    bt_ctpm_addr[in]    :the address of the ctpm;
    pbt_buf[in]        :data buffer;
    dw_lenth[in]        :the length of the data buffer;
[return]:
    FTS_TRUE     :success;
    FTS_FALSE    :fail;
*/
FTS_BOOL i2c_write_interface(FTS_BYTE bt_ctpm_addr, FTS_BYTE* pbt_buf, FTS_DWRD dw_lenth)
{
    int ret;
    ret=i2c_master_send(i2c_client, pbt_buf, dw_lenth);
    if(ret<=0)
    {
        printk("[TSP]i2c_write_interface error line = %d, ret = %d\n", __LINE__, ret);
        return FTS_FALSE;
    }

    return FTS_TRUE;
}

/*
[function]: 
    send a command to ctpm.
[parameters]:
    btcmd[in]        :command code;
    btPara1[in]    :parameter 1;    
    btPara2[in]    :parameter 2;    
    btPara3[in]    :parameter 3;    
    num[in]        :the valid input parameter numbers, if only command code needed and no parameters followed,then the num is 1;    
[return]:
    FTS_TRUE    :success;
    FTS_FALSE    :io fail;
*/
FTS_BOOL cmd_write(FTS_BYTE btcmd,FTS_BYTE btPara1,FTS_BYTE btPara2,FTS_BYTE btPara3,FTS_BYTE num)
{
    FTS_BYTE write_cmd[4] = {0};

    write_cmd[0] = btcmd;
    write_cmd[1] = btPara1;
    write_cmd[2] = btPara2;
    write_cmd[3] = btPara3;
    return i2c_write_interface(I2C_CTPM_ADDRESS, write_cmd, num);
}

/*
[function]: 
    write data to ctpm , the destination address is 0.
[parameters]:
    pbt_buf[in]    :point to data buffer;
    bt_len[in]        :the data numbers;    
[return]:
    FTS_TRUE    :success;
    FTS_FALSE    :io fail;
*/
FTS_BOOL byte_write(FTS_BYTE* pbt_buf, FTS_DWRD dw_len)
{
    
    return i2c_write_interface(I2C_CTPM_ADDRESS, pbt_buf, dw_len);
}


static int CTPDMA_i2c_write(FTS_BYTE slave,FTS_BYTE* pbt_buf, FTS_DWRD dw_len)
{
    
	int i = 0;
	for(i = 0 ; i < dw_len; i++)
	{
		CTPI2CDMABuf_va[i] = pbt_buf[i];
	}

	if(dw_len <= 8)
	{
		//i2c_client->addr = i2c_client->addr & I2C_MASK_FLAG;
		//MSE_ERR("Sensor non-dma write timing is %x!\r\n", this_client->timing);
		return i2c_master_send(i2c_client, pbt_buf, dw_len);
	}
	else
	{
		i2c_client->addr = i2c_client->addr & I2C_MASK_FLAG | I2C_DMA_FLAG;
		//MSE_ERR("Sensor dma timing is %x!\r\n", this_client->timing);
		return i2c_master_send(i2c_client, CTPI2CDMABuf_pa, dw_len);
	}    
}


static int CTPDMA_i2c_read(FTS_BYTE slave, FTS_BYTE *buf, FTS_DWRD len)
{
	int i = 0, err = 0;

	if(len < 8)
	{
		i2c_client->addr = i2c_client->addr & I2C_MASK_FLAG;
		//MSE_ERR("Sensor non-dma read timing is %x!\r\n", this_client->timing);
		return i2c_master_recv(i2c_client, buf, len);
	}
	else
	{
		i2c_client->addr = i2c_client->addr & I2C_MASK_FLAG | I2C_DMA_FLAG;
		//MSE_ERR("Sensor dma read timing is %x!\r\n", this_client->timing);
		err = i2c_master_recv(i2c_client, CTPI2CDMABuf_pa, len);
		
	    if(err < 0)
	    {
			return err;
		}

		for(i = 0; i < len; i++)
		{
			buf[i] = CTPI2CDMABuf_va[i];
		}
	}
}


/*
[function]: 
    read out data from ctpm,the destination address is 0.
[parameters]:
    pbt_buf[out]    :point to data buffer;
    bt_len[in]        :the data numbers;    
[return]:
    FTS_TRUE    :success;
    FTS_FALSE    :io fail;
*/
FTS_BOOL byte_read(FTS_BYTE* pbt_buf, FTS_BYTE bt_len)
{
    return i2c_read_interface(I2C_CTPM_ADDRESS, pbt_buf, bt_len);
}


/*
[function]: 
    burn the FW to ctpm.
[parameters]:(ref. SPEC)
    pbt_buf[in]    :point to Head+FW ;
    dw_lenth[in]:the length of the FW + 6(the Head length);    
    bt_ecc[in]    :the ECC of the FW
[return]:
    ERR_OK        :no error;
    ERR_MODE    :fail to switch to UPDATE mode;
    ERR_READID    :read id fail;
    ERR_ERASE    :erase chip fail;
    ERR_STATUS    :status error;
    ERR_ECC        :ecc error.
*/


#define    FTS_PACKET_LENGTH        2

static unsigned char CTPM_FW[]=
{
#include "ft_app.i"
};

E_UPGRADE_ERR_TYPE  fts_ctpm_fw_upgrade(FTS_BYTE* pbt_buf, FTS_DWRD dw_lenth)
{
    FTS_BYTE reg_val[2] = {0};
    FTS_DWRD i = 0;

    FTS_DWRD  packet_number;
    FTS_DWRD  j;
    FTS_DWRD  temp;
    FTS_DWRD  lenght;
    FTS_BYTE  packet_buf[FTS_PACKET_LENGTH + 6];
    FTS_BYTE  auc_i2c_write_buf[10];
    FTS_BYTE bt_ecc;
    int      i_ret;
	int ret;
	unsigned char ver;
    /*********Step 1:Reset  CTPM *****/
    /*write 0xaa to register 0xfc*/

	mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
    mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ZERO);  
    msleep(10);  
    mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
    mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ONE);
    printk("[TSP] Step 1: Reset CTPM test\n");
   
    delay_qt_ms(50);   
    /*********Step 2:Enter upgrade mode *****/
    auc_i2c_write_buf[0] = 0x55;
    auc_i2c_write_buf[1] = 0xaa;
    do
    {
        i ++;
        i_ret = ft5x0x_i2c_txdata(auc_i2c_write_buf, 2);
        delay_qt_ms(5);
    }while(i_ret <= 0 && i < 5 );

    /*********Step 3:check READ-ID***********************/        
    cmd_write(0x90,0x00,0x00,0x00,4);
	i=0;
	i_ret=0;
    do
    {
        i ++;
        i_ret = byte_read(reg_val,2);
        delay_qt_ms(10);
    }while(i_ret <= 0 && i < 5 );
    printk("[TSP] Step 2: CTPM ID,ID1 = 0x%x,ID2 = 0x%x\n",reg_val[0],reg_val[1]);
#if 0 /*zhouwl, temp disable this line???*/
    if (reg_val[0] == 0x79 && reg_val[1] == 0x3)
    {
        printk("[TSP] Step 3: CTPM ID,ID1 = 0x%x,ID2 = 0x%x\n",reg_val[0],reg_val[1]);
    }
    else
    {
        return ERR_READID;
        //i_is_new_protocol = 1;
    }
#endif
     /*********Step 4:erase app*******************************/
    ret = cmd_write(0x61,0x00,0x00,0x00,1);
   
    delay_qt_ms(1500);
    printk("[TSP] Step 4: erase.ret=%d\n",ret);

    /*********Step 5:write firmware(FW) to ctpm flash*********/
    bt_ecc = 0;
    printk("[TSP] Step 5: start upgrade. \n");
    dw_lenth = dw_lenth - 8;
    packet_number = (dw_lenth) / FTS_PACKET_LENGTH;
    packet_buf[0] = 0xbf;
    packet_buf[1] = 0x00;
    for (j=0;j<packet_number;j++)
    {
        temp = j * FTS_PACKET_LENGTH;
        packet_buf[2] = (FTS_BYTE)(temp>>8);
        packet_buf[3] = (FTS_BYTE)temp;
        lenght = FTS_PACKET_LENGTH;
        packet_buf[4] = (FTS_BYTE)(lenght>>8);
        packet_buf[5] = (FTS_BYTE)lenght;

        for (i=0;i<FTS_PACKET_LENGTH;i++)
        {
            packet_buf[6+i] = pbt_buf[j*FTS_PACKET_LENGTH + i]; 
            bt_ecc ^= packet_buf[6+i];
        }
        
        ret=CTPDMA_i2c_write(0x70, &packet_buf[0],FTS_PACKET_LENGTH + 6);
              //printk("[TSP] 111 ret 0x%x \n", ret);
        //delay_qt_ms(FTS_PACKET_LENGTH/6 + 1);
        if ((j * FTS_PACKET_LENGTH % 1024) == 0)
        {
              //printk("[TSP] upgrade the 0x%x th byte.\n", ((unsigned int)j) * FTS_PACKET_LENGTH);
        }
    }

    if ((dw_lenth) % FTS_PACKET_LENGTH > 0)
    {
        temp = packet_number * FTS_PACKET_LENGTH;
        packet_buf[2] = (FTS_BYTE)(temp>>8);
        packet_buf[3] = (FTS_BYTE)temp;

        temp = (dw_lenth) % FTS_PACKET_LENGTH;
        packet_buf[4] = (FTS_BYTE)(temp>>8);
        packet_buf[5] = (FTS_BYTE)temp;

        for (i=0;i<temp;i++)
        {
            packet_buf[6+i] = pbt_buf[ packet_number*FTS_PACKET_LENGTH + i]; 
            bt_ecc ^= packet_buf[6+i];
        }
             // printk("[TSP]temp 0x%x \n", temp);
        ret = CTPDMA_i2c_write(0x70, &packet_buf[0],temp+6);    
              //printk("[TSP] 222 ret 0x%x \n", ret);
        delay_qt_ms(20);
    }
 
    //send the last six byte
    for (i = 0; i<6; i++)
    {
        temp = 0x6ffa + i;
        packet_buf[2] = (FTS_BYTE)(temp>>8);
        packet_buf[3] = (FTS_BYTE)temp;
        temp =1;
        packet_buf[4] = (FTS_BYTE)(temp>>8);
        packet_buf[5] = (FTS_BYTE)temp;
        packet_buf[6] = pbt_buf[ dw_lenth + i]; 
        bt_ecc ^= packet_buf[6];

        CTPDMA_i2c_write(0x70,&packet_buf[0],7);  
        delay_qt_ms(20);
    }

    /*********Step 6: read out checksum***********************/
    /*send the opration head*/
    //cmd_write(0xcc,0x00,0x00,0x00,1);
    //byte_read(reg_val,1);
i2c_smbus_read_i2c_block_data(i2c_client, 0xcc, 1, &(reg_val[0]));
    //printk("[TSP] Step 6:  ecc read 0x%x, new firmware 0x%x. \n", reg_val[0], bt_ecc);
    if(reg_val[0] != bt_ecc)
    {
        //return ERR_ECC;
    }

    /*********Step 7: reset the new FW***********************/
    //cmd_write(0x07,0x00,0x00,0x00,1);
	mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
    mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ZERO);  
    msleep(1);  
    mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
    mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ONE);

    return ERR_OK;
}


static int fts_ctpm_fw_upgrade_with_i_file(void)
{
   FTS_BYTE*     pbt_buf = FTS_NULL;
   int i_ret;
   unsigned char version=0;
    FTS_BYTE flag;
    FTS_DWRD i = 0;
    //=========FW upgrade========================*/
   pbt_buf = CTPM_FW;
	
	printk("version=%x ,pbt_buf[sizeof(CTPM_FW)-2]=%d\n",version,pbt_buf[sizeof(CTPM_FW)-2]);
	printk("[TSP]ID_ver=%x, fw_ver=%x\n", ft5x0x_read_ID_ver(), ft5x0x_read_fw_ver());
#if 0 /*zhouwl, temp disable this line*/
if(0xa8 != ft5x0x_read_ID_ver())
{
	if(ft5x0x_read_ID_ver() != pbt_buf[sizeof(CTPM_FW)-1])
	{
        return;
	}
	
    do
    {
        i ++;
        version =ft5x0x_read_fw_ver();
        delay_qt_ms(2);
    }while( i < 5 );
    
	if(version==pbt_buf[sizeof(CTPM_FW)-2])
	{
		return;
	}
}
#endif
   /*call the upgrade function*/
   i_ret =  fts_ctpm_fw_upgrade(pbt_buf,sizeof(CTPM_FW));
   if (i_ret != 0)
   {
	printk("[TSP]upgrade error\n");
       //error handling ...
       //TBD
   }
	msleep(200);  
    ft5x0x_write_reg(0xfc,0x04);
	msleep(4000);
	flag=0;
	i2c_smbus_read_i2c_block_data(i2c_client, 0xFC, 1, &flag);
	//printk("flag=%d\n",flag);
   return i_ret;
}

unsigned char fts_ctpm_get_upg_ver(void)
{
    unsigned int ui_sz;
    ui_sz = sizeof(CTPM_FW);
    if (ui_sz > 2)
    {
        return CTPM_FW[ui_sz - 2];
    }
    else
    {
        //TBD, error handling?
        return 0xff; //default value
    }
}
#endif
#ifdef ESD_CHECK	
static void ESD_read_id_workqueue(struct work_struct *work)
{
	char data;
	if(tpd_halt) 
		return; 
	i2c_smbus_read_i2c_block_data(i2c_client, 0x88, 1, &data);
//	TPD_DEBUG("ESD_read_id_workqueue data: %d\n", data);
	printk("ESD_read_id_workqueue data: %d\n", data);
	if((data > 5)&&(data < 10))
	{
		//add_timer();
	}
	else
	{

	 	mt65xx_eint_mask(CUST_EINT_TOUCH_PANEL_NUM);
		 if(tpd_state)
		 {
			 input_mt_sync(tpd->dev);
	                input_sync(tpd->dev);
			tpd_state = 0;
		 }
		msleep(5);  
            #ifdef TPD_POWER_SOURCE_2800
            		hwPowerDown(TPD_POWER_SOURCE_2800,  "TP");
            #endif			
            #ifdef TPD_POWER_SOURCE_1800
			hwPowerDown(TPD_POWER_SOURCE_1800,  "TP");
            #endif    
	     msleep(5);  
            #ifdef TPD_POWER_SOURCE_2800
			hwPowerOn(TPD_POWER_SOURCE_2800, VOL_2800, "TP");
            #endif
            #ifdef TPD_POWER_SOURCE_1800
			hwPowerOn(TPD_POWER_SOURCE_1800, VOL_1800, "TP");
            #endif    
 
		msleep(100);
		mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
		mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
		mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ZERO);  
		msleep(10);  
		mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
		mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
		mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ONE);
	 	 mt65xx_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM); 
		 
		 msleep(200);
	}
	if(tpd_halt) 
		mt65xx_eint_mask(CUST_EINT_TOUCH_PANEL_NUM); 
	else 
		queue_delayed_work(ctp_read_id_workqueue, &ctp_read_id_work,400); //schedule a work for the first detection					

}
#endif
static  void tpd_down(int x, int y, int p) {
	// input_report_abs(tpd->dev, ABS_PRESSURE, p);
	 input_report_key(tpd->dev, BTN_TOUCH, 1);
	 input_report_abs(tpd->dev, ABS_MT_TOUCH_MAJOR, 20);
	 input_report_abs(tpd->dev, ABS_MT_POSITION_X, x);
	 input_report_abs(tpd->dev, ABS_MT_POSITION_Y, y);
	 /* Lenovo-sw yexm1, optimize the code, 2012-9-19 begin */
	 //printk("\r\n D[%4d %4d %4d] ", x, y, p);
	/* Lenovo-sw yexm1, optimize the code, 2012-9-19 end */
	 /* track id Start 0 */
       input_report_abs(tpd->dev, ABS_MT_TRACKING_ID, p); 
	 input_mt_sync(tpd->dev);
     if (FACTORY_BOOT == get_boot_mode()|| RECOVERY_BOOT == get_boot_mode())
     {   
       tpd_button(x, y, 1);  
     }
	 if(y > TPD_RES_Y) //virtual key debounce to avoid android ANR issue
	 {
         /* Lenovo-sw yexm1 modify, 2012-10-15, delete the delay */
         //msleep(50);
		 printk("D virtual key \n");
	 }
	 TPD_EM_PRINT(x, y, x, y, p-1, 1);
 }
 
static  void tpd_up(int x, int y,int *count) {
	 //if(*count>0) {
		 //input_report_abs(tpd->dev, ABS_PRESSURE, 0);
		 input_report_key(tpd->dev, BTN_TOUCH, 0);
		 //input_report_abs(tpd->dev, ABS_MT_TOUCH_MAJOR, 0);
		 //input_report_abs(tpd->dev, ABS_MT_POSITION_X, x);
		 //input_report_abs(tpd->dev, ABS_MT_POSITION_Y, y);
		 //printk("U[%4d %4d %4d] ", x, y, 0);
		 input_mt_sync(tpd->dev);
		 TPD_EM_PRINT(x, y, x, y, 0, 0);
	//	 (*count)--;
     //printk("\r\n u[%4d %4d] ", x, y);
     if (FACTORY_BOOT == get_boot_mode()|| RECOVERY_BOOT == get_boot_mode())
     {   
        tpd_button(x, y, 0); 
     }   		 

 }

 static int tpd_touchinfo(struct touch_info *cinfo, struct touch_info *pinfo)
 {

	int i = 0;
	
	//char data[30] = {0};
	char data[50] = {0};

    u16 high_byte,low_byte;
	u8 report_rate =0;
	int  ret = 0;

	p_point_num = point_num;
      //printk("\r\n zhaoshaopeng tpd_touchinfo \r\n");
       #if 0
	 /* Lenovo-sw yexm1, optimize the code, 2012-9-19 begin */
	i2c_smbus_read_i2c_block_data(i2c_client, 0x00, 32, &(data[0]));
	//i2c_smbus_read_i2c_block_data(i2c_client, 0x08, 8, &(data[8]));
	//i2c_smbus_read_i2c_block_data(i2c_client, 0x10, 8, &(data[16]));
	//i2c_smbus_read_i2c_block_data(i2c_client, 0x18, 8, &(data[24]));
	//i2c_smbus_read_i2c_block_data(i2c_client, 0xa6, 1, &(data[32]));
	/* Lenovo-sw yexm1, optimize the code, 2012-9-19 end */
        #else
	ret = i2c_smbus_read_i2c_block_data(i2c_client, 0x00, 8, &(data[0]));
	 if(ret < 0)
	 {
	 	printk("I2c error line:%d\n",__LINE__);
		try_to_reset();
	 }
	ret = i2c_smbus_read_i2c_block_data(i2c_client, 0x08, 8, &(data[8]));
	if(ret < 0)
	 {
	 	printk("I2c error line:%d\n",__LINE__);
		try_to_reset();
	 }
	ret = i2c_smbus_read_i2c_block_data(i2c_client, 0x10, 8, &(data[16]));
	if(ret < 0)
	 {
	 	printk("I2c error line:%d\n",__LINE__);
		try_to_reset();
	 }
	//zhaoshaopeng start
	//i2c_smbus_read_i2c_block_data(i2c_client, 0xa6, 1, &(data[24]));
	ret = i2c_smbus_read_i2c_block_data(i2c_client, 0x18, 8, &(data[24]));
	if(ret < 0)
	 {
	 	printk("I2c error line:%d\n",__LINE__);
		try_to_reset();
	 }
	ret = i2c_smbus_read_i2c_block_data(i2c_client, 0x20, 8, &(data[32]));
	if(ret < 0)
	 {
	 	printk("I2c error line:%d\n",__LINE__);
		try_to_reset();
	 }
	ret = i2c_smbus_read_i2c_block_data(i2c_client, 0xa6, 1, &(data[40]));
	if(ret < 0)
	 {
	 	printk("I2c error line:%d\n",__LINE__);
		try_to_reset();
	 }
        #endif
	ret = i2c_smbus_read_i2c_block_data(i2c_client, 0x88, 1, &report_rate);
	if(ret < 0)
	 {
	 	printk("I2c error line:%d\n",__LINE__);
		try_to_reset();
	 }
	 if(report_rate < 8)
	 {
	   report_rate = 0x8;
	   if((i2c_smbus_write_i2c_block_data(i2c_client, 0x88, 1, &report_rate))< 0)
	   {
		   TPD_DMESG("I2C read report rate error, line: %d\n", __LINE__);
		try_to_reset();
	   }
	 }
	
	/* Device Mode[2:0] == 0 :Normal operating Mode*/
	if((data[0] & 0x70) != 0) return false; 

	/*get the number of the touch points*/
	point_num= data[2] & 0x0f;
	
	//TPD_DEBUG("point_num =%d\n",point_num);
	
//	if(point_num == 0) return false;

	   //TPD_DEBUG("Procss raw data...\n");

		
		for(i = 0; i < point_num; i++)
		{
			cinfo->p[i] = data[3+6*i] >> 6; //event flag 
            cinfo->id[i] = data[3+6*i+2]>>4; //touch id
	       /*get the X coordinate, 2 bytes*/
			high_byte = data[3+6*i];
			high_byte <<= 8;
			high_byte &= 0x0f00;
			low_byte = data[3+6*i + 1];
			cinfo->x[i] = high_byte |low_byte;

				//cinfo->x[i] =  cinfo->x[i] * 480 >> 11; //calibra
		
			/*get the Y coordinate, 2 bytes*/
			
			high_byte = data[3+6*i+2];
			
//		    TPD_DEBUG(" cinfo->finger_id= %d,high_byte=%d\n", cinfo->finger_id[i],high_byte);	
			high_byte <<= 8;
			high_byte &= 0x0f00;
			low_byte = data[3+6*i+3];
			cinfo->y[i] = high_byte |low_byte;

			  //cinfo->y[i]=  cinfo->y[i] * 800 >> 11;
		
			cinfo->count++;
			
			     }
		//TPD_DEBUG(" cinfo->x[0] = %d, cinfo->y[0] = %d, cinfo->p[0] = %d\n", cinfo->x[0], cinfo->y[0], cinfo->p[0]);	
		//TPD_DEBUG(" cinfo->x[1] = %d, cinfo->y[1] = %d, cinfo->p[1] = %d\n", cinfo->x[1], cinfo->y[1], cinfo->p[1]);		
		//TPD_DEBUG(" cinfo->x[2]= %d, cinfo->y[2]= %d, cinfo->p[2] = %d\n", cinfo->x[2], cinfo->y[2], cinfo->p[2]);	
// add for m536
#ifdef FTS_CTL_FACE_DETECT
				if(face_detect_enable==1)
				{
				mutex_lock(&ps_operator_lock);
					printk("lanhong tpd_touchinfo face_detect_enable = %x\n",face_detect_enable);
					volatile unsigned char face_detect_enable_reg = 0,ps_data;
					is_face_detect=0;
					face_detect_value=0;
		
					 ret = i2c_smbus_read_i2c_block_data(i2c_client, 0xb0, 1, &face_detect_enable_reg);
					 if(ret < 0)
					 {
					 	printk("I2c error line:%d\n",__LINE__);
						try_to_reset();
					 }
					printk("lanhong i2c_smbus_read_i2c_block_data face_detect_enable_reg = %x\n",face_detect_enable_reg);
					//face_detect_enable = data[FT_FACE_DETECT_POS];
					ret = i2c_smbus_read_i2c_block_data(i2c_client, 0x01,1, &ps_data);
					 if(ret < 0)
					 {
					 	printk("I2c error line:%d\n",__LINE__);
						try_to_reset();
					 }
					 printk("lanhong  xxxxxxxxxxxxxxxxxx ctp ps_data=0x%x\n",ps_data);	
					if (FT_FACE_DETECT_ENABLE == face_detect_enable_reg) {
						is_face_detect = ps_data;
						face_detect_value = data[FT_FACE_DETECT_POS] & 0x1F;
						printk("\r\n face_detect_enable:%x is_face_detect:0x%x face_detect_value:0x%x\r\n", face_detect_enable, is_face_detect, face_detect_value); 
					}
					mutex_unlock(&ps_operator_lock);
				}
#endif	

//end
	 return true;

 };

 static int touch_event_handler(void *unused)
 {
  
    struct touch_info cinfo, pinfo;
	 int i=0,err =0;
//add for m536
#ifdef FTS_CTL_FACE_DETECT
		 hwm_sensor_data sensor_data;
#endif
// end
	 struct sched_param param = { .sched_priority = RTPM_PRIO_TPD };
	 sched_setscheduler(current, SCHED_RR, &param);
 
	 do
	 {
	  mt65xx_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM); 
		 set_current_state(TASK_INTERRUPTIBLE); 
		  wait_event_interruptible(waiter,tpd_flag!=0);
						 
			 tpd_flag = 0;
			 
		 set_current_state(TASK_RUNNING);
		 

		  if (tpd_touchinfo(&cinfo, &pinfo)) 
		  {
		  #ifdef FTS_CTL_FACE_DETECT

			if(!tpd_suspend_status)
		#endif
		{
			TPD_DEBUG_SET_TIME;
			if(point_num >0) 
             	{
			    for(i =0; i<point_num && i<5; i++)
    			           {

			         tpd_down(cinfo.x[i], cinfo.y[i], cinfo.id[i]);
			       
    			           }
			    input_sync(tpd->dev);
				}
				
			else  
            {
			    tpd_up(cinfo.x[0], cinfo.y[0], 0);
                //TPD_DEBUG("release --->\n"); 
                //input_mt_sync(tpd->dev);
                input_sync(tpd->dev);
            }
		}
	#ifdef FTS_CTL_FACE_DETECT
	        mutex_lock(&ps_operator_lock);

			//printk("lanhong FTS_CTL_FACE_DETECT  0000000000000000000000\n\r");
			//printk("lanhong face_detect_enable = %x\n",face_detect_enable);
			printk("lanhong is_face_detect = %x\n",is_face_detect);
			if (FT_FACE_DETECT_ENABLE == face_detect_enable) {
				if (FT_FACE_DETECT_ON == is_face_detect )
				{
					//printk("lanhong FTS_CTL_FACE_DETECT  11111111111111111111\n\r");
					sensor_data.values[0] = 0;
					sensor_data.value_divide = 1;
					sensor_data.status = SENSOR_STATUS_ACCURACY_MEDIUM;
					TPD_DEBUG("eint tp_ps 0x13 raw value %d \n", sensor_data.values[0] );
					//let up layer to know
					if((err = hwmsen_get_interrupt_data(ID_PROXIMITY, &sensor_data)))
					{
					TPD_DEBUG("call hwmsen_get_interrupt_data fail = %d\n", err);
					}
				}
				else if (FT_FACE_DETECT_OFF == is_face_detect)
				{
					//printk("lanhong FTS_CTL_FACE_DETECT  2222222222222222222222\n\r");
					sensor_data.values[0] = 1;

					sensor_data.value_divide = 1;
					sensor_data.status = SENSOR_STATUS_ACCURACY_MEDIUM;
					TPD_DEBUG("eint tp_ps 0x13 raw value %d \n", sensor_data.values[0] );
					//let up layer to know
					if((err = hwmsen_get_interrupt_data(ID_PROXIMITY, &sensor_data)))
					{
						TPD_DEBUG("call hwmsen_get_interrupt_data fail = %d\n", err);
					}
				}
			}
			mutex_unlock(&ps_operator_lock);
#endif
        }

        if(tpd_mode==12)
        {
           //power down for desence debug
           //power off, need confirm with SA
#ifdef TPD_POWER_SOURCE_2800
    		hwPowerDown(TPD_POWER_SOURCE_2800,  "TP");
#endif

#ifdef TPD_POWER_SOURCE_1800
    		hwPowerDown(TPD_POWER_SOURCE_1800,  "TP");
#endif    
	    msleep(20);
          
        }

 }while(!kthread_should_stop());
 
	 return 0;
 }
 
 static int tpd_detect (struct i2c_client *client, struct i2c_board_info *info) 
 {
	 strcpy(info->type, TPD_DEVICE);	
	  return 0;
 }
 
 static void tpd_eint_interrupt_handler(void)
 {
	 //TPD_DEBUG("TPD interrupt has been triggered\n");
	 TPD_DEBUG_PRINT_INT;
	 tpd_flag = 1;
	 wake_up_interruptible(&waiter);
	 
 }
 #ifdef FTS_CTL_FACE_DETECT
static int ft_psensor_enable(struct i2c_client *client, unsigned char en)
{
	int ret = 0;
	unsigned char buf[2];

	printk("lanhong ft_psensor_enable........................................\n\r");
	ret = i2c_smbus_write_i2c_block_data(client, PSENSOR_ENABLE_REG, 1, &en);
/*
	buf[0] = PSENSOR_ENABLE_REG;
	buf[1]= en;

	ret = ft6x06_i2c_Write(client, &buf, 2);
*/
	if(ret<0)
	 dev_err(&client->dev, "%s:operate psensor failed\n",
			 __func__);
	return ret;
}

 static int tp_ps_operate(void* self, uint32_t command, void* buff_in, int size_in,
						  void* buff_out, int size_out, int* actualout)
 {
	 int err = 0;
	 int value;
	 //hwm_sensor_data* sensor_data;
	 //hwm_sensor_data sensor_data;
	 U8 ps_store_data[4];
 
 
	 printk("tp_ps_operate command %d\n", command);
 
	 //APS_FUN(f);
	 switch (command)
	 {
	 case SENSOR_DELAY:
		 if((buff_in == NULL) || (size_in < sizeof(int)))
		 {
			 TPD_DEBUG("Set delay parameter error!\n");
			 err = -EINVAL;
		 }
		 // Do nothing
		 break;
 
	 case SENSOR_ENABLE:
		 if((buff_in == NULL) || (size_in < sizeof(int)))
		 {
			 printk("Enable sensor parameter error!\n");
			 err = -EINVAL;
		 }
		 else
		 {
			 value = *(int *)buff_in;
			 printk("tp_ps enable %d !\n", value);
			 if(value)
			 {
				//if(face_detect_enable==0)
				{
					 face_detect_enable = 1;
					 ft_psensor_enable(i2c_client, 1);
					printk("func:%s,face_detect_enable = %d\n",__func__,__LINE__);
				}
 
			 }
			 else
			 {
				//if(face_detect_enable==1)
				{
				 	face_detect_enable = 0;
				 	ft_psensor_enable(i2c_client, 0);
				 	printk("func:%s,face_detect_enable = %d\n",__func__,__LINE__);
				}
			 }

			 if(value)
			 {
				 wake_lock(&ps_lock);
			 }
			 else
			 {
				 wake_unlock(&ps_lock);
			 }

		 }
		 break;
 
	 case SENSOR_GET_DATA:
		 if((buff_out == NULL) || (size_out< sizeof(hwm_sensor_data)))
		 {
			 TPD_DEBUG("get sensor data parameter error!\n");
			 err = -EINVAL;
		 }
		 break;
	 default:
		 TPD_DEBUG("proxmy sensor operate function no this parameter %d!\n", command);
		 err = -1;
		 break;
	 }
 
	 return err;
 }
#endif
 static void tpd_reset_sr(void)
 	{
 	 static char data = 0x3;
#ifdef ESD_CHECK	
 	cancel_delayed_work_sync(&ctp_read_id_work);
#endif
	 TPD_DMESG("TPD enter sleep\n");
	tpd_halt = 1; //add this line 
	 mt65xx_eint_mask(CUST_EINT_TOUCH_PANEL_NUM);
#ifdef TPD_CLOSE_POWER_IN_SLEEP	
	 hwPowerDown(TPD_POWER_SOURCE,"TP");
#else
i2c_smbus_write_i2c_block_data(i2c_client, 0xA5, 1, &data);  //TP enter sleep mode
#endif
    msleep(20);
	    TPD_DMESG("TPD wake up\n");
#ifdef TPD_CLOSE_POWER_IN_SLEEP	
		hwPowerOn(TPD_POWER_SOURCE,VOL_3300,"TP"); 
#else
		discard_resume_first_eint = KAL_TRUE;
	
		//msleep(100);
		mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
		mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
		mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ONE);
		msleep(1);
		mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
		mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
		mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ZERO);  
		msleep(10);  
		mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
		mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
		mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ONE);
#endif
		msleep(200);//add this line 
	   mt65xx_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);  
#ifdef ESD_CHECK	
			msleep(1);	
		queue_delayed_work(ctp_read_id_workqueue, &ctp_read_id_work,400); //schedule a work for the first detection 				
#endif
		
		   msleep(20);
		if((i2c_smbus_read_i2c_block_data(i2c_client, 0x00, 1, &data))< 0)
		{
			TPD_DMESG("resume I2C transfer error, line: %d\n", __LINE__);
		
		}
		tpd_halt = 0;//add this line 
		tpd_up(0,0,0);
		input_sync(tpd->dev);
 	}
 
 static int __devinit tpd_probe(struct i2c_client *client, const struct i2c_device_id *id)
 {	 
	int retval = TPD_OK;
#ifdef ESD_CHECK	
	int ret;
#endif
#ifdef FTS_CTL_FACE_DETECT
		struct hwmsen_object obj_ps;
#endif

	char data;
	u8 report_rate=0;
	int err=0;
	int reset_count = 0;

reset_proc:   
	i2c_client = client;

       printk("\r\n zhaoshaopeng tpd_prob ft5316 \r\n");
#ifdef TPD_POWER_SOURCE_2800
    	hwPowerOn(TPD_POWER_SOURCE_2800, VOL_2800, "TP");
#endif

#ifdef TPD_POWER_SOURCE_1800
    	hwPowerOn(TPD_POWER_SOURCE_1800, VOL_1800, "TP");
#endif    

#ifdef TPD_CLOSE_POWER_IN_SLEEP	 
	hwPowerDown(TPD_POWER_SOURCE,"TP");
	hwPowerOn(TPD_POWER_SOURCE,VOL_3300,"TP");
	msleep(100);
#else
    mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
    mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ONE);
    msleep(1);
      	
    mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
    mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ZERO);  
    msleep(10);  
		
    mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
    mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ONE);
    msleep(200);
#endif

    mt_set_gpio_mode(GPIO_CTP_EINT_PIN, GPIO_CTP_EINT_PIN_M_EINT);
    mt_set_gpio_dir(GPIO_CTP_EINT_PIN, GPIO_DIR_IN);
    mt_set_gpio_pull_enable(GPIO_CTP_EINT_PIN, GPIO_PULL_ENABLE);
    mt_set_gpio_pull_select(GPIO_CTP_EINT_PIN, GPIO_PULL_UP);

    mt65xx_eint_set_sens(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_SENSITIVE);
    mt65xx_eint_set_hw_debounce(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_DEBOUNCE_CN);
    mt65xx_eint_registration(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_DEBOUNCE_EN, CUST_EINT_TOUCH_PANEL_POLARITY, tpd_eint_interrupt_handler, 1); 
    mt65xx_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);
 
    msleep(100);
#ifdef CONFIG_SUPPORT_FTS_CTP_UPG
	CTPI2CDMABuf_va = (u8 *)dma_alloc_coherent(NULL, 4096, &CTPI2CDMABuf_pa, GFP_KERNEL);
    	if(!CTPI2CDMABuf_va)
	{
    		printk("[TSP] dma_alloc_coherent error\n");
	}
#endif		
	if((i2c_smbus_read_i2c_block_data(i2c_client, 0x00, 1, &data))< 0)
	{
		TPD_DMESG("I2C transfer error, line: %d\n", __LINE__);
                #ifdef TPD_POWER_SOURCE_2800
                    	hwPowerDown(TPD_POWER_SOURCE_2800, "TP");
                #endif
				
                #ifdef TPD_POWER_SOURCE_1800
                    	hwPowerDown(TPD_POWER_SOURCE_1800, "TP");
                #endif    
		  msleep(10);
#ifdef TPD_RESET_ISSUE_WORKAROUND
        if ( reset_count < TPD_MAX_RESET_COUNT )
        {
            reset_count++;
            goto reset_proc;
        }
#endif
                printk("\r\n zhaoshaopeng tpd_prob ft5316  probe fail, return -1\r\n");
		   return -1; 
	}

#ifdef FT_FM_UPDATE
	msleep(50);
	i2c_smbus_read_i2c_block_data(i2c_client, FT5x0x_REG_DIFF_ID, 1, &uc_factory_id);
		
	printk("read vendor id 0XA8 uc_factory_id =%x \r\n",uc_factory_id);
#endif
	//set report rate 80Hz
	report_rate = 0x8; 
	if((i2c_smbus_write_i2c_block_data(i2c_client, 0x88, 1, &report_rate))< 0)
	{
	    if((i2c_smbus_write_i2c_block_data(i2c_client, 0x88, 1, &report_rate))< 0)
	    {
		   TPD_DMESG("I2C read report rate error, line: %d\n", __LINE__);
	    }
		   
	}

	tpd_load_status = 1;

	#ifdef VELOCITY_CUSTOM_FT5206
	if((err = misc_register(&tpd_misc_device)))
	{
		printk("mtk_tpd: tpd_misc_device register failed\n");
		
	}
	#endif
 #ifdef CONFIG_SUPPORT_FTS_CTP_UPG
    	printk("[TSP] Step 0:init \n");
	msleep(100);
	fts_ctpm_fw_upgrade_with_i_file();
    	printk("[TSP] Step 8:init stop\n");
	printk("[wj]the version is 0x%02x.\n", ft5x0x_read_fw_ver());
#endif	

#ifdef ESD_CHECK	
	ctp_read_id_workqueue = create_workqueue("ctp_read_id");
	INIT_DELAYED_WORK(&ctp_read_id_work, ESD_read_id_workqueue);
	ret = queue_delayed_work(ctp_read_id_workqueue, &ctp_read_id_work,400); //schedule a work for the first detection					
    	printk("[TSP] ret =%d\n",ret);
#endif
    	
//zhaoshaopeng add for ft firmware update
#ifdef  FT_FM_UPDATE
	//ft5x0x_read_reg(0xa6,&buf);//can be deleted
	msleep(50);  //must add or "Step 3:check READ-ID" will return error 
       if(fts_fw_auto_update(i2c_client) == 0)
     	{
     	     printk("\r\n zhaoshaopeng tp firmware update ing...! \r\n");
     	}
#endif
//zhaoshaopeng end

	thread = kthread_run(touch_event_handler, 0, TPD_DEVICE);
	 if (IS_ERR(thread))
		 { 
		  retval = PTR_ERR(thread);
		  TPD_DMESG(TPD_DEVICE " failed to create kernel thread: %d\n", retval);
		}
#ifdef FTS_CTL_FACE_DETECT
	 if (ft_psensor_drv_init(client) < 0)
		 dev_err(&client->dev, "%s:[FTS] create fts control psensor driver failed\n",
				 __func__);


	obj_ps.polling = 0;
	printk("lanhong ft_psensor_drv_init 3333333333333333333333\n\r");
	obj_ps.sensor_operate = tp_ps_operate;
	if((retval = hwmsen_attach(ID_PROXIMITY, &obj_ps)))
	{
		TPD_DEBUG("lanhong attach fail = %d\n", retval);
	}
	TPD_DEBUG("lanhong attach retval = %d\n", retval);
	//added by Ekin for ps sensor
	wake_lock_init(&ps_lock, WAKE_LOCK_SUSPEND, "ps wakelock");
	//added end


#endif
	#ifdef FTS_CTL_IIC
		if (ft_rw_iic_drv_init(client) < 0)
			dev_err(&client->dev, "%s:[FTS] create fts control iic driver failed\n",
					__func__);
	#endif
#ifdef FT_FM_UPDATE
    tpd_reset_sr();
#endif
	printk("\r\n zhaoshaopeng ft5206 Touch Panel Device Probe %s\n", (retval < TPD_OK) ? "FAIL" : "PASS");
   return 0;
   
 }

 static int __devexit tpd_remove(struct i2c_client *client)
 
 {
   
	 TPD_DEBUG("TPD removed\n");

     	#ifdef FTS_CTL_IIC
	ft_rw_iic_drv_exit();
	#endif 
#ifdef CONFIG_SUPPORT_FTS_CTP_UPG
	 
	if(CTPI2CDMABuf_va)
	{
		dma_free_coherent(NULL, 4096, CTPI2CDMABuf_va, CTPI2CDMABuf_pa);
		CTPI2CDMABuf_va = NULL;
		CTPI2CDMABuf_pa = 0;
	}
#endif	
#ifdef ESD_CHECK	
	destroy_workqueue(ctp_read_id_workqueue);
#endif	
   return 0;
 }
 
 
 static int tpd_local_init(void)
 {

 
  TPD_DMESG("Focaltech FT5206 I2C Touchscreen Driver (Built %s @ %s)\n", __DATE__, __TIME__);
 
 
   if(i2c_add_driver(&tpd_i2c_driver)!=0)
   	{
  		TPD_DMESG("ft5206 unable to add i2c driver.\n");
      	return -1;
    }
    if(tpd_load_status == 0) 
    {
    	TPD_DMESG("ft5206 add error touch panel driver.\n");
    	i2c_del_driver(&tpd_i2c_driver);
    	return -1;
    }
	
#ifdef TPD_HAVE_BUTTON     
    tpd_button_setting(TPD_KEY_COUNT, tpd_keys_local, tpd_keys_dim_local);// initialize tpd button data
#endif   
  
#if (defined(TPD_WARP_START) && defined(TPD_WARP_END))    
    TPD_DO_WARP = 1;
    memcpy(tpd_wb_start, tpd_wb_start_local, TPD_WARP_CNT*4);
    memcpy(tpd_wb_end, tpd_wb_start_local, TPD_WARP_CNT*4);
#endif 

#if (defined(TPD_HAVE_CALIBRATION) && !defined(TPD_CUSTOM_CALIBRATION))
    memcpy(tpd_calmat, tpd_def_calmat_local, 8*4);
    memcpy(tpd_def_calmat, tpd_def_calmat_local, 8*4);	
#endif  
		TPD_DMESG("end %s, %d\n", __FUNCTION__, __LINE__);  
		tpd_type_cap = 1;
    return 0; 
 }

 static void try_to_reset(void)
 {
 	static char data = 0x3;
	int reval;
	char data_temp[50] = {0};
	
 #ifdef FTS_CTL_FACE_DETECT		
		 tpd_suspend_status = 1;
		 if (face_detect_enable)
		 {
		 	 tp_suspend_flag = 0;
			 printk("haungzs tpd_suspend face_detect_enable\n");
			 return;
		 }
#endif

	 TPD_DMESG("TPD enter sleep\n");
#ifdef FTS_CTL_FACE_DETECT
	tp_suspend_flag = 1;
#endif
	tpd_halt = 1; //add this line 
	 mt_eint_mask(CUST_EINT_TOUCH_PANEL_NUM);

	i2c_smbus_write_i2c_block_data(i2c_client, 0xA5, 1, &data);  //TP enter sleep mode
	TPD_DMESG("TPD enter sleep done\n");
	msleep(20);

	// for resume 

//	char data;
//  	int retval;
 
   TPD_DMESG("TPD wake up\n");
#ifdef FTS_CTL_FACE_DETECT

	tpd_suspend_status = 0;
 	if(face_detect_enable && !tp_suspend_flag)
	{
	
		printk("haungzs tpd_suspend face_detect_enable\n");
		return;
	}
#endif
	 discard_resume_first_eint = KAL_TRUE;
	 //msleep(100);
	 mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
	 mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
	 mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ONE);
	 msleep(1);
	 mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
	 mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
	 mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ZERO);	
	 msleep(10);  
	 mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
	 mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
	 mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ONE);

	 msleep(200);//add this line 
  	 mt_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);  
	  msleep(20);
	if((i2c_smbus_read_i2c_block_data(i2c_client, 0x00, 1, &data))< 0)
	{
		TPD_DMESG("resume I2C transfer error, line: %d\n", __LINE__);
	
	}
	tpd_halt = 0;//add this line 
	tpd_up(0,0,0);
	input_sync(tpd->dev);
	
#ifdef FTS_CTL_FACE_DETECT
	if(tp_suspend_flag)
	{
		tp_suspend_flag =0;
		ft_psensor_enable(i2c_client, 1);
	}
#endif
	
 }
 static void tpd_resume( struct early_suspend *h )
 {
  //int retval = TPD_OK;
  char data;
  int retval;
 
   TPD_DMESG("TPD wake up\n");
 #ifdef FTS_CTL_FACE_DETECT

	tpd_suspend_status = 0;
 	if(face_detect_enable && !tp_suspend_flag)
	{
	
		printk("haungzs tpd_suspend face_detect_enable\n");
		return;
	}
#endif
#ifdef TPD_CLOSE_POWER_IN_SLEEP	
	hwPowerOn(TPD_POWER_SOURCE,VOL_3300,"TP"); 
#else
	discard_resume_first_eint = KAL_TRUE;

	//msleep(100);
    mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
    mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ONE);
    msleep(1);
	mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
    mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ZERO);  
    msleep(10);  
    mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
    mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ONE);
#endif
	msleep(200);//add this line 
   mt65xx_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);  
#ifdef ESD_CHECK	
    	msleep(1);  
	queue_delayed_work(ctp_read_id_workqueue, &ctp_read_id_work,400); //schedule a work for the first detection					
#endif
	
       msleep(20);
	if((i2c_smbus_read_i2c_block_data(i2c_client, 0x00, 1, &data))< 0)
	{
		TPD_DMESG("resume I2C transfer error, line: %d\n", __LINE__);
	
	}
	tpd_halt = 0;//add this line 
	tpd_up(0,0,0);
	input_sync(tpd->dev);
 #ifdef FTS_CTL_FACE_DETECT	
	if(face_detect_enable)
	{
		ft_psensor_enable(i2c_client, 1);
		TPD_DMESG("Enable psensor\n");
	}
	tp_suspend_flag =0;
 #endif
	TPD_DMESG("TPD wake up done\n");
	 //return retval;
 }
 
 static void tpd_suspend( struct early_suspend *h )
 {
	// int retval = TPD_OK;
	 static char data = 0x3;
	int reval;
	char data_temp[50] = {0};
#ifdef ESD_CHECK	
 	cancel_delayed_work_sync(&ctp_read_id_work);
#endif
 #ifdef FTS_CTL_FACE_DETECT		
		 tpd_suspend_status = 1;
		 if (face_detect_enable)
		 {
		 	 tp_suspend_flag = 0;
			 printk("haungzs tpd_suspend face_detect_enable\n");
			 return;
		 }
         tp_suspend_flag = 1;
#endif

    TPD_DMESG("TPD enter sleep\n");
   	tpd_halt = 1; //add this line 
	 mt65xx_eint_mask(CUST_EINT_TOUCH_PANEL_NUM);
#ifdef TPD_CLOSE_POWER_IN_SLEEP	
	hwPowerDown(TPD_POWER_SOURCE,"TP");
#else
i2c_smbus_write_i2c_block_data(i2c_client, 0xA5, 1, &data);  //TP enter sleep mode
#endif
	TPD_DMESG("TPD enter sleep done\n");
	 //return retval;
 } 


 static struct tpd_driver_t tpd_device_driver = {
		 .tpd_device_name = "FT5206",
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
 static int __init tpd_driver_init(void) {
	 printk("\r\r zhaoshaopeng MediaTek FT5206 touch panel driver init\n");
#if defined(TPD_I2C_NUMBER)	
    i2c_register_board_info(TPD_I2C_NUMBER, &ft5206_i2c_tpd, 1);
#else
	i2c_register_board_info(0, &ft5206_i2c_tpd, 1);
#endif	
		 if(tpd_driver_add(&tpd_device_driver) < 0)
			 TPD_DMESG("add FT5206 driver failed\n");
	 return 0;
 }
 
 /* should never be called */
 static void __exit tpd_driver_exit(void) {
	 TPD_DMESG("MediaTek FT5206 touch panel driver exit\n");
	 //input_unregister_device(tpd->dev);
	 tpd_driver_remove(&tpd_device_driver);
 }
 
 module_init(tpd_driver_init);
 module_exit(tpd_driver_exit);

