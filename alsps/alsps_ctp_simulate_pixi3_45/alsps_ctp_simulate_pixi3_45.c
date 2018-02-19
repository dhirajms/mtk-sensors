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

/* drivers/hwmon/mt6516/amit/tmd2771.c - TMD2771 ALS/PS driver
 * 
 * Author: MingHsien Hsieh <minghsien.hsieh@mediatek.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/kobject.h>
#include <linux/earlysuspend.h>
#include <linux/platform_device.h>
#include <asm/atomic.h>

#include <mach/mt_typedefs.h>
#include <mach/mt_gpio.h>
#include <mach/mt_pm_ldo.h>

#define POWER_NONE_MACRO MT65XX_POWER_NONE

#include <linux/hwmsensor.h>
#include <linux/hwmsen_dev.h>
#include <linux/sensors_io.h>
#include <asm/io.h>
#include <cust_eint.h>
#include <cust_alsps.h>
/******************************************************************************
 * configuration
*******************************************************************************/
/********************************************/
struct ctp_simulate_priv {
    struct alsps_hw  *hw;
    struct i2c_client *client;
    struct work_struct  eint_work;

    
    /*misc*/
    u16		    als_modulus;
    atomic_t    i2c_retry;
    atomic_t    als_suspend;
    atomic_t    als_debounce;   /*debounce time after enabling als*/
    atomic_t    als_deb_on;     /*indicates if the debounce is on*/
    atomic_t    als_deb_end;    /*the jiffies representing the end of debounce*/
    atomic_t    ps_mask;        /*mask ps: always return far away*/
    atomic_t    ps_debounce;    /*debounce time after enabling ps*/
    atomic_t    ps_deb_on;      /*indicates if the debounce is on*/
    atomic_t    ps_deb_end;     /*the jiffies representing the end of debounce*/
    atomic_t    ps_suspend;


    /*data*/
    u16         als;
    u16          ps;
    u8          _align;
    u16         als_level_num;
    u16         als_value_num;
    u32         als_level[C_CUST_ALS_LEVEL-1];
    u32         als_value[C_CUST_ALS_LEVEL];

    atomic_t    als_cmd_val;    /*the cmd value can't be read, stored in ram*/
    atomic_t    ps_cmd_val;     /*the cmd value can't be read, stored in ram*/
    atomic_t    ps_thd_val_high;     /*the cmd value can't be read, stored in ram*/
	atomic_t    ps_thd_val_low;     /*the cmd value can't be read, stored in ram*/
    ulong       enable;         /*enable mask*/
    ulong       pending_intr;   /*pending interrupt*/

    /*early suspend*/
#if defined(CONFIG_HAS_EARLYSUSPEND)
    struct early_suspend    early_drv;
#endif     
};

typedef enum {
    CMC_BIT_ALS    = 1,
    CMC_BIT_PS     = 2,
} CMC_BIT;
static struct ctp_simulate_priv *ctp_simulate_obj = NULL;
/*----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
#define CTP_PS_TAG                  "[CTPPS] "
#define CTP_PS_LOG(fmt, args...)    printk( CTP_PS_TAG ": " fmt, ##args)

/********************************************/
 
extern   int FT3407_pls_enable(void);
extern  int FT3407_pls_disable(void);
extern int  get_FT3407_data(void );
extern int tp_vendor_id_ps;
extern int hwmsen_alsps_sensor_add(struct sensor_init_info* obj);
static int pls_enable(void)
{
	
		return FT3407_pls_enable();
}

static int pls_disable(void)
{
	
		return FT3407_pls_disable();
	
}

static int get_data(void)
{
	int alsps_value=-1;
	alsps_value= get_FT3407_data();
	
	if(alsps_value<0)
		return 1 ;//1 is far;
	else
		return alsps_value;
}


int ctp_simulate_ps_operate(void* self, uint32_t command, void* buff_in, int size_in,
		void* buff_out, int size_out, int* actualout)
{
	int err=0;
	int value;
	hwm_sensor_data* sensor_data;
	struct ctp_simulate_priv *obj = (struct ctp_simulate_priv *)self;
	switch (command)
	{
		case SENSOR_DELAY:
			CTP_PS_LOG("SENSOR_DELAY  \n");
			break;

		case SENSOR_ENABLE:
			CTP_PS_LOG("[FT_PS] SENSOR_ENABLE  \n");

			if((buff_in == NULL) || (size_in < sizeof(int)))
			{
				CTP_PS_LOG("Enable sensor parameter error!\n");
				err = -EINVAL;
			}
			else
			{		
				value = *(int *)buff_in;
				if(value)
				{
					if(err != pls_enable())
					{
						CTP_PS_LOG("enable ps fail: %d\n", err); 
						return -1;
					}
					set_bit(CMC_BIT_PS, &obj->enable);

				}
				else
				{
					if(err != pls_disable())
					{
						printk("disable ps fail: %d\n", err); 
						return -1;
					}
					clear_bit(CMC_BIT_PS, &obj->enable);

				}
			}
			break;

		case SENSOR_GET_DATA:
			//printk("[FT_PS] SENSOR_GET_DATA  \n");
			if((buff_out == NULL) || (size_out< sizeof(hwm_sensor_data)))
			{
				CTP_PS_LOG("get sensor data parameter error!\n");
				err = -EINVAL;
			}
			else
			{
				sensor_data = (hwm_sensor_data *)buff_out;
				CTP_PS_LOG("SENSOR_GET_DATA");
                                //mdelay(160);
				//printk("[FT_PS] ps_operate ps data=%d!\n",get_data());
				sensor_data->values[0] = get_data();
				sensor_data->value_divide = 1;
				sensor_data->status = SENSOR_STATUS_ACCURACY_MEDIUM;			
			}
			
			break;
		default:
			
			break;
	}
	
	return 0;
}



/*----------------------------------------------------------------------------*/
static int ctp_simulate_for_auto_local_init(void)
{	
	struct ctp_simulate_priv *obj;
	struct hwmsen_object obj_ps, obj_als;
	int err = 0;

	if(!(obj = kzalloc(sizeof(*obj), GFP_KERNEL)))
	{
		err = -ENOMEM;
		goto exit;
	}
	memset(obj, 0, sizeof(*obj));
	ctp_simulate_obj = obj;

	obj_ps.self = ctp_simulate_obj;
	
	obj_ps.polling = 0; 
	obj_ps.sensor_operate = ctp_simulate_ps_operate;
	if(err = hwmsen_attach(ID_PROXIMITY, &obj_ps))
	{
		CTP_PS_LOG("attach fail = %d\n", err);
		goto exit;
	}
	//CTP_PS_LOG("ctp_simulate_for_auto_local_init ok!");    
	printk("ctp_simulate_for_auto_local_init ok!");    

	return 0;

exit :
	return -1;
}
/*----------------------------------------------------------------------------*/

static int ctp_simulate_remove(struct platform_device *pdev)
{
	CTP_PS_LOG("ctp_simulate_remove");    
	return 0;
}

static struct sensor_init_info ctp_simulate_init_info = {
		.name = "ctp_simulate_alsps",
		.init = ctp_simulate_for_auto_local_init,
		.uninit = ctp_simulate_remove,
};

/*----------------------------------------------------------------------------*/
static int __init ctp_simulate_alsps_init(void)
{
	CTP_PS_LOG("ctp_simulate_alsps_init");
	hwmsen_alsps_sensor_add(&ctp_simulate_init_info);
	return 0;
}
/*----------------------------------------------------------------------------*/

static void __exit ctp_simulate_alsps_exit(void)
{
	CTP_PS_LOG("ctp_simulate_alsps_exit");
}
/*----------------------------------------------------------------------------*/
module_init(ctp_simulate_alsps_init);
module_exit(ctp_simulate_alsps_exit);
/*----------------------------------------------------------------------------*/
MODULE_AUTHOR("xiaopu.zhu");
MODULE_DESCRIPTION("ctp simulate aslps driver");
MODULE_LICENSE("GPL");
