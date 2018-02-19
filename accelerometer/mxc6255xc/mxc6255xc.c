/* MXC622X motion sensor driver
 *
 *
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

#include <cust_acc.h>
#include <linux/hwmsensor.h>
#include <linux/hwmsen_dev.h>
#include <linux/sensors_io.h>
#include "mxc6255xc.h"
#include <linux/hwmsen_helper.h>

#ifdef MT6516
#include <mach/mt6516_devs.h>
#include <mach/mt6516_typedefs.h>
#include <mach/mt6516_gpio.h>
#include <mach/mt6516_pll.h>
#endif

#ifdef MT6573
#include <mach/mt6573_devs.h>
#include <mach/mt6573_typedefs.h>
#include <mach/mt6573_gpio.h>
#include <mach/mt6573_pll.h>
#endif

#ifdef MT6575
#include <mach/mt6575_devs.h>
#include <mach/mt6575_typedefs.h>
#include <mach/mt6575_gpio.h>
#include <mach/mt6575_pm_ldo.h>
#endif

#ifdef MT6572
#include <mach/mt_typedefs.h>
#include <mach/mt_gpio.h>
#include <mach/mt_pm_ldo.h>
#endif

#include <linux/hwmsen_dev.h>

/*-------------------------MT6516&MT6573 define-------------------------------*/
#ifdef MT6516
#define POWER_NONE_MACRO MT6516_POWER_NONE
#endif

#ifdef MT6573
#define POWER_NONE_MACRO MT65XX_POWER_NONE
#endif

#ifdef MT6575
#define POWER_NONE_MACRO MT65XX_POWER_NONE
#endif

#ifdef MT6572
#define POWER_NONE_MACRO MT65XX_POWER_NONE
#endif

#define SW_SIMULATE_I2C	0			//if use GPIO simulate I2C, define 1, 
									// if use hardware I2C, define 0

#define MXC622X_ATTR_CREATE  //fangjie add
extern int gsensor_init_status; //fangjie add

/*----------------------------------------------------------------------------*/
#define I2C_DRIVERID_MXC622X 345
/*----------------------------------------------------------------------------*/
#define DEBUG 1
/*----------------------------------------------------------------------------*/
//#define CONFIG_MXC622X_LOWPASS   /*apply low pass filter on output*/       //lyon 
/*----------------------------------------------------------------------------*/
#define MXC622X_AXIS_X          0
#define MXC622X_AXIS_Y          1
#define MXC622X_AXIS_Z          2
#define MXC622X_AXES_NUM        3
#define MXC622X_DATA_LEN        2
#define MXC622X_DEV_NAME        "MXC6255X"
#define FTM_CUST_ACC "/data/mxc6255x"
/*----------------------------------------------------------------------------*/
static struct device *compatible_mxc6255x;   //fangjie add 

static const struct i2c_device_id mxc622x_i2c_id[] = {{MXC622X_DEV_NAME,0},{}};
/*the adapter id will be available in customization*/ //commented by xen, 6572 no used
//static unsigned short mxc622x_force[] = {0x00, MXC622X_I2C_SLAVE_ADDR, I2C_CLIENT_END, I2C_CLIENT_END};
//static const unsigned short *const mxc622x_forces[] = { mxc622x_force, NULL };
//static struct i2c_client_address_data mxc622x_addr_data = { .forces = mxc622x_forces,};

static struct i2c_board_info __initdata i2c_mxc622x={ I2C_BOARD_INFO(MXC622X_DEV_NAME, (MXC622X_I2C_SLAVE_ADDR>>1))};    //added by xen for test 20131108

/*----------------------------------------------------------------------------*/
static int mxc622x_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id); 
static int mxc622x_i2c_remove(struct i2c_client *client);
static int mxc622x_i2c_detect(struct i2c_client *client, int kind, struct i2c_board_info *info);
static int mxc622x_suspend(struct i2c_client *client, pm_message_t msg) ;
static int mxc622x_resume(struct i2c_client *client);

/*----------------------------------------------------------------------------*/
typedef enum {
    ADX_TRC_FILTER  = 0x01,
    ADX_TRC_RAWDATA = 0x02,
    ADX_TRC_IOCTL   = 0x04,
    ADX_TRC_CALI	= 0X08,
    ADX_TRC_INFO	= 0X10,
} ADX_TRC;
/*----------------------------------------------------------------------------*/
struct scale_factor{
    u8  whole;
    u8  fraction;
};
/*----------------------------------------------------------------------------*/
struct data_resolution {
    struct scale_factor scalefactor;
    int                 sensitivity;
};
/*----------------------------------------------------------------------------*/
#define C_MAX_FIR_LENGTH (32)
/*----------------------------------------------------------------------------*/
struct data_filter {
    s16 raw[C_MAX_FIR_LENGTH][MXC622X_AXES_NUM];
    int sum[MXC622X_AXES_NUM];
    int num;
    int idx;
};
/*----------------------------------------------------------------------------*/
struct mxc622x_i2c_data {
    struct i2c_client *client;
    struct acc_hw *hw;
    struct hwmsen_convert   cvt;
    
    /*misc*/
    struct data_resolution *reso;
    atomic_t                trace;
    atomic_t                suspend;
    atomic_t                selftest;
	atomic_t				filter;
	atomic_t				sensor_power; //test
    s16                     cali_sw[MXC622X_AXES_NUM+1];

    /*data*/
    s8                      offset[MXC622X_AXES_NUM+1];  /*+1: for 4-byte alignment*/
    signed  char                     data[MXC622X_AXES_NUM+1];

#if defined(CONFIG_MXC622X_LOWPASS)
    atomic_t                firlen;
    atomic_t                fir_en;
    struct data_filter      fir;
#endif 
    /*early suspend*/
#if defined(CONFIG_HAS_EARLYSUSPEND)
    struct early_suspend    early_drv;
#endif     
};
/*----------------------------------------------------------------------------*/
static struct i2c_driver mxc622x_i2c_driver = {
    .driver = {
    //    .owner          = THIS_MODULE,
        .name           = MXC622X_DEV_NAME,
    },
	.probe      		= mxc622x_i2c_probe,
	.remove    			= mxc622x_i2c_remove,
	.detect				= mxc622x_i2c_detect,
#if !defined(CONFIG_HAS_EARLYSUSPEND)    
    .suspend            = mxc622x_suspend,
    .resume             = mxc622x_resume,
#endif
	.id_table = mxc622x_i2c_id,
	//.address_data = &mxc622x_addr_data,    //modified by xen 20131108
};

/*----------------------------------------------------------------------------*/
static struct i2c_client *mxc622x_i2c_client = NULL;
// static struct platform_driver mxc622x_gsensor_driver; //fangjie delete
static struct mxc622x_i2c_data *obj_i2c_data = NULL;
//static bool sensor_power = false;

static GSENSOR_VECTOR3D gsensor_gain;
static char selftestRes[8]= {0}; 


/*----------------------------------------------------------------------------*/
#define GSE_TAG                  "[Gsensor] "
#define GSE_FUN(f)               printk(KERN_INFO GSE_TAG"%s\n", __FUNCTION__)
#define GSE_ERR(fmt, args...)    printk(KERN_ERR GSE_TAG"%s %d : "fmt, __FUNCTION__, __LINE__, ##args)
#define GSE_LOG(fmt, args...)    printk(KERN_INFO GSE_TAG fmt, ##args)
/*----------------------------------------------------------------------------*/
static struct data_resolution mxc622x_data_resolution[] = {
 /*8 combination by {FULL_RES,RANGE}*/
    {{ 3, 9}, 256},   /*+/-2g  in 10-bit resolution:  3.9 mg/LSB*/
    {{ 7, 8}, 128},   /*+/-4g  in 10-bit resolution:  7.8 mg/LSB*/
    {{15, 6},  64},   /*+/-8g  in 10-bit resolution: 15.6 mg/LSB*/
    {{31, 2},  32},   /*+/-16g in 10-bit resolution: 31.2 mg/LSB*/
    {{ 3, 9}, 256},   /*+/-2g  in 10-bit resolution:  3.9 mg/LSB (full-resolution)*/
    {{ 3, 9}, 256},   /*+/-4g  in 11-bit resolution:  3.9 mg/LSB (full-resolution)*/
    {{ 3, 9}, 256},   /*+/-8g  in 12-bit resolution:  3.9 mg/LSB (full-resolution)*/
    {{ 3, 9}, 256},   /*+/-16g in 13-bit resolution:  3.9 mg/LSB (full-resolution)*/            
};
/*----------------------------------------------------------------------------*/
static struct data_resolution mxc622x_offset_resolution = {{15, 6}, 64};
static void MXC622X_power(struct acc_hw *hw, unsigned int on) 
{
	static unsigned int power_on = 0;

	if(hw->power_id != POWER_NONE_MACRO)		// have externel LDO
	{        
		GSE_LOG("power %s\n", on ? "on" : "off");
		if(power_on == on)	// power status not change
		{
			GSE_LOG("ignore power control: %d\n", on);
		}
		else if(on)	// power on
		{
			if(!hwPowerOn(hw->power_id, hw->power_vol, "MXC622X"))
			{
				GSE_ERR("power on fails!!\n");
			}
		}
		else	// power off
		{
			if (!hwPowerDown(hw->power_id, "MXC622X"))
			{
				GSE_ERR("power off fail!!\n");
			}			  
		}
	}
	power_on = on;    
}

/*----------------------------------------------------------------------------*/
static int MXC622X_SetDataResolution(struct mxc622x_i2c_data *obj)
{
	int err;
	u8  dat, reso;
#if 0 
	if((err = hwmsen_read_byte(obj->client, MXC622X_REG_DATA_FORMAT, &dat)))
	{
		GSE_ERR("write data format fail!!\n");
		return err;
	}

	/*the data_reso is combined by 3 bits: {FULL_RES, DATA_RANGE}*/
	reso  = (dat & MXC622X_FULL_RES) ? (0x04) : (0x00);
	reso |= (dat & MXC622X_RANGE_16G); 

	if(reso < sizeof(mxc622x_data_resolution)/sizeof(mxc622x_data_resolution[0]))
	{        
		obj->reso = &mxc622x_data_resolution[reso];
		return 0;
	}
	else
	{
		return -EINVAL;
	}
#endif 
	obj->reso = &mxc622x_data_resolution[2];
	return 0;

}
/*----------------------------------------------------------------------------*/
static int MXC622X_ReadData(struct i2c_client *client, signed char data[MXC622X_AXES_NUM])
{
	char addr = MXC622X_REG_DATAX0;
	signed char buf[MXC622X_DATA_LEN] = {0};
	int err = 0;
	struct mxc622x_i2c_data *priv = i2c_get_clientdata(client); 

	if(NULL == client)
	{
		err = -EINVAL;
	}
	else if(err = hwmsen_read_block(client, addr, buf, MXC622X_DATA_LEN))
	{
		GSE_ERR("error: %d\n", err);
	}
	else
	{
		data[MXC622X_AXIS_X] = -(signed char)(buf[1]);
		data[MXC622X_AXIS_Y] = (signed char)(buf[0]);
		data[MXC622X_AXIS_Z] = 32;
		//printk("[+++++++ ACC MXC622X +++++++++++] %s: data[0] = %d, data[1] = %d\n", __FUNCTION__, data[0], data[1]);

	}
	
	return err;
}
/*----------------------------------------------------------------------------*/
static int MXC622X_ReadOffset(struct i2c_client *client, s8 ofs[MXC622X_AXES_NUM])
{    
	int err =  MXC622X_SUCCESS;
/*
	if((err = hwmsen_read_block(client, MXC622X_REG_OFSX, ofs, MXC622X_AXES_NUM)))
	{
		GSE_ERR("error: %d\n", err);
	}
*/	
	return err;    
}
/*----------------------------------------------------------------------------*/
static int MXC622X_ResetCalibration(struct i2c_client *client)
{
	struct mxc622x_i2c_data *obj = i2c_get_clientdata(client);
	s8 ofs[MXC622X_AXES_NUM] = {0x00, 0x00, 0x00};
	int err = MXC622X_SUCCESS;
/*
	if((err = hwmsen_write_block(client, MXC622X_REG_OFSX, ofs, MXC622X_AXES_NUM)))
	{
		GSE_ERR("error: %d\n", err);
	}

	memset(obj->cali_sw, 0x00, sizeof(obj->cali_sw));
*/
	return err;    
}
/*----------------------------------------------------------------------------*/
static int MXC622X_ReadCalibration(struct i2c_client *client, int dat[MXC622X_AXES_NUM])
{
    struct mxc622x_i2c_data *obj = i2c_get_clientdata(client);
    int err =  MXC622X_SUCCESS;
    int mul;
/*
    if ((err = MXC622X_ReadOffset(client, obj->offset))) {
        GSE_ERR("read offset fail, %d\n", err);
        return err;
    }    
    
    mul = obj->reso->sensitivity/mxc622x_offset_resolution.sensitivity;

    dat[obj->cvt.map[MXC622X_AXIS_X]] = obj->cvt.sign[MXC622X_AXIS_X]*(obj->offset[MXC622X_AXIS_X]*mul + obj->cali_sw[MXC622X_AXIS_X]);
    dat[obj->cvt.map[MXC622X_AXIS_Y]] = obj->cvt.sign[MXC622X_AXIS_Y]*(obj->offset[MXC622X_AXIS_Y]*mul + obj->cali_sw[MXC622X_AXIS_Y]);
    dat[obj->cvt.map[MXC622X_AXIS_Z]] = obj->cvt.sign[MXC622X_AXIS_Z]*(obj->offset[MXC622X_AXIS_Z]*mul + obj->cali_sw[MXC622X_AXIS_Z]);                        
 */                                      
    return err;
}
/*----------------------------------------------------------------------------*/
static int MXC622X_ReadCalibrationEx(struct i2c_client *client, int act[MXC622X_AXES_NUM], int raw[MXC622X_AXES_NUM])
{  
	/*raw: the raw calibration data; act: the actual calibration data*/
	struct mxc622x_i2c_data *obj = i2c_get_clientdata(client);
	int err;
	int mul;

	if((err = MXC622X_ReadOffset(client, obj->offset)))
	{
		GSE_ERR("read offset fail, %d\n", err);
		return err;
	}    
/*
	mul = obj->reso->sensitivity/mxc622x_offset_resolution.sensitivity;
	raw[MXC622X_AXIS_X] = obj->offset[MXC622X_AXIS_X]*mul + obj->cali_sw[MXC622X_AXIS_X];
	raw[MXC622X_AXIS_Y] = obj->offset[MXC622X_AXIS_Y]*mul + obj->cali_sw[MXC622X_AXIS_Y];
	raw[MXC622X_AXIS_Z] = obj->offset[MXC622X_AXIS_Z]*mul + obj->cali_sw[MXC622X_AXIS_Z];

	act[obj->cvt.map[MXC622X_AXIS_X]] = obj->cvt.sign[MXC622X_AXIS_X]*raw[MXC622X_AXIS_X];
	act[obj->cvt.map[MXC622X_AXIS_Y]] = obj->cvt.sign[MXC622X_AXIS_Y]*raw[MXC622X_AXIS_Y];
	act[obj->cvt.map[MXC622X_AXIS_Z]] = obj->cvt.sign[MXC622X_AXIS_Z]*raw[MXC622X_AXIS_Z];                        
*/                       
	return 0;
}
/*----------------------------------------------------------------------------*/
static int MXC622X_WriteCalibration(struct i2c_client *client, int dat[MXC622X_AXES_NUM])
{
	struct mxc622x_i2c_data *obj = i2c_get_clientdata(client);
	int err = MXC622X_SUCCESS;
#if 0	
        int cali[MXC622X_AXES_NUM], raw[MXC622X_AXES_NUM];
	int lsb = mxc622x_offset_resolution.sensitivity;
	int divisor = obj->reso->sensitivity/lsb;

	if((err = MXC622X_ReadCalibrationEx(client, cali, raw)))	/*offset will be updated in obj->offset*/
	{ 
		GSE_ERR("read offset fail, %d\n", err);
		return err;
	}


	/*calculate the real offset expected by caller*/
	cali[MXC622X_AXIS_X] += dat[MXC622X_AXIS_X];
	cali[MXC622X_AXIS_Y] += dat[MXC622X_AXIS_Y];
	cali[MXC622X_AXIS_Z] += dat[MXC622X_AXIS_Z];

	GSE_LOG("UPDATE: (%+3d %+3d %+3d)\n", 
		dat[MXC622X_AXIS_X], dat[MXC622X_AXIS_Y], dat[MXC622X_AXIS_Z]);

	obj->offset[MXC622X_AXIS_X] = (s8)(obj->cvt.sign[MXC622X_AXIS_X]*(cali[obj->cvt.map[MXC622X_AXIS_X]])/(divisor));
	obj->offset[MXC622X_AXIS_Y] = (s8)(obj->cvt.sign[MXC622X_AXIS_Y]*(cali[obj->cvt.map[MXC622X_AXIS_Y]])/(divisor));
	obj->offset[MXC622X_AXIS_Z] = (s8)(obj->cvt.sign[MXC622X_AXIS_Z]*(cali[obj->cvt.map[MXC622X_AXIS_Z]])/(divisor));

	/*convert software calibration using standard calibration*/
	obj->cali_sw[MXC622X_AXIS_X] = obj->cvt.sign[MXC622X_AXIS_X]*(cali[obj->cvt.map[MXC622X_AXIS_X]])%(divisor);
	obj->cali_sw[MXC622X_AXIS_Y] = obj->cvt.sign[MXC622X_AXIS_Y]*(cali[obj->cvt.map[MXC622X_AXIS_Y]])%(divisor);
	obj->cali_sw[MXC622X_AXIS_Z] = obj->cvt.sign[MXC622X_AXIS_Z]*(cali[obj->cvt.map[MXC622X_AXIS_Z]])%(divisor);

	GSE_LOG("NEWOFF: (%+3d %+3d %+3d): (%+3d %+3d %+3d) / (%+3d %+3d %+3d)\n", 
		obj->offset[MXC622X_AXIS_X]*divisor + obj->cali_sw[MXC622X_AXIS_X], 
		obj->offset[MXC622X_AXIS_Y]*divisor + obj->cali_sw[MXC622X_AXIS_Y], 
		obj->offset[MXC622X_AXIS_Z]*divisor + obj->cali_sw[MXC622X_AXIS_Z], 
		obj->offset[MXC622X_AXIS_X], obj->offset[MXC622X_AXIS_Y], obj->offset[MXC622X_AXIS_Z],
		obj->cali_sw[MXC622X_AXIS_X], obj->cali_sw[MXC622X_AXIS_Y], obj->cali_sw[MXC622X_AXIS_Z]);

	if((err = hwmsen_write_block(obj->client, MXC622X_REG_OFSX, obj->offset, MXC622X_AXES_NUM)))
	{
		GSE_ERR("write offset fail: %d\n", err);
		return err;
	}
#endif 
	return err;
}
/*----------------------------------------------------------------------------*/
static int MXC622X_CheckDeviceID(struct i2c_client *client)
{
	u8 databuf[10];    
	int res = 0;

	memset(databuf, 0, sizeof(u8)*10);    
	databuf[0] = MXC622X_REG_DEVID;    
        i2c_master_send(client, databuf, 0x01);
	res = i2c_master_send(client, databuf, 0x01);
	if(res <= 0)
	{
		goto exit_MXC622X_CheckDeviceID;
	}
	
	udelay(500);

	databuf[0] = 0x0;        
	res = i2c_master_recv(client, databuf, 0x01);
	if(res <= 0)
	{
		goto exit_MXC622X_CheckDeviceID;
	}
	
      printk("tangzibo_CheckDeviceID[130] %x\n ", databuf[0]);
         databuf[0]= (databuf[0]&0x3f);
printk("tangzibo_CheckDeviceID[131] %x\n ", databuf[0]);
	//if(databuf[0]!=MXC622X_FIXED_DEVID)
	{
	//	return MXC622X_ERR_IDENTIFICATION;
	}

	exit_MXC622X_CheckDeviceID:
	if (res <= 0)
	{
		return MXC622X_ERR_I2C;
	}
	//lyon
	return MXC622X_SUCCESS;
}
/*----------------------------------------------------------------------------*/
static int MXC622X_SetPowerMode(struct i2c_client *client, bool enable)
{
	char databuf[2]={0};    
	int res = 0;
	char addr = MXC622X_REG_CTRL;

	struct mxc622x_i2c_data *obj = i2c_get_clientdata(client);

	
	if(enable == ((bool)atomic_read(&obj->sensor_power))) 
	{
		GSE_LOG("Sensor power status is newest!\n");
		return MXC622X_SUCCESS;
	}

	databuf[1] &= ~MXC622X_MEASURE_MODE;
	
	if(enable == TRUE)
	{
		databuf[1] |= MXC622X_MEASURE_MODE;
	}
	else
	{
		databuf[1] |= MXC622X_CTRL_PWRDN;
	}
	
	databuf[0] = addr;


	res = i2c_master_send(client, databuf, 0x2);

	if(res <= 0)
	{
		GSE_LOG("set power mode failed!\n");
		return MXC622X_ERR_I2C;
	}
	else if(atomic_read(&obj->trace) & ADX_TRC_INFO)
	{
		GSE_LOG("set power mode ok %d!\n", databuf[1]);
	}

	//sensor_power = enable;
	atomic_set(&obj->sensor_power, enable);
        mdelay(20);
	
	return MXC622X_SUCCESS;    
}
/*----------------------------------------------------------------------------*/
static int MXC622X_SetDataFormat(struct i2c_client *client, u8 dataformat)
{
	struct mxc622x_i2c_data *obj = i2c_get_clientdata(client);
	u8 databuf[10];    
	int res = 0;
/*
	memset(databuf, 0, sizeof(u8)*10);    
	databuf[0] = MXC622X_REG_DATA_FORMAT;    
	databuf[1] = dataformat;

	res = i2c_master_send(client, databuf, 0x2);

	if(res <= 0)
	{
		return MXC622X_ERR_I2C;
	}
	
*/
	return MXC622X_SetDataResolution(obj);    
}
/*----------------------------------------------------------------------------*/
static int MXC622X_SetBWRate(struct i2c_client *client, u8 bwrate)
{
	u8 databuf[10];    
	int res = 0;
/*
	memset(databuf, 0, sizeof(u8)*10);    
	databuf[0] = MXC622X_REG_BW_RATE;    
	databuf[1] = bwrate;

	res = i2c_master_send(client, databuf, 0x2);

	if(res <= 0)
	{
		return MXC622X_ERR_I2C;
	}
*/
//lyon
	return MXC622X_SUCCESS;    
}
/*----------------------------------------------------------------------------*/
static int MXC622X_SetIntEnable(struct i2c_client *client, u8 intenable)
{
	u8 databuf[10];    
	int res = 0;
/*
	memset(databuf, 0, sizeof(u8)*10);    
	databuf[0] = MXC622X_REG_INT_ENABLE;    
	databuf[1] = intenable;

	res = i2c_master_send(client, databuf, 0x2);

	if(res <= 0)
	{
		return MXC622X_ERR_I2C;
	}
*/	
	return MXC622X_SUCCESS;    
}
/*----------------------------------------------------------------------------*/
static int mxc622x_gpio_config(void)
{
   //because we donot use EINT to support low power
   // config to GPIO input mode + PD 
    
    //set to GPIO_GSE_1_EINT_PIN
   // mt_set_gpio_mode(GPIO_GSE_1_EINT_PIN, GPIO_GSE_1_EINT_PIN_M_GPIO);
   // mt_set_gpio_dir(GPIO_GSE_1_EINT_PIN, GPIO_DIR_IN);
	//mt_set_gpio_pull_enable(GPIO_GSE_1_EINT_PIN, GPIO_PULL_ENABLE);
	//mt_set_gpio_pull_select(GPIO_GSE_1_EINT_PIN, GPIO_PULL_DOWN);
    //set to GPIO_GSE_2_EINT_PIN
	//mt_set_gpio_mode(GPIO_GSE_2_EINT_PIN, GPIO_GSE_2_EINT_PIN_M_GPIO);
   // mt_set_gpio_dir(GPIO_GSE_2_EINT_PIN, GPIO_DIR_IN);
	//mt_set_gpio_pull_enable(GPIO_GSE_2_EINT_PIN, GPIO_PULL_ENABLE);
	//mt_set_gpio_pull_select(GPIO_GSE_2_EINT_PIN, GPIO_PULL_DOWN);
	return 0;
}
static int mxc622x_init_client(struct i2c_client *client, int reset_cali)
{
	struct mxc622x_i2c_data *obj = i2c_get_clientdata(client);
	int res = 0;

        res = MXC622X_CheckDeviceID(client); 
	if(res != MXC622X_SUCCESS)
	{
		return res;
	}	
	printk("MXC622X_CheckDeviceID ok \n");
	
	res = MXC622X_SetBWRate(client, MXC622X_BW_100HZ);
	if(res != MXC622X_SUCCESS ) 
	{
		return res;
	}
	printk("MXC622X_SetBWRate OK!\n");
	
	res = MXC622X_SetDataFormat(client, MXC622X_RANGE_2G);
	if(res != MXC622X_SUCCESS) 
	{
		return res;
	}
	printk("MXC622X_SetDataFormat OK!\n");

	gsensor_gain.x = gsensor_gain.y = gsensor_gain.z = obj->reso->sensitivity;


	res = MXC622X_SetIntEnable(client, 0x00);        
	if(res != MXC622X_SUCCESS)
	{
		return res;
	}
	printk("MXC622X disable interrupt function!\n");

	res = MXC622X_SetPowerMode(client, false);
	if(res != MXC622X_SUCCESS)
	{
             return res;
	}
        printk("MXC622X_SetPowerMode OK!\n");


	if(0 != reset_cali)
	{ 
		/*reset calibration only in power on*/
		res = MXC622X_ResetCalibration(client);
		if(res != MXC622X_SUCCESS)
		{
		    return res;
		}
	}
	printk("MXC622X_init_client OK!\n");
#ifdef CONFIG_MXC622X_LOWPASS
	memset(&obj->fir, 0x00, sizeof(obj->fir));  
#endif

	mdelay(20);


	return MXC622X_SUCCESS;
}
/*----------------------------------------------------------------------------*/
static int MXC622X_ReadChipInfo(struct i2c_client *client, char *buf, int bufsize)
{
	u8 databuf[10];    

	memset(databuf, 0, sizeof(u8)*10);

	if((NULL == buf)||(bufsize<=30))
	{
		return -1;
	}
	
	if(NULL == client)
	{
		*buf = 0;
		return -2;
	}

	sprintf(buf, "MXC622X Chip");
	return 0;
}
/*----------------------------------------------------------------------------*/
static int MXC622X_ReadSensorData(struct i2c_client *client, char *buf, int bufsize)
{
	struct mxc622x_i2c_data *obj =  obj_i2c_data; //(struct mxc622x_i2c_data*)i2c_get_clientdata(client);
	client = obj->client;
	char databuf[MXC622X_AXES_NUM] = {0};
	int acc[MXC622X_AXES_NUM] = {0};
	int res = 0;
	
	memset(acc, 0, sizeof(u8)*MXC622X_AXES_NUM);

	if(NULL == buf)
	{
		return -1;
	}
	if(NULL == client)
	{
		*buf = 0;
		return -2;
	}
	//if(sensor_power == FALSE)
	#if 0
	if(((bool)atomic_read(&obj->sensor_power)) == FALSE)
	{
		printk("Gsensor MXC622X_ReadSensorData!\n");
		res = MXC622X_SetPowerMode(client, true);
		if(res)
		{
			GSE_ERR("Power on mxc622x error %d!\n", res);
		}
		msleep(20);
	}
	#endif

	
	if(MXC622X_ReadData(client, obj->data))
	{        
		GSE_ERR("I2C error: ret value=%d", res);
		return -3;
	}
	else
	{

		mm_segment_t old_fs = get_fs();
		char strbuf[MXC622X_BUFSIZE];
		s16   data[MXC622X_AXES_NUM+1];
		struct file *fp;
		set_fs(KERNEL_DS);
#if 0
		fp = filp_open(FTM_CUST_ACC,O_RDONLY,0);

		if(fp != 0xfffffffe)//open file suscess
		{
		//fp->f_op->llseek(fp, 0, 0);
			fp->f_pos = 0;
			fp->f_op->read(fp,
			strbuf,
			MXC622X_BUFSIZE,
			&fp->f_pos);	
			filp_close(fp, NULL);	
			set_fs(old_fs);	
			sscanf(strbuf, "%02x %02x %02x", &data[MXC622X_AXIS_X], &data[MXC622X_AXIS_Y], &data[MXC622X_AXIS_Z]);
			obj->data[MXC622X_AXIS_X] -= data[MXC622X_AXIS_X];
			obj->data[MXC622X_AXIS_Y] -= data[MXC622X_AXIS_Y];
			obj->data[MXC622X_AXIS_Z] -= (data[MXC622X_AXIS_Z]-(obj->cvt.sign[MXC622X_AXIS_Z]*obj->reso->sensitivity));		
			//printk("BMA220_SET_CALIBRATION value is %x %x %x -> %d %d %d\r\n",data[BMA220_AXIS_X], data[BMA220_AXIS_Y], data[BMA220_AXIS_Z],obj->data[BMA220_AXIS_X],obj->data[BMA220_AXIS_Y],obj->data[BMA220_AXIS_Z]);
		}
#endif 	
//printk("raw data x=%d, y=%d, z=%d \n",obj->data[BMA150_AXIS_X],obj->data[BMA150_AXIS_Y],obj->data[BMA150_AXIS_Z]);
		//obj->data[MXC622X_AXIS_X] += obj->cali_sw[MXC622X_AXIS_X];
		//obj->data[MXC622X_AXIS_Y] += obj->cali_sw[MXC622X_AXIS_Y];
		//obj->data[MXC622X_AXIS_Z] += obj->cali_sw[MXC622X_AXIS_Z];

		/*remap coordinate*/
		//acc[obj->cvt.map[MXC622X_AXIS_X]] = obj->cvt.sign[MXC622X_AXIS_X]*obj->data[MXC622X_AXIS_X];
		//acc[obj->cvt.map[MXC622X_AXIS_Y]] = obj->cvt.sign[MXC622X_AXIS_Y]*obj->data[MXC622X_AXIS_Y];
		//acc[obj->cvt.map[MXC622X_AXIS_Z]] = obj->cvt.sign[MXC622X_AXIS_Z]*obj->data[MXC622X_AXIS_Z];
		//printk("cvt x=%d, y=%d, z=%d \n",obj->cvt.sign[MXC622X_AXIS_X],obj->cvt.sign[MXC622X_AXIS_Y]);

	        //printk("+++++++++++++ acc : %d, %d\n", obj->data[MXC622X_AXIS_X], obj->data[MXC622X_AXIS_Y]);
                acc[MXC622X_AXIS_X] = (obj->data[MXC622X_AXIS_X]);
		acc[MXC622X_AXIS_Y] = obj->data[MXC622X_AXIS_Y];
#if 0	
	printk("++++++before+++++++ acc : %d, %d\n", acc[0], acc[1]);
		
		if(acc[MXC622X_AXIS_X]>127)
		{
		
		 acc[MXC622X_AXIS_X]=0-(255-acc[MXC622X_AXIS_X]);
		
		}
		else
		{
		acc[MXC622X_AXIS_X]=acc[MXC622X_AXIS_X];
		
		}
		
		
		if(databuf[MXC622X_AXIS_Y]>127)
		{
		
		  acc[MXC622X_AXIS_Y]=0-(255-acc[MXC622X_AXIS_Y]);
		
		}
		else
		{
		 acc[MXC622X_AXIS_Y]=acc[MXC622X_AXIS_Y];
		
		}
#endif
		//	printk("+++++atfer ++++++++ acc : %d, %d\n", acc[0], acc[1]);
		//GSE_LOG("Mapped gsensor data: %d, %d!\n", acc[MXC622X_AXIS_X], acc[MXC622X_AXIS_Y]);
		//Out put the mg
		acc[MXC622X_AXIS_X] =- acc[MXC622X_AXIS_X] * GRAVITY_EARTH_1000 / 64;
		acc[MXC622X_AXIS_Y] = acc[MXC622X_AXIS_Y] * GRAVITY_EARTH_1000 / 64;
		acc[MXC622X_AXIS_Z] = 32 * GRAVITY_EARTH_1000 / 64;

		//printk("test black  acc : %d, %d, %d\n", acc[0], acc[1], acc[2]);
		sprintf(buf, "%04x %04x %04x", acc[MXC622X_AXIS_X], acc[MXC622X_AXIS_Y], acc[MXC622X_AXIS_Z]);
		/*
		if(atomic_read(&obj->trace) & ADX_TRC_IOCTL)
		{
			GSE_LOG("gsensor data: %s!\n", buf);
		}*/
	}
	
	return 0;
}
/*----------------------------------------------------------------------------*/
static int MXC622X_ReadRawData(struct i2c_client *client, char *buf)
{
	struct mxc622x_i2c_data *obj = (struct mxc622x_i2c_data*)i2c_get_clientdata(client);
	int res = 0;

	if (!buf || !client)
	{
		return EINVAL;
	}
	
	if((res = MXC622X_ReadData(client, obj->data)))
	{        
		GSE_ERR("I2C error: ret value=%d", res);
		return EIO;
	}
	else
	{
		sprintf(buf, "%04x %04x %04x", obj->data[MXC622X_AXIS_X], 
			obj->data[MXC622X_AXIS_Y], obj->data[MXC622X_AXIS_Z]);
	
	}
	
	return 0;
}
/*----------------------------------------------------------------------------*/
static int MXC622X_SET_CALIBRATION(struct i2c_client *client)
{
	struct mxc622x_i2c_data *obj = (struct mxc622x_i2c_data*)i2c_get_clientdata(client);
	char strbuf[MXC622X_BUFSIZE];
	s16   data[MXC622X_AXES_NUM+1];
	struct file *fp;
	MXC622X_ReadData(client, data);
	sprintf(strbuf, "%02x %02x %02x", data[MXC622X_AXIS_X], data[MXC622X_AXIS_Y], data[MXC622X_AXIS_Z]);	
	//write to nvram
	mm_segment_t old_fs = get_fs();
	set_fs(KERNEL_DS);

	fp = filp_open(FTM_CUST_ACC,O_WRONLY|O_CREAT, 0644);
	fp->f_pos = 0;
	fp->f_op->write(fp,
				  strbuf,
				  MXC622X_BUFSIZE,
				  &fp->f_pos);	
	filp_close(fp, NULL);	
	printk("MXC622X_SET_CALIBRATION value is %x %x %x\r\n",data[MXC622X_AXIS_X], data[MXC622X_AXIS_Y], data[MXC622X_AXIS_Z]);

	set_fs(old_fs);	
	return 0;
}




static ssize_t show_chipinfo_value(struct device_driver *ddri, char *buf)
{
	struct i2c_client *client = mxc622x_i2c_client;
	char strbuf[MXC622X_BUFSIZE];
	if(NULL == client)
	{
		GSE_ERR("i2c client is null!!\n");
		return 0;
	}
	
	MXC622X_ReadChipInfo(client, strbuf, MXC622X_BUFSIZE);
	return snprintf(buf, PAGE_SIZE, "%s\n", strbuf);        
}

static ssize_t gsensor_init(struct device_driver *ddri, char *buf, size_t count)
	{
		struct i2c_client *client = mxc622x_i2c_client;
		char strbuf[MXC622X_BUFSIZE];
		
		if(NULL == client)
		{
			GSE_ERR("i2c client is null!!\n");
			return 0;
		}
		mxc622x_init_client(client, 1);
		return snprintf(buf, PAGE_SIZE, "%s\n", strbuf);			
	}



/*----------------------------------------------------------------------------*/
static ssize_t show_sensordata_value(struct device_driver *ddri, char *buf)
{
	struct i2c_client *client = mxc622x_i2c_client;
	char strbuf[MXC622X_BUFSIZE];
	
	if(NULL == client)
	{
		GSE_ERR("i2c client is null!!\n");
		return 0;
	}
	MXC622X_ReadSensorData(client, strbuf, MXC622X_BUFSIZE);
	//BMA150_ReadRawData(client, strbuf);
	return snprintf(buf, PAGE_SIZE, "%s\n", strbuf);            
}

static ssize_t show_sensorrawdata_value(struct device_driver *ddri, char *buf, size_t count)
	{
		struct i2c_client *client = mxc622x_i2c_client;
		char strbuf[MXC622X_BUFSIZE];
		
		if(NULL == client)
		{
			GSE_ERR("i2c client is null!!\n");
			return 0;
		}
		//BMA150_ReadSensorData(client, strbuf, BMA150_BUFSIZE);
		MXC622X_ReadRawData(client, strbuf);
		return snprintf(buf, PAGE_SIZE, "%s\n", strbuf);			
	}

/*----------------------------------------------------------------------------*/
static ssize_t show_cali_value(struct device_driver *ddri, char *buf)
{
	struct i2c_client *client = mxc622x_i2c_client;
	struct mxc622x_i2c_data *obj;
	int err, len = 0, mul;
	int tmp[MXC622X_AXES_NUM];

	if(NULL == client)
	{
		GSE_ERR("i2c client is null!!\n");
		return 0;
	}

	obj = i2c_get_clientdata(client);



	if(err = MXC622X_ReadOffset(client, obj->offset))
	{
		return -EINVAL;
	}
	else if(err = MXC622X_ReadCalibration(client, tmp))
	{
		return -EINVAL;
	}
	else
	{    
		mul = obj->reso->sensitivity/mxc622x_offset_resolution.sensitivity;
		len += snprintf(buf+len, PAGE_SIZE-len, "[HW ][%d] (%+3d, %+3d, %+3d) : (0x%02X, 0x%02X, 0x%02X)\n", mul,                        
			obj->offset[MXC622X_AXIS_X], obj->offset[MXC622X_AXIS_Y], obj->offset[MXC622X_AXIS_Z],
			obj->offset[MXC622X_AXIS_X], obj->offset[MXC622X_AXIS_Y], obj->offset[MXC622X_AXIS_Z]);
		len += snprintf(buf+len, PAGE_SIZE-len, "[SW ][%d] (%+3d, %+3d, %+3d)\n", 1, 
			obj->cali_sw[MXC622X_AXIS_X], obj->cali_sw[MXC622X_AXIS_Y], obj->cali_sw[MXC622X_AXIS_Z]);

		len += snprintf(buf+len, PAGE_SIZE-len, "[ALL]    (%+3d, %+3d, %+3d) : (%+3d, %+3d, %+3d)\n", 
			obj->offset[MXC622X_AXIS_X]*mul + obj->cali_sw[MXC622X_AXIS_X],
			obj->offset[MXC622X_AXIS_Y]*mul + obj->cali_sw[MXC622X_AXIS_Y],
			obj->offset[MXC622X_AXIS_Z]*mul + obj->cali_sw[MXC622X_AXIS_Z],
			tmp[MXC622X_AXIS_X], tmp[MXC622X_AXIS_Y], tmp[MXC622X_AXIS_Z]);
		
		return len;
    }
}
/*----------------------------------------------------------------------------*/
static ssize_t store_cali_value(struct device_driver *ddri, char *buf, size_t count)
{
	struct i2c_client *client = mxc622x_i2c_client;  
	int err, x, y, z;
	int dat[MXC622X_AXES_NUM];

	if(!strncmp(buf, "rst", 3))
	{
		if(err = MXC622X_ResetCalibration(client))
		{
			GSE_ERR("reset offset err = %d\n", err);
		}	
	}
	else if(3 == sscanf(buf, "0x%02X 0x%02X 0x%02X", &x, &y, &z))
	{
		dat[MXC622X_AXIS_X] = x;
		dat[MXC622X_AXIS_Y] = y;
		dat[MXC622X_AXIS_Z] = z;
		if(err = MXC622X_WriteCalibration(client, dat))
		{
			GSE_ERR("write calibration err = %d\n", err);
		}		
	}
	else
	{
		GSE_ERR("invalid format\n");
	}
	
	return count;
}


/*----------------------------------------------------------------------------*/
static ssize_t show_firlen_value(struct device_driver *ddri, char *buf)
{
#ifdef CONFIG_MXC622X_LOWPASS
	struct i2c_client *client = mxc622x_i2c_client;
	struct mxc622x_i2c_data *obj = i2c_get_clientdata(client);
	if(atomic_read(&obj->firlen))
	{
		int idx, len = atomic_read(&obj->firlen);
		GSE_LOG("len = %2d, idx = %2d\n", obj->fir.num, obj->fir.idx);

		for(idx = 0; idx < len; idx++)
		{
			GSE_LOG("[%5d %5d %5d]\n", obj->fir.raw[idx][MXC622X_AXIS_X], obj->fir.raw[idx][MXC622X_AXIS_Y], obj->fir.raw[idx][MXC622X_AXIS_Z]);
		}
		
		GSE_LOG("sum = [%5d %5d %5d]\n", obj->fir.sum[MXC622X_AXIS_X], obj->fir.sum[MXC622X_AXIS_Y], obj->fir.sum[MXC622X_AXIS_Z]);
		GSE_LOG("avg = [%5d %5d %5d]\n", obj->fir.sum[MXC622X_AXIS_X]/len, obj->fir.sum[MXC622X_AXIS_Y]/len, obj->fir.sum[MXC622X_AXIS_Z]/len);
	}
	return snprintf(buf, PAGE_SIZE, "%d\n", atomic_read(&obj->firlen));
#else
	return snprintf(buf, PAGE_SIZE, "not support\n");
#endif
}
/*----------------------------------------------------------------------------*/
static ssize_t store_firlen_value(struct device_driver *ddri, char *buf, size_t count)
{
#ifdef CONFIG_MXC622X_LOWPASS
	struct i2c_client *client = mxc622x_i2c_client;  
	struct mxc622x_i2c_data *obj = i2c_get_clientdata(client);
	int firlen;

	if(1 != sscanf(buf, "%d", &firlen))
	{
		GSE_ERR("invallid format\n");
	}
	else if(firlen > C_MAX_FIR_LENGTH)
	{
		GSE_ERR("exceeds maximum filter length\n");
	}
	else
	{ 
		atomic_set(&obj->firlen, firlen);
		if(NULL == firlen)
		{
			atomic_set(&obj->fir_en, 0);
		}
		else
		{
			memset(&obj->fir, 0x00, sizeof(obj->fir));
			atomic_set(&obj->fir_en, 1);
		}
	}
#endif    
	return count;
}
/*----------------------------------------------------------------------------*/
static ssize_t show_trace_value(struct device_driver *ddri, char *buf)
{
	ssize_t res;
	struct mxc622x_i2c_data *obj = obj_i2c_data;
	if (obj == NULL)
	{
		GSE_ERR("i2c_data obj is null!!\n");
		return 0;
	}
	
	res = snprintf(buf, PAGE_SIZE, "0x%04X\n", atomic_read(&obj->trace));     
	return res;    
}
/*----------------------------------------------------------------------------*/
static ssize_t store_trace_value(struct device_driver *ddri, char *buf, size_t count)
{
	struct mxc622x_i2c_data *obj = obj_i2c_data;
	int trace;
	if (obj == NULL)
	{
		GSE_ERR("i2c_data obj is null!!\n");
		return 0;
	}
	
	if(1 == sscanf(buf, "0x%x", &trace))
	{
		atomic_set(&obj->trace, trace);
	}	
	else
	{
		GSE_ERR("invalid content: '%s', length = %d\n", buf, count);
	}
	
	return count;    
}
/*----------------------------------------------------------------------------*/
static ssize_t show_status_value(struct device_driver *ddri, char *buf)
{
	ssize_t len = 0;    
	struct mxc622x_i2c_data *obj = obj_i2c_data;
	if (obj == NULL)
	{
		GSE_ERR("i2c_data obj is null!!\n");
		return 0;
	}	
	
	if(obj->hw)
	{
		len += snprintf(buf+len, PAGE_SIZE-len, "CUST: %d %d (%d %d)\n", 
	            obj->hw->i2c_num, obj->hw->direction, obj->hw->power_id, obj->hw->power_vol);   
	}
	else
	{
		len += snprintf(buf+len, PAGE_SIZE-len, "CUST: NULL\n");
	}
	return len;    
}
/*----------------------------------------------------------------------------*/
static ssize_t show_power_status_value(struct device_driver *ddri, char *buf)
{
	//if(sensor_power)
	#if 0
	if(((bool)atomic_read(&obj->sensor_power)))
		printk("G sensor is in work mode, sensor_power = %d\n", ((bool)atomic_read(&obj->sensor_power)));
	else
		printk("G sensor is in standby mode, sensor_power = %d\n", ((bool)atomic_read(&obj->sensor_power)));
     #endif
	 
	return 0;
}
/*----------------------------------------------------------------------------*/
static DRIVER_ATTR(chipinfo,   S_IWUSR | S_IRUGO, show_chipinfo_value,      NULL);
static DRIVER_ATTR(sensordata, S_IWUSR | S_IRUGO, show_sensordata_value,    NULL);
static DRIVER_ATTR(cali,       S_IWUSR | S_IRUGO, show_cali_value,          store_cali_value);
static DRIVER_ATTR(firlen,     S_IWUSR | S_IRUGO, show_firlen_value,        store_firlen_value);
static DRIVER_ATTR(trace,      S_IWUSR | S_IRUGO, show_trace_value,         store_trace_value);
static DRIVER_ATTR(status,               S_IRUGO, show_status_value,        NULL);
static DRIVER_ATTR(powerstatus,               S_IRUGO, show_power_status_value,        NULL);
/*----------------------------------------------------------------------------*/
static struct driver_attribute *mxc622x_attr_list[] = {
	&driver_attr_chipinfo,     /*chip information*/
	&driver_attr_sensordata,   /*dump sensor data*/
	&driver_attr_cali,         /*show calibration data*/
	&driver_attr_firlen,       /*filter length: 0: disable, others: enable*/
	&driver_attr_trace,        /*trace log*/
	&driver_attr_status,
	&driver_attr_powerstatus,
};
/*----------------------------------------------------------------------------*/

//BEGIN: fangjie add for Gsensor compatible design for MMITest G-sensor test. 
#if 1 //#ifdef SYS_COMPATIBLE
static ssize_t check_gsensorid_show(struct device *dev,
                                     struct device_attribute *attr, char *buf)
{
	unsigned char *strid=NULL;
	strid = "MXC6255XC";
	return sprintf(buf, "%s\n", strid);
}
static DEVICE_ATTR(gsensorid, S_IRUGO | S_IWUSR, check_gsensorid_show, NULL);
#endif
//END: fangjie add for Gsensor compatible design for MMITest G-sensor test. 

static int mxc622x_create_attr(struct device_driver *driver) 
{
	int idx, err = 0;
	int num = (int)(sizeof(mxc622x_attr_list)/sizeof(mxc622x_attr_list[0]));
	if (driver == NULL)
	{
		return -EINVAL;
	}

	for(idx = 0; idx < num; idx++)
	{
		if(err = driver_create_file(driver, mxc622x_attr_list[idx]))
		{            
			GSE_ERR("driver_create_file (%s) = %d\n", mxc622x_attr_list[idx]->attr.name, err);
			break;
		}
	}    
	return err;
}
/*----------------------------------------------------------------------------*/
static int mxc622x_delete_attr(struct device_driver *driver)
{
	int idx ,err = 0;
	int num = (int)(sizeof(mxc622x_attr_list)/sizeof(mxc622x_attr_list[0]));

	if(driver == NULL)
	{
		return -EINVAL;
	}
	

	for(idx = 0; idx < num; idx++)
	{
		driver_remove_file(driver, mxc622x_attr_list[idx]);
	}
	

	return err;
}

/*----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/

	 int mxc622x_gsensor_operate(void* self, uint32_t command, void* buff_in, int size_in,
		void* buff_out, int size_out, int* actualout)
{
	int err = 0;
	int value, sample_delay;	
	struct mxc622x_i2c_data *priv = (struct mxc622x_i2c_data*)self;
	hwm_sensor_data* gsensor_data;
	char buff[MXC622X_BUFSIZE];
	bool power_mode;
	
	//GSE_FUN(f);
	switch (command)
	{
		case SENSOR_DELAY:
			break;

		case SENSOR_ENABLE:
			if((buff_in == NULL) || (size_in < sizeof(int)))
			{
				GSE_ERR("Enable sensor parameter error!\n");
				err = -EINVAL;
			}
			else
			{
				value = *(int *)buff_in;
				//if(((value == 0) && (sensor_power == false)) ||((value == 1) && (sensor_power == true)))
				power_mode = (bool)atomic_read(&priv->sensor_power);
				if(((value == 0) && (power_mode == false)) ||((value == 1) && (power_mode == true)))
				{
					GSE_LOG("Gsensor device have updated!\n");
				}
				else
				{
					//err = MXC622X_SetPowerMode(priv->client, !sensor_power); 
					err = MXC622X_SetPowerMode(priv->client, !power_mode);
				}
			}
			break;

		case SENSOR_GET_DATA:
			if((buff_out == NULL) || (size_out< sizeof(hwm_sensor_data)))
			{
				GSE_ERR("get sensor data parameter error!\n");
				err = -EINVAL;
			}
			else
			{
				gsensor_data = (hwm_sensor_data *)buff_out;

				err = MXC622X_ReadSensorData(priv->client, buff, MXC622X_BUFSIZE);
				if(!err)
				{
				   sscanf(buff, "%x %x %x", &gsensor_data->values[0], 
					   &gsensor_data->values[1], &gsensor_data->values[2]);				
				   gsensor_data->status = SENSOR_STATUS_ACCURACY_MEDIUM;				
				   gsensor_data->value_divide = 1000;
				}
			}
			break;
		default:
			GSE_ERR("gsensor operate function no this parameter %d!\n", command);
			err = -1;
			break;
	}
	
	return err;
}

/****************************************************************************** 
 * Function Configuration
******************************************************************************/
static int mxc622x_open(struct inode *inode, struct file *file)
{
	
        file->private_data = mxc622x_i2c_client;

	if(file->private_data == NULL)
	{
		GSE_ERR("null pointer!!\n");
		return -EINVAL;
	}
	return nonseekable_open(inode, file);

}
/*----------------------------------------------------------------------------*/
static int mxc622x_release(struct inode *inode, struct file *file)
{
	file->private_data = NULL;
	return 0;
}
/*----------------------------------------------------------------------------*/
//static int mxc622x_ioctl(struct inode *inode, struct file *file, unsigned int cmd,
//       unsigned long arg)
static long mxc622x_unlocked_ioctl(struct file *file, unsigned int cmd,unsigned long arg)
{
	struct i2c_client *client = (struct i2c_client*)file->private_data;
	struct mxc622x_i2c_data *obj = (struct mxc622x_i2c_data*)i2c_get_clientdata(client);	
	char strbuf[MXC622X_BUFSIZE];
	void __user *data;
	SENSOR_DATA sensor_data;
	int err = MXC622X_SUCCESS;
	int cali[3];

	//GSE_FUN(f);
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
		GSE_ERR("access error: %08X, (%2d, %2d)\n", cmd, _IOC_DIR(cmd), _IOC_SIZE(cmd));
		return -EFAULT;
	}

        printk("mxc622x_ioctl cmd = 0x%x\n",cmd);

	switch(cmd)
	{
		case GSENSOR_IOCTL_INIT:
			mxc622x_init_client(client, 0);	      
			printk("mxc622x_ioctl GSENSOR_IOCTL_INIT = 0x%x\n",GSENSOR_IOCTL_INIT);		
			break;

		case GSENSOR_IOCTL_READ_CHIPINFO:
			data = (void __user *) arg;
			printk("mxc622x_ioctl GSENSOR_IOCTL_READ_CHIPINFO = 0x%x\n",GSENSOR_IOCTL_READ_CHIPINFO);		
			if(data == NULL)
			{
				err = -EINVAL;
				break;	  
			}
			
			MXC622X_ReadChipInfo(client, strbuf, MXC622X_BUFSIZE);
			if(copy_to_user(data, strbuf, strlen(strbuf)+1))
			{
				err = -EFAULT;
				break;
			}				 
			break;	  

		case GSENSOR_IOCTL_READ_SENSORDATA:
			data = (void __user *) arg;
			printk("mxc622x_ioctl GSENSOR_IOCTL_READ_SENSORDATA = 0x%x\n",GSENSOR_IOCTL_READ_SENSORDATA);	
			if(data == NULL)
			{
				err = -EINVAL;
				break;	  
			}
			
			MXC622X_ReadSensorData(client, strbuf, MXC622X_BUFSIZE);
			if(copy_to_user(data, strbuf, strlen(strbuf)+1))
			{
				err = -EFAULT;
				break;	  
			}				 
			break;

		case GSENSOR_IOCTL_READ_GAIN:
			data = (void __user *) arg;
			printk("mxc622x_ioctl GSENSOR_IOCTL_READ_GAIN = 0x%x\n",GSENSOR_IOCTL_READ_GAIN);	
			if(data == NULL)
			{
				err = -EINVAL;
				break;	  
			}			
			
			if(copy_to_user(data, &gsensor_gain, sizeof(GSENSOR_VECTOR3D)))
			{
				err = -EFAULT;
				break;
			}				 
			break;

		case GSENSOR_IOCTL_READ_RAW_DATA:
			data = (void __user *) arg;
			printk("mxc622x_ioctl GSENSOR_IOCTL_READ_RAW_DATA = 0x%x\n",GSENSOR_IOCTL_READ_RAW_DATA);	
			if(data == NULL)
			{
				err = -EINVAL;
				break;	  
			}
			MXC622X_ReadRawData(client, strbuf);
			if(copy_to_user(data, strbuf, strlen(strbuf)+1))
			{
				err = -EFAULT;
				break;	  
			}
			break;	  

		case GSENSOR_IOCTL_SET_CALI:
			data = (void __user*)arg;
			printk("mxc622x_ioctl GSENSOR_IOCTL_SET_CALI 1 = 0x%x\n",GSENSOR_IOCTL_SET_CALI);	
			if(data == NULL)
			{
				err = -EINVAL;
				break;	  
			}
			printk("mxc622x_ioctl GSENSOR_IOCTL_SET_CALI 2\n");	
			if(copy_from_user(&sensor_data, data, sizeof(sensor_data)))
			{
				err = -EFAULT;
				break;	  
			}
			printk("mxc622x_ioctl GSENSOR_IOCTL_SET_CALI 3\n");	
			if(atomic_read(&obj->suspend))
			{
				GSE_ERR("Perform calibration in suspend state!!\n");
				printk("mxc622x_ioctl GSENSOR_IOCTL_SET_CALI 4\n");	
				err = -EINVAL;
			}
			else
			{
                                printk("mxc622x_ioctl GSENSOR_IOCTL_SET_CALI 5\n");	
				//cali[MXC622X_AXIS_X] = sensor_data.x * obj->reso->sensitivity / GRAVITY_EARTH_1000;
				//cali[MXC622X_AXIS_Y] = sensor_data.y * obj->reso->sensitivity / GRAVITY_EARTH_1000;
				//cali[MXC622X_AXIS_Z] = sensor_data.z * obj->reso->sensitivity / GRAVITY_EARTH_1000;			  
				err = MXC622X_WriteCalibration(client, cali);			 
			}
			break;

		case GSENSOR_IOCTL_CLR_CALI:
			printk("mxc622x_ioctl GSENSOR_IOCTL_CLR_CALI = 0x%x\n",GSENSOR_IOCTL_CLR_CALI);	
			err = MXC622X_ResetCalibration(client);
			break;

		case GSENSOR_IOCTL_GET_CALI:
			data = (void __user*)arg;
			printk("mxc622x_ioctl GSENSOR_IOCTL_GET_CALI = 0x%x\n",GSENSOR_IOCTL_GET_CALI);	
			if(data == NULL)
			{
				err = -EINVAL;
				break;	  
			}
			if((err = MXC622X_ReadCalibration(client, cali)))
			{
				break;
			}
		/*	
			sensor_data.x = cali[MXC622X_AXIS_X] * GRAVITY_EARTH_1000 / obj->reso->sensitivity;
			sensor_data.y = cali[MXC622X_AXIS_Y] * GRAVITY_EARTH_1000 / obj->reso->sensitivity;
			sensor_data.z = cali[MXC622X_AXIS_Z] * GRAVITY_EARTH_1000 / obj->reso->sensitivity;
			if(copy_to_user(data, &sensor_data, sizeof(sensor_data)))
			{
				err = -EFAULT;
				break;
			}	
                 */	
			break;
	
		//case GSENSOR_IOCTL_SET_CALIBRATION:
		//     printk("mxc622x_ioctl GSENSOR_IOCTL_SET_CALIBRATION = 0x%x\n",GSENSOR_IOCTL_SET_CALIBRATION);				
		//     err = MXC622X_SET_CALIBRATION(client);

		//break;	
	
		default:
		       printk("mxc622x_ioctl default = 0x%x\n",cmd);	
			GSE_ERR("unknown IOCTL: 0x%08x\n", cmd);
			err = -ENOIOCTLCMD;
			break;
			
	}
        printk("mxc622x_ioctl err = 0x%x\n",err);		
	return err;
}


/*----------------------------------------------------------------------------*/
static struct file_operations mxc622x_fops = {
	.owner = THIS_MODULE,
	.open = mxc622x_open,
	.release = mxc622x_release,
	.unlocked_ioctl = mxc622x_unlocked_ioctl,
	//.ioctl = mxc622x_ioctl,
};
/*----------------------------------------------------------------------------*/
static struct miscdevice mxc622x_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "gsensor",
	.fops = &mxc622x_fops,
};
/*----------------------------------------------------------------------------*/
//#ifndef CONFIG_HAS_EARLYSUSPEND
/*----------------------------------------------------------------------------*/
static int mxc622x_suspend(struct i2c_client *client, pm_message_t msg) 
{
	struct mxc622x_i2c_data *obj = i2c_get_clientdata(client);    
	int err = 0;
	GSE_FUN();    

	if(msg.event == PM_EVENT_SUSPEND)
	{   
		
		if(obj == NULL)
		{
			GSE_ERR("null pointer!!\n");
			return -EINVAL;
		}
		atomic_set(&obj->suspend, 1);
		
		if((err = MXC622X_SetPowerMode(obj->client, false)))
		{
			GSE_ERR("write power control fail!!\n");
			return err;
		}        
		MXC622X_power(obj->hw, 0);
		GSE_LOG("mxc622x_suspend ok\n");
	}
	return err;
}
/*----------------------------------------------------------------------------*/
static int mxc622x_resume(struct i2c_client *client)
{
	struct mxc622x_i2c_data *obj = i2c_get_clientdata(client);        
	int err;
	GSE_FUN();

	if(obj == NULL)
	{
		GSE_ERR("null pointer!!\n");
		return -EINVAL;
	}

	MXC622X_power(obj->hw, 1);
	if((err = mxc622x_init_client(client, 0)))
	{
		GSE_ERR("initialize client fail!!\n");
		return err;        
	}
	atomic_set(&obj->suspend, 0);
	GSE_LOG("mxc622x_resume ok\n");

	return 0;
}
/*----------------------------------------------------------------------------*/
//#else /*CONFIG_HAS_EARLY_SUSPEND is defined*/
/*----------------------------------------------------------------------------*/
static void mxc622x_early_suspend(struct early_suspend *h) 
{
 	int err;
	struct mxc622x_i2c_data *obj = container_of(h, struct mxc622x_i2c_data, early_drv);
	GSE_FUN();    

	if(obj == NULL)
	{
		GSE_ERR("null pointer!!\n");
		return;
	}
	atomic_set(&obj->suspend, 1); 
	if((err = MXC622X_SetPowerMode(obj->client, false)))
	{
		GSE_ERR("write power control fail!!\n");
		return;
	}

	//sensor_power = false;
	atomic_set(&obj->sensor_power,  0);
	
	//MXC622X_power(obj->hw, 0);
}
/*----------------------------------------------------------------------------*/
static void mxc622x_late_resume(struct early_suspend *h)
{
	int err;
	struct mxc622x_i2c_data *obj = container_of(h, struct mxc622x_i2c_data, early_drv);
	if((err = MXC622X_SetPowerMode(obj->client, true)))
	{
		GSE_ERR("write power control fail!!\n");
		return;
	}
	//sensor_power = true;
		atomic_set(&obj->sensor_power,  1);

}
/*----------------------------------------------------------------------------*/
//#endif /*CONFIG_HAS_EARLYSUSPEND*/
/*----------------------------------------------------------------------------*/
static int mxc622x_i2c_detect(struct i2c_client *client, int kind, struct i2c_board_info *info) 
{    
	strcpy(info->type, MXC622X_DEV_NAME);
	return 0;
}

/*----------------------------------------------------------------------------*/
static int mxc622x_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct i2c_client *new_client;
	struct mxc622x_i2c_data *obj;
	struct hwmsen_object sobj;
	int err = 0;

	GSE_FUN();
	
	if(!(obj = kzalloc(sizeof(*obj), GFP_KERNEL)))
	{
		err = -ENOMEM;
		goto exit;
	}
	
	memset(obj, 0, sizeof(struct mxc622x_i2c_data));

	obj->hw = get_cust_acc_hw();
	      printk("tangzibo_mxc622x_i2c_probe obj->hw->direction=%x\n ", obj->hw->direction);
	if((err = hwmsen_get_convert(obj->hw->direction, &obj->cvt)))
	{
		printk("tangzibo invalid direction: %d\n", obj->hw->direction);
		goto exit;
	}

	obj_i2c_data = obj;
	obj->client = client;
	new_client = obj->client;
	i2c_set_clientdata(new_client,obj);
	
	atomic_set(&obj->trace, 0);
	atomic_set(&obj->suspend, 0);
	atomic_set(&obj->sensor_power,  1);
	
#ifdef CONFIG_MXC622X_LOWPASS
	if(obj->hw->firlen > C_MAX_FIR_LENGTH)
	{
		atomic_set(&obj->firlen, C_MAX_FIR_LENGTH);
	}	
	else
	{
		atomic_set(&obj->firlen, obj->hw->firlen);
	}
	
	if(atomic_read(&obj->firlen) > 0)
	{
		atomic_set(&obj->fir_en, 1);
	}
	
#endif

	mxc622x_i2c_client = new_client;	

	if((err = mxc622x_init_client(new_client, 1)))
	{
		goto exit_init_failed;
	}
		gsensor_init_status=MXC622X_SUCCESS; //fangjie add

	if((err = misc_register(&mxc622x_device)))
	{
		GSE_ERR("mxc622x_device register failed\n");
		goto exit_misc_device_register_failed;
	}
		 //BEGIN: add by fangjie
		#ifdef MXC622X_ATTR_CREATE //fangjie add
			#if 0 //old
			 if(err = mxc622x_create_attr(&mxc622x_gsensor_driver.driver))
			 {
				 GSE_ERR("create attribute err = %d\n", err);
				 goto exit_create_attr_failed;
			 }
			 #endif	
			if(err = mxc622x_create_attr(&mxc622x_i2c_driver.driver))
			 {
				 GSE_ERR("create attribute err = %d\n", err);
				 goto exit_create_attr_failed;
			 }
		#endif
		//END: add by fangjie
	
	sobj.self = obj;
	sobj.polling = 1;
	sobj.sensor_operate = mxc622x_gsensor_operate;

	if((err = hwmsen_attach(ID_ACCELEROMETER, &sobj)))
	{
		GSE_ERR("attach fail = %d\n", err);
		goto exit_kfree;
	}

#ifdef CONFIG_HAS_EARLYSUSPEND

	obj->early_drv.level    = EARLY_SUSPEND_LEVEL_DISABLE_FB - 1,
	obj->early_drv.suspend  = mxc622x_early_suspend,
	obj->early_drv.resume   = mxc622x_late_resume,    
	register_early_suspend(&obj->early_drv);	
#endif 

	//BEGIN: fangjie add for Gsensor compatible design for MMITest G-sensor test. 
	#if 1 //#ifdef SYS_COMPATIBLE
		compatible_mxc6255x=hwmsen_get_compatible_dev();
		if (device_create_file(compatible_mxc6255x, &dev_attr_gsensorid) < 0)
		{
			pr_err("Failed to create device file(%s)!\n", dev_attr_gsensorid.attr.name);
		}
		dev_set_drvdata(compatible_mxc6255x, NULL);
	#endif
	//END: fangjie add for Gsensor compatible design for MMITest G-sensor test. 

	GSE_LOG("%s: OK\n", __func__);    
	return 0;

	exit_create_attr_failed:
	misc_deregister(&mxc622x_device);
	exit_misc_device_register_failed:
	exit_init_failed:
	//i2c_detach_client(new_client);
	exit_kfree:
	kfree(obj);
	exit:
	GSE_ERR("%s: err = %d\n", __func__, err);        
	return err;
}

/*----------------------------------------------------------------------------*/
static int mxc622x_i2c_remove(struct i2c_client *client)
{
	int err = 0;	
		#ifdef MXC622X_ATTR_CREATE //fangjie add
			#if 0
			 if(err = mxc622x_delete_attr(&mxc622x_gsensor_driver.driver))
			 {
				 GSE_ERR("mxc622x_delete_attr fail: %d\n", err);
			 }
			 #endif
			if(err = mxc622x_delete_attr(&mxc622x_i2c_driver.driver))
			 {
				 GSE_ERR("mxc622x_delete_attr fail: %d\n", err);
			 }
		#endif
	if((err = misc_deregister(&mxc622x_device)))
	{
		GSE_ERR("misc_deregister fail: %d\n", err);
	}

	if((err = hwmsen_detach(ID_ACCELEROMETER)))
	    

	mxc622x_i2c_client = NULL;
	i2c_unregister_device(client);
	kfree(i2c_get_clientdata(client));
	return 0;
}
#if 0
/*----------------------------------------------------------------------------*/
static int mxc622x_probe(struct platform_device *pdev) 
{
	struct acc_hw *hw = get_cust_acc_hw();
	GSE_FUN();

	MXC622X_power(hw, 1);
	//mxc622x_force[0] = hw->i2c_num;
	if(i2c_add_driver(&mxc622x_i2c_driver))
	{
		GSE_ERR("add driver error\n");
		return -1;
	}

	return 0;
}
	 /*----------------------------------------------------------------------------*/
#endif
static int  mxc622x_local_init(void)
{
   	struct acc_hw *hw = get_cust_acc_hw();
	GSE_FUN();
	MXC622X_power(hw, 1);
	//bma222e_force[0] = hw->i2c_num;
	if(i2c_add_driver(&mxc622x_i2c_driver) )
	{
		GSE_ERR("add driver error\n");
		return -1;
	}	
	return 0;
}

/*----------------------------------------------------------------------------*/
static int mxc622x_remove(struct platform_device *pdev)
{
    struct acc_hw *hw = get_cust_acc_hw();

    GSE_FUN();    
    MXC622X_power(hw, 0);    
    i2c_del_driver(&mxc622x_i2c_driver);
    return 0;
}
/*----------------------------------------------------------------------------*/
static struct sensor_init_info mxc6255x_init_info = {
		.name = "mxc6255x",
		.init =  mxc622x_local_init,
		.uninit =  mxc622x_remove,
	
};

#if 0	 
	 /*----------------------------------------------------------------------------*/
	 static struct platform_driver mxc622x_gsensor_driver = {
		 .probe 	 = mxc622x_probe,
		 .remove	 = mxc622x_remove,	 
		 .driver	 = {
			 .name	= "gsensor",
			 //.owner = THIS_MODULE,
		 }
	 };
#endif

/*----------------------------------------------------------------------------*/
static int __init mxc622x_init(void)
{
        struct acc_hw *hw = get_cust_acc_hw();     //added by xen for test 20131108
		 GSE_FUN();
		 //struct acc_hw *hw = get_cust_acc_hw(); //fangjie
	GSE_LOG("%s: i2c_number=%d\n", __func__,hw->i2c_num);
	i2c_register_board_info(hw->i2c_num, &i2c_mxc622x, 1);
		 //i2c_register_board_info(MXC622X_I2C_NUM, &i2c_mxc622x, 1); //fangjie add
		 
		 /*
		 if(platform_driver_register(&mxc622x_gsensor_driver)) //fangjie
		 {
			 GSE_ERR("failed to register driver");
			 return -ENODEV;
		 }
		 */
		hwmsen_gsensor_add(&mxc6255x_init_info);
	return 0;    
}
/*----------------------------------------------------------------------------*/
static void __exit mxc622x_exit(void)
{
	GSE_FUN();
		 //platform_driver_unregister(&mxc622x_gsensor_driver); //fangjie delete
}
/*----------------------------------------------------------------------------*/
module_init(mxc622x_init);
module_exit(mxc622x_exit);
/*----------------------------------------------------------------------------*/
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("MXC622X I2C driver");
MODULE_AUTHOR("");
