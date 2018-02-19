

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
#include "tmd2772.h"
/*----------------------------------------------------------------------------*/

#define TMD2772_DEV_NAME     "tmd2772"
/*----------------------------------------------------------------------------*/
#define APS_TAG                  "[ALS/PS] "
#define APS_FUN(f)               printk(APS_TAG"%s\n", __FUNCTION__)
#define APS_ERR(fmt, args...)    printk(  APS_TAG"%s %d : "fmt, __FUNCTION__, __LINE__, ##args)
#define APS_LOG(fmt, args...)    printk( APS_TAG fmt, ##args)
#define APS_DBG(fmt, args...)    printk( APS_TAG fmt, ##args)  
#define  GPIO_ALS_EINT_PIN    GPIO28
#define EINT2_PS_N_M_EINT GPIO_MODE_02               

#if 1 //yfpan  
 #define CUST_EINT_POLARITY_LOW         0
 #define CUST_EINT_LEVEL_SENSITIVE      1
 #define CUST_EINT_DEBOUNCE_DISABLE     0
  	
 #define CUST_EINT_ALS_NUM              2
 #define CUST_EINT_ALS_DEBOUNCE_CN      0
 #define CUST_EINT_ALS_POLARITY         CUST_EINT_POLARITY_LOW
 #define CUST_EINT_ALS_SENSITIVE        CUST_EINT_LEVEL_SENSITIVE
 #define CUST_EINT_ALS_DEBOUNCE_EN      CUST_EINT_DEBOUNCE_DISABLE
#endif //yfpan

/*for interrup work mode support --add by liaoxl.lenovo 12.08.2011*/
extern void mt65xx_eint_unmask(unsigned int line);
extern void mt65xx_eint_mask(unsigned int line);
extern void mt65xx_eint_set_polarity(kal_uint8 eintno, kal_bool ACT_Polarity);
extern void mt65xx_eint_set_hw_debounce(kal_uint8 eintno, kal_uint32 ms);
extern kal_uint32 mt65xx_eint_set_sens(kal_uint8 eintno, kal_bool sens);
extern void mt65xx_eint_registration(kal_uint8 eintno, kal_bool Dbounce_En,
										 kal_bool ACT_Polarity, void (EINT_FUNC_PTR)(void),
										 kal_bool auto_umask);
	
/*----------------------------------------------------------------------------*/
static struct i2c_client *tmd2772_i2c_client = NULL;
/*----------------------------------------------------------------------------*/
static const struct i2c_device_id tmd2772_i2c_id[] = {{TMD2772_DEV_NAME,0},{}};
static struct i2c_board_info __initdata i2c_tmd2772={ I2C_BOARD_INFO("tmd2772", (0X72>>1))};

/*----------------------------------------------------------------------------*/
static int tmd2772_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id); 
static int tmd2772_i2c_remove(struct i2c_client *client);
static int tmd2772_i2c_detect(struct i2c_client *client, /*int kind,*/  struct i2c_board_info *info);
/*----------------------------------------------------------------------------*/
static int tmd2772_i2c_suspend(struct i2c_client *client, pm_message_t msg);
static int tmd2772_i2c_resume(struct i2c_client *client);

static struct tmd2772_priv *g_tmd2772_ptr = NULL;

int  boot_ps_ok=0;
 struct PS_CALI_DATA_STRUCT
{
    int close;
    int far_away;
    int valid;
    u8 offset;//add for boot cali
} ;

static struct PS_CALI_DATA_STRUCT ps_cali={{0,0,0},};
static struct PS_CALI_DATA_STRUCT boot_ps_cali={{0,0,0},};

static int intr_flag_value = 0;


/*yi.zheng.hz modify for debug p sensor calibration at 2013-06-27 start*/
#if 1//ndef USER_BUILD_KERNEL  /*only open in engineer version*/
#define DEBUG_TMD2772_CALIBRATION
#endif

#ifdef DEBUG_TMD2772_CALIBRATION
//latest ps value read in eint
u16 ps_latest[2];
#endif
/*yi.zheng.hz modify for debug p sensor calibration at 2013-06-27 end*/

/***************wwl add psensor auto dectet start *******************/
#if defined(MTK_AUTO_DETECT_ALSPS)
static int tmd2772_for_auto_local_init(void);
static int tmd2772_remove(struct platform_device *pdev);

static struct sensor_init_info tmd2772_init_info = {
		.name = TMD2772_DEV_NAME,
		.init = tmd2772_for_auto_local_init,
		.uninit = tmd2772_remove,
	
};
static int tmd2772_init_flag = 0;
extern int hwmsen_alsps_sensor_add(struct sensor_init_info* obj);

#endif//#if defined(MTK_AUTO_DETECT_ALSPS)

/***************wwl add psensor auto dectet   end *******************/

#define USE_CALIBRATION_ON_CALL
static int tmd2772_read_data_for_cali(struct i2c_client *client, struct PS_CALI_DATA_STRUCT *ps_data_cali, int cali_num);
/*----------------------------------------------------------------------------*/
typedef enum {
    CMC_BIT_ALS    = 1,
    CMC_BIT_PS     = 2,
} CMC_BIT;
/*----------------------------------------------------------------------------*/
struct tmd2772_i2c_addr {    /*define a series of i2c slave address*/
    u8  write_addr;  
    u8  ps_thd;     /*PS INT threshold*/
};
/*----------------------------------------------------------------------------*/
struct tmd2772_priv {
    struct alsps_hw  *hw;
    struct i2c_client *client;
    struct work_struct  eint_work;

    /*i2c address group*/
    struct tmd2772_i2c_addr  addr;
    
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
/*----------------------------------------------------------------------------*/
static struct i2c_driver tmd2772_i2c_driver = {	
	.probe      = tmd2772_i2c_probe,
	.remove     = tmd2772_i2c_remove,
	.detect     = tmd2772_i2c_detect,
	.suspend    = tmd2772_i2c_suspend,
	.resume     = tmd2772_i2c_resume,
	.id_table   = tmd2772_i2c_id,
//	.address_data = &tmd2772_addr_data,
	.driver = {
//		.owner          = THIS_MODULE,
		.name           = TMD2772_DEV_NAME,
	},
};

static struct tmd2772_priv *tmd2772_obj = NULL;
/*************wwl add psensor get calibration data when calling    start*****************/
#if  !defined(MTK_AUTO_DETECT_ALSPS)
static struct platform_driver tmd2772_alsps_driver;
#endif
/*************wwl add psensor get calibration data when calling    end*****************/
/*******************************************/
#define C_I2C_FIFO_SIZE         8  
static int tmd2772_i2c_read_block(struct i2c_client *client, u8 addr, u8 *data, u8 len)
{
        u8 beg = addr;
	struct i2c_msg msgs[2] = {
		{
			.addr = client->addr,	.flags = 0,
			.len = 1,	.buf = &beg
		},
		{
			.addr = client->addr,	.flags = I2C_M_RD,
			.len = len,	.buf = data,
		}
	};
	int err;

	if (!client)
		return -EINVAL;
	else if (len > C_I2C_FIFO_SIZE) {
		APS_ERR(" length %d exceeds %d\n", len, C_I2C_FIFO_SIZE);
		return -EINVAL;
	}

	err = i2c_transfer(client->adapter, msgs, sizeof(msgs)/sizeof(msgs[0]));
	if (err != 2) {
		APS_ERR("i2c_transfer error: (%d %p %d) %d\n",
			addr, data, len, err);
		err = -EIO;
	} else {
		err = 0;
	}
	return err;

}
/****************************************************/

/*----------------------------------------------------------------------------*/
int tmd2772_get_addr(struct alsps_hw *hw, struct tmd2772_i2c_addr *addr)
{
	if(!hw || !addr)
	{
		return -EFAULT;
	}
	addr->write_addr= hw->i2c_addr[0];
	return 0;
}
/*----------------------------------------------------------------------------*/
static void tmd2772_power(struct alsps_hw *hw, unsigned int on) 
{
	static unsigned int power_on = 0;

	//APS_LOG("power %s\n", on ? "on" : "off");

	if(hw->power_id != POWER_NONE_MACRO)
	{
		if(power_on == on)
		{
			APS_LOG("ignore power control: %d\n", on);
		}
		else if(on)
		{
			if(!hwPowerOn(hw->power_id, hw->power_vol, "tmd2772")) 
			{
				APS_ERR("power on fails!!\n");
			}
		}
		else
		{
			if(!hwPowerDown(hw->power_id, "tmd2772")) 
			{
				APS_ERR("power off fail!!\n");   
			}
		}
	}
	power_on = on;
}
/*----------------------------------------------------------------------------*/
static long tmd2772_enable_als(struct i2c_client *client, int enable)
{
		struct tmd2772_priv *obj = i2c_get_clientdata(client);
		u8 databuf[2];	  
		long res = 0;
		u8 buffer[1];
		u8 reg_value[1];
		uint32_t testbit_PS;
		
	
		if(client == NULL)
		{
			APS_DBG("CLIENT CANN'T EQUL NULL\n");
			return -1;
		}
		
		#if 0	/*yucong MTK enable_als function modified for fixing reading register error problem 2012.2.16*/
		buffer[0]= TMD2772_CMM_ENABLE;
		res = i2c_master_send(client, buffer, 0x1);
		if(res <= 0)
		{
			goto EXIT_ERR;
		}
		res = i2c_master_recv(client, reg_value, 0x1);
		if(res <= 0)
		{
			goto EXIT_ERR;
		}
		printk("Yucong:0x%x, %d, %s\n", reg_value[0], __LINE__, __FUNCTION__);
		
		if(enable)
		{
			databuf[0] = TMD2772_CMM_ENABLE;	
			databuf[1] = reg_value[0] |0x0B;
			res = i2c_master_send(client, databuf, 0x2);
			if(res <= 0)
			{
				goto EXIT_ERR;
			}
			/*Lenovo-sw chenlj2 add 2011-06-03,modify ps to ALS below two lines */
			atomic_set(&obj->als_deb_on, 1);
			atomic_set(&obj->als_deb_end, jiffies+atomic_read(&obj->als_debounce)/(1000/HZ));
			APS_DBG("tmd2772 power on\n");
		}
		else
		{
			databuf[0] = TMD2772_CMM_ENABLE;	
			databuf[1] = reg_value[0] &0xFD;
			res = i2c_master_send(client, databuf, 0x2);
			if(res <= 0)
			{
				goto EXIT_ERR;
			}
			/*Lenovo-sw chenlj2 add 2011-06-03,modify ps_deb_on to als_deb_on */
			atomic_set(&obj->als_deb_on, 0);
			APS_DBG("tmd2772 power off\n");
		}
		#endif
		#if 1
		/*yucong MTK enable_als function modified for fixing reading register error problem 2012.2.16*/
		testbit_PS = test_bit(CMC_BIT_PS, &obj->enable) ? (1) : (0);
		if(enable)
		{
			if(testbit_PS){	
			databuf[0] = TMD2772_CMM_ENABLE;	
			databuf[1] = 0x2F;
			res = i2c_master_send(client, databuf, 0x2);
			if(res <= 0)
				{
					goto EXIT_ERR;
				}
			/*debug code for reading register value*/
			#if 0
			res = i2c_master_recv(client, reg_value, 0x1);
			if(res <= 0)
				{
					goto EXIT_ERR;
				}
			printk("Yucong:0x%x, %d, %s\n", reg_value[0], __LINE__, __FUNCTION__);
			#endif
			}
			else{
			databuf[0] = TMD2772_CMM_ENABLE;	
			databuf[1] = 0x2B;
			res = i2c_master_send(client, databuf, 0x2);
			if(res <= 0)
				{
					goto EXIT_ERR;
				}

			/*debug code for reading register value*/
			#if 0
			res = i2c_master_recv(client, reg_value, 0x1);
			if(res <= 0)
				{
					goto EXIT_ERR;
				}
			printk("Yucong:0x%x, %d, %s\n", reg_value[0], __LINE__, __FUNCTION__);
			#endif

			}
			atomic_set(&obj->als_deb_on, 1);
			atomic_set(&obj->als_deb_end, jiffies+atomic_read(&obj->als_debounce)/(1000/HZ));
			APS_DBG("tmd2772 power on\n");
		}
		else
		{	
			if(testbit_PS){
			databuf[0] = TMD2772_CMM_ENABLE;	
			databuf[1] = 0x2D;
			res = i2c_master_send(client, databuf, 0x2);
			if(res <= 0)
				{
					goto EXIT_ERR;
				}
			}
			else{
			databuf[0] = TMD2772_CMM_ENABLE;	
			databuf[1] = 0x00;
			res = i2c_master_send(client, databuf, 0x2);
			if(res <= 0)
				{
					goto EXIT_ERR;
				}
			}
			/*Lenovo-sw chenlj2 add 2011-06-03,modify ps_deb_on to als_deb_on */
			atomic_set(&obj->als_deb_on, 0);
			APS_DBG("tmd2772 power off\n");
		}
		#endif
		#if 0 /*yucong add for debug*/
			buffer[0]= TMD2772_CMM_ENABLE;
			res = i2c_master_send(client, buffer, 0x1);
			if(res <= 0)
			{
				goto EXIT_ERR;
			}
			res = i2c_master_recv(client, reg_value, 0x1);
			if(res <= 0)
			{
				goto EXIT_ERR;
			}
			printk("Yucong:0x%x, %d, %s\n", reg_value[0], __LINE__, __FUNCTION__);
		#endif
		
		return 0;
		
	EXIT_ERR:
		APS_ERR("tmd2772_enable_als fail\n");
		return res;
}

/*----------------------------------------------------------------------------*/
static long tmd2772_enable_ps(struct i2c_client *client, int enable)
{
	struct tmd2772_priv *obj = i2c_get_clientdata(client);
	u8 databuf[2];    
	long res = 0;
	u8 buffer[1];
	u8 reg_value[1];
	uint32_t testbit_ALS;

	if(client == NULL)
	{
		APS_DBG("CLIENT CANN'T EQUL NULL\n");
		return -1;
	}
#if 0	/*yucong MTK modified for fixing reading register error problem 2012.2.16*/
	buffer[0]= TMD2772_CMM_ENABLE;
	res = i2c_master_send(client, buffer, 0x1);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	res = i2c_master_recv(client, reg_value, 0x1);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	/*yucong MTK: lenovo orignal code*/
	if(enable)
	{
		databuf[0] = TMD2772_CMM_ENABLE;    
		databuf[1] = reg_value[0] |0x0d;
		res = i2c_master_send(client, databuf, 0x2);
		if(res <= 0)
		{
			goto EXIT_ERR;
		}
		atomic_set(&obj->ps_deb_on, 1);
		atomic_set(&obj->ps_deb_end, jiffies+atomic_read(&obj->ps_debounce)/(1000/HZ));
		APS_DBG("tmd2772 power on\n");

		/*for interrup work mode support -- by liaoxl.lenovo 12.08.2011*/
		if(0 == obj->hw->polling_mode_ps)
		{
			if(1 == ps_cali.valid)
			{
				databuf[0] = TMD2772_CMM_INT_LOW_THD_LOW;	
				databuf[1] = (u8)(ps_cali.far_away & 0x00FF);
				res = i2c_master_send(client, databuf, 0x2);
				if(res <= 0)
				{
					goto EXIT_ERR;
					return TMD2772_ERR_I2C;
				}
				databuf[0] = TMD2772_CMM_INT_LOW_THD_HIGH;	
				databuf[1] = (u8)((ps_cali.far_away & 0xFF00) >> 8);
				res = i2c_master_send(client, databuf, 0x2);
				if(res <= 0)
				{
					goto EXIT_ERR;
					return TMD2772_ERR_I2C;
				}
				databuf[0] = TMD2772_CMM_INT_HIGH_THD_LOW;	
				databuf[1] = (u8)(ps_cali.close & 0x00FF);
				res = i2c_master_send(client, databuf, 0x2);
				if(res <= 0)
				{
					goto EXIT_ERR;
					return TMD2772_ERR_I2C;
				}
				databuf[0] = TMD2772_CMM_INT_HIGH_THD_HIGH; 
				databuf[1] = (u8)((ps_cali.close & 0xFF00) >> 8);;
				res = i2c_master_send(client, databuf, 0x2);
				if(res <= 0)
				{
					goto EXIT_ERR;
					return TMD2772_ERR_I2C;
				}
			}
			else
			{
				databuf[0] = TMD2772_CMM_INT_LOW_THD_LOW;	
				databuf[1] = (u8)(480 & 0x00FF);
				res = i2c_master_send(client, databuf, 0x2);
				if(res <= 0)
				{
					goto EXIT_ERR;
					return TMD2772_ERR_I2C;
				}
				databuf[0] = TMD2772_CMM_INT_LOW_THD_HIGH;	
				databuf[1] = (u8)((480 & 0xFF00) >> 8);
				res = i2c_master_send(client, databuf, 0x2);
				if(res <= 0)
				{
					goto EXIT_ERR;
					return TMD2772_ERR_I2C;
				}
				databuf[0] = TMD2772_CMM_INT_HIGH_THD_LOW;	
				databuf[1] = (u8)(700 & 0x00FF);
				res = i2c_master_send(client, databuf, 0x2);
				if(res <= 0)
				{
					goto EXIT_ERR;
					return TMD2772_ERR_I2C;
				}
				databuf[0] = TMD2772_CMM_INT_HIGH_THD_HIGH; 
				databuf[1] = (u8)((700 & 0xFF00) >> 8);;
				res = i2c_master_send(client, databuf, 0x2);
				if(res <= 0)
				{
					goto EXIT_ERR;
					return TMD2772_ERR_I2C;
				}
		
			}
		
			databuf[0] = TMD2772_CMM_Persistence;
			databuf[1] = 0x20;
			res = i2c_master_send(client, databuf, 0x2);
			if(res <= 0)
			{
				goto EXIT_ERR;
				return TMD2772_ERR_I2C;
			}
			databuf[0] = TMD2772_CMM_ENABLE;	
			databuf[1] = reg_value[0] | 0x0d | 0x20;
			res = i2c_master_send(client, databuf, 0x2);
			if(res <= 0)
			{
				goto EXIT_ERR;
				return TMD2772_ERR_I2C;
			}
		
			mt65xx_eint_unmask(CUST_EINT_ALS_NUM);
		}
	}
	else
	{
		databuf[0] = TMD2772_CMM_ENABLE;    
		databuf[1] = reg_value[0] &0xfb;
		res = i2c_master_send(client, databuf, 0x2);
		if(res <= 0)
		{
			goto EXIT_ERR;
		}
		atomic_set(&obj->ps_deb_on, 0);
		APS_DBG("tmd2772 power off\n");

		/*for interrup work mode support -- by liaoxl.lenovo 12.08.2011*/
		if(0 == obj->hw->polling_mode_ps)
		{
			cancel_work_sync(&obj->eint_work);
			mt65xx_eint_mask(CUST_EINT_ALS_NUM);
		}
	}
#endif
#if 1	
	/*yucong MTK: enable_ps function modified for fixing reading register error problem 2012.2.16*/
	testbit_ALS = test_bit(CMC_BIT_ALS, &obj->enable) ? (1) : (0);
	if(enable)
	{
		if(testbit_ALS){
		databuf[0] = TMD2772_CMM_ENABLE;    
		databuf[1] = 0x0F;
		res = i2c_master_send(client, databuf, 0x2);
		if(res <= 0)
			{
				goto EXIT_ERR;
			}
		/*debug code for reading register value*/
		#if 0
		res = i2c_master_recv(client, reg_value, 0x1);
		if(res <= 0)
			{
				goto EXIT_ERR;
			}
		printk("Yucong:0x%x, %d, %s\n", reg_value[0], __LINE__, __FUNCTION__);
		#endif
		}else{
		databuf[0] = TMD2772_CMM_ENABLE;    
		databuf[1] = 0x0D;
		res = i2c_master_send(client, databuf, 0x2);
		if(res <= 0)
			{
				goto EXIT_ERR;
			}
		}
		/*debug code for reading register value*/
		#if 0
		res = i2c_master_recv(client, reg_value, 0x1);
		if(res <= 0)
			{
				goto EXIT_ERR;
			}
		printk("Yucong:0x%x, %d, %s\n", reg_value[0], __LINE__, __FUNCTION__);
		#endif
		//fix cover hole before call.
		//atomic_set(&obj->ps_deb_on, 1);
		//atomic_set(&obj->ps_deb_end, jiffies+atomic_read(&obj->ps_debounce)/(1000/HZ));
		APS_DBG("tmd2772 power on\n");


		/*for interrup work mode support -- by liaoxl.lenovo 12.08.2011*/
		if(0 == obj->hw->polling_mode_ps)
		{
			//if(1 == ps_cali.valid)
			{
				databuf[0] = TMD2772_CMM_INT_LOW_THD_LOW;	
				databuf[1] = (u8)(ps_cali.far_away & 0x00FF);
				res = i2c_master_send(client, databuf, 0x2);
				if(res <= 0)
				{
					goto EXIT_ERR;
					return TMD2772_ERR_I2C;
				}
				databuf[0] = TMD2772_CMM_INT_LOW_THD_HIGH;	
				databuf[1] = (u8)((ps_cali.far_away & 0xFF00) >> 8);
				res = i2c_master_send(client, databuf, 0x2);
				if(res <= 0)
				{
					goto EXIT_ERR;
					return TMD2772_ERR_I2C;
				}
				databuf[0] = TMD2772_CMM_INT_HIGH_THD_LOW;	
				databuf[1] = (u8)(ps_cali.close & 0x00FF);
				res = i2c_master_send(client, databuf, 0x2);
				if(res <= 0)
				{
					goto EXIT_ERR;
					return TMD2772_ERR_I2C;
				}
				databuf[0] = TMD2772_CMM_INT_HIGH_THD_HIGH; 
				databuf[1] = (u8)((ps_cali.close & 0xFF00) >> 8);;
				res = i2c_master_send(client, databuf, 0x2);
				if(res <= 0)
				{
					goto EXIT_ERR;
					return TMD2772_ERR_I2C;
				}
			}
			#if 0
			else
			{
				databuf[0] = TMD2772_CMM_INT_LOW_THD_LOW;	
				databuf[1] = (u8)(750 & 0x00FF);
				res = i2c_master_send(client, databuf, 0x2);
				if(res <= 0)
				{
					goto EXIT_ERR;
					return TMD2772_ERR_I2C;
				}
				databuf[0] = TMD2772_CMM_INT_LOW_THD_HIGH;	
				databuf[1] = (u8)((750 & 0xFF00) >> 8);
				res = i2c_master_send(client, databuf, 0x2);
				if(res <= 0)
				{
					goto EXIT_ERR;
					return TMD2772_ERR_I2C;
				}
				databuf[0] = TMD2772_CMM_INT_HIGH_THD_LOW;	
				databuf[1] = (u8)(900 & 0x00FF);
				res = i2c_master_send(client, databuf, 0x2);
				if(res <= 0)
				{
					goto EXIT_ERR;
					return TMD2772_ERR_I2C;
				}
				databuf[0] = TMD2772_CMM_INT_HIGH_THD_HIGH; 
				databuf[1] = (u8)((900 & 0xFF00) >> 8);;
				res = i2c_master_send(client, databuf, 0x2);
				if(res <= 0)
				{
					goto EXIT_ERR;
					return TMD2772_ERR_I2C;
				}
		
			}
			#endif
		
			databuf[0] = TMD2772_CMM_Persistence;
			databuf[1] = 0x20;//0x50; //modify by zy PR350462
			res = i2c_master_send(client, databuf, 0x2);
			if(res <= 0)
			{
				goto EXIT_ERR;
				return TMD2772_ERR_I2C;
			}
			if(testbit_ALS){
			databuf[0] = TMD2772_CMM_ENABLE;    
			databuf[1] = 0x2F;
			res = i2c_master_send(client, databuf, 0x2);
			if(res <= 0)
				{
					goto EXIT_ERR;
				}
			/*debug code for reading register value*/
			#if 0
			res = i2c_master_recv(client, reg_value, 0x1);
			if(res <= 0)
				{
					goto EXIT_ERR;
				}
			printk("Yucong:0x%x, %d, %s\n", reg_value[0], __LINE__, __FUNCTION__);
			#endif
			}else{
			databuf[0] = TMD2772_CMM_ENABLE;    
			databuf[1] = 0x2D;
			res = i2c_master_send(client, databuf, 0x2);
			if(res <= 0)
				{
					goto EXIT_ERR;
				}
			}
			/*debug code for reading register value*/
			#if 0
			res = i2c_master_recv(client, reg_value, 0x1);
			if(res <= 0)
				{
					goto EXIT_ERR;
				}
			printk("Yucong:0x%x, %d, %s\n", reg_value[0], __LINE__, __FUNCTION__);
			#endif
		
			mt65xx_eint_unmask(CUST_EINT_ALS_NUM);
		}
	}
	else
	{
	/*yucong MTK: enable_ps function modified for fixing reading register error problem 2012.2.16*/
	if(testbit_ALS){
		databuf[0] = TMD2772_CMM_ENABLE;    
		databuf[1] = 0x2B;
		res = i2c_master_send(client, databuf, 0x2);
		if(res <= 0)
			{
				goto EXIT_ERR;
			}
		}else{
		databuf[0] = TMD2772_CMM_ENABLE;    
		databuf[1] = 0x00;
		res = i2c_master_send(client, databuf, 0x2);
		if(res <= 0)
			{
				goto EXIT_ERR;
			}
		}
		atomic_set(&obj->ps_deb_on, 0);
		APS_DBG("tmd2772 power off\n");

		/*for interrup work mode support -- by liaoxl.lenovo 12.08.2011*/
		if(0 == obj->hw->polling_mode_ps)
		{
			cancel_work_sync(&obj->eint_work);
			mt65xx_eint_mask(CUST_EINT_ALS_NUM);
		}
	}
#endif
	return 0;
	
EXIT_ERR:
	APS_ERR("tmd2772_enable_ps fail\n");
	return res;
}
/*----------------------------------------------------------------------------*/
static int tmd2772_enable(struct i2c_client *client, int enable)
{
	struct tmd2772_priv *obj = i2c_get_clientdata(client);
	u8 databuf[2];    
	int res = 0;
	u8 buffer[1];
	u8 reg_value[1];

	if(client == NULL)
	{
		APS_DBG("CLIENT CANN'T EQUL NULL\n");
		return -1;
	}

	/* modify to restore reg setting after cali ---liaoxl.lenovo */
	buffer[0]= TMD2772_CMM_ENABLE;
	res = i2c_master_send(client, buffer, 0x1);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	res = i2c_master_recv(client, reg_value, 0x1);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}

	if(enable)
	{
		databuf[0] = TMD2772_CMM_ENABLE;    
		databuf[1] = reg_value[0] | 0x01;
		res = i2c_master_send(client, databuf, 0x2);
		if(res <= 0)
		{
			goto EXIT_ERR;
		}
		APS_DBG("tmd2772 power on\n");
	}
	else
	{
		databuf[0] = TMD2772_CMM_ENABLE;    
		databuf[1] = reg_value[0] & 0xFE;
		res = i2c_master_send(client, databuf, 0x2);
		if(res <= 0)
		{
			goto EXIT_ERR;
		}
		atomic_set(&obj->ps_deb_on, 0);
		/*Lenovo-sw chenlj2 add 2011-06-03,close als_deb_on */
		atomic_set(&obj->als_deb_on, 0);
		APS_DBG("tmd2772 power off\n");
	}
	return 0;
	
EXIT_ERR:
	APS_ERR("tmd2772_enable fail\n");
	return res;
}

/*----------------------------------------------------------------------------*/
/*for interrup work mode support -- by liaoxl.lenovo 12.08.2011*/
static int tmd2772_check_and_clear_intr(struct i2c_client *client) 
{
	struct tmd2772_priv *obj = i2c_get_clientdata(client);
	int res,intp,intl;
	u8 buffer[2];

	//if (mt_get_gpio_in(GPIO_ALS_EINT_PIN) == 1) /*skip if no interrupt*/  
	//    return 0;

	buffer[0] = TMD2772_CMM_STATUS;
	res = i2c_master_send(client, buffer, 0x1);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	res = i2c_master_recv(client, buffer, 0x1);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	//printk("yucong TMD2772_check_and_clear_intr status=0x%x\n", buffer[0]);
	res = 1;
	intp = 0;
	intl = 0;
	if(0 != (buffer[0] & 0x20))
	{
		res = 0;
		intp = 1;
	}
	if(0 != (buffer[0] & 0x10))
	{
		res = 0;
		intl = 1;		
	}

	if(0 == res)
	{
		if((1 == intp) && (0 == intl))
		{
			buffer[0] = (TAOS_TRITON_CMD_REG|TAOS_TRITON_CMD_SPL_FN|0x05);
		}
		else if((0 == intp) && (1 == intl))
		{
			buffer[0] = (TAOS_TRITON_CMD_REG|TAOS_TRITON_CMD_SPL_FN|0x06);
		}
		else
		{
			buffer[0] = (TAOS_TRITON_CMD_REG|TAOS_TRITON_CMD_SPL_FN|0x07);
		}
		res = i2c_master_send(client, buffer, 0x1);
		if(res <= 0)
		{
			goto EXIT_ERR;
		}
		else
		{
			res = 0;
		}
	}

	return res;

EXIT_ERR:
	APS_ERR("tmd2772_check_and_clear_intr fail\n");
	return 1;
}
/*----------------------------------------------------------------------------*/

/*yucong add for interrupt mode support MTK inc 2012.3.7*/
static int tmd2772_check_intr(struct i2c_client *client) 
{
	struct tmd2772_priv *obj = i2c_get_clientdata(client);
	int res,intp,intl;
	u8 buffer[2];

	//if (mt_get_gpio_in(GPIO_ALS_EINT_PIN) == 1) /*skip if no interrupt*/  
	//    return 0;

	buffer[0] = TMD2772_CMM_STATUS;
	res = i2c_master_send(client, buffer, 0x1);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	res = i2c_master_recv(client, buffer, 0x1);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	//APS_ERR("tmd2772_check_and_clear_intr status=0x%x\n", buffer[0]);
	res = 1;
	intp = 0;
	intl = 0;
	if(0 != (buffer[0] & 0x20))
	{
		res = 0;
		intp = 1;
	}
	if(0 != (buffer[0] & 0x10))
	{
		res = 0;
		intl = 1;		
	}

	return res;

EXIT_ERR:
	APS_ERR("tmd2772_check_intr fail\n");
	return 1;
}

static int tmd2772_clear_intr(struct i2c_client *client) 
{
	struct tmd2772_priv *obj = i2c_get_clientdata(client);
	int res;
	u8 buffer[2];

#if 0
	if((1 == intp) && (0 == intl))
	{
		buffer[0] = (TAOS_TRITON_CMD_REG|TAOS_TRITON_CMD_SPL_FN|0x05);
	}
	else if((0 == intp) && (1 == intl))
	{
		buffer[0] = (TAOS_TRITON_CMD_REG|TAOS_TRITON_CMD_SPL_FN|0x06);
	}
	else
#endif
	{
		buffer[0] = (TAOS_TRITON_CMD_REG|TAOS_TRITON_CMD_SPL_FN|0x07);
	}
	res = i2c_master_send(client, buffer, 0x1);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	else
	{
		res = 0;
	}

	return res;

EXIT_ERR:
	APS_ERR("tmd2772_check_and_clear_intr fail\n");
	return 1;
}


/*-----------------------------------------------------------------------------*/
void tmd2772_eint_func(void)
{
	struct tmd2772_priv *obj = g_tmd2772_ptr;
	if(!obj)
	{
		return;
	}
	
	schedule_work(&obj->eint_work);
}

/*----------------------------------------------------------------------------*/
/*for interrup work mode support -- by liaoxl.lenovo 12.08.2011*/
int tmd2772_setup_eint(struct i2c_client *client)
{
	struct tmd2772_priv *obj = i2c_get_clientdata(client);        

	g_tmd2772_ptr = obj;
	
	mt_set_gpio_mode(GPIO_ALS_EINT_PIN, EINT2_PS_N_M_EINT);
	mt_set_gpio_dir(GPIO_ALS_EINT_PIN, GPIO_DIR_IN);
	mt_set_gpio_pull_enable(GPIO_ALS_EINT_PIN, TRUE);
	mt_set_gpio_pull_select(GPIO_ALS_EINT_PIN, GPIO_PULL_UP);

	mt65xx_eint_set_sens(CUST_EINT_ALS_NUM, CUST_EINT_ALS_SENSITIVE);
	mt65xx_eint_set_polarity(CUST_EINT_ALS_NUM, CUST_EINT_ALS_POLARITY);
	mt65xx_eint_set_hw_debounce(CUST_EINT_ALS_NUM, CUST_EINT_ALS_DEBOUNCE_CN);
	mt65xx_eint_registration(CUST_EINT_ALS_NUM, CUST_EINT_ALS_DEBOUNCE_EN, CUST_EINT_ALS_POLARITY, tmd2772_eint_func, 0);

	mt65xx_eint_unmask(CUST_EINT_ALS_NUM);  
    return 0;
}

/*----------------------------------------------------------------------------*/

static int tmd2772_init_client_for_cali(struct i2c_client *client)
{

	struct tmd2772_priv *obj = i2c_get_clientdata(client);
	u8 databuf[2];    
	int res = 0;
#if 0
	u8 id = 0;
   //---------read id:

	databuf[0]=0x92;
	res = i2c_master_send(client, databuf, 0x1);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	res = i2c_master_recv(client, &id, 0x1);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
       APS_DBG("***id = %d\n", id);
#endif
   
	databuf[0] = TMD2772_CMM_ENABLE;    
	databuf[1] = 0x01;
	res = i2c_master_send(client, databuf, 0x2);
	if(res <= 0)
	{
		goto EXIT_ERR;
		return TMD2772_ERR_I2C;
	}
	
	databuf[0] = TMD2772_CMM_ATIME;    
	databuf[1] = 0xEE;//0xEE
	res = i2c_master_send(client, databuf, 0x2);
	if(res <= 0)
	{
		goto EXIT_ERR;
		return TMD2772_ERR_I2C;
	}

	databuf[0] = TMD2772_CMM_PTIME;    
	databuf[1] = 0xFF;
	res = i2c_master_send(client, databuf, 0x2);
	if(res <= 0)
	{
		goto EXIT_ERR;
		return TMD2772_ERR_I2C;
	}

	databuf[0] = TMD2772_CMM_WTIME;    
	databuf[1] = 0xFF;//0xFF
	res = i2c_master_send(client, databuf, 0x2);
	if(res <= 0)
	{
		goto EXIT_ERR;
		return TMD2772_ERR_I2C;
	}

#if MIN_CURRENT_MODE

	databuf[0] = TMD2772_CMM_CONFIG;    
	databuf[1] = 0x01;
	res = i2c_master_send(client, databuf, 0x2);
	if(res <= 0)
	{
		goto EXIT_ERR;
		return TMD2772_ERR_I2C;
	}	
#else

	databuf[0] = TMD2772_CMM_CONFIG;    
	databuf[1] = 0x00;
	res = i2c_master_send(client, databuf, 0x2);
	if(res <= 0)
	{
		goto EXIT_ERR;
		return TMD2772_ERR_I2C;
	}	
#endif

	databuf[0] = TMD2772_CMM_PPCOUNT;    
	databuf[1] = PPCOUNT_VALUE;
	res = i2c_master_send(client, databuf, 0x2);
	if(res <= 0)
	{
		goto EXIT_ERR;
		return TMD2772_ERR_I2C;
	}

#if MIN_CURRENT_MODE

	databuf[0] = TMD2772_CMM_CONTROL;    
	databuf[1] = TMD2772_CMM_CONTROL_VALUE;//;0x20;//0xf0;//modify to 100mA, PR356949
	res = i2c_master_send(client, databuf, 0x2);
	if(res <= 0)
	{
		goto EXIT_ERR;
		return TMD2772_ERR_I2C;
	}	
#else

	databuf[0] = TMD2772_CMM_CONTROL;    
	databuf[1] = TMD2772_CMM_CONTROL_VALUE;//;0x20;//0xE0;//modify to 100mA, PR356949
	res = i2c_master_send(client, databuf, 0x2);
	if(res <= 0)
	{
		goto EXIT_ERR;
		return TMD2772_ERR_I2C;
	}
#endif

	databuf[0] = TMD2772_CMM_ENABLE;	
	databuf[1] = 0x05;
	res = i2c_master_send(client, databuf, 0x2);
	if(res <= 0)
	{
		goto EXIT_ERR;
		return TMD2772_ERR_I2C;
	}

	return TMD2772_SUCCESS;

EXIT_ERR:
	APS_ERR("init dev: %d\n", res);
	return res;

}

static int tmd2772_init_client(struct i2c_client *client)
{
	struct tmd2772_priv *obj = i2c_get_clientdata(client);
	u8 databuf[2];    
	int res = 0;
   
	databuf[0] = TMD2772_CMM_ENABLE;    
	databuf[1] = 0x00;
	res = i2c_master_send(client, databuf, 0x2);
	if(res <= 0)
	{
		goto EXIT_ERR;
		return TMD2772_ERR_I2C;
	}
	
	databuf[0] = TMD2772_CMM_ATIME;    
	databuf[1] = 0xEE;
	res = i2c_master_send(client, databuf, 0x2);
	if(res <= 0)
	{
		goto EXIT_ERR;
		return TMD2772_ERR_I2C;
	}

	databuf[0] = TMD2772_CMM_PTIME;    
	databuf[1] = 0xFF;
	res = i2c_master_send(client, databuf, 0x2);
	if(res <= 0)
	{
		goto EXIT_ERR;
		return TMD2772_ERR_I2C;
	}

	databuf[0] = TMD2772_CMM_WTIME;    
	databuf[1] = 0xEE;
	res = i2c_master_send(client, databuf, 0x2);
	if(res <= 0)
	{
		goto EXIT_ERR;
		return TMD2772_ERR_I2C;
	}
	/*for interrup work mode support -- by liaoxl.lenovo 12.08.2011*/
	if(0 == obj->hw->polling_mode_ps)
	{
		//if(1 == ps_cali.valid)
		{
			databuf[0] = TMD2772_CMM_INT_LOW_THD_LOW;	
			databuf[1] = (u8)(ps_cali.far_away & 0x00FF);
			res = i2c_master_send(client, databuf, 0x2);
			if(res <= 0)
			{
				goto EXIT_ERR;
				return TMD2772_ERR_I2C;
			}
			databuf[0] = TMD2772_CMM_INT_LOW_THD_HIGH;	
			databuf[1] = (u8)((ps_cali.far_away & 0xFF00) >> 8);
			res = i2c_master_send(client, databuf, 0x2);
			if(res <= 0)
			{
				goto EXIT_ERR;
				return TMD2772_ERR_I2C;
			}
			databuf[0] = TMD2772_CMM_INT_HIGH_THD_LOW;	
			databuf[1] = (u8)(ps_cali.close & 0x00FF);
			res = i2c_master_send(client, databuf, 0x2);
			if(res <= 0)
			{
				goto EXIT_ERR;
				return TMD2772_ERR_I2C;
			}
			databuf[0] = TMD2772_CMM_INT_HIGH_THD_HIGH;	
			databuf[1] = (u8)((ps_cali.close & 0xFF00) >> 8);;
			res = i2c_master_send(client, databuf, 0x2);
			if(res <= 0)
			{
				goto EXIT_ERR;
				return TMD2772_ERR_I2C;
			}
		}
		#if 0
		else
		{
			databuf[0] = TMD2772_CMM_INT_LOW_THD_LOW;	
			databuf[1] = (u8)(750 & 0x00FF);
			res = i2c_master_send(client, databuf, 0x2);
			if(res <= 0)
			{
				goto EXIT_ERR;
				return TMD2772_ERR_I2C;
			}
			databuf[0] = TMD2772_CMM_INT_LOW_THD_HIGH;	
			databuf[1] = (u8)((750 & 0xFF00) >> 8);
			res = i2c_master_send(client, databuf, 0x2);
			if(res <= 0)
			{
				goto EXIT_ERR;
				return TMD2772_ERR_I2C;
			}
			databuf[0] = TMD2772_CMM_INT_HIGH_THD_LOW;	
			databuf[1] = (u8)(900 & 0x00FF);
			res = i2c_master_send(client, databuf, 0x2);
			if(res <= 0)
			{
				goto EXIT_ERR;
				return TMD2772_ERR_I2C;
			}
			databuf[0] = TMD2772_CMM_INT_HIGH_THD_HIGH;	
			databuf[1] = (u8)((900 & 0xFF00) >> 8);;
			res = i2c_master_send(client, databuf, 0x2);
			if(res <= 0)
			{
				goto EXIT_ERR;
				return TMD2772_ERR_I2C;
			}

		}
		#endif
#if 0
		databuf[0] = TMD2772_CMM_Persistence;
		databuf[1] = 0x20;//0x50;//modify by zy PR350462
		res = i2c_master_send(client, databuf, 0x2);
		if(res <= 0)
		{
			goto EXIT_ERR;
			return TMD2772_ERR_I2C;
		}
		databuf[0] = TMD2772_CMM_ENABLE;	
		databuf[1] = 0x20;
		res = i2c_master_send(client, databuf, 0x2);
		if(res <= 0)
		{
			goto EXIT_ERR;
			return TMD2772_ERR_I2C;
		}
#endif
	}
	
		databuf[0] = TMD2772_CMM_Persistence;
		databuf[1] = 0x20;//0x50; //modify by zy PR350462
		res = i2c_master_send(client, databuf, 0x2);
		if(res <= 0)
		{
			goto EXIT_ERR;
			return TMD2772_ERR_I2C;
		}
		databuf[0] = TMD2772_CMM_ENABLE;	
		databuf[1] = 0x20;
		res = i2c_master_send(client, databuf, 0x2);
		if(res <= 0)
		{
			goto EXIT_ERR;
			return TMD2772_ERR_I2C;
		}

#if MIN_CURRENT_MODE
	databuf[0] = TMD2772_CMM_CONFIG;    
	databuf[1] = 0x01;
	res = i2c_master_send(client, databuf, 0x2);
	if(res <= 0)
	{
		goto EXIT_ERR;
		return TMD2772_ERR_I2C;
	}
#else
	databuf[0] = TMD2772_CMM_CONFIG;    
	databuf[1] = 0x00;
	res = i2c_master_send(client, databuf, 0x2);
	if(res <= 0)
	{
		goto EXIT_ERR;
		return TMD2772_ERR_I2C;
	}
#endif

       /*Lenovo-sw chenlj2 add 2011-06-03,modified pulse 2  to 4 */
	databuf[0] = TMD2772_CMM_PPCOUNT;    
	databuf[1] = PPCOUNT_VALUE;
	res = i2c_master_send(client, databuf, 0x2);
	if(res <= 0)
	{
		goto EXIT_ERR;
		return TMD2772_ERR_I2C;
	}

#if MIN_CURRENT_MODE
        /*Lenovo-sw chenlj2 add 2011-06-03,modified gain 16  to 1 */
	databuf[0] = TMD2772_CMM_CONTROL;    
	databuf[1] = TMD2772_CMM_CONTROL_VALUE;//0x20;//0xf0;//modify to 100mA, PR356949
	res = i2c_master_send(client, databuf, 0x2);
	if(res <= 0)
	{
		goto EXIT_ERR;
		return TMD2772_ERR_I2C;
	}
#else
	databuf[0] = TMD2772_CMM_CONTROL;    
	databuf[1] = TMD2772_CMM_CONTROL_VALUE;//0x20;//0xE0;//modify to 100mA, PR356949
	res = i2c_master_send(client, databuf, 0x2);
	if(res <= 0)
	{
		goto EXIT_ERR;
		return TMD2772_ERR_I2C;
	}
#endif

	/*for interrup work mode support -- by liaoxl.lenovo 12.08.2011*/
	if(res = tmd2772_setup_eint(client))
	{
		APS_ERR("setup eint: %d\n", res);
		return res;
	}
	if(res = tmd2772_check_and_clear_intr(client))
	{
		APS_ERR("check/clear intr: %d\n", res);
		//    return res;
	}
	
	return TMD2772_SUCCESS;

EXIT_ERR:
	APS_ERR("init dev: %d\n", res);
	return res;
}

int tmd2772_read_als(struct i2c_client *client, u16 *data)
{
	struct tmd2772_priv *obj = i2c_get_clientdata(client);	 
	u16 c0_value, c1_value;	 
	u32 c0_nf, c1_nf;
	u8 als_value_low[1], als_value_high[1];
	u8 buffer[1];
	u16 atio;
	u16 als_value;
	int res = 0;
	u8 reg_value[1];
	
	if(client == NULL)
	{
		APS_DBG("CLIENT CANN'T EQUL NULL\n");
		return -1;
	}

	/*debug tag for yucong*/
	#if 0
	buffer[0]= TMD2772_CMM_ENABLE;
	res = i2c_master_send(client, buffer, 0x1);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	res = i2c_master_recv(client, reg_value, 0x1);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	printk("Yucong:0x%x, %d, %s\n", reg_value[0], __LINE__, __FUNCTION__);
	#endif
//get adc channel 0 value
	buffer[0]= TMD2772_CMM_C0DATA_L;
	res = i2c_master_send(client, buffer, 0x1);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	res = i2c_master_recv(client, als_value_low, 0x1);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	//printk("yucong: TMD2772_CMM_C0DATA_L = 0x%x\n", als_value_low[0]);

	buffer[0]= TMD2772_CMM_C0DATA_H;
	res = i2c_master_send(client, buffer, 0x1);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	res = i2c_master_recv(client, als_value_high, 0x01);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	//printk("yucong: TMD2772_CMM_C0DATA_H = 0x%x\n", als_value_high[0]);
	c0_value = als_value_low[0] | (als_value_high[0]<<8);
	c0_nf = obj->als_modulus*c0_value;
	//APS_DBG("c0_value=%d, c0_nf=%d, als_modulus=%d\n", c0_value, c0_nf, obj->als_modulus);

//get adc channel 1 value
	buffer[0]= TMD2772_CMM_C1DATA_L;
	res = i2c_master_send(client, buffer, 0x1);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	res = i2c_master_recv(client, als_value_low, 0x1);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	//printk("yucong: TMD2772_CMM_C1DATA_L = 0x%x\n", als_value_low[0]);	

	buffer[0]= TMD2772_CMM_C1DATA_H;
	res = i2c_master_send(client, buffer, 0x1);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	res = i2c_master_recv(client, als_value_high, 0x01);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	//printk("yucong: TMD2772_CMM_C1DATA_H = 0x%x\n", als_value_high[0]);	

	c1_value = als_value_low[0] | (als_value_high[0]<<8);
	c1_nf = obj->als_modulus*c1_value;	
	//APS_DBG("c1_value=%d, c1_nf=%d, als_modulus=%d\n", c1_value, c1_nf, obj->als_modulus);

	if((c0_value > c1_value) &&(c0_value < 50000))
	{  	/*Lenovo-sw chenlj2 add 2011-06-03,add {*/
		atio = (c1_nf*100)/c0_nf;

	//APS_DBG("atio = %d\n", atio);
	if(atio<30)
	{
		*data = (13*c0_nf - 24*c1_nf)/10000;
	}
	else if(atio>= 30 && atio<38) /*Lenovo-sw chenlj2 add 2011-06-03,modify > to >=*/
	{ 
		*data = (16*c0_nf - 35*c1_nf)/10000;
	}
	else if(atio>= 38 && atio<45)  /*Lenovo-sw chenlj2 add 2011-06-03,modify > to >=*/
	{ 
		*data = (9*c0_nf - 17*c1_nf)/10000;
	}
	else if(atio>= 45 && atio<54) /*Lenovo-sw chenlj2 add 2011-06-03,modify > to >=*/
	{ 
		*data = (6*c0_nf - 10*c1_nf)/10000;
	}
	else
		*data = 0;
	/*Lenovo-sw chenlj2 add 2011-06-03,add }*/
    }
	else if (c0_value > 50000)
	{
		*data = 65535;
	}
	else if(c0_value == 0)
        {
                *data = 0;
        }
        else
	{
		APS_DBG("als_value is invalid!!\n");
		return -1;
	}	
	APS_DBG("als_value_lux = %d\n", *data);
	//printk("yucong: als_value_lux = %d\n", *data);
	return 0;	 

	
	
EXIT_ERR:
	APS_ERR("tmd2772_read_ps fail\n");
	return res;
}
int tmd2772_read_als_ch0(struct i2c_client *client, u16 *data)
{
	struct tmd2772_priv *obj = i2c_get_clientdata(client);	 
	u16 c0_value;	 
	u8 als_value_low[1], als_value_high[1];
	u8 buffer[1];
	int res = 0;
	
	if(client == NULL)
	{
		APS_DBG("CLIENT CANN'T EQUL NULL\n");
		return -1;
	}
//get adc channel 0 value
	buffer[0]= TMD2772_CMM_C0DATA_L;
	res = i2c_master_send(client, buffer, 0x1);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	res = i2c_master_recv(client, als_value_low, 0x1);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	
	buffer[0]= TMD2772_CMM_C0DATA_H;
	res = i2c_master_send(client, buffer, 0x1);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	res = i2c_master_recv(client, als_value_high, 0x01);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	
	c0_value = als_value_low[0] | (als_value_high[0]<<8);
	*data = c0_value;
	return 0;	 

	
	
EXIT_ERR:
	APS_ERR("tmd2772_read_ps fail\n");
	return res;
}
/*----------------------------------------------------------------------------*/

static int tmd2772_get_als_value(struct tmd2772_priv *obj, u16 als)
{
	int idx;
	int invalid = 0;
	for(idx = 0; idx < obj->als_level_num; idx++)
	{
		if(als < obj->hw->als_level[idx])
		{
			break;
		}
	}
	
	if(idx >= obj->als_value_num)
	{
		APS_ERR("exceed range\n"); 
		idx = obj->als_value_num - 1;
	}
	
	if(1 == atomic_read(&obj->als_deb_on))
	{
		unsigned long endt = atomic_read(&obj->als_deb_end);
		if(time_after(jiffies, endt))
		{
			atomic_set(&obj->als_deb_on, 0);
		}
		
		if(1 == atomic_read(&obj->als_deb_on))
		{
			invalid = 1;
		}
	}

	if(!invalid)
	{
		//APS_DBG("ALS: %05d => %05d\n", als, obj->hw->als_value[idx]);	
		return obj->hw->als_value[idx];
	}
	else
	{
		//APS_ERR("ALS: %05d => %05d (-1)\n", als, obj->hw->als_value[idx]);    
		return -1;
	}
}
/*----------------------------------------------------------------------------*/
long tmd2772_read_ps_fx(struct i2c_client *client, u16 *data)
{
	struct tmd2772_priv *obj = i2c_get_clientdata(client);    
	u16 ps_value;    
	u8 ps_value_low[1], ps_value_high[1];
	u8 buffer[1];
	long res = 0;

	if(client == NULL)
	{
		APS_DBG("CLIENT CANN'T EQUL NULL\n");
		return -1;
	}
	printk(KERN_ALERT "The func is %s\n",__func__);
	buffer[0]= TMD2772_CMM_PDATA_L;
	res = tmd2772_i2c_read_block(client, buffer[0], ps_value_low, 0x01);
	if(res != 0)
	{
		goto EXIT_ERR;
	}
	buffer[0]= TMD2772_CMM_PDATA_H;
	res = tmd2772_i2c_read_block(client, buffer[0], ps_value_high, 0x01);
	if(res != 0)
	{
		goto EXIT_ERR;
	}

	*data = ps_value_low[0] | (ps_value_high[0]<<8);
	printk("ps_data=%d, low:%d  high:%d", *data, ps_value_low[0], ps_value_high[0]);
	return 0;    

EXIT_ERR:
	APS_ERR("tmd2772_read_ps fail\n");
	return res;
}
long tmd2772_read_ps(struct i2c_client *client, u16 *data)
{
	struct tmd2772_priv *obj = i2c_get_clientdata(client);    
	u16 ps_value;    
	u8 ps_value_low[1], ps_value_high[1];
	u8 buffer[1];
	long res = 0;

	if(client == NULL)
	{
		APS_DBG("CLIENT CANN'T EQUL NULL\n");
		return -1;
	}

	buffer[0]= TMD2772_CMM_PDATA_L;
	res = i2c_master_send(client, buffer, 0x1);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	res = i2c_master_recv(client, ps_value_low, 0x1);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}

	buffer[0]= TMD2772_CMM_PDATA_H;
	res = i2c_master_send(client, buffer, 0x1);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	res = i2c_master_recv(client, ps_value_high, 0x01);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}

	*data = ps_value_low[0] | (ps_value_high[0]<<8);
	printk("ps_data=%d, low:%d  high:%d", *data, ps_value_low[0], ps_value_high[0]);
	return 0;    

EXIT_ERR:
	APS_ERR("tmd2772_read_ps fail\n");
	return res;
}
/*----------------------------------------------------------------------------*/
static int tmd2772_get_ps_value(struct tmd2772_priv *obj, u16 ps)
{
	int val, mask = atomic_read(&obj->ps_mask);
	int invalid = 0;
	static int val_temp=1;
	 /*Lenovo-sw chenlj2 add 2011-10-12 begin*/
	 u16 temp_ps[1];
	 /*Lenovo-sw chenlj2 add 2011-10-12 end*/
	 
	
	//APS_LOG("tmd2772_get_ps_value  1 %d," ,ps_cali.close);
	//APS_LOG("tmd2772_get_ps_value  2 %d," ,ps_cali.far_away);
	//APS_LOG("tmd2772_get_ps_value  3 %d,", ps_cali.valid);

	//APS_LOG("tmd2772_get_ps_value  ps %d,", ps);
    /*Lenovo-sw zhuhc delete 2011-10-12 begin*/
	//return 1;
    /*Lenovo-sw zhuhc delete 2011-10-12 end*/

//        mdelay(160);
	tmd2772_read_ps_fx(obj->client,temp_ps);
	APS_LOG("c=%d, f=%d, p1=%d, p2=%d, v=%d", ps_cali.close, ps_cali.far_away, ps, temp_ps[0], ps_cali.valid);

/*yi.zheng.hz modify for debug p sensor calibration at 2013-06-27 start*/
#ifdef DEBUG_TMD2772_CALIBRATION
	ps_latest[0] = ps;
	ps_latest[1] = temp_ps[0];
#endif
/*yi.zheng.hz modify for debug p sensor calibration at 2013-06-27 end*/

//	if(ps_cali.valid == 1)
	{
			//APS_LOG("tmd2772_get_ps_value val_temp  = %d",val_temp);
			if((ps >ps_cali.close)&&(temp_ps[0] >ps_cali.close))
			{
				val = 0;  /*close*/
				val_temp = 0;
				intr_flag_value = 1;
			}
			else if((ps <ps_cali.far_away)&&(temp_ps[0] < ps_cali.far_away))
			{
				val = 1;  /*far away*/
				val_temp = 1;
				intr_flag_value = 0;
			}
			else
				val = val_temp;

			//APS_LOG("tmd2772_get_ps_value val  = %d",val);
	}
#if 0
	else
	{
			if((ps > atomic_read(&obj->ps_thd_val_high))&&(temp_ps[0]  > atomic_read(&obj->ps_thd_val_high)))
			{
				val = 0;  /*close*/
				val_temp = 0;
				intr_flag_value = 1;
			}
			else if((ps < atomic_read(&obj->ps_thd_val_low))&&(temp_ps[0]  < atomic_read(&obj->ps_thd_val_low)))
			{
				val = 1;  /*far away*/
				val_temp = 1;
				intr_flag_value = 0;
			}
			else
			       val = val_temp;	
			
	}
#endif
	
	if(atomic_read(&obj->ps_suspend))
	{
		invalid = 1;
	}
	else if(1 == atomic_read(&obj->ps_deb_on))
	{
		unsigned long endt = atomic_read(&obj->ps_deb_end);
		if(time_after(jiffies, endt))
		{
			atomic_set(&obj->ps_deb_on, 0);
		}
		
		if (1 == atomic_read(&obj->ps_deb_on))
		{
			invalid = 1;
		}
	}
	else if (obj->als > 45000)
	{
		//invalid = 1;
		APS_DBG("ligh too high will result to failt proximiy\n");
		return 1;  /*far away*/
	}

	if(!invalid)
	{
		//APS_DBG("PS:  %05d => %05d\n", ps, val);
		return val;
	}	
	else
	{
		return -1;
	}	
}


/*----------------------------------------------------------------------------*/
/*for interrup work mode support -- by liaoxl.lenovo 12.08.2011*/
static void tmd2772_eint_work(struct work_struct *work)
{
	struct tmd2772_priv *obj = (struct tmd2772_priv *)container_of(work, struct tmd2772_priv, eint_work);
	int err;
	hwm_sensor_data sensor_data;
	u8 buffer[1];
	u8 reg_value[1];
	u8 databuf[2];
	int res = 0;

	if((err = tmd2772_check_intr(obj->client)))
	{
		APS_ERR("tmd2772_eint_work check intrs: %d\n", err);
	}
	else
	{
		//get raw data
		tmd2772_read_ps_fx(obj->client, &obj->ps);
		//mdelay(160);
		tmd2772_read_als_ch0(obj->client, &obj->als);
		APS_DBG("tmd2772_eint_work close=%d far=%d rawdata ps=%d als_ch0=%d!\n",ps_cali.close, ps_cali.far_away, obj->ps,obj->als);
		//printk("tmd2772_eint_work rawdata ps=%d als_ch0=%d!\n",obj->ps,obj->als);
		sensor_data.values[0] = tmd2772_get_ps_value(obj, obj->ps);
		sensor_data.value_divide = 1;
		sensor_data.status = SENSOR_STATUS_ACCURACY_MEDIUM;	
		//APS_LOG("tmd2772_eint_work close  = %d,far_away = %d,valid = %d",ps_cali.close,ps_cali.far_away,ps_cali.valid);
/*singal interrupt function add*/
#if 1
		if(intr_flag_value){
				//printk("yucong interrupt value ps will < 750");
				databuf[0] = TMD2772_CMM_INT_LOW_THD_LOW;	
				//databuf[1] = (u8)((atomic_read(&obj->ps_thd_val_low)) & 0x00FF);
				databuf[1] = ps_cali.far_away & 0x00FF; //modify for PR350462
				res = i2c_master_send(obj->client, databuf, 0x2);
				if(res <= 0)
				{
					return;
				}
				databuf[0] = TMD2772_CMM_INT_LOW_THD_HIGH;	
				//databuf[1] = (u8)(((atomic_read(&obj->ps_thd_val_low)) & 0xFF00) >> 8);
				databuf[1] = (ps_cali.far_away & 0xFF00)>>8;  //modify for PR350462
				res = i2c_master_send(obj->client, databuf, 0x2);
				if(res <= 0)
				{
					return;
				}
				databuf[0] = TMD2772_CMM_INT_HIGH_THD_LOW;	
				databuf[1] = (u8)(0x00FF);
				res = i2c_master_send(obj->client, databuf, 0x2);
				if(res <= 0)
				{
					return;
				}
				databuf[0] = TMD2772_CMM_INT_HIGH_THD_HIGH; 
				databuf[1] = (u8)((0xFF00) >> 8);;
				res = i2c_master_send(obj->client, databuf, 0x2);
				if(res <= 0)
				{
					return;
				}
		}
		else{	
				//printk("yucong interrupt value ps will > 900");
				databuf[0] = TMD2772_CMM_INT_LOW_THD_LOW;	
				databuf[1] = (u8)(0 & 0x00FF);
				res = i2c_master_send(obj->client, databuf, 0x2);
				if(res <= 0)
				{
					return;
				}
				databuf[0] = TMD2772_CMM_INT_LOW_THD_HIGH;	
				databuf[1] = (u8)((0 & 0xFF00) >> 8);
				res = i2c_master_send(obj->client, databuf, 0x2);
				if(res <= 0)
				{
					return;
				}
				databuf[0] = TMD2772_CMM_INT_HIGH_THD_LOW;	
				//databuf[1] = (u8)((atomic_read(&obj->ps_thd_val_high)) & 0x00FF);
				databuf[1] = ps_cali.close & 0x00FF;//modify for PR350462
				res = i2c_master_send(obj->client, databuf, 0x2);
				if(res <= 0)
				{
					return;
				}
				databuf[0] = TMD2772_CMM_INT_HIGH_THD_HIGH; 
				//databuf[1] = (u8)(((atomic_read(&obj->ps_thd_val_high)) & 0xFF00) >> 8);
				databuf[1] = (ps_cali.close & 0xFF00) >> 8; //modify for PR350462
				res = i2c_master_send(obj->client, databuf, 0x2);
				if(res <= 0)
				{
					return;
				}
		}
#endif
		//let up layer to know
		if((err = hwmsen_get_interrupt_data(ID_PROXIMITY, &sensor_data)))
		{
		  APS_ERR("call hwmsen_get_interrupt_data fail = %d\n", err);
		}
	}
	tmd2772_clear_intr(obj->client);
	mt65xx_eint_unmask(CUST_EINT_ALS_NUM);      
}


static int tmd2772_open(struct inode *inode, struct file *file)
{
	file->private_data = tmd2772_i2c_client;

	if (!file->private_data)
	{
		APS_ERR("null pointer!!\n");
		return -EINVAL;
	}
	
	return nonseekable_open(inode, file);
}
/*----------------------------------------------------------------------------*/
static int tmd2772_release(struct inode *inode, struct file *file)
{
	file->private_data = NULL;
	return 0;
}

static tmd2772_WriteCalibration(struct PS_CALI_DATA_STRUCT *data_cali)
{

	   APS_LOG("tmd2772_WriteCalibration  1 %d," ,data_cali->close);
		   APS_LOG("tmd2772_WriteCalibration  2 %d," ,data_cali->far_away);
		   APS_LOG("tmd2772_WriteCalibration  3 %d,", data_cali->valid);
		   
	  if(data_cali->valid == 1)
	  {
			ps_cali.close = data_cali->close;
			ps_cali.far_away= data_cali->far_away;
			ps_cali.valid = 1;
	  }
	  
}

static u16 prox_mean_ori;
//static u16 prox_mean_off;
static u16 offset_value;
static u16 thd_low;
static u16 thd_hi;

static int tmd2772_read_data_for_cali(struct i2c_client *client, struct PS_CALI_DATA_STRUCT *ps_data_cali, int cali_num)
{
     	int i=0 ,err = 0,j = 0;
	u16 buffer[21];
	u16 tmp = 0;
	u8 offset = 0;
	u16 value_buf[M]; //now M = 20
	u16 prox_max = 0,prox_mean = 0;
	u32	sum = 0;
	
	u8 databuf[2];
	u8 myoffset = 0;
	struct tmd2772_priv *obj = NULL;

	int num = cali_num-1;  //We do not use buffer[0] data, because it maybe invalid

	if((cali_num < 1)||(cali_num > M))
	goto EXIT_ERR;

#if MIN_CURRENT_MODE

	mdelay(10);		
	if(err = tmd2772_read_ps_fx(client,tmp))
	{
		APS_ERR("tmd2772_read_data_for_cali fail: %d\n", i); 
		break;
	}	
	while(tmp >= 1023)//判断Prox是否饱和
	{
		buffer[0] = TMD2772_CMM_OFFSET;
 		offset +=PLUS_OFFSET;
		buffer[1] = offset;
		res = i2c_master_send(client, buffer, 0x2);
		if(res <= 0)
		{
			goto EXIT_ERR;
			return TMD2772_ERR_I2C;
		}
		
		mdelay(10);

		if(err = tmd2772_read_ps_fx(client,tmp))
		{
			APS_ERR("tmd2772_read_data_for_cali fail: %d\n", i); 
			break;
		}
		if(offset >= 0x7f)break;	
		
	}
			
	 for(i = 0;i<M;i++)
	 {
		if(err = tmd2772_read_ps_fx(client,&buffer[i]))
		{
				APS_ERR("tmd2772_read_data_for_cali fail: %d\n", i); 
				break;
		}
		else
		{			
				sum += buffer[i];
		}
		mdelay(10);//10
	 }
	 
	for(j = 0;j<M;j++)
		APS_LOG("buffer[%d] = %d\t",j,buffer[j]);

	 if(i == M)
	 {
 	
			prox_mean = sum/M;
			printk("wwl2****prox_mean=%d\r\n",prox_mean);
			if(prox_mean -PROX_MEAN >= PROX_MEAN_OFFSET) //大于希望绕射偏差值
			{
				buffer[0]= TMD2772_CMM_OFFSET;
				buffer[1]= offset+(prox_mean - PROX_MEAN)/OFFSET_OF_COUNT; //减去多余的绕射值
				err= i2c_master_send(client, buffer, 0x2);
				if(err<= 0)
				{
					goto EXIT_ERR;
					return TMD2772_ERR_I2C;
				}				
			}
			else if(PROX_MEAN-prox_mean > PROX_MEAN_OFFSET)//小于希望绕射值偏差
			{
				buffer[0]= TMD2772_CMM_OFFSET;
				offset = offset-(PROX_MEAN-prox_mean)/OFFSET_OF_COUNT; //增加少于的绕射值
				buffer[1] = offset;
				
				err= i2c_master_send(client, buffer, 0x2);
				if(err<= 0)
				{
					goto EXIT_ERR;
					return TMD2772_ERR_I2C;
				}
			}

			 for(i = 0;i<M;i++)
			 {
				if(err = tmd2772_read_ps_fx(client,&buffer[i]))
				{
						APS_ERR("tmd2772_read_data_for_cali fail: %d\n", i); 
						break;
				}
				else
				{
						if(buffer[i] >prox_max) 
						prox_max= buffer[i];				
						sum += buffer[i];
				}
				mdelay(10);//10
			 }
			 
			 for(j = 0;j<M;j++)
			 	APS_LOG("buffer[%d] = %d\t",j,buffer[j]);
			 
			prox_mean = sum/M;
			printk("wwl2****prox_mean=%d\r\n",prox_mean);
			ps_data_cali->close = (((prox_max - prox_mean) * 200 + 50) / 100 + prox_mean);
			ps_data_cali->far_away=( ((prox_max - prox_mean) * 170 + 50) / 100 + prox_mean);
			ps_data_cali->valid = 1;
			
			APS_LOG("tmd2772_read_data_for_cali close  = %d,far_away = %d,valid = %d",ps_data_cali->close,ps_data_cali->far_away,ps_data_cali->valid);
	
	 	}
	 	else
	 	{
	 		ps_data_cali->valid = 0;
	 		err=  -1;
	 	}
#else
		//Need to set 0 to offset register before calibration, because last calibration may modify its value
		ps_data_cali->offset = 0;
		databuf[0]= TMD2772_CMM_OFFSET;
		databuf[1]= 0x0;
		err= i2c_master_send(client, databuf, 0x2);
		if(err<= 0)
		{
			goto EXIT_ERR;
			//return TMD2772_ERR_I2C;
		}

	       mdelay(10); //add for zoe suggest, let buffer[0] calibration data valid, if not delay, it will be 0
	 	for(i = 0;i<cali_num;i++)
		{
			mdelay(10);//10
			if(err = tmd2772_read_ps_fx(client,&buffer[i]))
			{
					APS_ERR("tmd2772_read_data_for_cali fail: %d\n", i); 
					break;
			}
			else
			{			
					sum += buffer[i];
			}
		 }

		for(j = 0;j<cali_num;j++)
			APS_LOG("buffer[%d] = %d\t",j,buffer[j]);
			
		if(i == cali_num)
		{
			sum -= buffer[0]; // We do not use buffer[0] data
			prox_mean = sum/num;
			APS_LOG("wwl3****prox_mean=%d\r\n",prox_mean);


                     prox_mean_ori = prox_mean;
					 
			if((prox_mean > 700)&&(prox_mean <= 800))
			{
				databuf[0] = TMD2772_CMM_OFFSET;
				databuf[1] = 0x42;
				ps_data_cali->offset = databuf[1]; //need save
				err= i2c_master_send(client, databuf, 0x2);
				if(err<= 0)
				{
					goto EXIT_ERR;
				}
				//APS_LOG("offset111");
				goto RECALI;
			}
			else if(prox_mean > 800)
			{
				databuf[0] = TMD2772_CMM_OFFSET;
				databuf[1] = 0x7f;
				ps_data_cali->offset = databuf[1]; //need save
				err= i2c_master_send(client, databuf, 0x2);
				if(err<= 0)
				{
					goto EXIT_ERR;
				}
				//APS_LOG("offset222");
				goto RECALI;
			}
			else if(prox_mean < 100)
			{
				databuf[0] = TMD2772_CMM_OFFSET;
				databuf[1] = 0xff;
				ps_data_cali->offset = databuf[1]; //need save
				err= i2c_master_send(client, databuf, 0x2);
				if(err<= 0)
				{
					goto EXIT_ERR;
				}
				//APS_LOG("offset333");
				goto RECALI;
			}
			
			goto GET_THRE;

RECALI:		
			mdelay(10);//10
			sum = 0;

		 	for(i = 0;i<cali_num;i++)
			{
				mdelay(10);//10
				if(err = tmd2772_read_ps_fx(client,&buffer[i]))
				{
						APS_ERR("tmd2772_read_data_for_cali fail: %d\n", i); 
						break;
				}
				else
				{			
						sum += buffer[i];
				}
			 }

			for(j = 0;j<cali_num;j++)
				APS_LOG("buffer[%d] = %d\t",j,buffer[j]);

			if(i == cali_num)
			{
				sum -= buffer[0];  //We do not use buffer[0] data
				prox_mean = sum/num;
	   	              APS_LOG("opto-sensor****prox_mean=%d\r\n", prox_mean);
				if(prox_mean<80 || prox_mean>800)goto EXIT_ERR;
			}
			else
			{
				goto EXIT_ERR;
			}
GET_THRE:
		
			ps_data_cali->close = prox_mean+60; //this can customize, adjust close and farway distance
			ps_data_cali->far_away=prox_mean+20;
			ps_data_cali->valid = 1;
			APS_LOG("tmd2772_read_data_for_cali close  = %d,far_away = %d,valid = %d, ofs = %d",ps_data_cali->close,ps_data_cali->far_away,ps_data_cali->valid, ps_data_cali->offset);
			return err; //Do not go through to EXIT_ERR

		}
		else
		{
		 	ps_data_cali->valid = 0;
		 	err=  -1;
		}
	
#endif	

EXIT_ERR:


	//Use default threshold value for close and faraway
	obj = i2c_get_clientdata(client);
	if((boot_ps_ok==1)&&((boot_ps_cali.close-60)>200))
	{
		databuf[0] = TMD2772_CMM_OFFSET;
		databuf[1] = boot_ps_cali.offset;
		err= i2c_master_send(client, databuf, 0x2);

		ps_data_cali->close = boot_ps_cali.close; //this can customize, adjust close and farway distance
		ps_data_cali->far_away=boot_ps_cali.far_away;
		ps_data_cali->valid = 1;
		APS_LOG("boot ps ok, close=%d, far=%d, offset=%d\n", ps_data_cali->close, ps_data_cali->far_away, ps_data_cali->offset);
		return 0;
	}
	else
	{
		//If calibration failed, recover the offset register to 0 to use default threshold.
		databuf[0] = TMD2772_CMM_OFFSET;
		databuf[1] = 0x0;
		err= i2c_master_send(client, databuf, 0x2);
		
		ps_data_cali->close = obj->hw->ps_threshold_high;
		ps_data_cali->far_away = obj->hw->ps_threshold_low;
		ps_data_cali->offset = 0;
		ps_data_cali->valid = 0;
		APS_LOG("tmd2772_read_data_for_cali fail use default close  = %d,far_away = %d,valid = %d",ps_data_cali->close,ps_data_cali->far_away,ps_data_cali->valid);
		err=  -1;
		return err;
	}
	 	
}


static int tmd2772_read_data_for_cali_for_call(struct i2c_client *client, struct PS_CALI_DATA_STRUCT *ps_data_cali, int cali_num)
{
     	int i=0 ,err = 0,j = 0;
	u16 buffer[21];
	u16 tmp = 0;
	u8 offset = 0;
	u16 value_buf[4]; //now M = 20
	u16 prox_max = 0,prox_mean = 0;
	u32	sum = 0;
	
	u8 databuf[2];
	u8 myoffset = 0;
	struct tmd2772_priv *obj = NULL;

        int num = cali_num-1;  //we only use 3

		//Need to set 0 to offset register before calibration, because last calibration may modify its value
		ps_data_cali->offset = 0;
		databuf[0]= TMD2772_CMM_OFFSET;
		databuf[1]= 0x0;
		err= i2c_master_send(client, databuf, 0x2);
		if(err<= 0)
		{
			goto EXIT_ERR;
		}
		
		mdelay(10); //add for zoe suggest, let buffer[0] calibration data valid, if not delay, it will be 0
	 	for(i = 0;i<cali_num;i++)
		{
			mdelay(10);//10
			if(err = tmd2772_read_ps_fx(client,&buffer[i]))
			{
					APS_ERR("tmd2772_read_data_for_cali fail: %d\n", i); 
					break;
			}
			else
			{			
					sum += buffer[i];
			}
		 }

	
		for(j = 0;j<cali_num;j++)
			APS_LOG("buffer[%d] = %d\t",j,buffer[j]);
		
		if(i == cali_num)
		{
			sum -= buffer[0];   // We do not use the first data
			prox_mean = sum/num;
			APS_LOG("wwl3****prox_mean=%d\r\n",prox_mean);
			if((prox_mean > 700)&&(prox_mean <= 800))
			{
				databuf[0] = TMD2772_CMM_OFFSET;
				databuf[1] = 0x25;
				ps_data_cali->offset = databuf[1]; //need save
				err= i2c_master_send(client, databuf, 0x2);
				if(err<= 0)
				{
					goto EXIT_ERR;
				}
				APS_LOG("offset is %x\n",databuf[1]);
				goto RECALI;
			}
			else if(prox_mean > 800)
			{
				databuf[0] = TMD2772_CMM_OFFSET;
				databuf[1] = 0x2b;
				ps_data_cali->offset = databuf[1]; //need save
				err= i2c_master_send(client, databuf, 0x2);
				if(err<= 0)
				{
					goto EXIT_ERR;
				}
				APS_LOG("offsetis %x\n",databuf[1]);
				goto RECALI;
			}
			else if(prox_mean < 100)
			{
				databuf[0] = TMD2772_CMM_OFFSET;
				databuf[1] = 0x98;
				ps_data_cali->offset = databuf[1]; //need save
				err= i2c_master_send(client, databuf, 0x2);
				if(err<= 0)
				{
					goto EXIT_ERR;
				}
				//APS_LOG("offset333");
				goto RECALI;
			}
			
			goto GET_THRE;

RECALI:		
			mdelay(10);//10
			sum = 0;

		 	for(i = 0;i<cali_num;i++)
			{
				mdelay(10);//10
				if(err = tmd2772_read_ps_fx(client,&buffer[i]))
				{
						APS_ERR("tmd2772_read_data_for_cali fail: %d\n", i); 
						break;
				}
				else
				{
						sum += buffer[i];
				}
			 }

			
			for(j = 0;j<cali_num;j++)
				APS_LOG("buffer[%d] = %d\t",j,buffer[j]);

			if(i == cali_num)
			{
				sum -= buffer[0];
				prox_mean = sum/num;  // We do not use the first data
	   	              APS_LOG("opto-sensor****prox_mean=%d\r\n",prox_mean);
				if(prox_mean<80 || prox_mean>800)goto EXIT_ERR;
			}
			else
			{
				goto EXIT_ERR;
			}
GET_THRE:
		
			ps_data_cali->close = prox_mean+100; //this can customize, adjust close and farway distance
			ps_data_cali->far_away=prox_mean+40;
			ps_data_cali->valid = 1;
			APS_LOG("tmd2772_read_data_for_cali close  = %d,far_away = %d,valid = %d, ofs = %d",ps_data_cali->close,ps_data_cali->far_away,ps_data_cali->valid, ps_data_cali->offset);
			return err; //Do not go through to EXIT_ERR

		}
		else
		{
		 	ps_data_cali->valid = 0;
		 	err=  -1;
		}
	


EXIT_ERR:


	//Use default threshold value for close and faraway
	obj = i2c_get_clientdata(client);
	if((boot_ps_ok==1)&&((boot_ps_cali.close-60)>200))
	{
		databuf[0] = TMD2772_CMM_OFFSET;
		databuf[1] = boot_ps_cali.offset;
		err= i2c_master_send(client, databuf, 0x2);

		ps_data_cali->close = boot_ps_cali.close; //this can customize, adjust close and farway distance
		ps_data_cali->far_away=boot_ps_cali.far_away;
		ps_data_cali->offset = boot_ps_cali.offset;
		ps_data_cali->valid = 1;
		APS_LOG("boot ps ok, close=%d, far=%d, offset=%d\n", ps_data_cali->close, ps_data_cali->far_away, ps_data_cali->offset);
		return 0;
	}
	else
	{
		//If calibration failed, recover the offset register to 0 to use default threshold.
		databuf[0] = TMD2772_CMM_OFFSET;
		databuf[1] = 0x0;
		err= i2c_master_send(client, databuf, 0x2);
		
		ps_data_cali->close = obj->hw->ps_threshold_high;
		ps_data_cali->far_away = obj->hw->ps_threshold_low;
		ps_data_cali->offset = 0;
		ps_data_cali->valid = 0;
		APS_LOG("tmd2772_read_data_for_cali fail use default close  = %d,far_away = %d,valid = %d",ps_data_cali->close,ps_data_cali->far_away,ps_data_cali->valid);
		err=  -1;
		return err;
	}
	 	
}


/*----------------------------------------------------------------------------*/
static long tmd2772_unlocked_ioctl(struct file *file, unsigned int cmd,
       unsigned long arg)
{
	struct i2c_client *client = (struct i2c_client*)file->private_data;
	struct tmd2772_priv *obj = i2c_get_clientdata(client);  
	long err = 0;
	void __user *ptr = (void __user*) arg;
	int dat;
	uint32_t enable;
	struct PS_CALI_DATA_STRUCT ps_cali_temp;

	switch (cmd)
	{
		case ALSPS_SET_PS_MODE:
			if(copy_from_user(&enable, ptr, sizeof(enable)))
			{
				err = -EFAULT;
				goto err_out;
			}
			if(enable)
			{
				if(err = tmd2772_enable_ps(obj->client, 1))
				{
					APS_ERR("enable ps fail: %d\n", err); 
					goto err_out;
				}
				
				set_bit(CMC_BIT_PS, &obj->enable);
			}
			else
			{
				if(err = tmd2772_enable_ps(obj->client, 0))
				{
					APS_ERR("disable ps fail: %d\n", err); 
					goto err_out;
				}
				
				clear_bit(CMC_BIT_PS, &obj->enable);
			}
			break;

		case ALSPS_GET_PS_MODE:
			enable = test_bit(CMC_BIT_PS, &obj->enable) ? (1) : (0);
			if(copy_to_user(ptr, &enable, sizeof(enable)))
			{
				err = -EFAULT;
				goto err_out;
			}
			break;

		case ALSPS_GET_PS_DATA:    
			if(err = tmd2772_read_ps(obj->client, &obj->ps))
			{
				goto err_out;
			}
			
			dat = tmd2772_get_ps_value(obj, obj->ps);
			if(copy_to_user(ptr, &dat, sizeof(dat)))
			{
				err = -EFAULT;
				goto err_out;
			}  
			break;

		case ALSPS_GET_PS_RAW_DATA:    
			if(err = tmd2772_read_ps(obj->client, &obj->ps))
			{
				goto err_out;
			}
			
			dat = obj->ps;
			if(copy_to_user(ptr, &dat, sizeof(dat)))
			{
				err = -EFAULT;
				goto err_out;
			}  
			break;              

		case ALSPS_SET_ALS_MODE:
			if(copy_from_user(&enable, ptr, sizeof(enable)))
			{
				err = -EFAULT;
				goto err_out;
			}
			if(enable)
			{
				if(err = tmd2772_enable_als(obj->client, 1))
				{
					APS_ERR("enable als fail: %d\n", err); 
					goto err_out;
				}
				set_bit(CMC_BIT_ALS, &obj->enable);
			}
			else
			{
				if(err = tmd2772_enable_als(obj->client, 0))
				{
					APS_ERR("disable als fail: %d\n", err); 
					goto err_out;
				}
				clear_bit(CMC_BIT_ALS, &obj->enable);
			}
			break;

		case ALSPS_GET_ALS_MODE:
			enable = test_bit(CMC_BIT_ALS, &obj->enable) ? (1) : (0);
			if(copy_to_user(ptr, &enable, sizeof(enable)))
			{
				err = -EFAULT;
				goto err_out;
			}
			break;

		case ALSPS_GET_ALS_DATA: 
			if(err = tmd2772_read_als(obj->client, &obj->als))
			{
				goto err_out;
			}

			dat = tmd2772_get_als_value(obj, obj->als);
			if(copy_to_user(ptr, &dat, sizeof(dat)))
			{
				err = -EFAULT;
				goto err_out;
			}              
			break;

		case ALSPS_GET_ALS_RAW_DATA:    
			if(err = tmd2772_read_als(obj->client, &obj->als))
			{
				goto err_out;
			}

			dat = obj->als;
			if(copy_to_user(ptr, &dat, sizeof(dat)))
			{
				err = -EFAULT;
				goto err_out;
			}              
			break;
		default:
			APS_ERR("%s not supported = 0x%04x", __FUNCTION__, cmd);
			err = -ENOIOCTLCMD;
			break;
	}

	err_out:
	return err;    
}
/*----------------------------------------------------------------------------*/
static struct file_operations tmd2772_fops = {
	.owner = THIS_MODULE,
	.open = tmd2772_open,
	.release = tmd2772_release,
	.unlocked_ioctl = tmd2772_unlocked_ioctl,
};
/*----------------------------------------------------------------------------*/
static struct miscdevice tmd2772_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "als_ps",
	.fops = &tmd2772_fops,
};
/*----------------------------------------------------------------------------*/
static int tmd2772_i2c_suspend(struct i2c_client *client, pm_message_t msg) 
{
	//struct tmd2772_priv *obj = i2c_get_clientdata(client);    
	//int err;
	APS_FUN();    
#if 0
	if(msg.event == PM_EVENT_SUSPEND)
	{   
		if(!obj)
		{
			APS_ERR("null pointer!!\n");
			return -EINVAL;
		}
		
		atomic_set(&obj->als_suspend, 1);
		if(err = tmd2772_enable_als(client, 0))
		{
			APS_ERR("disable als: %d\n", err);
			return err;
		}

		atomic_set(&obj->ps_suspend, 1);
		if(err = tmd2772_enable_ps(client, 0))
		{
			APS_ERR("disable ps:  %d\n", err);
			return err;
		}
		
		tmd2772_power(obj->hw, 0);
	}
#endif
	return 0;
}
/*----------------------------------------------------------------------------*/
static int tmd2772_i2c_resume(struct i2c_client *client)
{
	//struct tmd2772_priv *obj = i2c_get_clientdata(client);        
	//int err;
	APS_FUN();
#if 0
	if(!obj)
	{
		APS_ERR("null pointer!!\n");
		return -EINVAL;
	}

	tmd2772_power(obj->hw, 1);
	if(err = tmd2772_init_client(client))
	{
		APS_ERR("initialize client fail!!\n");
		return err;        
	}
	atomic_set(&obj->als_suspend, 0);
	if(test_bit(CMC_BIT_ALS, &obj->enable))
	{
		if(err = tmd2772_enable_als(client, 1))
		{
			APS_ERR("enable als fail: %d\n", err);        
		}
	}
	atomic_set(&obj->ps_suspend, 0);
	if(test_bit(CMC_BIT_PS,  &obj->enable))
	{
		if(err = tmd2772_enable_ps(client, 1))
		{
			APS_ERR("enable ps fail: %d\n", err);                
		}
	}
#endif
	return 0;
}
/*----------------------------------------------------------------------------*/
static void tmd2772_early_suspend(struct early_suspend *h) 
{   /*early_suspend is only applied for ALS*/
	struct tmd2772_priv *obj = container_of(h, struct tmd2772_priv, early_drv);   
	int err;
	APS_FUN();    

	if(!obj)
	{
		APS_ERR("null pointer!!\n");
		return;
	}

	#if 1
	atomic_set(&obj->als_suspend, 1);
	if(test_bit(CMC_BIT_ALS, &obj->enable))
	{
		if(err = tmd2772_enable_als(obj->client, 0))
		{
			APS_ERR("disable als fail: %d\n", err); 
		}
	}
	#endif
}
/*----------------------------------------------------------------------------*/
static void tmd2772_late_resume(struct early_suspend *h)
{   /*early_suspend is only applied for ALS*/
	struct tmd2772_priv *obj = container_of(h, struct tmd2772_priv, early_drv);         
	int err;
	APS_FUN();

	if(!obj)
	{
		APS_ERR("null pointer!!\n");
		return;
	}

        #if 1
	atomic_set(&obj->als_suspend, 0);
	if(test_bit(CMC_BIT_ALS, &obj->enable))
	{
		if(err = tmd2772_enable_als(obj->client, 1))
		{
			APS_ERR("enable als fail: %d\n", err);        

		}
	}
	#endif
}

int tmd2772_ps_operate(void* self, uint32_t command, void* buff_in, int size_in,
		void* buff_out, int size_out, int* actualout)
{
	int err = 0;
	int value;
	hwm_sensor_data* sensor_data;
	struct tmd2772_priv *obj = (struct tmd2772_priv *)self;
	
	//APS_FUN(f);
	switch (command)
	{
		case SENSOR_DELAY:
			if((buff_in == NULL) || (size_in < sizeof(int)))
			{
				APS_ERR("Set delay parameter error!\n");
				err = -EINVAL;
			}
			// Do nothing
			break;

		case SENSOR_ENABLE:
			if((buff_in == NULL) || (size_in < sizeof(int)))
			{
				APS_ERR("Enable sensor parameter error!\n");
				err = -EINVAL;
			}
			else
			{				
				value = *(int *)buff_in;
				if(value)
				{
#ifdef USE_CALIBRATION_ON_CALL
				//add for calibration on call setup, now we read 3 times, before setting the interrupt threshold  register
#if 0
				databuf[0] = TMD2772_CMM_ENABLE;
				databuf[1] = 0x05;
				res = i2c_master_send(client, databuf, 0x2);
				if(res <= 0)
				{
				  goto EXIT_ERR;
				  return TMD2772_ERR_I2C;
				}
#endif
				//APS_LOG("111*\n");
				tmd2772_init_client_for_cali(obj->client);
				tmd2772_read_data_for_cali_for_call(obj->client,&ps_cali, 4);  //Note: We do not use first ps read data
				//APS_LOG("222*\n");
				
			       offset_value = ps_cali.offset;
                   thd_hi = ps_cali.close;
                   thd_low = ps_cali.far_away;
#endif

				
					if(err = tmd2772_enable_ps(obj->client, 1))
					{
						APS_ERR("enable ps fail: %d\n", err); 
						return -1;
					}
					set_bit(CMC_BIT_PS, &obj->enable);
					#if 0	
					if(err = tmd2772_enable_als(obj->client, 1))
					{
						APS_ERR("enable als fail: %d\n", err); 
						return -1;
					}
					set_bit(CMC_BIT_ALS, &obj->enable);
					#endif
				}
				else
				{
					if(err = tmd2772_enable_ps(obj->client, 0))
					{
						APS_ERR("disable ps fail: %d\n", err); 
						return -1;
					}
					clear_bit(CMC_BIT_PS, &obj->enable);
					#if 0
					if(err = tmd2772_enable_als(obj->client, 0))
					{
						APS_ERR("disable als fail: %d\n", err); 
						return -1;
					}
					clear_bit(CMC_BIT_ALS, &obj->enable);
					#endif
				}
			}
			break;

		case SENSOR_GET_DATA:
			if((buff_out == NULL) || (size_out< sizeof(hwm_sensor_data)))
			{
				APS_ERR("get sensor data parameter error!\n");
				err = -EINVAL;
			}
			else
			{
				sensor_data = (hwm_sensor_data *)buff_out;	
				tmd2772_read_ps(obj->client, &obj->ps);
				
                                //mdelay(160);
				tmd2772_read_als_ch0(obj->client, &obj->als);
				APS_ERR("tmd2772_ps_operate als data=%d!\n",obj->als);
				sensor_data->values[0] = tmd2772_get_ps_value(obj, obj->ps);
				sensor_data->value_divide = 1;
				sensor_data->status = SENSOR_STATUS_ACCURACY_MEDIUM;			
			}
			break;
		default:
			APS_ERR("proxmy sensor operate function no this parameter %d!\n", command);
			err = -1;
			break;
	}
	
	return err;
}

int tmd2772_als_operate(void* self, uint32_t command, void* buff_in, int size_in,
		void* buff_out, int size_out, int* actualout)
{
	int err = 0;
	int value;
	hwm_sensor_data* sensor_data;
	struct tmd2772_priv *obj = (struct tmd2772_priv *)self;

	switch (command)
	{
		case SENSOR_DELAY:
			if((buff_in == NULL) || (size_in < sizeof(int)))
			{
				APS_ERR("Set delay parameter error!\n");
				err = -EINVAL;
			}
			// Do nothing
			break;

		case SENSOR_ENABLE:
			if((buff_in == NULL) || (size_in < sizeof(int)))
			{
				APS_ERR("Enable sensor parameter error!\n");
				err = -EINVAL;
			}
			else
			{
				value = *(int *)buff_in;				
				if(value)
				{
					if(err = tmd2772_enable_als(obj->client, 1))
					{
						APS_ERR("enable als fail: %d\n", err); 
						return -1;
					}
					set_bit(CMC_BIT_ALS, &obj->enable);
				}
				else
				{
					if(err = tmd2772_enable_als(obj->client, 0))
					{
						APS_ERR("disable als fail: %d\n", err); 
						return -1;
					}
					clear_bit(CMC_BIT_ALS, &obj->enable);
				}
				
			}
			break;

		case SENSOR_GET_DATA:
			if((buff_out == NULL) || (size_out< sizeof(hwm_sensor_data)))
			{
				APS_ERR("get sensor data parameter error!\n");
				err = -EINVAL;
			}
			else
			{
				sensor_data = (hwm_sensor_data *)buff_out;
				/*yucong MTK add for fixing know issue*/
				#if 1
				tmd2772_read_als(obj->client, &obj->als);
				//if(obj->als == 0) //modify for PR353124
				//{
				//	sensor_data->values[0] = -1;		
					
				//}else{
					u16 b[2];
					int i;
					for(i = 0;i < 2;i++){
					tmd2772_read_als(obj->client, &obj->als);
					b[i] = obj->als;
					}
					(b[1] > b[0])?(obj->als = b[0]):(obj->als = b[1]);
					sensor_data->values[0] = tmd2772_get_als_value(obj, obj->als);
				//}
				#endif
				sensor_data->value_divide = 1;
				sensor_data->status = SENSOR_STATUS_ACCURACY_MEDIUM;
			}
			break;
		default:
			APS_ERR("light sensor operate function no this parameter %d!\n", command);
			err = -1;
			break;
	}
	
	return err;
}


/*----------------------------------------------------------------------------*/
static int tmd2772_i2c_detect(struct i2c_client *client, /*int kind,*/ struct i2c_board_info *info) 
{    
	strcpy(info->type, TMD2772_DEV_NAME);
	return 0;
}


/*yi.zheng.hz add for debug p sensor calibration at 2013-06-27 start*/
#ifdef DEBUG_TMD2772_CALIBRATION
static ssize_t show_cali_value(struct device_driver *ddri, char *buf)
{
	int len = 0;

	len = snprintf(buf, PAGE_SIZE, "latest call cali: close = %d, far = %d, valid = %d, offset = %d\nboot cali: close=%d, far = %d, valid = %d, offset=%d\nlatest ps : %d,  %d\n", 
		ps_cali.close, ps_cali.far_away, ps_cali.valid, ps_cali.offset, 
		boot_ps_cali.close, boot_ps_cali.far_away, boot_ps_cali.valid, boot_ps_cali.offset,
		ps_latest[0], ps_latest[1]);
	return len;
}

static ssize_t show_ps(struct device_driver *ddri, char *buf)
{
	int len = 0;
	long err;


	if(err = tmd2772_read_ps_fx(g_tmd2772_ptr->client,&g_tmd2772_ptr->ps))
	{
		APS_ERR("tmd2772_read_data_for_cali fail !!!\n"); 
		
	}	
       else
	{
	    len = snprintf(buf, PAGE_SIZE, "%d\n", g_tmd2772_ptr->ps);
	}
	return len;
}

static ssize_t show_low_threshold(struct device_driver *ddri, char *buf)
{
	int len = 0;
	long err;


	    len = snprintf(buf, PAGE_SIZE, "%d\n", 
		thd_low);
	return len;
}

static ssize_t show_high_threshold(struct device_driver *ddri, char *buf)
{
	int len = 0;
	long err;


	    len = snprintf(buf, PAGE_SIZE, "%d\n", 
		thd_hi);
	return len;
}

static ssize_t show_offset(struct device_driver *ddri, char *buf)
{
	int len = 0;
	long err;


	    len = snprintf(buf, PAGE_SIZE, "%d\n", 
		offset_value);
	return len;
}

static ssize_t show_originalPS(struct device_driver *ddri, char *buf)
{
	int len = 0;
	long err;


	    len = snprintf(buf, PAGE_SIZE, "%d\n", 
		prox_mean_ori);
	return len;
}

static ssize_t show_style(struct device_driver *ddri, char *buf)
{
	int len = 0;
	long err;


	    len = snprintf(buf, PAGE_SIZE, "TMD\n");
	return len;
}

/* cat /sys/bus/platform/drivers/als_ps/cali */
static DRIVER_ATTR(cali,       S_IWUSR | S_IRUGO, show_cali_value,          NULL);
static DRIVER_ATTR(ps,       S_IWUSR | S_IRUGO, show_ps,          NULL);
static DRIVER_ATTR(low_threshold,       S_IWUSR | S_IRUGO, show_low_threshold,          NULL);
static DRIVER_ATTR(high_threshold,       S_IWUSR | S_IRUGO, show_high_threshold,          NULL);
static DRIVER_ATTR(offset,       S_IWUSR | S_IRUGO, show_offset,          NULL);
static DRIVER_ATTR(originalPS,       S_IWUSR | S_IRUGO, show_originalPS,          NULL);
static DRIVER_ATTR(style,       S_IWUSR | S_IRUGO, show_style,          NULL);

static struct driver_attribute *tmd2772_attr_list[] = {
	&driver_attr_cali,
	&driver_attr_low_threshold,
	&driver_attr_high_threshold,
	&driver_attr_offset,
	&driver_attr_originalPS,
	&driver_attr_ps,
	&driver_attr_style,
};
/*----------------------------------------------------------------------------*/
static int tmd2772_create_attr(struct device_driver *driver) 
{
	int idx, err = 0;
	int num = (int)(sizeof(tmd2772_attr_list)/sizeof(tmd2772_attr_list[0]));
	if (driver == NULL)
	{
		return -EINVAL;
	}

	for(idx = 0; idx < num; idx++)
	{
		if((err = driver_create_file(driver, tmd2772_attr_list[idx])))
		{            
			APS_ERR("driver_create_file (%s) = %d\n", tmd2772_attr_list[idx]->attr.name, err);
			break;
		}
	}    
	return err;
}
/*----------------------------------------------------------------------------*/
static int tmd2772_delete_attr(struct device_driver *driver)
{
	int idx ,err = 0;
	int num = (int)(sizeof(tmd2772_attr_list)/sizeof(tmd2772_attr_list[0]));

	if(driver == NULL)
	{
		return -EINVAL;
	}
	

	for(idx = 0; idx < num; idx++)
	{
		driver_remove_file(driver, tmd2772_attr_list[idx]);
	}

	return err;
}
#endif/*DEBUG_TMD2772_CALIBRATION*/
/*yi.zheng.hz add for debug p sensor calibration at 2013-06-27 end*/

/*----------------------------------------------------------------------------*/
static int tmd2772_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct tmd2772_priv *obj;
	struct hwmsen_object obj_ps, obj_als;
	int err = 0;

	if(!(obj = kzalloc(sizeof(*obj), GFP_KERNEL)))
	{
		err = -ENOMEM;
		goto exit;
	}
	memset(obj, 0, sizeof(*obj));
	tmd2772_obj = obj;

	obj->hw = get_cust_alsps_hw();
	tmd2772_get_addr(obj->hw, &obj->addr);

	/*for interrup work mode support -- by liaoxl.lenovo 12.08.2011*/
	INIT_WORK(&obj->eint_work, tmd2772_eint_work);
	obj->client = client;
	i2c_set_clientdata(client, obj);	
	atomic_set(&obj->als_debounce, 300);
	atomic_set(&obj->als_deb_on, 0);
	atomic_set(&obj->als_deb_end, 0);
	atomic_set(&obj->ps_debounce, 200);
	atomic_set(&obj->ps_deb_on, 0);
	atomic_set(&obj->ps_deb_end, 0);
	atomic_set(&obj->ps_mask, 0);
	atomic_set(&obj->als_suspend, 0);
	atomic_set(&obj->als_cmd_val, 0xDF);
	atomic_set(&obj->ps_cmd_val,  0xC1);
	//atomic_set(&obj->ps_thd_val_high,  obj->hw->ps_threshold_high);
	//atomic_set(&obj->ps_thd_val_low,  obj->hw->ps_threshold_low);
	obj->enable = 0;
	obj->pending_intr = 0;
	obj->als_level_num = sizeof(obj->hw->als_level)/sizeof(obj->hw->als_level[0]);
	obj->als_value_num = sizeof(obj->hw->als_value)/sizeof(obj->hw->als_value[0]);  
	/*Lenovo-sw chenlj2 add 2011-06-03,modified gain 16 to 1/5 accoring to actual thing */
	obj->als_modulus = (400*100*ZOOM_TIME)/(1*150);//(1/Gain)*(400/Tine), this value is fix after init ATIME and CONTROL register value
										//(400)/16*2.72 here is amplify *100 //16
	BUG_ON(sizeof(obj->als_level) != sizeof(obj->hw->als_level));
	memcpy(obj->als_level, obj->hw->als_level, sizeof(obj->als_level));
	BUG_ON(sizeof(obj->als_value) != sizeof(obj->hw->als_value));
	memcpy(obj->als_value, obj->hw->als_value, sizeof(obj->als_value));
	atomic_set(&obj->i2c_retry, 3);
	set_bit(CMC_BIT_ALS, &obj->enable);
	set_bit(CMC_BIT_PS, &obj->enable);

	
	tmd2772_i2c_client = client;

	tmd2772_init_client_for_cali(client);
	if(err= tmd2772_read_data_for_cali(client,&boot_ps_cali, M))
		{
			boot_ps_ok=0;
		}

	else
		{
			boot_ps_ok=1;
		}

	offset_value = boot_ps_cali.offset;
	thd_hi = boot_ps_cali.close;
    thd_low = boot_ps_cali.far_away;
	
	if(err = tmd2772_init_client(client))
	{
		goto exit_init_failed;
	}
	APS_LOG("tmd2772_init_client() OK!\n");

	if(err = misc_register(&tmd2772_device))
	{
		APS_ERR("tmd2772_device register failed\n");
		goto exit_misc_device_register_failed;
	}

/*yi.zheng.hz modify for debug p sensor calibration at 2013-06-27 start*/
#ifdef DEBUG_TMD2772_CALIBRATION
	struct device_driver *alsps_driver = NULL;
#if defined(MTK_AUTO_DETECT_ALSPS)
	alsps_driver = &(tmd2772_init_info.platform_diver_addr->driver);
#else
	alsps_driver = &tmd2772_alsps_driver.driver;
#endif
         if(err = tmd2772_create_attr(alsps_driver))
	{
		APS_ERR("create attribute err = %d\n", err);
		goto exit_create_attr_failed;
	}
#endif/*DEBUG_TMD2772_CALIBRATION*/
/*yi.zheng.hz modify for debug p sensor calibration at 2013-06-27 end*/


	obj_ps.self = tmd2772_obj;
	/*for interrup work mode support -- by liaoxl.lenovo 12.08.2011*/
	if(1 == obj->hw->polling_mode_ps)
	{
		obj_ps.polling = 1;
	}
	else
	{
		obj_ps.polling = 0;
	}

	obj_ps.sensor_operate = tmd2772_ps_operate;
	if(err = hwmsen_attach(ID_PROXIMITY, &obj_ps))
	{
		APS_ERR("attach fail = %d\n", err);
		goto exit_hwmsen_attach_failed;// exit_create_attr_failed;
	}
	
	obj_als.self = tmd2772_obj;
	obj_als.polling = 1;
	obj_als.sensor_operate = tmd2772_als_operate;
	if(err = hwmsen_attach(ID_LIGHT, &obj_als))
	{
		APS_ERR("attach fail = %d\n", err);
		goto exit_hwmsen_attach_failed;// exit_create_attr_failed;
	}


#if defined(CONFIG_HAS_EARLYSUSPEND)
	obj->early_drv.level    = EARLY_SUSPEND_LEVEL_DISABLE_FB - 1,
	obj->early_drv.suspend  = tmd2772_early_suspend,
	obj->early_drv.resume   = tmd2772_late_resume,    
	register_early_suspend(&obj->early_drv);
#endif

	APS_LOG("%s: OK\n", __func__);
/*************wwl add psensor get calibration data when calling    start*****************/
#if defined(MTK_AUTO_DETECT_ALSPS)
    	tmd2772_init_flag = 0;
#endif
/*************wwl add psensor get calibration data when calling    end*****************/
	return 0;

/*yi.zheng.hz modify for debug p sensor calibration at 2013-06-27 start*/
	exit_hwmsen_attach_failed:
#ifdef DEBUG_TMD2772_CALIBRATION
	tmd2772_delete_attr(alsps_driver);
#endif
/*yi.zheng.hz modify for debug p sensor calibration at 2013-06-27 end*/

	exit_create_attr_failed:
	misc_deregister(&tmd2772_device);
	exit_misc_device_register_failed:
	exit_init_failed:
	//i2c_detach_client(client);
	exit_kfree:
	kfree(obj);
	exit:
	tmd2772_i2c_client = NULL;           
//	MT6516_EINTIRQMask(CUST_EINT_ALS_NUM);  /*mask interrupt if fail*/
	APS_ERR("%s: err = %d\n", __func__, err);
/*************wwl add psensor get calibration data when calling    start*****************/
#if defined(MTK_AUTO_DETECT_ALSPS)
	tmd2772_init_flag = -1;
#endif
/*************wwl add psensor get calibration data when calling    end*****************/

	return err;
}
/*----------------------------------------------------------------------------*/
static int tmd2772_i2c_remove(struct i2c_client *client)
{
	int err;	

/*yi.zheng.hz modify for debug p sensor calibration at 2013-06-27 start*/
#ifdef DEBUG_TMD2772_CALIBRATION
	struct device_driver *alsps_driver = NULL;
#if defined(MTK_AUTO_DETECT_ALSPS)
	alsps_driver = &(tmd2772_init_info.platform_diver_addr->driver);
#else
	alsps_driver = &tmd2772_alsps_driver.driver;
#endif

	if(err = tmd2772_delete_attr(alsps_driver))
	{
		APS_ERR("tmd2772_delete_attr fail: %d\n", err);
	}
#endif/*DEBUG_TMD2772_CALIBRATION*/
/*yi.zheng.hz modify for debug p sensor calibration at 2013-06-27 end*/
	if(err = misc_deregister(&tmd2772_device))
	{
		APS_ERR("misc_deregister fail: %d\n", err);    
	}
	
	tmd2772_i2c_client = NULL;
	i2c_unregister_device(client);
	kfree(i2c_get_clientdata(client));

	return 0;
}
/*----------------------------------------------------------------------------*/
/*************wwl add psensor get calibration data when calling    start*****************/
#if defined(MTK_AUTO_DETECT_ALSPS)
static int tmd2772_for_auto_local_init(void)
#else
static int tmd2772_probe(struct platform_device *pdev) 
#endif
/*************wwl add psensor get calibration data when calling    end*****************/
{
	struct alsps_hw *hw = get_cust_alsps_hw();
printk("tmd2772_probe");

	tmd2772_power(hw, 1);    
	//tmd2772_force[0] = hw->i2c_num;
	//tmd2772_force[1] = hw->i2c_addr[0];
	//APS_DBG("I2C = %d, addr =0x%x\n",tmd2772_force[0],tmd2772_force[1]);
	if(i2c_add_driver(&tmd2772_i2c_driver))
	{
		APS_ERR("add driver error\n");
		return -1;
	} 
/*************wwl add psensor get calibration data when calling    start*****************/
#if defined(MTK_AUTO_DETECT_ALSPS)
    if(tmd2772_init_flag)
    {
        return -1;
    }
#endif
/*************wwl add psensor get calibration data when calling    end*****************/
	return 0;
}
/*----------------------------------------------------------------------------*/
static int tmd2772_remove(struct platform_device *pdev)
{
	struct alsps_hw *hw = get_cust_alsps_hw();
	APS_FUN();    
	tmd2772_power(hw, 0);    
	i2c_del_driver(&tmd2772_i2c_driver);
	return 0;
}
/*----------------------------------------------------------------------------*/
/*************wwl add psensor get calibration data when calling    start*****************/
#if !defined(MTK_AUTO_DETECT_ALSPS) 
static struct platform_driver tmd2772_alsps_driver = {
	.probe      = tmd2772_probe,
	.remove     = tmd2772_remove,    
	.driver     = {
		.name  = "als_ps",
//		.owner = THIS_MODULE,
	}
};
#endif
/*************wwl add psensor get calibration data when calling    end*****************/
/*----------------------------------------------------------------------------*/
static int __init tmd2772_init(void)
{
	APS_FUN();
	printk("tmd2772_init");
	i2c_register_board_info(1, &i2c_tmd2772, 1); //i2c bus1;
/*************wwl add psensor get calibration data when calling    start*****************/
	#if defined(MTK_AUTO_DETECT_ALSPS)
	hwmsen_alsps_sensor_add(&tmd2772_init_info);
	#else
	if(platform_driver_register(&tmd2772_alsps_driver))
	{
		APS_ERR("failed to register driver");
		return -ENODEV;
	}
	#endif
/*************wwl add psensor get calibration data when calling    end*****************/

	return 0;
}
/*----------------------------------------------------------------------------*/
static void __exit tmd2772_exit(void)
{
	APS_FUN();
/*************wwl add psensor get calibration data when calling    start*****************/
	#if !defined(MTK_AUTO_DETECT_ALSPS)
	platform_driver_unregister(&tmd2772_alsps_driver);
	#endif
/*************wwl add psensor get calibration data when calling    end*****************/
}
/*----------------------------------------------------------------------------*/
module_init(tmd2772_init);
module_exit(tmd2772_exit);
/*----------------------------------------------------------------------------*/
MODULE_AUTHOR("Dexiang Liu");
MODULE_DESCRIPTION("tmd2772 driver");
MODULE_LICENSE("GPL");
