/*****************************************************************************
 *
 * Copyright (c) 2013 mCube, Inc.  All rights reserved.
 *
 * This source is subject to the mCube Software License.
 * This software is protected by Copyright and the information and source code
 * contained herein is confidential. The software including the source code
 * may not be copied and the information contained herein may not be used or
 * disclosed except with the written permission of mCube Inc.
 *
 * All other rights reserved.
 *
 * This code and information are provided "as is" without warranty of any
 * kind, either expressed or implied, including but not limited to the
 * implied warranties of merchantability and/or fitness for a
 * particular purpose.
 *
 * The following software/firmware and/or related documentation ("mCube Software")
 * have been modified by mCube Inc. All revisions are subject to any receiver's
 * applicable license agreements with mCube Inc.
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
 *
 *****************************************************************************/

/*****************************************************************************
 *** HEADER FILES
 *****************************************************************************/
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

#include <cust_acc.h>
#include <linux/hwmsensor.h>
#include <linux/hwmsen_dev.h>
#include <linux/sensors_io.h>
#include <linux/hwmsen_helper.h>

#include <linux/ioctl.h>
#include <linux/wakelock.h>

#include "mc3433.h"

/*****************************************************************************
 *** CONFIGURATION
 *****************************************************************************/
//#define _MC3433_SUPPORT_AUTO_DETECT_
//#define _MC3433_SUPPORT_DOT_CALIBRATION_
#define _MC3433_SUPPORT_LPF_
#define _MC3433_SUPPORT_CONCURRENCY_PROTECTION_
//#define _MC3433_SUPPORT_APPLY_AVERAGE_AGORITHM_
//#define _MC3433_SUPPORT_PERIODIC_DOC_
//#define _MC3433_SUPPORT_VPROXIMITY_SENSOR_
#define _MC3433_SUPPORT_LRF_
//#define _MC3433_SUPPORT_POWER_SAVING_SHUTDOWN_POWER_

#define C_MAX_FIR_LENGTH    (32)

#define VIRTUAL_Z    0

/*****************************************************************************
 *** CONSTANT / DEFINITION
 *****************************************************************************/
/**************************
 *** CONFIGURATION
 **************************/
#define MC3433_DEV_NAME                        "MC3433"
#define MC3433_DEV_DRIVER_VERSION              "2.1.5"
#define MC3433_DEV_DRIVER_VERSION_VIRTUAL_Z    "1.0.1"

/**************************
 *** COMMON
 **************************/
#define MC3433_AXIS_X      0
#define MC3433_AXIS_Y      1
#define MC3433_AXIS_Z      2
#define MC3433_AXES_NUM    3
#define MC3433_DATA_LEN    6

#define MC3433_RESOLUTION_LOW     1
#define MC3433_RESOLUTION_HIGH    2

#define MC3433_LOW_REOLUTION_DATA_SIZE     3
#define MC3433_HIGH_REOLUTION_DATA_SIZE    6

#define MC3433_INIT_SUCC    (0)
#define MC3433_INIT_FAIL    (-1)

#define MC3433_REGMAP_LENGTH    (64)

/**************************
 *** DEBUG
 **************************/
    /*********************
     *** G-Sensor
     *********************/
    #if 1
        #define GSE_TAG                  "[MC3433] "
        #define GSE_FUN(f)               printk(KERN_ERR GSE_TAG"%s\n", __FUNCTION__)
        #define GSE_ERR(fmt, args...)    printk(KERN_ERR GSE_TAG"%s %d : "fmt, __FUNCTION__, __LINE__, ##args)
        #define GSE_LOG(fmt, args...)    printk(KERN_INFO GSE_TAG fmt, ##args)
    #else
        #define GSE_TAG
        #define GSE_FUN(f)               do {} while (0)
        #define GSE_ERR(fmt, args...)    do {} while (0)
        #define GSE_LOG(fmt, args...)    do {} while (0)
    #endif
    
    /*********************
     *** vProximity Sensor
     *********************/
    #if 0
        #define PS_TAG                  "[mCube/Psensor] "
        #define PS_FUN(f)               printk(KERN_INFO PS_TAG"%s\n", __FUNCTION__)
        #define PS_ERR(fmt, args...)    printk(KERN_ERR  PS_TAG"%s %d : "fmt, __FUNCTION__, __LINE__, ##args)
        #define PS_LOG(fmt, args...)    printk(KERN_ERR PS_TAG fmt, ##args)
    #else
        #define PS_TAG
        #define PS_FUN(f)               do {} while (0)
        #define PS_ERR(fmt, args...)    do {} while (0)
        #define PS_LOG(fmt, args...)    do {} while (0)
    #endif       

/*****************************************************************************
 *** DATA TYPE / STRUCTURE DEFINITION / ENUM
 *****************************************************************************/

static const struct i2c_device_id                MC3433_i2c_id[]              = { {MC3433_DEV_NAME, 0}, {} };
static struct i2c_board_info __initdata    MC3433_i2c_board_info        = { I2C_BOARD_INFO(MC3433_DEV_NAME, MC3433_I2C_SLAVE_WRITE_ADDR>>1) };

static int MC3433_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id); 
static int MC3433_i2c_remove(struct i2c_client *client);
static int MC3433_i2c_detect(struct i2c_client *client, int kind ,struct i2c_board_info *info);

typedef enum
{
    MCUBE_TRC_FILTER  = 0x01,
    MCUBE_TRC_RAWDATA = 0x02,
    MCUBE_TRC_IOCTL   = 0x04,
    MCUBE_TRC_CALI    = 0X08,
    MCUBE_TRC_INFO    = 0X10,
    MCUBE_TRC_REGXYZ  = 0X20,
}   MCUBE_TRC;

struct scale_factor
{
    u8    whole;
    u8    fraction;
};

struct data_resolution
{
    struct scale_factor    scalefactor;
    int                    sensitivity;
};

struct data_filter
{
    s16    raw[C_MAX_FIR_LENGTH][MC3433_AXES_NUM];
    int    sum[MC3433_AXES_NUM];
    int    num;
    int    idx;
};

struct MC3433_i2c_data
{
    //================================================
    struct i2c_client          *client;
    struct acc_hw              *hw;
    struct hwmsen_convert       cvt;

    //================================================
    struct data_resolution     *reso;
    atomic_t                    trace;
    atomic_t                    suspend;
    atomic_t                    selftest;
    atomic_t                    filter;
    atomic_t			sensor_power;
    s16                         cali_sw[MC3433_AXES_NUM + 1];
    
    //================================================
    s16                         offset[MC3433_AXES_NUM + 1];
    s16                         data[MC3433_AXES_NUM + 1];
    
    //================================================
    #if defined(_MC3433_SUPPORT_LPF_)
        atomic_t                firlen;
        atomic_t                fir_en;
        struct data_filter      fir;
    #endif 

    //================================================
    #if defined(CONFIG_HAS_EARLYSUSPEND)
        struct early_suspend    early_drv;
    #endif     
};

#ifdef _MC3433_SUPPORT_LRF_
    typedef struct
    {
        s16    nIsNewRound;
        s16    nPreDiff;
        s16    nPreValue;
        s16    nMaxValue;
        s16    nMinValue;
        s16    nRepValue;
        s16    nNewDataMonitorCount;
    }   S_LRF_CB;
#endif

/*****************************************************************************
 *** EXTERNAL FUNCTION
 *****************************************************************************/
extern struct acc_hw* get_cust_acc_hw(void);


//static int _MC3433_i2c_auto_probe(struct i2c_client *client);

#ifndef CONFIG_HAS_EARLYSUSPEND
    static int MC3433_suspend(struct i2c_client *client, pm_message_t msg);
    static int MC3433_resume(struct i2c_client *client);
#endif

#ifdef MTK_AUTO_DETECT_ACCELEROMETER
    static int MC3433_local_init(void);
    static int MC3433_remove(struct platform_device *pdev);
    static struct sensor_init_info  MC3433_init_info;
#else
    static struct platform_driver MC3433_gsensor_driver;
#endif

static int MC3433_SetPowerMode(struct i2c_client *client, bool enable);
static int MC3433_WriteCalibration(struct i2c_client *client, int dat[MC3433_AXES_NUM]);

static void MC3433_SetGain(void);

/*****************************************************************************
 *** STATIC VARIBLE & CONTROL BLOCK DECLARATION
 *****************************************************************************/
static unsigned char    s_bResolution = 0x00;
static unsigned char    s_bPCODE      = 0x00;

static int    s_nInitFlag = MC3433_INIT_FAIL;

//static       unsigned short                      MC3433_i2c_auto_probe_addr[] = { 0x4C, 0x6C, 0x4E, 0x6D, 0x6E, 0x6F };

static struct i2c_driver    MC3433_i2c_driver = {
	.driver = {
		.name = MC3433_DEV_NAME,
		},
	.probe  = MC3433_i2c_probe,
	.remove = MC3433_i2c_remove,
	.detect	= MC3433_i2c_detect,

	#if !defined(CONFIG_HAS_EARLYSUSPEND)    
	.suspend = MC3433_suspend,
	.resume  = MC3433_resume,
	#endif

	.id_table = MC3433_i2c_id,
	};

static struct i2c_client        *MC3433_i2c_client = NULL;
extern int gsensor_init_status;
static struct MC3433_i2c_data   *MC3433_obj_i2c_data = NULL;

static struct data_resolution    MC3433_data_resolution[1] = {
	{ {7, 8}, 256 }
};

static struct data_resolution    MC3433_offset_resolution = { {7, 8}, 256 };


static GSENSOR_VECTOR3D    gsensor_gain, gsensor_offset;

static char    selftestRes[10] = {0};

static struct file* fd_file = NULL;
static mm_segment_t oldfs;
static unsigned char offset_buf[6]; 
static signed int    offset_data[3];
static signed int    gain_data[3];
static unsigned char    s_baOTP_OffsetData[6] = { 0 };
static signed int    s_nIsRBM_Enabled = false;

#ifdef _MC3433_SUPPORT_LRF_
    static S_LRF_CB    s_taLRF_CB[MC3433_AXES_NUM];
#endif

#ifdef _MC3433_SUPPORT_CONCURRENCY_PROTECTION_
    static struct semaphore    s_tSemaProtect;
#endif

#ifdef _MC3433_SUPPORT_DOT_CALIBRATION_
    static int    s_nIsCaliLoaded = false;
    static int    LPF_FirstRun = 1;
#endif

static int             LPF_SamplingRate = 5;
static int             LPF_CutoffFrequency = 0x00000004;
static unsigned int    iAReal0_X;
static unsigned int    iAcc0Lpf0_X;
static unsigned int    iAcc0Lpf1_X;
static unsigned int    iAcc1Lpf0_X;
static unsigned int    iAcc1Lpf1_X;

static unsigned int    iAReal0_Y;
static unsigned int    iAcc0Lpf0_Y;
static unsigned int    iAcc0Lpf1_Y;
static unsigned int    iAcc1Lpf0_Y;
static unsigned int    iAcc1Lpf1_Y;

static unsigned int    iAReal0_Z;
static unsigned int    iAcc0Lpf0_Z;
static unsigned int    iAcc0Lpf1_Z;
static unsigned int    iAcc1Lpf0_Z;
static unsigned int    iAcc1Lpf1_Z;

static signed char    s_bAccuracyStatus = SENSOR_STATUS_ACCURACY_MEDIUM;

#ifdef _MC3433_SUPPORT_PERIODIC_DOC_
    static DECLARE_WAIT_QUEUE_HEAD(wq_MC3433_open_status);

    static atomic_t    s_t_MC3433_open_status = ATOMIC_INIT(0);

    static unsigned char    s_bIsPDOC_Enabled = false;
#endif

#ifdef _MC3433_SUPPORT_VPROXIMITY_SENSOR_
    static int    P_STATUS;
    static int    prev_P_STATUS = 0;
#endif

/*****************************************************************************
 *** MACRO
 *****************************************************************************/
#ifdef _MC3433_SUPPORT_CONCURRENCY_PROTECTION_
    #define MC3433_MUTEX_INIT()   sema_init(&s_tSemaProtect, 1)

    #define MC3433_MUTEX_LOCK()                             \
                if (down_interruptible(&s_tSemaProtect))    \
                    return (-ERESTARTSYS)

    #define MC3433_MUTEX_LOCK_RETURN_VOID()                 \
                if (down_interruptible(&s_tSemaProtect))    \
                    return

    #define MC3433_MUTEX_UNLOCK()   up(&s_tSemaProtect)
#else
    #define MC3433_MUTEX_INIT()                do {} while (0)
    #define MC3433_MUTEX_LOCK()                do {} while (0)
    #define MC3433_MUTEX_LOCK_RETURN_VOID()    do {} while (0)
    #define MC3433_MUTEX_UNLOCK()              do {} while (0)
#endif

#define REMAP_IF_MC3250_READ(nDataX, nDataY)                        \
            if (MC3433_PCODE_3250 == s_bPCODE)                      \
            {                                                       \
                int    _nTemp = 0;                                  \
                                                                    \
                _nTemp = nDataX;                                    \
                nDataX = nDataY;                                    \
                nDataY = -_nTemp;                                   \
                GSE_LOG("[%s] 3250 read remap\n", __FUNCTION__);    \
            }

#define REMAP_IF_MC3250_WRITE(nDataX, nDataY)                        \
            if (MC3433_PCODE_3250 == s_bPCODE)                       \
            {                                                        \
                int    _nTemp = 0;                                   \
                                                                     \
                _nTemp = nDataX;                                     \
                nDataX = -nDataY;                                    \
                nDataY = _nTemp;                                     \
                GSE_LOG("[%s] 3250 write remap\n", __FUNCTION__);    \
            }

#define REMAP_IF_MC34XX_N(nDataX, nDataY)                                                \
            if ((MC3433_PCODE_3410N == s_bPCODE) || (MC3433_PCODE_3430N == s_bPCODE))    \
            {                                                                            \
                nDataX = -nDataX;                                                        \
                nDataY = -nDataY;                                                        \
                GSE_LOG("[%s] 34X0N remap\n", __FUNCTION__);                             \
            }

#define REMAP_IF_MC35XX(nDataX, nDataY)                                                                                                                          \
            if ((MC3433_PCODE_3510B == s_bPCODE) || (MC3433_PCODE_3510C == s_bPCODE) || (MC3433_PCODE_3530B == s_bPCODE) || (MC3433_PCODE_3530C == s_bPCODE))    \
            {                                                                                                                                                    \
                nDataX = -nDataX;                                                                                                                                \
                nDataY = -nDataY;                                                                                                                                \
                GSE_LOG("[%s] 35X0 remap\n", __FUNCTION__);                                                                                                      \
            }

#define IS_MC35XX()    ((MC3433_PCODE_3510B == s_bPCODE) || (MC3433_PCODE_3510C == s_bPCODE) || (MC3433_PCODE_3530B == s_bPCODE) || (MC3433_PCODE_3530C == s_bPCODE))

/*****************************************************************************
 *** TODO
 *****************************************************************************/
#define DATA_PATH              "/sdcard2/mcube-register-map.txt"

#ifdef _MC3433_SUPPORT_DOT_CALIBRATION_
    static char    file_path[MC3433_BUF_SIZE]        = "/data/data/com.mcube.acc/files/mcube-calib.txt";
    static char    backup_file_path[MC3433_BUF_SIZE] = "/data/misc/sensors/mcube-calib.txt";
#endif

/*****************************************************************************
 *** FUNCTION
 *****************************************************************************/

/*****************************************
 *** GetLowPassFilter
 *****************************************/
static unsigned int GetLowPassFilter(unsigned int X0,unsigned int Y1)
{
    unsigned int lTemp;
    
    lTemp = Y1;
    lTemp *= LPF_CutoffFrequency;    // 4HZ LPF RC=0.04
    X0 *= LPF_SamplingRate;		
    lTemp += X0;
    lTemp += LPF_CutoffFrequency;
    lTemp /= (LPF_CutoffFrequency + LPF_SamplingRate);		
    Y1 = lTemp;
    
    return Y1;
}

/*****************************************
 *** openFile
 *****************************************/
static struct file *openFile(char *path,int flag,int mode) 
{ 
    struct file *fp = NULL; 
    
    fp = filp_open(path, flag, mode); 

    if (IS_ERR(fp) || !fp->f_op) 
    {
        GSE_LOG("Calibration File filp_open return NULL\n");
        return NULL; 
    }
    else 
    {
        return fp; 
    }
} 
 
/*****************************************
 *** readFile
 *****************************************/
#ifdef _MC3433_SUPPORT_DOT_CALIBRATION_
static int readFile(struct file *fp,char *buf,int readlen) 
{ 
    if (fp->f_op && fp->f_op->read) 
        return fp->f_op->read(fp,buf,readlen, &fp->f_pos); 
    else 
        return -1; 
} 
#endif

/*****************************************
 *** writeFile
 *****************************************/
static int writeFile(struct file *fp,char *buf,int writelen) 
{ 
    if (fp->f_op && fp->f_op->write) 
        return fp->f_op->write(fp,buf,writelen, &fp->f_pos); 
    else 
        return -1; 
}
 
/*****************************************
 *** closeFile
 *****************************************/
static int closeFile(struct file *fp) 
{ 
    filp_close(fp,NULL); 
    return 0; 
} 

/*****************************************
 *** initKernelEnv
 *****************************************/
static void initKernelEnv(void) 
{ 
    oldfs = get_fs(); 
    set_fs(KERNEL_DS);
    printk(KERN_INFO "initKernelEnv\n");
} 

/*****************************************
 *** mcube_write_log_data
 *****************************************/
static int mcube_write_log_data(struct i2c_client *client, u8 data[0x3f])
{
    #define _WRT_LOG_DATA_BUFFER_SIZE    (66 * 50)
    
    s16 rbm_data[3] = {0}, raw_data[3] = {0};
    int err = 0;
    char *_pszBuffer = NULL;
    int n = 0, i = 0;

    initKernelEnv();
    fd_file = openFile(DATA_PATH ,O_RDWR | O_CREAT,0); 
    if (fd_file == NULL) 
    {
        GSE_LOG("mcube_write_log_data fail to open\n");	
    }
    else
    {
        rbm_data[MC3433_AXIS_X] = (s16)((data[0x0d]) | (data[0x0e] << 8));
        rbm_data[MC3433_AXIS_Y] = (s16)((data[0x0f]) | (data[0x10] << 8));
        rbm_data[MC3433_AXIS_Z] = (s16)((data[0x11]) | (data[0x12] << 8));
        
        raw_data[MC3433_AXIS_X] = (rbm_data[MC3433_AXIS_X] + offset_data[0]/2)*gsensor_gain.x/gain_data[0];
        raw_data[MC3433_AXIS_Y] = (rbm_data[MC3433_AXIS_Y] + offset_data[1]/2)*gsensor_gain.y/gain_data[1];
        raw_data[MC3433_AXIS_Z] = (rbm_data[MC3433_AXIS_Z] + offset_data[2]/2)*gsensor_gain.z/gain_data[2];
        
        _pszBuffer = kzalloc(_WRT_LOG_DATA_BUFFER_SIZE, GFP_KERNEL);

        if (NULL == _pszBuffer)
        {
            GSE_ERR("fail to allocate memory for buffer\n");
            return -1;
        }

        memset(_pszBuffer, 0, _WRT_LOG_DATA_BUFFER_SIZE); 
        
        n += sprintf(_pszBuffer+n, "G-sensor RAW X = %d  Y = %d  Z = %d\n", raw_data[0] ,raw_data[1] ,raw_data[2]);
        n += sprintf(_pszBuffer+n, "G-sensor RBM X = %d  Y = %d  Z = %d\n", rbm_data[0] ,rbm_data[1] ,rbm_data[2]);

        for (i = 0; i < 64; i++)
        {
            n += sprintf(_pszBuffer+n, "mCube register map Register[%x] = 0x%x\n",i,data[i]);
        }

        msleep(50);		

        if ((err = writeFile(fd_file,_pszBuffer,n)) > 0) 
            GSE_LOG("buf:%s\n",_pszBuffer); 
        else 
            GSE_LOG("write file error %d\n",err); 
        
        kfree(_pszBuffer);
        set_fs(oldfs); 
        closeFile(fd_file); 
    }

    return 0;
}

#ifdef _MC3433_SUPPORT_DOT_CALIBRATION_
/*****************************************
 *** mcube_read_cali_file
 *****************************************/
static int mcube_read_cali_file(struct i2c_client *client)
{
    int cali_data[3] = {0},cali_data1[3] = {0};
    int err = 0;
    char buf[64] = {0};

    GSE_LOG("[%s] sizeof(buf): %d\n",__func__, sizeof(buf));

    initKernelEnv();
    fd_file = openFile(file_path,O_RDONLY,0); 

    if (fd_file == NULL) 
    {
        GSE_LOG("%s:fail to open calibration file: %s\n", __func__, file_path);
        fd_file = openFile(backup_file_path,O_RDONLY,0); 

        if(fd_file == NULL)	
        {	
            GSE_LOG("%s:fail to open calibration file: %s\n", __func__, backup_file_path);
            cali_data[0] = 0;
            cali_data[1] = 0;
            cali_data[2] = 0;
            return (-1);
        }
        else
        {		
            GSE_LOG("Open backup calibration file successfully: %s\n", backup_file_path);
        }
    }
    else
    {
        GSE_LOG("Open calibration file successfully: %s\n", file_path);
    }
			
    memset(buf, 0, sizeof(buf)); 

    if ((err = readFile(fd_file, buf, sizeof(buf))) > 0) 
        GSE_LOG("cali_file: buf:%s\n",buf); 
    else 
        GSE_LOG("read file error %d\n",err); 
    
    set_fs(oldfs); 
    closeFile(fd_file); 

    sscanf(buf, "%d %d %d",&cali_data[MC3433_AXIS_X], &cali_data[MC3433_AXIS_Y], &cali_data[MC3433_AXIS_Z]);
    GSE_LOG("cali_data: %d %d %d\n", cali_data[MC3433_AXIS_X], cali_data[MC3433_AXIS_Y], cali_data[MC3433_AXIS_Z]); 	
    
    cali_data1[MC3433_AXIS_X] = cali_data[MC3433_AXIS_X] * gsensor_gain.x / GRAVITY_EARTH_1000;
    cali_data1[MC3433_AXIS_Y] = cali_data[MC3433_AXIS_Y] * gsensor_gain.y / GRAVITY_EARTH_1000;
    cali_data1[MC3433_AXIS_Z] = cali_data[MC3433_AXIS_Z] * gsensor_gain.z / GRAVITY_EARTH_1000;
    
    GSE_LOG("cali_data1: %d %d %d\n", cali_data1[MC3433_AXIS_X], cali_data1[MC3433_AXIS_Y], cali_data1[MC3433_AXIS_Z]); 	
    
    MC3433_WriteCalibration(client, cali_data1);
    
    return 0;
}

/*****************************************
 *** mcube_load_cali
 *****************************************/
static void    mcube_load_cali(struct i2c_client *pt_i2c_client)
{
    if (false == s_nIsCaliLoaded)
    {
        GSE_LOG("[%s] loading cali file...\n", __FUNCTION__);
        
        if (MC3433_RETCODE_SUCCESS == mcube_read_cali_file(pt_i2c_client))
            s_nIsCaliLoaded = true;
    }
}

#endif // _MC3433_SUPPORT_DOT_CALIBRATION_

/*****************************************
 *** hwmsen_read_byte_sr
 *****************************************/
static int hwmsen_read_byte_sr(struct i2c_client *client, u8 addr, u8 *data)
{
    u8 buf = 0;
    int ret = 0;
    
    client->addr = ((client->addr) & (I2C_MASK_FLAG | I2C_WR_FLAG |I2C_RS_FLAG));
    buf = addr;
    ret = i2c_master_send(client, (const char*)&buf, 1<<8 | 1);

    if (ret < 0) {
        HWM_ERR("send command error!!\n");
        return -EFAULT;
    }
    
    *data = buf;
    client->addr = client->addr& I2C_MASK_FLAG;

    return 0;
}

/*****************************************
 *** mcube_psensor_ioctl
 *****************************************/
#ifdef _MC3433_SUPPORT_VPROXIMITY_SENSOR_
static long mcube_psensor_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{ 
    int err=0;
    void __user *data;
    int pSensor=0;
    
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
		PS_ERR("access error: %08X, (%2d, %2d)\n", cmd, _IOC_DIR(cmd), _IOC_SIZE(cmd));
		return -EFAULT;
	}

	switch(cmd)
	{
		case PSENSOR_IOCTL_SET_POSTURE:
			data = (void __user*)arg;
			if(data == NULL)
			{
				err = -EINVAL;
				break;	  
			}
			
			if(copy_from_user(&pSensor, data, sizeof(int)))
			{
				err = -EFAULT;
				break;	  
			}
			
			P_STATUS = pSensor;
			PS_LOG("IOCTL Get P_STATUS = %d",pSensor);		
			break;
		default:
			
			PS_ERR("unknown IOCTL: 0x%08x\n", cmd);
			//err = -ENOIOCTLCMD;
			break;	
	}       
	return err;
}

/*****************************************
 *** STATIC STRUCTURE:: fops
 *****************************************/
static struct file_operations mcube_psensor_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = mcube_psensor_ioctl,
};

/*****************************************
 *** STATIC STRUCTURE:: misc-device
 *****************************************/
static struct miscdevice mcube_psensor_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "psensor",
	.fops  = &mcube_psensor_fops,
};

/*****************************************
 *** psensor_ps_operate
 *****************************************/
static int psensor_ps_operate(void* self, uint32_t command, void* buff_in, int size_in,
		void* buff_out /*sensor_data*/, int size_out, int* actualout)
{
	int err = 0;
	int value;
	int val=-1;
	hwm_sensor_data* sensor_data;
	//struct mcube_psensor_priv *obj = (struct mcube_psensor_priv *)self;
	
	//PS_FUN(f);
	switch (command)
	{
		case SENSOR_DELAY:
			if((buff_in == NULL) || (size_in < sizeof(int)))
			{
				PS_ERR("Set delay parameter error!\n");
				err = -EINVAL;
			}
			// Do nothing
			break;

		case SENSOR_ENABLE:
			value = *(int *)buff_in;
			if(value)
			{
				if((buff_in == NULL) || (size_in < sizeof(int)))
				{
					PS_ERR("Enable sensor parameter error!\n");
					err = -EINVAL;
				}
				
				else
				{				
					value = *(int *)buff_in;
					//obj->enable = 1;
					PS_ERR("enable ps \n"); 
				}
			}
			else
			{
				//obj->enable = 0;
				PS_ERR("disable ps \n"); 
			}
			break;
		case SENSOR_GET_DATA:
			PS_LOG("fwq get ps data !!!!!!\n");
			if((buff_out == NULL) || (size_out< sizeof(hwm_sensor_data)))
			{
				PS_ERR("get sensor data parameter error!\n");
				err = -EINVAL;
			}
			else
			{
				sensor_data = (hwm_sensor_data *)buff_out;				
				PS_LOG("mcube sensor data P_STATUS=%d\n",P_STATUS);
				
				if(prev_P_STATUS != P_STATUS )
				{
					if(P_STATUS == 0)
					{
						PS_LOG("TURN ON LCD\n");
						val=1;
						PS_LOG("Set val = 1, Proximity sensor far away\n");
					}
					else if (P_STATUS == 1)
					{	
						PS_LOG("TURN OFF LCD\n");
						val=0;
						PS_LOG("Set val = 0, Proximity sensor close\n");
					}
					PS_LOG("mcube sensor data prev_P_STATUS=%d, P_STATUS=%d\n",prev_P_STATUS,P_STATUS);
					prev_P_STATUS = P_STATUS;	
				}
				else
				{
					PS_LOG("P_STATUS %5d=>%5d\n",prev_P_STATUS,P_STATUS);
					prev_P_STATUS = P_STATUS;
					val= (P_STATUS==0)? (1):(0);
				}	
				sensor_data->values[0] = val;	
				sensor_data->value_divide = 1;
				sensor_data->status = SENSOR_STATUS_ACCURACY_MEDIUM;
				prev_P_STATUS = P_STATUS;	
			}
			break;
		default:
			PS_ERR("proxmy sensor operate function has no this command %d!\n", command);
			err = -1;
			break;
	}
	
	return err;
}
#endif  // end of _MC3433_SUPPORT_VPROXIMITY_SENSOR_

/*****************************************
 *** MC3433_power
 *****************************************/
static void MC3433_power(struct acc_hw *hw, unsigned int on) 
{
	static unsigned int power_on = 0;

	if(hw->power_id != MT65XX_POWER_NONE)		// have externel LDO
	{        
		GSE_LOG("power %s\n", on ? "on" : "off");
		if(power_on == on)	// power status not change
		{
			GSE_LOG("ignore power control: %d\n", on);
		}
		else if(on)	// power on
		{
			if(!hwPowerOn(hw->power_id, hw->power_vol, MC3433_DEV_NAME))
			{
				GSE_ERR("power on fails!!\n");
			}
		}
		else	// power off
		{
			if (!hwPowerDown(hw->power_id, "MC3433"))
			{
				GSE_ERR("power off fail!!\n");
			}			  
		}
	}
	power_on = on;    
}

#ifdef _MC3433_SUPPORT_DOT_CALIBRATION_
/*****************************************
 *** MC3433_rbm
 *****************************************/
static void MC3433_rbm(struct i2c_client *client, int enable)
{
    u8    _baDataBuf[3] = { 0 };
    
    _baDataBuf[0] = 0x43; 
    hwmsen_write_block(client, 0x07, _baDataBuf, 0x01);
    
    hwmsen_read_block(client, 0x04, _baDataBuf, 0x01);
    
    //GSE_LOG("[%s] REG(0x04): 0x%X, enable: %d\n", __FUNCTION__, _baDataBuf[0], enable);
    
    if (0x00 == (_baDataBuf[0] & 0x40))
    {
        _baDataBuf[0] = 0x6D; 
        hwmsen_write_block(client, 0x1B, _baDataBuf, 0x01);

        _baDataBuf[0] = 0x43; 
        hwmsen_write_block(client, 0x1B, _baDataBuf, 0x01);
    }
    
    //hwmsen_read_block(client, 0x04, _baDataBuf, 0x01);
    //GSE_LOG("BEGIN - REG(0x04): 0x%X\n", _baDataBuf[0]);
    
    if (1 == enable)
    {
        _baDataBuf[0] = 0x00;
        hwmsen_write_block(client, 0x3B, _baDataBuf, 0x01);
        
        _baDataBuf[0] = 0x02; 
        hwmsen_write_block(client, 0x14, _baDataBuf, 0x01);
        
        if (MC3433_RESOLUTION_LOW == s_bResolution)
        {
            gsensor_gain.x = gsensor_gain.y = gsensor_gain.z = 1024;
        }

        s_nIsRBM_Enabled = 1;
        LPF_FirstRun = 1;
        
        GSE_LOG("set rbm!!\n");
    }
    else if (0 == enable)
    {
        _baDataBuf[0] = 0x00; 
        hwmsen_write_block(client, 0x14, _baDataBuf, 0x01);
        
        _baDataBuf[0] = s_bPCODE; 
        hwmsen_write_block(client, 0x3B, _baDataBuf, 0x01);
        
        MC3433_SetGain();

        s_nIsRBM_Enabled = 0;
        
        GSE_LOG("clear rbm!!\n");
    }
    
    hwmsen_read_block(client, 0x04, _baDataBuf, 0x01);
    
    //GSE_LOG("RBM CONTROL DONE - REG(0x04): 0x%X\n", _baDataBuf[0]);
    
    if (_baDataBuf[0] & 0x40)
    {
        _baDataBuf[0] = 0x6D; 
        hwmsen_write_block(client, 0x1B, _baDataBuf, 0x01);
        
        _baDataBuf[0] = 0x43; 
        hwmsen_write_block(client, 0x1B, _baDataBuf, 0x01);
    }
    
    //hwmsen_read_block(client, 0x04, _baDataBuf, 0x01);
    //GSE_LOG("END - REG(0x04): 0x%X\n", _baDataBuf[0]);
    
    _baDataBuf[0] = 0x41; 
    hwmsen_write_block(client, 0x07, _baDataBuf, 0x01);
    
    msleep(220);
}

/*****************************************
 *** MC3433_ReadData_RBM
 *****************************************/
static int MC3433_ReadData_RBM(struct i2c_client *client, int data[MC3433_AXES_NUM])
{
	u8 addr = 0x0d;
	u8 rbm_buf[MC3433_DATA_LEN] = {0};
	int err = 0;

	if(NULL == client)
	{
		err = -EINVAL;
		return err;
	}
	
/********************************************/
	
	if((err = hwmsen_read_block(client, addr, rbm_buf, 0x06)))
    {
    	GSE_ERR("error: %d\n", err);
    	return err;
    }

	data[MC3433_AXIS_X] = (s16)((rbm_buf[0]) | (rbm_buf[1] << 8));
	data[MC3433_AXIS_Y] = (s16)((rbm_buf[2]) | (rbm_buf[3] << 8));
	data[MC3433_AXIS_Z] = (s16)((rbm_buf[4]) | (rbm_buf[5] << 8));

	GSE_LOG("rbm_buf<<<<<[%02x %02x %02x %02x %02x %02x]\n",rbm_buf[0], rbm_buf[2], rbm_buf[2], rbm_buf[3], rbm_buf[4], rbm_buf[5]);
	GSE_LOG("RBM<<<<<[%04x %04x %04x]\n", data[MC3433_AXIS_X], data[MC3433_AXIS_Y], data[MC3433_AXIS_Z]);
	GSE_LOG("RBM<<<<<[%04d %04d %04d]\n", data[MC3433_AXIS_X], data[MC3433_AXIS_Y], data[MC3433_AXIS_Z]);


/********************************************/
    REMAP_IF_MC3250_READ(data[0], data[1]);
    REMAP_IF_MC34XX_N(data[0], data[1]);
    REMAP_IF_MC35XX(data[0], data[1]);

	return err;
}
#endif // _MC3433_SUPPORT_DOT_CALIBRATION_

/*****************************************
 *** MC3433_ValidateSensorIC
 *****************************************/
static int MC3433_ValidateSensorIC(unsigned char bPCode)
{
    unsigned char    _baOurSensorList[] = {
	MC3433_PCODE_3210 , MC3433_PCODE_3230 ,
	MC3433_PCODE_3250 ,
	MC3433_PCODE_3410 , MC3433_PCODE_3430 ,
	MC3433_PCODE_3410N, MC3433_PCODE_3430N,
	MC3433_PCODE_3510B, MC3433_PCODE_3530B,
	MC3433_PCODE_3510C, MC3433_PCODE_3530C
	};

    int    _nSensorCount = (sizeof(_baOurSensorList) / sizeof(_baOurSensorList[0]));
    int    _nCheckIndex  = 0;

    GSE_ERR("[%s] code to be verified: 0x%X, _nSensorCount: %d\n", __FUNCTION__, bPCode, _nSensorCount);

    for (_nCheckIndex = 0; _nCheckIndex < _nSensorCount; _nCheckIndex++)
    {
    //modify by xiaopu.zhu for ingore check id error
        if (_baOurSensorList[_nCheckIndex] == bPCode) 
	{
            return (MC3433_RETCODE_SUCCESS);
	}
	else if(_baOurSensorList[_nCheckIndex] == (bPCode & 0xF1))
	{
	    return (MC3433_RETCODE_SUCCESS);
	}
    }
//modify by xiaopu.zhu for ingore check id error
    return (MC3433_RETCODE_ERROR_IDENTIFICATION);
}

/*****************************************
 *** MC3433_Read_Chip_ID
 *****************************************/
static int MC3433_Read_Chip_ID(struct i2c_client *client, char *buf)
{
    u8     _bChipID[4] = { 0 };

    GSE_LOG("[%s]\n", __func__);

    if (!buf || !client)
        return EINVAL;

    if (hwmsen_read_block(client, 0x3C, _bChipID, 4))
    {
        GSE_ERR("[%s] i2c read fail\n", __func__);
        _bChipID[0] = 0;
        _bChipID[1] = 0;
        _bChipID[2] = 0;
        _bChipID[3] = 0;
    }

    GSE_LOG("[%s] %02X-%02X-%02X-%02X\n", __func__, _bChipID[3], _bChipID[2], _bChipID[1], _bChipID[0]);

    return sprintf(buf, "%02X-%02X-%02X-%02X\n", _bChipID[3], _bChipID[2], _bChipID[1], _bChipID[0]);
}

/*****************************************
 *** MC3433_Read_Reg_Map
 *****************************************/
static int MC3433_Read_Reg_Map(struct i2c_client *p_i2c_client, u8 *pbUserBuf)
{
    u8     _baData[MC3433_REGMAP_LENGTH] = { 0 };
    int    _nIndex = 0;

    GSE_LOG("[%s]\n", __func__);

    if(NULL == p_i2c_client)
        return (-EINVAL);

    for(_nIndex = 0; _nIndex < MC3433_REGMAP_LENGTH; _nIndex++)
    {
        hwmsen_read_block(p_i2c_client, _nIndex, &_baData[_nIndex], 1);

        if (NULL != pbUserBuf)
            pbUserBuf[_nIndex] = _baData[_nIndex];

        printk(KERN_INFO "[Gsensor] REG[0x%02X] = 0x%02X\n", _nIndex, _baData[_nIndex]);
    }

    mcube_write_log_data(p_i2c_client, _baData);

    return (0);
}

/*****************************************
 *** MC3433_SaveDefaultOffset
 *****************************************/
static void MC3433_SaveDefaultOffset(struct i2c_client *p_i2c_client)
{
    GSE_LOG("[%s]\n", __func__);

    hwmsen_read_block(p_i2c_client, 0x21, &s_baOTP_OffsetData[0], 3);
    hwmsen_read_block(p_i2c_client, 0x24, &s_baOTP_OffsetData[3], 3);

    GSE_LOG("s_baOTP_OffsetData: 0x%02X - 0x%02X - 0x%02X - 0x%02X - 0x%02X - 0x%02X\n",
            s_baOTP_OffsetData[0], s_baOTP_OffsetData[1], s_baOTP_OffsetData[2],
            s_baOTP_OffsetData[3], s_baOTP_OffsetData[4], s_baOTP_OffsetData[5]);
}

/*****************************************
 *** MC3433_LPF
 *****************************************/
#ifdef _MC3433_SUPPORT_LPF_
static void MC3433_LPF(struct MC3433_i2c_data *priv, s16 data[MC3433_AXES_NUM])
{
    if(atomic_read(&priv->filter))
    {
        if(atomic_read(&priv->fir_en) && !atomic_read(&priv->suspend))
        {
            int idx, firlen = atomic_read(&priv->firlen);   
            if(priv->fir.num < firlen)
            {                
                priv->fir.raw[priv->fir.num][MC3433_AXIS_X] = data[MC3433_AXIS_X];
                priv->fir.raw[priv->fir.num][MC3433_AXIS_Y] = data[MC3433_AXIS_Y];
                priv->fir.raw[priv->fir.num][MC3433_AXIS_Z] = data[MC3433_AXIS_Z];
                priv->fir.sum[MC3433_AXIS_X] += data[MC3433_AXIS_X];
                priv->fir.sum[MC3433_AXIS_Y] += data[MC3433_AXIS_Y];
                priv->fir.sum[MC3433_AXIS_Z] += data[MC3433_AXIS_Z];
                if(atomic_read(&priv->trace) & MCUBE_TRC_FILTER)
                {
                    GSE_LOG("add [%2d] [%5d %5d %5d] => [%5d %5d %5d]\n", priv->fir.num,
                    priv->fir.raw[priv->fir.num][MC3433_AXIS_X], priv->fir.raw[priv->fir.num][MC3433_AXIS_Y], priv->fir.raw[priv->fir.num][MC3433_AXIS_Z],
                    priv->fir.sum[MC3433_AXIS_X], priv->fir.sum[MC3433_AXIS_Y], priv->fir.sum[MC3433_AXIS_Z]);
                }
                priv->fir.num++;
                priv->fir.idx++;
            }
            else
            {
                idx = priv->fir.idx % firlen;
                priv->fir.sum[MC3433_AXIS_X] -= priv->fir.raw[idx][MC3433_AXIS_X];
                priv->fir.sum[MC3433_AXIS_Y] -= priv->fir.raw[idx][MC3433_AXIS_Y];
                priv->fir.sum[MC3433_AXIS_Z] -= priv->fir.raw[idx][MC3433_AXIS_Z];
                priv->fir.raw[idx][MC3433_AXIS_X] = data[MC3433_AXIS_X];
                priv->fir.raw[idx][MC3433_AXIS_Y] = data[MC3433_AXIS_Y];
                priv->fir.raw[idx][MC3433_AXIS_Z] = data[MC3433_AXIS_Z];
                priv->fir.sum[MC3433_AXIS_X] += data[MC3433_AXIS_X];
                priv->fir.sum[MC3433_AXIS_Y] += data[MC3433_AXIS_Y];
                priv->fir.sum[MC3433_AXIS_Z] += data[MC3433_AXIS_Z];
                priv->fir.idx++;
                data[MC3433_AXIS_X] = priv->fir.sum[MC3433_AXIS_X]/firlen;
                data[MC3433_AXIS_Y] = priv->fir.sum[MC3433_AXIS_Y]/firlen;
                data[MC3433_AXIS_Z] = priv->fir.sum[MC3433_AXIS_Z]/firlen;
                if(atomic_read(&priv->trace) & MCUBE_TRC_FILTER)
                {
                    GSE_LOG("add [%2d] [%5d %5d %5d] => [%5d %5d %5d] : [%5d %5d %5d]\n", idx,
                    priv->fir.raw[idx][MC3433_AXIS_X], priv->fir.raw[idx][MC3433_AXIS_Y], priv->fir.raw[idx][MC3433_AXIS_Z],
                    priv->fir.sum[MC3433_AXIS_X], priv->fir.sum[MC3433_AXIS_Y], priv->fir.sum[MC3433_AXIS_Z],
                    data[MC3433_AXIS_X], data[MC3433_AXIS_Y], data[MC3433_AXIS_Z]);
                }
            }
        }
    }	
}
#endif    // END OF #ifdef _MC3433_SUPPORT_LPF_

#ifdef _MC3433_SUPPORT_LRF_
/*****************************************
 *** _MC3433_LowResFilter
 *****************************************/
static void _MC3433_LowResFilter(s16 nAxis, s16 naData[MC3433_AXES_NUM])
{
    #define _LRF_DIFF_COUNT_POS                  2
    #define _LRF_DIFF_COUNT_NEG                  (-_LRF_DIFF_COUNT_POS)
    #define _LRF_DIFF_BOUNDARY_POS               (_LRF_DIFF_COUNT_POS + 1)
    #define _LRF_DIFF_BOUNDARY_NEG               (_LRF_DIFF_COUNT_NEG - 1)
    #define _LRF_DIFF_DATA_UNCHANGE_MAX_COUNT    11
    
    signed int    _nCurrDiff = 0;
    signed int    _nSumDiff  = 0;
    s16           _nCurrData = naData[nAxis];
    
    _nCurrDiff = (_nCurrData - s_taLRF_CB[nAxis].nRepValue);
    
    if ((_LRF_DIFF_COUNT_NEG < _nCurrDiff) && (_nCurrDiff < _LRF_DIFF_COUNT_POS))
    {
        if (s_taLRF_CB[nAxis].nIsNewRound)
        {
            s_taLRF_CB[nAxis].nMaxValue = _nCurrData;
            s_taLRF_CB[nAxis].nMinValue = _nCurrData;
            
            s_taLRF_CB[nAxis].nIsNewRound = 0;
            s_taLRF_CB[nAxis].nNewDataMonitorCount = 0;
        }
        else
        {
            if (_nCurrData > s_taLRF_CB[nAxis].nMaxValue)
                s_taLRF_CB[nAxis].nMaxValue = _nCurrData;
            else if (_nCurrData < s_taLRF_CB[nAxis].nMinValue)
                s_taLRF_CB[nAxis].nMinValue = _nCurrData;
            
            if (s_taLRF_CB[nAxis].nMinValue != s_taLRF_CB[nAxis].nMaxValue)
            {
                if (_nCurrData == s_taLRF_CB[nAxis].nPreValue)
                    s_taLRF_CB[nAxis].nNewDataMonitorCount++;
                else
                    s_taLRF_CB[nAxis].nNewDataMonitorCount = 0;
            }
        }
        
        if (1 != (s_taLRF_CB[nAxis].nMaxValue - s_taLRF_CB[nAxis].nMinValue))
            s_taLRF_CB[nAxis].nRepValue = ((s_taLRF_CB[nAxis].nMaxValue + s_taLRF_CB[nAxis].nMinValue) / 2);
        
        _nSumDiff = (_nCurrDiff + s_taLRF_CB[nAxis].nPreDiff);
        
        if (_nCurrDiff)
            s_taLRF_CB[nAxis].nPreDiff = _nCurrDiff;
        
        if ((_LRF_DIFF_BOUNDARY_NEG < _nSumDiff) && (_nSumDiff < _LRF_DIFF_BOUNDARY_POS))
        {
            if (_LRF_DIFF_DATA_UNCHANGE_MAX_COUNT > s_taLRF_CB[nAxis].nNewDataMonitorCount)
            {
                naData[nAxis] = s_taLRF_CB[nAxis].nRepValue;
                goto _LRF_RETURN;
            }
        }
    }
    
    s_taLRF_CB[nAxis].nRepValue   = _nCurrData;
    s_taLRF_CB[nAxis].nPreDiff    = 0;
    s_taLRF_CB[nAxis].nIsNewRound = 1;
    
_LRF_RETURN:
    
    GSE_LOG(">>>>> [_MC3433_LowResFilter][%d] _nCurrDiff: %4d       _nSumDiff: %4d         _nCurrData:    %4d         Rep:    %4d\n", nAxis, _nCurrDiff, _nSumDiff, _nCurrData, s_taLRF_CB[nAxis].nRepValue);
    
    s_taLRF_CB[nAxis].nPreValue = _nCurrData;
    
    #undef _LRF_DIFF_COUNT_POS
    #undef _LRF_DIFF_COUNT_NEG
    #undef _LRF_DIFF_BOUNDARY_POS
    #undef _LRF_DIFF_BOUNDARY_NEG
    #undef _LRF_DIFF_DATA_UNCHANGE_MAX_COUNT
}
#endif    // END OF #ifdef _MC3433_SUPPORT_LRF_

/*****************************************
 *** _MC3433_ReadData_RBM2RAW
 *****************************************/
static void    _MC3433_ReadData_RBM2RAW(s16 waData[MC3433_AXES_NUM])
{
    waData[MC3433_AXIS_X] = (waData[MC3433_AXIS_X] + offset_data[MC3433_AXIS_X] / 2) * 1024 / gain_data[MC3433_AXIS_X] + 8096;
    waData[MC3433_AXIS_Y] = (waData[MC3433_AXIS_Y] + offset_data[MC3433_AXIS_Y] / 2) * 1024 / gain_data[MC3433_AXIS_Y] + 8096;
    waData[MC3433_AXIS_Z] = (waData[MC3433_AXIS_Z] + offset_data[MC3433_AXIS_Z] / 2) * 1024 / gain_data[MC3433_AXIS_Z] + 8096;

    GSE_LOG("RBM->RAW <<<<<[%08d %08d %08d]\n", waData[MC3433_AXIS_X], waData[MC3433_AXIS_Y], waData[MC3433_AXIS_Z]);

    iAReal0_X             = (0x0010 * waData[MC3433_AXIS_X]);
    iAcc1Lpf0_X           = GetLowPassFilter(iAReal0_X,iAcc1Lpf1_X);
    iAcc0Lpf0_X           = GetLowPassFilter(iAcc1Lpf0_X,iAcc0Lpf1_X);
    waData[MC3433_AXIS_X] = (iAcc0Lpf0_X / 0x0010);
    
    iAReal0_Y             = (0x0010 * waData[MC3433_AXIS_Y]);
    iAcc1Lpf0_Y           = GetLowPassFilter(iAReal0_Y,iAcc1Lpf1_Y);
    iAcc0Lpf0_Y           = GetLowPassFilter(iAcc1Lpf0_Y,iAcc0Lpf1_Y);
    waData[MC3433_AXIS_Y] = (iAcc0Lpf0_Y / 0x0010);
    
    iAReal0_Z             = (0x0010 * waData[MC3433_AXIS_Z]);
    iAcc1Lpf0_Z           = GetLowPassFilter(iAReal0_Z,iAcc1Lpf1_Z);
    iAcc0Lpf0_Z           = GetLowPassFilter(iAcc1Lpf0_Z,iAcc0Lpf1_Z);
    waData[MC3433_AXIS_Z] = (iAcc0Lpf0_Z / 0x0010);
    
    GSE_LOG("RBM->RAW->LPF <<<<<[%08d %08d %08d]\n", waData[MC3433_AXIS_X], waData[MC3433_AXIS_Y], waData[MC3433_AXIS_Z]);	

    waData[MC3433_AXIS_X] = (waData[MC3433_AXIS_X] - 8096) * gsensor_gain.x / 1024;
    waData[MC3433_AXIS_Y] = (waData[MC3433_AXIS_Y] - 8096) * gsensor_gain.y / 1024;
    waData[MC3433_AXIS_Z] = (waData[MC3433_AXIS_Z] - 8096) * gsensor_gain.z / 1024;

    GSE_LOG("RBM->RAW->LPF->RAW <<<<<[%08d %08d %08d]\n", waData[MC3433_AXIS_X], waData[MC3433_AXIS_Y], waData[MC3433_AXIS_Z]);

    iAcc0Lpf1_X=iAcc0Lpf0_X;
    iAcc1Lpf1_X=iAcc1Lpf0_X;
    iAcc0Lpf1_Y=iAcc0Lpf0_Y;
    iAcc1Lpf1_Y=iAcc1Lpf0_Y;
    iAcc0Lpf1_Z=iAcc0Lpf0_Z;
    iAcc1Lpf1_Z=iAcc1Lpf0_Z;
}

/*****************************************
 *** MC3433_ReadData
 *****************************************/
static int MC3433_ReadData(struct i2c_client *pt_i2c_client, s16 waData[MC3433_AXES_NUM])
{
	u8 _baData[MC3433_DATA_LEN] = { 0 };

	GSE_LOG("[%s] s_nIsRBM_Enabled: %d\n", __FUNCTION__, s_nIsRBM_Enabled);

	if (NULL == pt_i2c_client)
	{
		GSE_ERR("ERR: Null Pointer\n");	

		return (MC3433_RETCODE_ERROR_NULL_POINTER);
	}
	
	if (!s_nIsRBM_Enabled)
	{
		if (MC3433_RESOLUTION_LOW == s_bResolution)
		{
			if (hwmsen_read_block(pt_i2c_client, MC3433_REG_XOUT, _baData, MC3433_LOW_REOLUTION_DATA_SIZE))
			{
				GSE_ERR("ERR: fail to read data via I2C!\n");

				return (MC3433_RETCODE_ERROR_I2C);
			}

			waData[MC3433_AXIS_X] = ((s8) _baData[0]);
			waData[MC3433_AXIS_Y] = ((s8) _baData[1]);
			waData[MC3433_AXIS_Z] = ((s8) _baData[2]);	 

			GSE_LOG("[%s][low] X: %d, Y: %d, Z: %d\n", __FUNCTION__, waData[MC3433_AXIS_X], waData[MC3433_AXIS_Y], waData[MC3433_AXIS_Z]);


			#ifdef _MC3433_SUPPORT_LRF_
			_MC3433_LowResFilter(MC3433_AXIS_X, waData);
			_MC3433_LowResFilter(MC3433_AXIS_Y, waData);
			_MC3433_LowResFilter(MC3433_AXIS_Z, waData);
			#endif
		}
		else if (MC3433_RESOLUTION_HIGH == s_bResolution)
		{
			if (hwmsen_read_block(pt_i2c_client, MC3433_REG_XOUT_EX_L, _baData, MC3433_HIGH_REOLUTION_DATA_SIZE))
			{
				GSE_ERR("ERR: fail to read data via I2C!\n");

				return (MC3433_RETCODE_ERROR_I2C);
			}

			waData[MC3433_AXIS_X] = ((signed short) ((_baData[0]) | (_baData[1]<<8)));
			waData[MC3433_AXIS_Y] = ((signed short) ((_baData[2]) | (_baData[3]<<8)));
			waData[MC3433_AXIS_Z] = ((signed short) ((_baData[4]) | (_baData[5]<<8)));

			GSE_LOG("[%s][high] X: %d, Y: %d, Z: %d\n", __FUNCTION__, waData[MC3433_AXIS_X], waData[MC3433_AXIS_Y], waData[MC3433_AXIS_Z]);
		}

		GSE_LOG("RAW<<<<<[%04d %04d %04d]\n", waData[MC3433_AXIS_X], waData[MC3433_AXIS_Y], waData[MC3433_AXIS_Z]);

		#ifdef _MC3433_SUPPORT_LPF_
		{
			struct MC3433_i2c_data   *_ptPrivData = i2c_get_clientdata(pt_i2c_client);  

			MC3433_LPF(_ptPrivData, waData);
			GSE_LOG("LPF<<<<<[%04d %04d %04d]\n", waData[MC3433_AXIS_X], waData[MC3433_AXIS_Y], waData[MC3433_AXIS_Z]);
		}
	#endif
	}
	else
	{
		if (hwmsen_read_block(pt_i2c_client, MC3433_REG_XOUT_EX_L, _baData, MC3433_HIGH_REOLUTION_DATA_SIZE))
		{
		    GSE_ERR("ERR: fail to read data via I2C!\n");

		    return (MC3433_RETCODE_ERROR_I2C);
		}

		waData[MC3433_AXIS_X] = ((s16)((_baData[0]) | (_baData[1] << 8)));
		waData[MC3433_AXIS_Y] = ((s16)((_baData[2]) | (_baData[3] << 8)));
		waData[MC3433_AXIS_Z] = ((s16)((_baData[4]) | (_baData[5] << 8)));

		GSE_LOG("RBM<<<<<[%08d %08d %08d]\n", waData[MC3433_AXIS_X], waData[MC3433_AXIS_Y], waData[MC3433_AXIS_Z]);

		_MC3433_ReadData_RBM2RAW(waData);
	}

	REMAP_IF_MC3250_READ(waData[MC3433_AXIS_X], waData[MC3433_AXIS_Y]);
	REMAP_IF_MC34XX_N(waData[MC3433_AXIS_X], waData[MC3433_AXIS_Y]);
	REMAP_IF_MC35XX(waData[MC3433_AXIS_X], waData[MC3433_AXIS_Y]);

	return (MC3433_RETCODE_SUCCESS);
}

/*****************************************
 *** MC3433_ReadOffset
 *****************************************/
static int MC3433_ReadOffset(struct i2c_client *client, s16 ofs[MC3433_AXES_NUM])
{    
	int err=0;
	u8 off_data[6]={0};
	

	if (MC3433_RESOLUTION_HIGH == s_bResolution)
	{
		if ((err = hwmsen_read_block(client, MC3433_REG_XOUT_EX_L, off_data, MC3433_DATA_LEN))) 
    	{
    		GSE_ERR("error: %d\n", err);
    		return err;
    	}
		ofs[MC3433_AXIS_X] = ((s16)(off_data[0]))|((s16)(off_data[1])<<8);
		ofs[MC3433_AXIS_Y] = ((s16)(off_data[2]))|((s16)(off_data[3])<<8);
		ofs[MC3433_AXIS_Z] = ((s16)(off_data[4]))|((s16)(off_data[5])<<8);
	}
	else if (MC3433_RESOLUTION_LOW == s_bResolution) 
	{
		if ((err = hwmsen_read_block(client, 0, off_data, 3))) 
    	{
    		GSE_ERR("error: %d\n", err);
    		return err;
    	}
		ofs[MC3433_AXIS_X] = (s8)off_data[0];
		ofs[MC3433_AXIS_Y] = (s8)off_data[1];
		ofs[MC3433_AXIS_Z] = (s8)off_data[2];			
	}

	GSE_LOG("MC3433_ReadOffset %d %d %d \n",ofs[MC3433_AXIS_X] ,ofs[MC3433_AXIS_Y],ofs[MC3433_AXIS_Z]);

    REMAP_IF_MC3250_READ(ofs[0], ofs[1]);
    REMAP_IF_MC34XX_N(ofs[0], ofs[1]);
    REMAP_IF_MC35XX(ofs[0], ofs[1]);

    return err;  
}

/*****************************************
 *** MC3433_ResetCalibration
 *****************************************/
static int MC3433_ResetCalibration(struct i2c_client *client)
{
	struct MC3433_i2c_data *obj = i2c_get_clientdata(client);
	u8 buf[MC3433_AXES_NUM] = {0x00, 0x00, 0x00};
	s16 tmp=0;
	int err=0;

    u8  bMsbFilter       = 0x3F;
    s16 wSignBitMask     = 0x2000;
    s16 wSignPaddingBits = 0xC000;

	buf[0] = 0x43;
	
	if((err = hwmsen_write_block(client, 0x07, buf, 1)))
	{
		GSE_ERR("error 0x07: %d\n", err);
	}


	if((err = hwmsen_write_block(client, 0x21, offset_buf, 6))) // add by liang for writing offset register as OTP value 
	{
		GSE_ERR("error: %d\n", err);
	}
	
	buf[0] = 0x41;

	if((err = hwmsen_write_block(client, 0x07, buf, 1)))
	{
		GSE_ERR("error: %d\n", err);
	}

	msleep(20);

	if (IS_MC35XX())
    {
        bMsbFilter       = 0x7F;
        wSignBitMask     = 0x4000;
        wSignPaddingBits = 0x8000;
    }

	tmp = ((offset_buf[1] & bMsbFilter) << 8) + offset_buf[0];  // add by Liang for set offset_buf as OTP value 
	if (tmp & wSignBitMask)
		tmp |= wSignPaddingBits;
	offset_data[0] = tmp;
				
	tmp = ((offset_buf[3] & bMsbFilter) << 8) + offset_buf[2];  // add by Liang for set offset_buf as OTP value 
		if (tmp & wSignBitMask)
			tmp |= wSignPaddingBits;
	offset_data[1] = tmp;
				
	tmp = ((offset_buf[5] & bMsbFilter) << 8) + offset_buf[4];  // add by Liang for set offset_buf as OTP value 
	if (tmp & wSignBitMask)
		tmp |= wSignPaddingBits;
	offset_data[2] = tmp;	

	memset(obj->cali_sw, 0x00, sizeof(obj->cali_sw));

	return err;  
}

/*****************************************
 *** MC3433_ReadCalibration
 *****************************************/
static int MC3433_ReadCalibration(struct i2c_client *client, int dat[MC3433_AXES_NUM])
{
    struct MC3433_i2c_data *obj = i2c_get_clientdata(client);
    int err =0;
	
    if ((err = MC3433_ReadOffset(client, obj->offset))) {
        GSE_ERR("read offset fail, %d\n", err);
        return err;
    }    
    
    dat[MC3433_AXIS_X] = obj->offset[MC3433_AXIS_X];
    dat[MC3433_AXIS_Y] = obj->offset[MC3433_AXIS_Y];
    dat[MC3433_AXIS_Z] = obj->offset[MC3433_AXIS_Z];  

	GSE_LOG("MC3433_ReadCalibration %d %d %d \n",dat[obj->cvt.map[MC3433_AXIS_X]] ,dat[obj->cvt.map[MC3433_AXIS_Y]],dat[obj->cvt.map[MC3433_AXIS_Z]]);
                                      
    return 0;
}

/*****************************************
 *** MC3433_WriteCalibration
 *****************************************/
static int MC3433_WriteCalibration(struct i2c_client *client, int dat[MC3433_AXES_NUM])
{
    struct MC3433_i2c_data *obj = i2c_get_clientdata(client);
    int err = 0;
    u8 buf[9] ={ 0};
    s16 tmp = 0, x_gain = 0, y_gain = 0, z_gain = 0;
    s32 x_off = 0, y_off = 0, z_off = 0;
    int cali[MC3433_AXES_NUM] = {0};
    
    u8  bMsbFilter       = 0x3F;
    s16 wSignBitMask     = 0x2000;
    s16 wSignPaddingBits = 0xC000;
    s32 dwRangePosLimit  = 0x1FFF;
    s32 dwRangeNegLimit  = -0x2000;
    
    GSE_LOG("UPDATE dat: (%+3d %+3d %+3d)\n", dat[MC3433_AXIS_X], dat[MC3433_AXIS_Y], dat[MC3433_AXIS_Z]);
    
    cali[MC3433_AXIS_X] = obj->cvt.sign[MC3433_AXIS_X]*(dat[obj->cvt.map[MC3433_AXIS_X]]);
    cali[MC3433_AXIS_Y] = obj->cvt.sign[MC3433_AXIS_Y]*(dat[obj->cvt.map[MC3433_AXIS_Y]]);
    cali[MC3433_AXIS_Z] = obj->cvt.sign[MC3433_AXIS_Z]*(dat[obj->cvt.map[MC3433_AXIS_Z]]);	
    
    REMAP_IF_MC3250_WRITE(cali[MC3433_AXIS_X], cali[MC3433_AXIS_Y]);
    REMAP_IF_MC34XX_N(cali[MC3433_AXIS_X], cali[MC3433_AXIS_Y]);
    REMAP_IF_MC35XX(cali[MC3433_AXIS_X], cali[MC3433_AXIS_Y]);
    
    GSE_LOG("UPDATE dat: (%+3d %+3d %+3d)\n", cali[MC3433_AXIS_X], cali[MC3433_AXIS_Y], cali[MC3433_AXIS_Z]);
    
    // read registers 0x21~0x29
    if ((err = hwmsen_read_block(client, 0x21, buf, 3))) 
    {
        GSE_ERR("error: %d\n", err);
        return err;
    }
    if ((err = hwmsen_read_block(client, 0x24, &buf[3], 3))) 
    {
        GSE_ERR("error: %d\n", err);
        return err;
    }
    if ((err = hwmsen_read_block(client, 0x27, &buf[6], 3))) 
    {
        GSE_ERR("error: %d\n", err);
        return err;
    }
    
    if (IS_MC35XX())
    {
        bMsbFilter       = 0x7F;
        wSignBitMask     = 0x4000;
        wSignPaddingBits = 0x8000;
        dwRangePosLimit  = 0x3FFF;
        dwRangeNegLimit  = -0x4000;
    }
    
    // get x,y,z offset
    tmp = ((buf[1] & bMsbFilter) << 8) + buf[0];
    if (tmp & wSignBitMask)
        tmp |= wSignPaddingBits;
    x_off = tmp;
    
    tmp = ((buf[3] & bMsbFilter) << 8) + buf[2];
    if (tmp & wSignBitMask)
        tmp |= wSignPaddingBits;
    y_off = tmp;
    
    tmp = ((buf[5] & bMsbFilter) << 8) + buf[4];
    if (tmp & wSignBitMask)
        tmp |= wSignPaddingBits;
    z_off = tmp;
    
    // get x,y,z gain
    x_gain = ((buf[1] >> 7) << 8) + buf[6];
    y_gain = ((buf[3] >> 7) << 8) + buf[7];
    z_gain = ((buf[5] >> 7) << 8) + buf[8];
    
    // prepare new offset
    x_off = x_off + 16 * cali[MC3433_AXIS_X] * 256 * 128 / 3 / gsensor_gain.x / (40 + x_gain);
    y_off = y_off + 16 * cali[MC3433_AXIS_Y] * 256 * 128 / 3 / gsensor_gain.y / (40 + y_gain);
    z_off = z_off + 16 * cali[MC3433_AXIS_Z] * 256 * 128 / 3 / gsensor_gain.z / (40 + z_gain);
    
    //add for over range 
    if( x_off > dwRangePosLimit) 
    {
        x_off = dwRangePosLimit;
    }
    else if( x_off < dwRangeNegLimit)
    {
        x_off = dwRangeNegLimit;
    }
    
    if( y_off > dwRangePosLimit) 
    {
        y_off = dwRangePosLimit;
    }
    else if( y_off < dwRangeNegLimit)
    {
        y_off = dwRangeNegLimit;
    }
    
    if( z_off > dwRangePosLimit) 
    {
        z_off = dwRangePosLimit;
    }
    else if( z_off < dwRangeNegLimit)
    {
        z_off = dwRangeNegLimit;
    }
    
    //storege the cerrunt offset data with DOT format
    offset_data[0] = x_off;
    offset_data[1] = y_off;
    offset_data[2] = z_off;
    
    //storege the cerrunt Gain data with GOT format
    gain_data[0] = 256*8*128/3/(40+x_gain);
    gain_data[1] = 256*8*128/3/(40+y_gain);
    gain_data[2] = 256*8*128/3/(40+z_gain);
    
    buf[0]=0x43;
    hwmsen_write_block(client, 0x07, buf, 1);
    
    buf[0] = x_off & 0xff;
    buf[1] = ((x_off >> 8) & bMsbFilter) | (x_gain & 0x0100 ? 0x80 : 0);
    buf[2] = y_off & 0xff;
    buf[3] = ((y_off >> 8) & bMsbFilter) | (y_gain & 0x0100 ? 0x80 : 0);
    buf[4] = z_off & 0xff;
    buf[5] = ((z_off >> 8) & bMsbFilter) | (z_gain & 0x0100 ? 0x80 : 0);
    
    hwmsen_write_block(client, 0x21, buf, 6);
    
    buf[0]=0x41;
    hwmsen_write_block(client, 0x07, buf, 1);		
    
    msleep(50);
    
    return err;
}

/*****************************************
 *** MC3433_SetPowerMode
 *****************************************/
static int MC3433_SetPowerMode(struct i2c_client *client, bool enable)
{
    u8 databuf[2] = {0};    
    int res = 0;
    u8 addr = MC3433_REG_MODE_FEATURE;
    struct MC3433_i2c_data *obj = i2c_get_clientdata(client);
    
    if(enable == ((bool)atomic_read(&obj->sensor_power)))
        GSE_LOG("Sensor power status should not be set again!!!\n");
    
    if(hwmsen_read_byte_sr(client, addr, databuf))
    {
        GSE_ERR("read power ctl register err!\n");
        return MC3433_RETCODE_ERROR_I2C;
    }

    GSE_LOG("set power read MC3433_REG_MODE_FEATURE =%x\n", databuf[0]);   
    
    if(enable)
    {
        databuf[1] = 0x41;
        databuf[0] = MC3433_REG_MODE_FEATURE;
        res = i2c_master_send(client, databuf, 0x2);
    
        #ifdef _MC3433_SUPPORT_DOT_CALIBRATION_
            mcube_load_cali(client);
        #endif
    }
    else
    {
        databuf[1] = 0x43;
        databuf[0] = MC3433_REG_MODE_FEATURE;
        res = i2c_master_send(client, databuf, 0x2);
    }
    
    if(res <= 0)
    {
        GSE_LOG("fwq set power mode failed!\n");
        return MC3433_RETCODE_ERROR_I2C;
    }
    else if(atomic_read(&obj->trace) & MCUBE_TRC_INFO)
    {
        GSE_LOG("fwq set power mode ok %d!\n", databuf[1]);
    }
    
    atomic_set(&obj->sensor_power, enable);

    return MC3433_RETCODE_SUCCESS;    
}

/*****************************************
 *** MC3433_SetResolution
 *****************************************/
static void MC3433_SetResolution(void)
{
    GSE_LOG("[%s]\n", __FUNCTION__);

    switch (s_bPCODE)
    {
    case MC3433_PCODE_3230:
    case MC3433_PCODE_3430:
    case MC3433_PCODE_3430N:
    case MC3433_PCODE_3530B:
    case MC3433_PCODE_3530C:
         s_bResolution = MC3433_RESOLUTION_LOW;
         break;

    case MC3433_PCODE_3210:
    case MC3433_PCODE_3250:
    case MC3433_PCODE_3410:
    case MC3433_PCODE_3410N:
    case MC3433_PCODE_3510B:
    case MC3433_PCODE_3510C:
         s_bResolution = MC3433_RESOLUTION_HIGH;
         break;

    default:
         GSE_ERR("ERR: no resolution assigned!\n");
         break;
    }

    GSE_LOG("[%s] s_bResolution: %d  ----------s_bPCODE :%d \n", __FUNCTION__, s_bResolution,s_bPCODE);
}

/*****************************************
 *** MC3433_ConfigRegRange
 *****************************************/
static void MC3433_ConfigRegRange(struct i2c_client *pt_i2c_client)
{
    unsigned char    _baDataBuf[2] = { 0 };

    _baDataBuf[0] = MC3433_REG_RANGE_CONTROL;
    _baDataBuf[1] = 0x3F;

    if (MC3433_RESOLUTION_LOW == s_bResolution)
        _baDataBuf[1] = 0x32;

    if ((MC3433_PCODE_3510B == s_bPCODE) || (MC3433_PCODE_3510C == s_bPCODE))
        _baDataBuf[1] = 0x25;
    else if ((MC3433_PCODE_3530B == s_bPCODE) || (MC3433_PCODE_3530C == s_bPCODE))
        _baDataBuf[1] = 0x02;

    i2c_master_send(pt_i2c_client, _baDataBuf, 0x2);

    GSE_LOG("[%s] set 0x%X\n", __FUNCTION__, _baDataBuf[1]);
}

/*****************************************
 *** MC3433_SetGain
 *****************************************/
static void MC3433_SetGain(void)
{
    gsensor_gain.x = gsensor_gain.y = gsensor_gain.z = 1024;

    if (MC3433_RESOLUTION_LOW == s_bResolution)
    {
        gsensor_gain.x = gsensor_gain.y = gsensor_gain.z = 86;

        if ((MC3433_PCODE_3530B == s_bPCODE) || (MC3433_PCODE_3530C == s_bPCODE))
        {
            gsensor_gain.x = gsensor_gain.y = gsensor_gain.z = 64;
        }
    }
    
    GSE_LOG("[%s] gain: %d / %d / %d\n", __FUNCTION__, gsensor_gain.x, gsensor_gain.y, gsensor_gain.z);
}

/*****************************************
 *** MC3433_Init
 *****************************************/
static int MC3433_Init_client(struct i2c_client *client, int reset_cali)
{
    unsigned char    _baDataBuf[2] = { 0 };

    GSE_LOG("[%s]\n", __FUNCTION__);

    #ifdef _MC3433_SUPPORT_POWER_SAVING_SHUTDOWN_POWER_
/*
        if (MC3433_RETCODE_SUCCESS != _MC3433_i2c_auto_probe(client))
        {
            //GSE_ERR("ERR: fail to probe mCube sensor!\n");
            return (MC3433_RETCODE_ERROR_I2C);
        }
*/
        //GSE_LOG("[%s] confirmed i2c addr: 0x%X\n", __FUNCTION__, client->addr);
    #endif

    _baDataBuf[0] = MC3433_REG_MODE_FEATURE;
    _baDataBuf[1] = 0x43;
    i2c_master_send(client, _baDataBuf, 0x2);

    MC3433_SetResolution();
    
    _baDataBuf[0] = MC3433_REG_SAMPLE_RATE;
    _baDataBuf[1] = 0x00;

    if (IS_MC35XX())
        _baDataBuf[1] = 0x0A;

    i2c_master_send(client, _baDataBuf, 0x2);
    
    _baDataBuf[0] = MC3433_REG_TAP_DETECTION_ENABLE;
    _baDataBuf[1] = 0x00;
    i2c_master_send(client, _baDataBuf, 0x2);
    
    _baDataBuf[0] = MC3433_REG_INTERRUPT_ENABLE;
    _baDataBuf[1] = 0x00;
    i2c_master_send(client, _baDataBuf, 0x2);

    MC3433_ConfigRegRange(client);

    MC3433_SetGain();

    #ifdef _MC3433_SUPPORT_DOT_CALIBRATION_
        MC3433_rbm(client,0);
    #endif

    #ifdef _MC3433_SUPPORT_LPF_
    {
        struct MC3433_i2c_data   *_pt_i2c_data = i2c_get_clientdata(client);

        memset(&_pt_i2c_data->fir, 0x00, sizeof(_pt_i2c_data->fir));
    }
    #endif

    #ifdef _MC3433_SUPPORT_LRF_
        memset(&s_taLRF_CB, 0, sizeof(s_taLRF_CB));
    #endif

    #ifdef _MC3433_SUPPORT_PERIODIC_DOC_
        init_waitqueue_head(&wq_MC3433_open_status);
    #endif

    GSE_LOG("[%s] init ok.\n", __FUNCTION__);

    return (MC3433_RETCODE_SUCCESS);
}

/*****************************************
 *** MC3433_ReadChipInfo
 *****************************************/
static int MC3433_ReadChipInfo(struct i2c_client *client, char *buf, int bufsize)
{
	if((NULL == buf)||(bufsize<=30))
	{
		return -1;
	}
	
	if(NULL == client)
	{
		*buf = 0;
		return -2;
	}

	sprintf(buf, "MC3433 Chip");
	return 0;
}

/*****************************************
 *** MC3433_ReadSensorData
 *****************************************/
static int MC3433_ReadSensorData(struct i2c_client *pt_i2c_client, char *pbBuf, int nBufSize)
{
	int	_naAccelData[MC3433_AXES_NUM] = { 0 };
	struct MC3433_i2c_data   *obj= ((struct MC3433_i2c_data*) i2c_get_clientdata(pt_i2c_client));

	//GSE_LOG("[%s]\n", __FUNCTION__);	

	if ((NULL == pt_i2c_client) || (NULL == pbBuf))
	{
		GSE_ERR("ERR: Null Pointer, pt_i2c_client: 0x%X, pbBuf: 0x%X\n", (unsigned int) pt_i2c_client, (unsigned int) pbBuf);	

		return (MC3433_RETCODE_ERROR_NULL_POINTER);
	}

	if (false == ((bool)atomic_read(&obj->sensor_power)))
	{
		if (MC3433_RETCODE_SUCCESS != MC3433_SetPowerMode(pt_i2c_client, true))
			GSE_ERR("ERR: fail to set power mode!\n");
	}

	#ifdef _MC3433_SUPPORT_DOT_CALIBRATION_
	mcube_load_cali(pt_i2c_client);

	if ((s_nIsRBM_Enabled) && (1 == LPF_FirstRun))
	{
		int	_nLoopIndex = 0;

		LPF_FirstRun = 0;

		for (_nLoopIndex = 0; _nLoopIndex < (LPF_SamplingRate + LPF_CutoffFrequency); _nLoopIndex++)
			MC3433_ReadData(pt_i2c_client, obj->data);
	}
	#endif

	if (MC3433_RETCODE_SUCCESS != MC3433_ReadData(pt_i2c_client, obj->data))
	{        
		GSE_ERR("ERR: fail to read data!\n");

		return (MC3433_RETCODE_ERROR_I2C);
	}

	//output format: mg
	//GSE_LOG("[%s] raw data: %d, %d, %d\n", __FUNCTION__, obj->data[MC3433_AXIS_X], obj->data[MC3433_AXIS_Y], obj->data[MC3433_AXIS_Z]);
	_naAccelData[(obj->cvt.map[MC3433_AXIS_X])] = (obj->cvt.sign[MC3433_AXIS_X] * obj->data[MC3433_AXIS_X]);
	_naAccelData[(obj->cvt.map[MC3433_AXIS_Y])] = (obj->cvt.sign[MC3433_AXIS_Y] * obj->data[MC3433_AXIS_Y]);
	_naAccelData[(obj->cvt.map[MC3433_AXIS_Z])] = (obj->cvt.sign[MC3433_AXIS_Z] * obj->data[MC3433_AXIS_Z]);

	//GSE_LOG("[%s] map data: %d, %d, %d!\n", __FUNCTION__, _naAccelData[MC3433_AXIS_X], _naAccelData[MC3433_AXIS_Y], _naAccelData[MC3433_AXIS_Z]);

	_naAccelData[MC3433_AXIS_X] = (_naAccelData[MC3433_AXIS_X] * GRAVITY_EARTH_1000 / gsensor_gain.x);
	_naAccelData[MC3433_AXIS_Y] = (_naAccelData[MC3433_AXIS_Y] * GRAVITY_EARTH_1000 / gsensor_gain.y);
	_naAccelData[MC3433_AXIS_Z] = (_naAccelData[MC3433_AXIS_Z] * GRAVITY_EARTH_1000 / gsensor_gain.z);	
	GSE_LOG("[%s] accel data: X=:%d, Y=:%d, Z=:%d!\n", __FUNCTION__, _naAccelData[MC3433_AXIS_X], _naAccelData[MC3433_AXIS_Y], _naAccelData[MC3433_AXIS_Z]);

	sprintf(pbBuf, "%04x %04x %04x", _naAccelData[MC3433_AXIS_X], _naAccelData[MC3433_AXIS_Y], _naAccelData[MC3433_AXIS_Z]);

	return (MC3433_RETCODE_SUCCESS);
}

/*****************************************
 *** _MC3433_ReadAverageData
 *****************************************/
#ifdef _MC3433_SUPPORT_APPLY_AVERAGE_AGORITHM_
static int _MC3433_ReadAverageData(struct i2c_client *client, char *buf)
{
	struct MC3433_i2c_data *obj = (struct MC3433_i2c_data*)i2c_get_clientdata(client);
	int acc[MC3433_AXES_NUM]={0};
	s16 sensor_data[3]={0};
	s16 sensor_data_max[3]={0};
	s16 sensor_data_mini[3]={0};
	s32 sensor_data_sum[3]={0};

	int i = 0,j=0;

	MC3433_ReadData(client, sensor_data);
	GSE_LOG("MC3433_ReadRawData MC3433_ReadData: %d, %d, %d!\n", sensor_data[MC3433_AXIS_X], sensor_data[MC3433_AXIS_Y], sensor_data[MC3433_AXIS_Z]);
	sensor_data_max[MC3433_AXIS_X]=sensor_data[MC3433_AXIS_X];
	sensor_data_max[MC3433_AXIS_Y]=sensor_data[MC3433_AXIS_Y];
	sensor_data_max[MC3433_AXIS_Z]=sensor_data[MC3433_AXIS_Z];
	
	sensor_data_mini[MC3433_AXIS_X]=sensor_data[MC3433_AXIS_X];
	sensor_data_mini[MC3433_AXIS_Y]=sensor_data[MC3433_AXIS_Y];
	sensor_data_mini[MC3433_AXIS_Z]=sensor_data[MC3433_AXIS_Z];
	
	sensor_data_sum[MC3433_AXIS_X]+=sensor_data[MC3433_AXIS_X];
	sensor_data_sum[MC3433_AXIS_Y]+=sensor_data[MC3433_AXIS_Y];
	sensor_data_sum[MC3433_AXIS_Z]+=sensor_data[MC3433_AXIS_Z];
	
	for(i=0; i<11 ; i++)
	{
		MC3433_ReadData(client, sensor_data);
		GSE_LOG("MC3433_ReadRawData MC3433_ReadData: %d, %d, %d!\n", sensor_data[MC3433_AXIS_X], sensor_data[MC3433_AXIS_Y], sensor_data[MC3433_AXIS_Z]);
		
		sensor_data_sum[MC3433_AXIS_X]+=sensor_data[MC3433_AXIS_X];
		sensor_data_sum[MC3433_AXIS_Y]+=sensor_data[MC3433_AXIS_Y];
		sensor_data_sum[MC3433_AXIS_Z]+=sensor_data[MC3433_AXIS_Z];
		for(j=0; j<3 ; j++)
		{
			if(sensor_data[j]>sensor_data_max[j])
			{
				sensor_data_max[j]=sensor_data[j];
			}
			if(sensor_data[j]<sensor_data_mini[j])
			{
				sensor_data_mini[j]=sensor_data[j];
			}
		}
	}
	GSE_LOG("MC3433_ReadRawData sensor_data_max: %d, %d, %d!\n", sensor_data_max[MC3433_AXIS_X], sensor_data_max[MC3433_AXIS_Y], sensor_data_max[MC3433_AXIS_Z]);
	GSE_LOG("MC3433_ReadRawData sensor_data_mini: %d, %d, %d!\n", sensor_data_mini[MC3433_AXIS_X], sensor_data_mini[MC3433_AXIS_Y], sensor_data_mini[MC3433_AXIS_Z]);
	sensor_data[MC3433_AXIS_X] = (s16)((sensor_data_sum[MC3433_AXIS_X]-sensor_data_max[MC3433_AXIS_X]-sensor_data_mini[MC3433_AXIS_X])/10);
	sensor_data[MC3433_AXIS_Y] = (s16)((sensor_data_sum[MC3433_AXIS_Y]-sensor_data_max[MC3433_AXIS_Y]-sensor_data_mini[MC3433_AXIS_Y])/10);
	sensor_data[MC3433_AXIS_Z] = (s16)((sensor_data_sum[MC3433_AXIS_Z]-sensor_data_max[MC3433_AXIS_Z]-sensor_data_mini[MC3433_AXIS_Z])/10);
	GSE_LOG("MC3433_ReadRawData sensor_data: %d, %d, %d!\n", sensor_data[MC3433_AXIS_X], sensor_data[MC3433_AXIS_Y], sensor_data[MC3433_AXIS_Z]);
	
	acc[(obj->cvt.map[MC3433_AXIS_X])] = obj->cvt.sign[MC3433_AXIS_X] * sensor_data[MC3433_AXIS_X];
	acc[(obj->cvt.map[MC3433_AXIS_Y])] = obj->cvt.sign[MC3433_AXIS_Y] * sensor_data[MC3433_AXIS_Y];
	acc[(obj->cvt.map[MC3433_AXIS_Z])] = obj->cvt.sign[MC3433_AXIS_Z] * sensor_data[MC3433_AXIS_Z];
	
	GSE_LOG("MC3433_ReadRawData mapdata: %d, %d, %d!\n", acc[MC3433_AXIS_X], acc[MC3433_AXIS_Y], acc[MC3433_AXIS_Z]);
	
	acc[MC3433_AXIS_X] = (acc[MC3433_AXIS_X]*GRAVITY_EARTH_1000/gsensor_gain.x);
	acc[MC3433_AXIS_Y] = (acc[MC3433_AXIS_Y]*GRAVITY_EARTH_1000/gsensor_gain.y);
	acc[MC3433_AXIS_Z] = (acc[MC3433_AXIS_Z]*GRAVITY_EARTH_1000/gsensor_gain.z);	
	
	GSE_LOG("MC3433_ReadRawData mapdata1: %d, %d, %d!\n", acc[MC3433_AXIS_X], acc[MC3433_AXIS_Y], acc[MC3433_AXIS_Z]);
	
	sprintf(buf, "%04x %04x %04x", acc[MC3433_AXIS_X], acc[MC3433_AXIS_Y], acc[MC3433_AXIS_Z]);

	return 0;
}
#endif    // END OF #ifdef _MC3433_SUPPORT_APPLY_AVERAGE_AGORITHM_

/*****************************************
 *** MC3433_ReadRawData
 *****************************************/
static int MC3433_ReadRawData(struct i2c_client *client, char *buf)
{
	int res = 0;

	if (!buf || !client)
	{
		return EINVAL;
	}

	if(((bool)atomic_read(&MC3433_obj_i2c_data->sensor_power)) == false)
	{
		res = MC3433_SetPowerMode(client, true);
		if(res)
		{
			GSE_ERR("Power on MC3433 error %d!\n", res);
		}
	}

	#ifdef _MC3433_SUPPORT_APPLY_AVERAGE_AGORITHM_
		return (_MC3433_ReadAverageData(client, buf));
	#else
	{
		s16 sensor_data[3]={0};
	
		if((res = MC3433_ReadData(client, sensor_data)))
		{        
			GSE_ERR("I2C error: ret value=%d", res);
			return EIO;
		}
		else
		{
			sprintf(buf, "%04x %04x %04x", sensor_data[MC3433_AXIS_X], 
			sensor_data[MC3433_AXIS_Y], sensor_data[MC3433_AXIS_Z]);
		}
	}
	#endif
	
	return 0;
}

#ifdef _MC3433_SUPPORT_DOT_CALIBRATION_
/*****************************************
 *** MC3433_ReadRBMData
 *****************************************/
static int MC3433_ReadRBMData(struct i2c_client *client, char *buf)
{
	int res = 0;
	int data[3] = { 0 };

	if (!buf || !client)
	{
		return EINVAL;
	}

	if(((bool)atomic_read(&MC3433_obj_i2c_data->sensor_power)) == false)
	{
		res = MC3433_SetPowerMode(client, true);
		if(res)
		{
			GSE_ERR("Power on MC3433 error %d!\n", res);
		}
	}
	
	if((res = MC3433_ReadData_RBM(client, data)))
	{        
		GSE_ERR("I2C error: ret value=%d", res);
		return EIO;
	}
	else
	{
		sprintf(buf, "%04x %04x %04x", data[MC3433_AXIS_X], 
			data[MC3433_AXIS_Y], data[MC3433_AXIS_Z]);
	
	}
	
	return res;
}
#endif // _MC3433_SUPPORT_DOT_CALIBRATION_

/*****************************************
 *** MC3433_JudgeTestResult
 *****************************************/
static int MC3433_JudgeTestResult(struct i2c_client *client)
{
    int    res                  = 0;
    int    self_result          = 0;
    s16    acc[MC3433_AXES_NUM] = { 0 };
	
    if((res = MC3433_ReadData(client, acc)))
    {        
        GSE_ERR("I2C error: ret value=%d", res);
        return EIO;
    }
    else
    {			
        acc[MC3433_AXIS_X] = acc[MC3433_AXIS_X] * 1000 / gsensor_gain.x;
        acc[MC3433_AXIS_Y] = acc[MC3433_AXIS_Y] * 1000 / gsensor_gain.y;
        acc[MC3433_AXIS_Z] = acc[MC3433_AXIS_Z] * 1000 / gsensor_gain.z;
        
        self_result = (  (acc[MC3433_AXIS_X] * acc[MC3433_AXIS_X])
                       + (acc[MC3433_AXIS_Y] * acc[MC3433_AXIS_Y])
                       + (acc[MC3433_AXIS_Z] * acc[MC3433_AXIS_Z]));
        
        if ( (self_result > 475923) && (self_result < 2185360) )    //between 0.7g and 1.5g 
        {												 
            GSE_ERR("MC3433_JudgeTestResult successful\n");
            return MC3433_RETCODE_SUCCESS;
        }
        else
        {
            GSE_ERR("MC3433_JudgeTestResult failt\n");
            return -EINVAL;
        }
    }
}

/*****************************************
 *** MC3433_SelfCheck
 *****************************************/
static void MC3433_SelfCheck(struct i2c_client *client, u8 *pUserBuf)
{
    u8    _bRData1 = 0;
    u8    _bRData2 = 0;
    u8    _bRData3 = 0;
    u8    _baDataBuf[2] = { 0 };

    hwmsen_read_block(client, 0x20, &_bRData1, 1);
    hwmsen_read_block(client, 0x3B, &_bRData2, 1);

    _baDataBuf[0] = 0x43;
    hwmsen_write_block(client, 0x07, _baDataBuf, 1);

    mdelay(10);

    for ( ; ; )
    {
        _baDataBuf[0] = 0x6D;
        hwmsen_write_block(client, 0x1B, _baDataBuf, 1);
        
        _baDataBuf[0] = 0x43;
        hwmsen_write_block(client, 0x1B, _baDataBuf, 1);

        _bRData3 = 0x00;
        hwmsen_read_block(client, 0x04, &_bRData3, 1);

        if (_bRData3 & 0x40)
            break;
    }

    _baDataBuf[0] = (_bRData2 & 0xFE);
    hwmsen_write_block(client, 0x3B, _baDataBuf, 1);

    _baDataBuf[0] = 0x03;
    hwmsen_write_block(client, 0x20, _baDataBuf, 1);

    _baDataBuf[0] = 0x40;
    hwmsen_write_block(client, 0x14, _baDataBuf, 1);

    mdelay(10);

    _baDataBuf[0] = pUserBuf[0];
    hwmsen_write_block(client, 0x00, _baDataBuf, 1);

    _baDataBuf[0] = 0x41;
    hwmsen_write_block(client, 0x07, _baDataBuf, 1);

    mdelay(10);

    _baDataBuf[0] = 0x43;
    hwmsen_write_block(client, 0x07, _baDataBuf, 1);

    mdelay(10);

    MC3433_Read_Reg_Map(client, pUserBuf);

    mdelay(10);

    _baDataBuf[0] = 0x00;
    hwmsen_write_block(client, 0x14, _baDataBuf, 1);

    _baDataBuf[0] = _bRData1;
    hwmsen_write_block(client, 0x20, _baDataBuf, 1);

    _baDataBuf[0] = _bRData2;
    hwmsen_write_block(client, 0x3B, _baDataBuf, 1);

    mdelay(10);

    for ( ; ; )
    {
        _baDataBuf[0] = 0x6D;
        hwmsen_write_block(client, 0x1B, _baDataBuf, 1);
        
        _baDataBuf[0] = 0x43;
        hwmsen_write_block(client, 0x1B, _baDataBuf, 1);

        _bRData3 = 0xFF;
        hwmsen_read_block(client, 0x04, &_bRData3, 1);
        
        if (!(_bRData3 & 0x40))
            break;
    }

    mdelay(10);
}

/*****************************************
 *** MC3433_GetOpenStatus
 *****************************************/
#ifdef _MC3433_SUPPORT_PERIODIC_DOC_
static int    MC3433_GetOpenStatus(void)
{
    //GSE_LOG("[%s] %d\n", __FUNCTION__, atomic_read(&s_t_MC3433_open_status));

    wait_event_interruptible(wq_MC3433_open_status, (atomic_read(&s_t_MC3433_open_status) != 0));

    //GSE_LOG("[%s] pass wait_event_interruptible: %d\n", __FUNCTION__, atomic_read(&s_t_MC3433_open_status));
    
    return (atomic_read(&s_t_MC3433_open_status));
}
#endif

/*****************************************
 *** show_chipinfo_value
 *****************************************/
static ssize_t show_chipinfo_value(struct device_driver *ddri, char *buf)
{
	struct i2c_client *client = MC3433_i2c_client;
	char strbuf[MC3433_BUF_SIZE]={0};
    GSE_LOG("fwq show_chipinfo_value \n");
	if(NULL == client)
	{
		GSE_ERR("i2c client is null!!\n");
		return 0;
	}
	
	MC3433_ReadChipInfo(client, strbuf, MC3433_BUF_SIZE);
	return snprintf(buf, PAGE_SIZE, "%s\n", strbuf);        
}

/*****************************************
 *** show_sensordata_value
 *****************************************/
static ssize_t show_sensordata_value(struct device_driver *ddri, char *buf)
{
	struct i2c_client *client = MC3433_i2c_client;
	char strbuf[MC3433_BUF_SIZE] = { 0 };
	
	if(NULL == client)
	{
		GSE_ERR("i2c client is null!!\n");
		return 0;
	}

    MC3433_MUTEX_LOCK();
	MC3433_ReadSensorData(client, strbuf, MC3433_BUF_SIZE);
    MC3433_MUTEX_UNLOCK();
	return snprintf(buf, PAGE_SIZE, "%s\n", strbuf);            
}

/*****************************************
 *** show_cali_value
 *****************************************/
static ssize_t show_cali_value(struct device_driver *ddri, char *buf)
{
	struct i2c_client *client = MC3433_i2c_client;
	struct MC3433_i2c_data *obj;
	int err = 0;
	int len = 0;
	int mul = 0;
	int tmp[MC3433_AXES_NUM] = { 0 };

    GSE_LOG("fwq show_cali_value \n");

	if(NULL == client)
	{
		GSE_ERR("i2c client is null!!\n");
		return 0;
	}

	obj = (struct MC3433_i2c_data*) i2c_get_clientdata(client);

	
	if((err = MC3433_ReadOffset(client, obj->offset)))
	{
		return -EINVAL;
	}
	else if((err = MC3433_ReadCalibration(client, tmp)))
	{
		return -EINVAL;
	}
	else
	{    
// +++ 20130104 -- obj->reso is no longer used, replaced by gensor_gain to avoid system crash		
		//mul = obj->reso->sensitivity/MC3433_offset_resolution.sensitivity;
		mul = gsensor_gain.x /MC3433_offset_resolution.sensitivity;
		/*len += snprintf(buf+len, PAGE_SIZE-len, "[HW ][%d] (%+3d, %+3d, %+3d) : (0x%02X, 0x%02X, 0x%02X)\n", mul,                        
			obj->offset[MC3433_AXIS_X], obj->offset[MC3433_AXIS_Y], obj->offset[MC3433_AXIS_Z],
			obj->offset[MC3433_AXIS_X], obj->offset[MC3433_AXIS_Y], obj->offset[MC3433_AXIS_Z]);
		*/	
// +++ 20130104 -- obj->reso is no longer used, replaced by gensor_gain to avoid system crash
		len += snprintf(buf+len, PAGE_SIZE-len, "[HW] (%+3d, %+3d, %+3d) : (0x%02X, 0x%02X, 0x%02X)\n",                        
			obj->offset[MC3433_AXIS_X], obj->offset[MC3433_AXIS_Y], obj->offset[MC3433_AXIS_Z],
			obj->offset[MC3433_AXIS_X], obj->offset[MC3433_AXIS_Y], obj->offset[MC3433_AXIS_Z]);
			
		len += snprintf(buf+len, PAGE_SIZE-len, "[SW][%d] (%+3d, %+3d, %+3d)\n", 1, 
			obj->cali_sw[MC3433_AXIS_X], obj->cali_sw[MC3433_AXIS_Y], obj->cali_sw[MC3433_AXIS_Z]);

		len += snprintf(buf+len, PAGE_SIZE-len, "[ALL]  (%+3d, %+3d, %+3d) : (%+3d, %+3d, %+3d)\n", 
			obj->offset[MC3433_AXIS_X]*mul + obj->cali_sw[MC3433_AXIS_X],
			obj->offset[MC3433_AXIS_Y]*mul + obj->cali_sw[MC3433_AXIS_Y],
			obj->offset[MC3433_AXIS_Z]*mul + obj->cali_sw[MC3433_AXIS_Z],
			tmp[MC3433_AXIS_X], tmp[MC3433_AXIS_Y], tmp[MC3433_AXIS_Z]);
		
		return len;
    }
}

/*****************************************
 *** store_cali_value
 *****************************************/
static ssize_t store_cali_value(struct device_driver *ddri, const char *buf, size_t count)
{
	struct i2c_client *client = MC3433_i2c_client;  
	int err=0;
	int x=0;
	int y=0;
	int z=0;
	int dat[MC3433_AXES_NUM]={0};

	if(!strncmp(buf, "rst", 3))
	{
        MC3433_MUTEX_LOCK();
		err = MC3433_ResetCalibration(client);
        MC3433_MUTEX_UNLOCK();

		if (err)
			GSE_ERR("reset offset err = %d\n", err);
	}
	else if(3 == sscanf(buf, "0x%02X 0x%02X 0x%02X", &x, &y, &z))
	{
		dat[MC3433_AXIS_X] = x;
		dat[MC3433_AXIS_Y] = y;
		dat[MC3433_AXIS_Z] = z;

        MC3433_MUTEX_LOCK();
		err = MC3433_WriteCalibration(client, dat);
        MC3433_MUTEX_UNLOCK();

		if (err)
			GSE_ERR("write calibration err = %d\n", err);
	}
	else
	{
		GSE_ERR("invalid format\n");
	}
	
	return count;
}

/*****************************************
 *** show_selftest_value
 *****************************************/
static ssize_t show_selftest_value(struct device_driver *ddri, char *buf)
{
    struct i2c_client *client = MC3433_i2c_client;

    if (NULL == client)
    {
        GSE_ERR("i2c client is null!!\n");
        return 0;
    }

    return snprintf(buf, 8, "%s\n", selftestRes);
}

/*****************************************
 *** store_selftest_value
 *****************************************/
static ssize_t store_selftest_value(struct device_driver *ddri, const char *buf, size_t count)
{   /*write anything to this register will trigger the process*/
    struct i2c_client *client = MC3433_i2c_client;  
    int num = 0;
    
    if (1 != sscanf(buf, "%d", &num))
    {
        GSE_ERR("parse number fail\n");
        return count;
    }
    else if(0 == num)
    {
        GSE_ERR("invalid data count\n");
        return count;
    }
       
    GSE_LOG("NORMAL:\n");
    MC3433_MUTEX_LOCK();
    MC3433_SetPowerMode(client, true); 
    MC3433_MUTEX_UNLOCK();
    GSE_LOG("SELFTEST:\n");    
    
    if (!MC3433_JudgeTestResult(client))
    {
        GSE_LOG("SELFTEST : PASS\n");
        strcpy(selftestRes,"y");
    }	
    else
    {
        GSE_LOG("SELFTEST : FAIL\n");		
        strcpy(selftestRes,"n");
    }
    
    return count;
}

/*****************************************
 *** show_firlen_value
 *****************************************/
static ssize_t show_firlen_value(struct device_driver *ddri, char *buf)
{
    #ifdef _MC3433_SUPPORT_LPF_
        struct i2c_client *client = MC3433_i2c_client;
        struct MC3433_i2c_data *obj = i2c_get_clientdata(client);
        GSE_LOG("fwq show_firlen_value \n");
        if(atomic_read(&obj->firlen))
        {
            int idx = 0, len = atomic_read(&obj->firlen);
            GSE_LOG("len = %2d, idx = %2d\n", obj->fir.num, obj->fir.idx);
            
            for(idx = 0; idx < len; idx++)
            {
                GSE_LOG("[%5d %5d %5d]\n", obj->fir.raw[idx][MC3433_AXIS_X], obj->fir.raw[idx][MC3433_AXIS_Y], obj->fir.raw[idx][MC3433_AXIS_Z]);
            }
            
            GSE_LOG("sum = [%5d %5d %5d]\n", obj->fir.sum[MC3433_AXIS_X], obj->fir.sum[MC3433_AXIS_Y], obj->fir.sum[MC3433_AXIS_Z]);
            GSE_LOG("avg = [%5d %5d %5d]\n", obj->fir.sum[MC3433_AXIS_X]/len, obj->fir.sum[MC3433_AXIS_Y]/len, obj->fir.sum[MC3433_AXIS_Z]/len);
        }
        return snprintf(buf, PAGE_SIZE, "%d\n", atomic_read(&obj->firlen));
    #else
        GSE_LOG("fwq show_firlen_value \n");
        return snprintf(buf, PAGE_SIZE, "not support\n");
    #endif
}

/*****************************************
 *** store_firlen_value
 *****************************************/
static ssize_t store_firlen_value(struct device_driver *ddri, const char *buf, size_t count)
{
    #ifdef _MC3433_SUPPORT_LPF_
    struct i2c_client *client = MC3433_i2c_client;  
    struct MC3433_i2c_data *obj = i2c_get_clientdata(client);
    int firlen = 0;

    GSE_LOG("fwq store_firlen_value \n");
    
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
	if(0 == firlen)
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

/*****************************************
 *** show_trace_value
 *****************************************/
static ssize_t show_trace_value(struct device_driver *ddri, char *buf)
{
	ssize_t res = 0;
	struct MC3433_i2c_data *obj = MC3433_obj_i2c_data;

    GSE_LOG("fwq show_trace_value \n");

	if (obj == NULL)
	{
		GSE_ERR("i2c_data obj is null!!\n");
		return 0;
	}
	
	res = snprintf(buf, PAGE_SIZE, "0x%04X\n", atomic_read(&obj->trace));     
	return res;    
}

/*****************************************
 *** store_trace_value
 *****************************************/
static ssize_t store_trace_value(struct device_driver *ddri, const char *buf, size_t count)
{
	struct MC3433_i2c_data *obj = MC3433_obj_i2c_data;
	int trace = 0;

    GSE_LOG("fwq store_trace_value \n");

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

/*****************************************
 *** show_status_value
 *****************************************/
static ssize_t show_status_value(struct device_driver *ddri, char *buf)
{
	ssize_t len = 0;    
	struct MC3433_i2c_data *obj = MC3433_obj_i2c_data;

    GSE_LOG("fwq show_status_value \n");

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

/*****************************************
 *** show_power_status
 *****************************************/
static ssize_t show_power_status(struct device_driver *ddri, char *buf)
{
	ssize_t res = 0;
	u8 uData = 0;
	struct MC3433_i2c_data *obj = MC3433_obj_i2c_data;

	if (obj == NULL)
	{
		GSE_ERR("i2c_data obj is null!!\n");
		return 0;
	}
	hwmsen_read_byte(obj->client, MC3433_REG_MODE_FEATURE, &uData);
	
	res = snprintf(buf, PAGE_SIZE, "0x%04X\n", uData);     
	return res;   
}

/*****************************************
 *** show_version_value
 *****************************************/
static ssize_t show_version_value(struct device_driver *ddri, char *buf)
{
	if ( 1 == VIRTUAL_Z)	
		return snprintf(buf, PAGE_SIZE, "%s\n", MC3433_DEV_DRIVER_VERSION_VIRTUAL_Z);
	else
		return snprintf(buf, PAGE_SIZE, "%s\n", MC3433_DEV_DRIVER_VERSION);	
}

/*****************************************
 *** show_chip_id
 *****************************************/
static ssize_t show_chip_id(struct device_driver *ddri, char *buf)
{
	struct MC3433_i2c_data   *_pt_i2c_data = MC3433_obj_i2c_data;

	return MC3433_Read_Chip_ID(_pt_i2c_data->client, buf);
}

/*****************************************
 *** show_virtual_z
 *****************************************/
static ssize_t show_virtual_z(struct device_driver *ddri, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%s\n", VIRTUAL_Z == 1 ? "Virtual Z Support": "No Virtual Z Support");     
}

/*****************************************
 *** show_regiter_map
 *****************************************/
static ssize_t show_regiter_map(struct device_driver *ddri, char *buf)
{
    u8         _bIndex       = 0;
    u8         _baRegMap[64] = { 0 };
    ssize_t    _tLength      = 0;

    struct i2c_client *client = MC3433_i2c_client;

    if ((0xA5 == buf[0]) && (0x7B == buf[1]) && (0x40 == buf[2]))
    {
        MC3433_MUTEX_LOCK();
        MC3433_Read_Reg_Map(client, buf);
        MC3433_MUTEX_UNLOCK();

        buf[0x21] = s_baOTP_OffsetData[0];
        buf[0x22] = s_baOTP_OffsetData[1];
        buf[0x23] = s_baOTP_OffsetData[2];
        buf[0x24] = s_baOTP_OffsetData[3];
        buf[0x25] = s_baOTP_OffsetData[4];
        buf[0x26] = s_baOTP_OffsetData[5];

        _tLength = 64;
    }
    else
    {
        MC3433_MUTEX_LOCK();
        MC3433_Read_Reg_Map(client, _baRegMap);
        MC3433_MUTEX_UNLOCK();
    
        for (_bIndex = 0; _bIndex < 64; _bIndex++)
            _tLength += snprintf((buf + _tLength), (PAGE_SIZE - _tLength), "Reg[0x%02X]: 0x%02X\n", _bIndex, _baRegMap[_bIndex]); 
    }

    return (_tLength);
}

/*****************************************
 *** store_regiter_map
 *****************************************/
static ssize_t store_regiter_map(struct device_driver *ddri, const char *buf, size_t count)
{
    // reserved
    //GSE_LOG("[%s] buf[0]: 0x%02X\n", __FUNCTION__, buf[0]);

    return count;
}

/*****************************************
 *** show_chip_orientation
 *****************************************/
static ssize_t show_chip_orientation(struct device_driver *ptDevDrv, char *pbBuf)
{
    ssize_t          _tLength = 0;
    struct acc_hw   *_ptAccelHw = get_cust_acc_hw();
    
    GSE_LOG("[%s] default direction: %d\n", __FUNCTION__, _ptAccelHw->direction);
    
    _tLength = snprintf(pbBuf, PAGE_SIZE, "default direction = %d\n", _ptAccelHw->direction);     
    
    return (_tLength);    
}

/*****************************************
 *** store_chip_orientation
 *****************************************/
static ssize_t store_chip_orientation(struct device_driver *ptDevDrv, const char *pbBuf, size_t tCount)
{
    int                       _nDirection = 0;
    struct MC3433_i2c_data   *_pt_i2c_obj = MC3433_obj_i2c_data;

    if (NULL == _pt_i2c_obj)
        return (0);

    if (1 == sscanf(pbBuf, "%d", &_nDirection))
    {
        if (hwmsen_get_convert(_nDirection, &_pt_i2c_obj->cvt))
            GSE_ERR("ERR: fail to set direction\n");
    }

    GSE_LOG("[%s] set direction: %d\n", __FUNCTION__, _nDirection);

    return (tCount);
}

/*****************************************
 *** show_accuracy_status
 *****************************************/
static ssize_t show_accuracy_status(struct device_driver *ddri, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", s_bAccuracyStatus);     
}

/*****************************************
 *** store_accuracy_status
 *****************************************/
static ssize_t store_accuracy_status(struct device_driver *ddri, const char *buf, size_t count)
{
    int    _nAccuracyStatus = 0;
    
    if (1 != sscanf(buf, "%d", &_nAccuracyStatus))
    {
        GSE_ERR("incorrect argument\n");
        return count;
    }

    if (SENSOR_STATUS_ACCURACY_HIGH < _nAccuracyStatus)
    {
        GSE_ERR("illegal accuracy status\n");
        return count;
    }

    s_bAccuracyStatus = ((int8_t) _nAccuracyStatus);

    return count;
}

/*****************************************
 *** show_selfcheck_value
 *****************************************/
static ssize_t show_selfcheck_value(struct device_driver *ptDevDriver, char *pbBuf)
{
    struct i2c_client   *_pt_i2c_client = MC3433_i2c_client;

    //GSE_LOG("[%s] 0x%02X\n", __FUNCTION__, pbBuf[0]);

    MC3433_MUTEX_LOCK();
    MC3433_SelfCheck(_pt_i2c_client, pbBuf);
    MC3433_Init_client(_pt_i2c_client, 0);
    MC3433_MUTEX_UNLOCK();

	return (64);
}

/*****************************************
 *** store_selfcheck_value
 *****************************************/
static ssize_t store_selfcheck_value(struct device_driver *ddri, const char *buf, size_t count)
{
    // reserved
    //GSE_LOG("[%s] buf[0]: 0x%02X\n", __FUNCTION__, buf[0]);

    return count;
}

/*****************************************
 *** show_chip_validate_value
 *****************************************/
static ssize_t show_chip_validate_value(struct device_driver *ptDevDriver, char *pbBuf)
{
    unsigned char    _bChipValidation = 0;

    _bChipValidation = MC3433_ValidateSensorIC(s_bPCODE);

    return snprintf(pbBuf, PAGE_SIZE, "%d\n", _bChipValidation);        
}

/*****************************************
 *** show_pdoc_enable_value
 *****************************************/
static ssize_t show_pdoc_enable_value(struct device_driver *ptDevDriver, char *pbBuf)
{
    #ifdef _MC3433_SUPPORT_PERIODIC_DOC_
        return snprintf(pbBuf, PAGE_SIZE, "%d\n", s_bIsPDOC_Enabled);        
    #else
        unsigned char    _bIsPDOC_Enabled = false;
        
        return snprintf(pbBuf, PAGE_SIZE, "%d\n", _bIsPDOC_Enabled);		 
    #endif
}

/*****************************************
 *** DRIVER ATTRIBUTE LIST TABLE
 *****************************************/
static DRIVER_ATTR(chipinfo   ,           S_IRUGO, show_chipinfo_value,   NULL                  );
static DRIVER_ATTR(sensordata ,           S_IRUGO, show_sensordata_value, NULL                  );
static DRIVER_ATTR(cali       , S_IWUSR | S_IRUGO, show_cali_value,       store_cali_value      );
static DRIVER_ATTR(selftest   , S_IWUSR | S_IRUGO, show_selftest_value,   store_selftest_value  );
static DRIVER_ATTR(firlen     , S_IWUSR | S_IRUGO, show_firlen_value,     store_firlen_value    );
static DRIVER_ATTR(trace      , S_IWUSR | S_IRUGO, show_trace_value,      store_trace_value     );
static DRIVER_ATTR(status     ,           S_IRUGO, show_status_value,     NULL                  );
static DRIVER_ATTR(power      ,           S_IRUGO, show_power_status,     NULL                  );
static DRIVER_ATTR(version    ,           S_IRUGO, show_version_value,    NULL                  );
static DRIVER_ATTR(chipid     ,           S_IRUGO, show_chip_id,          NULL                  );
static DRIVER_ATTR(virtualz   ,           S_IRUGO, show_virtual_z,        NULL                  );
static DRIVER_ATTR(regmap     , S_IWUSR | S_IRUGO, show_regiter_map,      store_regiter_map     );
static DRIVER_ATTR(orientation, S_IWUSR | S_IRUGO, show_chip_orientation, store_chip_orientation);
static DRIVER_ATTR(accuracy   , S_IWUSR | S_IRUGO, show_accuracy_status , store_accuracy_status );
static DRIVER_ATTR(selfcheck  , S_IWUSR | S_IRUGO, show_selfcheck_value , store_selfcheck_value );
static DRIVER_ATTR(validate   ,           S_IRUGO, show_chip_validate_value, NULL                  );
static DRIVER_ATTR(pdoc       ,           S_IRUGO, show_pdoc_enable_value  , NULL                  );

static struct driver_attribute   *MC3433_attr_list[] = {
   //&driver_attr_chipinfo,
   //&driver_attr_sensordata,
   &driver_attr_cali,
   //&driver_attr_selftest,
   //&driver_attr_firlen,
   &driver_attr_trace,
   &driver_attr_status,
   //&driver_attr_power, 
   &driver_attr_version, 
   &driver_attr_chipid,
   //&driver_attr_virtualz,
   //&driver_attr_regmap,
   &driver_attr_orientation,
   &driver_attr_accuracy,
  // &driver_attr_selfcheck,
   &driver_attr_validate,
   //&driver_attr_pdoc,
};

/*****************************************
 *** MC3433_create_attr
 *****************************************/
static int MC3433_create_attr(struct device_driver *driver) 
{
	int idx, err = 0;
	int num = (int)(sizeof(MC3433_attr_list)/sizeof(MC3433_attr_list[0]));
	if (driver == NULL)
	{
		return -EINVAL;
	}

	for(idx = 0; idx < num; idx++)
	{
		if((err = driver_create_file(driver, MC3433_attr_list[idx])))
		{            
			GSE_ERR("driver_create_file (%s) = %d\n", MC3433_attr_list[idx]->attr.name, err);
			break;
		}
	}    
	return err;
}

/*****************************************
 *** MC3433_delete_attr
 *****************************************/
static int MC3433_delete_attr(struct device_driver *driver)
{
	int idx ,err = 0;
	int num = (int)(sizeof(MC3433_attr_list)/sizeof(MC3433_attr_list[0]));

	if(driver == NULL)
	{
		return -EINVAL;
	}
	

	for(idx = 0; idx < num; idx++)
	{
		driver_remove_file(driver, MC3433_attr_list[idx]);
	}
	

	return err;
}

/*****************************************
 *** gsensor_operate
 *****************************************/
static int gsensor_operate(void* self, uint32_t command, void* buff_in, int size_in,
		void* buff_out, int size_out, int* actualout)
{
	int err = 0;
	int value = 0;	
	struct MC3433_i2c_data *priv = (struct MC3433_i2c_data*)self;
	hwm_sensor_data* gsensor_data = NULL;
	char buff[MC3433_BUF_SIZE] = { 0 };
	
	GSE_FUN(f);
	switch (command)
	{
		case SENSOR_DELAY:
			GSE_LOG("fwq set delay\n");
			if((buff_in == NULL) || (size_in < sizeof(int)))
			{
				GSE_ERR("Set delay parameter error!\n");
				err = -EINVAL;
			}
			else
			{
				value = *(int *)buff_in;
				if(value >= 50)
				{
				    atomic_set(&priv->filter, 0);
				}
				else
				{			
	    			    #if defined(_MC3433_SUPPORT_LPF_)  
	 				priv->fir.num = 0;
					priv->fir.idx = 0;
					priv->fir.sum[MC3433_AXIS_X] = 0;
					priv->fir.sum[MC3433_AXIS_Y] = 0;
					priv->fir.sum[MC3433_AXIS_Z] = 0;
					atomic_set(&priv->filter, 1);
                                    #endif					
				}
			}
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

                		GSE_LOG("fwq sensor enable gsensor: %d\n", value);

				if(((value == 0) && (((bool)atomic_read(&priv->sensor_power)) == false)) ||((value == 1) && (((bool)atomic_read(&priv->sensor_power)) == true)))
				{
					GSE_LOG("Gsensor device have updated!\n");
				}
				else
				{
                    			MC3433_MUTEX_LOCK();
					err = MC3433_SetPowerMode( priv->client, !((bool)atomic_read(&priv->sensor_power)));
                   			MC3433_MUTEX_UNLOCK();
				}
            
		        #ifdef _MC3433_SUPPORT_PERIODIC_DOC_
				if (0 == value)
				atomic_set(&s_t_MC3433_open_status, 0);
				else
				atomic_set(&s_t_MC3433_open_status, 1);

				wake_up(&wq_MC3433_open_status);
		        #endif
			}
			break;

		case SENSOR_GET_DATA:
			//GSE_LOG("[LLF]fwq sensor operate get data--------------------------------\n");
			if((buff_out == NULL) || (size_out< sizeof(hwm_sensor_data)))
			{
				GSE_ERR("get sensor data parameter error!\n");
				err = -EINVAL;
			}
			else
			{
				gsensor_data = (hwm_sensor_data *)buff_out;
                		MC3433_MUTEX_LOCK();
				MC3433_ReadSensorData(priv->client, buff, MC3433_BUF_SIZE);
               			MC3433_MUTEX_UNLOCK();
				sscanf(buff, "%x %x %x", &gsensor_data->values[0], &gsensor_data->values[1], &gsensor_data->values[2]);				
                		gsensor_data->status = s_bAccuracyStatus;//SENSOR_STATUS_ACCURACY_MEDIUM
				gsensor_data->value_divide = 1000;
				//printk("MC3433_ReadSensorData: X =:%d, Y=: %d, Z=: %d  sensor_status:%d \n",gsensor_data->values[0],gsensor_data->values[1],gsensor_data->values[2],gsensor_data->status);
			}
			break;
		default:
			GSE_ERR("gsensor operate function no this parameter %d!\n", command);
			err = -1;
			break;
	}
	
	return err;
}

/*****************************************
 *** MC3433_open
 *****************************************/
static int MC3433_open(struct inode *inode, struct file *file)
{
	file->private_data = MC3433_i2c_client;

	if(file->private_data == NULL)
	{
		GSE_ERR("null pointer!!\n");
		return -EINVAL;
	}
	return nonseekable_open(inode, file);
}

/*****************************************
 *** MC3433_release
 *****************************************/
static int MC3433_release(struct inode *inode, struct file *file)
{
	file->private_data = NULL;
	return 0;
}

/*****************************************
 *** MC3433_ioctl
 *****************************************/
static long MC3433_unlocked_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct i2c_client *client = (struct i2c_client*)file->private_data;
	struct MC3433_i2c_data *obj = (struct MC3433_i2c_data*)i2c_get_clientdata(client);	
	char strbuf[MC3433_BUF_SIZE] = {0};
	void __user *data = NULL;
	SENSOR_DATA sensor_data = {0};
	long err = 0;
	int cali[3] = {0};

        #ifdef _MC3433_SUPPORT_DOT_CALIBRATION_
            int prod = -1;
            int tempZ = 0;
            unsigned char _bTempPCode = 0x00;
        #endif

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

	switch(cmd)
	{
		case GSENSOR_IOCTL_INIT:
			GSE_LOG("fwq GSENSOR_IOCTL_INIT\n");
			MC3433_MUTEX_LOCK();
			MC3433_Init_client(client, 0);			
			MC3433_MUTEX_UNLOCK();
			break;

		case GSENSOR_IOCTL_READ_CHIPINFO:
			GSE_LOG("fwq GSENSOR_IOCTL_READ_CHIPINFO\n");
			data = (void __user *) arg;
			if(data == NULL)
			{
				err = -EINVAL;
				break;	  
			}
			
			MC3433_ReadChipInfo(client, strbuf, MC3433_BUF_SIZE);
			if(copy_to_user(data, strbuf, strlen(strbuf)+1))
			{
				err = -EFAULT;
				break;
			}				 
			break;	  

		case GSENSOR_IOCTL_READ_SENSORDATA:
			data = (void __user *) arg;
			if(data == NULL)
			{
				err = -EINVAL;
				break;	  
			}
			MC3433_MUTEX_LOCK();
			#ifdef _MC3433_SUPPORT_APPLY_AVERAGE_AGORITHM_
				MC3433_ReadRawData(client, strbuf);
			#else
				MC3433_ReadSensorData(client, strbuf, MC3433_BUF_SIZE);
			#endif
				MC3433_MUTEX_UNLOCK();
			if(copy_to_user(data, strbuf, strlen(strbuf)+1))
			{
				err = -EFAULT;
				break;	  
			}				 
			break;

		case GSENSOR_IOCTL_READ_GAIN:
			GSE_LOG("fwq GSENSOR_IOCTL_READ_GAIN\n");
			data = (void __user *) arg;
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

		case GSENSOR_IOCTL_READ_OFFSET:
			GSE_LOG("fwq GSENSOR_IOCTL_READ_OFFSET\n");
			data = (void __user *) arg;
			if(data == NULL)
			{
				err = -EINVAL;
				break;	  
			}
			
			if(copy_to_user(data, &gsensor_offset, sizeof(GSENSOR_VECTOR3D)))
			{
				err = -EFAULT;
				break;
			}				 
			break;

		case GSENSOR_IOCTL_READ_RAW_DATA:
			GSE_LOG("fwq GSENSOR_IOCTL_READ_RAW_DATA\n");
			data = (void __user *) arg;
			if(data == NULL)
			{
				err = -EINVAL;
				break;	  
			}
            		MC3433_MUTEX_LOCK();
			MC3433_ReadRawData(client, strbuf);
            		MC3433_MUTEX_UNLOCK();
			if(copy_to_user(data, strbuf, strlen(strbuf)+1))
			{
				err = -EFAULT;
				break;	  
			}
			break;	  

		case GSENSOR_IOCTL_SET_CALI:
			GSE_LOG("fwq GSENSOR_IOCTL_SET_CALI!!\n");
			data = (void __user*)arg;
			if(data == NULL)
			{
				err = -EINVAL;
				break;	  
			}
			if(copy_from_user(&sensor_data, data, sizeof(sensor_data)))
			{
				err = -EFAULT;
				break;	  
			}
			if(atomic_read(&obj->suspend))
			{
				GSE_ERR("Perform calibration in suspend state!!\n");
				err = -EINVAL;
			}
			else
			{
				obj->cali_sw[MC3433_AXIS_X] += sensor_data.x;
				obj->cali_sw[MC3433_AXIS_Y] += sensor_data.y;
				obj->cali_sw[MC3433_AXIS_Z] += sensor_data.z;
				
				cali[MC3433_AXIS_X] = sensor_data.x * gsensor_gain.x / GRAVITY_EARTH_1000;
				cali[MC3433_AXIS_Y] = sensor_data.y * gsensor_gain.y / GRAVITY_EARTH_1000;
				cali[MC3433_AXIS_Z] = sensor_data.z * gsensor_gain.z / GRAVITY_EARTH_1000;	
			  
                		MC3433_MUTEX_LOCK();
				err = MC3433_WriteCalibration(client, cali);			 
                		MC3433_MUTEX_UNLOCK();
			}
			break;

		case GSENSOR_IOCTL_CLR_CALI:
			GSE_LOG("fwq GSENSOR_IOCTL_CLR_CALI!!\n");
            		MC3433_MUTEX_LOCK();
			err = MC3433_ResetCalibration(client);
            		MC3433_MUTEX_UNLOCK();
			break;

		case GSENSOR_IOCTL_GET_CALI:
			GSE_LOG("fwq MC3433 GSENSOR_IOCTL_GET_CALI\n");
			data = (void __user*)arg;
			if(data == NULL)
			{
				err = -EINVAL;
				break;	  
			}
//			if((err = MC3433_ReadCalibration(client, cali)))
//			{
//				break;
//			}

			sensor_data.x = obj->cali_sw[MC3433_AXIS_X];
			sensor_data.y = obj->cali_sw[MC3433_AXIS_Y];
			sensor_data.z = obj->cali_sw[MC3433_AXIS_Z];
			if(copy_to_user(data, &sensor_data, sizeof(sensor_data)))
			{
				err = -EFAULT;
				break;
			}		
			break;

        //add in Sensors_io.h
        // ==============================================================================
        #ifdef _MC3433_SUPPORT_DOT_CALIBRATION_
        
		case GSENSOR_MCUBE_IOCTL_SET_CALI:
			GSE_LOG("fwq GSENSOR_MCUBE_IOCTL_SET_CALI!!\n");
			data = (void __user*)arg;
			if(data == NULL)
			{
				err = -EINVAL;
				break;	  
			}
			if(copy_from_user(&sensor_data, data, sizeof(sensor_data)))
			{
				err = -EFAULT;
				break;	  
			}
			if(atomic_read(&obj->suspend))
			{
				GSE_ERR("Perform calibration in suspend state!!\n");
				err = -EINVAL;
			}
			else
			{
				cali[MC3433_AXIS_X] = sensor_data.x * gsensor_gain.x / GRAVITY_EARTH_1000;
				cali[MC3433_AXIS_Y] = sensor_data.y * gsensor_gain.y / GRAVITY_EARTH_1000;
				cali[MC3433_AXIS_Z] = sensor_data.z * gsensor_gain.z / GRAVITY_EARTH_1000;	
			  
                		MC3433_MUTEX_LOCK();
				err = MC3433_WriteCalibration(client, cali);			 
                		MC3433_MUTEX_UNLOCK();
			}
			break;

		case GSENSOR_IOCTL_SET_CALI_MODE:
			GSE_LOG("fwq MC3433 GSENSOR_IOCTL_SET_CALI_MODE\n");
			break;

		case GSENSOR_MCUBE_IOCTL_READ_RBM_DATA:
			GSE_LOG("fwq GSENSOR_MCUBE_IOCTL_READ_RBM_DATA\n");
			data = (void __user *) arg;
			if(data == NULL)
			{
				err = -EINVAL;
				break;	  
			}
            		MC3433_MUTEX_LOCK();
			MC3433_ReadRBMData(client, strbuf);
            		MC3433_MUTEX_UNLOCK();
			if(copy_to_user(data, strbuf, strlen(strbuf)+1))
			{
				err = -EFAULT;
				break;	  
			}
			break;

		case GSENSOR_MCUBE_IOCTL_SET_RBM_MODE:
			GSE_LOG("fwq GSENSOR_MCUBE_IOCTL_SET_RBM_MODE\n");
            		MC3433_MUTEX_LOCK();
			MC3433_rbm(client,1);
            		MC3433_MUTEX_UNLOCK();
			break;

		case GSENSOR_MCUBE_IOCTL_CLEAR_RBM_MODE:
			GSE_LOG("fwq GSENSOR_MCUBE_IOCTL_CLEAR_RBM_MODE\n");
            		MC3433_MUTEX_LOCK();
			MC3433_rbm(client,0);
            		MC3433_MUTEX_UNLOCK();
			break;

		case GSENSOR_MCUBE_IOCTL_REGISTER_MAP:
			GSE_LOG("fwq GSENSOR_MCUBE_IOCTL_REGISTER_MAP\n");

			MC3433_Read_Reg_Map(client, NULL);

			break;

		case GSENSOR_MCUBE_IOCTL_READ_PRODUCT_ID:
			GSE_LOG("fwq GSENSOR_MCUBE_IOCTL_READ_PRODUCT_ID\n");
			data = (void __user *) arg;
			if(data == NULL)
			{
				err = -EINVAL;
				break;	  
			}

            		if (MC3433_RETCODE_SUCCESS != (prod = MC3433_ValidateSensorIC(s_bPCODE)))
				GSE_LOG("Not mCube accelerometers!\n");

			if(copy_to_user(data, &prod, sizeof(prod)))
			{
				err = -EFAULT;
				break;
			}				 
			break;

		case GSENSOR_MCUBE_IOCTL_READ_FILEPATH:
			GSE_LOG("fwq GSENSOR_MCUBE_IOCTL_READ_FILEPATH\n");
			data = (void __user *) arg;
			if(data == NULL)
			{
				err = -EINVAL;
				break;	  
			}
			
			if(copy_to_user(data, file_path, (strlen(file_path)+1)))
			{
				err = -EFAULT;
				break;
			}				 
			break;		

		case GSENSOR_MCUBE_IOCTL_VIRTUAL_Z:
			GSE_LOG("fwq GSENSOR_MCUBE_IOCTL_VIRTUAL_Z\n");
			data = (void __user *) arg;
			tempZ = VIRTUAL_Z;
			if(data == NULL)
			{
				err = -EINVAL;
				break;	  
			}
			if(copy_to_user(data, &tempZ, sizeof(tempZ)))
			{
				err = -EFAULT;
				break;
			}				 
			break;

		case GSENSOR_MCUBE_IOCTL_READ_PCODE:
			GSE_LOG("fwq GSENSOR_MCUBE_IOCTL_READ_PCODE\n");
			data = (void __user *) arg;
			if(data == NULL)
			{
				err = -EINVAL;
				break;	  
			}

			_bTempPCode = s_bPCODE;
			GSE_LOG("mCube PCode = %2x!\n", _bTempPCode);
			if(copy_to_user(data, &_bTempPCode, sizeof(_bTempPCode)))
			{
				err = -EFAULT;
				break;
			}				 
			break;

        #endif    // END of _MC3433_SUPPORT_DOT_CALIBRATION_

        // ==============================================================================
        #ifdef _MC3433_SUPPORT_PERIODIC_DOC_
        
		case GSENSOR_MCUBE_IOCTL_GET_OFLAG:
		    {
		        int            _nSensorsOpenStatus = 0;
		        void __user   *_pArg = ((void __user *) arg);
		        
		        GSE_LOG("[%s] GSENSOR_MCUBE_IOCTL_GET_OFLAG\n", __func__);
		        
		        _nSensorsOpenStatus = MC3433_GetOpenStatus();			
		        
		        if(copy_to_user(_pArg, &_nSensorsOpenStatus, sizeof(_nSensorsOpenStatus)))
		            return (-EFAULT);
		    }
		    break; 	   

        #endif    // END of _MC3433_SUPPORT_PERIODIC_DOC_
			
		default:
			GSE_ERR("unknown IOCTL: 0x%08x\n", cmd);
			err = -ENOIOCTLCMD;
			break;
			
	}

	return err;
}

/*****************************************
 *** MC3433_reset
 *****************************************/
static void MC3433_reset(struct i2c_client *client)
{
	GSE_LOG("[%s] reset's i2c addr: 0x%X\n", __FUNCTION__, client->addr);

	unsigned char    _baBuf[2] = { 0 };

	_baBuf[1] = 0x43;
	_baBuf[0] = MC3433_REG_MODE_FEATURE;
	i2c_master_send(client, _baBuf, 0x2);

	hwmsen_read_block(client, 0x04, _baBuf, 0x01);

	if (0x00 == (_baBuf[0] & 0x40))
	{
		_baBuf[0] = 0x6D; 
		hwmsen_write_block(client, 0x1B, _baBuf, 0x01);

		_baBuf[0] = 0x43; 
		hwmsen_write_block(client, 0x1B, _baBuf, 0x01);
	}

	_baBuf[0] = 0x43;
	hwmsen_write_block(client, 0x07, _baBuf, 1);	

	_baBuf[0] = 0x80;
	hwmsen_write_block(client, 0x1C, _baBuf, 1);	

	_baBuf[0] = 0x80;
	hwmsen_write_block(client, 0x17, _baBuf, 1);	

	msleep(5);

	_baBuf[0] = 0x00;
	hwmsen_write_block(client, 0x1C, _baBuf, 1);	

	_baBuf[0] = 0x00;
	hwmsen_write_block(client, 0x17, _baBuf, 1);	

	msleep(5);

	hwmsen_read_block(client, 0x21, offset_buf, 6);

	hwmsen_read_block(client, 0x04, _baBuf, 0x01);

	if (_baBuf[0] & 0x40)
	{
		_baBuf[0] = 0x6D; 
		hwmsen_write_block(client, 0x1B, _baBuf, 0x01);

		_baBuf[0] = 0x43; 
		hwmsen_write_block(client, 0x1B, _baBuf, 0x01);
	}

	_baBuf[1] = 0x41;
	_baBuf[0] = MC3433_REG_MODE_FEATURE;
	i2c_master_send(client, _baBuf, 0x2);
}

/*****************************************
 *** STRUCT:: MC3433_fops
 *****************************************/
static struct file_operations    MC3433_fops = {
	.owner          = THIS_MODULE,
	.open           = MC3433_open,
	.release        = MC3433_release,
	.unlocked_ioctl = MC3433_unlocked_ioctl,
};

/*****************************************
 *** STRUCT:: MC3433_device
 *****************************************/
static struct miscdevice    MC3433_device = {
        .minor = MISC_DYNAMIC_MINOR,
        .name  = "gsensor",
        .fops  = &MC3433_fops,
};

#ifndef CONFIG_HAS_EARLYSUSPEND
/*****************************************
 *** MC3433_suspend
 *****************************************/
static int MC3433_suspend(struct i2c_client *client, pm_message_t msg) 
{
	struct MC3433_i2c_data *obj = i2c_get_clientdata(client);    
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

        MC3433_MUTEX_LOCK();
        err = MC3433_SetPowerMode(client, false);
        MC3433_MUTEX_UNLOCK();

        if (err)
        {
            GSE_ERR("write power control fail!!\n");
            return err;
        }     

		MC3433_power(obj->hw, 0);
	}
	return err;
}

/*****************************************
 *** MC3433_resume
 *****************************************/
static int MC3433_resume(struct i2c_client *client)
{
	struct MC3433_i2c_data *obj = i2c_get_clientdata(client);        
	int err;
	GSE_FUN();

	if(obj == NULL)
	{
		GSE_ERR("null pointer!!\n");
		return -EINVAL;
	}

	MC3433_power(obj->hw, 1);

	MC3433_MUTEX_LOCK();
	err = MC3433_Init_client(client, 0);

	if(err)
	{
	MC3433_MUTEX_UNLOCK();
	GSE_ERR("initialize client fail!!\n");
	return err;        
	}

	err = MC3433_SetPowerMode(client, true);
	MC3433_MUTEX_UNLOCK();

	if (err)
	{
	GSE_ERR("write power control fail!!\n");
	return err;
	}     

	atomic_set(&obj->suspend, 0);

	return 0;
}
#else
/*****************************************
 *** MC3433_early_suspend
 *****************************************/
static void MC3433_early_suspend(struct early_suspend *h) 
{
	struct MC3433_i2c_data *obj = container_of(h, struct MC3433_i2c_data, early_drv);   
	int err;
	GSE_FUN();    

	if(obj == NULL)
	{
		GSE_ERR("null pointer!!\n");
		return;
	}
	atomic_set(&obj->suspend, 1); 
	/*
	if(err = hwmsen_write_byte(obj->client, MC3433_REG_POWER_CTL, 0x00))
	{
		GSE_ERR("write power control fail!!\n");
		return;
	}  
	*/
	MC3433_MUTEX_LOCK_RETURN_VOID();
	err = MC3433_SetPowerMode(obj->client, false);
	MC3433_MUTEX_UNLOCK();

	if (err)
	{
		GSE_ERR("write power control fail!!\n");
		return;
	}     

	MC3433_power(obj->hw, 0);
}

/*****************************************
 *** MC3433_late_resume
 *****************************************/
static void MC3433_late_resume(struct early_suspend *h)
{
	struct MC3433_i2c_data *obj = container_of(h, struct MC3433_i2c_data, early_drv);         
	int err;
	GSE_FUN();

	if(obj == NULL)
	{
		GSE_ERR("null pointer!!\n");
		return;
	}

	MC3433_power(obj->hw, 1);

	MC3433_MUTEX_LOCK_RETURN_VOID();
	err = MC3433_Init_client(obj->client, 0);

	if(err)
	{
	MC3433_MUTEX_UNLOCK();
	GSE_ERR("initialize client fail!!\n");
	return;
	}

	err = MC3433_SetPowerMode(obj->client, true);
	MC3433_MUTEX_UNLOCK();

	if (err)
	{
	GSE_ERR("write power control fail!!\n");
	return;
	}     


	atomic_set(&obj->suspend, 0);    
}
#endif

/*****************************************
 *** _MC3433_i2c_i2c_checkID
 *****************************************/

static int MC3433_i2c_checkID(struct i2c_client *client)
{
	unsigned char    baDataBuf[2] = {0};

	s_bPCODE = 0x00;

        client->addr = MC3433_I2C_SLAVE_WRITE_ADDR>>1;

        //GSE_LOG("[%s] probing addr: 0x%X\n", __FUNCTION__, client->addr);

        baDataBuf[0] = 0x3B;
        if (0 > i2c_master_send(client, &(baDataBuf[0]), 1))
        {
            GSE_ERR("ERR: addr: 0x%X fail to communicate-2!\n", client->addr);
            return (MC3433_RETCODE_ERROR_I2C);
        }
    
        if (0 > i2c_master_recv(client, &(baDataBuf[0]), 1))
        {
            GSE_ERR("ERR: addr: 0x%X fail to communicate-3!\n", client->addr);
	    return (MC3433_RETCODE_ERROR_I2C);
        }

        if (MC3433_RETCODE_SUCCESS == MC3433_ValidateSensorIC(baDataBuf[0]))
        {
            //GSE_LOG("[%s] addr: 0x%X confirmed ok to use.\n", __FUNCTION__, client->addr);

	    if((baDataBuf[0]& 0xf1)==MC3433_PCODE_3510C ||(baDataBuf[0]& 0xf1)==MC3433_PCODE_3530C)
	    {
            s_bPCODE = (baDataBuf[0]&0xf1);//modify by xiaopu.zhu for ingore check id error
	    }
	    else
	    {
	    	s_bPCODE = baDataBuf[0]; 
	    }
            MC3433_SaveDefaultOffset(client);

            return (MC3433_RETCODE_SUCCESS);
        }

	return (MC3433_RETCODE_ERROR_I2C);
}

static int MC3433_i2c_detect(struct i2c_client *client, int kind,struct i2c_board_info *info) 
{    
	strcpy(info->type, MC3433_DEV_NAME);
	return 0;
}

/*****************************************
 *** MC3433_i2c_probe
 *****************************************/
static int MC3433_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct i2c_client *new_client;
	struct MC3433_i2c_data *obj;
	struct hwmsen_object sobj;
	int err = 0;
	GSE_FUN();
/*
    if (MC3433_RETCODE_SUCCESS != _MC3433_i2c_auto_probe(client))
    {
        //GSE_ERR("ERR: fail to probe mCube sensor!\n");
        goto exit;
    }
*/
    	GSE_LOG("[%s] confirmed i2c addr: 0x%X\n", __FUNCTION__, client->addr);

	if(!(obj = kzalloc(sizeof(*obj), GFP_KERNEL)))
	{
		err = -ENOMEM;
		goto exit;
	}
	
	memset(obj, 0, sizeof(struct MC3433_i2c_data));

	obj->hw = get_cust_acc_hw();
	
	if((err = hwmsen_get_convert(obj->hw->direction, &obj->cvt)))
	{
		GSE_ERR("invalid direction: %d\n", obj->hw->direction);
		goto exit_kfree;
	}

	MC3433_obj_i2c_data = obj;
	obj->client = client;
	new_client = obj->client;
	i2c_set_clientdata(new_client,obj);
	
	atomic_set(&obj->trace, 0);
	atomic_set(&obj->suspend, 0);
	atomic_set(&obj->sensor_power,  1);
	
    #ifdef _MC3433_SUPPORT_LPF_
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

	MC3433_i2c_client = new_client;	

	msleep(500);
	MC3433_reset(new_client);
/*
    if (MC3433_RETCODE_SUCCESS != _MC3433_i2c_auto_probe(client))
    {
        //GSE_ERR("ERR: fail to probe mCube sensor!\n");
        goto exit_init_failed;
    }
*/
    //GSE_LOG("[%s] 2nd confirmed i2c addr: 0x%X\n", __FUNCTION__, client->addr);
//-----------------------------readID;
	if(err = MC3433_i2c_checkID(client))
	{
		GSE_ERR("=============checkID failed!--------------\n");
		goto exit_kfree;
	}

	hwmsen_read_block(client, 0x21, offset_buf, 6);

	if((err = MC3433_Init_client(new_client, 1)))
	{
		goto exit_init_failed;
	}
	gsensor_init_status=MC3433_RETCODE_SUCCESS;
	
	MC3433_MUTEX_INIT();

	if((err = misc_register(&MC3433_device)))
	{
		GSE_ERR("MC3433_device register failed\n");
		goto exit_misc_device_register_failed;
	}
/*
    #ifdef MTK_AUTO_DETECT_ACCELEROMETER
        if((err = MC3433_create_attr(&MC3433_init_info.platform_diver_addr->driver)));
    #else
        if((err = MC3433_create_attr(&MC3433_gsensor_driver.driver)));
    #endif
        {
            GSE_ERR("create attribute err = %d\n", err);
            goto exit_create_attr_failed;
        }

	if(driver_create_file(&MC3433_i2c_client->drv, &driver_attr_orientation)>=0)
	{            
		printk("device_create_file_ftsfwupgradeapp \n");        
	}
	*/
	sobj.self = obj;
	sobj.polling = 1;
	sobj.sensor_operate = gsensor_operate;

	if((err = hwmsen_attach(ID_ACCELEROMETER, &sobj)))
	{
		GSE_ERR("attach fail = %d\n", err);
		goto exit_hwmsen_attach_failed;
	}

    #ifdef _MC3433_SUPPORT_VPROXIMITY_SENSOR_
    // p-sensor
    {
        struct hwmsen_object obj_ps;
        
        if((err = misc_register(&mcube_psensor_device)))
        {
            PS_ERR("mcube_psensor_device register failed\n");
            goto exit_misc_device_psensor_register_failed;
        }
        
        obj_ps.polling = 1;
        obj_ps.sensor_operate = psensor_ps_operate;
        
        if((err = hwmsen_attach(ID_PROXIMITY, &obj_ps)))
        {
            PS_ERR("Proximity sensor attach fail = %d\n", err);
            goto exit_hwmsen_attach_psensor_failed;
        }
    }
    #endif

    #ifdef CONFIG_HAS_EARLYSUSPEND
        obj->early_drv.level    = EARLY_SUSPEND_LEVEL_DISABLE_FB - 1,
        obj->early_drv.suspend  = MC3433_early_suspend,
        obj->early_drv.resume   = MC3433_late_resume,    
        register_early_suspend(&obj->early_drv);
    #endif 

	GSE_LOG("%s: OK\n", __func__);    
	s_nInitFlag = MC3433_INIT_SUCC;
	return 0;

#ifdef _MC3433_SUPPORT_VPROXIMITY_SENSOR_
	exit_hwmsen_attach_psensor_failed:
	misc_deregister(&mcube_psensor_device);
	exit_misc_device_psensor_register_failed:
#endif    // END of _MC3433_SUPPORT_VPROXIMITY_SENSOR_

	exit_hwmsen_attach_failed:
/*
#ifdef MTK_AUTO_DETECT_ACCELEROMETER
        MC3433_delete_attr(&MC3433_init_info.platform_diver_addr->driver);
#else
        MC3433_delete_attr(&MC3433_gsensor_driver.driver);
#endif
*/
	exit_create_attr_failed:
	misc_deregister(&MC3433_device);

	exit_misc_device_register_failed:
	exit_init_failed:
	//i2c_detach_client(new_client);

exit_kfree:
	kfree(obj);

exit:
	GSE_ERR("%s: err = %d\n", __func__, err);        
	s_nInitFlag = MC3433_INIT_FAIL;

	return err;
}

/*****************************************
 *** MC3433_i2c_remove
 *****************************************/
static int MC3433_i2c_remove(struct i2c_client *client)
{
	int err = 0;	
/*	
    #ifdef MTK_AUTO_DETECT_ACCELEROMETER
        if((err = MC3433_delete_attr(&MC3433_init_info.platform_diver_addr->driver)));
    #else
        if((err = MC3433_delete_attr(&MC3433_gsensor_driver.driver)));
    #endif
        {
            GSE_ERR("MC3433_delete_attr fail: %d\n", err);
        }
*/

	if((err = misc_deregister(&MC3433_device)))
	{
		GSE_ERR("misc_deregister fail: %d\n", err);
	}

	if((err = hwmsen_detach(ID_ACCELEROMETER)))
	{
		GSE_ERR("hwmsen_detach fail: %d\n", err);
	}

    #ifdef _MC3433_SUPPORT_VPROXIMITY_SENSOR_
        misc_deregister(&mcube_psensor_device);
    #endif

	MC3433_i2c_client = NULL;
	i2c_unregister_device(client);
	kfree(i2c_get_clientdata(client));

	return 0;
}

/*****************************************
 *** MC3433_remove
 *****************************************/
static int MC3433_remove(struct platform_device *pdev)
{
    struct acc_hw *hw = get_cust_acc_hw();

    GSE_FUN();    
    MC3433_power(hw, 0);    
    i2c_del_driver(&MC3433_i2c_driver);

    return 0;
}

/*****************************************
 *** MC3433_probe
 *****************************************/
#ifndef MTK_AUTO_DETECT_ACCELEROMETER
static int MC3433_probe(struct platform_device *pdev) 
{
	struct acc_hw *hw = get_cust_acc_hw();
	GSE_FUN();

	MC3433_power(hw, 1);

	if(i2c_add_driver(&MC3433_i2c_driver))
	{
		GSE_ERR("add driver error\n");

		return -1;
	}

	return 0;
}


/*****************************************
 *** PLATFORM_DRIVER:: MC3433_gsensor_driver
 *****************************************/
static struct platform_driver MC3433_gsensor_driver = {
	.probe  = MC3433_probe,
	.remove = MC3433_remove,    
	.driver = {
	.name  = MC3433_DEV_NAME,
	.owner = THIS_MODULE,
               }
 };
#endif


#ifdef MTK_AUTO_DETECT_ACCELEROMETER

/*****************************************
 *** MC3433_local_init
 *****************************************/
static int  MC3433_local_init(void)
{
    struct acc_hw *hw = get_cust_acc_hw();
    
    GSE_FUN();    
    MC3433_power(hw, 1);    
    
    if(i2c_add_driver(&MC3433_i2c_driver))
    {
        GSE_ERR("add driver error\n");
        return -1;
    }

    if(MC3433_INIT_FAIL == s_nInitFlag)
    {
        return -1;
    }
    
    return 0;
}

static struct sensor_init_info  MC3433_init_info =
    {
        .name   = MC3433_DEV_NAME,
        .init   = MC3433_local_init,
        .uninit = MC3433_remove,
    };
#endif

/*****************************************
 *** MC3433_init
 *****************************************/
static int __init MC3433_init(void)
{
	struct acc_hw *hw = get_cust_acc_hw();
	i2c_register_board_info(hw->i2c_num, &MC3433_i2c_board_info, 1);
	
    #ifdef MTK_AUTO_DETECT_ACCELEROMETER
        hwmsen_gsensor_add(&MC3433_init_info);
    #else
        if(platform_driver_register(&MC3433_gsensor_driver))
        {
            GSE_ERR("failed to register driver");
            return -ENODEV;
        }
    #endif

	return 0;    
}

/*****************************************
 *** MC3433_exit
 *****************************************/
static void __exit MC3433_exit(void)
{
	GSE_FUN();

    #ifndef MTK_AUTO_DETECT_ACCELEROMETER
        platform_driver_unregister(&MC3433_gsensor_driver);
    #endif
}

/*----------------------------------------------------------------------------*/
module_init(MC3433_init);
module_exit(MC3433_exit);
/*----------------------------------------------------------------------------*/
MODULE_DESCRIPTION("MC3433 G-Sensor Driver");
MODULE_AUTHOR("TCL-BSP-ZXP");
MODULE_LICENSE("GPL");
MODULE_VERSION(MC3433_DEV_DRIVER_VERSION);

