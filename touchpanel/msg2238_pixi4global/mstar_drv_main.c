////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2006-2014 MStar Semiconductor, Inc.
// All rights reserved.
//
// Unless otherwise stipulated in writing, any and all information contained
// herein regardless in any format shall remain the sole proprietary of
// MStar Semiconductor Inc. and be kept in strict confidence
// (??MStar Confidential Information??) by the recipient.
// Any unauthorized act including without limitation unauthorized disclosure,
// copying, use, reproduction, sale, distribution, modification, disassembling,
// reverse engineering and compiling of the contents of MStar Confidential
// Information is unlawful and strictly prohibited. MStar hereby reserves the
// rights to any and all damages, losses, costs and expenses resulting therefrom.
//
////////////////////////////////////////////////////////////////////////////////

/**
 *
 * @file    mstar_drv_main.c
 *
 * @brief   This file defines the interface of touch screen
 *
 * @version v2.3.0.0
 *
 */

/*=============================================================*/
// INCLUDE FILE
/*=============================================================*/

#include "mstar_drv_main.h"
#include "mstar_drv_utility_adaption.h"
#include "mstar_drv_platform_porting_layer.h"
#include "mstar_drv_ic_fw_porting_layer.h"

#ifdef CTP_PSENSOR_SUPPORT
#include <linux/miscdevice.h>
#endif
//BEGIN:  fangjie add for ctp compatible design for MMITest rawdata test.
#include <linux/hwmsen_dev.h>
static struct device *compatible_msgtp;  
//extern struct device *hwmsen_get_compatible_dev();
//END: fangjie add for ctp compatible design for MMITest rawdata test.

/*=============================================================*/
// CONSTANT VALUE DEFINITION
/*=============================================================*/


/*=============================================================*/
// EXTERN VARIABLE DECLARATION
/*=============================================================*/

#ifdef CONFIG_ENABLE_FIRMWARE_DATA_LOG
extern FirmwareInfo_t g_FirmwareInfo;
extern u8 *g_LogModePacket;
extern u16 g_FirmwareMode;
#endif //CONFIG_ENABLE_FIRMWARE_DATA_LOG

#ifdef CONFIG_ENABLE_GESTURE_WAKEUP
extern u16 g_GestureWakeupMode;
#endif //CONFIG_ENABLE_GESTURE_WAKEUP

extern u8 g_ChipType;

/*=============================================================*/
// LOCAL VARIABLE DEFINITION
/*=============================================================*/

static u16 _gDebugReg[MAX_DEBUG_REGISTER_NUM] = {0};
static u32 _gDebugRegCount = 0;

static u8 *_gPlatformFwVersion = NULL; // internal use firmware version for MStar

#ifdef CONFIG_ENABLE_ITO_MP_TEST
static ItoTestMode_e _gItoTestMode = 0;
#endif //CONFIG_ENABLE_ITO_MP_TEST

static u32 _gIsUpdateComplete = 0;

static u8 *_gFwVersion = NULL; // customer firmware version

static struct class *_gFirmwareClass = NULL;
static struct device *_gFirmwareCmdDev = NULL;

/*=============================================================*/
// GLOBAL VARIABLE DEFINITION
/*=============================================================*/

#ifdef CONFIG_ENABLE_FIRMWARE_DATA_LOG
struct kset *g_TouchKSet = NULL;
struct kobject *g_TouchKObj = NULL;
#endif //CONFIG_ENABLE_FIRMWARE_DATA_LOG

u8 g_FwData[94][1024];
u32 g_FwDataCount = 0;

/*=============================================================*/
// LOCAL FUNCTION DEFINITION
/*=============================================================*/


/*=============================================================*/
// GLOBAL FUNCTION DEFINITION
/*=============================================================*/

ssize_t DrvMainFirmwareChipTypeShow(struct device *pDevice, struct device_attribute *pAttr, char *pBuf)
{
    DBG("*** %s() ***\n", __func__);

    return sprintf(pBuf, "%d", g_ChipType);
}

ssize_t DrvMainFirmwareChipTypeStore(struct device *pDevice, struct device_attribute *pAttr, const char *pBuf, size_t nSize)
{
    DBG("*** %s() ***\n", __func__);

//    g_ChipType = DrvIcFwLyrGetChipType();

    return nSize;
}

static DEVICE_ATTR(chip_type, SYSFS_AUTHORITY, DrvMainFirmwareChipTypeShow, DrvMainFirmwareChipTypeStore);

ssize_t DrvMainFirmwareDriverVersionShow(struct device *pDevice, struct device_attribute *pAttr, char *pBuf)
{
    DBG("*** %s() ***\n", __func__);

    return sprintf(pBuf, "%s", DEVICE_DRIVER_RELEASE_VERSION);
}

ssize_t DrvMainFirmwareDriverVersionStore(struct device *pDevice, struct device_attribute *pAttr, const char *pBuf, size_t nSize)
{
    DBG("*** %s() ***\n", __func__);

    return nSize;
}

static DEVICE_ATTR(driver_version, SYSFS_AUTHORITY, DrvMainFirmwareDriverVersionShow, DrvMainFirmwareDriverVersionStore);

/*--------------------------------------------------------------------------*/

ssize_t DrvMainFirmwareUpdateShow(struct device *pDevice, struct device_attribute *pAttr, char *pBuf)
{
    DBG("*** %s() _gFwVersion = %s ***\n", __func__, _gFwVersion);

    return sprintf(pBuf, "%s\n", _gFwVersion);
}

ssize_t DrvMainFirmwareUpdateStore(struct device *pDevice, struct device_attribute *pAttr, const char *pBuf, size_t nSize)
{
    DrvPlatformLyrDisableFingerTouchReport();

    DBG("*** %s() g_FwDataCount = %d ***\n", __func__, g_FwDataCount);

    if (0 != DrvIcFwLyrUpdateFirmware(g_FwData, EMEM_ALL))
    {
        _gIsUpdateComplete = 0;
        DBG("Update FAILED\n");
    }
    else
    {
        _gIsUpdateComplete = 1;
        DBG("Update SUCCESS\n");
    }

#ifdef CONFIG_ENABLE_FIRMWARE_DATA_LOG
    DrvIcFwLyrRestoreFirmwareModeToLogDataMode();    
#endif //CONFIG_ENABLE_FIRMWARE_DATA_LOG

    DrvPlatformLyrEnableFingerTouchReport();
  
    return nSize;
}

static DEVICE_ATTR(update, SYSFS_AUTHORITY, DrvMainFirmwareUpdateShow, DrvMainFirmwareUpdateStore);

ssize_t DrvMainFirmwareVersionShow(struct device *pDevice, struct device_attribute *pAttr, char *pBuf)
{
    DBG("*** %s() _gFwVersion = %s ***\n", __func__, _gFwVersion);

    return sprintf(pBuf, "%s\n", _gFwVersion);
}

ssize_t DrvMainFirmwareVersionStore(struct device *pDevice, struct device_attribute *pAttr, const char *pBuf, size_t nSize)
{
    u16 nMajor = 0, nMinor = 0;
    
    DrvIcFwLyrGetCustomerFirmwareVersion(&nMajor, &nMinor, &_gFwVersion);

    DBG("*** %s() _gFwVersion = %s ***\n", __func__, _gFwVersion);

    return nSize;
}

static DEVICE_ATTR(version, SYSFS_AUTHORITY, DrvMainFirmwareVersionShow, DrvMainFirmwareVersionStore);

ssize_t DrvMainFirmwareDataShow(struct device *pDevice, struct device_attribute *pAttr, char *pBuf)
{
    DBG("*** %s() g_FwDataCount = %d ***\n", __func__, g_FwDataCount);
    
    return g_FwDataCount;
}

ssize_t DrvMainFirmwareDataStore(struct device *pDevice, struct device_attribute *pAttr, const char *pBuf, size_t nSize)
{
    u32 nCount = nSize / 1024;
    u32 i;

    DBG("*** %s() ***\n", __func__);

    if (nCount > 0) // nSize >= 1024
   	{
        for (i = 0; i < nCount; i ++)
        {
            memcpy(g_FwData[g_FwDataCount], pBuf+(i*1024), 1024);

            g_FwDataCount ++;
        }
    }
    else // nSize < 1024
    {
    		if (nSize > 0)
    		{
            memcpy(g_FwData[g_FwDataCount], pBuf, nSize);

            g_FwDataCount ++;
    		}
    }

    DBG("*** g_FwDataCount = %d ***\n", g_FwDataCount);

    if (pBuf != NULL)
    {
        DBG("*** buf[0] = %c ***\n", pBuf[0]);
    }
   
    return nSize;
}

static DEVICE_ATTR(data, SYSFS_AUTHORITY, DrvMainFirmwareDataShow, DrvMainFirmwareDataStore);

/*--------------------------------------------------------------------------*/

#ifdef CONFIG_ENABLE_ITO_MP_TEST
ssize_t DrvMainFirmwareTestShow(struct device *pDevice, struct device_attribute *pAttr, char *pBuf)
{
    DBG("*** %s() ***\n", __func__);
    DBG("*** ctp mp test status = %d ***\n", DrvIcFwLyrGetMpTestResult());
    
    return sprintf(pBuf, "%d", DrvIcFwLyrGetMpTestResult());
}

ssize_t DrvMainFirmwareTestStore(struct device *pDevice, struct device_attribute *pAttr, const char *pBuf, size_t nSize)
{
    u32 nMode = 0;

    DBG("*** %s() ***\n", __func__);
    
    if (pBuf != NULL)
    {
        sscanf(pBuf, "%x", &nMode);   

        DBG("Mp Test Mode = 0x%x\n", nMode);

        if (nMode == ITO_TEST_MODE_OPEN_TEST) //open test
        {
            _gItoTestMode = ITO_TEST_MODE_OPEN_TEST;
            DrvIcFwLyrScheduleMpTestWork(ITO_TEST_MODE_OPEN_TEST);
        }
        else if (nMode == ITO_TEST_MODE_SHORT_TEST) //short test
        {
            _gItoTestMode = ITO_TEST_MODE_SHORT_TEST;
            DrvIcFwLyrScheduleMpTestWork(ITO_TEST_MODE_SHORT_TEST);
        }
        else
        {
            DBG("*** Undefined MP Test Mode ***\n");
        }
    }

    return nSize;
}

static DEVICE_ATTR(test, SYSFS_AUTHORITY, DrvMainFirmwareTestShow, DrvMainFirmwareTestStore);

ssize_t DrvMainFirmwareTestLogShow(struct device *pDevice, struct device_attribute *pAttr, char *pBuf)
{
    u32 nLength = 0;
    
    DBG("*** %s() ***\n", __func__);
    
    DrvIcFwLyrGetMpTestDataLog(_gItoTestMode, pBuf, &nLength);
    
    return nLength;
}

ssize_t DrvMainFirmwareTestLogStore(struct device *pDevice, struct device_attribute *pAttr, const char *pBuf, size_t nSize)
{
    DBG("*** %s() ***\n", __func__);

    return nSize;
}

static DEVICE_ATTR(test_log, SYSFS_AUTHORITY, DrvMainFirmwareTestLogShow, DrvMainFirmwareTestLogStore);

ssize_t DrvMainFirmwareTestFailChannelShow(struct device *pDevice, struct device_attribute *pAttr, char *pBuf)
{
    u32 nCount = 0;

    DBG("*** %s() ***\n", __func__);
    
    DrvIcFwLyrGetMpTestFailChannel(_gItoTestMode, pBuf, &nCount);
    
    return nCount;
}

ssize_t DrvMainFirmwareTestFailChannelStore(struct device *pDevice, struct device_attribute *pAttr, const char *pBuf, size_t nSize)
{
    DBG("*** %s() ***\n", __func__);

    return nSize;
}

static DEVICE_ATTR(test_fail_channel, SYSFS_AUTHORITY, DrvMainFirmwareTestFailChannelShow, DrvMainFirmwareTestFailChannelStore);
#endif //CONFIG_ENABLE_ITO_MP_TEST

/*--------------------------------------------------------------------------*/

#ifdef CONFIG_ENABLE_GESTURE_WAKEUP

ssize_t DrvMainFirmwareGestureWakeupModeShow(struct device *pDevice, struct device_attribute *pAttr, char *pBuf)
{
    DBG("*** %s() ***\n", __func__);
    DBG("g_GestureWakeupMode = 0x%x\n", g_GestureWakeupMode);

    return sprintf(pBuf, "%x", g_GestureWakeupMode);
}

ssize_t DrvMainFirmwareGestureWakeupModeStore(struct device *pDevice, struct device_attribute *pAttr, const char *pBuf, size_t nSize)
{
    u32 nLength, nWakeupMode;
    
    DBG("*** %s() ***\n", __func__);

    if (pBuf != NULL)
    {
        sscanf(pBuf, "%x", &nWakeupMode);   
        DBG("nWakeupMode = 0x%x\n", nWakeupMode);

        nLength = nSize;
        DBG("nLength = %d\n", nLength);

        if ((nWakeupMode & GESTURE_WAKEUP_MODE_DOUBLE_CLICK_FLAG) == GESTURE_WAKEUP_MODE_DOUBLE_CLICK_FLAG)
        {
            g_GestureWakeupMode = g_GestureWakeupMode | GESTURE_WAKEUP_MODE_DOUBLE_CLICK_FLAG;
        }
        else
        {
            g_GestureWakeupMode = g_GestureWakeupMode & (~GESTURE_WAKEUP_MODE_DOUBLE_CLICK_FLAG);
        }
        
        if ((nWakeupMode & GESTURE_WAKEUP_MODE_UP_DIRECT_FLAG) == GESTURE_WAKEUP_MODE_UP_DIRECT_FLAG)
        {
            g_GestureWakeupMode = g_GestureWakeupMode | GESTURE_WAKEUP_MODE_UP_DIRECT_FLAG;
        }
        else
        {
            g_GestureWakeupMode = g_GestureWakeupMode & (~GESTURE_WAKEUP_MODE_UP_DIRECT_FLAG);
        }

        if ((nWakeupMode & GESTURE_WAKEUP_MODE_DOWN_DIRECT_FLAG) == GESTURE_WAKEUP_MODE_DOWN_DIRECT_FLAG)
        {
            g_GestureWakeupMode = g_GestureWakeupMode | GESTURE_WAKEUP_MODE_DOWN_DIRECT_FLAG;
        }
        else
        {
            g_GestureWakeupMode = g_GestureWakeupMode & (~GESTURE_WAKEUP_MODE_DOWN_DIRECT_FLAG);
        }

        if ((nWakeupMode & GESTURE_WAKEUP_MODE_LEFT_DIRECT_FLAG) == GESTURE_WAKEUP_MODE_LEFT_DIRECT_FLAG)
        {
            g_GestureWakeupMode = g_GestureWakeupMode | GESTURE_WAKEUP_MODE_LEFT_DIRECT_FLAG;
        }
        else
        {
            g_GestureWakeupMode = g_GestureWakeupMode & (~GESTURE_WAKEUP_MODE_LEFT_DIRECT_FLAG);
        }

        if ((nWakeupMode & GESTURE_WAKEUP_MODE_RIGHT_DIRECT_FLAG) == GESTURE_WAKEUP_MODE_RIGHT_DIRECT_FLAG)
        {
            g_GestureWakeupMode = g_GestureWakeupMode | GESTURE_WAKEUP_MODE_RIGHT_DIRECT_FLAG;
        }
        else
        {
            g_GestureWakeupMode = g_GestureWakeupMode & (~GESTURE_WAKEUP_MODE_RIGHT_DIRECT_FLAG);
        }

        if ((nWakeupMode & GESTURE_WAKEUP_MODE_m_CHARACTER_FLAG) == GESTURE_WAKEUP_MODE_m_CHARACTER_FLAG)
        {
            g_GestureWakeupMode = g_GestureWakeupMode | GESTURE_WAKEUP_MODE_m_CHARACTER_FLAG;
        }
        else
        {
            g_GestureWakeupMode = g_GestureWakeupMode & (~GESTURE_WAKEUP_MODE_m_CHARACTER_FLAG);
        }

        if ((nWakeupMode & GESTURE_WAKEUP_MODE_W_CHARACTER_FLAG) == GESTURE_WAKEUP_MODE_W_CHARACTER_FLAG)
        {
            g_GestureWakeupMode = g_GestureWakeupMode | GESTURE_WAKEUP_MODE_W_CHARACTER_FLAG;
        }
        else
        {
            g_GestureWakeupMode = g_GestureWakeupMode & (~GESTURE_WAKEUP_MODE_W_CHARACTER_FLAG);
        }

        if ((nWakeupMode & GESTURE_WAKEUP_MODE_C_CHARACTER_FLAG) == GESTURE_WAKEUP_MODE_C_CHARACTER_FLAG)
        {
            g_GestureWakeupMode = g_GestureWakeupMode | GESTURE_WAKEUP_MODE_C_CHARACTER_FLAG;
        }
        else
        {
            g_GestureWakeupMode = g_GestureWakeupMode & (~GESTURE_WAKEUP_MODE_C_CHARACTER_FLAG);
        }
       
        if ((nWakeupMode & GESTURE_WAKEUP_MODE_e_CHARACTER_FLAG) == GESTURE_WAKEUP_MODE_e_CHARACTER_FLAG)
        {
            g_GestureWakeupMode = g_GestureWakeupMode | GESTURE_WAKEUP_MODE_e_CHARACTER_FLAG;
        }
        else
        {
            g_GestureWakeupMode = g_GestureWakeupMode & (~GESTURE_WAKEUP_MODE_e_CHARACTER_FLAG);
        }

        if ((nWakeupMode & GESTURE_WAKEUP_MODE_V_CHARACTER_FLAG) == GESTURE_WAKEUP_MODE_V_CHARACTER_FLAG)
        {
            g_GestureWakeupMode = g_GestureWakeupMode | GESTURE_WAKEUP_MODE_V_CHARACTER_FLAG;
        }
        else
        {
            g_GestureWakeupMode = g_GestureWakeupMode & (~GESTURE_WAKEUP_MODE_V_CHARACTER_FLAG);
        }

        if ((nWakeupMode & GESTURE_WAKEUP_MODE_O_CHARACTER_FLAG) == GESTURE_WAKEUP_MODE_O_CHARACTER_FLAG)
        {
            g_GestureWakeupMode = g_GestureWakeupMode | GESTURE_WAKEUP_MODE_O_CHARACTER_FLAG;
        }
        else
        {
            g_GestureWakeupMode = g_GestureWakeupMode & (~GESTURE_WAKEUP_MODE_O_CHARACTER_FLAG);
        }

        if ((nWakeupMode & GESTURE_WAKEUP_MODE_S_CHARACTER_FLAG) == GESTURE_WAKEUP_MODE_S_CHARACTER_FLAG)
        {
            g_GestureWakeupMode = g_GestureWakeupMode | GESTURE_WAKEUP_MODE_S_CHARACTER_FLAG;
        }
        else
        {
            g_GestureWakeupMode = g_GestureWakeupMode & (~GESTURE_WAKEUP_MODE_S_CHARACTER_FLAG);
        }

        if ((nWakeupMode & GESTURE_WAKEUP_MODE_Z_CHARACTER_FLAG) == GESTURE_WAKEUP_MODE_Z_CHARACTER_FLAG)
        {
            g_GestureWakeupMode = g_GestureWakeupMode | GESTURE_WAKEUP_MODE_Z_CHARACTER_FLAG;
        }
        else
        {
            g_GestureWakeupMode = g_GestureWakeupMode & (~GESTURE_WAKEUP_MODE_Z_CHARACTER_FLAG);
        }

        DBG("g_GestureWakeupMode = 0x%x\n", g_GestureWakeupMode);
    }
        
    return nSize;
}

static DEVICE_ATTR(gesture_wakeup_mode, SYSFS_AUTHORITY, DrvMainFirmwareGestureWakeupModeShow, DrvMainFirmwareGestureWakeupModeStore);

#endif //CONFIG_ENABLE_GESTURE_WAKEUP

/*--------------------------------------------------------------------------*/

ssize_t DrvMainFirmwareDebugShow(struct device *pDevice, struct device_attribute *pAttr, char *pBuf)
{
    u32 i;
    u8 nBank, nAddr;
    u16 szRegData[MAX_DEBUG_REGISTER_NUM] = {0};
    u8 szOut[MAX_DEBUG_REGISTER_NUM*25] = {0}, szValue[10] = {0};

    DBG("*** %s() ***\n", __func__);
    
    DbBusEnterSerialDebugMode();
    DbBusStopMCU();
    DbBusIICUseBus();
    DbBusIICReshape();
    mdelay(300);
    
    for (i = 0; i < _gDebugRegCount; i ++)
    {
        szRegData[i] = RegGet16BitValue(_gDebugReg[i]);
    }

    DbBusIICNotUseBus();
    DbBusNotStopMCU();
    DbBusExitSerialDebugMode();

    for (i = 0; i < _gDebugRegCount; i ++)
    {
    	  nBank = (_gDebugReg[i] >> 8) & 0xFF;
    	  nAddr = _gDebugReg[i] & 0xFF;
    	  
        DBG("reg(0x%X,0x%X)=0x%04X\n", nBank, nAddr, szRegData[i]);

        strcat(szOut, "reg(");
        sprintf(szValue, "0x%X", nBank);
        strcat(szOut, szValue);
        strcat(szOut, ",");
        sprintf(szValue, "0x%X", nAddr);
        strcat(szOut, szValue);
        strcat(szOut, ")=");
        sprintf(szValue, "0x%04X", szRegData[i]);
        strcat(szOut, szValue);
        strcat(szOut, "\n");
    }

    return sprintf(pBuf, "%s\n", szOut);
}

ssize_t DrvMainFirmwareDebugStore(struct device *pDevice, struct device_attribute *pAttr, const char *pBuf, size_t nSize)
{
    u32 i;
    char *pCh;

    DBG("*** %s() ***\n", __func__);

    if (pBuf != NULL)
    {
        DBG("*** %s() pBuf[0] = %c ***\n", __func__, pBuf[0]);
        DBG("*** %s() pBuf[1] = %c ***\n", __func__, pBuf[1]);
        DBG("*** %s() pBuf[2] = %c ***\n", __func__, pBuf[2]);
        DBG("*** %s() pBuf[3] = %c ***\n", __func__, pBuf[3]);
        DBG("*** %s() pBuf[4] = %c ***\n", __func__, pBuf[4]);
        DBG("*** %s() pBuf[5] = %c ***\n", __func__, pBuf[5]);

        DBG("nSize = %d\n", nSize);
       
        i = 0;
        while ((pCh = strsep((char **)&pBuf, " ,")) && (i < MAX_DEBUG_REGISTER_NUM))
        {
            DBG("pCh = %s\n", pCh);
            
            _gDebugReg[i] = DrvCommonConvertCharToHexDigit(pCh, strlen(pCh));

            DBG("_gDebugReg[%d] = 0x%04X\n", i, _gDebugReg[i]);
            i ++;
        }
        _gDebugRegCount = i;
        
        DBG("_gDebugRegCount = %d\n", _gDebugRegCount);
    }

    return nSize;
}

static DEVICE_ATTR(debug, SYSFS_AUTHORITY, DrvMainFirmwareDebugShow, DrvMainFirmwareDebugStore);

/*--------------------------------------------------------------------------*/

ssize_t DrvMainFirmwarePlatformVersionShow(struct device *pDevice, struct device_attribute *pAttr, char *pBuf)
{
    DBG("*** %s() _gPlatformFwVersion = %s ***\n", __func__, _gPlatformFwVersion);

    return sprintf(pBuf, "%s\n", _gPlatformFwVersion);
}

ssize_t DrvMainFirmwarePlatformVersionStore(struct device *pDevice, struct device_attribute *pAttr, const char *pBuf, size_t nSize)
{
    DrvIcFwLyrGetPlatformFirmwareVersion(&_gPlatformFwVersion);

#ifdef CONFIG_ENABLE_FIRMWARE_DATA_LOG
    DrvIcFwLyrRestoreFirmwareModeToLogDataMode();    
#endif //CONFIG_ENABLE_FIRMWARE_DATA_LOG

    DBG("*** %s() _gPlatformFwVersion = %s ***\n", __func__, _gPlatformFwVersion);

    return nSize;
}

static DEVICE_ATTR(platform_version, SYSFS_AUTHORITY, DrvMainFirmwarePlatformVersionShow, DrvMainFirmwarePlatformVersionStore);

#if 1 //#ifdef SYS_COMPATIBLE
extern Msg22xxSwId_e gSwId_ctpid;
static ssize_t msg2238_ctpid_show(struct device *dev,
                                     struct device_attribute *attr, char *buf)
{
	unsigned char *strid=NULL;
	#if 0
	if (gSwId_ctpid == MSG22XX_SW_ID_EACH)
	{	
		strid="M2_EACH";
	}
	else if (gSwId_ctpid == MSG22XX_SW_ID_GREEN)
	{	
		strid="M2_GREEN";
	}
	else
	{
		strid = "M2_NONE";
	}
	#endif
	strid = "MSG2238";
	return sprintf(buf, "%s\n", strid);
}
static DEVICE_ATTR(ctpid, 0444, msg2238_ctpid_show, NULL);
static void MSG2238_read_ctpid_for_compatible(void)
{
		compatible_msgtp=hwmsen_get_compatible_dev();
		if (device_create_file(compatible_msgtp, &dev_attr_ctpid) < 0)
		{
			pr_err("Failed to create device file(%s)!\n", dev_attr_ctpid.attr.name);
		}
		dev_set_drvdata(compatible_msgtp, NULL);
}
#endif
//END: fangjie add for ctp compatible design for MMITest rawdata test. 2014062


/*--------------------------------------------------------------------------*/

#ifdef CONFIG_ENABLE_FIRMWARE_DATA_LOG
ssize_t DrvMainFirmwareModeShow(struct device *pDevice, struct device_attribute *pAttr, char *pBuf)
{
#if defined(CONFIG_ENABLE_CHIP_MSG26XXM)
    g_FirmwareMode = DrvIcFwLyrGetFirmwareMode();
    
    DBG("%s() firmware mode = 0x%x\n", __func__, g_FirmwareMode);

    return sprintf(pBuf, "%x", g_FirmwareMode);
#elif defined(CONFIG_ENABLE_CHIP_MSG21XXA) || defined(CONFIG_ENABLE_CHIP_MSG22XX)
    DrvIcFwLyrGetFirmwareInfo(&g_FirmwareInfo);
    g_FirmwareMode = g_FirmwareInfo.nFirmwareMode;

    DBG("%s() firmware mode = 0x%x, can change firmware mode = %d\n", __func__, g_FirmwareInfo.nFirmwareMode, g_FirmwareInfo.nIsCanChangeFirmwareMode);

    return sprintf(pBuf, "%x,%d", g_FirmwareInfo.nFirmwareMode, g_FirmwareInfo.nIsCanChangeFirmwareMode);
#endif
}

ssize_t DrvMainFirmwareModeStore(struct device *pDevice, struct device_attribute *pAttr, const char *pBuf, size_t nSize)
{
    DBG("*** %s() ***\n", __func__);

    u32 nMode;
    
    if (pBuf != NULL)
    {
        sscanf(pBuf, "%x", &nMode);   
        DBG("firmware mode = 0x%x\n", nMode);

        if (nMode == FIRMWARE_MODE_DEMO_MODE) //demo mode
        {
            g_FirmwareMode = DrvIcFwLyrChangeFirmwareMode(FIRMWARE_MODE_DEMO_MODE);
        }
        else if (nMode == FIRMWARE_MODE_DEBUG_MODE) //debug mode
        {
            g_FirmwareMode = DrvIcFwLyrChangeFirmwareMode(FIRMWARE_MODE_DEBUG_MODE);
        }
#if defined(CONFIG_ENABLE_CHIP_MSG21XXA) || defined(CONFIG_ENABLE_CHIP_MSG22XX)
        else if (nMode == FIRMWARE_MODE_RAW_DATA_MODE) //raw data mode
        {
            g_FirmwareMode = DrvIcFwLyrChangeFirmwareMode(FIRMWARE_MODE_RAW_DATA_MODE);
        }
#endif //CONFIG_ENABLE_CHIP_MSG21XXA || CONFIG_ENABLE_CHIP_MSG22XX
        else
        {
            DBG("*** Undefined Firmware Mode ***\n");
        }
    }

    DBG("*** g_FirmwareMode = 0x%x ***\n", g_FirmwareMode);

    return nSize;
}

static DEVICE_ATTR(mode, SYSFS_AUTHORITY, DrvMainFirmwareModeShow, DrvMainFirmwareModeStore);
/*
ssize_t DrvMainFirmwarePacketShow(struct device *pDevice, struct device_attribute *pAttr, char *pBuf)
{
    u32 i = 0;
    u32 nLength = 0;
    
    DBG("*** %s() ***\n", __func__);
    
    if (g_LogModePacket != NULL)
    {
        DBG("g_FirmwareMode=%x, g_LogModePacket[0]=%x, g_LogModePacket[1]=%x\n", g_FirmwareMode, g_LogModePacket[0], g_LogModePacket[1]);
        DBG("g_LogModePacket[2]=%x, g_LogModePacket[3]=%x\n", g_LogModePacket[2], g_LogModePacket[3]);
        DBG("g_LogModePacket[4]=%x, g_LogModePacket[5]=%x\n", g_LogModePacket[4], g_LogModePacket[5]);

#if defined(CONFIG_ENABLE_CHIP_MSG26XXM)
        if ((g_FirmwareMode == FIRMWARE_MODE_DEBUG_MODE) && (g_LogModePacket[0] == 0xA5 || g_LogModePacket[0] == 0xAB || g_LogModePacket[0] == 0xA7))
#elif defined(CONFIG_ENABLE_CHIP_MSG21XXA) || defined(CONFIG_ENABLE_CHIP_MSG22XX)
        if ((g_FirmwareMode == FIRMWARE_MODE_DEBUG_MODE || g_FirmwareMode == FIRMWARE_MODE_RAW_DATA_MODE) && (g_LogModePacket[0] == 0x62))
#endif
        {
            for (i = 0; i < g_FirmwareInfo.nLogModePacketLength; i ++)
            {
                pBuf[i] = g_LogModePacket[i];
            }

            nLength = g_FirmwareInfo.nLogModePacketLength;
            DBG("nLength = %d\n", nLength);
        }
        else
        {
            DBG("CURRENT MODE IS NOT DEBUG MODE/WRONG DEBUG MODE HEADER\n");
//        nLength = 0;
        }
    }
    else
    {
        DBG("g_LogModePacket is NULL\n");
//        nLength = 0;
    }

    return nLength;
}

ssize_t DrvMainFirmwarePacketStore(struct device *pDevice, struct device_attribute *pAttr, const char *pBuf, size_t nSize)
{
    DBG("*** %s() ***\n", __func__);

    return nSize;
}

static DEVICE_ATTR(packet, SYSFS_AUTHORITY, DrvMainFirmwarePacketShow, DrvMainFirmwarePacketStore);
*/
ssize_t DrvMainFirmwareSensorShow(struct device *pDevice, struct device_attribute *pAttr, char *pBuf)
{
    DBG("*** %s() ***\n", __func__);

#if defined(CONFIG_ENABLE_CHIP_MSG26XXM)
    if (g_FirmwareInfo.nLogModePacketHeader == 0xA5 || g_FirmwareInfo.nLogModePacketHeader == 0xAB)
    {
        return sprintf(pBuf, "%d,%d", g_FirmwareInfo.nMx, g_FirmwareInfo.nMy);
    }
    else if (g_FirmwareInfo.nLogModePacketHeader == 0xA7)
    {
        return sprintf(pBuf, "%d,%d,%d,%d", g_FirmwareInfo.nMx, g_FirmwareInfo.nMy, g_FirmwareInfo.nSs, g_FirmwareInfo.nSd);
    }
    else
    {
        DBG("Undefined debug mode packet format : 0x%x\n", g_FirmwareInfo.nLogModePacketHeader);
        return 0;
    }
#elif defined(CONFIG_ENABLE_CHIP_MSG21XXA) || defined(CONFIG_ENABLE_CHIP_MSG22XX)
    return sprintf(pBuf, "%d", g_FirmwareInfo.nLogModePacketLength);
#endif
}

ssize_t DrvMainFirmwareSensorStore(struct device *pDevice, struct device_attribute *pAttr, const char *pBuf, size_t nSize)
{
    DBG("*** %s() ***\n", __func__);
/*
    DrvIcFwLyrGetFirmwareInfo(&g_FirmwareInfo);
#if defined(CONFIG_ENABLE_CHIP_MSG21XXA) || defined(CONFIG_ENABLE_CHIP_MSG22XX)
    g_FirmwareMode = g_FirmwareInfo.nFirmwareMode;
#endif //CONFIG_ENABLE_CHIP_MSG21XXA || CONFIG_ENABLE_CHIP_MSG22XX
*/    
    return nSize;
}

static DEVICE_ATTR(sensor, SYSFS_AUTHORITY, DrvMainFirmwareSensorShow, DrvMainFirmwareSensorStore);

ssize_t DrvMainFirmwareHeaderShow(struct device *pDevice, struct device_attribute *pAttr, char *pBuf)
{
    DBG("*** %s() ***\n", __func__);

    return sprintf(pBuf, "%d", g_FirmwareInfo.nLogModePacketHeader);
}

ssize_t DrvMainFirmwareHeaderStore(struct device *pDevice, struct device_attribute *pAttr, const char *pBuf, size_t nSize)
{
    DBG("*** %s() ***\n", __func__);
/*
    DrvIcFwLyrGetFirmwareInfo(&g_FirmwareInfo);
#if defined(CONFIG_ENABLE_CHIP_MSG21XXA) || defined(CONFIG_ENABLE_CHIP_MSG22XX)
    g_FirmwareMode = g_FirmwareInfo.nFirmwareMode;
#endif //CONFIG_ENABLE_CHIP_MSG21XXA || CONFIG_ENABLE_CHIP_MSG22XX
*/    
    return nSize;
}

static DEVICE_ATTR(header, SYSFS_AUTHORITY, DrvMainFirmwareHeaderShow, DrvMainFirmwareHeaderStore);

//------------------------------------------------------------------------------//

ssize_t DrvMainKObjectPacketShow(struct kobject *pKObj, struct kobj_attribute *pAttr, char *pBuf)
{
    u32 i = 0;
    u32 nLength = 0;

    DBG("*** %s() ***\n", __func__);

    if (strcmp(pAttr->attr.name, "packet") == 0)
    {
        if (g_LogModePacket != NULL)
        {
            DBG("g_FirmwareMode=%x, g_LogModePacket[0]=%x, g_LogModePacket[1]=%x\n", g_FirmwareMode, g_LogModePacket[0], g_LogModePacket[1]);
            DBG("g_LogModePacket[2]=%x, g_LogModePacket[3]=%x\n", g_LogModePacket[2], g_LogModePacket[3]);
            DBG("g_LogModePacket[4]=%x, g_LogModePacket[5]=%x\n", g_LogModePacket[4], g_LogModePacket[5]);

#if defined(CONFIG_ENABLE_CHIP_MSG26XXM)
            if ((g_FirmwareMode == FIRMWARE_MODE_DEBUG_MODE) && (g_LogModePacket[0] == 0xA5 || g_LogModePacket[0] == 0xAB || g_LogModePacket[0] == 0xA7))
#elif defined(CONFIG_ENABLE_CHIP_MSG21XXA) || defined(CONFIG_ENABLE_CHIP_MSG22XX)
            if ((g_FirmwareMode == FIRMWARE_MODE_DEBUG_MODE || g_FirmwareMode == FIRMWARE_MODE_RAW_DATA_MODE) && (g_LogModePacket[0] == 0x62))
#endif
            {
                for (i = 0; i < g_FirmwareInfo.nLogModePacketLength; i ++)
                {
                    pBuf[i] = g_LogModePacket[i];
                }

                nLength = g_FirmwareInfo.nLogModePacketLength;
                DBG("nLength = %d\n", nLength);
            }
            else
            {
                DBG("CURRENT MODE IS NOT DEBUG MODE/WRONG DEBUG MODE HEADER\n");
//            nLength = 0;
            }
        }
        else
        {
            DBG("g_LogModePacket is NULL\n");
//            nLength = 0;
        }
    }
    else
    {
        DBG("pAttr->attr.name = %s \n", pAttr->attr.name);
//        nLength = 0;
    }

    return nLength;
}

ssize_t DrvMainKObjectPacketStore(struct kobject *pKObj, struct kobj_attribute *pAttr, const char *pBuf, size_t nCount)
{
    DBG("*** %s() ***\n", __func__);
/*
    if (strcmp(pAttr->attr.name, "packet") == 0)
    {

    }
*/    	
    return nCount;
}

static struct kobj_attribute packet_attr = __ATTR(packet, 0666, DrvMainKObjectPacketShow, DrvMainKObjectPacketStore);

/* Create a group of attributes so that we can create and destroy them all at once. */
static struct attribute *attrs[] = {
    &packet_attr.attr,
    NULL,	/* need to NULL terminate the list of attributes */
};

/*
 * An unnamed attribute group will put all of the attributes directly in
 * the kobject directory. If we specify a name, a subdirectory will be
 * created for the attributes with the directory being the name of the
 * attribute group.
 */
struct attribute_group attr_group = {
    .attrs = attrs,
};
#endif //CONFIG_ENABLE_FIRMWARE_DATA_LOG


#ifdef CTP_PSENSOR_SUPPORT
extern struct i2c_client *g_I2cClient;
#define MSG2238ALSPS_DEVICE_NAME               		"msg2238-alsps-tpd-dev"
int g2238ps_state = -1;
int mstar2238_pls_opened=0;
int msg2238_tp_resume_flag=0;//add by PR653624

int  msg2238_ps_mode_enable(bool enable)
{
	int err = 0;
	unsigned char dbbus_tx_data[4];
	unsigned char dbbus_rx_data[4];
	printk("fj: ctp ps msg2238_ps_mode_enable\n");
	msg2238_tp_resume_flag=0;
	if(enable)
	{
	      printk("fj: Before the PS enable wake up the CTP\n");
	      dbbus_tx_data[0] = 0x52;
	      dbbus_tx_data[1] = 0x00;
	      dbbus_tx_data[2] = 0x4A;
	      dbbus_tx_data[3] = 0xA0;
	      //err = i2c_master_send(g_I2cClient, &dbbus_tx_data[0], 4);
	      err =  IicWriteData(SLAVE_I2C_ID_DWI2C, &dbbus_tx_data[0], 4);
	      if(err>=0){
	      		printk("fj: i2c_client-addr=%d\n",g_I2cClient->addr);
	     		printk("fj: pls wuyongtao_ctp ctp ps open OK \n");
			return 0;
	      }
	     else{
		 	printk("fj:  pls wuyongtao_ctp ctp ps open failed err = %d \n",err);
			return -1;
		  }
	}
	else
	{     
	      dbbus_tx_data[0] = 0x52;
	      dbbus_tx_data[1] = 0x00;
	      dbbus_tx_data[2] = 0x4A;
	      dbbus_tx_data[3] = 0xA1;
	      //err = i2c_master_send(g_I2cClient, &dbbus_tx_data[0], 4);
		err =  IicWriteData(SLAVE_I2C_ID_DWI2C, &dbbus_tx_data[0], 4);
	      if(err>=0){
			printk ("fj: pls  wuyongtao_ctp ctp ps close OK \n");
	      }
	      else{
			printk("fj: pls  wuyongtao_ctp ctp ps close failed err = %d \n",err);
	     }
	}
	return 0;
} 

 int mstar2238_pls_enable(void)
 {
 	int ret=0;
	 TPD_DMESG("fj: mstar2238_pls_enable\n");
	 if(mstar2238_pls_opened>0){
		printk("fj: msg2238_pls have enabled\n");
		return 0;
	 }
	 ret=msg2238_ps_mode_enable(true);
	 if(ret>=0){
		mstar2238_pls_opened = 1 ; 
	 }
	 return 0;
 }
 
 int mstar2238_pls_disable(void)
 {
	 //TPD_DMESG("fj: mstar2238_pls_disable");
	 printk("fj: mstar2238_pls_disable");
	  if(mstar2238_pls_opened<1){
		printk("fj: msg2238_pls have disabled\n");
		return 0;
	 }
	msg2238_ps_mode_enable(false);
	 mstar2238_pls_opened = 0;
	// del_timer_sync(&ps_timer);	 
	 return 0;
 }
 static int mstar2238_pls_open(struct inode *inode, struct file *file)
 {
	 TPD_DMESG("%s\n", __func__);
	 if (mstar2238_pls_opened)
		 return -EBUSY;
	 mstar2238_pls_opened = 1 ;
	 return 0;
 }
 static int mstar2238_pls_release(struct inode *inode, struct file *file)
 {
	 TPD_DMESG("%s", __func__);
	 mstar2238_pls_opened = 0;
	 return 0;//ap3212c_pls_disable(AP3212C_PLS_BOTH);
 }
static int mstar2238_pls_unlocked_ioctl(struct file *file, unsigned int cmd,unsigned long arg)
 {
	 void __user *argp = (void __user *)arg;
	 TPD_DMESG("%s: cmd %d", __func__, _IOC_NR(cmd));
	 switch (cmd) {
	 case 1:
		 printk("%s: mstar2238_pls_enable*************************\n", __func__);
		 mstar2238_pls_enable();
		 break;
	 case 2:
		 printk("%s: mstar2238_pls_disable*************************\n", __func__);
		 mstar2238_pls_disable();
		 break;
	 default:
		 printk("%s: invalid cmd %d\n", __func__, _IOC_NR(cmd));
		 return -EINVAL;
	 }
	 return 0;
 }

 static struct file_operations mstar2238_pls_fops = {
	 .owner 		 = THIS_MODULE,
	 .open			 = mstar2238_pls_open,
	 .release		 = mstar2238_pls_release,
	 .unlocked_ioctl	 = mstar2238_pls_unlocked_ioctl,
 };
 static struct miscdevice mstar2238_pls_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = MSG2238ALSPS_DEVICE_NAME,
	.fops = &mstar2238_pls_fops,
};

int  get_msg2238_data(void )
{
	return  g2238ps_state;
}

#if 0
static char *flashs;
static char flashs_flag=0;
static ssize_t firmware_memory_show(struct device *dev,struct device_attribute *attr, char *buf)
{
	unsigned char dbbus_tx_data[3];
        unsigned char dbbus_rx_data[6] ;
	unsigned char chnum=0;

	if(flashs_flag)
	{
		printk("[MSG2138A]============new flash print out!char is :%s \n",flashs);
		flashs_flag=0;
		chnum=sprintf(buf,"flashs=: %s \n",flashs);
		printk("[MSG2138A]=======buf==:%s \n",buf);
		kfree(flashs);
		return chnum;
	}
	dbbus_tx_data[0] = 0x53;
    	dbbus_tx_data[1] = 0x00;
    	dbbus_tx_data[2] = 0x4A;
	HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX_TP, &dbbus_tx_data[0], 3);
    	HalTscrCReadI2CSeq(FW_ADDR_MSG21XX_TP, &dbbus_rx_data[0], 2);
	msleep(20);
	dbbus_tx_data[0] = 0x53;
    	dbbus_tx_data[1] = 0x0f;
    	dbbus_tx_data[2] = 0x18;
	HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX_TP, &dbbus_tx_data[0], 3);
    	HalTscrCReadI2CSeq(FW_ADDR_MSG21XX_TP, &dbbus_rx_data[2], 4);
	return sprintf(buf, "Ps_open:0x%x 0x%x \nFar_near:0x%x 0x%x 0x%x 0x%x \nmstar2238_pls_opened=%d \n",dbbus_rx_data[0],dbbus_rx_data[1],
			dbbus_rx_data[2],dbbus_rx_data[3],dbbus_rx_data[4],dbbus_rx_data[5],mstar2238_pls_opened);
}

static ssize_t firmware_memory_store(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
	unsigned char address1,address2;
	unsigned char  rw_flag=0;//0 Read,1 Write;
	unsigned short regaddr;
	unsigned char datelenth;
	unsigned char dbbus_tx[3];
	unsigned char dbbus_rx[256];
	unsigned char index=0;

	flashs=kmalloc(256*sizeof(char), GFP_KERNEL);
	memset(flashs, 0, sizeof(flashs));

	printk("[MSG2138A] *************************************buf =   %s \n",buf);
	//sprintf(flashs,"%s",buf);
	if (sscanf(buf, "%02x:%02x:%02x:%02x",&address1,&address2,&datelenth,&rw_flag))
	{
		regaddr=(address1<<8) | address2;
		if(rw_flag==1){ //write reg;
			printk("[MSG2138A] ----------regaddr=0x%x,----------write data=:0x%x ------flages=: %d \n",regaddr,datelenth,rw_flag);
			drvDB_WriteReg8Bit(address1,address2,datelenth);// write data;
			datelenth=1; //read the writed data;
		}else{
			printk("[MSG2138A] ----------regaddr=0x%x,-----------read datelenth=:%d\n",regaddr,datelenth);
		}
		if(datelenth>256)
		{
			printk("[MSG2138A] Your requie data lenthe is too long!\n ");
			return 0;
		}
		if(datelenth<1){
			datelenth=1;
		}
		if(regaddr>0xffff){
			printk("[MSG2138A] Your requie data's address is wrong!\n ");
			return 0;
		}
		if(regaddr>0xff){
			dbbus_tx[0] = 0x53;
			dbbus_tx[1] = regaddr>>8;
			dbbus_tx[2] = regaddr<<8;
		}
		else
		{
			dbbus_tx[0] = 0x53;
		    	dbbus_tx[1] = 0x00;
		    	dbbus_tx[2] = regaddr;
		}
		HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX_TP, &dbbus_tx[0], 3);
	    	HalTscrCReadI2CSeq(FW_ADDR_MSG21XX_TP, &dbbus_rx[0], datelenth);
		//dbbus_rx[0]=datelenth;
		flashs_flag=1;//flash have new data;
		printk("[MSG2138A] The read data is:<--------->flashs=: ");
		for(index=0;index<datelenth;index++){
			sprintf(flashs,"%s 0x%02x", flashs, dbbus_rx[index]);
		}
		printk("%s \n",flashs);
	}
	return size;
}

static DEVICE_ATTR(memory, 0666, firmware_memory_show,firmware_memory_store);
#endif
#endif //#ifdef CTP_PSENSOR_SUPPORT

//------------------------------------------------------------------------------//

s32 DrvMainTouchDeviceInitialize(void)
{
    s32 nRetVal = 0;
#ifdef CONFIG_ENABLE_FIRMWARE_DATA_LOG
    u8 *pDevicePath = NULL;
#endif //CONFIG_ENABLE_FIRMWARE_DATA_LOG

    DBG("*** %s() ***\n", __func__);

	MSG2238_read_ctpid_for_compatible();
    /* set sysfs for firmware */
    _gFirmwareClass = class_create(THIS_MODULE, "ms-touchscreen-msg20xx");
    if (IS_ERR(_gFirmwareClass))
        DBG("Failed to create class(firmware)!\n");

    _gFirmwareCmdDev = device_create(_gFirmwareClass, NULL, 0, NULL, "device");
    if (IS_ERR(_gFirmwareCmdDev))
        DBG("Failed to create device(_gFirmwareCmdDev)!\n");

    // version
    if (device_create_file(_gFirmwareCmdDev, &dev_attr_version) < 0)
        DBG("Failed to create device file(%s)!\n", dev_attr_version.attr.name);
    // update
    if (device_create_file(_gFirmwareCmdDev, &dev_attr_update) < 0)
        DBG("Failed to create device file(%s)!\n", dev_attr_update.attr.name);
    // data
    if (device_create_file(_gFirmwareCmdDev, &dev_attr_data) < 0)
        DBG("Failed to create device file(%s)!\n", dev_attr_data.attr.name);
#ifdef CONFIG_ENABLE_ITO_MP_TEST
    // test
    if (device_create_file(_gFirmwareCmdDev, &dev_attr_test) < 0)
        pr_err("Failed to create device file(%s)!\n", dev_attr_test.attr.name);
    // test_log
    if (device_create_file(_gFirmwareCmdDev, &dev_attr_test_log) < 0)
        pr_err("Failed to create device file(%s)!\n", dev_attr_test_log.attr.name);
    // test_fail_channel
    if (device_create_file(_gFirmwareCmdDev, &dev_attr_test_fail_channel) < 0)
        pr_err("Failed to create device file(%s)!\n", dev_attr_test_fail_channel.attr.name);
#endif //CONFIG_ENABLE_ITO_MP_TEST

#ifdef CONFIG_ENABLE_FIRMWARE_DATA_LOG
    // mode
    if (device_create_file(_gFirmwareCmdDev, &dev_attr_mode) < 0)
        DBG("Failed to create device file(%s)!\n", dev_attr_mode.attr.name);
    // packet
//    if (device_create_file(_gFirmwareCmdDev, &dev_attr_packet) < 0)
//        DBG("Failed to create device file(%s)!\n", dev_attr_packet.attr.name);
    // sensor
    if (device_create_file(_gFirmwareCmdDev, &dev_attr_sensor) < 0)
        DBG("Failed to create device file(%s)!\n", dev_attr_sensor.attr.name);
    // header
    if (device_create_file(_gFirmwareCmdDev, &dev_attr_header) < 0)
        DBG("Failed to create device file(%s)!\n", dev_attr_header.attr.name);
#endif //CONFIG_ENABLE_FIRMWARE_DATA_LOG

    // debug
    if (device_create_file(_gFirmwareCmdDev, &dev_attr_debug) < 0)
        DBG("Failed to create device file(%s)!\n", dev_attr_debug.attr.name);
    // platform version
    if (device_create_file(_gFirmwareCmdDev, &dev_attr_platform_version) < 0)
        DBG("Failed to create device file(%s)!\n", dev_attr_platform_version.attr.name);
#ifdef CONFIG_ENABLE_GESTURE_WAKEUP
    // gesture wakeup mode
    if (device_create_file(_gFirmwareCmdDev, &dev_attr_gesture_wakeup_mode) < 0)
        DBG("Failed to create device file(%s)!\n", dev_attr_gesture_wakeup_mode.attr.name);
#endif //CONFIG_ENABLE_GESTURE_WAKEUP
    // chip type
    if (device_create_file(_gFirmwareCmdDev, &dev_attr_chip_type) < 0)
        DBG("Failed to create device file(%s)!\n", dev_attr_chip_type.attr.name);
    // driver version
    if (device_create_file(_gFirmwareCmdDev, &dev_attr_driver_version) < 0)
        DBG("Failed to create device file(%s)!\n", dev_attr_driver_version.attr.name);

    dev_set_drvdata(_gFirmwareCmdDev, NULL);

#ifdef CONFIG_ENABLE_ITO_MP_TEST
    DrvIcFwLyrCreateMpTestWorkQueue();
#endif //CONFIG_ENABLE_ITO_MP_TEST
    
#ifdef CONFIG_ENABLE_FIRMWARE_DATA_LOG
    /* create a kset with the name of "kset_example" which is located under /sys/kernel/ */
    g_TouchKSet = kset_create_and_add("kset_example", NULL, kernel_kobj);
    if (!g_TouchKSet)
    {
        DBG("*** kset_create_and_add() failed, nRetVal = %d ***\n", nRetVal);

        nRetVal = -ENOMEM;
    }

    g_TouchKObj = kobject_create();
    if (!g_TouchKObj)
    {
        DBG("*** kobject_create() failed, nRetVal = %d ***\n", nRetVal);

        nRetVal = -ENOMEM;
		    kset_unregister(g_TouchKSet);
		    g_TouchKSet = NULL;
    }

    g_TouchKObj->kset = g_TouchKSet;

    nRetVal = kobject_add(g_TouchKObj, NULL, "%s", "kobject_example");
    if (nRetVal != 0)
    {
        DBG("*** kobject_add() failed, nRetVal = %d ***\n", nRetVal);

		    kobject_put(g_TouchKObj);
		    g_TouchKObj = NULL;
		    kset_unregister(g_TouchKSet);
		    g_TouchKSet = NULL;
    }
    
    /* create the files associated with this kobject */
    nRetVal = sysfs_create_group(g_TouchKObj, &attr_group);
    if (nRetVal != 0)
    {
        DBG("*** sysfs_create_file() failed, nRetVal = %d ***\n", nRetVal);

        kobject_put(g_TouchKObj);
		    g_TouchKObj = NULL;
		    kset_unregister(g_TouchKSet);
		    g_TouchKSet = NULL;
    }
    
    pDevicePath = kobject_get_path(g_TouchKObj, GFP_KERNEL);
    DBG("DEVPATH = %s\n", pDevicePath);
#endif //CONFIG_ENABLE_FIRMWARE_DATA_LOG

   // g_ChipType = DrvIcFwLyrGetChipType(); //move to MsDrvInterfaceTouchDeviceProbe for

#ifdef CONFIG_ENABLE_FIRMWARE_DATA_LOG
#if defined(CONFIG_ENABLE_CHIP_MSG26XXM)
    // get firmware mode for parsing packet judgement.
    g_FirmwareMode = DrvIcFwLyrGetFirmwareMode();
#endif //CONFIG_ENABLE_CHIP_MSG26XXM

    memset(&g_FirmwareInfo, 0x0, sizeof(FirmwareInfo_t));

    DrvIcFwLyrGetFirmwareInfo(&g_FirmwareInfo);

#if defined(CONFIG_ENABLE_CHIP_MSG21XXA) || defined(CONFIG_ENABLE_CHIP_MSG22XX)
    g_FirmwareMode = g_FirmwareInfo.nFirmwareMode;
#endif //CONFIG_ENABLE_CHIP_MSG21XXA || CONFIG_ENABLE_CHIP_MSG22XX
#endif //CONFIG_ENABLE_FIRMWARE_DATA_LOG

#ifdef CTP_PSENSOR_SUPPORT
	nRetVal = misc_register(&mstar2238_pls_device);
	if (nRetVal != 0) {
		printk("cannot register miscdev on mstar2238_pls_device\n");
	}
	#if 0
	if(device_create_file(&g_I2cClient->dev, &dev_attr_memory)>=0)
	{            
		printk("device_create_file_version \n");        
	}
	#endif
#endif


#ifdef CONFIG_UPDATE_FIRMWARE_BY_SW_ID
    DrvIcFwLyrCheckFirmwareUpdateBySwId();
#endif //CONFIG_UPDATE_FIRMWARE_BY_SW_ID

    return nRetVal;
}

//------------------------------------------------------------------------------//
