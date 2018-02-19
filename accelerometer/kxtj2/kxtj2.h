
#ifndef KXTJ2_H
#define KXTJ2_H
	 
#include <linux/ioctl.h>
	 
#define KXTJ2_I2C_SLAVE_ADDR		0x1C //8bit I2C address//pin1=GND ,if pin1=IOVDD ,address = 0x1e
	 
 /* KXTJ2 Register Map  (Please refer to KXTJ2 Specifications) */
#define KXTJ2_REG_DEVID				0x0F
#define	KXTJ2_REG_BW_RATE			0x21
#define KXTJ2_REG_POWER_CTL			0x1B
#define KXTJ2_REG_CTL_REG3			0x1D

#define KXTJ2_DCST_RESP				0x0C
#define KXTJ2_REG_DATA_FORMAT		0x1B
#define KXTJ2_REG_DATA_RESOLUTION	0x1B

#define KXTJ2__RESOLUTION_MASK		0x40
#define KXTJ2_BW_MASK               0x0F

#define KXTJ2_REG_DATAX0			0x06

#define KXTJ2_FIXED_DEVID			0x12

#define KXTJ2_BW_200HZ				0x05
#define KXTJ2_BW_100HZ				0x04
#define KXTJ2_BW_50HZ				0x03

#define KXTJ2__RESOLUTION_12BIT     0x40
#define KXTJ2__RESOLUTION_8BIT 		0X00

#define KXTJ2_MEASURE_MODE			0x80		 
#define KXTJ2_RANGE_MASK			0x18
#define KXTJ2_RANGE_2G				0x00
#define KXTJ2_RANGE_4G				0x08
#define KXTJ2_RANGE_8G				0x10
#define KXTJ2_REG_INT_ENABLE		0x1E

#define KXTJ2_SELF_TEST        		0x10
	 	 
	 
#define KXTJ2_SUCCESS						0
#define KXTJ2_ERR_I2C						-1
#define KXTJ2_ERR_STATUS					-3
#define KXTJ2_ERR_SETUP_FAILURE				-4
#define KXTJ2_ERR_GETGSENSORDATA			-5
#define KXTJ2_ERR_IDENTIFICATION			-6
	 
	 
	 
#define KXTJ2_BUFSIZE				256
	 
#endif

