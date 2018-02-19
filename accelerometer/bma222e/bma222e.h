/*
 *
 * BMA222E driver for MT65xx
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */
#ifndef BMA222E_H
#define BMA222E_H
	 
#include <linux/ioctl.h>
 
#define BMA222E_I2C_SLAVE_WRITE_ADDR		0x30
 #define BMA222E_DEV_NAME        "BMA222E"
 /* BMA222E Register Map  (Please refer to BMA222E Specifications) */
#define BMA222E_REG_DEVID				0x00
#define BMA222E_FIXED_DEVID			0xF8
#define BMA222E_REG_OFSX				0x16
#define BMA222E_REG_OFSX_HIGH			0x1A
#define BMA222E_REG_BW_RATE			0x10
#define BMA222E_BW_MASK				0x1f
#define BMA222E_BW_200HZ				0x0d
#define BMA222E_BW_100HZ				0x0c
#define BMA222E_BW_50HZ				0x0b
#define BMA222E_BW_25HZ				0x0a
#define BMA222E_REG_POWER_CTL		0x11		
#define BMA222E_REG_DATA_FORMAT		0x0f
#define BMA222E_RANGE_MASK			0x0f
#define BMA222E_RANGE_2G				0x03
#define BMA222E_RANGE_4G				0x05
#define BMA222E_RANGE_8G				0x08
#define BMA222E_REG_DATAXLOW			0x03
#define BMA222E_REG_DATA_RESOLUTION	0x14
#define BMA222E_MEASURE_MODE			0x80	
#define BMA222E_SELF_TEST           			0x32
#define BMA222E_SELF_TEST_AXIS_X		0x01
#define BMA222E_SELF_TEST_AXIS_Y		0x02
#define BMA222E_SELF_TEST_AXIS_Z		0x03
#define BMA222E_SELF_TEST_POSITIVE	0x00
#define BMA222E_SELF_TEST_NEGATIVE	0x04
#define BMA222E_INT_REG_1           			0x16
#define BMA222E_INT_REG_2          		 	0x17

#define BMA222E_SUCCESS						0
#define BMA222E_ERR_I2C						-1
#define BMA222E_ERR_STATUS					-3
#define BMA222E_ERR_SETUP_FAILURE			-4
#define BMA222E_ERR_GETGSENSORDATA			-5
#define BMA222E_ERR_IDENTIFICATION			-6	 
	 
#define BMA222E_BUFSIZE				256
	 
#endif

