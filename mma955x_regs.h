/******************************************************************************
 * Copyright (C) 2012 Cosmed, Ltd. <http://www.cosmed.com/>
 * Author: Gabriele Filosofi <gabrielef@cosmed.it>
 * File Name		: mma_regs.h
 * Description		: data structure related to MMA955X.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THE PRESENT SOFTWARE IS PROVIDED ON AN "AS IS" BASIS, WITHOUT WARRANTIES
 * OR CONDITIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED, FOR THE SOLE
 * PURPOSE TO SUPPORT YOUR APPLICATION DEVELOPMENT.
 * AS A RESULT, FREESCALE SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
 * INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
 * CONTENT OF SUCH SOFTWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
 * INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
 *
 * THIS SOFTWARE IS SPECIFICALLY DESIGNED FOR EXCLUSIVE USE WITH FREESCALE PARTS.
 ******************************************************************************
 * Revision 1.0.0 3/15/2012 First Release;
 ******************************************************************************/
#ifndef __MMA_REGS_H__
#define __MMA_REGS_H__

#define MODULE_NAME	"mma955x"
#define DEVICE_NAME	"mma"
#define DRIVER_VERSION	"Ver. 1.0"
#define VENDOR_NAME	"Cosmed"

/* Macros for handling a big-endian device like mma955x */
#define LOBYTE(W)	(unsigned char)((W) & 0xFF)
#define HIBYTE(W)	(unsigned char)(((W) >> 8) & 0xFF)
#define MAKEWORD(h, l)	(unsigned short)((l) | ((h) << 8))
#define MAKEDWORD(hh, hl, lh, ll)	(unsigned int)((MAKEWORD(lh, ll)) | ((MAKEWORD(hh, hl)) << 16))
#define SWAP16(x) ((u16)((((u16)(x) & (u16)0x00ffU) << 8) | \
			(((u16)(x) & (u16)0xff00U) >> 8) ))
#define SWAP32(x) ((u32)((((u32)(x) & (u32)0x000000ffUL) << 24) | \
			(((u32)(x) & (u32)0x0000ff00UL) <<  8) | \
			(((u32)(x) & (u32)0x00ff0000UL) >>  8) | \
			(((u32)(x) & (u32)0xff000000UL) >> 24) ))

/* Macros for fixed point arithmetic */
#define FP_SHIFT	3     // shifts to produce a fixed-point number
#define FP_SCALE	1000  // scaling factor
#define PRECISION	1000
#define INT_TO_FIXP(n) ((FIXPOINT)((n * FP_SCALE)))
#define FLOAT_TO_FIXP(n) ((FIXPOINT)((float)n * FP_SCALE))
#define FIXP_INT_PART(n) (n / FP_SCALE)
#define FIXP_DEC_PART(n) (n % FP_SCALE)

typedef unsigned int FIXPOINT;

/* Accelerometer ODRs (Hz) */
static const FIXPOINT mma_odr_list[] = { 
	FLOAT_TO_FIXP(488.0),
	FLOAT_TO_FIXP(244.0),
	FLOAT_TO_FIXP(122.0),
	FLOAT_TO_FIXP(61.0),
	FLOAT_TO_FIXP(30.0),
	FLOAT_TO_FIXP(15.0),
	FLOAT_TO_FIXP(7.0),
	FLOAT_TO_FIXP(3.0),
	-1,
};

/* Gyroscope ODRs (Hz) */
static const FIXPOINT gyro_odr_list[] = { 
	FLOAT_TO_FIXP(760.0),
	FLOAT_TO_FIXP(380.0),
	FLOAT_TO_FIXP(190.0),
	FLOAT_TO_FIXP(95.0),
	-1,
};

/* P/T sensor ODRs (Hz) */
static const FIXPOINT amb_odr_list[] = { 
	FLOAT_TO_FIXP(488.0),
	FLOAT_TO_FIXP(244.0),
	FLOAT_TO_FIXP(122.0),
	-1,
};


enum {
	MMA_SLEEP = 0,		/* stopped AFE activity */
	MMA_DOZE,		/* slow AFE activity (not used here) */
	MMA_RUN,		/* full AFE activity */
};

/* Accelerometer Full-Scale or ranges */
#define ACC_RANGE_2G	2	/* range is +-2g; sensitivity is 0.061 mg/LSB or 16394/g */
#define ACC_RANGE_4G	4	/* range is +-4g; sensitivity is 0.122 mg/LSB or 8196/g (default) */
#define ACC_RANGE_8G	8	/* range is +-8g; sensitivity is 0.244 mg/LSB or 4096/g */

static const int acc_supported_resolution[] = {16, 14, 12, 10, -1};

/* Data types */
#define SENSOR_DATA	0x01
#define FIFO_DATA	0x02

/* sysfs entries table */
struct SysfsInfo_t {
	char 	* grpName;			// sysfs group name; NULL is treated as ungrouped
	struct 	device_attribute *AttrEntry;	// Pointer to attribute table
	int 	TotalEntries;			// Number of attributes
	int 	Instance;			// No. of instances for group
};

typedef struct {
	short x;
	short y;
	short z;
}vec_t;

typedef struct {
	vec_t G;	/* acceleration */
	vec_t O;	/* angular rate */
/*
Pressure data is arranged as 20-bit unsigned value. The data is stored as Pa with bits 19-2 and with fractions of a Pa in bits 1-0.
Altitude data is arranged as 20-bit 2’s compl. value. The data is stored as meters with bits 19-4 and with fractions of a meter in bits 3-0.
Temperature data is arranged as 12-bit 2’s compl. value. The data is stored as degrees °C with bits 11-4 and with fractions of a degree °C in bits 3-0.
*/
	int P;		/* barometric absolute pressure in .25Pa units or altimeter output in .0625m units */
	short T;	/* K5 internal temperature in .0625°C units */
}HubData_t, *pHubData_t;

typedef struct {
	vec_t G;	/* acceleration */
	short distance;
	short speed;
	short sleepCnt;
	short status;
	short stepCnt;
//	short calories;
}PedData_t, *pPedData_t;

struct chipInfo_t{
	int chipId;		/* x for item mma955x */
	int chipAddr;		/* i2c address */
	char name[16];		/* mma9550, mma9551,.. */
	struct i2c_client *client;

	int activity;
	int position;		/* defines the chip orientation relative to PCB */

	/* FIFO parameters */
	int enablefifo;
	int fifo_threshold;
	int min_fifo_th;
	int max_fifo_th;

	FIXPOINT * pOdrList;
	char ** psamplinglist;
	int * pSupportedResolution;
	int * pSupportedRange;
	FIXPOINT * pCalibration_offsetList;
	FIXPOINT * pZ_lock_angle_thresholdList;
	int * pBack_front_trip_angle_thresholdList;
	int * pTrip_angle_thresholdList;

	int odr;
	int resolution;
	int poll_time;

	/* MMA9550-specific (hub) */
	int acc_invert;
	int acc_range;
	int acc_output_stage;
	short acc_ofs_x;
	short acc_ofs_y;
	short acc_ofs_z;
	int gyro_invert;
	int gyro_range;
	short gyro_ofs_x;
	short gyro_ofs_y;
	short gyro_ofs_z;
	int amb_p_mode;
	char amb_ofs_P;
	char amb_ofs_H;
	char amb_ofs_T;

	/* MMA9553-specific (pedometer) */
	int sleep_min;
	int sleep_max;
	int sleep_thd;
	int steplen;
	int height;
	int weight;
	int gender;
	int filter_step;
	int filter_time;
	int speed_period;
	int step_coalesce;
	int activity_thd;

	/* sysfs entries */
	struct SysfsInfo_t *pSysfsInfo;
	int SysfsInfoSize;

	/* Function pointers */
	int (*Init)(struct chipInfo_t *);
	int (*SendCommand)(struct i2c_client *, u8 *, u8, u8 *, u8);
	int (*SendRSC)(struct i2c_client *client, int, int, int);
	int (*Read)(struct i2c_client *, void *);
};


struct mxc_mma_device_t{
	struct chipInfo_t *pChip;

	/* Character (FIFO) layer members */
	int major;
	char devname[32];
	int version;

	//struct input_polled_dev *poll_dev;

	struct mutex data_lock;

	/* Control information */
	int data_event_type;

	/* Kernel object */
	struct kobject *kobj;

	/* Input layer members */
	struct input_dev *inp;

	struct class *class;
	struct device *sys_device;

	struct task_struct *hIsrThread;
};

#endif //MMA_REGS_H
