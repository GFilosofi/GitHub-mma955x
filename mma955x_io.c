/******************** (C) COPYRIGHT 2012 Cosmed, Ltd. *************
* Copyright (C) 2012 Cosmed, Ltd. <http://www.cosmed.com/>
*
* File Name		: mma_955x.c
* Authors		: Gabriele Filosofi <gabrielef@cosmed.it>
			  Gabriele is willing to be considered the contact and update points
			  for the driver
* Version		: V.1.0.0
* Date			: 2012/Sept/20
* Description		: Linux kernel driver for Freescale 3-Axis Intelligent
* Motion-Sensing Platform MMA9550/MMA9551/MMA9553/MMA9559.
* Note: The device incorporates dedicated accelerometer MEMS transducers,
* signal-conditioning, data conversion, a 32-bit programmable
* microcontroller, and flexible communications and I/O pins.
* The host sends the commands to MMA955x devices via either the
* I2C or SPI serial bus and the MMA955x platform’s Command Interpreter
* (CI) interprets and responds to those commands.
* The MMA955x device can also be configured and programmed to act as
* a bus master ("HUB application"): the device talks to secondary sensors
* such as pressure sensors, magnetometers, or gyroscopes through its
* master serial communication interface. K5MB has the following devces connected to this bus:
* a. L3GD20: gyroscope
* b. MPL3115A2: altimeter/barometer and thermometer
* Three device member exist, each with a specific set of pre-loaded applications:
*
* MMA9550 Basic Motion
* MMA9551 Gesture
* MMA9553 Pedometer
* MMA9559 Foundation
*
* Custom applications (such as our "HUB") can be added to any devices.
******************************************************************************
*  This program is free software; you can redistribute it and/or modify
*  it under the terms of the GNU General Public License as published by
*  the Free Software Foundation; either version 2 of the License, or
*  (at your option) any later version.
*
*  This program is distributed in the hope that it will be useful,
*  but WITHOUT ANY WARRANTY; without even the implied warranty of
*  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*  GNU General Public License for more details.
*
*  You should have received a copy of the GNU General Public License
*  along with this program; if not, write to the Free Software
*  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
******************************************************************************
* Revision 1.0.0 20/09/2012 First Release;
******************************************************************************/
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/pm.h>
#include <linux/mutex.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/hwmon-sysfs.h>
#include <linux/err.h>
#include <linux/hwmon.h>
#include <linux/input-polldev.h>
#include <linux/delay.h>
#include <linux/kthread.h>
#include <cosmed/mma955x.h>
#include "mma955x_io.h"
#include "mma955x_regs.h"
#include "mma955x_sysfs.h"

#undef DEBUG
//#define DEBUG

//extern const struct file_operations mma_fops;
static struct file_operations mma_ops =
{//todo
};

extern int mmaInitializeInputInterface(struct mxc_mma_device_t *pDev);
extern int mmaDeInitializeInputInterface(struct mxc_mma_device_t *pDev);
extern int mmaReportEvent(struct mxc_mma_device_t *pDev, void *pBuf);
extern int mmaInitializeSysfs(struct i2c_client *pClient);
extern int mmaDeInitializeSysfs(struct mxc_mma_device_t *pDev);

static int mmaInit(struct chipInfo_t *pChip);
static int mmaSendCommand(struct i2c_client *client, u8 *txbuf, u8 txlen, u8 *rxbuf, u8 rxlen);
static int rsc_sendCommand(struct i2c_client *client, int appId, int RSC, int bClear);
static int hub_read(struct i2c_client *client, void *pData);
static int ped_read(struct i2c_client *client, void *pData);

/* Variable to determine where the device is in poll mode or interrupted mode */
#if 0 /* WARNING: this part of code must be revisited to allow multi-device operations */
static int poll_mode = 0;
static struct timer_list stall_timer; /* Workaround to clear pending interrupt*/
static struct semaphore chip_ready;
#endif

#define CONFIG_SENSORS_MMA_POSITION	0

#define MODE_CHANGE_DELAY_MS	150	/* ms */

/* poll interval (ms)
#define POLL_INTERVAL_MIN	1
#define POLL_INTERVAL_MAX	500
#define POLL_INTERVAL		50
#define POLL_STOP_TIME		10000	//if sensor is standby, set POLL_STOP_TIME to slow down the poll
*/

/* Barometer/Altimeter Full-Scale or ranges */
#define PR_RANGE_0P25		0	/* range is 20..110 kPa; max sensitivity is 0.25 Pa/LSB or 0.3m/LSB */

#if 0
/* Barometer and Termometer output data rate ODR */
#define	LPS001WP_PRS_ODR_MASK	0x30	/* Mask to access odr bits only	*/
#define	LPS001WP_PRS_ODR_7_1	0x00	/* 7Hz baro and 1Hz term ODR	*/
#define	LPS001WP_PRS_ODR_7_7	0x01	/* 7Hz baro and 7Hz term ODR	*/
#define	LPS001WP_PRS_ODR_12_12	0x11	/* 12.5Hz baro and 12.5Hz term ODR */

static const struct {
	unsigned int cutoff_ms;
	unsigned int mask;
} acc_odr_table[] = {
		{122,	LPS001WP_PRS_ODR_12_12 },
		{244,	LPS001WP_PRS_ODR_7_7 },
		{488,	LPS001WP_PRS_ODR_7_1 },
};
#endif

//static u8 rst_rom[]		= { RST_CLR_SUSP_SYSID, CMD_WR_CFG, 0x01, 0x01, 0x80, };	/* reboot from flash code to ROM CI */

/* assign Quick-Read registers (MB26-MB31) to low-pass-filtered data in AFE registers 0x18 through 0x1D (FRONTEND_488_100_LPF) */
//static u8 mbox_cfg_1[]	= { MBOX_APPID, CMD_WR_CFG, 0x2C, 0x0C, 0x06, 0x18, 0x06, 0x19, 0x06, 0x1A, 0x06, 0x1B, 0x06, 0x1C, 0x06, 0x1D, };

/* set MBOX to Legacy mode. Device's default at reset is Normal mode */
//static u8 mboxcfg_legacy[]	= { MBOXCFG_APPID, CMD_WR_CFG, 0x00, 0x01, 0x10, };

#if 0
/* read the six bytes starting at MB26, which contains the XYZ data in constant-read mode */
static u8 mbox_sts_qread[]	= { MBOX_APPID, CMD_RD_STS, 0x1A, 0x06, };
#endif

/* set up the interrupt pin INT_O to go active after a COCO */
//static u8 mboxcfg_int[]	= { MBOXCFG_APPID, CMD_WR_CFG, 0x00, 0x01, 0x80, };

/* Gp and Op data vector rotations to compensate for device orientation relative to display graphics */
static int mma_position_setting[8][3][3] =
{
  {{ 1,  0,  0}, { 0,  1,  0}, {0, 0,  1}}, /* this is the identity matrx (no rotation) */
  {{ 0, -1,  0}, { 1,  0,  0}, {0, 0,	1}},
  {{-1,  0,  0}, { 0, -1,  0}, {0, 0,	1}},
  {{ 0,  1,  0}, {-1,  0,  0}, {0, 0,	1}},

  {{-1,  0,  0}, { 0,  1,  0}, {0, 0, -1}},
  {{ 0,  1,  0}, { 1,  0,  0}, {0, 0, -1}},
  {{ 1,  0,  0}, { 0, -1,  0}, {0, 0, -1}},
  {{ 0, -1,  0}, {-1,  0,  0}, {0, 0, -1}},
};

static struct mma955x_platform_data *plat_data;


/* MMA955x-common Sysfs info */
static struct device_attribute mma_attributes[] = {
	__ATTR(device_id, S_IRUGO, mma_devid_show, NULL),
	__ATTR(name, S_IRUGO, mma_name_show, NULL),
	__ATTR(poll_time, S_IWUSR | S_IRUGO, mma_poll_time_show, mma_poll_time_store),
	__ATTR(sleep, S_IWUSR | S_IRUGO, mma_sleep_show, mma_sleep_store),
	__ATTR(vendor, S_IRUGO, mma_vendor_show, NULL),
	__ATTR(version, S_IRUGO, mma_version_show, NULL),
};

/* MMA9550-specific Sysfs info (hub) */
static struct device_attribute hub_attributes[] = {
	__ATTR(resolution, S_IWUSR | S_IRUGO, hub_resolution_show, hub_resolution_store),
	__ATTR(resolutions, S_IRUGO, hub_resolutions_show, NULL),
	__ATTR(position, S_IWUSR | S_IRUGO, hub_position_show, hub_position_store),
	__ATTR(acc_invert, S_IWUSR | S_IRUGO, hub_acc_invert_show, hub_acc_invert_store),
	__ATTR(gyro_invert, S_IWUSR | S_IRUGO, hub_gyro_invert_show, hub_gyro_invert_store),
	__ATTR(acc_output_stage, S_IWUSR | S_IRUGO, hub_acc_output_stage_show, hub_acc_output_stage_store),
	__ATTR(acc_offset, S_IWUSR | S_IRUGO, hub_acc_offset_show, hub_acc_offset_store),
	__ATTR(acc_range, S_IWUSR | S_IRUGO, hub_acc_range_show, hub_acc_range_store),
	__ATTR(amb_p_mode, S_IWUSR | S_IRUGO, hub_amb_p_mode_show, hub_amb_p_mode_store),
	__ATTR(gyro_range, S_IWUSR | S_IRUGO, hub_gyro_range_show, hub_gyro_range_store),
	__ATTR(amb_offset, S_IWUSR | S_IRUGO, hub_amb_offset_show, hub_amb_offset_store),
	__ATTR(utility, S_IWUSR, NULL, hub_utility_store),
};

/* MMA9553-specific Sysfs info (pedometer) */
static struct device_attribute ped_attributes[] = {
	__ATTR(sleep_min, S_IWUSR | S_IRUGO, ped_sleep_min_show, ped_sleep_min_store),
	__ATTR(sleep_max, S_IWUSR | S_IRUGO, ped_sleep_max_show, ped_sleep_max_store),
	__ATTR(sleep_thd, S_IWUSR | S_IRUGO, ped_sleep_thd_show, ped_sleep_thd_store),
	__ATTR(steplen, S_IWUSR | S_IRUGO, ped_steplen_show, ped_steplen_store),
	__ATTR(height, S_IWUSR | S_IRUGO, ped_height_show, ped_height_store),
	__ATTR(weight, S_IWUSR | S_IRUGO, ped_weight_show, ped_weight_store),
	__ATTR(filter_step, S_IWUSR | S_IRUGO, ped_filter_step_show, ped_filter_step_store),
	__ATTR(filter_time, S_IWUSR | S_IRUGO, ped_filter_time_show, ped_filter_time_store),
	__ATTR(gender, S_IWUSR | S_IRUGO, ped_gender_show, ped_gender_store),
	__ATTR(speed_period, S_IWUSR | S_IRUGO, ped_speed_period_show, ped_speed_period_store),
	__ATTR(step_coalesce, S_IWUSR | S_IRUGO, ped_step_coalesce_show, ped_step_coalesce_store),
	__ATTR(activity_thd, S_IWUSR | S_IRUGO, ped_activity_thd_show, ped_activity_thd_store),
};

static struct SysfsInfo_t hub_sysfsInfo[] = {
	{
		NULL,
		&mma_attributes[0],
		(sizeof(mma_attributes)/sizeof(mma_attributes[0])),
		1
	},
	{
		NULL,
		&hub_attributes[0],
		(sizeof(hub_attributes)/sizeof(hub_attributes[0])),
		1
	},
};

static struct SysfsInfo_t ped_sysfsInfo[] = {
	{
		NULL,
		&mma_attributes[0],
		(sizeof(mma_attributes)/sizeof(mma_attributes[0])),
		1
	},
	{
		NULL,
		&ped_attributes[0],
		(sizeof(ped_attributes)/sizeof(ped_attributes[0])),
		1
	},
};

/* MMA9550-specific chip info (hub) */
struct chipInfo_t mma9550 = {
	.chipId = MMA9550_ID,
	.Init = mmaInit,
	.SendCommand = mmaSendCommand,
	.SendRSC = rsc_sendCommand,
	.Read = hub_read,
	.name = "mma9550",
	.poll_time = 5,		/* ms / 10 */
	.pOdrList = (FIXPOINT *)mma_odr_list,
	.pSupportedResolution = (int *)acc_supported_resolution,
/*
	.pSupportedRange = (FIXPOINT *)SupportedRange,
	.psamplinglist = (char **)samplinglist,
*/
	.pSysfsInfo = hub_sysfsInfo,
	.SysfsInfoSize = sizeof(hub_sysfsInfo)/sizeof(hub_sysfsInfo[0]),

	.amb_p_mode = HUB_AMB_BAROMETER_MODE,
};

/* MMA9553-specific chip info (pedometer) */
struct chipInfo_t mma9553 = {
	.chipId = MMA9553_ID,
	.Init = mmaInit,
	.SendCommand = mmaSendCommand,
	.SendRSC = rsc_sendCommand,
	.Read = ped_read,
	.name = "mma9553",
	.poll_time = 5,		/* ms / 10 */
	.pOdrList = (FIXPOINT *)mma_odr_list,
	.pSupportedResolution = (int *)acc_supported_resolution,
/*
	.pSupportedRange = (FIXPOINT *)SupportedRange,
	.psamplinglist = (char **)samplinglist,
*/
	.pSysfsInfo = ped_sysfsInfo,
	.SysfsInfoSize = sizeof(ped_sysfsInfo)/sizeof(ped_sysfsInfo[0]),

	.sleep_min = 0x0000,
	.sleep_max = 0x0000,
	.sleep_thd = 0x0001,
	.steplen = 0x00,
	.height = 175,			/* subject's height (cm) */
	.weight = 80,			/* subject's weight (kg) */
	.gender =  PEDOMETER_CFG_GENDER_FEMALE,
	.filter_step = 4,
	.filter_time = 3,
	.speed_period = 0x05,
	.step_coalesce = 0x01,
	.activity_thd = 0x0000,
};

struct chipInfo_t * chipTable[] = {
	&mma9550,
	&mma9553,
};

/*
* This method is used to read a response packet from device.
* It is assumed the device's flash CI is running.
* client : Pointer to i2c client structure
* buf : ptr to a buffer in which response data to be read (header is skipped)
* length : no of bytes to read
* return : Actual number of bytes read
*
* Mailbox Response format:
* MB0: APP_ID
* MB1: STATUS (0x80 - Command Complete, no errors)
* MB2: requested data count
* MB3: actual data count
* MB4..MB23: data
*/
static int mmaGetResponse(struct i2c_client *client, u8 length, u8 *buf)
{
	int ret = 0;
	struct i2c_msg msg[2];
	u8 w_data, err;
	u8 *p;

	p = kzalloc(length + 4, GFP_KERNEL);
	if (!p) {
		ret = -ENOMEM;
		dev_err(&client->dev, "\n%s:: Memory allocation fails %x", __func__, ret);
		goto out;
	}
	memset(p, 0, length + 4);

	/* The first byte to write is the register address of the first device internal register (0x00 to 0x21)
	   that is to be read. It must be 0 */
	w_data = 0x00;
	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].len = 1;
	msg[0].buf = &w_data;

	msg[1].addr = client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len = length + 4;
	msg[1].buf = p;

	ret = i2c_transfer(client->adapter, msg, 2);
	if (ret < 0) {
		dev_err(&client->dev, "\n%s:: Read from device fails status %d", __func__, ret);
		goto exit_free;
	}
	//printk("\nREPLY: %x %x %x %x", p[0], p[1], p[2], p[3]);

	/* check for errors */
	err = p[1] & 0x7f;
	if (err) {
		dev_err(&client->dev, "\n%s:: Read from device error status %d", __func__, -err);
	}

	ret = (unsigned int)p[2];
	memcpy(buf, p+4, p[2]);
exit_free:
	kfree(p);
out:
	return ret;
}

/*
* This method is used to transfer a command packet to device and read back a response if required.
* It is assumed the device's flash CI is running.
* client : Pointer to i2c client structure
* txbuf : ptr to data packet
* txlen : no of bytes to be written
* rxbuf : ptr to data buffer for the response packet if needed
* rxlen : no of bytes read
* returns : 0 for successful transfer or actual number of bytes read
*
* Mailbox Command format:
* MB0: APP_ID (the application the host wants to write or read)
* MB1: Command (four different command exist)
* MB2: offset for start op. (points to specific data)
* MB3: number of byte the host intends to transfer
* MB4..MB23: data (only for write op.)
*/
static int mmaSendCommand(struct i2c_client *client, u8 *txbuf, u8 txlen, u8 *rxbuf, u8 rxlen)
{
	int ret;
	struct i2c_msg msg[2];
	u8 w_data;
	int retries = 15;
	u8 val;

	/* The first byte to write is the register address of the first device internal register (0x00 to 0x21)
	   that is to be updated. It must be 0 */
	w_data = 0x00;
	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].len = 1;
	msg[0].buf = &w_data;

	msg[1].addr = client->addr;
	msg[1].flags = I2C_M_NOSTART;
	msg[1].len = txlen;
	msg[1].buf = txbuf;

	ret = i2c_transfer(client->adapter, msg, 2);
	if (ret < 0) {
		dev_err(&client->dev, "\n%s:: Write to device fails status %d", __func__, ret);
		goto out;
	}

	do {/* All CI response packets utilize the same set of common return codes in the most-significant nibble of
	       Mailbox 1. Bit 8 is used as “Command Complete” or “COCO.” It is set to 1 when the command is complete
	       (with or without errors). Mailbox 1 bits 6-4 hold any applicable error code. */
		msleep(1);
		//printk("+");
		val = i2c_smbus_read_byte_data(client, 1);
	} while (!(val & 0x80) && retries--);
	if (retries < 0) {
		dev_err(&client->dev, "\n%s:: Read request command not completed", __func__);
		ret = -EINVAL;
		goto out;
	}

	if(txbuf[1] != CMD_WR_CFG)
	{/* Only if the command is a "read" one a read transfer follows */
		ret = mmaGetResponse(client, rxlen, rxbuf);
		if (ret < 0) {
			goto out;
		}
	}
	else
		ret = 0;
out:
	return ret;
}

/*
* This method is used to generate an hw reset.
* It is assumed the device's ROM CI is running.
* client : Pointer to i2c client structure
* return 0 : For successful transfer
*/
static int mmaReset(struct i2c_client *client)
{
	int ret;
	struct i2c_msg msg[2];
	u8 w_data;
	u8 rst_flash[2] = { 0x29, CMD_RD_FIX };		/* reboot to flash (it is interpreted y ROM-based CI) */

	w_data = 0x00;
	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].len = 1;
	msg[0].buf = &w_data;

	msg[1].addr = client->addr;
	msg[1].flags = I2C_M_NOSTART;
	msg[1].len = sizeof(rst_flash);
	msg[1].buf = rst_flash;

	ret = i2c_transfer(client->adapter, msg, 2);
	if (ret < 0) {
		dev_err(&client->dev, "\n%s:: Write to device fails status %d", __func__, ret);
		goto out;
	}
	ret = 0;
out:
	msleep(MODE_CHANGE_DELAY_MS);
	return ret;
}

#if 0
/*
* This method is used to schedule the running of one or more applications.
* It is assumed the device's flash CI is running.
* client : Pointer to i2c client structure
* appIds : 32-bit word with all 1s corresponding to application IDs want to start
* return 0 : For successful transfer
*/
static int sched_reqStart(struct i2c_client *client, int appIds)
{
	int ret;
	struct mxc_mma_device_t *pDev = i2c_get_clientdata(client);
	u8 reg_req[4] = { SCHED_APPID, CMD_RD_CFG, 0x00, 0x04};
	u8 reg_set[8] = { SCHED_APPID, CMD_WR_CFG, 0x00, 0x04, 0, 0, 0, 0};
	int val = 0;

	mutex_lock(&pDev->data_lock);
	ret = mmaSendCommand(client, reg_req, sizeof(reg_req), (u8 *)&val, 4);
	if(ret < 4)
		goto out;
	val |= appIds;

	reg_set[4] = (u8)(val >> 24);
	reg_set[5] = (u8)(val >> 16);
	reg_set[6] = (u8)(val >> 8);
	reg_set[7] = (u8)val;

	ret = mmaSendCommand(client, reg_set, sizeof(reg_set), NULL, 0);
	if(ret < 0)
		goto out;
	ret = 0;
out:
	mutex_unlock(&pDev->data_lock);
	return ret;
}
#endif

/*
* This method is used to get and print the chip version
* client : Pointer to i2c client structure
* return 0 : For successful
*/
static int ver_getVersion(struct i2c_client *client, mmaVer_t *p)
{
	u8 cmd[4] = { VERSION_APPID, 0x00, 0, sizeof(mmaVer_t)};
	return mmaSendCommand(client, cmd, sizeof(cmd), (u8 *)p, sizeof(mmaVer_t));
}

/*
* This method is used to reset, suspend or clear an application
* client : Pointer to i2c client structure
* appId : application ID (0..31)
* command : RESET/SUSPEND/CLEAR
* bClear : set to 1 if want o exit suspend state
* return 0 : For successful
*/
static int rsc_sendCommand(struct i2c_client *client, int appId, int RSC, int bClear)
{
	int ret;
	struct mxc_mma_device_t *pDev = i2c_get_clientdata(client);
	mmaRscCfg_t rsc;
	u8 cmd[8] = { RST_CLR_SUSP_SYSID, CMD_WR_CFG, offsetof(mmaRscCfg_t, sr[RSC].word), sizeof(rsc.sr[RSC].word), 0, 0, 0, 0 };
	unsigned int val = 0;

	if(!bClear) {
		val = (1 << appId);
	}
	cmd[4] = (u8)(val >> 24);
	cmd[5] = (u8)(val >> 16);
	cmd[6] = (u8)(val >> 8);
	cmd[7] = (u8)(val);

	mutex_lock(&pDev->data_lock);
	ret = mmaSendCommand(client, cmd, sizeof(cmd), NULL, 0);
	if(ret < 0)
		goto out;
	ret = 0;
out:
	mutex_unlock(&pDev->data_lock);
	return ret;
}

#if 0
static int mma9550_qread(struct i2c_client *client)
{//TODO
/* read the six bytes starting at MB26, which contains the XYZ data in constant-read mode */
	int ret;
	u8 buf[6];
	memset(buf, 0, sizeof(buf));

	ret = mmaSendCommand(client, mbox_sts_qread, sizeof(mbox_sts_qread), buf, sizeof(buf));
	#ifdef DEBUG
	printk("\nacc_X = 0x%04x", MAKEWORD(buf[0],buf[1]));
	printk("\nacc_Y = 0x%04x", MAKEWORD(buf[2],buf[3]));
	printk("\nacc_Z = 0x%04x", MAKEWORD(buf[4],buf[5]));
	#endif
	return ret;
}
#endif

/*
* This method reads accelerometer data at the assigned AFE output stage
* stage : AFE output stage
* return 0 : For successful */
static int afe_getStage(struct i2c_client *client, int stage, vec_t *p)
{
	int ret;
	u8 cmd[4] = { AFE_APPID, CMD_RD_STS, offsetof(mmaAfeSts_t, acc_out[stage]), sizeof(vec_t)};
	ret = mmaSendCommand(client, cmd, sizeof(cmd), (u8 *)p, sizeof(vec_t));
	#ifdef DEBUG
	printk("\nacc_X = %d", (short)SWAP16(p->x));
	printk("\nacc_Y = %d", (short)SWAP16(p->y));
	printk("\nacc_Z = %d", (short)SWAP16(p->z));
	printk("\n");
	#endif
	return ret;
}

/*
* This method reads AFE output data
* return 0 : For successful */
int afe_getSts(struct i2c_client *client, mmaAfeSts_t *p)
{
	int ret, stage;
	u8 cmd[4] = { AFE_APPID, CMD_RD_STS, 0, sizeof(mmaAfeSts_t)};
	ret = mmaSendCommand(client, cmd, sizeof(cmd), (u8 *)p, sizeof(mmaAfeSts_t));
//	#ifdef DEBUG
	for(stage = 0; stage < FRONTEND_OUTPUTS; stage++)
	{
		printk("\nstage %d: acc_X = %d", stage, (short)SWAP16(p->acc_out[stage][0]));
		printk("\nstage %d: acc_Y = %d", stage, (short)SWAP16(p->acc_out[stage][1]));
		printk("\nstage %d: acc_Z = %d", stage, (short)SWAP16(p->acc_out[stage][2]));
	}
	printk("\noutputTemp = 0x%04x", (short)SWAP16(p->outputTemp));
	printk("\noutputEIC = 0x%04x", (short)SWAP16(p->outputEIC));
	printk("\nframeCounter = 0x%04x", (short)SWAP16(p->frameCounter));
	printk("\n");
//	#endif
	return ret;
}

#if 0
/*
* This method reads basic motion output data
* return 0 : For successful */
static int basic_getSts(struct i2c_client *client)
{
	int ret;
	mmaBasicSts_t sts;
	u8 cmd[] = { BASIC_APPID, CMD_RD_STS, 0, sizeof(sts)};
	ret = mmaSendCommand(client, cmd, sizeof(cmd), (u8 *)&sts, sizeof(sts));
	#ifdef DEBUG
	printk("\ngpioState = %d", sts.gpioState);
	printk("\ndataCnt = %d", sts.dataCnt);
	printk("\nZlockCnt = %d", sts.ZlockCnt);
	printk("\nPLmixCnt = %d", sts.PLmixCnt);
	printk("\neventCnt = %d", sts.eventCnt);
	printk("\ndummy = %d", sts.dummy);
	printk("\n>acc_X = %d", (short)SWAP16(sts.acc_X));
	printk("\n>acc_Y = %d", (short)SWAP16(sts.acc_Y));
	printk("\n>acc_Z = %d", (short)SWAP16(sts.acc_Z));
	printk("\nfacc_X = %d", (short)SWAP16(sts.facc_X));
	printk("\nfacc_Y = %d", (short)SWAP16(sts.facc_Y));
	printk("\nfacc_Z = %d", (short)SWAP16(sts.facc_Z));
	printk("\n");
	#endif
	return ret;
}

/*
* This method reads basic motion config data
* return 0 : For successful */
static int basic_getCfg(struct i2c_client *client)
{
	int ret;
	mmaBasicCfg_t cfg;
	u8 cmd[4] = { BASIC_APPID, CMD_RD_CFG, 0, sizeof(cfg)};
	ret = mmaSendCommand(client, cmd, sizeof(cmd), (u8 *)&cfg, sizeof(cfg));
	#ifdef DEBUG
	printk("\neventCnt = %x", cfg.eventCnt);
	printk("\ncfg = %x", cfg.cfg);
	printk("\n");
	#endif
	return ret;
}
#endif

/*
* This method reads hub output data
* return 0 : For successful */
int hub_getSts(struct i2c_client *client, mmaHubSts_t* p)
{
	int ret;
	short P,T;
	struct mxc_mma_device_t *pDev = i2c_get_clientdata(client);
	u8 cmd[4] = { HUB_APPID, CMD_RD_STS, 0, sizeof(mmaHubSts_t)};
	ret = mmaSendCommand(client, cmd, sizeof(cmd), (u8 *)p, sizeof(mmaHubSts_t));
//	#ifdef DEBUG
	printk("\ngyroAddr = 0x%02x", p->gyroAddr);
	printk("\nambAddr = 0x%02x", p->ambAddr);
	printk("\ngyro_Ox = %d", (short)SWAP16(p->gyro_Ox));
	printk("\ngyro_Oy = %d", (short)SWAP16(p->gyro_Oy));
	printk("\ngyro_Oz = %d", (short)SWAP16(p->gyro_Oz));
	/*
	Pressure data is 20-bit unsigned value in .25Pa units (the data is stored as Pa with bits 19-2 and with fractions of a Pa in bits 1-0). Range is -131072 to 131071.75 Pa
	Altitude data is 20-bit 2’s compl. value in .0625m units (the data is stored as meters with bits 19-4 and with fractions of a meter in bits 3-0). Range is -32768 to 32767.9375 m
	Temperature data is 12-bit 2’s compl. value in .0625°C units. The data is stored as degrees °C with bits 11-4 and with fractions of a degree °C in bits 3-0. Range is -128C to 127.9375 °C
	*/
	P=(short)SWAP32(p->amb_P);
	T=(short)SWAP16(p->amb_T);

	if(pDev->pChip->amb_p_mode == HUB_AMB_BAROMETER_MODE)
		printk("\namb_P = %d.%d Pa", (short)(P >> 2), (P & 0x0003)*FLOAT_TO_FIXP(0.25));
	else
		printk("\namb_P = %d.%d m", (short)(P >> 4), (P & 0x000F)*FLOAT_TO_FIXP(0.0625));
	printk("\namb_T = %d.%d C", (short)(T >> 4), (T & 0x000F)*FLOAT_TO_FIXP(0.0625));

	printk("\nacc_mod = %d", (unsigned short)SWAP16(p->acc_mod));
	printk("\n");
//	#endif
	return ret;
}

/*
* This method reads hub config data
* return 0 : For successful */
int hub_getCfg(struct i2c_client *client, mmaHubCfg_t* p)
{
	int ret;
	u8 cmd[4] = { HUB_APPID, CMD_RD_CFG, 0x00, sizeof(mmaHubCfg_t) };
	ret = mmaSendCommand(client, cmd, sizeof(cmd), (u8 *)p, sizeof(mmaHubCfg_t));
//	#ifdef DEBUG
	//printk("\ngyroFlag = 0x%02x", p->gyroFlag);
	printk("\ngyroCfg = 0x%02x", p->gyroCfg);
	//printk("\nambFlag = 0x%02x", p->ambFlag);
	printk("\nambCfg = 0x%02x", p->ambCfg);
	printk("\nambP_ofs = 0x%02x", p->ambP_ofs);	/* resolution: 4 Pa/LSB */
	printk("\nambH_ofs = 0x%02x", p->ambH_ofs);	/* resolution: 1 m/LSB */
	printk("\nambT_ofs = 0x%02x", p->ambT_ofs);	/* resolution: 0.0625 °C/LSB */
	printk("\n");
//	#endif
	return ret;
}

/*
* This method reads pedometer output data
* return 0 : For successful */
int ped_getSts(struct i2c_client *client, mmaPedSts_t *p)
{
	int ret;
	u8 cmd[4] = { PEDOM_APPID, CMD_RD_STS, 0x00, sizeof(mmaPedSts_t)};
	ret = mmaSendCommand(client, cmd, sizeof(cmd), (u8 *)p, sizeof(mmaPedSts_t));
	#ifdef DEBUG
	printk("\nstatus = 0x%04x", (u16)SWAP16(p->status));
	printk("\nstepCnt = 0x%04x", (u16)SWAP16(p->stepCnt));
	printk("\ndistance = 0x%04x", (u16)SWAP16(p->distance));
	printk("\nspeed = 0x%04x", (u16)SWAP16(p->speed));
	printk("\ncalories = 0x%04x", (u16)SWAP16(p->calories));
	printk("\nsleepCnt = 0x%04x", (u16)SWAP16(p->sleepCnt));
	printk("\n");
	#endif
	return ret;
}

/*
* This method reads pedometer config data
* return 0 : For successful */
int ped_getCfg(struct i2c_client *client, mmaPedCfg_t *p)
{
	int ret;
	u8 cmd[4] = { PEDOM_APPID, CMD_RD_CFG, 0x00, sizeof(mmaPedCfg_t) };
	ret = mmaSendCommand(client, cmd, sizeof(cmd), (u8 *)p, sizeof(mmaPedCfg_t));
//	#ifdef DEBUG
	printk("\nsleepMin = 0x%04x", (u16)SWAP16(p->sleepMin));
	printk("\nsleepMax = 0x%04x", (u16)SWAP16(p->sleepMax));
	printk("\nsleepThd = 0x%04x", (u16)SWAP16(p->sleepThd));
	printk("\nconfigStepLen = 0x%04x", (u16)SWAP16(p->configStepLen));
	printk("\nheightWeight = 0x%04x", (u16)SWAP16(p->heightWeight));
	printk("\nfilter = 0x%04x", (u16)SWAP16(p->filter));
	printk("\nspeedPeriod = 0x%04x", (u16)SWAP16(p->speedPeriod));
	printk("\nactThd = 0x%04x", (u16)SWAP16(p->actThd));
	printk("\n");
//	#endif
	return ret;
}

/*
* This method is used to rotate read accel. data vector depeding on device orientation on PCB
* data : ptr to data vector
* return 0 : For successful reading */
static int hub_acc_convert(struct mxc_mma_device_t* pDev, vec_t *v)
{
	short raw[3], tmp[3];
	int i,j;
	int position = pDev->pChip->position;

	if(position < 0 || position > 7 )
		position = 0;

	raw[0] = v->x;
	raw[1] = v->y;
	raw[2] = v->z;

	/* Gp vector rotation to compensate for device orientation relative to display graphics */
	for(i = 0; i < 3 ; i++)
	{
		tmp[i] = 0;
		for(j = 0; j < 3; j++)
		tmp[i] += raw[j] * mma_position_setting[position][i][j];
	}

	/* At rest, the mma955x's AFE gives Gpz = -1g when z is downward. Some applications may require the opposite */
	if((pDev->pChip->acc_invert) && (pDev->pChip->acc_output_stage != AFE_STAGE_0_ABS))
	{
		v->x = -tmp[0];
		v->y = -tmp[1];
		v->z = -tmp[2];
	}
	else
	{
		v->x = tmp[0];
		v->y = tmp[1];
		v->z = tmp[2];
	}

	return 0;
}

/*
* This method is used to rotate gyroscope data vector depeding on device orientation on PCB
* data : ptr to data vector
* return 0 : For successful reading */
static int hub_gyro_convert(struct mxc_mma_device_t* pDev, vec_t *v)
{
	short raw[3], tmp[3];
	int i,j;
	int position = pDev->pChip->position;

	if(position < 0 || position > 7 )
		position = 0;

	raw[0] = v->x;
	raw[1] = v->y;
	raw[2] = v->z;

	/* Op vector rotation to compensate for device orientation relative to display graphics */
	for(i = 0; i < 3 ; i++)
	{
		tmp[i] = 0;
		for(j = 0; j < 3; j++)
		tmp[i] += raw[j] * mma_position_setting[position][i][j];
	}

	/* The L3GD20 gyroscope gives Opz < 0 when device is rotated clockwise around z axis. Some applications may require the opposite */
	if(pDev->pChip->gyro_invert)
	{
		v->x = -tmp[0];
		v->y = -tmp[1];
		v->z = -tmp[2];
	}
	else
	{
		v->x = tmp[0];
		v->y = tmp[1];
		v->z = tmp[2];
	}

	return 0;
}

static int mma_device_stop(struct i2c_client *client)
{
	int ret;
	struct mxc_mma_device_t *pDev = i2c_get_clientdata(client);
	u8 cmd[5] = { SLEEP_WAKE_APPID, CMD_WR_CFG, 0x06, 0x01, 0x01 };
	ret = mmaSendCommand(client, cmd, sizeof(cmd), NULL, 0);
	if(ret < 0)
		goto out;
	pDev->pChip->activity = MMA_SLEEP;
	ret = 0;
out:
	return ret;
}

/*
* This method is used to read hub data from mma9550 device
* data : ptr to data vector
* return 0 : For successful reading */
static int hub_read(struct i2c_client *client, void *pData)
{
	int ret;
	HubData_t data;
	struct mxc_mma_device_t *pDev = i2c_get_clientdata(client);
	vec_t vec;
	u8 tmp[6];

	u8 gyro_cmd[4] = { HUB_APPID, CMD_RD_STS, offsetof(mmaHubSts_t, gyro_Ox), sizeof(vec_t)};
	u8 amb_cmd[4] = { HUB_APPID, CMD_RD_STS, offsetof(mmaHubSts_t, amb_P), sizeof(vec_t)};

	mutex_lock(&pDev->data_lock);

	/* get accelerometer data from desired AFE stage */
	ret = afe_getStage(client, pDev->pChip->acc_output_stage, &vec);
	if (ret < sizeof(vec_t)) {
		dev_err(&client->dev, "i2c block read failed\n");
		ret = -EIO;
		goto out;
	}

	data.G.x = (short)SWAP16(vec.x);
	data.G.y = (short)SWAP16(vec.y);
	data.G.z = (short)SWAP16(vec.z);

	/* get angular rate data from HUB application status */
	ret = mmaSendCommand(client, gyro_cmd, sizeof(gyro_cmd), (u8 *)&vec, sizeof(vec_t));
	if (ret < sizeof(vec_t)) {
		dev_err(&client->dev, "i2c block read failed\n");
		ret = -EIO;
		goto out;
	}

	data.O.x = (short)SWAP16(vec.x);
	data.O.y = (short)SWAP16(vec.y);
	data.O.z = (short)SWAP16(vec.z);

	/* get pressure & temperature data from HUB application status */
	ret = mmaSendCommand(client, amb_cmd, sizeof(amb_cmd), tmp, sizeof(tmp));
	if (ret < sizeof(tmp)) {
		dev_err(&client->dev, "i2c block read failed\n");
		ret = -EIO;
		goto out;
	}

	data.P = ((tmp[0] << 24) & 0xff000000) | ((tmp[1] << 16) & 0x00ff0000) | ((tmp[2] << 8) & 0x0000ff00) | tmp[3];
	data.T = ((tmp[4] << 8) & 0xff00) | tmp[5];

	mutex_unlock(&pDev->data_lock);

	/* post processing */
	hub_acc_convert(pDev, &data.G);
	hub_gyro_convert(pDev, &data.O);

	memcpy((u8 *)pData, (u8 *)&data, sizeof(data));

	ret = 0;
out:
	return ret;
}

/*
* This method is used to read pedometer data from mma9553 device
* client :
* data : ptr to data vector
* return 0 : For successful reading
*/
static int ped_read(struct i2c_client *client, void *pData)
{
	PedData_t data;
	mmaPedSts_t sts;
	int ret;
	struct mxc_mma_device_t *pDev = i2c_get_clientdata(client);
	vec_t vec;

	mutex_lock(&pDev->data_lock);

	ret = ped_getSts(client, &sts);

	data.distance = (short)SWAP16(sts.distance);
	data.speed = (short)SWAP16(sts.speed);
	data.sleepCnt = (short)SWAP16(sts.sleepCnt);
	data.status = (short)SWAP16(sts.status);
	data.stepCnt = (short)SWAP16(sts.stepCnt);

	/* get accelerometer data from desired AFE stage */
	ret = afe_getStage(client, pDev->pChip->acc_output_stage, &vec);
	if (ret < sizeof(vec_t)) {
		dev_err(&client->dev, "i2c block read failed\n");
		ret = -EIO;
		goto out;
	}

	data.G.x = (short)SWAP16(vec.x);
	data.G.y = (short)SWAP16(vec.y);
	data.G.z = (short)SWAP16(vec.z);

	/* post processing */
	hub_acc_convert(pDev, &data.G);

	memcpy((u8 *)pData, (u8 *)&data, sizeof(data));

	ret = 0;
out:
	mutex_unlock(&pDev->data_lock);
	return ret;
}


#if 0 /* WARNING: this part of code must be revisited to allow multi-device operations */
/*
* Interrupt handler for mma955x chip
* irq : IRQ number.
* dev_id : Pointer to dev_id structure.
* return 1
*/
static irqreturn_t mma_interrupt(int irq, void *dev_id)
{
	if(plat_data->int1 == irq)
	{
		//disable_irq_nosync(irq);
		up(&chip_ready);
	}
	return IRQ_RETVAL(1);
}

/*
* Workaround timer routine.
* data.
* return None
*/
static void stall_timer_fn(unsigned long data)
{
	up(&chip_ready);
}
#endif

/*
* Interrupt service thread implementation.
* data : Pointer which is used to get device/chip data .
* return 0 	: After servicing thread.
*/
static int mmaIsrThread(void *data)
{
	wait_queue_t wait;
	char buff[1024];

	struct mxc_mma_device_t *pDev = (struct mxc_mma_device_t *)data;
	struct chipInfo_t *pChip = pDev->pChip;

	init_waitqueue_entry(&wait, current);

	while(1)
	{
		if(pChip->activity != MMA_SLEEP)
		{
			pChip->Read(pChip->client, (void *)buff);
			mmaReportEvent(pDev, buff);
		}
		msleep(20);
	}
	return 0;
}

/*
* This method is used to init the device
* It is assumed the device's flash CI is running.
* return 0 : For successful init
*/
static int mmaInit(struct chipInfo_t *pChip)
{
	int ret;
	struct i2c_client *client = NULL;
	struct mxc_mma_device_t *pDev = NULL;

	if(pChip == NULL)
	{
		printk("\n%s:: NULL pointer", __func__);
		return -ENOMEM;
	}

	client = pChip->client;
	pDev = i2c_get_clientdata(client);

	mutex_lock(&pDev->data_lock);

	/* The device will be running from ROM or running from flash firmware.
	   This commad yields a hw reset and a boot-to-flash */
	if((ret = mmaReset(client)) < 0)
		goto out;

	switch(pChip->chipId)
	{
		case MMA9550_ID:
		{
			u8 cmd_afe[5] = { AFE_APPID, CMD_WR_CFG, 0x00, 0x01, 0x80 };	/* set Full-Scale Data Range +-4g */
			u8 cmd_afe1[9] = { AFE_APPID, CMD_WR_CFG, 0x08, 0x05, 0x03, 0x03, 0x00, 0x00, 0x07 };
			u8 cmd_sched[5] = { SCHED_APPID, CMD_WR_CFG, 0x32, 0x01, 0xd7 };
			//u8 cmd_sched_mi2c[] = { SCHED_APPID, CMD_WR_CFG, 0x28, 0x04, 0x04, 0x00, 0x00, 0x00 };
			u8 cmd_wake[5] = { SLEEP_WAKE_APPID, CMD_WR_CFG, 0x06, 0x01, 0x00 };
			//u8 cmd_pl[14] = { PL_APPID, CMD_WR_CFG, 0x00, 0x07, 0x20, 0x00, 0x07, 0x37, 0x14, 0x41, 0x09, 0x00, 0xAF, 0x60 };

			if((ret = mmaSendCommand(client, cmd_afe, sizeof(cmd_afe), NULL, 0)) < 0)
				goto out;

			if((ret = mmaSendCommand(client, cmd_afe1, sizeof(cmd_afe1), NULL, 0)) < 0)
				goto out;

			if((ret = mmaSendCommand(client, cmd_sched, sizeof(cmd_sched), NULL, 0)) < 0)
				goto out;

			/* The device will be waked up */
			if((ret = mmaSendCommand(client, cmd_wake, sizeof(cmd_wake), NULL, 0)) < 0)
				goto out;

			msleep(MODE_CHANGE_DELAY_MS);
			pChip->activity = MMA_RUN;
		}
		break;
		case MMA9553_ID:
		{
			u8 cmd_afe[5] = { AFE_APPID, CMD_WR_CFG, 0x00, 0x01, 0x80 };	/* set Full-Scale Data Range +-4g */
			u8 cmd_wake[5] = { SLEEP_WAKE_APPID, CMD_WR_CFG, 0x06, 0x01, 0x00 };
			u8 ped_cfg[20] = { PEDOM_APPID, CMD_WR_CFG, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x80, 0x00, 0xaf, 0x50, 0x04, 0x03, 0x05, 0x01, 0x00, 0x00 };

			if((ret = mmaSendCommand(client, cmd_afe, sizeof(cmd_afe), NULL, 0)) < 0)
				goto out;

			/* The device will be waked up */
			if((ret = mmaSendCommand(client, cmd_wake, sizeof(cmd_wake), NULL, 0)) < 0)
				goto out;

			msleep(MODE_CHANGE_DELAY_MS);
			pChip->activity = MMA_RUN;

			/* set initial config values */
			if((ret = pChip->SendCommand(client, ped_cfg, sizeof(ped_cfg), NULL, 0)) < 0)
				goto out;
		}
		break;
		default:
		{
			printk("Invalid chip id\n");
			ret = -EINVAL;
			goto out;
		}

	}
	ret = 0;
out:
	mutex_unlock(&pDev->data_lock);
	return ret;
}

static int mmaSetDefaultVal(struct chipInfo_t *pChip)
{
	if(pChip == NULL)
	{
		printk("%s: NULL pointer\r\n", __func__);
		return -ENOMEM;
	}

	switch(pChip->chipId)
	{
		case MMA9550_ID:
		{
/* todo...... */
			pChip->odr = 1;
			pChip->position = CONFIG_SENSORS_MMA_POSITION;
			pChip->resolution = 0; 		// 16-bit
			pChip->acc_invert = 0;
			pChip->acc_range = ACC_RANGE_4G;
			pChip->gyro_invert = 0;
			pChip->gyro_range = HUB_GYR_RANGE_500DPS;
			pChip->acc_output_stage = AFE_STAGE_0;
			pChip->acc_ofs_x = 0;
			pChip->acc_ofs_y = 0;
			pChip->acc_ofs_z = 0;
			pChip->gyro_ofs_x = 0;
			pChip->gyro_ofs_y = 0;
			pChip->gyro_ofs_z = 0;
			pChip->amb_ofs_P = 0;
			pChip->amb_ofs_H = 0;
			pChip->amb_ofs_T = 0;
		}
		break;
		case MMA9553_ID:
		{
			pChip->odr = 4;
			pChip->sleep_min = 0x0000;
			pChip->sleep_max = 0x0000;
			pChip->sleep_thd = 0x0001;
			pChip->steplen = 0x00;
			pChip->height = 0xaf;		/* height: 175 cm */
			pChip->weight = 0x50;		/* weight: 80 kg */
			pChip->filter_step = 0x04;
			pChip->speed_period = 0x05;
			pChip->step_coalesce = 0x01;
			pChip->activity_thd = 0x0000;
		}
		break;
		default:
		{
			printk("Invalid chip id\n");
			return -EINVAL;
		}
	}
	return 0;
}

static int mmaIdentifyChip(struct chipInfo_t *pChip)
{
	mmaVer_t ver;
	struct i2c_client *client = NULL;
	struct mxc_mma_device_t *pDev = NULL;

	if(pChip == NULL)
	{
		printk("\n%s:: NULL pointer", __func__);
		return -ENOMEM;
	}

	client = pChip->client;
	pDev = i2c_get_clientdata(client);

	memset((u8 *)&ver, 0, sizeof(ver));
	ver_getVersion(client, &ver);

	/* WORKAROUND: we use a weak condition for chip identification */
	if ((ver.rom_major != 1) || (ver.rom_minor != 1)) {
		printk("\n%s:: Not a valid MMA955%d chipset found", __func__, pChip->chipId);
		return -EIO;
	}

	/* print chip versions */
	printk("\n%s:: Found MMA955%d chipset with internal unique ID 0x%08x", __func__, pChip->chipId, (int)SWAP32(ver.id)); /* ex: 0x1cda3155 */
	printk("\nROM ver = %x.%x", ver.rom_major, ver.rom_minor);	/* ex: 01.01 */
	printk("\nFactory FW ver = %x.%x", ver.fw_major, ver.fw_minor);		/* ex: 02.02 */
	printk("\nHW ver = %x.%x", ver.hw_major, ver.hw_minor);		/* ex: 01.06 */
	printk("\nFactory FW build = %x.%x", ver.build_major, ver.build_minor);	/* ex: 0x0341 */

	return 0;
}


/*
* Probe function for mma955x device.
* client 	: Pointer to i2c client.
* id 		: Pointer to i2c device id.
* return  -EPERM 	: If device pointer is NULL
* return  -ENOMEM 	: If memory allocation to the device fails
* return  0 		: If device is successfully registered.
*/
static int __devinit mma955x_probe(struct i2c_client *client, const struct i2c_device_id *id)
{

	int ret = 0, i = 0;
	//struct input_dev *inp;
	struct chipInfo_t *pChip = NULL;
	struct mxc_mma_device_t *pDev = NULL;
	struct i2c_adapter *adapter;
	//struct input_polled_dev *poll_dev;
	adapter = to_i2c_adapter(client->dev.parent);

	ret = i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE | I2C_FUNC_I2C);
	if (!ret)
		goto err_out;

	plat_data = (struct mma955x_platform_data *)client->dev.platform_data;
	if (plat_data == NULL) {
		ret = -ENODEV;
		dev_err(&client->dev, "lack of platform data!\n");
		goto err_out;
	}

	pDev = kzalloc(sizeof(struct mxc_mma_device_t), GFP_KERNEL);
	if(!pDev){
		ret = -ENOMEM;
		dev_err(&client->dev, "alloc data memory error!\n");
		goto err_out;
	}

	printk(KERN_INFO"\r\nProbing Module: %s %s\r\n", MODULE_NAME, DRIVER_VERSION);
	printk(KERN_INFO "Build Date: %s [%s]\r\n\r\n", __DATE__, __TIME__);

	//sema_init(&temp, 0);
	i2c_set_clientdata(client, pDev);

	/* bind the right device to the driver */
	/* Warning: there is no obvious way to identify a specific device Id by hw query, so we pass id via platform data */
	if(plat_data->chipId < 0)
	{
		ret = -EIO;
		dev_err(&client->dev, "Invalid platform data!\n");
		goto err_alloc_poll_device;
	}

	/* Associate chip layer */
	for(i = 0; i < sizeof(chipTable)/sizeof(chipTable[0]); i++)
	{
		if(chipTable[i]->chipId == plat_data->chipId)
		{
			pChip = chipTable[i];
			pChip->client = client;
			pChip->chipAddr = plat_data->chipAddr; //not used
			break;
		}
	}

	if(i >= (sizeof(chipTable)/sizeof(chipTable[0])))
	{
		ret = -ENOMEM;
		dev_err(&client->dev, "Chipset not supported by MMA driver!\n");
		goto err_alloc_poll_device;
	}

	mmaSetDefaultVal(pChip);

	/* @todo : event type*/
	pDev->data_event_type = 0x25;

	pDev->pChip = pChip;
	pDev->version = 1;

	strcpy(pDev->devname, "mma");

	mutex_init(&pDev->data_lock);

	/* Initialize chipset */
	pChip->Init(pChip);

	if((ret = mmaIdentifyChip(pChip)) < 0)
		goto err_alloc_poll_device;

	#if 1//def DEBUG
	if(pDev->pChip->chipId == MMA9553_ID)
	{
		mmaPedSts_t sts;
		mmaPedCfg_t cfg;
		vec_t vec;
		mmaAfeSts_t sts1;

		ped_getCfg(client, &cfg);
		ped_getSts(client, &sts);
		afe_getSts(client, &sts1);
		afe_getStage(client, AFE_STAGE_1, &vec);
		afe_getStage(client, AFE_STAGE_0_ABS, &vec);
		afe_getStage(client, AFE_STAGE_0_GM, &vec);
		afe_getStage(client, AFE_STAGE_0_LPF, &vec);
		afe_getStage(client, AFE_STAGE_0_HPF, &vec);
	}

	if(pDev->pChip->chipId == MMA9550_ID)
	{
		mmaHubSts_t sts;
		mmaHubCfg_t cfg;
		vec_t vec;
		mmaAfeSts_t sts1;
		hub_getCfg(client, &cfg);
		hub_getSts(client, &sts);
		//sched_reqStart(client, (unsigned int)(1 << HUB_APPID));
		afe_getSts(client, &sts1);
		afe_getStage(client, AFE_STAGE_0, &vec);
		//mma9550_qread(client);
	}
	#endif


#if 1
	/* Register character device */
	pDev->major = register_chrdev(0, pDev->pChip->name, &mma_ops);
	if(ret < 0)
	{
		printk(KERN_INFO "%s:: Unable to register device\r\n", __func__);
		goto error_request_irq;
	}
#endif

	#if 0 /* WARNING: this is a different implementation that uses a polled_device instead of the thread */
	poll_dev = input_allocate_polled_device();
	if (!poll_dev) {
		ret = -ENOMEM;
		dev_err(&client->dev, "alloc poll device failed!\n");
		goto err_alloc_poll_device;
	}

	poll_dev->poll = mma955x_dev_poll;
	poll_dev->poll_interval = POLL_STOP_TIME;
	poll_dev->poll_interval_min = POLL_INTERVAL_MIN;
	poll_dev->poll_interval_max = POLL_INTERVAL_MAX;
	poll_dev->private = pDev;

	inp = poll_dev->input;
	inp->name = MODULE_NAME;
	inp->uniq = mma955x_id2name(plat_data->chipId);
	inp->id.bustype = BUS_I2C;
	inp->id.vendor = 0x1211;
	inp->id.version = 1;

	inp->evbit[0] = BIT_MASK(EV_ABS);

	input_set_abs_params(inp, ABS_X, -32768, 32767, FUZZ, FLAT);	/* x-axis acceleration Gx */
	input_set_abs_params(inp, ABS_Y, -32768, 32767, FUZZ, FLAT);	/* y-axis acceleration Gy */
	input_set_abs_params(inp, ABS_Z, -32768, 32767, FUZZ, FLAT);	/* z-axis acceleration Gz */

	input_set_abs_params(inp, ABS_RX, -32768, 32767, FUZZ, FLAT);	/* x-axis angular rate Ox */
	input_set_abs_params(inp, ABS_RY, -32768, 32767, FUZZ, FLAT);	/* y-axis angular rate Oy */
	input_set_abs_params(inp, ABS_RZ, -32768, 32767, FUZZ, FLAT);	/* z-axis angular rate Oz */

	/* Either pressure in .25Pa units or Altitude in .0625m units.
	   Pressure range is -131072 to 131071.75 Pa
	   Altitude range is -32768 to 32767.9375 m */
	input_set_abs_params(inp, ABS_PR, -524288, 524287, FUZZ, FLAT);

	/* Temperature in .25°C units.
	   Temperature range is -128C to 127.9375 °C */
	input_set_abs_params(inp, ABS_TEMP, -2048, 2047, FUZZ, FLAT);

	pDev->poll_dev = poll_dev;
	ret = input_register_polled_device(poll_dev);
	if (ret) {
		dev_err(&client->dev, "register poll device failed!\n");
		goto err_register_polled_device;
	}
	#endif

	/* Initialize input layer */
	ret = mmaInitializeInputInterface(pDev);
	if (ret) {
		dev_err(&client->dev, "create input interface failed!\n");
		ret = -EINVAL;
		goto err_create_input;
	}

	/* Create sysfs entries */
	ret = mmaInitializeSysfs(client);
	if (ret) {
		dev_err(&client->dev, "create sysfs entries failed!\n");
		ret = -EINVAL;
		goto err_create_sysfs;
	}

	/* Initialize chip ready semaphore */
	#if 0 /* WARNING: this part of code must be revisited to allow multi-device operations */
	sema_init(&chip_ready, 0);
	setup_timer(&stall_timer, stall_timer_fn, 0);
	#endif

	/* Start Interrupt Service Thread */
	pDev->hIsrThread = kthread_run(mmaIsrThread, pDev, "mma955x_ist");
	if (IS_ERR(pDev->hIsrThread))
	{
		 printk(KERN_INFO "Error creating mma955x_ist.\n");
		goto error_create_thread;
	}

	#if 0 /* WARNING: this part of code must be revisited to allow multi-device operations */
	if (plat_data->int1 > 0)
	{
		/* interrupt base */
		/* when to register interrupt is to be considered later */
		printk(KERN_INFO "Configuring IRQ ==> %d\r\n", plat_data->int1);
		set_irq_type(plat_data->int1, IRQF_TRIGGER_RISING);
		/* register interrupt handle */
		ret = request_irq(plat_data->int1, mma_interrupt, IRQF_TRIGGER_RISING, DEVICE_NAME, pDev);

		if (ret < 0) {
			dev_err(&client->dev, "request_irq(%d) returned error %d\n", plat_data->int1, ret);
			goto error_request_irq;
		}
	}
	else
	{
		/* poll base */
		poll_mode = 1; //WaARNING: cannot be a static or global variable !
	}
	#endif

	dev_info(&client->dev, "mma955%d device is probed successfully.\r\n", pDev->pChip->chipId);

	return 0;

error_request_irq:
	kthread_stop(pDev->hIsrThread);
error_create_thread:
	mmaDeInitializeSysfs(pDev);
err_create_sysfs:
	mmaDeInitializeInputInterface(pDev);
err_create_input:
	//input_unregister_polled_device(poll_dev);
//err_register_polled_device:
	//input_free_polled_device(poll_dev);
err_alloc_poll_device:
	kfree(pDev);
err_out:
	return ret;
}

static int __devexit mma955x_remove(struct i2c_client *client)
{
	int ret;
	struct mxc_mma_device_t *pDev = i2c_get_clientdata(client);
	ret = mma_device_stop(client);
	if(pDev) {
		/* Remove sysfs entries */
		printk(KERN_INFO "%s:: Remove sysfs...\n", __func__);
		mmaDeInitializeSysfs(pDev);

		/* unegister character device */
		printk(KERN_INFO "%s:: Unregister char interface\n", __func__);
		unregister_chrdev(pDev->major, pDev->pChip->name);

		/* DeInitialize input layer */
		printk(KERN_INFO "%s:: Unregister input interface...\n", __func__);
		mmaDeInitializeInputInterface(pDev);

		/* Stop IST */
		printk(KERN_INFO "%s:: Stopping thread...\n", __func__);
		//up(&chip_ready);
		kthread_stop(pDev->hIsrThread);

		//input_unregister_polled_device(poll_dev);
		//input_free_polled_device(poll_dev);
		kfree(pDev);
		pDev = NULL;
	}
	return ret;
}

#ifdef CONFIG_PM_SLEEP
static int mma955x_suspend(struct device *dev)
{
	int ret = 0;
	struct i2c_client *client = to_i2c_client(dev);
	struct mxc_mma_device_t *pDev = i2c_get_clientdata(client);
	if(pDev->pChip->activity == MMA_RUN)
		ret = mma_device_stop(client);
	return ret;
}

static int mma955x_resume(struct device *dev)
{
	int ret;
	struct i2c_client *client = to_i2c_client(dev);
	struct mxc_mma_device_t *pDev = i2c_get_clientdata(client);
	u8 cmd[5] = { SLEEP_WAKE_APPID, CMD_WR_CFG, 0x06, 0x01, 0x00 };
	if(pDev->pChip->activity == MMA_SLEEP)
	{
		ret = mmaSendCommand(client, cmd, sizeof(cmd), NULL, 0);
		if(ret < 0)
			goto out;
		pDev->pChip->activity = MMA_RUN;
	}
	ret = 0;
out:
	return ret;
}
#endif

static const struct i2c_device_id mma955x_id[] = {
	{"mma955x", 0},
	{ }
};

MODULE_DEVICE_TABLE(i2c, mma955x_id);

static SIMPLE_DEV_PM_OPS(mma955x_pm_ops, mma955x_suspend, mma955x_resume);
static struct i2c_driver mma955x_driver = {
	.driver = {
		   .name = MODULE_NAME,
		   .owner = THIS_MODULE,
		   .pm = &mma955x_pm_ops,
		   },
	.probe = mma955x_probe,
	.remove = __devexit_p(mma955x_remove),
	.id_table = mma955x_id,
};

static int __init mma955x_init(void)
{
	/* register driver */
	int res;

	res = i2c_add_driver(&mma955x_driver);
	if (res < 0) {
		printk(KERN_INFO "add mma955x i2c driver failed\n");
		return -ENODEV;
	}
	return res;
}

static void __exit mma955x_exit(void)
{
	i2c_del_driver(&mma955x_driver);
}

MODULE_AUTHOR("Cosmed, Ltd.");
MODULE_DESCRIPTION("MMA955X 3-Axis Intelligent Motion-Sensing Platform driver");
MODULE_LICENSE("GPL");

module_init(mma955x_init);
module_exit(mma955x_exit);
