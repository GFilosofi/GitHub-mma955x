/******************** (C) COPYRIGHT 2012 Cosmed, Ltd. *************
* Copyright (C) 2012 Cosmed, Ltd. <http://www.cosmed.com/>
*
* File Name		: mma_sysfs.c
* Authors		: Gabriele Filosofi <gabrielef@cosmed.it>
			  Gabriele is willing to be considered the contact and update points
			  for the driver
* Version		: V.1.0.0
* Date			: 2012/Sept/20
* Description		: sysfs implementation for Freescale 3-Axis Intelligent
* Motion-Sensing Platform MMA9550/MMA9551/MMA9553/MMA9559.
 ******************************************************************************
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
#include <asm/uaccess.h>
#include <linux/input.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include "mma955x_regs.h"
#include <mach/hardware.h>
#include "mma955x_io.h"


/******************************************************************************
*			MMA955x-common Sysfs info
*******************************************************************************/
/*
* This method is used to get name of chip 
* dev : Pointer to device structure
* attr : Pointer to device attributes
* buf : Pointer to buffer in which name of chip will be copied
* return : Length of string
* filesys item: /sys/class/mma955x/mma955x/name
*/
ssize_t mma_name_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t ret;
	struct mxc_mma_device_t *pDev = dev_get_drvdata(dev);
	if(pDev)
		sprintf(buf, "%s\n", pDev->pChip->name);
	ret = strlen(buf) + 1;
	return ret;
}

/*
* This method is used to get name of vendor.
* filesys item: /sys/class/mma955x/mma955x/vendor
*/
ssize_t mma_vendor_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t ret;
	sprintf(buf, "%s\n", VENDOR_NAME);
	ret = strlen(buf) + 1;
	return ret;
}

/*
* This method is used to get the "x" identifier of the MMA955x chip.
* filesys item: /sys/class/mma955x/mma955x/device_id
*/
ssize_t mma_devid_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t ret;
	struct mxc_mma_device_t *pDev = dev_get_drvdata(dev);
	if(pDev)
		sprintf(buf, "%d\n", pDev->pChip->chipId);
		//sprintf(buf, "0x%x\n", pDev->pChip->chipAddr);
	ret = strlen(buf) + 1;
	return ret;
}

/*
* This method is used to get version of chip driver.
* filesys item: /sys/class/mma955x/mma955x/version
*/
ssize_t mma_version_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;
	struct mxc_mma_device_t *pDev = dev_get_drvdata(dev);
	if(pDev)
		sprintf(buf, "%d\n", pDev->version);
	ret = strlen(buf) + 1;
	return ret;
}

/*
* This method is used to get the sleep activity state (0: no; 1: yes).
* filesys item: /sys/class/mma955x/mma955x/sleep
*/
ssize_t mma_sleep_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t ret;
	struct mxc_mma_device_t *pDev = dev_get_drvdata(dev);
	struct chipInfo_t *pChip = pDev->pChip;
	struct i2c_client *client = pChip->client;
	u8 cmd[4] = { SLEEP_WAKE_APPID, CMD_RD_CFG, 0x06, 0x01 };
	u8 val;
	int sleep;

	mutex_lock(&pDev->data_lock);
	ret = pChip->SendCommand(client, cmd, sizeof(cmd), &val, 1);
	if(ret < 1)
		goto out;
	if((val & SLEEPWAKE_CFG_SNCEN_MASK) && (pChip->activity == MMA_SLEEP))
		sleep = 1;
	else
		sleep = 0;
	ret = sprintf(buf, "%d\n", sleep);
out:
	mutex_unlock(&pDev->data_lock);
	return ret;
}

/*
* This method is used to set the sleep activity state. 
* This options let the device to work at a lower power consumption rate.
* dev : Pointer to device structure
* attr : Pointer to device attributes
* buf : ptr to data buffer
* count : Length of string
* Accepted values : 0, 1
* return : Length of string for valid attribute value.
* return -EINVAL : Invalid attribute value.
* filesys item: /sys/class/mma955x/mma955x/sleep
*/
ssize_t mma_sleep_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	ssize_t ret;
	struct mxc_mma_device_t *pDev = dev_get_drvdata(dev);
	struct chipInfo_t *pChip = pDev->pChip;
	struct i2c_client *client = pChip->client;

	u8 cmd_req[4] = { SLEEP_WAKE_APPID, CMD_RD_CFG, 0x06, 0x01 };
	u8 cmd_set[5] = { SLEEP_WAKE_APPID, CMD_WR_CFG, 0x06, 0x01, 0 };
	u8 val = 0;

	unsigned long sleep = simple_strtoul(buf, NULL, 10);

	sleep = (sleep > 0) ? 1 : 0;

	mutex_lock(&pDev->data_lock);
	ret = pChip->SendCommand(client, cmd_req, sizeof(cmd_req), &val, 1);
	if(ret < 1)
		goto out;

	if(!sleep && pChip->activity == MMA_SLEEP)
	{
		cmd_set[4] = val & ~(SLEEPWAKE_CFG_SNCEN_MASK);
		ret = pChip->SendCommand(client, cmd_set, sizeof(cmd_set), NULL, 0);
		if(ret < 0)
			goto out;
		pChip->activity = MMA_RUN;
		printk("mma sleep setting inactive\n");
	}
	else if(sleep  && pDev->pChip->activity == MMA_RUN)
	{
		cmd_set[4] = val | SLEEPWAKE_CFG_SNCEN_MASK;
		ret = pChip->SendCommand(client, cmd_set, sizeof(cmd_set), NULL, 0);
		if(ret < 0)
			goto out;
		pChip->activity= MMA_SLEEP;
		printk("mma sleep setting active\n");
	}
	ret = count;
out:
	mutex_unlock(&pDev->data_lock);
	return ret;
}

/*
* This method is used to get poll time.
* filesys item: /sys/class/mma955x/mma955x/poll_time
*/
ssize_t mma_poll_time_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct mxc_mma_device_t *pDev = dev_get_drvdata(dev);
	struct chipInfo_t *pChip = pDev->pChip;
	return sprintf(buf, "%d\n", pChip->poll_time * 10);
}

/*
* This method is used to set the polling time. 
* Accepted values : ms
* Note: this attribute is stored in ms x 10 units
* filesys item: /sys/class/mma955x/mma955x/poll_time
*/
ssize_t mma_poll_time_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct mxc_mma_device_t *pDev = dev_get_drvdata(dev);
	struct chipInfo_t *pChip = pDev->pChip;
	unsigned long poll_time_ms = simple_strtoul(buf, NULL, 10);

	if (poll_time_ms <= 0) 
        {
		printk("Invalid poll time selection (buf: %s count: %d)\n", buf, count);
                return -EINVAL;
        }

	pChip->poll_time = (poll_time_ms / 10);
	printk("poll time set to: %d\n", pChip->poll_time * 10);
	return count;
}


/******************************************************************************
*		MMA9550-specific Sysfs info (hub)
*******************************************************************************/

/*
* This method is used to get the position of the chip relative to the display graphics
* filesys item: /sys/class/mma955x/mma955x/poll_time
*/
ssize_t hub_position_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct mxc_mma_device_t *pDev = dev_get_drvdata(dev);
	return sprintf(buf, "%d\n", pDev->pChip->position);
}

/*
* This method is used to set the position of the chip relative to the display graphics
* Accepted values : 0..7
* filesys item: /sys/class/mma955x/mma955x/position
*/
ssize_t hub_position_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct mxc_mma_device_t *pDev = dev_get_drvdata(dev);
	struct chipInfo_t *pChip = pDev->pChip;
	int position = simple_strtoul(buf, NULL, 10);

	if((position > 7) || (position < 0))
	{
		printk("Invalid position selection (buf: %s count: %d)\n", buf, count);
		return -EINVAL;
	}

	pChip->position = position;
	printk("position set to: %d\n", pChip->position);
	return count;
}

/*
* This method is used to get the axis invertion flag for accelerometer
* filesys item: /sys/class/mma955x/mma955x/acc_invert
*/
ssize_t hub_acc_invert_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct mxc_mma_device_t *pDev = dev_get_drvdata(dev);
	return sprintf(buf, "%d\n", pDev->pChip->acc_invert);
}

/*
* This method is used to set the axis invertion flag for accelerometer
* Accepted values : 0, 1
* filesys item: /sys/class/mma955x/mma955x/acc_invert
*/
ssize_t hub_acc_invert_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct mxc_mma_device_t *pDev = dev_get_drvdata(dev);
	int acc_invert = simple_strtoul(buf, NULL, 10);
	pDev->pChip->acc_invert = acc_invert;
	return count;
}

/*
* This method is used to get the axis invertion flag for gyroscope
* filesys item: /sys/class/mma955x/mma955x/gyro_invert
*/
ssize_t hub_gyro_invert_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct mxc_mma_device_t *pDev = dev_get_drvdata(dev);
	return sprintf(buf, "%d\n", pDev->pChip->gyro_invert);
}

/*
* This method is used to set the axis invertion flag for gyroscope
* Accepted values : 0, 1
* filesys item: /sys/class/mma955x/mma955x/gyro_invert
*/
ssize_t hub_gyro_invert_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct mxc_mma_device_t *pDev = dev_get_drvdata(dev);
	int gyro_invert = simple_strtoul(buf, NULL, 10);
	pDev->pChip->gyro_invert = gyro_invert;
	return count;
}

/*
* This method is used to get gyroscope range
* filesys item: /sys/class/mma955x/mma955x/gyro_range
*/
ssize_t hub_gyro_range_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct mxc_mma_device_t *pDev = dev_get_drvdata(dev);
	return sprintf(buf, "%d\n", pDev->pChip->gyro_range);
}

/*
* This method is used to set the gyroscope range
* Accepted values : 0 (250 dps), 1 (500 dps), 2 (2000 dps)
* filesys item: /sys/class/mma955x/mma955x/gyro_range
*/
ssize_t hub_gyro_range_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	ssize_t ret;
	u8 val;
	struct mxc_mma_device_t *pDev = dev_get_drvdata(dev);
	struct chipInfo_t *pChip = pDev->pChip;
	struct i2c_client *client = pChip->client;

	u8 cmd[6] = { HUB_APPID, CMD_WR_CFG, 0x00, 0x02, 0x01, 0};	/* this command re-init gyroscope with current config values */

	int gyro_range = simple_strtoul(buf, NULL, 10);

	switch(gyro_range)
	{
		case HUB_GYR_RANGE_250DPS:	val = 0x81;//(u8)(gyro_range << HUB_GYR_RANGE_MASK_BIT);
		break;
		case HUB_GYR_RANGE_500DPS:	val = 0x91;//(u8)(gyro_range << HUB_GYR_RANGE_MASK_BIT);
		break;
		case HUB_GYR_RANGE_2000DPS:	val = 0xa1;//(u8)(gyro_range << HUB_GYR_RANGE_MASK_BIT);
		break;
		default:
		{
			printk("Invalid gyroscope range selection (buf: %s count: %d)\n", buf, count);
			return -EINVAL;
		}

	}

	cmd[5] = val;

	mutex_lock(&pDev->data_lock);
	ret = pChip->SendCommand(client, cmd, sizeof(cmd), NULL, 0);
	if(ret < 0)
		goto out;

	pChip->gyro_range = gyro_range;
	printk("gyroscope range set to: %d\n", pChip->gyro_range);

	ret = count;
out:
	mutex_unlock(&pDev->data_lock);
	return ret;
}


/*
* This method is used to get AFE output stage for accelerometer event readings.
* Accepted values : 0..5
* filesys item: /sys/class/mma955x/mma955x/acc_stage
*/
ssize_t hub_acc_output_stage_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct mxc_mma_device_t *pDev = dev_get_drvdata(dev);
	return sprintf(buf, "%d\n", pDev->pChip->acc_output_stage);
}

/*
* This method is used to set AFE output stage for accelerometer event readings.
* Accepted values : 0..5
* filesys item: /sys/class/mma955x/mma955x/acc_stage
*/
ssize_t hub_acc_output_stage_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct mxc_mma_device_t *pDev = dev_get_drvdata(dev);
	int acc_output_stage = simple_strtoul(buf, NULL, 10);
	if(acc_output_stage >= FRONTEND_OUTPUTS)
	{
		printk("Invalid AFE output stage selection (buf: %s count: %d)\n", buf, count);
		return -EINVAL;
	}

	pDev->pChip->acc_output_stage = acc_output_stage;
	return count;
}

/*
* This method is used to parse the string according to delimiter.
* Data 	: Pointer to the buffer containing string to be parsed.
* Length	: length of string in bytes.
* Token 	: Token for parsing the string .
* TokenCnt 	: length of token string.
* return TokenCount 	: No of token occurrences found in input string.
*/
static int ParseString(char *Data, int Length, char Delimiter, char *Tokens[], int TokenCnt)
{
	int TokenCount = 0;
	int Iterator;

	TokenCnt--;
	Tokens[TokenCount++] = Data;
	for(Iterator = 0; (Iterator < Length) && TokenCnt; Iterator++)
	{
		if(Data[Iterator] == Delimiter)
		{
			Data[Iterator] = '\0';
			Tokens[TokenCount] = &Data[Iterator +1 ];
			TokenCount++;
			TokenCnt--;
		}
	}
	Iterator = 0;
	while(Iterator < TokenCount)
	{
		printk("TOKEN %d: [%s]\r\n", Iterator, Tokens[Iterator]);
		Iterator++;
	}

	return TokenCount;
}

/*
* This method is used to get calibration offset values.
* filesys item: /sys/class/mma955x/mma955x/
*/
ssize_t hub_acc_offset_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t ret;
	struct mxc_mma_device_t *pDev = dev_get_drvdata(dev);
	short x_offset, y_offset, z_offset;

	x_offset = pDev->pChip->acc_ofs_x;
	y_offset = pDev->pChip->acc_ofs_y;
	z_offset = pDev->pChip->acc_ofs_z;

	sprintf(buf, "%d,%d,%d\n", x_offset, y_offset, z_offset);
	ret = strlen(buf) + 1;
	return ret;
}

/*
* This method is used to set calibration offsets.
* Accepted values : 
* filesys item: /sys/class/mma955x/mma955x/acc_offset
*/
ssize_t hub_acc_offset_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	ssize_t ret;
	int i=0;
	char *params[3];
	short val[3];
	long lval[3];
	u8 cmd[10] = { AFE_APPID, CMD_WR_CFG, 0x02, 0x06, 0, 0, 0, 0, 0, 0};

	struct mxc_mma_device_t *pDev = dev_get_drvdata(dev);
	struct chipInfo_t *pChip = pDev->pChip;
	struct i2c_client *client = pChip->client;

	ParseString((char *)buf, strlen(buf), ',', params, 3);
	while(params[i] != NULL && i < 3)
	{
		if (strict_strtol(params[i], 10, &lval[i]))
			return -EINVAL;
		if((lval[i] > 32767) || (lval[i] < -32768))
		{
			printk("Invalid calibration offset value(buf: %s count: %d)\n", buf, count);
			return -EINVAL;
		}
		val[i] = (short)lval[i];
		i++;
	}

	cmd[4] = HIBYTE(val[0]);
	cmd[5] = LOBYTE(val[0]);
	cmd[6] = HIBYTE(val[1]);
	cmd[7] = LOBYTE(val[1]);
	cmd[8] = HIBYTE(val[2]);
	cmd[9] = LOBYTE(val[2]);

	mutex_lock(&pDev->data_lock);
	ret = pChip->SendCommand(client, cmd, sizeof(cmd), NULL, 0);
	if(ret < 0)
		goto out;

	pChip->acc_ofs_x = val[0];
	pChip->acc_ofs_y = val[1];
	pChip->acc_ofs_z = val[2];
	mutex_unlock(&pDev->data_lock);
	ret = count;
out:
	return ret;
}

/*
* This method is used to get ambient sensors calibration offset values.
* filesys item: /sys/class/mma955x/mma955x/amb_offset
*/
ssize_t hub_amb_offset_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t ret;
	struct mxc_mma_device_t *pDev = dev_get_drvdata(dev);
	short P_offset, H_offset, T_offset;

	P_offset = pDev->pChip->amb_ofs_P;
	H_offset = pDev->pChip->amb_ofs_H;
	T_offset = pDev->pChip->amb_ofs_T;

	sprintf(buf, "%d,%d,%d\n", P_offset, H_offset, T_offset);
	ret = strlen(buf) + 1;
	return ret;
}

/*
* This method is used to set calibration offsets.
* Accepted values : 
* filesys item: /sys/class/mma955x/mma955x/amb_offset
*/
ssize_t hub_amb_offset_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	ssize_t ret;
	int i=0;
	char *params[3];
	char val[3];
	long lval[3];
	u8 cmd[7] = { HUB_APPID, CMD_WR_CFG, 0x04, 0x03, 0, 0, 0};
	u8 cmd_init[5] = { HUB_APPID, CMD_WR_CFG, 0x02, 0x01, 0x01};	/* this command re-init gyroscope with current config values */

	struct mxc_mma_device_t *pDev = dev_get_drvdata(dev);
	struct chipInfo_t *pChip = pDev->pChip;
	struct i2c_client *client = pChip->client;

	ParseString((char *)buf, strlen(buf), ',', params, 3);
	while(params[i] != NULL && i < 3)
	{
		if (strict_strtol(params[i], 10, &lval[i]))
			return -EINVAL;
		if((lval[i] > 128) || (lval[i] < -127))
		{
			printk("Invalid calibration offset value(buf: %s count: %d)\n", buf, count);
			return -EINVAL;
		}
		val[i] = (char)lval[i];
		i++;
	}

	cmd[4] = (u8)val[0];
	cmd[5] = (u8)val[1];
	cmd[6] = (u8)val[2];

	mutex_lock(&pDev->data_lock);
	ret = pChip->SendCommand(client, cmd, sizeof(cmd), NULL, 0);
	if(ret < 0)
		goto out;

	ret = pChip->SendCommand(client, cmd_init, sizeof(cmd_init), NULL, 0);
	if(ret < 0)
		goto out;

	pChip->amb_ofs_P = val[0];
	pChip->amb_ofs_H = val[1];
	pChip->amb_ofs_T = val[2];
	ret = count;
out:
	mutex_unlock(&pDev->data_lock);
	return ret;
}

/*
* This method is used to get accelerometer range.
* filesys item: /sys/class/mma955x/mma955x/acc_range
*/
ssize_t hub_acc_range_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct mxc_mma_device_t *pDev = dev_get_drvdata(dev);
	return sprintf(buf, "%d\n", pDev->pChip->acc_range);
}

/*
* This method is used to set accelerometer range.
* Accepted values : 2, 4, 8 (for +-2g, +-4g, +-8g)
* filesys item: /sys/class/mma955x/mma955x/acc_range
*/
ssize_t hub_acc_range_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	ssize_t ret;
	struct mxc_mma_device_t *pDev = dev_get_drvdata(dev);
	struct chipInfo_t *pChip = pDev->pChip;
	struct i2c_client *client = pChip->client;

	u8 cmd[5] = { AFE_APPID, CMD_WR_CFG, 0x00, 0x01, 0};

	int range = simple_strtoul(buf, NULL, 10);
	switch(range)
	{
		case ACC_RANGE_2G: cmd[4] = (FS_2G << AFE_CSR_FS_BIT);	/* range is +-2g; sensitivity is 0.061 mg/LSB or 16394/g */
		break;
		case ACC_RANGE_4G: cmd[4] = (FS_4G << AFE_CSR_FS_BIT);	/* range is +-4g; sensitivity is 0.122 mg/LSB or 8196/g */
		break;
		case ACC_RANGE_8G: cmd[4] = (FS_8G << AFE_CSR_FS_BIT);	/* range is +-8g; sensitivity is 0.244 mg/LSB or 4096/g */
		break;
		default:
		{
			printk("Invalid Range, (buf: %s count: %d)\n", buf, count);
			return -EINVAL;
		}
		break;
	}

	mutex_lock(&pDev->data_lock);
	pChip->SendRSC(client, AFE_APPID, SUSPEND, 0);	/* enter suspend mode before configuring */
	ret = pChip->SendCommand(client, cmd, sizeof(cmd), NULL, 0);
	pChip->SendRSC(client, AFE_APPID, SUSPEND, 1);	/* exit suspend mode */
	if(ret < 0)
		goto out;
	pChip->acc_range = range;
	ret = count;
out:
	mutex_unlock(&pDev->data_lock);
	return ret;
}


/*
* This method is used to get list of supported resolutions. 
* filesys item: /sys/class/mma955x/mma955x/resolutions
*/
ssize_t hub_resolutions_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;
	int len = 0;
	int i = 0;

	struct mxc_mma_device_t *pDev = dev_get_drvdata(dev);
	struct chipInfo_t *pChip = pDev->pChip;

	while(pChip->pSupportedResolution[i] != -1)
	{
		sprintf(&buf[len], "%d,",pChip->pSupportedResolution[i]);
		len = strlen(buf);
		i++;
	}
	buf[len - 1] = '\0';
	ret = strlen(buf) + 1;
	return ret;

}

/*
* This method is used to get current resolution
* filesys item: /sys/class/mma955x/mma955x/resolution
*/
ssize_t hub_resolution_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct mxc_mma_device_t *pDev = dev_get_drvdata(dev);
	struct chipInfo_t *pChip = pDev->pChip;
	return sprintf(buf, "%d\n", pChip->pSupportedResolution[pChip->resolution]);
}


/*
* This method is used to set resolution. 
* Accepted values :
* filesys item: /sys/class/mma955x/mma955x/acc_resolution
*/
ssize_t hub_resolution_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int i = 0;
	unsigned long val = 0;

	struct mxc_mma_device_t *pDev = dev_get_drvdata(dev);
	struct chipInfo_t *pChip = pDev->pChip;
	strict_strtoul(buf, 10, &val);

	for(i = 0; pChip->pSupportedResolution[i] != -1; i++)
	{
		if(val == pChip->pSupportedResolution[i])
		{
			printk("Setting resolution to %d\r\n", pChip->pSupportedResolution[i]);
//todo...			pChip->SetRegVal(CMD_RESOLUTION, i);
			pChip->resolution = i;
			return count;
		}
	}
	printk("Invalid Resolution, (buf: %s count: %d)\n", buf, count);
	return -EINVAL;
}


/******************************************************************************
*		MMA9553-specific Sysfs info (pedometer)
*******************************************************************************/
/*
* This method is used to get current sleep minimum (units: 0.244mg/LSB).
* It is the inf of the |G| inerval where the pedometer may auto-suspend.
* it is expected to be near and below 1g (4096 at 0.244 mg/LSB)
* filesys item: /sys/class/mma955x/mma955x/sleep_min
*/
ssize_t ped_sleep_min_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct mxc_mma_device_t *pDev = dev_get_drvdata(dev);
	return sprintf(buf, "%d\n", pDev->pChip->sleep_min);
}

/*
* This method is used to set sleep minimum (units: 0.244mg/LSB).
* Accepted values : 0..65535
* filesys item: /sys/class/mma955x/mma955x/sleep_min
*/
ssize_t ped_sleep_min_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	ssize_t ret;
	struct mxc_mma_device_t *pDev = dev_get_drvdata(dev);
	struct chipInfo_t *pChip = pDev->pChip;
	struct i2c_client *client = pChip->client;

	u8 cmd[6] = { PEDOM_APPID, CMD_WR_CFG, 0x00, 0x02, 0, 0 };
	u8 cmd_init[5] = { PEDOM_APPID, CMD_WR_CFG, 0x06, 0x01, 0x80 }; /* this command re-init the pedometer with current config values */

	int sleep_min = simple_strtoul(buf, NULL, 10);

	if((sleep_min < 0) || (sleep_min > 0xFFFF))
	{
		printk("Invalid Sleep Minimum selection (buf: %s count: %d)\n", buf, count);
		return -EINVAL;
	}

	cmd[4] = HIBYTE(sleep_min);
	cmd[5] = LOBYTE(sleep_min);

	mutex_lock(&pDev->data_lock);
	ret = pChip->SendCommand(client, cmd, sizeof(cmd), NULL, 0);
	if(ret < 0)
		goto out;
	ret = pChip->SendCommand(client, cmd_init, sizeof(cmd_init), NULL, 0);
	if(ret < 0)
		goto out;

	pChip->sleep_min = sleep_min;
	ret = count;
out:
	mutex_unlock(&pDev->data_lock);
	return ret;
}


/*
* This method is used to get current sleep maximum (units: 0.244mg/LSB).
* It is the sup of the |G| inerval where the pedometer may auto-suspend.
* it is expected to be near and above 1g (4096 at 0.244 mg/LSB)
* filesys item: /sys/class/mma955x/mma955x/sleep_max
*/
ssize_t ped_sleep_max_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct mxc_mma_device_t *pDev = dev_get_drvdata(dev);
	return sprintf(buf, "%d\n", pDev->pChip->sleep_max);
}

/*
* This method is used to set sleep maximum (units: 0.244mg/LSB).
* Accepted values : 0 (disable auto-suspend), 1..65535
* filesys item: /sys/class/mma955x/mma955x/sleep_max
*/
ssize_t ped_sleep_max_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	ssize_t ret;
	struct mxc_mma_device_t *pDev = dev_get_drvdata(dev);
	struct chipInfo_t *pChip = pDev->pChip;
	struct i2c_client *client = pChip->client;

	u8 cmd[6] = { PEDOM_APPID, CMD_WR_CFG, 0x02, 0x02, 0, 0 };
	u8 cmd_init[5] = { PEDOM_APPID, CMD_WR_CFG, 0x06, 0x01, 0x80 }; /* this command re-init the pedometer with current config values */

	int sleep_max = simple_strtoul(buf, NULL, 10);

	if((sleep_max < 0) || (sleep_max > 0xFFFF))
	{
		printk("Invalid Sleep Maximum selection (buf: %s count: %d)\n", buf, count);
		return -EINVAL;
	}

	cmd[4] = HIBYTE(sleep_max);
	cmd[5] = LOBYTE(sleep_max);

	mutex_lock(&pDev->data_lock);
	ret = pChip->SendCommand(client, cmd, sizeof(cmd), NULL, 0);
	if(ret < 0)
		goto out;
	ret = pChip->SendCommand(client, cmd_init, sizeof(cmd_init), NULL, 0);
	if(ret < 0)
		goto out;
	if(!sleep_max)
		printk("mma auto-suspend disabled\n");

	pChip->sleep_max = sleep_max;
	ret = count;
out:
	mutex_unlock(&pDev->data_lock);
	return ret;
}

/*
* This method is used to get current sleep threshold (units: 0.244mg/LSB)
* It is the number of samples where sleep_min<G<sleep_max for the pedometer to auto-suspend.
* filesys item: /sys/class/mma955x/mma955x/sleep_thd
*/
ssize_t ped_sleep_thd_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct mxc_mma_device_t *pDev = dev_get_drvdata(dev);
	return sprintf(buf, "%d\n", pDev->pChip->sleep_thd);
}

/*
* This method is used to set sleep threshold (units: 0.244mg/LSB).
* Accepted values : 0..65535
* filesys item: /sys/class/mma955x/mma955x/sleep_thd
*/
ssize_t ped_sleep_thd_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	ssize_t ret;
	struct mxc_mma_device_t *pDev = dev_get_drvdata(dev);
	struct chipInfo_t *pChip = pDev->pChip;
	struct i2c_client *client = pChip->client;

	u8 cmd[6] = { PEDOM_APPID, CMD_WR_CFG, 0x04, 0x02, 0, 0 };
	u8 cmd_init[5] = { PEDOM_APPID, CMD_WR_CFG, 0x06, 0x01, 0x80 }; /* this command re-init the pedometer with current config values */

	int sleep_thd = simple_strtoul(buf, NULL, 10);

	if(sleep_thd < 0)
	{
		printk("Invalid Sleep Threshold selection (buf: %s count: %d)\n", buf, count);
		return -EINVAL;
	}

	cmd[4] = HIBYTE(sleep_thd);
	cmd[5] = LOBYTE(sleep_thd);

	mutex_lock(&pDev->data_lock);
	ret = pChip->SendCommand(client, cmd, sizeof(cmd), NULL, 0);
	if(ret < 0)
		goto out;
	ret = pChip->SendCommand(client, cmd_init, sizeof(cmd_init), NULL, 0);
	if(ret < 0)
		goto out;

	pChip->sleep_thd = sleep_thd;
	ret = count;
out:
	mutex_unlock(&pDev->data_lock);
	return ret;
}

/*
* This method is used to get current step lenght (units: cm).
* filesys item: /sys/class/mma955x/mma955x/steplen
*/
ssize_t ped_steplen_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct mxc_mma_device_t *pDev = dev_get_drvdata(dev);
	return sprintf(buf, "%d\n", pDev->pChip->steplen);
}

/*
* This method is used to set step lenght (units: cm)
* Accepted values : 0 (estimate based on gender and height), 1..255
* filesys item: /sys/class/mma955x/mma955x/steplen
*/
ssize_t ped_steplen_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	ssize_t ret;
	struct mxc_mma_device_t *pDev = dev_get_drvdata(dev);
	struct chipInfo_t *pChip = pDev->pChip;
	struct i2c_client *client = pChip->client;

	u8 cmd[6] = { PEDOM_APPID, CMD_WR_CFG, 0x06, 0x02, 0x80, 0 };

	int steplen = simple_strtoul(buf, NULL, 10);

	if((steplen < 0) || (steplen > 255))
	{
		printk("Invalid Step Lenght selection (buf: %s count: %d)\n", buf, count);
		return -EINVAL;
	}

	cmd[5] = (u8)steplen;

	mutex_lock(&pDev->data_lock);
	ret = pChip->SendCommand(client, cmd, sizeof(cmd), NULL, 0);
	if(ret < 0)
		goto out;

	pChip->steplen = steplen;
	ret = count;
out:
	mutex_unlock(&pDev->data_lock);
	return ret;
}


/*
* This method is used to get the subject height (units: cm)
* filesys item: /sys/class/mma955x/mma955x/height
*/
ssize_t ped_height_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct mxc_mma_device_t *pDev = dev_get_drvdata(dev);
	return sprintf(buf, "%d\n", pDev->pChip->height);
}

/*
* This method is used to set the subject height (units: cm)
* Accepted values :
* filesys item: /sys/class/mma955x/mma955x/height
*/
ssize_t ped_height_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	ssize_t ret;
	struct mxc_mma_device_t *pDev = dev_get_drvdata(dev);
	struct chipInfo_t *pChip = pDev->pChip;
	struct i2c_client *client = pChip->client;

	u8 cmd[5] = { PEDOM_APPID, CMD_WR_CFG, 0x08, 0x01, 0 };
	u8 cmd_init[5] = { PEDOM_APPID, CMD_WR_CFG, 0x06, 0x01, 0x80 }; /* this command re-init the pedometer with current config values */

	int height = simple_strtoul(buf, NULL, 10);

	if((height < 0) || (height > 255))
	{
		printk("Invalid Height selection (buf: %s count: %d)\n", buf, count);
		return -EINVAL;
	}

	cmd[4] = (u8)height;

	mutex_lock(&pDev->data_lock);
	ret = pChip->SendCommand(client, cmd, sizeof(cmd), NULL, 0);
	if(ret < 0)
		goto out;

	ret = pChip->SendCommand(client, cmd_init, sizeof(cmd_init), NULL, 0);
	if(ret < 0)
		goto out;

	pChip->height = height;
	ret = count;
out:
	mutex_unlock(&pDev->data_lock);
	return ret;
}


/*
* This method is used to get the subject weight (units: kg)
* filesys item: /sys/class/mma955x/mma955x/weight
*/
ssize_t ped_weight_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct mxc_mma_device_t *pDev = dev_get_drvdata(dev);
	return sprintf(buf, "%d\n", pDev->pChip->weight);
}

/*
* This method is used to set the subject weight (units: kg)
* Accepted values :
* filesys item: /sys/class/mma955x/mma955x/weight
*/
ssize_t ped_weight_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	ssize_t ret;
	struct mxc_mma_device_t *pDev = dev_get_drvdata(dev);
	struct chipInfo_t *pChip = pDev->pChip;
	struct i2c_client *client = pChip->client;

	u8 cmd[5] = { PEDOM_APPID, CMD_WR_CFG, 0x09, 0x01, 0 };
	u8 cmd_init[5] = { PEDOM_APPID, CMD_WR_CFG, 0x06, 0x01, 0x80 }; /* this command re-init the pedometer with current config values */

	int weight = simple_strtoul(buf, NULL, 10);

	if((weight < 0) || (weight > 255))
	{
		printk("Invalid weight selection (buf: %s count: %d)\n", buf, count);
		return -EINVAL;
	}

	cmd[4] = (u8)weight;

	mutex_lock(&pDev->data_lock);
	ret = pChip->SendCommand(client, cmd, sizeof(cmd), NULL, 0);
	if(ret < 0)
		goto out;

	ret = pChip->SendCommand(client, cmd_init, sizeof(cmd_init), NULL, 0);
	if(ret < 0)
		goto out;

	pChip->weight = weight;
	ret = count;
out:
	mutex_unlock(&pDev->data_lock);
	return ret;
}

/*
* This method is used to get current filter step (units: step).
* The number of steps that must occur within filter_time to decide the user is making steps
* filesys item: /sys/class/mma955x/mma955x/filter_step
*/
ssize_t ped_filter_step_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct mxc_mma_device_t *pDev = dev_get_drvdata(dev);
	return sprintf(buf, "%d\n", pDev->pChip->filter_step);
}

/*
* This method is used to set filter_step (units: step)
* Accepted values : 0 (disable), 1..6
* filesys item: /sys/class/mma955x/mma955x/filter_step
*/
ssize_t ped_filter_step_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	ssize_t ret;
	struct mxc_mma_device_t *pDev = dev_get_drvdata(dev);
	struct chipInfo_t *pChip = pDev->pChip;
	struct i2c_client *client = pChip->client;

	u8 cmd[5] = { PEDOM_APPID, CMD_WR_CFG, 0x0A, 0x01, 0 };
	u8 cmd_init[5] = { PEDOM_APPID, CMD_WR_CFG, 0x06, 0x01, 0x80 }; /* this command re-init the pedometer with current config values */

	int filter_step = simple_strtoul(buf, NULL, 10);

	if((filter_step < 0) || (filter_step > 6))
	{
		printk("Invalid filter_step selection (buf: %s count: %d)\n", buf, count);
		return -EINVAL;
	}

	cmd[4] = (u8)filter_step;

	mutex_lock(&pDev->data_lock);
	ret = pChip->SendCommand(client, cmd, sizeof(cmd), NULL, 0);
	if(ret < 0)
		goto out;

	ret = pChip->SendCommand(client, cmd_init, sizeof(cmd_init), NULL, 0);
	if(ret < 0)
		goto out;

	pChip->filter_step = filter_step;
	ret = count;
out:
	mutex_unlock(&pDev->data_lock);
	return ret;
}

/*
* This method is used to get subject's gender.
* filesys item: /sys/class/mma955x/mma955x/gender
*/
ssize_t ped_gender_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct mxc_mma_device_t *pDev = dev_get_drvdata(dev);
	return sprintf(buf, "%d\n", pDev->pChip->gender);
}

/*
* This method is used to set subject's gender
* Accepted values : 0 (Female), 1 (Male)
* filesys item: /sys/class/mma955x/mma955x/gender
*/
ssize_t ped_gender_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	ssize_t ret;
	u8 val = 0;
	struct mxc_mma_device_t *pDev = dev_get_drvdata(dev);
	struct chipInfo_t *pChip = pDev->pChip;
	struct i2c_client *client = pChip->client;

	u8 cmd_req[4] = { PEDOM_APPID, CMD_RD_CFG, 0x0B, 0x01 };
	u8 cmd_set[5] = { PEDOM_APPID, CMD_WR_CFG, 0x0B, 0x01, 0};
	u8 cmd_init[5] = { PEDOM_APPID, CMD_WR_CFG, 0x06, 0x01, 0x80 }; /* this command re-init the pedometer with current config values */

	int gender = simple_strtoul(buf, NULL, 10);

	mutex_lock(&pDev->data_lock);
	ret = pChip->SendCommand(client, cmd_req, sizeof(cmd_req), &val, 1);
	if(ret < 1)
		goto out;

	if(!gender)
	{
		cmd_set[4] = val & ~(PEDOMETER_CFG_GENDER_MASK);
		pChip->gender = PEDOMETER_CFG_GENDER_FEMALE;
		printk("mma gender setting Female\n");
	}
	else
	{
		cmd_set[4] = val | PEDOMETER_CFG_GENDER_MASK;
		pChip->gender = PEDOMETER_CFG_GENDER_MALE;
		printk("mma gender setting Male\n");
	}

	ret = pChip->SendCommand(client, cmd_set, sizeof(cmd_set), NULL, 0);
	if(ret < 0)
		goto out;

	ret = pChip->SendCommand(client, cmd_init, sizeof(cmd_init), NULL, 0);
	if(ret < 0)
		goto out;

	ret = count;
out:
	mutex_unlock(&pDev->data_lock);
	return ret;
}

/*
* This method is used to get filter_time  (units: s)
* The time interval within filter_step steps must be done to decide the user is making steps
* filesys item: /sys/class/mma955x/mma955x/filter_time
*/
ssize_t ped_filter_time_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct mxc_mma_device_t *pDev = dev_get_drvdata(dev);
	return sprintf(buf, "%d\n", pDev->pChip->filter_time);
}

/*
* This method is used to set filter_time (units: s)
* Accepted values : 0 (disable step filtering), 1..127
* filesys item: /sys/class/mma955x/mma955x/filter_time
*/
ssize_t ped_filter_time_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	ssize_t ret;
	u8 val = 0;
	struct mxc_mma_device_t *pDev = dev_get_drvdata(dev);
	struct chipInfo_t *pChip = pDev->pChip;
	struct i2c_client *client = pChip->client;

	u8 cmd_req[4] = { PEDOM_APPID, CMD_RD_CFG, 0x0B, 0x01 };
	u8 cmd_set[5] = { PEDOM_APPID, CMD_WR_CFG, 0x0B, 0x01, 0};
	u8 cmd_init[5] = { PEDOM_APPID, CMD_WR_CFG, 0x06, 0x01, 0x80 }; /* this command re-init the pedometer with current config values */

	int filter_time = simple_strtoul(buf, NULL, 10);

	if((filter_time < 0) || (filter_time > 127))
	{
		printk("Invalid filter_time selection (buf: %s count: %d)\n", buf, count);
		return -EINVAL;
	}

	mutex_lock(&pDev->data_lock);
	ret = pChip->SendCommand(client, cmd_req, sizeof(cmd_req), &val, 1);
	if(ret < 1)
		goto out;

	cmd_set[4] = (u8)(val & PEDOMETER_CFG_GENDER_MASK) | (u8)(filter_time);

	ret = pChip->SendCommand(client, cmd_set, sizeof(cmd_set), NULL, 0);
	if(ret < 0)
		goto out;

	ret = pChip->SendCommand(client, cmd_init, sizeof(cmd_init), NULL, 0);
	if(ret < 0)
		goto out;
	if(!filter_time)
		printk("mma step filtering disabled\n");

	pChip->filter_time = filter_time;
	ret = count;
out:
	mutex_unlock(&pDev->data_lock);
	return ret;
}

/*
* This method is used to get current speed period (units: s)
* It is the number of seconds in which to compute the average speed
* filesys item: /sys/class/mma955x/mma955x/speed_period
*/
ssize_t ped_speed_period_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct mxc_mma_device_t *pDev = dev_get_drvdata(dev);
	return sprintf(buf, "%d\n", pDev->pChip->speed_period);
}

/*
* This method is used to set speed_period (units: s)
* Accepted values : 1..5
* filesys item: /sys/class/mma955x/mma955x/speed_period
*/
ssize_t ped_speed_period_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	ssize_t ret;
	struct mxc_mma_device_t *pDev = dev_get_drvdata(dev);
	struct chipInfo_t *pChip = pDev->pChip;
	struct i2c_client *client = pChip->client;

	u8 cmd[5] = { PEDOM_APPID, CMD_WR_CFG, 0x0c, 0x01, 0 };
	u8 cmd_init[5] = { PEDOM_APPID, CMD_WR_CFG, 0x06, 0x01, 0x80 }; /* this command re-init the pedometer with current config values */

	int speed_period = simple_strtoul(buf, NULL, 10);

	if((speed_period < 1) || (speed_period > 5))
	{
		printk("Invalid Speed Period selection (buf: %s count: %d)\n", buf, count);
		return -EINVAL;
	}

	cmd[4] = (u8)speed_period;

	mutex_lock(&pDev->data_lock);
	ret = pChip->SendCommand(client, cmd, sizeof(cmd), NULL, 0);
	if(ret < 0)
		goto out;

	ret = pChip->SendCommand(client, cmd_init, sizeof(cmd_init), NULL, 0);
	if(ret < 0)
		goto out;

	pChip->speed_period = speed_period;
	ret = count;
out:
	mutex_unlock(&pDev->data_lock);
	return ret;
}

/*
* This method is used to get current step coalesce (units: step).
* filesys item: /sys/class/mma955x/mma955x/step_coalesce
*/
ssize_t ped_step_coalesce_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct mxc_mma_device_t *pDev = dev_get_drvdata(dev);
	return sprintf(buf, "%d\n", pDev->pChip->step_coalesce);
}

/*
* This method is used to set step_coalesce (units: step)
* Accepted values :
* filesys item: /sys/class/mma955x/mma955x/step_coalesce
*/
extern int ped_getCfg(struct i2c_client *client, mmaPedCfg_t *p); //butta
ssize_t ped_step_coalesce_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	ssize_t ret;
	struct mxc_mma_device_t *pDev = dev_get_drvdata(dev);
	struct chipInfo_t *pChip = pDev->pChip;
	struct i2c_client *client = pChip->client;

	u8 cmd[5] = { PEDOM_APPID, CMD_WR_CFG, 0x0D, 0x01, 0 };
	u8 cmd_init[5] = { PEDOM_APPID, CMD_WR_CFG, 0x06, 0x01, 0x80 }; /* this command re-init the pedometer with current config values */

	int step_coalesce = simple_strtoul(buf, NULL, 10);

	step_coalesce = (step_coalesce > 0) ? 1 : 0;

	cmd[4] = (u8)step_coalesce;

	mutex_lock(&pDev->data_lock);
	ret = pChip->SendCommand(client, cmd, sizeof(cmd), NULL, 0);
	if(ret < 0)
		goto out;

	ret = pChip->SendCommand(client, cmd_init, sizeof(cmd_init), NULL, 0);
	if(ret < 0)
		goto out;

	pChip->step_coalesce = step_coalesce;
	ret = count;
out:

//butta.
if(step_coalesce == 1)
{
	mmaPedCfg_t cfg;
	ped_getCfg(client, &cfg);
}

	mutex_unlock(&pDev->data_lock);
	return ret;
}

/*
* This method is used to get current activity threshold (units: samples).
* The internal activity level must be stable for activity_thd samples before activity is update 
* filesys item: /sys/class/mma955x/mma955x/activity_thd
*/
ssize_t ped_activity_thd_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct mxc_mma_device_t *pDev = dev_get_drvdata(dev);
	return sprintf(buf, "%d\n", pDev->pChip->activity_thd);
}

/*
* This method is used to set activity threshold (units: samples)
* Accepted values : 0 (disable activity debouncer), 1..65535
* filesys item: /sys/class/mma955x/mma955x/activity_thd
*/
ssize_t ped_activity_thd_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	ssize_t ret;
	struct mxc_mma_device_t *pDev = dev_get_drvdata(dev);
	struct chipInfo_t *pChip = pDev->pChip;
	struct i2c_client *client = pChip->client;

	u8 cmd[6] = { PEDOM_APPID, CMD_WR_CFG, 0x0E, 0x02, 0, 0 };
	u8 cmd_init[5] = { PEDOM_APPID, CMD_WR_CFG, 0x06, 0x01, 0x80 }; /* this command re-init the pedometer with current config values */

	int activity_thd = simple_strtoul(buf, NULL, 10);

	if((activity_thd < 0) || (activity_thd > 0xFFFF))
	{
		printk("Invalid Activity Threshold selection (buf: %s count: %d)\n", buf, count);
		return -EINVAL;
	}

	cmd[4] = HIBYTE(activity_thd);
	cmd[5] = LOBYTE(activity_thd);

	mutex_lock(&pDev->data_lock);
	ret = pChip->SendCommand(client, cmd, sizeof(cmd), NULL, 0);
	if(ret < 0)
		goto out;

	ret = pChip->SendCommand(client, cmd_init, sizeof(cmd_init), NULL, 0);
	if(ret < 0)
		goto out;

	if(!activity_thd)
		printk("mma activity debouncer disabled\n");

	pChip->activity_thd = activity_thd;
	ret = count;
out:
	mutex_unlock(&pDev->data_lock);
	return ret;
}

/*
* This method is used to get the pressure reading mode of the P/T ambient sensors
* filesys item: /sys/class/mma955x/mma955x/amb_p_mode
*/
ssize_t hub_amb_p_mode_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct mxc_mma_device_t *pDev = dev_get_drvdata(dev);
	return sprintf(buf, "%d\n", pDev->pChip->amb_p_mode);
}

/*
* This method is used to set the pressure reading mode of the P/T ambient sensors
* Accepted values : 0 (barometer), 1 (altimeter)
* filesys item: /sys/class/mma955x/mma955x/amb_p_mode
*/
ssize_t hub_amb_p_mode_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	ssize_t ret;
	u8 val;
	struct mxc_mma_device_t *pDev = dev_get_drvdata(dev);
	struct chipInfo_t *pChip = pDev->pChip;
	struct i2c_client *client = pChip->client;

	u8 cmd[6] = { HUB_APPID, CMD_WR_CFG, 0x02, 0x02, 0x01, 0};	/* this command re-init P/T ambient sensors with current config values */

	int amb_p_mode = simple_strtoul(buf, NULL, 10);

	switch(amb_p_mode)
	{
		case HUB_AMB_BAROMETER_MODE:	val = 0x38; /* RAW mode; barometer output mode; Oversampling x128 */
		break;
		case HUB_AMB_ALTIMETER_MODE:	val = 0xB8; /* RAW mode; altimeter output mode; Oversampling x128 */
		break;
		default:
		{
			printk("Invalid pressure mode selection (buf: %s count: %d)\n", buf, count);
			return -EINVAL;
		}

	}

	cmd[5] = val;

	mutex_lock(&pDev->data_lock);
	ret = pChip->SendCommand(client, cmd, sizeof(cmd), NULL, 0);
	if(ret < 0)
		goto out;

	pChip->amb_p_mode = amb_p_mode;
	printk("pressure mode set to: %d\n", pChip->amb_p_mode);

	ret = count;
out:
	mutex_unlock(&pDev->data_lock);
	return ret;
}

#if 1 /* for debugging */
/*
* This method is used to perform some utility task 
* Accepted values : 0,1..
* filesys item: /sys/class/mma955x/mma955x/utility
*/
extern int hub_getCfg(struct i2c_client *client, mmaHubCfg_t* p);
extern int hub_getSts(struct i2c_client *client, mmaHubSts_t* p);
extern int afe_getSts(struct i2c_client *client, mmaAfeSts_t *p);
ssize_t hub_utility_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct mxc_mma_device_t *pDev = dev_get_drvdata(dev);
	struct chipInfo_t *pChip = pDev->pChip;
	struct i2c_client *client = pChip->client;
	int utility = simple_strtoul(buf, NULL, 10);

	switch(utility)
	{
		case 0: printk("0: menu, 1: clear, 2: reset, 3: susp_on, 4: susp_off, 5: hubCfg, 6: hubSts, 7: afeSts\n");
		break;
		case 1: pChip->SendRSC(client, HUB_APPID, CLEAR, 0);	/* executes user_app_clear() callback func on mma955x */
		break;
		case 2: pChip->SendRSC(client, HUB_APPID, RESET, 0);	/* executes user_app_reset() callback func on mma955x */
		break;
		case 3: pChip->SendRSC(client, HUB_APPID, SUSPEND, 0);	/* enter suspend mode */
		break;
		case 4: pChip->SendRSC(client, HUB_APPID, SUSPEND, 1);	/* exit suspend mode */
		break;
		case 5: /* prints HUB configuration */
		{
			mmaHubCfg_t cfg;
			mutex_lock(&pDev->data_lock);
			hub_getCfg(client, &cfg);
			mutex_unlock(&pDev->data_lock);
		}
		break;
		case 6: /* prints HUB status */
		{
			mmaHubSts_t sts;
			mutex_lock(&pDev->data_lock);
			hub_getSts(client, &sts);
			mutex_unlock(&pDev->data_lock);
		}
		break;
		case 7: /* prints Analog-Front-End status */
		{
			mmaAfeSts_t sts;
			mutex_lock(&pDev->data_lock);
			afe_getSts(client, &sts);
			mutex_unlock(&pDev->data_lock);
		}
		break;
	}
	return count;
}
#endif

#if 0
/* Device model classes */
struct class sensor_class_obj = {
	.name        = "mma",
};
#endif

/*
* This method is used to initialize sysfs
* client : Pointer to i2c client structure
* return -EPERM : If device pointer is null
* return 0 : If class device successfully created
*/
int mmaInitializeSysfs(struct i2c_client *client)
{
	ssize_t ret = 0;
	int i = 0;
	int Iterator = 0;
	int Instance = 0;
	struct mxc_mma_device_t *pDev = i2c_get_clientdata(client);
	struct chipInfo_t *pChip = pDev->pChip;
	struct SysfsInfo_t *pSysfsInfo = pChip->pSysfsInfo;

	if(pDev == NULL)
	{
		printk("%s: pDev => NULL pointer\r\n", __func__);
		return -EPERM;
	}

//	pDev->class = class_create(THIS_MODULE, pDev->pChip->name);
	pDev->class = class_create(THIS_MODULE, pDev->devname);
	if (IS_ERR(pDev->class)) {
			printk(KERN_ERR "Unable to create sysfs class for mma\n");
			ret = PTR_ERR(pDev->class);
	}

	pDev->sys_device = device_create(pDev->class, NULL, MKDEV(pDev->major, 0), pDev, pDev->pChip->name);
  
	if (IS_ERR(pDev->sys_device)) {
		printk(KERN_ERR "Unable to create sysfs class device for mma\n");
		ret = PTR_ERR(pDev->sys_device);
		return ret;
	}

	dev_set_drvdata(pDev->sys_device, pDev);

	/* Create common entries */
	for(Iterator = 0; Iterator < pChip->SysfsInfoSize; Iterator++)
	{
		for(Instance = 0; Instance < pSysfsInfo[Iterator].Instance; Instance++)
		{		
			for(i=0; i < pSysfsInfo[Iterator].TotalEntries; i++)
			{
				if(sysfs_create_file(&pDev->sys_device->kobj, &pSysfsInfo[Iterator].AttrEntry[i].attr) < 0)
					printk("%s sys file creation failed.\r\n", pSysfsInfo[Iterator].AttrEntry[i].attr.name);
			}

		}
	}

	return ret;	
}

/*
* This method is used to de-initialize sysfs
* return -EPERM : If device pointer is null
* return 0 : If class device successfully destroyed
*/
int mmaDeInitializeSysfs(struct mxc_mma_device_t *pDev)
{
	if(pDev == NULL)
	{
		printk("%s: pDev => NULL pointer\r\n", __func__);
		return -EPERM;
	}
	device_destroy(pDev->class, MKDEV(pDev->major, 0));
	class_destroy(pDev->class);
	return 0;
}

