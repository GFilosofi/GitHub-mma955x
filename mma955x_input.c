/******************** (C) COPYRIGHT 2012 Cosmed, Ltd. *************
* Copyright (C) 2012 Cosmed, Ltd. <http://www.cosmed.com/>
*
* File Name		: mma_input.c
* Authors		: Gabriele Filosofi <gabrielef@cosmed.it>
			  Gabriele is willing to be considered the contact and update points
			  for the driver
* Version		: V.1.0.0
* Date			: 2012/Sept/20
* Description		: input interface implementation for Freescale 3-Axis Intelligent
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
#include "mma955x_regs.h"
#include <mach/hardware.h>
#include <cosmed/mma955x.h>
#include "mma955x_io.h"

#define	FUZZ		0
#define	FLAT		0

/* input define mappings for Barometer/Altimeter & internal Termometer */
#define ABS_PR		ABS_PRESSURE
#define ABS_TEMP	ABS_MISC

/* input define mappings for ambient Relative Humidity & Temperature */
//#define ABS_AMBTEMP	??
#define ABS_AMBRH	ABS_GAS

/* input define mappings for Pedometer */
#define ABS_DIST	ABS_RX
#define ABS_SPEED	ABS_RY
#define ABS_SLEEPCNT	ABS_RZ
#define ABS_STATUS	ABS_PRESSURE
#define ABS_STEPCNT	ABS_MISC

/*
* This method is used to initialize input interface. 
* pDev 	: Pointer to device structure.
* return 0 : If device is successfully registered.	
*/
int mmaInitializeInputInterface(struct mxc_mma_device_t *pDev)
{
	int ret = 0;
	
	if(pDev == NULL)
	{
		printk("%s: pDev => NULL pointer\r\n", __func__);
		return -EPERM;
	}

	pDev->inp = input_allocate_device();

	if (!pDev->inp) {
		ret = -ENOMEM;
		printk(KERN_ERR "%s: Failed to allocate input device-1\n", __func__);
		return ret;
	}

	switch(pDev->pChip->chipId)
	{
		case MMA9550_ID:
		{
			set_bit(EV_ABS, pDev->inp->evbit);	// Accelerometer readings
			input_set_abs_params(pDev->inp, ABS_X, -32768, 32767, FUZZ, FLAT);	/* x-axis acceleration Gx */
			input_set_abs_params(pDev->inp, ABS_Y, -32768, 32767, FUZZ, FLAT);	/* y-axis acceleration Gy */
			input_set_abs_params(pDev->inp, ABS_Z, -32768, 32767, FUZZ, FLAT);	/* z-axis acceleration Gz */

			input_set_abs_params(pDev->inp, ABS_RX, -32768, 32767, FUZZ, FLAT);	/* x-axis angular rate Ox */
			input_set_abs_params(pDev->inp, ABS_RY, -32768, 32767, FUZZ, FLAT);	/* y-axis angular rate Oy */
			input_set_abs_params(pDev->inp, ABS_RZ, -32768, 32767, FUZZ, FLAT);	/* z-axis angular rate Oz */

			/*
			Pressure data is 20-bit unsigned value in .25Pa units (the data is stored as Pa with bits 19-2 and with fractions of a Pa in bits 1-0). Range is -131072 to 131071.75 Pa
			Altitude data is 20-bit 2’s compl. value in .0625m units (the data is stored as meters with bits 19-4 and with fractions of a meter in bits 3-0). Range is -32768 to 32767.9375 m
			Temperature data is 12-bit 2’s compl. value in .0625°C units. The data is stored as degrees °C with bits 11-4 and with fractions of a degree °C in bits 3-0. Range is -128C to 127.9375 °C 
			*/
			input_set_abs_params(pDev->inp, ABS_PR, -524288, 524287, FUZZ, FLAT);	/* barometric pressure/altimeter */
			input_set_abs_params(pDev->inp, ABS_TEMP, -2048, 2047, FUZZ, FLAT);	/* Temperature */
		}
		break;
		case MMA9553_ID:
		{
			set_bit(EV_ABS, pDev->inp->evbit);	// Pedometer readings
			input_set_abs_params(pDev->inp, ABS_X, -32768, 32767, FUZZ, FLAT);	/* x-axis acceleration Gx */
			input_set_abs_params(pDev->inp, ABS_Y, -32768, 32767, FUZZ, FLAT);	/* y-axis acceleration Gy */
			input_set_abs_params(pDev->inp, ABS_Z, -32768, 32767, FUZZ, FLAT);	/* z-axis acceleration Gz */

			input_set_abs_params(pDev->inp, ABS_DIST, -32768, 32767, FUZZ, FLAT);		/* distance in m */ 
			input_set_abs_params(pDev->inp, ABS_SPEED, -32768, 32767, FUZZ, FLAT);		/* average speed in m/hour over a speed_period */
			input_set_abs_params(pDev->inp, ABS_SLEEPCNT, -32768, 32767, FUZZ, FLAT);	/* current value of the auto-suspend debounce counter */
			input_set_abs_params(pDev->inp, ABS_STATUS, -32768, 32767, FUZZ, FLAT);		/* status */
			input_set_abs_params(pDev->inp, ABS_STEPCNT, -32768, 32767, FUZZ, FLAT);	/* step count */
		}
		break;
		default:
		{
			printk(KERN_ERR "%s: Unhandled device id: %s\n", __func__, pDev->inp->name);
			return -1;
		}
		break;
	}

	pDev->inp->name = "mma955x";
	pDev->inp->uniq = pDev->pChip->name;
	pDev->inp->id.bustype = BUS_I2C;
	pDev->inp->id.vendor = 0x1211;
	pDev->inp->id.version = 1;

	ret = input_register_device(pDev->inp);
	if (ret) {
		printk(KERN_ERR "%s: Unable to register input device: %s\n", __func__, pDev->inp->name);
		return ret;
	}

	return ret;	
}

/*
* This method is used to deinitialize input interface. 
* pDev : Pointer to device structure.
* return 0 : If device is successfully deinitialized.	
*/
int mmaDeInitializeInputInterface(struct mxc_mma_device_t *pDev)
{
	if(pDev == NULL)
	{
		printk("%s: pDev => NULL pointer\r\n", __func__);
		return -EPERM;
	}

	if(pDev->inp)
	{
		input_unregister_device(pDev->inp);
		input_free_device(pDev->inp);
	}
	return 0;
}

/*
* This method is used to report event to upper layer. 
* pDev : Pointer to device structure.
* pBuf : Pointer to buffer containing input data.
* return 0 : If device is successfully deinitialized.	
*/
int mmaReportEvent(struct mxc_mma_device_t *pDev, void *pBuf)
{
	if(pDev == NULL)
		return -ENOMEM;

	switch(pDev->pChip->chipId)
	{
		case MMA9550_ID:
		{
			pHubData_t pData = (pHubData_t)pBuf;
			if(pDev->inp && pData)
			{
				input_report_abs(pDev->inp, ABS_X, pData->G.x);		/* x-axis acceleration Gx */
				input_report_abs(pDev->inp, ABS_Y, pData->G.y);		/* y-axis acceleration Gy */
				input_report_abs(pDev->inp, ABS_Z, pData->G.z);		/* z-axis acceleration Gz */

				input_report_abs(pDev->inp, ABS_RX, pData->O.x);	/* x-axis angular rate Ox */
				input_report_abs(pDev->inp, ABS_RY, pData->O.y);	/* y-axis angular rate Oy */
				input_report_abs(pDev->inp, ABS_RZ, pData->O.z);	/* z-axis angular rate Oz */

				input_report_abs(pDev->inp, ABS_PR, pData->P);		/* barometric pressure/altimeter */
				input_report_abs(pDev->inp, ABS_TEMP, pData->T);	/* temperature */

				input_sync(pDev->inp);
#ifdef DEBUG
				printk("X: [%04x] Y:[%04x] Z: [%04x]\r\n", pData->G.x, pData->G.y, pData->G.z);
#endif
			}
		}
		break;
		case MMA9553_ID:
		{
			pPedData_t pData = (pPedData_t)pBuf;
			if(pDev->inp && pData)
			{
				input_report_abs(pDev->inp, ABS_X, pData->G.x);		/* x-axis acceleration Gx */
				input_report_abs(pDev->inp, ABS_Y, pData->G.y);		/* y-axis acceleration Gy */
				input_report_abs(pDev->inp, ABS_Z, pData->G.z);		/* z-axis acceleration Gz */

				input_report_abs(pDev->inp, ABS_RX, pData->distance);	/* distance in m */
				input_report_abs(pDev->inp, ABS_RY, pData->speed);	/* average speed in m/hour over a speed_period */
				input_report_abs(pDev->inp, ABS_RZ, pData->sleepCnt);	/* current value of the auto-suspend debounce counter */

				input_report_abs(pDev->inp, ABS_PR, pData->status);	/* status */
				input_report_abs(pDev->inp, ABS_TEMP, pData->stepCnt);	/* step count */

				input_sync(pDev->inp);
#ifdef DEBUG
				printk("X: [%04x] Y:[%04x] Z: [%04x]\r\n", pData->G.x, pData->G.y, pData->G.z);
				printk("stepconut: [%04x] status:[%04x] speed: [%04x]\r\n", pData->stepCnt, pData->status, pData->speed);
#endif
			}
		}
		break;
		default:
			printk("%s:: Unhandled device id\r\n", __func__);
			break;
	}
	return 0;
}
