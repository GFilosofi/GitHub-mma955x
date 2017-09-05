/******************** (C) COPYRIGHT 2012 Freescale Semiconductor, Inc. *************
 *
 * File Name		: mma_sysfs.h
 * Authors		: Rick Zhang(rick.zhang@freescale.com)
 			  Rick is willing to be considered the contact and update points 
 			  for the driver
 * Version		: V.1.0.0
 * Date			: 2012/Mar/15
 * Description		: MAG3110  declarations and defines required mmanetometer sysfs entries
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
#include <linux/device.h>
#include <linux/i2c.h>
#include "mma955x_regs.h"
#include <mach/hardware.h>
#include "mma955x_io.h"

#define DECLARE_ATTR(_name, _mode, _show, _store)		\
{								\
	.attr   = { .name = __stringify(_name), .mode = _mode,	\
		    .owner = THIS_MODULE },  			\
	.show   = mma_##_show,					\
	.store  = mma_##_store,					\
}

/* MMA955x-common Sysfs info */
extern ssize_t mma_devid_show(struct device *dev, struct device_attribute *attr, char *buf);
extern ssize_t mma_name_show(struct device *dev, struct device_attribute *attr, char *buf);
extern ssize_t mma_vendor_show(struct device *dev, struct device_attribute *attr, char *buf);
extern ssize_t mma_version_show(struct device *dev, struct device_attribute *attr, char *buf);
extern ssize_t mma_sleep_show(struct device *dev, struct device_attribute *attr, char *buf);
extern ssize_t mma_sleep_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
extern ssize_t mma_poll_time_show(struct device *dev, struct device_attribute *attr, char *buf);
extern ssize_t mma_poll_time_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);

/* MMA9550-specific Sysfs info (hub) */
extern ssize_t hub_position_show(struct device *dev, struct device_attribute *attr, char *buf);
extern ssize_t hub_position_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
extern ssize_t hub_acc_invert_show(struct device *dev, struct device_attribute *attr, char *buf);
extern ssize_t hub_acc_invert_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
extern ssize_t hub_gyro_invert_show(struct device *dev, struct device_attribute *attr, char *buf);
extern ssize_t hub_gyro_invert_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
extern ssize_t hub_acc_output_stage_show(struct device *dev, struct device_attribute *attr, char *buf);
extern ssize_t hub_acc_output_stage_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
extern ssize_t hub_acc_offset_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
extern ssize_t hub_acc_offset_show(struct device *dev, struct device_attribute *attr, char *buf);
extern ssize_t hub_acc_range_show(struct device *dev, struct device_attribute *attr, char *buf);
extern ssize_t hub_acc_range_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
extern ssize_t hub_resolutions_show(struct device *dev, struct device_attribute *attr, char *buf);
extern ssize_t hub_resolution_show(struct device *dev, struct device_attribute *attr, char *buf);
extern ssize_t hub_resolution_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
extern ssize_t hub_amb_p_mode_show(struct device *dev, struct device_attribute *attr, char *buf);
extern ssize_t hub_amb_p_mode_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
extern ssize_t hub_gyro_range_show(struct device *dev, struct device_attribute *attr, char *buf);
extern ssize_t hub_gyro_range_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
extern ssize_t hub_amb_range_show(struct device *dev, struct device_attribute *attr, char *buf);
extern ssize_t hub_amb_range_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
extern ssize_t hub_amb_offset_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
extern ssize_t hub_amb_offset_show(struct device *dev, struct device_attribute *attr, char *buf);
extern ssize_t hub_utility_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);

/* MMA9553-specific Sysfs info (pedometer) */
extern ssize_t ped_sleep_min_show(struct device *dev, struct device_attribute *attr, char *buf);
extern ssize_t ped_sleep_min_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
extern ssize_t ped_sleep_max_show(struct device *dev, struct device_attribute *attr, char *buf);
extern ssize_t ped_sleep_max_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
extern ssize_t ped_sleep_thd_show(struct device *dev, struct device_attribute *attr, char *buf);
extern ssize_t ped_sleep_thd_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
extern ssize_t ped_steplen_show(struct device *dev, struct device_attribute *attr, char *buf);
extern ssize_t ped_steplen_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
extern ssize_t ped_height_show(struct device *dev, struct device_attribute *attr, char *buf);
extern ssize_t ped_height_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
extern ssize_t ped_weight_show(struct device *dev, struct device_attribute *attr, char *buf);
extern ssize_t ped_weight_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
extern ssize_t ped_filter_step_show(struct device *dev, struct device_attribute *attr, char *buf);
extern ssize_t ped_filter_step_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
extern ssize_t ped_gender_show(struct device *dev, struct device_attribute *attr, char *buf);
extern ssize_t ped_gender_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
extern ssize_t ped_filter_time_show(struct device *dev, struct device_attribute *attr, char *buf);
extern ssize_t ped_filter_time_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
extern ssize_t ped_speed_period_show(struct device *dev, struct device_attribute *attr, char *buf);
extern ssize_t ped_speed_period_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
extern ssize_t ped_step_coalesce_show(struct device *dev, struct device_attribute *attr, char *buf);
extern ssize_t ped_step_coalesce_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
extern ssize_t ped_activity_thd_show(struct device *dev, struct device_attribute *attr, char *buf);
extern ssize_t ped_activity_thd_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);


