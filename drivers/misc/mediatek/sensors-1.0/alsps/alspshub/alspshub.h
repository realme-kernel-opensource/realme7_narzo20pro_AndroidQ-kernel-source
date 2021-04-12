/*
 * Copyright (C) 2016 MediaTek Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See http://www.gnu.org/licenses/gpl-2.0.html for more details.
 */

/*
 * Definitions for ALSPS als/ps sensor chip.
 */
#ifndef __ALSPSHUB_H__
#define __ALSPSHUB_H__

#include <linux/ioctl.h>


/*ALSPS related driver tag macro*/
#define ALSPS_SUCCESS						0
#define ALSPS_ERR_I2C						-1
#define ALSPS_ERR_STATUS					-3
#define ALSPS_ERR_SETUP_FAILURE				-4
#define ALSPS_ERR_GETGSENSORDATA			-5
#define ALSPS_ERR_IDENTIFICATION			-6

/*----------------------------------------------------------------------------*/
enum ALSPS_NOTIFY_TYPE {
	ALSPS_NOTIFY_PROXIMITY_CHANGE = 0,
};

/* zhoujunwei@ODM_HQ.BSP.Sensors.Config, 2020/04/03, sync sensor data */
#if defined(ODM_HQ_EDIT) && !defined(TARGET_WATERMELON_Q_PROJECT)
/* zuoqiquan@ODM_HQ.BSP.Sensors.Config, 2019/12/24, Add for als data filter */
#define ALS_FIR_LEN 5
struct als_data_filter
{
	unsigned int raw[ALS_FIR_LEN];
	unsigned int number;
	unsigned int idx;
};
#endif /*ODM_HQ_EDIT*/

#endif

