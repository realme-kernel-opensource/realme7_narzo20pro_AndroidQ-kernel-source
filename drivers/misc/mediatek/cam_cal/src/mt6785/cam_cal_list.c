/*
 * Copyright (C) 2018 MediaTek Inc.
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
#include <linux/kernel.h>
#include "cam_cal_list.h"
#include "eeprom_i2c_common_driver.h"
#include "eeprom_i2c_custom_driver.h"
#include "kd_imgsensor.h"
#ifdef ODM_HQ_EDIT
/* fengbin@ODM.Camera.Drv 20191024 for 3p9 crosstalk data*/
#include "P24C64E/P24C64E.h"
#endif

#define IMX586_MAX_EEPROM_SIZE 0x24D0
#ifdef ODM_HQ_EDIT
/* Houbing.Peng@ODM.Camera.Drv  20190910 OTP Bringup*/
#define OV8856_MAX_EEPROM_SIZE_16K 0x4000
#endif

#ifdef ODM_HQ_EDIT
/* Degao.Lan@ODM.Camera.Drv  20191218 OTP Bringup*/
#define S5KGM1SP_MAX_EEPROM_SIZE_16K 0x4000
#endif

#ifdef ODM_HQ_EDIT
/* fengbin@ODM.Camera.Drv 20191024 for 3p9 crosstalk data*/
struct stCAM_CAL_FUNC_STRUCT g_camCalCMDFunc[] = {
        {S5K3P9SP_SENSOR_ID, p24c64e_selective_read_region},
        /*      ADD before this line */
        {0, 0} /*end of list*/
};
#endif

struct stCAM_CAL_LIST_STRUCT g_camCalList[] = {
	/*Below is commom sensor */
#ifdef ODM_HQ_EDIT
/* Houbing.Peng@ODM.Camera.Drv 20190910 for OTP Bringup*/
	{IMX682_SENSOR_ID, 0xA0, Common_read_region},
	{S5KGW1_SENSOR_ID, 0xA0, Common_read_region},
	{S5KGM1SP_SENSOR_ID, 0xA0, Common_read_region, S5KGM1SP_MAX_EEPROM_SIZE_16K},
	{IMX471_SENSOR_ID, 0xA8, Common_read_region},
	{S5K3P9SP_SENSOR_ID, 0xA8, Common_read_region},
	{OV8856_SENSOR_ID, 0xA2, Common_read_region, OV8856_MAX_EEPROM_SIZE_16K},
	{OV02B10_SENSOR_ID, 0xA4, Common_read_region},
	{GC2375H_SENSOR_ID, 0xA4, Common_read_region},
	{GC02K0_SENSOR_ID, 0xA4, Common_read_region},
#else
	{IMX519_SENSOR_ID, 0xA0, Common_read_region},
	{S5K2T7SP_SENSOR_ID, 0xA4, Common_read_region},
	{S5K2LQSX_SENSOR_ID, 0xA0, Common_read_region},
	{S5K4H7_SENSOR_ID, 0xA2, Common_read_region},
	{IMX386_SENSOR_ID, 0xA0, Common_read_region},
	{S5K2L7_SENSOR_ID, 0xA0, Common_read_region},
	{IMX398_SENSOR_ID, 0xA0, Common_read_region},
	{IMX350_SENSOR_ID, 0xA0, Common_read_region},
	{IMX386_MONO_SENSOR_ID, 0xA0, Common_read_region},
	{IMX586_SENSOR_ID, 0xA0, Common_read_region, IMX586_MAX_EEPROM_SIZE},
	{IMX499_SENSOR_ID, 0xA0, Common_read_region},
#endif

	/*  ADD before this line */
	{0, 0, 0}       /*end of list */
};

unsigned int cam_cal_get_sensor_list(
	struct stCAM_CAL_LIST_STRUCT **ppCamcalList)
{
	if (ppCamcalList == NULL)
		return 1;

	*ppCamcalList = &g_camCalList[0];
	return 0;
}

#ifdef ODM_HQ_EDIT
/* fengbin@ODM.Camera.Drv 20191024 for 3p9 crosstalk data*/
unsigned int cam_cal_get_func_list(
     struct stCAM_CAL_FUNC_STRUCT **ppCamcalFuncList)
     {
         if (ppCamcalFuncList == NULL)
                return 1;
         *ppCamcalFuncList = &g_camCalCMDFunc[0];
         return 0;
     }
#endif
