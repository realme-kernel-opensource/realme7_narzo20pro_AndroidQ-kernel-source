/*
 * Copyright (C) 2017 MediaTek Inc.
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

#include <linux/delay.h>
#include <linux/string.h>

#include "kd_camera_typedef.h"
#include "kd_camera_feature.h"

#include "imgsensor_sensor.h"
#include "imgsensor_hw.h"
#ifdef ODM_HQ_EDIT
/* Houbing.Peng@ODM_HQ Cam.Drv 20191112 add for change BUCK_VS2 vol from 1.35v to 1.4v */
#include <mt-plat/upmu_common.h>
/*Houbing.Peng@ODM 20200416 add for sala bringup*/
#include <soc/oppo/oppo_project.h>
#endif
enum IMGSENSOR_RETURN imgsensor_hw_init(struct IMGSENSOR_HW *phw)
{
	struct IMGSENSOR_HW_SENSOR_POWER      *psensor_pwr;
	struct IMGSENSOR_HW_CFG               *pcust_pwr_cfg;
	struct IMGSENSOR_HW_CUSTOM_POWER_INFO *ppwr_info;
	int i, j;
	char str_prop_name[LENGTH_FOR_SNPRINTF];
	struct device_node *of_node
		= of_find_compatible_node(NULL, NULL, "mediatek,imgsensor");

	mutex_init(&phw->common.pinctrl_mutex);

	for (i = 0; i < IMGSENSOR_HW_ID_MAX_NUM; i++) {
		if (hw_open[i] != NULL)
			(hw_open[i]) (&phw->pdev[i]);

		if (phw->pdev[i]->init != NULL)
			(phw->pdev[i]->init)(
				phw->pdev[i]->pinstance, &phw->common);
	}

	for (i = 0; i < IMGSENSOR_SENSOR_IDX_MAX_NUM; i++) {
		psensor_pwr = &phw->sensor_pwr[i];

		#ifdef CONFIG_MACH_MT6785
		/*Houbing.Peng@ODM 20200416 add for sala bringup*/
        /*Chejian@ODM_HQ Cam.Drv 20201112 for sala3*/
		if (is_project(OPPO_20682)) {
			pcust_pwr_cfg = imgsensor_custom_config_20682;
            if (get_Operator_Version() >= 90 && get_Operator_Version() <= 93){
                pcust_pwr_cfg = imgsensor_custom_config_SALA3;
            }
		} else {
			pcust_pwr_cfg = imgsensor_custom_config;
		}
		#else
		pcust_pwr_cfg = imgsensor_custom_config;
		#endif
		while (pcust_pwr_cfg->sensor_idx != i &&
		       pcust_pwr_cfg->sensor_idx != IMGSENSOR_SENSOR_IDX_NONE)
			pcust_pwr_cfg++;

		if (pcust_pwr_cfg->sensor_idx == IMGSENSOR_SENSOR_IDX_NONE)
			continue;

		ppwr_info = pcust_pwr_cfg->pwr_info;
		while (ppwr_info->pin != IMGSENSOR_HW_PIN_NONE) {
			for (j = 0;
				j < IMGSENSOR_HW_ID_MAX_NUM &&
					ppwr_info->id != phw->pdev[j]->id;
				j++) {
			}

			psensor_pwr->id[ppwr_info->pin] = j;
			ppwr_info++;
		}
	}

	for (i = 0; i < IMGSENSOR_SENSOR_IDX_MAX_NUM; i++) {
		memset(str_prop_name, 0, sizeof(str_prop_name));
		snprintf(str_prop_name,
					sizeof(str_prop_name),
					"cam%d_%s",
					i,
					"enable_sensor");
		if (of_property_read_string(
			of_node, str_prop_name,
			&phw->enable_sensor_by_index[i]) < 0) {
			PK_DBG("Property cust-sensor not defined\n");
			phw->enable_sensor_by_index[i] = NULL;
		}
	}

	return IMGSENSOR_RETURN_SUCCESS;
}

enum IMGSENSOR_RETURN imgsensor_hw_release_all(struct IMGSENSOR_HW *phw)
{
	int i;

	for (i = 0; i < IMGSENSOR_HW_ID_MAX_NUM; i++) {
		if (phw->pdev[i]->release != NULL)
			(phw->pdev[i]->release)(phw->pdev[i]->pinstance);
	}
	return IMGSENSOR_RETURN_SUCCESS;
}

static enum IMGSENSOR_RETURN imgsensor_hw_power_sequence(
		struct IMGSENSOR_HW             *phw,
		enum   IMGSENSOR_SENSOR_IDX      sensor_idx,
		enum   IMGSENSOR_HW_POWER_STATUS pwr_status,
		struct IMGSENSOR_HW_POWER_SEQ   *ppower_sequence,
		char *pcurr_idx)
{
	struct IMGSENSOR_HW_SENSOR_POWER *psensor_pwr =
					&phw->sensor_pwr[sensor_idx];
	struct IMGSENSOR_HW_POWER_SEQ    *ppwr_seq = ppower_sequence;
	struct IMGSENSOR_HW_POWER_INFO   *ppwr_info;
	struct IMGSENSOR_HW_DEVICE       *pdev;
	int                               pin_cnt = 0;

	static DEFINE_RATELIMIT_STATE(ratelimit, 1 * HZ, 30);

#ifdef CONFIG_FPGA_EARLY_PORTING  /*for FPGA*/
	if (1) {
		PK_DBG("FPGA return true for power control\n");
		return IMGSENSOR_RETURN_SUCCESS;
	}
#endif


#ifdef ODM_HQ_EDIT
/* Lijian@ODM.Camera.Drv 20190912 MTK patch for mipi switch */
	while (ppwr_seq < ppower_sequence + IMGSENSOR_HW_SENSOR_MAX_NUM &&
		ppwr_seq->name != NULL) {
		if (!strcmp(ppwr_seq->name, PLATFORM_POWER_SEQ_NAME)) {
			if (sensor_idx == ppwr_seq->_idx)
				break;
		} else {
			if (!strcmp(ppwr_seq->name, pcurr_idx))
				break;
		}
		ppwr_seq++;
	}

	if (ppwr_seq->name == NULL)
#else
	while (ppwr_seq->name != NULL &&
		ppwr_seq < ppower_sequence + IMGSENSOR_HW_SENSOR_MAX_NUM &&
		strcmp(ppwr_seq->name, pcurr_idx)) {
		ppwr_seq++;
	}

	if (ppwr_seq->name == NULL)
#endif
		return IMGSENSOR_RETURN_ERROR;

	ppwr_info = ppwr_seq->pwr_info;

	while (ppwr_info->pin != IMGSENSOR_HW_PIN_NONE &&
	       ppwr_info < ppwr_seq->pwr_info + IMGSENSOR_HW_POWER_INFO_MAX) {

		if (pwr_status == IMGSENSOR_HW_POWER_STATUS_ON) {
			if (ppwr_info->pin != IMGSENSOR_HW_PIN_UNDEF) {
				pdev =
				phw->pdev[psensor_pwr->id[ppwr_info->pin]];
                if (__ratelimit(&ratelimit))
					PK_DBG(
					"sensor_idx %d, ppwr_info->pin %d, ppwr_info->pin_state_on %d",
					sensor_idx,
					ppwr_info->pin,
					ppwr_info->pin_state_on);
				#ifdef ODM_HQ_EDIT
				/* Houbing.Peng@ODM_HQ Cam.Drv 20191112 add for enable VS2 force pwm mode */
				if (sensor_idx == 0 && ppwr_info->pin == IMGSENSOR_HW_PIN_MCLK) {
					//pmic_config_interface(PMIC_RG_BUCK_VS2_VOTER_EN_SET_ADDR, 0x1, PMIC_RG_BUCK_VS2_VOTER_EN_SET_MASK, PMIC_RG_BUCK_VS2_VOTER_EN_SET_SHIFT);
					pmic_config_interface(PMIC_RG_VS2_FPWM_ADDR, 0x1, PMIC_RG_VS2_FPWM_MASK, PMIC_RG_VS2_FPWM_SHIFT);
				}
				#endif
				if (pdev->set != NULL)
					pdev->set(pdev->pinstance,
					sensor_idx,
				    ppwr_info->pin, ppwr_info->pin_state_on);
			}

			mdelay(ppwr_info->pin_on_delay);
		}

		ppwr_info++;
		pin_cnt++;
	}

	if (pwr_status == IMGSENSOR_HW_POWER_STATUS_OFF) {
		while (pin_cnt) {
			ppwr_info--;
			pin_cnt--;

			if (__ratelimit(&ratelimit))
				PK_DBG(
				"sensor_idx %d, ppwr_info->pin %d, ppwr_info->pin_state_off %d",
				sensor_idx,
				ppwr_info->pin,
				ppwr_info->pin_state_off);

			if (ppwr_info->pin != IMGSENSOR_HW_PIN_UNDEF) {
				pdev =
				phw->pdev[psensor_pwr->id[ppwr_info->pin]];
				#ifdef ODM_HQ_EDIT
				/* Houbing.Peng@ODM_HQ Cam.Drv 20191112 add for enable VS2 force pwm mode */
				if (sensor_idx == 0 && ppwr_info->pin == IMGSENSOR_HW_PIN_MCLK) {
					//pmic_config_interface(PMIC_RG_BUCK_VS2_VOTER_EN_CLR_ADDR, 0x1, PMIC_RG_BUCK_VS2_VOTER_EN_CLR_MASK, PMIC_RG_BUCK_VS2_VOTER_EN_CLR_SHIFT);
					pmic_config_interface(PMIC_RG_VS2_FPWM_ADDR, 0x0, PMIC_RG_VS2_FPWM_MASK, PMIC_RG_VS2_FPWM_SHIFT);
				}
				#endif
				if (pdev->set != NULL)
					pdev->set(pdev->pinstance,
					sensor_idx,
				ppwr_info->pin, ppwr_info->pin_state_off);
			}

			mdelay(ppwr_info->pin_on_delay);
		}
	}

	return IMGSENSOR_RETURN_SUCCESS;
}

enum IMGSENSOR_RETURN imgsensor_hw_power(
		struct IMGSENSOR_HW *phw,
		struct IMGSENSOR_SENSOR *psensor,
		enum IMGSENSOR_HW_POWER_STATUS pwr_status)
{
	enum IMGSENSOR_SENSOR_IDX sensor_idx = psensor->inst.sensor_idx;
	char *curr_sensor_name = psensor->inst.psensor_list->name;
	char str_index[LENGTH_FOR_SNPRINTF];

#ifndef ODM_HQ_EDIT
/* Lijian@ODM.Camera.Drv 20190827 for snesor bringup */
	printk("sensor_idx %d, power %d curr_sensor_name %s, enable list %s\n",
		sensor_idx,
		pwr_status,
		curr_sensor_name,
		phw->enable_sensor_by_index[sensor_idx] == NULL
		? "NULL"
		: phw->enable_sensor_by_index[sensor_idx]);
#endif

	if (phw->enable_sensor_by_index[sensor_idx] &&
	!strstr(phw->enable_sensor_by_index[sensor_idx], curr_sensor_name))
		return IMGSENSOR_RETURN_ERROR;

#ifdef ODM_HQ_EDIT
/* Lijian@ODM.Camera.Drv 20190827 for snesor bringup */
	printk("sensor_idx %d, power %d curr_sensor_name %s, enable list %s\n",
		sensor_idx,
		pwr_status,
		curr_sensor_name,
		phw->enable_sensor_by_index[sensor_idx] == NULL
		? "NULL"
		: phw->enable_sensor_by_index[sensor_idx]);
#endif
	snprintf(str_index, sizeof(str_index), "%d", sensor_idx);
	imgsensor_hw_power_sequence(
			phw,
			sensor_idx,
			pwr_status,
			platform_power_sequence,
			str_index);

	#ifdef CONFIG_MACH_MT6785
	/*Houbing.Peng@ODM 20200416 add for sala bringup*/
	if (is_project(OPPO_20682)) {
		imgsensor_hw_power_sequence(
				phw,
				sensor_idx,
				pwr_status, sensor_power_sequence_20682, curr_sensor_name);
	} else {
		imgsensor_hw_power_sequence(
				phw,
				sensor_idx,
				pwr_status, sensor_power_sequence, curr_sensor_name);
	}
	#else
	imgsensor_hw_power_sequence(
			phw,
			sensor_idx,
			pwr_status, sensor_power_sequence, curr_sensor_name);
	#endif

	return IMGSENSOR_RETURN_SUCCESS;
}

enum IMGSENSOR_RETURN imgsensor_hw_dump(struct IMGSENSOR_HW *phw)
{
	int i;

	for (i = 0; i < IMGSENSOR_HW_ID_MAX_NUM; i++) {
		if (phw->pdev[i]->dump != NULL)
			(phw->pdev[i]->dump)(phw->pdev[i]->pinstance);
	}
	return IMGSENSOR_RETURN_SUCCESS;
}

