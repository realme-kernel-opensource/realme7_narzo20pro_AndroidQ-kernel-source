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

#ifndef __MTK_PD_INTF_H
#define __MTK_PD_INTF_H

#include "adapter_class.h"

#ifdef ODM_HQ_EDIT
/* wangtao@BSP.CHG.Basic, 2019/11/28, sjc Add for PD */
#define VBUS_5V        5000    //5v
#define VBUS_9V        9000    //9v

#define IBUS_2A        2000   //2A
#define IBUS_3A        3000   //3A
#endif

/* PD charging */
struct mtk_pdc {
	struct tcpc_device *tcpc;
	struct adapter_power_cap cap;
	int pdc_max_watt;
	int pdc_max_watt_setting;

	bool check_impedance;
	int pd_cap_max_watt;
	int pd_idx;
	int pd_reset_idx;
	int pd_boost_idx;
	int pd_buck_idx;
	int vbus_l;
	int vbus_h;

	struct mutex access_lock;
	struct mutex pmic_sync_lock;
	struct wakeup_source suspend_lock;
	int ta_vchr_org;
	bool to_check_chr_type;
	bool to_tune_ta_vchr;
	bool is_cable_out_occur;
	bool is_connect;
	bool is_enabled;
};

extern bool mtk_pdc_check_charger(struct charger_manager *info);
extern void mtk_pdc_plugout_reset(struct charger_manager *info);
extern void mtk_pdc_set_max_watt(struct charger_manager *info, int watt);
extern int mtk_pdc_get_max_watt(struct charger_manager *info);
extern int mtk_pdc_get_setting(struct charger_manager *info, int *vbus,
				int *cur, int *idx);
extern void mtk_pdc_init_table(struct charger_manager *info);
extern bool mtk_pdc_init(struct charger_manager *info);
extern int mtk_pdc_setup(struct charger_manager *info, int idx);
extern void mtk_pdc_plugout(struct charger_manager *info);
extern void mtk_pdc_check_cable_impedance(struct charger_manager *info);
extern void mtk_pdc_reset(struct charger_manager *info);
extern bool mtk_pdc_check_leave(struct charger_manager *info);

#ifdef CONFIG_OPPO_HQ_EULER_CHARGER
//Junbo.Guo@ODM_WT.BSP.CHG, 2019/12/9, Modify for PD
extern int oppo_pdc_setup(struct charger_manager *info,int vbus);

#else

#ifdef ODM_HQ_EDIT
/* wangtao@BSP.CHG.Basic, 2019/11/28, sjc Add for PD */
extern int oppo_pdc_setup(int *vbus_mv, int *ibus_ma);
extern int oppo_pdc_get(int *vbus_mv, int *ibus_ma);
#endif
#endif

#ifdef CONFIG_MTK_PUMP_EXPRESS_PLUS_SUPPORT



#else /* NOT CONFIG_MTK_PUMP_EXPRESS_PLUS_SUPPORT */


#endif /* CONFIG_MTK_PUMP_EXPRESS_PLUS_SUPPORT */


#endif /* __MTK_PD_INTF_H */
