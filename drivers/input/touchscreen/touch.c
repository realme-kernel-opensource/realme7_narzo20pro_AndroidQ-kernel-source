/***************************************************
 * File:touch.c
 * VENDOR_EDIT
 * Copyright (c)  2008- 2030  Oppo Mobile communication Corp.ltd.
 * Description:
 *             tp dev
 * Version:1.0:
 * Date created:2016/09/02
 * Author: hao.wang@Bsp.Driver
 * TAG: BSP.TP.Init
*/

#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/input.h>
#include <linux/serio.h>
#include "oppo_touchscreen/tp_devices.h"
#include "oppo_touchscreen/touchpanel_common.h"
#include <soc/oppo/oppo_project.h>
#include <soc/oppo/device_info.h>
#include "touch.h"

#define MAX_LIMIT_DATA_LENGTH         100

extern char *saved_command_line;

int g_tp_dev_vendor = TP_UNKNOWN;

/*if can not compile success, please update vendor/oppo_touchsreen*/
struct tp_dev_name tp_dev_names[] = {
    {TP_OFILM, "OFILM"},
    {TP_BIEL, "BIEL"},
    {TP_TRULY, "TRULY"},
    {TP_BOE, "BOE"},
    {TP_G2Y, "G2Y"},
    {TP_TPK, "TPK"},
    {TP_JDI, "JDI"},
    {TP_TIANMA, "TIANMA"},
    {TP_SAMSUNG, "SAMSUNG"},
    {TP_DSJM, "DSJM"},
    {TP_BOE_B8, "BOEB8"},
    {TP_TM, "TM"},
    {TP_INX, "INX"},
    {TP_UNKNOWN, "UNKNOWN"},
};

//int g_tp_dev_vendor = TP_UNKNOWN;
typedef enum {
    TP_INDEX_NULL,
    himax_83112a,
    ili9881_auo,
    ili9881_tm,
    nt36525b_boe,
    nt36525b_hlt,
    nt36672c,
    ili9881_inx,
    ili9881h_boe,
    nt36525b_inx
} TP_USED_INDEX;
TP_USED_INDEX tp_used_index  = TP_INDEX_NULL;


#define GET_TP_DEV_NAME(tp_type) ((tp_dev_names[tp_type].type == (tp_type))?tp_dev_names[tp_type].name:"UNMATCH")

bool __init tp_judge_ic_match(char *tp_ic_name)
{
    pr_err("[TP] tp_ic_name = %s \n", tp_ic_name);
	
    //pr_err("[TP] boot_command_line = %s \n", saved_command_line);
	switch(   get_project()	) {
    case 19101:
        if (strstr(tp_ic_name, "synaptics-s3908") && (strstr(saved_command_line, "mdss_dsi_oppo19101boe_nt37800_1080_2400_cmd")
            || strstr(saved_command_line, "mdss_dsi_oppo19101boe_nt37800_1080_2340_cmd"))) {
            g_tp_dev_vendor = TP_BOE;
            return true;
        }
        if (strstr(tp_ic_name, "synaptics-s3706") && (strstr(saved_command_line, "mdss_dsi_oppo19125samsung_s6e3fc2x01_1080_2340_cmd")
            || strstr(saved_command_line, "mdss_dsi_oppo19125samsung_ams641rw01_1080_2340_cmd"))) {
            g_tp_dev_vendor = TP_SAMSUNG;
            return true;
        }
        if (strstr(tp_ic_name, "sec-s6sy771") && (strstr(saved_command_line, "mdss_dsi_oppo19101samsung_sofx01f_a_1080_2400_cmd")
            || strstr(saved_command_line, "mdss_dsi_oppo19101samsung_amb655uv01_1080_2400_cmd"))) {
            g_tp_dev_vendor = TP_SAMSUNG;
            return true;
        }
        pr_err("[TP] Driver does not match the project\n");
        break;

    case 19125:
    case 19127:
        if (strstr(tp_ic_name, "synaptics-s3706") && (strstr(saved_command_line, "mdss_dsi_oppo19125samsung_s6e3fc2x01_1080_2340_cmd")
            || strstr(saved_command_line, "mdss_dsi_oppo19125samsung_ams641rw01_1080_2340_cmd")
            || strstr(saved_command_line, "mdss_dsi_oppo19101samsung_sofx01f_a_1080_2400_cmd"))) {
            g_tp_dev_vendor = TP_SAMSUNG;
            return true;
        }
        pr_err("[TP] Driver does not match the project\n");
        break;
#ifdef ODM_HQ_EDIT
/*Benshan.Cheng@ODM_HQ.BSP.TP.Function, 2019/10/14 add for oppo arch tp*/
	case 19661:
        if (strstr(tp_ic_name, "novatek,nf_nt36672c") /*&& (strstr(saved_command_line, "mdss_dsi_oppo19125samsung_s6e3fc2x01_1080_2340_cmd")
            || strstr(saved_command_line, "mdss_dsi_oppo19125samsung_ams641rw01_1080_2340_cmd")
            || strstr(saved_command_line, "mdss_dsi_oppo19101samsung_sofx01f_a_1080_2400_cmd"))*/) {
            tp_used_index = nt36672c;
            if (strstr(saved_command_line, "nt36672c_tianma")) {
					g_tp_dev_vendor = TP_TIANMA;
			} else if (strstr(saved_command_line, "nt36672c_jdi")) {
					g_tp_dev_vendor = TP_JDI;
			} else if (strstr(saved_command_line, "nt36672c_boe")) {
					g_tp_dev_vendor = TP_BOE;
			} else {
					g_tp_dev_vendor = TP_UNKNOWN;
					break;
			}
			pr_err( "[TP] g_tp_dev_vendor: %s\n",tp_dev_names[g_tp_dev_vendor].name);
            return true;
        }
        pr_err("[TP] Driver does not match the project\n");
        break;
	case 20682:
		if (strstr(tp_ic_name, "novatek,nf_nt36672c") /*&& (strstr(saved_command_line, "mdss_dsi_oppo19125samsung_s6e3fc2x01_1080_2340_cmd")
			|| strstr(saved_command_line, "mdss_dsi_oppo19125samsung_ams641rw01_1080_2340_cmd")
			|| strstr(saved_command_line, "mdss_dsi_oppo19101samsung_sofx01f_a_1080_2400_cmd"))*/) {
			tp_used_index = nt36672c;
			if (strstr(saved_command_line, "nt36672c_tianma")) {
					g_tp_dev_vendor = TP_TIANMA;
			} else if (strstr(saved_command_line, "nt36672c_jdi")) {
					g_tp_dev_vendor = TP_JDI;
			} else if (strstr(saved_command_line, "nt36672c_boe")) {
					g_tp_dev_vendor = TP_BOE;
			} else {
					g_tp_dev_vendor = TP_UNKNOWN;
					break;
			}
			pr_err( "[TP] g_tp_dev_vendor: %s\n",tp_dev_names[g_tp_dev_vendor].name);
			return true;
		}
		pr_err("[TP] Driver does not match the project\n");
		break;

#endif
	case 19751:
		//if (strstr(tp_ic_name, "nova-nt36672") && (strstr(saved_command_line, "mdss_dsi_oppo19101samsung_sofx01f_a_1080_2400_cmd"))) {
        //    g_tp_dev_vendor = TP_BOE;
		//	tp_used_index = nt36672c;
        //    return true;
        //}
		if (strstr(tp_ic_name, "himax,hx83112a_nf")/* && (strstr(saved_command_line, "mdss_dsi_oppo19101samsung_sofx01f_a_1080_2400_cmd"))*/) {
            g_tp_dev_vendor = TP_DSJM;
			tp_used_index = himax_83112a;
			pr_err("[TP] Driver himax_83112a\n");
            return true;
        }
        pr_err("[TP] Driver does not match the project\n");
		break;
//#ifdef ODM_HQ_EDIT_OPPO6769
		/*Benshan.Cheng@ODM_HQ.BSP.TP.Function, 2019/10/14 add for oppo arch tp*/
	case 20671:
				if (strstr(tp_ic_name, "ilitek,ili9881h")) {
					tp_used_index = ili9881h_boe;
					g_tp_dev_vendor = TP_BOE;		
				} else if (strstr(tp_ic_name, "novatek,nf_nt36525b")) {
					tp_used_index = nt36525b_inx;
					g_tp_dev_vendor = TP_INX;		
				} else {
					tp_used_index = TP_INDEX_NULL;
					g_tp_dev_vendor = TP_UNKNOWN;
					pr_err("[TP] Driver does not match the project\n");
					break;
				}
				pr_err( "[TP] g_tp_dev_vendor: %s\n",tp_dev_names[g_tp_dev_vendor].name);
				return true; 
//#endif
    default:
        pr_err("Invalid project\n");
        break;
    }

    pr_err("Lcd module not found\n");
  	
    return false;
}

int tp_util_get_vendor(struct hw_resource *hw_res, struct panel_info *panel_data)
{
    char* vendor;

    panel_data->test_limit_name = kzalloc(MAX_LIMIT_DATA_LENGTH, GFP_KERNEL);
    if (panel_data->test_limit_name == NULL) {
        pr_err("[TP]panel_data.test_limit_name kzalloc error\n");
    }

   panel_data->tp_type = g_tp_dev_vendor;

    if (panel_data->tp_type == TP_UNKNOWN) {
        pr_err("[TP]%s type is unknown\n", __func__);
        return 0;
    }

    vendor = GET_TP_DEV_NAME(panel_data->tp_type);

    strcpy(panel_data->manufacture_info.manufacture, vendor);

    snprintf(panel_data->fw_name, MAX_FW_NAME_LENGTH,
            "tp/%d/FW_%s_%s.img", get_project(), panel_data->chip_name, vendor);

    if (panel_data->test_limit_name) {
        snprintf(panel_data->test_limit_name, MAX_LIMIT_DATA_LENGTH,
            "tp/%d/LIMIT_%s_%s.img",get_project(), panel_data->chip_name, vendor);
    }

	if (tp_used_index == himax_83112a) {
		memcpy(panel_data->manufacture_info.version, "HX_DSJM", 7);
		panel_data->firmware_headfile.firmware_data = FW_18621_HX83112A_NF_DSJM;
		panel_data->firmware_headfile.firmware_size = sizeof(FW_18621_HX83112A_NF_DSJM);
	}

	switch (get_project()) {
	case 19661:
		if (tp_used_index == nt36672c) {
			if (g_tp_dev_vendor == TP_TIANMA) {
				memcpy(panel_data->manufacture_info.version, "NVT_TM", 6);
				panel_data->firmware_headfile.firmware_data = FW_19661_NT36672C_TIANMA;
				panel_data->firmware_headfile.firmware_size = sizeof(FW_19661_NT36672C_TIANMA);
			} else if (g_tp_dev_vendor == TP_JDI) {
				memcpy(panel_data->manufacture_info.version, "NVT_JDI", 7);
				panel_data->firmware_headfile.firmware_data = FW_19661_NT36672C_JDI;
				panel_data->firmware_headfile.firmware_size = sizeof(FW_19661_NT36672C_JDI);
			} else if (g_tp_dev_vendor == TP_BOE) {
				memcpy(panel_data->manufacture_info.version, "NVT_BOE", 7);
				panel_data->firmware_headfile.firmware_data = FW_19661_NT36672C_BOE;
				panel_data->firmware_headfile.firmware_size = sizeof(FW_19661_NT36672C_BOE);
			} else {
				pr_info("[TP] 19661 tp vendor not found\n");
			}
		} else {
			pr_info("[TP] 19661 tp ic not found\n");
		}
	break;
	case 20682:
		if (tp_used_index == nt36672c) {
			if (g_tp_dev_vendor == TP_TIANMA) {
				memcpy(panel_data->manufacture_info.version, "A539TM", 6);
				panel_data->firmware_headfile.firmware_data = FW_20682_NT36672C_TIANMA;
				panel_data->firmware_headfile.firmware_size = sizeof(FW_20682_NT36672C_TIANMA);
			} else if (g_tp_dev_vendor == TP_JDI) {
				memcpy(panel_data->manufacture_info.version, "A539JDI", 7);
				panel_data->firmware_headfile.firmware_data = FW_20682_NT36672C_JDI;
				panel_data->firmware_headfile.firmware_size = sizeof(FW_20682_NT36672C_JDI);
			} else if (g_tp_dev_vendor == TP_BOE) {
				memcpy(panel_data->manufacture_info.version, "A539BOE", 7);
				panel_data->firmware_headfile.firmware_data = FW_20682_NT36672C_BOE;
				panel_data->firmware_headfile.firmware_size = sizeof(FW_20682_NT36672C_BOE);
			} else {
				pr_info("[TP] 20682 tp vendor not found\n");
			}
		} else {
			pr_info("[TP] 20682 tp ic not found\n");
		}
	break;
	case 20671:
		if (tp_used_index == ili9881h_boe) {
			memcpy(panel_data->manufacture_info.version, "ILI_BOE", 7);
			panel_data->firmware_headfile.firmware_data = FW_20671_ILI9881H_NF_BOE;
			panel_data->firmware_headfile.firmware_size = sizeof(FW_20671_ILI9881H_NF_BOE);
		} else if (tp_used_index == nt36525b_inx) {
			memcpy(panel_data->manufacture_info.version, "NVT_INX", 7);
			panel_data->firmware_headfile.firmware_data = FW_20671_NT36525B_NF_INX;
			panel_data->firmware_headfile.firmware_size = sizeof(FW_20671_NT36525B_NF_INX);
		} else {
			pr_info("[TP] 20671 tp ic not found\n");
		}
		break;
	default:
		pr_err("tp_util_get_vendor:Invalid project\n");
		break;
	}
	panel_data->version_len = strlen(panel_data->manufacture_info.version);
    panel_data->manufacture_info.fw_path = panel_data->fw_name;

    pr_info("[TP]vendor:%s fw:%s limit:%s\n",
        vendor,
        panel_data->fw_name,
        panel_data->test_limit_name == NULL ? "NO Limit" : panel_data->test_limit_name);

    return 0;
}

