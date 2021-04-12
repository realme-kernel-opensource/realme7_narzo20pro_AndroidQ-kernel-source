/********************************************
 ** Copyright (C) 2019 OPPO Mobile Comm Corp. Ltd.
 ** ODM_HQ_EDIT
 ** File: hq_devinfo.c
 ** Description: Source file for proc devinfo
 ** Version :1.0
 ** Date : 2019/8/16
 ** Author: sunjingtao@ODM_HQ.BSP.system
 ********************************************/
#include <linux/mmc/host.h>
#include <linux/mmc/card.h>
#include <linux/mmc/mmc.h>
#include <linux/hq_devinfo.h>
#include "oppoversion.h"
#include "../../input/touchscreen/mediatek/tpd.h"
#include "meta_display.c"
#include <linux/iio/consumer.h>
#define devinfo_name "devinfo"

//extern char battery_name[20];
extern struct mmc_card *card_devinfo;
extern char *hq_lcm_name;
char buff[128];
char sensor_vendor[3][80];
Cam_buff cam_buff; 
int stage_volt = 0;

static int hq_devinfo_probe(struct platform_device *pdev);
static int hq_devinfo_remove(struct platform_device *pdev);

#ifndef ODM_HQ_EDIT
/*wangtao@ODM.HQ.BSP.CHG 2019/10/17 modify kernel error*/
struct sensor_devinfo sensorinfo[] = {
	{"icm4x6xx_acc","ICM"},
	{"bmi160_acc","BOSCH"},
	{"bma253","BOSCH"},
	{"lis2doc","ST"},
	{"mmc5603","MICROCHIP"},
	{"af6133e","VTC"},
	{"stk3a5xx_p","SensorTek"},
	{"stk3a5xx_l","SensorTek"},
};
#endif
static void match_sub_board(int boardid){
	int MB = boardid & 0xf;
	int KB = (boardid >> 4) & 0x1;

	printk("kb_id == %d\n",KB);
	if((0 == MB) || (1 == MB) || (2 == MB) || (3 == MB)){
		if(0 == KB){
			sprintf(buff, "Device version: %s\nDevice manufacture: %s\n","MTK","sub-match");
		}else{
			sprintf(buff, "Device version: %s\nDevice manufacture: %s\n","MTK","sub-unmatch");
		}
	}else{
		sprintf(buff, "Device version: %s\nDevice manufacture: %s\n","MTK","UNKOWN");
	}
}

static void get_sub_board(void){
	char *ptr;
	ptr = strstr(saved_command_line, "board_id=");
	if(ptr != 0){
		ptr += strlen("board_id=");
		match_sub_board(simple_strtol(ptr, NULL, 10));
	}else{
		sprintf(buff, "Device version: %s\nDevice manufacture: %s","MTK","UNKOWN");
	}
}

static void get_stage_version(void){
	printk("stage_version adc vol == %d\n",stage_volt);

	if(stage_volt  >= 1300)
		sprintf(buff, "stage version: %s\n","MP3");
	else if (stage_volt  >= 1150)
		sprintf(buff, "stage version: %s\n","MP2");
	else if (stage_volt  >= 1050)
		sprintf(buff, "stage version: %s\n","MP1");
	else if (stage_volt  >= 950)
		sprintf(buff, "stage version: %s\n","PVT");
	else if (stage_volt  >= 850)
		sprintf(buff, "stage version: %s\n","DVT3");
	else if (stage_volt  >= 760)
		sprintf(buff, "stage version: %s\n","DVT2");
	else if (stage_volt  >= 650)
		sprintf(buff, "stage version: %s\n","DVT1");
	else if (stage_volt  >= 550)
		sprintf(buff, "stage version: %s\n","EVT3");
	else if (stage_volt  >= 420)
		sprintf(buff, "stage version: %s\n","EVT2");
	else if (stage_volt  >= 300)
		sprintf(buff, "stage version: %s\n","EVT1");
	else if (stage_volt  >= 180)
		sprintf(buff, "stage version: %s\n","T1");
	else if (stage_volt  >= 80)
		sprintf(buff, "stage version: %s\n","T0");
	else if (stage_volt  >= 0)
		sprintf(buff, "stage version: %s\n","EVB1");
	else
		sprintf(buff, "stage version: %s\n","UNKOWN");
}


static void register_info(char name[]){
	if(!strcmp(name, "emmc")){
		char manfid[8];
		int manfid_m ;

		if(card_devinfo == NULL){
			sprintf(buff, "Device version: %s\nDevice manufacture: %s","UNKOWN", "UNKOWN");
			return;
		}
		manfid_m= card_devinfo->cid.manfid;
		switch(manfid_m){
			case 0x15:
				strcpy(manfid, "SAMSUNG");
				break;
			case 0x45:
				strcpy(manfid, "SANDISK");
				break;
			case 0x13:
				strcpy(manfid, "MICRON");
				break;
			case 0x90:
				strcpy(manfid, "HYNIX");
				break;
			default:
				strcpy(manfid, "UNKOWN");
				break;
		}
		sprintf(buff, "Device version: %s\nDevice manufacture: %s",card_devinfo->cid.prod_name, manfid);
		return;
	}else if(!strcmp(name, "emmc_version")){
		if(card_devinfo == NULL){
			sprintf(buff, "Device version: %s\nDevice manufacture: %s","UNKOWN", "UNKOWN");
			return;
		}
		if(card_devinfo->ext_csd.rev < 7){
			sprintf(buff, "Device version: %s\nDevice manufacture: 0x%x",card_devinfo->cid.prod_name, card_devinfo->cid.fwrev);
		}else{
			u8 fwrev = card_devinfo->ext_csd.fwrev[0];
			sprintf(buff, "Device version: %s\nDevice manufacture: 0x%x", card_devinfo->cid.prod_name, fwrev);
		}		
		return;
	}else if(!strcmp(name, "lcd")){
		printk("devinfo lcd name = %s\n",hq_lcm_name);
		if(!strcmp(hq_lcm_name, "hx83112a_dsbj_jdi")){
			sprintf(buff, "Device version: %s\nDevice manufacture: %s","hx83112a","JDI_HX");
		}else{
			sprintf(buff, "Device version: %s\nDevice manufacture: %s","UNKOWN","UNKOWN");
		}
		return;
	}else if(!strcmp(name, "tp")){
		//if(!strcmp(oppo_tp_data.tp_dev_name, "fts_ts") || !strcmp(oppo_tp_data.tp_dev_name, "himax_tp") || !strcmp(oppo_tp_data.tp_dev_name, "gt9xx")){
		//	sprintf(buff, "Device version: 0x%x\nDevice manufacture: %s\nDevice fw_path: %s", oppo_tp_data.version, oppo_tp_data.manufacture, oppo_tp_data.fw_name);
		//}else{
			sprintf(buff, "Device version: %s\nDevice manufacture: %s\nDevice fw_path: %s","UNKOWN","UNKOWN","UNKOWN");
		//}
		return;
	}else if(!strcmp(name, "Sensor_gyro")){
		sprintf(buff, "Device version: %s\nDevice manufacture: %s","UNKOWN","UNKOWN");
		return;
	}else if(!strcmp(name, "Cam_b")){
		sprintf(buff, "Device version: %s\nDevice manufacture: %s","UNKOWN","UNKOWN");
		return;
	}else if(!strcmp(name, "Cam_b2")){
		sprintf(buff, "Device version: %s\nDevice manufacture: %s","UNKOWN","UNKOWN");
		return;
	}else if(!strcmp(name, "Cam_f")){
		sprintf(buff, "Device version: %s\nDevice manufacture: %s","UNKOWN","UNKOWN");
		return;
}else if(!strcmp(name, "stage_ver")){
		get_stage_version();
		return;
	}else if(!strcmp(name, "sub_mainboard")){
		get_sub_board();
		return;
	}
}

static void hq_parse_sensor_devinfo(int type, char ic_name[], char vendor[]){
	sprintf(sensor_vendor[type], "Device version:  %s\nDevice manufacture:  %s\n",
			ic_name, vendor);
}

#ifndef ODM_HQ_EDIT
/*wangtao@ODM.HQ.BSP.CHG 2019/10/17 modify kernel error*/
void hq_register_sensor_info(int type, char ic_name[]){
	int i;
	if(type >= 0 && type <3) {
		for (i = 0; i < sizeof(sensorinfo)/sizeof(struct sensor_devinfo); i++) {
			/* Jianmin.Niu@ODM.HQ.BSP.Sensors.Config 2019/2/1 Update string compare */
			if(!strncmp(ic_name, sensorinfo[i].ic_name, strlen(ic_name))) {
				hq_parse_sensor_devinfo(type, sensorinfo[i].ic_name, sensorinfo[i].vendor_name);
				break;
			}
		}
	}
	return;
}
#endif
#ifdef ODM_HQ_EDIT
/*wangtao@ODM.HQ.BSP.CHG 2019/10/17 modify kernel error*/
void hq_register_sensor_info(int type, char ic_name[]){
	return;
}
#endif
static const char * const devinfo_proc_list[] = {
	"Cam_b",
	"Cam_f",
	"Cam_b2",
	"Sensor_accel",
	"Sensor_alsps",
	"Sensor_gyro",
	"Sensor_msensor",
	"lcd",
	"tp",
	"emmc",
	"emmc_version",
	//"battery",
	"stage_ver",
	"sub_mainboard"
};

HQ_DEVINFO_ATTR(Cam_b, "%s",buff);
HQ_DEVINFO_ATTR(Cam_f, "%s",buff);
HQ_DEVINFO_ATTR(Cam_b2, "%s",buff);
HQ_DEVINFO_ATTR(Sensor_accel, "%s",sensor_vendor[ACCEL_HQ]);
HQ_DEVINFO_ATTR(Sensor_alsps, "%s",sensor_vendor[ALSPS_HQ]);
HQ_DEVINFO_ATTR(Sensor_gyro, "%s",buff);
HQ_DEVINFO_ATTR(Sensor_msensor, "%s",sensor_vendor[MSENSOR_HQ]);
HQ_DEVINFO_ATTR(lcd, "%s",buff);
HQ_DEVINFO_ATTR(tp, "%s",buff);
HQ_DEVINFO_ATTR(emmc, "%s",buff);
HQ_DEVINFO_ATTR(emmc_version, "%s",buff);
//HQ_DEVINFO_ATTR(battery, "Device version: 4.4V NON_VOOC\nDevice manufacture: %s",battery_name);
HQ_DEVINFO_ATTR(stage_ver, "%s",buff);
HQ_DEVINFO_ATTR(sub_mainboard, "%s",buff);

static const struct file_operations *proc_fops_list[] = {
	&Cam_b_fops,
	&Cam_f_fops,
	&Cam_b2_fops,
	&Sensor_accel_fops,
	&Sensor_alsps_fops,
	&Sensor_gyro_fops,
	&Sensor_msensor_fops,
	&lcd_fops,
	&tp_fops,
	&emmc_fops,
	&emmc_version_fops,
	//&battery_fops,
	&stage_ver_fops,
	&sub_mainboard_fops,
};

static const struct of_device_id hq_devinfo_of_match[] = {
    { .compatible = "mediatek,hq_devinfo", },
    {},
};


static struct platform_driver hq_devinfo_platform_driver = {
    .probe = hq_devinfo_probe,
    .remove = hq_devinfo_remove,
    .driver = {
        .name = "hq_devinfo",
        .of_match_table = hq_devinfo_of_match,
    },
};

static int hq_devinfo_probe(struct platform_device *pdev)
{
	int id_volt = 0;
	int ret = 0;
	int auxadc_voltage = 0;
	struct iio_channel *channel;

	printk("%s:enter\n", __func__);
	channel = iio_channel_get(&(pdev->dev), "stageID-channel");
	if (IS_ERR(channel)) {
		ret = PTR_ERR(channel);
		printk("[%s] iio channel not found %d\n",__func__, ret);
		return -1;
	}

	if (channel)
		ret = iio_read_channel_processed(channel, &auxadc_voltage);

	printk("[%s]auxadc_voltage is %d\n", __func__, auxadc_voltage);
	id_volt = auxadc_voltage * 1500 / 4096;
	printk("[%s]battery_id_voltage is %d\n", __func__, id_volt);
	stage_volt = id_volt;
	printk(" hq_devinfo_probe success\n");

	return 0;
}

static int hq_devinfo_remove(struct platform_device *pdev)
{
    platform_driver_unregister(&hq_devinfo_platform_driver);
    return 0;
}


static int __init devinfo_init(void){
	struct proc_dir_entry *prEntry;
	struct proc_dir_entry *devinfo_dir;
	int i, num;

	printk(" devinfo_init start\n");
	devinfo_dir  = proc_mkdir(devinfo_name, NULL);

	if (!devinfo_dir) {
		pr_notice("[%s]: failed to create /proc/%s\n",__func__, devinfo_name);
		return -1;
	}
	
	num = ARRAY_SIZE(devinfo_proc_list);
	for (i = 0; i < num; i++) {
		prEntry = proc_create(devinfo_proc_list[i], 0444, devinfo_dir, proc_fops_list[i]);
		if (prEntry)
			continue;
		pr_notice("[%s]: failed to create /proc/devinfo/%s\n", __func__, devinfo_proc_list[i]);
	}
	
	oppoversion_init();
	meta_work_around();

	if (platform_driver_register(& hq_devinfo_platform_driver)) {
		printk("Failed to register  hq_devinfo_platform_driver!\n");
		return -1;
	}

	return 0;
}


static void __exit  devinfo_exit(void)
{
    platform_driver_unregister(& hq_devinfo_platform_driver);
}

module_init(devinfo_init);
module_exit( devinfo_exit);
MODULE_AUTHOR("Venco <sunjingtao@vanyol.com>");
MODULE_DESCRIPTION("Huaqin Devices Info Driver");
MODULE_LICENSE("GPL");
