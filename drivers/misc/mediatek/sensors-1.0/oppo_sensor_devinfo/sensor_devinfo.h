/******************************************************************
** Copyright (C), 2004-2017, OPPO Mobile Comm Corp., Ltd.
** VENDOR_EDIT
** File: - oppo_sensor_devinfo.c
** Description: Source file for sensor device infomation registered in scp.
** Version: 1.0
** Date : 2017/12/12
** Author: Fei.Mo@PSW.BSP.Sensor
**
** --------------------------- Revision History: ---------------------
* <version>	<date>		<author>              		<desc>
* Revision 1.0      2017/12/12        Fei.Mo@PSW.BSP.Sensor   	Created,need to sync acp info
*******************************************************************/
#ifndef SENSOR_DEVINFO_H
#define SENSOR_DEVINFO_H

enum {
    ALSPS_UNKNOW = 0,
    ALSPS_TMD2725,
    ALSPS_APSD9922,
    ALSPS_STK3335,
    ALSPS_UNION_TSL2540_STK3331,
    ALSPS_MULT_APDS9925_STK3332,
/* zhoujunwei@ODM.BSP.Sensor  2020/03/18 sync sensor devinfo */
    ALSPS_UNION_TSL2591_STK3332,
    ALSPS_STK3331,
    ALSPS_STK2232,
#ifdef ODM_HQ_EDIT
/* zuoqiquan@ODM.BSP.Sensor  2019/11/1 sync oppo sensor code */
    ALSPS_STK33562,
#endif /*ODM_HQ_EDIT*/
};

enum {
    GSENSOR_UNKNOW = 0,
    GSENSOR_LIS3DH,
    GSENSOR_LSM6DS3,
    GSENSOR_BMI160,
    GSENSOR_LIS2HH12,
    GSENSOR_LSM6DSM,
#ifdef ODM_HQ_EDIT
/* zuoqiquan@ODM.BSP.Sensor  2019/11/1 sync oppo sensor code */
    GSENSOR_BMA253,
    GSENSOR_LIS2DOC,
    GSENSOR_ICM40607,
    GSENSOR_MC3419P,
    GSENSOR_BMI220,
#endif /*ODM_HQ_EDIT*/
};

enum {
    MAG_UNKNOW = 0,
    MAG_AKM09911,
    MAG_MMC3530,
    MAG_MMC5603,
    MAG_MXG4300,
#ifdef ODM_HQ_EDIT
/* zuoqiquan@ODM.BSP.Sensor  2019/11/1 sync oppo sensor code */
    MAG_AF6133E,
    MAG_AF6133,//add af6133
    MAG_AKM09918,
#endif /*ODM_HQ_EDIT*/
};

enum {
    GYRO_UNKNOW = 0,
    GYRO_LSM6DS3,
    GYRO_BMI160,
    GYRO_LSM6DSM,
#ifdef ODM_HQ_EDIT
/* zuoqiquan@ODM.BSP.Sensor  2019/11/1 sync oppo sensor code */
    GYRO_ICM40607,
    GYRO_BMI220,
#endif /*ODM_HQ_EDIT*/
};

/*
 * @sensortye ,defined in hwmsensor.h, ID_ACCELEROMETER ,etc
 * return sensor name , for gsensor ,GSENSOR_LIS3DH,etc
*/
extern int get_sensor_name(int sensortye);

/*
 * @sensortye ,defined in hwmsensor.h, ID_ACCELEROMETER ,etc
 * @ data,  sensor calibration return , for light sensor , return gain
*/
extern int get_sensor_parameter(int sensortye ,int * data);

/*
 * @sensortye ,defined in hwmsensor.h, ID_ACCELEROMETER ,etc
 * @ data ,sensor calibration need to update , for light sensor , update gain
*/
extern int update_sensor_parameter(int sensortye ,int * data);
#endif
