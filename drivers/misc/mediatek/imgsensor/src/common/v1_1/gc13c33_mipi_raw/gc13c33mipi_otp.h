/*****************************************************************************
 *
 * Filename:
 * ---------
 *     GC13c33mipi_Sensor.h
 *
 * Project:
 * --------
 *     ALPS
 *
 * Description:
 * ------------
 *     CMOS sensor header file
 *
 ****************************************************************************/
#ifndef _GC13C33MIPI_SENSOR_H
#define _GC13C33MIPI_SENSOR_H

#define OTP_WRITE_ID   0x20 /*0x72  add 0117 edan */
#define EEPROM_WRITE_ID 0xa4 //0xa0

#define CROSSTALK_BUF_SIZE 772 /*770+2*/
#define CROSSTALK_FLAG_OFFSET 0x006c
#define CROSSTALK_START_ADDR 0x006d  //add 0117 edan 07a1

#define GC13C33_OTP_FOR_CUSTOMER  0
#define OTP_FLAG_EMPTY            0x00
#define OTP_FLAG_VALID            0x01
#define OTP_FLAG_INVALID          0x02
#define OTP_FLAG_INVALID2         0x03

#define OTP_GROUP_CNT             2
#define OTP_START_ADDR 0x0000
#define OTP_DATA_LENGTH 2048

#define DPC_FLAG_OFFSET 0x0088
#define DPC_TOTAL_NUMBER_OFFSET 0x0090
#define DPC_ERROR_NUMBER_OFFSET 0x0098
#define DPC_DATA_OFFSET 0x00a0
#define DPC_BUF_SIZE              802 /* 200 x 4 + 2 */

#define REG_INFO_FLAG_OFFSET 0x19a0
#define REG_INFO_ADDR_OFFSET 0x19a8
#define REG_INFO_VALUE_OFFSET 0x19b8
#define REG_MAX_GROUP             15
#define REG_GROUP_SIZE            3


#define AF_INFO_FLAG_OFFSET 0x3ea8
#define AF_INFO_OFFSET 0x3eb0
#define AF_INFO_SIZE 4
#define WB_INFO_FLAG_OFFSET 0x3ef0
#define WB_INFO_OFFSET 0x3ef8
#define WB_INFO_SIZE 4
#define WB_GOLDEN_INFO_OFFSET 0x3f38
#define WB_GOLDEN_INFO_SIZE 4
#define RG_TYPICAL       0x0400
#define BG_TYPICAL       0x0400
#define MODULE_INFO_FLAG_OFFSET 0x3f78
#define MODULE_INFO_OFFSET 0x3f80
#define MODULE_INFO_SIZE 8

#define GC13C33_OTP_ID_SIZE           9
#define GC13C33_OTP_ID_DATA_OFFSET    0x0020
struct gc13c33_otp {
	kal_uint8 otp_id[GC13C33_OTP_ID_SIZE];
	kal_uint8  reg_flag;
	kal_uint8  reg_num;
	kal_uint16 regs[REG_MAX_GROUP][2];
	kal_uint8  wb_flag;
	kal_uint16 rg_gain;
	kal_uint16 bg_gain;
	kal_uint8  golden_flag;
	kal_uint16 golden_rg;
	kal_uint16 golden_bg;
};

extern int iReadRegI2C(u8 *a_pSendData, u16 a_sizeSendData, u8 *a_pRecvData, u16 a_sizeRecvData, u16 i2cId);
extern int iWriteRegI2C(u8 *a_pSendData, u16 a_sizeSendData, u16 i2cId);
extern int iWriteReg(u16 a_u2Addr, u32 a_u4Data, u32 a_u4Bytes, u16 i2cId);
#endif
