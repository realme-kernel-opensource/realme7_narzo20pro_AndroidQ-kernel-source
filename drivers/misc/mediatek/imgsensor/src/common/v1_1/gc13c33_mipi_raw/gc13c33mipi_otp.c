/*****************************************************************************
 *
 * Filename:
 * ---------
 *     gc13c33mipi_Sensor.c
 *
 * Project:
 * --------
 *     ALPS
 *
 * Description:
 * ------------
 *     Source code of Sensor driver
 *
 * Driver Version:
 * ------------
 *     V0.2010.200
 *
 *------------------------------------------------------------------------------
 * Upper this line, this part is controlled by CC/CQ. DO NOT MODIFY!!
 *============================================================================
 ****************************************************************************/
 
#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/atomic.h>
#include <linux/types.h>

#include "kd_camera_typedef.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"

#include "gc13c33mipi_otp.h"

/****************************Modify Following Strings for Debug****************************/
#define PFX "GC13C33MIPI_OTP"
/****************************   Modify end    *******************************************/
#define GC13C33_OTP_DEBUG 1
#if GC13C33_OTP_DEBUG
#define LOG_INF(format, args...)	pr_debug(PFX "[%s] " format, __func__, ##args)
#else
#define LOG_INF(format, args...)
#endif

#define GC13C33_OTP_DUMP    0
#define DPC_DEBUG           0
#define CT_DEBUG            0
#define OTP_REG_DEBUG       0
#define OTP_ID_DEBUG        0
static kal_uint16 read_cmos_sensor(kal_uint32 addr)
{
	kal_uint16 get_byte = 0;
	char pu_send_cmd[2] = { (char)(addr >> 8), (char)(addr & 0xff) };

	iReadRegI2C(pu_send_cmd, 2, (u8 *) &get_byte, 1, OTP_WRITE_ID);
	return get_byte;
}

static void write_cmos_sensor(kal_uint16 addr, kal_uint16 para)
{
	char pusendcmd[3] = { (char)(addr >> 8), (char)(addr & 0xff), (char)(para & 0xff) };

	iWriteRegI2C(pusendcmd, 3, OTP_WRITE_ID);
}

static kal_uint16 read_eeprom(kal_uint16 addr)
{
	kal_uint16 get_byte = 0;
	char pu_send_cmd[2] = {(char)(addr >> 8), (char)(addr & 0xff) };

	iReadRegI2C(pu_send_cmd, 2, (u8 *) &get_byte, 1, EEPROM_WRITE_ID);
	return get_byte;
}

static kal_uint8 ct_read_data[CROSSTALK_BUF_SIZE];
void gc13c33_read_crosstalk_data(void)
{
	kal_uint16 i = 0;
	kal_uint16 ct_flag = 0;
	kal_uint16 checksum = 0;

	memset(&ct_read_data, 0, CROSSTALK_BUF_SIZE * sizeof(kal_uint8));
	ct_flag = read_eeprom(CROSSTALK_FLAG_OFFSET);
	if (ct_flag == OTP_FLAG_VALID) {
	for (i = 0; i < CROSSTALK_BUF_SIZE; i++)
			ct_read_data[i] = read_eeprom(CROSSTALK_START_ADDR + i);

	for (i = 2; i < (CROSSTALK_BUF_SIZE - 1); i++)
			checksum += ct_read_data[i];

		if ((checksum % 255) == ct_read_data[CROSSTALK_BUF_SIZE - 1])
			LOG_INF("check success! calc_checksum = %d, read_checksum = %d\n",
			(checksum % 255), ct_read_data[CROSSTALK_BUF_SIZE - 1]);
		else {
			memset(ct_read_data, 0, CROSSTALK_BUF_SIZE);
			LOG_INF("check error! calc_checksum = %d, read_checksum = %d\n",
			(checksum % 255), ct_read_data[CROSSTALK_BUF_SIZE - 1]);
		}
#if CT_DEBUG
		for (i = 0; i < CROSSTALK_BUF_SIZE; i++)
			LOG_INF("ct_data[%d] = %x\n", i, ct_read_data[i]);
#endif
	} else
		LOG_INF("crosstalk data is zero!\n");
}

void gc13c33_copy_crosstalk_data(kal_uint8 *ct_data)
{
	LOG_INF("start copy crosstalk_data!\n");
	memcpy(ct_data, (void *)&ct_read_data[0], CROSSTALK_BUF_SIZE);
}
static struct gc13c33_otp gc13c33_otp_data;

static kal_uint8 gc13c33_otp_read_byte(kal_uint16 addr)
{
	write_cmos_sensor(0x0a69, (addr >> 8) & 0x3f);
	write_cmos_sensor(0x0a6a, addr & 0xff);
	write_cmos_sensor(0x0313, 0x20);
	return read_cmos_sensor(0x0a6c);
}

static void gc13c33_otp_read_group(kal_uint16 addr, kal_uint8 *data, kal_uint16 length)
{
	kal_uint16 i = 0;

	write_cmos_sensor(0x0a69, (addr >> 8) & 0x3f);
	write_cmos_sensor(0x0a6a, addr & 0xff);
	write_cmos_sensor(0x0313, 0x20);
	write_cmos_sensor(0x0313, 0x12);

	for (i = 0; i < length; i++)
		data[i] = read_cmos_sensor(0x0a6c);
}

static kal_uint8 dpc_read_data[DPC_BUF_SIZE];
void gc13c33_otp_read_dpc_info(void)
{
#if	DPC_DEBUG
	kal_uint16 i = 0;
#endif
	kal_uint8 flag_dpc = 0;
	kal_uint8 total_number = 0;

	memset(&dpc_read_data, 0, DPC_BUF_SIZE * sizeof(kal_uint8));

	
	flag_dpc = gc13c33_otp_read_byte(DPC_FLAG_OFFSET);
	LOG_INF("flag_dpc = 0x%x\n", flag_dpc);
	switch (flag_dpc & 0x03) {
	case OTP_FLAG_EMPTY:
		LOG_INF("DPC is empty!\n");
		break;
	case OTP_FLAG_VALID:
		LOG_INF("DPC is valid!\n");
		total_number = gc13c33_otp_read_byte(DPC_TOTAL_NUMBER_OFFSET)
					+ gc13c33_otp_read_byte(DPC_ERROR_NUMBER_OFFSET);
		LOG_INF("total_number = %d\n", total_number);
		gc13c33_otp_read_group(DPC_DATA_OFFSET, &dpc_read_data[2], 4 * total_number);
		break;
	case OTP_FLAG_INVALID:
	case OTP_FLAG_INVALID2:
		LOG_INF("DPC is invalid!\n");
		break;
	default:
		break;
	}

	dpc_read_data[0] = total_number & 0xff;
	dpc_read_data[1] = (total_number >> 8) & 0xff;
#if DPC_DEBUG
	for (i = 0; i < (4 * total_number + 2); i++)
		LOG_INF("dpc_data[%d] = 0x%x\n", i, dpc_read_data[i]);
#endif
}

void gc13c33_otp_copy_dpc_data(kal_uint8 *dpc_data)
{
	LOG_INF("start copy dpc_data!\n");
	memcpy(dpc_data, (void *)&dpc_read_data[0], DPC_BUF_SIZE);
}


static void gc13c33_otp_read_reg_info(void)
{

	kal_uint16 temp = 0, i = 0;

	gc13c33_otp_data.reg_flag = gc13c33_otp_read_byte(REG_INFO_FLAG_OFFSET);
	LOG_INF("reg_flag = 0x%x\n", gc13c33_otp_data.reg_flag & 0x03);
	if ((gc13c33_otp_data.reg_flag & 0x03) == OTP_FLAG_VALID) {
		for (i = 0; i < REG_MAX_GROUP; i++) {
			temp = gc13c33_otp_read_byte(REG_INFO_ADDR_OFFSET + REG_GROUP_SIZE * 8 * i);
			if ((temp & 0xc0) == 0x40) {
				gc13c33_otp_data.regs[gc13c33_otp_data.reg_num][0] =
					((temp & 0x0f) << 8)
					+ gc13c33_otp_read_byte(REG_INFO_ADDR_OFFSET + 8 + REG_GROUP_SIZE * 8 * i);
				gc13c33_otp_data.regs[gc13c33_otp_data.reg_num][1] =
					gc13c33_otp_read_byte(REG_INFO_VALUE_OFFSET + REG_GROUP_SIZE * 8 * i);
				gc13c33_otp_data.reg_num++;
			}
#if OTP_REG_DEBUG
			LOG_INF("check[%d] = 0x%x, addr = 0x%x, value = 0x%x\n", 
				i,
				temp & 0xc0,
				gc13c33_otp_data.regs[i][0],
				gc13c33_otp_data.regs[i][1]);
#endif
		}
		LOG_INF("reg_num = %d\n", gc13c33_otp_data.reg_num);
	} else {
		LOG_INF("register is zero\n");
	}
}

static void gc13c33_otp_load_regs(void)
{
	kal_uint8 i = 0;

	if ((gc13c33_otp_data.reg_flag & 0x03) == OTP_FLAG_VALID) {
		LOG_INF("load registers\n");
		for (i = 0; i < gc13c33_otp_data.reg_num; i++)
			write_cmos_sensor(gc13c33_otp_data.regs[i][0], gc13c33_otp_data.regs[i][1]);
		}
}

#if GC13C33_OTP_FOR_CUSTOMER
static void gc13c33_otp_read_af_info(void)
{
	kal_uint8 index = 0, flag_af = 0;
	kal_uint16 af_infinity = 0, af_macro = 0;
	kal_uint8 af[AF_INFO_SIZE] = { 0 };
	kal_uint16 check = 0, i = 0;

	memset(&af, 0, AF_INFO_SIZE * sizeof(kal_uint8));

	flag_af = gc13c33_otp_read_byte(AF_INFO_FLAG_OFFSET);
	LOG_INF("flag_af = 0x%x", flag_af);

	for (index = 0; index < OTP_GROUP_CNT; index++)
		switch ((flag_af >> (2 * (1 - index))) & 0x03) {
		case OTP_FLAG_EMPTY:
			LOG_INF("af info group %d is empty!\n", index + 1);
			break;
		case OTP_FLAG_VALID:
			LOG_INF("af info group %d is valid!\n", index + 1);
			gc13c33_otp_read_group(AF_INFO_OFFSET + AF_INFO_SIZE * 8 * index, &af[0], AF_INFO_SIZE);
			for (i = 0; i < AF_INFO_SIZE -1; i++)
				LOG_INF("addr = 0x%x, data = 0x%x\n",
				AF_INFO_OFFSET + AF_INFO_SIZE * 8 * index + i * 8, af[i]);

			for (i = 0; i < AF_INFO_SIZE - 1; i++)
				check += af[i];

			if ((check % 255 + 1) == af[AF_INFO_SIZE - 1]) {
				af_infinity = ((af[0] & 0xf0) << 4) + af[1];
				af_macro = ((af[0] & 0x0f) << 8) + af[2];
				LOG_INF("af_infinity = 0x%x, af_macro = 0x%x", af_infinity, af_macro);
			} else
				LOG_INF("af info Check sum %d error!! check sum = %d, sum = %d",
					index + 1, af[AF_INFO_SIZE - 1], (check % 255 + 1));
			break;
		case OTP_FLAG_INVALID:
		case OTP_FLAG_INVALID2:
			LOG_INF("af info group %d is invalid!\n", index + 1);
			break;
		default:
			break;
		}	
}

static void gc13c33_otp_read_module_info(void)
{
	kal_uint8 index = 0, flag_module = 0;
	kal_uint8 module_id = 0, lens_id = 0, af_id = 0, vcm_id = 0;
	kal_uint8 year = 0, month = 0, day = 0;
	kal_uint8 info[MODULE_INFO_SIZE] = { 0 };
	kal_uint16 check = 0, i = 0;

	memset(&info, 0, MODULE_INFO_SIZE * sizeof(kal_uint8));

	flag_module = gc13c33_otp_read_byte(MODULE_INFO_FLAG_OFFSET);
	LOG_INF("flag_module = 0x%x\n", flag_module);

	for (index = 0; index < OTP_GROUP_CNT; index++)
		switch ((flag_module >> (2 * (1 - index))) & 0x03) {
		case OTP_FLAG_EMPTY:
			LOG_INF("module info group %d is empty!\n", index + 1);
			break;
		case OTP_FLAG_VALID:
			LOG_INF("module info group %d is valid!\n", index + 1);
			gc13c33_otp_read_group(MODULE_INFO_OFFSET + MODULE_INFO_SIZE * 8 * index,
			&info[0], MODULE_INFO_SIZE);
			for (i = 0; i < MODULE_INFO_SIZE - 1; i++)
				LOG_INF("addr = 0x%x, data = 0x%x\n",
				MODULE_INFO_OFFSET + MODULE_INFO_SIZE * 8 * index + i * 8, info[i]);

			for (i = 0; i < MODULE_INFO_SIZE - 1; i++)
				check += info[i];

			if ((check % 255 + 1) == info[MODULE_INFO_SIZE - 1]) {
				module_id = info[0];
				lens_id = info[1];
				af_id = info[2];
				vcm_id = info[3];
				year = info[4];
				month = info[5];
				day = info[6];

				LOG_INF("module_id = 0x%x\n", module_id);
				LOG_INF("lens_id = 0x%x\n", lens_id);
				LOG_INF("af_id = 0x%x, driver_id = 0x%x\n", (af_id >> 4) & 0x0f, af_id & 0x0f);
				LOG_INF("vcm_id = 0x%x\n", vcm_id);
				LOG_INF("data = %d-%d-%d\n", year, month, day);
			} else
				LOG_INF("module info Check sum %d error! check sum = %d, sum = %d\n",
					index + 1, info[MODULE_INFO_SIZE - 1], (check % 255 + 1));
			break;
		case OTP_FLAG_INVALID:
		case OTP_FLAG_INVALID2:
			LOG_INF("module info group %d is invalid!\n", index + 1);
			break;
		default:
			break;
		}
}

static void gc13c33_otp_read_wb_info(void)
{
	kal_uint8  flag_wb = 0;
	kal_uint8  wb[WB_INFO_SIZE] = { 0 };
	kal_uint8  golden[WB_GOLDEN_INFO_SIZE] = { 0 };
	kal_uint16 checkwb = 0, checkgolden = 0;
	kal_uint8  index = 0, i = 0;

	memset(&wb, 0, WB_INFO_SIZE * sizeof(kal_uint8));
	memset(&golden, 0, WB_GOLDEN_INFO_SIZE * sizeof(kal_uint8));

	flag_wb = gc13c33_otp_read_byte(WB_INFO_FLAG_OFFSET);
	LOG_INF("flag_wb = 0x%x\n", flag_wb);

	for (index = 0; index < OTP_GROUP_CNT; index++) {
		switch ((flag_wb >> (2 * (1 - index))) & 0x03) {
		case OTP_FLAG_EMPTY:
			LOG_INF("wb unit group %d is empty!\n", index + 1);
			gc13c33_otp_data.wb_flag = gc13c33_otp_data.wb_flag | OTP_FLAG_EMPTY;
			break;
		case OTP_FLAG_VALID:
			LOG_INF("wb unit group %d is valid!\n", index + 1);
			gc13c33_otp_read_group(WB_INFO_OFFSET + WB_INFO_SIZE * 8 * index, &wb[0], WB_INFO_SIZE);
			for (i = 0; i < WB_INFO_SIZE; i++)
				LOG_INF("addr = 0x%x, data = 0x%x\n",
				WB_INFO_OFFSET +  WB_INFO_SIZE * 8 * index + i * 8, wb[i]);

			for (i = 0; i < WB_INFO_SIZE - 1; i++)
				checkwb += wb[i];

			LOG_INF("cal_checkwb = 0x%x, otp_checkwb = 0x%x\n", checkwb % 255 + 1, wb[3]);

			if ((checkwb % 255 + 1) == wb[WB_INFO_SIZE - 1]) {
				gc13c33_otp_data.rg_gain = (wb[0] | ((wb[1] & 0xf0) << 4)) > 0 ?
					(wb[0] | ((wb[1] & 0xf0) << 4)) : 0x400;
				gc13c33_otp_data.bg_gain = (((wb[1] & 0x0f) << 8) | wb[2]) > 0 ?
					(((wb[1] & 0x0f) << 8) | wb[2]) : 0x400;
				gc13c33_otp_data.wb_flag = gc13c33_otp_data.wb_flag | OTP_FLAG_VALID;
			} else
				LOG_INF("wb unit check sum %d error!\n", index + 1);
			break;
		case OTP_FLAG_INVALID:
		case OTP_FLAG_INVALID2:
			LOG_INF("wb unit group %d is invalid!\n", index + 1);
			gc13c33_otp_data.wb_flag = gc13c33_otp_data.wb_flag | OTP_FLAG_INVALID;
			break;
		default:
			break;
		}

		switch ((flag_wb >> (2 * (3 - index))) & 0x03) {
		case OTP_FLAG_EMPTY:
			LOG_INF("wb golden group %d is empty!\n", index + 1);
			gc13c33_otp_data.golden_flag = gc13c33_otp_data.golden_flag | OTP_FLAG_EMPTY;
			break;
		case OTP_FLAG_VALID:
			LOG_INF("wb golden group %d is valid!\n", index + 1);
			gc13c33_otp_read_group(WB_GOLDEN_INFO_OFFSET + WB_GOLDEN_INFO_SIZE * 8 * index,
			&golden[0], WB_GOLDEN_INFO_SIZE);
			for (i = 0; i < 4; i++)
				LOG_INF("addr = 0x%x, data = 0x%x\n",
				WB_GOLDEN_INFO_OFFSET + WB_GOLDEN_INFO_SIZE * 8 * index + i * 8, golden[i]);
			for (i = 0; i < WB_GOLDEN_INFO_SIZE - 1; i++)
				checkgolden += golden[i];

			LOG_INF("cal_checkgolden = 0x%x, otp_checkgolden = 0x%x\n", checkgolden % 255 + 1, golden[3]);

			if ((checkgolden % 255 + 1) == golden[WB_GOLDEN_INFO_SIZE - 1]) {
				gc13c33_otp_data.golden_rg = (golden[0] | ((golden[1] & 0xf0) << 4)) > 0 ?
					(golden[0] | ((golden[1] & 0xf0) << 4)) : RG_TYPICAL;
				gc13c33_otp_data.golden_bg = (((golden[1] & 0x0f) << 8) | golden[2]) > 0 ?
					(((golden[1] & 0x0f) << 8) | golden[2]) : BG_TYPICAL;
				gc13c33_otp_data.golden_flag = gc13c33_otp_data.golden_flag | OTP_FLAG_VALID;
			} else
				LOG_INF("wb golden check sum %d error!\n", index + 1);
			break;
		case OTP_FLAG_INVALID:
		case OTP_FLAG_INVALID2:
			LOG_INF("wb golden group %d is invalid!\n", index + 1);
			gc13c33_otp_data.golden_flag = gc13c33_otp_data.golden_flag | OTP_FLAG_INVALID;
			break;
		default:
			break;
		}
	}
}

static void gc13c33_otp_load_wb(void)
{
	kal_uint16 r_gain_current = 0, g_gain_current = 0, b_gain_current = 0, base_gain = 0;
	kal_uint16 r_gain = 1024, g_gain = 1024, b_gain = 1024;
	kal_uint16 rg_typical = 0, bg_typical = 0;

	if (OTP_FLAG_VALID == (gc13c33_otp_data.golden_flag & 0x01)) {
		rg_typical = gc13c33_otp_data.golden_rg;
		bg_typical = gc13c33_otp_data.golden_bg;
		LOG_INF("golden_flag = %d, rg_typical = 0x%x, bg_typical = 0x%x\n",
			gc13c33_otp_data.golden_flag, rg_typical, bg_typical);
	} else {
		rg_typical = RG_TYPICAL;
		bg_typical = BG_TYPICAL;
		LOG_INF("golden_flag = %d, rg_typical = 0x%x, bg_typical = 0x%x\n",
			gc13c33_otp_data.golden_flag, rg_typical, bg_typical);
	}

	if (OTP_FLAG_VALID == (gc13c33_otp_data.wb_flag & 0x01)) {
		r_gain_current = 2048 * rg_typical / gc13c33_otp_data.rg_gain;
		b_gain_current = 2048 * bg_typical / gc13c33_otp_data.bg_gain;
		g_gain_current = 2048;

		base_gain = (r_gain_current < b_gain_current) ? r_gain_current : b_gain_current;
		base_gain = (base_gain < g_gain_current) ? base_gain : g_gain_current;

		r_gain = 0x400 * r_gain_current / base_gain;
		g_gain = 0x400 * g_gain_current / base_gain;
		b_gain = 0x400 * b_gain_current / base_gain;
		LOG_INF("r_gain = 0x%x, g_gain = 0x%x, b_gain = 0x%x\n", r_gain, g_gain, b_gain);

		write_cmos_sensor(0x0440, g_gain & 0xff);
		write_cmos_sensor(0x0441, r_gain & 0xff);
		write_cmos_sensor(0x0442, b_gain & 0xff);
		write_cmos_sensor(0x0443, g_gain & 0xff);
		write_cmos_sensor(0x0444, g_gain & 0xff);
		write_cmos_sensor(0x0445, r_gain & 0xff);
		write_cmos_sensor(0x0446, b_gain & 0xff);
		write_cmos_sensor(0x0447, g_gain & 0xff);
		write_cmos_sensor(0x0448, g_gain & 0xff);
		write_cmos_sensor(0x0449, r_gain & 0xff);
		write_cmos_sensor(0x044a, b_gain & 0xff);
		write_cmos_sensor(0x044b, g_gain & 0xff);
		write_cmos_sensor(0x044c, g_gain & 0xff);
		write_cmos_sensor(0x044d, r_gain & 0xff);
		write_cmos_sensor(0x044e, b_gain & 0xff);
		write_cmos_sensor(0x044f, g_gain & 0xff);
		write_cmos_sensor(0x0450, (g_gain >> 8) & 0x0f);
		write_cmos_sensor(0x0451, (r_gain >> 8) & 0x0f);
		write_cmos_sensor(0x0452, (b_gain >> 8) & 0x0f);
		write_cmos_sensor(0x0453, (g_gain >> 8) & 0x0f);
		write_cmos_sensor(0x0454, (g_gain >> 8) & 0x0f);
		write_cmos_sensor(0x0455, (r_gain >> 8) & 0x0f);
		write_cmos_sensor(0x0456, (b_gain >> 8) & 0x0f);
		write_cmos_sensor(0x0457, (g_gain >> 8) & 0x0f);
		write_cmos_sensor(0x0458, (g_gain >> 8) & 0x0f);
		write_cmos_sensor(0x0459, (r_gain >> 8) & 0x0f);
		write_cmos_sensor(0x045a, (b_gain >> 8) & 0x0f);
		write_cmos_sensor(0x045b, (g_gain >> 8) & 0x0f);
		write_cmos_sensor(0x045c, (g_gain >> 8) & 0x0f);
		write_cmos_sensor(0x045d, (r_gain >> 8) & 0x0f);
		write_cmos_sensor(0x045e, (b_gain >> 8) & 0x0f);
		write_cmos_sensor(0x045f, (g_gain >> 8) & 0x0f);
	}
}
#endif

static void gc13c33_otp_debug(void)
{
#if GC13C33_OTP_DUMP

	kal_uint8 debugdata[OTP_DATA_LENGTH] = {0};
	kal_uint16 i;

	memset(&debugdata, 0, OTP_DATA_LENGTH * sizeof(kal_uint8));
	gc13c33_otp_read_group(OTP_START_ADDR, &debugdata[0], OTP_DATA_LENGTH);
	for(i = 0; i < OTP_DATA_LENGTH; i++)
		LOG_INF("addr = 0x%x, data = 0x%x\n", OTP_START_ADDR + i * 8, debugdata[i]);
#endif
}

static void gc13c33_otp_read(void)
{
	gc13c33_otp_read_dpc_info();
	gc13c33_otp_read_reg_info();
#if GC13C33_OTP_FOR_CUSTOMER
	gc13c33_otp_read_af_info();
	gc13c33_otp_read_module_info();
	gc13c33_otp_read_wb_info();
#endif
	gc13c33_otp_debug();
}

static void gc13c33_otp_update(void)
{
	gc13c33_otp_load_regs();
#if GC13C33_OTP_FOR_CUSTOMER
	gc13c33_otp_load_wb();
#endif
}

void gc13c33_otp_identify(void)
{
	LOG_INF("start");
	gc13c33_read_crosstalk_data();
	memset(&gc13c33_otp_data, 0, sizeof(gc13c33_otp_data));

	/* Read init */
	write_cmos_sensor(0x031c, 0x91);
	write_cmos_sensor(0x0a67, 0x80);
	write_cmos_sensor(0x0317, 0x08);
	write_cmos_sensor(0x0a54, 0x08);
	write_cmos_sensor(0x0313, 0x00);
	write_cmos_sensor(0x0a59, 0x40);
	write_cmos_sensor(0x0a65, 0x10);
	gc13c33_otp_read_group(GC13C33_OTP_ID_DATA_OFFSET, &gc13c33_otp_data.otp_id[0], GC13C33_OTP_ID_SIZE);
	gc13c33_otp_read();
}
void gc13c33_otp_function(void)
{
	kal_uint8 i = 0, flag = 0;
	kal_uint8 otp_id[GC13C33_OTP_ID_SIZE];
	LOG_INF("start");
	memset(&otp_id, 0, GC13C33_OTP_ID_SIZE);
	write_cmos_sensor(0x031c, 0x91);
	write_cmos_sensor(0x0a67, 0x80);
	write_cmos_sensor(0x0317, 0x08);
	write_cmos_sensor(0x0a54, 0x08);
	write_cmos_sensor(0x0313, 0x00);
	write_cmos_sensor(0x0a59, 0x40);
	write_cmos_sensor(0x0a65, 0x10);

	gc13c33_otp_read_group(GC13C33_OTP_ID_DATA_OFFSET, &otp_id[0], GC13C33_OTP_ID_SIZE);
	for (i = 0; i < GC13C33_OTP_ID_SIZE; i++) {
#if OTP_ID_DEBUG
		LOG_INF("identify_otp_id[%d]: 0x%x, doublecheck_otp_id[%d]: 0x%x\n",
		i, gc13c33_otp_data.otp_id[i], i, otp_id[i]);
#endif
		if (otp_id[i] != gc13c33_otp_data.otp_id[i]) {
			flag = 1;
			break;
		}
	}
	if (flag == 1) {
		LOG_INF("otp id mismatch, read again");
		memset(&gc13c33_otp_data, 0, sizeof(gc13c33_otp_data));
		for (i = 0; i < GC13C33_OTP_ID_SIZE; i++)
			gc13c33_otp_data.otp_id[i] = otp_id[i];
	gc13c33_otp_read();
	} else
		LOG_INF("otp id check success! load data directly!");
	gc13c33_otp_update();


}
