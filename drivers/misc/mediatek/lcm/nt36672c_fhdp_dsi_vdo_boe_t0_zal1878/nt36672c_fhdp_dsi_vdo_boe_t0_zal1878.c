/********************************************
 ** Copyright (C) 2019 OPPO Mobile Comm Corp. Ltd.
 ** ODM_HQ_EDIT
 ** File: nt36672c_fhdp_dsi_vdo_boe_t0_zal1878.c
 ** Description: Source file for LCD driver
 **          To Control LCD driver
 ** Version :1.0
 ** Date : 2019/10/14
 ** Author: Liyan@ODM_HQ.Multimedia.LCD
 ** ---------------- Revision History: --------------------------
 ** <version>    <date>          < author >              <desc>
 **  1.0           2019/10/14   Liyan@ODM_HQ   Source file for LCD driver
 ********************************************/

#define LOG_TAG "LCM_NT36672C_T0"

#ifndef BUILD_LK
#include <linux/string.h>
#include <linux/kernel.h>
#endif

#include "lcm_drv.h"

#ifdef BUILD_LK
#include <platform/upmu_common.h>
#include <platform/mt_gpio.h>
#include <platform/mt_i2c.h>
#include <platform/mt_pmic.h>
#include <string.h>
#elif defined(BUILD_UBOOT)
#include <asm/arch/mt_gpio.h>
#else
#include "disp_dts_gpio.h"
#endif

#ifdef BUILD_LK
#define LCM_LOGI(string, args...)  dprintf(0, "[LK/"LOG_TAG"]"string, ##args)
#define LCM_LOGD(string, args...)  dprintf(1, "[LK/"LOG_TAG"]"string, ##args)
#else
#define LCM_LOGI(fmt, args...)  pr_debug("[KERNEL/"LOG_TAG"]"fmt, ##args)
#define LCM_LOGD(fmt, args...)  pr_debug("[KERNEL/"LOG_TAG"]"fmt, ##args)
#endif

#define NT36672C_LCM_ID (0x09)
#define FRAMERATE_90HZ

static const unsigned int BL_MIN_LEVEL = 20;
#ifndef BUILD_LK
typedef struct LCM_UTIL_FUNCS LCM_UTIL_FUNCS;
typedef struct LCM_PARAMS LCM_PARAMS;
typedef struct LCM_DRIVER LCM_DRIVER;
#endif
static LCM_UTIL_FUNCS lcm_util;

#define SET_RESET_PIN(v)    (lcm_util.set_reset_pin((v)))
#define MDELAY(n)        (lcm_util.mdelay(n))
#define UDELAY(n)        (lcm_util.udelay(n))


#define dsi_set_cmdq_V22(cmdq, cmd, count, ppara, force_update) \
    lcm_util.dsi_set_cmdq_V22(cmdq, cmd, count, ppara, force_update)
#define dsi_set_cmdq_V2(cmd, count, ppara, force_update) \
    lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update) \
        lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd) lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums) \
        lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd) \
      lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size) \
        lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)

#ifdef CONFIG_SET_LCD_BIAS_ODM_HQ
#define SET_LCD_BIAS_EN(en, seq, value)                           lcm_util.set_lcd_bias_en(en, seq, value)
#endif

#ifndef BUILD_LK
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/i2c.h>
#include <linux/irq.h>
/* #include <linux/jiffies.h> */
/* #include <linux/delay.h> */
#include <linux/uaccess.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <soc/oppo/device_info.h>
#endif

#define MTK_GPIO_DESC_BASE 301
#define GPIO_LCD_VSP_EN (MTK_GPIO_DESC_BASE + 23)
#define GPIO_LCD_VSN_EN (MTK_GPIO_DESC_BASE + 202)

/* static unsigned char lcd_id_pins_value = 0xFF; */
static const unsigned char LCD_MODULE_ID = 0x01;
static int nt_g_gesture = 0;
extern void lcd_queue_load_tp_fw(void);
extern nvt_tp;

#define LCM_DSI_CMD_MODE    0
#define FRAME_WIDTH        (1080)
#define FRAME_HEIGHT    (2400)
#define LCM_DENSITY        (320)

#define LCM_PHYSICAL_WIDTH        (67608)
#define LCM_PHYSICAL_HEIGHT        (142728)

#define REGFLAG_DELAY        0xFFFC
#define REGFLAG_UDELAY    0xFFFB
#define REGFLAG_END_OF_TABLE    0xFFFD
#define REGFLAG_RESET_LOW    0xFFFE
#define REGFLAG_RESET_HIGH    0xFFFF

#ifndef TRUE
#define TRUE 1
#endif

#ifndef FALSE
#define FALSE 0
#endif

struct LCM_setting_table {
    unsigned int cmd;
    unsigned char count;
    unsigned char para_list[64];
};

/* Liyan@ODM.HQ.Multimedia.LCM 2019/09/19 modified for backlight remapping */
static int blmap_table[] = {
	36, 16,
	16, 22,
	17, 21,
	19, 20,
	19, 20,
	20, 17,
	22, 15,
	22, 14,
	24, 10,
	24, 8,
	26, 4,
	27, 0,
	29, 9,
	29, 9,
	30, 14,
	33, 25,
	34, 30,
	36, 44,
	37, 49,
	40, 65,
	40, 69,
	43, 88,
	46, 109,
	47, 112,
	50, 135,
	53, 161,
	53, 163,
	60, 220,
	60, 223,
	64, 257,
	63, 255,
	71, 334,
	71, 331,
	75, 375,
	80, 422,
	84, 473,
	89, 529,
	88, 518,
	99, 653,
	98, 640,
	103, 707,
	117, 878,
	115, 862,
	122, 947,
	128, 1039,
	135, 1138,
	132, 1102,
	149, 1355,
	157, 1478,
	166, 1611,
	163, 1563,
	183, 1900,
	180, 1844,
	203, 2232,
	199, 2169,
	209, 2344,
	236, 2821,
	232, 2742,
	243, 2958,
	255, 3188,
	268, 3433,
	282, 3696,
	317, 4405,
	176, 1560
};

static struct LCM_setting_table lcm_suspend_setting[] = {
    {0x28, 0, {} },
	{REGFLAG_DELAY, 10, {} },
    {0x10, 0, {} },
    {REGFLAG_DELAY, 60, {} },
};

static struct LCM_setting_table init_setting_vdo[] = {

	{0XFF, 1, {0X10}},
	{0XFB, 1, {0X01}},
	{0XB0, 1, {0X00}},
	//VESA off for C-PHY
	{0XC0, 1, {0X00}}, 
	{0XC2, 2, {0X1B,0XA0}},
	
	{0XFF, 1, {0X20}},
	{0XFB, 1, {0X01}},
	{0X01, 1, {0X66}},
	{0X06, 1, {0X3C}},
	{0X07, 1, {0X3C}},
	{0X69, 1, {0XEA}},
	{0X89, 1, {0X23}},
	{0X95, 1, {0XD8}},
	{0X96, 1, {0XD8}},
	{0XF2, 1, {0X64}},
	{0XF4, 1, {0X64}},
	{0XF6, 1, {0X64}},
	{0XF8, 1, {0X64}},
	
	//GAMMA
	//CMD2_Page0
	{0xFF, 1, {0x20}},
	{0xFB, 1, {0x01}},
	//R(+)
	{0xB0, 16, {0x00,0x00,0x00,0x2C,0x00,0x61,0x00,0x87,0x00,0xA5,0x00,0xBE,0x00,0xD3,0x00,0xE6}},
	{0xB1, 16, {0x00,0xF7,0x01,0x2F,0x01,0x57,0x01,0x98,0x01,0xC4,0x02,0x0C,0x02,0x42,0x02,0x44}},
	{0xB2, 16, {0x02,0x79,0x02,0xB4,0x02,0xDB,0x03,0x0C,0x03,0x2F,0x03,0x56,0x03,0x67,0x03,0x77}},
	{0xB3, 14, {0x03,0x8A,0x03,0xA1,0x03,0xB9,0x03,0xCB,0x03,0xCD,0x03,0xD8,0x00,0x00}},
	//G(+)
	{0xB4, 16, {0x00,0x00,0x00,0x2C,0x00,0x61,0x00,0x87,0x00,0xA5,0x00,0xBE,0x00,0xD3,0x00,0xE6}},
	{0xB5, 16, {0x00,0xF7,0x01,0x2F,0x01,0x57,0x01,0x98,0x01,0xC4,0x02,0x0C,0x02,0x42,0x02,0x44}},
	{0xB6, 16, {0x02,0x79,0x02,0xB4,0x02,0xDB,0x03,0x0C,0x03,0x2F,0x03,0x56,0x03,0x67,0x03,0x77}},
	{0xB7, 14, {0x03,0x8A,0x03,0xA1,0x03,0xB9,0x03,0xCB,0x03,0xCD,0x03,0xD8,0x00,0x00}},
	//B(+)
	{0xB8, 16, {0x00,0x00,0x00,0x2C,0x00,0x61,0x00,0x87,0x00,0xA5,0x00,0xBE,0x00,0xD3,0x00,0xE6}},
	{0xB9, 16, {0x00,0xF7,0x01,0x2F,0x01,0x57,0x01,0x98,0x01,0xC4,0x02,0x0C,0x02,0x42,0x02,0x44}},
	{0xBA, 16, {0x02,0x79,0x02,0xB4,0x02,0xDB,0x03,0x0C,0x03,0x2F,0x03,0x56,0x03,0x67,0x03,0x77}},
	{0xBB, 14, {0x03,0x8A,0x03,0xA1,0x03,0xB9,0x03,0xCB,0x03,0xCD,0x03,0xD8,0x00,0x00}},
	//CMD2_Page1
	{0xFF, 1, {0x21}},
	{0xFB, 1, {0x01}},
	//R(-)
	{0xB0, 16, {0x00,0x00,0x00,0x2C,0x00,0x61,0x00,0x87,0x00,0xA5,0x00,0xBE,0x00,0xD3,0x00,0xE6}},
	{0xB1, 16, {0x00,0xF7,0x01,0x2F,0x01,0x57,0x01,0x98,0x01,0xC4,0x02,0x0C,0x02,0x42,0x02,0x44}},
	{0xB2, 16, {0x02,0x79,0x02,0xB4,0x02,0xDB,0x03,0x0C,0x03,0x2F,0x03,0x56,0x03,0x67,0x03,0x77}},
	{0xB3, 14, {0x03,0x8A,0x03,0xA1,0x03,0xB9,0x03,0xCB,0x03,0xCD,0x03,0xD8,0x00,0x00}},
	//G(-)
	{0xB4, 16, {0x00,0x00,0x00,0x2C,0x00,0x61,0x00,0x87,0x00,0xA5,0x00,0xBE,0x00,0xD3,0x00,0xE6}},
	{0xB5, 16, {0x00,0xF7,0x01,0x2F,0x01,0x57,0x01,0x98,0x01,0xC4,0x02,0x0C,0x02,0x42,0x02,0x44}},
	{0xB6, 16, {0x02,0x79,0x02,0xB4,0x02,0xDB,0x03,0x0C,0x03,0x2F,0x03,0x56,0x03,0x67,0x03,0x77}},
	{0xB7, 14, {0x03,0x8A,0x03,0xA1,0x03,0xB9,0x03,0xCB,0x03,0xCD,0x03,0xD8,0x00,0x00}},
	//B(-)
	{0xB8, 16, {0x00,0x00,0x00,0x2C,0x00,0x61,0x00,0x87,0x00,0xA5,0x00,0xBE,0x00,0xD3,0x00,0xE6}},
	{0xB9, 16, {0x00,0xF7,0x01,0x2F,0x01,0x57,0x01,0x98,0x01,0xC4,0x02,0x0C,0x02,0x42,0x02,0x44}},
	{0xBA, 16, {0x02,0x79,0x02,0xB4,0x02,0xDB,0x03,0x0C,0x03,0x2F,0x03,0x56,0x03,0x67,0x03,0x77}},
	{0xBB, 14, {0x03,0x8A,0x03,0xA1,0x03,0xB9,0x03,0xCB,0x03,0xCD,0x03,0xD8,0x00,0x00}},
	
	{0XFF, 1, {0X24}},
	{0XFB, 1, {0X01}},
	{0X07, 1, {0XA3}},
	{0X08, 1, {0XA3}},
	{0X09, 1, {0X00}},
	{0X0A, 1, {0X22}},
	{0X0B, 1, {0X0F}},
	{0X0C, 1, {0X0F}},
	{0X0D, 1, {0X17}},
	{0X0E, 1, {0X15}},
	{0X0F, 1, {0X13}},
	{0X10, 1, {0X2D}},
	{0X11, 1, {0X2F}},
	{0X12, 1, {0X1C}},
	{0X13, 1, {0X24}},
	{0X14, 1, {0X24}},
	{0X15, 1, {0X0B}},
	{0X16, 1, {0X0C}},
	{0X17, 1, {0X01}},
	{0X1F, 1, {0XA3}},
	{0X20, 1, {0XA3}},
	{0X21, 1, {0X00}},
	{0X22, 1, {0X22}},
	{0X23, 1, {0X0F}},
	{0X24, 1, {0X0F}},
	{0X25, 1, {0X17}},
	{0X26, 1, {0X15}},
	{0X27, 1, {0X13}},
	{0X28, 1, {0X2C}},
	{0X29, 1, {0X2E}},
	{0X2A, 1, {0X1C}},
	{0X2B, 1, {0X24}},
	{0X2D, 1, {0X24}},
	{0X2F, 1, {0X0B}},
	{0X30, 1, {0X0C}},
	{0X31, 1, {0X01}},
	{0X32, 1, {0X09}},
	{0X33, 1, {0X02}},
	{0X34, 1, {0X00}},
	{0X35, 1, {0X01}},
	{0X36, 1, {0X01}},
	{0X37, 1, {0X0C}},
	{0X38, 1, {0X0A}},
	
	//20191011
	{0X4E, 1, {0X50}},
	{0X4F, 1, {0X50}},
	{0X53, 1, {0X50}},
	
	{0X7B, 1, {0X11}},
	{0X7C, 1, {0X00}},
	{0X7D, 1, {0X04}},
	{0X80, 1, {0X04}},
	{0X81, 1, {0X04}},
	{0X82, 1, {0X13}},
	{0X84, 1, {0X31}},
	{0X85, 1, {0X00}},
	{0X86, 1, {0X00}},
	{0X87, 1, {0X00}},
	{0X90, 1, {0X13}},
	{0X92, 1, {0X31}},
	{0X93, 1, {0X00}},
	{0X94, 1, {0X00}},
	{0X95, 1, {0X00}},
	{0X9C, 1, {0XF4}},
	{0X9D, 1, {0X01}},
	{0XA0, 1, {0X11}},
	{0XA2, 1, {0X11}},
	{0XA4, 1, {0X04}},
	{0XA5, 1, {0X04}},
	{0XC9, 1, {0X00}},
	{0XD9, 1, {0X80}},
	
	{0XFF, 1, {0X25}},
	{0XFB, 1, {0X01}},
#ifdef FRAMERATE_90HZ
	{0x18, 1, {0x20}}, // (0x21 for 60HZ, 0x20 for 90HZ)
#else
	{0x18, 1, {0x21}}, // (0x21 for 60HZ, 0x20 for 90HZ)
#endif
	{0X19, 1, {0X04}},
	{0X66, 1, {0XD8}},
	{0X67, 1, {0X01}},
	{0X68, 1, {0X58}},
	{0X69, 1, {0X10}},
	{0X6B, 1, {0X00}},
	{0X6C, 1, {0X15}},
	{0X71, 1, {0X1D}},
	{0X77, 1, {0X62}},
	{0X7E, 1, {0X15}},
	{0X7F, 1, {0X00}},
	{0X84, 1, {0X6D}},
	{0X8D, 1, {0X00}},
	{0XC0, 1, {0XD5}},
	{0XC1, 1, {0X11}},
	{0XC3, 1, {0X00}},
	{0XC4, 1, {0X11}},
	{0XC5, 1, {0X11}},
	{0XC6, 1, {0X11}},
	{0XD6, 1, {0X80}},
	{0XD7, 1, {0X02}},
	{0XDA, 1, {0X02}},
	{0XDD, 1, {0X02}},
	{0XE0, 1, {0X02}},
	{0XEF, 1, {0X00}},
	{0XF0, 1, {0X00}},
	{0XF1, 1, {0X04}},
	
	{0XFF, 1, {0X26}},
	{0XFB, 1, {0X01}},
	{0X00, 1, {0X00}},
	
	// 10/13 Modify}},
	//{0X01, 1, {0XF4}},
	//{0X02, 1, {0XF4}},
	{0X01, 1, {0XEF}},
	{0X02, 1, {0XEF}},
	{0X03, 1, {0X00}},
	//{0X04, 1, {0XF4}},
	{0X04, 1, {0XEF}},
	
	{0X05, 1, {0X08}},
	{0X06, 1, {0X25}},
	{0X07, 1, {0X25}},
	{0X08, 1, {0X25}},
	{0X14, 1, {0X06}},
	{0X15, 1, {0X01}},
	{0X74, 1, {0XAF}},
	{0X81, 1, {0X11}},
	{0X83, 1, {0X04}},
	{0X84, 1, {0X03}},
	{0X85, 1, {0X01}},
	{0X86, 1, {0X03}},
	{0X87, 1, {0X01}},
	{0X88, 1, {0X12}},
	{0X8A, 1, {0X1A}},
	{0X8B, 1, {0X11}},
	{0X8C, 1, {0X24}},
	{0X8E, 1, {0X42}},
	{0X8F, 1, {0X11}},
	{0X90, 1, {0X11}},
	{0X91, 1, {0X11}},
	{0X9A, 1, {0X80}},
	{0X9B, 1, {0X42}},
	{0X9C, 1, {0X00}},
	{0X9D, 1, {0X00}},
	{0X9E, 1, {0X00}},
	
	{0XFF, 1, {0X27}},
	{0XFB, 1, {0X01}},
	{0X01, 1, {0X60}},
	{0X20, 1, {0X81}},
	{0X21, 1, {0XEA}},
	{0X25, 1, {0X82}},
	{0X26, 1, {0X1F}},
	{0X6E, 1, {0X9A}},
	{0X6F, 1, {0X78}},
	{0X70, 1, {0X00}},
	{0X71, 1, {0X00}},
	{0X72, 1, {0X00}},
	{0X73, 1, {0X00}},
	{0X74, 1, {0X00}},
	{0X75, 1, {0X00}},
	{0X76, 1, {0X00}},
	{0X77, 1, {0X00}},
	{0X7D, 1, {0X09}},
	{0X7E, 1, {0X69}},
	{0X7F, 1, {0X03}},
	{0X80, 1, {0X23}},
	{0X82, 1, {0X09}},
	{0X83, 1, {0X69}},
	{0X88, 1, {0X02}},
	{0XE3, 1, {0X02}},
	{0XE4, 1, {0XE0}},
	{0XE9, 1, {0X03}},
	{0XEA, 1, {0X2F}},
	
	{0XFF, 1, {0X2A}},
	{0XFB, 1, {0X01}},
	{0x03, 1, {0x20}},
	{0x00, 1, {0x91}},
	{0X06, 1, {0X0D}},
	{0X07, 1, {0X3A}},
	{0X0A, 1, {0X60}},
	{0X0C, 1, {0X06}},
	{0X0D, 1, {0X40}},
	{0X0E, 1, {0X02}},
	{0X0F, 1, {0X01}},
	{0X11, 1, {0X55}},
	
	// 10/13 Modify
	{0X15, 1, {0X0F}},
	//{0X16, 1, {0X4C}},
	{0X16, 1, {0X00}},
	{0X19, 1, {0X0E}},
	//{0X1A, 1, {0XC5}},
	{0X1A, 1, {0X79}},
	
	{0X1B, 1, {0X14}},
	{0X1D, 1, {0X36}},
	{0X1E, 1, {0X54}},
	{0X1F, 1, {0X54}},
	{0X20, 1, {0X54}},
	{0X27, 1, {0X81}},
	{0X28, 1, {0X78}},
	{0X29, 1, {0X05}},
	{0X2A, 1, {0X0E}},
	{0X2F, 1, {0X00}},
	{0X30, 1, {0X43}},
	{0X34, 1, {0XFF}},
	
	// 10/13 Modify
	//{0X35, 1, {0X29}},
	{0X35, 1, {0X28}},
	
	{0X36, 1, {0X8C}},
	{0X37, 1, {0XFB}},
	
	// 10/13 Modify}},
	//{0X38, 1, {0X2B}},
	{0X38, 1, {0X2A}},
	
	{0X39, 1, {0X8A}},
	{0X3A, 1, {0X43}},
	{0X46, 1, {0X40}},
	{0X47, 1, {0X02}},
	{0X4A, 1, {0XEF}},
	{0X4E, 1, {0X0F}},
	{0X4F, 1, {0X4C}},
	{0X52, 1, {0X0E}},
	{0X53, 1, {0XC5}},
	{0X54, 1, {0X14}},
	{0X56, 1, {0X36}},
	{0X57, 1, {0X7F}},
	{0X58, 1, {0X7F}},
	{0X59, 1, {0X7F}},
	{0X88, 1, {0X72}},
	{0XEE, 1, {0X11}},
	{0XF0, 1, {0X5E}},
	{0XF1, 1, {0XF8}},
	{0XF5, 1, {0X01}},
	
	{0XFF, 1, {0X2C}},
	{0XFB, 1, {0X01}},
	
	//20191010
	{0X03, 1, {0X1D}},
	{0X04, 1, {0X1D}},
	{0X05, 1, {0X1D}},
	
	//{0X03, 1, {0X11}},
	//{0X04, 1, {0X11}},
	//{0X05, 1, {0X11}},
	
	{0X0D, 1, {0X01}},
	{0X0E, 1, {0X01}},
	
	//20191010
	{0X17, 1, {0X7A}},
	{0X18, 1, {0X7A}},
	{0X19, 1, {0X7A}},
	
	//{0X17, 1, {0X44}},
	//{0X18, 1, {0X44}},
	//{0X19, 1, {0X44}},
	
	{0X2D, 1, {0XAF}},
	{0X2F, 1, {0X00}},
	{0X30, 1, {0XF4}},
	{0X31, 1, {0XF4}},
	{0X32, 1, {0X00}},
	{0X33, 1, {0XF4}},
	{0X35, 1, {0X30}},
	{0X36, 1, {0X30}},
	{0X37, 1, {0X30}},
	{0X4D, 1, {0X11}},
	{0X4E, 1, {0X04}},
	{0X4F, 1, {0X3D}},
	
	//{0XFF, 1, {0XE0}},
	//{0XFB, 1, {0X01}},
	//{0X25, 1, {0X02}},
	//{0X4E, 1, {0X02}},
	
	{0XFF, 1, {0XF0}},
	{0XFB, 1, {0X01}},
	{0X5A, 1, {0X00}},
	{0XA0, 1, {0X08}},
	
	{0XFF, 1, {0XD0}},
	{0XFB, 1, {0X01}},
	{0X09, 1, {0XAD}},
	//{0xFF, 1, {0x23}},/* 11bit PWM */
	//{0xFB, 1, {0x01}},/* 11bit PWM */
	//{0x00, 1, {0x68}},/* 11bit PWM */

	{0XFF, 1, {0x10}},
	//{0x35, 1, {0x00}}, // TE
	{0x53, 1, {0x24}},
	{0x55, 1, {0x00}},
	{0x11, 1, {0x00}},
	{REGFLAG_DELAY, 100, {} },
	{0x29, 1, {0x00}},
	{REGFLAG_DELAY, 40, {} }
	/* bist mode */
	//{0XFF, 1, {0X20}},
	//{0XFB, 1, {0X01}},
	//{0X86, 1, {0X03}},
};

static struct LCM_setting_table bl_level[] = {
    {0x51, 1, {0xFF} },
    {REGFLAG_END_OF_TABLE, 0x00, {} }
};
static struct LCM_setting_table lcm_cabc_enter_setting[] = {
    {0x53, 1, {0x2C}},
    {0x55, 1, {0x02}},
    {0x53, 1, {0x24}},
    {REGFLAG_END_OF_TABLE, 0x00, {} }
};
static struct LCM_setting_table lcm_cabc_exit_setting[] = {
    {0x53, 1, {0x2C}},
    {0x55, 1, {0x00}},
    {0x53, 1, {0x24}},
    {REGFLAG_END_OF_TABLE, 0x00, {} }
};

static void push_table(void *cmdq, struct LCM_setting_table *table,
    unsigned int count, unsigned char force_update)
{
    unsigned int i;
    unsigned int cmd;

    for (i = 0; i < count; i++) {
        cmd = table[i].cmd;

        switch (cmd) {

        case REGFLAG_DELAY:
            if (table[i].count <= 10)
                MDELAY(table[i].count);
            else
                MDELAY(table[i].count);
            break;

        case REGFLAG_UDELAY:
            UDELAY(table[i].count);
            break;

        case REGFLAG_END_OF_TABLE:
            break;

        default:
            dsi_set_cmdq_V22(cmdq, cmd,
                table[i].count,
                table[i].para_list,
                force_update);
        }
    }
}


static void lcm_set_util_funcs(const LCM_UTIL_FUNCS *util)
{
    memcpy(&lcm_util, util, sizeof(LCM_UTIL_FUNCS));
}


static void lcm_get_params(LCM_PARAMS *params)
{
    memset(params, 0, sizeof(LCM_PARAMS));

    params->type = LCM_TYPE_DSI;

    params->width = FRAME_WIDTH;
    params->height = FRAME_HEIGHT;
    params->physical_width = LCM_PHYSICAL_WIDTH/1000;
    params->physical_height = LCM_PHYSICAL_HEIGHT/1000;
    params->physical_width_um = LCM_PHYSICAL_WIDTH;
    params->physical_height_um = LCM_PHYSICAL_HEIGHT;
    params->density = LCM_DENSITY;

	params->dsi.IsCphy = 1;
#if (LCM_DSI_CMD_MODE)
    params->dsi.mode = CMD_MODE;
    params->dsi.switch_mode = SYNC_PULSE_VDO_MODE;
    //lcm_dsi_mode = CMD_MODE;
#else
    params->dsi.mode = SYNC_PULSE_VDO_MODE;//SYNC_PULSE_VDO_MODE;
    params->dsi.switch_mode = CMD_MODE;
    //lcm_dsi_mode = SYNC_PULSE_VDO_MODE;
#endif
    //LCM_LOGI("lcm_get_params lcm_dsi_mode %d\n", lcm_dsi_mode);
    params->dsi.switch_mode_enable = 0;

    /* DSI */
    /* Command mode setting */
    params->dsi.LANE_NUM = LCM_THREE_LANE;//LCM_THREE_LANE;
    /* The following defined the fomat for data coming from LCD engine. */
    params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
    params->dsi.data_format.trans_seq = LCM_DSI_TRANS_SEQ_MSB_FIRST;
    params->dsi.data_format.padding = LCM_DSI_PADDING_ON_LSB;
    params->dsi.data_format.format = LCM_DSI_FORMAT_RGB888;

    /* Highly depends on LCD driver capability. */
    params->dsi.packet_size = 256;
    /* video mode timing */
    params->dsi.PS = LCM_PACKED_PS_24BIT_RGB888;

    params->dsi.vertical_sync_active = 10;
    params->dsi.vertical_backporch = 10;
    params->dsi.vertical_frontporch = 54;
    //params->dsi.vertical_frontporch_for_low_power = 540;
    params->dsi.vertical_active_line = FRAME_HEIGHT;

    params->dsi.horizontal_sync_active = 20;
    params->dsi.horizontal_backporch = 22;
    params->dsi.horizontal_frontporch = 256;
    params->dsi.horizontal_active_pixel = FRAME_WIDTH;
    params->dsi.ssc_disable = 1;
#ifndef CONFIG_FPGA_EARLY_PORTING
#if (LCM_DSI_CMD_MODE)
    /* this value must be in MTK suggested table */
    params->dsi.PLL_CLOCK = 540;
#else
    /* this value must be in MTK suggested table */
#ifdef FRAMERATE_90HZ
    params->dsi.PLL_CLOCK = 540;
#else
    params->dsi.PLL_CLOCK = 346;
#endif
#endif
    //params->dsi.PLL_CK_CMD = 220;
    //params->dsi.PLL_CK_VDO = 255;
#else
    params->dsi.pll_div1 = 0;
    params->dsi.pll_div2 = 0;
    params->dsi.fbk_div = 0x1;
#endif
    /* clk continuous video mode */
    params->dsi.cont_clock = 0;

    params->dsi.clk_lp_per_line_enable = 0;
    params->dsi.esd_check_enable = 0;
    params->dsi.customization_esd_check_enable = 1;
    params->dsi.lcm_esd_check_table[0].cmd = 0x0A;
    params->dsi.lcm_esd_check_table[0].count = 1;
    params->dsi.lcm_esd_check_table[0].para_list[0] = 0x9C;
/* Liyan@ODM.HQ.Multimedia.LCM 2019/09/19 modified for backlight remapping */
	params->blmap = blmap_table;
	params->blmap_size = sizeof(blmap_table)/sizeof(blmap_table[0]);
	params->brightness_max = 2047;
	params->brightness_min = 6;
	/* liyan@ODM.Multimedia.LCD  2019/10/22 modify for t3-PREBEGIN */
	params->dsi.HS_ZERO = 48;
	/* Liyan@ODM.HQ.Multimedia.LCM 2019/11/07 add for lcd devinfo */
	register_device_proc("lcd", "nt36672c_dpt", "boe vdo mode");
}

static void lcm_init_power(void)
{
	LCM_LOGI("%s: enter\n", __func__);
	if ((!gpio_get_value(GPIO_LCD_VSP_EN)) && (!gpio_get_value(GPIO_LCD_VSN_EN))) { //when vsp and vsn is not enable
		LCM_LOGI("%s: set lcd bias on\n", __func__);
#ifdef CONFIG_SET_LCD_BIAS_ODM_HQ
		SET_LCD_BIAS_EN(ON, VSP_FIRST_VSN_AFTER, 5500);  //open lcd bias
		MDELAY(10);
#endif
	}
	LCM_LOGI("%s: exit\n", __func__);
}

static void lcm_suspend_power(void)
{
	LCM_LOGI("%s: enter\n", __func__);
	if (nt_g_gesture <= 0) { //when touch gesture is not enable
		LCM_LOGI("%s: set lcd bias off\n", __func__);
#ifdef CONFIG_SET_LCD_BIAS_ODM_HQ
		SET_LCD_BIAS_EN(OFF, VSN_FIRST_VSP_AFTER, 5500);  //close lcd bias
		MDELAY(10);
#endif
	}
	LCM_LOGI("%s: exit\n", __func__);
}

static void lcm_resume_power(void)
{
	LCM_LOGI("%s: enter\n", __func__);
	lcm_init_power();
	LCM_LOGI("%s: exit\n", __func__);
}

static void lcm_init(void)
{
    int size;
	LCM_LOGI("%s: enter\n", __func__);

	SET_RESET_PIN(1);
	MDELAY(10);
	SET_RESET_PIN(0);
	MDELAY(10);
	SET_RESET_PIN(1);
	MDELAY(10);

       if(nvt_tp)
               lcd_queue_load_tp_fw();

    size = sizeof(init_setting_vdo) /
        sizeof(struct LCM_setting_table);
    push_table(NULL, init_setting_vdo, size, 1);

	LCM_LOGI("%s: exit\n", __func__);
}

static void lcm_suspend(void)
{
    LCM_LOGI("%s: enter\n", __func__);
    push_table(NULL, lcm_suspend_setting,
        sizeof(lcm_suspend_setting) / sizeof(struct LCM_setting_table),
        1);
    MDELAY(10);

    LCM_LOGI("%s: exit\n", __func__);
}

static void lcm_resume(void)
{
    LCM_LOGI("%s: enter\n", __func__);
    lcm_init();
    LCM_LOGI("%s: exit\n", __func__);
}

#ifdef BUILD_LK
static unsigned int lcm_compare_id(void)
{
}
#endif

static void lcm_setbacklight_cmdq(void *handle, unsigned int level)
{

    LCM_LOGI("%s,nt36672c backlight: level = %d\n", __func__, level);
#if 0 /* 11bit */
	if (level > 2047){
		level = 2047;
	}else if(level > 0 && level < 6){
		level = 6;
	}
	bl_level[0].para_list[0] = (level >> 8) & 0x07;
	bl_level[0].para_list[1] = level & 0xFF;
#else /* 8bit */
		if (level > 2047) {
			level = 255;
		} else if (level > 0 && level < 10) {
			level = 10;
		} else { /* change level_11bit to level_8bit */
			level = level >> 3;
		}
		bl_level[0].para_list[0] = level;
#endif
    push_table(handle,
        bl_level,
        sizeof(bl_level) / sizeof(struct LCM_setting_table),
        1);
}

static void lcm_set_cabc_mode_cmdq(void *handle, unsigned int level)
{
    LCM_LOGI("%s [lcd] cabc_mode is %d \n", __func__, level);
    if (level) {
        push_table(handle,lcm_cabc_enter_setting, sizeof(lcm_cabc_enter_setting) / sizeof(struct LCM_setting_table), 1);
    } else {
        push_table(handle,lcm_cabc_exit_setting, sizeof(lcm_cabc_exit_setting) / sizeof(struct LCM_setting_table), 1);
    }
}

LCM_DRIVER nt36672c_fhdp_dsi_vdo_boe_t0_zal1878_lcm_drv = {
    .name = "nt36672c_boe_t0",
    .set_util_funcs = lcm_set_util_funcs,
    .get_params = lcm_get_params,
    .init = lcm_init,
    .suspend = lcm_suspend,
    .resume = lcm_resume,
#ifdef BUILD_LK
    .compare_id = lcm_compare_id,
#endif
    .init_power = lcm_init_power,
    .suspend_power = lcm_suspend_power,
    .resume_power = lcm_resume_power,
    .set_backlight_cmdq = lcm_setbacklight_cmdq,
    .set_cabc_mode_cmdq = lcm_set_cabc_mode_cmdq,
};
