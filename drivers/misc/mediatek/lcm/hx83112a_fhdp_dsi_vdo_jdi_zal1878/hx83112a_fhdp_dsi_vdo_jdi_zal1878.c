/********************************************
 ** Copyright (C) 2019 OPPO Mobile Comm Corp. Ltd.
 ** ODM_HQ_EDIT
 ** File: hx83112a_fhdp_dsi_vdo_jdi_zal1878.c
 ** Description: Source file for LCD driver
 **          To Control LCD driver
 ** Version :1.0
 ** Date : 2019/08/12
 ** Author: Liyan@ODM_HQ.Multimedia.LCD
 ** ---------------- Revision History: --------------------------
 ** <version>    <date>          < author >              <desc>
 **  1.0           2019/08/12   Liyan@ODM_HQ   Source file for LCD driver
 ********************************************/

#define LOG_TAG "LCM_HX83112A"

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

#define HX83112A_LCM_ID (0x05)

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
#endif

#define MTK_GPIO_DESC_BASE 301
#define GPIO_LCD_VSP_EN (MTK_GPIO_DESC_BASE + 23)
#define GPIO_LCD_VSN_EN (MTK_GPIO_DESC_BASE + 202)

/* static unsigned char lcd_id_pins_value = 0xFF; */
static const unsigned char LCD_MODULE_ID = 0x01;
extern int g_gesture;

#define LCM_DSI_CMD_MODE    0
#define FRAME_WIDTH        (1080)
#define FRAME_HEIGHT    (2340)
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
	{REGFLAG_DELAY, 50, {} },
    {0x10, 0, {} },
    {REGFLAG_DELAY, 100, {} },
};
#if 0
static struct LCM_setting_table init_setting_vdo_otp[] = {
	{0xB9, 3, {0x83, 0x11, 0x2A}},
	{0xB1, 8, {0x08, 0x27, 0x27, 0x80, 0x80, 0x4B, 0x4E, 0xCC}},
	{0xBD, 1, {0x01}},
	{0xB1, 7, {0x64, 0x01, 0x09, 0x20, 0x50, 0x00, 0x00}},
	{0xBF, 2, {0x1A, 0xAF }},
	{0xBD, 1, {0x00}},
	{0xB2, 15, {0x00, 0x02, 0x00, 0x90, 0x24, 0x00, 0x02, 0x20, 0xE9, 0x11,
			  0x11, 0x00, 0x15, 0xA3, 0x87}},
	{0xD2, 2, {0x2C, 0x2C}},
	{0xB4, 27, {0x66, 0x66, 0xB0, 0x70, 0xC7, 0xDC, 0x3F, 0xAA, 0x01, 0x01,
			  0x00, 0xC5, 0x00, 0xFF, 0x00, 0xFF, 0x03, 0x00, 0x02, 0x03,
			  0x00, 0x31, 0x05, 0x08, 0x09, 0x00, 0x31}},
	{0xBD, 1, {0x02}},
	{0xB4, 9, {0x00, 0x92, 0x12, 0x11, 0x88, 0x12, 0x12, 0x00, 0x70}},
	{0xBD, 1, {0x00}},
	{0xB6, 3, {0x81, 0x81, 0xE3}},
	{0xC1, 1, {0x01}},
	{0xBD, 1, {0x01}},
	{0xC1, 57, {0xFF, 0xFA, 0xF5, 0xF1, 0xEC, 0xE8, 0xE3, 0xDA, 0xD6, 0xD1,
			  0xCD, 0xC8, 0xC3, 0xBF, 0xBA, 0xB5, 0xB1, 0xAC, 0xA8, 0x9E,
			  0x96, 0x8D, 0x85, 0x7D, 0x75, 0x6D, 0x65, 0x5E, 0x57, 0x4F,
			  0x48, 0x42, 0x3B, 0x33, 0x2C, 0x25, 0x1E, 0x18, 0x11, 0x0A,
			  0x06, 0x05, 0x03, 0x01, 0x00, 0x1C, 0x9B, 0x30, 0x8F, 0x23,
			  0x3A, 0xFD, 0x04, 0x26, 0xDA, 0xF3, 0x00}},
	{0xBD, 1, {0x02}},
	{0xC1, 57, {0xFF, 0xFA, 0xF5, 0xF1, 0xEC, 0xE8, 0xE3, 0xDA, 0xD6, 0xD1,
			  0xCD, 0xC8, 0xC3, 0xBF, 0xBA, 0xB5, 0xB1, 0xAC, 0xA8, 0x9E,
			  0x96, 0x8D, 0x85, 0x7D, 0x75, 0x6D, 0x65, 0x5E, 0x57, 0x4F,
			  0x48, 0x42, 0x3B, 0x33, 0x2C, 0x25, 0x1E, 0x18, 0x11, 0x0A,
			  0x06, 0x05, 0x03, 0x01, 0x00, 0x1C, 0x9B, 0x30, 0x8F, 0x23,
			  0x3A, 0xFD, 0x04, 0x26, 0xDA, 0xF3, 0x00}},
	{0xBD, 1, {0x03}},
	{0xC1, 57, {0xFF, 0xFA, 0xF5, 0xF1, 0xEC, 0xE8, 0xE3, 0xDA, 0xD6, 0xD1,
			  0xCD, 0xC8, 0xC3, 0xBF, 0xBA, 0xB5, 0xB1, 0xAC, 0xA8, 0x9E,
			  0x96, 0x8D, 0x85, 0x7D, 0x75, 0x6D, 0x65, 0x5E, 0x57, 0x4F,
			  0x48, 0x42, 0x3B, 0x33, 0x2C, 0x25, 0x1E, 0x18, 0x11, 0x0A,
			  0x06, 0x05, 0x03, 0x01, 0x00, 0x1C, 0x9B, 0x30, 0x8F, 0x23,
			  0x3A, 0xFD, 0x04, 0x26, 0xDA, 0xF3, 0x00}},
	{0xBD, 1, {0x00}},
	{0xC0, 2, {0x23, 0x23}},
	{0xCC, 1, {0x08}},
	{0xD3, 43, {0xC0, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x00, 0x00, 0x03,
			  0x47, 0x23, 0x34, 0x05, 0x05, 0x05, 0x05, 0x32, 0x10, 0x03,
			  0x00, 0x03, 0x32, 0x10, 0x07, 0x00, 0x07, 0x32, 0x10, 0x01,
			  0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x0C, 0x11,
			  0x09, 0x36, 0x00}},
	{0xBD, 1, {0x01}},
	{0xD3, 8, {0x00, 0x00, 0x19, 0x00, 0x00, 0x4A, 0x00, 0xA1}},
	{0xBD, 1, {0x00}},
	{0xD5, 48, {0x18, 0x18, 0x19, 0x18, 0x18, 0x18, 0x18, 0x20, 0x18, 0x18,
			  0x10, 0x10, 0x18, 0x18, 0x18, 0x18, 0x03, 0x03, 0x02, 0x02,
			  0x01, 0x01, 0x00, 0x00, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18,
			  0x18, 0x18, 0x18, 0x18, 0x31, 0x31, 0x2F, 0x2F, 0x30, 0x30,
			  0x18, 0x18, 0x35, 0x35, 0x36, 0x36, 0x37, 0x37}},
	{0xD6, 48, {0x18, 0x18, 0x19, 0x18, 0x18, 0x18, 0x19, 0x20, 0x18, 0x18,
			  0x10, 0x10, 0x18, 0x18, 0x18, 0x18, 0x00, 0x00, 0x01, 0x01,
			  0x02, 0x02, 0x03, 0x03, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18,
			  0x18, 0x18, 0x18, 0x18, 0x31, 0x31, 0x2F, 0x2F, 0x30, 0x30,
			  0x18, 0x18, 0x35, 0x35, 0x36, 0x36, 0x37, 0x37}},
	{0xD8, 24, {0xAA, 0xAA, 0xFF, 0xAA, 0xBF, 0xAA, 0xAA, 0xAA, 0xFF, 0xAA,
			  0xBF, 0xAA, 0xAA, 0xAA, 0x55, 0xAA, 0xBF, 0xAA, 0xAA, 0xAA,
			  0x55, 0xAA, 0xBF, 0xAA}},
	{0xBD, 1, {0x01}},
	{0xD8, 24, {0xAA, 0xAA, 0xFF, 0xAA, 0xBF, 0xAA, 0xAA, 0xAA, 0xFF, 0xAA,
			  0xBF, 0xAA, 0xAA, 0xAA, 0xFF, 0xAA, 0xBF, 0xAA, 0xAA, 0xAA,
			  0xFF, 0xAA, 0xBF, 0xAA}},
	{0xBD, 1, {0x02}},
	{0xD8, 12, {0xAA, 0xAA, 0xFF, 0xAA, 0xBF, 0xAA, 0xAA, 0xAA, 0xFF, 0xAA,
			  0xBF, 0xAA}},
	{0xBD, 1, {0x03}},
	{0xD8, 24, {0xBA, 0xAA, 0xAA, 0xAA, 0xBF, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA,
			  0xBF, 0xAA, 0xBA, 0xAA, 0xAA, 0xAA, 0xBF, 0xAA, 0xAA, 0xAA,
			  0xAA, 0xAA, 0xBF, 0xAA}},
	{0xBD, 1, {0x00}},
	{0xE7, 24, {0x0E, 0x0E, 0x1E, 0x66, 0x1E, 0x66, 0x00, 0x32, 0x02, 0x02,
			  0x00, 0x00, 0x02, 0x02, 0x02, 0x05, 0x14, 0x14, 0x32, 0xB9,
			  0x23, 0xB9, 0x08, 0x03}},
	{0xBD, 1, {0x01}},
	{0xE7, 10, {0x02, 0x07, 0xA8, 0x01, 0xA8, 0x0D, 0xA1, 0x0E, 0x01, 0x01}},
	{0xBD, 1, {0x02}},
	{0xE7, 29, {0x00, 0x00, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			  0x00, 0x00, 0x0E, 0x00, 0x00, 0x00, 0x00, 0x22, 0x00}},
	{0xBD, 1, {0x00}},
	{0xC6, 2, {0x77, 0xBF}},
	{0xC7, 6, {0x70, 0x00, 0x04, 0xE0, 0x33, 0x00}},
	{0xE9, 1, {0xC3}},
	{0xCB, 2, {0xD1, 0xD3}},
	{0xE9, 1, {0x3F}},
	{0xBA, 3, {0x73, 0x03, 0xE4}},
	{0xC3, 4, {0x24, 0x11, 0x02, 0x00}},
	{0xC4, 6, {0x01, 0xA2, 0xA6, 0x26, 0x00, 0x00}},
	{0xCF, 4, {0x00, 0x14, 0x00, 0xC0}},

	{0x35, 1, {0x00}},
	{0x53, 1, {0x24}},
	{0x55, 1, {0x00}},
	{0x11, 1, {0x00}},
	{REGFLAG_DELAY, 120, {} },
	{0x29, 1, {0x00}},
	{REGFLAG_DELAY, 20, {} }
};
#endif
static struct LCM_setting_table init_setting_vdo[] = {
	{0xB9, 3, {0x83, 0x11, 0x2A}},
	{0xCF, 4, {0x00, 0x14, 0x00, 0xC0}},

	{0x35, 1, {0x00}},
	{0x53, 1, {0x24}},
	{0x55, 1, {0x00}},
	{0x11, 1, {0x00}},
	{REGFLAG_DELAY, 120, {} },
	{0x29, 1, {0x00}},
	{REGFLAG_DELAY, 20, {} }
};

static struct LCM_setting_table bl_level[] = {
    {0x51, 2, {0x0F,0xFF} },
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
    params->dsi.LANE_NUM = LCM_FOUR_LANE;//LCM_FOUR_LANE;
    /* The following defined the fomat for data coming from LCD engine. */
    params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
    params->dsi.data_format.trans_seq = LCM_DSI_TRANS_SEQ_MSB_FIRST;
    params->dsi.data_format.padding = LCM_DSI_PADDING_ON_LSB;
    params->dsi.data_format.format = LCM_DSI_FORMAT_RGB888;

    /* Highly depends on LCD driver capability. */
    params->dsi.packet_size = 256;
    /* video mode timing */
    params->dsi.PS = LCM_PACKED_PS_24BIT_RGB888;

    params->dsi.vertical_sync_active = 2;
    params->dsi.vertical_backporch = 12;
    params->dsi.vertical_frontporch = 32;
    //params->dsi.vertical_frontporch_for_low_power = 540;
    params->dsi.vertical_active_line = FRAME_HEIGHT;

    params->dsi.horizontal_sync_active = 4;
    params->dsi.horizontal_backporch = 12;
    params->dsi.horizontal_frontporch = 20;
    params->dsi.horizontal_active_pixel = FRAME_WIDTH;
    params->dsi.ssc_disable = 1;
#ifndef CONFIG_FPGA_EARLY_PORTING
#if (LCM_DSI_CMD_MODE)
    /* this value must be in MTK suggested table */
    params->dsi.PLL_CLOCK = 488;
#else
    /* this value must be in MTK suggested table */
    params->dsi.PLL_CLOCK = 488;
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

    params->dsi.CLK_HS_PRPR= 6;

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
}

static void lcm_init_power(void)
{
	LCM_LOGI("%s: enter\n", __func__);
	if ((!gpio_get_value(GPIO_LCD_VSP_EN)) && (!gpio_get_value(GPIO_LCD_VSN_EN))) { //when vsp and vsn is not enable
		LCM_LOGI("%s: set lcd bias on\n", __func__);
#ifdef CONFIG_SET_LCD_BIAS_ODM_HQ
		SET_LCD_BIAS_EN(ON, VSP_FIRST_VSN_AFTER, 5500);  //open lcd bias
		MDELAY(15);
#endif
	}
	LCM_LOGI("%s: exit\n", __func__);
}

static void lcm_suspend_power(void)
{
	LCM_LOGI("%s: enter\n", __func__);
	if (g_gesture <= 0) { //when touch gesture is not enable
		LCM_LOGI("%s: set lcd bias off\n", __func__);
#ifdef CONFIG_SET_LCD_BIAS_ODM_HQ
		SET_LCD_BIAS_EN(OFF, VSN_FIRST_VSP_AFTER, 5500);  //close lcd bias
		MDELAY(15);
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
	MDELAY(5);
	SET_RESET_PIN(0);
	MDELAY(5);
	SET_RESET_PIN(1);
	MDELAY(50);

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

    LCM_LOGI("%s,hx83112a backlight: level = %d\n", __func__, level);

    if (level > 2047) {
        level = 4095;
    } else if (level > 0 && level < 10) {
        level = 10;
    } else { /* HX831112A only support 12bit pwm, so change level_11bit to level_12bit */
        level = (level << 1) | (level >> 10);
    }

	bl_level[0].para_list[0] = (level >> 8) & 0x0F;
	bl_level[0].para_list[1] = level & 0xFF;

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

LCM_DRIVER hx83112a_fhdp_dsi_vdo_jdi_zal1878_lcm_drv = {
    .name = "hx83112a_dsbj_jdi",
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
