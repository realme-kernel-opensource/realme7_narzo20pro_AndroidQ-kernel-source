/********************************************
 ** Copyright (C) 2018 OPPO Mobile Comm Corp. Ltd.
 ** ODM_HQ_EDIT
 ** File: nt36525b_hdp_dsi_vdo_inx_zal1851.c
 ** Description: Source file for LCD driver
 **          To Control LCD driver
 ** Version :1.0
 ** Date : 2020/03/02
 ** Author: Liyan@ODM_HQ.Multimedia.LCD
 ** ---------------- Revision History: --------------------------
 ** <version>    <date>          < author >              <desc>
 **  1.0           2020/03/02   Liyan@ODM_HQ   Source file for LCD driver
 ********************************************/

#define LOG_TAG "LCM_NT36525B"

#ifndef BUILD_LK
#include <linux/string.h>
#include <linux/kernel.h>
#include <mt-plat/mtk_boot_common.h>
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
#include <mt-plat/mtk_boot.h>

#ifdef BUILD_LK
#define LCM_LOGI(string, args...)  dprintf(0, "[LK/"LOG_TAG"]"string, ##args)
#define LCM_LOGD(string, args...)  dprintf(1, "[LK/"LOG_TAG"]"string, ##args)
#else
#define LCM_LOGI(fmt, args...)  pr_debug("[KERNEL/"LOG_TAG"]"fmt, ##args)
#define LCM_LOGD(fmt, args...)  pr_debug("[KERNEL/"LOG_TAG"]"fmt, ##args)
#endif

#define NT36525B_LCM_ID (0x15)

static const unsigned int BL_MIN_LEVEL = 20;
typedef struct LCM_UTIL_FUNCS LCM_UTIL_FUNCS;
typedef struct LCM_PARAMS LCM_PARAMS;
typedef struct LCM_DRIVER LCM_DRIVER;
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

#define MTK_GPIO_DESC_BASE 325
#define GPIO_LCD_VSP_EN (MTK_GPIO_DESC_BASE + 169)
#define GPIO_LCD_VSN_EN (MTK_GPIO_DESC_BASE + 165)

/* static unsigned char lcd_id_pins_value = 0xFF; */
static const unsigned char LCD_MODULE_ID = 0x01;
extern int gesture_flag;
static int boot_mode = -1;

extern unsigned int esd_recovery_backlight_level;
extern void lcd_queue_load_tp_fw(void);

//static int cabc_lastlevel = 1;


#define LCM_DSI_CMD_MODE    0
#define FRAME_WIDTH        (720)
#define FRAME_HEIGHT    (1600)
#define LCM_DENSITY        (320)

#define LCM_PHYSICAL_WIDTH        (67932)
#define LCM_PHYSICAL_HEIGHT        (150960)

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

extern int nvt_tp_oppo6769;
struct LCM_setting_table {
    unsigned int cmd;
    unsigned char count;
    unsigned char para_list[64];
};

/* Liyan@ODM.HQ.Multimedia.LCM 2019/09/19 modified for backlight remapping */
static int blmap_table[] = {
	57, 9,
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
    {REGFLAG_DELAY, 1, {} },
    {0x10, 0, {} },
    {REGFLAG_DELAY, 100, {} },
    //{0x4F, 1, {0x01} },
    //{REGFLAG_DELAY, 120, {} }
};

static struct LCM_setting_table lcm_suspend_setting_factory[] = {
    {0x28, 0, {} },
    {REGFLAG_DELAY, 1, {} },
    {0x10, 0, {} },
    {REGFLAG_DELAY, 100, {} },
    {0x4F, 1, {0x01} },
    {REGFLAG_DELAY, 120, {} }
};

static struct LCM_setting_table init_setting_vdo[] = {
	{0xFF,1,{0x23}},
	{REGFLAG_UDELAY, 1, {}},
	{0xFB,1,{0x01}},
	{0x00,1,{0x60}}, // PWM 11bit

	{0xFF,1,{0x10}},
	{REGFLAG_UDELAY, 1, {}},
	{0xFB,1,{0x01}},
	{0x35,1,{0x00}},
	{0x53,1,{0x24}},

	{0x11,1,{0x00}},
	{REGFLAG_DELAY, 100, {}},
	{0x29,1,{0x00}},
};

static struct LCM_setting_table bl_level[] = {
    {0x51, 2, {0x0F,0xFF} },
    {REGFLAG_END_OF_TABLE, 0x00, {} }
};

static struct LCM_setting_table bl_level_dimming_exit[] = {
	{0x53, 1, {0x24}},
    {0x51, 2, {0x07, 0xFF} },
    {REGFLAG_END_OF_TABLE, 0x00, {} }
};

#if 0
#ifdef VENDOR_EDIT
//Jinzhu.Han@RM.MM.Display.Driver, 2019/01/10, Modified for 3level cabc feature.
static struct LCM_setting_table lcm_cabc_enter_setting_ui[] = {
    {0x53, 1, {0x2C}},
    {0x55, 1, {0x01}},
    {0x53, 1, {0x24}},
    {REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table lcm_cabc_enter_setting_image[] = {
    {0x53, 1, {0x2C}},
    {0x55, 1, {0x02}},
    {0x53, 1, {0x24}},
    {REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table lcm_cabc_enter_setting_video[] = {
    {0x53, 1, {0x2C}},
    {0x55, 1, {0x03}},
    {0x53, 1, {0x24}},
    {REGFLAG_END_OF_TABLE, 0x00, {}}
};
#else
static struct LCM_setting_table lcm_cabc_enter_setting[] = {
    {0x53, 1, {0x2C}},
    {0x55, 1, {0x02}},
    {0x53, 1, {0x24}},
    {REGFLAG_END_OF_TABLE, 0x00, {} }
};
#endif

static struct LCM_setting_table lcm_cabc_exit_setting[] = {
    {0x53, 1, {0x2C}},
    {0x55, 1, {0x00}},
    {0x53, 1, {0x24}},
    {REGFLAG_END_OF_TABLE, 0x00, {} }
};
#endif
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
    int boot_mode = 0;
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

    params->dsi.vertical_sync_active = 2;
    params->dsi.vertical_backporch = 244;
    params->dsi.vertical_frontporch = 10;
    //params->dsi.vertical_frontporch_for_low_power = 540;
    params->dsi.vertical_active_line = FRAME_HEIGHT;

    params->dsi.horizontal_sync_active = 4;
    params->dsi.horizontal_backporch = 44;
    params->dsi.horizontal_frontporch = 12;
    params->dsi.horizontal_active_pixel = FRAME_WIDTH;
    params->dsi.ssc_disable = 1;
#ifndef CONFIG_FPGA_EARLY_PORTING
    /* this value must be in MTK suggested table */
    params->dsi.PLL_CLOCK = 367;
    params->dsi.data_rate = 733; /* 336.5M */
    //params->dsi.PLL_CK_CMD = 220;
    //params->dsi.PLL_CK_VDO = 255;
#else
    params->dsi.pll_div1 = 0;
    params->dsi.pll_div2 = 0;
    params->dsi.fbk_div = 0x1;
#endif
    /* clk continuous video mode */
    params->dsi.cont_clock = 0;

    //params->dsi.CLK_HS_PRPR= 6;

    params->dsi.clk_lp_per_line_enable = 0;
    if (get_boot_mode() == META_BOOT) {
        boot_mode++;
        LCM_LOGI("META_BOOT\n");
    }
    if (get_boot_mode() == ADVMETA_BOOT) {
        boot_mode++;
        LCM_LOGI("ADVMETA_BOOT\n");
    }
    if (get_boot_mode() == ATE_FACTORY_BOOT) {
        boot_mode++;
        LCM_LOGI("ATE_FACTORY_BOOT\n");
    }
    if (get_boot_mode() == FACTORY_BOOT)     {
        boot_mode++;
        LCM_LOGI("FACTORY_BOOT\n");
    }
    if (boot_mode == 0) {
        LCM_LOGI("neither META_BOOT or FACTORY_BOOT\n");
        params->dsi.esd_check_enable = 1;
        params->dsi.customization_esd_check_enable = 1;
        params->dsi.lcm_esd_check_table[0].cmd = 0x0A;
        params->dsi.lcm_esd_check_table[0].count = 1;
        params->dsi.lcm_esd_check_table[0].para_list[0] = 0x9C;
    }
/* Liyan@ODM.HQ.Multimedia.LCM 2019/09/19 modified for backlight remapping */
	params->blmap = blmap_table;
	params->blmap_size = sizeof(blmap_table)/sizeof(blmap_table[0]);
	params->brightness_max = 2047;
	params->brightness_min = 10;
#ifndef BUILD_LK
	register_device_proc("lcd", "nt36525b_innolux", "innolux");
#endif
}

static void lcm_init_power(void)
{
	LCM_LOGI("%s: enter\n", __func__);
	if ((!gpio_get_value(GPIO_LCD_VSP_EN)) && (!gpio_get_value(GPIO_LCD_VSN_EN))) { //when vsp and vsn is not enable
		LCM_LOGI("%s: set lcd bias on\n", __func__);
#ifdef CONFIG_SET_LCD_BIAS_ODM_HQ
		SET_LCD_BIAS_EN(ON, VSP_FIRST_VSN_AFTER, 6000);  //open lcd bias
		MDELAY(5);
#endif
	}
	LCM_LOGI("%s: exit\n", __func__);
}

static void lcm_suspend_power(void)
{
	LCM_LOGI("%s: enter\n", __func__);
	if (gesture_flag <= 0) { //when touch gesture is not enable
		LCM_LOGI("%s: set lcd bias off\n", __func__);
#ifdef CONFIG_SET_LCD_BIAS_ODM_HQ
		SET_LCD_BIAS_EN(OFF, VSN_FIRST_VSP_AFTER, 6000);  //close lcd bias
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

static void lcm_hw_reset(void)
{
	LCM_LOGI("%s: enter\n", __func__);
	SET_RESET_PIN(1);
	MDELAY(10);
	SET_RESET_PIN(0);
	MDELAY(3);
	SET_RESET_PIN(1);
	MDELAY(5);
	LCM_LOGI("%s: exit\n", __func__);
}

static void lcm_init(void)
{
    int size;

    SET_RESET_PIN(1);
    MDELAY(5);
    SET_RESET_PIN(0);
    MDELAY(3);
    SET_RESET_PIN(1);
    MDELAY(10);

    if(nvt_tp_oppo6769)
	lcd_queue_load_tp_fw();

    size = sizeof(init_setting_vdo) /
        sizeof(struct LCM_setting_table);
    push_table(NULL, init_setting_vdo, size, 1);

#if 0 //def VENDOR_EDIT
    //Jinzhu.Han@RM.MM.Display.Driver, 2019/01/10, Add for 3level cabc feature.
    printk("set the cabc_lastlevel = %d\n",cabc_lastlevel);
	switch (cabc_lastlevel) {
        case 2 :
            push_table(NULL, lcm_cabc_enter_setting_image, sizeof(lcm_cabc_enter_setting_image) / sizeof(struct LCM_setting_table), 1);
            break;
        case 3 :
            push_table(NULL, lcm_cabc_enter_setting_video, sizeof(lcm_cabc_enter_setting_video) / sizeof(struct LCM_setting_table), 1);
            break;
        default :
            push_table(NULL, lcm_cabc_enter_setting_ui, sizeof(lcm_cabc_enter_setting_ui) / sizeof(struct LCM_setting_table), 1);
            break;
    }
#endif
}

static void lcm_suspend(void)
{
    LCM_LOGI("%s: enter\n", __func__);
    if(boot_mode == -1){
        boot_mode = get_boot_mode();
        LCM_LOGD("get boot_mode in lcm %d \n",boot_mode);
    }

    if ((boot_mode == META_BOOT|| boot_mode == FACTORY_BOOT )) {
        push_table(NULL, lcm_suspend_setting_factory,
          sizeof(lcm_suspend_setting_factory) / sizeof(struct LCM_setting_table),
          1);
    }
    else {
        push_table(NULL, lcm_suspend_setting,
          sizeof(lcm_suspend_setting) / sizeof(struct LCM_setting_table),
          1);
    }
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
	if (level == 0){
		bl_level_dimming_exit[1].para_list[0] = (level >> 8) & 0x07;
		bl_level_dimming_exit[1].para_list[1] = level & 0xFF;
        push_table(handle,
			bl_level_dimming_exit,
			sizeof(bl_level_dimming_exit) / sizeof(struct LCM_setting_table),
			1);
	}
	else{
		if (level > 2047){
			level = 2047;
		}else if(level > 0 && level < 10){
			level = 10;
		}

		bl_level[0].para_list[0] = (level >> 8) & 0x07;
		bl_level[0].para_list[1] = level & 0xFF;
		push_table(handle,
			bl_level,
			sizeof(bl_level) / sizeof(struct LCM_setting_table),
			1);
		}
}
#if 0
#ifdef VENDOR_EDIT
//Jinzhu.Han@RM.MM.Display.Driver, 2019/01/10, Add for 3level cabc feature.
static void lcm_set_cabc_mode_cmdq(void *handle, unsigned int level)
{
	printk("%s [lcd] cabc_mode is %d ,lastlevel is %d \n", __func__, level,cabc_lastlevel);
	switch (level) {
        case 1 :
            push_table(handle,lcm_cabc_enter_setting_ui, sizeof(lcm_cabc_enter_setting_ui) / sizeof(struct LCM_setting_table), 1);
            break;
        case 2 :
            push_table(handle,lcm_cabc_enter_setting_image, sizeof(lcm_cabc_enter_setting_image) / sizeof(struct LCM_setting_table), 1);
            break;
        case 3 :
            push_table(handle,lcm_cabc_enter_setting_video, sizeof(lcm_cabc_enter_setting_video) / sizeof(struct LCM_setting_table), 1);
            break;
        default :
            push_table(handle,lcm_cabc_exit_setting, sizeof(lcm_cabc_exit_setting) / sizeof(struct LCM_setting_table), 1);
            break;
        }
        if (level > 0) {
            cabc_lastlevel = level;
        }
}
#else
static void lcm_set_cabc_mode_cmdq(void *handle, unsigned int level)
{
    printk("%s [lcd] cabc_mode is %d \n", __func__, level);
    if (level) {
        push_table(handle,lcm_cabc_enter_setting, sizeof(lcm_cabc_enter_setting) / sizeof(struct LCM_setting_table), 1);
    } else {
        push_table(handle,lcm_cabc_exit_setting, sizeof(lcm_cabc_exit_setting) / sizeof(struct LCM_setting_table), 1);
    }
}
#endif
#endif

struct LCM_DRIVER nt36525b_hdp_dsi_vdo_inx_zal1851_lcm_drv = {
    .name = "nt36525b_inx",
    .set_util_funcs = lcm_set_util_funcs,
    .get_params = lcm_get_params,
    .init = lcm_init,
    .suspend = lcm_suspend,
    .resume = lcm_resume,
    .hw_reset_before_lp11 = lcm_hw_reset, // for lcm bias_on&hw_reset before mipi dsi goes to LP11
#ifdef BUILD_LK
    .compare_id = lcm_compare_id,
#endif
    .init_power = lcm_init_power,
    .resume_power = lcm_resume_power,
    .suspend_power = lcm_suspend_power,
    .set_backlight_cmdq = lcm_setbacklight_cmdq,
 //   .set_cabc_mode_cmdq = lcm_set_cabc_mode_cmdq,
};
