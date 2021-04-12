/**********************************************************************************
* Copyright (c)  2008-2015  zhangchao hauqin Mobile Comm Corp., Ltd
* ODM_HQ_EDIT
* Description: USB NTC 2
* Version   : 1.0
* Date      : 2019-09-17
* Author    : zhangchao_hq
* ------------------------------ Revision History: --------------------------------
* <version>         <date>              <author>                      <desc>
* Revision 1.0    2019-09-17      zhangchao_hq                 Created for USB NTC 2
***********************************************************************************/

#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/dmi.h>
#include <linux/acpi.h>
#include <linux/thermal.h>
#include <linux/platform_device.h>
#include <mt-plat/aee.h>
#include <linux/types.h>
#include <linux/delay.h>
#include <linux/proc_fs.h>
#include <linux/syscalls.h>
#include <linux/sched.h>
#include <linux/writeback.h>
#include <linux/uaccess.h>
//#include "mt-plat/mtk_thermal_monitor.h"
//#include "mach/mtk_thermal.h"
//#include "mtk_thermal_timer.h"
#include <linux/uidgid.h>
#include <linux/slab.h>
#if defined(CONFIG_MEDIATEK_MT6577_AUXADC)
#include <linux/iio/consumer.h>
#endif

 
#if defined(CONFIG_MEDIATEK_MT6577_AUXADC)
struct iio_channel *thermistor_ch3;
#endif


int oppo_usb_ntc_adc2_read(void)
{
	int adc_volt = 0;
	int ret = 0;
	int auxadc_voltage = 0;

	//printk("%s:enter\n", __func__);

#if defined(CONFIG_MEDIATEK_MT6577_AUXADC)
	if (thermistor_ch3)
		ret = iio_read_channel_processed(thermistor_ch3, &auxadc_voltage);
#endif

	//printk("[%s]usb_ntc_adc2 auxadc_voltage is %d\n", __func__, auxadc_voltage);
	adc_volt = auxadc_voltage * 1500 / 4096;
	//printk("[%s]usb_ntc_adc2 voltage is %d\n", __func__, adc_volt);

	return adc_volt;
}

#if defined(CONFIG_MEDIATEK_MT6577_AUXADC)
static int usb_ntc_adc2_probe(struct platform_device *pdev)
{
	int err = 0;
	int ret = 0;

	printk("[%s]\n", __func__);

	if (!pdev->dev.of_node) {
		printk("[%s] Only DT based supported\n",
			__func__);
		return -ENODEV;
	}

	thermistor_ch3 = devm_kzalloc(&pdev->dev, sizeof(*thermistor_ch3),
		GFP_KERNEL);
	if (!thermistor_ch3)
		return -ENOMEM;


	thermistor_ch3 = iio_channel_get(&pdev->dev, "thermistor-ch3");
	ret = IS_ERR(thermistor_ch3);
	if (ret) {
		printk("[%s] fail to get auxadc iio ch3: %d\n",
			__func__, ret);
		return ret;
	}

	oppo_usb_ntc_adc2_read();
	
	return err;
}

#ifdef CONFIG_OF
const struct of_device_id usbntc_adc2_of_match[2] = {
	{.compatible = "mediatek,usbntc-adc2",},
	{},
};
#endif

#define USB_NTC_NAME    "usbntc-adc2"
static struct platform_driver usb_ntc_adc2 = {
	.remove = NULL,
	.shutdown = NULL,
	.probe = usb_ntc_adc2_probe,
	.suspend = NULL,
	.resume = NULL,
	.driver = {
		.name = USB_NTC_NAME,
#ifdef CONFIG_OF
		.of_match_table = usbntc_adc2_of_match,
#endif
	},
};
#endif /*CONFIG_MEDIATEK_MT6577_AUXADC*/


static int __init usb_ntc_adc2_init(void)
{
#if defined(CONFIG_MEDIATEK_MT6577_AUXADC)
	int err = 0;
#endif

	printk("[%s]\n", __func__);


#if defined(CONFIG_MEDIATEK_MT6577_AUXADC)
	err = platform_driver_register(&usb_ntc_adc2);
	if (err) {
		printk("usb_ntc_adc2 driver callback register failed.\n");
		return err;
	}
#endif

	return 0;
}

static void __exit usb_ntc_adc2_exit(void)
{
	printk("[%s]\n", __func__);
}

module_init(usb_ntc_adc2_init);
module_exit(usb_ntc_adc2_exit);
