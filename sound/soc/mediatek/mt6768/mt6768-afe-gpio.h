/* SPDX-License-Identifier: GPL-2.0 */
/*
 * mt6768-afe-gpio.h  --  Mediatek 6768 afe gpio ctrl definition
 *
 * Copyright (c) 2018 MediaTek Inc.
 * Author: Michael Hsiao <michael.hsiao@mediatek.com>
 */

#ifndef _MT6768_AFE_GPIO_H_
#define _MT6768_AFE_GPIO_H_

struct mtk_base_afe;

int mt6768_afe_gpio_init(struct mtk_base_afe *afe);

#ifdef ODM_HQ_EDIT
/*sunjingtao@ODM.HQ.Multimedia.Audio 2020/03/04 modified for speaker bringup*/
int mt6768_afe_gpio_extamp_select(struct mtk_base_afe *afe, bool enable, int mode);
#endif /* ODM_HQ_EDIT */

int mt6768_afe_gpio_request(struct mtk_base_afe *afe, bool enable,
			    int dai, int uplink);

#endif
