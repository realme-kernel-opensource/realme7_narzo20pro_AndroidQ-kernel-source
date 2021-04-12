/*
* Copyright (C) 2016 MediaTek Inc.
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License version 2 as
* published by the Free Software Foundation.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
* See http://www.gnu.org/licenses/gpl-2.0.html for more details.
*/
/*****************************************************************************
 *
 * Filename:
 * ---------
 *   P24C64E.h
 *
 * Project:
 * --------
 *   ALPS
 *
 * Description:
 * ------------
 *   Header file of CAM_CAL driver
 *
 *
 * Author:
 * -------
 *   Dream Yeh (MTK08783)
 *
 *============================================================================
 */
#ifndef __P24C64E_H
#define __P24C64E_H
#include <linux/i2c.h>

unsigned int p24c64e_selective_read_region(struct i2c_client *client, unsigned int addr,
	unsigned char *data, unsigned int size);


#endif /* __CAM_CAL_H */

