/* Copyright (C) 2008 Rodolfo Giometti <giometti@linux.it>
 * Copyright (C) 2008 Eurotech S.p.A. <info@eurotech.it>
 * Based on a previous work by Copyright (C) 2008 Texas Instruments, Inc.
 *
 * Copyright (c) 2011, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
 
#define BAT_TAG "[BAT][BMS]"
#define ERROR_TAG "[ERR]"
#define BAT_DBG(...)  printk(KERN_INFO BAT_TAG __VA_ARGS__)
#define BAT_DBG_E(...)  printk(KERN_ERR BAT_TAG ERROR_TAG __VA_ARGS__)

enum bq27546_driverStatus {
	BQ27546_I2C_UNINIT,
	BQ27546_I2C_SUCCESS,
	BQ27546_I2C_FAIL,
};
