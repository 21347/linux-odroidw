/*
 * linux/regulator/rc5t619-regulator.h
 *
 * Regulator driver for RC5T619 power management chip.
 *
 * Copyright (C) 2012-2013 RICOH COMPANY,LTD
 *
 * Based on code
 *	Copyright (C) 2011 NVIDIA Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */
 
#ifndef __LINUX_REGULATOR_RC5T619_H
#define __LINUX_REGULATOR_RC5T619_H

#include <linux/regulator/machine.h>
#include <linux/regulator/driver.h>

#define rc5t619_rails(_name) "RC5T619_"#_name

/* RICHOH Regulator IDs */
enum regulator_id {
	RC5T619_ID_DC1,
	RC5T619_ID_DC2,	
	RC5T619_ID_DC3,
	RC5T619_ID_DC4,
	RC5T619_ID_DC5,
	RC5T619_ID_LDO1,
	RC5T619_ID_LDO2,
	RC5T619_ID_LDO3,
	RC5T619_ID_LDO4,
	RC5T619_ID_LDO5,
	RC5T619_ID_LDO6,
	RC5T619_ID_LDO7,
	RC5T619_ID_LDO8,
	RC5T619_ID_LDO9,
	RC5T619_ID_LDO10,
	RC5T619_ID_LDORTC1,
	RC5T619_ID_LDORTC2,
};

struct rc5t619_regulator_platform_data {
	struct regulator_init_data regulator;
	int init_uV;
	unsigned init_enable:1;
	unsigned init_apply:1;
	int sleep_uV;
	int sleep_slots;
	unsigned long ext_pwr_req;
	unsigned long flags;
};

extern	int rc5t619_regulator_enable_eco_mode(struct regulator_dev *rdev);
extern	int rc5t619_regulator_disable_eco_mode(struct regulator_dev *rdev);
extern	int rc5t619_regulator_enable_eco_slp_mode(struct regulator_dev *rdev);
extern	int rc5t619_regulator_disable_eco_slp_mode(struct regulator_dev *rdev);


#endif
