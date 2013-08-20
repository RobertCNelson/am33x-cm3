/*
 * AM33XX-CM3 firmware
 *
 * Cortex-M3 (CM3) firmware for power management on Texas Instruments' AM33XX series of SoCs
 *
 * Copyright (C) 2011 Texas Instruments Incorporated - http://www.ti.com/
 *
 *  This software is licensed under the  standard terms and conditions in the Texas Instruments  Incorporated
 *  Technology and Software Publicly Available Software License Agreement , a copy of which is included in the
 *  software download.
*/

#ifndef __LDO_H__
#define __LDO_H__

enum ldo_id {
	LDO_CORE,
	LDO_MPU,

	LDO_COUNT,
};

void ldo_wait_for_on(enum ldo_id id);
void ldo_wait_for_ret(enum ldo_id id);
void ldo_power_up(enum ldo_id id);
void ldo_power_down(enum ldo_id id);
void ldo_init(void);

#endif
