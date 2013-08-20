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

#include <prm335x.h>
#include <ldo.h>
#include <ldo_335x.h>

const unsigned int am335x_ldo_regs[LDO_COUNT] = {
	[LDO_CORE] 	= AM335X_PRM_LDO_SRAM_CORE_CTRL,
	[LDO_MPU]	= AM335X_PRM_LDO_SRAM_MPU_CTRL,
};

