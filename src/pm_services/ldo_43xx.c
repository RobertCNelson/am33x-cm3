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

#include <prm43xx.h>
#include <ldo.h>
#include <ldo_43xx.h>

/* TODO */
/* Note: Need to additionally check for Voltage Monitoring here */
const unsigned int am43xx_ldo_regs[LDO_COUNT] = {
	[LDO_CORE]	= AM43XX_PRM_LDO_SRAM_CORE_CTRL,
	[LDO_MPU]	= AM43XX_PRM_LDO_SRAM_MPU_CTRL,
};

