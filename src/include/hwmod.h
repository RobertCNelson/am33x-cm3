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

#ifndef __HWMOD_H__
#define __HWMOD_H__

#define MODULE_DISABLE  0x0
#define MODULE_ENABLE   0x2

enum hwmod_id {
	HWMOD_CLKDIV32K,
	HWMOD_EMIF,
	HWMOD_EMIF_FW,
	HWMOD_GPIO0,
	HWMOD_I2C0,
	HWMOD_IEEE5000,
	HWMOD_L3,
	HWMOD_L3_INSTR,
	HWMOD_L4FW,
	HWMOD_L4HS,
	HWMOD_L4LS,
	HWMOD_MPU,
	HWMOD_OCMCRAM,
	HWMOD_OTFA_EMIF,

	HWMOD_COUNT,
	HWMOD_END = -1,
};

void hwmod_init(void);
int module_state_change(int state, enum hwmod_id id);
int interconnect_modules_enable(void);
int interconnect_modules_disable(void);
int essential_modules_disable(void);
int essential_modules_enable(void);
void mpu_disable(void);
void mpu_enable(void);

#endif

