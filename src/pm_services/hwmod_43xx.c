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

#include <cm43xx.h>
#include <hwmod.h>
#include <hwmod_43xx.h>

const unsigned int am43xx_hwmods[HWMOD_COUNT] = {
	[HWMOD_CLKDIV32K]	= AM43XX_CM_WKUP_CLKDIV32K_CLKCTRL,
	[HWMOD_EMIF]		= AM43XX_CM_PER_EMIF_CLKCTRL,
	[HWMOD_EMIF_FW]		= AM43XX_CM_PER_EMIF_FW_CLKCTRL,
	[HWMOD_GPIO0]		= AM43XX_CM_WKUP_GPIO0_CLKCTRL,
	[HWMOD_I2C0]		= AM43XX_CM_WKUP_I2C0_CLKCTRL,
	[HWMOD_IEEE5000]	= AM43XX_CM_PER_IEEE5000_CLKCTRL,
	[HWMOD_L3]		= AM43XX_CM_PER_L3_CLKCTRL,
	[HWMOD_L3_INSTR]	= AM43XX_CM_PER_L3_INSTR_CLKCTRL,
	[HWMOD_L4FW]		= AM43XX_CM_PER_L4FW_CLKCTRL,
	[HWMOD_L4HS]		= AM43XX_CM_PER_L4HS_CLKCTRL,
	[HWMOD_L4LS]		= AM43XX_CM_PER_L4LS_CLKCTRL,
	[HWMOD_MPU]		= AM43XX_CM_MPU_MPU_CLKCTRL,
	[HWMOD_OCMCRAM]		= AM43XX_CM_PER_OCMCRAM_CLKCTRL,
	[HWMOD_OTFA_EMIF]	= AM43XX_CM_PER_OTFA_EMIF_CLKCTRL,
	[HWMOD_OCPWP]		= AM43XX_CM_PER_OCPWP_CLKCTRL,
};

const enum hwmod_id am43xx_essential_hwmods[] = {
	HWMOD_IEEE5000,
	HWMOD_EMIF_FW,
	HWMOD_OTFA_EMIF,
	HWMOD_OCMCRAM,
	HWMOD_OCPWP,
	HWMOD_END,
};

const enum hwmod_id am43xx_interconnect_hwmods[] = {
	HWMOD_L4LS,
	HWMOD_L4HS,
	HWMOD_L4FW,
	HWMOD_L3_INSTR,
	HWMOD_L3,
	HWMOD_END,
};

