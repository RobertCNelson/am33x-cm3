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

#include <prcm.h>
#include <clockdomain.h>
#include <clockdomain_335x.h>

const unsigned int am335x_clkdms[CLKDM_COUNT] = {
	[CLKDM_MPU]		= AM335X_CM_MPU_CLKSTCTRL,
	[CLKDM_CLK_24MHZ]	= AM335X_CM_PER_CLK_24MHZ_CLKSTCTRL,
	[CLKDM_CPSW]		= AM335X_CM_PER_CPSW_CLKSTCTRL,
	[CLKDM_ICSS]		= AM335X_CM_PER_ICSS_CLKSTCTRL,
	[CLKDM_L3]		= AM335X_CM_PER_L3_CLKSTCTRL,
	[CLKDM_L3S]		= AM335X_CM_PER_L3S_CLKSTCTRL,
	[CLKDM_L4FW]		= AM335X_CM_PER_L4FW_CLKSTCTRL,
	[CLKDM_L4HS]		= AM335X_CM_PER_L4HS_CLKSTCTRL,
	[CLKDM_L4LS]		= AM335X_CM_PER_L4LS_CLKSTCTRL,
	[CLKDM_LCDC]		= AM335X_CM_PER_LCDC_CLKSTCTRL,
	[CLKDM_OCPWP_L3]	= AM335X_CM_PER_OCPWP_L3_CLKSTCTRL,
	[CLKDM_WKUP]		= AM335X_CM_WKUP_CLKSTCTRL,
};

const enum clkdm_id am335x_sleep_clkdms[] = {
	CLKDM_OCPWP_L3,
	CLKDM_ICSS,
	CLKDM_CPSW,
	CLKDM_LCDC,
	CLKDM_CLK_24MHZ,
	CLKDM_L4LS,
	CLKDM_L4HS,
	CLKDM_L4FW,
	CLKDM_L3S,
	CLKDM_L3,

	CLKDM_END,
};
