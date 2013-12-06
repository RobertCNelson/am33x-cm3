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

#include <cm335x.h>
#include <cm43xx.h>
#include <clockdomain.h>
#include <clockdomain_43xx.h>

const unsigned int am43xx_clkdms[CLKDM_COUNT] = {
	[CLKDM_L3S_TSC]		= AM43XX_CM_L3S_TSC_CLKSTCTRL,
	[CLKDM_MPU]		= AM43XX_CM_MPU_CLKSTCTRL,
	[CLKDM_CPSW]		= AM43XX_CM_PER_CPSW_CLKSTCTRL,
	[CLKDM_DSS]		= AM43XX_CM_PER_DSS_CLKSTCTRL,
	[CLKDM_EMIF]		= AM43XX_CM_PER_EMIF_CLKSTCTRL,
	[CLKDM_ICSS]		= AM43XX_CM_PER_ICSS_CLKSTCTRL,
	[CLKDM_L3]		= AM43XX_CM_PER_L3_CLKSTCTRL,
	[CLKDM_L3S]		= AM43XX_CM_PER_L3S_CLKSTCTRL,
	[CLKDM_L4LS]		= AM43XX_CM_PER_L4LS_CLKSTCTRL,
	[CLKDM_LCDC]		= AM43XX_CM_PER_LCDC_CLKSTCTRL,
	[CLKDM_OCPWP_L3]	= AM43XX_CM_PER_OCPWP_L3_CLKSTCTRL,
	[CLKDM_WKUP]		= AM43XX_CM_WKUP_CLKSTCTRL,
};

const enum clkdm_id am43xx_sleep_clkdms[] = {
	CLKDM_EMIF,
	CLKDM_DSS,
	CLKDM_LCDC,
	CLKDM_ICSS,
	CLKDM_CPSW,
	CLKDM_OCPWP_L3,
	CLKDM_L4LS,
	CLKDM_L3S,
	CLKDM_L3,

	CLKDM_END,
};
