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

#ifndef __CLOCKDOMAIN_H__
#define __CLOCKDOMAIN_H__

enum clkdm_id {
	CLKDM_L3S_TSC,
	CLKDM_MPU,
	CLKDM_CLK_24MHZ,
	CLKDM_CPSW,
	CLKDM_DSS,
	CLKDM_EMIF,
	CLKDM_ICSS,
	CLKDM_L3,
	CLKDM_L3S,
	CLKDM_L4FW,
	CLKDM_L4HS,
	CLKDM_L4LS,
	CLKDM_LCDC,
	CLKDM_OCPWP_L3,
	CLKDM_WKUP,

	CLKDM_COUNT,
	CLKDM_END = -1,
};

void clockdomain_init(void);
void clkdm_sleep(void);
void clkdm_wake(void);
void mpu_clkdm_sleep(void);
void mpu_clkdm_wake(void);
void wkup_clkdm_sleep(void);
void wkup_clkdm_wake(void);

#endif

