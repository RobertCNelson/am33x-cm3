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
#include <device_am335x.h>
#include <system_am335.h>
#include <clockdomain.h>

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

static const unsigned int am335x_clkdms[CLKDM_COUNT] = {
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

static const unsigned int am43xx_clkdms[CLKDM_COUNT] = {
	[CLKDM_L3S_TSC]		= AM43XX_CM_L3S_TSC_CLKSTCTRL,
	[CLKDM_MPU]		= AM43XX_CM_MPU_CLKSTCTRL,
	[CLKDM_CLK_24MHZ]	= AM335X_CM_PER_CLK_24MHZ_CLKSTCTRL, /* ? */
	[CLKDM_CPSW]		= AM43XX_CM_PER_CPSW_CLKSTCTRL,
	[CLKDM_DSS]		= AM43XX_CM_PER_DSS_CLKSTCTRL,
	[CLKDM_EMIF]		= AM43XX_CM_PER_EMIF_CLKSTCTRL,
	[CLKDM_ICSS]		= AM43XX_CM_PER_ICSS_CLKSTCTRL,
	[CLKDM_L3]		= AM43XX_CM_PER_L3_CLKSTCTRL,
	[CLKDM_L3S]		= AM43XX_CM_PER_L3S_CLKSTCTRL,
	[CLKDM_L4FW]		= AM335X_CM_PER_L4FW_CLKSTCTRL, /* ? */
	[CLKDM_L4HS]		= AM43XX_CM_PER_L4HS_CLKCTRL, /* ? */
	[CLKDM_L4LS]		= AM43XX_CM_PER_L4LS_CLKSTCTRL,
	[CLKDM_LCDC]		= AM43XX_CM_PER_LCDC_CLKSTCTRL,
	[CLKDM_OCPWP_L3]	= AM43XX_CM_PER_OCPWP_L3_CLKSTCTRL,
	[CLKDM_WKUP]		= AM43XX_CM_WKUP_CLKSTCTRL,
};

static const unsigned int *clkdms;

void clockdomain_init(void)
{
	/* yes this is ugly */
	if (soc_id == AM335X_SOC_ID)
		clkdms = am335x_clkdms;
	else if (soc_id == AM43XX_SOC_ID)
		clkdms = am43xx_clkdms;
}

#define DEFAULT_CLKTRCTRL_SHIFT		0
#define DEFAULT_CLKTRCTRL_MASK		(3 << DEFAULT_CLKTRCTRL_SHIFT)
#define DEFAULT_CLKTRCTRL_WAKE		0x2
#define DEFAULT_CLKTRCTRL_SLEEP		0x1

#define CLKDM_SLEEP	0x1
#define CLKDM_WAKE	0x2

static void _clkdm_sleep(int reg)
{
	int var;

	var = __raw_readl(reg);
	var = var_mod(var, DEFAULT_CLKTRCTRL_MASK, DEFAULT_CLKTRCTRL_SLEEP);
	__raw_writel(var, reg);
}

static void _clkdm_wakeup(int reg)
{
	int var;

	var = __raw_readl(reg);
	var = var_mod(var, DEFAULT_CLKTRCTRL_MASK, DEFAULT_CLKTRCTRL_WAKE);
	__raw_writel(var, reg);
}

int clkdm_state_change(int state, enum clkdm_id id)
{
	if (state == CLKDM_SLEEP)
		_clkdm_sleep(clkdms[id]);
	else
		_clkdm_wakeup(clkdms[id]);

	return 0;
}

static int clkdms_state_change(int state, const enum clkdm_id *ids)
{
	int ret = 0;
	int i;
	for (i = 0; ids[i] != CLKDM_END && !ret; i++)
		ret = clkdm_state_change(state, ids[i]);
	return ret;
}

static const enum clkdm_id am335x_sleep_clkdms[] = {
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

static const enum clkdm_id am43xx_sleep_clkdms[] = {
	CLKDM_EMIF,
	CLKDM_L3S_TSC,
	CLKDM_DSS,
	CLKDM_LCDC,
	CLKDM_CLK_24MHZ,
	CLKDM_ICSS,
	CLKDM_CPSW,
	CLKDM_OCPWP_L3,
	CLKDM_L4LS,
	CLKDM_L4HS,
	CLKDM_L4FW,
	CLKDM_L3S,
	CLKDM_L3,
	CLKDM_END,
};

/* CLKDM related */
void clkdm_sleep(void)
{
	if (soc_id == AM335X_SOC_ID)
		clkdms_state_change(CLKDM_SLEEP, am335x_sleep_clkdms);
	else if (soc_id == AM43XX_SOC_ID)
		clkdms_state_change(CLKDM_SLEEP, am43xx_sleep_clkdms);
}

void clkdm_wake(void)
{
	if (soc_id == AM335X_SOC_ID)
		clkdms_state_change(CLKDM_WAKE, am335x_sleep_clkdms);
	else if (soc_id == AM43XX_SOC_ID)
		clkdms_state_change(CLKDM_WAKE, am43xx_sleep_clkdms);
}

void mpu_clkdm_sleep(void)
{
	clkdm_state_change(CLKDM_SLEEP, CLKDM_MPU);
}

void mpu_clkdm_wake(void)
{
	clkdm_state_change(CLKDM_WAKE, CLKDM_MPU);
}

void wkup_clkdm_sleep(void)
{
	clkdm_state_change(CLKDM_SLEEP, CLKDM_WKUP);
}

void wkup_clkdm_wake(void)
{
	clkdm_state_change(CLKDM_WAKE, CLKDM_WKUP);
}
