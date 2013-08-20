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

#include <device_am335x.h>
#include <system_am335.h>
#include <clockdomain.h>
#include <clockdomain_335x.h>
#include <clockdomain_43xx.h>

static const unsigned int *clkdms;
static const enum clkdm_id *sleep_clkdms;

void clockdomain_init(void)
{
	if (soc_id == AM335X_SOC_ID) {
		clkdms = am335x_clkdms;
		sleep_clkdms = am335x_sleep_clkdms;
	} else if (soc_id == AM43XX_SOC_ID) {
		clkdms = am43xx_clkdms;
		sleep_clkdms = am43xx_sleep_clkdms;
	}
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

void clkdm_sleep(void)
{
	clkdms_state_change(CLKDM_SLEEP, sleep_clkdms);
}

void clkdm_wake(void)
{
	clkdms_state_change(CLKDM_WAKE, sleep_clkdms);
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