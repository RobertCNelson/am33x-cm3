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

#include <io.h>
#include <prmam335x.h>
#include <prcm_core.h>
#include <ldo.h>
#include <ldo_335x.h>
#include <ldo_43xx.h>

static const unsigned int *ldo_regs;

void ldo_wait_for_on(enum ldo_id id)
{
	/* Poll for LDO status to be out of retention (SRAMLDO_STATUS) */
	while (__raw_readl(ldo_regs[LDO_CORE]) & SRAMLDO_STATUS);
}

void ldo_wait_for_ret(enum ldo_id id)
{
	/* Poll for LDO Status to be in retention (SRAMLDO_STATUS) */
	while (!(__raw_readl(ldo_regs[LDO_CORE]) & SRAMLDO_STATUS));
}

void ldo_power_up(enum ldo_id id)
{
	unsigned int val;

	/* Disable RETMODE for LDO */
	val = __raw_readl(ldo_regs[id]);
	val &= ~RETMODE_ENABLE;
	__raw_writel(val, ldo_regs[id]);
}

void ldo_power_down(enum ldo_id id)
{
	unsigned int val;

	/* Configure RETMODE_ENABLE for LDO */
	val = __raw_readl(ldo_regs[id]);
	val |= RETMODE_ENABLE;
	__raw_writel(val, ldo_regs[id]);
}

void ldo_init(void)
{
	if (soc_id == AM335X_SOC_ID)
		ldo_regs = am335x_ldo_regs;
	else if (soc_id == AM43XX_SOC_ID)
		ldo_regs = am43xx_ldo_regs;
}
