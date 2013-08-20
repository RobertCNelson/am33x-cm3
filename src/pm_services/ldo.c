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
#include <prcm.h>
#include <prmam335x.h>
#include <prm43xx.h>
#include <system_am335.h>

void core_ldo_power_down(void)
{
	unsigned int core_ldo;

	/* Configure RETMODE_ENABLE for CORE LDO */
	if (soc_id == AM335X_SOC_ID && soc_rev > AM335X_REV_ES1_0) {
		core_ldo = __raw_readl(AM335X_PRM_LDO_SRAM_CORE_CTRL);
		core_ldo |= RETMODE_ENABLE;
		__raw_writel(core_ldo, AM335X_PRM_LDO_SRAM_CORE_CTRL);

		/* Poll for LDO Status to be in retention (SRAMLDO_STATUS) */
		while (!(__raw_readl(AM335X_PRM_LDO_SRAM_CORE_CTRL) & SRAMLDO_STATUS));
	} else if (soc_id == AM43XX_SOC_ID) {
		/* TODO */
		/* Note: Need to additionally check for Voltage Monitoring here */
	}
}

void core_ldo_power_up(void)
{
	unsigned int core_ldo;

	/* Disable RETMODE for CORE LDO */
	if (soc_id == AM335X_SOC_ID && soc_rev > AM335X_REV_ES1_0) {
		core_ldo = __raw_readl(AM335X_PRM_LDO_SRAM_CORE_CTRL);
		core_ldo &= ~RETMODE_ENABLE;
		__raw_writel(core_ldo, AM335X_PRM_LDO_SRAM_CORE_CTRL);

		/* Poll for LDO status to be out of retention (SRAMLDO_STATUS) */
		while (__raw_readl(AM335X_PRM_LDO_SRAM_CORE_CTRL) & SRAMLDO_STATUS);
	} else if (soc_id == AM43XX_SOC_ID) {
		/* TODO */
		/* Note: Need to additionally check for Voltage Monitoring here */
	}
}

void sram_ldo_ret_mode(int state)
{
	int var = __raw_readl(AM335X_PRM_LDO_SRAM_MPU_CTRL);

	if (state == RETMODE_ENABLE)
		var |= RETMODE_ENABLE;
	else
		var &= ~RETMODE_ENABLE;

	__raw_writel(var, AM335X_PRM_LDO_SRAM_MPU_CTRL);
}
