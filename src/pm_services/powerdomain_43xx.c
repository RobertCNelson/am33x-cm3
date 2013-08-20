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
#include <prm43xx.h>
#include <powerdomain.h>
#include <powerdomain_43xx.h>

/* PRM_MPU bits */
const struct pd_mpu_bits am43xx_mpu_bits = {
	.ram_retst_mask		= AM43XX_MPU_RAM_RETSTATE_MASK,
	.ram_retst_shift	= AM43XX_MPU_RAM_RETSTATE_SHIFT,
	.l2_retst_mask		= AM43XX_MPU_L2_RETSTATE_MASK,
	.l2_retst_shift		= AM43XX_MPU_L2_RETSTATE_SHIFT,
	.l1_retst_mask		= AM43XX_MPU_L1_RETSTATE_MASK,
	.l1_retst_shift		= AM43XX_MPU_L1_RETSTATE_SHIFT,
	.lpstchg_mask		= AM43XX_MPU_LOWPOWERSTATECHANGE_MASK,
	.lpstchg_shift		= AM43XX_MPU_LOWPOWERSTATECHANGE_SHIFT,
	.logicretst_mask	= AM43XX_MPU_LOGICRETSTATE_MASK,
	.logicretst_shift	= AM43XX_MPU_LOGICRETSTATE_SHIFT,
	.pwrst_mask		= AM43XX_MPU_POWERSTATE_MASK,
	.pwrst_shift		= AM43XX_MPU_POWERSTATE_SHIFT,
};

/* PRM_PER bits */
const struct pd_per_bits am43xx_per_bits = {
	.per_retst_mask		= AM43XX_PER_MEM_RETSTATE_MASK,
	.per_retst_shift	= AM43XX_PER_MEM_RETSTATE_SHIFT,
	.ram1_retst_mask	= AM43XX_PER_RAM1_MEM_RETSTATE_MASK,
	.ram1_retst_shift	= AM43XX_PER_RAM1_MEM_RETSTATE_SHIFT,
	.ram2_retst_mask	= AM43XX_PER_RAM2_MEM_RETSTATE_MASK,
	.ram2_retst_shift	= AM43XX_PER_RAM2_MEM_RETSTATE_SHIFT,
	.icss_retst_mask	= AM43XX_PER_ICSS_MEM_RETSTATE_MASK,
	.icss_retst_shift	= AM43XX_PER_ICSS_MEM_RETSTATE_SHIFT,
	.lpstchg_mask		= AM43XX_PER_LOWPOWERSTATECHANGE_MASK,
	.lpstchg_shift		= AM43XX_PER_LOWPOWERSTATECHANGE_SHIFT,
	.logicretst_mask	= AM43XX_PER_LOGICRETSTATE_MASK,
	.logicretst_shift	= AM43XX_PER_LOGICRETSTATE_SHIFT,
	.pwrst_mask		= AM43XX_PER_POWERSTATE_MASK,
	.pwrst_shift		= AM43XX_PER_POWERSTATE_SHIFT,
};

const struct powerdomain_regs am43xx_pd_regs[] = {
	[PD_MPU] = {
		.stctrl		= AM43XX_PM_MPU_PWRSTCTRL,
		.pwrstst	= AM43XX_PM_MPU_PWRSTST,
	},
	[PD_PER] = {
		.stctrl		= AM43XX_PM_PER_PWRSTCTRL,
		.pwrstst	= AM43XX_PM_PER_PWRSTST,
	},
};
