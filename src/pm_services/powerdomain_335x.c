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
#include <prmam335x.h>
#include <prm43xx.h>
#include <powerdomain.h>
#include <powerdomain_335x.h>

/* PRM_MPU bits */
const struct pd_mpu_bits am335x_mpu_bits = {
	.ram_retst_mask		= AM335X_MPU_RAM_RETSTATE_MASK,
	.ram_retst_shift	= AM335X_MPU_RAM_RETSTATE_SHIFT,
	.l2_retst_mask		= AM335X_MPU_L2_RETSTATE_MASK,
	.l2_retst_shift		= AM335X_MPU_L2_RETSTATE_SHIFT,
	.l1_retst_mask		= AM335X_MPU_L1_RETSTATE_MASK,
	.l1_retst_shift		= AM335X_MPU_L1_RETSTATE_SHIFT,
	.lpstchg_mask		= AM335X_MPU_LOWPOWERSTATECHANGE_MASK,
	.lpstchg_shift		= AM335X_MPU_LOWPOWERSTATECHANGE_SHIFT,
	.logicretst_mask	= AM335X_MPU_LOGICRETSTATE_MASK,
	.logicretst_shift	= AM335X_MPU_LOGICRETSTATE_SHIFT,
	.pwrst_mask		= AM335X_MPU_POWERSTATE_MASK,
	.pwrst_shift		= AM335X_MPU_POWERSTATE_SHIFT,
};

/* PRM_PER bits */
const struct pd_per_bits am335x_per_bits = {
	.per_retst_mask		= AM335X_PER_MEM_RETSTATE_MASK,
	.per_retst_shift	= AM335X_PER_MEM_RETSTATE_SHIFT,
	.ram1_retst_mask	= AM335X_PER_RAM_MEM_RETSTATE_MASK,
	.ram1_retst_shift	= AM335X_PER_RAM_MEM_RETSTATE_SHIFT,
	.icss_retst_mask	= AM335X_PER_ICSS_MEM_RETSTATE_MASK,
	.icss_retst_shift	= AM335X_PER_ICSS_MEM_RETSTATE_SHIFT,
	.lpstchg_mask		= AM335X_PER_LOWPOWERSTATECHANGE_MASK,
	.lpstchg_shift		= AM335X_PER_LOWPOWERSTATECHANGE_SHIFT,
	.logicretst_mask	= AM335X_PER_LOGICRETSTATE_MASK,
	.logicretst_shift	= AM335X_PER_LOGICRETSTATE_SHIFT,
	.pwrst_mask		= AM335X_PER_POWERSTATE_MASK,
	.pwrst_shift		= AM335X_PER_POWERSTATE_SHIFT,
};

const struct powerdomain_regs am335x_pd_regs[] = {
	[PD_MPU] = {
		.stctrl		= AM335X_PM_MPU_PWRSTCTRL,
		.pwrstst	= AM335X_PM_MPU_PWRSTST,
	},
	[PD_PER] = {
		.stctrl		= AM335X_PM_PER_PWRSTCTRL,
		.pwrstst	= AM335X_PM_PER_PWRSTST,
	},
};

