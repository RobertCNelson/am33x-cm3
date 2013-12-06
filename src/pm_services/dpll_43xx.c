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

#include <device_common.h>
#include <cm43xx.h>
#include <dpll.h>
#include <dpll_43xx.h>

const struct dpll_regs am43xx_dpll_regs[DPLL_COUNT] = {
	[DPLL_PER] = {
		.dpll_pwr_sw_ctrl_reg	= DPLL_PWR_SW_CTRL,
		.sw_ctrl_dpll_bit	= SW_CTRL_PER_DPLL,
		.isoscan_bit		= ISOSCAN_PER,
		.ret_bit		= RET_PER,
		.reset_bit		= RESET_PER,
		.iso_bit		= ISO_PER,
		.pgoodin_bit		= PGOODIN_PER,
		.ponin_bit		= PONIN_PER,
		.dpll_pwr_sw_status_reg	= DPLL_PWR_SW_STATUS,
		.pgoodout_status_bit	= PGOODOUT_PER_STATUS,
		.ponout_status_bit	= PONOUT_PER_STATUS,
		.clkmode_reg		= AM43XX_CM_CLKMODE_DPLL_PER,
		.idlest_reg		= AM43XX_CM_IDLEST_DPLL_PER,
		.clksel_reg		= AM43XX_CM_CLKSEL_DPLL_PER,
	},
	[DPLL_DISP] = {
		.dpll_pwr_sw_ctrl_reg	= DPLL_PWR_SW_CTRL,
		.sw_ctrl_dpll_bit	= SW_CTRL_DISP_DPLL,
		.isoscan_bit		= ISOSCAN_DISP,
		.ret_bit		= RET_DISP,
		.reset_bit		= RESET_DISP,
		.iso_bit		= ISO_DISP,
		.pgoodin_bit		= PGOODIN_DISP,
		.ponin_bit		= PONIN_DISP,
		.dpll_pwr_sw_status_reg	= DPLL_PWR_SW_STATUS,
		.pgoodout_status_bit	= PGOODOUT_DISP_STATUS,
		.ponout_status_bit	= PONOUT_DISP_STATUS,
		.clkmode_reg		= AM43XX_CM_CLKMODE_DPLL_DISP,
		.idlest_reg		= AM43XX_CM_IDLEST_DPLL_DISP,
		.clksel_reg		= AM43XX_CM_CLKSEL_DPLL_DISP,
	},
	[DPLL_DDR] = {
		.dpll_pwr_sw_ctrl_reg	= DPLL_PWR_SW_CTRL,
		.sw_ctrl_dpll_bit	= SW_CTRL_DDR_DPLL,
		.isoscan_bit		= ISOSCAN_DDR,
		.ret_bit		= RET_DDR,
		.reset_bit		= RESET_DDR,
		.iso_bit		= ISO_DDR,
		.pgoodin_bit		= PGOODIN_DDR,
		.ponin_bit		= PONIN_DDR,
		.dpll_pwr_sw_status_reg	= DPLL_PWR_SW_STATUS,
		.pgoodout_status_bit	= PGOODOUT_DDR_STATUS,
		.ponout_status_bit	= PONOUT_DDR_STATUS,
		.clkmode_reg		= AM43XX_CM_CLKMODE_DPLL_DDR,
		.idlest_reg		= AM43XX_CM_IDLEST_DPLL_DDR,
		.clksel_reg		= AM43XX_CM_CLKSEL_DPLL_DDR,
	},
	[DPLL_MPU] = {
		.clkmode_reg		= AM43XX_CM_CLKMODE_DPLL_MPU,
		.idlest_reg		= AM43XX_CM_IDLEST_DPLL_MPU,
		.clksel_reg		= AM43XX_CM_CLKSEL_DPLL_MPU,
	},
	[DPLL_CORE] = {
		.clkmode_reg		= AM43XX_CM_CLKMODE_DPLL_CORE,
		.idlest_reg		= AM43XX_CM_IDLEST_DPLL_CORE,
		.clksel_reg		= AM43XX_CM_CLKSEL_DPLL_CORE,
	},
};

const enum dpll_id am43xx_power_down_plls[] = {
	DPLL_DDR,
	DPLL_DISP,
	DPLL_PER,
	DPLL_END,
};
