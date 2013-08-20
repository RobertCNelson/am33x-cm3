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

#ifndef __DPLL_H__
#define __DPLL_H__

enum dpll_id {
	DPLL_PER,
	DPLL_DISP,
	DPLL_DDR,
	DPLL_MPU,
	DPLL_CORE,

	DPLL_COUNT,
	DPLL_END = -1,
};

struct dpll_regs {
	unsigned int dpll_pwr_sw_ctrl_reg;
	unsigned int sw_ctrl_dpll_bit;
	unsigned int isoscan_bit;
	unsigned int ret_bit;
	unsigned int reset_bit;
	unsigned int iso_bit;
	unsigned int pgoodin_bit;
	unsigned int ponin_bit;

	unsigned int dpll_pwr_sw_status_reg;
	unsigned int pgoodout_status_bit;
	unsigned int ponout_status_bit;

	unsigned int clkmode_reg;
	unsigned int idlest_reg;
};

void plls_power_down(void);
void plls_power_up(void);

void pll_bypass(enum dpll_id dpll);
void pll_lock(enum dpll_id dpll);

void dpll_reset(void);
void dpll_init(void);

#endif

