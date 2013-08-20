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

#include <stdint.h>
#include <stddef.h>

#include <device_am335x.h>
#include <prcm.h>
#include <system_am335.h>
#include <dpll.h>

struct dpll_pd_data {
	int dpll_reg;
	int sw_ctrl_dpll_bit;
	int isoscan_bit;
	int ret_bit;
	int reset_bit;
	int iso_bit;
	int pgoodin_bit;
	int ponin_bit;
	int dpll_pwr_sw_status_reg;
	int pgoodout_status_bit;
	int ponout_status_bit;
};

static const struct dpll_pd_data dpll_data[]  = {
	{
		.dpll_reg		= DPLL_PWR_SW_CTRL,
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
	},
	{
		.dpll_reg		= DPLL_PWR_SW_CTRL,
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
	},
	{
		.dpll_reg		= DPLL_PWR_SW_CTRL,
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
	},
};

/* DPLL power-down Sequence PG 2.x */
static void dpll_power_down(unsigned int dpll)
{
	unsigned int var;

	/* Configure bit to select Control module selection for DPLL */
	var = __raw_readl(dpll_data[dpll].dpll_reg);
	var |= dpll_data[dpll].sw_ctrl_dpll_bit;
	__raw_writel(var, dpll_data[dpll].dpll_reg);

	/* Assert ISO bit high */
	var = __raw_readl(dpll_data[dpll].dpll_reg);
	var |= dpll_data[dpll].iso_bit;
	__raw_writel(var, dpll_data[dpll].dpll_reg);

	/* ISO_SCAN, RET should be asserted high */
	var = __raw_readl(dpll_data[dpll].dpll_reg);
	var |= (dpll_data[dpll].isoscan_bit | dpll_data[dpll].ret_bit);
	__raw_writel(var, dpll_data[dpll].dpll_reg);

	/* Assert DPLL reset to 1 */
	var = __raw_readl(dpll_data[dpll].dpll_reg);
	var |= dpll_data[dpll].reset_bit;
	__raw_writel(var, dpll_data[dpll].dpll_reg);

	/* PGOODIN signal is de-asserted low */
	var = __raw_readl(dpll_data[dpll].dpll_reg);
	var &= ~dpll_data[dpll].pgoodin_bit;
	__raw_writel(var, dpll_data[dpll].dpll_reg);

	/* PONIN signal is de-asserted low */
	var = __raw_readl(dpll_data[dpll].dpll_reg);
	var &= ~dpll_data[dpll].ponin_bit;
	__raw_writel(var, dpll_data[dpll].dpll_reg);

	/* Poll for PONOUT and PGOODOUT signal status as 0 */
	while (__raw_readl(dpll_data[dpll].dpll_pwr_sw_status_reg) &
			(dpll_data[dpll].pgoodout_status_bit |
			dpll_data[dpll].ponout_status_bit));
}

/* DPLL Power-up Sequence */
static void dpll_power_up(unsigned int dpll)
{
	unsigned int var;

	/* PONIN is asserted high */
	var = __raw_readl(dpll_data[dpll].dpll_reg);
	var |= dpll_data[dpll].ponin_bit;
	__raw_writel(var, dpll_data[dpll].dpll_reg);

	/* Poll for PONOUT to become high  */
	while (!(__raw_readl(dpll_data[dpll].dpll_pwr_sw_status_reg) &
				dpll_data[dpll].ponout_status_bit));

	/* PGOODIN is asserted high */
	var = __raw_readl(dpll_data[dpll].dpll_reg);
	var |= dpll_data[dpll].pgoodin_bit;
	__raw_writel(var, dpll_data[dpll].dpll_reg);

	/* Poll for PGOODOUT to become high */
	while (!(__raw_readl(dpll_data[dpll].dpll_pwr_sw_status_reg) &
				dpll_data[dpll].pgoodout_status_bit));

	/* De-assert DPLL RESET to 0 */
	var = __raw_readl(dpll_data[dpll].dpll_reg);
	var &= ~dpll_data[dpll].reset_bit;
	__raw_writel(var, dpll_data[dpll].dpll_reg);

	/* ISO_SCAN, RET should be de-asserted low */
	var = __raw_readl(dpll_data[dpll].dpll_reg);
	var &= ~(dpll_data[dpll].isoscan_bit | dpll_data[dpll].ret_bit);
	__raw_writel(var, dpll_data[dpll].dpll_reg);

	/* De-assert ISO signal */
	var = __raw_readl(dpll_data[dpll].dpll_reg);
	var &= ~dpll_data[dpll].iso_bit;
	__raw_writel(var, dpll_data[dpll].dpll_reg);

	/* Re-Configure bit to select PRCM selection for DPLL */
	var = __raw_readl(dpll_data[dpll].dpll_reg);
	var &= ~dpll_data[dpll].sw_ctrl_dpll_bit;
	__raw_writel(var, dpll_data[dpll].dpll_reg);
}

/* DPLL retention update for PG 2.0 */
void am33xx_power_down_plls(void)
{
	if (soc_id == AM335X_SOC_ID && soc_rev > AM335X_REV_ES1_0) {
		dpll_power_down(DPLL_DDR);
		dpll_power_down(DPLL_DISP);
		dpll_power_down(DPLL_PER);
	} else if (soc_id == AM43XX_SOC_ID) {
		/* TODO */
	}
}

/* DPLL retention update for PG 2.x */
void am33xx_power_up_plls(void)
{
	if (soc_id == AM335X_SOC_ID && soc_rev > AM335X_REV_ES1_0) {
		dpll_power_up(DPLL_DDR);
		dpll_power_up(DPLL_DISP);
		dpll_power_up(DPLL_PER);
	} else if (soc_id == AM43XX_SOC_ID) {
		/* TODO */
	}
}

struct dpll_context {
	int clk_mode_addr;
	int idlest_addr;
	int pll_mode;
};

struct dpll_context am33xx_dplls[] = {
	{
		.clk_mode_addr	= AM335X_CM_CLKMODE_DPLL_PER,
		.idlest_addr	= AM335X_CM_IDLEST_DPLL_PER,
		.pll_mode	= 0,
	},
	{
		.clk_mode_addr	= AM335X_CM_CLKMODE_DPLL_DISP,
		.idlest_addr	= AM335X_CM_IDLEST_DPLL_DISP,
		.pll_mode	= 0,
	},
	{
		.clk_mode_addr	= AM335X_CM_CLKMODE_DPLL_DDR,
		.idlest_addr	= AM335X_CM_IDLEST_DPLL_DDR,
		.pll_mode	= 0,
	},
	{
		.clk_mode_addr	= AM335X_CM_CLKMODE_DPLL_MPU,
		.idlest_addr	= AM335X_CM_IDLEST_DPLL_MPU,
		.pll_mode	= 0,
	},
	{
		.clk_mode_addr	= AM335X_CM_CLKMODE_DPLL_CORE,
		.idlest_addr	= AM335X_CM_IDLEST_DPLL_CORE,
		.pll_mode	= 0,
	},
};

void pll_bypass(unsigned int dpll)
{
	am33xx_dplls[dpll].pll_mode = __raw_readl(am33xx_dplls[dpll].clk_mode_addr);
	__raw_writel(((am33xx_dplls[dpll].pll_mode & ~DPLL_EN_MASK) |
			DPLL_LP_BYP_MODE), am33xx_dplls[dpll].clk_mode_addr);

	/* Wait for DPLL to enter bypass mode */
	while (__raw_readl(am33xx_dplls[dpll].idlest_addr));
}

void pll_lock(unsigned int dpll)
{
	__raw_writel(am33xx_dplls[dpll].pll_mode, am33xx_dplls[dpll].clk_mode_addr);

	if ((am33xx_dplls[dpll].pll_mode & 0x7) == 0x7)
		/* Make sure DPLL Clock is out of Bypass */
		while (!(__raw_readl(am33xx_dplls[dpll].idlest_addr)));
}
