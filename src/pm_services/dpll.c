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
#include <prcm_core.h>
#include <dpll.h>
#include <dpll_335x.h>
#include <dpll_43xx.h>

/* DPLL CLOCKMODE register */
#define DPLL_EN_MASK					(0x7 << 0)
#define DPLL_LP_BYP_MODE				(0x5 << 0)
#define DPLL_LOCK_MODE					(0x7 << 0)
#define DPLL_DIV_PER_SHIFT				(0)
#define DPLL_DIV_PER_MASK				(0xff)

static const struct dpll_regs *dpll_regs;
static const enum dpll_id *power_down_plls;

static unsigned int pll_mode[DPLL_COUNT];

/* DPLL power-down Sequence PG 2.x */
static void dpll_power_down(enum dpll_id dpll)
{
	unsigned int var;

	/* Configure bit to select Control module selection for DPLL */
	var = __raw_readl(dpll_regs[dpll].dpll_pwr_sw_ctrl_reg);
	var |= dpll_regs[dpll].sw_ctrl_dpll_bit;
	__raw_writel(var, dpll_regs[dpll].dpll_pwr_sw_ctrl_reg);

	/* Assert ISO bit high */
	var = __raw_readl(dpll_regs[dpll].dpll_pwr_sw_ctrl_reg);
	var |= dpll_regs[dpll].iso_bit;
	__raw_writel(var, dpll_regs[dpll].dpll_pwr_sw_ctrl_reg);

	/* ISO_SCAN, RET should be asserted high */
	var = __raw_readl(dpll_regs[dpll].dpll_pwr_sw_ctrl_reg);
	var |= (dpll_regs[dpll].isoscan_bit | dpll_regs[dpll].ret_bit);
	__raw_writel(var, dpll_regs[dpll].dpll_pwr_sw_ctrl_reg);

	/* Assert DPLL reset to 1 */
	var = __raw_readl(dpll_regs[dpll].dpll_pwr_sw_ctrl_reg);
	var |= dpll_regs[dpll].reset_bit;
	__raw_writel(var, dpll_regs[dpll].dpll_pwr_sw_ctrl_reg);

	/* PGOODIN signal is de-asserted low */
	var = __raw_readl(dpll_regs[dpll].dpll_pwr_sw_ctrl_reg);
	var &= ~dpll_regs[dpll].pgoodin_bit;
	__raw_writel(var, dpll_regs[dpll].dpll_pwr_sw_ctrl_reg);

	/* PONIN signal is de-asserted low */
	var = __raw_readl(dpll_regs[dpll].dpll_pwr_sw_ctrl_reg);
	var &= ~dpll_regs[dpll].ponin_bit;
	__raw_writel(var, dpll_regs[dpll].dpll_pwr_sw_ctrl_reg);

	/* Poll for PONOUT and PGOODOUT signal status as 0 */
	while (__raw_readl(dpll_regs[dpll].dpll_pwr_sw_status_reg) &
			(dpll_regs[dpll].pgoodout_status_bit |
			dpll_regs[dpll].ponout_status_bit));
}

/* DPLL Power-up Sequence */
static void dpll_power_up(enum dpll_id dpll)
{
	unsigned int var;

	/* PONIN is asserted high */
	var = __raw_readl(dpll_regs[dpll].dpll_pwr_sw_ctrl_reg);
	var |= dpll_regs[dpll].ponin_bit;
	__raw_writel(var, dpll_regs[dpll].dpll_pwr_sw_ctrl_reg);

	/* Poll for PONOUT to become high  */
	while (!(__raw_readl(dpll_regs[dpll].dpll_pwr_sw_status_reg) &
				dpll_regs[dpll].ponout_status_bit));

	/* PGOODIN is asserted high */
	var = __raw_readl(dpll_regs[dpll].dpll_pwr_sw_ctrl_reg);
	var |= dpll_regs[dpll].pgoodin_bit;
	__raw_writel(var, dpll_regs[dpll].dpll_pwr_sw_ctrl_reg);

	/* Poll for PGOODOUT to become high */
	while (!(__raw_readl(dpll_regs[dpll].dpll_pwr_sw_status_reg) &
				dpll_regs[dpll].pgoodout_status_bit));

	/* De-assert DPLL RESET to 0 */
	var = __raw_readl(dpll_regs[dpll].dpll_pwr_sw_ctrl_reg);
	var &= ~dpll_regs[dpll].reset_bit;
	__raw_writel(var, dpll_regs[dpll].dpll_pwr_sw_ctrl_reg);

	/* ISO_SCAN, RET should be de-asserted low */
	var = __raw_readl(dpll_regs[dpll].dpll_pwr_sw_ctrl_reg);
	var &= ~(dpll_regs[dpll].isoscan_bit | dpll_regs[dpll].ret_bit);
	__raw_writel(var, dpll_regs[dpll].dpll_pwr_sw_ctrl_reg);

	/* De-assert ISO signal */
	var = __raw_readl(dpll_regs[dpll].dpll_pwr_sw_ctrl_reg);
	var &= ~dpll_regs[dpll].iso_bit;
	__raw_writel(var, dpll_regs[dpll].dpll_pwr_sw_ctrl_reg);

	/* Re-Configure bit to select PRCM selection for DPLL */
	var = __raw_readl(dpll_regs[dpll].dpll_pwr_sw_ctrl_reg);
	var &= ~dpll_regs[dpll].sw_ctrl_dpll_bit;
	__raw_writel(var, dpll_regs[dpll].dpll_pwr_sw_ctrl_reg);
}

/* DPLL retention update for PG 2.0 */
void plls_power_down(void)
{
	int i;

	for (i = 0; power_down_plls && power_down_plls[i] != DPLL_END; i++)
		dpll_power_down(power_down_plls[i]);
}

/* DPLL retention update for PG 2.x */
void plls_power_up(void)
{
	int i;

	for (i = 0; power_down_plls && power_down_plls[i] != DPLL_END; i++)
		dpll_power_up(power_down_plls[i]);
}

void pll_bypass(enum dpll_id dpll)
{
	pll_mode[dpll] = __raw_readl(dpll_regs[dpll].clkmode_reg);
	__raw_writel(((pll_mode[dpll] & ~DPLL_EN_MASK) |
			DPLL_LP_BYP_MODE), dpll_regs[dpll].clkmode_reg);

	/* Wait for DPLL to enter bypass mode */
	while (__raw_readl(dpll_regs[dpll].idlest_reg));
}

void pll_lock(enum dpll_id dpll)
{
	__raw_writel(pll_mode[dpll], dpll_regs[dpll].clkmode_reg);

	if ((pll_mode[dpll] & 0x7) == 0x7)
		/* Make sure DPLL Clock is out of Bypass */
		while (!(__raw_readl(dpll_regs[dpll].idlest_reg)));
}

unsigned int dpll_get_div(enum dpll_id dpll)
{
	unsigned int val;

	val = __raw_readl(dpll_regs[dpll].clksel_reg);
	val &= DPLL_DIV_PER_MASK;
	val >>= DPLL_DIV_PER_SHIFT;

	return val;
}

void dpll_reset(void)
{
	int i = 0;

	for (i = 0; i < DPLL_COUNT; i++)
		pll_mode[i] = 0;
}

void dpll_init(void)
{
	if (soc_id == AM335X_SOC_ID) {
		dpll_regs = am335x_dpll_regs;
		if (soc_rev > AM335X_REV_ES1_0)
			power_down_plls = am335x_pg2_power_down_plls;

	} else if (soc_id == AM43XX_SOC_ID) {
		dpll_regs = am43xx_dpll_regs;
		power_down_plls = am43xx_power_down_plls;
	}
}
