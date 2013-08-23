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

#include <low_power.h>
#include <device_am335x.h>
#include <system_am335.h>
#include <prcm.h>
#include <hwmod.h>

static const unsigned int am335x_hwmods[HWMOD_COUNT] = {
	[HWMOD_CLKDIV32K]	= AM335X_CM_PER_CLKDIV32K_CLKCTRL,
	[HWMOD_EMIF]		= AM335X_CM_PER_EMIF_CLKCTRL,
	[HWMOD_EMIF_FW]		= AM335X_CM_PER_EMIF_FW_CLKCTRL,
	[HWMOD_GPIO0]		= AM335X_CM_WKUP_GPIO0_CLKCTRL,
	[HWMOD_I2C0]		= AM335X_CM_WKUP_I2C0_CLKCTRL,
	[HWMOD_IEEE5000]	= AM335X_CM_PER_IEEE5000_CLKCTRL,
	[HWMOD_L3]		= AM335X_CM_PER_L3_CLKCTRL,
	[HWMOD_L3_INSTR]	= AM335X_CM_PER_L3_INSTR_CLKCTRL,
	[HWMOD_L4FW]		= AM335X_CM_PER_L4FW_CLKCTRL,
	[HWMOD_L4HS]		= AM335X_CM_PER_L4HS_CLKCTRL,
	[HWMOD_L4LS]		= AM335X_CM_PER_L4LS_CLKCTRL,
	[HWMOD_MPU]		= AM335X_CM_MPU_MPU_CLKCTRL,
	[HWMOD_OCMCRAM]		= AM335X_CM_PER_OCMCRAM_CLKCTRL,
};

static const unsigned int am43xx_hwmods[HWMOD_COUNT] = {
	[HWMOD_CLKDIV32K]	= AM43XX_CM_WKUP_CLKDIV32K_CLKCTRL,
	[HWMOD_EMIF]		= AM43XX_CM_PER_EMIF_CLKCTRL,
	[HWMOD_EMIF_FW]		= AM43XX_CM_PER_EMIF_FW_CLKCTRL,
	[HWMOD_GPIO0]		= AM43XX_CM_WKUP_GPIO0_CLKCTRL,
	[HWMOD_I2C0]		= AM43XX_CM_WKUP_I2C0_CLKCTRL,
	[HWMOD_IEEE5000]	= AM43XX_CM_PER_IEEE5000_CLKCTRL,
	[HWMOD_L3]		= AM43XX_CM_PER_L3_CLKCTRL,
	[HWMOD_L3_INSTR]	= AM43XX_CM_PER_L3_INSTR_CLKCTRL,
	[HWMOD_L4FW]		= AM43XX_CM_PER_L4FW_CLKCTRL,
	[HWMOD_L4HS]		= AM43XX_CM_PER_L4HS_CLKCTRL,
	[HWMOD_L4LS]		= AM43XX_CM_PER_L4LS_CLKCTRL,
	[HWMOD_MPU]		= AM43XX_CM_MPU_MPU_CLKCTRL,
	[HWMOD_OCMCRAM]		= AM43XX_CM_PER_OCMCRAM_CLKCTRL,
	[HWMOD_OTFA_EMIF]	= AM43XX_CM_PER_OTFA_EMIF_CLKCTRL,
};

static const unsigned int *hwmods;

void hwmod_init(void)
{
	if (soc_id == AM335X_SOC_ID) {
		hwmods = am335x_hwmods;
	} else if (soc_id == AM43XX_SOC_ID) {
		hwmods = am43xx_hwmods;
	}
}

#define DEFAULT_IDLEST_SHIFT		16
#define DEFAULT_IDLEST_MASK		(3 << DEFAULT_IDLEST_SHIFT)
#define DEFAULT_IDLEST_IDLE_VAL		3
#define DEFAULT_IDLEST_ACTIVE_VAL 	0

/* TODO: Add a timeout and bail out */
static void _module_enable(int reg)
{
	__raw_writel(MODULE_ENABLE, reg);

	while ((__raw_readl(reg) & DEFAULT_IDLEST_MASK)>>DEFAULT_IDLEST_SHIFT !=
		DEFAULT_IDLEST_ACTIVE_VAL);
}

static void _module_disable(int reg)
{
	__raw_writel(MODULE_DISABLE, reg);

	while ((__raw_readl(reg) & DEFAULT_IDLEST_MASK)>>DEFAULT_IDLEST_SHIFT !=
		DEFAULT_IDLEST_IDLE_VAL);
}

int module_state_change(int state, enum hwmod_id id)
{
	if (state == MODULE_DISABLE)
		_module_disable(hwmods[id]);
	else
		_module_enable(hwmods[id]);

	return 0;
}

static int modules_state_change(int state, const enum hwmod_id *ids)
{
	int i;

	for (i = 0; ids[i] != HWMOD_END; i++)
		if (state == MODULE_DISABLE)
			_module_disable(hwmods[ids[i]]);

	for (i--; i >= 0; i--)
		if (state == MODULE_ENABLE)
			_module_enable(hwmods[ids[i]]);

	return 0;
}

static const enum hwmod_id am335x_essential_modules[] = {
	HWMOD_CLKDIV32K,
	HWMOD_IEEE5000,
	HWMOD_EMIF_FW,
	HWMOD_OCMCRAM,
	HWMOD_END,
};

static const enum hwmod_id am43xx_essential_modules[] = {
	HWMOD_EMIF_FW,
	HWMOD_OTFA_EMIF,
	HWMOD_OCMCRAM,
	HWMOD_END,
};

/*
 * Looks like we'll have to ensure that we disable some modules when going down
 * ideally this list should have 0 entries but we need to check
 * what are the things that are really really necessary here
 */
int essential_modules_disable(void)
{
	/* Disable only the bare essential modules */
	if (soc_id == AM335X_SOC_ID)
		modules_state_change(MODULE_DISABLE, am335x_essential_modules);
	else if (soc_id == AM43XX_SOC_ID)
		modules_state_change(MODULE_DISABLE, am43xx_essential_modules);

	return 0;
}

int essential_modules_enable(void)
{
	/* Enable only the bare essential modules */
	if (soc_id == AM335X_SOC_ID)
		modules_state_change(MODULE_ENABLE, am335x_essential_modules);
	else if (soc_id == AM43XX_SOC_ID)
		modules_state_change(MODULE_ENABLE, am43xx_essential_modules);

	return 0;
}

static const enum hwmod_id am335x_interconnect_modules[] = {
	HWMOD_L4LS,
	HWMOD_L4HS,
	HWMOD_L4FW,
	HWMOD_L3_INSTR,
	HWMOD_L3,
	HWMOD_END,
};

static const enum hwmod_id am43xx_interconnect_modules[] = {
	HWMOD_L4LS,
	HWMOD_L4HS,
	HWMOD_L4FW,
	HWMOD_L3_INSTR,
	HWMOD_L3,
	HWMOD_END,
};

int interconnect_modules_disable(void)
{
	if (soc_id == AM335X_SOC_ID)
		modules_state_change(MODULE_DISABLE, am335x_interconnect_modules);
	else if (soc_id == AM43XX_SOC_ID)
		modules_state_change(MODULE_DISABLE, am43xx_interconnect_modules);

	return 0;
}

int interconnect_modules_enable(void)
{
	if (soc_id == AM335X_SOC_ID)
		modules_state_change(MODULE_ENABLE, am335x_interconnect_modules);
	else if (soc_id == AM43XX_SOC_ID)
		modules_state_change(MODULE_ENABLE, am43xx_interconnect_modules);

	return 0;
}

void mpu_disable(void)
{
	module_state_change(MODULE_DISABLE, HWMOD_MPU);
}

void mpu_enable(void)
{
	module_state_change(MODULE_ENABLE, HWMOD_MPU);
}
