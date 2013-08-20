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
#include <hwmod.h>
#include <hwmod_335x.h>
#include <hwmod_43xx.h>

static const unsigned int *hwmods;
static const enum hwmod_id *essential_modules;
static const enum hwmod_id *interconnect_modules;

void hwmod_init(void)
{
	if (soc_id == AM335X_SOC_ID) {
		hwmods = am335x_hwmods;
		essential_modules = am335x_essential_modules;
		interconnect_modules = am335x_interconnect_modules;
	} else if (soc_id == AM43XX_SOC_ID) {
		hwmods = am43xx_hwmods;
		essential_modules = am43xx_essential_modules;
		interconnect_modules = am43xx_interconnect_modules;
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

/*
 * Looks like we'll have to ensure that we disable some modules when going down
 * ideally this list should have 0 entries but we need to check
 * what are the things that are really really necessary here
 */
int essential_modules_disable(void)
{
	/* Disable only the bare essential modules */
	modules_state_change(MODULE_DISABLE, essential_modules);

	return 0;
}

int essential_modules_enable(void)
{
	/* Enable only the bare essential modules */
	modules_state_change(MODULE_ENABLE, essential_modules);

	return 0;
}

int interconnect_modules_disable(void)
{
	modules_state_change(MODULE_DISABLE, interconnect_modules);

	return 0;
}

int interconnect_modules_enable(void)
{
	modules_state_change(MODULE_ENABLE, interconnect_modules);

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
