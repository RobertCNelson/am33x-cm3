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
#include <hwmod.h>
#include <hwmod_335x.h>
#include <hwmod_43xx.h>

#define HWMOD_DISABLE	0x0
#define HWMOD_ENABLE	0x2

static const unsigned int *hwmods;
static const enum hwmod_id *essential_hwmods;
static const enum hwmod_id *interconnect_hwmods;

void hwmod_init(void)
{
	if (soc_id == AM335X_SOC_ID) {
		hwmods = am335x_hwmods;
		essential_hwmods = am335x_essential_hwmods;
		interconnect_hwmods = am335x_interconnect_hwmods;
	} else if (soc_id == AM43XX_SOC_ID) {
		hwmods = am43xx_hwmods;
		essential_hwmods = am43xx_essential_hwmods;
		interconnect_hwmods = am43xx_interconnect_hwmods;
	}
}

#define DEFAULT_IDLEST_SHIFT		16
#define DEFAULT_IDLEST_MASK		(3 << DEFAULT_IDLEST_SHIFT)
#define DEFAULT_IDLEST_IDLE_VAL		3
#define DEFAULT_IDLEST_ACTIVE_VAL 	0

/* TODO: Add a timeout and bail out */
static void _hwmod_enable(int reg)
{
	__raw_writel(HWMOD_ENABLE, reg);

	while ((__raw_readl(reg) & DEFAULT_IDLEST_MASK)>>DEFAULT_IDLEST_SHIFT !=
		DEFAULT_IDLEST_ACTIVE_VAL);
}

static void _hwmod_disable(int reg)
{
	__raw_writel(HWMOD_DISABLE, reg);

	while ((__raw_readl(reg) & DEFAULT_IDLEST_MASK)>>DEFAULT_IDLEST_SHIFT !=
		DEFAULT_IDLEST_IDLE_VAL);
}

static int hwmods_state_change(int state, const enum hwmod_id *ids)
{
	int i;

	for (i = 0; ids[i] != HWMOD_END; i++)
		if (state == HWMOD_DISABLE)
			_hwmod_disable(hwmods[ids[i]]);

	for (i--; i >= 0; i--)
		if (state == HWMOD_ENABLE)
			_hwmod_enable(hwmods[ids[i]]);

	return 0;
}

void hwmod_enable(enum hwmod_id id)
{
	_hwmod_enable(hwmods[id]);
}

void hwmod_disable(enum hwmod_id id)
{
	_hwmod_disable(hwmods[id]);
}

/*
 * Looks like we'll have to ensure that we disable some hwmods when going down
 * ideally this list should have 0 entries but we need to check
 * what are the things that are really really necessary here
 */
int essential_hwmods_disable(void)
{
	/* Disable only the bare essential hwmods */
	hwmods_state_change(HWMOD_DISABLE, essential_hwmods);

	return 0;
}

int essential_hwmods_enable(void)
{
	/* Enable only the bare essential hwmods */
	hwmods_state_change(HWMOD_ENABLE, essential_hwmods);

	return 0;
}

int interconnect_hwmods_disable(void)
{
	hwmods_state_change(HWMOD_DISABLE, interconnect_hwmods);

	return 0;
}

int interconnect_hwmods_enable(void)
{
	hwmods_state_change(HWMOD_ENABLE, interconnect_hwmods);

	return 0;
}

