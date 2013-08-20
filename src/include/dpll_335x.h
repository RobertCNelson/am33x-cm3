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

#ifndef __DPLL_335X_H__
#define __DPLL_335X_H__

enum dpll_id;
struct dpll_regs;

extern const struct dpll_regs am335x_dpll_regs[];
extern const enum dpll_id am335x_pg2_power_down_plls[];

#endif

