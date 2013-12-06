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

#ifndef __DPLL_43XX_H__
#define __DPLL_43XX_H__

struct dpll_regs;

extern const struct dpll_regs am43xx_dpll_regs[];
extern const enum dpll_id am43xx_power_down_plls[];

#endif

