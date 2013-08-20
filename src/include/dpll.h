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

/* DPLL */
#define DPLL_PER                0
#define DPLL_DISP               1
#define DPLL_DDR                2
#define DPLL_MPU                3
#define DPLL_CORE               4

void am33xx_power_down_plls(void);
void am33xx_power_up_plls(void);

void pll_bypass(unsigned int dpll);
void pll_lock(unsigned int dpll);

#endif

