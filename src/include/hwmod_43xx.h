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

#ifndef __HWMOD_43XX_H__
#define __HWMOD_43XX_H__

enum hwmod_id;

extern const unsigned int am43xx_hwmods[];
extern const enum hwmod_id am43xx_essential_hwmods[];
extern const enum hwmod_id am43xx_interconnect_hwmods[];

#endif

