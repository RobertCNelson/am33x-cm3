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

#ifndef __HWMOD_335X_H__
#define __HWMOD_335X_H__

enum hwmod_id;

extern const unsigned int am335x_hwmods[];
extern const enum hwmod_id am335x_essential_modules[];
extern const enum hwmod_id am335x_interconnect_modules[];

#endif

