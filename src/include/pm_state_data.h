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

#ifndef __PM_STATE_DATA_H__
#define __PM_STATE_DATA_H__

union state_data;

extern union state_data rtc_mode_data;
extern union state_data standby_data;
extern union state_data ds0_data;
extern union state_data ds0_data_hs;
extern union state_data ds1_data;
extern union state_data ds1_data_hs;
extern union state_data ds2_data;
extern union state_data idle_data;
extern union state_data idle_v2_data;

#endif
