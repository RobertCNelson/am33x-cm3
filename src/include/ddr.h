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

#ifndef __DDR_H__
#define __DDR_H__

void ddr_io_suspend(void);
void ddr_io_resume(void);

void vtp_disable(void);
void vtp_enable(void);

void set_ddr_reset(void);
void clear_ddr_reset(void);

void vtt_low(void);
void vtt_high(void);

#endif
