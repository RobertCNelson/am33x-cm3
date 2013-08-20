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

#ifndef __PM_HANDLERS_H__
#define __PM_HANDLERS_H__

struct cmd_data;

void a8_lp_rtc_handler(struct cmd_data *);
void a8_lp_ds0_handler(struct cmd_data *);
void a8_lp_ds1_handler(struct cmd_data *);
void a8_lp_ds2_handler(struct cmd_data *);
void a8_standalone_handler(struct cmd_data *);
void a8_standby_handler(struct cmd_data *);
void a8_cpuidle_handler(struct cmd_data *);

void generic_wake_handler(int);
void a8_wake_rtc_handler(void);
void a8_wake_ds0_handler(void);
void a8_wake_ds1_handler(void);
void a8_wake_ds2_handler(void);
void a8_wake_standby_handler(void);
void a8_wake_cpuidle_handler(void);

#endif
