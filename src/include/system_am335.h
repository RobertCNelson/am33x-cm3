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

#ifndef __SYSTEM_AM335_H__
#define __SYSTEM_AM335_H__

#include <stdint.h>
#include <stddef.h>

struct cmd_data;

/* Debug info */
bool halt_on_resume;

void system_init(void);
void system_core_clock_update(void);

void a8_notify(int);
void a8_m3_low_power_sync(int);
void a8_m3_low_power_fast(int);

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

void init_m3_state_machine(void);

void trace_init(void);
void trace_update(void);
void trace_get_current_pos(void);
void trace_set_current_pos(void);

int rtc_enable_check(void);
unsigned int rtc_reg_read(int);
void rtc_reg_write(unsigned int, int);

int i2c_write(const unsigned char *);

#endif
