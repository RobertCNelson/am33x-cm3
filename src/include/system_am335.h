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

int cmd_wake_sources;

unsigned int soc_id;
unsigned int soc_rev;
unsigned int soc_type;

void pm_reset(void);

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

void setup_soc(void);

int a8_i2c_sleep_handler(unsigned short);
int a8_i2c_wake_handler(unsigned short);

#define AM335X_SOC_ID		0xB944
#define AM335X_REV_ES1_0	0
#define AM335X_REV_ES2_0	1
#define AM335X_REV_ES2_1	2

#define AM43XX_SOC_ID		0xB98C

#define SOC_TYPE_TEST	0
#define SOC_TYPE_EMU	1
#define SOC_TYPE_HS	2
#define SOC_TYPE_GP	3

#endif
