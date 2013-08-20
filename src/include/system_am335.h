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

#define BITBAND_SRAM_REF 	UMEM_ALIAS
#define BITBAND_SRAM_BASE 	0x22000000
#define BITBAND_SRAM(a,b) 	((BITBAND_SRAM_BASE + ((int)(a))*32 + (b*4)))

#define BITBAND_PERI_REF 	DMEM_ALIAS
#define BITBAND_PERI_BASE 	0x42000000
#define BITBAND_PERI(a,b) 	((BITBAND_PERI_BASE + (*(a) - BITBAND_PERI_REF)*32 + (b*4)))

#define BB_USB_WAKE		*((volatile int *)(BITBAND_SRAM(&cmd_wake_sources, 0)))
#define BB_I2C0_WAKE		*((volatile int *)(BITBAND_SRAM(&cmd_wake_sources, 1)))
#define BB_RTC_ALARM_WAKE	*((volatile int *)(BITBAND_SRAM(&cmd_wake_sources, 2)))
#define BB_TIMER1_WAKE		*((volatile int *)(BITBAND_SRAM(&cmd_wake_sources, 3)))
#define BB_UART0_WAKE		*((volatile int *)(BITBAND_SRAM(&cmd_wake_sources, 4)))
#define BB_GPIO0_WAKE0		*((volatile int *)(BITBAND_SRAM(&cmd_wake_sources, 5)))
#define BB_GPIO0_WAKE1		*((volatile int *)(BITBAND_SRAM(&cmd_wake_sources, 6)))
#define BB_WDT1_WAKE		*((volatile int *)(BITBAND_SRAM(&cmd_wake_sources, 7)))
#define BB_ADTSC_WAKE		*((volatile int *)(BITBAND_SRAM(&cmd_wake_sources, 8)))
/* Not used currently */
#define BB_RTC_TIMER_WAKE	*((volatile int *)(BITBAND_SRAM(&cmd_wake_sources, 9)))
#define BB_USBWOUT0		*((volatile int *)(BITBAND_SRAM(&cmd_wake_sources, 10)))
#define BB_MPU_WAKE		*((volatile int *)(BITBAND_SRAM(&cmd_wake_sources, 11)))
#define BB_USBWOUT1		*((volatile int *)(BITBAND_SRAM(&cmd_wake_sources, 12)))

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
