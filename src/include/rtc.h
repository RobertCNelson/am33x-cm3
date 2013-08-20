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

#ifndef __RTC_H__
#define __RTC_H__

#define RTC_SECONDS_REG		0x00
#define RTC_MINUTES_REG		0x04
#define RTC_HOURS_REG		0x08
#define RTC_DAYS_REG		0x0C
#define RTC_MONTHS_REG		0x10
#define RTC_YEARS_REG		0x14
#define RTC_WEEKS_REG		0x18

#define RTC_ALARM_SECONDS_REG	0x20
#define RTC_ALARM_MINUTES_REG	0x24
#define RTC_ALARM_HOURS_REG	0x28
#define RTC_ALARM_DAYS_REG	0x2c
#define RTC_ALARM_MONTHS_REG	0x30
#define RTC_ALARM_YEARS_REG	0x34

#define RTC_CTRL_REG		0x40
#define RTC_STATUS_REG		0x44
#define RTC_INTERRUPTS_REG	0x48
#define RTC_COMP_LSB_REG	0x4c
#define RTC_COMP_MSB_REG	0x50
#define RTC_OSC_REG		0x54

#define RTC_KICK0		0x6c
#define RTC_KICK1		0x70
#define RTC_SYSCONFIG		0x78
#define RTC_IRQWAKEEN_0		0x7c

#define RTC_ALARM2_SECONDS_REG	0x80
#define RTC_ALARM2_MINUTES_REG	0x84
#define RTC_ALARM2_HOURS_REG	0x88
#define RTC_ALARM2_DAYS_REG	0x8c
#define RTC_ALARM2_MONTHS_REG	0x90
#define RTC_ALARM2_YEARS_REG	0x94

#define RTC_PMIC_REG		0x98
#define RTC_DEBOUNCE_REG	0x9c

/* RTC_CTRL_REG bit fields: */
#define RTC_CTRL_DISABLE	(1<<6)
#define RTC_CTRL_STOP		(1<<0)

/* RTC_STATUS_REG bit fields: */
#define RTC_STATUS_POWER_UP        (1<<7)
#define RTC_STATUS_ALARM           (1<<6)
#define RTC_STATUS_1D_EVENT        (1<<5)
#define RTC_STATUS_1H_EVENT        (1<<4)
#define RTC_STATUS_1M_EVENT        (1<<3)
#define RTC_STATUS_1S_EVENT        (1<<2)
#define RTC_STATUS_RUN             (1<<1)
#define RTC_STATUS_BUSY            (1<<0)

/* RTC_INTERRUPTS_REG bit fields: */
#define RTC_INTERRUPTS_IT_ALARM    (1<<3)
#define RTC_INTERRUPTS_IT_TIMER    (1<<2)

int rtc_enable_check(void);
unsigned int rtc_reg_read(int);
void rtc_reg_write(unsigned int, int);

#endif
