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

#include <device_am335x.h>
#include <io.h>
#include <clockdomain.h>
#include <rtc.h>

int rtc_enable_check(void)
{
	if (clkdm_active(CLKDM_RTC))
		return 0;
	else
		while(1)
		;
}

void rtc_reg_write(unsigned int val, int reg)
{
	__raw_writel(val, reg + RTCSS_BASE);
}

unsigned int rtc_reg_read(int reg)
{
	return __raw_readl(reg + RTCSS_BASE);
}
